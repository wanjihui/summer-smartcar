//菜单具体构建+摄像头

#include "config.h"


typedef enum {
    DISPLAY_MODE_MENU,     // 菜单模式（默认）
    DISPLAY_MODE_BIN,      // 二值化调参
    DISPLAY_MODE_TRACK,    // 搜线+显示
} display_mode_enum;

static display_mode_enum display_mode = DISPLAY_MODE_MENU;  // 当前显示模式 默认菜单模式
static bool k3_wait_release = false;                         // K3长按防抖（key_clear_state不清press_time，需等物理松开）

static Menu_Item *key;                   // 当前选中节点的指针
static Menu_Item head;                   // 菜单的根节点

static int32_t test = 10;
static float test1 = 3.14;
static bool test2 = true;

//菜单初始化和构建
void menu_init(void)
{
    head.data = NULL;
    head.father = NULL;
    head.first_son = NULL;
    head.last_brother = NULL;
    head.next_brother = NULL;
    head.kind = MENU_Folder;
    head.name = "MENU";
    head.No = 0;
    head.sons = 0;
    head.select = false;
    head.isLimit = false;
    head.limit_min = 0.0f;
    head.limit_max = 0.0f;

    //构建具体菜单

    // =====folder1测试文件夹=====
    {
    Menu_Item *folder1 = dynamicCreate_Menu_Folder(&head, "folder1");
    Menu_Item *num1=dynamicCreate_Menu_Number(folder1, "aaa", &test, int32_Box);
    Menu_Set_Limit(num1, 0, 100,1.0f);

    Menu_Item *num2 = dynamicCreate_Menu_Number(folder1, "bbb", &test1, float_Box);
    Menu_Set_Limit(num2, -5.0f, 5.0f,1.0f);

    dynamicCreate_Menu_Number(folder1, "ccc", &test2, bool_Box);
    }

    // =====PID文件夹=====
    {
        Menu_Item *pid = dynamicCreate_Menu_Folder(&head, "PID");
        Menu_Item *v;

        // 舵机
        v = dynamicCreate_Menu_Number(pid, "servo_kp",  &servo_kp,  float_Box);
        Menu_Set_Limit(v, 0, 5,0.1f);
        v = dynamicCreate_Menu_Number(pid, "servo_kd",  &servo_kd,  float_Box);
        Menu_Set_Limit(v, 0, 2,0.01f);
        v = dynamicCreate_Menu_Number(pid, "center",    &servo_center, float_Box);
        Menu_Set_Limit(v, 75, 105,0.5);
        v = dynamicCreate_Menu_Number(pid, "max_angle", &servo_max_cha, float_Box);
        Menu_Set_Limit(v, 1, 45,0.5);
        v = dynamicCreate_Menu_Number(pid, "deadband",  &servo_dead, float_Box);
        Menu_Set_Limit(v, 0, 10,0.5);
        v = dynamicCreate_Menu_Number(pid, "slew_rate", &servo_max_add, float_Box);
        Menu_Set_Limit(v, 0.5f, 5,0.5);

        // 电机
        v = dynamicCreate_Menu_Number(pid, "base_duty", &motor_base_duty, int32_Box);
        Menu_Set_Limit(v, 0, 50,1);
        v = dynamicCreate_Menu_Number(pid, "max_duty",  &motor_max_duty, int32_Box);
        Menu_Set_Limit(v, 0, 50,1);
        v = dynamicCreate_Menu_Number(pid, "motor_kp",  &motor_kp, float_Box);
        Menu_Set_Limit(v, 0, 10,0.1);
        v = dynamicCreate_Menu_Number(pid, "motor_kd",  &motor_kd, float_Box);
        Menu_Set_Limit(v, 0, 2,0.01);
    }


    // =====Threshold阈值文件夹=====
    {
        Menu_Item *vis = dynamicCreate_Menu_Folder(&head, "Threshold");
        Menu_Item *v;

        v = dynamicCreate_Menu_Number(vis, "vis_low",  &vis_low,  uint8_Box);
        Menu_Set_Limit(v, 0, 255,1.0);
        v = dynamicCreate_Menu_Number(vis, "vis_high", &vis_high, uint8_Box);
        Menu_Set_Limit(v, 0, 255,1.0);
    }

    key = head.first_son;
}

//显示指针
static void show_key(void)
{
    Menu_Item *h = key->father;
    Menu_Item *s = h->first_son;
    for(int i = 0; i < h->sons; i++)
    {
        if(s == key)
        {
            if(s->select)
                ips200_show_string(0, i*16, "*>");
            else
                ips200_show_string(0, i*16, "->");
        }
        else
            ips200_show_string(0, i*16, "  ");
        s = s->next_brother;
    }
}

//显示数据
static void show_number(void)
{
    Menu_Item *h = key->father;
    Menu_Item *s = h->first_son;
    for(int i = 0; i < h->sons; i++)
    {
        switch (s->kind)
        {
        case int32_Box:
            ips200_show_int(90, i*16, *(int32_t *)s->data, 5);
            break;
        case float_Box:
            ips200_show_float(90, i*16, *(float *)s->data, 3, 2);
            break;
        case bool_Box:
            ips200_show_char(90, i*16, *(bool *)s->data ? 'Y' : 'N');
            break;
        case uint8_Box:
            ips200_show_int(90, i*16, *(uint8_t *)s->data, 3);
            break;
        default:
            break;
        }
        s = s->next_brother;
    }
}

void menu_show(void)
{
    Menu_Item *h = key->father;
    Menu_Item *s = h->first_son;
    for(int i = 0; i < h->sons; i++)
    {
        ips200_show_string(18, i*16, s->name);
        s = s->next_brother;
    }
    show_number();
    show_key();
}

static void k1_handle(void)
{
    if (key->select == false)
        key = key->last_brother;
    else
    {
        switch (key->kind)
        {
            case MENU_Folder:
                if (key->first_son != NULL)
                { ips200_clear(); key = key->first_son; key->select = false; }
                break;

            case bool_Box:
                *(bool*)key->data = !(*(bool*)key->data);
                break;

            case int32_Box:
                (*(int32_t*)key->data) += (int32_t)key->step;
                if (key->isLimit && *(int32_t*)key->data > (int32_t)key->limit_max)
                    *(int32_t*)key->data = (int32_t)key->limit_max;
                break;
            case uint32_Box:
                (*(uint32_t*)key->data) += (uint32_t)key->step;
                if (key->isLimit && *(uint32_t*)key->data > (uint32_t)key->limit_max)
                    *(uint32_t*)key->data = (uint32_t)key->limit_max;
                break;
            case int16_Box:
                (*(int16_t*)key->data) += (int16_t)key->step;
                if (key->isLimit && *(int16_t*)key->data > (int16_t)key->limit_max)
                    *(int16_t*)key->data = (int16_t)key->limit_max;
                break;
            case uint16_Box:
                (*(uint16_t*)key->data) += (uint16_t)key->step;
                if (key->isLimit && *(uint16_t*)key->data > (uint16_t)key->limit_max)
                    *(uint16_t*)key->data = (uint16_t)key->limit_max;
                break;
            case int8_Box:
                (*(int8_t*)key->data) += (int8_t)key->step;
                if (key->isLimit && *(int8_t*)key->data > (int8_t)key->limit_max)
                    *(int8_t*)key->data = (int8_t)key->limit_max;
                break;
            case uint8_Box:
                (*(uint8_t*)key->data) += (uint8_t)key->step;
                if (key->isLimit && *(uint8_t*)key->data > (uint8_t)key->limit_max)
                    *(uint8_t*)key->data = (uint8_t)key->limit_max;
                break;
            case float_Box:
                (*(float*)key->data) += key->step;
                if (key->isLimit && *(float*)key->data > key->limit_max)
                    *(float*)key->data = key->limit_max;
                break;
        }
    }
    menu_show();
}

static void k2_handle(void)
{
    if (key->select == false)
        key = key->next_brother;
    else
    {
        switch (key->kind)
        {
            case MENU_Folder:
                if (key->father->father != NULL)
                { ips200_clear(); key = key->father; key->select = false; }
                break;

            case bool_Box:
                break;

            case int32_Box:
                (*(int32_t*)key->data) -= (int32_t)key->step;
                if (key->isLimit && *(int32_t*)key->data < (int32_t)key->limit_min)
                    *(int32_t*)key->data = (int32_t)key->limit_min;
                break;
            case uint32_Box:
                (*(uint32_t*)key->data) -= (uint32_t)key->step;
                if (key->isLimit && *(uint32_t*)key->data < (uint32_t)key->limit_min)
                    *(uint32_t*)key->data = (uint32_t)key->limit_min;
                break;
            case int16_Box:
                (*(int16_t*)key->data) -= (int16_t)key->step;
                if (key->isLimit && *(int16_t*)key->data < (int16_t)key->limit_min)
                    *(int16_t*)key->data = (int16_t)key->limit_min;
                break;
            case uint16_Box:
                (*(uint16_t*)key->data) -= (uint16_t)key->step;
                if (key->isLimit && *(uint16_t*)key->data < (uint16_t)key->limit_min)
                    *(uint16_t*)key->data = (uint16_t)key->limit_min;
                break;
            case int8_Box:
                (*(int8_t*)key->data) -= (int8_t)key->step;
                if (key->isLimit && *(int8_t*)key->data < (int8_t)key->limit_min)
                    *(int8_t*)key->data = (int8_t)key->limit_min;
                break;
            case uint8_Box:
                (*(uint8_t*)key->data) -= (uint8_t)key->step;
                if (key->isLimit && *(uint8_t*)key->data < (uint8_t)key->limit_min)
                    *(uint8_t*)key->data = (uint8_t)key->limit_min;
                break;
            case float_Box:
                (*(float*)key->data) -= key->step;
                if (key->isLimit && *(float*)key->data < key->limit_min)
                    *(float*)key->data = key->limit_min;
                break;
        }
    }
    menu_show();
}

static void k3_handle(void)
{
    key->select = !key->select;
    menu_show();
}

static void k4_handle(void)
{
    if(key->father->father != NULL)
    {
        key->select = false;
        key = key->father;
        ips200_clear();
        menu_show();
    }
    else
        motor_stop();//根目录下k4停车

}

//K3长按：模式轮切
static void k3_long_handle(void)
{
    if(display_mode == DISPLAY_MODE_MENU)
    {
        display_mode = DISPLAY_MODE_BIN;
        Menu_Item *s = head.first_son;
        for (int i = 0; i < head.sons; i++)
        {
            if (s->kind == MENU_Folder && s->name[0] == 'T')
            { key = s->first_son; key->select = false; break; }
            s = s->next_brother;
        }
        ips200_clear();
    }
    else if(display_mode == DISPLAY_MODE_BIN)
    {
        display_mode = DISPLAY_MODE_TRACK;
        key->select = false;
    }
    else
    {
        display_mode = DISPLAY_MODE_MENU;
        key = head.first_son;
        key->select = false;
        ips200_clear();
        menu_show();
    }
}

//K4在BIN/TRACK模式：返回MENU模式
static void K4_back(void)
{
    display_mode = DISPLAY_MODE_MENU;
    key = head.first_son;
    key->select = false;
    ips200_clear();
    menu_show();
}

// ===== 主循环 =====
void menu(void)
{
    // K3长按：模式轮切（加防抖：key_clear_state防止重复触发）
    if(key_get_state(KEY_3) == KEY_RELEASE)
        k3_wait_release = false;
    if(!k3_wait_release && key_get_state(KEY_3) == KEY_LONG_PRESS)
    { k3_wait_release = true; k3_long_handle(); key_clear_state(KEY_3); }

    // =====MENU模式=====
    if(display_mode == DISPLAY_MODE_MENU)
    {
        if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
        { k1_handle(); key_clear_state(KEY_1); }
        if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
        { k2_handle(); key_clear_state(KEY_2); }
        if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
        { k3_handle(); key_clear_state(KEY_3); }
        if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
        { k4_handle(); key_clear_state(KEY_4); }
    }
    // =====BIN/TRACK按键处理=====
    else
    {
        if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
        { k1_handle(); key_clear_state(KEY_1); }
        if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
        { k2_handle(); key_clear_state(KEY_2); }
        if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
        { k3_handle(); key_clear_state(KEY_3); }
        if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
        { key_clear_state(KEY_4); K4_back(); }

        if (mt9v03x_finish_flag)
        {
            if (display_mode == DISPLAY_MODE_BIN)
                vis_bin_draw();
            else if (display_mode == DISPLAY_MODE_TRACK)
                vis_draw(); 
            menu_show();
            mt9v03x_finish_flag = 0;
        }
    }
}
