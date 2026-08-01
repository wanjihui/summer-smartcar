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

    // =====PID文件夹=====
    {
        Menu_Item *pid = dynamicCreate_Menu_Folder(&head, "PID");
        Menu_Item *v;

        // 舵机
        v = dynamicCreate_Menu_Number(pid, "servokp1",  &servo_kp1, float_Box);
        Menu_Set_Limit(v, 0, 5,0.05f);
        v = dynamicCreate_Menu_Number(pid, "servokp2",  &servo_kp2, float_Box);
        Menu_Set_Limit(v, 0, 0.01f,0.0001f);
        v = dynamicCreate_Menu_Number(pid, "servo_kd",  &servo_kd,  float_Box);
        Menu_Set_Limit(v, 0, 2,0.01f);
        v = dynamicCreate_Menu_Number(pid, "gyro_kd",   &gyro_kd, float_Box);
        Menu_Set_Limit(v, 0.0f, 0.05f, 0.001f);
        v = dynamicCreate_Menu_Number(pid, "gyro_kdc",  &gyro_kd_curve, float_Box);
        Menu_Set_Limit(v, 0.0f, 0.05f, 0.001f);
        v = dynamicCreate_Menu_Number(pid, "far", &asc_far, uint8_Box);
        Menu_Set_Limit(v, 5, 60, 5.0f);
        v = dynamicCreate_Menu_Number(pid, "st_far", &straight_far, uint8_Box);
        Menu_Set_Limit(v, 5, 60, 5.0f);
        v = dynamicCreate_Menu_Number(pid, "max_add", &servo_max_add, float_Box);
        Menu_Set_Limit(v, 0.5f, 20,0.5f);
        v = dynamicCreate_Menu_Number(pid, "center",    &servo_center, float_Box);
        Menu_Set_Limit(v, 75, 105,0.1f);
        v = dynamicCreate_Menu_Number(pid, "deadband",  &servo_dead, float_Box);
        Menu_Set_Limit(v, 0, 10,0.5f);


        // 电机
        v = dynamicCreate_Menu_Number(pid, "base_duty", &motor_base_duty, int32_Box);
        Menu_Set_Limit(v, 0, 50,1.0f);
        v = dynamicCreate_Menu_Number(pid, "cur_duty", &motor_curve_duty, int32_Box);
        Menu_Set_Limit(v, 0, 50,1.0f);
        v = dynamicCreate_Menu_Number(pid, "motor_kp",  &motor_kp, float_Box);
        Menu_Set_Limit(v, 0, 10,0.01f);
        v = dynamicCreate_Menu_Number(pid, "motor_kd",  &motor_kd, float_Box);
        Menu_Set_Limit(v, 0, 2,0.01f);
        v = dynamicCreate_Menu_Number(pid, "diff_max",  &motor_diff_max, int32_Box);
        Menu_Set_Limit(v, 2, 20,1.0f);
        dynamicCreate_Menu_Number(pid, "car_run",   &car_run,  bool_Box);
    }

    // =====Debug文件夹=====
    {
        Menu_Item *dbg = dynamicCreate_Menu_Folder(&head, "Debug");
        Menu_Item *v;

        v = dynamicCreate_Menu_Number(dbg, "err",     (float*)&err, float_Box);
        v = dynamicCreate_Menu_Number(dbg, "valid",   (int16*)&asc_valid_dbg, int16_Box);
        v = dynamicCreate_Menu_Number(dbg, "hold",    (uint8*)&hold_dbg, uint8_Box);
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
            ips200_show_float(90, i*16, *(float *)s->data, 6, 4);
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
        key = head.first_son;   // 兜底：无Debug文件夹时回根目录
        Menu_Item *s = head.first_son;
        for (int i = 0; i < head.sons; i++)
        {
            if (s->kind == MENU_Folder && s->name[0] == 'D')
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

        /* 显示降频：每4帧刷一次屏（50FPS下≈12.5Hz）。图像是给人看的，
         * 无需每帧刷新；SPI写屏约18ms，减少对控制链路的阻塞 */
        static uint8 disp_cnt = 0;
        if (vis_frame_ready)
        {
            if (++disp_cnt >= 4)
            {
                disp_cnt = 0;
                if (display_mode == DISPLAY_MODE_BIN)
                    vis_bin_draw();
                else if (display_mode == DISPLAY_MODE_TRACK)
                    vis_draw();
                menu_show();
            }
            vis_frame_ready = 0;
        }
    }
}
