//菜单具体构建

#include "Mymenu.h"

Menu_Item *key;//当前选中节点的指针
Menu_Item head;//菜单的根节点

int32_t test = 10;
float test1 = 3.14;
bool test2 = true;

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

    Menu_Item *folder1 = dynamicCreate_Menu_Folder(&head, "folder1");
    Menu_Item *folder2 = dynamicCreate_Menu_Folder(&head, "folder2");
    dynamicCreate_Menu_Folder(folder2, "folder3");
    dynamicCreate_Menu_Folder(folder2, "folder4");

    Menu_Item *num1 = dynamicCreate_Menu_Number(folder1, "aaa", &test, int32_Box);
    Menu_Set_Limit(num1, 0, 100);                   // int32 限幅 0~100

    Menu_Item *num2 = dynamicCreate_Menu_Number(folder1, "bbb", &test1, float_Box);
    Menu_Set_Limit(num2, -5.0f, 5.0f);              // float 限幅 -5.0~5.0

    dynamicCreate_Menu_Number(folder1, "ccc", &test2, bool_Box); // bool 不需要限幅

    key = head.first_son;
}

static void show_key(void)
{
    Menu_Item *h = key->father;
    Menu_Item *s = h->first_son;

    for(int i = 0; i < h->sons; i++)
    {
        if(s == key)
        {
            if(s->select)
                ips200_show_string(0, i*8, "*>");    // 编辑模式
            else
                ips200_show_string(0, i*8, "->");    // 导航模式
        }
        else
        {
            ips200_show_string(0, i*8, "  ");
        }
        s = s->next_brother;
    }
}

static void show_number(void)
{
    Menu_Item *h = key->father;
    Menu_Item *s = h->first_son;

    for(int i = 0; i < h->sons; i++)
    {
        switch (s->kind)
        {
        case int32_Box:
            ips200_show_int(80, i*8, *(int32_t *)s->data, 5);
            break;
        case float_Box:
            ips200_show_float(80, i*8, *(float *)s->data, 3, 2);
            break;
        case bool_Box:
            if(*(bool *)s->data == true)
                ips200_show_char(80, i*8, 'Y');
            else
                ips200_show_char(80, i*8, 'N');
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
        ips200_show_string(14, i*8, s->name);//从x=14显示菜单项名称，2*6指示符+2留空
        s = s->next_brother;
    }
    show_number();
    show_key();
}
// K1 短按：导航模式→上移 / 编辑模式→进入子菜单或数值+1或bool翻转
void k1_handle(void)
{
    if(key->select == false)
    {
        // 导航模式：指针上移（环形链表，无边界）
        key = key->last_brother;
    }
    else
    {
        // 编辑模式：看类型
        switch(key->kind)
        {
            case MENU_Folder:
                if(key->first_son != NULL)
                {
                    key = key->first_son;
                    key->select = false;            // 进入后切回导航模式
                }
                break;

            case bool_Box:
                *(bool*)key->data = !(*(bool*)key->data);
                break;

            case int32_Box:
                (*(int32_t*)key->data)++;
                if(key->isLimit && *(int32_t*)key->data > (int32_t)key->limit_max)
                    *(int32_t*)key->data = (int32_t)key->limit_max;
                break;
            case uint32_Box:
                (*(uint32_t*)key->data)++;
                if(key->isLimit && *(uint32_t*)key->data > (uint32_t)key->limit_max)
                    *(uint32_t*)key->data = (uint32_t)key->limit_max;
                break;
            case int16_Box:
                (*(int16_t*)key->data)++;
                if(key->isLimit && *(int16_t*)key->data > (int16_t)key->limit_max)
                    *(int16_t*)key->data = (int16_t)key->limit_max;
                break;
            case uint16_Box:
                (*(uint16_t*)key->data)++;
                if(key->isLimit && *(uint16_t*)key->data > (uint16_t)key->limit_max)
                    *(uint16_t*)key->data = (uint16_t)key->limit_max;
                break;
            case int8_Box:
                (*(int8_t*)key->data)++;
                if(key->isLimit && *(int8_t*)key->data > (int8_t)key->limit_max)
                    *(int8_t*)key->data = (int8_t)key->limit_max;
                break;
            case uint8_Box:
                (*(uint8_t*)key->data)++;
                if(key->isLimit && *(uint8_t*)key->data > (uint8_t)key->limit_max)
                    *(uint8_t*)key->data = (uint8_t)key->limit_max;
                break;
            case float_Box:
                (*(float*)key->data) += 0.1f;
                if(key->isLimit && *(float*)key->data > key->limit_max)
                    *(float*)key->data = key->limit_max;
                break;

            default: break;
        }
    }
    menu_show();
}

// K2 短按：导航模式→下移 / 编辑模式→退出父菜单或数值-1(bool无效)
void k2_handle(void)
{
    if(key->select == false)
    {
        // 导航模式：指针下移（环形链表，无边界）
        key = key->next_brother;
    }
    else
    {
        // 编辑模式：看类型
        switch(key->kind)
        {
            case MENU_Folder:
                if(key->father->father != NULL)
                {
                    key = key->father;
                    key->select = false;            // 退出后切回导航模式
                }
                break;

            case bool_Box:
                // K2 不操作 bool
                break;

            case int32_Box:
                (*(int32_t*)key->data)--;
                if(key->isLimit && *(int32_t*)key->data < (int32_t)key->limit_min)
                    *(int32_t*)key->data = (int32_t)key->limit_min;
                break;
            case uint32_Box:
                (*(uint32_t*)key->data)--;
                if(key->isLimit && *(uint32_t*)key->data < (uint32_t)key->limit_min)
                    *(uint32_t*)key->data = (uint32_t)key->limit_min;
                break;
            case int16_Box:
                (*(int16_t*)key->data)--;
                if(key->isLimit && *(int16_t*)key->data < (int16_t)key->limit_min)
                    *(int16_t*)key->data = (int16_t)key->limit_min;
                break;
            case uint16_Box:
                (*(uint16_t*)key->data)--;
                if(key->isLimit && *(uint16_t*)key->data < (uint16_t)key->limit_min)
                    *(uint16_t*)key->data = (uint16_t)key->limit_min;
                break;
            case int8_Box:
                (*(int8_t*)key->data)--;
                if(key->isLimit && *(int8_t*)key->data < (int8_t)key->limit_min)
                    *(int8_t*)key->data = (int8_t)key->limit_min;
                break;
            case uint8_Box:
                (*(uint8_t*)key->data)--;
                if(key->isLimit && *(uint8_t*)key->data < (uint8_t)key->limit_min)
                    *(uint8_t*)key->data = (uint8_t)key->limit_min;
                break;
            case float_Box:
                (*(float*)key->data) -= 0.1f;
                if(key->isLimit && *(float*)key->data < key->limit_min)
                    *(float*)key->data = key->limit_min;
                break;

            default: break;
        }
    }
    menu_show();
}

// K3 短按：切换选中状态
void k3_handle(void)
{
    key->select = !key->select;
    menu_show();
}

// K4 短按：全局返回上一级
void k4_handle(void)
{
    if(key->father->father != NULL)   // 不在根层级
    {
        key->select = false;          // 回到导航模式
        key = key->father;
        menu_show();
    }
}