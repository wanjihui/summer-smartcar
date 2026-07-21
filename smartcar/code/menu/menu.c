//菜单的框架基础，包括菜单项的创建，菜单项的结构体定义等

#include "menu.h"

//菜单池，提取分配内存
#define MENU_MAX_SIZE    (64)//注意，按实际情况调整菜单项内存池
static Menu_Item menu_item_arr[MENU_MAX_SIZE];
static uint8_t menu_arr_index = 0;

//初始化菜单项
static void Create_Menu_Item(Menu_Item *father, Menu_Item *me, const char name[],void *data, MENU_KIND kind)
{
    if(father->kind != MENU_Folder)        //父节点不是文件夹类型
        return;                            //则不创建子节点

    me->name = name;                       //当前节点的名字 
    me->sons = 0;                          //当前节点的下级节点数
    me->select = false;                    //当前节点是否被选中
    me->data = data;                       //当前节点对应的数据
    me->kind = kind;                       //当前节点对应的数据类型 

    me->father = father;                   //当前节点的父节点
    me->first_son = NULL;                  //当前节点下的第一个子节点
    me->next_brother = NULL;               //当前节点的下一个兄弟节点
    me->last_brother = NULL;               //当前节点的上一个兄弟节点

    me->isLimit   = false;                  //当前节点对应的数据是否限幅
    me->limit_min = 0.0f;                   
    me->limit_max = 0.0f;
    me->step = 0;
    
    if(father->sons == 0)                   //当前节点是父节点的第一个子节点
    {
        father->first_son = me;
        me->next_brother = me;              //环形链表：单节点自环
        me->last_brother = me;
    }
    else
    {
        Menu_Item *first = father->first_son;
        Menu_Item *last  = first->last_brother; //环形链表 first->last_brother 就是末节点

        last->next_brother = me;            //旧末下一个  ->新节点
        me->last_brother   = last;          //新节点上一个->旧末
        me->next_brother   = first;         //新末下一个  ->首节点
        first->last_brother = me;           //首节点上一个->新末即新节点
    }
    me->No = father->sons;                  //当前节点在同级节点中排第几
    father->sons++;
}

//下为创建文件夹和创建文件这两种节点的静态动态接口函数

//创建文件夹
void Create_Menu_Folder(Menu_Item *father, Menu_Item *me, const char name[])
{
    Create_Menu_Item(father, me, name, NULL, MENU_Folder);
}
//创建文件
void Create_Menu_Number(Menu_Item *father, Menu_Item *me, const char name[], void *data, MENU_KIND kind)
{
    Create_Menu_Item(father, me, name, data, kind);
}

//动态创建文件夹
Menu_Item* dynamicCreate_Menu_Folder(Menu_Item *father, const char name[])
{
    if(menu_arr_index >= MENU_MAX_SIZE)
        return NULL;
    Menu_Item *me = &menu_item_arr[menu_arr_index++];
    Create_Menu_Item(father, me, name, NULL, MENU_Folder);
    return me;
}
//动态创建文件
Menu_Item* dynamicCreate_Menu_Number(Menu_Item *father, const char name[], void *data, MENU_KIND kind)
{
    if(menu_arr_index >= MENU_MAX_SIZE)
        return NULL;
    Menu_Item *me = &menu_item_arr[menu_arr_index++];
    Create_Menu_Item(father, me, name, data, kind);
    return me;
}

//设置数据限幅和步进
void Menu_Set_Limit(Menu_Item *me, float min, float max, float step)
{
    me->isLimit   = true;
    me->limit_min = min;
    me->limit_max = max;
    me->step = step;
}