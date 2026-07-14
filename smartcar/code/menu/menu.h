#ifndef __MENU_H_
#define __MENU_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef enum MENU_KIND {
    MENU_Folder = 0,        //枚举默认第一个赋值为0，此为规范写法
    int32_Box,
    uint32_Box,
    int16_Box,
    uint16_Box,
    int8_Box,
    uint8_Box,
    float_Box,
    bool_Box,
}MENU_KIND;

typedef struct Menu_Item
{
    const char *name;
    void *data;             //当前节点对应的数据
    MENU_KIND kind;         //当前节点对应的数据类型
    bool select;             //当前节点是否被选中

    uint8_t sons;           //当前节点对应的下级节点数
    uint8_t No;             //当前节点在同级节点中排第几

    struct Menu_Item *father;
    struct Menu_Item *first_son;
    struct Menu_Item *next_brother;
    struct Menu_Item *last_brother;

    bool isLimit;       //当前节点对应的数据是否限幅
    float limit_min;
    float limit_max;
} Menu_Item;

void Create_Menu_Folder(Menu_Item *father, Menu_Item *me, const char name[]);
void Create_Menu_Number(Menu_Item *father, Menu_Item *me, const char name[], void *data, MENU_KIND kind);
Menu_Item* dynamicCreate_Menu_Folder(Menu_Item *father, const char name[]);
Menu_Item* dynamicCreate_Menu_Number(Menu_Item *father, const char name[], void *data, MENU_KIND kind);
void Menu_Set_Limit(Menu_Item *me, float min, float max);

#endif