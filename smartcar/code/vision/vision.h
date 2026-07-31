#ifndef VISION_H
#define VISION_H

#include "zf_common_headfile.h"


// ========== 图像尺寸 ==========
#define pho_h             MT9V03X_H        // 120 行
#define pho_w             MT9V03X_W        // 188 列
#define pho_center_x      (pho_w / 2)      // 94，图像水平中心
#define pho_w_min    0                     // 最左列
#define pho_w_max    (pho_w - 1)           // 187，最右列
#define BORDER_INVALID 255                 // 边界无效标记（不与0~187合法列号冲突）
#define BIN_PARAM_H  48                    // 阈值参数区高度(3行)

extern uint8 asc_far;                       // ASC采样远行（顶部行），菜单可调，默认10
extern uint8 straight_far;                  // 直线判定采样远行（顶部行），菜单可调，默认30

typedef uint8 border_line[pho_h];        // 重定义一个边界线数组类型，存储每行的边界点列号

// ========== 全局变量 ==========
extern border_line l_border;       // 左边界，每行一个列号
extern border_line r_border;       // 右边界
extern border_line center_line;    // 中线 = (左+右)/2，车应该沿它走
extern uint8 l_border_exist[pho_h];  // 左边界该行是否真实存在（1=真边界 0=出画伪边界/无）
extern uint8 r_border_exist[pho_h];  // 右边界该行是否真实存在

extern volatile float err;     // 赛道中线-图像中心=偏差,>0偏右,右转;<0偏左,左转
extern volatile uint8_t vis_frame_ready; // 新帧已搜线完成，显示层可刷新
extern volatile int16  asc_valid_dbg;    // 诊断：ASC窗口有效行数
extern volatile uint8  hold_dbg;         // 诊断：锁存激活标志


// ========== 二值化图像缓存 ==========
extern uint8 Image_Used[pho_h][pho_w];   // Otsu二值化后的图像，0=黑 255=白

// ========== 函数声明 ==========
void vis_deal(void);   //搜线，无返回值
void vis_draw(void);   //图像显示
void vis_bin_draw(void);  //二值化显示（调参）
int  is_straight(void);         // 直道判定，1=直道 0=弯道

#endif 