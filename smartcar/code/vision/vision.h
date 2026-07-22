#ifndef VISION_H
#define VISION_H

#include "zf_common_headfile.h"


// ========== 图像尺寸 ==========
#define pho_h             MT9V03X_H        // 120 行
#define pho_w             MT9V03X_W        // 188 列
#define pho_center_x      (pho_w / 2)      // 94，图像水平中心
#define pho_w_min    0                     // 最左列
#define pho_w_max    (pho_w - 1)           // 187，最右列
#define BIN_PARAM_H  48                    // 阈值参数区高度(3行)

typedef uint8 border_line[pho_h];        // 重定义一个边界线数组类型，存储每行的边界点列号

// ========== 全局变量 ==========
extern uint8  g_bin_image[pho_h][pho_w];        // 二值化后的图像数组

extern border_line l_border;       // 左边界，每行一个列号
extern border_line r_border;       // 右边界
extern border_line center_line;    // 中线 = (左+右)/2，车应该沿它走

extern volatile float err;     // 赛道中线-图像中心=偏差,>0偏右,右转;<0偏左,左转
extern volatile uint8_t vis_frame_ready; // 新帧已搜线完成，显示层可刷新

extern uint8 vis_low;//二值化参数
extern uint8 vis_high;

// ========== 函数声明 ==========
int  vis_deal(void);   //搜线，return 0正常 1丢线
void vis_draw(void);   //图像显示
void vis_bin_draw(void);  //二值化显示（调参）

#endif 