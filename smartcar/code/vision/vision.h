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
#define SEED_MAX_POINTS  (pho_h * 3)       // 种子点最大存储数（预留横向空间）

extern uint8 lookahead;                    // 预瞄距离（行），菜单可调，默认30

typedef uint8 border_line[pho_h];        // 重定义一个边界线数组类型，存储每行的边界点列号

// ========== 全局变量 ==========
extern border_line l_border;       // 左边界，每行一个列号
extern border_line r_border;       // 右边界
extern border_line center_line;    // 中线 = (左+右)/2，车应该沿它走

extern uint16 points_l[SEED_MAX_POINTS][2];  // 左边界种子点（用于完整描绘）
extern uint16 points_r[SEED_MAX_POINTS][2];  // 右边界种子点
extern uint16 data_l;                        // 左种子点数量
extern uint16 data_r;                        // 右种子点数量

extern volatile float err;     // 赛道中线-图像中心=偏差,>0偏右,右转;<0偏左,左转
extern volatile uint8_t vis_frame_ready; // 新帧已搜线完成，显示层可刷新


// ========== 自适应寻线参数（菜单可调）==========
extern int32_t Block_Size;   // 局部窗口边长（奇数），默认9
extern int32_t Clip_Value;   // 阈值偏置，越大越严格，默认4
extern float   err_alpha;    // err EMA平滑系数，0.2~0.6
#define MAX_TRACK_POINTS  200   // 单边最大追踪步数（编译期常量）

// ========== 工具宏 ==========
#define clip(x, min, max)  (((x) > (max)) ? (max) : (((x) < (min)) ? (min) : (x)))
#define MAX(a, b)          (((a) > (b)) ? (a) : (b))
#define MIN(a, b)          (((a) < (b)) ? (a) : (b))

// ========== MCZSCS 自适应寻线用 ==========
extern uint8 Image_Used[pho_h][pho_w];   // Otsu二值化后的图像，0=黑 255=白
#define AT(x, y)  (Image_Used[clip(y, 0, pho_h-1)][clip(x, 0, pho_w-1)])

// ========== 函数声明 ==========
int  vis_deal(void);   //搜线，return 0正常 1丢线
void vis_draw(void);   //图像显示
void vis_bin_draw(void);  //二值化显示（调参）
int  is_straight(void);         // 直道判定，1=直道 0=弯道
int  lost_line_left(void);      // 左丢线检测，-1=正常 else=丢线行号
int  lost_line_right(void);     // 右丢线检测

#endif