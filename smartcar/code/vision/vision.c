#include "config.h"

uint8  g_bin_image[pho_h][pho_w];

 border_line l_border;
 border_line r_border;
 border_line center_line;

 volatile float err;
 volatile int chongchu;

 // 双阈值参数 实时调节
uint8 vis_low  = DEFAULT_VIS_LOW;
uint8 vis_high = DEFAULT_VIS_HIGH;

// 双阈值二值化判定
static int is_white(uint8 p)
{
    if (p < vis_low)  return 0;
    if (p > vis_high) return 1;
    return (p >= (vis_low + vis_high) / 2) ? 1 : 0;
}

// 图像底部找左右种子，return 1找到，0失败
static int find_seeds(int *seed_l_x, int *seed_r_x)
{
    int y = pho_h - 2;

    // 左种子：从左往右扫，黑变白进入赛道，3连白确认
    *seed_l_x = -1;
    for (int x = 1; x < pho_w - 3; x++)
    {
        if (!is_white(mt9v03x_image[y][x - 1])
            && is_white(mt9v03x_image[y][x])
            && is_white(mt9v03x_image[y][x + 1])
            && is_white(mt9v03x_image[y][x + 2]))
        {
            *seed_l_x = x;
            break;
        }
    }

    // 右种子：从右往左扫，黑变白进入赛道，3连白确认
    *seed_r_x = -1;
    for (int x = pho_w - 2; x >= 2; x--)
    {
        if (is_white(mt9v03x_image[y][x])
            && !is_white(mt9v03x_image[y][x + 1])
            && is_white(mt9v03x_image[y][x - 1])
            && is_white(mt9v03x_image[y][x - 2]))
        {
            *seed_r_x = x;
            break;
        }
    }
    return (*seed_l_x >= 0 && *seed_r_x >= 0) ? 1 : 0;
}

/*
 * 八邻域种子生长，搜线核心算法
 *
 * 方向表：左顺时针，右逆时针
 * 每步：8方向找黑变白跳变，选y最小的（即最上边的），记录
 *
 * 断点续搜：
 *   8方向没候选
 *   - 连续往上扫3行，全行扫描加3连白确认
 *     左：从左往右扫黑变白
 *     右：从右往左扫黑变白
 *   - 找到了续上，继续正常生长
 *   - 3行全失败，终止
 *
 * 终止：两边都爬到顶 (cy==0, cry==0)
 *       3行重试全失败
 *       存储种子达SEED_MAX_POINTS
 */

#define SEED_MAX_POINTS  (pho_h * 3)//预留横向

static uint16 points_l[SEED_MAX_POINTS][2];//存储种子坐标
static uint16 points_r[SEED_MAX_POINTS][2];
static uint16 data_l, data_r;//存储种子生长数
static uint8  highest_y;//记录种子生长最高点


static void seed_grow(int lx, int ly, int rx, int ry)
{
    //左右方向表
    static const int8 dir_l[8][2] = {
        { 0, 1}, {-1, 1}, {-1, 0}, {-1,-1}, { 0,-1}, { 1,-1}, { 1, 0}, { 1, 1}
    };
    static const int8 dir_r[8][2] = {
        { 0, 1}, { 1, 1}, { 1, 0}, { 1,-1}, { 0,-1}, {-1,-1}, {-1, 0}, {-1, 1}
    };

    int clx = lx, cly = ly;
    int crx = rx, cry = ry;
    int il = 0, ir = 0;

    for (int iter = 0; iter < SEED_MAX_POINTS; iter++)
    {
        // ===== 左边界 =====
        points_l[il][0] = clx;
        points_l[il][1] = cly;
        il++;

        int best_ly = 999, best_lx = -1;//初始化候选种子坐标

        for (int i = 0; i < 8; i++)
        {
            int nx1  = clx + dir_l[i][0];
            int ny1  = cly + dir_l[i][1];
            int nx2 = clx + dir_l[(i + 1) & 7][0];//&7等效%8，更快
            int ny2 = cly + dir_l[(i + 1) & 7][1];

            //跳过超出图像范围的点 防止越界
            if (nx1 < 0 || nx1 >= pho_w || ny1 < 0 || ny1 >= pho_h) continue;
            if (nx2 < 0 || nx2 >= pho_w || ny2 < 0 || ny2 >= pho_h) continue;

            if (!is_white(mt9v03x_image[ny1][nx1])
                && is_white(mt9v03x_image[ny2][nx2]))
            {
                 if (ny1 < best_ly) { best_ly = ny1; best_lx = nx1; }//选y小的
            }
        }

        if (best_ly != 999)
        {
            clx = best_lx;
            cly = best_ly;
        }
        else if (cly > 0)
        {
            // 8邻域失败，往上扫3行
            int found = 0;//是否扫描到标志位
            for (int retry = 0; retry < 3 && cly > 0; retry++)
            {
                cly--;
                for (int x = 1; x < pho_w - 3; x++)
                {
                    if (!is_white(mt9v03x_image[cly][x-1])
                        && is_white(mt9v03x_image[cly][x])
                        && is_white(mt9v03x_image[cly][x+1])
                        && is_white(mt9v03x_image[cly][x+2]))
                    {
                        clx = x;
                        found = 1;
                        break;
                    }
                }
                if (found) break;
            }
            if (!found) break;
        }

        // ===== 右边界 =====
        points_r[ir][0] = crx;
        points_r[ir][1] = cry;

        int best_ry = 999, best_rx = -1;

        for (int i = 0; i < 8; i++)
        {
            int nx  = crx + dir_r[i][0];
            int ny  = cry + dir_r[i][1];
            int nx2 = crx + dir_r[(i + 1) & 7][0];
            int ny2 = cry + dir_r[(i + 1) & 7][1];

            if (nx < 0 || nx >= pho_w || ny < 0 || ny >= pho_h) continue;
            if (nx2 < 0 || nx2 >= pho_w || ny2 < 0 || ny2 >= pho_h) continue;

            if (!is_white(mt9v03x_image[ny][nx])
                && is_white(mt9v03x_image[ny2][nx2]))
            {
                if (ny < best_ry) { best_ry = ny; best_rx = nx; }
            }
        }

        if (best_ry != 999)
        {
            crx = best_rx;
            cry = best_ry;
        }
        else if (cry > 0)
        {
            // 8邻域失败，往上扫3行
            int found = 0;
            for (int retry = 0; retry < 3 && cry > 0; retry++)
            {
                cry--;
                for (int x = pho_w - 2; x >= 2; x--)
                {
                    if (is_white(mt9v03x_image[cry][x])
                        && !is_white(mt9v03x_image[cry][x+1])
                        && is_white(mt9v03x_image[cry][x-1])
                        && is_white(mt9v03x_image[cry][x-2]))
                    {
                        crx = x;
                        found = 1;
                        break;
                    }
                }
                if (found) break;
            }
            if (!found) break;
        }

        ir++;

        if (cly == 0 && cry == 0) break;
    }

    data_l = il;//左右种子生长数
    data_r = ir;
    highest_y = (cly < cry) ? cly : cry;
}

/*
 * 从种子点序列提取按行边界数组
 * dir：左+1 右-1（种子在黑色边界上，赛道为内侧白像素）
 */
static void pho_border(uint16 points[][2], uint16 total,
                           border_line border, int dir)
{
    /* 先填默认值 */
    for (int y = 0; y < pho_h; y++)
    {
        border[y] = (dir > 0) ? pho_w_min : pho_w_max;
    }

    /* 按 y 写入，同行只取第一个 */
    int last_y = -1;
    for (int i = 0; i < total; i++)
    {
        int py = points[i][1];
        int px = points[i][0];

        if (py == last_y) continue;
        last_y = py;

        //防越界
        int val = px + dir;
        if (val < pho_w_min)    val = pho_w_min;
        if (val > pho_w_max)    val = pho_w_max;
        border[py] = (uint8)val;    }
}

/*
 * 计算中线数组和赛道偏差
 * err即屏幕中线离赛道中线距离
 * err计算: 40~100行，每5行取一个点，线性加权
 * err>0右转,err<0左转
 */
static void pho_center(void)
{
    for (int y = 0; y < pho_h-5; y++)
    {
        center_line[y] = (uint8)((l_border[y] + r_border[y]) / 2);
    }

    float sum = 0, w_sum = 0;
    for (int row = 40; row <= 100; row += 5)
    {
        float w = (float)(row - 35) / 65.0f;  // 线性加权，近处权重大
        sum += ((float)center_line[row] - pho_center_x) * w;
        w_sum += w;
    }
    err = sum / w_sum;
}


/*
 * 搜线主函数
 * 无屏幕显示
 * return: 0正常, 1丢线
 */
int vis_deal(void)
{
    int lx, rx;
    if (!find_seeds(&lx, &rx)) return 1;   // 找不到种子 丢线

    data_l = 0; data_r = 0;
    seed_grow(lx, pho_h - 2, rx, pho_h - 2);

    //左右边线
    pho_border(points_l, data_l, l_border, +1);
    pho_border(points_r, data_r, r_border, -1);

    //中线和err
    pho_center();

    return 0;
}

/*
 * 显示搜线结果
 * 调用 vis_deal() 读取了边界数组后
 * 灰度原图上叠加 红色左右边界 绿色中线
 * 跳过底部5行不画
 */
void vis_draw(void)
{
    ips200_show_gray_image(0, BIN_PARAM_H, (const uint8 *)mt9v03x_image,
                       pho_w, pho_h, pho_w, pho_h, 0);

    for (int y = 0; y < pho_h - 5; y++)
    {
        uint8 l = l_border[y];
        uint8 r = r_border[y];
        uint8 c = center_line[y];
        int dy = y + BIN_PARAM_H;          
        // 图像放到参数区域下方

        //防越界
        ips200_draw_point(l, dy, RGB565_RED);
        if (l < pho_w_max)  ips200_draw_point(l + 1, dy, RGB565_RED);

        ips200_draw_point(r, dy, RGB565_RED);
        if (r > pho_w_min)  ips200_draw_point(r - 1, dy, RGB565_RED);

        ips200_draw_point(c, dy, RGB565_GREEN);
        if (c < pho_w_max)  ips200_draw_point(c + 1, dy, RGB565_GREEN);
    }
}


/*
 * 显示双阈值二值化结果 + 底部数值显示
 * 纯黑白 用来确认阈值是否把赛道和背景分开
 */
void vis_bin_draw(void)
{
    for (int y = 0; y < pho_h; y++)
        for (int x = 0; x < pho_w; x++)
            g_bin_image[y][x] = is_white(mt9v03x_image[y][x]) ? 255 : 0;

    ips200_show_gray_image(0, BIN_PARAM_H, (const uint8 *)g_bin_image,
                       pho_w, pho_h, pho_w, pho_h, 8);
}