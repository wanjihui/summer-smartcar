#include "config.h"

uint8  g_bin_image[pho_h][pho_w];

 border_line l_border;
 border_line r_border;
 border_line center_line;

 volatile float err;
 volatile uint8_t vis_frame_ready;

 // 双阈值参数 实时调节
uint8 vis_low  = DEFAULT_VIS_LOW;
uint8 vis_high = DEFAULT_VIS_HIGH;

/* ================================================================
 * 双阈值二值化判定
 *   p < vis_low        → 0（黑，非赛道）
 *   p > vis_high       → 1（白，赛道）
 *   vis_low ≤ p ≤ vis_high → 中值自动判定
 * ================================================================ */
static int is_white(uint8 p)
{
    if (p < vis_low)  return 0;
    if (p > vis_high) return 1;
    return (p >= (vis_low + vis_high) / 2) ? 1 : 0;
}

/* ================================================================
 * 90→40行独立找左右种子，左右各自返回，不同行也没关系
 * 返回: 左右种子独立标记，至少一个找到就可以生长
 * ================================================================ */
static void find_seeds(int *llx, int *lly, int *rrx, int *rry)
{
    *llx = -1; *rrx = -1;

    for (int y = 90; y >= 40; y--)
    {
        // 左种子：从中间向左扫，找赛道左边缘（白→黑转跳）+ 3连白确认赛道内
        if (*llx < 0 && is_white(mt9v03x_image[y][pho_center_x]))
        {
            for (int x = pho_center_x; x >= 2; x--)
            {
                if (!is_white(mt9v03x_image[y][x - 1])
                    && is_white(mt9v03x_image[y][x])
                    && is_white(mt9v03x_image[y][x + 1])
                    && is_white(mt9v03x_image[y][x + 2]))
                {
                    *llx = x; *lly = y; break;
                }
            }
            if (*llx < 0) { *llx = 0; *lly = y; }  // 扫到底全白 → 出界
        }

        // 右种子：从中间向右扫，找赛道右边缘（白→黑转跳）+ 3连白确认赛道内
        if (*rrx < 0 && is_white(mt9v03x_image[y][pho_center_x]))
        {
            for (int x = pho_center_x; x < pho_w - 3; x++)
            {
                if (is_white(mt9v03x_image[y][x])
                    && !is_white(mt9v03x_image[y][x + 1])
                    && is_white(mt9v03x_image[y][x - 1])
                    && is_white(mt9v03x_image[y][x - 2]))
                {
                    *rrx = x; *rry = y; break;
                }
            }
            if (*rrx < 0) { *rrx = pho_w_max; *rry = y; }  // 扫到底全白 → 出界
        }

        if (*llx >= 0 && *rrx >= 0) break;
    }
}

/* ================================================================
 * 底部扫描：逐行扫 118→79（40行），直接填 l_border/r_border
 * 赛道近处宽且可靠，不需要种子生长，逐行扫更快更准
 * ================================================================ */
static void bottom_scan(void)
{
    for (int y = 118; y >= 79; y--)
    {
        int lx = -1, rx = -1;
        int m = pho_center_x;

        if (!is_white(mt9v03x_image[y][m])) { l_border[y] = 0; r_border[y] = 0; continue; }

        // 左边界：从中间向左扫，找赛道左边缘（白→黑）+2连白
        for (int x = m; x >= 1; x--)
        {
            if (!is_white(mt9v03x_image[y][x - 1])  // 白→黑转跳
                && is_white(mt9v03x_image[y][x])
                && is_white(mt9v03x_image[y][x + 1]))
            {
                lx = x + 1; break;
            }
        }
        if (lx < 0) lx = pho_w_min;  // 扫到底全白 → 出界

        // 右边界：从中间向右扫，找赛道右边缘（白→黑）+2连白
        for (int x = m; x < pho_w - 2; x++)
        {
            if (is_white(mt9v03x_image[y][x])
                && !is_white(mt9v03x_image[y][x + 1])
                && is_white(mt9v03x_image[y][x - 1]))
            {
                rx = x - 1; break;
            }
        }
        if (rx < 0) rx = pho_w_max;  // 扫到底全白 → 出界

        l_border[y] = (uint8)lx;
        r_border[y] = (uint8)rx;
    }
}

/* ================================================================
 * 八邻域种子生长 搜线核心算法
 *
 * 左右各自独立生长，防止一侧失败影响另一侧
 *
 * 方向表：
 *   左边界（逆时针）
 *   右边界（顺时针）
 *
 * 每步逻辑：
 *   遍历8个方向对 (i, i+1)，找"方向i=黑 且 方向i+1=白"的跳变
 *   选 ny 最小（图像最上方）的候选 → 种子整体向上爬行
 *
 * 断点续搜：
 *   8方向全失败 → 连续往上扫5行
 *   找到 → 续上继续正常生长
 *   5行全失败 → 终止该侧搜索
 *
 * 终止条件：
 *   爬到顶 (cy <= 20)
 *   5行重试全失败
 *   存储点数达到 SEED_MAX_POINTS
 * ================================================================ */

#define SEED_MAX_POINTS  (pho_h * 3)  // 预留横向空间

static uint16 points_l[SEED_MAX_POINTS][2];  // 左边界种子坐标存储
static uint16 points_r[SEED_MAX_POINTS][2];  // 右边界种子坐标存储
static uint16 data_l, data_r;                // 左右各自实际存储点数

/* ---左边界生长--- */
static void seed_grow_left(int lx, int ly)
{
    // 逆时针方向
    static const int8 dir_l[8][2] = {
        { 0, 1}, {-1, 1}, {-1, 0}, {-1,-1}, { 0,-1}, { 1,-1}, { 1, 0}, { 1, 1}
    };

    int cx = lx, cy = ly;   // 当前种子坐标（黑像素）
    int idx = 0;            // 存储种子数

    for (int iter = 0; iter < SEED_MAX_POINTS; iter++)
    {
        // 存入当前种子
        points_l[idx][0] = cx;
        points_l[idx][1] = cy;
        idx++;

        int best_y = 999, best_x = -1;  // 候选坐标默认值

        //----八邻域搜索----
        for (int i = 0; i < 8; i++)
        {
            int nx1 = cx + dir_l[i][0];              //方向i应为黑
            int ny1 = cy + dir_l[i][1];
            int nx2 = cx + dir_l[(i + 1) & 7][0];    //方向i+1应为白
            int ny2 = cy + dir_l[(i + 1) & 7][1];    //&7等效%8 更快

            //防越界
            if (nx1 < 0 || nx1 >= pho_w || ny1 < 0 || ny1 >= pho_h) continue;
            if (nx2 < 0 || nx2 >= pho_w || ny2 < 0 || ny2 >= pho_h) continue;

            //选y最小（最上）的候选
            if (!is_white(mt9v03x_image[ny1][nx1])
                && is_white(mt9v03x_image[ny2][nx2]))
            {
                if (ny1 < best_y) { best_y = ny1; best_x = nx1; }
            }
        }

        if (best_y != 999)
        {
            cx = best_x;
            cy = best_y;
        }
        else if (cy > 0)
        {
            // ---- 断点续搜：往上扫5行 ----
            int found = 0;
            for (int retry = 0; retry < 5 && cy > 0; retry++)
            {
                cy--;  // 上移一行
                // 从中间向左扫，找赛道左边缘（白→黑）+ 3连白
                for (int x = pho_center_x; x >= 2; x--)
                {
                    if (!is_white(mt9v03x_image[cy][x - 1])
                        && is_white(mt9v03x_image[cy][x])
                        && is_white(mt9v03x_image[cy][x + 1])
                        && is_white(mt9v03x_image[cy][x + 2]))
                    {
                        cx = x;     //种子更新
                        found = 1;
                        break;
                    }
                }
                if (found) break;
            }
            if (!found) break;  //3行全失败 终止
        }

        if (cy <= 40) break;  // 爬到采样区顶部，不再往上
    }

    data_l = idx;
}

/* ---右边界生长--- */
static void seed_grow_right(int rx, int ry)
{
    //顺时针方向表
    static const int8 dir_r[8][2] = {
        { 0, 1}, { 1, 1}, { 1, 0}, { 1,-1}, { 0,-1}, {-1,-1}, {-1, 0}, {-1, 1}
    };

    int cx = rx, cy = ry;   // 当前种子坐标（黑像素）
    int idx = 0;            // 已存种子数

    for (int iter = 0; iter < SEED_MAX_POINTS; iter++)
    {
        //存入当前种子
        points_r[idx][0] = cx;
        points_r[idx][1] = cy;
        idx++;

        int best_y = 999, best_x = -1;

        //----八邻域搜索----
        for (int i = 0; i < 8; i++)
        {
            int nx  = cx + dir_r[i][0];              //方向i 应为黑
            int ny  = cy + dir_r[i][1];
            int nx2 = cx + dir_r[(i + 1) & 7][0];    //方向i+1 应为白
            int ny2 = cy + dir_r[(i + 1) & 7][1];

            //防越界
            if (nx < 0 || nx >= pho_w || ny < 0 || ny >= pho_h) continue;
            if (nx2 < 0 || nx2 >= pho_w || ny2 < 0 || ny2 >= pho_h) continue;

            //选y最小（最上）的候选
            if (!is_white(mt9v03x_image[ny][nx])
                && is_white(mt9v03x_image[ny2][nx2]))
            {
                if (ny < best_y) { best_y = ny; best_x = nx; }
            }
        }

        if (best_y != 999)
        {
            cx = best_x;
            cy = best_y;
        }
        else if (cy > 0)
        {
            //----断点续搜 往上扫5行----
            int found = 0;
            for (int retry = 0; retry < 5 && cy > 0; retry++)
            {
                cy--;
                // 从中间向右扫，找赛道右边缘（白→黑）+ 3连白
                for (int x = pho_center_x; x < pho_w - 3; x++)
                {
                    if (is_white(mt9v03x_image[cy][x])
                        && !is_white(mt9v03x_image[cy][x + 1])
                        && is_white(mt9v03x_image[cy][x - 1])
                        && is_white(mt9v03x_image[cy][x - 2]))
                    {
                        cx = x;     //新种子更新
                        found = 1;
                        break;
                    }
                }
                if (found) break;
            }
            if (!found) break;  //3行全失败 终止
        }

        if (cy <= 40) break;  // 爬到采样区顶部，不再往上
    }

    data_r = idx;
}

/* ================================================================
 * 从种子点序列提取按行边界数组
 *
 * 种子点是黑像素（边界外侧），dir 将其转为白像素（赛道内侧）：
 *   dir = +1（左边界）→ 向右移1列
 *   dir = -1（右边界）→ 向左移1列
 *
 * 未生长到的行默认填0
 * 同行多个点时只取第一个
 * ================================================================ */
static void pho_border(uint16 points[][2], uint16 total,
                           border_line border, int dir)
{
    /* 先全部填 0，表示"无数据" */
    for (int y = 0; y < pho_h; y++)
    {
        border[y] = 0;
    }

    /* 按 y 写入，同行只取第一个 */
    int last_y = -1;
    for (int i = 0; i < total; i++)
    {
        int py = points[i][1];
        int px = points[i][0];

        if (py == last_y) continue;
        last_y = py;

        // 种子（黑像素）→ 赛道内侧（白像素）
        int val = px + dir;
        if (val < pho_w_min)    val = pho_w_min;
        if (val > pho_w_max)    val = pho_w_max;
        border[py] = (uint8)val;
    }
}

/* ================================================================
 * 计算中线数组和赛道偏差 err
 *
 * center_line[y] = (l_border[y] + r_border[y]) / 2
 *
 * err 加权平均（100→40 行，每5行采样，远处权重大）：
 *   w = (100-row)/60 + 0.2 → row40=1.2, row100=0.2
 *
 * err>0 赛道偏右 车需右转
 * err<0 赛道偏左 车需左转
 *
 * 跳过任一边界为 0（无数据）的行
 * w_sum == 0（全部无效）→ err 保留上一帧值不变
 * ================================================================ */
static void pho_center(void)
{
    for (int y = pho_h-5; y >= 0; y--)  //底部115行开始画
    {
        center_line[y] = (uint8)((l_border[y] + r_border[y]) / 2);
    }

    float sum = 0, w_sum = 0;
    for (int row = 100; row >= 40; row -= 5)  //近处100行→远处40行
    {
        if (l_border[row] == 0 || r_border[row] == 0)
            continue;  // 无数据行不计入

        float w = (float)(100 - row) / 60.0f + 0.2f;  // row40=1.2, row100=0.2 远处权重大
        sum += ((float)center_line[row] - pho_center_x) * w;
        w_sum += w;
    }

    if (w_sum > 0.0f)
        err = sum / w_sum;
    // else: 全部采样行无效 → err 保留上一帧值，不做更新
}


/* ================================================================
 * 兜底搜线：两边种子都找不到时，逐行扫边界（与bottom_scan同款逻辑）
 *
 * 扫描 100→40 行，找每行左右边界，填 l_border/r_border
 * 都不做质心——质心容易被噪声拉偏，且 l=r 不画红线
 *
 * return: 1 有有效数据, 0 全部无效 → 真正丢线
 * ================================================================ */
static int fallback_scan(void)
{
    for (int y = 0; y < pho_h; y++)
    {
        l_border[y] = 0;
        r_border[y] = 0;
    }

    int valid = 0;

    for (int y = 100; y >= 40; y--)
    {
        int lx = -1, rx = -1;
        int m = pho_center_x;

        if (!is_white(mt9v03x_image[y][m])) continue;

        // 左边界：从中间向左扫，找赛道左边缘
        for (int x = m; x >= 1; x--)
        {
            if (!is_white(mt9v03x_image[y][x - 1])  // 白→黑转跳
                && is_white(mt9v03x_image[y][x])
                && is_white(mt9v03x_image[y][x + 1]))
            {
                lx = x + 1; break;
            }
        }
        if (lx < 0) lx = pho_w_min;

        // 右边界：从中间向右扫
        for (int x = m; x < pho_w - 2; x++)
        {
            if (is_white(mt9v03x_image[y][x])
                && !is_white(mt9v03x_image[y][x + 1])
                && is_white(mt9v03x_image[y][x - 1]))
            {
                rx = x - 1; break;
            }
        }
        if (rx < 0) rx = pho_w_max;

        l_border[y] = (uint8)lx;
        r_border[y] = (uint8)rx;
        valid = 1;
    }

    return valid;
}


/* ================================================================
 * 赛道半宽：直道标定，线性插值查表
 * ================================================================ */
static const int hw_y[9] = {100, 90, 80, 70, 60, 50, 40, 30, 20};
static const int hw_v[9] = { 92, 89, 84, 74, 63, 52, 42, 31, 20};

static int get_half_width(int y)
{
    if (y >= hw_y[0]) return hw_v[0];          // >=100
    if (y <= hw_y[8]) return hw_v[8];          // <=20
    for (int i = 0; i < 8; i++)
    {
        if (y <= hw_y[i] && y >= hw_y[i+1])
        {
            int dy = hw_y[i] - hw_y[i+1];
            int dv = hw_v[i] - hw_v[i+1];
            return hw_v[i] - dv * (hw_y[i] - y) / dy;
        }
    }
    return 50;  // 不会到这里
}

/* ================================================================
 * 断点补偿：种子生长中断在 79~20 行范围内时
 *   l有r缺 → 右→左扫白→黑找右边界
 *   r有l缺 → 左→右扫黑→白找左边界
 *   都缺 → 质心兜底
 * ================================================================ */
static void fill_gaps(void)
{
    for (int y = 79; y >= 40; y--)
    {
        if (l_border[y] != 0 && r_border[y] != 0) continue;

        // 只有左边界 → 找右边界
        if (l_border[y] != 0 && r_border[y] == 0)
        {
            for (int x = pho_center_x; x < pho_w - 2; x++)  // 从中向右扫
            {
                if (is_white(mt9v03x_image[y][x])
                    && !is_white(mt9v03x_image[y][x + 1])
                    && is_white(mt9v03x_image[y][x - 1]))
                {
                    r_border[y] = (uint8)(x - 1); break;
                }
            }
            // 扫不到且边缘全白 → 边界在画外，半宽推算
            if (r_border[y] == 0 && is_white(mt9v03x_image[y][pho_w_max]))
            {
                int rv = (int)l_border[y] + 2 * get_half_width(y);
                r_border[y] = (uint8)(rv > pho_w_max ? pho_w_max : rv);
            }
            else if (r_border[y] == 0)
                r_border[y] = pho_w_max;  // 找不到也不是全白 → 兜底
        }

        // 只有右边界 → 找左边界
        if (r_border[y] != 0 && l_border[y] == 0)
        {
            for (int x = pho_center_x; x >= 1; x--)  // 从中向左扫
            {
                if (!is_white(mt9v03x_image[y][x - 1])
                    && is_white(mt9v03x_image[y][x])
                    && is_white(mt9v03x_image[y][x + 1]))
                {
                    l_border[y] = (uint8)(x + 1); break;
                }
            }
            // 扫不到且边缘全白 → 边界在画外，半宽推算
            if (l_border[y] == 0 && is_white(mt9v03x_image[y][0]))
            {
                int lv = (int)r_border[y] - 2 * get_half_width(y);
                l_border[y] = (uint8)(lv < pho_w_min ? pho_w_min : lv);
            }
            else if (l_border[y] == 0)
                l_border[y] = pho_w_min;  // 找不到也不是全白 → 兜底
        }

        // 两边都缺 → 质心兜底
        if (l_border[y] == 0 && r_border[y] == 0)
        {
            int sum = 0, cnt = 0;
            for (int x = 0; x < pho_w; x++)
            {
                if (is_white(mt9v03x_image[y][x]))
                {
                    sum += x;
                    cnt++;
                }
            }
            if (cnt >= 3)
            {
                uint8_t c = (uint8_t)(sum / cnt);
                l_border[y] = c;
                r_border[y] = c;
            }
        }
    }
}


/* ================================================================
 * 搜线主函数
 *
 * 正常：find_seeds(90→40) → seed_grow → pho_border → fill_gaps(79→40) → bottom_scan(118→79) → pho_center(100→40)
 * 兜底：两边都没种子 → fallback_scan(100→40) → bottom_scan(118→79) → pho_center
 *
 * return: 0 有数据, 1 完全丢线
 * ================================================================ */
/* ================================================================
 * 手遮摄像头检测：底部5行+顶部5行全黑 → 认为摄像头被遮住
 * ================================================================ */
static int camera_is_covered(void)
{
    // 底部 118→110（9行）
    for (int y = 118; y >= 110; y--)
        for (int x = 0; x < pho_w; x++)
            if (is_white(mt9v03x_image[y][x])) return 0;
    // 顶部 10→0（11行）
    for (int y = 10; y >= 0; y--)
        for (int x = 0; x < pho_w; x++)
            if (is_white(mt9v03x_image[y][x])) return 0;
    return 1;  // 全是黑的 → 遮住了
}

int vis_deal(void)
{
    int llx, lly, rrx, rry;
    data_l = 0; data_r = 0;

    /* 手遮摄像头 → 立即停车 */
    if (camera_is_covered())
    {
        motor_stop();
        car_run = false;
        return 1;
    }

    /* 清零边界数组（pho_border只清自己那边，这边统一清） */
    for (int y = 0; y < pho_h; y++)
    {
        l_border[y] = 0;
        r_border[y] = 0;
    }

    /* 第一步：90→40独立找左右种子 */
    find_seeds(&llx, &lly, &rrx, &rry);

    /* 第二步：各自独立生长（一边找到就长一边） */
    if (llx >= 0)
    {
        seed_grow_left(llx, lly);
        pho_border(points_l, data_l, l_border, +1);
    }
    if (rrx >= 0)
    {
        seed_grow_right(rrx, rry);
        pho_border(points_r, data_r, r_border, -1);
    }

    /* 第三步：补全边界（缺哪边找哪边）或兜底 */
    if (llx >= 0 || rrx >= 0)
    {
        fill_gaps();  // 有一边就找另一边，都缺才质心
    }
    else if (!fallback_scan())
    {
        return 1;  // 完全丢线
    }

    /* 第四步：底部40行逐行扫描（最后执行，覆写118→79，不被fallback清掉） */
    bottom_scan();

    /* 汇聚：计算中线和偏差 */
    pho_center();
    return 0;
}

/* ================================================================
 * 显示搜线结果
 *
 * 灰度原图上叠加：
 *   红色 左右边界
 *   绿色 中线
 *
 * 跳过 border==0（无数据）的行不绘制
 * 跳过底部5行（太近，无参考价值）
 * ================================================================ */
void vis_draw(void)
{
    ips200_show_gray_image(0, BIN_PARAM_H, (const uint8 *)mt9v03x_image,
                       pho_w, pho_h, pho_w, pho_h, 0);

    for (int y = 0; y <= 115; y++)  //从底部115行向上画
    {
        uint8 l = l_border[y];
        uint8 r = r_border[y];
        uint8 c = center_line[y];
        int dy = y + BIN_PARAM_H;          // 图像放到参数区域下方

        if (c == 0) continue;              // center_line=0 → 完全无数据，跳过

        // 红线：左右边界都有且不同才画
        if (l != 0 && r != 0 && l != r)
        {
            ips200_draw_point(l, dy, RGB565_RED);
            if (l < pho_w_max)  ips200_draw_point(l + 1, dy, RGB565_RED);

            ips200_draw_point(r, dy, RGB565_RED);
            if (r > pho_w_min)  ips200_draw_point(r - 1, dy, RGB565_RED);
        }

        // 绿线：有center_line数据就画
        ips200_draw_point(c, dy, RGB565_GREEN);
        if (c < pho_w_max)  ips200_draw_point(c + 1, dy, RGB565_GREEN);
    }
}


/* ================================================================
 * 双阈值二值化显示（BIN 模式）
 * 0=黑, 255=白，用于确认阈值是否把赛道和背景正确分开
 * ================================================================ */
void vis_bin_draw(void)
{
    for (int y = 0; y < pho_h; y++)
        for (int x = 0; x < pho_w; x++)
        {
            g_bin_image[y][x] = is_white(mt9v03x_image[y][x]) ? 255 : 0;
        }


    ips200_show_gray_image(0, BIN_PARAM_H, (const uint8 *)g_bin_image,
                       pho_w, pho_h, pho_w, pho_h, 0);
}
