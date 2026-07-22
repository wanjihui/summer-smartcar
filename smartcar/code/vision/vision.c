#include "config.h"

uint8  g_bin_image[pho_h][pho_w];

 border_line l_border;
 border_line r_border;
 border_line center_line;

 volatile float err;
 volatile uint8_t vis_frame_ready;  // 新帧已处理标志，main设置，menu消费后清零

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
 * 图像底部逐行向上找左右种子
 * 从 y=118 向上扫到 y=112（共7行）
 * 找到左右种子都存在的第一行就返回
 * return: 1 找到, 0 全失败 → 进入兜底搜线
 * ================================================================ */
static int find_seeds(int *seed_l_x, int *seed_r_x, int *seed_y)
{
    for (int y = 118; y >= 112; y--)  //扫描118→112共7行
    {
        // 左种子：从左往右扫，黑→白跳变 + 3连白确认
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

        // 右种子：从右往左扫，白→黑跳变 + 3连白确认
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

        if (*seed_l_x >= 0 && *seed_r_x >= 0)
        {
            *seed_y = y;
            return 1;  // 当前行两个种子都找到
        }
    }
    return 0;  // 7行全失败 → 进入兜底搜线
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
 *   8方向全失败 → 连续往上扫3行
 *   左：整行左→右扫描，找黑→白跳变 + 3连白确认
 *   右：整行右→左扫描，找白→黑跳变 + 3连白确认
 *   找到 → 续上继续正常生长
 *   3行全失败 → 终止该侧搜索
 *
 * 终止条件：
 *   爬到顶 (cy == 0)
 *   3行重试全失败
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
            // ---- 断点续搜：往上扫3行 ----
            int found = 0;
            for (int retry = 0; retry < 3 && cy > 0; retry++)
            {
                cy--;  // 上移一行
                // 整行左→右扫描，找黑→白跳变 + 3连白确认
                for (int x = 1; x < pho_w - 3; x++)
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

        if (cy == 0) break;  // 爬到图像顶部
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
            //----断点续搜 往上扫3行----
            int found = 0;
            for (int retry = 0; retry < 3 && cy > 0; retry++)
            {
                cy--;
                // 整行右→左扫描，找白→黑跳变 + 3连白确认
                for (int x = pho_w - 2; x >= 2; x--)
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

        if (cy == 0) break;  //爬到图像顶部
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
 * err 加权平均（110→70 行，每5行采样，从近到远）：
 *   线性加权 w = (row - 65) / 45 → row70 权重≈0.11, row110 权重=1.0
 *   近处权重大，远处权重小
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
    for (int row = 110; row >= 70; row -= 5)  //近处110行→远处70行
    {
        if (l_border[row] == 0 || r_border[row] == 0)
            continue;  // 无数据行不计入

        float w = (float)(row - 65) / 45.0f;  // row70≈0.11, row110=1.0
        sum += ((float)center_line[row] - pho_center_x) * w;
        w_sum += w;
    }

    if (w_sum > 0.0f)
        err = sum / w_sum;
    // else: 全部采样行无效 → err 保留上一帧值，不做更新
}


/* ================================================================
 * 兜底搜线：种子失败时白像素质心法直接算中线
 *
 * 扫描 110→70 行，每行统计白像素 x 坐标平均值 → center_line
 * 设 l_border = r_border = center_line（使 pho_center 的 (l+r)/2
 * 原样输出质心值，err 计算和 vis_draw 绿线自然纳入）
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

    for (int y = 110; y >= 70; y--)
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
            uint8_t c = (uint8_t)(sum / cnt);       // 白像素质心
            center_line[y] = c;                      // 直接存中线
            l_border[y]   = c;                       // pho_center: (c+c)/2=c
            r_border[y]   = c;                       // 同时标记该行有效（≠0）
            valid = 1;
        }
    }

    return valid;
}


/* ================================================================
 * 断点补偿：种子生长中断在 110~70 行范围内时，白像素质心法补全
 *
 * 已有数据的行（l≠0 且 r≠0）保持不变
 * 缺失行用质心法填 center_line，l_border = r_border = 质心
 * ================================================================ */
static void fill_gaps(void)
{
    for (int y = 110; y >= 70; y--)
    {
        if (l_border[y] != 0 && r_border[y] != 0) continue;  // 已有数据

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
            center_line[y] = c;
            l_border[y]   = c;
            r_border[y]   = c;
        }
    }
}


/* ================================================================
 * 搜线主函数
 *
 * 正常路径：find_seeds → seed_grow ×2 → pho_border ×2 → fill_gaps_110_70 → pho_center
 * 兜底路径：find_seeds 失败 → fallback_scan → pho_center
 *
 * return: 0 有数据, 1 完全丢线
 * ================================================================ */
int vis_deal(void)
{
    int lx, rx, seed_y;

    if (find_seeds(&lx, &rx, &seed_y))
    {
        /* 正常路径：八邻域种子生长 */
        data_l = 0; data_r = 0;
        seed_grow_left(lx, seed_y);
        seed_grow_right(rx, seed_y);
        pho_border(points_l, data_l, l_border, +1);
        pho_border(points_r, data_r, r_border, -1);
        fill_gaps();  // 补全种子生长断掉的行
    }
    else if (!fallback_scan())
    {
        /* 兜底也失败 → 真正丢线 */
        return 1;
    }
    /* else: 兜底成功 → l_border/r_border 已填充，继续算 err */

    /* 两条路径汇聚：计算中线和偏差 */
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

        if (l == 0 || r == 0) continue;    // 无数据行不绘制

        if (l != r)  // 八邻域数据：画左右边界红线
        {
            ips200_draw_point(l, dy, RGB565_RED);
            if (l < pho_w_max)  ips200_draw_point(l + 1, dy, RGB565_RED);

            ips200_draw_point(r, dy, RGB565_RED);
            if (r > pho_w_min)  ips200_draw_point(r - 1, dy, RGB565_RED);
        }

        // 中线绿线（八邻域和质心法都画）
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
