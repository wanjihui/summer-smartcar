
#include "config.h"

 border_line l_border;                      // 左边界，每行一个列号
 border_line r_border;                      // 右边界
 border_line center_line;                   // 中线（边框平滑 + 中线平滑后，绘图和err共用）

 volatile float err;                        // 中线偏离图像中心的像素均值，>0偏右
 volatile uint8_t vis_frame_ready;          // 新帧处理完成，显示层可刷新

 uint8 lookahead = DEFAULT_LOOKAHEAD;       // 预瞄行数（从行100向上偏移），菜单可调

/* ================================================================
 * 局部窗口参数（菜单可调）
 *   Block_Size: 局部窗口边长（奇数），默认 9×9
 *   Clip_Value: 阈值 = 窗口均值 - 偏置，越大越严格
 * ================================================================ */
int32_t Block_Size = 9;
int32_t Clip_Value = 4;

/* ================================================================
 * Otsu 二值化图像缓存
 *   0=黑（背景）, 255=白（赛道）
 * ================================================================ */
uint8 Image_Used[pho_h][pho_w];

/* ================================================================
 * 4 方向行进表（移植自 MCZSCS）
 *   dir=0→上, 1→右, 2→下, 3→左
 *   dir_front[dir]:      直行方向
 *   dir_frontleft[dir]:  左手前（左巡线用）
 *   dir_frontright[dir]: 右手前（右巡线用）
 * ================================================================ */
static const int dir_front[4][2]      = {{ 0,-1}, { 1, 0}, { 0, 1}, {-1, 0}};
static const int dir_frontleft[4][2]  = {{-1,-1}, { 1,-1}, { 1, 1}, {-1, 1}};
static const int dir_frontright[4][2] = {{ 1,-1}, { 1, 1}, {-1, 1}, {-1,-1}};

/* ================================================================
 * 滑动窗口增量更新 O(n) — 移植自 MCZSCS Image.c updata_sum()
 * ================================================================ */
static int updata_sum(int sum, int x, int y, int half, int dirx, int diry)
{
    uint8 flag = 0;
    if (diry != 0)
    {
        if (diry > 0)
        {
            for (int dx = -half; dx <= half; ++dx)
            { sum -= AT(x + dx, y - half); sum += AT(x + dx, y + half + 1); }
        }
        else
        {
            for (int dx = -half; dx <= half; ++dx)
            { sum -= AT(x + dx, y + half); sum += AT(x + dx, y - half - 1); }
        }
        flag = 1;
    }
    if (dirx != 0)
    {
        if (flag) y += diry;
        if (dirx > 0)
        {
            for (int dy = -half; dy <= half; ++dy)
            { sum -= AT(x - half, y + dy); sum += AT(x + half + 1, y + dy); }
        }
        else
        {
            for (int dy = -half; dy <= half; ++dy)
            { sum -= AT(x + half, y + dy); sum += AT(x - half - 1, y + dy); }
        }
    }
    return sum;
}

/* ================================================================
 * 大津法 (Otsu) — 下采样 4x，安全钳 20~235
 * 移植自 loongson_learn，double → float
 * ================================================================ */
static int compute_otsu(void)
{
    unsigned long hist[256] = {0};
    for (int y = 0; y < pho_h; y += 2)
        for (int x = 0; x < pho_w; x += 2)
            hist[mt9v03x_image[y][x]]++;

    unsigned long total = (unsigned long)((pho_h + 1) / 2) * ((pho_w + 1) / 2);
    if (total == 0) return -1;

    unsigned long sum_all = 0;
    for (int i = 0; i < 256; i++) sum_all += (unsigned long)i * hist[i];

    unsigned long wB = 0, sumB = 0;
    float max_between = -1.0f;
    int best_th = 0;

    for (int t = 0; t < 256; t++)
    {
        wB += hist[t];
        if (wB == 0) continue;
        unsigned long wF = total - wB;
        if (wF == 0) break;
        sumB += (unsigned long)t * hist[t];
        float mB = (float)sumB / wB;
        float mF = (float)(sum_all - sumB) / wF;
        float diff = mB - mF;
        float between = (float)wB * (float)wF * diff * diff;
        if (between > max_between) { max_between = between; best_th = t; }
    }
    if (best_th < 20)  best_th = 20;
    if (best_th > 235) best_th = 235;
    return best_th;
}

/* ================================================================
 * find_seeds — 从图像底部向上全屏扫描找左右种子
 *
 * 基于二值图 Image_Used（0=黑/背景，255=白/赛道）
 * 左右各自从边缘向对侧全屏扫描，找 2连黑→3连白 = 赛道边界
 * 与保底扫描、bottom_scan 逻辑统一
 *
 * 调参: 无独立参数，依赖 Otsu 阈值质量
 * 返回: 左右种子坐标，(llx,lly) 和 (rrx,rry)，-1 表示未找到
 * ================================================================ */
static void find_seeds(int *llx, int *lly, int *rrx, int *rry)
{
    *llx = -1; *rrx = -1;

    for (int y = 115; y >= 70; y--)
    {
        /* 左种子：从左边缘全屏扫，找 2黑→3白 */
        if (*llx < 0)
        {
            for (int x = 2; x < pho_w - 2; x++)
            {
                if (Image_Used[y][x - 2] == 0
                    && Image_Used[y][x - 1] == 0
                    && Image_Used[y][x] == 255
                    && Image_Used[y][x + 1] == 255
                    && Image_Used[y][x + 2] == 255)
                { *llx = x; *lly = y; break; }
            }
        }

        /* 右种子：从右边缘全屏扫，找 2黑→3白 */
        if (*rrx < 0)
        {
            for (int x = pho_w_max - 2; x >= 2; x--)
            {
                if (Image_Used[y][x + 2] == 0
                    && Image_Used[y][x + 1] == 0
                    && Image_Used[y][x] == 255
                    && Image_Used[y][x - 1] == 255
                    && Image_Used[y][x - 2] == 255)
                { *rrx = x; *rry = y; break; }
            }
        }

        if (*llx >= 0 && *rrx >= 0) break;
    }
}
/* ================================================================
 * 底部扫描：逐行扫 118→(种子行+1)，直接填 l_border/r_border
 * 种子行以下无种子覆盖，逐行扫更快更准
 * ================================================================ */
static void bottom_scan(int seed_y)
{
    int end = seed_y + 1;
    if (end < 41) end = 41;
    for (int y = 118; y >= end; y--)
    {
        // 找该行最长连续白段
        int best_s = -1, best_e = -1, best_len = 0;
        int run_s = -1;
        for (int x = 0; x < pho_w; x++)
        {
            if (Image_Used[y][x] == 255)
            {
                if (run_s < 0) run_s = x;
            }
            else
            {
                if (run_s >= 0)
                {
                    int len = x - run_s;
                    if (len > best_len) { best_len = len; best_s = run_s; best_e = x - 1; }
                    run_s = -1;
                }
            }
        }
        if (run_s >= 0)
        {
            int len = pho_w - run_s;
            if (len > best_len) { best_len = len; best_s = run_s; best_e = pho_w - 1; }
        }

        if (best_len >= 3)
        {
            l_border[y] = (uint8)best_s;
            r_border[y] = (uint8)best_e;
        }
        else
        {
            l_border[y] = BORDER_INVALID;
            r_border[y] = BORDER_INVALID;
        }
    }
}

/* ================================================================
 * 种子生长 — 4方向摸墙 + 局部自适应阈值
 *
 * 移植自 MCZSCS findline_adaptive，适配无IPM像素空间
 * 每步只看2个方向（前方+侧前方），3种动作（直走/转向前进/原地转）
 * 局部9×9滑动窗口阈值自动适应远近亮度变化
 *
 * 保底: 连续转4次没前进 → 逐行从边缘向中心重扫找新起点
 * 调参: Block_Size(窗口,默认9), Clip_Value(偏置,默认4)
 * ================================================================ */

uint16 points_l[SEED_MAX_POINTS][2];  // 左边界种子坐标存储
uint16 points_r[SEED_MAX_POINTS][2];  // 右边界种子坐标存储
uint16 data_l, data_r;                // 左右各自实际存储点数

/* ---左边界生长（4方向左手摸墙 + 局部自适应阈值 + 断点逐行回退）--- */
static void seed_grow_left(int lx, int ly)
{
    int half = Block_Size >> 1;
    int cx = lx, cy = ly;
    int idx = 0;
    int dir = 0;          // 0=上 1=右 2=下 3=左
    int turn = 0;         // 连续转弯计数
    int last_y = cy;
    int same_row_cnt = 1;

    /* 初始化局部窗口和 */
    int local_sum = 0;
    for (int dy = -half; dy <= half; dy++)
        for (int dx = -half; dx <= half; dx++)
            local_sum += AT(cx + dx, cy + dy);
    int local_thres = local_sum / (Block_Size * Block_Size) - Clip_Value;

    while (idx < MAX_TRACK_POINTS
           && cx > 0 && cx < pho_w - 1
           && cy > 1 && cy < pho_h - 1)
    {
        /* 存入当前点 */
        points_l[idx][0] = cx;
        points_l[idx][1] = cy;
        idx++;

        /* 同行连续5点 → 振荡，停止 */
        if (cy == last_y)
        {
            same_row_cnt++;
            if (same_row_cnt >= 5) break;
        }
        else { same_row_cnt = 1; last_y = cy; }

        int front     = AT(cx + dir_front[dir][0],     cy + dir_front[dir][1]);
        int frontleft = AT(cx + dir_frontleft[dir][0], cy + dir_frontleft[dir][1]);

        if (front < local_thres)
        {
            /* 前方是黑 → 右转贴墙 */
            dir = (dir + 1) & 3;
            turn++;
            if (turn >= 4)
            {
                /* ======== 保底：逐行从边缘→中心扫描（黑→白跳变） ======== */
                int found = 0;
                for (int ry = cy - 1; ry > 20 && !found; ry--)
                {
                    for (int x = 2; x < pho_center_x; x++)
                    {
                        if (Image_Used[ry][x - 2] == 0
                            && Image_Used[ry][x - 1] == 0
                            && Image_Used[ry][x] == 255
                            && Image_Used[ry][x + 1] == 255
                            && Image_Used[ry][x + 2] == 255)
                        {
                            cx = x; cy = ry; dir = 0; turn = 0;
                            local_sum = 0;
                            for (int dy = -half; dy <= half; dy++)
                                for (int dx = -half; dx <= half; dx++)
                                    local_sum += AT(cx + dx, cy + dy);
                            local_thres = local_sum
                                        / (Block_Size * Block_Size) - Clip_Value;
                            found = 1;
                            break;
                        }
                    }
                }
                if (!found) break;  // 所有行都扫不到 → 真丢了
            }
        }
        else if (frontleft < local_thres)
        {
            /* 左前方是黑 → 直走 */
            local_sum = updata_sum(local_sum, cx, cy, half,
                                   dir_front[dir][0], dir_front[dir][1]);
            cx += dir_front[dir][0];
            cy += dir_front[dir][1];
            turn = 0;
            local_thres = local_sum / (Block_Size * Block_Size) - Clip_Value;
        }
        else
        {
            /* 前方和左前方都是白 → 左转靠墙 */
            local_sum = updata_sum(local_sum, cx, cy, half,
                                   dir_frontleft[dir][0], dir_frontleft[dir][1]);
            cx += dir_frontleft[dir][0];
            cy += dir_frontleft[dir][1];
            dir = (dir + 3) & 3;       // 等价于 dir-1
            turn = 0;
            local_thres = local_sum / (Block_Size * Block_Size) - Clip_Value;
        }

        if (cy < 20) break;                                     // 爬到顶部
        if (cy < 70 && cx >= pho_w_max - 3) break;             // 上半屏左边界逼近右边缘，异常
    }

    data_l = idx;
}

/* ---右边界生长（4方向右手摸墙 + 局部自适应阈值 + 断点逐行回退）--- */
static void seed_grow_right(int rx, int ry)
{
    int half = Block_Size >> 1;
    int cx = rx, cy = ry;
    int idx = 0;
    int dir = 0;          // 0=上 1=右 2=下 3=左
    int turn = 0;
    int last_y = cy;
    int same_row_cnt = 1;

    /* 初始化局部窗口和 */
    int local_sum = 0;
    for (int dy = -half; dy <= half; dy++)
        for (int dx = -half; dx <= half; dx++)
            local_sum += AT(cx + dx, cy + dy);
    int local_thres = local_sum / (Block_Size * Block_Size) - Clip_Value;

    while (idx < MAX_TRACK_POINTS
           && cx > 0 && cx < pho_w - 1
           && cy > 1 && cy < pho_h - 1)
    {
        /* 存入当前点 */
        points_r[idx][0] = cx;
        points_r[idx][1] = cy;
        idx++;

        /* 同行连续5点 → 振荡，停止 */
        if (cy == last_y)
        {
            same_row_cnt++;
            if (same_row_cnt >= 5) break;
        }
        else { same_row_cnt = 1; last_y = cy; }

        int front      = AT(cx + dir_front[dir][0],      cy + dir_front[dir][1]);
        int frontright = AT(cx + dir_frontright[dir][0], cy + dir_frontright[dir][1]);

        if (front < local_thres)
        {
            /* 前方是黑 → 左转贴墙 */
            dir = (dir + 3) & 3;       // 等价于 dir-1
            turn++;
            if (turn >= 4)
            {
                /* ======== 保底：逐行从边缘→中心扫描（黑→白跳变） ======== */
                int found = 0;
                for (int ry_idx = cy - 1; ry_idx > 40 && !found; ry_idx--)
                {
                    for (int x = pho_w_max - 2; x > pho_center_x; x--)
                    {
                        if (Image_Used[ry_idx][x + 2] == 0
                            && Image_Used[ry_idx][x + 1] == 0
                            && Image_Used[ry_idx][x] == 255
                            && Image_Used[ry_idx][x - 1] == 255
                            && Image_Used[ry_idx][x - 2] == 255)
                        {
                            cx = x; cy = ry_idx; dir = 0; turn = 0;
                            local_sum = 0;
                            for (int dy = -half; dy <= half; dy++)
                                for (int dx = -half; dx <= half; dx++)
                                    local_sum += AT(cx + dx, cy + dy);
                            local_thres = local_sum
                                        / (Block_Size * Block_Size) - Clip_Value;
                            found = 1;
                            break;
                        }
                    }
                }
                if (!found) break;
            }
        }
        else if (frontright < local_thres)
        {
            /* 右前方是黑 → 直走 */
            local_sum = updata_sum(local_sum, cx, cy, half,
                                   dir_front[dir][0], dir_front[dir][1]);
            cx += dir_front[dir][0];
            cy += dir_front[dir][1];
            turn = 0;
            local_thres = local_sum / (Block_Size * Block_Size) - Clip_Value;
        }
        else
        {
            /* 前方和右前方都是白 → 右转靠墙 */
            local_sum = updata_sum(local_sum, cx, cy, half,
                                   dir_frontright[dir][0], dir_frontright[dir][1]);
            cx += dir_frontright[dir][0];
            cy += dir_frontright[dir][1];
            dir = (dir + 1) & 3;       // 右转
            turn = 0;
            local_thres = local_sum / (Block_Size * Block_Size) - Clip_Value;
        }

        if (cy < 20) break;
        if (cy < 70 && cx <= pho_w_min + 3) break;    // 上半屏右边界逼近左边缘，异常
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
 * 未生长到的行默认填 BORDER_INVALID（无数据标记）
 * 同行多个点取中位数（抗野值优于均值）
 * ================================================================ */
static uint8 row_xs[SEED_MAX_POINTS];  // 同行x值排序缓存（可重用）

static void pho_border(uint16 points[][2], uint16 total,
                           border_line border, int dir)
{
    /* 先全部填 BORDER_INVALID，表示"无数据" */
    for (int y = 0; y < pho_h; y++)
        border[y] = BORDER_INVALID;

    if (total == 0) return;

    /* 按 y 分组，同行多点取中位数再偏移 */
    int cur_y  = points[0][1];
    int cnt = 0;

    for (int i = 0; i < total; i++)
    {
        int py = points[i][1];
        int px = points[i][0];

        if (py != cur_y)
        {
            // 上一行结算：排序 → 中位数 → 偏移 → 写入
            if (cnt > 0)
            {
                for (int a = 0; a < cnt - 1; a++)
                    for (int b = a + 1; b < cnt; b++)
                        if (row_xs[a] > row_xs[b])
                            { uint8 t = row_xs[a]; row_xs[a] = row_xs[b]; row_xs[b] = t; }

                int med = row_xs[(cnt - 1) / 2];  // 下中位数，偏内侧更安全
                int val = med + dir;
                if (val < pho_w_min) val = pho_w_min;
                if (val > pho_w_max) val = pho_w_max;
                border[cur_y] = (uint8)val;
            }

            cur_y = py;
            cnt = 0;
        }
        row_xs[cnt++] = (uint8)px;
    }

    // 最后一组结算
    if (cnt > 0)
    {
        for (int a = 0; a < cnt - 1; a++)
            for (int b = a + 1; b < cnt; b++)
                if (row_xs[a] > row_xs[b])
                    { uint8 t = row_xs[a]; row_xs[a] = row_xs[b]; row_xs[b] = t; }

        int med = row_xs[(cnt - 1) / 2];
        int val = med + dir;
        if (val < pho_w_min) val = pho_w_min;
        if (val > pho_w_max) val = pho_w_max;
        border[cur_y] = (uint8)val;
    }
}


/* ================================================================
 * 边界3点滑动平均：消除单行噪声，保持趋势
 * 只平滑非零连续段，跳过缺失行
 * ================================================================ */
static void border_smooth(border_line border)
{
    for (int y = 1; y < pho_h - 1; y++)
    {
        uint8 l = border[y - 1];
        uint8 m = border[y];
        uint8 r = border[y + 1];
        if (l == BORDER_INVALID || m == BORDER_INVALID || r == BORDER_INVALID) continue;

        int sum = (int)l + (int)m + (int)r;
        uint8 v = (uint8)((sum + 1) / 3);  // +1 四舍五入

        // 变化超过 8px → 不信任此次平滑，保留原值
        int diff = (int)v - (int)m;
        if (diff < 0) diff = -diff;
        if (diff > 8) continue;  // 跳变太大，跳过不改

        border[y] = v;
    }
}

/* ================================================================
 * pho_center — 中线生成 + 偏差计算
 *
 * 1. border_smooth(l_border/r_border) — 平滑左右边界
 * 2. center_line[]: 双边→中点, 单边→中点到屏幕边, 都缺→白像素重心
 * 3. border_smooth(center_line) — 平滑中线
 * 4. 在 y_look±10 窗口内收集 center_line≠255 的行 → 自适应步长采样 → 均值 = err
 *
 * 调参: lookahead(预瞄行,默认30); 单边→白像素重心
 * ================================================================ */
static int candidates[pho_h];  // 候选行缓存（可重用）

/* ================================================================
 * 丢线检测：检查边界是否跑到了图像边缘（丢线/出界）
 *
 * 左丢线: 边界 == 0（图像最左列）→ 赛道偏出左侧视野
 * 右丢线: 边界 == pho_w_max（图像最右列）→ 赛道偏出右侧视野
 *
 * 返回 -1 = 未丢线，其他值 = 丢线起始行号
 * y=50→10 是从中段向上扫，避开近处噪声
 * ================================================================ */
int lost_line_left(void)
{
    for (int y = 50; y > 10; y--)
        if (l_border[y] == 0) return y;
    return -1;
}

int lost_line_right(void)
{
    for (int y = 50; y > 10; y--)
        if (r_border[y] == pho_w_max) return y;
    return -1;
}

/* ================================================================
 * 直道判定：取左右边界中段，检查是否接近直线
 *
 * 两步判断：
 *   1. 首尾连线斜率是否接近 0（边界竖直 = 直道）
 *      kk ∈ (-0.5, 0.5) — 50行内横向偏移 <25px 视为直道
 *   2. 中间每点偏离首尾连线的距离 < 8px
 *      任一采样点超标 → 弯道
 *
 * y_start=100, y_end=50：中段区域，避开近处车轮和远处压缩
 * 返回: 1=直道  0=弯道
 * ================================================================ */
int is_straight(void)
{
    int y_start = 100, y_end = 50, step = 5;

    /* 两端必须有效 */
    if (l_border[y_start] == BORDER_INVALID || l_border[y_end] == BORDER_INVALID)
        return 0;
    if (r_border[y_start] == BORDER_INVALID || r_border[y_end] == BORDER_INVALID)
        return 0;

    /* 左边界斜率 */
    float kl = (float)(l_border[y_end] - l_border[y_start])
             / (float)(y_end - y_start);
    if (kl > 0.5f || kl < -0.5f) return 0;

    /* 右边界斜率 */
    float kr = (float)(r_border[y_end] - r_border[y_start])
             / (float)(y_end - y_start);
    if (kr > 0.5f || kr < -0.5f) return 0;

    /* 中间点偏离检查（左右各至少6个有效点） */
    int okL = 0, okR = 0;
    for (int y = y_start; y >= y_end; y -= step)
    {
        if (l_border[y] != BORDER_INVALID)
        {
            float expectedL = (float)l_border[y_start]
                            + kl * (float)(y - y_start);
            float diff = (float)l_border[y] - expectedL;
            if (diff < 0) diff = -diff;
            if (diff > 8.0f) return 0;
            okL++;
        }
        if (r_border[y] != BORDER_INVALID)
        {
            float expectedR = (float)r_border[y_start]
                            + kr * (float)(y - y_start);
            float diff = (float)r_border[y] - expectedR;
            if (diff < 0) diff = -diff;
            if (diff > 8.0f) return 0;
            okR++;
        }
    }

    return (okL >= 6 && okR >= 6) ? 1 : 0;
}

static void pho_center(void)
{
    // ---- 平滑边界 ----
    border_smooth(l_border);
    border_smooth(r_border);

    int y_look = 100 - (int)lookahead;
    if (y_look < 0) y_look = 0;
    if (y_look >= pho_h) y_look = pho_h - 1;

    // ---- 生成 center_line[]（显示 + err 共用） ----
    for (int y = pho_h - 1; y >= 0; y--)
    {
        int l = (int)l_border[y];
        int r = (int)r_border[y];
        int c;

        if (l != BORDER_INVALID && r != BORDER_INVALID)
        {
            c = (l + r) / 2;
        }
        else if (l != BORDER_INVALID)
        {
            c = (l + pho_w_max) / 2;        // 只有左 → 中点到右屏幕边
        }
        else if (r != BORDER_INVALID)
        {
            c = r / 2;                       // 只有右 → 中点到左屏幕边
        }
        else
        {
            // 两边都缺 → 白像素重心兜底
            int sum = 0, cnt = 0;
            for (int x = 0; x < pho_w; x++)
            {
                if (Image_Used[y][x] == 255)
                { sum += x; cnt++; }
            }
            c = (cnt >= 3) ? (sum + cnt / 2) / cnt : BORDER_INVALID;
        }

        if (c == BORDER_INVALID)
        {
            center_line[y] = BORDER_INVALID;
            continue;
        }

        if (c < pho_w_min) c = pho_w_min;
        if (c > pho_w_max) c = pho_w_max;
        center_line[y] = (uint8)c;
    }

    // ---- 平滑中线（消除半宽偏移可能引入的不连续）----
    border_smooth(center_line);

    // ---- 从中线数组直接采样计算 err ----
    int window_top  = y_look - 10;
    int window_bot  = y_look + 10;
    if (window_top < 0)  window_top = 0;
    if (window_bot >= pho_h) window_bot = pho_h - 1;

    // 收集窗口内中线有效的行
    int cand_cnt = 0;
    for (int y = window_bot; y >= window_top; y--)
    {
        uint8 c = center_line[y];
        if (c == BORDER_INVALID) continue;
        if (c < 5 || c > pho_w_max - 5) continue;   // 中线贴边，不可靠

        candidates[cand_cnt++] = y;
    }

    // 自适应采样率
    int step;
    if (cand_cnt >= 16)           step = 4;
    else if (cand_cnt >= 12)      step = 3;
    else if (cand_cnt >= 8)       step = 2;
    else
    {
        // 不足8行 → 向下扩展
        int expand_y = window_bot + 1;
        while (cand_cnt < 8 && expand_y < pho_h)
        {
            if (center_line[expand_y] != BORDER_INVALID)
                candidates[cand_cnt++] = expand_y;
            expand_y++;
        }
        step = 2;
    }

    // 按步长采样取均值
    int sum = 0, cnt = 0;
    for (int i = 0; i < cand_cnt; i += step)
    {
        int y = candidates[i];
        sum += ((int)center_line[y] - pho_center_x);
        cnt++;
    }

    if (cnt > 0)
        err = (float)sum / (float)cnt;
    // else: err 保留上帧不变
}

/* ================================================================
 * fallback_scan — 种子完全找不到时的最后手段
 *
 * 每行独立扫全行，找最长连续白段 → l=段首, r=段尾
 * len≥3 才写入，不依赖种子/邻域/上一行
 * ================================================================ */
static void fallback_scan(void)
{
    int useful_rows = 0;

    for (int y = 1; y < pho_h - 1; y++)
    {
        // 找该行最长连续白段
        int best_s = -1, best_e = -1, best_len = 0;
        int run_s = -1;
        for (int x = 0; x < pho_w; x++)
        {
            if (Image_Used[y][x] == 255)
            {
                if (run_s < 0) run_s = x;
            }
            else
            {
                if (run_s >= 0)
                {
                    int len = x - run_s;
                    if (len > best_len) { best_len = len; best_s = run_s; best_e = x - 1; }
                    run_s = -1;
                }
            }
        }
        if (run_s >= 0)
        {
            int len = pho_w - run_s;
            if (len > best_len) { best_len = len; best_s = run_s; best_e = pho_w - 1; }
        }

        if (best_len >= 3)
        {
            l_border[y] = (uint8)best_s;
            r_border[y] = (uint8)best_e;
            useful_rows++;
        }
    }

    if (useful_rows > 0)
    {
        border_smooth(l_border);
        border_smooth(r_border);
    }
}


/* ================================================================
 * 手遮摄像头检测：底部+顶部全黑 → 摄像头被遮住
 * 基于二值图 Image_Used（0=黑 255=白）
 * ================================================================ */
static int camera_is_covered(void)
{
    for (int y = 118; y >= 110; y--)
        for (int x = 0; x < pho_w; x++)
            if (Image_Used[y][x] == 255) return 0;
    for (int y = 10; y >= 0; y--)
        for (int x = 0; x < pho_w; x++)
            if (Image_Used[y][x] == 255) return 0;
    return 1;
}

/* ================================================================
 * 搜线主函数
 *
 * 流程：
 *   1. Otsu 自适应阈值 + 一次性二值化 → Image_Used[][]
 *   2. find_seeds(115→70) → 找左右种子（基于二值图）
 *   3. seed_grow_left/right  → 4方向自适应摸墙追踪
 *   4. pho_border → 种子序列提取按行边界
 *   5. bottom_scan / fallback_scan → 补全缺失行
 *   6. pho_center → 拟合中线 + 计算 err
 *
 * 移植自 MCZSCS + loongson_learn，适配无 IPM 的 MM32 平台
 * ================================================================ */
int vis_deal(void)
{
    int llx, lly, rrx, rry;
    data_l = 0; data_r = 0;

    /* ---- Otsu 自适应阈值（每5帧重算一次）---- */
    static int cached_th = -1;
    static int frame_since_otsu = 999;
    if (frame_since_otsu >= 5)
    {
        int th = compute_otsu();
        if (th > 0) cached_th = th;
        frame_since_otsu = 0;
    }
    else { frame_since_otsu++; }

    /* ---- 一次性二值化整张图 ---- */
    int th = (cached_th >= 0) ? cached_th : 128;
    for (int y = 0; y < pho_h; y++)
        for (int x = 0; x < pho_w; x++)
            Image_Used[y][x] = (mt9v03x_image[y][x] > th) ? 255 : 0;

    /* ---- 手遮摄像头 → 停车 ---- */
    if (camera_is_covered())
    {
        motor_stop();
        car_run = false;
        return 1;
    }

    /* ---- 清空边界数组 ---- */
    for (int y = 0; y < pho_h; y++)
    {
        l_border[y] = BORDER_INVALID;
        r_border[y] = BORDER_INVALID;
    }

    /* ---- 找种子 → 生长 → 边界提取 ---- */
    find_seeds(&llx, &lly, &rrx, &rry);
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

    /* ---- 补全 / 回退 ---- */
    {
        int gap_start = 79;
        if (llx >= 0 && lly > gap_start) gap_start = lly;
        if (rrx >= 0 && rry > gap_start) gap_start = rry;

        if (llx >= 0 || rrx >= 0)
        {
            bottom_scan(gap_start);
        }
        else
        {
            fallback_scan();
        }
    }

    /* ---- 中线 + 偏差 ---- */
    pho_center();
    vis_frame_ready = 1;   // 通知显示层新帧就绪
    return 0;
}

/* ================================================================
 * 显示搜线结果
 *
 * 灰度原图上叠加：
 *   红色   该行双边都有效
 *   黄色   该行仅单边有效（白像素重心推算）
 *   绿色   中线（平滑后的 center_line[]）
 * ================================================================ */
void vis_draw(void)
{
    ips200_show_gray_image(0, BIN_PARAM_H, (const uint8 *)mt9v03x_image,
                       pho_w, pho_h, pho_w, pho_h, 0);

    // ---- 边界 + 中线：逐行绘制，每行独立判双边/单边 ----
    for (int y = 115; y >= 20; y--)
    {
        int dy = y + BIN_PARAM_H;
        uint8 lb = l_border[y];
        uint8 rb = r_border[y];

        // 本行双边都有效 → 红；缺一边 → 黄
        int both = (lb != BORDER_INVALID && rb != BORDER_INVALID);
        int col  = both ? RGB565_RED : RGB565_YELLOW;

        if (lb != BORDER_INVALID)
        {
            ips200_draw_point(lb, (uint16)dy, col);
            if (lb < pho_w_max) ips200_draw_point((uint16)(lb + 1), (uint16)dy, col);
        }
        if (rb != BORDER_INVALID)
        {
            ips200_draw_point(rb, (uint16)dy, col);
            if (rb < pho_w_max) ips200_draw_point((uint16)(rb + 1), (uint16)dy, col);
        }

        // 中线（绿色）
        uint8 c = center_line[y];
        if (c != BORDER_INVALID)
        {
            ips200_draw_point(c, (uint16)dy, RGB565_GREEN);
            if (c < pho_w_max) ips200_draw_point((uint16)(c + 1), (uint16)dy, RGB565_GREEN);
        }
    }
}


/* ================================================================
 * 双阈值二值化显示（BIN 模式）
 * 0=黑, 255=白，用于确认阈值是否把赛道和背景正确分开
 * ================================================================ */
void vis_bin_draw(void)
{


    ips200_show_gray_image(0, BIN_PARAM_H, (const uint8 *)Image_Used,
                       pho_w, pho_h, pho_w, pho_h, 0);
}
