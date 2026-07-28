#include "config.h"

uint8  g_bin_image[pho_h][pho_w];

 border_line l_border;
 border_line r_border;
 border_line center_line;

 volatile float err;
 volatile uint8_t vis_frame_ready;

 uint8 lookahead = DEFAULT_LOOKAHEAD;  // 预瞄距离，菜单可调

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
 * 从下往上扫 115→40，独立找左右种子，近处赛道宽更可靠
 * 返回: 左右种子独立标记，至少一个找到就可以生长
 * ================================================================ */
static void find_seeds(int *llx, int *lly, int *rrx, int *rry)
{
    *llx = -1; *rrx = -1;

    // 从下往上扫 115→40，近处赛道宽，种子更可靠
    int l_fallback = -1, r_fallback = -1;  // 出界兜底暂存

    for (int y = 115; y >= 40; y--)
    {
        // 左种子：从中间向左扫，找赛道左边缘（白→黑转跳）+ 3连白
        if (*llx < 0 && is_white(mt9v03x_image[y][pho_center_x]))
        {
            for (int x = pho_center_x; x >= 1; x--)
            {
                if (!is_white(mt9v03x_image[y][x - 1])
                    && is_white(mt9v03x_image[y][x])
                    && is_white(mt9v03x_image[y][x + 1])
                    && is_white(mt9v03x_image[y][x + 2]))
                { *llx = x; *lly = y; break; }
            }
            // 出界不要急于接受——记下行号，继续向下找真实边界
            if (*llx < 0 && l_fallback < 0) { l_fallback = y; }
        }

        // 右种子：从中间向右扫，找赛道右边缘（白→黑转跳）+ 3连白
        if (*rrx < 0 && is_white(mt9v03x_image[y][pho_center_x]))
        {
            for (int x = pho_center_x; x < pho_w - 1; x++)
            {
                if (is_white(mt9v03x_image[y][x])
                    && !is_white(mt9v03x_image[y][x + 1])
                    && is_white(mt9v03x_image[y][x - 1])
                    && is_white(mt9v03x_image[y][x - 2]))
                { *rrx = x; *rry = y; break; }
            }
            if (*rrx < 0 && r_fallback < 0) { r_fallback = y; }
        }

        if (*llx >= 0 && *rrx >= 0) break;
    }
    // 扫到底没找到真实种子 → 用出界兜底
    if (*llx < 0 && l_fallback >= 0) { *llx = 0; *lly = l_fallback; }
    if (*rrx < 0 && r_fallback >= 0) { *rrx = pho_w_max; *rry = r_fallback; }
}
/* ================================================================
 * 底部扫描：逐行扫 118→(种子行+1)，直接填 l_border/r_border
 * 种子行以下无种子覆盖，逐行扫更快更准
 * ================================================================ */
static void bottom_scan(int seed_y)
{
    int end = seed_y + 1;  // 种子行已覆盖，从上一行开始扫
    if (end < 79) end = 79;  // 种子很高时至少扫到79（与fill_gaps接上）
    for (int y = 118; y >= end; y--)
    {
        int lx = -1, rx = -1;
        int m = pho_center_x;

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
        if (lx < 0) lx = BORDER_INVALID;  // 扫到底没找到

        // 右边界：从中间向右扫，找赛道右边缘（白→黑）+2连白
        for (int x = m; x < pho_w - 1; x++)
        {
            if (is_white(mt9v03x_image[y][x])
                && !is_white(mt9v03x_image[y][x + 1])
                && is_white(mt9v03x_image[y][x - 1]))
            {
                rx = x - 1; break;
            }
        }
        if (rx < 0) rx = BORDER_INVALID;  // 扫到底没找到

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
 *   爬到顶 (cy <= 40)
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
                for (int x = pho_center_x; x >= 1; x--)  // x=1→检查pixel[0]
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
            if (!found) break;  // 5行全失败 终止
        }

        if (cy <= 40) break;  // 爬到采样区顶部
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
                for (int x = pho_center_x; x < pho_w - 1; x++)
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
            if (!found) break;  // 5行全失败 终止
        }

        if (cy <= 40) break;  // 爬到采样区顶部
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
 * 同行多个点取均值，减少单点噪声
 * ================================================================ */
static void pho_border(uint16 points[][2], uint16 total,
                           border_line border, int dir)
{
    /* 先全部填 BORDER_INVALID，表示"无数据" */
    for (int y = 0; y < pho_h; y++)
        border[y] = BORDER_INVALID;

    if (total == 0) return;

    /* 按 y 分组，同行多点取均值再偏移 */
    int cur_y  = points[0][1];
    int sum_px = 0, cnt = 0;

    for (int i = 0; i < total; i++)
    {
        int py = points[i][1];
        int px = points[i][0];

        if (py != cur_y)
        {
            // 上一行结算：均值 → 偏移 → 写入
            int avg = (sum_px + cnt / 2) / cnt;  // 四舍五入
            int val = avg + dir;
            if (val < pho_w_min) val = pho_w_min;
            if (val > pho_w_max) val = pho_w_max;
            border[cur_y] = (uint8)val;

            cur_y  = py;
            sum_px = 0;
            cnt    = 0;
        }
        sum_px += px;
        cnt++;
    }

    // 最后一组结算
    if (cnt > 0)
    {
        int avg = (sum_px + cnt / 2) / cnt;
        int val = avg + dir;
        if (val < pho_w_min) val = pho_w_min;
        if (val > pho_w_max) val = pho_w_max;
        border[cur_y] = (uint8)val;
    }
}

static int get_half_width(int y);  // 前向声明

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
 * 计算中线偏差 err — 预瞄窗口采样取均值
 *
 *   直接在 lookahead 前后 10 行逐行取 (l+r)/2 或 border±hw，
 *   不拟合曲线。多行均值 = 天然低通滤波。
 * ================================================================ */
static void pho_center(void)
{
    // ---- 平滑边界 ----
    border_smooth(l_border);
    border_smooth(r_border);

    int hw = get_half_width(0);
    int y_look = 100 - (int)lookahead;

    // ---- 生成 center_line[]（显示用，逐行判断） ----
    for (int y = pho_h - 1; y >= 0; y--)
    {
        int l = (int)l_border[y];
        int r = (int)r_border[y];
        int c;

        if (l != BORDER_INVALID && r != BORDER_INVALID)
            c = (l + r) / 2;
        else if (l != BORDER_INVALID)
            c = l + hw;
        else if (r != BORDER_INVALID)
            c = r - hw;
        else
        {
            center_line[y] = BORDER_INVALID;
            continue;
        }

        if (c < pho_w_min) c = pho_w_min;
        if (c > pho_w_max) c = pho_w_max;
        center_line[y] = (uint8)c;
    }

    // ---- 计算 err：预瞄窗口均值 ----
    int sum = 0, cnt = 0;
    for (int y = y_look + 10; y >= y_look - 10; y -= 5)
    {
        if (y < 0 || y >= pho_h) continue;

        int l = (int)l_border[y];
        int r = (int)r_border[y];
        int c;

        if (l != BORDER_INVALID && r != BORDER_INVALID)
            c = (l + r) / 2;
        else if (l != BORDER_INVALID)
            c = l + hw;
        else if (r != BORDER_INVALID)
            c = r - hw;
        else
            continue;

        sum += (c - pho_center_x);
        cnt++;
    }

    if (cnt > 0)
        err = (float)sum / (float)cnt;
    // else: err 保留上帧不变
}

/* ================================================================
 * 回退扫描：种子追踪完全失败时的最后手段
 *
 * 不依赖种子 / 邻域生长，每行独立扫描原始灰度图：
 *   左边界：从中心向左扫，找 白→黑 跳变
 *   右边界：从中心向右扫，找 白→黑 跳变
 *
 * 有效行条件（同时满足）：
 *   1. 左右边界都找到
 *   2. 赛道宽度 > 4px
 *
 * 但直接扫原始灰度图，避免额外维护二值图缓存
 * ================================================================ */
static void fallback_scan(void)
{
    int useful_rows = 0;

    for (int y = 1; y < pho_h - 1; y++)
    {
        int lb = BORDER_INVALID, rb = BORDER_INVALID;

        /* 左边界：从中向左扫，找 白->黑 跳变 */
        for (int x = pho_center_x; x >= 1; x--)
        {
            if (is_white(mt9v03x_image[y][x]) && !is_white(mt9v03x_image[y][x - 1]))
            {
                lb = x;
                break;
            }
        }

        /* 右边界：从中向右扫，找 白->黑 跳变 */
        for (int x = pho_center_x; x < pho_w - 1; x++)
        {
            if (is_white(mt9v03x_image[y][x]) && !is_white(mt9v03x_image[y][x + 1]))
            {
                rb = x;
                break;
            }
        }

        if (lb != BORDER_INVALID && rb != BORDER_INVALID && rb - lb > 4)
        {
            l_border[y] = (uint8)lb;
            r_border[y] = (uint8)rb;
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
 * 赛道半宽（常量，菜单可调）
 * ================================================================ */
uint8 half_width = DEFAULT_HALF_WIDTH;

static int get_half_width(int y)
{
    (void)y;
    return (int)half_width;
}

/* ================================================================
 * 断点补偿：种子生长中断时（gap_start→40 行范围内）
 *   l有r缺 → 中心→右扫找右边界，出界则半宽推算
 *   r有l缺 → 中心→左扫找左边界，出界则半宽推算
 *   都缺 → 质心兜底
 * ================================================================ */
static void fill_gaps(int gap_start)
{
    for (int y = gap_start; y >= 40; y--)
    {
        if (l_border[y] != BORDER_INVALID && r_border[y] != BORDER_INVALID) continue;

        // 只有左边界 → 找右边界
        if (l_border[y] != BORDER_INVALID && r_border[y] == BORDER_INVALID)
        {
            for (int x = pho_center_x; x < pho_w - 1; x++)  // 从中向右扫
            {
                if (is_white(mt9v03x_image[y][x])
                    && !is_white(mt9v03x_image[y][x + 1])
                    && is_white(mt9v03x_image[y][x - 1]))
                {
                    r_border[y] = (uint8)(x - 1); break;
                }
            }
            // 扫不到且边缘全白 → 边界在画外，半宽推算
            if (r_border[y] == BORDER_INVALID && is_white(mt9v03x_image[y][pho_w_max]))
            {
                int rv = (int)l_border[y] + 2 * get_half_width(y);
                r_border[y] = (uint8)(rv > pho_w_max ? pho_w_max : rv);
            }
            else if (r_border[y] == BORDER_INVALID)
                r_border[y] = BORDER_INVALID;  // 找不到也不是全白 → 放弃
        }

        // 只有右边界 → 找左边界
        if (r_border[y] != BORDER_INVALID && l_border[y] == BORDER_INVALID)
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
            if (l_border[y] == BORDER_INVALID && is_white(mt9v03x_image[y][0]))
            {
                int lv = (int)r_border[y] - 2 * get_half_width(y);
                l_border[y] = (uint8)(lv < pho_w_min ? pho_w_min : lv);
            }
            else if (l_border[y] == BORDER_INVALID)
                l_border[y] = BORDER_INVALID;  // 找不到也不是全白 → 放弃
        }

        // 两边都缺 → 质心兜底
        if (l_border[y] == BORDER_INVALID && r_border[y] == BORDER_INVALID)
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
            else  // 白像素太少 → 用半宽+图像中心产生合成边界
            {
                int hw = get_half_width(y);
                int cl = pho_center_x - hw;
                int cr = pho_center_x + hw;
                l_border[y] = (uint8)(cl < pho_w_min ? pho_w_min : cl);
                r_border[y] = (uint8)(cr > pho_w_max ? pho_w_max : cr);
            }
        }
    }
}


/* ================================================================
 * 搜线主函数
 *
 * 正常：find_seeds(115→40) → seed_grow → pho_border → fill_gaps(种子行→40) → bottom_scan(118→种子行+1) → pho_center
 * 丢线：两边都没种子 → 拟合自然失败 → err 保留上帧值
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

    /* 清空边界数组（pho_border只清自己那边，这边统一清） */
    for (int y = 0; y < pho_h; y++)
    {
        l_border[y] = BORDER_INVALID;
        r_border[y] = BORDER_INVALID;
    }

    /* 第一步：115→40从下往上找左右种子 */
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
    {
        int gap_start = 79;  // 兜底：种子以上至少到79
        if (llx >= 0 && lly > gap_start) gap_start = lly;
        if (rrx >= 0 && rry > gap_start) gap_start = rry;

        if (llx >= 0 || rrx >= 0)
        {
            fill_gaps(gap_start);  // 有一边就找另一边，都缺才质心
            bottom_scan(gap_start);
        }
        else
        {
            /* 两边都没种子 → 回退逐行扫描挽救本帧 */
            fallback_scan();
        }
    }

    /* 汇聚：计算中线和偏差 */
    pho_center();
    return 0;
}

/* ================================================================
 * 显示搜线结果
 *
 * 灰度原图上叠加：
 *   红色   左右边界（双边都有效且不相等时两侧都画）
 *   黄色   仅单边有效时的边界（方便观察丢线侧）
 *   绿色   中线（拟合曲线，连续光滑）
 *
 * 绘制范围：行 115→40
 * ================================================================ */
void vis_draw(void)
{
    ips200_show_gray_image(0, BIN_PARAM_H, (const uint8 *)mt9v03x_image,
                       pho_w, pho_h, pho_w, pho_h, 0);

    // ---- 左右边界：从 border 数组逐行画（红色/黄色，2px宽） ----
    for (int y = 115; y >= 40; y--)
    {
        int dy = y + BIN_PARAM_H;
        uint8 lb = l_border[y];
        uint8 rb = r_border[y];
        if (lb != BORDER_INVALID)
        {
            int col = (data_r > 0) ? RGB565_RED : RGB565_YELLOW;
            ips200_draw_point(lb, (uint16)dy, col);
            if (lb < pho_w_max) ips200_draw_point((uint16)(lb + 1), (uint16)dy, col);
        }
        if (rb != BORDER_INVALID)
        {
            int col = (data_l > 0) ? RGB565_RED : RGB565_YELLOW;
            ips200_draw_point(rb, (uint16)dy, col);
            if (rb < pho_w_max) ips200_draw_point((uint16)(rb + 1), (uint16)dy, col);
        }
    }

    // ---- 中线：逐行中心点（绿色） ----
    for (int y = 115; y >= 40; y--)
    {
        uint8 c = center_line[y];
        if (c == BORDER_INVALID) continue;

        int dy = y + BIN_PARAM_H;
        ips200_draw_point(c, dy, RGB565_GREEN);
        if (c < pho_w_max) ips200_draw_point(c + 1, dy, RGB565_GREEN);
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
