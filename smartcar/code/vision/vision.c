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
 * 二次曲线拟合（最小二乘）
 *
 *   模型：x = a·y² + b·y + c
 *   输入：border（边界数组），y_start..y_end 采样范围，step 采样间隔
 *   输出：fit 结构体，valid=1 表示拟合成功
 *
 *   采样策略：
 *     - 至少需要 FIT_MIN_PTS 个有效点才拟合
 *     - 自动跳过 border[y]==BORDER_INVALID（无数据）的行
 *     - 等权最小二乘，所有采样点同等对待
 * ================================================================ */
#define FIT_MIN_PTS  10

typedef struct {
    float a, b, c;      // 二次曲线系数
    int   valid;        // 1=拟合成功
    int   n_points;     // 有效采样点数
} quad_fit_t;

static void fit_quadratic(border_line border, int y_start, int y_end,
                          int step, quad_fit_t *fit)
{
    fit->valid = 0;
    fit->n_points = 0;

    float sy = 0, sy2 = 0, sy3 = 0, sy4 = 0;
    float sx = 0, sxy = 0, sxy2 = 0;
    int n = 0;

    for (int y = y_start; y >= y_end; y -= step)
    {
        // 无数据行跳过
        if (border[y] == BORDER_INVALID) continue;

        float yy = (float)y;
        float xx = (float)border[y];
        float y2 = yy * yy;

        sy   += yy;
        sy2  += y2;
        sy3  += y2 * yy;
        sy4  += y2 * y2;
        sx   += xx;
        sxy  += xx * yy;
        sxy2 += xx * y2;
        n++;
    }

    fit->n_points = n;
    if (n < FIT_MIN_PTS) return;

    // 克莱姆法则解 3×3 正规方程
    float det = n*(sy2*sy4 - sy3*sy3)
              - sy*(sy*sy4 - sy3*sy2)
              + sy2*(sy*sy3 - sy2*sy2);

    if (det > -1e-9f && det < 1e-9f) return;  // 奇异，拟合失败

    float inv_det = 1.0f / det;

    fit->c = (sx *(sy2*sy4 - sy3*sy3)
            - sy *(sxy*sy4 - sxy2*sy3)
            + sy2*(sxy*sy3 - sxy2*sy2)) * inv_det;

    fit->b = (n  *(sxy*sy4 - sxy2*sy3)
            - sx *(sy*sy4 - sy3*sy2)
            + sy2*(sy*sxy2 - sxy*sy2)) * inv_det;

    fit->a = (n  *(sy2*sxy2 - sy3*sxy)
            - sy *(sy*sxy2 - sy2*sxy)
            + sx *(sy*sy3 - sy2*sy2)) * inv_det;

    fit->valid = 1;
}

/* ================================================================
 * 从二次曲线计算指定 y 处的 x 值
 * ================================================================ */
static float quad_eval(const quad_fit_t *fit, float y)
{
    return fit->a * y * y + fit->b * y + fit->c;
}

/* ================================================================
 * 计算中线曲线和赛道偏差 err
 *
 *   1. 拟合左边界 l_border → fit_L
 *   2. 拟合右边界 r_border → fit_R
 *   3. 双边都有效：center(y) = (fit_L(y) + fit_R(y)) / 2
 *   4. 只有单边：center(y) = fit_L(y) + hw  或  fit_R(y) - hw
 *   5. 预瞄点读偏差 → err
 *   6. 曲率从 fit_L/fit_R 导出
 *
 *   center_line[] 供 vis_draw 显示（拟合曲线值，连续光滑）
 * ================================================================ */
static void pho_center(void)
{
    quad_fit_t fit_L, fit_R;
    int y_start = 100, y_end = 40, step = 3;

    // ---- 平滑 + 分别拟合左右边界 ----
    border_smooth(l_border);
    border_smooth(r_border);
    fit_quadratic(l_border, y_start, y_end, step, &fit_L);
    fit_quadratic(r_border, y_start, y_end, step, &fit_R);

    // ---- 生成 center_line[] + 计算 err（同逻辑） ----
    int hw = get_half_width(0);

    // 统计左右有效点数（直接数 border 数组，不依赖 fit 和 seed）
    int nL = 0, nR = 0;
    for (int y = y_start; y >= y_end; y -= step)
    {
        if (l_border[y] != BORDER_INVALID) nL++;
        if (r_border[y] != BORDER_INVALID) nR++;
    }
    int both = (nL >= FIT_MIN_PTS && nR >= FIT_MIN_PTS
             && nL > nR / 3 && nR > nL / 3);

    int y_look = y_start - (int)lookahead;
    float x_pred;

    if (both)
    {
        // 双边扫到：拟合曲线取中点
        for (int y = pho_h - 1; y >= 0; y--)
        {
            if (l_border[y] == BORDER_INVALID && r_border[y] == BORDER_INVALID)
            {
                center_line[y] = BORDER_INVALID;
                continue;
            }
            float cl = quad_eval(&fit_L, (float)y);
            float cr = quad_eval(&fit_R, (float)y);
            int c = (int)((cl + cr) / 2.0f + 0.5f);
            if (c < pho_w_min) c = pho_w_min;
            if (c > pho_w_max) c = pho_w_max;
            center_line[y] = (uint8)c;
        }
        x_pred = (quad_eval(&fit_L, (float)y_look) + quad_eval(&fit_R, (float)y_look)) / 2.0f;
    }
    else if (nL >= FIT_MIN_PTS)
    {
        // 只有左边界：原始边界偏移
        for (int y = pho_h - 1; y >= 0; y--)
        {
            if (l_border[y] == BORDER_INVALID)
            {
                center_line[y] = BORDER_INVALID;
                continue;
            }
            int c = (int)l_border[y] + hw;
            if (c < pho_w_min) c = pho_w_min;
            if (c > pho_w_max) c = pho_w_max;
            center_line[y] = (uint8)c;
        }
        int sum = 0, cnt = 0;
        for (int y = y_look + 10; y >= y_look - 10; y--)
        {
            if (y < 0 || y >= pho_h) continue;
            if (l_border[y] == BORDER_INVALID) continue;
            sum += (int)l_border[y] + hw;
            cnt++;
        }
        x_pred = (cnt > 0) ? (float)sum / (float)cnt : 0.0f;
    }
    else if (nR >= FIT_MIN_PTS)
    {
        // 只有右边界：原始边界偏移
        for (int y = pho_h - 1; y >= 0; y--)
        {
            if (r_border[y] == BORDER_INVALID)
            {
                center_line[y] = BORDER_INVALID;
                continue;
            }
            int c = (int)r_border[y] - hw;
            if (c < pho_w_min) c = pho_w_min;
            if (c > pho_w_max) c = pho_w_max;
            center_line[y] = (uint8)c;
        }
        int sum = 0, cnt = 0;
        for (int y = y_look + 10; y >= y_look - 10; y--)
        {
            if (y < 0 || y >= pho_h) continue;
            if (r_border[y] == BORDER_INVALID) continue;
            sum += (int)r_border[y] - hw;
            cnt++;
        }
        x_pred = (cnt > 0) ? (float)sum / (float)cnt : 0.0f;
    }
    else
    {
        for (int y = 0; y < pho_h; y++)
            center_line[y] = BORDER_INVALID;
        return;
    }

    err = x_pred - (float)pho_center_x;
}

/* ================================================================
 * (fallback_scan 已删除 — 丢线由 pho_center 拟合失败自动兜底)
 * ================================================================ */


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
        }
        // 两边都没种子 → 拟合自然失败，err 保留上帧值

        /* 第四步：种子行以下空白区逐行扫描 */
        bottom_scan(gap_start);
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

    // ---- 左边界：种子点逐行画（红色/黄色，2px宽） ----
    for (int i = 0; i < data_l; i++)
    {
        int x = (int)points_l[i][0] + 1;  // dir=+1 黑→白
        int y = (int)points_l[i][1];
        if (x < 0) x = 0; if (x > pho_w_max) x = pho_w_max;
        int dy = y + BIN_PARAM_H;
        int col = (data_r > 0) ? RGB565_RED : RGB565_YELLOW;
        ips200_draw_point((uint16)x, (uint16)dy, col);
        if (x < pho_w_max) ips200_draw_point((uint16)(x + 1), (uint16)dy, col);
    }

    // ---- 右边界：种子点逐行画（红色/黄色，2px宽） ----
    for (int i = 0; i < data_r; i++)
    {
        int x = (int)points_r[i][0] - 1;  // dir=-1 黑→白
        int y = (int)points_r[i][1];
        if (x < 0) x = 0; if (x > pho_w_max) x = pho_w_max;
        int dy = y + BIN_PARAM_H;
        int col = (data_l > 0) ? RGB565_RED : RGB565_YELLOW;
        ips200_draw_point((uint16)x, (uint16)dy, col);
        if (x < pho_w_max) ips200_draw_point((uint16)(x + 1), (uint16)dy, col);
    }

    // ---- 中线：拟合曲线逐点（绿色） ----
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
