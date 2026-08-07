
#include "config.h"

 border_line l_border;                      // 左边界，每行一个列号
 border_line r_border;                      // 右边界
 border_line center_line;                   // 中线（边框平滑 + 中线平滑后，绘图和err共用）
 uint8 l_border_exist[pho_h];               // 左边界该行是否真实存在（1=真边界 0=出画伪边界/无）
 uint8 r_border_exist[pho_h];               // 右边界该行是否真实存在

 volatile float err;                        // 中线偏离图像中心的像素均值，>0偏右
 volatile uint8_t vis_frame_ready;          // 新帧处理完成，显示层可刷新
 volatile int16  asc_range_dbg = 0;          // 诊断：ASC窗口行数（100-y_far）

 uint8 asc_far = DEFAULT_ASC_FAR;             // ASC采样远行（顶部行），菜单可调，默认10
static int16 last_valid_width = 60;            // 最近双边真实路宽（逐行扫描外推基准）

/* ================================================================
 * 十字识别与补线 — 编译期常量
 * ================================================================ */
#define CROSS_ROI_TOP                 12    /* 独立扫描ROI顶行 */
#define CROSS_ROI_BOTTOM              75    /* 独立扫描ROI底行 */
#define CROSS_CORNER_SEARCH_TOP       30    /* 拐点搜索顶行 */
#define CROSS_CORNER_SEARCH_BOTTOM    65    /* 拐点搜索底行 */
#define CROSS_CONTEXT_ROWS             7    /* 过渡带上下文半径 */
#define CROSS_ABOVE_INVALID_MIN        2    /* 上方最少无效行数 */
#define CROSS_BELOW_VALID_MIN          4    /* 下方最少有效行数 */
#define CROSS_CORNER_SIDE_MARGIN       5    /* 拐点距种子列最小边距 */
#define CROSS_CORNER_MIN_GAP          18    /* 两拐点最小间距 */
#define CROSS_CORNER_MAX_GAP         130    /* 两拐点最大间距 */
#define CROSS_LANE_ROW_MAX_GAP       150    /* 车道行双边最大间距 */
#define CROSS_CORNER_MAX_ROW_DIFF     18    /* 两拐点最大行差 */
#define CROSS_MID_MAX_OFFSET          36    /* 拐点中点距种子列最大偏移 */
#define CROSS_TANGENT_SEARCH_TOP      30    /* 切线搜索顶行 */
#define CROSS_TANGENT_SEARCH_BOTTOM   72    /* 切线搜索底行 */
#define CROSS_TANGENT_HALF_WINDOW      3    /* 切线窗口半宽 */
#define CROSS_TANGENT_MAX_STEP        10    /* 切线窗口内最大阶跃 */
#define CROSS_TANGENT_EDGE_MARGIN      2    /* 切线边沿距图像边界最小距离 */
#define CROSS_SLOPE_SCALE            256    /* 斜率定点化比例 */
#define CROSS_FULL_WIDTH_TOP          42    /* 全宽白带检测顶行 */
#define CROSS_FULL_WIDTH_BOTTOM       95    /* 全宽白带检测底行 */
#define CROSS_FULL_WIDTH_WHITE_MIN    (MT9V03X_W - 12)  /* 全宽白带最少白像素 */
#define CROSS_EDGE_SAMPLE_COLS         8    /* 图像边缘采样列数 */
#define CROSS_EDGE_WHITE_MIN           7    /* 边缘采样区最少白像素 */
#define CROSS_FULL_WIDTH_ROWS          3    /* 连续满足白带条件的行数 */
#define CROSS_CONFIRM_FRAMES           2    /* 连续确认帧数 */
#define CROSS_HOLD_MISSED_FRAMES       2    /* 漏检保持帧数 */
#define CROSS_COOLDOWN_FRAMES         8    /* 离开十字后冷却帧数，防止重复触发 */

/* ================================================================
 * 斑马线检测 — 编译期常量
 * ================================================================ */
#define ZEBRA_SAMPLE_ROWS              3    /* 底部采样行数 */
#define ZEBRA_SAMPLE_ROW_STEP          5    /* 采样行间隔 */
#define ZEBRA_VALID_ROWS_MIN           2    /* 最少有效采样行数 */
#define ZEBRA_WHITE_RATIO_MIN         25    /* 白像素占比下限(%) */
#define ZEBRA_WHITE_RATIO_MAX         75    /* 白像素占比上限(%) */
#define ZEBRA_TRANSITIONS_MIN         10    /* 最少黑白跳变次数 */
#define ZEBRA_RUN_WIDTH_MIN            4    /* 连续同色段最小宽度 */
#define ZEBRA_BLACK_RUNS_MIN           5    /* 最少黑色段数 */
#define ZEBRA_WHITE_RUNS_MIN           5    /* 最少白色段数 */
#define ZEBRA_FILTER_RADIUS            2    /* 中值滤波半径(±2=5像素窗口) */
#define ZEBRA_HOLD_MISSED_FRAMES       2    /* 漏检保持帧数 */

/* 十字状态机 */
typedef enum {
    CROSS_STATE_NONE = 0,
    CROSS_STATE_CANDIDATE,
    CROSS_STATE_DETECTED,
} cross_state_enum;

/* 十字静态变量 */
static cross_state_enum cross_state = CROSS_STATE_NONE;
static uint8  cross_left_col,  cross_left_row;
static uint8  cross_right_col, cross_right_row;
static bool   cross_corners_valid;
static uint8  cross_confirm_count;
static uint8  cross_missed_count;
static uint16 cross_scan_left[MT9V03X_H];
static uint16 cross_scan_right[MT9V03X_H];
static bool   cross_scan_left_valid[MT9V03X_H];
static bool   cross_scan_right_valid[MT9V03X_H];
static uint8  cross_scan_seed_col;
static uint8  cross_cooldown_count;      /* 离开十字后冷却倒计时 */

volatile bool cross_active;              /* control.c读取，十字检测中降速 */

/* 斑马线静态变量 */
static bool   zebra_detected;
static uint8  zebra_missed_count;
static uint8  zebra_count;              /* 上升沿计数（发车重置）*/
static bool   zebra_prev;               /* 上一帧检测状态 */

/* ================================================================
 * Otsu 二值化图像缓存
 *   0=黑（背景）, 255=白（赛道）
 * ================================================================ */
uint8 Image_Used[pho_h][pho_w];

/* ================================================================
 * 大津法 (Otsu) — 全屏 下采样4x4，安全钳 20~235
 * 移植自 loongson_learn，double → float
 * ================================================================ */
static int compute_otsu(void)
{
    unsigned long hist[256] = {0};
    /* 全屏统计（行0~119），下采样4x4抗干扰 */
    for (int y = 0; y < pho_h; y += 4)
        for (int x = 0; x < pho_w; x += 4)
            hist[mt9v03x_image[y][x]]++;

    unsigned long total = (unsigned long)((pho_h + 3) / 4) * ((pho_w + 3) / 4);
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
 * calc_adaptive_far — 自适应预瞄远行
 *
 * 用上一帧 err 的 |err| 分段选择 ASC 采样远行：
 *   直道看远（far 减小，预瞄更远），弯道看近（far 增大，跟住弯心）
 * 偏移相对菜单调好的 asc_far，阈值/偏移为编译期常量(ADAPT_*)，
 * 最终钳位 5~95
 * ================================================================ */
static int calc_adaptive_far(void)
{
    float e = (err > 0) ? err : -err;
    int y_far = (int)asc_far;

    if      (e <= (float)ADAPT_ERR_TH1) y_far += ADAPT_OFF_STR;
    else if (e <= (float)ADAPT_ERR_TH2) y_far += ADAPT_OFF_SML;
    else                                y_far += ADAPT_OFF_BIG;

    if (y_far < 10)  y_far = 10;
    if (y_far > 100) y_far = 100;
    return y_far;
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
 * 2. center_line[]: 双边→中点(记录宽度), 单边→真实路宽外推, 都缺→白像素重心
 * 3. border_smooth(center_line) — 平滑中线
 * 4. ASC多行加权平均: 100→asc_far, 权重2.0→2.5递增(远重近轻), 均值=err
 *
 * 调参: far(ASC远行基准,默认20); 单边→最新真实路宽外推; 双侧丢线→锁存
 * ================================================================ */

/* ================================================================
 * sweep_boundaries — 逐行扫描替代种子生长
 *
 * 从底行最长白列出发，逐行向上扫边界：
 *   左边界: 从 prev_l ± range 搜 B→W→W（黑→白→白）= 真左边界
 *   右边界: 从 prev_r ± range 搜 W→B（白→黑）= 真右边界
 *   单边丢: last_valid_width 外推，exist=0
 *   双边丢: 中心区域白检测 → 继续 / 黑区5行停止
 *
 * 直接填充 l_border[] / r_border[] / exist[]，替代种子生长全流程。
 * 返回: 1=双侧丢线（pho_center安全钳触发） 0=正常
 * ================================================================ */
static int sweep_boundaries(void)
{
    /* ---- 底行找最长白列作种子 ---- */
    int y_bot = pho_h - 1;
    int best_s = -1, best_e = -1, best_len = 0;
    int run_s = -1;
    for (int x = 0; x < pho_w; x++)
    {
        if (Image_Used[y_bot][x] == 255)
        { if (run_s < 0) run_s = x; }
        else if (run_s >= 0)
        {
            int len = x - run_s;
            if (len > best_len) { best_len = len; best_s = run_s; best_e = x - 1; }
            run_s = -1;
        }
    }
    if (run_s >= 0)
    { int len = pho_w - run_s;
      if (len > best_len) { best_len = len; best_s = run_s; best_e = pho_w - 1; } }

    if (best_len < 3) return 1;  /* 底行无赛道 */

    /* 验证真边界（exist=黑白跳变） */
    int prev_l = best_s;
    int prev_r = best_e;
    int l_exist = (best_s > 0 && Image_Used[y_bot][best_s - 1] == 0);
    int r_exist = (best_e < pho_w_max && Image_Used[y_bot][best_e + 1] == 0);

    last_valid_width = best_e - best_s;
    if (last_valid_width < 20)  last_valid_width = 20;
    if (last_valid_width > 160) last_valid_width = 160;

    l_border[y_bot] = (uint8)best_s;
    r_border[y_bot] = (uint8)best_e;
    l_border_exist[y_bot] = (uint8)l_exist;
    r_border_exist[y_bot] = (uint8)r_exist;

    /* ---- 向上逐行扫描 ---- */
    int consec_lost = 0;

    for (int y = y_bot - 1; y >= 20; y--)
    {
        int range = last_valid_width / 4;
        if (range < 3) range = 3;
        if (range > 25) range = 25;

        int new_l = BORDER_INVALID, new_r = BORDER_INVALID;
        int found_l = 0, found_r = 0;
        int l_real = 0, r_real = 0;

        /* --- 搜左边界: 从 prev_l-range 向右找 B→W→W --- */
        {
            int x0 = prev_l - range; if (x0 < 0) x0 = 0;
            int x1 = prev_l + range; if (x1 > pho_w - 3) x1 = pho_w - 3;
            for (int x = x0; x <= x1; x++)
            {
                if (Image_Used[y][x] == 0
                    && Image_Used[y][x + 1] == 255
                    && Image_Used[y][x + 2] == 255)
                { new_l = x + 1; l_real = 1; found_l = 1; break; }
            }
            /* 局部未找到 → 扩大范围（3×range半径）搜 B→W→W */
            if (!found_l)
            {
                int lx0 = prev_l - range * 3; if (lx0 < 0) lx0 = 0;
                int lx1 = prev_l + range * 3; if (lx1 > pho_w - 3) lx1 = pho_w - 3;
                for (int x = lx0; x <= lx1; x++)
                {
                    if (Image_Used[y][x] == 0
                        && Image_Used[y][x + 1] == 255
                        && Image_Used[y][x + 2] == 255)
                    { new_l = x + 1; l_real = 1; found_l = 1; break; }
                }
            }
        }

        /* --- 搜右边界: 从 prev_r+range 向左找 W→B（白→黑跳变） --- */
        {
            int x0 = prev_r + range; if (x0 >= pho_w) x0 = pho_w_max;
            int x1 = prev_r - range; if (x1 < 0) x1 = 0;
            for (int x = x0; x >= x1; x--)
            {
                if (Image_Used[y][x] == 255
                    && x + 1 < pho_w
                    && Image_Used[y][x + 1] == 0)
                { new_r = x; r_real = 1; found_r = 1; break; }
            }
            /* 局部未找到 → 扩大范围（3×range半径）搜 W→B */
            if (!found_r)
            {
                int rx0 = prev_r + range * 3; if (rx0 >= pho_w) rx0 = pho_w_max;
                int rx1 = prev_r - range * 3; if (rx1 < 0) rx1 = 0;
                for (int x = rx0; x >= rx1; x--)
                {
                    if (Image_Used[y][x] == 255
                        && x + 1 < pho_w
                        && Image_Used[y][x + 1] == 0)
                    { new_r = x; r_real = 1; found_r = 1; break; }
                }
            }
        }

        /* --- 处理边界状态 --- */
        if (found_l && found_r)
        {
            /* 双边都找到 → 更新路宽 */
            last_valid_width = new_r - new_l;
            if (last_valid_width < 20)  last_valid_width = 20;
            if (last_valid_width > 160) last_valid_width = 160;
            consec_lost = 0;
        }
        else if (found_l && !found_r)
        {
            if (y >= 70)
            {
                /* 底部单边 → 屏幕边缘作对侧，pho_center中点=强转向 */
                new_r = pho_w_max;
                r_real = 1;
            }
            else
            {
                /* 上部单边 → 路宽外推 */
                new_r = new_l + last_valid_width;
                if (new_r > pho_w_max) new_r = pho_w_max;
            }
            consec_lost = 0;
        }
        else if (!found_l && found_r)
        {
            if (y >= 70)
            {
                new_l = 0;
                l_real = 1;
            }
            else
            {
                new_l = new_r - last_valid_width;
                if (new_l < 0) new_l = 0;
            }
            consec_lost = 0;
        }
        else
        {
            /* 双边搜不到 → 统计全行白像素总量。
             * 旧逻辑只看中心±20列(74~114)，赛道平移至边缘时中心为空
             * → 误判为"真丢线"→both_lost=1→err被锁。
             * 改为全行统计：全行有白=赛道在边缘(锚点漂移但赛道未丢)
             * → 沿用上一行边界，不计consec_lost。全行也无白=真丢线。 */
            int all_w = 0;
            for (int x = 0; x < pho_w; x++)
                if (Image_Used[y][x] == 255) all_w++;

            if (all_w > 10)
            {
                /* 赛道在画面内(可能偏边缘) → 沿用上一行边界 */
                new_l = prev_l; new_r = prev_r;
                l_real = 0; r_real = 0;
                consec_lost = 0;
            }
            else
            {
                /* 全行无白 → 真丢线 */
                consec_lost++;
                if (consec_lost >= 5)
                {
                    /* 5行连续全黑 → 停止，上方全标无效 */
                    for (int ry = y; ry >= 0; ry--)
                    {
                        l_border[ry] = BORDER_INVALID;
                        r_border[ry] = BORDER_INVALID;
                        l_border_exist[ry] = 0;
                        r_border_exist[ry] = 0;
                    }
                    return 1;
                }
                /* 本行标无效，沿用prev继续试 */
                l_border[y] = BORDER_INVALID;
                r_border[y] = BORDER_INVALID;
                l_border_exist[y] = 0;
                r_border_exist[y] = 0;
                continue;
            }
        }

        /* 写入本行 */
        l_border[y] = (uint8)new_l;
        r_border[y] = (uint8)new_r;
        l_border_exist[y] = (uint8)l_real;
        r_border_exist[y] = (uint8)r_real;
        prev_l = new_l;
        prev_r = new_r;
    }

    return 0;
}

static void pho_center(void)
{
    // ---- 平滑边界 ----
    border_smooth(l_border);
    border_smooth(r_border);

    static int last_real_width = 60;   // 最近一次双边真实宽度（单边外推基准）

    // ---- 生成 center_line[]（显示 + err 共用） ----
    for (int y = pho_h - 1; y >= 0; y--)
    {
        int l = (int)l_border[y];
        int r = (int)r_border[y];
        int c;

        /* 双边真 → 中点（记录宽度为外推基准）；
         * 单边真 → 最近真实路宽外推（不再拉到屏幕边缘）；
         * 都无 → 白像素重心 */
        if (l != BORDER_INVALID && r != BORDER_INVALID
            && l_border_exist[y] && r_border_exist[y])
        {
            c = (l + r) / 2;
            last_real_width = r - l;
            if (last_real_width < 20)  last_real_width = 20;
            if (last_real_width > 160) last_real_width = 160;
        }
        else if (l != BORDER_INVALID && l_border_exist[y])
        {
            c = l + last_real_width / 2;
        }
        else if (r != BORDER_INVALID && r_border_exist[y])
        {
            c = r - last_real_width / 2;
        }
        else
        {
            /* 双边都缺 → 白像素重心。找不到就是真没赛道，标 INVALID 不画绿线。
             * 底部连通滤波已清除环境白，白像素重心安全可靠。 */
            int sum = 0, cnt = 0;
            for (int x = 0; x < pho_w; x++)
            {
                if (Image_Used[y][x] == 255)
                { sum += x; cnt++; }
            }
            if (cnt >= 3)
                c = (sum + cnt / 2) / cnt;
            else
                c = BORDER_INVALID;
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

    // ---- ASC 多行加权平均（自适应预瞄，直道看远/弯道看近）----
    // 范围 100→calc_adaptive_far()，权重 2.0→2.5 线性递增，远行权重大=预瞄靠前
    int y_near = 100;
    int y_far  = calc_adaptive_far();
    asc_range_dbg = y_near - y_far;

    float total_dev = 0.0f, total_w = 0.0f;
    float w = 2.0f;
    float w_step = 0.5f / (float)(y_near - y_far);   // 2.0 → 2.5 远重近轻
    int   valid_cnt = 0;       // 实际有效行数

    for (int y = y_near; y >= y_far; y--)
    {
        uint8 c = center_line[y];
        if (c != BORDER_INVALID && c >= 3 && c <= pho_w_max - 3)
        {
            total_dev += ((float)c - (float)pho_center_x) * w;
            total_w   += w;
            valid_cnt++;
        }
        w += w_step;   // 每行递增，远行权重大
    }

    int theo_rows = y_near - y_far;
    if (total_w > 0.0f)
    {
        err = total_dev / total_w;
        // 入弯时赛道行数 < 理论行数 → err偏小 → 按比例补偿
        if (valid_cnt > 0 && valid_cnt < theo_rows)
        {
            float scale = (float)theo_rows / (float)valid_cnt;
            if (scale > 2.5f) scale = 2.5f;   // 安全钳，最多放大2.5倍
            err *= scale;
        }
    }
    // else: 总权重为0 → err保留上帧不变
}

/* ---- 旧 fallback_scan 已删除，sweep_boundaries() 内置兜底处理 ---- */

/* ================================================================
 * 十字识别 — 辅助函数
 * ================================================================ */
static uint8 cross_limit_u8(int32 value, uint8 lower, uint8 upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return (uint8)value;
}

static uint8 cross_abs_diff(uint8 a, uint8 b)
{
    return (a >= b) ? (a - b) : (b - a);
}

/* ================================================================
 * 十字用二值边缘搜索（基于 Image_Used[][], 0=黑 255=白）
 *
 *  左边界：从 start 向 end（递减）搜 B→W 或直接黑像素
 *  右边界：从 start 向 end（递增）搜 W→B 或直接黑像素
 * ================================================================ */
static uint16 cross_find_left_edge_bin(uint8 row, int16 start, int16 end, bool *found)
{
    *found = false;
    start = (int16)cross_limit_u8(start, 0, MT9V03X_W - 1);
    end   = (int16)cross_limit_u8(end,   0, MT9V03X_W - 1);
    if (start < end) { int16 t = start; start = end; end = t; }

    for (int16 col = start; col >= end; col--)
    {
        if (Image_Used[row][col] == 0)
        { *found = true; return (uint16)col; }
        if (col > 0 && Image_Used[row][col] == 255 && Image_Used[row][col - 1] == 0)
        { *found = true; return (uint16)col; }
    }
    return 0;
}

static uint16 cross_find_right_edge_bin(uint8 row, int16 start, int16 end, bool *found)
{
    *found = false;
    start = (int16)cross_limit_u8(start, 0, MT9V03X_W - 1);
    end   = (int16)cross_limit_u8(end,   0, MT9V03X_W - 1);
    if (start > end) { int16 t = start; start = end; end = t; }

    for (int16 col = start; col <= end; col++)
    {
        if (Image_Used[row][col] == 0)
        { *found = true; return (uint16)col; }
        if (col < MT9V03X_W - 1 && Image_Used[row][col] == 255 && Image_Used[row][col + 1] == 0)
        { *found = true; return (uint16)col; }
    }
    return MT9V03X_W - 1;
}

/* ================================================================
 * 车道行有效性：独立扫描后双边都有效、分别在种子列两侧、间距合理
 * ================================================================ */
static bool cross_lane_row_valid(uint8 row)
{
    uint16 lc = cross_scan_left[row];
    uint16 rc = cross_scan_right[row];
    if (!cross_scan_left_valid[row] || !cross_scan_right_valid[row])
        return false;
    if (lc + CROSS_CORNER_SIDE_MARGIN >= cross_scan_seed_col)  return false;
    if (rc <= cross_scan_seed_col + CROSS_CORNER_SIDE_MARGIN)  return false;
    if (lc >= rc) return false;
    uint16 gap = rc - lc;
    return gap >= CROSS_CORNER_MIN_GAP && gap <= CROSS_LANE_ROW_MAX_GAP;
}

/* ================================================================
 * 全宽白带检测 — 存在连续3行近全屏白，说明遇到十字横道
 * ================================================================ */
static bool cross_has_white_band(void)
{
    uint8 consec = 0;
    for (uint16 row = CROSS_FULL_WIDTH_TOP; row <= CROSS_FULL_WIDTH_BOTTOM; row++)
    {
        uint16 white_cnt = 0;
        uint8  left_w = 0, right_w = 0;
        for (uint16 col = 0; col < MT9V03X_W; col++)
        {
            if (Image_Used[row][col] == 255)
            {
                white_cnt++;
                if (col < CROSS_EDGE_SAMPLE_COLS) left_w++;
                if (col >= MT9V03X_W - CROSS_EDGE_SAMPLE_COLS) right_w++;
            }
        }
        if (white_cnt >= CROSS_FULL_WIDTH_WHITE_MIN
            && left_w  >= CROSS_EDGE_WHITE_MIN
            && right_w >= CROSS_EDGE_WHITE_MIN)
        {
            consec++;
            if (consec >= CROSS_FULL_WIDTH_ROWS) return true;
        }
        else
        {
            consec = 0;
        }
    }
    return false;
}

/* ================================================================
 * 单侧拐点搜索 — 过渡带检测 + 切线匹配
 *
 *   1. 过渡带：上方无效行≥2 且 下方有效行≥4 → 纵道重新出现
 *   2. 切线：在切线搜索区内找边缘斜率最接近"拐点→底角连线"
 *      的点作为拐点
 * ================================================================ */
static bool cross_find_corner(bool left_side, uint8 *corner_col, uint8 *corner_row)
{
    const uint16 *edge  = left_side ? cross_scan_left  : cross_scan_right;
    const bool   *valid = left_side ? cross_scan_left_valid : cross_scan_right_valid;
    int32 bottom_col = left_side ? 0 : (MT9V03X_W - 1);
    bool transition_found = false;
    bool tangent_found = false;
    uint16 best_score = 0xFFFF;
    uint8  best_col = 0, best_row = 0;

    /* 阶段一：过渡带检测 */
    for (uint16 row = CROSS_CORNER_SEARCH_TOP; row <= CROSS_CORNER_SEARCH_BOTTOM; row++)
    {
        if (!cross_lane_row_valid((uint8)row)) continue;

        uint8 above_invalid = 0, below_valid = 0;
        for (uint16 s = row - CROSS_CONTEXT_ROWS; s < row; s++)
            if (!cross_lane_row_valid((uint8)s)) above_invalid++;
        for (uint16 s = row; s <= row + CROSS_CONTEXT_ROWS && s <= CROSS_ROI_BOTTOM; s++)
            if (cross_lane_row_valid((uint8)s)) below_valid++;

        if (above_invalid >= CROSS_ABOVE_INVALID_MIN
            && below_valid >= CROSS_BELOW_VALID_MIN)
        {
            transition_found = true;
            break;
        }
    }
    if (!transition_found) return false;

    /* 阶段二：切线匹配 */
    for (uint16 row = CROSS_TANGENT_SEARCH_TOP; row <= CROSS_TANGENT_SEARCH_BOTTOM; row++)
    {
        bool continuous = true;
        /* 窗口内边沿连续有效 */
        for (uint16 s = row - CROSS_TANGENT_HALF_WINDOW;
             s <= row + CROSS_TANGENT_HALF_WINDOW; s++)
        {
            if (!valid[s]
                || edge[s] <= CROSS_TANGENT_EDGE_MARGIN
                || edge[s] + CROSS_TANGENT_EDGE_MARGIN >= MT9V03X_W)
            { continuous = false; break; }
        }
        if (!continuous) continue;

        /* 窗口内无大阶跃 */
        for (uint16 s = row - CROSS_TANGENT_HALF_WINDOW;
             s < row + CROSS_TANGENT_HALF_WINDOW; s++)
        {
            int32 step = (int32)edge[s + 1] - edge[s];
            if (step < 0) step = -step;
            if (step > CROSS_TANGENT_MAX_STEP) { continuous = false; break; }
        }
        if (!continuous) continue;

        /* 局部斜率 vs. 到图像底角的连线斜率 */
        int32 local_slope  = ((int32)edge[row + CROSS_TANGENT_HALF_WINDOW]
                           -  edge[row - CROSS_TANGENT_HALF_WINDOW])
                           * CROSS_SLOPE_SCALE
                           / (int32)(2 * CROSS_TANGENT_HALF_WINDOW);
        int32 repair_slope = (bottom_col - edge[row]) * CROSS_SLOPE_SCALE
                           / (int32)((MT9V03X_H - 1) - row);
        int32 diff = local_slope - repair_slope;
        if (diff < 0) diff = -diff;

        uint16 score = (uint16)diff;
        if (!tangent_found || score < best_score)
        {
            tangent_found = true;
            best_score = score;
            best_col = (uint8)edge[row];
            best_row = (uint8)row;
        }
    }
    if (!tangent_found) return false;

    *corner_col = best_col;
    *corner_row = best_row;
    return true;
}

/* ================================================================
 * 拐点对提取 — 独立逐行扫描 + 双边拐点匹配
 * ================================================================ */
static bool cross_find_pair(uint8 seed_col,
                            uint8 *l_col, uint8 *l_row,
                            uint8 *r_col, uint8 *r_row)
{
    cross_scan_seed_col = seed_col;
    /* 每行都从同一种子列独立搜索，避免边线跟踪在横道上漂移 */
    for (uint16 row = CROSS_ROI_TOP; row <= CROSS_ROI_BOTTOM; row++)
    {
        cross_scan_left[row] = cross_find_left_edge_bin(
            (uint8)row, seed_col, 0, &cross_scan_left_valid[row]);
        cross_scan_right[row] = cross_find_right_edge_bin(
            (uint8)row, seed_col, MT9V03X_W - 1, &cross_scan_right_valid[row]);
    }

    bool lf = cross_find_corner(true,  l_col, l_row);
    bool rf = cross_find_corner(false, r_col, r_row);
    if (!lf || !rf || *l_col >= *r_col) return false;

    uint8 row_diff = cross_abs_diff(*l_row, *r_row);
    uint16 gap     = (uint16)*r_col - *l_col;
    uint8  mid     = (uint8)(((uint16)*l_col + *r_col) / 2);

    return gap >= CROSS_CORNER_MIN_GAP
        && gap <= CROSS_CORNER_MAX_GAP
        && row_diff <= CROSS_CORNER_MAX_ROW_DIFF
        && cross_abs_diff(mid, seed_col) <= CROSS_MID_MAX_OFFSET;
}

/* ================================================================
 * 十字补线 — 两个上拐点分别连接图像左下角和右下角
 * ================================================================ */
static void cross_repair(void)
{
    if (cross_state != CROSS_STATE_DETECTED || !cross_corners_valid)
        return;

    /* 两拐点从同一行开始补线 */
    uint8 start_row = (cross_left_row > cross_right_row)
                    ? cross_left_row : cross_right_row;

    for (uint16 row = start_row; row < MT9V03X_H; row++)
    {
        /* 左拐点 → (0, 底行) */
        {
            int32 rs = (int32)(MT9V03X_H - 1) - cross_left_row;
            int32 ro = (int32)row - cross_left_row;
            int32 cs = (int32)0 - cross_left_col;
            int32 c  = cross_left_col;
            if (rs > 0) c += (cs * ro) / rs;
            l_border[row] = (uint8)cross_limit_u8(c, 0, MT9V03X_W - 1);
            l_border_exist[row] = 1;
        }
        /* 右拐点 → (pho_w_max, 底行) */
        {
            int32 rs = (int32)(MT9V03X_H - 1) - cross_right_row;
            int32 ro = (int32)row - cross_right_row;
            int32 cs = (int32)(MT9V03X_W - 1) - cross_right_col;
            int32 c  = cross_right_col;
            if (rs > 0) c += (cs * ro) / rs;
            r_border[row] = (uint8)cross_limit_u8(c, 0, MT9V03X_W - 1);
            r_border_exist[row] = 1;
        }
    }
}

/* ================================================================
 * 十字检测主函数 — 状态机 + 确认/保持
 * ================================================================ */
static void cross_detect(void)
{
    /* 冷却期：跳过检测，逐帧递减 */
    if (cross_cooldown_count > 0) {
        cross_cooldown_count--;
        return;
    }

    uint8  lc = 0, lr = 0, rc = 0, rr = 0;
    bool   pair_ok = false;

    if (cross_has_white_band())
    {
        uint8 seed = (uint8)((l_border[MT9V03X_H - 1] + r_border[MT9V03X_H - 1]) / 2);
        if (seed < 5 || seed > MT9V03X_W - 5)
            seed = MT9V03X_W / 2;

        pair_ok = cross_find_pair(seed, &lc, &lr, &rc, &rr);

        /* 斜入兜底：参考列可能落入侧向支路，用画面中心重试 */
        if (!pair_ok && seed != MT9V03X_W / 2)
            pair_ok = cross_find_pair(MT9V03X_W / 2, &lc, &lr, &rc, &rr);
    }

    if (pair_ok)
    {
        cross_left_col  = lc; cross_left_row  = lr;
        cross_right_col = rc; cross_right_row = rr;
        cross_corners_valid = true;
        cross_missed_count  = 0;
        if (cross_confirm_count < CROSS_CONFIRM_FRAMES)
            cross_confirm_count++;
        cross_state = (cross_confirm_count >= CROSS_CONFIRM_FRAMES)
                    ? CROSS_STATE_DETECTED : CROSS_STATE_CANDIDATE;
        cross_active = (cross_state == CROSS_STATE_DETECTED);
    }
    else if (cross_state == CROSS_STATE_DETECTED
             && cross_missed_count < CROSS_HOLD_MISSED_FRAMES)
    {
        cross_missed_count++;
    }
    else
    {
        /* 离开十字 → 启动冷却防重触发 */
        if (cross_state == CROSS_STATE_DETECTED)
            cross_cooldown_count = CROSS_COOLDOWN_FRAMES;
        cross_state = CROSS_STATE_NONE;
        cross_active = false;
        cross_corners_valid = false;
        cross_confirm_count = 0;
        cross_missed_count  = 0;
    }
}

/* ================================================================
 * 十字复位 — 发车时清零冷却和状态
 * ================================================================ */
void cross_reset(void)
{
    cross_state          = CROSS_STATE_NONE;
    cross_active         = false;
    cross_corners_valid  = false;
    cross_confirm_count  = 0;
    cross_missed_count   = 0;
    cross_cooldown_count = 0;
    zebra_count          = 0;
    zebra_prev           = false;
}


/* ================================================================
 * 斑马线检测 — 5像素多数表决滤波
 *
 *   对横向 ±ZEBRA_FILTER_RADIUS 窗口做多数表决，
 *   消除单/双像素噪点后再判断该列是否为白。
 * ================================================================ */
static bool zebra_pixel_is_white(uint8 row, uint16 col)
{
    uint16 start = (col > ZEBRA_FILTER_RADIUS)
                 ? col - ZEBRA_FILTER_RADIUS : 0;
    uint16 end   = col + ZEBRA_FILTER_RADIUS;
    uint8  white = 0, total = 0;

    if (end >= MT9V03X_W) end = MT9V03X_W - 1;

    for (uint16 c = start; c <= end; c++)
    {
        if (Image_Used[row][c] == 255) white++;
        total++;
    }
    return (white * 2 >= total + 1);  /* 多数：≥ceil(n/2) */
}

/* ================================================================
 * 斑马线行有效性 — 检查单行是否含规则黑白条纹
 *
 *   条件：白像素占比 25%~75%、跳变 ≥10、
 *         黑白连续段各 ≥5（段宽 ≥4）
 * ================================================================ */
static bool zebra_row_is_valid(uint8 row)
{
    bool   last_white  = zebra_pixel_is_white(row, 0);
    uint16 white_count = last_white ? 1 : 0;
    uint16 run_width   = 1;
    uint8  transitions = 0;
    uint8  black_runs  = 0;
    uint8  white_runs  = 0;

    for (uint16 col = 1; col < MT9V03X_W; col++)
    {
        bool cur = zebra_pixel_is_white(row, col);
        if (cur) white_count++;

        if (cur == last_white)
        {
            run_width++;
            continue;
        }

        /* 颜色跳变 */
        transitions++;
        if (run_width >= ZEBRA_RUN_WIDTH_MIN)
        {
            if (last_white) white_runs++; else black_runs++;
        }
        last_white = cur;
        run_width = 1;
    }

    /* 收尾最后一段 */
    if (run_width >= ZEBRA_RUN_WIDTH_MIN)
    {
        if (last_white) white_runs++; else black_runs++;
    }

    return ((uint32)white_count * 100 >= (uint32)MT9V03X_W * ZEBRA_WHITE_RATIO_MIN)
        && ((uint32)white_count * 100 <= (uint32)MT9V03X_W * ZEBRA_WHITE_RATIO_MAX)
        && (transitions   >= ZEBRA_TRANSITIONS_MIN)
        && (black_runs    >= ZEBRA_BLACK_RUNS_MIN)
        && (white_runs    >= ZEBRA_WHITE_RUNS_MIN);
}

/* ================================================================
 * 斑马线检测 — 底部3条采样线，≥2条有效则确认
 *
 *   采样行：119, 114, 109（最底行向上每5行一条）
 *   保持机制：漏检≤2帧时维持上一帧状态
 * ================================================================ */
static void zebra_detect(void)
{
    uint8 valid_rows = 0;

    for (uint8 s = 0; s < ZEBRA_SAMPLE_ROWS; s++)
    {
        uint8 row = MT9V03X_H - 1 - s * ZEBRA_SAMPLE_ROW_STEP;
        if (zebra_row_is_valid(row))
            valid_rows++;
    }

    if (valid_rows >= ZEBRA_VALID_ROWS_MIN)
    {
        zebra_detected      = true;
        zebra_missed_count  = 0;
    }
    else if (zebra_detected && zebra_missed_count < ZEBRA_HOLD_MISSED_FRAMES)
    {
        zebra_missed_count++;
    }
    else
    {
        zebra_detected      = false;
        zebra_missed_count  = 0;
    }
}


/* ================================================================
 * 底部连通滤波：只保留与图像底部连通的白像素（赛道），清除环境白
 *
 * 算法：自底向上逐行传播连通标记
 *   1. 底部行：所有白像素初始标记为连通
 *   2. 向上逐行：膨胀上一行的连通标记（±2列），当前行只保留
 *      与膨胀标记重叠的白像素，不连通的清零（正扫中向左传播连通）
 *
 * 优势：±2列膨胀容忍小间断，白墙/反光/场边杂物不与底部连通 → 被滤除
 * ================================================================ */
static void filter_bottom_connected(void)
{
    uint8 connected[pho_w];
    uint8 prev[pho_w];

    /* 底部行：所有白像素初始标记为连通 */
    for (int x = 0; x < pho_w; x++)
        connected[x] = (Image_Used[pho_h - 1][x] == 255);

    /* 向上逐行传播 */
    for (int y = pho_h - 2; y >= 0; y--)
    {
        /* 保存上一行连通标记（合并循环在读connected的同时写它，必须用副本） */
        for (int x = 0; x < pho_w; x++)
            prev[x] = connected[x];

        /* Pass 1: 合并 膨胀+过滤+正扫（±2列膨胀，从prev读） */
        for (int x = 0; x < pho_w; x++)
        {
            if (Image_Used[y][x] == 255)
            {
                /* 检查上方连通：prev[x] ±2 列 或 左边已连通 */
                if (prev[x]
                    || (x > 0        && prev[x - 1])
                    || (x > 1        && prev[x - 2])
                    || (x < pho_w - 1 && prev[x + 1])
                    || (x < pho_w - 2 && prev[x + 2])
                    || (x > 0        && connected[x - 1]))
                {
                    connected[x] = 1;
                }
                else
                {
                    connected[x] = 0;
                    Image_Used[y][x] = 0;
                }
            }
            else
            {
                connected[x] = 0;
            }
        }
    }
}

/* ================================================================
 * 出界检测：最底行全黑 → 赛道出画
 *   斑马线优先：检测到斑马线时强制不出界
 * ================================================================ */
static int out_of_bounds_check(void)
{
    if (zebra_detected) return 0;  /* 斑马线优先 */

    int y_bot = pho_h - 1;
    for (int x = 0; x < pho_w; x++)
        if (Image_Used[y_bot][x] == 255)
            return 0;
    return 1;
}

/* ================================================================
 * 搜线主函数
 *
 * 流程：
 *   1. Otsu 自适应阈值 + 一次性二值化 → Image_Used[][]
 *   2. 底部连通滤波 → 只留与底部连通的白
 *   3. 斑马线检测 → zebra_detected（优先于出界判定）
 *   4. 出界检测 → 停车（斑马线时强制不出界）
 *   5. sweep_boundaries → 逐行扫描边界
 *   6. cross_detect + cross_repair → 十字补线（覆盖被横道干扰的边界）
 *   7. pho_center → 中线 + 偏差
 * ================================================================ */
void vis_deal(void)
{
    /* ---- Otsu 自适应阈值（每5帧重算，EMA滤波抑制跳变）---- */
    static int cached_th = -1;
    static int frame_since_otsu = 0;
    frame_since_otsu++;
    if (frame_since_otsu >= 5)
    {
        int th = compute_otsu();
        if (th > 0)
        {
            if (cached_th < 0)
                cached_th = th;
            else
                cached_th = (int)(0.3f * (float)th + 0.7f * (float)cached_th);
        }
        frame_since_otsu = 0;
    }

    /* ---- 一次性二值化整张图 ---- */
    int th = (cached_th >= 0) ? cached_th : 128;
    for (int y = 0; y < pho_h; y++)
        for (int x = 0; x < pho_w; x++)
            Image_Used[y][x] = (mt9v03x_image[y][x] > th) ? 255 : 0;

    /* ---- 底部连通滤波 ---- */
    filter_bottom_connected();

    /* ---- 斑马线检测（先于出界判定，斑马线不出界）---- */
    zebra_detect();

    /* 斑马线计数停车：检测到2次上升沿→终点→停车 */
    {
        if (zebra_detected && !zebra_prev) zebra_count++;
        zebra_prev = zebra_detected;
        if (zebra_count >= 2) {
            control_state_stop();
            zebra_count = 0; zebra_prev = false;
            return;
        }
    }

    /* ---- 出界检测：连续10帧底行全黑才停车 ---- */
    {
        static uint8 oob_cnt = 0;
        if (out_of_bounds_check()) {
            oob_cnt++;
            if (oob_cnt >= 5) {
                control_state_stop();
                oob_cnt = 0;
                return;
            }
        } else {
            oob_cnt = 0;
        }
    }

    /* ---- 清空边界数组 ---- */
    for (int y = 0; y < pho_h; y++)
    {
        l_border[y] = BORDER_INVALID;
        r_border[y] = BORDER_INVALID;
        l_border_exist[y] = 0;
        r_border_exist[y] = 0;
    }

    /* ---- 逐行扫描边界（替代种子生长 + pho_border + 补全）---- */
    sweep_boundaries();

    /* ---- 十字检测 + 补线（覆盖被横道干扰的边界行）---- */
    cross_detect();
    cross_repair();

    /* ---- 中线 + 偏差 ---- */
    pho_center();

    vis_frame_ready = 1;
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

        // 本行双边都是真边界 → 红；缺一边或含出画伪边界 → 黄
        int both = (lb != BORDER_INVALID && rb != BORDER_INVALID
                    && l_border_exist[y] && r_border_exist[y]);
        int col  = both ? RGB565_RED : RGB565_YELLOW;

        if (lb != BORDER_INVALID)
        {
            ips200_draw_point((uint16)lb, (uint16)dy, col);
            if (lb < pho_w_max) ips200_draw_point((uint16)(lb + 1), (uint16)dy, col);
        }
        if (rb != BORDER_INVALID)
        {
            ips200_draw_point((uint16)rb, (uint16)dy, col);
            if (rb > pho_w_min) ips200_draw_point((uint16)(rb - 1), (uint16)dy, col);
        }

        // 中线（绿色）
        uint8 c = center_line[y];
        if (c != BORDER_INVALID)
        {
            ips200_draw_point(c, (uint16)dy, RGB565_GREEN);
            if (c < pho_w_max) ips200_draw_point((uint16)(c + 1), (uint16)dy, RGB565_GREEN);
        }
    }

    /* ---- 十字拐点标记（调试用）---- */
    if (cross_corners_valid)
    {
        int16 lx = (int16)cross_left_col;
        int16 ly = (int16)cross_left_row + BIN_PARAM_H;
        int16 rx = (int16)cross_right_col;
        int16 ry = (int16)cross_right_row + BIN_PARAM_H;

        /* 左上拐点：品红色十字 */
        ips200_draw_line((uint16)(lx - 3), (uint16)ly, (uint16)(lx + 3), (uint16)ly, RGB565_MAGENTA);
        ips200_draw_line((uint16)lx, (uint16)(ly - 3), (uint16)lx, (uint16)(ly + 3), RGB565_MAGENTA);
        /* 右上拐点：青色十字 */
        ips200_draw_line((uint16)(rx - 3), (uint16)ry, (uint16)(rx + 3), (uint16)ry, RGB565_CYAN);
        ips200_draw_line((uint16)rx, (uint16)(ry - 3), (uint16)rx, (uint16)(ry + 3), RGB565_CYAN);
    }

    /* ---- 斑马线 / 十字 状态文字 ---- */
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(0, 160, "ZEBRA:");
    ips200_show_string(56, 160, zebra_detected ? "YES" : "NO ");
    ips200_show_string(88, 160, "CROSS:");
    if (cross_state == CROSS_STATE_DETECTED)
        ips200_show_string(144, 160, "YES");
    else if (cross_state == CROSS_STATE_CANDIDATE)
        ips200_show_string(144, 160, "CAND");
    else
        ips200_show_string(144, 160, "NONE");
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
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
