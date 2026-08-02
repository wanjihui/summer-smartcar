
#include "config.h"

 border_line l_border;                      // 左边界，每行一个列号
 border_line r_border;                      // 右边界
 border_line center_line;                   // 中线（边框平滑 + 中线平滑后，绘图和err共用）
 uint8 l_border_exist[pho_h];               // 左边界该行是否真实存在（1=真边界 0=出画伪边界/无）
 uint8 r_border_exist[pho_h];               // 右边界该行是否真实存在

 volatile float err;                        // 中线偏离图像中心的像素均值，>0偏右
 volatile uint8_t vis_frame_ready;          // 新帧处理完成，显示层可刷新
 volatile uint8  straight_dbg  = 0;          // 诊断：当前帧直线判定结果
 volatile int16  asc_range_dbg = 0;          // 诊断：ASC窗口行数（100-y_far）

 uint8 asc_far = DEFAULT_ASC_FAR;             // ASC采样远行（顶部行），菜单可调，默认10
uint8 straight_far = DEFAULT_STRAIGHT_FAR;    // 直线判定采样远行（顶部行），菜单可调，默认30
static int16 last_valid_width = 60;            // 最近双边真实路宽（逐行扫描外推基准）

/* ================================================================
 * Otsu 二值化图像缓存
 *   0=黑（背景）, 255=白（赛道）
 * ================================================================ */
uint8 Image_Used[pho_h][pho_w];

/* ================================================================
 * 大津法 (Otsu) — 下采样 4x，安全钳 20~235
 * 移植自 loongson_learn，double → float
 * ================================================================ */
static int compute_otsu(void)
{
    unsigned long hist[256] = {0};
    /* 只统计下半屏（行60~119），避免上半屏背景白色干扰阈值 */
    for (int y = pho_h / 2; y < pho_h; y += 2)
        for (int x = 0; x < pho_w; x += 2)
            hist[mt9v03x_image[y][x]]++;

    unsigned long total = (unsigned long)((pho_h - pho_h / 2 + 1) / 2) * ((pho_w + 1) / 2);
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
 * 4. ASC多行加权平均: 100→自适应far, 权重2.5→2.0递减(近重远轻), 均值=err
 *
 * 调参: far(ASC远行基准,默认10)+自适应分段偏移; 单边→最新真实路宽外推; 双侧丢线→锁存
 * ================================================================ */

/* ================================================================
 * 直道判定：检查中线是否接近竖直直线
 *
 * 用 center_line[] 替代左右边界分别判断：
 *   - 中线已含双边/单边/兜底逻辑，不依赖两侧边界同时有效
 *   - 中线已经 border_smooth 平滑，抗噪更好
 *   - 只需一条线的斜率+偏离检查，比旧版左右各6点更简洁
 *
 * y_start=100, y_end=straight_far（默认30）：中段区域，终点菜单可调
 * 返回: 1=直道  0=弯道
 * ================================================================ */
int is_straight(void)
{
    straight_dbg = 0;  // 默认非直道，确保每帧更新

    int y_start = 100, step = 5;
    int y_end = (int)straight_far;
    if (y_end < 5)  y_end = 5;
    if (y_end > 95) y_end = 95;

    /* 两端必须有效 */
    if (center_line[y_start] == BORDER_INVALID
        || center_line[y_end] == BORDER_INVALID)
        return 0;

    /* 中线斜率：|k| < 0.45 → (100-y_end)行内偏移 <0.45×行数 px */
    float kc = (float)(center_line[y_end] - center_line[y_start])
             / (float)(y_end - y_start);
    if (kc > 0.25f || kc < -0.25f) return 0;

    straight_dbg = 1;
    return 1;
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
 * sweep_boundaries — 逐行扫描替代种子生长
 *
 * 从底行最长白列出发，逐行向上扫边界：
 *   左边界: 从 prev_l ± range 搜 B→W→W（黑→白→白）= 真左边界
 *   右边界: 从 prev_r ± range 搜 W→B（白→黑）= 真右边界
 *   单边丢: last_valid_width 外推，exist=0
 *   双边丢: 中心区域白检测 → 十字继续 / 黑区5行停止
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

    // ---- ASC 多行加权平均（远行由自适应预瞄决定）----
    // 范围 100→自适应far（直道看远/弯道看近），权重 2.0→2.5 线性递增，远行权重大=预瞄靠前
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
 *   2. 手遮检测 → 停车
 *   3. 底部连通滤波 → 只留与底部连通的白
 *   4. sweep_boundaries → 逐行扫描边界（替代种子生长全流程）
 *   5. pho_center → 中线 + 偏差
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

    /* ---- 手遮摄像头 → 停车 ---- */
    if (camera_is_covered())
    {
        motor_stop();
        car_run = false;
        return;
    }

    /* ---- 底部连通滤波 ---- */
    filter_bottom_connected();

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

    /* ---- 中线 + 偏差 ---- */
    pho_center();

    /* 调试：每帧更新直线判定（不受car_run限制） */
    is_straight();

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
