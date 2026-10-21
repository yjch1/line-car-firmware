/* ============================================================================
 *  舵机单路隔离测试 —— 用来判定「D8 / D12 无力」到底在代码还是在硬件
 *  ---------------------------------------------------------------------------
 *  特点：
 *    · 完全不用定时器、不用中断、不用软串口、不用超声波/编码器/PID
 *      → 排除一切"与主程序交互"的可能性
 *    · 50Hz 由 delayMicroseconds 直接产生，脉宽 500~2500us
 *    · 四路同时输出，但一次只让一路动，其余保持 90°，便于逐路观察
 *
 *  测试流程（每步 2.5 秒，串口打印当前步骤）：
 *    1. 四路全部 90°        ← 基线，确认四路都在收信号
 *    2. 只有 D4  → 135°     ← 已知正常，作为对照组
 *    3. 只有 D7  → 135°     ← 已知正常
 *    4. 只有 D8  → 135°     ← 待测
 *    5. 只有 D8  →  45°     ← 待测（换个角度）
 *    6. 只有 D12 → 135°     ← 待测
 *    7. 只有 D12 →  45°     ← 待测（换个角度）
 *    8. 四路回 90°，循环
 *
 *  判读：
 *    · 步骤 4/5 中 D8 能动 → 引脚与接线正常，问题在原程序的交互或 0° 端点
 *    · 步骤 4/5 中 D8 完全不动，但步骤 2 的 D4 能动
 *        → 代码无关，是 D8 这个引脚/这颗舵机/这条线的问题
 *    · 注意观察舵机在 45° 与 135° 时是否"有劲"（用手轻推能顶住）
 * ============================================================================ */

const uint8_t PINS[4] = {4, 7, 8, 12};      // D4 / D7 / D8 / D12

#define US_MIN   500                        // 0°   对应脉宽
#define US_MAX  2500                        // 270° 对应脉宽
#define FRAME_US 20000UL                    // 20ms 一帧 = 50Hz

/* 角度 → 脉宽（0~270° 线性映射到 500~2500us） */
static uint16_t degToUs(float deg)
{
    if(deg < 0.0f)   deg = 0.0f;
    if(deg > 270.0f) deg = 270.0f;
    return (uint16_t)(US_MIN + deg * (US_MAX - US_MIN) / 270.0f + 0.5f);
}

/* 输出一帧：四路依次（错开）拉高，再补足到 20ms */
static void outputFrame(uint16_t us0, uint16_t us1, uint16_t us2, uint16_t us3)
{
    uint16_t us[4] = {us0, us1, us2, us3};
    unsigned long t0 = micros();

    for(uint8_t i = 0; i < 4; i++)
    {
        digitalWrite(PINS[i], HIGH);
        delayMicroseconds(us[i]);
        digitalWrite(PINS[i], LOW);
    }

    unsigned long used = micros() - t0;
    if(used < FRAME_US) delayMicroseconds(FRAME_US - used);
}

/* 保持指定角度 duration_ms 毫秒；只有 target 通道取 deg，其余保持 90° */
static void hold(uint8_t target, float deg, uint16_t duration_ms)
{
    uint16_t us[4] = {degToUs(90), degToUs(90), degToUs(90), degToUs(90)};
    if(target < 4) us[target] = degToUs(deg);

    unsigned long end = millis() + duration_ms;
    while((long)(millis() - end) < 0) outputFrame(us[0], us[1], us[2], us[3]);
}

static void announce(const char *what)
{
    Serial.println(what);
}

void setup()
{
    Serial.begin(9600);

    for(uint8_t i = 0; i < 4; i++)
    {
        pinMode(PINS[i], OUTPUT);
        digitalWrite(PINS[i], LOW);
    }

    Serial.println(F("=== servo isolated test: D4/D7/D8/D12, 500-2500us @50Hz ==="));
    delay(500);
}

void loop()
{
    /* 1. 四路 90° 基线 */
    announce("1) ALL 90deg  (check all four are holding)");
    hold(4, 90, 2500);

    /* 2~3. 已知正常的两路，作对照 */
    announce("2) D4  -> 135deg");
    hold(0, 135, 2500);

    announce("3) D7  -> 135deg");
    hold(1, 135, 2500);

    /* 4~5. D8 两个角度 */
    announce("4) D8  -> 135deg   <<< TEST");
    hold(2, 135, 2500);

    announce("5) D8  ->  45deg   <<< TEST");
    hold(2, 45, 2500);

    /* 6~7. D12 两个角度 */
    announce("6) D12 -> 135deg   <<< TEST");
    hold(3, 135, 2500);

    announce("7) D12 ->  45deg   <<< TEST");
    hold(3, 45, 2500);

    /* 8. 回中位 */
    announce("8) ALL 90deg  (cycle end)");
    hold(4, 90, 2500);
}
