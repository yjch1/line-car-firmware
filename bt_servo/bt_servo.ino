/* ============================================================================
 *  单路舵机 · 蓝牙 + USB 双路角度控制（D4）
 *  ---------------------------------------------------------------------------
 *  舵机驱动与 2.ino 完全一致：Timer2 比较中断（OCR2B）产生 50Hz，
 *  20.00ms 周期，2us 分辨率，0~270° 线性对应 500~2500us，
 *  平滑推进 240°/s，全程持续输出不卸力。
 *
 *  ★ 蓝牙与 USB 同时可用（两个口各自独立，互不干扰）：
 *      蓝牙 → D2(RX) / D3(TX)   SoftwareSerial
 *      USB  → D0(RX) / D1(TX)   硬件串口
 *    两路都能发指令，回显同时打到两路。
 *    （一个硬件串口接不了两个设备：UART 是推挽输出，蓝牙与 USB 芯片会互相对打，
 *      所以蓝牙必须挪到软件串口才能和 USB 并存。）
 *
 *  指令：发送角度数字，回车/换行结束；不发换行也行，静默 30ms 自动断帧
 *        90 / 135.5 / 270      超范围自动钳位
 *  上电停在 90°。
 *
 *  接线：
 *    舵机信号 → D4，舵机电源独立 5~6V 且与 Arduino 共地
 *    蓝牙 TXD → D2，蓝牙 RXD → D3
 *    USB       → 原口不动
 *
 *  ⚠ 不要用 Servo 库：它独占 Timer1。
 *  ⚠ 软串口收字节时会关中断约 1ms，若正好落在舵机脉冲高电平期间，该帧脉宽会被
 *    拉长约 1ms（舵机可能轻抖一下）。只在收到指令的那几毫秒发生，平时为零。
 *    要追求信号绝对干净，把 BT_USE_HW_SERIAL 改成 1，蓝牙回 D0/D1。
 * ============================================================================ */

/* 0 = 蓝牙 D2/D3，USB 保留 D0/D1（可同时双向使用）★ 推荐
 * 1 = 蓝牙 D0/D1（与实例代码一致），此时 USB 只能看输出、不能输入 */
#define BT_USE_HW_SERIAL  0
#define BT_BAUD           9600      // HC-05/06 出厂默认 9600

/* 无帧尾兼容：收完最后一个字节后静默超过这么久，就把已收内容当作一整帧解析。
 * 30ms 远大于 9600 下单字节的 1.04ms，不会把 "135" 切成 "1" "3" "5"。 */
#define FRAME_GAP_MS      30

#if BT_USE_HW_SERIAL
  #define BT  Serial
#else
  #include <SoftwareSerial.h>
  SoftwareSerial btSerial(2, 3);
  #define BT  btSerial
#endif

/* ★ 收帧上下文必须定义在【所有函数之前】：
 *   Arduino IDE 会在第一个函数前自动插入函数原型，若类型定义在后面，
 *   原型里的 RxCtx 就是"未声明" → 报 variable or field 'applyBuf' declared void。 */
typedef struct
{
    char          buf[12];
    uint8_t       idx;
    unsigned long lastMs;
} RxCtx;

#if BT_USE_HW_SERIAL
  static RxCtx rxMain;
#else
  static RxCtx rxBt, rxUsb;
#endif

/* ============================================================================
 *  舵机驱动（与 2.ino 同一套，单通道）
 * ========================================================================== */
#define SERVO_N        1

/* 时间基准：Timer2 预分频 /32 → 500kHz → 2us/tick；20ms = 10000 tick */
#define SERVO_CYCLE    10000
#define SERVO_SEG_MAX  240      // 8 位计数器单次最多排 240 tick（留回绕余量）
#define SERVO_ON0      4        // 起点不能是 0，否则写完 OCR2B 已被越过，要白等一圈

#define SERVO_DEG_FULL    270.0f
#define SERVO_DEG_TENTH   2700L
#define SERVO_US_0        500
#define SERVO_US_FULL     2500
/* 脉宽底端下限：0°=500us 压在行程端点会"歇力"，抬到 550us 保证有力矩。
 * 只夹底端：90°/180°/270° 一点不受影响，仅 0~6.75° 成为死区。 */
#define SERVO_US_MIN_OUT  550

#define SERVO_TK0         (uint16_t)(SERVO_US_0 / 2)
#define SERVO_TK_SPAN     (uint16_t)((SERVO_US_FULL - SERVO_US_0) / 2)
#define SERVO_TK_MIN_OUT  (uint16_t)(SERVO_US_MIN_OUT / 2)

/* 平滑推进：240°/s → 每 20ms 4.8° → 0~270° 约 1.13s
 * 想更快改 600（≈0.45s），更柔和改 120（≈2.25s）。 */
#define SERVO_SLEW_DPS    240
#define SERVO_STEP        ((int16_t)((long)SERVO_SLEW_DPS * 200L / 1000L))

#define SERVO_PIN         4
#define SERVO_HOME_DEG    90       // 上电位置

static volatile int16_t  svTgtTenth;          // 目标角（0.1°），主循环写
static volatile int16_t  svCurTenth;          // 当前角（0.1°），ISR 写
static volatile uint16_t svTicks;             // 当前脉宽（2us/tick）

static uint16_t svEvtAt[2 * SERVO_N + 1];     // on, off, 周期结束
static uint16_t svAbsCur = 0;
static uint8_t  svEvt    = 0;

/* 角度（0.1°）→ 脉宽（tick） */
static uint16_t servoAngleToTicks(int16_t tenth)
{
    if(tenth < 0)                tenth = 0;
    if(tenth > SERVO_DEG_TENTH)  tenth = (int16_t)SERVO_DEG_TENTH;

    uint16_t tk = (uint16_t)( SERVO_TK0 +
                   ((uint32_t)tenth * SERVO_TK_SPAN + (uint32_t)SERVO_DEG_TENTH / 2)
                       / (uint32_t)SERVO_DEG_TENTH );

    if(tk < SERVO_TK_MIN_OUT) tk = SERVO_TK_MIN_OUT;
    return tk;
}

/* ★★★ 对外唯一接口：设定角度 0~270，非阻塞，立刻返回
 *  返回值：实际生效的角度（已钳位）
 *  转动由 Timer2 中断按 SERVO_SLEW_DPS 平滑推进，中途再改目标会从当前姿态接上 */
float servoWrite(float deg)
{
    if(deg < 0.0f)           deg = 0.0f;
    if(deg > SERVO_DEG_FULL) deg = SERVO_DEG_FULL;

    int16_t t = (int16_t)(deg * 10.0f + 0.5f);

    uint8_t sreg = SREG;            // 保存/恢复而不是 interrupts()，嵌套安全
    cli();
    svTgtTenth = t;
    SREG = sreg;

    return (float)t * 0.1f;
}

/* 周期末尾推进一次角度。此时无脉冲输出，做除法不影响脉宽。
 * 必须 noinline：否则 32 位除法会把 r12~r31 全拖进 ISR 序言。 */
static void __attribute__((noinline)) servoTrack()
{
    int16_t cur = svCurTenth;
    int16_t d   = (int16_t)(svTgtTenth - cur);

    if(d >  SERVO_STEP) d =  SERVO_STEP;
    if(d < -SERVO_STEP) d = -SERVO_STEP;
    if(d == 0) return;                        // 已到位 → 零开销

    cur += d;
    svCurTenth = cur;
    svTicks    = servoAngleToTicks(cur);
}

static void servoPlan()
{
    uint16_t len = svTicks;
    if(len < SERVO_TK0) len = SERVO_TK0;
    svEvtAt[0] = SERVO_ON0;
    svEvtAt[1] = SERVO_ON0 + len;
    svEvtAt[2] = SERVO_CYCLE;
}

static void servoInit()
{
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW);

    /* 上电即到预设位：当前角 = 目标角，第一个周期就输出该角度，不做过渡 */
    int16_t home = (int16_t)((int16_t)SERVO_HOME_DEG * 10);
    svTgtTenth = home;
    svCurTenth = home;
    svTicks    = servoAngleToTicks(home);
    servoPlan();

    svAbsCur = 0;
    svEvt    = 0;

    uint8_t sreg = SREG;
    cli();
    TCCR2A = 0;                        // Normal 模式
    /* 预分频 /32：CS22 CS21 CS20 = 0 1 1
     * （写成 _BV(CS22)|_BV(CS21) 是 /256，整条时间轴会放大 8 倍，舵机收不到有效信号） */
    TCCR2B = _BV(CS21) | _BV(CS20);
    TCNT2  = 0;
    OCR2B  = 4;                        // 4 tick 后第一次比较中断
    TIMSK2 = _BV(OCIE2B);              // COMPA 留给 Tone.cpp，不能碰
    SREG = sreg;
}

ISR(TIMER2_COMPB_vect)
{
    /* ① 执行所有到点事件 */
    while(svEvt < 2 * SERVO_N && svEvtAt[svEvt] <= svAbsCur)
    {
        digitalWrite(SERVO_PIN, (svEvt & 1) ? LOW : HIGH);   // 0=上升沿 1=下降沿
        svEvt++;
    }

    /* ② 周期走完 → 拉低、推进角度、重排事件 */
    if(svEvt >= 2 * SERVO_N && svAbsCur >= SERVO_CYCLE)
    {
        digitalWrite(SERVO_PIN, LOW);
        servoTrack();
        svAbsCur = 0;
        svEvt    = 0;
        servoPlan();
    }

    /* ③ 排下一个比较点（8 位计数器，必须分段） */
    uint16_t target = (svEvt < 2 * SERVO_N) ? svEvtAt[svEvt] : SERVO_CYCLE;
    uint16_t d = (target > svAbsCur) ? (uint16_t)(target - svAbsCur) : 1;
    if(d > SERVO_SEG_MAX) d = SERVO_SEG_MAX;
    if(d < 4) d = 4;                   // 小于 4 tick 可能已被计数器越过

    OCR2B = (uint8_t)(TCNT2 + d);
    svAbsCur += d;
}

/* ============================================================================
 *  双路口令接收
 * ========================================================================== */
/* 解析一个纯数字角度；非法（含非数字字符、空串）返回 false */
static bool parseAngle(const char *s, float *out)
{
    uint8_t i = 0;
    while(s[i] == ' ') i++;

    bool neg = false;
    if(s[i] == '-' || s[i] == '+') { neg = (s[i] == '-'); i++; }

    float v   = 0.0f;
    bool  any = false;

    while(s[i] >= '0' && s[i] <= '9') { v = v * 10.0f + (s[i] - '0'); i++; any = true; }

    if(s[i] == '.')
    {
        i++;
        float m = 0.1f;
        while(s[i] >= '0' && s[i] <= '9')
        {
            v += (s[i] - '0') * m;
            m *= 0.1f;
            i++;
            any = true;
        }
    }

    if(!any) return false;
    while(s[i] == ' ') i++;
    if(s[i] != 0) return false;        // 尾部有垃圾 → 整帧丢弃

    *out = neg ? -v : v;
    return true;
}

static void reportOne(Stream &o, bool ok, float got)
{
    if(ok) { o.print(F("D4 -> ")); o.println(got, 1); }
    else     o.println(F("# bad: send a number 0-270"));
}

/* 一路应答：蓝牙必回；USB 存在时也回 */
static void report(bool ok, float got)
{
#if !BT_USE_HW_SERIAL
    reportOne(Serial, ok, got);
#endif
    reportOne(BT, ok, got);
}

/* 解析并执行一帧 */
static void applyBuf(RxCtx &c)
{
    if(c.idx == 0) return;

    c.buf[c.idx] = 0;
    c.idx = 0;

    float deg;
    bool ok = parseAngle(c.buf, &deg);
    report(ok, ok ? servoWrite(deg) : 0.0f);
}

/* 非阻塞收帧：遇 \r/\n 立即解析；没有帧尾则靠静默 FRAME_GAP_MS 断帧 */
static void pollPort(Stream &in, RxCtx &c)
{
    while(in.available() > 0)
    {
        char ch = (char)in.read();
        c.lastMs = millis();

        if(ch == '\r' || ch == '\n')
        {
            applyBuf(c);
        }
        else if((ch >= '0' && ch <= '9') || ch == '.' || ch == '-' || ch == '+' || ch == ' ')
        {
            if(c.idx < sizeof(c.buf) - 1) c.buf[c.idx++] = ch;
            else                          c.idx = 0;       // 超长 → 丢弃整帧
        }
        else
        {
            c.idx = 0;                                     // 非法字符 → 丢弃半帧
        }
    }

    if(c.idx > 0 && millis() - c.lastMs >= FRAME_GAP_MS) applyBuf(c);
}

void setup()
{
    servoInit();                    // 先把舵机拉起来，输出立刻稳定

#if !BT_USE_HW_SERIAL
    Serial.begin(9600);             // USB D0/D1
#endif
    BT.begin(BT_BAUD);              // 蓝牙 D2/D3

    /* 两路都打一行启动提示，方便确认哪一路通了 */
#if !BT_USE_HW_SERIAL
    Serial.println(F("# servo D4 ready. USB: D0/D1  BT: D2/D3, send 0-270"));
#endif
    BT.println(F("# servo D4 ready. send angle 0-270"));
}

void loop()
{
#if BT_USE_HW_SERIAL
    pollPort(BT, rxMain);
#else
    pollPort(BT,     rxBt);         // 蓝牙 D2/D3
    pollPort(Serial, rxUsb);        // USB  D0/D1
#endif
}
