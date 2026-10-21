/* ============================================================================
 *  差速轮机器人 · 直线行驶闭环控制系统
 *  ---------------------------------------------------------------------------
 *  平台 : Arduino UNO / Nano (ATmega328P, 16MHz)
 *  目标 : 纠正电机差异、电池电压波动、左右轮径/齿比不一致造成的转向偏移
 *
 *  【控制结构】三层闭环，自下而上
 *    ① 内环  双轮速度环 (PI)   —— 20ms 定点调度，抗电池跌落/电机差异/摩擦
 *    ② 中环  差速环 (PI)       —— 航向误差 → 左右轮速度差，纠正持续跑偏
 *    ③ 外环  循迹环 (P+积分)   —— 三探头横向误差 → 转向修正，兜住真实路径
 *
 *  【状态变量】
 *    f  任务号 1~3   ← 串口 "1,x"
 *    r  现场号 1~3   ← 串口 "2,xx"
 *
 *  ========================================================================== */

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

/* ========================= 1. 硬件引脚 =========================
 *  ⚠ 引脚冲突警告（务必核对）
 *  Arduino 上 A3 与数字口 3 是【同一个引脚】(PC3)。
 *  本设计把中循迹探头移到 D4，避开与编码器 A3 的冲突。
 *  若你的实车已按旧接线，请核对下表并只改这一段。
 *  --------------------------------------------------------------- */
#define PIN_LINE_L     A4      // 左循迹探头（数字输出）
#define PIN_LINE_M     4       // 中循迹探头（原 D3 → 因与 A3 冲突改 D4）
#define PIN_LINE_R     A5      // 右循迹探头

#define PIN_ENC_L      A3      // 左编码器  PCINT11 / PC3
#define PIN_ENC_R      A2      // 右编码器  PCINT10 / PC2

#define PIN_ML_FWD     6       // 左轮正转 PWM (Motor-1 IN1)
#define PIN_ML_REV     5       // 左轮反转 PWM (Motor-1 IN2)
#define PIN_MR_FWD     10      // 右轮正转 PWM (Motor-0 IN1)
#define PIN_MR_REV     9       // 右轮反转 PWM (Motor-0 IN2)

#define PIN_TRIG       14      // 超声波 TRIG
#define PIN_ECHO       15      // 超声波 ECHO

/* ========================= 2. 机械与电气标定 =========================
 *  ★ 以下参数必须按你的实车实测标定，见文末"标定流程"
 *  --------------------------------------------------------------- */
#define ENC_PPR_L      30.0    // 左编码器每转边沿数（15孔盘 × 上下沿 = 30）
#define ENC_PPR_R      30.0    // 右编码器每转边沿数
#define WHEEL_DIA_L_CM 5.00    // 左轮有效直径 cm
#define WHEEL_DIA_R_CM 5.00    // 右轮有效直径 cm  ← 若与左不同，在此填真实值
#define TRACK_WIDTH_CM 14.0    // 左右轮接触点间距 cm（算航向用）

// 单脉冲行进距离（cm）：π·D / 每转边沿数
#define CM_PER_EDGE_L  (3.14159265 * WHEEL_DIA_L_CM / ENC_PPR_L)
#define CM_PER_EDGE_R  (3.14159265 * WHEEL_DIA_R_CM / ENC_PPR_R)

/* ========================= 3. 控制周期与滤波 ========================= */
#define CTRL_PERIOD_MS 20UL    // 内环周期 20ms = 50Hz
#define CTRL_HZ        (1000.0 / CTRL_PERIOD_MS)
#define SPEED_LP_A     0.50    // 一阶低通系数（测速平滑）0.3~0.7
#define STALL_US       150000UL// 超 150ms 无脉冲 → 该轮视为停转
#define PWM_MAX        255     // PWM 上限
#define PWM_MIN        0       // 启动死区补偿：低速不转时填 40~70，0=关闭

/* 超声波 */
#define STOP_NEAR_CM   13      // 近于此值判定为已抵近/过冲 → 仍按障碍处理
#define STOP_FAR_CM    17      // 远于此值视为无障碍
#define ECHO_TIMEOUT_US 2000UL // 回波超时（可测约 26~29cm）
#define ULTRA_PERIOD_US 20000UL

/* 横向纠偏（循迹环） */
#define LINE_ERR_GAIN  0.45    // 横向误差(cm) → 差速速度(cm/s) 的比例
#define HEADING_KP     1.20    // 航向误差 → 差速速度 比例
#define HEADING_KI     0.35    // 航向误差积分 → 消除持续跑偏
#define HEADING_CLAMP  14.0    // 中环差速输出限幅 cm/s
#define TRIM_MAX       6.0     // 静态机械补偿上限 cm/s（左正右负 = 右偏补偿）

/* 掉线保护 */
#define LOST_LINE_MS   1200    // 连续丢线超此时长 → 停车保护
#define OFF_LINE_CONF  3       // 横向纠偏连续命中此时长才生效（去抖）

/* ========================= 4. 任务流程配置 ========================= */
#define BOOT_WAIT_TASK 1       // 1=上电原地等 "1,x"
#define START_FWD_MS   1500UL  // 出发段时长
#define UNLOCK_DELAY_MS 3000UL // 进入巡线 3s 后解锁超声波

int f = 0;             // 任务号 1~3
int r = 0;             // 现场号 1~3

bool taskReady   = !BOOT_WAIT_TASK;
bool phase2      = false;
bool ultraArmed  = false;
bool waitingR    = false;
bool stopLocked  = false;
bool running     = false;   // 直线闭环是否使能

unsigned long fwdUntil    = 0;
unsigned long lineStart   = 0;   // 进入巡线时刻
unsigned long lostSince   = 0;

/* 调试阶段量 */
#define ST_WAIT_TASK 0
#define ST_START_FWD 1
#define ST_LINE      2
#define ST_STOP_READ 3
int stage = BOOT_WAIT_TASK ? ST_WAIT_TASK : ST_LINE;

/* ========================= 5. 编码器 =========================
 *  用 PCINT 引脚变化中断测"相邻边沿时间间隔"，而非窗口数脉冲。
 *  15孔盘一圈仅 30 边沿（0.52cm/脉冲），30cm/s 时 50ms 窗口只 3 个脉冲，
 *  窗口计数法量化误差 ±10cm/s，会把 PID 搅疯；测周期法分辨率约 0.007cm/s。
 * ========================================================== */
volatile long     encCntL = 0, encCntR = 0;      // 累计脉冲（带符号，测里程）
volatile uint32_t encDtL  = 0, encDtR  = 0;      // 相邻边沿间隔 us
volatile uint32_t encTmL  = 0, encTmR  = 0;      // 上一边沿时刻 us
volatile uint8_t  pcLast  = 0;
volatile int8_t   dirL = 1, dirR = 1;           // 指令方向（单路编码器无法自辨转向）

ISR(PCINT1_vect)
{
    uint32_t now = micros();
    uint8_t  cur = PINC;
    uint8_t  chg = cur ^ pcLast;
    pcLast = cur;

    if(chg & _BV(PC2)) {                         // A2 → 右轮
        encDtR = now - encTmR;
        encTmR = now;
        encCntR += dirR;
    }
    if(chg & _BV(PC3)) {                         // A3 → 左轮
        encDtL = now - encTmL;
        encTmL = now;
        encCntL += dirL;
    }
}

/* 由脉冲间隔换算带符号速度(cm/s)，自动处理停转衰减 */
static double readSpeed(volatile uint32_t *dt, volatile uint32_t *tm,
                        double cmPerEdge, int8_t dir)
{
    // 4字节变量在中断里被整体改写，主循环读会撕裂 → 快照后再算
    uint32_t d, tmSnap;
    uint8_t  s;
    noInterrupts();
    d      = *dt;
    tmSnap = *tm;
    s      = (uint8_t)dir;
    interrupts();

    uint32_t age = micros() - tmSnap;            // 距最后脉冲已过多久
    uint32_t t   = (d > age) ? d : age;          // 取大者：停转时随 age 平滑归零

    double mag = 0.0;
    if(t != 0 && t < STALL_US)
        mag = cmPerEdge * 1000000.0 / (double)t; // v = 距离/时间
    return s ? mag : -mag;
}

/* ========================= 6. 闭环控制器 =========================
 *  innerPI  单轮速度环
 *  headPI   差速环（航向保持）
 * ========================================================== */
struct WheelPI {
    double kp, ki;
    double integ;      // 积分项
    double lastMeas;   // 上一拍实测（微分/抗突跳用）

    void reset() { integ = 0.0; lastMeas = 0.0; }
    void gains(double p, double i) { kp = p; ki = i; }

    // 速度环：out = kp*e + ∫ki*e dt，抗积分饱和
    double step(double setpoint, double meas, double dtSec)
    {
        double e   = setpoint - meas;
        integ += ki * e * dtSec;
        if(integ >  PWM_MAX) integ =  PWM_MAX;
        if(integ < -PWM_MAX) integ = -PWM_MAX;

        double out = kp * e + integ;
        if(out >  PWM_MAX) out =  PWM_MAX;
        if(out < -PWM_MAX) out = -PWM_MAX;
        lastMeas = meas;
        return out;
    }
} wl, wr;

struct HeadPI {
    double kp, ki;
    double integ;

    void reset() { integ = 0.0; }
    void gains(double p, double i) { kp = p; ki = i; }

    // 航向环：out = kp*e + ∫ki*e dt，输出为左右轮速度差(cm/s)
    double step(double headingErr, double dtSec, double limit)
    {
        integ += ki * headingErr * dtSec;
        if(integ >  limit) integ =  limit;
        if(integ < -limit) integ = -limit;

        double out = kp * headingErr + integ;
        if(out >  limit) out =  limit;
        if(out < -limit) out = -limit;
        return out;
    }
} head;

/* 运行时可改参数 */
double kpSpeed = 4.0;    // 速度环比例
double kiSpeed = 10.0;   // 速度环积分
double trimBias = 0.0;   // 静态机械补偿：>0 左轮加速(右转)，<0 右轮加速
double baseSpeed = 30.0; // 闭环基速 cm/s

double tgtL = 0.0, tgtR = 0.0;
double spdL = 0.0, spdR = 0.0;   // 滤波后实测
int    pwmL = 0,    pwmR = 0;

/* 横向与航向估计 */
double lineErrCm  = 0.0;   // 横向偏移估计(cm)，正=车偏线右侧
double headingRad = 0.0;   // 航向角(rad)，由左右轮里程差积分得到
uint8_t offConf = 0;       // 横向纠偏连续命中计数

/* ========================= 7. 电机输出层 =========================
 *  全工程唯一写电机引脚处。pwm>=0 正转，pwm<0 反转，0 松手。
 *  ★ 左轮 = Motor-1(D6/D5)，右轮 = Motor-0(D10/D9)
 *    这是由原始 L() 语义反推的：被倒转的那一路才是左轮。
 * ========================================================== */
static void motorLR(int pL, int pR)
{
    if(pL >= 0) { analogWrite(PIN_ML_FWD, pL); analogWrite(PIN_ML_REV, 0);    dirL =  1; }
    else        { analogWrite(PIN_ML_FWD, 0);   analogWrite(PIN_ML_REV, -pL); dirL = -1; }

    if(pR >= 0) { analogWrite(PIN_MR_FWD, pR); analogWrite(PIN_MR_REV, 0);    dirR =  1; }
    else        { analogWrite(PIN_MR_FWD, 0);   analogWrite(PIN_MR_REV, -pR); dirR = -1; }
}

/* 运动原语：只设目标速度，不直接写 PWM */
static void stopAll()
{
    tgtL = tgtR = 0.0;
    motorLR(0, 0);            // 立刻松引脚，不等下一拍
    wl.reset(); wr.reset();
}

/* ========================= 8. 横向偏移与航向估计 =========================
 *  三探头 (a=左,b=中,c=右) → 横向偏差等级
 *    b亮            → 在线中央，偏置 0
 *    a+b亮 / b+c亮  → 轻度偏移，±1
 *    仅a / 仅c      → 重度偏移，±2
 *    全白           → 丢线
 *  横向偏差按"探头间距"换算成 cm，供循迹环使用。
 * ========================================================== */
#define SENSOR_GAP_CM 3.0     // 相邻探头中心距 cm

static uint8_t a = 0, b = 0, c = 0;

static double estimateLineErr(uint8_t la, uint8_t mb, uint8_t rc)
{
    if(mb) {                                  // 中间见线
        if(la && !rc)  return -SENSOR_GAP_CM; // 线在左 → 车偏右
        if(rc && !la)  return  SENSOR_GAP_CM; // 线在右 → 车偏左
        return 0.0;
    }
    if(la && !rc)  return -2.0 * SENSOR_GAP_CM;  // 仅左见线
    if(rc && !la)  return  2.0 * SENSOR_GAP_CM;  // 仅右见线
    return 0.0;
}

/* 航向角：由左右轮里程差 / 轮距 积分得到（差速积分法） */
static void updateHeading()
{
    static double distL = 0.0, distR = 0.0;
    long cl, cr;
    noInterrupts();
    cl = encCntL; cr = encCntR;
    interrupts();

    distL += (double)cl * CM_PER_EDGE_L;
    distR += (double)cr * CM_PER_EDGE_R;
    headingRad = (distR - distL) / TRACK_WIDTH_CM;
}

/* ========================= 9. 直线闭环核心 =========================
 *  每 CTRL_PERIOD_MS 执行一次：
 *    ① 测速(周期法) → 低通滤波
 *    ② 横向偏差(循迹) + 航向误差(积分差速)
 *    ③ 三环合成左右目标速度
 *    ④ 双轮 PI → PWM
 * ========================================================== */
static unsigned long lastCtrl = 0;

static void controlStep()
{
    unsigned long now = millis();
    unsigned long el  = now - lastCtrl;
    if(el < CTRL_PERIOD_MS) return;
    lastCtrl = now;
    double dt = (el > 200) ? 0.2 : el / 1000.0;   // 钳上限，防长阻塞后积分冲顶

    /* ① 测速 + 滤波 */
    double sL = readSpeed(&encDtL, &encTmL, CM_PER_EDGE_L, dirL);
    double sR = readSpeed(&encDtR, &encTmR, CM_PER_EDGE_R, dirR);
    spdL += SPEED_LP_A * (sL - spdL);
    spdR += SPEED_LP_A * (sR - spdR);

    if(!running) { stopAll(); return; }

    /* 出发段：纯直行 1.5s，闭环不介入（此时还没有路径参考） */
    if(fwdUntil && now < fwdUntil) {
        tgtL = tgtR = baseSpeed;
        double oL = wl.step(tgtL, spdL, dt);
        double oR = wr.step(tgtR, spdR, dt);
        pwmL = (int)oL; pwmR = (int)oR;
        motorLR(pwmL, pwmR);
        return;
    }

    /* ② 横向偏差（循迹环输入） */
    double rawErr = estimateLineErr(a, b, c);
    lineErrCm = rawErr;

    /* 掉线检测 */
    if(a == 0 && b == 0 && c == 0) {
        if(lostSince == 0) lostSince = now;
        if(now - lostSince > LOST_LINE_MS) { stopAll(); running = false; return; }
    } else {
        lostSince = 0;
    }

    /* 横向纠偏：连续命中 OFF_LINE_CONF 次才生效，避免探头抖动导致蛇形 */
    if(rawErr != 0.0) { if(offConf < 255) offConf++; }
    else              { offConf = 0; }

    /* ③ 差速合成
     *   lineCmd : 循迹横向纠偏（外环）
     *   headCmd : 航向保持（中环），直接用航向角误差
     *   trimBias: 静态机械补偿（电机/轮径不对称）
     *   三者叠加得到左右轮速度差 delta
     */
    double lineCmd = 0.0;
    if(offConf >= OFF_LINE_CONF)
        lineCmd = -LINE_ERR_GAIN * rawErr;      // 线偏左则左轮减速、右轮加速 → 车左转

    updateHeading();
    double headCmd = head.step(headingRad, dt, HEADING_CLAMP);

    double delta = lineCmd + headCmd + trimBias;
    if(delta >  HEADING_CLAMP) delta =  HEADING_CLAMP;
    if(delta < -HEADING_CLAMP) delta = -HEADING_CLAMP;

    /* 基速：跑偏时自动降速，留出转向余量 */
    double base = baseSpeed;
    base -= fabs(delta) * 0.8;
    if(base < 12.0) base = 12.0;

    tgtL = base - delta * 0.5;   // delta>0 → 左轮减速、右轮加速 → 右转
    tgtR = base + delta * 0.5;

    /* ④ 双轮 PI → PWM */
    double oL = wl.step(tgtL, spdL, dt);
    double oR = wr.step(tgtR, spdR, dt);

    // 启动死区补偿
    if(oL > 0 && oL < PWM_MIN) oL = PWM_MIN;
    if(oL < 0 && oL > -PWM_MIN) oL = -PWM_MIN;
    if(oR > 0 && oR < PWM_MIN) oR = PWM_MIN;
    if(oR < 0 && oR > -PWM_MIN) oR = -PWM_MIN;

    pwmL = (int)oL; pwmR = (int)oR;
    motorLR(pwmL, pwmR);
}

/* ========================= 10. 超声波 ========================= */
static volatile float distance = 999.0;
static uint8_t ultraState = 0;
static unsigned long lastTrig = 0;

static void updateUltrasonic()
{
    if(ultraState == 0) {
        if(micros() - lastTrig > ULTRA_PERIOD_US) {
            digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
            digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
            digitalWrite(PIN_TRIG, LOW);
            lastTrig = micros();
            ultraState = 1;
        }
    } else {
        uint32_t dur = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
        distance = (dur > 0) ? (dur / 58.0f) : 999.0f;
        ultraState = 0;
    }
}

/* 停车判定：距离落在 [STOP_NEAR, STOP_FAR] 带内即锁停
 * ⚠ 历史坑：早期用 (d>0 && d<17) 单边判据，车稍快就会从 >17 一步跨到 <13，
 *   掉出窗口反而放行，越近越危险。故此处保留近端下界作为"已抵近"兜底。 */
static void checkObstacle()
{
    static uint8_t clearCnt = 0;
    if(!ultraArmed) { stopLocked = false; return; }

    if(distance <= STOP_FAR_CM && distance >= 0.0f) {
        clearCnt   = 0;
        stopLocked = true;
        if(!waitingR) { waitingR = true; stage = ST_STOP_READ; }
    } else if(stopLocked) {
        if(++clearCnt >= 2) stopLocked = false;
    } else {
        clearCnt = 0;
    }
}

/* ========================= 11. 串口指令 =========================
 *  "1,x"   任务号 f=x，调 qu()，出发 1.5s
 *  "2,xx"  现场号 r=xx%10，与 f 比对
 *  R       使能直线闭环    S  停止并复位
 *  V30     基速   K4  速度Kp   J10 速度Ki
 *  H1.2   航向Kp  D0.35 航向Ki  M6  静态补偿(trim)
 * ========================================================== */
static void qu()  { }   // 取件机构（待实现）
static void fan() { }   // 放置（待实现）
static void gui() { }   // 归位（待实现）

static void handleTask(int code, int val)
{
    int d = val % 10;
    if(d < 1 || d > 3) { Serial.println(F("# bad code: need 1,x or 2,xx")); return; }

    if(code == 1) {
        f = d; taskReady = true; phase2 = false; ultraArmed = false;
        waitingR = false; stopLocked = false;
        qu();
        fwdUntil  = millis() + START_FWD_MS;
        lineStart = 0;
        stage     = ST_START_FWD;
        running   = false;
        wl.reset(); wr.reset(); head.reset();
        Serial.print(F("# task f=")); Serial.println(f);
        return;
    }

    if(code == 2) {
        if(!phase2) { Serial.println(F("# not armed yet: wait 3s line-follow")); return; }
        r = d;
        Serial.print(F("# read r=")); Serial.print(r);
        Serial.print(F("  f="));      Serial.println(f);

        if(r == f) {
            fan(); gui();
            waitingR = false; stopLocked = false;
            stage = ST_LINE;
            Serial.println(F("# MATCH -> placed, back to line"));
        } else {
            waitingR = false; stopLocked = false;
            stage = ST_LINE;
            Serial.println(F("# NOT match -> back to line"));
        }
        running = true;
        return;
    }
    Serial.println(F("# unknown task cmd"));
}

static void applyCmd(char *c)
{
    // 含逗号 → 任务指令
    for(char *p = c; *p; p++) {
        if(*p == ',') {
            *p = 0;
            handleTask(atoi(c), atoi(p + 1));
            return;
        }
    }

    if(c[0] == 'R')      { running = true;  lostSince = 0; offConf = 0; head.reset(); Serial.println(F("# running")); return; }
    if(c[0] == 'S')      { running = false; stopAll(); Serial.println(F("# stopped")); return; }
    if(c[0] == '0')      { head.reset(); wl.reset(); wr.reset(); trimBias = 0.0; Serial.println(F("# reset")); return; }

    if(c[0] == 'V') { baseSpeed = atof(c + 1); if(baseSpeed < 0 || baseSpeed > 60) baseSpeed = 30.0; Serial.print(F("# base ")); Serial.println(baseSpeed); return; }
    if(c[0] == 'K') { kpSpeed = atof(c + 1); wl.gains(kpSpeed, kiSpeed); wr.gains(kpSpeed, kiSpeed); Serial.print(F("# speedKp ")); Serial.println(kpSpeed); return; }
    if(c[0] == 'J') { kiSpeed = atof(c + 1); wl.gains(kpSpeed, kiSpeed); wr.gains(kpSpeed, kiSpeed); Serial.print(F("# speedKi ")); Serial.println(kiSpeed); return; }
    if(c[0] == 'H') { head.kp = atof(c + 1); Serial.print(F("# headKp ")); Serial.println(head.kp); return; }
    if(c[0] == 'D') { head.ki = atof(c + 1); Serial.print(F("# headKi ")); Serial.println(head.ki); return; }
    if(c[0] == 'M') { trimBias = atof(c + 1); Serial.print(F("# trim ")); Serial.println(trimBias); return; }

    Serial.println(F("# unknown cmd"));
}

static void handleSerial()
{
    static char buf[16];
    static uint8_t idx = 0;
    while(Serial.available() > 0) {
        char ch = (char)Serial.read();
        if(ch == '\n' || ch == '\r') {
            if(idx > 0) { buf[idx] = 0; applyCmd(buf); idx = 0; }
        } else if(idx < sizeof(buf) - 1) {
            buf[idx++] = ch;
        }
    }
}

/* ========================= 12. setup / loop ========================= */
void setup()
{
    Serial.begin(115200);

    pinMode(PIN_LINE_L, INPUT);
    pinMode(PIN_LINE_M, INPUT);
    pinMode(PIN_LINE_R, INPUT);

    pinMode(PIN_ML_FWD, OUTPUT); pinMode(PIN_ML_REV, OUTPUT);
    pinMode(PIN_MR_FWD, OUTPUT); pinMode(PIN_MR_REV, OUTPUT);

    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);

    pinMode(PIN_ENC_L, INPUT_PULLUP);
    pinMode(PIN_ENC_R, INPUT_PULLUP);
    delay(2);
    pcLast = PINC;
    PCMSK1 |= _BV(PCINT10) | _BV(PCINT11);   // 只放行 A2/A3，挡掉 A4/A5 循迹
    PCICR  |= _BV(PCIE1);

    wl.gains(kpSpeed, kiSpeed);
    wr.gains(kpSpeed, kiSpeed);
    head.gains(HEADING_KP, HEADING_KI);

    motorLR(0, 0);
    lastCtrl = millis();

    Serial.println(F("# ready. 1,x=task  2,xx=report  R=run  S=stop  0=reset"));
}

void loop()
{
    // ① 读循迹传感器
    a = digitalRead(PIN_LINE_L);
    b = digitalRead(PIN_LINE_M);
    c = digitalRead(PIN_LINE_R);

    // ② 串口指令（非阻塞）
    handleSerial();

    // ③ 超声波 + 障碍判定
    updateUltrasonic();
    checkObstacle();

    // ④ 闭环控制（20ms 定点）
    controlStep();

    // ⑤ 出发段 1.5s 走完 → 进入巡线并启动计时
    if(stage == ST_START_FWD && millis() >= fwdUntil) {
        fwdUntil = 0;
        stage    = ST_LINE;
        lineStart = millis();
        running  = true;
        Serial.println(F("# forward done -> straight closed-loop"));
    }

    // ⑥ 进入巡线 3s 后解锁超声波
    if(stage == ST_LINE && lineStart != 0 && !ultraArmed &&
       millis() - lineStart >= UNLOCK_DELAY_MS) {
        ultraArmed = true;
        phase2    = true;
        Serial.println(F("# 3s elapsed -> ultrasonic armed"));
    }

    // ⑦ 调试输出，100ms 一次
    //    a,b,c,lineErr,headDeg,spdL,spdR,pwmL,pwmR,dist,f,r,stage
    static unsigned long lastPrint = 0;
    if(millis() - lastPrint > 100) {
        lastPrint = millis();
        Serial.print(a);            Serial.print(',');
        Serial.print(b);            Serial.print(',');
        Serial.print(c);            Serial.print(',');
        Serial.print(lineErrCm, 1); Serial.print(',');
        Serial.print(headingRad * 57.2958, 1); Serial.print(',');
        Serial.print(spdL, 1);      Serial.print(',');
        Serial.print(spdR, 1);      Serial.print(',');
        Serial.print(pwmL);         Serial.print(',');
        Serial.print(pwmR);         Serial.print(',');
        Serial.print(distance, 1);  Serial.print(',');
        Serial.print(f);            Serial.print(',');
        Serial.print(r);            Serial.print(',');
        Serial.println(stage);
    }
}