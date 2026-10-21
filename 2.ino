/* ============================================================================
 *  循迹搬运小车 · 双轮编码器速度环 PID
 *  ---------------------------------------------------------------------------
 *  接线
 *    循迹传感器 : A4 / 中 D3 / A5（左右是否对调由 TRACK_MIRROR 决定）
 *    左轮驱动   : Motor-1 → 正转 D6    反转 D5
 *    右轮驱动   : Motor-0 → 正转 D10   反转 D9
 *    编码器     : 左 A3(PCINT11)   右 A2(PCINT10)   ← PCMSK1 / PCINT1_vect
 *    超声波     : TRIG D15(A1)   ECHO D14(A0)   ← 与 ECHOPIN/TRIGPIN 一致
 *    物块相机   : TXD(IO9) → D0   （硬件串口 RX，9600 8N1）    4_v4.py
 *    放置区相机 : TXD(IO9) → 物块相机 RXD(IO10)，菊花链，9600 8N1  111.py
 *                 由物块相机原样转发到 D0 —— D0 上始终只有一个驱动
 *    舵机 1~4   : D4 / D7 / D8 / D12（270° 舵机，Timer2 软件 PWM，单周期 20ms ≈ 50Hz）
 *                 接口只有 servoWrite(信号线, 角度)，角度 0~270，平滑推进
 *  ---------------------------------------------------------------------------
 *  结构（三层）
 *    ① 底层  motorLR()          唯一直接写电机引脚的地方
 *    ② 中层  updateSpeedPID()   每 20ms 测速 → 双轮 PID → PWM
 *    ③ 上层  du() / move_tai()  只设目标速度 tgtL / tgtR，不碰 PWM
 *  ---------------------------------------------------------------------------
 *  ★ 定时器与中断分配（改引脚前务必先看这张表）
 *    Timer0  micros/millis + 左电机 PWM(D5/D6)       —— 不能动
 *    Timer1  右电机 PWM(D9/D10)                      —— 不能动（Servo 库会抢占它！）
 *    Timer2  舵机 4 路软件 PWM（2us tick，COMPB 比较中断，约 2.5kHz）
 *    PCINT0  D11 软串口（仅 SITE_ON_HW_UART=0 的备用方案用，当前不使能）
 *    PCINT1  编码器 A2/A3 + 超声波 ECHO（A0 或 A1）   —— 共用同一个 ISR
 *    三个 ISR 的向量号分别是 3 / 4 / 8，互不冲突；COMPA(7) 留给 Tone.cpp。
 *  ---------------------------------------------------------------------------
 *  视觉协议（菊花链：两台相机都是 9600，放置帧由物块相机转发到 D0）
 *    ① 物块相机 4_v4.py   → D0（硬件串口）**9600**
 *         "1,1" 圆柱 / "1,2" 三棱柱 / "1,3" 正方体；无目标 "0,0"
 *         锁定后每 100ms 重发 → 物块种类 f
 *    ② 放置区相机 111.py  → 物块相机 IO10（菊花链）→ D0，**9600**
 *         '<' dd '>' CHK '\n'  共 7 字节，ASCII
 *         dd = 11 圆形 / 22 三角形 / 33 正方形 / 00 无目标
 *         CHK = ('<'+dd+'>') 字节累加和，两位十六进制
 *         例：b"<11>DC\n"  b"<22>DE\n"  b"<33>E0\n"  b"<00>DA\n"
 *         ★ 字节间留 220us 空隙（接收端每个字节要独占一个中断窗口）
 *         ★ 每 400ms 心跳重发一次，不是"变化才发"
 *         → 放置区域号 visSite；1.5s 无合法帧自动清零
 *  ---------------------------------------------------------------------------
 *  任务流程
 *    等 物块相机识别 → qu() → 直行 1.5s → 巡线
 *      → 超声波测距 ≤22cm（连续 3 个样本）→ 停车 → 读放置区相机区域号
 *        → 区域号 == f → fan()+gui() 投放 → 前进离开 → 回空闲
 *        → 区域号 != f → 前进走出当前盒 → 继续找下一个（不回空闲）
 *  ---------------------------------------------------------------------------
 *  优先级（move_tai 内，从上到下）
 *    阶段闸门(含 ST_CORNER) > 过近急停 > zhi 对正 > L/R 原地拐 > 丢线盲走
 *    ★ 拐弯保命规则：
 *      1) 退出对正必须"连续"稳回线上（BACK_ONLINE_MS），单次 q==10 不算
 *      2) 三个探头同时丢线（q==0）不停死，按最后看到的方向盲拐，最多 LOST_MS
 *      3) 限幅/去抖都不够时，先调 BACK_ONLINE_MS（60~150）试
 *  ---------------------------------------------------------------------------
 *  直角弯（ST_CORNER：5cm 急停解除后自动触发）
 *    15cm 半径圆弧转 90°
 *      → 弧中若"先丢线、再见线"：巡线 1s → 后退到丢线停住
 *      → kai() 四步机构动作 → 回空闲
 *    全程 quiet=true：忽略 1,x，不输出任何串口
 *  ---------------------------------------------------------------------------
 *  机构动作（qu / fan / gui / kai）
 *    表驱动的非阻塞序列，前一路舵机到位才下发下一条；
 *    ★ 四个入口统一走 4 格环形队列：当前动作没走完就排队，绝不打断。
 *      qu(); fan(); gui(); 连着调用会依次执行，不会被顶掉。
 * ========================================================================== */

#include <string.h>
#include <util/delay_basic.h>   // _delay_loop_2：仅 SITE_ON_HW_UART=0 的软串口备用方案需要

/* ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
 *  BENCH_LINE_ONLY —— 基础巡线隔离测试模式
 *   1 = 只跑「读三探头 → 按 q 给动作 → 速度闭环」，其余全不跑：
 *         不看物块相机、不做超声波判停、不跑任务状态机/直角弯/舵机动作。
 *         上电立刻开始巡线；板载 LED = 任意探头见到黑线就亮。
 *   0 = 正常工作模式（比赛用）。
 *  ⚠ 排查完必须改回 0，否则永远不会执行任务流程。
 * ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★ */
#define BENCH_LINE_ONLY   0

/* ---------------- 硬件引脚 ---------------- */
/* ★★★ 超声波：pulseIn 阻塞式测距 ★★★
 *   TRIG = D15(A1)，ECHO = D14(A0)。与示例代码完全同接法。
 *   pulseIn 会阻塞到回波结束（无回波时阻塞到超时），但对 24cm 判停来说
 *   阻塞时长 = 回波宽度 = 距离×58us ≈ 1.4ms，加上超时上限 25ms 完全可接受，
 *   换来的是不再依赖 PCINT 边沿捕获，逻辑简单、不受中断时序干扰。 */
#define TRIGPIN 15
#define ECHOPIN 14

/* ★★★ 循迹探头左右定义（决定 100 / 1 / 110 三个分支往哪边修正）★★★
 *   0 = A4 是左探头、A5 是右探头        ← 当前（已确认 A4 装在左侧）
 *   1 = A4 装在右侧、A5 在左侧（接线与丝印相反）
 *
 *  判据：q==110（A4 + 中探头同时压线）时，车应该往哪边修正？
 *    · 应该往【左】修 → 线偏在车的左侧 → 填 0（A4 在左）
 *    · 应该往【右】修 → 线偏在车的右侧 → 填 1（A4 在右）
 *  改这一处，q=100 / q=1 / q=110 / 丢线盲走 会一起换边，方向始终自洽。
 *  ⚠ 不要只单独翻 zhi() 的符号，那样 100 和 110 会互相矛盾。 */
#define TRACK_MIRROR   0

/* ---------------- 机械参数（按你的实车核对！） ---------------- */
#define ENC_EDGES_PER_REV 30       // 码盘 15 孔 → 转一圈 30 个边沿（上升沿+下降沿）
/* ★ 上面这个是本文件里唯一还没实测确认的机械常量。
 *   标定法：让车按 baseSpeed=30 走完出发段（START_FWD_MS=1.5s，理论应走 45cm），
 *          拿尺量实际位移 D，则   新值 = 30 × 45 / D
 *          （走 36cm → 37.5；走 45cm → 30，说明原来就是对的）
 *   它只影响"绝对速度"的标定：两轮共用同一个常量，所以**不会造成左右跑偏**。
 *   但直角弯是按时长开环的（弧长 23.6cm ÷ 基速），速度标错 → 弧长算错
 *   → 转不到 90°。转弯角度不对时，先查这个，再查 CORNER_TRACK_CM。 */
#define WHEEL_DIA_CM      5.0      // 轮子直径 5cm（半径 2.5cm，已实测确认）
// 一个脉冲对应车走了多远：π*D / 一圈脉冲数 = 3.1416*5/30 = 0.5236 cm
#define CM_PER_EDGE (3.14159265 * WHEEL_DIA_CM / (double)ENC_EDGES_PER_REV)

/* ---------------- 速度环参数 ---------------- */
#define CTRL_PERIOD_MS 20       // 速度环周期 20ms（50Hz），够快也不浪费算力
#define SPEED_LP       0.50     // 测速一阶低通，越大跟得越快、越抖（0.3~0.7 之间调）
#define STALL_US       150000UL // 超过 150ms 没有新脉冲 → 认为这个轮子停了
#define PWM_MAX        255
/* ★★★ 电机启动死区（实测：PWM < 50 轮子基本不动）★★★
 *  死区不补偿会出三个问题，而且都是"看起来像 PID 调不好"：
 *   ① 低速时 PID 输出 20~40，轮子不转 → 误差不减小 → 积分一直涨 →
 *      冲破死区后车猛地窜一下，然后超速、再回落，走走停停。
 *   ② 两个轮子的死区不一定相同（左 45、右 55 很常见）→ 同样的 PWM 一转动一不转
 *      → 车直接往一侧偏，而且偏得没规律。
 *   ③ 目标速度低于"死区对应的速度"时（约 50/4.5 ≈ 11 cm/s），根本到不了。
 *  这里填实测死区：任何非零输出都被抬到 ±PWM_MIN，保证"只要该转就一定转起来"。
 *  代价：最低可控速度 ≈ 11 cm/s，所以 baseSpeed 不要低于 15。 */
#define PWM_MIN        50

/* ---- 电池电压跌落的兜底（速度环已经消除压降，这里只管"补偿不动了"的情况）----
 *  电压降 → 同样 PWM 出力变小 → 双轮速度环把 PWM 顶上去补偿，这一步是自动的，
 *  所以压降本身不会让车跑偏。真正会偏的是：某一轮 PWM 已经顶到 255 还追不上
 *  目标速度 —— 那一轮开始掉速，车就往另一边走。
 *  这里检测"连续顶满"，自动把基速按 2% 往下调，把 PWM 拉回有裕量的区间。
 *  只降不升（避免在临界点来回抖），改回原速需重新上电或重设基速。 */
#define SAT_ADAPT       1       // 1 = 启用自动降速；0 = 关闭（基速永远不动）
#define SAT_HOLD_CNT   50        // 连续顶满 50 个周期（50×20ms = 1s）才动作
#define SAT_MIN_SPEED  16.0      // 基速最低降到这么多（必须高于死区对应速度 ~11cm/s）
#define SAT_STEP       0.98      // 每次降 2%

/* ---------------- PID 出厂参数（先按这个跑，再照现象调） ---------------- */
double kp = 4.0;                // 比例：拉速度的力度
double ki = 10.0;               // 积分：消稳态误差（车到不了目标速度就加它）
double kd = 0.0;                // 微分：编码器分辨率粗，D 一般别开，最多 0.05 级别

/* ---------------- 运行参数（改这里重新编译） ---------------- */
double baseSpeed = 30.0;        // 直行基础速度 cm/s
double turnSpeed = 26.0;        // 原地对转速度 cm/s（仅 TURN_ARC=0 时使用）

/* ★★★ 转向方式 ★★★
 *  0 = 原地对转：左轮反转 + 右轮正转，前进位移 0。  ← 当前
 *      当前参数(turnSpeed=26, 轮距 12cm)下角速度 = 2x26/12 = 4.33rad/s = 248°/s，
 *      90° 只要 0.36s，且车身不前冲 —— 纠大偏差必须快，就用它。
 *      代价：依赖两个轮子都能反转，任一路反转没出力就会只往另一边走。
 *  1 = 差速弧线：两轮都正转，一快一慢。不依赖反转、机械冲击小，
 *      但角速度只有 153°/s，而且转 90° 要同时前冲 18cm → 急弯/直角必然错过。
 *      只有当某一路反转确实坏了，才退回这个折中方案。 */
#define TURN_ARC        0
#define TURN_DELTA     16.0     // 内外轮【各自】相对 baseSpeed 的偏移 cm/s
                                // ★ 实际轮速差 = 2×16 = 32 cm/s，不是 16
                                //   → 角速度 = 32/12(轮距) = 2.67 rad/s ≈ 153°/s
                                //   （注释以前写成"速度差 16"，数字对但单位错）
double zhiDelta  = 18.0;        // zhi() 对正时左右轮的速度差 cm/s（12 太温柔，会拖成大偏差）

/* ========================= 0.4 四路舵机（Timer2 软件 PWM） =========================
 *  为什么不能用 Servo 库：
 *    Servo 库在 AVR 上独占 Timer1，而右电机 D9/D10 正是 Timer1 的 PWM 通道。
 *    一旦 include <Servo.h>，analogWrite(9/10) 立刻失效 → 右轮直接不转。
 *    左电机占 Timer0、右电机占 Timer1，唯一空着的是 Timer2。
 *
 *  ★ 用比较中断 TIMER2_COMPB（vector 8），不用溢出也不要 COMPA：
 *    · COMPA（vector 7）已被 Arduino 核心的 Tone.cpp 占用，重复定义会链接报错；
 *    · 溢出中断是"每 16us 无条件进一次"，62.5kHz 下光保存/恢复寄存器就吃掉
 *      37% CPU（实测 ISR 182 条指令、24 个 push）；
 *    · COMPB 只在真有事件时中断，一个 20ms 周期约 50 次（2.5kHz），
 *      占用降到约 3.2%（= 49 次 × 13us / 20ms），计数器分辨率从 16us 提到 2us。
 *
 *  做法：Timer2 自由运行（预分频 /32，2us/tick，8 位 0~255 循环），
 *  每次中断把"下一个事件"排进 OCR2B：
 *    on0 → off0 → on1 → off1 → on2 → off2 → on3 → off3 → 周期结束
 *    20ms / 2us = 10000 tick，四路错开 3ms 依次拉高，避免同时启动的电流冲击
 *
 *  ★ 270° 舵机：对外只有一个接口 —— servoWrite(信号线, 角度)
 *    · 0~270° 线性对应 500~2500us（见 SERVO_US_0 / SERVO_US_FULL；端点可标定）
 *      换 180° 舵机只需把 SERVO_DEG_FULL 与 SERVO_DEG_TENTH 一起改
 *    · 角度不是"一步到位"：每个 20ms 周期最多推进 SERVO_SLEW_DPS 度，
 *      在 ISR 的【周期末尾】逐步逼近目标 —— 那时四路脉冲都已结束，
 *      既不会打断正在输出的脉冲，也不会拉长脉宽
 *    · 全程持续输出：每个周期都重排事件表，到位后继续按目标脉宽锁位，
 *      不存在"到了就停发脉冲"的卸力（失力）状态
 *
 *  ✔ 脉宽抖动源已消除：放置区相机现在走 **D0 硬件串口**（SITE_ON_HW_UART=1），
 *    USART 由移位寄存器收字节，CPU 完全不参与，不再有关中断窗口。
 *    （历史：早前它挂在 D11 软件串口上，每字符要关中断 78us/148us，
 *      落在脉冲高电平窗口内会把脉宽拉长 10°~20°。该方案已废弃，
 *      仅在 SITE_ON_HW_UART=0 时作为备用路径保留。）
 *  ⚠ 机构动作（qu/fan/gui）里只调 servoWrite()，不要写 delay()：
 *    脉宽由中断持续输出，delay 只会拖慢控制环。
 * ================================================================================ */
#define SERVO_N        4

/* ★ 时间基准：预分频 /32 → 16MHz/32 = 500kHz → 2us/tick
 *   20ms = 10000 tick。分辨率 2us，270° 行程共 1000 档 = 0.27°/档。
 *   （旧的 16us 溢出中断方案只有 62 档，且 62.5kHz 中断吃掉 37% CPU。） */
#define SERVO_CYCLE    10000    // 10000 tick × 2us = 20.0ms（即 50Hz）
#define SERVO_STAGGER  1500     // 1500 * 2us = 3ms，四路错开
#define SERVO_SEG_MAX  240      // 单次最多排 240 tick（480us）：比较是模 256 的，
                                // 取 255 也不会错；留 240 是为了单格中断占比只有 2.7%，
                                // 给排查和后续加通道留余量
/* ★ 第 0 路的起点不能是 0 tick：
 *   周期收尾后 svAbsCur=0，排下一个比较点时距离为 0，必须抬到最小 4 tick
 *   （否则写入 OCR2B 的瞬间计数器已经越过，要白等一整圈 512us）。
 *   若 on0 仍写 0，它就会被记在"逻辑时刻 4"，实测脉宽恒短 8us。
 *   统一让四路起点都从 4 tick 开始，偏差就归零了。 */
#define SERVO_ON0      4

/* ---- 270° 舵机标定：换 180° 舵机时只改 SERVO_DEG_TENTH 一处 ----
 * ★ 单一真源：以 0.1° 定点 SERVO_DEG_TENTH 为准，度数由它推出来。
 *   以前两个各写一遍（270.0f / 2700L），换舵机时只改一个就会出现
 *   "角度与脉宽脱节"，而且 #if 里没法做浮点比较、写不出编译期断言。 */
#define SERVO_DEG_TENTH   2700L                             // 0.1° 定点满量程（真源）
#define SERVO_DEG_FULL    ((float)SERVO_DEG_TENTH / 10.0f)  // 机械行程（度）= 270.0
/* ★★ 舵机脉宽端点 ★★
 *  0~270° 线性映射到 500~2500us，这是 270° 舵机的通用规格，先按标称值用。
 *
 *  ⚠ 历史提醒：早先有一次"90° 多了 9°"的实测，后来查明当时 Timer2 预分频
 *    写错（/256 而不是 /32），脉宽整体放大 8 倍，那次观测不可信，不能拿来
 *    反推端点。要精调必须在**定时器确认正确**之后做两点标定：
 *      命令 0°（500us）记下实际角 A0，命令 270°（2500us）记下 A270，
 *      k = 2000/(A270-A0)，SERVO_US_0 = 500 - A0*k，SERVO_US_FULL = 2500 + (270-A270)*k
 *
 *  若换 180° 舵机：把 SERVO_DEG_FULL 与 SERVO_DEG_TENTH 一起改。
 *  若舵机是 1000~2000us 规格：只改下面这两行，换算会自动跟随。 */
#define SERVO_US_0        500       //   0° 对应脉宽
#define SERVO_US_FULL     2500      // 270° 对应脉宽

/* ★★ 输出脉宽下限：保证任何一路都不会停在行程端点"歇力" ★★
 *  0° 对应 500us，正好压在舵机行程尽头 —— 端点附近保持力矩骤降，舵机表现为
 *  "推得动、使不上劲"。D12 在 qu() 里要走到 0°，D8 上电初始位虽然设成 270°（那也是
 *  上端点、同样会失力），所以这个底端夹紧是给 D12/qu() 用的。
 *  两处都会踩到端点，所以这里把脉宽底端抬到 550us。
 *
 *  ★ 只夹底端，不动量程：
 *      0°  → 550us（实际约 6.75°，这一段成为死区）
 *      8°  → 560us   ← 不受影响
 *      90° → 1166us  ← 完全不变
 *     180° → 1834us  ← 完全不变
 *     270° → 2500us  ← 完全不变
 *    所以不会像"改 SERVO_US_0"那样把所有角度整体平移。
 *
 *  机构若必须压到底才能夹紧，把这行改小（但不要低于 SERVO_US_0）。 */
#define SERVO_US_MIN_OUT  550
/* ---- 上电初始角（度），顺序与 servoPin 一致 ----
 *   D4 → 90°   D7 → 90°   D8 → 270°  D12 → 180°
 * 想改站位只改这一行，不用动其它逻辑。 */
#define SERVO_HOME_D4      90
#define SERVO_HOME_D7      90
#define SERVO_HOME_D8       270
#define SERVO_HOME_D12    180

/* 角度（0.1°）→ Timer tick 的换算，直接由上面两个端点推出来，端点一改就自动跟随：
 *     tick = TK0 + (tenth * SPAN + 1350) / 2700
 *   TK0  = US_0/2            （500us ÷ 2us = 250 tick）
 *   SPAN = (US_FULL-US_0)/2  （2000us ÷ 2us = 1000 tick）
 *   tenth 最大 2700，2700*1000 = 2700000 需要 32 位 —— 这没关系：
 *   换算只在 noinline 的 servoTrack() 里跑（周期末尾、无脉冲输出时），
 *   不进 ISR 序言，也就不会把 r12~r31 的压栈带进 2.5kHz 的中断里。 */
#define SERVO_TK0         (uint16_t)(SERVO_US_0 / 2)
#define SERVO_TK_SPAN     (uint16_t)((SERVO_US_FULL - SERVO_US_0) / 2)
#define SERVO_TK_MIN_OUT  (uint16_t)(SERVO_US_MIN_OUT / 2)

/* ================= 舵机时序的编译期自检 =================
 * 这三个约束一旦被破坏，故障现象都很难反查（脉宽被截断、周期被吃掉、
 * 角度与脉宽脱节），所以让它们在编译期就炸掉，而不是等上电才表现异常。 */

/* ① 单路脉宽必须小于通道错开间隔：否则 off_i 落在 on_{i+1} 之后，事件表乱序 */
#if ((SERVO_US_FULL - SERVO_US_0) / 2) >= SERVO_STAGGER
  #error "舵机脉宽超过 SERVO_STAGGER：相邻通道事件会交错，后一路脉宽被截断"
#endif

/* ② 最后一路的下降沿必须在周期内，否则末尾事件被吃掉 */
#if (SERVO_ON0 + (SERVO_N - 1) * SERVO_STAGGER + (SERVO_US_FULL / 2)) >= SERVO_CYCLE
  #error "最后一路脉宽越过 SERVO_CYCLE：周期不够，末尾事件会被吃掉"
#endif

/* ③ 角度满量程与 0.1° 定点必须同步 —— 已由"单一真源"保证：
 *   SERVO_DEG_FULL 直接由 SERVO_DEG_TENTH 推导，改一处即可，不会再脱节。 */

/* ---- 平滑推进：每周期（20ms）最多转 SERVO_STEP 个 0.1° 单位 ----
 * 240°/s → 每周期 4.8° → 0~270° 约 1.13s
 * 想更快改 600（≈0.45s），想更柔和改 120（≈2.25s）。
 * 出现抖动/异响/电流冲击时优先降它，不要去改 PID。 */
#define SERVO_SLEW_DPS    240
/* ★ 不再硬编码 200：那个 200 = 周期 20ms × 10（0.1°/度）。
 *   改成由 SERVO_CYCLE 推出来，以后改周期时步进速率会自动跟随，
 *   否则 SERVO_CYCLE 一改，"多少秒转完 270°"就静默变了。 */
#define SERVO_CYCLE_MS    ((long)SERVO_CYCLE / 500L)          /* 10000/500 = 20ms */
#define SERVO_STEP_RAW    ((long)SERVO_SLEW_DPS * SERVO_CYCLE_MS * 10L / 1000L)
/* 不能为 0：SERVO_SLEW_DPS<5 时 STEP 会变 0，servoTrack 永远 continue，
 * servoReached 永远为假 -> actRun/actDone 挂死到 40s 任务超时。 */
#define SERVO_STEP        ((int16_t)(SERVO_STEP_RAW < 1 ? 1 : SERVO_STEP_RAW))

const uint8_t servoPin[SERVO_N] = {4, 7, 8, 12};

/* 上电初始角（度），顺序与 servoPin 一一对应
 * ★ 必须是 uint16_t：270° 装不进 uint8_t（narrowing conversion 会让整个
 *   sketch 编译失败，板子里跑的就还是上一次的旧固件）。 */
const uint16_t servoHomeDeg[SERVO_N] =
{
    SERVO_HOME_D4, SERVO_HOME_D7, SERVO_HOME_D8, SERVO_HOME_D12
};


static volatile int16_t svTgtTenth[SERVO_N];   // 目标角（0.1°），主循环写
static volatile int16_t svCurTenth[SERVO_N];   // 当前角（0.1°），ISR 写
static volatile uint16_t svTicks[SERVO_N];     // 当前脉宽（2us/tick），ISR 内换算

static uint16_t svEvtAt[2 * SERVO_N + 1];      // on0,off0,on1,off1,...,周期结束
static uint16_t svAbsCur = 0;                  // 当前逻辑时刻（tick，0..SERVO_CYCLE）
static uint8_t  svEvt    = 0;                  // 下一个待执行的事件序号

/* 角度（0.1°）→ 脉宽（Timer tick），端点改了自动适配 */
static uint16_t servoAngleToTicks(int16_t tenth)
{
    if(tenth < 0)                tenth = 0;
    if(tenth > SERVO_DEG_TENTH)  tenth = (int16_t)SERVO_DEG_TENTH;

    uint16_t tk = (uint16_t)( SERVO_TK0 +
                   ((uint32_t)tenth * SERVO_TK_SPAN + (uint32_t)SERVO_DEG_TENTH / 2)
                       / (uint32_t)SERVO_DEG_TENTH );

    /* 极限位力矩兜底：SERVO_US_MIN_OUT = 550us（比 0° 的 500us 高 50us ≈ 6.75°），
     * 保证任何一路都不会停在行程端点失力。 */
    if(tk < SERVO_TK_MIN_OUT) tk = SERVO_TK_MIN_OUT;

    return tk;
}

/* 信号线 → 通道号；不是注册过的舵机口返回 -1 */
static int8_t servoChOf(uint8_t pin)
{
    for(uint8_t i = 0; i < SERVO_N; i++) if(servoPin[i] == pin) return (int8_t)i;
    return -1;
}

/* ★★★ 对外唯一接口：按【信号线 + 角度】控制 270° 舵机
 *   pin : 信号线接的 Arduino 引脚（D4 / D7 / D8 / D12）
 *   deg : 目标角度 0~270，超范围自动钳位（不会顶到机械限位）
 *   返回: true = 引脚有效；false = 不是舵机口，什么都没做
 *   特性: 非阻塞，立刻返回。转动由 Timer2 中断按 SERVO_SLEW_DPS 的速率
 *         平滑推进；中途随时改目标会当前姿态接上，不会跳变、不会回弹。
 */
bool servoWrite(uint8_t pin, float deg)
{
    int8_t ch = servoChOf(pin);
    if(ch < 0) return false;

    if(deg < 0.0f)           deg = 0.0f;
    if(deg > SERVO_DEG_FULL) deg = SERVO_DEG_FULL;

    int16_t t = (int16_t)(deg * 10.0f + 0.5f);   // 0.1° 定点，四舍五入

    /* ★ 用 SREG 保存/恢复而不是 interrupts()：
     *   后者会无条件开中断，若本函数从某个关中断的上下文（ISR、别的临界区）
     *   被调用，会把上层的中断屏蔽状态冲掉。 */
    uint8_t sreg = SREG;
    cli();
    svTgtTenth[ch] = t;                          // 16 位写入，防 ISR 读到半新半旧
    SREG = sreg;

    return true;
}

/* 该路是否已到达目标角 —— 编排多步机构动作时用它代替 delay() */
bool servoReached(uint8_t pin)
{
    int8_t ch = servoChOf(pin);
    if(ch < 0) return false;

    uint8_t sreg = SREG;
    cli();
    int16_t tgt = svTgtTenth[ch];
    int16_t cur = svCurTenth[ch];
    SREG = sreg;

    return (tgt == cur);
}

/* 周期末尾推进一次角度。
 * 放在这里的原因：此刻四路脉冲都已结束，到下一周期第一个上升沿还有约
 * 9.5ms 空档，在这里做几次整数除法不会拉长任何一路的脉宽。
 *
 * ★ 必须 noinline：一旦被内联进 ISR，编译器会把 25 位宽的乘除序列所需的
 *   r12~r31 全部压栈，ISR 序言从 8 个 push 变 24 个，62.5kHz 下白烧 20% CPU。
 * ★ 只有角度真的变了才重算 tick：舵机静止时这里 4 次比较就返回，
 *   主循环拿到的 CPU 完全不受影响。（servoTrack 内不会改 svTgtTenth，
 *   所以 TCNT2 在周期末的时间基准不会被扰动。） */
static void __attribute__((noinline)) servoTrack()
{
    for(uint8_t i = 0; i < SERVO_N; i++)
    {
        int16_t cur = svCurTenth[i];
        int16_t d   = (int16_t)(svTgtTenth[i] - cur);

        if(d >  SERVO_STEP) d =  SERVO_STEP;
        if(d < -SERVO_STEP) d = -SERVO_STEP;

        if(d == 0) continue;                    // 已到位 → 零开销

        cur = (int16_t)(cur + d);
        svCurTenth[i] = cur;
        svTicks[i]    = servoAngleToTicks(cur);
    }
}

static void servoPlan()
{
    uint8_t k = 0;
    for(uint8_t i = 0; i < SERVO_N; i++)
    {
        uint16_t on  = (uint16_t)(SERVO_ON0 + (uint16_t)i * SERVO_STAGGER);
        uint16_t len = svTicks[i];
        /* ★ 真正的脉宽下限在 servoAngleToTicks() 里（钳到 SERVO_US_MIN_OUT=550us
         *   → 275 tick），这里再按 SERVO_TK0(250) 兜一次是够不着的死判断；
         *   保留它只为防"svTicks 未初始化"这一种情况，所以按真正的下限兜。 */
        if(len == 0) len = SERVO_TK_MIN_OUT;
        svEvtAt[k++] = on;
        svEvtAt[k++] = on + len;
    }
    svEvtAt[k] = SERVO_CYCLE;                   // 周期结束（尾哨兵，见 ISR 注释）
}

static void servoInit()
{
    for(uint8_t i = 0; i < SERVO_N; i++)
    {
        pinMode(servoPin[i], OUTPUT);
        digitalWrite(servoPin[i], LOW);

        /* 上电即预设初始位：当前角直接等于目标角，第一个周期就输出该角度，
         * 不做任何过渡。若让当前角从 0 起步，舵机会先往 0° 甩一下再走回来 ——
         * 那正是"上电抖一下 / 打到机构"的来源。
         * 注意乘 10 前必须先转 int16：90*10=900 会溢出 uint8。 */
        int16_t home  = (int16_t)((int16_t)servoHomeDeg[i] * 10);   // 0.1°
        svTgtTenth[i] = home;
        svCurTenth[i] = home;
        svTicks[i]    = servoAngleToTicks(home);
    }
    servoPlan();

    svAbsCur = 0;
    svEvt    = 0;

    uint8_t svSreg = SREG;                 // 保存/恢复而不是 interrupts()：
    cli();                                 // 后者会无条件开中断，破坏上层屏蔽状态
    TCCR2A = 0;                            // Normal 模式（关掉核心初始化的相位修正 PWM）
    /* ★★★ 预分频编码（ATmega328P datasheet Table 18-9）：
     *   CS22 CS21 CS20 = 0 1 1 → /32  → 16MHz/32 = 500kHz → 2us/tick
     *   写成 _BV(CS22)|_BV(CS21) = 0b110 是 /256（16us/tick），
     *   整个时间轴会被放大 8 倍：周期 160ms、90° 脉宽 9.33ms，
     *   远超舵机 500~2500us 的有效范围 → 舵机判为非法信号 → 完全无力。 */
    TCCR2B = _BV(CS21) | _BV(CS20);        // /32 → 2us/tick
    TCNT2  = 0;
    OCR2B  = (uint8_t)(TCNT2 + SERVO_ON0);  // 第一个比较点：与四路起点同一个常量
    /* ★ 用 COMPB（vector 8）而不是 OVF（vector 9）：
     *   溢出中断是"每 16us 无条件进一次"，62.5kHz 光保存/恢复寄存器就吃掉
     *   37% CPU；COMPB 只在真有事件时中断，约 2.5kHz，单次约 13us → 占用约 3.3%。
     *   而 COMPA（vector 7）仍留给 Tone.cpp，不能碰。 */
    TIMSK2 = _BV(OCIE2B);
    SREG = svSreg;
}

/* ========================= 0.4.1 舵机中断 =========================
 *  比较中断（OCR2B）驱动：每次进入时把"下一个事件"排到 8 位计数器能覆盖的
 *  范围内，跑完一个 20ms 周期约 50 次中断（旧方案 1250 次）。
 *
 *  为什么必须分段（SERVO_SEG_MAX）：
 *    TCNT2 只有 8 位，OCR2B 也只能到 255 tick。而"第 4 路关断"到"周期结束"
 *    之间常有 4000+ tick，所以分多次推进，每次 240 tick（480us）。
 *    中断实际约 13us（含一次 digitalWrite 的查表开销），占一格的 2.7%，
 *    绝不会因为处理时间长而错过下一个比较点。
 * ================================================================ */
ISR(TIMER2_COMPB_vect)
{
    /* ① 执行所有已到点的事件（正常只会命中一个） */
    while(svEvt < 2 * SERVO_N && svEvtAt[svEvt] <= svAbsCur)
    {
        uint8_t ch = (uint8_t)(svEvt >> 1);
        digitalWrite(servoPin[ch], (svEvt & 1) ? LOW : HIGH);   // 偶数=上升沿，奇数=下降沿
        svEvt++;
    }

    /* ② 一个 20ms 周期走完 → 四路全拉低，推进角度，重排事件表 */
    if(svEvt >= 2 * SERVO_N && svAbsCur >= SERVO_CYCLE)
    {
        for(uint8_t i = 0; i < SERVO_N; i++) digitalWrite(servoPin[i], LOW);

        servoTrack();                           // 此刻无脉冲输出，算多久都不影响脉宽

        svAbsCur = 0;
        svEvt    = 0;
        servoPlan();
    }

    /* ③ 排下一个比较点 */
    uint16_t target = (svEvt < 2 * SERVO_N) ? svEvtAt[svEvt] : SERVO_CYCLE;

    /* 距离为 0 时必须抬到最小段：写入 OCR2B 的瞬间计数器还在走，
     * 若排到 1~2 tick 处可能已经被越过，要等一整圈（512us）才匹配。
     * 最小段直接取 SERVO_ON0（4 tick = 8us），与上面"四路起点都从 4 开始"
     * 是同一个理由，不再写两处魔数。 */
    uint16_t d = (target > svAbsCur) ? (uint16_t)(target - svAbsCur) : SERVO_ON0;
    if(d > SERVO_SEG_MAX) d = SERVO_SEG_MAX;
    if(d < SERVO_ON0)     d = SERVO_ON0;

    OCR2B   = (uint8_t)(TCNT2 + d);
    svAbsCur += d;
}

/* ========================= 0.5 双 K230 视觉接入 =========================
 *  两台相机【分工不同、脚本不同、波特率也不同】，共用同一根 RX：
 *
 *  ① 物块相机 —— `4_识别并串口上报v4.py`  →  Arduino **D0（硬件串口）**
 *       UART1，IO9(TXD) →[1kΩ]→ D0，**9600** 8N1
 *       "1,1" 圆柱 / "1,2" 三棱柱 / "1,3" 正方体；无目标 "0,0"
 *       锁定后每 100ms 重发 → 物块种类 1/2/3 → visTask
 *
 *  ② 放置区相机 —— `111.py`      →  Arduino **D0（同一硬件串口）**
 *       UART2，IO11(TXD) →[1kΩ]→ D0，**115200** 8N1
 *       ★ 两路 TX 各串 1kΩ 再并联：都是推挽输出，直并会互相对打
 *       帧：'<' dd '>' CHK '\n'（7 字节）
 *       dd = 11 圆形 / 22 三角形 / 33 正方形 / 00 无目标 → 区域号 1/2/3
 *       CHK = ('<'+dd+'>') 累加和，两位十六进制大写
 *       每 400ms 心跳重发一次；字节之间强制留 220us 空隙
 *       波特率按阶段切换：ST_IDLE 听 9600，任务中听 115200
 *
 *  ★ 波特率为什么可以是 115200：
 *    因为两台相机现在都挂在 **D0 硬件 USART** 上（SITE_ON_HW_UART=1）。
 *    USART 由移位寄存器收字节，CPU 完全不参与，115200 只意味着"一帧更短"，
 *    没有任何时序风险，也不需要重新标定延时宏。
 *    （历史：早前放置相机在 D11 上走**软件串口**，那时 115200 会失败 ——
 *      一位只有 138 周期，字节间 1.5 位的空隙塞不下 Timer2 舵机中断的 205 周期，
 *      每帧必坏 1~2 字节。所以当年被压到 57600。那段论证只对软串口成立，
 *      SITE_ON_HW_UART=0 的降级路径才需要它，当前配置下不适用。）
 *
 * ★ 两路怎么接：两路 TX **各串一个 1kΩ 电阻** 后并联到 D0。
 *   两个 UART TX 都是推挽输出，直接并在一条线上会互相对打（一个拉低、一个顶高）。
 *   ★ 引入下面的命令协议之后，"任何时刻只有一台在发"变成了系统的**逻辑**保证，
 *     所以这 1kΩ 已经从"功能必需"降级为"故障保险"：万一有一台还在跑旧脚本、
 *     或者复位后异常输出，没有电阻就是硬碰硬（对打电流可达 20~40mA，超过 IO
 *     持续能力，可能打坏引脚或闩锁）；有电阻则只有 1.65mA，低电平仍能被拉到底，
 *     逻辑照样正确。成本两分钱，建议保留。
 * ★ 只接 TXD：不要把 Arduino D1(TX) 接到任何相机的 RXD ——
 *    下载程序时 D1 上的数据会干扰视觉端。
 *    （但现在 D1 被用作命令线，见下面的 CMD_* 定义，需要 5V→3.3V 分压）
 * ======================================================================== */
#define VISION_BAUD      9600       // 硬件串口 9600 ← 物块相机 4_v4.py
/* ★★★ 菊花链方案（当前）：放置区相机不再直接接 D0 ★★★
 *   111.py(IO11 TX,115200) → 4_v4.py(IO11 RX,UART2) → 4_v4.py(IO9 TX,9600) → D0
 *   物料相机把放置帧【原样透传】，所以到了 D0 上已经是 9600。
 *   ⇒ D0 上永远只有物料相机一个驱动，两台不可能抢线；
 *   ⇒ 不需要切波特率，也就没有 Serial.end()/begin() 带来的丢帧。
 *   两个值必须相等；不等时 visionBaud() 会退回"按阶段切换"的旧方案。 */
#define SITE_BAUD        9600

/* 命令线（D1→两台 K230 的 RX）在菊花链方案里用不到了：
 *   1 = 保留（共用 D0 的旧方案，需要 D1 分压接两路 RX）
 *   0 = 关闭（当前）—— 不再往 D1 发任何字节 */
#define K230_CMD_ENABLE  0

/* ================= 向 K230 下发命令（D1/TX 单向广播）=================
 *  D1 之前一直闲置（只接板载 FT232RL 的 RXD，那是个输入，不会被反向驱动），
 *  正好拿来当命令线：**一路接到两台 K230 的 RX**，各串一个 5V→3.3V 分压。
 *      Arduino D1 ──[1k]──┬── K230#1 IO10 (UART1_RXD)
 *                         ├── K230#2 IO12 (UART2_RXD)
 *                         └──[2k]── GND        ← 5V→3.33V
 *
 *  命令都是单字节，K230 侧只认自己关心的那条：
 *    'A' → K230#1（物块）  开始识别并发送
 *    'M' → K230#1          关闭 REPL 打印（识别结果照发，只是不再 print）
 *    'B' → K230#2（放置）  开始识别并发送
 *    'X' → 全部            停止发送，进入待机（可被 A/B 重新唤醒）
 *
 *  ★ 这条链路顺带把"两台相机抢同一条 RX"的问题彻底解决了：
 *    任何时刻只有一台在发，物理上不可能撞车。
 *
 *  ★ 波特率注意：命令和图像数据走的是**同一个 USART**，TX 与 RX 同速率。
 *    所以必须"先用当前速率发 X，切完再用新速率发 A/B" —— 见 visionBaud()。
 *  ★ 切换前必须 Serial.flush()，否则最后一个字节会被 end() 截断。
 */
#define CMD_WAKE_BLOCK  'A'
#define CMD_MUTE_PRINT  'M'
#define CMD_WAKE_SITE   'B'
#define CMD_SLEEP_ALL   'X'

static void k230Send(char c)
{
#if K230_CMD_ENABLE
    Serial.write((uint8_t)c);
    Serial.flush();          // 等真的移出线路
#else
    (void)c;                 // 菊花链方案：没有命令线，直接空操作
#endif
}

/* ★★★ 放置区相机接入方式（改这一行即可）★★★
 *   1 = 与物块相机【共用 D0 硬件串口】（当前，推荐）
 *        ST_IDLE 阶段切 9600 收物块相机 "1,1"
 *        任务阶段切 115200 收放置相机 "<11>DC"
 *        硬件 USART 收字节全程由移位寄存器完成，CPU 完全不参与 ——
 *        不关中断、没有位时序误差、不打扰舵机/编码器/超声波。
 *        代价：两路 TX 必须各串 1kΩ 再并联，且两台相机不能同时说话
 *        （靠"按阶段切波特率 + 只在本阶段解析本路协议"天然隔离）。
 *   0 = 放置相机单独走软串口（原方案，D11 或 D2）
 */
#define SITE_ON_HW_UART  1

// 物块种类：1 圆柱 / 2 三棱柱 / 3 正方体（0 = 无）
int visTask = 0;
unsigned long visTaskMs = 0;
/* 物块相机识别到之后要不要关它的 REPL 打印：只关一次，避免每帧都发命令。
 * 打印会占 K230 的 CPU（一次 print 可能几十 ms），关掉能明显提帧率。 */
bool printMuted = false;

// 放置区域号：1 / 2 / 3（0 = 无）
int visSite = 0;
unsigned long visSiteMs = 0;

/* 物块相机每 100ms 重发 → 1200ms 无新帧即失效。 */
#define VISION_TIMEOUT_MS 1200UL
/* ★ 放置区相机（111.py）每 SEND_PERIOD_MS(400ms) 发一次心跳，所以这里可以
 *  收紧：1.5s 收不到任何合法帧 = 链路断了，立即把 visSite 清零，
 *  而不是像老协议那样靠 30s 长超时兜底（旧做法会把上一个盒子的区域号
 *  一直留到下一次投放，正是"幽灵投放/不投放"的来源之一）。
 *  相机侧无目标时会显式发 <00>，因此正常情况下 visSite 自己就会归零。 */
#define SITE_TIMEOUT_MS   2500UL

/* ========================= 0. 任务流程（单任务·间隙跳变判区） =========================
 *  一个搬运任务 = 取件 + 投放：
 *
 *    物块相机(D0) → 【搬取物种类】识别 → 1,1 圆柱 / 1,2 三棱柱 / 1,3 正方体
 *    放置区相机(D0) → 【放置区域】识别 → <11> 圆形 / <22> 三角形 / <33> 正方形
 *                  归一成区域号 1 / 2 / 3（相机同时报 <00> 表示没看到标志）
 *
 *  ★ 现场布局：路线旁沿线依次摆着多个停车盒（放置区），车从盒子旁边经过，
 *    侧向超声波对着盒壁，所以车"贴着某个盒"时测距会稳定在较近的值。
 *
 *  ★ 到站判定：测距 ≤ STOP_CM(22cm) 连续 3 个样本 → 停车 → 读放置区相机
 *    给出该盒的区域号（圆形/三角形/正方形标志 → 1/2/3）。
 *
 *  完整链路（一个任务）：
 *    开机 → 原地等 物块相机识别
 *     └─[取件] 识别到搬取物种类 f(1..3) → qu() → 出发直行 1.5s
 *           └─[巡线] 循迹 + 超声波测距
 *                 └─[到站] 测距 ≤22cm → 停车稳定 150ms → 读区域号：
 *                       ├─ 区域号 == f → fan()+gui() 投放 → 前进离开 → 完成
 *                       └─ 区域号 != f（或等不到相机）→ 前进走出当前盒 → 继续找
 *
 *  ★ 离开用"测距变远"判定（> LEAVE_CLEAR_CM），不用固定时长：
 *    盒宽与车速都会变，固定时长要么没走出去（刚回巡线又停）、
 *    要么一下跨过两个盒（漏掉目标）。
 * ========================================================== */
int f = 0;              // 搬取物种类（1~3），由物块相机给出

bool dropDone   = false;   // 本次任务是否已【完成】投放（动作走完才算）
bool dropSent   = false;   // 本次到站是否已下发 fan()/gui()
int  dropCode   = 0;       // 本次到站锁存的区域号（一旦读到就固定，见 ST_DROP）
uint8_t stopCnt = 0;       // 本任务内第几次到站停车（从 1 开始数）
/* ★ 任务成功完成 → 强制重新武装取件。
 *   原来只靠"相机报 0,0（物体看不见了）"来复位 taskArm，这有个死锁：
 *   物块相机的值在任务期间是冻结的（串口切到 115200），任务结束时若
 *   **下一个物块已经在视野里**，visTask 会从 1 直接变成 2、永远不经过 0，
 *   taskArm 就再也不会置位 —— 表现是"第一个任务跑完，之后再也不取件"。
 *   投放成功意味着手里那个物块确实已经放掉了，此时重新武装是安全的。 */
bool taskRearm  = false;

/* ★★★ 投放判据开关 —— 排障与兜底都用它，一次就能定位"为什么不投放"
 *
 *   0 = 严格按区域号：visSite == f 才投放        ← 比赛要求，依赖放置区相机
 *   1 = 按到站顺序  ：第 f 次停车就投放           ← 不依赖相机，相机彻底不可用时的兜底
 *   2 = 到站即投放  ：任何一次停车都投放          ← 纯排障，用来验证 fan()/gui() 本身
 *
 *   判定方法：
 *     设成 2 烧录 →
 *        三个站点都投放了  ⇒ fan()/gui()/舵机/动作序列全好，问题 100% 在 visSite
 *        三个站点还是不投  ⇒ 问题在动作层（servoReached / actQueue），与视觉无关
 *     确认后再决定用 1（顺序兜底）还是 0（正式比赛）。
 * ==================================================================== */
#define DROP_MODE        0

#define START_FWD_MS   1500UL   // 取件后出发直行时长
/* ---- 到站停车与离开（超声波判据）★ 按实车标定 ----------------------
 *  STOP_CM        : 测距 ≤ 此值 → 认定到达投放点，停车读放置区相机
 *  STOP_CONFIRM   : 连续 N 个样本都 ≤ STOP_CM 才停（挡单帧多径反射）
 *  LEAVE_CLEAR_CM : 离开时测距超过它 → 已走出当前盒子，可恢复巡线
 *  LEAVE_MIN_MS   : 离开时至少前进这么久，防止在盒边反复停/走
 *  LEAVE_MAX_MS   : 离开超时兜底（无回波或一直贴着盒子）
 * ------------------------------------------------------------------ */
#define STOP_CM          24.0   // ★ 到站停车距离（cm）—— 按实车要求：小于此值即视为到达盒子
                                /* ★★★ 为什么从 22 抬到 26（超声波停车失效的真正根因）★★★
                                 * 现场是【侧向超声波对着盒壁】，车沿黑线从盒子旁边经过。
                                 * 测距曲线是一条"谷"：distance = sqrt(h² + x²)，h = 盒子到路线的
                                 * 横向距离，x = 车与盒子的纵向偏移。谷底就是 h。
                                 *
                                 * 蒙特卡洛仿真（400次/档，±1cm 抖动，30cm/s，20ms 采样）：
                                 *   盒子横距 h   带内采样数   STOP_CM=22 成功率   STOP_CM=26 成功率
                                 *      18cm          63            100%                100%
                                 *      20cm          55            100%                100%
                                 *      21cm          51            100%                100%
                                 *      22cm          47            100%                100%
                                 *      24cm          33              0%   ←停车完全失效   100%
                                 *      26cm           1              0%   ←停车完全失效   100%
                                 *
                                 * 也就是说：只要盒子横距落在 22~26cm 这段，
                                 * 谷底整条都落在判据之外，distance 从头到尾没进过 (5,22]，
                                 * cnt 永远是 0 —— 车"经过投放盒不停"，一片叶子都没沾上。
                                 * 这与"读数确实没降到 22 以下"的现象完全吻合。
                                 * 把阈值抬到 26cm 后，h≤24cm 全部回到 100%。 */
#define STOP_HYST_CM      4.0   // ★ 到站滞回：进入 5~26cm 后，短暂跳到 26~30cm
                                //   （边界抖动）不清零 cnt，只有 >30 或 <=5 才清。
#define STOP_CONFIRM      2     // ★ 连续 2 个样本（40ms）即可判停。
                                //   原来是 3（要求 3 个连续样本都在带内）；超声波有
                                //   ±1~2cm 抖动，要求连续 3 个会让 cnt 在边界反复清零。
                                //   仿真：盒子停靠 20cm 时，CONF=3 只有 94%，CONF=2 是 100%。
#define EMERG_CM          5.0   // ★ 过近急停（防撞），比到站更近，是最后防线
#define EMERG_CONFIRM     3     // 连续 3 个样本才急停
#define EMERG_UNLOCK_MS 3000UL  // 急停锁满这么久 → 强制放行（脱困）
#define ULTRA_ARM_DELAY_MS 5000UL  // ★ qu() 全部到位后，再等这么久才启用超声波。
                                //   起点区域常有相邻物块、器材腿、墙面等近距离目标，
                                //   起步阶段就判停会把车锁死在原地，一步都走不出去。
                                //   ★ 与原来的"4s 起步锁定"不同：那个是"超声波照常采样、
                                //     照常计票，只是把票丢掉"；这个是这段时间根本不开超声波，
                                //     到点之后到站与急停全程生效，不再有任何屏蔽窗口。
                                //   ⚠ 代价：启用前车按基速走过的距离里若有盒子会被冲过
                                //     （30cm/s × 5s ≈ 150cm）。
/* ★ 必须明显大于 STOP_CM(26)：若两者相等，车刚停稳（≈STOP_CM）就满足"已离开"，
 *   会在原地反复进出 ST_LEAVE。留 6cm 余量 = 32cm。 */
#define LEAVE_CLEAR_CM   32.0   // 测距 > 此值 = 已离开当前盒
#define LEAVE_MIN_MS    300UL   // 离开时的最小前进时间
#define LEAVE_MAX_MS   2000UL   // 离开超时兜底
#define SETTLE_MS       400UL   // ★ 150 → 400：等车身稳 + 等 111.py 曝光稳定。
                                //   原值 150ms 太短 —— 车刚停稳时机械振动会让 111.py 画面糊掉，
                                //   这段时间很可能一帧有效帧都没有。
#define DROP_WAIT_MS   1800UL   // ★ 800 → 1800：等相机报区域号的上限，超时当作"不是目标"。
                                //   111.py 心跳 SEND_PERIOD_MS=400ms，且新码要等 TRACK_MIN_HITS=4
                                //   帧确认 + SEND_MIN_GAP_MS=200ms 限流才发出，最坏 ~600ms。
                                //   原窗口 150+800=950ms 余量只有 350ms，任何一次 GC 停顿 /
                                //   Otsu 重估错过窗口 → "到站了却不投放"。
                                //   现窗口 400+1800=2200ms ≈ 5.5 个心跳周期，可自愈。
#define DROP_TAIL_MS    300UL   // ★ 投放动作全部到位后再停这么久才离开。
                                //   servoReached 只保证"舵机角度到了"，而夹爪张开、
                                //   挡板下落要把物块真正放下去还需要一点时间。
                                //   原来一到位就开走，物块经常落在盒子边缘。
                                //   / Hold still briefly after the servos reach their
                                //     targets: the gripper is open but the block is still
                                //     falling, so driving off immediately lands it on the rim.
#define TASK_TIMEOUT_MS 40000UL // 单任务兜底超时：卡住也能自己回 IDLE

bool ultraArmed = false;        // 超声波是否已启用（qu 走完 + ULTRA_ARM_DELAY_MS 后置 true，
                                //   见 move_tai() 顶部"超声波启用闸"）
bool stopLocked  = false;       // 已判定到达目标区域，锁住电机
bool zhiLocked   = false;       // 对正锁：q==110 进入，连续稳回线上才退出

/* 直角弯流程：5cm 急停解除 → 15cm 半径圆弧 → (看到黑线则)1s 巡线 → 后退到丢线 → kai()
 * quiet 置位期间忽略 1,x，也不再输出任何串口内容。 */
bool quiet       = false;       // 直角弯 → kai() 全程的静默闸
bool cornerReq   = false;       // 5cm 急停已触发，解除后要起直角弯
unsigned long cornerReqMs = 0;  // 置位时刻：超过 CORNER_REQ_MS 未消费就作废，
                                // 防止它在 ST_DROP / ST_LEAVE 里残留、之后凭空引爆

float distance = 999;           // 超声波测距 cm（999 = 无回波）
uint16_t distSeq = 0;           // 样本序号：每个 20ms 周期只 +1，判区按它取样本

unsigned long fwdUntil  = 0;    // 出发段结束时刻
/* ★★★ 超声波启用时刻：qu() 动作全部到位后再等 ULTRA_ARM_DELAY_MS 才开超声波 ★★★
 *
 *   原来的做法是"起步 4s 锁定"——超声波照常采样、照常计票，只是把票丢掉。
 *   那种做法有两个毛病：
 *     ① 逻辑绕：锁定窗口内 cnt 每轮被强制清零，和"没启用"效果一样，
 *        却在代码里多出一条容易被误改的分支；
 *     ② 语义错：既然这段时间的读数一律作废，不如干脆不开超声波。
 *
 *   现在改为：qu() 走完 → 记 ultraStartAt = millis() + ULTRA_ARM_DELAY_MS，
 *   到点才置 ultraArmed。此后到站(STOP_CM)与急停(EMERG_CM)全程生效，
 *   不再有任何"屏蔽窗口"。
 *   ⚠ 代价：启用前车按基速走过的距离里若有盒子会被冲过
 *     （30cm/s × 5s ≈ 150cm），起点到第一个投放盒需大于这个距离。 */
unsigned long ultraStartAt = 0;  // 超声波启用时刻（qu 到位 + ULTRA_ARM_DELAY_MS）

unsigned long leaveStart = 0;   // 投放后前进离开的起点时刻
unsigned long dropStart = 0;    // 到区停车稳定的起点时刻
unsigned long dropTailUntil = 0;// 投放动作完成后的停留截止时刻（见 DROP_TAIL_MS）
unsigned long zhiStart  = 0;

/* 任务阶段 */
#define ST_IDLE      0    // 无任务执行，原地等物块相机
#define ST_PICK_FWD  1    // 取件 + 出发段直行
#define ST_LINE      2    // 巡线 + 判区
#define ST_DROP      3    // 到达目标区域，投放
#define ST_LEAVE     4    // 投放后前进离开
#define ST_CORNER    5    // 5cm 急停后：直角弯 → 可选巡线/后退 → kai()
int stage = ST_IDLE;
unsigned long taskStartMs = 0;  // 任务起点，供 TASK_TIMEOUT_MS 兜底

/* 循迹三探头原始值 + 合成状态码（du() 写、move_tai 读）
 * fq = 最后一次"看得见线"的状态，丢线盲走时用来判断往哪边找 */
int a = 0, b = 0, c = 0;
int q = 0;
int fq = 0;

/* 两轮累计里程（脉冲数，带符号）—— 只给直行航向保持用，任务启动时清零 */
volatile long encCntL = 0, encCntR = 0;

/* 航向保持的状态量（见 4.5 节）
 *   prevCnt*  : 上一拍的里程快照
 *   headE     : 累积里程差 cm（正 = 右轮多走 = 车头偏左）
 *   headCorr  : 缓存的修正量 cm/s，供 qian() 每轮直接取用
 *   straightMode : 当前这一拍是否"应当直行"。转弯时不计入里程差 */
static long   prevCntL = 0, prevCntR = 0;
static double headE    = 0;
static double headCorr = 0;
static bool   straightMode = true;

/* ========================= 3.5 机构动作序列（非阻塞） =========================
 *  四个动作都是"几个舵机按顺序转到位"，用同一套表驱动：
 *    每轮 actRun() 检查当前那一路是否到位，到位才下发下一条。
 *    全程不写 delay()，也不碰 tgtL/tgtR —— 车该怎么走还怎么走。
 *
 *  ★ 表里的顺序就是执行顺序（前一路到位后才动下一路）。
 *  ★ 四个入口 qu/fan/gui/kai 一律走 actQueue()：当前动作没走完就【排队】，
 *    绝不打断正在执行的动作。qu(); fan(); gui(); 连着调用会依次执行。
 * ========================================================================== */
static const uint8_t QU_PIN[3]  = {12, 7, 8};         // qu ：D12 → D7 → D8
static const float   QU_DEG[3]  = { 250, 45, 225};

static const uint8_t FAN_PIN[4] = {7, 8, 4, 12};      // fan：D7 → D8 → D4 → D12
static const float   FAN_DEG[4] = { 40, 220, 180, 180};

static const uint8_t GUI_PIN[3] = {4, 8, 7};          // gui：D4 → D8 → D7
static const float   GUI_DEG[3] = { 90, 225, 45};

static const uint8_t KAI_PIN[4] = {12, 8, 7, 4};      // kai：D12 → D8 → D7 → D4
static const float   KAI_DEG[4] = {180, 270, 30, 90};

static const uint8_t *actPins = 0;
static const float   *actDegs = 0;
static uint8_t        actLen  = 0, actIdx = 0;
static bool           actBusy = false;

/* ★★★ 待执行动作：4 格环形队列（原来是 1 格，而且 actStart 会直接抢占）★★★
 *
 *   原来的行为：actStart() 无条件清队列 + 覆盖当前动作。
 *   后果正是"上一个还没走完就进下一个" —— 已经下发的那一路舵机会停在半路，
 *   后面的步骤永远不执行。现场能看到的典型现象：
 *     · 取件 qu() 三步要 0.75~3s，而车 1.5s 就出发了；若 qu 没走完就判到站，
 *       fan() 会把 qu 顶掉 —— 夹爪停在半开位置就直接去投放。
 *     · kai() 会顶掉尚未走完的 gui()，机构回不到位。
 *
 *   现在：所有动作一律入队，actRun() 等当前动作【最后一个舵机也到位】之后
 *   才从队头取下一个。任何调用都不会打断正在执行的动作。
 *   队列满（4 个）才丢弃 —— 正常流程最多同时排 2 个（fan → gui）。
 *
 *   RAM 代价：4 × (2+2+1) = 20 字节。
 * ========================================================================== */
#define ACT_Q_MAX 4
static const uint8_t *qPins[ACT_Q_MAX];
static const float   *qDegs[ACT_Q_MAX];
static uint8_t        qLen[ACT_Q_MAX];
static uint8_t        qHead = 0;      // 队头：下一个要执行的
static uint8_t        qCnt  = 0;      // 队列里排着几个

/* 立即开始一段动作（只由 actQueue / actRun 调用） */
static void actBegin(const uint8_t *pins, const float *degs, uint8_t len)
{
    actPins = pins; actDegs = degs;
    actLen  = len;  actIdx  = 0;
    actBusy = (len > 0);
    if(actBusy) servoWrite(pins[0], degs[0]);
}

/* 入队：空闲且队列空 → 立即开始；否则排到队尾。
 * 返回 false = 队列已满，本次动作被丢弃（正常流程不会发生）。 */
static bool actQueue(const uint8_t *pins, const float *degs, uint8_t len)
{
    if(len == 0) return true;
    if(!actBusy && qCnt == 0) { actBegin(pins, degs, len); return true; }
    if(qCnt >= ACT_Q_MAX) return false;
    uint8_t t = (uint8_t)((qHead + qCnt) % ACT_Q_MAX);
    qPins[t] = pins; qDegs[t] = degs; qLen[t] = len;
    qCnt++;
    return true;
}

/* 每轮 loop 调一次；返回 true 表示当前动作与队列都空了 */
static bool actRun()
{
    if(!actBusy)
    {
        if(qCnt == 0) return true;
        const uint8_t *p = qPins[qHead];
        const float   *d = qDegs[qHead];
        uint8_t        l = qLen[qHead];
        qHead = (uint8_t)((qHead + 1) % ACT_Q_MAX);
        qCnt--;
        actBegin(p, d, l);
    }

    if(!servoReached(actPins[actIdx])) return false;   // 这一路还没转到位

    /* ★ 返回值的契约是"当前动作和队列都空了"，不是"这一段跑完了"：
     *   fan() 之后排着 gui() 时，fan 跑完 qCnt 仍是 1，这里必须返回 false。 */
    if(++actIdx >= actLen) { actBusy = false; return (qCnt == 0); }

    servoWrite(actPins[actIdx], actDegs[actIdx]);      // 下发下一条
    return false;
}

static bool actDone() { return !actBusy && qCnt == 0; }

/* 三类物料的取放动作完全一样，所以不带参数 —— 想按物块区分时再加重载。
 * ★ 四个入口全部走队列：当前动作没走完就排队，不打断。 */
void qu()  { actQueue(QU_PIN,  QU_DEG,  3); }   // 取件
void fan() { actQueue(FAN_PIN, FAN_DEG, 4); }   // 投放
void gui() { actQueue(GUI_PIN, GUI_DEG, 3); }   // 归位：排在 fan 之后
void kai() { actQueue(KAI_PIN, KAI_DEG, 4); }   // 直角弯后的机构动作

#if !SITE_ON_HW_UART          /* ← 共用 D0 硬件串口时，整段软串口代码不参与编译 */
/* ---- 放置区相机：软串口 @SITE_BAUD（PCINT 捕获起始位）-------------------
 *  ★★★ 走这条降级路径时，必须把 SITE_BAUD 改回 57600（111.py 同步改），
 *      不能沿用上面的 115200：
 *        一位的时长 = F_CPU / SITE_BAUD 个时钟周期
 *          115200 →  138 周期 (  8.7us) ← 软串口收不了，空隙塞不下舵机中断
 *           57600 →  277 周期 ( 17.4us) ← 可用
 *           38400 →  416 周期 ( 26.0us) ← 更稳，舵机误差更大
 *            9600 → 1667 周期 (104.2us)
 *      原因见 SITE_BAUD 上方那段历史说明。
 *  _delay_loop_2(n) ≈ 4n-1 个时钟周期（16 位计数，各速率都够用）。
 *
 *  ★ 采样期间**不能 sei()**：Timer2 的舵机比较中断会把每位推长，累积到第 8 位
 *    就完全错位。关中断的代价是每字节阻塞 8.5 位（57600 下约 148us，
 *    270° 舵机约 20°），而放置区相机每 400ms 才一帧，可以忽略。
 *  ⚠ 软串口下 115200 必然丢帧、9600 舵机又要甩 120°，57600 是唯一平衡点。
 * -------------------------------------------------------------------- */
/* ★★★ 接收引脚选择（改这一行即可换口，其余代码自动跟随）★★★
 *   11 = D11（PB3 / PCINT3 / PCINT0 组）  ← 当前
 *    2 = D2 （PD2 / PCINT18 / PCINT2 组）  ← 备选
 *
 *   为什么留这个开关：BigFish 拓展板的官方测试例程里写着
 *       int servo_port[6] = {4, 7, 11, 3, 8, 12};      // 舵机接口
 *       LedControl lc = LedControl(12, 11, 13, 1);    // 8×8 点阵 DIN=12 CLK=11 CS=13
 *   也就是说 **D11 在拓展板上既是舵机口、又是点阵的 CLK 线**。若点阵模块插着，
 *   D11 上就挂着 MAX7219 的 CLK 输入。它虽是高阻输入、正常不驱动，但一旦出现
 *   "K230 明明在发、D11 就是收不到"，第一件事就是把这里改成 2 —— D2 是这块
 *   板子上唯一一路完全空闲的引脚（BigFish 只把它做成 {A3,2} 传感器口）。
 *   改完记得把 K230 的 IO11 那根线一起挪到 D2。 */
#define SITE_RX_PIN     11

#if   SITE_RX_PIN == 11
  #define RX_PINR    PINB
  #define RX_BIT     _BV(PB3)
  #define RX_PCMSK   PCMSK0
  #define RX_PCINT   PCINT3
  #define RX_PCIE    PCIE0
  #define RX_PCIF    PCIF0
  #define RX_VECT    PCINT0_vect
#elif SITE_RX_PIN == 2
  #define RX_PINR    PIND
  #define RX_BIT     _BV(PD2)
  #define RX_PCMSK   PCMSK2
  #define RX_PCINT   PCINT18
  #define RX_PCIE    PCIE2
  #define RX_PCIF    PCIF2
  #define RX_VECT    PCINT2_vect
#else
  #error "SITE_RX_PIN 只支持 11(D11) 或 2(D2)"
#endif

#define SOFT_BIT_CYCLES (F_CPU / SITE_BAUD)                 // 一位的时钟周期数
/* ★ 下面这些推导与数值全部按 **57600**（一位 277 周期）算的 —— 走降级路径时
 *   SITE_BAUD 必须先改回 57600，否则这些常量会随波特率重算、标定失去意义。 */
/* ★ 必须把循环体自身开销扣掉（数值取自 objdump 实测，不是估的）：
 *   循环体 = sbic(2) + movw(1) + add(1) + subi(1) + sbc(1) + brne(2) = 8 周期，
 *   _delay_loop_2(n) = 4n-1，所以要 4n-1 ≈ 277-8 = 269 → n = 67（得 275/位）。
 *   起始的 1.5 位对齐还要扣掉进入 ISR 到执行延时之间的序言（约 42 周期）：
 *   1.5×277 = 415，415-42 = 373 → 4n-1 = 373 → n = 93。
 *   实测落点：bit0 +0.05 位、bit7 −0.02 位，全程远离 ±0.5 位容限。 */
#define SOFT_BODY_CYCLES  8
#define SOFT_BIT_LOOPS  (((SOFT_BIT_CYCLES - SOFT_BODY_CYCLES) + 1) / 4)
#define SOFT_HALF_LOOPS (((SOFT_BIT_CYCLES * 3 / 2 - 35) + 1) / 4)

/* ★ 环形缓冲必须明显大于一帧长度。
 *   111.py 一帧 7 字节，而 8 槽环的可用容量只有 7（head==tail 视为空）——
 *   刚好卡在临界值上：主循环只要在整帧期间被耽搁一次（PID 双精度运算、
 *   超声波处理），第 7 个字节 '\n' 就会被 ISR 丢掉，解析器于是永远等不到
 *   收尾，整帧作废。扩到 16 槽（可用 15）后留了一倍余量。 */
#define SITE_BUF_N   16
#define SITE_BUF_MSK (SITE_BUF_N - 1)     // 必须是 2 的幂减 1

static volatile uint8_t siteBuf[SITE_BUF_N];
static volatile uint8_t siteHead = 0, siteTail = 0;

ISR(RX_VECT)
{
    if(RX_PINR & RX_BIT) return;                // 只认下降沿（起始位）

    RX_PCMSK &= (uint8_t)~_BV(RX_PCINT);        // 采样期间屏蔽自己，防止数据位重入
    PCIFR    |= _BV(RX_PCIF);

    if(RX_PINR & RX_BIT)                        // 毛刺复核：不是真起始位就退出
    {
        RX_PCMSK |= _BV(RX_PCINT);
        return;
    }

    _delay_loop_2(SOFT_HALF_LOOPS);             // 对齐到 bit0 中心

    /* ★ 用【移位掩码】而不是 (1 << i)：后者会被 gcc 编译成"循环左移 i 次"，
     *   每多一位就多花 4 个周期 —— 第 0 位约 +10 周期、第 7 位 +36 周期，
     *   而一位总共只有 138 周期。于是位周期逐位变长，到高位已经错了 1 个位以上，
     *   115200 下必然读出乱码。掩码版本每次循环固定多花 4~7 个周期，可以补偿。 */
    uint8_t v = 0;
    for(uint8_t m = 1; m; m <<= 1)
    {
        if(RX_PINR & RX_BIT) v |= m;             // 低位在前
        _delay_loop_2(SOFT_BIT_LOOPS);
    }

    RX_PCMSK |= _BV(RX_PCINT);                   // 重新武装

    uint8_t nxt = (uint8_t)((siteHead + 1) & SITE_BUF_MSK);
    if(nxt != siteTail) { siteBuf[siteHead] = v; siteHead = nxt; }
}
#endif   /* !SITE_ON_HW_UART */

/* ---- 严格字段解析：1~2 位纯数字，且以 ',' 或字符串结束收尾；否则 -1 ----
 *  ★ 不能用"取个位(%10)"：那会把 "1,11" 折叠成 1、"1,13" 折叠成 3、
 *    "1,10" 折叠成 0。噪声、丢字节拼接出来的帧就会被当成合法类别，
 *    触发幽灵任务或误清零。宁可整帧丢弃，也不要误采信。
 * -------------------------------------------------------------------- */
static int parseField(const char *s)
{
    if(*s < '0' || *s > '9') return -1;          // 空字段 / 负号 / 字母
    int v = 0;
    while(*s >= '0' && *s <= '9')
    {
        v = v * 10 + (*s - '0');
        if(v > 99) return -1;                    // 超长数字，溢出前就掐掉
        s++;
    }
    return (*s == '\0' || *s == ',') ? v : -1;   // 尾部必须是结束或下一字段
}

/* ---- 物块相机单帧解析（D0 硬件串口，4_v4.py）---------------------------
 *  协议保持原样，未改动：逗号分隔的裸 ASCII
 *      "1,1" / "1,2" / "1,3" → 物块种类 1/2/3 → visTask
 *      "0,0"                 → 无目标 → 清零
 *
 *  ★ 放置区相机的 '<dd>CHK\n' 帧由 siteFeed() 的状态机解析，不走本函数。
 * -------------------------------------------------------------------- */
static void parseVisionFrame(const char *s)
{
    const char *comma = strchr(s, ',');
    if(comma == 0 || comma == s) return;

    /* ★ 首字段必须**恰好一位**数字（协议是 "1,x" / "0,x"）。
     *   原来只要求"以数字开头"，"12,3"、"99,1" 这类噪声或丢字节拼出来的串
     *   也会被当成合法帧；锁死一位后，长度不对整帧丢弃，宁可漏一帧。 */
    if(comma != s + 1) return;

    int val = parseField(comma + 1);
    if(val < 0) return;

    /* ★ 首字段的【取值】也要校验，不只校验长度。
     *   协议只有两种：物块相机 "1,1/1,2/1,3"（有目标）和 "0,0"（无目标）。
     *   原来不校验首字段值，"0,2" 会被当成物块 2 而误起任务，
     *   "2,1" 这类噪声也会被采信。 */
    if(*s == '0')
    {
        if(val == 0) visTask = 0;                // "0,0" = 无目标
        return;                                  // "0,x" 其它组合一律不采信
    }
    if(*s != '1') return;                        // 首字段只允许 0 或 1

    if(val < 1 || val > 3) return;               // 非法/半帧：保留旧值

    visTask   = val;
    visTaskMs = millis();

    /* ★ 识别成功的那一刻，通知物块相机把 REPL 打印关掉（只发一次）。
     *   识别结果照常发送，只是不再 print —— 打印在 K230 上很贵。 */
    if(!printMuted)
    {
        printMuted = true;
        k230Send(CMD_MUTE_PRINT);
    }
}

/* ---- 软串口成帧解析（放置区相机，D11）--------------------------------
 *  111.py 发的帧格式（7 字节 ASCII）：
 *      '<' d0 d1 '>' h1 h0 '\n'
 *      d0 d1 = 两位十进制编码：11 圆 / 22 三角 / 33 方 / 00 无目标
 *      h1 h0 = ('<' + d0 + d1 + '>') 的字节累加和，两位大写十六进制
 *
 *  为什么用状态机而不是"攒到 \n 再按逗号切"：
 *    软串口在这种负载下仍可能偶发错位。裸 ASCII 一旦错一个字节，
 *    后面所有帧的边界就全乱了，而且自己察觉不到。带帧头 + 校验后：
 *      · 任何位置看到 '<' 就立刻重新对齐，坏帧最多丢一帧；
 *      · 校验不过直接丢弃，绝不把半帧当成合法区域号（避免"幽灵投放"）。
 * -------------------------------------------------------------------- */
static int hexVal(char ch)
{
    if(ch >= '0' && ch <= '9') return ch - '0';
    if(ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    if(ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    return -1;
}

/* 放置区帧解析状态机：**逐字节喂入**。
 *   硬件串口方案：pollVision() 每收到一个字节直接调它；
 *   软串口方案  ：pollSiteVision() 从环形缓冲里取字节再调它。
 *   两条路共用同一份解析逻辑，避免两份代码行为不一致。 */
static void siteFeed(char ch)
{
    static uint8_t st = 0;                   // 0=等'<' 1=d0 2=d1 3=等'>' 4=h0 5=h1 6=等'\n'
    static uint8_t d0 = 0, d1 = 0, h0 = 0, h1 = 0;

    if(ch == '<') { st = 1; return; }         // 帧头：任何位置都能重新对齐

    switch(st)
    {
        case 1:
            if(ch >= '0' && ch <= '9') { d0 = (uint8_t)(ch - '0'); st = 2; }
            else st = 0;
            break;

        case 2:
            if(ch >= '0' && ch <= '9') { d1 = (uint8_t)(ch - '0'); st = 3; }
            else st = 0;
            break;

        case 3:
            st = (ch == '>') ? 4 : 0;
            break;

        case 4:
            {
                int v = hexVal(ch);
                if(v >= 0) { h0 = (uint8_t)v; st = 5; } else st = 0;
                break;
            }

            case 5:
            {
                int v = hexVal(ch);
                if(v >= 0) { h1 = (uint8_t)v; st = 6; } else st = 0;
                break;
            }

            case 6:
                if(ch == '\n')
                {
                    uint8_t sum = (uint8_t)((uint8_t)'<' + (uint8_t)('0' + d0) +
                                            (uint8_t)('0' + d1) + (uint8_t)'>');
                    if(sum == (uint8_t)((h0 << 4) | h1))
                    {
                        uint8_t code = (uint8_t)(d0 * 10 + d1);
                        if(code == 11 || code == 22 || code == 33)
                        {
                            visSite   = code / 11;          // 11/22/33 → 1/2/3
                            visSiteMs = millis();
                        }
                        else if(code == 0)
                        {
                            visSite   = 0;                  // 相机明确报"无目标"
                            visSiteMs = millis();
                        }
                    }
                }
                st = 0;
                break;

            default:
                st = 0;
                break;
    }
}

#if !SITE_ON_HW_UART
/* 软串口方案：把环形缓冲里的字节逐个喂给上面的状态机 */
static void pollSiteVision()
{
    while(siteTail != siteHead)
    {
        char ch = (char)siteBuf[siteTail];
        siteTail = (uint8_t)((siteTail + 1) & SITE_BUF_MSK);
        siteFeed(ch);
    }
}
#endif

/* ---- 视觉入口 ---------------------------------------------------------
 *  SITE_ON_HW_UART = 1（当前）：两台相机共用 D0，按阶段切波特率
 *      ST_IDLE  → 9600   只解析物块相机 "1,1"
 *      任务阶段 → 115200  只解析放置相机 "<11>DC"
 *  SITE_ON_HW_UART = 0        ：物块相机 D0(9600) + 放置相机软串口
 * -------------------------------------------------------------------- */
#if SITE_ON_HW_UART
static void visionBaud(bool blockMode)
{
#if SITE_BAUD == VISION_BAUD
    /* 菊花链方案：两端都是 9600，不需要切波特率，也不该切 ——
     * 每切一次 Serial.end() 都会把 RX 环形缓冲清空、丢掉正在收的半帧。 */
    (void)blockMode;
    return;
#endif
    /* ★ 初值 0 而不是 -1：setup() 里已经 Serial.begin(VISION_BAUD)，
     *   初值同步成"9600 已生效"，第一次调用就不会白做一遍 end/begin。 */
    static int8_t cur = 0;                   // 0=9600 1=115200
    int8_t want = blockMode ? 0 : 1;
    if(want == cur) return;

    /* ① 先用【当前】速率让正在工作的那台停下 —— 否则它会继续往 RX 上灌数据，
     *    而我们马上就要把波特率改掉，那串数据必然变成乱码。 */
    k230Send(CMD_SLEEP_ALL);

    /* ② 切速率 */
    Serial.end();                            // 切速率要重新初始化 USART
    Serial.begin(blockMode ? VISION_BAUD : SITE_BAUD);
    cur = want;

    /* ③ 再用【新】速率唤醒该上场的那台 */
    k230Send(blockMode ? CMD_WAKE_BLOCK : CMD_WAKE_SITE);

    /* ④ 回到待机时把区域号作废。
     *   相机侧收到 'X' 后虽然会补发一帧 <00>，但那时串口已经切到 9600，
     *   这一帧多半会作为乱码丢掉。与其指望它，不如主控自己清干净 ——
     *   否则下一个任务到站时会拿着上一个盒子的区域号去比对。 */
    if(blockMode) { visSite = 0; visSiteMs = 0; }
}
#endif

static void pollVision()
{
    static char buf[16];
    static uint8_t idx = 0;

#if SITE_ON_HW_UART
    bool blockMode = (stage == ST_IDLE);     // 只有待机时才需要听物块相机

    /* ★ 只在"模式切换的那一次"清缓冲：放在循环里会把每个字节都清掉，
     *   "1,1" 永远攒不满，物块相机相当于完全失效。 */
    static bool lastBlock = true;
    if(blockMode != lastBlock)
    {
        idx = 0;
        lastBlock = blockMode;
        /* 回到待机：区域号作废，避免下一个任务拿着上一个盒子的值去比对 */
        if(blockMode) { visSite = 0; visSiteMs = 0; }
    }

    visionBaud(blockMode);

    /* ★ 命令心跳：每秒重发一次唤醒码。
     *   两台 K230 的上电时间不确定，可能比主板晚几百 ms，只发一次会漏掉；
     *   而漏掉的后果是"相机一直待机、车不动"。重发是幂等的（重复 'A' 无害），
     *   代价只有每秒 1 个字节。
     *   顺带还兜住了另一种情况：某台 K230 中途被看门狗复位——
     *   它重启后 active 默认为假，靠这条心跳在一秒内重新唤醒。 */
#if K230_CMD_ENABLE
    static unsigned long lastCmdMs = 0;
    if(millis() - lastCmdMs > 1000UL)
    {
        lastCmdMs = millis();
        k230Send(blockMode ? CMD_WAKE_BLOCK : CMD_WAKE_SITE);
    }
#endif
#endif

    while(Serial.available() > 0)
    {
        char ch = (char)Serial.read();

#if SITE_ON_HW_UART
        if(!blockMode)
        {
            /* 任务中：串口在 115200，只可能是放置区相机的帧 */
            siteFeed(ch);
            continue;
        }
        /* 待机中：串口在 9600，只可能是物块相机的 "1,1" / "0,0" */
#endif
        if(ch == '\n' || ch == '\r')
        {
            if(idx > 0) { buf[idx] = 0; parseVisionFrame(buf); idx = 0; }
        }
        else if(ch >= 32 && ch < 127 && idx < sizeof(buf) - 1)
        {
            buf[idx++] = ch;
        }
        else
        {
            idx = 0;
        }
    }

#if !SITE_ON_HW_UART
    pollSiteVision();                        // 软串口：从环形缓冲取字节
#endif

    /* ★ 两路的超时只在"本阶段真的在听它"时才计时：
     *   任务中物块相机的串口已经切到 115200，本来就收不到，若照常超时清零，
     *   回到 ST_IDLE 时 taskArm 会立刻复位 → 同一个物块被重复取。
     *   反之待机时放置区相机的值也会过期，不该拿去判区。 */
#if SITE_ON_HW_UART
    if(blockMode)
    {
        if(visTask && millis() - visTaskMs > VISION_TIMEOUT_MS) visTask = 0;
    }
    else
    {
        if(visSite && millis() - visSiteMs > SITE_TIMEOUT_MS)   visSite = 0;
    }
#else
    if(visTask && millis() - visTaskMs > VISION_TIMEOUT_MS) visTask = 0;
    if(visSite && millis() - visSiteMs > SITE_TIMEOUT_MS)   visSite = 0;
#endif
}

/* 视觉接收 + 任务启动
 *   物块种类 f ← visTask（4_v4.py，D0，1/2/3）
 *   区域号 r   ← visSite（111.py，D11），在 ST_DROP 停车之后读取
 *  ---------------------------------------------------------------------- */
static void pollVisionAndQueue()
{
    pollVision();

    /* 直角弯 → kai() 全程忽略 1,x：这期间不许起新任务，也不输出任何串口。
     * 注意只是不采信，pollVision() 仍在跑，值不会因为停更而超时失效。 */
    if(quiet) return;

    /* 搬取物种类：只在空闲阶段才维护"是否允许启动"。
     *  两个坑必须同时堵：
     *   · 若在任务进行中也因收到 0,0 把 taskArm 置回 true，
     *     任务一结束就会立刻起一个"没取件"的幽灵任务；
     *   · 若在任务进行中记录识别值，回到 IDLE 后它仍等于当前值，
     *     物体还在也永不启动（永久空闲）。 */
    static bool taskArm = true;

    if(stage != ST_IDLE) return;

    /* ★ 任务成功投放后由 ST_LEAVE 置位：无条件重新武装。
     *   这条是补"visTask 从 1 直接变 2、永远不经过 0"的死锁，
     *   细节见 taskRearm 声明处。 */
    if(taskRearm) { taskArm = true; taskRearm = false; }

    if(visTask == 0) taskArm = true;        // 必须先看不到，才允许下一次触发

    if(taskArm && visTask >= 1 && visTask <= 3)
    {
        taskArm     = false;
        f           = visTask;
        qu();
        fwdUntil    = millis() + START_FWD_MS;   // 出发段从任务启动起计时（边走边取件）
        taskStartMs = millis();
        stage       = ST_PICK_FWD;
        ultraArmed  = false;
        stopLocked  = false;
        zhiLocked   = false;
        dropDone    = false;
        dropSent    = false;
        dropCode    = 0;          // 上个任务锁存的区域号不能带到新任务
        dropTailUntil = 0;        // 投放停留计时也不能带过来
        stopCnt     = 0;          // 到站计数归零：DROP_MODE=1 靠它判"第几个盒子"
        cornerReq   = false;      // 上一个任务残留的拐弯请求不能带到新任务
        /* ★ 超声波暂不启用：qu() 走完（在 ST_PICK_FWD 里等 actDone）后，
         *   由那边设 ultraStartAt = millis() + ULTRA_ARM_DELAY_MS。
         *   这里先置 0 并把 ultraArmed 关掉，保证起步阶段超声波完全不参与判停。 */
        ultraArmed  = false;
        ultraStartAt = 0;
        quiet       = false;
        fq          = 0;          // 丢线盲走的方向记忆也清掉，避免沿用上个任务的

        uint8_t sreg0 = SREG;     // 航向里程清零：每个任务从零偏置开始
        cli();
        encCntL = 0;  encCntR = 0;
        SREG = sreg0;
        prevCntL = 0;  prevCntR = 0;
        headE    = 0;  headCorr = 0;
        /* 不清 visTask：留着它才能让 taskArm 保持 false，
         * 避免任务完成后"同一物块还在视野里"被重复触发。
         * 但上一任务的区域值必须清掉，否则停车后第一帧就误判。 */
        visSite     = 0;  visSiteMs = 0;
    }
}

/* ========================= 1. 编码器 =========================
 * 用引脚变化中断(PCINT)数脉冲，顺便量出"相邻两个脉冲之间的时间"。
 *
 * 为什么不用轮询那一套？
 *   15 孔码盘一圈才 30 个脉冲，等于 0.52cm 才有一次计数。
 *   如果按"50ms 窗口里数到几个脉冲"来算速度，30cm/s 时窗口里只有 3 个脉冲，
 *   分辨率就是 ±10cm/s，PID 会被这个量化台阶搞疯。
 *   所以这里改成测周期：速度 = 0.5236cm / 两个脉冲的间隔时间，
 *   micros() 精度 4us，测速分辨率立刻拉满。
 * ========================================================== */
volatile unsigned long encDtL  = 0, encDtR = 0;   // 相邻两个边沿的间隔(us)
volatile unsigned long encTmL  = 0, encTmR = 0;   // 上一个边沿发生的时刻(us)
volatile uint8_t       pcLast  = 0;
volatile int8_t        dirL = 1, dirR = 1;         // 当前指令方向 +1正转 -1反转（ISR 也读）

// 单路编码器分不出转向，所以脉冲的正负用"当前指令方向"来定 —— 这点后面讲
// ★ A2/A3 = PC2/PC3 = PCINT10/PCINT11 → 属于 PCINT1 组（PCMSK1 / PCIE1）
//   核对依据 avr/iom328p.h：PCMSK1 段内 PCINT10=bit2、PCINT11=bit3，
//   而 PCMSK1 覆盖的是 PC0~PC6（即 A0~A5）。
ISR(PCINT1_vect)
{
    unsigned long now = micros();
    uint8_t cur = PINC;
    uint8_t chg = cur ^ pcLast;
    pcLast = cur;

    if(chg & _BV(PC2))                 // A2 → 右编码器
    {
        encDtR = now - encTmR;
        encTmR = now;
        encCntR += dirR;
    }
    if(chg & _BV(PC3))                 // A3 → 左编码器
    {
        encDtL = now - encTmL;
        encTmL = now;
        encCntL += dirL;
    }
}

/* 由"脉冲间隔"换算速度(cm/s)，顺带处理停转 */
double readSpeedCmps(volatile unsigned long *dt, volatile unsigned long *tm)
{
    // ★ 修补：encDt/encTm 是 4 字节变量、在中断里被整体改写，
    //   主循环直接读有可能读到"前两字节是新的、后两字节是旧的"（撕裂读）。
    //   一旦撕裂，age 会大得离谱 → 测速瞬间掉到 0 → PID 当场抽一下。
    //   所以先关中断把两个值快照出来再算。关中断只有几条指令，不会丢脉冲。
    /* ★ age 也必须在同一个临界区里取：原来 tmSnap 在 cli 内快照、age 却在
     *   开中断之后才用 micros() 算，中间可能插进一次编码器中断把 encTm 推后，
     *   于是 age 相对 tmSnap 少算了那一段（方向偏小，影响不大但没道理）。 */
    unsigned long d, tmSnap, age;
    uint8_t sreg = SREG;
    cli();
    age    = micros();                       // 先取"现在"，再和快照一起算
    d      = *dt;
    tmSnap = *tm;
    SREG = sreg;

    if(tmSnap == 0) return 0.0;               // 上电还没来过脉冲，别报幻影速度

    age -= tmSnap;                            // 距最后一个脉冲过去多久
    unsigned long t = (d > age) ? d : age;    // 取大者：停转时会随 age 平滑掉到 0

    if(t == 0 || t >= STALL_US) return 0.0;
    return CM_PER_EDGE * 1000000.0 / (double)t;
}

/* ========================= 2. 手写 PID =========================
 * 就是你贴的那条式子离散化：out = Kp*e + Ki*∫e dt + Kd*de/dt
 * 写成手写版是为了不依赖 PID_v1 库（省得装库报错），也方便看清每一项在干嘛。
 * ========================================================== */
struct PIDCtl
{
    double kp, ki, kd;
    double iTerm;      // 积分累加项
    double lastIn;     // 上一拍测量值（微分作用在测量值上，避免设定值突变时爆冲）
    int8_t lastSign;   // 上一拍目标速度的符号

    void reset() { iTerm = 0; lastIn = 0; lastSign = 1; }

    void setGains(double p, double i, double d) { kp = p; ki = i; kd = d; }

    double compute(double setpoint, double input, double dtSec)
    {
        // ★ 目标换向（比如原地转弯时右轮由正转变反转）→ 清积分，
        //   否则上一方向的积分会顶着车往反方向冲
        int8_t sgn = (setpoint >= 0) ? 1 : -1;
        if(sgn != lastSign)
        {
            iTerm    = 0;
            lastSign = sgn;
            /* 换向这一拍：测速符号取自【指令方向】(motorLR 写的 dirL/dirR)，
             * 而轮子还在按原方向惯性转 —— 直接用测量值会把误差算成"快到了"，
             * 反向力矩被削弱。这一拍按 0 处理，先刹住再说。 */
            input = 0.0;
        }

        double e = setpoint - input;

        /* 抗积分饱和：iTerm 只能占"输出上限里 kp*e 之外剩下的那部分"。
         * 只钳到 +-PWM_MAX 的话，堵转 0.4s 就能顶满 255，松开后要两秒多才退下来，
         * 表现就是"顶住以后一松手猛窜"。 */
        double pe   = kp * e;
        double room = PWM_MAX - (pe > 0 ? pe : -pe);
        if(room < 0) room = 0;

        iTerm += ki * e * dtSec;
        if(iTerm >  room) iTerm =  room;
        if(iTerm < -room) iTerm = -room;

        double d = (dtSec > 0) ? kd * (input - lastIn) / dtSec : 0.0;
        lastIn = input;

        double out = kp * e + iTerm - d;
        if(out >  PWM_MAX) out =  PWM_MAX;
        if(out < -PWM_MAX) out = -PWM_MAX;
        return out;
    }
};

PIDCtl pidL = {0, 0, 0, 0, 0, 1};
PIDCtl pidR = {0, 0, 0, 0, 0, 1};

/* ========================= 3. 电机输出层 =========================
 * 唯一直接写电机引脚的地方。pwmL / pwmR 是带符号 PWM：
 *   正数 = 正转，负数 = 反转，0 = 松开
 *
 * ★★★ 左右是怎么定的（上一版我定错了，转弯方向反了就是这里）
 *   原地左转 = 左轮倒转 + 右轮正转，原地右转反过来。
 *   你原来的 L() 是「M0 正转 + M1 倒转」，被倒转的那一路就是左轮，
 *   所以 **左轮 = M1(D6/D5)，右轮 = M0(D10/D9)**。
 *   上一版我把 M0 当左轮，结果 L() 全反了 —— 已修正。
 * ========================================================== */
/* 电机输出是否已经是 0：供 ting() 幂等判断（ST_IDLE 每轮都会调用它） */
bool motorsIdle = true;

void motorLR(int pwmL, int pwmR)
{
    motorsIdle = (pwmL == 0 && pwmR == 0);

    // 左轮 = Motor-1 : D6 正转 / D5 反转
    if(pwmL >= 0) { analogWrite(6,  pwmL);  analogWrite(5, 0);      dirL =  1; }
    else          { analogWrite(6,  0);     analogWrite(5, -pwmL);  dirL = -1; }

    // 右轮 = Motor-0 : D10 正转 / D9 反转
    if(pwmR >= 0) { analogWrite(10, pwmR);  analogWrite(9, 0);      dirR =  1; }
    else          { analogWrite(10, 0);     analogWrite(9, -pwmR);  dirR = -1; }
}

/* ========================= 4. 上层动作：只设目标速度 ========================= */
double tgtL = 0, tgtR = 0;      // 左右轮目标速度 cm/s（带符号）
double spdL = 0, spdR = 0;      // 实测速度 cm/s（低通后）

/* ========================= 4.5 直行航向保持 =========================
 *  速度环只保证"每个轮子各自跑到目标速度"，管不了车头朝向：
 *  两轮直径/摩擦/齿隙有微小差异时，车会以固定角速度慢慢偏，
 *  走一段就累积成大偏差 —— 这是**航向偏置**，纯速度环消除不掉。
 *
 *  这里用两轮【累计里程差】做 P 修正，只在直行（qian）时叠加到目标速度：
 *      右轮多走 e cm（说明车头偏左）→ 左轮加速、右轮减速 → 把车头掰回来
 *  它是一个会自动收敛的闭环，不需要人工猜 STRAIGHT_TRIM。
 *
 *  ★ 只在 qian() 生效：L/R/zhi 是刻意转向，不能被它拽回去。
 * ========================================================================== */
#define HEAD_KP       0.80      // cm/s 每 cm 里程差（仿真：0.6~1.0 全程无振荡，取 0.8）
#define HEAD_MAX      7.0       // 最大修正量 cm/s，防止它压过循迹
#define HEAD_SOFT_CM   20.0     // 里程差软饱和（12cm 轮距下约 95 度）；原 400cm 是死参数

/* 每 20ms 调一次（在 updateSpeedPID 里）。按【增量】统计，且只在直行时累积。
 * ★ 不能用"累计计数直接相减"：转弯时两轮本来就该走不同距离，
 *   一次 90° 弧线（轮距 12cm）就会积累 18.8cm 的假偏差，
 *   回到直行后带着满额修正跑偏 —— 表现就是"转完就歪、转向像没生效"。
 * ★ 也不能每轮 loop 调：里面有 double 软浮点，几十 us 一次会把循环拖慢一个量级。 */
static void headUpdate()
{
    long dl, dr;
    uint8_t sreg = SREG;                 // 32 位变量，ISR 在写，必须原子快照
    cli();
    dl = encCntL;  dr = encCntR;
    SREG = sreg;

    long dL = dl - prevCntL;
    long dR = dr - prevCntR;
    prevCntL = dl;  prevCntR = dr;

    if(!straightMode) return;            // 转向/停车期间不计入

    headE += (double)(dR - dL) * CM_PER_EDGE;     // 正 = 右轮多走 = 车头偏左
    if(headE >  HEAD_SOFT_CM) headE =  HEAD_SOFT_CM;
    if(headE < -HEAD_SOFT_CM) headE = -HEAD_SOFT_CM;

    double corr = HEAD_KP * headE;

    /* 上限取 HEAD_MAX 与"基速的 35%"里较小的那个：
     * 基速被电池兜底降到 SAT_MIN_SPEED(16) 时，±7 已是 44% 的差速，会把车拽歪；
     * 按比例压到 ±5.6 更稳。 */
    double lim = baseSpeed * 0.35;
    if(lim > HEAD_MAX) lim = HEAD_MAX;
    if(corr >  lim) corr =  lim;
    if(corr < -lim) corr = -lim;

    headCorr = corr;
}

void qian()                                  // 直行（带航向保持）
{
    straightMode = true;                     // 只有这一段计入航向偏差
    tgtL = baseSpeed + headCorr;
    tgtR = baseSpeed - headCorr;
}
#if TURN_ARC
/* 弧线内侧轮目标速度：保证不倒转（baseSpeed 被电池兜底降下来时也不会变负） */
static double arcInner(double v) { return (v < 3.0) ? 3.0 : v; }
#endif

void L()                                     // 左转
{
    straightMode = false;
#if TURN_ARC
    tgtL =  arcInner(baseSpeed - TURN_DELTA);
    tgtR =  baseSpeed + TURN_DELTA;
#else
    tgtL = -turnSpeed;
    tgtR =  turnSpeed;
#endif
}
void R()                                     // 右转
{
    straightMode = false;
#if TURN_ARC
    tgtL =  baseSpeed + TURN_DELTA;
    tgtR =  arcInner(baseSpeed - TURN_DELTA);
#else
    tgtL =  turnSpeed;
    tgtR = -turnSpeed;
#endif
}
void hou()  { straightMode = false;  tgtL = -baseSpeed * 0.7;  tgtR = -baseSpeed * 0.7; }  // 后退
// ★ zhi 对正：q==110 = 中 + a 探头同时压线 → 线偏在车的左侧 → 要往左修，
//   所以右轮快、左轮慢。zhiDelta 只管"修正力度"，符号不要动。
//   ★ 若整车左右修正方向都反了，去改头部的 TRACK_MIRROR，
//     只翻这个分支会让 q==100 与 q==110 互相矛盾。
void zhi()  { straightMode = false;
              tgtL =  baseSpeed - zhiDelta/2;
              tgtR =  baseSpeed + zhiDelta/2; }

// ★ 停车：目标清零 + 立刻松开引脚（不等 PID 那 20ms），紧急情况要的就是快
void ting()
{
    straightMode = false;
    tgtL = 0; tgtR = 0;
    pidL.reset();
    pidR.reset();

    /* ST_IDLE / ST_DROP 每轮 loop 都会调用本函数；输出已经是 0 时
     * 不必每轮再写 4 次 analogWrite（空转时能省掉一成以上 CPU）。 */
    if(!motorsIdle) motorLR(0, 0);
}

/* ========================= 5. 速度环：测速 → PID → PWM ========================= */
void updateSpeedPID()
{
    static unsigned long last = 0;
#if SAT_ADAPT
    static uint8_t satCnt = 0;       // 顶满计数：停车时清零，避免跨启停累积
#endif
    unsigned long now = millis();
    if(now - last < CTRL_PERIOD_MS) return;

    double dt = (now - last) / 1000.0;
    if(dt > 0.05) dt = 0.05;      // ★ 任何阻塞（软串口收字节、机构动作）都不该改变 PID 增益，
                                  //   否则一次 1ms 抖动会让 iTerm 一步冲高
    last = now;

    // ① 测速（乘上指令方向得到带符号的速度）
    double sL = readSpeedCmps(&encDtL, &encTmL) * dirL;
    double sR = readSpeedCmps(&encDtR, &encTmR) * dirR;

    // ② 一阶低通：滤掉码盘开孔误差带来的抖动
    spdL += SPEED_LP * (sL - spdL);
    spdR += SPEED_LP * (sR - spdR);

    // ③ 航向修正：每 20ms 更新一次，供 qian() 直接取用
    headUpdate();

    // ④ 两个轮子各跑一个 PID。目标一样、参数一样 → 两轮转速一致 → 车就直
    if(tgtL == 0 && tgtR == 0)
    {
#if SAT_ADAPT
        satCnt = 0;                  // 停车不累积顶满计数
#endif
        motorLR(0, 0);
        return;
    }

    double oL = pidL.compute(tgtL, spdL, dt);
    double oR = pidR.compute(tgtR, spdR, dt);

    // 电机启动死区补偿（PWM_MIN = 0 时不生效）
    if(oL > 0 && oL < PWM_MIN) oL = PWM_MIN;
    if(oL < 0 && oL > -PWM_MIN) oL = -PWM_MIN;
    if(oR > 0 && oR < PWM_MIN) oR = PWM_MIN;
    if(oR < 0 && oR > -PWM_MIN) oR = -PWM_MIN;

    int oLi = (int)oL, oRi = (int)oR;

#if SAT_ADAPT
    /* ---- 顶满检测：PWM 已经到 255 还追不上 → 说明电池带不动当前基速 ---- */
    if(oLi >= PWM_MAX - 2 || oLi <= -(PWM_MAX - 2) ||
       oRi >= PWM_MAX - 2 || oRi <= -(PWM_MAX - 2))
    {
        if(++satCnt >= SAT_HOLD_CNT)
        {
            satCnt = 0;
            if(baseSpeed > SAT_MIN_SPEED)
            {
                baseSpeed *= SAT_STEP;
                if(baseSpeed < SAT_MIN_SPEED) baseSpeed = SAT_MIN_SPEED;  // 严格不低于下限
            }
        }
    }
    else
    {
        satCnt = 0;
    }
#endif

    motorLR(oLi, oRi);
}

/* ========================= 6. 循迹状态机（原逻辑保留） =========================
 *  a / b / c / q / fq 已在全局区声明（见"任务阶段"上方）
 */
// （stopLocked / zhiLocked / zhiStart 都在第 0 节统一声明了）

unsigned long onlineStart = 0;   // 连续"中探头稳在线上"的起始时刻，0 = 当前不在线上
unsigned long lostStart   = 0;   // 开始丢线的时刻，0 = 没丢线

// ★★★ 超声波停车锁（stopLocked）为什么必须有：
//   老代码把 STOP() 排在 move_tai() 后面，只是在末尾把引脚擦一遍，
//   下一轮 loop 开头 move_tai() 又立刻拉满 PWM —— 电机只被"闪"一下，
//   平均输出仍接近全速，看起来就是压根没停。停车必须做成锁。

/* ★★★ 超声波时序（pulseIn 阻塞式）
 *   回波宽度(us) = 距离(cm) × 58。24cm → 1392us。
 *   ECHO_TIMEOUT_US = pulseIn 的超时上限，超时返回 0 → 判为无回波。
 *     取 30000us(≈517cm)：宁可多等一会也要拿到真实读数，
 *     因为"无回波"会同时触发急停和离开判定，宁缺毋滥。
 *   ULTRA_INTERVAL_US = 采样周期。60ms：pulseIn 在无回波时会阻塞满
 *     ECHO_TIMEOUT_US，采样周期必须大于它，否则主循环会被测距卡死。 */
#define ECHO_TIMEOUT_US   30000UL
#define ULTRA_INTERVAL_US 60000UL
#define ZHI_TIMEOUT_MS    400

/* ---- ★★★ 拐弯 / 丢线 兜底参数（治"直角拐点拐不过去"） ----
 *  问题现象：直角拐点上探头必然有一瞬间只看到 q==10，
 *           原来 zhi() 只要读到一次 10 就立刻放行去 qian()，对正被打断，
 *           车就直冲过拐点 → 直角拐不过去。
 *  修法：① 退出对正必须"连续稳定回到线上" BACK_ONLINE_MS；
 *        ② 三个探头同时离开黑线（丢线）不能停死，按最后看到的方向盲走找线，
 *           给 LOST_MS 的窗口，超时才停。
 */
#define BACK_ONLINE_MS  100     // 中探头连续稳在线上这么久，才敢退出对正/拐弯
#define LOST_GRACE_MS    60     // 刚丢线先沿用原动作，不理会单次采样抖动
#define LOST_MS         600     // 丢线后最多盲走找线这么久，超时停车
/* ★ 丢线恢复：原来 600ms 一超时就直接 ting()，而且 lostLine() 还在被每轮调用，
 *   于是"停住"是永久的 —— 车被带偏一次就再也起不来，这就是"卡死"。
 *   改成：超时后进入"停一下 → 倒退一段 → 再给一轮找线窗口"，
 *   试满 LOST_RETRY 轮仍找不到，才真正停车放弃。
 *   倒退距离 ≈ 21cm/s × 450ms ≈ 9.5cm，足够让探头重新压回线上。 */
#define LOST_STOP_MS    250UL   // 每轮先停这么久（稳住，避免边冲边找）
#define LOST_REV_MS     450UL   // 然后倒退这么久
#define LOST_RETRY        4     // 最多试几轮

// distance 定义在第 0 节（判区要用，已提前声明）

void du()
{
    // ★ 改成先算一个"原始状态"nq：
    //   nq == 0 就是【丢线】（三个探头全白，或者只剩左右两个亮这种怪状态），
    //   以前这里直接 q = fq（而 fq 永远是 0），等于丢线就变 q=0 → 停车；
    //   现在如实把 0 交给 move_tai，由它决定"盲走找线"还是停。
    int nq;
    if      (b == 1 && a == 0 && c == 0)  nq = 10;
    else if (a == 0 && b == 0 && c == 0)  nq = 0;      // 全白 → 丢线
    else if (a == 1 && b == 0 && c == 0)  nq = 100;
    else if (a == 0 && b == 0 && c == 1)  nq = 1;
    else if (a == 0 && b == 1 && c == 1)  nq = 1;
    else if (a == 1 && b == 1 && c == 0)  nq = 110;
    else                                   nq = 0;      // 101 / 111 之类 → 也当丢线

    if(nq != 0) fq = nq;     // ★ 记住最后一次"看得见线"的状态，丢线续走要用它
    q = nq;
}

/* --------------------------------------------------------------------------
 * 丢线兜底动作（直角拐弯的关键）
 *   hard = false：刚丢线的头 LOST_GRACE_MS，先沿用"最后那个状态"的动作，
 *                 免得一次采样抖动就把车带偏；
 *   hard = true ：确认丢线了 → 按【最后看到线的方向】硬找线：
 *                 最后是 100 / 110（线在左边）→ 往左拐；最后是 1（线在右边）→ 往右拐。
 *   fq 就是"最后一次看得见线"的状态。
 * ------------------------------------------------------------------------ */
void lostMotion(bool hard)
{
    if(fq == 100)              { L();   return; }   // 最后只剩左探头见线 → 往左拐
    if(fq == 1)                { R();   return; }   // 最后只剩右探头见线 → 往右拐
    if(fq == 110)                                     // 线偏左、车头刚过拐点
    {
        if(hard)               { L();   return; }
        else                   { zhi(); return; }
    }
    qian();                                          // 本来是直行 → 继续往前找
}

void lostLine()
{
    unsigned long el = millis() - lostStart;

    /* ① 还在找线窗口内：按最后看到线的方向盲走 */
    if(el < LOST_MS) { lostMotion(el >= LOST_GRACE_MS); return; }

    /* ② 超时 → 有界恢复：停一下 → 倒退 → 再找
     *    用纯时间算术推进，不需要额外状态变量；
     *    一旦重新压到线（q != 0），move_tai() 会把 lostStart 清零，本流程自动复位。 */
    unsigned long t   = el - LOST_MS;
    unsigned long cyc = LOST_STOP_MS + LOST_REV_MS;
    uint8_t       k   = (uint8_t)(t / cyc);

    if(k >= LOST_RETRY) { ting(); return; }     // 试满仍找不到 → 真停，不乱窜

    unsigned long ph = t - (unsigned long)k * cyc;
    if(ph < LOST_STOP_MS) { ting(); return; }   // 先停稳
    hou();                                      // 再倒退把探头带回线上
}

/* ========================= 6.5 直角弯（5cm 急停后触发） =========================
 *  几何：沿半径 CORNER_R_CM 的圆弧转过 90°。
 *     路径中心线弧长 = R × π/2 = 15 × 1.5708 ≈ 23.6cm
 *     内外轮速度比    = (R ± 轮距/2) / R
 *  时间是开环的：到点即认为转够 90°。现场角度偏就先量 CORNER_TRACK_CM
 *  （左右轮中心距），它比 R 更影响差速比。
 *
 *  整个流程（见 move_tai 的 ST_CORNER）：
 *    CV_ARC   走弧；弧中一旦看到黑线（q != 0）立刻转 CV_FOLLOW
 *    CV_FOLLOW 巡线 LINE_FOLLOW_MS，然后转 CV_BACK
 *    CV_BACK  后退到丢线（q == 0）停住，然后 kai()
 *    CV_KAI   等 kai() 的四步走完 → 回 ST_IDLE、解除静默
 * ============================================================================== */
#define CORNER_R_CM      15.0   // 圆弧半径（路径中心线）
#define CORNER_TRACK_CM  12.0   // ★ 左右轮中心距，必须按实车实测
#define CORNER_LEFT       1     // 1 = 左转（逆时针）；0 = 右转
#define CORNER_REQ_MS   500UL   // 急停解除后多久内必须开始拐弯，超时作废
#define ARC_LOST_N        5     // 连续 N 拍丢线才算"离开原线"
#define ARC_SEEN_N        5     // 连续 N 拍见线才算"遇到新线"
#define REV_MIN_MS      250UL   // 后退至少这么久，才允许用"丢线"退出
#define LINE_FOLLOW_MS 1000UL   // 弧线中看到黑线后，巡线多久
#define REV_LOST_MS    3000UL   // 后退找丢线的兜底上限

#define CV_ARC     0
#define CV_FOLLOW  1
#define CV_BACK    2
#define CV_KAI     3

static uint8_t       cornerStep  = CV_ARC;
static bool          arcLost     = false;   // 弧线中是否已离开过原线
static unsigned long cornerStart = 0, followStart = 0, backStart = 0;
static uint16_t      cornerArcMs = 0;      // 弧线超时兜底
static int16_t       cornerNeedEdges = 36; // 转 90 度需要的两轮边沿差
static long          cornerDiff0 = 0;      // 进弯瞬间的 (encCntR-encCntL)
static uint8_t       arcCnt = 0, arcSeenCnt = 0;
static double        cornerVin = 0, cornerVout = 0;

static void cornerSetup()
{
    double v = baseSpeed;
    if(v < 12.0) v = 12.0;                            // 低于死区速度就没有转向能力

    double k = (CORNER_TRACK_CM * 0.5) / CORNER_R_CM;
    cornerVin  = v * (1.0 - k);
    cornerVout = v * (1.0 + k);
    if(cornerVin < 12.0) cornerVin = 12.0;             // 内轮必须高过死区(约 11cm/s)才转得动

    /* 用【差速里程】而不是时间去收口：
     *   转 90 度需要的两轮里程差 = 轮距 x pi/2，与车速、电池电压、打滑都无关。
     *   12cm x 1.5708 = 18.85cm，除以 0.5236cm/边沿 = 36 个边沿。
     *   原来按时间(785ms)收口，而弧线是从 5cm 急停后的 v=0 起步，速度环主极点
     *   约 0.8s，整段都在加速，实测只转 55~68 度，差 30 度以上。 */
    cornerNeedEdges = (int16_t)((CORNER_TRACK_CM * 1.5707963) / CM_PER_EDGE + 0.5);

    /* 时间只作超时兜底：按标称速度算出的时间 x2.5 */
    cornerArcMs = (uint16_t)((CORNER_R_CM * 1.5707963) / v * 1000.0 * 2.5);
}

static void cornerArc()
{
    straightMode = false;      // ★ 弧线是刻意转向，绝不能被航向环当成"直行偏差"累积
#if CORNER_LEFT
    tgtL = cornerVin;   tgtR = cornerVout;            // 左转：右轮在外圈，走快
#else
    tgtL = cornerVout;  tgtR = cornerVin;             // 右转：左轮在外圈，走快
#endif
}

/* 拐弯中那 1 秒用的简易循迹：不带对正锁、不带丢线超时，只按 q 给动作 */
static void lineFollowOnce()
{
    switch(q)
    {
        case 10:   qian();  break;
        case 100:  L();     break;
        case 1:    R();     break;
        case 110:  zhi();   break;
        case 0:    qian();  break;      // 短暂丢线先直行找线
        default:   ting();  break;
    }
}

/* ---- 投放动作下发 + 离开判定（三个 DROP_MODE 分支共用）----------------
 *  match = true  → 确认是目标位：下发 fan()+gui()（只发一次），并等动作
 *                  全部走完才切 ST_LEAVE；走完才算 dropDone。
 *  match = false → 不是目标位：立刻切 ST_LEAVE，继续找下一个盒子。
 *  ★ 原来这段在两个 #if 分支里各抄了一遍，改一处极易漏另一处。
 * -------------------------------------------------------------------- */
static void dropAct(bool match)
{
    if(match && !dropSent)
    {
        dropSent = true;
        fan();
        gui();
    }

    /* 不是目标位：立刻走，不用等任何动作 */
    if(!match)
    {
        leaveStart = millis();
        stage      = ST_LEAVE;
        return;
    }

    /* ★ 是目标位：必须等 fan() + gui() 的最后一个舵机都到位。
     *   注意 actDone() 里有队列计数 —— fan 跑完时 gui 还排着，它仍然返回 false。 */
    if(!actDone()) { ting(); return; }

    /* 再停 DROP_TAIL_MS：舵机到位 ≠ 物块已经落进盒子（见常量处的说明）。
     * dropTailUntil 从"第一次发现动作全完成"开始计，只设一次。 */
    if(dropTailUntil == 0) dropTailUntil = millis() + DROP_TAIL_MS;
    if(millis() < dropTailUntil) { ting(); return; }

    dropDone      = true;               // 动作走完 + 停留够了，才算投放完成
    dropTailUntil = 0;
    leaveStart    = millis();
    stage         = ST_LEAVE;
}

void move_tai()
{
    /* ★★★ 超声波启用闸：qu() 走完 + ULTRA_ARM_DELAY_MS 之后才置 ultraArmed。
     *
     *   起点区域常有相邻物块、器材腿、墙面等近距离目标。超声波一启用就可能被它们
     *   判成到站(STOP_CM)或急停(EMERG_CM)，车起步就停住，一步都走不出去。
     *
     *   ★ 为什么必须放在【所有阶段判断之前】：
     *     ST_PICK_FWD 只持续 START_FWD_MS(1.5s)，而延时是 5s。若把这段逻辑
     *     放进 ST_PICK_FWD 分支，条件永远走不完 —— 超声波会【永不启用】，
     *     车一路冲过所有盒子，比不判停还糟。
     *
     *   原来的做法是"4s 起步锁定"：超声波照常采样、照常计票，只是把票丢掉。
     *   效果与"不开超声波"完全相同，却多出一条历史上反复写错的屏蔽分支
     *   （屏蔽了到站 / 没屏蔽到站，两次都踩过）。现在改为根本不开超声波，
     *   启用之后到站与急停全程生效，代码里不再存在任何屏蔽窗口。
     *
     *   ★ qu() 若因舵机卡死一直不返回 actDone()，超声波就不会启用，车会一路
     *     巡线到底。这是不误停的安全失败方向，40s TASK_TIMEOUT_MS 会兜底收掉。 */
    if(!ultraArmed)
    {
        if(ultraStartAt == 0)
        {
            if(actDone()) ultraStartAt = millis() + ULTRA_ARM_DELAY_MS;
        }
        else if(millis() >= ultraStartAt)
        {
            ultraArmed  = true;        // 到点了，正式启用超声波
            ultraStartAt = 0;          // 置 0 表示"已启用"，不再重复置位
        }
    }

    // ★ 每轮先把两个计时器刷新一遍（放在最前面，保证不会留下过期的旧值）
    //    onlineStart：q==10 已经连续保持多久了
    //    lostStart  ：q==0（丢线）已经持续多久了
    if(q == 10) { if(onlineStart == 0) onlineStart = millis(); }
    else          onlineStart = 0;

    if(q == 0)  { if(lostStart == 0)   lostStart   = millis(); }
    else          lostStart   = 0;

    // ★★★ backOnline：中探头"稳稳"回到线上（连续 BACK_ONLINE_MS 都是 10）
    //   它才是对正/拐弯的正确退出条件；单独一次 10 不算 —— 直角拐点的病根就在这
    bool backOnline = (q == 10 && onlineStart != 0 &&
                       (millis() - onlineStart) >= BACK_ONLINE_MS);

    // ★★★ 第 0 道闸：任务阶段，优先级最高，压过循迹/超声波
    //     空闲：原地等 物块相机识别搬取物
    if(stage == ST_IDLE)
    {
        ting();
        return;
    }
    /* ---- 取件 → 出发段 → 巡线。超声波在 qu() 走完后再等 5s 才启用 ---- */
    if(stage == ST_PICK_FWD)
    {
        if(millis() < fwdUntil) { qian(); return; }

        // ★ 到点就地切入巡线，不留给 loop() 收尾：
        //   否则这一拍会落到底下的循迹逻辑（stage 还是 PICK_FWD），
        //   出发段末尾多一次方向突变。
        stage = ST_LINE;

        /* ★★★ 必须清零这两个计时器：
         *   它们在本函数开头（早于所有阶段判断）就一直在累计 ——
         *   空闲等识别 + 出发段 1.5s 加起来早就超过 LOST_MS(600ms)。
         *   若不清零，进入巡线的第一轮 lostLine() 就判"丢线超时"直接 ting()，
         *   表现就是"取完件不巡线、车立刻停住"。
         *   清零后由下一轮开头的维护代码重新起算，给足 600ms 找线窗口。 */
        lostStart   = 0;
        onlineStart = 0;

        qian();
        return;
    }
/* ---- 到站（22cm 停车后）：读放置区相机 → 投放 或 离开继续找 ---- */
    if(stage == ST_DROP)
    {
        ting();                                   // 停稳车身，别晃着识别

        if(millis() - dropStart < SETTLE_MS) return;

#if DROP_MODE == 0
        /* ★ 到站锁存：本次停车第一次读到的合法区域号就固定下来，
         *   之后相机即使因为标志暂时移出视野而补发 <00>，也不再把判断依据抹掉。
         *   原来没有这一步时，只要停车那一刻标志恰好不在视野里（车身晃动、
         *   安装角度偏），visSite 会被清成 0，于是明明看到了却仍然不投放。 */
        if(visSite >= 1 && dropCode == 0) dropCode = visSite;

        if(dropCode >= 1)                         // 本次到站已拿到区域号
        {
            dropAct(dropCode == f);
            return;
        }

        /* 相机还没报：再等 DROP_WAIT_MS；等不到就当作"不是目标位"离开 */
        if(millis() - dropStart > SETTLE_MS + DROP_WAIT_MS)
        {
            leaveStart = millis();
            stage      = ST_LEAVE;
        }
        return;
#else
        /* ---- 非 0 模式：不依赖放置区相机 ---- */
        dropAct((DROP_MODE == 2) ? true : (stopCnt == f));
        return;
#endif
    }
    /* ---- 离开当前盒：前进，直到测距 > LEAVE_CLEAR_CM（走出盒子） ----
     *  ★ 用"测距变远"而不是固定时长：盒宽与车速都会变，固定时长要么
     *    没走出去（刚回巡线又停）、要么一下跨过两个盒（漏掉目标）。 */
    if(stage == ST_LEAVE)
    {
        /* ★★★ 最后一道防线：动作没走完，绝不开走。
         *   即使上游哪个分支（任务超时、相机等待超时、dropAct 的异常路径）提前把
         *   stage 置成了 ST_LEAVE，这里也会先停住，等 fan()/gui() 全部到位再前进。
         *   否则就是"一边投放一边开走"，落点必偏 —— 这正是 fan 看起来没执行完
         *   就进入下一阶段的原因。
         *   leaveStart 每轮刷新，保证离开计时从"动作真的走完"那一刻才开始。 */
        if(!actDone())
        {
            ting();
            leaveStart = millis();
            return;
        }

        unsigned long el = millis() - leaveStart;

        /* distance == 999 表示【无回波】，不是"很远"。
         * 原来直接比 distance > 27，声波一丢就判"已离开"，车还贴着盒子就回巡线，
         * 随即又被 22cm 判停 —— 原地抽搐。 */
        bool far = (distance > 0.0f && distance < 900.0f && distance > LEAVE_CLEAR_CM);

        if(el >= LEAVE_MIN_MS && (far || el >= LEAVE_MAX_MS))
        {
            stopLocked = false;
            if(dropDone)
            {
                stage      = ST_IDLE;      // 已投放 → 任务完成
                ultraArmed = false;
                taskRearm  = true;         // 强制重新武装取件（见 taskRearm 声明处）
            }
            else
            {
                /* ★ 没投成 → 回巡线继续找下一个盒。这两个计时器在
                 *   ST_DROP / ST_LEAVE 期间一直在累计（车是停的、q 可能是 0），
                 *   不清零的话一进 ST_LINE 就被判"丢线已超时 600ms"，
                 *   直接跳进倒退恢复流程，而不是先正常循迹。 */
                lostStart   = 0;
                onlineStart = 0;
                stage       = ST_LINE;
            }
            return;
        }
        qian();
        return;
    }

    /* ---- 5cm 急停解除 → 起直角弯（此时才允许进入，避免和急停抢电机） ---- */
    if(cornerReq && !stopLocked && stage == ST_LINE &&
       millis() - cornerReqMs < CORNER_REQ_MS)
    {
        cornerReq   = false;
        quiet       = true;                  // 期间忽略 1,x，也不输出串口
        taskStartMs = millis();              // 重开 40s 兜底，别把整段动作掐断
        stage       = ST_CORNER;
        cornerStep  = CV_ARC;
        arcLost     = false;
        arcCnt      = 0;
        arcSeenCnt  = 0;
        cornerStart = millis();
        {
            uint8_t sreg = SREG;                     // 记下进弯时的里程差基准
            cli();
            cornerDiff0 = (long)encCntR - (long)encCntL;
            SREG = sreg;
        }
        zhiLocked   = false;
        cornerSetup();
        cornerArc();
        return;
    }

    /* ---- ST_CORNER：弧线 → 可选巡线/后退 → kai() ---- */
    if(stage == ST_CORNER)
    {
        switch(cornerStep)
        {
            case CV_ARC:
            {
                /* 第一步：里程收口 —— 两轮差速里程够了就是转够 90 度了 */
                long d;
                uint8_t sreg = SREG;
                cli();
                d = (long)encCntR - (long)encCntL;
                SREG = sreg;
                d -= cornerDiff0;

#if CORNER_LEFT
                bool arcDone = (d >=  cornerNeedEdges);
#else
                bool arcDone = (d <= -cornerNeedEdges);
#endif
                /* 第二步：黑线识别要连续若干拍，单帧抖动不算 */
                if(q == 0) { if(arcCnt < 255) arcCnt++; }
                else       { arcCnt = 0; }
                if(arcCnt >= ARC_LOST_N) arcLost = true;

                if(arcLost)
                {
                    if(q != 0) { if(arcSeenCnt < 255) arcSeenCnt++; }
                    else       { arcSeenCnt = 0; }
                }

                if(arcLost && arcSeenCnt >= ARC_SEEN_N)
                {
                    cornerStep  = CV_FOLLOW;
                    followStart = millis();
                    lineFollowOnce();
                    return;
                }
                if(arcDone || millis() - cornerStart >= cornerArcMs)
                {
                    cornerStep = CV_KAI;         // 转够 / 超时 -> 直接 kai()
                    kai();
                    ting();
                    return;
                }
                cornerArc();
                return;
            }

            case CV_FOLLOW:
                if(millis() - followStart >= LINE_FOLLOW_MS)
                {
                    cornerStep = CV_BACK;        // 满 1s → 后退找丢线
                    backStart  = millis();
                    hou();
                    return;
                }
                lineFollowOnce();
                return;

            case CV_BACK:
                /* 先强制后退 REV_MIN_MS：进 CV_BACK 时 q 很可能已经是 0
                 * （巡线末拍丢线），直接判 q==0 会一步都不退。 */
                if(millis() - backStart < REV_MIN_MS) { hou(); return; }

                if(q == 0 || millis() - backStart >= REV_LOST_MS)
                {
                    ting();                      // 丢线 → 停住
                    cornerStep = CV_KAI;
                    kai();
                    return;
                }
                hou();
                return;

            case CV_KAI:
                if(actDone())                    // kai() 四步走完
                {
                    quiet      = false;
                    cornerStep = CV_ARC;

                    if(dropDone)
                    {
                        /* 任务已完成（先投放、后拐弯）→ 回空闲并重新武装 */
                        stage     = ST_IDLE;
                        taskRearm = true;
                    }
                    else
                    {
                        /* ★★★ 还没投放就拐了弯 —— 这条路径以前无条件回 ST_IDLE：
                         *   车手里还夹着物块，回空闲等于"放弃这个任务"；
                         *   而且下一个物块放上来还会再调 qu() 抓一次，
                         *   轻则空抓、重则顶住机构。
                         *   正确做法是回巡线把任务走完。
                         *
                         *   一次性把这些状态复位（缺一项都会出怪现象）：
                         *   · lostStart/onlineStart：ST_CORNER 期间车是停的或大角度转向，
                         *     这两个计时器一直在累计，不清的话一进 ST_LINE
                         *     立刻被判"丢线已超时 600ms"，直接进倒退恢复。
                         *   · headE/headCorr：拐弯前累积的里程差与回巡线无关，
                         *     不清会带着满额差速起步，刚回来就被拽偏。
                         *   · taskStartMs：40s 兜底是从任务开始算的，拐弯常常发生在
                         *     任务中后段，不重开的话回到巡线马上就超时。
                         *   · cornerReq/stopLocked/ultraArmed：确保以一个干净状态进巡线。 */
                        lostStart   = 0;
                        onlineStart = 0;
                        headE       = 0;
                        headCorr    = 0;
                        cornerReq   = false;
                        stopLocked  = false;
                        /* ★ 不动 ultraArmed：超声波启用与否只由 move_tai() 顶部的
                         *   "启用闸"决定（qu 走完 + 5s）。这里若强行置 true，
                         *   一次直角弯就会把 5s 延时绕过，起点附近的物块又会被误判。 */
                        taskStartMs = millis();
                        stage       = ST_LINE;
                    }
                }
                return;

            default:
                cornerStep = CV_ARC;
                return;
        }
    }

    // ★★★ 超声波停车锁：优先级高于对正和循迹
    if(stopLocked)
    {
        ting();
        return;
    }

    if(zhiLocked)
    {
        // ★★★ 就是这里改的（直角拐不过去的病根）
        //   原来：if(q == 10) 就立刻解锁去 qian()。
        //         直角拐点上探头必然有一瞬间只看到 10，对正当场被打断 → 车直冲过拐点。
        //   现在：必须"连续稳定回到线上"（backOnline）才放行，中间的单次抖动不算。
        if(backOnline)
        {
            zhiLocked = false;
            qian();
            return;
        }

        bool worse    = (q == 100 || q == 1 || q == 0);
        bool overtime = (millis() - zhiStart > ZHI_TIMEOUT_MS);

        /* ★ 顺序很重要：超时若只清标志就往下走，switch 里的 case 110
         *   会当轮立刻重新上锁并刷新 zhiStart → 400ms 超时等于没写。
         *   所以超时这一拍必须【强制直行并 return】。 */
        if(overtime) { zhiLocked = false; qian(); return; }
        if(worse)    { zhiLocked = false; }        // 交给 switch 的 100/1/0 分支
        else         { zhi(); return; }
    }

    switch(q)
    {
        case 10:   qian();      break;
        case 100:  L();         break;
        case 1:    R();         break;
        // ★ 进入对正时【上锁】——这是直角拐弯修复真正生效的开关。
        //   锁定后由上面的 if(zhiLocked) 分支接管：必须"连续稳回线上"
        //   (backOnline) 才放行，单次 q==10 的抖动不算数。
        case 110:  zhiStart  = millis();
                   zhiLocked = true;
                   zhi();
                   break;
        // ★★★ 0 = 丢线：不再直接 ting() 停死，改成"盲走找线"
        case 0:    lostLine();  break;
        default:   ting();      break;
    }
}

/* ========================= 7. 超声波测距（pulseIn 阻塞式） =========================
 *  接法与示例一致：TRIG=D15(A1) 输出，ECHO=D14(A0) 输入。
 *
 *  为什么改成 pulseIn：原来用 PCINT 边沿中断捕获回波宽度，逻辑复杂、
 *  且依赖中断时序（详见旧注释里 A1 被当 ECHOPIN 时 10us 触发脉冲被
 *  误当回波那段）。pulseIn 由 Arduino 内核直接量脉宽，行为可预期。
 *
 *  阻塞代价：一次 pulseIn = 回波宽度。24cm ≈ 1.4ms；无回波时阻塞到
 *  ECHO_TIMEOUT_US(30ms)。30ms @16MHz 约 4800 个时钟，相对 20ms 速度
 *  环节拍偏大，所以把采样周期放大到 60ms —— 无回波时平均占用降到 1/2，
 *  且 distSeq 的节拍变慢，STOP() 的"连续 N 个样本"自然对应更长的时间窗。
 */
void updateUltrasonic()
{
    static unsigned long lastTrig = 0;
    static float         prevDist = 999.0f;   // 上一帧距离，用于突变检测
    unsigned long now = micros();

    if(now - lastTrig < ULTRA_INTERVAL_US) return;   // 未到采样周期
    lastTrig = now;

    /* ---- 10us 触发脉冲 ---- */
    digitalWrite(TRIGPIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGPIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGPIN, LOW);

    /* ---- 量回波宽度 ---- */
    unsigned long w = pulseIn(ECHOPIN, HIGH, ECHO_TIMEOUT_US);

    /* 有效窗口：150us≈2.6cm 是常见模块的最小量程，下限滤掉毛刺；
     * 上限由 pulseIn 的超时兜住（超时返回 0）。 */
    float d = (w >= 150) ? (w / 58.0f) : 999.0f;

    /* ★★★ 测距突变检测：相邻两帧距离跳变超过 JUMP_CM 视为"离开了当前盒子"。
     *
     *   场景：车贴着盒子时测距稳定在 20~24cm，一旦驶离，盒壁侧面瞬间退到
     *   几十厘米甚至直接无回波(999)。这个"台阶"比"缓慢变远"可靠得多 ——
     *   盒子宽度、盒间距、车速都会变，用固定距离阈值判断离开迟早出错。
     *
     *   只在两次都是有效测距、或"有效→无回波"时判定；
     *   "无回波→有效"不算突变（那是重新看到东西，不是离开）。
     */
    bool wasValid = (prevDist < 900.0f);
    bool nowValid = (d < 900.0f);
    if(wasValid && nowValid)
    {
        float diff = prevDist - d;
        if(diff < 0) diff = -diff;
        if(diff >= JUMP_CM) leftBox = true;
    }
    else if(wasValid && !nowValid)
    {
        leftBox = true;                    // 有回波 → 完全无回波 = 一定离开了
    }

    prevDist = d;
    distance = d;
    distSeq++;
}

/* ========================= 7.5 超声波到站判定 =========================
 *  测距 ≤ STOP_CM(22cm) 连续 STOP_CONFIRM(3) 个样本 → 认定到达投放点：
 *    停车 + 进入 ST_DROP，由 move_tai() 在那里读放置区相机，
 *    决定"投放"还是"离开继续找下一个盒"。
 *
 *  ★ 只在 ST_LINE 阶段生效：进入 ST_DROP 后由该阶段自己 ting()，
 *    不会重复触发，也就不需要额外的解锁条件。
 *  ★ 为什么必须"连续 N 个样本"：单帧多径反射、有人走过都会瞬间报 22cm
 *    以内，单帧就停会误判。3 个样本 ≈ 40ms 窗口（超声 20ms 一个样本）。
 * ========================================================== */
void STOP()
{
    static uint8_t  cnt = 0, emergCnt = 0;
    static uint16_t lastSeq = 0;
    static unsigned long lockAt = 0;

    /* ST_CORNER 期间不参与判停：这一段是开环动作，靠自己的时序收口 */
    if(!ultraArmed || stage == ST_IDLE || stage == ST_LEAVE || stage == ST_CORNER)
    { cnt = emergCnt = 0; return; }

    /* ★ 全流程只有这一道闸：ultraArmed。
     *   它在 qu() 走完 + ULTRA_ARM_DELAY_MS 之后才置 true（见 ST_PICK_FWD）。
     *   启用之后到站(STOP_CM)与急停(EMERG_CM)【全程生效】，不再有任何屏蔽窗口。
     *
     *   历史遗留（已删除，保留记录以免重犯）：
     *     · 4s 起步锁定 startLockUntil —— 超声波照常采样计票、只是把票丢掉，
     *       效果和"不开超声波"一样，却多一条易错的屏蔽分支；
     *     · 脱困窗口 bypassUntil / EMERG_BYPASS_MS(1.5s) —— 急停解锁后屏蔽判停，
     *       会让车冲过真正的盒子，历史上被误当"停车失效"的根因之一。 */
    if(distSeq != lastSeq)
    {
        lastSeq = distSeq;

        if(distance > 0.0f && distance <= EMERG_CM)
        {
            if(emergCnt < EMERG_CONFIRM) emergCnt++;
        }
        else emergCnt = 0;

        /* ★★★ 到站计票必须排除急停区（这是"5cm 急停后不拐弯"的第一个根因）：
         *   如果只判 distance <= STOP_CM，而 4~5cm 的急停样本同样满足它，
         *   于是车贴着墙角停住时 cnt 已经攒够。等急停解除、距离恢复到
         *   6~26cm，这里立刻 `cnt >= STOP_CONFIRM` → 误进 ST_DROP，
         *   而 ST_CORNER 的进入条件要求 stage == ST_LINE，直角弯就永远起不来。
         *   把急停区剔除后，cnt 只在 5~26cm 这段"真的贴着盒子"的区间计票。 */
        if(stage == ST_LINE)
        {
            if(distance > EMERG_CM && distance <= STOP_CM)
            {
                if(cnt < STOP_CONFIRM) cnt++;          // 稳稳贴着盒子 → 计票
            }
            else if(distance <= EMERG_CM || distance > STOP_CM + STOP_HYST_CM)
            {
                cnt = 0;                               // 明确离开（≤5 或 >30）才清票
            }
            /* 落在 (STOP_CM, STOP_CM+STOP_HYST_CM] 的滞回带：不清票，
             * 容忍超声波在判据边界附近的抖动，别让单帧跳变把票清零。 */
        }
        else cnt = 0;
    }

    /* ---- ① 过近急停（5cm）：优先级最高，只停车、不进投放流程 ---- */
    if(emergCnt >= EMERG_CONFIRM)
    {
        if(!stopLocked)
        {
            stopLocked = true;
            lockAt     = millis();
            cornerReq   = true;     // ★ 记下这次 5cm 急停，解除后起直角弯
            cornerReqMs = millis(); //   带上时刻，超时未消费就作废
        }
        ting();

        /* 车贴住箱壁/墙角时距离永远是 4cm，只等"距离恢复"会永久卡死。
         * 锁满 3 秒 → 强制放行，让它真正开出去。
         * ★ 这里不再设脱困屏蔽窗口：以前放行后会屏蔽判停 1.5s，
         *   而这 1.5s 内车已走 45cm，足以让下一次计票从 0 重新开始 ——
         *   没有窗口也就没有"冲过真正的盒子"的风险。 */
        if(millis() - lockAt > EMERG_UNLOCK_MS)
        {
            stopLocked  = false;
            emergCnt    = 0;
            /* ★★★ 直角弯窗口必须从"解锁那一刻"重新计（这是"5cm 急停后不拐弯"
             *   的第二个根因）：cornerReqMs 记的是急停发生的时刻，而车可能被
             *   锁满 3s 才放行，CORNER_REQ_MS 只有 500ms —— 解锁时窗口早就
             *   过期，ST_CORNER 永远进不去，cornerReq 就一直挂着。
             *   解锁时刷新时刻，等于"放行后 500ms 内必须开始拐"。 */
            if(cornerReq) cornerReqMs = millis();
        }
        return;
    }

    /* ---- 障碍消失 → 立即解锁 ----
     *  ★ 原来只有"锁满 3 秒强制放行"这一条出路：车停下后距离一旦回到 5cm 以上，
     *    emergCnt 被清零、进不了上面的分支，stopLocked 就永远为 true，
     *    车会一直停到 40s 任务超时才动。这里补上"距离恢复即解锁"。 */
    if(stage == ST_LINE && stopLocked)
    {
        stopLocked = false;
        /* 同上一处：锁了多久都算数，窗口从解锁时刻重新计 */
        if(cornerReq) cornerReqMs = millis();
    }

    /* ---- ② 到站（22cm）：只在巡线阶段，停车交给 ST_DROP 读相机 ---- */
    if(cnt >= STOP_CONFIRM)
    {
        cnt        = 0;
        stopLocked = true;
        lockAt     = millis();            // ★ 必须同步：否则后面若撞进 5cm 分支，
                                          //   millis()-lockAt(0) 会瞬间超过 3s，
                                          //   被当成"已锁满"立刻强制放行
        dropStart  = millis();
        dropSent   = false;               // 新的一次到站，重新允许下发投放动作
        dropCode   = 0;                   // 新的一次到站，区域号锁存清零
        if(stopCnt < 255) stopCnt++;      // 本任务第几次到站（DROP_MODE=1 用它判"第几个盒子"）
        stage      = ST_DROP;

        /* ★★★ 停车必须清掉航向积分 —— 这是"投放完一离开盒子就歪、然后卡死"的根因：
         *   headE 是从上一个任务起点开始一路累加的两轮里程差，停车前它可能已经
         *   顶到 HEAD_SOFT_CM(±20cm)。车停稳后这个值毫无意义（车都没动），
         *   但 ST_LEAVE 里一调 qian()，headCorr = HEAD_KP×headE 立刻打到
         *   ±min(基速35%, HEAD_MAX)=±7 的差速（30↔37/23），车刚起步就被往一边拽，
         *   方向盘打死 → 冲出轨 → 丢线 600ms → ting() → 永久停车。 */
        headE    = 0;
        headCorr = 0;

        ting();                       // 立刻松引脚，不等 20ms 的 PID 节拍
    }
}

static void statusLed()
{
    static unsigned long last = 0;
    static unsigned long okMs = 0;          // 最近一次测到有效距离的时刻
    unsigned long now = millis();
    /* ★ 刷新周期必须远小于最窄的闪烁窗口：
     *   原来的 100ms 会把 80ms 的方波"采样"成不规则慢闪，根本数不出频率。
     *   改成 10ms 后，(now/80)&1 才是真正的 6.25Hz。 */
    if(now - last < 10) return;
    last = now;

    /* 超声波是否还活着：收到过 <900cm 的值就刷新 okMs。
     * 超过 3 秒没有有效值 → 认为没回波（接错线/没供电/模块坏）。 */
    if(distance > 0.0f && distance < 900.0f) okMs = now;
    bool ultraDead = (now - okMs > 3000);

    bool on;
    switch(stage)
    {
        /* ★ ST_IDLE 心跳：60ms 短闪 / 2s。
         *   原来这里全灭 —— 于是"没烧进去/程序跑飞"和"正常待机等物块"看起来
         *   一模一样，没法判断板子到底活没活。有了心跳，灯完全不亮就是固件没跑。 */
        case ST_IDLE:     on = (now % 2000UL) < 60UL; break;
        case ST_PICK_FWD: on = true;                  break;
        case ST_LINE:     /* 正常：1Hz 慢闪；超声波收不到回波：约 8Hz 快闪 */
                          on = ultraDead ? ((now / 60) & 1) : ((now / 500) & 1);
                          break;
        /* ★ ST_DROP 三态指示 —— 不用接串口就能判定"为什么没投放"：
         *   6Hz 快闪  visSite == 0       → D11 一个字都没收到
         *                                  查：K230 IO11→D11、共地、先给 Arduino 上电
         *   常亮      visSite ≠ f        → 收到了，但区域号与物块种类不符
         *                                  （相机看到的是别的标志 / f 识别错）
         *   2Hz 慢闪  visSite == f       → 已匹配，正在等 fan()+gui() 走完 */
        case ST_DROP:     if(visSite == 0)      on = (now / 80) & 1;
                          else if(visSite != f) on = true;
                          else                  on = (now / 250) & 1;
                          break;
        case ST_CORNER:   on = (now / 80) & 1;        break;   // 约 6Hz：直角弯
        case ST_LEAVE:    on = ((now / 150) & 3) < 2; break;
        default:          on = false;                 break;
    }
    digitalWrite(13, on);
}

/* ========================= 8. setup / loop ========================= */
void setup()
{
    /* 硬件串口 D0：两台相机共用（SITE_ON_HW_UART=1）
     *   上电默认 9600（待机听物块相机），进入任务后 pollVision() 会自动切 115200。
     *   若 SITE_ON_HW_UART=0，则这里只服务物块相机，放置相机走软串口。 */
    Serial.begin(VISION_BAUD);

    /* ★ 开机唤醒物块相机。
     *   等 300ms：K230 上电比 Arduino 慢得多，太早发的命令它会漏掉。
     *   注意这里用的是 9600 —— 与 setup 里的 Serial.begin 一致。 */
    delay(300);
    k230Send(CMD_WAKE_BLOCK);

#if !SITE_ON_HW_UART
    // 软串口接放置区相机：上拉保证空闲为高，边沿进对应的 PCINT 向量
    //   具体是哪一位/哪个向量，由上面的 SITE_RX_PIN 自动决定
    pinMode(SITE_RX_PIN, INPUT_PULLUP);
    RX_PCMSK |= _BV(RX_PCINT);
    PCICR    |= _BV(RX_PCIE);
#endif

    // 循迹传感器：左 A4 / 中 D3 / 右 A5
    //   D3 是数字口 3（PD3），A3 是模拟口（PC3，Arduino 编号 17），两者不同引脚，
    //   与编码器 A3 不冲突。D4 已留作舵机。
    pinMode(A4, INPUT);
    pinMode(3,  INPUT);
    pinMode(A5, INPUT);

    // 电机
    pinMode(5,  OUTPUT);
    pinMode(6,  OUTPUT);
    pinMode(9,  OUTPUT);
    pinMode(10, OUTPUT);

    // 超声波
    pinMode(TRIGPIN, OUTPUT);
    pinMode(ECHOPIN, INPUT_PULLUP);   // 上拉：模块没接时引脚不会浮空乱触发 ECHO_PCINT

    // ★ 编码器：A2/A3 走引脚变化中断
    //   A2 = PC2 = PCINT10，A3 = PC3 = PCINT11 → 同属 PCINT1 组
    //   必须用【PCMSK1 + PCIE1 + PCINT1_vect】。
    //   （PCMSK2 里的 PCINT16~23 对应的是 PD0~PD7，即 D0~D7，不是 A2/A3）
    //   A4/A5 = PC4/PC5 = PCINT12/13，虽同在 PCMSK1，但不在掩码里，
    //   循迹信号的变化不会触发本中断。
    pinMode(A2, INPUT_PULLUP);
    pinMode(A3, INPUT_PULLUP);
    delay(2);            // 等上拉稳定，再去读初始电平，免得第一个假跳变被计入
    echoRise = 0;
    pcLast  = PINC;
    // A2=PCINT10、A3=PCINT11，以及 ECHO_PCINT（跟随 ECHOPIN，见文件头部）
    PCMSK1 |= _BV(PCINT10) | _BV(PCINT11) | _BV(ECHO_PCINT);
    PCICR  |= _BV(PCIE1);

    pidL.setGains(kp, ki, kd);
    pidR.setGains(kp, ki, kd);
    motorLR(0, 0);

    servoInit();         // 4 路舵机：上电即到初始位 D4=90 D7=90 D8=270 D12=180

    pinMode(13, OUTPUT); // 板载 LED：阶段指示（排障用，可删）
}

void loop()
{
    // ① 读循迹传感器：中 D3 固定；左右按 TRACK_MIRROR 决定谁是 a 谁是 c
#if TRACK_MIRROR
    a = digitalRead(A5);
    b = digitalRead(3);
    c = digitalRead(A4);
#else
    a = digitalRead(A4);
    b = digitalRead(3);
    c = digitalRead(A5);
#endif

#if BENCH_LINE_ONLY
    /* ---- 隔离模式：只验证 传感器 + PID + 电机 ---- */
    du();
    if(q == 0) ting();        // 丢线就停，方便判断"到底有没有读到线"
    else       lineFollowOnce();
    updateSpeedPID();
    digitalWrite(13, q != 0); // LED = 任意探头见到黑线
    return;
#endif

    // ② 双 K230 视觉接收 + 任务调度
    pollVisionAndQueue();

    // ③ 超声波测距 + 过近急停保护（放在运动指令之前，同轮内先立锁再决策）
    updateUltrasonic();
    STOP();

    // ④ 状态机：只决定"目标速度"，不直接写 PWM
    du();
    move_tai();

    // ⑤ 机构动作序列推进（qu/fan/gui/kai，非阻塞）
    actRun();

    // ⑥ 速度环：测速 → PID → PWM（内部按 20ms 节拍执行）
    updateSpeedPID();

    // ⑦ 任务兜底超时：任一阶段卡住都能自己回空闲，不必断电重来
    /* ★ 超时也要先看动作是否在跑：fan/gui 还没走完就先让它们走完（最多多给 3s），
     *   否则超时那一刻直接把 stage 改成 ST_IDLE，车看起来"提前进入了下一阶段"，
     *   而舵机还在原地慢慢转。
     *   但动作真的卡死时不能无限等 —— 3s 后照样强制回 IDLE。 */
    if(stage != ST_IDLE && millis() - taskStartMs > TASK_TIMEOUT_MS
       && (actDone() || millis() - taskStartMs > TASK_TIMEOUT_MS + 3000UL))
    {
        ting();
        stage      = ST_IDLE;
        ultraArmed = false;
        stopLocked = false;
        zhiLocked  = false;
        cornerReq  = false;
        quiet      = false;

        /* ★ 故意【不】清 visTask：
         *   清了会让 taskArm 立刻复位，而相机 100ms 后又报同一个物体 →
         *   马上重起一个一模一样的任务，一次卡死就变成无限循环跑车。
         *   保留 visTask 时 taskArm 仍是 false，必须等"物体被拿走"（相机发 0,0）
         *   才解锁重试，符合"同一物体只取一次"的语义。 */
    }

    // ⑧ 阶段指示灯（排障用，可删）
    statusLed();
}
