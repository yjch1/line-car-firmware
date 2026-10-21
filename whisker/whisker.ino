/* ============================================================================
 *  循迹搬运小车 · 双轮编码器速度环 PID
 *  ---------------------------------------------------------------------------
 *  接线
 *    循迹传感器 : A4 / 中 D3 / A5（左右是否对调由 TRACK_MIRROR 决定）
 *    左轮驱动   : Motor-1 → 正转 D6    反转 D5
 *    右轮驱动   : Motor-0 → 正转 D10   反转 D9
 *    编码器     : 左 A3(PCINT11)   右 A2(PCINT10)   ← PCMSK1 / PCINT1_vect
 *    触须       : DATA A0（碰到物体 = 低电平，松开 = 高）  ← 机器谱示例接法，第4脚悬空
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
 *    PCINT1  仅编码器 A2/A3（触须是普通数字输入，不占中断）
 *    两个 ISR 的向量号是 4 / 8，互不冲突；COMPA(7) 留给 Tone.cpp。
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
 *         ★ 每 400ms 心跳重发一次，不是"变化才发"
 *         → 放置区域号 visSite；2.5s 无合法帧自动清零
 *  ---------------------------------------------------------------------------
 *  任务流程
 *    等 物块相机识别 → qu() 原地取件（★ 取件走完之前车不动）
 *      → qu 完成时刻 = 全任务时间基准 quDoneMs
 *      → 从 quDoneMs 起直行 START_FWD_MS(1.5s) → 巡线
 *      → 从 quDoneMs 起 WHISKER_ARM_DELAY_MS(7.5s) 后【触须】上岗
 *        ⚠ 7.5s @30cm/s ≈ 225cm：这段距离内的盒子一律不判停。
 *          起点到首个投放盒若小于此值，必须下调 WHISKER_ARM_DELAY_MS。
 *      → 巡线中触须被盒壁压住（连续 2 个样本）→ 停车 → 读放置区相机区域号
 *        → 区域号 == f → fan()+gui() 投放 → 巡线离开（触须松开）→ 回空闲
 *        → 区域号 != f → 前进走出当前盒 → 继续找下一个（不回空闲）
 *  ---------------------------------------------------------------------------
 *  ★ 节奏控制（防抢跑）
 *      ① 机构动作每一步到位后停 ACT_STEP_HOLD_MS(400ms) 再下发下一条；
 *      ② 两个任务阶段之间停 STAGE_GAP_MS(1s) 才进下一阶段。
 *      两者都用 millis() 计时、不写 delay()：期间速度环/触须/循迹照常跑，
 *      车不会带着最后一次 PWM 盲冲。把常量改成 0 即可关闭。
 *  ---------------------------------------------------------------------------
 *  优先级（move_tai 内，从上到下）
 *    阶段闸门 > 触须到站停车锁 > PD 巡线（含急弯升级 pivot） > 丢线盲走
 *    ★ 拐弯保命规则：
 *      1) 退出对正必须"连续"稳回线上（BACK_ONLINE_MS），单次 q==10 不算
 *      2) 三个探头同时丢线（q==0）不停死，按最后看到的方向盲拐，最多 LOST_MS
 *      3) 限幅/去抖都不够时，先调 BACK_ONLINE_MS（60~150）试
 *  ---------------------------------------------------------------------------
 *  （历史说明：ST_CORNER 直角弯旁路已随 5cm 超声波急停一起移除）
 *    15cm 半径圆弧转 90°
 *      → 弧中若"先丢线、再见线"：巡线 1s → 后退到丢线停住
 *  ---------------------------------------------------------------------------
 *  机构动作（qu / fan / gui / kai）
 *    表驱动的非阻塞序列，前一路舵机到位、并停够 400ms，才下发下一条；
 *    ★ 四个入口统一走 4 格环形队列：当前动作没走完就排队，绝不打断。
 *      qu(); fan(); gui(); 连着调用会依次执行，不会被顶掉。
 * ========================================================================== */

#include <string.h>

/* ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
 *  BENCH_LINE_ONLY —— 基础巡线隔离测试模式
 *   1 = 只跑「读三探头 → 按 q 给动作 → 速度闭环」，其余全不跑：
 *         不看物块相机、不做触须判停、不跑任务状态机/直角弯/舵机动作。
 *         上电立刻开始巡线；三个探头全白（丢线）就停车。
 *   0 = 正常工作模式（比赛用）。
 *  ⚠ 排查完必须改回 0，否则永远不会执行任务流程。
 * ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★ */
#define BENCH_LINE_ONLY   0

/* ---------------- 硬件引脚 ---------------- */
/* ---- 触须传感器（到站 / 离开的开关量判据）2026-10-20 ----
 *  开关量：拉簧与金属环接触 = 触发 → DATA 输出【低电平 0】；
 *          松开            = 未触发 → DATA 输出【高电平 1】。
 *  接线：GND / VCC(5V) / DATA→A0，第 4 脚悬空。
 *  示例代码打印 !(digitalRead(A0))：碰到 = 1，与本文件 whiskerOn() 语义一致。
 *
 *  ★★★ 必须用 INPUT_PULLUP，不能用裸 INPUT ★★★
 *    触须是机械接触式开关，"未触发"那一侧靠拉簧自分断 —— 若模块本身不是
 *    推挽输出，DATA 在松开时就是【悬空】的。悬空脚会随走线/电机干扰随机
 *    读到 0，表现为"车在空地上莫名判到站"。
 *    开内部上拉（20~50kΩ）后：模块推挽输出高电平时它只是并联、毫无影响；
 *    模块是开漏或干接点时，松开态被稳稳拉高 —— 两种情况下电平都确定。
 *  （原超声波 TRIG/ECHO 已整体移除，A0/A1 释放。） */
#define WHISKER_PIN  A0
#define WHISKER_ON   LOW     /* 触发（压住盒壁）时的电平；实物相反就改成 HIGH */
/* 触须是否压着盒壁（true = 触发） */
#define whiskerOn()  (digitalRead(WHISKER_PIN) == WHISKER_ON)

/* ★★★ 循迹探头左右定义（决定 100 / 1 / 110 三个分支往哪边修正）★★★
 *   0 = A4 是左探头、A5 是右探头        ← 当前（已确认 A4 装在左侧）
 *   1 = A4 装在右侧、A5 在左侧（接线与丝印相反）
 *
 *  判据：q==110（A4 + 中探头同时压线）时，车应该往哪边修正？
 *    · 应该往【左】修 → 线偏在车的左侧 → 填 0（A4 在左）
 *    · 应该往【右】修 → 线偏在车的右侧 → 填 1（A4 在右）
 *  改这一处，q=100 / q=1 / q=110 / 丢线盲走 会一起换边，方向始终自洽。
 *  ⚠ 左右修正方向若反了，改 TRACK_MIRROR 一处即可（PD 输出符号自动跟随）。 */
#define TRACK_MIRROR   0

/* ---------------- 机械参数（按你的实车核对！） ---------------- */
#define ENC_EDGES_PER_REV 30       // 码盘 15 孔 → 转一圈 30 个边沿（上升沿+下降沿）
/* ★ 上面这个是本文件里唯一还没实测确认的机械常量。
 *   标定法：让车按 baseSpeed=30 走完出发段（START_FWD_MS=1.5s，理论应走 45cm），
 *          拿尺量实际位移 D，则   新值 = 30 × 45 / D
 *          （走 36cm → 37.5；走 45cm → 30，说明原来就是对的）
 *   它只影响"绝对速度"的标定：两轮共用同一个常量，所以**不会造成左右跑偏**。
 *   但直角弯是按时长开环的（弧长 23.6cm ÷ 基速），速度标错 → 弧长算错
 */
#define WHEEL_DIA_CM      5.0      // 轮子直径 5cm（半径 2.5cm，已实测确认）
// 一个脉冲对应车走了多远：π*D / 一圈脉冲数 = 3.1416*5/30 = 0.5236 cm
#define CM_PER_EDGE (3.14159265 * WHEEL_DIA_CM / (double)ENC_EDGES_PER_REV)

/* ---------------- 速度环参数 ---------------- */
#define CTRL_PERIOD_MS 20       // 速度环周期 20ms（50Hz），够快也不浪费算力
/* ★ 测速低通从 0.50 降到 0.35（2026-10-21）：
 *   码盘只有 30 边沿/圈 = 0.5236cm/边沿，30cm/s 时边沿周期仅 17.5ms。
 *   "单周期测速"对开孔偏心极敏感（15 倍频纹波），0.50 跟得太快，
 *   纹波原样打进 PI 的差模通道 → 车会有一层细抖。0.35 的 τ≈50ms，
 *   仍远快于车体响应，但把纹波压掉一半。 */
#define SPEED_LP       0.35     // 测速一阶低通，越大跟得越快、越抖
#define STALL_US       150000UL // 超过 150ms 没有新脉冲 → 认为这个轮子停了
#define PWM_MAX        255
/* ★★★ 电机死区：必须【分轮 × 分方向】四个独立值（2026-10-21 重构）★★★
 *
 *  为什么不能像以前那样共用一个 PWM_MIN=50：
 *    · 左轮 6/5 走 Timer0 = 976Hz，右轮 10/9 走 Timer1 = 31.4kHz，载频差 32 倍。
 *      976Hz 下电流纹波大得多，低速时脉冲"一顿一顿"，实测死区通常比右轮高 3~6。
 *    · 正转与反转的电刷位置、齿轮背隙不同，两者也会差 3~8。
 *  同一 duty 两轮出力不等，就是"直线越走越偏"的系统性来源之一，
 *  所以四个值必须各自标定，不能共用一个数。
 *
 *  标定法（架空、单轮、另一轮断电）：
 *    duty 从 30 起每 30ms +1，记录"第一次连续出现 3 个编码器边沿"时的值；
 *    重复 9 次取 90 分位（宁可偏大，也不要出现"该转却不转"）。四组各做一遍。
 *
 *  下面四个先按典型值给，务必用你自己的实测替换。 */
#define D_MIN_LF       52       // 左轮 正转（Timer0，纹波大，通常偏高）
#define D_MIN_LB       55       // 左轮 反转
#define D_MIN_RF       48       // 右轮 正转（Timer1，纹波小，通常偏低）
#define D_MIN_RB       51       // 右轮 反转

/* ---- 驱动状态机的时序（见第 3 节 motorTick）----
 *  ★ 本板没有短路制动，(255,255) 不是刹车 —— 停机只有 (0,0) 断电滑行。
 *    所以没有"刹车时长"这种东西，只有"换向前先断电多久"。 */
#define M_REV_WAIT_MS 150       // 换向前全断这么久：让转速先掉下来，
                                // 避免反电动势与电源叠加（直接翻脚≈堵转电流 2 倍）。
                                // 滑行比电气刹车慢，所以这个值比原来保守。
#define M_BOOST_MS     60       // 起步/换向后的破静摩擦脉冲
#define M_BOOST_ADD    45       // 破死区时在目标占空比上额外加的量
/* 最低可指令速度：duty=D_MIN 时约 11cm/s。低于它 PD 会顶死在死区边界来回跳 */
#define V_MOVE_MIN     11.5

/* ---- 模态解耦 PI 参数（见第 5 节 updateSpeedPID）----
 *  在【共模 / 差模】两个正交坐标上各放一个 PI：
 *    共模 = 两轮之和 → 吸收电池压降（纯共模扰动，不会引起偏航）
 *    差模 = 两轮之差 → 吸收左右机械失配（纯差模扰动，不影响总速）
 *  这就是"编码器矫正"真正接进巡线的位置：差模环把 (vL-vR) 强行压到
 *  PD 要求的值上，于是"同 PWM 不同转速"的固定失配被彻底消掉。
 *  增益沿用原来两轮独立 PI 的 4.0/10.0（Ti=0.4s）：结构换了但环路增益不变，
 *  所以闭环响应与现在基本一致，改的是误差通道而不是手感 —— 便于对比验证。 */
#define KP_SUM          4.0     // 共模比例
#define KI_SUM         10.0     // 共模积分（Ti = Kp/Ki = 0.4s）
#define KP_DIFF         4.0     // 差模比例
#define KI_DIFF        10.0     // 差模积分
#define I_MAX          80.0     // 两个积分器各自的限幅（约 1/3 满量程）

/* ★★★ 前馈模型：cmd = Kff × (v − V_AT_DMIN)  ★★★
 *
 *  这是 2026-10-21 修正的一个**建模错误**。原来写的是 cmd = Kff × v，
 *  即"指令与速度成正比、过原点"。但死区让这句话不成立：
 *      duty = D_MIN 时轮子已经是 11cm/s，不是 0 —— 曲线有个台阶。
 *
 *  两条线的差别（取 a=0.222cm/s per duty, D_MIN=52）：
 *      正比模型 cmd = 3.5·v     v=30 → 105 ✓   v=15 → 52.5 ✗（正确值 19.8，过驱动 2.6 倍）
 *      仿射模型 cmd = 5.66·(v−11) v=30 → 107.5 ✓  v=15 → 22.6 ✓
 *  低速段（转弯时慢轮 12~15cm/s 正是这一段）过驱动会靠积分拉回来，
 *  但每次都先冲一下再收 —— 观感就是"转弯时慢轮一窜一窜"。
 *
 *  ★ Kff 的正确含义是【斜率】不是【割线】：
 *      Kff = 255 / (a × (255 − D_MIN))
 *  标定后按你的实测 a 与 D_MIN 重算，或直接让它在运行中自学习。 */
#define V_AT_DMIN      11.0     // duty = D_MIN 时的车速（cm/s）。标定后按实测填
#define KFF_INIT        5.6     // 斜率形式：a=0.222、D_MIN=52 → 255/(0.222×203)=5.66
#define KFF_MIN         3.0
#define KFF_MAX        10.0

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

/* ---------------- PID 参数已上移到"速度环参数"区 ----------------
 *  原来是 kp/ki/kd 三个全局量、两轮各一套 PID；现在改成模态解耦的两个 PI
 *  （KP_SUM/KI_SUM 共模 + KP_DIFF/KI_DIFF 差模）。
 *  增益数值仍取 4.0/10.0，闭环响应与旧版基本一致 —— 换的是误差通道，不是手感。 */

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
 *  ✔ 脉宽抖动源已消除：放置区相机走 **D0 硬件串口**，USART 由移位寄存器
 *    收字节，CPU 完全不参与，不再有关中断窗口。
 *    （历史：早前它挂在 D11 软件串口上，每字符要关中断 78us/148us，
 *      落在脉冲高电平窗口内会把脉宽拉长 10°~20°。该方案已删除。）
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
        /* svTicks 由 servoAngleToTicks() 写入，那里已钳到 SERVO_TK_MIN_OUT(275 tick)，
         * 所以 len 恒 > 0，不需要再兜一次。 */
        uint16_t len = svTicks[i];
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

/* ========================= 0.5 双 K230 视觉接入（菊花链） =========================
 *  两台相机【分工不同、脚本不同】，但到 Arduino 只有一根线、一个波特率：
 *
 *      放置区相机 111.py ──(IO9 TX = UART1_TXD, 9600)──▶ 物块相机 IO10(UART1_RXD)
 *      物块相机 4_v4.py  ──(IO9 TX = UART1_TXD, 9600)──▶ Arduino D0(RX, 硬件 USART)
 *                                            （放置帧由物块相机原样透传）
 *
 *  ① 物块相机 —— `4_识别并串口上报v4.py`  →  Arduino **D0（硬件串口）**
 *       UART1，IO9(TXD) →[1kΩ]→ D0，**9600** 8N1
 *       "1,1" 圆柱 / "1,2" 三棱柱 / "1,3" 正方体；无目标 "0,0"
 *       锁定后每 100ms 重发 → 物块种类 1/2/3 → visTask
 *
 *  ② 放置区相机 —— `111.py`  →  物块相机  →  Arduino D0（同一根线）
 *       UART1，IO9(TXD) →[1kΩ]→ 物块相机 IO10(UART1_RXD)，**9600** 8N1
 *       帧：'<' dd '>' CHK '\n'（7 字节）
 *       dd = 11 圆形 / 22 三角形 / 33 正方形 / 00 无目标 → 区域号 1/2/3
 *       CHK = ('<'+dd+'>') 累加和，两位十六进制大写
 *       每 400ms 心跳重发一次（帧是整帧一次发完，不再有字节间空隙）
 *
 *  ★ 两台相机都是 9600，代码里只有一个波特率常量 VISION_BAUD：
 *    不存在"按阶段切波特率"这回事，也就不会有 Serial.end()/begin() 清空
 *    RX 缓冲、丢掉半帧的问题。D0 上永远只有物块相机一个驱动，两台不可能抢线。
 *
 * ★ 1kΩ 串行电阻为什么还要留：
 *   两个 UART TX 都是推挽输出，直并在一条线上会互相对打（一个拉低、一个顶高）。
 *   菊花链之后"任何时刻只有一个驱动"已是拓扑保证，所以这 1kΩ 降级为故障保险：
 *   万一有相机跑着旧脚本、或复位后异常输出，没有电阻就是硬碰硬
 *   （对打电流可达 20~40mA，超过 IO 持续能力，可能打坏引脚或闩锁）；
 *   有电阻则只有 1.65mA，低电平仍能被拉到底，逻辑照样正确。成本两分钱。
 * ★ 只接 TXD：不要把 Arduino D1(TX) 接到任何相机的 RXD ——
 *    下载程序时 D1 上的数据会干扰视觉端。
 *    （K230_CMD_ENABLE=0，当前 D1 完全空闲，不发任何字节。）
 * ======================================================================== */
#define VISION_BAUD      9600       // 唯一波特率：物块相机 4_v4.py 及其透传的放置帧

/* ★ 菊花链：111.py(IO9 TX) → 4_v4.py(UART1 RX=IO10) → 4_v4.py(IO9 TX,9600) → D0
 *   物块相机把放置帧【原样透传】，所以到 D0 时已经是 9600。
 *   ⇒ D0 上永远只有物块相机一个驱动，两台不可能抢线。 */

/* 命令线（D1→两台 K230 的 RX）：菊花链之后两台相机不再需要被唤醒/休眠，
 * 只剩 'M'（关掉 K230 的 REPL 打印、省 CPU 提帧率）还有用。
 *   1 = 启用（D1 串 1k 分压接两路 RX）
 *   0 = 关闭（当前）—— 不往 D1 发任何字节 */
#define K230_CMD_ENABLE  0

/* 命令都是单字节，K230 侧只认自己关心的那条：
 *    'A' → K230#1（物块）  开始识别并发送
 *    'M' → K230#1          关闭 REPL 打印（识别结果照发，只是不再 print）
 *  ★ 命令与图像数据共用同一个 USART，所以必须用 VISION_BAUD 发。 */
#define CMD_WAKE_BLOCK  'A'
#define CMD_MUTE_PRINT  'M'

static void k230Send(char c)
{
#if K230_CMD_ENABLE
    Serial.write((uint8_t)c);
    Serial.flush();          // 等真的移出线路
#else
    (void)c;                 // 菊花链方案：没有命令线，直接空操作
#endif
}

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
/* ★ 放置区相机（111.py）每 400ms 发一次心跳，2.5s 收不到任何合法帧 = 链路断了，
 *  立即把 visSite 清零 —— 不能像老协议那样靠 30s 长超时兜底，否则上一个盒子的
 *  区域号会一直留到下一次投放，正是"幽灵投放/不投放"的来源之一。
 *  （取 2.5s 而不是 1.5s：留 6 个心跳周期的余量，容忍车身晃动导致的偶发漏帧。）
 *  相机侧无目标时会显式发 <00>，正常情况下 visSite 自己就会归零。 */
#define SITE_TIMEOUT_MS   2500UL

/* ========================= 0. 任务流程（单任务·间隙跳变判区） =========================
 *  一个搬运任务 = 取件 + 投放：
 *
 *    物块相机(D0) → 【搬取物种类】识别 → 1,1 圆柱 / 1,2 三棱柱 / 1,3 正方体
 *    放置区相机(D0) → 【放置区域】识别 → <11> 圆形 / <22> 三角形 / <33> 正方形
 *                  归一成区域号 1 / 2 / 3（相机同时报 <00> 表示没看到标志）
 *
 *  ★ 现场布局：路线旁沿线依次摆着多个停车盒（放置区），车从盒子旁边经过，
 *    侧向触须贴着盒壁，车"贴着某个盒"的整个行程里触须都保持压住。
 *
 *  ★ 到站判定：触须被盒壁压住（连续 WHISKER_CONFIRM 个样本）→ 停车 → 读放置区相机
 *    给出该盒的区域号（圆形/三角形/正方形标志 → 1/2/3）。
 *
 *  完整链路（一个任务）：
 *    开机 → 原地等 物块相机识别
 *     └─[取件] 识别到搬取物种类 f(1..3) → qu() → 出发直行 1.5s
 *           └─[巡线] 循迹 + 侧向触须
 *                 └─[到站] 触须压住 → 停车稳定 SETTLE_MS(1s) → 读区域号：
 *                       ├─ 区域号 == f → fan()+gui() 投放 → 巡线离开（触须松开）→ 完成
 *                       └─ 区域号 != f（或等不到相机）→ 前进走出当前盒 → 继续找
 *
 *  ★ 离开用"触须松开"判定，不用固定时长：
 *    停车扫码完成后继续巡线前进，触须一旦松开（弹回）即视为
 *    已离开当前物料放置区，再恢复后续流程。
 * ========================================================== */
int f = 0;              // 搬取物种类（1~3），由物块相机给出

bool dropDone   = false;   // 本次任务是否已【完成】投放（动作走完才算）
bool dropSent   = false;   // 本次到站是否已下发 fan()/gui()
int  dropCode   = 0;       // 本次到站锁存的区域号（一旦采信就固定，见 ST_DROP）
uint8_t stopCnt = 0;       // 本任务内第几次到站停车（从 1 开始数）

/* ---- 到站后的逐帧采样状态（见 ST_DROP）----
 *  dropCand/dropCandCnt：候选码与它连续出现的帧数，同码 2 帧才采信；
 *  dropFrameMs：上一次采样到的帧时间戳，用来识别"有新帧了"。
 *  ★ 用帧时间戳而不是毫秒差，是因为 visSiteMs 每收到一个合法帧都会刷新
 *    （包括 <00>），天然就是一个帧序号，不需要另加计数器。 */
static uint8_t       dropCand    = 0;
static uint8_t       dropCandCnt = 0;
static unsigned long dropFrameMs = 0;
/* ★ 到站窗口内的【多数表决】票箱（2026-10-21）：
 *   下标即区域号 1..3。原来要求"同一码连续 2 帧"，而 111.py 的识别本身会抖
 *   （33 → 00 → 33），"连续"等于要求它不抖 —— 实测凑不齐就整盒跳过。
 *   改成投票后：同码再来一帧就到 2 票（比原来还快），
 *   窗口到期只要有过 1 票也退让采信（宁可多投一次，不漏投）。 */
static uint8_t       dropVote[4] = {0, 0, 0, 0};
static uint8_t       dropVoteTot = 0;
/* ★ 任务成功完成 → 强制重新武装取件。
 *   原来只靠"相机报 0,0（物体看不见了）"来复位 taskArm，这有个死锁：
 *   物块相机的值在任务期间不刷新（解析全给了放置区帧），任务结束时若
 *   **下一个物块已经在视野里**，visTask 会从 1 直接变成 2、永远不经过 0，
 *   taskArm 就再也不会置位 —— 表现是"第一个任务跑完，之后再也不取件"。
 *   投放成功意味着手里那个物块确实已经放掉了，此时重新武装是安全的。 */
bool taskRearm  = false;

/* ★★★ 投放判据开关 —— 排障与兜底都用它，一次就能定位"为什么不投放"
 *
 *   0 = 严格按区域号：visSite == f 才投放        ← 依赖放置区相机（现场误检多，已暂停）
 *   1 = 按到站顺序  ：第 f 次停车就投放           ← 当前：不依赖相机
 *   2 = 到站即投放  ：任何一次停车都投放          ← 纯排障，用来验证 fan()/gui() 本身
 *
 *   ★ 当前取 1：放置区相机被黑色台面误检、菊花链又不稳，先把它从【投放判据】里
 *     摘掉 —— 停车仍然只由超声波决定，投放只由"第几次到站"决定。相机代码保留，
 *     改回 0 即恢复原行为（一行）。
 *   ★ 1 的前提：盒子必须【依次经过、且每次都能判停】。漏停一个，后面全部错位 ——
 *     所以这个模式的成败完全取决于超声波判停，判停参数不要乱动。
 * ==================================================================== */
#define DROP_MODE        0      /* 规则要求按形状投放（盒序随机），必须形状识别 */

/* ★★★ 电机标定模式（2026-10-21 新增）★★★
 *  1 = 上电后不跑任务，自动把四个 D_MIN_* 和两个轮子的增益标出来，
 *      结果从 D1/TX 打印（串口助手 9600 8N1），然后停在那里等复位。
 *  0 = 正常跑车。
 *  ★ 标定完必须改回 0，否则整车不会动。
 *  ★ 标定时把车架空（两个轮子离地），否则测出来的是"打滑的占空比"。 */
#define CALIB_MODE       0

#define START_FWD_MS   1500UL   // 取件后出发直行时长
/* ---- 到站停车与离开（触须开关判据）2026-10-20 ----
 *  触须碰到盒壁 = 到站（替代原超声波测距判停）
 *  WHISKER_CONFIRM : 连续 N 个样本都触发才停（滤机械抖动）
 *  LEAVE_MIN_MS    : 离开时至少前进这么久，防止在盒边反复停/走
 *  LEAVE_MAX_MS    : 离开超时兜底（触须一直被压住不放）
 * ------------------------------------------------------------------ */
#define LEAVE_MIN_MS    300UL   // 离开时的最小前进时间
#define LEAVE_MAX_MS   2000UL   // 离开超时兜底
#define SETTLE_MS      1000UL   // ★ 停车后等 1s 再读放置区相机：等车身机械振动
                                //   完全衰减 + 等 111.py 曝光/识别稳定（心跳 400ms，
                                //   1s 覆盖约 2 个心跳周期）。
/* ★ 到站后的【居中爬行】（只有 DROP_MODE=0 要扫描时才用）
 *
 *  判停点是盒子的近沿，标志在盒子中间 —— 不往前挪一小段，相机看不到标志。
 *
 *  ★★★ 本板没有电气刹车，停车一定过冲，所以位移必须按"两段"算 ★★★
 *      总前进量 = 触须触发后的滑行 + 爬行 + 爬行后的滑行
 *               ≈ (baseSpeed × 0.1~0.15s) + (DROP_CRAWL_CMS × T) + (DROP_CRAWL_CMS × 0.1s)
 *      30cm/s 起步时：滑行 3~4.5cm，之后全部用 15cm/s 慢爬。
 *
 *    当前 200ms @15cm/s = 3.0cm，爬完再滑 1.5~2cm →
 *      合计 7.5~9.5cm，正好落在 17.8cm 盒子的中心（8.9cm）。
 *    原来 300ms @30cm/s = 9cm，加上两段滑行是 15~17cm ——
 *      已经越过盒子中心，相机看到的是后半段，标志可能滑出视野下沿，
 *      这是"相机识别到了却不投"的另一个来源。
 *
 *  ★ 标定法（只需做一次，但必须实测）：
 *      在盒子正中贴一个十字标记，让车完整跑一次，看停车后标记在画面里的位置：
 *        标记偏下（车停早了）→ 加大 DROP_CENTER_MS
 *        标记偏上/出画（车冲过了）→ 减小 DROP_CENTER_MS，最低到 0
 *      每 100ms ≈ 1.5cm，按偏差量折算即可。 */
#define DROP_CRAWL_CMS   15.0   // 居中爬行速度（cm/s）：必须 > V_MOVE_MIN(11.5)
#define DROP_CENTER_MS  200UL   // 爬行时长：200ms × 15cm/s ≈ 3.0cm

/* ★ 到站后停多久才处置 / how long to stand still after the arrival stop：
 *   DROP_MODE=0（要扫描）：1s —— 等车身振动衰减 + 相机曝光稳定
 *   其它（不扫描）      ：300ms —— 停住就能投/走，别让每个盒子都"顿"一下 */
#if DROP_MODE == 0
  #define DROP_STOP_MS  SETTLE_MS
#else
  #define DROP_STOP_MS  300UL
#endif
/* ★ DROP_WAIT_MS：停车后等"合法区域码"的上限。
 *   从 800 抬到 1800：111.py 一次上报要 TRACK_MIN_HITS(4) 帧确认，心跳 400ms，
 *   车身停稳后第一帧合法码最坏要 ~800ms 才出现。原来 800ms 的窗口只够 2 个心跳，
 *   丢一帧就整盒跳过 —— 这正是"相机识别到了、车却不投"的直接原因。
 *   现在窗口 = SETTLE(1000) + 1800 ≈ 2.8s ≈ 7 个心跳周期。 */
#define DROP_WAIT_MS   1800UL   // 等相机报区域号的上限，超时当作"不是目标"
#define DROP_TAIL_MS   1000UL   // ★ 投放动作全部到位后再停 1s 才离开（原 300ms）。
                                //   servoReached 只保证"舵机角度到了"，而夹爪张开、
                                //   挡板下落要把物块真正放下去还需要一点时间。
                                //   原来一到位就开走，物块经常落在盒子边缘。
                                //   / Hold still briefly after the servos reach their
                                //     targets: the gripper is open but the block is still
                                //     falling, so driving off immediately lands it on the rim.
#define TASK_TIMEOUT_MS 50000UL // 单任务兜底超时：卡住也能自己回 IDLE
                                /* ★ 从 40s 抬到 50s：新增的 400ms/1s 间隔会让
                                 *   一个完整任务多花 4~6s（qu 3 步 + fan/gui 7 步
                                 *   + kai 4 步 + 3 处阶段间隔），40s 在长赛道上会误判超时。 */

/* ★★★ 强制间隔（防抢跑）★★★
 *  ACT_STEP_HOLD_MS : 机构动作里某一路舵机到位后，先停这么久再下发下一条。
 *                     qu/fan/gui/kai 全部自动生效 —— 只加在 actRun() 一处，
 *                     不在四个入口里各写一遍。
 *  STAGE_GAP_MS     : 两个任务阶段之间整体停这么久，才进下一阶段。
 *  ★ 两者都用 millis() 计时，不用 delay()：delay 期间速度环/超声波/循迹全停，
 *    电机会保持最后一次 PWM 盲冲（见文件头"机构动作里不要写 delay()"）。
 *  想关掉：把对应常量改成 0 即可，逻辑无需改动。 */
#define ACT_STEP_HOLD_MS 400UL
#define STAGE_GAP_MS    1000UL

/* ★★★ 取件与超声波的时间基准（两条都从 qu() 走完那一刻算起）★★★
 *  QU_DONE_TIMEOUT_MS：取件兜底。正常 qu 三步约 2.5~3.5s；若动作因为异常
 *      没跑完，等这么久就强制认为取件结束，不许把整车卡死在起步阶段。
 *  WHISKER_ARM_DELAY_MS：取件完成后延迟这么久才让触须上岗（到站判停）。
 *      起点周围常有相邻物块、器材腿，取件+起步段触须一动就误判到站；
 *      推迟到车已经正常巡线之后再上岗，判停才是干净的。
 *  想更早/更晚上岗只改这一个数。 */
#define QU_DONE_TIMEOUT_MS  8000UL
#define WHISKER_ARM_DELAY_MS 7500UL
#define WHISKER_CONFIRM      2   /* 连续 2 个样本压住 -> 判停 */
/* ★ 全静默窗（MATERIAL_MUTE_MS）已整体删除（2026-10-21）。
 *  它原来的两个理由现在都不成立：
 *    ① 物块相机每 100ms 的心跳干扰放置帧 —— 那是「按 stage 分流」造成的，
 *       现在改成按帧头分流，两条流各有独立缓冲，互不干扰。
 *    ② 起点附近的数据没有意义 —— 视觉状态本来就是「到了才用」：
 *       到站时用 dropFrameMs 把旧帧挡在外面，投票箱也在到站那一刻清零。
 *  而它带来的副作用是实打实的：视觉开闸晚于触须上岗，就出现
 *  「能停车却读不到码」的死亡窗口，车停对了盒子却被判不是目标位。
 *  删掉之后这一类 bug 在结构上不可能再出现，也不需要一个 #error 去守它。
 */

bool whiskerArmed = false;      /* 触须是否已上岗（qu() 完成 +WHISKER_ARM_DELAY_MS 后置 true） */
bool stopLocked  = false;       // 已判定到达目标区域，锁住电机
/* 收到过多少帧【校验通过】的放置区帧（含 <00>）。排障用：
 *   到站后 fc 不涨   → 字节流根本没到 / 格式不对（查菊花链转发与接线）
 *   fc 在涨但 v=0    → 帧到了，但相机报的是"无目标"（标志不在视野里） */
uint16_t siteOkCnt = 0;

/* 校验失败/断帧计数（排障用）：若这个数在涨而 siteOkCnt 不涨，
 * 说明线上有干扰或波特率不匹配，而不是相机没识别到。 */
uint16_t linkBadCnt = 0;

/* 排障计数：到站几次。
 * "平地顿一顿"时看这个数：
 *   a 在涨 → 是触须判到站（车被盒壁压住了）
 *   a 不涨 → 不是判停，是循迹丢线恢复（停 250ms + 倒退 450ms）在顿 */
uint16_t arriveCnt    = 0;

/* ST_DROP 的两段式：0=未开始 1=向前爬行居中 2=停稳扫描 */
uint8_t       dropPhase   = 0;
unsigned long dropPhaseMs = 0;

unsigned long fwdUntil  = 0;    // 出发段结束时刻（0 = 还没开始算，等 qu() 走完）
unsigned long quDoneMs  = 0;    // qu() 走完的时刻，0 = 还没走完（全任务的起点基准）
unsigned long leaveStart = 0;   // 投放后巡线离开的起点时刻
unsigned long dropTailUntil = 0;// 投放动作完成后的停留截止时刻（见 DROP_TAIL_MS）

/* 任务阶段 */
#define ST_IDLE      0    // 无任务执行，原地等物块相机
#define ST_PICK_FWD  1    // 取件 + 出发段直行
#define ST_LINE      2    // 巡线 + 判区
#define ST_DROP      3    // 到达目标区域，投放
#define ST_LEAVE     4    // 投放后巡线离开（测距>22cm 即出区）
int stage = ST_IDLE;
unsigned long taskStartMs = 0;  // 任务起点，供 TASK_TIMEOUT_MS 兜底

/* 阶段切换的唯一入口：改阶段的同时记下 STAGE_GAP_MS 的间隔。
 * ★ 新增分支时请用 stageGoto()，直接写 stage = xxx 会绕过阶段间隔。
 */
static unsigned long stageHold = 0;      // 阶段间隔截止时刻，0 = 无间隔

static void stageGoto(int next)
{
    stage     = next;
    stageHold = millis() + STAGE_GAP_MS;
}

/* ★ 无间隔切换（2026-10-21）：只用在"车本来就在动、下一阶段立刻接上"的地方。
 *   典型两处：出发段 → 巡线、以及"不是目标盒 → 继续巡线"。
 *   这两处中间停 1s 纯粹是浪费：
 *   车要重新起步、积分被清零、用户看到的就是"前进 1s 顿一下"。
 *   需要"先停稳再进下一阶段"的地方（投放完成回 IDLE、到站进 ST_DROP）继续用 stageGoto()。
 *
 *   ★ 必须同时清 lostStart：
 *     进 ST_LINE 后的第一轮 lineFollow() 会调 lostLine()，那里算 el = millis()-lostStart。
 *     出发段/离开段可能已经累积了几百毫秒的"q==0"，不清就会一进巡线就判
 *     "丢线超时" → 立刻停/倒退。这正是原来必须靠 1s 间隔顺带清计时器的原因。 */
extern unsigned long lostStart;      // 定义在第 6 节

static void stageGotoNow(int next)
{
    stage     = next;
    stageHold = 0;
    lostStart = 0;
}

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

/* ========================= 3.5 机构动作：显式 step 状态机 =========================
 *  一个动作 = 若干个"舵机 + 目标角"按顺序走完。执行器只有两个阶段：
 *      PH_SEND ：本步目标已下发，等 servoReached() 认可
 *      PH_HOLD ：已到位，等 ACT_STEP_HOLD_MS 走完
 *  推进条件写死在 actRun() 的 switch 里，所以"抢跑"在结构上不可能发生：
 *      没到位 → 出不了 PH_SEND；没停够 → 出不了 PH_HOLD。
 *
 *  流程（哪个动作走哪几步）只写在 actStart() 的 switch 里，是唯一真源；
 *  步骤内容放在一张连续表 STEP_TAB，用 [first, last] 区间取。
 *  加一步只改表 + 一个数字，不会再出现"四套数组长度对不上 / 顺序错位"。
 *  全程不写 delay()，也不碰 tgtL/tgtR —— 车该怎么走还怎么走。
 * ========================================================================== */
enum { ACT_NONE = 0, ACT_QU, ACT_FAN, ACT_GUI };

/* 阶段：用 0 表示空闲，两个工作阶段从 1 开始 */
enum { PH_IDLE = 0, PH_SEND, PH_HOLD };

struct ActStep { uint8_t pin; float deg; };

/* 步骤表：注释里的顺序就是执行顺序 */
static const ActStep STEP_TAB[] =
{
    /* qu  取件       */ {12, 250}, { 7,  45}, { 8, 225},             // 0..2
    /* fan 投放       */ { 7,  40}, { 8, 220}, { 4, 180}, {12, 180},  // 3..6
    /* gui 归位       */ { 4,  90}, { 8, 225}, { 7,  45},             // 7..9
};

/* ★ 编译期自检：表长度必须与 actStart() 里的区间完全对齐。
 *   往表里加了一步却忘了改 actStart，这里直接编译不过 ——
 *   正是为了防止"表与下标错位"这一类难查的 bug（上次 kai 动错轴就是这一族）。 */
static_assert(sizeof(STEP_TAB) / sizeof(STEP_TAB[0]) == 10,
              "STEP_TAB 行数与 actStart() 的区间不同步：改表必须同步改 actLast");

static uint8_t       actId    = ACT_NONE;
static uint8_t       actPhase = PH_IDLE;
static uint8_t       actIdx   = 0;      // 当前步
static uint8_t       actLast  = 0;      // 本动作的最后一步
static unsigned long actHoldUntil = 0;  // 本步停留截止时刻

/* ★★★ 待执行动作：4 格环形队列（只存动作号，1 字节/格）★★★
 *  当前动作没走完就排队，绝不被顶掉：qu(); fan(); gui(); 连着调用会依次执行。
 *  队列满（4 个）才丢弃 —— 正常流程最多同时排 2 个（fan → gui）。 */
#define ACT_Q_MAX 4
static uint8_t qId[ACT_Q_MAX];
static uint8_t qHead = 0;      // 队头：下一个要执行的
static uint8_t qCnt  = 0;      // 队列里排着几个

static bool actDone() { return actId == ACT_NONE && qCnt == 0; }

/* 动作 → 步骤区间，并下发第一步。
 * ★ 只有这里能改 actId，所以"流程"只有一份定义，加动作只动这张 switch。 */
static void actStart(uint8_t id)
{
    uint8_t first = 0;

    switch(id)
    {
        case ACT_QU:       first =  0; actLast =  2; break;   // D12 → D7  → D8
        case ACT_FAN:      first =  3; actLast =  6; break;   // D7  → D8  → D4 → D12
        case ACT_GUI:      first =  7; actLast =  9; break;   // D4  → D8  → D7
        default:           return;                            // 非法动作号：什么都不做
    }

    actId    = id;
    actIdx   = first;
    actPhase = PH_SEND;
    servoWrite(STEP_TAB[first].pin, STEP_TAB[first].deg);
}

/* 入队：空闲且队列空 → 立即开始；否则排到队尾。
 * 返回 false = 队列已满，本次动作被丢弃（正常流程不会发生）。 */
static bool actQueue(uint8_t id)
{
    if(id == ACT_NONE) return false;
    if(actId == ACT_NONE && qCnt == 0) { actStart(id); return true; }
    if(qCnt >= ACT_Q_MAX) return false;

    qId[(uint8_t)((qHead + qCnt) % ACT_Q_MAX)] = id;
    qCnt++;
    return true;
}

/* 每轮 loop 调一次；返回 true 表示当前动作与队列都空了。
 *
 *  ★ 三步节奏全在这一个 switch 里，谁也跳不过去：
 *      ① PH_SEND 等到位 → 才进 PH_HOLD（防"没到位就动下一路"）
 *      ② PH_HOLD 停够 400ms → 才下发下一步（防"刚到位就动下一路"）
 *      ③ 最后一步停够 → 才从队头取下一个动作（防两个动作叠在一起）
 *    "到位"只代表角度到了，机构（夹爪张开、挡板下落）还需要时间，这就是 ② 的意义。 */
static bool actRun()
{
    if(actId == ACT_NONE)
    {
        if(qCnt == 0) return true;
        uint8_t id = qId[qHead];
        qHead = (uint8_t)((qHead + 1) % ACT_Q_MAX);
        qCnt--;
        actStart(id);
        if(actId == ACT_NONE) return actDone();       // 非法动作号：直接跳过
    }

    switch(actPhase)
    {
        case PH_SEND:
            if(!servoReached(STEP_TAB[actIdx].pin)) return false;   // ① 这一路还没转到位
            actPhase     = PH_HOLD;
            actHoldUntil = millis() + ACT_STEP_HOLD_MS;            // ② 到位 → 开始停留
            if(actHoldUntil == 0) actHoldUntil = 1;                // 0 是"无停留"哨兵
            return false;

        case PH_HOLD:
            if(millis() < actHoldUntil) return false;              // ② 还没停够
            if(actIdx >= actLast)                                  // ③ 本动作走完
            {
                /* ★ 取件走完的时刻是全任务的时间基准：出发段、超声波上岗都从
                 *   这一刻算起（见 quDoneMs）。必须在这里标，别处都不可靠。 */
                if(actId == ACT_QU) quDoneMs = millis();
                actId    = ACT_NONE;
                actPhase = PH_IDLE;
                return (qCnt == 0);
            }
            actIdx++;
            servoWrite(STEP_TAB[actIdx].pin, STEP_TAB[actIdx].deg);
            actPhase = PH_SEND;
            return false;

        default:
            return false;
    }
}

/* 三类物料的取放动作完全一样，所以不带参数 —— 想按物块区分时再加重载。
 * ★ 三个入口全部走队列：当前动作没走完就排队，不打断。 */
void qu()  { actQueue(ACT_QU); }    // 取件
void fan() { actQueue(ACT_FAN); }   // 投放
void gui() { actQueue(ACT_GUI); }   // 归位：排在 fan 之后

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

/* ---- 放置区帧解析（111.py，7 字节 ASCII）--------------------------------
 *      '<' d0 d1 '>' h1 h0 '\n'
 *      d0 d1 = 两位十进制编码：11 圆 / 22 三角 / 33 方 / 00 无目标
 *      h1 h0 = ('<' + d0 + d1 + '>') 的字节累加和，两位大写十六进制
 *
 *  ★★★ 容错策略：遇到"不该出现的字节"就【原地忽略】，绝不整帧作废 ★★★
 *    D0 上同时跑着两台相机的字节：物块相机每 100ms 插一行 "1,1\n"，
 *    放置帧则由 444.py 逐块转发。转发一旦被切成两段，线上就是：
 *
 *        <22    ← 转发块 1
 *        1,1\n  ← 物块相机的心跳，插在中间
 *        >DE\n  ← 转发块 2
 *
 *    原来的写法在"等 '>'"这一步收到 '1' 就整帧清零，这帧直接报废。
 *    每帧都被切 → 一帧都收不全 → 到站永远判不出区域号（相机侧显示一切正常）。
 *    改成"忽略杂字节、保持当前状态"后，同一段字节流可以复原：
 *        '<'→d0='2'→d1='2'→('1' ',' '1' 忽略)→'>'→'D'→'E'→'\n'
 *        → 累加和 0xDE 与 <22> 吻合 → 采信 22。
 *    安全性：d0/d1/h0/h1 都取自本帧，最后仍要过累加和；拼错的组合只有 1/256
 *    能碰巧通过，上层还有"同码连续 2 帧"再确认一次。
 * -------------------------------------------------------------------- */
static int hexVal(char ch)
{
    if(ch >= '0' && ch <= '9') return ch - '0';
    if(ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    if(ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    return -1;
}

/* ---- 放置区帧 <dd>CHK\n 的状态机 ----
 *  ★ siteSt 提到文件作用域（原来是函数内 static），因为 linkFeed() 要靠它
 *    判断"当前是否正处在一帧中间"，从而决定这个字节该送给谁。
 *    0 = 帧外，1..6 = 收帧进度。
 *
 *  ★ 新增 n 计数（收帧字节数）：某个分支一旦等不到预期字符就会一直停在那一档，
 *    以前无所谓（本来也只喂放置帧），现在如果不能自己复位，
 *    会把后面所有的物块帧一起吃掉 —— 那是一个比丢帧严重得多的故障。
 *    完整的帧最多 7 字节，收到第 12 个还没闭合就一定断帧，直接复位。 */
static uint8_t siteSt = 0;

static void siteFeed(char ch)
{
    static uint8_t d0 = 0, d1 = 0, h0 = 0, h1 = 0;
    static uint8_t n  = 0;                   // 本帧已收字节数（防卡死）

    if(ch == '<') { siteSt = 1; n = 0; return; }   // 帧头：任何位置都能重新对齐
    if(siteSt == 0) return;                        // 帧外字节：丢弃
    if(++n > 12) { siteSt = 0; return; }           // 断帧 → 复位

    switch(siteSt)
    {
        case 1:                               // 第一位：协议只有 00/11/22/33
            if(ch >= '0' && ch <= '3') { d0 = (uint8_t)(ch - '0'); siteSt = 2; }
            break;

        case 2:                               // 第二位必须【等于第一位】
            /* ★ 协议里 dd 恒为 00/11/22/33，两位永远相同。
             *   于是"不相等的数字"一定是杂字节，原地忽略即可，不必整帧作废。
             *   例：<2 | 1,1\n | 2>DE\n → '1' 不等于 '2' 被忽略，
             *       后面的 '2' 才被当成第二位 → 帧完整复原。 */
            if(ch == (char)('0' + d0)) { d1 = d0; siteSt = 3; }
            break;

        case 3:                               // '>'
            if(ch == '>') siteSt = 4;
            break;

        case 4:                               // 校验高半字节
        {
            int v = hexVal(ch);
            if(v >= 0) { h0 = (uint8_t)v; siteSt = 5; }
            break;
        }

        case 5:                               // 校验低半字节
        {
            int v = hexVal(ch);
            if(v >= 0) { h1 = (uint8_t)v; siteSt = 6; }
            break;
        }

        case 6:                               // 等换行，然后校验
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
                    siteOkCnt++;                        // 合法帧计数（含 <00>），排障用
                }
                else linkBadCnt++;                      // 校验错：计数，便于现场判断链路质量
                siteSt = 0;
            }
            break;

        default:
            siteSt = 0;
            break;
    }
}

/* ---- 物块帧 1,x\n 的解析（帧外字节才进来）----
 *  单独一个行缓冲：它与放置帧在字节层面完全分开，
 *  不会再把 '1' 塞进 <...> 的校验位里。 */
static void blockFeed(char ch)
{
    static char    buf[8];
    static uint8_t n = 0;

    if(ch == '\n' || ch == '\r')
    {
        if(n > 0) { buf[n] = 0; parseVisionFrame(buf); n = 0; }
        return;
    }
    if(ch >= 32 && ch < 127)
    {
        if(n < sizeof(buf) - 1) buf[n++] = ch;
        else                    n = 0;      // 超长行：整行丢弃重来
    }
    else n = 0;
}

/* ---- 统一入口：按【帧头】分流，不再按 stage 分流 ----
 *
 *  ★ 这是本次重写的核心。旧实现按 stage 决定"解析哪一路"：
 *      ST_IDLE → 只解析 1,x；其它 → 只解析 <cc>CHK。
 *    但物块相机每 100ms 无条件发心跳（它不知道主板在哪个阶段），
 *    于是任务期间 1,x 和 <cc>CHK 在同一根线上交替出现，而解析器只开了一路：
 *      · 1,x 被喂进 siteFeed  → 校验位被 '1' 吃掉 → 整帧作废
 *      · 阶段切换瞬间残留的半帧被喂给另一路 → 又丢一帧
 *    实测丢帧率约 7%（帧宽 7.3ms ÷ 心跳周期 100ms），到站后 2.8s 窗口里
 *    连丢两三帧就判"不是目标位"开走 —— 这是"识别到了却不投"的通讯侧真因。
 *
 *  ★ 现在改成：'<' 开头的一律归放置帧；其余一律归物块帧。
 *    两条支路的缓冲互相独立，任何字节最多只污染它自己那一路。 */
static void linkFeed(char ch)
{
    if(ch == '<')        { siteFeed(ch); return; }   // 帧头：无条件归放置区
    if(siteSt != 0)      { siteFeed(ch); return; }   // 帧内字节：只能给放置区
    blockFeed(ch);                                   // 帧外：只可能是物块帧
}

/* ---- 视觉入口 ---------------------------------------------------------
 *  D0 上一根线、一个波特率（VISION_BAUD=9600），两条相机流在这里交汇：
 *      物块相机 -> 1,x\n        (100ms 心跳，无条件发)
 *      放置相机 -> <cc>CHK\n     (400ms 心跳，由物块相机转发)
 *  ★ 分流由 linkFeed() 按【帧头】完成，不再看 stage —— 原因见 linkFeed 上方。
 *
 *  ★ 全静默窗（原 MATERIAL_MUTE_MS）已整体删除：
 *    它当初的作用是让起点附近两台相机别互相干扰，但真正的干扰源是
 *    「按 stage 分流」这个错误假设，现在已从根上解决。
 *    保留静默窗反而带来一个更危险的副作用：触须先上岗、视觉后开闸，
 *    中间那段能停车却读不到码的死亡窗口 —— 车停对了盒子却被判不是目标位。
 *    这个窗口只由 MATERIAL_MUTE_MS 与 WHISKER_ARM_DELAY_MS 的相对大小决定，
 *    靠人工记是记不住的（历史上已经错过一次 8000 vs 7500），所以直接删掉。
 * -------------------------------------------------------------------- */
static void pollVision()
{
    /* ★ 命令心跳：每秒重发一次唤醒码（K230_CMD_ENABLE=0 时整段不编译）。
     *   物块相机的上电时间不确定，可能比主板晚几百 ms，只发一次会漏掉。 */
#if K230_CMD_ENABLE
    static unsigned long lastCmdMs = 0;
    if(millis() - lastCmdMs > 1000UL)
    {
        lastCmdMs = millis();
        k230Send(CMD_WAKE_BLOCK);
    }
#endif

    while(Serial.available() > 0) linkFeed((char)Serial.read());

    /* ★ 两条链各自独立超时，不再「只在听它的时候才计时」。
     *   现在两路始终都在解析，所以超时也必须始终生效，否则断链发现不了。
     *   visTask 归零不会误触发任务：taskArm 只在 ST_IDLE 里维护。 */
    if(visTask && millis() - visTaskMs > VISION_TIMEOUT_MS) visTask = 0;
    if(visSite && millis() - visSiteMs > SITE_TIMEOUT_MS)   visSite = 0;
}

/* 视觉接收 + 任务启动
 *   物块种类 f ← visTask（4_v4.py，D0，1/2/3）
 *   区域号 r   ← visSite（111.py，经物块相机透传到 D0），在 ST_DROP 停车之后读取
 *  ---------------------------------------------------------------------- */
static void pollVisionAndQueue()
{
    pollVision();

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
        /* ★ 出发段与超声波都不在这里定时 —— 它们从 qu() 走完那一刻算起。
         *   fwdUntil=0 是"还没开始算"的哨兵，move_tai 里见到 0 会先把车停住
         *   等取件动作走完，再按 quDoneMs 起算 1.5s。 */
        fwdUntil    = 0;
        quDoneMs    = 0;
        taskStartMs = millis();
        stageGoto(ST_PICK_FWD);
        stopLocked  = false;
        dropDone    = false;
        dropSent    = false;
        dropCode    = 0;          // 上个任务锁存的区域号不能带到新任务
        dropTailUntil = 0;        // 投放停留计时也不能带过来
        stopCnt     = 0;          // 到站计数归零：DROP_MODE=1 靠它判"第几个盒子"
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
volatile unsigned long encDtL  = 0, encDtR  = 0;   // 相邻两个边沿的间隔(us)
volatile unsigned long encTmL  = 0, encTmR  = 0;   // 上一个边沿发生的时刻(us)
volatile uint8_t       pcLast  = 0;
volatile int8_t        dirL = 1, dirR = 1;         // 当前指令方向 +1正转 -1反转（ISR 也读）

/* 超声波回波测宽：ECHO = D14 = A0 = PC0 = PCINT8（当前 ECHOPIN=14），
 * 与编码器同属 PCINT1 组，所以它的边沿也进这个 ISR —— 顺带处理，零额外开销。
 * ★ 具体是哪一位由 ECHO_PC 决定（跟着 ECHOPIN 走），这里不要写死。 */
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

    /* ★★★ 离群剔除（2026-10-21 新增）
     *   单周期测速对码盘开孔偏心极敏感：15 孔/圈 → 速度上带 15 倍频纹波，
     *   再叠一个边沿抖动就会蹦出 2~3 倍的假峰，直接打进差模环 →
     *   车会有一层细密的左右抖。这里把"比上次大 60% 以上 / 小 40% 以上"的样本
     *   丢掉并沿用上次值：真实加减速是渐变的，单拍跳不了这么多，不会拖慢响应。
     *
     *   ★ 状态必须【按轮分开】。本函数被左右轮共用一个 static 会串味：
     *     左右交替调用时 lastV 是对方的读数，剔除判据直接失效。
     *     用传入的指针当钥匙，两个调用点天然分开。 */
    static const volatile unsigned long *lastKey = 0;
    static double lastV = 0.0;
    if(lastKey != dt) { lastKey = dt; lastV = 0.0; }

    double v = CM_PER_EDGE * 1000000.0 / (double)t;
    if(lastV > 1.0 && (v > lastV * 1.60 || v < lastV * 0.40)) return lastV;

    lastV = v;
    return v;
}

/* ========================= 2. 手写 PID =========================
 * 就是你贴的那条式子离散化：out = Kp*e + Ki*∫e dt + Kd*de/dt
 * 写成手写版是为了不依赖 PID_v1 库（省得装库报错），也方便看清每一项在干嘛。
 * ========================================================== */
/* ========================= 2.5 模态 PI 的积分器 =========================
 *  两轮独立 PID 已废弃，结构见第 5 节"模态解耦"：
 *    sumI  —— 共模积分，吃电池压降（压降是纯共模扰动，不该引起偏航）
 *    diffI —— 差模积分，吃左右机械失配（失配是纯差模扰动，不该影响总速）
 *  放成文件级静态量而不是 updateSpeedPID 里的 static，
 *  是因为 ting() 停车时要一并清零，不能等它自己慢慢退。
 * ====================================================================== */
static double sumI  = 0.0;
static double diffI = 0.0;

/* ========================= 3. 电机输出层：分轮状态机 =========================
 *  硬件：每台电机两个 PWM 脚（无独立方向脚），所以是 sign-magnitude 驱动：
 *      正转 = (正转脚=duty, 反转脚=0)     反转 = (0, duty)     停 = (0,0) 全断
 *  左轮 = D6 正 / D5 反（Timer0, 976Hz）；右轮 = D10 正 / D9 反（Timer1, 31.4kHz）
 *
 *  ★★★ 本板【没有短路制动】(2026-10-21 用户确认) ★★★
 *    所以整台车只有三种电气状态：正转 / 反转 / 断电滑行，没有第四种。
 *    "停车"就是 (0,0)，刹车在电机层无法实现。
 *
 *    这带来一个必须接受的物理事实：停车一定会过冲
 *        过冲 ≈ 车速 × 惯性时间(约 0.1~0.15s)   30cm/s → 3~5cm
 *    既然电机层补不了，就只能在【提前量】上补 —— 见 DROP_CENTER_MS 的标定说明。
 *
 *  ★ 那为什么还要状态机？因为它还管三件必须排时序的事：
 *    ① 换向：直接翻脚会让反电动势与电源叠加（冲击电流接近堵转电流 2 倍），
 *       中间必须插一段全断，让转速先掉下来。
 *    ② 起步：静摩擦 > 动摩擦，给刚好等于目标的占空比常常起不来，要给一脚 Boost。
 *    ③ 停机幂等：ting() 每轮 loop 都调用，状态机保证不会反复重发。
 *
 *  状态：COAST → BOOST → DRIVE →(停车) COAST
 *                                →(换向) WAIT → BOOST → DRIVE
 * ========================================================================== */
#define M_COAST  0      // 全断，滑行（唯一的"停"）
#define M_DRIVE  1      // 按指令出力
#define M_WAIT   2      // 换向前先全断 T_REV_WAIT_MS，让电机转速降下来
#define M_BOOST  3      // 破静摩擦的短脉冲

struct MotDrv
{
    uint8_t  pinF, pinB;     // 正转脚 / 反转脚
    uint8_t  dMinF, dMinB;   // 该方向的最小启动占空比（四个值各不相同）
    uint8_t  st;             // 当前状态
    int16_t  want;           // 期望的带符号占空比（-255..255）
    int8_t   dir;            // 已建立的正转/反转方向 ±1，0=未建立
    int16_t  outDuty;        // 上次真正写到引脚的占空比（写缓存）
    int8_t   outDir;         // 上次写出的方向（写缓存，-1/0/1）
    unsigned long t0;        // 进入当前状态的时刻
};

static MotDrv motL = {6,  5, D_MIN_LF, D_MIN_LB, M_COAST, 0, 0, 0, 0, 0};
static MotDrv motR = {10, 9, D_MIN_RF, D_MIN_RB, M_COAST, 0, 0, 0, 0, 0};

/* ★ 写缓存：状态机每轮 loop 都会被推一次，但绝大多数时候输出没变。
 *   不做缓存的话，稳态巡线时每秒会白写几百次 analogWrite（每次几个 us）。 */
static void motWrite(MotDrv *m, int16_t duty, int8_t d)
{
    if(m->outDir == d && m->outDuty == duty) return;
    m->outDir  = d;
    m->outDuty = duty;

    /* 铁律：先把要关的那一脚清零，再给另一脚加占空比。
     * 反过来的话，两脚会有一瞬间同时为高 → 一次意外刹车脉冲 → 电流尖峰。 */
    if(d > 0)      { analogWrite(m->pinB, 0);                analogWrite(m->pinF, (uint8_t)duty); }
    else if(d < 0) { analogWrite(m->pinF, 0);                analogWrite(m->pinB, (uint8_t)duty); }
    else           { analogWrite(m->pinF, 0);                analogWrite(m->pinB, 0); }
}

/* 没有短路制动，这个函数只是"换向前把两脚全断"的统一入口 */
static void motOpen(MotDrv *m)
{
    motWrite(m, 0, 0);
}

/* 状态推进。本板无电气制动，所以只有"出力/断电"两种电气动作，
 * 状态机存在的意义是给【换向】排时序，以及给【起步】破静摩擦。 */
static void motTick(MotDrv *m)
{
    unsigned long now = millis();
    int16_t aw = (m->want < 0) ? (int16_t)(-m->want) : m->want;   // 目标幅值
    int8_t  nd = (m->want > 0) ? 1 : ((m->want < 0) ? -1 : 0);    // 目标方向

    switch(m->st)
    {
        case M_COAST:
            motWrite(m, 0, 0);
            m->dir = 0;
            if(aw > 0) { m->st = M_BOOST; m->t0 = now; }   // 静止起步：直接给破死区脉冲
            break;

        case M_DRIVE:
            if(aw == 0) { m->st = M_COAST; m->dir = 0; break; }        // 停车：直接全断
            if(nd != m->dir) { m->st = M_WAIT; m->t0 = now; break; }   // 换向：先断电
            {
                int16_t dMin = (m->dir > 0) ? m->dMinF : m->dMinB;
                if(aw < dMin) aw = dMin;       // 映射层已保证，这里是最后一道保险
                motWrite(m, aw, m->dir);
            }
            break;

        case M_WAIT:
            motOpen(m);                                    // 全断，等转速掉下来
            if(now - m->t0 >= M_REV_WAIT_MS)
            {
                if(aw > 0) { m->dir = nd; m->st = M_BOOST; m->t0 = now; }
                else       { m->st = M_COAST; m->dir = 0; }
            }
            break;

        case M_BOOST:
        {
            if(aw == 0) { m->st = M_COAST; m->dir = 0; break; }
            int16_t boost = aw + M_BOOST_ADD;                 // 破静摩擦
            int16_t dMin  = (m->dir > 0) ? m->dMinF : m->dMinB;
            if(boost > PWM_MAX) boost = PWM_MAX;
            if(boost < dMin)    boost = dMin;
            motWrite(m, boost, m->dir);
            if(now - m->t0 >= M_BOOST_MS) m->st = M_DRIVE;
            break;
        }

        default:
            m->st = M_COAST;
            break;
    }
}

/* 电机输出是否已经静止：供 ting() 幂等判断（ST_IDLE 每轮都会调用它） */
bool motorsIdle = true;

/* 推进两台电机的状态机。loop() 每轮调用一次，成本在输出不变时只有两次比较。 */
void motorService()
{
    motTick(&motL);
    motTick(&motR);
    motorsIdle = (motL.st == M_COAST && motR.st == M_COAST);
}

/* ★ 对外接口保持原样：带符号 PWM，正=正转 负=反转 0=停。
 *   所有调用点（updateSpeedPID / ting / setup / BENCH）都不用改。
 *   dirL/dirR 仍表示【指令方向】，测速符号要用它。 */
void motorLR(int pwmL, int pwmR)
{
    motL.want = (int16_t)pwmL;
    motR.want = (int16_t)pwmR;

    dirL = (pwmL >= 0) ? 1 : -1;
    dirR = (pwmR >= 0) ? 1 : -1;

    motorService();          // 立刻推一次，别等到下一轮 loop
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
// ★ 停车：目标清零 + 立刻下发（不等 PID 那 20ms），紧急情况要的就是快
void ting()
{
    straightMode = false;
    tgtL = 0; tgtR = 0;
    sumI  = 0; diffI = 0;        // 两个模态积分一起清，停车不许带旧账

    /* ★ 这里不再"直接把引脚写 0"。
     *   旧写法 motorLR(0,0) 是【断电滑行】—— 30cm/s 下惯性必然再冲 3~5cm，
     *   这就是"触须碰到盒子还往前顶几厘米"的直接原因。
     *   现在 motorLR(0,0) 只是把目标置 0，由电机状态机按车速决定：
     *   还在动 → 先刹车 60ms 再滑行；已经停了 → 直接滑行。
     *   motorsIdle 为真说明两台都已在 COAST，不必重复下发（省 CPU）。 */
    if(!motorsIdle) motorLR(0, 0);
}

/* ---- 指令 → 占空比映射（死区补偿的第一级，也是最重要的一级）----
 *  把 0..255 的指令线性铺到 dMin..255，整段单调、增益恒定。
 *
 *  ★ 为什么必须替换掉旧写法 if(o<dMin) o=dMin：
 *    钳位会让指令 1..50 全部输出同一个占空比 dMin —— 这一段增益为 0。
 *    PID 想把速度从 11 拉到 12，输出从 10 加到 40，车纹丝不动；
 *    积分于是继续涨，一旦冲破 50 就猛地窜到 20cm/s，然后超速、回落、再涨。
 *    这就是"走走停停"的完整机制，跟 Kp/Ki 调得好不好无关。
 *    映射后每一格指令都对应一格实际出力，积分才有物理意义。 */
static int dutyMap(double cmd, uint8_t dMin)
{
    double mag = (cmd < 0) ? -cmd : cmd;            // 不能叫 a：会和循迹全局量 a 撞名
    if(mag < 0.5) return 0;                         // 小于半格视为 0，别为噪声抖电机
    if(mag > PWM_MAX) mag = PWM_MAX;
    double d = (double)dMin + (mag / (double)PWM_MAX) * (double)(PWM_MAX - dMin);
    return (int)((cmd < 0) ? -(d + 0.5) : (d + 0.5));
}

/* ========================= 5. 速度环：测速 → 模态解耦 PI → 死区映射 =========================
 *  ★ 为什么不再是"两轮各一个 PID"（2026-10-21 重构）：
 *    左右轮速差有两个来源，性质完全不同：
 *      · 电池压降 → 两轮【同向】变化（共模扰动）
 *      · 电机/齿轮/轮径失配 → 两轮【反向】变化（差模扰动）
 *    两个独立 PID 把这两种扰动投在同一条轴上，积分器会互相打架；
 *    一旦两轮的抗饱和不对称，退饱和瞬间就偏航。
 *    改成模态分解后，两种扰动各走各的通道，互不污染。
 *
 *  ★ 这也是"编码器矫正没接进巡线"的正解：
 *    巡线给的是【轮速差目标】(tgtL−tgtR)，差模环把实测差压到这个值上。
 *    于是"同 PWM 两轮转速不同"这个固定失配被彻底消掉，
 *    不需要再靠 headCorr 去补 —— headCorr 只管 qian() 的盲直行。
 * ============================================================================ */
/* 前馈：把目标速度换算成"映射前的指令"。
 *  ★ 必须用仿射式：duty=D_MIN 时车速已经是 V_AT_DMIN，不是 0。 */
static double ffOf(double v, double kff)
{
    double mag = (v < 0) ? -v : v;
    double cmd = kff * (mag - V_AT_DMIN);     // 不能叫 c：与循迹全局量 c 撞名
    if(cmd < 0.0) cmd = 0.0;
    return (v < 0) ? -cmd : cmd;
}

void updateSpeedPID()
{
    static unsigned long last = 0;
    static double KffL = KFF_INIT, KffR = KFF_INIT;   // 前馈增益（会自学习，见 ⑪）
#if SAT_ADAPT
    static uint8_t satCnt = 0;       // 顶满计数：停车时清零，避免跨启停累积
#endif
    unsigned long now = millis();
    if(now - last < CTRL_PERIOD_MS) return;

    double dt = (now - last) / 1000.0;
    if(dt > 0.05) dt = 0.05;      // ★ 任何抖动（loop 里偶发的一次长分支）都不该改变 PID 增益，
                                  //   否则一次 1ms 抖动会让 iTerm 一步冲高
    last = now;

    // ① 测速（乘上指令方向得到带符号的速度）
    double sL = readSpeedCmps(&encDtL, &encTmL) * dirL;
    double sR = readSpeedCmps(&encDtR, &encTmR) * dirR;

    // ② 一阶低通：滤掉码盘开孔误差带来的抖动
    spdL += SPEED_LP * (sL - spdL);
    spdR += SPEED_LP * (sR - spdR);

    /* ②' 换向那一拍：指令方向已翻，轮子却还在按原方向惯性转 ——
     *   测速乘上新符号会变成"已经在反向跑"，误差被算小、反向力矩被削弱。
     *   旧 PIDCtl 用 lastSign 做过这件事（清积分 + 本拍测速按 0），
     *   换成模态 PI 后必须保留，否则原地对转的前 100ms 会憋出一个大积分，
     *   转完直接冲过头 —— 表现就是"转弯甩尾、转完画龙"。
     *   ★ 这里读到的 dirL/dirR 是上一次 motorLR 写入的值，
     *     也就是"上一拍刚下的指令方向"，滞后一个控制周期（20ms），可接受。 */
    static int8_t prevDirL = 1, prevDirR = 1;
    if(dirL != prevDirL || dirR != prevDirR)
    {
        prevDirL = dirL;   prevDirR = dirR;
        sumI = diffI = 0;
        spdL = 0;  spdR = 0;        // 本拍测速作废
    }

    // ③ 航向修正：每 20ms 更新一次，供 qian() 直接取用
    headUpdate();

    /* ---- ④ 停机：清积分，交给电机层去走"刹车 → 滑行" ---- */
    if(tgtL == 0 && tgtR == 0)
    {
        sumI = diffI = 0;
#if SAT_ADAPT
        satCnt = 0;                  // 停车不累积顶满计数
#endif
        if(!motorsIdle) motorLR(0, 0);
        return;
    }

    /* ---- ⑤ 目标下限：死区里不可达的速度要抬到可达值 ----
     *   低于 V_MOVE_MIN(≈11.5cm/s) 的指令，无论给多大 PWM 都只会得到 11cm/s
     *   或 0 —— 让积分去追一个物理上不存在的点，它就会一路压到负饱和，
     *   表现出来就是"该慢的地方走走停停"。抬到下限是确定性的做法。 */
    double tL = tgtL, tR = tgtR;
    if(tL > 0 && tL <  V_MOVE_MIN) tL =  V_MOVE_MIN;
    if(tL < 0 && tL > -V_MOVE_MIN) tL = -V_MOVE_MIN;
    if(tR > 0 && tR <  V_MOVE_MIN) tR =  V_MOVE_MIN;
    if(tR < 0 && tR > -V_MOVE_MIN) tR = -V_MOVE_MIN;

    /* ---- ⑥ 模态误差：两轮之和 = 共模，两轮之差 = 差模 ---- */
    double eSum  = (tL + tR) - (spdL + spdR);
    double eDiff = (tL - tR) - (spdL - spdR);

    /* ---- ⑦ 前馈：逐轮给，天然吃掉"左右载频差 32 倍"的出力差异 ----
     *  ff 在闭环之外，所以它的自学习不会改变环路增益，不会引起振荡。
     *  ★ 用仿射式 Kff×(v − V_AT_DMIN)，不是正比式 Kff×v —— 原因见常量区说明：
     *    死区让 v(duty) 有台阶，正比式会在低速段过驱动 2 倍以上。 */
    double ffL = ffOf(tL, KffL);
    double ffR = ffOf(tR, KffR);

    /* ---- ⑧ 模态 PI ---- */
    double uSum  = KP_SUM  * eSum  + sumI;
    double uDiff = KP_DIFF * eDiff + diffI;

    double oL = ffL + (uSum + uDiff) * 0.5;
    double oR = ffR + (uSum - uDiff) * 0.5;

    /* ---- ⑨ 条件积分抗饱和 ----
     *   已经顶死还往同方向积，只会让退饱和变慢（"顶住一松手猛窜"）。 */
    bool satHi = (oL >  PWM_MAX - 2) || (oR >  PWM_MAX - 2);
    bool satLo = (oL < -(PWM_MAX - 2)) || (oR < -(PWM_MAX - 2));

    /* 刹车/全断期间【不积分】：此时轮子在做受迫减速，测速不代表出力，
     * 积进去的只是"我以为它该多快"的假误差，退出来要好几秒。 */
    bool inTransit = (motL.st == M_WAIT || motR.st == M_WAIT);

    if(!inTransit)
    {
        if(!(satHi && eSum  > 0) && !(satLo && eSum  < 0)) sumI  += KI_SUM  * eSum  * dt;
        if(!(satHi && eDiff > 0) && !(satLo && eDiff < 0)) diffI += KI_DIFF * eDiff * dt;
    }
    if(sumI  >  I_MAX) sumI  =  I_MAX;
    if(sumI  < -I_MAX) sumI  = -I_MAX;
    if(diffI >  I_MAX) diffI =  I_MAX;
    if(diffI < -I_MAX) diffI = -I_MAX;

    /* ---- ⑩ 限幅 → 死区映射（关键：不能再用硬钳位，见 dutyMap 注释） ---- */
    if(oL >  PWM_MAX) oL =  PWM_MAX;
    if(oL < -PWM_MAX) oL = -PWM_MAX;
    if(oR >  PWM_MAX) oR =  PWM_MAX;
    if(oR < -PWM_MAX) oR = -PWM_MAX;

    int oLi = dutyMap(oL, (oL >= 0) ? D_MIN_LF : D_MIN_LB);
    int oRi = dutyMap(oR, (oR >= 0) ? D_MIN_RF : D_MIN_RB);

    /* ---- ⑪ 前馈增益在线自学习：电池衰减 → 同速度要更大占空比 → Kff 变大 ----
     *   只在"稳态（误差 <3cm/s）+ 未饱和 + 速度够大（测速可信）"时学，
     *   τ ≈ 50 个控制周期 ≈ 1s，远慢于速度环，不会和 PI 抢活。 */
    if(!satHi && !satLo && eSum > -3.0 && eSum < 3.0)
    {
        double aL = (spdL < 0) ? -spdL : spdL;      // 幅值
        double aR = (spdR < 0) ? -spdR : spdR;
        double mL = (oL    < 0) ? -oL    : oL;      // 指令幅值
        double mR = (oR    < 0) ? -oR    : oR;
        /* 分母必须减掉台阶高度，否则学出来的是割线不是斜率；
         * 同时要求速度足够大，避免在 V_AT_DMIN 附近除出天文数字。 */
        if(aL > V_AT_DMIN + 8.0) KffL += 0.02 * (mL / (aL - V_AT_DMIN) - KffL);
        if(aR > V_AT_DMIN + 8.0) KffR += 0.02 * (mR / (aR - V_AT_DMIN) - KffR);
    }
    if(KffL < KFF_MIN) KffL = KFF_MIN;
    if(KffL > KFF_MAX) KffL = KFF_MAX;
    if(KffR < KFF_MIN) KffR = KFF_MIN;
    if(KffR > KFF_MAX) KffR = KFF_MAX;

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
// （stopLocked 在第 0 节统一声明了）

unsigned long lostStart   = 0;   // 开始丢线的时刻，0 = 没丢线

// ★★★ 到站停车锁（stopLocked）为什么必须有：
//   老代码把 STOP() 排在 move_tai() 后面，只是在末尾把引脚擦一遍，
//   下一轮 loop 开头 move_tai() 又立刻拉满 PWM —— 电机只被"闪"一下，
//   平均输出仍接近全速，看起来就是压根没停。停车必须做成锁。


/* ---- ★★★ 拐弯 / 丢线 兜底参数（治"直角拐点拐不过去"） ----
 *  问题现象：直角拐点上探头必然有一瞬间只看到 q==10，
 *           原来 zhi() 只要读到一次 10 就立刻放行去 qian()，对正被打断，
 *           车就直冲过拐点 → 直角拐不过去。
 *  修法：① 退出对正必须"连续稳定回到线上" BACK_ONLINE_MS；
 *        ② 三个探头同时离开黑线（丢线）不能停死，按最后看到的方向盲走找线，
 *           给 LOST_MS 的窗口，超时才停。
 */
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
    else if (a == 0 && b == 1 && c == 1)  nq = 11;   // ★ 从"1"拆出：分级误差需要区分
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
        else                   { L();   return; }
    }
    qian();                                          // 本来是直行 → 继续往前找
}

void lostLine()
{
    /* ★ 防御：lostStart==0 表示"计时器刚被清零、这一拍才开始丢线"。
     *   若照常算 el = millis() - 0 ≈ 几万毫秒，会直接掉进"超时恢复"分支 →
     *   ting() 或倒退，表现就是"莫名其妙顿一下/倒一下"。
     *   这里按"刚丢线"处理，正常走盲走窗口。 */
    if(lostStart == 0) { lostMotion(false); return; }

    unsigned long el = millis() - lostStart;

    /* ① 还在找线窗口内：按最后看到线的方向盲走 */
    if(el < LOST_MS) { lostMotion(el >= LOST_GRACE_MS); return; }

    /* ② 超时 → 有界恢复：停一下 → 倒退 → 再找
     *    用纯时间算术推进，不需要额外状态变量；
     *    一旦重新压到线（q != 0），move_tai() 会把 lostStart 清零，本流程自动复位。 */
    unsigned long t   = el - LOST_MS;
    unsigned long cyc = LOST_STOP_MS + LOST_REV_MS;
    /* ★ 用 unsigned long 而不是 uint8_t：丢线超过 256 轮（约 179s）时，
     *   uint8_t 会把商截回 0..255 并回绕，"试满 4 轮就放弃"突然失效，
     *   车又开始无限"停一下/倒一下"。 */
    if((t / cyc) >= LOST_RETRY) { ting(); return; }   // 试满仍找不到 → 真停，不乱窜

    unsigned long k   = t / cyc;

    unsigned long ph = t - (unsigned long)k * cyc;
    if(ph < LOST_STOP_MS) { ting(); return; }   // 先停稳
    hou();                                      // 再倒退把探头带回线上
}

/* ========================= 6.5 巡线模块（PD 差速，2026-10-20 重写） =========================
 *  旧版是"状态 → 全速动作"的 bang-bang：q 在 10/100/110/1 之间一跳，
 *  轮速目标就在 +30 与 -26 之间来回切 —— 蛇形、顿挫、稳定性差。
 *
 *  新版分两层：
 *    ① 细修（PD 差速）：三探头状态换算成【分级误差】，PD 输出叠加在基速上，
 *       小偏差平滑修正，轮速不再跳变；
 *    ② 粗修（升级 pivot）：单侧探头持续压线超过 PIVOT_AFTER_MS（急弯/大偏差），
 *       升级为原地对转 —— 保留旧版纠大偏差的能力。
 *  丢线恢复沿用久经验证的 lostLine()（盲走 + 停-退-找），不在本模块重写。
 *  调参顺序：先 LINE_KP（太小压不住、太大蛇形），再 LINE_KD（阻尼振荡就减它）。
 * ========================================================================== */
#define LINE_KP          6.0    // cm/s 每级误差（1 级 ≈ 一个探头间距）
#define LINE_KD          4.0    // 阻尼项：分级误差上 D 别给大，防跳变毛刺
#define LINE_STEER_MAX  18.0    // PD 输出上限 cm/s（PD 模式不让任何轮子倒转）
#define PIVOT_AFTER_MS  150UL   // ★ 250→150：单侧探头只压线 150ms 就升级为原地对转。
                                //   轮速差被死区钳着（慢轮最低 ≈11cm/s），
                                //   30cm/s 基速下最大差速只有 19cm/s —— 靠差速修大偏差
                                //   必然是"一边停转一边拖"，又慢又揉。早点升级成
                                //   "一正一反"的真对转，转向才干净利落。

int8_t lineErr = 0;              // 当前分级误差（负 = 线在左侧）
int8_t lineErrPrev = 0;
static unsigned long pivotMs = 0;

/* 三探头 → 分级误差。负 = 线偏左，正 = 线偏右。
 *  110(中+左) → -2   100(仅左) → -3   10(中) → 0
 *  011(中+右) → +2   001(仅右) → +3
 *  000 丢线、111/101 横线 → 不在这里处理（丢线走 lostLine，横线直行通过） */
static int8_t lineErrOf(int qq)
{
    switch(qq)
    {
        case 110: return -2;
        case 100: return -3;
        case 10:  return  0;
        case 11:  return  2;
        case 1:   return  3;
        default:  return  0;
    }
}

/* 每轮 loop 调用（仅 ST_LINE）。 */
void lineFollow()
{
    /* 丢线 → 交给久经验证的恢复流程（盲走 → 停-退-找 有界恢复） */
    if(q == 0) { lostLine(); return; }

    int8_t e = lineErrOf(q);

    double steer = LINE_KP * e + LINE_KD * (e - lineErrPrev);
    lineErrPrev = e;
    lineErr     = e;

    if(steer >  LINE_STEER_MAX) steer =  LINE_STEER_MAX;
    if(steer < -LINE_STEER_MAX) steer = -LINE_STEER_MAX;

    /* 急弯升级：单侧探头连续压线 → 原地对转（保留纠大偏差的旧能力） */
    if(e <= -3 || e >= 3)
    {
        if(pivotMs == 0) pivotMs = millis();
        if(millis() - pivotMs >= PIVOT_AFTER_MS)
        {
            pivotMs = millis();
            if(e < 0) L(); else R();
            return;
        }
    }
    else pivotMs = 0;

    straightMode = false;          // 巡线是闭环转向，不计入航向保持
    tgtL = baseSpeed + steer;      // steer>0 = 线偏右 → 左快右慢 = 右转（与旧 zhi 同号）
    tgtR = baseSpeed - steer;
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
        stageGotoNow(ST_LEAVE);        // 无间隔：本来就没停过，再顿 1s 纯浪费
        return;
    }

    /* ★ 是目标位：必须等 fan() + gui() 的最后一个舵机都到位。
     *   注意 actDone() 里有队列计数 —— fan 跑完时 gui 还排着，它仍然返回 false。 */
    if(!actDone()) { ting(); return; }

    /* 再停 DROP_TAIL_MS：舵机到位 ≠ 物块已经落进盒子（见常量处的说明）。
     * dropTailUntil 从"第一次发现动作全完成"开始计，只设一次。
     * ★ actDone() 返回 true 时，每个舵机步后那 400ms 停留已经走过一遍了，
     *   这里的 1s 是在"最后一步也停够"之后再叠加的落料稳定时间。 */
    if(dropTailUntil == 0) dropTailUntil = millis() + DROP_TAIL_MS;
    if(millis() < dropTailUntil) { ting(); return; }

    dropDone      = true;               // 动作走完 + 停留够了，才算投放完成
    dropTailUntil = 0;
    leaveStart    = millis();
    /* ★ 无间隔：DROP_TAIL_MS(1s) 已经把"物块落稳"等够了，
     *   再来 1s 阶段间隔就是白等 —— 这就是"投放完要愣一下才走"的来源。 */
    stageGotoNow(ST_LEAVE);
}

void move_tai()
{
    // ★ 每轮先把丢线计时器刷新一遍（放在最前面，保证不会留下过期的旧值）
    if(q == 0)  { if(lostStart == 0)   lostStart   = millis(); }
    else          lostStart   = 0;


    /* ★★★ 阶段间隔：上一阶段刚结束，先原地停 STAGE_GAP_MS 再进下一阶段。
     *   位置很关键 —— 放在阶段闸门之前、计时器维护之后：车停住，但循迹状态
     *   照常刷新；间隔结束时把两个计时器清零，否则它们在间隔期间一直累计，
     *   一进 ST_LINE 就被判"丢线已超时 600ms"，直接跳进倒退恢复流程。 */
    if(stageHold)
    {
        if(millis() < stageHold) { ting(); return; }
        stageHold   = 0;
        lostStart   = 0;
        /* 间隔期间车是停着的，若这时正挂着一个待消费的拐弯请求，
         * CORNER_REQ_MS(500ms) 会在间隔里过期 → 直角弯永远起不来。
         * 与 5cm 急停解锁处的处理保持一致：窗口从"真正能动"重新计。 */

        /* ★★★ 必须 return，不能继续往下走。
         *   lostStart 刚被清零，而下面的 switch(q) 里 q==0 会调 lostLine()，
         *   那里算 el = millis() - lostStart = millis() - 0 ≈ 几万毫秒 →
         *   直接判"丢线已超时"，ting() + 跳过整个恢复流程。
         *   表现出来就是：每次阶段间隔结束、刚要起步的那一拍，PWM 被清零一次、
         *   PID 积分被重置，于是"巡线时莫名其妙顿一下"。
         *   这一拍先回去，下一拍 lostStart 由开头的维护代码重新起算。 */
        return;
    }

    // ★★★ 第 0 道闸：任务阶段，优先级最高，压过循迹/超声波
    //     空闲：原地等 物块相机识别搬取物
    if(stage == ST_IDLE)
    {
        ting();
        return;
    }
    //     出发段：取件后直行 1.5s，这期间谁的账都不买
    if(stage == ST_PICK_FWD)
    {
        /* ★★★ 取件没走完，绝不起步。
         *   原来出发段从"任务启动"计时 1.5s，而 qu() 三步要 2.5~3.5s ——
         *   车早就开走了，爪子还在半路。落点因此每次都不同：电压高一点、
         *   摩擦小一点就取到了，反之就夹空，现场表现就是"有时可以有时不行"。
         *   现在改成：原地等 qu() 走完 → 再直行 1.5s。
         *   QU_DONE_TIMEOUT_MS 是兜底，动作真卡住也不许把车钉死在起点。 */
        if(quDoneMs == 0)
        {
            if(millis() - taskStartMs < QU_DONE_TIMEOUT_MS) { ting(); return; }
            quDoneMs = millis();                 // 兜底：强制认为取件结束
        }
        if(fwdUntil == 0) fwdUntil = quDoneMs + START_FWD_MS;

        if(millis() < fwdUntil) { qian(); return; }

        // ★ 到点就地切入巡线，不留给 loop() 收尾：
        //   否则这一拍会落到底下的循迹逻辑（stage 还是 PICK_FWD），
        //   出发段末尾多一次方向突变。
        stageGotoNow(ST_LINE);   /* ★ 无间隔：车正以 baseSpeed 前进，
                                  *   直接接巡线即可，没必要停 1s 再重新起步 */
        /* ★ 这里不再置 whiskerArmed：触须统一由"取件完成 +WHISKER_ARM_DELAY_MS"
         *   在 STOP() 里上岗，时间基准只有一处。 */

        /* 丢线/在线计时器由阶段间隔的出口（本函数开头 stageHold 那段）统一清零；
         * 那里才是车真正开始巡线的时刻，不会把 1s 间隔算成"丢线超时"。 */

        qian();
        return;
    }
/* ---- 到站（触须停车后）：读放置区相机 → 投放 或 离开继续找 ---- */
    if(stage == ST_DROP)
    {
        if(dropPhase == 0) { dropPhase = 1; dropPhaseMs = millis(); }

#if DROP_MODE == 0
        /* ---- 第 1 段（仅扫描模式）：向前爬 DROP_CENTER_MS，挪到盒子中间 ----
         *  超声波一进入 21cm 就判停，停的是盒子的【近沿】，而标志贴在盒子中间，
         *  这时候相机视野里根本没有标志 —— 这是"相机识别到了却不投"的另一半原因。
         *  ★ 爬行期间 straightMode=false：不把这几厘米记进航向偏差。 */
        if(dropPhase == 1)
        {
            if(millis() - dropPhaseMs < DROP_CENTER_MS)
            {
                /* ★ 低速爬行（不用 baseSpeed）：本板没有刹车，爬完还会再滑
                 *   车速×0.1~0.15s。用 30cm/s 爬完要滑 3~4cm，这一段是不可控的；
                 *   换成 15cm/s 后滑动只剩 1.5~2cm，总位移才能真正算准。 */
                straightMode = false;
                tgtL = tgtR = DROP_CRAWL_CMS;
                return;
            }
            dropPhase   = 2;                      // 居中结束 → 停稳扫描
            dropPhaseMs = millis();
        }
#endif
        /* 不扫描的模式：dropPhase 直接停在 1，dropPhaseMs 是进入 ST_DROP 的时刻，
         * 下面这句就是"停 300ms 就处置"，不白跑那 9cm。 */

        ting();                                   // 停稳车身

        if(millis() - dropPhaseMs < DROP_STOP_MS) return;

#if DROP_MODE == 0
        /* ★★★ 扫描读取：逐帧采样 + 同码连续确认
         *
         *  为什么不能"一进 ST_DROP 就取 visSite"：
         *    visSite 是全任务共享的，上一个盒子的号要等 SITE_TIMEOUT_MS 才失效，
         *    一到站就采信，会把上一个盒子的号当成这一个的。
         *  为什么也不能"只认晚于到站时间的那一帧"（上一版）：
         *    111.py 要 4 帧确认、400ms 心跳，停稳后第一帧合法码最坏 ~800ms 才来，
         *    而窗口只有 800ms —— 丢一帧就整盒跳过，也就是"相机明明识别到了却不投"。
         *  现在的做法：
         *    · 每来一个新帧（visSiteMs 变了）就采一次样，含 <00>；
         *    · 同一码连续 2 帧才采信（排除单帧误读、邻盒串码）；
         *    · 窗口到期还没凑够 2 帧，就退一步认这 1 帧 —— 宁可多投一次，
         *      也不要因为偶发漏帧把整个盒子跳过；
         *    · 窗口内一帧合法码都没有（链路断了），才算"不是目标位"离开。 */
        /* ★★★ 窗口内多数表决（替代原来的"同码连续 2 帧"）
         *   111.py 的识别本身会抖（33 → 00 → 33），要求"连续"等于要求它不抖 ——
         *   实测很难凑齐，窗口一到就整盒跳过，这是"识别到却不投"的第二个原因。
         *   改成投票：窗口内出现的非零码各记一票，最后取票最多的那个。 */
        if(visSiteMs != dropFrameMs)                  // 有比"到站那一刻"更新的帧
        {
            dropFrameMs = visSiteMs;

            if(visSite >= 1 && visSite <= 3)
            {
                if(dropVote[visSite] < 250) dropVote[visSite]++;
                if(dropVoteTot       < 250) dropVoteTot++;
                dropCand    = visSite;
                dropCandCnt = dropVote[visSite];      // 排障输出里的 N
            }
        }

        uint8_t best = 0, bestN = 0;                  // 得票最多的码与票数
        for(uint8_t i = 1; i <= 3; i++)
            if(dropVote[i] > bestN) { bestN = dropVote[i]; best = i; }

        bool expired = (millis() - dropPhaseMs > SETTLE_MS + DROP_WAIT_MS);

        if(dropCode == 0 && bestN >= 2)              dropCode = best;  // 2 票即采信
        if(dropCode == 0 && expired && bestN >= 1)   dropCode = best;  // 到期退让认 1 票

        if(dropCode >= 1)                             // 本次到站已拿到区域号
        {
            dropAct(dropCode == f);
            return;
        }

        if(expired)                                   // 一帧合法码都没等到
        {
            leaveStart = millis();
            stageGotoNow(ST_LEAVE);                   // 无间隔：车都停了 2.8s 了
        }
        return;
#else
        /* ---- 非 0 模式：不依赖放置区相机 ---- */
        dropAct((DROP_MODE == 2) ? true : (stopCnt == f));
        return;
#endif
    }
    /* ---- 离开当前物料放置区：继续巡线前进，直到触须松开 ----
     *  ★ 用"测距变远"而不是固定时长：盒宽与车速都会变，固定时长要么
     *    没走出去（刚回巡线又停）、要么一下跨过两个盒（漏掉目标）。
     *  ★ 离开期间走巡线（lineFollow），不是盲直行 —— 保持贴线，
     *    出区后无缝衔接 ST_LINE。 */
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

        /* ★ 离开判定（触须版）：触须松开（回到高电平）= 已离开当前物料放置区。
         *   停车时触须被盒壁压住，车前进越过盒子远沿后弹簧弹回 → 松开。 */
        bool left = !whiskerOn();

        if(el >= LEAVE_MIN_MS && (left || el >= LEAVE_MAX_MS))
        {
            stopLocked = false;
            if(dropDone)
            {
                stageGoto(ST_IDLE);        // 已投放 → 任务完成（带 1s 间隔再回空闲）
                whiskerArmed = false;
                taskRearm  = true;         // 强制重新武装取件（见 taskRearm 声明处）
            }
            else
            {
                /* ★ 没投成 → 回巡线继续找下一个盒。无间隔切换：
                 *   ST_LEAVE 里本来就在跑 lineFollow()，车没停过，不需要间隔。 */
                lostStart   = 0;
                stageGotoNow(ST_LINE);
            }
            return;
        }
        lineFollow();                     // 继续巡线（不是盲直行）
        return;
    }

    // ★★★ 到站停车锁：优先级高于对正和循迹
    if(stopLocked)
    {
        ting();
        return;
    }

    /* 巡线（PD 差速）：丢线交给 lostLine()，在线交给分级误差 PD */
    lineFollow();

}

/* ========================= 7.5 到站判定（触须开关版） =========================
 *  上岗条件（唯一一处）：qu() 走完之后再等 WHISKER_ARM_DELAY_MS(7.5s)，
 *    由本函数把 whiskerArmed 置 true。起点周围有相邻物块/器材腿，
 *    取件 + 起步段不允许触须参与决策。
 *  到站判据：巡线中触须被盒壁压住（连续 WHISKER_CONFIRM 个样本）
 *            → 停车 + 进入 ST_DROP，由 move_tai() 在那里读放置区相机，
 *              决定"投放"还是"离开继续找下一个盒"。
 *
 *  ★ 只在 ST_LINE 阶段生效：进入 ST_DROP 后由该阶段自己 ting()。
 *  ★ 连续 N 个样本：触须沿盒壁滑行时弹簧会颤，单帧抖动用计数滤掉。
 *  ★ "离开当前物料区"的判定不在本函数：ST_LEAVE 以"触须松开"为出口。
 * ========================================================== */
void STOP()
{
    static uint8_t cnt = 0;

    if(!whiskerArmed && quDoneMs && millis() - quDoneMs >= WHISKER_ARM_DELAY_MS)
        whiskerArmed = true;
    if(!whiskerArmed) { cnt = 0; return; }
    if(stage != ST_LINE) { cnt = 0; return; }

    if(whiskerOn())
    {
        if(cnt < WHISKER_CONFIRM) cnt++;
    }
    else cnt = 0;

    if(cnt >= WHISKER_CONFIRM)
    {
        cnt        = 0;
        stopLocked = true;
        dropSent   = false;
        dropCode   = 0;
        dropCand    = 0;
        dropCandCnt = 0;
        dropVote[1] = dropVote[2] = dropVote[3] = 0;   // 上一个盒子的票不能带过来
        dropVoteTot = 0;
        dropFrameMs = visSiteMs;
        if(stopCnt < 255) stopCnt++;
        if(arriveCnt < 65535) arriveCnt++;
        stageGoto(ST_DROP);
        dropPhase   = 0;
        dropPhaseMs = 0;
        headE    = 0;
        headCorr = 0;
        ting();
    }
}

/* ========================= 7.9 串口排障输出（D1/TX） ==========================
 *  D0(RX) 接相机，D1(TX) 完全空闲（K230_CMD_ENABLE=0），所以直接开串口助手
 *  看状态，不影响接收。波特率必须与 Serial.begin 一致：9600。
 *  每 500ms 一行、约 35 字节，占用 <4ms/500ms，可忽略。
 *
 *  字段： S=阶段  V=区域号  #=合法放置帧数  E=校验失败/断帧数
 *         F=物块种类  C=已锁存码  N=候选得票  U=触须已上岗  a=到站次数  q=循迹状态
 *
 *  ★ E 与 # 的关系是判断"链路坏了"还是"相机没识别到"的关键：
 *      # 在涨、V=0   → 链路好，是相机报"无目标"（标志不在视野 / 停车位置偏）
 *      E 在涨、# 不涨 → 链路坏（干扰 / 波特率不一致 / 断线），与识别无关
 *      两者都不涨   → 线上一帧都没有：查菊花链转发与接线
 *  ★ 比赛时把 DEBUG_SERIAL 改 0，这段代码完全不参与编译。
 * ============================================================================ */
#define DEBUG_SERIAL 1

static void dbgLine()
{
#if DEBUG_SERIAL
    static unsigned long last = 0;
    if(millis() - last < 500UL) return;      // 500ms 一行，别把 TX 塞满
    last = millis();

    Serial.print(F("S")); Serial.print(stage);
    Serial.print(F(" V")); Serial.print(visSite);
    Serial.print(F(" #")); Serial.print(siteOkCnt);
    Serial.print(F(" E")); Serial.print(linkBadCnt);   // 校验失败/断帧：涨而 # 不涨 = 线路问题
    Serial.print(F(" F")); Serial.print(f);
    Serial.print(F(" C")); Serial.print(dropCode);
    Serial.print(F(" N")); Serial.print(dropCandCnt);
    Serial.print(F(" U")); Serial.print(whiskerArmed);
    Serial.print(F(" a")); Serial.print(arriveCnt);
    Serial.print(F(" q")); Serial.print(q);
    Serial.write('\n');
#endif
}

/* ========================= 8. setup / loop ========================= */

/* ========================= 9. 电机标定工具（CALIB_MODE=1 时才编译） =========================
 *  为什么要工具：四个 D_MIN 靠"改代码→烧录→看轮子"试出来要烧 4~8 次，
 *  而且人眼判断"转没转"比编码器差一个量级。这里自动扫一遍，一次烧录拿全部数据。
 *
 *  输出可直接填回的常量：
 *      D_MIN_LF / D_MIN_LB / D_MIN_RF / D_MIN_RB
 *      V_AT_DMIN（取左右平均）、KFF_INIT（取左右平均）
 * ========================================================================================== */
#if CALIB_MODE

static long calibSnap(volatile long *cnt)        // 32 位量，ISR 在写，必须原子快照
{
    uint8_t sreg = SREG;
    cli();
    long v = *cnt;                               // 形参不能叫 c：与循迹全局量 c 撞名
    SREG = sreg;
    return v;
}

static void calibDrive(uint8_t pinF, uint8_t pinB, uint8_t duty)
{
    if(duty == 0) { analogWrite(pinF, 0); analogWrite(pinB, 0); return; }
    analogWrite(pinB, 0);                        // 先清零，再给占空（与正式代码同一铁律）
    analogWrite(pinF, duty);
}

/* 从小到大扫占空比，第一次出现 >=2 个编码器边沿就认为"起来了"。
 * 取 2 个边沿而不是 1 个：单边沿可能是一个抖动毛刺。 */
static uint8_t calibMinDuty(uint8_t pinF, uint8_t pinB, volatile long *cnt)
{
    for(uint16_t d = 26; d <= 140; d += 2)
    {
        long c0 = calibSnap(cnt);
        calibDrive(pinF, pinB, (uint8_t)d);
        delay(60);
        long c1 = calibSnap(cnt);
        calibDrive(pinF, pinB, 0);

        long e = c1 - c0; if(e < 0) e = -e;
        if(e >= 2)
        {
            delay(150);
            return (uint8_t)d;
        }
        delay(90);                               // 让轮子彻底停住再试下一个
    }
    return 0;                                    // 0 = 扫到底都没转，接线/供电有问题
}

/* 固定占空比下的稳态速度（cm/s） */
static double calibSpeed(uint8_t pinF, uint8_t pinB, volatile long *cnt, uint8_t duty)
{
    calibDrive(pinF, pinB, duty);
    delay(350);                                  // 先跑到稳态
    long c0 = calibSnap(cnt);
    delay(600);
    long c1 = calibSnap(cnt);
    calibDrive(pinF, pinB, 0);

    long e = c1 - c0; if(e < 0) e = -e;
    return (double)e * CM_PER_EDGE / 0.6;
}

static void calibRun()
{
    Serial.println(F(""));
    Serial.println(F("==== CALIB START (car must be lifted) ===="));

    dirL = 1;  dirR = 1;                         // 让编码器计数恒为正向，便于取绝对值

    Serial.println(F("-- sweep L forward --"));
    uint8_t mLF = calibMinDuty(6,  5,  &encCntL); delay(300);
    Serial.print(F("D_MIN_LF = ")); Serial.println(mLF);

    Serial.println(F("-- sweep L reverse --"));
    uint8_t mLB = calibMinDuty(5,  6,  &encCntL); delay(300);
    Serial.print(F("D_MIN_LB = ")); Serial.println(mLB);

    Serial.println(F("-- sweep R forward --"));
    uint8_t mRF = calibMinDuty(10, 9,  &encCntR); delay(300);
    Serial.print(F("D_MIN_RF = ")); Serial.println(mRF);

    Serial.println(F("-- sweep R reverse --"));
    uint8_t mRB = calibMinDuty(9,  10, &encCntR); delay(300);
    Serial.print(F("D_MIN_RB = ")); Serial.println(mRB);

    /* 两个工作点求斜率 a = Δv/Δduty，再反推 V_AT_DMIN 与 Kff。
     * 用 150/210 而不是 90/120：低端受死区和静摩擦影响大，线性度差。 */
    Serial.println(F("-- gain points --"));
    double vL1 = calibSpeed(6,  5, &encCntL, 150); delay(300);
    double vL2 = calibSpeed(6,  5, &encCntL, 210); delay(300);
    double vR1 = calibSpeed(10, 9, &encCntR, 150); delay(300);
    double vR2 = calibSpeed(10, 9, &encCntR, 210); delay(300);

    Serial.print(F("VL150 = ")); Serial.print((int)(vL1 * 10));
    Serial.print(F("  VL210 = ")); Serial.print((int)(vL2 * 10));
    Serial.print(F("  VR150 = ")); Serial.print((int)(vR1 * 10));
    Serial.print(F("  VR210 = ")); Serial.println((int)(vR2 * 10));
    Serial.println(F("(上面四个是 cm/s 的 10 倍，例如 300 表示 30.0 cm/s)"));

    if(mLF && mRF && vL2 > vL1 && vR2 > vR1)
    {
        double aL = (vL2 - vL1) / 60.0;          // cm/s per duty
        double aR = (vR2 - vR1) / 60.0;
        /* ★ 用【正转】死区做基准，因为上面两个速度点都是正转测的。
         *   若取 min(正,反)，一旦反转没起来（mLB=0）外推就会跑飞。 */
        double dL = mLF;
        double dR = mRF;

        double vAtL = vL1 - aL * (150.0 - dL);   // 外推到 duty = D_MIN
        double vAtR = vR1 - aR * (150.0 - dR);

        double kffL = 255.0 / (aL * (255.0 - dL));
        double kffR = 255.0 / (aR * (255.0 - dR));

        Serial.print(F("A_L = "));   Serial.print((int)(aL   * 1000));
        Serial.print(F("  A_R = ")); Serial.println((int)(aR * 1000));
        Serial.print(F("V_AT_DMIN_L = ")); Serial.print((int)(vAtL * 10));
        Serial.print(F("  V_AT_DMIN_R = ")); Serial.println((int)(vAtR * 10));
        Serial.print(F("KFF_L = ")); Serial.print((int)(kffL * 100));
        Serial.print(F("  KFF_R = ")); Serial.println((int)(kffR * 100));
        Serial.println(F("(A 是 1000 倍，V_AT_DMIN 是 10 倍，KFF 是 100 倍)"));

        Serial.print(F(">> V_AT_DMIN = "));
        Serial.println((int)((vAtL + vAtR) * 5));         // 平均值的 10 倍
        Serial.print(F(">> KFF_INIT  = "));
        Serial.println((int)((kffL + kffR) * 50));        // 平均值的 100 倍
    }
    else
    {
        Serial.println(F("!! 增益点无效：某个轮子扫到 140 都没转，或两点速度没递增"));
        Serial.println(F("!! 先查接线/供电/编码器，再看 D_MIN 是否为 0"));
    }

    Serial.println(F("==== CALIB DONE ===="));
    Serial.println(F("把上面的整数除以倍率，填回常量区，然后把 CALIB_MODE 改回 0"));

    for(;;) { }                                  // 停在这里，不进主循环
}
#endif   /* CALIB_MODE */

void setup()
{
    /* 硬件串口 D0：物块相机直连，放置区相机的帧由它透传过来，
     * 全程只有一个波特率（VISION_BAUD），任务期间不再切速率。 */
    Serial.begin(VISION_BAUD);

    /* ★ 开机唤醒物块相机。
     *   等 300ms：K230 上电比 Arduino 慢得多，太早发的命令它会漏掉。 */
    delay(300);
    k230Send(CMD_WAKE_BLOCK);

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

    /* 触须：开关量输入，触发=低。上拉保证"松开"态是确定的高电平，
     * 不会因悬空而随机读 0 造成误判到站（详见文件头触须说明）。 */
    pinMode(WHISKER_PIN, INPUT_PULLUP);

    // ★ 编码器：A2/A3 走引脚变化中断
    //   A2 = PC2 = PCINT10，A3 = PC3 = PCINT11 → 同属 PCINT1 组
    //   必须用【PCMSK1 + PCIE1 + PCINT1_vect】。
    //   （PCMSK2 里的 PCINT16~23 对应的是 PD0~PD7，即 D0~D7，不是 A2/A3）
    //   A4/A5 = PC4/PC5 = PCINT12/13，虽同在 PCMSK1，但不在掩码里，
    //   循迹信号的变化不会触发本中断。
    pinMode(A2, INPUT_PULLUP);
    pinMode(A3, INPUT_PULLUP);
    delay(2);            // 等上拉稳定，再去读初始电平，免得第一个假跳变被计入
    pcLast  = PINC;
    // 掩码只放行 A2=PCINT10、A3=PCINT11 两路编码器（触须是普通数字输入，不占中断）
    PCMSK1 |= _BV(PCINT10) | _BV(PCINT11);
    PCICR  |= _BV(PCIE1);

    /* 电机进入 COAST（两脚全低）；两个模态积分由全局初始化保证为 0 */
    motorLR(0, 0);

    servoInit();         // 4 路舵机：上电即到初始位 D4=90 D7=90 D8=270 D12=180

#if CALIB_MODE
    calibRun();          // ★ 标定模式：不返回，跑完停在那里等复位
#endif
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
    else       lineFollow();
    updateSpeedPID();
    return;
#endif

    // ② 双 K230 视觉接收 + 任务调度
    pollVisionAndQueue();

    // ③ 到站判定（触须开关：被盒壁压住 = 到站）
    STOP();

    // ④ 状态机：只决定"目标速度"，不直接写 PWM
    du();
    move_tai();

    // ⑤ 机构动作序列推进（qu/fan/gui/kai，非阻塞）
    actRun();

    // ⑥ 速度环：测速 → 模态解耦 PI → 死区映射（内部按 20ms 节拍执行）
    updateSpeedPID();

    // ⑥' 电机状态机推进
    //     刹车 60ms / 死区 25ms / Boost 60ms 都要按毫秒走；
    //     只在 20ms 的控制节拍里推的话，25ms 的死区会被量成 40ms，
    //     换向总耗时变成 60+40+60，手感发滞。所以单独每轮推一次。
    motorService();

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
        stageHold  = 0;               // 兜底复位：别让残留的阶段间隔挡住下一个任务
        whiskerArmed = false;
        stopLocked = false;

        /* ★ 故意【不】清 visTask：
         *   清了会让 taskArm 立刻复位，而相机 100ms 后又报同一个物体 →
         *   马上重起一个一模一样的任务，一次卡死就变成无限循环跑车。
         *   保留 visTask 时 taskArm 仍是 false，必须等"物体被拿走"（相机发 0,0）
         *   才解锁重试，符合"同一物体只取一次"的语义。 */
    }

    // ⑧ 排障输出（DEBUG_SERIAL=0 时整段不编译）
    dbgLine();
}
