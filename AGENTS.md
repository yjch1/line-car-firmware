# AGENTS.md —— 给 AI 编码代理的项目说明

> opencode / Claude Code / Codex 等会自动读取本文件。**先读完这一页再动手**，
> 能省掉你自己摸索环境的时间（这些坑都已经被踩过一遍了）。

## 这是什么

ATmega328P（Arduino Uno / BigFish 扩展板）**循迹搬运小车**固件。
仓库里维护**两条独立固件线**，判据不同、有意分叉：

| 线 | 入口文件 | 到站判据 |
|---|---|---|
| 超声波 | `ultrasonic/ultrasonic.ino` | HC-SR04 测距 ≤ `STOP_CM`，连续 N 个样本确认 |
| 触须 | `whisker/whisker.ino` | 机械触须（whisker）压到盒壁 |

两条线**共享底盘代码**（双轮编码器 PID、四路舵机、视觉串口、任务状态机）。
改公共逻辑时：要么两边都改，要么先在一条线上改完再合过去 —— 不要两边各改一版然后指望它们一致。

## 目录

```
ultrasonic/   超声波线（当前主线）
whisker/      触须线
archive/      历史快照，只读，用来对比参数演进（4 个文件，勿删）
tools/build.ps1   编译检查脚本
straight/ bt_servo/ servotest/   早期独立测试 sketch，与主线无关
*.md          项目文档（注意时效性，见文末）
```

## 编译验证（改完必须跑）

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\build.ps1
```

期望输出两行 `通过   0 错误 0 告警` + 退出码 0。单目标：末尾加 `ultrasonic` 或 `whisker`。

当前基线（用作回归参考）：

```
ultrasonic  通过   0 错误 0 告警   text 11922  data 75  bss 295
whisker     通过   0 错误 0 告警   text 11739  data 53  bss 245
```

ATmega328P：flash 32KB、**SRAM 仅 2KB**。静态占用约 370B，余量充足，但别在栈上加大的局部数组。

## 硬性红线

1. **不要在仓库根目录创建任何 `2.ino`。** 历史上两个工具各改一份同名文件互相覆盖，丢过一次工作。
2. `.ino` 必须与所在文件夹同名，否则 Arduino IDE 打不开。
3. 所有文本文件保持 **LF**。仓库有 `.gitattributes` 强制，**不要删除或修改它** —— 本机系统级 `core.autocrlf=true`，没有它 git 每次 checkout 都会把 `.ino` 改成 CRLF，文件字节数就变了。
4. `.ps1` 里若含中文，**文件必须带 UTF-8 BOM**。Windows PowerShell 5.1 默认按 GBK 读脚本，没有 BOM 会把中文注释解成乱码并报语法错误。
5. **不要 `#include <Servo.h>`**：该库独占 Timer1，而右电机 D9/D10 正是 Timer1 的 PWM 通道，引入后右轮直接不转。舵机由 Timer2 软件 PWM 驱动，接口是 `servoWrite(信号线, 角度)`。
6. 本机 `npm` / `pnpm` 是 `.ps1` 包装，PowerShell 执行策略禁止直接运行；需要时调 `npm.cmd`。

## 硬件接线（已实测确认，不要凭 archive/ 里的旧备份推断）

| 项 | 接线 |
|---|---|
| 循迹 | 左 A4 / 中 D3 / 右 A5（`TRACK_MIRROR=0` 即 A4 在左侧） |
| 左轮 | Motor-1：正转 D6 / 反转 D5 |
| 右轮 | Motor-0：正转 D10 / 反转 D9 |
| 编码器 | 左 A3(PCINT11) / 右 A2(PCINT10)，均 `INPUT_PULLUP` |
| 超声波 | **TRIG D15(A1) / ECHO D14(A0)** |
| 舵机 | D4 / D7 / D8 / D12（270°，Timer2 软件 PWM，COMPB 2.5kHz） |
| 视觉 | 物块相机 → D0（9600）；放置区相机菊花链经物块相机转发到 D0 |

⚠️ `archive/` 和 `2.ino.bak`、`.before_*` 等历史文件里全是 `TRIGPIN 14 / ECHOPIN 15`，**那是改线之前**的接法。照它们"修正"会让 `distance` 恒 999，到站/急停/直角弯全部静默失效。

定时器分配：Timer0 = micros/millis + 左电机 PWM（不可动）；Timer1 = 右电机 PWM（不可动）；Timer2 = 舵机；PCINT1 = 编码器 + 超声波 ECHO（A0 = PC0 = PCINT8）。

## 读代码请省 token（**这一节直接决定开销**）

- `ultrasonic.ino` / `whisker.ino` 各约 **2700 行 / 13 万字符**，整篇读一次就吃掉大量上下文预算。
  **先 grep 定位符号，再局部读**：`STOP(`、`updateUltrasonic`、`move_tai`、`ULTRA_ARM_DELAY_MS`、
  `ultraArmed`、`stage` —— 然后用 `offset`/`limit` 只取那 80~150 行。
- `archive/` 里 4 个 `.ino` 合计约 **52 万字符，不要整篇读**。要对比参数演进就 `grep '#define'`，
  要对比某段逻辑就 `grep -A/-B` 取上下文。
- `项目理解文档.md` 20KB，只在需要背景时读，且**结论一律以代码为准**（它描述的版本已经过时）。
- 需要"能不能编过"的结论时，直接跑 `tools\build.ps1`，不要自己拼 avr-g++ 参数 —— 那会白烧一轮。
- 改完只重编受影响的单条线：`... -File tools\build.ps1 ultrasonic`。

## 当前未解问题（优先处理）

**超声波线：测距小于 22cm 时不判停。**

已知参数（`ultrasonic/ultrasonic.ino`）：`STOP_CM 20.0`、`STOP_CONFIRM 2`、`EMERG_CM 5.0`、
`EMERG_CONFIRM 3`、`ULTRA_ARM_DELAY_MS 7500`、`SETTLE_MS 1000`、`DROP_WAIT_MS 1800`。

判停链路：`updateUltrasonic()`（PCINT 量回波宽度写 `distance`）→ `STOP()` 计票 → `stage = ST_DROP` → `move_tai()` 停车。

已排除：
- 引脚接反（15/14 已与实车核对）。
- `zhiLocked` 从未置 true 的死代码问题（后续版本已修）。

**重点怀疑**：旧版本里 `distance == 999`（无回波）会落进 `distance > STOP_CM + STOP_HYST_CM` 分支把确认票清零，
于是漏一拍就永远攒不满 `STOP_CONFIRM` —— 现象正是"测距明明小于 22cm 却不停"。请先核对当前版本是否还残留这个语义
（"测不到" ≠ "障碍很远"，999 应当既不计票也不清票）。

**另一个必须先确认的未知项**：`ULTRA_ARM_DELAY_MS 7500` × 30cm/s ≈ **225cm 盲区**。
起点到首个投放盒若短于这个距离，那段路上的盒子根本不会判停。先问人确认场地实际距离，别急着改代码。

诊断手段：文件里若有 `DEBUG_TELEMETRY` 开关就打开它（只输出数字与逗号，不会误触发相机命令）；
串口监视器**只看不发** —— PC 的 TX 也挂在 D0 上，一打字就会被当成视觉帧。

## 文档时效性（重要）

`项目理解文档.md`、`2.ino 分析报告.md`、`全量扫描报告_20261020.md` 写于 2026-10-17~20，基于**更早的版本**：

- 它们断言"`zhiLocked` 从未置 true、整套对正是死代码" → **已修复**。
- 它们描述的"黑块 `hei()` 锁定直行"机制 → 超声波线里**已删除**（`q==111` 被并入丢线处理）。
- 它们引用的行号**全部失效**。

**以代码为准，文档只当背景资料。** 不要照文档里的行号去改代码。

## Git 工作流

- 改之前先 `git add -A; git commit -m "..."`（或确认工作区干净），改完再提交；小步提交。
- **动手前先看 `git status`**：如果工作区不干净且不是你改的，先停下问人，不要覆盖别人的改动。
- 不要 `git push --force`，不要 `git reset --hard` 覆盖历史。
- 提交前必须跑通 `tools\build.ps1`，没编译验证过的改动不要提交。

## 不要做的事（前车之鉴）

- 不要凭"历史备份里是这么写的"去改常量或引脚 —— 备份里的接线是改线前的。
- 不要在没跑编译的情况下声称"修好了"。
- 不要试图把两条固件线改成一样：判据不同是设计，不是 bug。
- 不要删 `archive/`：那是唯一能对比参数演进过程的记录。
