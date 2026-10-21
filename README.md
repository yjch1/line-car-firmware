# 零件分拣车固件 · 两条线

本目录同时维护**两支独立固件**。2026-10-20 曾出现过"两个工具各改一份同名 `2.ino`、互相覆盖"的事故，
所以这里把两条线拆成两个互不干扰的入口 —— **不要再在根目录放任何 `2.ino`**。

| 线 | 入口 | 到站判据 |
|---|---|---|
| 超声波 | `ultrasonic/ultrasonic.ino` | HC-SR04 测距 ≤ `STOP_CM`，连续若干样本确认 |
| 触须   | `whisker/whisker.ino`      | 机械触须（whisker）压到盒壁，边沿/电平确认 |

## 用法

- Arduino IDE 直接打开对应的 `.ino` 文件即可（文件夹名与文件名一致，IDE 不会报"sketch must be in a folder"）。
- 两条线共享同一套底盘代码：电机 PID、四路舵机、视觉串口、任务状态机。
  **改公共逻辑时要么两边都改，要么先在一条线上改完再合过去** —— 不要两边各改一版然后指望它们一致。

## 历史快照

`archive/` 是 2026-10-20 前后的备份，原样保留、不参与编译，仅供回溯比对：

| 文件 | 说明 |
|---|---|
| `2_20261020_0329_超声波.ino` | 早期超声波版（STOP_CM 24 / ULTRA_ARM_DELAY 5000） |
| `2_20261020_1818_超声波.ino` | 中期超声波版（STOP_CM 21 / 7500） |
| `2_20261020_2040_触须.ino`   | 触须版重构前 |
| `2_20261020_2200_触须.ino`   | 触须版清死代码前（还残留超声波宏） |

另有两份源文件在 `C:\Users\MECHREUO\Desktop\chu\`（未删除，仅复制过来）：

- `2_20261020_2003_改前_超声波版.ino` → 已作为 `ultrasonic/ultrasonic.ino`
- `2_1792470586580060902.ino`        → 已作为 `whisker/whisker.ino`

云端另有一份交接目录（dir_id `BrsTGXOEcKlP`），含 `111(1).py`、`444.py`、交接说明等，
判断云端是否最新**不能看 dir_list 的 size**（它是过期元数据），必须下载后比 md5。

## 版本管理

整个目录是 git 仓库（`main` 分支），已提交：

```
a9b540a chore: 补齐 git 骨架（.gitignore / .gitattributes 锁 LF，日志文件不入库）
ee271ac refactor: 拆分为两条固件线（ultrasonic / whisker）+ archive 历史快照
dbc12b4 baseline: 初始状态 + 历史快照与文档
```

常用命令：

```powershell
git log --oneline          # 看历史
git status --short         # 看谁改了文件（另一个工具改了也会显示）
git diff                   # 看工作区具体改了哪几行
git diff --cached          # 看已暂存、还没提交的内容
git checkout -- <文件>     # 丢弃某个文件的改动，退回上次提交
git revert <提交号>        # 撤销某次提交（保留历史，安全）
```

**改代码前先 `git add -A; git commit -m "..."`**，这样任何一次"越改越烂"都能一条命令退回。

### 两个必须保留的配置文件

| 文件 | 作用 |
|---|---|
| `.gitattributes` | **把所有文本文件锁成 LF**。本机系统级 `core.autocrlf=true`，没有它，git 每次 checkout 都会把 `.ino` 从 LF 改成 CRLF —— 文件字节数变化，和 Arduino 存档、历史快照全部 diff 变红，`git checkout` 也就不再等于"精确恢复"。 |
| `.gitignore` | 忽略编译产物（`*.hex/*.elf/*.map/*.o`）、工具日志（`.knowcode/`）、临时目录、Windows 垃圾文件。 |

### 推到 GitHub

仓库名建议别叫 `2`，先改名（git 不受影响）：

```powershell
Rename-Item 'D:\GitHub\2' 'D:\GitHub\line-car-firmware'
```

然后在 GitHub 网页上新建一个**空仓库**（不要勾 README/gitignore），再回来：

```powershell
cd D:\GitHub\line-car-firmware
git remote add origin https://github.com/<你的用户名>/<仓库名>.git
git branch -M main
git push -u origin main
```

首次推送会弹出浏览器登录（本机装了 Git Credential Manager，不用手搓 token）。
若 GitHub 访问慢，可换 SSH（`git remote set-url origin git@github.com:...`）或推到极狐 / Gitee。


## 其它目录

- `straight/`、`bt_servo/`、`servotest/` —— 早期验证用的独立测试 sketch，与两条主线无关。
- `serial_vote.py` —— PC 端串口帧投票工具，用来核实物块相机上报的 `1,x` 是否稳定。
- `*.md` —— 项目理解文档、分析报告、扫描报告（注意：写于 2026-10-17~20，**部分结论已过期**，
  例如"`zhiLocked` 是死代码"在后续版本里已修）。
