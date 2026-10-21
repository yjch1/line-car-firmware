#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""K230 视觉帧 · 2 秒严格窗口众数投票

从串口读取 K230 上报的帧，严格采集 2.0 秒，统计
    1,1 / 1,2 / 1,3
各出现多少次，取出现次数最多的那一类作为有效输出。

帧格式（与 4_识别并串口上报v4.py 一致）：ASCII "src,val\\n"，9600 8N1
    识别到 → "1,1" / "1,2" / "1,3"
    无目标 → "0,0"
    兼容 INCLUDE_COORD=True 的 "1,1,320,240"

用法
    python serial_vote.py                  # 自动找串口
    python serial_vote.py -p COM3          # 指定端口
    python serial_vote.py -w 2.0           # 指定窗口（秒）
    python serial_vote.py -l               # 列出可用串口
    python serial_vote.py --demo           # 无硬件自检
"""

import argparse
import sys
import time

TAG = {1: "1,1", 2: "1,2", 3: "1,3"}


class FrameVoter:
    """窗口内的帧计数 + 众数选举。

    只做三件事：按行切帧、分类计数、选出众数。
    平局规则：计数相同则取"最近一次出现更晚"的那类（视觉上更可信）。
    """

    __slots__ = ("counts", "last", "zeros", "bad", "frames", "_partial")

    def __init__(self):
        self.counts = {1: 0, 2: 0, 3: 0}
        self.last = {1: -1.0, 2: -1.0, 3: -1.0}
        self.zeros = 0          # "0,0" 帧（视觉端明确报无目标）
        self.bad = 0            # 非法帧 / 半帧
        self.frames = 0         # 有效帧（1,1 / 1,2 / 1,3）
        self._partial = b""     # 跨包残留，等下一个换行

    def feed_bytes(self, chunk, t):
        """喂入原始字节，按 '\\n' 切帧；t = 这批数据的到达时刻。"""
        self._partial += chunk
        while True:
            i = self._partial.find(b"\n")
            if i < 0:
                if len(self._partial) > 64:   # 长期无换行 → 丢弃，防止无界增长
                    self._partial = b""
                return
            line = self._partial[:i]
            self._partial = self._partial[i + 1:]
            self._frame(line, t)

    def _frame(self, raw, t):
        s = raw.strip()
        if not s:
            return
        parts = s.split(b",")
        if len(parts) < 2:                    # 没有逗号 → 不是本协议
            self.bad += 1
            return
        try:
            int(parts[0])                     # 首字段必须是数字（脚本恒为 1）
            val = int(parts[1])
        except ValueError:
            self.bad += 1
            return

        if val == 0:                          # "0,0" 无目标
            self.zeros += 1
            return
        if val not in (1, 2, 3):              # 越界值不采信
            self.bad += 1
            return

        self.frames += 1
        self.counts[val] += 1
        self.last[val] = t

    def result(self):
        """返回 (值 或 None, 说明文字)。"""
        if self.frames == 0:
            return None, "2 秒内没有有效帧"
        ranked = sorted(self.counts, key=lambda k: (-self.counts[k], -self.last[k]))
        best = ranked[0]
        n = self.counts[best]
        tied = [k for k in (1, 2, 3) if self.counts[k] == n]
        if len(tied) > 1:
            return best, "计数平局 %s，取最近出现的 %s" % (
                " / ".join(TAG[k] for k in tied), TAG[best])
        return best, ""


def collect(port, baud, window):
    """严格采集 window 秒，返回 (投票器, 实测耗时)。"""
    import serial

    ser = serial.Serial(port, baud, timeout=0.002)
    ser.reset_input_buffer()

    v = FrameVoter()
    t0 = time.monotonic()
    deadline = t0 + window

    while True:
        now = time.monotonic()
        if now >= deadline:
            break
        # 快到期时缩短等待，避免最后一次 read 越过窗口边界
        ser.timeout = min(0.01, max(deadline - now, 0.001))
        chunk = ser.read(ser.in_waiting or 1)
        t_now = time.monotonic()
        if chunk and t_now <= deadline:       # 只在窗口内到达的数据才计入
            v.feed_bytes(chunk, t_now)

    elapsed = time.monotonic() - t0
    ser.close()
    return v, elapsed


def report(v, elapsed, window):
    print()
    print("=" * 52)
    print(" 采集窗口   : %.3f s  (目标 %.1f s)" % (elapsed, window))
    print(" 有效帧     : %d" % v.frames)
    print(" 0,0 帧     : %d  (视觉端报无目标)" % v.zeros)
    print(" 非法/半帧  : %d" % v.bad)
    print("-" * 52)
    for k in (1, 2, 3):
        n = v.counts[k]
        pct = (100.0 * n / v.frames) if v.frames else 0.0
        print(" %s : %4d 次  %5.1f%%  %s" % (TAG[k], n, pct, "#" * int(pct / 4)))
    print("-" * 52)

    best, note = v.result()
    if best is None:
        print(" 有效输出   : 无目标")
        if v.zeros:
            print("              (收到 %d 帧 0,0)" % v.zeros)
    else:
        pct = 100.0 * v.counts[best] / v.frames
        print(" 有效输出   : %s   (%d/%d 帧, %.1f%%)"
              % (TAG[best], v.counts[best], v.frames, pct))
        if note:
            print(" 说明       : %s" % note)
    print("=" * 52)
    return best


def pick_port():
    """优先选 USB 转串口（CH340 / CP210x / FTDI）。"""
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    if not ports:
        return None
    for p in ports:
        d = (p.description or "").lower()
        if any(s in d for s in ("ch340", "cp210", "ftdi", "usb-serial", "uart")):
            return p.device
    return ports[0].device


def demo():
    """无硬件自检：验证计数、众数、平局、无帧、非法帧、跨包半帧。"""
    cases = [
        ("1,1 占多数", [b"1,1\n"] * 5 + [b"1,2\n"] * 2 + [b"0,0\n"], 1),
        ("1,3 占多数", [b"1,3\n"] * 4 + [b"1,1\n"], 3),
        ("1,2 占多数", [b"1,2\n"] * 3, 2),
        ("全是 0,0", [b"0,0\n"] * 3, None),
        ("无任何帧", [], None),
        ("非法帧忽略", [b"abc\n", b"1,\n", b"1,11\n", b"1,2\n"], 2),
        ("坐标帧兼容", [b"1,1,320,240\n", b"1,3\n", b"1,3\n"], 3),
        ("半帧跨包", [b"1,", b"1\n"], 1),
        ("CRLF 结尾", [b"1,2\r\n", b"1,2\r\n"], 2),
    ]

    ok = True
    print("=== 自检：计数与投票 ===")
    for name, chunks, want in cases:
        v = FrameVoter()
        for i, c in enumerate(chunks):
            v.feed_bytes(c, i * 0.01)
        got, _ = v.result()
        flag = "PASS" if got == want else "FAIL"
        if got != want:
            ok = False
        print(" [%s] %-12s 期望=%-4s 实际=%-4s 有效=%d 非法=%d 0,0=%d"
              % (flag, name, want, got, v.frames, v.bad, v.zeros))

    print("\n=== 自检：平局取最近出现的 ===")
    v = FrameVoter()
    v.feed_bytes(b"1,1\n1,2\n1,1\n1,2\n", 0.1)
    v.last[2] = 0.9                       # 人为让 1,2 更晚出现
    got, note = v.result()
    flag = "PASS" if got == 2 else "FAIL"
    if got != 2:
        ok = False
    print(" [%s] 期望=2 实际=%s  %s" % (flag, got, note))

    print("\n=== 自检：窗口严格性（模拟 0.3s 窗口 + 定时喂帧）===")
    v = FrameVoter()
    t0 = time.monotonic()
    deadline = t0 + 0.3
    while time.monotonic() < deadline:
        v.feed_bytes(b"1,1\n", time.monotonic())
        time.sleep(0.02)
    el = time.monotonic() - t0
    good = abs(el - 0.3) < 0.03 and v.frames > 0
    if not good:
        ok = False
    print(" [%s] 实测窗口 %.3fs (目标 0.300s)，收到 %d 帧"
          % ("PASS" if good else "FAIL", el, v.frames))

    print("\n自检结果： %s" % ("全部通过" if ok else "有失败项"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(
        description="串口帧 2 秒窗口众数投票（1,1 / 1,2 / 1,3）")
    ap.add_argument("-p", "--port", help="串口名，如 COM3")
    ap.add_argument("-b", "--baud", type=int, default=9600, help="波特率（默认 9600）")
    ap.add_argument("-w", "--window", type=float, default=2.0, help="窗口秒数（默认 2.0）")
    ap.add_argument("-l", "--list", action="store_true", help="列出可用串口")
    ap.add_argument("--demo", action="store_true", help="无硬件自检")
    a = ap.parse_args()

    if a.demo:
        return demo()

    import serial
    from serial.tools import list_ports

    if a.list:
        ports = list(list_ports.comports())
        if not ports:
            print("没有可用串口")
            return 2
        for p in ports:
            print("%-10s %s" % (p.device, p.description))
        return 0

    port = a.port or pick_port()
    if not port:
        print("找不到串口，用 -p 指定（-l 可列出）")
        return 2

    print("端口 %s  %d 8N1  窗口 %.2fs  采集中..." % (port, a.baud, a.window))
    v, elapsed = collect(port, a.baud, a.window)
    best = report(v, elapsed, a.window)
    return 0 if best else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n已中断")
        sys.exit(130)
    except Exception as e:
        print("错误：%s" % e)
        sys.exit(1)
