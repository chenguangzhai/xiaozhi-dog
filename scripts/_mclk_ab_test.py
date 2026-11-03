#!/usr/bin/env python3
"""MCLK 串扰 A/B 复测工具。

用途：改完硬件（抽线、串 33Ω、加 guard trace）后，跑这个脚本拿到
      CAP0~CAP3 四个驱动档位在 6.144MHz 下的 I2C 应答率，
      和改动前的基线对比，判断措施有没有打到主战场。

判读：
  CAP3 明显上升（如 20% -> 50%+）  -> 措施有效，打到了耦合路径
  CAP3 基本不变（仍 ~20%）        -> 措施没打到主战场，换下一个
  CAP1 已明显低于 CAP0            -> 软件规避余量为零，必须改硬件

用法：
  python scripts/_mclk_ab_test.py            # 默认 3 次冷启动
  python scripts/_mclk_ab_test.py 5          # 5 次
  python scripts/_mclk_ab_test.py 5 COM9     # 指定端口
"""

import re
import sys
import time

import serial

PORT = sys.argv[2] if len(sys.argv) > 2 else "COM9"
ROUNDS = int(sys.argv[1]) if len(sys.argv) > 1 else 3
BOOT_SECONDS = 10.0
BAUD = 115200

CAP_RE = re.compile(r"MCLKSWEEP\[(CAP\d)\]: \d+ Hz -> 0x18 (\d+)/(\d+)")
DIAG_RE = re.compile(r"DIAG probe\(after I2S\) 0x18: (\d+)/100 ok")
INIT_OK = "Open codec device OK"
INIT_FAIL = "Failed to create Es8311AudioCodec"


def capture_one():
    """硬复位一次，抓一段日志，返回解析结果。"""
    with serial.Serial(PORT, BAUD, timeout=0.2, dsrdtr=False, rtscts=False) as p:
        p.dtr = False
        p.rts = False
        time.sleep(0.3)
        p.rts = True
        time.sleep(0.15)
        p.rts = False

        buf = bytearray()
        deadline = time.time() + BOOT_SECONDS
        while time.time() < deadline:
            chunk = p.read(4096)
            if chunk:
                buf.extend(chunk)

    text = buf.decode("utf-8", errors="replace")
    caps = {}
    for name, ok, total in CAP_RE.findall(text):
        caps[name] = 100.0 * int(ok) / int(total)

    diag = DIAG_RE.search(text)
    return {
        "caps": caps,
        "diag": int(diag.group(1)) if diag else None,
        "init_ok": INIT_OK in text,
        "init_fail": INIT_FAIL in text,
        "reboots": text.count("ESP-ROM"),
    }


def main():
    print(f"端口 {PORT}，{ROUNDS} 次冷启动，每次 {BOOT_SECONDS:.0f}s\n")
    rows = []
    for i in range(1, ROUNDS + 1):
        try:
            r = capture_one()
        except serial.SerialException as e:
            print(f"第 {i} 次失败：串口打不开（{e}）")
            print("请先关掉自己的串口监视器 / idf.py monitor。")
            return 1
        rows.append(r)
        caps = "  ".join(f"{k}={r['caps'][k]:.0f}%" for k in sorted(r["caps"]))
        state = "codec OK" if r["init_ok"] else ("codec FAIL" if r["init_fail"] else "未抓到")
        print(f"第 {i} 次  {caps}  | DIAG={r['diag']}/100  | {state}  | 复位 {r['reboots']} 次")
        if r["reboots"] != 1:
            print("   警告：复位次数不为 1，镜像可能损坏，需重烧 app 分区")

    print("\n" + "=" * 62)
    print(f"{'档位':<8}{'平均':>8}{'最低':>8}{'最高':>8}   备注")
    print("-" * 62)
    for cap in ("CAP0", "CAP1", "CAP2", "CAP3"):
        vals = [r["caps"][cap] for r in rows if cap in r["caps"]]
        if not vals:
            print(f"{cap:<8}{'--':>8}")
            continue
        note = ""
        if cap == "CAP0":
            note = "实际运行用的档位"
        elif cap == "CAP3":
            note = "耦合最强的档位，用来看改善幅度"
        print(f"{cap:<8}{sum(vals) / len(vals):7.0f}%{min(vals):7.0f}%{max(vals):7.0f}%   {note}")
    print("=" * 62)
    print("判读：CAP3 涨上去 = 措施有效；CAP1 已掉 = 软件规避余量为零")
    return 0


if __name__ == "__main__":
    sys.exit(main())
