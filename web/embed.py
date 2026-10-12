#!/usr/bin/env python3
"""把 web/dist/index.html 转成 C raw-string 常量，替换 ld14_web.cc 的 kPage。

用法:  python web/embed.py
备份:  原 kPage 改名为 kPageLegacy（旧简易页兜底，可手动切回）。
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent
DIST = ROOT / "dist" / "index.html"
CC = ROOT.parent / "main" / "boards" / "xiaozhi-pcb-arm" / "ld14_web.cc"

html = DIST.read_text(encoding="utf-8")
print(f"dist/index.html: {len(html)} bytes")
if len(html) > 380 * 1024:
    print("WARN: 单文件超过 380KB，请检查 app 分区余量", file=sys.stderr)

src = CC.read_text(encoding="utf-8")
m = re.search(r'const char kPage\[\] = R"HTML\(.*?\n\)HTML";', src, re.S)
if not m:
    print("kPage not found", file=sys.stderr)
    sys.exit(1)
legacy = m.group(0).replace("kPage[]", "kPageLegacy[]", 1)
new_const = 'const char kPage[] = R"HTML(' + "\n" + html + "\n" + ')HTML";'
CC.write_text(src[: m.start()] + legacy + "\n\n" + new_const + src[m.end():],
              encoding="utf-8")
print("kPage replaced, legacy kept as kPageLegacy")
