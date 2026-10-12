#!/usr/bin/env python3
"""Pack robot-eye EAF files into a single flash image for the `eyeeaf` partition.

Usage: gen_eye_eaf.py <assets_EAF_dir> <output_bin>

Image layout (little-endian):
  offset 0:  magic "EYEA" (4B), entry count u32
  then per entry: alias[32] (utf-8, NUL padded), data offset u32, data size u32
  then raw EAF blobs, 4-byte aligned.

Keep the alias list in sync with eye_emotions.h and main/CMakeLists.txt.
"""
import os
import struct
import sys

ALIASES = [
    ("L-shangxin-1.eaf", "eye_shangxin_L.eaf"),
    ("R-shangxin-1.eaf", "eye_shangxin_R.eaf"),
    ("L-danzuoyanbi-1.eaf", "eye_danzuoyanbi_L.eaf"),
    ("R-danzuoyanbi-1.eaf", "eye_danzuoyanbi_R.eaf"),
    ("L-shengqi-1.eaf", "eye_shengqi_L.eaf"),
    ("R-shengqi-1.eaf", "eye_shengqi_R.eaf"),
    ("Public-zuoyougundong-1.eaf", "eye_zuoyougundong.eaf"),
    ("Public-weixiao-1.eaf", "eye_weixiao.eaf"),
    ("Public-wuyu-1.eaf", "eye_wuyu.eaf"),
    ("Public-zhayan-1.eaf", "eye_zhayan.eaf"),
    ("Public-biyan-1.eaf", "eye_biyan.eaf"),
]

ENTRY_FMT = "<32sII"
ENTRY_SIZE = struct.calcsize(ENTRY_FMT)
HEADER_FMT = "<4sI"


def main() -> int:
    src_dir, out_path = sys.argv[1], sys.argv[2]
    blobs = []
    for src_name, _ in ALIASES:
        src = os.path.join(src_dir, src_name)
        if not os.path.isfile(src):
            print(f"gen_eye_eaf: missing {src}", file=sys.stderr)
            return 1
        with open(src, "rb") as f:
            blobs.append(f.read())

    header_len = struct.calcsize(HEADER_FMT) + ENTRY_SIZE * len(ALIASES)
    offset = header_len
    table = b""
    body = b""
    for (_, alias), blob in zip(ALIASES, blobs):
        alias_b = alias.encode("utf-8")
        assert len(alias_b) < 32, alias
        table += struct.pack(ENTRY_FMT, alias_b, offset, len(blob))
        body += blob
        pad = (-len(blob)) % 4
        body += b"\xff" * pad
        offset += len(blob) + pad

    with open(out_path, "wb") as f:
        f.write(struct.pack(HEADER_FMT, b"EYEA", len(ALIASES)))
        f.write(table)
        f.write(body)
    print(f"gen_eye_eaf: {len(ALIASES)} entries, {offset} bytes -> {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
