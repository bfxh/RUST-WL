#!/usr/bin/env python3
"""同屏对比渲染器：读 `out/compare.bin`（gold-sample 转储）→ 左右并排 GIF。

用法： python scripts/render_compare.py
依赖： Pillow。产物：`out/compare.gif`（vxl-phys 左 / rapier 右，同场景同相机）。

路径安全：**不接受外部路径参数**——输入/输出是固定常量（`out/` 下），
每次读写在 sink 处用 `_resolve()` 做 realpath 包含校验（拒绝越出 out/）。
"""
import struct
import os
import math
from PIL import Image, ImageDraw, ImageFont

PANEL_W, PANEL_H = 470, 300
W, H = PANEL_W * 2, PANEL_H
FPS = 30

ROOT = os.path.realpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
OUT_ROOT = os.path.join(ROOT, "out")
SRC_NAME = "compare.bin"
DST_NAME = "compare.gif"


def _resolve(rel_name):
    if os.path.isabs(rel_name) or ".." in rel_name.replace("\\", "/").split("/"):
        raise SystemExit(f"拒绝越界路径：{rel_name}")
    full = os.path.realpath(os.path.join(OUT_ROOT, rel_name))
    root = os.path.realpath(OUT_ROOT)
    if not (full == root or full.startswith(root + os.sep)):
        raise SystemExit(f"越出 out/：{rel_name}")
    return full


def parse():
    path = _resolve(SRC_NAME)
    with open(path, "rb") as f:
        assert f.read(4) == b"VXLC", "magic 不符"
        tpf, = struct.unpack("<I", f.read(4))
        half, = struct.unpack("<f", f.read(4))
        frames = []
        while True:
            head = f.read(12)
            if len(head) < 12:
                break
            tick, vms, rms = struct.unpack("<Iff", head)
            n, = struct.unpack("<I", f.read(4))
            vxl = [struct.unpack("<7f", f.read(28)) for _ in range(n)]
            rap = [struct.unpack("<7f", f.read(28)) for _ in range(n)]
            frames.append((tick, vms, rms, vxl, rap))
    return tpf, half, frames


def quat_mat(q):
    x, y, z, w = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def mv(m, v):
    return tuple(m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2] for i in range(3))


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def norm(a):
    l = math.sqrt(dot(a, a)) or 1.0
    return (a[0] / l, a[1] / l, a[2] / l)


def make_cam(panel_x):
    eye = (3.4, 2.7, 3.6)
    target = (0.0, 1.35, 0.0)
    fwd = norm(sub(target, eye))
    right = norm(cross(fwd, (0.0, 1.0, 0.0)))
    up = cross(right, fwd)
    f = (PANEL_H * 0.52) / math.tan(math.radians(40.0) * 0.5)

    def project(p):
        d = sub(p, eye)
        x, y, z = dot(d, right), dot(d, up), dot(d, fwd)
        if z <= 0.05:
            return None
        return (panel_x + PANEL_W * 0.5 + x * f / z, PANEL_H * 0.62 - y * f / z, z)

    return project


LIGHT = norm((-0.4, 1.0, 0.3))


def shade(base, n):
    l = max(0.0, dot(n, LIGHT)) * 0.7 + 0.32
    return tuple(int(min(255.0, c * 255.0 * l)) for c in base)


def draw_scene(dr, project, poses, half, tint):
    # 地板网格（同几何：半 20×0.5×20，顶面 y=0）
    grid = []
    for i in range(-6, 7):
        for j in (-6, 6):
            for a, b in (((i, j), (j, j)),):
                pass
    prims = []
    for gx in range(-6, 7):
        for a, b in (((gx, -6), (gx, 6)), ((-6, gx), (6, gx))):
            p0, p1 = project((float(a[0]), 0.0, float(a[1]))), project((float(b[0]), 0.0, float(b[1])))
            if p0 and p1:
                prims.append((900.0, "line", (p0, p1), (58, 64, 74)))
    for (x, y, z, qx, qy, qz, qw) in poses:
        m = quat_mat((qx, qy, qz, qw))
        cs = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    cs.append(add((x, y, z), mv(m, (sx * half, sy * half, sz * half))))
        for fq in ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4),
                   (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)):
            tri = [cs[i] for i in fq]
            scr = [project(p) for p in tri]
            if any(s is None for s in scr):
                continue
            n = norm(cross(sub(tri[1], tri[0]), sub(tri[2], tri[0])))
            prims.append((sum(s[2] for s in scr) / 4, "poly",
                          [(s[0], s[1]) for s in scr], shade(tint, n)))
    prims.sort(key=lambda t: -t[0])
    for (_z, tag, a, col) in prims:
        if tag == "poly":
            dr.polygon(a, fill=col)
        else:
            dr.line([a[0][:2], a[1][:2]], fill=col, width=1)


def main():
    tpf, half, frames = parse()
    try:
        font = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 12)
        fontb = ImageFont.truetype("C:/Windows/Fonts/arialbd.ttf", 15)
    except Exception:
        font = fontb = ImageFont.load_default()
    proj_l = make_cam(0)
    proj_r = make_cam(PANEL_W)
    out = []
    av_v = av_r = 0.0
    for fi, (tick, vms, rms, vxl, rap) in enumerate(frames):
        av_v += vms
        av_r += rms
        im = Image.new("RGB", (W, H), (10, 12, 16))
        dr = ImageDraw.Draw(im)
        dr.rectangle([0, 0, W, 24], fill=(18, 22, 30))
        dr.text((8, 5), "vxl-phys（本仓）", font=fontb, fill=(120, 220, 255))
        dr.text((PANEL_W + 8, 5), "rapier 0.35（默认 TGS-Soft）", font=fontb, fill=(255, 190, 120))
        dr.line([(PANEL_W, 0), (PANEL_W, H)], fill=(60, 68, 80), width=2)
        draw_scene(dr, proj_l, vxl, half, (0.86, 0.60, 0.28))
        draw_scene(dr, proj_r, rap, half, (0.45, 0.70, 0.95))
        for px, ms, name in ((8, vms, "vxl"), (PANEL_W + 8, rms, "rap")):
            dr.text((px, H - 42), f"{name}: {ms:.2f} ms/tick  ⇒  {1000/max(ms,1e-6):.0f} FPS",
                    font=font, fill=(215, 225, 240))
            dr.text((px, H - 26), f"{len(vxl)} 盒 · 同场景 · 同 tick {tick}",
                    font=font, fill=(150, 165, 185))
        dr.text((8, H - 62), f"累计均: vxl {av_v/(fi+1):.2f} ms | rapier {av_r/(fi+1):.2f} ms",
                font=font, fill=(160, 230, 180))
        out.append(im)
        if fi % 20 == 0:
            print(f"  {fi}/{len(frames)}")
    out[0].save(_resolve(DST_NAME), save_all=True, append_images=out[1:],
                duration=int(1000 / FPS), loop=0, optimize=False)
    print(f"完成：out/{DST_NAME}（{len(out)} 帧 @ {FPS} fps）")


if __name__ == "__main__":
    main()
