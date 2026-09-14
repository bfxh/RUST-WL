#!/usr/bin/env python3
"""演示渲染器：读 `out/showcase.bin` 转储 → Pillow 软光栅 → `out/showcase.gif`。

用法： python scripts/render_demo.py [--stride N]
依赖： Pillow（纯 Python 侧；引擎 crate 仍零外部依赖）。

路径安全：**不接受外部路径参数**——输入/输出是固定常量（`out/` 下），
且每次读写在 sink 处用 `_resolve()` 做 realpath 包含校验（拒绝越出 out/）。
"""
import struct
import sys
import math
import os
from PIL import Image, ImageDraw, ImageFont

W, H = 560, 315
FPS = 30
BG_TOP = (22, 26, 34)
BG_BOT = (10, 12, 16)

ROOT = os.path.realpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
OUT_ROOT = os.path.join(ROOT, "out")
SRC_NAME = "showcase.bin"
DST_NAME = "showcase.gif"
FRAMES_DIR = "frames"


def _resolve(rel_name):
    """把 out/ 下的相对名解析成绝对路径；realpath 包含校验，越界即拒。"""
    if os.path.isabs(rel_name) or ".." in rel_name.replace("\\", "/").split("/"):
        raise SystemExit(f"拒绝越界路径：{rel_name}")
    full = os.path.realpath(os.path.join(OUT_ROOT, rel_name))
    root = os.path.realpath(OUT_ROOT)
    if not (full == root or full.startswith(root + os.sep)):
        raise SystemExit(f"越出 out/：{rel_name}")
    return full


# ---------------------------------------------------------------- 转储解析
def parse():
    path = _resolve(SRC_NAME)
    with open(path, "rb") as f:
        assert f.read(4) == b"VXLD", "magic 不符"
        ver, ticks, tpf, dt = struct.unpack("<IIIf", f.read(16))
        nx, ny, nz = struct.unpack("<III", f.read(12))
        ox, oy, oz, step = struct.unpack("<4f", f.read(16))
        (nsplat,) = struct.unpack("<I", f.read(4))
        splats = []
        for _ in range(nsplat):
            c = struct.unpack("<3f", f.read(12))
            s = struct.unpack("<3f", f.read(12))
            (op,) = struct.unpack("<f", f.read(4))
            col = struct.unpack("<3f", f.read(12))
            splats.append((c, s, op, col))
        frames = []
        nbits = (nx * ny * nz + 7) // 8
        while True:
            head = f.read(8)
            if len(head) < 8:
                break
            tick, ms = struct.unpack("<If", head)
            (n,) = struct.unpack("<I", f.read(4))
            bodies = []
            for _ in range(n):
                kind, awake = struct.unpack("<BB", f.read(2))
                pos = struct.unpack("<3f", f.read(12))
                rot = struct.unpack("<4f", f.read(16))
                half, pts = None, None
                if kind == 0:
                    half = struct.unpack("<3f", f.read(12))
                elif kind == 1:
                    half = struct.unpack("<1f", f.read(4))
                elif kind == 2:
                    (cnt,) = struct.unpack("<I", f.read(4))
                    pts = [struct.unpack("<3f", f.read(12)) for _ in range(cnt)]
                bodies.append((kind, awake, pos, rot, half, pts))
            bits = f.read(nbits)
            frames.append((tick, ms, bodies, bits))
    return dict(
        ticks=ticks, tpf=tpf, dt=dt, dims=(nx, ny, nz),
        origin=(ox, oy, oz), step=step, splats=splats, frames=frames,
    )


# ---------------------------------------------------------------- 数学
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


class Cam:
    def __init__(self, eye, target, fov=42.0):
        self.eye = eye
        f = norm(sub(target, eye))
        up = (0.0, 1.0, 0.0)
        r = norm(cross(f, up))
        u = cross(r, f)
        self.mat = (r, u, f)
        self.fov = math.radians(fov)
        self.f = (H * 0.5) / math.tan(self.fov * 0.5)

    def project(self, p):
        d = sub(p, self.eye)
        x = dot(d, self.mat[0])
        y = dot(d, self.mat[1])
        z = dot(d, self.mat[2])
        if z <= 0.05:
            return None
        return (W * 0.5 + x * self.f / z, H * 0.5 - y * self.f / z, z)


LIGHT = norm((-0.45, 1.0, 0.35))


def shade(color, n):
    """color 为 0..1 线性色；按法线点光后转 0..255。"""
    l = max(0.0, dot(n, LIGHT)) * 0.75 + 0.28
    return tuple(int(min(255.0, c * 255.0 * l)) for c in color)


def hull2d(pts):
    p = sorted(set(pts))
    if len(p) < 3:
        return p

    def cr(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lo = []
    for q in p:
        while len(lo) >= 2 and cr(lo[-2], lo[-1], q) <= 0:
            lo.pop()
        lo.append(q)
    up = []
    for q in reversed(p):
        while len(up) >= 2 and cr(up[-2], up[-1], q) <= 0:
            up.pop()
        up.append(q)
    return lo[:-1] + up[:-1]


# ---------------------------------------------------------------- 主渲染
def main():
    stride = 1
    if "--stride" in sys.argv:
        stride = max(1, int(sys.argv[sys.argv.index("--stride") + 1]))
    src = _resolve(SRC_NAME)
    if not os.path.exists(src):
        raise SystemExit(f"找不到转储：{src}（先在仓库根跑 showcase 示例）")
    data = parse()
    nx, ny, nz = data["dims"]
    ox, oy, oz = data["origin"]
    step = data["step"]
    splats = data["splats"]
    frames = data["frames"][::stride]

    try:
        font = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 13)
        fontb = ImageFont.truetype("C:/Windows/Fonts/arialbd.ttf", 15)
    except Exception:
        font = ImageFont.load_default()
        fontb = font

    frames_dir = _resolve(FRAMES_DIR)
    os.makedirs(frames_dir, exist_ok=True)
    out_frames = []
    total_ms = 0.0
    vox_cache = (None, [])
    for fi, (tick, ms, bodies, bits) in enumerate(frames):
        total_ms += ms
        ang = math.radians(28.0 + 90.0 * fi / max(1, len(frames)))
        cam = Cam((13.0 * math.cos(ang), 8.5, 13.0 * math.sin(ang)), (0.0, 1.6, 0.0))
        im = Image.new("RGB", (W, H), BG_BOT)
        dr = ImageDraw.Draw(im, "RGBA")
        for y in range(H):
            t = y / H
            dr.line([(0, y), (W, y)],
                    fill=tuple(int(BG_TOP[i] * (1 - t) + BG_BOT[i] * t) for i in range(3)))
        prims = []

        # ---- 体素面（占用位不变 ⇒ 复用上一帧面表）----
        if bits != vox_cache[0]:

            def has(ix, iy, iz):
                idx = ix * ny * nz + iy * nz + iz
                return (bits[idx // 8] >> (idx % 8)) & 1

            faces = []
            for iy in range(ny):
                for iz in range(nz):
                    ix = 0
                    while ix < nx:
                        if not has(ix, iy, iz):
                            ix += 1
                            continue
                        jx = ix
                        while jx + 1 < nx and has(jx + 1, iy, iz):
                            jx += 1
                        top = iy + 1 >= ny or any(
                            not has(xx, iy + 1, iz) for xx in range(ix, jx + 1))
                        if top:
                            y = oy + (iy + 1) * step
                            x0, x1 = ox + ix * step, ox + (jx + 1) * step
                            z0, z1 = oz + iz * step, oz + (iz + 1) * step
                            faces.append((((x0, y, z0), (x1, y, z0), (x1, y, z1), (x0, y, z1)),
                                          (0.62, 0.66, 0.72)))
                        ix = jx + 1
            for ix in range(nx):
                for iy in range(ny):
                    for iz in range(nz):
                        if not has(ix, iy, iz):
                            continue
                        x0, x1 = ox + ix * step, ox + (ix + 1) * step
                        y0, y1 = oy + iy * step, oy + (iy + 1) * step
                        z0, z1 = oz + iz * step, oz + (iz + 1) * step
                        for dx, dz, quad in (
                            (1, 0, ((x1, y0, z0), (x1, y0, z1), (x1, y1, z1), (x1, y1, z0))),
                            (-1, 0, ((x0, y0, z1), (x0, y0, z0), (x0, y1, z0), (x0, y1, z1))),
                            (0, 1, ((x1, y0, z1), (x0, y0, z1), (x0, y1, z1), (x1, y1, z1))),
                            (0, -1, ((x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0))),
                        ):
                            jx, jz = ix + dx, iz + dz
                            if 0 <= jx < nx and 0 <= jz < nz and has(jx, iy, jz):
                                continue
                            faces.append((quad, (0.52, 0.56, 0.62)))
            vox_cache = (bits, faces)
        for quad, col in vox_cache[1]:
            scr = [cam.project(p) for p in quad]
            if any(s is None for s in scr):
                continue
            n = norm(cross(sub(quad[1], quad[0]), sub(quad[2], quad[0])))
            prims.append((sum(s[2] for s in scr) / 4, "poly",
                          [(s[0], s[1]) for s in scr], shade(col, n)))

        # ---- 喷溅（软光斑）----
        for (c, s, op, col) in splats:
            scr = cam.project(c)
            if scr is None:
                continue
            z = scr[2]
            r = max(3.0, s[0] * 2.6 * cam.f / z)
            prims.append((z, "splat", (scr[0], scr[1], r, col, op), None))

        # ---- 刚体 ----
        for (kind, awake, pos, rot, half, pts) in bodies:
            m = quat_mat(rot)
            if kind == 0:
                hx, hy, hz = half
                cs = []
                for sx in (-1, 1):
                    for sy in (-1, 1):
                        for sz in (-1, 1):
                            cs.append(add(pos, mv(m, (sx * hx, sy * hy, sz * hz))))
                for fq in ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4),
                           (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)):
                    tri = [cs[i] for i in fq]
                    scr = [cam.project(p) for p in tri]
                    if any(s is None for s in scr):
                        continue
                    n = norm(cross(sub(tri[1], tri[0]), sub(tri[2], tri[0])))
                    base = (0.86, 0.62, 0.30) if awake else (0.52, 0.56, 0.60)
                    prims.append((sum(s[2] for s in scr) / 4, "poly",
                                  [(s[0], s[1]) for s in scr], shade(base, n)))
            elif kind == 1:
                scr = cam.project(pos)
                if scr:
                    z = scr[2]
                    r = half[0] * cam.f / z
                    sc = (0.85, 0.30, 0.25) if awake else (0.5, 0.52, 0.55)
                    prims.append((z, "sphere", (scr[0], scr[1], r),
                                  tuple(int(255.0 * c) for c in sc)))
            elif kind == 2 and pts:
                scr = [cam.project(add(pos, mv(m, p))) for p in pts]
                scr = [s for s in scr if s]
                if len(scr) >= 3:
                    poly = hull2d([(s[0], s[1]) for s in scr])
                    if len(poly) >= 3:
                        hc = (0.30, 0.80, 0.55) if awake else (0.40, 0.55, 0.48)
                        prims.append((sum(s[2] for s in scr) / len(scr), "hull", poly,
                                      tuple(int(255.0 * c) for c in hc)))

        # ---- 接地阴影 ----
        shadow_y = oy + 3 * step + 0.01
        for (kind, awake, pos, rot, half, pts) in bodies:
            if kind == 0 and half and len(half) == 3:
                prims.append((900.0, "shadow",
                              (pos[0], pos[2], max(half[0], half[2]) * 1.15, shadow_y), None))
            elif kind == 1 and half:
                prims.append((900.0, "shadow",
                              (pos[0], pos[2], half[0] * 1.15, shadow_y), None))

        prims.sort(key=lambda t: -t[0])
        for (_z, tag, a, col) in prims:
            if tag == "poly":
                dr.polygon(a, fill=col)
            elif tag == "hull":
                dr.polygon(a, fill=col, outline=(16, 40, 28))
            elif tag == "sphere":
                px, py, r = a
                dr.ellipse([px - r, py - r, px + r, py + r], fill=col)
                dr.ellipse([px - r * 0.45, py - r * 0.8, px + r * 0.2, py - r * 0.2],
                           fill=(255, 235, 225, 110))
            elif tag == "splat":
                px, py, r, c, op = a
                cc = tuple(int(255 * min(1.0, v)) for v in c)
                for k in range(5, 0, -1):
                    rr = r * k / 5
                    dr.ellipse([px - rr, py - rr, px + rr, py + rr],
                               fill=cc + (int(60 * op * (1.0 - k / 6.0)),))
            elif tag == "shadow":
                px, py, r = a[0], a[1], a[2]
                c = cam.project((px, shadow_y, py))
                if c:
                    s = r * cam.f / max(1.0, c[2])
                    dr.ellipse([c[0] - s, c[1] - s * 0.35, c[0] + s, c[1] + s * 0.35],
                               fill=(0, 0, 0, 60))

        # ---- 叠加信息 ----
        fps = 1000.0 / ms if ms > 0 else 0
        dr.rectangle([0, 0, W, 22], fill=(10, 12, 16, 200))
        dr.text((8, 4), "vxl-phys", font=fontb, fill=(120, 220, 255))
        dr.text((86, 5), "体素 · 多边形 · 高斯喷溅 · 刚体（四域同场）",
                font=font, fill=(200, 210, 225))
        dr.text((8, H - 18),
                f"tick {tick}  |  {ms:.2f} ms/tick  |  {fps:.0f} FPS  |  体 {len(bodies)}",
                font=font, fill=(200, 210, 225))
        avg = total_ms / (fi + 1)
        dr.text((W - 200, H - 18), f"均 {avg:.2f} ms ⇒ {1000 / avg:.0f} FPS",
                font=font, fill=(160, 230, 180))

        out_name = FRAMES_DIR + "/f%04d.png" % fi
        im.save(_resolve(out_name))
        out_frames.append(im)
        if fi % 25 == 0:
            print(f"  渲染 {fi}/{len(frames)}")

    out_frames[0].save(_resolve(DST_NAME), save_all=True, append_images=out_frames[1:],
                       duration=int(1000 / FPS), loop=0, optimize=True)
    print(f"完成：out/{DST_NAME}（{len(out_frames)} 帧 @ {FPS} fps）")


if __name__ == "__main__":
    main()
