# -*- coding: utf-8 -*-
"""Jury-facing perception panels — what the RTX depth station actually sees,
in the cell's presentation design language (Ozon blue/magenta, zone colours).

For every vision still in a run directory composes a 1920x1080 panel:
  left   real RGB frame from the overhead head;
  right  the SAME frame's depth in graded blue-steel, the item segmented and
         tinted in its ROUTE colour, measured OBB drawn with dimension
         callouts (mm, from the multi-read depth fusion — not from pixels);
  footer measured dims, reads/confidence meter, the OFFICIAL-RULE reasoning
         line, and a large route-decision chip (B/C/D/REVIEW) with the
         Russian category wording from the task statement.

Usage:
  python tools/make_perception_panels.py <run_dir> [out_dir]
  python tools/make_perception_panels.py --selftest [out_dir]   # mock panel
Optionally assembles perception_demo.mp4 (2.5 s per panel) if ffmpeg exists.
"""
import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

OZON_BLUE = (0, 91, 255)
MAGENTA = (240, 30, 120)
BG = (13, 14, 18)
PANEL_BG = (22, 24, 30)
EDGE = (46, 50, 60)
TXT = (235, 237, 242)
SUB = (150, 156, 168)

ROUTE_COL = {"B": (74, 140, 242), "C": (242, 140, 38),
             "D": (51, 199, 89), "REVIEW": (184, 64, 153)}
ROUTE_RU = {"B": "ПОДХОДИТ ДЛЯ СОРТИРОВКИ",
            "C": "НЕ ПОДХОДИТ ПО ГАБАРИТАМ",
            "D": "ТРЕБУЕТ ДОУПАКОВКИ",
            "REVIEW": "РУЧНОЙ РАЗБОР"}
ROUTE_EN = {"B": "MAIN SORTER", "C": "OVERSIZE / UNDERSIZE",
            "D": "REPACK", "REVIEW": "MANUAL REVIEW"}


def font(sz, mono=False, bold=True):
    cands = (["C:/Windows/Fonts/consola.ttf" if not bold else
              "C:/Windows/Fonts/consolab.ttf"] if mono else
             ["C:/Windows/Fonts/arialbd.ttf" if bold else
              "C:/Windows/Fonts/arial.ttf"])
    cands += ["/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
              "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"]
    for cand in cands:
        try:
            return ImageFont.truetype(cand, sz)
        except Exception:
            continue
    return ImageFont.load_default()


def depth_view(d):
    """Depth array -> (blue-steel uint8 RGB, item mask)."""
    d = d.astype(float)
    finite = np.isfinite(d)
    lo, hi = (np.percentile(d[finite], [1, 99]) if finite.any() else (0, 1))
    g = np.clip((d - lo) / max(hi - lo, 1e-6), 0, 1)
    g[~finite] = 1.0
    near = (1 - g)                                # near = bright
    # graded blue-steel: dark navy floor -> pale steel near
    r = (18 + 130 * near).astype(np.uint8)
    gch = (22 + 150 * near).astype(np.uint8)
    b = (34 + 190 * near).astype(np.uint8)
    img = np.dstack([r, gch, b])
    # item mask: meaningfully above the belt plane, seeded at the closest
    # point inside the central corridor (same logic as validation)
    H0, W0 = d.shape
    strip = d[int(H0 * 0.40):int(H0 * 0.60), :]
    sf = np.isfinite(strip)
    belt_d = np.median(strip[sf]) if sf.any() else 0
    cand = finite & (d < belt_d - 0.015)
    mask = np.zeros_like(cand)
    corridor = np.zeros_like(cand)
    corridor[int(H0 * 0.30):int(H0 * 0.70), int(W0 * 0.22):int(W0 * 0.78)] = True
    if (cand & corridor).any():
        dd = np.where(cand & corridor, d, np.inf)
        seed = np.unravel_index(np.argmin(dd), dd.shape)
        stack = [seed]
        mask[seed] = True
        while stack:
            y, x = stack.pop()
            for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                if 0 <= ny < H0 and 0 <= nx < W0 and cand[ny, nx] \
                        and not mask[ny, nx]:
                    mask[ny, nx] = True
                    stack.append((ny, nx))
    return img, mask


def _rounded(d, xy, radius, fill=None, outline=None, width=1):
    d.rounded_rectangle(xy, radius=radius, fill=fill, outline=outline,
                        width=width)


def compose_panel(slug, rgb_img, depth_rgb, mask, fused, run_tag=""):
    zone = fused.get("zone", "?")
    col = ROUTE_COL.get(zone, (170, 170, 180))
    W, H = 1920, 1080
    panel = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(panel)
    # brand rails
    d.rectangle([0, 0, W, 8], fill=OZON_BLUE)
    d.rectangle([0, H - 8, W, H], fill=MAGENTA)
    # header
    d.text((56, 30), "RTX DEPTH STATION — PERCEPTION EVIDENCE",
           font=font(38), fill=TXT)
    d.text((56, 84), "измерение и классификация в движении · официальные "
           "правила трека 3 · без остановки ленты",
           font=font(22, bold=False), fill=SUB)
    tag = f"item: {slug}" + (f"   ·   run: {run_tag}" if run_tag else "")
    tw = d.textlength(tag, font=font(26, mono=True))
    d.text((W - 56 - tw, 44), tag, font=font(26, mono=True), fill=SUB)

    # view panels
    vw, vh = 860, 610
    y0 = 186
    for i, (im, title, subtitle) in enumerate((
            (rgb_img, "SCENE VIEW · RGB",
             "верхняя камера над зоной измерения"),
            (None, "SENSOR VIEW · RTX DEPTH",
             "глубина + сегментация + габариты (fusion)"))):
        x0 = 56 + i * (vw + 88)
        _rounded(d, [x0 - 10, y0 - 10, x0 + vw + 10, y0 + vh + 10], 18,
                 fill=PANEL_BG, outline=EDGE, width=2)
        d.text((x0 + 2, y0 - 44), title, font=font(25), fill=TXT)
        stw = d.textlength(subtitle, font=font(17, bold=False))
        d.text((x0 + vw - stw, y0 - 38), subtitle,
               font=font(17, bold=False), fill=SUB)
        if im is not None:
            panel.paste(im.resize((vw, vh)), (x0, y0))

    # depth view with overlays
    x0d = 56 + (vw + 88)
    dep = Image.fromarray(depth_rgb).convert("RGB")
    dims = fused.get("dims_mm")
    if mask is not None and mask.any():
        ys, xs = np.where(mask)
        bx0, bx1, by0, by1 = xs.min(), xs.max(), ys.min(), ys.max()
        ov = np.array(dep, dtype=np.uint8)
        tint = np.zeros_like(ov)
        tint[..., 0], tint[..., 1], tint[..., 2] = col
        m3 = mask[..., None]
        ov = np.where(m3, (0.52 * ov + 0.48 * tint).astype(np.uint8), ov)
        dep = Image.fromarray(ov)
        dd = ImageDraw.Draw(dep)
        dd.rectangle([bx0, by0, bx1, by1], outline=col, width=4)
        # corner ticks (industrial reticle look)
        t = 18
        for cx, cy, sx, sy in ((bx0, by0, 1, 1), (bx1, by0, -1, 1),
                               (bx0, by1, 1, -1), (bx1, by1, -1, -1)):
            dd.line([cx, cy, cx + sx * t, cy], fill=(255, 255, 255), width=2)
            dd.line([cx, cy, cx, cy + sy * t], fill=(255, 255, 255), width=2)
        dep = dep.resize((vw, vh))
        dd = ImageDraw.Draw(dep)
        # dimension callouts from the FUSED measurement (mm)
        sx_, sy_ = vw / depth_rgb.shape[1], vh / depth_rgb.shape[0]
        if dims:
            fx0, fx1 = bx0 * sx_, bx1 * sx_
            fy0, fy1 = by0 * sy_, by1 * sy_
            f22 = font(22, mono=True)
            wlab = f"{dims[0]:.0f} mm"
            dd.line([fx0, fy1 + 22, fx1, fy1 + 22], fill=(255, 255, 255),
                    width=2)
            dd.text(((fx0 + fx1) / 2 - dd.textlength(wlab, f22) / 2,
                     fy1 + 28), wlab, font=f22, fill=(255, 255, 255))
            hlab = f"{dims[1]:.0f} mm"
            dd.line([fx1 + 22, fy0, fx1 + 22, fy1], fill=(255, 255, 255),
                    width=2)
            dd.text((fx1 + 30, (fy0 + fy1) / 2 - 12), hlab, font=f22,
                    fill=(255, 255, 255))
    else:
        dep = dep.resize((vw, vh))
    panel.paste(dep, (x0d, y0))
    rgb_small = None  # freed

    # footer strip
    fy = y0 + vh + 34
    _rounded(d, [46, fy, W - 46, H - 42], 18, fill=PANEL_BG, outline=EDGE,
             width=2)
    dims_s = (" × ".join(f"{v:.0f}" for v in dims) + " мм") if dims else "n/a"
    d.text((80, fy + 26), "ИЗМЕРЕНО (fusion)", font=font(20), fill=SUB)
    d.text((80, fy + 56), dims_s, font=font(46, mono=True), fill=TXT)
    n_r = fused.get("n_reads")
    conf = fused.get("confidence")
    d.text((600, fy + 26), "ЧТЕНИЯ · УВЕРЕННОСТЬ", font=font(20), fill=SUB)
    d.text((600, fy + 56), f"{n_r if n_r is not None else '—'} reads",
           font=font(34, mono=True), fill=TXT)
    # confidence meter
    if conf is not None:
        mx0, mx1, my = 600, 980, fy + 104
        d.rounded_rectangle([mx0, my, mx1, my + 14], 7, fill=(40, 44, 52))
        d.rounded_rectangle([mx0, my,
                             mx0 + (mx1 - mx0) * float(min(conf, 1.0)),
                             my + 14], 7, fill=col)
        d.text((mx1 + 14, my - 8), f"{float(conf):.2f}",
               font=font(22, mono=True), fill=TXT)
    reason = (fused.get("reason") or "")[:104]
    d.text((80, fy + 146), f"правило: {reason}",
           font=font(19, mono=True, bold=False), fill=SUB)

    # route decision chip
    chip_x0, chip_y0 = W - 560, fy + 22
    _rounded(d, [chip_x0, chip_y0, W - 78, H - 64], 16, fill=col)
    zlab = f"→ {zone}"
    d.text((chip_x0 + 34, chip_y0 + 10), zlab, font=font(82),
           fill=(12, 13, 16))
    d.text((chip_x0 + 34, chip_y0 + 106), ROUTE_RU.get(zone, ""),
           font=font(25), fill=(12, 13, 16))
    en = ROUTE_EN.get(zone, "")
    d.text((chip_x0 + 34, chip_y0 + 138), en, font=font(18, bold=False),
           fill=(30, 32, 38))
    return panel


def mock_inputs():
    """Synthetic RGB/depth/fused record for --selftest design iteration."""
    Wv, Hv = 640, 480
    rng = np.random.default_rng(7)
    rgb = np.full((Hv, Wv, 3), 52, np.uint8)
    rgb[:, :, 2] += 6
    rgb[Hv // 2 - 90:Hv // 2 + 90, Wv // 2 - 130:Wv // 2 + 130] = (188, 176, 158)
    rgb += rng.integers(0, 6, rgb.shape, dtype=np.uint8)
    depth = np.full((Hv, Wv), 1.30, float)
    depth[Hv // 2 - 90:Hv // 2 + 90, Wv // 2 - 130:Wv // 2 + 130] = 1.06
    depth += rng.normal(0, 0.002, depth.shape)
    fused = {"zone": "B", "dims_mm": [312.0, 218.0, 174.0],
             "confidence": 0.97, "n_reads": 5,
             "reason": "fits 450x320x320, no circle evidence: circ=0.61 "
                       "sect=0.00 elong=1.4 [macro head: 5 reads]"}
    return Image.fromarray(rgb), depth, fused


def main():
    args = [a for a in sys.argv[1:]]
    if args and args[0] == "--selftest":
        out = Path(args[1]) if len(args) > 1 else Path("panel_selftest")
        out.mkdir(parents=True, exist_ok=True)
        rgb, depth, fused = mock_inputs()
        dview, mask = depth_view(depth)
        p = compose_panel("mock_box", rgb, dview, mask, fused, "selftest")
        p.save(out / "perception_mock.png")
        print(f"panel: {out/'perception_mock.png'}")
        return 0

    run = Path(args[0])
    out = Path(args[1]) if len(args) > 1 else run / "perception_panels"
    out.mkdir(parents=True, exist_ok=True)
    reads = {}
    rl = run / "reads_log.json"
    if rl.is_file():
        reads = json.loads(rl.read_text(encoding="utf-8"))
    panels = []
    for rgb_p in sorted(run.glob("vision_rgb_*.png")):
        slug = rgb_p.stem.replace("vision_rgb_", "")
        npy_p = run / f"vision_depth_{slug}.npy"
        if not npy_p.is_file():
            continue
        rgb = Image.open(rgb_p).convert("RGB")
        dview, mask = depth_view(np.load(npy_p))
        fused = (reads.get(slug) or {}).get("fused", {})
        p = compose_panel(slug, rgb, dview, mask, fused, run.name)
        fp = out / f"perception_{slug}.png"
        p.save(fp)
        panels.append(fp)
        print(f"panel: {fp}")
    if panels and shutil.which("ffmpeg"):
        lst = out / "list.txt"
        lst.write_text("".join(f"file '{p.name}'\nduration 2.5\n"
                               for p in panels) + f"file '{panels[-1].name}'\n",
                       encoding="utf-8")
        res = subprocess.run(
            ["ffmpeg", "-y", "-f", "concat", "-safe", "0", "-i", str(lst),
             "-vf", "fps=30", "-c:v", "libx264", "-pix_fmt", "yuv420p",
             str(out / "perception_demo.mp4")], cwd=out, capture_output=True)
        if res.returncode == 0:
            print(f"video: {out / 'perception_demo.mp4'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
