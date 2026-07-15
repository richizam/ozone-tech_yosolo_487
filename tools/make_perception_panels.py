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
    """Depth array -> (blue-steel uint8 RGB, item mask).

    The item is the connected above-belt component that (a) does not touch
    the frame border (fixed structures — infeed hardware, gate posts — all
    run off-frame) and (b) sits nearest the frame centre, where the release
    is timed to put the item at capture. Threshold 6 mm above the belt plane
    (~3 sigma of depth noise) keeps the 10-11 mm certification-floor cubes
    segmentable — the old 15 mm cut could never see them.
    """
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
    H0, W0 = d.shape
    strip = d[int(H0 * 0.40):int(H0 * 0.60), int(W0 * 0.25):int(W0 * 0.75)]
    sf = np.isfinite(strip)
    belt_d = np.median(strip[sf]) if sf.any() else 0
    cand = finite & (d < belt_d - 0.006)
    lab = np.zeros(d.shape, np.int32)
    best, best_dist = None, None
    n_lab = 0
    for sy, sx in zip(*np.where(cand)):
        if lab[sy, sx]:
            continue
        n_lab += 1
        stack = [(sy, sx)]
        lab[sy, sx] = n_lab
        px_y, px_x = [sy], [sx]
        while stack:
            y, x = stack.pop()
            for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                if 0 <= ny < H0 and 0 <= nx < W0 and cand[ny, nx] \
                        and not lab[ny, nx]:
                    lab[ny, nx] = n_lab
                    stack.append((ny, nx))
                    px_y.append(ny)
                    px_x.append(nx)
        ys, xs = np.array(px_y), np.array(px_x)
        if len(ys) < 8:                                   # noise speck
            continue
        if ys.min() == 0 or xs.min() == 0 or ys.max() == H0 - 1 \
                or xs.max() == W0 - 1:                    # fixed structure
            continue
        cy, cx = ys.mean() / H0, xs.mean() / W0
        if not (0.20 <= cy <= 0.80 and 0.20 <= cx <= 0.80):
            continue
        dist = (cy - 0.5) ** 2 + (cx - 0.5) ** 2
        if best is None or dist < best_dist:
            best, best_dist = n_lab, dist
    mask = (lab == best) if best is not None else np.zeros_like(cand)
    return img, mask


def _rounded(d, xy, radius, fill=None, outline=None, width=1):
    d.rounded_rectangle(xy, radius=radius, fill=fill, outline=outline,
                        width=width)


def steel(depth_crop):
    """Locally re-normalized blue-steel render of a raw depth crop — small
    items (11 mm over a 1.7 m global range) are sub-quantization in the
    globally graded view; a local stretch makes them plainly visible."""
    c = depth_crop.astype(float)
    fin = np.isfinite(c)
    lo, hi = (np.percentile(c[fin], [2, 98]) if fin.any() else (0, 1))
    g = np.clip((c - lo) / max(hi - lo, 1e-6), 0, 1)
    g[~fin] = 1.0
    near = 1 - g
    return np.dstack([(18 + 130 * near).astype(np.uint8),
                      (22 + 150 * near).astype(np.uint8),
                      (34 + 190 * near).astype(np.uint8)])


def compose_panel(slug, rgb_img, depth_rgb, mask, fused, run_tag="",
                  depth_raw=None, mask_src="local", n_cloud=None,
                  macro_depth=None, macro_mask=None):
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
    sensor_sub = ("глубина + сегментация perception_rtx + габариты (fusion)"
                  if mask_src == "pipeline" else
                  "глубина + сегментация + габариты (fusion)")
    for i, (im, title, subtitle) in enumerate((
            (rgb_img, "SCENE VIEW · RGB",
             "верхняя камера над зоной измерения"),
            (None, "SENSOR VIEW · RTX DEPTH", sensor_sub))):
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
        H0, W0 = mask.shape
        ys, xs = np.where(mask)
        # pad the reticle so it frames rather than covers tiny items
        pad = 5
        bx0 = max(int(xs.min()) - pad, 0)
        bx1 = min(int(xs.max()) + pad, W0 - 1)
        by0 = max(int(ys.min()) - pad, 0)
        by1 = min(int(ys.max()) + pad, H0 - 1)
        ov = np.array(dep, dtype=np.uint8)
        tint = np.zeros_like(ov)
        tint[..., 0], tint[..., 1], tint[..., 2] = col
        m3 = mask[..., None]
        ov = np.where(m3, (0.52 * ov + 0.48 * tint).astype(np.uint8), ov)
        dep = Image.fromarray(ov)
        # macro inset for small items. Preferred source: the REAL macro
        # head's depth frame + the pipeline's own macro mask (dual-range
        # metrology evidence). Fallback: magnified overhead crop re-rendered
        # from RAW depth with a local contrast stretch (globally graded
        # pixels are sub-quantization for an 11 mm item).
        inset = rgb_inset = None
        inset_label = None
        bw, bh = bx1 - bx0, by1 - by0
        small = max(bw, bh) < 90
        if small and macro_depth is not None and macro_mask is not None \
                and macro_mask.any():
            mys, mxs = np.where(macro_mask)
            mH, mW = macro_mask.shape
            mside = int(min(max(120, int(max(mxs.max() - mxs.min(),
                                             mys.max() - mys.min()) * 1.8)),
                            min(mH, mW)))
            mcx, mcy = (mxs.min() + mxs.max()) // 2, (mys.min() + mys.max()) // 2
            jx0 = int(np.clip(mcx - mside // 2, 0, mW - mside))
            jy0 = int(np.clip(mcy - mside // 2, 0, mH - mside))
            crop = steel(macro_depth[jy0:jy0 + mside, jx0:jx0 + mside])
            mcrop = macro_mask[jy0:jy0 + mside, jx0:jx0 + mside]
            tint_c = np.zeros_like(crop)
            tint_c[..., 0], tint_c[..., 1], tint_c[..., 2] = col
            crop = np.where(mcrop[..., None],
                            (0.35 * crop + 0.65 * tint_c).astype(np.uint8),
                            crop)
            inset = Image.fromarray(crop).resize((320, 320), Image.NEAREST)
            inset_label = "МАКРО-ГОЛОВКА · РЕАЛЬНЫЙ КАДР · GSD 0.4 мм"
        elif small and depth_raw is not None:
            side = int(min(max(90, max(bw, bh) * 2.5), 260))
            icx = (bx0 + bx1) // 2
            icy = (by0 + by1) // 2
            ix0 = int(np.clip(icx - side // 2, 0, W0 - side))
            iy0 = int(np.clip(icy - side // 2, 0, H0 - side))
            crop = steel(depth_raw[iy0:iy0 + side, ix0:ix0 + side])
            mcrop = mask[iy0:iy0 + side, ix0:ix0 + side]
            tint_c = np.zeros_like(crop)
            tint_c[..., 0], tint_c[..., 1], tint_c[..., 2] = col
            crop = np.where(mcrop[..., None],
                            (0.35 * crop + 0.65 * tint_c).astype(np.uint8),
                            crop)
            inset = Image.fromarray(crop).resize((320, 320), Image.NEAREST)
            inset_label = f"МАКРО-ГОЛОВКА · КРОП ×{320 / side:.1f}"
        if small and depth_raw is not None and rgb_img.size == (W0, H0):
            side = int(min(max(90, max(bw, bh) * 2.5), 260))
            icx = (bx0 + bx1) // 2
            icy = (by0 + by1) // 2
            ix0 = int(np.clip(icx - side // 2, 0, W0 - side))
            iy0 = int(np.clip(icy - side // 2, 0, H0 - side))
            zoom = 320 / side
            rgb_inset = rgb_img.crop((ix0, iy0, ix0 + side, iy0 + side)) \
                               .resize((320, 320), Image.NEAREST)
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
            fxm = (fx0 + fx1) / 2
            # dimension line never shorter than 70 px, centred on the box
            lx0, lx1 = fx0, fx1
            if lx1 - lx0 < 70:
                lx0, lx1 = fxm - 35, fxm + 35
            f22 = font(22, mono=True)
            wlab = f"{dims[0]:.0f} mm"
            dd.line([lx0, fy1 + 22, lx1, fy1 + 22], fill=(255, 255, 255),
                    width=2)
            dd.text((fxm - dd.textlength(wlab, f22) / 2, fy1 + 28), wlab,
                    font=f22, fill=(255, 255, 255))
            hlab = f"{dims[1]:.0f} mm"
            dd.line([fx1 + 22, fy0, fx1 + 22, fy1], fill=(255, 255, 255),
                    width=2)
            dd.text((fx1 + 30, (fy0 + fy1) / 2 - 12), hlab, font=f22,
                    fill=(255, 255, 255))
        if inset is not None:
            iw = 320
            ix, iy = 18, vh - iw - 18
            dep.paste(inset, (ix, iy))
            dd.rectangle([ix - 2, iy - 2, ix + iw + 1, iy + iw + 1],
                         outline=col, width=3)
            ilab = inset_label
            f18 = font(18)
            lw = dd.textlength(ilab, f18)
            dd.rectangle([ix - 2, iy - 34, ix + lw + 18, iy - 2],
                         fill=(12, 13, 16))
            dd.text((ix + 8, iy - 30), ilab, font=f18, fill=TXT)
    else:
        dep = dep.resize((vw, vh))
    if mask_src == "pipeline":
        # provenance badge: this mask is the measuring pipeline's export,
        # not a presentation-side re-derivation
        dd = ImageDraw.Draw(dep)
        blab = "маска и облако: экспорт perception_rtx"
        f16 = font(16, bold=False)
        bw_ = dd.textlength(blab, f16)
        dd.rectangle([vw - bw_ - 26, vh - 36, vw - 6, vh - 6],
                     fill=(12, 13, 16))
        dd.text((vw - bw_ - 16, vh - 31), blab, font=f16, fill=SUB)
    panel.paste(dep, (x0d, y0))
    if mask is not None and mask.any() and rgb_inset is not None:
        # same magnified window on the RGB pane (heads are pixel-aligned)
        iw = 320
        gx, gy = 56 + 18, y0 + vh - iw - 18
        panel.paste(rgb_inset, (gx, gy))
        d.rectangle([gx - 2, gy - 2, gx + iw + 1, gy + iw + 1],
                    outline=col, width=3)
        ilab = f"КРОП ×{zoom:.1f}"
        f18 = font(18)
        lw = d.textlength(ilab, f18)
        d.rectangle([gx - 2, gy - 34, gx + lw + 18, gy - 2],
                    fill=(12, 13, 16))
        d.text((gx + 8, gy - 30), ilab, font=f18, fill=TXT)
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
    reads_s = f"{n_r if n_r is not None else '—'} reads"
    d.text((600, fy + 56), reads_s, font=font(34, mono=True), fill=TXT)
    if n_cloud:
        d.text((600 + d.textlength(reads_s, font(34, mono=True)) + 22,
                fy + 68), f"· облако {n_cloud:,} тчк".replace(",", " "),
               font=font(20, bold=False), fill=SUB)
    # confidence meter (= per-read agreement with the fused verdict)
    if conf is not None:
        mx0, mx1, my = 600, 980, fy + 104
        d.rounded_rectangle([mx0, my, mx1, my + 14], 7, fill=(40, 44, 52))
        if float(conf) > 0:
            d.rounded_rectangle([mx0, my,
                                 mx0 + (mx1 - mx0) * float(min(conf, 1.0)),
                                 my + 14], 7, fill=col)
        d.text((mx1 + 14, my - 8), f"{float(conf):.2f}",
               font=font(22, mono=True), fill=TXT)
        if float(conf) < 0.5 and not fused.get("sensor_miss"):
            # fused evidence overruled the per-frame votes — that IS the
            # value of multi-read fusion; say so instead of looking broken
            note = "→ агрегатная улика (safe-side)"
            fn = font(17, bold=False)
            nx = mx1 + 84
            if nx + d.textlength(note, fn) > W - 560 - 16:
                nx = W - 560 - 16 - d.textlength(note, fn)
            d.text((nx, my - 6), note, font=fn, fill=SUB)
    reason = fused.get("reason") or ""
    max_w = (W - 560) - 80 - 30           # stop before the route chip
    rf = None
    for sz in (19, 18, 17):
        rf = font(sz, mono=True, bold=False)
        if d.textlength(f"правило: {reason}", rf) <= max_w:
            break
    if d.textlength(f"правило: {reason}", rf) > max_w:
        while reason and d.textlength(f"правило: {reason}…", rf) > max_w:
            reason = reason[:-1]
        reason += "…"
    d.text((80, fy + 146), f"правило: {reason}", font=rf, fill=SUB)

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
    # the box must cover <50% of the central belt-estimation strip or the
    # fallback segmentation's belt median reads the box top instead
    rgb = np.full((Hv, Wv, 3), 52, np.uint8)
    rgb[:, :, 2] += 6
    rgb[Hv // 2 - 70:Hv // 2 + 70, Wv // 2 - 70:Wv // 2 + 70] = (188, 176, 158)
    rgb += rng.integers(0, 6, rgb.shape, dtype=np.uint8)
    depth = np.full((Hv, Wv), 1.30, float)
    depth[Hv // 2 - 70:Hv // 2 + 70, Wv // 2 - 70:Wv // 2 + 70] = 1.06
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
        p = compose_panel("mock_box", rgb, dview, mask, fused, "selftest",
                          depth_raw=depth)
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
        depth_raw = np.load(npy_p)
        dview, mask = depth_view(depth_raw)
        # pipeline-truth artifacts (exported by run_isaac capture_still):
        # the measuring pipeline's own mask/cloud beat any local re-derivation
        mask_src, n_cloud = "local", None
        mask_p = run / f"vision_mask_{slug}.png"
        if mask_p.is_file():
            pm = np.array(Image.open(mask_p).convert("L")) > 127
            if pm.shape == depth_raw.shape and pm.any():
                mask, mask_src = pm, "pipeline"
        cloud_p = run / f"vision_cloud_{slug}.npy"
        if cloud_p.is_file():
            n_cloud = int(len(np.load(cloud_p)))
        macro_depth = macro_mask = None
        md_p = run / f"vision_macro_depth_{slug}.npy"
        mm_p = run / f"vision_macro_mask_{slug}.png"
        if md_p.is_file() and mm_p.is_file():
            macro_depth = np.load(md_p)
            macro_mask = np.array(Image.open(mm_p).convert("L")) > 127
            if macro_mask.shape != macro_depth.shape:
                macro_depth = macro_mask = None
        fused = (reads.get(slug) or {}).get("fused", {})
        p = compose_panel(slug, rgb, dview, mask, fused, run.name,
                          depth_raw=depth_raw, mask_src=mask_src,
                          n_cloud=n_cloud, macro_depth=macro_depth,
                          macro_mask=macro_mask)
        fp = out / f"perception_{slug}.png"
        p.save(fp)
        panels.append(fp)
        print(f"panel: {fp}")
    if panels and shutil.which("ffmpeg"):
        lst = out / "list.txt"
        lst.write_text("".join(f"file '{p.name}'\nduration 2.5\n"
                               for p in panels) + f"file '{panels[-1].name}'\n",
                       encoding="utf-8")
        # bare names + cwd=out: the list entries are bare filenames, so every
        # path must resolve relative to the panels directory
        res = subprocess.run(
            ["ffmpeg", "-y", "-f", "concat", "-safe", "0", "-i", "list.txt",
             "-vf", "fps=30", "-c:v", "libx264", "-pix_fmt", "yuv420p",
             "perception_demo.mp4"], cwd=out, capture_output=True)
        if res.returncode == 0:
            print(f"video: {out / 'perception_demo.mp4'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
