# -*- coding: utf-8 -*-
"""Jury-facing perception panels: what the RTX depth station actually sees.

For every vision still in a run directory, composes a side-by-side panel:
  left  = the real RGB view from the overhead head;
  right = the SAME frame's depth in grayscale (sensor's-eye view) with the
          item segmented/highlighted and its measured extents drawn;
  footer = measured dims (mm), official-rule verdict B/C/D, confidence and
          the decision reason from the multi-read fusion (reads_log.json).

Usage: python tools/make_perception_panels.py <run_dir> [out_dir]
Optionally assembles perception_demo.mp4 (2 s per panel) if ffmpeg exists.
"""
import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROUTE_COL = {"B": (64, 140, 242), "C": (242, 140, 38), "D": (51, 199, 89)}


def font(sz):
    for cand in ("C:/Windows/Fonts/consolab.ttf", "C:/Windows/Fonts/arialbd.ttf",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"):
        try:
            return ImageFont.truetype(cand, sz)
        except Exception:
            continue
    return ImageFont.load_default()


def depth_view(npy_path):
    d = np.load(npy_path).astype(float)
    finite = np.isfinite(d)
    lo, hi = np.percentile(d[finite], [1, 99]) if finite.any() else (0, 1)
    g = np.clip((d - lo) / max(hi - lo, 1e-6), 0, 1)
    g[~finite] = 1.0
    img = (255 * (1 - g)).astype(np.uint8)          # near = bright
    # item mask: meaningfully above the belt plane (background ~= the
    # far mode of the depth histogram)
    # reference plane = the BELT surface: median depth of the central
    # corridor strip (p85 of the whole frame lands on the FLOOR and then
    # the belt itself joins the item's connected component)
    H0 = d.shape[0]
    strip = d[int(H0 * 0.40):int(H0 * 0.60), :]
    sf = np.isfinite(strip)
    belt_d = np.median(strip[sf]) if sf.any() else 0
    cand = finite & (d < belt_d - 0.015)
    # keep only the connected blob around the CLOSEST point (the item's
    # top) — the raw threshold also catches belt rails and gantry hardware
    mask = np.zeros_like(cand)
    # seed inside the belt corridor only — the side profiler housings are
    # physically closer to the camera than any item top
    corridor = np.zeros_like(cand)
    W0 = d.shape[1]
    # tight central box: the still is captured at window entry, so the item
    # sits near frame centre; guide rails/brackets live at the edges
    corridor[int(H0 * 0.30):int(H0 * 0.70), int(W0 * 0.22):int(W0 * 0.78)] = True
    if (cand & corridor).any():
        dd = np.where(cand & corridor, d, np.inf)
        seed = np.unravel_index(np.argmin(dd), dd.shape)
        stack = [seed]
        mask[seed] = True
        H, W = cand.shape
        while stack:
            y, x = stack.pop()
            for ny, nx in ((y-1, x), (y+1, x), (y, x-1), (y, x+1)):
                if 0 <= ny < H and 0 <= nx < W and cand[ny, nx] \
                        and not mask[ny, nx]:
                    mask[ny, nx] = True
                    stack.append((ny, nx))
    return img, mask


def main():
    run = Path(sys.argv[1])
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else run / "perception_panels"
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
        gimg, mask = depth_view(npy_p)
        dep = Image.fromarray(gimg).convert("RGB")
        fused = (reads.get(slug) or {}).get("fused", {})
        zone = fused.get("zone", "?")
        col = ROUTE_COL.get(zone, (200, 200, 200))
        # highlight the segmented item + its pixel bbox on the depth view
        if mask.any():
            ys, xs = np.where(mask)
            x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
            ov = np.array(dep, dtype=np.uint8)
            m3 = mask[..., None]
            tint = np.zeros_like(ov)
            tint[..., 0], tint[..., 1], tint[..., 2] = col
            ov = np.where(m3, (0.55 * ov + 0.45 * tint).astype(np.uint8), ov)
            dep = Image.fromarray(ov)
            dd = ImageDraw.Draw(dep)
            dd.rectangle([x0, y0, x1, y1], outline=col, width=4)
        # compose: two 896x672 views + footer
        W, H, FH = 1920, 1080, 250
        vw, vh = 896, 672
        panel = Image.new("RGB", (W, H), (12, 13, 16))
        panel.paste(rgb.resize((vw, vh)), (36, 60))
        panel.paste(dep.resize((vw, vh)), (36 + vw + 56, 60))
        d = ImageDraw.Draw(panel)
        d.text((36, 14), "SCENE VIEW (RGB)", font=font(30), fill=(220, 222, 228))
        d.text((36 + vw + 56, 14), "RTX DEPTH STATION VIEW (sensor's eye)",
               font=font(30), fill=(220, 222, 228))
        y = H - FH + 18
        dims = fused.get("dims_mm")
        dims_s = " x ".join(f"{v:.0f}" for v in dims) + " mm" if dims else "n/a"
        d.text((36, y), f"ITEM: {slug}", font=font(40), fill=(235, 236, 240))
        d.text((36, y + 58), f"measured dims: {dims_s}", font=font(34),
               fill=(200, 203, 210))
        conf = fused.get("confidence")
        n_r = fused.get("n_reads")
        d.text((36, y + 110),
               f"reads fused: {n_r}   confidence: "
               f"{conf if conf is not None else 'n/a'}",
               font=font(34), fill=(200, 203, 210))
        reason = (fused.get("reason") or "")[:96]
        d.text((36, y + 162), f"rule: {reason}", font=font(28),
               fill=(160, 164, 174))
        # big verdict chip
        d.rounded_rectangle([W - 420, y + 10, W - 60, y + 180], radius=18,
                            fill=col)
        d.text((W - 385, y + 30), f"-> {zone}", font=font(96),
               fill=(15, 15, 18))
        lab = {"B": "SORTER", "C": "OVERSIZE", "D": "REPACK"}.get(zone, "")
        d.text((W - 385, y + 132), lab, font=font(34), fill=(15, 15, 18))
        p = out / f"perception_{slug}.png"
        panel.save(p)
        panels.append(p)
        print(f"panel: {p}")
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
