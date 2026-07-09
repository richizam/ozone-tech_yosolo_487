# -*- coding: utf-8 -*-
"""Final-cinematic end card: the validated metrics, Ozon design language.
Usage: python tools/make_endcard.py [out.png]"""
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

REPO = Path(__file__).resolve().parent.parent
OZON_BLUE = (0, 91, 255)
MAGENTA = (240, 30, 120)


def font(sz, bold=True):
    for cand in ("C:/Windows/Fonts/arialbd.ttf" if bold else
                 "C:/Windows/Fonts/arial.ttf",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(cand, sz)
        except Exception:
            continue
    return ImageFont.load_default()


def main():
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else REPO / "docs" / \
        "report" / "isaac_evidence" / "final_arb" / "endcard.png"
    W, H = 1920, 1080
    img = Image.new("RGB", (W, H), (13, 14, 18))
    d = ImageDraw.Draw(img)
    # Ozon accents
    d.rectangle([0, 0, W, 10], fill=OZON_BLUE)
    d.rectangle([0, H - 10, W, H], fill=MAGENTA)
    logo_p = REPO / "isaac" / "assets" / "ozon_logo.png"
    if logo_p.is_file():
        logo = Image.open(logo_p).convert("RGBA")
        lw = 340
        lh = int(logo.height * lw / logo.width)
        img.paste(logo.resize((lw, lh)), (W - lw - 70, 52), logo.resize((lw, lh)))
    d.text((70, 60), "SORT CELL — VALIDATED DIGITAL TWIN", font=font(64),
           fill=(240, 242, 246))
    d.text((70, 150), "NVIDIA Isaac Sim 6.0.1 · PhysX 5 · RTX sensors in the loop",
           font=font(36, bold=False), fill=(150, 155, 165))
    rows = [
        ("132", "item trials · 12-run validation matrix"),
        ("100%", "classification from the RTX depth station, in motion"),
        ("98.5%", "nominal routing (single exception: safe operator call-out)"),
        ("11/11", "every robustness sweep: friction ×0.7/×1.3 · mass ×1.3 · "
                  "close spacing · off-center · jam drill"),
        ("0", "unsafe errors — containment 1.0 in every run"),
        ("0", "direct velocity writes: motion is surface contact only"),
        ("6064", "logged ARB actuator commands · 40 ms latency · 6 m/s² ramp"),
    ]
    y = 280
    for big, small in rows:
        d.text((70, y), big, font=font(72), fill=OZON_BLUE
               if big not in ("0",) else (80, 220, 120))
        d.text((360, y + 18), small, font=font(38, bold=False),
               fill=(210, 213, 220))
        y += 106
    d.text((70, H - 68), "perception → classification → patch-level ARB "
           "actuation → physical routing — no scripted item motion",
           font=font(30, bold=False), fill=(150, 155, 165))
    img.save(out)
    print(f"endcard: {out}")


if __name__ == "__main__":
    raise SystemExit(main())
