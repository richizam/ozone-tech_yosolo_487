# -*- coding: utf-8 -*-
"""Sign textures for the cell: EN headline + RU subtitle, zone colour coded.

Rendered once with matplotlib (DejaVu Sans ships Cyrillic glyphs) into
cell/assets/signs/*.png and referenced by MJCF billboard/decal planes.
Regeneration is cheap and idempotent; files are content-stamped so a text or
colour change in params.SIGNS invalidates the cache automatically.
"""
import hashlib
import json
from pathlib import Path

from cell import params as P

SIGN_DIR = Path(__file__).parent / "assets" / "signs"
_ACCENT_YELLOW = (0.98, 0.80, 0.10)


def _bg_accent(zone):
    if zone is None:
        return (0.16, 0.18, 0.23), _ACCENT_YELLOW
    r, g, b = P.ROUTE_RGBA[zone]
    return (r * 0.72, g * 0.72, b * 0.72), (1.0, 1.0, 1.0)


def _render(path, en, ru, zone):
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure
    from matplotlib.patches import Rectangle

    bg, accent = _bg_accent(zone)
    fig = Figure(figsize=(10.24, 2.56), dpi=100)
    FigureCanvasAgg(fig)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_axis_off()
    ax.set_xlim(0, 1024)
    ax.set_ylim(0, 256)
    ax.add_patch(Rectangle((0, 0), 1024, 256, color=bg))
    ax.add_patch(Rectangle((0, 0), 34, 256, color=accent))
    ax.text(58, 160, en, color="white", fontsize=46, fontweight="bold",
            va="center", ha="left", family="DejaVu Sans")
    ax.text(58, 62, ru, color=(0.88, 0.90, 0.94), fontsize=27,
            va="center", ha="left", family="DejaVu Sans")
    fig.savefig(path, format="png")


def ensure_signs():
    """Render any missing/stale sign textures; return {key: filename}."""
    SIGN_DIR.mkdir(parents=True, exist_ok=True)
    out = {}
    for key, en, ru, _pos, zone in P.SIGNS:
        stamp = hashlib.sha1(json.dumps([en, ru, zone]).encode()).hexdigest()[:8]
        path = SIGN_DIR / f"{key}_{stamp}.png"
        if not path.exists():
            for old in SIGN_DIR.glob(f"{key}_*.png"):
                old.unlink()
            _render(path, en, ru, zone)
        out[key] = path.name
    return out
