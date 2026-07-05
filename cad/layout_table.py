# -*- coding: utf-8 -*-
"""Top-view layout drawing of the TRANSFER-TABLE architecture, generated
directly from cell/params.py (single source of truth — the same numbers the
simulation runs on).

  * cad/out/layout_table_top_view.png
  * printed reach check of the arm exception station

The arm-primary baseline drawing remains in cad/layout_v0.py.
"""
import sys
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrow, Polygon

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from cell import params as P  # noqa: E402
from cell.arm_ik import ik  # noqa: E402

OUT = Path(__file__).parent / "out"
OUT.mkdir(parents=True, exist_ok=True)
MM = 1000.0


def main():
    a, b, tb, cb = P.BELT_A, P.BELT_B, P.TABLE, P.CONNECT_B
    cc, cd = P.CHUTE_C, P.CHUTE_D
    cages = P.cages_for("table")
    base = P.ARM_BASE["table"]

    # ---------------- reach check of the exception station
    pts = {
        "routing zone W": (tb["route_x"], tb["y"], 0.95),
        "lane B start": (tb["lane_B_cx"], tb["y"] + tb["width"] / 2 - 0.1, 0.95),
        "lane C exit": (tb["x1"] - 0.05, tb["lane_C_cy"], 0.95),
        "lane D exit": (tb["lane_D_cx"], tb["y"] - tb["width"] / 2 + 0.05, 0.95),
        "D drop": (P.PLACE_BY_MODE["table"]["D"]["xy"][0], P.PLACE_BY_MODE["table"]["D"]["xy"][1], 1.1),
        "C drop": (P.PLACE_BY_MODE["table"]["C"]["xy"][0], P.PLACE_BY_MODE["table"]["C"]["xy"][1], 1.1),
    }
    usable = P.ARM["reach"] - P.ARM["reach_margin"]
    print(f"exception station {base}, usable reach {usable:.2f} m")
    for name, xyz in pts.items():
        d = float(np.hypot(xyz[0] - base[0], xyz[1] - base[1]))
        ik(np.array(xyz), base=base)          # raises if unreachable
        print(f"  {name:<16} d={d:5.2f}  {'OK' if d <= usable else '!! margin'}")

    # ---------------- drawing (mm)
    fig, ax = plt.subplots(figsize=(14, 9))
    ax.add_patch(Rectangle((0, 0), 10000, 6000, fill=False, ec="red", ls="--", lw=1.5))
    ax.text(80, 5800, "Рабочая зона / work zone 10000×6000", color="red", fontsize=9)

    # conveyor A + gates
    ax.add_patch(Rectangle((a["x0"] * MM, (a["y"] - a["width"] / 2) * MM),
                           (tb["x0"] - a["x0"]) * MM, a["width"] * MM, fc="#b0c4de", ec="k"))
    ax.add_patch(FancyArrow(1200, a["y"] * MM, 900, 0, width=40, color="k"))
    ax.annotate("A: конвейер 1 м/с", (1500, a["y"] * MM + 320), fontsize=9)
    for gx, lbl in ((a["hold2_x"], "предгейт /\npre-gate hold"), (a["gate_x"], "эскейпмент /\ngate")):
        ax.plot([gx * MM, gx * MM], [(a["y"] - 0.33) * MM, (a["y"] + 0.33) * MM], color="crimson", lw=2)
        ax.annotate(lbl, (gx * MM - 260, (a["y"] + 0.38) * MM), color="crimson", fontsize=7)

    # vision station
    ax.plot([6000, 6000], [(a["y"] - 0.6) * MM, (a["y"] + 0.6) * MM], color="purple", lw=2)
    ax.annotate("DWS-станция: глубинная сетка +\nпрофилометры (3 головы)", (4450, (a["y"] + 0.68) * MM),
                color="purple", fontsize=8)

    # transfer table + lanes
    ax.add_patch(Rectangle((tb["x0"] * MM, (tb["y"] - tb["width"] / 2) * MM),
                           (tb["x1"] - tb["x0"]) * MM, tb["width"] * MM, fc="#8fa8c8", ec="k"))
    ax.annotate("трансферный стол /\ntri-directional transfer table",
                (tb["x0"] * MM + 80, (tb["y"] - 0.13) * MM), fontsize=8)
    ax.plot([tb["route_x"] * MM, tb["route_x"] * MM],
            [(tb["y"] - tb["width"] / 2) * MM, (tb["y"] + tb["width"] / 2) * MM], color="k", ls=":", lw=1)
    for (x, y, dx, dy, col) in ((tb["lane_B_cx"], tb["y"] + 0.15, 0, 0.3, "tab:blue"),
                                (tb["x1"] - 0.35, tb["lane_C_cy"], 0.3, 0, "tab:red"),
                                (tb["lane_D_cx"], tb["y"] - 0.15, 0, -0.3, "tab:green")):
        ax.add_patch(FancyArrow(x * MM, y * MM, dx * MM, dy * MM, width=30, color=col))

    # B connector + fixed belt B
    ax.add_patch(Rectangle(((cb["cx"] - cb["width"] / 2) * MM, cb["y0"] * MM),
                           cb["width"] * MM, (cb["y1"] - cb["y0"]) * MM, fc="#8fa8c8", ec="k"))
    ax.add_patch(Rectangle(((b["cx"] - b["width"] / 2) * MM, b["y0"] * MM),
                           b["width"] * MM, (b["y1"] - b["y0"]) * MM, fc="#b0c4de", ec="k"))
    ax.annotate("B: инфид сортера (fixed)", ((b["cx"] - 1.05) * MM, (b["y1"] - 0.3) * MM), fontsize=9)

    # chutes (as trapezoid shadows)
    ax.add_patch(Polygon(np.array([[cc["x0"], cc["cy"] - cc["width"] / 2], [cc["x1"], cc["cy"] - cc["width"] / 2],
                                   [cc["x1"], cc["cy"] + cc["width"] / 2], [cc["x0"], cc["cy"] + cc["width"] / 2]]) * MM,
                         closed=True, fc="#c9cdd4", ec="k", alpha=0.9))
    ax.annotate("склиз C ↓0.32", (cc["x0"] * MM + 40, (cc["cy"] - 0.28) * MM), fontsize=7)
    ax.add_patch(Polygon(np.array([[cd["cx"] - cd["width"] / 2, cd["y0"]], [cd["cx"] + cd["width"] / 2, cd["y0"]],
                                   [cd["cx"] + cd["width"] / 2, cd["y1"]], [cd["cx"] - cd["width"] / 2, cd["y1"]]]) * MM,
                         closed=True, fc="#c9cdd4", ec="k", alpha=0.9))
    ax.annotate("склиз D ↓0.32", ((cd["cx"] + 0.33) * MM, (cd["y1"] + 0.15) * MM), fontsize=7)

    # cages (open side dashed)
    for name, col in (("C", "#f4a7a3"), ("D", "#a3d9a5")):
        cg = cages[name]
        cx, cy = cg["center"]; ix, iy = cg["inner"]
        ax.add_patch(Rectangle(((cx - ix / 2) * MM, (cy - iy / 2) * MM), ix * MM, iy * MM, fc=col, ec="k"))
        ax.annotate(f"{name}: ролл-кейдж\n(открытый борт)", ((cx - ix / 2 + 0.05) * MM, (cy - 0.1) * MM), fontsize=7)

    # exception arm
    ax.add_patch(Circle((base[0] * MM, base[1] * MM), 150, fc="#444", zorder=5))
    ax.add_patch(Circle((base[0] * MM, base[1] * MM), P.ARM["reach"] * MM, fill=False, ec="#444", ls=":", lw=1.2))
    ax.add_patch(Circle((base[0] * MM, base[1] * MM), usable * MM, fill=False, ec="green", ls="--", lw=1))
    ax.annotate("рука-исключение /\nexception arm (reach 1.3 м)", (base[0] * MM - 500, (base[1] - 0.75) * MM), fontsize=8)
    for name, xyz in pts.items():
        ax.plot(xyz[0] * MM, xyz[1] * MM, marker="x", color="green", ms=9, mew=2, zorder=6)

    # fence
    ax.add_patch(Rectangle((6700, 1000, ), 3300, 5000, fill=False, ec="orange", lw=2))
    ax.annotate("ограждение + световая завеса, e-stop", (6780, 1090), color="darkorange", fontsize=8)

    ax.set_xlim(-300, 10300); ax.set_ylim(-300, 6300)
    ax.set_aspect("equal"); ax.set_xlabel("mm"); ax.set_ylabel("mm")
    ax.set_title("Transfer-table architecture — top view (generated from cell/params.py)")
    fig.tight_layout()
    fig.savefig(OUT / "layout_table_top_view.png", dpi=200)
    print(f"saved -> {OUT / 'layout_table_top_view.png'}")


if __name__ == "__main__":
    main()
