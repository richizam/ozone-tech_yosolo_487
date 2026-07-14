# -*- coding: utf-8 -*-
"""Engineering schema generator — draws the REQUIRED project materials
(схема компоновки участка перекладки + kinematic diagram of the tilt-tray
carrier) directly from cell/params.py, so the drawings are exact and can
never drift from the simulated build.

Outputs (docs/report/figures/):
  layout_plan.png         dimensioned top view: A feed, накопитель/escapement,
                          vision station, tilt-tray train, B incline + belt B,
                          C/D chutes + cages, REVIEW pen, exception arm
  carrier_kinematics.png  side-section kinematics: chain, trunnion, revolute
                          joint, tray sweep envelope (38 deg command,
                          46 deg joint limit), chute-mouth interface

Usage:  python tools/make_layout_schema.py
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import (Rectangle, Circle, FancyArrow, Arc, Polygon)
import math

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
from cell import params as P                                    # noqa: E402

FIG = REPO / "docs" / "report" / "figures"
FIG.mkdir(parents=True, exist_ok=True)

C_BELT = "#2b2e33"
C_STEEL = "#8d939c"
C_FRAME = "#5a6068"
C_B = "#4a8cf2"
C_C = "#f28c26"
C_D = "#33c759"
C_REV = "#b84099"


def dim_h(ax, x0, x1, y, label, off=0.09):
    ax.annotate("", (x0, y), (x1, y),
                arrowprops=dict(arrowstyle="<->", lw=0.9, color="k"))
    ax.text((x0 + x1) / 2, y + off, label, ha="center", va="bottom",
            fontsize=7.5)


def dim_v(ax, x, y0, y1, label, off=0.09):
    ax.annotate("", (x, y0), (x, y1),
                arrowprops=dict(arrowstyle="<->", lw=0.9, color="k"))
    ax.text(x + off, (y0 + y1) / 2, label, ha="left", va="center",
            fontsize=7.5, rotation=90)


def layout_plan():
    a, s = P.BELT_A, P.SORTER
    bc, bb = P.B_CONNECT, P.BELT_B
    fig, ax = plt.subplots(figsize=(15, 9), dpi=170)

    # work zone 10 x 6 m
    ax.add_patch(Rectangle((0, 0), *P.ZONE, fill=False, ec="crimson",
                           ls="--", lw=1.2))
    ax.text(0.08, 5.82, "Рабочая зона 10000 × 6000 мм (фикс. по схеме ТЗ)",
            color="crimson", fontsize=9)

    def point_marker(x, y, letter, note):
        ax.add_patch(Circle((x, y), 0.14, fc="white", ec="k", lw=1.6,
                            zorder=6))
        ax.text(x, y, letter, ha="center", va="center", fontsize=11,
                weight="bold", zorder=7)
        ax.text(x, y - 0.24, note, ha="center", va="top", fontsize=6.8,
                style="italic")

    # official flow points per the task schema (doc-1783095831 p.5)
    point_marker(0.35, a["y"] + 0.55, "A",
                 "подача 500×700 мм\n(ФИКС. по схеме ТЗ)")
    point_marker(P.BELT_B["cx"] - 1.05, 5.35, "B",
                 "инфид сортера 500×700 мм\n(ФИКС. относительно A)")
    point_marker(P.STATIONS["C"]["x"] - 0.75, 1.5, "C",
                 "ролл-кейдж 1200×800×800\n(свободное размещение)")
    point_marker(P.STATIONS["D"]["x"] + 0.75, 1.5, "D",
                 "ролл-кейдж 1200×800×800\n(свободное размещение)")

    # A: feed belt (fixed, 1 m/s)
    ax.add_patch(Rectangle((a["x0"], a["y"] - a["width"] / 2),
                           a["nose_x"] - a["x0"], a["width"],
                           fc=C_BELT, ec="k"))
    ax.text(2.0, a["y"], "A — подающий конвейер 500 мм · 1 м/с (ФИКС.)",
            color="w", fontsize=8.5, va="center")
    # escapement / накопитель
    for x, nm in ((a["hold2_x"], "предгейт"), (a["gate_x"], "эскейпмент")):
        ax.plot([x, x], [a["y"] - a["width"] / 2, a["y"] + a["width"] / 2],
                color="#ffd23f", lw=2.5)
        ax.text(x, a["y"] - a["width"] / 2 - 0.12, f"{nm}\nx={x:.2f}",
                ha="center", va="top", fontsize=7)
    ax.text(5.85, a["y"] + a["width"] / 2 + 0.34,
            "НАКОПИТЕЛЬ (метрирование:\nлента не останавливается)",
            ha="center", fontsize=7.5, style="italic")
    # vision станция
    vx = P.VIRTUAL_SENSOR["overhead_pos"][0]
    ax.add_patch(Rectangle((vx - 0.22, a["y"] - 1.05), 0.44, 2.10,
                           fill=False, ec="#7a5cff", lw=1.4, ls=":"))
    ax.text(vx, a["y"] + 1.18, "зона измерения\n(RTX depth ×3 + макро)",
            ha="center", fontsize=7.5, color="#7a5cff")

    # tilt-tray train
    ax.add_patch(Rectangle((s["x_west"], s["y"] - 0.38),
                           s["x_east"] - s["x_west"], 0.76,
                           fc="#c9ccd2", ec="k"))
    n_vis = int((s["x_east"] - s["x_west"]) // s["pitch"])
    for i in range(n_vis):
        x0 = s["x_west"] + i * s["pitch"] + 0.005
        ax.add_patch(Rectangle((x0, s["y"] - s["tray_w"] / 2),
                               s["tray_l"], s["tray_w"],
                               fc="#aeb3ba", ec="#5a6068", lw=0.7))
    ax.text((s["x_west"] + s["x_east"]) / 2, s["y"] + 0.5,
            f"ТИЛТ-ЛОТОЧНЫЙ СОРТЕР — {s['n_carriers']} кареток, "
            f"шаг {s['pitch']*1000:.0f} мм, v={s['v_mps']} м/с, "
            f"наклон ±{s['tilt_deg']:.0f}°",
            ha="center", fontsize=8.5, weight="bold")

    # stations + chutes + cages
    st = P.STATIONS
    for z, col in (("C", C_C), ("D", C_D)):
        x = st[z]["x"]
        cc = P.CHUTE_C if z == "C" else P.CHUTE_D
        y1, pad_end = P.chute_run(cc)
        ax.add_patch(Polygon([(x - 0.35, cc["y0"]), (x + 0.35, cc["y0"]),
                              (x + 0.35, y1), (x - 0.35, y1)],
                             fc=col, alpha=0.25, ec=col))
        cage = P.CAGES["sorter"][z]
        cx, cy = cage["center"]
        ax.add_patch(Rectangle((cx - cage["inner"][0] / 2 - 0.03,
                                cy - cage["inner"][1] / 2 - 0.03),
                               cage["inner"][0] + 0.06,
                               cage["inner"][1] + 0.06,
                               fill=False, ec=col, lw=1.8))
        ax.text(cx, cy, f"{z}\nролл-кейдж", ha="center", va="center",
                fontsize=8.5, color=col, weight="bold")
    # REVIEW pen (north)
    x = st["REVIEW"]["x"]
    ccr = P.CHUTE_REVIEW
    y1r, _ = P.chute_run(ccr)
    ax.add_patch(Polygon([(x - 0.35, ccr["y0"]), (x + 0.35, ccr["y0"]),
                          (x + 0.35, y1r), (x - 0.35, y1r)],
                         fc=C_REV, alpha=0.25, ec=C_REV))
    pen = P.REVIEW_PEN
    ax.add_patch(Rectangle((pen["center"][0] - pen["inner"][0] / 2 - 0.03,
                            pen["center"][1] - pen["inner"][1] / 2 - 0.03),
                           pen["inner"][0] + 0.06, pen["inner"][1] + 0.06,
                           fill=False, ec=C_REV, lw=1.8))
    ax.text(pen["center"][0], pen["center"][1], "РУЧНОЙ\nРАЗБОР",
            ha="center", va="center", fontsize=7.5, color=C_REV)
    ax.text(pen["center"][0] + 0.48, pen["center"][1] - 0.42,
            "(доп. поток нашей архитектуры:\nнеуверенная классификация)",
            ha="left", fontsize=6.5, color=C_REV, style="italic")

    # B connector + belt B
    ax.add_patch(Polygon([(bc["cx"] - bc["width"] / 2, bc["y0"]),
                          (bc["cx"] + bc["width"] / 2, bc["y0"]),
                          (bc["cx"] + bc["width"] / 2, bc["y1"]),
                          (bc["cx"] - bc["width"] / 2, bc["y1"])],
                         fc=C_B, alpha=0.30, ec=C_B))
    ax.text(bc["cx"] - 0.45, (bc["y0"] + bc["y1"]) / 2,
            f"наклонный конвейер B\n0.42→0.70 м · {bc['speed']} м/с",
            fontsize=7.5, color=C_B, va="center", ha="right")
    ax.add_patch(Rectangle((bb["cx"] - bb["width"] / 2, bb["y0"]),
                           bb["width"], bb["y1"] - bb["y0"],
                           fc=C_BELT, ec="k"))
    ax.text(bb["cx"], (bb["y0"] + bb["y1"]) / 2,
            "B — инфид сортера (ФИКС.)", color="w", ha="center",
            va="center", fontsize=8, rotation=90)

    # exception arm
    ax.add_patch(Circle(P.ARM["base_xy"] if hasattr(P, "ARM") and
                        isinstance(getattr(P, "ARM"), dict) and
                        "base_xy" in P.ARM else (8.10, 2.42), 0.16,
                        fc="#cfe8ff", ec="#3a76c4", lw=1.5))
    ax.text(8.10, 2.42, "UR", ha="center", va="center", fontsize=7.5,
            color="#3a76c4", weight="bold")
    ax.text(8.10, 2.10, "манипулятор разбора\nнештатных ситуаций",
            ha="center", va="top", fontsize=7)

    # dimensions
    dim_h(ax, 0, a["nose_x"], 0.42, f"{a['nose_x']*1000:.0f} мм (лента A)")
    dim_h(ax, s["x_west"], s["x_east"], 1.32,
          f"{(s['x_east']-s['x_west'])*1000:.0f} мм (верхняя ветвь)")
    dim_v(ax, 9.72, 3.0, 6.0, f"{3.0*1000:.0f} мм")
    for z, col, dx in (("C", C_C, 0.0), ("B", C_B, -0.62), ("D", C_D, 0.0),
                       ("REVIEW", C_REV, 0.72)):
        x = st[z]["x"]
        side = st[z]["side"]
        ax.annotate(f"ст. {z}  x={x:.2f}",
                    (x, s["y"] + side * 0.40),
                    (x + dx, s["y"] + side * (0.40 + 0.42)),
                    ha="center", fontsize=7.5, color=col,
                    arrowprops=dict(arrowstyle="->", color=col))

    ax.set_xlim(-0.3, 10.6)
    ax.set_ylim(-0.3, 6.3)
    ax.set_aspect("equal")
    ax.set_title("Схема компоновки участка перекладки — генерируется из "
                 "cell/params.py (единый источник геометрии симуляции)",
                 fontsize=10)
    ax.set_xlabel("x, м")
    ax.set_ylabel("y, м")
    ax.grid(alpha=0.15, lw=0.4)
    fig.tight_layout()
    fig.savefig(FIG / "layout_plan.png")
    plt.close(fig)
    print(f"wrote {FIG/'layout_plan.png'}")


def carrier_kinematics():
    s = P.SORTER
    fig, ax = plt.subplots(figsize=(11, 8), dpi=170)
    y0 = 0.0                                   # train axis in section view
    pz = s["pivot_z"]

    # chute mouth interface (south)
    cc = P.CHUTE
    ax.plot([-1.05, -(0.31)], [cc["z0"], cc["z0"]], color=C_C, lw=2)
    ax.text(-1.02, cc["z0"] + 0.012, "кромка лотка C/D  z=395",
            fontsize=7.5, color=C_C)

    # shuttle / trunnion (schematic, from build_carrier geometry)
    shut_c = s["shuttle_top"] - 0.025
    ax.add_patch(Rectangle((-0.026, shut_c - 0.017), 0.052, 0.036,
                           fc=C_FRAME, ec="k", lw=0.8))
    ax.annotate("хребтовая балка (спайн)", (0.028, shut_c),
                (0.30, 0.50), fontsize=7,
                arrowprops=dict(arrowstyle="->", lw=0.8))
    ax.add_patch(Rectangle((-0.075, shut_c - 0.10), 0.15, 0.05,
                           fc="#3c4046", ec="k", lw=0.8))
    ax.annotate("звено цепи (кинематич.\nпривод трассы)",
                (0.077, shut_c - 0.075), (0.30, 0.40), fontsize=7,
                arrowprops=dict(arrowstyle="->", lw=0.8))
    # shaft + saddle
    ax.add_patch(Circle((0, pz - 0.028), 0.008, fc="#e8e8ee", ec="k"))
    ax.annotate("вал Ø16 + седла-подшипники", (-0.008, pz - 0.028),
                (-0.92, 0.70), fontsize=7,
                arrowprops=dict(arrowstyle="->", lw=0.8))
    # revolute joint symbol
    ax.add_patch(Circle((0, pz), 0.016, fill=False, ec="crimson", lw=1.6))
    ax.plot(0, pz, "+", color="crimson", ms=8)
    ax.annotate(f"револьвентный шарнир (ось X)\nпривод: {s['drive_max_torque']:.0f} Н·м, "
                f"{s['tilt_rate_dps']:.0f}°/с, задержка {s['latency_s']*1000:.0f} мс",
                (0.016, pz), (0.26, pz + 0.16), fontsize=7.5,
                arrowprops=dict(arrowstyle="->", lw=0.8, color="crimson"),
                color="crimson")

    # tray flat + tilted
    w2 = s["tray_w"] / 2
    ax.plot([-w2, w2], [s["tray_top"], s["tray_top"]], color="k", lw=3)
    ax.text(w2 + 0.02, s["tray_top"], f"лоток {s['tray_l']*1000:.0f}×"
            f"{s['tray_w']*1000:.0f} мм (V-профиль 2°)", fontsize=7.5,
            va="center")
    for ang, col, lw, lab, at_top in (
            (s["tilt_deg"], "#1f77b4", 2.2,
             f"рабочий наклон {s['tilt_deg']:.0f}°", False),
            (s["tilt_deg"] + 8, "#9aa7b8", 1.4,
             "предел шарнира 46° (аварийный)", True)):
        r = math.radians(ang)
        x1, z1 = -w2 * math.cos(r), pz - w2 * math.sin(r)
        x2, z2 = w2 * math.cos(r), pz + w2 * math.sin(r)
        ax.plot([x1, x2], [z1, z2], color=col, lw=lw,
                ls="-" if lw > 2 else "--")
        if at_top:
            ax.text(x2 + 0.02, z2, lab, fontsize=7.5, color=col, ha="left")
        else:
            ax.text(x1 - 0.02, z1, lab, fontsize=7.5, color=col, ha="right")
    arc = Arc((0, pz), 0.5, 0.5, theta1=180, theta2=180 + s["tilt_deg"],
              color="#1f77b4", lw=1.2)
    ax.add_patch(arc)

    # discharge fall corridor + cage sill
    ax.annotate("", (-0.55, 0.20), (-0.31, cc["z0"]),
                arrowprops=dict(arrowstyle="->", lw=1.6, color=C_C))
    ax.text(-0.62, 0.26, "гравитационный\nсход 32°", fontsize=7.5,
            color=C_C, ha="center")

    # dims
    dim_v(ax, 0.62, 0.0, s["tray_top"],
          f"верх лотка {s['tray_top']*1000:.0f} мм")
    dim_v(ax, 0.76, 0.0, pz, f"ось наклона {pz*1000:.0f} мм")
    dim_v(ax, -1.12, 0.0, cc["z0"], f"{cc['z0']*1000:.0f} мм")
    ax.plot([-1.2, 1.0], [0, 0], color="k", lw=1)
    for x in [-1.2 + 0.06 * i for i in range(37)]:
        ax.plot([x, x - 0.03], [0, -0.025], color="k", lw=0.6)

    ax.set_xlim(-1.35, 1.35)
    ax.set_ylim(-0.08, 1.05)
    ax.set_aspect("equal")
    ax.set_title("Кинематическая схема каретки (поперечное сечение) — "
                 "генерируется из cell/params.py", fontsize=10)
    ax.set_xlabel("y, м (поперек трассы)")
    ax.set_ylabel("z, м")
    ax.grid(alpha=0.15, lw=0.4)
    fig.tight_layout()
    fig.savefig(FIG / "carrier_kinematics.png")
    plt.close(fig)
    print(f"wrote {FIG/'carrier_kinematics.png'}")


if __name__ == "__main__":
    layout_plan()
    carrier_kinematics()
