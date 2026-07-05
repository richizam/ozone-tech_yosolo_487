# -*- coding: utf-8 -*-
"""Parametric cell layout v0 — first executive-part CAD artifact.

Generates from one set of parameters (all mm, per the official scheme
doc-1783009942.pdf — fixed elements respected, free elements placed by us):

  * cad/out/layout_top_view.png   — dimensioned top view with arm reach check
  * cad/out/cell_layout_v0.glb    — 3D scene (viewable in any glTF viewer / Blender / FreeCAD)
  * printed reach-feasibility table (asserts every pick/place point is reachable)

Fixed by the organizers:  work zone 6000x10000; conveyor A axis and profile
(width 500, top height 700); conveyor B profile and its position relative to A;
cages are 1200x800x800 roll-cages; belt speed 1 m/s.
Chosen by us (free per the rules): conveyor A length, accumulator design,
arm base position, C/D cage positions, fence, camera position.

The layout is authored HERE as code so every dimension has a single source of
truth; Phase 3 re-exports the frozen result as STEP + ЕСКД-style PDF drawings
via FreeCAD (or KOMPAS-3D if a license is available — neutral formats are what
the submission requires either way).
"""
import json
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrow

OUT = Path(__file__).parent / "out"
OUT.mkdir(parents=True, exist_ok=True)

# ----------------------------------------------------------------------------- parameters
P = {
    # work zone (fixed)
    "zone": (10000.0, 6000.0),
    # conveyor A (axis fixed at y=3000, profile fixed; LENGTH is ours to choose)
    "a_y": 3000.0, "a_width": 500.0, "a_height": 700.0,
    "a_x0": 0.0, "a_x1": 7500.0,          # we extend A deep into the cell (allowed)
    # accumulator tray at the end of A (design is ours)
    "acc_len": 600.0,                       # x: 7500..8100
    # vision station (ours): overhead RGB-D across A, upstream of accumulator
    "cam_x": 6000.0,                        # 1500 mm before accumulator start = 1.5 s @ 1 m/s
    # arm (ours): UR10-class, reach 1300
    "arm_base": (8400.0, 3250.0), "arm_reach": 1300.0, "reach_margin": 150.0,
    # conveyor B (fixed relative to A per scheme: parallel offset, runs to zone edge)
    "b_cx": 8400.0, "b_y0": 4200.0, "b_y1": 6000.0, "b_width": 500.0,
    # roll-cages 1200 x 800 x 800 (positions ours)
    "cage_c_center": (9250.0, 2950.0), "cage_c_size": (1200.0, 800.0),   # oversize
    "cage_d_center": (8350.0, 2000.0), "cage_d_size": (1200.0, 800.0),   # repack
    # safety fence (ours) with light curtain on the service aisle
    "fence": (6700.0, 1200.0, 10000.0, 6000.0),  # x0, y0, x1, y1
}

# pick / place points the arm must reach
acc_center = (P["a_x1"] + P["acc_len"] / 2.0, P["a_y"])            # pick from accumulator
b_infeed = (P["b_cx"], P["b_y0"] + 150.0)                          # place onto B belt
c_drop = (P["cage_c_center"][0] - P["cage_c_size"][0] / 2 + 250.0, P["cage_c_center"][1])
d_drop = (P["cage_d_center"][0], P["cage_d_center"][1] + P["cage_d_size"][1] / 2 - 250.0)

POINTS = {"PICK accumulator": acc_center, "PLACE B (sorter)": b_infeed,
          "DROP C (oversize)": c_drop, "DROP D (repack)": d_drop}


def reach_report():
    base = np.array(P["arm_base"])
    usable = P["arm_reach"] - P["reach_margin"]
    print(f"Arm base {tuple(base)}  reach {P['arm_reach']:.0f} mm  usable (margin {P['reach_margin']:.0f}): {usable:.0f} mm")
    ok_all = True
    rows = []
    for name, pt in POINTS.items():
        d = float(np.linalg.norm(np.array(pt) - base))
        ok = d <= usable
        ok_all &= ok
        rows.append({"point": name, "xy_mm": [round(pt[0]), round(pt[1])], "dist_mm": round(d), "reachable": ok})
        print(f"  {name:<20} {str(tuple(round(c) for c in pt)):<16} d={d:7.0f} mm  {'OK' if ok else '!! OUT OF REACH'}")
    (OUT / "reach_check.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    if not ok_all:
        raise SystemExit("Layout infeasible — adjust parameters.")
    return rows


# ----------------------------------------------------------------------------- 2D top view
def draw_top_view():
    fig, ax = plt.subplots(figsize=(14, 9))
    zx, zy = P["zone"]
    ax.add_patch(Rectangle((0, 0), zx, zy, fill=False, ec="red", ls="--", lw=1.5))
    ax.text(80, zy - 200, "Рабочая зона / work zone 10000×6000", color="red", fontsize=9)

    # conveyor A + accumulator
    ax.add_patch(Rectangle((P["a_x0"], P["a_y"] - P["a_width"] / 2), P["a_x1"] - P["a_x0"], P["a_width"],
                           fc="#b0c4de", ec="k"))
    ax.add_patch(Rectangle((P["a_x1"], P["a_y"] - P["a_width"] / 2), P["acc_len"], P["a_width"],
                           fc="#ffd27f", ec="k"))
    ax.annotate("A: конвейер 1 м/с (h=700)", (1500, P["a_y"] + 350), fontsize=9)
    ax.annotate("накопитель /\naccumulator", (P["a_x1"] - 250, P["a_y"] - 950), fontsize=8)
    ax.add_patch(FancyArrow(1200, P["a_y"], 900, 0, width=40, color="k"))

    # vision station
    ax.plot([P["cam_x"], P["cam_x"]], [P["a_y"] - 600, P["a_y"] + 600], color="purple", lw=2)
    ax.annotate("RGB-D камера (look-ahead 1.5 s)", (P["cam_x"] - 1650, P["a_y"] + 700), color="purple", fontsize=8)

    # conveyor B
    ax.add_patch(Rectangle((P["b_cx"] - P["b_width"] / 2, P["b_y0"]), P["b_width"], P["b_y1"] - P["b_y0"],
                           fc="#b0c4de", ec="k"))
    ax.annotate("B: инфид сортера (fixed)", (P["b_cx"] - 1100, P["b_y1"] - 350), fontsize=9)

    # cages
    for key, label, color in (("cage_c", "C: негабарит\n1200×800×800", "#f4a7a3"),
                              ("cage_d", "D: доупаковка\n1200×800×800", "#a3d9a5")):
        cx, cy = P[f"{key}_center"]; w, h = P[f"{key}_size"]
        ax.add_patch(Rectangle((cx - w / 2, cy - h / 2), w, h, fc=color, ec="k"))
        ax.annotate(label, (cx - w / 2 + 60, cy - 100), fontsize=8)

    # arm + reach
    bx, by = P["arm_base"]
    ax.add_patch(Circle((bx, by), 150, fc="#444", ec="k", zorder=5))
    ax.add_patch(Circle((bx, by), P["arm_reach"], fill=False, ec="#444", ls=":", lw=1.5))
    ax.add_patch(Circle((bx, by), P["arm_reach"] - P["reach_margin"], fill=False, ec="green", ls="--", lw=1))
    ax.annotate("UR10-class arm\nreach 1300 (margin 150)", (bx - 500, by - 2350), fontsize=8)

    for name, pt in POINTS.items():
        ax.plot(*pt, marker="x", color="green", ms=10, mew=2, zorder=6)
        ax.plot([bx, pt[0]], [by, pt[1]], color="green", lw=0.8, alpha=0.6)

    # fence + light curtain
    fx0, fy0, fx1, fy1 = P["fence"]
    ax.add_patch(Rectangle((fx0, fy0), fx1 - fx0, fy1 - fy0, fill=False, ec="orange", lw=2))
    ax.plot([fx0, fx0], [P["a_y"] + 400, fy1], color="orange", lw=4, alpha=0.5)
    ax.annotate("ограждение + световая завеса /\nfence + light curtain, e-stop", (fx0 + 80, fy0 + 120),
                color="darkorange", fontsize=8)

    ax.set_xlim(-300, zx + 300); ax.set_ylim(-300, zy + 300)
    ax.set_aspect("equal"); ax.set_xlabel("mm"); ax.set_ylabel("mm")
    ax.set_title("Cell layout v0 — top view (fixed elements per official scheme; free elements = our design)")
    fig.tight_layout()
    fig.savefig(OUT / "layout_top_view.png", dpi=200)
    print(f"Saved -> {OUT / 'layout_top_view.png'}")


# ----------------------------------------------------------------------------- 3D scene
def build_3d():
    import trimesh
    from trimesh.creation import box, cylinder

    S = trimesh.Scene()

    def add(mesh, name, color, transform=None):
        mesh.visual.face_colors = color
        if transform is not None:
            mesh.apply_transform(transform)
        S.add_geometry(mesh, node_name=name)

    def T(x, y, z):
        m = np.eye(4); m[:3, 3] = [x, y, z]; return m

    zx, zy = P["zone"]
    add(box(extents=[zx, zy, 20]), "floor", [220, 220, 220, 255], T(zx / 2, zy / 2, -10))

    # conveyor A (top surface at 700) + accumulator
    a_len = P["a_x1"] - P["a_x0"]
    add(box(extents=[a_len, P["a_width"], P["a_height"]]), "conveyor_A", [120, 140, 200, 255],
        T(P["a_x0"] + a_len / 2, P["a_y"], P["a_height"] / 2))
    add(box(extents=[P["acc_len"], P["a_width"], P["a_height"]]), "accumulator", [255, 200, 100, 255],
        T(P["a_x1"] + P["acc_len"] / 2, P["a_y"], P["a_height"] / 2))

    # conveyor B
    b_len = P["b_y1"] - P["b_y0"]
    add(box(extents=[P["b_width"], b_len, P["a_height"]]), "conveyor_B", [120, 140, 200, 255],
        T(P["b_cx"], P["b_y0"] + b_len / 2, P["a_height"] / 2))

    # cages: floor + 4 walls, height 800
    for key, color in (("cage_c", [235, 130, 120, 255]), ("cage_d", [120, 200, 130, 255])):
        cx, cy = P[f"{key}_center"]; w, h = P[f"{key}_size"]; hh = 800.0; t = 30.0
        add(box(extents=[w, h, t]), f"{key}_floor", color, T(cx, cy, t / 2))
        for i, (dx, dy, ex, ey) in enumerate([(0, h / 2, w, t), (0, -h / 2, w, t),
                                              (w / 2, 0, t, h), (-w / 2, 0, t, h)]):
            add(box(extents=[ex, ey, hh]), f"{key}_wall{i}", color, T(cx + dx, cy + dy, hh / 2))

    # arm pedestal + stylized links + translucent reach envelope
    bx, by = P["arm_base"]
    add(cylinder(radius=150, height=650), "arm_pedestal", [70, 70, 70, 255], T(bx, by, 325))
    add(cylinder(radius=80, height=900), "arm_link1", [90, 90, 100, 255], T(bx, by, 650 + 450))
    sphere = trimesh.creation.icosphere(subdivisions=3, radius=P["arm_reach"])
    add(sphere, "reach_envelope", [80, 160, 80, 40], T(bx, by, 800))

    # camera post + crossbar over A
    add(cylinder(radius=40, height=1800), "cam_post", [120, 60, 160, 255], T(P["cam_x"], P["a_y"] - 800, 900))
    add(box(extents=[60, 1600, 60]), "cam_bar", [120, 60, 160, 255], T(P["cam_x"], P["a_y"], 1780))

    # fence posts
    fx0, fy0, fx1, fy1 = P["fence"]
    for i, (x, y) in enumerate([(x, y) for x in np.arange(fx0, fx1 + 1, 825) for y in (fy0, fy1)] +
                               [(fx0, y) for y in np.arange(fy0, fy1 + 1, 800)]):
        add(box(extents=[50, 50, 1800]), f"fence_{i}", [255, 165, 0, 200], T(x, y, 900))

    out = OUT / "cell_layout_v0.glb"
    S.export(out)
    print(f"Saved -> {out}")


if __name__ == "__main__":
    reach_report()
    draw_top_view()
    build_3d()
