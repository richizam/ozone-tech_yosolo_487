# -*- coding: utf-8 -*-
"""MJCF scene generation from cell.params — the layout has one source of truth.

Two executive architectures share one generator (build_xml(mode)):
  "table" (primary): conveyor A -> escapement gate -> widened tri-directional
          transfer table with ACTUATED EXIT GATES; lanes exit north to the B
          connector, east and south over guided two-stage brake chutes into
          aperture-walled roll cages. The arm stands at an exception station.
  "arm"   (preserved baseline): conveyor A -> accumulator with stop wall; the
          arm picks every item (tag arm-primary-baseline).

Containment-by-design (scored under «Качество манипуляции», «отсутствие
избыточного брака»): items are guided and bounded at every hop — funnel rails
on belt A, edge rails + normally-closed lift gates on the table, side-railed
chutes with a high-friction brake runout that releases the item just above
the cage floor, and cage walls that are closed except for an entry aperture
sized to the chute. Nothing is thrown; nothing free-falls more than ~130 mm.

Presentation layer (video/jury): bilingual signage at every station, floor
decals, zone-coloured lane markings + chevrons, destination beacons, gate
lamps and an andon tower — all contype/conaffinity 0 (zero physics cost),
animated at runtime by cell/visuals.py.
"""
import json
from pathlib import Path

import numpy as np

from cell import params as P

ASSETS = Path(__file__).parent / "assets"

STEEL = "0.55 0.57 0.62 1"
DARK = "0.22 0.24 0.28 1"


def load_manifest():
    return json.loads((ASSETS / "manifest.json").read_text(encoding="utf-8"))


def _rgba(zone, scale=1.0, alpha=1.0):
    r, g, b = P.ROUTE_RGBA[zone]
    return f"{r * scale:.3f} {g * scale:.3f} {b * scale:.3f} {alpha}"


# --------------------------------------------------------------------- cages
def _cage_xml(name, cage):
    """Roll-cage 1200x800x800: frame posts + semi-transparent walls (the jury
    can see the item stay inside). A closed cage has four full walls; a cage
    fed by a chute has an entry APERTURE (flanks + header strip) instead of a
    missing wall, so items cannot bounce back out."""
    cx, cy = cage["center"]
    ix, iy = cage["inner"]
    t = cage["wall_t"]
    h = cage["wall_h"]
    hx, hy, hh = ix / 2, iy / 2, h / 2
    wall = _rgba(name, 0.9, 0.45)
    frame = _rgba(name, 1.0, 1.0)
    g = [f'<geom name="cage{name}_floor" type="box" size="{hx + t} {hy + t} {t / 2}" '
         f'pos="{cx} {cy} {t / 2}" rgba="{_rgba(name, 0.55)}"/>']
    walls = {"+y": (0, hy + t / 2, hx + t, t / 2), "-y": (0, -(hy + t / 2), hx + t, t / 2),
             "+x": (hx + t / 2, 0, t / 2, hy), "-x": (-(hx + t / 2), 0, t / 2, hy)}
    open_side = cage.get("open_side")
    for side, (dx, dy, sx, sy) in walls.items():
        if side == open_side:
            # entry aperture instead of an open wall: flank strips + header +
            # under-runout skirt — the port is exactly chute-sized, nothing
            # can roll back out below or beside the runout
            aw2 = cage["aperture_w"] / 2
            top = cage["aperture_top"]
            skirt_h2 = 0.165             # skirt 0..0.33: overlaps the slope
                                         # UNDERSIDE (~0.328 at the wall) but
                                         # stays 3 cm below the riding surface
                                         # so box corners can never clip it
            hdr_h = (t + h - top) / 2
            if side in ("+x", "-x"):
                for sgn, nm in ((1, "a"), (-1, "b")):
                    fl = (sy - aw2) / 2
                    g.append(f'<geom name="cage{name}_fl{nm}" type="box" size="{sx} {fl} {hh}" '
                             f'pos="{cx + dx} {cy + sgn * (aw2 + fl)} {t + hh}" rgba="{wall}"/>')
                if hdr_h > 0.005:
                    g.append(f'<geom name="cage{name}_hdr" type="box" size="{sx} {aw2} {hdr_h}" '
                             f'pos="{cx + dx} {cy} {top + hdr_h}" rgba="{wall}"/>')
                g.append(f'<geom name="cage{name}_skirt" type="box" size="{sx} {aw2} {skirt_h2}" '
                         f'pos="{cx + dx} {cy} {skirt_h2}" rgba="{wall}"/>')
            else:
                for sgn, nm in ((1, "a"), (-1, "b")):
                    fl = (sx - aw2) / 2
                    g.append(f'<geom name="cage{name}_fl{nm}" type="box" size="{fl} {sy} {hh}" '
                             f'pos="{cx + sgn * (aw2 + fl)} {cy + dy} {t + hh}" rgba="{wall}"/>')
                if hdr_h > 0.005:
                    g.append(f'<geom name="cage{name}_hdr" type="box" size="{aw2} {sy} {hdr_h}" '
                             f'pos="{cx} {cy + dy} {top + hdr_h}" rgba="{wall}"/>')
                g.append(f'<geom name="cage{name}_skirt" type="box" size="{aw2} {sy} {skirt_h2}" '
                         f'pos="{cx} {cy + dy} {skirt_h2}" rgba="{wall}"/>')
        else:
            g.append(f'<geom name="cage{name}_w{side}" type="box" size="{sx} {sy} {hh}" '
                     f'pos="{cx + dx} {cy + dy} {t + hh}" rgba="{wall}"/>')
    # high-friction landing mat over the whole cage floor: kills the residual
    # slide so items settle in place instead of ramming the far wall
    if open_side:
        g.append(f'<geom name="cage{name}_mat" type="box" size="{hx} {hy} 0.004" '
                 f'pos="{cx} {cy} {t + 0.004}" friction="{P.CAGE_MAT_FRICTION}" '
                 f'priority="2" rgba="0.15 0.15 0.17 1"/>')
    # roll-cage frame: corner posts + top rails (visual only)
    for sx in (-1, 1):
        for sy in (-1, 1):
            g.append(f'<geom name="cage{name}_p{sx}{sy}" type="cylinder" size="0.022 {(t + h) / 2}" '
                     f'pos="{cx + sx * (hx + t)} {cy + sy * (hy + t)} {(t + h) / 2}" '
                     f'rgba="{frame}" contype="0" conaffinity="0" group="2"/>')
    for ax, nm in (("x", "tx"), ("y", "ty")):
        for sgn in (-1, 1):
            if ax == "x":
                g.append(f'<geom name="cage{name}_{nm}{sgn}" type="box" size="{hx + t} 0.012 0.012" '
                         f'pos="{cx} {cy + sgn * (hy + t)} {t + h}" rgba="{frame}" '
                         f'contype="0" conaffinity="0" group="2"/>')
            else:
                g.append(f'<geom name="cage{name}_{nm}{sgn}" type="box" size="0.012 {hy + t} 0.012" '
                         f'pos="{cx + sgn * (hx + t)} {cy} {t + h}" rgba="{frame}" '
                         f'contype="0" conaffinity="0" group="2"/>')
    return g


# --------------------------------------------------------------------- chutes
def _incline_xml(name, center, half, euler, rgba="0.55 0.55 0.6 1",
                 friction="0.12 0.005 0.0001"):
    # priority=1: the polished chute surface DICTATES the contact friction
    # (otherwise MuJoCo takes the max of both geoms and sticky items freeze
    # on the slope when placed without momentum)
    return (f'<geom name="{name}" type="box" size="{half[0]} {half[1]} {half[2]}" '
            f'pos="{center[0]} {center[1]} {center[2]}" euler="{euler[0]} {euler[1]} {euler[2]}" '
            f'friction="{friction}" priority="1" rgba="{rgba}"/>')


def _chute_xml(zone, cc, axis, wall_at):
    """One continuous guided slope (32 deg, mu < tan(32 deg): items can never
    rest on it) through the cage aperture onto a flat high-friction BRAKE PAD
    just above the cage floor. Side guides run from the table edge to the
    cage wall plane (`wall_at`); inside the cage the flanks take over."""
    g = []
    rail_t = 0.015
    frame = _rgba(zone, 1.0, 1.0)
    if axis == "x":
        p0, p1, z0, z1 = cc["x0"], cc["x1"], cc["z0"], cc["z1"]
        pad_end = cc["pad_x1"]
        fixed = cc["cy"]
    else:
        p0, p1, z0, z1 = cc["y0"], cc["y1"], cc["z0"], cc["z1"]
        pad_end = cc["pad_y1"]
        fixed = cc["cx"]
    length = float(np.hypot(p1 - p0, z0 - z1))
    ang = float(np.arctan2(z0 - z1, abs(p1 - p0)))
    # tilt sign: Ry(+ang) drops the +x side (C descends eastward);
    # Rx(+ang) raises the +y side (D descends southward) — both downhill
    euler_seg = (0, ang, 0) if axis == "x" else (ang, 0, 0)
    mid, zmid = (p0 + p1) / 2, (z0 + z1) / 2 - 0.015
    if axis == "x":
        center, half = (mid, fixed, zmid), (length / 2, cc["width"] / 2, 0.015)
    else:
        center, half = (fixed, mid, zmid), (cc["width"] / 2, length / 2, 0.015)
    g.append(_incline_xml(f"chute{zone}", center, half, euler_seg,
                          "0.55 0.55 0.6 1", cc["friction"]))
    # side guides out to the cage wall plane only
    r1 = wall_at
    rmid = (p0 + r1) / 2
    rlen = float(np.hypot(r1 - p0, (z0 - z1) * abs(r1 - p0) / abs(p1 - p0)))
    for sgn, nm in ((1, "l"), (-1, "r")):
        off = sgn * (cc["width"] / 2 + rail_t)
        rz = z0 - (z0 - z1) * abs(rmid - p0) / abs(p1 - p0) + P.GUIDE_H / 2
        if axis == "x":
            rc, rh = (rmid, fixed + off, rz), (rlen / 2, rail_t, P.GUIDE_H / 2)
        else:
            rc, rh = (fixed + off, rmid, rz), (rail_t, rlen / 2, P.GUIDE_H / 2)
        g.append(_incline_xml(f"chute{zone}_rail_{nm}", rc, rh, euler_seg,
                              frame, cc["friction"]))
    # brake pad: flat, 5 mm below the slope tail (a downhill step, never a lip)
    pz = z1 - 0.005
    pmid = (p1 + pad_end) / 2
    plen = abs(pad_end - p1)
    if axis == "x":
        pc, ph = (pmid, fixed, pz - 0.015), (plen / 2, cc["width"] / 2, 0.015)
    else:
        pc, ph = (fixed, pmid, pz - 0.015), (cc["width"] / 2, plen / 2, 0.015)
    g.append(_incline_xml(f"chute{zone}_pad", pc, ph, (0, 0, 0),
                          "0.30 0.31 0.35 1", cc["pad_friction"]))

    # HOOD over the aperture: a cover PARALLEL to the slope (never a catch
    # face) with a flared funnel mouth and a brow strip on the wall plane —
    # together they close the fly-out window above the flow
    hood = P.HOOD
    tanang = float(np.tan(ang))
    dirn = 1.0 if (axis == "x") else -1.0        # downhill direction sign
    aw2 = 0.40                                    # matches cage aperture_w/2

    def surf(c):
        return z0 - abs(c - p0) * tanang

    n_up = float(np.cos(ang))                     # vertical rise per normal unit
    s0 = wall_at - dirn * hood["up"]
    s1 = wall_at + dirn * hood["into"]
    hmid = (s0 + s1) / 2
    hlen = float(np.hypot(s1 - s0, abs(s1 - s0) * tanang))
    zc = surf(hmid) + hood["clearance"] * n_up
    lowfric = "0.2 0.005 0.0001"
    hood_col = _rgba(zone, 0.9, 0.45)
    if axis == "x":
        hcz, hh_ = (hmid, fixed, zc), (hlen / 2, aw2, 0.012)
    else:
        hcz, hh_ = (fixed, hmid, zc), (aw2, hlen / 2, 0.012)
    g.append(_incline_xml(f"hood{zone}", hcz, hh_, euler_seg, hood_col, lowfric))
    # brow: closes the strip between hood top and wall top on the wall plane
    bz0, bz1 = hood["brow_z0"], 0.83
    bh2 = (bz1 - bz0) / 2
    if axis == "x":
        bc, bh_ = (wall_at, fixed, bz0 + bh2), (0.015, aw2, bh2)
    else:
        bc, bh_ = (fixed, wall_at, bz0 + bh2), (aw2, 0.015, bh2)
    g.append(f'<geom name="hood{zone}_brow" type="box" size="{bh_[0]} {bh_[1]} {bh_[2]}" '
             f'pos="{bc[0]} {bc[1]} {bc[2]}" rgba="{hood_col}"/>')
    # zone-coloured flow arrows painted on the slope (visual)
    n_dash = max(3, int(length / 0.20))
    for i in range(n_dash):
        f_ = (i + 0.5) / n_dash
        da = p0 + (p1 - p0) * f_
        dz = z0 + (z1 - z0) * f_ + 0.004
        if axis == "x":
            dc, dh = (da, fixed, dz), (0.055, 0.035, 0.001)
        else:
            dc, dh = (fixed, da, dz), (0.035, 0.055, 0.001)
        g.append(f'<geom name="flow{zone}_{i}" type="box" size="{dh[0]} {dh[1]} {dh[2]}" '
                 f'pos="{dc[0]} {dc[1]} {dc[2]}" euler="{euler_seg[0]} {euler_seg[1]} {euler_seg[2]}" '
                 f'rgba="{_rgba(zone, 1.0, 0.85)}" contype="0" conaffinity="0" group="2"/>')
    return g


# ---------------------------------------------------------------- table gates
def _gate_xml(zone):
    """Normally-closed vertical-lift exit gate: the route command is the only
    thing that opens a path off the table. Panel is a real collider driven by
    a position actuator; frame posts + status lamp are visual."""
    gp = P.GATES[zone]
    tb = P.TABLE
    span = (gp["c1"] - gp["c0"]) / 2
    mid = (gp["c0"] + gp["c1"]) / 2
    t2, h2 = P.GATES["panel_t"] / 2, P.GATES["panel_h"] / 2
    zc = tb["top"] + P.GATES["gap"] + h2
    if gp["axis"] == "x":                # panel long side along x, at y = line
        px, py = mid, gp["line"]
        size = f"{span} {t2} {h2}"
        posts = [(gp["c0"] - 0.035, gp["line"]), (gp["c1"] + 0.035, gp["line"])]
    else:                                # panel long side along y, at x = line
        px, py = gp["line"], mid
        size = f"{t2} {span} {h2}"
        posts = [(gp["line"], gp["c0"] - 0.035), (gp["line"], gp["c1"] + 0.035)]
    body = (
        f'<body name="gate{zone}" pos="{px} {py} {zc}">\n'
        f'      <joint name="gj{zone}" type="slide" axis="0 0 1" range="-0.002 {P.GATES["travel"]}" damping="18"/>\n'
        f'      <geom name="gate{zone}_panel" type="box" size="{size}" '
        f'friction="0.2 0.005 0.0001" priority="1" rgba="{_rgba(zone, 0.85, 0.9)}"/>\n'
        f'    </body>')
    extras = []
    top_z = tb["top"] + P.GATES["travel"] + P.GATES["panel_h"] + 0.06
    for i, (qx, qy) in enumerate(posts):
        extras.append(f'<geom name="gate{zone}_post{i}" type="cylinder" size="0.025 {top_z / 2}" '
                      f'pos="{qx} {qy} {top_z / 2}" rgba="{STEEL}" contype="0" conaffinity="0" group="2"/>')
    extras.append(f'<geom name="glamp{zone}" type="box" size="0.055 0.055 0.03" '
                  f'pos="{px} {py} {top_z + 0.04}" rgba="{_rgba(zone, 0.35)}" '
                  f'contype="0" conaffinity="0" group="2"/>')
    act = (f'<position name="ga{zone}" joint="gj{zone}" kp="1400" kv="320" '
           f'forcerange="-600 600" ctrlrange="0 {P.GATES["travel"]}"/>')
    return body, extras, act


# ------------------------------------------------------- presentation statics
def _lane_visuals():
    """Zone-coloured lane dashes + chevrons on the table, roller/puck hints —
    the routing zone visibly IS a powered multi-directional roller field."""
    tb = P.TABLE
    g = []
    # entry-strip transport rollers (visual)
    for i, x in enumerate(np.arange(7.0, 7.95, 0.15)):
        g.append(f'<geom name="roller_{i}" type="cylinder" size="0.03 {tb["width"] / 2 - 0.02}" '
                 f'pos="{x:.3f} {tb["y"]} {tb["top"] - 0.015}" euler="1.5708 0 0" '
                 f'rgba="{DARK}" contype="0" conaffinity="0" group="2"/>')
    # omni-roller pucks in the routing zone (visual)
    k = 0
    for x in np.arange(8.06, 8.52, 0.15):
        for y in np.arange(2.62, 3.42, 0.19):
            g.append(f'<geom name="puck_{k}" type="cylinder" size="0.030 0.0025" '
                     f'pos="{x:.3f} {y:.3f} {tb["top"] + 0.002}" rgba="0.30 0.32 0.36 1" '
                     f'contype="0" conaffinity="0" group="2"/>')
            k += 1
    lanes = {
        "B": [((8.40, y), 0.0) for y in (3.08, 3.22, 3.36)],
        "C": [((x, 3.00), 90.0) for x in (8.10, 8.24, 8.38)],
        "D": [((8.05, y), 0.0) for y in (2.92, 2.78, 2.64)],
    }
    z = {"B": 0.7040, "C": 0.7046, "D": 0.7052}
    for zone, dashes in lanes.items():
        for i, ((x, y), yaw) in enumerate(dashes):
            g.append(f'<geom name="lane{zone}_{i}" type="box" size="0.048 0.026 0.001" '
                     f'pos="{x} {y} {z[zone]}" euler="0 0 {np.radians(yaw + 90):.4f}" '
                     f'rgba="{_rgba(zone, 0.5, 0.9)}" contype="0" conaffinity="0" group="2"/>')
    # chevron arrowheads at the three exits: two wings swept back from the tip
    chev = {"B": ((8.40, 3.48), 90.0), "C": ((8.50, 3.00), 0.0), "D": ((8.05, 2.52), -90.0)}
    for zone, ((x, y), deg) in chev.items():
        for j, sweep in enumerate((135.0, -135.0)):
            wa = float(np.radians(deg + sweep))
            g.append(f'<geom name="lane{zone}_ch{j}" type="box" size="0.062 0.018 0.001" '
                     f'pos="{x + 0.055 * np.cos(wa):.3f} {y + 0.055 * np.sin(wa):.3f} {z[zone]}" '
                     f'euler="0 0 {wa:.4f}" '
                     f'rgba="{_rgba(zone, 0.5, 0.9)}" contype="0" conaffinity="0" group="2"/>')
    return g


def _beacons():
    g = []
    spots = {"B": (8.72, 4.35), "C": (9.95, 3.66), "D": (8.05, 1.05)}
    for zone, (x, y) in spots.items():
        g.append(f'<geom name="beacon{zone}_pole" type="cylinder" size="0.02 0.66" '
                 f'pos="{x} {y} 0.66" rgba="{STEEL}" contype="0" conaffinity="0" group="2"/>')
        g.append(f'<geom name="beacon{zone}" type="sphere" size="0.058" '
                 f'pos="{x} {y} 1.40" rgba="{_rgba(zone, 0.35)}" contype="0" conaffinity="0" group="2"/>')
    return g


def _tower():
    # andon tower at the vision station: green run / amber routing / red jam
    g = []
    tx, ty = 6.0, 3.55
    g.append(f'<geom name="tower_pole" type="cylinder" size="0.022 0.78" pos="{tx} {ty} 0.78" '
             f'rgba="{STEEL}" contype="0" conaffinity="0" group="2"/>')
    for name, col, z in (("tower_g", "0.1 0.65 0.2", 1.62), ("tower_a", "0.85 0.6 0.05", 1.74),
                         ("tower_r", "0.75 0.1 0.1", 1.86)):
        g.append(f'<geom name="{name}" type="cylinder" size="0.05 0.055" pos="{tx} {ty} {z}" '
                 f'rgba="{col} 0.35" contype="0" conaffinity="0" group="2"/>')
    return g


def _vision_markers():
    """Make the invisible logic visible: measurement window, scan sheet,
    escapement-gate and pre-gate hold lines painted on/over belt A."""
    a = P.BELT_A
    g = []
    w0, w1 = P.VIRTUAL_SENSOR["window_x"]
    g.append(f'<geom name="vis_window" type="box" size="{(w1 - w0) / 2} {a["width"] / 2} 0.001" '
             f'pos="{(w0 + w1) / 2} {a["y"]} {a["top"] + 0.002}" rgba="0.45 0.25 0.55 0.25" '
             f'contype="0" conaffinity="0" group="2"/>')
    g.append(f'<geom name="vis_sheet" type="box" size="0.002 {a["width"] / 2} 0.26" '
             f'pos="6.0 {a["y"]} {a["top"] + 0.26}" rgba="0.75 0.35 0.95 0.16" '
             f'contype="0" conaffinity="0" group="2"/>')
    for name, x, col in (("line_hold", a["hold2_x"], "0.85 0.75 0.2 0.8"),
                         ("line_gate", a["gate_x"], "0.9 0.55 0.1 0.9")):
        g.append(f'<geom name="{name}" type="box" size="0.008 {a["width"] / 2} 0.0012" '
                 f'pos="{x} {a["y"]} {a["top"] + 0.002}" rgba="{col}" contype="0" conaffinity="0" group="2"/>')
    return g


def _escapement_gate():
    """Visible escapement gate at the accumulator entry: a lightweight flag
    panel (non-colliding — the belt drive enforces the hold) that LIFTS when
    the gate logic releases the next item. Driven by run_sim from gate state."""
    a = P.BELT_A
    x = a["gate_x"] + 0.02
    body = (
        f'<body name="egate" pos="{x} {a["y"]} {a["top"] + 0.10}">\n'
        f'      <joint name="ej" type="slide" axis="0 0 1" range="-0.002 0.55" damping="8"/>\n'
        f'      <geom name="egate_panel" type="box" size="0.012 {a["width"] / 2} 0.09" '
        f'rgba="0.9 0.45 0.1 0.85" contype="0" conaffinity="0" group="2" mass="0.4"/>\n'
        f'    </body>')
    extras = []
    for sgn in (-1, 1):
        extras.append(f'<geom name="egate_post{["l", "r"][sgn > 0]}" type="cylinder" size="0.02 0.7" '
                      f'pos="{x} {a["y"] + sgn * (a["width"] / 2 + 0.06)} 0.7" rgba="{STEEL}" '
                      f'contype="0" conaffinity="0" group="2"/>')
    act = '<position name="ea" joint="ej" kp="120" kv="22" forcerange="-80 80" ctrlrange="0 0.55"/>'
    return body, extras, act


def _signage(sign_files):
    """Billboard planes (auto-faced to the overview camera) + floor decals."""
    assets, geoms = [], []
    cx, cy = P.OVERVIEW_CAM_XY
    for key, _en, _ru, (x, y, z), _zone in P.SIGNS:
        fname = sign_files[key]
        assets.append(f'<texture name="tex_{key}" type="2d" file="signs/{fname}"/>')
        assets.append(f'<material name="mat_{key}" texture="tex_{key}"/>')
        dx, dy = cx - x, cy - y
        n = float(np.hypot(dx, dy)) or 1.0
        dx, dy = dx / n, dy / n
        yaw = float(np.arctan2(dx, -dy))
        geoms.append(f'<geom name="sign_{key}" type="plane" size="0.70 0.175 0.05" '
                     f'pos="{x} {y} {z}" euler="1.5708 0 {yaw:.4f}" material="mat_{key}" '
                     f'contype="0" conaffinity="0" group="2"/>')
        geoms.append(f'<geom name="sign_{key}_post" type="cylinder" size="0.028 {(z - 0.175) / 2}" '
                     f'pos="{x - dx * 0.04} {y - dy * 0.04} {(z - 0.175) / 2}" rgba="{STEEL}" '
                     f'contype="0" conaffinity="0" group="2"/>')
    for key, (x, y), yaw, (hl, hw) in P.FLOOR_DECALS:
        geoms.append(f'<geom name="decal_{key}" type="plane" size="{hl} {hw} 0.05" '
                     f'pos="{x} {y} 0.004" euler="0 0 {np.radians(yaw):.4f}" '
                     f'material="mat_{key}" contype="0" conaffinity="0" group="2"/>')
    return assets, geoms


# ----------------------------------------------------------------- executive
def _executive_geoms(mode):
    """Belt-A termination + routing hardware for the chosen architecture.
    Returns (worldbody_xml_list, actuator_xml_list)."""
    a = P.BELT_A
    g, acts = [], []
    if mode == "arm":
        g.append(f'<geom name="beltA" type="box" size="{(a["x_stop"] - a["x0"]) / 2} {a["width"] / 2} {a["top"] / 2}" '
                 f'pos="{(a["x0"] + a["x_stop"]) / 2} {a["y"]} {a["top"] / 2}" rgba="0.35 0.42 0.55 1"/>')
        g.append(f'<geom name="acc_stop" type="box" size="0.015 {a["width"] / 2} 0.05" '
                 f'pos="{a["x_stop"] + 0.015} {a["y"]} {a["top"] + 0.05}" rgba="0.9 0.75 0.4 1"/>')
        g.append(f'<geom name="acc_rail_l" type="box" size="0.66 0.015 0.08" '
                 f'pos="{a["x_stop"] - 0.645} {a["y"] + a["width"] / 2 + 0.015} {a["top"] + 0.08}" rgba="0.9 0.75 0.4 1"/>')
        g.append(f'<geom name="acc_rail_r" type="box" size="0.66 0.015 0.08" '
                 f'pos="{a["x_stop"] - 0.645} {a["y"] - a["width"] / 2 - 0.015} {a["top"] + 0.08}" rgba="0.9 0.75 0.4 1"/>')
        for cname, cage in P.cages_for("arm").items():
            g += _cage_xml(cname, cage)
        return g, acts

    # ------------------------------------------------------------- table mode
    tb, cb = P.TABLE, P.CONNECT_B
    cc, cd = P.CHUTE_C, P.CHUTE_D
    ty0, ty1 = tb["y"] - tb["width"] / 2, tb["y"] + tb["width"] / 2

    # belt A ends where the table begins (smooth same-height handover)
    g.append(f'<geom name="beltA" type="box" size="{(tb["x0"] - a["x0"]) / 2} {a["width"] / 2} {a["top"] / 2}" '
             f'pos="{(a["x0"] + tb["x0"]) / 2} {a["y"]} {a["top"] / 2}" rgba="0.35 0.42 0.55 1"/>')
    # entry funnel rails on the last stretch of belt A — placed DOWNSTREAM of
    # the vision station so the measurement volume stays clear (x > 6.66)
    g.append(f'<geom name="feed_rail_l" type="box" size="0.12 0.015 0.08" '
             f'pos="{tb["x0"] - 0.12} {a["y"] + a["width"] / 2 + 0.015} {a["top"] + 0.08}" rgba="0.9 0.75 0.4 1"/>')
    g.append(f'<geom name="feed_rail_r" type="box" size="0.12 0.015 0.08" '
             f'pos="{tb["x0"] - 0.12} {a["y"] - a["width"] / 2 - 0.015} {a["top"] + 0.08}" rgba="0.9 0.75 0.4 1"/>')

    # the transfer table surface
    g.append(f'<geom name="table" type="box" size="{(tb["x1"] - tb["x0"]) / 2} {tb["width"] / 2} {tb["top"] / 2}" '
             f'pos="{(tb["x0"] + tb["x1"]) / 2} {tb["y"]} {tb["top"] / 2}" rgba="0.42 0.5 0.62 1"/>')

    rail_h, rail_t, rail_z = 0.08, 0.015, a["top"] + 0.08
    # north edge: open only at the B-lane exit (gate guards the opening)
    bx0, bx1 = cb["cx"] - cb["width"] / 2, cb["cx"] + cb["width"] / 2
    g.append(f'<geom name="trail_n1" type="box" size="{(bx0 - tb["x0"]) / 2} {rail_t} {rail_h}" '
             f'pos="{(tb["x0"] + bx0) / 2} {ty1 + rail_t} {rail_z}" rgba="0.9 0.75 0.4 1"/>')
    if tb["x1"] > bx1:
        g.append(f'<geom name="trail_n2" type="box" size="{(tb["x1"] - bx1) / 2} {rail_t} {rail_h}" '
                 f'pos="{(bx1 + tb["x1"]) / 2} {ty1 + rail_t} {rail_z}" rgba="0.9 0.75 0.4 1"/>')
    # south edge: open only at the D-lane exit
    dx0, dx1 = cd["cx"] - cd["width"] / 2, cd["cx"] + cd["width"] / 2
    g.append(f'<geom name="trail_s1" type="box" size="{(dx0 - tb["x0"]) / 2} {rail_t} {rail_h}" '
             f'pos="{(tb["x0"] + dx0) / 2} {ty0 - rail_t} {rail_z}" rgba="0.9 0.75 0.4 1"/>')
    if tb["x1"] > dx1:
        g.append(f'<geom name="trail_s2" type="box" size="{(tb["x1"] - dx1) / 2} {rail_t} {rail_h}" '
                 f'pos="{(dx1 + tb["x1"]) / 2} {ty0 - rail_t} {rail_z}" rgba="0.9 0.75 0.4 1"/>')
    # east edge: open only at the C-lane exit
    cy0, cy1 = cc["cy"] - cc["width"] / 2, cc["cy"] + cc["width"] / 2
    g.append(f'<geom name="trail_e1" type="box" size="{rail_t} {(cy0 - ty0) / 2} {rail_h}" '
             f'pos="{tb["x1"] + rail_t} {(ty0 + cy0) / 2} {rail_z}" rgba="0.9 0.75 0.4 1"/>')
    g.append(f'<geom name="trail_e2" type="box" size="{rail_t} {(ty1 - cy1) / 2} {rail_h}" '
             f'pos="{tb["x1"] + rail_t} {(cy1 + ty1) / 2} {rail_z}" rgba="0.9 0.75 0.4 1"/>')
    # west edge corners (belt A feeds through the middle)
    g.append(f'<geom name="trail_w1" type="box" size="{rail_t} {(a["y"] - a["width"] / 2 - ty0) / 2} {rail_h}" '
             f'pos="{tb["x0"] - rail_t} {(ty0 + a["y"] - a["width"] / 2) / 2} {rail_z}" rgba="0.9 0.75 0.4 1"/>')
    g.append(f'<geom name="trail_w2" type="box" size="{rail_t} {(ty1 - a["y"] - a["width"] / 2) / 2} {rail_h}" '
             f'pos="{tb["x0"] - rail_t} {(a["y"] + a["width"] / 2 + ty1) / 2} {rail_z}" rgba="0.9 0.75 0.4 1"/>')

    # powered connector to the fixed belt B
    g.append(f'<geom name="connectB" type="box" size="{cb["width"] / 2} {(cb["y1"] - cb["y0"]) / 2} {cb["top"] / 2}" '
             f'pos="{cb["cx"]} {(cb["y0"] + cb["y1"]) / 2} {cb["top"] / 2}" rgba="0.42 0.5 0.62 1"/>')
    g.append(f'<geom name="crail_l" type="box" size="{rail_t} {(cb["y1"] - cb["y0"]) / 2} {rail_h}" '
             f'pos="{bx0 - rail_t} {(cb["y0"] + cb["y1"]) / 2} {rail_z}" rgba="0.9 0.75 0.4 1"/>')
    g.append(f'<geom name="crail_r" type="box" size="{rail_t} {(cb["y1"] - cb["y0"]) / 2} {rail_h}" '
             f'pos="{bx1 + rail_t} {(cb["y0"] + cb["y1"]) / 2} {rail_z}" rgba="0.9 0.75 0.4 1"/>')
    # connector lane dashes (blue) — the B path reads as one continuous lane
    for i, y in enumerate(np.arange(cb["y0"] + 0.10, cb["y1"], 0.16)):
        g.append(f'<geom name="claneB_{i}" type="box" size="0.026 0.048 0.001" '
                 f'pos="{cb["cx"]} {y:.3f} {cb["top"] + 0.004}" rgba="{_rgba("B", 0.5, 0.9)}" '
                 f'contype="0" conaffinity="0" group="2"/>')

    # guided single-slope brake chutes into the cage apertures
    cages_t = P.cages_for("table")
    wall_c = cages_t["C"]["center"][0] - cages_t["C"]["inner"][0] / 2 - cages_t["C"]["wall_t"] / 2
    wall_d = cages_t["D"]["center"][1] + cages_t["D"]["inner"][1] / 2 + cages_t["D"]["wall_t"] / 2
    g += _chute_xml("C", cc, "x", wall_c)
    g += _chute_xml("D", cd, "y", wall_d)

    # actuated exit gates (normally closed)
    for zone in ("B", "C", "D"):
        body, extras, act = _gate_xml(zone)
        g.append(body)
        g += extras
        acts.append(act)

    for cname, cage in P.cages_for("table").items():
        g += _cage_xml(cname, cage)

    g += _lane_visuals()
    g += _beacons()
    return g, acts


def build_xml(manifest, mode=None):
    mode = mode or P.EXEC_DEFAULT
    a, b = P.BELT_A, P.BELT_B
    arm = P.ARM
    bx, by = P.ARM_BASE[mode]

    from cell.signs import ensure_signs
    sign_assets, sign_geoms = _signage(ensure_signs())

    meshes = "\n    ".join(
        f'<mesh name="m_{e["slug"]}" file="{e["file"]}" maxhullvert="64"/>' for e in manifest)

    items = []
    welds = []
    for i, e in enumerate(manifest):
        slug = e["slug"]
        px = 0.6 + i * 0.85
        pz = e["dims_m"][2] / 2 + 0.001
        items.append(
            f'<body name="item_{slug}" pos="{px} -1.2 {pz}">\n'
            f'      <freejoint name="fj_{slug}"/>\n'
            f'      <geom name="g_{slug}" type="mesh" mesh="m_{slug}" mass="{e["mass_kg"]}" '
            f'friction="0.9 0.02 0.0005" rgba="0.75 0.72 0.65 1"/>\n'
            f'    </body>')
        welds.append(f'<weld name="w_{slug}" body1="wrist" body2="item_{slug}" active="false" '
                     f'solref="0.004 1"/>')

    exec_geoms, exec_acts = _executive_geoms(mode)
    executive = "\n    ".join(exec_geoms)
    egate_body, egate_extras, egate_act = _escapement_gate()
    statics = "\n    ".join(_vision_markers() + _tower() + egate_extras + sign_geoms)
    extra_actuators = "\n    ".join(exec_acts + [egate_act])

    xml = f"""
<mujoco model="sortmaster_cell_{mode}">
  <compiler meshdir="{ASSETS / 'meshes'}" texturedir="{ASSETS}" angle="radian"/>
  <option timestep="{P.SIM['timestep']}" integrator="implicitfast"/>
  <visual>
    <headlight ambient="0.45 0.45 0.45" diffuse="0.7 0.7 0.7"/>
    <global offwidth="1280" offheight="720"/>
  </visual>

  <asset>
    <texture type="2d" name="grid" builtin="checker" rgb1="0.90 0.90 0.92" rgb2="0.80 0.80 0.84" width="256" height="256"/>
    <material name="floor" texture="grid" texrepeat="12 8"/>
    {chr(10).join('    ' + t for t in sign_assets)}
    {meshes}
  </asset>

  <worldbody>
    <light dir="0 0 -1" pos="6 3 6" directional="true"/>

    <!-- presentation cameras -->
    <camera name="top_view" pos="7.7 3.0 6.5" xyaxes="1 0 0 0 1 0"/>
    <camera name="overview" pos="2.4 -2.4 4.6" xyaxes="0.7006 -0.7136 0 0.3288 0.3229 0.8880"/>
    <camera name="routing" pos="8.3 0.0 3.4" xyaxes="0.9999 0.0167 0 -0.0112 0.6688 0.7432"/>

    <!-- look-ahead vision station: overhead depth + profile scanners; the
         sensor hangs BELOW its crossbar so the mount never shadows the FOV -->
    <camera name="lookahead" pos="6.0 3.0 2.2" xyaxes="1 0 0 0 1 0" fovy="45"/>
    <geom name="cam_post" type="cylinder" size="0.04 1.175" pos="6.0 2.2 1.175"
          rgba="0.45 0.25 0.55 1" contype="0" conaffinity="0" group="2"/>
    <geom name="cam_bar" type="box" size="0.03 0.42 0.03" pos="6.0 2.6 2.32"
          rgba="0.45 0.25 0.55 1" contype="0" conaffinity="0" group="2"/>
    <geom name="cam_head" type="box" size="0.06 0.06 0.035" pos="6.0 3.0 2.255"
          rgba="0.2 0.1 0.3 1" contype="0" conaffinity="0" group="2"/>

    <geom name="floor" type="plane" size="12 8 0.1" pos="5 3 0" material="floor"/>

    <!-- conveyor B: sorter infeed (FIXED) -->
    <geom name="beltB" type="box" size="{b['width'] / 2} {(b['y1'] - b['y0']) / 2} {b['top'] / 2}"
          pos="{b['cx']} {(b['y0'] + b['y1']) / 2} {b['top'] / 2}" rgba="0.35 0.42 0.55 1"/>

    <!-- executive architecture: {mode} -->
    {executive}

    <!-- escapement gate + vision markers + signage -->
    {egate_body}
    {statics}

    <!-- 4-axis palletizer arm ({'primary picker' if mode == 'arm' else 'exception-recovery station'}) -->
    <geom name="pedestal" type="cylinder" size="0.15 {arm['pedestal_h'] / 2}"
          pos="{bx} {by} {arm['pedestal_h'] / 2}" rgba="0.25 0.25 0.28 1"/>
    <body name="yawcol" pos="{bx} {by} {arm['pedestal_h']}">
      <joint name="j1" type="hinge" axis="0 0 1" range="-7 7" damping="2"/>
      <geom type="cylinder" size="0.09 {arm['yaw_col_h'] / 2}" pos="0 0 {arm['yaw_col_h'] / 2}"
            rgba="0.85 0.55 0.1 1" contype="0" conaffinity="0" group="2"/>
      <body name="upper" pos="0 0 {arm['yaw_col_h']}">
        <joint name="j2" type="hinge" axis="0 1 0" range="-2.4 2.4" damping="2"/>
        <geom type="capsule" size="0.06" fromto="0 0 0 {arm['L1']} 0 0" rgba="0.85 0.55 0.1 1"
              contype="0" conaffinity="0" group="2"/>
        <body name="fore" pos="{arm['L1']} 0 0">
          <joint name="j3" type="hinge" axis="0 1 0" range="-2.8 2.8" damping="2"/>
          <geom type="capsule" size="0.05" fromto="0 0 0 {arm['L2']} 0 0" rgba="0.85 0.55 0.1 1"
                contype="0" conaffinity="0" group="2"/>
          <body name="wrist" pos="{arm['L2']} 0 0">
            <joint name="j4" type="hinge" axis="0 1 0" range="-3.0 3.0" damping="1"/>
            <geom type="cylinder" size="0.05 0.02" pos="0 0 -0.02" rgba="0.3 0.3 0.32 1"
                  contype="0" conaffinity="0" group="2"/>
            <geom name="tool" type="cylinder" size="0.04 0.1" pos="0 0 -0.148"
                  rgba="0.2 0.2 0.22 1" contype="0" conaffinity="0" group="2"/>
            <site name="tcp" pos="0 0 {-arm['tool_len']}" size="0.012" rgba="1 0 0 1"/>
          </body>
        </body>
      </body>
    </body>

    <!-- items (parked off-cell until spawned) -->
    {chr(10).join('    ' + it for it in items)}
  </worldbody>

  <equality>
    {chr(10).join('    ' + w for w in welds)}
  </equality>

  <actuator>
    <position name="a1" joint="j1" kp="6000" kv="500" forcerange="-900 900" ctrlrange="-7 7"/>
    <position name="a2" joint="j2" kp="12000" kv="900" forcerange="-1600 1600" ctrlrange="-2.4 2.4"/>
    <position name="a3" joint="j3" kp="12000" kv="900" forcerange="-1600 1600" ctrlrange="-2.8 2.8"/>
    <position name="a4" joint="j4" kp="2000" kv="120" forcerange="-250 250" ctrlrange="-3 3"/>
    {extra_actuators}
  </actuator>
</mujoco>
"""
    return xml


def make_model(mode=None):
    import mujoco
    manifest = load_manifest()
    xml = build_xml(manifest, mode=mode)
    model = mujoco.MjModel.from_xml_string(xml)
    return model, manifest, xml
