# -*- coding: utf-8 -*-
"""MJCF scene generation from cell.params — the layout has one source of truth.

EXECUTIVE (v3, mirrors isaac/scene_usd.py): a linear TILT-TRAY SORTER.
  * belt A ends in a knife-edge nose (thin driven slab the trays run under);
  * two pop-up stop blades: the pre-gate hold (single item in the vision
    window) and the normally-closed escapement (releases exactly one item,
    synchronized to an inbound empty carrier);
  * a carrier train along the belt axis: kinematically-slid carrier bodies
    (position-controlled chain, analytic s(t)) each carrying a dished TRAY on
    a real hinge joint + position actuator — freight rides by CONTACT only;
  * discharge stations: C/D tilt south onto 32-deg brake chutes into
    aperture-walled roll cages, B tilts north onto a powered incline
    connector feeding the FIXED belt B, REVIEW tilts north into the
    manual-review pen;
  * exception arm between the cages (chute-snag recovery only).

Containment-by-design: side guides on belt A, tray end lips, side-railed
chutes with a high-friction brake runout, aperture-walled cages, closed
walls everywhere else.

Presentation layer (video/jury): bilingual signage, floor decals, station
portals, destination beacons, andon tower — all contype/conaffinity 0.

Contact classes: item geoms carry conaffinity 3 so the stop blades
(contype 2, conaffinity 0) collide with items but never with the belt body
they retract into.
"""
import json
import math
from pathlib import Path

import numpy as np

from cell import params as P

ASSETS = Path(__file__).parent / "assets"

STEEL = "0.55 0.57 0.62 1"
DARK = "0.22 0.24 0.28 1"
TAN32 = math.tan(math.radians(32.0))
# chute side rails start this far downhill of the chute's top edge: the
# tilting tray lip sweeps a lens (y within 70 mm of the chute top) that the
# rail nose must stay clear of — same reason the Isaac twin's tray clears
# the slope by TRAY_LIP_Z - z0
RAIL_SETBACK = 0.07


def load_manifest():
    manifest = json.loads((ASSETS / "manifest.json").read_text(encoding="utf-8"))
    extra = ASSETS / "manifest_extra.json"       # synthetic borderline items
    if extra.exists():                           # (tools/make_borderline_items.py)
        manifest += json.loads(extra.read_text(encoding="utf-8"))
    custom = ASSETS / "manifest_custom.json"     # expert-provided STLs
    if custom.exists():                          # (tools/expert_check.py)
        manifest += json.loads(custom.read_text(encoding="utf-8"))
    return manifest


def _rgba(zone, scale=1.0, alpha=1.0):
    r, g, b = P.ROUTE_RGBA[zone]
    return f"{r * scale:.3f} {g * scale:.3f} {b * scale:.3f} {alpha}"


# --------------------------------------------------------------------- cages
def _cage_xml(name, cage):
    """Roll-cage / review pen: frame posts + semi-transparent walls. A cage
    fed by a chute has an entry APERTURE (flanks + header + low sill skirt)
    instead of a missing wall, so items cannot bounce back out."""
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
            # entry aperture: flank strips + header + under-chute sill skirt —
            # the port is exactly chute-sized, nothing rolls back out below
            # or beside the runout
            aw2 = cage["aperture_w"] / 2
            top = cage["aperture_top"]
            sill = cage.get("sill_top", 0.20)
            skirt_h2 = (sill - t) / 2
            skirt_zc = (sill + t) / 2
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
                         f'pos="{cx + dx} {cy} {skirt_zc}" rgba="{wall}"/>')
            else:
                for sgn, nm in ((1, "a"), (-1, "b")):
                    fl = (sx - aw2) / 2
                    g.append(f'<geom name="cage{name}_fl{nm}" type="box" size="{fl} {sy} {hh}" '
                             f'pos="{cx + sgn * (aw2 + fl)} {cy + dy} {t + hh}" rgba="{wall}"/>')
                if hdr_h > 0.005:
                    g.append(f'<geom name="cage{name}_hdr" type="box" size="{aw2} {sy} {hdr_h}" '
                             f'pos="{cx} {cy + dy} {top + hdr_h}" rgba="{wall}"/>')
                g.append(f'<geom name="cage{name}_skirt" type="box" size="{aw2} {sy} {skirt_h2}" '
                         f'pos="{cx} {cy + dy} {skirt_zc}" rgba="{wall}"/>')
        else:
            g.append(f'<geom name="cage{name}_w{side}" type="box" size="{sx} {sy} {hh}" '
                     f'pos="{cx + dx} {cy + dy} {t + hh}" rgba="{wall}"/>')
    # high-friction landing mat: kills residual slide so items settle in
    # place instead of ramming the far wall
    if open_side:
        g.append(f'<geom name="cage{name}_mat" type="box" size="{hx} {hy} 0.004" '
                 f'pos="{cx} {cy} {t + 0.004}" friction="{P.CAGE_MAT_FRICTION}" '
                 f'priority="2" solref="0.008 1" rgba="0.15 0.15 0.17 1"/>')
    # frame posts + top rails (visual only)
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
def _chute_xml(zone, cc, wall_at):
    """32-deg gravity brake chute along the y axis (cc["dir"] = -1 south into
    the C/D cages, +1 north into the review pen). One continuous guided slope
    (mu < tan 32 deg: nothing rests on it) from just under the tilted tray
    lip, through the destination wall aperture, onto a flat high-friction
    BRAKE PAD just above the floor. priority=1: the polished chute dictates
    the contact friction."""
    d = float(cc["dir"])
    y0, z0, z1, cx = cc["y0"], cc["z0"], cc["z1"], cc["cx"]
    y1, pad_end = P.chute_run(cc)
    frame = _rgba(zone, 1.0, 1.0)
    g = []
    length = float(np.hypot(y1 - y0, z0 - z1))
    # rotX(+32) raises the +y side: a south-running chute descends with +32,
    # a north-running one with -32 (mirrors isaac.build_chute)
    ang = -d * math.radians(32.0)
    mid, zmid = (y0 + y1) / 2, (z0 + z1) / 2 - 0.015
    g.append(f'<geom name="chute{zone}" type="box" size="{cc["width"] / 2} {length / 2} 0.015" '
             f'pos="{cx} {mid} {zmid}" euler="{ang:.6f} 0 0" '
             f'friction="{cc["friction"]}" priority="1" rgba="0.55 0.55 0.6 1"/>')
    # MOUTH CHAMFER: steep infill strip at the mouth edge (see params.CHUTE).
    # The 59-deg working face is a thin plate; an unrotated FILLER block is
    # sunk underneath so a fast item edge that penetrates the 4 mm shell
    # meets solid backing instead of wedging in the shell/plate cavity
    # (solver blow-up, close_spacing s7). The filler stays inside the face's
    # footprint so nothing protrudes into the fall corridor.
    ch_rise, ch_run = cc["chamfer_rise"], cc["chamfer_run"]
    ch_top_y = y0 - d * cc["chamfer_top_inset"]
    ch_base_y = ch_top_y + d * ch_run
    ch_ang = -d * math.atan2(ch_rise + 0.003, ch_run)
    ch_len = math.hypot(ch_run, ch_rise + 0.003)
    g.append(f'<geom name="chute{zone}_chamfer" type="box" '
             f'size="{cc["width"] / 2} {ch_len / 2:.4f} 0.002" '
             f'pos="{cx} {(ch_top_y + ch_base_y) / 2:.4f} '
             f'{z0 + (ch_rise - 0.003) / 2:.4f}" euler="{ch_ang:.6f} 0 0" '
             f'friction="{cc["friction"]}" priority="1" '
             f'rgba="0.50 0.50 0.55 1"/>')
    fill_y0 = ch_top_y + d * 0.004               # inside the face footprint
    fill_y1 = ch_base_y - d * 0.002
    g.append(f'<geom name="chute{zone}_chamfill" type="box" '
             f'size="{cc["width"] / 2} {abs(fill_y1 - fill_y0) / 2:.4f} 0.014" '
             f'pos="{cx} {(fill_y0 + fill_y1) / 2:.4f} {z0 - 0.017:.4f}" '
             f'friction="{cc["friction"]}" priority="1" '
             f'rgba="0.50 0.50 0.55 1"/>')
    # MOUTH CHEEKS: side wings over the throat gap (see params.CHUTE)
    ck_t = 0.015
    ck_y0 = y0 + d * cc["cheek_inset"]
    ck_y1 = ck_y0 + d * cc["cheek_len"]
    ck_cy = (ck_y0 + ck_y1) / 2
    for sgn, nm in ((1, "l"), (-1, "r")):
        off = sgn * (cc["width"] / 2 + ck_t)
        g.append(f'<geom name="chute{zone}_cheek_{nm}" type="box" '
                 f'size="{ck_t} {cc["cheek_len"] / 2:.4f} 0.034" '
                 f'pos="{cx + off} {ck_cy:.4f} {z0 + cc["cheek_h"] - 0.034:.4f}" '
                 f'friction="{cc["friction"]}" priority="1" '
                 f'rgba="{frame}"/>')
    # side guides from just below the tray-lip sweep down to the wall plane
    rail_t = 0.015
    r0 = y0 + d * RAIL_SETBACK
    rmid = (r0 + wall_at) / 2
    rlen = float(np.hypot(wall_at - r0, abs(wall_at - r0) * TAN32))
    for sgn, nm in ((1, "l"), (-1, "r")):
        off = sgn * (cc["width"] / 2 + rail_t)
        rz = z0 - abs(rmid - y0) * TAN32 + P.GUIDE_H / 2
        g.append(f'<geom name="chute{zone}_rail_{nm}" type="box" size="{rail_t} {rlen / 2} {P.GUIDE_H / 2}" '
                 f'pos="{cx + off} {rmid} {rz}" euler="{ang:.6f} 0 0" '
                 f'friction="{cc["friction"]}" priority="1" rgba="{frame}"/>')
        # LOW GUARD BRIDGE cheek-end -> rail start (see isaac twin): closes
        # the lateral window the deleted stubs covered; 60 mm tall, clears
        # the tilted lip-tip arc by ~80 mm.
        gb0 = y0 + d * cc.get("cheek_len", 0.048)
        gb1 = y0 + d * RAIL_SETBACK
        gmid = (gb0 + gb1) / 2
        glen = abs(gb1 - gb0)
        gz = z0 - abs(gmid - y0) * TAN32 + 0.030
        g.append(f'<geom name="chute{zone}_guard_{nm}" type="box" '
                 f'size="{rail_t} {glen / 2 + 0.005:.4f} 0.030" '
                 f'pos="{cx + off} {gmid:.4f} {gz:.4f}" euler="{ang:.6f} 0 0" '
                 f'friction="{cc["friction"]}" priority="1" rgba="{frame}"/>')
    # brake pad: flat, 5 mm below the slope tail (a downhill step, never a
    # lip). Soft (rubber-faced): absorbs the landing instead of returning it
    # — a stiff contact can eject a thin light item (pen pogo)
    pmid = (y1 + pad_end) / 2
    plen = abs(pad_end - y1)
    g.append(f'<geom name="chute{zone}_pad" type="box" size="{cc["width"] / 2} {plen / 2} 0.015" '
             f'pos="{cx} {pmid} {z1 - 0.020}" friction="{cc["pad_friction"]}" '
             f'priority="1" solref="0.008 1" rgba="0.30 0.31 0.35 1"/>')
    # NO HOOD: the deep-drop chute crosses the aperture at z ~0.22 — no
    # fly-out window remains, and the open chute keeps every jam point
    # vertically extractable by the exception arm.
    # zone-coloured flow arrows painted on the slope (visual)
    n_dash = max(3, int(length / 0.20))
    for i in range(n_dash):
        f_ = (i + 0.5) / n_dash
        da = y0 + (y1 - y0) * f_
        dz = z0 + (z1 - z0) * f_ + 0.004
        g.append(f'<geom name="flow{zone}_{i}" type="box" size="0.035 0.055 0.001" '
                 f'pos="{cx} {da} {dz}" euler="{ang:.6f} 0 0" '
                 f'rgba="{_rgba(zone, 1.0, 0.85)}" contype="0" conaffinity="0" group="2"/>')
    return g


# ---------------------------------------------------------- B incline connector
def _b_connector_xml():
    """Powered incline belt from the B-station tray lip up to the FIXED belt
    B infeed (16.6 deg; every B item is non-round by rule and holds by
    friction). The drive is cell/belt.py's kinematic conveyor model."""
    bc = P.B_CONNECT
    ang = math.atan2(bc["z_top1"] - bc["z_top0"], bc["y1"] - bc["y0"])
    length = float(np.hypot(bc["y1"] - bc["y0"], bc["z_top1"] - bc["z_top0"]))
    cy = (bc["y0"] + bc["y1"]) / 2
    cz = (bc["z_top0"] + bc["z_top1"]) / 2 - 0.015
    g = [f'<geom name="bconnect" type="box" size="{bc["width"] / 2} {length / 2} 0.015" '
         f'pos="{bc["cx"]} {cy} {cz}" euler="{ang:.6f} 0 0" '
         f'friction="0.06 0.004 0.0001" priority="1" rgba="0.35 0.42 0.55 1"/>']
    # side rails absorb the +x drift the item keeps from the moving tray.
    # Their south nose starts clear of the tilting tray-lip sweep.
    rail_t = 0.015
    r0 = bc["y0"] + RAIL_SETBACK
    rcy = (r0 + bc["y1"]) / 2
    rlen2 = (bc["y1"] - r0) / (2 * math.cos(ang))
    rcz = cz + (rcy - cy) * math.tan(ang) + 0.10
    for sgn, nm in ((1, "l"), (-1, "r")):
        g.append(f'<geom name="bconnect_rail_{nm}" type="box" size="{rail_t} {rlen2:.5f} 0.07" '
                 f'pos="{bc["cx"] + sgn * (bc["width"] / 2 + rail_t)} {rcy} {rcz:.5f}" '
                 f'euler="{ang:.6f} 0 0" rgba="0.9 0.75 0.4 1"/>')
    # connector lane dashes (blue) — the B path reads as one continuous lane
    for i, y in enumerate(np.arange(bc["y0"] + 0.10, bc["y1"], 0.16)):
        z = bc["z_top0"] + (y - bc["y0"]) * math.tan(ang) + 0.004
        g.append(f'<geom name="claneB_{i}" type="box" size="0.026 0.048 0.001" '
                 f'pos="{bc["cx"]} {y:.3f} {z:.4f}" euler="{ang:.6f} 0 0" '
                 f'rgba="{_rgba("B", 0.5, 0.9)}" contype="0" conaffinity="0" group="2"/>')
    return g


# ------------------------------------------------------------------ stop blades
def _blade_xml(name, x):
    """Pop-up stop blade (escapement / pre-gate hold): a thin panel on a
    vertical slide, normally LOWERED flush below the belt surface, raised so
    its bottom skims ~3 mm above the belt — a 9 mm pen cannot slip under.
    contype 2 / conaffinity 0: collides with items (conaffinity 3) only,
    never with the belt body it retracts into."""
    a = P.BELT_A
    w2 = (a["width"] - 0.02) / 2
    h2 = 0.06
    zc = a["top"] - h2 - 0.046
    body = (
        f'<body name="{name}" pos="{x} {a["y"]} {zc}">\n'
        f'      <joint name="{name}_j" type="slide" axis="0 0 1" range="-0.002 0.20" damping="8"/>\n'
        f'      <geom name="{name}_panel" type="box" size="0.012 {w2} {h2}" mass="2.0" '
        f'contype="2" conaffinity="0" friction="0.1 0.005 0.0001" priority="1" '
        f'solref="0.008 1" rgba="0.16 0.17 0.20 1"/>\n'
        f'      <geom name="{name}_strip" type="box" size="0.013 {w2 + 0.001} 0.006" pos="0 0 {h2}" '
        f'rgba="0.95 0.78 0.06 1" contype="0" conaffinity="0" group="2"/>\n'
        f'    </body>')
    extras = []
    for sgn in (-1, 1):
        extras.append(f'<geom name="{name}_post{["l", "r"][sgn > 0]}" type="cylinder" size="0.02 0.35" '
                      f'pos="{x} {a["y"] + sgn * (a["width"] / 2 + 0.06)} 0.35" rgba="{STEEL}" '
                      f'contype="0" conaffinity="0" group="2"/>')
    act = (f'<position name="{name}_a" joint="{name}_j" kp="2200" kv="140" '
           f'forcerange="-900 900" ctrlrange="0 {P.BELT_A["blade_up"]}"/>')
    return body, extras, act


# ------------------------------------------------------------ tilt-tray train
def carrier_home(i):
    """Initial (x, z, leg) of carrier i on the loop (same math as the chain
    controller in cell/sorter.py, s(t=0))."""
    S = P.SORTER
    L_top = S["x_east"] - S["x_west"]
    s0 = (i * S["pitch"]) % (2 * L_top)
    if s0 < L_top:
        return S["x_west"] + s0, S["shuttle_top"] - 0.025, "top"
    return S["x_east"] - (s0 - L_top), S["return_z"] - 0.05, "return"


def _carrier_xml(i):
    """One sorter carrier: a body on two slide joints (x along the chain,
    z for the top/return leg) — position-controlled by cell/sorter.py every
    physics step, so contacts see real solver velocity — carrying a dished
    TRAY on a hinge joint with a position actuator (the tilt drive).

    Hinge axis is -x so a POSITIVE joint angle tilts the tray NORTH (+y edge
    down): the drive goal is side * tilt_deg exactly as in isaac/sorter.py.
    Tray surface friction pairs LOW via priority (combine=min analogue):
    gravity discharge is guaranteed for every item incl. the mu=0.95 sack."""
    S = P.SORTER
    x0, z0, _leg = carrier_home(i)
    pivot_dz = S["pivot_z"] - (S["shuttle_top"] - 0.025)
    dish = math.radians(S["dish_deg"])
    hy = S["tray_w"] / 4 / math.cos(dish)
    hz = S["tray_t"] / 2
    z_in = S["tray_top"] - S["pivot_z"]           # surface at the centre line
    czz = z_in - hz + (S["tray_w"] / 4) * math.tan(dish)
    mu = S["tray_mu"][1]
    # rolling coeff 0.0025 + condim 6 (rolling friction needs the full
    # contact dimensionality): the dimpled tray liner's rolling resistance —
    # caps a lying rod's roll-up on the tilting tray so it dribbles over the
    # lip into the chute mouth instead of launching ballistically off the
    # dish-valley joint (PhysX exhibits this damping natively; MuJoCo needs
    # it explicit). Rounds are barely affected (decel ~ coeff/r).
    fr = (f'friction="{mu} 0.005 0.0025" condim="6" priority="1" '
          f'solref="0.01 1"')
    plates = []
    for sgn, nm in ((1, "n"), (-1, "s")):
        plates.append(
            f'<geom name="tray{i}_p{nm}" type="box" size="{S["tray_l"] / 2} {hy:.6f} {hz}" '
            f'pos="0 {sgn * S["tray_w"] / 4} {czz:.6f}" euler="{sgn * dish:.6f} 0 0" '
            f'mass="3.0" {fr} rgba="0.16 0.17 0.19 1"/>')
    lips = []
    for sgn, nm in ((1, "e"), (-1, "w")):
        lips.append(
            f'<geom name="tray{i}_l{nm}" type="box" size="{S["lip_t"] / 2} {S.get("lip_w", S["tray_w"]) / 2} {S["lip_h"] / 2}" '
            f'pos="{sgn * (S["tray_l"] / 2 - S["lip_t"] / 2)} 0 {z_in + S["lip_h"] / 2}" '
            f'mass="0.3" {fr} rgba="0.42 0.30 0.10 1"/>')
    lim = math.radians(S["tilt_deg"] + 8.0)
    body = (
        f'<body name="car{i}" pos="{x0:.6f} {S["y"]} {z0:.6f}">\n'
        f'      <joint name="cjx{i}" type="slide" axis="1 0 0"/>\n'
        f'      <joint name="cjz{i}" type="slide" axis="0 0 1"/>\n'
        f'      <geom name="shuttle{i}" type="box" size="0.27 0.27 0.025" mass="40" '
        f'contype="0" conaffinity="0" group="2" rgba="0.24 0.26 0.30 1"/>\n'
        f'      <body name="tray{i}" pos="0 0 {pivot_dz:.6f}">\n'
        f'        <joint name="tj{i}" type="hinge" axis="-1 0 0" range="{-lim:.5f} {lim:.5f}" '
        f'damping="3" armature="0.05"/>\n'
        f'        {chr(10).join("        " + p for p in plates + lips)}\n'
        f'      </body>\n'
        f'    </body>')
    act = (f'<position name="ta{i}" joint="tj{i}" kp="600" kv="40" '
           f'forcerange="-{S["drive_max_torque"]} {S["drive_max_torque"]}" '
           f'ctrlrange="{-lim:.5f} {lim:.5f}"/>')
    return body, act


# ------------------------------------------------------- presentation statics
def _stations_xml():
    """Discharge-station portals + route-colour lamps (visual only; the
    mechanism is the tray)."""
    g = []
    for zone, st in P.STATIONS.items():
        x, side = st["x"], st["side"]
        by = P.SORTER["y"] + side * 0.46
        col = _rgba(zone, 1.0, 1.0)
        for sgn in (-1, 1):
            g.append(f'<geom name="st{zone}_post{["a", "b"][sgn > 0]}" type="box" size="0.02 0.02 0.50" '
                     f'pos="{x + sgn * 0.30} {by} 0.50" rgba="{STEEL}" contype="0" conaffinity="0" group="2"/>')
        g.append(f'<geom name="st{zone}_beam" type="box" size="0.32 0.02 0.02" '
                 f'pos="{x} {by} 1.02" rgba="{STEEL}" contype="0" conaffinity="0" group="2"/>')
        g.append(f'<geom name="st{zone}_lamp" type="box" size="0.10 0.025 0.025" '
                 f'pos="{x} {by} 1.08" rgba="{col}" contype="0" conaffinity="0" group="2"/>')
    # debris CATCH PAN under the top run: freight that slips through an
    # inter-tray gap at induction (sub-3 mm arrives unmetered; a thin pen on
    # an unlucky tray phase deflects at the nose) lands HERE and the
    # watchdog raises an operator call-out — never a floor spill.
    S = P.SORTER
    g.append(f'<geom name="train_pan" type="box" '
             f'size="{(S["pan_x1"] - S["pan_x0"]) / 2} {S["pan_y_half"]} 0.005" '
             f'pos="{(S["pan_x0"] + S["pan_x1"]) / 2} {S["y"]} '
             f'{S["pan_z_top"] - 0.005}" friction="0.8 0.01 0.0001" '
             f'rgba="0.13 0.14 0.16 1"/>')
    # enclosed end modules (the carrier wrap teleports happen inside them)
    y = P.SORTER["y"]
    g.append(f'<geom name="train_end_e" type="box" size="0.15 0.40 0.28" pos="9.57 {y} 0.46" '
             f'rgba="{DARK}" contype="0" conaffinity="0" group="2"/>')
    g.append(f'<geom name="train_end_w" type="box" size="0.15 0.40 0.21" pos="6.59 {y} 0.40" '
             f'rgba="{DARK}" contype="0" conaffinity="0" group="2"/>')
    # operator manual-handling station
    mx, my = P.MANUAL_STATION
    g.append(f'<geom name="manual_station" type="box" size="0.35 0.55 0.01" pos="{mx} {my} 0.01" '
             f'rgba="0.62 0.30 0.10 0.8" contype="0" conaffinity="0" group="2"/>')
    return g


def _beacons():
    g = []
    spots = {"B": (8.85, 4.95), "C": (6.70, 1.35), "D": (9.55, 1.35)}
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
    cam_x = P.VIRTUAL_SENSOR["overhead_pos"][0]
    g.append(f'<geom name="vis_window" type="box" size="{(w1 - w0) / 2} {a["width"] / 2} 0.001" '
             f'pos="{(w0 + w1) / 2} {a["y"]} {a["top"] + 0.002}" rgba="0.45 0.25 0.55 0.25" '
             f'contype="0" conaffinity="0" group="2"/>')
    g.append(f'<geom name="vis_sheet" type="box" size="0.002 {a["width"] / 2} 0.26" '
             f'pos="{cam_x} {a["y"]} {a["top"] + 0.26}" rgba="0.75 0.35 0.95 0.16" '
             f'contype="0" conaffinity="0" group="2"/>')
    for name, x, col in (("line_hold", a["hold2_x"], "0.85 0.75 0.2 0.8"),
                         ("line_gate", a["gate_x"], "0.9 0.55 0.1 0.9")):
        g.append(f'<geom name="{name}" type="box" size="0.008 {a["width"] / 2} 0.0012" '
                 f'pos="{x} {a["y"]} {a["top"] + 0.002}" rgba="{col}" contype="0" conaffinity="0" group="2"/>')
    return g


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
def _executive_geoms():
    """Belt-A knife nose + stop blades + tilt-tray train + stations + chutes
    + cages + review pen + B connector. Returns (worldbody, actuators)."""
    a = P.BELT_A
    g, acts = [], []

    # belt A: main body up to the knife section
    g.append(f'<geom name="beltA" type="box" size="{(a["knife_x0"] - a["x0"]) / 2} {a["width"] / 2} {a["top"] / 2}" '
             f'pos="{(a["x0"] + a["knife_x0"]) / 2} {a["y"]} {a["top"] / 2}" '
             f'friction="0.06 0.004 0.0001" priority="1" rgba="0.35 0.42 0.55 1"/>')
    # knife-edge nose: thin driven slab the trays run under (zero-gap handoff)
    g.append(f'<geom name="knife" type="box" size="{(a["nose_x"] - a["knife_x0"]) / 2} {a["width"] / 2} {a["knife_t"] / 2}" '
             f'pos="{(a["knife_x0"] + a["nose_x"]) / 2} {a["y"]} {a["top"] - a["knife_t"] / 2}" '
             f'friction="0.06 0.004 0.0001" priority="1" '
             f'rgba="0.38 0.45 0.58 1"/>')
    # low side guides along belt A; above the knife the guide bottom stays
    # clear of the tray-lip sweep
    for sgn, nm in ((1, "l"), (-1, "r")):
        g.append(f'<geom name="beltA_guide_{nm}" type="box" size="{a["knife_x0"] / 2} 0.015 0.05" '
                 f'pos="{a["knife_x0"] / 2} {a["y"] + sgn * (a["width"] / 2 + 0.015)} {a["top"] + 0.04}" '
                 f'rgba="{STEEL}"/>')
        g.append(f'<geom name="knife_guide_{nm}" type="box" size="{(a["nose_x"] - a["knife_x0"]) / 2} 0.015 0.045" '
                 f'pos="{(a["knife_x0"] + a["nose_x"]) / 2} {a["y"] + sgn * (a["width"] / 2 + 0.015)} '
                 f'{a["top"] + 0.045}" rgba="{STEEL}"/>')

    # stop blades: normally-closed escapement + pre-gate hold
    for name, x in (("egate", a["gate_x"]), ("hold2", a["hold2_x"])):
        body, extras, act = _blade_xml(name, x)
        g.append(body)
        g += extras
        acts.append(act)

    # the tilt-tray carrier train
    for i in range(P.SORTER["n_carriers"]):
        body, act = _carrier_xml(i)
        g.append(body)
        acts.append(act)

    # chutes into the aperture-walled destinations
    cages = P.cages_for()
    wall_c = (cages["C"]["center"][1] + cages["C"]["inner"][1] / 2
              + cages["C"]["wall_t"] / 2)
    wall_d = (cages["D"]["center"][1] + cages["D"]["inner"][1] / 2
              + cages["D"]["wall_t"] / 2)
    rp = P.REVIEW_PEN
    wall_r = rp["center"][1] - rp["inner"][1] / 2 - rp["wall_t"] / 2
    g += _chute_xml("C", P.CHUTE_C, wall_c)
    g += _chute_xml("D", P.CHUTE_D, wall_d)
    g += _chute_xml("REVIEW", P.CHUTE_REVIEW, wall_r)
    for cname, cage in cages.items():
        g += _cage_xml(cname, cage)
    g += _cage_xml("REVIEW", rp)

    # powered incline connector to the FIXED belt B
    g += _b_connector_xml()

    g += _stations_xml()
    g += _beacons()
    return g, acts


def build_xml(manifest, mode=None):
    """mode is accepted for legacy callers and ignored: the executive is the
    tilt-tray sorter ('sorter') — EXEC modes collapsed (see cell/params.py)."""
    mode = "sorter"
    a, b = P.BELT_A, P.BELT_B
    arm = P.ARM
    bx, by = P.ARM_BASE[mode]
    cam = P.VIRTUAL_SENSOR["overhead_pos"]

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
        # thin items (pen-class, min extent < 20 mm): stiff + overdamped
        # contact (dampratio 2) + a small activation margin — holds a heavy
        # neighbor's static load without sinking past the thin body's
        # half-thickness AND kills restitution on impact.
        soft = (' solref="0.004 2" margin="0.001"'
                if min(e["dims_m"]) < 0.02 else "")
        # conaffinity 3: items also collide with the stop blades (contype 2)
        items.append(
            f'<body name="item_{slug}" pos="{px} -1.2 {pz}">\n'
            f'      <freejoint name="fj_{slug}"/>\n'
            f'      <geom name="g_{slug}" type="mesh" mesh="m_{slug}" mass="{e["mass_kg"]}" '
            f'friction="0.9 0.02 0.0005" conaffinity="3"{soft} rgba="0.75 0.72 0.65 1"/>\n'
            f'    </body>')
        welds.append(f'<weld name="w_{slug}" body1="wrist" body2="item_{slug}" active="false" '
                     f'solref="0.004 1"/>')

    exec_geoms, exec_acts = _executive_geoms()
    executive = "\n    ".join(exec_geoms)
    statics = "\n    ".join(_vision_markers() + _tower() + sign_geoms)
    extra_actuators = "\n    ".join(exec_acts)

    xml = f"""
<mujoco model="sortmaster_cell_{mode}">
  <compiler meshdir="{ASSETS / 'meshes'}" texturedir="{ASSETS}" angle="radian"/>
  <option timestep="{P.SIM['timestep']}" integrator="implicitfast"
          cone="elliptic" noslip_iterations="3"
          iterations="200" ls_iterations="100"/>
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
    <camera name="lookahead" pos="{cam[0]} {cam[1]} {cam[2]}" xyaxes="1 0 0 0 1 0" fovy="45"/>
    <geom name="cam_post" type="cylinder" size="0.04 1.175" pos="{cam[0]} 2.2 1.175"
          rgba="0.45 0.25 0.55 1" contype="0" conaffinity="0" group="2"/>
    <geom name="cam_bar" type="box" size="0.03 0.42 0.03" pos="{cam[0]} 2.6 2.32"
          rgba="0.45 0.25 0.55 1" contype="0" conaffinity="0" group="2"/>
    <geom name="cam_head" type="box" size="0.06 0.06 0.035" pos="{cam[0]} {cam[1]} 2.255"
          rgba="0.2 0.1 0.3 1" contype="0" conaffinity="0" group="2"/>

    <geom name="floor" type="plane" size="12 8 0.1" pos="5 3 0" material="floor"/>

    <!-- conveyor B: sorter infeed (FIXED) -->
    <geom name="beltB" type="box" friction="0.06 0.004 0.0001" priority="1" size="{b['width'] / 2} {(b['y1'] - b['y0']) / 2} {b['top'] / 2}"
          pos="{b['cx']} {(b['y0'] + b['y1']) / 2} {b['top'] / 2}" rgba="0.35 0.42 0.55 1"/>

    <!-- executive architecture: tilt-tray sorter -->
    {executive}

    <!-- vision markers + signage -->
    {statics}

    <!-- 4-axis palletizer arm (exception-recovery station) -->
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
    """mode accepted for legacy callers (tests, tools) and collapsed to the
    single 'sorter' executive."""
    import mujoco
    manifest = load_manifest()
    xml = build_xml(manifest, mode=mode)
    model = mujoco.MjModel.from_xml_string(xml)
    return model, manifest, xml
