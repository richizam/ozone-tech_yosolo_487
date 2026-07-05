# -*- coding: utf-8 -*-
"""MJCF scene generation from cell.params — the layout has one source of truth.

Two executive architectures share one generator (build_xml(mode)):
  "table" (primary): conveyor A -> escapement gate -> widened tri-directional
          transfer table; lanes exit north to the B connector, east over a
          gravity chute into cage C, south over a chute into cage D. The arm
          stands at an exception station south-east of the routing zone.
  "arm"   (preserved baseline): conveyor A -> accumulator with stop wall; the
          arm picks every item (tag arm-primary-baseline).
"""
import json
from pathlib import Path

from cell import params as P

ASSETS = Path(__file__).parent / "assets"


def load_manifest():
    return json.loads((ASSETS / "manifest.json").read_text(encoding="utf-8"))


def _cage_xml(name, cage):
    cx, cy = cage["center"]
    ix, iy = cage["inner"]
    t = cage["wall_t"]
    h = cage["wall_h"]
    hx, hy, hh = ix / 2, iy / 2, h / 2
    rgba = "0.85 0.5 0.45 1" if name == "C" else "0.5 0.8 0.5 1"
    g = [f'<geom name="cage{name}_floor" type="box" size="{hx + t} {hy + t} {t / 2}" '
         f'pos="{cx} {cy} {t / 2}" rgba="{rgba}"/>']
    walls = {"+y": (0, hy + t / 2, hx + t, t / 2), "-y": (0, -(hy + t / 2), hx + t, t / 2),
             "+x": (hx + t / 2, 0, t / 2, hy), "-x": (-(hx + t / 2), 0, t / 2, hy)}
    for side, (dx, dy, sx, sy) in walls.items():
        if side == cage.get("open_side"):
            continue                    # roll-container with its front open
        g.append(f'<geom name="cage{name}_w{side}" type="box" size="{sx} {sy} {hh}" '
                 f'pos="{cx + dx} {cy + dy} {t + hh}" rgba="{rgba}"/>')
    return "\n      ".join(g)


def _incline_xml(name, center, half, euler, rgba="0.55 0.55 0.6 1", friction="0.12 0.005 0.0001"):
    # priority=1: the polished chute surface DICTATES the contact friction
    # (otherwise MuJoCo takes the max of both geoms and sticky items freeze
    # on the slope when placed without momentum)
    return (f'<geom name="{name}" type="box" size="{half[0]} {half[1]} {half[2]}" '
            f'pos="{center[0]} {center[1]} {center[2]}" euler="{euler[0]} {euler[1]} {euler[2]}" '
            f'friction="{friction}" priority="1" rgba="{rgba}"/>')


def _executive_geoms(mode):
    """Belt-A termination + routing hardware for the chosen architecture."""
    a = P.BELT_A
    g = []
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
            g.append(_cage_xml(cname, cage))
        return g

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
    # north edge: open only at the B-lane exit
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

    # gravity chutes into the open cage sides
    import numpy as np
    cl = float(np.hypot(cc["x1"] - cc["x0"], cc["z0"] - cc["z1"]))
    ang_c = float(np.arctan2(cc["z0"] - cc["z1"], cc["x1"] - cc["x0"]))
    g.append(_incline_xml("chuteC",
                          ((cc["x0"] + cc["x1"]) / 2, cc["cy"], (cc["z0"] + cc["z1"]) / 2 - 0.015),
                          (cl / 2, cc["width"] / 2, 0.015), (0, ang_c, 0)))
    for sgn, nm in ((1, "l"), (-1, "r")):
        g.append(_incline_xml(f"chuteC_rail_{nm}",
                              ((cc["x0"] + cc["x1"]) / 2, cc["cy"] + sgn * (cc["width"] / 2 + rail_t),
                               (cc["z0"] + cc["z1"]) / 2 + 0.05),
                              (cl / 2, rail_t, 0.06), (0, ang_c, 0), rgba="0.9 0.75 0.4 1"))
    dl = float(np.hypot(cd["y0"] - cd["y1"], cd["z0"] - cd["z1"]))
    ang_d = float(np.arctan2(cd["z0"] - cd["z1"], cd["y0"] - cd["y1"]))
    g.append(_incline_xml("chuteD",
                          (cd["cx"], (cd["y0"] + cd["y1"]) / 2, (cd["z0"] + cd["z1"]) / 2 - 0.015),
                          (cd["width"] / 2, dl / 2, 0.015), (ang_d, 0, 0)))
    for sgn, nm in ((1, "l"), (-1, "r")):
        g.append(_incline_xml(f"chuteD_rail_{nm}",
                              (cd["cx"] + sgn * (cd["width"] / 2 + rail_t), (cd["y0"] + cd["y1"]) / 2,
                               (cd["z0"] + cd["z1"]) / 2 + 0.05),
                              (rail_t, dl / 2, 0.06), (ang_d, 0, 0), rgba="0.9 0.75 0.4 1"))

    for cname, cage in P.cages_for("table").items():
        g.append(_cage_xml(cname, cage))
    return g


def build_xml(manifest, mode=None):
    mode = mode or P.EXEC_DEFAULT
    a, b = P.BELT_A, P.BELT_B
    arm = P.ARM
    bx, by = P.ARM_BASE[mode]

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

    executive = "\n    ".join(_executive_geoms(mode))

    xml = f"""
<mujoco model="sortmaster_cell_{mode}">
  <compiler meshdir="{ASSETS / 'meshes'}" angle="radian"/>
  <option timestep="{P.SIM['timestep']}" integrator="implicitfast"/>
  <visual>
    <headlight ambient="0.45 0.45 0.45" diffuse="0.7 0.7 0.7"/>
    <global offwidth="1280" offheight="720"/>
  </visual>

  <asset>
    <texture type="2d" name="grid" builtin="checker" rgb1="0.90 0.90 0.92" rgb2="0.80 0.80 0.84" width="256" height="256"/>
    <material name="floor" texture="grid" texrepeat="12 8"/>
    {meshes}
  </asset>

  <worldbody>
    <light dir="0 0 -1" pos="6 3 6" directional="true"/>

    <!-- presentation cameras -->
    <camera name="top_view" pos="7.7 3.0 6.5" xyaxes="1 0 0 0 1 0"/>
    <camera name="overview" pos="4.8 -0.8 3.0" xyaxes="0.766 -0.645 0 0.272 0.322 0.911"/>

    <!-- look-ahead vision station: overhead depth + profile scanners; the
         sensor hangs BELOW its crossbar so the mount never shadows the FOV -->
    <camera name="lookahead" pos="6.0 3.0 2.2" xyaxes="1 0 0 0 1 0" fovy="45"/>
    <geom name="cam_post" type="cylinder" size="0.04 1.175" pos="6.0 2.2 1.175"
          rgba="0.45 0.25 0.55 1" contype="0" conaffinity="0"/>
    <geom name="cam_bar" type="box" size="0.03 0.42 0.03" pos="6.0 2.6 2.32"
          rgba="0.45 0.25 0.55 1" contype="0" conaffinity="0"/>
    <geom name="cam_head" type="box" size="0.06 0.06 0.035" pos="6.0 3.0 2.255"
          rgba="0.2 0.1 0.3 1" contype="0" conaffinity="0"/>

    <geom name="floor" type="plane" size="12 8 0.1" pos="5 3 0" material="floor"/>

    <!-- conveyor B: sorter infeed (FIXED) -->
    <geom name="beltB" type="box" size="{b['width'] / 2} {(b['y1'] - b['y0']) / 2} {b['top'] / 2}"
          pos="{b['cx']} {(b['y0'] + b['y1']) / 2} {b['top'] / 2}" rgba="0.35 0.42 0.55 1"/>

    <!-- executive architecture: {mode} -->
    {executive}

    <!-- 4-axis palletizer arm ({'primary picker' if mode == 'arm' else 'exception-recovery station'}) -->
    <geom name="pedestal" type="cylinder" size="0.15 {arm['pedestal_h'] / 2}"
          pos="{bx} {by} {arm['pedestal_h'] / 2}" rgba="0.25 0.25 0.28 1"/>
    <body name="yawcol" pos="{bx} {by} {arm['pedestal_h']}">
      <joint name="j1" type="hinge" axis="0 0 1" range="-7 7" damping="2"/>
      <geom type="cylinder" size="0.09 {arm['yaw_col_h'] / 2}" pos="0 0 {arm['yaw_col_h'] / 2}"
            rgba="0.85 0.55 0.1 1" contype="0" conaffinity="0"/>
      <body name="upper" pos="0 0 {arm['yaw_col_h']}">
        <joint name="j2" type="hinge" axis="0 1 0" range="-2.4 2.4" damping="2"/>
        <geom type="capsule" size="0.06" fromto="0 0 0 {arm['L1']} 0 0" rgba="0.85 0.55 0.1 1"
              contype="0" conaffinity="0"/>
        <body name="fore" pos="{arm['L1']} 0 0">
          <joint name="j3" type="hinge" axis="0 1 0" range="-2.8 2.8" damping="2"/>
          <geom type="capsule" size="0.05" fromto="0 0 0 {arm['L2']} 0 0" rgba="0.85 0.55 0.1 1"
                contype="0" conaffinity="0"/>
          <body name="wrist" pos="{arm['L2']} 0 0">
            <joint name="j4" type="hinge" axis="0 1 0" range="-3.0 3.0" damping="1"/>
            <geom type="cylinder" size="0.05 0.02" pos="0 0 -0.02" rgba="0.3 0.3 0.32 1"
                  contype="0" conaffinity="0"/>
            <geom name="tool" type="cylinder" size="0.04 0.1" pos="0 0 -0.148"
                  rgba="0.2 0.2 0.22 1" contype="0" conaffinity="0"/>
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
