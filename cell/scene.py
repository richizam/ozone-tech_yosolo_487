# -*- coding: utf-8 -*-
"""MJCF scene generation from cell.params — the layout has one source of truth."""
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
    walls = [(0, hy + t / 2, hx + t, t / 2), (0, -(hy + t / 2), hx + t, t / 2),
             (hx + t / 2, 0, t / 2, hy), (-(hx + t / 2), 0, t / 2, hy)]
    for i, (dx, dy, sx, sy) in enumerate(walls):
        g.append(f'<geom name="cage{name}_w{i}" type="box" size="{sx} {sy} {hh}" '
                 f'pos="{cx + dx} {cy + dy} {t + hh}" rgba="{rgba}"/>')
    return "\n      ".join(g)


def build_xml(manifest):
    a, b = P.BELT_A, P.BELT_B
    arm = P.ARM
    bx, by = arm["base"]

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

    xml = f"""
<mujoco model="sortmaster_cell">
  <compiler meshdir="{ASSETS / 'hulls'}" angle="radian"/>
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

    <geom name="floor" type="plane" size="12 8 0.1" pos="5 3 0" material="floor"/>

    <!-- conveyor A: belt body + accumulator stop wall + guide rails (FIXED axis/profile) -->
    <geom name="beltA" type="box" size="{(a['x_stop'] - a['x0']) / 2} {a['width'] / 2} {a['top'] / 2}"
          pos="{(a['x0'] + a['x_stop']) / 2} {a['y']} {a['top'] / 2}" rgba="0.35 0.42 0.55 1"/>
    <geom name="acc_stop" type="box" size="0.015 {a['width'] / 2} 0.05"
          pos="{a['x_stop'] + 0.015} {a['y']} {a['top'] + 0.05}" rgba="0.9 0.75 0.4 1"/>
    <geom name="acc_rail_l" type="box" size="0.66 0.015 0.08"
          pos="{a['x_stop'] - 0.645} {a['y'] + a['width'] / 2 + 0.015} {a['top'] + 0.08}" rgba="0.9 0.75 0.4 1"/>
    <geom name="acc_rail_r" type="box" size="0.66 0.015 0.08"
          pos="{a['x_stop'] - 0.645} {a['y'] - a['width'] / 2 - 0.015} {a['top'] + 0.08}" rgba="0.9 0.75 0.4 1"/>

    <!-- conveyor B: sorter infeed (FIXED) -->
    <geom name="beltB" type="box" size="{b['width'] / 2} {(b['y1'] - b['y0']) / 2} {b['top'] / 2}"
          pos="{b['cx']} {(b['y0'] + b['y1']) / 2} {b['top'] / 2}" rgba="0.35 0.42 0.55 1"/>

    <!-- roll-cages C / D (positions = our design) -->
      {_cage_xml("C", P.CAGE_C)}
      {_cage_xml("D", P.CAGE_D)}

    <!-- 4-axis palletizer arm (our design), collision-free v0 -->
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


def make_model():
    import mujoco
    manifest = load_manifest()
    xml = build_xml(manifest)
    model = mujoco.MjModel.from_xml_string(xml)
    return model, manifest, xml
