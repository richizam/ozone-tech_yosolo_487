# -*- coding: utf-8 -*-
"""Single source of truth for the cell geometry and motion parameters.

Everything the official scheme fixes is marked FIXED; everything else is our
design. Values in METERS (MuJoCo convention). The CAD script cad/layout_v0.py
consumes the same numbers (in mm) via LAYOUT_MM.
"""

# ---------------------------------------------------------------- work zone (FIXED)
ZONE = (10.0, 6.0)                     # x, y extent

# ---------------------------------------------------------------- conveyor A (axis/profile FIXED, length ours)
BELT_A = {
    "y": 3.0, "width": 0.5, "top": 0.7,
    "x0": 0.0, "x_stop": 8.1,          # stop wall of the accumulator
    "speed": 1.0,                       # m/s (FIXED)
    "acc_x0": 7.5,                      # accumulator = last 0.6 m with rails
    "gate_x": 6.3,                      # escapement gate: holds the next item
                                        # until the accumulator is clear
}

# ---------------------------------------------------------------- conveyor B (FIXED)
BELT_B = {"cx": 8.4, "y0": 4.2, "y1": 6.0, "width": 0.5, "top": 0.7, "speed": 1.0,
          "y_delivered": 5.7}

# ---------------------------------------------------------------- roll-cages 1200x800x800 (positions ours)
CAGE_C = {"center": (9.25, 2.95), "inner": (1.2, 0.8), "wall_h": 0.8, "wall_t": 0.03}
CAGE_D = {"center": (8.35, 2.0), "inner": (1.2, 0.8), "wall_h": 0.8, "wall_t": 0.03}

# ---------------------------------------------------------------- arm (ours): 4-axis palletizer kinematics
ARM = {
    "base": (8.4, 3.25),
    "pedestal_h": 0.65,
    "yaw_col_h": 0.25,                 # shoulder pivot z = pedestal_h + yaw_col_h
    "L1": 0.7, "L2": 0.6,              # upper arm / forearm
    "tool_len": 0.25,                  # wrist center -> TCP (suction plate)
    "reach": 1.3, "reach_margin": 0.15,
    "joint_vmax": (2.2, 2.0, 2.0, 3.0),  # rad/s per joint — palletizer-class
    "settle_attach_s": 0.20,
    "settle_release_s": 0.20,
}
SHOULDER_Z = ARM["pedestal_h"] + ARM["yaw_col_h"]  # 0.90

# ---------------------------------------------------------------- task points (derived design values)
LIFT_Z = 1.25                          # safe TCP transfer height
PLACE = {
    # B: gentle place onto the moving sorter infeed belt
    "B": {"xy": (BELT_B["cx"], 4.35), "mode": "place", "z_clear": 0.02},
    # C/D: controlled low drop into the cage (above wall + item)
    "C": {"xy": (8.9, 2.95), "mode": "drop", "z_clear": 0.05},
    "D": {"xy": (8.35, 2.15), "mode": "drop", "z_clear": 0.05},
}
CAGE_WALL_TOP = 0.83                   # cage floor 0.03 + walls 0.8

# ---------------------------------------------------------------- sorter limits (FIXED, official rules; mm)
LIMIT_MIN_MM = 10.0
LIMIT_MAX_MM = (450.0, 320.0, 320.0)
CIRCLE_RATIO = 0.8

# ---------------------------------------------------------------- simulation
SIM = {
    "timestep": 0.002,
    "control_decimation": 10,          # control at 50 Hz
    "settle_speed": 0.05,              # m/s: item considered settled below this
    "settle_time": 0.3,                # s at low speed before pick
}

# ---------------------------------------------------------------- mm view for CAD
def layout_mm():
    """The same layout in mm for drawings (cad/layout_v0.py)."""
    return {
        "zone": (ZONE[0] * 1000, ZONE[1] * 1000),
        "a_y": BELT_A["y"] * 1000, "a_width": BELT_A["width"] * 1000,
        "a_height": BELT_A["top"] * 1000,
        "a_x0": BELT_A["x0"] * 1000, "a_x1": BELT_A["acc_x0"] * 1000,
        "acc_len": (BELT_A["x_stop"] - BELT_A["acc_x0"]) * 1000,
        "cam_x": 6000.0,
        "arm_base": (ARM["base"][0] * 1000, ARM["base"][1] * 1000),
        "arm_reach": ARM["reach"] * 1000, "reach_margin": ARM["reach_margin"] * 1000,
        "b_cx": BELT_B["cx"] * 1000, "b_y0": BELT_B["y0"] * 1000,
        "b_y1": BELT_B["y1"] * 1000, "b_width": BELT_B["width"] * 1000,
        "cage_c_center": (CAGE_C["center"][0] * 1000, CAGE_C["center"][1] * 1000),
        "cage_c_size": (CAGE_C["inner"][0] * 1000, CAGE_C["inner"][1] * 1000),
        "cage_d_center": (CAGE_D["center"][0] * 1000, CAGE_D["center"][1] * 1000),
        "cage_d_size": (CAGE_D["inner"][0] * 1000, CAGE_D["inner"][1] * 1000),
        "fence": (6700.0, 1200.0, 10000.0, 6000.0),
    }
