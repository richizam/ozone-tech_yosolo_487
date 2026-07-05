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
    "hold2_x": 5.5,                     # pre-gate hold: keeps followers UPSTREAM
                                        # of the vision window so the item at the
                                        # gate is always measured alone
}

# ---------------------------------------------------------------- conveyor B (FIXED)
BELT_B = {"cx": 8.4, "y0": 4.2, "y1": 6.0, "width": 0.5, "top": 0.7, "speed": 1.0,
          "y_delivered": 5.7}

# ---------------------------------------------------------------- executive architecture
# "table": tri-directional powered transfer table routes every item without
#          grasping; the arm is an exception-recovery unit (jams, unstable
#          items) — the primary architecture (see transfer_table_roadmap_changes.md).
# "arm":   the preserved arm-primary baseline (tag arm-primary-baseline).
EXEC_DEFAULT = "table"

# ---------------------------------------------------------------- transfer table (ours, "table" mode)
TABLE = {
    "x0": 6.9, "x1": 8.55,             # belt A hands over at x0; routing zone at the end
    "y": 3.0, "width": 1.1, "top": 0.7,
    "speed": 0.8,                       # controlled surface speed on the table
    "route_x": 7.95,                    # routing zone start: lateral drives engage here
    "lane_B_cx": 8.4,                   # exit north through the connector to belt B
    "lane_C_cy": 3.0,                   # exit east over the chute into cage C
    "lane_D_cx": 8.05,                  # exit south over the chute into cage D
}
# short powered connector from the table's north edge to the fixed belt B
CONNECT_B = {"cx": 8.4, "width": 0.5, "y0": 3.55, "y1": 4.2, "top": 0.7, "speed": 0.8}
# gravity chutes from the table edges into the cages: ONE continuous 32 deg
# slope (mu=0.40 < tan 32 deg, so no item can ever rest statically on it — no
# seams, no stall points, recovery drops always slide) that passes through the
# cage entry aperture and lands on a short high-friction BRAKE PAD just above
# the cage floor. The item is guided and decelerated the whole way down:
# never thrown, never free-falling.
CHUTE_C = {"x0": 8.55, "x1": 9.40, "cy": 3.0, "width": 0.62, "z0": 0.7, "z1": 0.16,
           "friction": "0.40 0.005 0.0001",
           "pad_x1": 9.65, "pad_friction": "0.45 0.01 0.0001"}
CHUTE_D = {"y0": 2.45, "y1": 1.60, "cx": 8.05, "width": 0.62, "z0": 0.7, "z1": 0.16,
           "friction": "0.40 0.005 0.0001",
           "pad_y1": 1.35, "pad_friction": "0.45 0.01 0.0001"}
GUIDE_H = 0.14                          # chute side-guide height above the surface
# chute hood: a cover parallel to the slope over the cage aperture — closes
# the fly-out window without presenting any catch face to the flow (its
# upstream edge sits ~0.98 high, above any nose rotating off the table lip)
HOOD = {"clearance": 0.50,              # normal gap slope->hood (max item 0.36)
        "up": 0.30, "into": 0.28,       # span upstream / into the cage
        "brow_z0": 0.80}                # wall-plane brow strip (hood top overlaps it;
                                        # item apexes pass at <= 0.78)

# actuated vertical-lift exit gates on the transfer table (closed by default:
# an item can only leave through the gate its route command opened)
GATES = {
    "travel": 0.66,                     # lift stroke, m: open panel bottom at 1.364
                                        # clears the 500 mm max item AND an arm-carried
                                        # load at max lift (item top <= 1.29)
    "panel_h": 0.30, "panel_t": 0.024,  # panel height / thickness
    "gap": 0.004,                       # closed-state clearance over the table top
    "B": {"axis": "x", "c0": 8.15, "c1": 8.55, "line": 3.532},   # north exit
    "C": {"axis": "y", "c0": 2.69, "c1": 3.31, "line": 8.528},   # east exit
    "D": {"axis": "x", "c0": 7.74, "c1": 8.36, "line": 2.468},   # south exit
}

# route colour code, used consistently: lane markings, gate lamps, destination
# beacons, cage frames and signage all share the zone colour
ROUTE_RGBA = {
    "B": (0.25, 0.55, 0.95),            # sorter = blue
    "C": (0.95, 0.55, 0.15),            # oversize = orange
    "D": (0.20, 0.78, 0.35),            # repack = green
}

# ---------------------------------------------------------------- roll-cages 1200x800x800 (positions ours)
# per executive mode. In table mode the receiving wall is NOT left open: it
# carries an entry APERTURE just big enough for the chute runout plus item
# clearance — flanks and a header strip close the rest, so a routed item can
# roll or bounce inside the cage but cannot leave it again.
CAGES = {
    "arm": {
        "C": {"center": (9.25, 2.95), "inner": (1.2, 0.8), "wall_h": 0.8, "wall_t": 0.03,
              "open_side": None, "long_axis": "x"},
        "D": {"center": (8.35, 2.0), "inner": (1.2, 0.8), "wall_h": 0.8, "wall_t": 0.03,
              "open_side": None, "long_axis": "x"},
    },
    "table": {
        # rotated 90 deg (0.8 across belt direction) so it fits inside the
        # 10 m work zone; the aperture faces the C chute. aperture_top = wall
        # top: the vertical clearance is closed by the chute HOOD + brow
        # (parallel to the flow — a header strip would be a catch face for
        # long items still rotating off the table lip).
        "C": {"center": (9.5, 3.0), "inner": (0.8, 1.2), "wall_h": 0.8, "wall_t": 0.03,
              "open_side": "-x", "aperture_w": 0.80, "aperture_top": 0.83},
        # aperture faces the D chute (the slope crosses the wall at y=1.965)
        "D": {"center": (8.05, 1.55), "inner": (1.2, 0.8), "wall_h": 0.8, "wall_t": 0.03,
              "open_side": "+y", "aperture_w": 0.80, "aperture_top": 0.83},
    },
}
# high-friction landing mat on the cage floor at the entry half: kills the
# residual slide speed so items settle instead of ramming the far wall
CAGE_MAT_FRICTION = "0.9 0.01 0.0001"

# ---------------------------------------------------------------- containment validation
CONTAIN = {
    "margin": 0.06,                    # m beyond the outer wall face = escape
    "z_fly": 1.20,                     # anything this high left the cage volume
    "settle_speed": 0.10,              # m/s: item at rest inside the cage
    "settle_time": 0.5,                # s below settle_speed -> settled
    # fill-level limit (modeled fill sensor): when the accumulated footprint
    # of contained items plus the incoming one exceeds this fraction of the
    # cage floor, the cage is FULL — the item is pulled to manual handling
    # and a cage-swap call-out is raised (piling above the 0.8 m walls is how
    # items escape; a real cell swaps the roll-cage instead)
    "cage_full_fraction": 0.60,
}
CAGE_C = CAGES["arm"]["C"]              # arm-mode aliases (existing code paths)
CAGE_D = CAGES["arm"]["D"]


def cages_for(mode):
    return CAGES[mode]

# ---------------------------------------------------------------- signage (billboards + floor decals)
# every functional station is labelled, EN headline + RU subtitle; billboards
# auto-face the overview camera so the demo video reads at a glance
SIGNS = [
    # key, EN, RU, (x, y, z of panel center), zone colour or None (steel grey)
    ("a_infeed",  "A — INFEED CONVEYOR",     "А — подача товаров",        (1.30, 3.80, 1.75), None),
    ("vision",    "VISION / MEASUREMENT",    "зона измерения товара",     (6.00, 3.80, 1.95), None),
    ("table",     "ACTIVE TRANSFER TABLE",   "активный стол-перекладчик", (7.25, 4.05, 1.70), None),
    ("b_sorter",  "B — MAIN SORTER",         "В — основной сортировщик",  (8.42, 5.55, 1.75), "B"),
    ("c_oversize", "C — OVERSIZE",           "С — негабарит",             (9.55, 4.00, 1.55), "C"),
    ("d_repack",  "D — REPACK",              "D — доупаковка",            (7.05, 1.30, 1.55), "D"),
    ("arm_exc",   "EXCEPTION ARM",           "разбор нештатных ситуаций", (9.30, 1.95, 1.75), None),
]
FLOOR_DECALS = [
    # key reuses the sign texture; (x, y), yaw deg, half-size (len, wid)
    ("a_infeed",  (1.30, 2.30), 0.0, (0.55, 0.22)),
    ("vision",    (6.00, 2.20), 0.0, (0.55, 0.22)),
    ("b_sorter",  (7.75, 5.05), 90.0, (0.55, 0.22)),
    ("c_oversize", (9.45, 2.15), 0.0, (0.50, 0.22)),
    ("d_repack",  (6.70, 1.75), 0.0, (0.50, 0.22)),
]
OVERVIEW_CAM_XY = (4.8, -0.8)          # billboards yaw to face this camera

# ---------------------------------------------------------------- arm (ours): 4-axis palletizer kinematics
ARM_BASE = {
    "arm": (8.4, 3.25),                 # primary picker position (baseline)
    "table": (8.55, 2.2),               # exception station: covers the routing
                                        # zone, all three lane starts, the D
                                        # chute and the D drop (reach-checked)
}
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
PLACE_BY_MODE = {
    "arm": {
        # B: gentle place onto the moving sorter infeed belt
        "B": {"xy": (BELT_B["cx"], 4.35), "mode": "place", "z_clear": 0.02},
        # C/D: controlled low drop into the cage (above wall + item)
        "C": {"xy": (8.9, 2.95), "mode": "drop", "z_clear": 0.05},
        "D": {"xy": (8.35, 2.15), "mode": "drop", "z_clear": 0.05},
    },
    "table": {
        # recovery PLACES the item back on the table at its lane exit (gentle,
        # «мягкость обращения») — the route stays assigned, so the table drive
        # re-delivers it through the normal guided path (gate -> chute -> cage)
        "C": {"xy": (8.30, 3.0), "mode": "drop", "z_clear": 0.02, "surface_z": 0.70},
        "D": {"xy": (8.05, 2.60), "mode": "drop", "z_clear": 0.02, "surface_z": 0.70},
    },
}
PLACE = PLACE_BY_MODE["arm"]           # legacy alias
ARM_HOME_XY = {"arm": (7.9, 3.0), "table": (8.35, 2.3)}
JAM_TIMEOUT_S = 10.0                   # routing watchdog window
JAM_MIN_PROGRESS_M = 0.06              # less displacement per window = jammed
                                       # (queue creep is flow, not a fault)
CAGE_WALL_TOP = 0.83                   # cage floor 0.03 + walls 0.8

# ---------------------------------------------------------------- sorter limits (FIXED, official rules; mm)
LIMIT_MIN_MM = 10.0
LIMIT_MAX_MM = (450.0, 320.0, 320.0)
CIRCLE_RATIO = 0.8

# ---------------------------------------------------------------- virtual sensor (implementation-true)
# The `camera` perception mode is a virtual multi-head depth/dimensioning
# station (DWS-tunnel class), implemented in perception/pipeline.py by ray
# casting against the true item surface. Every value below is CONSUMED by the
# implementation — nothing here is declarative-only. `oracle` mode bypasses
# this sensor entirely and injects ground truth (debug baseline).
VIRTUAL_SENSOR = {
    "type": "multi_head_depth_profiler",
    "model": "ray_cast_depth_grid+light_section_profilers",
    "conveyor_speed_mps": BELT_A["speed"],      # items are measured IN MOTION
    "window_x": (5.85, 6.28),                   # measurement window on belt A (m);
                                                # ends before the escapement gate
    "overhead_pos": (6.0, 3.0, 2.2),            # overhead head (x, y, z), m
    "ground_res_mm": 3.0,                       # overhead grid ground sampling
    "profile_plane_spacing_mm": 4.0,            # light-section plane pitch along belt
    "profile_angular_res_deg": 0.1,             # top profiler fan resolution
    "side_head_angular_res_deg": 0.2,           # side profiler fan resolution
    "side_head_offset_m": 0.45,                 # side heads' lateral offset
    "side_head_z_m": 1.05,                      # side heads' height
    "capture_period_s": 0.12,                   # multi-read cadence (~8.3 Hz)
    "depth_noise_mm": 0.0,                      # Gaussian sigma per ray; 0 = ideal
                                                # optics baseline (scenario-tunable:
                                                # sensor: {depth_noise_mm: ...})
    "processing_latency_s": 0.08,               # fusion verdict -> route command
    "blind_zone_note": "single item per window enforced by the pre-gate hold",
}

# ---------------------------------------------------------------- classification policy (used by run_sim fusion)
# Safe-side decision policy on top of the official rule order. All knobs are
# implementation-true (consumed in cell/run_sim.py) and scenario-tunable via a
# `classification:` block.
CLASSIFICATION = {
    "min_confidence_for_B": 0.30,   # fused confidence below this never routes to B
    "low_confidence_route": "D",    # ...it diverts to the repack/manual-review lane
    "stable_reads_required": 5,     # commit after N reads with a stable tail
    "stable_dims_tol_mm": 8.0,      # two reads "agree" within this
    "read_cap": 30,                 # hard cap on reads per item
    "weak_evidence_reroute": True,  # persistent weak circle evidence -> D (never B)
}

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
