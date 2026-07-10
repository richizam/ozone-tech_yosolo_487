# -*- coding: utf-8 -*-
"""Single source of truth for the cell geometry and motion parameters.

Everything the official scheme fixes is marked FIXED; everything else is our
design. Values in METERS (MuJoCo convention). The CAD script cad/layout_v0.py
consumes the same numbers (in mm) via LAYOUT_MM.

EXECUTIVE (v3): a linear TILT-TRAY SORTER. Items are classified in motion on
belt A, inducted one-per-carrier onto a moving train of shallow tilt trays,
and discharged by gravity at their route's station: C and D tilt SOUTH onto
32 deg brake chutes into the roll-cages, B tilts NORTH onto a powered incline
connector that feeds the fixed main-sorter belt B, REVIEW tilts NORTH into a
manual-review pen (low confidence / double occupancy / never-discharged).
Carrier transport is size-independent: an 11 mm cube rides and discharges
exactly like a 489 mm pouf — the reason this executive replaced the ARB deck
(a roller/ARB diverter has a ~50-75 mm minimum product footprint).
"""
import math

# ---------------------------------------------------------------- work zone (FIXED)
ZONE = (10.0, 6.0)                     # x, y extent

# ---------------------------------------------------------------- conveyor A (axis/profile FIXED, length ours)
BELT_A = {
    "y": 3.0, "width": 0.5, "top": 0.7,
    "x0": 0.0,
    "speed": 1.0,                       # m/s (FIXED)
    "nose_x": 6.90,                     # belt A ends in a knife-edge nose; the
                                        # item rides off it onto a passing tray
    "knife_x0": 6.40,                   # last stretch is a thin knife-edge
    "knife_t": 0.015,                   # section (15 mm) the trays run UNDER
    "gate_x": 6.35,                     # escapement stop: releases exactly one
                                        # item, synchronized to an empty carrier
    "hold2_x": 5.35,                    # pre-gate hold: keeps followers UPSTREAM
                                        # of the vision window so the item at the
                                        # gate is always measured alone
    # raised-blade drive target: panel bottom skims 3 mm above the belt so a
    # 9 mm pen / 10-11 mm cube cannot slip under a closed stop, while the
    # crest (0.123 m over the belt) stays un-climbable for the helmet
    # (needs a 0.123 m COM rise; 1 m/s carries 0.051 m)
    "blade_up": 0.169,
}

# ---------------------------------------------------------------- conveyor B (FIXED)
BELT_B = {"cx": 8.4, "y0": 4.2, "y1": 6.0, "width": 0.5, "top": 0.7, "speed": 1.0,
          "y_delivered": 5.7}

# ---------------------------------------------------------------- executive architecture
# "sorter": linear tilt-tray sorter (primary; size-independent carrier divert)
EXEC_DEFAULT = "sorter"

# ---------------------------------------------------------------- tilt-tray sorter train (ours)
# Carriers ride a closed track loop along the belt-A axis. The visible TOP RUN
# spans x_west..x_east at deck height; the RETURN RUN travels back under the
# deck at return_z (true return time — carrier availability is never
# optimistic). The traction chain is position-controlled at constant speed
# (per-carrier encoder); each TRAY is a dynamic rigid body on a real revolute
# joint + angular drive. Freight is moved by contact physics only.
SORTER = {
    "y": 3.0,                          # train axis = belt A axis
    "v_mps": 0.5,                      # chain speed
    "pitch": 0.6,                      # carrier pitch > largest footprint 0.5
    "n_carriers": 9,                   # loop = 2 x 2.7 m top/return = 5.4 m
                                       # -> 9 carriers at 0.6 m pitch
    "x_west": 6.72,                    # top-run entry (under the belt-A knife)
    "x_east": 9.42,                    # top-run end = east end-module face
    "return_z": 0.26,                  # under-deck return run height
                                       # (tray+lips top 0.34 clears the
                                       # deepened chute undersides at 0.355)
    # tray: shallow-V dished plate (real tilt trays are dished so round items
    # self-centre and cannot roll off during carry)
    "tray_l": 0.59, "tray_w": 0.62,    # x along travel / y across
    "tray_top": 0.64,                  # tray surface height at the centre line
    "tray_t": 0.024,
    "dish_deg": 2.0,                   # V half-plate inward angle
    "lip_h": 0.030, "lip_t": 0.012,    # low end fences on the +-x edges only
    "lip_w": 0.62,                     # full width: with the deep-drop chute
                                       # (no tray->chute bridge, no yaw drag)
                                       # a long box discharges cleanly between
                                       # full fences, and a rolling pen cannot
                                       # take a corner exit during the tilt
    "pivot_z": 0.616,                  # revolute (tilt) axis height, along +x
    "shuttle_top": 0.582,              # carrier body below the tray
    # tilt actuation (PhysX angular drive) + realistic command dynamics
    "tilt_deg": 38.0,                  # discharge tilt (see calcs: slide
                                       # guaranteed for mu_eff <= 0.32)
    "tilt_rate_dps": 160.0,            # drive target ramp
    "latency_s": 0.040,                # command -> drive onset
    "noise_frac": 0.01,                # per-command target gain noise (1 sigma)
    "settle_s": 0.40,                  # hold at full tilt before re-flatten
    "drive_stiffness": 800.0,
    "drive_damping": 80.0,
    # actuator sizing from the mass sweep: the heaviest item (sack) at
    # 1.3x lands ~0.11 m off-centre — the transient impact torque exceeded
    # a 90 N*m drive, the tray yielded and shed the freight sideways with
    # no tilt ever commanded. 150 N*m holds the worst-case landing.
    "drive_max_torque": 150.0,
    # tray surface: smooth low-friction (combine=min) so gravity discharge is
    # guaranteed for EVERY item incl. the mu=0.95 soft sack:
    # tan(38 deg)=0.781 >> 0.32
    "tray_mu": (0.32, 0.30),
    "occupied_callout_x": 9.30,        # an occupied carrier reaching the east
                                       # module = dead-tilt fault -> operator
    # SEAT TIME: no tilt command until this long after the tray is tagged —
    # a real sorter never tilts during landing settle. Long C-bound freight
    # otherwise tilts AT the landing instant (its length-extra pushes the C
    # trigger back to the landing point) and discharges while still bouncing
    # from the 60 mm induction drop (box_l edge_items_all floor drop).
    # 0.15 s: enough for a flat box to seat after the drop, small enough
    # that the C discharge point shifts < 0.08 m east (mouth half 0.35).
    "seat_time_s": 0.15,
    # DISCHARGE CONFIRM, shape-independent: the geometric "gone" condition
    # must PERSIST before the carrier is released (an item pivoting on the
    # tray edge oscillates through any threshold — a one-tick confirm let
    # the re-flatten scoop a mid-pivot sack), and the tray then dwells
    # tilted long enough for the item to finish falling before it moves.
    "confirm_persist_s": 0.15,
    "discharge_dwell_s": 0.90,
    # debris CATCH PAN under the top run: freight thinner than the
    # escapement's 3 mm skim gap (a 2 mm card) can arrive unmetered and
    # knife into an inter-tray gap — it lands on the pan and the watchdog
    # raises an operator call-out (a real sorter's drip pan / debris tray).
    # SWEEP-CORRIDOR CONSTRAINT: the full-tilt tray plane is
    # z = pivot_z - tan(tilt)*|y - y0| (0.616 - 0.781*|dy|), so the pan must
    # satisfy pan_z_top <= 0.616 - 0.781*pan_y_half - margin, and its edge
    # must stay north of the discharge fall corridor (|dy| < 0.24). A 0.33 x
    # 0.515 pan deflected the rolling pen off the C chute mouth (floor drop).
    "pan_z_top": 0.42, "pan_y_half": 0.22, "pan_x0": 6.78, "pan_x1": 9.28,
}

# ---------------------------------------------------------------- induction (physical, synchronized)
# The item is released by the escapement so that it rides off the knife nose
# and lands (60 mm drop) centred on its assigned tray. Landing offset is
# measured and logged per item (induction-sync evidence).
INDUCT = {
    "land_x": 7.01,                    # designed landing point (= nose + flight)
    "drop_m": 0.06,                    # nose top 0.70 -> tray top 0.64
    "flight_s": 0.11,                  # sqrt(2*drop/g)
    "accel_mps2": 5.0,                 # re-acceleration of a gate-held item
                                       # (belt friction mu~0.5 * g)
    "release_tol_s": 0.05,             # release timing tolerance
    "tag_radius_m": 0.22,              # landing->carrier association gate
    # rolling smalls: a lying cylinder (pen, rod) re-accelerates from a
    # blade hold by ROLLING — a_roll ~ (2/3)*mu*g, well under the sliding
    # lock the release model assumes. Aim such freight at a correspondingly
    # later tray, or a held pen lands ~0.2 m behind tray centre and the
    # association gate calls an induction miss (close-spacing floor drop).
    "roll_accel_frac": 0.42,           # a_roll / a_slide for held releases
    "roll_min_dim_m": 0.05,            # rolls if: thinnest dim under this,
    "roll_sect_max": 1.6,              #   round-ish section (mid/min), and
    "roll_min_elong": 2.5,             #   clearly elongated (a lying rod)
}

# ---------------------------------------------------------------- discharge stations
# order along the train: C (south), B (north), D (south), REVIEW (north).
# side: -1 tilts south, +1 tilts north. trigger_lead_m: the tilt command
# fires this far upstream of the station centre so the item (which keeps the
# carrier's +x velocity while sliding) lands centred on the chute.
STATIONS = {
    "C":      {"x": 7.45, "side": -1, "trigger_lead_m": 0.34},
    "B":      {"x": 8.40, "side": +1, "trigger_lead_m": 0.34},
    "D":      {"x": 8.75, "side": -1, "trigger_lead_m": 0.34},
    "REVIEW": {"x": 9.05, "side": +1, "trigger_lead_m": 0.34},
}

# gravity chutes from the tray lip into the cages: ONE continuous 32 deg
# slope (mu=0.40 < tan 32 deg, so no item can ever rest statically on it)
# that passes through the cage entry aperture and lands on a short
# high-friction BRAKE PAD just above the cage floor. z0 sits 50 mm under
# the tilted tray lip (lip_z ~ 0.445): the sliding item's leading edge
# stays AIRBORNE until its CG is nearly off the tray, so a long box can
# never bridge tray->chute and get yaw-dragged into a wedge (smoke10).
CHUTE = {"width": 0.70, "z0": 0.395, "z1": 0.16, "len_pad": 0.25,
         # polished slide sheet: mu/tan32 = 0.45 — small light freight (9 mm
         # rod, 10 mm cube) slides decisively even with convex-hull faceting
         # and solver stiction; at 0.40 it stalled 4 cm past the mouth
         "friction": "0.28 0.005 0.0001", "pad_friction": "0.45 0.01 0.0001",
         # MOUTH CHAMFER: a 59-deg infill strip at the mouth raises the catch
         # surface to z0+rise, shrinking the lip->chute free fall from 50 mm
         # to ~30 mm: thin flat freight (the plate) lands FLAT instead of
         # tipping onto its rim (a rim-rolling disc cleared the 40 mm stub
         # rails), and the ballistic under-fly window narrows. The top edge
         # stays clear of the tilted-lip tip trace (test-locked).
         "chamfer_rise": 0.020, "chamfer_run": 0.014,
         "chamfer_top_inset": 0.002,
         # MOUTH CHEEKS: side wings over the throat gap (lip->chamfer fall
         # zone). A hot small roller with residual +x drift can cross the
         # open sides of the gap before touching any chute surface (stress
         # s99 pen pogo at v_entry 2.4); the cheeks close that window. Their
         # top is capped by the tilting PLATE-EDGE arc (test-locked). All
         # mouth parts are STIFF: soft solref on a thin shell lets a fast
         # item penetrate past its thickness and wedge (solver blow-up) —
         # compliance belongs only on thick backed surfaces (brake pad,
         # cage mat).
         "cheek_len": 0.048, "cheek_h": 0.033, "cheek_inset": 0.004}
CHUTE_C = {"cx": STATIONS["C"]["x"], "y0": 2.75, "dir": -1, **CHUTE}
CHUTE_D = {"cx": STATIONS["D"]["x"], "y0": 2.75, "dir": -1, **CHUTE}
CHUTE_REVIEW = {"cx": STATIONS["REVIEW"]["x"], "y0": 3.25, "dir": +1,
                **{**CHUTE, "width": 0.62}}


def chute_run(cc):
    """(y_end_of_slope, y_end_of_pad) for a south(-1)/north(+1) chute."""
    dy = (cc["z0"] - cc["z1"]) / math.tan(math.radians(32.0))
    y1 = cc["y0"] + cc["dir"] * dy
    return y1, y1 + cc["dir"] * cc["len_pad"]


# powered incline connector from the B station tray lip up to the FIXED
# belt B infeed (rise 0.28 m over 0.94 m = 16.6 deg; every B item is
# non-round by rule, holds by friction: tan 16.6 = 0.30 << mu_pair ~0.6)
# width 0.62 = flared infeed: a 445 mm box lands with margin, then the
# fixed 0.5 m belt B's own side guides take over at the crest
B_CONNECT = {"cx": BELT_B["cx"], "width": 0.62, "y0": 3.26, "y1": 4.20,
             "z_top0": 0.42, "z_top1": 0.70, "speed": 0.9}

GUIDE_H = 0.14                          # chute side-guide height above the surface

# ---------------------------------------------------------------- item materials (ours)
# Per-item physical plausibility: friction/restitution/damping by material
# class. slug -> (static_friction, dynamic_friction, restitution,
#                 linear_damping, angular_damping). Mass stays manifest truth.
# Belt pairing combines "average"; the tray and chute pair "min" (guaranteed
# discharge slide); the brake pad pairs "max" (guaranteed braking).
MATERIALS = {
    "box_s": (0.55, 0.45, 0.05, 0.0, 0.05),     # cardboard box
    "box_l": (0.55, 0.45, 0.05, 0.0, 0.05),     # cardboard box (oversize)
    "lunchbox": (0.50, 0.42, 0.08, 0.0, 0.05),  # rigid plastic box
    "detergent": (0.45, 0.38, 0.10, 0.0, 0.05), # HDPE jug, irregular base
    "bottle": (0.35, 0.28, 0.15, 0.0, 0.05),    # PET, low friction, can roll
    "plate": (0.35, 0.30, 0.12, 0.0, 0.05),     # glazed ceramic, rolling risk
    "cylinder": (0.50, 0.42, 0.06, 0.0, 0.05),  # cardboard tube (hex prism)
    "helmet": (0.40, 0.32, 0.20, 0.0, 0.08),    # ABS shell, curved contact
    "sack": (0.95, 0.90, 0.00, 0.25, 0.60),     # soft sack: grips + damps
    "pouf": (0.85, 0.80, 0.00, 0.20, 0.50),     # fabric pouf: grips + damps
    "pen": (0.40, 0.35, 0.10, 0.0, 0.05),       # small plastic pen
    "_default": (0.90, 0.85, 0.00, 0.0, 0.05),
}

# route colour code, used consistently: lane markings, station lamps,
# destination beacons, cage frames and signage all share the zone colour
ROUTE_RGBA = {
    "B": (0.25, 0.55, 0.95),            # sorter = blue
    "C": (0.95, 0.55, 0.15),            # oversize = orange
    "D": (0.20, 0.78, 0.35),            # repack = green
    "REVIEW": (0.72, 0.25, 0.60),       # manual review = magenta
}

# ---------------------------------------------------------------- roll-cages 1200x800x800 (positions ours)
# The receiving wall carries an entry APERTURE the chute passes through: the
# slope crosses the wall plane at z ~0.24, so the aperture sill (skirt) is
# LOW (0.20) and the flanks/header close the rest — a routed item can roll or
# bounce inside the cage but cannot leave it again.
CAGES = {
    "sorter": {
        "C": {"center": (STATIONS["C"]["x"], 1.865), "inner": (0.8, 1.2),
              "wall_h": 0.8, "wall_t": 0.03, "open_side": "+y",
              "aperture_w": 0.76, "aperture_top": 0.83, "sill_top": 0.20},
        "D": {"center": (STATIONS["D"]["x"], 1.865), "inner": (0.8, 1.2),
              "wall_h": 0.8, "wall_t": 0.03, "open_side": "+y",
              "aperture_w": 0.76, "aperture_top": 0.83, "sill_top": 0.20},
    },
}
# manual-review pen (north-east): uncertain items, double occupancy and
# discharge-fault freight end HERE — a dedicated review zone, never a main
# flow. Same aperture-walled containment idea, smaller.
REVIEW_PEN = {"center": (STATIONS["REVIEW"]["x"], 3.865), "inner": (0.6, 0.7),
              "wall_h": 0.5, "wall_t": 0.03, "open_side": "-y",
              "aperture_w": 0.62, "aperture_top": 0.53, "sill_top": 0.20}
# high-friction landing mat on the cage floor at the entry half: kills the
# residual slide speed so items settle instead of ramming the far wall
CAGE_MAT_FRICTION = "0.9 0.01 0.0001"

# ---------------------------------------------------------------- containment validation
CONTAIN = {
    "margin": 0.06,                    # m beyond the outer wall face = escape
    "z_fly": 1.20,                     # anything this high left the cage volume
    "settle_speed": 0.10,              # m/s: item at rest inside the cage
    "settle_time": 0.5,                # s below settle_speed -> settled
    "cage_full_fraction": 0.60,
}


def cages_for(mode="sorter"):
    return CAGES["sorter"]

# ---------------------------------------------------------------- signage (billboards + floor decals)
SIGNS = [
    # key, EN, RU, (x, y, z of panel center), zone colour or None (steel grey)
    ("a_infeed",  "A — INFEED CONVEYOR",     "А — подача товаров",        (1.30, 3.80, 1.75), None),
    ("vision",    "VISION / MEASUREMENT",    "зона измерения товара",     (5.85, 3.80, 1.95), None),
    ("sorter",    "TILT-TRAY SORTER",        "сортер с поворотными лотками", (7.35, 3.75, 1.70), None),
    ("b_sorter",  "B — MAIN SORTER",         "В — основной сортировщик",  (8.42, 5.55, 1.75), "B"),
    ("c_oversize", "C — OVERSIZE",           "С — негабарит",             (7.45, 0.95, 1.55), "C"),
    ("d_repack",  "D — REPACK",              "D — доупаковка",            (8.75, 0.95, 1.55), "D"),
    ("review",    "MANUAL REVIEW",           "ручной разбор",             (9.75, 4.45, 1.55), "REVIEW"),
    ("arm_exc",   "EXCEPTION ARM",           "разбор нештатных ситуаций", (8.10, 1.42, 1.75), None),
]
FLOOR_DECALS = [
    # key reuses the sign texture; (x, y), yaw deg, half-size (len, wid)
    ("a_infeed",  (1.30, 2.30), 0.0, (0.55, 0.22)),
    ("vision",    (5.60, 2.20), 0.0, (0.55, 0.22)),
    ("b_sorter",  (7.75, 5.05), 90.0, (0.55, 0.22)),
    ("c_oversize", (6.35, 1.45), 90.0, (0.50, 0.22)),
    ("d_repack",  (9.65, 1.45), 90.0, (0.50, 0.22)),
]
OVERVIEW_CAM_XY = (4.8, -0.8)          # billboards yaw to face this camera

# ---------------------------------------------------------------- arm (ours): exception recovery only
ARM_BASE = {
    # exception station between the two cages, SOUTH of the chutes: the arm
    # reaches jams on the C and D chute bodies and places them route-correct
    # into their cages. It never reaches over the running train (grasp points
    # are clamped south of the tray sweep) and no link path crosses any
    # chute, cage or train structure at any commanded pose.
    # base sits in the INTER-CHUTE service aisle (between chute C's east
    # rail at x 7.80 and chute D's west rail at x 8.40, north of the cage
    # wall plane y 2.265) — the pedestal footprint overlaps nothing and no
    # link path ever crosses a cage wall: grasp (y <= 2.64), lift, transfer
    # and release all happen north of the wall plane.
    "sorter": (8.10, 2.42),
}
ARM_BASE_ISAAC = {"sorter": (8.10, 2.42)}
ARM = {
    "base": (8.10, 2.42),
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

# recovery policy (route-specific, keeps category correctness):
#   C snag on its chute  -> place into cage C
#   D snag on its chute  -> place into cage D
#   anything else (train top, B connector, induction) -> operator call-out;
#   a stuck TRAY needs no arm at all: its freight rides to the REVIEW
#   station / end-line call-out by design.
# grasp points are CLAMPED to y <= ARM_GRASP_Y_MAX: an item still overlapping
# the tray sweep (y > 2.62) is not a chute snag — it rides out safely.
ARM_GRASP_Y_MAX = 2.64
PLACE_BY_MODE = {
    # release onto the CHUTE LINE north of the cage wall plane (y 2.42 >
    # wall 2.265): the freight slides the last stretch through the aperture
    # like any routed item — the chute IS the cage's door, and the arm
    # never reaches over or through a wall. surface_z is the release height
    # above the local chute surface (z ~0.19 at y 2.42).
    "sorter": {
        "C": {"xy": (STATIONS["C"]["x"], 2.42), "mode": "drop", "z_clear": 0.05,
              "surface_z": 0.40},
        "D": {"xy": (STATIONS["D"]["x"], 2.42), "mode": "drop", "z_clear": 0.05,
              "surface_z": 0.40},
    },
}
LIFT_Z = 1.25                          # safe TCP transfer height
ARM_HOME_XY = {"sorter": (8.10, 1.55)}
JAM_TIMEOUT_S = 10.0                   # routing watchdog window
JAM_MIN_PROGRESS_M = 0.06              # less displacement per window = jammed
CAGE_WALL_TOP = 0.83                   # cage floor 0.03 + walls 0.8

# operator manual-handling station (call-out removals are placed here)
MANUAL_STATION = (10.4, 0.6)

# ---------------------------------------------------------------- sorter limits (FIXED, official rules; mm)
LIMIT_MIN_MM = 10.0
LIMIT_MAX_MM = (450.0, 320.0, 320.0)
CIRCLE_RATIO = 0.8

# ---------------------------------------------------------------- virtual sensor (implementation-true)
# The vision station is a DUAL-RANGE multi-head depth/dimensioning tunnel
# (DWS class): one overhead metrology head + two side profiler heads + one
# close-range MACRO head. The macro head engages only when the overhead
# estimate of the smallest dimension is below macro_engage_mm: legal-
# metrology guard bands scale with the measuring head's ground sampling, so
# the undersize certification floor is 10 mm + 2*gsd — 16 mm on the overhead
# head, ~10.8 mm on the macro head. An 11 mm cube therefore certifies as
# sortable (B) from the macro head, honestly, while a 10 mm cube and a 9 mm
# pen stay conservatively C. Items are measured IN MOTION.
VIRTUAL_SENSOR = {
    "type": "multi_head_depth_profiler_dual_range",
    "model": "ray_cast_depth_grid+light_section_profilers+macro_head",
    "conveyor_speed_mps": BELT_A["speed"],      # items are measured IN MOTION
    "window_x": (5.65, 6.05),                   # measurement window on belt A;
                                                # every verdict commits BEFORE
                                                # the item presses the gate
    # the overhead head sits UPSTREAM of the window centre so the macro
    # head's housing/mount (hanging at z 1.42 over the window centre) never
    # enters its optical path: the rig's projected shadow lands at x > 6.1,
    # past every read (a dual-range tunnel must not self-occlude)
    "overhead_pos": (5.55, 3.0, 2.2),           # overhead head (x, y, z), m
    "ground_res_mm": 3.0,                       # overhead grid ground sampling
    "guard_overhead_mm": 6.0,                   # 2 * overhead gsd class
    "macro_pos": (5.85, 3.0, 1.42),             # close-range head (clears the
                                                # 0.5 m max inbound envelope)
    "macro_fov_deg": 24.0,                      # -> ~0.31 m footprint at belt
    "macro_gsd_mm": 0.40,
    "guard_macro_mm": 0.80,                     # 2 * macro gsd
    "macro_engage_mm": 25.0,                    # engage when min dim < this
    "profile_plane_spacing_mm": 4.0,            # light-section plane pitch
    "profile_angular_res_deg": 0.1,             # top profiler fan resolution
    "side_head_angular_res_deg": 0.2,           # side profiler fan resolution
    "side_head_offset_m": 0.45,                 # side heads' lateral offset
    "side_head_z_m": 1.05,                      # side heads' height
    "capture_period_s": 0.12,                   # multi-read cadence (~8.3 Hz)
    "depth_noise_mm": 0.0,                      # Gaussian sigma per ray
    "processing_latency_s": 0.08,               # fusion verdict -> route command
    "blind_zone_note": "single item per window enforced by the pre-gate hold",
}

# ---------------------------------------------------------------- classification policy (used by run fusion)
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
    # 1 kHz: a 50 g pen settling under multi-kg cage freight on compliant
    # pads is a stiff 60:1 mass-ratio contact stack — at 2 ms it explodes
    # (mjWARN_BADQACC) whenever arrival micro-timing lines up; halving the
    # step removes the class instead of re-tuning compliances per shuffle.
    "timestep": 0.001,
    "control_decimation": 20,          # control at 50 Hz
    "settle_speed": 0.05,              # m/s: item considered settled below this
    "settle_time": 0.3,                # s at low speed before pick
}

# ---------------------------------------------------------------- derived design checks (imported by tests)
TRAY_LIP_Z = (SORTER["pivot_z"]
              + (SORTER["tray_top"] - SORTER["pivot_z"])
              * math.cos(math.radians(SORTER["tilt_deg"]))
              - SORTER["tray_w"] / 2 * math.sin(math.radians(SORTER["tilt_deg"])))
SLIDE_ONSET_DEG = math.degrees(math.atan(SORTER["tray_mu"][0]))  # ~17.7
B_INCLINE_DEG = math.degrees(math.atan(
    (B_CONNECT["z_top1"] - B_CONNECT["z_top0"])
    / (B_CONNECT["y1"] - B_CONNECT["y0"])))


# ---------------------------------------------------------------- mm view for CAD
def layout_mm():
    """The same layout in mm for drawings (cad/layout_v0.py)."""
    cg = cages_for()
    return {
        "zone": (ZONE[0] * 1000, ZONE[1] * 1000),
        "a_y": BELT_A["y"] * 1000, "a_width": BELT_A["width"] * 1000,
        "a_height": BELT_A["top"] * 1000,
        "a_x0": BELT_A["x0"] * 1000, "a_x1": BELT_A["nose_x"] * 1000,
        "cam_x": VIRTUAL_SENSOR["overhead_pos"][0] * 1000,
        "arm_base": (ARM["base"][0] * 1000, ARM["base"][1] * 1000),
        "arm_reach": ARM["reach"] * 1000, "reach_margin": ARM["reach_margin"] * 1000,
        "b_cx": BELT_B["cx"] * 1000, "b_y0": BELT_B["y0"] * 1000,
        "b_y1": BELT_B["y1"] * 1000, "b_width": BELT_B["width"] * 1000,
        "sorter_x": (SORTER["x_west"] * 1000, SORTER["x_east"] * 1000),
        "sorter_pitch": SORTER["pitch"] * 1000,
        "stations": {k: v["x"] * 1000 for k, v in STATIONS.items()},
        "cage_c_center": (cg["C"]["center"][0] * 1000, cg["C"]["center"][1] * 1000),
        "cage_c_size": (cg["C"]["inner"][0] * 1000, cg["C"]["inner"][1] * 1000),
        "cage_d_center": (cg["D"]["center"][0] * 1000, cg["D"]["center"][1] * 1000),
        "cage_d_size": (cg["D"]["inner"][0] * 1000, cg["D"]["inner"][1] * 1000),
        "fence": (6700.0, 1200.0, 10000.0, 6000.0),
    }
