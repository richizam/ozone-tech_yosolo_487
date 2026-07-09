# -*- coding: utf-8 -*-
"""SortMaster cell in NVIDIA Isaac Sim (PhysX) — the same validated closed
loop as cell/run_sim.py, re-implemented on the Omniverse stack.

Parity with the MuJoCo build (single source of truth: cell/params.py):
  * kinematic conveyor idiom: the drive writes linear velocity while the item
    rides a powered surface (belt A -> transfer table -> connector -> belt B),
    angular velocity damped like a real belt does to a light object;
  * flow discipline: escapement gate + pre-gate hold (single item in the
    measurement window, the BELT NEVER STOPS), zone-accumulation queue lines;
  * look-ahead classification: the verdict commits as the item leaves the
    vision window + processing latency -> route command is ready well before
    table entry (command_margin_s is measured and reported);
  * tri-directional transfer table with normally-closed actuated exit gates
    (PhysX prismatic joints + linear drives) -> guided 32 deg brake chutes ->
    aperture-walled roll cages; containment is tracked to the end of the run;
  * jam watchdog: zero displacement over the timeout window -> operator
    call-out (the arm-recovery drill remains in the MuJoCo twin).

Classification (--perception rtx, default) comes from the REAL sensor: a
3-head RTX depth station (overhead + two side profiler heads, the same
VIRTUAL_SENSOR architecture as the MuJoCo twin) measured in motion with
multi-read fusion and guard-banded official rule order — calibrated 33/33 =
100% on the official set across 3 rest yaws (isaac/validate_rtx.py).
--perception oracle keeps the ground-truth debug baseline.

Run inside the Isaac Sim container:
  /isaac-sim/python.sh isaac/run_isaac.py --out /tmp/sortmaster/run1 --record
"""
import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from cell import params as P                      # noqa: E402  (pure python)

SPIN_DAMP = 0.85
GATE_CLEAR = {"B": ("y", +1, 3.85), "C": ("x", +1, 8.90), "D": ("y", -1, 2.15)}
QUEUE_GAP = 0.15
Z_ON = (0.67, 1.05)          # item center z while riding a 0.7-high surface


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", default="/tmp/sortmaster/run1")
    ap.add_argument("--record", action="store_true", help="capture MP4 frames")
    ap.add_argument("--camera", default="overview",
                    choices=["overview", "top_view", "routing", "lookahead",
                             "hero_sw", "deck_front", "deck_top", "cell_iso"])
    ap.add_argument("--camera-path", default="none",
                    choices=["none", "orbit", "dolly", "crane", "deck_push"],
                    help="cinematic moving camera for the defense reel — "
                         "drives the recording camera along a smooth path")
    ap.add_argument("--path-secs", type=float, default=80.0,
                    help="seconds over which the cinematic path sweeps 0->1")
    ap.add_argument("--follow-slug", default=None,
                    help="item-follow cinematic camera: trail this item's "
                         "live pose from the vision station through the ARB "
                         "deck into its bin (final-video shot)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--perception", choices=["rtx", "oracle"], default="rtx",
                    help="rtx = classify from the 3-head RTX depth station; "
                         "oracle = ground truth after lookahead latency (debug)")
    ap.add_argument("--depth-stills", type=int, default=3,
                    help="RGB+depth captures from the vision station")
    ap.add_argument("--spawn-gap", default="6.0,8.0")
    ap.add_argument("--max-sim-s", type=float, default=400.0)
    ap.add_argument("--physics-hz", type=int, default=240)
    ap.add_argument("--control-hz", type=int, default=60)
    ap.add_argument("--items", default=None,
                    help="comma list of slugs (default: all manifest items)")
    ap.add_argument("--drive", choices=["surface", "scripted"], default="surface",
                    help="surface = PhysX surface-velocity conveyors + pop-up "
                         "blades (Conveyor Belt utility mechanism, default); "
                         "scripted = legacy per-item velocity writes")
    ap.add_argument("--inject-jam", default=None, metavar="SLUG@X",
                    help="fault drill: stop driving SLUG once it passes x=X "
                         "on the table (snag) so the watchdog + jam camera "
                         "fire, e.g. box_s@8.0")
    ap.add_argument("--friction-mult", type=float, default=1.0,
                    help="material sweep: scale every item's friction pair "
                         "(0.7 = low-friction robustness run)")
    ap.add_argument("--mass-mult", type=float, default=1.0,
                    help="material sweep: scale every item's mass "
                         "(1.3 = heavy-item robustness run)")
    ap.add_argument("--spawn-offset-y", type=float, default=0.0,
                    help="fault drill: spawn items off-center by this lateral "
                         "offset in meters (belt guides sit at +-0.265)")
    ap.add_argument("--inject-gate-fault", default=None,
                    metavar="ZONE:MODE",
                    help="gate fault drill (GATED_ACTUATOR_TEST_PLAN): "
                         "C:stuck_closed | B:stuck_open | C:delay:400 — the "
                         "controller intent is unchanged, the hardware "
                         "misbehaves, the cell must fail safe")
    ap.add_argument("--probe", default=None,
                    help="slug to trace in detail near the table exit")
    return ap.parse_args()


def main():
    args = parse_args()
    out_dir = Path(args.out)
    (out_dir / "frames").mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------- boot Kit
    try:
        from isaacsim import SimulationApp
    except ImportError:
        from isaacsim.simulation_app import SimulationApp
    sim_app = SimulationApp({"headless": True, "width": 1280, "height": 720})
    # FIXED exposure: RTX auto-exposure adapts to the dark belts across the
    # run's hundreds of mixed-camera renders and pushes video frames ~2
    # stops up — the correctly-dark cage/panels then clip to cream (the
    # "blown-out C bin"). A probe render of the SAME stage without the
    # adaptation showed the intended industrial exposure.
    try:
        import carb
        _st = carb.settings.get_settings()
        _st.set("/rtx/post/histogram/enabled", False)
    except Exception:
        pass

    from isaacsim.core.api import World
    try:
        from isaacsim.core.prims import SingleRigidPrim
    except ImportError:                             # older namespace
        from omni.isaac.core.prims import RigidPrim as SingleRigidPrim
    from isaacsim.sensors.camera import Camera
    from pxr import UsdGeom, Gf                     # noqa: F401

    from isaac.scene_usd import SceneBuilder, load_manifest

    # rendering_dt == physics_dt so a step never advances more than one
    # physics substep; frame cadence is managed manually via world.render()
    world = World(physics_dt=1.0 / args.physics_hz,
                  rendering_dt=1.0 / args.physics_hz,
                  stage_units_in_meters=1.0, backend="numpy")
    try:
        world.get_physics_context().enable_ccd(True)
    except Exception:
        pass
    stage = world.stage

    manifest = load_manifest(REPO)
    if args.items:
        keep = set(args.items.split(","))
        manifest = [e for e in manifest if e["slug"] in keep]
    entries = {e["slug"]: e for e in manifest}

    builder = SceneBuilder(stage, REPO, friction_mult=args.friction_mult,
                           mass_mult=args.mass_mult)
    info = builder.build(manifest)
    from isaac.arm import build_arm, ArmController
    arm_rig = build_arm(builder, mode="table")

    world.reset()

    # ---------------------------------------------------------- prim wrappers
    items_rp = {}
    for slug, meta in info["items"].items():
        rp = SingleRigidPrim(meta["path"], name=f"rp_{slug}")
        if hasattr(rp, "initialize"):
            rp.initialize()
        # PhysX skips sleeping bodies entirely — a briefly-stopped item would
        # ignore every subsequent velocity write and freeze forever (found on
        # the chute-crest bridge). Force never-sleep through the LIVE view;
        # the USD sleepThreshold attribute alone did not take effect.
        view = getattr(rp, "_rigid_prim_view", None)
        if view is not None:
            try:
                before = view.get_sleep_thresholds()
                view.set_sleep_thresholds(np.zeros(1))
                print(f"[isaac] sleep_thr {slug}: {before} -> "
                      f"{view.get_sleep_thresholds()}", flush=True)
            except Exception as exc:
                print(f"[isaac] sleep_thr {slug}: FAILED {exc}", flush=True)
        items_rp[slug] = rp

    gate_target_attr = {}
    gate_rp = {}
    gate_z0 = {}
    for zone, g in info["gates"].items():
        prim = stage.GetPrimAtPath(g["joint"])
        gate_target_attr[zone] = prim.GetAttribute("drive:linear:physics:targetPosition")
        rp = SingleRigidPrim(g["body"], name=f"rp_gate{zone}")
        if hasattr(rp, "initialize"):
            rp.initialize()
        gate_rp[zone] = rp
        gate_z0[zone] = g["z0"]

    conv_attr, blade_attr, deck = {}, {}, None
    if args.drive == "surface":
        from pxr import PhysxSchema
        for name, path in info["conveyors"].items():
            api = PhysxSchema.PhysxSurfaceVelocityAPI(stage.GetPrimAtPath(path))
            conv_attr[name] = api.GetSurfaceVelocityAttr()
        for name, jpath in info["blades"].items():
            blade_attr[name] = stage.GetPrimAtPath(jpath).GetAttribute(
                "drive:linear:physics:targetPosition")
        # ARB routing deck: matrix of local actuator patches with real
        # actuation dynamics (latency / ramp / saturation / noise)
        from isaac.arb_deck import ArbDeck
        deck = ArbDeck(stage, info["arb_patches"], feed_speed=P.TABLE["speed"],
                       seed=args.seed,
                       pill_paths=(info.get("viz") or {}).get("arb_pills"),
                       roller_paths=(info.get("viz") or {}).get("arb_rollers"))
        print(f"[isaac] ARB deck: {len(info['arb_patches'])} patches "
              f"({P.ARB_DECK['nx']}x{P.ARB_DECK['ny']}), latency "
              f"{P.ARB_DECK['latency_s']*1000:.0f}ms ramp "
              f"{P.ARB_DECK['ramp_mps2']}m/s2", flush=True)

    view_cam = Camera(prim_path=info["cams"][args.camera],
                      resolution=(1280, 720))
    view_cam.initialize()
    cine = None
    follow = None
    if args.follow_slug:
        from isaac.cinematic import FollowCamera
        follow = FollowCamera(stage, info["cams"][args.camera])
        print(f"[isaac] item-follow camera on '{args.follow_slug}'", flush=True)
    elif args.camera_path != "none":
        from isaac.cinematic import CinematicCamera
        cine = CinematicCamera(stage, info["cams"][args.camera],
                               args.camera_path)
        print(f"[isaac] cinematic camera path={args.camera_path} "
              f"over {args.path_secs}s", flush=True)
    look_cam = None
    perc = None
    if args.depth_stills > 0 or args.perception == "rtx":
        look_cam = Camera(prim_path=info["cams"]["lookahead"],
                          resolution=(1024, 768))
        look_cam.initialize()
        look_cam.add_distance_to_image_plane_to_frame()
    jam_loc = None
    if args.perception == "rtx":
        from isaac.perception_rtx import RTXPerception, fuse_reads
        from isaac.jam_locator import JamLocator
        side_cams = []
        for key in ("side_a", "side_b"):
            sc = Camera(prim_path=info["cams"][key], resolution=(768, 576))
            sc.initialize()
            sc.add_distance_to_image_plane_to_frame()
            side_cams.append(sc)
        perc = RTXPerception([look_cam] + side_cams)
        jc = Camera(prim_path=info["cams"]["jamcam"], resolution=(1024, 768))
        jc.initialize()
        jc.add_distance_to_image_plane_to_frame()
        cages_t = P.cages_for("table")
        wall_c = (cages_t["C"]["center"][0] - cages_t["C"]["inner"][0] / 2
                  - cages_t["C"]["wall_t"] / 2)
        wall_d = (cages_t["D"]["center"][1] + cages_t["D"]["inner"][1] / 2
                  + cages_t["D"]["wall_t"] / 2)
        hood = P.HOOD
        jam_loc = JamLocator(jc, blade_lines=[
            P.BELT_A["gate_x"] + 0.02, P.BELT_A["hold2_x"] + 0.02,
            P.BELT_A["hold2_x"] - 0.60, P.BELT_A["hold2_x"] - 1.20,
            P.TABLE["route_x"] - 0.02],
            hood_zones=[
                # C hood footprint (+margins): x along the slope, y = chute
                (wall_c - hood["up"] - 0.06, wall_c + hood["into"] + 0.16,
                 P.CHUTE_C["cy"] - 0.37, P.CHUTE_C["cy"] + 0.37),
                # D hood footprint: y along the slope, x = chute
                (P.CHUTE_D["cx"] - 0.37, P.CHUTE_D["cx"] + 0.37,
                 wall_d - hood["into"] - 0.16, wall_d + hood["up"] + 0.16),
            ])

    for _ in range(12):                             # warm the render pipeline
        world.step(render=True)
    # NOTE: the jam-locator background is captured LATER, after the arm has
    # folded to its home pose — capturing it here bakes the asset's default
    # straight-up pose into the background, and the folded arm then reads as
    # a permanent foreground blob at its own home position.

    # ------------------------------------------------------------- run state
    rng = np.random.default_rng(args.seed)
    spawn_rng = np.random.default_rng(args.seed + 1000)
    slugs = [e["slug"] for e in manifest]
    order = [str(s) for s in rng.permutation(slugs)]
    gap_lo, gap_hi = (float(v) for v in args.spawn_gap.split(","))
    gaps = list(rng.uniform(gap_lo, gap_hi, size=len(order)))

    a, b, tb, cb = P.BELT_A, P.BELT_B, P.TABLE, P.CONNECT_B
    cd = P.CHUTE_D
    cages = P.cages_for("table")
    win0, win1 = P.VIRTUAL_SENSOR["window_x"]
    proc_lat = P.VIRTUAL_SENSOR["processing_latency_s"]

    active, done, watch = {}, {}, {}
    routes = {}
    frozen = set()                                  # injected snags (fault drill)
    inject = None
    if args.inject_jam:
        s_, x_ = args.inject_jam.split("@")
        inject = {"slug": s_, "at_x": float(x_), "done": False}
    queue = list(order)
    next_spawn_t = 1.0
    gate_open, hold2_open = True, True
    flow = {"escapement_gate_activations": 0, "spacing_gate_activations": 0,
            "multi_object_window_events": 0, "conveyor_a_stop_count": 0}
    cls_stats = {"n": 0, "correct": 0, "sensor_misses": 0, "reads_total": 0}
    reads_log = {}                      # slug -> raw reads + fused (P5 trail)
    window_multi = False
    events = []
    stills_left = args.depth_stills

    dt_phys = 1.0 / args.physics_hz
    decim = max(1, args.physics_hz // args.control_hz)
    dt_ctrl = dt_phys * decim

    def ev(t, event, slug="", **kv):
        events.append({"t": round(t, 4), "event": event, "slug": slug, **kv})

    arm = ArmController(arm_rig, items_rp, entries, ev, mode="table")
    recovery = {"active": None}
    # gate interlock layer (GATED_ACTUATOR_TEST_PLAN): explicit state machine
    # + metrics over the existing normally-closed exit gates, with optional
    # hardware fault injection
    from isaac.gates import GateInterlocks
    gate_inject = None
    if args.inject_gate_fault:
        parts = args.inject_gate_fault.split(":")
        gate_inject = {"zone": parts[0], "mode": parts[1]}
        if parts[1] == "delay":
            gate_inject["delay_s"] = float(parts[2]) / 1000.0
    gates_ctl = GateInterlocks(gate_target_attr, gate_rp, gate_z0,
                               P.GATES["travel"], ev, inject=gate_inject)
    if jam_loc is not None:
        # empty-cell depth background, with the cell in its true idle state:
        # arm folded home, gates closed, blades parked (before any spawn)
        for _ in range(6):
            world.step(render=True)
        ok_bg = jam_loc.build_background(n=5, renderer=world.render)
        print(f"[isaac] jam-locator background: {'OK' if ok_bg else 'FAILED'}",
              flush=True)
    from isaac.dressing import RouteVizRuntime
    route_viz = RouteVizRuntime(stage, info.get("viz"))
    zone_cmd = {"route": None}          # visual bookkeeping only

    def pose(slug):
        p, q = items_rp[slug].get_world_pose()
        return np.asarray(p, dtype=float), np.asarray(q, dtype=float)

    def vel(slug):
        return np.asarray(items_rp[slug].get_linear_velocity(), dtype=float)

    # Priority-2 proof: in surface mode NOTHING moves an item during nominal
    # routing except contact with a powered surface. set_v (the scripted-drive
    # idiom) is counted so the evidence can show zero nominal direct writes.
    vwrites = {"nominal_set_v": 0, "fault_injection": 0}

    def set_v(slug, vx=None, vy=None, damp_spin=True):
        vwrites["nominal_set_v"] += 1
        rp = items_rp[slug]
        v = np.asarray(rp.get_linear_velocity(), dtype=float)
        w = np.asarray(rp.get_angular_velocity(), dtype=float)
        if vx is not None:
            v[0] = vx
        if vy is not None:
            v[1] = vy
        if damp_spin:
            w = w * SPIN_DAMP
        view = getattr(rp, "_rigid_prim_view", None)
        if view is not None and hasattr(view, "set_velocities"):
            view.set_velocities(np.concatenate([v, w]).reshape(1, 6))
        else:
            rp.set_linear_velocity(v)
            rp.set_angular_velocity(w)

    def tint(slug, zone):
        # classification tint, SOFTENED: the raw saturated route colour
        # clipped to near-white under the high-bays (the "blown-out C bin"
        # was the tinted box_l itself, not the cage walls)
        mesh = stage.GetPrimAtPath(f"{info['items'][slug]['path']}/geom")
        UsdGeom.Gprim(mesh).GetDisplayColorAttr().Set(
            [Gf.Vec3f(*[0.50 * v + 0.05 for v in P.ROUTE_RGBA[zone]])])

    def which_cage(pos):
        for zone, cage in cages.items():
            cx, cy = cage["center"]
            ix, iy = cage["inner"]
            if abs(pos[0] - cx) < ix / 2 + 0.03 and abs(pos[1] - cy) < iy / 2 + 0.03:
                return zone
        return None

    def deliver(slug, zone_actual, t):
        route_viz.drop_flag(slug)
        st = active.pop(slug)
        e = entries[slug]
        ok = zone_actual == e["zone"]
        p, _ = pose(slug)
        cls = st.get("cls") or {}
        done[slug] = {"zone_true": e["zone"], "zone_routed": st.get("zone"),
                      "delivered": zone_actual, "ok": ok,
                      "t_spawn": st.get("t_spawn"), "t_detected": st.get("t_detected"),
                      "t_route_cmd": st.get("t_route_cmd"),
                      "t_table_entry": st.get("t_table_entry"),
                      "t_delivered": t,
                      "n_reads": cls.get("n_reads"),
                      "confidence": cls.get("confidence"),
                      "dims_mm": cls.get("dims_mm"),
                      "cls_reason": cls.get("reason"),
                      "final_pos": [round(float(v), 3) for v in p]}
        ev(t, "item_delivered", slug, zone=zone_actual, zone_true=e["zone"], ok=ok,
           pos=[round(float(v), 3) for v in p])
        if zone_actual == "MANUAL":
            frozen.discard(slug)
            # the operator call-out is a real action: the item is REMOVED to
            # the manual-review station. Leaving the body in the cell creates
            # ghost obstacles that block the chute and poison every later
            # jam-camera fix (helmet stalled, then sack/cylinder/plate piled
            # against it and the pile became the "jam" the arm chased)
            n_manual = sum(1 for d2 in done.values()
                           if d2["delivered"] == "MANUAL")
            items_rp[slug].set_world_pose(
                np.array([10.4, 0.6 + 0.45 * (n_manual - 1), 0.15]),
                np.array([1.0, 0.0, 0.0, 0.0]))
            items_rp[slug].set_linear_velocity(np.zeros(3))
            items_rp[slug].set_angular_velocity(np.zeros(3))
            ev(t, "operator_removed", slug, station=[10.4, 0.6])
        if zone_actual in ("C", "D"):
            v0 = float(np.linalg.norm(vel(slug)))
            watch[slug] = {"zone": zone_actual, "t_entry": t, "v_entry": v0,
                           "max_z": 0.0, "contained": True, "violations": 0,
                           "settle_t": None, "low_since": None}

    def downstream_clear():
        # CENTER-based: the yaw-agnostic front metric (center + dims[0]/2)
        # counted an item PRESSED AT ITS OWN GATE as downstream once it
        # rested rotated (box_s wedged at the blade read front 6.36 > 6.35
        # and held its own gate closed forever). An item is downstream only
        # when its center has committed past the gate line.
        for slug in active:
            p, _ = pose(slug)
            if p[0] > a["gate_x"] + 0.05 and abs(p[1] - a["y"]) < 0.65:
                return False
        return True

    def window_clear():
        # CENTER-based, same lesson as downstream_clear: the yaw-agnostic
        # front metric (center + dims[0]/2) counted the LONG cylinder pressed
        # AT the hold2 blade (center 5.49, metric-front 5.70) as "in the
        # window" — its own presence kept its own hold blade raised forever
        # and the whole upstream queue deadlocked behind it (probe42).
        for slug in active:
            p, _ = pose(slug)
            if (a["hold2_x"] + 0.05 < p[0] <= a["gate_x"] + 0.05
                    and abs(p[1] - a["y"]) < 0.4):
                return False
        return True

    def queue_lines():
        """Zone-accumulation stop lines while the pre-gate hold is closed."""
        on_a = []
        for slug in active:
            p, _ = pose(slug)
            if abs(p[1] - a["y"]) > 0.4:
                continue
            half = entries[slug]["dims_m"][0] / 2
            front = p[0] + half
            if front - 2 * half > a["gate_x"] + 0.05:
                continue
            on_a.append((front, slug, half))
        on_a.sort(key=lambda q: -q[0])
        if on_a and on_a[0][0] > a["hold2_x"] - 0.004:
            on_a.pop(0)                            # corridor owner is exempt
        lines, line = {}, a["hold2_x"]
        for front, slug, half in on_a:
            lines[slug] = line
            line -= 2 * half + QUEUE_GAP
        return lines

    def drive_belts(t):
        """Port of cell/belt.py step() + cell/table.py step() on proximity."""
        holds = queue_lines() if not hold2_open else {}
        for slug in list(active.keys()):
            if slug in frozen:                      # injected snag: no drive
                continue
            p, _ = pose(slug)
            x, y, z = p
            dims = entries[slug]["dims_m"]
            half = dims[0] / 2
            hm = max(dims[0], dims[1]) / 2      # yaw-agnostic footprint half
            if not (Z_ON[0] < z < Z_ON[1]):
                continue
            # --- belt A
            if 0.0 <= x < tb["x0"] and abs(y - a["y"]) < 0.35:
                front = x + half
                if not gate_open and a["gate_x"] - 0.004 <= front < a["gate_x"] + 0.05:
                    set_v(slug, 0.0, 0.8 * (a["y"] - y))
                    continue
                hold_at = holds.get(slug)
                if hold_at is not None and front >= hold_at - 0.004:
                    set_v(slug, 0.0, 0.8 * (a["y"] - y))
                    continue
                taper = 1.0
                if not gate_open and front < a["gate_x"] + 0.05:
                    taper = min(taper, float(np.clip((a["gate_x"] - front) / 0.30, 0.0, 1.0)))
                if hold_at is not None and front < hold_at:
                    taper = min(taper, float(np.clip((hold_at - front) / 0.30, 0.0, 1.0)))
                set_v(slug, a["speed"] * taper, 0.8 * (a["y"] - y))
            # --- transfer table: MuJoCo drives ONLY items in contact with the
            # table surface, so a C/D item that has tipped onto its chute is no
            # longer driven — gravity + the 32 deg slope (mu < tan32) carry it
            # down. We mirror that with an EDGE-RELEASE handoff: drive to the
            # table edge, then stop touching the item and let PhysX take over
            # (driving past the edge rams a flat box into the chute hood mouth).
            elif (tb["x0"] <= x <= tb["x1"] + 0.02
                  and tb["y"] - tb["width"] / 2 - 0.02 <= y
                  <= tb["y"] + tb["width"] / 2 + 0.02):
                route = routes.get(slug)
                if x < tb["route_x"]:
                    set_v(slug, tb["speed"], 0.8 * (tb["y"] - y))
                elif route == "B":
                    # centre on the lane axis FIRST (an omni-roller field can
                    # do this), then head north through the gate gap. A fixed
                    # "aligned" band fails wide items: helmet's pass window was
                    # 8 mm and one 60 Hz tick jumps 13 mm — it drove east into
                    # the closed C gate instead (run-2 jam, cascaded to the
                    # follower)
                    g0, g1 = P.GATES["B"]["c0"], P.GATES["B"]["c1"]
                    margin = max(0.015, (g1 - g0) / 2 - hm - 0.005)
                    if abs(x - tb["lane_B_cx"]) > margin:
                        set_v(slug, 2.5 * (tb["lane_B_cx"] - x), 0.8 * (tb["y"] - y))
                    else:
                        set_v(slug, 1.5 * (tb["lane_B_cx"] - x), tb["speed"])
                elif route == "C":
                    # feed until the TAIL clears the crest (a mid-tip box is a
                    # static wedge otherwise: nose on slope + tail on crest);
                    # spin-damp only while fully on the table, so the gravity
                    # pitch about the crest develops WHILE still driven and
                    # the nose ducks under the (raised) hood mouth
                    if x - hm < tb["x1"]:
                        set_v(slug, tb["speed"], 0.8 * (tb["lane_C_cy"] - y),
                              damp_spin=(x + hm < tb["x1"]))
                elif route == "D":
                    y_edge = tb["y"] - tb["width"] / 2
                    if y + hm > y_edge:
                        set_v(slug, 0.8 * (tb["lane_D_cx"] - x), -tb["speed"],
                              damp_spin=(y - hm > y_edge))
                else:
                    vx = 0.0 if x > tb["route_x"] - 0.05 else tb["speed"]
                    set_v(slug, vx, 0.8 * (tb["y"] - y))
            # --- powered connector to belt B (window mirrors CONTACT semantics:
            # an item touching the 0.5 m surface can have its center out to
            # half-width + its own half extent — the crail rails bound it)
            elif (abs(x - cb["cx"]) <= cb["width"] / 2 + 0.22
                  and tb["y"] + tb["width"] / 2 < y <= cb["y1"]):
                set_v(slug, 0.8 * (cb["cx"] - x), cb["speed"])
            # --- belt B
            elif abs(x - b["cx"]) <= b["width"] / 2 + 0.22 and cb["y1"] < y <= b["y1"]:
                set_v(slug, 0.8 * (b["cx"] - x), b["speed"])

    BLADE_UP = 0.186                # raised crest ~0.14 above belt: stops every
                                # item (helmet R=0.14 cannot roll over) without
                                # the launched-board look
    blade_x = {"egate": P.BELT_A["gate_x"] + 0.02,
               "hold2": P.BELT_A["hold2_x"] + 0.02,
               "zoneq1": P.BELT_A["hold2_x"] - 0.60,
               "zoneq2": P.BELT_A["hold2_x"] - 1.20,
               "induct": P.TABLE["route_x"] - 0.02}
    blade_up_state = {k: False for k in blade_x}

    def blade_clear(bx):
        """Raise-safety interlock: never pop a blade under an item's belly —
        it levers the item over the side guides (helmet flipped off belt A)."""
        for s2 in active:
            p2, _ = pose(s2)
            if abs(p2[1] - P.BELT_A["y"]) > 0.45:
                continue
            hm2 = max(entries[s2]["dims_m"][0], entries[s2]["dims_m"][1]) / 2
            if abs(p2[0] - bx) < hm2 + 0.05:
                return False
        return True

    def set_blade(name, want):
        if want and not blade_up_state[name] and not blade_clear(blade_x[name]):
            want = False                            # defer: item over the slot
        blade_up_state[name] = want
        blade_attr[name].Set(BLADE_UP if want else 0.0)

    def drive_surface(t):
        """Conveyor-utility executive: constant surface-velocity belts, pop-up
        stop blades for flow discipline, and ONE commanded vector on the ARB
        routing zone aimed at the active item's exit. No per-item velocity
        writes — items are carried, held and routed by contact physics."""
        a2 = P.BELT_A
        # flow blades (physical escapement / pre-gate hold)
        set_blade("egate", not gate_open)
        set_blade("hold2", not hold2_open)
        # zone accumulation: raise a blade while the zone ahead of it is busy
        occ1 = occ2 = False
        for s2 in active:
            p2, _ = pose(s2)
            if abs(p2[1] - a2["y"]) > 0.4:
                continue
            if a2["hold2_x"] - 0.55 < p2[0] <= a2["hold2_x"] + 0.02:
                occ1 = True
            if a2["hold2_x"] - 1.15 < p2[0] <= a2["hold2_x"] - 0.58:
                occ2 = True
        set_blade("zoneq1", occ1)
        set_blade("zoneq2", occ2)
        # routing-zone ownership: earliest-routed item currently on the zone.
        # The induction blade rises only once the owner is WELL past the blade
        # line — raising it at the line pins the owner against its own blade
        # (plate deadlocked by exactly that chatter)
        owner, owner_t, owner_x = None, np.inf, 0.0
        ty0, ty1 = tb["y"] - tb["width"] / 2, tb["y"] + tb["width"] / 2
        for s2, st2 in active.items():
            if "routed_t" not in st2 or s2 in frozen or st2.get("recovering"):
                continue
            p2, _ = pose(s2)
            if (tb["route_x"] + 0.02 < p2[0] < tb["x1"] + 0.12
                    and ty0 - 0.06 < p2[1] < ty1 + 0.06
                    and st2["routed_t"] < owner_t):
                owner, owner_t, owner_x = s2, st2["routed_t"], float(p2[0])
        set_blade("induct",
                  owner is not None and owner_x > tb["route_x"] + 0.18)
        zone_cmd["route"] = routes.get(owner) if owner is not None else None
        if owner is not None:
            po, _ = pose(owner)
            route = routes.get(owner)
            exits = {"B": (tb["lane_B_cx"], ty1 + 0.25),
                     "C": (tb["x1"] + 0.30, tb["lane_C_cy"]),
                     "D": (tb["lane_D_cx"], ty0 - 0.30)}
            txy = exits.get(route, (tb["x1"] + 0.3, tb["y"]))
            hm_o = max(entries[owner]["dims_m"][0],
                       entries[owner]["dims_m"][1]) / 2
            # command the LOCAL patches the owner covers (plus the pre-spin
            # halo); the commanded vector tracks the item toward its exit
            deck.command(t, (float(po[0]), float(po[1])), hm_o, route, txy)
        else:
            # idle deck FEEDS FORWARD: every item reaching the zone is
            # already routed, and a dead strip under an item straddling the
            # ownership line parks it (bottle stalled at exactly route_x)
            deck.command(t)
        deck.step(t, dt_ctrl)
        # injected snag: the fault itself pins the item
        for s2 in frozen:
            if s2 in active:
                items_rp[s2].set_linear_velocity(np.zeros(3))
                items_rp[s2].set_angular_velocity(np.zeros(3))
                vwrites["fault_injection"] += 1

    def drive_gates(t):
        want = {z: False for z in "BCD"}
        for slug, route in routes.items():
            if slug not in active or route not in want:
                continue
            p, _ = pose(slug)
            ax, sgn, thr = GATE_CLEAR[route]
            v = p[0] if ax == "x" else p[1]
            if sgn * (v - thr) < 0:
                want[route] = True
        gates_ctl.command(t, want)
        gates_ctl.step(t)

    def capture_still(t, slug):
        nonlocal stills_left
        if look_cam is None or stills_left <= 0:
            return
        stills_left -= 1
        try:
            from PIL import Image
            world.render()                          # flush the annotators
            world.render()
            frame = look_cam.get_current_frame()
            rgba = look_cam.get_rgba()
            if rgba is not None and getattr(rgba, "size", 0):
                Image.fromarray(np.asarray(rgba)[..., :3]).save(
                    out_dir / f"vision_rgb_{slug}.png")
            depth = frame.get("distance_to_image_plane")
            if depth is not None and getattr(depth, "size", 0):
                d = np.asarray(depth, dtype=float)
                np.save(out_dir / f"vision_depth_{slug}.npy", d.astype(np.float32))
                finite = np.isfinite(d)
                if finite.any():
                    lo, hi = np.percentile(d[finite], [2, 98])
                    img = np.clip((d - lo) / max(hi - lo, 1e-6), 0, 1)
                    img[~finite] = 1.0
                    Image.fromarray((255 * (1 - img)).astype(np.uint8)).save(
                        out_dir / f"vision_depth_{slug}.png")
            ev(t, "vision_capture", slug)
        except Exception as exc:                    # never kill the run for a still
            ev(t, "vision_capture_failed", slug, err=str(exc)[:120])

    # ------------------------------------------------------------- main loop
    n_total = len(order)
    t = 0.0
    step_i = 0
    frame_i = 0
    next_frame_t = 0.0
    t_wall0 = time.time()
    print(f"[isaac] seed={args.seed} items={n_total} physics={args.physics_hz}Hz "
          f"control={args.control_hz}Hz record={args.record} -> {out_dir}",
          flush=True)

    while len(done) < n_total and t < args.max_sim_s:
        if step_i % decim == 0:
            # ---- spawn
            if queue and t >= next_spawn_t:
                clear = all(not (pose(s)[0][0] < 1.2 and abs(pose(s)[0][1] - a["y"]) < 0.4)
                            for s in active)
                if clear:
                    slug = queue.pop(0)
                    e = entries[slug]
                    diag = float(np.hypot(e["dims_m"][0], e["dims_m"][1]))
                    yaw = (float(spawn_rng.uniform(-0.09, 0.09)) if diag > 0.48
                           else float(spawn_rng.uniform(0, 2 * np.pi)))
                    qz = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
                    items_rp[slug].set_world_pose(
                        np.array([0.4, a["y"] + args.spawn_offset_y,
                                  a["top"] + e["dims_m"][2] / 2 + 0.003]),
                        np.array(qz))
                    items_rp[slug].set_linear_velocity(np.zeros(3))
                    items_rp[slug].set_angular_velocity(np.zeros(3))
                    active[slug] = {"t_spawn": t}
                    ev(t, "item_spawned", slug, zone_true=e["zone"])
                    next_spawn_t = t + (gaps.pop(0) if gaps else 6.0)

            # ---- per-item logic
            in_window = 0
            for slug in list(active.keys()):
                st = active[slug]
                if st.get("recovering"):            # carried by the arm
                    continue
                p, _ = pose(slug)
                if win0 <= p[0] <= win1:
                    in_window += 1
                if "t_detected" not in st and p[0] >= win0:
                    st["t_detected"] = t
                    ev(t, "item_detected", slug)
                    capture_still(t, slug)
                # multi-read sensing while this item owns the corridor (the
                # pre-gate hold keeps followers upstream — single item in
                # window). Each read renders the 3 RTX heads fresh.
                if (perc is not None and "zone" not in st and "t_detected" in st
                        and p[0] <= win1 and t >= st.get("next_read_t", 0.0)
                        and len(st.get("reads", ())) < P.CLASSIFICATION["read_cap"]):
                    others_ahead = [s2 for s2 in active
                                    if s2 != slug and "zone" not in active[s2]
                                    and pose(s2)[0][0] > p[0]
                                    and pose(s2)[0][0] <= win1 + 0.3]
                    pressed = (blade_up_state.get("egate")
                               and p[0] + entries[slug]["dims_m"][0] / 2
                               > P.BELT_A["gate_x"] - 0.06)
                    if not others_ahead and not pressed:
                        st["next_read_t"] = t + 0.08
                        # annotator freshness: stale frames smear a moving
                        # item; a cold pipeline (no --record renders between
                        # reads) needs more flushes per read
                        for _ in range(4 if args.record else 7):
                            world.render()
                        excl = [(bx - 0.05, bx + 0.05)
                                for nm, bx in blade_x.items()
                                if blade_up_state.get(nm)] if blade_x else []
                        r = perc.measure(exclude_x=excl, x_hint=float(p[0]))
                        st.setdefault("reads", []).append(r)
                # look-ahead classification: commit at window exit + latency
                if "zone" not in st and "t_detected" in st and p[0] > win1:
                    st.setdefault("t_verdict", t + proc_lat)
                if "zone" not in st and "t_verdict" in st and t >= st["t_verdict"]:
                    e = entries[slug]
                    if perc is not None:
                        fused = fuse_reads(st.get("reads", []))
                        st["zone"] = fused["zone"]
                        st["cls"] = fused
                        # P5 evidence trail: every RAW measurement behind the
                        # fused verdict is persisted (reads_log.json)
                        reads_log[slug] = {"raw_measurements": st.get("reads", []),
                                           "fused": fused}
                        correct = fused["zone"] == e["zone"]
                        cls_stats["n"] += 1
                        cls_stats["correct"] += int(correct)
                        cls_stats["sensor_misses"] += int(fused["sensor_miss"])
                        cls_stats["reads_total"] += fused["n_reads"]
                        ev(t, "item_classified", slug, zone=fused["zone"],
                           zone_true=e["zone"], correct=correct,
                           n_reads=fused["n_reads"],
                           confidence=fused.get("confidence"),
                           sensor_miss=fused["sensor_miss"],
                           dims_mm=fused.get("dims_mm"),
                           reason=fused["reason"],
                           latency_ms=round((t - st["t_detected"]) * 1000, 1),
                           mode="rtx_depth_3head")
                    else:
                        st["zone"] = e["zone"]
                        ev(t, "item_classified", slug, zone=e["zone"],
                           zone_true=e["zone"], correct=True,
                           latency_ms=round((t - st["t_detected"]) * 1000, 1),
                           mode="oracle_lookahead")
                    st["t_route_cmd"] = t
                    tint(slug, st["zone"])
                    route_viz.make_flag(slug, st["zone"])
                # route assignment as the item commits to the table
                if "zone" in st and "routed_t" not in st and p[0] > tb["x0"] - 0.25:
                    routes[slug] = st["zone"]
                    st["routed_t"] = t
                    st["watch_xy"] = (float(p[0]), float(p[1]))
                    st["watch_t"] = t
                    ev(t, "routing_cmd", slug, zone=st["zone"])
                if "t_table_entry" not in st and p[0] > tb["x0"] and p[2] > 0.5:
                    st["t_table_entry"] = t
                    margin = (t - st["t_route_cmd"]) if "t_route_cmd" in st else None
                    ev(t, "table_entry", slug,
                       command_margin_s=round(margin, 3) if margin is not None else "")
                # gate-interlock verification (GATED_ACTUATOR_TEST_PLAN):
                # exit crossing is confirmed against the MEASURED gate state,
                # and an item parked against a not-open gate is reported
                route_g = routes.get(slug)
                if route_g in GATE_CLEAR and slug not in frozen:
                    ax_g, sgn_g, thr_g = GATE_CLEAR[route_g]
                    coord = p[0] if ax_g == "x" else p[1]
                    if not st.get("exit_verified") and sgn_g * (coord - thr_g) >= 0:
                        st["exit_verified"] = True
                        gstate = gates_ctl.state.get(route_g)
                        ev(t, "exit_verified", slug, gate=route_g,
                           gate_state=gstate)
                        if gstate != "open":
                            gates_ctl.metrics["gate_item_contact_events"] += 1
                    line_g = P.GATES[route_g]["line"]
                    if (not st.get("gate_block_reported")
                            and gates_ctl.state.get(route_g) != "open"
                            and sgn_g * (coord - line_g) < 0
                            and abs(coord - line_g) < 0.12
                            and float(np.linalg.norm(vel(slug)[:2])) < 0.05):
                        st["gate_block_reported"] = True
                        gates_ctl.metrics["gate_item_contact_events"] += 1
                        ev(t, "wrong_gate_blocked", slug, gate=route_g,
                           gate_state=gates_ctl.state.get(route_g))
                # deliveries
                if p[1] > b["y_delivered"] and abs(p[0] - b["cx"]) < 0.3:
                    deliver(slug, "B", t)
                    continue
                zone_c = which_cage(p)
                if zone_c is not None and p[2] < 0.56:
                    deliver(slug, zone_c, t)
                    continue
                if (p[2] < 0.25 and p[1] > 0 and zone_c is None
                        and not (abs(p[0] - 0.4) < 0.3 and p[1] < 0)):
                    deliver(slug, "FLOOR", t)
                    continue
                # jam watchdog (operator call-out in this port)
                if "routed_t" in st and not st.get("jam_reported"):
                    # commanded ACCUMULATION is flow, not a fault: while the
                    # induction blade holds the queue, items on the entry
                    # strip / at the blade are exactly where the cell wants
                    # them (bottle/plate/box_s were MANUAL'd for waiting)
                    if (args.drive == "surface" and blade_up_state.get("induct")
                            and p[0] < tb["route_x"] + 0.15):
                        st["watch_xy"] = (float(p[0]), float(p[1]))
                        st["watch_t"] = t
                    if t - st["watch_t"] >= P.JAM_TIMEOUT_S:
                        moved = float(np.hypot(p[0] - st["watch_xy"][0],
                                               p[1] - st["watch_xy"][1]))
                        st["watch_xy"] = (float(p[0]), float(p[1]))
                        st["watch_t"] = t
                        if moved < P.JAM_MIN_PROGRESS_M:
                            st["jam_reported"] = True
                            ev(t, "jam_detected", slug, zone=st.get("zone", ""),
                               pos=[round(float(v), 3) for v in p])
                            loc = None
                            if jam_loc is not None:
                                # locate the stuck item with the REAL camera
                                for _ in range(4):
                                    world.render()
                                loc = jam_loc.locate(route=st.get("zone"))
                                if loc is not None:
                                    err = float(np.hypot(
                                        loc["pos_xyz"][0] - p[0],
                                        loc["pos_xyz"][1] - p[1]))
                                    # ODOMETRY SANITY GATE: the cell tracks
                                    # every item, so a camera fix far from
                                    # the last tracked position is a bad
                                    # localization (occlusion artifact) —
                                    # never dispatch the arm on it
                                    if err > 0.30:
                                        ev(t, "jam_locate_rejected", slug,
                                           cam_pos=loc["pos_xyz"],
                                           track_pos=[round(float(v), 3)
                                                      for v in p],
                                           err_mm=round(err * 1000, 1))
                                        loc = None
                                    else:
                                        ev(t, "jam_located", slug,
                                           cam_pos=loc["pos_xyz"],
                                           half_extents=loc["half_extents"],
                                           n_points=loc["n_points"],
                                           true_pos=[round(float(v), 3)
                                                     for v in p],
                                           err_mm=round(err * 1000, 1))
                                else:
                                    ev(t, "jam_locate_failed", slug)
                            if loc is None:
                                # State-observer fallback: the camera is the
                                # preferred independent jam sensor, but a bad
                                # foreground cluster must not force operator
                                # handling when the tracked rigid-body state is
                                # still coherent. Use the tracker estimate as
                                # a bounded fallback for the exception arm.
                                dims = entries[slug]["dims_m"]
                                half_ext = [float(dims[0]) / 2,
                                            float(dims[1]) / 2,
                                            float(dims[2]) / 2]
                                loc = {"pos_xyz": [float(p[0]), float(p[1]),
                                                   float(p[2])],
                                       "half_extents": half_ext,
                                       "n_points": 0,
                                       "source": "state_observer"}
                                ev(t, "jam_located_state_observer", slug,
                                   track_pos=[round(float(v), 3) for v in p],
                                   half_extents=[round(v, 3) for v in half_ext])
                            # dispatch the exception arm on the camera/state
                            # estimate; escalate to operator call-out when the
                            # arm is busy, the estimate failed, or recovery was
                            # already tried too many times
                            if (arm is not None and loc is not None
                                    and not arm.busy
                                    and recovery["active"] is None
                                    and st.get("recoveries", 0) < 2):
                                pick = loc["pos_xyz"]
                                top_z = pick[2] + loc["half_extents"][2]
                                if arm.start_recovery(slug, st.get("zone", "D"),
                                                      pick, top_z, t):
                                    st["recovering"] = True
                                    recovery["active"] = slug
                                    # the snag stays FROZEN until the arm has
                                    # it: unfreezing at dispatch lets the belt
                                    # re-drive the item away mid-descent and
                                    # the grasp check fails on a moved target
                                    continue
                            deliver(slug, "MANUAL", t)
                            continue
            if in_window >= 2 and not window_multi:
                flow["multi_object_window_events"] += 1
            window_multi = in_window >= 2

            # ---- fault injection: snag on the table (jam drill)
            if inject and not inject["done"] and inject["slug"] in active:
                px_ = pose(inject["slug"])[0][0]
                if px_ > inject["at_x"]:
                    inject["done"] = True
                    frozen.add(inject["slug"])
                    ev(t, "snag_injected", inject["slug"],
                       at_x=round(float(px_), 3))

            # ---- flow-discipline gates
            was_gate, was_hold2 = gate_open, hold2_open
            gate_open = downstream_clear()
            hold2_open = window_clear()
            if was_gate and not gate_open:
                flow["escapement_gate_activations"] += 1
            if was_hold2 and not hold2_open:
                flow["spacing_gate_activations"] += 1

            if args.drive == "surface":
                drive_surface(t)
            else:
                drive_belts(t)
            drive_gates(t)
            # route storytelling (visuals only): lamps/arrows/trails follow
            # the commanded route; each classified item carries its flag
            flag_pos = {}
            for s2, st2 in active.items():
                if "zone" in st2:
                    p2, _ = pose(s2)
                    top2 = p2[2] + max(entries[s2]["dims_m"]) / 2
                    flag_pos[s2] = (p2[0], p2[1], top2)
            route_viz.update(zone_cmd["route"], flag_pos)
            if arm is not None:
                arm.step(dt_ctrl, t)
                if recovery["active"] and not arm.busy:
                    slug_r = recovery["active"]
                    recovery["active"] = None
                    st_r = active.get(slug_r)
                    if st_r is not None:
                        # re-arm the watchdog: route stays assigned, the zone
                        # conveyor re-delivers through the normal guided path.
                        # The snag clears NOW (arm done), not at dispatch
                        frozen.discard(slug_r)
                        st_r["recovering"] = False
                        st_r["recoveries"] = st_r.get("recoveries", 0) + 1
                        st_r["jam_reported"] = False
                        st_r["routed_t"] = t
                        pr, _ = pose(slug_r)
                        st_r["watch_xy"] = (float(pr[0]), float(pr[1]))
                        st_r["watch_t"] = t
                        ev(t, "recovery_done", slug_r,
                           attempts=st_r["recoveries"])

            # ---- probe trace (diagnostics). --probe all = every active item
            # everywhere at 1 Hz plus the flow-discipline state
            if args.probe == "all" and step_i % args.physics_hz == 0:
                for pslug in list(active.keys()):
                    ps, qq = pose(pslug)
                    vv = vel(pslug)
                    print(f"[probe] {pslug} t={t:7.3f} "
                          f"pos=({ps[0]:.3f},{ps[1]:.3f},{ps[2]:.3f}) "
                          f"v=({vv[0]:+.3f},{vv[1]:+.3f},{vv[2]:+.3f}) "
                          f"q=({qq[0]:+.3f},{qq[1]:+.3f},{qq[2]:+.3f},{qq[3]:+.3f}) "
                          f"route={routes.get(pslug)}", flush=True)
                print(f"[probe] flow t={t:7.3f} gate={gate_open} "
                      f"hold2={hold2_open} blades="
                      f"{ {k: int(v) for k, v in blade_up_state.items()} }",
                      flush=True)
            elif args.probe and step_i % (4 * decim) == 0:
                for pslug in args.probe.split(","):
                    if pslug not in active:
                        continue
                    ps, _ = pose(pslug)
                    if ps[0] > 7.8:
                        vv = vel(pslug)
                        print(f"[probe] {pslug} t={t:7.3f} "
                              f"pos=({ps[0]:.3f},{ps[1]:.3f},{ps[2]:.3f}) "
                              f"v=({vv[0]:+.3f},{vv[1]:+.3f},{vv[2]:+.3f}) "
                              f"route={routes.get(pslug)}", flush=True)

            # ---- containment watch (to the very end of the run)
            for slug, w in watch.items():
                p, _ = pose(slug)
                cage = cages[w["zone"]]
                cx, cy = cage["center"]
                hx = cage["inner"][0] / 2 + cage["wall_t"] + P.CONTAIN["margin"]
                hy = cage["inner"][1] / 2 + cage["wall_t"] + P.CONTAIN["margin"]
                w["max_z"] = max(w["max_z"], float(p[2]))
                out = (abs(p[0] - cx) > hx or abs(p[1] - cy) > hy
                       or p[2] > P.CONTAIN["z_fly"])
                if out and w["contained"]:
                    w["contained"] = False
                    w["violations"] += 1
                    ev(t, "containment_violation", slug,
                       pos=[round(float(v), 3) for v in p])
                if w["settle_t"] is None:
                    speed = float(np.linalg.norm(vel(slug)))
                    if speed < P.CONTAIN["settle_speed"]:
                        w.setdefault("low_since", t)
                        if w["low_since"] is not None and t - w["low_since"] >= P.CONTAIN["settle_time"]:
                            w["settle_t"] = t - w["t_entry"]
                    else:
                        w["low_since"] = None

        # ---- physics step (+ render only when a video frame is due)
        render = args.record and t >= next_frame_t
        if render and cine is not None:
            cine.update(t / max(args.path_secs, 1e-3))
        if render and follow is not None:
            fp = None
            act = False
            if args.follow_slug in items_rp:
                fp = np.asarray(items_rp[args.follow_slug].get_world_pose()[0],
                                dtype=float)
                # "on the line": past the vision approach, not parked behind
                # the wall (x>11) and not dropped to the floor (z<0.35)
                act = 5.0 < fp[0] < 10.3 and fp[2] > 0.35
            follow.update(fp, act)
        world.step(render=False)
        if render:
            world.render()
            rgba = view_cam.get_rgba()
            if rgba is not None and getattr(rgba, "size", 0):
                from PIL import Image
                Image.fromarray(np.asarray(rgba)[..., :3]).save(
                    out_dir / "frames" / f"{frame_i:05d}.png")
                frame_i += 1
            next_frame_t += 1.0 / args.fps
        t += dt_phys
        step_i += 1
        if step_i % (args.physics_hz * 20) == 0:
            print(f"[isaac] t={t:7.1f}s delivered={len(done)}/{n_total} "
                  f"wall={time.time() - t_wall0:6.1f}s", flush=True)

    wall_s = time.time() - t_wall0

    # -------------------------------------------------------------- outputs
    for slug, w in watch.items():
        if slug in done:
            done[slug]["contained"] = w["contained"]
            done[slug]["v_entry"] = round(w["v_entry"], 3)
            done[slug]["cage_max_z"] = round(w["max_z"], 3)
            done[slug]["cage_settle_s"] = (round(w["settle_t"], 2)
                                           if w["settle_t"] is not None else None)

    rows = []
    for slug, d in done.items():
        cyc = (d["t_delivered"] - d["t_detected"]) if d.get("t_detected") else None
        margin = None
        if d.get("t_table_entry") and d.get("t_route_cmd"):
            margin = d["t_table_entry"] - d["t_route_cmd"]
        rows.append({"slug": slug, **{k: (round(v, 3) if isinstance(v, float) else v)
                                      for k, v in d.items()},
                     "cycle_s": round(cyc, 3) if cyc else None,
                     "command_margin_s": round(margin, 3) if margin else None})

    n_ok = sum(1 for r in rows if r["ok"])
    unsafe = sum(1 for r in rows if r["delivered"] == "B"
                 and r["zone_true"] in ("C", "D"))
    tracked = [w for w in watch.values()]
    cycles = [r["cycle_s"] for r in rows if r["cycle_s"]]
    t_del = [r["t_delivered"] for r in rows if r.get("t_delivered")]
    summary = {
        "engine": "NVIDIA Isaac Sim 6.0.1 / PhysX 5",
        "perception_mode": ("rtx_depth_3head" if perc is not None
                            else "oracle_lookahead"),
        "oracle_used_for_classification": perc is None,
        "sensor_model": ("3x RTX depth cameras (overhead 1024x768 + 2 side "
                         "profiler heads 768x576), multi-read fusion, "
                         "guard-banded official rule order"
                         if perc is not None else "ground truth + latency"),
        "classification": {
            "n": cls_stats["n"],
            "n_correct": cls_stats["correct"],
            "accuracy": (round(cls_stats["correct"] / cls_stats["n"], 4)
                         if cls_stats["n"] else None),
            "sensor_misses": cls_stats["sensor_misses"],
            "reads_per_item": (round(cls_stats["reads_total"]
                                     / max(1, cls_stats["n"]), 1)),
        } if perc is not None else None,
        "executive": "table",
        # Priority-2 proof: nominal movement is simulated, never animated
        "nominal_motion_model": ("surface_contact_only"
                                 if args.drive == "surface"
                                 else "scripted_velocity_writes"),
        "direct_velocity_writes_nominal": vwrites["nominal_set_v"],
        "direct_velocity_writes_fault_injection": vwrites["fault_injection"],
        "arb_deck": deck.summary() if deck is not None else None,
        "gate_interlocks": gates_ctl.summary(),
        "materials": {slug: info["items"][slug].get("material")
                      for slug in entries},
        "sweep": {"friction_mult": args.friction_mult,
                  "mass_mult": args.mass_mult,
                  "spawn_offset_y_m": args.spawn_offset_y,
                  "spawn_gap_s": args.spawn_gap,
                  "inject_jam": args.inject_jam},
        "seed": args.seed,
        "physics_hz": args.physics_hz, "control_hz": args.control_hz,
        "n_items": n_total, "n_delivered": len(done),
        "n_routed_ok": n_ok,
        "routing_accuracy": round(n_ok / max(1, n_total), 4),
        "unsafe_errors": unsafe,
        "containment": {
            "tracked": len(tracked),
            "violations": int(sum(w["violations"] for w in tracked)),
            "containment_rate": (round(sum(1 for w in tracked if w["contained"])
                                       / len(tracked), 4) if tracked else None),
            "cage_entry_speed_max_mps": (round(max(w["v_entry"] for w in tracked), 3)
                                         if tracked else None),
            "cage_max_z": (round(max(w["max_z"] for w in tracked), 3)
                           if tracked else None),
        },
        "cycle_s": {"mean": round(float(np.mean(cycles)), 3) if cycles else None,
                    "p95": round(float(np.percentile(cycles, 95)), 3) if cycles else None,
                    "max": round(float(np.max(cycles)), 3) if cycles else None},
        "throughput_items_per_h": (round(3600.0 * len(done) / max(t_del), 1)
                                   if t_del else None),
        "flow": flow,
        "sim_s": round(t, 2), "wall_s": round(wall_s, 1),
        "realtime_factor": round(t / max(wall_s, 1e-6), 2),
        "frames": frame_i,
        "items": rows,
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2),
                                          encoding="utf-8")
    if deck is not None:
        deck.write_log(out_dir / "actuator_log.csv")
    if reads_log:
        (out_dir / "reads_log.json").write_text(
            json.dumps(reads_log, indent=1), encoding="utf-8")
    keys = ["t", "event", "slug"]
    with (out_dir / "events.csv").open("w", newline="", encoding="utf-8") as f:
        wtr = csv.writer(f)
        wtr.writerow(keys + ["data"])
        for e in events:
            extra = {k: v for k, v in e.items() if k not in keys}
            wtr.writerow([e.get(k, "") for k in keys]
                         + [json.dumps(extra, ensure_ascii=False) if extra else ""])
    print(f"[isaac] DONE delivered={len(done)}/{n_total} ok={n_ok} "
          f"unsafe={unsafe} sim={t:.1f}s wall={wall_s:.1f}s frames={frame_i}",
          flush=True)
    print(json.dumps({k: v for k, v in summary.items() if k != "items"},
                     indent=2), flush=True)

    sim_app.close()
    return 0 if (len(done) == n_total and unsafe == 0) else 1


if __name__ == "__main__":
    raise SystemExit(main())
