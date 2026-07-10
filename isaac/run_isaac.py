# -*- coding: utf-8 -*-
"""SortMaster cell in NVIDIA Isaac Sim (PhysX) — closed loop on the tilt-tray
sorter executive.

Flow (all freight motion is surface/gravity contact physics):
  * belt A (1 m/s, FIXED) carries items through the RTX vision station; the
    pre-gate hold keeps a single item in the measurement window; multi-read
    fusion + guard-banded official rule order produce the B/C/D verdict
    BEFORE the item reaches the escapement;
  * the escapement releases the item synchronized to an inbound EMPTY tray;
    the item rides off the belt-A knife nose and lands on the moving tray
    (landing offset measured per item);
  * the carrier train (real prismatic velocity drives; revolute + angular
    drive tilt trays) carries each item to its route's discharge station:
    C/D tilt south onto 32-deg brake chutes into the roll-cages, B tilts
    north onto a powered incline connector feeding the FIXED belt B, REVIEW
    tilts north into the manual-review pen (low confidence, double
    occupancy, discharge-miss fallback);
  * jam watchdog: zero displacement over the timeout window -> jam camera
    fix -> exception arm recovers chute snags route-correct into their own
    cage; anything it cannot reach safely is an operator call-out. A stuck
    tray needs no arm: its freight rides to the end-line call-out.

Every actuator command is logged; direct_velocity_writes_nominal must be 0.

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
Z_ON = (0.55, 1.05)          # item center z while riding belt (0.7) or tray


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
                    help="cinematic moving camera for the defense reel")
    ap.add_argument("--path-secs", type=float, default=80.0)
    ap.add_argument("--follow-slug", default=None,
                    help="item-follow cinematic camera: trail this item from "
                         "the vision station along the sorter into its bin")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--perception", choices=["rtx", "oracle"], default="rtx")
    ap.add_argument("--depth-stills", type=int, default=3)
    ap.add_argument("--spawn-gap", default="6.0,8.0")
    ap.add_argument("--max-sim-s", type=float, default=400.0)
    ap.add_argument("--physics-hz", type=int, default=240)
    ap.add_argument("--control-hz", type=int, default=60)
    ap.add_argument("--items", default=None,
                    help="comma list of slugs (default: all manifest items)")
    ap.add_argument("--manifest-extra", action="store_true",
                    help="merge manifest_extra.json + manifest_edge.json "
                         "(borderline + edge-case synthetic items)")
    ap.add_argument("--drive", choices=["surface", "scripted"],
                    default="surface",
                    help="surface = contact physics only (default); "
                         "scripted = legacy per-item velocity writes (debug)")
    ap.add_argument("--inject-jam", default=None, metavar="SLUG@Y",
                    help="fault drill: pin SLUG once it descends past y=Y on "
                         "its chute (snag) so the watchdog + jam camera + "
                         "arm recovery fire, e.g. box_l@2.6")
    ap.add_argument("--inject-tray-fault", default=None, metavar="ZONE",
                    help="fault drill: the first carrier assigned a ZONE item "
                         "has a dead tilt actuator -> discharge misses -> "
                         "review fallback also dead -> end-line call-out")
    ap.add_argument("--friction-mult", type=float, default=1.0)
    ap.add_argument("--mass-mult", type=float, default=1.0)
    ap.add_argument("--spawn-offset-y", type=float, default=0.0)
    ap.add_argument("--probe", default=None,
                    help="'all' or comma slugs to trace near the sorter")
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
    # FIXED exposure: RTX auto-exposure adapts across hundreds of
    # mixed-camera renders and clips correctly-dark surfaces to cream.
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

    manifest = load_manifest(REPO, extra=args.manifest_extra)
    if args.items:
        keep = set(args.items.split(","))
        manifest = [e for e in manifest if e["slug"] in keep]
    entries = {e["slug"]: e for e in manifest}

    builder = SceneBuilder(stage, REPO, friction_mult=args.friction_mult,
                           mass_mult=args.mass_mult)
    info = builder.build(manifest)
    from isaac.arm import build_arm, ArmController
    arm_rig = build_arm(builder, mode="sorter")

    world.reset()

    # ---------------------------------------------------------- prim wrappers
    def make_rp(path, name):
        rp = SingleRigidPrim(path, name=name)
        if hasattr(rp, "initialize"):
            rp.initialize()
        view = getattr(rp, "_rigid_prim_view", None)
        if view is not None:
            try:
                view.set_sleep_thresholds(np.zeros(1))
            except Exception:
                pass
        return rp

    items_rp = {}
    for slug, meta in info["items"].items():
        items_rp[slug] = make_rp(meta["path"], f"rp_{slug}")

    blade_attr = {}
    for name, jpath in info["blades"].items():
        blade_attr[name] = stage.GetPrimAtPath(jpath).GetAttribute(
            "drive:linear:physics:targetPosition")

    # ------------------------------------------------------------- run state
    rng = np.random.default_rng(args.seed)
    spawn_rng = np.random.default_rng(args.seed + 1000)
    slugs = [e["slug"] for e in manifest]
    order = [str(s) for s in rng.permutation(slugs)]
    gap_lo, gap_hi = (float(v) for v in args.spawn_gap.split(","))
    gaps = list(rng.uniform(gap_lo, gap_hi, size=len(order)))

    a, b = P.BELT_A, P.BELT_B
    cages = P.cages_for()
    watch_zones = dict(cages)
    watch_zones["REVIEW"] = P.REVIEW_PEN
    win0, win1 = P.VIRTUAL_SENSOR["window_x"]
    proc_lat = P.VIRTUAL_SENSOR["processing_latency_s"]

    active, done, watch = {}, {}, {}
    routes = {}
    frozen = set()                                  # injected snags
    frozen_pose = {}
    inject = None
    if args.inject_jam:
        s_, y_ = args.inject_jam.split("@")
        inject = {"slug": s_, "at_y": float(y_), "done": False}
    queue = list(order)
    next_spawn_t = 1.0
    hold2_open = True
    flow = {"spacing_gate_activations": 0, "multi_object_window_events": 0,
            "conveyor_a_stop_count": 0}
    cls_stats = {"n": 0, "correct": 0, "sensor_misses": 0, "reads_total": 0}
    reads_log = {}
    window_multi = False
    events = []
    stills_left = args.depth_stills

    dt_phys = 1.0 / args.physics_hz
    decim = max(1, args.physics_hz // args.control_hz)
    dt_ctrl = dt_phys * decim

    def ev(t, event, slug="", **kv):
        events.append({"t": round(t, 4), "event": event, "slug": slug, **kv})
        st = active.get(slug)
        if st is not None:
            if event == "tilt_cmd":
                st["t_tilt_cmd"] = t
                st["station_cmd"] = kv.get("station")
            elif event == "induction_landed":
                st["t_inducted"] = t
                st["carrier"] = kv.get("carrier")
                st["landing_offset_m"] = kv.get("offset_m")
            elif event == "escapement_release":
                st["t_released"] = t
            elif event == "end_line_callout":
                st["end_callout"] = True
            elif event == "double_occupancy":
                st["review_reason"] = "double_occupancy"
            elif event == "discharge_missed":
                st["review_reason"] = "discharge_miss"

    # ---- the sorter executive
    from isaac.sorter import SorterControl
    sorter = SorterControl(stage, info["carriers"], ev, seed=args.seed,
                           rp_factory=make_rp,
                           dead_route=args.inject_tray_fault)
    sorter.debug_tilt = bool(args.probe)

    def blade_clear_egate():
        for s2 in active:
            p2, _ = pose(s2)
            if abs(p2[1] - a["y"]) > 0.45:
                continue
            hm2 = max(entries[s2]["dims_m"][0], entries[s2]["dims_m"][1]) / 2
            if abs(p2[0] - a["gate_x"]) < hm2 + 0.05:
                return False
        return True

    sorter.bind_escapement(blade_attr["egate"], blade_clear_egate)
    blade_attr["egate"].Set(P.BELT_A["blade_up"])  # normally closed (metering)
    blade_up_state = {"egate": True, "hold2": False}
    # the chain is position-controlled EVERY physics step (240 Hz): smooth
    # kinematic shuttle motion -> the joint-coupled dynamic trays carry real
    # solver velocity for item contact
    world.add_physics_callback("sorter_chain", sorter.physics_tick)
    print(f"[isaac] sorter: {P.SORTER['n_carriers']} carriers, pitch "
          f"{P.SORTER['pitch']} m, v {P.SORTER['v_mps']} m/s, tilt "
          f"{P.SORTER['tilt_deg']} deg, latency "
          f"{P.SORTER['latency_s']*1000:.0f} ms", flush=True)

    # ---- cameras / perception
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
    macro_cam = None
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
        macro_cam = Camera(prim_path=info["cams"]["macro"],
                           resolution=(768, 768))
        macro_cam.initialize()
        macro_cam.add_distance_to_image_plane_to_frame()
        perc = RTXPerception([look_cam] + side_cams, macro=macro_cam)
        jc = Camera(prim_path=info["cams"]["jamcam"], resolution=(1024, 768))
        jc.initialize()
        jc.add_distance_to_image_plane_to_frame()
        wall_c = (cages["C"]["center"][1] + cages["C"]["inner"][1] / 2
                  + cages["C"]["wall_t"] / 2)
        wall_d = (cages["D"]["center"][1] + cages["D"]["inner"][1] / 2
                  + cages["D"]["wall_t"] / 2)
        rp_ = P.REVIEW_PEN
        wall_r = rp_["center"][1] - rp_["inner"][1] / 2 - rp_["wall_t"] / 2
        jam_loc = JamLocator(jc, blade_lines=[a["gate_x"], a["hold2_x"]],
                             hood_zones=[])

    for _ in range(12):                             # warm the render pipeline
        world.step(render=True)

    arm = ArmController(arm_rig, items_rp, entries, ev, mode="sorter")
    recovery = {"active": None}
    if jam_loc is not None:
        for _ in range(6):
            world.step(render=True)
        ok_bg = jam_loc.build_background(n=5, renderer=world.render)
        print(f"[isaac] jam-locator background: {'OK' if ok_bg else 'FAILED'}",
              flush=True)
    from isaac.dressing import RouteVizRuntime
    route_viz = RouteVizRuntime(stage, info.get("viz"))

    def pose(slug):
        p, q = items_rp[slug].get_world_pose()
        return np.asarray(p, dtype=float), np.asarray(q, dtype=float)

    def vel(slug):
        return np.asarray(items_rp[slug].get_linear_velocity(), dtype=float)

    # Priority proof: in surface mode NOTHING moves an item during nominal
    # routing except contact with a powered surface / tilted tray / gravity.
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
        mesh = stage.GetPrimAtPath(f"{info['items'][slug]['path']}/geom")
        UsdGeom.Gprim(mesh).GetDisplayColorAttr().Set(
            [Gf.Vec3f(*[0.50 * v + 0.05 for v in
                        P.ROUTE_RGBA.get(zone, (0.6, 0.6, 0.6))])])

    def which_dest(pos):
        for zone, cage in watch_zones.items():
            cx, cy = cage["center"]
            ix, iy = cage["inner"]
            if abs(pos[0] - cx) < ix / 2 + 0.03 and abs(pos[1] - cy) < iy / 2 + 0.03:
                return zone
        return None

    def deliver(slug, zone_actual, t):
        route_viz.drop_flag(slug)
        sorter.clear_item(slug)
        st = active.pop(slug)
        e = entries[slug]
        ok = zone_actual == e["zone"]
        p, _ = pose(slug)
        cls = st.get("cls") or {}
        done[slug] = {"zone_true": e["zone"], "zone_routed": st.get("zone"),
                      "delivered": zone_actual, "ok": ok,
                      "t_spawn": st.get("t_spawn"), "t_detected": st.get("t_detected"),
                      "t_route_cmd": st.get("t_route_cmd"),
                      "t_released": st.get("t_released"),
                      "t_inducted": st.get("t_inducted"),
                      "t_tilt_cmd": st.get("t_tilt_cmd"),
                      "carrier": st.get("carrier"),
                      "landing_offset_m": st.get("landing_offset_m"),
                      "review_reason": st.get("review_reason"),
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
            n_manual = sum(1 for d2 in done.values()
                           if d2["delivered"] == "MANUAL")
            mx, my = P.MANUAL_STATION
            items_rp[slug].set_world_pose(
                np.array([mx, my + 0.45 * (n_manual - 1), 0.15]),
                np.array([1.0, 0.0, 0.0, 0.0]))
            items_rp[slug].set_linear_velocity(np.zeros(3))
            items_rp[slug].set_angular_velocity(np.zeros(3))
            ev(t, "operator_removed", slug, station=[mx, my])
        if zone_actual in watch_zones:
            v0 = float(np.linalg.norm(vel(slug)))
            watch[slug] = {"zone": zone_actual, "t_entry": t, "v_entry": v0,
                           "max_z": 0.0, "contained": True, "violations": 0,
                           "settle_t": None, "low_since": None}

    def window_clear():
        """Single item in the hold2..gate corridor (measurement discipline)."""
        for slug in active:
            p, _ = pose(slug)
            if (a["hold2_x"] + 0.05 < p[0] <= a["gate_x"] + 0.05
                    and abs(p[1] - a["y"]) < 0.4):
                return False
        return True

    def blade_clear_hold2():
        for s2 in active:
            p2, _ = pose(s2)
            if abs(p2[1] - a["y"]) > 0.45:
                continue
            hm2 = max(entries[s2]["dims_m"][0], entries[s2]["dims_m"][1]) / 2
            if abs(p2[0] - a["hold2_x"]) < hm2 + 0.05:
                return False
        return True

    def set_hold2(want):
        if want and not blade_up_state["hold2"] and not blade_clear_hold2():
            want = False
        blade_up_state["hold2"] = want
        blade_attr["hold2"].Set(P.BELT_A["blade_up"] if want else 0.0)

    def drive_scripted(t):
        """Legacy per-item velocity writes (debug mode: --drive scripted).
        Same route geometry, no train: for camera/pipeline debugging only."""
        for slug in list(active.keys()):
            if slug in frozen or active[slug].get("recovering"):
                continue
            p, _ = pose(slug)
            x, y, z = p
            if not (Z_ON[0] < z < Z_ON[1]):
                continue
            route = routes.get(slug) or active[slug].get("zone")
            st_x = P.STATIONS.get(route, P.STATIONS["REVIEW"])["x"] \
                if route else None
            if x < a["nose_x"] and abs(y - a["y"]) < 0.35:
                set_v(slug, a["speed"], 0.8 * (a["y"] - y))
            elif st_x is not None and x < st_x - 0.02 and abs(y - a["y"]) < 0.4:
                set_v(slug, P.SORTER["v_mps"], 0.8 * (a["y"] - y))
            elif st_x is not None and abs(y - a["y"]) < 0.55:
                side = P.STATIONS.get(route, P.STATIONS["REVIEW"])["side"]
                set_v(slug, 0.0, side * 1.0)

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
            # dual-range evidence: the macro head still for small freight
            if macro_cam is not None and min(entries[slug]["dims_m"]) < 0.03:
                mf = macro_cam.get_current_frame()
                md = mf.get("distance_to_image_plane")
                if md is not None and getattr(md, "size", 0):
                    d2 = np.asarray(md, dtype=float)
                    fin = np.isfinite(d2)
                    if fin.any():
                        lo, hi = np.percentile(d2[fin], [2, 98])
                        img = np.clip((d2 - lo) / max(hi - lo, 1e-6), 0, 1)
                        img[~fin] = 1.0
                        Image.fromarray((255 * (1 - img)).astype(np.uint8)).save(
                            out_dir / f"vision_macro_{slug}.png")
            ev(t, "vision_capture", slug)
        except Exception as exc:
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
                # multi-read sensing while this item owns the corridor
                if (perc is not None and "zone" not in st and "t_detected" in st
                        and p[0] <= win1 and t >= st.get("next_read_t", 0.0)
                        and len(st.get("reads", ())) < P.CLASSIFICATION["read_cap"]):
                    others_ahead = [s2 for s2 in active
                                    if s2 != slug and "zone" not in active[s2]
                                    and pose(s2)[0][0] > p[0]
                                    and pose(s2)[0][0] <= win1 + 0.3]
                    pressed = (blade_up_state.get("egate")
                               and p[0] + entries[slug]["dims_m"][0] / 2
                               > a["gate_x"] - 0.06)
                    if not others_ahead and not pressed:
                        st["next_read_t"] = t + 0.08
                        # annotator freshness: the item never pauses inside
                        # the window (the escapement sits downstream), so a
                        # stale depth frame smears/truncates a 1 m/s item —
                        # flush harder than the old gate-paused build
                        for _ in range(6 if args.record else 10):
                            world.render()
                        excl = [(bx - 0.05, bx + 0.05)
                                for nm, bx in (("egate", a["gate_x"]),
                                               ("hold2", a["hold2_x"]))
                                if blade_up_state.get(nm)]
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
                           mode="rtx_depth_dual_range")
                    else:
                        st["zone"] = e["zone"]
                        ev(t, "item_classified", slug, zone=e["zone"],
                           zone_true=e["zone"], correct=True,
                           latency_ms=round((t - st["t_detected"]) * 1000, 1),
                           mode="oracle_lookahead")
                    st["t_route_cmd"] = t
                    tint(slug, st["zone"])
                    route_viz.make_flag(slug, st["zone"])
                # ---- induction: offer the head item to the sorter
                if (args.drive == "surface" and "zone" in st
                        and "t_released" not in st and slug not in frozen
                        and p[0] > win1 - 0.05 and p[0] < a["nose_x"]
                        and abs(p[1] - a["y"]) < 0.35):
                    dims_meas = (st.get("cls") or {}).get("dims_mm")
                    dims_m = [v / 1000.0 for v in dims_meas] if dims_meas \
                        else list(entries[slug]["dims_m"])
                    sorter.offer(t, slug, st["zone"], float(p[0]),
                                 vx=float(vel(slug)[0]), ready=True,
                                 length_m=dims_m[0], dims_m=dims_m)
                # ---- landing confirmation (tag the carrier)
                if ("t_released" in st and "t_inducted" not in st
                        and p[0] > a["nose_x"] - 0.10):
                    idx = sorter.confirm_landing(t, slug, p)
                    if idx is not None:
                        routes[slug] = sorter.cars[idx]["route"]
                        st["routed_t"] = t
                        st["watch_xy"] = (float(p[0]), float(p[1]))
                        st["watch_t"] = t
                        ev(t, "routing_cmd", slug, zone=routes[slug],
                           carrier=idx)
                # end-of-line operator call-out (dead tilt drill)
                if st.get("end_callout") and not st.get("end_handled"):
                    st["end_handled"] = True
                    deliver(slug, "MANUAL", t)
                    continue
                # deliveries
                if p[1] > b["y_delivered"] and abs(p[0] - b["cx"]) < 0.3:
                    deliver(slug, "B", t)
                    continue
                dest = which_dest(p)
                if dest is not None and p[2] < (0.42 if dest == "REVIEW"
                                                else 0.56):
                    deliver(slug, dest, t)
                    continue
                if (p[2] < 0.22 and dest is None
                        and not (abs(p[0] - 0.4) < 0.3 and p[1] < 0)):
                    deliver(slug, "FLOOR", t)
                    continue
                # jam watchdog (operator call-out / arm recovery)
                if "routed_t" in st and not st.get("jam_reported"):
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
                                for _ in range(4):
                                    world.render()
                                loc = jam_loc.locate(route=st.get("zone"))
                                if loc is not None:
                                    err = float(np.hypot(
                                        loc["pos_xyz"][0] - p[0],
                                        loc["pos_xyz"][1] - p[1]))
                                    if err > 0.30:
                                        ev(t, "jam_locate_rejected", slug,
                                           cam_pos=loc["pos_xyz"],
                                           err_mm=round(err * 1000, 1))
                                        loc = None
                                    else:
                                        ev(t, "jam_located", slug,
                                           cam_pos=loc["pos_xyz"],
                                           err_mm=round(err * 1000, 1))
                                else:
                                    ev(t, "jam_locate_failed", slug)
                            if loc is None:
                                dims = entries[slug]["dims_m"]
                                loc = {"pos_xyz": [float(p[0]), float(p[1]),
                                                   float(p[2])],
                                       "half_extents": [d / 2 for d in dims],
                                       "n_points": 0,
                                       "source": "state_observer"}
                                ev(t, "jam_located_state_observer", slug)
                            # ARM POLICY: only chute snags on the arm's side
                            # (C/D routes, south of the tray sweep) are arm
                            # jobs; everything else is an operator call-out.
                            zone_j = st.get("zone", "")
                            arm_ok = (zone_j in ("C", "D")
                                      and p[1] < P.ARM_GRASP_Y_MAX
                                      and p[2] < 0.60
                                      and abs(p[0] - P.STATIONS[zone_j]["x"]) < 0.5)
                            if (arm is not None and arm_ok and not arm.busy
                                    and recovery["active"] is None
                                    and st.get("recoveries", 0) < 2):
                                dims_j = entries[slug]["dims_m"]
                                pick = [float(p[0]), float(p[1]), float(p[2])]
                                top_z = float(p[2]) + float(dims_j[2]) / 2
                                if arm.start_recovery(slug, zone_j,
                                                      pick, top_z, t):
                                    st["recovering"] = True
                                    recovery["active"] = slug
                                    # station out of service while the arm
                                    # works its chute (industrial lockout)
                                    sorter.station_hold.add(zone_j)
                                    continue
                            deliver(slug, "MANUAL", t)
                            continue
                # watchdog progress reset while commanded-held at a blade
                if "routed_t" not in st:
                    st["watch_xy"] = (float(p[0]), float(p[1]))
                    st["watch_t"] = t
            if in_window >= 2 and not window_multi:
                flow["multi_object_window_events"] += 1
            window_multi = in_window >= 2

            # ---- fault injection: chute snag (jam drill)
            if inject and not inject["done"] and inject["slug"] in active:
                pp_, pq_ = pose(inject["slug"])
                zone_i = active[inject["slug"]].get("zone")
                on_chute = (pp_[2] < 0.50 and zone_i in ("C", "D")
                            and pp_[1] < inject["at_y"])
                if on_chute:
                    inject["done"] = True
                    frozen.add(inject["slug"])
                    frozen_pose[inject["slug"]] = (pp_.copy(), pq_.copy())
                    ev(t, "snag_injected", inject["slug"],
                       at_y=round(float(pp_[1]), 3))

            # ---- flow discipline + executive
            was_hold2 = hold2_open
            hold2_open = window_clear()
            if was_hold2 and not hold2_open:
                flow["spacing_gate_activations"] += 1
            set_hold2(not hold2_open)

            if args.drive == "surface":
                def item_pos_of(slug2):
                    if slug2 in items_rp and slug2 in active:
                        return pose(slug2)[0]
                    return None
                sorter.step(t, dt_ctrl, item_pos_of=item_pos_of)
                blade_up_state["egate"] = sorter.egate_up
            else:
                drive_scripted(t)

            # snag pin: hold the injected jam's POSE until the arm grasps it
            carried = (arm.carry[0] if (arm is not None and arm.carry) else None)
            if carried in frozen:
                frozen.discard(carried)
                frozen_pose.pop(carried, None)
            for s2 in list(frozen):
                if s2 in active and s2 != carried:
                    if s2 in frozen_pose:
                        items_rp[s2].set_world_pose(*frozen_pose[s2])
                    items_rp[s2].set_linear_velocity(np.zeros(3))
                    items_rp[s2].set_angular_velocity(np.zeros(3))
                    vwrites["fault_injection"] += 1

            # route storytelling (visuals only)
            flag_pos = {}
            for s2, st2 in active.items():
                if "zone" in st2:
                    p2, _ = pose(s2)
                    top2 = p2[2] + max(entries[s2]["dims_m"]) / 2
                    flag_pos[s2] = (p2[0], p2[1], top2)
            route_viz.update(None, flag_pos)
            if arm is not None:
                arm.step(dt_ctrl, t)
                if recovery["active"] and not arm.busy:
                    slug_r = recovery["active"]
                    recovery["active"] = None
                    sorter.station_hold.clear()
                    st_r = active.get(slug_r)
                    if st_r is not None:
                        frozen.discard(slug_r)
                        st_r["recovering"] = False
                        st_r["recoveries"] = st_r.get("recoveries", 0) + 1
                        st_r["jam_reported"] = False
                        pr, _ = pose(slug_r)
                        st_r["watch_xy"] = (float(pr[0]), float(pr[1]))
                        st_r["watch_t"] = t
                        ev(t, "recovery_done", slug_r,
                           attempts=st_r["recoveries"])

            # ---- probe trace
            if args.probe == "all" and step_i % args.physics_hz == 0:
                for pslug in list(active.keys()):
                    ps, qq = pose(pslug)
                    vv = vel(pslug)
                    print(f"[probe] {pslug} t={t:7.3f} "
                          f"pos=({ps[0]:.3f},{ps[1]:.3f},{ps[2]:.3f}) "
                          f"v=({vv[0]:+.3f},{vv[1]:+.3f},{vv[2]:+.3f}) "
                          f"route={routes.get(pslug)}", flush=True)
                cars_s = " ".join(
                    f"c{c['i']}:{c['leg'][0]}{sorter.car_x(c)[0]:.2f}"
                    f"{'*' if c['slug'] else ''}" for c in sorter.cars)
                print(f"[probe] train t={t:7.3f} {cars_s}", flush=True)
            elif args.probe and step_i % (4 * decim) == 0:
                for pslug in args.probe.split(","):
                    if pslug not in active:
                        continue
                    ps, _ = pose(pslug)
                    if ps[0] > 6.2:
                        vv = vel(pslug)
                        print(f"[probe] {pslug} t={t:7.3f} "
                              f"pos=({ps[0]:.3f},{ps[1]:.3f},{ps[2]:.3f}) "
                              f"v=({vv[0]:+.3f},{vv[1]:+.3f},{vv[2]:+.3f}) "
                              f"route={routes.get(pslug)}", flush=True)

            # ---- containment watch (to the very end of the run)
            for slug, w in watch.items():
                p, _ = pose(slug)
                cage = watch_zones[w["zone"]]
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
                act = 4.5 < fp[0] < 10.3 and fp[2] > 0.30
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
        if d.get("t_tilt_cmd") and d.get("t_route_cmd"):
            margin = d["t_tilt_cmd"] - d["t_route_cmd"]
        rows.append({"slug": slug, **{k: (round(v, 3) if isinstance(v, float) else v)
                                      for k, v in d.items()},
                     "cycle_s": round(cyc, 3) if cyc else None,
                     "command_margin_s": round(margin, 3) if margin else None})

    n_ok = sum(1 for r in rows if r["ok"])
    unsafe = sum(1 for r in rows if r["delivered"] == "B"
                 and r["zone_true"] in ("C", "D"))
    floor_drops = sum(1 for r in rows if r["delivered"] == "FLOOR")
    review_deliveries = sum(1 for r in rows if r["delivered"] == "REVIEW")
    manual_calls = sum(1 for r in rows if r["delivered"] == "MANUAL")
    recovered = set()
    for e in events:
        if e["event"] == "recovery_done":
            recovered.add(e["slug"])
    review_diversions = sum(1 for r in rows
                            if r["delivered"] in ("REVIEW", "MANUAL"))
    recovery_success = sum(
        1 for r in rows if r["slug"] in recovered
        and (r["ok"] or r["delivered"] in ("REVIEW", "MANUAL")))
    contain_violations = int(sum(w["violations"] for w in watch.values()))
    tracked = [w for w in watch.values()]
    cycles = [r["cycle_s"] for r in rows if r["cycle_s"]]
    margins = [r["command_margin_s"] for r in rows if r["command_margin_s"]]
    t_del = [r["t_delivered"] for r in rows if r.get("t_delivered")]
    summary = {
        "engine": "NVIDIA Isaac Sim 6.0.1 / PhysX 5",
        "perception_mode": ("rtx_depth_dual_range" if perc is not None
                            else "oracle_lookahead"),
        "oracle_used_for_classification": perc is None,
        "sensor_model": ("RTX depth: overhead 1024x768 + 2 side profilers "
                         "768x576 + close-range macro head 768x768 (small-"
                         "item certification), multi-read fusion, guard-"
                         "banded official rule order"
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
        "executive": "tilt_tray_sorter",
        "nominal_motion_model": ("surface_contact_only"
                                 if args.drive == "surface"
                                 else "scripted_velocity_writes"),
        "direct_velocity_writes_nominal": vwrites["nominal_set_v"],
        "direct_velocity_writes_fault_injection": vwrites["fault_injection"],
        "sorter": sorter.summary(),
        "materials": {slug: info["items"][slug].get("material")
                      for slug in entries},
        "sweep": {"friction_mult": args.friction_mult,
                  "mass_mult": args.mass_mult,
                  "spawn_offset_y_m": args.spawn_offset_y,
                  "spawn_gap_s": args.spawn_gap,
                  "inject_jam": args.inject_jam,
                  "inject_tray_fault": args.inject_tray_fault},
        "seed": args.seed,
        "physics_hz": args.physics_hz, "control_hz": args.control_hz,
        "n_items": n_total, "n_delivered": len(done),
        "n_routed_ok": n_ok,
        "routing_accuracy": round(n_ok / max(1, n_total), 4),
        "unsafe_errors": unsafe,
        "floor_drops": floor_drops,
        "review_deliveries": review_deliveries,
        "manual_callouts": manual_calls,
        "review_diversions": review_diversions,
        "recovery": {
            "jams_recovered_by_arm": len(recovered),
            "recovery_success": recovery_success,
            "policy": "C->cage C, D->cage D (route-correct), else operator; "
                      "stuck tray -> review/end-line by design (safe recovery "
                      "separate from routing_accuracy)",
        },
        "containment": {
            "tracked": len(tracked),
            "violations": contain_violations,
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
        "command_margin_s": {
            "min": round(float(np.min(margins)), 3) if margins else None,
            "mean": round(float(np.mean(margins)), 3) if margins else None},
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
    sorter.write_log(out_dir / "actuator_log.csv")
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
    # exit nonzero for ANY bad physical outcome. A floor spill, a containment
    # escape, an unsafe misroute, a negative command margin or a failed
    # recovery are failures the CI gate must catch. MANUAL/REVIEW deliveries
    # are safe designed outcomes (they already cost routing_accuracy).
    bad = (len(done) != n_total or unsafe > 0 or floor_drops > 0
           or contain_violations > 0
           or (margins and min(margins) <= 0)
           or (len(recovered) > 0 and recovery_success < len(recovered)))
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
