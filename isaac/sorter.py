# -*- coding: utf-8 -*-
"""Tilt-tray sorter controller: the executive of the SortMaster cell.

Owns the carrier train (chain tracking + wrap), the synchronized escapement
release (induction), per-carrier route assignment, and the discharge tilt
actuation with real command dynamics:

  * command LATENCY (~40 ms pipeline before the drive target starts moving);
  * a target RAMP at tilt_rate_dps (the drive target never step-changes);
  * per-command GAIN NOISE (an actuator never lands exactly on its setpoint);
  * force SATURATION lives in the PhysX drive itself (maxForce N*m).

Nothing in here ever writes an item's state: items are carried by tray
contact, discharged by gravity on the tilted low-friction tray, braked by
the chute + pad. The only direct writes are to MACHINE parts: drive targets,
the escapement blade, and the carrier wrap teleports at the enclosed end
modules (always empty of freight — an occupied carrier reaching the east
module line is an operator call-out first).

Evidence: every tilt command is logged to actuator_log.csv; summary() carries
carrier_commands_count / tilt_time_ms / discharge_latency_ms / landing
offsets / wraps / spacing error, and run_isaac derives command_margin_s per
item (discharge command time - classification-ready time).
"""
import math

import numpy as np

from cell import params as P

BLADE_UP = P.BELT_A["blade_up"]         # panel bottom skims 3 mm above the
                                        # belt: a 9 mm pen cannot slip under


def _roll_deg(q):
    """Quaternion (w,x,y,z) -> rotation about X in degrees (tray tilt)."""
    w, x, y, z = [float(v) for v in q]
    return math.degrees(math.atan2(2 * (w * x + y * z),
                                   1 - 2 * (x * x + y * y)))


class SorterControl:
    """Closed-loop controller for the carrier train."""

    def __init__(self, stage, carriers, ev, seed=0, rp_factory=None,
                 dead_route=None):
        S = P.SORTER
        self.S = S
        self.stage = stage
        self.ev = ev
        self.rng = np.random.default_rng(seed + 4242)
        self.L_top = S["x_east"] - S["x_west"]
        self.loop_len = 2 * self.L_top
        self.v = float(S["v_mps"])
        self.z_top = S["shuttle_top"] - 0.025
        self.z_ret = S["return_z"] - 0.05
        self.dead_route = dead_route            # --inject-tray-fault ZONE
        self.dead_carriers = set()
        self.t_phys = 0.0               # chain time (advanced per physics step)
        self.pivot_dz = S["pivot_z"] - (S["shuttle_top"] - 0.025)
        self.cars = []
        for i, c in enumerate(carriers):
            shuttle_rp = rp_factory(c["shuttle"], f"rp_car{i}")
            tray_rp = rp_factory(c["tray"], f"rp_tray{i}")
            tilt = stage.GetPrimAtPath(c["tilt_joint"])
            s0 = float(c.get("s0", i * S["pitch"]))
            self.cars.append({
                "i": i,
                "shuttle": shuttle_rp, "tray": tray_rp,
                "tilt_attr": tilt.GetAttribute(
                    "drive:angular:physics:targetPosition"),
                "leg": c.get("leg0", "top"),
                "s0": s0,
                "x": (S["x_west"] + s0 if s0 < self.L_top
                      else S["x_east"] - (s0 - self.L_top)),
                "slug": None, "route": None,
                # tilt actuation state
                "cmd": None,            # pending (t_effective, target_deg)
                "target": 0.0,          # ramped drive target written now
                "goal": 0.0,            # post-latency commanded endpoint
                "t_cmd": None, "t_onset": None, "t_full": None,
                "station": None,
                "flat_at": None,        # scheduled re-flatten time
                "expected": None,       # induction: (slug, zone, t_land_est)
            })
        # escapement state
        self.egate_attr = None          # provided via bind_escapement
        self.egate_up = True
        self.release = None             # active release: dict
        self.queue_head = None
        # metrics
        self.commands = 0
        self.cmd_log = []               # (t, carrier, station, target_deg)
        self.wraps = 0
        self.landing_offsets = []
        self.tilt_times = []
        self.discharge_latencies = []
        self.double_occupancy = 0
        self.discharge_misses = 0
        self.end_callouts = 0
        self.releases = 0
        self.max_spacing_err = 0.0
        self._t = 0.0
        self.debug_tilt = False         # per-tick roll trace (--probe runs)
        self.station_hold = set()       # stations out of service (arm active)
        self.mouth_hold = set()         # chute-mouth photo-eye occupancy
        self.hold_at_blade = {}         # slug -> held at escapement (mouth)
        self.release_holds = 0          # escapement holds (mouth occupied)
        self.stuck_flattens = 0
        self.purge_tilts = 0

    # ---------------------------------------------------------------- helpers
    def bind_escapement(self, blade_attr, blade_clear_fn):
        self.egate_attr = blade_attr
        self.blade_clear = blade_clear_fn

    def car_x(self, c):
        """Chain-encoder x of the carrier (the controller's own odometry)."""
        return c["x"], None, None

    def tray_roll(self, c):
        _, q = c["tray"].get_world_pose()
        return _roll_deg(q)

    def occupied(self, c):
        return c["slug"] is not None

    def car_s(self, c):
        """Loop coordinate of the carrier (chain encoder)."""
        return (c["s0"] + self.v * self.t_phys) % self.loop_len

    def empty_available_cars(self):
        """Every empty, untasked carrier ANYWHERE on the loop — a release can
        target a carrier still on the return leg (it wraps onto the top run
        before the item arrives; the escapement waits for the match).
        PHYSICALLY FLAT required (tray-position feedback, |roll| < 3 deg):
        a commanded-flat tray can still be tilted — an obstructed re-flatten
        during the jam drill left one leaning, and freight released onto it
        slid straight off (fault_jam box_s floor drop)."""
        out = []
        for c in self.cars:
            if not self.occupied(c) \
                    and c["expected"] is None and c["target"] == 0.0 \
                    and c["cmd"] is None and c["flat_at"] is None \
                    and abs(self.tray_roll(c)) < 3.0:
                out.append((self.car_s(c), c))
        return out

    # -------------------------------------------------------------- induction
    @staticmethod
    def freight_rolls(dims_m):
        """A lying rod/cylinder re-accelerates by rolling, not sliding lock:
        thin, round-sectioned, elongated (SENSOR dims, metres)."""
        ind = P.INDUCT
        if not dims_m or len(dims_m) < 3:
            return False
        hi, mid, lo = sorted((float(v) for v in dims_m), reverse=True)
        lo = max(lo, 1e-6)
        return (lo < ind["roll_min_dim_m"] and mid / lo < ind["roll_sect_max"]
                and hi / lo >= ind["roll_min_elong"])

    def offer(self, t, slug, zone, x_center, vx=0.0, ready=True,
              length_m=None, dims_m=None):
        """run_isaac offers the head item pressed at (or approaching) the
        escapement. When a suitable empty carrier is inbound, the blade drops
        and the item is released to ride the knife nose onto that carrier.
        length_m: the SENSOR-measured longest dimension — long freight is
        aimed slightly behind the tray centre so its leading edge lands clear
        of the front lip (box_l wedged ON the lip at +0.07 offset otherwise).
        dims_m: full measured dims — a lying rod is aimed with the ROLLING
        re-acceleration model. Returns True while the release is active."""
        if self.release is not None:
            return self.release["slug"] == slug
        if zone in self.mouth_hold or zone in self.station_hold:
            # destination mouth occupied / station out of service: hold the
            # freight AT THE ESCAPEMENT (the accumulator is the buffer —
            # this linear sorter cannot recirculate an occupied carrier
            # through the end-module wrap). Released as soon as clear.
            if not self.hold_at_blade.get(slug):
                self.hold_at_blade[slug] = True
                self.release_holds += 1
                self.ev(t, "escapement_hold", slug, zone=zone)
            return False
        if self.hold_at_blade.pop(slug, None):
            self.ev(t, "escapement_hold_released", slug, zone=zone)
        if not ready or self.egate_attr is None:
            return False
        a = P.BELT_A
        ind = P.INDUCT
        land_bias = -0.05 if (length_m or 0.0) > 0.34 else 0.0
        # time for the item to reach the landing point after the blade drops:
        # already-moving items ride at belt speed; a gate-held item first
        # re-accelerates under belt friction — by ROLLING (slower) if it is
        # a lying rod
        if vx > 0.85 * a["speed"]:
            t_item = (a["nose_x"] - x_center) / a["speed"] + ind["flight_s"]
        else:
            v0, acc = max(0.0, vx), ind["accel_mps2"]
            if self.freight_rolls(dims_m):
                acc *= ind["roll_accel_frac"]
            t_acc = (a["speed"] - v0) / acc
            d_acc = v0 * t_acc + 0.5 * acc * t_acc ** 2
            d_rest = max(0.0, (a["nose_x"] - x_center) - d_acc)
            t_item = t_acc + d_rest / a["speed"] + ind["flight_s"]
        # pick the empty carrier whose centre hits land_x closest to t_item —
        # measured along the LOOP (a matching carrier is usually still on the
        # return leg; the escapement holds until the timing lines up)
        # the length bias is applied in TIME (t_item), not in the target
        # point: the matcher fires when dt_c - t_item first crosses +tol, so
        # a target-point shift cancels out of the landed offset — an earlier
        # carrier (t_item lowered by bias/v) is what actually lands the item
        # behind the tray centre, clear of the front lip
        t_item += land_bias / self.v
        s_land = ind["land_x"] - self.S["x_west"]
        best = None
        for s, c in self.empty_available_cars():
            dt_c = ((s_land - s) % self.loop_len) / self.v
            if dt_c < t_item - ind["release_tol_s"]:
                continue                     # already too close / just past
            err = abs(dt_c - t_item)
            if best is None or err < best[0]:
                best = (err, c, dt_c)
        if best is None or best[0] > ind["release_tol_s"]:
            return False
        _, car, dt_c = best
        car["expected"] = {"slug": slug, "zone": zone, "len_m": length_m,
                           "t_land": t + dt_c, "t_release": t}
        self.release = {"slug": slug, "car": car["i"], "t": t}
        self.releases += 1
        if self.dead_route is not None and zone == self.dead_route \
                and not self.dead_carriers:
            self.dead_carriers.add(car["i"])
            self.ev(t, "tray_fault_armed", slug, carrier=car["i"],
                    route=zone)
        self.ev(t, "escapement_release", slug, carrier=car["i"],
                zone=zone, t_land_est=round(t + dt_c, 3),
                len_m=round(length_m or 0.0, 3), bias=land_bias)
        return True

    def confirm_landing(self, t, slug, item_pos):
        """Called by run_isaac while an inducted item descends: associates it
        with its carrier once it sits on the tray (induction photo-eye +
        chain encoder handshake) and logs the landing offset."""
        for c in self.cars:
            exp = c["expected"]
            if exp is None or exp["slug"] != slug:
                continue
            x, _, _ = self.car_x(c)
            dx = float(item_pos[0]) - x
            on_z = (item_pos[2] < self.S["tray_top"] + 0.19
                    and item_pos[0] > P.BELT_A["nose_x"] + 0.02)
            near = abs(dx) < P.INDUCT["tag_radius_m"] \
                and abs(item_pos[1] - self.S["y"]) < 0.36
            if on_z and near:
                if self.occupied(c):
                    self.double_occupancy += 1
                    c["route"] = "REVIEW"
                    self.ev(t, "double_occupancy", slug, carrier=c["i"],
                            with_slug=c["slug"])
                else:
                    c["slug"] = slug
                    c["route"] = exp["zone"]
                    c["len_m"] = exp.get("len_m") or 0.0
                c["t_tagged"] = t
                self.landing_offsets.append(dx)
                c["expected"] = None
                if self.release and self.release["slug"] == slug:
                    self.release = None
                self.ev(t, "induction_landed", slug, carrier=c["i"],
                        offset_m=round(dx, 4))
                return c["i"]
            if t > exp["t_land"] + 0.8:
                self.ev(t, "induction_miss", slug, carrier=c["i"])
                c["expected"] = None
                if self.release and self.release["slug"] == slug:
                    self.release = None
                # SUSPECT-CARRIER PURGE: the freight is somewhere — most
                # likely riding the matched tray's trailing edge (or the
                # follower) below the association gate. Flag both: each gets
                # a precautionary tilt into REVIEW at its next pass. Empty
                # tray -> harmless flatten; loaded tray -> the freight lands
                # in the manual-review pen instead of circumnavigating the
                # loop and falling off the return run.
                follower = self.cars[(c["i"] - 1) % len(self.cars)]
                for sc in (c, follower):
                    if not sc.get("suspect"):
                        sc["suspect"] = True
                        self.ev(t, "carrier_suspect_flagged", slug,
                                carrier=sc["i"])
        return None

    def carrier_of(self, slug):
        for c in self.cars:
            if c["slug"] == slug:
                return c
        return None

    def clear_item(self, slug):
        """Item delivered / removed by the operator: free its carrier."""
        for c in self.cars:
            if c["slug"] == slug:
                c["slug"], c["route"] = None, None
                c.pop("cmd_issued_at_station", None)
                c.pop("callout", None)
            if c["expected"] and c["expected"]["slug"] == slug:
                c["expected"] = None
        if self.release and self.release["slug"] == slug:
            self.release = None

    # -------------------------------------------------------------- discharge
    def _command_tilt(self, t, c, station, zone):
        # rotX sign convention: POSITIVE roll about +x dips the SOUTH (-y)
        # edge, so a south-side station (side=-1) commands +tilt
        side = P.STATIONS[station]["side"]
        # small freight discharges at a reduced angle (no vault energy)
        frac = (self.S["small_tilt_frac"]
                if 0.0 < (c.get("len_m") or 0.0) < self.S["small_len_m"]
                else 1.0)
        goal = -side * self.S["tilt_deg"] * frac * (
            1.0 + float(self.rng.normal(0.0, self.S["noise_frac"])))
        c["cmd"] = (t + self.S["latency_s"], goal)
        c["t_cmd"], c["t_onset"], c["t_full"] = t, None, None
        c["gone_since"] = None
        c["station"] = station
        self.commands += 1
        self.cmd_log.append((round(t, 4), c["i"], station, round(goal, 2)))
        self.ev(t, "tilt_cmd", c["slug"] or "", carrier=c["i"],
                station=station, target_deg=round(goal, 2))

    def item_discharged(self, t, c, item_pos):
        # gone = past the tray edge ON THE COMMANDED SIDE (0.33 > tray half
        # 0.31 + sway) or below the tray SWEEP. Side-aware: a wide box
        # resting correctly on the B connector keeps its CENTRE near
        # y 3.35-3.45 — a symmetric 0.38 line misread it as still aboard
        # and the stuck-tilt path threw it (twin stress drill).
        # The z line sits UNDER the tilted tray's lowest point (0.425): at
        # z < 0.50 a slow small item that fell back onto its own tilted
        # tray edge read as "discharged" — the re-flatten then SCOOPED it
        # over the mouth containment and dumped it outside the east rail
        # (twin stress s99 pen).
        side = P.STATIONS.get(c.get("station") or "", {}).get("side", 0)
        dy = float(item_pos[1]) - self.S["y"]
        # length-aware z line: small freight can lie ON the tilted tray at
        # z 0.48 (scoop case) -> 0.42; big freight's on-tray centre never
        # dips below ~0.53, and its centre crosses the side line LATE while
        # already committed down the chute -> keep the 0.50 line or a late
        # C discharge reads "stuck" at full tilt and the flatten scoops the
        # half-off box (edge_items_all box_l).
        z_line = 0.42 if (c.get("len_m") or 0.0) < 0.25 else 0.50
        gone = ((side != 0 and side * dy > 0.33) or abs(dy) > 0.42
                or float(item_pos[2]) < z_line)
        if gone and c["t_cmd"] is not None:
            self.discharge_latencies.append(t - c["t_cmd"])
        return gone

    # ----------------------------------------------------------- physics tick
    def physics_tick(self, dt):
        """Chain drive, called EVERY PHYSICS STEP (240 Hz): the kinematic
        shuttles are position-controlled along the loop (top run at deck
        height, true-time return run under the deck); the dynamic trays
        follow through their revolute joints, so freight always rides a body
        with real solver velocity. Leg changes (the end-wheel wraps) happen
        inside the enclosed end modules, always with an EMPTY tray."""
        self.t_phys += float(dt)
        S = self.S
        y = S["y"]
        for c in self.cars:
            s = (c["s0"] + self.v * self.t_phys) % self.loop_len
            if s < self.L_top:
                leg, x, z = "top", S["x_west"] + s, self.z_top
            else:
                leg, x, z = ("return", S["x_east"] - (s - self.L_top),
                             self.z_ret)
            if leg != c["leg"]:
                self._wrap_bookkeeping(c, leg)
            c["leg"], c["x"] = leg, x
            c["shuttle"].set_world_pose(np.array([x, y, z]),
                                        np.array([1.0, 0.0, 0.0, 0.0]))
            if leg != "top" or c.get("_sync_tray"):
                # keep the (empty) tray glued to the shuttle off the top run
                # so the joint never accumulates error across the hop
                c["tray"].set_world_pose(
                    np.array([x, y, z + self.pivot_dz]),
                    np.array([1.0, 0.0, 0.0, 0.0]))
                c["tray"].set_linear_velocity(
                    np.array([self.v if leg == "top" else -self.v, 0.0, 0.0]))
                c["tray"].set_angular_velocity(np.zeros(3))
                c["_sync_tray"] = leg != "top"

    def _wrap_bookkeeping(self, c, new_leg):
        c["target"] = 0.0
        c["goal"] = 0.0
        c["cmd"] = None
        c["flat_at"] = None
        c["t_cmd"] = None
        c["station"] = None
        c["tilt_attr"].Set(0.0)
        c.pop("cmd_issued_at_station", None)
        c.pop("callout", None)
        if new_leg == "return" and c["slug"] is not None:
            # should be unreachable: the end-line call-out removes freight
            # before the module face — never carry an item around the wheel
            self.ev(self._t, "occupied_wrap", c["slug"], carrier=c["i"])
            c["slug"], c["route"] = None, None
        c["_sync_tray"] = True
        self.wraps += 1

    # ------------------------------------------------------------------- step
    def step(self, t, dt, item_pos_of=None):
        """One control tick: escapement blade, discharge triggering, tilt
        target ramps, re-flattening. The chain itself advances in
        physics_tick(). item_pos_of(slug) -> np.array pos, from run_isaac."""
        self._t = t
        S = self.S
        # chain health: encoder x vs actual tray pose (evidence metric)
        if int(t * 2) != int((t - dt) * 2):        # 2 Hz check
            for c in self.cars:
                p, _ = c["tray"].get_world_pose()
                err = abs(float(p[0]) - c["x"])
                self.max_spacing_err = max(self.max_spacing_err, err)
        # ---- discharge triggering (position-based, closed loop)
        for c in self.cars:
            if c["leg"] != "top":
                continue
            x, pos, _ = self.car_x(c)
            if self.occupied(c) and c["cmd"] is None and c["flat_at"] is None \
                    and c["target"] == 0.0:
                route = c["route"]
                st = P.STATIONS.get(route)
                # review fallback: any still-occupied carrier fires at REVIEW
                if st is not None and route != "REVIEW" \
                        and x > st["x"] + 0.45:
                    self.discharge_misses += 1
                    self.ev(t, "discharge_missed", c["slug"], carrier=c["i"],
                            station=route)
                    # a load hanging over the SOUTH edge after a stuck tilt
                    # can only discharge south: use the next south station
                    # (contained in a cage) instead of tilting it north over
                    # open floor toward the review pen
                    if (c.get("stuck_side") == -1 and route != "D"
                            and x < P.STATIONS["D"]["x"] - 0.42):
                        c["route"] = "D"
                        route, st = "D", P.STATIONS["D"]
                    else:
                        c["route"] = "REVIEW"
                        route, st = "REVIEW", P.STATIONS["REVIEW"]
                if st is not None and (route in self.station_hold
                        or route in self.mouth_hold):
                    # station out of service (exception arm working there):
                    # do not fire; the carrier rides on and the REVIEW
                    # fallback re-routes it (industrial out-of-service flow)
                    st = None
                if st is not None:
                    # long freight discharges earlier: its leading edge
                    # travels further before the CG clears the tray
                    extra = 0.9 * max(0.0, (c.get("len_m") or 0.0) - 0.30)
                    # rolling smalls exit at the mouth's WEST third: their
                    # eastward carrier drift then carries them INTO the
                    # chute instead of past its east edge (a roller's exit
                    # speed barely drops with tilt angle - rolling has no
                    # friction threshold - so aim, not energy, is the knob)
                    ln = c.get("len_m") or 0.0
                    if 0.0 < ln < self.S["small_len_m"]:
                        extra += 0.22
                    trig = st["x"] - st["trigger_lead_m"] - extra
                    # SEAT TIME: never tilt during landing settle — long
                    # C-bound freight otherwise tilts AT the landing instant
                    # and discharges while still bouncing from the drop.
                    # NOTE: the flat value is DESIGN-BINDING for C freight —
                    # induction lands essentially at the C trigger, so every
                    # C discharge fires right at this boundary. A length-
                    # scaled seat was tried (box_l late-slide hunt) and
                    # REJECTED by the matrix: deferring long freight shifts
                    # every C discharge east and misroutes nominal seeds.
                    seated = (c.get("t_tagged") is None
                              or t - c["t_tagged"] >= self.S["seat_time_s"])
                    if seated and x >= trig \
                            and not c.get("cmd_issued_at_station") == route:
                        c["cmd_issued_at_station"] = route
                        if c["i"] in self.dead_carriers:
                            # hardware fault drill: a dead actuator swallows
                            # EVERY command (station AND review fallback) —
                            # the freight rides to the end-line call-out
                            self.commands += 1
                            self.cmd_log.append((round(t, 4), c["i"],
                                                 route, 0.0))
                            self.ev(t, "tilt_cmd_dead", c["slug"],
                                    carrier=c["i"], station=route)
                        else:
                            self._command_tilt(t, c, route, c["route"])
            # suspect purge: an induction-miss suspect gets a precautionary
            # REVIEW tilt (untagged freight may be riding it — see
            # confirm_landing). occupied() is False, so after the tilt the
            # empty-tilt path re-flattens it without discharge-confirm noise.
            if (c.get("suspect") and not self.occupied(c)
                    and c["cmd"] is None and c["flat_at"] is None
                    and c["target"] == 0.0
                    and "REVIEW" not in self.station_hold
                    and "REVIEW" not in self.mouth_hold
                    and c["i"] not in self.dead_carriers):
                stp = P.STATIONS["REVIEW"]
                if x >= stp["x"] - stp["trigger_lead_m"] \
                        and c.get("cmd_issued_at_station") != "RVW_PURGE":
                    c["cmd_issued_at_station"] = "RVW_PURGE"
                    c["suspect"] = False
                    self.purge_tilts += 1
                    self.ev(t, "suspect_purge_tilt", c["slug"] or "",
                            carrier=c["i"])
                    self._command_tilt(t, c, "REVIEW", "REVIEW")
                    # full dwell: latency + ramp + a stowaway's slide-off
                    # (the 0.4 s empty-tilt settle is not enough for that)
                    c["flat_at"] = t + 1.8
            # end-of-line: an occupied carrier at the east module line is an
            # operator call-out (dead tilt / unresolvable freight)
            if (self.occupied(c) and x >= S["occupied_callout_x"]
                    and not c.get("callout")):
                c["callout"] = True
                self.end_callouts += 1
                self.ev(t, "end_line_callout", c["slug"], carrier=c["i"])
        # ---- tilt pipelines: latency -> ramp -> settle -> re-flatten
        rate = S["tilt_rate_dps"] * dt
        for c in self.cars:
            if c["cmd"] is not None and t >= c["cmd"][0]:
                c["goal"] = c["cmd"][1]
                c["cmd"] = None
                c["t_onset"] = t
            # ramp the drive target toward the goal
            if abs(c["target"] - c["goal"]) > 1e-6:
                d = c["goal"] - c["target"]
                c["target"] += math.copysign(min(abs(d), rate), d)
                c["tilt_attr"].Set(float(c["target"]))
            # measure full-tilt arrival (physical joint angle)
            if c["t_onset"] is not None and c["t_full"] is None \
                    and abs(c["goal"]) > 1.0:
                roll = self.tray_roll(c)
                if self.debug_tilt and int(t * 5) != int((t - dt) * 5):
                    print(f"[tilt] t={t:7.3f} car={c['i']} x={c['x']:.3f} "
                          f"target={c['target']:+.1f} goal={c['goal']:+.1f} "
                          f"roll={roll:+.2f}", flush=True)
                # SIGN-AWARE arrival: |roll| alone certified a retry tilt
                # against the STALE opposite-side angle (smoke10 box_l)
                if roll * math.copysign(1.0, c["goal"]) >= 0.92 * abs(c["goal"]):
                    c["t_full"] = t
                    self.tilt_times.append(t - c["t_cmd"])
                    self.ev(t, "tilt_full", c["slug"] or "", carrier=c["i"],
                            tilt_ms=round((t - c["t_cmd"]) * 1000, 1))
            # discharge confirmation -> schedule re-flatten. The "gone"
            # condition must PERSIST (a mid-pivot item oscillates through
            # any geometric threshold — a one-tick confirm let the flatten
            # scoop a mid-pivot sack), and the tray then dwells tilted so
            # the item finishes falling before the tray moves.
            if abs(c["goal"]) > 1.0 and self.occupied(c) \
                    and item_pos_of is not None and c["flat_at"] is None:
                ip = item_pos_of(c["slug"])
                gone_now = ip is not None and self.item_discharged(t, c, ip)
                if gone_now and c.get("gone_since") is None:
                    c["gone_since"] = t
                elif not gone_now:
                    c["gone_since"] = None
                # GEOGRAPHIC late-discharge guard: a load still aboard a
                # tilted tray this close to the NEXT station's mouth would
                # exit INSIDE that mouth (low-friction box_l slid 1.8 s —
                # under the stuck TIMER — and cross-delivered C into D).
                # Position, not time, bounds a legal discharge. ONLY the
                # C->D pair: cross-delivery needs a SAME-FLANK downstream
                # mouth (C and D discharge south; REVIEW discharges north,
                # and the south run past D is pans + seam fin). A D->REVIEW
                # guard would also false-fire: the D-REVIEW gap is shorter
                # than the lead and the guard flattened EVERY D discharge
                # at tilt-full (caught by the twin fault_jam smoke).
                nxt = {"C": "D"}.get(c["station"])
                late_geo = (nxt is not None and c["t_full"] is not None
                            and c["x"] >= P.STATIONS[nxt]["x"]
                            - S["late_flatten_lead_m"])
                if gone_now and t - c["gone_since"] >= \
                        S["confirm_persist_s"]:
                    self.ev(t, "discharge_confirmed", c["slug"],
                            carrier=c["i"], station=c["station"],
                            latency_ms=round(
                                (c["gone_since"] - c["t_cmd"]) * 1000, 1))
                    c["slug"], c["route"] = None, None
                    c["gone_since"] = None
                    c["flat_at"] = t + S["discharge_dwell_s"]
                elif (late_geo
                      or (c["t_full"] is not None and t - c["t_full"] >
                          (2.0 if (c.get("len_m") or 0.0) >= 0.35 else 1.2))
                      or (c["t_onset"] is not None and c["t_full"] is None
                          and t - c["t_onset"] > 2.8)):
                    # STUCK-TILT TIMEOUT: the item did not clear a full tilt
                    # (wedged on a lip / freak pose). Re-flatten so the
                    # carrier becomes eligible for the REVIEW fallback — a
                    # tilted tray must never ride to the end line silently.
                    self.stuck_flattens += 1
                    # remember which side the item hangs over: a half-off
                    # load discharges reliably to THAT side only
                    if ip is not None:
                        c["stuck_side"] = -1 if float(ip[1]) < self.S["y"] \
                            else +1
                    self.ev(t, "tilt_stuck_flatten", c["slug"],
                            carrier=c["i"], station=c["station"],
                            side=c.get("stuck_side"),
                            trigger="next_mouth_geo" if late_geo else "timer")
                    c["goal"] = 0.0
                    c["flat_at"] = t + 0.05
                    c["station"] = None
                    c["t_cmd"] = None
                    c.pop("cmd_issued_at_station", None)
            # empty tilted carrier (item never landed / already gone)
            if abs(c["goal"]) > 1.0 and not self.occupied(c) \
                    and c["flat_at"] is None:
                c["flat_at"] = t + S["settle_s"]
            if c["flat_at"] is not None and t >= c["flat_at"]:
                c["goal"] = 0.0
                c["flat_at"] = None
                c["t_cmd"] = None
                c["station"] = None
        # ---- escapement blade actuation (owned by the sorter). The blade
        # re-raises as soon as the released item is committed past the nose
        # (not at landing) so a close follower cannot slip through.
        if self.egate_attr is not None:
            rel = self.release
            if (rel is not None and not rel.get("past_nose")
                    and item_pos_of is not None):
                ip = item_pos_of(rel["slug"])
                if ip is not None and float(ip[0]) > P.BELT_A["nose_x"] + 0.03:
                    rel["past_nose"] = True
            want_up = rel is None or rel.get("past_nose", False)
            if want_up and not self.egate_up:
                if self.blade_clear():
                    self.egate_attr.Set(BLADE_UP)
                    self.egate_up = True
            elif not want_up and self.egate_up:
                self.egate_attr.Set(0.0)
                self.egate_up = False

    # -------------------------------------------------------------- evidence
    def summary(self):
        lo = np.abs(np.array(self.landing_offsets)) if self.landing_offsets \
            else np.array([0.0])
        return {
            "mechanism": "tilt_tray_linear_sorter",
            "n_carriers": self.S["n_carriers"],
            "carrier_pitch_m": self.S["pitch"],
            "train_speed_mps": self.S["v_mps"],
            "tilt_deg": self.S["tilt_deg"],
            "actuator_latency_ms": round(self.S["latency_s"] * 1000, 1),
            "tilt_rate_dps": self.S["tilt_rate_dps"],
            "gain_noise_frac": self.S["noise_frac"],
            "carrier_commands_count": self.commands,
            "escapement_releases": self.releases,
            "tilt_time_ms": (round(float(np.mean(self.tilt_times)) * 1000, 1)
                             if self.tilt_times else None),
            "discharge_latency_ms": (
                round(float(np.mean(self.discharge_latencies)) * 1000, 1)
                if self.discharge_latencies else None),
            "landing_offset_mean_mm": round(float(np.mean(lo)) * 1000, 1),
            "landing_offset_max_mm": round(float(np.max(lo)) * 1000, 1),
            "carrier_wraps": self.wraps,
            "chain_spacing_err_max_mm": round(self.max_spacing_err * 1000, 1),
            "double_occupancy_events": self.double_occupancy,
            "discharge_misses_to_review": self.discharge_misses,
            "stuck_tilt_flattens": self.stuck_flattens,
            "suspect_purge_tilts": self.purge_tilts,
            "escapement_holds": self.release_holds,
            "end_line_callouts": self.end_callouts,
        }

    def write_log(self, path):
        import csv
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t", "carrier", "station", "target_deg"])
            w.writerows(self.cmd_log)
