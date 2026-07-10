# -*- coding: utf-8 -*-
"""Cell controller: a deterministic state machine driving the palletizer arm.

Cycle:  IDLE -> MOVE_ABOVE -> DESCEND -> ATTACH -> LIFT -> TRANSFER -> LOWER
        -> RELEASE -> RETRACT -> IDLE
plus the fault path: any phase watchdog timeout or a failed grasp check
-> ABORT (release weld, retract home, report) — the cell never deadlocks and
never drags a bad grasp through the cell.

The joint reference is rate-limited toward each waypoint (per-joint vmax) and
position actuators track it. Pick/place heights use the item's LIVE world
AABB, so rolled or tilted items are handled correctly.
"""
import numpy as np
import mujoco

from cell import params as P
from cell.arm_ik import ik, unwrap_yaw

JOINTS = ("j1", "j2", "j3", "j4")
GRASP_XY_TOL = 0.08          # m: TCP-to-item horizontal alignment for suction
GRASP_Z_TOL = 0.05           # m: TCP must be this close to the item top


class Controller:
    def __init__(self, model, data, bus, mode="sorter"):
        self.m, self.d, self.bus = model, data, bus
        self.mode = mode = "sorter"        # EXEC modes collapsed to 'sorter'
        self.base = P.ARM_BASE[mode]
        # Recovery policy (matches the Isaac twin): a jammed item is recovered
        # to its CORRECT category — C snag -> cage C, D snag -> cage D; every
        # other jam location is an operator call-out handled by run_sim.
        self.place = P.PLACE_BY_MODE[mode]
        self.jids = [model.joint(j).id for j in JOINTS]
        self.qadr = [model.jnt_qposadr[j] for j in self.jids]
        self.aids = [model.actuator(f"a{i+1}").id for i in range(4)]
        self.wrist_bid = model.body("wrist").id
        self.tcp_sid = model.site("tcp").id
        self.vmax = np.array(P.ARM["joint_vmax"])
        hx, hy = P.ARM_HOME_XY[mode]
        self.q_home = ik(np.array([hx, hy, P.LIFT_Z]), base=self.base)
        self.q_ref = self.q_home.copy()
        self.state = "IDLE"
        self.job = None
        self.timer = 0.0
        self._target = self.q_home.copy()
        self._deadline = np.inf
        self._apply_ctrl()

    # ------------------------------------------------------------------ low level
    def _apply_ctrl(self):
        for aid, q in zip(self.aids, self.q_ref):
            self.d.ctrl[aid] = q

    def _q_actual(self):
        return np.array([self.d.qpos[a] for a in self.qadr])

    def _at_target(self, q_target, tol=0.03):
        return (np.allclose(self.q_ref, q_target, atol=1e-9)
                and np.max(np.abs(self._q_actual() - q_target)) < tol)

    def _rate_limit_toward(self, q_target, dt):
        dq = np.clip(q_target - self.q_ref, -self.vmax * dt, self.vmax * dt)
        self.q_ref = self.q_ref + dq

    def _set_target(self, q_target, t):
        self._target = q_target
        travel = float(np.max(np.abs(q_target - self.q_ref) / self.vmax))
        self._deadline = t + 3.0 * travel + 2.0    # settle margin: PD creeps
                                                   # under load near contact

    def _ik_wp(self, xyz):
        q = ik(np.asarray(xyz, dtype=float), base=self.base)
        q[0] = unwrap_yaw(q[0], self.q_ref[0])
        return q

    def tcp_pos(self):
        return self.d.site_xpos[self.tcp_sid].copy()

    # ------------------------------------------------------------------ item helpers
    def _item_ids(self, slug):
        return self.m.body(f"item_{slug}").id, self.m.geom(f"g_{slug}").id

    def item_world_aabb(self, slug):
        """(center_xyz, half_extents_xyz) of the item geom in WORLD frame,
        valid for any orientation."""
        bid, gid = self._item_ids(slug)
        aabb = self.m.geom_aabb[gid]           # local: center(3), half(3)
        R = self.d.geom_xmat[gid].reshape(3, 3)
        center = self.d.geom_xpos[gid] + R @ aabb[:3]
        half = np.abs(R) @ aabb[3:]
        return center, half

    def _weld(self, slug, active):
        eid = self.m.equality(f"w_{slug}").id
        if active:
            bid, _ = self._item_ids(slug)
            p1, q1 = self.d.xpos[self.wrist_bid], self.d.xquat[self.wrist_bid]
            p2, q2 = self.d.xpos[bid], self.d.xquat[bid]
            nq1 = np.zeros(4); mujoco.mju_negQuat(nq1, q1)
            rel_p = np.zeros(3); mujoco.mju_rotVecQuat(rel_p, p2 - p1, nq1)
            rel_q = np.zeros(4); mujoco.mju_mulQuat(rel_q, nq1, q2)
            self.m.eq_data[eid, 3:6] = rel_p
            self.m.eq_data[eid, 6:10] = rel_q
            self.m.eq_data[eid, 10] = 1.0
        self.d.eq_active[eid] = 1 if active else 0

    # ------------------------------------------------------------------ job API
    def start_job(self, slug, zone, entry, t):
        center, half = self.item_world_aabb(slug)
        top_z = center[2] + half[2]
        self.job = {"slug": slug, "zone": zone, "t_start": t,
                    "pick_xy": (center[0], center[1]), "weld_on": False}
        try:
            wp_above = self._ik_wp((center[0], center[1], P.LIFT_Z))
            self._ik_wp((center[0], center[1], top_z + 0.002))  # pick reachable?
        except ValueError:
            self.bus.publish("cell_event", t=t, event="pick_unreachable", slug=slug,
                             pos=[round(float(v), 3) for v in center])
            self._abort(t)
            return
        self.state = "MOVE_ABOVE"
        self._set_target(wp_above, t)
        self._pick_wp = (center[0], center[1], top_z + 0.002)
        self.bus.publish("cell_event", t=t, event="recovery_start", slug=slug,
                         target=zone)

    @property
    def busy(self):
        return self.state != "IDLE"

    # ------------------------------------------------------------------ main step
    def step(self, dt, t):
        if self.state == "IDLE":
            self._rate_limit_toward(self.q_home, dt)
            self._apply_ctrl()
            return

        if self.state in ("MOVE_ABOVE", "DESCEND", "LIFT", "TRANSFER", "LOWER", "RETRACT", "ABORT_RETRACT"):
            self._rate_limit_toward(self._target, dt)
            self._apply_ctrl()
            if self._at_target(self._target):
                self._advance(t)
            elif t > self._deadline:
                self.bus.publish("cell_event", t=t, event="phase_timeout",
                                 slug=self.job["slug"], state=self.state,
                                 err=round(float(np.max(np.abs(self._q_actual() - self._target))), 4))
                self._abort(t)
        elif self.state in ("ATTACH", "RELEASE"):
            self.timer -= dt
            if self.timer <= 0:
                self._advance(t)

    # ------------------------------------------------------------------ transitions
    def _grasp_ok(self, t):
        """Suction sanity check before welding: TCP must actually be at the
        item's top surface (v0 model of vacuum-contact feedback)."""
        slug = self.job["slug"]
        center, half = self.item_world_aabb(slug)
        tcp = self.tcp_pos()
        xy_err = float(np.hypot(tcp[0] - center[0], tcp[1] - center[1]))
        z_err = float(abs(tcp[2] - (center[2] + half[2])))
        ok = xy_err < GRASP_XY_TOL and z_err < GRASP_Z_TOL
        if not ok:
            self.bus.publish("cell_event", t=t, event="grasp_check_failed",
                             slug=slug, xy_err=round(xy_err, 3), z_err=round(z_err, 3))
        return ok

    def _abort(self, t):
        j = self.job
        carrying = bool(j.get("weld_on"))
        if carrying:
            self._weld(j["slug"], False)
            j["weld_on"] = False
        self.bus.publish("cell_event", t=t, event="job_abort", slug=j["slug"],
                         carrying=carrying)
        self.state = "ABORT_RETRACT"
        self._set_target(self.q_home.copy(), t)

    def _advance(self, t):
        j = self.job
        if self.state == "ABORT_RETRACT":
            self.state = "IDLE"
            self.job = None
            return
        try:
            self._advance_inner(j, t)
        except ValueError:
            # a waypoint became unreachable (item shifted badly) — abort safely
            self.bus.publish("cell_event", t=t, event="waypoint_unreachable",
                             slug=j["slug"], state=self.state)
            self._abort(t)

    def _advance_inner(self, j, t):

        if self.state == "MOVE_ABOVE":
            self.state = "DESCEND"
            self._set_target(self._ik_wp(self._pick_wp), t)

        elif self.state == "DESCEND":
            if not self._grasp_ok(t):
                self._abort(t)
                return
            self._weld(j["slug"], True)
            j["weld_on"] = True
            self.timer = P.ARM["settle_attach_s"]
            self.state = "ATTACH"
            self.bus.publish("cell_event", t=t, event="attached", slug=j["slug"])

        elif self.state == "ATTACH":
            # plan carry heights from the LIVE carried geometry (any tilt handled):
            # the item bottom must clear the accumulator rails during the swing
            center, half = self.item_world_aabb(j["slug"])
            hang = self.tcp_pos()[2] - (center[2] - half[2])   # TCP to item bottom
            j["hang"] = hang
            j["lift_z"] = min(max(P.LIFT_Z, P.BELT_A["top"] + 0.16 + hang + 0.06), 1.29)
            self.state = "LIFT"
            self._set_target(self._ik_wp((j["pick_xy"][0], j["pick_xy"][1], j["lift_z"])), t)

        elif self.state == "LIFT":
            place = self.place[j["zone"]]
            tx, ty = place["xy"]
            surface = (P.BELT_B["top"] if place["mode"] == "place"
                       else place.get("surface_z", P.CAGE_WALL_TOP))
            self._place_wp = (tx, ty, surface + j["hang"] + place["z_clear"])
            self.state = "TRANSFER"
            self._set_target(self._ik_wp((tx, ty, j["lift_z"])), t)

        elif self.state == "TRANSFER":
            self.state = "LOWER"
            self._set_target(self._ik_wp(self._place_wp), t)

        elif self.state == "LOWER":
            self._weld(j["slug"], False)
            j["weld_on"] = False
            self.timer = P.ARM["settle_release_s"]
            self.state = "RELEASE"
            self.bus.publish("cell_event", t=t, event="released", slug=j["slug"], zone=j["zone"],
                             cycle_s=round(t - j["t_start"], 3))

        elif self.state == "RELEASE":
            self.state = "RETRACT"
            self._set_target(self._ik_wp((self._place_wp[0], self._place_wp[1], P.LIFT_Z)), t)

        elif self.state == "RETRACT":
            self.bus.publish("cell_event", t=t, event="job_done", slug=j["slug"])
            self.state = "IDLE"
            self.job = None
