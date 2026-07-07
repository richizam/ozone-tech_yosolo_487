# -*- coding: utf-8 -*-
"""4-axis palletizer exception arm for the Isaac Sim cell.

Faithful port of cell/controller.py + cell/arm_ik.py. In the MuJoCo twin the
arm's links are collision-free (contype 0) and the item is carried by a weld —
the arm's only physical footprint IS the carried item. This port keeps exactly
those semantics: the links are kinematic visuals driven by the same
rate-limited joint references from the same closed-form IK, and the carried
item is kinematically attached to the computed TCP (weld analog).

Recovery is SENSOR-TRUE: the pick target comes from the jam camera's
background-subtraction fix (position + measured top height), not from
simulator state.

State machine (identical cycle):
  IDLE -> MOVE_ABOVE -> DESCEND -> ATTACH -> LIFT -> TRANSFER -> LOWER
       -> RELEASE -> RETRACT -> IDLE      (+ ABORT_RETRACT on any timeout)
"""
import math

import numpy as np

from cell import params as P
from cell.arm_ik import fk, ik, unwrap_yaw


def build_arm(builder, mode="table"):
    """Kinematic arm visuals: nested Xforms with runtime-set rotations.
    Returns the xform paths + geometry constants for runtime driving."""
    from pxr import Gf, UsdGeom

    bx, by = P.ARM_BASE[mode]
    arm = P.ARM
    stage = builder.stage
    root = "/World/arm"
    UsdGeom.Xform.Define(stage, root)

    # static pedestal (collides — it is real furniture in the cell)
    builder.add_cylinder("arm_pedestal_col", (bx, by, arm["pedestal_h"] / 2),
                         0.15, arm["pedestal_h"] / 2, color=(0.25, 0.25, 0.28))

    def _xform(path, translate):
        xf = UsdGeom.Xform.Define(stage, path)
        ops = UsdGeom.Xformable(xf.GetPrim())
        ops.AddTranslateOp().Set(Gf.Vec3d(*translate))
        rot = ops.AddRotateXYZOp()
        rot.Set(Gf.Vec3f(0, 0, 0))
        return rot

    def _capsule(path, length, radius, color):
        cap = UsdGeom.Capsule.Define(stage, path)
        cap.CreateAxisAttr("X")
        cap.CreateHeightAttr(float(max(0.01, length - 2 * radius)))
        cap.CreateRadiusAttr(float(radius))
        UsdGeom.Xformable(cap.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(length / 2, 0, 0))
        cap.CreateDisplayColorAttr([Gf.Vec3f(*color)])

    orange = (0.85, 0.55, 0.10)
    # yaw column: rotates about Z at the pedestal top
    rot_yaw = _xform(f"{root}/yaw", (bx, by, arm["pedestal_h"]))
    col = UsdGeom.Cylinder.Define(stage, f"{root}/yaw/column")
    col.CreateRadiusAttr(0.09)
    col.CreateHeightAttr(float(arm["yaw_col_h"]))
    col.CreateAxisAttr("Z")
    UsdGeom.Xformable(col.GetPrim()).AddTranslateOp().Set(
        Gf.Vec3d(0, 0, arm["yaw_col_h"] / 2))
    col.CreateDisplayColorAttr([Gf.Vec3f(*orange)])
    # upper arm: pitches about local Y at the shoulder
    rot_upper = _xform(f"{root}/yaw/upper", (0, 0, arm["yaw_col_h"]))
    _capsule(f"{root}/yaw/upper/link", arm["L1"], 0.06, orange)
    # forearm
    rot_fore = _xform(f"{root}/yaw/upper/fore", (arm["L1"], 0, 0))
    _capsule(f"{root}/yaw/upper/fore/link", arm["L2"], 0.05, orange)
    # wrist + suction tool (points down when q4 = -(q2+q3))
    rot_wrist = _xform(f"{root}/yaw/upper/fore/wrist", (arm["L2"], 0, 0))
    tool = UsdGeom.Cylinder.Define(stage, f"{root}/yaw/upper/fore/wrist/tool")
    tool.CreateRadiusAttr(0.04)
    tool.CreateHeightAttr(float(arm["tool_len"] - 0.05))
    tool.CreateAxisAttr("Z")
    UsdGeom.Xformable(tool.GetPrim()).AddTranslateOp().Set(
        Gf.Vec3d(0, 0, -(arm["tool_len"] - 0.05) / 2 - 0.02))
    tool.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.2, 0.22)])
    return {"yaw": rot_yaw, "upper": rot_upper, "fore": rot_fore,
            "wrist": rot_wrist, "base": (bx, by)}


class ArmController:
    """Rate-limited kinematic execution of the pick/recover cycle."""

    GRASP_XY_TOL = 0.10
    GRASP_Z_TOL = 0.08

    def __init__(self, rig, items_rp, entries, publish, mode="table"):
        self.rig = rig
        self.items_rp = items_rp
        self.entries = entries
        self.pub = publish
        self.mode = mode
        self.base = rig["base"]
        self.place = P.PLACE_BY_MODE[mode]
        self.vmax = np.array(P.ARM["joint_vmax"])
        # park pose: FOLDED UP in joint space (upper arm raised, forearm
        # tucked), yawed toward the open south-east floor. An IK'd XY home
        # folds the elbow OUT over cage C and the retract sweep clips the
        # chute hood on video; a compact high fold keeps every link within
        # ~0.35 m of the column at all yaws — clips nothing, anywhere.
        q2h, q3h = math.radians(-78.0), math.radians(132.0)
        self.q_fold = (q2h, q3h)
        self.q_home = np.array([math.radians(-40.0), q2h, q3h, -(q2h + q3h)])
        self.q_ref = self.q_home.copy()
        self.state = "IDLE"
        self.job = None
        self.timer = 0.0
        self._target = self.q_home.copy()
        self._deadline = np.inf
        self.carry = None                 # (slug, dz) while attached
        self._apply()

    # ------------------------------------------------------------- kinematics
    def _apply(self):
        from pxr import Gf
        q1, q2, q3, q4 = [math.degrees(float(v)) for v in self.q_ref]
        self.rig["yaw"].Set(Gf.Vec3f(0, 0, q1))
        # MuJoCo hinge about +y: positive pitches DOWN; USD rotateXYZ Y
        # rotation is the same right-handed axis
        self.rig["upper"].Set(Gf.Vec3f(0, q2, 0))
        self.rig["fore"].Set(Gf.Vec3f(0, q3, 0))
        self.rig["wrist"].Set(Gf.Vec3f(0, q4, 0))
        if self.carry is not None:
            slug, dz = self.carry
            tcp, _ = fk(self.q_ref, base=self.base)
            rp = self.items_rp[slug]
            rp.set_world_pose(np.array([tcp[0], tcp[1], tcp[2] - dz]),
                              np.array([1.0, 0.0, 0.0, 0.0]))
            rp.set_linear_velocity(np.zeros(3))
            rp.set_angular_velocity(np.zeros(3))

    def tcp(self):
        return fk(self.q_ref, base=self.base)[0]

    def _rate_toward(self, q_target, dt):
        dq = np.clip(q_target - self.q_ref, -self.vmax * dt, self.vmax * dt)
        self.q_ref = self.q_ref + dq

    def _set_target(self, q_target, t):
        self._target = np.asarray(q_target, dtype=float)
        travel = float(np.max(np.abs(self._target - self.q_ref) / self.vmax))
        self._deadline = t + 3.0 * travel + 2.0

    def _wp(self, xyz):
        q = ik(np.asarray(xyz, dtype=float), base=self.base)
        q[0] = unwrap_yaw(q[0], self.q_ref[0])
        return q

    @property
    def busy(self):
        return self.state != "IDLE"

    # ---------------------------------------------------------------- job API
    def start_recovery(self, slug, zone, pick_xyz, top_z, t):
        """Recover a jammed item: pick at the CAMERA-measured position, lift,
        re-deliver onto its lane exit so the zone conveyor re-routes it."""
        try:
            wp_above = self._wp((pick_xyz[0], pick_xyz[1], P.LIFT_Z))
            self._wp((pick_xyz[0], pick_xyz[1], top_z + 0.002))
        except ValueError:
            self.pub(t, "pick_unreachable", slug,
                     pos=[round(float(v), 3) for v in pick_xyz])
            return False
        self.job = {"slug": slug, "zone": zone, "t_start": t,
                    "pick": (float(pick_xyz[0]), float(pick_xyz[1]),
                             float(top_z))}
        self.state = "MOVE_ABOVE"
        self._set_target(wp_above, t)
        self.pub(t, "recovery_started", slug, zone=zone)
        return True

    # ------------------------------------------------------------------ step
    def step(self, dt, t):
        if self.state == "IDLE":
            self._rate_toward(self.q_home, dt)
            self._apply()
            return
        if self.state in ("MOVE_ABOVE", "DESCEND", "LIFT", "TRANSFER",
                          "LOWER", "RETRACT", "FOLD", "ABORT_RETRACT"):
            self._rate_toward(self._target, dt)
            self._apply()
            at = (np.allclose(self.q_ref, self._target, atol=1e-9))
            if at:
                self._advance(t)
            elif t > self._deadline:
                self.pub(t, "phase_timeout", self.job["slug"],
                         state=self.state)
                self._abort(t)
        elif self.state in ("ATTACH", "RELEASE"):
            self.timer -= dt
            self._apply()
            if self.timer <= 0:
                self._advance(t)

    def _abort(self, t):
        j = self.job
        if self.carry is not None:
            self.carry = None
        self.pub(t, "job_abort", j["slug"])
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
            self.pub(t, "waypoint_unreachable", j["slug"], state=self.state)
            self._abort(t)

    def _advance_inner(self, j, t):
        if self.state == "MOVE_ABOVE":
            px, py, top = j["pick"]
            self.state = "DESCEND"
            self._set_target(self._wp((px, py, top + 0.002)), t)
        elif self.state == "DESCEND":
            slug = j["slug"]
            p, _ = self.items_rp[slug].get_world_pose()
            tcp = self.tcp()
            xy_err = float(np.hypot(tcp[0] - p[0], tcp[1] - p[1]))
            if xy_err > self.GRASP_XY_TOL + 0.10:
                self.pub(t, "grasp_check_failed", slug,
                         xy_err=round(xy_err, 3))
                self._abort(t)
                return
            dz = float(tcp[2] - p[2])              # TCP to item centre
            self.carry = (slug, dz)
            self.timer = P.ARM["settle_attach_s"]
            self.state = "ATTACH"
            self.pub(t, "attached", slug)
        elif self.state == "ATTACH":
            px, py, _ = j["pick"]
            j["lift_z"] = min(P.LIFT_Z + 0.05, 1.29)
            self.state = "LIFT"
            self._set_target(self._wp((px, py, j["lift_z"])), t)
        elif self.state == "LIFT":
            place = self.place[j["zone"]] if j["zone"] in self.place else None
            if place is None:                      # B has no drop: lane exit
                tx, ty = P.TABLE["lane_B_cx"], P.TABLE["y"]
                surface = P.TABLE["top"]
                clear = 0.02
            else:
                tx, ty = place["xy"]
                surface = place.get("surface_z", P.CAGE_WALL_TOP)
                clear = place["z_clear"]
            slug = j["slug"]
            half_z = self.entries[slug]["dims_m"][2] / 2
            dz = self.carry[1] if self.carry else half_z
            j["place_wp"] = (tx, ty, surface + dz + half_z + clear + 0.01)
            self.state = "TRANSFER"
            self._set_target(self._wp((tx, ty, j["lift_z"])), t)
        elif self.state == "TRANSFER":
            self.state = "LOWER"
            self._set_target(self._wp(j["place_wp"]), t)
        elif self.state == "LOWER":
            self.carry = None                      # release the vacuum
            self.timer = P.ARM["settle_release_s"]
            self.state = "RELEASE"
            self.pub(t, "released", j["slug"], zone=j["zone"],
                     cycle_s=round(t - j["t_start"], 3))
        elif self.state == "RELEASE":
            self.state = "RETRACT"
            wp = j["place_wp"]
            self._set_target(self._wp((wp[0], wp[1], P.LIFT_Z)), t)
        elif self.state == "RETRACT":
            # FOLD at the current yaw BEFORE swinging home: folding and
            # yawing simultaneously sweeps a half-extended forearm across
            # the C-chute hood on video
            q2h, q3h = self.q_fold
            self.state = "FOLD"
            self._set_target(np.array([self.q_ref[0], q2h, q3h,
                                       -(q2h + q3h)]), t)
        elif self.state == "FOLD":
            self.pub(t, "job_done", j["slug"])
            self.state = "IDLE"
            self.job = None
