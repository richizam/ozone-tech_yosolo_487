# -*- coding: utf-8 -*-
"""ARB routing deck controller: a matrix of LOCAL surface-velocity actuator
patches instead of one intelligent surface.

Each patch is its own kinematic PhysX body with PhysxSurfaceVelocityAPI (built
by scene_usd from cell/params.arb_patches()). The controller gives every patch
the actuation dynamics a real Activated-Roller-Belt cell has:

  * a command PIPELINE: a new command takes latency_s to reach the hardware;
  * a velocity RAMP: the attained surface velocity slews at ramp_mps2, never
    step-changes;
  * SATURATION at v_max;
  * per-command GAIN NOISE (an actuator never lands exactly on its setpoint).

Only the patches the routed item covers (plus a pre-spin halo — the cells it
is about to cross) receive the divert command; the rest of the deck keeps
feeding forward. Every state change is a logged actuator command, so the
evidence can show actuator_commands_count / latency / max attained speed and
prove no direct velocity write ever moves an item during nominal routing.
"""
import numpy as np
from pxr import Gf, UsdGeom

from cell import params as P

IDLE_PILL = (0.14, 0.145, 0.16)         # embedded-roller dark metal
FORWARD_PILL = (0.16, 0.18, 0.22)       # feeding: barely lighter (spinning)


class ArbDeck:
    def __init__(self, stage, patches, feed_speed, cfg=None, seed=0,
                 pill_paths=None):
        """patches: [{path, r, c, cx, cy, hx, hy}] from the scene builder;
        pill_paths: optional {"r{r}c{c}": [pill prim paths]} visual overlay
        (official-asset shells); patch collider boxes are tinted too so the
        activation is visible in the primitive fallback as well."""
        self.cfg = dict(P.ARB_DECK if cfg is None else cfg)
        self.patches = list(patches)
        self.n = len(self.patches)
        self.speed = float(feed_speed)
        self.feed = np.array([self.speed, 0.0])
        self.rng = np.random.default_rng(seed + 777)
        from pxr import PhysxSchema
        self.attrs, self.color_attrs, self.ids = [], [], []
        for p in self.patches:
            prim = stage.GetPrimAtPath(p["path"])
            api = PhysxSchema.PhysxSurfaceVelocityAPI(prim)
            self.attrs.append(api.GetSurfaceVelocityAttr())
            pid = f"r{p['r']}c{p['c']}"
            self.ids.append(pid)
            cols = [UsdGeom.Gprim(prim).GetDisplayColorAttr()]
            for pp in (pill_paths or {}).get(pid, ()):
                pr = stage.GetPrimAtPath(pp)
                if pr:
                    cols.append(UsdGeom.Gprim(pr).GetDisplayColorAttr())
            self.color_attrs.append(cols)
        self.pstate = [{"cmd_state": "forward", "cmd_vec": self.feed.copy(),
                        "pending": None, "state": "forward",
                        "target": self.feed.copy(), "v": self.feed.copy()}
                       for _ in range(self.n)]
        self.commands = 0
        self.cmd_log = []               # (t, patch, state, vx, vy)
        self.state_counts = {}
        self.max_speed = self.speed
        self._lat_sum, self._lat_n = 0.0, 0

    # ------------------------------------------------------------- commands
    def command(self, t, owner_xy=None, owner_half=0.0, route=None,
                exit_xy=None):
        """Issue this control tick's desired state per patch. Commands enter
        the latency pipeline; nothing moves instantly."""
        if owner_xy is None or route is None or exit_xy is None:
            desired = [("forward", self.feed)] * self.n
        else:
            d = np.array([exit_xy[0] - owner_xy[0], exit_xy[1] - owner_xy[1]])
            nrm = float(np.linalg.norm(d))
            vec = self.speed * d / nrm if nrm > 1e-6 else np.zeros(2)
            pad = self.cfg["activation_pad_m"]
            state = f"divert:{route}"
            desired = []
            for p in self.patches:
                near = (abs(p["cx"] - owner_xy[0]) <= owner_half + pad + p["hx"]
                        and abs(p["cy"] - owner_xy[1])
                        <= owner_half + pad + p["hy"])
                desired.append((state, vec) if near
                               else ("forward", self.feed))
        for i, (state, vec) in enumerate(desired):
            st = self.pstate[i]
            # closed-loop aiming: while diverting, the commanded vector tracks
            # the item — re-issue only when it moved appreciably (hysteresis)
            stale = (state != st["cmd_state"]
                     or (state != "forward"
                         and float(np.linalg.norm(vec - st["cmd_vec"])) > 0.10))
            if stale:
                st["cmd_state"] = state
                st["cmd_vec"] = np.array(vec, dtype=float)
                st["pending"] = (t + self.cfg["latency_s"], state,
                                 st["cmd_vec"].copy())
                self.commands += 1
                self.state_counts[state] = self.state_counts.get(state, 0) + 1
                self.cmd_log.append((round(t, 4), self.ids[i], state,
                                     round(float(vec[0]), 3),
                                     round(float(vec[1]), 3)))

    # ----------------------------------------------------------------- step
    def step(self, t, dt):
        """Advance the actuation model one control tick: latency pipeline,
        gain noise at hardware, ramp toward target, saturation."""
        ramp_dv = self.cfg["ramp_mps2"] * dt
        for i, st in enumerate(self.pstate):
            if st["pending"] is not None and t >= st["pending"][0]:
                t_eff, state, vec = st["pending"]
                st["pending"] = None
                gain = 1.0 + float(self.rng.normal(0.0, self.cfg["noise_frac"]))
                st["target"] = vec * gain
                self._lat_sum += self.cfg["latency_s"]
                self._lat_n += 1
                if st["state"] != state:
                    st["state"] = state
                    self._tint(i, state)
            dv = st["target"] - st["v"]
            n = float(np.linalg.norm(dv))
            if n < 1e-6:
                continue
            st["v"] = (st["target"].copy() if n <= ramp_dv
                       else st["v"] + dv * (ramp_dv / n))
            spd = float(np.linalg.norm(st["v"]))
            if spd > self.cfg["v_max"]:
                st["v"] *= self.cfg["v_max"] / spd
                spd = self.cfg["v_max"]
            self.max_speed = max(self.max_speed, spd)
            # PhysX surfaceVelocity is local-frame scaled by the prim's xform
            # scale: pre-divide by the patch half-extents so st["v"] is the
            # TRUE surface speed in m/s (same normalization as make_conveyor)
            p = self.patches[i]
            self.attrs[i].Set(Gf.Vec3f(float(st["v"][0] / p["hx"]),
                                       float(st["v"][1] / p["hy"]), 0.0))

    def _tint(self, i, state):
        if state.startswith("divert:"):
            col = Gf.Vec3f(*P.ROUTE_RGBA[state.split(":")[1]])
        else:
            col = Gf.Vec3f(*FORWARD_PILL)
        for at in self.color_attrs[i]:
            at.Set([col])

    # -------------------------------------------------------------- outputs
    def summary(self):
        return {
            "mechanism": "activated_roller_belt_patch_matrix",
            "grid": [self.cfg["nx"], self.cfg["ny"]],
            "patch_size_m": [round(2 * self.patches[0]["hx"], 4),
                             round(2 * self.patches[0]["hy"], 4)],
            "actuator_commands_count": self.commands,
            "actuator_latency_ms": round(self.cfg["latency_s"] * 1000, 1),
            "ramp_mps2": self.cfg["ramp_mps2"],
            "saturation_mps": self.cfg["v_max"],
            "gain_noise_frac": self.cfg["noise_frac"],
            "max_surface_speed_mps": round(self.max_speed, 3),
            "commands_by_state": dict(sorted(self.state_counts.items())),
        }

    def write_log(self, path):
        import csv
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t", "patch", "state", "vx_cmd", "vy_cmd"])
            w.writerows(self.cmd_log)
