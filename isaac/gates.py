# -*- coding: utf-8 -*-
"""Gate-interlock instrumentation + fault injection
(GATED_ACTUATOR_TEST_PLAN.md).

The cell's three exit gates are normally-closed vertical-lift panels on
prismatic drives (built by scene_usd.build_gate) — the physical interlock
has been part of the validated build from the start: an item can only leave
the routing table through the gate its route command opened. This module
adds what the test plan asks for on top of the SAME hardware:

  * an explicit per-gate state machine (closed / opening / open / closing)
    derived from the MEASURED joint position, not the command;
  * logged transitions (gate_opening/gate_open/gate_closing/gate_closed),
    command→fully-open latency and measured travel time;
  * interlock violation counters (wrong_gate_open_events — a gate off its
    seat without an open intent);
  * gate fault injection: stuck_closed, stuck_open, delay:<ms> — the
    CONTROLLER intent is still logged, the hardware misbehaves, and the
    cell must fail safe (watchdog → recovery/operator call-out).
"""
import numpy as np

OPEN_FRAC = 0.92                 # measured fraction of travel = fully open
CLOSED_FRAC = 0.05
TIMEOUT_S = 2.0                  # commanded open but not open in time


class GateInterlocks:
    def __init__(self, target_attr, gate_rp, z0, travel, ev, inject=None):
        """target_attr/gate_rp/z0: per-zone drive attr, body wrapper and
        closed-state body height; travel: lift stroke (m); ev: event logger;
        inject: {"zone", "mode": stuck_closed|stuck_open|delay,
        "delay_s": float}."""
        self.attr = target_attr
        self.rp = gate_rp
        self.z0 = z0
        self.travel = float(travel)
        self.ev = ev
        self.inject = inject or {}
        zones = list(target_attr)
        self.state = {z: "closed" for z in zones}
        self.intent = {z: False for z in zones}
        self.pending = {z: None for z in zones}     # (t_eff, want) delay fault
        self.t_open_cmd = {z: None for z in zones}
        self.t_close_cmd = {z: None for z in zones}
        self.t_opening = {z: None for z in zones}
        self.timeout_flag = {z: False for z in zones}
        self.metrics = {"gate_commands_count": 0,
                        "wrong_gate_open_events": 0,
                        "gate_timeout_faults": 0,
                        "gate_item_contact_events": 0}
        self.open_latency_s = []
        self.travel_time_s = []

    # ------------------------------------------------------------ commands
    def command(self, t, want):
        """Controller intent per zone. Faults act between intent and
        hardware: the intent is logged either way."""
        for z, w in want.items():
            w = bool(w)
            if w != self.intent[z]:
                self.intent[z] = w
                self.metrics["gate_commands_count"] += 1
                self.ev(t, "gate_commanded", "", gate=z,
                        target="open" if w else "closed")
                self.timeout_flag[z] = False
                if w:
                    self.t_open_cmd[z] = t
                else:
                    self.t_close_cmd[z] = t
                inj = self.inject if self.inject.get("zone") == z else None
                if inj and inj["mode"] == "delay":
                    self.pending[z] = (t + inj["delay_s"], w)
                    continue
                self._apply(z, w)

    def _apply(self, z, w):
        inj = self.inject if self.inject.get("zone") == z else None
        if inj and inj["mode"] == "stuck_closed":
            w = False                       # hardware never lifts
        elif inj and inj["mode"] == "stuck_open":
            w = True                        # hardware never seats
        self.attr[z].Set(self.travel if w else 0.0)

    # ---------------------------------------------------------------- step
    def step(self, t):
        for z in self.state:
            if self.pending[z] is not None and t >= self.pending[z][0]:
                self._apply(z, self.pending[z][1])
                self.pending[z] = None
            pz = float(np.asarray(self.rp[z].get_world_pose()[0])[2])
            frac = (pz - self.z0[z]) / self.travel
            prev = self.state[z]
            if frac >= OPEN_FRAC:
                cur = "open"
            elif frac <= CLOSED_FRAC:
                cur = "closed"
            else:
                cur = "opening" if self.intent[z] else "closing"
            if cur != prev:
                self.state[z] = cur
                self.ev(t, f"gate_{cur}", "", gate=z,
                        frac=round(float(frac), 3))
                if cur == "opening":
                    self.t_opening[z] = t
                elif cur == "open":
                    if self.t_open_cmd[z] is not None:
                        self.open_latency_s.append(t - self.t_open_cmd[z])
                    if self.t_opening[z] is not None:
                        self.travel_time_s.append(t - self.t_opening[z])
                # interlock violation: off the seat with no open intent
                if prev == "closed" and cur != "closed" and not self.intent[z]:
                    self.metrics["wrong_gate_open_events"] += 1
                    self.ev(t, "wrong_gate_open", "", gate=z)
            # stuck-open flavour of the same violation: a CLOSE was commanded
            # but the panel stays fully open past the timeout (measured from
            # the close command — measuring from the open command flagged
            # every normal transit longer than the timeout)
            if (not self.intent[z] and cur == "open"
                    and not self.timeout_flag[z]
                    and self.t_close_cmd[z] is not None
                    and t - self.t_close_cmd[z] > TIMEOUT_S):
                self.timeout_flag[z] = True
                self.metrics["wrong_gate_open_events"] += 1
                self.ev(t, "wrong_gate_open", "", gate=z, mode="stuck_open")
            # commanded open but not open in time -> hardware fault
            if (self.intent[z] and cur != "open"
                    and not self.timeout_flag[z]
                    and self.t_open_cmd[z] is not None
                    and t - self.t_open_cmd[z] > TIMEOUT_S):
                self.timeout_flag[z] = True
                self.metrics["gate_timeout_faults"] += 1
                self.ev(t, "gate_timeout", "", gate=z,
                        commanded_s=round(t - self.t_open_cmd[z], 2))

    # ------------------------------------------------------------- summary
    def summary(self):
        lat = [v for v in self.open_latency_s]
        trv = [v for v in self.travel_time_s]
        return {
            "enabled": True,
            "mechanism": "normally_closed_vertical_lift_gates",
            "injection": self.inject or None,
            "gate_commands_count": self.metrics["gate_commands_count"],
            "gate_open_latency_ms": (round(1000 * float(np.mean(lat)), 1)
                                     if lat else None),
            "gate_travel_time_ms": (round(1000 * float(np.mean(trv)), 1)
                                    if trv else None),
            "wrong_gate_open_events": self.metrics["wrong_gate_open_events"],
            "gate_timeout_faults": self.metrics["gate_timeout_faults"],
            "gate_item_contact_events":
                self.metrics["gate_item_contact_events"],
        }
