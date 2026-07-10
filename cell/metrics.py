# -*- coding: utf-8 -*-
"""Event recording -> runs/<stamp>/{events.csv, items.csv, summary.json}.

events.csv mirrors the Isaac build's event log exactly: one row per event
(t, event, slug, data-json) with the shared executive vocabulary —
escapement_release, induction_landed (offset_m), routing_cmd, tilt_cmd,
tilt_full, discharge_confirmed, discharge_missed, double_occupancy,
end_line_callout, jam_detected, recovery_done, item_delivered, ...

items.csv is the per-item evidence table. Timing model:

    detected_time_s          item enters the measurement window (cycle start)
    sensor_capture_start_s   first sensor read (camera mode)
    route_command_time_s     fused verdict published = route command generated
    t_released_s             escapement release (synchronized induction)
    t_inducted_s             landed on its carrier (landing_offset_mm logged)
    t_tilt_cmd_s             discharge tilt command at the station
    destination_entry_time_s item crosses into its destination
    containment_confirm      item SETTLED inside the destination (C/D/REVIEW)

    perception_latency_ms = 1000 * (route_command - sensor_capture_start)
    command_margin_s      = t_tilt_cmd - route_command   (>0: the route was
                            ready before the executive needed it)
    cycle_time_s          = containment_confirm - detected_time
"""
import csv
import json
import time
from pathlib import Path

import numpy as np


def _pct(x, q):
    return round(float(np.percentile(x, q)), 2) if x else None


def _mean(x, nd=2):
    return round(float(np.mean(x)), nd) if x else None


class Metrics:
    def __init__(self, bus, out_dir):
        self.out = Path(out_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        self.bus = bus
        self.rows = {}                 # slug -> row dict
        self.wall_t0 = time.time()
        for topic in ("item_spawned", "item_detected", "item_classified",
                      "routing_cmd", "cell_event", "item_delivered",
                      "containment_final"):
            bus.subscribe(topic, self._make_handler(topic))

    def _make_handler(self, topic):
        def h(**msg):
            slug = msg.get("slug")
            if not slug:
                return                 # machine-only event (empty carrier tilt)
            row = self.rows.setdefault(slug, {"slug": slug})
            t = msg.get("t")
            if topic == "item_spawned":
                row["t_spawn"] = t
                row["asset_name"] = msg.get("asset")
                row["zone_true"] = msg.get("zone_true")
            elif topic == "item_detected":
                row["detected_time_s"] = t
            elif topic == "item_classified":
                row["route_command_time_s"] = t          # verdict = route command
                row["sensor_capture_start_s"] = msg.get("t_capture_start")
                row["sensor_capture_end_s"] = msg.get("t_capture_end")
                row["perception_latency_ms"] = msg.get("latency_ms")
                row["zone_perceived"] = msg["zone"]      # what routing follows
                row["zone_true"] = msg.get("zone_true", row.get("zone_true"))
                row["zone_raw"] = msg.get("zone_raw", msg["zone"])
                row["cls_confidence"] = msg.get("confidence")
                row["measured_max_ratio"] = msg.get("ratio")
                d = msg.get("dims_mm")
                if d:
                    srt = sorted((float(v) for v in d), reverse=True)
                    row["measured_length_mm"] = round(srt[0], 1)
                    row["measured_width_mm"] = round(srt[1], 1)
                    row["measured_height_mm"] = round(srt[2], 1)
                row["n_reads"] = msg.get("n_reads")
                row["rule_dimension_result"] = msg.get("rule_dim")
                row["rule_circularity_result"] = msg.get("rule_circ")
                row["low_confidence"] = msg.get("low_confidence", False)
                row["fallback_reason"] = msg.get("fallback_reason")
                row["cls_flags"] = msg.get("flags", "")
            elif topic == "routing_cmd":
                row["t_exec_assign"] = t                 # carrier took the route
                row["route_command"] = msg.get("zone")
                row["carrier"] = msg.get("carrier")
            elif topic == "cell_event":
                ev = msg.get("event")
                if ev == "escapement_release":
                    row["t_released_s"] = t
                elif ev == "induction_landed":
                    row["t_inducted_s"] = t
                    row["carrier"] = msg.get("carrier")
                    off = msg.get("offset_m")
                    if off is not None:
                        row["landing_offset_mm"] = round(abs(off) * 1000, 1)
                elif ev == "tilt_cmd":
                    row["t_tilt_cmd_s"] = t
                    row["station_cmd"] = msg.get("station")
                elif ev == "tilt_full":
                    row["tilt_ms"] = msg.get("tilt_ms")
                elif ev == "discharge_confirmed":
                    row["discharge_latency_ms"] = msg.get("latency_ms")
                elif ev == "discharge_missed":
                    row["review_reason"] = "discharge_miss"
                elif ev == "double_occupancy":
                    row["review_reason"] = "double_occupancy"
                elif ev == "end_line_callout":
                    row["end_line_callout"] = True
                elif ev == "jam_detected":
                    row["jam"] = True
                elif ev == "recovery_start":
                    row["arm_intervention"] = True
                    row["recovery_action"] = f"arm_to_{msg.get('target')}"
                    row["t_recovery_start"] = t
                elif ev == "recovery_done":
                    row["recovered_ok"] = msg.get("ok", True)
                    if row.get("t_recovery_start") is not None:
                        row["recovery_time_s"] = round(t - row["t_recovery_start"], 2)
                elif ev == "attached":
                    row["t_attached"] = t
                elif ev == "released":
                    row["t_released_by_arm"] = t
                    row["arm_job_cycle_s"] = msg.get("cycle_s")
                elif ev == "containment_violation":
                    row["contained"] = False
                    row["containment_violation"] = True
            elif topic == "item_delivered":
                row["destination_entry_time_s"] = t
                row["zone_actual"] = msg["zone"]
                row["ok"] = msg["ok"]
                row["v_entry"] = msg.get("v_entry")
                row["route_command"] = msg.get("route_command",
                                               row.get("route_command"))
                row["gate_hold_s"] = msg.get("gate_hold_s")
                row["hold2_hold_s"] = msg.get("hold2_hold_s")
            elif topic == "containment_final":
                row["contained"] = msg["contained"]
                row["containment_violation"] = not msg["contained"]
                row["cage_settle_s"] = msg.get("settle_s")
                row["t_containment_settle"] = msg.get("t_settle")
                row["cage_max_z"] = msg.get("max_z")
        return h

    # ------------------------------------------------------------- derivations
    @staticmethod
    def _derive(row):
        """Per-item derived fields: cycle, margin, error direction."""
        z_true, z_act = row.get("zone_true"), row.get("zone_actual")
        z_perc = row.get("zone_perceived")
        row["classification_correct"] = (None if z_perc is None or z_true is None
                                         else z_perc == z_true)
        row["destination_expected"] = z_perc
        row["destination_reached"] = z_act
        # error direction: actual==B for a non-B item feeds the sorter a bad
        # item (UNSAFE — what this cell exists to prevent); a true-B item that
        # ends anywhere safe (C/D/review/manual) is a CONSERVATIVE error
        row["unsafe_error"] = bool(z_act == "B" and z_true not in (None, "B"))
        row["conservative_error"] = bool(z_true == "B" and z_act not in (None, "B"))
        row["operator_callout"] = bool(z_act == "MANUAL")
        row["conveyor_a_stopped_for_classification"] = False   # the belt drive
        # never halts for perception; spacing holds are per-item escapements
        t_cmd = row.get("route_command_time_s")
        # command margin: was the route ready before the executive needed it?
        # (mirrors isaac/run_isaac.py: tilt command time - route command time)
        t_need = row.get("t_tilt_cmd_s")
        if t_cmd is not None and t_need is not None:
            row["command_margin_s"] = round(t_need - t_cmd, 3)
        # cycle: detection-zone entry -> containment confirmed
        t0 = row.get("detected_time_s")
        if z_act in ("C", "D", "REVIEW"):
            t1 = row.get("t_containment_settle") or row.get("destination_entry_time_s")
        else:
            t1 = row.get("destination_entry_time_s")
        row["cycle_start_s"] = t0
        row["cycle_end_s"] = round(t1, 3) if t1 is not None else None
        if t0 is not None and t1 is not None:
            row["cycle_time_s"] = round(t1 - t0, 2)
        if (row.get("destination_entry_time_s") is not None
                and row.get("t_exec_assign") is not None):
            row["transit_time_s"] = round(
                row["destination_entry_time_s"] - row["t_exec_assign"], 2)
        row["contained_at_cycle_end"] = row.get("contained")

    COLS = [
        # identity / run context
        "run_id", "seed", "scenario", "perception_mode", "executive_mode",
        "sensor_model", "slug", "asset_name",
        # classification
        "zone_true", "zone_raw", "zone_perceived", "classification_correct",
        "measured_length_mm", "measured_width_mm", "measured_height_mm",
        "measured_max_ratio", "n_reads", "cls_confidence",
        "rule_dimension_result", "rule_circularity_result",
        "low_confidence", "fallback_reason", "cls_flags",
        # executive (tilt-tray sorter)
        "route_command", "carrier", "landing_offset_mm", "station_cmd",
        "tilt_ms", "discharge_latency_ms", "review_reason",
        "destination_expected", "destination_reached",
        "zone_actual", "ok",
        "unsafe_error", "conservative_error", "operator_callout",
        "end_line_callout",
        # containment
        "contained_at_cycle_end", "containment_violation", "v_entry",
        "cage_settle_s", "cage_max_z",
        # exceptions
        "jam", "arm_intervention", "recovery_action", "recovered_ok",
        "recovery_time_s",
        # timing
        "t_spawn", "detected_time_s", "sensor_capture_start_s",
        "sensor_capture_end_s", "route_command_time_s", "t_exec_assign",
        "t_released_s", "t_inducted_s", "t_tilt_cmd_s",
        "t_attached", "t_released_by_arm",
        "destination_entry_time_s", "t_containment_settle",
        "gate_hold_s", "hold2_hold_s",
        "perception_latency_ms", "command_margin_s",
        "conveyor_a_stopped_for_classification",
        "cycle_start_s", "cycle_end_s", "cycle_time_s", "transit_time_s",
        "arm_job_cycle_s",
    ]

    def _write_event_log(self):
        """events.csv in the Isaac build's exact format: t, event, slug, data."""
        keys = ["t", "event", "slug"]
        with (self.out / "events.csv").open("w", newline="", encoding="utf-8") as f:
            wtr = csv.writer(f)
            wtr.writerow(keys + ["data"])
            for topic, msg in self.bus.history:
                event = msg.get("event", topic) if topic == "cell_event" else topic
                extra = {k: v for k, v in msg.items()
                         if k not in ("t", "slug", "event")}
                wtr.writerow([round(msg.get("t", 0.0), 4), event,
                              msg.get("slug", "")]
                             + [json.dumps(extra, ensure_ascii=False,
                                           default=str) if extra else ""])

    def finalize(self, sim_time, extra=None):
        extra = dict(extra or {})
        run_const = {k: extra.get(k) for k in
                     ("run_id", "seed", "scenario", "perception_mode",
                      "executive_mode", "sensor_model")}
        rows = list(self.rows.values())
        for r in rows:
            self._derive(r)
            r.update(run_const)
        self._write_event_log()
        with open(self.out / "items.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=self.COLS, extrasaction="ignore")
            w.writeheader()
            w.writerows(rows)

        cycles = [r["cycle_time_s"] for r in rows if r.get("cycle_time_s") is not None]
        transits = [r["transit_time_s"] for r in rows if r.get("transit_time_s") is not None]
        margins = [r["command_margin_s"] for r in rows if r.get("command_margin_s") is not None]
        latencies = [r["perception_latency_ms"] for r in rows
                     if r.get("perception_latency_ms") is not None]
        n_ok = sum(1 for r in rows if r.get("ok"))
        classified = [r for r in rows if r.get("zone_perceived")]
        cls_ok = sum(1 for r in classified if r["zone_perceived"] == r.get("zone_true"))
        unsafe = sum(1 for r in rows if r.get("unsafe_error"))
        conservative = sum(1 for r in rows if r.get("conservative_error"))
        # containment: a C/D/REVIEW delivery only counts as clean if the item
        # stayed inside its destination for the rest of the run
        caged = [r for r in rows if r.get("contained") is not None]
        contained = sum(1 for r in caged if r["contained"])
        v_entries = [r["v_entry"] for r in caged if r.get("v_entry") is not None]
        settles = [r["cage_settle_s"] for r in caged if r.get("cage_settle_s") is not None]
        max_zs = [r["cage_max_z"] for r in caged if r.get("cage_max_z") is not None]
        # exception handling + throughput
        jams = sum(1 for r in rows if r.get("jam"))
        interventions = sum(1 for r in rows if r.get("arm_intervention"))
        recoveries_ok = sum(1 for r in rows if r.get("recovered_ok"))
        rec_times = [r["recovery_time_s"] for r in rows
                     if r.get("recovery_time_s") is not None]
        callouts = sum(1 for r in rows if r.get("operator_callout"))
        review_deliveries = sum(1 for r in rows if r.get("zone_actual") == "REVIEW")
        merged_rejected = sum(1 for r in rows
                              if "unstable_reads_rejected" in (r.get("cls_flags") or ""))
        low_conf_fallbacks = sum(1 for r in rows if r.get("fallback_reason"))
        t_del = sorted(r["destination_entry_time_s"] for r in rows
                       if r.get("destination_entry_time_s") is not None)
        throughput = (3600.0 * (len(t_del) - 1) / (t_del[-1] - t_del[0])
                      if len(t_del) >= 2 and t_del[-1] > t_del[0] else None)
        routing_acc = round(n_ok / len(rows), 4) if rows else None
        summary = {
            "items": len(rows),
            "routed_correctly": n_ok,
            "routing_accuracy": routing_acc,
            "end_to_end_accuracy": routing_acc,
            "classification_accuracy": round(cls_ok / len(classified), 4) if classified else None,
            "unsafe_errors": unsafe,
            "conservative_errors": conservative,
            # ---- isaac-shaped nested vocabulary (mirrors isaac/run_isaac.py)
            "containment": {
                "tracked": len(caged),
                "violations": len(caged) - contained,
                "containment_rate": (round(contained / len(caged), 4)
                                     if caged else None),
                "cage_entry_speed_max_mps": (round(float(np.max(v_entries)), 3)
                                             if v_entries else None),
                "cage_max_z": (round(float(np.max(max_zs)), 3)
                               if max_zs else None),
            },
            "cycle_s": {"mean": _mean(cycles, 3),
                        "p95": _pct(cycles, 95),
                        "max": round(float(np.max(cycles)), 3) if cycles else None},
            "command_margin_s": {
                "min": round(float(np.min(margins)), 3) if margins else None,
                "mean": _mean(margins, 3)},
            "review_deliveries": review_deliveries,
            "manual_callouts": callouts,
            # ---- flat legacy keys (validation matrix / dashboards)
            "cage_deliveries": len(caged),
            "containment_rate": round(contained / len(caged), 4) if caged else None,
            "containment_violations": len(caged) - contained,
            "cage_entry_speed_mean_mps": _mean(v_entries),
            "cage_entry_speed_max_mps": round(float(np.max(v_entries)), 2) if v_entries else None,
            "cage_settle_mean_s": _mean(settles),
            "jam_detected_count": jams,
            "arm_interventions": interventions,
            "recovery_success": round(recoveries_ok / interventions, 3) if interventions else None,
            "average_recovery_time_s": _mean(rec_times),
            "operator_callouts": callouts,
            "throughput_items_per_h": round(throughput, 1) if throughput else None,
            "transit_mean_s": _mean(transits),
            "transit_p95_s": _pct(transits, 95),
            "cycle_mean_s": _mean(cycles),
            "cycle_p95_s": _pct(cycles, 95),
            "cycle_max_s": round(float(np.max(cycles)), 2) if cycles else None,
            "cycle_min_s": round(float(np.min(cycles)), 2) if cycles else None,
            "perception_latency_ms_mean": _mean(latencies, 1),
            "perception_latency_ms_p95": _pct(latencies, 95),
            "command_margin_s_min": round(float(np.min(margins)), 3) if margins else None,
            "command_margin_s_mean": _mean(margins, 3),
            "items_with_negative_command_margin": sum(1 for m in margins if m <= 0),
            "merged_detection_rejected_count": merged_rejected,
            "low_confidence_fallback_count": low_conf_fallbacks,
            "sim_time_s": round(sim_time, 1),
            "wall_time_s": round(time.time() - self.wall_t0, 1),
            **extra,
        }
        (self.out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        with open(self.out / "events_raw.jsonl", "w", encoding="utf-8") as f:
            for topic, msg in self.bus.history:
                f.write(json.dumps({"topic": topic, **{k: v for k, v in msg.items()
                                                       if k != "dims_mm"}}, default=str) + "\n")
        return summary
