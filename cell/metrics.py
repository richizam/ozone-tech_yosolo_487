# -*- coding: utf-8 -*-
"""Event recording -> runs/<stamp>/events.csv + summary.json."""
import csv
import json
import time
from pathlib import Path

import numpy as np


class Metrics:
    def __init__(self, bus, out_dir):
        self.out = Path(out_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        self.bus = bus
        self.rows = {}                 # slug -> row dict
        self.wall_t0 = time.time()
        for topic in ("item_spawned", "item_classified", "item_settled",
                      "routing_cmd", "cell_event", "item_delivered"):
            bus.subscribe(topic, self._make_handler(topic))

    def _make_handler(self, topic):
        def h(**msg):
            slug = msg.get("slug")
            row = self.rows.setdefault(slug, {"slug": slug})
            t = msg.get("t")
            if topic == "item_spawned":
                row["t_spawn"] = t
            elif topic == "item_classified":
                row["t_classified"] = t
                row["zone_perceived"] = msg["zone"]          # what routing follows
                row["zone_true"] = msg.get("zone_true", msg["zone"])
                row["zone_raw"] = msg.get("zone_raw", msg["zone"])
                row["cls_confidence"] = msg.get("confidence")
                row["cls_flags"] = msg.get("flags", "")
            elif topic == "item_settled":
                row["t_settled"] = t
            elif topic == "routing_cmd":
                row["t_pick_start"] = t
            elif topic == "cell_event" and msg.get("event") == "attached":
                row["t_attached"] = t
            elif topic == "cell_event" and msg.get("event") == "released":
                row["t_released"] = t
                row["cycle_s"] = msg.get("cycle_s")
            elif topic == "cell_event" and msg.get("event") == "jam_detected":
                row["jam"] = True
            elif topic == "cell_event" and msg.get("event") == "recovery_done":
                row["recovered_ok"] = msg.get("ok")
            elif topic == "item_delivered":
                row["t_delivered"] = t
                row["zone_actual"] = msg["zone"]
                row["ok"] = msg["ok"]
        return h

    def finalize(self, sim_time, extra=None):
        rows = list(self.rows.values())
        cols = ["slug", "zone_true", "zone_raw", "zone_perceived", "zone_actual", "ok",
                "cls_confidence", "cls_flags", "jam", "recovered_ok", "t_spawn", "t_classified",
                "t_settled", "t_pick_start", "t_attached", "t_released", "t_delivered", "cycle_s"]
        with open(self.out / "events.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
            w.writeheader()
            w.writerows(rows)

        cycles = [r["cycle_s"] for r in rows if r.get("cycle_s")]
        n_ok = sum(1 for r in rows if r.get("ok"))
        classified = [r for r in rows if r.get("zone_perceived")]
        cls_ok = sum(1 for r in classified if r["zone_perceived"] == r.get("zone_true"))
        delivered = [r for r in rows if r.get("zone_actual") and r.get("zone_perceived")]
        exec_ok = sum(1 for r in delivered if r["zone_actual"] == r["zone_perceived"])
        # error direction: B->C/D only diverts a sortable item to inspection
        # (conservative); C/D->B feeds a bad item to the sorter (unsafe — the
        # failure this cell exists to prevent)
        errs = [r for r in rows if r.get("zone_actual") and r["zone_actual"] != r.get("zone_true")]
        unsafe = sum(1 for r in errs if r["zone_actual"] == "B")
        conservative = sum(1 for r in errs if r.get("zone_true") == "B" and r["zone_actual"] in ("C", "D"))
        # exception handling + throughput (transfer-table architecture)
        jams = sum(1 for r in rows if r.get("jam"))
        recovered = sum(1 for r in rows if r.get("jam") and r.get("zone_actual") in ("C", "D"))
        t_del = sorted(r["t_delivered"] for r in rows if r.get("t_delivered") is not None)
        throughput = (3600.0 * (len(t_del) - 1) / (t_del[-1] - t_del[0])
                      if len(t_del) >= 2 and t_del[-1] > t_del[0] else None)
        summary = {
            "items": len(rows),
            "routed_correctly": n_ok,
            "routing_accuracy": round(n_ok / len(rows), 4) if rows else None,
            "classification_accuracy": round(cls_ok / len(classified), 4) if classified else None,
            "executive_accuracy": round(exec_ok / len(delivered), 4) if delivered else None,
            "unsafe_errors": unsafe,
            "conservative_errors": conservative,
            "arm_interventions": jams,
            "recovery_success": round(recovered / jams, 3) if jams else None,
            "routing_no_intervention": round((len(delivered) - jams) / len(delivered), 4) if delivered else None,
            "throughput_items_per_h": round(throughput, 1) if throughput else None,
            "cycle_mean_s": round(float(np.mean(cycles)), 2) if cycles else None,
            "cycle_p95_s": round(float(np.percentile(cycles, 95)), 2) if cycles else None,
            "cycle_max_s": round(float(np.max(cycles)), 2) if cycles else None,
            "sim_time_s": round(sim_time, 1),
            "wall_time_s": round(time.time() - self.wall_t0, 1),
            **(extra or {}),
        }
        (self.out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        with open(self.out / "events_raw.jsonl", "w", encoding="utf-8") as f:
            for topic, msg in self.bus.history:
                f.write(json.dumps({"topic": topic, **{k: v for k, v in msg.items()
                                                       if k != "dims_mm"}}, default=str) + "\n")
        return summary
