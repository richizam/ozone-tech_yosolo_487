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
                row["zone_expected"] = msg["zone"]
            elif topic == "item_settled":
                row["t_settled"] = t
            elif topic == "routing_cmd":
                row["t_pick_start"] = t
            elif topic == "cell_event" and msg.get("event") == "attached":
                row["t_attached"] = t
            elif topic == "cell_event" and msg.get("event") == "released":
                row["t_released"] = t
                row["cycle_s"] = msg.get("cycle_s")
            elif topic == "item_delivered":
                row["t_delivered"] = t
                row["zone_actual"] = msg["zone"]
                row["ok"] = msg["ok"]
        return h

    def finalize(self, sim_time, extra=None):
        rows = list(self.rows.values())
        cols = ["slug", "zone_expected", "zone_actual", "ok", "t_spawn", "t_classified",
                "t_settled", "t_pick_start", "t_attached", "t_released", "t_delivered", "cycle_s"]
        with open(self.out / "events.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
            w.writeheader()
            w.writerows(rows)

        cycles = [r["cycle_s"] for r in rows if r.get("cycle_s")]
        n_ok = sum(1 for r in rows if r.get("ok"))
        summary = {
            "items": len(rows),
            "routed_correctly": n_ok,
            "routing_accuracy": round(n_ok / len(rows), 4) if rows else None,
            "cycle_mean_s": round(float(np.mean(cycles)), 2) if cycles else None,
            "cycle_p95_s": round(float(np.percentile(cycles, 95)), 2) if cycles else None,
            "cycle_max_s": round(float(np.max(cycles)), 2) if cycles else None,
            "sim_time_s": round(sim_time, 1),
            "wall_time_s": round(time.time() - self.wall_t0, 1),
            **(extra or {}),
        }
        (self.out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        return summary
