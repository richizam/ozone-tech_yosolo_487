# -*- coding: utf-8 -*-
"""Discrete-event flow model of the cell (SimPy) — throughput & queueing.

Pipeline mirror of the physics cell:
  arrival -> feed-belt travel -> escapement gate (waits for accumulator)
  -> gate-to-wall travel + settle -> arm PICK (accumulator still occupied)
  -> accumulator freed -> arm TRANSFER+RETURN -> arm free

Timing inputs are taken from the measured physics runs (runs/*/events.csv):
  pick   = t_attached - t_pick_start
  rest   = cycle_s - pick + retract allowance
Sweeps the offered load and reports throughput, waits, queue sizes.

    python -m flow.model            # writes flow/out/sweep.csv + PNG plots
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np
import simpy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

OUT = Path(__file__).parent / "out"

TRAVEL_SPAWN_TO_GATE = 5.9      # m at 1 m/s (spawn x=0.4 -> gate 6.3)
TRAVEL_GATE_TO_WALL = 1.8       # m at 1 m/s
SETTLE = 0.4                    # s: decel creep + settle detection
RETRACT = 1.0                   # s: return to home after release
BELT_BUFFER_CAP = 14            # items the feed belt can hold upstream of the gate


def measured_phase_times():
    """Pull pick/cycle statistics from all physics runs on disk."""
    picks, cycles = [], []
    for ev in (ROOT / "runs").glob("*/events.csv"):
        summary = ev.parent / "summary.json"
        if summary.exists():
            try:
                if json.loads(summary.read_text(encoding="utf-8")).get("routing_accuracy") != 1.0:
                    continue                          # skip runs with faults
            except (json.JSONDecodeError, OSError):
                continue
        with open(ev, encoding="utf-8") as f:
            for row in csv.DictReader(f):
                try:
                    if row.get("t_attached") and row.get("t_pick_start"):
                        picks.append(float(row["t_attached"]) - float(row["t_pick_start"]))
                    if row.get("cycle_s"):
                        cycles.append(float(row["cycle_s"]))
                except ValueError:
                    continue
    if not picks or not cycles:                      # fallback to design numbers
        picks, cycles = [0.9], [2.4]
    return np.array(picks), np.array(cycles)


def run_once(rate_per_h, picks, cycles, n_items=3000, seed=0):
    rng = np.random.default_rng(seed)
    env = simpy.Environment()
    arm = simpy.Resource(env, capacity=1)
    accumulator = simpy.Resource(env, capacity=1)

    stats = {"done": 0, "waits": [], "gate_queue_max": 0, "buffer_overflow": 0}

    def item(env, t_spawn):
        yield env.timeout(TRAVEL_SPAWN_TO_GATE)
        stats["gate_queue_max"] = max(stats["gate_queue_max"], len(accumulator.queue))
        if len(accumulator.queue) >= BELT_BUFFER_CAP:
            stats["buffer_overflow"] += 1            # belt full: upstream line must stop
        acc_req = accumulator.request()
        yield acc_req
        yield env.timeout(TRAVEL_GATE_TO_WALL + SETTLE)
        arm_req = arm.request()
        yield arm_req
        pick = float(rng.choice(picks))
        yield env.timeout(pick)
        accumulator.release(acc_req)                 # item lifted: next may enter
        stats["waits"].append(env.now - t_spawn - TRAVEL_SPAWN_TO_GATE - TRAVEL_GATE_TO_WALL - SETTLE - pick)
        transfer = max(0.3, float(rng.choice(cycles)) - pick)
        yield env.timeout(transfer + RETRACT)
        arm.release(arm_req)
        stats["done"] += 1

    def generator(env):
        mean_gap = 3600.0 / rate_per_h
        for _ in range(n_items):
            yield env.timeout(rng.exponential(mean_gap))
            env.process(item(env, env.now))

    env.process(generator(env))
    env.run()
    waits = np.array(stats["waits"]) if stats["waits"] else np.array([0.0])
    makespan_h = env.now / 3600.0
    return {
        "offered_per_h": rate_per_h,
        "throughput_per_h": round(stats["done"] / makespan_h, 1),
        "wait_mean_s": round(float(waits.mean()), 2),
        "wait_p95_s": round(float(np.percentile(waits, 95)), 2),
        "gate_queue_max": stats["gate_queue_max"],
        "buffer_overflow_items": stats["buffer_overflow"],
    }


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    picks, cycles = measured_phase_times()
    print(f"measured: pick mean {picks.mean():.2f}s (n={len(picks)}), "
          f"cycle mean {cycles.mean():.2f}s p95 {np.percentile(cycles, 95):.2f}s")

    # theoretical capacity: the arm is the bottleneck (accumulator overlaps)
    svc_arm = cycles.mean() + RETRACT
    print(f"arm service ~{svc_arm:.2f}s -> capacity ~{3600 / svc_arm:.0f} items/h")

    rows = [run_once(r, picks, cycles, seed=1) for r in (300, 500, 700, 850, 1000, 1150, 1300)]
    with open(OUT / "sweep.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    for r in rows:
        print(r)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    offered = [r["offered_per_h"] for r in rows]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))
    axes[0].plot(offered, [r["throughput_per_h"] for r in rows], "o-")
    axes[0].plot(offered, offered, "--", color="gray", lw=1, label="offered = served")
    axes[0].axhline(3600 / svc_arm, color="crimson", ls=":", label=f"arm capacity ≈ {3600 / svc_arm:.0f}/h")
    axes[0].set_xlabel("offered load, items/h"); axes[0].set_ylabel("throughput, items/h"); axes[0].legend()
    axes[1].plot(offered, [r["wait_p95_s"] for r in rows], "o-", color="darkorange")
    axes[1].set_xlabel("offered load, items/h"); axes[1].set_ylabel("p95 wait at gate, s")
    axes[2].plot(offered, [r["gate_queue_max"] for r in rows], "o-", color="seagreen")
    axes[2].axhline(BELT_BUFFER_CAP, color="crimson", ls=":", label="belt buffer capacity")
    axes[2].set_xlabel("offered load, items/h"); axes[2].set_ylabel("max queue before gate"); axes[2].legend()
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.suptitle("SortMaster flow model — physics-measured service times")
    fig.tight_layout()
    fig.savefig(OUT / "flow_sweep.png", dpi=160)
    print(f"-> {OUT / 'sweep.csv'}\n-> {OUT / 'flow_sweep.png'}")

    (OUT / "params_used.json").write_text(json.dumps({
        "pick_mean_s": round(float(picks.mean()), 3),
        "cycle_mean_s": round(float(cycles.mean()), 3),
        "cycle_p95_s": round(float(np.percentile(cycles, 95)), 3),
        "arm_service_s": round(float(svc_arm), 3),
        "capacity_per_h": round(float(3600 / svc_arm), 1),
        "samples": int(len(cycles)),
    }, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
