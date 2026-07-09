#!/usr/bin/env bash
# GATED_ACTUATOR_TEST_PLAN stages: the gate-interlock layer is instrumentation
# over the existing normally-closed exit gates, so Stage 1/2 validate the
# nominal cell WITH gate logging, and Stage 3 injects gate hardware faults
# (the cell must fail safe: watchdog -> recovery/operator call-out, no silent
# wrong delivery).  bash run_gated_stages.sh [out_root]
set -u
OUT_ROOT="${1:-/root/sortmaster_out/gated_arb}"
REPO=/root/sortmaster
mkdir -p "$OUT_ROOT" /tmp/sortmaster_signs
chmod 777 "$OUT_ROOT" /tmp/sortmaster_signs

run_one() {
  name="$1"; shift
  echo "=== $name : $* ==="
  rm -rf "$OUT_ROOT/$name"
  docker run --rm --name "isaacgate-$name" --gpus all --network=host \
    --entrypoint /isaac-sim/python.sh \
    -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
    -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v $REPO:/workspace/sortmaster:ro \
    -v /root/sortmaster_out:/workspace/sortmaster_out \
    -v /tmp/sortmaster_signs:/tmp/sortmaster_signs \
    -v /root/.cache/ov/hub:/var/cache/hub \
    -v /root/docker/isaac-sim/cache/main:/isaac-sim/.cache \
    -v /root/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache \
    -v /root/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs \
    -v /root/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config \
    -v /root/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data \
    -v /root/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg \
    nvcr.io/nvidia/isaac-sim:6.0.1 \
    /workspace/sortmaster/isaac/run_isaac.py \
    --out "/workspace/sortmaster_out/gated_arb/$name" \
    --depth-stills 0 --perception rtx --drive surface "$@" \
    > "$OUT_ROOT/$name.log" 2>&1
  echo "--- $name exit=$? ---"
  python3 - "$OUT_ROOT/$name/summary.json" <<'EOF'
import json, sys
try:
    s = json.load(open(sys.argv[1]))
    g = s.get("gate_interlocks") or {}
    print(f"  delivered={s['n_delivered']}/{s['n_items']} ok={s['n_routed_ok']} "
          f"unsafe={s['unsafe_errors']} contain={s['containment']['containment_rate']} "
          f"gate_cmds={g.get('gate_commands_count')} "
          f"open_lat={g.get('gate_open_latency_ms')}ms "
          f"travel={g.get('gate_travel_time_ms')}ms "
          f"wrong_open={g.get('wrong_gate_open_events')} "
          f"timeouts={g.get('gate_timeout_faults')} "
          f"contacts={g.get('gate_item_contact_events')}")
except Exception as e:
    print(f"  NO SUMMARY: {e}")
EOF
}

# Stage 1: one item per route
run_one stage1_bcd --seed 42 --items box_s,box_l,bottle --max-sim-s 120
# Stage 2: all official items, seed 42
run_one stage2_seed42 --seed 42 --max-sim-s 400
# Stage 3: gate hardware faults (fail-safe expected) + flow faults
run_one stage3_gateC_stuck_closed --seed 42 --items box_s,box_l,bottle \
  --inject-gate-fault C:stuck_closed --max-sim-s 180
run_one stage3_gateB_stuck_open --seed 42 --items box_s,box_l,bottle \
  --inject-gate-fault B:stuck_open --max-sim-s 120
run_one stage3_gateC_delay --seed 42 --items box_s,box_l,bottle \
  --inject-gate-fault C:delay:400 --max-sim-s 120

echo "=== GATED STAGES DONE ==="
touch "$OUT_ROOT/GATED_DONE"
