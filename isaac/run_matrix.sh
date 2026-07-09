#!/usr/bin/env bash
# Final validation matrix for the ARB-deck build (SUPER_REALISTIC plan P9).
# Runs on the 5090 host; each run is a fresh headless Isaac container.
#   bash run_matrix.sh /root/sortmaster_out/final_arb
set -u
OUT_ROOT="${1:-/root/sortmaster_out/final_arb}"
REPO=/root/sortmaster
# the isaac-sim container runs as uid 1234: it must be able to create the
# per-run output dirs inside OUT_ROOT, and dressing.py WRITES its generated
# label textures into /tmp/sortmaster_signs (the Ozon logo also lives there)
mkdir -p "$OUT_ROOT" /tmp/sortmaster_signs
chmod 777 "$OUT_ROOT" /tmp/sortmaster_signs

run_one() {
  name="$1"; shift
  echo "=== $name : $* ==="
  rm -rf "$OUT_ROOT/$name"
  docker run --rm --name "isaacrun-$name" --gpus all --network=host \
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
    --out "/workspace/sortmaster_out/final_arb/$name" \
    --depth-stills 0 --perception rtx --drive surface "$@" \
    > "$OUT_ROOT/$name.log" 2>&1
  echo "--- $name exit=$? ---"
  python3 - "$OUT_ROOT/$name/summary.json" <<'EOF'
import json, sys
try:
    s = json.load(open(sys.argv[1]))
    print(f"  delivered={s['n_delivered']}/{s['n_items']} ok={s['n_routed_ok']} "
          f"unsafe={s['unsafe_errors']} "
          f"cls={(s.get('classification') or {}).get('accuracy')} "
          f"contain={s['containment']['containment_rate']} "
          f"cmds={(s.get('arb_deck') or {}).get('actuator_commands_count')} "
          f"setv={s.get('direct_velocity_writes_nominal')} sim={s['sim_s']}s")
except Exception as e:
    print(f"  NO SUMMARY: {e}")
EOF
}

# --- six-seed nominal (66 item trials)
for SEED in 42 1 2 3 7 99; do
  run_one "seed${SEED}_nominal" --seed "$SEED" --max-sim-s 400
done
# --- robustness / fault sweeps (P3 + P6)
run_one low_friction  --seed 42 --friction-mult 0.7 --max-sim-s 400
run_one high_friction --seed 42 --friction-mult 1.3 --max-sim-s 400
run_one high_mass     --seed 42 --mass-mult 1.3 --max-sim-s 400
run_one close_spacing --seed 42 --spawn-gap 3.5,4.5 --max-sim-s 400
# 0.06 m = the physical loading envelope: at 0.10 a crosswise-yawed 435 mm
# item SPAWNS overlapping the side guide and falls outside — an artifact
# (freight cannot materialize inside a rail), probed and documented
run_one off_center    --seed 42 --spawn-offset-y 0.06 --max-sim-s 400
run_one fault_jam     --seed 42 --inject-jam box_l@8.62 --max-sim-s 420

echo "=== MATRIX DONE ==="
touch "$OUT_ROOT/MATRIX_DONE"
