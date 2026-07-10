#!/usr/bin/env bash
# Re-run the two edge-set runs into an existing matrix dir (v13 controller:
# length-aware discharge confirm). Everything else in the matrix is
# decision-identical under the diff (the z-line only matters when the side
# confirm has not fired within the stuck-tilt window).
#   bash rerun_edges.sh /root/sortmaster_out/xbelt_matrix
set -u
OUT_ROOT="${1:-/root/sortmaster_out/xbelt_matrix}"
REPO=/root/sortmaster
OUT_ROOT_IN="/workspace/sortmaster_out/$(basename "$OUT_ROOT")"
mkdir -p "$OUT_ROOT" /tmp/sortmaster_signs

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
    --out "$OUT_ROOT_IN/$name" \
    --depth-stills 0 --perception rtx --drive surface "$@" \
    > "$OUT_ROOT/$name.log" 2>&1
  echo "--- $name exit=$? ---"
  python3 - "$OUT_ROOT/$name/summary.json" <<'EOF'
import json, sys
try:
    s = json.load(open(sys.argv[1]))
    srt = s.get("sorter") or {}
    m = s.get("command_margin_s") or {}
    print(f"  delivered={s['n_delivered']}/{s['n_items']} ok={s['n_routed_ok']} "
          f"unsafe={s['unsafe_errors']} floor={s.get('floor_drops')} "
          f"cls={(s.get('classification') or {}).get('accuracy')} "
          f"contain={s['containment']['containment_rate']} "
          f"margin_min={m.get('min')} "
          f"cmds={srt.get('carrier_commands_count')} "
          f"land_max={srt.get('landing_offset_max_mm')}mm "
          f"setv={s.get('direct_velocity_writes_nominal')} sim={s['sim_s']}s")
except Exception as e:
    print(f"  NO SUMMARY: {e}")
EOF
}

run_one edge_items_all --seed 42 --manifest-extra --max-sim-s 700
run_one edge_small     --seed 42 --manifest-extra \
  --items edge_cube11,edge_cube10,edge_rod9,edge_card2,pen,bl_rod_b \
  --max-sim-s 320

echo "=== EDGES DONE ==="
touch "$OUT_ROOT/EDGES_DONE"
