#!/usr/bin/env bash
# Final video evidence (SUPER_REALISTIC plan P7): overview, ARB-deck
# close-up, sensor station, fault recovery. Run AFTER the validation matrix
# (never two Kit instances on one GPU).  bash showcase.sh [out_root]
set -u
OUT_ROOT="${1:-/root/sortmaster_out/showcase_arb}"
REPO=/root/sortmaster
mkdir -p "$OUT_ROOT" /tmp/sortmaster_signs
chmod 777 "$OUT_ROOT" /tmp/sortmaster_signs

run_rec() {
  name="$1"; shift
  echo "=== $name : $* ==="
  rm -rf "$OUT_ROOT/$name"
  docker run --rm --name "isaacrec-$name" --gpus all --network=host \
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
    --out "/workspace/sortmaster_out/showcase_arb/$name" \
    --perception rtx --drive surface --record --fps 30 "$@" \
    > "$OUT_ROOT/$name.log" 2>&1
  echo "--- $name exit=$? frames=$(ls "$OUT_ROOT/$name/frames" 2>/dev/null | wc -l) ---"
  bash /root/make_mp4.sh "$OUT_ROOT/$name" "$OUT_ROOT/$name.mp4" 30 \
    >> "$OUT_ROOT/$name.log" 2>&1 && echo "mp4 ok: $name.mp4"
}

# 1. industrial overview: whole cell, complete flow, all 11 items
run_rec nominal_overview  --seed 42 --camera overview --depth-stills 3 --max-sim-s 400
# 2. ARB transfer-deck close-up: patches activating, gates, physical contact
run_rec deck_closeup      --seed 42 --camera routing --depth-stills 0 --max-sim-s 400
# 3. sensor station: items passing the RTX heads (B/C/D mix)
run_rec sensor_station    --seed 42 --items box_s,bottle,helmet --camera lookahead --depth-stills 3 --max-sim-s 200
# 4. fault recovery: jam -> watchdog -> jam camera -> UR10e recovery
run_rec fault_recovery    --seed 42 --camera routing --inject-jam box_s@8.05 --depth-stills 0 --max-sim-s 420

echo "=== SHOWCASE DONE ==="
ls -la "$OUT_ROOT"/*.mp4
touch "$OUT_ROOT/SHOWCASE_DONE"
