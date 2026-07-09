#!/usr/bin/env bash
# Video set v2 (user directive): clean nominal evidence FIRST, then faults.
#   1 nominal_full      whole cell, all 11 items, no arm, no stalls
#   2 close_route_B/C/D clean ARB actuation per route, deck close-up
#   3 nominal_sensor    vision-station pass + depth stills (perception demo
#                       panels composed offline from stills + reads_log)
#   4 fault_recovery    injected snag -> watchdog -> jam cam -> UR10e
#   5 gate_fault        C gate stuck closed -> timeout -> safe call-out
# bash showcase2.sh [out_root]
set -u
OUT_ROOT="${1:-/root/sortmaster_out/showcase2}"
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
    --out "/workspace/sortmaster_out/showcase2/$name" \
    --perception rtx --drive surface --record --fps 30 "$@" \
    > "$OUT_ROOT/$name.log" 2>&1
  echo "--- $name exit=$? frames=$(ls "$OUT_ROOT/$name/frames" 2>/dev/null | wc -l) ---"
  bash /root/make_mp4.sh "$OUT_ROOT/$name" "$OUT_ROOT/$name.mp4" 30 \
    >> "$OUT_ROOT/$name.log" 2>&1 && echo "mp4 ok: $name.mp4"
}

# --- clean nominal evidence first (the primary story)
run_rec nominal_full   --seed 42 --camera overview --depth-stills 4 --max-sim-s 400
run_rec close_route_B  --seed 42 --items box_s,lunchbox,detergent --camera routing --depth-stills 0 --max-sim-s 150
run_rec close_route_C  --seed 42 --items box_l,pouf,pen           --camera routing --depth-stills 0 --max-sim-s 150
run_rec close_route_D  --seed 42 --items bottle,helmet,plate      --camera routing --depth-stills 0 --max-sim-s 150
run_rec nominal_sensor --seed 42 --items box_s,bottle,helmet --camera lookahead --depth-stills 3 --max-sim-s 200
# --- fault handling exists, but is not required in normal flow
run_rec fault_recovery --seed 42 --camera routing --inject-jam box_s@8.05 --depth-stills 0 --max-sim-s 420
run_rec gate_fault     --seed 42 --items box_s,box_l,bottle --camera routing --inject-gate-fault C:stuck_closed --depth-stills 0 --max-sim-s 180

echo "=== SHOWCASE2 DONE ==="
ls -la "$OUT_ROOT"/*.mp4
touch "$OUT_ROOT/SHOWCASE2_DONE"
