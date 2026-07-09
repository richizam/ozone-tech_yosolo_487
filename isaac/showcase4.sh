#!/usr/bin/env bash
# Final video set v4: clean gate area + industrial context + item-follow.
# Ordered so the two new/critical shots (wide context + item-follow) render
# FIRST for early visual check. Clean nominal = seed 7. bash showcase4.sh [out]
set -u
OUT_ROOT="${1:-/root/sortmaster_out/showcase4}"
REPO=/root/sortmaster
mkdir -p "$OUT_ROOT" /tmp/sortmaster_signs
chmod 777 "$OUT_ROOT" /tmp/sortmaster_signs

run() {
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
    --out "/workspace/sortmaster_out/showcase4/$name" \
    --perception rtx --drive surface --record --fps 30 "$@" \
    > "$OUT_ROOT/$name.log" 2>&1
  echo "--- $name exit=$? frames=$(ls "$OUT_ROOT/$name/frames" 2>/dev/null | wc -l) ---"
  bash /root/make_mp4.sh "$OUT_ROOT/$name" "$OUT_ROOT/$name.mp4" 30 \
    >> "$OUT_ROOT/$name.log" 2>&1 && echo "mp4 ok: $name.mp4"
}

# ---- new/critical shots FIRST (early visual check of context + follow cam)
run perfect_hero  --seed 7 --camera hero_sw --depth-stills 0 --max-sim-s 400
run item_follow_perception_to_bin --seed 7 --follow-slug pouf --camera overview --depth-stills 0 --max-sim-s 400
# ---- remaining PERFECT nominal (seed 7, no arm) static angles
run perfect_iso       --seed 7 --camera cell_iso  --depth-stills 0 --max-sim-s 400
run perfect_deckfront --seed 7 --camera deck_front --depth-stills 0 --max-sim-s 400
run perfect_decktop   --seed 7 --camera deck_top  --depth-stills 0 --max-sim-s 400
# ---- CINEMATIC camera paths (defense reel)
run cine_orbit  --seed 7 --camera overview --camera-path orbit  --path-secs 82 --depth-stills 0 --max-sim-s 400
run cine_dolly  --seed 7 --camera overview --camera-path dolly  --path-secs 82 --depth-stills 0 --max-sim-s 400
run cine_crane  --seed 7 --camera overview --camera-path crane  --path-secs 82 --depth-stills 0 --max-sim-s 400
# ---- retained TECHNICAL evidence
run route_B --seed 42 --items box_s,lunchbox,detergent --camera deck_front --depth-stills 0 --max-sim-s 150
run route_C --seed 42 --items box_l,pouf,pen           --camera deck_front --depth-stills 0 --max-sim-s 150
run route_D --seed 42 --items bottle,helmet,plate      --camera deck_front --depth-stills 0 --max-sim-s 150
run sensor  --seed 42 --items box_s,bottle,helmet --camera lookahead --depth-stills 3 --max-sim-s 200
run fault_recovery --seed 42 --camera routing --inject-jam box_s@8.05 --depth-stills 0 --max-sim-s 420
run gate_fault --seed 42 --items box_s,box_l,bottle --camera routing --inject-gate-fault C:stuck_closed --depth-stills 0 --max-sim-s 180

echo "=== SHOWCASE4 DONE ==="
ls -la "$OUT_ROOT"/*.mp4
touch "$OUT_ROOT/SHOWCASE4_DONE"
