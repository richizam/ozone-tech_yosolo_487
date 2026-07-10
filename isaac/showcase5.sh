#!/usr/bin/env bash
# Final video set v5 — TILT-TRAY sorter build.
# Clean nominal seed picked from the matrix (0 jams). bash showcase5.sh [out] [seed]
set -u
OUT_ROOT="${1:-/root/sortmaster_out/showcase5}"
CLEAN_SEED="${2:-7}"
REPO=/root/sortmaster
mkdir -p "$OUT_ROOT" /tmp/sortmaster_signs
chmod 777 "$OUT_ROOT" /tmp/sortmaster_signs
OUT_ROOT_IN="/workspace/sortmaster_out/$(basename "$OUT_ROOT")"

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
    --out "$OUT_ROOT_IN/$name" \
    --perception rtx --drive surface --record --fps 30 "$@" \
    > "$OUT_ROOT/$name.log" 2>&1
  echo "--- $name exit=$? frames=$(ls "$OUT_ROOT/$name/frames" 2>/dev/null | wc -l) ---"
  bash /root/make_mp4.sh "$OUT_ROOT/$name" "$OUT_ROOT/$name.mp4" 30 \
    >> "$OUT_ROOT/$name.log" 2>&1 && echo "mp4 ok: $name.mp4"
}

# ---- hero + item-follow FIRST (early visual check of the new machine)
run perfect_hero  --seed "$CLEAN_SEED" --camera hero_sw --depth-stills 0 --max-sim-s 400
run item_follow_perception_to_bin --seed "$CLEAN_SEED" --follow-slug pouf --camera overview --depth-stills 0 --max-sim-s 400
# ---- remaining PERFECT nominal static angles
run perfect_iso       --seed "$CLEAN_SEED" --camera cell_iso   --depth-stills 0 --max-sim-s 400
run perfect_deckfront --seed "$CLEAN_SEED" --camera deck_front --depth-stills 0 --max-sim-s 400
run perfect_decktop   --seed "$CLEAN_SEED" --camera deck_top   --depth-stills 0 --max-sim-s 400
# ---- CINEMATIC camera paths (defense reel)
run cine_orbit  --seed "$CLEAN_SEED" --camera overview --camera-path orbit  --path-secs 82 --depth-stills 0 --max-sim-s 400
run cine_dolly  --seed "$CLEAN_SEED" --camera overview --camera-path dolly  --path-secs 82 --depth-stills 0 --max-sim-s 400
run cine_crane  --seed "$CLEAN_SEED" --camera overview --camera-path crane  --path-secs 82 --depth-stills 0 --max-sim-s 400
# ---- TECHNICAL evidence clips
run route_B --seed 42 --items box_s,lunchbox,detergent --camera deck_front --depth-stills 0 --max-sim-s 200
run route_C --seed 42 --items box_l,pouf,pen           --camera routing    --depth-stills 0 --max-sim-s 200
run route_D --seed 42 --items bottle,helmet,plate      --camera routing    --depth-stills 0 --max-sim-s 200
# the HEADLINE: the 11 mm cube (and friends) carried and discharged to B
run edge_small_items --seed 42 --manifest-extra \
  --items edge_cube11,edge_cube10,edge_rod9,pen --camera deck_front \
  --depth-stills 4 --max-sim-s 260
run sensor  --seed 42 --items box_s,bottle,helmet,edge_cube11 --manifest-extra --camera lookahead --depth-stills 4 --max-sim-s 260
# ---- fault / recovery drills on camera
run fault_recovery --seed 42 --camera routing --inject-jam box_l@2.62 --depth-stills 0 --max-sim-s 430
run fault_tray     --seed 42 --camera deck_front --inject-tray-fault D --depth-stills 0 --max-sim-s 430

echo "=== SHOWCASE5 DONE ==="
ls -la "$OUT_ROOT"/*.mp4
touch "$OUT_ROOT/SHOWCASE5_DONE"
