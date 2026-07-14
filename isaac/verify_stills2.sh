#!/usr/bin/env bash
# Robust dressed-stills pass (full cosmetic dressing, NO SM_LEAN). Same intent
# as verify_stills.sh, hardened against the sporadic Isaac RTX headless flake
# ("Waiting on Semaphore ... ERROR_OUT_OF_HOST_MEMORY" — a driver-state hiccup,
# NOT a real host-RAM shortage: the box has 95 GB free). Each shot runs under a
# hard `timeout`; on no-frame it is retried ONCE in a fresh container (a new
# process clears the leaked Vulkan state). A stuck shot can no longer block the
# whole set the way the first run hung on b_transfer.
#   bash verify_stills2.sh [out]
set -u
OUT_ROOT="${1:-/root/sortmaster_out/stills_dressed2}"
REPO=/root/sortmaster
mkdir -p "$OUT_ROOT" /tmp/sortmaster_signs
chmod 777 "$OUT_ROOT" 2>/dev/null || true
OUT_ROOT_IN="/workspace/sortmaster_out/$(basename "$OUT_ROOT")"
PER_SHOT_TIMEOUT="${PER_SHOT_TIMEOUT:-600}"

_run_once() {   # name cam extra-args...
  name="$1"; cam="$2"; shift 2
  docker rm -f "isaacdress-$name" >/dev/null 2>&1 || true
  timeout --signal=KILL "$PER_SHOT_TIMEOUT" \
  docker run --rm --name "isaacdress-$name" --gpus all --network=host \
    --ulimit nofile=1048576:1048576 \
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
    --out "$OUT_ROOT_IN/$name" --record --fps 8 \
    --perception rtx --drive surface --depth-stills 0 \
    --camera "$cam" --max-sim-s 16 "$@" \
    >> "$OUT_ROOT/$name.log" 2>&1
  docker rm -f "isaacdress-$name" >/dev/null 2>&1 || true
}

shot() {   # name cam extra-args...
  name="$1"; cam="$2"; shift 2
  echo "=== dressed still $name ($cam) ==="
  rm -rf "$OUT_ROOT/$name" "$OUT_ROOT/$name.png"
  : > "$OUT_ROOT/$name.log"
  for attempt in 1 2; do
    echo "--- $name attempt $attempt ---" >> "$OUT_ROOT/$name.log"
    _run_once "$name" "$cam" "$@"
    last=$(ls "$OUT_ROOT/$name/frames" 2>/dev/null | tail -1)
    if [ -n "$last" ]; then
      cp "$OUT_ROOT/$name/frames/$last" "$OUT_ROOT/$name.png"
      echo "png ok: $name.png (attempt $attempt)"
      return 0
    fi
    echo "NO FRAME ($name) attempt $attempt — settling before retry"
    sleep 10
  done
  echo "NO FRAME ($name) — FAILED after 2 attempts, see $name.log"
  return 1
}

# curated variety: box + round + soft + oversize + edge — dressed
CURATED="box_l,bottle,helmet,pouf,pen"

shot hero_ne      hero_ne      --seed 7  --items "$CURATED"
sleep 8
shot cell_iso     cell_iso     --seed 7  --items "$CURATED"          # WIDE — floor check
sleep 8
shot b_transfer   b_transfer   --seed 42 --items box_s,lunchbox
sleep 8
shot cage_closeup deck_front   --seed 7  --items "$CURATED"
sleep 8
shot mech_c_side  mech_c_side  --seed 42 --items box_l,pen

echo "=== DRESSED STILLS DONE ==="
touch "$OUT_ROOT/DRESSED_STILLS_DONE"
