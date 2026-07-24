#!/usr/bin/env bash
# Final video set v5 — TILT-TRAY sorter build — HARDENED runner.
# Identical clip list / cameras / seeds to showcase5.sh, but every clip runs
# under a hard `timeout` and is retried ONCE in a fresh container on a no-frame
# result. This defends the long (~2-3 h) unattended render against the sporadic
# Isaac RTX headless flake ("Waiting on Semaphore ... ERROR_OUT_OF_HOST_MEMORY")
# that hung a stills shot mid-render — a driver-state hiccup, not a real OOM
# (the box has 95 GB RAM free). One hung clip can no longer stall the whole set.
#   bash showcase5b.sh [out] [seed]
set -u
OUT_ROOT="${1:-/root/sortmaster_out/showcase5}"
CLEAN_SEED="${2:-7}"
REPO=/root/sortmaster
mkdir -p "$OUT_ROOT" /tmp/sortmaster_signs
chmod 777 "$OUT_ROOT" /tmp/sortmaster_signs
OUT_ROOT_IN="/workspace/sortmaster_out/$(basename "$OUT_ROOT")"
PER_CLIP_TIMEOUT="${PER_CLIP_TIMEOUT:-2400}"   # 40 min hard ceiling per clip

_render_once() {   # name extra-args...
  name="$1"; shift
  docker rm -f "isaacrec-$name" >/dev/null 2>&1 || true
  timeout --signal=KILL "$PER_CLIP_TIMEOUT" \
  docker run --rm --name "isaacrec-$name" --gpus all --network=host \
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
    --out "$OUT_ROOT_IN/$name" \
    --perception rtx --drive surface --record --fps 30 "$@" \
    >> "$OUT_ROOT/$name.log" 2>&1
  docker rm -f "isaacrec-$name" >/dev/null 2>&1 || true
}

run() {
  name="$1"; shift
  # Resume guard: a finished clip already has a non-empty mp4. Skip it so a
  # re-launch after a crash/kill (or a pre-rendered probe clip) is not redone.
  if [ -s "$OUT_ROOT/$name.mp4" ]; then
    echo "=== $name : SKIP (mp4 already exists) ==="; return
  fi
  echo "=== $name : $* ==="
  rm -rf "$OUT_ROOT/$name" "$OUT_ROOT/$name.mp4"
  : > "$OUT_ROOT/$name.log"
  ok=0
  for attempt in 1 2; do
    echo "--- $name attempt $attempt ---" >> "$OUT_ROOT/$name.log"
    _render_once "$name" "$@"
    nf=$(ls "$OUT_ROOT/$name/frames" 2>/dev/null | wc -l)
    echo "--- $name attempt $attempt frames=$nf ---"
    if [ "$nf" -gt 0 ]; then ok=1; break; fi
    echo "no frames — settling before retry"; sleep 12
  done
  if [ "$ok" = 1 ]; then
    bash /root/make_mp4.sh "$OUT_ROOT/$name" "$OUT_ROOT/$name.mp4" 30 \
      >> "$OUT_ROOT/$name.log" 2>&1 && echo "mp4 ok: $name.mp4" \
      || echo "MP4 FAIL: $name (frames exist, see log)"
  else
    echo "CLIP FAILED after 2 attempts: $name"
  fi
}

# PRIORITY REORDER (final window): TECHNICAL evidence first, beauty last —
# the tail is the sacrificial end if the clock wins. Resume guard skips
# any clip whose mp4 already exists (hero renders once, ever).
run perfect_hero  --seed "$CLEAN_SEED" --camera hero_ne --depth-stills 0 --max-sim-s 400
run mech_tilt_C   --seed 42 --items box_l,pen --camera mech_c_side --depth-stills 0 --max-sim-s 200
run b_transfer    --seed 42 --items box_s,lunchbox --camera b_transfer --depth-stills 0 --max-sim-s 200
run route_B --seed 42 --items box_s,lunchbox,detergent --camera deck_front --depth-stills 0 --max-sim-s 200
run route_C --seed 42 --items box_l,pouf,pen           --camera routing    --depth-stills 0 --max-sim-s 200
run route_D --seed 42 --items bottle,helmet,plate      --camera routing    --depth-stills 0 --max-sim-s 200
run edge_small_items --seed 42 --manifest-extra   --items edge_cube11,edge_cube10,edge_rod9,pen --camera deck_front   --depth-stills 4 --max-sim-s 260
run sensor  --seed 42 --items box_s,bottle,helmet,edge_cube11 --manifest-extra --camera lookahead --depth-stills 4 --max-sim-s 260
run fault_recovery --seed 42 --camera routing --inject-jam box_l@2.62 --depth-stills 0 --max-sim-s 430
run fault_tray     --seed 42 --camera deck_front --inject-tray-fault D --depth-stills 0 --max-sim-s 430
run item_follow_perception_to_bin --seed "$CLEAN_SEED" --follow-slug pouf --camera overview --depth-stills 0 --max-sim-s 400
run perfect_iso       --seed "$CLEAN_SEED" --camera cell_iso   --depth-stills 0 --max-sim-s 400
run perfect_deckfront --seed "$CLEAN_SEED" --camera deck_front --depth-stills 0 --max-sim-s 400
run perfect_decktop   --seed "$CLEAN_SEED" --camera deck_top   --depth-stills 0 --max-sim-s 400
run cine_orbit  --seed "$CLEAN_SEED" --camera overview --camera-path orbit  --path-secs 82 --depth-stills 0 --max-sim-s 400
run cine_dolly  --seed "$CLEAN_SEED" --camera overview --camera-path dolly  --path-secs 82 --depth-stills 0 --max-sim-s 400
run cine_crane  --seed "$CLEAN_SEED" --camera overview --camera-path crane  --path-secs 82 --depth-stills 0 --max-sim-s 400

echo "=== SHOWCASE5 DONE ==="
ls -la "$OUT_ROOT"/*.mp4 2>/dev/null
echo "--- any clips fail? ---"
grep -hE 'CLIP FAILED|MP4 FAIL' "$OUT_ROOT"/*.log 2>/dev/null || echo 'all clips ok'
touch "$OUT_ROOT/SHOWCASE5_DONE"
