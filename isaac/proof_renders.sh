#!/usr/bin/env bash
# Hard-correction PROOF RENDERS (full dressing, fps 30 -> mp4):
#   proof_tilt_side : screenshot-2 angle — side view of the FULL C-station
#                     tilt + return cycle (before = before_fix_evidence/
#                     mech_tilt_C.mp4, identical camera/seed/items)
#   proof_decktop   : screenshot-1 angle — top-down over the mouths with
#                     the helmet (before = perfect_decktop.mp4)
#   proof_cage      : screenshot-3 angle — cage/arm quarter (before =
#                     perfect_deckfront.mp4)
#   proof_pouf      : largest supported item (489 mm pouf) carried and
#                     discharged — the largest-item transfer test
#   bash proof_renders.sh [out]
set -u
OUT_ROOT="${1:-/root/sortmaster_out/proof_renders}"
REPO=/root/sortmaster
mkdir -p "$OUT_ROOT" /tmp/sortmaster_signs
chmod 777 "$OUT_ROOT" 2>/dev/null || true
OUT_ROOT_IN="/workspace/sortmaster_out/$(basename "$OUT_ROOT")"
PER_CLIP_TIMEOUT="${PER_CLIP_TIMEOUT:-2400}"

_render_once() {
  name="$1"; shift
  docker rm -f "isaacproof-$name" >/dev/null 2>&1 || true
  timeout --signal=KILL "$PER_CLIP_TIMEOUT" \
  docker run --rm --name "isaacproof-$name" --gpus all --network=host \
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
    --perception rtx --drive surface --record --fps 30 --depth-stills 0 \
    "$@" >> "$OUT_ROOT/$name.log" 2>&1
  docker rm -f "isaacproof-$name" >/dev/null 2>&1 || true
}

run() {
  name="$1"; shift
  echo "=== proof $name ==="
  rm -rf "$OUT_ROOT/$name" "$OUT_ROOT/$name.mp4"
  : > "$OUT_ROOT/$name.log"
  for attempt in 1 2; do
    _render_once "$name" "$@"
    nf=$(ls "$OUT_ROOT/$name/frames" 2>/dev/null | wc -l)
    if [ "$nf" -gt 0 ]; then
      bash /root/make_mp4.sh "$OUT_ROOT/$name" "$OUT_ROOT/$name.mp4" 30 \
        >> "$OUT_ROOT/$name.log" 2>&1 && echo "mp4 ok: $name ($nf frames)"
      return 0
    fi
    echo "no frames ($name) attempt $attempt"; sleep 10
  done
  echo "PROOF CLIP FAILED: $name"
}

run proof_tilt_side --seed 42 --items box_l,pen --camera mech_c_side --max-sim-s 200
run proof_decktop   --seed 42 --items helmet,bottle,plate --camera deck_top --max-sim-s 240
run proof_cage      --seed 7 --items box_l,bottle,helmet,pouf,pen --camera deck_front --max-sim-s 240
run proof_pouf      --seed 7 --items pouf --camera routing --max-sim-s 160

echo "=== PROOF RENDERS DONE ==="
ls -la "$OUT_ROOT"/*.mp4 2>/dev/null
touch "$OUT_ROOT/PROOF_DONE"
