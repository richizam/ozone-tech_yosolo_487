#!/usr/bin/env bash
# Visual-verification stills WITH FULL DRESSING (no SM_LEAN). Curated small
# item set so the dressed 20+ prim scene fits 24 GB VRAM while still looking
# populated. One short --record run per presentation camera, keep a frame.
#   bash verify_stills.sh [out]
set -u
OUT_ROOT="${1:-/root/sortmaster_out/stills_dressed}"
REPO=/root/sortmaster
mkdir -p "$OUT_ROOT" /tmp/sortmaster_signs
chmod 777 "$OUT_ROOT" 2>/dev/null || true
OUT_ROOT_IN="/workspace/sortmaster_out/$(basename "$OUT_ROOT")"

shot() {
  name="$1"; cam="$2"; shift 2
  echo "=== dressed still $name ($cam) ==="
  rm -rf "$OUT_ROOT/$name"
  # NOTE: no SM_LEAN -> full cosmetic dressing renders.
  docker run --rm --name "isaacdress-$name" --gpus all --network=host \
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
    > "$OUT_ROOT/$name.log" 2>&1
  last=$(ls "$OUT_ROOT/$name/frames" 2>/dev/null | tail -1)
  [ -n "$last" ] && cp "$OUT_ROOT/$name/frames/$last" "$OUT_ROOT/$name.png" \
    && echo "png ok: $name.png" || echo "NO FRAME ($name) — check $name.log"
}

# curated variety: box + round + soft + oversize + edge — 5 items, dressed
CURATED="box_l,bottle,helmet,pouf,pen"

shot hero_ne      hero_ne      --seed 7 --items "$CURATED"
shot mech_c_side  mech_c_side  --seed 42 --items box_l,pen
shot b_transfer   b_transfer   --seed 42 --items box_s,lunchbox
shot cage_closeup deck_front   --seed 7 --items "$CURATED"
shot return_guard cell_iso     --seed 7 --items bottle,helmet

echo "=== DRESSED STILLS DONE ==="
touch "$OUT_ROOT/DRESSED_STILLS_DONE"
