#!/usr/bin/env bash
# Quick presentation stills: short runs on each presentation camera, first
# clean frame kept as PNG.  bash capture_stills.sh [out]
set -u
OUT_ROOT="${1:-/root/sortmaster_out/stills_v17}"
REPO=/root/sortmaster
mkdir -p "$OUT_ROOT" /tmp/sortmaster_signs
chmod 777 "$OUT_ROOT"
OUT_ROOT_IN="/workspace/sortmaster_out/$(basename "$OUT_ROOT")"

shot() {
  name="$1"; cam="$2"; shift 2
  echo "=== still $name ($cam) ==="
  rm -rf "$OUT_ROOT/$name"
  docker run --rm --name "isaacstill-$name" --gpus all --network=host \
    --entrypoint /isaac-sim/python.sh \
    -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
    -v $REPO:/workspace/sortmaster:ro \
    -v /root/sortmaster_out:/workspace/sortmaster_out \
    -v /tmp/sortmaster_signs:/tmp/sortmaster_signs \
    -v /root/docker/isaac-sim/cache/main:/isaac-sim/.cache \
    -v /root/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache \
    nvcr.io/nvidia/isaac-sim:6.0.1 \
    /workspace/sortmaster/isaac/run_isaac.py \
    --out "$OUT_ROOT_IN/$name" --record --fps 10 \
    --perception rtx --drive surface --depth-stills 0 \
    --camera "$cam" --max-sim-s 14 "$@" \
    > "$OUT_ROOT/$name.log" 2>&1
  last=$(ls "$OUT_ROOT/$name/frames" 2>/dev/null | tail -1)
  [ -n "$last" ] && cp "$OUT_ROOT/$name/frames/$last" "$OUT_ROOT/$name.png" \
    && echo "png ok: $name.png"
}

shot hero_ne      hero_ne      --seed 7
shot mech_c_side  mech_c_side  --seed 42 --items box_l,pen
shot b_transfer   b_transfer   --seed 42 --items box_s,lunchbox
shot cage_closeup deck_front   --seed 7
echo "=== STILLS DONE ==="
touch "$OUT_ROOT/STILLS_DONE"
