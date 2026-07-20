#!/usr/bin/env bash
# Assemble a recorded run's frames (frames/%05d.png) into an H.264 MP4.
#   bash make_mp4.sh <run_dir> <out.mp4> [fps]
set -eu
DIR="$1"; OUT="$2"; FPS="${3:-30}"
ffmpeg -y -framerate "$FPS" -i "$DIR/frames/%05d.png" \
  -c:v libx264 -pix_fmt yuv420p -crf 18 -preset medium \
  -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2" "$OUT"
