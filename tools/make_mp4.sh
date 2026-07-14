#!/usr/bin/env bash
# Stitch the numbered PNG frames run_isaac.py writes (<clipdir>/frames/NNNNN.png)
# into an H.264 mp4. Runs on the HOST (frames live on the host filesystem);
# needs host ffmpeg (apt-get install ffmpeg). This is the helper showcase5*.sh
# invokes as /root/make_mp4.sh — deploy a copy there on a fresh box.
#   make_mp4.sh <clipdir> <out.mp4> [fps]
set -u
CLIP="${1:?clipdir}"; OUT="${2:?out.mp4}"; FPS="${3:-30}"
FR="$CLIP/frames"
n=$(ls "$FR"/*.png 2>/dev/null | wc -l)
if [ "$n" -eq 0 ]; then echo "make_mp4: no frames in $FR"; exit 1; fi
# glob input keeps us independent of the exact zero-padding / start index.
ffmpeg -y -framerate "$FPS" -pattern_type glob -i "$FR/*.png" \
  -c:v libx264 -pix_fmt yuv420p -crf 18 -preset medium \
  -movflags +faststart "$OUT" </dev/null 2>&1 | tail -2
if [ -s "$OUT" ]; then
  echo "make_mp4: wrote $OUT ($n frames @ ${FPS}fps, $(du -h "$OUT" | cut -f1))"
else
  echo "make_mp4: FAILED to write $OUT"; exit 1
fi
