#!/usr/bin/env bash
# Pull the tilt-tray build's evidence set from the render server into the
# repo's report tree. Run from the repo root (Git Bash):
#   bash tools/fetch_evidence.sh [matrix_dir_name] [showcase_dir_name]
# Server is overridable for a new box, e.g.:
#   SM_HOST=root@85.218.235.6 SM_PORT=35206 bash tools/fetch_evidence.sh xbelt_matrix_final showcase5
set -u
HOST="${SM_HOST:-root@90.224.159.6}"
PORT="${SM_PORT:-40576}"
MATRIX="${1:-xbelt_matrix}"
SHOW="${2:-showcase5}"
DEST="docs/report/isaac_evidence/xbelt"
mkdir -p "$DEST/matrix" "$DEST/videos_final" "$DEST/perception" "$DEST/stills"

# consolidated matrix + per-run summaries/logs (small text artifacts)
scp -P $PORT "$HOST:/root/sortmaster_out/$MATRIX/matrix_summary.json" "$DEST/matrix/" || true
for run in seed42_nominal seed1_nominal seed2_nominal seed3_nominal seed7_nominal seed99_nominal \
           edge_items_all edge_small low_friction high_mass close_spacing off_center fault_jam fault_tray; do
  mkdir -p "$DEST/matrix/$run"
  scp -P $PORT "$HOST:/root/sortmaster_out/$MATRIX/$run/summary.json" "$DEST/matrix/$run/" 2>/dev/null || true
  scp -P $PORT "$HOST:/root/sortmaster_out/$MATRIX/$run/events.csv"   "$DEST/matrix/$run/" 2>/dev/null || true
  scp -P $PORT "$HOST:/root/sortmaster_out/$MATRIX/$run/actuator_log.csv" "$DEST/matrix/$run/" 2>/dev/null || true
  scp -P $PORT "$HOST:/root/sortmaster_out/$MATRIX/$run/reads_log.json"   "$DEST/matrix/$run/" 2>/dev/null || true
done

# final MP4s (cinematics, statics, edge cases, faults)
scp -P $PORT "$HOST:/root/sortmaster_out/$SHOW/*.mp4" "$DEST/videos_final/" || true

# perception stills + depth: RGB / depth / macro pngs, raw npy, the
# pipeline-truth masks/clouds (vision_mask/cloud/macro_*) and the fused
# reads log (make_perception_panels.py composes the jury panels from
# these locally)
for run in sensor edge_small_items; do
  mkdir -p "$DEST/perception/$run"
  scp -P $PORT "$HOST:/root/sortmaster_out/$SHOW/$run/vision_*.png" "$DEST/perception/$run/" 2>/dev/null || true
  scp -P $PORT "$HOST:/root/sortmaster_out/$SHOW/$run/vision_*.npy" "$DEST/perception/$run/" 2>/dev/null || true
  scp -P $PORT "$HOST:/root/sortmaster_out/$SHOW/$run/reads_log.json" "$DEST/perception/$run/" 2>/dev/null || true
  scp -P $PORT "$HOST:/root/sortmaster_out/$SHOW/$run/summary.json"   "$DEST/perception/$run/" 2>/dev/null || true
done

# perception side-by-side panels + demo mp4 (built on the host by
# tools/make_perception_panels.py) and the metrics end card
scp -P $PORT "$HOST:/root/sortmaster_out/$SHOW/panels/*"   "$DEST/perception/" 2>/dev/null || true
scp -P $PORT "$HOST:/root/sortmaster_out/$SHOW/endcard.png" "$DEST/videos_final/" 2>/dev/null || true

echo "--- fetched into $DEST ---"
find "$DEST" -type f | sort | sed 's/^/  /'
du -sh "$DEST"
