#!/usr/bin/env bash
# Presentation pipeline: clearance audit -> stills -> arm-drill recheck ->
# full showcase. Run under nohup/setsid; progress via markers + logs in OUT.
#   bash present_pipeline.sh /root/sortmaster_out
set -u
OUT="${1:-/root/sortmaster_out}"
REPO=/root/sortmaster

echo "[pipeline] audit" > "$OUT/present_pipeline.log"
docker run --rm --name isaacaudit --gpus all --network=host \
  --entrypoint /isaac-sim/python.sh \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
  -v $REPO:/workspace/sortmaster:ro \
  -v $OUT:/workspace/sortmaster_out \
  -v /root/docker/isaac-sim/cache/main:/isaac-sim/.cache \
  -v /root/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache \
  nvcr.io/nvidia/isaac-sim:6.0.1 \
  /workspace/sortmaster/isaac/audit_clearance.py \
  --out /workspace/sortmaster_out/audit_v17 \
  >> "$OUT/present_pipeline.log" 2>&1
AUDIT_RC=$?
echo "[pipeline] audit rc=$AUDIT_RC" >> "$OUT/present_pipeline.log"
touch "$OUT/AUDIT_V17_DONE"
if [ "$AUDIT_RC" != "0" ]; then
  echo "[pipeline] AUDIT FAILED - stopping before renders" \
    >> "$OUT/present_pipeline.log"
  touch "$OUT/PIPELINE_ABORTED"
  exit 1
fi

echo "[pipeline] stills" >> "$OUT/present_pipeline.log"
bash $REPO/isaac/capture_stills.sh "$OUT/stills_v17" \
  >> "$OUT/present_pipeline.log" 2>&1

echo "[pipeline] fault_jam recheck (relocated arm)" \
  >> "$OUT/present_pipeline.log"
rm -rf "$OUT/faultjam_v17"
docker run --rm --name isaacrun-fjv17 --gpus all --network=host \
  --entrypoint /isaac-sim/python.sh \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
  -v $REPO:/workspace/sortmaster:ro \
  -v $OUT:/workspace/sortmaster_out \
  -v /tmp/sortmaster_signs:/tmp/sortmaster_signs \
  -v /root/docker/isaac-sim/cache/main:/isaac-sim/.cache \
  -v /root/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache \
  nvcr.io/nvidia/isaac-sim:6.0.1 \
  /workspace/sortmaster/isaac/run_isaac.py \
  --out /workspace/sortmaster_out/faultjam_v17 \
  --seed 42 --inject-jam box_l@2.62 --max-sim-s 430 \
  --perception rtx --drive surface --depth-stills 0 \
  >> "$OUT/present_pipeline.log" 2>&1
FJ_RC=$?
echo "[pipeline] fault_jam rc=$FJ_RC" >> "$OUT/present_pipeline.log"
touch "$OUT/FAULTJAM_V17_DONE"
if [ "$FJ_RC" != "0" ]; then
  echo "[pipeline] FAULT_JAM FAILED - stopping before showcase" \
    >> "$OUT/present_pipeline.log"
  touch "$OUT/PIPELINE_ABORTED"
  exit 1
fi

echo "[pipeline] showcase" >> "$OUT/present_pipeline.log"
bash $REPO/isaac/showcase5.sh "$OUT/showcase_v17" 7 \
  >> "$OUT/present_pipeline.log" 2>&1
echo "[pipeline] showcase done" >> "$OUT/present_pipeline.log"
touch "$OUT/SHOWCASE_V17_DONE"
