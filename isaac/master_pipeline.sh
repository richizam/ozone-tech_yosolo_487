#!/usr/bin/env bash
# Master validation chain (server-side, run detached). Stops before the
# showcase so gates + audit + stills can be reviewed first.
#   setsid nohup bash master_pipeline.sh > /root/sortmaster_out/master.log 2>&1 &
set -u
OUT=/root/sortmaster_out
REPO=/root/sortmaster
DK=(--rm --gpus all --network=host --entrypoint /isaac-sim/python.sh
    -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y
    -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all
    -v "$REPO":/workspace/sortmaster:ro
    -v "$OUT":/workspace/sortmaster_out
    -v /tmp/sortmaster_signs:/tmp/sortmaster_signs
    -v /root/.cache/ov/hub:/var/cache/hub
    -v /root/docker/isaac-sim/cache/main:/isaac-sim/.cache
    -v /root/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache
    -v /root/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs
    -v /root/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config
    -v /root/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data
    -v /root/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg)

export SM_LEAN=1   # validation runs skip cosmetic dressing (VRAM guard)
rm -f "$OUT"/MATRIX_PHASE_DONE "$OUT"/PHASE_FAILED

echo "=== [master] MATRIX start $(date -u) ==="
bash "$REPO/isaac/run_matrix.sh" "$OUT/xbelt_matrix_final"
echo "=== [master] CONSOLIDATE $(date -u) ==="
python3 "$REPO/tools/consolidate_isaac_matrix.py" "$OUT/xbelt_matrix_final"
GATE_RC=$?
echo "=== [master] gates rc=$GATE_RC ==="
# a failed gate stops the pipeline: everything downstream (audit, stills,
# clips) would otherwise present artifacts from a build that did not pass
if [ "$GATE_RC" -ne 0 ]; then
  echo "=== [master] ABORT: gates failed (rc=$GATE_RC) ==="
  touch "$OUT/GATES_FAILED"
  exit "$GATE_RC"
fi

echo "=== [master] AUDIT $(date -u) ==="
docker run --name isaacaudit "${DK[@]}" \
  nvcr.io/nvidia/isaac-sim:6.0.1 \
  /workspace/sortmaster/isaac/audit_clearance.py \
  --out /workspace/sortmaster_out/audit_final
AUDIT_RC=$?
echo "=== [master] audit rc=$AUDIT_RC ==="
if [ "$AUDIT_RC" -ne 0 ]; then
  echo "=== [master] ABORT: clearance audit failed (rc=$AUDIT_RC) ==="
  touch "$OUT/AUDIT_FAILED"
  exit "$AUDIT_RC"
fi

echo "=== [master] MATRIX_PHASE_DONE $(date -u) (gates rc=$GATE_RC) ==="
touch "$OUT/MATRIX_PHASE_DONE"
