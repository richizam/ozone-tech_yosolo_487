#!/usr/bin/env bash
# queue_v5 — hardened matrix->clip->showcase campaign runner (5090 host).
#
# Encodes the three ops lessons this project paid for:
#   1. SINGLE INSTANCE: an flock(1) lock — a dropped SSH session relaunching
#      the queue must NOT start a zombie twin (a zombie once wiped the
#      per-run logs of a failing pass while re-running it).
#   2. ALWAYS-TERMINAL STATUS: every exit path (success, gate fail, crash,
#      SIGTERM) stamps $STATUS_FILE via trap — silence is impossible. The
#      old watcher only reported the green branch and sat mute for ~2 h in
#      front of a red gate.
#   3. EVIDENCE OUTLIVES RERUNS: each pass runs in its own timestamped
#      directory, nothing ever reruns inside a used one, and the per-pass
#      log + gate verdict + floor/unsafe forensics are tee'd/copied into
#      $LOG_ROOT, which no step ever deletes.
#
#   bash queue_v5.sh [max_passes]     # default 2
set -u
REPO=/root/sortmaster
LOG_ROOT=/root/queue_v5_logs                # never deleted by anything here
STATUS_FILE="$LOG_ROOT/STATUS"
MAX_PASSES="${1:-2}"
mkdir -p "$LOG_ROOT"

exec 9>"/root/queue_v5.lock"
if ! flock -n 9; then
  echo "$(date -u +%FT%TZ) REFUSED: another queue_v5 holds the lock" >> "$STATUS_FILE"
  exit 97
fi

STATE="STARTING"
DETAIL=""
stamp() { echo "$(date -u +%FT%TZ) $*" | tee -a "$STATUS_FILE"; }
on_exit() {
  rc=$?
  [ "$STATE" = "DONE" ] || STATE="DEAD(rc=$rc,phase=$PHASE)"
  stamp "TERMINAL: $STATE $DETAIL"
}
PHASE="init"
trap on_exit EXIT

run_pass() {
  local tag out
  tag="pass_$(date -u +%Y%m%d_%H%M%S)"
  out="/root/sortmaster_out/xbelt_v5_$tag"
  stamp "PHASE matrix $tag: starting 14-run matrix -> $out"
  bash "$REPO/isaac/run_matrix.sh" "$out" 2>&1 | tee "$LOG_ROOT/$tag.matrix.log"
  # gate verdict via the consolidator (nonzero exit on any red gate)
  python3 "$REPO/tools/consolidate_isaac_matrix.py" "$out" \
      > "$LOG_ROOT/$tag.consolidate.log" 2>&1
  local gates=$?
  cp -f "$out"/*/summary.json "$LOG_ROOT/$tag.summaries/" 2>/dev/null || {
    mkdir -p "$LOG_ROOT/$tag.summaries"
    for d in "$out"/*/; do
      [ -f "$d/summary.json" ] && cp -f "$d/summary.json" \
        "$LOG_ROOT/$tag.summaries/$(basename "$d").summary.json"
    done
  }
  # forensics snapshot for ANY floor/unsafe before anything else can touch
  # the pass directory: events + summary of every offending run
  python3 - "$out" "$LOG_ROOT/$tag.forensics" <<'PYEOF'
import json, os, shutil, sys
out, dst = sys.argv[1], sys.argv[2]
for d in sorted(os.listdir(out)):
    sj = os.path.join(out, d, "summary.json")
    if not os.path.isfile(sj):
        continue
    try:
        s = json.load(open(sj))
    except Exception:
        continue
    if (s.get("floor_drops") or 0) or (s.get("unsafe_errors") or 0):
        os.makedirs(dst, exist_ok=True)
        for f in os.listdir(os.path.join(out, d)):
            if f.endswith((".json", ".csv", ".jsonl", ".log")):
                shutil.copy2(os.path.join(out, d, f),
                             os.path.join(dst, f"{d}__{f}"))
        print(f"forensics: {d} floor={s.get('floor_drops')} "
              f"unsafe={s.get('unsafe_errors')}")
PYEOF
  echo "$out" > "$LOG_ROOT/$tag.outdir"
  return $gates
}

PHASE="matrix"
PASS_OK=""
for i in $(seq 1 "$MAX_PASSES"); do
  if run_pass; then
    PASS_OK="yes"
    stamp "GATES PASS on attempt $i"
    break
  fi
  stamp "GATES FAIL on attempt $i (forensics + summaries in $LOG_ROOT)"
done
if [ -z "$PASS_OK" ]; then
  STATE="NEEDS_ATTENTION"
  DETAIL="matrix red after $MAX_PASSES attempt(s) — do NOT rerun blindly; read forensics"
  exit 3
fi

# ---- unknown-shapes clip: the extra manifest must exist ONLY during this
# step (an uploaded manifest_custom.json once auto-merged 10 items into
# every matrix run and invalidated a pass)
PHASE="unknown_clip"
CUSTOM="$REPO/cell/assets/manifest_custom.json"
UNK_SRC="/root/unkclip"                       # STLs + manifest staged here
cp -f "$REPO/isaac/make_mp4.sh" /root/make_mp4.sh
if [ -d "$UNK_SRC" ] && [ -f "$UNK_SRC/manifest_custom.json" ]; then
  restore_manifest() { rm -f "$CUSTOM"; }
  trap 'restore_manifest; on_exit' EXIT
  cp -f "$UNK_SRC"/*.stl "$REPO/cell/assets/meshes/" 2>/dev/null || true
  cp -f "$UNK_SRC/manifest_custom.json" "$CUSTOM"
  UNK_SLUGS=$(python3 -c "import json; print(','.join(
      e['slug'] for e in json.load(open('$UNK_SRC/manifest_custom.json'))))")
  UNK_OUT="/root/sortmaster_out/unknown_clip"
  mkdir -p "$UNK_OUT"; chmod 777 "$UNK_OUT"
  stamp "PHASE unknown_clip: rendering 10 unknown shapes ($UNK_SLUGS)"
  timeout --signal=KILL 2400 \
  docker run --rm --name isaacrec-unknown --gpus all --network=host \
    --ulimit nofile=1048576:1048576 \
    --entrypoint /isaac-sim/python.sh \
    -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
    -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /root/sortmaster:/workspace/sortmaster:ro \
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
    --out /workspace/sortmaster_out/unknown_clip/unknown_shapes \
    --seed 42 --items "$UNK_SLUGS" --camera deck_front --depth-stills 4 \
    --perception rtx --drive surface --record --fps 30 --max-sim-s 700 \
    > "$LOG_ROOT/unknown_clip.log" 2>&1 \
    || stamp "unknown_clip run FAILED (non-fatal, see log)"
  restore_manifest
  trap on_exit EXIT
  if ls "$UNK_OUT/unknown_shapes/frames"/*.png >/dev/null 2>&1; then
    bash /root/make_mp4.sh "$UNK_OUT/unknown_shapes" \
        "$UNK_OUT/unknown_shapes.mp4" 30 \
        >> "$LOG_ROOT/unknown_clip.log" 2>&1 \
      && stamp "unknown_clip mp4 OK" || stamp "unknown_clip MP4 FAIL"
  fi
else
  stamp "PHASE unknown_clip: SKIPPED ($UNK_SRC not staged)"
fi

# ---- showcase re-render (hardened v5b runner: per-clip timeout + retry,
# resume guard skips finished mp4s — safe to kill when the window closes)
PHASE="showcase"
if [ -f "$REPO/isaac/showcase5b.sh" ]; then
  stamp "PHASE showcase: starting showcase5b"
  bash "$REPO/isaac/showcase5b.sh" /root/sortmaster_out/showcase5 2>&1 \
      | tee "$LOG_ROOT/showcase.log" || stamp "showcase FAILED (non-fatal)"
else
  stamp "PHASE showcase: SKIPPED (showcase5b.sh not present)"
fi

STATE="DONE"
DETAIL="matrix sealed$([ -d "$UNK_SRC" ] && echo ', clip attempted')"
stamp "ALL PHASES COMPLETE"
