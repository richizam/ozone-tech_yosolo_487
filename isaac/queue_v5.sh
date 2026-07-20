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
if [ -d "$UNK_SRC" ] && [ -f "$UNK_SRC/manifest_custom.json" ]; then
  restore_manifest() { rm -f "$CUSTOM"; }
  trap 'restore_manifest; on_exit' EXIT
  cp -f "$UNK_SRC"/*.stl "$REPO/cell/assets/meshes/" 2>/dev/null || true
  cp -f "$UNK_SRC/manifest_custom.json" "$CUSTOM"
  stamp "PHASE unknown_clip: manifest isolated in place, rendering"
  bash "$REPO/isaac/showcase5b.sh" unknown_shapes 2>&1 \
      | tee "$LOG_ROOT/unknown_clip.log" || stamp "unknown_clip FAILED (non-fatal)"
  restore_manifest
  trap on_exit EXIT
else
  stamp "PHASE unknown_clip: SKIPPED ($UNK_SRC not staged)"
fi

# ---- showcase re-render, priority order (lowest value last, safe to kill)
PHASE="showcase"
if [ -x "$REPO/isaac/showcase_priority.sh" ]; then
  stamp "PHASE showcase: starting priority re-render"
  bash "$REPO/isaac/showcase_priority.sh" 2>&1 \
      | tee "$LOG_ROOT/showcase.log" || stamp "showcase FAILED (non-fatal)"
else
  stamp "PHASE showcase: SKIPPED (showcase_priority.sh not present)"
fi

STATE="DONE"
DETAIL="matrix sealed$([ -d "$UNK_SRC" ] && echo ', clip attempted')"
stamp "ALL PHASES COMPLETE"
