#!/usr/bin/env bash
# Cierre de la campaña sweep+guardas: ejecutar SOLO tras GATES: PASS.
#   SM_HOST=root@70.30.221.109 SM_PORT=44742 bash tools/finalize_sweep_campaign.sh
# Trae matriz + clip + stills, regenera endcard, imprime el diff de agregados
# para el re-sello de docs. El merge a master queda como paso humano.
set -eu
HOST="${SM_HOST:-root@70.30.221.109}"
PORT="${SM_PORT:-44742}"
DEST=docs/report/isaac_evidence/xbelt

echo "== matriz =="
for run in seed42_nominal seed1_nominal seed2_nominal seed3_nominal \
           seed7_nominal seed99_nominal edge_items_all edge_small \
           low_friction high_mass close_spacing off_center fault_jam \
           fault_tray; do
  mkdir -p "$DEST/matrix/$run"
  for f in summary.json events.csv actuator_log.csv reads_log.json; do
    scp -q -P "$PORT" "$HOST:/root/sortmaster_out/xbelt_matrix_sweep/$run/$f" \
        "$DEST/matrix/$run/" 2>/dev/null || true
  done
done
scp -q -P "$PORT" "$HOST:/root/sortmaster_out/xbelt_matrix_sweep/matrix_summary.json" \
    "$DEST/matrix/" 2>/dev/null || true

echo "== clip de formas nuevas =="
scp -q -P "$PORT" "$HOST:/root/sortmaster_out/unknown_shapes_demo.mp4" \
    "$DEST/videos_final/unknown_shapes_demo.mp4" 2>/dev/null || echo "  (aun no)"
mkdir -p "$DEST/perception/unknown_shapes"
scp -q -P "$PORT" "$HOST:/root/sortmaster_out/unknown_shapes_demo/vision_*" \
    "$DEST/perception/unknown_shapes/" 2>/dev/null || true
scp -q -P "$PORT" "$HOST:/root/sortmaster_out/unknown_shapes_demo/reads_log.json" \
    "$DEST/perception/unknown_shapes/" 2>/dev/null || true

echo "== endcard =="
python tools/make_endcard.py "$DEST/matrix/matrix_summary.json" \
    "$DEST/videos_final/endcard.png"

echo "== agregados (para re-sello de docs si difieren) =="
python - <<'EOF'
import json
d = json.load(open('docs/report/isaac_evidence/xbelt/matrix/matrix_summary.json'))
a = d['aggregate']
for k in ('nominal_classification_accuracy', 'nominal_routing_accuracy',
          'total_unsafe_errors', 'total_floor_drops', 'min_containment_rate',
          'min_command_margin_s', 'total_direct_velocity_writes_nominal'):
    print(f"  {k} = {a.get(k)}")
c = a.get('cube11_proof') or {}
print(f"  cube11: {c.get('delivered')} dims={c.get('dims_mm')}")
EOF
echo "== showcase sincronizado hasta ahora =="
ls "$DEST/videos_final/" | wc -l
echo "LISTO — revisa agregados; luego: merge de fix/section-multiaxis a master"
