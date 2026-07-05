# -*- coding: utf-8 -*-
"""YOLO26n vs YOLO11n on the belt-item detection task.

Trains both nano models on the synthetic vision-station dataset, evaluates on
the val split, exports both to ONNX and benchmarks CPU inference latency —
the deployment-relevant figure (our reproducibility path is CPU-only).

    python -m perception.yolo_compare [--epochs 12] [--imgsz 448]
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

DATA = ROOT / "datasets" / "belt_items" / "data.yaml"
OUT = ROOT / "docs" / "metrics" / "yolo_comparison.json"


def bench_onnx(onnx_path, imgsz, n=40):
    import onnxruntime as ort
    sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    name = sess.get_inputs()[0].name
    x = np.random.rand(1, 3, imgsz, imgsz).astype(np.float32)
    for _ in range(8):
        sess.run(None, {name: x})
    times = []
    for _ in range(n):
        t0 = time.perf_counter()
        sess.run(None, {name: x})
        times.append((time.perf_counter() - t0) * 1000)
    return {"mean_ms": round(float(np.mean(times)), 1),
            "p95_ms": round(float(np.percentile(times, 95)), 1)}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=12)
    ap.add_argument("--imgsz", type=int, default=448)
    ap.add_argument("--batch", type=int, default=8)
    args = ap.parse_args(argv)

    from ultralytics import YOLO

    results = {}
    for tag, weights in (("yolo26n", "yolo26n.pt"), ("yolo11n", "yolo11n.pt")):
        print(f"\n===== {tag} =====")
        model = YOLO(weights)
        t0 = time.time()
        model.train(data=str(DATA), epochs=args.epochs, imgsz=args.imgsz,
                    batch=args.batch, device="cpu", workers=0, seed=0,
                    deterministic=True, verbose=False, plots=False,
                    project=str(ROOT / "runs" / "yolo"), name=tag, exist_ok=True)
        train_s = round(time.time() - t0, 1)
        val = model.val(data=str(DATA), imgsz=args.imgsz, device="cpu", workers=0,
                        verbose=False, plots=False,
                        project=str(ROOT / "runs" / "yolo"), name=f"{tag}_val", exist_ok=True)
        onnx_path = model.export(format="onnx", imgsz=args.imgsz, device="cpu")
        n_params = sum(p.numel() for p in model.model.parameters())
        results[tag] = {
            "params_M": round(n_params / 1e6, 2),
            "train_time_s": train_s,
            "mAP50": round(float(val.box.map50), 4),
            "mAP50_95": round(float(val.box.map), 4),
            "onnx_cpu": bench_onnx(onnx_path, args.imgsz),
            "onnx_file": str(Path(onnx_path).name),
            "nms_free_end2end": tag == "yolo26n",
        }
        print(tag, json.dumps(results[tag], indent=2))

    results["_meta"] = {
        "dataset": "300 synthetic vision-station frames (240 train / 60 val), 1 class",
        "epochs": args.epochs, "imgsz": args.imgsz,
        "role": "detection/tracking only — categories always come from measured geometry",
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(results, indent=2), encoding="utf-8")
    print(f"\n-> {OUT}")


if __name__ == "__main__":
    main()
