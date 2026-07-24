# -*- coding: utf-8 -*-
"""Precision/recall per category from the SEALED evidence of both engines —
the official quality metric per the expert answer of 21-07 («метрика
качества определения категории — precision/recall по каждой категории»).

Exception-stream accounting (expert answer: uncertain classification must
not be silently booked as a routine category): deliveries to REVIEW/MANUAL
are the cell's DESIGNED exception flow and are reported as their own row,
not as C/D predictions. FLOOR would be a hard failure row (zero in the
sealed data).

Sources:
  Isaac  — docs/report/isaac_evidence/xbelt/validation_branch/
           xbelt_v5_pass_*/*/summary.json  (items[] with zone_true/routed)
  Twin   — runs/validation_<stamp>/*/events.csv item_delivered rows
Writes docs/report/classification_pr.{md,json}.
"""
import csv
import glob
import json
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CATS = ("B", "C", "D")
EXC = ("REVIEW", "MANUAL")


def add(conf, exc, true, pred):
    if pred in EXC:
        exc[true] += 1
    else:
        conf[(true, pred)] += 1


def pr_table(conf, exc):
    rows = []
    for c in CATS:
        tp = conf.get((c, c), 0)
        fp = sum(v for (t, p), v in conf.items() if p == c and t != c)
        fn = sum(v for (t, p), v in conf.items() if t == c and p != c)
        prec = tp / (tp + fp) if tp + fp else None
        rec = tp / (tp + fn) if tp + fn else None
        rows.append({"category": c, "tp": tp, "fp": fp, "fn": fn,
                     "precision": None if prec is None else round(prec, 4),
                     "recall": None if rec is None else round(rec, 4),
                     "diverted_to_exceptions": exc.get(c, 0)})
    return rows


def isaac_counts(nominal_only):
    conf, exc = defaultdict(int), defaultdict(int)
    passes = sorted(glob.glob(str(
        ROOT / "docs/report/isaac_evidence/xbelt/validation_branch"
        / "xbelt_v5_pass_*")))
    if not passes:
        return None, None
    for sj in glob.glob(passes[-1] + "/*/summary.json"):
        run = Path(sj).parent.name
        if nominal_only and "nominal" not in run:
            continue
        s = json.load(open(sj, encoding="utf-8"))
        for it in s.get("items", []):
            add(conf, exc, it["zone_true"], it.get("delivered") or "MANUAL")
    return conf, exc


def twin_counts(stamp, nominal_only):
    conf, exc = defaultdict(int), defaultdict(int)
    for ev in glob.glob(str(ROOT / "runs" / stamp / "*" / "events.csv")):
        run = Path(ev).parent.name
        if nominal_only and not run.startswith("base_"):
            continue
        with open(ev, encoding="utf-8") as f:
            for row in csv.reader(f):
                if len(row) >= 4 and row[1] == "item_delivered":
                    d = json.loads(row[3])
                    add(conf, exc, d.get("zone_true"), d.get("zone"))
    return conf, exc


def main():
    stamp = sys.argv[1] if len(sys.argv) > 1 else "validation_20260720_212942"
    out = {"metric": "precision/recall per category (expert answer 21-07)",
           "exception_accounting": "REVIEW/MANUAL deliveries are the designed "
           "exception stream, reported separately — never booked as routine "
           "C/D classifications"}
    for eng, fn in (("isaac", isaac_counts),
                    ("twin", lambda n: twin_counts(stamp, n))):
        for scope, nom in (("nominal", True), ("all_runs", False)):
            conf, exc = fn(nom)
            if conf is None:
                continue
            out[f"{eng}_{scope}"] = {
                "table": pr_table(conf, exc),
                "n_routine": sum(conf.values()),
                "n_exceptions": sum(exc.values())}
    dst = ROOT / "docs/report/classification_pr.json"
    dst.write_text(json.dumps(out, indent=1, ensure_ascii=False),
                   encoding="utf-8")
    lines = ["# Precision / Recall по категориям (официальная метрика)", "",
             "> Источник: запечатанные матрицы обоих движков; исключения "
             "(REVIEW/MANUAL) — штатный поток нештатных случаев, отдельная "
             "строка, не C/D.", ""]
    for key in ("isaac_nominal", "isaac_all_runs", "twin_nominal",
                "twin_all_runs"):
        if key not in out:
            continue
        t = out[key]
        lines += [f"## {key}  (routine {t['n_routine']}, "
                  f"exceptions {t['n_exceptions']})", "",
                  "| cat | precision | recall | tp | fp | fn | → exceptions |",
                  "|---|---|---|---|---|---|---|"]
        for r in t["table"]:
            lines.append(
                f"| {r['category']} | {r['precision']} | {r['recall']} "
                f"| {r['tp']} | {r['fp']} | {r['fn']} "
                f"| {r['diverted_to_exceptions']} |")
        lines.append("")
    (ROOT / "docs/report/classification_pr.md").write_text(
        "\n".join(lines), encoding="utf-8")
    print(json.dumps({k: v for k, v in out.items() if k.endswith("nominal")},
                     indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
