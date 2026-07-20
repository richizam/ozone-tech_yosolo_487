# -*- coding: utf-8 -*-
"""Defence visuals: REAL sensor point clouds with the measurement drawn on
top — what the depth station sees, the cutting-station section with its
angular-bin radii, and the verdict with the exact rule that fired.

Each figure (1920x1080, the cell's presentation design language):
  left   3D scatter of the twin sensor's actual point cloud (height-coloured)
  right  the decisive cross-section: band points, mirror closure, angular
         bins with the min/max radii highlighted, r_in/R printed
  footer fused dims + features + zone chip, same grammar as the perception
         panels (docs/report/isaac_evidence/xbelt/perception/panels/)

Items: officials (bottle, box_s, helmet) + the blind-spot pair (hex prism
lying/standing) + the oversized rod — the defence storyline.

Usage: .venv/Scripts/python tools/make_cloud_figures.py [out_dir]
CPU-only (MuJoCo twin sensor). ~2 min.
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                       # noqa: E402
import trimesh                           # noqa: E402

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import mujoco                                          # noqa: E402
from cell import params as P                           # noqa: E402
from cell.scene import make_model                      # noqa: E402
from perception.pipeline import (BELT_Z,               # noqa: E402
                                 LookaheadPerception,
                                 _mirrored_hull_ratio, min_area_rect)
from perception.validate import park_item, settle_item  # noqa: E402

OZON_BLUE = "#005BFF"
MAGENTA = "#F01E78"
BG = "#0D0E12"
PANEL = "#16181E"
TXT = "#EBEDF2"
SUB = "#969CA8"
ZONE_COL = {"B": "#4A8CF2", "C": "#F28C26", "D": "#33C759",
            "REVIEW": "#B84099"}
ZONE_RU = {"B": "ПОДХОДИТ ДЛЯ СОРТИРОВКИ", "C": "НЕ ПОДХОДИТ ПО ГАБАРИТАМ",
           "D": "ТРЕБУЕТ ДОУПАКОВКИ", "REVIEW": "РУЧНОЙ РАЗБОР"}

ASSETS = REPO / "cell" / "assets"
MESHES = ASSETS / "meshes"
CUSTOM = ASSETS / "manifest_custom.json"


# --------------------------------------------------------- probe items
def probe_meshes():
    """(slug, mesh, title, note) for the defence storyline."""
    hexm = trimesh.creation.annulus(r_min=0.0, r_max=0.060, height=0.104,
                                    sections=6)
    lying = hexm.copy()
    lying.apply_transform(trimesh.transformations.rotation_matrix(
        np.pi / 2, [1, 0, 0]))
    rod = trimesh.creation.cylinder(radius=0.020, height=0.490)
    rod.apply_transform(trimesh.transformations.rotation_matrix(
        np.pi / 2, [0, 1, 0]))
    # dual-range small set (QA-сессия 17-07: игральная кость 12 мм —
    # ЛЕГАЛЬНЫЙ товар B; мелкие круглые — никогда в B)
    die = trimesh.creation.box(extents=[0.012] * 3)
    smallcyl = trimesh.creation.cylinder(radius=0.006, height=0.040,
                                         sections=64)
    smallcyl.apply_transform(trimesh.transformations.rotation_matrix(
        np.pi / 2, [0, 1, 0]))
    sphere = trimesh.creation.icosphere(subdivisions=3, radius=0.006)
    return [
        ("hex_lying", lying, "Шестигранная призма 120×104 — ЛЁЖА",
         "слепая зона закрыта: развёртка осей находит сечение 0.87"),
        ("hex_standing", hexm, "Шестигранная призма 120×104 — СТОЯ",
         "силуэт сверху — шестиугольник: circ 0.88 решает сам"),
        ("rod_490", rod, "Цилиндр Ø40×490 — перемер",
         "OBB 490 мм > 450: габаритный гейт (приоритет правил)"),
        ("die_12mm", die, "Игральная кость 12 мм — ЛЕГАЛЬНЫЙ товар",
         "макро-голова 0.4 мм/px: сечение-hull 0.71 < 0.8 — честно B "
         "(пол сертификации 10.8 мм)"),
        ("cyl_12x40", smallcyl, "Цилиндр Ø12×40 — ЛЁЖА",
         "макро-облако: зеркальное сечение ~1.0 — круг, D (раньше слепая "
         "зона малых форм)"),
        ("sphere_12mm", sphere, "Сфера Ø12 — минимальный круглый",
         "силуэт-круг + сечения: D; габарит 12 мм > пола 10.8 — "
         "не путать с недомером"),
    ]


def register_custom(entries):
    prev = CUSTOM.read_text(encoding="utf-8") if CUSTOM.exists() else None
    man = []
    for slug, mesh, _, _ in entries:
        mesh.export(str(MESHES / f"{slug}.stl"))
        dims = sorted(np.asarray(
            mesh.bounding_box_oriented.primitive.extents), reverse=True)
        man.append({"slug": slug, "source": f"{slug}.stl",
                    "file": f"{slug}.stl",
                    "dims_m": [float(d) for d in dims],
                    "half_z": float(min(dims)) / 2, "mass_kg": 0.6,
                    "zone": "D", "category": "defence figure"})
    CUSTOM.write_text(json.dumps(man, indent=1), encoding="utf-8")
    return prev


def cleanup_custom(prev, entries):
    if prev is None:
        CUSTOM.unlink(missing_ok=True)
    else:
        CUSTOM.write_text(prev, encoding="utf-8")
    for slug, _, _, _ in entries:
        (MESHES / f"{slug}.stl").unlink(missing_ok=True)


# ----------------------------------------------------- section geometry
def best_section(verdict, pts):
    """Rebuild the decisive cutting station from the sensor cloud with the
    CLASSIFIER'S OWN estimators (perception/pipeline.py): angular-bin ratio
    on the primary (min-area-rect) axis, mirrored-hull ratio on swept
    secondary axes — so the figure can never show a section the classifier
    would not score."""
    z_rel = pts[:, 2] - BELT_Z
    xy = pts[:, :2]
    c = xy.mean(axis=0)
    height = float(z_rel.max())
    zc = 0.5 * height
    _, _, ang = min_area_rect(xy)
    edges = np.linspace(np.deg2rad(30), np.deg2rad(150), 9)
    # SMALL (macro-head) regime: the classifier's macro path scores the
    # rectangle-stable mirrored-hull section, and station geometry scales
    # with the body — mirror that here so the figure shows the estimator
    # that actually decided
    span_all = float(np.ptp(xy @ np.array([np.cos(ang), np.sin(ang)])))
    small = span_all < 0.11
    best = None
    for k_ax in range(12):
        a = ang + k_ax * np.pi / 12.0
        primary = (k_ax == 0) and not small
        u = (xy - c) @ np.array([np.cos(a), np.sin(a)])
        v = (xy - c) @ np.array([-np.sin(a), np.cos(a)])
        u_min, u_max = float(u.min()), float(u.max())
        span = u_max - u_min
        if small:
            if span < 0.008:
                continue
            stations = np.linspace(u_min + 0.2 * span, u_max - 0.2 * span, 5)
            half_band = max(0.002, 0.08 * span)
        else:
            if span < 0.11:
                continue
            stations = np.linspace(u_min + 0.05, u_max - 0.05, 7)
            half_band = 0.005
        for s in stations:
            band = np.abs(u - s) < half_band
            if int(band.sum()) < 12:
                continue
            vb, zb = v[band], z_rel[band]
            if float(zb.max()) < 0.008:
                continue
            vc = 0.5 * (float(vb.min()) + float(vb.max()))
            if primary:
                theta = np.arctan2(zb - zc, vb - vc)
                rho = np.hypot(vb - vc, zb - zc)
                rho_out = []
                for lo, hi in zip(edges[:-1], edges[1:]):
                    m = (theta >= lo) & (theta < hi)
                    if m.any():
                        rho_out.append(float(rho[m].max()))
                if len(rho_out) < 6:
                    continue
                r = min(rho_out) / max(rho_out)
                r_in, r_out = min(rho_out), max(rho_out)
            else:
                pr = _mirrored_hull_ratio(vb - vc, zb, zc)
                if pr is None:
                    continue
                r = float(pr)
                rr = float(np.hypot(vb - vc, zb - zc).max())
                r_in, r_out = r * rr, rr
            if best is None or r > best["ratio"]:
                best = {"ratio": r, "axis_deg": np.degrees(a) % 180.0,
                        "s": s, "vb": vb - vc, "zb": zb - zc,
                        "metodo": ("бины (осн. ось)" if primary
                                   else "зерк. hull (макро)" if small
                                   else "зерк. hull (развёртка)"),
                        "r_in": r_in, "r_out": r_out}
    return best


# ------------------------------------------------------------- figure
def make_figure(slug, title, note, pts, verdict, out_dir):
    z_rel = (pts[:, 2] - BELT_Z) * 1000
    zone = verdict["zone"]
    col = ZONE_COL.get(zone, "#AAAAB4")

    fig = plt.figure(figsize=(19.2, 10.8), dpi=100)
    fig.patch.set_facecolor(BG)
    fig.add_artist(plt.Rectangle((0, 0.992), 1, 0.008, color=OZON_BLUE,
                                 transform=fig.transFigure))
    fig.add_artist(plt.Rectangle((0, 0), 1, 0.008, color=MAGENTA,
                                 transform=fig.transFigure))
    fig.text(0.03, 0.955, "ОБЛАКО ТОЧЕК СЕНСОРА — ИЗМЕРЕНИЕ И РЕШЕНИЕ",
             color=TXT, fontsize=26, fontweight="bold", family="sans-serif")
    fig.text(0.03, 0.925, title + "  ·  реальный лучевой сенсор двойника "
             "(та же геометрия, что RTX-станция)", color=SUB, fontsize=14)

    # --- left: 3D cloud
    ax = fig.add_axes([0.02, 0.16, 0.46, 0.72], projection="3d")
    ax.set_facecolor(BG)
    xs = (pts[:, 0] - pts[:, 0].mean()) * 1000
    ys = (pts[:, 1] - pts[:, 1].mean()) * 1000
    step = max(1, len(pts) // 9000)
    ax.scatter(xs[::step], ys[::step], z_rel[::step], c=z_rel[::step],
               cmap="Blues_r", s=2.2, depthshade=False)
    ax.set_box_aspect((np.ptp(xs), np.ptp(ys), max(np.ptp(z_rel), 40)))
    ax.view_init(elev=28, azim=-55)
    for pane in (ax.xaxis, ax.yaxis, ax.zaxis):
        pane.set_pane_color((0.09, 0.09, 0.12, 1.0))
        pane.label.set_color(SUB)
        [t.set_color(SUB) for t in pane.get_ticklabels()]
    ax.set_xlabel("x, мм", labelpad=8)
    ax.set_ylabel("y, мм", labelpad=8)
    ax.set_zlabel("z, мм", labelpad=6)
    ax.set_title(f"{len(pts):,} точек · вид сенсора".replace(",", " "),
                 color=SUB, fontsize=12)

    # --- right: decisive section
    sec = best_section(verdict, pts)
    ax2 = fig.add_axes([0.55, 0.20, 0.41, 0.64])
    ax2.set_facecolor(PANEL)
    for sp in ax2.spines.values():
        sp.set_color("#2E323C")
    ax2.tick_params(colors=SUB)
    if sec is not None:
        vb, zb = sec["vb"] * 1000, sec["zb"] * 1000
        ax2.scatter(vb, zb, s=7, c="#7FB2FF", label="точки станции")
        # the estimator closes the section by mirror symmetry about the
        # resting mid-height (same closure as the ground-truth section) —
        # show the reconstructed half so r/R is visually traceable
        ax2.scatter(vb, -zb, s=7, facecolors="none", edgecolors="#4A5568",
                    linewidths=0.6, label="зеркальное замыкание")
        r_in, r_out = sec["r_in"] * 1000, sec["r_out"] * 1000
        th = np.linspace(0, 2 * np.pi, 200)
        ax2.plot(r_out * np.cos(th), r_out * np.sin(th), color=MAGENTA,
                 lw=1.6, ls="--", label=f"R описанная = {r_out:.0f} мм")
        ax2.plot(r_in * np.cos(th), r_in * np.sin(th), color="#33C759",
                 lw=1.6, ls="--", label=f"r вписанная = {r_in:.0f} мм")
        ax2.set_title(f"решающее сечение · ось {sec['axis_deg']:.0f}° "
                      f"({sec['metodo']}) · станция u={sec['s']*1000:+.0f} мм"
                      f"  →  r/R = {sec['ratio']:.3f}",
                      color=TXT, fontsize=13)
        ax2.legend(loc="lower right", facecolor=PANEL, edgecolor="#2E323C",
                   labelcolor=TXT, fontsize=10)
        ax2.set_aspect("equal")
    else:
        ax2.text(0.5, 0.5, "секций нет (объект мал/плосок)", color=SUB,
                 ha="center", transform=ax2.transAxes)
    ax2.set_xlabel("v, мм", color=SUB)
    ax2.set_ylabel("z − z_c, мм", color=SUB)

    # --- footer: measurement + verdict
    fig.add_artist(plt.Rectangle((0.02, 0.03), 0.96, 0.105, transform=fig.transFigure,
                                 facecolor=PANEL, edgecolor="#2E323C"))
    d = verdict.get("dims_mm")
    dims_s = " × ".join(f"{v:.0f}" for v in d) + " мм" if d is not None else "n/a"
    fig.text(0.045, 0.098, "ИЗМЕРЕНО", color=SUB, fontsize=12)
    fig.text(0.045, 0.055, dims_s, color=TXT, fontsize=26, family="monospace",
             fontweight="bold")
    ratio = verdict.get("max_ratio", verdict.get("ratio"))
    feats = (f"max r/R = {ratio:.3f}" if ratio is not None else "")
    reason = (verdict.get("reason") or verdict.get("rule") or "")[:110]
    fig.text(0.30, 0.098, "ПРИЗНАКИ · ПРАВИЛО", color=SUB, fontsize=12)
    fig.text(0.30, 0.075, feats, color=TXT, fontsize=14, family="monospace")
    fig.text(0.30, 0.048, reason, color=SUB, fontsize=11, family="monospace")
    fig.text(0.045, 0.145, note, color=SUB, fontsize=12)
    fig.add_artist(plt.Rectangle((0.78, 0.045), 0.19, 0.075,
                                 transform=fig.transFigure, facecolor=col))
    fig.text(0.795, 0.088, f"→ {zone}", color="#0C0D10", fontsize=30,
             fontweight="bold")
    fig.text(0.795, 0.058, ZONE_RU.get(zone, ""), color="#0C0D10",
             fontsize=10.5, fontweight="bold")

    out = out_dir / f"cloud_{slug}.png"
    fig.savefig(out, facecolor=BG)
    plt.close(fig)
    print(f"figura: {out}")


def main():
    out_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else \
        REPO / "docs" / "report" / "figures" / "clouds"
    out_dir.mkdir(parents=True, exist_ok=True)

    entries = probe_meshes()
    prev = register_custom(entries)
    try:
        model, manifest, _ = make_model()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        percep = LookaheadPerception(model)
        # park EVERYTHING first, then measure one item at a time — a
        # leftover item under the camera merges into the cloud and fakes
        # a giant measurement
        for idx, e in enumerate(manifest):
            park_item(model, data, e["slug"], idx, e)
        mujoco.mj_forward(model, data)

        def park(slug):
            idx = next(i for i, x in enumerate(manifest)
                       if x["slug"] == slug)
            park_item(model, data, slug, idx, manifest[idx])
            mujoco.mj_forward(model, data)

        for slug, mesh, title, note in entries:
            e = next(x for x in manifest if x["slug"] == slug)
            settle_item(model, data, slug, e, 0.0, 0.35)
            pts = percep.cloud(data)
            v = percep.analyze(pts)
            sections = v.get("sections") or []
            v["max_ratio"] = max((r for _, r in sections), default=None)
            make_figure(slug, title, note, pts, v, out_dir)
            park(slug)

        # officials, from their real meshes on the belt
        for slug in ("bottle", "box_s", "helmet", "edge_cube11"):
            try:
                e = next(x for x in manifest if x["slug"] == slug)
            except StopIteration:
                continue
            settle_item(model, data, slug, e, 0.0, 0.3)
            pts = percep.cloud(data)
            if pts is None or len(pts) < 40:
                print(f"{slug}: sin nube (¿fuera de manifiesto base?)")
                continue
            v = percep.analyze(pts)
            sections = v.get("sections") or []
            v["max_ratio"] = max((r for _, r in sections), default=None)
            titles = {"bottle": "Бутылка — эталон круга в сечении",
                      "box_s": "Коробка 317×206×199 — эталон B",
                      "helmet": "Шлем — купол (агрегатная улика)",
                      "edge_cube11": "Куб 11 мм — пол сертификации"}
            make_figure(slug, titles.get(slug, slug), "официальный набор",
                        pts, v, out_dir)
            park(slug)
    finally:
        cleanup_custom(prev, entries)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
