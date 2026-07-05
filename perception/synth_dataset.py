# -*- coding: utf-8 -*-
"""Synthetic detector dataset from the cell's own vision station.

Renders RGB frames of official items on conveyor A (randomized position/yaw,
occasionally two items) and auto-labels bounding boxes by projecting each
item's world AABB through the camera. Single class 'item' — the detector's
job in this architecture is localization/tracking only; the category always
comes from measured geometry.

    python -m perception.synth_dataset --n 300 --out datasets/belt_items
"""
import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import mujoco  # noqa: E402

from cell import params as P  # noqa: E402
from cell.scene import make_model  # noqa: E402

W, H = 640, 480


def project(pts_w, cam_pos, cam_mat, fx, fy, cx, cy):
    q = (pts_w - cam_pos) @ cam_mat            # world -> camera frame
    z = -q[:, 2]
    u = cx + fx * q[:, 0] / z
    v = cy - fy * q[:, 1] / z
    return u, v


def aabb_corners(model, data, gid):
    aabb = model.geom_aabb[gid]
    R = data.geom_xmat[gid].reshape(3, 3)
    c = data.geom_xpos[gid] + R @ aabb[:3]
    h = aabb[3:]
    corners = np.array([[sx * h[0], sy * h[1], sz * h[2]]
                        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    return c + corners @ R.T


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=300)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--out", default=str(ROOT / "datasets" / "belt_items"))
    ap.add_argument("--val-frac", type=float, default=0.2)
    args = ap.parse_args(argv)

    rng = np.random.default_rng(args.seed)
    model, manifest, _ = make_model(mode="table")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, height=H, width=W)

    cam_id = model.camera("lookahead").id
    fovy = np.deg2rad(model.cam_fovy[cam_id])
    fy = (H / 2) / np.tan(fovy / 2)
    fx = fy
    cx, cy = W / 2, H / 2

    out = Path(args.out)
    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        (out / sub).mkdir(parents=True, exist_ok=True)

    def park_all():
        for i, e in enumerate(manifest):
            jid = model.joint(f"fj_{e['slug']}").id
            qa, da = model.jnt_qposadr[jid], model.jnt_dofadr[jid]
            data.qpos[qa:qa + 3] = [0.6 + i * 0.85, -1.2, e["dims_m"][2] / 2 + 0.001]
            data.qpos[qa + 3:qa + 7] = [1, 0, 0, 0]
            data.qvel[da:da + 6] = 0

    def place(e, x, y, yaw):
        jid = model.joint(f"fj_{e['slug']}").id
        qa, da = model.jnt_qposadr[jid], model.jnt_dofadr[jid]
        data.qpos[qa:qa + 3] = [x, y, P.BELT_A["top"] + e["dims_m"][2] / 2 + 0.004]
        data.qpos[qa + 3:qa + 7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        data.qvel[da:da + 6] = 0

    n_val = int(args.n * args.val_frac)
    import imageio.v2 as imageio
    written = 0
    for i in range(args.n):
        park_all()
        picks = rng.choice(len(manifest), size=2 if rng.random() < 0.25 else 1, replace=False)
        placed = []
        x_slots = [float(rng.uniform(5.45, 6.4))]
        if len(picks) == 2:
            x_slots.append(x_slots[0] + float(rng.uniform(0.55, 0.9)) * rng.choice([-1, 1]))
        for k, idx in enumerate(picks):
            e = manifest[idx]
            diag = float(np.hypot(e["dims_m"][0], e["dims_m"][1]))
            yaw = (float(rng.uniform(-0.09, 0.09)) if diag > 0.48
                   else float(rng.uniform(0, 2 * np.pi)))
            place(e, np.clip(x_slots[k], 5.1, 6.6), 3.0 + float(rng.uniform(-0.05, 0.05)), yaw)
            placed.append(e)
        mujoco.mj_forward(model, data)
        for _ in range(150):
            mujoco.mj_step(model, data)

        renderer.update_scene(data, camera="lookahead")
        rgb = renderer.render()
        cam_pos = data.cam_xpos[cam_id]
        cam_mat = data.cam_xmat[cam_id].reshape(3, 3)
        labels = []
        for e in placed:
            gid = model.geom(f"g_{e['slug']}").id
            u, v = project(aabb_corners(model, data, gid), cam_pos, cam_mat, fx, fy, cx, cy)
            u0, u1 = np.clip([u.min(), u.max()], 0, W - 1)
            v0, v1 = np.clip([v.min(), v.max()], 0, H - 1)
            if u1 - u0 < 4 or v1 - v0 < 4:
                continue
            labels.append(f"0 {(u0 + u1) / 2 / W:.6f} {(v0 + v1) / 2 / H:.6f} "
                          f"{(u1 - u0) / W:.6f} {(v1 - v0) / H:.6f}")
        if not labels:
            continue
        split = "val" if i < n_val else "train"
        imageio.imwrite(out / f"images/{split}/{i:05d}.jpg", rgb, quality=92)
        (out / f"labels/{split}/{i:05d}.txt").write_text("\n".join(labels), encoding="utf-8")
        written += 1

    (out / "data.yaml").write_text(
        f"path: {out.resolve()}\ntrain: images/train\nval: images/val\n"
        f"names:\n  0: item\n", encoding="utf-8")
    print(f"dataset: {written} frames -> {out}")


if __name__ == "__main__":
    main()
