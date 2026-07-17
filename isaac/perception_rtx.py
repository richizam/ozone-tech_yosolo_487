# -*- coding: utf-8 -*-
"""RTX depth-camera perception for the Isaac Sim cell.

This is the genuine sensor-in-the-loop path the MuJoCo twin models with ray
casts: the overhead station camera RENDERS a real RTX depth frame as the item
crosses the vision window; we back-project it to a world-frame point cloud,
segment the item above the belt, measure its geometry, and classify it with
the OFFICIAL rule order — the category is derived from the rendered depth, not
from ground truth.

Rule order (cell/params.py, identical to the mesh reference classifier):
  1. dimensions gate first: any extent < 10 mm (undersize) or the sorted
     extents exceeding 450x320x320 mm (oversize)  -> C
  2. then shape: a circular cross-section (r_in/R_circ >= 0.8)              -> D
  3. otherwise                                                             -> B

Two depth-derived shape features reproduce the mesh classifier's
`max over principal axes of r_in/R_circ` from a single overhead view:
  * footprint circularity — angular radius ratio of the top-down silhouette
    (min/max sector radius about the centroid): a disk ~1.0, a square ~0.71,
    an oval detergent ~0.72 — matches the ground-truth axis ratios;
  * dome score — the fraction of the footprint whose top surface is NOT a flat
    plateau: boxes have flat tops (~0), a bottle/cylinder/helmet/sack dome is
    curved (~0.8). A circular cross-section always shows up in one or the other.

Everything is numpy-only (no scipy/OpenGL), deterministic, and headless-safe.
"""
import numpy as np

from cell import params as P

BELT_Z = P.BELT_A["top"]
LIMIT_MIN_MM = P.LIMIT_MIN_MM
LIMIT_MAX_MM = np.array(P.LIMIT_MAX_MM, dtype=float)


def quat_to_mat(q):
    """(w, x, y, z) -> 3x3 rotation matrix (columns are the rotated axes)."""
    w, x, y, z = [float(v) for v in q]
    n = math_sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def math_sqrt(v):
    return float(np.sqrt(v))


def fuse_reads(reads, guard_mm=6.0, circle_ratio=P.CIRCLE_RATIO,
               dome_tau=0.75, flank_tau_mm=6.0, window_x=None):
    """Fuse the window's multi-read evidence per the OFFICIAL decision order,
    with legal-metrology guard bands: a fused dimension within the measuring
    head's uncertainty of a limit cannot take the permissive branch, and zero
    valid reads divert to the manual-review lane — never to the sorter.

    DUAL-RANGE metrology: reads may carry `dims_macro_mm` from the
    close-range macro head (gsd ~0.4 mm). Macro-sourced dimensions are fused
    with a MEDIAN (unbiased) and certified against the macro guard band
    (10 mm + 2*gsd = ~10.8 mm), so an 11 mm cube honestly certifies as
    sortable while a 10 mm cube or a 9 mm pen stays conservatively C.
    Overhead-sourced dimensions keep the validated MIN fusion + 6 mm guard.

    Returns {'zone', 'reason', 'dims_mm', features..., 'n_reads', 'sensor_miss'}."""
    if window_x is None:
        w0, w1 = P.VIRTUAL_SENSOR["window_x"]
        window_x = (w0 - 0.12, w1 + 0.42)
    guard_macro = P.VIRTUAL_SENSOR.get("guard_macro_mm", 0.8)
    reads = [r for r in reads if r]
    # a read longer than any certifiable single item is a multi-item echo
    # (contact pair in the window) — invalid, never evidence
    reads = [r for r in reads if r["dims_mm"][0] <= 520.0]
    if not reads:
        return {"zone": "D", "reason": "sensor_miss -> manual-review lane",
                "dims_mm": None, "n_reads": 0, "sensor_miss": True,
                "confidence": 0.0}
    # drop partial-view reads: a cloud touching the crop boundary (a stale
    # frame from before the item fully entered the window) reads truncated
    # dims and smeared shape
    full = [r for r in reads
            if r.get("x_lo", window_x[0] + 1) > window_x[0] + 0.015
            and r.get("x_hi", window_x[1] - 1) < window_x[1] - 0.015]
    if full:
        reads = full
    # FRAME-COMPLETENESS GATE: a stale annotator frame of the 1 m/s item
    # yields a truncated sliver (tiny x-span / collapsed point count) whose
    # min() would poison every dimension AND whose distorted silhouette
    # would poison the shape features. A real dimensioning tunnel rejects
    # frames that fail completeness checks; reads of the SAME item at the
    # SAME resting yaw must agree on span and density.
    if len(reads) >= 2:
        spans = np.array([float(r.get("x_hi", 0.0)) - float(r.get("x_lo", 0.0))
                          for r in reads])
        npts = np.array([float(r.get("n_points", 0)) for r in reads])
        good = [r for r, sp, pn in zip(reads, spans, npts)
                if sp >= 0.65 * float(spans.max())
                and pn >= 0.25 * float(npts.max())]
        if good:
            reads = good

    # DUAL-BOUND per-axis fusion (each read sorted desc): report the median;
    # certify UNDERSIZE against the LOW bound (25th pct — conservative: a
    # residual truncated read can only push toward the safe C branch) and
    # OVERSIZE against the HIGH bound (75th pct — an under-measuring read can
    # never hide a too-big item). Both bounds are honest per-frame data.
    arr = np.array([r["dims_mm"] for r in reads], dtype=float)
    dims = np.median(arr, axis=0)
    dims_lo = np.percentile(arr, 25, axis=0)
    dims_hi = np.percentile(arr, 75, axis=0)
    guards = np.full(3, float(guard_mm))
    macro_reads = [r for r in reads if r.get("dims_macro_mm")]
    macro_note = ""
    if macro_reads:
        dm = np.median(np.array([r["dims_macro_mm"] for r in macro_reads],
                                dtype=float), axis=0)
        if dims[0] <= 240.0:
            # the whole item fits the macro footprint: certify all three
            # dimensions from the close-range head
            dims = dims_lo = dims_hi = dm
            guards = np.full(3, guard_macro)
        else:
            # long item: length from overhead, the two smallest (the
            # undersize-critical ones) from the macro head
            merged = sorted([(dims[0], guard_mm, dims_lo[0], dims_hi[0]),
                             (float(dm[1]), guard_macro, float(dm[1]),
                              float(dm[1])),
                             (float(dm[2]), guard_macro, float(dm[2]),
                              float(dm[2]))],
                            key=lambda p: -p[0])
            dims = np.array([m[0] for m in merged])
            guards = np.array([m[1] for m in merged])
            dims_lo = np.array([m[2] for m in merged])
            dims_hi = np.array([m[3] for m in merged])
        macro_note = (f" [macro head: {len(macro_reads)} reads, floor "
                      f"{LIMIT_MIN_MM + guard_macro:.1f} mm]")
    # shape features across ALL accepted frames (the completeness gate above
    # already rejected degenerate reads). CIRCLE EVIDENCE aggregates at the
    # 75th percentile — the SAFE direction: roundness seen clearly in several
    # frames counts as evidence even when motion smear erases it in others
    # (the helmet's per-frame circularity swings 0.13..0.75 with the frame
    # phase). The B-blocking gates are immune: a box's dome is ~0 and the
    # jug's aspect is 0.79 in EVERY frame.
    top = reads

    def med(key):
        return float(np.median([r[key] for r in top]))

    def hi(key):
        return float(np.percentile([r[key] for r in top], 75))

    circ, sect, dome = (hi("footprint_circularity"), hi("section_ratio"),
                        hi("dome_score"))
    # swept-axis section channel (hex blind-spot fix): certified-circle
    # evidence only — the safe-side rules below keep reading the
    # primary-axis section untouched
    sect_sw = float(np.percentile(
        [r.get("section_sweep", r["section_ratio"]) for r in top], 75))
    elong = med("elongation")
    aspect = med("aspect_hw")
    flanks = [float(r["flank_mm"]) for r in top if float(r["flank_mm"]) >= 0.0]
    flank = float(np.median(flanks)) if flanks else None
    d_votes = sum(1 for r in reads if r["zone"] == "D")
    under = bool(np.any(dims_lo < LIMIT_MIN_MM + guards))
    # oversize also certifies on the LOW (de-smeared) bound: after the
    # completeness gate, motion mixing can only INFLATE an extent — the low
    # bound is the honest estimate, and a genuinely too-big item exceeds the
    # limit in its cleanest frames too (445 mm at a 444 mm guard line stays
    # honest borderline; 460 mm is caught in every frame)
    over = bool(np.any(np.sort(dims_lo)[::-1] > LIMIT_MAX_MM - guard_mm))
    flank_s = "n/a" if flank is None else f"{flank:.1f}mm"
    feats = (f"circ={circ:.2f} sect={sect:.2f} sweep={sect_sw:.2f} "
             f"dome={dome:.2f} flank={flank_s} elong={elong:.1f} "
             f"h/w={aspect:.2f}")
    if under:
        i_min = int(np.argmin(dims_lo - guards))
        zone, reason = "C", (f"undersize: dim {dims_lo[i_min]:.1f} mm below "
                             f"certification floor "
                             f"{LIMIT_MIN_MM + guards[i_min]:.1f} mm"
                             + macro_note)
    elif over:
        zone, reason = "C", f"oversize (guard band {guard_mm:.0f} mm)"
    elif (circ >= circle_ratio or sect >= circle_ratio
          or sect_sw >= circle_ratio
          or (dome >= dome_tau and aspect >= 0.85)
          or (circ >= 0.78 and dome >= 0.40)
          or (circ >= 0.60 and dome >= 0.35 and aspect >= 0.85)):
        # last clause: TALL ROUND DOME, triple-gated — near-circular
        # footprint AND curved top AND as tall as it is wide (helmet:
        # circ 0.63 / dome 0.48 / h/w 0.94 from the upstream oblique head).
        # Each B item is blocked on a DIFFERENT gate with margin:
        # detergent by aspect (0.79), box_s/lunchbox by dome (0.0/0.15),
        # the 11 mm cube (square, circ 0.71) by dome (~0), bottle by circ
        zone, reason = "D", f"circle evidence: {feats}"
    elif elong >= 2.2 and (flank is None or sect >= 0.38):
        # SAFE-SIDE: an elongated prism goes to the sorter ONLY with an
        # affirmatively rectangular section (sect < 0.38). The mirror-closure
        # section of the hex-prism trap reads 0.38-0.62 at DWS fidelity —
        # inside the sensor's uncertainty of the 0.8 circle criterion — and a
        # designed 0.78 squircle / 16x12 mm rod cannot be certified non-round
        # either: all of them divert to repack review, never to the sorter.
        zone, reason = "D", ("ambiguous prism section on elongated item "
                             f"(safe side): {feats}")
    elif d_votes >= 2 and 2 * d_votes >= len(reads):
        zone, reason = "D", f"persistent D evidence: {d_votes}/{len(reads)} reads"
    else:
        zone, reason = "B", f"fits, no circle evidence: {feats}" + macro_note
    # confidence = per-read zone agreement with the fused verdict (logged
    # evidence quality, not a routing gate — the guard bands and safe-side
    # rules above already make the low-evidence decisions)
    votes = sum(1 for r in reads if r["zone"] == zone)
    return {"zone": zone, "reason": reason,
            "dims_mm": [round(float(v), 1) for v in dims],
            "guards_mm": [round(float(g), 2) for g in guards],
            "macro_reads": len(macro_reads),
            "footprint_circularity": round(circ, 3),
            "section_ratio": round(sect, 3), "dome_score": round(dome, 3),
            "flank_mm": None if flank is None else round(flank, 1),
            "n_reads": len(reads),
            "d_votes": d_votes, "sensor_miss": False,
            "confidence": round(votes / len(reads), 3)}


class RTXPerception:
    """Multi-head RTX depth station -> B/C/D verdict.

    Mirrors the VIRTUAL_SENSOR architecture from cell/params.py with real
    rendered sensors: one overhead depth head plus two side heads (the side
    heads see the section below its widest line — an overhead view alone
    cannot distinguish a hex prism from a box at DWS sampling density)."""

    def __init__(self, camera, belt_z=BELT_Z, z_margin=0.006,
                 circle_ratio=P.CIRCLE_RATIO, dome_tau=0.75, elong_min=1.8,
                 window_x=None, belt_y=P.BELT_A["y"],
                 belt_half_w=P.BELT_A["width"] / 2 - 0.01,
                 grid_mm=4.0, min_points=60, macro=None):
        # belt_half_w stays INSIDE the physical side guides (rails at
        # width/2 + 0.015): the guides are permanent hardware in the frame
        # and merging their slivers into the item cloud reads as oversize
        if window_x is None:
            w0, w1 = P.VIRTUAL_SENSOR["window_x"]
            window_x = (w0 - 0.12, w1 + 0.42)      # crop wider than the read
                                                   # window, ends before the
                                                   # escapement slab
        self.cams = list(camera) if isinstance(camera, (list, tuple)) else [camera]
        self.cam = self.cams[0]                    # primary (overhead) head
        self.macro = macro                         # close-range head (dual
                                                   # range small-item metrology)
        self.macro_engage_mm = P.VIRTUAL_SENSOR.get("macro_engage_mm", 25.0)
        self.macro_gsd_mm = P.VIRTUAL_SENSOR.get("macro_gsd_mm", 0.4)
        self.belt_z = belt_z
        self.z_margin = z_margin
        self.circle_ratio = circle_ratio    # official 0.8 (footprint + section)
        self.dome_tau = dome_tau            # curved-top blobs (sack/helmet class)
        self.elong_min = elong_min          # run the section test on lying prisms
        self.window_x = window_x
        self.belt_y = belt_y
        self.belt_half_w = belt_half_w
        self.grid = grid_mm / 1000.0
        self.min_points = min_points

    # ------------------------------------------------------- depth -> world XYZ
    @staticmethod
    def _cam_pose_usd(cam):
        """Camera position + rotation (columns = USD camera axes in world),
        read straight from the USD prim transform. Camera.get_world_pose()
        uses the world-axes convention (x-forward) — NOT what the pinhole
        back-projection below assumes — so we bypass it."""
        from pxr import Usd, UsdGeom
        xf = UsdGeom.Xformable(cam.prim)
        m = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        # Gf.Matrix4d is row-vector convention: rows 0..2 = local axes in world
        R = np.array([[m[0][0], m[0][1], m[0][2]],
                      [m[1][0], m[1][1], m[1][2]],
                      [m[2][0], m[2][1], m[2][2]]], dtype=float).T
        pos = np.array([m[3][0], m[3][1], m[3][2]], dtype=float)
        return pos, R

    @classmethod
    def _head_grid(cls, cam, edge_filter=False, edge_tau=0.008):
        """Full-frame unprojection, image shape preserved: (world HxWx3,
        valid HxW). The measuring path flattens this via _head_points; the
        evidence export keeps the pixel layout so a saved segmentation mask
        overlays the saved RGB/depth still 1:1."""
        frame = cam.get_current_frame()
        depth = frame.get("distance_to_image_plane")
        if depth is None:
            return None, None
        depth = np.asarray(depth, dtype=np.float32)
        if depth.ndim != 2 or depth.size == 0:
            return None, None
        H, W = depth.shape
        K = np.asarray(cam.get_intrinsics_matrix(), dtype=float)
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        pos, R = cls._cam_pose_usd(cam)
        us, vs = np.meshgrid(np.arange(W), np.arange(H))
        D = depth
        valid = np.isfinite(D) & (D > 0.05) & (D < 4.0)
        if edge_filter:
            # silhouette-edge smear: a grazing ray at the item's outline
            # returns depth interpolated toward the background, which
            # back-projects to a phantom point along the ray that still
            # passes the height filter. Drop high-gradient pixels and erode
            # the survivor mask by one pixel.
            gv, gu = np.gradient(np.nan_to_num(D, nan=1e3))
            sharp = (np.abs(gu) < edge_tau) & (np.abs(gv) < edge_tau)
            er = sharp.copy()
            er[1:, :] &= sharp[:-1, :]
            er[:-1, :] &= sharp[1:, :]
            er[:, 1:] &= sharp[:, :-1]
            er[:, :-1] &= sharp[:, 1:]
            valid &= er
        xc = (us - cx) / fx * D                     # camera +X (right)
        yc = -(vs - cy) / fy * D                    # camera +Y (up); v grows down
        zc = -D                                     # look along -Z
        world = pos + np.stack([xc, yc, zc], axis=-1) @ R.T
        return world, valid

    @classmethod
    def _head_points(cls, cam, edge_filter=False, edge_tau=0.008):
        world, valid = cls._head_grid(cam, edge_filter, edge_tau)
        if world is None:
            return None
        return world.reshape(-1, 3)[valid.reshape(-1)]

    def _points_world(self):
        """Overhead cloud only — the metrology head (dims/footprint/dome)."""
        return self._head_points(self.cams[0])

    def _points_side(self):
        """Edge-filtered side-profiler clouds (section reconstruction only)."""
        clouds = []
        for cam in self.cams[1:]:
            pts = self._head_points(cam, edge_filter=True)
            if pts is not None and len(pts):
                clouds.append(pts)
        return np.concatenate(clouds, axis=0) if clouds else None

    def _segment(self, world, exclude_x=()):
        # measuring volume: bounded ABOVE by the max inbound envelope
        # (0.5 m + margin) like a real dimensioner's specified volume — the
        # macro head's drop-tube mount lives above this ceiling and can never
        # enter a segmentation
        m = ((world[:, 2] > self.belt_z + self.z_margin)
             & (world[:, 2] < self.belt_z + 0.56)
             & (np.abs(world[:, 1] - self.belt_y) < self.belt_half_w)
             & (world[:, 0] > self.window_x[0]) & (world[:, 0] < self.window_x[1]))
        # known-hardware exclusion: a RAISED stop blade inside the crop is
        # commanded state, not freight — drop its slab or it merges with the
        # item's cloud and explodes the measured dims
        for x0, x1 in exclude_x:
            m &= ~((world[:, 0] > x0) & (world[:, 0] < x1))
        return world[m]

    # ----------------------------------------------------------------- features
    @staticmethod
    def _footprint_circularity(xy, k=36):
        """min/max sector radius about the centroid ~ r_in/R_circ of the
        top-down silhouette. Robust to a few stray points via percentiles."""
        c = xy.mean(axis=0)
        d = xy - c
        r = np.hypot(d[:, 0], d[:, 1])
        ang = np.arctan2(d[:, 1], d[:, 0])
        bins = ((ang + np.pi) / (2 * np.pi) * k).astype(int) % k
        sect = np.full(k, np.nan)
        for b in range(k):
            rr = r[bins == b]
            if rr.size:
                sect[b] = np.percentile(rr, 90)     # outer contour in that sector
        sect = sect[np.isfinite(sect)]
        if sect.size < k // 2:
            return 0.0
        lo, hi = np.percentile(sect, 10), np.percentile(sect, 90)
        return float(lo / hi) if hi > 1e-6 else 0.0

    def _raster(self, obj, axis1, axis2, c):
        """Top-surface height field over the item's principal-axis grid."""
        t1 = (obj[:, :2] - c) @ axis1
        t2 = (obj[:, :2] - c) @ axis2
        h = obj[:, 2] - self.belt_z
        n1 = max(1, int((t1.max() - t1.min()) / self.grid) + 1)
        n2 = max(1, int((t2.max() - t2.min()) / self.grid) + 1)
        i1 = np.clip(((t1 - t1.min()) / self.grid).astype(int), 0, n1 - 1)
        i2 = np.clip(((t2 - t2.min()) / self.grid).astype(int), 0, n2 - 1)
        top = np.zeros((n1, n2), dtype=float)
        np.maximum.at(top, (i1, i2), h)
        return top, top > 1e-4

    @staticmethod
    def _dome_score(top, filled):
        """Fraction of the footprint whose top surface is not a flat plateau."""
        if filled.sum() < 6:
            return 0.0
        Hmax = float(top[filled].max())
        if Hmax < 1e-3:
            return 0.0
        plateau = (top >= 0.85 * Hmax) & filled
        return float(1.0 - plateau.sum() / filled.sum())

    @staticmethod
    def _hull2d(pts):
        """Convex hull of 2D points (Andrew monotone chain), CCW, no scipy."""
        P = np.unique(np.round(pts, 5), axis=0)
        if len(P) < 3:
            return P
        P = P[np.lexsort((P[:, 1], P[:, 0]))]

        def cross2(o, a, b):
            # explicit 2D cross: numpy 2.0 removed np.cross on 2-vectors, so
            # the old call ran only under Isaac's bundled numpy 1.x and threw
            # for anyone using the pinned requirements (numpy 2.x)
            return ((a[0] - o[0]) * (b[1] - o[1])
                    - (a[1] - o[1]) * (b[0] - o[0]))

        def half(seq):
            out = []
            for p in seq:
                while len(out) >= 2 and cross2(out[-2], out[-1], p) <= 0:
                    out.pop()
                out.append(p)
            return out

        lower = half(P)
        upper = half(P[::-1])
        return np.array(lower[:-1] + upper[:-1])

    @staticmethod
    def _plateau_frac(top, filled):
        """Median over mid-slices of (width of the near-max plateau) / (full
        slice width). A lying box's flat top spans its width (~1.0); a hex
        prism's top facet spans half (~0.5); a cylinder tends to ~0.1. Robust
        to a small resting tilt, unlike the mirrored-hull reconstruction."""
        n1 = top.shape[0]
        lo, hi = int(0.2 * n1), max(int(0.2 * n1) + 1, int(0.8 * n1))
        fracs = []
        for i in range(lo, hi):
            row = top[i][filled[i]]
            if row.size >= 4 and row.max() > 0.02:
                fracs.append(float((row >= 0.90 * row.max()).sum() / row.size))
        return float(np.median(fracs)) if fracs else 1.0

    @staticmethod
    def _flank_range_mm(y_abs, z, zbin=0.005):
        """Range (mm) of the flank's median lateral offset across height bins
        spanning the mid 70% of the visible flank. Vertical box wall -> ~0;
        hex slant / cylinder curve -> the flank offset shifts with height."""
        if len(z) < 40:
            return -1.0                            # insufficient: NOT "vertical"
        z0, z1 = np.percentile(z, 10), np.percentile(z, 90)
        if z1 - z0 < 0.012:
            return -1.0
        nb = max(3, int((z1 - z0) / zbin))
        idx = np.clip(((z - z0) / (z1 - z0) * nb).astype(int), 0, nb - 1)
        med = np.full(nb, np.nan)
        for b in range(nb):
            yy = y_abs[idx == b]
            if yy.size >= 4:
                med[b] = np.median(yy)
        med = med[np.isfinite(med)]
        if med.size < 3:
            return -1.0
        return float((np.max(med) - np.min(med)) * 1000.0)

    @staticmethod
    def _radial_section_sweep(xy_c, hh, ang0, height, n_axes=12):
        """Max radial-bin section ratio over swept cutting axes — the MuJoCo
        twin's validated formulation, ported verbatim: 7 body stations per
        axis, +-5 mm bands, outer radius per 15-deg angular bin over the
        30..150-deg window about the resting axis (v_center, height/2);
        ratio = min/max of the bin maxima. A lying hexagon reads ~0.89 on
        the axis aligned with the prism; boxes stay <= ~0.72 on EVERY axis
        (the 120-deg window keeps their 45-deg corners)."""
        if height < 0.008:
            return 0.0
        zc = 0.5 * height
        edges = np.linspace(np.deg2rad(30), np.deg2rad(150), 9)
        best = 0.0
        for k_ax in range(n_axes):
            a_c = ang0 + k_ax * (np.pi / n_axes)
            u = xy_c @ np.array([np.cos(a_c), np.sin(a_c)])
            v = xy_c @ np.array([-np.sin(a_c), np.cos(a_c)])
            u_min, u_max = float(u.min()), float(u.max())
            if u_max - u_min < 0.11:
                continue
            for s in np.linspace(u_min + 0.05, u_max - 0.05, 7):
                band = np.abs(u - s) < 0.005
                if int(band.sum()) < 12:
                    continue
                vb, zb = v[band], hh[band]
                if float(zb.max()) < 0.008:
                    continue
                # mirror closure about the resting axis (same philosophy as
                # _section_ratio): the overhead head sees only the TOP
                # surface — without the reconstructed lower half a box band
                # is a single roof line and its ratio reads circular
                vb = np.concatenate([vb, vb])
                zb = np.concatenate([zb, height - zb])
                vc = 0.5 * (float(vb.min()) + float(vb.max()))
                theta = np.arctan2(zb - zc, vb - vc)
                rho = np.hypot(vb - vc, zb - zc)
                rho_out = []
                for lo_e, hi_e in zip(edges[:-1], edges[1:]):
                    m = (theta >= lo_e) & (theta < hi_e)
                    if m.any():
                        rho_out.append(float(rho[m].max()))
                # ALL 8 bins must be populated: a corner-clipping diagonal
                # cut of a box leaves the extreme bins empty (its telltale
                # corners vanish with them) and the surviving mid-bins read
                # circular — the twin never sees this because its side-fan
                # flanks always populate the low bins. A genuine circular
                # section closes the full contour after mirroring.
                if len(rho_out) == 8:
                    r = min(rho_out) / max(rho_out)
                    if r > best:
                        best = r
        return float(best)

    @classmethod
    def _section_ratio(cls, t1, t2, h):
        """r_in/R_circ of the transverse cross-section, reconstructed from the
        overhead profile — the OFFICIAL 0.8 criterion applied to depth data.

        Points of the middle 60% of the item's length are projected to the
        (t2, h) section plane and the top profile is mirrored about the
        mid-height plane (items resting on a face/line are symmetric there:
        a lying cylinder's section closes to the full circle, a hex prism to
        the full hexagon, a box to the full rectangle). On the section's
        convex hull:  R = max centroid-to-vertex distance,  r_in = min
        centroid-to-edge distance — the exact inscribed/circumscribed ratio:
        circle 1.0, hexagon 0.866, square 0.707, box_s section 0.70."""
        span = t1.max() - t1.min()
        mid = (t1 > t1.min() + 0.2 * span) & (t1 < t1.max() - 0.2 * span)
        if mid.sum() < 30:
            return 0.0
        y, z = t2[mid], h[mid]
        zmax = np.percentile(z, 99)
        pts = np.concatenate([np.stack([y, z], axis=1),
                              np.stack([y, zmax - z], axis=1)], axis=0)
        hull = cls._hull2d(pts)
        if len(hull) < 3:
            return 0.0
        c = hull.mean(axis=0)
        R = float(np.max(np.linalg.norm(hull - c, axis=1)))
        if R < 1e-6:
            return 0.0
        r_in = np.inf
        for i in range(len(hull)):
            a, b = hull[i], hull[(i + 1) % len(hull)]
            ab = b - a
            L2 = float(ab @ ab)
            t = 0.0 if L2 < 1e-12 else float(np.clip((c - a) @ ab / L2, 0, 1))
            r_in = min(r_in, float(np.linalg.norm(a + t * ab - c)))
        return float(r_in / R)

    # -------------------------------------------------------------- classify
    @staticmethod
    def _identity_gate(obj, x_hint, gap=0.06):
        """Keep only the x-connected cluster containing the tracked item's
        dead-reckoned position: another item co-occupying the crop (ahead
        past the gate, or a tailgater) must never be committed under this
        item's identity (MuJoCo pipeline's identity gate)."""
        if x_hint is None or len(obj) == 0:
            return obj
        xs = np.sort(obj[:, 0])
        breaks = np.nonzero(np.diff(xs) > gap)[0]
        seg_lo = [xs[0]] + [xs[i + 1] for i in breaks]
        seg_hi = [xs[i] for i in breaks] + [xs[-1]]
        best = None
        for lo, hi in zip(seg_lo, seg_hi):
            mid_d = 0.0 if lo <= x_hint <= hi else min(abs(x_hint - lo),
                                                       abs(x_hint - hi))
            if best is None or mid_d < best[0]:
                best = (mid_d, lo, hi)
        _, lo, hi = best
        return obj[(obj[:, 0] >= lo - 1e-6) & (obj[:, 0] <= hi + 1e-6)]

    # ------------------------------------------------- close-range macro head
    def _measure_macro(self, exclude_x=(), x_hint=None):
        """Small-item metrology from the close-range head: sub-millimetre
        ground sampling, edge-filtered + eroded cloud, extents with a
        half-pixel-per-side silhouette compensation (the gradient erosion
        removes the outermost sample ring). Returns (dims_mm sorted desc,
        xy cloud, obj cloud) or None."""
        if self.macro is None:
            return None
        pts = self._head_points(self.macro, edge_filter=True)
        if pts is None or not len(pts):
            return None
        # the macro head's OWN z floor: 1.5 mm above the calibrated belt
        # plane (RTX depth is noise-free at 0.4 mm GSD) — a 2 mm card is a
        # measurable item here, not a sensor miss like on the overhead head
        m = ((pts[:, 2] > self.belt_z + 0.0015)
             & (pts[:, 2] < self.belt_z + 0.56)
             & (np.abs(pts[:, 1] - self.belt_y) < self.belt_half_w)
             & (pts[:, 0] > self.window_x[0]) & (pts[:, 0] < self.window_x[1]))
        for x0, x1 in exclude_x:
            m &= ~((pts[:, 0] > x0) & (pts[:, 0] < x1))
        obj = pts[m]
        obj = self._identity_gate(obj, x_hint)
        if len(obj) < 25:
            return None
        xy = obj[:, :2]
        c = xy.mean(axis=0)
        _, _, vt = np.linalg.svd(xy - c, full_matrices=False)
        t1 = (xy - c) @ vt[0]
        t2 = (xy - c) @ vt[1]
        comp = self.macro_gsd_mm / 1000.0          # one gsd total (half/side)
        L = float(np.percentile(t1, 99.7) - np.percentile(t1, 0.3)) + comp
        Wd = float(np.percentile(t2, 99.7) - np.percentile(t2, 0.3)) + comp
        Hh = float(np.percentile(obj[:, 2], 99.7) - self.belt_z)
        dims = sorted([L * 1000.0, Wd * 1000.0, Hh * 1000.0], reverse=True)
        return dims, xy, obj

    # ------------------------------------------------------ evidence export
    def _mask_head(self, cam, macro=False, exclude_x=(), x_hint=None):
        """Pixel-space segmentation of one head's CURRENT frame with the
        measuring pipeline's own filters: same measuring-volume crop and
        z-floor as _segment()/_measure_macro(), same raised-hardware
        exclusions, same identity gate. Returns (mask HxW bool, cloud Nx3)."""
        world, valid = self._head_grid(cam, edge_filter=macro)
        if world is None:
            return None, None
        x, y, z = world[..., 0], world[..., 1], world[..., 2]
        z_lo = 0.0015 if macro else self.z_margin  # macro's own z floor
        m = (valid & (z > self.belt_z + z_lo) & (z < self.belt_z + 0.56)
             & (np.abs(y - self.belt_y) < self.belt_half_w)
             & (x > self.window_x[0]) & (x < self.window_x[1]))
        for x0, x1 in exclude_x:
            m &= ~((x > x0) & (x < x1))
        if x_hint is not None and m.any():
            kept = self._identity_gate(world[m], x_hint)
            if len(kept):                       # clusters are x-disjoint, so
                m &= ((x >= kept[:, 0].min() - 1e-6)   # the x-range filter IS
                      & (x <= kept[:, 0].max() + 1e-6))  # the cluster choice
        return m, world[m]

    def export_masks(self, exclude_x=(), x_hint=None):
        """Evidence for the saved stills: the pipeline's OWN segmentation
        mask + world-space item cloud per head, pixel-aligned with the
        frame the stills capture saves. Nothing here is a re-derivation —
        it is the measuring code path evaluated per pixel."""
        out = {"overhead": self._mask_head(self.cams[0], False,
                                           exclude_x, x_hint)}
        out["macro"] = (self._mask_head(self.macro, True, exclude_x, x_hint)
                        if self.macro is not None else (None, None))
        return out

    def measure(self, debug=False, exclude_x=(), x_hint=None):
        """Return a dict with dims/features/verdict, or None on a sensor miss.
        exclude_x: [(x0, x1), ...] slabs of known raised hardware to drop.
        x_hint: dead-reckoned x of the tracked item (identity gate)."""
        world = self._points_world()
        if world is None:
            if debug:
                print("[rtx] MISS: no depth frame", flush=True)
            return None
        obj = self._segment(world, exclude_x)
        obj = self._identity_gate(obj, x_hint)
        if len(obj) < self.min_points:
            # SMALL-ITEM PATH: freight below the overhead head's reliable
            # sampling (an 11 mm cube is ~7 px there) is measured by the
            # close-range macro head instead of being declared a miss.
            m = self._measure_macro(exclude_x, x_hint)
            if m is not None:
                dims_mm, mxy, mobj = m
                circ = self._footprint_circularity(mxy)
                mc = mxy.mean(axis=0)
                _, _, mvt = np.linalg.svd(mxy - mc, full_matrices=False)
                top, filled = self._raster(mobj, mvt[0], mvt[1], mc)
                dome = self._dome_score(top, filled)
                guard_macro = P.VIRTUAL_SENSOR.get("guard_macro_mm", 0.8)
                under = any(d < LIMIT_MIN_MM + guard_macro for d in dims_mm)
                if under:
                    zone, reason = "C", ("undersize (macro head, floor "
                                         f"{LIMIT_MIN_MM + guard_macro:.1f} mm)")
                elif circ >= self.circle_ratio or dome >= self.dome_tau:
                    zone, reason = "D", f"circle (macro): circ={circ:.2f}"
                else:
                    zone, reason = "B", (f"fits (macro head): circ={circ:.2f} "
                                         f"dome={dome:.2f}")
                elong = (dims_mm[0] / dims_mm[1]) if dims_mm[1] > 1e-6 else 99.0
                return {
                    "zone": zone, "reason": reason,
                    "dims_mm": [round(float(v), 2) for v in dims_mm],
                    "dims_macro_mm": [round(float(v), 2) for v in dims_mm],
                    "footprint_circularity": round(circ, 3),
                    "section_ratio": 0.0, "dome_score": round(dome, 3),
                    "flank_mm": -1.0, "plateau_frac": 1.0,
                    "aspect_hw": round(dims_mm[2] / max(dims_mm[1], 1e-6), 3),
                    "elongation": round(elong, 2),
                    "n_points": int(len(mobj)), "head": "macro",
                    "x_lo": round(float(mobj[:, 0].min()), 3),
                    "x_hi": round(float(mobj[:, 0].max()), 3),
                }
            if debug and len(world):
                lo = world.min(axis=0)
                hi = world.max(axis=0)
                above = int((world[:, 2] > self.belt_z + self.z_margin).sum())
                print(f"[rtx] MISS: {len(world)} pts, bounds x[{lo[0]:.2f},{hi[0]:.2f}] "
                      f"y[{lo[1]:.2f},{hi[1]:.2f}] z[{lo[2]:.2f},{hi[2]:.2f}] "
                      f"above_belt={above} segmented={len(obj)}", flush=True)
            return None
        # STALENESS GATE against the cell's own item odometry: a depth frame
        # whose cloud sits well BEHIND the tracked position is a stale
        # render of the moving item (no legitimate geometry lags the track —
        # a crop-clipped entry biases the cloud FORWARD, never back).
        if x_hint is not None and len(obj):
            mid = 0.5 * (float(obj[:, 0].min()) + float(obj[:, 0].max()))
            if x_hint - mid > 0.20:
                return None
        xy = obj[:, :2]
        c = xy.mean(axis=0)
        # principal horizontal axes (PCA on the footprint)
        _, _, vt = np.linalg.svd(xy - c, full_matrices=False)
        axis1, axis2 = vt[0], vt[1]
        t1 = (xy - c) @ axis1
        t2 = (xy - c) @ axis2
        # robust extents (1st/99th percentile) to shed depth speckle
        L = float(np.percentile(t1, 99) - np.percentile(t1, 1))
        Wd = float(np.percentile(t2, 99) - np.percentile(t2, 1))
        Hh = float(np.percentile(obj[:, 2], 99.5) - self.belt_z)
        dims_mm = np.array(sorted([L * 1000, Wd * 1000, Hh * 1000], reverse=True))

        circ = self._footprint_circularity(xy)
        top, filled = self._raster(obj, axis1, axis2, c)
        dome = self._dome_score(top, filled)
        elong = (L / Wd) if Wd > 1e-6 else 99.0
        # section r_in/R from the overhead profile (mirror closure) — clean
        # for round bodies resting symmetrically (bottle ~0.95)
        sect = 0.0
        hh = obj[:, 2] - self.belt_z
        if elong >= self.elong_min:
            sect = self._section_ratio(t1, t2, hh)
        # AXIS-SWEPT radial section (hex-prism blind-spot fix, mirrors the
        # MuJoCo twin's validated method): the footprint PCA axis is
        # DEGENERATE for near-square footprints — a squat prism at an odd
        # yaw gets diagonal cutting stations whose contour reads
        # rectangle-like (a lying hex read 0.73 instead of 0.866). The
        # official criterion is a circular section about ANY axis, so body
        # stations are scanned over 12 candidate axes with the twin's
        # radial-bin ratio (outer radius per angular bin about the resting
        # axis) and the evidence keeps the maximum. Feeds ONLY the
        # certified-circle rule (>= 0.8) through its own channel; the
        # safe-side ambiguous-prism rule keeps reading the primary-axis
        # mirror section, so elongated B freight cannot be pushed into D
        # by a diagonal cut. Validated in the twin: lying hex 0.89,
        # boxes <= 0.72 on every axis (officials + 30-shape corpus).
        ang0 = float(np.arctan2(axis1[1], axis1[0]))
        sect_sweep = self._radial_section_sweep(xy - c, hh, ang0,
                                                float(np.percentile(hh, 99.5)))
        sect_sweep = max(sect_sweep, sect)
        # side profiler heads: flank verticality, for ALL items. A box or a
        # jug presents a near-VERTICAL wall (offset ~constant in z); a lying
        # cylinder/hex flank slants by millimetres; a dome (helmet) sweeps
        # inward by ~100 mm over its height — vertical-section circle
        # evidence the overhead view cannot measure. Immune to resting tilt.
        flank_mm = -1.0                            # -1 = no side data
        side = self._points_side()
        if side is not None:
            sd = self._segment(side, exclude_x)
            if len(sd) >= 60:
                st1 = (sd[:, :2] - c) @ axis1
                st2 = (sd[:, :2] - c) @ axis2
                sz = sd[:, 2] - self.belt_z
                span = t1.max() - t1.min()
                m = (st1 > t1.min() + 0.2 * span) & (st1 < t1.max() - 0.2 * span)
                if m.sum() >= 40:
                    flank_mm = self._flank_range_mm(np.abs(st2[m]), sz[m])
        plateau = (self._plateau_frac(top, filled) if elong >= self.elong_min
                   else 1.0)
        aspect_hw = (Hh / Wd) if Wd > 1e-6 else 0.0

        # DUAL RANGE: when the smallest overhead dimension is inside the
        # macro engage band, re-measure the undersize-critical dimensions on
        # the close-range head (sub-mm gsd). The fusion certifies them
        # against the macro guard band.
        dims_macro = None
        if self.macro is not None and float(dims_mm.min()) < self.macro_engage_mm:
            m = self._measure_macro(exclude_x, x_hint)
            if m is not None:
                dims_macro = [round(float(v), 2) for v in m[0]]

        check_dims = np.array(dims_macro) if (
            dims_macro is not None and dims_mm[0] <= 240.0) else dims_mm
        if dims_macro is not None and dims_mm[0] > 240.0:
            check_dims = np.array([dims_mm[0]] + dims_macro[1:])
        undersize = bool(np.any(check_dims < LIMIT_MIN_MM))
        oversize = bool(np.any(dims_mm > LIMIT_MAX_MM))
        if undersize or oversize:
            zone, reason = "C", ("undersize" if undersize else "oversize")
        elif circ >= self.circle_ratio:
            zone, reason = "D", f"circle: footprint={circ:.2f}"
        elif sect >= self.circle_ratio:
            zone, reason = "D", f"circle: section r_in/R={sect:.2f}"
        elif sect_sweep >= self.circle_ratio:
            zone, reason = "D", ("circle: swept section "
                                 f"r_in/R={sect_sweep:.2f}")
        elif dome >= self.dome_tau:
            zone, reason = "D", f"dome={dome:.2f}"
        elif circ >= 0.78 and dome >= 0.40:
            # near-circular footprint with a substantially curved top: dome
            # class (helmet) whose silhouette flickers at the 0.8 line
            zone, reason = "D", f"near-circular dome: circ={circ:.2f} dome={dome:.2f}"
        elif elong >= 2.2 and sect >= 0.38:
            # elongated prism with an ambiguous (non-rectangular) section:
            # per-read safe-side vote (the fused rule mirrors this)
            zone, reason = "D", f"ambiguous prism section: {sect:.2f}"
        else:
            zone, reason = "B", (f"box: circ={circ:.2f} sect={sect:.2f} "
                                 f"dome={dome:.2f}")
        return {
            "zone": zone, "reason": reason,
            "dims_mm": [round(float(v), 1) for v in dims_mm],
            "dims_macro_mm": dims_macro,
            "footprint_circularity": round(circ, 3),
            "section_ratio": round(sect, 3),
            "section_sweep": round(sect_sweep, 3),
            "dome_score": round(dome, 3),
            "flank_mm": round(flank_mm, 1),
            "plateau_frac": round(plateau, 3),
            "aspect_hw": round(aspect_hw, 3),
            "elongation": round(elong, 2),
            "n_points": int(len(obj)), "head": "overhead",
            # cloud x bounds: lets the fusion drop partial-view reads (item
            # touching the crop boundary, e.g. a stale entry frame)
            "x_lo": round(float(obj[:, 0].min()), 3),
            "x_hi": round(float(obj[:, 0].max()), 3),
        }
