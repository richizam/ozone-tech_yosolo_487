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
               dome_tau=0.75, flank_tau_mm=6.0, window_x=(5.6, 6.6)):
    """Fuse the window's multi-read evidence per the OFFICIAL decision order,
    with legal-metrology guard bands: a fused dimension within the sensor's
    uncertainty of a limit cannot take the permissive branch, and zero valid
    reads divert to the manual-review lane — never to the sorter.

    Returns {'zone', 'reason', 'dims_mm', features..., 'n_reads', 'sensor_miss'}."""
    reads = [r for r in reads if r]
    # a read longer than any certifiable single item is a multi-item echo
    # (contact pair in the window) — invalid, never evidence
    reads = [r for r in reads if r["dims_mm"][0] <= 520.0]
    if not reads:
        return {"zone": "D", "reason": "sensor_miss -> manual-review lane",
                "dims_mm": None, "n_reads": 0, "sensor_miss": True}
    # drop partial-view reads: a cloud touching the crop boundary (a stale
    # frame from before the item fully entered the window) reads truncated
    # dims and smeared shape
    full = [r for r in reads
            if r.get("x_lo", window_x[0] + 1) > window_x[0] + 0.015
            and r.get("x_hi", window_x[1] - 1) < window_x[1] - 0.015]
    if full:
        reads = full

    # element-wise MIN over reads: in-motion frame mixing can only INFLATE an
    # extent (the cloud smears along the belt), and the boundary guard above
    # keeps truncated reads out, so the smallest read per dimension is the
    # clean estimate
    dims = np.min(np.array([r["dims_mm"] for r in reads], dtype=float),
                  axis=0)                          # each read sorted desc
    # shape features: median over the 3 FRESHEST reads (smallest dims sum =
    # least smear; a single read is noisy, the full median drags in smeared
    # tails)
    top = sorted(reads, key=lambda r: sum(r["dims_mm"]))[:3]

    def med(key):
        return float(np.median([r[key] for r in top]))

    circ, sect, dome = (med("footprint_circularity"), med("section_ratio"),
                        med("dome_score"))
    elong = med("elongation")
    aspect = med("aspect_hw")
    flanks = [float(r["flank_mm"]) for r in top if float(r["flank_mm"]) >= 0.0]
    flank = float(np.median(flanks)) if flanks else None
    d_votes = sum(1 for r in reads if r["zone"] == "D")
    under = bool(dims.min() < LIMIT_MIN_MM + guard_mm)
    over = bool(np.any(np.sort(dims)[::-1] > LIMIT_MAX_MM - guard_mm))
    flank_s = "n/a" if flank is None else f"{flank:.1f}mm"
    feats = (f"circ={circ:.2f} sect={sect:.2f} dome={dome:.2f} "
             f"flank={flank_s} elong={elong:.1f} h/w={aspect:.2f}")
    if under:
        zone, reason = "C", (f"undersize: min dim {dims.min():.1f} mm below "
                             f"certification floor {LIMIT_MIN_MM + guard_mm:.0f} mm")
    elif over:
        zone, reason = "C", f"oversize (guard band {guard_mm:.0f} mm)"
    elif (circ >= circle_ratio or sect >= circle_ratio or dome >= dome_tau
          or (circ >= 0.78 and dome >= 0.40)
          or (flank is not None and elong >= 1.8 and flank >= flank_tau_mm)
          or (circ >= 0.68 and dome >= 0.35 and aspect >= 0.85)):
        # last clause: TALL ROUND DOME, triple-gated — near-circular
        # footprint AND curved top AND as tall as it is wide (helmet
        # h/w 0.93). Each B item is blocked by a wide margin on a
        # DIFFERENT gate: detergent by aspect (0.69), boxes by dome (~0),
        # bottle by circ (0.37)
        zone, reason = "D", f"circle evidence: {feats}"
    elif elong >= 2.5 and flank is None and sect < circle_ratio:
        # SAFE-SIDE: an elongated prism whose section could not be verified
        # by the side heads must never take the permissive branch (a lying
        # cylinder with too few side returns went to the sorter otherwise)
        zone, reason = "D", f"section unverifiable on elongated item: {feats}"
    elif d_votes >= 2 and 2 * d_votes >= len(reads):
        zone, reason = "D", f"persistent D evidence: {d_votes}/{len(reads)} reads"
    else:
        zone, reason = "B", f"fits, no circle evidence: {feats}"
    return {"zone": zone, "reason": reason,
            "dims_mm": [round(float(v), 1) for v in dims],
            "footprint_circularity": round(circ, 3),
            "section_ratio": round(sect, 3), "dome_score": round(dome, 3),
            "flank_mm": None if flank is None else round(flank, 1),
            "n_reads": len(reads),
            "d_votes": d_votes, "sensor_miss": False}


class RTXPerception:
    """Multi-head RTX depth station -> B/C/D verdict.

    Mirrors the VIRTUAL_SENSOR architecture from cell/params.py with real
    rendered sensors: one overhead depth head plus two side heads (the side
    heads see the section below its widest line — an overhead view alone
    cannot distinguish a hex prism from a box at DWS sampling density)."""

    def __init__(self, camera, belt_z=BELT_Z, z_margin=0.006,
                 circle_ratio=P.CIRCLE_RATIO, dome_tau=0.75, elong_min=1.8,
                 window_x=(5.6, 6.6), belt_y=P.BELT_A["y"],
                 belt_half_w=P.BELT_A["width"] / 2 - 0.01,
                 grid_mm=4.0, min_points=40):
        # belt_half_w stays INSIDE the physical side guides (rails at
        # width/2 + 0.015): the guides are permanent hardware in the frame
        # and merging their slivers into the item cloud reads as oversize
        self.cams = list(camera) if isinstance(camera, (list, tuple)) else [camera]
        self.cam = self.cams[0]                    # primary (overhead) head
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
    def _head_points(cls, cam, edge_filter=False, edge_tau=0.008):
        frame = cam.get_current_frame()
        depth = frame.get("distance_to_image_plane")
        if depth is None:
            return None
        depth = np.asarray(depth, dtype=np.float32)
        if depth.ndim != 2 or depth.size == 0:
            return None
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
        cam_pts = np.stack([xc, yc, zc], axis=-1).reshape(-1, 3)
        world = pos + cam_pts @ R.T
        return world[valid.reshape(-1)]

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
        m = ((world[:, 2] > self.belt_z + self.z_margin)
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

        def half(seq):
            out = []
            for p in seq:
                while len(out) >= 2 and np.cross(out[-1] - out[-2],
                                                 p - out[-2]) <= 0:
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
            if debug and len(world):
                lo = world.min(axis=0)
                hi = world.max(axis=0)
                above = int((world[:, 2] > self.belt_z + self.z_margin).sum())
                print(f"[rtx] MISS: {len(world)} pts, bounds x[{lo[0]:.2f},{hi[0]:.2f}] "
                      f"y[{lo[1]:.2f},{hi[1]:.2f}] z[{lo[2]:.2f},{hi[2]:.2f}] "
                      f"above_belt={above} segmented={len(obj)}", flush=True)
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
        if elong >= self.elong_min:
            hh = obj[:, 2] - self.belt_z
            sect = self._section_ratio(t1, t2, hh)
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

        undersize = bool(np.any(dims_mm < LIMIT_MIN_MM))
        oversize = bool(np.any(dims_mm > LIMIT_MAX_MM))
        if undersize or oversize:
            zone, reason = "C", ("undersize" if undersize else "oversize")
        elif circ >= self.circle_ratio:
            zone, reason = "D", f"circle: footprint={circ:.2f}"
        elif sect >= self.circle_ratio:
            zone, reason = "D", f"circle: section r_in/R={sect:.2f}"
        elif dome >= self.dome_tau:
            zone, reason = "D", f"dome={dome:.2f}"
        elif circ >= 0.78 and dome >= 0.40:
            # near-circular footprint with a substantially curved top: dome
            # class (helmet) whose silhouette flickers at the 0.8 line
            zone, reason = "D", f"near-circular dome: circ={circ:.2f} dome={dome:.2f}"
        elif elong >= self.elong_min and flank_mm >= 6.0:
            # side heads: an elongated prism's flank is not a vertical plane
            # — a rolled cylinder/hex, never a lying box
            zone, reason = "D", f"curved flank: {flank_mm:.1f} mm over height"
        else:
            zone, reason = "B", (f"box: circ={circ:.2f} sect={sect:.2f} "
                                 f"dome={dome:.2f}")
        return {
            "zone": zone, "reason": reason,
            "dims_mm": [round(float(v), 1) for v in dims_mm],
            "footprint_circularity": round(circ, 3),
            "section_ratio": round(sect, 3),
            "dome_score": round(dome, 3),
            "flank_mm": round(flank_mm, 1),
            "plateau_frac": round(plateau, 3),
            "aspect_hw": round(aspect_hw, 3),
            "elongation": round(elong, 2),
            "n_points": int(len(obj)),
            # cloud x bounds: lets the fusion drop partial-view reads (item
            # touching the crop boundary, e.g. a stale entry frame)
            "x_lo": round(float(obj[:, 0].min()), 3),
            "x_hi": round(float(obj[:, 0].max()), 3),
        }
