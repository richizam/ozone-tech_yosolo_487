# -*- coding: utf-8 -*-
"""Look-ahead perception: overhead RGB-D -> point cloud -> measured geometry
-> official rules. No privileged state is read: inputs are the depth image
plus calibration constants (camera pose/intrinsics, belt plane, ROI).

Category logic (identical constants as tools/classify_mesh.py):
  1. dims gate (any < 10 mm undersize; sorted > 450x320x320 oversize) -> C
  2. circle-in-section: max r_in/R_circ over
       - the plan silhouette (catches plates, poufs, round footprints)
       - horizontal rings where the surface is visible at mid-heights (domes)
       - vertical cross-profiles along the item axis: a circle is fitted to the
         upper envelope; a good fit reconstructs the full section as that circle
         (lying cylinders/bottles), otherwise the envelope is closed down to the
         belt (boxes -> rectangle-ish section)
     ratio >= 0.8 -> D
  3. otherwise -> B
"""
import numpy as np
import mujoco
import shapely
from shapely.geometry import Polygon
from scipy.spatial import ConvexHull

from cell import params as P
from tools.classify_mesh import chebyshev_radius, MIN_DIM_MM, MAX_DIMS_MM, RATIO_THRESHOLD, CATEGORY_NAMES
from perception.geometry import min_area_rect

BELT_Z = 0.700                # calibrated belt plane (measured: 0.7000 +- 0.0000)
RAY_GROUPS = np.array([1, 1, 0, 1, 1, 1], dtype=np.uint8)   # group 2 = presentation layer, invisible to rays
ROI_X = (5.35, 6.55)          # camera footprint used for analysis (ends before
                              # the table entry so downstream items stay out)
ROI_Y_HALF = 0.24             # belt corridor half-width, INSIDE the physical
                              # side guides (inner faces at +-0.25): the
                              # guides are permanent hardware in the frame and
                              # merging their slivers reads as a giant item
                              # (same crop as the Isaac RTX head)
Z_MIN, Z_MAX = 0.004, 0.55    # item height window above belt (masks the mount bar)
CIRCLE_RMS_FRAC = 0.05        # accept a circle fit if rms < 5% of radius
CIRCLE_MIN_ARC = 100.0        # and the visible arc covers at least this many degrees

SENSOR_MODEL = P.VIRTUAL_SENSOR["model"]      # stamped into run logs


def _poly_ratio(pts_2d):
    """r_in / R_circ of the convex hull of 2D points (same math as ground truth)."""
    if len(pts_2d) < 6:
        return None
    try:
        hull = ConvexHull(pts_2d)
    except Exception:
        return None
    hp = np.asarray(pts_2d)[hull.vertices]
    poly = Polygon(hp)
    R = shapely.minimum_bounding_radius(poly)
    if R <= 1e-9:
        return None
    r = chebyshev_radius(np.asarray(pts_2d))
    return None if r is None else float(r / R)


class LookaheadPerception:
    """Depth sensing via ray casting (mj_multiRay) — deterministic, no OpenGL,
    identical results headless and in CI. Two virtual sensors, both standard
    in dimensioning (DWS) tunnels:
      1. overhead depth grid (~3 mm ground sampling): silhouette, dims, height
      2. light-section profile scanner: a fan of in-plane rays per cross
         section, swept along the belt axis (in reality one scanner + belt
         motion) — dense ANGULAR coverage of section contours, which a
         top-down grid cannot achieve on steep flanks and end caps."""

    # geometry/sampling constants below are the SAME values published in
    # cell.params.VIRTUAL_SENSOR (asserted in tests) — one visible config
    SCAN_CLEARANCE = 0.25       # top profiler head rides this far above the item
    FAN_ANGLES = np.arange(-0.84, 0.8401,
                           np.deg2rad(P.VIRTUAL_SENSOR["profile_angular_res_deg"]))
    FAN_STEP = P.VIRTUAL_SENSOR["profile_plane_spacing_mm"] / 1000.0
    SIDE_Y_OFF = P.VIRTUAL_SENSOR["side_head_offset_m"]
    SIDE_Z = P.VIRTUAL_SENSOR["side_head_z_m"]
    SIDE_TILTS = np.arange(-0.53, 1.0501,
                           np.deg2rad(P.VIRTUAL_SENSOR["side_head_angular_res_deg"]))

    def __init__(self, model, ground_res=None, noise_mm=None, noise_rng=None):
        self.m = model
        if ground_res is None:
            ground_res = P.VIRTUAL_SENSOR["ground_res_mm"] / 1000.0
        # per-ray Gaussian depth noise (sigma, mm). 0 = ideal-optics baseline.
        self.noise_m = (P.VIRTUAL_SENSOR["depth_noise_mm"] if noise_mm is None
                        else float(noise_mm)) / 1000.0
        self.noise_rng = noise_rng if noise_rng is not None else np.random.default_rng(0)
        cam_id = model.camera("lookahead").id
        self.cam_id = cam_id
        self.cam_pos = np.array(model.cam_pos0[cam_id] if hasattr(model, "cam_pos0")
                                else model.cam_pos[cam_id], dtype=float)
        # overhead grid: aim one ray at each belt-plane grid point in the ROI
        gx = np.arange(ROI_X[0], ROI_X[1], ground_res)
        gy = np.arange(P.BELT_A["y"] - ROI_Y_HALF, P.BELT_A["y"] + ROI_Y_HALF, ground_res)
        gxx, gyy = np.meshgrid(gx, gy)
        targets = np.column_stack([gxx.ravel(), gyy.ravel(),
                                   np.full(gxx.size, BELT_Z)])
        dirs = targets - self.cam_pos
        ranges = np.linalg.norm(dirs, axis=1)
        dirs /= ranges[:, None]
        # calibrated empty-belt depth map: per-ray background range. A return
        # counts as "item" only if it is >3 sigma NEARER than the background —
        # the standard DWS background-subtraction model, which keeps the belt
        # plane out of the cloud even under sensor noise.
        self._belt_dist = ranges
        self._dirs = dirs.astype(np.float64)
        self._vec_flat = self._dirs.reshape(-1).copy()
        self._nray = len(dirs)
        self._geomid = np.full(self._nray, -1, dtype=np.int32)
        self._dist = np.zeros(self._nray, dtype=np.float64)
        # profiler fans (directions in the y-z plane, shared by all scan planes)
        self._fan_dirs = np.column_stack([
            np.zeros_like(self.FAN_ANGLES),
            np.sin(self.FAN_ANGLES),
            -np.cos(self.FAN_ANGLES)]).astype(np.float64)
        t = self.SIDE_TILTS
        self._side_dirs = {
            +1: np.column_stack([np.zeros_like(t), np.cos(t), -np.sin(t)]).astype(np.float64),
            -1: np.column_stack([np.zeros_like(t), -np.cos(t), -np.sin(t)]).astype(np.float64),
        }
        self._fans = [(self._fan_dirs, self._fan_dirs.reshape(-1).copy())] + [
            (d, d.reshape(-1).copy()) for d in self._side_dirs.values()]
        n_max = max(len(self.FAN_ANGLES), len(t))
        self._fan_geomid = np.full(n_max, -1, dtype=np.int32)
        self._fan_dist = np.zeros(n_max, dtype=np.float64)
        self._scan_xs = np.arange(ROI_X[0], ROI_X[1], self.FAN_STEP)

    # ------------------------------------------------------------------ sensing
    def _mask(self, pts):
        z_rel = pts[:, 2] - BELT_Z
        return ((z_rel > Z_MIN) & (z_rel < Z_MAX)
                & (pts[:, 0] > ROI_X[0]) & (pts[:, 0] < ROI_X[1])
                & (np.abs(pts[:, 1] - P.BELT_A["y"]) < ROI_Y_HALF))

    def cloud(self, data, x_hint=None):
        """World-frame point cloud of the item in the ROI, or None."""
        # overhead grid; geom group 2 (presentation layer: lane markings,
        # signage, light-curtain sheet, lamp geoms) is transparent to the
        # depth sensors, exactly as it is to a real ToF/laser head
        mujoco.mj_multiRay(self.m, data, self.cam_pos, self._vec_flat,
                           RAY_GROUPS, 1, -1, self._geomid, self._dist, None,
                           self._nray, mujoco.mjMAXVAL)
        sig = max(self.noise_m, 5e-4)
        hit = self._geomid >= 0
        dist = self._dist.copy()
        if self.noise_m > 0.0 and hit.any():
            dist[hit] += self.noise_rng.normal(0.0, self.noise_m, int(hit.sum()))
        # background subtraction against the calibrated empty-belt depth map
        keep = hit & (dist < self._belt_dist - 3.0 * sig)
        grid_pts = self.cam_pos + self._dirs[keep] * dist[keep, None]
        grid_pts = grid_pts[self._mask(grid_pts)]
        if len(grid_pts) < 40:
            return None
        # profile-scanner sweep, restricted to planes that can hit the item;
        # the head rides SCAN_CLEARANCE above the measured item top so small
        # features subtend enough scan angles (real profilers mount close)
        x_lo, x_hi = grid_pts[:, 0].min() - 0.02, grid_pts[:, 0].max() + 0.02
        z_scan = float(grid_pts[:, 2].max()) + self.SCAN_CLEARANCE
        y0 = P.BELT_A["y"]
        fan_pts = []
        for x in self._scan_xs:
            if not (x_lo <= x <= x_hi):
                continue
            heads = [(np.array([x, y0, z_scan]), *self._fans[0]),
                     (np.array([x, y0 - self.SIDE_Y_OFF, self.SIDE_Z]), *self._fans[1]),
                     (np.array([x, y0 + self.SIDE_Y_OFF, self.SIDE_Z]), *self._fans[2])]
            for origin, dirs, vec in heads:
                n = len(dirs)
                mujoco.mj_multiRay(self.m, data, origin, vec,
                                   RAY_GROUPS, 1, -1, self._fan_geomid[:n], self._fan_dist[:n],
                                   None, n, mujoco.mjMAXVAL)
                fh = self._fan_geomid[:n] >= 0
                if fh.any():
                    fd = self._fan_dist[:n].copy()
                    if self.noise_m > 0.0:
                        fd[fh] += self.noise_rng.normal(0.0, self.noise_m, int(fh.sum()))
                    # per-ray background range: distance at which this ray
                    # would hit the empty belt plane (inf for upward rays)
                    dz = dirs[:, 2]
                    bg = np.where(dz < -1e-6,
                                  (origin[2] - BELT_Z) / np.where(dz < -1e-6, -dz, 1.0),
                                  np.inf)
                    fkeep = fh & (fd < bg - 3.0 * sig)
                    if fkeep.any():
                        fan_pts.append(origin + dirs[fkeep] * fd[fkeep, None])
        if fan_pts:
            fan_pts = np.vstack(fan_pts)
            fan_pts = fan_pts[self._mask(fan_pts)]
            pts = np.vstack([grid_pts, fan_pts])
        else:
            pts = grid_pts
        pts = self._denoise(pts)
        if pts is None or len(pts) < 40:
            return None
        return self._select_cluster(pts, x_hint)

    def _denoise(self, pts):
        """Morphological cleanup under sensor noise (standard DWS practice):
        background-subtraction tails are ISOLATED salt returns, real items are
        DENSE blobs — keep points whose 3x3-cell plan neighborhood holds >=5
        returns (cell = 6 mm). No-op for the ideal-optics baseline."""
        if self.noise_m <= 0.0 or len(pts) == 0:
            return pts
        cell = 0.006
        ij = np.floor(pts[:, :2] / cell).astype(np.int64)
        ij -= ij.min(axis=0)
        shape = ij.max(axis=0) + 1
        flat = ij[:, 0] * shape[1] + ij[:, 1]
        counts = np.bincount(flat, minlength=int(shape[0] * shape[1])
                             ).reshape(shape)
        padded = np.pad(counts, 1)
        neigh = sum(padded[di:di + shape[0], dj:dj + shape[1]]
                    for di in range(3) for dj in range(3))
        return pts[neigh[ij[:, 0], ij[:, 1]] >= 5]

    @staticmethod
    def _select_cluster(pts, x_hint=None, gap=0.07, min_pts=40, max_hint_dist=0.10):
        """Instance isolation: split the cloud at along-belt gaps and keep the
        cluster of the tracked item (x_hint from the belt tracker); without a
        hint, keep the largest cluster.

        Identity gate: if no cluster COVERS the tracked position (within
        max_hint_dist), there is no valid measurement of THIS item — return
        None (sensor miss) rather than a neighbor's cloud. Committing the
        nearest cluster spoofed a tailgater's identity onto a thin item whose
        own returns vanished (found by the stress scenario)."""
        order = np.argsort(pts[:, 0])
        xs = pts[order, 0]
        splits = np.where(np.diff(xs) > gap)[0]
        if len(splits) == 0:
            clusters = [pts]
        else:
            bounds = [0] + (splits + 1).tolist() + [len(xs)]
            clusters = [pts[order[bounds[i]:bounds[i + 1]]]
                        for i in range(len(bounds) - 1)]
            clusters = [c for c in clusters if len(c) >= min_pts] or clusters
        if x_hint is not None:
            def dist(c):
                lo, hi = c[:, 0].min(), c[:, 0].max()
                return 0.0 if lo <= x_hint <= hi else min(abs(x_hint - lo), abs(x_hint - hi))
            best = min(clusters, key=dist)
            return best if dist(best) <= max_hint_dist else None
        return max(clusters, key=len)

    # ------------------------------------------------------------------ analysis
    def analyze(self, pts):
        z_rel = pts[:, 2] - BELT_Z
        L, W, ang = min_area_rect(pts[:, :2])
        if self.noise_m > 0.0:
            # calibrated robust extents for the noisy path: percentile spans
            # along the rect axes instead of hull extremes (kills the
            # extreme-value bias of max-statistics over ~10^4 noisy returns),
            # minus the known boundary-inflation bias of ~2 sigma
            ud = np.array([np.cos(ang), np.sin(ang)])
            vd = np.array([-np.sin(ang), np.cos(ang)])
            pu, pv = pts[:, :2] @ ud, pts[:, :2] @ vd
            cal = 2.0 * self.noise_m
            L = max(float(np.percentile(pu, 99.5) - np.percentile(pu, 0.5)) - cal, 0.001)
            W = max(float(np.percentile(pv, 99.5) - np.percentile(pv, 0.5)) - cal, 0.001)
            height = max(float(np.percentile(z_rel, 99.5)) - cal, 0.001)
        else:
            height = float(z_rel.max())
        dims_mm = np.sort([L * 1000, W * 1000, height * 1000])[::-1]

        sections = []
        # (a) plan silhouette (catches plates, poufs, round footprints)
        sil = _poly_ratio(pts[:, :2])
        if sil is not None:
            sections.append(("silhouette", sil))
        # (b) radial-profile ratio per cross-section along the item axis:
        # about the section's resting axis (mid-height, silhouette center),
        # the outer radius as a function of angle gives min/max rho — the
        # inscribed/circumscribed ratio for convex sections. One formulation
        # covers cylinders, prisms (hexagon 0.866, octagon 0.749), domes AND
        # small rounded end caps (the «Цилиндр» trap) without circle fitting.
        u_dir = np.array([np.cos(ang), np.sin(ang)])
        v_dir = np.array([-np.sin(ang), np.cos(ang)])
        center = pts[:, :2].mean(axis=0)
        u = (pts[:, :2] - center) @ u_dir
        v = (pts[:, :2] - center) @ v_dir
        # station plan (per the agreed slicing strategy): a body sweep along
        # the main axis PLUS densified transverse slices near BOTH ends —
        # rounded end caps are small, and their circular sections must be
        # confirmed by SEVERAL NEARBY slices, not one lucky band
        u_min, u_max = float(u.min()), float(u.max())
        stations = [(s, "body") for s in np.linspace(u_min + 0.05, u_max - 0.05, 7)]
        end_offsets = np.arange(0.006, 0.046, 0.004)
        stations += [(u_min + o, "end") for o in end_offsets]
        stations += [(u_max - o, "end") for o in end_offsets]
        circ_hits = []                 # (u_station, kind) with ratio >= threshold
        end_support = []               # end stations just below threshold (>= T-0.03)
        circ_minor = []                # 2*min(rho) of circular-ish sections: the
                                       # pose-independent minor thickness of round
                                       # bodies (a pen resting on its clip still
                                       # measures its 9 mm barrel)
        # 30..150 deg: the extreme bins are grazing-sampled by every head and
        # systematically under-measure the boundary (-0.1 ratio bias on small
        # caps); the 120-deg window still contains square corners at 45 deg,
        # so box sections keep their true ~0.71 while circles read ~1
        theta_edges = np.linspace(np.deg2rad(30), np.deg2rad(150), 9)
        for s, s_kind in stations:
            band = np.abs(u - s) < 0.005
            if band.sum() < 12:
                continue
            ub, vb, zb = u[band], v[band], z_rel[band]
            h_band = float(zb.max())
            if h_band < 0.008:
                continue
            vc = 0.5 * (vb.min() + vb.max())
            # the section axis of a resting item sits at half the ITEM height
            # (a tapered end cap keeps the body's axis, not the band's own)
            zc = 0.5 * height
            theta = np.arctan2(zb - zc, vb - vc)
            rho = np.hypot(vb - vc, zb - zc)
            # (b1) prism path: outer radius per angular bin, min/max ratio
            rho_out = []
            for lo_e, hi_e in zip(theta_edges[:-1], theta_edges[1:]):
                in_bin = (theta >= lo_e) & (theta < hi_e)
                if in_bin.any():
                    rho_out.append(float(rho[in_bin].max()))
            if len(rho_out) >= 6:
                ratio = min(rho_out) / max(rho_out)
                sections.append((f"radial@u{s * 1000:+.0f}mm", ratio))
                if ratio >= 0.75 and s_kind == "body":
                    circ_minor.append(2.0 * min(rho_out))
                if ratio >= RATIO_THRESHOLD:
                    circ_hits.append((s, s_kind))
                elif s_kind == "end" and ratio >= RATIO_THRESHOLD - 0.03:
                    end_support.append(s)
            # (b2) surface-of-revolution path: tapering features (end caps!)
            # have rho constant across angles at each axial position even
            # though rho varies along the axis — test per 2 mm axial sub-bin
            votes, refutes = 0, 0
            for u_sub in np.arange(ub.min(), ub.max(), 0.002):
                sub = (ub >= u_sub) & (ub < u_sub + 0.002)
                if sub.sum() < 6:
                    continue
                th = theta[sub]
                if np.ptp(th) < np.deg2rad(80):
                    continue
                r_sub = rho[sub]
                med = float(np.median(r_sub))
                if med < 0.004:
                    continue
                if sub.sum() >= 8 and np.ptp(ub[sub]) > 5e-4:
                    # remove the axial taper (expected for cones/domes) so the
                    # spread measures angular asymmetry only
                    trend = np.polyval(np.polyfit(ub[sub], r_sub, 1), ub[sub])
                    resid = r_sub - trend
                else:
                    resid = r_sub - med
                spread = float(np.percentile(resid, 95) - np.percentile(resid, 5)) / med
                if spread < 0.12:
                    votes += 1
                elif spread > 0.5:
                    refutes += 1        # interior fill (a flat face crossing the band)
            if votes >= 2 and votes > refutes:
                sections.append((f"revolution@u{s * 1000:+.0f}mm", 0.995))
                circ_hits.append((s, s_kind))

        max_ratio = max((r for _, r in sections), default=0.0)
        minor_mm = float(np.median(circ_minor) * 1000) if circ_minor else None
        undersize = bool(np.any(dims_mm < MIN_DIM_MM)
                         or (minor_mm is not None and minor_mm < MIN_DIM_MM))
        oversize = bool(np.any(dims_mm > MAX_DIMS_MM))

        # circularity verdict: the plan silhouette or any BODY section over
        # threshold decides directly; END sections must be corroborated by a
        # second nearby slice (>= 2 distinct stations within 12 mm) so a
        # single noisy band at an item end cannot flip the category
        sil_circ = sil is not None and sil >= RATIO_THRESHOLD
        body_circ = any(k == "body" for _, k in circ_hits)
        end_us = sorted(set(round(s_u, 4) for s_u, k in circ_hits if k == "end"))
        # strict cluster: two FULL hits nearby — decides on its own;
        # relaxed cluster: a hit corroborated only by a near-threshold
        # neighbour — needs cross-read persistence (see the fusion layer)
        end_strict = any(b - a <= 0.012 for a, b in zip(end_us, end_us[1:]))
        support = sorted(set(round(s_u, 4) for s_u in end_support))
        end_relaxed = (not end_strict) and any(
            any(abs(sp - h) <= 0.012 for sp in support) for h in end_us)
        isolated_end = bool(end_us) and not end_strict and not end_relaxed
        circular = sil_circ or body_circ or end_strict

        if undersize or oversize:
            zone = "C"
        elif circular or end_relaxed:
            zone = "D"                # single-read verdict; the multi-read
            # fusion layer requires persistence for relaxed-only evidence
        else:
            zone = "B"

        margin_ratio = abs(max_ratio - RATIO_THRESHOLD)
        margin_dims = float(min(np.min(np.abs(dims_mm - MIN_DIM_MM)),
                                np.min(np.abs(np.sort(dims_mm)[::-1] - MAX_DIMS_MM))))
        flags = []
        if margin_ratio < 0.05:
            flags.append("near_ratio_threshold")
        if margin_dims < 5.0:
            flags.append("near_dim_limit")
        if isolated_end:
            flags.append("isolated_end_circle")   # seen once, not corroborated
        confidence = round(float(min(1.0, 6.0 * margin_ratio + 0.02 * margin_dims)), 3)

        return {
            "zone": zone,
            "category": CATEGORY_NAMES[zone],
            "circular": bool(circular),        # strict evidence: decides alone
            "circular_relaxed": bool(end_relaxed),
            "dims_mm": [round(float(d), 1) for d in dims_mm],
            "minor_mm": None if minor_mm is None else round(minor_mm, 1),
            "max_ratio": round(float(max_ratio), 3),
            "sections": [(k, round(float(r), 3)) for k, r in sections],
            "undersize": undersize, "oversize": oversize,
            "confidence": confidence, "flags": flags,
            "n_points": int(len(pts)),
        }

    def classify(self, data, x_hint=None):
        pts = self.cloud(data, x_hint=x_hint)
        if pts is None or len(pts) < 40:
            return None
        return self.analyze(pts)
