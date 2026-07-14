# -*- coding: utf-8 -*-
"""Industrial presentation layer for the Isaac Sim cell — visuals only.

Everything here is NON-COLLIDING and placed outside the perception crop and
behind every sensor's near plane: physics, classification and the jam-camera
background are bit-identical with this layer on or off (the MuJoCo twin's
contype-0 discipline).

What it adds (user spec + presentation overhaul):
  * the DRIVE is visible: transport rollers/end drums under belt A/B and the
    incline connector, dark rubber belt surfaces, metallic rollers;
  * industrial materials (isaac.materials.PRESETS): brushed steel chute
    shells, powder-coated dark frames, galvanized cage tube, worn
    safety-yellow guards, matte belt rubber;
  * real roll cages: galvanized tube frame + wire-mesh panels + casters +
    framed chute aperture; route colour ONLY on the label plate and a thin
    top-rail accent stripe;
  * B-transfer continuity (nose apron, drive motor/gearbox, legs) and
    brushed under-shells + signage on every chute;
  * visible sensor hardware: rigid gantry mounts, connector boxes and
    conduit runs for the overhead head, the two side profilers, the macro
    head and the jam camera;
  * housed HMI/andon: recessed route lamps, stack light, e-stops, pinch
    labels, floor conduit;
  * warehouse lighting: dimmer dome, high-bay area fixtures, two soft aisle
    lights — no local spotlight on the arm, no hot cage speculars.
"""
import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade, Vt

from cell import params as P

from isaac.materials import preset as mat_preset

RUBBER = (0.085, 0.088, 0.095)
STEEL = (0.62, 0.64, 0.68)
FRAME = (0.30, 0.33, 0.38)
BLACK = (0.045, 0.045, 0.05)
OZON_BLUE = (0.0, 0.357, 1.0)                      # #005BFF
# named industrial finishes (materials.PRESETS): (color, rough, metallic)
BRUSHED, BRUSHED_R, BRUSHED_M = mat_preset("brushed_steel")
POWDER_DK, POWDER_DK_R, POWDER_DK_M = mat_preset("powder_steel_dark")
GALV, GALV_R, GALV_M = mat_preset("galvanized")
BELT_RUB, BELT_RUB_R, BELT_RUB_M = mat_preset("belt_rubber")
YELLOW, YELLOW_R, YELLOW_M = mat_preset("safety_yellow_worn")
SIGN_DIR = Path("/tmp/sortmaster_signs")

ROOT = "/World/dressing"


class Dressing:
    def __init__(self, stage):
        self.stage = stage
        self._i = 0
        self._vismats = {}
        UsdGeom.Xform.Define(stage, ROOT)

    # ------------------------------------------------------------ primitives
    def _path(self, tag):
        self._i += 1
        return f"{ROOT}/{tag}_{self._i}"

    def _ensure_pbr(self):
        if getattr(self, "_pbr", "unset") != "unset":
            return self._pbr
        self._pbr = None
        try:
            from isaac.materials import make_grunge_textures, PbrLibrary
            ta, tr = make_grunge_textures(str(SIGN_DIR))
            self._pbr = PbrLibrary(self.stage, ROOT, tex_albedo=ta,
                                   tex_rough=tr)
        except Exception as exc:
            print(f"[dressing] OmniPBR unavailable ({exc})", flush=True)
        return self._pbr

    def _vis(self, prim, color, roughness=0.80, metallic=0.05, textured=True):
        """Matte industrial PBR with world-triplanar grunge (dirt/scratch/
        edge wear) — bare displayColor renders as toy plastic under RTX.
        Runtime-tinted prims (route lamps/LEDs) must NOT bind —
        a bound material overrides displayColor updates. Big flat backdrops
        pass textured=False (a tiled detail map reads as wallpaper on them)."""
        seed = hash(prim.GetPath().pathString) & 0x7fffffff
        pbr = self._ensure_pbr()
        if pbr is not None:
            try:
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                    pbr.get(color, roughness=roughness, metallic=metallic,
                            textured=textured, seed=seed))
                return
            except Exception:
                pass
        jit = (0.90, 1.0, 1.08)[seed % 3]
        c = tuple(min(1.0, float(v) * jit) for v in color)
        key = (round(c[0], 3), round(c[1], 3), round(c[2], 3), roughness)
        mat = self._vismats.get(key)
        if mat is None:
            path = f"{ROOT}/VisMats/m_{len(self._vismats)}"
            mat = UsdShade.Material.Define(self.stage, path)
            sh = UsdShade.Shader.Define(self.stage, f"{path}/pbr")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(*c))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(
                float(roughness))
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(
                float(metallic))
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),
                                                      "surface")
            self._vismats[key] = mat
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)

    def box(self, center, half, color, tag="box", euler_deg=(0, 0, 0),
            opacity=None, bind=True, textured=True, roughness=0.80,
            metallic=0.05):
        cube = UsdGeom.Cube.Define(self.stage, self._path(tag))
        cube.CreateSizeAttr(2.0)
        xf = UsdGeom.Xformable(cube.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(*[float(c) for c in center]))
        if any(abs(e) > 1e-9 for e in euler_deg):
            xf.AddRotateXYZOp().Set(Gf.Vec3f(*[float(e) for e in euler_deg]))
        xf.AddScaleOp().Set(Gf.Vec3f(*[float(h) for h in half]))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if opacity is not None:
            cube.CreateDisplayOpacityAttr([float(opacity)])
        elif bind:
            self._vis(cube.GetPrim(), color, roughness=roughness,
                      metallic=metallic, textured=textured)
        return cube.GetPrim()

    def cyl(self, center, radius, half_h, color, axis="Z", tag="cyl",
            bind=True, roughness=0.80, metallic=0.05):
        c = UsdGeom.Cylinder.Define(self.stage, self._path(tag))
        c.CreateRadiusAttr(float(radius))
        c.CreateHeightAttr(float(2 * half_h))
        c.CreateAxisAttr(axis)
        UsdGeom.Xformable(c.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(*[float(v) for v in center]))
        c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if bind:
            self._vis(c.GetPrim(), color, roughness=roughness,
                      metallic=metallic)
        return c.GetPrim()

    # ------------------------------------------------------------ label signs
    def _label_texture(self, text, fname, bg=OZON_BLUE, fg=(255, 255, 255)):
        """Render a label PNG with PIL (ships in the Isaac container)."""
        from PIL import Image, ImageDraw, ImageFont
        SIGN_DIR.mkdir(parents=True, exist_ok=True)
        W, H = 768, 192
        img = Image.new("RGB", (W, H),
                        tuple(int(255 * v) for v in bg))
        d = ImageDraw.Draw(img)
        font = None
        for cand in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                     "/isaac-sim/kit/resources/fonts/OpenSans-SemiBold.ttf"):
            try:
                font = ImageFont.truetype(cand, 96)
                break
            except Exception:
                continue
        if font is None:
            font = ImageFont.load_default()
        tb = d.textbbox((0, 0), text, font=font)
        d.text(((W - tb[2] + tb[0]) / 2 - tb[0], (H - tb[3] + tb[1]) / 2 - tb[1]),
               text, fill=fg, font=font)
        # accent bar, Ozon-style
        d.rectangle([0, H - 14, W, H], fill=(255, 255, 255))
        p = SIGN_DIR / fname
        img.save(p)
        return str(p)

    def _textured_quad(self, tag, tex, center, width, yaw_deg=0.0,
                       tilt_deg=90.0):
        return self._quad_with_texture(tex, center, width, yaw_deg, tilt_deg)

    def label(self, text, fname, center, width, yaw_deg=0.0, bg=OZON_BLUE,
              tilt_deg=90.0, fg=(255, 255, 255)):
        """Textured label quad (UsdPreviewSurface + UsdUVTexture)."""
        tex = self._label_texture(text, fname, bg=bg, fg=fg)
        return self._quad_with_texture(tex, center, width, yaw_deg, tilt_deg)

    def _quad_with_texture(self, tex, center, width, yaw_deg=0.0,
                           tilt_deg=90.0, _pair=True):
        # SIGN ORIENTATION RULE: a doubleSided single quad mirrors its text
        # when viewed from behind (the showcase hero read "RETROS B").
        # Every upright sign is therefore a back-to-back PAIR of
        # single-sided quads, each reading correctly from its own side.
        if _pair and abs(tilt_deg) > 30.0:
            nx = math.sin(math.radians(yaw_deg))
            ny = -math.cos(math.radians(yaw_deg))
            off = 0.004
            front = (center[0] + nx * off, center[1] + ny * off, center[2])
            back = (center[0] - nx * off, center[1] - ny * off, center[2])
            p = self._quad_with_texture(tex, front, width, yaw_deg,
                                        tilt_deg, _pair=False)
            self._quad_with_texture(tex, back, width, yaw_deg + 180.0,
                                    tilt_deg, _pair=False)
            return p
        path = self._path("label")
        mesh = UsdGeom.Mesh.Define(self.stage, path)
        w2, h2 = width / 2, width / 8               # 4:1 board
        pts = [(-w2, -h2, 0), (w2, -h2, 0), (w2, h2, 0), (-w2, h2, 0)]
        mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
        mesh.CreateDoubleSidedAttr(False)
        st = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray,
            UsdGeom.Tokens.faceVarying)
        st.Set(Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0),
                              Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]))
        xf = UsdGeom.Xformable(mesh.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in center]))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(float(tilt_deg), 0.0, float(yaw_deg)))
        # material
        mpath = f"{path}_mat"
        mat = UsdShade.Material.Define(self.stage, mpath)
        sh = UsdShade.Shader.Define(self.stage, f"{mpath}/pbr")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
        tx = UsdShade.Shader.Define(self.stage, f"{mpath}/tex")
        tx.CreateIdAttr("UsdUVTexture")
        tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(tex)
        rd = UsdShade.Shader.Define(self.stage, f"{mpath}/st")
        rd.CreateIdAttr("UsdPrimvarReader_float2")
        rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            rd.ConnectableAPI(), "result")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            tx.ConnectableAPI(), "rgb")
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
        return mesh.GetPrim()

    # ---------------------------------------------------------------- camera
    def camera_box(self, pos, yaw_deg=0.0, tag="cam", lens_down=True,
                   connector=True):
        """Black sensor housing with a lens ring, mounted ABOVE/BEHIND the
        actual camera origin so the rendered view is never occluded (near
        clip 0.05 m). Carries its own connector box on the top face (the
        conduit run down the mount is added by the caller — mount routes
        differ per head). connector=False for heads whose top face lies
        inside the perception corridor (macro): their connector moves onto
        the mount above the corridor ceiling."""
        x, y, z = pos
        self.box((x, y, z + 0.075), (0.075, 0.055, 0.045), BLACK, tag=tag,
                 euler_deg=(0, 0, yaw_deg))
        if lens_down:
            self.cyl((x, y, z + 0.022), 0.028, 0.008, (0.02, 0.02, 0.025),
                     tag=f"{tag}_lens")
        self.box((x, y, z + 0.135), (0.012, 0.012, 0.015), FRAME,
                 tag=f"{tag}_mnt")
        if connector:   # connector box on the housing top rear (M12 look)
            self.box((x - 0.045, y, z + 0.128), (0.016, 0.014, 0.010),
                     (0.10, 0.11, 0.13), tag=f"{tag}_conn")

    def _conduit(self, pts, tag="conduit", half=0.010):
        """Rigid cable conduit as 2-3 elongated box segments between bend
        points (replaces thin hanging-line prims, which read as debris)."""
        col = (0.34, 0.36, 0.40)
        for (x0, y0, z0), (x1, y1, z1) in zip(pts[:-1], pts[1:]):
            dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
            c = ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)
            if abs(dz) >= max(abs(dx), abs(dy)):        # vertical run
                self.box(c, (half, half, abs(dz) / 2 + half), col, tag=tag)
            elif abs(dx) >= abs(dy):                    # run along x
                self.box(c, (abs(dx) / 2 + half, half, half), col, tag=tag)
            else:                                       # run along y
                self.box(c, (half, abs(dy) / 2 + half, half), col, tag=tag)

    # ============================================================== sections
    def rollers(self):
        """Make the DRIVE readable. Belt conveyors read as: dark rubber band
        (the conveyor top itself) + proud end drums + side skirts + yellow
        guards. The tilt-tray train carries its own engineered embodiment
        (chassis, skirts, end modules) from scene_usd; the incline connector
        gets skirts + a tail drum here."""
        a, b, bc = P.BELT_A, P.BELT_B, P.B_CONNECT
        r = 0.035
        ang = math.atan2(bc["z_top1"] - bc["z_top0"], bc["y1"] - bc["y0"])
        cy_c = (bc["y0"] + bc["y1"]) / 2
        cz_c = (bc["z_top0"] + bc["z_top1"]) / 2
        clen = float(np.hypot(bc["y1"] - bc["y0"], bc["z_top1"] - bc["z_top0"]))
        if not getattr(self, "shells", False):
            # belt A: side skirts + end drums peeking beyond the band ends
            # (skirts end at x 6.10 — the extended west end-module wall at
            # x 6.118 would otherwise interpenetrate them; the module and
            # knife cheeks carry the enclosure from there)
            for sgn in (-1, 1):
                self.box((3.04,
                          a["y"] + sgn * (a["width"] / 2 + 0.035),
                          a["top"] - 0.09),
                         (3.06, 0.02, 0.115),
                         FRAME, tag="cheekA")
            self.cyl((-0.045, a["y"], a["top"] - r), r + 0.008,
                     a["width"] / 2 + 0.01, (0.5, 0.52, 0.55), axis="Y",
                     tag="drumA")
            # belt B: side skirts + drums (band runs along Y)
            for sgn in (-1, 1):
                self.box((b["cx"] + sgn * (b["width"] / 2 + 0.035),
                          (b["y0"] + b["y1"]) / 2, b["top"] - 0.09),
                         (0.02, (b["y1"] - b["y0"]) / 2 + 0.02, 0.115), FRAME,
                         tag="cheekB")
            for y in (b["y0"] - 0.045, b["y1"] + 0.045):
                if 0.0 < y < 6.0:
                    self.cyl((b["cx"], y, b["top"] - r), r + 0.008,
                             b["width"] / 2 + 0.01, (0.5, 0.52, 0.55),
                             axis="X", tag="drumB")
        # knife-edge nose: slim side cheeks + a small nose roller under the
        # lip (the thin-section transfer hardware small-item lines use)
        for sgn in (-1, 1):
            self.box(((a["knife_x0"] + a["nose_x"]) / 2,
                      a["y"] + sgn * (a["width"] / 2 + 0.028),
                      a["top"] - a["knife_t"] / 2),
                     ((a["nose_x"] - a["knife_x0"]) / 2, 0.012,
                      a["knife_t"] / 2 + 0.012), FRAME, tag="knife_cheek")
        self.cyl((a["nose_x"] - 0.008, a["y"], a["top"] - a["knife_t"] - 0.009),
                 0.011, a["width"] / 2 - 0.01, (0.5, 0.52, 0.55), axis="Y",
                 tag="knife_drum")
        # incline connector: sloped side skirt + head/tail drums.
        # SWEEP-AUDIT FIXES: (1) the skirts started at the tray line — the
        # B-station tray at joint-limit tilt clipped them by 11-26 mm and
        # the RETURN-leg trays grazed their south tips; skirts now start at
        # y 3.45 (the same sweep-corridor rule the rails follow). (2) the
        # EAST skirt stood 13 mm inside the parallel REVIEW slide corridor
        # (freight on REVIEW reaches x 8.71) — deleted entirely; the east
        # flank is carried by the belt edge + head drum. (3) the tail drum
        # sat across the tray's fault envelope (-25.8 mm at the joint
        # limit) — moved down-slope under the mouth apron, out of the sweep.
        sk_y0 = 3.45
        sk_cy = (sk_y0 + bc["y1"]) / 2
        sk_len = float(np.hypot(bc["y1"] - sk_y0,
                                (bc["y1"] - sk_y0) * math.tan(ang)))
        sk_frac = (sk_cy - bc["y0"]) / (bc["y1"] - bc["y0"])
        sk_cz = bc["z_top0"] + sk_frac * (bc["z_top1"] - bc["z_top0"])
        self.box((bc["cx"] - (bc["width"] / 2 + 0.035), sk_cy,
                  sk_cz - 0.085), (0.02, sk_len / 2, 0.105), FRAME,
                 tag="cheekCn", euler_deg=(math.degrees(ang), 0, 0))
        for y, z in ((bc["y0"] + 0.065, bc["z_top0"] - r - 0.010),
                     (bc["y1"] + 0.03, bc["z_top1"] - r)):
            self.cyl((bc["cx"], y, z), r + 0.006, bc["width"] / 2 + 0.01,
                     (0.5, 0.52, 0.55), axis="X", tag="drumCn")

    def cages_detail(self):
        """A. ROLL-CAGE SHELLS. The collider walls stay exactly where they
        are but render nearly invisible (scene_usd.build_cage); the
        industrial read comes from this NON-COLLIDING shell on each cage's
        exact footprint: galvanized tubular edge frame (~22 mm), wire-mesh
        wall panels (translucent panel + wire grid; the REVIEW pen instead
        gets SOLID light-grey sheet panels, matte — its translucent sheets
        blew out white on camera), base frame with four casters, and a
        framed aperture where the chute crosses the wall. Route colour
        lives ONLY in signage plates and lamp housings — the former thin
        top-rail accent stripes read as loose coloured strips on camera
        and were deleted; the pen's single accent is its magenta label
        plate."""
        cages = dict(P.cages_for())
        cages["REVIEW"] = P.REVIEW_PEN
        for zone, cage in cages.items():
            cx, cy = cage["center"]
            ix, iy = cage["inner"]
            t, h = cage["wall_t"], cage["wall_h"]
            hx, hy = ix / 2 + t, iy / 2 + t      # collider outer half-extents
            px_, py_ = hx + 0.010, hy + 0.010    # shell plane just outside
            z_top = t + h
            open_side = cage.get("open_side")
            route = P.ROUTE_RGBA.get(zone, (0.5, 0.5, 0.5))
            walls = {"+y": (0, 1), "-y": (0, -1), "+x": (1, 1), "-x": (1, -1)}
            # tubular horizontal rails: top rail on EVERY wall (the cage's
            # top edge), mid + bottom rails only on the closed walls (they
            # would cross the chute aperture on the open one)
            for side, (axis_x, sgn) in walls.items():
                zs = ((z_top,) if side == open_side
                      else (z_top, 0.10 + (z_top - 0.10) / 2, 0.10))
                for zz in zs:
                    if axis_x:           # wall normal +-x, rail runs along y
                        self.cyl((cx + sgn * px_, cy, zz), 0.011, hy, GALV,
                                 axis="Y", tag=f"cage{zone}_rail",
                                 roughness=GALV_R, metallic=GALV_M)
                    else:                # wall normal +-y, rail runs along x
                        self.cyl((cx, cy + sgn * py_, zz), 0.011, hx, GALV,
                                 axis="X", tag=f"cage{zone}_rail",
                                 roughness=GALV_R, metallic=GALV_M)
                if side == open_side:
                    continue
                # wall panel: C/D keep the wire-mesh look (translucent grey
                # sheet + vertical wires); the REVIEW pen gets SOLID
                # light-grey sheet-metal panels bound matte (rough 0.70) —
                # its unbound translucent sheets blew out white on camera
                pen = zone == "REVIEW"
                p_col = (0.58, 0.60, 0.62) if pen else (0.60, 0.62, 0.64)
                p_op = None if pen else 0.22
                wz0, wz1 = 0.13, z_top - 0.03
                wzc, wzh = (wz0 + wz1) / 2, (wz1 - wz0) / 2
                if axis_x:
                    self.box((cx + sgn * px_, cy, wzc), (0.0015, hy - 0.04,
                             wzh), p_col, opacity=p_op, roughness=0.70,
                             tag=f"cage{zone}_mesh")
                    for wy in np.arange(-hy + 0.10, hy - 0.05, 0.13):
                        self.box((cx + sgn * px_, cy + float(wy), wzc),
                                 (0.0028, 0.0028, wzh), GALV, bind=False,
                                 tag=f"cage{zone}_wire")
                else:
                    self.box((cx, cy + sgn * py_, wzc), (hx - 0.04, 0.0015,
                             wzh), p_col, opacity=p_op, roughness=0.70,
                             tag=f"cage{zone}_mesh")
                    for wx in np.arange(-hx + 0.10, hx - 0.05, 0.13):
                        self.box((cx + float(wx), cy + sgn * py_, wzc),
                                 (0.0028, 0.0028, wzh), GALV, bind=False,
                                 tag=f"cage{zone}_wire")
            # aperture frame on the open wall (the chute crosses INSIDE it):
            # two verticals just outside the chute rails + a sill tube that
            # clears the chute underside (chute crosses the plane at z~0.24)
            if open_side in ("+y", "-y"):
                aw2 = cage["aperture_w"] / 2
                ap_top = cage["aperture_top"]
                sill = cage.get("sill_top", 0.20)
                wy = cy + (py_ if open_side == "+y" else -py_)
                vx = aw2 + (0.014 if zone != "REVIEW" else 0.045)
                if vx < hx - 0.01:       # REVIEW's aperture spans the wall:
                    for sgn in (-1, 1):  # its corner posts ARE the frame
                        self.cyl((cx + sgn * vx, wy,
                                  (sill + ap_top) / 2), 0.011,
                                 (ap_top - sill) / 2, GALV, axis="Z",
                                 tag=f"cage{zone}_apfrm", roughness=GALV_R,
                                 metallic=GALV_M)
                self.cyl((cx, wy, 0.185), 0.011, aw2 + 0.02, GALV, axis="X",
                         tag=f"cage{zone}_apsill", roughness=GALV_R,
                         metallic=GALV_M)
            # base frame (four square-tube edges) + four casters with forks
            for sgn in (-1, 1):
                self.box((cx, cy + sgn * py_, 0.075), (hx, 0.016, 0.016),
                         GALV, tag=f"cage{zone}_bframe", roughness=GALV_R,
                         metallic=GALV_M)
                self.box((cx + sgn * px_, cy, 0.075), (0.016, hy, 0.016),
                         GALV, tag=f"cage{zone}_bframe", roughness=GALV_R,
                         metallic=GALV_M)
            for sx in (-1, 1):
                for sy in (-1, 1):
                    wx, wy2 = cx + sx * (hx - 0.07), cy + sy * (hy - 0.07)
                    self.box((wx, wy2, 0.052), (0.020, 0.015, 0.012),
                             POWDER_DK, tag=f"cage{zone}_fork",
                             roughness=POWDER_DK_R, metallic=POWDER_DK_M)
                    self.cyl((wx, wy2, 0.030), 0.028, 0.013, BLACK, axis="Y",
                             tag=f"caster{zone}")
            # (top-rail route accent stripes deleted: thin colour strips
            # floating over the rails read as debris — route colour stays
            # in signage plates and lamp housings only)
            # printed label on the visible wall + a tall mast sign. Yaw
            # convention (rotateXYZ(90,0,yaw) on a +Z-facing quad): yaw 0 =
            # text front faces SOUTH (-y), yaw -90 = faces WEST (-x) — the
            # overview camera sits south-west, so fronts point that way
            # (double-sided quads show MIRRORED text from behind).
            # wall-level printed label only — the tall mast boards were
            # oversized signalization over the C/D boxes (user directive:
            # keep the destinations readable, not billboarded)
            txt = {"C": "C OVERSIZE", "D": "D REPACK",
                   "REVIEW": "MANUAL REVIEW"}[zone]
            fn = f"cage_{zone.lower()}.png"
            if zone == "REVIEW":
                # the pen's SOUTH face is its aperture: label the west face
                lw = min(0.68, cage["inner"][1] + 0.1)
                lz = min(0.35, h - 0.05)
                self.box((cx - px_ - 0.016, cy, lz), (0.005, lw / 2 + 0.02,
                         lw / 8 + 0.02), POWDER_DK, tag=f"cage{zone}_plate",
                         roughness=POWDER_DK_R, metallic=POWDER_DK_M)
                # the pen's SINGLE accent: magenta label plate (matches the
                # chute-sign dimming; lamps carry the rest of the colour)
                self.label(txt, fn, (cx - hx - 0.02 - 0.012, cy, lz), lw,
                           yaw_deg=-90.0,
                           bg=tuple(0.62 * v for v in route))
            else:
                lw = min(0.85, cage["inner"][0] + 0.15)
                lz = min(0.45, h - 0.05)
                self.box((cx, cy - py_ - 0.016, lz), (lw / 2 + 0.02, 0.005,
                         lw / 8 + 0.02), POWDER_DK, tag=f"cage{zone}_plate",
                         roughness=POWDER_DK_R, metallic=POWDER_DK_M)
                self.label(txt, fn, (cx, cy - hy - 0.02 - 0.012, lz), lw,
                           yaw_deg=0.0)
        # B lane label on the sorter infeed (west face) — MOUNTED on a
        # square-tube signpost off the belt-B west skirt (a free-floating
        # board read as debris on the showcase footage)
        b = P.BELT_B
        sign_x = b["cx"] - b["width"] / 2 - 0.06
        self.box((sign_x + 0.020, 5.0, 0.94), (0.018, 0.018, 0.94), FRAME,
                 tag="lane_b_post")
        self.box((sign_x + 0.014, 5.0, 1.35), (0.006, 0.30, 0.075),
                 POWDER_DK, tag="lane_b_plate", roughness=POWDER_DK_R,
                 metallic=POWDER_DK_M)
        self.label("B SORTER", "lane_b.png", (sign_x, 5.0, 1.35), 0.95,
                   yaw_deg=-90.0)
        # brand board over the vision station gantry, facing the camera
        self.label("OZON  SORT CELL", "brand.png",
                   (P.VIRTUAL_SENSOR["overhead_pos"][0], 3.9, 2.65), 1.6,
                   yaw_deg=0.0)

    def sensors_hw(self):
        vs = P.VIRTUAL_SENSOR
        ox, oy, oz = vs["overhead_pos"]
        # portal gantry across belt A: square-tube posts + beam + knee braces
        for sgn in (-1, 1):
            self.box((ox, oy + sgn * 1.05, 1.25), (0.045, 0.045, 1.25),
                     FRAME, tag="gantry_post")
            self.box((ox, oy + sgn * 1.05, 0.02), (0.09, 0.09, 0.02),
                     POWDER_DK, tag="gantry_base", roughness=POWDER_DK_R,
                     metallic=POWDER_DK_M)
            self.box((ox, oy + sgn * 0.93, 2.40), (0.032, 0.11, 0.028),
                     FRAME, tag="gantry_brace", euler_deg=(sgn * 40.0, 0, 0))
        self.box((ox, oy, 2.52), (0.05, 1.10, 0.05), FRAME, tag="gantry_beam")
        # overhead head: rigid square-tube drop from the beam onto the
        # housing (it hung from a 15 mm stub with an air gap below the beam)
        self.box((ox, oy, 2.41), (0.022, 0.022, 0.062), FRAME,
                 tag="cam_overhead_drop")
        self.camera_box((ox, oy, oz), tag="cam_overhead")
        # two side profiler heads on stalks, angled inward
        off, zc = vs["side_head_offset_m"], vs["side_head_z_m"]
        for sgn, nm in ((-1, "l"), (1, "r")):
            py = oy + sgn * off
            self.box((ox, py, zc / 2 - 0.03), (0.022, 0.022, zc / 2 - 0.03),
                     FRAME, tag=f"profpost_{nm}")
            self.box((ox, py, 0.02), (0.055, 0.055, 0.02), POWDER_DK,
                     tag=f"profbase_{nm}", roughness=POWDER_DK_R,
                     metallic=POWDER_DK_M)
            # mount collar closing the post-top -> housing-bottom gap
            self.box((ox, py, 1.028), (0.024, 0.024, 0.044), FRAME,
                     tag=f"profcollar_{nm}")
            self.box((ox, py + sgn * 0.02, zc + 0.055), (0.055, 0.042, 0.038),
                     BLACK, tag=f"profiler_{nm}")
            self.cyl((ox, py - sgn * 0.028, zc + 0.04), 0.02, 0.006,
                     (0.02, 0.02, 0.025), axis="Y", tag=f"proflens_{nm}")
        # close-range MACRO head (dual-range small-item metrology): hangs from
        # the gantry beam on a slim drop tube, looking straight down from
        # 1.42 m — 0.2 m above the 0.5 m max inbound envelope. Every part of
        # the mount stays ABOVE the sensor's measuring volume ceiling
        # (belt + 0.56 m), so it can never enter a depth segmentation.
        mx, my, mz = vs["macro_pos"]
        self.cyl((mx + 0.09, my, (mz + 0.06 + 2.47) / 2), 0.014,
                 (2.47 - mz - 0.06) / 2, FRAME, tag="macrodrop")
        self.box((mx + 0.045, my, mz + 0.055), (0.06, 0.025, 0.016),
                 FRAME, tag="macroarm")
        self.camera_box((mx, my, mz), tag="cam_macro", connector=False)
        # macro connector + conduit live at the TOP of the drop tube — every
        # new part stays above the perception-corridor ceiling (z 2.3)
        self.box((mx + 0.09, my, 2.44), (0.018, 0.016, 0.012),
                 (0.10, 0.11, 0.13), tag="cam_macro_conn")
        self._conduit([(mx + 0.112, my, 2.44), (mx + 0.112, my, 2.545),
                       ((mx + ox) / 2, my, 2.545), (ox, my, 2.545)],
                      tag="macro_conduit")
        # jam camera: ceiling mast that stops ABOVE the lens plane (a pole
        # through the frame centre blinds the jam locator), now tied into
        # the roof deck with an upper mast leg + a horizontal truss arm
        self.box((8.15, 2.3, 4.35), (0.03, 0.03, 0.42), FRAME, tag="jammast")
        self.box((8.15, 2.3, 5.24), (0.03, 0.03, 0.47), FRAME,
                 tag="jammast_up")
        self.box((8.15, 2.15, 5.35), (0.025, 0.155, 0.025), FRAME,
                 tag="jammast_arm")
        self._conduit([(8.19, 2.3, 3.95), (8.19, 2.3, 5.33),
                       (8.19, 2.12, 5.33)], tag="jam_conduit")
        self.camera_box((8.15, 2.3, 3.8), tag="cam_jam")
        # industrial sensing detail (§6): cable tray along the gantry beam,
        # rigid CONDUIT runs to each head (thin hanging cable lines read as
        # floating debris on RTX footage), small status LEDs on the housings
        self.box((ox, oy, 2.585), (0.05, 1.10, 0.012), (0.42, 0.44, 0.47),
                 tag="cabletray")
        for hy, hz in ((oy, oz + 0.12), (oy - off, zc + 0.09),
                       (oy + off, zc + 0.09)):
            self._conduit([(ox + 0.04, hy, hz), (ox + 0.04, hy, 2.57)],
                          tag="camconduit")
            self.box((ox + 0.055, hy, hz), (0.006, 0.006, 0.006),
                     (0.15, 0.75, 0.25), tag="camled")

    def _truss(self, x0, x1, y, z_bot=5.15, z_top=5.55):
        """A visible steel roof-truss girder running along x at height z:
        top + bottom chords with a zig-zag web (the ceiling structure the
        area lights hang from). Non-colliding, sensor-invisible dressing."""
        col = (0.34, 0.36, 0.40)
        hx = (x1 - x0) / 2
        cx = (x0 + x1) / 2
        for zc in (z_bot, z_top):                       # chords
            self.box((cx, y, zc), (hx, 0.03, 0.025), col, tag="truss_chord")
        n = max(2, int((x1 - x0) / 0.7))
        for i in range(n):                              # zig-zag web
            xa = x0 + (i + 0.5) * (x1 - x0) / n
            self.cyl((xa, y, (z_bot + z_top) / 2), 0.012,
                     (z_top - z_bot) / 2 + 0.02, col, axis="Z",
                     tag="truss_web")
        for xa in (x0 + 0.05, x1 - 0.05):               # end posts to roof
            self.box((xa, y, z_top + 0.25), (0.03, 0.03, 0.25), col,
                     tag="truss_post")

    def _highbay(self, cx, cy, w, d, intensity):
        """Overhead LED high-bay AREA light on the truss: a downward RectLight
        (even, soft, shadow-friendly illumination over the whole cell — no
        local hot spots) inside a visible fixture housing + reflector.

        A USD RectLight emits from its -Z face; with the prim unrotated at
        the ceiling that is straight down, so NO rotation is applied (an
        earlier RotateX(180) flipped it to light the roof — the whole cell
        went black). The visible housing/reflector geometry sits ABOVE the
        emitter plane so it never occludes the downward light."""
        st = self.stage
        z = 5.05
        self._i += 1
        lp = f"/World/lights/highbay_{self._i}"
        rect = UsdLux.RectLight.Define(st, lp)
        rect.CreateWidthAttr(float(w))
        rect.CreateHeightAttr(float(d))
        rect.CreateIntensityAttr(float(intensity))
        rect.GetPrim().CreateAttribute("inputs:normalize",
                                       Sdf.ValueTypeNames.Bool).Set(True)
        rect.GetPrim().CreateAttribute("inputs:color",
                                       Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(1.0, 0.96, 0.89))
        UsdGeom.Xformable(rect.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(cx, cy, z))                        # unrotated -> emits -Z
        # visible fixture ABOVE the emitter: dark housing + bright diffuser
        # face (reads as a lit LED panel from below without blocking light)
        self.box((cx, cy, z + 0.12), (w / 2 + 0.03, d / 2 + 0.03, 0.05),
                 (0.12, 0.13, 0.15), tag="hb_housing")
        self.box((cx, cy, z + 0.055), (w / 2, d / 2, 0.004),
                 (0.95, 0.95, 0.92), tag="hb_face")

    def lighting_env(self):
        st = self.stage
        # soft neutral ambient FILL only (the area lights are the key light)
        # — raised ~25% (presentation pass: shadows lift, no exposure change)
        dome = UsdLux.DomeLight(st.GetPrimAtPath("/World/lights/dome"))
        if dome:
            dome.GetIntensityAttr().Set(190.0)
            dome.GetPrim().CreateAttribute(
                "inputs:color", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(0.55, 0.58, 0.64))
        # kill the hard directional sun: overhead area lights carry the scene
        sun = UsdLux.DistantLight(st.GetPrimAtPath("/World/lights/sun"))
        if sun:
            sun.GetIntensityAttr().Set(120.0)
            sun.GetPrim().CreateAttribute(
                "inputs:angle", Sdf.ValueTypeNames.Float).Set(3.0)
        # ceiling truss girders spanning the whole cell (two bays along y)
        for ty in (2.0, 4.0):
            self._truss(0.4, 9.7, ty)
        # roof deck panel above the truss (so lights read as ceiling-mounted,
        # not floating) — dark, high, out of every camera's action framing
        self.box((5.0, 3.0, 5.75), (5.0, 3.1, 0.04), (0.12, 0.13, 0.15),
                 tag="roofdeck")
        # AREA-light grid on the truss: even overhead coverage of the whole
        # 10x6 cell — no local spotlight near the arm, no blown-out corner.
        # RectLights emit downward; normalize keeps brightness size-stable.
        # Intensity +22% (presentation pass: broad fill up, exposure fixed).
        for (lx, ly, w, d) in ((1.6, 3.0, 2.4, 3.0), (4.2, 3.0, 2.6, 3.0),
                               (6.6, 3.0, 2.4, 3.2), (8.7, 2.6, 2.6, 3.4)):
            self._highbay(lx, ly, w, d, 22000.0)
        # ONE key-zone accent over the vision/measurement station only. The
        # former routing-deck accent (8.25, 3.0) sat directly over the
        # arm/cage corner and printed hot specular patches on the cage
        # shells — removed; the deck is carried by the high-bays + the two
        # aisle soft lights below.
        for (lx, ly, w, d, inten) in ((6.0, 3.0, 1.1, 1.3, 6800.0),):
            self._i += 1
            lp = f"/World/lights/accent_{self._i}"
            r = UsdLux.RectLight.Define(st, lp)
            r.CreateWidthAttr(float(w))
            r.CreateHeightAttr(float(d))
            r.CreateIntensityAttr(float(inten))
            r.GetPrim().CreateAttribute("inputs:normalize",
                                        Sdf.ValueTypeNames.Bool).Set(True)
            r.GetPrim().CreateAttribute("inputs:color",
                                        Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(1.0, 0.97, 0.92))
            UsdGeom.Xformable(r.GetPrim()).AddTranslateOp().Set(
                Gf.Vec3d(lx, ly, 4.75))             # unrotated -> emits down
        # two LARGE SOFT area lights along the south presentation aisle at
        # z 4.0, neutral colour: they wrap the machine faces the hero/routing
        # cameras see, with soft normalized falloff (no hot cage speculars)
        for lx in (3.0, 7.2):
            self._i += 1
            lp = f"/World/lights/aisle_{self._i}"
            r = UsdLux.RectLight.Define(st, lp)
            r.CreateWidthAttr(2.8)
            r.CreateHeightAttr(1.4)
            r.CreateIntensityAttr(9000.0)
            r.GetPrim().CreateAttribute("inputs:normalize",
                                        Sdf.ValueTypeNames.Bool).Set(True)
            r.GetPrim().CreateAttribute("inputs:color",
                                        Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(1.0, 1.0, 1.0))
            UsdGeom.Xformable(r.GetPrim()).AddTranslateOp().Set(
                Gf.Vec3d(lx, 0.9, 4.0))             # unrotated -> emits down
        # backdrop walls: kill the white void on the camera-facing sides —
        # plain matte concrete (a tiled detail map reads as wallpaper on a
        # big flat wall)
        self.box((5.0, 6.35, 2.6), (7.5, 0.06, 2.6), (0.17, 0.19, 0.23),
                 tag="wall_n", textured=False)
        self.box((10.6, 3.0, 2.6), (0.06, 3.6, 2.6), (0.17, 0.19, 0.23),
                 tag="wall_e", textured=False)
        # yellow walkway markings on the floor (industrial)
        for y in (0.6, 5.6):
            self.box((4.8, y, 0.003), (4.6, 0.045, 0.001), YELLOW,
                     tag="floorline")
        self.box((9.9, 3.05, 0.003), (0.045, 2.5, 0.001), YELLOW,
                 tag="floorline")

    # ------------------------------------------------------ conveyor details
    def conveyor_details(self):
        """Hazard striping, plinth + legs cladding, and white direction
        chevrons painted on the belts (the visible roller/flow direction) —
        the ONLY painted arrows in the cell live ON the two wide fixed belt
        surfaces, where they read as conveyor paint."""
        a, b = P.BELT_A, P.BELT_B
        # black dashes over the yellow side guides -> yellow/black safety
        # edge. ONLY when the official shells are absent: the shells carry
        # their own side rails, and doubled rail hardware crowds the freight
        # corridor on camera (items read as clipping through the stripes)
        if not getattr(self, "shells", False):
            for x in np.arange(0.3, a["knife_x0"] - 0.1, 0.45):
                for sgn in (-1, 1):
                    self.box((float(x), a["y"] + sgn * (a["width"] / 2 + 0.015),
                              a["top"] + 0.041), (0.11, 0.017, 0.051), BLACK,
                             tag="hzA")
        # plinth legs on belt A and belt B (reads as supports)
        legs_needed = not getattr(self, "shells", False)
        for x in (np.arange(0.6, a["knife_x0"] - 0.3, 1.2)
                  if legs_needed else []):
            for sgn in (-1, 1):
                self.box((float(x), a["y"] + sgn * (a["width"] / 2 + 0.045),
                          0.26), (0.045, 0.028, 0.26), FRAME, tag="legA2")
        for y in (np.arange(b["y0"] + 0.4, b["y1"] - 0.2, 1.2)
                  if legs_needed else []):
            for sgn in (-1, 1):
                self.box((b["cx"] + sgn * (b["width"] / 2 + 0.045), float(y),
                          0.26), (0.028, 0.045, 0.26), FRAME, tag="legB2")
        # white direction chevrons ON the belts (1 mm thick, far below the
        # perception z-margin) — the flow direction is unmistakable
        wht = (0.92, 0.92, 0.95)
        for x in np.arange(0.6, a["knife_x0"] - 0.25, 0.55):
            for j, sw in ((0, 40.0), (1, -40.0)):
                self.box((float(x) - 0.03 * j, a["y"] + (0.05 if j else -0.05),
                          a["top"] + 0.0005), (0.075, 0.012, 0.0004), wht,
                         tag="dirA", euler_deg=(0, 0, sw))
        for y in np.arange(b["y0"] + 0.3, b["y1"] - 0.2, 0.5):
            for j, sw in ((0, 50.0), (1, 130.0)):
                self.box((b["cx"] + (0.05 if j else -0.05), float(y),
                          b["top"] + 0.0005), (0.075, 0.012, 0.0004), wht,
                         tag="dirB", euler_deg=(0, 0, sw))
        # (incline chevrons removed: on the narrow slope they read as
        # loose strips on camera — wide fixed belts keep painted chevrons)

    # -------------------------------------------------- B transfer continuity
    def b_transfer(self):
        """C. The tray -> incline handoff reads as engineered hardware:
        a transition nose apron under the incline mouth, a side-mounted
        drive motor + gearbox at the head drum, and support legs under the
        span. All NON-COLLIDING, all positioned from P.B_CONNECT, all beside
        or UNDER the belt surface plane (never above it, where items slide)
        and clear of the tray-sweep corridor (nothing above z 0.36 south of
        y 3.42; the sweep bottoms at z 0.62 over y 3.31). The end drums both
        ends already come from rollers()."""
        bc = P.B_CONNECT
        slope = (bc["z_top1"] - bc["z_top0"]) / (bc["y1"] - bc["y0"])
        # transition nose apron: steep deflector plate under the mouth,
        # closing the visual void between the tray lip line and the belt.
        # SWEEP-AUDIT FIX: the old apron reached z 0.28 and its under-skirt
        # z 0.21 — both inside the RETURN-leg tray band (top 0.30 at
        # return_z 0.24). Apron shortened + raised (bottom 0.352, 23 mm
        # above the return lip chamfers); the skirt is deleted — the moved
        # tail drum now covers that view line.
        self.box((bc["cx"], 3.298, 0.375), (bc["width"] / 2 - 0.01, 0.030,
                 0.004), BRUSHED, tag="bnose_apron", euler_deg=(-50.0, 0, 0),
                 roughness=BRUSHED_R, metallic=BRUSHED_M)
        # head-drum drive: gearbox block + motor cylinder + label, mounted
        # beside the east edge at the top end (y 4.23 — far north of 3.42)
        my_, mz_ = bc["y1"] + 0.03, bc["z_top1"] - 0.035
        gx = bc["cx"] + bc["width"] / 2 + 0.10
        self.box((gx, my_, mz_), (0.045, 0.050, 0.050), POWDER_DK,
                 tag="bdrive_gearbox", roughness=POWDER_DK_R,
                 metallic=POWDER_DK_M)
        self.cyl((gx + 0.115, my_, mz_), 0.042, 0.070, POWDER_DK, axis="X",
                 tag="bdrive_motor", roughness=POWDER_DK_R,
                 metallic=POWDER_DK_M)
        self.cyl((gx - 0.065, my_, mz_), 0.016, 0.055, BRUSHED, axis="X",
                 tag="bdrive_shaft", roughness=BRUSHED_R, metallic=BRUSHED_M)
        self.label("B-LIFT DRIVE", "bdrive.png", (gx + 0.19, my_, mz_), 0.15,
                   yaw_deg=90.0)
        # support legs + cross braces under the span. SWEEP-AUDIT FIX: the
        # flank legs at +-0.345 stood 15 mm inside the parallel REVIEW slide
        # corridor (wide freight on REVIEW reaches x 8.71); legs moved to
        # +-0.27 — centre supports fully under the belt, clear of both the
        # REVIEW corridor (west edge 8.712) and the D corridor.
        for ly in (3.62, 4.06):
            surf = bc["z_top0"] + (ly - bc["y0"]) * slope
            hcz = (surf - 0.055) / 2
            for sgn in (-1, 1):
                self.box((bc["cx"] + sgn * 0.27, ly,
                          hcz), (0.022, 0.022, hcz - 0.004), FRAME,
                         tag="bconn_leg")
            self.box((bc["cx"], ly, 0.14), (0.27 + 0.011, 0.018,
                     0.018), FRAME, tag="bconn_brace")

    # ------------------------------------------------------------ chute shells
    def chute_shells(self):
        """D. Brushed-steel under-shells + edge trim under each gravity
        chute (C / D / REVIEW), following the exact 32-deg slope a few mm
        BELOW the collider surface (items never touch them), a brake-pad end
        trim inside the destination, and per-chute station signage yawed to
        face the east camera line."""
        for zone, cc in (("C", P.CHUTE_C), ("D", P.CHUTE_D),
                         ("REVIEW", P.CHUTE_REVIEW)):
            d = float(cc["dir"])
            y0, z0, z1 = cc["y0"], cc["z0"], cc["z1"]
            y1, pad_end = P.chute_run(cc)
            cx = cc["cx"]
            length = float(np.hypot(y1 - y0, z0 - z1))
            ang = -d * 32.0
            mid, zmid = (y0 + y1) / 2, (z0 + z1) / 2 - 0.015
            # under-shell panel: 10 mm below the collider underside.
            # SWEEP-AUDIT FIX: the shell's up-slope overhang (+15 mm past
            # the mouth) poked through the slide plane into the item
            # corridor AND grazed the RETURN-leg tray lips passing under
            # the mouth — the shell now starts 75 mm DOWN-slope of the
            # mouth (y-band fully clear of the return trays' +-0.31 reach).
            in_y = 0.150                       # down-slope inset at the mouth
            sh_mid_y = mid + d * (in_y / 2) * math.cos(math.radians(32.0))
            sh_mid_z = zmid - 0.031 - (in_y / 2) * math.sin(math.radians(32.0))
            sh_len = length - in_y
            self.box((cx, sh_mid_y, sh_mid_z), (cc["width"] / 2 + 0.030,
                     sh_len / 2, 0.006), BRUSHED, tag=f"chsh{zone}",
                     euler_deg=(ang, 0, 0), roughness=BRUSHED_R,
                     metallic=BRUSHED_M)
            # edge trim bands under both slope edges — FLUSH against the
            # shell underside (the old 20 mm-deep hanging bands dipped into
            # the return-leg tray band)
            for sgn in (-1, 1):
                self.box((cx + sgn * (cc["width"] / 2 + 0.024), sh_mid_y,
                          sh_mid_z - 0.002), (0.006, sh_len / 2, 0.008),
                         BRUSHED, tag=f"chtrim{zone}", euler_deg=(ang, 0, 0),
                         roughness=BRUSHED_R, metallic=BRUSHED_M)
            # brake-pad end trim (below the pad's working surface)
            self.box((cx, pad_end - d * 0.012, z1 - 0.033),
                     (cc["width"] / 2 + 0.020, 0.010, 0.012), POWDER_DK,
                     tag=f"chpadtrim{zone}", roughness=POWDER_DK_R,
                     metallic=POWDER_DK_M)
            # (chute-side signs REMOVED — sweep-audit fix: the coloured quad
            # pairs on 10 mm sticks stood in the inter-chute aisle where the
            # exception arm operates and read as floating coloured slats
            # from the cage angles; route identification is carried by the
            # cage label plates and the station portal beacons.)

    # ------------------------------------------------------- route visuals
    def route_viz(self):
        """Route storytelling WITHOUT floor/surface decals. The former
        chevron trails (station -> destination runs on the floor, the
        chutes, the B incline and belt B) read as scattered plastic litter
        on the jury footage and were DELETED — the route-colour language
        now lives ONLY in signage plates and lamp housings; the only
        painted arrows left are the white conveyor chevrons ON belts A/B
        (conveyor_details). Returns prim paths for runtime lamp
        brightness."""
        S = P.SORTER
        viz = {"lamps": {}}
        # name the mechanism on the hardware — centred on the SOLID skirt
        # segment between the C and D station cutouts (7.81..8.39): the old
        # 0.72-wide board at x 8.05 overhung BOTH cutouts and floated over
        # the discharge openings (sweep-audit fix)
        self.label("TILT-TRAY SORTER", "sorter_deck.png",
                   (8.10, S["y"] - 0.395, 0.55), 0.52, yaw_deg=0.0)
        dim = {z: tuple(0.35 * v for v in P.ROUTE_RGBA[z])
               for z in P.ROUTE_RGBA}
        # ACTIVE ROUTE status as a LOW floor-standing HMI console (was a
        # 1.9 m mast that read as a vertical stick beside the deck) — the
        # requested HMI/status panel. Placed north-west of the table, out of
        # the arm workspace and clear of the deck sightline. Andon pass:
        # enclosure door seam + handle, recessed lamp bezels, a 3-lens stack
        # light on a short mast, and a floor conduit run toward the deck.
        px, py = 7.35, 4.75
        self.box((px, py, 0.44), (0.22, 0.13, 0.44), FRAME, tag="hmi_body")
        # 45-deg bevel strip along the console's south top edge (H: bevels
        # on the most visible shells) + door seam + handle on the south face
        self.box((px, py - 0.125, 0.885), (0.22, 0.012, 0.012), POWDER_DK,
                 tag="hmi_bevel", euler_deg=(45.0, 0, 0),
                 roughness=POWDER_DK_R, metallic=POWDER_DK_M)
        self.box((px, py - 0.132, 0.36), (0.155, 0.0015, 0.30),
                 (0.055, 0.058, 0.066), tag="hmi_door_seam")
        self.box((px + 0.125, py - 0.138, 0.42), (0.008, 0.007, 0.045),
                 (0.10, 0.11, 0.13), tag="hmi_handle")
        self.box((px, py - 0.12, 0.82), (0.22, 0.02, 0.14), (0.04, 0.04, 0.05),
                 tag="hmi_screen_bezel")
        self.label("ACTIVE ROUTE", "active_route.png", (px, py - 0.135, 0.95),
                   0.44, yaw_deg=0.0)
        for i, z in enumerate(("B", "C", "D", "REVIEW")):
            cap = "R" if z == "REVIEW" else z
            lx = px - 0.165 + 0.11 * i
            # dark bezel FRAME proud of the lamp face: the lamp face sits
            # recessed ~11 mm inside its own housing opening
            self.box((lx, py - 0.145, 0.858), (0.050, 0.016, 0.008),
                     (0.055, 0.058, 0.066), tag=f"lampbezel{z}")
            self.box((lx, py - 0.145, 0.742), (0.050, 0.016, 0.008),
                     (0.055, 0.058, 0.066), tag=f"lampbezel{z}")
            for sgn in (-1, 1):
                self.box((lx + sgn * 0.050, py - 0.145, 0.80),
                         (0.008, 0.016, 0.066), (0.055, 0.058, 0.066),
                         tag=f"lampbezel{z}")
            lp = self.box((lx, py - 0.13, 0.80),
                          (0.042, 0.02, 0.05), dim[z], tag=f"lamp{z}",
                          bind=False)
            viz["lamps"][z] = lp.GetPath().pathString
            self.label(cap, f"lampcap_{cap}.png",
                       (lx, py - 0.135, 0.68), 0.085,
                       yaw_deg=0.0, bg=tuple(0.55 * v for v in P.ROUTE_RGBA[z]))
        # 3-lens stack light (red/amber/green translucent) on a short mast
        self.cyl((px + 0.14, py, 0.925), 0.010, 0.045, POWDER_DK,
                 tag="stack_mast", roughness=POWDER_DK_R,
                 metallic=POWDER_DK_M)
        for k, lens_col in enumerate(((0.80, 0.10, 0.08), (0.85, 0.55, 0.06),
                                      (0.12, 0.68, 0.18))):
            lens = self.cyl((px + 0.14, py, 0.995 + 0.046 * k), 0.026, 0.022,
                            lens_col, tag="stack_lens", bind=False)
            UsdGeom.Gprim(lens).CreateDisplayOpacityAttr([0.55])
        self.cyl((px + 0.14, py, 1.135), 0.028, 0.006, BLACK,
                 tag="stack_cap")
        # floor cable conduit strip: console base -> deck north skirt line
        self.box((px, (py - 0.14 + 3.44) / 2, 0.012),
                 (0.045, (py - 0.14 - 3.44) / 2, 0.012), (0.34, 0.36, 0.40),
                 tag="hmi_floorduct")
        # E-STOPS (red mushroom on yellow plate): one on the console flank,
        # one on the south chassis beam (mid C-D segment), one on the north
        # beam west segment — all outside the tray-sweep corridor y 2.62-3.38
        # and clear of every station discharge cutout.
        # (south-beam unit lives on the short east segment x 9.11..9.22 —
        # the C-D mid segment carries the TILT-TRAY SORTER label)
        for (ex, ey, ez, nrm) in ((px + 0.22, py, 0.62, "+x"),
                                  (9.165, 2.590, 0.55, "-y"),
                                  (7.40, 3.410, 0.55, "+y")):
            if nrm == "+x":
                self.box((ex + 0.004, ey, ez), (0.004, 0.045, 0.045), YELLOW,
                         tag="estop_plate", roughness=YELLOW_R,
                         metallic=YELLOW_M)
                self.cyl((ex + 0.016, ey, ez), 0.022, 0.011,
                         (0.70, 0.06, 0.05), axis="X", tag="estop_btn")
            else:
                sgn = -1.0 if nrm == "-y" else 1.0
                self.box((ex, ey + sgn * 0.004, ez), (0.045, 0.004, 0.045),
                         YELLOW, tag="estop_plate", roughness=YELLOW_R,
                         metallic=YELLOW_M)
                self.cyl((ex, ey + sgn * 0.016, ez), 0.022, 0.011,
                         (0.70, 0.06, 0.05), axis="Y", tag="estop_btn")
        # pinch-point warning labels on the C / D station portal posts —
        # posts moved to +-0.44 (they used to stand INSIDE the chute width;
        # these labels hung 98 mm inside the item corridor with them)
        for wx in (P.STATIONS["C"]["x"] - 0.44, P.STATIONS["D"]["x"] + 0.44):
            self.label("! PINCH POINT", "pinch.png", (wx, 2.514, 0.78), 0.13,
                       yaw_deg=0.0, bg=YELLOW, fg=(25, 25, 25))
        # (floating per-item route flags fully retired — no textures, no
        # runtime quads: they read as hovering debris over the freight)
        return viz

    # ---------------------------------------------------------- Ozon brand
    def ozon_brand(self):
        """Ozon design language (brandlab.ozon.ru): Ozon blue + magenta
        accent, white lowercase wordmark, clean panels."""
        MAGENTA = (0.83, 0.07, 0.55)
        # big wordmark on the backdrop wall: the OFFICIAL logo image
        # (wikimedia Ozon_logo_clear.svg), composited onto a white panel
        try:
            from PIL import Image
            logo = Image.open(str(SIGN_DIR / "ozon_logo.png")).convert("RGBA")
            W = 1280
            H = W // 4
            panel = Image.new("RGBA", (W, H), (255, 255, 255, 255))
            lw = int(W * 0.72)
            lh = int(lw * logo.height / logo.width)
            lg = logo.resize((lw, lh))
            panel.alpha_composite(lg, ((W - lw) // 2, (H - lh) // 2))
            panel.convert("RGB").save(str(SIGN_DIR / "ozon_wall.png"))
            # flush against the wall face (y 6.29): the old y 6.27 board
            # floated 20 mm proud and read as a hovering plate from the side
            self._textured_quad("ozon_wall", str(SIGN_DIR / "ozon_wall.png"),
                                (5.0, 6.286, 3.4), 2.6, yaw_deg=0.0)
        except Exception as exc:
            print(f"[dressing] logo board fallback ({exc})", flush=True)
            self.label("ozon", "ozon_wall_txt.png", (5.0, 6.286, 3.4), 2.6,
                       yaw_deg=0.0)
        self.box((5.0, 6.284, 2.94), (1.3, 0.005, 0.035), MAGENTA,
                 tag="brand_accent")
        # blue band + magenta kick strip along the sorter-train south skirt.
        # SEGMENTED to the actual skirt panels (sweep-audit fix): the old
        # continuous strips crossed the C and D station discharge cutouts
        # where there is no skirt behind them — free-floating coloured bars
        # across the openings, visible through the cage apertures. Brand
        # colour belongs ON panels, so the strips now exist only where the
        # panel exists.
        S = P.SORTER
        cuts = sorted((P.STATIONS[k]["x"] - 0.36, P.STATIONS[k]["x"] + 0.36)
                      for k in ("C", "D"))
        segs, cur = [], S["x_west"] + 0.05
        for c0, c1 in cuts:
            if c0 > cur:
                segs.append((cur, c0))
            cur = max(cur, c1)
        if cur < S["x_east"] - 0.05:
            segs.append((cur, S["x_east"] - 0.05))
        for a_, b_ in segs:
            if b_ - a_ < 0.08:
                continue
            self.box(((a_ + b_) / 2, S["y"] - 0.394, 0.435),
                     ((b_ - a_) / 2, 0.005, 0.028), OZON_BLUE,
                     tag="ozon_band")
            self.box(((a_ + b_) / 2, S["y"] - 0.394, 0.385),
                     ((b_ - a_) / 2, 0.005, 0.011), MAGENTA,
                     tag="ozon_kick")
        # blue crossbeam accent on the vision gantry
        vs = P.VIRTUAL_SENSOR
        self.box((vs["overhead_pos"][0], vs["overhead_pos"][1], 2.46),
                 (0.052, 1.10, 0.012), OZON_BLUE, tag="gantry_accent")

    # ---------------------------------------------------- industrial context
    def industrial_context(self):
        """Subtle warehouse context to fill the empty walls/floor WITHOUT
        clutter — control cabinet, safety fencing, cable trays, structural
        columns, an infeed light curtain, e-stops, and floor safety zones.
        All on the NORTH/EAST backdrop side so nothing occludes the open
        south-west presentation cameras; all non-colliding + sensor-safe
        (flat floor decals sit under the 6 mm perception z-margin, and every
        item is well clear of the vision window at x 5.85-6.28 and the
        routing/jam camera crop)."""
        FR = (0.28, 0.30, 0.34)
        DK = (0.10, 0.11, 0.13)
        RED = (0.55, 0.06, 0.06)
        HAZ = (0.42, 0.36, 0.10)

        # --- structural building columns at the far (N) corners: floor->truss
        # I-beam-ish box columns with base plates (real structure, not sticks)
        for cx in (0.5, 9.6):
            self.box((cx, 6.15, 2.75), (0.08, 0.08, 2.75), FR, tag="col",
                     textured=False)
            self.box((cx, 6.15, 0.05), (0.16, 0.16, 0.05), DK, tag="col_base")
        # --- wall-mounted electrical control cabinet on the north wall
        ccx = 2.6
        self.box((ccx, 6.16, 0.95), (0.45, 0.12, 0.62), FR, tag="ecab",
                 textured=False)
        self.box((ccx, 6.03, 0.95), (0.42, 0.02, 0.58), (0.20, 0.22, 0.26),
                 tag="ecab_door")            # door face
        self.box((ccx + 0.30, 6.00, 0.95), (0.03, 0.015, 0.10), DK,
                 tag="ecab_handle")
        for zz in (1.34, 1.40, 1.46):        # ventilation louvres
            self.box((ccx, 6.02, zz), (0.30, 0.006, 0.012), DK, tag="ecab_vent")
        self.box((ccx - 0.28, 6.00, 1.30), (0.05, 0.012, 0.05), RED,
                 tag="ecab_estop")           # e-stop on the cabinet
        # small wall HMI screen beside the cabinet
        self.box((ccx + 0.75, 6.05, 1.20), (0.16, 0.02, 0.11),
                 (0.05, 0.05, 0.06), tag="wall_hmi")
        self.box((ccx + 0.75, 6.03, 1.20), (0.14, 0.008, 0.09),
                 (0.10, 0.22, 0.30), tag="wall_hmi_scr")

        # --- cable trays: along the north wall from the cabinet, + a drop
        for x0, x1, yy, zz in ((1.0, 9.2, 6.22, 2.35),):
            self.box(((x0 + x1) / 2, yy, zz), ((x1 - x0) / 2, 0.05, 0.02),
                     (0.34, 0.36, 0.40), tag="cabletray_n")
        self.box((ccx, 6.20, 1.75), (0.05, 0.03, 0.55), (0.34, 0.36, 0.40),
                 tag="cabletray_drop")

        # --- safety fence sections on the EAST backdrop edge (guarding the
        # cell without blocking the SW cameras): posts + two horizontal rails
        for fy in np.arange(1.4, 5.2, 0.95):
            self.box((10.15, float(fy), 0.55), (0.03, 0.03, 0.55), FR,
                     tag="fence_post")
            for zz in (0.55, 0.95):
                self.box((10.15, float(fy) + 0.475, zz), (0.02, 0.44, 0.02),
                         HAZ, tag="fence_rail")
        self.box((10.15, 5.0, 0.15), (0.03, 0.03, 0.15), RED,
                 tag="fence_estop")          # e-stop on a fence post

        # --- infeed light curtain at belt A start: emitter + receiver strips
        a = P.BELT_A
        for sgn in (-1, 1):
            self.box((0.35, a["y"] + sgn * (a["width"] / 2 + 0.06), a["top"]
                      + 0.28), (0.02, 0.02, 0.30), (0.10, 0.10, 0.12),
                     tag="lc_strip")
            for zz in np.arange(a["top"] + 0.06, a["top"] + 0.5, 0.08):
                self.box((0.35, a["y"] + sgn * (a["width"] / 2 + 0.06),
                          float(zz)), (0.012, 0.012, 0.006), (0.75, 0.10, 0.10),
                         tag="lc_led")

        # --- floor zone signage: REMOVED. A flat text decal on the ground
        # reads correctly from only ONE side; the hero/showcase cameras orbit
        # the cell from the NE/E, so the label rendered mirrored (letters
        # reversed) from the beauty angles and would read backwards from some
        # frames of the moving showcase no matter which yaw we pick. It is the
        # same "floor litter" class the jury flagged (the hazard-hatch strips
        # were deleted for the same reason). Cell labeling is carried by the
        # upright wall/gantry signs (B SORTER, OZON SORT CELL, ACTIVE ROUTE),
        # which are back-to-back quad pairs and read correctly from every side.
        # The long yellow walkway/aisle lines (lighting_env) stay: continuous
        # directional aisle paint is orientation-agnostic and reads clean.

    # ------------------------------------------------- real sensor assets
    def sensor_assets(self):
        """Reference official Isaac Sim sensor models (Intel RealSense D455)
        as the visible camera bodies; fall back to the hand-made housings on
        any failure (offline jury box). Purely visual references — no physics
        APIs, mounted just above each lens plane."""
        try:
            try:
                from isaacsim.storage.native import get_assets_root_path
            except ImportError:
                from isaacsim.core.utils.nucleus import get_assets_root_path
            root = get_assets_root_path()
            if not root:
                raise RuntimeError("no assets root")
            import omni.client
            usd = None
            for cand in ("/Isaac/Sensors/RealSense/D455/rsd455.usd",
                         "/Isaac/Sensors/RealSense/D455/d455.usd",
                         "/Isaac/Sensors/RealSense/D455/D455.usd"):
                res, _e = omni.client.stat(root + cand)
                if res == omni.client.Result.OK:
                    usd = root + cand
                    break
            if usd is None:
                res, entries = omni.client.list(
                    root + "/Isaac/Sensors/RealSense/D455")
                names = [e.relative_path for e in entries]
                usds = [n for n in names if n.endswith(".usd")]
                if not usds:
                    raise RuntimeError(f"no D455 usd in {names[:8]}")
                usd = root + "/Isaac/Sensors/RealSense/D455/" + usds[0]
            vs = P.VIRTUAL_SENSOR
            spots = [("rs_overhead", vs["overhead_pos"], (0, 180, 0)),
                     ("rs_side_l", (vs["overhead_pos"][0],
                                    vs["overhead_pos"][1] - vs["side_head_offset_m"],
                                    vs["side_head_z_m"]), (0, 140, 90)),
                     ("rs_side_r", (vs["overhead_pos"][0],
                                    vs["overhead_pos"][1] + vs["side_head_offset_m"],
                                    vs["side_head_z_m"]), (0, 140, -90)),
                     ("rs_jam", (8.3, 2.6, 3.8), (0, 180, 0))]
            for nm, pos, rot in spots:
                path = f"{ROOT}/sensors/{nm}"
                prim = self.stage.DefinePrim(path, "Xform")
                prim.GetReferences().AddReference(usd)
                from isaac.asset_shells import sanitize
                sanitize(prim)          # kills colliders AND shipped lights
                xf = UsdGeom.Xformable(prim)
                xf.AddTranslateOp().Set(Gf.Vec3d(pos[0], pos[1],
                                                 float(pos[2]) + 0.045))
                xf.AddRotateXYZOp().Set(Gf.Vec3f(*[float(r) for r in rot]))
            print("[dressing] RealSense D455 sensor assets referenced",
                  flush=True)
            return True
        except Exception as exc:
            print(f"[dressing] sensor assets unavailable ({exc}); using "
                  f"built housings", flush=True)
            return False

    def dress(self):
        self.arb_pills = None
        self.arb_rollers = None
        try:
            from isaac.asset_shells import conveyor_shells
            res = conveyor_shells(self.stage, top=P.BELT_A["top"])
            self.shells = bool(res)
            self.arb_pills = (res or {}).get("arb_pills")
            self.arb_rollers = (res or {}).get("arb_rollers")
        except Exception as exc:
            print(f"[dressing] conveyor shells unavailable ({exc})", flush=True)
            self.shells = False
        if self.shells:
            print("[dressing] official conveyor shells referenced", flush=True)
            from pxr import UsdGeom as _UG2
            conv_root = self.stage.GetPrimAtPath("/World/conveyors")
            for pr in (conv_root.GetChildren() if conv_root else ()):
                # hide the collider boxes the shells replace (belts + every
                # ARB deck patch); the nose-over strips stay visible
                if (pr.GetName() in ("beltA", "beltB", "entry", "connectB")
                        or pr.GetName().startswith("zone_")):
                    _UG2.Imageable(pr).MakeInvisible()
            # the shells carry their own side rails: hide the primitive
            # belt-A guides VISUALLY (their collision stays authoritative)
            # so the freight corridor shows one rail system, not two
            for nm in ("beltA_guide_l", "beltA_guide_r"):
                pr = self.stage.GetPrimAtPath(f"/World/statics/{nm}")
                if pr:
                    _UG2.Imageable(pr).MakeInvisible()
        self.rollers()
        self.conveyor_details()
        self.cages_detail()
        self.b_transfer()
        self.chute_shells()
        self.sensors_hw()
        self.sensor_assets()
        self.lighting_env()
        self.ozon_brand()
        self.industrial_context()
        viz = self.route_viz()
        viz["arb_pills"] = self.arb_pills
        viz["arb_rollers"] = self.arb_rollers
        return viz


def dress_scene(stage):
    return Dressing(stage).dress()


class RouteVizRuntime:
    """Per-tick route storytelling (visuals only): the housed ACTIVE ROUTE
    console lamps brighten for the commanded route. The chevron trails and
    the floating per-item route flags are gone (floor litter / hovering
    debris on camera) — `drop_flag`/`update` keep their signatures because
    the executive still calls them every tick."""

    def __init__(self, stage, viz):
        self.stage = stage
        self.viz = viz or {}
        self.bright = {z: Gf.Vec3f(*c) for z, c in P.ROUTE_RGBA.items()}
        self.dim = {z: Gf.Vec3f(*[0.30 * v for v in c])
                    for z, c in P.ROUTE_RGBA.items()}
        self._lamp_attrs = {}
        for z, p in self.viz.get("lamps", {}).items():
            self._lamp_attrs[z] = [self._attr(p)]
        self._last_route = "?"

    def _attr(self, path):
        return UsdGeom.Gprim(self.stage.GetPrimAtPath(path)).GetDisplayColorAttr()

    def drop_flag(self, slug):
        """Compatibility no-op: per-item route flags were removed."""

    # -------------------------------------------------------------- update
    def update(self, active_route, item_positions=None):
        """active_route: commanded zone route or None. item_positions is
        accepted (and ignored) for executive API compatibility — the
        per-item flag quads no longer exist."""
        if active_route != self._last_route:
            self._last_route = active_route
            for z, attrs in self._lamp_attrs.items():
                col = self.bright[z] if z == active_route else self.dim[z]
                for at in attrs:
                    at.Set([col])
