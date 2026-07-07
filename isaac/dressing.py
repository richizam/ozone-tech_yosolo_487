# -*- coding: utf-8 -*-
"""Industrial presentation layer for the Isaac Sim cell — visuals only.

Everything here is NON-COLLIDING and placed outside the perception crop and
behind every sensor's near plane: physics, classification and the jam-camera
background are bit-identical with this layer on or off (the MuJoCo twin's
contype-0 discipline).

What it adds (user spec):
  * the DRIVE is visible: transport rollers under belt A/B and the connector,
    end drums, an omni-puck field on the ARB routing zone, dark rubber belt
    surfaces, metallic rollers;
  * industrial materials: safety-yellow guards, steel frames, dark floor with
    yellow walkway markings, dark backdrop walls (no white void);
  * real roll cages: vertical tubes, top rails, base frame with casters, and
    printed labels («C OVERSIZE», «D REPACK») in Ozon blue;
  * visible sensor hardware: gantry + black camera boxes with lenses for the
    overhead head, the two side profilers and the jam camera;
  * warehouse lighting: dimmer dome, high-bay fixtures with soft shadows.
"""
import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade, Vt

from cell import params as P

RUBBER = (0.085, 0.088, 0.095)
STEEL = (0.62, 0.64, 0.68)
FRAME = (0.30, 0.33, 0.38)
YELLOW = (0.95, 0.78, 0.06)
BLACK = (0.045, 0.045, 0.05)
OZON_BLUE = (0.0, 0.357, 1.0)                      # #005BFF
SIGN_DIR = Path("/tmp/sortmaster_signs")

ROOT = "/World/dressing"


class Dressing:
    def __init__(self, stage):
        self.stage = stage
        self._i = 0
        UsdGeom.Xform.Define(stage, ROOT)

    # ------------------------------------------------------------ primitives
    def _path(self, tag):
        self._i += 1
        return f"{ROOT}/{tag}_{self._i}"

    def box(self, center, half, color, tag="box", euler_deg=(0, 0, 0),
            opacity=None):
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
        return cube.GetPrim()

    def cyl(self, center, radius, half_h, color, axis="Z", tag="cyl"):
        c = UsdGeom.Cylinder.Define(self.stage, self._path(tag))
        c.CreateRadiusAttr(float(radius))
        c.CreateHeightAttr(float(2 * half_h))
        c.CreateAxisAttr(axis)
        UsdGeom.Xformable(c.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(*[float(v) for v in center]))
        c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
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

    def label(self, text, fname, center, width, yaw_deg=0.0, bg=OZON_BLUE,
              tilt_deg=90.0):
        """Textured label quad (UsdPreviewSurface + UsdUVTexture)."""
        tex = self._label_texture(text, fname, bg=bg)
        path = self._path("label")
        mesh = UsdGeom.Mesh.Define(self.stage, path)
        w2, h2 = width / 2, width / 8               # 4:1 board
        pts = [(-w2, -h2, 0), (w2, -h2, 0), (w2, h2, 0), (-w2, h2, 0)]
        mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
        mesh.CreateDoubleSidedAttr(True)
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
    def camera_box(self, pos, yaw_deg=0.0, tag="cam", lens_down=True):
        """Black sensor housing with a lens ring, mounted ABOVE/BEHIND the
        actual camera origin so the rendered view is never occluded (near
        clip 0.05 m)."""
        x, y, z = pos
        self.box((x, y, z + 0.075), (0.075, 0.055, 0.045), BLACK, tag=tag,
                 euler_deg=(0, 0, yaw_deg))
        if lens_down:
            self.cyl((x, y, z + 0.022), 0.028, 0.008, (0.02, 0.02, 0.025),
                     tag=f"{tag}_lens")
        self.box((x, y, z + 0.135), (0.012, 0.012, 0.015), FRAME,
                 tag=f"{tag}_mnt")

    # ============================================================== sections
    def rollers(self):
        """Make the DRIVE readable. Belt conveyors read as: dark rubber band
        (the conveyor top itself) + proud end drums + side skirts + yellow
        guards. The transfer table reads as machinery: a roller deck on the
        entry strip and an omni-puck field on the ARB routing zone, both
        proud of the deck by 2 mm (non-colliding — items visually ride them,
        which is exactly the point)."""
        a, b, tb, cb = P.BELT_A, P.BELT_B, P.TABLE, P.CONNECT_B
        r = 0.035
        # belt A: side skirts + end drums peeking beyond the band ends
        for sgn in (-1, 1):
            self.box(((tb["x0"]) / 2, a["y"] + sgn * (a["width"] / 2 + 0.035),
                      a["top"] - 0.09), (tb["x0"] / 2 + 0.02, 0.02, 0.115),
                     FRAME, tag="cheekA")
        for x in (-0.045, tb["x0"] + 0.02):
            self.cyl((x, a["y"], a["top"] - r), r + 0.008,
                     a["width"] / 2 + 0.01, (0.5, 0.52, 0.55), axis="Y",
                     tag="drumA")
        # belt B + connector: side skirts + drums (bands run along Y)
        for nm, cx, y0, y1 in (("B", b["cx"], b["y0"], b["y1"]),
                               ("Cn", cb["cx"], cb["y0"], cb["y1"])):
            for sgn in (-1, 1):
                self.box((cx + sgn * (b["width"] / 2 + 0.035),
                          (y0 + y1) / 2, b["top"] - 0.09),
                         (0.02, (y1 - y0) / 2 + 0.02, 0.115), FRAME,
                         tag=f"cheek{nm}")
            for y in (y0 - 0.045, y1 + 0.045):
                if 0.0 < y < 6.0:
                    self.cyl((cx, y, b["top"] - r), r + 0.008,
                             b["width"] / 2 + 0.01, (0.5, 0.52, 0.55),
                             axis="X", tag=f"drum{nm}")
        # table entry strip: PROUD transport rollers (apex 2 mm above deck)
        for x in np.arange(tb["x0"] + 0.08, tb["route_x"] - 0.04, 0.15):
            self.cyl((float(x), tb["y"], tb["top"] + 0.002 - r), r,
                     tb["width"] / 2 - 0.02, STEEL, axis="Y", tag="rollT")
        # ARB routing zone: omni-puck field (the thing that steers items)
        for x in np.arange(tb["route_x"] + 0.06, tb["x1"] - 0.03, 0.13):
            for y in np.arange(tb["y"] - tb["width"] / 2 + 0.08,
                               tb["y"] + tb["width"] / 2 - 0.05, 0.13):
                self.cyl((float(x), float(y), tb["top"] + 0.0015), 0.032,
                         0.0012, (0.60, 0.62, 0.66), tag="puck")
        # table skirt (machinery housing impression)
        self.box(((tb["x0"] + tb["x1"]) / 2, tb["y"], (tb["top"] - 0.05) / 2),
                 ((tb["x1"] - tb["x0"]) / 2 + 0.03, tb["width"] / 2 + 0.03,
                  (tb["top"] - 0.05) / 2), (0.22, 0.24, 0.28), tag="skirtT")

    def cages_detail(self):
        cages = P.cages_for("table")
        for zone, cage in cages.items():
            cx, cy = cage["center"]
            ix, iy = cage["inner"]
            t, h = cage["wall_t"], cage["wall_h"]
            hx, hy = ix / 2 + t, iy / 2 + t
            col = P.ROUTE_RGBA[zone]
            # vertical tubes along the walls
            for sgn_x in (-1, 1):
                for y in np.arange(-hy + 0.10, hy - 0.05, 0.20):
                    self.cyl((cx + sgn_x * hx, cy + float(y), h / 2 + t),
                             0.012, h / 2, col, tag=f"tube{zone}")
            for sgn_y in (-1, 1):
                for x in np.arange(-hx + 0.10, hx - 0.05, 0.20):
                    self.cyl((cx + float(x), cy + sgn_y * hy, h / 2 + t),
                             0.012, h / 2, col, tag=f"tube{zone}")
            # base frame + casters
            self.box((cx, cy, 0.045), (hx + 0.02, hy + 0.02, 0.014),
                     (0.25, 0.26, 0.30), tag=f"base{zone}")
            for sx in (-1, 1):
                for sy in (-1, 1):
                    self.cyl((cx + sx * (hx - 0.06), cy + sy * (hy - 0.06),
                              0.028), 0.026, 0.014, BLACK, axis="Y",
                             tag=f"caster{zone}")
            # printed label on the visible wall + a tall mast sign. Yaw
            # convention (rotateXYZ(90,0,yaw) on a +Z-facing quad): yaw 0 =
            # text front faces SOUTH (-y), yaw -90 = faces WEST (-x) — the
            # overview camera sits south-west, so fronts point that way
            # (double-sided quads show MIRRORED text from behind).
            txt = "C OVERSIZE" if zone == "C" else "D REPACK"
            fn = f"cage_{zone.lower()}.png"
            if zone == "C":
                self.label(txt, fn, (cx, cy - hy - 0.02, 0.45), 0.85,
                           yaw_deg=0.0)
                self.label(txt, fn, (cx, cy - hy - 0.30, 1.55), 1.05,
                           yaw_deg=0.0)
            else:
                self.label(txt, fn, (cx - hx - 0.02, cy, 0.45), 0.85,
                           yaw_deg=-90.0)
                self.label(txt, fn, (cx - hx - 0.35, cy, 1.55), 1.05,
                           yaw_deg=-90.0)
        # B lane label on the sorter infeed (west face)
        b = P.BELT_B
        self.label("B SORTER", "lane_b.png",
                   (b["cx"] - b["width"] / 2 - 0.06, 5.0, 1.35), 0.95,
                   yaw_deg=-90.0)
        # brand board over the vision station gantry, facing the camera
        self.label("OZON  SORT CELL", "brand.png", (6.0, 3.9, 2.65), 1.6,
                   yaw_deg=0.0)

    def sensors_hw(self):
        vs = P.VIRTUAL_SENSOR
        ox, oy, oz = vs["overhead_pos"]
        # portal gantry across belt A
        for sgn in (-1, 1):
            self.box((ox, oy + sgn * 1.05, 1.25), (0.045, 0.045, 1.25),
                     FRAME, tag="gantry_post")
        self.box((ox, oy, 2.52), (0.05, 1.10, 0.05), FRAME, tag="gantry_beam")
        self.camera_box((ox, oy, oz), tag="cam_overhead")
        # two side profiler heads on stalks, angled inward
        off, zc = vs["side_head_offset_m"], vs["side_head_z_m"]
        for sgn, nm in ((-1, "l"), (1, "r")):
            py = oy + sgn * off
            self.box((ox, py, zc / 2 - 0.03), (0.022, 0.022, zc / 2 - 0.03),
                     FRAME, tag=f"profpost_{nm}")
            self.box((ox, py + sgn * 0.02, zc + 0.055), (0.055, 0.042, 0.038),
                     BLACK, tag=f"profiler_{nm}")
            self.cyl((ox, py - sgn * 0.028, zc + 0.04), 0.02, 0.006,
                     (0.02, 0.02, 0.025), axis="Y", tag=f"proflens_{nm}")
        # jam camera hangs from a ceiling mast that stops ABOVE the lens
        # plane (a pole through z=3.8 sits dead-centre in the camera's view
        # and blinded the jam locator — box_s recovery failed on camera)
        self.box((8.3, 2.6, 4.35), (0.03, 0.03, 0.42), FRAME, tag="jammast")
        self.camera_box((8.3, 2.6, 3.8), tag="cam_jam")

    def lighting_env(self):
        st = self.stage
        # dim the white dome, warm the sun, lift shadows softly
        dome = UsdLux.DomeLight(st.GetPrimAtPath("/World/lights/dome"))
        if dome:
            dome.GetIntensityAttr().Set(180.0)
            dome.GetPrim().CreateAttribute(
                "inputs:color", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(0.55, 0.58, 0.64))
        sun = UsdLux.DistantLight(st.GetPrimAtPath("/World/lights/sun"))
        if sun:
            sun.GetIntensityAttr().Set(900.0)
            sun.GetPrim().CreateAttribute(
                "inputs:angle", Sdf.ValueTypeNames.Float).Set(4.0)
        # high-bay fixtures: sphere lights with radius -> soft shadows
        for i, (lx, ly) in enumerate([(2.5, 3.0), (5.5, 3.0), (8.5, 3.2),
                                      (8.5, 1.2)]):
            lp = f"/World/lights/bay_{i}"
            lgt = UsdLux.SphereLight.Define(st, lp)
            lgt.CreateRadiusAttr(0.22)
            lgt.CreateIntensityAttr(28000.0)
            lgt.GetPrim().CreateAttribute(
                "inputs:color", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(1.0, 0.97, 0.90))
            UsdGeom.Xformable(lgt.GetPrim()).AddTranslateOp().Set(
                Gf.Vec3d(lx, ly, 4.6))
            self.cyl((lx, ly, 4.85), 0.16, 0.05, (0.15, 0.16, 0.18),
                     tag="fixture")
            self.box((lx, ly, 5.15), (0.012, 0.012, 0.25), FRAME,
                     tag="fixture_rod")
        # backdrop walls: kill the white void on the camera-facing sides
        self.box((5.0, 6.35, 2.6), (7.5, 0.06, 2.6), (0.16, 0.18, 0.22),
                 tag="wall_n")
        self.box((10.6, 3.0, 2.6), (0.06, 3.6, 2.6), (0.16, 0.18, 0.22),
                 tag="wall_e")
        # yellow walkway markings on the floor (industrial)
        for y in (0.6, 5.6):
            self.box((4.8, y, 0.003), (4.6, 0.045, 0.001), YELLOW,
                     tag="floorline")
        self.box((9.9, 3.05, 0.003), (0.045, 2.5, 0.001), YELLOW,
                 tag="floorline")

    def dress(self):
        self.rollers()
        self.cages_detail()
        self.sensors_hw()
        self.lighting_env()


def dress_scene(stage):
    Dressing(stage).dress()
