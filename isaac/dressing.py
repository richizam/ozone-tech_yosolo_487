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
YELLOW = (0.58, 0.48, 0.13)   # worn industrial yellow
BLACK = (0.045, 0.045, 0.05)
OZON_BLUE = (0.0, 0.357, 1.0)                      # #005BFF
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

    def _vis(self, prim, color, roughness=0.72, metallic=0.12):
        """Matte industrial PBR (bare displayColor renders as toy plastic).
        Runtime-tinted prims (route lamps/trails/LEDs) must NOT bind —
        a bound material overrides displayColor updates."""
        seed = hash(prim.GetPath().pathString) % 3
        jit = (0.90, 1.0, 1.08)[seed]
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
            opacity=None, bind=True):
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
            self._vis(cube.GetPrim(), color)
        return cube.GetPrim()

    def cyl(self, center, radius, half_h, color, axis="Z", tag="cyl",
            bind=True):
        c = UsdGeom.Cylinder.Define(self.stage, self._path(tag))
        c.CreateRadiusAttr(float(radius))
        c.CreateHeightAttr(float(2 * half_h))
        c.CreateAxisAttr(axis)
        UsdGeom.Xformable(c.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(*[float(v) for v in center]))
        c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if bind:
            self._vis(c.GetPrim(), color)
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
              tilt_deg=90.0):
        """Textured label quad (UsdPreviewSurface + UsdUVTexture)."""
        tex = self._label_texture(text, fname, bg=bg)
        return self._quad_with_texture(tex, center, width, yaw_deg, tilt_deg)

    def _quad_with_texture(self, tex, center, width, yaw_deg=0.0,
                           tilt_deg=90.0):
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
        if getattr(self, "shells", False):
            return                                  # official assets carry the look
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
            col = tuple(0.60 * v + 0.14 for v in P.ROUTE_RGBA[zone])
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
            # wall-level printed label only — the tall mast boards were
            # oversized signalization over the C/D boxes (user directive:
            # keep the destinations readable, not billboarded)
            txt = "C OVERSIZE" if zone == "C" else "D REPACK"
            fn = f"cage_{zone.lower()}.png"
            self.label(txt, fn, (cx, cy - hy - 0.02, 0.45), 0.85,
                       yaw_deg=0.0)
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
        # industrial sensing detail (§6): cable tray along the gantry beam,
        # cable drops to each head, small status LEDs on the housings
        self.box((ox, oy, 2.585), (0.05, 1.10, 0.012), (0.42, 0.44, 0.47),
                 tag="cabletray")
        for hy, hz in ((oy, oz + 0.12), (oy - off, zc + 0.09),
                       (oy + off, zc + 0.09)):
            self.cyl((ox + 0.04, hy, (2.57 + hz) / 2), 0.004,
                     (2.57 - hz) / 2, (0.06, 0.06, 0.07), tag="camcable")
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
        dome = UsdLux.DomeLight(st.GetPrimAtPath("/World/lights/dome"))
        if dome:
            dome.GetIntensityAttr().Set(150.0)
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
        for (lx, ly, w, d) in ((1.6, 3.0, 2.4, 3.0), (4.2, 3.0, 2.6, 3.0),
                               (6.6, 3.0, 2.4, 3.2), (8.7, 2.6, 2.6, 3.4)):
            self._highbay(lx, ly, w, d, 18000.0)
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

    # ------------------------------------------------------ conveyor details
    def conveyor_details(self):
        """Hazard striping, plinth + legs cladding, and white direction
        chevrons painted on the belts (the visible roller/flow direction)."""
        a, b, tb, cb = P.BELT_A, P.BELT_B, P.TABLE, P.CONNECT_B
        # black dashes over the yellow side guides -> yellow/black safety
        # edge. ONLY when the official shells are absent: the shells carry
        # their own side rails, and doubled rail hardware crowds the freight
        # corridor on camera (items read as clipping through the stripes)
        if not getattr(self, "shells", False):
            for x in np.arange(0.3, 6.5, 0.45):
                for sgn in (-1, 1):
                    self.box((float(x), a["y"] + sgn * (a["width"] / 2 + 0.015),
                              a["top"] + 0.041), (0.11, 0.017, 0.051), BLACK,
                             tag="hzA")
        # plinth band + proud legs on belt A and belt B (reads as supports)
        if getattr(self, "shells", False):
            for x in []:
                pass
        legs_needed = not getattr(self, "shells", False)
        for x in (np.arange(0.6, tb["x0"] - 0.2, 1.2) if legs_needed else []):
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
        for x in np.arange(0.6, tb["x0"] - 0.15, 0.55):
            for j, sw in ((0, 40.0), (1, -40.0)):
                self.box((float(x) - 0.03 * j, a["y"] + (0.05 if j else -0.05),
                          a["top"] + 0.0005), (0.075, 0.012, 0.0004), wht,
                         tag="dirA", euler_deg=(0, 0, sw))
        for nm, cx, y0, y1 in (("B", b["cx"], b["y0"] + 0.3, b["y1"] - 0.2),
                               ("Cn", cb["cx"], cb["y0"] + 0.1, cb["y1"] - 0.05)):
            for y in np.arange(y0, y1, 0.5):
                for j, sw in ((0, 50.0), (1, 130.0)):
                    self.box((cx + (0.05 if j else -0.05), float(y),
                              b["top"] + 0.0005), (0.075, 0.012, 0.0004), wht,
                             tag=f"dir{nm}", euler_deg=(0, 0, sw))

    # ------------------------------------------------------- route visuals
    def _chevron(self, center, yaw_deg, color, size=0.075, z_thick=0.0005,
                 tag="chev", bind=True):
        """V-shaped arrowhead from two rotated bars; points along yaw
        (0 deg = +x)."""
        paths = []
        for sw in (140.0, -140.0):
            wa = math.radians(yaw_deg + sw)
            c = (center[0] + size * 0.55 * math.cos(wa),
                 center[1] + size * 0.55 * math.sin(wa), center[2])
            p = self.box(c, (size, 0.016, z_thick), color, tag=tag,
                         euler_deg=(0, 0, yaw_deg + sw), bind=bind)
            paths.append(p.GetPath().pathString)
        return paths

    def route_viz(self):
        """Everything that answers 'where is THIS item going': zone arrows on
        the routing deck, chevron trails to each container, and the ACTIVE
        ROUTE indicator panel. Returns prim paths for runtime brightness."""
        tb, cb, b = P.TABLE, P.CONNECT_B, P.BELT_B
        cc, cd = P.CHUTE_C, P.CHUTE_D
        viz = {"zone_arrows": {}, "trails": {}, "lamps": {}}
        # name the mechanism on the hardware: the deck IS an ARB sorter
        self.label("ARB SORTER DECK", "arb_deck.png",
                   (8.25, tb["y"] - tb["width"] / 2 - 0.02, 0.52), 0.62,
                   yaw_deg=0.0)
        dim = {z: tuple(0.35 * v for v in P.ROUTE_RGBA[z]) for z in "BCD"}
        # NO painted arrows on the deck (user directive: the deck is real
        # hardware — angled roller modules with per-module status LEDs; the
        # LEDs and the ACTIVE ROUTE panel carry the state story instead)
        # chevron trails: routing table -> container
        trails = {"B": [], "C": [], "D": []}
        for y in np.arange(cb["y0"] + 0.12, b["y1"] - 0.3, 0.42):
            trails["B"] += self._chevron((cb["cx"], float(y), b["top"] + 0.004),
                                         90.0, dim["B"], tag="trB", bind=False)
        ang_c = math.degrees(math.atan2(cc["z0"] - cc["z1"], cc["x1"] - cc["x0"]))
        for x in np.arange(cc["x0"] + 0.12, cc["x1"] - 0.05, 0.28):
            zc = cc["z0"] - (float(x) - cc["x0"]) * math.tan(math.radians(ang_c)) + 0.006
            for sw in (140.0, -140.0):
                wa = math.radians(sw)
                c = (float(x) + 0.06 * 0.55 * math.cos(wa), cc["cy"]
                     + 0.06 * 0.55 * math.sin(wa), zc)
                p = self.box(c, (0.07, 0.015, 0.0005), dim["C"], tag="trC",
                             euler_deg=(0, -ang_c, sw), bind=False)
                trails["C"].append(p.GetPath().pathString)
        ang_d = math.degrees(math.atan2(cd["z0"] - cd["z1"], cd["y0"] - cd["y1"]))
        for y in np.arange(cd["y0"] - 0.12, cd["y1"] + 0.05, -0.28):
            zc = cd["z0"] - (cd["y0"] - float(y)) * math.tan(math.radians(ang_d)) + 0.006
            for sw in (140.0, -140.0):
                wa = math.radians(-90.0 + sw)
                c = (cd["cx"] + 0.06 * 0.55 * math.cos(wa),
                     float(y) + 0.06 * 0.55 * math.sin(wa), zc)
                p = self.box(c, (0.07, 0.015, 0.0005), dim["D"], tag="trD",
                             euler_deg=(ang_d, 0, -90.0 + sw), bind=False)
                trails["D"].append(p.GetPath().pathString)
        viz["trails"] = trails
        # ACTIVE ROUTE indicator panel by the table (mast + 3 lamps)
        px, py = 7.95, 4.15
        self.box((px, py, 0.95), (0.025, 0.025, 0.95), FRAME, tag="armast")
        self.label("ACTIVE ROUTE", "active_route.png", (px, py - 0.03, 2.05),
                   0.85, yaw_deg=0.0)
        for i, z in enumerate("BCD"):
            lp = self.box((px - 0.26 + 0.26 * i, py - 0.03, 1.72),
                          (0.09, 0.02, 0.09), dim[z], tag=f"lamp{z}",
                          bind=False)
            viz["lamps"][z] = lp.GetPath().pathString
            self.label(z, f"lampcap_{z}.png",
                       (px - 0.26 + 0.26 * i, py - 0.035, 1.52), 0.17,
                       yaw_deg=0.0, bg=tuple(0.55 * v for v in P.ROUTE_RGBA[z]))
        # floating per-item route flags: textures made here, quads at runtime
        flags = {
            "B": str(self._label_texture(">> B SORTER", "flag_b.png",
                                         bg=OZON_BLUE)),
            "C": str(self._label_texture(">> C OVERSIZE", "flag_c.png",
                                         bg=(0.80, 0.42, 0.08))),
            "D": str(self._label_texture(">> D REPACK", "flag_d.png",
                                         bg=(0.10, 0.55, 0.22))),
        }
        viz["flag_textures"] = flags
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
            self._textured_quad("ozon_wall", str(SIGN_DIR / "ozon_wall.png"),
                                (5.0, 6.27, 3.4), 2.6, yaw_deg=0.0)
        except Exception as exc:
            print(f"[dressing] logo board fallback ({exc})", flush=True)
            self.label("ozon", "ozon_wall_txt.png", (5.0, 6.27, 3.4), 2.6,
                       yaw_deg=0.0)
        self.box((5.0, 6.26, 2.94), (1.3, 0.012, 0.035), MAGENTA,
                 tag="brand_accent")
        # blue band along the table skirt + magenta kick strip
        tb = P.TABLE
        self.box(((tb["x0"] + tb["x1"]) / 2, tb["y"] - tb["width"] / 2 - 0.036,
                  0.50), ((tb["x1"] - tb["x0"]) / 2 + 0.03, 0.006, 0.045),
                 OZON_BLUE, tag="ozon_band")
        self.box(((tb["x0"] + tb["x1"]) / 2, tb["y"] - tb["width"] / 2 - 0.036,
                  0.42), ((tb["x1"] - tb["x0"]) / 2 + 0.03, 0.006, 0.014),
                 MAGENTA, tag="ozon_kick")
        # blue crossbeam accent on the vision gantry
        vs = P.VIRTUAL_SENSOR
        self.box((vs["overhead_pos"][0], vs["overhead_pos"][1], 2.46),
                 (0.052, 1.10, 0.012), OZON_BLUE, tag="gantry_accent")

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
        self.sensors_hw()
        self.sensor_assets()
        self.lighting_env()
        self.ozon_brand()
        viz = self.route_viz()
        viz["arb_pills"] = self.arb_pills
        viz["arb_rollers"] = self.arb_rollers
        return viz


def dress_scene(stage):
    return Dressing(stage).dress()


class RouteVizRuntime:
    """Per-tick route storytelling (visuals only): floating route flags that
    follow each classified item, the ACTIVE ROUTE lamp panel, the routing-deck
    arrows and the chevron trails all brighten for the commanded route."""

    def __init__(self, stage, viz):
        self.stage = stage
        self.viz = viz or {}
        self.bright = {z: Gf.Vec3f(*P.ROUTE_RGBA[z]) for z in "BCD"}
        self.dim = {z: Gf.Vec3f(*[0.30 * v for v in P.ROUTE_RGBA[z]])
                    for z in "BCD"}
        self._color_attrs = {"lamps": {}, "zone_arrows": {}, "trails": {}}
        for z, p in self.viz.get("lamps", {}).items():
            self._color_attrs["lamps"][z] = [self._attr(p)]
        for z, ps in self.viz.get("zone_arrows", {}).items():
            self._color_attrs["zone_arrows"][z] = [self._attr(p) for p in ps]
        for z, ps in self.viz.get("trails", {}).items():
            self._color_attrs["trails"][z] = [self._attr(p) for p in ps]
        self._flags = {}                 # slug -> (translate_op, route)
        self._flag_i = 0
        self._last_route = "?"

    def _attr(self, path):
        return UsdGeom.Gprim(self.stage.GetPrimAtPath(path)).GetDisplayColorAttr()

    # --------------------------------------------------------------- flags
    def make_flag(self, slug, route):
        """Small floating billboard «>> B SORTER» that follows the item."""
        tex = self.viz.get("flag_textures", {}).get(route)
        if tex is None or slug in self._flags:
            return
        self._flag_i += 1
        path = f"{ROOT}/flags/flag_{self._flag_i}"
        mesh = UsdGeom.Mesh.Define(self.stage, path)
        w2, h2 = 0.34, 0.085
        mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(-w2, 0, -h2),
                                             Gf.Vec3f(w2, 0, -h2),
                                             Gf.Vec3f(w2, 0, h2),
                                             Gf.Vec3f(-w2, 0, h2)]))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
        mesh.CreateDoubleSidedAttr(True)
        st_pv = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        st_pv.Set(Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0),
                                 Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]))
        mpath = f"{path}_mat"
        mat = UsdShade.Material.Define(self.stage, mpath)
        sh = UsdShade.Shader.Define(self.stage, f"{mpath}/pbr")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
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
        tr = UsdGeom.Xformable(mesh.GetPrim()).AddTranslateOp()
        tr.Set(Gf.Vec3d(0, 0, -5))
        self._flags[slug] = tr

    def drop_flag(self, slug):
        tr = self._flags.get(slug)
        if tr is not None:
            tr.Set(Gf.Vec3d(0, 0, -5))

    # -------------------------------------------------------------- update
    def update(self, active_route, item_positions):
        """active_route: commanded zone route or None; item_positions:
        {slug: (x, y, top_z)} for items that should carry their flag."""
        if active_route != self._last_route:
            self._last_route = active_route
            for group in ("lamps", "zone_arrows", "trails"):
                for z, attrs in self._color_attrs[group].items():
                    col = self.bright[z] if z == active_route else self.dim[z]
                    for at in attrs:
                        at.Set([col])
        for slug, (x, y, top_z) in item_positions.items():
            tr = self._flags.get(slug)
            if tr is not None:
                tr.Set(Gf.Vec3d(float(x), float(y) - 0.02, float(top_z) + 0.17))
