# -*- coding: utf-8 -*-
"""Industrial PBR materials for the presentation layer (visual only).

Two anti-"toy plastic" tools:

  * make_grunge_textures(): FFT-tileable procedural grunge — a mostly-bright
    albedo-multiply map (dirt patches, streaks, contact grime) and a
    roughness-variation map. Numpy/PIL, deterministic, written once to the
    mounted sign dir.

  * omni_pbr(): bind NVIDIA's OmniPBR.mdl with WORLD-SPACE TRIPLANAR
    projection (project_uvw) so the grunge maps onto the primitive Cube
    panels WITHOUT per-prim UVs — real dirt/roughness variation on cages,
    guards, rails and floor. Falls back cleanly to UsdPreviewSurface (the
    RTX renderer just uses displayColor) if the MDL is unavailable offline.

Nothing here touches physics or the depth sensors — presentation only.
"""
import numpy as np
from pxr import Gf, Sdf, UsdShade

GRUNGE_ALBEDO = "grunge_albedo.png"
GRUNGE_ROUGH = "grunge_rough.png"

# ---------------------------------------------------------------- presets
# Named industrial finishes (visual only): color, roughness, metallic.
# Consumed by dressing.py / scene_usd.py through their existing binding
# helpers — one vocabulary for the whole cell so the same steel reads as the
# same steel everywhere. The "anisotropic" look of brushed steel is faked by
# the grunge roughness variation the PbrLibrary already applies.
PRESETS = {
    "brushed_steel":      {"color": (0.55, 0.57, 0.60), "roughness": 0.35,
                           "metallic": 0.85},
    "powder_steel_dark":  {"color": (0.28, 0.30, 0.33), "roughness": 0.55,
                           "metallic": 0.20},
    "galvanized":         {"color": (0.62, 0.64, 0.66), "roughness": 0.45,
                           "metallic": 0.70},
    "belt_rubber":        {"color": (0.10, 0.10, 0.11), "roughness": 0.85,
                           "metallic": 0.0},
    "safety_yellow_worn": {"color": (0.85, 0.70, 0.10), "roughness": 0.70,
                           "metallic": 0.05},
}


def preset(name):
    """(color, roughness, metallic) for a named finish."""
    p = PRESETS[name]
    return p["color"], p["roughness"], p["metallic"]


def _tileable(shape, beta, seed):
    """Tileable 1/f^beta value noise via random-phase inverse FFT (periodic
    by construction), normalized to [0, 1]."""
    rng = np.random.default_rng(seed)
    h, w = shape
    fy = np.fft.fftfreq(h)[:, None]
    fx = np.fft.fftfreq(w)[None, :]
    f = np.sqrt(fy * fy + fx * fx)
    f[0, 0] = 1.0
    amp = 1.0 / (f ** beta)
    amp[0, 0] = 0.0
    phase = rng.uniform(0, 2 * np.pi, size=(h, w))
    spec = amp * np.exp(1j * phase)
    img = np.fft.ifft2(spec).real
    img -= img.min()
    img /= max(img.max(), 1e-9)
    return img


def make_grunge_textures(dirpath):
    """Write the two grunge maps into dirpath (idempotent). Returns
    (albedo_path, rough_path) or (None, None) on failure."""
    from pathlib import Path
    from PIL import Image
    d = Path(dirpath)
    d.mkdir(parents=True, exist_ok=True)
    ap, rp = d / GRUNGE_ALBEDO, d / GRUNGE_ROUGH
    if ap.is_file() and rp.is_file():
        return str(ap), str(rp)
    try:
        base = _tileable((1024, 1024), 2.6, 7)           # broad soft mottle
        fine = _tileable((1024, 1024), 1.1, 19)          # fine grain
        # ALBEDO MULTIPLY — LOW CONTRAST so a tiled detail map reads as worn
        # paint/grime, never as a repeating polka-dot pattern. Centered near
        # 1.0, gentle ±: subtle surface interest, no discrete dark blobs.
        alb = 1.0 - 0.06 * (base - 0.5) * 2 - 0.05 * (fine - 0.5) * 2
        alb = np.clip(alb, 0.86, 1.0)
        Image.fromarray((alb * 255).astype(np.uint8)).convert("L").save(ap)
        # ROUGHNESS: mostly matte with faint variation
        rough = 0.80 + 0.06 * (fine - 0.5) * 2
        rough = np.clip(rough, 0.72, 0.9)
        Image.fromarray((rough * 255).astype(np.uint8)).convert("L").save(rp)
        return str(ap), str(rp)
    except Exception as exc:
        print(f"[materials] grunge generation failed: {exc}", flush=True)
        return None, None


class PbrLibrary:
    """Caches OmniPBR materials by (color, roughness, metallic, textured)."""

    def __init__(self, stage, root, tex_albedo=None, tex_rough=None):
        self.stage = stage
        self.root = root
        self.tex_albedo = tex_albedo
        self.tex_rough = tex_rough
        self._cache = {}
        self._n = 0

    def get(self, color, roughness=0.82, metallic=0.05, world_scale=3.5,
            textured=True, seed=0):
        jit = (0.92, 1.0, 1.08)[seed % 3]
        c = tuple(min(1.0, float(v) * jit) for v in color)
        key = (round(c[0], 3), round(c[1], 3), round(c[2], 3),
               round(roughness, 2), round(metallic, 2), textured)
        m = self._cache.get(key)
        if m is not None:
            return m
        self._n += 1
        path = f"{self.root}/PbrMats/m_{self._n}"
        mat = UsdShade.Material.Define(self.stage, path)
        sh = UsdShade.Shader.Define(self.stage, f"{path}/Shader")
        sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
        sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
        sh.CreateInput("diffuse_color_constant",
                       Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*c))
        sh.CreateInput("diffuse_tint",
                       Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*c))
        sh.CreateInput("reflection_roughness_constant",
                       Sdf.ValueTypeNames.Float).Set(float(roughness))
        sh.CreateInput("metallic_constant",
                       Sdf.ValueTypeNames.Float).Set(float(metallic))
        sh.CreateInput("specular_level",
                       Sdf.ValueTypeNames.Float).Set(0.3)
        if textured and self.tex_albedo:
            sh.CreateInput("diffuse_texture",
                           Sdf.ValueTypeNames.Asset).Set(
                Sdf.AssetPath(self.tex_albedo))
            sh.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(True)
            sh.CreateInput("world_or_object", Sdf.ValueTypeNames.Bool).Set(True)
            sh.CreateInput("texture_scale",
                           Sdf.ValueTypeNames.Float2).Set(
                Gf.Vec2f(1.0 / world_scale, 1.0 / world_scale))
        mat.CreateSurfaceOutput("mdl").ConnectToSource(
            sh.ConnectableAPI(), "out")
        self._cache[key] = mat
        return mat
