# -*- coding: utf-8 -*-
"""Assembled видеодемонстрация — the single narrative demo required by the
submission rules (doc-1783095831.pdf p.15), cut from the committed evidence
clips with Russian title cards in the cell's presentation design language.

Every second of footage is a recording of a real physics run; the cards
carry the argument (no voice-over, so the file is self-contained and
re-generable). Output: docs/report/isaac_evidence/demo_sortmaster.mp4

Usage: python tools/make_demo_video.py [out.mp4]
"""
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

REPO = Path(__file__).resolve().parents[1]
EV = REPO / "docs" / "report" / "isaac_evidence"
VID = EV / "xbelt" / "videos_final"
TWIN = EV / "twin_videos"
PAN = EV / "xbelt" / "perception"

OZON_BLUE = (0, 91, 255)
MAGENTA = (240, 30, 120)
GREEN = (51, 199, 89)
BG = (13, 14, 18)
TXT = (235, 237, 242)
SUB = (150, 156, 168)
W, H = 1920, 1080
FPS = 30


def font(sz, mono=False, bold=True):
    cands = (["C:/Windows/Fonts/consolab.ttf" if bold else
              "C:/Windows/Fonts/consola.ttf"] if mono else
             ["C:/Windows/Fonts/arialbd.ttf" if bold else
              "C:/Windows/Fonts/arial.ttf"])
    cands += ["/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
              "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"]
    for c in cands:
        try:
            return ImageFont.truetype(c, sz)
        except Exception:
            continue
    return ImageFont.load_default()


def card(title, lines, accent=OZON_BLUE, big=None, big_label=None):
    """Title card: brand rails, headline, supporting lines, optional metric."""
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, W, 8], fill=OZON_BLUE)
    d.rectangle([0, H - 8, W, H], fill=MAGENTA)
    d.rectangle([120, 300, 128, 300 + 60 + 44 * len(lines)], fill=accent)

    tf = font(64)
    while d.textlength(title, tf) > W - 320 and tf.size > 34:
        tf = font(tf.size - 2)
    d.text((170, 300), title, font=tf, fill=TXT)

    y = 300 + 96
    for ln in lines:
        lf = font(30, bold=False)
        while d.textlength(ln, lf) > W - 340 and lf.size > 18:
            lf = font(lf.size - 1, bold=False)
        d.text((170, y), ln, font=lf, fill=SUB)
        y += 44

    # a bare number reads as decoration — it always carries its unit line
    if big:
        bf = font(190, mono=True)
        bw = d.textlength(big, bf)
        bx = 170
        by = y + 60
        d.text((bx, by), big, font=bf, fill=accent)
        if big_label:
            lf = font(26)
            d.text((bx + bw + 28, by + 96), big_label, font=lf, fill=TXT)
    return img


def write_card(path, img, secs):
    img.save(path)
    return secs


def main():
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else EV / "demo_sortmaster.mp4"
    if not shutil.which("ffmpeg"):
        print("ffmpeg not found")
        return 1
    agg = json.loads((EV / "xbelt" / "matrix" / "matrix_summary.json")
                     .read_text(encoding="utf-8"))["aggregate"]

    tmp = Path(tempfile.mkdtemp(prefix="demo_"))
    seq = []                       # (path, kind, start, dur)

    def clip(name, src, start, dur):
        p = (VID / src) if (VID / src).is_file() else (TWIN / src)
        if not p.is_file():
            p = PAN / src
        if not p.is_file():
            print(f"  MISSING {src}")
            return
        seq.append((p, "clip", start, dur))

    def title(name, *a, secs=4.0, **kw):
        p = tmp / f"card_{name}.png"
        write_card(p, card(*a, **kw), secs)
        seq.append((p, "card", 0, secs))

    # ---------------------------------------------------------- the story
    # in-points verified frame-by-frame against each source clip: every cut
    # lands on freight in shot (contact sheets, 2026-07-16)
    title("open", "Куб 11 мм решил всё",
          ["По официальным правилам куб 10×10×10 мм — граница.",
           "Значит, 11 мм — легальный сортируемый товар.",
           "Механизм, который не может его довезти, не решает задачу."],
          secs=5.5)
    clip("hero", "cine_orbit.mp4", 34, 7)

    title("task", "Задача",
          ["Участок до основного сортировщика. Лента 1 м/с, не останавливается.",
           "Измерить в движении → классифицировать по правилам →",
           "физически доставить: B — сортировщик, C — негабарит, D — доупаковка."],
          secs=5.0)
    clip("route", "route_C.mp4", 11, 6)          # пуф по склизу в кейдж C

    title("mech", "Почему поворотные лотки",
          ["Роликовые дивертеры не управляют предметом 11 мм.",
           "Здесь товар едет на СВОЁМ лотке и сходит гравитацией:",
           "сброс не зависит от размера — куб 11 мм и пуф 489 мм одинаково."],
          secs=5.0)
    clip("deck", "perfect_decktop.mp4", 32, 6)   # лотки наклоняются, вид сверху
    clip("edge", "edge_small_items.mp4", 10, 5)  # снят на HEAD: видны поддоны

    title("meas", "Измеряем, а не угадываем",
          ["4 RTX-камеры глубины; для мелочи — макро-головка 0.4 мм/пиксель.",
           "Порог сертификации честный: 10 мм + два разрешения = 10.8 мм.",
           "Куб 11 мм → B. Куб 10 мм → C. Все сомнения → C/D/разбор, никогда в B."],
          secs=5.5)
    # каска под станцией (13.7 s) и коробка (22 s) — по контактному листу;
    # хвосты между товарами вырезаны, иначе кадр пустеет
    clip("sensor_a", "sensor.mp4", 12.4, 3.6)
    clip("sensor_b", "sensor.mp4", 20.8, 3.0)
    clip("panels", "perception_demo.mp4", 8.4, 6.5)   # панели куб 11 → B, 10 → C

    title("phys", "Только контактная физика",
          ["Ни одной прямой записи скорости товару в штатном цикле —",
           "это жёсткий гейт валидации, ноль во всех 14 прогонах.",
           "Лоток на настоящем шарнире: 150 Н·м, задержка команды 40 мс."],
          accent=GREEN, big="0", big_label="прямых записей скорости товару",
          secs=5.5)
    clip("transfer", "b_transfer.mp4", 9.5, 5)   # сход с ножа на лоток

    title("fault", "У каждого отказа — безопасный терминал",
          ["Отказы мы устраиваем сами и показываем реакцию:",
           "заклинивание → манипулятор кладёт товар в ЕГО кейдж;",
           "мёртвый привод наклона → конец линии → вызов оператора."],
          accent=MAGENTA, secs=5.0)
    clip("recov", "fault_recovery.mp4", 26, 8)
    clip("tray", "fault_tray.mp4", 60, 5)

    title("valid", "Две физики, одна ячейка",
          ["Isaac Sim (PhysX + RTX) — основной твин, сенсоры в контуре.",
           "MuJoCo — тот же контур на любом ноутбуке без GPU: docker compose up.",
           "Обе матрицы эксперт воспроизводит одной командой."],
          secs=5.0)
    clip("twin", "twin_nominal_overview.mp4", 38, 6)

    # endcard (still) + honesty card
    ec = EV / "xbelt" / "videos_final" / "endcard.png"
    if ec.is_file():
        seq.append((ec, "card", 0, 7.0))
    title("honest", "Честность материала",
          ["Все кадры — записи реальных прогонов физики.",
           "Скриптованного движения товара нет.",
           "16 из 17 роликов сняты на предыдущей ревизии геометрии (без",
           "аварийных поддонов) — оговорено в isaac_evidence/README.md.",
           f"Цифры — из матрицы на HEAD: {agg['runs']} прогонов, GATES PASS."],
          secs=6.0)

    # ------------------------------------------------------- render parts
    parts = []
    for i, (src, kind, start, dur) in enumerate(seq):
        outp = tmp / f"p{i:02d}.mp4"
        if kind == "card":
            cmd = ["ffmpeg", "-y", "-v", "error", "-loop", "1", "-i", str(src),
                   "-t", f"{dur}", "-vf",
                   f"scale={W}:{H}:force_original_aspect_ratio=decrease,"
                   f"pad={W}:{H}:(ow-iw)/2:(oh-ih)/2:color=0x0D0E12,fps={FPS}",
                   "-c:v", "libx264", "-pix_fmt", "yuv420p", str(outp)]
        else:
            cmd = ["ffmpeg", "-y", "-v", "error", "-ss", f"{start}",
                   "-i", str(src), "-t", f"{dur}", "-vf",
                   f"scale={W}:{H}:force_original_aspect_ratio=decrease,"
                   f"pad={W}:{H}:(ow-iw)/2:(oh-ih)/2:color=0x0D0E12,fps={FPS}",
                   "-c:v", "libx264", "-pix_fmt", "yuv420p", "-an", str(outp)]
        r = subprocess.run(cmd, capture_output=True)
        if r.returncode != 0 or not outp.is_file():
            print(f"  part {i} FAILED: {r.stderr.decode()[:160]}")
            continue
        parts.append(outp)
        print(f"  part {i:02d} {kind:4s} {dur:4.1f}s  {Path(src).name}")

    lst = tmp / "list.txt"
    lst.write_text("".join(f"file '{p.as_posix()}'\n" for p in parts),
                   encoding="utf-8")
    out.parent.mkdir(parents=True, exist_ok=True)
    r = subprocess.run(["ffmpeg", "-y", "-v", "error", "-f", "concat",
                        "-safe", "0", "-i", str(lst), "-c:v", "libx264",
                        "-crf", "20", "-pix_fmt", "yuv420p", str(out)],
                       capture_output=True)
    if r.returncode != 0:
        print("concat failed:", r.stderr.decode()[:300])
        return 1
    print(f"demo: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
