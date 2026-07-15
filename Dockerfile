# Среда воспроизведения для экспертной проверки — цифровой двойник MuJoCo,
# только CPU (GPU не требуется). Полная валидационная матрица запускается
# одной командой и завершается ненулевым кодом при провале любого гейта:
#
#   docker build -t sortmaster-twin .
#   docker run --rm sortmaster-twin                 # 22 прогона + гейты
#
# Одиночный прогон с видео (offscreen-рендер через OSMesa):
#   docker run --rm -v "$PWD/out:/out" sortmaster-twin \
#     python -m cell.run_sim --scenario scenarios/base.yaml --seed 42 \
#       --perception camera --record /out/twin_nominal.mp4 --camera overview
#
# Isaac-контур намеренно вне этого образа: он проверяется официальным
# контейнером nvcr.io/nvidia/isaac-sim:6.0.1 (см. isaac/README.md).
FROM python:3.12-slim

# libgl1/libglib2.0-0 — зависимости opencv; libosmesa6 — offscreen-рендер
# MuJoCo без дисплея и GPU
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1 libglib2.0-0 libosmesa6 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

ENV MUJOCO_GL=osmesa \
    PYTHONUNBUFFERED=1

# полная матрица: 22 прогона x жёсткие гейты, отчет в runs/validation_*/
CMD ["python", "-m", "cell.validate"]
