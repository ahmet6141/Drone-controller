#!/usr/bin/env bash
# YK-250 geliştirme ortamı: Python paketleri + başsız (headless) Workbench önizlemesi için Mesa EGL.
# Kullanım: bash ucav250/setup_env.sh   (root ya da sudo gerekir; tekrar çalıştırmak zararsızdır)
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
if ! ldconfig -p 2>/dev/null | grep -q libEGL.so.1; then
  (apt-get install -y -q libegl1 libegl-mesa0 libgl1-mesa-dri libgbm1 >/dev/null 2>&1) || \
  (apt-get update -q >/dev/null 2>&1 && apt-get install -y -q libegl1 libegl-mesa0 libgl1-mesa-dri libgbm1 >/dev/null)
fi
python3 -m pip install -q -r "$HERE/requirements.txt" 2>/dev/null || python3 -m pip install -q -r "$HERE/requirements.txt"
python3 -m pip install -q "numpy==1.26.4"     # neuralfoil/aerosandbox numpy 2 çekebilir; bpy için geri sabitle
python3 - <<'PY'
import numpy, bpy, manifold3d, trimesh, shapely, scipy, neuralfoil  # noqa: F401
print("ucav250 ortamı hazır: numpy", numpy.__version__, "| bpy", bpy.app.version_string)
PY
