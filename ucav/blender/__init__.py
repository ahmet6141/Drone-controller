"""YELKOVAN YK-38 — Blender (bpy 4.5) modülleri.

* ``util``      — ağdan nesne, malzeme yuvaları, yumuşak/keskin gölgeleme, koleksiyonlar, ebeveyn ters matrisi,
  pivot ve yerel eksenler (delta dönüşü), EXACT boolean.
* ``airframe``  — gövde, kanat, kuyruk, kumanda yüzeyleri, kaporta/pervane, taret, kuyular/kapaklar, ayrıntılar
  (``build(scene)``).
* ``gear`` (iniş takımı), ``rig`` (U_Root kontrol paneli ve sürücüler), ``animation`` (showcase / mechanisms
  klipleri), ``materials`` (UM_* düğüm ağaçları, boya şemaları, işaretler), ``studio`` (Cycles, pist/stüdyo,
  kameralar), ``render`` (JPG / MP4), ``printprep`` (baskı segmentleri, STL, rapor), ``build`` (tek komutla
  kurulum ve çıktılar — ``python3 ucav/blender/build.py --help``).

İçe aktarma hiçbir iş yapmaz; ``bpy`` yalnızca bu alt paketteki modüllerde gerekir.
"""
from __future__ import annotations

__all__ = ["util", "airframe", "gear", "rig", "materials", "studio", "animation", "render", "printprep",
           "build"]
