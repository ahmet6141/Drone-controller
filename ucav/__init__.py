"""YELKOVAN YK-38 — MALE görünümlü, benzinli itici sivil gözetleme/araştırma İHA gövdesi.

Faydalı yük yalnızca EO/IR kamera taretidir; silah, mühimmat, askı noktası ve bırakma mekanizması yoktur.

Paket düzeni:

* ``spec.yaml`` — tek doğruluk kaynağı (panel tarafından doğrulanmış tasarım sayıları, Türkçe açıklamalı).
* ``params`` — spec'i okur; tipli veri sınıfları ve türetilmiş geometri (gövde kesitleri, kanat/kuyruk kesitleri,
  menteşe hatları, takım pivotları, pervane, taret, malzemeler). Saf Python + numpy.
* ``airfoils`` — UIUC koordinatları (SD7062, SD7032), NACA 4 haneli üretici, ortak parametreli yeniden örnekleme,
  karıştırma, kalınlık ölçekleme ve küt firar kenarı.
* ``sizing`` — boyutlandırma kontrolü (``python3 ucav/sizing.py --check``) → ``ucav/out/sizing.md``.
* ``blender/`` — Blender (bpy 4.5) modelleme, rig, malzeme, render ve baskı hazırlığı modülleri.

İçe aktarma hiçbir iş yapmaz ve ``bpy`` gerektirmez: ``from ucav import params``.
"""
from __future__ import annotations

__version__ = "1.0.0"
__all__ = ["airfoils", "params", "sizing"]
