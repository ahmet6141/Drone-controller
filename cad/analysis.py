"""Mühendislik kontrolleri (CadQuery gerektirmez): titreşim ve dayanım.

1. Gimbal sönümleyicileri: yalıtım frekansı ve hover frekansında titreşim geçirgenliği.
2. Gimbal taşıyıcı kolu (konsol kiriş, uçta kütle): ilk eğilme frekansı motor dönüş (1×) ve
   kanat geçiş (3×) bantlarından uzak olmalı. İki durum: sönümleyiciler çalışırken (kol yalnızca
   kendi kütlesini taşır) ve en kötü durumda (sönümleyici sıkışmış, gimbal kola rijit bağlı).
3. Koruma direkleri: yanal darbe yükünde taban eğilme gerilmesi ve emniyet katsayısı.

Malzeme değerleri tahminidir (params.E_PRINTED, SIGMA_ALLOW); baskı numunesi ve IMU FFT /
vurma testiyle doğrulanmalıdır. Kullanım: python3 cad/analysis.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import gimbal_2axis as G  # noqa: E402  (yalnızca analitik yardımcılar)
import params as P  # noqa: E402

BAND_MARGIN = 0.20            # uyarı bandı: frekans bandının ±%20'si
LATERAL_LOAD_N = 10.0         # koruma halkasına yanal darbe / tutma yükü (4 direğe dağılır, en kötü durumda 2)
MIN_SAFETY = 2.0
MAX_TRANSMISSIBILITY = 0.20


def gimbal_mass_kg() -> float:
    """Sönümleyicilerin taşıdığı kütle: basılan parçalar + 2 motor + STorM32 + IMU (sönümleyiciler hariç)."""
    return (P.GIMBAL_PRINTED_G + 2 * P.GIMBAL_MOTOR_MASS_G + 10.0 + 2.0) / 1000.0


def bands() -> dict[str, tuple[float, float]]:
    lo, hi = P.HOVER_ROT_HZ
    return {"1× dönüş": (lo, hi), f"{P.PROP_BLADES}× kanat geçişi": (P.PROP_BLADES * lo, P.PROP_BLADES * hi)}


def near_band(f: float) -> str | None:
    for name, (lo, hi) in bands().items():
        if lo * (1 - BAND_MARGIN) <= f <= hi * (1 + BAND_MARGIN):
            return name
    return None


def isolator() -> dict[str, float]:
    k = 4 * P.DAMPER_K * 1000.0                      # N/m, 4 sönümleyici paralel
    f_n = math.sqrt(k / gimbal_mass_kg()) / (2 * math.pi)
    def transmissibility(f: float) -> float:
        r = f / f_n
        return 1.0 / abs(r * r - 1.0)
    lo, _ = P.HOVER_ROT_HZ
    return {"f_n": f_n, "T_1x": transmissibility(lo), "T_3x": transmissibility(P.PROP_BLADES * lo)}


def boom(e_scale: float = 1.0) -> dict[str, float]:
    """Konsol kiriş: f = (1/2π)·√(3EI / ((M_uç + 0,236·m_kiriş)·L³))."""
    sec = G.boom_section()
    e = P.E_PRINTED["PA-CF"] * e_scale
    ei = e * sec["I"] * 1e-12                        # mm⁴ → m⁴
    length = sec["L"] / 1000.0
    m_beam = sec["area"] * sec["L"] * 1e-9 * P.DENSITY["PA-CF"] * 1000.0   # mm³ → m³, g/cm³ → kg/m³
    k = 3 * ei / length ** 3
    m_damper_half = 0.003                            # sönümleyicilerin kol tarafı (≈ 3 g)
    f_isolated = math.sqrt(k / (m_damper_half + 0.236 * m_beam)) / (2 * math.pi)
    f_rigid = math.sqrt(k / (gimbal_mass_kg() + 0.236 * m_beam)) / (2 * math.pi)
    return {"L_mm": sec["L"], "I_mm4": sec["I"], "f_isolated": f_isolated, "f_rigid": f_rigid}


def guard_posts() -> dict[str, float]:
    """Direk tabanında eğilme: yük en kötü durumda 2 direğe biner; konik ayak gerilme yığılmasını azaltır."""
    h = (P.PROP_PLANE_Z - P.GUARD_BELOW - P.HUB_T) / 1000.0
    d = P.POST_D / 1000.0
    moment = LATERAL_LOAD_N / 2 * h
    sigma = 32 * moment / (math.pi * d ** 3)
    kt = 1.2 if P.POST_FOOT[0] > 0 else 2.0
    return {"sigma_MPa": sigma / 1e6, "kt": kt, "safety": P.SIGMA_ALLOW["PA-CF"] / (kt * sigma)}


def checks() -> list[tuple[str, bool, str]]:
    iso = isolator()
    b = boom()
    span = [boom(s)["f_rigid"] for s in (0.6, 1.0, 1.5)]
    posts = guard_posts()
    rigid_band = near_band(b["f_rigid"])
    return [
        ("gimbal sönümleyici yalıtımı", iso["T_1x"] <= MAX_TRANSMISSIBILITY,
         f"f_n {iso['f_n']:.0f} Hz → hover'da geçirgenlik %{100 * iso['T_1x']:.0f} (≤ %{100 * MAX_TRANSMISSIBILITY:.0f})"),
        ("taşıyıcı kol (sönümleyici çalışırken)", near_band(b["f_isolated"]) is None,
         f"{b['f_isolated']:.0f} Hz — bantların dışında"),
        ("taşıyıcı kol (en kötü durum: rijit gimbal)", rigid_band is None,
         f"{b['f_rigid']:.0f} Hz (E ±: {span[0]:.0f}–{span[2]:.0f} Hz)"
         + (f" — {rigid_band} bandına yakın" if rigid_band else " — 1× ve 3× bantlarının arasında")),
        ("koruma direği dayanımı", posts["safety"] >= MIN_SAFETY,
         f"σ {posts['sigma_MPa']:.1f} MPa, Kt {posts['kt']:.1f} → emniyet {posts['safety']:.1f} ≥ {MIN_SAFETY:.0f}"),
        ("taşıyıcı kol cıvataları gövde plakasında", G.boom_clamp_x()[1] <= P.FRAME_FRONT_X - P.BOOM_EDGE + 1e-9,
         f"ön sıra x = {G.boom_clamp_x()[1]:.0f} mm, plaka ucu {P.FRAME_FRONT_X:.0f} mm"),
    ]


def rib_sweep(heights=(4.0, 6.0, 8.0, 10.0, 12.0, 14.0)) -> list[tuple[float, float, str | None]]:
    """Kaburga yüksekliğine göre en kötü durum frekansı (tasarım kararı için)."""
    original = P.BOOM_RIB
    out = []
    try:
        for h in heights:
            P.BOOM_RIB = (original[0], h)
            f = boom()["f_rigid"]
            out.append((h, f, near_band(f)))
    finally:
        P.BOOM_RIB = original
    return out


def main() -> int:
    for name, (lo, hi) in bands().items():
        print(f"Hover {name}: {lo:.0f}–{hi:.0f} Hz")
    b = boom()
    print(f"Taşıyıcı kol: serbest boy {b['L_mm']:.1f} mm, kesit atalet momenti {b['I_mm4']:.0f} mm⁴")
    print("Kaburga yüksekliği → en kötü durum frekansı:")
    for h, f, band in rib_sweep():
        mark = "  ← seçili" if abs(h - P.BOOM_RIB[1]) < 1e-9 else ""
        print(f"  {h:4.0f} mm: {f:5.0f} Hz {'(' + band + ' bandına yakın)' if band else '(güvenli pencere)'}{mark}")
    ok = True
    for name, passed, detail in checks():
        ok &= passed
        print(f"  [{'OK ' if passed else 'HATA'}] {name}: {detail}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
