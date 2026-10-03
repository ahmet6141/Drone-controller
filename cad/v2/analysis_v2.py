#!/usr/bin/env python3
"""DC7 v2 mühendislik kontrolleri (CadQuery gerektirmez): kol–kanal modülünün titreşimi ve dayanımı.

Kol, gövde soketinde ankastre, değişken kesitli bir konsol kiriştir (U kesit, altı açık). Ucunda
motor + pervane + motor yuvası; kanal halkası ve ızgara da kolla birlikte salınır. Izgara düzlem
dışında esnek olduğu için halka + ızgaranın ne kadarının katıldığı belirsizdir: katılım oranı p,
halkanın kanal–kol köprüsü etrafında rijit döndüğü üst sınıra (p = 1) göre 0…1 aralığında taranır.
Yöntem: Euler–Bernoulli kirişi, Stodola (Rayleigh) iterasyonu ile ilk eğilme frekansı (saf Python).

Hedef: frekans aralığının tamamı motor dönüş (1×) bandının ve kanat geçiş (3×) bandının ±%20
uyarı paylarının ARASINDA kalsın (v1'deki gimbal taşıyıcı kolu ile aynı ölçüt, cad/analysis.py).
Değerler tahminidir; ilk numunelerde vurma testi + IMU FFT ile doğrulanır (docs/12 §6).
Kullanım: python3 cad/v2/analysis_v2.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parents[1] / "tools"))

import analysis as A1  # noqa: E402  (bantlar ve uyarı payı v1 ile ortak)
import budget_calc  # noqa: E402
import params as P  # noqa: E402
import v2_params as V  # noqa: E402

MATERIAL = "PA6-GF30"
E_CONDITIONED = 0.7            # nem almış PA6 (%50 RH): kuru modülün ≈ %70'i
SIGMA_ALLOW = {"PA6-GF30": 90e6}   # Pa, nemli durum çekme dayanımı (kuru ≈ 150 MPa), kaba
DROP_G = 10.0                  # kanal kenarına düşme / çarpma: tüm kütlenin 10 g ivmesi tek kanala (kaba)
MIN_SAFETY = 2.0
PARTICIPATION = (0.0, 0.5, 1.0)
POD_MASS_G = 13.2              # motor yuvası (CAD: airframe.motor_pod(); tests/test_v2.py ±%10 ile doğrular)


def u_section(w: float, h: float, t: float) -> dict[str, float]:
    """Altı açık U kesit (mm): üst flanş w×t + iki yan duvar t×(h−t). Köşe yuvarlatmaları ihmal (muhafazakâr)."""
    af, zf = w * t, h - t / 2
    aw, zw = 2 * t * (h - t), (h - t) / 2
    a = af + aw
    zb = (af * zf + aw * zw) / a
    iy = w * t ** 3 / 12 + af * (zf - zb) ** 2 + 2 * t * (h - t) ** 3 / 12 + aw * (zw - zb) ** 2
    iz = t * w ** 3 / 12 + 2 * ((h - t) * t ** 3 / 12 + t * (h - t) * (w / 2 - t / 2) ** 2)
    return {"A": a, "Iy": iy, "Iz": iz, "c": max(zb, h - zb)}


def free_length() -> float:
    """Soket ağzından (gövde yüzeyi) kolun motor yuvasına birleştiği yere (mm)."""
    return (P.MOTOR_R - V.ARM_ROOT_R) - V.MOTOR_POD[0] + 3.0


def section(s: float, root=None, tip=None) -> dict[str, float]:
    """Soketten s mm uzakta kesit: ARM_PRISM boyunca sabit, sonra uca doğrusal incelme."""
    (w0, h0), (w1, h1) = root or V.ARM_ROOT, tip or V.ARM_TIP
    L = free_length()
    u = 0.0 if s <= V.ARM_PRISM else min((s - V.ARM_PRISM) / (L - V.ARM_PRISM), 1.0)
    return u_section(w0 + u * (w1 - w0), h0 + u * (h1 - h0), V.ARM_WALL)


def duct_masses_g() -> dict[str, float]:
    """Kanal halkası ve ızgaranın analitik kütleleri (g)."""
    rho = V.DENSITY[MATERIAL]
    z0, z1 = V.DUCT_Z
    r_mid = V.DUCT_IN_R + V.DUCT_WALL / 2
    ring = 2 * math.pi * r_mid * V.DUCT_WALL * (z1 - z0) * 1.08 * rho / 1000      # +%8 çan ağzı ve kenar
    g = V.GRILLE
    fill = 1.0 - (g["cell"] / (g["cell"] + g["rib"])) ** 2
    c_out = P.MOTOR_BELL_D / 2 + P.COLLAR_GAP + P.COLLAR_W
    disc = math.pi * (V.DUCT_IN_R ** 2 - c_out ** 2) * g["t"] * fill
    spokes = 3 * (V.DUCT_IN_R - c_out) * V.SPOKE[0] * (V.SPOKE[1] - g["t"])
    c_in = P.MOTOR_BELL_D / 2 + P.COLLAR_GAP
    hub = math.pi * (c_out ** 2 - c_in ** 2) * P.SPOKE_T + 3 * math.pi * (P.POST_D / 2) ** 2 * (z0 - P.HUB_T)
    return {"ring": ring, "grille": (disc + spokes) * rho / 1000, "hub": hub * rho / 1000}


def tip_mass_g(p: float, profile: dict | None = None) -> float:
    """Motor eksenindeki eşdeğer kütle: motor + pervane + yuva + ızgara göbeği (bilezik ve direkler)
    + p × (halka ve ızgaranın rijit dönme payı)."""
    pr = profile or budget_calc.load(budget_calc.HW_DIR / "variants" / "tier-a-entegre.yaml")
    prop = pr["propulsion"]
    d = duct_masses_g()
    # Halka köprü noktası etrafında dönerse: halka noktaları 0…2δ hareket eder → <(1+cos φ)²> = 1,5;
    # ızgara (disk) için <(1 + (r/R)cos φ)²> = 1,25
    rigid = 1.5 * d["ring"] + 1.25 * d["grille"]
    return prop["motor"]["mass_g"] + prop["prop"]["mass_g"] + POD_MASS_G + d["hub"] + p * rigid


def frequency(p: float = 0.5, e_scale: float = 1.0, axis: str = "Iy", root=None, tip=None, n: int = 60) -> float:
    """İlk eğilme frekansı (Hz): kök ankastre, motor yuvası rijit (kol ucundan motor eksenine)."""
    L = free_length()
    off = V.MOTOR_POD[0] - 3.0
    xs = [L * i / n for i in range(n + 1)] + [L + off * i / 10 for i in range(1, 11)]
    e = V.E_MOLDED[MATERIAL] * e_scale
    rho = V.DENSITY[MATERIAL] * 1000.0                                   # kg/m³
    EI, m = [], []
    for i, x in enumerate(xs):
        sec = section(min(x, L), root, tip)
        EI.append(e * sec[axis] * 1e-12 * (1.0 if x <= L else 50.0))
        dx = (xs[min(i + 1, len(xs) - 1)] - xs[max(i - 1, 0)]) / 2
        m.append(rho * sec["A"] * 1e-6 * dx * 1e-3 * 1.06 if x <= L else 0.0)   # +%6 kaburgalar
    m[-1] += tip_mass_g(p) / 1000.0
    y = [(x / xs[-1]) ** 2 for x in xs]
    w2 = 0.0
    for _ in range(25):
        F = [mi * yi for mi, yi in zip(m, y)]
        M = [sum(F[j] * (xs[j] - xs[i]) * 1e-3 for j in range(i + 1, len(xs))) for i in range(len(xs))]
        k = [Mi / EIi for Mi, EIi in zip(M, EI)]
        th, yn = [0.0], [0.0]
        for i in range(1, len(xs)):
            dx = (xs[i] - xs[i - 1]) * 1e-3
            th.append(th[-1] + (k[i] + k[i - 1]) / 2 * dx)
            yn.append(yn[-1] + (th[i] + th[i - 1]) / 2 * dx)
        w2 = sum(mi * yi * yi for mi, yi in zip(m, y)) / sum(mi * yi * yni for mi, yi, yni in zip(m, y, yn))
        y = [v / yn[-1] for v in yn]
    return math.sqrt(w2) / (2 * math.pi)


def window() -> tuple[float, float]:
    """Güvenli pencere: 1× bandının üst payı ile 3× bandının alt payı arası (Hz)."""
    b = list(A1.bands().values())
    return b[0][1] * (1 + A1.BAND_MARGIN), b[1][0] * (1 - A1.BAND_MARGIN)


def vertical_range(root=None, tip=None) -> tuple[float, float]:
    fs = [frequency(p, e, "Iy", root, tip) for p in PARTICIPATION for e in (E_CONDITIONED, 1.0)]
    return min(fs), max(fs)


def drop_safety(profile: dict | None = None) -> dict[str, float]:
    """Kanal dış kenarına düşme: F = AUW × DROP_G, soket ağzında eğilme gerilmesi (kaba)."""
    pr = profile or budget_calc.load(budget_calc.HW_DIR / "variants" / "tier-a-entegre.yaml")
    auw = budget_calc.compute(pr).auw_g / 1000.0
    force = auw * 9.81 * DROP_G
    arm = (P.MOTOR_R - V.ARM_ROOT_R) + V.DUCT_IN_R + V.DUCT_WALL + V.DUCT_FLARE[1]     # mm
    sec = section(0.0)
    sigma = force * arm * sec["c"] / sec["Iy"] * 1e6                       # N·mm·mm/mm⁴ = MPa → Pa
    return {"force_N": force, "sigma_MPa": sigma / 1e6, "safety": SIGMA_ALLOW[MATERIAL] / sigma}


def section_sweep(heights=(24.0, 28.0, 34.0, 37.0)) -> list[tuple[float, float, float, bool]]:
    """Kök yüksekliğine göre dikey frekans aralığı (uç yüksekliği oranla) — tasarım kararı için."""
    lo, hi = window()
    out = []
    for h in heights:
        tip = (V.ARM_TIP[0], V.ARM_TIP[1] * h / V.ARM_ROOT[1])
        fmin, fmax = vertical_range((V.ARM_ROOT[0], h), tip)
        out.append((h, fmin, fmax, lo <= fmin and fmax <= hi))
    return out


def checks() -> list[tuple[str, bool, str]]:
    lo, hi = window()
    fmin, fmax = vertical_range()
    lat = frequency(0.5, E_CONDITIONED, "Iz")
    drop = drop_safety()
    return [
        ("kol dikey eğilme frekansı bantların arasında", lo <= fmin and fmax <= hi,
         f"{fmin:.0f}–{fmax:.0f} Hz (katılım 0–1, E %{100 * E_CONDITIONED:.0f}–100); pencere {lo:.0f}–{hi:.0f} Hz"),
        ("kol yanal eğilme frekansı 1× bandının üstünde", lat >= lo,
         f"{lat:.0f} Hz ≥ {lo:.0f} (p = 0,5, nemli; ızgara düzlem içinde rijit olduğundan gerçekte daha yüksek)"),
        ("kanal kenarına düşme (kol kökü)", drop["safety"] >= MIN_SAFETY,
         f"F {drop['force_N']:.0f} N → σ {drop['sigma_MPa']:.0f} MPa, emniyet {drop['safety']:.1f} ≥ {MIN_SAFETY:.0f}"),
    ]


def main() -> int:
    lo, hi = window()
    d = duct_masses_g()
    print(f"Kol: serbest boy {free_length():.0f} mm, kök {V.ARM_ROOT[0]:g}×{V.ARM_ROOT[1]:g} → uç "
          f"{V.ARM_TIP[0]:g}×{V.ARM_TIP[1]:g} mm, et {V.ARM_WALL} mm ({MATERIAL})")
    print(f"Uç kütlesi: {tip_mass_g(0):.0f} g (+ halka {d['ring']:.1f} g ve ızgara {d['grille']:.1f} g'ın katılan payı "
          f"→ p = 1'de {tip_mass_g(1):.0f} g)")
    print(f"Güvenli pencere (1× üst payı … 3× alt payı): {lo:.0f}–{hi:.0f} Hz")
    print("Kök yüksekliği → dikey frekans aralığı:")
    for h, fmin, fmax, ok in section_sweep():
        mark = "  ← seçili" if abs(h - V.ARM_ROOT[1]) < 1e-9 else ""
        print(f"  {h:4.0f} mm: {fmin:4.0f}–{fmax:4.0f} Hz {'(pencerede)' if ok else '(bant payına giriyor)'}{mark}")
    ok = True
    for name, passed, detail in checks():
        ok &= passed
        print(f"  [{'OK ' if passed else 'HATA'}] {name}: {detail}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
