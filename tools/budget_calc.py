#!/usr/bin/env python3
"""Ağırlık / itki / güç / uçuş süresi bütçe hesaplayıcısı.

config/hardware/*.yaml donanım profillerini okur ve her profil için:
  * toplam kalkış ağırlığı (AUW) ve kategori bazında kütle dağılımı,
  * maksimum itki ve batarya akım sınırı dahil gerçek itki/ağırlık (T/W) oranı,
  * hover itkisi, hover gücü, hover akımı ve tahmini hover süresi,
  * PX4 MPC_THR_HOVER / ArduPilot MOT_THST_HOVER için başlangıç değeri
hesaplar ve profildeki `limits` bölümüne göre kontrol eder.

Kullanım:
    python3 tools/budget_calc.py config/hardware/tier-b-pro.yaml
    python3 tools/budget_calc.py --all
    python3 tools/budget_calc.py --all --markdown
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
HW_DIR = ROOT / "config" / "hardware"

# Yük altında hücre gerilimi / nominal gerilim oranı (yüksek akımda gerilim çökmesi).
LOADED_VOLTAGE_RATIO = 0.90
# Karma uçuşta (manevra + rüzgâr) hover süresine uygulanan düşüş katsayısı.
MIXED_FLIGHT_FACTOR = 0.85
# Ölçülen ilk noktanın altında itki-güç ilişkisi: momentum teorisi P ∝ T^1.5.
LOW_THRUST_EXPONENT = 1.5


@dataclass
class Check:
    name: str
    ok: bool
    detail: str


@dataclass
class Budget:
    profile_id: str
    name: str
    auw_g: float
    battery_g: float
    mass_by_category: dict[str, float]
    max_thrust_g: float
    battery_limited_thrust_g: float
    tw_ratio: float
    tw_ratio_battery_limited: float
    hover_thrust_per_motor_g: float
    hover_throttle: float
    hover_efficiency_g_per_w: float
    propulsion_hover_w: float
    avionics_w: float
    total_hover_w: float
    hover_current_a: float
    usable_wh: float
    hover_time_min: float
    mixed_flight_time_min: float
    payload_margin_g: float
    thr_mdl_fac: float | None = None
    airframe_cost_usd: float = 0.0
    ground_cost_usd: float = 0.0
    unpriced: list[str] = field(default_factory=list)
    checks: list[Check] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return all(c.ok for c in self.checks)


def _curve(points: list[list[float]]) -> list[tuple[float, float]]:
    pts = sorted((float(t), float(p)) for t, p in points)
    if len(pts) < 2:
        raise ValueError("thrust_curve en az 2 nokta içermeli")
    for (t0, p0), (t1, p1) in zip(pts, pts[1:]):
        if t1 <= t0 or p1 <= p0:
            raise ValueError("thrust_curve itki ve güçte kesin artan olmalı")
    return pts


def _segment_exponent(t0: float, p0: float, t1: float, p1: float) -> float:
    """İki ölçüm noktası arasında P = p0·(T/t0)^k üssü (log-log doğrusal)."""
    return math.log(p1 / p0) / math.log(t1 / t0)


def power_at_thrust(curve: list[tuple[float, float]], thrust_g: float) -> float:
    """Motor başına itki (g) için elektrik gücü (W).

    Noktalar arası log-log (güç yasası) enterpolasyon: pervane gücü itkinin ~1.4–1.8. kuvvetiyle
    arttığından doğrusal enterpolasyondan daha gerçekçidir. İlk noktanın altında k = 1.5.
    """
    if thrust_g <= 0:
        return 0.0
    if thrust_g > curve[-1][0]:
        raise ValueError(
            f"{thrust_g:.0f} g itki, ölçülen eğrinin üstünde (maks {curve[-1][0]:.0f} g)"
        )
    if thrust_g <= curve[0][0]:
        t0, p0 = curve[0]
        return p0 * (thrust_g / t0) ** LOW_THRUST_EXPONENT
    for (t0, p0), (t1, p1) in zip(curve, curve[1:]):
        if t0 <= thrust_g <= t1:
            return p0 * (thrust_g / t0) ** _segment_exponent(t0, p0, t1, p1)
    raise AssertionError("ulaşılamaz")


def thrust_at_power(curve: list[tuple[float, float]], power_w: float) -> float:
    """Motor başına güç (W) için itki (g); `power_at_thrust`'ın tersi, eğri maksimumunda sınırlı."""
    if power_w <= 0:
        return 0.0
    if power_w >= curve[-1][1]:
        return curve[-1][0]
    if power_w <= curve[0][1]:
        t0, p0 = curve[0]
        return t0 * (power_w / p0) ** (1.0 / LOW_THRUST_EXPONENT)
    for (t0, p0), (t1, p1) in zip(curve, curve[1:]):
        if p0 <= power_w <= p1:
            return t0 * (power_w / p0) ** (1.0 / _segment_exponent(t0, p0, t1, p1))
    raise AssertionError("ulaşılamaz")


def fit_thr_mdl_fac(throttle_points: list[list[float]]) -> float:
    """PX4 THR_MDL_FAC tahmini (en küçük kareler).

    PX4 itki modeli: rel_itki = f·u² + (1 − f)·u  (u: normalize gaz, rel_itki: itki / maks itki).
    y = rel_itki − u ve x = u² − u dönüşümüyle y = f·x olur → f = Σxy / Σx².
    `throttle_points`: [[gaz_oranı 0–1, itki_g], ...]; u = 1 noktası zorunludur.
    """
    pts = sorted((float(u), float(t)) for u, t in throttle_points)
    if not pts or abs(pts[-1][0] - 1.0) > 1e-6:
        raise ValueError("throttle_points %100 gaz (u = 1.0) noktasını içermeli")
    t_max = pts[-1][1]
    sxy = sxx = 0.0
    for u, t in pts:
        x = u * u - u
        sxy += x * (t / t_max - u)
        sxx += x * x
    if sxx == 0.0:
        raise ValueError("THR_MDL_FAC için 0 < u < 1 aralığında en az bir nokta gerekli")
    return min(max(sxy / sxx, 0.0), 1.0)


def compute(profile: dict) -> Budget:
    meta = profile["profile"]
    prop = profile["propulsion"]
    bat = profile["battery"]
    limits = profile.get("limits", {})

    n = int(prop["motor_count"])
    curve = _curve(prop["thrust_curve"]["points"])

    mass_by_cat: dict[str, float] = {}
    avionics_w = 0.0
    cost = 0.0
    unpriced: list[str] = []

    def add_cost(item: dict, qty: float) -> float:
        if item.get("price_usd") is None:
            unpriced.append(item.get("name", "?"))
            return 0.0
        return float(item["price_usd"]) * qty

    for comp in profile.get("components", []):
        qty = comp.get("qty", 1)
        cat = comp.get("category", "diğer")
        mass_by_cat[cat] = mass_by_cat.get(cat, 0.0) + comp["mass_g"] * qty
        avionics_w += comp.get("power_w", 0.0) * qty
        cost += add_cost(comp, qty)
    cost += add_cost(prop["motor"], n) + add_cost(prop["prop"], n) + add_cost(prop["esc"], 1)
    cost += add_cost(bat, 1)
    ground = sum(add_cost(g, g.get("qty", 1)) for g in profile.get("ground_equipment", []))

    propulsion_g = n * (prop["motor"]["mass_g"] + prop["prop"]["mass_g"]) + prop["esc"]["mass_g"]
    mass_by_cat["itki"] = mass_by_cat.get("itki", 0.0) + propulsion_g
    mass_by_cat["batarya"] = float(bat["mass_g"])
    auw = sum(mass_by_cat.values())

    nominal_v = bat["cells_series"] * bat["nominal_cell_v"]
    usable_wh = nominal_v * bat["capacity_mah"] / 1000.0 * bat["usable_fraction"]

    # Tezgâh verisi serbest pervane içindir; gövde gölgelemesi ve koruma/ağ itkiyi düşürür.
    # Motor, uçuşta gereken itkinin 1/k katını "tezgâh eşdeğeri" olarak üretmelidir.
    k_inst = float(prop.get("installation_factor", 1.0))
    if not 0.5 <= k_inst <= 1.0:
        raise ValueError("installation_factor 0.5–1.0 aralığında olmalı")

    hover_per_motor = auw / n
    p_motor = power_at_thrust(curve, hover_per_motor / k_inst)
    propulsion_hover_w = n * p_motor
    total_hover_w = propulsion_hover_w + avionics_w

    max_thrust = n * curve[-1][0] * k_inst
    # Bataryanın sürekli akım sınırında motorlara kalan güç → ulaşılabilir itki
    p_bat_max = bat["max_continuous_a"] * nominal_v * LOADED_VOLTAGE_RATIO
    p_motor_lim = (p_bat_max - avionics_w) / n
    thrust_lim = n * thrust_at_power(curve, p_motor_lim) * k_inst

    min_tw = limits.get("min_tw_ratio", 2.0)
    budget = Budget(
        profile_id=meta["id"],
        name=meta["name"],
        auw_g=auw,
        battery_g=float(bat["mass_g"]),
        mass_by_category=mass_by_cat,
        max_thrust_g=max_thrust,
        battery_limited_thrust_g=thrust_lim,
        tw_ratio=max_thrust / auw,
        tw_ratio_battery_limited=thrust_lim / auw,
        hover_thrust_per_motor_g=hover_per_motor,
        hover_throttle=auw / max_thrust,
        hover_efficiency_g_per_w=hover_per_motor / p_motor,
        propulsion_hover_w=propulsion_hover_w,
        avionics_w=avionics_w,
        total_hover_w=total_hover_w,
        hover_current_a=total_hover_w / nominal_v,
        usable_wh=usable_wh,
        hover_time_min=usable_wh / total_hover_w * 60.0,
        mixed_flight_time_min=usable_wh / total_hover_w * 60.0 * MIXED_FLIGHT_FACTOR,
        payload_margin_g=thrust_lim / min_tw - auw,
        airframe_cost_usd=cost,
        ground_cost_usd=ground,
        unpriced=unpriced,
    )
    if prop["thrust_curve"].get("throttle_points"):
        budget.thr_mdl_fac = fit_thr_mdl_fac(prop["thrust_curve"]["throttle_points"])

    if "max_auw_g" in limits:
        budget.checks.append(Check(
            "AUW", auw <= limits["max_auw_g"],
            f"{auw:.0f} g ≤ {limits['max_auw_g']} g"))
    budget.checks.append(Check(
        "T/W (batarya sınırlı)", budget.tw_ratio_battery_limited >= min_tw,
        f"{budget.tw_ratio_battery_limited:.2f} ≥ {min_tw}"))
    if "min_hover_time_min" in limits:
        budget.checks.append(Check(
            "Hover süresi", budget.hover_time_min >= limits["min_hover_time_min"],
            f"{budget.hover_time_min:.1f} dk ≥ {limits['min_hover_time_min']} dk"))
    if "max_hover_throttle" in limits:
        budget.checks.append(Check(
            "Hover gazı", budget.hover_throttle <= limits["max_hover_throttle"],
            f"{budget.hover_throttle:.2f} ≤ {limits['max_hover_throttle']}"))
    return budget


def load(path: Path) -> dict:
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def all_profiles() -> list[Path]:
    return sorted(HW_DIR.glob("*.yaml"))


def render_text(b: Budget) -> str:
    lines = [
        f"== {b.name} ({b.profile_id}) ==",
        f"  AUW                     : {b.auw_g:7.0f} g  (batarya {b.battery_g:.0f} g)",
    ]
    for cat, m in sorted(b.mass_by_category.items(), key=lambda kv: -kv[1]):
        lines.append(f"    - {cat:<20}: {m:7.0f} g  (%{100 * m / b.auw_g:4.1f})")
    lines += [
        f"  Maks itki (toplam)      : {b.max_thrust_g:7.0f} g   T/W {b.tw_ratio:.2f}",
        f"  Batarya sınırlı itki    : {b.battery_limited_thrust_g:7.0f} g   T/W {b.tw_ratio_battery_limited:.2f}",
        f"  Hover itkisi / motor    : {b.hover_thrust_per_motor_g:7.0f} g   verim {b.hover_efficiency_g_per_w:.2f} g/W",
        f"  Hover gazı (normalize)  : {b.hover_throttle:7.2f}     → MPC_THR_HOVER / MOT_THST_HOVER başlangıcı",
        f"  Hover gücü              : {b.total_hover_w:7.0f} W  (itki {b.propulsion_hover_w:.0f} W + aviyonik {b.avionics_w:.0f} W)",
        f"  Hover akımı             : {b.hover_current_a:7.1f} A",
        f"  Kullanılabilir enerji   : {b.usable_wh:7.1f} Wh",
        f"  Hover süresi            : {b.hover_time_min:7.1f} dk   (karma uçuş ≈ {b.mixed_flight_time_min:.1f} dk)",
        f"  Ek yük payı (min T/W'de): {b.payload_margin_g:7.0f} g",
    ]
    if b.thr_mdl_fac is not None:
        lines.append(f"  THR_MDL_FAC (tahmini)   : {b.thr_mdl_fac:7.2f}     → PX4 itki modeli doğrusallaştırma")
    lines.append(f"  Tahmini maliyet         : hava aracı ${b.airframe_cost_usd:,.0f} + yer ekipmanı ${b.ground_cost_usd:,.0f}"
                 + (f"  (fiyatsız: {', '.join(b.unpriced)})" if b.unpriced else ""))
    for c in b.checks:
        lines.append(f"  [{'OK ' if c.ok else 'HATA'}] {c.name}: {c.detail}")
    return "\n".join(lines)


def render_markdown(budgets: list[Budget]) -> str:
    rows = [
        ("AUW (g)", lambda b: f"{b.auw_g:.0f}"),
        ("Batarya (g)", lambda b: f"{b.battery_g:.0f}"),
        ("Maks itki (g)", lambda b: f"{b.max_thrust_g:.0f}"),
        ("T/W (motor)", lambda b: f"{b.tw_ratio:.2f}"),
        ("T/W (batarya sınırlı)", lambda b: f"{b.tw_ratio_battery_limited:.2f}"),
        ("Hover itkisi/motor (g)", lambda b: f"{b.hover_thrust_per_motor_g:.0f}"),
        ("Hover verimi (g/W)", lambda b: f"{b.hover_efficiency_g_per_w:.2f}"),
        ("Hover gazı (MPC_THR_HOVER)", lambda b: f"{b.hover_throttle:.2f}"),
        ("Hover gücü (W)", lambda b: f"{b.total_hover_w:.0f}"),
        ("Hover akımı (A)", lambda b: f"{b.hover_current_a:.1f}"),
        ("Hover süresi (dk)", lambda b: f"{b.hover_time_min:.1f}"),
        ("Karma uçuş (dk)", lambda b: f"{b.mixed_flight_time_min:.1f}"),
        ("Ek yük payı (g)", lambda b: f"{b.payload_margin_g:.0f}"),
        ("THR_MDL_FAC (tahmini)", lambda b: "—" if b.thr_mdl_fac is None else f"{b.thr_mdl_fac:.2f}"),
        ("Maliyet: hava aracı (USD, ≈)", lambda b: f"{b.airframe_cost_usd:,.0f}"),
        ("Maliyet: yer ekipmanı (USD, ≈)", lambda b: f"{b.ground_cost_usd:,.0f}"),
        ("Limit kontrolleri", lambda b: "✅" if b.ok else "⚠️ " + ", ".join(
            c.name for c in b.checks if not c.ok)),
    ]
    header = "| Metrik | " + " | ".join(b.profile_id for b in budgets) + " |"
    sep = "|---|" + "---:|" * len(budgets)
    body = ["| " + label + " | " + " | ".join(fn(b) for b in budgets) + " |" for label, fn in rows]
    return "\n".join([header, sep, *body])


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("profiles", nargs="*", type=Path, help="donanım profili YAML dosyaları")
    ap.add_argument("--all", action="store_true", help="config/hardware altındaki tüm profiller")
    ap.add_argument("--markdown", action="store_true", help="Markdown karşılaştırma tablosu üret")
    args = ap.parse_args(argv)

    paths = all_profiles() if args.all else args.profiles
    if not paths:
        ap.error("en az bir profil verin ya da --all kullanın")

    budgets = [compute(load(p)) for p in paths]
    if args.markdown:
        print(render_markdown(budgets))
    else:
        print("\n\n".join(render_text(b) for b in budgets))
    return 0 if all(b.ok for b in budgets) else 1


if __name__ == "__main__":
    sys.exit(main())
