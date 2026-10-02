#!/usr/bin/env python3
"""Sensör kataloğu: alternatifler, PX4 / ArduPilot uyumluluk matrisi ve uyumluluk genişliği.

config/sensors/catalog.yaml dosyasını okur; her seçeneğin sürücü parametrelerini ve enum
değerlerini resmi referanslara (tools/data/) göre doğrular, seviye seçimlerini donanım
profilleriyle (config/hardware/) karşılaştırır ve rapor üretir.

Kullanım:
    python3 tools/sensor_matrix.py                    # özet + doğrulama (hata → çıkış kodu 1)
    python3 tools/sensor_matrix.py --markdown         # görev bazında karşılaştırma tabloları
    python3 tools/sensor_matrix.py --role optical_flow
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import validate_params  # noqa: E402
from validate_params import AP_REF, PX4_REF, _load_ref, ap_value_errors, px4_value_errors  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "config" / "sensors" / "catalog.yaml"
HW_DIR = ROOT / "config" / "hardware"

STACKS = ("px4", "ardupilot")
SUPPORT = {
    "px4": ("native", "dronecan", "mavlink", "companion"),
    "ardupilot": ("native", "dronecan", "mavlink", "msp", "companion"),
}
SUPPORT_LABEL = {"native": "yerel", "dronecan": "DroneCAN", "mavlink": "MAVLink", "msp": "MSP",
                 "companion": "companion"}
PORT = "port"
# "port" değeri yalnızca seri port seçen parametrelerde geçerlidir.
PORT_CODES = {"px4": {101, 102}, "ardupilot": {1, 2}}
SERIAL_PROTOCOL_PARAM = "SERIAL1_PROTOCOL"  # tüm SERIALn_PROTOCOL'ler aynı enum'u paylaşır


def load_catalog(path: Path = CATALOG) -> dict:
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def load_refs() -> dict[str, dict]:
    return {"px4": _load_ref(PX4_REF), "ardupilot": _load_ref(AP_REF)}


def param_errors(stack: str, name: str, value, ref: dict) -> list[str]:
    """Katalogdaki tek (ad, değer) çiftini ilgili yığının referansına göre denetler."""
    entry = ref.get(name)
    if entry is None:
        return [f"{name}: {stack} referansında yok"]
    codes = entry[3] if stack == "px4" else entry[2]
    if value == PORT:
        if not codes or not PORT_CODES[stack] <= {int(c) for c in codes}:
            return [f"{name}: seri port seçen bir parametre değil, 'port' kullanılamaz"]
        return []
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return [f"{name}: değer sayı ya da 'port' olmalı ({value!r})"]
    check = px4_value_errors if stack == "px4" else ap_value_errors
    return check(name, float(value), entry)


def _component_names(tier_id: str) -> list[str]:
    with open(HW_DIR / f"{tier_id}.yaml", encoding="utf-8") as fh:
        profile = yaml.safe_load(fh)
    return [c["name"] for c in profile.get("components", [])]


def validate(catalog: dict, refs: dict[str, dict] | None = None, check_tiers: bool = True) -> list[str]:
    refs = refs or load_refs()
    errs: list[str] = []
    roles = catalog.get("roles", {})
    bridges = catalog.get("bridges", {})
    px4_topics = bridges.get("px4_dds", {}).get("topics", {})

    for role_id, role in roles.items():
        bridge = role.get("px4_bridge")
        if bridge is not None and bridge not in px4_topics:
            errs.append(f"rol {role_id}: px4_bridge {bridge!r} bridges.px4_dds.topics içinde yok")
    for name, value in bridges.get("ardupilot_mavlink", {}).get("params", {}).items():
        errs += [f"bridges.ardupilot_mavlink: {e}" for e in param_errors("ardupilot", name, value, refs["ardupilot"])]

    seen: set[str] = set()
    options = catalog.get("options", [])
    for opt in options:
        oid = opt.get("id", "?")
        where = f"seçenek {oid}"
        if oid in seen:
            errs.append(f"{where}: kimlik tekrar ediyor")
        seen.add(oid)
        if not opt.get("roles"):
            errs.append(f"{where}: rol yok")
        for r in opt.get("roles", []):
            if r not in roles:
                errs.append(f"{where}: bilinmeyen rol {r!r}")
        for key in ("mass_g", "price_usd"):
            v = opt.get(key)
            if v is not None and (isinstance(v, bool) or not isinstance(v, (int, float)) or v <= 0):
                errs.append(f"{where}: {key} pozitif sayı ya da null olmalı")
        if not isinstance(opt.get("price_verified"), bool):
            errs.append(f"{where}: price_verified true/false olmalı")
        for stack in STACKS:
            sup = opt.get(stack)
            if not isinstance(sup, dict):
                errs.append(f"{where}: {stack} desteği tanımlı değil")
                continue
            if sup.get("support") not in SUPPORT[stack]:
                errs.append(f"{where}: {stack} desteği {sup.get('support')!r} geçersiz {SUPPORT[stack]}")
            params = sup.get("params") or {}
            if sup.get("support") in ("native", "dronecan", "msp") and not params:
                errs.append(f"{where}: {stack} {sup['support']} desteği için parametre gerekli")
            for name, value in params.items():
                errs += [f"{where}: {e}" for e in param_errors(stack, name, value, refs[stack])]
            if "serial_protocol" in sup:
                errs += [f"{where}: serial_protocol: {e}" for e in param_errors(
                    "ardupilot", SERIAL_PROTOCOL_PARAM, sup["serial_protocol"], refs["ardupilot"])]
        if opt.get("px4", {}).get("support") == "companion":
            for r in opt.get("roles", []):
                role = roles.get(r, {})
                if not role.get("companion_only") and not role.get("px4_bridge"):
                    errs.append(f"{where}: companion desteği var ama {r!r} rolünün PX4 köprüsü yok")

    by_id = {o.get("id"): o for o in options}
    for tier_id, picks in catalog.get("selection", {}).items():
        if check_tiers:
            try:
                names = _component_names(tier_id)
            except FileNotFoundError:
                errs.append(f"seçim {tier_id}: config/hardware/{tier_id}.yaml yok")
                continue
        for role_id, oid in picks.items():
            opt = by_id.get(oid)
            if role_id not in roles:
                errs.append(f"seçim {tier_id}: bilinmeyen rol {role_id!r}")
            if opt is None:
                errs.append(f"seçim {tier_id}.{role_id}: seçenek {oid!r} yok")
                continue
            if role_id not in opt.get("roles", []):
                errs.append(f"seçim {tier_id}.{role_id}: {oid} bu rolü üstlenmiyor")
            if check_tiers:
                match = opt.get("match")
                if not match or not any(match in n for n in names):
                    errs.append(f"seçim {tier_id}.{role_id}: {oid} ({match!r}) donanım profilinde bulunamadı")

    for stack in STACKS:
        for group, spec in catalog.get("breadth", {}).get(stack, {}).items():
            names = [spec["param"]] if isinstance(spec, dict) else spec
            for name in names:
                if name not in refs[stack]:
                    errs.append(f"breadth.{stack}.{group}: {name} referansta yok")
    return errs


def breadth(catalog: dict, refs: dict[str, dict] | None = None) -> dict[str, dict[str, int]]:
    """Referans verisinde, gruplara göre kaç sürücü / tip olduğunu sayar."""
    refs = refs or load_refs()
    out: dict[str, dict[str, int]] = {}
    for stack in STACKS:
        counts = {}
        for group, spec in catalog.get("breadth", {}).get(stack, {}).items():
            if isinstance(spec, dict):
                entry = refs[stack][spec["param"]]
                codes = entry[3] if stack == "px4" else entry[2]
                excluded = {float(c) for c in spec.get("exclude", [])}
                counts[group] = sum(1 for c in codes if float(c) not in excluded)
            else:
                counts[group] = sum(1 for n in spec if n in refs[stack])
        out[stack] = counts
    return out


def _fmt_support(opt: dict, stack: str, catalog: dict) -> str:
    sup = opt[stack]
    kind = sup["support"]
    params = sup.get("params") or {}
    text = SUPPORT_LABEL[kind]
    if kind == "companion" and stack == "px4":
        bridges = catalog["bridges"]["px4_dds"]["topics"]
        topics = sorted({bridges[catalog["roles"][r]["px4_bridge"]] for r in opt["roles"]
                         if catalog["roles"][r].get("px4_bridge")})
        text += " → " + ", ".join(f"`{t}`" for t in topics) if topics else " (yalnızca companion)"
    elif params:
        text += ": " + ", ".join(f"`{k}={v}`" for k, v in params.items())
    elif kind == "companion":
        text += " (yalnızca companion)"
    if stack == "ardupilot" and "serial_protocol" in sup:
        text += f", `SERIALn_PROTOCOL={sup['serial_protocol']}`"
    return text


def _fmt_price(opt: dict) -> str:
    if opt.get("price_usd") is None:
        return "—"
    return f"${opt['price_usd']:g}" + ("" if opt.get("price_verified") else " (doğrulanmadı)")


def render_markdown(catalog: dict, role_filter: str | None = None) -> str:
    out = []
    selections = catalog.get("selection", {})
    for role_id, role in catalog["roles"].items():
        if role_filter and role_id != role_filter:
            continue
        opts = [o for o in catalog["options"] if role_id in o["roles"]]
        if not opts:
            continue
        out += [f"### {role['title']}", "",
                "| Sensör | Arayüz | Özellik | PX4 v1.17 | ArduPilot 4.7.1 | Kütle | Fiyat | Seviye |",
                "|---|---|---|---|---|---:|---:|---|"]
        for o in opts:
            tiers = [t.split("-")[1].upper() for t, picks in selections.items() if picks.get(role_id) == o["id"]]
            mass = "—" if o.get("mass_g") is None else f"{o['mass_g']:g} g"
            out.append(f"| {o['name']} | {o['interface']} | {o.get('specs', '')} | {_fmt_support(o, 'px4', catalog)} "
                       f"| {_fmt_support(o, 'ardupilot', catalog)} | {mass} | {_fmt_price(o)} | {', '.join(tiers) or '—'} |")
        out.append("")
    return "\n".join(out).rstrip() + "\n"


def render_summary(catalog: dict, counts: dict[str, dict[str, int]], errors: list[str]) -> str:
    lines = [f"Sensör kataloğu: {len(catalog['options'])} seçenek, {len(catalog['roles'])} görev"]
    for role_id, role in catalog["roles"].items():
        opts = [o for o in catalog["options"] if role_id in o["roles"]]
        cheapest = min((o for o in opts if o.get("price_usd")), key=lambda o: o["price_usd"], default=None)
        lines.append(f"  - {role['title']:<38}: {len(opts)} seçenek"
                     + (f", en ucuz {cheapest['name']} (${cheapest['price_usd']:g})" if cheapest else ""))
    lines.append("Uyumluluk genişliği (resmi referanslardan sayılan sürücü / tip):")
    for stack in STACKS:
        lines.append(f"  {stack:<10}: " + ", ".join(f"{g} {n}" for g, n in counts[stack].items()))
    topics = catalog["bridges"]["px4_dds"]["topics"].values()
    lines.append("  companion köprüsü (PX4 DDS girişleri): " + ", ".join(topics))
    lines.append(f"Doğrulama: {len(errors)} hata")
    lines += [f"  HATA: {e}" for e in errors]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--catalog", type=Path, default=CATALOG)
    ap.add_argument("--markdown", action="store_true", help="görev bazında Markdown tabloları")
    ap.add_argument("--role", help="yalnızca bu görev (ör. optical_flow)")
    args = ap.parse_args(argv)

    catalog = load_catalog(args.catalog)
    refs = load_refs()
    errors = validate(catalog, refs)
    if args.role and args.role not in catalog["roles"]:
        ap.error(f"bilinmeyen görev {args.role!r}; seçenekler: {', '.join(catalog['roles'])}")
    if args.markdown or args.role:
        print(render_markdown(catalog, args.role))
        for e in errors:
            print("HATA:", e, file=sys.stderr)
    else:
        print(render_summary(catalog, breadth(catalog, refs), errors))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
