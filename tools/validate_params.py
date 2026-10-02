#!/usr/bin/env python3
"""PX4 ve ArduPilot parametre dosyalarını resmi parametre referansına göre doğrular.

Referanslar (tools/data/):
  * px4_v1.17_params.json            — docs.px4.io/v1.17 parametre referansından
  * ardupilot_copter_4.7.1_params.json — ardupilot.org Copter 4.7.1 referansından

Kontroller: ad var mı, tip (PX4: INT32 ↔ 6, FLOAT ↔ 9), min/maks aralığı, enum ve bitmask değerleri.

Kullanım:
    python3 tools/validate_params.py              # config/ altındaki tüm dosyalar
    python3 tools/validate_params.py dosya.params dosya.param
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATA = Path(__file__).resolve().parent / "data"
PX4_REF = DATA / "px4_v1.17_params.json"
AP_REF = DATA / "ardupilot_copter_4.7.1_params.json"
PARAM_NAME = re.compile(r"^[A-Z][A-Z0-9_]{0,15}$")  # her iki yığında da en fazla 16 karakter
PX4_TYPE_CODES = {"INT32": "6", "FLOAT": "9"}         # MAV_PARAM_TYPE_INT32 / _REAL32


def _load_ref(path: Path) -> dict:
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)["params"]


def parse_px4(path: Path) -> list[tuple[int, str, str, str]]:
    """QGroundControl parametre dosyası: 'sistem<TAB>bileşen<TAB>AD<TAB>değer<TAB>tip'."""
    rows = []
    for lineno, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        fields = raw.split("\t")
        if len(fields) != 5:
            raise ValueError(f"{path}:{lineno}: 5 TAB ayrılmış alan bekleniyordu")
        _, _, name, value, ptype = fields
        rows.append((lineno, name, value, ptype))
    return rows


def parse_ardupilot(path: Path) -> list[tuple[int, str, str]]:
    """Mission Planner / MAVProxy formatı: 'AD,değer' ('#' sonrası yorum)."""
    rows = []
    for lineno, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        name, sep, value = line.partition(",")
        if not sep:
            raise ValueError(f"{path}:{lineno}: 'AD,değer' bekleniyordu")
        rows.append((lineno, name.strip(), value.strip()))
    return rows


def _check_value(name: str, value: float, lo, hi, codes, bitmask: bool, is_int: bool) -> list[str]:
    errs = []
    if is_int and value != int(value):
        errs.append(f"{name}: tam sayı bekleniyordu ({value})")
    if bitmask:
        if value < 0 or value != int(value):
            errs.append(f"{name}: bitmask negatif olmayan tam sayı olmalı")
        elif codes and int(value) >= (1 << (max(codes) + 1)):
            errs.append(f"{name}: bitmask {int(value)} tanımlı bitlerin dışında (en yüksek bit {max(codes)})")
    elif codes and lo is None and hi is None and value not in codes:
        errs.append(f"{name}: {value:g} izinli değerlerden biri değil {codes}")
    if lo is not None and value < lo:
        errs.append(f"{name}: {value:g} < min {lo:g}")
    if hi is not None and value > hi:
        errs.append(f"{name}: {value:g} > maks {hi:g}")
    return errs


def px4_value_errors(name: str, value: float, entry: list) -> list[str]:
    """Tek PX4 değerini referans girdisine ([tip, min, maks, kodlar, bitmask]) göre denetler."""
    ptype, lo, hi, codes, bitmask = entry
    is_int = ptype == "INT32"
    errs = []
    if is_int and codes and not bitmask:
        # PX4'te INT32 enum'lar katıdır (aralık verilmiş olsa bile)
        if value != int(value) or int(value) not in codes:
            errs.append(f"{name}={value:g} izinli değerlerden biri değil {codes}")
        codes = None
    return errs + _check_value(name, value, lo, hi, codes, bitmask, is_int)


def ap_value_errors(name: str, value: float, entry: list) -> list[str]:
    """Tek ArduPilot değerini referans girdisine ([min, maks, kodlar, bitmask]) göre denetler."""
    lo, hi, codes, bitmask = entry
    return _check_value(name, value, lo, hi, codes, bitmask, False)


def validate_px4(path: Path, ref: dict | None = None) -> list[str]:
    ref = ref or _load_ref(PX4_REF)
    errs, seen = [], set()
    for lineno, name, value, ptype in parse_px4(path):
        where = f"{path.name}:{lineno}"
        if not PARAM_NAME.match(name):
            errs.append(f"{where}: geçersiz ad {name!r}")
            continue
        if name in seen:
            errs.append(f"{where}: {name} iki kez tanımlı")
        seen.add(name)
        if name not in ref:
            errs.append(f"{where}: {name} PX4 v1.17'de yok")
            continue
        ptype_ref = ref[name][0]
        if PX4_TYPE_CODES.get(ptype_ref) != ptype:
            errs.append(f"{where}: {name} tipi {ptype_ref}, dosyada {ptype}")
        try:
            v = float(value)
        except ValueError:
            errs.append(f"{where}: {name} sayısal değil")
            continue
        if ptype_ref == "INT32" and not re.fullmatch(r"-?\d+", value):
            errs.append(f"{where}: {name} INT32, değer tam sayı yazılmalı")
        errs += [f"{where}: {e}" for e in px4_value_errors(name, v, ref[name])]
    return errs


def validate_ardupilot(path: Path, ref: dict | None = None) -> list[str]:
    ref = ref or _load_ref(AP_REF)
    errs, seen = [], set()
    for lineno, name, value in parse_ardupilot(path):
        where = f"{path.name}:{lineno}"
        if not PARAM_NAME.match(name):
            errs.append(f"{where}: geçersiz ad {name!r}")
            continue
        if name in seen:
            errs.append(f"{where}: {name} iki kez tanımlı")
        seen.add(name)
        if name not in ref:
            errs.append(f"{where}: {name} ArduPilot Copter 4.7.1'de yok")
            continue
        try:
            v = float(value)
        except ValueError:
            errs.append(f"{where}: {name} sayısal değil")
            continue
        errs += [f"{where}: {e}" for e in ap_value_errors(name, v, ref[name])]
    return errs


def default_files() -> tuple[list[Path], list[Path]]:
    cfg = ROOT / "config"
    return sorted((cfg / "px4").rglob("*.params")), sorted((cfg / "ardupilot").rglob("*.param"))


def main(argv: list[str]) -> int:
    if argv:
        px4 = [Path(a) for a in argv if a.endswith(".params")]
        ap = [Path(a) for a in argv if a.endswith(".param")]
    else:
        px4, ap = default_files()
    errors = []
    for p in px4:
        errors += validate_px4(p)
    for p in ap:
        errors += validate_ardupilot(p)
    for e in errors:
        print("HATA:", e)
    print(f"{len(px4)} PX4 + {len(ap)} ArduPilot dosyası kontrol edildi, {len(errors)} hata.")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
