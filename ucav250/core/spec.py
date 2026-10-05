"""Spec loader: ``ucav250/spec.yaml`` is the single source of truth (all lengths in metres, SI units)."""
from __future__ import annotations

import functools
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC_PATH = ROOT / "spec.yaml"
OUT_DIR = ROOT / "out"
DATA_DIR = ROOT / "data"


@functools.lru_cache(maxsize=None)
def load(path: str | None = None) -> dict:
    p = Path(path) if path else SPEC_PATH
    with open(p, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def get(spec: dict, dotted: str, default=None):
    """``get(spec, "wing.span")`` -> nested value (KeyError if missing and no default)."""
    cur = spec
    for k in dotted.split("."):
        if isinstance(cur, dict) and k in cur:
            cur = cur[k]
        elif default is not None:
            return default
        else:
            raise KeyError(dotted)
    return cur


def reload() -> dict:
    load.cache_clear()
    return load()
