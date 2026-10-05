"""Build the full part registry from the spec by running every producer module in order."""
from __future__ import annotations

import importlib

from . import spec as S
from .parts import Registry

# Producer modules (ucav250/design/<name>.py), in dependency order. Each defines register(reg, spec).
# "hardware" must stay last: it turns every Fastener record into geometry.
MODULES = ["chassis", "wing", "tail", "shell", "propulsion", "fuel", "gear", "systems", "payload", "hardware"]


def build_registry(spec: dict | None = None, modules: list[str] | None = None, strict: bool = True) -> Registry:
    spec = spec or S.load()
    reg = Registry(spec)
    for name in modules or MODULES:
        try:
            mod = importlib.import_module(f"ucav250.design.{name}")
        except ModuleNotFoundError as exc:
            if strict and exc.name == f"ucav250.design.{name}":
                raise
            reg.note(f"module {name} not available: {exc}")
            continue
        mod.register(reg, spec)
    return reg
