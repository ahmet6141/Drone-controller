"""One command for every YK-250 output (run from the repository root):

    python3 -m ucav250.build_all                 # analyses + registry checks + BOM + GA + guide + Blender + previews
    python3 -m ucav250.build_all --quick         # quick checks, no previews
    python3 -m ucav250.build_all --no-blender    # pure-Python outputs only

Exit code 1 if any check, sizing limit, structural margin or mass/CG limit fails.
"""
from __future__ import annotations

import argparse
import importlib
import json
import sys
import time


def _step(name, fn, results):
    t0 = time.time()
    try:
        r = fn()
        ok = bool(r.get("ok", True)) if isinstance(r, dict) else bool(r == 0 or r is None)
    except SystemExit as exc:
        r, ok = {"exit": exc.code}, exc.code in (0, None)
    results[name] = {"ok": ok, "seconds": round(time.time() - t0, 1)}
    print(f"[build_all] {name}: {'OK' if ok else 'FAIL'} ({results[name]['seconds']} s)", flush=True)
    return r


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--no-blender", action="store_true")
    ap.add_argument("--no-previews", action="store_true")
    a = ap.parse_args(argv)
    res = {}
    for mod, label in (("ucav250.analysis.sizing", "sizing"), ("ucav250.analysis.structures", "structures")):
        try:
            m = importlib.import_module(mod)
        except ModuleNotFoundError:
            print(f"[build_all] {label}: module missing, skipped")
            continue
        _step(label, lambda m=m: m.main(["--check"]) if label == "sizing" else m.main([]), res)
    from .core.assemble import build_registry
    reg = _step("registry", lambda: {"ok": True, "parts": len(build_registry().parts)}, res)
    reg = build_registry()
    from .analysis import checks, mass
    _step("checks", lambda: checks.run_all(reg, quick=a.quick), res)
    _step("mass", lambda: mass.run(reg), res)
    from .outputs import assembly_guide, bom, drawings
    _step("bom", lambda: bom.write(reg), res)
    _step("drawings", lambda: drawings.draw(reg), res)
    _step("assembly_guide", lambda: {"ok": not assembly_guide.write(reg)["problems"]}, res)
    if not a.no_blender:
        def blender():
            from .blender import build as B
            from .core.spec import OUT_DIR
            B.reset_scene()
            info = B.build_scene(reg)
            if not (a.no_previews or a.quick):
                info["previews"] = B.render_previews(reg.spec, OUT_DIR / "previews")
            import bpy
            bpy.ops.wm.save_as_mainfile(filepath=str(OUT_DIR / "yk250.blend"), compress=True)
            bpy.ops.export_scene.gltf(filepath=str(OUT_DIR / "yk250.glb"), export_format="GLB")
            return info
        _step("blender", blender, res)
    ok = all(v["ok"] for v in res.values())
    print(json.dumps(res))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
