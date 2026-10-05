"""Turkish assembly guide -> out/montaj_kilavuzu.md.

Steps come from ``spec.assembly.steps``: ``[{step, title, text, tools: [...], checks: [...], subassembly}]`` (Turkish
text). For every step the guide lists the parts installed (``Part.step``) and the fasteners (``Fastener.step``) with
designation, nut/insert and torque, plus the joined parts. Parts or fasteners whose step has no description are
reported so that nothing is installed "silently".

CLI:  python3 -m ucav250.outputs.assembly_guide
"""
from __future__ import annotations

import sys
from collections import defaultdict

from ..core.parts import Registry


def build(reg: Registry) -> tuple[str, list[str]]:
    spec = reg.spec
    steps = {int(s["step"]): s for s in (spec.get("assembly", {}) or {}).get("steps", []) or []}
    parts_by = defaultdict(list)
    for p in reg.parts.values():
        if p.process != "consumable" and p.group != "hardware":
            parts_by[p.step].append(p)
    fast_by = defaultdict(list)
    for f in reg.fasteners():
        fast_by[f.step].append(f)
    problems = []
    all_steps = sorted(set(parts_by) | set(fast_by) | set(steps))
    meta = spec.get("meta", {}) or {}
    L = [f"# {meta.get('name', 'YK-250')} — montaj kılavuzu", "",
         "Bu belge `ucav250/spec.yaml` ve parça kaydından otomatik üretilir; elle düzenlemeyin. Sıkma torkları kuru "
         "dişler içindir (aksi belirtilmedikçe). Her adımın sonundaki denetimler yapılmadan sonraki adıma geçilmez.", ""]
    gen = (spec.get("assembly", {}) or {}).get("general", []) or []
    if gen:
        L += ["## Genel kurallar", ""] + [f"- {g}" for g in gen] + [""]
    for k in all_steps:
        s = steps.get(k)
        if s is None and (parts_by.get(k) or fast_by.get(k)):
            problems.append(f"adım {k}: tanım yok ({len(parts_by.get(k, []))} parça, {len(fast_by.get(k, []))} "
                            f"bağlantı elemanı)")
        title = s["title"] if s else f"Adım {k}"
        L += [f"## Adım {k} — {title}", ""]
        if s and s.get("subassembly"):
            L += [f"*Alt montaj:* {s['subassembly']}", ""]
        if s and s.get("text"):
            L += [s["text"].strip(), ""]
        if s and s.get("tools"):
            L += ["**Aletler:** " + ", ".join(s["tools"]), ""]
        ps = sorted(parts_by.get(k, []), key=lambda p: p.id)
        if ps:
            L += ["| Parça no | Ad | Malzeme / süreç | Takıldığı yer |", "|---|---|---|---|"]
            for p in ps:
                L.append(f"| {p.id} | {p.name_tr} | {p.material} / {p.process} | {p.parent or '—'} |")
            L.append("")
        fs = fast_by.get(k, [])
        if fs:
            roll = defaultdict(list)
            for f in fs:
                roll[(f.spec, f.nut, f.torque_nm)].append(f)
            L += ["| Bağlantı elemanı | Somun / insert | Adet | Tork (N·m) | Bağladığı parçalar |", "|---|---|---:|---:|---|"]
            for (sp, nut, tq), group in sorted(roll.items(), key=lambda kv: kv[0][0]):
                joined = sorted({j for f in group for j in f.joins})
                L.append(f"| {sp} | {nut or '—'} | {len(group)} | {'—' if tq is None else tq} | "
                         f"{', '.join(joined[:8])}{' …' if len(joined) > 8 else ''} |")
            L.append("")
        if s and s.get("checks"):
            L += ["**Denetimler:**", ""] + [f"- [ ] {c}" for c in s["checks"]] + [""]
    if problems:
        L += ["## Eksik adım tanımları", ""] + [f"- {p}" for p in problems] + [""]
    return "\n".join(L), problems


def write(reg: Registry) -> dict:
    from ..core.spec import OUT_DIR
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    md, problems = build(reg)
    (OUT_DIR / "montaj_kilavuzu.md").write_text(md, encoding="utf-8")
    return {"file": str(OUT_DIR / "montaj_kilavuzu.md"), "problems": problems}


def main(argv=None) -> int:
    from ..core.assemble import build_registry
    r = write(build_registry())
    print(r)
    return 0 if not r["problems"] else 1


if __name__ == "__main__":
    sys.exit(main())
