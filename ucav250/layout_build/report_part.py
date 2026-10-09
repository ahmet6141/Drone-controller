# =====================================================================================================================
# report (Turkish)
# =====================================================================================================================
TR_ITEMS = [
    ("part_numbers: module ranges", "parça numarası aralıkları modüller arasında çakışmıyor"),
    ("part ids: convention", "parça kimlikleri YK250-<KOD>-NNN kuralına ve modül aralığına uyuyor"),
    ("part ids unique", "parça kimlikleri yerleşim içinde tekil"),
    ("root_part", "kök parça (orta kanat kutusu) bir şasi elemanı"),
    ("references", "çapraz başvurular çözülüyor (temas, iniş yüzeyleri, RF pencereleri, mafsallar, açıklık seçicileri, "
                   "kütle kalemleri, erişim kapakları)"),
    ("stations ordered", "istasyonlar x boyunca sıralı, kimlikler tekil"),
    ("stations: type", "istasyon türü, malzeme/süreç/katman anahtarları, kalınlık >= süreç alt sınırı, yüzler"),
    ("cut-outs (pass-throughs)", "geçiş kesikleri çerçeve gövdesi içinde"),
    ("frames do not cut", "çerçeveler yakıt, taret, takım kuyusu, yük/paraşüt/teçhizat hacimlerini kesmiyor"),
    ("fuel bays conform", "yakıt bölmeleri ok açılı kiriş çerçevelerini izliyor (çerçeve gövdesine boşluk)"),
    ("usable fuel volume", "ok açılı bölmelerin kullanılabilir yakıt hacmi >= gerekli hacim"),
    ("harness / push-rod / bridle", "kablo, itme çubuğu ve kayış geçişleri tanımlı kesiklerden"),
    ("equipment envelopes inside", "teçhizat zarfları dış yüzeyin (OML) içinde, 10 mm pay"),
    ("member envelopes inside", "şasi elemanları OML içinde (kaplamaya oturan yüzler hariç)"),
    ("fitting envelopes inside", "bağlantı parçaları OML / kuyruk içinde"),
    ("actuator envelopes inside", "kanat / dikey eyleyicileri kesit içinde, 3 mm pay"),
    ("harness trunks inside", "kablo demetleri OML içinde (yarıçap + 3 mm)"),
    ("engine mount truss inside", "motor bağlantı kafesi kaporta içinde"),
    ("turret growth envelope", "taret büyüme zarfı (toplanmış) OML içinde"),
    ("no overlaps", "içerik / yapı çakışması yok"),
    ("gear retraction sequence", "takım toplama dizisi"),
    ("turret E180 envelope along", "taret E180 zarfı strok boyunca bölme duvarları / tavan / çerçevelere"),
    ("turret E180 envelope vs elevator", "taret E180 zarfı asansör raylarına / bilyalı vidaya"),
    ("turret E180 envelope vs sliding", "taret E180 zarfı kayar kapaklara (bağlı dizi)"),
    ("HD59 ball vs aperture", "HD59 topu ile açıklık halkası arası (radyal)"),
    ("stabilators (whole range, both sides)", "stabilatörler (tüm sapma) egzoz zarflarına"),
    ("stabilators (whole range) outside", "stabilatörler egzoz duman konisinin dışında"),
    ("ailerons / flaps", "kanatçık / flap (tüm sapma) kanat eyleyicilerine"),
    ("parachute hatch", "paraşüt kapağı (0-110°) dış antenlere, sondalara, ışıklara"),
    ("layout.mass_placement =", "kütle yerleşimi yerleşim nesnelerinden yeniden hesaplananla aynı"),
    ("spec mass items at", "spec kütle kalemleri yerleşim konumlarında (sizing --update-spec uygulandı)"),
    ("empty-aircraft CG", "boş uçak AM'si: spec kalemleri ile yerleşim kalemleri arasındaki fark"),
    ("turret field of regard", "taret görüş alanı: bütün dış çıkıntılar -5° konisinin üstünde"),
    ("RF windows", "RF pencereleri: iç antenlerin tümü GFRP panel / uç kapağı altında"),
    ("GNSS antennas", "GNSS antenleri üst yüzey pencerelerinin altında"),
    ("Li-ion buffer battery", "Li-ion tampon batarya ile yakıt hücreleri arası"),
    ("fuel cells to the firewall", "yakıt hücreleri ile yangın perdesi ön yüzü arası (CS-LUAS.967(c))"),
    ("engine dynamic envelope", "motor dinamik zarfı ile diğer nesneler arası"),
    ("engine-mount truss vs SG750", "motor bağlantı kafesi ile SG750 arası"),
    ("cylinder/head hot zone", "silindir/kafa sıcak bölgesi ile kompozit yapı arası"),
    ("exhaust routing envelopes vs unshielded", "egzoz zarfı ile kalkansız kompozit yapı arası"),
    ("exhaust routing envelopes vs harness", "egzoz zarfı ile kablo demetleri arası"),
    ("ventral fin vs exhaust", "ventral kanatçık ile egzoz zarfı / duman konisi"),
    ("propeller disc keep-out", "pervane diski yasak bölgesi boş"),
    ("parachute deployment volume", "paraşüt açılma hacmi boş"),
    ("shell panels:", "kabuk panelleri: kenar payı, aralık, menteşe, RF malzemesi"),
    ("upper body covered", "üst gövde burundan kaporta çıkışına kadar panellerle kaplı"),
    ("maintenance: every", "bakım: her teçhizat ve yakıt hücresi sökülebilir/menteşeli bir kapağın altında"),
    ("maintenance access matrix", "bakım erişim matrisi: hiçbir kalem için birincil yapı sökülmüyor"),
    ("assembly steps numbered", "montaj adımları 1..N, Türkçe başlık/alt montaj/metin/takım/kontrol"),
    ("assembly covers", "montaj şasi tezgâhından ayara kadar bütün grupları kapsıyor"),
    ("transport units", "taşıma birimleri R-29 / R-30 ile uyumlu"),
    ("field re-assembly", "sahada montaj ve bakım matrisi mevcut"),
    ("mechanism definitions", "mekanizma tanımları (alanlar, aralıklar, özellikler, ifadeler, L/R çiftleri, diziler)"),
    ("layout.clearances in checks.py", "layout.clearances checks.py LISTE biçiminde"),
    ("first-cut fitting pre-sizing", "bağlantıların ilk ön boyutlandırması: emniyet payları >= 0"),
]


def _tr(item: str) -> str:
    for k, v in TR_ITEMS:
        if item.startswith(k):
            return v
    return item


def _fmt(v):
    if v is None:
        return "–"
    if isinstance(v, float):
        return f"{v:.4g}".replace(".", ",")
    if isinstance(v, list):
        return "[" + "; ".join(_fmt(x) for x in v) + "]"
    return str(v)


def write_report(ctx: Ctx, res: dict, figs: dict, out_dir) -> Path:
    S, L = ctx.S, ctx.L
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    W = []
    w = W.append
    w("# YK-250 HANÇER — Yerleşim ve yapı arayüzü kontrolleri\n")
    w("Bu rapor `python3 -m ucav250.analysis.layout_check` tarafından `ucav250/spec.yaml` (`layout`, `assembly`) "
      "bölümlerinden üretilir; ayrıntılı tasarım modülleri (şasi, kanat, kuyruk, kabuk, itki, yakıt, takım, sistemler, "
      "faydalı yük) birbirlerinin geometrisini okumadan bu arayüzden çalışır. Açıklama ve gerekçeler: "
      "`docs/03_yerlesim_ve_yapi_konsepti.md`.\n")
    w(f"**Sonuç: {res['n_pass']}/{res['n']} kontrol geçti** ({res['n_objects']} yerleşim nesnesi, {res['close_pairs']} "
      f"yakın çift, süre {_fmt(res['elapsed_s'])} s).\n")
    w("## 1. Kontrol özeti\n")
    w("| No | Kontrol | Değer | Sınır | Sonuç |\n|---|---|---|---|---|")
    for r in res["rows"]:
        det = f" — {r['detail']}" if r["detail"] and not r["ok"] else ""
        w(f"| {r['check']} | {_tr(r['item'])}{det} | {_fmt(r['value'])} | {_fmt(r['limit'])} | "
          f"{'GEÇTİ' if r['ok'] else '**KALDI**'} |")
    cg = res["cg"]
    w("\n## 2. Kütle yerleşimi ve ağırlık merkezi\n")
    w(f"Boş kütle {_fmt(cg['empty_kg'])} kg; spec kalemlerinden AM {_fmt(cg['cg_spec'])} m, yerleşim nesnelerinden "
      f"AM {_fmt(cg['cg_layout'])} m. Aşağıdaki kalemlerin konumu boyutlandırma evresinde varsayımdı; yerleşim evresinde "
      "yerleştirilen nesnelerin ağırlık merkezinden hesaplanır (`layout.mass_placement`) ve `sizing.mass_items` bunu "
      "uygular. Boyutlandırma evresi konumları `position_sizing_phase` alanındadır.\n")
    w("| Kütle kalemi | Boyutlandırma evresi x (m) | Yerleşim x (m) | Δx (m) | Esas |\n|---|---|---|---|---|")
    for k, v in L["mass_placement"].items():
        ps = v.get("position_sizing_phase")
        dx = (v["position"][0] - ps[0]) if ps else None
        w(f"| {k} | {_fmt(ps[0]) if ps else '–'} | {_fmt(v['position'][0])} | {_fmt(dx)} | {v['basis'][:110]} |")
    w("\n## 3. Arayüz bağlantılarının ilk ön boyutlandırması\n")
    w("Yöntem: kapalı biçimli pim kesme / eğilme / ezilme ve cıvata grubu kesmesi, `spec.materials` izin verilen "
      "değerleri; yük katsayıları `structures` (FoS 1,5, bağlantı 1,15, pimli bağlantı ezilme 2,0, sık sökülen "
      "bağlantı 1,5) ve standards.yaml önerileri. Ayrıntılı tasarım SE/test ile değiştirir.\n")
    w("| Kalem | Uygulanan | İzin verilen | Emniyet payı | Esas |\n|---|---|---|---|---|")
    for r in res["presizing"]:
        w(f"| {r['item']} | {_fmt(r['applied'])} | {_fmt(r['allowable'])} | {_fmt(r['MS'])} | {r['basis']} |")
    w("\n## 4. İstasyonlar (çerçeveler / perdeler)\n")
    w("| Kimlik | x (m) | Tür | Malzeme / süreç / katman | t (mm) | Kesikler |\n|---|---|---|---|---|---|")
    for s_ in L["stations"]:
        cuts = ", ".join(c["id"] for c in s_.get("cutouts", [])) or "–"
        w(f"| {s_['id']} | {_fmt(float(s_['x']))} | {s_['type']}{' / ' + s_['subtype'] if s_.get('subtype') else ''} "
          f"| {s_['material']} / {s_['process']} / {s_.get('layup') or '–'} | {_fmt(float(s_['t']) * 1000)} | {cuts} |")
    w("\n## 5. Şasi elemanları ve bağlantılar\n")
    w("| Kimlik | Parça | Ad | Malzeme | Yük yolu |\n|---|---|---|---|---|")
    for m in L["chassis"]["members"]:
        w(f"| {m['id']} | {m['part']}{' (L/R)' if m.get('mirror') else ''} | {m.get('name_tr', m['name'])} | "
          f"{m['material']} | {m['load_path'][:120]} |")
    for f in L["chassis"]["fittings"]:
        w(f"| {f['id']} | {f['part']}{' (L/R)' if f.get('mirror') else ''} | {f.get('name_tr', f['name'])} | "
          f"{f['material']} | {str(f.get('attach', ''))[:120]} |")
    wj = L["chassis"]["wing_joint"]
    pins = wj["main_spar"]["pins"]
    w(f"\nDış panel birleşimi y = {_fmt(float(wj['plane']['y']))} m: dil-çatal, iki Ø{pins[0]['diameter'] * 1000:.0f} mm "
      f"Ti-6Al-4V pim ({', '.join(str(_r(p['position'], 3)) for p in pins)}), arka kirişte Ø"
      f"{wj['rear_spar']['pin']['diameter'] * 1000:.0f} mm sürükleme pimi; takma yolu ana kiriş ekseni boyunca "
      f"{_fmt(wj['insertion']['stroke'])} m.\n")
    em = L["chassis"]["engine_mount"]
    w(f"Motor bağlantısı: {em['type']} — 4 x M8 cıvata, sönümleyici: {em['isolators']['make_model']}; yangın perdesi "
      f"yığını {', '.join(l_['layer'] + ' ' + _fmt(l_['t'] * 1000) + ' mm' for l_ in em['firewall_stackup']['layers_fwd_to_aft'])}.\n")
    br = L["chassis"]["parachute"]["bridle"]
    w(f"Paraşüt: Y-kayış, ön ayak {_fmt(br['forward_leg']['length'])} m (FS1810? → {br['forward_leg']['fitting']}), arka ayak "
      f"{_fmt(br['aft_leg']['length'])} m ({br['aft_leg']['fitting']}); açılma yükü 13,1 kN tek ayakta, nihai yalnız "
      "(CRASH-004).\n")
    w("## 6. Mekanizmalar\n")
    w("| Mafsal | Tür | Eksen | Alt / üst | Özellik / ifade |\n|---|---|---|---|---|")
    for j in L["mechanisms"]["joints"]:
        w(f"| {j['name']} | {j['kind']} | {_r(j['axis'], 3)} | {_fmt(float(j['lo']))} / {_fmt(float(j['hi']))} | "
          f"{j.get('prop') or ''} {('`' + j['expr'] + '`') if j.get('expr') else ''} |")
    for k, sq in L["mechanisms"]["sequences"].items():
        w(f"\nDizi `{k}`: {len(sq['states'])} örnek durum, denetim özelliği `{sq['control']}`.")
    w("\n## 7. Kabuk\n")
    cnt = {}
    for p in L["shell"]["panels"]:
        cnt[p["attach"]] = cnt.get(p["attach"], 0) + (2 if p.get("mirror") else 1)
    rf = [p["id"] for p in L["shell"]["panels"] if p.get("rf_window")]
    w(f"Panel sayısı (L/R ayrı): {', '.join(f'{k} {v}' for k, v in cnt.items())}; RF pencereleri: {', '.join(rf)}.\n")
    w("## 8. Bakım erişim matrisi\n")
    w("| Kalem | Erişim | Bağlantı elemanı | Birincil yapı sökülür mü |\n|---|---|---|---|")
    for r in S["assembly"]["maintenance_access"]:
        w(f"| {r['item']} | {', '.join(r['access'])} | {r['fasteners']} | "
          f"{'evet' if r['primary_structure_removed'] else 'hayır'} |")
    w("\n## 9. Montaj ve taşıma\n")
    st_ = S["assembly"]["steps"]
    w(f"{len(st_)} montaj adımı (spec.assembly.steps): " + "; ".join(f"{s_['step']}. {s_['title_tr']}" for s_ in st_) +
      ".\n")
    w(S["assembly"]["transport"]["text_tr"] + "\n")
    w("## 10. Şekiller\n")
    for k, v in (figs or {}).items():
        w(f"![{k}](../docs/fig/{Path(v).name})")
    text = "\n".join(W) + "\n"
    (out_dir / "layout.md").write_text(text)
    js = {k: v for k, v in res.items() if k != "cg"}
    js["cg"] = {k: v for k, v in res["cg"].items() if k != "placements"}
    (out_dir / "layout.json").write_text(json.dumps(js, indent=1, default=str))
    return out_dir / "layout.md"


