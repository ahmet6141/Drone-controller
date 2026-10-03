"""DC7 v2 bütünleşik gövde — CadQuery parametrik modelleri (seri üretime uygun).

Parçalar (üretim malzemesi):
  * top_shell        — üst kabuk / kanopi (PC/ABS), z > Z_SPLIT
  * bottom_tub       — taşıyıcı alt kabuk (PC/ABS): kol soketleri, batarya rayları, vida göbekleri
  * nose_cover       — burun üst yarısı: alın + yanakların üstü (PC/ABS), x > NOSE_SPLIT_X, z > PIVOT_Z
  * nose_chin        — burun alt yarısı: yanakların altı + sönümleyici perdesi (PC/ABS); gimbal iki yarının
                       arasına oturur (pitch ekseni ayrım düzleminde), cad/v2/gimbal_v2.py
  * arm_duct         — kol (U kesit, kaburgalı) + motor yuvası + çan ağızlı kanal + altıgen ızgara
                       (PA6-GF30, tek kalıp ×4)
  * pod              — avuç ayağı (PC/ABS) + tpu_tip (TPU)
  * battery          — akıllı batarya: kuyruk kapağı (gövde çizgisini tamamlar) + paket kabuğu (PC/ABS)
Gövde kesitleri v2_params.STATIONS'tan (paketleme çalışmasıyla aynı) üretilir.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

import layout_v2 as K  # noqa: E402
import params as P  # noqa: E402
import v2_params as V  # noqa: E402

BIG = 1000.0


def _loft(inset: float = 0.0, xs: list[float] | None = None) -> cq.Workplane:
    xs = xs or [s[0] for s in V.STATIONS]
    wp = cq.Workplane("YZ")
    prev = None
    for x in xs:
        wp = wp.workplane(offset=x if prev is None else x - prev)
        wp = wp.spline(V.section_points(x, inset), periodic=True).close()
        prev = x
    return wp.loft(ruled=False)


def _xbox(x0: float, x1: float, y=BIG, z0=-BIG, z1=BIG) -> cq.Workplane:
    return cq.Workplane("XY").box(x1 - x0, 2 * y, z1 - z0, centered=False).translate((x0, -y, z0))


def battery_box(x: float | None = None, clear: float = 0.0) -> tuple[float, float, float, float, float, float]:
    bx = K.solve_battery_x() if x is None else x
    px, py, pz = V.PACK
    z0, z1 = V.PACK_Z
    return (bx - px / 2 - clear, bx + px / 2 + clear, -py / 2 - clear, py / 2 + clear, z0 - clear, z1 + clear)


def cap_x() -> tuple[float, float]:
    """Batarya kuyruk kapağının x aralığı (gövde loft'unun son dilimi)."""
    x0 = V.STATIONS[0][0]
    return x0, x0 + V.TAIL_CAP


def outer() -> cq.Workplane:
    return _loft(0.0)


def shell(wall: float = V.WALL) -> cq.Workplane:
    """Kapalı uçlu kabuk: dış loft − iç loft (uçlardan et kalınlığı kadar kırpılmış)."""
    x_lo, x_hi = V.body_x_range()
    inner = _loft(wall).intersect(_xbox(x_lo + wall, x_hi - wall))
    return outer().cut(inner)


def _place(wp: cq.Workplane, mx: float, my: float) -> cq.Workplane:
    """Yerel kol koordinatından (motor orijinde, kol −x yönünde) gövde koordinatına."""
    return wp.rotate((0, 0, 0), (0, 0, 1), math.degrees(math.atan2(my, mx))).translate((mx, my, 0))


def _arm_root_cut() -> cq.Workplane:
    """Kol kökü soket açıklıkları (4 köşe): kökün kendi şekli (V uçlu, yuvarlatılmış kesit) + ARM_FIT."""
    w, h = V.ARM_ROOT
    g = V.ARM_FIT
    x0 = arm_span()[0]
    zc = sum(arm_root_z()) / 2
    local = _rr_prism(w + 2 * g, h + 2 * g, 4.0 + g, zc, x0 - 6.0, -(P.MOTOR_R - 90.0)).intersect(_v_keep(g))
    cut = None
    for mx, my in P.motor_positions():
        c = _place(local, mx, my)
        cut = c if cut is None else cut.union(c)
    return cut


def arm_root_z() -> tuple[float, float]:
    h = V.ARM_ROOT[1]
    top = 18.0
    return top - h, top


def body_parts() -> dict[str, cq.Workplane]:
    sh = shell()
    # Batarya tüneli ağzı (kuyruk) — kapak bu dilimi doldurur
    cx0, cx1 = cap_x()
    sh = sh.cut(_xbox(cx0 - 1.0, cx1))
    # Avuç ayağı açıklığı (karın)
    sh = sh.cut(cq.Workplane("XY").workplane(offset=-80.0).center(V.POD_X, 0).circle(V.POD_TOP_R - 3.0).extrude(40.0))
    # Kol soketleri
    sh = sh.cut(_arm_root_cut())
    body = sh.cut(_xbox(V.NOSE_SPLIT_X, BIG))
    # Burun (ince et): ağız gimbal kapsülünü yanaklar arasında taşır; astar ve motor/pim geçişleri
    nose = (shell(V.NOSE_WALL).intersect(_xbox(V.NOSE_SPLIT_X, BIG)).cut(mouth())
            .union(_mouth_lining()).cut(_mouth_holes()))
    top = body.intersect(_xbox(-BIG, BIG, z0=V.Z_SPLIT))
    bottom = body.intersect(_xbox(-BIG, BIG, z1=V.Z_SPLIT))
    nose_top = nose.intersect(_xbox(-BIG, BIG, z0=V.PIVOT_Z))
    chin = nose.intersect(_xbox(-BIG, BIG, z1=V.PIVOT_Z)).union(_bulkhead())
    return {"top_shell": top, "bottom_tub": bottom, "nose_cover": nose_top, "nose_chin": chin}


def mouth(grow: float = 0.0) -> cq.Workplane:
    """Burun ağzı (gövde koordinatında): gimbal kapsülünün süpürme hacmi + boşluk. Arkası pitch ekseni etrafında
    yay, üstü düz alın, önü ve altı açık (kamera −90°'de aşağı bakar). grow > 0: astarın dış yüzü."""
    px, _, pz = V.GIMBAL_PIVOT
    hy, r, top = V.MOUTH_HALF_Y + grow, V.MOUTH_R + grow, pz + V.MOUTH_TOP + grow
    disc = _cyl_y(px, pz, -hy, hy, r)
    front = _xbox(px, BIG, y=hy, z0=-BIG, z1=top)
    return disc.union(front).intersect(_xbox(-BIG, BIG, y=hy, z1=top))


def _cyl_y(cx: float, cz: float, y0: float, y1: float, r: float) -> cq.Workplane:
    return cq.Workplane("XZ").workplane(offset=-y0).center(cx, cz).circle(r).extrude(-(y1 - y0))


def _mouth_lining() -> cq.Workplane:
    """Ağız astarı (burunla aynı parça, siyah): yanakların iç yüzleri, alın alt yüzü ve arka yay. Kabuğun içini
    ağızdan ayırır; kabuk etine 0,3 mm girerek onunla birleşir."""
    return mouth(V.MOUTH_WALL).cut(mouth()).intersect(_loft(V.NOSE_WALL - 0.3))


def mouth_liner() -> cq.Workplane:
    """Ağız astarı tek başına (render: 2K kalıpta siyah ikinci enjeksiyon)."""
    return _mouth_lining().cut(_mouth_holes())


def _mouth_holes() -> cq.Workplane:
    """Sağ astarda pitch motoru geçişi, sol astarda rulman pimi geçişi (sönümleyici yolu için ≥ 2 mm boşluk)."""
    px, _, pz = V.GIMBAL_PIVOT
    hy, t = V.MOUTH_HALF_Y, V.MOUTH_WALL
    motor = _cyl_y(px, pz, hy - 1.0, hy + t + 1.0, P.GIMBAL_MOTOR_D / 2 + 2.0)
    pin = _cyl_y(px, pz, -hy - t - 1.0, -hy + 1.0, 4.5)
    return motor.union(pin)


def _bulkhead() -> cq.Workplane:
    """Burun perdesi (alt yarıya ait): gimbal taşıyıcısı 4 sönümleyiciyle önüne bağlanır; elektronik bölmesini
    toz ve sudan ayırır. Sönümleyici cıvatalarında arkaya doğru pullar (2 × et); ortada kablo geçişi (kamera FFC
    şeridi + motor kabloları)."""
    x0, t = V.NOSE_SPLIT_X, V.BAY_WALL
    _, _, pz = V.GIMBAL_PIVOT
    # Alt kısmı alt yarının etiyle birleşir; üst kısmı üst yarıdan 0,1 mm ayrık (ayrı parçalar)
    wall = (_xbox(x0, x0 + t, z1=pz).intersect(_loft(V.NOSE_WALL - 0.3))
            .union(_xbox(x0, x0 + t, z0=pz - 0.5).intersect(_loft(V.NOSE_WALL + 0.1))))
    dy, dz = V.GIMBAL_DAMPERS
    for sy in (-1, 1):
        for sz in (-1, 1):
            c = (sy * dy / 2, pz + sz * dz / 2)
            wall = wall.union(cq.Workplane("YZ").workplane(offset=x0 - t).center(*c).circle(4.5).extrude(t + 0.1))
            wall = wall.cut(cq.Workplane("YZ").workplane(offset=x0 - t - 1.0).center(*c)
                            .circle(P.M3 / 2).extrude(2 * t + 2.0))
    slot = cq.Workplane("YZ").workplane(offset=x0 - 1.0).center(0, pz).slot2D(20.0, 7.0).extrude(t + 2.0)
    return wall.cut(slot)


# --- kol–kanal modülü (yerel: motor ekseni orijinde, kol −x yönünde gövdeye uzanır) ---------------
def _rr(w: float, h: float, r: float) -> cq.Sketch:
    return cq.Sketch().rect(w, h).vertices().fillet(min(r, w / 2 - 0.2, h / 2 - 0.2))


def _rr_prism(w: float, h: float, r: float, zc: float, xa: float, xb: float) -> cq.Workplane:
    """x yönünde sabit kesitli, köşeleri yuvarlatılmış dikdörtgen prizma."""
    return cq.Workplane("YZ").placeSketch(_rr(w, h, r).moved(cq.Location(cq.Vector(0, zc, xa))),
                                          _rr(w, h, r).moved(cq.Location(cq.Vector(0, zc, xb)))).loft()


def arm_span() -> tuple[float, float]:
    """Kolun yerel x aralığı: gövde içindeki kök ucu (V ucunun sivri noktası) → motor yuvası."""
    root = -(P.MOTOR_R - V.ARM_ROOT_R) - V.ARM_INSERT
    return root, -V.MOTOR_POD[0] + 3.0


def arm_prism_end() -> float:
    """Sabit kesitli kök bölümünün bittiği yerel x (gövde yüzeyi + ARM_PRISM)."""
    return -(P.MOTOR_R - V.ARM_ROOT_R - V.ARM_PRISM)


def _v_keep(offset: float = 0.0) -> cq.Workplane:
    """Kol kökünün 90° V ucu (yerel): lx ≥ x_v + |ly|. Yüzler gövde eksenlerine paraleldir (arka kollarda
    batarya tüneli duvarına, ön kollarda companion kartına paralel) → tek kalıp ×4 korunur. offset > 0
    yüzleri dışa kaydırır (soket boşluğu, kovan eti)."""
    xv = arm_span()[0] - offset * math.sqrt(2.0)
    L = 60.0
    return (cq.Workplane("XY").workplane(offset=-60.0)
            .polyline([(xv, 0.0), (xv + L, L), (xv + 400.0, L), (xv + 400.0, -L), (xv + L, -L)]).close()
            .extrude(160.0))


def _arm_section(x: float) -> tuple[float, float, float]:
    """(genişlik, yükseklik, merkez z): kökte sabit kesit, sonra motor ucuna doğrusal incelme."""
    xp, x1 = arm_prism_end(), arm_span()[1]
    t = min(max((x - xp) / (x1 - xp), 0.0), 1.0)
    (w0, h0), (w1, h1) = V.ARM_ROOT, V.ARM_TIP
    top0, top1 = arm_root_z()[1], V.ARM_TOP_TIP_Z + P.HUB_T
    w, h, top = w0 + t * (w1 - w0), h0 + t * (h1 - h0), top0 + t * (top1 - top0)
    return w, h, top - h / 2


def arm() -> cq.Workplane:
    """U kesitli (altı açık) kol + iç kaburgalar: iki parçalı kalıpta maçasız çıkar, kablolar içinden geçer.
    Kök: sabit kesitli, V uçlu; uçta et kalınlığında kapalı duvar."""
    x0, x1 = arm_span()
    xp = arm_prism_end()
    t = V.ARM_WALL
    secs = [(x0 - 1.0, *_arm_section(x0)), (xp, *_arm_section(xp)), (x1, *_arm_section(x1))]
    outer_ = cq.Workplane("YZ").placeSketch(
        *[_rr(w, h, 4.0).moved(cq.Location(cq.Vector(0, zc, x))) for x, w, h, zc in secs]).loft(ruled=True)
    inner_ = cq.Workplane("YZ").placeSketch(
        *[_rr(w - 2 * t, h + 6, 2.0).moved(cq.Location(cq.Vector(0, zc - t - 3, x + dx)))
          for (x, w, h, zc), dx in zip(secs, (-1.0, 0.0, 1.0))]).loft(ruled=True)
    body = outer_.intersect(_v_keep()).cut(inner_.intersect(_v_keep(-t)))
    x_full = x0 + V.ARM_ROOT[0] / 2                                   # V ucundan sonra tam kesit
    ribs = None
    for k in range(4):                                                # çapraz kaburgalar (≤ 0,6 × et)
        xr = x_full + 4.0 + (x1 - 8.0 - x_full - 4.0) * k / 3
        w, h, zc = _arm_section(xr)
        rib = cq.Workplane("XY").box(V.RIB_RATIO * t, w - t, h - t, centered=(True, True, False)).translate((xr, 0, zc - h / 2))
        ribs = rib if ribs is None else ribs.union(rib)
    return body.union(ribs.intersect(outer_))                         # kaburga köşeleri dış yüzeyden taşmasın


def motor_pod() -> cq.Workplane:
    r, h = V.MOTOR_POD
    z1 = P.HUB_T
    s = P.MOTOR_HOLE_SPACING / 2
    pod = cq.Workplane("XY").workplane(offset=z1 - h).circle(r).extrude(h)
    # hafifletme: 4 cep (motor delikleri arasında)
    pod = pod.cut(cq.Workplane("XY").workplane(offset=z1 - h + 2.0)
                  .pushPoints([(13.0 * math.cos(math.radians(a)), 13.0 * math.sin(math.radians(a))) for a in (0, 90, 180, 270)])
                  .circle(3.2).extrude(h - 4.0))
    pod = pod.cut(cq.Workplane("XY").workplane(offset=z1 - h).pushPoints([(s, s), (s, -s), (-s, s), (-s, -s)])
                  .circle(P.M3 / 2).extrude(h))
    pod = pod.cut(cq.Workplane("XY").workplane(offset=z1 - h).circle(4.5).extrude(h))
    # alttan oyuk (et kalınlığı sabit, motor kabloları için)
    pod = pod.cut(cq.Workplane("XY").workplane(offset=z1 - h).circle(r - V.ARM_WALL).circle(9.0).extrude(h - 3.0))
    return pod


def duct_ring() -> cq.Workplane:
    """Kalıptan çıkma açılı (iç duvar yukarı açılır) çan ağızlı kanal; yuvarlak üst kenar."""
    z0, z1 = V.DUCT_Z
    h = z1 - z0
    f, d = V.DUCT_FLARE
    t = V.DUCT_WALL
    r0 = V.DUCT_IN_R
    dr = (h - f) * math.tan(math.radians(V.DRAFT_DEG))
    return (cq.Workplane("XZ").moveTo(r0, z0).lineTo(r0 + t, z0).lineTo(r0 + t + dr, z1 - f)
            .lineTo(r0 + t + dr + d, z1).threePointArc((r0 + dr + d + t / 2, z1 + t / 2), (r0 + dr + d, z1))
            .lineTo(r0 + dr, z1 - f).close().revolve(360.0, (0, 0, 0), (0, 1, 0)))


def _honeycomb(z0: float, t: float, r_out: float, r_in: float, cell: float, rib: float) -> cq.Workplane:
    """Bal peteği disk (z0 … z0 + t), r_in … r_out arası: hücre iç çapı (düzlükler arası) `cell`, kaburga `rib`.
    polygon() köşesi +x'te → düzlükler ±y'ye bakar; y aralığı = hücre + kaburga, sütunlar x'te pitch·sin60
    aralıklı ve yarım adım kaydırılmış."""
    disc = cq.Workplane("XY").workplane(offset=z0).circle(r_out).circle(r_in).extrude(t)
    pitch = cell + rib
    d_circ = cell / math.cos(math.radians(30))
    n = int(r_out // (pitch * math.sin(math.radians(60)))) + 2
    pts = []
    for j in range(-n, n + 1):
        for i in range(-n, n + 1):
            x = j * pitch * math.sin(math.radians(60))
            y = (i + 0.5 * (j % 2)) * pitch
            r = math.hypot(x, y)
            if r_in - d_circ / 2 < r < r_out + d_circ / 2:
                pts.append((x, y))
    holes = (cq.Workplane("XY").workplane(offset=z0 - 1).pushPoints(pts)
             .polygon(6, d_circ).extrude(t + 2))
    return disc.cut(holes)


def grille() -> cq.Workplane:
    """Alt parmak ızgarası: hücre iç çapı ≤ 10 mm; göbek bileziği + 3 direk motor yuvasına iner;
    bilezikten yuvaya inen motor çanı eteği dönen çanı yandan da kapatır."""
    z0 = V.DUCT_Z[0]
    g = V.GRILLE
    c_in = P.MOTOR_BELL_D / 2 + P.COLLAR_GAP
    c_out = c_in + P.COLLAR_W
    disc = _honeycomb(z0, g["t"], V.DUCT_IN_R + 0.5, c_out - 0.5, g["cell"], g["rib"])
    collar = cq.Workplane("XY").workplane(offset=z0).circle(c_out).circle(c_in).extrude(P.SPOKE_T)
    # Radyal kaburgalar (60°, 180° = kol üstü, 300°): halkayı göbeğe bağlar, kalıpta göbekteki yolluktan
    # halkaya akış yolu olur
    sw, sh = V.SPOKE
    for ang in (60.0, 180.0, 300.0):
        spoke = (cq.Workplane("XY").box(V.DUCT_IN_R + 0.5 - (c_out - 0.5), sw, sh, centered=(False, True, False))
                 .translate((c_out - 0.5, 0, z0 + g["t"] - sh)).rotate((0, 0, 0), (0, 0, 1), ang))
        disc = disc.union(spoke)
    post_r = (c_in + c_out) / 2
    posts = None
    for ang in (0.0, 120.0, 240.0):
        x, y = post_r * math.cos(math.radians(ang)), post_r * math.sin(math.radians(ang))
        post = (cq.Workplane("XY").workplane(offset=P.HUB_T).center(x, y).circle(P.POST_D / 2)
                .extrude(z0 - P.HUB_T + 0.1))
        posts = post if posts is None else posts.union(post)
    # Motor çanı eteği: dikey havalandırma yarıkları (parmak geçmez), direklerin arasında
    st, slot_w = V.BELL_SKIRT
    h = z0 - P.HUB_T
    skirt = cq.Workplane("XY").workplane(offset=P.HUB_T).circle(c_in + st).circle(c_in).extrude(h + 0.1)
    slots = None
    for k in range(12):
        sl = (cq.Workplane("XY").box(st + 2.0, slot_w, h - 6.0, centered=(False, True, False))
              .translate((c_in - 1.0, 0, P.HUB_T + 3.0)).rotate((0, 0, 0), (0, 0, 1), 15.0 + 30.0 * k))
        slots = sl if slots is None else slots.union(sl)
    return disc.union(collar).union(posts).union(skirt.cut(slots))


def top_grille() -> cq.Workplane:
    """Üst parmak ızgarası (yerel: motor ekseni orijinde; çıkarılabilir PC parça, tek kalıp ×4).
    Bal peteği disk + dış bant + göbekte somun kapağı + üstte 3 radyal kaburga; çan ağzının üst kenarına
    6 ayakla oturur, 3 geçme tırnakla kenarın altından tutunur. Kenar ile disk arasındaki ≈ 4 mm'lik
    yan aralık ek hava girişidir ve parmak geçirmez."""
    g = V.TOP_GRILLE
    z, t = g["z"], g["t"]
    dr = (V.DUCT_Z[1] - V.DUCT_Z[0] - V.DUCT_FLARE[0]) * math.tan(math.radians(V.DRAFT_DEG))
    lip_in = V.DUCT_IN_R + dr + V.DUCT_FLARE[1]
    lip_out = lip_in + V.DUCT_WALL
    lip_top = V.DUCT_Z[1] + V.DUCT_WALL / 2
    r_out, cap_r, cap_top = g["r_out"], g["cap_r"], g["cap_top"]
    part = _honeycomb(z, t, r_out - 2.5, cap_r - 0.5, g["cell"], g["rib"])
    part = part.union(cq.Workplane("XY").workplane(offset=z).circle(r_out).circle(r_out - 3.0).extrude(t))
    cap = (cq.Workplane("XY").workplane(offset=z).circle(cap_r).extrude(cap_top - z)
           .cut(cq.Workplane("XY").workplane(offset=z - 0.1).circle(cap_r - 1.2).extrude(cap_top - z - 1.1)))
    part = part.union(cap)
    sw, sh = g["spoke"]
    for ang in (90.0, 210.0, 330.0):
        part = part.union(cq.Workplane("XY").box(r_out - 1.0 - (cap_r - 0.5), sw, sh, centered=(False, True, False))
                          .translate((cap_r - 0.5, 0, z + t - 0.1)).rotate((0, 0, 0), (0, 0, 1), ang))
    r_leg = (lip_in + lip_out) / 2
    for k in range(6):
        part = part.union(cq.Workplane("XY").box(1.4, 2.4, z - lip_top - 0.05 + 0.1, centered=(True, True, False))
                          .translate((r_leg, 0, lip_top + 0.05)).rotate((0, 0, 0), (0, 0, 1), 60.0 * k))
    for ang in (30.0, 150.0, 270.0):                                  # geçme tırnakları
        tab = (cq.Workplane("XY").box(1.2, 8.0, z + 0.1 - (lip_top - 3.5), centered=(False, True, False))
               .translate((lip_out + 0.4, 0, lip_top - 3.5)))
        hook = (cq.Workplane("XY").box(1.0, 8.0, 1.0, centered=(False, True, False))
                .translate((lip_out - 0.6, 0, lip_top - 3.5)))
        part = part.union(tab.union(hook).rotate((0, 0, 0), (0, 0, 1), ang))
    return part


def arm_duct() -> cq.Workplane:
    """Tek parça kol–kanal modülü (yerel koordinat)."""
    part = arm().union(motor_pod()).union(duct_ring()).union(grille())
    # Kanal ↔ kol köprüsü: kolun kanal altından geçtiği yerde
    xr = -(V.DUCT_IN_R + V.DUCT_WALL / 2)
    w, h, zc = _arm_section(xr)
    top = zc + h / 2
    web = (cq.Workplane("XY").box(10.0, w, V.DUCT_Z[0] - top + 0.5, centered=(True, True, False))
           .translate((xr, 0, top - 0.2)))
    return part.union(web)


def arm_duct_placed(module: cq.Workplane) -> list[cq.Workplane]:
    out = []
    for mx, my in P.motor_positions():
        ang = math.degrees(math.atan2(my, mx))
        out.append(module.rotate((0, 0, 0), (0, 0, 1), ang).translate((mx, my, 0)))
    return out


# --- avuç ayağı ----------------------------------------------------------------------------------
def belly_z() -> float:
    """Avuç ayağının bağlandığı karın yüzeyi (x = POD_X)."""
    return V.section_at(V.POD_X)[0]


def pod() -> cq.Workplane:
    """Avuç ayağı: üstte karın deliğine giren boyun, içte sensör tablası (tabandan SENSOR_RECESS).
    Sensörlerin mercekleri tabladan aşağı uzanır ve sensör camına (sensor_window) sıfır boşlukla değer."""
    t = V.WALL
    z_top = belly_z()
    z_bot = V.POD_BOTTOM_Z + P.BUMPER_H
    r0, r1 = V.POD_TOP_R, V.POD_BOTTOM_R
    neck_r = V.POD_TOP_R - 3.0 - 0.3
    z_mid = (z_top + z_bot) / 2 + 0.25 * (z_top - z_bot)
    body = (cq.Workplane("XZ").moveTo(neck_r - t, z_top + 4.0).lineTo(neck_r, z_top + 4.0).lineTo(neck_r, z_top)
            .lineTo(r0, z_top).spline([(r0 - 0.8, z_mid), (r1, z_bot)], includeCurrent=True)
            .lineTo(r1 - t, z_bot).spline([(r0 - 0.8 - t, z_mid), (r0 - t, z_top - t)], includeCurrent=True)
            .lineTo(neck_r - t, z_top - t).close().revolve(360.0, (0, 0, 0), (0, 1, 0)))
    shelf_z = V.POD_BOTTOM_Z + V.SENSOR_RECESS
    shelf = cq.Workplane("XY").workplane(offset=shelf_z).circle(r1 + 0.5).extrude(2.0)
    shelf = shelf.intersect(_pod_inner())
    openings = {"CM3 Wide": ("circle", 13.0), "VL53L8CX": ("rect", (9.0, 5.0)), "MTF-01": ("rect", (18.0, 7.0))}
    for name, ((x, y), _) in V.POD_SENSORS.items():
        kind, size = openings[name]
        wp = cq.Workplane("XY").workplane(offset=shelf_z - 0.1).center(x, y)
        wp = wp.circle(size / 2) if kind == "circle" else wp.rect(*size)
        shelf = shelf.cut(wp.extrude(2.2))
    # Üst kenar gövde alt yüzeyine tam oturur (karın x yönünde eğimli → arkada boşluk kalmasın)
    collar = (cq.Workplane("XY").workplane(offset=z_top - 1.0).circle(r0).circle(r0 - t).extrude(8.0)
              .cut(outer().translate((-V.POD_X, 0, 0))))
    return body.union(shelf).union(collar).translate((V.POD_X, 0, 0))


def _pod_inner() -> cq.Workplane:
    """Ayak gövdesinin iç hacmi (tabla ve cam bunun içinde kalır)."""
    z_top, z_bot = belly_z(), V.POD_BOTTOM_Z + P.BUMPER_H
    r0, r1 = V.POD_TOP_R, V.POD_BOTTOM_R
    z_mid = (z_top + z_bot) / 2 + 0.25 * (z_top - z_bot)
    return (cq.Workplane("XZ").moveTo(0, z_top).lineTo(r0 - 0.2, z_top)
            .spline([(r0 - 1.0, z_mid), (r1 - 0.2, z_bot)], includeCurrent=True)
            .lineTo(0, z_bot).close().revolve(360.0, (0, 0, 0), (0, 1, 0)))


def sensor_window() -> cq.Workplane:
    """IR geçirgen koruyucu cam (PC/PMMA, siyah IR mürekkep maskeli): tablanın hemen altında, sensörlere
    sıfır hava boşluğuyla (ToF çapraz konuşması en az); toz ve sudan korur."""
    shelf_z = V.POD_BOTTOM_Z + V.SENSOR_RECESS
    disc = cq.Workplane("XY").workplane(offset=shelf_z - V.SENSOR_WINDOW_T).circle(V.POD_BOTTOM_R).extrude(V.SENSOR_WINDOW_T)
    inner = _pod_inner().translate((0, 0, 0))
    r_fit = V.POD_BOTTOM_R - V.WALL - 0.6
    return disc.intersect(inner).intersect(
        cq.Workplane("XY").workplane(offset=shelf_z - 2.0).circle(r_fit).extrude(3.0)).translate((V.POD_X, 0, 0))


def pod_tip() -> cq.Workplane:
    """TPU uç halkası: avuca değen alt kenar tam yuvarlak."""
    r_out = V.POD_BOTTOM_R
    r_in = r_out - P.BUMPER_WALL
    rho = (r_out - r_in) / 2
    z0 = V.POD_BOTTOM_Z
    return (cq.Workplane("XZ").moveTo(r_in, z0 + rho).threePointArc(((r_in + r_out) / 2, z0), (r_out, z0 + rho))
            .lineTo(r_out, z0 + P.BUMPER_H + 0.2).lineTo(r_in, z0 + P.BUMPER_H + 0.2).close()
            .revolve(360.0, (0, 0, 0), (0, 1, 0)).translate((V.POD_X, 0, 0)))


# --- akıllı batarya ------------------------------------------------------------------------------
def battery_parts() -> dict[str, cq.Workplane]:
    x0, x1, y0, y1, z0, z1 = battery_box()
    cx0, cx1 = cap_x()
    x0 = cx0 + V.CAP_WALL - 0.5                                     # paket kapağın arka duvarına bağlanır
    t = V.PACK_WALL
    # Kuyruk kapağı: gövde loft'unun son dilimi (gövde çizgisini tamamlar), öne açık kap
    cap_outer = outer().intersect(_xbox(cx0, cx1))
    cap_inner = _loft(V.CAP_WALL).intersect(_xbox(cx0 + V.CAP_WALL, cx1 + 1.0))
    cap = cap_outer.cut(cap_inner)
    pack = (cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0).edges("|X").fillet(4.0)
            .translate(((x0 + x1) / 2, 0, (z0 + z1) / 2)))
    pack = pack.cut(cq.Workplane("XY").box(x1 - x0 - 2 * t, y1 - y0 - 2 * t, z1 - z0 - 2 * t)
                    .translate(((x0 + x1) / 2, 0, (z0 + z1) / 2)))
    shell_ = cap.union(pack)
    latch = None
    for sy in (-1, 1):                                                # kapak yanlarında kilit düğmeleri
        hw = V.half_width(cx0 + 7.0, (z0 + z1) / 2)
        b = (cq.Workplane("XZ").workplane(offset=-sy * (hw + 0.6)).center(cx0 + 7.0, (z0 + z1) / 2)
             .slot2D(10.0, 4.0, angle=90).extrude(sy * 1.6))
        latch = b if latch is None else latch.union(b)
    cells = None
    for iy in range(3):
        for iz in range(2):
            yc = (iy - 1) * V.CELL_D
            zc = z0 + t + V.CELL_D / 2 + iz * V.CELL_D
            c = (cq.Workplane("YZ").workplane(offset=x1 - V.CELL_L - 3.0).center(yc, zc).circle(V.CELL_D / 2 - 0.2)
                 .extrude(V.CELL_L))
            cells = c if cells is None else cells.union(c)
    return {"battery_shell": shell_, "battery_latch": latch, "battery_cells": cells}


# --- gövde iç detayları ----------------------------------------------------------------------------
def _sleeves() -> cq.Workplane:
    """Kol soket kovanları (gövde içinde): kökün V uçlu şekli + boşluk + et; ayrım düzlemine açık (maçasız)."""
    w, h = V.ARM_ROOT
    g, t = V.ARM_FIT, V.SLEEVE_WALL
    x0 = arm_span()[0]
    zc = sum(arm_root_z()) / 2
    x_out = -(P.MOTOR_R - V.ARM_ROOT_R) + 4.0
    outer_ = _rr_prism(w + 2 * (g + t), h + 2 * (g + t), 4.0 + g + t, zc, x0 - 6.0, x_out).intersect(_v_keep(g + t))
    inner_ = _rr_prism(w + 2 * g, h + 2 * g, 4.0 + g, zc, x0 - 8.0, x_out + 2.0).intersect(_v_keep(g))
    local = outer_.cut(inner_)
    out = None
    for mx, my in P.motor_positions():
        s_ = _place(local, mx, my)
        out = s_ if out is None else out.union(s_)
    return out


def _bosses() -> tuple[cq.Workplane, cq.Workplane]:
    """Üst ↔ alt kabuk vida kuleleri: gövde iç boşluğu boyunca tabandan omuza; (üst, alt) parçalar.
    Alt kulede Ø2,8 geçiş deliği (vida alttan), üst kulede Ø2,2 pilot (M2,5 kendinden kılavuzlu)."""
    inner = _loft(V.WALL)
    top = bottom = None
    for x, y0 in V.BOSSES:
        for y in (y0, -y0):
            col = cq.Workplane("XY").workplane(offset=-120.0).center(x, y).circle(V.BOSS_D / 2).extrude(240.0)
            col = col.intersect(inner)
            t_ = (col.intersect(_xbox(-BIG, BIG, z0=V.Z_SPLIT))
                  .cut(cq.Workplane("XY").workplane(offset=V.Z_SPLIT).center(x, y).circle(1.1).extrude(12.0)))
            b_ = (col.intersect(_xbox(-BIG, BIG, z1=V.Z_SPLIT))
                  .cut(cq.Workplane("XY").workplane(offset=-120.0).center(x, y).circle(1.4).extrude(240.0)))
            top = t_ if top is None else top.union(t_)
            bottom = b_ if bottom is None else bottom.union(b_)
    return top, bottom


def _rails() -> cq.Workplane:
    x0, x1, y0, y1, z0, z1 = battery_box(clear=V.BATTERY_CLEAR)
    floor = min(V.section_at(x0 + (x1 - x0) * k / 8, V.WALL)[0] for k in range(9))     # tüm boyunca tabana otursun
    out = None
    for y in (y0 - 0.6, y1 + 0.6):
        r = (cq.Workplane("XY").box(x1 - x0, V.RIB_RATIO * V.WALL, z0 - floor + 6.0, centered=(False, True, False))
             .translate((x0, y, floor - 1.0)))
        out = r if out is None else out.union(r)
    return out


def _vents(shell_: cq.Workplane) -> cq.Workplane:
    """Yan havalandırma yarıkları (Pi bölgesi) ve kanopi önünde emiş yarıkları."""
    for side in (-1, 1):
        for k in range(6):
            x = 18.0 + 8.0 * k
            cutter = (cq.Workplane("XZ").workplane(offset=-side * 60.0).center(x, -28.0)
                      .slot2D(16.0, 3.4, angle=90).extrude(side * 30.0))
            shell_ = shell_.cut(cutter)
    for k in range(5):
        shell_ = shell_.cut(cq.Workplane("XY").workplane(offset=20.0).center(46.0 + 5.0 * k, 0)
                            .slot2D(26.0, 2.6, angle=90).extrude(40.0))
    return shell_


def body_parts_detailed() -> dict[str, cq.Workplane]:
    parts = body_parts()
    inner = _loft(V.WALL)
    sleeves = _sleeves().intersect(inner)                            # kabuk içinde kalsın
    boss_top, boss_bottom = _bosses()
    top = parts["top_shell"].union(sleeves.intersect(_xbox(-BIG, V.NOSE_SPLIT_X, z0=V.Z_SPLIT))).union(boss_top)
    bottom = parts["bottom_tub"].union(sleeves.intersect(_xbox(-BIG, V.NOSE_SPLIT_X, z1=V.Z_SPLIT))).union(boss_bottom)
    bottom = bottom.union(_rails().intersect(_loft(V.WALL - 0.3)))             # raylar kabuk etine gömülür
    top = _vents(top)
    bottom = _vents(bottom)
    # Kol kökleri soket ağızlarından geçer: soketleri tekrar aç
    cut = _arm_root_cut()
    return {"top_shell": top.cut(cut), "bottom_tub": bottom.cut(cut), "nose_cover": parts["nose_cover"],
            "nose_chin": parts["nose_chin"]}
