"""DC7 v2 ön gimbal (CadQuery): kamera burnun önünde, gövdenin orta hattında ve orta yüksekliğinde; aşağı sarkmaz.

Gövdeden kameraya:
  * gimbal_bracket — sönümlü U taşıyıcı (burnun içinde, görünmez): arka plakası 4 sönümleyiciyle burun perdesine
                     bağlanır; yan plakaları yanakların içinde pitch motorunu (+y) ve rulmanı (−y) taşır.
  * gimbal_frame   — pitch ile dönen U çerçeve: arka plakasında roll motoru, kolları pitch eksenine iner.
  * roll motoru    — kameranın tam arkasında, optik eksenle eş eksenli: kamera kendi ekseni etrafında döner. Ön
                     görünüş simetriktir ve roll ekseninde karşı ağırlık gerekmez.
  * gimbal_cradle  — roll rotoruna bağlı beşik plakası; CM3 kartı ara parçalarla önünde.
  * kamera başlığı — beşiğe takılan, öne doğru daralan PC kapak + mercek halkası + cam.
Pitch ekseni dönen kapsülün (çerçeve + roll motoru + beşik + başlık + kamera) ağırlık merkezinden geçer
(pitch_axis_x): motorlar yük tutmak için değil, yalnızca ivmelenme için tork harcar.

Eksen sırası (dışta pitch, içte roll): kamera θ kadar aşağı baktığında gövde roll'ünün sin θ kadarı mekanik olarak
düzeltilemez ve görüntüde yatay kayma (pan) olarak kalır. Bu kayma EIS ile, takipte de gövde yaw'ı ile giderilir
(docs/10 §3.4). θ = 0'da düzeltme tamdır. v1'in sırası (dışta roll) bu kaybı yaşamaz, ama roll'de pitch motoru
kameranın çevresinde ±30° döner; bu süpürme kanallar arasındaki koridora sığmaz (docs/12 §3.2).

Gimbal koordinatı (v1 ile aynı): orijin kamera kartının ön yüzü (optik eksen üzerinde); x ileri, y sol, z yukarı.
"""
from __future__ import annotations

import math
import sys
from functools import lru_cache
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

import airframe as A  # noqa: E402  (mass_props: hassas hacim integrali)
import gimbal_2axis as G  # noqa: E402  (kamera temsili ve yardımcılar)
import params as P  # noqa: E402
import v2_params as V  # noqa: E402

T = P.GIMBAL_PART_T
GAP = 0.5                                             # motor ↔ plaka
MATERIAL = "PA6-GF30"                                  # çerçeve, beşik, taşıyıcı: enjeksiyon (prototip: MJF PA12 / PA-CF)
R_MOTOR = P.GIMBAL_MOTOR_D / 2
# --- kapsül (pitch ile döner) ------------------------------------------------------------------
HEAD_BACK, HEAD_FRONT, HEAD_R = (30.0, 28.0), (21.0, 20.0), 5.0    # kamera başlığı kesiti (y × z), köşe yarıçapı
HEAD_X = (-3.0, 1.0, 8.5)                             # arka kenar, düz bölümün sonu, ön yüz (ön yüze doğru daralır)
HEAD_WALL = 1.0
CRADLE_X = (-6.0, -3.0)
ROLL_X = (CRADLE_X[0] - GAP - P.GIMBAL_MOTOR_H, CRADLE_X[0] - GAP)    # roll motoru (rotor önde, beşikte)
FRAME_X = (ROLL_X[0] - GAP - T, ROLL_X[0] - GAP)                     # çerçeve arka plakası (roll statoru)
FRAME_DISC_R = 12.0                                   # stator deseni (Ø16) + et
ARM_Y = (19.5, 19.5 + T)                              # çerçeve kolları: başlığın roll süpürmesinin (18,4 mm) dışında
ARM_HALF_Z = 8.0
PITCH_BOSS_R = 10.5                                   # pitch rotor deseni (Ø16) + et
PITCH_MOTOR_Y = (ARM_Y[1] + GAP, ARM_Y[1] + GAP + P.GIMBAL_MOTOR_H)
# --- taşıyıcı (gövdeye sönümleyicilerle bağlı, sabit) ---------------------------------------
BRACKET_Y = (PITCH_MOTOR_Y[1] + GAP, PITCH_MOTOR_Y[1] + GAP + T)     # +y: pitch statoru
BEARING = (5.0, 11.0, 5.0)                            # 685ZZ: iç Ø, dış Ø, genişlik
BEARING_Y = (-(V.MOUTH_HALF_Y + V.MOUTH_WALL + V.BRACKET_GAP + 0.25 + BEARING[2]),
             -(V.MOUTH_HALF_Y + V.MOUTH_WALL + V.BRACKET_GAP + 0.25))
BEARING_BOSS = (BEARING_Y[0] - 0.25, BEARING_Y[1] + 0.25, BEARING[1] / 2 + 2.5)   # y0, y1, yarıçap
SIDE_HALF_Z = 11.5                                    # yan plakaların yarı yüksekliği
DAMPER_PLATE = (V.GIMBAL_DAMPERS[0] / 2 + 6.0, V.GIMBAL_DAMPERS[1] / 2 + 6.0)   # arka plaka yarı genişlik, yarı yükseklik
# Kütleler (g): temsili parçalar
PIN_G, BEARING_G, DAMPER_G = 1.5, 1.3, 1.0
IMU_G = 2.0                                            # kamera IMU kartı (başlığın içinde, kartın arkasında)


def _rr(w: float, h: float, r: float) -> cq.Sketch:
    return cq.Sketch().rect(w, h).vertices().fillet(min(r, w / 2 - 0.1, h / 2 - 0.1))


def _loft_x(sections: list[tuple[float, float, float, float]]) -> cq.Workplane:
    """x boyunca köşeleri yuvarlatılmış dikdörtgen kesitlerden doğrusal loft: (x, genişlik_y, yükseklik_z, r)."""
    return cq.Workplane("YZ").placeSketch(
        *[_rr(w, h, r).moved(cq.Location(cq.Vector(0, 0, x))) for x, w, h, r in sections]).loft(ruled=True)


def _plate_xz(y0: float, y1: float, sketch: cq.Sketch) -> cq.Workplane:
    """x-z düzleminde çizim, y0 → y1 kalınlığında (çizim koordinatı: (x, z))."""
    return cq.Workplane("XZ").workplane(offset=-y0).placeSketch(sketch).extrude(-(y1 - y0))


def _cyl_y(cx: float, cz: float, y0: float, y1: float, r: float) -> cq.Workplane:
    return G._cyl_y(cx, cz, y0, y1, r)


def _cyl_x(cy: float, cz: float, x0: float, x1: float, r: float) -> cq.Workplane:
    return G._cyl_x(cy, cz, x0, x1, r)


def _bolt_circle(d: float) -> list[tuple[float, float]]:
    return G._circle_pts(d)


# --- kapsül parçaları --------------------------------------------------------------------------
def camera() -> cq.Workplane:
    """Kamera kartı + mercek bloğu tek katı (çakışma kontrolleri için)."""
    return G.camera()


def camera_parts() -> list[tuple[str, cq.Workplane]]:
    """CM3 temsili (render için): kart, mercek bloğu ve namlusu, mercek camı."""
    w, h, t = P.CAM_BOARD
    lw, lh, ld = P.CAM_LENS
    return [("camera_pcb", G._box(-t, 0.0, -w / 2, w / 2, -h / 2, h / 2)),
            ("camera_lens", G._box(0.0, ld - 2.0, -lw / 2, lw / 2, -lh / 2, lh / 2)),
            ("camera_lens", G._cyl_x(0.0, 0.0, ld - 2.0, ld, 3.6)),
            ("camera_glass", G._cyl_x(0.0, 0.0, ld, ld + 0.3, 2.6))]


def head() -> dict[str, cq.Workplane]:
    """Kamera başlığı: kartı ve mercek bloğunu örten, öne doğru daralan PC kapak (1 mm), mercek halkası, cam.
    Arka kesiti beşikle aynıdır; daralan ön yüz pitch süpürme yarıçapını küçültür (ağız alçak kalır)."""
    (wb, hb), (wf, hf), r = HEAD_BACK, HEAD_FRONT, HEAD_R
    x0, xm, x1 = HEAD_X
    t = HEAD_WALL
    outer = _loft_x([(x0, wb, hb, r), (xm, wb, hb, r), (x1, wf, hf, r)])
    inner = _loft_x([(x0 - 1.0, wb - 2 * t, hb - 2 * t, r - t), (xm, wb - 2 * t, hb - 2 * t, r - t),
                     (x1 - t, wf - 2 * t, hf - 2 * t, r - t)])
    lens = cq.Workplane("YZ").workplane(offset=x1 - t - 0.5).circle(5.0).extrude(t + 1.0)
    housing = outer.cut(inner).cut(lens)
    bezel = (cq.Workplane("YZ").workplane(offset=x1 - 0.2).circle(7.0).circle(5.0).extrude(1.7)
             .faces(">X").edges().chamfer(0.4))
    glass = cq.Workplane("YZ").workplane(offset=x1 - 0.6).circle(5.0).extrude(0.6)
    return {"camera_housing": housing, "camera_bezel": bezel, "camera_glass": glass}


def cradle() -> cq.Workplane:
    """Beşik: başlığın arka kapağı. Önde CM3 kartı için 4 ara parça, arkada roll rotoru deseni (Ø16)."""
    w, h, t = P.CAM_BOARD
    x0, x1 = CRADLE_X
    part = cq.Workplane("YZ").workplane(offset=x0).placeSketch(_rr(*HEAD_BACK, HEAD_R)).extrude(x1 - x0)
    hw, hh = P.CAM_HOLES
    top = h / 2 - P.CAM_HOLE_OFFSET
    pts = [(hw / 2, top), (-hw / 2, top), (hw / 2, top - hh), (-hw / 2, top - hh)]
    for y, z in pts:
        part = part.union(_cyl_x(y, z, x1 - 0.1, -t, 2.2))
        part = part.cut(_cyl_x(y, z, x0 - 1.0, 0.0, P.M2 / 2))
    for y, z in _bolt_circle(P.GIMBAL_MOTOR_HOLE_CIRCLE):
        part = part.cut(_cyl_x(y, z, x0 - 1.0, x1 + 1.0, P.M2 / 2))
    return part.cut(_cyl_x(0.0, 0.0, x0 - 1.0, x1 + 1.0, 4.0))          # FFC / kablo geçişi


def roll_motor() -> cq.Workplane:
    return _cyl_x(0.0, 0.0, ROLL_X[0], ROLL_X[1], R_MOTOR)


def frame(xp: float) -> cq.Workplane:
    """U çerçeve: arka plaka (roll statoru) + iki kol (pitch eksenine; +y kolunda pitch rotoru deseni,
    −y kolunda rulman pimi)."""
    x0, x1 = FRAME_X
    back = (cq.Workplane("YZ").workplane(offset=x0)
            .placeSketch(cq.Sketch().circle(FRAME_DISC_R).reset()
                         .push([(0, 0)]).rect(2 * ARM_Y[1], 2 * ARM_HALF_Z).reset().clean()
                         .reset().vertices().fillet(2.0))
            .extrude(x1 - x0))
    for y, z in _bolt_circle(P.GIMBAL_MOTOR_HOLE_CIRCLE):
        back = back.cut(_cyl_x(y, z, x0 - 1.0, x1 + 1.0, P.M2 / 2))
    back = back.cut(_cyl_x(0.0, 0.0, x0 - 1.0, x1 + 1.0, 4.0))
    arm_shape = (cq.Sketch().push([((x0 + xp) / 2, 0.0)]).rect(xp - x0, 2 * ARM_HALF_Z)
                 .push([(xp, 0.0)]).circle(PITCH_BOSS_R).clean())
    part = back
    for sy in (-1, 1):
        y0, y1 = (ARM_Y[0], ARM_Y[1]) if sy > 0 else (-ARM_Y[1], -ARM_Y[0])
        part = part.union(_plate_xz(y0, y1, arm_shape))
    for x, z in _bolt_circle(P.GIMBAL_MOTOR_HOLE_CIRCLE):
        part = part.cut(_cyl_y(xp + x, z, ARM_Y[0] - 1.0, ARM_Y[1] + 1.0, P.M2 / 2))
    part = part.cut(_cyl_y(xp, 0.0, ARM_Y[0] - 1.0, ARM_Y[1] + 1.0, 4.0))
    return part.cut(_cyl_y(xp, 0.0, -ARM_Y[1] - 1.0, -ARM_Y[0] + 1.0, BEARING[0] / 2))      # pim yuvası


def pitch_motor(xp: float) -> cq.Workplane:
    return _cyl_y(xp, 0.0, PITCH_MOTOR_Y[0], PITCH_MOTOR_Y[1], R_MOTOR)


def pin(xp: float) -> cq.Workplane:
    return _cyl_y(xp, 0.0, BEARING_Y[0], -ARM_Y[0], BEARING[0] / 2)


def bearing(xp: float) -> cq.Workplane:
    return (_cyl_y(xp, 0.0, BEARING_Y[0], BEARING_Y[1], BEARING[1] / 2)
            .cut(_cyl_y(xp, 0.0, BEARING_Y[0] - 1.0, BEARING_Y[1] + 1.0, BEARING[0] / 2)))


# --- taşıyıcı ----------------------------------------------------------------------------------
def bracket_x(xp: float) -> tuple[float, float]:
    """Taşıyıcı arka plakasının x aralığı: ağzın arka astarının (R_MOUTH + et) arkasında, BRACKET_GAP boşlukla."""
    x1 = xp - V.MOUTH_R - V.MOUTH_WALL - V.BRACKET_GAP
    return x1 - T, x1


def bracket(xp: float) -> cq.Workplane:
    """Sönümlü U taşıyıcı: arka plaka (4 sönümleyici, ağzın arkasında) + yan plakalar (yanakların içinde)."""
    xb0, xb1 = bracket_x(xp)
    hy, hz = DAMPER_PLATE
    yl, yr = BEARING_BOSS[0], BRACKET_Y[1]
    sk = (cq.Sketch().push([(0.0, 0.0)]).rect(2 * hy, 2 * hz)
          .push([((yl + yr) / 2, 0.0)]).rect(yr - yl, 2 * SIDE_HALF_Z).reset().clean().reset().vertices().fillet(3.0))
    back = cq.Workplane("YZ").workplane(offset=xb0).placeSketch(sk).extrude(T)
    dy, dz = V.GIMBAL_DAMPERS
    for sy in (-1, 1):
        for sz in (-1, 1):
            back = back.cut(_cyl_x(sy * dy / 2, sz * dz / 2, xb0 - 1.0, xb1 + 1.0, P.M3 / 2))
    back = back.cut(_cyl_x(0.0, 0.0, xb0 - 1.0, xb1 + 1.0, 7.0))          # kablo geçişi (FFC + motor kabloları)
    side = (cq.Sketch().push([((xb0 + xp) / 2, 0.0)]).rect(xp - xb0, 2 * SIDE_HALF_Z)
            .push([(xp, 0.0)]).circle(SIDE_HALF_Z).clean())
    plus = _plate_xz(BRACKET_Y[0], BRACKET_Y[1], side)
    for x, z in _bolt_circle(P.GIMBAL_MOTOR_HOLE_CIRCLE):
        plus = plus.cut(_cyl_y(xp + x, z, BRACKET_Y[0] - 1.0, BRACKET_Y[1] + 1.0, P.M2 / 2))
    plus = plus.cut(_cyl_y(xp, 0.0, BRACKET_Y[0] - 1.0, BRACKET_Y[1] + 1.0, 4.0))
    by0, by1, br = BEARING_BOSS
    minus = _plate_xz(by0, by0 + T, side).union(_cyl_y(xp, 0.0, by0, by1, br))
    minus = minus.cut(_cyl_y(xp, 0.0, BEARING_Y[0], BEARING_Y[1], BEARING[1] / 2))      # rulman yuvası
    minus = minus.cut(_cyl_y(xp, 0.0, by0 - 1.0, by1 + 1.0, BEARING[0] / 2 + 0.5))
    return back.union(plus).union(minus)


def dampers(xp: float) -> list[cq.Workplane]:
    """4 kauçuk sönümleyici (eksenleri x yönünde): taşıyıcı arka plakası ↔ burun perdesi."""
    xb0 = bracket_x(xp)[0]
    rad = P.DAMPER_H / 2 + 0.3
    dy, dz = V.GIMBAL_DAMPERS
    return [cq.Workplane("XY").sphere(rad).translate((xb0 - P.DAMPER_H / 2, sy * dy / 2, sz * dz / 2))
            for sy in (-1, 1) for sz in (-1, 1)]


def bulkhead_x(xp: float) -> float:
    """Sönümleyicilerin bağlandığı burun perdesinin ön yüzü (gimbal koordinatı)."""
    return bracket_x(xp)[0] - P.DAMPER_H


# --- denge -------------------------------------------------------------------------------------
def _mass(shape: cq.Workplane, density: float) -> tuple[float, tuple[float, float, float]]:
    vol, c = A.mass_props(shape)
    return vol / 1000.0 * density, c


def capsule_masses(xp: float) -> list[tuple[str, float, tuple[float, float, float]]]:
    """Pitch ile dönen kapsülün kütleleri (g) ve ağırlık merkezleri. Pitch rotoru ve pim eksen üzerindedir."""
    rho = V.DENSITY[MATERIAL]
    h = head()
    out = [("kamera", P.CAM_MASS_G, (G.CAM_CG_X, 0.0, 0.0)), ("kamera IMU", IMU_G, (-2.0, 0.0, 0.0)),
           ("roll motoru", P.GIMBAL_MOTOR_MASS_G, (sum(ROLL_X) / 2, 0.0, 0.0))]
    out += [(n, *_mass(s, rho)) for n, s in (("gimbal_cradle", cradle()), ("gimbal_frame", frame(xp)))]
    out += [(n, *_mass(s, d)) for n, s, d in (("camera_housing", h["camera_housing"], V.DENSITY["PC"]),
                                              ("camera_bezel", h["camera_bezel"], 2.70),
                                              ("camera_glass", h["camera_glass"], 1.19))]
    return out


@lru_cache(maxsize=1)
def pitch_axis_x(iterations: int = 4) -> float:
    """Pitch eksenini kapsülün ağırlık merkezine koyar (x); çerçeve kolları eksene uzandığı için yinelemeli."""
    xp = -10.0
    for _ in range(iterations):
        items = capsule_masses(xp)
        xp = sum(m * c[0] for _, m, c in items) / sum(m for _, m, _ in items)
    return round(xp, 2)


def roll_balance() -> tuple[float, float]:
    """Roll ile dönen grubun (kamera + IMU + beşik + başlık; rotor eksenel simetrik) ağırlık merkezinin optik
    eksenden y, z kayması (mm). Simetrik tasarımda ≈ 0: karşı ağırlık gerekmez."""
    items = [i for i in capsule_masses(pitch_axis_x()) if i[0] not in ("roll motoru", "gimbal_frame")]
    m = sum(mi for _, mi, _ in items)
    return (sum(mi * c[1] for _, mi, c in items) / m, sum(mi * c[2] for _, mi, c in items) / m)


# --- hareket -----------------------------------------------------------------------------------
def roll_group(detailed: bool = True) -> list[tuple[str, cq.Workplane]]:
    cam = camera_parts() if detailed else [("camera_pcb", camera())]
    return cam + [("gimbal_cradle", cradle())] + list(head().items())


def posed(pitch: float = 0.0, roll: float = 0.0, detailed: bool = True) -> list[tuple[str, cq.Workplane]]:
    """Gimbal koordinatında poz: roll (optik eksen etrafında) sonra pitch (+ = burun yukarı) uygulanır."""
    xp = pitch_axis_x()
    rolled = [(n, s.rotate((0, 0, 0), (1, 0, 0), roll)) for n, s in roll_group(detailed)]
    moving = rolled + [("gimbal_motor", roll_motor()), ("gimbal_frame", frame(xp))]
    moving = [(n, s.rotate((xp, 0, 0), (xp, 1, 0), -pitch)) for n, s in moving]
    static = [("gimbal_motor", pitch_motor(xp)), ("gimbal_pin", pin(xp)), ("gimbal_bearing", bearing(xp)),
              ("gimbal_bracket", bracket(xp))] + [("damper", d) for d in dampers(xp)]
    return moving + static


def placed(pitch: float = 0.0, roll: float = 0.0, detailed: bool = True) -> list[tuple[str, cq.Workplane]]:
    """Gövde koordinatında (kamera merkezi V.GIMBAL_POS)."""
    return [(n, s.translate(V.GIMBAL_POS)) for n, s in posed(pitch, roll, detailed)]


def capsule(pitch: float = 0.0, roll: float = 0.0) -> cq.Workplane:
    """Dönen kapsülün tek katısı (çakışma ve süpürme kontrolleri için), gövde koordinatında."""
    items = [s for n, s in posed(pitch, roll, detailed=False)
             if n not in ("gimbal_bracket", "damper", "gimbal_pin", "gimbal_bearing")
             and not (n == "gimbal_motor" and s.val().BoundingBox().ymin > ARM_Y[1])]
    out = items[0]
    for s in items[1:]:
        out = out.union(s)
    return out.translate(V.GIMBAL_POS)


def sweep_extent(poses: list[tuple[float, float]], step: float = 10.0) -> dict[str, float]:
    """Kapsülün verilen pozlardaki en uç noktaları (pitch eksenine göre, x-z düzleminde): ağız boyutu için.
    r_max destek fonksiyonuyla bulunur: kapsül pitch ekseni etrafında `step` aralıkla döndürülür ve sınır kutusunun
    eksenden uzaklığı ölçülür (eğri yüzeylerin uç noktaları da yakalanır; açı hatası ≤ r·(1 − cos(step/2)))."""
    xp = pitch_axis_x()
    out = {"r_max": 0.0, "z_max": -1e9, "z_min": 1e9}
    for pitch, roll in poses:
        shape = capsule(pitch, roll).translate(tuple(-c for c in V.GIMBAL_POS))
        bb = shape.val().BoundingBox()
        out["z_max"] = max(out["z_max"], bb.zmax)
        out["z_min"] = min(out["z_min"], bb.zmin)
        for k in range(int(round(360.0 / step))):
            rb = shape.rotate((xp, 0, 0), (xp, 1, 0), k * step).val().BoundingBox()
            out["r_max"] = max(out["r_max"], rb.xmax - xp)
    return out


def controller_box() -> tuple[float, float, float, float, float, float]:
    """Gimbal kontrolcüsü (gövde koordinatı, v2_params.GIMBAL_CTRL): uçuş kontrolcüsünün üstünde."""
    (cx, cy, cz), (lx, ly, lz) = V.GIMBAL_CTRL["center"], V.GIMBAL_CTRL["size"]
    return (cx - lx / 2, cx + lx / 2, cy - ly / 2, cy + ly / 2, cz - lz / 2, cz + lz / 2)


def hardware_masses() -> list[tuple[str, float, tuple[float, float, float]]]:
    """Kamera hariç tüm gimbal donanımı (gövde koordinatı): profildeki "fırçasız gimbal" kütlesi ve ağırlık merkezi."""
    xp = pitch_axis_x()
    g = V.GIMBAL_POS

    def body(c: tuple[float, float, float]) -> tuple[float, float, float]:
        return (c[0] + g[0], c[1] + g[1], c[2] + g[2])
    items = [(n, m, body(c)) for n, m, c in capsule_masses(xp) if n != "kamera"]
    items.append(("pitch motoru", P.GIMBAL_MOTOR_MASS_G, body((xp, sum(PITCH_MOTOR_Y) / 2, 0.0))))
    items.append(("pim + rulman", PIN_G + BEARING_G, body((xp, BEARING_Y[0], 0.0))))
    m, c = _mass(bracket(xp), V.DENSITY[MATERIAL])
    items.append(("gimbal_bracket", m, body(c)))
    items.append(("sönümleyiciler", 4 * DAMPER_G, body((bracket_x(xp)[0] - P.DAMPER_H / 2, 0.0, 0.0))))
    items.append(("kontrolcü", V.GIMBAL_CTRL["g"], V.GIMBAL_CTRL["center"]))
    return items


def hardware_cg(controller: bool = True) -> tuple[tuple[float, float, float], float]:
    """Gimbal donanımının ağırlık merkezi ve kütlesi (kamera hariç); controller=False: kontrolcü kartı da hariç
    (yerleşimde ayrı bileşendir)."""
    items = [i for i in hardware_masses() if controller or i[0] != "kontrolcü"]
    m = sum(mi for _, mi, _ in items)
    return tuple(round(sum(mi * c[i] for _, mi, c in items) / m, 1) for i in range(3)), round(m, 1)


def printed_parts() -> dict[str, cq.Workplane]:
    """Üretilen gimbal parçaları (gimbal koordinatında)."""
    xp = pitch_axis_x()
    return {"gimbal_cradle": cradle(), "gimbal_frame": frame(xp), "gimbal_bracket": bracket(xp),
            "camera_housing": head()["camera_housing"]}
