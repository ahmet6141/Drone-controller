#!/usr/bin/env python3
"""8×8 çok bölgeli ToF (VL53L8CX / VL53L7CX / VL53L5CX) karesinden avuç analizi.

Sensör, görüş alanını 8×8 = 64 bölgeye böler ve her bölge için ayrı bir mesafe verir
(çok düşük çözünürlüklü bir derinlik kamerası gibi). Bu modül bir kareden:
  * her bölgenin 3B noktasını (sensör çerçevesi: x sağ, y ileri, z optik eksen/aşağı),
  * zeminden belirgin şekilde yakın, bitişik bölge kümesini (avuç),
  * avuç merkezinin optik eksenden yatay sapmasını, mesafesini,
  * düzlem uydurma ile eğimini ve düzlük artığını,
  * temas kararını (merkez bölgeler temas eşiğinin altında mı)
hesaplar. Saf Python'dur; Faz 1'de dc7_palm_landing düğümüne aynen taşınır.

Kullanım:
    python3 tools/tof8x8.py --demo
    python3 tools/tof8x8.py --demo --palm-distance 0.15 --tilt 20
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field

N = 8                       # 8×8 bölge
FOV_DEG = {"vl53l8cx": 45.0, "vl53l5cx": 45.0, "vl53l7cx": 60.0}  # yatay = dikey FOV


@dataclass
class PalmEstimate:
    found: bool
    reason: str
    zones: int = 0
    distance_m: float | None = None
    offset_x_m: float | None = None      # + sağ
    offset_y_m: float | None = None      # + ileri
    tilt_deg: float | None = None
    flatness_rms_m: float | None = None
    contact: bool = False
    mask: list[list[bool]] = field(default_factory=lambda: [[False] * N for _ in range(N)])


def zone_tangents(fov_deg: float) -> list[float]:
    """Bölge merkez ışınlarının tan(açı) değerleri (sütun/satır 0..7)."""
    step = math.radians(fov_deg) / N
    return [math.tan((i - (N - 1) / 2) * step) for i in range(N)]


def to_points(frame_mm, fov_deg: float = 45.0, radial: bool = True):
    """8×8 mesafe karesini (mm, geçersiz bölge None) 3B noktalara çevirir.

    radial=True : sürücü ışın boyunca mesafe veriyorsa (fiziksel uçuş yolu)
    radial=False: sürücü optik eksene dik (Z) mesafe veriyorsa
    Hangisinin geçerli olduğu kullanılan sürücünün belgesinden doğrulanmalıdır.
    """
    tan = zone_tangents(fov_deg)
    pts = [[None] * N for _ in range(N)]
    for r in range(N):
        for c in range(N):
            d = frame_mm[r][c]
            if d is None or d <= 0:
                continue
            d /= 1000.0
            tx, ty = tan[c], tan[r]
            z = d / math.sqrt(tx * tx + ty * ty + 1.0) if radial else d
            pts[r][c] = (z * tx, z * ty, z)
    return pts


def _solve3(m, v):
    """3×3 doğrusal sistem (Cramer); tekilse None."""
    def det(a):
        return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
                - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    d = det(m)
    if abs(d) < 1e-12:
        return None
    out = []
    for i in range(3):
        mi = [row[:] for row in m]
        for r in range(3):
            mi[r][i] = v[r]
        out.append(det(mi) / d)
    return out


def fit_plane(points):
    """z = a·x + b·y + c en küçük kareler; (a, b, c, rms) veya None."""
    if len(points) < 3:
        return None
    sxx = sum(p[0] * p[0] for p in points)
    syy = sum(p[1] * p[1] for p in points)
    sxy = sum(p[0] * p[1] for p in points)
    sx = sum(p[0] for p in points)
    sy = sum(p[1] for p in points)
    sxz = sum(p[0] * p[2] for p in points)
    syz = sum(p[1] * p[2] for p in points)
    sz = sum(p[2] for p in points)
    sol = _solve3([[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, len(points)]], [sxz, syz, sz])
    if sol is None:
        return None
    a, b, c = sol
    rms = math.sqrt(sum((p[2] - (a * p[0] + b * p[1] + c)) ** 2 for p in points) / len(points))
    return a, b, c, rms


def analyze(frame_mm, *, fov_deg: float = 45.0, radial: bool = True,
            ground_m: float | None = None, band_m: float = 0.06,
            min_separation_m: float = 0.15, contact_m: float = 0.035,
            max_palm_distance_m: float = 1.0) -> PalmEstimate:
    """Tek kareden avuç tespiti.

    1. En yakın geçerli bölgeden başlayıp `band_m` kalınlığındaki bitişik bölgeleri topla (küme).
    2. Küme, geri kalan bölgelerden (veya bilinen zemin mesafesinden) en az `min_separation_m`
       yakın olmalı; aksi hâlde gördüğümüz şey zemindir.
    3. Kümenin merkezi, mesafesi, eğimi ve düzlüğü; merkez 2×2 bölgeden temas kararı.
    """
    pts = to_points(frame_mm, fov_deg, radial)
    valid = [(r, c) for r in range(N) for c in range(N) if pts[r][c] is not None]
    if len(valid) < 4:
        return PalmEstimate(False, "geçerli bölge yetersiz")

    r0, c0 = min(valid, key=lambda rc: pts[rc[0]][rc[1]][2])
    z_min = pts[r0][c0][2]
    if z_min > max_palm_distance_m:
        return PalmEstimate(False, "yakında yüzey yok")

    # 4-komşuluk ile bitişik yakın bölge kümesi
    mask = [[False] * N for _ in range(N)]
    stack = [(r0, c0)]
    while stack:
        r, c = stack.pop()
        if mask[r][c] or pts[r][c] is None or pts[r][c][2] > z_min + band_m:
            continue
        mask[r][c] = True
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            rr, cc = r + dr, c + dc
            if 0 <= rr < N and 0 <= cc < N:
                stack.append((rr, cc))

    cluster = [pts[r][c] for r, c in valid if mask[r][c]]
    rest = sorted(pts[r][c][2] for r, c in valid if not mask[r][c])
    z_palm = sorted(p[2] for p in cluster)[len(cluster) // 2]

    if rest:
        background = rest[len(rest) // 2]
    elif ground_m is not None:
        background = ground_m
    else:
        return PalmEstimate(False, "tüm görüş tek yüzey ve zemin mesafesi bilinmiyor", len(cluster))
    if background - z_palm < min_separation_m:
        return PalmEstimate(False, "yakın yüzey zeminden ayrışmıyor (zemin)", len(cluster))

    plane = fit_plane(cluster)
    centre = [(r, c) for r in (N // 2 - 1, N // 2) for c in (N // 2 - 1, N // 2)]
    near = sum(1 for r, c in centre if pts[r][c] is not None and pts[r][c][2] <= contact_m)
    return PalmEstimate(
        found=True,
        reason="avuç",
        zones=len(cluster),
        distance_m=z_palm,
        offset_x_m=sum(p[0] for p in cluster) / len(cluster),
        offset_y_m=sum(p[1] for p in cluster) / len(cluster),
        tilt_deg=None if plane is None else math.degrees(math.atan(math.hypot(plane[0], plane[1]))),
        flatness_rms_m=None if plane is None else plane[3],
        contact=near >= 3,
        mask=mask,
    )


def synthetic_frame(*, palm_distance_m: float = 0.40, ground_m: float = 1.10,
                    palm_center_m: tuple[float, float] = (0.0, 0.0),
                    palm_size_m: tuple[float, float] = (0.09, 0.10), tilt_deg: float = 0.0,
                    fov_deg: float = 45.0, radial: bool = True, with_palm: bool = True):
    """Test/demo için 8×8 kare: zemin üstünde (x eksenine göre eğik) dikdörtgen avuç.

    Her bölge yalnızca merkez ışınıyla örneklenir (gerçek sensör bölge konisindeki baskın hedefi verir).
    """
    tan = zone_tangents(fov_deg)
    k = math.tan(math.radians(tilt_deg))
    cx, cy = palm_center_m
    w, h = palm_size_m
    frame = []
    for r in range(N):
        row = []
        for c in range(N):
            tx, ty = tan[c], tan[r]
            t = ground_m
            if with_palm:
                tp = (palm_distance_m - k * cx) / (1.0 - k * tx)   # z = z0 + k·(x − cx) düzlemi
                if 0 < tp < ground_m and abs(tp * tx - cx) <= w / 2 and abs(tp * ty - cy) <= h / 2:
                    t = tp
            d = t * math.sqrt(tx * tx + ty * ty + 1.0) if radial else t
            row.append(int(round(d * 1000)))
        frame.append(row)
    return frame


def render(frame_mm, mask=None) -> str:
    lines = ["      " + " ".join(f"  s{c} " for c in range(N))]
    for r in range(N):
        cells = []
        for c in range(N):
            v = frame_mm[r][c]
            cell = "   --" if v is None else f"{v:5d}"
            cells.append(("[" if mask and mask[r][c] else " ") + cell)
        lines.append(f"  s{r}  " + "".join(cells))
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="8×8 ToF avuç analizi demosu")
    ap.add_argument("--demo", action="store_true", help="sentetik kare üret ve analiz et")
    ap.add_argument("--sensor", choices=sorted(FOV_DEG), default="vl53l8cx")
    ap.add_argument("--palm-distance", type=float, default=0.40)
    ap.add_argument("--ground", type=float, default=1.10)
    ap.add_argument("--offset-x", type=float, default=0.0)
    ap.add_argument("--tilt", type=float, default=0.0)
    ap.add_argument("--no-palm", action="store_true")
    args = ap.parse_args(argv)
    if not args.demo:
        ap.error("şimdilik yalnızca --demo destekleniyor")

    fov = FOV_DEG[args.sensor]
    frame = synthetic_frame(palm_distance_m=args.palm_distance, ground_m=args.ground,
                            palm_center_m=(args.offset_x, 0.0), tilt_deg=args.tilt,
                            fov_deg=fov, with_palm=not args.no_palm)
    est = analyze(frame, fov_deg=fov, ground_m=args.ground)
    zone_cm = 2 * args.palm_distance * math.tan(math.radians(fov / N) / 2) * 100
    print(f"{args.sensor}: {fov:.0f}°×{fov:.0f}° FOV, bölge başına {fov / N:.2f}°; "
          f"{args.palm_distance:.2f} m'de bir bölge ≈ {zone_cm:.1f} cm")
    print("Mesafeler (mm); [ işaretli bölgeler avuç kümesi:")
    print(render(frame, est.mask))
    if est.found:
        print(f"Avuç: {est.zones} bölge, mesafe {est.distance_m:.3f} m, sapma x={est.offset_x_m:+.3f} m "
              f"y={est.offset_y_m:+.3f} m, eğim {est.tilt_deg:.1f}°, düzlük ±{est.flatness_rms_m * 1000:.1f} mm, "
              f"temas: {'EVET' if est.contact else 'hayır'}")
    else:
        print(f"Avuç yok: {est.reason}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
