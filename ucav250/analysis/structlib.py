"""Hand-calculation library for ucav250 structures (SI units throughout). Aircraft-independent and unit tested.

References (standard aeronautical practice):
* Gust load factor: CS-VLA 341 / CS-23.341 (Pratt formula) — n = 1 ± kg·ρ0·Ude·V·a / (2·W/S),
  kg = 0.88·μ / (5.3 + μ), μ = 2·(W/S) / (ρ·c̄·a·g).
* Schrenk approximation for spanwise lift (average of elliptic and planform-chord distributions).
* Lug static strength: Melcon & Hoblit / Bruhn D1 simplified (net-section tension and shear-out/bearing with
  conservative efficiency factors); fitting factor applied separately.
* Margins of safety: MS = allowable / (FoS · fitting · applied) − 1 (limit loads in, ultimate allowables).
* Classical lamination theory (Jones, Mechanics of Composite Materials, ch. 4): ply stiffness Q, rotated Q̄, ABD
  matrices, ply stresses in material axes, first-ply failure by maximum stress and Tsai-Wu (F12* = −1/2).
* Plate buckling: isotropic SS plates (Bruhn C5, k = 4.0 compression, 5.35 shear for long plates); specially
  orthotropic SS plates in compression (Whitney / Jones eq. 5.x, minimum over the half-wave number) and long plates in
  shear (Kollár & Springer 2003, long-plate fit, isotropic limit 5.35 π² D / b²); sandwich transverse-shear
  correction N = N_b / (1 + N_b / S), S = G_c d² / c (Allen 1969, Zenkert 1995).
* Sandwich local instabilities (Zenkert, An Introduction to Sandwich Construction, 1995; HexWeb design handbook):
  face wrinkling σ = Q (E_f E_c G_c)^(1/3) with the design coefficient Q = 0.5, shear crimping N = G_c d² / c,
  intracell dimpling σ = 2 E_f / (1 − ν²) (t_f / s)² (honeycomb only).
* Columns: Johnson parabola / Euler (Bruhn C2): σ_cr = F_cy − F_cy² (KL/ρ)² / (4 π² E) below the transition
  slenderness π √(2E/F_cy), Euler above.
* Joints: elastic bolt-group method for eccentric in-plane shear and for tension from a moment (Bruhn D1, Niu ch. 9);
  pin bending in double shear with the Melcon-Hoblit moment arm M = P/2 (t_outer/2 + g + t_inner/4); potted-insert
  pull-out by core shear on the effective potting cylinder P = 2 π b_p c τ_c (ECSS-E-HB-32-22A concept, without its
  test-based correction factors).
* Pin-jointed space truss by the direct stiffness method; closed-cell torsion shear flow q = T / (2 A) (Bredt).
* Landing gear energy method (STANAG 4703 Annex B UL.GL.1): n_j = (h + (1 − L) d) / (e_f d), n = n_j + L; descent
  velocity and drop height of CS-LUAS Appendix H.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

RHO0 = 1.225
G0 = 9.80665


# =====================================================================================================================
# flight envelope
# =====================================================================================================================
def gust_factor(ws_npm2: float, cla_per_rad: float, mac: float, rho: float = RHO0) -> float:
    mu = 2.0 * ws_npm2 / (rho * mac * cla_per_rad * G0)
    return 0.88 * mu / (5.3 + mu)


def gust_n(V: float, Ude: float, ws_npm2: float, cla_per_rad: float, mac: float, rho: float = RHO0) -> tuple:
    """(n_up, n_down) for an equivalent airspeed V (m/s) and derived gust velocity Ude (m/s)."""
    kg = gust_factor(ws_npm2, cla_per_rad, mac, rho)
    dn = kg * RHO0 * Ude * V * cla_per_rad / (2.0 * ws_npm2)
    return 1.0 + dn, 1.0 - dn


def vn_diagram(ws_npm2: float, clmax: float, clmin: float, n_pos: float, n_neg: float, VC: float, VD: float,
               cla_per_rad: float, mac: float, Ude_C: float = 15.24, Ude_D: float = 7.62, rho: float = RHO0) -> dict:
    """Manoeuvre + gust envelope (EAS). Returns key speeds and the governing positive/negative load factors."""
    VS = math.sqrt(2 * ws_npm2 / (rho * clmax))
    VA = VS * math.sqrt(n_pos)
    VSneg = math.sqrt(2 * ws_npm2 / (rho * abs(clmin)))
    VG = VSneg * math.sqrt(abs(n_neg))
    gc = gust_n(VC, Ude_C, ws_npm2, cla_per_rad, mac, rho)
    gd = gust_n(VD, Ude_D, ws_npm2, cla_per_rad, mac, rho)
    n_up = max(n_pos, gc[0], gd[0])
    n_dn = min(n_neg, gc[1], gd[1])
    return {"VS": VS, "VA": VA, "VG": VG, "VC": VC, "VD": VD, "gust_C": gc, "gust_D": gd,
            "n_limit_pos": n_up, "n_limit_neg": n_dn, "kg": gust_factor(ws_npm2, cla_per_rad, mac, rho)}


# =====================================================================================================================
# spanwise loads
# =====================================================================================================================
def schrenk(y: np.ndarray, chord: np.ndarray, semispan: float) -> np.ndarray:
    """Normalised spanwise lift per unit span ``l(y)`` with ∫ l dy over the half span = 1 (Schrenk average of
    planform chord and elliptic distributions). ``y`` from root (0) to tip (semispan)."""
    y = np.asarray(y, float)
    c = np.asarray(chord, float)
    area = np.trapz(c, y)
    ell = 4.0 * area / (math.pi * semispan) * np.sqrt(np.clip(1 - (y / semispan) ** 2, 0, None))
    l = 0.5 * (c + ell)
    return l / np.trapz(l, y)


def beam_loads(y: np.ndarray, w: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Shear V(y) and bending M(y) of a cantilever (root at y[0], free tip at y[-1]) under distributed load w(y)
    (N/m, positive up). Returns arrays at the same stations (V positive up outboard of the cut)."""
    y = np.asarray(y, float)
    w = np.asarray(w, float)
    V = np.zeros_like(y)
    M = np.zeros_like(y)
    for i in range(len(y) - 2, -1, -1):
        dy = y[i + 1] - y[i]
        V[i] = V[i + 1] + 0.5 * (w[i] + w[i + 1]) * dy
        M[i] = M[i + 1] + 0.5 * (V[i] + V[i + 1]) * dy
    return V, M


# =====================================================================================================================
# sections
# =====================================================================================================================
def tube(ro: float, ri: float) -> dict:
    A = math.pi * (ro ** 2 - ri ** 2)
    I = math.pi / 4 * (ro ** 4 - ri ** 4)
    return {"A": A, "I": I, "J": 2 * I, "Z": I / ro, "c": ro}


def rect(b: float, h: float) -> dict:
    return {"A": b * h, "I": b * h ** 3 / 12, "Z": b * h ** 2 / 6, "c": h / 2}


def spar_caps(h_eff: float, cap_area: float) -> dict:
    """Idealised I/box spar: two caps of ``cap_area`` separated by ``h_eff`` (centroid distance)."""
    return {"I": 2 * cap_area * (h_eff / 2) ** 2, "c": h_eff / 2}


# =====================================================================================================================
# joints and fittings
# =====================================================================================================================
def ms(allowable: float, applied_limit: float, fos: float = 1.5, fitting: float = 1.0) -> float:
    """Margin of safety for a limit load against an ultimate allowable."""
    if applied_limit <= 0:
        return math.inf
    return allowable / (fos * fitting * applied_limit) - 1.0


def bolt_shear(d: float, Fsu: float, planes: int = 1) -> float:
    return planes * math.pi / 4 * d ** 2 * Fsu


def bearing(d: float, t: float, Fbru: float) -> float:
    return d * t * Fbru


def lug_axial(Ftu: float, Fbru_or_Ftu: float, w: float, D: float, t: float, e: float) -> dict:
    """Simplified static strength of a single lug loaded axially along its centre line (pin hole D, width w, edge
    distance e from hole centre to the lug end, thickness t). Net-section tension with efficiency Kt (0.9 for
    D/w <= 0.5 decreasing) and shear-bearing with Kbr from e/D (Bruhn D1.5 trend, conservative fit)."""
    Kt = max(0.7, 1.0 - 0.3 * max(0.0, D / w - 0.3) / 0.4)
    P_net = Kt * Ftu * (w - D) * t
    eD = e / D
    Kbr = float(np.clip(0.9 * (eD - 0.5), 0.15, 1.8))
    P_br = Kbr * Fbru_or_Ftu * D * t
    return {"P_net": P_net, "P_shear_bearing": P_br, "P_allow": min(P_net, P_br), "Kt": Kt, "Kbr": Kbr}


def euler_buckling(E: float, I: float, L: float, K: float = 1.0) -> float:
    return math.pi ** 2 * E * I / (K * L) ** 2


def thin_plate_shear_buckling(E: float, t: float, b: float, nu: float = 0.33, ks: float = 5.35) -> float:
    """Critical shear stress of a long flat panel of width b with simply supported edges."""
    return ks * math.pi ** 2 * E / (12 * (1 - nu ** 2)) * (t / b) ** 2


@dataclass
class Margin:
    item: str
    load: str
    applied: float
    allowable: float
    fos: float = 1.5
    fitting: float = 1.0
    unit: str = "N"
    ref: str = ""

    @property
    def ms(self) -> float:
        return ms(self.allowable, self.applied, self.fos, self.fitting)

    def row(self) -> dict:
        return {"item": self.item, "load": self.load, "applied": self.applied, "allowable": self.allowable,
                "fos": self.fos, "fitting": self.fitting, "ms": self.ms, "unit": self.unit, "ref": self.ref}


# =====================================================================================================================
# classical lamination theory
# =====================================================================================================================
def ply_Q(E1: float, E2: float, G12: float, nu12: float) -> np.ndarray:
    """Reduced stiffness matrix Q (plane stress) of an orthotropic ply in its material axes."""
    nu21 = nu12 * E2 / E1
    d = 1.0 - nu12 * nu21
    return np.array([[E1 / d, nu12 * E2 / d, 0.0], [nu12 * E2 / d, E2 / d, 0.0], [0.0, 0.0, G12]])


def _T_stress(theta: float) -> np.ndarray:
    """Stress transformation from laminate (x, y) to ply material axes (1, 2) for a ply at ``theta`` (rad)."""
    c, s_ = math.cos(theta), math.sin(theta)
    return np.array([[c * c, s_ * s_, 2 * c * s_], [s_ * s_, c * c, -2 * c * s_], [-c * s_, c * s_, c * c - s_ * s_]])


def Qbar(Q: np.ndarray, theta_deg: float) -> np.ndarray:
    """Transformed reduced stiffness of a ply at ``theta_deg`` (engineering shear strain)."""
    th = math.radians(theta_deg)
    T = _T_stress(th)
    R = np.diag([1.0, 1.0, 2.0])
    return np.linalg.inv(T) @ Q @ R @ T @ np.linalg.inv(R)


def laminate_abd(plies: list) -> dict:
    """ABD matrices of a laminate. ``plies`` = [(props, theta_deg, t), ...] from the bottom (z = -h/2) up, ``props`` a
    dict with E (or E1), E2, G12, nu12. Returns A, B, D (SI), the total thickness h, the ply z-interfaces and the
    ply list."""
    h = float(sum(p[2] for p in plies))
    z = -h / 2.0
    A, B, D = np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3))
    zs = [z]
    for props, th, t in plies:
        Qb = Qbar(ply_Q(props.get("E1", props.get("E")), props["E2"], props["G12"], props["nu12"]), th)
        z1 = z + t
        A += Qb * (z1 - z)
        B += Qb * (z1 ** 2 - z ** 2) / 2.0
        D += Qb * (z1 ** 3 - z ** 3) / 3.0
        z = z1
        zs.append(z)
    return {"A": A, "B": B, "D": D, "h": h, "z": zs, "plies": plies}


def laminate_engineering(lam: dict) -> dict:
    """Membrane engineering constants of a laminate (symmetric laminates; from the inverse of A)."""
    a = np.linalg.inv(lam["A"])
    h = lam["h"]
    return {"Ex": 1.0 / (a[0, 0] * h), "Ey": 1.0 / (a[1, 1] * h), "Gxy": 1.0 / (a[2, 2] * h), "nuxy": -a[0, 1] / a[0, 0]}


def ply_stresses(lam: dict, N, M=(0.0, 0.0, 0.0), restrained: bool = False) -> list:
    """Material-axis stresses (σ1, σ2, τ12) at the top and bottom of every ply for running loads N (N/m) and moments M
    (N m/m) on the laminate. ``restrained``: curvature restrained (κ = 0, membrane response ε = A⁻¹ N), the state of
    an unsymmetric face bonded to a sandwich core (the sandwich as a whole is symmetric)."""
    if restrained:
        e0, k = np.linalg.solve(lam["A"], np.asarray(N, float)), np.zeros(3)
    else:
        ABD = np.block([[lam["A"], lam["B"]], [lam["B"], lam["D"]]])
        e = np.linalg.solve(ABD, np.r_[np.asarray(N, float), np.asarray(M, float)])
        e0, k = e[:3], e[3:]
    out = []
    for i, (props, th, t) in enumerate(lam["plies"]):
        Q = ply_Q(props.get("E1", props.get("E")), props["E2"], props["G12"], props["nu12"])
        T = _T_stress(math.radians(th))
        R = np.diag([1.0, 1.0, 2.0])
        for zz in (lam["z"][i], lam["z"][i + 1]):
            eps_xy = e0 + zz * k
            eps_12 = R @ T @ np.linalg.inv(R) @ eps_xy
            out.append({"ply": i, "z": zz, "sigma": Q @ eps_12, "eps": eps_12})
    return out


def first_ply_failure(lam: dict, N, M=(0.0, 0.0, 0.0), restrained: bool = False) -> dict:
    """First-ply failure strength ratio R (load multiplier to the first ply failure; R >= 1: no failure) by maximum
    stress and by Tsai-Wu (F12 = -0.5 sqrt(F11 F22)). Ply strengths from the ply props: Ftu / Fcu (fibre 1),
    F2tu / F2cu (transverse; fabric plies use Ftu / Fcu), Fsu (in-plane shear). ``restrained`` as in ply_stresses."""
    best = {"R_max_stress": math.inf, "R_tsai_wu": math.inf, "ply_max_stress": None, "ply_tsai_wu": None, "mode": ""}
    for r in ply_stresses(lam, N, M, restrained):
        props = lam["plies"][r["ply"]][0]
        Xt, Xc = props["Ftu"], props["Fcu"]
        Yt, Yc = props.get("F2tu") or Xt, props.get("F2cu") or Xc
        Ss = props["Fsu"]
        s1, s2, t12 = r["sigma"]
        cand = [(Xt / s1 if s1 > 0 else (Xc / -s1 if s1 < 0 else math.inf), "fibre"),
                (Yt / s2 if s2 > 0 else (Yc / -s2 if s2 < 0 else math.inf), "transverse"),
                (Ss / abs(t12) if t12 != 0 else math.inf, "shear")]
        Rm, mode = min(cand)
        if Rm < best["R_max_stress"]:
            best.update(R_max_stress=Rm, ply_max_stress=r["ply"], mode=mode)
        F1, F2 = 1 / Xt - 1 / Xc, 1 / Yt - 1 / Yc
        F11, F22, F66 = 1 / (Xt * Xc), 1 / (Yt * Yc), 1 / Ss ** 2
        F12 = -0.5 * math.sqrt(F11 * F22)
        a = F11 * s1 ** 2 + F22 * s2 ** 2 + F66 * t12 ** 2 + 2 * F12 * s1 * s2
        b = F1 * s1 + F2 * s2
        if a > 0:
            Rt = (-b + math.sqrt(b * b + 4 * a)) / (2 * a)
        elif b > 0:
            Rt = 1.0 / b
        else:
            Rt = math.inf
        if Rt < best["R_tsai_wu"]:
            best.update(R_tsai_wu=Rt, ply_tsai_wu=r["ply"])
    best["R"] = min(best["R_max_stress"], best["R_tsai_wu"])
    return best


# =====================================================================================================================
# plate and sandwich stability
# =====================================================================================================================
def plate_compression_buckling(E: float, t: float, b: float, nu: float = 0.33, k: float = 4.0) -> float:
    """Critical compressive stress of a flat isotropic plate of width b (k = 4.0: long plate, SS edges)."""
    return k * math.pi ** 2 * E / (12 * (1 - nu ** 2)) * (t / b) ** 2


def orthotropic_compression_buckling(D: np.ndarray, a: float, b: float) -> float:
    """Critical running load N_x (N/m) of a specially orthotropic SS plate (length a along the load, width b):
    N = π²/b² min_m [D11 (m b/a)² + 2 (D12 + 2 D66) + D22 (a/(m b))²] over the half-wave number m."""
    D11, D22, H = D[0, 0], D[1, 1], D[0, 1] + 2 * D[2, 2]
    m_opt = a / b * (D22 / D11) ** 0.25
    ms = range(max(1, int(m_opt) - 2), int(m_opt) + 3)
    vals = [D11 * (m * b / a) ** 2 + 2 * H + D22 * (a / (m * b)) ** 2 for m in ms]
    return math.pi ** 2 / b ** 2 * min(vals)


def orthotropic_shear_buckling_long(D: np.ndarray, b: float) -> float:
    """Critical shear running load N_xy (N/m) of a long specially orthotropic SS plate of width b (Kollár & Springer
    long-plate fit, K = (D12 + 2 D66) / sqrt(D11 D22)); the isotropic limit is 5.35 π² D / b²."""
    D11, D22, H = D[0, 0], D[1, 1], D[0, 1] + 2 * D[2, 2]
    K = H / math.sqrt(D11 * D22)
    if K <= 1.0:
        return 4.0 / b ** 2 * (D11 * D22 ** 3) ** 0.25 * (8.125 + 5.045 * K)
    return 4.0 / b ** 2 * math.sqrt(D22 * H) * (11.71 + 1.46 / K ** 2)


def orthotropic_shear_buckling_finite(D: np.ndarray, a: float, b: float) -> float:
    """Critical shear running load N_xy (N/m) of a finite specially orthotropic SS plate, length a along x and width b
    along y (fix round 3): the long-plate value on the SHORT side (D11 / D22 swapped when a < b) times the isotropic
    finite-length factor (5.35 + 4 (s/l)²) / 5.35 of Timoshenko & Gere (k_s = 5.35 + 4 (b/a)², a >= b). An estimate
    for near-isotropic (+-45 dominated) laminates; it reduces to the long-plate value for l >> s."""
    D = np.asarray(D, float)
    if a >= b:
        s_, l_, Dq = b, a, D
    else:
        s_, l_ = a, b
        Dq = D.copy()
        Dq[0, 0], Dq[1, 1] = D[1, 1], D[0, 0]
        Dq[0, 2], Dq[1, 2] = D[1, 2], D[0, 2]
        Dq[2, 0], Dq[2, 1] = D[2, 1], D[2, 0]
    return orthotropic_shear_buckling_long(Dq, s_) * (5.35 + 4.0 * (s_ / l_) ** 2) / 5.35


def sandwich_shear_stiffness(Gc: float, d: float, c: float) -> float:
    """Transverse shear stiffness S = G_c d² / c (N/m) of a sandwich (d: distance between face centroids)."""
    return Gc * d ** 2 / c


def sandwich_buckling(N_bending: float, Gc: float, d: float, c: float) -> float:
    """Sandwich buckling load with transverse shear flexibility: N = N_b / (1 + N_b / S)."""
    S_ = sandwich_shear_stiffness(Gc, d, c)
    return N_bending / (1.0 + N_bending / S_)


def sandwich_D(face_lam: dict, d: float) -> np.ndarray:
    """Bending stiffness matrix of a symmetric sandwich with two equal thin faces (membrane A of one face):
    D = A_face d² / 2 (face own bending neglected)."""
    return face_lam["A"] * d ** 2 / 2.0


def face_wrinkling(Ef: float, Ec: float, Gc: float, Q: float = 0.5) -> float:
    """Face wrinkling stress σ_wr = Q (E_f E_c G_c)^(1/3) (design coefficient Q = 0.5)."""
    return Q * (Ef * Ec * Gc) ** (1.0 / 3.0)


def shear_crimping_stress(Gc: float, d: float, c: float, tf1: float, tf2: float) -> float:
    """Mean face stress at shear crimping, N_crimp / (t_f1 + t_f2), N_crimp = G_c d² / c."""
    return Gc * d ** 2 / c / (tf1 + tf2)


def intracell_dimpling(Ef: float, nuf: float, tf: float, cell: float) -> float:
    """Intracell dimpling stress of a honeycomb-core face σ = 2 E_f / (1 − ν²) (t_f / s)²."""
    return 2.0 * Ef / (1.0 - nuf ** 2) * (tf / cell) ** 2


def sandwich_strip_pressure(p: float, b: float, d: float, tf: float, fixed: bool = False) -> dict:
    """Sandwich strip of span b under uniform pressure p (per unit width, the conservative long-panel limit of a
    plate): bending moment M = p b²/8 (simply supported) or p b²/12 (clamped edge), face stress σ = M/(d t_f),
    core shear τ = (p b/2)/d."""
    M = p * b ** 2 / (12.0 if fixed else 8.0)
    V = p * b / 2.0
    return {"M": M, "V": V, "sigma_face": M / (d * tf), "tau_core": V / d}


# =====================================================================================================================
# columns, tubes, closed sections
# =====================================================================================================================
def johnson_euler(E: float, Fcy: float, A: float, I: float, L: float, K: float = 1.0) -> dict:
    """Column critical stress / load: Johnson parabola below the transition slenderness, Euler above."""
    rho = math.sqrt(I / A)
    sl = K * L / rho
    sl_t = math.pi * math.sqrt(2 * E / Fcy)
    if sl < sl_t:
        sig = Fcy - Fcy ** 2 * sl ** 2 / (4 * math.pi ** 2 * E)
        mode = "Johnson"
    else:
        sig = math.pi ** 2 * E / sl ** 2
        mode = "Euler"
    return {"sigma_cr": sig, "P_cr": sig * A, "slenderness": sl, "transition": sl_t, "mode": mode}


def tube_torsion_stress(T: float, ro: float, ri: float) -> float:
    """Shear stress at the outer fibre of a round tube in torsion τ = T ro / J."""
    return T * ro / (math.pi / 2 * (ro ** 4 - ri ** 4))


def bredt_shear_flow(T: float, A_enclosed: float) -> float:
    """Shear flow q = T / (2 A) of a single closed cell (N/m)."""
    return T / (2.0 * A_enclosed)


# =====================================================================================================================
# joints
# =====================================================================================================================
def bolt_group_inplane(points, F, M: float = 0.0) -> np.ndarray:
    """Elastic bolt-group shear: bolt resultant forces (n,) for an in-plane force F (Fx, Fy) at the group centroid
    plus a moment M about it (equal bolts)."""
    P = np.asarray(points, float)
    c = P.mean(axis=0)
    r = P - c
    J = float((r ** 2).sum())
    n = len(P)
    Fd = np.tile(np.asarray(F, float) / n, (n, 1))
    Fm = np.zeros_like(Fd) if J == 0 else M / J * np.column_stack([-r[:, 1], r[:, 0]])
    return np.linalg.norm(Fd + Fm, axis=1)


def bolt_group_tension(points, N: float, Mx: float = 0.0, My: float = 0.0) -> np.ndarray:
    """Elastic bolt tensions (n,) for an axial force N (positive = tension) and moments about the in-plane centroid
    axes of the group (points in the joint plane, x/y): T_i = N/n + Mx y_i / Σy² − My x_i / Σx²."""
    P = np.asarray(points, float)
    r = P - P.mean(axis=0)
    n = len(P)
    sx, sy = float((r[:, 0] ** 2).sum()), float((r[:, 1] ** 2).sum())
    T = np.full(n, N / n)
    if sy > 0:
        T = T + Mx * r[:, 1] / sy
    if sx > 0:
        T = T - My * r[:, 0] / sx
    return T


def bolt_group_6dof(points, axes, F, M, ref, k_axial: float = 1.0, k_shear: float = 1.0) -> dict:
    """Elastic 6-DOF bolt group of a rigid fitting: bolts at ``points`` (n,3) with unit ``axes`` (n,3), axial stiffness
    k_axial and shear stiffness k_shear (same for all bolts), loaded by the force F (3,) and moment M (3,) about the
    point ``ref``. Solves the rigid-body translation t and rotation w of the fitting (u_i = t + w x r_i) and returns per
    bolt the axial force (signed, + along the axis) and the shear force magnitude."""
    P = np.asarray(points, float) - np.asarray(ref, float)
    A = np.asarray(axes, float)
    A = A / np.linalg.norm(A, axis=1)[:, None]
    K = np.zeros((6, 6))
    Ks = []
    for r, a in zip(P, A):
        Ki = k_axial * np.outer(a, a) + k_shear * (np.eye(3) - np.outer(a, a))
        Rx = np.array([[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0]])   # w x r = -Rx w ... (r x)
        B = np.hstack([np.eye(3), -Rx])                                                  # u = t + w x r = t - [r]x w
        K += B.T @ Ki @ B
        Ks.append((Ki, B))
    q = np.linalg.solve(K, np.concatenate([np.asarray(F, float), np.asarray(M, float)]))
    ax, sh, f = [], [], []
    for (Ki, B), a in zip(Ks, A):
        fi = Ki @ (B @ q)
        f.append(fi)
        ax.append(float(fi @ a))
        sh.append(float(np.linalg.norm(fi - (fi @ a) * a)))
    return {"axial": np.array(ax), "shear": np.array(sh), "forces": np.array(f), "dof": q}


def pin_bending_moment(P: float, t_outer: float, t_inner: float, gap: float = 0.0) -> float:
    """Bending moment of a pin in double shear (two outer lugs of thickness t_outer, inner lug t_inner, load P on the
    pin): M = P/2 (t_outer/2 + gap + t_inner/4) (Melcon & Hoblit, Bruhn D1.8)."""
    return P / 2.0 * (t_outer / 2.0 + gap + t_inner / 4.0)


def pin_bending_stress(M: float, D: float, d_inner: float = 0.0) -> float:
    """Elastic bending stress of a round (hollow) pin."""
    return M * (D / 2.0) / (math.pi / 64.0 * (D ** 4 - d_inner ** 4))


def insert_pullout(tau_core: float, b_p: float, c: float) -> float:
    """Potted-insert pull-out capacity by core shear on the effective potting cylinder: P = 2 π b_p c τ_c."""
    return 2.0 * math.pi * b_p * c * tau_core


# =====================================================================================================================
# pin-jointed space truss (direct stiffness method)
# =====================================================================================================================
def truss3d(nodes, members, fixed, loads, EA=1.0) -> dict:
    """Axial forces of a pin-jointed space truss. ``nodes`` (n,3); ``members`` [(i, j), ...]; ``fixed`` node indices
    (all three DOF restrained); ``loads`` {node: (Fx, Fy, Fz)}; ``EA`` scalar or per member. Returns the member forces
    (tension positive), member lengths, nodal displacements and support reactions."""
    X = np.asarray(nodes, float)
    n = len(X)
    EA = np.broadcast_to(np.asarray(EA, float), (len(members),))
    K = np.zeros((3 * n, 3 * n))
    geo = []
    for k, (i, j) in enumerate(members):
        d = X[j] - X[i]
        L = float(np.linalg.norm(d))
        e = d / L
        k_ = EA[k] / L * np.outer(e, e)
        for a, b_, sgn in ((i, i, 1), (j, j, 1), (i, j, -1), (j, i, -1)):
            K[3 * a:3 * a + 3, 3 * b_:3 * b_ + 3] += sgn * k_
        geo.append((L, e))
    F = np.zeros(3 * n)
    for node, f in loads.items():
        F[3 * node:3 * node + 3] += np.asarray(f, float)
    fixed = set(int(i) for i in fixed)
    free = [k for k in range(3 * n) if k // 3 not in fixed]
    u = np.zeros(3 * n)
    u[free] = np.linalg.solve(K[np.ix_(free, free)], F[free])
    N = np.array([EA[k] / geo[k][0] * float(geo[k][1] @ (u[3 * j:3 * j + 3] - u[3 * i:3 * i + 3]))
                  for k, (i, j) in enumerate(members)])
    R = K @ u - F
    return {"N": N, "L": np.array([g[0] for g in geo]), "u": u.reshape(n, 3),
            "reactions": {i: R[3 * i:3 * i + 3] for i in sorted(fixed)}}


# =====================================================================================================================
# landing gear (CS-LUAS Appendix H, STANAG 4703 Annex B)
# =====================================================================================================================
def sink_speed(ws_npm2: float) -> float:
    """Limit descent velocity V = 0.51 (M g / S)^0.25 m/s, at least 2.13, need not exceed 3.05 (CS-LUAS H.2(b))."""
    return min(max(0.51 * ws_npm2 ** 0.25, 2.13), 3.05)


def drop_height(ws_npm2: float) -> float:
    """Free-drop height h = 0.0132 sqrt(M g / S), 0.235 to 0.475 m (CS-LUAS H.13(a))."""
    return min(max(0.0132 * math.sqrt(ws_npm2), 0.235), 0.475)


def landing_nj(h: float, d: float, ef: float, L: float = 0.667) -> dict:
    """Energy-method wheel (ground-reaction) load factor n_j = (h + (1 − L) d)/(e_f d) and the inertia load factor
    n = n_j + L (STANAG 4703 Annex B UL.GL.1)."""
    nj = (h + (1.0 - L) * d) / (ef * d)
    return {"nj": nj, "n_inertia": nj + L}


def plate_central_patch_moment(P: float, a: float, c: float, nu: float = 0.3) -> float:
    """Largest radial bending moment per unit width (N m / m) of a simply supported circular plate of radius ``a``
    under a load P spread uniformly over a central circle of radius ``c`` (Timoshenko & Woinowsky-Krieger, Theory of
    Plates and Shells, sec. 19): M = P/(4 pi) [(1 + nu) ln(a/c) + 1 - (1 - nu) c^2/(4 a^2)] (at the centre; the
    simply supported edge is the conservative bound for a land surrounded by a stiffer sandwich)."""
    c = min(max(c, 1e-6), a)
    return P / (4.0 * math.pi) * ((1.0 + nu) * math.log(a / c) + 1.0 - (1.0 - nu) * c * c / (4.0 * a * a))
