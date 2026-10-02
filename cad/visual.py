"""Renk–malzeme–yüzey (CMF) tanımları: GLB renkleri ve Blender PBR malzemeleri (CadQuery gerektirmez).

Tasarım dili: karbon gövde + mat antrasit PA-CF baskı parçalar + **güvenlik turuncusu** avuç
tutamağı (operatör iniş noktasını uzaktan görür) + siyah TPU tampon. Elektronik ve satın alınan
parçalar gerçek renklerinde.
"""
from __future__ import annotations

# ad → taban rengi (sRGB hex), metalik, pürüzlülük, kaplama (clearcoat), desen, açıklama
MATERIALS: dict[str, dict] = {
    "carbon": {"color": "#121314", "metallic": 0.0, "roughness": 0.30, "coat": 0.7, "pattern": "weave",
               "desc": "karbon fiber gövde ve kollar"},
    "pa_cf": {"color": "#2a2c30", "metallic": 0.0, "roughness": 0.70, "coat": 0.0, "pattern": "grain",
              "desc": "PA-CF baskı: mat antrasit"},
    "accent": {"color": "#ff6a13", "metallic": 0.0, "roughness": 0.55, "coat": 0.15, "pattern": "grain",
               "desc": "güvenlik turuncusu: avuç tutamağı"},
    "tpu": {"color": "#161616", "metallic": 0.0, "roughness": 0.92, "coat": 0.0, "pattern": None,
            "desc": "TPU tampon ve sönümleyiciler"},
    "aluminum": {"color": "#aeb3ba", "metallic": 1.0, "roughness": 0.32, "coat": 0.0, "pattern": None,
                 "desc": "alüminyum ara parçalar"},
    "anodized": {"color": "#3b3f46", "metallic": 1.0, "roughness": 0.38, "coat": 0.0, "pattern": None,
                 "desc": "eloksallı motor çanı (titanyum gri)"},
    "steel": {"color": "#d4d7db", "metallic": 1.0, "roughness": 0.22, "coat": 0.0, "pattern": None,
              "desc": "çelik / nikel kaplama (mil, portlar)"},
    "brass": {"color": "#b8913d", "metallic": 1.0, "roughness": 0.35, "coat": 0.0, "pattern": None,
              "desc": "pirinç ara parçalar"},
    "pcb": {"color": "#1d5b33", "metallic": 0.0, "roughness": 0.45, "coat": 0.35, "pattern": None,
            "desc": "devre kartı"},
    "plastic": {"color": "#101010", "metallic": 0.0, "roughness": 0.50, "coat": 0.0, "pattern": None,
                "desc": "siyah plastik / çip"},
    "prop": {"color": "#1c1f23", "metallic": 0.0, "roughness": 0.28, "coat": 0.5, "pattern": None,
             "desc": "pervane (polikarbonat)"},
    "glass": {"color": "#050505", "metallic": 0.0, "roughness": 0.04, "coat": 1.0, "pattern": None,
              "desc": "mercek"},
    "battery": {"color": "#16324f", "metallic": 0.0, "roughness": 0.40, "coat": 0.5, "pattern": None,
                "desc": "batarya ısıl daralan kılıf"},
    "yellow": {"color": "#f2c200", "metallic": 0.0, "roughness": 0.45, "coat": 0.0, "pattern": None,
               "desc": "XT60 konnektör"},
    "red": {"color": "#b3121b", "metallic": 0.0, "roughness": 0.45, "coat": 0.0, "pattern": None,
            "desc": "kablo / kayış"},
    "copper": {"color": "#b87333", "metallic": 1.0, "roughness": 0.35, "coat": 0.0, "pattern": None,
               "desc": "motor sargısı"},
}

# Parça adı öneki → malzeme (en uzun önek kazanır). Montajdaki her ad bir öneke uymalıdır.
PART_MATERIAL: dict[str, str] = {
    "guard_ring": "pa_cf", "guard_mount": "pa_cf",
    "grip_tube": "accent", "grip_sensor_mount": "pa_cf", "grip_bumper": "tpu",
    "gimbal_cradle": "pa_cf", "gimbal_roll_arm": "pa_cf", "gimbal_top": "pa_cf", "gimbal_boom": "pa_cf",
    "companion_tray": "pa_cf", "battery_plate": "pa_cf", "companion_shell": "pa_cf",
    "frame": "carbon", "frame_standoff": "aluminum", "frame_stack": "plastic", "frame_stack_pcb": "pcb",
    "motor_base": "anodized", "motor_winding": "copper", "motor_bell": "anodized", "motor_shaft": "steel",
    "motor_nut": "steel", "gimbal_motor": "anodized", "prop": "prop",
    "pi_pcb": "pcb", "pi_port": "steel", "pi_cooler": "plastic", "pi_chip": "plastic", "pi_header": "plastic",
    "pi_standoff": "brass", "hat_pcb": "pcb",
    "camera_pcb": "pcb", "camera_lens": "plastic", "camera_glass": "glass",
    "battery": "battery", "battery_strap": "red", "xt60": "yellow", "wire_red": "red", "wire_black": "plastic",
    "gnss_mast": "carbon", "gnss": "plastic", "flow_pcb": "pcb", "flow_sensor": "plastic",
    "antenna": "plastic", "damper": "tpu", "standoff": "aluminum",
    "tof_pcb": "pcb", "tof_sensor": "plastic",
}


def material_for(name: str) -> str:
    """Nesne adından malzeme (Blender'da glTF nesne adları; '.001' gibi ekler yok sayılır)."""
    base = name.split(".")[0]
    hits = [prefix for prefix in PART_MATERIAL if base.startswith(prefix)]
    if not hits:
        raise KeyError(f"{name!r} için malzeme tanımı yok (visual.PART_MATERIAL)")
    return PART_MATERIAL[max(hits, key=len)]


def rgb(hex_color: str) -> tuple[float, float, float]:
    h = hex_color.lstrip("#")
    return tuple(int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4))


def srgb_to_linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def linear_rgb(hex_color: str) -> tuple[float, float, float]:
    return tuple(srgb_to_linear(c) for c in rgb(hex_color))
