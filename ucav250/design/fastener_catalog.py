"""Standard fastener dimensions (metres) used by hardware.py and by producers sizing holes and edge distances."""
from __future__ import annotations

# ISO 4762 socket head cap screw: head diameter dk, head height k
ISO4762 = {3: (0.0055, 0.003), 4: (0.007, 0.004), 5: (0.0085, 0.005), 6: (0.010, 0.006), 8: (0.013, 0.008),
           10: (0.016, 0.010), 12: (0.018, 0.012)}
# ISO 7380 button head: dk, k
ISO7380 = {3: (0.0057, 0.00165), 4: (0.0076, 0.0022), 5: (0.0095, 0.00275), 6: (0.0105, 0.0033), 8: (0.014, 0.0044)}
# ISO 4017 hex head: across flats s, head height k
ISO4017 = {3: (0.0055, 0.002), 4: (0.007, 0.0028), 5: (0.008, 0.0035), 6: (0.010, 0.004), 8: (0.013, 0.0053),
           10: (0.016, 0.0064), 12: (0.018, 0.0075)}
# ISO 7040 nyloc nut: across flats s, height m
ISO7040 = {3: (0.0055, 0.004), 4: (0.007, 0.005), 5: (0.008, 0.005), 6: (0.010, 0.006), 8: (0.013, 0.008),
           10: (0.016, 0.010), 12: (0.018, 0.012)}
# ISO 7089 plain washer: inner d1, outer d2, thickness h
ISO7089 = {3: (0.0032, 0.007, 0.0005), 4: (0.0043, 0.009, 0.0008), 5: (0.0053, 0.010, 0.001),
           6: (0.0064, 0.012, 0.0016), 8: (0.0084, 0.016, 0.0016), 10: (0.0105, 0.020, 0.002),
           12: (0.013, 0.024, 0.0025)}
# ISO 273 medium clearance hole diameters
CLEARANCE = {2.5: 0.0029, 3: 0.0034, 4: 0.0045, 5: 0.0055, 6: 0.0066, 8: 0.009, 10: 0.011, 12: 0.0135}
# Floating anchor nutplate (MS21047/NAS1068 style, metric equivalent): base length, width, height, rivet pitch
NUTPLATE = {3: (0.018, 0.007, 0.0045, 0.0125), 4: (0.022, 0.0085, 0.0055, 0.0155), 5: (0.025, 0.0095, 0.006, 0.018),
            6: (0.028, 0.0105, 0.007, 0.0205)}
# Quarter-turn panel fastener (Camloc 4002 / Dzus style): stud head diameter, head height, receptacle plate L x W x t,
# receptacle barrel diameter/height
QUARTER_TURN = {"stud_head_d": 0.0115, "stud_head_h": 0.0025, "stud_d": 0.0048, "rec_plate": (0.028, 0.010, 0.001),
                "rec_barrel": (0.009, 0.007)}
# Potted/threaded insert for sandwich panels: flange diameter, body diameter, length
INSERT = {3: (0.010, 0.007, 0.008), 4: (0.012, 0.008, 0.010), 5: (0.014, 0.009, 0.012), 6: (0.016, 0.011, 0.014)}

# Recommended tightening torques (N·m), A2-70 stainless / 8.8 steel into steel nuts, dry (typical ISO tables);
# producers may override with a lower value for composite joints (bearing limited).
TORQUE = {"A2-70": {3: 1.1, 4: 2.6, 5: 5.1, 6: 8.8, 8: 21.0, 10: 42.0},
          "8.8": {3: 1.3, 4: 3.0, 5: 5.9, 6: 10.0, 8: 25.0, 10: 49.0, 12: 86.0},
          "12.9": {3: 2.1, 4: 4.9, 5: 9.9, 6: 17.0, 8: 41.0, 10: 83.0, 12: 145.0}}


def clearance(d_mm: float) -> float:
    return CLEARANCE.get(d_mm, round(d_mm * 1.1, 1) * 1e-3)


def size_from_spec(spec: str) -> int | None:
    """Nominal size from a designation like 'ISO 4762 M6x20-12.9' -> 6."""
    import re
    m = re.search(r"M(\d+(?:\.\d+)?)", spec)
    return None if m is None else int(float(m.group(1)))
