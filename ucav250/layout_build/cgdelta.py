from build import *
L, A = build()
M = S["mass"]
items = {i["name"]: i for i in M["items"]}
mt = sum(float(i["mass_kg"]) for i in M["items"])
mom = 0.0
rows = []
for k, v in L["mass_placement"].items():
    it = items[k]; p = v["position"]; m = float(it["mass_kg"])
    rows.append((k, m, p[0] - it["x"])); mom += m * (p[0] - it["x"])
bc = {it["name"]: it for it in L["rules"]["bay_contents"]["items"]}
bx = bc["buffer_battery"]["center"][0]
it = items["buffer_battery_12S2P_liion"]; rows.append(("battery", it["mass_kg"], bx - it["x"])); mom += it["mass_kg"] * (bx - it["x"])
pb = L["rules"]["boxes"]["parachute_bay"]["x"]; it = items["parachute_uavos_200"]
rows.append(("parachute", it["mass_kg"], 0.5 * sum(pb) - it["x"])); mom += it["mass_kg"] * (0.5 * sum(pb) - it["x"])
for r in rows: print(f"{r[0]:42s} {r[1]:6.3f} dx {r[2]:+.3f} -> {r[1]*r[2]:+.3f}")
print("empty moment change", round(mom, 3), "kg m; dCG empty", round(mom / mt, 4), "MTOW", round(mom / 149.9, 4))
