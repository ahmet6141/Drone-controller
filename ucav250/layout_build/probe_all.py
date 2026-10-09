import json, time
t=time.time()
from b_common import *
import b_stations, b_chassis, b_mech, b_systems, b_shell, b_assembly
st=b_stations.stations(); print("stations", len(st), list(st[0].keys()))
mem=b_chassis.members(); print("members", len(mem), list(mem[0].keys()))
wj=b_chassis.wing_joint(); print("wing_joint", list(wj.keys()))
fi=b_chassis.fittings(); print("fittings", len(fi), list(fi[0].keys()))
me=b_mech.mechanisms(); print("mech", list(me.keys()), len(me.get("joints",[])))
print("keep_outs", len(b_mech.keep_outs()), "clearances", len(b_mech.clearances()))
L=copy.deepcopy(S["layout"]); b_systems.apply_rule_edits(L)
eqs=b_systems.equipment(L); print("equipment", len(eqs), list(eqs[0].keys()))
print("antennas", len(b_systems.antennas()), "adl", len(b_systems.air_data_lights()), "wact", len(b_systems.wing_actuators()))
h=b_systems.harness(); print("harness", list(h.keys()))
pn=b_shell.panels(); print("panels", len(pn), list(pn[0].keys()))
print("steps", len(b_assembly.steps()))
print(time.time()-t)
