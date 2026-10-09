import sys, math
sys.path.insert(0, '/home/user/Drone-controller')
import numpy as np
from ucav250.core import spec as SPEC
from ucav250.analysis import sizing as Z
S = SPEC.load()
af = Z.Airframe(S)
print("x    halfw   zc     ztop0   zbot0  ztop(y=.2) zbot(y=.2) area")
for x in np.arange(0.0, 4.01, 0.1):
    a, bt, bb, zc, nt, nb = (float(v[0]) for v in af.sec(x))
    print(f"{x:4.2f} {a:6.3f} {zc:6.3f} {af.z_top(x):7.3f} {af.z_bot(x):7.3f} {af.z_top(x,0.2):7.3f} {af.z_bot(x,0.2):7.3f} {af.area(x):6.4f} nt={nt:.2f} nb={nb:.2f}")
P = S['wing']['planform']
for y in (0.0, 0.33, 0.4, 0.5, 0.6, 0.7, 1.0):
    print('spar_x main/rear at y', y, Z.spar_x(P, y, P['main_spar_frac']), Z.spar_x(P, y, P['rear_spar_frac']))
T = Z.trapezoid(P)
print('trapezoid c(0.7)', T['c'](0.7), 'xle', T['xle'](0.7))
