"""Top deck thickness screening: 420x550 steel deck on four 2020 posts, arm at centre."""
import numpy as np
from plate_fem import Plate

EI_2020 = 70000 * 6.98e3     # MISUMI HFS5-2020 class Ix ~0.70 cm^4 (check catalog)
EI_2040 = 70000 * 4.5e4      # 2040 strong axis ~4.5 cm^4 (approximate, check catalog)
POSTS = [(sx*146, sy*208) for sx in (-1, 1) for sy in (-1, 1)]
BOLTS = [(sx*62, sy*62) for sx in (-1, 1) for sy in (-1, 1)]

def grid(half, key):
    v = set(np.round(np.arange(-half, half + 1e-6, 5.0), 3)) | {k for k in key} | {-k for k in key}
    return np.array(sorted(v))

def run(t, t_reinf=5.0, frame=None, case='V', mass_arm=25.0, g_dyn=2.0, moment=150e3):
    xs = grid(210, [146, 62, 77.5]); ys = grid(275, [208, 62, 77.5])
    inside_r = lambda x, y: abs(x) < 77.5 and abs(y) < 77.5
    teff = lambda x, y: (t**3 + t_reinf**3) ** (1/3) if (t_reinf and inside_r(x, y)) else t
    p = Plate(xs, ys, teff)
    for x, y in POSTS:
        p.fix_w(p.box_nodes(x, y, 10, 10))
    if frame:
        EI_cross = EI_2020 if frame == '2020' else EI_2040
        for x in (-146, 146): p.beam('y', x, -208, 208, EI_2020 if frame == '2020' else EI_2040)
        for y in (-208, 208): p.beam('x', y, -146, 146, EI_2020)
        for y in (-62, 62): p.beam('x', y, -146, 146, EI_cross)
    # deck self weight x g_dyn
    area_w = 7.85e-6 * 9.81 * g_dyn
    for n in range(p.n):
        pass
    # lumped self weight on nodes (approx): total / nodes
    tot = 420 * 550 * t * area_w
    p.F[0::3] += -tot / p.n
    if case == 'V':
        for x, y in BOLTS: p.force(x, y, -mass_arm * 9.81 * g_dyn / 4)
    else:  # static arm weight + overturning moment about X
        for x, y in BOLTS:
            p.force(x, y, -mass_arm * 9.81 / 4 + np.sign(y) * moment / (2 * 124) * -1)
    u = p.solve()
    w = u[0::3]
    wb = {b: w[p.near(*b, 0.1)[0]] for b in BOLTS}
    tilt = np.degrees(abs((wb[(62, 62)] + wb[(-62, 62)]) - (wb[(62, -62)] + wb[(-62, -62)])) / 2 / 124)
    s = p.stresses(lambda x, y: t)
    def far(cx, cy):
        return min(np.hypot(cx - x, cy - y) for x, y in BOLTS + POSTS) > 15
    smax = max(v for v, cx, cy in s if far(cx, cy))
    return dict(t=t, frame=frame or 'posts only', case=case, w_max=abs(w).max(), tilt_deg=tilt, sigma_max=smax)

if __name__ == '__main__':
    print(f"{'frame':12s} {'case':4s} {'t':>3s} {'|w|max mm':>10s} {'tilt deg':>9s} {'sVM MPa':>8s}")
    for frame in (None, '2020', '2040'):
        for t in (2, 3, 4, 5, 6):
            for case in ('V', 'M'):
                r = run(t, frame=frame, case=case)
                print(f"{r['frame']:12s} {case:4s} {t:3d} {r['w_max']:10.2f} {r['tilt_deg']:9.3f} {r['sigma_max']:8.0f}")
