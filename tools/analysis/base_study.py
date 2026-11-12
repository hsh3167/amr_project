"""Base plate thickness screening: 390x530 steel plate with drive wheel wells.
Drive yoke M5 mounts are supports; posts, battery, electronics and caster reactions are loads."""
import numpy as np
from plate_fem import Plate

POSTS = [(sx*146, sy*208) for sx in (-1, 1) for sy in (-1, 1)]
DRIVE = [(sx*136, y) for sx in (-1, 1) for y in (-55, -25, 25, 55)]
CASTER = [(cx+dx, cy+np.sign(cy)*dy) for cx in (-138, 138) for cy in (-215, 215) for dx in (-27, 27) for dy in (-68, 29)]

def inside(x, y):
    return not (abs(x) > 145 and abs(y) < 99)

def grid(half, key):
    v = set(np.round(np.arange(-half, half + 1e-6, 5.0), 3)) | set(key) | {-k for k in key}
    return np.array(sorted(v))

def run(t, top_mass=32.0, battery=4.0, elec=2.0, g=2.0, caster_share=0.40, total_sprung=44.2):
    xs = grid(195, [146, 136, 145, 165, 111]); ys = grid(265, [208, 99, 55, 25, 244, 147, 186, 283])
    p = Plate(xs, ys, lambda x, y: t, inside)
    p.fix_w(np.concatenate([p.near(x, y, 3) for x, y in DRIVE]))
    W = 9.81 * g
    p.F[0::3] += -(390*530 - 2*50*198) * t * 7.85e-6 * W / p.n
    for x, y in POSTS: p.force(x, y, -top_mass * W / 4, r=10)
    bat = p.box_nodes(0, -52, 117, 77); p.F[3*bat] += -battery * W / len(bat)
    el = p.box_nodes(0, 170, 100, 40); p.F[3*el] += -elec * W / len(el)
    fc = caster_share * total_sprung * W / 4
    for x, y in CASTER: p.force(x, y, fc / 4, r=3)
    u = p.solve(); w = u[0::3]
    s = p.stresses()
    far = lambda cx, cy: min(np.hypot(cx-x, cy-y) for x, y in POSTS+DRIVE+CASTER) > 12
    smax = max(v for v, cx, cy in s if far(cx, cy))
    corner = max(abs(w[p.near(x, y, 10)]).max() for x, y in POSTS)
    return t, abs(w).max(), corner, smax

if __name__ == '__main__':
    print('t  |w|max  post_w  sVM(2g)')
    for t in (2.3, 3.2, 4.0, 4.5, 6.0):
        print('%.1f %6.2f %6.2f %6.0f' % run(t))
