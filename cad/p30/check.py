"""P30 checks: geometry, interference (nominal, drive stroke, caster swivel), support, springs, LiDAR shadows, simple statics.

Geometric checks only plus hand-calculation statics. Strength, fatigue, bush wear and real fit are NOT verified here.
"""
import math
from build123d import Axis, Solid
from ..validation import _known, bounds_overlap, metrics, support_connectivity
from .build import P30, build_records, drive, lidar_poses, tname, GROUPS

G = 9.81
DENSITY = {'al': 2.70e-6, 'steel': 7.85e-6}   # kg/mm3


def _box(r):
    b = r.shape.bounding_box()
    return (tuple(b.min), tuple(b.max))


_SOLIDS = {}


def _solids(r):
    """Per-solid bounding boxes (cached by shape id). Solid-vs-solid booleans avoid an OCCT access
    violation seen with compound-vs-compound common on the tolerance-repaired camera BRep."""
    key = id(r.shape)
    if key not in _SOLIDS:
        _SOLIDS[key] = [(s, (tuple(b.min), tuple(b.max))) for s in r.shape.solids() for b in (s.bounding_box(),)]
    return _SOLIDS[key]


def common_volume(a, b):
    v = 0.0
    for sa, ba in _solids(a):
        for sb, bb in _solids(b):
            if bounds_overlap(ba, bb):
                cut = sa & sb
                v += 0 if cut is None else cut.volume
    return v


def _hits(movers, obstacles, limit=.01):
    out = []
    boxes = {id(o): _box(o) for o in obstacles}
    for a in movers:
        ba = _box(a)
        for b in obstacles:
            if not bounds_overlap(ba, boxes[id(b)]):
                continue
            v = common_volume(a, b)
            if v > limit:
                out.append({'a': a.name, 'b': b.name, 'volume_mm3': round(v, 3)})
    return out


def _all_pairs(records):
    boxes = [_box(r) for r in records]; hits = []; known = []; tested = 0
    for i, a in enumerate(records):
        for j in range(i+1, len(records)):
            b = records[j]
            if not bounds_overlap(boxes[i], boxes[j]):
                continue
            tested += 1; v = common_volume(a, b)
            if v <= .01:
                continue
            item = {'a': a.name, 'b': b.name, 'volume_mm3': round(v, 3)}
            reason = _known(a.name, b.name, v)
            (known if reason else hits).append({**item, 'reason': reason} if reason else item)
    return tested, hits, known


def geometry(records):
    errors = []
    names = [r.name for r in records]
    if len(set(names)) != len(names):
        errors.append('Duplicate part names')
    for r in records:
        solids = r.shape.solids()
        if not solids or not all(s.is_valid for s in solids):
            errors.append('Invalid shape: '+r.name)
        elif min(s.volume for s in solids) <= 0:
            errors.append('Non-positive volume: '+r.name)
    return errors


def stroke_interference(P, records):
    """Each drive side at -10, -5, +5, +10 mm (module rebuilt, spring at its posed length) against every other part."""
    half = P.L['drive']['travel_mm']/2; out = {'poses': 0, 'hits': [], 'pairs_checked': 0}
    for n, side in (('LEFT', -1), ('RIGHT', 1)):
        own = {r.name for r in records if r.metadata.get('moving_wheel') == n or r.metadata.get('pose_dependent') == n}
        obstacles = [r for r in records if r.name not in own]
        for dz in (-half, -half/2, half/2, half):
            moved = [r for r in drive(P30({n: dz}), side) if r.metadata.get('moving_wheel') == n or r.metadata.get('pose_dependent') == n]
            out['poses'] += 1
            out['pairs_checked'] += len(moved)*len(obstacles)
            for h in _hits(moved, obstacles):
                out['hits'].append({**h, 'pose': f'{n}={dz:+g}'})
    return out


def stroke_contacts(P):
    """P30-5: the housing bottoms must land on the droop stoppers (lower holder) at -10 and the striker must meet the bump stopper tip at +10 (gap <= 0.1)."""
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    half = P.L['drive']['travel_mm']/2; res = {}
    for n, side in (('LEFT', -1), ('RIGHT', 1)):
        for dz, a, b in ((-half, '48_droop_stopper_C-USLH10', '39_linear_bush_'+P.L['drive']['bush']['sku']),
                         (half, '46_bump_stopper_C-USLH10', '54_bump_striker_M6_jam_nut')):
            rr = {r.name: r for r in drive(P30({n: dz}), side)}
            d = BRepExtrema_DistShapeShape(rr[a+'_'+n].shape.wrapped, rr[b+'_'+n].shape.wrapped)
            res[f'{n}{dz:+g}:{a[:2]}-{b[:2]}'] = round(d.Value(), 3)
    return res


def caster_swivel(records, step=30):
    hits = []; tested = 0
    for r in records:
        if not (r.name.startswith('43_GL65_OEM_fork_') or r.name.startswith('44_GL65_OEM_wheel_')):
            continue
        x, y = r.metadata['swivel_xy']; caster = r.metadata['caster']
        obstacles = [o for o in records if o.metadata.get('caster') != caster and not o.name.endswith('_'+caster)]
        for a in range(step, 360, step):
            moved = r.shape.translate((-x, -y, 0)).rotate(Axis.Z, a).translate((x, y, 0))
            tmp = type(r)(r.name+f'@{a}', r.group, moved, r.color, r.status, dict(r.metadata))
            tested += 1
            hits += [{**h, 'angle_deg': a} for h in _hits([tmp], obstacles)]
    return {'samples': tested, 'step_deg': step, 'hits': hits}


def ground(records):
    tires = [r.shape.bounding_box().min.Z for r in records if r.name.startswith('30_NBR')]
    wheels = [r.shape.bounding_box().min.Z for r in records if r.name.startswith('44_GL65_OEM_wheel_')]
    return {'drive_tire_min_z': [round(v, 3) for v in tires], 'caster_wheel_min_z': [round(v, 3) for v in wheels],
            'pass': all(abs(v) < .05 for v in tires+wheels) and len(tires) == 2 and len(wheels) == 4}


def springs(P):
    q = P.L['drive']; sp = q['spring']; half = q['travel_mm']/2
    nominal = P.spring_nominal; rows = {}
    for dz in (-half, 0, half):
        length = nominal-dz
        rows[f'{dz:+g}'] = {'length_mm': round(length, 2), 'deflection_mm': round(sp['free']-length, 2), 'force_N': round((sp['free']-length)*sp['rate'], 1)}
    ok = (sp['free']-(nominal+half) > 0 and nominal-half > sp['solid'] and sp['free']-(nominal-half) <= sp['max_deflection'])
    return {'sku': sp['sku'], 'rate_N_mm': sp['rate'], 'per_wheel': rows, 'max_deflection_mm': sp['max_deflection'],
            'solid_mm': sp['solid'], 'pass': ok}


def lidar_shadows(P, records, radius=700):
    """Approximate blocked sectors in each scan plane: angular span of every part section within `radius` (nominal pose)."""
    from ..parts.common import span
    out = []
    for pose in lidar_poses(P):
        n = pose['name']; x0, y0, z = pose['x'], pose['y'], pose['scan_z']
        slab = span(x0-radius, x0+radius, y0-radius, y0+radius, z-.5, z+.5)
        blocked = []
        for r in records:
            if r.name.endswith(n) and r.group == 'lidars':
                continue
            b = r.shape.bounding_box()
            if not (b.min.Z < z+.5 and b.max.Z > z-.5):
                continue
            cut = r.shape & slab
            if cut is None or not cut.solids():
                continue
            angles = []
            for v in cut.vertices():
                h = math.degrees(math.atan2(v.X-x0, v.Y-y0))-pose['heading_deg']
                angles.append((h+180) % 360-180)
            if not angles:
                continue
            lo, hi = min(angles), max(angles)
            if hi-lo > 180:      # wraps behind the scanner: inside the blind sector
                continue
            if hi < -135 or lo > 135:
                continue
            dist = min(math.hypot(v.X-x0, v.Y-y0) for v in cut.vertices())
            blocked.append({'part': r.name, 'from_deg': round(max(lo, -135), 1), 'to_deg': round(min(hi, 135), 1), 'nearest_mm': round(dist, 1)})
        out.append({'lidar': n, 'scan_z': round(z, 2), 'heading_deg': pose['heading_deg'], 'fov_deg': [-135, 135],
                    'blocked': sorted(blocked, key=lambda b: b['from_deg']),
                    'method': 'BRep section vertices in a 1 mm slab; angles relative to heading; approximate, not a safety analysis'})
    return out


def _hull_area(pts):
    pts = sorted(set(pts))
    if len(pts) < 3:
        return 0.0
    cross = lambda o, a, b: (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lo, hi = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    h = lo[:-1]+hi[:-1]
    return abs(sum(h[i][0]*h[i-1][1]-h[i-1][0]*h[i][1] for i in range(len(h))))/2


def camera_fov(P, records, depth=1500.0):
    """Camera view pyramids (depth FOV) from each of the three front optics of the Astra STEP, boolean against every
    other part. Reports the intersecting parts, the nearest distance to the optic and the share of the image they cover
    (convex hull of the projected intersection, near objects can hide more behind them). Geometric only."""
    from build123d import Face, Location, Rotation, Shell, Vector, Vertex, Wire
    sn = P.L['sensors']; H, V = sn['camera_fov_deg']; th, tv = math.tan(math.radians(H/2)), math.tan(math.radians(V/2))
    out = []
    for side in (-1, 1):
        n = 'LEFT' if side < 0 else 'RIGHT'
        cam = next(r for r in records if r.name == '70_Orbbec_Astra_Mini_Pro_MX6000_'+n)
        fr = Location((side*sn['camera_x'], sn['camera_y'], sn['camera_z']))*Rotation(0, 0, -side*sn['camera_yaw_deg'])*Rotation(-sn['camera_pitch_down_deg'], 0, 0)
        inv = fr.inverse(); lb = cam.shape.moved(inv).bounding_box()
        for lx in sn['camera_optics_local_x']:
            ax, ay, az = lx, lb.max.Y+.2, lb.center().Z
            A = Vector(ax, ay, az)
            C = [Vector(ax+u*depth*th, ay+depth, az+w*depth*tv) for u, w in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
            pyr = Solid(Shell([Face(Wire.make_polygon([A, C[i], C[(i+1) % 4]], close=True)) for i in range(4)]
                              + [Face(Wire.make_polygon(C, close=True))])).moved(fr)
            fb = pyr.bounding_box(); apex = Vertex(ax, ay, az).moved(fr)
            hits = []
            for r in records:
                if r.group == 'cameras' and r.name.startswith('70_'):
                    continue
                b = r.shape.bounding_box()
                if b.max.X < fb.min.X or b.min.X > fb.max.X or b.max.Y < fb.min.Y or b.min.Y > fb.max.Y or b.max.Z < fb.min.Z or b.min.Z > fb.max.Z:
                    continue
                vol, near, uv = 0.0, None, []
                for s in r.shape.solids():
                    cut = s & pyr
                    for cs in ([] if cut is None else cut.solids()):
                        vol += cs.volume
                        dd = cs.distance_to(apex); near = dd if near is None else min(near, dd)
                        for v in cs.moved(inv).vertices():
                            if v.Y-ay > 1e-6:
                                uv.append((max(-1, min(1, (v.X-ax)/(v.Y-ay)/th)), max(-1, min(1, (v.Z-az)/(v.Y-ay)/tv))))
                if vol > 1e-3:
                    hits.append({'part': r.name, 'group': r.group, 'volume_mm3': round(vol, 1), 'nearest_mm': round(near, 1),
                                 'image_pct': round(100*_hull_area(uv)/4, 2)})
            out.append({'camera': n, 'optic_local_x': lx, 'fov_deg': [H, V], 'hits': sorted(hits, key=lambda h: -h['image_pct'])})
    return {'pyramids': out, 'profile_hits': sorted({h['part'] for p in out for h in p['hits'] if h['group'] == 'frame'}),
            'method': 'Pyramid from the front plane of each optic opening (STEP), 1.5 m deep, boolean vs every part; '
                      'image share from the convex hull of the projected intersection; not an optical simulation'}


def statics(P, records):
    lc = P.L['load_case']; q = P.L['drive']; sp = q['spring']; c = P.L['caster']
    W = lc['mass_kg']*G; W0 = lc['mass_no_arm_kg']*G
    F = sp['target_force_N']; half = q['travel_mm']/2
    caster_each = (W-2*F)/4; caster_each0 = (W0-2*F)/4
    # traction, friction-limited
    mu = lc['friction_mu']; Fmin = (sp['free']-(P.spring_nominal+half))*sp['rate']
    trac = 2*mu*F
    # bush tilt couple from the traction force at the contact patch (lever = bush flange height above ground).
    # The carriage pitches about X; each single bushing resists it over its own length, the two bushings share it.
    bu = q['bush']; zc = q['housing_center_z']        # P30-4: lever = contact patch to the LHSS20 housing centre
    couple = mu*F*zc/1000
    radial = couple/(bu['length']/1000)/2
    fric = bu['rolling_mu']*2*radial
    # deck arm members: 3030 Ix ~ 2.8e4 mm4, W ~ 1.8e3 mm3 (generic catalogue order of magnitude), arm 20 kg x 3 g shared by 2
    Fm = 20*G*3/2; Lm = 2*(P.L['frame']['outer_x']-P.s)
    return {
        'weight_N': round(W, 1), 'drive_spring_each_N': F, 'drive_share_pct': round(200*F/W, 1),
        'caster_each_N': round(caster_each, 1), 'caster_each_no_arm_N': round(caster_each0, 1),
        'caster_capacity_N': round(P.parts['caster']['capacity_each_kg']*G, 1),
        'traction_limit_N': round(trac, 1), 'traction_accel_m_s2': round(trac/lc['mass_kg'], 2),
        'traction_at_full_droop_each_N': round(mu*Fmin, 1),
        'rated_torque_force_each_N': round(P.parts['drive_motor']['rated_torque_Nm']/(P.parts['drive_motor']['tire_dia']/2000), 1),
        'tip_over_accel_g': {'lateral': round(c['x']/lc['cg_height_mm'], 2), 'longitudinal': round(c['y']/lc['cg_height_mm'], 2),
                             'note': 'support polygon = rigid casters; CG height assumed'},
        'bush_tilt_couple_Nm': round(couple, 2), 'bush_radial_each_N': round(radial, 1),
        'bush_load_ratio_dynamic': round(radial/bu['dynamic_load_N'], 3),
        'bush_life_km_at_full_traction': round(50*(bu['dynamic_load_N']/radial)**3),
        'slide_friction_N': round(fric, 2), 'slide_friction_mu': bu['rolling_mu'],
        'slide_friction_vs_spring_pct': round(100*fric/F, 2),
        'bush_note': 'Ball bushing rolling friction from the catalogue order of magnitude; seal drag not included (measure). '
                     'MISUMI advises against single bushings under large moment load: here two singles share the pitch moment; '
                     'a double-length housing (LHSW20 class) is the upgrade if wear shows',
        'arm_member_3g': {'force_each_N': round(Fm, 1), 'span_mm': Lm, 'sigma_MPa': round(Fm*Lm/4/1.8e3, 1),
                          'deflection_mm': round(Fm*Lm**3/(48*7e4*2.8e4), 3)},
        'scope': 'Hand calculations with assumed CG (centre) and generic profile section; no FEM, no fatigue, no dynamic loads'}


def mass(records):
    total = {}
    for r in records:
        mat = None
        if r.status in ('generic_profile',):
            mat = 'al'
        elif r.name.startswith(('11_', '12_', '13_', '36_', '37_', '51_', '52_', '61_motor_driver_bracket',
                                '50_', '44_spring_stud', '33_', '34_')):
            mat = 'steel'          # P30-3: all plates and brackets SS400 / SPCC
        if mat:
            total.setdefault(r.group if r.group in ('frame', 'plates') else 'brackets_and_drive_fab', 0)
            total[r.group if r.group in ('frame', 'plates') else 'brackets_and_drive_fab'] += sum(s.volume for s in r.shape.solids())*DENSITY[mat]
    return {k: round(v, 2) for k, v in total.items()}


def validate(P=None, records=None, full=True):
    P = P or P30(); records = records or build_records(P)
    errors = geometry(records)
    report = {'variant': 'P30', 'revision': P.L['revision'], 'components': len(records), 'solids': sum(len(r.shape.solids()) for r in records)}
    report['ground'] = ground(records)
    if not report['ground']['pass']:
        errors.append('Wheel contact heights')
    report['springs'] = springs(P)
    if not report['springs']['pass']:
        errors.append('Spring catalogue travel exceeded')
    report['statics'] = statics(P, records)
    report['fabricated_mass_kg'] = mass(records)
    if full:
        tested, hits, known = _all_pairs(records)
        report['static_interference'] = {'boolean_pairs': tested, 'hits': hits, 'known_manufacturer_overlaps': known}
        errors += ['Static interference: '+h['a']+' x '+h['b'] for h in hits]
        report['stroke_interference'] = stroke_interference(P, records)
        errors += ['Stroke interference: '+h['a']+' x '+h['b']+' @'+h['pose'] for h in report['stroke_interference']['hits']]
        report['stroke_end_contacts_mm'] = sc = stroke_contacts(P)
        errors += ['Stroke end stop gap '+k for k, v in sc.items() if v > .1]
        report['caster_swivel'] = cs = caster_swivel(records)
        errors += ['Caster swivel: '+h['a']+' x '+h['b'] for h in cs['hits']]
        report['support'] = sup = support_connectivity(records)
        errors += ['Unsupported: '+n for n in sup['unsupported']]
        report['lidar_shadows'] = lidar_shadows(P, records)
        report['camera_fov'] = camera_fov(P, records)
    report['errors'] = errors
    report['valid'] = not errors
    return report
