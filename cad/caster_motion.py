"""Suspension kinematics.

Rev12-14 concept 'hinge': six independent hinge (swing-arm) modules, pivot axes parallel to +Y.
Motion keys: LEFT / RIGHT drive wheels (vertical travel of the wheel centre, +-10 mm) and
LF / LB / RF / RB casters (vertical travel of the caster mounting centre, +-5 mm).

Rev15 concept 'bogie': on each side the drive wheel and the front caster hang on one bogie link
that turns about an axis parallel to +X. Motion keys LEFT / RIGHT are the bogie poses (value =
vertical travel of the front caster mounting centre relative to the chassis, +-10 mm; the drive
wheel moves the other way by the lever ratio). LB / RB stay hinge modules. Positive raises the wheel.
"""
from math import asin, cos, degrees, sin

from build123d import Axis

CASTER_WHEELS = ('LF', 'LB', 'RF', 'RB')
DRIVE_WHEELS = ('LEFT', 'RIGHT')
MOTION_KEYS = DRIVE_WHEELS + CASTER_WHEELS


def is_bogie(d):
    return d.layout.get('suspension', {}).get('concept') == 'bogie'


def motion_keys(d):
    return DRIVE_WHEELS + ('LB', 'RB') if is_bogie(d) else MOTION_KEYS


def hinge_wheels(d):
    """Wheels that sit on their own Rev12-14 hinge module."""
    return ('LB', 'RB') if is_bogie(d) else MOTION_KEYS


def is_bogie_key(d, key):
    return is_bogie(d) and key in DRIVE_WHEELS


def caster_xy(d, wheel):
    return (d.p['caster_x'] * (-1 if wheel[0] == 'L' else 1),
            d.p['caster_y'] * (1 if wheel[1] == 'F' else -1))


def caster_motion_key(wheel, d=None):
    if d is not None and is_bogie(d) and wheel[1] == 'F':
        return 'LEFT' if wheel[0] == 'L' else 'RIGHT'
    return wheel


def side_of(d, key):
    if key in DRIVE_WHEELS:
        return -1 if key == 'LEFT' else 1
    return -1 if key[0] == 'L' else 1


def half_travel(d, key):
    if is_bogie_key(d, key):
        return d.layout['suspension']['bogie']['travel_mm'] / 2
    kind = 'drive' if key in DRIVE_WHEELS else 'caster'
    return d.layout['suspension'][kind]['travel_mm'] / 2


def bogie_lengths(d):
    """(pivot to front caster, pivot to drive axle) along Y."""
    q = d.layout['suspension']['bogie']
    return d.p['caster_y'] - q['pivot_y_mm'], q['pivot_y_mm']


def bogie_stack(d):
    """Rev16 pivot stack along +X for the right bogie (mirror for the left), outboard to inboard.

    Shoulder-bolt head next to the outer wall, outboard U-bracket ear, collar, bearing holder 1 (pocket
    outboard), mid collar, holder 2 (pocket inboard), collar, inboard ear (past the shoulder end), lock nut.
    The collar stack is spacer_* longer than the shoulder, so the nut clamps ears, collars and inner rings."""
    q = d.layout['suspension']['bogie']
    bt, B, inset, T = q['bracket_t_mm'], q['bearing_B_mm'], q['bearing_inset_mm'], q['holder_T_mm']
    s = {'head': (q['outer_wall_x_mm'] - q['head_gap_mm'] - q['bolt_head_t_mm'], q['outer_wall_x_mm'] - q['head_gap_mm'])}
    x = s['head'][0]
    if q.get('holder_count', 2) == 1:
        # Rev17d: one double-bearing holder (BGMW6001DD-30, two 6001 back to back, lip 2 mm outboard, mouth 5 mm inboard).
        n = q['bearings_per_holder']
        for name, length in (('ear_out', bt), ('spacer_out', q['spacer_outer_mm']), ('race_out', n*B),
                             ('spacer_in', q['spacer_inner_mm']), ('ear_in', bt), ('nut', q['nut_h_mm'])):
            s[name] = (x - length, x)
            x -= length
        s['holder_out'] = (s['race_out'][1] + inset - T, s['race_out'][1] + inset)
        s['shoulder_end'] = s['head'][0] - q['bolt_shoulder_l_mm']
        s['thread_tip'] = s['shoulder_end'] - q['bolt_thread_l_mm']
        s['bearing_x'] = (s['race_out'][1] - B/2, s['race_out'][0] + B/2)
        s['center_x'] = sum(s['bearing_x']) / 2
        return s
    for name, length in (('ear_out', bt), ('spacer_out', q['spacer_outer_mm']), ('race_out', B),
                         ('spacer_mid', q['spacer_mid_mm']), ('race_in', B), ('spacer_in', q['spacer_inner_mm']),
                         ('ear_in', bt), ('nut', q['nut_h_mm'])):
        s[name] = (x - length, x)
        x -= length
    s['holder_out'] = (s['race_out'][1] + inset - T, s['race_out'][1] + inset)
    s['holder_in'] = (s['race_in'][0] - inset, s['race_in'][0] - inset + T)
    s['shoulder_end'] = s['head'][0] - q['bolt_shoulder_l_mm']
    s['thread_tip'] = s['shoulder_end'] - q['bolt_thread_l_mm']
    s['bearing_x'] = tuple(sum(s[k]) / 2 for k in ('race_out', 'race_in'))
    s['center_x'] = sum(s['bearing_x']) / 2
    return s


def pivot_of(d, key):
    """(x, y, z) of the pivot axis and the horizontal arm length to the reference wheel.

    Hinge keys: axis +Y. Bogie keys: axis +X through (y, z); x is the bearing-pair centre (reference only)."""
    side = side_of(d, key)
    if is_bogie_key(d, key):
        q = d.layout['suspension']['bogie']
        return (side * bogie_stack(d)['center_x'], q['pivot_y_mm'], q['pivot_z_mm']), bogie_lengths(d)[0]
    if key in DRIVE_WHEELS:
        q = d.layout['suspension']['drive']
        px = side * q['pivot_x_mm']
        return (px, 0.0, q['pivot_z_mm']), d.p['drive_track'] / 2 - q['pivot_x_mm']
    q = d.layout['suspension']['caster']
    x, y = caster_xy(d, key)
    return (x - side * q['arm_length_mm'], y, q['pivot_z_mm']), q['arm_length_mm']


def motion_angle(d, key, value):
    """Rotation (radians); positive value raises the reference wheel.

    Hinge: about +Y through the pivot. Bogie: about +X through the pivot (front caster up = +angle)."""
    _, length = pivot_of(d, key)
    if is_bogie_key(d, key):
        return asin(value / length)
    return -side_of(d, key) * asin(value / length)


def rotate_point(d, key, value, point):
    """Hinge keys: (x, z) of a point on the moving arm after the pose; Y is unchanged."""
    (px, _, pz), _ = pivot_of(d, key)
    a = motion_angle(d, key, value)
    rx, rz = point[0] - px, point[1] - pz
    return px + rx * cos(a) + rz * sin(a), pz - rx * sin(a) + rz * cos(a)


def rotate_yz(d, key, value, point):
    """Bogie keys: (y, z) of a point on the bogie after the pose; X is unchanged."""
    (_, py, pz), _ = pivot_of(d, key)
    a = motion_angle(d, key, value)
    ry, rz = point[0] - py, point[1] - pz
    return py + ry * cos(a) - rz * sin(a), pz + ry * sin(a) + rz * cos(a)


def seat_travel(d, key, value, x, z):
    return rotate_point(d, key, value, (x, z))[1] - z


def pose_shape(d, shape, key, value):
    if not value or key is None:
        return shape
    pivot, _ = pivot_of(d, key)
    if is_bogie_key(d, key):
        return shape.rotate(Axis((0, pivot[1], pivot[2]), (1, 0, 0)), degrees(motion_angle(d, key, value)))
    return shape.rotate(Axis(pivot, (0, 1, 0)), degrees(motion_angle(d, key, value)))
