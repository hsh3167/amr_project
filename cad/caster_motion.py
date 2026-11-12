"""Rev12 suspension kinematics: six independent hinge (swing-arm) modules.

Every pivot axis is parallel to +Y (direction of travel), so forward and reverse
behave the same. Motion keys: LEFT / RIGHT drive wheels (value = vertical travel
of the wheel centre, +-10 mm) and LF / LB / RF / RB casters (value = vertical
travel of the caster mounting centre, +-5 mm). Positive raises the wheel.
"""
from math import asin, cos, degrees, sin

from build123d import Axis

CASTER_WHEELS = ('LF', 'LB', 'RF', 'RB')
DRIVE_WHEELS = ('LEFT', 'RIGHT')
MOTION_KEYS = DRIVE_WHEELS + CASTER_WHEELS


def caster_xy(d, wheel):
    return (d.p['caster_x'] * (-1 if wheel[0] == 'L' else 1),
            d.p['caster_y'] * (1 if wheel[1] == 'F' else -1))


def caster_motion_key(wheel):
    return wheel


def side_of(d, key):
    if key in DRIVE_WHEELS:
        return -1 if key == 'LEFT' else 1
    return -1 if key[0] == 'L' else 1


def half_travel(d, key):
    kind = 'drive' if key in DRIVE_WHEELS else 'caster'
    return d.layout['suspension'][kind]['travel_mm'] / 2


def pivot_of(d, key):
    """(x, y, z) of the hinge axis and the horizontal arm length to the wheel/caster centre."""
    side = side_of(d, key)
    if key in DRIVE_WHEELS:
        q = d.layout['suspension']['drive']
        px = side * q['pivot_x_mm']
        return (px, 0.0, q['pivot_z_mm']), d.p['drive_track'] / 2 - q['pivot_x_mm']
    q = d.layout['suspension']['caster']
    x, y = caster_xy(d, key)
    return (x - side * q['arm_length_mm'], y, q['pivot_z_mm']), q['arm_length_mm']


def motion_angle(d, key, value):
    """Rotation (radians) about +Y through the pivot; positive value raises the wheel."""
    side = side_of(d, key)
    _, length = pivot_of(d, key)
    return -side * asin(value / length)


def rotate_point(d, key, value, point):
    """(x, z) of a point on the moving arm after the pose; Y is unchanged."""
    (px, _, pz), _ = pivot_of(d, key)
    a = motion_angle(d, key, value)
    rx, rz = point[0] - px, point[1] - pz
    return px + rx * cos(a) + rz * sin(a), pz - rx * sin(a) + rz * cos(a)


def seat_travel(d, key, value, x, z):
    return rotate_point(d, key, value, (x, z))[1] - z


def pose_shape(d, shape, key, value):
    if not value or key is None:
        return shape
    pivot, _ = pivot_of(d, key)
    return shape.rotate(Axis(pivot, (0, 1, 0)), degrees(motion_angle(d, key, value)))
