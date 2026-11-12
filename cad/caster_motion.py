"""Rev10 suspension kinematics in the assembly's millimetre coordinate system.

Motion keys:
  LEFT / RIGHT  drive side swing arm, value = vertical travel at the wheel centre radius (mm)
  REAR          rear transverse roll beam, value = vertical travel of the LEFT rear caster seat (mm)
Front casters (LF, RF) are fixed to the base plate.
"""
from math import asin, cos, degrees, sin

from build123d import Axis

CASTER_WHEELS = ('LF', 'LB', 'RF', 'RB')
REAR_CASTERS = ('LB', 'RB')
DRIVE_WHEELS = ('LEFT', 'RIGHT')
MOTION_KEYS = ('LEFT', 'RIGHT', 'REAR')


def caster_xy(d, wheel):
    return (d.p['caster_x'] * (-1 if wheel[0] == 'L' else 1),
            d.p['caster_y'] * (1 if wheel[1] == 'F' else -1))


def caster_motion_key(wheel):
    return 'REAR' if wheel in REAR_CASTERS else None


def half_travel(d, key):
    s = d.layout['suspension']
    if key in DRIVE_WHEELS:
        return s['drive']['travel_mm'] / 2
    q = s['rear_rocker']
    from math import radians, tan
    return round(d.p['caster_x'] * tan(radians(q['roll_limit_deg'])), 3)


def drive_pivot(d, key):
    side = -1 if key == 'LEFT' else 1
    q = d.layout['suspension']['drive']
    return side * q['pivot_x_mm'], 0.0, q['pivot_z_mm']


def drive_arm_length(d):
    return d.p['drive_track'] / 2 - d.layout['suspension']['drive']['pivot_x_mm']


def rear_pivot(d):
    return 0.0, -d.p['caster_y'], d.layout['suspension']['rear_rocker']['pivot_z_mm']


def motion_angle(d, key, value):
    """Rotation angle (radians) about +Y through the pivot for a motion value."""
    if key in DRIVE_WHEELS:
        side = -1 if key == 'LEFT' else 1
        return -side * asin(value / drive_arm_length(d))
    if key == 'REAR':
        # +value raises the left rear caster (x<0): rotation about +Y lifts -X.
        return asin(value / d.p['caster_x'])
    raise ValueError('Unknown motion key: ' + str(key))


def pivot_of(d, key):
    return drive_pivot(d, key) if key in DRIVE_WHEELS else rear_pivot(d)


def rotate_point(d, key, value, point):
    """Point (x, z) rotated by the pose; Y is unchanged."""
    px, _, pz = pivot_of(d, key)
    a = motion_angle(d, key, value)
    rx, rz = point[0] - px, point[1] - pz
    return px + rx * cos(a) + rz * sin(a), pz - rx * sin(a) + rz * cos(a)


def pose_shape(d, shape, key, value):
    if not value or key is None:
        return shape
    return shape.rotate(Axis(pivot_of(d, key), (0, 1, 0)), degrees(motion_angle(d, key, value)))


def seat_travel(d, key, value, x, z):
    """Vertical displacement of a point on the moving body."""
    return rotate_point(d, key, value, (x, z))[1] - z
