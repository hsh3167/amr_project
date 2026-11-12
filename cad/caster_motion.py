"""Rev11 suspension kinematics: six independent vertical rail-guided modules.

Motion keys: LEFT / RIGHT drive wheels (+-10 mm) and LF / LB / RF / RB casters (+-5 mm).
A positive value raises the wheel (bump); every moving part translates in +Z.
"""

CASTER_WHEELS = ('LF', 'LB', 'RF', 'RB')
DRIVE_WHEELS = ('LEFT', 'RIGHT')
MOTION_KEYS = DRIVE_WHEELS + CASTER_WHEELS


def caster_xy(d, wheel):
    return (d.p['caster_x'] * (-1 if wheel[0] == 'L' else 1),
            d.p['caster_y'] * (1 if wheel[1] == 'F' else -1))


def caster_motion_key(wheel):
    return wheel


def half_travel(d, key):
    kind = 'drive' if key in DRIVE_WHEELS else 'caster'
    return d.layout['suspension'][kind]['travel_mm'] / 2


def pose_shape(d, shape, key, value):
    if not value or key is None:
        return shape
    return shape.translate((0, 0, value))
