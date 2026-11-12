"""GL-65 rocker kinematics in the assembly's millimetre coordinate system."""
from math import asin, degrees

from build123d import Axis

CASTER_WHEELS = ('LF', 'LB', 'RF', 'RB')


def caster_xy(d, wheel):
    return (d.p['caster_x'] * (-1 if wheel[0] == 'L' else 1),
            d.p['caster_y'] * (1 if wheel[1] == 'F' else -1))


def rocker_pivot(d, wheel):
    x, y = caster_xy(d, wheel)
    q = d.layout['suspension']['caster']
    return x, y - (1 if y > 0 else -1) * q['arm_length_mm'], q['pivot_z_mm']


def rocker_angle(d, wheel, dz):
    length = d.layout['suspension']['caster']['arm_length_mm']
    return asin(dz / length) * (1 if wheel[1] == 'F' else -1)


def pose_shape(d, shape, wheel, dz):
    if not dz:
        return shape
    if wheel in CASTER_WHEELS:
        return shape.rotate(Axis(rocker_pivot(d, wheel), (1, 0, 0)), degrees(rocker_angle(d, wheel, dz)))
    return shape.translate((0, 0, dz))
