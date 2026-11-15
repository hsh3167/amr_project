"""P30 profile chassis: fast checks on the frame, plates and drive modules (no large OEM STEP files)."""
import pytest
from cad.p30.build import P30, build_records
from cad.p30 import check


@pytest.fixture(scope='module')
def light():
    P = P30()
    return P, build_records(P, ['frame', 'plates', 'drive_left', 'drive_right'])


def test_p30_shapes_valid(light):
    _, records = light
    assert check.geometry(records) == []


def test_p30_springs_and_heights(light):
    P, records = light
    s = check.springs(P)
    assert s['pass']
    assert 60 < s['per_wheel']['-10']['force_N'] < s['per_wheel']['+0']['force_N'] < s['per_wheel']['+10']['force_N'] < 200
    tires = [r.shape.bounding_box().min.Z for r in records if r.name.startswith('30_NBR')]
    assert len(tires) == 2 and all(abs(z) < .05 for z in tires)
    # tire stays inside the frame outline
    assert max(r.shape.bounding_box().max.X for r in records if r.name.startswith('30_NBR')) < P.L['frame']['outer_x']


def test_p30_stroke_end_stops():
    gaps = check.stroke_contacts(P30())
    assert all(v <= .1 for v in gaps.values())


def test_p30_drive_stroke_clear_of_frame(light):
    P, records = light
    result = check.stroke_interference(P, records)
    assert result['poses'] == 8 and result['hits'] == []


def test_p30_guide_parts_are_misumi_steps(light):
    P, records = light
    names = {r.name: r for r in records}
    for n in ('LEFT', 'RIGHT'):
        bush = names['39_linear_bush_'+P.L['drive']['bush']['sku']+'_'+n]; shaft = names['50_guide_shaft_'+P.L['drive']['shaft']['sku']+'_'+n]
        assert bush.status == shaft.status == 'misumi_step_exact'
        assert len(bush.shape.solids()) == len(shaft.shape.solids()) == 2
    st = check.statics(P, records)
    assert st['bush_load_ratio_dynamic'] < .5 and st['slide_friction_vs_spring_pct'] < 5


def test_p30_plates_are_thin_steel(light):
    P, records = light
    pl = P.L['plates']; q = P.L['drive']
    assert pl['corner_plate_t'] <= 6 and pl['floor_t'] <= 2 and pl['deck_t'] <= 2.3 and q['holder_t'] <= 6 and q['carriage_t'] <= 6
    fab = [r for r in records if r.name[:3] in ('11_', '12_', '13_', '36_', '51_', '52_')]
    assert fab and all(r.metadata['material'].startswith('SS400') for r in fab)
    # carriage and axle wall are one bent part; the old bolted pair is gone
    assert not any(r.name.startswith(('37_', '38_')) for r in records)
