import pytest
from cad.parameters import load_design
from cad.assembly import build_records
from cad.validation import validate,compare_baseline

@pytest.mark.parametrize('group',['chassis','payload_interface','structure','motor_left','caster_lf','lidars','cameras'])
def test_partial_generation_matches_baseline(group):
    d=load_design('baseline');records=build_records(d,[group])
    assert records and all(x.group==group for x in records)
    assert validate(records,d)['valid']
    assert compare_baseline(records,symmetric=False)['pass']

def test_full_component_count_and_contacts():
    d=load_design('baseline');records=build_records(d)
    report=validate(records,d,full=True)
    assert len(records)==85
    assert report['valid'],report

def test_official_caster_height_and_six_wheel_motion_contract():
    d=load_design();r=build_records(d,['caster_lf'])
    assert len(r)==3
    assert max(x.shape.bounding_box().max.Z for x in r)==pytest.approx(83,abs=1e-5)
    assert validate(r,d)['valid']
    d.motion={'LF':5,'LEFT':-10}
    r=build_records(d,['caster_lf','motor_left','motor_right'])
    assert validate(r,d)['valid']
    contacts={x.metadata.get('moving_wheel'):x.shape.bounding_box().min.Z for x in r if x.name.startswith(('30_','44_'))}
    assert contacts==pytest.approx({'LF':3.970043805,'LEFT':-10,'RIGHT':0},abs=1e-5)

def test_cached_multisolid_component_retains_name_and_geometry(tmp_path):
    from build123d import Compound,import_step
    from cad.parts.common import PartRecord,box
    from cad.export import write
    shape=Compound([box(10,10,10),box(10,10,10,20,0,0)])
    record=PartRecord('OEM_reference','fixture',shape)
    write([record],tmp_path,formats=('step',),name='cache_roundtrip')
    result=import_step(tmp_path/'cache_roundtrip.step')
    assert result.is_valid and len(result.solids())==2
    assert sum(s.volume for s in result.solids())==pytest.approx(2000)
    assert result.children[0].children[0].label=='OEM_reference'

def test_individual_step_contains_geometry_after_assembly_export(tmp_path):
    from build123d import import_step
    from cad.parts.common import PartRecord,box
    from cad.export import write

    record=PartRecord('mount','fixture',box(10,20,30))
    write([record],tmp_path,individual=True)
    part=import_step(tmp_path/'parts/mount.step')
    assert part.label=='mount'
    assert part.is_valid and len(part.solids())==1
    assert part.solids()[0].volume==pytest.approx(6000)


def test_misumi_spring_stack_stays_within_catalog_travel():
    from cad.validation import catalog_suspension_report

    report=catalog_suspension_report(load_design())
    assert report['target_mass_kg']==30
    assert report['modules']['drive']['spring_lengths_mm']==[67,57,47]
    assert report['modules']['caster']['spring_lengths_mm']==[29.5,24.5,19.5]
    for module in report['modules'].values():
        assert module['forces_N'][0]>0
        assert module['min_solid_margin_mm']>0
        assert module['max_working_deflection_margin_mm']>0
    assert abs(report['estimated_static_offset_mm'])<1


def test_caster_rockers_rotate_about_fixed_pins():
    from build123d import CenterOf

    from cad.caster_motion import pose_shape, rocker_pivot

    d=load_design()
    nominal=build_records(d,['suspension'])
    d.motion={'LF':5,'RB':-5}
    records=build_records(d,['suspension'])
    old={r.name:r for r in nominal}
    parts={r.name:r for r in records}
    for wheel in ('LF','LB','RF','RB'):
        pin=parts['80_caster_hinge_pin_'+wheel]
        bush=parts['80_caster_pivot_bushings_'+wheel]
        arm=parts['80_caster_rocker_arm_'+wheel]
        assert pin.metadata['moving_wheel'] is None
        assert len(bush.shape.solids())==2
        assert tuple(pin.shape.center(CenterOf.MASS))[1:]==pytest.approx(rocker_pivot(d,wheel)[1:])
        old_arm=old[arm.name].shape.center(CenterOf.MASS)
        moved_arm=arm.shape.center(CenterOf.MASS)
        if wheel in d.motion:
            assert moved_arm.Y!=pytest.approx(old_arm.Y)
            assert (moved_arm.Z-old_arm.Z)*d.motion[wheel]>0
        else:
            assert tuple(moved_arm)==pytest.approx(tuple(old_arm))
    arm=old['80_caster_rocker_arm_LF'].shape
    clevis=old['80_caster_clevis_with_stops_LF'].shape
    for sign in (-1,1):
        at_stop=pose_shape(d,arm,'LF',sign*5)&clevis
        assert at_stop is None or at_stop.volume<.01
        beyond_stop=pose_shape(d,arm,'LF',sign*5.25)&clevis
        assert beyond_stop is not None and beyond_stop.volume>.1
