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

def test_official_caster_height_and_rev10_motion_contract():
    d=load_design();r=build_records(d,['caster_lf'])
    assert len(r)==3
    assert max(x.shape.bounding_box().max.Z for x in r)==pytest.approx(83,abs=1e-5)
    assert validate(r,d)['valid']
    d.motion={'REAR':2,'LEFT':-10}
    r=build_records(d,['caster_lf','caster_lb','motor_left','motor_right'])
    assert validate(r,d)['valid']
    contacts={(x.metadata.get('caster') or x.metadata.get('moving_wheel')):x.shape.bounding_box().min.Z
              for x in r if x.name.startswith(('30_','44_'))}
    # Swing-arm camber lowers the tire edge 1.09 mm below the wheel-centre travel.
    assert contacts==pytest.approx({'LF':0,'LB':2.004831,'LEFT':-11.085242,'RIGHT':0},abs=1e-4)

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
    drive=report['drive']
    assert drive['sku']=='X-SWU17-70' and drive['installed_length_mm']==40
    assert drive['forces_N'][0]>0 and drive['min_solid_margin_mm']>0 and drive['max_working_deflection_margin_mm']>0
    assert 90<drive['estimated_ground_force_N']<120
    for case in report['static_cases'].values():
        assert 0<case['caster_capacity_ratio']<.5


def test_rev10_arms_rotate_about_fixed_pins_and_meet_stops():
    from build123d import CenterOf
    from cad.caster_motion import half_travel, pose_shape, drive_pivot, rear_pivot

    d=load_design()
    nominal={r.name:r for r in build_records(d,['suspension'])}
    for key in ('LEFT','RIGHT'):
        pin=nominal['80_drive_hinge_pin_'+key]
        assert pin.metadata['moving_wheel'] is None
        c=pin.shape.center(CenterOf.MASS)
        assert (c.X,c.Z)==pytest.approx((drive_pivot(d,key)[0],drive_pivot(d,key)[2]),abs=1e-6)
        arm=nominal['80_drive_swing_arm_6t_bent_'+key].shape
        washer=nominal['80_drive_droop_stop_washer_nut_'+key].shape
        sleeve=nominal['80_drive_bump_stop_sleeve_'+key].shape
        for value,stop in ((-10,washer),(10,sleeve)):
            at=pose_shape(d,arm,key,value)&stop
            assert at is None or at.volume<.01
            beyond=pose_shape(d,arm,key,value*1.15)&stop
            assert beyond is not None and beyond.volume>.1
    pin=nominal['80_caster_rear_hinge_pin_REAR'].shape.center(CenterOf.MASS)
    assert (pin.X,pin.Z)==pytest.approx((rear_pivot(d)[0],rear_pivot(d)[2]),abs=1e-6)
    beam=nominal['80_caster_rear_roll_beam_6t_REAR'].shape
    half=half_travel(d,'REAR')
    for value,name in ((half,'L'),(-half,'R')):
        stop=nominal['80_caster_rear_roll_stop_'+name+'_REAR'].shape
        at=pose_shape(d,beam,'REAR',value)&stop
        assert at is None or at.volume<.01
        beyond=pose_shape(d,beam,'REAR',value*1.3)&stop
        assert beyond is not None and beyond.volume>.1
    for wheel in ('LF','RF'):
        assert nominal['80_caster_fixed_box_bracket_'+wheel].metadata['moving_wheel'] is None
