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
    # Hinge arms tilt the wheels: the tire edge drops 1.35 mm below the wheel-centre travel.
    assert contacts==pytest.approx({'LF':5.12278,'LEFT':-11.353133,'RIGHT':0},abs=1e-4)

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
    assert report['modules']['drive']['sku']=='X-SWU21-70' and report['modules']['caster']['sku']=='X-SWU17-35'
    for module in report['modules'].values():
        assert module['forces_N'][0]>0 and module['min_solid_margin_mm']>0 and module['max_working_deflection_margin_mm']>0
    assert report['modules']['drive']['lever_ratio']<1 and report['modules']['caster']['lever_ratio']<1
    cases=report['static_cases']
    assert cases['with_arm']['within_travel'] and abs(cases['with_arm']['body_sink_mm'])<1
    assert cases['without_arm_shims_removed']['within_travel']
    assert not cases['without_arm_shims_kept']['within_travel']


def test_rev12_hinge_modules_meet_both_stops():
    from build123d import CenterOf
    from cad.caster_motion import pivot_of, pose_shape

    d=load_design()
    parts={r.name:r for r in build_records(d,['suspension','chassis'])}
    base=next(r for n,r in parts.items() if n.startswith('01_')).shape
    for wheel in ('LEFT','RIGHT','LF','LB','RF','RB'):
        kind='drive' if wheel in ('LEFT','RIGHT') else 'caster'
        half=d.layout['suspension'][kind]['travel_mm']/2
        pin=parts['80_'+kind+'_hinge_pin_'+wheel]
        assert pin.metadata['moving_wheel'] is None
        c=pin.shape.center(CenterOf.MASS);(px,_,pz),_=pivot_of(d,wheel)
        assert (c.X,c.Z)==pytest.approx((px,pz),abs=1e-6)
        assert parts['80_'+kind+'_swing_arm_6t_'+wheel].metadata['moving_wheel']==wheel
        bush=parts['80_'+kind+'_hinge_bushes_'+wheel]
        assert bush.metadata['moving_wheel']==wheel and len(bush.shape.solids())==2
        assert bush.shape&pin.shape is None or (bush.shape&pin.shape).volume<1e-6
        arm=parts['80_'+kind+'_swing_arm_6t_'+wheel].shape
        nut=parts['80_'+kind+'_droop_stop_washer_nut_'+wheel].shape
        stop=parts['80_'+kind+'_bump_stop_bolt_M6_'+wheel].shape
        # Rev12b: the C-USLH10 urethane tip on the arm meets the M6 striker bolt.
        stopper=parts['80_'+kind+'_bump_stopper_C-USLH10_'+wheel].shape
        assert arm&stop is None or (arm&stop).volume<.01
        for shape,other,limit in ((nut,base,-half),(stopper,stop,half)):
            at=pose_shape(d,shape,wheel,limit)&other
            assert at is None or at.volume<.01
            beyond=pose_shape(d,shape,wheel,limit*1.2)&other
            assert beyond is not None and beyond.volume>.1


def test_rev12a_compact_drive_arm():
    """Drive arm keeps only short pivot tabs; the module is asymmetric and the pin is no longer at catalog L100."""
    d=load_design()
    parts={r.name:r for r in build_records(d,['suspension'])}
    for wheel in ('LEFT','RIGHT'):
        arm=parts['80_drive_swing_arm_6t_'+wheel].shape
        assert arm.is_valid and len(arm.solids())==1 and arm.volume<65000
        b=arm.bounding_box()
        assert b.min.Y==pytest.approx(-42.5) and b.max.Y==pytest.approx(21)
        pin=parts['80_drive_hinge_pin_'+wheel]
        assert pin.metadata['sku']=='CLBKHM10-'+format(pin.metadata['length_mm'],'g') and pin.metadata['length_mm']<100
        stop=parts['80_drive_bump_stop_bolt_M6_'+wheel].shape.bounding_box().center()
        assert stop.Y==pytest.approx(d.layout['suspension']['drive']['bump_stop_y_mm'])


def test_rev12b_misumi_step_parts_and_static_clearance():
    """Pins, bushes and C-USLH10 stoppers are the supplied MISUMI STEP; nothing near the suspension overlaps."""
    from cad.validation import static_interference

    d=load_design()
    records=build_records(d,['chassis','structure','lidars','suspension'])
    parts={r.name:r for r in records}
    for wheel in ('LEFT','RIGHT','LF','LB','RF','RB'):
        kind='drive' if wheel in ('LEFT','RIGHT') else 'caster'
        for name in ('hinge_pin','hinge_bushes','bump_stopper_C-USLH10'):
            part=parts['80_'+kind+'_'+name+'_'+wheel]
            assert part.status=='misumi_step_exact' and part.shape.is_valid
        stopper=parts['80_'+kind+'_bump_stopper_C-USLH10_'+wheel].shape.bounding_box()
        assert stopper.size.Z==pytest.approx(24) and stopper.size.X==pytest.approx(10)
        assert parts['80_'+kind+'_spring_coil_'+wheel].shape.is_valid
    report=static_interference(records)
    assert report['hits']==[],report['hits']
