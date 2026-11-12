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
    assert contacts==pytest.approx({'LF':5,'LEFT':-10,'RIGHT':0},abs=1e-5)

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
    cases=report['static_cases']
    assert cases['with_arm']['within_travel'] and abs(cases['with_arm']['body_sink_mm'])<1
    assert cases['without_arm_shims_removed']['within_travel']
    assert not cases['without_arm_shims_kept']['within_travel']


def test_rev11_vertical_modules_meet_both_stops():
    d=load_design()
    parts={r.name:r for r in build_records(d,['suspension','chassis'])}
    base=next(r for n,r in parts.items() if n.startswith('01_'))
    for wheel in ('LEFT','RIGHT','LF','LB','RF','RB'):
        kind='drive' if wheel in ('LEFT','RIGHT') else 'caster'
        half=d.layout['suspension'][kind]['travel_mm']/2
        carrier=parts['80_'+kind+'_carrier_6t_bent_'+wheel].shape
        nut=parts['80_'+kind+'_droop_stop_washer_nut_'+wheel].shape
        sleeve=parts['80_'+kind+'_bump_stop_sleeve_'+wheel].shape
        top=parts['80_drive_preload_shim_'+wheel].shape if kind=='drive' else base.shape
        assert parts['80_'+kind+'_rail_'+wheel].metadata['moving_wheel'] is None
        assert parts['80_'+kind+'_rail_block_'+wheel].metadata['moving_wheel']==wheel
        for shape,other,limit in ((carrier,nut,-half),(sleeve,top,half)):
            at=shape.translate((0,0,limit))&other
            assert at is None or at.volume<.01
            beyond=shape.translate((0,0,limit*1.2))&other
            assert beyond is not None and beyond.volume>.1
