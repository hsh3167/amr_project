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
        assert parts['80_'+kind+'_swing_arm_3t_'+wheel].metadata['moving_wheel']==wheel
        bush=parts['80_'+kind+'_hinge_bushes_'+wheel]
        assert bush.metadata['moving_wheel']==wheel and len(bush.shape.solids())==2
        assert bush.shape&pin.shape is None or (bush.shape&pin.shape).volume<1e-6
        arm=parts['80_'+kind+'_swing_arm_3t_'+wheel].shape
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
    """Rev14: 3t channel arm, still asymmetric (-Y spring/stop side) with the CLBKHM10-78.5 pin span."""
    d=load_design()
    parts={r.name:r for r in build_records(d,['suspension'])}
    for wheel in ('LEFT','RIGHT'):
        arm=parts['80_drive_swing_arm_3t_'+wheel].shape
        assert arm.is_valid and len(arm.solids())==1 and arm.volume<65000
        b=arm.bounding_box()
        assert b.min.Y==pytest.approx(-44) and b.max.Y==pytest.approx(22.5)
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


def test_bom_csv_columns_and_totals():
    """Every BOM row parses to the header width and the total rows match the line sums (viewer reads these files)."""
    import csv
    from cad.parameters import ROOT

    for name in ('suspension_misumi.csv','concept_bom.csv'):
        rows=list(csv.reader((ROOT/'bom'/name).open(newline='',encoding='utf-8')))
        assert all(len(r)==len(rows[0]) for r in rows),name
    head,*rows=list(csv.reader((ROOT/'bom/suspension_misumi.csv').open(newline='',encoding='utf-8')))
    col={h:i for i,h in enumerate(head)}
    items=[r for r in rows if r[0]!='Total'];totals={r[2]:int(r[col['Line total KRW ex VAT']]) for r in rows if r[0]=='Total'}
    line=lambda r:int(r[col['Line total KRW ex VAT']])
    assert totals['MISUMI priced items']==sum(line(r) for r in items if r[col['Price basis']]=='MISUMI price')
    assert totals['Estimated items']==sum(line(r) for r in items if r[col['Price basis']].startswith('Estimate'))
    assert totals['Suspension total (6 modules)']==totals['MISUMI priced items']+totals['Estimated items']
    for r in items:
        if r[col['Unit price KRW ex VAT']]:
            assert line(r)==int(r[col['Unit price KRW ex VAT']])*int(r[col['Order qty']]),r[2]


def test_rev13_every_part_is_supported_and_caster_bolts_bear_on_the_plate():
    """No floating envelopes or fastener stacks: each part reaches the base plate through contacts."""
    from cad.validation import support_connectivity
    from cad.assembly import GROUPS

    d=load_design()
    records=build_records(d,[g for g in GROUPS if g!='cameras'])
    report=support_connectivity(records)
    assert report['unsupported']==[],report['unsupported']
    parts={r.name:r for r in records}
    assert not any(n.startswith(('11_corner_lower_','12_corner_upper_')) for n in parts)
    plate=d.parts['caster']['cad_height']-d.parts['caster']['plate_thickness_mm']
    for wheel in ('LF','LB','RF','RB'):
        bolts=parts['80_caster_plate_bolts_'+wheel].shape.bounding_box()
        assert bolts.min.Z==pytest.approx(plate-6) and bolts.max.Z==pytest.approx(plate+12)


def test_rev14_bend_limit_fastener_length_and_lidar_bracket():
    """Bent sheet parts are at most 3t; bolts end about 2 mm past their nuts; each LiDAR hangs on a main column."""
    d=load_design();s=d.layout['suspension']
    for kind in ('drive','caster'):
        assert s[kind]['arm_thickness_mm']<=3 and s[kind]['bracket_thickness_mm']<=3
    records=build_records(d,['suspension','lidars','structure','chassis'])
    parts={r.name:r for r in records}
    for wheel in ('LEFT','RIGHT','LF','LB','RF','RB'):
        kind='drive' if wheel in ('LEFT','RIGHT') else 'caster'
        bolts=parts['80_'+kind+'_bracket_bolts_'+wheel].shape.bounding_box()
        nuts=parts['80_'+kind+'_bracket_nuts_'+wheel].shape.bounding_box()
        assert 0<nuts.min.Z-bolts.min.Z<=2.5
        assert len(parts['80_'+kind+'_pivot_bosses_8t_'+wheel].shape.solids())==2
    for wheel in ('LF','LB','RF','RB'):
        bolts=parts['80_caster_plate_bolts_'+wheel].shape.bounding_box()
        nuts=parts['80_caster_plate_nuts_'+wheel].shape.bounding_box()
        assert 0<bolts.max.Z-nuts.max.Z<=2.5
        # 54 mm hole pitch: holes keep at least 5 mm of edge on the caster pad.
        arm=parts['80_caster_swing_arm_3t_'+wheel].shape.bounding_box()
        assert arm.size.Y>=2*(27+3.3+5)
    assert not any(n.startswith('64_') for n in parts)
    for n in ('FRONT_LEFT','REAR_RIGHT'):
        assert parts['65_lidar_bracket_bolts_M5_Tnut_'+n].metadata['quantity']==4
        assert parts['60_lidar_bracket_3t_'+n].shape.is_valid
