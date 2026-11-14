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

def test_official_caster_height_and_bogie_motion_contract():
    """Rev15: the front caster rides on the bogie (motion key LEFT); the drive wheel moves the other way."""
    from math import asin, cos, sin
    d=load_design();r=build_records(d,['caster_lf'])
    assert len(r)==3
    assert max(x.shape.bounding_box().max.Z for x in r)==pytest.approx(83,abs=1e-5)
    assert validate(r,d)['valid']
    q=d.layout['suspension']['bogie'];yp,zp=q['pivot_y_mm'],q['pivot_z_mm'];a=asin(10/(d.p['caster_y']-yp))
    d.motion={'LEFT':10,'LB':5}
    r=build_records(d,['caster_lf','caster_lb','motor_left','motor_right'])
    assert validate(r,d)['valid']
    contacts={x.name:x.shape.bounding_box().min.Z for x in r if x.name.startswith(('30_','44_'))}
    # Drive wheel centre (0, 90) about the pivot (60, 90): contact = -60 sin(a).
    assert contacts['30_NBR_OD180_W42p5_LEFT']==pytest.approx(-yp*sin(a),abs=1e-4)
    assert contacts['30_NBR_OD180_W42p5_RIGHT']==pytest.approx(0,abs=1e-6)
    lf=next(x for x in r if x.name=='44_GL65_OEM_wheel_LF')
    y,z=lf.metadata['nominal_center_yz']
    assert contacts['44_GL65_OEM_wheel_LF']==pytest.approx(zp+(y-yp)*sin(a)+(z-zp)*cos(a)-32,abs=.05)
    # The OEM wheel trails 22 mm behind the swivel, so it rises a little less than the mounting centre.
    assert 8<contacts['44_GL65_OEM_wheel_LF']<10 and contacts['44_GL65_OEM_wheel_LB']>4


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


def test_bogie_statics_and_rear_spring_within_catalog():
    from cad.validation import catalog_suspension_report

    report=catalog_suspension_report(load_design())
    assert report['concept'].startswith('Rev15 bogie')
    nominal=report['static_cases']['with_arm']
    assert nominal['drive_load_fraction']>0.5
    total=2*(nominal['drive_each_N']+nominal['front_caster_each_N']+nominal['rear_caster_each_N'])
    assert total==pytest.approx(55*9.80665,abs=.05)
    rear=report['rear_caster_spring']
    assert rear['sku']=='X-SWU17-35' and rear['forces_N'][0]>0 and rear['min_solid_margin_mm']>0 and rear['max_working_deflection_margin_mm']>0
    assert abs(rear['level_shim_mm']-rear['configured_shim_mm'])<.25 and abs(rear['rear_sink_with_arm_mm'])<1
    for case in report['static_cases']['cg_sensitivity']:
        assert case['rear_caster_each_N']>0


def test_rev12_hinge_modules_meet_both_stops():
    """Rev15: the rear casters keep the hinge modules; each meets its droop and bump stops."""
    from build123d import CenterOf
    from cad.caster_motion import pivot_of, pose_shape, hinge_wheels

    d=load_design()
    assert hinge_wheels(d)==('LB','RB')
    parts={r.name:r for r in build_records(d,['suspension','chassis'])}
    base=next(r for n,r in parts.items() if n.startswith('01_')).shape
    for wheel in hinge_wheels(d):
        half=d.layout['suspension']['caster']['travel_mm']/2
        pin=parts['80_caster_hinge_pin_'+wheel]
        assert pin.metadata['moving_wheel'] is None
        c=pin.shape.center(CenterOf.MASS);(px,_,pz),_=pivot_of(d,wheel)
        assert (c.X,c.Z)==pytest.approx((px,pz),abs=1e-6)
        bush=parts['80_caster_hinge_bushes_'+wheel]
        assert bush.metadata['moving_wheel']==wheel and len(bush.shape.solids())==2
        arm=parts['80_caster_swing_arm_3t_'+wheel].shape
        nut=parts['80_caster_droop_stop_washer_nut_'+wheel].shape
        stop=parts['80_caster_bump_stop_bolt_M6_'+wheel].shape
        stopper=parts['80_caster_bump_stopper_C-USLH10_'+wheel].shape
        assert arm&stop is None or (arm&stop).volume<.01
        for shape,other,limit in ((nut,base,-half),(stopper,stop,half)):
            at=pose_shape(d,shape,wheel,limit)&other
            assert at is None or at.volume<.01
            beyond=pose_shape(d,shape,wheel,limit*1.2)&other
            assert beyond is not None and beyond.volume>.1
    assert not any(n.startswith(('80_drive_','80_caster_swing_arm_3t_LF','80_caster_swing_arm_3t_RF')) for n in parts)


def test_rev15_bogie_link_pivot_and_rotation_limits():
    """One 3t bent link per side on an EA-MFCNW20 bush and D20 shaft; the stoppers meet the strikers at +-10 mm."""
    from build123d import CenterOf
    from cad.caster_motion import pose_shape

    d=load_design();q=d.layout['suspension']['bogie'];half=q['travel_mm']/2
    parts={r.name:r for r in build_records(d,['suspension','chassis'])}
    for key,s in (('LEFT',-1),('RIGHT',1)):
        link=parts['80_bogie_link_3t_'+key]
        assert link.shape.is_valid and len(link.shape.solids())==1 and link.metadata['moving_wheel']==key
        b=link.shape.bounding_box()
        assert (b.min.Y,b.max.Y)==pytest.approx((q['rear_y_mm'],q['front_y_mm']))
        assert b.max.Z<d.p['base_bottom_z']-4
        shaft=parts['80_bogie_pivot_shaft_D20_'+key]
        c=shaft.shape.center(CenterOf.MASS)
        assert (c.Y,c.Z)==pytest.approx((q['pivot_y_mm'],q['pivot_z_mm']),abs=1e-3) and shaft.metadata['moving_wheel'] is None
        bush=parts['80_bogie_pivot_bush_EA-MFCNW20_'+key]
        assert bush.metadata['moving_wheel']==key and bush.metadata['supply']=='company stock'
        bb=bush.shape.bounding_box();assert bb.size.X==pytest.approx(77)
        assert bush.shape&shaft.shape is None or (bush.shape&shaft.shape).volume<1e-6
        for tag,limit in (('front',half),('rear',-half)):
            stopper=parts[f'80_bogie_{tag}_stopper_C-USLH10_'+key].shape
            striker=parts[f'80_bogie_{tag}_striker_M6_'+key].shape
            at=pose_shape(d,stopper,key,limit)&striker
            assert at is None or at.volume<.01
            beyond=pose_shape(d,stopper,key,limit*1.2)&striker
            assert beyond is not None and beyond.volume>.1
        # CB5-12 stacks end about 2 mm past their nuts.
        for bolts,nuts,axis in (('bogie_bracket_bolts','bogie_bracket_nuts','Z'),('bogie_flange_bolts_CB5-20','bogie_flange_nuts_M5','X'),('bogie_splice_bolts_CB5-12','bogie_splice_nuts_M5','X')):
            bo=parts['80_'+bolts+'_'+key].shape.bounding_box();nu=parts['80_'+nuts+'_'+key].shape.bounding_box()
            past=min(getattr(nu.min,axis)-getattr(bo.min,axis),getattr(bo.max,axis)-getattr(nu.max,axis))
            assert 0<past<=2.5,(bolts,key,past)


def test_rev12b_misumi_step_parts_and_static_clearance():
    """MISUMI STEP pins, bushes and C-USLH10 stoppers on the rear hinge modules; nothing near the suspension overlaps."""
    from cad.validation import static_interference

    d=load_design()
    records=build_records(d,['chassis','structure','lidars','suspension','electronics'])
    parts={r.name:r for r in records}
    for wheel in ('LB','RB'):
        for name in ('hinge_pin','hinge_bushes','bump_stopper_C-USLH10'):
            part=parts['80_caster_'+name+'_'+wheel]
            assert part.status=='misumi_step_exact' and part.shape.is_valid
        stopper=parts['80_caster_bump_stopper_C-USLH10_'+wheel].shape.bounding_box()
        assert stopper.size.Z==pytest.approx(24) and stopper.size.X==pytest.approx(10)
        assert parts['80_caster_spring_coil_'+wheel].shape.is_valid
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
    line=lambda r:int(r[col['Line total KRW ex VAT']] or 0)
    assert totals['MISUMI priced items']==sum(line(r) for r in items if r[col['Price basis']]=='MISUMI price')
    assert totals['Estimated items']==sum(line(r) for r in items if r[col['Price basis']].startswith('Estimate'))
    total=next(k for k in totals if k.startswith('Suspension total'))
    assert totals[total]==totals['MISUMI priced items']+totals['Estimated items']
    for r in items:
        if r[col['Unit price KRW ex VAT']]:
            assert line(r)==int(r[col['Unit price KRW ex VAT']])*int(r[col['Order qty']]),r[2]
        if r[col['Price basis']]=='Company stock':
            assert line(r)==0,r[2]


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
    for name in ('80_caster_plate_bolts_LB','80_caster_plate_bolts_RB','80_bogie_caster_plate_bolts_LEFT','80_bogie_caster_plate_bolts_RIGHT'):
        bolts=parts[name].shape.bounding_box()
        assert bolts.min.Z==pytest.approx(plate-6) and bolts.max.Z==pytest.approx(plate+12)


def test_rev14_bend_limit_fastener_length_and_lidar_bracket():
    """Bent sheet parts are at most 3t; bolts end about 2 mm past their nuts; each LiDAR hangs on a main column."""
    d=load_design();s=d.layout['suspension']
    assert s['caster']['arm_thickness_mm']<=3 and s['caster']['bracket_thickness_mm']<=3
    assert s['bogie']['sheet_t_mm']<=3 and s['bogie']['bracket_t_mm']<=3
    records=build_records(d,['suspension','lidars','structure','chassis'])
    parts={r.name:r for r in records}
    for wheel in ('LB','RB'):
        bolts=parts['80_caster_bracket_bolts_'+wheel].shape.bounding_box()
        nuts=parts['80_caster_bracket_nuts_'+wheel].shape.bounding_box()
        assert 0<nuts.min.Z-bolts.min.Z<=2.5
        assert len(parts['80_caster_pivot_bosses_8t_'+wheel].shape.solids())==2
    for prefix in ('80_caster_plate_','80_bogie_caster_plate_'):
        for wheel in (('LB','RB') if prefix=='80_caster_plate_' else ('LEFT','RIGHT')):
            bolts=parts[prefix+'bolts_'+wheel].shape.bounding_box()
            nuts=parts[prefix+'nuts_'+wheel].shape.bounding_box()
            assert 0<bolts.max.Z-nuts.max.Z<=2.5
    for wheel in ('LB','RB'):
        # 54 mm hole pitch: holes keep at least 5 mm of edge on the caster pad.
        arm=parts['80_caster_swing_arm_3t_'+wheel].shape.bounding_box()
        assert arm.size.Y>=2*(27+3.3+5)
    assert not any(n.startswith('64_') for n in parts)
    for n in ('FRONT_LEFT','REAR_RIGHT'):
        assert parts['65_lidar_bracket_bolts_M5_Tnut_'+n].metadata['quantity']==4
        assert parts['60_lidar_bracket_3t_'+n].shape.is_valid


def test_rev15_company_parts_placed_below_the_scan_plane():
    """Battery, LCD and speaker are the company-supplied STEP files; all stay below the LiDAR scan plane."""
    d=load_design();scan=d.layout['lidars'][0]['scan_z']
    parts={r.name:r for r in build_records(d,['electronics'])}
    for name in ('50_company_7S9P_battery_STEP','56_company_LCD_7in_STEP','57_company_speaker_STEP'):
        part=parts[name]
        assert part.status=='company_supplied_step' and part.shape.is_valid
        assert part.shape.bounding_box().max.Z<scan
    case=parts['50_company_7S9P_battery_STEP'].metadata['placement']['case_mm']
    assert case==pytest.approx([274.79,153.83,88.7],abs=.02)
    assert parts['57_company_speaker_STEP'].metadata['trimmed_solids']>=1
    lcd=parts['56_company_LCD_7in_STEP'].shape.bounding_box()
    assert lcd.max.Y==pytest.approx(d.layout['electronics']['lcd_front_y']) and lcd.min.Z==pytest.approx(d.layout['electronics']['lcd_bottom_z'])
