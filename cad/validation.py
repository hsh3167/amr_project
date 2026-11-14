"""Geometric checks only. Strength, tire compliance and safety remain separate."""
import math
from build123d import Axis, CenterOf, import_step
from .caster_motion import CASTER_WHEELS, DRIVE_WHEELS, MOTION_KEYS, caster_xy, half_travel, motion_angle, pose_shape, rotate_point
from .parameters import ROOT
from .validate_layout import coverage

def metrics(shape):
    b=shape.bounding_box();c=shape.center(CenterOf.MASS)
    return {'volume_mm3':sum(s.volume for s in shape.solids()),'bounds_mm':[[b.min.X,b.max.X],[b.min.Y,b.max.Y],[b.min.Z,b.max.Z]],
            'center_mm':[c.X,c.Y,c.Z], 'solids':len(shape.solids())}

def overlap(a,b,tol=1e-6):
    aa=a.bounding_box();bb=b.bounding_box()
    return bounds_overlap((tuple(aa.min),tuple(aa.max)),(tuple(bb.min),tuple(bb.max)),tol)

def bounds_overlap(a,b,tol=1e-6):
    return all(min(x,y)-max(u,v)>tol for x,y,u,v in zip(a[1],b[1],a[0],b[0]))

def suspension_collisions(records,d):
    """Five sampled poses per wheel (LEFT, RIGHT, LF, LB, RF, RB) from the nominal assembly."""
    hits=[];tested=0
    fixed=[r for r in records if not r.metadata.get('moving_wheel') and 'spring_length_mm' not in r.metadata]
    moving=[r for r in records if r.metadata.get('moving_wheel')]
    boxes={}
    for r in fixed+moving:
        box=r.shape.bounding_box();boxes[id(r)]=(tuple(box.min),tuple(box.max))
    poses=0
    for key in MOTION_KEYS:
        half=half_travel(d,key)
        own=[r for r in moving if r.metadata['moving_wheel']==key and 'spring_length_mm' not in r.metadata]
        obstacles=fixed+[r for r in moving if r.metadata['moving_wheel']!=key and 'spring_length_mm' not in r.metadata]
        for dz in (-half,-half/2,0,half/2,half):
            poses+=1
            for a in own:
                shape=pose_shape(d,a.shape,key,dz)
                bounds=shape.bounding_box();moved_box=(tuple(bounds.min),tuple(bounds.max))
                for b in obstacles:
                    if not bounds_overlap(moved_box,boxes[id(b)]):continue
                    tested+=1;cut=shape&b.shape;volume=0 if cut is None else cut.volume
                    if volume>.01:hits.append({'wheel':key,'displacement_mm':dz,'a':a.name,'b':b.name,'volume_mm3':volume})
    return {'sampled_poses':poses,'boolean_pairs':tested,'hits':hits,
            'scope':'One wheel at a time (six hinge modules), five sampled positions; springs excluded; no continuous swept-volume or dynamic clearance proof'}


def catalog_suspension_report(d):
    from .parts.suspension import spring_state

    g=9.80665
    s=d.layout['suspension']
    modules={}
    for kind in ('drive','caster'):
        state=spring_state(d,kind)
        r=state['lever_ratio'];unsprung=s[kind]['unsprung_mass_kg_estimate']*g
        state['ground_force_N']=[round(f*r+unsprung,3) for f in state['forces_N']]
        state['estimated_ground_force_N']=state['ground_force_N'][1]
        state['wheel_rate_N_mm']=round(s[kind]['rate_N_mm']*r*r,3)
        modules[kind]=state
    rate=2*modules['drive']['wheel_rate_N_mm']+4*modules['caster']['wheel_rate_N_mm']
    def case(mass,shims=True):
        ground={}
        for kind in ('drive','caster'):
            m=modules[kind];spring=m['forces_N'][1] if shims else m['shim_removed_force_N']
            ground[kind]=spring*m['lever_ratio']+s[kind]['unsprung_mass_kg_estimate']*g
        sink=(mass*g-2*ground['drive']-4*ground['caster'])/rate
        drive=ground['drive']+modules['drive']['wheel_rate_N_mm']*sink
        caster=ground['caster']+modules['caster']['wheel_rate_N_mm']*sink
        within=all(abs(sink)<=s[k]['travel_mm']/2 for k in ('drive','caster'))
        return {'mass_kg':mass,'preload_shims':shims,'body_sink_mm':round(sink,3),'within_travel':within,
                'drive_each_N':round(drive,3),'caster_each_N':round(caster,3),
                'drive_load_fraction':round(2*drive/(mass*g),4)}
    cases={'with_arm':case(s['design_mass_kg']),
           'without_arm_shims_removed':case(s['design_mass_without_arm_kg'],False),
           'without_arm_shims_kept':case(s['design_mass_without_arm_kg'])}
    return {'concept':'Rev12 six independent hinge + spring modules','design_mass_kg':s['design_mass_kg'],
            'modules':modules,'combined_wheel_rate_N_mm':round(rate,3),'static_cases':cases,
            'scope':'Centered CG, flat floor, catalog spring rate (load +-10%), small-angle lever ratio. Caster load also varies with swivel direction (contact 68-112 mm from the hinge). Remove preload shims without the arm. Measure on prototype.'}


# Overlaps inside supplied manufacturer CAD that this project does not edit.
KNOWN_OEM_OVERLAPS=((('43_GL65_OEM_fork_','44_GL65_OEM_wheel_'),30.0,
                     'GL-65-ASF-EUS manufacturer STEP: wheel hub faces enter the fork legs by 0.2 mm at the axle'),)

def _known(a,b,volume):
    for (pa,pb),limit,reason in KNOWN_OEM_OVERLAPS:
        if ((a.startswith(pa) and b.startswith(pb)) or (a.startswith(pb) and b.startswith(pa))) and a[len(pa):]==b[len(pb):] and volume<=limit:
            return reason
    return None

def _all_pairs(records,wanted=None):
    boxes=[(tuple(b.min),tuple(b.max)) for b in (r.shape.bounding_box() for r in records)]
    hits=[];known=[];tested=0
    for i,a in enumerate(records):
        for j in range(i+1,len(records)):
            b=records[j]
            if wanted and not (wanted(a) or wanted(b)):continue
            if not bounds_overlap(boxes[i],boxes[j]):continue
            tested+=1;cut=a.shape&b.shape;volume=0 if cut is None else cut.volume
            if volume<=.01:continue
            item={'a':a.name,'b':b.name,'volume_mm3':round(volume,3)}
            reason=_known(a.name,b.name,volume)
            (known if reason else hits).append({**item,'reason':reason} if reason else item)
    return tested,hits,known

def static_interference(records):
    """Every bounding-box-overlapping pair of the nominal assembly (Rev12b)."""
    tested,hits,known=_all_pairs(records)
    return {'boolean_pairs':tested,'hits':hits,'known_manufacturer_overlaps':known,
            'scope':'All parts, nominal pose, BRep common volume > 0.01 mm3; contact without overlap is allowed'}

def suspension_pose_interference(d):
    """Each wheel at both stroke ends with its module rebuilt (springs at their posed length) against all parts except cameras."""
    from .assembly import GROUPS,build_records
    from .parameters import load_design
    hits=[];known=[];tested=0;poses=0
    for key in MOTION_KEYS:
        for sign in (-1,1):
            posed=load_design(d.variant);posed.motion[key]=sign*half_travel(posed,key)
            records=build_records(posed,[g for g in GROUPS if g!='cameras'])
            mine=lambda r,key=key:key in (r.metadata.get('moving_wheel'),r.metadata.get('wheel'),r.metadata.get('caster'))
            n,h,k=_all_pairs(records,mine);tested+=n;poses+=1
            hits+=[{**x,'wheel':key,'displacement_mm':posed.motion[key]} for x in h]
            known+=[{**x,'wheel':key,'displacement_mm':posed.motion[key]} for x in k]
    return {'sampled_poses':poses,'boolean_pairs':tested,'hits':hits,'known_manufacturer_overlaps':known,
            'scope':'One wheel at a time at full droop and full bump; springs, stoppers and fasteners included; cameras excluded; not a swept-volume proof'}


def caster_swivel_clearance(records,d):
    """Screen each OEM moving fork and wheel against its mounting hardware."""
    hits=[];pairs=0;poses=0
    def bounds(shape):
        b=shape.bounding_box()
        return (tuple(b.min),tuple(b.max))
    for wheel in CASTER_WHEELS:
        rotating=[r for r in records if r.metadata.get('caster')==wheel and r.name.startswith(('43_','44_'))]
        if not rotating:
            continue
        key=wheel;half=half_travel(d,key)
        x,y=caster_xy(d,wheel)
        axis=Axis((x,y,0),(0,0,1))
        nearby=[r for r in records if r.group=='chassis' or
                (r.group=='suspension' and r.metadata.get('wheel') in (wheel,key) and 'spring_length_mm' not in r.metadata)]
        levels=(-half,0,half)
        obstacles={}
        for dz in levels:
            obstacles[dz]=[]
            for obstacle in nearby:
                other=pose_shape(d,obstacle.shape,key,dz) if obstacle.metadata.get('moving_wheel') else obstacle.shape
                obstacles[dz].append((obstacle.name,other,bounds(other)))
        for angle in range(0,360,45):
            shapes=[(r.name,r.shape.rotate(axis,angle)) for r in rotating]
            for dz in levels:
                poses+=1
                for name,shape in shapes:
                    moved=pose_shape(d,shape,key,dz)
                    moved_bounds=bounds(moved)
                    for obstacle,other,other_bounds in obstacles[dz]:
                        if not bounds_overlap(moved_bounds,other_bounds):continue
                        pairs+=1
                        cut=moved&other
                        volume=0 if cut is None else cut.volume
                        if volume>.01:
                            hits.append({'wheel':wheel,'steer_deg':angle,'displacement_mm':dz,'a':name,'b':obstacle,'volume_mm3':volume})
    return {'sampled_poses':poses,'boolean_pairs':pairs,'hits':hits,
            'scope':'OEM fork and wheel, 45-degree swivel increments at droop, nominal and bump of its hinge arm; local hardware and chassis'}


def expected_contact(d,item):
    """Lowest Z of a wheel tilted by its hinge arm (rotation about +Y)."""
    key=item.metadata.get('moving_wheel');value=d.motion.get(key,0) if key else 0
    if not value:return 0.0
    if item.name.startswith('30_'):
        side=-1 if key=='LEFT' else 1
        radius=d.parts['drive_motor']['tire_dia']/2;half=d.parts['drive_motor']['tread_width']/2
        x=side*d.p['drive_track']/2
    else:
        radius=d.parts['caster']['wheel_dia']/2;half=0  # crowned OEM tread
        x=caster_xy(d,item.metadata['caster'])[0]
    a=motion_angle(d,key,value)
    zc=rotate_point(d,key,value,(x,radius))[1]
    return zc-radius*math.cos(a)-half*abs(math.sin(a))


def validate(records,d,full=False,collisions=False):
    errors=[]
    for item in records:
        if not item.shape.is_valid or not item.shape.solids() or sum(s.volume for s in item.shape.solids())<=0:
            errors.append('Invalid solid: '+item.name)
    names=[x.name for x in records]
    if len(names)!=len(set(names)):errors.append('Duplicate part names')
    wheels=[x for x in records if x.name.startswith(('30_','44_'))]
    for item in wheels:
        tolerance=.05 if item.name.startswith('44_') else 1e-5
        if abs(item.shape.bounding_box().min.Z-expected_contact(d,item))>tolerance:errors.append('Wheel contact/pose: '+item.name)
    for item in records:
        if item.name.startswith('30_') and not d.motion.get(item.metadata.get('moving_wheel')):
            b=item.shape.bounding_box()
            if abs(b.size.X-d.parts['drive_motor']['tread_width'])>1e-5 or abs(b.size.Z-180)>1e-5:
                errors.append('Drive tire dimensions: '+item.name)
        if item.name.startswith('32_') and not d.motion.get(item.metadata.get('moving_wheel')) and abs(item.shape.bounding_box().size.Y-d.parts['drive_motor']['shaft_flat'])>1e-5:
            errors.append('Motor shaft flat: '+item.name)
    for group in ('motor_left','motor_right'):
        motor=[x for x in records if x.group==group]
        if len(motor)==3 and not any(d.motion.get(x.metadata.get('moving_wheel')) for x in motor):
            bounds=[x.shape.bounding_box() for x in motor]
            if abs(max(b.max.X for b in bounds)-min(b.min.X for b in bounds)-d.parts['drive_motor']['overall_axial'])>1e-5:
                errors.append('Motor overall axial dimension: '+group)
    report={'valid':not errors,'components':len(records),'solids':sum(len(x.shape.solids()) for x in records),'errors':errors}
    if d.variant!='baseline' and any(x.group=='suspension' for x in records):
        report['suspension_catalog']=catalog_suspension_report(d)
    if full:
        expected=d.parts
        counts={k:sum(x.name.startswith(prefix) for x in records) for k,prefix in
                [('drive_motor','30_'),('caster','41_'),('lidar','61_'),('camera','70_')]}
        # Vendor cameras remain one logical component even if they have many solids.
        for k,count in counts.items():
            if count!=expected[k]['count']:errors.append('Component count: '+k)
        if len(wheels)!=6:errors.append('Expected six wheel contacts')
        scan=coverage(d.layout)
        if any(row['missed'] for row in scan['perimeters']):errors.append('Scan sample miss')
        report.update(counts=counts,lidar_sampling=scan)
        scan_z=d.layout['lidars'][0]['scan_z']
        below=('30_','31_','32_','50_','51_','52_','53_','54_','64_','80_','81_')
        above=('20_','70_','71_','72_')
        if any(x.shape.bounding_box().max.Z>=scan_z for x in records if x.name.startswith(below)):
            errors.append('Below-scan component crosses scan plane')
        if any(x.shape.bounding_box().min.Z<=scan_z for x in records if x.name.startswith(above)):
            errors.append('Above-scan component crosses scan plane')
        report['scan_plane_vertical_clearance_checked']=True
        report['deck_sensor_clearance_mm']={
            'deck_underside_above_scan':round(d.p['top_bottom_z']-scan_z,3),
            'deck_underside_above_camera_top':round(d.p['top_bottom_z']-max(
                x.shape.bounding_box().max.Z for x in records if x.name.startswith('70_')),3),
            'lowest_camera_mount_above_scan':round(min(
                x.shape.bounding_box().min.Z for x in records if x.name.startswith(('70_','71_','72_')))-scan_z,3),
        }
    if collisions:
        hits=[]
        for a in records:
            prefixes=(('10_','20_','50_','51_','52_','53_','70_') if a.name.startswith(('61_','62_'))
                      else ('10_','20_') if a.name.startswith('70_') else ())
            for b in records:
                if prefixes and b.name.startswith(prefixes) and overlap(a.shape,b.shape):
                    intersection=a.shape&b.shape
                    volume=0 if intersection is None else intersection.volume
                    if volume>0.01:hits.append({'a':a.name,'b':b.name,'volume_mm3':volume})
        report['selected_sensor_collisions']=hits
        if hits:errors.append('Sensor collision')
        if d.variant!='baseline' and not d.motion:
            travel=suspension_collisions(records,d);report['suspension_collision_sampling']=travel
            if travel['hits']:errors.append('Suspension sampled collision')
            swivel=caster_swivel_clearance(records,d);report['caster_swivel_sampling']=swivel
            if swivel['hits']:errors.append('Caster swivel sampled collision')
            if full:
                static=static_interference(records);report['static_interference']=static
                if static['hits']:errors.append('Static interference')
                posed=suspension_pose_interference(d);report['suspension_pose_interference']=posed
                if posed['hits']:errors.append('Suspension pose interference')
    report['valid']=not errors
    return report

def _named_nodes(root):
    result={}
    def walk(node):
        if node.label:result.setdefault(node.label,node)
        for child in node.children:walk(child)
    walk(root)
    return result

def compare_baseline(records,symmetric=True):
    reference=import_step(ROOT/'cad/baseline/rev05/reference.step')
    nodes=_named_nodes(reference);rows=[];failed=[]
    for item in records:
        old=nodes.get(item.name)
        if old is None:
            failed.append(item.name+': missing reference label');continue
        a=metrics(old);b=metrics(item.shape)
        bbox=max(abs(x-y) for aa,bb in zip(a['bounds_mm'],b['bounds_mm']) for x,y in zip(aa,bb))
        center=max(abs(x-y) for x,y in zip(a['center_mm'],b['center_mm']))
        volume=abs(a['volume_mm3']-b['volume_mm3'])
        delta=0.0
        if symmetric:
            for first,second in ((old,item.shape),(item.shape,old)):
                cut=first-second
                if cut is not None:delta+=abs(cut.volume)
        ok=(bbox<1e-4 and center<1e-4 and volume<max(1e-3,a['volume_mm3']*1e-8)
            and delta<max(1e-3,a['volume_mm3']*1e-7) and a['solids']==b['solids'])
        rows.append({'name':item.name,'pass':ok,'bounds_error_mm':bbox,'center_error_mm':center,
                     'volume_error_mm3':volume,'symmetric_difference_mm3':delta})
        if not ok:failed.append(item.name)
    return {'pass':not failed,'baseline_commit':'61483e873254c929061b77263d584c652aa565e6',
            'compared_parts':len(rows),'symmetric_difference_checked':symmetric,'failed':failed,'parts':rows}
