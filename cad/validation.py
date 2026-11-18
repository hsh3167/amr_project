"""Geometric checks only. Strength, tire compliance and safety remain separate."""
import math
from build123d import Axis, CenterOf, import_step
from .caster_motion import (CASTER_WHEELS, DRIVE_WHEELS, MOTION_KEYS, caster_motion_key, caster_xy, half_travel, is_bogie,
                            is_bogie_key, motion_angle, motion_keys, pose_shape, rotate_point, rotate_yz)
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
    """Five sampled poses per motion key (hinge: six wheels; bogie: two bogies + two rear casters) from the nominal assembly."""
    hits=[];tested=0
    fixed=[r for r in records if not r.metadata.get('moving_wheel') and 'spring_length_mm' not in r.metadata]
    moving=[r for r in records if r.metadata.get('moving_wheel')]
    boxes={}
    for r in fixed+moving:
        box=r.shape.bounding_box();boxes[id(r)]=(tuple(box.min),tuple(box.max))
    poses=0
    for key in motion_keys(d):
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
            'scope':'One motion key at a time (bogie or hinge module), five sampled positions; springs excluded; no continuous swept-volume or dynamic clearance proof'}


def bogie_report(d):
    """Rev15 statics: two pure bogies (pivot line Y=pivot_y) + two sprung rear casters, centred CG, flat floor."""
    from .parts.suspension import spring_state

    g=9.80665;s=d.layout['suspension'];q=s['bogie'];c=s['caster']
    yp=q['pivot_y_mm'];yc=d.p['caster_y'];yr=-d.p['caster_y']
    from .parts.suspension import bogie_spring_state
    spring=bogie_spring_state(d) if q.get('spring') else None
    Fs=spring['forces_N'][1] if spring else 0.0;ys=spring['xy_mm'][1] if spring else yp
    def case(mass,cg_y=0.0,fs=None):
        """Per side: drive (y=0), front caster (yc), rear caster (yr). Vertical sum, pitch moment about y=0 and the
        bogie moment about its pivot; Rev17: the spring (between chassis and bogie, at ys) adds Fs*(yp-ys) to the bogie."""
        side=mass*g/2;fs=Fs if fs is None else fs;ms=fs*(yp-ys)
        # N_c*yc + N_r*yr = side*cg ; N_d*yp - N_c*(yc-yp) = ms ; N_d + N_c + N_r = side
        # -> N_d = side - N_c - N_r, N_r = (side*cg - N_c*yc)/yr
        a=-yp-(yc-yp)+yp*yc/yr; b=ms-side*yp+yp*side*cg_y/yr
        caster=b/a;rear=(side*cg_y-caster*yc)/yr;drive=side-caster-rear
        pivot=drive+caster-fs
        return {'mass_kg':mass,'cg_y_mm':cg_y,'bogie_spring_N':round(fs,2),'pivot_each_N':round(pivot,2),'drive_each_N':round(drive,2),
                'front_caster_each_N':round(caster,2),'rear_caster_each_N':round(rear,2),
                'drive_load_fraction':round(2*drive/(mass*g),4)}
    rear=spring_state(d,'caster');ratio=rear['lever_ratio'];unsprung=c['unsprung_mass_kg_estimate']*g
    nominal=case(s['design_mass_kg'])
    needed=(nominal['rear_caster_each_N']-unsprung)/ratio
    length=c['free_length_mm']-needed/c['rate_N_mm']
    rear_rate=c['rate_N_mm']*ratio*ratio
    def rear_sink(mass):
        r=case(mass)['rear_caster_each_N'];have=rear['forces_N'][1]*ratio+unsprung
        return round((r-have)/rear_rate,3)
    pitch=[case(s['design_mass_kg'],y) for y in (-30,0,30)]
    tip_g=round(yp/q.get('cg_height_mm_estimate',200),3)
    from .caster_motion import bogie_stack
    st=bogie_stack(d);xo_b,xi_b=st['bearing_x'];span_b=xo_b-xi_b;xw=d.p['drive_track']/2;xc=d.p['caster_x']
    def bearings(cs):
        # Vertical wheel loads at their X; the bearing pair (spacing span_b) reacts force and roll moment.
        fw,fc=cs['drive_each_N'],cs['front_caster_each_N'];fs=cs.get('bogie_spring_N',0);F=fw+fc-fs
        xs=spring['xy_mm'][0] if spring else xi_b
        m=fw*(xw-xi_b)+fc*(xc-xi_b)-fs*(xs-xi_b)
        out=m/span_b;inn=F-out
        worst=max(abs(out),abs(inn))
        return {'cg_y_mm':cs['cg_y_mm'],'outer_bearing_N':round(out,1),'inner_bearing_N':round(inn,1),
                'roll_moment_about_centre_Nm':round((fw*(xw-st['center_x'])+fc*(xc-st['center_x']))/1000,2),
                's0_at_1g':round(q['bearing_C0r_kN']*1000/worst,2),'s0_at_2g':round(q['bearing_C0r_kN']*1000/(2*worst),2),
                's0_at_3g':round(q['bearing_C0r_kN']*1000/(3*worst),2)}
    pivot_report={'holder':q['holder_sku'],'bearing':q['bearing'],'C0r_kN':q['bearing_C0r_kN'],
                  'bearing_x_mm':[round(v,2) for v in st['bearing_x']],'bearing_spacing_mm':round(span_b,2),
                  'cases':[bearings(cs) for cs in [nominal]+pitch[::2]],
                  'scope':'Vertical wheel loads only (no side force); static load rating check, no fatigue or fretting'}
    spring_report=None
    if spring:
        half=q['travel_mm']/2
        spring_report={**spring,
            'drive_each_N_at_front_caster_poses':[case(s['design_mass_kg'],0,f)['drive_each_N'] for f in spring['forces_N']],
            'drive_each_without_spring_N':case(s['design_mass_kg'],0,0)['drive_each_N'],
            'drive_each_without_arm_N':case(s['design_mass_without_arm_kg'])['drive_each_N'],
            'drive_each_without_arm_without_spring_N':case(s['design_mass_without_arm_kg'],0,0)['drive_each_N'],
            'bogie_angular_rate_Nm_per_deg':round(spring["rate_N_mm"]*spring["arm_behind_pivot_mm"]**2*math.pi/180/1000,3),
            'scope':'Spring force at the poses = rate x posed length (front caster +-%g mm); the pose loads are quasi-static (chassis level)'%half}
    return {'concept':'Rev17 bogie (front caster + drive wheel, BGHK bearing pivot, coaxial bogie spring) + rear hinge casters'
            if spring else 'Rev16 bogie (front caster + drive wheel, BGHK bearing pivot) + rear hinge casters','design_mass_kg':s['design_mass_kg'],
            'bogie_spring':spring_report,
            'bogie':{'pivot_y_mm':yp,'pivot_z_mm':q['pivot_z_mm'],'front_arm_mm':yc-yp,'rear_arm_mm':yp,
                     'travel_front_caster_mm':q['travel_mm'],'drive_travel_mm':round(q['travel_mm']*yp/(yc-yp),2),
                     'step_rise_ratio':round(yp/yc,3),'pivot':q['holder_sku']},
            'pivot_bearings':pivot_report,
            'static_cases':{'with_arm':nominal,'cg_sensitivity':pitch,'without_arm':case(s['design_mass_without_arm_kg'])},
            'rear_caster_spring':{**rear,'needed_spring_force_N':round(needed,2),'needed_length_mm':round(length,3),
                                  'level_shim_mm':round(c['loaded_length_mm']-length,3),'configured_shim_mm':c['shim_mm'],
                                  'rear_sink_with_arm_mm':rear_sink(s['design_mass_kg']),
                                  'rear_sink_without_arm_mm':rear_sink(s['design_mass_without_arm_kg']),
                                  'wheel_rate_N_mm':round(rear_rate,3)},
            'forward_pitch_onset_g':tip_g,
            'scope':('Centred CG unless stated, flat floor, rigid bogie, catalog rear spring rate (+-10%). Pitch: the chassis can '
                     'rotate forward about the pivot line once deceleration x CG height > g x pivot_y (CG height 200 mm assumed); '
                     'the front stopper then hands the load to the front caster. Measure on prototype.')}


def catalog_suspension_report(d):
    from .parts.suspension import spring_state
    if is_bogie(d):
        return bogie_report(d)

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

def support_connectivity(records,tolerance=.2):
    """Rev13: every part must reach the base plate through touching parts (gap <= tolerance, i.e. clearance fits).

    Catches floating envelopes and fastener stacks that bear on nothing. Contact is geometric only:
    it does not prove that a joint is clamped or strong enough.
    """
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    boxes=[(tuple(b.min),tuple(b.max)) for b in (r.shape.bounding_box() for r in records)]
    near=lambda a,b:all(min(x1,y1)-max(x0,y0)>-tolerance for x0,x1,y0,y1 in zip(a[0],a[1],b[0],b[1]))
    links={r.name:set() for r in records}
    for i,a in enumerate(records):
        for j in range(i+1,len(records)):
            b=records[j]
            if not near(boxes[i],boxes[j]):continue
            dist=BRepExtrema_DistShapeShape(a.shape.wrapped,b.shape.wrapped)
            if dist.IsDone() and dist.Value()<=tolerance:
                links[a.name].add(b.name);links[b.name].add(a.name)
    # Manufacturer CAD without the swivel bearing balls: the fork hangs 0.87 mm under the plate head.
    for r in records:
        if r.name.startswith('43_GL65_OEM_fork_'):
            head='41_GL65_OEM_plate_head_'+r.name[len('43_GL65_OEM_fork_'):]
            if head in links:links[r.name].add(head);links[head].add(r.name)
    root=next(r.name for r in records if r.name.startswith('01_'))
    seen={root};todo=[root]
    while todo:
        for other in links[todo.pop()]-seen:
            seen.add(other);todo.append(other)
    unsupported=sorted(set(links)-seen)
    return {'root':root,'tolerance_mm':tolerance,'contacts':sum(map(len,links.values()))//2,
            'unsupported':unsupported,'isolated':[n for n in unsupported if not links[n]],
            'scope':'Contact graph from the base plate at the nominal pose; geometric support only, not joint strength'}

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
    for key in motion_keys(d):
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
        key=caster_motion_key(wheel,d);half=half_travel(d,key)
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
    """Lowest Z of a wheel tilted by its hinge arm (rotation about +Y) or carried by its bogie (rotation about +X)."""
    key=item.metadata.get('moving_wheel');value=d.motion.get(key,0) if key else 0
    if not value:return 0.0
    if is_bogie_key(d,key):
        # The bogie turns each wheel in its own rolling plane: contact = posed wheel centre - radius.
        if item.name.startswith('30_'):
            radius=d.parts['drive_motor']['tire_dia']/2;centre=(0.0,radius)
        else:
            radius=d.parts['caster']['wheel_dia']/2;centre=tuple(item.metadata['nominal_center_yz'])
        return rotate_yz(d,key,value,centre)[1]-radius
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
        below=('30_','31_','32_','50_','51_','52_','53_','54_','55_','56_','57_','64_','80_','81_')
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
                support=support_connectivity(records);report['support_connectivity']=support
                if support['unsupported']:errors.append('Unsupported parts: '+', '.join(support['unsupported']))
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
