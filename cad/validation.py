"""Geometric checks only. Strength, tire compliance and safety remain separate."""
import math
from build123d import Axis, CenterOf, import_step
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
    """Five independently sampled poses per wheel; positive volume, broad-phase first."""
    hits=[];tested=0
    fixed=[r for r in records if not r.metadata.get('moving_wheel') and 'spring_length_mm' not in r.metadata]
    moving=[r for r in records if r.metadata.get('moving_wheel')]
    boxes={}
    for r in fixed+moving:
        box=r.shape.bounding_box();boxes[id(r)]=(tuple(box.min),tuple(box.max))
    for wheel in ('LEFT','RIGHT','LF','LB','RF','RB'):
        half=d.layout['suspension']['drive' if wheel in ('LEFT','RIGHT') else 'caster']['travel_mm']/2
        own=[r for r in moving if r.metadata['moving_wheel']==wheel]
        obstacles=fixed+[r for r in moving if r.metadata['moving_wheel']!=wheel]
        for dz in (-half,-half/2,0,half/2,half):
            for a in own:
                offset=dz-d.motion.get(wheel,0);bb=boxes[id(a)]
                moved_box=((bb[0][0],bb[0][1],bb[0][2]+offset),(bb[1][0],bb[1][1],bb[1][2]+offset))
                shape=None
                for b in obstacles:
                    if not bounds_overlap(moved_box,boxes[id(b)]):continue
                    if shape is None:shape=a.shape.translate((0,0,offset))
                    tested+=1;cut=shape&b.shape;volume=0 if cut is None else cut.volume
                    if volume>.01:hits.append({'wheel':wheel,'displacement_mm':dz,'a':a.name,'b':b.name,'volume_mm3':volume})
    return {'sampled_poses':30,'boolean_pairs':tested,'hits':hits,'scope':'One wheel at a time, five sampled positions; springs/mates excluded; no continuous swept-volume or dynamic clearance proof'}


def catalog_suspension_report(d):
    from .parts.suspension import spring_state

    target=d.layout['suspension']['design_mass_kg']
    modules={kind:spring_state(d,kind) for kind in ('drive','caster')}
    for kind,state in modules.items():
        estimate=d.layout['suspension'][kind]['unsprung_mass_kg_estimate']
        state['estimated_ground_force_N']=round(state['forces_N'][1]+estimate*9.80665,3)
    total=2*modules['drive']['estimated_ground_force_N']+4*modules['caster']['estimated_ground_force_N']
    stiffness=2*d.layout['suspension']['drive']['rate_N_mm']+4*d.layout['suspension']['caster']['rate_N_mm']
    return {'target_mass_kg':target,'modules':modules,'nominal_total_ground_force_N':round(total,3),
            'estimated_static_offset_mm':round((target*9.80665-total)/stiffness,3),
            'nominal_drive_load_fraction':round(2*modules['drive']['estimated_ground_force_N']/total,4),
            'scope':'Centered static model, catalog nominal rates and estimated unsprung masses; measure six contact loads on prototype'}


def caster_swivel_clearance(records,d):
    """Screen the OEM moving fork and wheel against its new hardware."""
    half=d.layout['suspension']['caster']['travel_mm']/2
    hits=[];pairs=0;poses=0
    for wheel in ('LF','LB','RF','RB'):
        rotating=[r for r in records if r.metadata.get('moving_wheel')==wheel and r.name.startswith(('43_','44_'))]
        if not rotating:
            continue
        x=d.p['caster_x']*(-1 if wheel[0]=='L' else 1)
        y=d.p['caster_y']*(1 if wheel[1]=='F' else -1)
        axis=Axis((x,y,0),(0,0,1))
        nearby=[r for r in records if r.group=='chassis' or
                (r.group=='suspension' and r.metadata.get('wheel')==wheel and 'spring_length_mm' not in r.metadata)]
        def bounds(shape):
            b=shape.bounding_box()
            return (tuple(b.min),tuple(b.max))

        obstacles={}
        for dz in (-half,0,half):
            obstacles[dz]=[]
            for obstacle in nearby:
                other=obstacle.shape.translate((0,0,dz)) if obstacle.metadata.get('moving_wheel') else obstacle.shape
                obstacles[dz].append((obstacle.name,other,bounds(other)))
        for angle in range(0,360,45):
            shapes=[(r.name,r.shape.rotate(axis,angle)) for r in rotating]
            for dz in (-half,0,half):
                poses+=1
                for name,shape in shapes:
                    moved=shape.translate((0,0,dz))
                    moved_bounds=bounds(moved)
                    for obstacle,other,other_bounds in obstacles[dz]:
                        if not bounds_overlap(moved_bounds,other_bounds):continue
                        pairs+=1
                        cut=moved&other
                        volume=0 if cut is None else cut.volume
                        if volume>.01:
                            hits.append({'wheel':wheel,'steer_deg':angle,'displacement_mm':dz,'a':name,'b':obstacle,'volume_mm3':volume})
    return {'sampled_poses':poses,'boolean_pairs':pairs,'hits':hits,
            'scope':'OEM fork and wheel, 45-degree swivel increments at droop, nominal and bump; local hardware and chassis'}

def validate(records,d,full=False,collisions=False):
    errors=[]
    for item in records:
        if not item.shape.is_valid or not item.shape.solids() or sum(s.volume for s in item.shape.solids())<=0:
            errors.append('Invalid solid: '+item.name)
    names=[x.name for x in records]
    if len(names)!=len(set(names)):errors.append('Duplicate part names')
    wheels=[x for x in records if x.name.startswith(('30_','44_'))]
    for item in wheels:
        offset=d.motion.get(item.metadata.get('moving_wheel'),0)
        if abs(item.shape.bounding_box().min.Z-offset)>1e-5:errors.append('Wheel contact/pose: '+item.name)
    for item in records:
        if item.name.startswith('30_'):
            b=item.shape.bounding_box()
            if abs(b.size.X-d.parts['drive_motor']['tread_width'])>1e-5 or abs(b.size.Z-180)>1e-5:
                errors.append('Drive tire dimensions: '+item.name)
        if item.name.startswith('32_') and abs(item.shape.bounding_box().size.Y-d.parts['drive_motor']['shaft_flat'])>1e-5:
            errors.append('Motor shaft flat: '+item.name)
    for group in ('motor_left','motor_right'):
        motor=[x for x in records if x.group==group]
        if len(motor)==3:
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
