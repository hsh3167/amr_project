from .common import *

def make(d):
    p=d.p;w=p['base_width']/2;l=p['base_length']/2;inner=w-50;well=99
    outline=[(-w,-l),(w,-l),(w,-well),(inner,-well),(inner,well),(w,well),
             (w,l),(-w,l),(-w,well),(-inner,well),(-inner,-well),(-w,-well)]
    base=polygon_plate(outline,p['base_thickness'],p['base_bottom_z'])
    if d.variant!='baseline':
        from .suspension import chassis_mount_holes
        for x,y,diameter in chassis_mount_holes(d):
            base=drilled(base,[(x,y)],diameter,p['base_bottom_z'],p['base_thickness'])
    r=[PartRecord('01_lower_Al_plate_CONCEPT','chassis',base,(.60,.71,.78))]
    if d.variant=='baseline':
        for y in (-215,215):r.append(PartRecord('04_base_crossmember_'+str(y),'chassis',box(320,18,12,0,y,111),(.61,.70,.79)))
    top=box(p['top_width'],p['top_length'],p['top_thickness'],0,0,p['top_bottom_z']+p['top_thickness']/2)
    top=drilled(top,d.profile['main_columns_xy'],5.5,p['top_bottom_z'],p['top_thickness'])
    pitch=p['arm_bolt_pitch']/2
    top=drilled(top,[(x,y) for x in (-pitch,pitch) for y in (-pitch,pitch)],8.4,p['top_bottom_z'],p['top_thickness'])
    r.append(PartRecord('20_upper_Al_plate_PLACEHOLDER_HOLES','payload_interface',top,(.72,.81,.89)))
    r.append(PartRecord('21_arm_reinforcement_PLACEHOLDER','payload_interface',box(155,155,5,0,0,299),(.25,.40,.55)))
    return r

def brackets(d):
    r=[]
    if d.variant!='baseline':return r
    for side in (-1,1):
        x=side*(d.p['drive_track']/2-41)
        slot=cylx(7.15,8,x,0,90)&box(10,10.2,20,x,0,90)
        r.append(PartRecord('02_axle_flat_bracket_CONCEPT_'+str(side),'brackets',box(6,90,70,x,0,90)-slot,(.29,.43,.56)))
        r.append(PartRecord('03_bracket_bridge_CONCEPT_'+str(side),'brackets',box(43,90,6,side*151.5,0,123),(.29,.43,.56)))
    return r
