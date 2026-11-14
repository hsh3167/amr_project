from .common import *
from build123d import Compound

def make(d):
    p=d.p;w=p['base_width']/2;l=p['base_length']/2;inner=p.get('base_well_inner_x',w-50);well=99
    outline=[(-w,-l),(w,-l),(w,-well),(inner,-well),(inner,well),(w,well),
             (w,l),(-w,l),(-w,well),(-inner,well),(-inner,-well),(-w,-well)]
    base=polygon_plate(outline,p['base_thickness'],p['base_bottom_z'])
    if d.variant!='baseline':
        from .suspension import chassis_mount_holes
        for x,y,diameter in chassis_mount_holes(d):
            base=drilled(base,[(x,y)],diameter,p['base_bottom_z'],p['base_thickness'])
        # Rev13: CB5-12 holes for the main and LiDAR column end bolts.
        columns=[tuple(c) for c in d.profile['main_columns_xy']]+[(q['support_x'],q['support_y']) for q in d.layout['lidars']]
        base=drilled(base,columns,5.5,p['base_bottom_z'],p['base_thickness'])
    r=[PartRecord('01_lower_Al_plate_CONCEPT' if d.variant=='baseline' else '01_lower_steel_plate_3t','chassis',base,(.60,.71,.78))]
    if d.variant=='baseline':
        for y in (-215,215):r.append(PartRecord('04_base_crossmember_'+str(y),'chassis',box(320,18,12,0,y,111),(.61,.70,.79)))
    top=box(p['top_width'],p['top_length'],p['top_thickness'],0,0,p['top_bottom_z']+p['top_thickness']/2)
    top=drilled(top,d.profile['main_columns_xy'],5.5,p['top_bottom_z'],p['top_thickness'])
    if d.variant!='baseline':
        cm=d.layout['camera_mount']
        top=drilled(top,[(s*p['camera_x'],cm['profile_y']) for s in (-1,1)],5.5,p['top_bottom_z'],p['top_thickness'])
    pitch=p['arm_bolt_pitch']/2
    arm_mounts=[(x,y) for x in (-pitch,pitch) for y in (-pitch,pitch)]
    top=drilled(top,arm_mounts,8.4,p['top_bottom_z'],p['top_thickness'])
    r.append(PartRecord('20_upper_Al_plate_PLACEHOLDER_HOLES' if d.variant=='baseline' else '20_upper_steel_plate_3t_PLACEHOLDER_HOLES','payload_interface',top,(.72,.81,.89)))
    if d.variant=='baseline':
        reinforcement=box(155,155,5,0,0,299)
    else:
        bottom=p['top_bottom_z']-5
        reinforcement=drilled(box(155,155,5,0,0,bottom+2.5),arm_mounts,8.4,bottom,5)
    r.append(PartRecord('21_arm_reinforcement_PLACEHOLDER','payload_interface',reinforcement,(.25,.40,.55)))
    if d.variant!='baseline':
        # Rev13: the reinforcement is clamped to the deck by the four arm-mount bolts (replaced by the arm's own bolts later).
        from .suspension import screw,nut
        deck_top=p['top_bottom_z']+p['top_thickness']
        bolts=Compound([s for x,y in arm_mounts for s in (screw(x,y,deck_top,8,16)+nut(x,y,bottom-6.5,13,6.5,8)).solids()])
        r.append(PartRecord('22_arm_mount_bolts_M8_PLACEHOLDER','payload_interface',bolts,(.20,.22,.25),'catalog_envelope',
                            {'sku':'M8x16 hex socket + M8 nut','quantity':4}))
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
