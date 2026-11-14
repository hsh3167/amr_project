from .common import *
from build123d import Compound

def profile_shape(d,x,y,bottom,top):
    q=d.profile;w=q['nominal_section'];h=top-bottom
    s=box(w,w,h,0,0,h/2)-cylz(q['assumed_core_dia']/2,h+2,0,0,-1)
    for a in (0,90,180,270):
        for cutter in (box(q['assumed_slot_mouth'],2,h+2,0,w/2-1,h/2),
                       box(q['assumed_slot_inner_width'],2.5,h+2,0,w/2-3.25,h/2)):
            s=s-rz(cutter,a)
    return s.translate((x,y,bottom))

def column(d,x,y,bottom,top,name,group):
    return PartRecord(name,group,profile_shape(d,x,y,bottom,top),(.67,.77,.84),'generic_profile',
                      {'material':'Aluminum extrusion','length_mm':top-bottom,'section_mm':d.profile['nominal_section']})

def end_bolt(x,y,face,plate,up,core=4.2,engage=9):
    """CB5-12 socket head cap screw through a plate into the M5-tapped core of a 20x20 profile end.

    up=True: head under the plate (face = plate underside), screw points +Z.
    up=False: head on the plate (face = plate top), screw points -Z.
    The engaged thread is drawn at the tapped core diameter, the shank at M5 in a 5.5 hole.
    """
    if up:
        return cylz(4.25,5,x,y,face-5)+cylz(2.5,plate,x,y,face)+cylz(core/2,engage,x,y,face+plate)
    return cylz(4.25,5,x,y,face)+cylz(2.5,plate,x,y,face-plate)+cylz(core/2,engage,x,y,face-plate-engage)

def bolts_record(name,group,shapes,quantity):
    return PartRecord(name,group,Compound([s for c in shapes for s in c.solids()]),(.20,.22,.25),'catalog_envelope',
                      {'sku':'CB5-12','quantity':quantity,'fit':'M5 into the tapped profile core (core 4.2 = M5 tap drill); thread drawn at the core diameter'})

def make(d):
    result=[]
    for x,y in d.profile['main_columns_xy']:
        n=('L' if x<0 else 'R')+('F' if y>0 else 'B')
        deck_bottom=d.p['top_bottom_z']
        result.append(column(d,x,y,d.profile['main_bottom_z'],deck_bottom,'10_2020_Al_column_'+n,'structure'))
        if d.variant=='baseline':
            for prefix,z in (('11_corner_lower_',121),('12_corner_upper_',deck_bottom-2)):
                result.append(PartRecord(prefix+n,'structure',box(32,32,4,x,y,z),(.33,.49,.63)))
            continue
        # Rev13: no loose collar plates. Each column end is bolted through the plate into its tapped core.
        shapes=[end_bolt(x,y,d.p['base_bottom_z'],d.p['base_thickness'],True),
                end_bolt(x,y,deck_bottom+d.p['top_thickness'],d.p['top_thickness'],False)]
        result.append(bolts_record('13_column_end_bolts_CB5-12_'+n,'structure',shapes,2))
    return result
