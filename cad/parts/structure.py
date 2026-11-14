from .common import *

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
        # Rev12b: collar plates sit on the 3t base top / under the deck and clear the column
        # (20.4 square hole) and the adjacent LiDAR column (27 square, 0.5 gap).
        base_top=d.p['base_bottom_z']+d.p['base_thickness']
        for prefix,z in (('11_corner_lower_',base_top+2),('12_corner_upper_',deck_bottom-2)):
            plate=box(27,27,4,x,y,z)-box(20.4,20.4,6,x,y,z)
            result.append(PartRecord(prefix+n,'structure',plate,(.33,.49,.63),metadata={'material':'4t collar plate around the 20x20 column'}))
    return result
