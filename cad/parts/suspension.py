"""Six independent guided spring stages. Concept hardware, not load-rated products."""
from build123d import Helix, Circle, sweep, Plane, Compound
from .common import *
from .structure import column

def spring(x,y,bottom,height,diameter,wire,turns):
    path=Helix((height-wire)/turns,height-wire,(diameter-wire)/2)
    section=Plane(origin=path@0,z_dir=path%0)*Circle(wire/2)
    return sweep(section,path=path,is_frenet=True).translate((x,y,bottom+wire/2))

def positions(d):
    return {'LEFT':0.,'RIGHT':0.,'LF':0.,'LB':0.,'RF':0.,'RB':0.}

def make(d):
    r=[];q=d.layout['suspension'];p=d.p
    def add(name,shape,n,moving=False,**metadata):
        if moving:shape=shape.translate((0,0,d.motion.get(n,0)))
        r.append(PartRecord('80_'+name+'_'+n,'suspension',shape,(.48,.65,.78),
                            'suspension_concept',{'wheel':n,'moving_wheel':n if moving else None,**metadata}))
    c=q['caster'];top=d.parts['caster']['cad_height'];seat=p['base_bottom_z']-4
    for x in (-p['caster_x'],p['caster_x']):
        for y in (-p['caster_y'],p['caster_y']):
            n=('L' if x<0 else 'R')+('F' if y>0 else 'B')
            holes=[(x+dx,y+dy) for dx in (-12,12) for dy in (-27,27)]
            holes.extend((x+dx,y) for dx in (-32,32))
            plate=box(84,76,4,x,y,top+2)
            plate=drilled(plate,holes[:4],7,top,4)
            plate=drilled(plate,holes[4:],9,top,4)
            add('caster_carrier',plate,n,True)
            add('caster_fixed_seat',box(84,76,4,x,y,seat+2),n)
            for dx in (-32,32):
                label=str(dx);add('guide_'+label,cylz(4,41,x+dx,y,75),n)
                low=top-c['travel_mm']/2-3
                add('droop_stop_'+label,cylz(7,3,x+dx,y,low)-cylz(4.1,5,x+dx,y,low-1),n)
                add('bump_stop_'+label,cylz(7,3,x+dx,y,top+4+c['travel_mm']/2)-cylz(4.1,5,x+dx,y,top+3+c['travel_mm']/2),n)
            dz=d.motion.get(n,0)
            add('caster_spring',spring(x,y,top+4+dz,seat-top-4-dz,c['spring_od_mm'],c['wire_mm'],c['turns']),n,
                rate_N_mm=None,preload_N=None,travel_mm=c['travel_mm'])
    m=q['drive']
    for side in (-1,1):
        n='LEFT' if side<0 else 'RIGHT';ax=side*(p['drive_track']/2-41);x=side*(p['drive_track']/2-53)
        slot=cylx(7.15,8,ax,0,90)&box(10,10.2,20,ax,0,90)
        add('axle_clamp',box(6,90,40,ax,0,90)-slot,n,True)
        carrier=drilled(box(18,90,4,x,0,107),[(x,-35),(x,35)],9,105,4)
        add('drive_carrier',carrier,n,True)
        bridge=box(44,140,6,side*(p['drive_track']/2-59),0,123)
        bridge=drilled(bridge,[(x,-35),(x,35)],9,120,6)
        bridge=drilled(bridge,[(x,0)],24,120,6)
        add('drive_fixed_bridge',bridge,n)
        add('drive_upper_seat',box(26,140,4,x,0,168),n)
        for y in (-60,60):r.append(column(d,x,y,126,166,'81_2020_Al_suspension_column_'+n+str(y),'suspension'))
        for y in (-35,35):
            label=str(y);add('guide_'+label,cylz(4,102,x,y,68),n)
            low=105-m['travel_mm']/2-3;high=109+m['travel_mm']/2
            add('droop_stop_'+label,cylz(7,3,x,y,low)-cylz(4.1,5,x,y,low-1),n)
            add('bump_stop_'+label,cylz(7,120-high,x,y,high)-cylz(4.1,122-high,x,y,high-1),n)
        dz=d.motion.get(n,0)
        add('drive_spring',spring(x,0,109+dz,57-dz,m['spring_od_mm'],m['wire_mm'],m['turns']),n,
            rate_N_mm=None,preload_N=None,travel_mm=m['travel_mm'])
    return r
