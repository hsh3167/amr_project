"""Independent, guided suspension for the 30 kg AMR. Catalog dimensions in mm."""
from math import cos, pi, sin
from build123d import Axis, Circle, Compound, Helix, Plane, sweep
from .common import PartRecord, box, cylx, cylz, drilled, polygon_plate
from .structure import column


def drive_mounts(d, side):
    return [(side * (d.p['drive_track']/2 - 75), y) for y in (-55, -25, 25, 55)]


def caster_mounts(x, y):
    return [(x+dx, y+dy) for dx in (-32, 32) for dy in (-24, 24)]


def chassis_mount_holes(d):
    holes = [(x,y,5.5) for side in (-1,1) for x,y in drive_mounts(d,side)]
    for x in (-d.p['caster_x'], d.p['caster_x']):
        for y in (-d.p['caster_y'], d.p['caster_y']):
            holes.extend((a,b,5.5) for a,b in caster_mounts(x,y))
            holes.extend((x+dx,y,18.4) for dx in (-32,32))
    return holes


def ring(x, y, bottom, height, od, bore):
    return cylz(od/2,height,x,y,bottom)-cylz(bore/2,height+2,x,y,bottom-1)


def screw(x,y,bearing,diameter,length,up=False):
    """Catalog screw envelope, excluding the helical thread and socket recess."""
    head_od = {3:5.5,4:7,5:8.5,6:10}[diameter]
    shank_bottom,head_bottom=(bearing,bearing-diameter) if up else (bearing-length,bearing)
    return cylz(diameter/2,length,x,y,shank_bottom)+cylz(head_od/2,diameter,x,y,head_bottom)


def nut(x,y,bottom,across_flats,height,diameter):
    radius=across_flats/(2*cos(pi/6))
    points=[(x+radius*cos(i*pi/3),y+radius*sin(i*pi/3)) for i in range(6)]
    return polygon_plate(points,height,bottom)-cylz(diameter/2,height+2,x,y,bottom-1)


def guide_shaft(x,y,bottom,q):
    diameter,length,tap=q['shaft_d_mm'],q['shaft_length_mm'],q['shaft_tap_mm']
    depth=2.5*tap
    shaft=cylz(diameter/2,length,x,y,bottom)
    return shaft-cylz(tap/2,depth+1,x,y,bottom-1)-cylz(tap/2,depth+1,x,y,bottom+length-depth)


def coil(x,y,bottom,height,q):
    # The round sweep shows travel and the catalog OD/ID envelope. The stock
    # spring uses shaped wire; its rate and solid length come from the catalog.
    wire=(q['spring_od_mm']-q['spring_id_mm'])/2
    path=Helix((height-wire)/q['display_turns'],height-wire,(q['spring_od_mm']-wire)/2)
    section=Plane(origin=path@0,z_dir=path%0)*Circle(wire/2)
    return sweep(section,path=path,is_frenet=True).translate((x,y,bottom+wire/2))


def spring_state(d, kind):
    q=d.layout['suspension'][kind]
    half=q['travel_mm']/2
    installed=q['loaded_length_mm']-q['shim_mm']
    free=q['free_length_mm']
    rate=q['rate_N_mm']
    lengths=[installed+half,installed,installed-half]
    return {'sku':q['spring_sku'],'travel_mm':q['travel_mm'],'spring_lengths_mm':lengths,
            'forces_N':[round(rate*max(0,free-length),3) for length in lengths],
            'min_solid_margin_mm':round(lengths[2]-q['solid_length_mm'],3),
            'max_working_deflection_margin_mm':round(q['max_deflection_mm']-(free-lengths[2]),3),
            'installed_length_mm':installed}


def make(d):
    records=[]
    suspension=d.layout['suspension']
    p=d.p

    def add(name,shape,wheel,moving=False,sku=None,**metadata):
        if moving:
            shape=shape.translate((0,0,d.motion.get(wheel,0)))
        color=(.77,.64,.36) if 'bushing' in name else (.48,.65,.78)
        status='misumi_catalog_envelope' if sku else 'suspension_fabrication_design'
        records.append(PartRecord('80_'+name+'_'+wheel,'suspension',shape,color,status,
                                  {'wheel':wheel,'moving_wheel':wheel if moving else None,
                                   **({'sku':sku} if sku else {}),**metadata}))

    def add_spring(x,y,bottom,top,q,kind,wheel):
        dz=d.motion.get(wheel,0)
        state=spring_state(d,kind)
        length=top-bottom
        if abs(length-state['installed_length_mm'])>1e-6:
            raise ValueError('Spring stack and catalog seating length differ: '+wheel)
        add(kind+'_spring_coil',coil(x,y,bottom+dz,length-dz,q),wheel,sku=q['spring_sku'],
            spring_top_z=top,spring_length_mm=length,spring_xy=[x,y],rate_N_mm=q['rate_N_mm'],
            free_length_mm=q['free_length_mm'],nominal_spring_force_N=state['forces_N'][1],
            geometry='catalog envelope with illustrative round-wire sweep')

    q=suspension['caster']
    half=q['travel_mm']/2
    plate_bottom=d.parts['caster']['cad_height']
    fixed_bottom=p['base_bottom_z']-4
    shaft_bottom=plate_bottom+4
    shaft_top=shaft_bottom+q['shaft_length_mm']
    housing_top=shaft_top-half
    for x in (-p['caster_x'],p['caster_x']):
        for y in (-p['caster_y'],p['caster_y']):
            wheel=('L' if x<0 else 'R')+('F' if y>0 else 'B')
            guides=[(x+dx,y) for dx in (-32,32)]
            caster_bolts=[(x+dx,y+dy) for dx in (-12,12) for dy in (-27,27)]
            carrier=box(86,76,4,x,y,plate_bottom+2)
            carrier=drilled(carrier,caster_bolts,6.6,plate_bottom,4)
            carrier=drilled(carrier,guides,4.5,plate_bottom,4)
            add('caster_carrier',carrier,wheel,True,material='6061-T6')
            fixed=box(86,76,4,x,y,fixed_bottom+2)
            mounts=caster_mounts(x,y)
            fixed=drilled(fixed,mounts,5.5,fixed_bottom,4)
            for a,b in guides:
                fixed+=cylz(9,housing_top-fixed_bottom,a,b,fixed_bottom)
            fixed=drilled(fixed,guides,q['bush_od_mm'],fixed_bottom,housing_top-fixed_bottom)
            for a,b in guides:
                fixed+=ring(a,b,shaft_bottom+half,fixed_bottom-shaft_bottom-half,16,8.3)
            fixed+=cylz(2.45,8,x,y,fixed_bottom-8)
            add('caster_fixed_seat_12H7',fixed,wheel,material='6061-T6 with replaceable press-fit bushes',pilot_d_mm=4.9)
            for i,(a,b) in enumerate(guides):
                tag=str(i+1)
                add('caster_guide_'+tag,guide_shaft(a,b,shaft_bottom,q),wheel,True,sku=q['shaft_sku'])
                add('caster_bushing_'+tag,ring(a,b,housing_top-1-q['bush_length_mm'],q['bush_length_mm'],q['bush_od_mm'],q['bush_id_mm']),
                    wheel,sku=q['bush_sku'],housing_fit='12 H7; bore F7; shaft g6')
                add('caster_droop_retainer_'+tag,ring(a,b,shaft_top,3,15,4.4),wheel,True)
                add('caster_guide_upper_bolt_'+tag,screw(a,b,shaft_top+3,4,10),wheel,True,sku='CB4-10')
                add('caster_guide_lower_bolt_'+tag,screw(a,b,plate_bottom,4,10,True),wheel,True,sku='CB4-10')
            frame_top=p['base_bottom_z']+p['base_thickness']
            add('caster_frame_bolts',Compound([screw(a,b,frame_top,5,16) for a,b in mounts]),wheel,sku='CB5-16',quantity=4)
            add('caster_frame_nuts',Compound([nut(a,b,fixed_bottom-4,8,4,5) for a,b in mounts]),wheel,quantity=4)
            add('caster_plate_bolts',Compound([screw(a,b,plate_bottom-5,6,16,True) for a,b in caster_bolts]),
                wheel,True,sku='CB6-16',quantity=4,plate_thickness_reference_mm=5)
            add('caster_plate_nuts',Compound([nut(a,b,plate_bottom+4,10,5,6) for a,b in caster_bolts]),wheel,True,quantity=4)
            spring_bottom=plate_bottom+4+q['shim_mm']
            if q['shim_mm']>0:
                add('caster_preload_shim',ring(x,y,plate_bottom+4,q['shim_mm'],13,5.1),wheel,True)
            add('caster_lower_pilot',cylz(2.45,6+q['shim_mm'],x,y,plate_bottom+4),wheel,True)
            add_spring(x,y,spring_bottom,fixed_bottom,q,'caster',wheel)

    q=suspension['drive']
    half=q['travel_mm']/2
    lower,upper=74,120
    housing_bottom,housing_top=87,109
    for side in (-1,1):
        wheel='LEFT' if side<0 else 'RIGHT'
        ax=side*(p['drive_track']/2-41)
        x=side*(p['drive_track']/2-53)
        guides=[(x,-35),(x,35)]
        slot=cylx(7.15,8,ax,0,90)&box(10,10.2,20,ax,0,90)
        carrier=box(24,90,4,x,0,107)+box(6,46,40,ax,0,90)
        for a,b in guides:
            carrier+=cylz(10,22,a,b,housing_bottom)
        carrier=drilled(carrier,guides,q['bush_od_mm'],housing_bottom,22)-slot
        carrier=drilled(carrier,[(x,0)],3.4,105,4)
        add('drive_torque_carrier_14H7',carrier,wheel,True,material='S45C',axle_flat_mm=10.2,axle_plate_t_mm=6)
        # The stationary shoulder and included nut need a physical fit check.
        spacer=cylx(9,4.9,side*(abs(ax)+5.45),0,90)-cylx(7.15,6,side*(abs(ax)+5.45),0,90)
        add('drive_axial_spacer_trial',spacer,wheel,True,fit='Confirm stationary shoulder and bearing-face diameters on motor')
        washer=cylx(10,1,side*(abs(ax)-3.5),0,90)-cylx(7.15,3,side*(abs(ax)-3.5),0,90)
        add('drive_axle_washer',washer,wheel,True)
        axle_nut=nut(0,0,0,22,8,14.3).rotate(Axis.Y,90)
        axle_nut=axle_nut.translate((side*(abs(ax)-4)-8*(side>0),0,90))
        add('drive_axle_M14x1p5_nut_envelope',axle_nut,wheel,True,fit='OEM nut thickness and wrench envelope assumed')
        mounts=drive_mounts(d,side)
        yoke=box(44,140,6,side*(p['drive_track']/2-59),0,123)
        for a,b in guides:
            sign=1 if b>0 else -1
            yoke+=box(22,45,6,a,sign*47.5,71)
            yoke+=box(22,10,48,a,sign*60,96)
        yoke=drilled(yoke,mounts+guides,5.5,120,6)
        yoke=drilled(yoke,guides,5.5,68,6)
        yoke=drilled(yoke,[(x,0)],24,120,6)
        add('drive_fixed_double_support_yoke',yoke,wheel,material='Steel welded or machined frame; align shaft seats after fabrication')
        upper_seat=drilled(box(26,140,4,x,0,168),[(x,-60),(x,60)],5.5,166,4)
        upper_seat=drilled(upper_seat,[(x,0)],4.5,166,4)
        add('drive_upper_seat',upper_seat,wheel)
        for y in (-60,60):
            post=column(d,x,y,126,166,'81_2020_Al_suspension_column_'+wheel+str(y),'suspension')
            records.append(post)
            add('drive_upper_post_bolt_'+str(y),screw(x,y,170,5,12),wheel,sku='CB5-12')
        add('drive_chassis_bolts',Compound([screw(a,b,126,5,18) for a,b in mounts]),wheel,sku='CB5-18',quantity=4)
        add('drive_chassis_nuts',Compound([nut(a,b,p['base_bottom_z']-4,8,4,5) for a,b in mounts]),wheel,quantity=4)
        for i,(a,b) in enumerate(guides):
            tag=str(i+1)
            add('drive_guide_'+tag,guide_shaft(a,b,lower,q),wheel,sku=q['shaft_sku'])
            add('drive_bushing_'+tag,ring(a,b,housing_bottom+1,q['bush_length_mm'],q['bush_od_mm'],q['bush_id_mm']),
                wheel,True,sku=q['bush_sku'],housing_fit='14 H7; bore F7; shaft g6')
            add('drive_droop_stop_'+tag,ring(a,b,lower,housing_bottom-half-lower,18,10.3),wheel)
            add('drive_bump_stop_'+tag,ring(a,b,housing_top+half,upper-housing_top-half,18,10.3),wheel)
            add('drive_guide_upper_bolt_'+tag,screw(a,b,126,5,14),wheel,sku='CB5-14')
            add('drive_guide_lower_bolt_'+tag,screw(a,b,68,5,14,True),wheel,sku='CB5-14')
        guide=guide_shaft(x,0,109,{'shaft_d_mm':6,'shaft_length_mm':28,'shaft_tap_mm':3})
        add('drive_moving_spring_pilot',guide,wheel,True,sku='PSFJW6-28-M3-N3')
        add('drive_pilot_lower_bolt',screw(x,0,105,3,10,True),wheel,True,sku='CB3-10')
        tube=ring(x,0,124,36,9.5,6.5)+ring(x,0,160,6,9.5,4)
        add('drive_fixed_telescoping_pilot',tube,wheel)
        add('drive_pilot_upper_bolt',screw(x,0,170,4,10),wheel,sku='CB4-10')
        add('drive_spring_locating_ring',ring(x,0,109,2,21.5,18.1),wheel,True)
        if q['shim_mm']>0:
            add('drive_preload_shim',ring(x,0,109,q['shim_mm'],17.8,6.2),wheel,True)
        add_spring(x,0,109+q['shim_mm'],166,q,'drive',wheel)
    return records
