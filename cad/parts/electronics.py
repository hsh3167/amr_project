from functools import lru_cache
from .common import *
from build123d import Compound

COMPANY = 'company_supplied_step'


def make(d):
    p=d.p;e=d.layout['electronics']
    if d.variant=='baseline':
        items=[('50_company_7S9P_BATTERY_DUMMY',box(p['battery_width'],p['battery_length'],p['battery_height'],0,p['battery_y'],p['battery_center_z']),(.86,.63,.36))]
    else:
        items=[]
    items+=[('51_controller_PCB_DUMMY',box(165,104,7,0,e['logic_y'],e['pcb_center_z']),(.12,.46,.37)),
            ('52_control_computer_DUMMY',box(146,82,23,0,e['logic_y'],e['computer_center_z']),(.13,.19,.28)),
            ('54_REAR_power_distribution_DUMMY',box(160,65,40,e.get('power_bay_x',0),e['power_bay_y'],e['power_bay_center_z']),(.65,.39,.24))]
    md=e.get('motor_driver') if d.variant!='baseline' else None
    if not md:
        for x in (-e['driver_x'],e['driver_x']):items.append(('53_motor_driver_DUMMY_'+str(x),box(70,98,44,x,e['driver_y'],e['driver_center_z']),(.28,.38,.52)))
    records=[PartRecord(n,'electronics',s,c,'envelope_only') for n,s,c in items]
    if d.variant=='baseline':return records
    records+=motor_driver(d)
    # Rev13: every envelope stands on something. M4 hex standoffs (AF7 drawn as D7) at the corners,
    # inset 8 mm; the computer stands on the controller PCB.
    base_top=p['base_bottom_z']+p['base_thickness'];shapes={n:s for n,s,_ in items}
    pcb=shapes['51_controller_PCB_DUMMY'].bounding_box().max.Z
    for name,shape,_ in items:
        b=shape.bounding_box();floor=pcb if name.startswith('52_') else base_top;h=b.min.Z-floor
        if h<=0:continue
        legs=[cylz(3.5,h,x,y,floor) for x in (b.min.X+8,b.max.X-8) for y in (b.min.Y+8,b.max.Y-8)]
        label=name.split('_DUMMY')[0]+'_standoffs_M4'+name.split('_DUMMY')[1]
        records.append(PartRecord(label,'electronics',Compound(legs),(.55,.58,.62),'concept_support',
                                  {'height_mm':round(h,2),'quantity':4,'supports':name}))
    records+=battery(d)+lcd(d)+speaker(d)+dcdc(d)
    strap,_,buckle=_battery_strap(d)
    records+=[PartRecord('50_battery_hold_down_straps','electronics',strap,(.15,.15,.17),'catalog_envelope',
                         {'quantity':2,'sku':'25 mm polyester strap + cam buckle','fit':'Around the case through two base-plate slots each, closed under the plate; clear of the terminal block'}),
              PartRecord('50_battery_strap_buckles','electronics',buckle,(.30,.30,.33),'catalog_envelope',{'quantity':2})]
    gx,gy=_ground_xy(d)
    records.append(ground_stud(base_top,p['base_bottom_z'],gx,gy,'58_ground_stud_M4'))
    return records


def _battery_strap(d):
    from .vendor import source
    p=d.p;e=d.layout['electronics']
    case=max(source('BATTERY.STEP').solids(),key=lambda s:s.volume).bounding_box()
    hx,hy,h=case.size.X/2,case.size.Y/2,case.size.Z
    box_=(-hx,hx,e['battery_y']-hy,e['battery_y']+hy,e['battery_bottom_z'],e['battery_bottom_z']+h)
    return battery_strap(box_,'x',e['battery_strap_x'],p['base_bottom_z'],p['base_thickness'])


def mount_holes(d):
    """(x, y, diameter) of base-plate holes for the electronics brackets, the driver bolts and the ground stud."""
    if d.variant=='baseline':return []
    md=d.layout['electronics'].get('motor_driver')
    driver=[(md['x']+x,md['y']+y,3.4) for x,y in ZLAC_EAR_HOLES] if md else []
    gx,gy=_ground_xy(d)
    return [(x,y,5.5) for x,y in _lcd_bolts(d)+_speaker_bolts(d)]+driver+[(gx,gy,4.5)]


def mount_slots(d):
    """(x0, x1, y0, y1) base-plate slots for the battery straps."""
    if d.variant=='baseline':return []
    return _battery_strap(d)[1]


def motor_driver(d):
    """Rev17: ZLTECH ZLAC8015D wide side down directly on the 3t steel base plate (heat spreader), 4x M3 through the ears."""
    from .suspension import screw,nut
    p=d.p;md=d.layout['electronics']['motor_driver'];X,Y=md['x'],md['y']
    base_top=p['base_bottom_z']+p['base_thickness']
    to_span=lambda x0,x1,y0,y1,z0,z1:span(X+x0,X+x1,Y+y0,Y+y1,base_top+z0,base_top+z1)
    hole=lambda x,y,r:cylz(r,40,X+x,Y+y,base_top-5)
    meta={k:md[k] for k in ('sku','maker','channels','electrical','source','mount','fit_to_motor')}
    pts=[(X+x,Y+y) for x,y in ZLAC_EAR_HOLES]
    bolts=Compound([s for x,y in pts for s in (screw(x,y,base_top+ZLAC_FLANGE_T,3,10)+nut(x,y,p['base_bottom_z']-2.4,5.5,2.4,3)).solids()])
    return [PartRecord('53_motor_driver_'+md['sku'],'electronics',zlac_driver(to_span,hole),(.20,.22,.26),'drawing_envelope',
                       {**meta,'model':'Manual Fig.1 outline 168x89x29.7, ear-hole pitch 158; hole Y position, 3 mm base, cover and terminal blocks estimated'}),
            PartRecord('53_motor_driver_bolts_M3x10','electronics',bolts,(.20,.22,.25),'catalog_envelope',
                       {'sku':'M3x10 hex socket + washer + M3 nylock under the base plate','quantity':4,
                        'note':'Thermal pad or paste between the heatsink base and the steel plate'})]


def _ground_xy(d):
    return tuple(d.layout['electronics']['ground_stud_xy'])


def ground_stud(d_top,d_bottom,x,y,label,group='electronics'):
    """M4 ground stud: screw from below through the plate, ring terminal + nut on top (cable to the LCD frame and battery -)."""
    from .suspension import nut
    stud=cylz(2,d_top-d_bottom+9,x,y,d_bottom)+cylz(3.5,2.8,x,y,d_bottom-2.8)
    ring=cylz(4,.8,x,y,d_top)-cylz(2.15,2,x,y,d_top-.5)
    return PartRecord(label,group,Compound([s for c in (stud,ring,nut(x,y,d_top+.8,7,3.2,4)) for s in c.solids()]),(.70,.55,.20),
                      'catalog_envelope',{'sku':'M4x16 + ring terminal + M4 nut + star washer','role':'Chassis ground point; the 3D-printed LCD and speaker holders do not ground, so a cable runs from here to the LCD frame (and the battery negative)'})


def _bolt_stack(points,base_top,base_bottom,foot=3,washer=False):
    """CB5 head on a foot (3t steel or 6t print + 1.2 mm fender washer), through the foot and the 3t base plate, M5 nut underneath."""
    from .suspension import screw,nut
    w=1.2 if washer else 0;length=round(foot+w+3+4+2)
    parts=[]
    for x,y in points:
        parts+=(screw(x,y,base_top+foot+w,5,length)+nut(x,y,base_bottom-4,8,4,5)).solids()
        if washer:parts+=(cylz(7.5,w,x,y,base_top+foot)-cylz(2.75,w+2,x,y,base_top+foot-1)).solids()
    return Compound(parts)


# ---------------------------------------------------------------- battery
def battery(d):
    from .vendor import source
    p=d.p;e=d.layout['electronics']
    body,pads,detail=_battery_shapes(e['battery_y'],e['battery_bottom_z'],e['battery_pad_mm'],p['base_bottom_z']+p['base_thickness'])
    return [PartRecord('50_company_7S9P_battery_STEP','electronics',body,(.86,.63,.36),COMPANY,
                       {'source':'BATTERY.STEP.gz','placement':detail,
                        'limits':'Company-supplied STEP (case + terminal block); cell type, mass, BMS and charger not in the model. Hold-down strap TBD.'}),
            PartRecord('50_battery_support_pads','electronics',pads,(.55,.58,.62),'concept_support',
                       {'quantity':4,'height_mm':e['battery_pad_mm'],'material':'rubber pad (anti-vibration), strap or clamp TBD'})]


@lru_cache(maxsize=4)
def _battery_shapes(y,bottom,pad,base_top):
    from .vendor import source
    src=source('BATTERY.STEP');case=max(src.solids(),key=lambda s:s.volume).bounding_box()
    # Supplied frame: long side X, terminal block on top (+Z). Centre the case in X, case centre at y.
    dx=-(case.min.X+case.max.X)/2;dy=y-(case.min.Y+case.max.Y)/2;dz=bottom-case.min.Z
    solids=[s.translate((dx,dy,dz)) for s in src.solids()]
    b=Compound(solids).bounding_box()
    legs=[cylz(7.5,pad,x,yy,base_top) for x in (case.min.X+dx+15,case.max.X+dx-15) for yy in (case.min.Y+dy+15,case.max.Y+dy-15)]
    return (Compound(solids),Compound(legs),
            {'translate_mm':[round(dx,3),round(dy,3),round(dz,3)],'case_mm':[round(case.size.X,2),round(case.size.Y,2),round(case.size.Z,2)],
             'top_with_terminals_z':round(b.max.Z,2)})


# ---------------------------------------------------------------- LCD
def _lcd_frame(d):
    """Rotation about X (screen tilted up) and translation of the supplied LCD frame (screen +Y)."""
    from .vendor import source
    e=d.layout['electronics'];tilt=e['lcd_tilt_deg']
    rotated=Compound([s.rotate(Axis.X,tilt) for s in source('LCD.STEP').solids()]).bounding_box()
    return tilt,(0,e['lcd_front_y']-rotated.max.Y,e['lcd_bottom_z']-rotated.min.Z)


def _place(shape,tilt,move):
    return shape.rotate(Axis.X,tilt).translate(move)


def _lcd_bolts(d):
    e=d.layout['electronics']
    return [(x,e['lcd_front_y']-9) for x in (-102,102)]


def lcd(d):
    from .vendor import source
    p=d.p;e=d.layout['electronics'];base_top=p['base_bottom_z']+p['base_thickness']
    tilt,move=_lcd_frame(d)
    screen=Compound([_place(s,tilt,move) for s in source('LCD.STEP').solids()])
    # Rev17: one 3D-printed holder: back plate behind the LCD board (local Y -12.3), thick foot, side clamps and two ribs.
    ft=e['lcd_holder_foot_mm'];pt=e['lcd_holder_plate_mm']
    plate=_place(span(-95.9,95.9,-12.3-pt,-12.3,-90,45),tilt,move)&span(-120,120,0,400,base_top+ft,400)
    bottom=(plate&span(-120,120,0,400,base_top+ft,base_top+ft+.5)).bounding_box()
    foot=span(-110,110,bottom.min.Y,e['lcd_front_y'],base_top,base_top+ft)
    for s in (-1,1):foot=foot+span(*sorted((s*90,s*95)),bottom.min.Y-30,bottom.min.Y+1,base_top,base_top+ft)   # pads under the ribs
    clamps=[]
    for s in (-1,1):
        block=span(*sorted((s*89.9,s*95.9)),-12.3,4.6,-40,40)+span(*sorted((s*87.4,s*89.9)),3.1,4.6,-40,40)
        clamps.append(_place(block,tilt,move))
    ribs=[lcd_rib(tilt,move,-12.3-pt,base_top+ft,*sorted((s*90,s*95))) for s in (-1,1)]
    holder=plate+foot
    for c in clamps+ribs:holder=holder+c
    holder=drilled(holder,_lcd_bolts(d),5.5,base_top,ft)
    return [PartRecord('56_company_LCD_7in_STEP','electronics',screen,(.10,.12,.16),COMPANY,
                       {'source':'LCD.STEP.gz','tilt_deg':tilt,'translate_mm':[round(v,3) for v in move],
                        'limits':'Company-supplied STEP; panel mounting holes not in the model, so the frame edges are clamped'}),
            PartRecord('56_LCD_holder_PRINTED','electronics',holder,(.80,.66,.40),'concept_support',
                       {'material':PRINT_NOTE,'design':f'{pt:g} mm back plate + {ft:g} mm foot + side clamps (2.5 mm lip over the glass border) + 2 ribs, one print',
                        'ground':'Plastic holder: the LCD frame is grounded by a cable to 58_ground_stud_M4'}),
            PartRecord('56_LCD_holder_bolts_CB5-16','electronics',_bolt_stack(_lcd_bolts(d),base_top,p['base_bottom_z'],ft,True),(.20,.22,.25),
                       'catalog_envelope',{'sku':'CB5-16 + M5 fender washer OD15 + M5 nut','quantity':2})]


# ---------------------------------------------------------------- speaker
SPEAKER_FRAME=((70.5,175.5),(271.6,327.9),(92.7,188.7))  # supplied frame + box envelope (local)
SPEAKER_SCREWS=((123,183.24),(81,98.24),(165,98.24))       # D3.4 holes, frame bosses Y 293.9..303.9 (local)
SPEAKER_BACK=308.4                                           # rear face of the frame ribs (local)


def _speaker_move(d):
    e=d.layout['electronics'];(x0,x1),(y0,_),(z0,_)=SPEAKER_FRAME
    return (e['speaker_x']-(x0+x1)/2,e['speaker_front_y']-y0,e['speaker_bottom_z']-z0)


def _speaker_bolts(d):
    e=d.layout['electronics'];x=e['speaker_x'];y=e['speaker_front_y']+18
    return [(x-59,y),(x+59,y)]


@lru_cache(maxsize=2)
def _speaker_shape(move):
    from .vendor import source
    (x0,x1),(y0,y1),(z0,z1)=SPEAKER_FRAME
    keep=span(x0,x1,y0,y1,z0,z1);solids=[];trimmed=0
    for s in source('SPEAKER.STEP').solids():
        b=s.bounding_box()
        if b.min.X<x0-1e-6 or b.max.X>x1+1e-6 or b.min.Z<z0-1e-6 or b.max.Z>z1+1e-6 or b.max.Y>y1+1e-6:
            cut=s&keep;trimmed+=1
            solids.extend(cut.solids())
        else:solids.append(s)
    return Compound([s.translate(move) for s in solids]),trimmed


def speaker(d):
    p=d.p;base_top=p['base_bottom_z']+p['base_thickness'];move=_speaker_move(d)
    unit,trimmed=_speaker_shape(move)
    T=lambda s:s.translate(move)
    # Mounting plate on the frame's rear ribs (local Y 308.4) with a window for the box; foot under the frame.
    b0=SPEAKER_BACK;e=d.layout['electronics'];ft=e['speaker_holder_foot_mm'];pt=e['speaker_holder_plate_mm']
    plate=T(span(66,180,b0,b0+pt,base_top+ft-move[2],192))-T(span(84,162,b0-5,b0+pt+2,101.5,176))
    for x,z in SPEAKER_SCREWS:plate-=T(cyly(1.7,pt+4,x,b0+pt/2,z))
    y_plate=b0+move[1]
    foot=span(move[0]+66-8,move[0]+180+4,e['speaker_front_y'],y_plate+pt,base_top,base_top+ft)
    for x0 in e['speaker_rib_local_x']:foot=foot+span(move[0]+x0,move[0]+x0+5,y_plate+pt-1,y_plate+pt+25,base_top,base_top+ft)
    from .suspension import yz_prism
    ribs=[yz_prism([(y_plate+pt,base_top+ft),(y_plate+pt+25,base_top+ft),(y_plate+pt,base_top+ft+40)],move[0]+x0,move[0]+x0+5) for x0 in e['speaker_rib_local_x']]
    bracket=plate+foot
    for r_ in ribs:bracket=bracket+r_
    bracket=drilled(bracket,_speaker_bolts(d),5.5,base_top,ft)-unit   # seat the print on the speaker's own shape
    # Clear the heads of the rear caster hardware that comes up through the base plate here.
    from .suspension import chassis_mount_holes
    for x,y,dia in chassis_mount_holes(d):
        bracket-=cylz(dia/2+3.5,25,x,y,base_top-.5)   # Rev17b: tall enough for the rising droop-stop nut at bump
    screws=Compound([T(cyly(2.75,3,x,b0+pt+1.5,z)+cyly(1.5,b0+pt-293.9,x,(b0+pt+293.9)/2,z)) for x,z in SPEAKER_SCREWS])
    return [PartRecord('57_company_speaker_STEP','electronics',unit,(.18,.18,.2),COMPANY,
                       {'source':'SPEAKER.STEP.gz','translate_mm':[round(v,3) for v in move],'trimmed_solids':trimmed,
                        'limits':'Company-supplied speaker box assembly; the free-hanging lead wire and connector outside the frame/box envelope are trimmed (cable routing TBD)'}),
            PartRecord('57_speaker_holder_PRINTED','electronics',bracket,(.80,.66,.40),'concept_support',
                       {'material':PRINT_NOTE,'design':f'{pt:g} mm plate on the frame rear bosses with a box window + {ft:g} mm foot + 2 ribs; grille faces rear (-Y)'}),
            PartRecord('57_speaker_screws_M3','electronics',screws,(.20,.22,.25),'catalog_envelope',
                       {'sku':'M3x22 + washer + nut (verify on the frame bosses)','quantity':3}),
            PartRecord('57_speaker_holder_bolts_CB5-16','electronics',_bolt_stack(_speaker_bolts(d),base_top,p['base_bottom_z'],ft,True),(.20,.22,.25),
                       'catalog_envelope',{'sku':'CB5-16 + M5 fender washer OD15 + M5 nut','quantity':2})]


# ---------------------------------------------------------------- shared Rev17 / P30-5 helpers
PRINT_NOTE = 'FDM 3D print, PETG or ASA, 4 perimeters, >=40% infill; washers under every bolt head (plastic creep)'

# ZLAC8015D, manual V1.0 Fig.1: 168 x 89 overall (ears incl.), 158 = ear-hole pitch, thickness 29.7, side-hole pitch 42.
# Local frame: X = length (168), Y = width (89), Z = 0 at the heatsink mounting face, +Z away from it.
ZLAC_EAR_HOLES = [(sx*79, sy*37.5) for sx in (-1, 1) for sy in (-1, 1)]     # X pitch 158 (figure); Y position estimated
ZLAC_FLANGE_T = 3.0                                                       # assumed heatsink base / ear thickness


def zlac_local_boxes():
    """(x0,x1,y0,y1,z0,z1) boxes of the ZLAC8015D in its local frame (figure-based; inner details estimated)."""
    L, W, H = 168, 89, 29.7
    boxes = [(-L/2, L/2, -W/2, W/2, 0, ZLAC_FLANGE_T),                    # aluminium heatsink base + ears
             (-72, 72, -W/2+2, W/2-2, ZLAC_FLANGE_T, H-4.7)]              # cover between the ears (estimated)
    for sx in (-1, 1):                                                    # 2 x 5 screw terminal blocks at the ends
        boxes.append((sx*72-(12 if sx > 0 else 0), sx*72+(0 if sx > 0 else 12), -30, 30, H-4.7, H))
    return boxes


def zlac_driver(to_span, hole):
    """Driver solid from local boxes; to_span(x0,x1,y0,y1,z0,z1) -> world Solid, hole(x, y, r) -> world cylinder along local Z."""
    shape = None
    for b in zlac_local_boxes():
        s = to_span(*b); shape = s if shape is None else shape+s
    for x, y in ZLAC_EAR_HOLES:
        shape -= hole(x, y, 1.75)
    return shape


def lcd_rib(tilt, move, back_y, foot_top, x0, x1, height=45, depth=30):
    """Printed triangular rib behind a plate (local back face y = back_y), from the foot top up the plate."""
    from math import cos, radians, sin
    from .suspension import yz_prism
    c, s = cos(radians(tilt)), sin(radians(tilt))
    def back(z):   # world (Y, Z) of the back face at world height z
        zl = (z-move[2]-back_y*s)/c
        return back_y*c-zl*s+move[1], z
    a, b = back(foot_top), back(foot_top+height)
    return yz_prism([a, (a[0]-depth, foot_top), b], x0, x1)


def battery_strap(case, along, positions, under_z, plate_t, width=25, t=1.5):
    """Hold-down straps around a box case (x0,x1,y0,y1,z0,z1), closing under the floor plate.

    along = 'x': straps at x positions run over the top in the YZ plane (legs on the y faces), 'y': the other way.
    Returns (strap solid, slot rectangles (x0,x1,y0,y1), buckle solid)."""
    x0, x1, y0, y1, z0, z1 = case; straps = []; slots = []; buckles = []
    for p in positions:
        if along == 'x':
            ring = span(p-width/2, p+width/2, y0-t, y1+t, under_z-t, z1+t)-span(p-width/2-1, p+width/2+1, y0, y1, under_z, z1)
            slots += [(p-width/2-1, p+width/2+1, y-t-.5, y+.5) for y in (y0,)]+[(p-width/2-1, p+width/2+1, y1-.5, y1+t+.5)]
            buckles.append(span(p-width/2-2, p+width/2+2, (y0+y1)/2-20, (y0+y1)/2+20, z1+t, z1+t+9))
        else:
            ring = span(x0-t, x1+t, p-width/2, p+width/2, under_z-t, z1+t)-span(x0, x1, p-width/2-1, p+width/2+1, under_z, z1)
            slots += [(x0-t-.5, x0+.5, p-width/2-1, p+width/2+1), (x1-.5, x1+t+.5, p-width/2-1, p+width/2+1)]
            buckles.append(span((x0+x1)/2-20, (x0+x1)/2+20, p-width/2-2, p+width/2+2, z1+t, z1+t+9))
        straps.append(ring)
    return Compound(straps), slots, Compound(buckles)


# ---------------------------------------------------------------- DC/DC
def dcdc(d):
    p=d.p;e=d.layout['electronics'];base_top=p['base_bottom_z']+p['base_thickness']
    w,l,h=e['dcdc_size'];x,y,bottom=e['dcdc_x'],e['dcdc_y'],e['dcdc_bottom_z']
    rail_y=y+l/2-37.5;rail=span(x-60,x+60,rail_y-17.5,rail_y+17.5,bottom-7.5,bottom)
    legs=Compound([cylz(3.5,bottom-7.5-base_top,x+dx,rail_y,base_top) for dx in (-50,50)])
    return [PartRecord('55_DCDC_DDR-240B-24_ENVELOPE','electronics',span(x-w/2,x+w/2,y-l/2,y+l/2,bottom,bottom+h),(.55,.57,.60),'envelope_only',
                       {'model':'MEAN WELL DDR-240B-24 (company stock)','size_mm':[w,l,h],
                        'limits':'Size reservation from the remembered datasheet size 63x125.2x128.5 (lying on its side); confirm the datasheet, input range vs 7S 21-29.4 V and derating before fixing the layout'}),
            PartRecord('55_DIN_rail_TS35','electronics',rail,(.70,.72,.75),'catalog_envelope',{'length_mm':120}),
            PartRecord('55_DIN_rail_standoffs_M4','electronics',legs,(.55,.58,.62),'concept_support',
                       {'quantity':2,'height_mm':round(bottom-7.5-base_top,2)})]
