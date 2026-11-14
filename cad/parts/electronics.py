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
    for x in (-e['driver_x'],e['driver_x']):items.append(('53_motor_driver_DUMMY_'+str(x),box(70,98,44,x,e['driver_y'],e['driver_center_z']),(.28,.38,.52)))
    records=[PartRecord(n,'electronics',s,c,'envelope_only') for n,s,c in items]
    if d.variant=='baseline':return records
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
    return records


def mount_holes(d):
    """(x, y, diameter) of base-plate holes for the Rev15 electronics brackets."""
    if d.variant=='baseline':return []
    return [(x,y,5.5) for x,y in _lcd_bolts(d)+_speaker_bolts(d)]


def _bolt_stack(points,base_top,base_bottom):
    """CB5-12 head on a 3t foot, through the foot and the 3t base plate, M5 nut underneath (2 mm past the nut)."""
    from .suspension import screw,nut
    return Compound([s for x,y in points for s in (screw(x,y,base_top+3,5,12)+nut(x,y,base_bottom-4,8,4,5)).solids()])


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
    # Bracket plate 3 mm behind the LCD driver board (local Y -10.3), trimmed at the foot top.
    plate=_place(span(-95.9,95.9,-15.3,-12.3,-90,45),tilt,move)&span(-120,120,0,400,base_top+3,400)
    bottom=(plate&span(-120,120,0,400,base_top+3,base_top+3.5)).bounding_box()
    foot=span(-110,110,bottom.min.Y,e['lcd_front_y'],base_top,base_top+3)
    bracket=drilled(plate+foot,_lcd_bolts(d),5.5,base_top,3)
    # Printed side clamps: grip the LCD frame edges (X +-89.9) with a 2.5 mm lip over the glass border, screwed to the plate.
    clamps=[]
    for s in (-1,1):
        block=span(*sorted((s*89.9,s*95.9)),-12.3,4.6,-40,40)+span(*sorted((s*87.4,s*89.9)),3.1,4.6,-40,40)
        clamps.append(_place(block,tilt,move))
    return [PartRecord('56_company_LCD_7in_STEP','electronics',screen,(.10,.12,.16),COMPANY,
                       {'source':'LCD.STEP.gz','tilt_deg':tilt,'translate_mm':[round(v,3) for v in move],
                        'limits':'Company-supplied STEP; panel mounting holes not in the model, so the frame edges are clamped'}),
            PartRecord('56_LCD_bracket_3t','electronics',bracket,(.48,.65,.78),'concept_fabrication',
                       {'material':f'S355 3t laser-cut, 1 bend ({90+tilt:g} deg), window-free plate 3 mm behind the LCD board'}),
            PartRecord('56_LCD_side_clamps_PRINTED','electronics',Compound(clamps),(.80,.66,.40),'concept_support',
                       {'quantity':2,'material':'3D-printed clamp, 2x M3 into the bracket each (not drawn)'}),
            PartRecord('56_LCD_bracket_bolts_CB5-12','electronics',_bolt_stack(_lcd_bolts(d),base_top,p['base_bottom_z']),(.20,.22,.25),
                       'catalog_envelope',{'sku':'CB5-12 + M5 nut','quantity':2})]


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
    b0=SPEAKER_BACK
    plate=T(span(66,180,b0,b0+3,base_top+3-move[2],192))-T(span(84,162,b0-5,b0+5,101.5,176))
    for x,z in SPEAKER_SCREWS:plate-=T(cyly(1.7,10,x,b0+1.5,z))
    y_plate=b0+move[1]
    foot=span(move[0]+66-8,move[0]+180+4,d.layout['electronics']['speaker_front_y'],y_plate+3,base_top,base_top+3)
    bracket=drilled(plate+foot,_speaker_bolts(d),5.5,base_top,3)
    # Clear the heads of the rear caster hardware that comes up through the base plate here.
    from .suspension import chassis_mount_holes
    for x,y,dia in chassis_mount_holes(d):
        bracket-=cylz(dia/2+3.5,7,x,y,base_top-.5)
    screws=Compound([T(cyly(2.75,3,x,b0+4.5,z)+cyly(1.5,b0+3-293.9,x,(b0+3+293.9)/2,z)) for x,z in SPEAKER_SCREWS])
    return [PartRecord('57_company_speaker_STEP','electronics',unit,(.18,.18,.2),COMPANY,
                       {'source':'SPEAKER.STEP.gz','translate_mm':[round(v,3) for v in move],'trimmed_solids':trimmed,
                        'limits':'Company-supplied speaker box assembly; the free-hanging lead wire and connector outside the frame/box envelope are trimmed (cable routing TBD)'}),
            PartRecord('57_speaker_bracket_3t','electronics',bracket,(.48,.65,.78),'concept_fabrication',
                       {'material':'S355 3t laser-cut, 1 bend; plate on the frame rear bosses with a box window; grille faces rear (-Y)'}),
            PartRecord('57_speaker_screws_M3','electronics',screws,(.20,.22,.25),'catalog_envelope',
                       {'sku':'M3x20 + nut (verify on the frame bosses)','quantity':3}),
            PartRecord('57_speaker_bracket_bolts_CB5-12','electronics',_bolt_stack(_speaker_bolts(d),base_top,p['base_bottom_z']),(.20,.22,.25),
                       'catalog_envelope',{'sku':'CB5-12 + M5 nut','quantity':2})]


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
