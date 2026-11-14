from .common import *
from build123d import Compound

def make(d):
    p=d.p;e=d.layout['electronics']
    items=[('50_company_7S9P_BATTERY_DUMMY',box(p['battery_width'],p['battery_length'],p['battery_height'],0,p['battery_y'],p['battery_center_z']),(.86,.63,.36)),
           ('51_controller_PCB_DUMMY',box(165,104,7,0,e['logic_y'],e['pcb_center_z']),(.12,.46,.37)),
           ('52_control_computer_DUMMY',box(146,82,23,0,e['logic_y'],e['computer_center_z']),(.13,.19,.28)),
           ('54_REAR_power_distribution_DUMMY',box(160,65,40,0,e['power_bay_y'],e['power_bay_center_z']),(.65,.39,.24))]
    for x in (-e['driver_x'],e['driver_x']):items.append(('53_motor_driver_DUMMY_'+str(x),box(70,98,44,x,e['driver_y'],e['driver_center_z']),(.28,.38,.52)))
    records=[PartRecord(n,'electronics',s,c,'envelope_only') for n,s,c in items]
    if d.variant=='baseline':return records
    # Rev13: every envelope stands on something. M4 hex standoffs (AF7 drawn as D7) at the corners,
    # inset 8 mm; the computer stands on the controller PCB. Battery on 15 mm rubber pads (strap not drawn).
    base_top=p['base_bottom_z']+p['base_thickness'];shapes={n:s for n,s,_ in items}
    pcb=shapes['51_controller_PCB_DUMMY'].bounding_box().max.Z
    for name,shape,_ in items:
        b=shape.bounding_box();floor=pcb if name.startswith('52_') else base_top;h=b.min.Z-floor
        if h<=0:continue
        radius=7.5 if name.startswith('50_') else 3.5
        legs=[cylz(radius,h,x,y,floor) for x in (b.min.X+8,b.max.X-8) for y in (b.min.Y+8,b.max.Y-8)]
        label='50_battery_support_pads' if name.startswith('50_') else name.split('_DUMMY')[0]+'_standoffs_M4'+name.split('_DUMMY')[1]
        records.append(PartRecord(label,'electronics',Compound(legs),(.55,.58,.62),'concept_support',
                                  {'height_mm':round(h,2),'quantity':4,'supports':name}))
    return records
