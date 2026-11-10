from .common import *

def make(d):
    p=d.p;e=d.layout['electronics']
    items=[('50_company_7S9P_BATTERY_DUMMY',box(p['battery_width'],p['battery_length'],p['battery_height'],0,p['battery_y'],p['battery_center_z']),(.86,.63,.36)),
           ('51_controller_PCB_DUMMY',box(165,104,7,0,e['logic_y'],e['pcb_center_z']),(.12,.46,.37)),
           ('52_control_computer_DUMMY',box(146,82,23,0,e['logic_y'],e['computer_center_z']),(.13,.19,.28)),
           ('54_REAR_power_distribution_DUMMY',box(160,65,40,0,e['power_bay_y'],e['power_bay_center_z']),(.65,.39,.24))]
    for x in (-e['driver_x'],e['driver_x']):items.append(('53_motor_driver_DUMMY_'+str(x),box(70,98,44,x,e['driver_y'],e['driver_center_z']),(.28,.38,.52)))
    return [PartRecord(n,'electronics',s,c,'envelope_only') for n,s,c in items]
