"""Single data entry point; JSON remains editable without an AI session."""
from dataclasses import dataclass,field
from pathlib import Path
import json
import math

ROOT = Path(__file__).resolve().parent.parent

@dataclass
class Design:
    layout: dict
    parts: dict
    variant: str = "current"
    motion: dict = field(default_factory=dict)

    @property
    def p(self): return self.layout["parameters"]
    @property
    def profile(self): return self.layout["profile"]

def load_design(variant="current"):
    folder = ROOT/"cad"/("baseline/rev05" if variant=="baseline" else "")
    d=Design(json.loads((folder/"layout.json").read_text(encoding="utf-8")),
             json.loads((folder/"parts.json").read_text(encoding="utf-8")), variant)
    validate_parameters(d)
    return d

def validate_parameters(d):
    for key in ("base_width","base_length","base_thickness","top_width","top_length","top_thickness","drive_track"):
        value=d.p[key]
        if not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError(f"{key} must be positive and finite")
    if d.p["top_bottom_z"]<=d.p["base_bottom_z"]+d.p["base_thickness"]:
        raise ValueError("Upper deck must be above lower plate")
    if d.p["top_bottom_z"]<=d.layout["camera_mount"]["profile_bottom_z"]:
        raise ValueError("Upper deck must be above the camera hanger seat")
    if d.variant!='baseline':
        s=d.layout['suspension']
        q=s['drive']
        for key in ('travel_mm','spring_od_mm','spring_id_mm','display_turns','loaded_length_mm','free_length_mm',
                    'solid_length_mm','max_deflection_mm','rate_N_mm','pivot_x_mm','pivot_z_mm','spring_x_mm',
                    'arm_thickness_mm','arm_wall_y_mm','arm_wall_top_z_mm','arm_wall_bottom_z_mm','end_wall_x_mm',
                    'bracket_thickness_mm','ear_inner_y_mm','pin_d_mm','pin_length_mm','bearing_id_mm',
                    'bearing_od_mm','bearing_width_mm','hanger_thickness_mm'):
            if not isinstance(q[key],(int,float)) or not math.isfinite(q[key]) or q[key]<=0:
                raise ValueError('Invalid drive suspension parameter: '+key)
        if q['travel_mm']>20:raise ValueError('Frame redesign required for greater drive travel')
        if q['shim_mm']<0 or q['spring_od_mm']<=q['spring_id_mm']:raise ValueError('Spring fit is invalid: drive')
        if not q['pin_d_mm']==q['bearing_id_mm']<q['bearing_od_mm']:raise ValueError('Drive pivot bearing fit is invalid')
        if not q['pivot_x_mm']<q['spring_x_mm']<q['end_wall_x_mm']<d.p['drive_track']/2:
            raise ValueError('Drive pivot, spring and axle order is invalid')
        if q['arm_wall_top_z_mm']+q['travel_mm']/2*(d.p['base_width']/2-50-q['pivot_x_mm'])/(d.p['drive_track']/2-q['pivot_x_mm'])>=d.p['base_bottom_z']:
            raise ValueError('Drive arm reaches the base plate at full bump')
        span=2*(q['ear_inner_y_mm']+q['bracket_thickness_mm'])
        if not (q['arm_wall_y_mm']+q['bearing_width_mm']<q['ear_inner_y_mm'] and span<q['pin_length_mm']):
            raise ValueError('Drive pivot pin stack is invalid')
        r=s['rear_rocker']
        for key in ('roll_limit_deg','pivot_z_mm','beam_thickness_mm','beam_half_length_mm','beam_width_mm',
                    'pin_d_mm','pin_length_mm','bearing_od_mm','bearing_width_mm','stop_x_mm'):
            if not isinstance(r[key],(int,float)) or not math.isfinite(r[key]) or r[key]<=0:
                raise ValueError('Invalid rear rocker parameter: '+key)
        if r['roll_limit_deg']>3:raise ValueError('Rear roll limit too large for lateral stability')
        if not r['tab_bottom_z_mm']<r['pivot_z_mm']-r['bearing_od_mm']/2<r['pivot_z_mm']+r['bearing_od_mm']/2<d.parts['caster']['cad_height']:
            raise ValueError('Rear pivot bearing does not fit in the beam tabs')
        if r['pin_length_mm']<r['beam_width_mm']:raise ValueError('Rear pivot pin too short')
        installed=q['loaded_length_mm']-q['shim_mm']
        half=q['travel_mm']/2*(q['spring_x_mm']-q['pivot_x_mm'])/(d.p['drive_track']/2-q['pivot_x_mm'])
        if (installed+half>q['free_length_mm'] or installed-half<=q['solid_length_mm']
                or q['free_length_mm']-(installed-half)>q['max_deflection_mm']):
            raise ValueError('Catalog spring travel is exceeded: drive')

def set_parameter(key, value):
    path=ROOT/"cad/layout.json"
    data=json.loads(path.read_text(encoding="utf-8"));node=data
    keys=key.split(".")
    for field in keys[:-1]: node=node[field]
    if keys[-1] not in node: raise KeyError(key)
    old=node[keys[-1]]
    if isinstance(old,(dict,list)) or isinstance(old,bool): raise ValueError("Use JSON editor for structured values")
    value=json.loads(value)
    if isinstance(old,(int,float)) and (not isinstance(value,(int,float)) or not math.isfinite(value)):
        raise ValueError("Expected finite number")
    node[keys[-1]]=value
    # Validate using the candidate without writing a broken parameter file.
    if keys[0]=="parameters" and isinstance(value,(int,float)) and value<=0 and keys[-1] not in ("battery_y",):
        raise ValueError("Expected positive dimension")
    candidate=Design(data,json.loads((ROOT/'cad/parts.json').read_text(encoding='utf-8')))
    validate_parameters(candidate)
    path.write_text(json.dumps(data,indent=2)+"\n",encoding="utf-8")
    return {"key":key,"old":old,"new":value}
