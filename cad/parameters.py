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
        for kind,maximum in (('drive',20),('caster',10)):
            q=s[kind]
            for key in ('travel_mm','spring_od_mm','spring_id_mm','display_turns','loaded_length_mm','free_length_mm',
                        'solid_length_mm','max_deflection_mm','rate_N_mm','unsprung_mass_kg_estimate','pivot_z_mm',
                        'arm_thickness_mm','bracket_thickness_mm','pin_d_mm','pin_head_d_mm','boss_thickness_mm'):
                if not isinstance(q[key],(int,float)) or not math.isfinite(q[key]) or q[key]<=0:
                    raise ValueError('Invalid suspension parameter: '+kind+'.'+key)
            if q['travel_mm']>maximum:raise ValueError('Frame redesign required for greater '+kind+' travel')
            if q['shim_mm']<0 or q['spring_od_mm']<=q['spring_id_mm']:raise ValueError('Spring fit is invalid: '+kind)
            if q['spring_id_mm']<=8.4:raise ValueError('Spring must pass the M8 stud: '+kind)
            if q['ear_gap_mm']<q['bush_flange_t_mm']+.3:raise ValueError('No room for the bush flange: '+kind)
            # Rev12b: bush bore is the STEP nominal 10.0 (F7 after press-fit), equal to the g6 pin nominal.
            if not q['pin_d_mm']<=q['bush_bore_mm']<q['bush_od_mm']<q['bush_flange_d_mm']:raise ValueError('Hinge bush fit is invalid: '+kind)
            walls=((q['arm_wall_y_mm']+q['arm_thickness_mm'],q['arm_wall_y_pos_mm']+q['arm_thickness_mm']) if kind=='drive'
                   else (q['arm_half_width_mm'],)*2)
            pin=round(sum(walls)+2*(q['ear_gap_mm']+q['bracket_thickness_mm'])+1,1)
            if not 25<=pin<=100:raise ValueError('Hinge pin outside catalog CLBKHM10 L25-100: '+kind)
            if abs(float(q['pin_sku'].split('-')[-1])-pin)>1e-6:raise ValueError(f'Hinge pin SKU must be CLBKHM10-{pin:g}: '+kind)
            if kind=='drive':
                length=d.p['drive_track']/2-q['pivot_x_mm'];ratio=(q['spring_x_mm']-q['pivot_x_mm'])/length
                seat=q['arm_bottom_z_mm']+q['arm_thickness_mm']
                if not 0<q['pivot_x_mm']<q['spring_x_mm']<q['end_wall_x_mm']<d.p['drive_track']/2:
                    raise ValueError('Drive pivot, spring and axle order is invalid')
                rise=q['travel_mm']/2*(q['end_wall_x_mm']+q['arm_thickness_mm']/2-q['pivot_x_mm'])/length
                if q['arm_top_z_mm']+rise>=d.p['base_bottom_z']:raise ValueError('Drive arm reaches the base plate at full bump')
                if q['spring_x_mm']+q['spring_od_mm']/2>=d.p.get('base_well_inner_x',d.p['base_width']/2-50):
                    raise ValueError('Drive spring must sit under the base plate')
                if q['end_wall_half_width_mm']<11 or q['arm_wall_y_pos_mm']<11:
                    raise ValueError('Drive axle wall must carry the axle washer and nut')
                if not q['pivot_x_mm']+q['pivot_tab_x_mm']<q['bump_stop_x_mm']<q['spring_x_mm']-q['spring_od_mm']/2-6:
                    raise ValueError('Drive bump stop must sit between the pivot tabs and the spring')
                if not -q['arm_wall_y_mm']<q['bump_stop_y_mm']-6<q['bump_stop_y_mm']+6<q['arm_wall_y_pos_mm']:
                    raise ValueError('Drive bump stop must land on the arm floor')
            else:
                ratio=(q['arm_length_mm']-q['spring_offset_mm'])/q['arm_length_mm'];seat=d.parts['caster']['cad_height']+q['arm_thickness_mm']+q.get('seat_spacer_mm',0)
                if q['arm_length_mm']<=q['travel_mm']:raise ValueError('Caster arm too short')
            if abs(d.p['base_bottom_z']-seat-q['loaded_length_mm'])>1e-6:
                raise ValueError('Spring gap does not match the base plate: '+kind)
            # Urethane stopper on the arm + M6 striker below the base plate (5mm jam nut).
            # The stopper sits between pivot and wheel, so its rise is below half the wheel travel.
            if seat+q['bump_stopper_height_mm']+q['travel_mm']/2>=d.p['base_bottom_z']-5:
                raise ValueError('Bump stopper leaves no room for the striker bolt: '+kind)
            installed=q['loaded_length_mm']-q['shim_mm']
            half=q['travel_mm']/2*ratio
            if (installed+half>q['free_length_mm'] or installed-half<=q['solid_length_mm']
                    or q['free_length_mm']-(installed-half)>q['max_deflection_mm']):
                raise ValueError('Catalog spring travel is exceeded: '+kind)

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
