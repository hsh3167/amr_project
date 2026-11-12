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
    if d.variant!='baseline':
        for kind,maximum in (('drive',20),('caster',10)):
            q=d.layout['suspension'][kind]
            for key in ('travel_mm','spring_od_mm','spring_id_mm','display_turns','loaded_length_mm','free_length_mm',
                        'solid_length_mm','max_deflection_mm','rate_N_mm','shaft_d_mm','shaft_length_mm',
                        'bush_id_mm','bush_od_mm','bush_length_mm'):
                if not isinstance(q[key],(int,float)) or not math.isfinite(q[key]) or q[key]<=0:raise ValueError('Invalid suspension parameter: '+key)
            if q['travel_mm']>maximum:raise ValueError('Frame redesign required for greater '+kind+' travel')
            if q['shim_mm']<0 or q['spring_od_mm']<=q['spring_id_mm'] or not q['shaft_d_mm']<q['bush_id_mm']<q['bush_od_mm']:
                raise ValueError('Spring or guide fit is invalid: '+kind)
            installed=q['loaded_length_mm']-q['shim_mm']
            half=q['travel_mm']/2
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
