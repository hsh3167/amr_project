"""Lazy part-group factories and named assembly. No export or GUI side effects."""
from collections import OrderedDict
from build123d import Compound
from .parameters import load_design
from .parts import chassis,structure,drive,casters,electronics,sensors

GROUPS=('chassis','payload_interface','brackets','structure','motor_left','motor_right',
        'caster_lf','caster_lb','caster_rf','caster_rb','electronics','lidars','cameras','suspension')

def build_records(design=None,only=None):
    d=design or load_design();wanted=set(only or GROUPS);unknown=wanted-set(GROUPS)
    if unknown:raise ValueError('Unknown groups: '+','.join(sorted(unknown)))
    r=[]
    if wanted&{'chassis','payload_interface'}:r.extend(x for x in chassis.make(d) if x.group in wanted)
    if 'brackets' in wanted:r.extend(chassis.brackets(d))
    if 'structure' in wanted:r.extend(structure.make(d))
    for name,side in (('motor_left',-1),('motor_right',1)):
        if name in wanted:r.extend(drive.make(d,side))
    for x in (-d.p['caster_x'],d.p['caster_x']):
        for y in (-d.p['caster_y'],d.p['caster_y']):
            n=('l' if x<0 else 'r')+('f' if y>0 else 'b')
            if 'caster_'+n in wanted:r.extend(casters.make(d,x,y))
    if 'electronics' in wanted:r.extend(electronics.make(d))
    if 'lidars' in wanted:r.extend(sensors.lidars(d))
    if 'cameras' in wanted:r.extend(sensors.cameras(d))
    if d.variant!='baseline' and 'suspension' in wanted:
        from .parts.suspension import make
        r.extend(make(d))
    return r

def assemble(records,name='AMR'):
    groups=OrderedDict()
    for item in records:
        shape=item.shape
        # BREP cache reloads retain topology but not the Python assembly tree.
        # Restore solid children so cache hits export the same compact assembly
        # as cache misses instead of duplicating large OEM geometry definitions.
        if isinstance(shape,Compound) and not shape.children and len(shape.solids())>1:
            shape=Compound(label=item.name,children=list(shape.solids()))
            shape.color=item.shape.color
        groups.setdefault(item.group,[]).append(shape)
    return Compound(label=name,children=[Compound(label=k,children=v) for k,v in groups.items()])
