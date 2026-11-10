from .common import *

def make(d,x,y):
    n=('L' if x<0 else 'R')+('F' if y>0 else 'B');group='caster_'+n.lower();c=d.parts['caster'];r=[]
    if d.variant!='baseline':
        from .vendor import caster_records
        return caster_records(d,x,y,n,group)
    drop=d.p['base_bottom_z']-c['overall_height']
    for i,(dx,dy) in enumerate(((-27,-12),(-27,12),(27,-12),(27,12))):
        s=cylz(5.5,drop,x+dx,y+dy,c['overall_height'])-cylz(3.5,drop+2,x+dx,y+dy,c['overall_height']-1)
        r.append(PartRecord('40_caster_spacer_CONCEPT_'+n+str(i),group,s,(.30,.45,.58)))
    plate=drilled(box(65,48,5,x,y,79.5),[(x+dx,y+dy) for dx in (-27,27) for dy in (-12,12)],7,77,5)
    r.append(PartRecord('41_GL65_plate_65x48_HOLE54x24_'+n,group,plate,(.75,.80,.83)))
    r.append(PartRecord('42_swivel_head_'+n,group,cylz(12,15,x,y,60),(.31,.39,.46)))
    for dx in (-17,17):r.append(PartRecord('43_fork_'+n+'_'+str(dx),group,box(4.5,32,38,x+dx,y-12,51),(.68,.73,.79)))
    r.append(PartRecord('44_GL65_wheel_64OD_'+n,group,cylx(32,23,x,y-22,32),(.45,.54,.64)))
    return r
