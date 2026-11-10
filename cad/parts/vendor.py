"""Manufacturer BReps, normalized without scaling. See vendor/provenance.json."""
from functools import lru_cache
from pathlib import Path
import gzip
import tempfile
import hashlib,json
from build123d import import_step, Compound,import_brep,export_brep,Location,Rotation
from OCP.ShapeFix import ShapeFix_ShapeTolerance
from ..parameters import ROOT
from .common import *

@lru_cache(maxsize=2)
def source(name):
    path=ROOT/'cad/vendor'/name
    if path.exists():return import_step(path)
    compressed=path.with_suffix(path.suffix+'.gz')
    # Preserve original bytes; compressed storage is only a repository-size choice.
    with tempfile.TemporaryDirectory() as folder:
        tmp=Path(folder)/name;tmp.write_bytes(gzip.decompress(compressed.read_bytes()))
        return import_step(tmp)

def caster_records(d,x,y,n,group):
    original=source('GL-65-ASF-EUS.stp');r=[]
    # Source top-plate center -> origin XY, wheel tangent -> ground; axle -> X.
    for index,prefix in ((0,'43_GL65_OEM_fork_'),(1,'41_GL65_OEM_plate_head_'),(2,'44_GL65_OEM_wheel_')):
        s=rz(original.solids()[index].translate((-19.251901528418,23.9893391369419,51.7488089481697)),-90)
        r.append(PartRecord(prefix+n,group,s.translate((x,y,d.motion.get(n,0))),(.55,.64,.72),
                            'manufacturer_step_EUS_reference',{'source':'GL-65-ASF-EUS.stp','moving_wheel':n,'cad_height_mm':83}))
    return r

@lru_cache(maxsize=8)
def camera_shape(side,x,y,z,pitch,yaw):
    raw=ROOT/'cad/vendor/AstraMiniPro_3D_V0.1_20221012.stp.gz'
    key=hashlib.sha256(raw.read_bytes()+Path(__file__).read_bytes()+json.dumps([side,x,y,z,pitch,yaw]).encode()).hexdigest()[:20]
    folder=ROOT/'output/cache';folder.mkdir(parents=True,exist_ok=True)
    cache=folder/('camera_'+key+'.brep');report=cache.with_suffix('.json')
    if cache.exists():return import_brep(cache),json.loads(report.read_text())
    original=source('AstraMiniPro_3D_V0.1_20221012.stp');b=original.bounding_box()
    center=(b.min+b.max)*.5
    # Source optical axis +Z, height Y -> local front +Y and up +Z.
    loc=Location((side*x,y,z))*Rotation(0,0,-side*yaw)*Rotation(-pitch,0,0)*Rotation(0,0,180)*Rotation(90,0,0)*Location(-center)
    solids=[];repairs=[]
    # This OEM BRep has borderline p-curve tolerances after rigid placement.
    # Retain the untouched source; qualify any OCCT repair against placed geometry.
    for i,original_solid in enumerate(original.solids()):
        solid=Solid(original_solid.wrapped.Moved(loc.wrapped))
        # Raise only numerical topology tolerances to 0.00001 mm; no surfaces,
        # vertices, placements, units or dimensions are changed.
        ShapeFix_ShapeTolerance().LimitTolerance(solid.wrapped,1e-5)
        if not solid.is_valid:
            fixed=solid.fix();a=solid.bounding_box();b=fixed.bounding_box()
            err=max(abs(v-w) for v,w in zip(tuple(a.min)+tuple(a.max),tuple(b.min)+tuple(b.max)))
            delta=abs(fixed.volume-solid.volume)
            if not fixed.is_valid or err>1e-5 or delta>max(.001,abs(solid.volume)*1e-7):
                raise ValueError('OEM camera repair exceeded tolerance: '+str(i))
            repairs.append({'solid':i,'bounds_delta_mm':err,'volume_delta_mm3':delta})
            solid=fixed
        solids.append(solid)
    s=Compound(children=solids)
    detail={'cache_key':key,'repairs':repairs,'source_preserved':True,'pose':[side,x,y,z,pitch,yaw],
            'source_solid_volume_mm3':sum(v.volume for v in original.solids()),
            'placed_solid_volume_mm3':sum(v.volume for v in solids),
            'minimum_topology_tolerance_mm':1e-5,
            'open_reference_shells_excluded':3}
    export_brep(s,cache);report.write_text(json.dumps(detail,indent=2))
    return s,detail

def camera_records(d,side):
    s,detail=camera_shape(side,d.p['camera_x'],d.p['camera_y'],d.p['camera_z'],d.p['camera_pitch_down_deg'],d.p['camera_yaw_deg'])
    n='LEFT' if side<0 else 'RIGHT'
    return [PartRecord('70_Orbbec_Astra_Mini_Pro_MX6000_'+n,'cameras',s,(.65,.50,.82),
                       'manufacturer_step_family_revision_reference',{
                           'source':'AstraMiniPro_3D_V0.1_20221012.stp','source_solids':58,
                           'placement_repair':detail,
                           'datum':'original bounding-box center; source optics +Z mapped to local +Y',
                           'limits':'3D V0.1 differs from 2D V1.2 depth; verify MX6000 A V1.2 actual hardware and adapter before fabrication'} )]
