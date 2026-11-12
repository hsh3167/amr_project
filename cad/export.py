"""Opt-in output. Importing a factory never writes files or opens an app."""
from pathlib import Path
import json
from io import BytesIO
import re
import tempfile
from build123d import Location,Shape,Unit,export_step,export_stl,import_step
from build123d.exporters3d import _create_xde
from .assembly import assemble
from .validation import metrics
from OCP.ShapeFix import ShapeFix_ShapeTolerance
from OCP.BRepTools import BRepTools
from OCP.Message import Message_ProgressRange
from OCP.RWGltf import RWGltf_CafWriter
from OCP.TCollection import TCollection_AsciiString
from OCP.TColStd import TColStd_IndexedDataMapOfStringString

def display_glb(root,path):
    """Tessellate the CAD assembly and merge faces by material within each part."""
    original=root.location
    try:
        # Same Y-up and millimetre-to-metre convention as build123d export_gltf.
        root.location *= Location((0,0,0),(1,0,0),-90)
        BRepTools.Clean_s(root.wrapped)
        root.mesh(.4,.35)
        # Reuse build123d's named, colored assembly writer (pinned version).
        document=_create_xde(root,Unit.MM,auto_naming=False)
        writer=RWGltf_CafWriter(TCollection_AsciiString(str(path)),True)
        writer.SetParallel(True)
        writer.SetMergeFaces(True)
        # CAD materials are solid colors and have no UV-mapped textures.
        writer.SetForcedUVExport(False)
        if not writer.Perform(document,TColStd_IndexedDataMapOfStringString(),Message_ProgressRange()):
            raise ValueError('GLB export failed')
    finally:
        BRepTools.Clean_s(root.wrapped)
        root.location=original

def normalize_oem_roundtrip(root):
    """Explicit import normalization for the supplied OEM camera's STEP p-curves.

    STEP serialization loses its tiny edge-tolerance floor. Never normalize an
    unrelated component or hide a geometry change. Stock import validity is reported.
    """
    normalized=[]
    def walk(node):
        if node.label.startswith('70_'):
            before=sum(s.volume for s in node.solids())
            ShapeFix_ShapeTolerance().LimitTolerance(node.wrapped,1e-5)
            delta=abs(sum(s.volume for s in node.solids())-before)
            if delta>1e-8:raise ValueError('Camera import normalization changed volume')
            normalized.append({'name':node.label,'minimum_topology_tolerance_mm':1e-5,'volume_change_mm3':delta})
        else:
            for child in node.children:walk(child)
    walk(root)
    return normalized

def step_file(shape,path):
    # OCCT Windows filename handling fails for some long non-ASCII part paths.
    if shape.parent is not None:
        # XDE expects a root object. Keep the BRep and placement without its
        # assembly parent, which is outside the individual export traversal.
        standalone=Shape.cast(shape.wrapped)
        standalone.label=shape.label
        standalone.color=shape.color
        shape=standalone
    stream=BytesIO()
    if not export_step(shape,stream,timestamp='2026-11-11T00:00:00'):
        raise ValueError(f'STEP write failed: {path}')
    data=stream.getvalue()
    if b'\nDATA;\n#' not in data:
        raise ValueError(f'STEP contains no geometry: {path}')
    Path(path).write_bytes(data)

def stl_file(shape,path):
    with tempfile.TemporaryDirectory() as temp:
        tmp=Path(temp)/'part.stl'
        if not export_stl(shape,tmp,tolerance=.15,angular_tolerance=.2):raise ValueError('STL write failed')
        data=tmp.read_bytes()
        if len(data)<84 or (len(data)-84)%50 or int.from_bytes(data[80:84],'little')!=(len(data)-84)//50:
            raise ValueError('Invalid binary STL output')
        Path(path).write_bytes(data)

def write(records,folder,formats=('step',),individual=False,name='amr_current'):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    root=assemble(records,name);files=[];roundtrips=[]
    for fmt in formats:
        path=folder/(name+'.'+fmt)
        if fmt=='step':
            step_file(root,path)
            loaded=import_step(path)
            raw_valid=loaded.is_valid
            normalized=[] if raw_valid else normalize_oem_roundtrip(loaded)
            valid=raw_valid or bool(normalized) and loaded.is_valid
            if not valid or len(loaded.solids())!=sum(len(x.shape.solids()) for x in records):
                raise ValueError('STEP roundtrip failed')
            roundtrips.append({'file':path.name,'raw_import_valid':raw_valid,'valid_after_explicit_normalization':valid,
                               'normalized_components':normalized,'solids':len(loaded.solids())})
        elif fmt=='stl':stl_file(root,path)
        elif fmt=='glb':
            with tempfile.TemporaryDirectory() as temp:
                tmp=Path(temp)/'amr.glb'
                display_glb(root,tmp)
                path.write_bytes(tmp.read_bytes())
        else:raise ValueError('Unsupported format: '+fmt)
        files.append(path.name)
    if individual:
        part_dir=folder/'parts';part_dir.mkdir(exist_ok=True)
        for item in records:
            for fmt in formats:
                if fmt=='glb':continue
                path=part_dir/(re.sub(r'[^A-Za-z0-9_.-]','_',item.name)+'.'+fmt)
                if fmt=='step':step_file(item.shape,path)
                else:stl_file(item.shape,path)
    manifest={'engine':'build123d','root':name,'files':files,'step_roundtrip':roundtrips,'components':[
        {'name':x.name,'group':x.group,'status':x.status,'metadata':x.metadata,**metrics(x.shape)} for x in records]}
    (folder/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    return manifest
