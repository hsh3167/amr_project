from dataclasses import dataclass, field
from build123d import Solid, Plane, Face, Wire, Vector, Axis, Color

@dataclass
class PartRecord:
    name: str
    group: str
    shape: object
    color: tuple = (.6,.7,.8)
    status: str = "concept"
    metadata: dict = field(default_factory=dict)

    def __post_init__(self):
        self.shape.label=self.name
        self.shape.color=Color(*self.color)

def box(w,d,h,x=0,y=0,z=0):
    return Solid.make_box(w,d,h).translate((x-w/2,y-d/2,z-h/2))

def cylx(r,length,x=0,y=0,z=0):
    return Solid.make_cylinder(r,length,Plane(origin=(x-length/2,y,z),z_dir=(1,0,0)))

def cylz(r,height,x=0,y=0,bottom=0):
    return Solid.make_cylinder(r,height).translate((x,y,bottom))

def polygon_plate(points,thickness,bottom):
    face=Face(Wire.make_polygon([Vector(x,y,0) for x,y in points],close=True))
    return Solid.extrude(face,(0,0,thickness)).translate((0,0,bottom))

def rz(shape,angle): return shape.rotate(Axis.Z,angle)

def drilled(shape,points,diameter,bottom,height):
    for x,y in points: shape=shape-cylz(diameter/2,height+2,x,y,bottom-1)
    return shape
