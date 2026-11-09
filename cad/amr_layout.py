#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AMR Rev04 dimension-guided layout, mm; X right, Y forward, Z up.
Motor axial dimensions: supplied V2.02 PDF p.7. Undimensioned contours
and all chassis brackets remain assumptions; see docs/motor-datasheet-review.md.
"""
from pathlib import Path
import math
import cadquery as cq

import json
PARTS = json.loads((Path(__file__).with_name("parts.json")).read_text(encoding="utf-8"))
P = {
    "base_width": 390.0,  # concept, not a manufacturer dimension
    "base_length": 530.0,
    "base_bottom_z": 116.0,  # preserves battery clearance under top deck
    "base_thickness": 4.0,
    "top_width": 420.0,
    "top_length": 550.0,
    "top_bottom_z": 303.0,
    "top_thickness": 4.0,
    "extrusion": 20.0,
    "drive_track": 422.0,  # concept center-to-center
    "caster_x": 138.0,
    "caster_y": 215.0,
    "battery_width": 235.0, "battery_length": 155.0,
    "battery_height": 95.0,
    "battery_y": -52.0, "battery_center_z": 173.5,
    "arm_bolt_pitch": 124.0,  # placeholder only
    "camera_x": 120.0, "camera_y": 266.0,
    "camera_z": 335.0, "camera_yaw_deg": 23.0,
    "lidar_y": 230.0, "lidar_bottom_z": 312.0,
}
OUT = Path(__file__).resolve().parent.parent / "cad-output"
OUT.mkdir(exist_ok=True, parents=True)
assy = cq.Assembly(name="AMR_6W_Rev04")
part_count = 0
components = {}

def add(obj, name, color):
    global part_count
    shape = obj.val() if isinstance(obj, cq.Workplane) else obj
    assy.add(shape, name=name, color=cq.Color(*color))
    components[name] = shape
    part_count += 1

def box(w, d, h, x, y, z):
    return cq.Workplane("XY").box(w, d, h).translate((x,y,z))

def cylx(r, length, x, y, z):
    return cq.Workplane("YZ").circle(r).extrude(length).translate((x-length/2,y,z))

def cylz(r, height, x, y, zbottom):
    return cq.Workplane("XY").center(x,y).circle(r).extrude(height).translate((0,0,zbottom))

# Lower plate includes side slots for central wheel clearance.
outline = [(-195,-265),(195,-265),(195,-99),(145,-99),
           (145,99),(195,99),(195,265),(-195,265),
           (-195,99),(-145,99),(-145,-99),(-195,-99)]
base = (cq.Workplane("XY").polyline(outline).close()
        .extrude(P["base_thickness"]).translate((0,0,P["base_bottom_z"])))
add(base,"01_lower_Al_plate_CONCEPT",(.60,.71,.78))
# Concept steel L bracket, 6mm; actual clamp, washer/nut stack and strength TBD.
# The axle flats, not the six undimensioned cover screws, react motor torque.
for side in (-1,1):
    bx=side*(P["drive_track"]/2-41)
    bracket=box(6,90,70,bx,0,90)
    slot=cylx(7.15,8,bx,0,90).intersect(box(10,10.2,20,bx,0,90))
    add(bracket.cut(slot),"02_axle_flat_bracket_CONCEPT_"+str(side),(.29,.43,.56))
    add(box(43,90,6,side*151.5,0,123),"03_bracket_bridge_CONCEPT_"+str(side),(.29,.43,.56))
for y in (-215,215):
    add(box(320,18,12,0,y,111),"04_base_crossmember_"+str(y),(.61,.70,.79))
for x in (-146,146):
    for y in (-208,208):
        n=("L" if x<0 else "R")+("F" if y>0 else "B")
        add(box(20,20,183,x,y,211.5),"10_2020_extrusion_"+n,(.67,.77,.84))
        add(box(32,32,4,x,y,121),"11_corner_lower_"+n,(.33,.49,.63))
        add(box(32,32,4,x,y,301),"12_corner_upper_"+n,(.33,.49,.63))
# Upper plate: M4/M8 prototype holes are NOT qualified for an arm.
top=box(P["top_width"],P["top_length"],P["top_thickness"],0,0,305)
hole_pts=[(x,y) for x in (-146,146) for y in (-208,208)]
hole_pts += [(x,y) for x in (-62,62) for y in (-62,62)]
top=top.faces(">Z").workplane().pushPoints(hole_pts).hole(8.4)
add(top,"20_upper_Al_plate_PLACEHOLDER_HOLES",(.72,.81,.89))
add(box(155,155,5,0,0,299),"21_arm_reinforcement_PLACEHOLDER",(.25,.40,.55))

# Motor coordinates u: tire midplane=0, positive inward toward shaft.
# p.7 chain: outer face=-31.1, body inner face=33.1, thread start=35.2,
# shaft end=58.7. Total metal envelope=89.8; cable tail excluded.
wheel=PARTS["drive_motor"]
outer=-wheel["tread_width"]/2-wheel["outer_face_to_tire"]
shaft_end=outer+wheel["overall_axial"]
body_end=shaft_end-wheel["body_face_to_shaft_end"]
for side in (-1,1):
    n="LEFT" if side<0 else "RIGHT"
    xc=side*P["drive_track"]/2; z=wheel["tire_dia"]/2
    tire=(cq.Workplane("YZ").circle(wheel["tire_dia"]/2)
          .circle(wheel["assumed_body_dia"]/2).extrude(wheel["tread_width"])
          .translate((xc-wheel["tread_width"]/2,0,z)))
    add(tire,"30_NBR_OD180_W42p5_"+n,(.09,.13,.18))
    # OD140 casing is explicitly ASSUMED; axial limits are drawing-derived.
    casing=cylx(wheel["assumed_body_dia"]/2,body_end-outer,
                xc-side*(outer+body_end)/2,0,z)
    add(casing,"31_casing_OD140_ASSUMED_axial_DATASHEET_"+n,(.27,.48,.83))
    axle=cylx(wheel["shaft_dia"]/2,shaft_end-body_end,
              xc-side*(body_end+shaft_end)/2,0,z)
    axle=axle.intersect(box(100,wheel["shaft_flat"],30,xc-side*45,0,z))
    add(axle,"32_D14_AF10_M14x1p5_THREAD_BLANK_"+n,(.16,.27,.35))

# Caster actual height is 82mm. Unlike Rev01, caster wheels contact Z=0.
# A DROP/HANGER is required from chassis bottom (Z=116) to the caster plate
# top at Z=82; model a simplified hanger with footprint avoiding hole pattern.
caster=PARTS["caster"]
for x in (-P["caster_x"],P["caster_x"]):
    for y in (-P["caster_y"],P["caster_y"]):
        n=("L" if x<0 else "R")+("F" if y>0 else "B")
        # Conceptual drop spacer (real attachment and fasteners still required).
        drop=P["base_bottom_z"]-caster["overall_height"]
        for i,(dx,dy) in enumerate(((-27,-12),(-27,12),(27,-12),(27,12))):
            spacer=(cq.Workplane("XY").center(x+dx,y+dy).circle(5.5).circle(3.5)
                    .extrude(drop).translate((0,0,caster["overall_height"])))
            add(spacer,"40_caster_spacer_CONCEPT_"+n+str(i),(.30,.45,.58))
        plate=box(65,48,5,x,y,79.5)
        p=[(x+dx,y+dy) for dx in (-27,27) for dy in (-12,12)]
        plate=plate.faces(">Z").workplane().pushPoints(p).hole(7)
        add(plate,"41_GL65_plate_65x48_HOLE54x24_"+n,(.75,.80,.83))
        add(cylz(12,15,x,y,60),"42_swivel_head_"+n,(.31,.39,.46))
        for dx in (-17,17):
            add(box(4.5,32,38,x+dx,y-12,51),
                "43_fork_"+n+"_"+str(dx),(.68,.73,.79))
        add(cylx(32,23,x,y-22,32),
            "44_GL65_wheel_64OD_"+n,(.45,.54,.64))

# Battery volume is entirely provisional; 7S9P doesn't determine geometry.
add(box(P["battery_width"],P["battery_length"],P["battery_height"],
        0,P["battery_y"],P["battery_center_z"]),
    "50_company_7S9P_BATTERY_DUMMY",(.86,.63,.36))
add(box(165,104,7,0,122,146),"51_controller_PCB_DUMMY",(.12,.46,.37))
add(box(146,82,23,0,122,161),"52_control_computer_DUMMY",(.13,.19,.28))
for x in (-124,124):
    add(box(70,98,44,x,85,156),"53_motor_driver_DUMMY_"+str(x),(.28,.38,.52))

# TiM571-2050101 x2; sensor footprint from official SICK PDF, body simplified.
# Front looks toward +Y, rear toward -Y; scan line unobstructed assuming
# both are placed above the top deck and no future arm occludes them.
lidar=PARTS["lidar"]
for direction in (1,-1):
    n="FRONT" if direction==1 else "REAR"
    yy=direction*P["lidar_y"]
    add(box(94,85,5,0,yy,310),"60_lidar_mount_"+n,(.28,.43,.54))
    add(box(lidar["width"],lidar["depth"],36,0,yy,330),
        "61_SICK_TiM571_base_"+n,(.85,.88,.90))
    add(box(61,60,lidar["height"]-36,0,yy,348+(lidar["height"]-36)/2),
        "62_SICK_TiM571_optical_body_"+n,(.09,.16,.21))
    add(cylz(22,3,0,yy,410.12),
        "63_SICK_scanner_top_"+n,(.20,.80,.79))
# Orbbec Astra Mini Pro (MX6000) x2; ±23° diagonal front.
cam=PARTS["camera"]
for side in (-1,1):
    n="LEFT" if side<0 else "RIGHT"
    x=side*P["camera_x"]; y=P["camera_y"]; z=P["camera_z"]
    angle=-side*P["camera_yaw_deg"]
    body=box(cam["width"],cam["depth"],cam["height"],0,0,0)
    body=body.rotate((0,0,0),(0,0,1),angle).translate((x,y,z))
    add(body,"70_Orbbec_Astra_Mini_Pro_MX6000_"+n,(.65,.50,.82))
    add(box(28,26,18,x,y-6,318),"71_camera_bracket_PLACEHOLDER_"+n,(.67,.72,.80))

outfile=OUT/"amr_6w_layout_rev04.step"
assy.save(str(outfile))
# Reopen the exchange file, not just the in-memory assembly.
reloaded=cq.importers.importStep(str(outfile)).val()
solids=reloaded.Solids()
assert solids and all(v.isValid() and v.Volume()>0 for v in solids)
ground = {name: shape.BoundingBox().zmin for name,shape in components.items()
          if name.startswith(("30_", "44_"))}
assert len(ground)==6 and all(abs(z)<1e-6 for z in ground.values())
for n in ("LEFT","RIGHT"):
    tire_bb=components["30_NBR_OD180_W42p5_"+n].BoundingBox()
    assert abs(tire_bb.xlen-wheel["tread_width"])<1e-6
    assert abs(tire_bb.zlen-wheel["tire_dia"])<1e-6
    axle_bb=components["32_D14_AF10_M14x1p5_THREAD_BLANK_"+n].BoundingBox()
    assert abs(axle_bb.ylen-wheel["shaft_flat"])<1e-6
assert len(solids)==part_count
bb=reloaded.BoundingBox()
manifest={"revision":PARTS["revision"],"components":part_count,"solid_count":len(solids),
          "valid_solids":True,"step_bytes":outfile.stat().st_size,
          "bounds_mm":{"x":[bb.xmin,bb.xmax],"y":[bb.ymin,bb.ymax],"z":[bb.zmin,bb.zmax]},
          "motor_u_mm":{"outer_face":outer,"body_face":body_end,"shaft_end":shaft_end},
          "ground_mm":ground,
          "caster_drop_mm":P["base_bottom_z"]-caster["overall_height"],
          "counts":{k:PARTS[k]["count"] for k in ("drive_motor","caster","lidar","camera")},
          "scope":"Solid validity, file roundtrip, nominal contact, dimensions only; no structural/load/dynamics qualification."}
(OUT/"validation_rev04.json").write_text(json.dumps(manifest,indent=2)+"\n",encoding="utf-8")
print(json.dumps(manifest,indent=2))
