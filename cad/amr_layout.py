#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AMR Rev05 dimension-guided layout, mm; X right, Y forward, Z up.
Motor axial dimensions: supplied V2.02 PDF p.7. Undimensioned contours
and all chassis brackets remain assumptions; see docs/motor-datasheet-review.md.
"""
from pathlib import Path
import math
import cadquery as cq

import json
PARTS = json.loads((Path(__file__).with_name("parts.json")).read_text(encoding="utf-8"))
LAYOUT = json.loads(Path(__file__).with_name("layout.json").read_text(encoding="utf-8"))
P = LAYOUT["parameters"]
PROFILE = LAYOUT["profile"]
profile_parts = []
OUT = Path(__file__).resolve().parent.parent / "cad-output"
OUT.mkdir(exist_ok=True, parents=True)
assy = cq.Assembly(name="AMR_6W_Rev05")
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

def extrusion20(x,y,bottom,top,name):
    """Generic visual T-slot, not a purchased vendor's manufacturing profile."""
    w=PROFILE["nominal_section"];h=top-bottom
    profile=cq.Workplane("XY").rect(w,w).circle(PROFILE["assumed_core_dia"]/2).extrude(h)
    for angle in (0,90,180,270):
        mouth=box(PROFILE["assumed_slot_mouth"],2,h+2,0,w/2-1,h/2)
        undercut=box(PROFILE["assumed_slot_inner_width"],2.5,h+2,0,w/2-3.25,h/2)
        for cutter in (mouth,undercut):
            profile=profile.cut(cutter.rotate((0,0,0),(0,0,1),angle))
    shape=profile.translate((x,y,bottom))
    add(shape,name,(.67,.77,.84))
    profile_parts.append({"name":name,"material":PROFILE["material"],"section_mm":w,"length_mm":h})

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
for x,y in PROFILE["main_columns_xy"]:
    n=("L" if x<0 else "R")+("F" if y>0 else "B")
    extrusion20(x,y,PROFILE["main_bottom_z"],PROFILE["main_top_z"],"10_2020_Al_column_"+n)
    add(box(32,32,4,x,y,121),"11_corner_lower_"+n,(.33,.49,.63))
    add(box(32,32,4,x,y,301),"12_corner_upper_"+n,(.33,.49,.63))
# Upper plate: M4/M8 prototype holes are NOT qualified for an arm.
top=box(P["top_width"],P["top_length"],P["top_thickness"],0,0,305)
top=top.faces(">Z").workplane().pushPoints(PROFILE["main_columns_xy"]).hole(5.5)
arm_holes=[(x,y) for x in (-62,62) for y in (-62,62)]
top=top.faces(">Z").workplane().pushPoints(arm_holes).hole(8.4)
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
E=LAYOUT["electronics"]
add(box(165,104,7,0,E["logic_y"],E["pcb_center_z"]),"51_controller_PCB_DUMMY",(.12,.46,.37))
add(box(146,82,23,0,E["logic_y"],E["computer_center_z"]),"52_control_computer_DUMMY",(.13,.19,.28))
for x in (-E["driver_x"],E["driver_x"]):
    add(box(70,98,44,x,E["driver_y"],E["driver_center_z"]),"53_motor_driver_DUMMY_"+str(x),(.28,.38,.52))

# Rear power service bay is an envelope reservation, not a selected product.
add(box(160,65,40,0,E["power_bay_y"],E["power_bay_center_z"]),
    "54_REAR_power_distribution_DUMMY",(.65,.39,.24))

# Two diagonal outboard scanners. The270deg apertures face out;
# their90deg blind quadrants point into the chassis at the same scan plane.
lidar=PARTS["lidar"];mount=LAYOUT["lidar_mount"]
for pose in LAYOUT["lidars"]:
    n=pose["name"];x=pose["x"];y=pose["y"];z0=pose["bbox_bottom_z"]
    yaw=-pose["heading_deg"];pt=mount["plate_thickness"]
    def placed(obj):
        return obj.rotate((0,0,0),(0,0,1),yaw).translate((x,y,0))
    sensorplate=placed(box(mount["plate_width"],mount["plate_depth"],pt,0,0,z0-pt/2))
    sx=pose["support_x"];sy=pose["support_y"]
    length=math.hypot(x-sx,y-sy)+25
    bridge=box(length,25,pt,0,0,z0-pt/2)
    bridge=bridge.rotate((0,0,0),(0,0,1),math.degrees(math.atan2(y-sy,x-sx)))
    bridge=bridge.translate(((x+sx)/2,(y+sy)/2,0))
    add(sensorplate.union(bridge),"60_diagonal_lidar_mount_CONCEPT_"+n,(.28,.43,.54))
    extrusion20(sx,sy,P["base_bottom_z"]+P["base_thickness"],z0-pt,"64_2020_Al_lidar_column_"+n)
    add(placed(box(lidar["width"],lidar["depth"],36,0,0,z0+18)),
        "61_SICK_TiM571_envelope_base_"+n,(.85,.88,.90))
    add(placed(box(61,60,lidar["height"]-36,0,0,z0+36+(lidar["height"]-36)/2)),
        "62_SICK_TiM571_optical_envelope_"+n,(.09,.16,.21))
    add(cylz(22,3,x,y,z0+lidar["height"]-3),"63_SICK_scanner_top_"+n,(.20,.80,.79))

# Cameras hang from upper deck using Al profiles. All supports stay above
# the nominal241mm scan plane to avoid two new laser occluders.
cam=PARTS["camera"];cm=LAYOUT["camera_mount"]
for side in (-1,1):
    n="LEFT" if side<0 else "RIGHT"
    x=side*P["camera_x"];y=P["camera_y"];z=P["camera_z"]
    body=box(cam["width"],cam["depth"],cam["height"],0,0,0)
    body=body.rotate((0,0,0),(1,0,0),-P["camera_pitch_down_deg"])
    body=body.rotate((0,0,0),(0,0,1),-side*P["camera_yaw_deg"]).translate((x,y,z))
    add(body,"70_Orbbec_Astra_Mini_Pro_MX6000_"+n,(.65,.50,.82))
    add(box(cm["plate_width"],cm["plate_depth"],cm["plate_thickness"],x,cm["plate_y"],cm["plate_center_z"]),
        "71_camera_adapter_CONCEPT_"+n,(.67,.72,.80))
    extrusion20(x,cm["profile_y"],cm["profile_bottom_z"],cm["profile_top_z"],"72_2020_Al_camera_column_"+n)

outfile=OUT/"amr_6w_layout_rev05.step"
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
# Geometric FOV sampling at the actual layout's nominal scan plane.
import sys
sys.path.insert(0,str(Path(__file__).parent))
from validate_layout import coverage
scan_check=coverage(LAYOUT)
assert all(not row["missed"] for row in scan_check["perimeters"])
assert len(profile_parts)==8
actual_counts={"drive_motor":sum(n.startswith("30_") for n in components),
               "caster":sum(n.startswith("41_") for n in components),
               "lidar":sum(n.startswith("61_") for n in components),
               "camera":sum(n.startswith("70_") for n in components)}
assert all(actual_counts[k]==PARTS[k]["count"] for k in actual_counts)
scan_z=LAYOUT["lidars"][0]["scan_z"]
assert all(abs(pose["scan_z"]-scan_z)<1e-6 for pose in LAYOUT["lidars"])
below=("30_","31_","32_","50_","51_","52_","53_","54_","64_")
above=("20_","70_","71_","72_")
assert all(shape.BoundingBox().zmax<scan_z for n,shape in components.items() if n.startswith(below))
assert all(shape.BoundingBox().zmin>scan_z for n,shape in components.items() if n.startswith(above))
collision_pairs=[]
for n,shape in components.items():
    if n.startswith(("61_","62_")):
        for k,obstacle in components.items():
            if k.startswith(("10_","20_","50_","51_","52_","53_","70_")):
                volume=shape.intersect(obstacle).Volume()
                if volume>0.01:collision_pairs.append((n,k,volume))
    if n.startswith("70_"):
        for k,obstacle in components.items():
            if k.startswith(("10_","20_")):
                volume=shape.intersect(obstacle).Volume()
                if volume>0.01:collision_pairs.append((n,k,volume))
assert not collision_pairs,collision_pairs
manifest={"revision":LAYOUT["revision"],"components":part_count,"solid_count":len(solids),
          "valid_solids":True,"step_bytes":outfile.stat().st_size,
          "bounds_mm":{"x":[bb.xmin,bb.xmax],"y":[bb.ymin,bb.ymax],"z":[bb.zmin,bb.zmax]},
          "motor_u_mm":{"outer_face":outer,"body_face":body_end,"shaft_end":shaft_end},
          "ground_mm":ground,
          "caster_drop_mm":P["base_bottom_z"]-caster["overall_height"],
          "aluminum_profile_columns":profile_parts,"lidar_coverage_sampling":scan_check,
          "selected_sensor_collisions":collision_pairs,"scan_plane_vertical_clearance_checked":True,
          "counts":actual_counts,
          "scope":"Solid validity, file roundtrip, nominal contact, dimensions only; no structural/load/dynamics qualification."}
(OUT/"validation_rev05.json").write_text(json.dumps(manifest,indent=2)+"\n",encoding="utf-8")
print(json.dumps(manifest,indent=2))
