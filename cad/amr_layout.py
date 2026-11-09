#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AMR Project Rev 0.2 — parametric concept STEP assembly.
Requires CadQuery 2.x.  Run: python cad/amr_layout.py
Units mm. CAD coordinates: X right, Y forward, Z up.
The motor drawing, caster catalogue, LiDAR datasheet, and Orbbec published
dimensions inform bounding volumes but parts are NOT vendor-exact solids.
ONLY caster plate fixing pattern is taken directly from a catalogue;
all chassis/motor/sensor bracket drilling patterns must be validated.
"""
from pathlib import Path
import math
import cadquery as cq

# Confirmed catalog geometry is separated from concept layout geometry.
PARTS = {
    "drive_motor": {"model": "ZLLG65ASM250-4096 V2.0", "count": 2,
                    "tire_dia": 173.0, "tire_dia_tolerance": 2.0,
                    "tread_width": 44.0, "tread_width_tol": 1.0,
                    "wheel_dia_user_nominal": 170.0,
                    "single_shaft": True, "rated_voltage": 24.0,
                    "rated_rpm": 160, "rated_torque_Nm": 6.0,
                    "power_W": 350, "shaft_thread": "M16x1.5",
                    "motor_mass_kg": 3.75},
    "caster": {"model": "GL-65-ASF", "count": 4,
               "wheel_dia": 64.0, "tread_width": 23.0,
               "overall_height": 82.0, "swivel_offset": 22.0,
               "plate_width": 65.0, "plate_depth": 48.0,
               "hole_pitch_x": 54.0, "hole_pitch_y": 24.0,
               "plate_hole_dia": 7.0, "capacity_each_kg": 50.0},
    "lidar": {"model": "SICK TiM571-2050101", "count": 2,
              "width": 76.25, "depth": 74.39, "height": 101.12,
              "scan_deg": 270.0},
    "camera": {"model": "Orbbec Astra Mini Pro MX6000 A V1.2", "count": 2,
               "width": 84.9, "height": 20.0, "depth": 19.92,
               "horizontal_depth_fov_deg": 58.4},
}
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
assy = cq.Assembly(name="AMR_6W_Rev02")
part_count = 0

def add(obj, name, color):
    global part_count
    shape = obj.val() if isinstance(obj, cq.Workplane) else obj
    assy.add(shape, name=name, color=cq.Color(*color))
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
for s in (-1,1):
    add(box(70,27,5,s*164,0,119),"02_wheel_bracket_upper_"+str(s),(.29,.43,.56))
    add(box(8,90,46,s*143,0,94),"03_wheel_bracket_vertical_"+str(s),(.29,.43,.56))
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

# Accurate OD/tread-width wheels, but face/shaft body simplified.
wheel=PARTS["drive_motor"]
for side in (-1,1):
    n="LEFT" if side<0 else "RIGHT"
    xc=side*P["drive_track"]/2
    z=wheel["tire_dia"]/2
    # Annulus, +X extrusion. Existing shaft is on chassis-inboard side.
    tire=(cq.Workplane("YZ").circle(wheel["tire_dia"]/2).circle(45)
          .extrude(wheel["tread_width"])
          .translate((xc-wheel["tread_width"]/2,0,z)))
    add(tire,"30_173mm_rubber_tire_"+n,(.09,.13,.18))
    add(cylx(47,44,xc,0,z),"31_motor_hub_"+n,(.27,.48,.83))
    add(cylx(51,28,side*173,0,z),"32_motor_body_SIMPLIFIED_"+n,(.31,.41,.51))
    add(cylx(8,45,side*150,0,z),"33_M16_shaft_SIMPLIFIED_"+n,(.16,.27,.35))

# Caster actual height is 82mm. Unlike Rev01, caster wheels contact Z=0.
# A DROP/HANGER is required from chassis bottom (Z=116) to the caster plate
# top at Z=82; model a simplified hanger with footprint avoiding hole pattern.
caster=PARTS["caster"]
for x in (-P["caster_x"],P["caster_x"]):
    for y in (-P["caster_y"],P["caster_y"]):
        n=("L" if x<0 else "R")+("F" if y>0 else "B")
        # Conceptual drop spacer (real attachment and fasteners still required).
        add(box(37,45,34,x,y,99),"40_caster_DROP_hanger_"+n,(.30,.45,.58))
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
    add(box(70,98,44,x,65,156),"53_motor_driver_DUMMY_"+str(x),(.28,.38,.52))

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
    add(cylz(22,3,0,yy,413),
        "63_SICK_scanner_top_"+n,(.20,.80,.79))
# Orbbec Astra Mini Pro (MX6000) x2; ±23° diagonal front.
cam=PARTS["camera"]
for side in (-1,1):
    n="LEFT" if side<0 else "RIGHT"
    x=side*P["camera_x"]; y=P["camera_y"]; z=P["camera_z"]
    angle=side*P["camera_yaw_deg"]
    body=box(cam["width"],cam["depth"],cam["height"],0,0,0)
    body=body.rotate((0,0,0),(0,0,1),angle).translate((x,y,z))
    add(body,"70_Orbbec_Astra_Mini_Pro_MX6000_"+n,(.65,.50,.82))
    add(box(28,26,18,x,y-6,318),"71_camera_bracket_PLACEHOLDER_"+n,(.67,.72,.80))

outfile=OUT/"amr_6w_layout_rev02.step"
assy.save(str(outfile))
print("Generated:",outfile,"/ components:",part_count)
print("WARNING: dimensionally guided concept only; perform collision/fastener/stability review before fabrication.")
