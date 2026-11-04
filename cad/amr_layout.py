#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AMR Project / university differential 6-wheel base / conceptual parametric CAD.
Generates a STEP assembly with CadQuery 2.x.
Run: python cad/amr_layout.py  (requires pip install cadquery)
Axes: X = width, Y = forward, Z = height. Units: mm.
NOTE: Motor/caster footprints and ALL drilling patterns are PLACEHOLDERS.
Not validated for structural safety, clearances, battery sizing, or fabrication.
"""
from pathlib import Path
import cadquery as cq

# ---------------- Editable concept parameters (mm) ----------------
P = {
    "base_width": 390.0,
    "base_length": 530.0,
    "base_z": 116.0,
    "base_thickness": 4.0,
    "top_width": 420.0,
    "top_length": 550.0,
    "top_z": 303.0,
    "top_thickness": 4.0,
    "extrusion": 20.0,
    "wheel_dia": 169.0,         # GIVEN by project; tire width still a guess
    "wheel_width": 40.0,        # ASSUMPTION
    "wheel_track": 422.0,      # ASSUMPTION, wheel center-to-center
    "motor_body_dia": 100.0,   # ASSUMPTION
    "caster_dia": 70.0,        # ASSUMPTION
    "caster_offset": 14.0,     # ASSUMPTION
    "caster_x": 138.0,         # ASSUMPTION
    "caster_y": 215.0,         # ASSUMPTION
    "battery_width": 235.0,    # ASSUMPTION; replace with measured 7S9P pack
    "battery_length": 155.0,   # ASSUMPTION
    "battery_height": 95.0,    # ASSUMPTION
    "battery_y": -52.0,
    "battery_center_z": 173.5,
    "arm_bolt_pitch": 124.0,   # PLACEHOLDER, NOT a real arm interface
}
OUT = Path(__file__).resolve().parent.parent / "cad-output"
OUT.mkdir(exist_ok=True, parents=True)
assy = cq.Assembly(name="AMR_6W_Concept_Rev01")
all_parts = []


def add(shape, label, color):
    solid = shape.val() if isinstance(shape, cq.Workplane) else shape
    assy.add(solid, name=label, color=cq.Color(*color))
    all_parts.append(solid)


def centered_box(x, y, z, cx, cy, cz):
    return cq.Workplane("XY").box(x, y, z).translate((cx, cy, cz))


def x_axis_cylinder(radius, length, xcenter, ycenter, zcenter):
    return (cq.Workplane("YZ").circle(radius).extrude(length)
            .translate((xcenter - length / 2, ycenter, zcenter)))


# Base plate: wheel-well cutouts on both sides keep the 169 mm wheels clear.
base_outline = [
    (-195,-265), (195,-265), (195,-99), (145,-99),
    (145,99), (195,99), (195,265), (-195,265),
    (-195,99), (-145,99), (-145,-99), (-195,-99)
]
base = (cq.Workplane("XY").polyline(base_outline).close()
        .extrude(P["base_thickness"]).translate((0,0,P["base_z"])))
add(base, "01_bottom_plate_NOTCHED", (0.65,0.71,0.77))
for side in (-1, 1):
    add(centered_box(70, 27, 5, side*164,0,119),
        "02_motor_bracket_top_" + str(side), (.25,.38,.54))
    add(centered_box(8,90,46,side*143,0,94),
        "03_motor_bracket_vertical_" + str(side), (.25,.38,.54))
for y in (-215, 215):
    add(centered_box(320,18,12,0,y,111),
        "04_base_cross_member_" + str(y), (.61,.68,.75))

# 20×20 vertical corner posts and corner plate fasteners.
for x in (-146, 146):
    for y in (-208, 208):
        label = ("L" if x < 0 else "R") + ("F" if y > 0 else "B")
        add(centered_box(20,20,183,x,y,211.5),
            "10_extrusion_" + label, (.68,.76,.84))
        add(centered_box(32,32,4,x,y,121),
            "11_lower_corner_" + label, (.30,.45,.62))
        add(centered_box(32,32,4,x,y,301),
            "12_upper_corner_" + label, (.30,.45,.62))

# Top plate: drilling pattern is explicitly a placeholder.
top = centered_box(P["top_width"],P["top_length"],P["top_thickness"],0,0,P["top_z"] + 2)
top = top.faces(">Z").workplane()
hole_points = [(x,y) for x in (-146,146) for y in (-208,208)]
hole_points += [(x,y) for x in (-62,62) for y in (-62,62)]
top = top.pushPoints(hole_points).hole(8.4)
add(top,"20_top_plate_8mm_HOLES_PLACEHOLDER",(.72,.81,.89))
add(centered_box(155,155,5,0,0,299),
    "21_arm_reinforcement_PLACEHOLDER",(.26,.40,.56))

# In-wheel drive motors: motor housing and wheel shapes are estimated.
for side in (-1,1):
    xcenter = side * P["wheel_track"]/2
    prefix = "LEFT" if side < 0 else "RIGHT"
    tire = (cq.Workplane("YZ")
            .circle(P["wheel_dia"]/2).circle(45).extrude(P["wheel_width"])
            .translate((xcenter - P["wheel_width"]/2, 0, P["wheel_dia"]/2)))
    add(tire,"30_tire_" + prefix,(.09,.13,.19))
    add(x_axis_cylinder(44,42,xcenter,0,84.5),
        "31_hub_" + prefix,(.27,.49,.88))
    add(x_axis_cylinder(P["motor_body_dia"]/2,31,side*174,0,84.5),
        "32_motor_housing_ESTIMATED_" + prefix,(.27,.38,.50))
    add(x_axis_cylinder(15,16,side*155,0,84.5),
        "33_motor_mount_ESTIMATED_" + prefix,(.13,.22,.34))

# Four swivel casters. Wheel offset and fork details are placeholders.
for x in (-P["caster_x"],P["caster_x"]):
    for y in (-P["caster_y"],P["caster_y"]):
        n = ("L" if x < 0 else "R") + ("F" if y > 0 else "B")
        add(centered_box(76,75,5,x,y,113),
            "40_caster_mount_" + n,(.70,.77,.83))
        add(cq.Workplane("XY").center(x,y).circle(12).extrude(24).translate((0,0,83)),
            "41_caster_swivel_" + n,(.26,.36,.49))
        for leg in (-17,17):
            add(centered_box(5,23,45,x+leg,y-14,68),
                "42_caster_fork_" + n + "_" + str(leg),(.66,.75,.82))
        add(x_axis_cylinder(P["caster_dia"]/2,26,x,y-P["caster_offset"],35),
            "43_caster_wheel_" + n,(.47,.58,.67))

# Pack and controllers are clearance-envelope dummies, not measured parts.
add(centered_box(P["battery_width"],P["battery_length"],P["battery_height"],
                 0,P["battery_y"],P["battery_center_z"]),
    "50_BATTERY_7S9P_ENVELOPE_ONLY",(.86,.63,.36))
add(centered_box(165,104,7,0,122,146),
    "51_controller_PCB_ENVELOPE_ONLY",(.12,.46,.37))
add(centered_box(146,82,23,0,122,161),
    "52_controller_computer_ENVELOPE_ONLY",(.13,.19,.28))
for x in (-124,124):
    add(centered_box(70,98,44,x,65,156),
        "53_driver_ENVELOPE_" + str(x),(.28,.38,.52))

# Front lidar and camera.
add(centered_box(94,68,6,0,223,313),
    "60_lidar_bracket_ESTIMATED",(.29,.40,.54))
add(cq.Workplane("XY").center(0,223).circle(30).extrude(17).translate((0,0,318)),
    "61_lidar_dummy",(.20,.78,.76))
add(cq.Workplane("XY").center(0,223).circle(32).extrude(8).translate((0,0,335)),
    "62_lidar_top",(.20,.85,.82))
add(centered_box(22,38,21,100,278,331),
    "70_front_camera_dummy",(.68,.53,.85))
add(centered_box(11,11,21,100,275,313),
    "71_front_camera_bracket",(.65,.72,.80))

step_path = OUT / "amr_6w_layout_rev01.step"
assy.save(str(step_path))
print("Saved STEP assembly:", step_path)
print("Component count:", len(all_parts))
print("WARNING: dummy motor, caster, battery, mount geometry; not production-ready.")
