# AMR PROJECT — 6-Wheel Differential Mobile Base

University / educational AMR, minimal cash expenditure. **Revision 0.1: dimensional concept only; NOT a released drawing.**

## Live 3D viewer

After enabling GitHub Pages in **Settings → Pages → Deploy from a branch → main / (root)**:

- **Viewer:** https://hsh3167.github.io/amr_project/
- Launch file: [index.html](index.html)
- Orbit, zoom, pan, Top/Front/isometric views, exploded view, visibility toggles
- Switch camera configuration between 1 front-facing camera or 2 diagonal cameras
- Click a part for an explanation, export PNG screenshot, preview visual forward/turn movement
- **IMPORTANT:** The web animation is only a kinematic illustration. It is NOT a structural simulation, traction/vehicle-dynamics model, robot-arm simulation, sensor FOV verification or ROS2/Nav2 SLAM simulator.
- Browser needs internet to load Three.js from jsDelivr.

## Design requirements and assumptions

All lengths in **mm**.

| Item | Revision 0.1 | Status |
|---|---|---|
| Drive | 2 × 24 V, brake-less in-wheel motors, Ø169 | **Given** |
| Support | 4 × swivel caster; 2 front, 2 rear | **Given** |
| Structure | Aluminum plates + extrusions | **Given** |
| LiDAR | 1, front | **Given** |
| Cameras | 1 front by default; option for 2 diagonal front cameras | **Given** |
| Top deck | Removable, future manipulator expansion | **Given** |
| Nominal envelope | W≈470 × L≈590 × H≈360, incl. sensors | *Assumption* |
| Top plate | 420 × 550 × t4 | *Assumption* |
| Base plate | 390 × 530 × t4, with central side wheel cutouts | *Assumption* |
| Four corner posts | 20 × 20 extrusions, approx. 183 long | *Assumption* |
| Drive wheel track | 422 (center-to-center), wheel width 40 | *Assumption* |
| Drive axle height | 84.5 above floor (Ø169/2) | Geometric |
| Swivel casters | Ø70, offset 14; positions X=±138, Y=±215 | *Assumption* |
| Battery CAD envelope | 235 × 155 × 95 | *Placeholder only* |
| Center robot-arm bolt pattern | 124 × 124 with Ø8.4 holes | *Placeholder only* |

Coordinates: **X=width left/right; Y=forward/backward; Z=height** in CAD. In the interactive web viewer, Three.js uses **X=width, Z=forward, Y=height**.

### Mechanical decisions

1. Build a flat **lower plate with two side wheel wells** and a removable **upper plate** connected by **four standard 20×20 extrusions**, minimizing fabrication operations.
2. Mount two Ø169 hub wheels at the center of the robot's longitudinal axis. Rear/front casters are mounted symmetrically at the four corners.
3. Place the battery low near the center to avoid a high center of gravity; reserve a central upper-deck mounting region for a future arm.
4. Locate the control computer and motor drivers for access from the side while keeping cables within the chassis.
5. Design the motor brackets as **replaceable subcomponents**: hole positions, axle mount geometry, and shock isolation are unknown today.
6. Add a way to **shim/adjust caster height or hub-wheel contact loading** after prototype testing. Rigid 6-wheel assemblies can unload the central drive wheels on uneven floors.
7. Start tests on flat indoor floors. The motors do **not** have holding brakes; do not operate on ramps, and review emergency stopping and possible regenerative bus overvoltage with the electronics lead.

## Editable CAD: CadQuery → STEP

[**CAD generator source:** cad/amr_layout.py](cad/amr_layout.py)

The repository includes a **parametric CAD script** that exports a real multi-part STEP assembly. The visible web layout is a visual concept and is **not** automatically generated from the STEP file. To generate STEP locally:

~~~bash
pip install cadquery
python cad/amr_layout.py
~~~

Output: **cad-output/amr_6w_layout_rev01.step**.

You can import the STEP into SolidWorks or FreeCAD. Dimensions can be changed at the top of the Python script under the P dictionary. **Replace all dummy motor/caster geometry with accurate vendor CAD before making fabrication drawings.** The STEP file is not proof of fit, strength, or manufacturability.

## Battery decision — low-cost first

**Priority 1: Reuse the approved company 7S9P pack, but only after checking the exact chemistry, BMS, charger, power budget, state of health and dimensions.**

The topology \`7S9P\` alone does not define actual Ah, peak/continuous discharge current or physical size.

| Candidate | Nominal / full voltage | Cost reference* | Comment |
|---|---|---:|---|
| Reuse company 7S9P | Depends on chemistry; 7S Li-ion commonly 25.2–25.9V nominal / **29.4V fully charged** | 0 KRW if authorized | First choice *if compatible and healthy* |
| Novavolt KC 7S2P Li-ion 10Ah | 24V-class / 29.4V charged | 191,000 KRW | New-pack backup, check BMS current |
| Novavolt 8S2P LiFePO4 12Ah | 25.6V nominal / **29.2V fully charged** | 207,000 KRW | Robust option, but heavier per Wh; check output rating |
| Novavolt 8S1P LiFePO4 6Ah | 25.6V nominal / 29.2V charged | 129,000 KRW | Cheapest small-pack option, check motor acceleration-current requirement |
| Powercraft LiFePO4 8S2P 6.6Ah | 25.6V nominal / 29.2V charged | 162,000 KRW | 35A BMS listed; continuous pack current must be verified |

\*Website list prices observed on **2026-11-04**; stock, BMS current, charger and shipping costs can change. This is not an electrical signoff.

Sources:
- https://shop.novavolt.co.kr/kc_ncm
- https://shop.novavolt.co.kr/lfp_industrial
- https://m.powercraft.co.kr/goods/goods_list.php?cateCd=119
- https://powercraft.co.kr/goods/goods_view.php?goodsNo=1961

### Critical pack integration checks

- Confirm **cell chemistry** and **charger**. Never assume 7S means 29.4V unless it is 7S Li-ion (NMC/NCA etc). An 8S LiFePO4 charger typically targets 29.2V; the two charger profiles are **not interchangeable by assumption**.
- Check driver input-voltage limit against **maximum charge voltage, regen voltage spikes, undervoltage threshold**. A motor labeled "24V" does not prove the controller accepts 29.4V.
- Verify BMS continuous discharge current exceeds the **combined load** including both motor acceleration/stall limits, SBC, sensors and DC/DC converter, plus design margin.
- Verify pack age, swelling, drop damage, balancing, cell voltages and health using approved qualified procedures. Do not dismantle/rebuild a used battery pack.
- Add an **appropriately rated fuse near battery +**, protected wiring, a DC-rated power disconnect / motor power E-stop scheme, DC/DC rails, isolated low-voltage logic as needed, and charger connector safeguards.
- Brake-less wheels may drift on slopes; motor-controller stopping is not equivalent to a holding brake.
- Before arm installation, recalculate stability, braking, load on casters and motor wheel traction.

### Approximate capacity math, only if 7S9P Li-ion

At assumed 3.6V nominal **per cell** and 3.0Ah **per cell**, pack is approximately **25.2V × 27Ah = 680.4Wh**. At 2.5Ah/cell, total is 22.5Ah and 567Wh; at 4.0Ah/cell it is 36Ah and 907Wh. Actual cell specs and pack condition override the math.

## BOM and work planning

See [bom/concept_bom.csv](bom/concept_bom.csv). Costs are deliberately marked **TBD** until motor/controller specifications and vendor drawings are collected.

### Before fabrication

- [ ] Company owner has authorized component reuse and external student project use
- [ ] Capture the *actual wheel motor* 3D CAD, motor flange holes, encoder signals, power and drive specs
- [ ] Capture the actual caster drawings: wheel Ø, swivel offset, mounting holes, overall height, loading capacity
- [ ] Capture actual battery label, chemistry, dimensions, mass, charger and BMS current/communications
- [ ] Agree on payload, future robot-arm mass/torque, max velocity and floor type
- [ ] Confirm actual motor controllers, SBC, DC/DC, LiDAR and camera footprints
- [ ] Verify wheel/caster contact loading, center of gravity, motor mount stiffness and E-stop
- [ ] Replace dummy STEP solids and generate drawings and real BOM

### Proposed project phases

- **Phase 0**: layout review + verify reused parts
- **Phase 1**: chassis, power distribution, manual teleoperation
- **Phase 2**: odometry + LiDAR SLAM + Nav2 autonomous motion
- **Phase 3**: camera-based localization/object detection
- **Phase 4**: optional robot-arm mounting and manipulation after stability review

License: project-created code and concept material may be reused for educational purposes; external part geometry and vendor documentation remain subject to their own licenses.
