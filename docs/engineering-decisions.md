# AMR engineering decisions — Rev 0.1 (2026-11-04)

## CAD software decision
Recommendation: **FreeCAD 1.1 for production-ready part/assembly CAD and drawings**; keep **CadQuery** as the repeatable initial STEP generator; use **SolidCraft 0.30.1** experimentally for simple AI-assisted models. The GitHub Pages Three.js viewer is for team reviews only, not parametric editing or dynamics validation.

| Task | Tool | Reason |
|---|---|---|
| Team review in browser | Three.js | no installation / interactive part selection |
| Revision-0.1 STEP model | CadQuery | reproducible scripted build via GitHub Actions |
| Real design with accurate vendor CAD | FreeCAD 1.1 | Assembly, TechDraw, STEP and FEM/CAM ecosystem |
| AI CAD experiments | SolidCraft stable v0.30.1 | built-in MCP, CLI, parametric browser and desktop editors |

FreeCAD 1.1 (March 25, 2026) provides Assembly animation/BOM and TechDraw improvements. SolidCraft has assemblies, sketch/parametric modelling and experimental sheet-metal support, but documentation explicitly defers production drawings, CAM and simulation. The 2026-11-03 SolidCraft 0.40.0-rc.2 is a prerelease with a new kernel and incompatible file-format changes vs stable v0.30.1. Do not use an RC as the sole project master. STEP is good for exchange, but importing STEP does not preserve full native parametric history.

Sources: https://blog.freecad.org/2026/03/25/freecad-version-1-1-released/ ; https://freecad.github.io/Website/download/releases/1-1/ ; https://docs.solidcraft.io/ ; https://github.com/tangency-labs/solidcraft/releases

## Battery selection
**First choice:** reuse company-approved and verified 7S9P battery if chemistry, dimensions, charger, battery health, BMS current and 24V drive input range are suitable. 7S9P only specifies 7 series groups × 9 parallel cells; it does not specify energy or safe discharge current.

For ordinary 3.6V nominal Li-ion cells, a 7S9P pack is 25.2V nominal, up to 29.4V at full charge. With 3.0Ah cells assumed, theoretical capacity is 27Ah and nominal energy is about 680Wh. Other chemistries have different voltage ranges.

| New-pack comparison (web listed 2026-11-04) | Price |
|---|---:|
| Novavolt KC 24V 10Ah Li-ion 7S2P | ₩191,000 |
| Novavolt industrial 24V 12Ah LiFePO4 8S2P | ₩207,000 |
| Novavolt industrial 24V 6Ah LiFePO4 8S1P | ₩129,000 |

Prices exclude charger, wiring, shipping and pack integration. Sources: https://shop.novavolt.co.kr/kc_ncm ; https://shop.novavolt.co.kr/lfp_industrial

**Mandatory checks:** motor-driver maximum DC bus voltage including regenerative spikes; combined motor startup/stall current; BMS continuous and peak current; charger chemistry; pack state of health; fuse rated for DC; cable/connector ratings; safe power disconnect and E-stop. Do not disassemble/rebuild used packs. For early testing keep to flat ground because the motors have no brakes.

## Mechanical design considerations
Central Ø169mm 2WD plus four fixed-height swivel casters can lose traction on slightly uneven ground. Allow adjustment by caster shims/mounting slots and verify normal force distribution. Keep battery low and central. Future arm installation requires new center-of-gravity, tipping and top-deck bending calculations.

## Unknowns before release for fabrication
1. Actual wheel motor flange and mounting holes, wheel width, encoder and matching driver.
2. Actual caster diameter, swivel offset, assembly height, mount hole pattern and load rating.
3. Physical 7S9P pack dimensions and BMS documentation.
4. LiDAR and camera models, optical field of view and mounting envelopes.
5. Required payload, vehicle target mass, speed and future arm reach/loads.