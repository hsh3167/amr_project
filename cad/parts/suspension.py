"""Rev11 suspension: six independent vertical modules.

Each wheel: one MISUMI wide miniature rail on a bent-sheet fixed bracket, a bent
moving carrier on the block, one X-SWU spring on an M8 guide bolt. The guide-bolt
nut is the droop stop and a loose sleeve inside the spring is the bump stop.
No welding and no reamed bores. Rev09 (twin shafts + plain bushes, welded
rockers) and Rev10 (swing arms) stay in git history. Dimensions in mm.
"""
from math import cos, pi, sin
from build123d import Axis, Circle, Compound, Helix, Plane, sweep
from ..caster_motion import CASTER_WHEELS, caster_xy, pose_shape
from .common import PartRecord, cylx, cylz, drilled, polygon_plate, span

SCREW_HEAD = {3: 5.5, 4: 7, 5: 8.5, 6: 10, 8: 13}
NUT = {5: (8, 4), 6: (10, 5), 8: (13, 6.5)}


def ring(x, y, bottom, height, od, bore):
    return cylz(od/2, height, x, y, bottom)-cylz(bore/2, height+2, x, y, bottom-1)


def screw(x, y, bearing, diameter, length, up=False):
    """Catalog screw envelope, excluding the helical thread and socket recess."""
    head_od = SCREW_HEAD[diameter]
    shank_bottom, head_bottom = (bearing, bearing-diameter) if up else (bearing-length, bearing)
    return cylz(diameter/2, length, x, y, shank_bottom)+cylz(head_od/2, diameter, x, y, head_bottom)


def nut(x, y, bottom, across_flats, height, diameter):
    radius = across_flats/(2*cos(pi/6))
    points = [(x+radius*cos(i*pi/3), y+radius*sin(i*pi/3)) for i in range(6)]
    return polygon_plate(points, height, bottom)-cylz(diameter/2, height+2, x, y, bottom-1)


def coil(x, y, bottom, height, q):
    # The round sweep shows travel and the catalog OD/ID envelope. The stock
    # spring uses shaped wire; its rate and solid length come from the catalog.
    wire = (q['spring_od_mm']-q['spring_id_mm'])/2
    path = Helix((height-wire)/q['display_turns'], height-wire, (q['spring_od_mm']-wire)/2)
    section = Plane(origin=path@0, z_dir=path % 0)*Circle(wire/2)
    return sweep(section, path=path, is_frenet=True).translate((x, y, bottom+wire/2))


def spring_state(d, kind):
    q = d.layout['suspension'][kind]
    half = q['travel_mm']/2
    installed = q['loaded_length_mm']-q['shim_mm']
    free, rate = q['free_length_mm'], q['rate_N_mm']
    lengths = [installed+half, installed, installed-half]
    return {'sku': q['spring_sku'], 'travel_mm': q['travel_mm'], 'spring_lengths_mm': lengths,
            'forces_N': [round(rate*max(0, free-length), 3) for length in lengths],
            'min_solid_margin_mm': round(lengths[2]-q['solid_length_mm'], 3),
            'max_working_deflection_margin_mm': round(q['max_deflection_mm']-(free-lengths[2]), 3),
            'installed_length_mm': installed,
            'shim_removed_force_N': round(rate*max(0, free-q['loaded_length_mm']), 3)}


def drive_layout(d, side):
    q = d.layout['suspension']['drive']
    base_top = d.p['base_bottom_z']+d.p['base_thickness']
    bracket = [(side*a, b) for a in (128, 140) for b in (30, 80)]
    hanger = [(side*128, b) for b in (-35, -15)]
    return q, base_top, bracket, hanger


def caster_layout(d, wheel):
    x, y = caster_xy(d, wheel)
    o = 1 if y > 0 else -1
    q = d.layout['suspension']['caster']
    flange = [(x+dx, y-o*dy) for dx in (-28, 28) for dy in (88, 102)]
    guide = (x, y-o*q['spring_offset_mm'])
    return q, x, y, o, flange, guide


def chassis_mount_holes(d):
    holes = []
    for side in (-1, 1):
        _, _, bracket, hanger = drive_layout(d, side)
        holes.extend((a, b, 5.5) for a, b in bracket+hanger)
    for wheel in CASTER_WHEELS:
        _, _, _, _, flange, guide = caster_layout(d, wheel)
        holes.extend((a, b, 5.5) for a, b in flange)
        holes.append((*guide, 8.5))
    return holes


def rail_block(q, axis, face, normal, center, z_bottom, block_center_z):
    """Rail on a mounting face and the block riding on it.

    axis 'x': mounting face normal is X (drive); 'y': normal is Y (caster).
    face: coordinate of the rail mounting face; normal: +1/-1 direction from the
    face toward the block; center: rail centreline in the other horizontal axis.
    """
    h, w, rh, length = q['rail_h_mm'], q['rail_w_mm'], q['rail_height_mm'], q['rail_length_mm']
    bw, bl = q['block_w_mm'], q['block_l_mm']

    def piece(c0, c1, a0, a1, z0, z1):
        n0, n1 = sorted((face+normal*a0, face+normal*a1))
        return span(n0, n1, c0, c1, z0, z1) if axis == 'x' else span(c0, c1, n0, n1, z0, z1)

    rail = piece(center-w/2, center+w/2, 0, rh, z_bottom, z_bottom+length)
    zb0, zb1 = block_center_z-bl/2, block_center_z+bl/2
    block = piece(center-bw/2, center+bw/2, 3, h, zb0, zb1)
    block -= piece(center-w/2-.5, center+w/2+.5, 2, rh+.5, zb0-1, zb1+1)
    return rail, block


def make(d):
    records = []
    p = d.p
    base_bottom = p['base_bottom_z']
    base_top = base_bottom+p['base_thickness']

    def add(name, shape, wheel, moving=False, sku=None, **metadata):
        if moving:
            shape = pose_shape(d, shape, wheel, d.motion.get(wheel, 0))
        color = (.77, .64, .36) if 'rail' in name or 'block' in name else (.48, .65, .78)
        status = 'misumi_catalog_envelope' if sku else 'suspension_fabrication_design'
        records.append(PartRecord('80_'+name+'_'+wheel, 'suspension', shape, color, status,
                                  {'wheel': wheel, 'moving_wheel': wheel if moving else None,
                                   **({'sku': sku} if sku else {}), **metadata}))

    def spring_stack(kind, wheel, x, y, seat, top, sleeve_od):
        """Shim on the moving seat, coil, loose bump sleeve, fixed guide bolt and droop nut."""
        q = d.layout['suspension'][kind]
        half = q['travel_mm']/2
        state = spring_state(d, kind)
        dz = d.motion.get(wheel, 0)
        if abs(top-seat-q['loaded_length_mm']) > 1e-6:
            raise ValueError('Spring stack and catalog seating length differ: '+wheel)
        bottom = seat
        if kind == 'caster' and q['shim_mm'] > 0:
            add(kind+'_preload_shim', ring(x, y, seat, q['shim_mm'], q['spring_od_mm']+1, 8.4), wheel, True,
                remove_without_arm=True)
            bottom = seat+q['shim_mm']
        if kind == 'drive' and q['shim_mm'] > 0:
            add(kind+'_preload_shim', ring(x, y, top-q['shim_mm'], q['shim_mm'], q['spring_od_mm']+1, 8.4), wheel,
                remove_without_arm=True)
        length = state['installed_length_mm']
        add(kind+'_spring_coil', coil(x, y, bottom+dz, length-dz, q), wheel, sku=q['spring_sku'],
            spring_top_z=bottom+length, spring_length_mm=length, spring_xy=[x, y], rate_N_mm=q['rate_N_mm'],
            free_length_mm=q['free_length_mm'], nominal_spring_force_N=state['forces_N'][1],
            geometry='catalog envelope with illustrative round-wire sweep')
        sleeve_top = bottom+length-half-.05
        add(kind+'_bump_stop_sleeve', ring(x, y, bottom, sleeve_top-bottom, sleeve_od, 8.4), wheel, True,
            material='Loose steel or urethane tube on the guide bolt; cut to CAD length')
        return state

    def droop_stop(kind, wheel, x, y, carrier_bottom, head_bearing):
        half = d.layout['suspension'][kind]['travel_mm']/2
        washer_top = carrier_bottom-half-.05
        nut_bottom = washer_top-1.6-NUT[8][1]
        end = nut_bottom-1.5
        add(kind+'_guide_bolt_M8', screw(x, y, head_bearing, 8, head_bearing-end), wheel, sku='M8 hex bolt',
            length_mm=round(head_bearing-end, 2))
        add(kind+'_droop_stop_washer_nut', ring(x, y, washer_top-1.6, 1.6, 16, 8.4)+nut(x, y, nut_bottom, 13, 6.5, 8),
            wheel, adjust='Nut sets the droop limit; lock with a second nut')

    # ---------------------------------------------------------------- drive
    for side in (-1, 1):
        key = 'LEFT' if side < 0 else 'RIGHT'
        q, _, bracket_holes, hanger_holes = drive_layout(d, side)
        ct, cx = q['carrier_thickness_mm'], q['carrier_x_mm']
        sx, sy, seat = q['spring_x_mm'], q['spring_y_mm'], q['spring_bottom_z_mm']
        bt, ht = q['bracket_thickness_mm'], q['hanger_thickness_mm']

        def X(a, b):
            return (side*a, side*b) if side > 0 else (side*b, side*a)

        inner = cx-ct/2
        carrier = span(*X(inner, cx+ct/2), -45, 95, 50, seat)
        carrier += span(*X(sx-10, inner), -45, -5, seat-ct, seat)
        carrier -= cylx(7.15, ct+2, side*cx, 0, 90) & span(*X(cx-4, cx+4), -5.1, 5.1, 80, 100)
        carrier -= cylz(4.5, ct+2, side*sx, sy, seat-ct-1)
        add('drive_carrier_6t_bent', carrier, key, True, material='S355 6t laser-cut, 1 bend; AF10.2 trial slot')
        face = inner-q['rail_h_mm']
        rail, block = rail_block(q, 'x', side*face, side, q['rail_y_mm'], q['rail_z_bottom_mm'], q['block_center_z_mm'])
        add('drive_rail', rail, key, sku=q['rail_sku'], moments_Nm=[q['block_ma_Nm'], q['block_mb_Nm'], q['block_mc_Nm']])
        add('drive_rail_block', block, key, True, sku=q['rail_sku'].split('-')[0]+' block')
        rail_top = q['rail_z_bottom_mm']+q['rail_length_mm']
        bracket = span(*X(120, face), 15, 95, base_top, base_top+bt)
        bracket += span(*X(face-bt, face), 15, 95, q['rail_z_bottom_mm']-3, max(rail_top, base_top+bt))
        bracket = drilled(bracket, bracket_holes, 5.5, base_top, bt)
        add('drive_rail_bracket_L', bracket, key, material='6t sheet, 1 bend; rail face as rolled, check flatness')
        add('drive_bracket_bolts', Compound([screw(a, b, base_top+bt, 5, 16) for a, b in bracket_holes]),
            key, sku='CB5-16', quantity=4)
        add('drive_bracket_nuts', Compound([nut(a, b, base_bottom-4, 8, 4, 5) for a, b in bracket_holes]), key, quantity=4)
        # Axle stack: unchanged trial dimensions, now carried by the vertical carrier.
        spacer = cylx(9, 4.9, side*(cx+5.45), 0, 90)-cylx(7.15, 6, side*(cx+5.45), 0, 90)
        add('drive_axial_spacer_trial', spacer, key, True, fit='Confirm stationary shoulder and bearing-face diameters on motor')
        washer = cylx(10, 1, side*(cx-3.5), 0, 90)-cylx(7.15, 3, side*(cx-3.5), 0, 90)
        add('drive_axle_washer', washer, key, True)
        axle_nut = nut(0, 0, 0, 22, 8, 14.3).rotate(Axis.Y, 90).translate((side*(cx-4)-8*(side > 0), 0, 90))
        add('drive_axle_M14x1p5_nut_envelope', axle_nut, key, True, fit='OEM nut thickness and wrench envelope assumed')
        # C hanger carries the spring top over the wheel well.
        top = seat+q['loaded_length_mm']
        hanger = span(*X(120, 141), -45, -5, base_top, base_top+ht)
        hanger += span(*X(141-ht, 141), -40, -10, base_top, top+ht)
        hanger += span(*X(141-ht, sx+12), -40, -10, top, top+ht)
        hanger = drilled(hanger, [(side*sx, sy)], 8.5, top, ht)
        hanger = drilled(hanger, hanger_holes, 5.5, base_top, ht)
        add('drive_spring_hanger_C', hanger, key, material='4.5t sheet, 2 bends')
        add('drive_hanger_bolts', Compound([screw(a, b, base_top+ht, 5, 14) for a, b in hanger_holes]), key, sku='CB5-14', quantity=2)
        add('drive_hanger_nuts', Compound([nut(a, b, base_bottom-4, 8, 4, 5) for a, b in hanger_holes]), key, quantity=2)
        spring_stack('drive', key, side*sx, sy, seat, top, 13)
        droop_stop('drive', key, side*sx, sy, seat-ct, top+ht)

    # ---------------------------------------------------------------- casters
    plate_top = d.parts['caster']['cad_height']
    for wheel in CASTER_WHEELS:
        q, x, y, o, flange_holes, (gx, gy) = caster_layout(d, wheel)
        ct, li = q['carrier_thickness_mm'], q['carrier_leg_inner_mm']
        bt = q['bracket_thickness_mm']

        def Y(a, b):
            """Y extents from offsets measured toward the robot centre (positive = inboard)."""
            return tuple(sorted((y-o*a, y-o*b)))

        carrier_top = plate_top+ct
        bolts = [(x+dx, y+dy) for dx in (-12, 12) for dy in (-27, 27)]
        carrier = span(x-35, x+35, *Y(-35, li+ct), plate_top, carrier_top)
        block_top_z = carrier_top
        carrier += span(x-35, x+35, *Y(li, li+ct), block_top_z-q['block_l_mm']-1, carrier_top)
        carrier = drilled(carrier, bolts, 6.6, plate_top, ct)
        carrier = drilled(carrier, [(gx, gy)], 9, plate_top, ct)
        add('caster_carrier_6t_bent', carrier, wheel, True, material='S355 6t laser-cut, 1 bend')
        face = li+ct+q['rail_h_mm']
        bc = block_top_z-q['block_l_mm']/2
        rail_bottom = bc-q['block_l_mm']/2-q['travel_mm']/2-((q['rail_length_mm']-q['block_l_mm']-q['travel_mm'])/2)
        rail, block = rail_block(q, 'y', y-o*face, o, x, rail_bottom, bc)
        add('caster_rail', rail, wheel, sku=q['rail_sku'], moments_Nm=[q['block_ma_Nm'], q['block_mb_Nm'], q['block_mc_Nm']])
        add('caster_rail_block', block, wheel, True, sku=q['rail_sku'].split('-')[0]+' block')
        bracket = span(x-35, x+35, *Y(face, face+bt), rail_bottom-2, base_bottom)
        bracket += span(x-35, x+35, *Y(face, 110), base_bottom-bt, base_bottom)
        bracket = drilled(bracket, flange_holes, 5.5, base_bottom-bt, bt)
        add('caster_rail_bracket_L', bracket, wheel, material='6t sheet, 1 bend; rail face as rolled, check flatness')
        add('caster_bracket_bolts', Compound([screw(a, b, base_top, 5, 16) for a, b in flange_holes]),
            wheel, sku='CB5-16', quantity=4)
        add('caster_bracket_nuts', Compound([nut(a, b, base_bottom-bt-4, 8, 4, 5) for a, b in flange_holes]),
            wheel, quantity=4)
        add('caster_plate_bolts', Compound([screw(a, b, plate_top-5, 6, 16, True) for a, b in bolts]),
            wheel, True, sku='CB6-16', quantity=4, plate_thickness_reference_mm=5)
        add('caster_plate_nuts', Compound([nut(a, b, carrier_top, 10, 5, 6) for a, b in bolts]), wheel, True, quantity=4)
        spring_stack('caster', wheel, gx, gy, carrier_top, base_bottom, 10)
        droop_stop('caster', wheel, gx, gy, plate_top, base_top)
    return records
