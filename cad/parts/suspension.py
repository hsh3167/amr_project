"""Rev10 suspension: drive side swing arms, front fixed casters, rear roll beam.

Laser-cut and bent sheet parts plus catalog pins, bearings and springs; no welding.
Rev09 (welded caster rockers, twin-shaft drive guides) is kept in git history
(commit 4643d47) and in the earlier STEP outputs. Dimensions in mm.
"""
from math import cos, pi, sin
from build123d import Axis, Circle, Compound, Helix, Plane, sweep
from ..caster_motion import (CASTER_WHEELS, REAR_CASTERS, caster_xy, drive_arm_length, half_travel,
                             pose_shape, rotate_point, seat_travel)
from .common import PartRecord, cylx, cyly, cylz, drilled, polygon_plate, span

SCREW_HEAD = {3: 5.5, 4: 7, 5: 8.5, 6: 10, 8: 13}


def ring(x, y, bottom, height, od, bore):
    return cylz(od/2, height, x, y, bottom)-cylz(bore/2, height+2, x, y, bottom-1)


def ring_y(x, y0, y1, z, od, bore):
    """Ring along Y between y0 and y1."""
    length, mid = abs(y1-y0), (y0+y1)/2
    return cyly(od/2, length, x, mid, z)-cyly(bore/2, length+2, x, mid, z)


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


def drive_geometry(d):
    q = d.layout['suspension']['drive']
    web_top = q['arm_wall_top_z_mm']
    seat = (q['spring_x_mm'], web_top)
    half = q['travel_mm']/2
    # Vertical travel of the spring seat for wheel-centre travel of +-half.
    seat_half = seat_travel(d, 'RIGHT', half, *seat)
    return {'spring_bottom': web_top, 'spring_top': web_top+q['loaded_length_mm'],
            'seat_half_travel': seat_half, 'lever_ratio': (q['spring_x_mm']-q['pivot_x_mm'])/drive_arm_length(d)}


def spring_state(d, kind='drive'):
    q = d.layout['suspension'][kind]
    g = drive_geometry(d)
    half = g['seat_half_travel']
    installed = q['loaded_length_mm']-q['shim_mm']
    free, rate = q['free_length_mm'], q['rate_N_mm']
    lengths = [round(installed+half, 3), installed, round(installed-half, 3)]
    return {'sku': q['spring_sku'], 'travel_mm': q['travel_mm'], 'seat_travel_mm': round(2*half, 3),
            'lever_ratio': round(g['lever_ratio'], 4), 'spring_lengths_mm': lengths,
            'forces_N': [round(rate*max(0, free-length), 3) for length in lengths],
            'min_solid_margin_mm': round(lengths[2]-q['solid_length_mm'], 3),
            'max_working_deflection_margin_mm': round(q['max_deflection_mm']-(free-lengths[2]), 3),
            'installed_length_mm': installed}


def drive_mounts(d, side):
    q = d.layout['suspension']['drive']
    px = q['pivot_x_mm']
    pivot = [(side*(px+dx), y) for dx in (-12, 12) for y in (-22, 22)]
    hanger = [(side*137, y) for y in (-42, 42)]
    return pivot, hanger


def front_caster_mounts(x, y, d):
    q = d.layout['suspension']['front_caster_mount']
    return [(x+dx, y+dy) for dx in (-(q['half_x_mm']-10), q['half_x_mm']-10) for dy in (-25, 25)]


def rear_mounts(d):
    y = -d.p['caster_y']
    q = d.layout['suspension']['rear_rocker']
    pivot = [(x, y+dy) for x in (-20, 20) for dy in (13, 33)]
    stops = [(-q['stop_x_mm'], y), (q['stop_x_mm'], y)]
    return pivot, stops


def chassis_mount_holes(d):
    holes = []
    for side in (-1, 1):
        pivot, hanger = drive_mounts(d, side)
        holes.extend((x, y, 5.5) for x, y in pivot+hanger)
    for wheel in ('LF', 'RF'):
        holes.extend((x, y, 5.5) for x, y in front_caster_mounts(*caster_xy(d, wheel), d))
    pivot, stops = rear_mounts(d)
    holes.extend((x, y, 5.5) for x, y in pivot)
    holes.extend((x, y, 8.5) for x, y in stops)
    return holes


def drive_spring_notch(d):
    """Half-width and inboard edge of the base-plate notch the drive spring passes through."""
    return 14, d.layout['suspension']['drive']['spring_x_mm']-11


def make(d):
    records = []
    p = d.p
    base_bottom = p['base_bottom_z']
    base_top = base_bottom+p['base_thickness']

    def add(name, shape, wheel, moving=None, sku=None, **metadata):
        if moving:
            shape = pose_shape(d, shape, moving, d.motion.get(moving, 0))
        color = (.77, .64, .36) if 'bearing' in name else (.48, .65, .78)
        status = 'misumi_catalog_envelope' if sku else 'suspension_fabrication_design'
        records.append(PartRecord('80_'+name+'_'+wheel, 'suspension', shape, color, status,
                                  {'wheel': wheel, 'moving_wheel': moving,
                                   **({'sku': sku} if sku else {}), **metadata}))

    # ---------------------------------------------------------------- drive
    q = d.layout['suspension']['drive']
    g = drive_geometry(d)
    t = q['arm_thickness_mm']
    px, pz = q['pivot_x_mm'], q['pivot_z_mm']
    wall_y, top, bottom = q['arm_wall_y_mm'], q['arm_wall_top_z_mm'], q['arm_wall_bottom_z_mm']
    end_x, sx = q['end_wall_x_mm'], q['spring_x_mm']
    bt, ear_y = q['bracket_thickness_mm'], q['ear_inner_y_mm']
    ht = q['hanger_thickness_mm']
    bearing_w = q['bearing_width_mm']
    for side in (-1, 1):
        key = 'LEFT' if side < 0 else 'RIGHT'

        def X(a, b):
            return (side*a, side*b) if side > 0 else (side*b, side*a)

        # One bent 6t tray: two side walls (bearing bores), web (spring seat), end wall (axle slot).
        arm = None
        for sy in (-1, 1):
            y0, y1 = sorted((sy*wall_y, sy*(wall_y+t)))
            wall = span(*X(px-15, end_x+t/2), y0, y1, bottom, top)
            arm = wall if arm is None else arm+wall
        arm += span(*X(60, sx+9), -(wall_y+t), wall_y+t, top-t, top)
        arm += span(*X(end_x-t/2, end_x+t/2), -(wall_y+t), wall_y+t, 66, 112)
        arm -= cyly(q['bearing_od_mm']/2, 2*(wall_y+t)+2, side*px, 0, pz)
        arm -= cylz(4.5, t+2, side*sx, 0, top-t-1)
        slot = cylx(7.15, 8, side*end_x, 0, 90) & span(*X(end_x-4, end_x+4), -5.1, 5.1, 80, 100)
        arm -= slot
        add('drive_swing_arm_6t_bent', arm, key, key, material='S355 6t laser-cut, 3 bends; AF10.2 trial slot',
            pivot_xz=[side*px, pz], lever_ratio=round(g['lever_ratio'], 4))
        for sy in (-1, 1):
            y0, y1 = sorted((sy*wall_y, sy*(wall_y+bearing_w)))
            add('drive_pivot_bearing_'+('P' if sy > 0 else 'N'),
                ring_y(side*px, y0, y1, pz, q['bearing_od_mm'], q['bearing_id_mm']), key, key,
                sku=q['bearing'], fit='22 H7 housing, verify supplier and retention')
        # Axle stack: unchanged trial dimensions from Rev09, now carried by the arm.
        ax = end_x
        spacer = cylx(9, 4.9, side*(ax+5.45), 0, 90)-cylx(7.15, 6, side*(ax+5.45), 0, 90)
        add('drive_axial_spacer_trial', spacer, key, key, fit='Confirm stationary shoulder and bearing-face diameters on motor')
        washer = cylx(10, 1, side*(ax-3.5), 0, 90)-cylx(7.15, 3, side*(ax-3.5), 0, 90)
        add('drive_axle_washer', washer, key, key)
        from build123d import Axis as _Axis
        axle_nut = nut(0, 0, 0, 22, 8, 14.3).rotate(_Axis.Y, 90)
        axle_nut = axle_nut.translate((side*(ax-4)-8*(side > 0), 0, 90))
        add('drive_axle_M14x1p5_nut_envelope', axle_nut, key, key, fit='OEM nut thickness and wrench envelope assumed')

        # Fixed U bracket under the base plate, two ears hold the hinge pin.
        bracket = span(*X(px-20, px+20), -(ear_y+bt), ear_y+bt, base_bottom-bt, base_bottom)
        for sy in (-1, 1):
            y0, y1 = sorted((sy*ear_y, sy*(ear_y+bt)))
            bracket += span(*X(px-20, px+20), y0, y1, 80, base_bottom-bt)
        bracket -= cyly(q['pin_d_mm']/2+.005, 2*(ear_y+bt)+2, side*px, 0, pz)
        pivot_holes, hanger_holes = drive_mounts(d, side)
        bracket = drilled(bracket, pivot_holes, 5.5, base_bottom-bt, bt)
        add('drive_pivot_bracket_U', bracket, key, material='4.5t sheet, 2 bends; ream ear holes together after bending')
        add('drive_pivot_bracket_bolts', Compound([screw(a, b, base_top, 5, 14) for a, b in pivot_holes]),
            key, sku='CB5-14', quantity=4)
        add('drive_pivot_bracket_nuts', Compound([nut(a, b, base_bottom-bt-4, 8, 4, 5) for a, b in pivot_holes]),
            key, quantity=4)
        half_pin = q['pin_length_mm']/2
        pin_lo = -(ear_y+bt)
        pin = cyly(q['pin_d_mm']/2, q['pin_length_mm'], side*px, pin_lo+half_pin, pz)
        pin += cyly(q['pin_head_d_mm']/2, q['pin_head_thickness_mm'], side*px, pin_lo-q['pin_head_thickness_mm']/2, pz)
        pin -= cyly(3, 10, side*px, pin_lo+q['pin_length_mm']-4, pz)
        add('drive_hinge_pin', pin, key, sku=q['pin_sku'], configured_length='verify catalog length increment')
        pin_end = pin_lo+q['pin_length_mm']
        keeper = ring_y(side*px, pin_end, pin_end+1, pz, 16, 6.4)
        keeper += cyly(3, 8, side*px, pin_end-3, pz)+cyly(5, 6, side*px, pin_end+4, pz)
        add('drive_pin_retainer', keeper, key, sku='CB6-8', washer='M6 OD16 t1')
        for sy in (-1, 1):
            y0, y1 = sorted((sy*(wall_y+bearing_w), sy*ear_y))
            add('drive_pivot_spacer_'+('P' if sy > 0 else 'N'), ring_y(side*px, y0, y1, pz, 14, 10.1), key,
                fit='Clamps bearing inner ring to ear; thrust face')

        # Fixed C-shaped spring hanger on top of the base plate.
        spring_top = g['spring_top']
        notch_half, notch_x = drive_spring_notch(d)
        hanger = span(*X(124-ht/2, 145), -50, 50, base_top, base_top+ht)
        hanger += span(*X(124-ht/2, 124+ht/2), -15, 15, base_top, spring_top+ht)
        hanger += span(*X(124-ht/2, sx+12), -15, 15, spring_top, spring_top+ht)
        hanger -= cylz(13, ht+2, side*sx, 0, base_top-1)
        hanger = drilled(hanger, [(side*sx, 0)], 6.6, spring_top, ht)
        hanger = drilled(hanger, hanger_holes, 5.5, base_top, ht)
        add('drive_spring_hanger_C', hanger, key, material='4.5t sheet, 2 bends')
        add('drive_hanger_bolts', Compound([screw(a, b, base_top+ht, 5, 14) for a, b in hanger_holes]),
            key, sku='CB5-14', quantity=2)
        add('drive_hanger_nuts', Compound([nut(a, b, base_bottom-4, 8, 4, 5) for a, b in hanger_holes]),
            key, quantity=2)
        # Guide bolt: droop limit by washer+nut under the arm web, bump limit by the sleeve.
        web_bottom = top-t
        droop = min(rotate_point(d, key, -q['travel_mm']/2, (side*(sx+dx), web_bottom))[1] for dx in (-6, 6))
        bump = max(rotate_point(d, key, q['travel_mm']/2, (side*(sx+dx), top))[1] for dx in (-4.5, 4.5))
        washer_top = droop-.05
        bolt_end = washer_top-1.6-6
        bolt = screw(side*sx, 0, spring_top+ht, 6, spring_top+ht-bolt_end)
        add('drive_spring_guide_bolt', bolt, key, sku=q['guide_bolt'], length_mm=round(spring_top+ht-bolt_end, 2))
        stop = ring(side*sx, 0, washer_top-1.6, 1.6, 12, 6.4)+nut(side*sx, 0, washer_top-1.6-5, 10, 5, 6)
        add('drive_droop_stop_washer_nut', stop, key, droop_contact_z=round(droop, 3))
        sleeve = ring(side*sx, 0, bump+.05, spring_top-bump-.05, 9, 6.5)
        add('drive_bump_stop_sleeve', sleeve, key, material='Urethane or steel tube OD9 ID6.5; cut to length')
        dz = seat_travel(d, key, d.motion.get(key, 0), side*sx, top)
        state = spring_state(d)
        length = spring_top-g['spring_bottom']
        if abs(length-state['installed_length_mm']-q['shim_mm']) > 1e-6:
            raise ValueError('Spring stack and catalog seating length differ: '+key)
        add('drive_spring_coil', coil(side*sx, 0, g['spring_bottom']+q['shim_mm']+dz, length-q['shim_mm']-dz, q), key,
            sku=q['spring_sku'], spring_top_z=spring_top, spring_length_mm=length-q['shim_mm'],
            spring_xy=[side*sx, 0], rate_N_mm=q['rate_N_mm'], free_length_mm=q['free_length_mm'],
            nominal_spring_force_N=state['forces_N'][1], seat_travel_ratio=round(g['lever_ratio'], 4),
            geometry='catalog envelope with illustrative round-wire sweep')

    # ---------------------------------------------------------------- casters
    plate_top = d.parts['caster']['cad_height']
    f = d.layout['suspension']['front_caster_mount']
    for wheel in ('LF', 'RF'):
        x, y = caster_xy(d, wheel)
        ft, hx, hy, fl = f['thickness_mm'], f['half_x_mm'], f['half_y_mm'], f['flange_mm']
        bolts = [(x+dx, y+dy) for dx in (-12, 12) for dy in (-27, 27)]
        mount = span(x-hx, x+hx, y-hy, y+hy, plate_top, plate_top+ft)
        for sx_ in (-1, 1):
            x0, x1 = sorted((x+sx_*(hx-ft), x+sx_*hx))
            mount += span(x0, x1, y-hy, y+hy, plate_top+ft, base_bottom-ft)
            x0, x1 = sorted((x+sx_*(hx-fl), x+sx_*hx))
            mount += span(x0, x1, y-hy, y+hy, base_bottom-ft, base_bottom)
        mount = drilled(mount, bolts, 6.6, plate_top, ft)
        holes = front_caster_mounts(x, y, d)
        mount = drilled(mount, holes, 5.5, base_bottom-ft, ft)
        add('caster_fixed_box_bracket', mount, wheel, material='4.5t sheet, 4 bends; sets GL-65 height without spacers')
        add('caster_frame_bolts', Compound([screw(a, b, base_top, 5, 14) for a, b in holes]), wheel, sku='CB5-14', quantity=4)
        add('caster_frame_nuts', Compound([nut(a, b, base_bottom-ft-4, 8, 4, 5) for a, b in holes]), wheel, quantity=4)
        add('caster_plate_bolts', Compound([screw(a, b, plate_top-5, 6, 16, True) for a, b in bolts]),
            wheel, sku='CB6-16', quantity=4, plate_thickness_reference_mm=5)
        add('caster_plate_nuts', Compound([nut(a, b, plate_top+ft, 10, 5, 6) for a, b in bolts]), wheel, quantity=4)

    r = d.layout['suspension']['rear_rocker']
    yc = -d.p['caster_y']
    bt = r['beam_thickness_mm']
    hl, hw = r['beam_half_length_mm'], r['beam_width_mm']/2
    rz_, tw = r['pivot_z_mm'], r['tab_half_width_mm']
    yf, yb = yc+hw, yc-hw
    beam = span(-hl, hl, yb, yf, plate_top, plate_top+bt)
    beam -= span(-tw, tw, yc-9, yc+9, plate_top-1, plate_top+bt+1)
    for y0, y1 in ((yf-bt, yf), (yb, yb+bt)):
        beam += span(-tw, tw, y0, y1, r['tab_bottom_z_mm'], plate_top)
    beam -= cyly(r['bearing_od_mm']/2, 2*hw+2, 0, yc, rz_)
    bolts = [(sx_*d.p['caster_x']+dx, yc+dy) for sx_ in (-1, 1) for dx in (-12, 12) for dy in (-27, 27)]
    beam = drilled(beam, bolts, 6.6, plate_top, bt)
    add('caster_rear_roll_beam_6t', beam, 'REAR', 'REAR', material='S355 6t laser-cut, 2 bent tabs',
        roll_limit_deg=r['roll_limit_deg'])
    for name, (y0, y1) in (('F', (yf-bt, yf)), ('B', (yb, yb+bt))):
        add('caster_rear_pivot_bearing_'+name, ring_y(0, y0, y1, rz_, r['bearing_od_mm'], r['bearing_id_mm']),
            'REAR', 'REAR', sku=r['bearing'])
    add('caster_plate_bolts', Compound([screw(a, b, plate_top-5, 6, 16, True) for a, b in bolts]),
        'REAR', 'REAR', sku='CB6-16', quantity=8, plate_thickness_reference_mm=5)
    add('caster_plate_nuts', Compound([nut(a, b, plate_top+bt, 10, 5, 6) for a, b in bolts]), 'REAR', 'REAR', quantity=8)
    # Fixed L bracket: tab through the beam window, flange under the base plate.
    ct = r['bracket_thickness_mm']
    bracket = span(-15, 15, yc-ct/2, yc+ct/2, r['bracket_tab_bottom_z_mm'], base_bottom)
    bracket += span(-30, 30, yc+ct/2, yc+43, base_bottom-ct, base_bottom)
    bracket -= cyly(r['pin_d_mm']/2+.005, ct+2, 0, yc, rz_)
    pivot_holes, stops = rear_mounts(d)
    bracket = drilled(bracket, pivot_holes, 5.5, base_bottom-ct, ct)
    add('caster_rear_pivot_bracket_L', bracket, 'REAR', material='6t sheet, 1 bend')
    add('caster_rear_bracket_bolts', Compound([screw(a, b, base_top, 5, 14) for a, b in pivot_holes]),
        'REAR', sku='CB5-14', quantity=4)
    add('caster_rear_bracket_nuts', Compound([nut(a, b, base_bottom-ct-4, 8, 4, 5) for a, b in pivot_holes]),
        'REAR', quantity=4)
    pin_lo = yb-.5
    pin = cyly(r['pin_d_mm']/2, r['pin_length_mm'], 0, pin_lo+r['pin_length_mm']/2, rz_)
    pin += cyly(r['pin_head_d_mm']/2, r['pin_head_thickness_mm'], 0, pin_lo-r['pin_head_thickness_mm']/2, rz_)
    pin -= cyly(3, 10, 0, pin_lo+r['pin_length_mm']-4, rz_)
    add('caster_rear_hinge_pin', pin, 'REAR', sku=r['pin_sku'], configured_length='verify catalog length increment')
    pin_end = pin_lo+r['pin_length_mm']
    keeper = ring_y(0, pin_end, pin_end+1, rz_, 16, 6.4)+cyly(3, 8, 0, pin_end-3, rz_)+cyly(5, 6, 0, pin_end+4, rz_)
    add('caster_rear_pin_retainer', keeper, 'REAR', sku='CB6-8', washer='M6 OD16 t1')
    for name, (y0, y1) in (('F', (yc+ct/2, yf-bt)), ('B', (yb+bt, yc-ct/2))):
        add('caster_rear_pivot_spacer_'+name, ring_y(0, y0, y1, rz_, 14, 10.1), 'REAR',
            fit='Spaces bearing inner rings from the fixed tab')
    roll_half = half_travel(d, 'REAR')
    for sx_, (a, b) in zip((-1, 1), stops):
        # Rising end of the beam meets this stop at the roll limit.
        value = roll_half if sx_ < 0 else -roll_half
        contact = max(rotate_point(d, 'REAR', value, (a+dx, plate_top+bt))[1] for dx in (-4, 4))
        end = contact+.05
        stop = screw(a, b, base_top, 8, base_top-end)+nut(a, b, base_bottom-6.5, 13, 6.5, 8)
        add('caster_rear_roll_stop_'+('L' if sx_ < 0 else 'R'), stop, 'REAR', sku='M8 hex bolt + jam nut',
            contact_z=round(contact, 3), adjust='Set gap with feeler gauge; add urethane cap if impact noise')
    return records
