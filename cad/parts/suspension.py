"""Rev12 suspension: six independent hinge + spring modules (pivot axis +Y).

Per wheel: one bent-sheet bracket under the base plate, one bent arm, one catalog
hinge pin, one X-SWU spring between the arm and the base plate underside on an
M8 stud. The stud's top nut is the droop stop and an M6 bolt through the base
plate is the bump stop. No welding, rails, hangers or bearings. Earlier concepts
(Rev09 shafts/rockers, Rev10 swing arm + seesaw, Rev11 rails) are in git history.
Dimensions in mm.
"""
from math import cos, pi, sin
from build123d import Axis, Circle, Compound, Helix, Plane, sweep
from ..caster_motion import CASTER_WHEELS, caster_xy, pivot_of, pose_shape, rotate_point, seat_travel
from .common import PartRecord, cylx, cyly, cylz, drilled, polygon_plate, span

SCREW_HEAD = {3: 5.5, 4: 7, 5: 8.5, 6: 10, 8: 13}


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


def spring_point(d, key):
    """(x, y, seat z) of the spring on the moving arm."""
    if key in ('LEFT', 'RIGHT'):
        q = d.layout['suspension']['drive']
        side = -1 if key == 'LEFT' else 1
        return side*q['spring_x_mm'], q['spring_y_mm'], q['arm_bottom_z_mm']+q['arm_thickness_mm']
    q = d.layout['suspension']['caster']
    x, y = caster_xy(d, key)
    side = -1 if x < 0 else 1
    return x-side*q['spring_offset_mm'], y, d.parts['caster']['cad_height']+q['arm_thickness_mm']


def lever_ratio(d, kind):
    """Spring travel / wheel travel (also wheel force / spring force)."""
    key = 'RIGHT' if kind == 'drive' else 'RF'
    (px, _, _), length = pivot_of(d, key)
    x, _, _ = spring_point(d, key)
    return abs(x-px)/length


def spring_state(d, kind):
    q = d.layout['suspension'][kind]
    ratio = lever_ratio(d, kind)
    half = q['travel_mm']/2*ratio
    installed = q['loaded_length_mm']-q['shim_mm']
    free, rate = q['free_length_mm'], q['rate_N_mm']
    lengths = [round(installed+half, 3), installed, round(installed-half, 3)]
    return {'sku': q['spring_sku'], 'travel_mm': q['travel_mm'], 'lever_ratio': round(ratio, 4),
            'spring_travel_mm': round(2*half, 3), 'spring_lengths_mm': lengths,
            'forces_N': [round(rate*max(0, free-length), 3) for length in lengths],
            'min_solid_margin_mm': round(lengths[2]-q['solid_length_mm'], 3),
            'max_working_deflection_margin_mm': round(q['max_deflection_mm']-(free-lengths[2]), 3),
            'installed_length_mm': installed,
            'shim_removed_force_N': round(rate*max(0, free-q['loaded_length_mm']), 3)}


def bracket_holes(d, key):
    (px, py, _), _ = pivot_of(d, key)
    if key in ('LEFT', 'RIGHT'):
        return [(px+dx, py+dy) for dx in (-7, 7) for dy in (-25, 25)]
    return [(px, py+dy) for dy in (-15, 15)]


def bump_stop_xy(d, key):
    if key in ('LEFT', 'RIGHT'):
        return (140 if key == 'RIGHT' else -140), 20
    x, y = caster_xy(d, key)
    off = d.layout['suspension']['caster']['bump_stop_offset_mm']
    return x-(off if x > 0 else -off), y


def stud_hole(d, key):
    return 13 if key in ('LEFT', 'RIGHT') else 12


def chassis_mount_holes(d):
    holes = []
    for key in ('LEFT', 'RIGHT', *CASTER_WHEELS):
        holes.extend((a, b, 5.5) for a, b in bracket_holes(d, key))
        x, y, _ = spring_point(d, key)
        holes.append((x, y, stud_hole(d, key)))
        holes.append((*bump_stop_xy(d, key), 6.5))
    return holes


def make(d):
    records = []
    p = d.p
    base_bottom = p['base_bottom_z']
    base_top = base_bottom+p['base_thickness']

    def add(name, shape, wheel, moving=False, sku=None, **metadata):
        if moving:
            shape = pose_shape(d, shape, wheel, d.motion.get(wheel, 0))
        color = (.77, .64, .36) if 'pin' in name else (.48, .65, .78)
        status = 'misumi_catalog_envelope' if sku else 'suspension_fabrication_design'
        records.append(PartRecord('80_'+name+'_'+wheel, 'suspension', shape, color, status,
                                  {'wheel': wheel, 'moving_wheel': wheel if moving else None,
                                   **({'sku': sku} if sku else {}), **metadata}))

    def hinge(kind, key, x0, x1, ear_y, ear_bottom, q):
        """Fixed U bracket under the base plate, hinge pin and retainer."""
        (px, py, pz), _ = pivot_of(d, key)
        bt = q['bracket_thickness_mm']
        bracket = span(min(x0, x1), max(x0, x1), py-ear_y-bt, py+ear_y+bt, base_bottom-bt, base_bottom)
        for sy in (-1, 1):
            y0, y1 = sorted((py+sy*ear_y, py+sy*(ear_y+bt)))
            bracket += span(px-12, px+12, y0, y1, ear_bottom, base_bottom-bt)
        bracket -= cyly(q['pin_d_mm']/2+.005, 2*(ear_y+bt)+2, px, py, pz)
        holes = bracket_holes(d, key)
        bracket = drilled(bracket, holes, 5.5, base_bottom-bt, bt)
        add(kind+'_hinge_bracket_U', bracket, key, material=f'{bt}t sheet, 2 bends; ream both ear holes together')
        add(kind+'_bracket_bolts', Compound([screw(a, b, base_top, 5, 14) for a, b in holes]), key,
            sku='CB5-14', quantity=len(holes))
        add(kind+'_bracket_nuts', Compound([nut(a, b, base_bottom-bt-4, 8, 4, 5) for a, b in holes]), key,
            quantity=len(holes))
        span_y = 2*(ear_y+bt)
        length = round(span_y+1, 1)
        lo = py-ear_y-bt
        pin = cyly(q['pin_d_mm']/2, length, px, lo+length/2, pz)
        pin += cyly(q['pin_head_d_mm']/2, q['pin_head_thickness_mm'], px, lo-q['pin_head_thickness_mm']/2, pz)
        pin -= cyly(3, 10, px, lo+length-4, pz)
        add(kind+'_hinge_pin', pin, key, sku=q['pin_sku'], length_mm=length,
            fit='Fixed in ear holes (H7), arm turns on it; grease. Catalog D10 L25-100 step 0.1')
        end = lo+length
        keeper = cyly(8, 1, px, end+.5, pz)-cyly(3.2, 3, px, end+.5, pz)
        keeper += cyly(3, 8, px, end-3, pz)+cyly(5, 6, px, end+4, pz)
        add(kind+'_pin_retainer', keeper, key, sku='CB6-8', washer='M6 OD16 t1')

    def bushes(kind, key, outer, q):
        """Two flanged oil-free bushes pressed into the arm from outside; flanges face the bracket ears."""
        (px, py, pz), _ = pivot_of(d, key)
        ft, length = q['bush_flange_t_mm'], q['bush_length_mm']
        parts = []
        for sy in (-1, 1):
            face = py+sy*outer
            sleeve = cyly(q['bush_od_mm']/2, length-ft, px, face-sy*(length-ft)/2, pz)
            flange = cyly(q['bush_flange_d_mm']/2, ft, px, face+sy*ft/2, pz)
            parts.append(sleeve+flange-cyly(q['bush_bore_mm']/2, length+2, px, face-sy*(length/2-ft), pz))
        add(kind+'_hinge_bushes', Compound(parts), key, True, sku=q['bush_sku'], quantity=2,
            fit='Press into 14 H7 arm bore (m6 OD); runs on the g6 pin (F7 bore); flange = thrust washer')

    def spring_stack(kind, key, washer_od):
        q = d.layout['suspension'][kind]
        half = q['travel_mm']/2
        x, y, seat = spring_point(d, key)
        state = spring_state(d, kind)
        if abs(base_bottom-seat-q['loaded_length_mm']) > 1e-6:
            raise ValueError('Spring gap and catalog seating length differ: '+key)
        value = d.motion.get(key, 0)
        bottom = seat+q['shim_mm']
        add(kind+'_preload_shim', ring(x, y, seat, q['shim_mm'], q['spring_od_mm']+1, 8.4), key, True,
            remove_without_arm=True)
        length = state['installed_length_mm']
        dz = seat_travel(d, key, value, x, bottom)
        add(kind+'_spring_coil', coil(x, y, bottom+dz, length-dz, q), key, sku=q['spring_sku'],
            spring_top_z=bottom+length, spring_length_mm=length, spring_xy=[x, y], rate_N_mm=q['rate_N_mm'],
            free_length_mm=q['free_length_mm'], nominal_spring_force_N=state['forces_N'][1],
            lever_ratio=state['lever_ratio'], geometry='catalog envelope with illustrative round-wire sweep')
        # Washer and nut on the stud land on the base plate top at full droop.
        w = washer_od/2
        drop = min(seat_travel(d, key, -half, x+dx, base_top) for dx in (-w, w))
        washer_bottom = base_top-drop+.05
        nut_top = washer_bottom+1.6+6.5
        stud = cylz(4, nut_top+1.5-(seat-q['arm_thickness_mm']), x, y, seat-q['arm_thickness_mm'])
        add(kind+'_stud_M8', stud, key, True, sku='M8 stud', length_mm=round(nut_top+1.5-seat+q['arm_thickness_mm'], 1),
            fit='Threaded into the arm (tapped M8) and locked with a nut')
        stop = ring(x, y, washer_bottom, 1.6, washer_od, 8.4)+nut(x, y, washer_bottom+1.6, 13, 6.5, 8)
        add(kind+'_droop_stop_washer_nut', stop, key, True, adjust='Turn the nut to set the droop limit; lock with a second nut')
        return state

    def bump_stop(kind, key, x, y, surface):
        """M6 stop bolt through the base plate; its tip meets the arm at full bump."""
        half = d.layout['suspension'][kind]['travel_mm']/2
        rise = max(seat_travel(d, key, half, x+dx, surface) for dx in (-3, 3))
        tip = surface+rise+.05
        stop = screw(x, y, base_top, 6, base_top-tip)+nut(x, y, base_bottom-5, 10, 5, 6)
        add(kind+'_bump_stop_bolt_M6', stop, key, sku='M6 hex bolt + jam nut', tip_z=round(tip, 3),
            adjust='Set the bump limit with the jam nut; urethane cap optional')

    # ---------------------------------------------------------------- drive
    q = d.layout['suspension']['drive']
    t = q['arm_thickness_mm']
    for side in (-1, 1):
        key = 'LEFT' if side < 0 else 'RIGHT'
        (px, _, pz), _ = pivot_of(d, key)

        def X(a, b):
            return (side*a, side*b) if side > 0 else (side*b, side*a)

        wy, top, bot, ex = q['arm_wall_y_mm'], q['arm_top_z_mm'], q['arm_bottom_z_mm'], q['end_wall_x_mm']
        a0 = abs(px)-12
        arm = span(*X(a0, ex+t/2), -(wy+t), wy+t, bot, bot+t)
        for sy in (-1, 1):
            y0, y1 = sorted((sy*wy, sy*(wy+t)))
            arm += span(*X(a0, ex+t/2), y0, y1, bot+t, top)
        arm += span(*X(ex-t/2, ex+t/2), -(wy+t), wy+t, bot+t, top)
        arm -= cyly(q['bush_od_mm']/2, 2*(wy+t)+2, px, 0, pz)
        arm -= cylx(7.15, t+2, side*ex, 0, 90) & span(*X(ex-4, ex+4), -5.1, 5.1, 80, 100)
        sx, sy_, _ = spring_point(d, key)
        arm -= cylz(4, t+2, sx, sy_, bot-1)
        bushes('drive', key, wy+t, q)
        add('drive_swing_arm_6t', arm, key, True, material='S355 6t laser-cut tray, 3 bends; AF10.2 trial slot; M8 tapped seat',
            arm_length_mm=round(d.p['drive_track']/2-q['pivot_x_mm'], 1))
        # Axle stack: unchanged trial dimensions, carried by the arm end wall.
        spacer = cylx(9, 4.9, side*(ex+5.45), 0, 90)-cylx(7.15, 6, side*(ex+5.45), 0, 90)
        add('drive_axial_spacer_trial', spacer, key, True, fit='Confirm stationary shoulder and bearing-face diameters on motor')
        washer = cylx(10, 1, side*(ex-3.5), 0, 90)-cylx(7.15, 3, side*(ex-3.5), 0, 90)
        add('drive_axle_washer', washer, key, True)
        axle_nut = nut(0, 0, 0, 22, 8, 14.3).rotate(Axis.Y, 90).translate((side*(ex-4)-8*(side > 0), 0, 90))
        add('drive_axle_M14x1p5_nut_envelope', axle_nut, key, True, fit='OEM nut thickness and wrench envelope assumed')
        hinge('drive', key, side*(abs(px)-15), side*(abs(px)+15), wy+t+q['ear_gap_mm'], 83, q)
        spring_stack('drive', key, 16)
        bump_stop('drive', key, side*140, 20, bot+t)

    # ---------------------------------------------------------------- casters
    q = d.layout['suspension']['caster']
    t = q['arm_thickness_mm']
    plate_top = d.parts['caster']['cad_height']
    for wheel in CASTER_WHEELS:
        x, y = caster_xy(d, wheel)
        side = -1 if x < 0 else 1
        (px, _, pz), _ = pivot_of(d, wheel)
        hw = q['arm_half_width_mm']
        x0, x1 = sorted((px-side*12, x+side*35))
        arm = span(x0, x1, y-hw, y+hw, plate_top, plate_top+t)
        t0, t1 = sorted((px-side*12, px+side*25))
        for sy in (-1, 1):
            y0, y1 = sorted((y+sy*(hw-t), y+sy*hw))
            arm += span(t0, t1, y0, y1, plate_top+t, q['tab_top_z_mm'])
        arm -= cyly(q['bush_od_mm']/2, 2*hw+2, px, y, pz)
        bolts = [(x+dx, y+dy) for dx in (-12, 12) for dy in (-27, 27)]
        arm = drilled(arm, bolts, 6.6, plate_top, t)
        sx, sy_, _ = spring_point(d, wheel)
        arm -= cylz(4, t+2, sx, sy_, plate_top-1)
        bushes('caster', wheel, hw, q)
        add('caster_swing_arm_6t', arm, wheel, True, material='S355 6t laser-cut, 2 bent tabs; M8 tapped seat',
            arm_length_mm=q['arm_length_mm'])
        add('caster_plate_bolts', Compound([screw(a, b, plate_top-5, 6, 16, True) for a, b in bolts]),
            wheel, True, sku='CB6-16', quantity=4, plate_thickness_reference_mm=5)
        add('caster_plate_nuts', Compound([nut(a, b, plate_top+t, 10, 5, 6) for a, b in bolts]), wheel, True, quantity=4)
        hinge('caster', wheel, px-15, px+15, hw+q['ear_gap_mm'], 88, q)
        spring_stack('caster', wheel, 16)
        bump_stop('caster', wheel, *bump_stop_xy(d, wheel), plate_top+t)
    return records
