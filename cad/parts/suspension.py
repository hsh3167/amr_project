"""Suspension. Rev15 (concept 'bogie'): front caster + drive wheel on one 3t bent bogie link per side
(pivot axis +X; Rev16: two BGHK6001DD-30 bearing holders on a 12x35 shoulder bolt); rear casters keep the hinge
modules below.

Rev12a/12b/14 (concept 'hinge'): six independent hinge + spring modules (pivot axis +Y).

Per wheel: one bent-sheet bracket under the base plate, one bent arm, one catalog
hinge pin, one X-SWU spring between the arm and the base plate underside on an
M8 stud. The stud's top nut is the droop stop and an M6 bolt through the base
plate is the bump stop. No welding, rails, hangers or bearings. Earlier concepts
(Rev09 shafts/rockers, Rev10 swing arm + seesaw, Rev11 rails) are in git history.
Dimensions in mm.
"""
import math
from math import cos, pi, sin
from build123d import Axis, Circle, Compound, Face, Helix, Plane, Rectangle, Solid, Vector, Wire, sweep
from ..caster_motion import bogie_stack, CASTER_WHEELS, caster_xy, hinge_wheels, is_bogie, motion_angle, pivot_of, pose_shape, rotate_point, rotate_yz, seat_travel
from .common import PartRecord, cylx, cyly, cylz, drilled, polygon_plate, span
from .vendor import misumi_solid

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
    """Spring at its posed length. Rev12b: rectangular wire section and turn count
    taken from the MISUMI STEP (radial x axial, pitch at free length); ground end
    coils are not modelled. Rate and solid length stay catalog values."""
    radial = (q['spring_od_mm']-q['spring_id_mm'])/2
    axial = q.get('spring_wire_axial_mm', radial)
    path = Helix((height-axial)/q['display_turns'], height-axial, (q['spring_od_mm']-radial)/2)
    if 'spring_wire_axial_mm' in q:
        section = Plane(origin=path@0, x_dir=(1, 0, 0), z_dir=path % 0)*Rectangle(radial, axial)
    else:
        section = Plane(origin=path@0, z_dir=path % 0)*Circle(radial/2)
    return sweep(section, path=path, is_frenet=True).translate((x, y, bottom+axial/2))


def bore_chamfers(x, z, faces, radius, size=.5):
    """0.5x45 deg entry chamfers on a +Y bore: clears the bush sleeve-to-flange fillet (MISUMI STEP R0.3)."""
    cones = []
    for y, inward in faces:
        plane = Plane(origin=(x, y-inward*.1, z), z_dir=(0, inward, 0))
        cones.append(Solid.make_cone(radius+size+.1, radius, size+.1, plane))
    return cones


def spring_point(d, key):
    """(x, y, seat z) of the spring on the moving arm."""
    if key in ('LEFT', 'RIGHT'):
        q = d.layout['suspension']['drive']
        side = -1 if key == 'LEFT' else 1
        return side*q['spring_x_mm'], q['spring_y_mm'], q['arm_bottom_z_mm']+q['arm_thickness_mm']
    q = d.layout['suspension']['caster']
    x, y = caster_xy(d, key)
    side = -1 if x < 0 else 1
    # Rev14: 3t floor + fixed seat spacer keeps the Rev12 spring seat height.
    return x-side*q['spring_offset_mm'], y, d.parts['caster']['cad_height']+q['arm_thickness_mm']+q.get('seat_spacer_mm', 0)


def lever_ratio(d, kind):
    """Spring travel / wheel travel (also wheel force / spring force)."""
    key = 'RIGHT' if kind == 'drive' else 'RB'
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
        ys = d.layout['suspension']['drive']['bracket_hole_y_mm']
        return [(px+dx, py+dy) for dx in (-7, 7) for dy in ys]
    return [(px, py+dy) for dy in (-15, 15)]


def bump_stop_xy(d, key):
    if key in ('LEFT', 'RIGHT'):
        q = d.layout['suspension']['drive']
        return (q['bump_stop_x_mm'] if key == 'RIGHT' else -q['bump_stop_x_mm']), q['bump_stop_y_mm']
    x, y = caster_xy(d, key)
    off = d.layout['suspension']['caster']['bump_stop_offset_mm']
    return x-(off if x > 0 else -off), y


def stud_hole(d, key):
    return 13 if key in ('LEFT', 'RIGHT') else 12


def chassis_mount_holes(d):
    holes = []
    for key in hinge_wheels(d):
        holes.extend((a, b, 5.5) for a, b in bracket_holes(d, key))
        x, y, _ = spring_point(d, key)
        holes.append((x, y, stud_hole(d, key)))
        holes.append((*bump_stop_xy(d, key), 6.5))
    if is_bogie(d):
        q = d.layout['suspension']['bogie']
        for s in (-1, 1):
            holes.extend((s*x, y, q['holder_d1_mm']) for x, y in holder_bolt_xy(d))
            cx, cy = q['motor_cable_hole_xy_mm']
            holes.append((s*cx, cy, q['motor_cable_hole_d_mm']))
            holes.extend((s*q[k][0], q[k][1], 6.5) for k in ('stopper_front_xy_mm', 'stopper_rear_xy_mm'))
    return holes


def holder_bolt_xy(d):
    """Right-side holder flange bolts (2 per BGHK, pitch L1 along Y)."""
    q = d.layout['suspension']['bogie']; st = bogie_stack(d)
    return [(sum(st[k])/2, q['pivot_y_mm']+s*q['holder_L1_mm']/2) for k in ('holder_out', 'holder_in') if k in st for s in (-1, 1)]


def yz_prism(points, x0, x1):
    """Plate in the YZ plane from a (y, z) outline, extruded from x0 to x1."""
    face = Face(Wire.make_polygon([Vector(x0, y, z) for y, z in points], close=True))
    return Solid.extrude(face, Vector(x1-x0, 0, 0))


def nut_x(x0, y, z, across_flats, height, diameter):
    """Hex nut on an X axis, from x0 to x0+height."""
    return nut(0, 0, 0, across_flats, height, diameter).rotate(Axis.Y, 90).translate((x0, y, z))


def ring_x(x0, length, y, z, od, bore):
    return cylx(od/2, length, x0+length/2, y, z)-cylx(bore/2, length+2, x0+length/2, y, z)


def top_z(profile, y):
    for (y0, z0), (y1, z1) in zip(profile, profile[1:]):
        if y0 <= y <= y1:
            return z0+(z1-z0)*(y-y0)/(y1-y0)
    raise ValueError('Outside the bogie wall profile')


def bogie_records(d, add, base_bottom, base_top):
    """Rev15: one bogie per side, built for the right side (+X) and mirrored for the left.

    Pure bogie: drive wheel at Y0 and front caster at Y caster_y on one 3t bent link (outer wall = axle wall,
    rear floor + rear inner wall, front caster pad + front inner wall; 4 bends, no welding). Rev16 pivot: two
    company-stock BGHK6001DD-30 bearing holders hang from the base plate; a company-stock 12x35 shoulder bolt runs
    through both bearings and is clamped (E-GLNUTS10) through the two ears of a 3t U bracket bolted on the bogie
    floor. Two company-stock C-USLH10 stoppers on the floor meet M6 strikers and limit the rotation.
    """
    q = d.layout['suspension']['bogie']
    t = q['sheet_t_mm']
    yp, zp = q['pivot_y_mm'], q['pivot_z_mm']
    xo, xi = q['outer_wall_x_mm'], q['inner_wall_x_mm']
    yr, yf = q['rear_y_mm'], q['front_y_mm']
    fz = q['rear_floor_top_z_mm']
    fe = q['rear_floor_end_y_mm']
    ps = q['pad_start_y_mm']
    pad = d.parts['caster']['cad_height']  # pad underside on the GL-65 plate
    prof = [tuple(v) for v in q['wall_top_profile_yz']]
    st = bogie_stack(d)
    cy = d.p['caster_y']
    half = q['travel_mm']/2

    def right():
        out = []
        # ---- bogie link (one bent part)
        outer = yz_prism([(yr, fz-t), (fe, fz-t), (q['outer_wall_step_end_y_mm'], pad), (yf, pad)]+prof[::-1], xo, xo+t)
        inner_top = top_z(prof, fe)
        inner = yz_prism([(yr, fz-t), (fe, fz-t), (fe, inner_top)]+[v for v in prof[::-1] if v[0] < fe], xi, xi+t)
        link = outer+inner+span(xi, xo+t, yr, fe, fz-t, fz)
        link += span(xi, xo+t, ps, yf, pad, pad+t)+span(xi, xi+t, ps, yf, pad+t, q['front_inner_wall_top_z_mm'])
        slot = cylx(7.15, 30, xo, 0, 90) & span(xo-12, xo+t+5, -5.1, 5.1, 80, 100)
        link -= slot
        # Access holes on the pivot axis: bolt insertion + hex key (outer wall, before the wheel goes on),
        # socket for the lock nut (inner wall).
        link -= cylx(q['outer_wall_access_hole_d_mm']/2, 2*t, xo+t/2, yp, zp)
        link -= cylx(q['inner_wall_access_hole_d_mm']/2, 2*t, xi+t/2, yp, zp)
        stoppers = [tuple(q['stopper_front_xy_mm']), tuple(q['stopper_rear_xy_mm'])]
        for x, y in stoppers:
            link -= cylz(2.5, t+2, x, y, fz-t-1)
        bolts = [(d.p['caster_x']+dx, cy+dy) for dx in (-12, 12) for dy in (-27, 27)]
        link = drilled(link, bolts, 6.6, pad, t)
        out.append(('bogie_link_3t', link, True, {'material': 'S355 3t laser-cut, 4 bends (outer wall + rear floor + rear inner wall, '
                    'front caster pad + front inner wall); AF10.2 axle slot; D22/D30 pivot access holes', 'length_mm': yf-yr}))
        # Flat 3t splice plate bolted across the rear and front inner walls (they come off different flaps).
        sy0, sy1 = q['splice_y_mm']; sz0, sz1 = q['splice_z_mm']; zb = (sz0+sz1)/2
        splice = span(xi-t, xi, sy0, sy1, sz0, sz1)
        for y in q['splice_bolt_y_mm']:
            splice -= cylx(2.75, 2*t, xi-t/2, y, zb)
            link -= cylx(2.75, 2*t, xi+t/2, y, zb)
        out.append(('bogie_inner_splice_3t', splice, True, {'material': 'S355 3t flat laser-cut; joins the rear and front inner walls (4x CB5-12)'}))
        sb = Compound([cylx(4.25, 5, xi-t-2.5, y, zb)+cylx(2.5, 12, xi-t+6, y, zb) for y in q['splice_bolt_y_mm']])
        out.append(('bogie_splice_bolts_CB5-12', sb, True, {'sku': 'CB5-12', 'quantity': 4}))
        out.append(('bogie_splice_nuts_M5', Compound([nut_x(xi+t, y, zb, 8, 4, 5) for y in q['splice_bolt_y_mm']]), True, {'quantity': 4}))
        tp = q['torque_plate_t_mm']
        plate = span(xo-tp, xo, -q['torque_plate_half_y_mm'], q['torque_plate_half_y_mm'], fz, top_z(prof, 0)-4)-slot
        out.append(('bogie_axle_torque_plate', plate, True,
                    {'material': f'S355 {tp}t flat laser-cut, AF10.2 slot; clamped by the axle nut against the outer wall'}))
        # Axle stack (Rev14 trial dimensions; the nut moves in by the thicker torque plate).
        out.append(('bogie_axial_spacer_trial', ring_x(xo+t, 4.9, 0, 90, 18, 14.3), True,
                    {'fit': 'Confirm stationary shoulder and bearing-face diameters on motor'}))
        out.append(('bogie_axle_washer', ring_x(xo-tp-1, 1, 0, 90, 20, 14.3), True, {}))
        out.append(('bogie_axle_M14x1p5_nut_envelope', nut_x(xo-tp-9, 0, 90, 22, 8, 14.3), True,
                    {'fit': 'OEM nut thickness and wrench envelope assumed'}))
        # ---- caster pad hardware (as Rev14: head under the 2 mm OEM plate, nut on the pad)
        under = pad-d.parts['caster']['plate_thickness_mm']
        out.append(('bogie_caster_plate_bolts', Compound([screw(a, b, under, 6, 12, True) for a, b in bolts]), True,
                    {'sku': 'CB6-12', 'quantity': 4, 'plate_thickness_mm': d.parts['caster']['plate_thickness_mm']}))
        out.append(('bogie_caster_plate_nuts', Compound([nut(a, b, pad+t, 10, 5, 6) for a, b in bolts]), True, {'quantity': 4}))
        # ---- Rev16 pivot. Moving side (turns with the link): U bracket on the floor, shoulder bolt, collars, nut.
        bt = q['bracket_t_mm']; hy, wy = q['bracket_half_y_mm'], q['bracket_window_half_y_mm']
        eo, ei = st['ear_out'], st['ear_in']
        bracket = span(ei[0], eo[1], yp-hy, yp+hy, fz, fz+bt)-span(ei[1], eo[0], yp-wy, yp+wy, fz-1, fz+bt+1)
        for x0, x1 in (ei, eo):
            bracket += span(x0, x1, yp-hy, yp+hy, fz+bt, q['bracket_ear_top_z_mm'])
        bracket -= cylx(q['bolt_shoulder_d_mm']/2+.2, bt+2, sum(eo)/2, yp, zp)
        bracket -= cylx(5.25, bt+2, sum(ei)/2, yp, zp)
        ub = [(x, yp+s*q['bracket_bolt_dy_mm']) for x in q['bracket_bolt_x_mm'] for s in (-1, 1)]
        bracket = drilled(bracket, ub, 5.5, fz, bt)
        link = drilled(link, ub, 5.5, fz-t, t)
        out[0] = (out[0][0], link, *out[0][2:])
        out.append(('bogie_pivot_U_bracket_3t', bracket, True,
                    {'material': 'S355 3t laser-cut, 2 bends; window under the bearing holders; D12.2 / D10.5 ear holes; 4x CB5-12 to the floor'}))
        out.append(('bogie_U_bracket_bolts_CB5-12', Compound([screw(a, b, fz+bt, 5, 12) for a, b in ub]), True, {'sku': 'CB5-12', 'quantity': 4}))
        out.append(('bogie_U_bracket_nuts_M5', Compound([nut(a, b, fz-t-4, 8, 4, 5) for a, b in ub]), True, {'quantity': 4}))
        h0, h1 = st['head']; send = st['shoulder_end']
        bolt = (cylx(q['bolt_head_d_mm']/2, h1-h0, (h0+h1)/2, yp, zp)
                + cylx(q['bolt_shoulder_d_mm']/2, h0-send, (h0+send)/2, yp, zp)
                + cylx(5, send-st['thread_tip'], (send+st['thread_tip'])/2, yp, zp))
        bolt -= cylx(3.0, 6, h1-3, yp, zp)  # hex socket 6 (as a round recess)
        out.append((f"bogie_pivot_shoulder_bolt_12x{q['bolt_shoulder_l_mm']:g}", bolt, True,
                    {'sku': q['bolt_sku'], 'supply': 'company stock', 'status': 'catalog_envelope',
                     'source': f"ISO 7379 12x{q['bolt_shoulder_l_mm']:g}: head D18 x 8, shoulder D12 f9 x {q['bolt_shoulder_l_mm']:g}, M10 x 16", 'note': q['bolt_note'],
                     'fit': 'Pivot axle: inner rings and collars clamped between the outboard ear and the inboard ear'}))
        od, idd = q['spacer_od_mm'], q['spacer_id_mm']
        ck = [k for k in ('spacer_out', 'spacer_mid', 'spacer_in') if k in st]
        collars = [ring_x(st[k][0], st[k][1]-st[k][0], yp, zp, od, idd) for k in ck]
        out.append(('bogie_pivot_collars', Compound(collars), True,
                    {'quantity': len(ck), 'lengths_mm': [round(st[k][1]-st[k][0], 2) for k in ck],
                     'status': 'catalog_envelope', 'note': q['spacer_note']}))
        out.append(('bogie_pivot_nut_'+q['nut_sku'], nut_x(st['nut'][0], yp, zp, q['nut_af_mm'], q['nut_h_mm'], 10), True,
                    {'sku': q['nut_sku'], 'supply': 'company stock', 'status': 'catalog_table_model',
                     'fit': 'M10 lock nut (spring-ring type), 31 N m catalogue torque; hold the bolt with a 6 mm hex key'}))
        # Fixed side: two BGHK6001DD-30 (supplied MISUMI STEP, unscaled) hung flange-up under the base plate.
        # STEP frame: X = width L, Y = height from the flange underside (bore Y30), Z = thickness, lip face Z-15,
        # snap-ring mouth Z0. Lip faces point outward (outboard / inboard), so the bearings sit 2 mm from the outer faces.
        S = q['holder_S_mm']
        assert abs(base_bottom-q['holder_h_mm']-zp) < 1e-6, 'BGHK centre height must put the bore on the pivot axis'
        parts = {k: [] for k in ('Housing', 'Bearing', 'Ring')}
        from .vendor import source as vendor_source
        if q.get('holder_count', 2) == 1:
            # Rev17d: one BGMW6001DD-30 (supplied MISUMI STEP, unscaled). STEP frame: X = width, Y = height from the
            # flange underside (bore Y30), Z = thickness 0..23 with the 2 mm lip at Z0 -> lip outboard at holder_out[1].
            places = [(90, st['holder_out'][1])]
        else:
            # BGHK6001DD-30: lip face Z-15, mouth Z0; lips face outward on both holders.
            places = [(90, st['holder_out'][0]), (-90, st['holder_in'][1])]
        for turn, x_ref in places:
            for c in vendor_source(q['holder_step']).children:
                k = 'Housing' if c.label.startswith('Housing') else 'Bearing' if c.label.startswith('Bearing') else 'Ring'
                for solid in c.solids():
                    parts[k].append(solid.rotate(Axis.X, -90).rotate(Axis.Z, turn).translate((x_ref, yp, base_bottom)))
        nh = len(places); nb = len(parts['Bearing'])
        meta = {'sku': q['holder_sku'], 'supply': 'company stock', 'status': 'misumi_step_exact', 'source': q['holder_step']+'.gz'}
        out.append(('bogie_pivot_holders_'+q['holder_sku'], Compound(parts['Housing']), False,
                    {**meta, 'quantity': nh, 'fit': 'Hung flange-up under the base plate, 2x M5 each; 2 mm lip faces outboard'}))
        out.append(('bogie_pivot_bearings_6001DD', Compound(parts['Bearing']), False,
                    {**meta, 'quantity': nb, 'note': 'Bearing solids of the MISUMI STEP (6001DD, d12)'}))
        out.append(('bogie_pivot_snap_rings', Compound(parts['Ring']), False, {**meta, 'quantity': nh}))
        bx0, bx1 = q['backing_x_mm']; by0, by1 = q['backing_y_mm']; bkt = q['backing_t_mm']
        hb = holder_bolt_xy(d)
        backing = drilled(span(bx0, bx1, by0, by1, base_top, base_top+bkt), hb, 5.5, base_top, bkt)
        out.append(('bogie_pivot_backing_plate_3t', backing, False,
                    {'material': 'S355 3t flat laser-cut; spreads the bearing-pair couple into the 3t base plate'}))
        blen = round(bkt+(base_top-base_bottom)+S+4+2)
        out.append((f'bogie_holder_bolts_CB5-{blen}', Compound([screw(a, b, base_top+bkt, 5, blen) for a, b in hb]), False,
                    {'sku': f'CB5-{blen}', 'quantity': 4}))
        out.append(('bogie_holder_nuts_M5', Compound([nut(a, b, base_bottom-S-4, 8, 4, 5) for a, b in hb]), False, {'quantity': 4}))
        mcx, mcy = q['motor_cable_hole_xy_mm']; cd = q['motor_cable_hole_d_mm']
        grommet = (cylz(cd/2, base_top-base_bottom, mcx, mcy, base_bottom)+cylz(cd/2+3, 1.5, mcx, mcy, base_top)
                   + cylz(cd/2+3, 1.5, mcx, mcy, base_bottom-1.5))-cylz(10, 20, mcx, mcy, base_bottom-5)
        out.append(('motor_cable_grommet_D28', grommet, False,
                    {'status': 'catalog_envelope', 'note': q['motor_cable_note'], 'fit': 'Rubber grommet for a 28 mm hole in 3t plate, 20 mm bore'}))
        # ---- rotation limits: stopper on the floor, M6 striker from the base plate (contact exactly at the limit)
        for (x, y), limit, tag in ((stoppers[0], half, 'front'), (stoppers[1], -half, 'rear')):
            parts = [misumi_solid(q['bump_stopper_step'], c).rotate(Axis.Y, 90).translate((x, y, fz))
                     for c in q['bump_stopper_children']]
            top = max(s.bounding_box().max.Z for s in parts)
            out.append((f"bogie_{tag}_stopper_{q['bump_stopper_sku']}", Compound(parts), True,
                        {'sku': q['bump_stopper_sku'], 'supply': 'company stock', 'status': 'misumi_step_exact',
                         'source': q['bump_stopper_step']+'.gz', 'color': (.35, .78, .45), 'tip_z': round(top, 3),
                         'fit': 'M5x0.8 thread 7: 3t floor + M5 nut 4 underneath'}))
            out.append((f'bogie_{tag}_stopper_nut_M5', nut(x, y, fz-t-4, 8, 4, 5), True, {'quantity': 1}))
            r = q['bump_stopper_d_mm']/2
            tip = max(rotate_yz(d, 'RIGHT', limit, (y+dy, top))[1] for dy in (-r, r))+.05
            out.append((f'bogie_{tag}_striker_M6', screw(x, y, base_top, 6, base_top-tip)+nut(x, y, base_bottom-5, 10, 5, 6), False,
                        {'sku': 'M6 hex bolt + jam nut', 'tip_z': round(tip, 3), 'limit_mm': limit,
                         'adjust': 'Set the bogie rotation limit with the jam nut or washers'}))
        return out

    sp = q.get('spring')
    for side in (1, -1):
        key = 'RIGHT' if side > 0 else 'LEFT'
        for name, shape, moving, meta in right():
            if side < 0:
                shape = shape.mirror(Plane.YZ)
            add(name, shape, key, moving, **meta)
        if sp:
            # Rev17 bogie spring: coaxial with the rear stopper, from the (posed) rear floor up to the base plate underside.
            x, y = q[sp['location']]
            ys, zs = rotate_yz(d, key, d.motion.get(key, 0), (y, fz))
            # Top end stays centred on the fixed striker (x, y); the bottom end follows the posed floor (ys, zs),
            # so the coil leans by a small angle (about 1.6 deg at the stroke ends) instead of shifting sideways.
            length = math.hypot(base_bottom-zs, ys-y)
            sst = bogie_spring_state(d)
            lean = math.degrees(math.asin((ys-y)/length))
            floor_tilt = math.degrees(motion_angle(d, key, d.motion.get(key, 0)))
            # Ground end coils seat flat; the drawn rings are shortened so a tilted end ring edge does not cut the plate/floor.
            e = sp['od_mm']/2*math.sin(math.radians(max(abs(lean), abs(floor_tilt-lean))))+.05 if lean else 0
            shape = coil(x, y, base_bottom-length+e, length-2*e, {'spring_od_mm': sp['od_mm'], 'spring_id_mm': sp['id_mm'],
                                                                  'spring_wire_axial_mm': sp['wire_axial_mm'], 'display_turns': sp['display_turns']})
            if abs(lean) > 1e-9:
                shape = shape.rotate(Axis((0, y, base_bottom), (1, 0, 0)), lean)
            if side < 0:
                shape = shape.mirror(Plane.YZ)
            add('bogie_spring_'+sp['sku'], shape, key, False, sku=sp['sku'], status='catalog_envelope', color=(.86, .30, .25),
                spring_top_z=base_bottom, spring_length_mm=round(length, 3), spring_xy=[side*x, y], lean_deg=round(lean, 3),
                rate_N_mm=sp['rate_N_mm'], free_length_mm=sp['free_length_mm'],
                force_N=round(sp['rate_N_mm']*(sp['free_length_mm']-length), 2), nominal_force_N=sst['forces_N'][1],
                geometry='Coil drawn at the posed length (SWU wire section, display turns); rate and allowable deflection from the catalogue',
                fit='Sits on the rear bogie floor around the C-USLH10; top end around the M6 striker jam nut under the base plate')


def bogie_spring_state(d):
    """Rev17 bogie spring between the rear bogie floor and the base plate underside, at the front-caster stroke ends and nominal."""
    q = d.layout['suspension']['bogie']; sp = q['spring']; half = q['travel_mm']/2
    base_bottom = d.p['base_bottom_z']; x, y = q[sp['location']]
    lengths = [round(base_bottom-rotate_yz(d, 'RIGHT', v, (y, q['rear_floor_top_z_mm']))[1], 3) for v in (half, 0, -half)]
    forces = [round(sp['rate_N_mm']*max(0, sp['free_length_mm']-L), 2) for L in lengths]
    return {'sku': sp['sku'], 'xy_mm': [x, y], 'arm_behind_pivot_mm': q['pivot_y_mm']-y, 'rate_N_mm': sp['rate_N_mm'],
            'free_length_mm': sp['free_length_mm'], 'poses_mm': [half, 0, -half], 'lengths_mm': lengths, 'forces_N': forces,
            'min_length_margin_to_allowable_mm': round(min(lengths)-(sp['free_length_mm']-sp['max_deflection_mm']), 3),
            'preload_at_full_rebound_N': forces[0]}


def boss_plates(xs, pz, ys):
    """Flat 8t bosses inside the side walls: give the MPFZ sleeve (8 mm) its press length behind a 3t wall."""
    return Compound([span(xs[0], xs[1], y0, y1, pz-10, pz+10) for y0, y1 in ys])


def boss_screws(px, pz, faces, q):
    """CB4-10 from outside the wall into each boss, clear of the bracket ear (px +-12), toward the wheel."""
    dx = q['boss_screw_dx_mm']
    s = 1 if px > 0 else -1
    return [(px+s*dx, face, pz+dz, out) for face, out in faces for dz in (-6, 6)]


def screw_y(x, face, z, out, q):
    """CB4-10 along Y: head (D7 x 4) outside the wall face, 10 mm shank into wall + boss."""
    head = cyly(3.5, 4, x, face+out*2, z)
    shank = cyly(2, 10, x, face-out*5, z)
    return head+shank


def make(d):
    records = []
    p = d.p
    base_bottom = p['base_bottom_z']
    base_top = base_bottom+p['base_thickness']

    def add(name, shape, wheel, moving=False, sku=None, **metadata):
        if moving:
            shape = pose_shape(d, shape, wheel, d.motion.get(wheel, 0))
        color = metadata.pop('color', (.77, .64, .36) if 'pin' in name else (.48, .65, .78))
        status = metadata.pop('status', 'misumi_catalog_envelope' if sku else 'suspension_fabrication_design')
        records.append(PartRecord('80_'+name+'_'+wheel, 'suspension', shape, color, status,
                                  {'wheel': wheel, 'moving_wheel': wheel if moving else None,
                                   **({'sku': sku} if sku else {}), **metadata}))

    def hinge(kind, key, x0, x1, ear_lo, ear_hi, ear_bottom, q):
        """Fixed U bracket under the base plate, hinge pin and retainer.

        ear_lo / ear_hi: distance from the pivot (Y) to the inner face of the -Y / +Y ear.
        """
        (px, py, pz), _ = pivot_of(d, key)
        bt = q['bracket_thickness_mm']
        lo, hi = py-ear_lo-bt, py+ear_hi+bt
        bracket = span(min(x0, x1), max(x0, x1), lo, hi, base_bottom-bt, base_bottom)
        for y0, y1 in ((lo, lo+bt), (hi-bt, hi)):
            bracket += span(px-12, px+12, y0, y1, ear_bottom, base_bottom-bt)
        bracket -= cyly(q['pin_d_mm']/2+.005, hi-lo+2, px, (lo+hi)/2, pz)
        holes = bracket_holes(d, key)
        bracket = drilled(bracket, holes, 5.5, base_bottom-bt, bt)
        add(kind+'_hinge_bracket_U', bracket, key, material=f'S355 {bt}t sheet, 2 bends (bend limit 3t); ream both ear holes together')
        # Rev14: base 3 + flange 3 + nut 4 -> CB5-12 ends 2 mm past the nut.
        add(kind+'_bracket_bolts', Compound([screw(a, b, base_top, 5, 12) for a, b in holes]), key,
            sku='CB5-12', quantity=len(holes))
        add(kind+'_bracket_nuts', Compound([nut(a, b, base_bottom-bt-4, 8, 4, 5) for a, b in holes]), key,
            quantity=len(holes))
        length = round(hi-lo+1, 1)
        # Rev12b: the supplied MISUMI STEP (axis +X, head underside at X0) placed on +Y.
        pin = misumi_solid(q['pin_step'])
        if abs(pin.bounding_box().max.X-length) > 1e-6:
            raise ValueError('Pin STEP length differs from the bracket span: '+key)
        pin = pin.rotate(Axis.Z, 90).translate((px, lo, pz))
        add(kind+'_hinge_pin', pin, key, sku=q['pin_sku'], length_mm=length, status='misumi_step_exact',
            source=q['pin_step']+'.gz', fit='Fixed in ear holes (H7), arm turns on it; grease. M6 tapped end, depth ~18.6')
        end = lo+length
        tap = q['pin_tap_minor_d_mm']
        keeper = cyly(8, 1, px, end+.5, pz)-cyly(3.2, 3, px, end+.5, pz)
        # Thread engagement inside the pin is drawn at the tap minor diameter.
        keeper += cyly(tap/2, 7, px, end-3.5, pz)+cyly(3, 1, px, end+.5, pz)+cyly(5, 6, px, end+4, pz)
        add(kind+'_pin_retainer', keeper, key, sku='CB6-8', washer='M6 OD16 t1',
            thread='engaged length 7 shown at the tap minor diameter')

    def bushes(kind, key, outer_lo, outer_hi, q):
        """Two flanged oil-free bushes pressed into the arm from outside; flanges face the bracket ears."""
        (px, py, pz), _ = pivot_of(d, key)
        source = misumi_solid(q['bush_step'])  # axis +X, flange X-2..0, sleeve X0..8
        parts = []
        for sy, outer in ((-1, outer_lo), (1, outer_hi)):
            face = py+sy*outer
            parts.append(source.rotate(Axis.Z, 90 if sy < 0 else -90).translate((px, face, pz)))
        add(kind+'_hinge_bushes', Compound(parts), key, True, sku=q['bush_sku'], quantity=2, status='misumi_step_exact',
            source=q['bush_step']+'.gz',
            fit='Press into 14 H7 arm bore (m6 OD); runs on the g6 pin (F7 bore, STEP nominal 10.0); flange = thrust washer')

    def spring_stack(kind, key, washer_od):
        q = d.layout['suspension'][kind]
        half = q['travel_mm']/2
        x, y, seat = spring_point(d, key)
        state = spring_state(d, kind)
        if abs(base_bottom-seat-q['loaded_length_mm']) > 1e-6:
            raise ValueError('Spring gap and catalog seating length differ: '+key)
        value = d.motion.get(key, 0)
        bottom = seat+q['shim_mm']
        spacer = q.get('seat_spacer_mm', 0)
        floor = seat-spacer-q['arm_thickness_mm']
        if spacer:
            add(kind+'_seat_spacer', ring(x, y, seat-spacer, spacer, q['spring_od_mm']+1, 8.4), key, True,
                material=f'{spacer}t flat washer/plate (fixed)')
        if q['shim_mm'] > 0:   # Rev17: the rear caster shim is 0 (bogie springs unload the rear casters)
            add(kind+'_preload_shim', ring(x, y, seat, q['shim_mm'], q['spring_od_mm']+1, 8.4), key, True,
                remove_without_arm=True)
        length = state['installed_length_mm']
        r = q['spring_od_mm']/2
        dz = max(seat_travel(d, key, value, x+dx, bottom) for dx in (-r, r))  # tilted seat: lift to its high edge
        add(kind+'_spring_coil', coil(x, y, bottom+dz, length-dz, q), key, sku=q['spring_sku'],
            spring_top_z=bottom+length, spring_length_mm=length, spring_xy=[x, y], rate_N_mm=q['rate_N_mm'],
            free_length_mm=q['free_length_mm'], nominal_spring_force_N=state['forces_N'][1],
            lever_ratio=state['lever_ratio'], geometry='MISUMI STEP wire section and pitch; ground ends omitted; rate and solid length from catalog')
        # Washer and nut on the stud land on the base plate top at full droop.
        w = washer_od/2
        drop = min(seat_travel(d, key, -half, x+dx, base_top) for dx in (-w, w))
        washer_bottom = base_top-drop+.05
        nut_top = washer_bottom+1.6+6.5
        stud = cylz(4, nut_top+1.5-floor, x, y, floor)
        add(kind+'_stud_M8', stud, key, True, sku='M8 stud', length_mm=round(nut_top+1.5-floor, 1),
            fit='Tapped M8 in the 3t floor + threadlocker; droop load below 100 N')
        stop = ring(x, y, washer_bottom, 1.6, washer_od, 8.4)+nut(x, y, washer_bottom+1.6, 13, 6.5, 8)
        add(kind+'_droop_stop_washer_nut', stop, key, True, adjust='Turn the nut to set the droop limit; lock with a second nut')
        return state

    def bump_stop(kind, key, x, y, surface):
        """Rev12b: MISUMI C-USLH10 urethane stopper screwed into the arm (M5 tap);
        an M6 striker bolt through the base plate meets its urethane tip at full bump."""
        q = d.layout['suspension'][kind]
        half = q['travel_mm']/2
        step = q['bump_stopper_step']
        # Supplied STEP: axis +X, seat face X0, urethane tip X-17, M5 thread X0..7 -> tip up, thread into the arm.
        parts = [misumi_solid(step, child).rotate(Axis.Y, 90).translate((x, y, surface))
                 for child in q['bump_stopper_children']]
        top = max(s.bounding_box().max.Z for s in parts)
        add(kind+'_bump_stopper_'+q['bump_stopper_sku'], Compound(parts), key, True, sku=q['bump_stopper_sku'],
            status='misumi_step_exact', source=step+'.gz', color=(.35, .78, .45), tip_z=round(top, 3),
            fit='M5x0.8 thread 7: 3t floor + M5 nut 4 underneath; urethane Shore A90 tip')
        add(kind+'_bump_stopper_nut_M5', nut(x, y, surface-q['arm_thickness_mm']-4, 8, 4, 5), key, True, quantity=1)
        r = q['bump_stopper_d_mm']/2
        rise = max(seat_travel(d, key, half, x+dx, top) for dx in (-r, r))
        tip = top+rise+.05
        stop = screw(x, y, base_top, 6, base_top-tip)+nut(x, y, base_bottom-5, 10, 5, 6)
        add(kind+'_bump_stop_bolt_M6', stop, key, sku='M6 hex bolt + jam nut', tip_z=round(tip, 3),
            adjust='Striker on the stopper tip; set the bump limit with the jam nut or washers')

    if is_bogie(d):
        bogie_records(d, add, base_bottom, base_top)
    # ---------------------------------------------------------------- drive (hinge concept)
    q = d.layout['suspension']['drive']
    t = q['arm_thickness_mm']
    for side in (() if is_bogie(d) else (-1, 1)):
        key = 'LEFT' if side < 0 else 'RIGHT'
        (px, _, pz), _ = pivot_of(d, key)

        def X(a, b):
            return (side*a, side*b) if side > 0 else (side*b, side*a)

        # Rev14: 3t bent channel (bend limit 3t): floor + two full-length side walls + end wall (3 bends).
        # Flat parts carry the thick features: 8t bosses for the bushes, a 3t torque plate on the axle flats.
        wy, wp = q['arm_wall_y_mm'], q['arm_wall_y_pos_mm']
        top, bot, ex = q['arm_top_z_mm'], q['arm_bottom_z_mm'], q['end_wall_x_mm']
        a0 = abs(px)-12
        arm = span(*X(a0, ex+t), -(wy+t), wp+t, bot, bot+t)
        for y0, y1 in ((-(wy+t), -wy), (wp, wp+t)):
            arm += span(*X(a0, ex+t), y0, y1, bot+t, top)
        arm += span(*X(ex, ex+t), -wy, wp, bot+t, top)
        slot = cylx(7.15, 12, side*(ex+t/2), 0, 90) & span(*X(ex-5, ex+t+5), -5.1, 5.1, 80, 100)
        arm -= slot
        bosses = boss_plates(X(a0, abs(px)+q['pivot_tab_x_mm']), pz, ((-wy, -wy+q['boss_thickness_mm']), (wp-q['boss_thickness_mm'], wp)))
        bore = cyly(q['bush_od_mm']/2, wy+wp+2*t+2, px, (wp-wy)/2, pz)
        arm -= bore
        bosses -= bore
        for cone in bore_chamfers(px, pz, ((-(wy+t), 1), (wp+t, -1)), q['bush_od_mm']/2):
            arm -= cone
        screws = boss_screws(px, pz, ((-(wy+t), -1), (wp+t, 1)), q)
        for s in screws:
            arm -= cyly(2.25, 30, s[0], s[1], s[2])
            bosses -= cyly(2.25, 30, s[0], s[1], s[2])
        sx, sy_, _ = spring_point(d, key)
        arm -= cylz(4, t+2, sx, sy_, bot-1)
        arm -= cylz(q['bump_stopper_thread_d_mm']/2, t+2, *bump_stop_xy(d, key), bot-1)  # M5 for the stopper
        bushes('drive', key, wy+t, wp+t, q)
        add('drive_swing_arm_3t', arm, key, True,
            material='S355 3t laser-cut, floor + 2 full-length side walls + end wall (3 bends); AF10.2 slot; M8 tapped seat',
            arm_length_mm=round(d.p['drive_track']/2-q['pivot_x_mm'], 1))
        add('drive_pivot_bosses_8t', bosses, key, True, material='S355 8t flat laser-cut, bush bore 14 H7, 2x M4 tapped each',
            quantity=2)
        add('drive_boss_screws_CB4-10', Compound([screw_y(*s, q) for s in screws]), key, True, sku='CB4-10', quantity=4)
        plate = span(*X(ex-3, ex), -wy, wp, bot+t, top)-slot
        add('drive_axle_torque_plate_3t', plate, key, True,
            material='S355 3t flat laser-cut, AF10.2 slot; clamped by the axle nut against the end wall')
        # Axle stack: unchanged trial dimensions, carried by the arm end wall.
        spacer = cylx(9, 4.9, side*(ex+5.45), 0, 90)-cylx(7.15, 6, side*(ex+5.45), 0, 90)
        add('drive_axial_spacer_trial', spacer, key, True, fit='Confirm stationary shoulder and bearing-face diameters on motor')
        washer = cylx(10, 1, side*(ex-3.5), 0, 90)-cylx(7.15, 3, side*(ex-3.5), 0, 90)
        add('drive_axle_washer', washer, key, True)
        axle_nut = nut(0, 0, 0, 22, 8, 14.3).rotate(Axis.Y, 90).translate((side*(ex-4)-8*(side > 0), 0, 90))
        add('drive_axle_M14x1p5_nut_envelope', axle_nut, key, True, fit='OEM nut thickness and wrench envelope assumed')
        hinge('drive', key, side*(abs(px)-15), side*(abs(px)+15), wy+t+q['ear_gap_mm'], wp+t+q['ear_gap_mm'], 83, q)
        spring_stack('drive', key, 16)
        bump_stop('drive', key, *bump_stop_xy(d, key), bot+t)

    # ---------------------------------------------------------------- casters
    q = d.layout['suspension']['caster']
    t = q['arm_thickness_mm']
    plate_top = d.parts['caster']['cad_height']
    for wheel in (w for w in CASTER_WHEELS if w in hinge_wheels(d)):
        x, y = caster_xy(d, wheel)
        side = -1 if x < 0 else 1
        (px, _, pz), _ = pivot_of(d, wheel)
        # Rev14: 3t bent channel. Narrow section with side walls from the pivot; wider flat pad over the
        # GL-65 plate so the 54 mm hole pitch keeps 7 mm edge distance (Rev13 holes broke out of the edge).
        hw, pad = q['arm_half_width_mm'], q['caster_pad_half_width_mm']
        x0, x1 = sorted((px-side*12, x+side*35))
        pad_start = x-side*q['caster_pad_inboard_mm']
        n0, n1 = sorted((px-side*12, pad_start))
        p0, p1 = sorted((pad_start, x+side*35))
        arm = span(n0, n1, y-hw, y+hw, plate_top, plate_top+t)+span(p0, p1, y-pad, y+pad, plate_top, plate_top+t)
        w0, w1 = sorted((px-side*12, pad_start-side*3))
        for sy in (-1, 1):
            y0, y1 = sorted((y+sy*(hw-t), y+sy*hw))
            arm += span(w0, w1, y0, y1, plate_top+t, q['tab_top_z_mm'])
        b0, b1 = sorted((px-side*12, px+side*q['pivot_tab_x_mm']))
        bosses = boss_plates((b0, b1), pz, ((y-hw+t, y-hw+t+q['boss_thickness_mm']), (y+hw-t-q['boss_thickness_mm'], y+hw-t)))
        bore = cyly(q['bush_od_mm']/2, 2*hw+2, px, y, pz)
        arm -= bore
        bosses -= bore
        for cone in bore_chamfers(px, pz, ((y-hw, 1), (y+hw, -1)), q['bush_od_mm']/2):
            arm -= cone
        screws = boss_screws(px, pz, ((y-hw, -1), (y+hw, 1)), q)
        for s in screws:
            arm -= cyly(2.25, 30, s[0], s[1], s[2])
            bosses -= cyly(2.25, 30, s[0], s[1], s[2])
        bolts = [(x+dx, y+dy) for dx in (-12, 12) for dy in (-27, 27)]
        arm = drilled(arm, bolts, 6.6, plate_top, t)
        sx, sy_, _ = spring_point(d, wheel)
        arm -= cylz(4, t+2, sx, sy_, plate_top-1)
        arm -= cylz(q['bump_stopper_thread_d_mm']/2, t+2, *bump_stop_xy(d, wheel), plate_top-1)  # M5 for the stopper
        bushes('caster', wheel, hw, hw, q)
        add('caster_swing_arm_3t', arm, wheel, True,
            material='S355 3t laser-cut, narrow floor + 2 side walls (2 bends) + flat caster pad; M8 tapped seat',
            arm_length_mm=q['arm_length_mm'])
        add('caster_pivot_bosses_8t', bosses, wheel, True, material='S355 8t flat laser-cut, bush bore 14 H7, 2x M4 tapped each',
            quantity=2)
        add('caster_boss_screws_CB4-10', Compound([screw_y(*s, q) for s in screws]), wheel, True, sku='CB4-10', quantity=4)
        # Heads bear on the 2 mm OEM plate; plate 2 + floor 3 + nut 5 -> CB6-12 ends 2 mm past the nut.
        under = plate_top-d.parts['caster']['plate_thickness_mm']
        add('caster_plate_bolts', Compound([screw(a, b, under, 6, 12, True) for a, b in bolts]),
            wheel, True, sku='CB6-12', quantity=4, plate_thickness_mm=d.parts['caster']['plate_thickness_mm'])
        add('caster_plate_nuts', Compound([nut(a, b, plate_top+t, 10, 5, 6) for a, b in bolts]), wheel, True, quantity=4)
        hinge('caster', wheel, px-15, px+15, hw+q['ear_gap_mm'], hw+q['ear_gap_mm'], 88, q)
        spring_stack('caster', wheel, 16)
        bump_stop('caster', wheel, *bump_stop_xy(d, wheel), plate_top+t)
    return records
