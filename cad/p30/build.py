"""P30 chassis: 30x30 Al profile frame, vertical-slide drive suspension, rigid casters.

Reuses the Rev15 builders for purchased/company geometry (GL-65 OEM STEP, Astra camera STEP and cache,
company battery / LCD / speaker STEP, MISUMI C-USLH10 and X-SWU spring data, screw / nut / coil helpers).
Everything specific to P30 (frame, plates, drive module, brackets) is built here from cad/p30/layout.json.
Moving drive parts carry metadata moving_wheel=LEFT/RIGHT and move by a pure Z translation.
"""
import json
import math
from pathlib import Path
from build123d import Axis, Compound, Location, Rotation, Solid, Plane
from ..parameters import ROOT
from ..parts.common import PartRecord, box, cylx, cyly, cylz, span, polygon_plate, rz
from ..parts.suspension import coil, nut, screw

HERE = Path(__file__).resolve().parent
AL, STEEL, DARK = (.74, .78, .82), (.48, .52, .58), (.20, .22, .25)
PLATE, BUY, PRINT = (.62, .70, .78), (.35, .78, .45), (.80, .66, .40)


class P30:
    def __init__(self, motion=None):
        self.L = json.loads((HERE/'layout.json').read_text(encoding='utf-8'))
        self.parts = json.loads((ROOT/'cad/parts.json').read_text(encoding='utf-8'))
        self.motion = dict(motion or {})
        f, pl = self.L['frame'], self.L['plates']
        s = f['section']
        self.s = s
        self.z_lo0 = f['lower_bottom_z']; self.z_lo1 = self.z_lo0+s            # lower frame
        self.z_floor1 = self.z_lo1+pl['floor_t']                                 # floor top
        self.z_top0 = f['top_bottom_z']; self.z_top1 = self.z_top0+s            # top frame
        self.z_deck1 = self.z_top1+pl['deck_t']
        q = self.L['drive']
        self.z_hold_lo1 = self.z_lo1+q['holder_t']                               # lower holder top
        self.z_hold_up0 = self.z_top0-q['holder_t']                              # upper holder bottom
        sp = q['spring']
        self.spring_nominal = sp['free']-sp['target_force_N']/sp['rate']
        self.z_carr1 = self.z_hold_up0-self.spring_nominal                       # carriage top (nominal)
        self.z_carr0 = self.z_carr1-q['seat_t']                                  # spring seat bottom = axle plate top
        if abs(self.parts['caster']['cad_height']+pl['corner_plate_t']-self.z_lo0) > 1e-9:
            raise ValueError('Corner plate must fill the gap between the caster plate and the lower frame')


# ---------------------------------------------------------------- generic geometry
def profile(axis, c1, c2, a0, a1, s=30):
    """Generic 3030 slot-8 profile along axis ('x','y','z'); (c1, c2) = centre of the section in the other two axes."""
    h = s/2; length = a1-a0
    body = span(-h, h, -h, h, 0, length)-cylz(3.4, length+2, 0, 0, -1)
    for u0, u1, v0, v1 in ((-4, 4, h-2.2, h+1), (-8, 8, h-8, h-2.2)):
        for k in range(4):
            body -= rz(span(u0, u1, v0, v1, -1, length+1), 90*k)
    if axis == 'z':
        return body.translate((c1, c2, a0))
    if axis == 'x':
        return body.rotate(Axis.Y, 90).translate((a0, c1, c2))
    return body.rotate(Axis.X, -90).translate((c1, a0, c2))


def ring_x(x0, t, y, z, od, bore):
    x0, x1 = sorted((x0, x0+t))
    return cylx(od/2, x1-x0, (x0+x1)/2, y, z)-cylx(bore/2, x1-x0+2, (x0+x1)/2, y, z)


def nut_x(x0, t, y, z, af, d):
    """Hex nut with axis X between x0 and x0+t (t may be negative)."""
    x0, x1 = sorted((x0, x0+t))
    hexa = nut(0, 0, 0, af, x1-x0, d)
    return hexa.rotate(Axis.Y, 90).translate((x0, y, z))


def tnut_bolt(x, y, rail_face, stack, d, slot_dir, from_above=True, flush=False):
    """Bolt through `stack` mm of plate into a T-nut of a 3030 slot-8 rail face at Z=rail_face.

    from_above: plates on top of the rail, head up. Otherwise plates under the rail, head down.
    slot_dir: 'x' or 'y' = slot (and T-nut long side) direction. Returns one Compound.
    """
    sg = 1 if from_above else -1
    head = SCREW_HEAD[d]
    face = rail_face+sg*stack
    shank0 = rail_face-sg*6.5
    z0, z1 = sorted((face, shank0))
    shank = cylz(d/2-.01, z1-z0, x, y, z0)
    hd = None if flush else cylz(head/2, d, x, y, face if from_above else face-d)
    nz0, nz1 = sorted((rail_face-sg*2.3, rail_face-sg*6.3))
    w = (14.5, 9.5) if slot_dir == 'x' else (9.5, 14.5)
    tn = span(x-w[0]/2, x+w[0]/2, y-w[1]/2, y+w[1]/2, nz0, nz1)-cylz(d/2, nz1-nz0+2, x, y, nz0-1)
    return Compound([s for c in ((shank if hd is None else shank+hd), tn) for s in c.solids()])


SCREW_HEAD = {3: 5.5, 4: 7, 5: 8.5, 6: 10, 8: 13}


def tapped_bolt(x, y, head_face, clear_t, engage, d, down=True, minor=None):
    """Socket screw: head on head_face, shank through clear_t of clearance hole, then `engage` mm in a tapped hole.
    The engaged length is drawn at the tap-drill diameter so the tapped part can be drilled at the same size;
    pass `minor` to match a vendor STEP that draws its thread at another diameter."""
    minor = minor or {5: 4.2, 6: 5.0, 8: 6.8}[d]
    sg = -1 if down else 1
    head = cylz(SCREW_HEAD[d]/2, d, x, y, head_face if down else head_face-d)
    a = sorted((head_face, head_face+sg*clear_t))
    b = sorted((head_face+sg*clear_t, head_face+sg*(clear_t+engage)))
    return head+cylz(d/2-.01, a[1]-a[0], x, y, a[0])+cylz(minor/2-.01, b[1]-b[0], x, y, b[0])


def lbracket(x, y, z, dx, dy, up=True, size=28, t=4):
    """3030 cast angle bracket envelope in the corner between a post face (normal (dx,dy)) and a plate face at z."""
    sg = 1 if up else -1
    z0, z1 = sorted((z, z+sg*size))
    tz0, tz1 = sorted((z, z+sg*t))
    if dx:
        x0, x1 = sorted((x, x+dx*size)); fx0, fx1 = sorted((x, x+dx*t))
        leg_v = span(fx0, fx1, y-size/2+7, y+size/2-7, z0, z1)
        leg_h = span(x0, x1, y-size/2+7, y+size/2-7, tz0, tz1)
    else:
        y0, y1 = sorted((y, y+dy*size)); fy0, fy1 = sorted((y, y+dy*t))
        leg_v = span(x-size/2+7, x+size/2-7, fy0, fy1, z0, z1)
        leg_h = span(x-size/2+7, x+size/2-7, y0, y1, tz0, tz1)
    return leg_v+leg_h


def tname(t):
    """Thickness tag for part names: 2.3 -> '2p3t'."""
    return f'{t:g}'.replace('.', 'p')+'t'


def rec(name, group, shape, color=AL, status='concept_fabrication', **meta):
    return PartRecord(name, group, shape, color, status, meta)


# ---------------------------------------------------------------- frame
def frame(P):
    L = P.L; f = L['frame']; s = P.s; h = s/2
    X, Y = f['outer_x'], f['outer_y']
    zl = (P.z_lo0+P.z_lo1)/2; zt = (P.z_top0+P.z_top1)/2
    r = []
    meta = lambda n: {'material': 'Al 3030 slot-8 profile (company stock)', 'length_mm': n, 'section_status': f['section_status']}
    # lower frame
    r.append(rec('01_P30_lower_front_rail', 'frame', profile('x', Y-h, zl, -X, X), AL, 'generic_profile', **meta(2*X)))
    r.append(rec('02_P30_lower_rear_rail', 'frame', profile('x', -Y+h, zl, -X, X), AL, 'generic_profile', **meta(2*X)))
    for sx in (-1, 1):
        n = 'L' if sx < 0 else 'R'
        r.append(rec('03_P30_lower_inner_rail_'+n, 'frame', profile('y', sx*f['inner_rail_x'], zl, -Y+s, Y-s), AL, 'generic_profile', **meta(2*(Y-s))))
        for sy in (-1, 1):
            m = n+('F' if sy > 0 else 'B')
            a0, a1 = sorted((sy*f['stub_start_y'], sy*(Y-s)))
            r.append(rec('04_P30_lower_side_stub_'+m, 'frame', profile('y', sx*(X-h), zl, a0, a1), AL, 'generic_profile', **meta(a1-a0)))
    # posts
    cx, cy = f['corner_post_xy']
    for sx in (-1, 1):
        for sy in (-1, 1):
            m = ('L' if sx < 0 else 'R')+('F' if sy > 0 else 'B')
            if m not in f['lidar_corners']:
                r.append(rec('05_P30_corner_post_'+m, 'frame', profile('z', sx*cx, sy*cy, P.z_floor1, P.z_top0), AL, 'generic_profile',
                             **meta(P.z_top0-P.z_floor1)))
            r.append(rec('06_P30_bay_post_'+m, 'frame', profile('z', sx*(X-h), sy*f['bay_post_y'], P.z_hold_lo1, P.z_hold_up0), AL,
                         'generic_profile', **meta(P.z_hold_up0-P.z_hold_lo1)))
    # top frame
    r.append(rec('07_P30_top_front_rail', 'frame', profile('x', Y-h, zt, -X, X), AL, 'generic_profile', **meta(2*X)))
    r.append(rec('07_P30_top_rear_rail', 'frame', profile('x', -Y+h, zt, -X, X), AL, 'generic_profile', **meta(2*X)))
    for sx in (-1, 1):
        n = 'L' if sx < 0 else 'R'
        r.append(rec('08_P30_top_side_rail_'+n, 'frame', profile('y', sx*(X-h), zt, -Y+s, Y-s), AL, 'generic_profile', **meta(2*(Y-s))))
    for sy in (-1, 1):
        r.append(rec('09_P30_top_arm_member_'+('F' if sy > 0 else 'B'), 'frame', profile('x', sy*f['arm_member_y'], zt, -X+s, X-s), AL,
                     'generic_profile', **meta(2*(X-s))))
    return r


def post_brackets(P):
    """Two cast angle brackets at each post end (M6 + T-nuts not drawn)."""
    f = P.L['frame']; s = P.s; h = s/2; X = f['outer_x']; out = []
    cx, cy = f['corner_post_xy']
    for sx in (-1, 1):
        for sy in (-1, 1):
            m = ('L' if sx < 0 else 'R')+('F' if sy > 0 else 'B')
            shapes = []
            if m not in f['lidar_corners']:
                x, y = sx*cx, sy*cy
                for z, up in ((P.z_floor1, True), (P.z_top0, False)):
                    shapes += [lbracket(x-sx*h, y, z, -sx, 0, up), lbracket(x, y-sy*h, z, 0, -sy, up)]
            x, y = sx*(X-h), sy*f['bay_post_y']
            for z, up in ((P.z_hold_lo1, True), (P.z_hold_up0, False)):
                shapes += [lbracket(x-sx*h, y, z, -sx, 0, up), lbracket(x, y+sy*h, z, 0, sy, up)]
            out.append(rec('10_P30_post_angle_brackets_'+m, 'frame', Compound([q for c in shapes for q in c.solids()]), DARK,
                           'catalog_envelope', sku='3030 cast angle bracket 28x28 + 2x M6 T-nut bolts each (bolts not drawn)',
                           quantity=len(shapes)))
    return out


def plates(P):
    L = P.L; f, pl = L['frame'], L['plates']; X, Y = f['outer_x'], f['outer_y']; r = []
    c = L['caster']; cp = P.parts['caster']
    for sx in (-1, 1):
        for sy in (-1, 1):
            m = ('L' if sx < 0 else 'R')+('F' if sy > 0 else 'B')
            x0, x1 = (sx*v for v in pl['corner_plate_x']); y0, y1 = (sy*v for v in pl['corner_plate_y'])
            plate = span(x0, x1, y0, y1, P.z_lo0-pl['corner_plate_t'], P.z_lo0)
            # M6 taps for the GL-65 plate (hole pitch 24 X x 54 Y), tap drill D5
            for dx in (-cp['hole_pitch_y']/2, cp['hole_pitch_y']/2):
                for dy in (-cp['hole_pitch_x']/2, cp['hole_pitch_x']/2):
                    plate -= cylz(2.5, 20, sx*c['x']+dx, sy*c['y']+dy, P.z_lo0-15)
            for x, y in corner_plate_bolts(P, sx, sy):
                plate -= cylz(3.25, 20, x, y, P.z_lo0-15)
            r.append(rec('11_P30_caster_corner_plate_'+tname(pl['corner_plate_t'])+'_'+m, 'plates', plate, PLATE,
                         material=f"SS400 {pl['corner_plate_t']:g}t laser cut, M6 tapped for the caster, 3x M6 into T-nuts of the lower rails"))
    # floor
    nx, ny = pl['floor_notch_x'], pl['floor_notch_y']
    pts = [(-X, -Y), (X, -Y), (X, -ny), (nx, -ny), (nx, ny), (X, ny), (X, Y), (-X, Y), (-X, ny), (-nx, ny), (-nx, -ny), (-X, -ny)]
    floor = polygon_plate(pts, pl['floor_t'], P.z_lo1)
    for x, y, dd in floor_holes(P):
        floor -= cylz(dd/2+.25, 10, x, y, P.z_lo1-3)
    r.append(rec('12_P30_floor_plate_'+tname(pl['floor_t']), 'plates', floor, PLATE, material=f"SS400 {pl['floor_t']:g}t laser cut, notched at the wheel bays"))
    # deck
    deck = span(-X, X, -Y, Y, P.z_top1, P.z_deck1)
    q = L['drive']
    for sx in (-1, 1):
        deck -= cylz(12, 20, sx*q['spring_x'], 0, P.z_top1-5)       # droop-nut access
    for x, y, dd in deck_holes(P):
        deck -= cylz(dd/2+.25, 20, x, y, P.z_top1-5)
    a = pl['arm_bolt_pitch']/2
    for x in (-a, a):
        for y in (-a, a):
            deck -= cylz(4.5, 20, x, y, P.z_top1-5)
    r.append(rec('13_P30_deck_'+tname(pl['deck_t']), 'plates', deck, PLATE,
                 material=f"SS400 {pl['deck_t']:g}t laser cut; 4x D9 arm holes pitch 124 (arm pattern TBD) into T-nuts of the arm members; D24 droop-nut access holes"))
    return r


def corner_plate_bolts(P, sx, sy):
    f = P.L['frame']; h = P.s/2
    return [(sx*150, sy*(f['outer_y']-h)), (sx*190, sy*(f['outer_y']-h)), (sx*(f['outer_x']-h), sy*225)]


def frame_fasteners(P):
    """T-nut bolts that tie the plates to the frame (representative set)."""
    L = P.L; f = L['frame']; X, Y = f['outer_x'], f['outer_y']; h = P.s/2; r = []
    sh = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for x, y in corner_plate_bolts(P, sx, sy):
                sh.append(tnut_bolt(x, y, P.z_lo0, L['plates']['corner_plate_t'], 6, 'x' if abs(abs(y)-(Y-h)) < 1e-6 else 'y', False))
    r.append(rec('14_P30_corner_plate_bolts_M6_Tnut', 'plates', Compound([q for c in sh for q in c.solids()]), DARK, 'catalog_envelope',
                 sku='M6x16 + 3030 T-nut M6', quantity=len(sh)))
    sh = [tnut_bolt(x, y, P.z_top1, L['plates']['deck_t'], dd, 'x' if abs(abs(y)-(Y-h)) < 1e-6 else 'y') for x, y, dd in deck_holes(P)]
    r.append(rec('15_P30_deck_bolts_M6_Tnut', 'plates', Compound([q for c in sh for q in c.solids()]), DARK, 'catalog_envelope',
                 sku='M6x12 + 3030 T-nut M6 (deck to top frame)', quantity=len(sh)))
    own = floor_holes(P)[:12]
    sh = [tnut_bolt(x, y, P.z_lo1, L['plates']['floor_t'], dd, 'x' if abs(abs(y)-(Y-h)) < 1e-6 else 'y') for x, y, dd in own]
    r.append(rec('16_P30_floor_bolts_M5_Tnut', 'plates', Compound([q for c in sh for q in c.solids()]), DARK, 'catalog_envelope',
                 sku='M5x8 + 3030 T-nut M5', quantity=len(sh)))
    return r


# ---------------------------------------------------------------- casters (rigid, OEM STEP as Rev15)
def casters(P):
    from ..parts.vendor import source
    original = source('GL-65-ASF-EUS.stp'); c = P.L['caster']; r = []
    cp = P.parts['caster']
    for sx in (-1, 1):
        for sy in (-1, 1):
            n = ('L' if sx < 0 else 'R')+('F' if sy > 0 else 'B'); x, y = sx*c['x'], sy*c['y']; group = 'caster_'+n.lower()
            for index, prefix in ((0, '43_GL65_OEM_fork_'), (1, '41_GL65_OEM_plate_head_'), (2, '44_GL65_OEM_wheel_')):
                s = rz(original.solids()[index].translate((-19.251901528418, 23.9893391369419, 51.7488089481697)), -90).translate((x, y, 0))
                r.append(rec(prefix+n, group, s, (.55, .64, .72), 'manufacturer_step_EUS_reference',
                             source='GL-65-ASF-EUS.stp', caster=n, swivel_xy=[x, y], cad_height_mm=cp['cad_height']))
            bolts = [tapped_bolt(x+dx, y+dy, cp['cad_height']-cp['plate_thickness_mm'], cp['plate_thickness_mm'], 6, 6, down=False)
                     for dx in (-cp['hole_pitch_y']/2, cp['hole_pitch_y']/2) for dy in (-cp['hole_pitch_x']/2, cp['hole_pitch_x']/2)]
            r.append(rec('45_caster_bolts_M6x8_'+n, group, Compound([q for b in bolts for q in b.solids()]), DARK, 'catalog_envelope',
                         sku='M6x8 hex socket into the 8t corner plate', quantity=4))
    return r


# ---------------------------------------------------------------- drive module
def drive(P, side):
    L = P.L; q = L['drive']; f = L['frame']; w = P.parts['drive_motor']
    n = 'LEFT' if side < 0 else 'RIGHT'; group = 'drive_'+n.lower()
    dz = P.motion.get(n, 0.0)
    S = lambda a, b: (side*a, side*b)
    xc = side*q['wheel_x']; zc = w['tire_dia']/2
    gx = side*q['guide_x']; gy = q['guide_y']; kx = side*q['spring_x']     # shafts / spring stud
    r = []

    def mv(name, shape, color=AL, status='concept_fabrication', **meta):
        r.append(PartRecord(name+'_'+n, group, shape.translate((0, 0, dz)), color, status, {'moving_wheel': n, **meta}))

    def fx(name, shape, color=AL, status='concept_fabrication', **meta):
        r.append(PartRecord(name+'_'+n, group, shape, color, status, {'fixed_drive_side': n, **meta}))

    # motor (same datasheet geometry as cad/parts/drive.py)
    outer = -w['tread_width']/2-w['outer_face_to_tire']; end = outer+w['overall_axial']; body = end-w['body_face_to_shaft_end']
    tire = cylx(w['tire_dia']/2, w['tread_width'], xc, 0, zc)-cylx(w['assumed_body_dia']/2, w['tread_width']+2, xc, 0, zc)
    casing = cylx(w['assumed_body_dia']/2, body-outer, xc-side*(outer+body)/2, 0, zc)
    axle = cylx(w['shaft_dia']/2, end-body, xc-side*(body+end)/2, 0, zc) & box(100, w['shaft_flat'], 30, xc-side*45, 0, zc)
    mv('30_NBR_OD180_W42p5', tire, (.09, .13, .18), 'datasheet_dimension')
    mv('31_casing_OD140_ASSUMED', casing, (.27, .48, .83), 'assumed_casing')
    mv('32_D14_AF10_M14x1p5_axle', axle, (.16, .27, .35), 'datasheet_dimension')
    body_x = q['wheel_x']-body                                   # |x| of the motor body face
    a1 = body_x-q['axle_spacer_t']; a0 = a1-q['axle_plate_t']    # axle plate |x| range
    if abs(a1+q['bush']['shaft_height']-q['guide_x']) > 1e-6:
        raise ValueError('guide_x must equal the axle plate outer face + housing shaft height')
    mv('33_axle_spacer_4p9_trial', ring_x(side*body_x, -side*q['axle_spacer_t'], 0, zc, 18, 14.3), STEEL, fit='Confirm the motor shoulder stack (Rev15 trial value)')
    mv('34_axle_washer', ring_x(side*a0, -side*1.6, 0, zc, 24, 14.3), STEEL)
    mv('35_axle_nut_M14x1p5', nut_x(side*(a0-1.6), -side*8, 0, zc, 22, 14.3), DARK, 'catalog_envelope')
    thread_end = q['wheel_x']-end
    if not thread_end < a0-1.6-8:
        raise ValueError('Axle thread does not pass the nut')
    # P30-4 wheel mount (plan A): ONE flat SS400 axle plate, vertical (YZ), no bend, no plate-to-plate joint in the load path.
    # The motor axle is clamped on its faces; two MISUMI LHSS20 housings on its outer face ride the guide shafts, so the
    # plate (and the axle normal to it) is held parallel to the shafts by the machined housing mounting faces.
    bu = q['bush']; hy = q['axle_plate_half_y']; hcz = q['housing_center_z']; t = q['axle_plate_t']
    ap = span(*S(a0, a1), -hy, hy, q['axle_plate_bottom_z'], P.z_carr0)
    ap -= cylx(7.15, 30, side*(a0+a1)/2, 0, zc) & span(*S(a0-5, a1+5), -5.1, 5.1, zc-10, zc+10)   # D-hole AF10.2
    hz0 = hcz-bu['length']/2+8                                   # first mounting tap: 8 mm from the housing end (STEP)
    taps = [(y0, hz0+k*bu['mount_pitch']) for y0 in (-gy, gy) for k in (0, 1)]
    for y, z in taps:
        ap -= cylx(4.5, 30, side*(a0+a1)/2, y, z)                 # D9 clearance (+-0.5 float to align on the shafts)
    seat_bolts = [(side*(a0+a1)/2, y) for y in (-55, 55)]
    for x, y in seat_bolts:
        ap -= cylz(2.5, 13, x, y, P.z_carr0-12)                   # M6 tapped top edge (tap drill D5)
    mv(f'36_axle_plate_SS400_{tname(t)}', ap, PLATE,
       material=f'SS400 {t:g}t laser cut, flat (no bend); D-hole AF10.2; 4x D9 for the housing bolts; 2x M6 tapped top edge',
       fit='Camber set by the plate face and the LHSS20 mounting faces; check plate flatness (laser-cut hot-rolled plate)')
    # housings (MISUMI STEP as supplied). STEP frame: shaft axis +X (x -8..44, centre y-31 z0), mounting face y=0 (+Y out),
    # 2x M8 taps at x 0 and 36 on that face. Rotate: axis -> +Z, mounting face normal -> towards the plate.
    from ..parts.vendor import misumi_solid
    hs = [misumi_solid(bu['step']).rotate(Axis.Y, -90).rotate(Axis.Z, 90*side).translate((side*a1, y0, hz0)) for y0 in (-gy, gy)]
    mv('39_linear_bush_'+bu['sku'], Compound(hs), (.80, .82, .86), 'misumi_step_exact', sku=bu['sku'], quantity=2, supply=bu['supply'],
       unit_price_krw=bu['unit_price_krw'], source=bu['source'],
       fit='Hang both housings on the shafts, then tighten the 4 M8 bolts (D9 holes give +-0.5 float, no over-constraint)')
    hb = []
    tap = 2*3.323-.04                                            # STEP draws the M8 tap at D6.646
    for y, z in taps:
        x_in = side*a0
        hb.append(cylx(6.5, 8, x_in-side*4, y, z)+cylx(3.99, t, side*(a0+a1)/2, y, z)+cylx(tap/2, 10, side*(a1+5), y, z))
    mv('40_housing_bolts_M8x20', Compound([s_ for b in hb for s_ in b.solids()]), DARK, 'catalog_envelope', quantity=len(hb),
       note=f'From the inner face: {t:g} mm plate + 10 mm thread in the housing (tap depth 15)')
    # spring seat: flat 6t plate on the axle plate top edge (spring force presses it onto the edge; 2x M6 locate it)
    st = q['seat_t']; sy = q['seat_half_y']
    seat = span(*S(a0, q['seat_x_out']), -sy, sy, P.z_carr0, P.z_carr1)
    for x, y in seat_bolts:
        seat -= cylz(3.25, 20, x, y, P.z_carr0-5)
    seat -= cylz(3.4, 20, kx, 0, P.z_carr0-5)                                     # M8 stud tap
    for y in (-q['bump_stopper']['y'], q['bump_stopper']['y']):
        seat -= cylz(2.51, 20, gx, y, P.z_carr0-5)                                  # M5 tap for C-USLH10 (thread drawn at major)
    mv(f'37_spring_seat_SS400_{tname(st)}', seat, PLATE, material=f'SS400 {st:g}t laser cut, flat; M8 stud tap + 2x M5 stopper taps; 2x D6.5')
    sb = [tapped_bolt(x, y, P.z_carr1, st, 10, 6) for x, y in seat_bolts]
    mv('38_seat_bolts_M6x16', Compound([s_ for b in sb for s_ in b.solids()]), DARK, 'catalog_envelope', quantity=2)
    # stud + droop stop (washer + nylon nut on top of the upper holder at full droop)
    half = q['travel_mm']/2
    washer0 = P.z_top0+half                         # nominal washer bottom: lands on the holder top at -half
    stud_top = washer0+1.6+8+1.5
    stud = cylz(3.39, q['seat_t'], kx, 0, P.z_carr0)+cylz(4, stud_top-P.z_carr1, kx, 0, P.z_carr1)
    mv('44_spring_stud_M8', stud, STEEL, 'catalog_envelope', length_mm=round(stud_top-P.z_carr0, 1),
       fit='M8 threaded rod in the tapped spring seat + threadlocker; guides the spring through the D10 holder hole')
    mv('45_droop_stop_washer_nylock_M8', cylz(8, 1.6, kx, 0, washer0)-cylz(4.2, 4, kx, 0, washer0-1)+nut(kx, 0, washer0+1.6, 13, 8, 8.4),
       DARK, 'catalog_envelope', adjust='Nut sets the droop limit (-10 mm); reachable through the deck hole')
    # bump stoppers (company stock C-USLH10) on the spring seat
    bs = q['bump_stopper']
    stops = [misumi_solid(bs['step'], c).rotate(Axis.Y, 90).translate((gx, y, P.z_carr1)) for y in (-bs['y'], bs['y']) for c in bs['children']]
    tip = max(s.bounding_box().max.Z for s in stops)
    mv('46_bump_stopper_C-USLH10', Compound(stops), BUY, 'misumi_step_exact', sku=bs['sku'], supply='company stock (10)', quantity=2,
       tip_z=round(tip, 3))
    # spring (length follows the pose)
    sp = q['spring']; sq = {'spring_od_mm': sp['od'], 'spring_id_mm': sp['id'], 'spring_wire_axial_mm': sp['wire_axial'], 'display_turns': sp['turns']}
    length = P.z_hold_up0-(P.z_carr1+dz)
    r.append(PartRecord('47_spring_'+sp['sku']+'_'+n, group, coil(kx, 0, P.z_carr1+dz, length, sq), (.86, .30, .25), 'catalog_envelope',
                        {'spring_length_mm': round(length, 3), 'sku': sp['sku'], 'rate_N_mm': sp['rate'], 'force_N': round((sp['free']-length)*sp['rate'], 1),
                         'pose_dependent': n}))
    # ---- fixed: shafts, holders, bay posts are in frame(); strikers
    ys = (-gy, gy)
    shq = q['shaft']
    if abs(shq['length']-(P.z_hold_up0-P.z_hold_lo1)) > 1e-6:
        raise ValueError('Shaft length must equal the holder gap')
    shafts = [misumi_solid(shq['step']).rotate(Axis.Y, -90).translate((gx, y, P.z_hold_lo1)) for y in ys]
    fx('50_guide_shaft_'+shq['sku'], Compound(shafts), STEEL, 'misumi_step_exact', sku=shq['sku'], length_mm=shq['length'], quantity=2,
       unit_price_krw=shq['unit_price_krw'], source=shq['source'], supply='Purchase MISUMI (D20 g6, both ends M8 x 16 effective)')
    th = tname(q['holder_t'])
    for name, y0, y1 in (('51_lower_shaft_holder_'+th+'_F', q['lower_holder_y'][0], q['lower_holder_y'][1]),
                         ('51_lower_shaft_holder_'+th+'_B', -q['lower_holder_y'][1], -q['lower_holder_y'][0])):
        hold = span(*S(*q['holder_x']), y0, y1, P.z_lo1, P.z_hold_lo1)
        for y in ys:
            hold -= cylz(4.25, 20, gx, y, P.z_lo1-5)
        for y in lower_holder_bolts(P, y0, y1):
            hold -= cylz(2.75, 20, side*(f['outer_x']-P.s/2), y, P.z_lo1-5)
        fx(name, hold, PLATE, material=f"SS400 {q['holder_t']:g}t laser cut, on the side stub; D8.5 for the shaft bolt; 2x M5 T-nut")
    up = span(*S(*q['holder_x']), -q['upper_holder_half_y'], q['upper_holder_half_y'], P.z_hold_up0, P.z_top0)
    for y in ys:
        up -= cylz(4.25, 20, gx, y, P.z_hold_up0-5)
    up -= cylz(5, 20, kx, 0, P.z_hold_up0-5)
    for y in (-bs['y'], bs['y']):
        up -= cylz(3.0, 20, gx, y, P.z_hold_up0-5)
    for y in upper_holder_bolts(P):
        up -= cylz(3.25, 20, side*(f['outer_x']-P.s/2), y, P.z_hold_up0-5)
    fx('52_upper_shaft_holder_'+tname(q['holder_t']), up, PLATE,
       material=f"SS400 {q['holder_t']:g}t laser cut, under the top side rail; spring seat; M6 tapped strikers; 4x M6 T-nut")
    tap = 2*3.324-.04                                  # SFJW STEP draws the M8 tap at D6.648
    bolts = [tapped_bolt(gx, y, P.z_lo1, q['holder_t'], 14, 8, down=False, minor=tap) for y in ys]
    bolts += [tapped_bolt(gx, y, P.z_top0, q['holder_t'], 14, 8, minor=tap) for y in ys]
    fx('53_shaft_end_bolts_M8x20', Compound([s for b in bolts for s in b.solids()]), DARK, 'catalog_envelope', quantity=4)
    strike_tip = tip+half+.05
    strikers = [cylz(5, 6, gx, y, P.z_top0+5)+nut(gx, y, P.z_top0, 10, 5, 6)+cylz(2.99, P.z_top0+5-strike_tip, gx, y, strike_tip) for y in (-bs['y'], bs['y'])]
    fx('54_bump_striker_M6_jam_nut', Compound([s for c in strikers for s in c.solids()]), DARK, 'catalog_envelope', quantity=2,
       tip_z=round(strike_tip, 3), adjust='Striker meets the C-USLH10 tip at +10 mm; set with the jam nut')
    tb = [tnut_bolt(side*(f['outer_x']-P.s/2), y, P.z_lo1, q['holder_t'], 5, 'y') for y0, y1 in (q['lower_holder_y'], (-q['lower_holder_y'][1], -q['lower_holder_y'][0]))
          for y in lower_holder_bolts(P, y0, y1)]
    tb += [tnut_bolt(side*(f['outer_x']-P.s/2), y, P.z_top0, q['holder_t'], 6, 'y', False) for y in upper_holder_bolts(P)]
    fx('55_holder_bolts_Tnut', Compound([s for c in tb for s in c.solids()]), DARK, 'catalog_envelope', quantity=len(tb))
    return r


def lower_holder_bolts(P, y0, y1):
    sg = 1 if y0+y1 > 0 else -1
    return [sg*105, sg*135]


def upper_holder_bolts(P):
    return [-130, -40, 40, 130]


def floor_holes(P):
    """(x, y, d) of every bolt that passes the floor plate."""
    L = P.L; f = L['frame']; Y = f['outer_y']; h = P.s/2; e = L['electronics']
    pts = [(x, y, 5) for y in (Y-h,) for x in (-140, 140)]+[(x, -(Y-h), 5) for x in (0, 120)]
    pts += [(sx*f['inner_rail_x'], y, 5) for sx in (-1, 1) for y in (-160, -90, 90, 160)]
    pts += [(x, Y-h, 5) for x in (-100, 100)]                                  # LCD bracket
    pts += [(e['speaker_x']+dx, -(Y-h), 5) for dx in (-40, 40)]                # speaker bracket
    for pose in lidar_poses(P):
        pts += [(pose['x']+dx, pose['y'], 5) for dx in (-12, 12)]
    return pts


def deck_holes(P):
    f = P.L['frame']; X, Y = f['outer_x'], f['outer_y']; h = P.s/2
    return [(x, y, 6) for y in (-(Y-h), Y-h) for x in (-120, 0, 120)]+[(sx*(X-h), y, 6) for sx in (-1, 1) for y in (-210, -90, 90, 210)]


# ---------------------------------------------------------------- electronics
class _Shim:
    """Minimal Design-like object for the reused Rev15 electronics builders."""
    def __init__(self, P, extra):
        self.variant = 'current'
        self.p = {'base_bottom_z': P.z_lo1, 'base_thickness': P.L['plates']['floor_t']}
        self.layout = {'electronics': extra}


def electronics(P):
    from ..parts.vendor import source
    from ..parts import electronics as ev
    L = P.L; e = L['electronics']; r = []; top = P.z_floor1; f = L['frame']; h = P.s/2
    # battery: company STEP turned 90 deg about Z (long side along Y), on 4 rubber pads
    src = source('BATTERY.STEP'); solids = [s.rotate(Axis.Z, e['battery_yaw_deg']) for s in src.solids()]
    case = max(solids, key=lambda s: s.volume).bounding_box()
    mvb = (-(case.min.X+case.max.X)/2, e['battery_center_y']-(case.min.Y+case.max.Y)/2, top+e['battery_pad_mm']-case.min.Z)
    batt = Compound([s.translate(mvb) for s in solids])
    r.append(rec('60_company_7S9P_battery_STEP', 'electronics', batt, (.86, .63, .36), 'company_supplied_step', source='BATTERY.STEP.gz',
                 placement={'yaw_deg': e['battery_yaw_deg'], 'translate_mm': [round(v, 3) for v in mvb]},
                 limits='Hold-down strap (2x 25 mm strap through the floor slots) not drawn'))
    bb = batt.bounding_box(); cb = case
    pads = [cylz(7.5, e['battery_pad_mm'], x, y, top) for x in (cb.min.X+mvb[0]+15, cb.max.X+mvb[0]-15) for y in (cb.min.Y+mvb[1]+15, cb.max.Y+mvb[1]-15)]
    r.append(rec('60_battery_pads', 'electronics', Compound(pads), (.30, .30, .32), 'concept_support', quantity=4))
    # drivers on edge beside the battery, near their motors
    dx, dy, dz = e['driver_size']
    for sx in (-1, 1):
        x = sx*e['driver_x']
        r.append(rec('61_motor_driver_ENVELOPE_'+('L' if sx < 0 else 'R'), 'electronics', span(x-dx/2, x+dx/2, -dy/2, dy/2, top, top+dz), (.28, .38, .52),
                     'envelope_only', mount='on edge, 3t angle bracket to the floor (not drawn)'))
    # controller PCB + computer on M4 standoffs
    px, py = e['pcb_xy']; pcb0 = top+10
    pcb = box(165, 104, 7, px, py, pcb0+3.5); comp0 = pcb0+7+e['computer_above_pcb']
    comp = box(146, 82, 23, px, py, comp0+11.5)
    r.append(rec('62_controller_PCB_ENVELOPE', 'electronics', pcb, (.12, .46, .37), 'envelope_only'))
    r.append(rec('63_control_computer_ENVELOPE', 'electronics', comp, (.13, .19, .28), 'envelope_only'))
    legs = [cylz(3.5, 10, x, y, top) for x in (px-74.5, px+74.5) for y in (py-44, py+44)]
    legs += [cylz(3.5, e['computer_above_pcb'], x, y, pcb0+7) for x in (px-65, px+65) for y in (py-33, py+33)]
    r.append(rec('64_electronics_standoffs_M4', 'electronics', Compound(legs), (.55, .58, .62), 'concept_support', quantity=8))
    # power distribution
    qx, qy = e['power_bay_xy']; sx_, sy_, sz_ = e['power_bay_size']
    r.append(rec('65_power_distribution_ENVELOPE', 'electronics', span(qx-sx_/2, qx+sx_/2, qy-sy_/2, qy+sy_/2, top, top+sz_), (.65, .39, .24), 'envelope_only'))
    # DC/DC on a DIN rail (reuses the Rev15 builder)
    shim = _Shim(P, {'dcdc_size': e['dcdc_size'], 'dcdc_x': e['dcdc_xy'][0], 'dcdc_y': e['dcdc_xy'][1], 'dcdc_bottom_z': top+12})
    for item in ev.dcdc(shim):
        r.append(rec(item.name.replace('55_', '66_'), 'electronics', item.shape, item.color, item.status, **item.metadata))
    r += lcd(P)+speaker(P)
    return r


def lcd(P):
    from ..parts.vendor import source
    from ..parts.electronics import _lcd_frame, _place
    e = P.L['electronics']; top = P.z_floor1; Y = P.L['frame']['outer_y']; h = P.s/2
    shim = _Shim(P, {'lcd_tilt_deg': e['lcd_tilt_deg'], 'lcd_front_y': e['lcd_front_y'], 'lcd_bottom_z': top+3+1})
    tilt, move = _lcd_frame(shim)
    screen = Compound([_place(s, tilt, move) for s in source('LCD.STEP').solids()])
    plate = _place(span(-95.9, 95.9, -15.3, -12.3, -90, 45), tilt, move) & span(-120, 120, 0, 400, top+3, 400)
    bottom = (plate & span(-120, 120, 0, 400, top+3, top+3.5)).bounding_box()
    foot = span(-110, 110, bottom.min.Y, Y, top, top+3)
    bolts = [(x, Y-h) for x in (-100, 100)]
    for x, y in bolts:
        foot -= cylz(2.75, 10, x, y, top-1)
    clamps = []
    for s in (-1, 1):
        block = span(*sorted((s*89.9, s*95.9)), -12.3, 4.6, -40, 40)+span(*sorted((s*87.4, s*89.9)), 3.1, 4.6, -40, 40)
        clamps.append(_place(block, tilt, move))
    tb = [tnut_bolt(x, y, P.z_lo1, 3+P.L['plates']['floor_t'], 5, 'x') for x, y in bolts]
    return [rec('67_company_LCD_7in_STEP', 'electronics', screen, (.10, .12, .16), 'company_supplied_step', source='LCD.STEP.gz', tilt_deg=tilt),
            rec('67_LCD_bracket_3t', 'electronics', plate+foot, PLATE, material='SPCC 3t, 1 bend; foot bolted through the floor into the front rail'),
            rec('67_LCD_side_clamps_PRINTED', 'electronics', Compound(clamps), PRINT, 'concept_support', quantity=2),
            rec('67_LCD_bolts_M5_Tnut', 'electronics', Compound([s for c in tb for s in c.solids()]), DARK, 'catalog_envelope', quantity=2)]


def speaker(P):
    from ..parts.electronics import SPEAKER_FRAME, SPEAKER_SCREWS, SPEAKER_BACK, _speaker_shape
    e = P.L['electronics']; top = P.z_floor1; Y = P.L['frame']['outer_y']; h = P.s/2
    (x0, x1), (y0, _), (z0, _) = SPEAKER_FRAME
    move = (e['speaker_x']-(x0+x1)/2, e['speaker_front_y']-y0, e['speaker_bottom_z']-z0)
    unit, trimmed = _speaker_shape(move)
    T = lambda s: s.translate(move)
    b0 = SPEAKER_BACK
    plate = T(span(66, 180, b0, b0+3, top+3-move[2], 192))-T(span(84, 162, b0-5, b0+5, 101.5, 176))
    for x, z in SPEAKER_SCREWS:
        plate -= T(cyly(1.7, 10, x, b0+1.5, z))
    y_plate = b0+move[1]
    foot = span(move[0]+66, move[0]+180, e['speaker_front_y'], y_plate+3, top, top+3)
    bolts = [(e['speaker_x']+dx, -(Y-h)) for dx in (-40, 40)]
    for x, y in bolts:
        foot -= cylz(2.75, 10, x, y, top-1)
    screws = Compound([T(cyly(2.75, 3, x, b0+4.5, z)+cyly(1.5, b0+3-293.9, x, (b0+3+293.9)/2, z)) for x, z in SPEAKER_SCREWS])
    tb = [tnut_bolt(x, y, P.z_lo1, 3+P.L['plates']['floor_t'], 5, 'x') for x, y in bolts]
    return [rec('68_company_speaker_STEP', 'electronics', unit, (.18, .18, .2), 'company_supplied_step', source='SPEAKER.STEP.gz', trimmed_solids=trimmed,
                limits='Lead wire outside the frame/box envelope trimmed (as Rev15)'),
            rec('68_speaker_bracket_3t', 'electronics', plate+foot, PLATE, material='SPCC 3t, 1 bend; grille faces rear (-Y)'),
            rec('68_speaker_screws_M3', 'electronics', screws, DARK, 'catalog_envelope', quantity=3),
            rec('68_speaker_bolts_M5_Tnut', 'electronics', Compound([s for c in tb for s in c.solids()]), DARK, 'catalog_envelope', quantity=2)]


# ---------------------------------------------------------------- sensors
def lidar_bottom_z(P):
    """Scanner housing bottom: on the lidar plate, which sits on the floor plate."""
    return P.z_floor1+P.L['sensors']['lidar_plate'][2]


def lidar_poses(P):
    f = P.L['frame']; cx, cy = f['corner_post_xy']; out = []
    for m in f['lidar_corners']:
        sx = -1 if m[0] == 'L' else 1; sy = 1 if m[1] == 'F' else -1
        heading = math.degrees(math.atan2(sx, sy))
        out.append({'name': ('FRONT_' if sy > 0 else 'REAR_')+('LEFT' if sx < 0 else 'RIGHT'), 'x': sx*cx, 'y': sy*cy, 'heading_deg': heading,
                    'scan_z': lidar_bottom_z(P)+P.L['sensors']['scan_offset_above_envelope_min']})
    return out


def lidars(P):
    lid = P.parts['lidar']; sn = P.L['sensors']; r = []
    pw, pd, pt = sn['lidar_plate']; z0 = lidar_bottom_z(P)
    for pose in lidar_poses(P):
        n = pose['name']; x, y = pose['x'], pose['y']; yaw = -pose['heading_deg']
        place = lambda o: rz(o, yaw).translate((x, y, 0))
        plate = place(box(pw, pd, pt, 0, 0, z0-pt/2))
        for dx in (-12, 12):
            plate -= cylz(2.75, 10, x+dx, y, z0-pt-1)   # countersunk M5 (flush, under the scanner)
        r.append(rec('74_lidar_plate_3t_'+n, 'lidars', plate, PLATE, material='SS400 3t on the floor corner; M3 to the scanner, 2x M5 T-nut'))
        tb = [tnut_bolt(x+dx, y, P.z_lo1, pt+P.L['plates']['floor_t'], 5, 'x', flush=True) for dx in (-12, 12)]
        r.append(rec('74_lidar_plate_bolts_M5_Tnut_'+n, 'lidars', Compound([s for c in tb for s in c.solids()]), DARK, 'catalog_envelope', quantity=2))
        r.append(rec('75_SICK_TiM571_envelope_base_'+n, 'lidars', place(box(lid['width'], lid['depth'], 36, 0, 0, z0+18)), (.85, .88, .90), 'drawing_envelope'))
        r.append(rec('76_SICK_TiM571_optical_envelope_'+n, 'lidars', place(box(61, 60, lid['height']-39, 0, 0, z0+36+(lid['height']-39)/2)), (.09, .16, .21),
                     'drawing_envelope', scan_z=pose['scan_z'], heading_deg=pose['heading_deg']))
        r.append(rec('77_SICK_scanner_top_'+n, 'lidars', cylz(22, 3, x, y, z0+lid['height']-3), (.20, .80, .79), 'drawing_envelope'))
    return r


def cameras(P):
    from ..parts.vendor import camera_shape
    sn = P.L['sensors']; Y = P.L['frame']['outer_y']; h = P.s/2; r = []
    for side in (-1, 1):
        n = 'LEFT' if side < 0 else 'RIGHT'
        s, detail = camera_shape(side, sn['camera_x'], sn['camera_y'], sn['camera_z'], sn['camera_pitch_down_deg'], sn['camera_yaw_deg'])
        r.append(rec('70_Orbbec_Astra_Mini_Pro_MX6000_'+n, 'cameras', s, (.65, .50, .82), 'manufacturer_step_family_revision_reference',
                     source='AstraMiniPro_3D_V0.1_20221012.stp', placement_repair=detail))
        # 3t bracket, 2 bends: flange under the top front rail -> leg -> shelf; printed wedge on the shelf
        x0, x1 = sorted((side*108, side*160)); z_sh = sn['camera_shelf_z']; leg_y = 250
        flange = span(x0, x1, leg_y, Y, P.z_top0-3, P.z_top0)
        leg = span(x0, x1, leg_y, leg_y+3, z_sh, P.z_top0-3)
        shelf = span(x0, x1, leg_y+3, Y, z_sh, z_sh+3)
        xb = side*134
        bracket = flange+leg+shelf-cylz(2.75, 10, xb, Y-h, P.z_top0-5)
        r.append(rec('78_camera_bracket_3t_'+n, 'cameras', bracket, PLATE, material='SPCC 3t, 2 bends, 1x M5 T-nut into the top front rail'))
        frame_ = Location((side*sn['camera_x'], sn['camera_y'], sn['camera_z']))*Rotation(0, 0, -side*sn['camera_yaw_deg'])*Rotation(-sn['camera_pitch_down_deg'], 0, 0)
        local = s.moved(frame_.inverse()).bounding_box()
        block = box(local.size.X-4, local.size.Y-4, 60, local.center().X, local.center().Y, local.min.Z-30).moved(frame_)
        wedge = block & span(x0, x1, leg_y+3, Y, z_sh+3, z_sh+60)
        r.append(rec('79_camera_wedge_PRINTED_'+n, 'cameras', wedge, PRINT, 'concept_support', material='3D-printed wedge; M2 camera screws not drawn'))
        tb = tnut_bolt(xb, Y-h, P.z_top0, 3, 5, 'x', False, flush=True)   # countersunk: camera top is close under the flange
        r.append(rec('79_camera_bracket_bolt_M5_Tnut_'+n, 'cameras', tb, DARK, 'catalog_envelope', quantity=1, sku='M5 countersunk + T-nut'))
    return r


GROUPS = ('frame', 'plates', 'drive_left', 'drive_right', 'casters', 'electronics', 'lidars', 'cameras')


def build_records(P=None, only=None):
    P = P or P30(); want = set(only or GROUPS); r = []
    if 'frame' in want:
        r += frame(P)+post_brackets(P)
    if 'plates' in want:
        r += plates(P)+frame_fasteners(P)
    for side, g in ((-1, 'drive_left'), (1, 'drive_right')):
        if g in want:
            r += drive(P, side)
    if 'casters' in want:
        r += casters(P)
    if 'electronics' in want:
        r += electronics(P)
    if 'lidars' in want:
        r += lidars(P)
    if 'cameras' in want:
        r += cameras(P)
    return r
