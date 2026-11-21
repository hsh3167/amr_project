// Drive-test dynamics for the Eco and Pro models: 2-D pitch-plane multibody simulation
// (side view, x = forward, z = up, SI units, angles positive nose-up).
//
// Bodies: the sprung chassis (x, z, pitch) and sub-bodies joined to it by
//   'slide' joints: vertical slide in the chassis frame (stiff penalty across the slide, preloaded spring along it,
//                   rubber end stops); the sub-body keeps the chassis pitch (Eco drive wheels, Pro rear caster arms)
//   'pivot' joints: pin joint (stiff penalty) with its own pitch, a spring torque and rotation stops (Pro bogie links).
// The left and right sides are lumped (left = right). The drive wheel (one lumped wheel with its own spin) and the
// caster circles sit on any body; contacts are penalty springs against a polyline terrain that is uniform across
// the width. Underside / nose / tail and the moving carriages are sampled from the CAD silhouettes
// (cad-output/p30/side_profile.json, cad-output/side_profile.json) so they can scrape or hang up.
// Drive: wheel speed PI control with the ZLLG65 torque limit (current limit at stall torque, falling to zero
// at no-load speed through the rated point).
// Scope: planar, swivel offset of the casters ignored, assumed CG / inertia / tyre stiffness,
// no motor thermal limit. It is a design-comparison tool, not a substitute for a prototype test.

export const G = 9.81;
export const DT = 4e-4;     // s, semi-implicit Euler step
const FRAME_DT = 1/60;      // s, playback frame spacing

const tyre = (lc) => ({
  kt: 2*3.0e5, ct: 1200,                             // assumed NBR tyre radial stiffness 300 N/mm per wheel
  kc: 2*4e5, cc: 1500, crr: 0.02,                    // caster pair: contact stiffness, rolling resistance
  rp: 0.005, kp: 3e6, cp: 3000, muBody: 0.3,         // silhouette points: radius, stiffness, steel-on-floor friction
  mu: lc.friction_mu, eps: 0.05, sig0: 1e5, sig1: 400,           // tyre: LuGre bristle stiffness / damping (static friction)
  Kp: 7, Ki: 35, kj: 2e6, cj: 2e3,
});
const motor = (dm) => ({rated: dm.rated_torque_Nm, peak: dm.stall_torque_Nm, w0: dm.no_load_rpm*Math.PI/30, wr: dm.rated_speed_rpm*Math.PI/30});
const every = (rows, f, step = 20) => rows.filter(([y]) => Math.round(y) % step === 0).map(f);
const faces = (prof, f) => {        // vertical row of points on the nose and tail faces
  const o = [];
  for (const end of [prof.nose, prof.tail]) for (let z = end.z_low; z <= 200; z += 25) o.push(f(end.y, z));
  return o;
};

// ---- Eco (cad/p30): chassis + drive wheels on a vertical spring slide + 4 rigid casters -------------------
export function ecoParams(L, parts, prof) {
  const lc = L.load_case, dm = parts.drive_motor, dr = L.drive, sp = dr.spring;
  const m = lc.mass_kg, mw = 2*dr.unsprung_mass_kg_estimate, mc = m-mw;
  const R = dm.tire_dia/2000, zcg = lc.cg_height_mm/1000;
  const hs = (m*zcg-mw*R)/mc;                          // sprung CG height so that the whole robot CG = load_case
  const rc = parts.caster.wheel_dia/2000, cy = L.caster.y/1000;
  const pts = [...every(prof.sprung_bottom, ([y, z]) => [y/1000, z/1000-hs]), ...faces(prof, (y, z) => [y/1000, z/1000-hs])];
  const un = every(prof.unsprung_bottom, ([y, z]) => [y/1000, z/1000-R]);
  return {
    model: 'eco', name: 'Eco 모델 ' + L.revision,
    m, mc, zcg, R, hs, ref: [0, hs],
    I: mc*(0.6**2+0.35**2)/12,                         // assumed: uniform box 600 x 350 mm
    Iw: 2*0.0075,                                      // assumed: outer rotor + tyre per wheel
    ...tyre(lc),
    subs: [{kind: 'slide', name: 'drive', m: mw, au: 0, aw: R-hs, w0: -0.0005,
            F0: 2*sp.target_force_N, ks: 2*sp.rate*1000, cs: 30, smax: dr.travel_mm/2000, kst: 4e5, cst: 800}],
    drive: {b: 1, u: 0, w: 0},
    casters: [{b: 0, u: cy, w: rc-hs, tag: 'front'}, {b: 0, u: -cy, w: rc-hs, tag: 'rear'}], rc,
    points: [{b: 0, pts}, {b: 1, pts: un}],
    T1: motor(dm), tipLever: cy,
    source: {mass_kg: m, cg_height_mm: lc.cg_height_mm, friction_mu: lc.friction_mu, caster_y_mm: L.caster.y,
             spring: sp.sku, travel_mm: dr.travel_mm, motor: dm.model, stall_torque_Nm: dm.stall_torque_Nm, rated_torque_Nm: dm.rated_torque_Nm},
    doc: {
      structure: '차체 + 구동륜 수직 슬라이드(좌우 묶음) + 고정 캐스터 4개(앞뒤 2쌍)',
      bodies: [['차체(스프링 위)', mc, 'm − 구동부'], ['구동부(바퀴·축판·하우징, 좌우 합)', mw, `layout drive.unsprung_mass_kg_estimate ${dr.unsprung_mass_kg_estimate} kg × 2`]],
      params: [
        ['전체 질량', m+' kg', 'layout load_case', 'design'],
        ['무게중심 높이(전체)', lc.cg_height_mm+' mm', 'layout load_case (추정)', 'assumed'],
        ['차체 무게중심 높이', (hs*1000).toFixed(1)+' mm', '전체 무게중심이 위 값이 되도록 계산', 'derived'],
        ['차체 피치 관성', (mc*(0.6**2+0.35**2)/12).toFixed(2)+' kg·m²', '600×350 균일 상자 가정', 'assumed'],
        ['구동륜 지름', dm.tire_dia+' mm', 'ZLLG65 도면', 'catalog'],
        ['구동 스프링', `${sp.sku} 공칭 ${sp.target_force_N} N, ${sp.rate} N/mm (×2)`, 'MISUMI 카탈로그 / layout', 'catalog'],
        ['구동륜 스트로크', '±'+dr.travel_mm/2+' mm', 'layout drive.travel_mm (CAD 스토퍼)', 'design'],
        ['캐스터 위치 / 지름', `Y ±${L.caster.y} mm / Ø${parts.caster.wheel_dia}`, 'layout / GL-65-ASF', 'design'],
      ],
    },
  };
}

// ---- Pro (cad/layout.json, bogie concept): chassis + 2 bogies (front caster + drive wheel) + 2 sprung rear casters
export function proParams(L, parts, prof, bogieReport) {
  const S = L.suspension, q = S.bogie, ca = S.caster, LP = L.parameters, dm = parts.drive_motor;
  const lc = {mass_kg: S.design_mass_kg, cg_height_mm: 250, friction_mu: 0.6};   // same assumptions as the Eco load_case
  const m = lc.mass_kg, R = dm.tire_dia/2000, rc = parts.caster.wheel_dia/2000, zcg = lc.cg_height_mm/1000;
  const cy = LP.caster_y/1000, yp = q.pivot_y_mm/1000, zp = q.pivot_z_mm/1000;
  // bogie (per side 5.2 kg estimate): hub motor 3.15 kg at the axle, caster ~0.6 kg at its wheel, the rest is the
  // 3t link + U bracket + holder inner parts around Y=100 mm -> CG (assumed)
  const mbS = q.unsprung_mass_kg_estimate, mMot = dm.mass_kg_reference, mCas = 0.6, mLink = mbS-mMot-mCas;
  const yb = (mMot*0+mCas*cy+mLink*0.10)/mbS, zb = (mMot*R+mCas*0.045+mLink*0.085)/mbS;
  const Ib = 2*(mMot*((0-yb)**2+(R-zb)**2)+mCas*((cy-yb)**2+(0.045-zb)**2)+mLink*((0.10-yb)**2+(0.085-zb)**2+0.29**2/12));
  const mb = 2*mbS, mr = 2*ca.unsprung_mass_kg_estimate, mc = m-mb-mr;
  const yr = -cy, zr = rc;                                  // rear arm unsprung mass lumped at the caster wheel centre
  const ycs = -(mb*yb+mr*yr)/mc;                            // sprung CG Y so that the whole robot CG is centred
  const hs = (m*zcg-mb*zb-mr*zr)/mc;
  // bogie spring (Rev17 SWU21-60 coaxial with the rear stopper): pushes the link down behind the pivot
  const bs = bogieReport?.bogie_spring, sp = q.spring;
  const arm = (bs?.arm_behind_pivot_mm ?? (q.pivot_y_mm-q.stopper_rear_xy_mm[1]))/1000;
  const Fb0 = bs ? bs.forces_N[1] : sp.rate_N_mm*(sp.free_length_mm-51);
  const amax = Math.asin(q.travel_mm/2/(LP.caster_y-q.pivot_y_mm));
  const kStop = 3e5;                                        // N/m per urethane stopper (assumed)
  // rear caster hinge module: wheel force = spring force x lever ratio, wheel rate = rate x ratio^2
  const rs = bogieReport?.rear_caster_spring, ratio = rs?.lever_ratio ?? 0.7222;
  const Fr0 = (rs ? rs.forces_N[1] : ca.rate_N_mm*(ca.free_length_mm-ca.loaded_length_mm+ca.shim_mm))*ratio;
  const kr = ca.rate_N_mm*ratio*ratio*1000;
  const toC = (y, z) => [y-ycs, z-hs];
  const pts = [...every(prof.sprung_bottom, ([y, z]) => toC(y/1000, z/1000)), ...faces(prof, (y, z) => toC(y/1000, z/1000))];
  const bog = every(prof.bogie_bottom, ([y, z]) => [y/1000-yb, z/1000-zb]);
  const rear = every(prof.rear_bottom, ([y, z]) => [y/1000-yr, z/1000-zr]);
  return {
    model: 'pro', name: 'Pro 모델 Rev' + String(L.revision).replace('.', ''),
    m, mc, zcg, R, hs, ref: [ycs, hs],
    I: mc*(0.6**2+0.35**2)/12,
    Iw: 2*0.0075,
    ...tyre(lc),
    subs: [
      {kind: 'pivot', name: 'bogie', m: mb, I: Ib, pc: toC(yp, zp), pb: [yp-yb, zp-zb], cg: [yb, zb],
       F0: 2*Fb0, k: 2*sp.rate_N_mm*1000, arm, cr: 2, amax,
       // rotation stops: C-USLH10 urethane (assumed 3e5 N/m each, x2 sides) at the front stopper (nose-up, 40 mm ahead of
       // the pivot) and the rear stopper (nose-down, 85 mm behind); damping ~30 % of critical (assumed)
       ksrUp: 2*kStop*((q.stopper_front_xy_mm[1]-q.pivot_y_mm)/1000)**2, ksrDown: 2*kStop*((q.pivot_y_mm-q.stopper_rear_xy_mm[1])/1000)**2,
       csrUp: 5, csrDown: 11,
       frontArm: (LP.caster_y-q.pivot_y_mm)/1000},
      {kind: 'slide', name: 'rear', m: mr, au: yr-ycs, aw: zr-hs, w0: 0,
       F0: 2*Fr0, ks: 2*kr, cs: 20, smax: ca.travel_mm/2000, kst: 4e5, cst: 800},
    ],
    drive: {b: 1, u: -yb, w: R-zb},
    casters: [{b: 1, u: cy-yb, w: rc-zb, tag: 'front'}, {b: 2, u: 0, w: 0, tag: 'rear'}], rc,
    points: [{b: 0, pts}, {b: 1, pts: bog}, {b: 2, pts: rear}],
    T1: motor(dm), tipLever: cy,
    source: {mass_kg: m, cg_height_mm: lc.cg_height_mm, friction_mu: lc.friction_mu, caster_y_mm: LP.caster_y,
             bogie_pivot_y_mm: q.pivot_y_mm, bogie_spring: sp.sku, rear_spring: ca.spring_sku, bogie_travel_front_caster_mm: q.travel_mm,
             rear_travel_mm: ca.travel_mm, motor: dm.model, stall_torque_Nm: dm.stall_torque_Nm, rated_torque_Nm: dm.rated_torque_Nm},
    doc: {
      structure: '차체 + 좌우 보기 링크(구동륜 + 앞 캐스터, 피벗 Y+60, 스프링) + 뒤 캐스터 경첩·스프링 모듈',
      bodies: [['차체(스프링 위)', mc, 'm − 보기 − 뒤 캐스터 암'], ['보기 링크(구동륜·모터·앞 캐스터 포함, 좌우 합)', mb, `layout bogie.unsprung_mass_kg_estimate ${mbS} kg × 2`],
               ['뒤 캐스터 암(좌우 합)', mr, `layout caster.unsprung_mass_kg_estimate ${ca.unsprung_mass_kg_estimate} kg × 2`]],
      params: [
        ['전체 질량', m+' kg', 'layout suspension.design_mass_kg', 'design'],
        ['무게중심 높이(전체)', lc.cg_height_mm+' mm', 'Eco load_case와 같은 가정(Pro 손계산 피치 검토는 200 mm 가정)', 'assumed'],
        ['차체 무게중심 (Y, Z)', `(${(ycs*1000).toFixed(1)}, ${(hs*1000).toFixed(1)}) mm`, '전체 무게중심이 Y0·위 높이가 되도록 계산', 'derived'],
        ['차체 피치 관성', (mc*(0.6**2+0.35**2)/12).toFixed(2)+' kg·m²', '600×350 균일 상자 가정', 'assumed'],
        ['보기 무게중심 (Y, Z) / 관성', `(${(yb*1000).toFixed(0)}, ${(zb*1000).toFixed(0)}) mm / ${Ib.toFixed(3)} kg·m²`, '모터 3.15 kg·캐스터 0.6 kg·링크 나머지 분포 가정', 'assumed'],
        ['보기 피벗', `Y ${q.pivot_y_mm} · Z ${q.pivot_z_mm} mm`, 'layout bogie', 'design'],
        ['보기 회전 한계', `앞 캐스터 ±${q.travel_mm/2} mm (±${(amax*180/Math.PI).toFixed(2)}°)`, 'layout bogie.travel_mm (C-USLH10 스토퍼)', 'design'],
        ['보기 스프링', `${sp.sku} ${Fb0.toFixed(1)} N @공칭, ${sp.rate_N_mm} N/mm, 피벗 뒤 ${(arm*1000).toFixed(0)} mm (×2)`, 'MISUMI 카탈로그 / bogie_report', 'catalog'],
        ['뒤 캐스터 스프링', `${ca.spring_sku} 바퀴 환산 ${Fr0.toFixed(1)} N, ${(kr/1000).toFixed(2)} N/mm, ±${ca.travel_mm/2} mm (×2)`, '카탈로그 × 레버비 '+ratio.toFixed(3), 'catalog'],
        ['구동륜 지름', dm.tire_dia+' mm', 'ZLLG65 도면', 'catalog'],
        ['캐스터 위치 / 지름', `Y ±${LP.caster_y} mm / Ø${parts.caster.wheel_dia}`, 'layout / GL-65-ASF', 'design'],
      ],
    },
  };
}

// ---- terrain -------------------------------------------------------------------------------------------
export const TESTS = {
  slope: {label: '경사 주행', unit: '°', note: '평지 → 경사(길이 1.4 m, 꺾임 모서리 그대로) → 평지. 경사 중간에서 정지 1 s 후 재출발'},
  ramp:  {label: '경사 위 출발', unit: '°', note: '긴 경사면 위에서 출발 → 정지 1 s → 재출발(꺾임 없음, 순수 등판력)'},
  step:  {label: '단차 오르기', unit: 'mm', note: '직각 단차 1개, 폭 전체'},
  bump:  {label: '방지턱 넘기', unit: 'mm', note: '사다리꼴 방지턱: 앞뒤 경사 1:2(26.6°), 윗면 40 mm'},
  estop: {label: 'E-stop 급정지', unit: 'm/s²', note: '평지, 주행 속도에서 비상정지'}
};

export function terrain(kind, value, variant) {
  let p;
  if (kind === 'slope' && variant === 'ramp') {
    const t = Math.tan(value*Math.PI/180);
    return {pts: [[-4, -4*t], [10, 10*t]], goal: 1.6, holdAt: 0.8, xs: [], start: Math.atan(t)};
  }
  if (kind === 'slope') {
    const t = Math.tan(value*Math.PI/180), x0 = 0.9, Lr = 1.4;
    p = [[-3, 0], [x0, 0], [x0+Lr, Lr*t], [x0+Lr+4, Lr*t]];
    return {pts: p, goal: x0+Lr+0.9, holdAt: x0+Lr/2, xs: [x0, x0+Lr]};
  }
  if (kind === 'step') {
    const h = value/1000, x0 = 0.7;
    p = [[-3, 0], [x0, 0], [x0, h], [x0+5, h]];
    return {pts: p, goal: x0+0.75, xs: [x0]};
  }
  if (kind === 'bump') {
    const h = value/1000, x0 = 0.7, run = 2*h, top = 0.04;
    p = [[-3, 0], [x0, 0], [x0+run, h], [x0+run+top, h], [x0+2*run+top, 0], [x0+6, 0]];
    return {pts: p, goal: x0+2*run+top+0.75, xs: [x0, x0+2*run+top]};
  }
  p = [[-3, 0], [40, 0]];
  return {pts: p, goal: Infinity, xs: []};
}

function segments(pts) {
  const s = [];
  for (let i = 0; i < pts.length-1; i++) {
    const [ax, az] = pts[i], [bx, bz] = pts[i+1], dx = bx-ax, dz = bz-az, len = Math.hypot(dx, dz);
    if (len < 1e-9) continue;
    s.push({ax, az, dx, dz, len2: len*len, nx: -dz/len, nz: dx/len,
            xmin: Math.min(ax, bx), xmax: Math.max(ax, bx), zmax: Math.max(az, bz)});
  }
  return s;
}

export function groundZ(pts, x) {
  let z = pts[0][1];
  for (let i = 0; i < pts.length-1; i++) {
    const [ax, az] = pts[i], [bx, bz] = pts[i+1];
    if (x >= ax && x <= bx) z = bx > ax ? az+(bz-az)*(x-ax)/(bx-ax) : Math.max(az, bz);
    else if (x > bx) z = bz;
  }
  return z;
}

// circle (cx, cz, r) against the terrain polyline -> CB[4k..4k+3] = (nx, nz, pen, obstacle); returns the contact count.
// obstacle = 1 for a vertex (edge) contact or a face steeper than 30 deg (step riser), 0 for a walkable face.
// Vertex contacts shared by two segments are counted once.
const CB = new Float64Array(64);
function circleContacts(cx, cz, r, segs) {
  let n = 0, lastQx = NaN, lastQz = NaN;
  for (let i = 0; i < segs.length; i++) {
    const g = segs[i];
    if (cx+r < g.xmin || cx-r > g.xmax || cz-r > g.zmax) continue;
    let t = ((cx-g.ax)*g.dx+(cz-g.az)*g.dz)/g.len2;
    t = t < 0 ? 0 : t > 1 ? 1 : t;
    const qx = g.ax+t*g.dx, qz = g.az+t*g.dz, ex = cx-qx, ez = cz-qz, d = Math.sqrt(ex*ex+ez*ez);
    const side = ex*g.nx+ez*g.nz;
    if (side < 0 && t > 0 && t < 1) { CB[4*n] = g.nx; CB[4*n+1] = g.nz; CB[4*n+2] = r+d; CB[4*n+3] = Math.abs(g.nx) > 0.5 ? 1 : 0; n++; continue; }   // centre inside
    if (d >= r || d < 1e-9) continue;
    const end = t === 0 || t === 1;
    if (end && Math.abs(qx-lastQx) < 1e-9 && Math.abs(qz-lastQz) < 1e-9) continue;
    if (end) { lastQx = qx; lastQz = qz; }
    CB[4*n] = ex/d; CB[4*n+1] = ez/d; CB[4*n+2] = r-d; CB[4*n+3] = end || Math.abs(g.nx) > 0.5 ? 1 : 0; n++;
    if (n >= 15) break;
  }
  return n;
}

export function torqueLimit(P, w, mode) {
  const T = P.T1, peak = mode === 'rated' ? T.rated : T.peak;
  const back = T.rated*(T.w0-Math.abs(w))/(T.w0-T.wr);
  return 2*Math.max(0, Math.min(peak, back));
}

// ---- speed command profiles ----------------------------------------------------------------------------
function speedProfile(kind, opt, terr) {
  const v = opt.speed, acc = opt.accel ?? 0.6, settle = 0.4;
  if (kind === 'slope') {
    // drive, decelerate to stand still at the middle of the ramp, hold 1 s, restart
    let phase = 0, tStop = 0, vRef = 0;
    return (t, X, dt) => {
      if (t < settle) return 0;
      if (phase === 0) { vRef = Math.min(v, vRef+acc*dt); if (X >= terr.holdAt-vRef*vRef/(2*acc)) phase = 1; }
      else if (phase === 1) { vRef = Math.max(0, vRef-acc*dt); if (vRef === 0) { phase = 2; tStop = t; } }
      else if (phase === 2) { if (t-tStop >= 1.0) phase = 3; }
      else vRef = Math.min(v, vRef+acc*dt);
      return vRef;
    };
  }
  if (kind === 'estop') {
    const tCruise = settle+v/acc+0.6, decel = opt.value;
    return (t) => {
      if (t < settle) return 0;
      if (t < tCruise) return Math.min(v, (t-settle)*acc);
      return Math.max(0, v-(t-tCruise)*decel);
    };
  }
  return (t) => t < settle ? 0 : Math.min(v, (t-settle)*acc);
}

// ---- simulation ------------------------------------------------------------------------------------------
// bodies: B[0] = chassis, B[i] = P.subs[i-1]; state per body: x, z (CG), th, vx, vz, om (slide bodies copy the chassis pitch)
function makeBodies(P, terr) {
  const a = terr.start || 0, ca = Math.cos(a), sa = Math.sin(a);
  const ch = {x: -P.hs*sa, z: P.hs*ca+0.0001, th: a, vx: 0, vz: 0, om: 0, m: P.mc, I: P.I, kind: 'chassis'};
  const B = [ch];
  const at = (u, w) => [ch.x+u*ca-w*sa, ch.z+u*sa+w*ca];
  for (const s of P.subs) {
    const b = {...s, vx: 0, vz: 0, om: 0, th: a};
    if (s.kind === 'slide') {
      const [x, z] = at(s.au, s.aw); b.x = x-s.w0*sa; b.z = z+s.w0*ca;
    } else {
      const [x, z] = at(s.pc[0], s.pc[1]); b.x = x-(s.pb[0]*ca-s.pb[1]*sa); b.z = z-(s.pb[0]*sa+s.pb[1]*ca);
    }
    B.push(b);
  }
  return B;
}

// drive wheel displacement along the chassis normal relative to its nominal position, and its lower limit
function driveStroke(P) {
  const d = P.drive, s = P.subs[d.b-1];
  if (!s) return {min: -Infinity};
  if (s.kind === 'slide') return {min: -s.smax};
  // pivot: wheel point relative to the pivot, rotated by +-amax
  const ru = d.u-s.pb[0], rw = d.w-s.pb[1];
  const dz = a => ru*Math.sin(a)+rw*Math.cos(a)-rw;
  return {min: Math.min(dz(s.amax), dz(-s.amax)), pivot: true};
}

// kind: slope|ramp|step|bump|estop. opt: {value, speed, torque:'peak'|'rated', mode:'drive'|'ideal' (estop), mu}
export function simulate(P, kind0, opt) {
  const kind = kind0 === 'ramp' ? 'slope' : kind0;
  const terr = terrain(kind, opt.value, kind0 === 'ramp' ? 'ramp' : null), segs = segments(terr.pts);
  const mu = opt.mu ?? P.mu, ideal = kind === 'estop' && opt.mode === 'ideal';
  const settle = 0.4;
  const vprof = speedProfile(kind, opt, terr);
  const tEstop = settle+opt.speed/(opt.accel ?? 0.6)+0.6;
  const tmax = kind0 === 'ramp' ? 14 : kind === 'slope' ? 6+(terr.goal+0.5)/opt.speed+4 : kind === 'estop' ? tEstop+3 : 3+(terr.goal+0.5)/opt.speed+4;
  const B = makeBodies(P, terr), nb = B.length, ch = B[0];
  const F = new Float64Array(3*nb);           // fx, fz, M per body
  const Q = new Float64Array(nb);             // joint coordinate per sub-body (slide: m along the chassis normal, pivot: rad)
  const W = {phi: 0, w: 0, integ: 0, zb: 0};  // drive wheel spin, PI integrator, LuGre bristle
  const dr = P.drive, stroke = driveStroke(P);
  const dNom = [dr.u, dr.w];                  // wheel centre in its body frame
  const wNom = (() => { const s = P.subs[dr.b-1]; return s ? (s.kind === 'slide' ? s.aw : s.pc[1]+dr.w-s.pb[1]) : dr.w; })();
  const frames = [];
  let nextFrame = 0, t = 0, steps = 0;
  const hist = [];                    // [t, X] each 0.1 s for stall detection
  let nextHist = 0, result = null, holdX = null, holdMin = Infinity, liftR = 0, liftDur = 0, minNr = Infinity;
  let maxSlip = 0, scrape = 0, maxDecel = 0, vEstop = null, xEstop = null, xStopped = null, tStopped = null, prevV = 0, decelF = 0;
  const lastEdge = {drive: 0, front: 0, rear: 0, body: 0};
  const edge = {drive: 0, front: 0, rear: 0, body: 0};   // last |n_x| of each contact (pressing on an edge/riser)
  let wheelLiftDur = 0, maxPitchRel = 0, torqueSat = 0, dStroke = 0;

  while (t < tmax) {
    const vref = vprof(t, ch.x, DT);
    const locked = ideal && t >= tEstop, muBrake = locked ? opt.value/G : null;
    F.fill(0);
    F[1] = -P.mc*G;
    for (let i = 1; i < nb; i++) { F[3*i+1] = -B[i].m*G; if (B[i].kind === 'slide') { B[i].th = ch.th; B[i].om = ch.om; } }
    const c = Math.cos(ch.th), s = Math.sin(ch.th);

    // joints
    for (let i = 1; i < nb; i++) {
      const b = B[i];
      if (b.kind === 'slide') {
        const ax = ch.x+b.au*c-b.aw*s, az = ch.z+b.au*s+b.aw*c;
        const dx = b.x-ax, dz = b.z-az, su = dx*c+dz*s, sw = -dx*s+dz*c;
        const vAx = ch.vx-ch.om*(az-ch.z), vAz = ch.vz+ch.om*(ax-ch.x);
        const rvx = b.vx-vAx, rvz = b.vz-vAz, vu = rvx*c+rvz*s, vw = -rvx*s+rvz*c;
        const Fp = -(P.kj*su+P.cj*vu);
        let Fs = -(b.F0+b.ks*sw)-b.cs*vw;
        if (sw > b.smax) Fs += -b.kst*(sw-b.smax)-b.cst*vw;
        else if (sw < -b.smax) Fs += b.kst*(-b.smax-sw)-b.cst*vw;
        const jx = Fp*c-Fs*s, jz = Fp*s+Fs*c;
        F[3*i] += jx; F[3*i+1] += jz;
        const qx = ax-sw*s, qz = az+sw*c;
        F[0] -= jx; F[1] -= jz; F[2] += -(qx-ch.x)*jz+(qz-ch.z)*jx;
        Q[i] = sw;
      } else {
        const cb = Math.cos(b.th), sb = Math.sin(b.th);
        const pcx = ch.x+b.pc[0]*c-b.pc[1]*s, pcz = ch.z+b.pc[0]*s+b.pc[1]*c;
        const pbx = b.x+b.pb[0]*cb-b.pb[1]*sb, pbz = b.z+b.pb[0]*sb+b.pb[1]*cb;
        const vcx = ch.vx-ch.om*(pcz-ch.z), vcz = ch.vz+ch.om*(pcx-ch.x);
        const vbx = b.vx-b.om*(pbz-b.z), vbz = b.vz+b.om*(pbx-b.x);
        const jx = -P.kj*(pbx-pcx)-P.cj*(vbx-vcx), jz = -P.kj*(pbz-pcz)-P.cj*(vbz-vcz);
        F[3*i] += jx; F[3*i+1] += jz; F[3*i+2] += (pbx-b.x)*jz-(pbz-b.z)*jx;
        F[0] -= jx; F[1] -= jz; F[2] -= (pcx-ch.x)*jz-(pcz-ch.z)*jx;
        const phi = b.th-ch.th, wr = b.om-ch.om;
        let tau = b.arm*Math.max(0, b.F0-b.k*b.arm*phi)-b.cr*wr;
        if (phi > b.amax) tau += -b.ksrUp*(phi-b.amax)-b.csrUp*wr;
        else if (phi < -b.amax) tau += b.ksrDown*(-b.amax-phi)-b.csrDown*wr;
        F[3*i+2] += tau; F[2] -= tau;
        Q[i] = phi;
      }
    }

    // motor
    const wb = B[dr.b], cw = Math.cos(wb.th), sw_ = Math.sin(wb.th);
    const wx = wb.x+dNom[0]*cw-dNom[1]*sw_, wz = wb.z+dNom[0]*sw_+dNom[1]*cw;
    const wvx = wb.vx-wb.om*(wz-wb.z), wvz = wb.vz+wb.om*(wx-wb.x);
    let T = 0, Tw = 0;
    if (locked) { W.w = 0; }          // ideal brake: wheels held, friction does the rest
    else {
      const wc = vref/P.R, e = wc-W.w;
      W.integ += e*DT;
      T = P.Kp*e+P.Ki*W.integ;
      const lim = torqueLimit(P, W.w, opt.torque || 'peak');
      if (Math.abs(T) > lim) { W.integ -= e*DT; T = Math.sign(T)*lim; if (lim > 0) torqueSat += DT; }
      Tw += T; F[3*dr.b+2] += T;      // reaction on the body that carries the motor stator
    }

    // tyre
    let Nd = 0, slipD = 0, fwx = 0, fwz = 0;
    let nc = circleContacts(wx, wz, P.R, segs);
    // the contact with the largest normal force carries the LuGre bristle state (sticks without creep);
    // a second contact (inside corner of a step) uses the regularised Coulomb law
    let kMain = -1, nMax = 0;
    for (let k = 0; k < nc; k++) {
      const N = Math.max(0, P.kt*CB[4*k+2]-P.ct*(wvx*CB[4*k]+wvz*CB[4*k+1]));
      if (N > nMax) { nMax = N; kMain = k; }
    }
    if (kMain < 0) W.zb = 0;
    for (let k = 0; k < nc; k++) {
      const nx = CB[4*k], nz = CB[4*k+1], pen = CB[4*k+2];
      const N = Math.max(0, P.kt*pen-P.ct*(wvx*nx+wvz*nz));
      const tx = nz, tz = -nx;
      const slip = wvx*tx+wvz*tz-W.w*P.R;
      const m_ = muBrake ?? mu;
      let Ft;
      if (k === kMain) {
        const g = m_*N;
        const zd = g > 1e-6 ? slip-P.sig0*Math.abs(slip)*W.zb/g : slip;
        W.zb += zd*DT;
        const zl = g/P.sig0; if (W.zb > zl) W.zb = zl; else if (W.zb < -zl) W.zb = -zl;
        Ft = -(P.sig0*W.zb+P.sig1*zd);
        if (Ft > g*1.05) Ft = g*1.05; else if (Ft < -g*1.05) Ft = -g*1.05;
      } else Ft = -m_*N*slip/Math.sqrt(slip*slip+P.eps*P.eps);
      fwx += N*nx+Ft*tx; fwz += N*nz+Ft*tz;
      if (!locked) Tw -= Ft*P.R;
      Nd += N; if (k === kMain) slipD = slip;
      if (N > 5 && CB[4*k+3]) edge.drive = Math.max(edge.drive, Math.abs(nx));
    }
    F[3*dr.b] += fwx; F[3*dr.b+1] += fwz; F[3*dr.b+2] += (wx-wb.x)*fwz-(wz-wb.z)*fwx;

    // casters
    let Nf = 0, Nr = 0, gapR = 1;
    for (const cs of P.casters) {
      const b = B[cs.b], cb = Math.cos(b.th), sb = Math.sin(b.th);
      const px = b.x+cs.u*cb-cs.w*sb, pz = b.z+cs.u*sb+cs.w*cb;
      nc = circleContacts(px, pz, P.rc, segs);
      if (cs.tag === 'rear' && kind === 'estop') gapR = pz-P.rc;
      for (let k = 0; k < nc; k++) {
        const nx = CB[4*k], nz = CB[4*k+1], pen = CB[4*k+2];
        const vx = b.vx-b.om*(pz-b.z), vz = b.vz+b.om*(px-b.x);
        const N = Math.max(0, P.kc*pen-P.cc*(vx*nx+vz*nz));
        const tx = nz, tz = -nx, vt = vx*tx+vz*tz;
        const m_ = muBrake ?? P.crr;
        const Ft = -m_*N*vt/Math.sqrt(vt*vt+P.eps*P.eps);
        const fx = N*nx+Ft*tx, fz = N*nz+Ft*tz, cx_ = px-P.rc*nx, cz_ = pz-P.rc*nz;
        F[3*cs.b] += fx; F[3*cs.b+1] += fz; F[3*cs.b+2] += (cx_-b.x)*fz-(cz_-b.z)*fx;
        if (cs.tag === 'front') Nf += N; else Nr += N;
        if (N > 5 && CB[4*k+3]) { if (cs.tag === 'front') edge.front = Math.max(edge.front, Math.abs(nx)); else edge.rear = Math.max(edge.rear, Math.abs(nx)); }
      }
    }
    // silhouettes: chassis underside / nose / tail and the moving carriages
    let bodyN = 0;
    for (const set of P.points) {
      const b = B[set.b], cb = Math.cos(b.th), sb = Math.sin(b.th), pts = set.pts;
      for (let q = 0; q < pts.length; q++) {
        const u = pts[q][0], w = pts[q][1];
        const px = b.x+u*cb-w*sb, pz = b.z+u*sb+w*cb;
        nc = circleContacts(px, pz, P.rp, segs);
        for (let k = 0; k < nc; k++) {
          const nx = CB[4*k], nz = CB[4*k+1], pen = CB[4*k+2];
          const vx = b.vx-b.om*(pz-b.z), vz = b.vz+b.om*(px-b.x);
          const N = Math.max(0, P.kp*pen-P.cp*(vx*nx+vz*nz));
          const tx = nz, tz = -nx, vt = vx*tx+vz*tz;
          const Ft = -P.muBody*N*vt/Math.sqrt(vt*vt+P.eps*P.eps);
          const fx = N*nx+Ft*tx, fz = N*nz+Ft*tz;
          F[3*set.b] += fx; F[3*set.b+1] += fz; F[3*set.b+2] += (px-b.x)*fz-(pz-b.z)*fx;
          if (set.b === 0) bodyN += N;
        }
      }
    }
    if (bodyN > 5) edge.body = 1;
    // slide bodies keep the chassis pitch: their moments go to the chassis
    for (let i = 1; i < nb; i++) if (B[i].kind === 'slide') { F[2] += F[3*i+2]; F[3*i+2] = 0; }
    // integrate (semi-implicit Euler)
    lastEdge.drive = edge.drive; lastEdge.front = edge.front; lastEdge.rear = edge.rear; lastEdge.body = edge.body;
    edge.drive = edge.front = edge.rear = edge.body = 0;
    for (let i = 0; i < nb; i++) {
      const b = B[i];
      b.vx += F[3*i]/b.m*DT; b.vz += F[3*i+1]/b.m*DT;
      if (b.kind !== 'slide') b.om += F[3*i+2]/b.I*DT;
      b.x += b.vx*DT; b.z += b.vz*DT;
      if (b.kind !== 'slide') b.th += b.om*DT;
    }
    if (!locked) W.w += Tw/P.Iw*DT;
    W.phi += W.w*DT;
    t += DT; steps++;

    // ---- monitors
    // drive wheel displacement along the chassis normal (negative = extended, wheel below nominal)
    dStroke = -(wx-ch.x)*s+(wz-ch.z)*c-wNom;
    const after = t > settle+0.05;
    // pitch relative to the chord of the terrain under the caster base (tip-over = > 0.8 rad, past the caster pivot)
    const lv = P.tipLever, rel = ch.th-Math.atan2(groundZ(terr.pts, ch.x+lv)-groundZ(terr.pts, ch.x-lv), 2*lv);
    if (after) {
      maxPitchRel = Math.max(maxPitchRel, Math.abs(rel));
      maxSlip = Math.max(maxSlip, Math.abs(slipD));
      if (Nd < 1 && vref > 0) wheelLiftDur += DT;
      if (bodyN > 1) scrape += DT;
    }
    if (kind === 'estop' && t >= tEstop) {
      if (xEstop === null) { xEstop = ch.x; vEstop = ch.vx; }
      if (Nr < 1) liftDur += DT;
      liftR = Math.max(liftR, gapR);
      minNr = Math.min(minNr, Nr);
      // decel from the chassis velocity, low-passed over ~20 ms
      const a = -(ch.vx-prevV)/DT; decelF += (a-decelF)*DT/0.02; maxDecel = Math.max(maxDecel, decelF);
      if (tStopped === null && ch.vx < 0.005 && t > tEstop+0.05) { tStopped = t; xStopped = ch.x; }
    }
    prevV = ch.vx;
    if (kind === 'slope' && t > settle) {
      // stand-still hold on the ramp: record the position while the command is zero
      if (vref === 0 && ch.x > 0.5) { if (holdX === null) holdX = ch.x; holdMin = Math.min(holdMin, ch.x); }
    }

    if (t >= nextFrame) {
      nextFrame += FRAME_DT;
      const f = [+t.toFixed(4), ch.x, ch.z, ch.th, wx, wz, dStroke, W.phi, ch.vx, Nd, Nf, Nr, T, slipD, bodyN, vref];
      for (let i = 1; i < nb; i++) f.push(Q[i]);
      frames.push(f);
    }
    if (t >= nextHist) { hist.push([t, ch.x]); nextHist += 0.1; }

    // ---- end conditions
    if (Math.abs(rel) > 0.8 || Math.abs(ch.th) > 1.3) { result = {pass: false, reason: 'tip', text: '전도'}; break; }
    if (kind !== 'estop' && ch.x >= terr.goal) { result = {pass: true, reason: 'ok', text: '통과'}; break; }
    if (kind !== 'estop' && ch.x < hist[0][1]-0.25) { result = {pass: false, reason: 'slide_back', text: '구동륜 미끄러짐(경사에서 밀려 내려감)'}; break; }
    if (kind === 'estop' && tStopped !== null && t > tStopped+0.8) break;
    if (kind !== 'estop' && t > settle+4 && hist.length > 30 && !(kind === 'slope' && vref === 0)) {
      const old = hist[hist.length-31];
      if (ch.x-old[1] < 0.02) {                // no progress in 3 s
        let why = 'stuck', text = '걸려서 정지';
        if (wheelLiftDur > 0.5 && Nd < 5) { why = 'wheel_lift'; text = '구동륜 들림(서스펜션 스트로크 초과)'; }
        else if (dStroke < stroke.min+0.0005 && Math.abs(slipD) > 0.05) { why = 'droop'; text = stroke.pivot ? '구동륜 하중 부족(보기 링크 스토퍼 도달'+(Nf < 1 ? ', 앞 캐스터 들림' : '')+')' : '구동륜 하중 부족(서스펜션 최대 신장)'; }
        else if (bodyN > 1) { why = 'high_centre'; text = '차체 하부·앞면 걸림'; }
        else if (Math.abs(slipD) > 0.05) { why = 'slip'; text = '구동륜 미끄러짐'; }
        else if (torqueSat > 1) { why = 'torque'; text = '모터 토크 부족'; }
        // which wheel is pressing on the obstacle edge
        const who = lastEdge.body ? '차체 하부' : lastEdge.front > 0.3 ? '앞 캐스터' : lastEdge.rear > 0.3 ? '뒤 캐스터' : lastEdge.drive > 0.3 ? '구동륜' : null;
        result = {pass: false, reason: why, text: who ? who+' 걸림 · '+text : text, blocker: who};
        break;
      }
    }
  }
  if (!result) {
    if (kind === 'estop') result = {pass: true, reason: 'ok', text: '정지'};
    else if (torqueSat > 1) result = {pass: false, reason: 'torque', text: '모터 토크 부족(등판 속도 유지 불가)'};
    else if (maxSlip > 0.1) result = {pass: false, reason: 'slip', text: '구동륜 미끄러짐(마찰 한계)'};
    else result = {pass: false, reason: 'timeout', text: '시간 초과'};
  }
  if (kind === 'slope' && holdX !== null && holdX-holdMin > 0.03 && result.pass)
    result = {pass: false, reason: 'rollback', text: '경사 정지 유지 불가(밀림)'};
  const metrics = {max_pitch_rel_deg: +(maxPitchRel*180/Math.PI).toFixed(2), max_slip_m_s: +maxSlip.toFixed(3),
                   wheel_lift_s: +wheelLiftDur.toFixed(2), body_contact_s: +scrape.toFixed(2), torque_sat_s: +torqueSat.toFixed(2),
                   t_end_s: +t.toFixed(2)};
  if (kind === 'slope' && holdX !== null) metrics.hold_rollback_mm = +((holdX-holdMin)*1000).toFixed(1);
  if (kind === 'estop') {
    const dist = tStopped !== null && xEstop !== null ? xStopped-xEstop : null;
    metrics.stop_time_s = tStopped !== null ? +(tStopped-tEstop).toFixed(3) : null;
    metrics.stop_distance_mm = dist === null ? null : +(dist*1000).toFixed(0);
    metrics.mean_decel_m_s2 = dist ? +(vEstop**2/(2*dist)).toFixed(2) : null;
    metrics.speed_at_estop_m_s = +vEstop.toFixed(3);
    metrics.peak_decel_m_s2 = +maxDecel.toFixed(2);
    metrics.rear_lift_mm = +(Math.max(0, liftR)*1000).toFixed(1);
    metrics.rear_unloaded_s = +liftDur.toFixed(3);
    metrics.min_rear_load_N = +minNr.toFixed(1);
    if (result.pass && liftR > 0.002) result = {pass: true, reason: 'lift', text: '뒤 캐스터 들림(복귀)'};
  }
  return {kind, opt, terrain: terr, result, metrics, frames, steps,
          fields: ['t', 'X', 'Z', 'th', 'xw', 'zw', 'stroke', 'phi', 'v', 'Nd', 'Nf', 'Nr', 'T', 'slip', 'bodyN', 'vref', ...P.subs.map(s => 'q_'+s.name)]};
}

// largest value that passes on a grid (first failure ends the search); returns {limit, firstFail, rows}
export function sweep(P, kind, opt, values, ok = r => r.result.pass) {
  const rows = [];
  let limit = null, firstFail = null;
  for (const v of values) {
    const r = simulate(P, kind, {...opt, value: v});
    rows.push({value: v, pass: ok(r), reason: r.result.reason, text: r.result.text, ...r.metrics});
    if (ok(r)) limit = v; else { firstFail = {value: v, reason: r.result.reason, text: r.result.text}; break; }
  }
  return {limit, firstFail, rows};
}
