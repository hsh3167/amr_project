// Eco model drive-test dynamics: 2-D pitch-plane multibody simulation (side view, x = forward, z = up, SI units).
//
// Bodies: sprung chassis (x, z, pitch), the two drive wheels lumped into one (x, z, spin) held on the chassis
// vertical slide by a stiff penalty joint, preloaded spring and rubber end stops (+-10 mm), and the two caster
// pairs as rigid free-rolling circles on the chassis. Contacts are penalty springs against a polyline terrain
// that is uniform across the width; the chassis underside / nose / tail and the sliding axle carriage are
// sampled from the CAD silhouette (cad-output/p30/side_profile.json) so they can scrape or hang up.
// Drive: wheel speed PI control with the ZLLG65 torque limit (current limit at stall torque, falling to zero
// at no-load speed through the rated point).
// Scope: planar (left = right), swivel offset of the casters ignored, assumed CG / inertia / tyre stiffness,
// no motor thermal limit. It is a design-comparison tool, not a substitute for a prototype test.

export const G = 9.81;
const DT = 4e-4;            // s, semi-implicit Euler step
const FRAME_DT = 1/60;      // s, playback frame spacing

export function ecoParams(L, parts, prof) {
  const lc = L.load_case, dm = parts.drive_motor, dr = L.drive, sp = dr.spring;
  const m = lc.mass_kg, mw = 2*dr.unsprung_mass_kg_estimate, mc = m-mw;
  const R = dm.tire_dia/2000, zcg = lc.cg_height_mm/1000;
  const hs = (m*zcg-mw*R)/mc;                          // sprung CG height so that the whole robot CG = load_case
  const cas = parts.caster, rc = cas.wheel_dia/2000;
  const pts = [], un = [];
  const every = 20;                                    // mm between sampled silhouette points
  for (const [y, z] of prof.sprung_bottom) if (Math.round(y) % every === 0) pts.push([y/1000, z/1000-hs]);
  for (const end of [prof.nose, prof.tail]) {          // vertical row of points on the nose and tail faces
    for (let z = end.z_low; z <= 200; z += 25) pts.push([end.y/1000, z/1000-hs]);
  }
  for (const [y, z] of prof.unsprung_bottom) if (Math.round(y) % every === 0) un.push([y/1000, z/1000-R]);
  const nominalT = dm.rated_torque_Nm, peakT = dm.stall_torque_Nm;
  return {
    name: 'Eco ' + L.revision,
    m, mc, mw, hs, zcg, R,
    I: mc*(0.6**2+0.35**2)/12,                         // assumed: uniform box 600 x 350 mm
    Iw: 2*0.0075,                                      // assumed: outer rotor + tyre per wheel
    F0: 2*sp.target_force_N, ks: 2*sp.rate*1000, cs: 30,
    smax: dr.travel_mm/2000, kst: 4e5, cst: 800,
    kj: 2e6, cj: 2e3,
    kt: 2*3.0e5, ct: 1200,                             // assumed NBR tyre radial stiffness 300 N/mm per wheel
    casters: [{u: L.caster.y/1000, w: rc-hs}, {u: -L.caster.y/1000, w: rc-hs}], rc,
    kc: 2*4e5, cc: 1500, crr: 0.02,
    pts, un, rp: 0.005, kp: 3e6, cp: 3000, muBody: 0.3,
    mu: lc.friction_mu, eps: 0.05, sig0: 1e5, sig1: 400,           // tyre: LuGre bristle stiffness / damping (static friction)
    T1: {rated: nominalT, peak: peakT, w0: dm.no_load_rpm*Math.PI/30, wr: dm.rated_speed_rpm*Math.PI/30},
    Kp: 7, Ki: 35,
    tipLever: L.caster.y/1000,
    source: {mass_kg: m, cg_height_mm: lc.cg_height_mm, friction_mu: lc.friction_mu, caster_y_mm: L.caster.y,
             spring: sp.sku, travel_mm: dr.travel_mm, motor: dm.model, stall_torque_Nm: peakT, rated_torque_Nm: nominalT}
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

function groundSlope(pts, x) {
  for (let i = 0; i < pts.length-1; i++) {
    const [ax, az] = pts[i], [bx, bz] = pts[i+1];
    if (x >= ax && x < bx && bx > ax) return Math.atan2(bz-az, bx-ax);
  }
  return 0;
}

// circle (cx, cz, r) against the terrain polyline -> CB[3k..3k+2] = (nx, nz, pen); returns the contact count.
// Vertex contacts shared by two segments are counted once.
const CB = new Float64Array(48);
function circleContacts(cx, cz, r, segs) {
  let n = 0, lastQx = NaN, lastQz = NaN;
  for (let i = 0; i < segs.length; i++) {
    const g = segs[i];
    if (cx+r < g.xmin || cx-r > g.xmax || cz-r > g.zmax) continue;
    let t = ((cx-g.ax)*g.dx+(cz-g.az)*g.dz)/g.len2;
    t = t < 0 ? 0 : t > 1 ? 1 : t;
    const qx = g.ax+t*g.dx, qz = g.az+t*g.dz, ex = cx-qx, ez = cz-qz, d = Math.sqrt(ex*ex+ez*ez);
    const side = ex*g.nx+ez*g.nz;
    if (side < 0 && t > 0 && t < 1) { CB[3*n] = g.nx; CB[3*n+1] = g.nz; CB[3*n+2] = r+d; n++; continue; }   // centre inside
    if (d >= r || d < 1e-9) continue;
    const end = t === 0 || t === 1;
    if (end && Math.abs(qx-lastQx) < 1e-9 && Math.abs(qz-lastQz) < 1e-9) continue;
    if (end) { lastQx = qx; lastQz = qz; }
    CB[3*n] = ex/d; CB[3*n+1] = ez/d; CB[3*n+2] = r-d; n++;
    if (n >= 15) break;
  }
  return n;
}

function torqueLimit(P, w, mode) {
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
// kind: slope|step|bump|estop. opt: {value, speed, torque:'peak'|'rated', mode:'drive'|'ideal' (estop), mu}
export function simulate(P, kind0, opt) {
  const kind = kind0 === 'ramp' ? 'slope' : kind0;
  const terr = terrain(kind, opt.value, kind0 === 'ramp' ? 'ramp' : null), segs = segments(terr.pts);
  const mu = opt.mu ?? P.mu, ideal = kind === 'estop' && opt.mode === 'ideal';
  const settle = 0.4;
  const vprof = speedProfile(kind, opt, terr);
  const tEstop = settle+opt.speed/(opt.accel ?? 0.6)+0.6;
  const tmax = kind0 === 'ramp' ? 14 : kind === 'slope' ? 6+(terr.goal+0.5)/opt.speed+4 : kind === 'estop' ? tEstop+3 : 3+(terr.goal+0.5)/opt.speed+4;
  // state: chassis CG starts at x = 0, wheels at nominal (equilibrium is found during the settle time)
  const S = {X: 0, Z: P.hs, th: 0, vX: 0, vZ: 0, om: 0,
             xw: 0, zw: P.R-0.0004, vxw: 0, vzw: 0, phi: 0, wphi: 0, integ: 0, zb: 0};
  S.Z = P.hs+0.0001;
  if (terr.start) {                      // start at rest on the ramp: rotate the nominal pose about the ground origin
    const a = terr.start, ca = Math.cos(a), sa = Math.sin(a), wA = P.R-P.hs;
    S.th = a; S.X = -P.hs*sa; S.Z = P.hs*ca+0.0001;
    S.xw = S.X-wA*sa; S.zw = S.Z+wA*ca-0.0004;
  }
  const frames = [];
  let nextFrame = 0, t = 0, steps = 0;
  const hist = [];                    // [t, X] each 0.1 s for stall detection
  let nextHist = 0, result = null, holdX = null, holdMin = Infinity, maxTip = 0, liftR = 0, liftDur = 0, minNr = Infinity;
  let maxSlip = 0, scrape = 0, maxDecel = 0, vEstop = null, xEstop = null, xStopped = null, tStopped = null, prevV = 0, decelF = 0;
  const lastEdge = {drive: 0, front: 0, rear: 0, body: 0};
  const edge = {drive: 0, front: 0, rear: 0, body: 0};   // last |n_x| of each contact (pressing on an edge/riser)
  let wheelLiftDur = 0, bodyHit = 0, maxPitchRel = 0, torqueSat = 0, nTotal = 0;

  while (t < tmax) {
    const vref = vprof(t, S.X, DT);
    const c = Math.cos(S.th), s = Math.sin(S.th);
    let Fx = 0, Fz = -P.mc*G, M = 0, fwx = 0, fwz = -P.mw*G, Tw = 0;

    // slide joint
    const wA = P.R-P.hs, ax = S.X-wA*s, az = S.Z+wA*c;
    const eux = c, euz = s, ewx = -s, ewz = c;
    const dx = S.xw-ax, dz = S.zw-az, su = dx*eux+dz*euz, sw = dx*ewx+dz*ewz;
    const vAx = S.vX-S.om*(az-S.Z), vAz = S.vZ+S.om*(ax-S.X);
    const rvx = S.vxw-vAx, rvz = S.vzw-vAz, vu = rvx*eux+rvz*euz, vw = rvx*ewx+rvz*ewz;
    const Fp = -(P.kj*su+P.cj*vu);
    let Fs = -(P.F0+P.ks*sw)-P.cs*vw;
    if (sw > P.smax) Fs += -P.kst*(sw-P.smax)-P.cst*vw;
    else if (sw < -P.smax) Fs += P.kst*(-P.smax-sw)-P.cst*vw;
    const jx = Fp*eux+Fs*ewx, jz = Fp*euz+Fs*ewz;
    fwx += jx; fwz += jz;
    { const qx = ax+sw*ewx, qz = az+sw*ewz; Fx -= jx; Fz -= jz; M += -(qx-S.X)*jz+(qz-S.Z)*jx; }

    // motor
    let T = 0;
    if (ideal && t >= tEstop) { S.wphi = 0; }          // ideal brake: wheels held, friction does the rest
    else {
      const wc = vref/P.R, e = wc-S.wphi;
      S.integ += e*DT;
      T = P.Kp*e+P.Ki*S.integ;
      const lim = torqueLimit(P, S.wphi, opt.torque || 'peak');
      if (Math.abs(T) > lim) { S.integ -= e*DT; T = Math.sign(T)*lim; if (lim > 0) torqueSat += DT; }
      Tw += T; M += T;
    }
    const muBrake = ideal && t >= tEstop ? opt.value/G : null;

    // tyre
    let Nd = 0, slipD = 0;
    const locked = ideal && t >= tEstop;
    let nc = circleContacts(S.xw, S.zw, P.R, segs);
    // the contact with the largest normal force carries the LuGre bristle state (sticks without creep);
    // a second contact (inside corner of a step) uses the regularised Coulomb law
    let kMain = -1, nMax = 0;
    for (let k = 0; k < nc; k++) {
      const N = Math.max(0, P.kt*CB[3*k+2]-P.ct*(S.vxw*CB[3*k]+S.vzw*CB[3*k+1]));
      if (N > nMax) { nMax = N; kMain = k; }
    }
    if (kMain < 0) S.zb = 0;
    for (let k = 0; k < nc; k++) {
      const nx = CB[3*k], nz = CB[3*k+1], pen = CB[3*k+2];
      const N = Math.max(0, P.kt*pen-P.ct*(S.vxw*nx+S.vzw*nz));
      const tx = nz, tz = -nx;
      const slip = S.vxw*tx+S.vzw*tz-S.wphi*P.R;
      const m_ = muBrake ?? mu;
      let Ft;
      if (k === kMain) {
        const g = m_*N;
        const zd = g > 1e-6 ? slip-P.sig0*Math.abs(slip)*S.zb/g : slip;
        S.zb += zd*DT;
        const zl = g/P.sig0; if (S.zb > zl) S.zb = zl; else if (S.zb < -zl) S.zb = -zl;
        Ft = -(P.sig0*S.zb+P.sig1*zd);
        if (Ft > g*1.05) Ft = g*1.05; else if (Ft < -g*1.05) Ft = -g*1.05;
      } else Ft = -m_*N*slip/Math.sqrt(slip*slip+P.eps*P.eps);
      fwx += N*nx+Ft*tx; fwz += N*nz+Ft*tz;
      if (!locked) Tw -= Ft*P.R;
      Nd += N; if (k === kMain) slipD = slip;
      if (N > 5) edge.drive = Math.max(edge.drive, Math.abs(nx));
    }
    // unsprung carriage points (move with the wheel, pitch with the chassis)
    const un = P.un;
    for (let q = 0; q < un.length; q++) {
      const u = un[q][0], w = un[q][1];
      const px = S.xw+u*c-w*s, pz = S.zw+u*s+w*c;
      nc = circleContacts(px, pz, P.rp, segs);
      for (let k = 0; k < nc; k++) {
        const nx = CB[3*k], nz = CB[3*k+1], pen = CB[3*k+2];
        const vx = S.vxw-S.om*(pz-S.zw), vz = S.vzw+S.om*(px-S.xw);
        const N = Math.max(0, P.kp*pen-P.cp*(vx*nx+vz*nz));
        const tx = nz, tz = -nx, vt = vx*tx+vz*tz;
        const Ft = -P.muBody*N*vt/Math.sqrt(vt*vt+P.eps*P.eps);
        const fx = N*nx+Ft*tx, fz = N*nz+Ft*tz;
        fwx += fx; fwz += fz; M += (px-S.xw)*fz-(pz-S.zw)*fx;
      }
    }
    // casters
    let Nf = 0, Nr = 0, gapR = 1;
    for (let i = 0; i < 2; i++) {
      const cu = P.casters[i].u, cw = P.casters[i].w;
      const px = S.X+cu*c-cw*s, pz = S.Z+cu*s+cw*c;
      nc = circleContacts(px, pz, P.rc, segs);
      if (i === 1 && kind === 'estop') gapR = pz-P.rc;
      for (let k = 0; k < nc; k++) {
        const nx = CB[3*k], nz = CB[3*k+1], pen = CB[3*k+2];
        const vx = S.vX-S.om*(pz-S.Z), vz = S.vZ+S.om*(px-S.X);
        const N = Math.max(0, P.kc*pen-P.cc*(vx*nx+vz*nz));
        const tx = nz, tz = -nx, vt = vx*tx+vz*tz;
        const m_ = muBrake ?? P.crr;
        const Ft = -m_*N*vt/Math.sqrt(vt*vt+P.eps*P.eps);
        const fx = N*nx+Ft*tx, fz = N*nz+Ft*tz, cx_ = px-P.rc*nx, cz_ = pz-P.rc*nz;
        Fx += fx; Fz += fz; M += (cx_-S.X)*fz-(cz_-S.Z)*fx;
        if (i === 0) Nf += N; else Nr += N;
        if (N > 5) { if (i === 0) edge.front = Math.max(edge.front, Math.abs(nx)); else edge.rear = Math.max(edge.rear, Math.abs(nx)); }
      }
    }
    // chassis underside / nose / tail
    let bodyN = 0;
    const pts = P.pts;
    for (let q = 0; q < pts.length; q++) {
      const u = pts[q][0], w = pts[q][1];
      const px = S.X+u*c-w*s, pz = S.Z+u*s+w*c;
      nc = circleContacts(px, pz, P.rp, segs);
      for (let k = 0; k < nc; k++) {
        const nx = CB[3*k], nz = CB[3*k+1], pen = CB[3*k+2];
        const vx = S.vX-S.om*(pz-S.Z), vz = S.vZ+S.om*(px-S.X);
        const N = Math.max(0, P.kp*pen-P.cp*(vx*nx+vz*nz));
        const tx = nz, tz = -nx, vt = vx*tx+vz*tz;
        const Ft = -P.muBody*N*vt/Math.sqrt(vt*vt+P.eps*P.eps);
        const fx = N*nx+Ft*tx, fz = N*nz+Ft*tz;
        Fx += fx; Fz += fz; M += (px-S.X)*fz-(pz-S.Z)*fx;
        bodyN += N;
      }
    }
    if (bodyN > 5) edge.body = 1;
    // integrate (semi-implicit Euler)
    lastEdge.drive = edge.drive; lastEdge.front = edge.front; lastEdge.rear = edge.rear; lastEdge.body = edge.body;
    edge.drive = edge.front = edge.rear = edge.body = 0;
    S.vX += Fx/P.mc*DT; S.vZ += Fz/P.mc*DT; S.om += M/P.I*DT;
    S.X += S.vX*DT; S.Z += S.vZ*DT; S.th += S.om*DT;
    S.vxw += fwx/P.mw*DT; S.vzw += fwz/P.mw*DT;
    if (!locked) S.wphi += Tw/P.Iw*DT;
    S.xw += S.vxw*DT; S.zw += S.vzw*DT; S.phi += S.wphi*DT;
    t += DT; steps++;

    // ---- monitors
    const after = t > settle+0.05;
    // pitch relative to the chord of the terrain under the caster base (tip-over = > 0.8 rad, past the caster pivot)
    const lv = P.tipLever, rel = S.th-Math.atan2(groundZ(terr.pts, S.X+lv)-groundZ(terr.pts, S.X-lv), 2*lv);
    if (after) {
      maxPitchRel = Math.max(maxPitchRel, Math.abs(rel));
      maxSlip = Math.max(maxSlip, Math.abs(slipD));
      if (Nd < 1 && vref > 0) wheelLiftDur += DT;
      if (bodyN > 1) scrape += DT;
    }
    if (kind === 'estop' && t >= tEstop) {
      if (xEstop === null) { xEstop = S.X; vEstop = S.vX; }
      if (Nr < 1) liftDur += DT;
      liftR = Math.max(liftR, gapR);
      minNr = Math.min(minNr, Nr);
      // decel from the chassis velocity, low-passed over ~20 ms
      const a = -(S.vX-prevV)/DT; decelF += (a-decelF)*DT/0.02; maxDecel = Math.max(maxDecel, decelF);
      if (tStopped === null && S.vX < 0.005 && t > tEstop+0.05) { tStopped = t; xStopped = S.X; }
    }
    prevV = S.vX;
    if (kind === 'slope' && t > settle) {
      // stand-still hold on the ramp: record the position while the command is zero
      if (vref === 0 && S.X > 0.5) { if (holdX === null) holdX = S.X; holdMin = Math.min(holdMin, S.X); }
    }

    if (t >= nextFrame) {
      nextFrame += FRAME_DT;
      frames.push([+t.toFixed(4), S.X, S.Z, S.th, S.xw, S.zw, sw, S.phi, S.vX, Nd, Nf, Nr, T, slipD, bodyN, vref]);
    }
    if (t >= nextHist) { hist.push([t, S.X]); nextHist += 0.1; }

    // ---- end conditions
    if (Math.abs(rel) > 0.8 || Math.abs(S.th) > 1.3) { result = {pass: false, reason: 'tip', text: '전도'}; break; }
    if (kind !== 'estop' && S.X >= terr.goal) { result = {pass: true, reason: 'ok', text: '통과'}; break; }
    if (kind !== 'estop' && S.X < hist[0][1]-0.25) { result = {pass: false, reason: 'slide_back', text: '구동륜 미끄러짐(경사에서 밀려 내려감)'}; break; }
    if (kind === 'estop' && tStopped !== null && t > tStopped+0.8) break;
    if (kind !== 'estop' && t > settle+4 && hist.length > 30 && !(kind === 'slope' && vref === 0)) {
      const old = hist[hist.length-31];
      if (S.X-old[1] < 0.02) {                // no progress in 3 s
        let why = 'stuck', text = '걸려서 정지';
        if (wheelLiftDur > 0.5 && Nd < 5) { why = 'wheel_lift'; text = '구동륜 들림(서스펜션 스트로크 초과)'; }
        else if (sw < -P.smax+0.0005 && Math.abs(slipD) > 0.05) { why = 'droop'; text = '구동륜 하중 부족(서스펜션 최대 신장)'; }
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
          fields: ['t', 'X', 'Z', 'th', 'xw', 'zw', 'slide', 'phi', 'v', 'Nd', 'Nf', 'Nr', 'T', 'slip', 'bodyN', 'vref']};
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
