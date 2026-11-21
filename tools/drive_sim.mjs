// Drive-test limits: runs the web/drive-sim.js model over parameter grids.
//   node tools/drive_sim.mjs [eco|pro|all]   (default all)
//   eco -> cad-output/p30/drive_sim.json  (needs cad-output/p30/side_profile.json: python -m cad.p30.side_profile)
//   pro -> cad-output/drive_sim.json      (needs cad-output/side_profile.json:     python -m cad.side_profile)
import {readFileSync, writeFileSync, existsSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {dirname, join} from 'node:path';
import {ecoParams, proParams, simulate, sweep, G} from '../web/drive-sim.js';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const read = p => JSON.parse(readFileSync(join(root, p), 'utf8'));
const range = (a, b, d) => { const o = []; for (let v = a; v <= b+1e-9; v += d) o.push(+v.toFixed(4)); return o; };

const MODELS = {
  eco: {out: 'cad-output/p30/drive_sim.json', params: () => ecoParams(read('cad/p30/layout.json'), read('cad/parts.json'), read('cad-output/p30/side_profile.json')),
        method: '2-D pitch-plane multibody (chassis + lumped drive wheels on the spring slide + rigid caster pairs)',
        stops: {label: '구동 슬라이드 고무 스토퍼 강성', scale: (P, f) => ({...P, subs: [{...P.subs[0], kst: P.subs[0].kst*f, cst: P.subs[0].cst*Math.sqrt(f)}]})}},
  pro: {out: 'cad-output/drive_sim.json',
        params: () => proParams(read('cad/layout.json'), read('cad/parts.json'), read('cad-output/side_profile.json'),
                                existsSync(join(root, 'cad-output/build_report.json')) ? read('cad-output/build_report.json').suspension_catalog : null),
        method: '2-D pitch-plane multibody (chassis + lumped bogie links carrying the drive wheels and front casters on a pin joint with the '
          + 'bogie spring and stops + lumped rear caster arms on a spring slide)',
        stops: {label: '보기 링크 우레탄 스토퍼 강성', scale: (P, f) => { const b = P.subs[0];
          return {...P, subs: [{...b, ksrUp: b.ksrUp*f, ksrDown: b.ksrDown*f, csrUp: b.csrUp*Math.sqrt(f), csrDown: b.csrDown*Math.sqrt(f)}, ...P.subs.slice(1)]}; }}},
};

function run(name) {
  const M = MODELS[name], P = M.params(), t0 = Date.now();
  const log = (k, r) => console.log(`[${name}] ${k}: limit ${r.limit} · first fail ${r.firstFail ? r.firstFail.value+' ('+r.firstFail.text+')' : '-'}`);
  const out = {model: P.name, generated: new Date().toISOString().slice(0, 10), assumptions: P.source,
    method: M.method+', penalty contacts on a full-width terrain, wheel speed PI with the motor torque limit. '
      + 'Planar: left = right, caster swivel offset ignored. CG height, inertia and tyre stiffness assumed.',
    tests: {}};
  for (const v of [0.3, 0.5, 1.0]) {
    const opt = {speed: v, torque: 'peak'};
    const sl = sweep(P, 'slope', opt, range(0, 25, 1)); log('slope v'+v, sl);
    const st = sweep(P, 'step', opt, range(0, 50, 1)); log('step v'+v, st);
    const bu = sweep(P, 'bump', opt, range(0, 70, 2)); log('bump v'+v, bu);
    out.tests['v'+v] = {speed_m_s: v, slope_deg: sl, step_mm: st, bump_mm: bu};
  }
  // pure gradeability (start, hold and restart on a long ramp): peak torque, and the rated 4 Nm per wheel
  const rp = sweep(P, 'ramp', {speed: 0.5, torque: 'peak'}, range(0, 30, 1)); log('ramp peak torque', rp);
  const rr = sweep(P, 'ramp', {speed: 0.5, torque: 'rated'}, range(0, 30, 1)); log('ramp rated torque', rr);
  out.tests.ramp = {speed_m_s: 0.5, peak_torque: rp, rated_torque: rr};
  // E-stop with the real drive: commanded decel through the wheel speed loop, floor mu from the layout
  const drive = [0.5, 1, 1.5, 2, 2.5, 3, 4, 6, 10, 20].map(a => {
    const r = simulate(P, 'estop', {speed: 1.0, value: a, mode: 'drive', torque: 'peak'});
    return {command_m_s2: a, text: r.result.text, ...r.metrics};
  });
  console.log(`[${name}] estop drive`, drive.map(d => `${d.command_m_s2}->${d.mean_decel_m_s2}/${d.rear_lift_mm}`).join(' '));
  // tip-over limit: ideal brake on all six wheels (friction = a/g at every contact)
  const okNoLift = r => r.result.pass && r.metrics.rear_lift_mm <= 2;
  const ideal = {};
  for (const v of [0.5, 1.0, 1.5, 2.0]) {
    const lift = sweep(P, 'estop', {speed: v, mode: 'ideal'}, range(0.5, 15, 0.5), okNoLift); log('estop no-lift v'+v, lift);
    const tip = sweep(P, 'estop', {speed: v, mode: 'ideal'}, range(0.5, 15, 0.5)); log('estop no-tip v'+v, tip);
    ideal['v'+v] = {speed_m_s: v, no_lift: lift, no_tip: tip};
  }
  out.tests.estop = {drive_speed_m_s: 1.0, drive_brake: drive, ideal,
    static_lift_m_s2_note: 'rear caster unloads when m a h = rear caster static load x caster base',
    static_tip_m_s2: +(G*P.tipLever/P.zcg).toFixed(2)};
  // sensitivity of the obstacle limits to the assumed end-stop stiffness (x1/3, x3; damping scaled with sqrt)
  out.sensitivity = {parameter: M.stops.label, factors: {}};
  for (const k of [0.33, 3]) {
    const Ps = M.stops.scale(P, k), row = {};
    for (const v of [0.3, 0.5, 1.0]) row['v'+v] = {slope_deg: sweep(Ps, 'slope', {speed: v}, range(0, 25, 1)).limit,
      step_mm: sweep(Ps, 'step', {speed: v}, range(0, 50, 1)).limit, bump_mm: sweep(Ps, 'bump', {speed: v}, range(0, 70, 2)).limit};
    out.sensitivity.factors['x'+k] = row;
    console.log(`[${name}] stops x${k}`, JSON.stringify(row));
  }
  // standing loads after the settle time (flat floor), for the hand-calculation comparison
  const f = simulate(P, 'estop', {speed: 0.5, value: 1, mode: 'drive'}).frames.find(x => x[0] >= 0.39);
  out.static_loads_N = {drive_pair: +f[9].toFixed(1), front_caster_pair: +f[10].toFixed(1), rear_caster_pair: +f[11].toFixed(1)};
  out.runtime_s = (Date.now()-t0)/1000;
  writeFileSync(join(root, M.out), JSON.stringify(out, null, 1));
  console.log(`[${name}] wrote ${M.out} in ${out.runtime_s} s`);
}

const which = process.argv[2] || 'all';
for (const n of which === 'all' ? Object.keys(MODELS) : [which]) run(n);
