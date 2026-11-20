// Eco drive-test limits: runs the web/drive-sim.js model over parameter grids and writes cad-output/p30/drive_sim.json.
// node tools/drive_sim.mjs            (needs cad-output/p30/side_profile.json: python -m cad.p30.side_profile)
import {readFileSync, writeFileSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {dirname, join} from 'node:path';
import {ecoParams, simulate, sweep, G} from '../web/drive-sim.js';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const read = p => JSON.parse(readFileSync(join(root, p), 'utf8'));
const P = ecoParams(read('cad/p30/layout.json'), read('cad/parts.json'), read('cad-output/p30/side_profile.json'));
const range = (a, b, d) => { const o = []; for (let v = a; v <= b+1e-9; v += d) o.push(+v.toFixed(4)); return o; };
const t0 = Date.now();
const log = (k, r) => console.log(`${k}: limit ${r.limit} · first fail ${r.firstFail ? r.firstFail.value+' ('+r.firstFail.text+')' : '-'}`);

const out = {model: P.name, generated: new Date().toISOString().slice(0, 10), assumptions: P.source,
  method: '2-D pitch-plane multibody (chassis + lumped drive wheels on the spring slide + rigid caster pairs), penalty contacts on a full-width terrain, '
    + 'wheel speed PI with the motor torque limit. Planar: left = right, caster swivel offset ignored. CG height, inertia and tyre stiffness assumed.',
  tests: {}};

const speeds = [0.3, 0.5, 1.0];
for (const v of speeds) {
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
console.log('estop drive', drive.map(d => `${d.command_m_s2}->${d.mean_decel_m_s2}/${d.rear_lift_mm}`).join(' '));
// tip-over limit: ideal brake on all six wheels (friction = a/g at every contact)
const okNoLift = r => r.result.pass && r.metrics.rear_lift_mm <= 2;
const ideal = {};
for (const v of [0.5, 1.0, 1.5, 2.0]) {
  const lift = sweep(P, 'estop', {speed: v, mode: 'ideal'}, range(0.5, 15, 0.5), okNoLift); log('estop no-lift v'+v, lift);
  const tip = sweep(P, 'estop', {speed: v, mode: 'ideal'}, range(0.5, 15, 0.5)); log('estop no-tip v'+v, tip);
  ideal['v'+v] = {speed_m_s: v, no_lift: lift, no_tip: tip};
}
out.tests.estop = {drive_speed_m_s: 1.0, drive_brake: drive, ideal,
  static_lift_m_s2_note: 'rear caster unloads when m a h = rear caster static load x caster base (drive load is fixed by the spring)',
  static_tip_m_s2: +(G*P.tipLever/P.zcg).toFixed(2)};
out.runtime_s = (Date.now()-t0)/1000;
writeFileSync(join(root, 'cad-output/p30/drive_sim.json'), JSON.stringify(out, null, 1));
console.log('wrote cad-output/p30/drive_sim.json in', out.runtime_s, 's');
