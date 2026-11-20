"""Eco (P30) drive-test dynamics (web/drive-sim.js) sanity checks, run through Node when it is installed."""
import json
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
NODE = shutil.which('node')

SCRIPT = r"""
import {readFileSync} from 'node:fs';
import {ecoParams, simulate} from './web/drive-sim.js';
const read = p => JSON.parse(readFileSync(p, 'utf8'));
const P = ecoParams(read('cad/p30/layout.json'), read('cad/parts.json'), read('cad-output/p30/side_profile.json'));
const run = (k, o) => { const r = simulate(P, k, o); return {pass: r.result.pass, reason: r.result.reason, m: r.metrics, f0: r.frames[0], fl: r.frames.at(-1)}; };
console.log(JSON.stringify({
  m: P.m, zcg: P.zcg,
  flat: run('step', {value: 0, speed: 0.5}),
  ramp10: run('ramp', {value: 10, speed: 0.5}),
  ramp25: run('ramp', {value: 25, speed: 0.5}),
  step40: run('step', {value: 40, speed: 0.5}),
  drive: run('estop', {value: 20, speed: 1.0, mode: 'drive'}),
  ideal: run('estop', {value: 9, speed: 1.0, mode: 'ideal'}),
}));
"""


@pytest.fixture(scope='module')
def sim():
    if not NODE or not (ROOT/'cad-output/p30/side_profile.json').exists():
        pytest.skip('node or cad-output/p30/side_profile.json missing')
    out = subprocess.run([NODE, '--input-type=module', '-e', SCRIPT], cwd=ROOT, capture_output=True, text=True, timeout=120)
    assert out.returncode == 0, out.stderr
    return json.loads(out.stdout)


def test_static_load_split(sim):
    # standing still: drive tyre = 2 x 120 N spring + unsprung weight, the rest on the casters
    f = sim['flat']['fl']                     # last frame, cruising on the flat: [.., Nd, Nf, Nr, ..] at 9..11
    assert sim['flat']['pass']
    assert 300 < f[9] < 350 and abs(f[9]+f[10]+f[11]-55*9.81) < 15
    assert sim['m'] == 55 and abs(sim['zcg']-0.25) < 1e-9


def test_gradeability(sim):
    assert sim['ramp10']['pass'] and sim['ramp10']['m']['hold_rollback_mm'] < 30
    assert not sim['ramp25']['pass'] and sim['ramp25']['reason'] in ('slip', 'rollback', 'stuck', 'torque', 'slide_back')


def test_step_limited_by_caster(sim):
    assert not sim['step40']['pass']


def test_estop(sim):
    d = sim['drive']['m']
    # drive-wheel braking is friction-limited: mu x (spring preload + unsprung weight) / mass ~ 3.6 m/s2
    assert 2.5 < d['peak_decel_m_s2'] < 4.5 and d['rear_lift_mm'] < 2
    i = sim['ideal']
    assert i['pass'] and i['reason'] == 'lift' and i['m']['rear_lift_mm'] > 2   # rear casters lift, no tip-over
