// Drive-test simulation panel shared by the Pro (index.html) and Eco (eco.html) viewers:
// controls, 3-D playback on the CAD model, HUD + load chart, video recording, limits table and the
// "계산 내용" dialog that explains the model, equations, parameters and criteria.
import * as THREE from 'three';
import {simulate, TESTS, DT, G} from './drive-sim.js?v=3';

const esc = s => String(s).replace(/[&<>"]/g, c => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;'}[c]));
const SIMCFG = {
  slope: {label: '경사각', min: 0, max: 25, step: 1, def: 6, unit: '°', variants: [['kink', '평지→경사 진입'], ['ramp', '경사 위 출발']]},
  step: {label: '단차 높이', min: 0, max: 50, step: 1, def: 15, unit: ' mm'},
  bump: {label: '방지턱 높이', min: 0, max: 70, step: 2, def: 30, unit: ' mm'},
  estop: {label: '감속 명령', min: 0.5, max: 15, step: 0.5, def: 3, unit: ' m/s²', variants: [['drive', '구동륜 제동 (실제)'], ['ideal', '이상 제동 (전도 한계)']]}
};
const nearest = (list, v) => list.reduce((a, b) => Math.abs(b-v) < Math.abs(a-v) ? b : a, list[0]);
const fmtN = v => Math.round(v)+' N';
const mmS = v => (v > 0 ? '+' : '')+(v*1000).toFixed(1)+' mm';

/**
 * o: {P, report, section, wrap, scene, camera, orbit, robot, floor, grid, canvas, label, filePrefix,
 *     pose(frame) (move the suspension / wheel nodes; robot is at identity), resetPose(), requestRender(), onStart(),
 *     extraRows: [[label, frame => text]], handLoads: {drive, front, rear, note} (pair loads, N) | null}
 */
export function setupDriveSim(o) {
  const {P, report, section, wrap, scene, camera, orbit, robot, floor, grid, canvas} = o;
  const $ = id => document.getElementById(id);
  section.innerHTML = `<h2>주행 동역학 시뮬레이션</h2>
    <div class="tabs" id="simTabs"><button class="control active" data-sim="slope">경사</button><button class="control" data-sim="step">단차</button><button class="control" data-sim="bump">방지턱</button><button class="control" data-sim="estop">E-stop</button></div>
    <div class="sim-variants" id="simVariants"></div>
    <p class="hint" id="simNote" style="margin-top:8px"></p>
    <div class="range-row"><span id="simParamLabel">경사각</span><strong id="simParamVal">—</strong></div>
    <input id="simParam" type="range" aria-label="시험 값">
    <div class="range-row"><span>주행 속도</span><strong id="simSpeedVal">0.5 m/s</strong></div>
    <input id="simSpeed" type="range" min="0.2" max="2" step="0.1" value="0.5" aria-label="주행 속도">
    <div class="btn-row"><button class="control" id="simRun">▶ 실행</button><button class="control" id="simAtLimit">한계값 실행</button><button class="control" id="simAtFail">실패값 실행</button></div>
    <div class="btn-row"><button class="control" id="simRecord">● 영상 저장</button><button class="control" id="simCalc">ⓘ 계산 내용</button></div>
    <p class="hint" id="simStatus" style="margin-top:8px"></p>
    <div id="simLimits" class="hint">한계값 보고서 불러오는 중…</div>`;
  const extra = o.extraRows || [];
  wrap.insertAdjacentHTML('beforeend', `<div id="simHud" class="sim-hud" hidden>
    <div class="sim-hud-head"><b id="hudTitle">—</b><span id="hudResult"></span></div>
    <div class="sim-hud-grid" id="hudGrid">
      <span>시간</span><b id="hudT">—</b><span>속도</span><b id="hudV">—</b>
      <span>피치</span><b id="hudPitch">—</b><span>구동륜 이동</span><b id="hudSlide">—</b>
      <span>구동륜</span><b id="hudNd">—</b><span>토크</span><b id="hudTq">—</b>
      <span>앞 캐스터</span><b id="hudNf">—</b><span>뒤 캐스터</span><b id="hudNr">—</b>
      ${extra.map(([l], i) => `<span>${esc(l)}</span><b id="hudX${i}">—</b>`).join('')}
    </div>
    <canvas id="hudChart" width="600" height="192"></canvas>
    <div class="sim-legend"><span><i style="background:#6aa8ff"></i>구동륜 하중</span><span><i style="background:#4fe0c9"></i>앞 캐스터</span><span><i style="background:#ffcc73"></i>뒤 캐스터</span><span><i style="background:#e9eefb"></i>속도</span></div>
    <div class="sim-ctl"><button data-rate="0.25">0.25×</button><button data-rate="0.5">0.5×</button><button data-rate="1" class="active">1×</button><button id="simReplay" title="다시 재생">↺</button><button id="simClose" title="시뮬레이션 종료">✕</button></div>
  </div>`);
  document.body.insertAdjacentHTML('beforeend', `<dialog id="simCalcDialog" aria-labelledby="simCalcTitle">
    <div class="bom-head"><h2 id="simCalcTitle">계산 내용 · ${esc(P?.name || '')} 주행 동역학</h2><div class="tabs"><button class="control" id="simCalcClose" aria-label="닫기">✕ 닫기</button></div></div>
    <div class="bom-body calc" id="simCalcBody"></div></dialog>`);

  const sim = {kind: 'slope', variant: 'kink', res: null, t: 0, rate: 1, playing: false, hold: 0, terrain: null, rec: null, recCanvas: null};
  const kindNow = () => sim.kind === 'slope' && sim.variant === 'ramp' ? 'ramp' : sim.kind;
  function limitFor(kind, variant, speed) {
    const T = report?.tests; if (!T) return null;
    if (kind === 'slope' && variant === 'ramp') return T.ramp?.peak_torque;
    if (kind === 'estop') return variant === 'ideal' ? T.estop?.ideal?.['v'+nearest([0.5, 1, 1.5, 2], speed)]?.no_lift : null;
    const key = {slope: 'slope_deg', step: 'step_mm', bump: 'bump_mm'}[kind]; return T['v'+nearest([0.3, 0.5, 1], speed)]?.[key];
  }
  function ui() {
    const c = SIMCFG[sim.kind], inp = $('simParam');
    section.querySelectorAll('[data-sim]').forEach(b => b.classList.toggle('active', b.dataset.sim === sim.kind));
    $('simVariants').innerHTML = (c.variants || []).map(([k, l]) => `<button class="control${k === sim.variant ? ' active' : ''}" data-variant="${k}">${l}</button>`).join('');
    $('simVariants').style.display = c.variants ? 'grid' : 'none';
    $('simVariants').querySelectorAll('[data-variant]').forEach(b => b.addEventListener('click', () => { sim.variant = b.dataset.variant; ui(); }));
    $('simParamLabel').textContent = c.label;
    if (inp.dataset.kind !== sim.kind) { inp.min = c.min; inp.max = c.max; inp.step = c.step; inp.value = c.def; inp.dataset.kind = sim.kind; }
    $('simParamVal').textContent = inp.value+c.unit; $('simSpeedVal').textContent = (+$('simSpeed').value).toFixed(1)+' m/s';
    const note = (TESTS[kindNow()] || TESTS[sim.kind]).note;
    $('simNote').textContent = note+(sim.kind === 'estop' ? (sim.variant === 'ideal' ? ' · 6바퀴 모두 마찰계수 a/g로 미끄러지며 제동(전도 한계 확인용 가정)' : ' · 구동륜 속도 제어로 감속, 바닥 μ '+(P?.mu ?? 0.6)+', 캐스터는 자유 회전') : '');
    const L = limitFor(sim.kind, sim.variant, +$('simSpeed').value);
    $('simAtLimit').disabled = !L || L.limit == null; $('simAtFail').disabled = !L?.firstFail;
  }
  section.querySelectorAll('[data-sim]').forEach(b => b.addEventListener('click', () => { sim.kind = b.dataset.sim; sim.variant = (SIMCFG[sim.kind].variants || [[null]])[0][0]; ui(); }));
  $('simParam').addEventListener('input', ui); $('simSpeed').addEventListener('input', ui);
  const runAt = which => { const L = limitFor(sim.kind, sim.variant, +$('simSpeed').value); if (!L) return; const v = which === 'fail' ? L.firstFail?.value : L.limit; if (v == null) return; $('simParam').value = v; ui(); run(); };
  $('simAtLimit').addEventListener('click', () => runAt('limit')); $('simAtFail').addEventListener('click', () => runAt('fail'));
  $('simRun').addEventListener('click', () => run());
  $('simRecord').addEventListener('click', () => { if (sim.rec) { stopRecording(); return; } run(true); });
  $('simCalc').addEventListener('click', () => { $('simCalcBody').innerHTML = calcHTML(P, report, sim.res, o.handLoads); $('simCalcDialog').showModal(); });
  $('simCalcClose').addEventListener('click', () => $('simCalcDialog').close());

  // terrain: extruded polyline across the width (only where it differs from the plain floor)
  function buildTerrain(terr) {
    if (sim.terrain) { scene.remove(sim.terrain); sim.terrain.traverse(x => { x.geometry?.dispose(); x.material?.dispose(); }); sim.terrain = null; }
    const pts = terr.pts.map(([x, z]) => [x*1000, z*1000]), minZ = Math.min(...pts.map(p => p[1]));
    const below = minZ < -1, raised = pts.some(p => p[1] > 0.5);
    floor.visible = grid.visible = !below;
    if (!below && !raised) return;
    let a = 0, b = pts.length-1;
    if (!below) { while (a < pts.length-2 && pts[a+1][1] <= 0.5) a++; let k = a+1; while (k < pts.length && pts[k][1] > 0.5) k++; if (k < pts.length) b = k; }
    let poly;
    if (below) { const t = (pts[1][1]-pts[0][1])/(pts[1][0]-pts[0][0]); poly = [[-1500, -1500*t], [4000, 4000*t]]; }
    else { const feat = Math.max(...terr.xs.map(v => v*1000), pts[a][0]+500); poly = pts.slice(a, b+1).filter(([x]) => x <= feat+2500); if (poly.at(-1)[0] < feat+2500 && b === pts.length-1) poly.push([feat+2500, pts[b][1]]); }
    const base = below ? poly[0][1]-40 : -2;
    const shape = new THREE.Shape(); shape.moveTo(poly[0][0], base); for (const [x, z] of poly) shape.lineTo(x, z+(below ? 0 : 0.6)); shape.lineTo(poly.at(-1)[0], base); shape.closePath();
    const Wd = 1000, geo = new THREE.ExtrudeGeometry(shape, {depth: Wd, bevelEnabled: false});
    const g = new THREE.Group();
    const mesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({color: 0x3a5876, roughness: .85, metalness: .05}));
    mesh.rotation.y = -Math.PI/2; mesh.position.x = Wd/2; g.add(mesh);
    const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo, 20), new THREE.LineBasicMaterial({color: 0x8fb3d9, transparent: true, opacity: .55}));
    edges.rotation.copy(mesh.rotation); edges.position.copy(mesh.position); g.add(edges);
    scene.add(g); sim.terrain = g;
  }
  function applyFrame(f) {
    const X = f[1], Z = f[2], th = f[3], [ry, rz0] = P.ref;
    robot.position.set(0, 0, 0); robot.rotation.set(0, 0, 0); robot.updateMatrixWorld(true);
    o.pose(f);
    const c = Math.cos(th), s = Math.sin(th);
    const ox = X-(ry*c-rz0*s), oz = Z-(ry*s+rz0*c);        // CAD origin (ground, centre) from the chassis CG
    robot.position.set(0, oz*1000, ox*1000); robot.rotation.set(-th, 0, 0);
    const d = new THREE.Vector3(0, oz*1000+170, ox*1000+200).sub(orbit.target); orbit.target.add(d); camera.position.add(d);
    hud(f);
  }
  function hud(f) {
    const t = f[0], th = f[3], v = f[8], r = sim.res;
    $('hudT').textContent = t.toFixed(2)+' s'; $('hudV').textContent = v.toFixed(2)+' m/s';
    $('hudPitch').textContent = (th*180/Math.PI).toFixed(1)+'°'; $('hudSlide').textContent = mmS(f[6]);
    $('hudNd').textContent = fmtN(f[9]); $('hudNf').textContent = fmtN(f[10]); $('hudNr').textContent = fmtN(f[11]); $('hudTq').textContent = (f[12]/2).toFixed(1)+' Nm/륜';
    extra.forEach(([, fn], i) => { $('hudX'+i).textContent = fn(f); });
    const done = t >= r.frames.at(-1)[0]-1e-6, res = $('hudResult');
    if (done) { res.textContent = r.result.text; res.className = r.result.pass ? 'sim-ok' : 'sim-bad'; } else { res.textContent = '주행 중'; res.className = 'sim-run'; }
    drawChart(t);
  }
  function drawChart(tNow) {
    const c = $('hudChart'), x = c.getContext('2d'), Wc = c.width, H = c.height, F = sim.res.frames, t1 = F.at(-1)[0] || 1;
    x.clearRect(0, 0, Wc, H);
    let nMax = 400; for (const f of F) nMax = Math.max(nMax, f[9], f[10], f[11]); nMax = Math.min(nMax, 900);
    let vMax = 0.5; for (const f of F) vMax = Math.max(vMax, Math.abs(f[8])); vMax *= 1.15;
    x.strokeStyle = 'rgba(150,175,215,.18)'; x.lineWidth = 1; for (let i = 1; i < 4; i++) { const y = H*i/4; x.beginPath(); x.moveTo(0, y); x.lineTo(Wc, y); x.stroke(); }
    const line = (idx, col, scale, dash) => { x.strokeStyle = col; x.lineWidth = 2.2; x.setLineDash(dash || []); x.beginPath(); F.forEach((f, i) => { const px = f[0]/t1*Wc, py = H-4-Math.max(0, Math.min(1, f[idx]/scale))*(H-10); i ? x.lineTo(px, py) : x.moveTo(px, py); }); x.stroke(); x.setLineDash([]); };
    line(9, '#6aa8ff', nMax); line(10, '#4fe0c9', nMax); line(11, '#ffcc73', nMax); line(8, 'rgba(233,238,251,.85)', vMax, [6, 5]);
    x.fillStyle = '#8394ad'; x.font = '20px system-ui,sans-serif'; x.textAlign = 'left'; x.fillText(Math.round(nMax)+' N', 6, 22); x.textAlign = 'right'; x.fillText(vMax.toFixed(1)+' m/s', Wc-6, 22); x.textAlign = 'left';
    const cx = tNow/t1*Wc; x.strokeStyle = 'rgba(255,255,255,.7)'; x.lineWidth = 1.5; x.beginPath(); x.moveTo(cx, 0); x.lineTo(cx, H); x.stroke();
  }
  function run(record = false) {
    if (!P) { $('simStatus').textContent = '시뮬레이션 데이터(side_profile.json)가 없습니다.'; return; }
    if (sim.rec) stopRecording(true);
    o.onStart?.();
    const kind = kindNow(), opt = {value: +$('simParam').value, speed: +$('simSpeed').value, torque: 'peak', mode: sim.variant};
    $('simStatus').textContent = '계산 중…';
    setTimeout(() => {
      const t0 = performance.now(), r = simulate(P, kind, opt), ms = performance.now()-t0;
      sim.res = r; sim.t = 0; sim.hold = 0; sim.playing = true;
      const c = SIMCFG[sim.kind];
      $('hudTitle').textContent = (TESTS[kind]?.label || '')+' '+opt.value+c.unit+' · '+opt.speed.toFixed(1)+' m/s'+(sim.kind === 'estop' ? ' · '+(opt.mode === 'ideal' ? '이상 제동' : '구동 제동') : '');
      const m = r.metrics, ext = kind === 'estop' ? ` · 정지거리 ${m.stop_distance_mm ?? '—'} mm · 평균 감속 ${m.mean_decel_m_s2 ?? '—'} m/s² · 뒤 캐스터 들림 ${m.rear_lift_mm} mm` : (kind === 'slope' || kind === 'ramp') ? (m.hold_rollback_mm != null ? ` · 정지 중 밀림 ${m.hold_rollback_mm} mm` : '') : ` · 최대 피치 ${m.max_pitch_rel_deg}°`;
      $('simStatus').innerHTML = `결과: <b class="${r.result.pass ? 'sim-ok' : 'sim-bad'}">${esc(r.result.text)}</b>${ext} · 계산 ${Math.round(ms)} ms`;
      buildTerrain(r.terrain); $('simHud').hidden = false;
      const rx = r.frames[0][1];
      camera.position.set(-1750, 520, rx*1000+380); orbit.target.set(0, 170, rx*1000+200); orbit.update();
      applyFrame(r.frames[0]);
      if (record) startRecording();
      o.requestRender();
    }, 30);
  }
  function tick(dt) {
    if (!sim.res || !sim.playing) return false;
    const F = sim.res.frames, tEnd = F.at(-1)[0];
    sim.t = Math.min(tEnd, sim.t+dt*sim.rate);
    let i = Math.min(F.length-1, Math.floor(sim.t*60)); while (i > 0 && F[i][0] > sim.t) i--;
    applyFrame(F[i]);
    if (sim.t >= tEnd) { applyFrame(F.at(-1)); sim.hold += dt; if (sim.hold > 1.2) { sim.playing = false; if (sim.rec) stopRecording(); } }
    return true;
  }
  document.querySelectorAll('#simHud [data-rate]').forEach(b => b.addEventListener('click', () => { sim.rate = +b.dataset.rate; document.querySelectorAll('#simHud [data-rate]').forEach(x => x.classList.toggle('active', x === b)); }));
  $('simReplay').addEventListener('click', () => { if (!sim.res) return; sim.t = 0; sim.hold = 0; sim.playing = true; o.requestRender(); });
  $('simClose').addEventListener('click', () => close());
  function close() {
    if (sim.rec) stopRecording(true);
    const was = !!sim.res;
    sim.res = null; sim.playing = false; $('simHud').hidden = true;
    if (sim.terrain) { scene.remove(sim.terrain); sim.terrain = null; }
    floor.visible = grid.visible = true; robot.position.set(0, 0, 0); robot.rotation.set(0, 0, 0); robot.updateMatrixWorld(true);
    if (was) o.resetPose();
    o.requestRender();
  }
  // video: composite the WebGL frame and the HUD into a 2-D canvas and record it (MP4 where supported, else WebM)
  function startRecording() {
    const types = ['video/mp4;codecs=avc1', 'video/mp4', 'video/webm;codecs=vp9', 'video/webm'];
    const type = window.MediaRecorder && types.find(t => MediaRecorder.isTypeSupported(t));
    if (!type) { $('simStatus').textContent += ' · 이 브라우저는 영상 녹화를 지원하지 않습니다.'; return; }
    const rc = document.createElement('canvas'); rc.width = canvas.width; rc.height = canvas.height; sim.recCanvas = rc;
    const chunks = [], rec = new MediaRecorder(rc.captureStream(60), {mimeType: type, videoBitsPerSecond: 8e6});
    rec.ondataavailable = e => { if (e.data.size) chunks.push(e.data); };
    rec.onstop = () => { if (rec.discard) return; const blob = new Blob(chunks, {type}), a = document.createElement('a'); a.href = URL.createObjectURL(blob);
      a.download = `${o.filePrefix}_${kindNow()}_${$('simParam').value}_${(+$('simSpeed').value).toFixed(1)}mps.${type.includes('mp4') ? 'mp4' : 'webm'}`; a.click(); setTimeout(() => URL.revokeObjectURL(a.href), 4000); };
    rec.start(200); sim.rec = rec; sim.recType = type; $('simRecord').textContent = '■ 녹화 중지'; $('simRecord').classList.add('recording');
  }
  function stopRecording(discard = false) {
    const rec = sim.rec; if (!rec) return; sim.rec = null; rec.discard = discard; rec.stop(); sim.recCanvas = null;
    $('simRecord').textContent = '● 영상 저장'; $('simRecord').classList.remove('recording');
  }
  function capture() {
    const rc = sim.recCanvas; if (!rc || !sim.rec) return;
    if (rc.width !== canvas.width || rc.height !== canvas.height) { rc.width = canvas.width; rc.height = canvas.height; }
    const x = rc.getContext('2d');
    x.fillStyle = '#0b1726'; x.fillRect(0, 0, rc.width, rc.height); x.drawImage(canvas, 0, 0);
    const cells = [...$('hudGrid').children].map(e => e.textContent), rows = [];
    for (let i = 0; i < cells.length; i += 4) rows.push(cells.slice(i, i+4));
    const s = rc.width/canvas.clientWidth, pad = 14*s, w = 330*s, left = 16*s, top = 16*s, chartH = (w-2*pad)*192/600, body = (rows.length*17+50)*s, h = pad*2+body+chartH;
    x.fillStyle = 'rgba(14,21,34,.82)'; x.fillRect(left, top, w, h);
    x.textAlign = 'left'; x.fillStyle = '#e9eefb'; x.font = `600 ${13*s}px system-ui,sans-serif`; x.fillText($('hudTitle').textContent, left+pad, top+pad+10*s);
    x.fillStyle = getComputedStyle($('hudResult')).color; x.fillText($('hudResult').textContent, left+pad, top+pad+30*s);
    x.fillStyle = '#b4c2d9'; x.font = `${12*s}px system-ui,sans-serif`;
    rows.forEach((r, i) => { const y = top+pad+52*s+i*17*s; x.fillText(r[0]+'  '+r[1], left+pad, y); if (r[2]) x.fillText(r[2]+'  '+r[3], left+pad+160*s, y); });
    x.drawImage($('hudChart'), left+pad, top+pad+body, w-2*pad, chartH);
    x.fillStyle = '#8394ad'; x.font = `${10*s}px system-ui,sans-serif`; x.fillText(o.label+' · 2D 동역학 시뮬레이션(가정 포함) · 파랑 구동륜 / 청록 앞 캐스터 / 노랑 뒤 캐스터 하중, 점선 속도', left, rc.height-10*s);
  }
  // limits table from the drive_sim.json report
  (function () {
    const T = report?.tests; if (!T) { $('simLimits').textContent = '한계값 보고서 없음 (node tools/drive_sim.mjs)'; return; }
    const cell = (kind, variant, v, L, unit) => {
      if (L?.limit == null) return '—';
      const pr = L.rows?.find(r => r.value === L.limit)?.max_pitch_rel_deg;
      const tip = (pr != null ? '한계값에서 최대 피치 '+pr+'°' : '')+(L.firstFail ? ' · 다음 값 '+L.firstFail.value+unit+': '+esc(L.firstFail.text) : ' · 탐색 범위 끝까지 통과');
      return `<button data-run="${kind}|${variant}|${v}|${L.limit}" title="${tip}">${L.firstFail ? '' : '≥ '}${L.limit}${unit}${pr > 15 ? '*' : ''}</button>`;
    };
    const sp = [0.3, 0.5, 1];
    let h = '<table class="sim-table"><thead><tr><th>시험 (통과 한계)</th>'+sp.map(v => `<th>${v} m/s</th>`).join('')+'</tr></thead><tbody>';
    h += '<tr><td>경사 (평지→경사)</td>'+sp.map(v => `<td>${cell('slope', 'kink', v, T['v'+v]?.slope_deg, '°')}</td>`).join('')+'</tr>';
    h += '<tr><td>단차</td>'+sp.map(v => `<td>${cell('step', '', v, T['v'+v]?.step_mm, ' mm')}</td>`).join('')+'</tr>';
    h += '<tr><td>방지턱</td>'+sp.map(v => `<td>${cell('bump', '', v, T['v'+v]?.bump_mm, ' mm')}</td>`).join('')+'</tr></tbody></table>';
    const E = T.estop, ramp = T.ramp, drv = E?.drive_brake || [];
    const dmax = Math.max(...drv.map(d => d.mean_decel_m_s2 || 0)), idl = E?.ideal?.v1, tips = Object.values(E?.ideal || {}).map(x => x.no_tip);
    const tipTxt = tips.length && tips.every(t => !t.firstFail) ? '≥ '+Math.min(...tips.map(t => t.limit))+' m/s² 까지 없음' : tips.length ? Math.min(...tips.map(t => t.limit))+' m/s²' : '—';
    h += '<table class="sim-table"><tbody>';
    h += `<tr><td>경사 위 출발 · 최대 토크</td><td>${cell('slope', 'ramp', 0.5, ramp?.peak_torque, '°')}</td></tr>`;
    h += `<tr><td>경사 위 출발 · 정격 4 Nm 연속</td><td>${ramp?.rated_torque?.limit ?? '—'}°</td></tr>`;
    h += `<tr><td>E-stop 구동 제동 최대 감속 (1 m/s)</td><td><button data-run="estop|drive|1|20">${dmax.toFixed(1)} m/s²</button></td></tr>`;
    h += `<tr><td>뒤 캐스터 들림 없는 최대 감속</td><td>${cell('estop', 'ideal', 1, idl?.no_lift, ' m/s²')}</td></tr>`;
    h += `<tr><td>전도 (0.5~2 m/s, 이상 제동)</td><td>${tipTxt}</td></tr>`;
    h += '</tbody></table>';
    const fails = []; for (const v of sp) for (const [k, l, u] of [['slope_deg', '경사', '°'], ['step_mm', '단차', ' mm'], ['bump_mm', '방지턱', ' mm']]) { const f = T['v'+v]?.[k]?.firstFail; if (f) fails.push(`${l} ${v} m/s · ${f.value}${u}: ${f.text}`); }
    if (ramp?.peak_torque?.firstFail) fails.push(`경사 위 출발 · ${ramp.peak_torque.firstFail.value}°: ${ramp.peak_torque.firstFail.text}`);
    const big = []; for (const v of sp) for (const [k, l] of [['slope_deg', '경사'], ['step_mm', '단차'], ['bump_mm', '방지턱']]) { const L = T['v'+v]?.[k], pr = L?.rows?.find(r => r.value === L.limit)?.max_pitch_rel_deg; if (pr > 15) big.push(`${l} ${v} m/s`); }
    if (big.length) h += `<p style="margin-top:6px">* 한계값에서 차체 피치가 15°를 넘습니다(${big.join(', ')}). 바퀴가 잠깐 뜨며 넘어가는 경우라 실제 운용 한계로 보기 어렵습니다.</p>`;
    h += `<details style="margin-top:8px"><summary>실패 원인 (한계 바로 다음 값)</summary><p>${fails.map(esc).join('<br>')}</p></details>`;
    h += `<p style="margin-top:6px">표의 값을 누르면 그 조건으로 재생합니다. 측면 2D 모델(좌우 동일), 질량 ${P?.source.mass_kg ?? 55} kg·무게중심 높이 ${P?.source.cg_height_mm ?? 250} mm·바닥 μ ${P?.mu ?? 0.6}·관성·타이어 강성은 가정이며 캐스터 선회 오프셋은 무시합니다. 수식·파라미터는 <b>ⓘ 계산 내용</b>에 있습니다. 시제품 시험으로 확인해야 합니다.</p>`;
    $('simLimits').innerHTML = h;
    $('simLimits').querySelectorAll('[data-run]').forEach(b => b.addEventListener('click', () => { const [k, vr, sp_, val] = b.dataset.run.split('|'); sim.kind = k; sim.variant = vr || null; $('simSpeed').value = sp_; ui(); $('simParam').value = val; ui(); run(); }));
  })();
  ui();
  return {sim, run, tick, capture, close, active: () => !!sim.res, applyFrame,
          seek: t => { const F = sim.res.frames; const i = Math.min(F.length-1, Math.round(t*60)); sim.t = F[i][0]; sim.playing = false; applyFrame(F[i]); return F[i]; }};
}

// ---- "계산 내용" dialog ------------------------------------------------------------------------------------
const CAT = {design: ['설계값', 'b-design'], catalog: ['카탈로그', 'b-cat'], derived: ['계산', 'b-der'], assumed: ['가정', 'b-asm']};
const badge = k => `<span class="badge ${CAT[k][1]}">${CAT[k][0]}</span>`;

function schematic(P) {
  // nominal side view in CAD mm (Y forward -> right, Z up), from the parameter set
  const C = [P.ref[0]*1000, P.ref[1]*1000];
  const cg = [C];
  for (const s of P.subs) cg.push(s.kind === 'slide' ? [C[0]+s.au*1000, C[1]+s.aw*1000] : [C[0]+(s.pc[0]-s.pb[0])*1000, C[1]+(s.pc[1]-s.pb[1])*1000]);
  const at = (b, u, w) => [cg[b][0]+u*1000, cg[b][1]+w*1000];
  const X = y => y, Y = z => -z;
  const col = ['#8fb3d9', '#6aa8ff', '#ffcc73'];
  let g = `<line x1="-420" y1="0" x2="420" y2="0" stroke="#5f7894" stroke-width="2"/>`;
  for (const set of P.points) {
    const pts = set.pts.map(([u, w]) => at(set.b, u, w)).sort((a, b) => a[0]-b[0]);
    g += pts.map(([y, z]) => `<circle cx="${X(y).toFixed(1)}" cy="${Y(z).toFixed(1)}" r="2.6" fill="${col[set.b] || '#ccc'}" opacity=".8"/>`).join('');
  }
  const [wy, wz] = at(P.drive.b, P.drive.u, P.drive.w);
  g += `<circle cx="${X(wy)}" cy="${Y(wz)}" r="${P.R*1000}" fill="rgba(106,168,255,.12)" stroke="#6aa8ff" stroke-width="2.5"/><circle cx="${X(wy)}" cy="${Y(wz)}" r="4" fill="#6aa8ff"/>`;
  g += `<text x="${X(wy)}" y="${Y(wz)-P.R*1000-8}" fill="#9fc4ff" font-size="15" text-anchor="middle">구동륜 Ø${Math.round(P.R*2000)}</text>`;
  for (const c of P.casters) {
    const [y, z] = at(c.b, c.u, c.w);
    g += `<circle cx="${X(y)}" cy="${Y(z)}" r="${P.rc*1000}" fill="rgba(79,224,201,.12)" stroke="${c.tag === 'front' ? '#4fe0c9' : '#ffcc73'}" stroke-width="2.5"/>`;
    g += `<text x="${X(y)}" y="${Y(z)+P.rc*1000+20}" fill="#b4c2d9" font-size="14" text-anchor="middle">${c.tag === 'front' ? '앞' : '뒤'} 캐스터</text>`;
  }
  for (const s of P.subs) if (s.kind === 'pivot') {
    const [y, z] = [C[0]+s.pc[0]*1000, C[1]+s.pc[1]*1000];
    g += `<circle cx="${X(y)}" cy="${Y(z)}" r="7" fill="#0c1424" stroke="#ff8f8f" stroke-width="3"/><text x="${X(y)+10}" y="${Y(z)-12}" fill="#ffb0b0" font-size="14">보기 피벗</text>`;
  }
  const zc = P.zcg*1000;
  g += `<g stroke="#e9eefb" stroke-width="2"><circle cx="0" cy="${Y(zc)}" r="9" fill="none"/><line x1="-9" y1="${Y(zc)}" x2="9" y2="${Y(zc)}"/><line x1="0" y1="${Y(zc)-9}" x2="0" y2="${Y(zc)+9}"/></g>`;
  g += `<text x="14" y="${Y(zc)-10}" fill="#e9eefb" font-size="14">무게중심 Z${Math.round(zc)}</text>`;
  g += `<text x="-410" y="-300" fill="#8394ad" font-size="13">● 차체 하부 실루엣  </text><text x="-410" y="-282" fill="#6aa8ff" font-size="13">● ${P.model === 'pro' ? '보기 링크' : '구동부(슬라이드)'} 실루엣</text>`;
  if (P.model === 'pro') g += `<text x="-410" y="-264" fill="#ffcc73" font-size="13">● 뒤 캐스터 암 실루엣</text>`;
  g += `<text x="410" y="22" fill="#8394ad" font-size="13" text-anchor="end">→ 전진(+Y)</text>`;
  return `<svg class="calc-svg" viewBox="-420 -320 840 350" role="img" aria-label="측면 모델 구성도">${g}</svg>`;
}

function calcHTML(P, report, res, hand) {
  if (!P) return '<p>시뮬레이션 데이터가 없습니다.</p>';
  const sub = n => P.subs.find(s => s.name === n);
  const d = P.doc, f1 = v => (+v).toFixed(1), f0 = v => Math.round(v);
  let h = `<p class="calc-lead"><b>${esc(P.name)}</b> — ${esc(d.structure)}. 외부 물리엔진 없이 <code>web/drive-sim.js</code>에서 직접 계산하는 <b>측면(피치 평면) 2D 다물체 동역학</b>입니다. 좌우는 같다고 보고 한 쌍으로 묶었습니다. 같은 코드가 브라우저 재생과 한계값 표(<code>node tools/drive_sim.mjs</code>)에 쓰입니다.</p>`;
  h += schematic(P);
  h += `<h3>1. 강체와 자유도</h3><table class="calc-t"><thead><tr><th>물체</th><th>질량</th><th>자유도</th><th>근거</th></tr></thead><tbody>`;
  const dof = i => i === 0 ? '전후 x · 상하 z · 피치 θ' : P.subs[i-1].kind === 'slide' ? '전후 x · 상하 z (피치는 차체와 같음, 슬라이드 방향만 자유)' : '전후 x · 상하 z · 피치 θ (핀으로 차체와 연결)';
  d.bodies.forEach(([n, m, src], i) => { h += `<tr><td>${esc(n)}</td><td>${f1(m)} kg</td><td>${dof(i)}</td><td>${esc(src)}</td></tr>`; });
  h += `<tr><td>구동륜 회전</td><td>I<sub>w</sub> ${P.Iw} kg·m²</td><td>회전 ω</td><td>모터 외곽 회전자 + 타이어 추정</td></tr></tbody></table>`;
  h += `<h3>2. 운동방정식과 적분</h3><div class="eq">m·a = ΣF,  I·θ̈ = ΣM  (물체마다)<br>I<sub>w</sub>·ω̇ = T<sub>모터</sub> − R·F<sub>t</sub>  (구동륜)<br>v ← v + (F/m)·Δt,  x ← x + v·Δt  (반암시적 오일러, Δt = ${DT*1000} ms, 초당 ${Math.round(1/DT)} 스텝)</div>
    <p>모든 힘(중력 g = ${G}, 접촉, 마찰, 스프링, 관절, 모터)을 한 스텝에서 모두 더한 뒤 속도와 위치를 갱신합니다. 재생 영상은 계산 결과를 1/60 s마다 저장한 것입니다.</p>`;
  h += `<h3>3. 접촉 (페널티 방식)</h3><div class="eq">N = max(0, k·δ − c·δ̇)   δ: 겹친 깊이, δ̇: 법선 방향 속도</div>
    <p>바퀴·캐스터는 원, 차체 하부·앞뒤 면과 움직이는 하부 부품은 CAD 실루엣(10 mm 간격 단면의 최저점, 20 mm마다 사용)을 반지름 ${P.rp*1000} mm 점으로 넣었습니다. 지형은 꺾은선이며 면과 모서리 모두 접촉합니다(단차 모서리에 걸리는 것까지 계산).</p>
    <table class="calc-t"><thead><tr><th>접촉</th><th>강성 k</th><th>감쇠 c</th><th>마찰</th></tr></thead><tbody>
    <tr><td>구동륜 타이어 (2륜 합)</td><td>${f0(P.kt/1000)} N/mm</td><td>${P.ct} N·s/m</td><td>LuGre μ ${P.mu} ${badge('assumed')}</td></tr>
    <tr><td>캐스터 (2개 합)</td><td>${f0(P.kc/1000)} N/mm</td><td>${P.cc} N·s/m</td><td>구름 저항 ${P.crr}</td></tr>
    <tr><td>차체 실루엣 점</td><td>${f0(P.kp/1000)} N/mm</td><td>${P.cp} N·s/m</td><td>강판-바닥 μ ${P.muBody}</td></tr></tbody></table>`;
  h += `<h3>4. 마찰</h3><div class="eq">구동륜 (LuGre):  v<sub>s</sub> = v<sub>t</sub> − ω·R,  ż = v<sub>s</sub> − σ₀·|v<sub>s</sub>|·z/(μN),  F<sub>t</sub> = −(σ₀·z + σ₁·ż),  |F<sub>t</sub>| ≤ 1.05·μN<br>캐스터·차체:  F<sub>t</sub> = −μ<sub>r</sub>·N·v/√(v² + ε²)   (ε = ${P.eps} m/s)</div>
    <p>LuGre 모델은 타이어 접지면을 작은 강모(σ₀ = ${P.sig0.toExponential(0)} N/m, σ₁ = ${P.sig1} N·s/m)로 보고 변형 z로 정지 마찰을 표현합니다. 그래서 경사에서 서 있을 때 미끄러져 내려가지 않고, 마찰 한계(μN)를 넘으면 미끄러집니다. <b>이상 제동</b>(E-stop 전도 한계)에서는 구동륜을 잠그고 모든 접촉의 마찰계수를 a/g로 둡니다.</p>`;
  h += `<h3>5. 서스펜션</h3>`;
  for (const s of P.subs) {
    if (s.kind === 'slide') {
      const who = s.name === 'drive' ? '구동부 수직 슬라이드' : '뒤 캐스터 경첩 암 (90 mm 암, ±' + s.smax*1000 + ' mm → 수직 슬라이드로 근사)';
      h += `<p><b>${who}</b></p><div class="eq">F<sub>스프링</sub> = −max(0, F₀ + k·s) − c·ṡ   (스프링은 밀기만 함, s: 위로 +, F₀ = ${f1(s.F0)} N, k = ${f1(s.ks/1000)} N/mm, c = ${s.cs} N·s/m, 좌우 합)<br>|s| > ${s.smax*1000} mm → 스토퍼 k<sub>st</sub> = ${f0(s.kst/1000)} N/mm 추가,  슬라이드 직각 방향은 k<sub>j</sub> = ${f0(P.kj/1000)} N/mm로 구속</div>`;
    } else {
      h += `<p><b>보기 링크 (핀 조인트)</b> — 피벗을 k<sub>j</sub> = ${f0(P.kj/1000)} N/mm 페널티로 차체에 묶고, 링크는 피벗 둘레로 회전합니다(φ: 앞 캐스터가 올라가는 방향 +).</p>
        <div class="eq">τ = a·max(0, F₀ − k·a·φ) − c<sub>r</sub>·φ̇   (a = 피벗 뒤 ${f0(s.arm*1000)} mm, F₀ = ${f1(s.F0)} N, k = ${f1(s.k/1000)} N/mm, 좌우 합)<br>|φ| > ${(s.amax*180/Math.PI).toFixed(2)}° (앞 캐스터 ±${f0(s.frontArm*1000*Math.sin(s.amax))} mm) → 우레탄 스토퍼 (앞 캐스터 올라감 ${f0(s.ksrUp)} / 내려감 ${f0(s.ksrDown)} N·m/rad, C-USLH10 1개 3×10⁵ N/m 가정) 추가</div>
        <p>스프링은 링크의 구동륜 쪽을 눌러 구동륜 하중을 늘립니다. 링크 무게(모터 포함)는 보기 물체에 들어 있어, 링크가 돌 때 구동륜·앞 캐스터 하중이 바뀌는 것이 그대로 계산됩니다.</p>`;
    }
  }
  const T1 = P.T1;
  h += `<h3>6. 모터와 속도 제어</h3><div class="eq">ω<sub>ref</sub> = v<sub>ref</sub>/R,  T = K<sub>p</sub>·(ω<sub>ref</sub> − ω) + K<sub>i</sub>·∫(ω<sub>ref</sub> − ω)dt   (K<sub>p</sub> ${P.Kp}, K<sub>i</sub> ${P.Ki}, 2륜 합)<br>|T| ≤ 2·min(T<sub>스톨</sub>, T<sub>정격</sub>·(ω₀ − |ω|)/(ω₀ − ω<sub>정격</sub>))   (T<sub>스톨</sub> ${T1.peak} Nm, T<sub>정격</sub> ${T1.rated} Nm @ ${Math.round(T1.wr*30/Math.PI)} rpm, 무부하 ${Math.round(T1.w0*30/Math.PI)} rpm)</div>
    <p>토크가 한계에 닿으면 적분을 멈춥니다(와인드업 방지). 모터 반작용 토크는 모터 고정자가 붙은 물체(${P.model === 'pro' ? '보기 링크' : '차체'})에 걸립니다. "정격 4 Nm 연속" 한계는 T<sub>스톨</sub> 대신 T<sub>정격</sub>을 상한으로 쓴 결과입니다. 모터 발열·전류 제한 시간은 넣지 않았습니다.</p>`;
  h += `<h3>7. 시험 조건</h3><table class="calc-t"><thead><tr><th>시험</th><th>지형</th><th>속도 명령</th></tr></thead><tbody>
    <tr><td>경사 (평지→경사)</td><td>평지 0.9 m → 경사 1.4 m → 평지. 꺾임 모서리를 둥글리지 않음</td><td>0.4 s 정착 후 0.6 m/s² 가속, 경사 중간에서 정지 1 s 후 재출발</td></tr>
    <tr><td>경사 위 출발</td><td>끝없는 경사면 위에서 시작(꺾임 없음)</td><td>출발 → 0.8 m에서 정지 1 s → 재출발, 0.5 m/s</td></tr>
    <tr><td>단차</td><td>0.7 m 앞 직각 단차, 폭 전체</td><td>0.6 m/s² 가속 후 등속</td></tr>
    <tr><td>방지턱</td><td>앞뒤 1:2 경사, 윗면 40 mm 사다리꼴</td><td>0.6 m/s² 가속 후 등속</td></tr>
    <tr><td>E-stop</td><td>평지</td><td>등속 0.6 s 후 감속 명령 a로 0까지 (구동 제동) / 구동륜 잠금 + 전 바퀴 μ = a/g (이상 제동)</td></tr></tbody></table>`;
  h += `<h3>8. 판정 기준</h3><table class="calc-t"><tbody>
    <tr><td>통과</td><td>장애물 뒤 0.75 m(경사는 0.9 m)까지 차체 무게중심이 도달</td></tr>
    <tr><td>전도</td><td>앞뒤 캐스터 사이 지형 현에 대한 차체 피치 &gt; 0.8 rad(46°)</td></tr>
    <tr><td>걸림</td><td>3 s 동안 전진 &lt; 20 mm. 원인을 구동륜 들림 / 서스펜션 최대 신장(구동륜 하중 부족) / 차체 하부 걸림 / 미끄러짐 / 토크 부족으로 나누고, 장애물 모서리를 누르는 바퀴를 표시</td></tr>
    <tr><td>밀림</td><td>시작 위치보다 0.25 m 이상 뒤로 밀림, 또는 경사 정지 1 s 동안 30 mm 이상 밀림</td></tr>
    <tr><td>E-stop</td><td>평균 감속 = v²/(2·정지거리), 순간 감속은 20 ms 저역 필터. 뒤 캐스터가 바닥에서 2 mm 넘게 뜨면 "들림"</td></tr>
    <tr><td>한계값 탐색</td><td>격자(경사 1°, 단차 1 mm, 방지턱 2 mm, 감속 0.5 m/s²)를 0부터 올리며 첫 실패에서 멈춤. 표의 값 = 마지막 통과 값</td></tr></tbody></table>`;
  h += `<h3>9. 파라미터와 출처</h3><table class="calc-t"><thead><tr><th>항목</th><th>값</th><th>출처</th><th></th></tr></thead><tbody>`;
  const common = [
    ['바닥 마찰계수 μ', P.mu, '고무-콘크리트 일반값', 'assumed'],
    ['타이어 반경 강성', f0(P.kt/2000)+' N/mm/륜', 'NBR 타이어 추정', 'assumed'],
    ['구동륜 회전 관성', P.Iw/2+' kg·m²/륜', '외곽 회전자 추정', 'assumed'],
    ['캐스터 구름 저항', P.crr, 'PU 캐스터 일반값', 'assumed'],
    ['모터 토크', `스톨 ${T1.peak} / 정격 ${T1.rated} Nm`, 'ZLLG65 사양서 p.4', 'catalog'],
  ];
  for (const [n, v, src, k] of [...d.params, ...common]) h += `<tr><td>${esc(n)}</td><td>${esc(v)}</td><td>${esc(src)}</td><td>${badge(k)}</td></tr>`;
  h += `</tbody></table>`;
  const sens = report?.sensitivity, T = report?.tests;
  if (sens && T) {
    const sp = [0.3, 0.5, 1], keys = [['slope_deg', '경사', '°'], ['step_mm', '단차', ' mm'], ['bump_mm', '방지턱', ' mm']];
    const val = (o, v, k) => o?.['v'+v]?.[k];
    h += `<h3>민감도: ${esc(sens.parameter)}</h3><p>스토퍼 강성은 가정값입니다. 1/3배와 3배로 바꿔 같은 탐색을 다시 돌린 결과입니다(0.3 / 0.5 / 1.0 m/s). 값이 크게 바뀌는 시험은 바퀴·링크가 스토퍼에 닿은 채로 넘어가는 경우입니다.</p>
      <table class="calc-t"><thead><tr><th>시험</th><th>강성 1/3배</th><th>기준</th><th>3배</th></tr></thead><tbody>`;
    for (const [k, l, u] of keys) h += `<tr><td>${l}</td>${[sens.factors['x0.33'], null, sens.factors.x3].map(o => `<td>${sp.map(v => (o ? val(o, v, k) : T['v'+v]?.[k]?.limit) ?? '—').join(' / ')}${u}</td>`).join('')}</tr>`;
    h += `</tbody></table>`;
  }
  const sl = report?.static_loads_N;
  if (sl) {
    h += `<h3>10. 정지 하중 검산 (평지, 정착 후)</h3><table class="calc-t"><thead><tr><th></th><th>구동륜 쌍</th><th>앞 캐스터 쌍</th><th>뒤 캐스터 쌍</th><th>합</th></tr></thead><tbody>
      <tr><td>시뮬레이션</td><td>${sl.drive_pair} N</td><td>${sl.front_caster_pair} N</td><td>${sl.rear_caster_pair} N</td><td>${f1(sl.drive_pair+sl.front_caster_pair+sl.rear_caster_pair)} N</td></tr>`;
    if (hand) h += `<tr><td>손계산</td><td>${hand.drive ?? '—'} N</td><td>${hand.front ?? '—'} N</td><td>${hand.rear ?? '—'} N</td><td>${f1((hand.drive || 0)+(hand.front || 0)+(hand.rear || 0))} N</td></tr>`;
    h += `</tbody></table><p>전체 무게 ${f1(P.m*G)} N. ${hand?.note ? esc(hand.note) : ''}</p>`;
  }
  if (res) {
    const L = {max_pitch_rel_deg: '최대 상대 피치 (°)', max_slip_m_s: '최대 구동륜 미끄럼 속도 (m/s)', wheel_lift_s: '구동륜 무하중 시간 (s)', body_contact_s: '차체 하부 접촉 시간 (s)', torque_sat_s: '토크 한계 도달 시간 (s)', t_end_s: '종료 시각 (s)',
      hold_rollback_mm: '경사 정지 중 밀림 (mm)', stop_time_s: '정지 시간 (s)', stop_distance_mm: '정지 거리 (mm)', mean_decel_m_s2: '평균 감속 (m/s²)', speed_at_estop_m_s: 'E-stop 시 속도 (m/s)', peak_decel_m_s2: '순간 최대 감속 (m/s²)', rear_lift_mm: '뒤 캐스터 최대 들림 (mm)', rear_unloaded_s: '뒤 캐스터 무하중 시간 (s)', min_rear_load_N: '뒤 캐스터 최소 하중 (N)'};
    h += `<h3>현재 실행 결과</h3><p>${esc(TESTS[res.kind === 'slope' && res.terrain.start ? 'ramp' : res.kind]?.label || res.kind)} · 값 ${res.opt.value} · ${res.opt.speed} m/s → <b class="${res.result.pass ? 'sim-ok' : 'sim-bad'}">${esc(res.result.text)}</b> (${res.steps.toLocaleString()} 스텝)</p><table class="calc-t"><tbody>`;
    for (const [k, v] of Object.entries(res.metrics)) h += `<tr><td>${esc(L[k] || k)}</td><td>${v ?? '—'}</td></tr>`;
    h += `</tbody></table>`;
  }
  h += `<h3>한계와 주의</h3><ul class="calc-ul"><li>측면 2D: 좌우 롤, 한쪽 바퀴만 장애물에 오르는 경우, 캐스터 선회(오프셋 22 mm)는 계산하지 않습니다.</li>
    <li>무게중심 높이·관성·타이어 강성·바닥 마찰은 가정입니다. 무게중심이 올라가면 E-stop 들림 한계가 낮아집니다.</li>
    <li>페널티 접촉이라 방지턱 착지 같은 순간 충격 하중은 접촉 강성 가정에 따라 크게 달라집니다(정성적으로만 보세요).</li>
    <li>모터 발열, 배터리 전압 강하, 드라이버 전류 제한 시간은 넣지 않았습니다.</li>
    <li>설계 비교용 도구입니다. 시제품 주행 시험으로 확인해야 합니다.</li></ul>`;
  return h;
}
