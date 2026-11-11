import * as THREE from 'three';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';

// Display meshes exported from the same build123d BReps as the STEP.
export async function loadNativeCAD({robot,groups,parts,wheels,casters,part,tag,layout}){
  const [gltf,manifest]=await Promise.all([
    new GLTFLoader().loadAsync('cad-output/amr_current.glb'),
    fetch('cad-output/manifest.json').then(r=>{if(!r.ok)throw new Error('CAD manifest missing');return r.json()})
  ]);
  const suspension=part('suspension','6륜 독립 서스펜션','구동륜 ±10mm / 캐스터 ±5mm의 개념 구조. 스프링 강성·예압·감쇠와 체결 강도는 미확정입니다.');
  const wrapper=new THREE.Group();wrapper.scale.set(1000,1000,-1000);wrapper.add(gltf.scene);robot.add(wrapper);
  for(const [id,g] of Object.entries(groups))if(id!=='arm')g.clear();
  wheels.length=0;casters.length=0;
  const rolling={},swivels={},moves={},springs={};
  for(const [n,id,side] of [['LEFT','motorL',-1],['RIGHT','motorR',1]]){
    const o=new THREE.Group();o.position.set(side*layout.parameters.drive_track/2,90,0);groups[id].add(o);
    rolling[n]=o;wheels.push({group:o,sign:side});
  }
  for(const n of ['LF','LB','RF','RB']){
    const o=new THREE.Group();o.position.set(n[0]==='L'?-layout.parameters.caster_x:layout.parameters.caster_x,0,n[1]==='F'?layout.parameters.caster_y:-layout.parameters.caster_y);
    groups['caster'+n].add(o);swivels[n]=o;casters.push(o);
  }
  function groupFor(r){
    if(r.group.startsWith('caster_'))return 'caster'+r.group.slice(7).toUpperCase();
    if(r.group==='motor_left')return 'motorL';if(r.group==='motor_right')return 'motorR';
    if(r.group==='chassis'||r.group==='brackets')return 'base';
    if(r.group==='payload_interface')return 'deck';if(r.group==='structure')return 'profiles';
    if(r.group==='electronics')return r.name.startsWith('50_')?'battery':'electronics';
    if(r.group==='lidars')return r.name.endsWith('FRONT_LEFT')?'lidarFrontLeft':'lidarRearRight';
    if(r.group==='cameras')return r.name.endsWith('LEFT')?'cameraLeft':'cameraRight';
    return 'suspension';
  }
  wrapper.updateMatrixWorld(true);
  const bounds=new THREE.Box3().setFromObject(wrapper),size=bounds.getSize(new THREE.Vector3());
  const missing=[];
  for(const r of manifest.components){
    const name=r.name.replace(/[\[\].: /]/g,'_');
    const node=gltf.scene.getObjectByName(r.name)||gltf.scene.getObjectByName(name);
    if(!node){missing.push(r.name);continue}
    const id=groupFor(r),wheel=r.metadata?.moving_wheel;
    let parent=groups[id];
    if(r.name.startsWith('30_')||r.name.startsWith('31_'))parent=rolling[wheel];
    if(r.name.startsWith('43_')||r.name.startsWith('44_'))parent=swivels[wheel];
    parent.updateWorldMatrix(true,false);parent.attach(node);tag(node,id);
    node.traverse(o=>{if(o.isMesh){o.castShadow=true;o.receiveShadow=true}});
    if(wheel&&!r.name.startsWith('30_')&&!r.name.startsWith('31_'))(moves[wheel]??=[]).push({node,y:node.position.y});
    if(r.name.includes('_spring_')){
      const n=r.metadata.wheel,drive=['LEFT','RIGHT'].includes(n),pivot=new THREE.Group();
      pivot.position.set(drive?(n==='LEFT'?-1:1)*(layout.parameters.drive_track/2-53):(n[0]==='L'?-1:1)*layout.parameters.caster_x,drive?166:112,drive?0:(n[1]==='F'?1:-1)*layout.parameters.caster_y);
      suspension.add(pivot);pivot.updateWorldMatrix(true,false);pivot.attach(node);
      springs[n]={pivot,length:drive?57:25};
    }
  }
  robot.remove(wrapper);
  if(missing.length)throw new Error('CAD nodes missing: '+missing.join(', '));
  parts.casterLF.description='공식 GL-65-ASF-EUS STEP 기준. 설치높이83mm(카탈로그82), 판48X×65Y/홀24X×54Y. 실물 EUS/TUN 확인 필요. 4개 캐스터에 각각 독립 스프링 지지.';
  for(const n of ['LB','RF','RB'])parts['caster'+n].description=parts.casterLF.description;
  for(const id of ['cameraLeft','cameraRight'])parts[id].description='Orbbec 공식 Astra Mini Pro 3D V0.1의 58솔리드. 2D V1.2는 M2 피치80/깊이19.92, 3D 깊이24.264. 실물 버전·어댑터 확인 필요.';
  parts.profiles.description='주기둥4개와 센서 지지4개, 서스펜션 지지4개 모두 알루미늄 프로파일입니다. 공칭20×20 단면과 슬롯은 제조사 미지정 범용 형상입니다.';
  const poses={LEFT:0,RIGHT:0,LF:0,LB:0,RF:0,RB:0};
  function setPose(wheel,mm){
    if(!(wheel in poses)||!Number.isFinite(mm))throw new Error('Invalid wheel pose');
    const half=layout.suspension[['LEFT','RIGHT'].includes(wheel)?'drive':'caster'].travel_mm/2;
    if(Math.abs(mm)>half)throw new Error('Stroke exceeded');
    for(const item of moves[wheel]||[]){
      // Rolling wheel nodes are relative to an axle pivot whose Y axis remains vertical.
      item.node.position.y=item.y+mm;
    }
    if(rolling[wheel])rolling[wheel].position.y=90+mm;
    if(springs[wheel])springs[wheel].pivot.scale.y=(springs[wheel].length-mm)/springs[wheel].length;
    poses[wheel]=mm;
  }
  return {engine:'build123d',components:manifest.components.length,solids:manifest.components.reduce((n,r)=>n+r.solids,0),poses,setPose,
    bounds_mm:{min:bounds.min.toArray(),max:bounds.max.toArray(),width:size.x,length:size.z,height:size.y},
    columns:manifest.components.filter(r=>r.status==='generic_profile').map(r=>r.metadata),manifest};
}
