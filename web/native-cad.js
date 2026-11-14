import * as THREE from 'three';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';

async function loadLidarVisual() {
  try {
    const geometry = await new STLLoader().loadAsync('web/vendor/sick/sick_tim_5xx.stl');
    geometry.computeBoundingBox();
    if (geometry.getAttribute('position').count === 0 ||
        !Number.isFinite(geometry.boundingBox.min.z) || geometry.boundingBox.isEmpty()) {
      throw new Error('Empty SICK TiM5xx mesh');
    }
    return geometry;
  } catch (error) {
    console.warn('SICK TiM5xx visual unavailable; showing CAD envelopes.', error);
    return null;
  }
}

function addLidarVisuals(geometry, layout, groups, parts, tag) {
  const material = new THREE.MeshStandardMaterial({color:0x657988, roughness:.72});
  // The STL uses ROS metres (X forward, Y left, Z up); the viewer uses mm.
  const rosToViewer = new THREE.Matrix4().set(
    0,-1000,0,0, 0,0,1000,0, 1000,0,0,0, 0,0,0,1,
  );
  for (const pose of layout.lidars) {
    const mount = new THREE.Group();
    mount.position.set(pose.x,pose.bbox_bottom_z,pose.y);
    mount.rotation.y = THREE.MathUtils.degToRad(pose.heading_deg);
    const mesh = new THREE.Mesh(geometry, material);
    mesh.name = 'SICK TiM5xx family STL';
    mesh.applyMatrix4(rosToViewer);
    // Seat the lowest point of the family mesh on the existing mount plate.
    mesh.position.y = -geometry.boundingBox.min.z * 1000;
    mount.add(mesh);
    groups[pose.id].add(mount);
    tag(mesh, pose.id);
    parts[pose.id].description += ' 외형은 SICK 공식 TiM5xx 공용 STL입니다. FOV는 설계 스캔면을 사용합니다.';
  }
}

// Display the build123d meshes with the SICK family visual on both sensor mounts.
export async function loadNativeCAD({robot,groups,parts,wheels,casters,part,tag,layout}){
  const version=encodeURIComponent(layout.revision);
  const output=new URL('../cad-output/',import.meta.url);
  const loadFile=async (file)=>{
    const response=await fetch(new URL(`${file}?rev=${version}`,output),{cache:'no-cache'});
    if(!response.ok)throw new Error(`CAD file missing: ${file}`);
    return response;
  };
  const [gltf,manifest,lidarVisual]=await Promise.all([
    loadFile('amr_current.glb').then(r=>r.arrayBuffer()).then(data=>new GLTFLoader().parseAsync(data,output.href)),
    loadFile('manifest.json').then(r=>r.json()),
    loadLidarVisual(),
  ]);
  const suspension=part('suspension','Rev13 6륜 독립 경첩 서스펜션','6개 바퀴마다 절곡 브래킷 1개, 절곡 암 1개, MISUMI 힌지 핀 1개(구동 CLBKHM10-78.5, 캐스터 CLBKHM10-75), MPFZ10-10 부시 2개, 스프링 1개(구동 X-SWU21-70, 캐스터 X-SWU17-35)를 씁니다. Rev12b: 핀·부시·우레탄 스토퍼(C-USLH10)는 MISUMI 제공 STEP 형상 그대로 넣었고, 스프링은 STEP의 사각 선재 단면과 피치를 따릅니다. 범프 스토퍼는 암에 나사로 고정한 C-USLH10 우레탄 끝을 하판에서 내려온 M6 볼트가 받는 구조이고, M8 스터드 위 너트가 처짐 스토퍼입니다. 핀 축은 진행 방향과 나란해 전후진 거동이 같습니다. 레일·용접·행거가 없습니다. 가격은 상단 ☰ BOM·예상 가격에서 확인합니다.');
  const wrapper=new THREE.Group();wrapper.scale.set(1000,1000,-1000);wrapper.add(gltf.scene);robot.add(wrapper);
  const S=layout.suspension,LP=layout.parameters;
  const rolling={},swivels={},springs={},pivots={},holders={},arms={},lengths={};
  // Viewer axes: X=CAD X, Y=CAD Z, Z=CAD Y. A CAD rotation a about +Y is rotation.z=-a here.
  const pivotGroup=(parent,key,x,y,z)=>{const g=new THREE.Group();g.position.set(x,y,z);parent.add(g);(pivots[key]??=[]).push(g);return g};
  for(const [n,id,side] of [['LEFT','motorL',-1],['RIGHT','motorR',1]]){
    const px=side*S.drive.pivot_x_mm,pz=S.drive.pivot_z_mm;
    holders[n]=pivotGroup(groups[id],n,px,pz,0);arms[n]=pivotGroup(suspension,n,px,pz,0);
    lengths[n]=LP.drive_track/2-S.drive.pivot_x_mm;
    const o=new THREE.Group();o.position.set(side*LP.drive_track/2-px,90-pz,0);holders[n].add(o);
    rolling[n]=o;wheels.push({group:o,sign:side});
  }
  for(const n of ['LF','LB','RF','RB']){
    const side=n[0]==='L'?-1:1,x=side*LP.caster_x,y=(n[1]==='F'?1:-1)*LP.caster_y;
    const px=x-side*S.caster.arm_length_mm,pz=S.caster.pivot_z_mm;
    holders[n]=pivotGroup(groups['caster'+n],n,px,pz,y);arms[n]=pivotGroup(suspension,n,px,pz,y);
    lengths[n]=S.caster.arm_length_mm;
    const swivel=new THREE.Group();swivel.position.set(x-px,-pz,0);holders[n].add(swivel);
    swivels[n]=swivel;casters.push(swivel);
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
  const namedNodes=new Map();
  gltf.scene.traverse(node=>{if(node.name&&!namedNodes.has(node.name))namedNodes.set(node.name,node)});
  const missing=[];
  for(const r of manifest.components){
    const name=r.name.replace(/[\[\].: /]/g,'_');
    const node=namedNodes.get(r.name)||namedNodes.get(name);
    if(!node){missing.push(r.name);continue}
    if(lidarVisual&&r.group==='lidars'&&r.status==='drawing_envelope')continue;
    const id=groupFor(r),wheel=r.metadata?.moving_wheel,caster=r.metadata?.caster;
    let parent=groups[id];
    if(wheel)parent=r.group==='suspension'?arms[wheel]:holders[wheel];
    if(r.name.startsWith('30_')||r.name.startsWith('31_'))parent=rolling[wheel];
    if(r.name.startsWith('43_')||r.name.startsWith('44_'))parent=swivels[caster];
    parent.updateWorldMatrix(true,false);parent.attach(node);tag(node,id);
    if(r.metadata?.sku)node.traverse(mesh=>{if(mesh.isMesh)mesh.userData.sku=r.metadata.sku});
    if(Number.isFinite(r.metadata?.spring_length_mm)){
      const {wheel:n,spring_top_z:top,spring_length_mm:length,spring_xy:[x,y]}=r.metadata;
      const pivot=new THREE.Group();
      pivot.position.set(x,top,y);
      suspension.add(pivot);pivot.updateWorldMatrix(true,false);pivot.attach(node);
      springs[n]={pivot,length,x,seat:top-length};
    }
  }
  robot.remove(wrapper);
  if(missing.length)throw new Error('CAD nodes missing: '+missing.join(', '));
  if(lidarVisual)addLidarVisuals(lidarVisual,layout,groups,parts,tag);
    for(const n of ['LF','LB','RF','RB'])parts['caster'+n].description='공식 GL-65-ASF-EUS STEP 기준(장착높이 83mm, 판 48×65mm, 홀 24×54mm). 90mm 경첩 암에 달려 ±5mm 움직이고, 바로 위 X-SWU17-35 스프링이 받칩니다.';
  for(const id of ['cameraLeft','cameraRight'])parts[id].description='Orbbec 공식 Astra Mini Pro 3D V0.1의 58솔리드. 2D V1.2는 M2 피치80/깊이19.92, 3D 깊이24.264. 실물 버전·어댑터 확인 필요.';
  parts.profiles.description='주기둥 4개, LiDAR 받침 2개, 카메라 행거 2개는 알루미늄 프로파일입니다. Rev13: 각 기둥 양끝을 CB5-12 볼트로 판을 관통해 프로파일 중심 탭(M5)에 체결합니다. 공칭 20×20 단면과 슬롯은 제조사 미지정 범용 형상입니다.';
  const poses={LEFT:0,RIGHT:0,LF:0,LB:0,RF:0,RB:0};
  function setPose(wheel,mm){
    if(!(wheel in poses)||!Number.isFinite(mm))throw new Error('Invalid wheel pose');
    const half=S[['LEFT','RIGHT'].includes(wheel)?'drive':'caster'].travel_mm/2;
    if(Math.abs(mm)>half+1e-9)throw new Error('Stroke exceeded');
    const side=wheel[0]==='L'?-1:1;
    const a=-side*Math.asin(mm/lengths[wheel]);
    for(const g of pivots[wheel])g.rotation.z=-a;
    const spring=springs[wheel];
    if(spring){
      // Seat point rotates about the hinge; the spring top stays on the base plate.
      const p=pivots[wheel][0].position,rx=spring.x-p.x,rz=spring.seat-p.y;
      const dz=-rx*Math.sin(a)+rz*Math.cos(a)-rz;
      spring.pivot.scale.y=(spring.length-dz)/spring.length;
    }
    poses[wheel]=mm;
  }
  return {engine:'build123d',lidarVisual:lidarVisual?'sick_tim_5xx.stl':'drawing_envelope',components:manifest.components.length,solids:manifest.components.reduce((n,r)=>n+r.solids,0),poses,setPose,
    bounds_mm:{min:bounds.min.toArray(),max:bounds.max.toArray(),width:size.x,length:size.z,height:size.y},
    columns:manifest.components.filter(r=>r.status==='generic_profile').map(r=>r.metadata),manifest};
}
