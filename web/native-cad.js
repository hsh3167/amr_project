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
  const suspension=part('suspension','Rev10 구동 스윙암 + 후방 롤 빔','구동륜은 진행 방향과 평행한 Ø10 핀을 축으로 도는 6t 절곡 스윙암과 MISUMI X-SWU17-70 스프링을 사용합니다. 전방 캐스터 2개는 고정, 후방 캐스터 2개는 하나의 롤 빔(±1° 스토퍼)에 달려 3점 지지를 만듭니다. 용접 없이 레이저 절단·절곡품과 카탈로그 핀·베어링으로 구성합니다.');
  const wrapper=new THREE.Group();wrapper.scale.set(1000,1000,-1000);wrapper.add(gltf.scene);robot.add(wrapper);
  const rolling={},swivels={},holders={},springs={},pivots={LEFT:[],RIGHT:[],REAR:[]};
  const S=layout.suspension,drive=S.drive,rear=S.rear_rocker,LP=layout.parameters;
  // Viewer axes: X=CAD X, Y=CAD Z, Z=CAD Y. A CAD rotation a about +Y is rotation.z=-a here.
  const pivotGroup=(parent,x,y,z,key)=>{const g=new THREE.Group();g.position.set(x,y,z);parent.add(g);pivots[key].push(g);return g};
  const armMotor={},armSusp={};
  for(const [n,id,side] of [['LEFT','motorL',-1],['RIGHT','motorR',1]]){
    armMotor[n]=pivotGroup(groups[id],side*drive.pivot_x_mm,drive.pivot_z_mm,0,n);
    armSusp[n]=pivotGroup(suspension,side*drive.pivot_x_mm,drive.pivot_z_mm,0,n);
    const o=new THREE.Group();o.position.set(side*(LP.drive_track/2-drive.pivot_x_mm),90-drive.pivot_z_mm,0);armMotor[n].add(o);
    rolling[n]=o;wheels.push({group:o,sign:side});
  }
  const rearBeam=pivotGroup(suspension,0,rear.pivot_z_mm,-LP.caster_y,'REAR');
  for(const n of ['LF','LB','RF','RB']){
    const x=(n[0]==='L'?-1:1)*LP.caster_x,y=(n[1]==='F'?1:-1)*LP.caster_y;
    const swivel=new THREE.Group();
    if(n[1]==='B'){
      const holder=pivotGroup(groups['caster'+n],0,rear.pivot_z_mm,-LP.caster_y,'REAR');
      swivel.position.set(x,-rear.pivot_z_mm,0);holder.add(swivel);holders[n]=holder;
    }else{swivel.position.set(x,0,y);groups['caster'+n].add(swivel);holders[n]=groups['caster'+n]}
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
    if(wheel==='LEFT'||wheel==='RIGHT')parent=r.group==='suspension'?armSusp[wheel]:armMotor[wheel];
    if(wheel==='REAR')parent=r.group==='suspension'?rearBeam:holders[caster];
    if(r.name.startsWith('30_')||r.name.startsWith('31_'))parent=rolling[wheel];
    if(r.name.startsWith('43_')||r.name.startsWith('44_'))parent=swivels[caster];
    parent.updateWorldMatrix(true,false);parent.attach(node);tag(node,id);
    if(r.metadata?.sku)node.traverse(mesh=>{if(mesh.isMesh)mesh.userData.sku=r.metadata.sku});
    if(Number.isFinite(r.metadata?.spring_length_mm)){
      const {wheel:n,spring_top_z:top,spring_length_mm:length,spring_xy:[x,y]}=r.metadata;
      const pivot=new THREE.Group();
      pivot.position.set(x,top,y);
      suspension.add(pivot);pivot.updateWorldMatrix(true,false);pivot.attach(node);
      springs[n]={pivot,length,x,seatZ:top-length};
    }
  }
  robot.remove(wrapper);
  if(missing.length)throw new Error('CAD nodes missing: '+missing.join(', '));
  if(lidarVisual)addLidarVisuals(lidarVisual,layout,groups,parts,tag);
  for(const n of ['LF','RF'])parts['caster'+n].description='공식 GL-65-ASF-EUS STEP 기준(장착높이 83mm, 판 48×65mm, 홀 24×54mm). 4.5t 절곡 박스 브래킷으로 하판에 고정합니다.';
  for(const n of ['LB','RB'])parts['caster'+n].description='공식 GL-65-ASF-EUS STEP 기준. 후방 캐스터 2개는 하나의 6t 롤 빔에 달려 차체 중심의 Ø10 핀을 축으로 ±1° 기울고, 양 끝 M8 스토퍼 볼트로 제한됩니다.';
  for(const id of ['cameraLeft','cameraRight'])parts[id].description='Orbbec 공식 Astra Mini Pro 3D V0.1의 58솔리드. 2D V1.2는 M2 피치80/깊이19.92, 3D 깊이24.264. 실물 버전·어댑터 확인 필요.';
  parts.profiles.description='주기둥4개와 센서 지지4개는 알루미늄 프로파일입니다. 공칭20×20 단면과 슬롯은 제조사 미지정 범용 형상입니다.';
  const poses={LEFT:0,RIGHT:0,REAR:0};
  const armLength=LP.drive_track/2-drive.pivot_x_mm;
  const angleFor=(key,mm)=>key==='REAR'?Math.asin(mm/LP.caster_x):-(key==='LEFT'?-1:1)*Math.asin(mm/armLength);
  const halfFor=key=>key==='REAR'?LP.caster_x*Math.tan(rear.roll_limit_deg*Math.PI/180):drive.travel_mm/2;
  function setPose(key,mm){
    if(!(key in poses)||!Number.isFinite(mm))throw new Error('Invalid suspension pose');
    if(Math.abs(mm)>halfFor(key)+1e-6)throw new Error('Stroke exceeded');
    const a=angleFor(key,mm);
    for(const g of pivots[key])g.rotation.z=-a;
    const spring=springs[key];
    if(spring){
      // Seat point (x, seatZ) rotated about the arm pivot; the coil top stays fixed.
      const side=key==='LEFT'?-1:1,rx=spring.x-side*drive.pivot_x_mm,rz=spring.seatZ-drive.pivot_z_mm;
      const dz=-rx*Math.sin(a)+rz*Math.cos(a)-rz;
      spring.pivot.scale.y=(spring.length-dz)/spring.length;
    }
    poses[key]=mm;
  }
  return {engine:'build123d',lidarVisual:lidarVisual?'sick_tim_5xx.stl':'drawing_envelope',components:manifest.components.length,solids:manifest.components.reduce((n,r)=>n+r.solids,0),poses,setPose,
    bounds_mm:{min:bounds.min.toArray(),max:bounds.max.toArray(),width:size.x,length:size.z,height:size.y},
    columns:manifest.components.filter(r=>r.status==='generic_profile').map(r=>r.metadata),manifest};
}
