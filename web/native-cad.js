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
  const BOGIE=layout.suspension.concept==='bogie';
  const suspension=BOGIE?part('suspension','Pro 모델 보기 링크 서스펜션 (앞 캐스터+구동륜, 스프링)','좌우 각각 구동륜과 앞 캐스터를 3t 절곡 보기 링크 1개로 묶었습니다(바깥벽=구동축 벽, 뒤 바닥+안쪽벽, 앞 캐스터 패드+안쪽벽, 절곡 4회, 용접 없음). 피벗은 회사 재고 MISUMI BGMW6001DD-30(베어링 2개가 든 T형 단신 더블 홀더) 1개를 하판 밑에 거꾸로 달고, 회사 재고 숄더 볼트(12×30)를 통과시켜 링크 바닥의 3t U 브래킷 두 귀에 E-GLNUTS10 록너트로 조입니다(칼라 2개로 내륜 고정). 베어링 간격이 8mm라 충격 하중 여유가 작습니다(시제품 확인). 하판에는 모터 케이블용 Ø28 구멍(그로밋)이 있습니다. 피벗 축은 좌우(X) 방향, Y=+60mm·Z87입니다. Rev17: 피벗 뒤 85mm의 뒤 회전 스토퍼(C-USLH10)에 MISUMI SWU21-60 스프링(6.27N/mm, 설치 51mm에서 약 56N)을 동축으로 넣어 링크의 구동륜 쪽을 눌러 줍니다. 구동륜 하중이 약 152N→187N(정적 하중의 약 69%, 무게중심 중앙 가정)으로 늘고, 링크 회전에 스프링 강성이 생겨 서스펜션 역할도 합니다. 회전은 바닥의 회사 재고 C-USLH10 스토퍼 2개와 하판의 M6 스트라이커로 앞 캐스터 ±10mm에서 멈춥니다. 뒤 캐스터 2개는 Rev14 경첩+스프링 모듈입니다. Rev17g: 힌지 핀을 회사 재고 E-SHCCG6-75(Ø6 SUS304, E링 고정)로, 부시를 MPFZ6-10으로, 스프링을 SWL18-30(35.1N/mm, 바퀴 환산 5배)으로 바꿔 급제동 때 뒤 캐스터가 덜 뜹니다(시뮬레이션 1.0→1.75m/s²). 핀·부시는 MISUMI STEP 형상 그대로 넣었고, 스프링은 STEP의 사각 선재 단면과 피치를 따릅니다. 가격은 상단 ☰ BOM·예상 가격에서 확인합니다.'):part('suspension','Rev14 6륜 독립 경첩 서스펜션','6개 바퀴마다 절곡 브래킷 1개, 절곡 암 1개, MISUMI 힌지 핀 1개(구동 CLBKHM10-78.5, 캐스터 CLBKHM10-75), MPFZ10-10 부시 2개, 스프링 1개(구동 X-SWU21-70, 캐스터 X-SWU17-35)를 씁니다. Rev14: 절곡 3t 제한에 맞춰 암·브래킷은 3t 절곡, 부시 자리는 8t 평판 보스, 구동축은 3t 토크판으로 보강합니다. Rev12b: 핀·부시·우레탄 스토퍼(C-USLH10)는 MISUMI 제공 STEP 형상 그대로 넣었고, 스프링은 STEP의 사각 선재 단면과 피치를 따릅니다. 범프 스토퍼는 암에 나사로 고정한 C-USLH10 우레탄 끝을 하판에서 내려온 M6 볼트가 받는 구조이고, M8 스터드 위 너트가 처짐 스토퍼입니다. 핀 축은 진행 방향과 나란해 전후진 거동이 같습니다. 레일·용접·행거가 없습니다. 가격은 상단 ☰ BOM·예상 가격에서 확인합니다.');
  const wrapper=new THREE.Group();wrapper.scale.set(1000,1000,-1000);wrapper.add(gltf.scene);robot.add(wrapper);
  const S=layout.suspension,LP=layout.parameters;
  const rolling={},swivels={},springs={},pivots={},holders={},arms={},lengths={},axes={};
  // Viewer axes: X=CAD X, Y=CAD Z, Z=CAD Y. A CAD rotation a about +Y is rotation.z=-a here,
  // and a CAD rotation a about +X (bogie) is rotation.x=-a.
  const pivotGroup=(parent,key,x,y,z)=>{const g=new THREE.Group();g.position.set(x,y,z);parent.add(g);(pivots[key]??=[]).push(g);return g};
  const holderFor={};  // motion key -> group id -> pivot group (one per visibility group)
  const holder=(key,id)=>((holderFor[key]??={})[id]??=pivotGroup(groups[id],key,...pivotAt[key]));
  const pivotAt={};
  for(const [n,id,side] of [['LEFT','motorL',-1],['RIGHT','motorR',1]]){
    if(BOGIE){
      pivotAt[n]=[0,S.bogie.pivot_z_mm,S.bogie.pivot_y_mm];axes[n]='x';lengths[n]=LP.caster_y-S.bogie.pivot_y_mm;
    }else{
      pivotAt[n]=[side*S.drive.pivot_x_mm,S.drive.pivot_z_mm,0];axes[n]='z';lengths[n]=LP.drive_track/2-S.drive.pivot_x_mm;
    }
    holders[n]=holder(n,id);arms[n]=pivotGroup(suspension,n,...pivotAt[n]);
    const [px,pz,py]=pivotAt[n];
    const o=new THREE.Group();o.position.set(side*LP.drive_track/2-px,90-pz,-py);holders[n].add(o);
    rolling[n]=o;wheels.push({group:o,sign:side});
  }
  for(const n of ['LF','LB','RF','RB']){
    const side=n[0]==='L'?-1:1,x=side*LP.caster_x,y=(n[1]==='F'?1:-1)*LP.caster_y;
    const key=BOGIE&&n[1]==='F'?(side<0?'LEFT':'RIGHT'):n;
    if(key===n){
      pivotAt[n]=[x-side*S.caster.arm_length_mm,S.caster.pivot_z_mm,y];axes[n]='z';lengths[n]=S.caster.arm_length_mm;
      arms[n]=pivotGroup(suspension,n,...pivotAt[n]);
    }
    const h=holder(key,'caster'+n);if(key===n)holders[n]=h;
    const [px,pz,py]=pivotAt[key];
    const swivel=new THREE.Group();swivel.position.set(x-px,-pz,y-py);h.add(swivel);
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
    if(wheel)parent=r.group==='suspension'?arms[wheel]:holder(wheel,id);
    if(r.name.startsWith('30_')||r.name.startsWith('31_'))parent=rolling[wheel];
    if(r.name.startsWith('43_')||r.name.startsWith('44_'))parent=swivels[caster];
    parent.updateWorldMatrix(true,false);parent.attach(node);tag(node,id);
    if(r.metadata?.sku)node.traverse(mesh=>{if(mesh.isMesh)mesh.userData.sku=r.metadata.sku});
    if(Number.isFinite(r.metadata?.spring_length_mm)){
      const {wheel:n,spring_top_z:top,spring_length_mm:length,spring_xy:[x,y]}=r.metadata;
      const pivot=new THREE.Group();
      pivot.position.set(x,top,y);
      suspension.add(pivot);pivot.updateWorldMatrix(true,false);pivot.attach(node);
      springs[n]={pivot,length,x,y,seat:top-length};
    }
  }
  robot.remove(wrapper);
  if(missing.length)throw new Error('CAD nodes missing: '+missing.join(', '));
  if(lidarVisual)addLidarVisuals(lidarVisual,layout,groups,parts,tag);
    for(const n of ['LF','LB','RF','RB'])parts['caster'+n].description='공식 GL-65-ASF-EUS STEP 기준(장착높이 83mm, 판 48×65mm, 홀 24×54mm). '+(BOGIE&&n[1]==='F'?'보기 링크 앞쪽 패드에 CB6-12 4개로 고정되어 구동륜과 함께 피벗(Y=+60) 둘레를 돕니다(±10mm).':'90mm 경첩 암에 달려 ±5mm 움직이고, 바로 위 SWL18-30 스프링(Rev17g)이 받칩니다.');
  for(const id of ['cameraLeft','cameraRight'])parts[id].description='Orbbec 공식 Astra Mini Pro 3D V0.1의 58솔리드. 뒷면 M2 2개(2D V1.2 피치80)로 프린팅 행거 뒤판에 체결합니다. 3D 모델에는 구멍이 없어 위치는 2D 피치 기준(뒷면 중앙 높이) 가정입니다. 3D 깊이24.264, 실물 버전 확인 필요.';
  parts.profiles.description='주기둥 4개와 카메라 행거 2개는 알루미늄 프로파일입니다. 각 기둥 양끝을 CB5-12 볼트로 판을 관통해 프로파일 중심 탭(M5)에 체결합니다. Rev17f: LiDAR·카메라 위치를 Eco 모델과 같게 했습니다(하판·상판 440×600). LiDAR는 하판 위 9mm 프린팅 받침에 고정합니다(스캐너는 아래에서 M3 2개, 받침은 안쪽 혀 부분에서 위→아래 M5 3개로 하판에만 체결). 카메라는 상판 밑 3D 프린팅 판 1개(플랜지 + 세운 판 + 리브 2개 + 뒤판)에 뒷면 M2 2개(피치 80)로 매답니다. 공칭 20×20 단면과 슬롯은 제조사 미지정 범용 형상입니다.';
  const poses=BOGIE?{LEFT:0,RIGHT:0,LB:0,RB:0}:{LEFT:0,RIGHT:0,LF:0,LB:0,RF:0,RB:0};
  function setPose(wheel,mm){
    if(!(wheel in poses)||!Number.isFinite(mm))throw new Error('Invalid wheel pose');
    const drive=['LEFT','RIGHT'].includes(wheel);
    const half=(drive?(BOGIE?S.bogie:S.drive):S.caster).travel_mm/2;
    if(Math.abs(mm)>half+1e-9)throw new Error('Stroke exceeded');
    const side=wheel[0]==='L'?-1:1;
    if(axes[wheel]==='x'){
      // Bogie: +mm raises the front caster, rotation about CAD +X through the pivot.
      const a=Math.asin(mm/lengths[wheel]);
      for(const g of pivots[wheel])g.rotation.x=-a;
      const bs=springs[wheel];
      if(bs){
        // Rev17 bogie spring: its seat on the rear floor turns with the bogie; the top stays on the base plate.
        const p=pivots[wheel][0].position,ry=bs.y-p.z,rz=bs.seat-p.y;
        const dz=ry*Math.sin(a)+rz*Math.cos(a)-rz;
        bs.pivot.scale.y=(bs.length-dz)/bs.length;
      }
      poses[wheel]=mm;return;
    }
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
