import * as THREE from 'three';

const rad = THREE.MathUtils.degToRad;

function lineSegments(points, color, opacity = .9) {
  return new THREE.LineSegments(
    new THREE.BufferGeometry().setFromPoints(points),
    new THREE.LineBasicMaterial({ color, transparent: true, opacity, depthWrite: false }),
  );
}

function fill(vertices, indices, color, opacity) {
  const geometry = new THREE.BufferGeometry().setFromPoints(vertices);
  geometry.setIndex(indices);
  return new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({
    color, transparent: true, opacity, side: THREE.DoubleSide, depthWrite: false,
  }));
}

export function lidarSector(pose, apertureDeg, radius) {
  const sector = new THREE.Group();
  const heading = rad(pose.heading_deg);
  const aperture = rad(apertureDeg);
  const origin = new THREE.Vector3(pose.x, pose.scan_z, pose.y);
  const color = pose.heading_deg < 0 ? 0x43e1d3 : 0x8fa9ff;
  const at = (angle, distance) => origin.clone().add(new THREE.Vector3(
    Math.sin(angle) * distance, 0, Math.cos(angle) * distance,
  ));
  const arc = [];
  const indices = [];
  for (let i = 0; i <= 72; i++) {
    arc.push(at(heading - aperture / 2 + aperture * i / 72, radius));
    if (i > 0) indices.push(0, i, i + 1);
  }
  sector.add(fill([origin, ...arc], indices, color, .12));
  const border = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([origin, ...arc, origin]),
    new THREE.LineBasicMaterial({ color, transparent: true, opacity: .85, depthWrite: false }),
  );
  sector.add(border);

  const blindArc = [];
  for (let i = 0; i <= 24; i++) {
    blindArc.push(at(heading + aperture / 2 + (2 * Math.PI - aperture) * i / 24, radius * .44));
  }
  const blind = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints(blindArc),
    new THREE.LineDashedMaterial({ color: 0xffcc73, dashSize: 8, gapSize: 6, transparent: true, opacity: .8, depthWrite: false }),
  );
  blind.computeLineDistances();
  sector.add(blind);
  return sector;
}

export function cameraFrustum(parameters, side, horizontalDeg, verticalDeg, distance) {
  const yaw = rad(side * parameters.camera_yaw_deg);
  const pitch = rad(parameters.camera_pitch_down_deg);
  // Three.js uses X right, Y up and Z forward in this viewer.
  const forward = new THREE.Vector3(Math.sin(yaw) * Math.cos(pitch), -Math.sin(pitch), Math.cos(yaw) * Math.cos(pitch));
  const right = new THREE.Vector3(Math.cos(yaw), 0, -Math.sin(yaw));
  const up = new THREE.Vector3().crossVectors(forward, right);
  const origin = new THREE.Vector3(side * parameters.camera_x, parameters.camera_z, parameters.camera_y);
  const far = origin.clone().addScaledVector(forward, distance);
  const halfWidth = distance * Math.tan(rad(horizontalDeg / 2));
  const halfHeight = distance * Math.tan(rad(verticalDeg / 2));
  const corners = [[-1, -1], [1, -1], [1, 1], [-1, 1]].map(([x, y]) =>
    far.clone().addScaledVector(right, x * halfWidth).addScaledVector(up, y * halfHeight));
  const color = side < 0 ? 0xdaa9ff : 0xf5a8d4;
  const frustum = new THREE.Group();
  const edges = [];
  for (let i = 0; i < 4; i++) edges.push(origin, corners[i], corners[i], corners[(i + 1) % 4]);
  frustum.add(fill([origin, ...corners], [0,1,2, 0,2,3, 0,3,4, 0,4,1], color, .085));
  frustum.add(lineSegments(edges, color));
  return frustum;
}
