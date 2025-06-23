import * as THREE from 'three';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x202020);

// Fixed camera (no orbit, no rotation)
const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 100);
camera.position.set(5, 5, 5);
camera.lookAt(0, 0, 0);

// Renderer
const renderer = new THREE.WebGLRenderer({ antialias: true, canvas: document.getElementById('app') });
renderer.setSize(window.innerWidth, window.innerHeight);

// Lighting
scene.add(new THREE.HemisphereLight(0xffffff, 0x444444, 1.2));

// Gimbal body (simulated IMU)
const imu = new THREE.Mesh(
  new THREE.BoxGeometry(1, 0.4, 0.2),
  new THREE.MeshStandardMaterial({ color: 0x44aaee })
);
scene.add(imu);

// Transform controls (rotate like a joystick)
const transform = new TransformControls(camera, renderer.domElement);
transform.attach(imu);
transform.setMode('rotate');
scene.add(transform);

// Axis + grid
scene.add(new THREE.GridHelper(10, 20));
scene.add(new THREE.AxesHelper(2));

// Show only yaw-pitch-roll handles
transform.showX = true;
transform.showY = true;
transform.showZ = true;

// Add arc normals
const arcRadius = 1.2;
const normalMaterial = new THREE.LineBasicMaterial({ color: 0xffff00 });

function addNormal(direction, color) {
  const start = new THREE.Vector3();
  const end = direction.clone().normalize().multiplyScalar(arcRadius);
  const geometry = new THREE.BufferGeometry().setFromPoints([start, end]);
  const material = new THREE.LineBasicMaterial({ color });
  const line = new THREE.Line(geometry, material);
  imu.add(line);
}

addNormal(new THREE.Vector3(0, 1, 0), 0xff4444); // Pitch (X-axis)
addNormal(new THREE.Vector3(1, 0, 0), 0x44ff44); // Yaw (Y-axis)
addNormal(new THREE.Vector3(0, 0, 1), 0x4444ff); // Roll (Z-axis)

// Angle HUD
const angleDisplay = document.getElementById('angles');
const resetBtn = document.getElementById('reset');

// Euler update on rotation
transform.addEventListener('objectChange', () => {
  const e = imu.rotation;
  const toDeg = THREE.MathUtils.radToDeg;
  angleDisplay.textContent = `Pitch: ${toDeg(e.x).toFixed(1)}° Yaw: ${toDeg(e.y).toFixed(1)}° Roll: ${toDeg(e.z).toFixed(1)}°`;
});

// Reset rotation
resetBtn.addEventListener('click', () => {
  imu.rotation.set(0, 0, 0);
  angleDisplay.textContent = `Pitch: 0° Yaw: 0° Roll: 0°`;
});

// Resize
window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

function animate() {
  requestAnimationFrame(animate);
  renderer.render(scene, camera);
}
animate();
