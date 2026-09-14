import * as THREE from 'three';

export interface CameraController {
  yaw: number;
  pitch: number;
  distance: number;
  target: THREE.Vector3;
  setFollow: (v: boolean) => void;
  setTarget: (x: number, y: number, z: number) => void;
  onPointerDown: (e: PointerEvent) => void;
  onPointerMove: (e: PointerEvent) => void;
  onPointerUp: () => void;
  onWheel: (e: WheelEvent) => void;
  zoomBy: (delta: number) => void;
  zoomIn: () => void;
  zoomOut: () => void;
  fit: () => void;
  reset: () => void;
  update: () => void;
}

const DEFAULT_YAW = 0.8;
const DEFAULT_PITCH = 0.6;
const DEFAULT_DISTANCE = 8;

export function createCameraController(camera: THREE.PerspectiveCamera): CameraController {
  let follow = true;
  let dragging = false;
  let panning = false;
  let lastX = 0;
  let lastY = 0;

  const ctrl: CameraController = {
    yaw: DEFAULT_YAW,
    pitch: DEFAULT_PITCH,
    distance: DEFAULT_DISTANCE,
    target: new THREE.Vector3(0, 0, 0),
    setFollow(v) {
      follow = v;
    },
    setTarget(x, y, z) {
      ctrl.target.set(x, y, z);
    },
    onPointerDown(e) {
      dragging = true;
      panning = e.shiftKey || e.button === 1 || e.button === 2;
      lastX = e.clientX;
      lastY = e.clientY;
    },
    onPointerUp() {
      dragging = false;
      panning = false;
    },
    onPointerMove(e) {
      if (!dragging) return;
      const dx = e.clientX - lastX;
      const dy = e.clientY - lastY;
      lastX = e.clientX;
      lastY = e.clientY;
      if (panning) {
        const panScale = ctrl.distance * 0.002;
        const right = new THREE.Vector3();
        const forward = new THREE.Vector3();
        camera.getWorldDirection(forward);
        forward.y = 0;
        forward.normalize();
        right.crossVectors(forward, new THREE.Vector3(0, 1, 0)).normalize();
        ctrl.target.addScaledVector(right, -dx * panScale);
        ctrl.target.addScaledVector(forward, dy * panScale);
      } else {
        ctrl.yaw -= dx * 0.01;
        ctrl.pitch = Math.max(0.1, Math.min(1.4, ctrl.pitch + dy * 0.01));
      }
      ctrl.update();
    },
  onWheel(e) {
      e.preventDefault();
      ctrl.zoomBy(e.deltaY * 0.01);
    },
    zoomBy(delta) {
      ctrl.distance = Math.max(2, Math.min(40, ctrl.distance + delta));
      ctrl.update();
    },
    zoomIn() {
      ctrl.zoomBy(-1.2);
    },
    zoomOut() {
      ctrl.zoomBy(1.2);
    },
    fit() {
      ctrl.distance = DEFAULT_DISTANCE;
      ctrl.update();
    },
    reset() {
      ctrl.yaw = DEFAULT_YAW;
      ctrl.pitch = DEFAULT_PITCH;
      ctrl.distance = DEFAULT_DISTANCE;
      if (!follow) ctrl.target.set(0, 0, 0);
      ctrl.update();
    },
    update() {
      const t = ctrl.target;
      const cp = Math.cos(ctrl.pitch);
      camera.position.set(
        t.x + ctrl.distance * cp * Math.sin(ctrl.yaw),
        t.y + ctrl.distance * Math.sin(ctrl.pitch),
        t.z + ctrl.distance * cp * Math.cos(ctrl.yaw),
      );
      camera.lookAt(t);
    },
  };

  ctrl.update();
  return ctrl;
}
