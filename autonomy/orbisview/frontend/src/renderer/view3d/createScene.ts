import * as THREE from 'three';
import { Line2 } from 'three/examples/jsm/lines/Line2.js';
import { LineGeometry } from 'three/examples/jsm/lines/LineGeometry.js';
import { LineMaterial } from 'three/examples/jsm/lines/LineMaterial.js';

export interface View3DContext {
  renderer: THREE.WebGLRenderer;
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  grid: THREE.GridHelper;
  robot: THREE.Mesh;
  goal: THREE.Mesh;
  goalLine: THREE.Line;
  path: Line2;
  pathMaterial: LineMaterial;
  waypoints: THREE.Group;
  waypointRoute: THREE.Line;
  toolOverlay: THREE.Group;
  cloud: THREE.Points;
  footprint: THREE.LineLoop;
  mapPlane: THREE.Mesh;
  costmapPlane: THREE.Mesh;
  laser: THREE.Points;
  setSize: (w: number, h: number) => void;
  dispose: () => void;
}

function makePlane(yLift: number): THREE.Mesh {
  const geom = new THREE.PlaneGeometry(1, 1);
  const mat = new THREE.MeshBasicMaterial({
    transparent: true,
    opacity: 0.55,
    depthWrite: false,
    side: THREE.DoubleSide,
  });
  const mesh = new THREE.Mesh(geom, mat);
  mesh.rotation.x = -Math.PI / 2;
  mesh.position.y = yLift;
  mesh.visible = false;
  return mesh;
}

export function createView3DScene(mount: HTMLElement): View3DContext {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0f1419);

  const camera = new THREE.PerspectiveCamera(55, 1, 0.1, 200);
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  mount.appendChild(renderer.domElement);
  renderer.domElement.style.display = 'block';
  renderer.domElement.style.width = '100%';
  renderer.domElement.style.height = '100%';

  const grid = new THREE.GridHelper(20, 20, 0x334455, 0x1f2a33);
  scene.add(grid);

  const light = new THREE.DirectionalLight(0xffffff, 1);
  light.position.set(5, 10, 7);
  scene.add(light);
  scene.add(new THREE.AmbientLight(0x6688aa, 0.5));

  const robot = new THREE.Mesh(
    new THREE.ConeGeometry(0.2, 0.5, 12),
    new THREE.MeshStandardMaterial({ color: 0x69f0ae }),
  );
  robot.rotation.x = Math.PI / 2;
  scene.add(robot);

  const goal = new THREE.Mesh(
    new THREE.ConeGeometry(0.25, 0.55, 3),
    new THREE.MeshStandardMaterial({ color: 0xff7043 }),
  );
  goal.visible = false;
  scene.add(goal);

  const goalLine = new THREE.Line(
    new THREE.BufferGeometry(),
    new THREE.LineDashedMaterial({ color: 0xff7043, dashSize: 0.2, gapSize: 0.1 }),
  );
  goalLine.visible = false;
  scene.add(goalLine);

  const pathMaterial = new LineMaterial({
    color: 0x4fc3f7,
    linewidth: 2,
    transparent: true,
    opacity: 0.9,
    depthTest: true,
    worldUnits: false,
  });
  const path = new Line2(new LineGeometry(), pathMaterial);
  path.visible = false;
  scene.add(path);

  const waypoints = new THREE.Group();
  scene.add(waypoints);

  const waypointRoute = new THREE.Line(
    new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({ color: 0x4fc3f7, transparent: true, opacity: 0.55 }),
  );
  waypointRoute.visible = false;
  scene.add(waypointRoute);

  const toolOverlay = new THREE.Group();
  scene.add(toolOverlay);

  const cloudGeom = new THREE.BufferGeometry();
  cloudGeom.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
  cloudGeom.setAttribute('color', new THREE.Float32BufferAttribute([], 3));
  const cloud = new THREE.Points(
    cloudGeom,
    new THREE.PointsMaterial({ size: 0.06, vertexColors: true }),
  );
  scene.add(cloud);

  const footprint = new THREE.LineLoop(
    new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({ color: 0x80cbc4 }),
  );
  footprint.visible = false;
  scene.add(footprint);

  const mapPlane = makePlane(0.01);
  const costmapPlane = makePlane(0.02);
  scene.add(mapPlane);
  scene.add(costmapPlane);

  const laserGeom = new THREE.BufferGeometry();
  laserGeom.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
  const laser = new THREE.Points(
    laserGeom,
    new THREE.PointsMaterial({ size: 0.05, color: 0xffcc80 }),
  );
  laser.visible = false;
  scene.add(laser);

  const setSize = (w: number, h: number) => {
    const width = Math.max(1, w);
    const height = Math.max(1, h);
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    pathMaterial.resolution.set(width, height);
  };

  const dispose = () => {
    renderer.dispose();
    if (renderer.domElement.parentElement === mount) {
      mount.removeChild(renderer.domElement);
    }
    scene.traverse((obj) => {
      const mesh = obj as THREE.Mesh | THREE.Points | THREE.Line;
      if (mesh.geometry) mesh.geometry.dispose();
      const mat = (mesh as THREE.Mesh).material;
      if (!mat) return;
      if (Array.isArray(mat)) mat.forEach((m) => m.dispose());
      else mat.dispose();
    });
  };

  setSize(mount.clientWidth || 720, Math.max(mount.clientHeight || 0, 240));

  return {
    renderer,
    scene,
    camera,
    grid,
    robot,
    goal,
    goalLine,
    path,
    pathMaterial,
    waypoints,
    waypointRoute,
    toolOverlay,
    cloud,
    footprint,
    mapPlane,
    costmapPlane,
    laser,
    setSize,
    dispose,
  };
}
