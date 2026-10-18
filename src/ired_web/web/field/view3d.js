/* 3D view of the field (three.js), loaded only when the user picks 3D.
 * Reads the same `model` object as the top view; ROS coordinates throughout
 * (the `world` group turns ROS z-up into three.js y-up). */
import * as THREE from 'three';
import { OrbitControls } from '../vendor/three/OrbitControls.js';
import { STLLoader } from '../vendor/three/STLLoader.js';

export function createView3D(host, model, colors) {
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });   // throws without WebGL
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  host.appendChild(renderer.domElement);
  const scene = new THREE.Scene();
  const world = new THREE.Group();
  world.rotation.x = -Math.PI / 2;
  scene.add(world);
  scene.add(new THREE.HemisphereLight(0xffffff, 0x334455, 1.6));
  const sun = new THREE.DirectionalLight(0xffffff, 1.4);
  sun.position.set(3, 8, 4);
  scene.add(sun);

  // floor = the top view's own drawing, so both views always match
  let floorVersion = model.floorVersion;
  const floorTex = new THREE.CanvasTexture(model.floorCanvas);
  floorTex.colorSpace = THREE.SRGBColorSpace;
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(model.size, model.size), new THREE.MeshBasicMaterial({ map: floorTex }));
  floor.position.set(model.size / 2, model.size / 2, 0);
  world.add(floor);

  const wallMat = new THREE.MeshStandardMaterial({ color: 0x8aa0b0, transparent: true, opacity: 0.28 });
  const s = model.size;
  for (const [x, y, w, h] of [[s / 2, -0.02, s + 0.08, 0.04], [s / 2, s + 0.02, s + 0.08, 0.04], [-0.02, s / 2, 0.04, s + 0.08], [s + 0.02, s / 2, 0.04, s + 0.08]]) {
    const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, 0.4), wallMat);
    m.position.set(x, y, 0.2);
    world.add(m);
  }

  const boxes = new THREE.Group();
  world.add(boxes);
  let boxesKey = '';
  function syncBoxes() {
    const key = JSON.stringify(model.stations);
    if (key === boxesKey) return;
    boxesKey = key;
    boxes.clear();
    for (const st of model.stations) {
      const g = new THREE.Group();
      g.position.set(st.x, st.y, 0);
      g.rotation.z = THREE.MathUtils.degToRad(st.rotation);
      const body = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.2, 0.3), new THREE.MeshStandardMaterial({ color: colors.wood, roughness: 0.9 }));
      body.position.z = 0.15;
      g.add(body);
      for (const [sx, col, ry] of [[0.101, 0xffffff, Math.PI / 2], [-0.101, 0x9aa5ad, -Math.PI / 2]]) {
        const tag = new THREE.Mesh(new THREE.PlaneGeometry(0.07, 0.07), new THREE.MeshBasicMaterial({ color: col }));
        tag.position.set(sx, 0, 0.15);
        tag.rotation.y = ry;
        g.add(tag);
      }
      boxes.add(g);
    }
  }

  const robot = new THREE.Group();
  world.add(robot);
  const mesh = new THREE.Group();
  robot.add(mesh);
  new STLLoader().load(model.meshUrl, (geo) => {
    geo.computeVertexNormals();
    const m = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: 0xc9d3da, metalness: 0.2, roughness: 0.6 }));
    m.scale.setScalar(0.001);                        // STL in mm
    m.position.set(-0.094, 0.005, 0.11 - 0.0794);    // same origin as the URDF visual
    m.rotation.z = Math.PI;
    mesh.add(m);
  });

  const SCAN_N = 720;
  const scanGeo = new THREE.BufferGeometry();
  scanGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(SCAN_N * 3), 3));
  const scan = new THREE.Points(scanGeo, new THREE.PointsMaterial({ color: colors.amber, size: 0.045 }));
  world.add(scan);

  const pathLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineDashedMaterial({ color: colors.cyan, dashSize: 0.08, gapSize: 0.05 }));
  world.add(pathLine);
  const goal = new THREE.Mesh(new THREE.RingGeometry(0.17, 0.24, 48), new THREE.MeshBasicMaterial({ color: colors.magenta }));
  goal.position.z = 0.006;
  world.add(goal);

  const cam = new THREE.PerspectiveCamera(45, 1, 0.05, 50);
  cam.position.set(s / 2, 4.2, 3.6);
  const orbit = new OrbitControls(cam, renderer.domElement);
  orbit.target.set(s / 2, 0, -s / 2 + 0.1);
  orbit.maxPolarAngle = Math.PI / 2.1;
  orbit.update();

  function resize() {
    renderer.setSize(host.clientWidth, host.clientHeight);
    cam.aspect = host.clientWidth / host.clientHeight;
    cam.updateProjectionMatrix();
  }
  const ro = new ResizeObserver(resize);
  ro.observe(host);
  resize();

  return {
    canvas: renderer.domElement,
    pick(clientX, clientY) {                          // screen -> map point on the floor
      const r = renderer.domElement.getBoundingClientRect();
      const ray = new THREE.Raycaster();
      ray.setFromCamera(new THREE.Vector2(((clientX - r.left) / r.width) * 2 - 1, -((clientY - r.top) / r.height) * 2 + 1), cam);
      const hit = ray.intersectObject(floor)[0];
      return hit ? { x: hit.point.x, y: -hit.point.z } : null;
    },
    draw() {
      if (model.floorVersion !== floorVersion) { floorTex.needsUpdate = true; floorVersion = model.floorVersion; }
      syncBoxes();
      boxes.visible = model.layers.boxes;
      robot.position.set(model.robot.x, model.robot.y, 0);
      robot.rotation.z = model.robot.yaw;
      mesh.visible = model.layers.model;
      const p = scanGeo.attributes.position.array;
      const n = Math.min(SCAN_N, model.scan.length);
      for (let i = 0; i < SCAN_N; i++) {
        const q = i < n ? model.scan[i] : [model.robot.x, model.robot.y];
        p[i * 3] = q[0]; p[i * 3 + 1] = q[1]; p[i * 3 + 2] = 0.36;
      }
      scanGeo.attributes.position.needsUpdate = true;
      scan.visible = model.layers.scan;
      goal.visible = !!model.goal;
      if (model.goal) goal.position.set(model.goal[0], model.goal[1], 0.006);
      const route = model.path.length ? model.path : (model.goal ? [[model.robot.x, model.robot.y], model.goal] : []);
      pathLine.visible = route.length > 1 && model.layers.path;
      if (pathLine.visible) {
        pathLine.geometry.setFromPoints(route.map(([x, y]) => new THREE.Vector3(x, y, 0.01)));
        pathLine.computeLineDistances();
      }
      robot.visible = model.robotKnown;
      renderer.render(scene, cam);
    },
    dispose() {
      ro.disconnect();
      orbit.dispose();
      renderer.dispose();
      renderer.domElement.remove();
    },
  };
}
