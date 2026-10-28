// three.js view of a MuJoCo model: visual mesh geoms become meshes built straight
// from the model's own vertex/face buffers, updated from geom_xpos / geom_xmat.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { mjName } from "./g1sim.mjs";

export class RobotView {
  constructor(canvas, colors) {
    this.canvas = canvas;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: "low-power" });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(colors.sky);
    this.camera = new THREE.PerspectiveCamera(32, 1, 0.05, 50);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(2.4, -2.0, 1.25);
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.target.set(0.1, 0.0, 0.8);
    this.controls.enablePan = false;
    this.controls.minDistance = 1.2; this.controls.maxDistance = 6;
    this.controls.maxPolarAngle = Math.PI * 0.52;
    this.controls.update();

    const hemi = new THREE.HemisphereLight(0xffffff, 0x777777, 0.9);
    this.scene.add(hemi);
    const sun = new THREE.DirectionalLight(0xffffff, 1.6);
    sun.position.set(1.0, -1.5, 3.0);
    sun.castShadow = true;
    sun.shadow.mapSize.set(1024, 1024);
    sun.shadow.camera.near = 0.5; sun.shadow.camera.far = 8;
    sun.shadow.camera.left = -2; sun.shadow.camera.right = 2; sun.shadow.camera.top = 2; sun.shadow.camera.bottom = -2;
    sun.shadow.bias = -0.0005;
    this.scene.add(sun);

    const floor = new THREE.Mesh(new THREE.PlaneGeometry(12, 12), new THREE.MeshStandardMaterial({ color: colors.floor, roughness: 1 }));
    floor.receiveShadow = true;
    this.scene.add(floor);
    const grid = new THREE.GridHelper(12, 24, colors.grid, colors.grid);
    grid.rotation.x = Math.PI / 2; grid.position.z = 0.001;
    grid.material.transparent = true; grid.material.opacity = 0.5;
    this.scene.add(grid);

    // Double-sided: the STL files have inconsistent winding in places, so
    // single-sided drawing leaves holes. Normals come from creaseGeometry.
    this.robotMat = new THREE.MeshStandardMaterial({ color: colors.robot, roughness: 0.55, metalness: 0.05, side: THREE.DoubleSide });
    this.geoms = [];
    this.resize();
    window.addEventListener("resize", () => this.resize());
    // the canvas fills its stage, whose height follows the control panel, so watch the element itself
    if (typeof ResizeObserver !== "undefined") new ResizeObserver(() => this.resize()).observe(canvas);
  }

  resize() {
    const w = this.canvas.clientWidth, h = this.canvas.clientHeight;
    if (w === 0 || h === 0) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h; this.camera.updateProjectionMatrix();
  }

  /** Build meshes for every visual (group 2) mesh geom of the model. With a mesh
   *  pack, the model only holds placeholder meshes (identity mesh frame) and the
   *  real vertices come from the pack, keyed by mesh name. */
  buildRobot(model, pack = null) {
    const MESH = 7;
    for (let g = 0; g < model.ngeom; g++) {
      if (model.geom_type[g] !== MESH || model.geom_group[g] !== 2) continue;
      const m = model.geom_dataid[g];
      let geo;
      if (pack) {
        const name = mjName(model, model.name_meshadr, m).replace(/^.*\//, "");
        const pm = pack[name]; if (!pm) { console.warn("[render] no packed mesh for", name); continue; }
        geo = creaseGeometry(pm.vert, pm.face, 40);
      } else {
        const va = model.mesh_vertadr[m], vn = model.mesh_vertnum[m];
        const fa = model.mesh_faceadr[m], fn = model.mesh_facenum[m];
        geo = creaseGeometry(model.mesh_vert.subarray(3 * va, 3 * (va + vn)), model.mesh_face.subarray(3 * fa, 3 * (fa + fn)), 40);
      }
      const mesh = new THREE.Mesh(geo, this.robotMat);
      mesh.castShadow = true; mesh.receiveShadow = false;
      mesh.matrixAutoUpdate = false;
      this.scene.add(mesh);
      this.geoms.push({ id: g, mesh });
    }
  }

  /** Pose every mesh from the data's geom frames. */
  update(data) {
    const M = new THREE.Matrix4();
    for (const { id, mesh } of this.geoms) {
      const p = 3 * id, r = 9 * id, X = data.geom_xpos, R = data.geom_xmat;
      M.set(R[r], R[r + 1], R[r + 2], X[p],
            R[r + 3], R[r + 4], R[r + 5], X[p + 1],
            R[r + 6], R[r + 7], R[r + 8], X[p + 2],
            0, 0, 0, 1);
      mesh.matrix.copy(M); mesh.matrixWorldNeedsUpdate = true;
    }
  }

  /** Point the camera at a robot standing at (x, y) and facing `yaw` (rad, 0 = +x):
   *  a half-side front view, so the robot faces the viewer whichever way it stands. */
  frame(x, y, yaw = 0) {
    const c = Math.cos(yaw), s = Math.sin(yaw);
    const rot = (fx, fy) => [x + c * fx - s * fy, y + s * fx + c * fy];
    const [tx, ty] = rot(0.1, 0.0);
    const [cx, cy] = rot(2.4, -2.0);
    this.controls.target.set(tx, ty, 0.72);
    this.camera.position.set(cx, cy, 1.2);
    this.controls.update();
  }
  render() { this.controls.update(); this.renderer.render(this.scene, this.camera); }
}

/** Thick arrow (cylinder + cone) anchored at its tail, like the paper figures. */
export class ForceArrow {
  constructor(scene, color, width = 0.026) {
    this.width = width;
    const mat = new THREE.MeshStandardMaterial({ color, roughness: 0.6 });
    this.group = new THREE.Group();
    this.shaft = new THREE.Mesh(new THREE.CylinderGeometry(width / 2, width / 2, 1, 16), mat);
    this.head = new THREE.Mesh(new THREE.ConeGeometry(width, 2.2 * width, 20), mat);
    this.group.add(this.shaft, this.head);
    this.group.visible = false;
    scene.add(this.group);
  }
  /** tail: [x,y,z], vec: force vector in N, scale: metres per N. */
  set(tail, vec, scale) {
    const len = Math.hypot(vec[0], vec[1], vec[2]) * scale;
    if (len < 0.015) { this.group.visible = false; return; }
    this.group.visible = true;
    const headLen = 2.2 * this.width, shaftLen = Math.max(len - headLen, 0.001);
    this.shaft.scale.set(1, shaftLen, 1); this.shaft.position.set(0, shaftLen / 2, 0);
    this.head.position.set(0, shaftLen + headLen / 2, 0);
    this.group.position.set(tail[0], tail[1], tail[2]);
    const dir = new THREE.Vector3(vec[0], vec[1], vec[2]).normalize();
    this.group.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
  }
}

/**
 * Non-indexed geometry with per-corner normals smoothed across faces that meet
 * at less than creaseDeg, flat across sharper edges (what MuJoCo and most
 * CAD viewers do). vert: flat xyz, face: flat vertex indices into vert.
 */
function creaseGeometry(vert, face, creaseDeg) {
  const nf = face.length / 3, cosC = Math.cos((creaseDeg * Math.PI) / 180);
  const fnorm = new Float32Array(3 * nf);
  const adj = new Map(); // vertex -> faces using it
  for (let f = 0; f < nf; f++) {
    const a = face[3 * f], b = face[3 * f + 1], c = face[3 * f + 2];
    const ax = vert[3 * a], ay = vert[3 * a + 1], az = vert[3 * a + 2];
    const ux = vert[3 * b] - ax, uy = vert[3 * b + 1] - ay, uz = vert[3 * b + 2] - az;
    const vx = vert[3 * c] - ax, vy = vert[3 * c + 1] - ay, vz = vert[3 * c + 2] - az;
    let nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const l = Math.hypot(nx, ny, nz) || 1; nx /= l; ny /= l; nz /= l;
    fnorm[3 * f] = nx; fnorm[3 * f + 1] = ny; fnorm[3 * f + 2] = nz;
    for (const v of [a, b, c]) { let L = adj.get(v); if (!L) { L = []; adj.set(v, L); } L.push(f); }
  }
  const pos = new Float32Array(9 * nf), nrm = new Float32Array(9 * nf);
  for (let f = 0; f < nf; f++) {
    const fx = fnorm[3 * f], fy = fnorm[3 * f + 1], fz = fnorm[3 * f + 2];
    for (let k = 0; k < 3; k++) {
      const v = face[3 * f + k], o = 9 * f + 3 * k;
      pos[o] = vert[3 * v]; pos[o + 1] = vert[3 * v + 1]; pos[o + 2] = vert[3 * v + 2];
      let sx = 0, sy = 0, sz = 0;
      for (const g of adj.get(v)) {
        const gx = fnorm[3 * g], gy = fnorm[3 * g + 1], gz = fnorm[3 * g + 2];
        const d = fx * gx + fy * gy + fz * gz;
        if (d >= cosC) { sx += gx; sy += gy; sz += gz; }
        else if (-d >= cosC) { sx -= gx; sy -= gy; sz -= gz; } // flipped neighbour (inconsistent winding)
      }
      const l = Math.hypot(sx, sy, sz) || 1;
      nrm[o] = sx / l; nrm[o + 1] = sy / l; nrm[o + 2] = sz / l;
    }
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  geo.setAttribute("normal", new THREE.BufferAttribute(nrm, 3));
  return geo;
}

export { THREE };
