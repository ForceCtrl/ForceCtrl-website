// Demo 2: Stage-2 adaptive hold. The robot holds a basket on a rope; drop items
// in and watch the high-level policy raise its force command to match.
import loadMujoco from "../vendor/mujoco/mujoco.js";
import { loadSceneBinary, parseMeshPack, fetchMaybeGz, netStats, buildIndex, buildObs, applyAction, symToWorld, OBS_DIM } from "./g1sim.mjs";
import { buildHlFrame, History, hlActionToForce, HL_FRAME_DIM } from "./hlobs.js";
import { RobotView, ForceArrow, THREE } from "./render.js";
import { Policy } from "./policy.js";

// every asset is addressed relative to this file, so the demos work from any site root
const SIM = new URL("..", import.meta.url).href;
const $ = (id) => document.getElementById(id);
const put = (id, text) => { const el = document.getElementById(id); if (el) el.textContent = text; }; // panel fields are optional
const status = (t) => { $("status").textContent = t; console.log("[demo2]", t); };
const ARROW_SCALE = 0.009;
const G = 9.81;
async function fetchJSON(u) { return (await fetch(u)).json(); }

async function main() {
  const t0 = performance.now();
  const ticker = setInterval(() => status(`downloading… ${(netStats.bytes / 1e6).toFixed(1)} MB, ${netStats.files} files, ${((performance.now() - t0) / 1000).toFixed(0)} s`), 250);
  const [mujoco, llLayout, hl, cfg, mjb, meshes] = await Promise.all([
    fetchMaybeGz(SIM + "vendor/mujoco/mujoco.wasm").then((wasmBinary) => loadMujoco({ wasmBinary })), fetchJSON(SIM + "build/layout/ll_layout.json"), fetchJSON(SIM + "build/hl/hl_layout.json"),
    fetchJSON(SIM + "scene/demo2_config.json"), fetch(SIM + "scene/hl_demo.mjb").then((r) => r.arrayBuffer()).then((b) => new Uint8Array(b)), fetchMaybeGz(SIM + "scene/g1_meshes.bin").then(parseMeshPack),
  ]);
  const { model, data } = loadSceneBinary(mujoco, mjb, "hl_demo.mjb"); // precompiled with placeholder meshes; the real ones are drawn from the pack
  const layout = { ...llLayout, motion_joint_names: hl.motion_joint_names };
  const idx = buildIndex(mujoco, model, layout);
  const objBody = model.body(hl.object_body).id, objJ = model.jnt("hang_box/hang_box_joint").id;
  const objQadr = model.jnt_qposadr[objJ], objVadr = model.jnt_dofadr[objJ];
  const ropeId = model.tendon("tether_hang_box_right_wrist_yaw_link").id;
  const tensionAdr = model.sensor_adr[model.sensor("tether_hang_box_right_wrist_yaw_link_tension").id];
  model.tendon_range[2 * ropeId + 1] = cfg.rope_length; // right rope live, left stays slack
  const wristSite = model.site("tether_hang_box_right_wrist_yaw_link_wrist").id, boxSite = model.site("tether_hang_box_right_wrist_yaw_link_object").id;
  const rwBody = idx.forceBodies[1];

  status("loading policies…");
  const [llPolicy, hlPolicy] = await Promise.all([Policy.load(SIM + "policies/ll", layout, idx), Policy.load(SIM + "policies/hl", layout, idx)]);
  clearInterval(ticker); status(`loaded ${(netStats.bytes / 1e6).toFixed(1)} MB in ${((performance.now() - t0) / 1000).toFixed(1)} s; building the scene…`);
  const ref = hlPolicy.ref; // both policies track the same static hold frame
  const termDims = hl.actor_terms.map(([, d]) => d[0] / hl.actor_history_length);
  const hist = new History(hl.actor_history_length, HL_FRAME_DIM, termDims, true);

  // scene
  const nogl = new URLSearchParams(location.search).has("nogl");
  let view;
  try { view = nogl ? null : new RobotView($("view"), cfg.colors); } catch (e) { status("WebGL unavailable: " + e.message); view = null; }
  if (!view) view = { scene: { add() {} }, buildRobot() {}, frame() {}, update() {}, render() {}, canvas: $("view"), camera: null };
  view.buildRobot(model, meshes);
  { const [qw, qx, qy, qz] = cfg.init_qpos.slice(3, 7); // pelvis heading of the rest pose
    view.frame(ref.pelvisRef[0], ref.pelvisRef[1], Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))); }
  // The physics body is a 20 cm cube hung from the wrist by a 1 cm tether (the
  // cube does not collide with the robot). What you see is a plastic shopping
  // basket drawn around it, long axis across the forearm, with two bail handles
  // that hang over the wrist link 11 cm above the body centre. The arm there is
  // about 6 cm tall and 6.5 cm wide (measured from the meshes in this pose).
  const BK = { top: [0.34, 0.26], bot: [0.29, 0.21], rim: 0.02, bottom: -0.14, wrist: 0.11, armH: 0.031, armW: 0.034 };
  const bag = makeBasket(cfg.colors.basket || "#C9412F"); // store-basket red: clear of both arrow colours
  bag.matrixAutoUpdate = false; view.scene.add(bag);
  const items = []; // emoji sprites riding inside the basket
  function slotTexture(repeatX) {
    // alpha map for a perforated wall: white = plastic, black = hole
    const W = 1024, H = 512, c = document.createElement("canvas"); c.width = W; c.height = H;
    const x = c.getContext("2d"); x.fillStyle = "#fff"; x.fillRect(0, 0, W, H); x.fillStyle = "#000";
    const top = 48, bottom = 36, mid = 22, slotW = 40, barW = 30, pitch = slotW + barW;
    const rows = [[top, (H - top - bottom - mid) / 2], [top + (H - top - bottom - mid) / 2 + mid, (H - top - bottom - mid) / 2]];
    const n = Math.floor((W - barW) / pitch), x0 = (W - (n * pitch - barW)) / 2;
    for (let i = 0; i < n; i++) for (const [y0, h] of rows) { x.beginPath(); x.roundRect(x0 + i * pitch, y0, slotW, h, 10); x.fill(); }
    const t = new THREE.CanvasTexture(c); t.wrapS = THREE.RepeatWrapping; t.repeat.x = repeatX; t.offset.x = (1 - repeatX) / 2; return t;
  }
  function trapezoid(wb, wt, h) { // in its own x–z plane, bottom edge at z=0, facing +y
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.Float32BufferAttribute([-wb/2, 0, 0,  wb/2, 0, 0,  wt/2, 0, h,  -wt/2, 0, h], 3));
    g.setAttribute("uv", new THREE.Float32BufferAttribute([0, 0, 1, 0, 1, 1, 0, 1], 2));
    g.setIndex([0, 1, 2, 0, 2, 3]); g.computeVertexNormals(); return g;
  }
  function makeBasket(color) {
    const g = new THREE.Group();
    const plastic = new THREE.MeshStandardMaterial({ color, roughness: 0.4, metalness: 0.05, side: THREE.DoubleSide });
    const wallMat = (rep) => new THREE.MeshStandardMaterial({ color, roughness: 0.4, metalness: 0.05, side: THREE.DoubleSide, alphaMap: slotTexture(rep), alphaTest: 0.5, transparent: false });
    const h = BK.rim - BK.bottom, [tx, ty] = BK.top, [bx, by] = BK.bot;
    // four perforated walls, tilted outwards to the rim
    const walls = [[bx, tx, by / 2, ty / 2, 0, 1.0], [bx, tx, -by / 2, -ty / 2, Math.PI, 1.0], [by, ty, bx / 2, tx / 2, -Math.PI / 2, 0.7], [by, ty, -bx / 2, -tx / 2, Math.PI / 2, 0.7]];
    for (const [wb, wt, ob, ot, rot, rep] of walls) {
      const tilt = Math.atan2(Math.abs(ot) - Math.abs(ob), h); // lean outwards
      const pivot = new THREE.Group(); pivot.rotation.z = rot;
      const wall = new THREE.Mesh(trapezoid(wb, wt, Math.hypot(h, ot - ob)), wallMat(rep));
      wall.castShadow = true; wall.receiveShadow = true; wall.position.set(0, Math.abs(ob), BK.bottom); wall.rotation.x = -tilt; pivot.add(wall); g.add(pivot);
    }
    // floor
    const floor = new THREE.Mesh(new THREE.BoxGeometry(bx, by, 0.006), plastic); floor.position.z = BK.bottom + 0.003; floor.castShadow = true; g.add(floor);
    // rim: a flat frame around the top, slightly wider than the walls
    const rw = 0.005, rh = 0.010; // rim: 1 cm wide lip, 1 cm tall
    const outer = new THREE.Shape(); outer.moveTo(-tx/2 - rw, -ty/2 - rw); outer.lineTo(tx/2 + rw, -ty/2 - rw); outer.lineTo(tx/2 + rw, ty/2 + rw); outer.lineTo(-tx/2 - rw, ty/2 + rw); outer.closePath();
    const inner = new THREE.Path(); inner.moveTo(-tx/2 + rw, -ty/2 + rw); inner.lineTo(tx/2 - rw, -ty/2 + rw); inner.lineTo(tx/2 - rw, ty/2 - rw); inner.lineTo(-tx/2 + rw, ty/2 - rw); inner.closePath();
    outer.holes.push(inner);
    const rim = new THREE.Mesh(new THREE.ExtrudeGeometry(outer, { depth: rh, bevelEnabled: false }), plastic); rim.position.z = BK.rim - rh; rim.castShadow = true; g.add(rim);
    // two bail handles hanging over the wrist: each loop hugs the arm, then drops to the rim
    const wire = new THREE.MeshStandardMaterial({ color: 0x8c8f99, roughness: 0.35, metalness: 0.8 });
    const gripMat = new THREE.MeshStandardMaterial({ color, roughness: 0.45, metalness: 0.05 });
    const r = 0.0045, sx = BK.armW + 0.012, topZ = BK.wrist + BK.armH + 0.011; // grip centreline rests on top of the arm
    for (const y of [-0.028, 0.028]) {
      const fx = tx / 2 - 0.004; // handle feet on the rim itself
      const pts = [new THREE.Vector3(-fx, y, BK.rim - 0.004), new THREE.Vector3(-sx - 0.004, y, BK.wrist - 0.03), new THREE.Vector3(-sx, y, BK.wrist + 0.01), new THREE.Vector3(-sx * 0.6, y, topZ - 0.003), new THREE.Vector3(0, y, topZ),
                   new THREE.Vector3(sx * 0.6, y, topZ - 0.003), new THREE.Vector3(sx, y, BK.wrist + 0.01), new THREE.Vector3(sx + 0.004, y, BK.wrist - 0.03), new THREE.Vector3(fx, y, BK.rim - 0.004)];
      const loop = new THREE.Mesh(new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts, false, "centripetal", 0.6), 64, r, 10, false), wire); loop.castShadow = true; g.add(loop);
      const grip = new THREE.Mesh(new THREE.CylinderGeometry(0.011, 0.011, 0.075, 14), gripMat); grip.rotation.z = Math.PI / 2; grip.position.set(0, y, topZ); grip.castShadow = true; g.add(grip);
    }
    return g;
  }
  const emojiTex = new Map();
  function emojiSprite(emoji) {
    if (!emojiTex.has(emoji)) {
      const c = document.createElement("canvas"); c.width = c.height = 128;
      const x = c.getContext("2d"); x.font = "100px serif"; x.textAlign = "center"; x.textBaseline = "middle"; x.fillText(emoji, 64, 70);
      const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; emojiTex.set(emoji, t);
    }
    const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: emojiTex.get(emoji), transparent: true, depthTest: true, depthWrite: false }));
    sp.center.set(0.5, 0.06); // anchor near the bottom edge, so the item stands on the basket floor
    sp.scale.set(0.11, 0.11, 1); return sp;
  }
  const cmdArrow = new ForceArrow(view.scene, cfg.colors.cmd);
  const tenArrow = new ForceArrow(view.scene, cfg.colors.applied);

  // state
  let lastLL = new Float32Array(29), lastRaw = new Float32Array(6), fcmd = [[0, 0, 0], [0, 0, 0]];
  let llObs = new Float32Array(OBS_DIM), frame = new Float32Array(HL_FRAME_DIM);
  let massTarget = cfg.bag_mass, massNow = cfg.bag_mass, pendingKg = 0, frozen = false, running = true, stepping = false, acc = 0, prevT = performance.now();
  let tension = 0;
  function setMass(kg) {
    const q = Float64Array.from(data.qpos), v = Float64Array.from(data.qvel);
    const r = kg / model.body_mass[objBody]; model.body_mass[objBody] = kg;
    for (let k = 0; k < 3; k++) model.body_inertia[3 * objBody + k] *= r;
    mujoco.mj_setConst(model, data); data.qpos.set(q); data.qvel.set(v); mujoco.mj_forward(model, data);
    massNow = kg;
  }
  function reset() {
    for (let k = 0; k < model.nq; k++) data.qpos[k] = cfg.init_qpos[k];
    data.qvel.fill(0); data.xfrc_applied.fill(0); data.ctrl.fill(0);
    lastLL.fill(0); lastRaw.fill(0); fcmd = [[0, 0, 0], [0, 0, 0]]; hist.reset();
    massTarget = cfg.bag_mass; pendingKg = 0; setMass(cfg.bag_mass); tension = 0;
    if (typeof clearItems === "function") clearItems();
  }
  reset();
  const wristPos = () => [data.xpos[3 * rwBody], data.xpos[3 * rwBody + 1], data.xpos[3 * rwBody + 2]];
  const fbPos = (b) => [data.xpos[3 * b], data.xpos[3 * b + 1], data.xpos[3 * b + 2]];

  function policyStep() {
    // smooth mass change: a few kg per second, no impulse
    if (massNow !== massTarget) {
      const d = Math.max(-cfg.mass_rate * cfg.policy_dt, Math.min(cfg.mass_rate * cfg.policy_dt, massTarget - massNow));
      setMass(Math.abs(massTarget - massNow) <= Math.abs(d) ? massTarget : massNow + d);
    }
    buildHlFrame({ model, data, idx, ref, forceCmdSym: fcmd, lastRaw, hl, objBody, objQadr, objVadr, objRefPosW: cfg.obj_ref_pos_w, objActive: true,
                   contactPointW: [fbPos(idx.forceBodies[0]), fbPos(idx.forceBodies[1])] }, frame);
    const flat = hist.push(frame);
    if (!frozen) { lastRaw.set(hlPolicy.act(flat)); fcmd = hlActionToForce(hl, lastRaw); }
    buildObs(model, data, idx, ref, fcmd, lastLL, llObs);
    lastLL.set(llPolicy.act(llObs)); applyAction(data, idx, lastLL);
    let tsum = 0;
    for (let s = 0; s < cfg.sim_substeps; s++) { mujoco.mj_step(model, data); tsum += data.sensordata[tensionAdr]; }
    tension += 0.3 * (tsum / cfg.sim_substeps - tension);
  }

  // UI
  const itemsBox = $("items");
  cfg.items.forEach(([emoji, name, kg]) => {
    const b = document.createElement("button"); b.innerHTML = `<span class="em">${emoji}</span><span class="kg">${kg} kg</span>`; b.title = name; b.setAttribute("aria-label", `${name}, ${kg} kg`);
    b.addEventListener("click", () => { if (massTarget + pendingKg + kg > cfg.mass_max + 1e-6) return; pendingKg += kg; dropItem(emoji, kg); refreshButtons(); });
    b.dataset.kg = kg; itemsBox.appendChild(b);
  });
  function refreshButtons() { itemsBox.querySelectorAll("button").forEach((b) => { b.disabled = massTarget + pendingKg + Number(b.dataset.kg) > cfg.mass_max + 1e-6; }); }
  $("empty")?.addEventListener("click", () => { massTarget = cfg.bag_mass; pendingKg = 0; clearItems(); refreshButtons(); });
  $("freeze").addEventListener("change", (e) => { frozen = e.target.checked; });
  $("reset").addEventListener("click", () => { reset(); refreshButtons(); $("freeze").checked = false; frozen = false; });
  const q = new URLSearchParams(location.search);
  if (q.has("add")) { massTarget = Math.min(cfg.mass_max, cfg.bag_mass + Number(q.get("add"))); }
  window.__demo = { get mass() { return massNow; }, get cmd() { return fcmd; }, get tension() { return tension; }, get hand() { return wristPos(); }, add(kg) { massTarget = Math.min(cfg.mass_max, massTarget + kg); } };

  // an item falls from above the basket and settles inside it, riding with the basket from then on
  function dropItem(emoji, kg) {
    const sp = emojiSprite(emoji); bag.add(sp); items.push(sp);
    // soft contact shadow on the basket floor, so the item reads as resting on it
    const blob = new THREE.Mesh(new THREE.CircleGeometry(0.038, 24), new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0, depthWrite: false }));
    blob.position.set(0, 0, BK.bottom + 0.0075); bag.add(blob); items.push(blob);
    let landed = false;
    const land = () => { if (landed || sp.parent !== bag) return; landed = true; pendingKg = Math.max(0, pendingKg - kg); massTarget = Math.min(cfg.mass_max, massTarget + kg); refreshButtons(); };
    const n = (items.length >> 1) - 1, sx = [-0.085, 0.0, 0.085], sy = [-0.045, 0.045];
    const tx = sx[n % 3] + (Math.random() - 0.5) * 0.02, ty = sy[Math.floor(n / 3) % 2] + (Math.random() - 0.5) * 0.01;
    const tz = BK.bottom + 0.007; // standing on the floor; the basket is shallow enough that the tops show above the rim
    blob.position.x = tx; blob.position.y = ty;
    const t0 = performance.now(), dur = 650;
    const tick = (now) => {
      const u = Math.min(1, (now - t0) / dur), e = u * u;
      sp.position.set(tx, ty, tz + (1 - e) * 0.5);
      const squash = u > 0.9 ? 1 - 0.25 * Math.sin((u - 0.9) / 0.1 * Math.PI) : 1; // a small bounce on landing
      sp.scale.set(0.11 / squash, 0.11 * squash, 1);
      blob.material.opacity = 0.32 * e; const bs = 0.5 + 0.5 * e; blob.scale.set(bs, bs, 1);
      if (u >= 0.9) land(); // the weight arrives as the item touches the bottom
      if (u < 1) requestAnimationFrame(tick);
    };
    requestAnimationFrame(tick);
  }
  function clearItems() { items.forEach((o) => bag.remove(o)); items.length = 0; pendingKg = 0; }

  let frames = 0, fpsT = performance.now();
  function loop(now) {
    const wall = Math.min((now - prevT) / 1000, 0.1); prevT = now;
    if (running && !stepping) {
      stepping = true; acc += wall; let n = 0;
      while (acc >= cfg.policy_dt && n < 3) { policyStep(); acc -= cfg.policy_dt; n++; }
      if (acc > cfg.policy_dt) acc = 0;
      stepping = false;
    }
    view.update(data);
    const bp = 3 * objBody, X = data.xpos, Rw = data.xmat, wr = 9 * rwBody;
    // forearm axis = wrist link x axis, projected to the ground; basket long axis (x) across it
    const fx = Rw[wr], fy = Rw[wr + 3], fl = Math.hypot(fx, fy) || 1, ax = -fy / fl, ay = fx / fl;
    // horizontal position from the wrist (the body hangs within 1 cm of it and swings a little; drawing it
    // under the body would make the handles slide along the arm), height from the physics body
    const wp = 3 * rwBody;
    bag.matrix.set(ax, -ay, 0, X[wp], ay, ax, 0, X[wp + 1], 0, 0, 1, X[bp + 2], 0, 0, 0, 1); bag.matrixWorldNeedsUpdate = true;
    const w = wristPos();
    cmdArrow.set(w, symToWorld(idx, fcmd)[1], ARROW_SCALE);
    tenArrow.set(w, [0, 0, -tension], ARROW_SCALE);
    view.render();
    put("mass", massNow.toFixed(1));
    put("cmdN", Math.hypot(fcmd[1][0], fcmd[1][1], fcmd[1][2]).toFixed(1));
    put("tension", tension.toFixed(1));
    if (maxFrames && frames >= maxFrames) { status(`stopped after ${frames} frames; cmd ${$("cmdN").textContent} N`); return; }
    requestAnimationFrame(loop);
  }
  const maxFrames = Number(new URLSearchParams(location.search).get("frames") || 0);
  status(""); $("panel").classList.remove("loading"); refreshButtons();
  requestAnimationFrame(loop);
}
main().catch((e) => { status("error: " + e.message); console.error(e); });
