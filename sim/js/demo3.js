// Demo 3: Stage-2 adaptive raise. The robot holds a crate between its palms
// (friction only, no rope); drop items in and watch the high-level policy raise
// its squeeze and support commands to match.
import loadMujoco from "../vendor/mujoco/mujoco.js";
import { loadSceneBinary, parseMeshPack, fetchMaybeGz, netStats, buildIndex, buildObs, applyAction, symToWorld, OBS_DIM } from "./g1sim.mjs";
import { buildHlFrame, History, hlActionToForce, HL_FRAME_DIM } from "./hlobs.js";
import { RobotView, ForceArrow, THREE } from "./render.js";
import { Policy } from "./policy.js";
import { makeHandForceReader } from "./contact.js";

// every asset is addressed relative to this file, so the demos work from any site root
const SIM = new URL("..", import.meta.url).href;
const $ = (id) => document.getElementById(id);
const put = (id, text) => { const el = document.getElementById(id); if (el) el.textContent = text; }; // panel fields are optional
const status = (t) => { $("status").textContent = t; console.log("[demo3]", t); };
const ARROW_SCALE = 0.009;
const G = 9.81;
async function fetchJSON(u) { return (await fetch(u)).json(); }

async function main() {
  const t0 = performance.now();
  const ticker = setInterval(() => status(`downloading… ${(netStats.bytes / 1e6).toFixed(1)} MB, ${netStats.files} files, ${((performance.now() - t0) / 1000).toFixed(0)} s`), 250);
  const [mujoco, llLayout, hl, cfg, mjb, meshes] = await Promise.all([
    fetchMaybeGz(SIM + "vendor/mujoco/mujoco.wasm").then((wasmBinary) => loadMujoco({ wasmBinary })), fetchJSON(SIM + "build/layout/ll_layout.json"), fetchJSON(SIM + "build/raise/raise_layout.json"),
    fetchJSON(SIM + "scene/raise_config.json"), fetch(SIM + "scene/raise_demo.mjb").then((r) => r.arrayBuffer()).then((b) => new Uint8Array(b)), fetchMaybeGz(SIM + "scene/g1_meshes.bin").then(parseMeshPack),
  ]);
  const { model, data } = loadSceneBinary(mujoco, mjb, "raise_demo.mjb"); // precompiled with placeholder meshes; the real ones are drawn from the pack
  const layout = { ...llLayout, motion_joint_names: hl.motion_joint_names };
  const idx = buildIndex(mujoco, model, layout);
  const objBody = model.body(hl.object_body).id, objJ = model.jnt("milkbox/milkbox_joint").id;
  const objQadr = model.jnt_qposadr[objJ], objVadr = model.jnt_dofadr[objJ];
  const lwBody = idx.forceBodies[0], rwBody = idx.forceBodies[1];
  const handForce = makeHandForceReader(model, objBody, [lwBody, rwBody]);

  status("loading policies…");
  const [llPolicy, hlPolicy] = await Promise.all([Policy.load(SIM + "policies/ll", layout, idx, SIM + "policies/raise_hl_ref.json"), Policy.load(SIM + "policies/raise_hl", layout, idx)]);
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
  // The physics body is the paper's milkbox: a 31 x 22 x 38.5 cm box held
  // between the palms. What is drawn is a shallow fruit crate with the same
  // footprint, placed so the palm pads press on its side walls; the physics box
  // continues below it, unseen. Local +z points down in this pose, so the crate
  // is built z-up in a holder that is flipped and shifted to the crate floor.
  const [HX, HY, HZ] = cfg.box_half_extents;
  const CRATE_H = cfg.crate_height || 0.14, CRATE_FLOOR = cfg.crate_floor_local_z ?? 0.01; // floor at local +0.01: the pads span local z -0.12..-0.01
  const bag = new THREE.Group(); // world pose = the physics body
  bag.matrixAutoUpdate = false; view.scene.add(bag);
  const holder = new THREE.Group(); holder.rotation.x = Math.PI; holder.position.z = CRATE_FLOOR; bag.add(holder); // holder: z up, origin at the crate floor
  const items = []; // emoji sprites and their shadows, riding inside the crate
  function slotTexture(repeatX, rows) {
    // alpha map for a lattice wall: white = plastic, black = hole
    const W = 1024, H = 1024, c = document.createElement("canvas"); c.width = W; c.height = H;
    const x = c.getContext("2d"); x.fillStyle = "#fff"; x.fillRect(0, 0, W, H); x.fillStyle = "#000";
    const top = 90, bottom = 70, mid = 40, slotW = 92, barW = 30, pitch = slotW + barW; // wide openings, like a fruit crate
    const rowH = (H - top - bottom - mid * (rows - 1)) / rows;
    const n = Math.floor((W - barW) / pitch), x0 = (W - (n * pitch - barW)) / 2;
    for (let i = 0; i < n; i++) for (let r = 0; r < rows; r++) { x.beginPath(); x.roundRect(x0 + i * pitch, top + r * (rowH + mid), slotW, rowH, 8); x.fill(); }
    const t = new THREE.CanvasTexture(c); t.wrapS = THREE.RepeatWrapping; t.repeat.x = repeatX; t.offset.x = (1 - repeatX) / 2; return t;
  }
  {
    const color = cfg.colors.crate;
    const plastic = new THREE.MeshStandardMaterial({ color, roughness: 0.45, metalness: 0.05, side: THREE.DoubleSide });
    const wallMat = (rep) => new THREE.MeshStandardMaterial({ color, roughness: 0.45, metalness: 0.05, side: THREE.DoubleSide, alphaMap: slotTexture(rep, 2), alphaTest: 0.5 });
    const h = CRATE_H;
    for (const [w, off, rot, rep] of [[2 * HX, HY, 0, 1.0], [2 * HX, -HY, Math.PI, 1.0], [2 * HY, HX, -Math.PI / 2, 0.72], [2 * HY, -HX, Math.PI / 2, 0.72]]) {
      const pivot = new THREE.Group(); pivot.rotation.z = rot;
      const wall = new THREE.Mesh(new THREE.PlaneGeometry(w, h), wallMat(rep)); wall.rotation.x = Math.PI / 2;
      wall.position.set(0, Math.abs(off), h / 2); wall.castShadow = true; wall.receiveShadow = true; pivot.add(wall); holder.add(pivot);
    }
    const floor = new THREE.Mesh(new THREE.BoxGeometry(2 * HX, 2 * HY, 0.008), plastic); floor.position.z = 0.004; holder.add(floor);
    for (const z of [0.012, h - 0.012]) for (const [len, ox, oy, rz] of [[2 * HX + 0.012, 0, HY, 0], [2 * HX + 0.012, 0, -HY, 0], [2 * HY + 0.012, HX, 0, Math.PI / 2], [2 * HY + 0.012, -HX, 0, Math.PI / 2]]) {
      const bar = new THREE.Mesh(new THREE.BoxGeometry(len, 0.012, 0.024), plastic); bar.position.set(ox, oy, z); bar.rotation.z = rz; bar.castShadow = true; holder.add(bar);
    }
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) { const post = new THREE.Mesh(new THREE.BoxGeometry(0.016, 0.016, h), plastic); post.position.set(sx * HX, sy * HY, h / 2); post.castShadow = true; holder.add(post); }
  }
  const emojiTex = new Map();
  function emojiSprite(emoji) {
    if (!emojiTex.has(emoji)) {
      const c = document.createElement("canvas"); c.width = c.height = 128;
      const x = c.getContext("2d"); x.font = "100px serif"; x.textAlign = "center"; x.textBaseline = "middle"; x.fillText(emoji, 64, 70);
      const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; emojiTex.set(emoji, t);
    }
    const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: emojiTex.get(emoji), transparent: true, depthTest: true, depthWrite: false }));
    sp.center.set(0.5, 0.06); // anchor near the bottom edge, so the item stands on the crate floor
    sp.scale.set(0.11, 0.11, 1); return sp;
  }
  const cmdArrows = [new ForceArrow(view.scene, cfg.colors.cmd), new ForceArrow(view.scene, cfg.colors.cmd)];
  const actArrows = [new ForceArrow(view.scene, cfg.colors.applied), new ForceArrow(view.scene, cfg.colors.applied)];

  // state
  let lastLL = new Float32Array(29), lastRaw = new Float32Array(6), fcmd = [[0, 0, 0], [0, 0, 0]];
  let llObs = new Float32Array(OBS_DIM), frame = new Float32Array(HL_FRAME_DIM);
  let massTarget = cfg.box_mass, massNow = cfg.box_mass, pendingKg = 0, frozen = false, running = true, stepping = false, acc = 0, prevT = performance.now();
  const handF = [[0, 0, 0], [0, 0, 0]], handFs = [[0, 0, 0], [0, 0, 0]]; // raw and smoothed hand forces on the crate
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
    massTarget = cfg.box_mass; pendingKg = 0; setMass(cfg.box_mass); for (const f of handFs) f[0] = f[1] = f[2] = 0;
    if (typeof clearItems === "function") clearItems();
  }
  reset();
  const fbPos = (b) => [data.xpos[3 * b], data.xpos[3 * b + 1], data.xpos[3 * b + 2]];
  // where the palms should press: fixed points on the box faces, carried with the box
  const toWorld = (l) => { const R = data.xmat, r = 9 * objBody, p = 3 * objBody; return [data.xpos[p] + R[r]*l[0] + R[r+1]*l[1] + R[r+2]*l[2], data.xpos[p+1] + R[r+3]*l[0] + R[r+4]*l[1] + R[r+5]*l[2], data.xpos[p+2] + R[r+6]*l[0] + R[r+7]*l[1] + R[r+8]*l[2]]; };

  function policyStep() {
    // smooth mass change: a few kg per second, no impulse
    if (massNow !== massTarget) {
      const d = Math.max(-cfg.mass_rate * cfg.policy_dt, Math.min(cfg.mass_rate * cfg.policy_dt, massTarget - massNow));
      setMass(Math.abs(massTarget - massNow) <= Math.abs(d) ? massTarget : massNow + d);
    }
    buildHlFrame({ model, data, idx, ref, forceCmdSym: fcmd, lastRaw, hl, objBody, objQadr, objVadr, objRefPosW: cfg.obj_ref_pos_w, objActive: true,
                   contactPointW: cfg.contact_point_obj_b.map(toWorld) }, frame);
    const flat = hist.push(frame);
    if (!frozen) { lastRaw.set(hlPolicy.act(flat)); fcmd = hlActionToForce(hl, lastRaw); }
    buildObs(model, data, idx, ref, fcmd, lastLL, llObs);
    lastLL.set(llPolicy.act(llObs)); applyAction(data, idx, lastLL);
    for (let s = 0; s < cfg.sim_substeps; s++) mujoco.mj_step(model, data);
    handForce(data, handF);
    for (let b = 0; b < 2; b++) for (let k = 0; k < 3; k++) handFs[b][k] += 0.3 * (handF[b][k] - handFs[b][k]);
  }

  // UI
  const itemsBox = $("items");
  cfg.items.forEach(([emoji, name, kg]) => {
    const b = document.createElement("button"); b.innerHTML = `<span class="em">${emoji}</span><span class="kg">${kg} kg</span>`; b.title = name; b.setAttribute("aria-label", `${name}, ${kg} kg`);
    b.addEventListener("click", () => { if (massTarget + pendingKg + kg > cfg.mass_max + 1e-6) return; pendingKg += kg; dropItem(emoji, kg); refreshButtons(); });
    b.dataset.kg = kg; itemsBox.appendChild(b);
  });
  function refreshButtons() { itemsBox.querySelectorAll("button").forEach((b) => { b.disabled = massTarget + pendingKg + Number(b.dataset.kg) > cfg.mass_max + 1e-6; }); }
  $("empty")?.addEventListener("click", () => { massTarget = cfg.box_mass; pendingKg = 0; clearItems(); refreshButtons(); });
  $("freeze").addEventListener("change", (e) => { frozen = e.target.checked; });
  $("reset").addEventListener("click", () => { reset(); refreshButtons(); $("freeze").checked = false; frozen = false; });
  const q = new URLSearchParams(location.search);
  if (q.has("add")) { massTarget = Math.min(cfg.mass_max, cfg.box_mass + Number(q.get("add"))); }
  window.__demo = { get mass() { return massNow; }, get cmd() { return fcmd; }, get handForce() { return handFs; }, get box() { return fbPos(objBody); }, add(kg) { massTarget = Math.min(cfg.mass_max, massTarget + kg); } };

  // an item falls from above the crate and settles inside it, riding with the crate from then on
  function dropItem(emoji, kg) {
    const sp = emojiSprite(emoji); holder.add(sp); items.push(sp);
    // soft contact shadow on the basket floor, so the item reads as resting on it
    const blob = new THREE.Mesh(new THREE.CircleGeometry(0.038, 24), new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0, depthWrite: false }));
    blob.position.set(0, 0, 0.0085); holder.add(blob); items.push(blob);
    let landed = false;
    const land = () => { if (landed || sp.parent !== holder) return; landed = true; pendingKg = Math.max(0, pendingKg - kg); massTarget = Math.min(cfg.mass_max, massTarget + kg); refreshButtons(); };
    const n = (items.length >> 1) - 1, sx = [-0.085, 0.0, 0.085], sy = [-0.04, 0.04];
    const tx = sx[n % 3] + (Math.random() - 0.5) * 0.02, ty = sy[Math.floor(n / 3) % 2] + (Math.random() - 0.5) * 0.01;
    const tz = 0.009; // standing on the crate floor, seen through the lattice
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
  function clearItems() { items.forEach((o) => holder.remove(o)); items.length = 0; pendingKg = 0; }

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
    const bp = 3 * objBody, br = 9 * objBody, X = data.xpos, R = data.xmat;
    bag.matrix.set(R[br], R[br+1], R[br+2], X[bp], R[br+3], R[br+4], R[br+5], X[bp+1], R[br+6], R[br+7], R[br+8], X[bp+2], 0, 0, 0, 1); bag.matrixWorldNeedsUpdate = true;
    const cmdW = symToWorld(idx, fcmd);
    // purple arrows: the reaction the crate puts on each hand (equal and opposite to what the hand applies), as in the other demos
    for (let b = 0; b < 2; b++) { const p = fbPos(idx.forceBodies[b]); cmdArrows[b].set(p, cmdW[b], ARROW_SCALE); actArrows[b].set(p, [-handFs[b][0], -handFs[b][1], -handFs[b][2]], ARROW_SCALE); }
    view.render();
    put("mass", massNow.toFixed(1));
    const cL = Math.hypot(...fcmd[0]), cR = Math.hypot(...fcmd[1]);
    put("cmdAvg", ((cL + cR) / 2).toFixed(0)); put("cmdL", cL.toFixed(0)); put("cmdR", cR.toFixed(0));
    put("actL", Math.hypot(...handFs[0]).toFixed(0)); put("actR", Math.hypot(...handFs[1]).toFixed(0));
    if (maxFrames && frames >= maxFrames) { status(`stopped after ${frames} frames; cmd ${$("cmdL").textContent}/${$("cmdR").textContent} N`); return; }
    requestAnimationFrame(loop);
  }
  const maxFrames = Number(new URLSearchParams(location.search).get("frames") || 0);
  status(""); $("panel").classList.remove("loading"); refreshButtons();
  requestAnimationFrame(loop);
}
main().catch((e) => { status("error: " + e.message); console.error(e); });
