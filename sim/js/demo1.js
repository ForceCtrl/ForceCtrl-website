// Demo 1: Stage-1 force gauge. The right hand holds a stirrup handle chained to
// an overhead frame (an isometric pull-down rig); the slider sets a downward
// force command and the load cell in the chain shows what the low-level policy
// actually pulls. The left hand is unloaded.
import loadMujoco from "../vendor/mujoco/mujoco.js";
import { loadSceneBinary, parseMeshPack, fetchMaybeGz, netStats, buildIndex, buildObs, applyAction, symToWorld, OBS_DIM } from "./g1sim.mjs";
import { RobotView, ForceArrow, THREE } from "./render.js";
import { Policy } from "./policy.js";

// every asset is addressed relative to this file, so the demos work from any site root
const SIM = new URL("..", import.meta.url).href;
const $ = (id) => document.getElementById(id);
const put = (id, text) => { const el = document.getElementById(id); if (el) el.textContent = text; }; // panel fields are optional
const status = (t) => { $("status").textContent = t; console.log("[demo1]", t); };
const ARROW_SCALE = 0.009; // metres per newton, as in the paper figures

async function fetchJSON(u) { return (await fetch(u)).json(); }

async function main() {
  const t0 = performance.now();
  const ticker = setInterval(() => status(`downloading… ${(netStats.bytes / 1e6).toFixed(1)} MB, ${netStats.files} files, ${((performance.now() - t0) / 1000).toFixed(0)} s`), 250);
  const [mujoco, layout, cfg, mjb, meshes] = await Promise.all([
    fetchMaybeGz(SIM + "vendor/mujoco/mujoco.wasm").then((wasmBinary) => loadMujoco({ wasmBinary })), fetchJSON(SIM + "build/layout/ll_layout.json"), fetchJSON(SIM + "scene/demo_config.json"),
    fetch(SIM + "scene/g1_base.mjb").then((r) => r.arrayBuffer()).then((b) => new Uint8Array(b)), fetchMaybeGz(SIM + "scene/g1_meshes.bin").then(parseMeshPack),
  ]);
  const { model, data } = loadSceneBinary(mujoco, mjb, "g1_base.mjb"); // precompiled with placeholder meshes; the real ones are drawn from the pack
  console.log("[demo1] model loaded nq", model.nq);
  const idx = buildIndex(mujoco, model, layout);

  status("loading policy…");
  const policy = await Policy.load(SIM + "policies/ll", layout, idx);
  clearInterval(ticker); status(`loaded ${(netStats.bytes / 1e6).toFixed(1)} MB in ${((performance.now() - t0) / 1000).toFixed(1)} s; building the scene…`);
  const ref = policy.ref; // static hold clip: one frame serves throughout

  console.log("[demo1] reference ready");
  // scene
  const nogl = new URLSearchParams(location.search).has("nogl");
  let view;
  try { view = nogl ? null : new RobotView($("view"), cfg.colors); } catch (e) { status("WebGL unavailable: " + e.message); view = null; }
  if (!view) view = { scene: { add() {} }, buildRobot() {}, frame() {}, update() {}, render() {}, canvas: $("view"), camera: null };
  view.buildRobot(model, meshes);
  { const [qw, qx, qy, qz] = cfg.init_qpos.slice(3, 7); // pelvis heading of the rest pose
    view.frame(ref.pelvisRef[0], ref.pelvisRef[1], Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))); }
  const cmdArrow = new ForceArrow(view.scene, cfg.colors.cmd);
  const extArrow = new ForceArrow(view.scene, cfg.colors.applied);
  const hand = idx.forceBodies[1]; // right wrist
  const palmSite = model.site("robot/right_palm").id;
  const palmPos = () => [data.site_xpos[3 * palmSite], data.site_xpos[3 * palmSite + 1], data.site_xpos[3 * palmSite + 2]];

  // pull-down rig: an overhead crossbar on two uprights, a chain with a load cell
  // down to a stirrup handle at the right palm. The chain is a one-sided stiff
  // spring-damper in the physics (tension only, 1500 N/m, under 3 cm at 40 N).
  for (let k = 0; k < model.nq; k++) data.qpos[k] = cfg.init_qpos[k]; data.qvel.fill(0); mujoco.mj_forward(model, data); // rest pose for the rig geometry
  const TOP = 1.95, rest = palmPos();
  const anchor = [rest[0], rest[1], TOP];
  const chain = { k: 1500, c: 55, L0: TOP - rest[2], F: 0, dir: [0, 0, -1], prev: null, acc: 0, n: 0 };
  {
    const steel = new THREE.MeshStandardMaterial({ color: 0x2a2d36, roughness: 0.5, metalness: 0.4 });
    const alu = new THREE.MeshStandardMaterial({ color: 0xb9b4aa, roughness: 0.4, metalness: 0.6 });
    const cx = rest[0], cy = 0.0; // frame centred on the robot, the chain hangs over the right hand
    for (const sy of [-1, 1]) { // uprights 1.2 m apart, feet on the floor
      const post = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.06, TOP + 0.03), steel); post.position.set(cx, cy + sy * 0.6, (TOP + 0.03) / 2); post.castShadow = true; view.scene.add(post);
      const foot = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.08, 0.03), steel); foot.position.set(cx, cy + sy * 0.6, 0.015); foot.castShadow = true; view.scene.add(foot);
    }
    const bar = new THREE.Mesh(new THREE.BoxGeometry(0.06, 1.26, 0.06), steel); bar.position.set(cx, cy, TOP); bar.castShadow = true; view.scene.add(bar);
    const cell = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.035, 0.07), alu); cell.position.set(anchor[0], anchor[1], TOP - 0.03 - 0.035); cell.castShadow = true; view.scene.add(cell); // load cell under the bar
  }
  const chainMat = new THREE.LineBasicMaterial({ color: 0x1a1a1a });
  const chainGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
  view.scene.add(new THREE.Line(chainGeo, chainMat));
  // stirrup handle: a ring around the palm with a short strap up to the chain
  const handle = (() => {
    const g = new THREE.Group();
    const ring = new THREE.Mesh(new THREE.TorusGeometry(0.045, 0.008, 10, 24), new THREE.MeshStandardMaterial({ color: 0x1a1a1a, roughness: 0.8 }));
    ring.rotation.y = Math.PI / 2; g.add(ring); // ring in the y-z plane, the hand passes through along x
    const strap = new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.006, 0.08), new THREE.MeshStandardMaterial({ color: 0x3a2a1c, roughness: 0.9 })); strap.position.z = 0.085; g.add(strap);
    view.scene.add(g); return g;
  })();

  // state
  const dt = layout.sim_dt;
  let last = new Float32Array(29), obs = new Float32Array(OBS_DIM);
  let cmdN = 0, running = true, stepping = false, acc = 0, prevT = performance.now();
  let reading = 0;
  function reset() {
    for (let k = 0; k < model.nq; k++) data.qpos[k] = cfg.init_qpos[k];
    data.qvel.fill(0); data.xfrc_applied.fill(0); last.fill(0);
    mujoco.mj_forward(model, data); chain.prev = null; chain.F = 0; reading = 0;
  }
  reset();

  function wristPos() { return [data.xpos[3 * hand], data.xpos[3 * hand + 1], data.xpos[3 * hand + 2]]; }

  function chainSubstep() {
    const ch = chain, a = anchor, p = palmPos();
    const d = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    const len = Math.hypot(d[0], d[1], d[2]);
    const ext = len - ch.L0;
    let F = 0;
    if (ext > 0 && len > 1e-6) {
      ch.dir = [d[0] / len, d[1] / len, d[2] / len]; // from anchor to palm
      let vr = 0;
      if (ch.prev) vr = ((p[0] - ch.prev[0]) * ch.dir[0] + (p[1] - ch.prev[1]) * ch.dir[1] + (p[2] - ch.prev[2]) * ch.dir[2]) / dt;
      F = Math.max(0, ch.k * ext + ch.c * vr);
    }
    ch.prev = p; ch.F = F; ch.acc += F; ch.n += 1;
    for (let k = 0; k < 3; k++) data.xfrc_applied[6 * hand + k] = -F * ch.dir[k]; // chain pulls the hand back toward the anchor
  }

  function policyStep() {
    const cmdSym = [[0, 0, 0], [0, 0, -cmdN]];
    buildObs(model, data, idx, ref, cmdSym, last, obs);
    last.set(policy.act(obs));
    applyAction(data, idx, last);
    chain.acc = 0; chain.n = 0;
    for (let s = 0; s < cfg.sim_substeps; s++) { chainSubstep(); mujoco.mj_step(model, data); }
    reading += 0.25 * (chain.acc / chain.n - reading);
  }

  const cmdSlider = $("cmd"), cmdLabel = $("cmdval");
  const setCmd = (v) => { cmdN = Math.max(0, Math.min(40, Number(v) || 0)); cmdSlider.value = cmdN; cmdLabel.textContent = cmdN.toFixed(0); };
  cmdSlider.addEventListener("input", () => setCmd(cmdSlider.value));
  const q = new URLSearchParams(location.search);
  if (q.has("cmd")) setCmd(q.get("cmd"));
  window.__demo = { get cmd() { return cmdN; }, get reading() { return reading; }, setCmd, get hand() { return wristPos(); } };
  $("reset").addEventListener("click", reset);

  let frames = 0, fpsT = performance.now();
  function frame(now) {
    const wall = Math.min((now - prevT) / 1000, 0.1); prevT = now;
    if (running && !stepping) {
      stepping = true;
      acc += wall; let n = 0;
      while (acc >= cfg.policy_dt && n < 3) { policyStep(); acc -= cfg.policy_dt; n++; }
      if (acc > cfg.policy_dt) acc = 0; // fell behind: drop time instead of spiralling
      stepping = false;
    }
    // draw
    view.update(data);
    const p = palmPos();
    cmdArrow.set(p, symToWorld(idx, [[0, 0, 0], [0, 0, -cmdN]])[1], ARROW_SCALE);
    extArrow.set(p, [-reading * chain.dir[0], -reading * chain.dir[1], -reading * chain.dir[2]], ARROW_SCALE);
    handle.position.set(p[0], p[1], p[2]);
    const pos = chainGeo.attributes.position; pos.setXYZ(0, anchor[0], anchor[1], anchor[2] - 0.1); pos.setXYZ(1, p[0], p[1], p[2] + 0.125); pos.needsUpdate = true;
    view.render();
    put("reading", reading.toFixed(1));
    frames++; if (frames === 1) console.log("[demo1] first frame");
    if (maxFrames && frames >= maxFrames) { status(`stopped after ${frames} frames; cmd ${$("cmdval").textContent} N`); return; }
    requestAnimationFrame(frame);
  }
  const maxFrames = Number(new URLSearchParams(location.search).get("frames") || 0);
  status("");
  $("panel").classList.remove("loading");
  requestAnimationFrame(frame);
}

main().catch((e) => { status("error: " + e.message); console.error(e); });
