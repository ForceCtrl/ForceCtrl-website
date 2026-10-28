// Shared browser/node core: load the G1 scene into MuJoCo WASM, build the
// hands-only tracker observation, and run one policy step.
export const OBS_DIM = 163;

export function quatInv(q) { return [q[0], -q[1], -q[2], -q[3]]; }
export function quatMul(a, b) {
  return [
    a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
    a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
    a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
    a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]];
}
export function quatRotate(q, v) { // rotate v by q (wxyz)
  const [w, x, y, z] = q;
  const tx = 2*(y*v[2] - z*v[1]), ty = 2*(z*v[0] - x*v[2]), tz = 2*(x*v[1] - y*v[0]);
  return [v[0] + w*tx + (y*tz - z*ty), v[1] + w*ty + (z*tx - x*tz), v[2] + w*tz + (x*ty - y*tx)];
}
export function quatToMat(q) { // row-major 3x3
  const [w, x, y, z] = q;
  return [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
          2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
          2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)];
}

/** Precompiled model (.mjb from build/compile_mjb.mjs, same engine build): no XML
 *  compile, which costs 1.6 s in this WASM build even for an empty model. */
export function loadSceneBinary(mujoco, bytes, name = "scene.mjb") {
  const vfs = new mujoco.MjVFS(); vfs.addBuffer(name, bytes);
  const model = mujoco.MjModel.mj_loadBinary(name, vfs);
  const data = new mujoco.MjData(model);
  return { model, data };
}

/** Fetch the .gz twin of a file and inflate it in the browser (static hosts do
 *  not compress binaries); falls back to the plain file. */
export const netStats = { bytes: 0, files: 0 }; // bytes received so far, for a download progress line
async function readAll(r) {
  if (!r.body || !r.body.getReader) { const b = new Uint8Array(await r.arrayBuffer()); netStats.bytes += b.length; netStats.files++; return b; }
  const reader = r.body.getReader(), chunks = []; let n = 0;
  for (;;) { const { done, value } = await reader.read(); if (done) break; chunks.push(value); n += value.length; netStats.bytes += value.length; }
  const out = new Uint8Array(n); let o = 0; for (const c of chunks) { out.set(c, o); o += c.length; }
  netStats.files++; return out;
}
export async function fetchMaybeGz(u) {
  if (typeof DecompressionStream === "function") {
    const r = await fetch(u + ".gz");
    if (r && r.ok) {
      const buf = await readAll(r);
      if (buf[0] !== 0x1f || buf[1] !== 0x8b) return buf; // host already inflated it
      return new Uint8Array(await new Response(new Blob([buf]).stream().pipeThrough(new DecompressionStream("gzip"))).arrayBuffer());
    }
  }
  return readAll(await fetch(u));
}

/** Mesh pack from build/pack_meshes.py: JSON header + welded float32 vertices and uint32 faces per mesh. */
export function parseMeshPack(bytes) {
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const hl = dv.getUint32(0, true), header = JSON.parse(new TextDecoder().decode(bytes.subarray(4, 4 + hl))), base = 4 + hl;
  const out = {};
  for (const h of header) {
    const vb = bytes.byteOffset + base + h.off, fb = vb + 12 * h.nv;
    out[h.name] = { vert: new Float32Array(bytes.buffer.slice(vb, vb + 12 * h.nv)), face: new Uint32Array(bytes.buffer.slice(fb, fb + 12 * h.nf)) };
  }
  return out;
}

/** Name of object i of a given address table (mesh, body, ...). */
export function mjName(model, adrTable, i) { let s = ""; for (let k = adrTable[i]; model.names[k]; k++) s += String.fromCharCode(model.names[k]); return s; }

export async function loadScene(mujoco, xmlText, assets, dir = "/scene") {
  try { mujoco.FS.mkdir(dir); } catch (e) {}
  try { mujoco.FS.mkdir(dir + "/assets"); } catch (e) {}
  for (const [name, bytes] of Object.entries(assets)) mujoco.FS.writeFile(`${dir}/assets/${name}`, bytes);
  mujoco.FS.writeFile(`${dir}/scene.xml`, xmlText);
  const model = mujoco.MjModel.mj_loadXML(`${dir}/scene.xml`);
  const data = new mujoco.MjData(model);
  return { model, data };
}

/** Index tables derived from the layout JSON and the compiled model. */
export function buildIndex(mujoco, model, layout) {
  const nj = layout.joint_names.length;
  const qadr = [], vadr = [];
  for (const name of layout.joint_names) {
    const j = model.jnt("robot/" + name);
    qadr.push(model.jnt_qposadr[j.id]); vadr.push(model.jnt_dofadr[j.id]);
  }
  // actuator i drives which layout joint?
  const actJoint = [];
  for (let a = 0; a < model.nu; a++) {
    const jid = model.actuator_trnid[2 * a];
    const jname = model.jnt(jid).name.replace(/^robot\//, "");
    actJoint.push(layout.joint_names.indexOf(jname));
  }
  const anchorBody = model.body("robot/" + layout.anchor_body).id;
  const anchorMotionIdx = layout.motion_body_names.indexOf(layout.anchor_body);
  const forceBodies = layout.force_body_names.map((n) => model.body("robot/" + n).id);
  const gyroAdr = model.sensor_adr[model.sensor("robot/imu_ang_vel").id];
  const scale = layout.action_terms.joint_pos.fields.scale;
  const actionScale = layout.joint_names.map((n) => {
    for (const [pat, s] of Object.entries(scale)) if (new RegExp("^" + pat + "$").test(n)) return s;
    throw new Error("no scale for " + n);
  });
  // ONNX reference joints come in motion-NPZ order; argsort of their robot ids puts them in robot order.
  const motionIds = layout.motion_joint_names.map((n) => layout.joint_names.indexOf(n));
  const motionPerm = motionIds.map((id, i) => [id, i]).sort((a, b) => a[0] - b[0]).map((x) => x[1]);
  return { nj, qadr, vadr, actJoint, motionPerm, anchorBody, anchorMotionIdx, forceBodies, gyroAdr, actionScale,
           defaultJointPos: layout.default_joint_pos, Msym: layout.M_sym_to_link };
}

/**
 * Observation for the hands-only tracker.
 * ref: {jointPos[29], jointVel[29], anchorPosW[3], anchorQuatW[4]} from the ONNX reference outputs.
 * forceCmdSym: [[lx,ly,lz],[rx,ry,rz]] in the symmetric frame. lastAction: Float32Array(29).
 */
export function buildObs(model, data, idx, ref, forceCmdSym, lastAction, out) {
  out = out || new Float32Array(OBS_DIM);
  let k = 0;
  for (let i = 0; i < 29; i++) out[k++] = ref.jointPos[idx.motionPerm[i]];
  for (let i = 0; i < 29; i++) out[k++] = ref.jointVel[idx.motionPerm[i]];
  for (let b = 0; b < 2; b++) for (let i = 0; i < 3; i++) out[k++] = forceCmdSym[b][i];
  // anchor pose relative to the robot's anchor body
  const b = idx.anchorBody;
  const rp = [data.xpos[3*b], data.xpos[3*b+1], data.xpos[3*b+2]];
  const rq = [data.xquat[4*b], data.xquat[4*b+1], data.xquat[4*b+2], data.xquat[4*b+3]];
  const rqi = quatInv(rq);
  const dp = [ref.anchorPosW[0]-rp[0], ref.anchorPosW[1]-rp[1], ref.anchorPosW[2]-rp[2]];
  const pb = quatRotate(rqi, dp);
  out[k++] = pb[0]; out[k++] = pb[1]; out[k++] = pb[2];
  const m = quatToMat(quatMul(rqi, ref.anchorQuatW));
  out[k++] = m[0]; out[k++] = m[1]; out[k++] = m[3]; out[k++] = m[4]; out[k++] = m[6]; out[k++] = m[7];
  for (let i = 0; i < 3; i++) out[k++] = data.sensordata[idx.gyroAdr + i];
  for (let i = 0; i < 29; i++) out[k++] = data.qpos[idx.qadr[i]] - idx.defaultJointPos[i];
  for (let i = 0; i < 29; i++) out[k++] = data.qvel[idx.vadr[i]];
  for (let i = 0; i < 29; i++) out[k++] = lastAction[i];
  return out;
}

/** Write joint targets from a raw action into data.ctrl (actuator order). */
export function applyAction(data, idx, action) {
  for (let a = 0; a < idx.actJoint.length; a++) {
    const j = idx.actJoint[a];
    data.ctrl[a] = idx.defaultJointPos[j] + idx.actionScale[j] * action[j];
  }
}

/** Force command in the symmetric frame -> world-frame force on the link (force_hold_frame = world). */
export function symToWorld(idx, forceCmdSym) {
  return forceCmdSym.map((f, b) => {
    const M = idx.Msym[b];
    return [M[0][0]*f[0] + M[0][1]*f[1] + M[0][2]*f[2],
            M[1][0]*f[0] + M[1][1]*f[1] + M[1][2]*f[2],
            M[2][0]*f[0] + M[2][1]*f[1] + M[2][2]*f[2]];
  });
}
