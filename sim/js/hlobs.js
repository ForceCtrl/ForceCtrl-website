// High-level (Stage 2) observation: one 169-D frame per control step, stacked
// over a 5-step history. Term order follows the task's actor group.
import { quatInv, quatMul, quatRotate, quatToMat } from "./g1sim.mjs";

export const HL_FRAME_DIM = 169;

export function quatUnique(q) { return q[0] < 0 ? [-q[0], -q[1], -q[2], -q[3]] : q; }

/**
 * ctx: { model, data, idx, ref, forceCmdSym[2][3], lastRaw[6], hlLayout, objBody, objQadr, objVadr, objRefPosW[3], objActive, contactPointW[2][3] }
 */
export function buildHlFrame(ctx, out) {
  const { model, data, idx, ref, forceCmdSym, lastRaw, hl, objBody, objQadr, objVadr } = ctx;
  out = out || new Float32Array(HL_FRAME_DIM);
  let k = 0;
  for (let i = 0; i < 29; i++) out[k++] = ref.jointPos[idx.motionPerm[i]];
  for (let i = 0; i < 29; i++) out[k++] = ref.jointVel[idx.motionPerm[i]];
  for (let b = 0; b < 2; b++) for (let i = 0; i < 3; i++) out[k++] = forceCmdSym[b][i];
  const a = idx.anchorBody;
  const rp = [data.xpos[3*a], data.xpos[3*a+1], data.xpos[3*a+2]];
  const rq = [data.xquat[4*a], data.xquat[4*a+1], data.xquat[4*a+2], data.xquat[4*a+3]];
  const rqi = quatInv(rq);
  const toB = (v) => quatRotate(rqi, v);
  const pb = toB([ref.anchorPosW[0]-rp[0], ref.anchorPosW[1]-rp[1], ref.anchorPosW[2]-rp[2]]);
  out[k++] = pb[0]; out[k++] = pb[1]; out[k++] = pb[2];
  const m = quatToMat(quatMul(rqi, ref.anchorQuatW));
  out[k++] = m[0]; out[k++] = m[1]; out[k++] = m[3]; out[k++] = m[4]; out[k++] = m[6]; out[k++] = m[7];
  for (let i = 0; i < 3; i++) out[k++] = data.sensordata[idx.gyroAdr + i];
  for (let i = 0; i < 29; i++) out[k++] = data.qpos[idx.qadr[i]] - idx.defaultJointPos[i];
  for (let i = 0; i < 29; i++) out[k++] = data.qvel[idx.vadr[i]];
  for (let i = 0; i < 6; i++) out[k++] = lastRaw[i];
  for (let b = 0; b < 2; b++) for (let i = 0; i < 3; i++) {
    const half = 0.5 * (hl.force_max_b[b][i] - hl.force_min_b[b][i]);
    out[k++] = forceCmdSym[b][i] / Math.max(half, 1e-6);
  }
  // object state relative to the anchor
  const op = [data.xpos[3*objBody], data.xpos[3*objBody+1], data.xpos[3*objBody+2]];
  const oq = [data.xquat[4*objBody], data.xquat[4*objBody+1], data.xquat[4*objBody+2], data.xquat[4*objBody+3]];
  const opb = toB([op[0]-rp[0], op[1]-rp[1], op[2]-rp[2]]);
  out[k++] = opb[0]; out[k++] = opb[1]; out[k++] = opb[2];
  const oqb = quatUnique(quatMul(rqi, oq));
  out[k++] = oqb[0]; out[k++] = oqb[1]; out[k++] = oqb[2]; out[k++] = oqb[3];
  const lv = toB([data.qvel[objVadr], data.qvel[objVadr+1], data.qvel[objVadr+2]]);
  out[k++] = lv[0]; out[k++] = lv[1]; out[k++] = lv[2];
  // free-joint angular velocity is body-local in MuJoCo; mjlab reports it in world
  const wb = [data.qvel[objVadr+3], data.qvel[objVadr+4], data.qvel[objVadr+5]];
  const av = toB(quatRotate(oq, wb));
  out[k++] = av[0]; out[k++] = av[1]; out[k++] = av[2];
  const pe = toB([ctx.objRefPosW[0]-op[0], ctx.objRefPosW[1]-op[1], ctx.objRefPosW[2]-op[2]]);
  out[k++] = pe[0]; out[k++] = pe[1]; out[k++] = pe[2];
  out[k++] = ctx.objActive ? 1 : 0;
  for (let b = 0; b < 2; b++) {
    const fb = idx.forceBodies[b];
    const fp = [data.xpos[3*fb], data.xpos[3*fb+1], data.xpos[3*fb+2]];
    const c = ctx.contactPointW[b];
    const e = toB([c[0]-fp[0], c[1]-fp[1], c[2]-fp[2]]);
    out[k++] = e[0]; out[k++] = e[1]; out[k++] = e[2];
  }
  if (k !== HL_FRAME_DIM) throw new Error("hl frame dim " + k);
  return out;
}

/** Fixed-length history of frames, flattened for the policy. */
export class History {
  constructor(len, dim, termDims, termMajor = true) {
    this.len = len; this.dim = dim; this.termDims = termDims; this.termMajor = termMajor;
    this.frames = []; this.flat = new Float32Array(len * dim);
  }
  reset() { this.frames = []; }
  push(frame) {
    if (this.frames.length === 0) for (let i = 0; i < this.len; i++) this.frames.push(Float32Array.from(frame));
    else { this.frames.shift(); this.frames.push(Float32Array.from(frame)); }
    return this.flatten();
  }
  flatten() {
    const F = this.frames, L = this.len, out = this.flat;
    if (!this.termMajor) { for (let t = 0; t < L; t++) out.set(F[t], t * this.dim); return out; }
    let k = 0, off = 0;
    for (const d of this.termDims) {
      for (let t = 0; t < L; t++) for (let i = 0; i < d; i++) out[k++] = F[t][off + i];
      off += d;
    }
    return out;
  }
}

/** Raw HL action -> force command in the symmetric frame (tanh box mapping). */
export function hlActionToForce(hl, raw) {
  const f = [[0, 0, 0], [0, 0, 0]];
  for (let j = 0; j < 6; j++) {
    const s = hl.force_use_tanh ? Math.tanh(raw[j]) : raw[j];
    const lo = hl.force_min_b[Math.floor(j / 3)][j % 3], hi = hl.force_max_b[Math.floor(j / 3)][j % 3];
    f[Math.floor(j / 3)][j % 3] = lo + (s + 1) * 0.5 * (hi - lo);
  }
  return f;
}
