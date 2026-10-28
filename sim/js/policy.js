// Actor MLP in plain JS: observation normaliser + ELU layers, weights as fp16.
// The reference frame of the (static) hold clip is a small JSON next to it.
function f16to32(u) {
  const s = (u & 0x8000) ? -1 : 1, e = (u >> 10) & 0x1f, f = u & 0x3ff;
  if (e === 0) return s * f * Math.pow(2, -24);
  if (e === 31) return f ? NaN : s * Infinity;
  return s * (1 + f / 1024) * Math.pow(2, e - 15);
}
import { fetchMaybeGz } from "./g1sim.mjs";
export class Policy {
  constructor(meta, blob, ref, layout, idx) {
    this.meta = meta; this.layout = layout; this.idx = idx;
    const u16 = new Uint16Array(blob), w = new Float32Array(u16.length);
    for (let i = 0; i < u16.length; i++) w[i] = f16to32(u16[i]);
    const take = ([s, n]) => w.subarray(s, s + n);
    this.mean = take(meta.mean); this.std = take(meta.std);
    this.layers = meta.layers.map((L) => ({ w: take(L.w), b: take(L.b), out: L.out, in: L.in }));
    this.bufs = this.layers.map((L) => new Float32Array(L.out));
    this.h = new Float32Array(meta.obs_dim);
    // static reference
    const ai = layout.motion_body_names.indexOf(layout.anchor_body), rw = layout.motion_body_names.indexOf("right_wrist_yaw_link"), lw = layout.motion_body_names.indexOf("left_wrist_yaw_link");
    this.ref = { jointPos: ref.joint_pos, jointVel: ref.joint_vel, anchorPosW: ref.body_pos_w[ai], anchorQuatW: ref.body_quat_w[ai],
                 rightWristRef: ref.body_pos_w[rw], leftWristRef: ref.body_pos_w[lw], pelvisRef: ref.body_pos_w[0], timeStep: ref.time_step };
  }
  static async load(base, layout, idx, refUrl = base + "_ref.json") {
    const [meta, blob, ref] = await Promise.all([fetch(base + ".json").then((r) => r.json()), fetchMaybeGz(base + ".bin").then((u8) => u8.buffer.slice(u8.byteOffset, u8.byteOffset + u8.byteLength)), fetch(refUrl).then((r) => r.json())]);
    return new Policy(meta, blob, ref, layout, idx);
  }
  /** obs: Float32Array(obs_dim) -> Float32Array(action_dim) (reused buffer). */
  act(obs) {
    const h = this.h;
    for (let i = 0; i < h.length; i++) h[i] = (obs[i] - this.mean[i]) / this.std[i];
    let x = h;
    for (let l = 0; l < this.layers.length; l++) {
      const L = this.layers[l], y = this.bufs[l], W = L.w, nin = L.in;
      for (let o = 0; o < L.out; o++) {
        let s = L.b[o]; const row = o * nin;
        for (let i = 0; i < nin; i++) s += W[row + i] * x[i];
        y[o] = (l < this.layers.length - 1 && s < 0) ? Math.expm1(s) : s;
      }
      x = y;
    }
    return x;
  }
}
