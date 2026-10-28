// Contact force the object receives from each hand (world frame), summed over
// every contact between the object body and a geom in that hand's subtree.
// Elliptic cone: efc_force holds (normal, tangent1, tangent2, ...) directly in
// the contact frame, whose rows are the normal (geom1 -> geom2) and tangents.
export function makeHandForceReader(model, objBody, handBodies) {
  const handOf = new Int8Array(model.nbody).fill(-1);
  for (let b = 0; b < model.nbody; b++) { let p = b; while (p > 0) { const h = handBodies.indexOf(p); if (h >= 0) { handOf[b] = h; break; } p = model.body_parentid[p]; } }
  return function (data, out) {
    for (const o of out) o[0] = o[1] = o[2] = 0;
    for (let c = 0; c < data.ncon; c++) {
      const ct = data.contact.get(c);
      const b1 = model.geom_bodyid[ct.geom1], b2 = model.geom_bodyid[ct.geom2];
      let hand = -1, sign = 1;
      if (b1 === objBody && handOf[b2] >= 0) { hand = handOf[b2]; sign = -1; } else if (b2 === objBody && handOf[b1] >= 0) { hand = handOf[b1]; sign = 1; }
      if (hand >= 0) {
        const a = ct.efc_address, fr = ct.frame, f = data.efc_force, dim = Math.min(ct.dim, 3);
        for (let k = 0; k < 3; k++) { let s = 0; for (let j = 0; j < dim; j++) s += fr[3 * j + k] * f[a + j]; out[hand][k] += sign * s; }
      }
      if (typeof ct.delete === "function") ct.delete();
    }
    return out;
  };
}
