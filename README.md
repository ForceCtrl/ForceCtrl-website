# Ctrl-F project page

Static site, no build step. Any HTTP server works (videos will not autoplay from
`file://`):

```bash
python3 -m http.server 8765
```

`serve.py` is the same thing with caching off and the right MIME types for the
demos.

## Layout

| path | what |
|---|---|
| `index.html` | the page; all copy lives here |
| `styles.css` | design tokens at the top (`:root`), then one block per section; the light page with a dark video hero |
| `main.js` | lazy video loading / play-when-visible, section rail highlight, the demo tab switcher |
| `motion.js` | scroll motion: reveal on scroll, the hero settling into a card, the pinned method figure |
| `sim/` | the three in-browser demos (gitignored until the weights are released, see below) |
| `assets/clips/` | hardware clips, 720p H.264, cut by `../presentation/cut_clips.sh` from `~/Downloads/video-sel` |
| `assets/posters/` | poster frames for the clips |
| `assets/sim/` | mjlab simulation renders, cropped from the 9.18 meeting deck (`media17–22`, `media5–6`) |
| `assets/fig/` | paper figures (PNG renders of the PDFs) |
| `assets/video/` | the 2:37 supplementary video, 1080p (the submitted file with the Stage 1 diagram held 2.5 s longer at 38.5 s) |
| `assets/ctrlf_icra_draft.pdf` | paper draft |

## The in-browser demos (`sim/`)

The "Try it" section of the page loads one of three standalone pages in an
iframe, on demand: `sim/demo1.html` (Stage 1: the right hand pulls down on a
handle chained to an overhead load cell), `sim/demo2.html` (Stage 2: a shopping
basket hung on the right forearm, drop items in) and `sim/demo3.html` (Stage 2:
the milkbox held between the palms by friction, drop items in). Everything runs
client-side: MuJoCo 3.1.16 compiled to WebAssembly (`vendor/mujoco`, the
single-thread build, so no cross-origin isolation is needed), three.js 0.170
for rendering, and the policies as plain-JS fp16 MLPs (`js/policy.js`,
`policies/*.bin`). The pages address every asset relative to themselves, so
they work from any site root; large files are fetched as `.gz` twins and
inflated in the browser. One demo is about 7 MB.

Loading: this WASM build takes 1.6 s to compile *any* XML, even an empty
model, so the demos load precompiled binaries instead (`scene/*.mjb`, built by
`build/compile_mjb.mjs` with the same engine build; rebuild them whenever a
scene XML changes). Those binaries hold tiny placeholder meshes (a box whose
MuJoCo mesh frame is the identity); the real robot meshes come from one packed
file, `scene/g1_meshes.bin` (`build/pack_meshes.py`), and are drawn at the
geom frames. Physics is unaffected: the visual meshes have no collision and
the inertias come from the `<inertial>` tags (checked in
`build/binary_equiv.mjs`, identical closed loop).

Shared code in `sim/js/`: `g1sim.mjs` (scene load, low-level observation,
action to ctrl), `hlobs.js` (high-level 169-D frame and 5-step history),
`render.js` (robot meshes with crease-angle normals, force arrows),
`contact.js` (per-hand contact force on an object), `policy.js`.

Build inputs live in `sim/build/` and were produced on a GPU host from the
mjlab checkout:

- `dump_ll_layout.py`, `export_scene_ref.py`: the hands-only tracker's layout,
  scene and a reference rollout (clip `raise_hold_static:v0`).
- `export_hl.py`, `export_raise.py`: the adaptive-hold high-level policy on the
  one-hand hold clip and on the raise clip (`hl/`, `raise/`): scene, ONNX,
  layout, reference rollout.
- `onnx_to_bin.py`: ONNX actor to the fp16 blob + meta + static reference frame.
- `compile_mjb.mjs`, `pack_meshes.py`: the precompiled scenes and the mesh pack (see above).
- `make_demo_xml.py`: an exported scene to a demo scene (single-thread compile,
  elliptic cone, no contact sensors, only the objects the demo uses).
- `check_*.mjs`, `demo2_node.mjs`, `demo3_node.mjs`, `cord_test.mjs`,
  `pulldown_test.mjs`: Node replicas of the browser loops, checked against the
  mjlab rollouts.

Robot meshes in `sim/scene/assets/` are quadric-decimated copies of the mjlab
G1 STLs (12k faces per part, 24k for the torso, 16k for the hands).

The demos are gitignored because they carry the policy weights. To publish
them, drop `sim/` from `.gitignore` and leave out `sim/build/` and the
`vendor/*.tgz` archives (the page needs about 30 MB of `sim/`).

## Things to update before camera-ready

- `Code · upon acceptance` button in the hero → real repository link.
- Paper PDF and figures if the camera-ready differs from the draft.

## Deploying

GitHub Pages works as-is: push this directory to a repo and serve from the root.
Assets total about 70 MB; the largest single file is the 31 MB supplementary
video. `sim/` is gitignored for now, so the "Try it" tabs 404 on the deployed
page until it is published (see above).
