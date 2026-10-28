/* Lazy video playback: attach sources near the viewport, play only while visible. */
(function () {
  "use strict";
  const lazy = Array.from(document.querySelectorAll("video[data-lazy]"));
  const reduce = window.matchMedia("(prefers-reduced-motion: reduce)").matches;

  function load(v) {
    if (v.dataset.loaded) return;
    v.dataset.loaded = "1";
    v.preload = "auto";
    v.load();
  }

  // Videos inside one comparison grid form a sync group: they start together
  // once every member can play, the shorter ones hold their last frame, and the
  // whole group restarts when the longest has ended.
  // The task tiles are a looser group: they start together once all can play,
  // then each loops on its own (their lengths differ too much to hold a clock).
  const keyOf = (v) => v.closest(".grid, .tiles") || v;
  const free = (g) => g.classList && g.classList.contains("tiles");
  const groups = new Map();
  lazy.forEach((v) => {
    const g = keyOf(v);
    if (!groups.has(g)) groups.set(g, []);
    groups.get(g).push(v);
    if (!free(g)) v.removeAttribute("loop");
  });

  // The longest clip is the group's clock; the others are pulled to its time
  // whenever they drift by more than 80 ms, hold their last frame once they
  // run out, and all restart together when the clock ends.
  const clocks = new Map();
  function master(vs) {
    return vs.reduce((m, v) => (!m || (v.duration || 0) > (m.duration || 0) ? v : m), null);
  }
  function startGroup(vs) {
    if (reduce) return;
    const waiting = vs.filter((v) => v.readyState < 3);
    if (waiting.length) {
      let pending = waiting.length;
      waiting.forEach((v) => v.addEventListener("canplay", () => { pending -= 1; if (pending === 0) startGroup(vs); }, { once: true }));
      return;
    }
    vs.forEach((v) => { v.currentTime = 0; });
    Promise.all(vs.map((v) => v.play().catch(() => {}))).then(() => {
      vs.forEach((v) => { v.currentTime = 0; });
    });
    if (vs.free || clocks.has(vs)) return;
    const m = master(vs);
    const tick = () => {
      if (!clocks.has(vs)) return;
      const t = m.currentTime;
      vs.forEach((v) => {
        if (v === m) return;
        const target = Math.min(t, Math.max((v.duration || t) - 0.05, 0));
        if (Math.abs(v.currentTime - target) > 0.08) v.currentTime = target;
        if (v.ended || v.currentTime >= (v.duration || Infinity) - 0.06) { if (!v.paused) v.pause(); }
        else if (v.paused && !m.paused) { v.play().catch(() => {}); }
      });
      if (m.ended) {
        vs.forEach((v) => { v.currentTime = 0; });
        vs.forEach((v) => v.play().catch(() => {}));
      }
      requestAnimationFrame(tick);
    };
    clocks.set(vs, true);
    requestAnimationFrame(tick);
  }
  function stopGroup(vs) {
    clocks.delete(vs);
    vs.forEach((v) => v.pause());
  }

  groups.forEach((vs, g) => { vs.free = free(g); });
  const active = new Set();
  if ("IntersectionObserver" in window) {
    const near = new IntersectionObserver((entries) => {
      entries.forEach((e) => { if (e.isIntersecting) { load(e.target); near.unobserve(e.target); } });
    }, { rootMargin: "600px 0px" });
    // A group plays while any member is well in view and stops only once every
    // member has left the screen (a tall group would otherwise restart as its
    // top row scrolls out).
    const seen = new Map();
    const vis = new IntersectionObserver((entries) => {
      entries.forEach((e) => {
        seen.set(e.target, e.isIntersecting ? e.intersectionRatio : 0);
        const g = keyOf(e.target);
        const vs = groups.get(g);
        if (e.isIntersecting && e.intersectionRatio > 0.25) {
          vs.forEach(load);
          if (!active.has(g)) { active.add(g); startGroup(vs); }
        } else if (active.has(g) && vs.every((v) => !(seen.get(v) > 0))) {
          active.delete(g);
          stopGroup(vs);
        }
      });
    }, { threshold: [0, 0.25, 0.6] });
    lazy.forEach((v) => { near.observe(v); vis.observe(v); });
  } else {
    groups.forEach((vs) => { vs.forEach(load); startGroup(vs); });
  }

  // nav: highlight the section in view
  const links = Array.from(document.querySelectorAll(".nav-links a"));
  const targets = links.map((a) => document.querySelector(a.getAttribute("href"))).filter(Boolean);
  if (targets.length && "IntersectionObserver" in window) {
    const ratio = new Map();
    const io = new IntersectionObserver((entries) => {
      entries.forEach((e) => ratio.set(e.target.id, e.isIntersecting ? e.intersectionRatio : 0));
      let best = null, bestRatio = 0;
      ratio.forEach((r, id) => { if (r > bestRatio) { bestRatio = r; best = id; } });
      links.forEach((a) => a.classList.toggle("is-current", best !== null && a.getAttribute("href") === "#" + best));
    }, { rootMargin: "-20% 0px -60% 0px", threshold: [0, 0.1, 0.3, 0.6] });
    targets.forEach((t) => io.observe(t));
  }

  if (reduce) {
    const hero = document.querySelector("video[data-hero]");
    if (hero) { hero.removeAttribute("autoplay"); hero.pause(); }
    lazy.forEach((v) => {
      v.addEventListener("click", () => { v.paused ? v.play() : v.pause(); });
      v.style.cursor = "pointer";
    });
  }
})();

/* Live demos: one iframe, loaded on demand, swapped when another tab is picked. */
(function () {
  "use strict";
  const tabs = Array.from(document.querySelectorAll(".demo-tabs button"));
  const frame = document.getElementById("demo-iframe");
  const placeholder = document.querySelector(".demo-placeholder");
  if (!tabs.length || !frame) return;
  function select(b) {
    tabs.forEach((t) => t.classList.toggle("is-active", t === b));
    frame.hidden = false; if (placeholder) placeholder.hidden = true;
    frame.src = b.dataset.src; // assigning a new src discards the previous demo and its WASM instance
  }
  tabs.forEach((b) => b.addEventListener("click", () => select(b)));
  // The first demo loads by itself, once the section comes near the viewport
  // (it is about 10 MB, so not on page load).
  const section = frame.closest("section") || frame;
  if ("IntersectionObserver" in window) {
    const io = new IntersectionObserver((entries) => {
      if (!entries.some((e) => e.isIntersecting)) return;
      io.disconnect();
      if (!tabs.some((t) => t.classList.contains("is-active"))) select(tabs[0]);
    }, { rootMargin: "400px 0px" });
    io.observe(section);
  } else {
    select(tabs[0]);
  }
})();
