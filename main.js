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
  const groups = new Map();
  lazy.forEach((v) => {
    const g = v.closest(".grid") || v;
    if (!groups.has(g)) groups.set(g, []);
    groups.get(g).push(v);
    v.removeAttribute("loop");
  });

  function groupOf(v) { return groups.get(v.closest(".grid") || v); }

  function startGroup(vs) {
    if (reduce) return;
    const ready = () => vs.every((v) => v.readyState >= 3);
    const go = () => {
      vs.forEach((v) => { v.currentTime = 0; const p = v.play(); if (p && p.catch) p.catch(() => {}); });
    };
    if (ready()) { go(); return; }
    let pending = vs.length;
    vs.forEach((v) => {
      if (v.readyState >= 3) { pending -= 1; return; }
      v.addEventListener("canplaythrough", () => { pending -= 1; if (pending === 0) go(); }, { once: true });
    });
  }

  lazy.forEach((v) => {
    v.addEventListener("ended", () => {
      const vs = groupOf(v);
      if (vs.every((x) => x.ended || x.paused)) {
        if (vs.some((x) => x.ended) && vs.every((x) => x.ended)) startGroup(vs);
      }
    });
  });

  const active = new Set();
  if ("IntersectionObserver" in window) {
    const near = new IntersectionObserver((entries) => {
      entries.forEach((e) => { if (e.isIntersecting) { load(e.target); near.unobserve(e.target); } });
    }, { rootMargin: "600px 0px" });
    const vis = new IntersectionObserver((entries) => {
      entries.forEach((e) => {
        const g = e.target.closest(".grid") || e.target;
        const vs = groups.get(g);
        if (e.isIntersecting && e.intersectionRatio > 0.25) {
          vs.forEach(load);
          if (!active.has(g)) { active.add(g); startGroup(vs); }
        } else if (!e.isIntersecting && active.has(g)) {
          active.delete(g);
          vs.forEach((v) => v.pause());
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
