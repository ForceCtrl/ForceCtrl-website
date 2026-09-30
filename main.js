/* Page behaviour: lazy video playback and the section rail. */
(function () {
  "use strict";

  // ---- videos: attach sources only when near the viewport, play only while visible
  const lazy = Array.from(document.querySelectorAll("video[data-lazy]"));
  const reduce = window.matchMedia("(prefers-reduced-motion: reduce)").matches;

  function load(v) {
    if (v.dataset.loaded) return;
    v.dataset.loaded = "1";
    v.preload = "auto";
    v.load();
  }

  if ("IntersectionObserver" in window) {
    const near = new IntersectionObserver((entries) => {
      entries.forEach((e) => { if (e.isIntersecting) { load(e.target); near.unobserve(e.target); } });
    }, { rootMargin: "600px 0px" });

    const vis = new IntersectionObserver((entries) => {
      entries.forEach((e) => {
        const v = e.target;
        if (e.isIntersecting && e.intersectionRatio > 0.25) {
          load(v);
          if (!reduce) { const p = v.play(); if (p && p.catch) p.catch(() => {}); }
        } else {
          v.pause();
        }
      });
    }, { threshold: [0, 0.25, 0.6] });

    lazy.forEach((v) => { near.observe(v); vis.observe(v); });
  } else {
    lazy.forEach((v) => { load(v); if (!reduce) v.play().catch(() => {}); });
  }

  // reduced motion: hero stays on its poster frame, other clips play on click
  if (reduce) {
    const hero = document.querySelector("video[data-hero]");
    if (hero) { hero.removeAttribute("autoplay"); hero.pause(); }
    lazy.forEach((v) => {
      v.addEventListener("click", () => { v.paused ? v.play() : v.pause(); });
      v.style.cursor = "pointer";
    });
  }

  // ---- rail: highlight the section in view
  const links = Array.from(document.querySelectorAll(".rail__list a"));
  const targets = links.map((a) => document.querySelector(a.getAttribute("href"))).filter(Boolean);
  if (targets.length && "IntersectionObserver" in window) {
    const current = new Map();
    const io = new IntersectionObserver((entries) => {
      entries.forEach((e) => current.set(e.target.id, e.isIntersecting ? e.intersectionRatio : 0));
      let best = null, bestRatio = 0;
      current.forEach((r, id) => { if (r > bestRatio) { bestRatio = r; best = id; } });
      links.forEach((a) => a.classList.toggle("is-current", best !== null && a.getAttribute("href") === "#" + best));
    }, { rootMargin: "-25% 0px -55% 0px", threshold: [0, 0.1, 0.3, 0.6] });
    targets.forEach((t) => io.observe(t));
  }
})();
