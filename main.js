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
