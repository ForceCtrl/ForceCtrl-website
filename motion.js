/* Scroll motion for the combined page: content rises into place, the hero video
   settles into a rounded card as the page starts to scroll, and the method figure
   stays pinned while the stage being read lights up. Everything is skipped under
   prefers-reduced-motion, and the page reads the same without this script. */
(function () {
  "use strict";
  const reduce = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  const root = document.documentElement;
  root.classList.add("js");

  /* 1. reveal: section heads, media blocks and method steps rise and fade in once */
  const items = document.querySelectorAll(
    "section.sec > .head, section.sec > .wrap-mid:not(.method-pin), section.sec > .block, .method-pin .pin, .method-pin .step, section#cite > .wrap-copy"
  );
  items.forEach((el) => el.classList.add("rv"));
  if (reduce || !("IntersectionObserver" in window)) {
    items.forEach((el) => el.classList.add("in"));
  } else {
    // Shown once it is a little way into the screen. A block that leaves through
    // the bottom (the reader scrolled back up past it) is reset while out of
    // sight, so it rises in again on the way down; blocks that leave through
    // the top stay shown, so scrolling up never makes content blink.
    const show = new IntersectionObserver((entries) => {
      entries.forEach((e) => { if (e.isIntersecting) e.target.classList.add("in"); });
    }, { rootMargin: "0px 0px -12% 0px", threshold: 0.01 });
    const hide = new IntersectionObserver((entries) => {
      entries.forEach((e) => {
        if (!e.isIntersecting && e.boundingClientRect.top > 0) e.target.classList.remove("in");
      });
    }, { threshold: 0 });
    items.forEach((el) => { show.observe(el); hide.observe(el); });
  }

  /* 2. hero: over the first ~60 % of a screen of scrolling the full-bleed video
     shrinks a little into a rounded card and the title drifts up and fades */
  const hero = document.querySelector(".hero");
  if (hero && !reduce) {
    let queued = false;
    const update = () => {
      queued = false;
      const h = hero.offsetHeight || window.innerHeight;
      const p = Math.min(Math.max(window.scrollY / (0.6 * h), 0), 1);
      hero.style.setProperty("--p", p.toFixed(4));
    };
    window.addEventListener("scroll", () => { if (!queued) { queued = true; requestAnimationFrame(update); } }, { passive: true });
    window.addEventListener("resize", update);
    update();
  }

  /* nav: transparent while it sits over the dark hero, solid once past it */
  const nav = document.querySelector(".nav");
  if (nav && hero) {
    let q2 = false;
    const navUpdate = () => { q2 = false; nav.classList.toggle("over", window.scrollY < hero.offsetHeight - nav.offsetHeight); };
    window.addEventListener("scroll", () => { if (!q2) { q2 = true; requestAnimationFrame(navUpdate); } }, { passive: true });
    window.addEventListener("resize", navUpdate);
    navUpdate();
  }

  /* 3. method: the figure is pinned; the step nearest the middle of the screen
     lights its half of the figure and dims the other */
  const fig = document.querySelector(".fig-stages");
  const steps = Array.from(document.querySelectorAll(".method-pin .step"));
  if (fig && steps.length && "IntersectionObserver" in window) {
    const seen = new Map();
    const so = new IntersectionObserver((entries) => {
      entries.forEach((e) => seen.set(e.target, e.isIntersecting ? e.intersectionRatio : 0));
      let best = null, r = 0;
      seen.forEach((v, el) => { if (v > r) { r = v; best = el; } });
      const n = best ? best.dataset.step : "";
      fig.dataset.stage = n;
      steps.forEach((s) => s.classList.toggle("is-on", s.dataset.step === n));
    }, { rootMargin: "-35% 0px -35% 0px", threshold: [0, 0.01, 0.2, 0.5, 1] });
    steps.forEach((s) => so.observe(s));
  }
})();
