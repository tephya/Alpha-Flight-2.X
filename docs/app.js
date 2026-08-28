const header = document.querySelector("[data-header]");

const updateHeader = () => {
  header?.classList.toggle("is-scrolled", window.scrollY > 24);
};

updateHeader();
window.addEventListener("scroll", updateHeader, { passive: true });

const revealObserver = new IntersectionObserver(
  (entries) => {
    entries.forEach((entry) => {
      if (entry.isIntersecting) {
        entry.target.classList.add("is-visible");
        revealObserver.unobserve(entry.target);
      }
    });
  },
  { threshold: 0.14 }
);

document.querySelectorAll(".reveal").forEach((element) => revealObserver.observe(element));

const canTilt = window.matchMedia("(hover: hover) and (pointer: fine)");

if (canTilt.matches) {
  document.querySelectorAll("[data-tilt]").forEach((card) => {
    card.addEventListener("pointermove", (event) => {
      const bounds = card.getBoundingClientRect();
      const px = (event.clientX - bounds.left) / bounds.width;
      const py = (event.clientY - bounds.top) / bounds.height;
      const rotateY = (px - 0.5) * 7;
      const rotateX = (0.5 - py) * 7;

      card.style.setProperty("--rx", `${rotateX.toFixed(2)}deg`);
      card.style.setProperty("--ry", `${rotateY.toFixed(2)}deg`);
      card.style.setProperty("--mx", `${(px * 100).toFixed(1)}%`);
      card.style.setProperty("--my", `${(py * 100).toFixed(1)}%`);
    });

    card.addEventListener("pointerleave", () => {
      card.style.setProperty("--rx", "0deg");
      card.style.setProperty("--ry", "0deg");
      card.style.setProperty("--mx", "50%");
      card.style.setProperty("--my", "50%");
    });
  });
}

const dialog = document.querySelector("[data-lightbox-dialog]");
const stage = dialog?.querySelector("[data-lightbox-stage]");
const fullImage = dialog?.querySelector("[data-lightbox-image]");
const title = dialog?.querySelector("[data-lightbox-title]");
const caption = dialog?.querySelector("[data-lightbox-caption]");
const counter = dialog?.querySelector("[data-lightbox-counter]");
const lightboxItems = [...document.querySelectorAll("[data-lightbox]")];

let activeIndex = 0;
let zoom = 1;
let panX = 0;
let panY = 0;
let pointerStart = null;

const applyImageTransform = () => {
  fullImage?.style.setProperty("--zoom", zoom.toFixed(3));
  fullImage?.style.setProperty("--pan-x", `${panX}px`);
  fullImage?.style.setProperty("--pan-y", `${panY}px`);
  stage?.classList.toggle("is-zoomed", zoom > 1.01);
};

const resetTransform = () => {
  zoom = 1;
  panX = 0;
  panY = 0;
  applyImageTransform();
};

const renderLightboxItem = () => {
  const item = lightboxItems[activeIndex];
  if (!item || !fullImage || !title || !caption || !counter) return;

  fullImage.src = item.dataset.full;
  fullImage.alt = item.dataset.title;
  title.textContent = item.dataset.title;
  caption.textContent = item.dataset.caption;
  counter.textContent = `${String(activeIndex + 1).padStart(2, "0")} / ${String(
    lightboxItems.length
  ).padStart(2, "0")}`;
  resetTransform();
};

const openLightbox = (index) => {
  if (!dialog) return;
  activeIndex = index;
  renderLightboxItem();
  dialog.showModal();
  document.body.style.overflow = "hidden";
};

const closeLightbox = () => {
  dialog?.close();
  document.body.style.overflow = "";
  resetTransform();
};

const moveLightbox = (direction) => {
  activeIndex = (activeIndex + direction + lightboxItems.length) % lightboxItems.length;
  renderLightboxItem();
};

lightboxItems.forEach((item, index) => {
  item.addEventListener("click", () => openLightbox(index));
});

dialog?.querySelector("[data-lightbox-close]")?.addEventListener("click", closeLightbox);
dialog?.querySelector("[data-lightbox-prev]")?.addEventListener("click", () => moveLightbox(-1));
dialog?.querySelector("[data-lightbox-next]")?.addEventListener("click", () => moveLightbox(1));

dialog?.addEventListener("click", (event) => {
  if (event.target === dialog) closeLightbox();
});

dialog?.addEventListener("close", () => {
  document.body.style.overflow = "";
});

document.addEventListener("keydown", (event) => {
  if (!dialog?.open) return;

  if (event.key === "Escape") closeLightbox();
  if (event.key === "ArrowLeft") moveLightbox(-1);
  if (event.key === "ArrowRight") moveLightbox(1);
});

stage?.addEventListener(
  "wheel",
  (event) => {
    event.preventDefault();
    zoom = Math.min(4, Math.max(1, zoom + (event.deltaY < 0 ? 0.18 : -0.18)));
    if (zoom === 1) {
      panX = 0;
      panY = 0;
    }
    applyImageTransform();
  },
  { passive: false }
);

stage?.addEventListener("pointerdown", (event) => {
  if (zoom <= 1) return;
  pointerStart = { x: event.clientX - panX, y: event.clientY - panY };
  stage.setPointerCapture(event.pointerId);
  stage.classList.add("is-dragging");
});

stage?.addEventListener("pointermove", (event) => {
  if (!pointerStart) return;
  panX = event.clientX - pointerStart.x;
  panY = event.clientY - pointerStart.y;
  applyImageTransform();
});

const endDrag = (event) => {
  if (!pointerStart) return;
  pointerStart = null;
  stage?.releasePointerCapture?.(event.pointerId);
  stage?.classList.remove("is-dragging");
};

stage?.addEventListener("pointerup", endDrag);
stage?.addEventListener("pointercancel", endDrag);

stage?.addEventListener("dblclick", () => {
  zoom = zoom > 1 ? 1 : 2;
  panX = 0;
  panY = 0;
  applyImageTransform();
});
