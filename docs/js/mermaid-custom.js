/* docs/js/mermaid-custom.js */
(function () {
  // Utilidad: correr en carga inicial y en cada navegación SPA de Material
  const onReady = (fn) => {
    if (window.document$ && typeof window.document$.subscribe === "function") {
      window.document$.subscribe(() => fn());
    } else if (document.readyState === "loading") {
      document.addEventListener("DOMContentLoaded", fn);
    } else {
      fn();
    }
  };

  // Esperar a que mermaid exista (por CDN o por el tema)
  function waitForMermaid(cb, tries = 30) {
    if (window.mermaid) return cb();
    if (tries <= 0) return console.error("Mermaid no cargó");
    setTimeout(() => waitForMermaid(cb, tries - 1), 50);
  }

  // Config global (una sola vez)
  function configureMermaid() {
    if (!window.mermaid || window.__mermaid_custom_configured) return;
    window.mermaid.initialize({
      startOnLoad: false,     // nosotros controlamos el render
      securityLevel: "loose",
      theme: "base",
      fontFamily: "inherit",
      // themeVariables: { ... } // activa si quieres
    });
    window.__mermaid_custom_configured = true;
  }

  // Renderizar SOLO bloques .mermaid-custom
  async function renderCustomMermaid() {
    if (!window.mermaid) return;
    const blocks = document.querySelectorAll(".mermaid-custom:not([data-mermaid-custom-processed])");
    const jobs = [];
    [...blocks].forEach((block, i) => {
      const code = block.textContent.trim();
      const id = `merm-${Date.now()}-${i}`;
      const container = document.createElement("div");
      container.className = "mermaid-custom-rendered";
      block.setAttribute("data-mermaid-custom-processed", "true");
      block.replaceWith(container);
      const job = window.mermaid.render(id, code)
        .then(({ svg }) => { container.innerHTML = svg; })
        .catch(e => { container.innerHTML = `<pre>Mermaid error:\n${String(e)}</pre>`; });
      jobs.push(job);
    });
    await Promise.all(jobs);
  }

  // --- Post-procesos específicos para mindmap ---
  function fixMindmapStrokes() {
    document.querySelectorAll(".mermaid-custom-rendered svg g[class*='mindmap'] path").forEach(p => {
      const stroke = p.getAttribute("stroke");
      const fill = p.getAttribute("fill");
      if (!stroke || stroke === "none" || stroke === "transparent" || fill === "none") {
        const edge = getComputedStyle(p.closest("svg")).getPropertyValue("--merm-edge") || "#555";
        p.setAttribute("stroke", edge);
        p.setAttribute("stroke-width", "1.8");
        p.setAttribute("fill", "none");
        p.style.opacity = "1";
      }
    });

    // marcadores de flecha (si los hay)
    document.querySelectorAll(".mermaid-custom-rendered marker path, .mermaid-custom-rendered marker polygon")
      .forEach(m => {
        const edge = getComputedStyle(m.closest("svg")).getPropertyValue("--merm-edge") || "#555";
        m.setAttribute("stroke", edge);
        m.setAttribute("fill", edge);
        m.style.opacity = "1";
      });
  }

  // Traer nodos al frente (encima de las ramas)
  function bringMindmapNodesToFront() {
    document.querySelectorAll(".mermaid-custom-rendered svg g[class*='mindmap']").forEach(root => {
      const nodes = root.querySelectorAll(".node");
      nodes.forEach(node => root.appendChild(node)); // mover al final del <g> => pinta encima
    });
  }

  // Pipeline de render + fixes, en cada navegación/carga
  onReady(async () => {
    waitForMermaid(async () => {
      configureMermaid();
      await renderCustomMermaid();
      fixMindmapStrokes();
      bringMindmapNodesToFront();
    });
  });
})();
