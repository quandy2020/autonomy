/**
 * BicMap bridge for benchmark dashboard.
 * SDK built from local source: third_party/bic-map-src → static/vendor/bic-map/
 * Offline blank MapLibre style (no tile / glyph / CDN requests).
 */
(() => {
  const cfg = window.__BICMAP__ || {};
  const assets = window.__BICMAP_ASSETS__ || {};
  const ORIGIN = cfg.origin_lnglat || [116.3912, 39.9073];

  const DISPLAY_DEFS = [
    {
      id: "grid",
      label: "Grid",
      type: "grid",
      topic: "map://grid",
      color: "#5a6570",
      on: true,
      topics: [
        { topic: "map://grid", type: "grid" },
        { topic: "map://grid/0.5m", type: "grid" },
        { topic: "/map", type: "grid" },
        { topic: "/occupancy_grid", type: "grid" },
      ],
      channels: [
        { id: "", label: "—", type: "*" },
        { id: "1m", label: "1 m", type: "grid" },
        { id: "0.5m", label: "0.5 m", type: "grid" },
      ],
    },
    {
      id: "robot",
      label: "Robot",
      type: "pose",
      topic: "/robot",
      color: "#4cc98c",
      on: true,
      topics: [
        { topic: "/robot", type: "pose" },
        { topic: "/odom", type: "pose" },
        { topic: "/localization/pose", type: "pose" },
        { topic: "/tf/base_link", type: "pose" },
      ],
      channels: [
        { id: "", label: "—", type: "*" },
        { id: "pose", label: "pose", type: "pose" },
        { id: "pose.x", label: "pose.x", type: "pose" },
        { id: "pose.y", label: "pose.y", type: "pose" },
        { id: "yaw", label: "yaw", type: "pose" },
      ],
    },
    {
      id: "path",
      label: "Path",
      type: "path",
      topic: "/plan",
      color: "#c45c26",
      on: true,
      topics: [
        { topic: "/plan", type: "path" },
        { topic: "/global_plan", type: "path" },
        { topic: "/local_plan", type: "path" },
        { topic: "/trajectory", type: "path" },
        { topic: "/path", type: "path" },
      ],
      channels: [
        { id: "", label: "—", type: "*" },
        { id: "points", label: "points", type: "path" },
        { id: "poses", label: "poses", type: "path" },
      ],
    },
    {
      id: "start",
      label: "Start",
      type: "pose",
      topic: "/start",
      color: "#6ea8fe",
      on: true,
      topics: [
        { topic: "/start", type: "pose" },
        { topic: "/initialpose", type: "pose" },
        { topic: "/amcl_pose", type: "pose" },
      ],
      channels: [
        { id: "", label: "—", type: "*" },
        { id: "pose", label: "pose", type: "pose" },
      ],
    },
    {
      id: "goal",
      label: "Goal Pose",
      type: "pose",
      topic: "/goal_pose",
      color: "#e8a05c",
      on: true,
      topics: [
        { topic: "/goal_pose", type: "pose" },
        { topic: "/navigate_to_pose/goal", type: "pose" },
        { topic: "/move_base_simple/goal", type: "pose" },
      ],
      channels: [
        { id: "", label: "—", type: "*" },
        { id: "pose", label: "pose", type: "pose" },
        { id: "yaw", label: "yaw", type: "pose" },
      ],
    },
    {
      id: "dock",
      label: "Dock",
      type: "pose",
      topic: "/dock",
      color: "#f0b429",
      on: true,
      topics: [
        { topic: "/dock", type: "pose" },
        { topic: "/charge/station", type: "pose" },
        { topic: "/docking/goal", type: "pose" },
      ],
      channels: [
        { id: "", label: "—", type: "*" },
        { id: "pose", label: "pose", type: "pose" },
      ],
    },
    {
      id: "waypoints",
      label: "Waypoints",
      type: "path",
      topic: "/waypoints",
      color: "#b794f4",
      on: true,
      topics: [
        { topic: "/waypoints", type: "path" },
        { topic: "/patrol/path", type: "path" },
        { topic: "/route", type: "path" },
      ],
      channels: [
        { id: "", label: "—", type: "*" },
        { id: "points", label: "points", type: "path" },
      ],
    },
    {
      id: "ghost",
      label: "Pose Tool",
      type: "tool",
      topic: "tool://goal_pose",
      color: "#7ddea8",
      on: true,
      topics: [
        { topic: "tool://goal_pose", type: "tool" },
        { topic: "tool://publish_point", type: "tool" },
      ],
      channels: [{ id: "", label: "—", type: "*" }],
    },
  ];

  function defaultDisplayCfg() {
    return Object.fromEntries(
      DISPLAY_DEFS.map((d) => [
        d.id,
        { on: d.on !== false, topic: d.topic, color: d.color, channel: "" },
      ]),
    );
  }

  function normalizeTopicEntry(t, fallbackType) {
    if (typeof t === "string") return { topic: t, type: fallbackType || "*" };
    return {
      topic: t.topic || String(t),
      type: t.type || fallbackType || "*",
    };
  }

  function topicTypeFor(def, topic) {
    const entries = (def.topics || []).map((t) => normalizeTopicEntry(t, def.type));
    return entries.find((e) => e.topic === topic)?.type || def.type || "*";
  }

  function channelsFor(def, topic) {
    const tType = topicTypeFor(def, topic);
    return (def.channels || [{ id: "", label: "—", type: "*" }]).filter(
      (ch) => ch.type === "*" || ch.type === tType || tType === "*",
    );
  }

  const state = {
    map: null,
    ready: false,
    robotCtrl: null,
    lineCtrl: null,
    poiCtrl: null,
    goalCtrl: null,
    sceneId: cfg.default_scene || "navigate",
    lastTrack: null,
    mode: "navigate", // navigate | teleop
    viewMode: "2d", // 2d | 3d
    pendingGoal: null,
    poseDraft: null, // { x, y, yaw, screenAngleDeg, dragging }
    poseMoved: false,
    gridReady: false,
    cloudCtrl: null,
    resizeObs: null,
    needsRefit: true,
    displaysOpen: false,
    displaySlots: [], // ordered DISPLAY_DEFS ids that were added
    imagesOpen: false,
    imageLayout: "stack",
    imageSlots: [], // [{ id, topic, channel, kind }]
    imageFrames: {}, // id → { src?, ts?, w?, h? }
    imageTimer: null,
    imageSeq: 0,
    displayCfg: defaultDisplayCfg(),
    teleopActive: false,
    teleopVx: 0,
    teleopWz: 0,
    teleopTimer: null,
    teleopStarted: false,
  };

  const MAX_V = 0.45;
  const MAX_W = 0.9;
  const TELEOP_HZ = 8;
  const DISPLAYS_KEY = "benchmark.map.displays";
  const IMAGES_KEY = "benchmark.map.images";
  const VIEW_KEY = "benchmark.map.view.v1";
  const GRID_SRC = "bm-grid-src";
  const GRID_LAYER_MINOR = "bm-grid-minor";
  const GRID_LAYER_MAJOR = "bm-grid-major";
  const GRID_LAYER_AXES = "bm-grid-axes";
  const GRID_LAYER_GROUND = "bm-grid-ground";
  const DEFAULT_DISPLAY_SLOTS = ["grid", "robot", "path", "start", "goal", "ghost"];

  function imageTopicCatalog() {
    const list = Array.isArray(cfg.image_topics) ? cfg.image_topics : [];
    return list.length
      ? list
      : [
          "/camera/front/rgb",
          "/camera/front/depth",
          "/camera/front/compressed",
          "/camera/rear/rgb",
          "/camera/rear/depth",
          "/camera/top/rgb",
          "/camera/left/rgb",
          "/camera/right/rgb",
          "/perception/image",
        ];
  }

  function imageChannelCatalog() {
    const list = Array.isArray(cfg.image_channels) ? cfg.image_channels : [];
    const fallback = [
      { id: "rgb", label: "rgb", type: "rgb" },
      { id: "rgb8", label: "rgb8", type: "rgb" },
      { id: "bgr8", label: "bgr8", type: "rgb" },
      { id: "mono8", label: "mono8", type: "mono" },
      { id: "depth", label: "depth", type: "depth" },
      { id: "depth16", label: "depth16", type: "depth" },
      { id: "compressed", label: "compressed", type: "compressed" },
      { id: "raw", label: "raw", type: "mono" },
    ];
    return (list.length ? list : fallback).map((c) => ({
      id: c.id || c,
      label: c.label || c.id || c,
      type: c.type || imageTopicType(c.id || c) || "*",
    }));
  }

  function imageTopicType(topic) {
    const t = String(topic || "").toLowerCase();
    if (t.includes("depth")) return "depth";
    if (t.includes("compress")) return "compressed";
    if (t.includes("mono") || t.includes("gray") || t.endsWith("/raw")) return "mono";
    return "rgb";
  }

  function imageChannelsFor(topic) {
    const tType = imageTopicType(topic);
    const list = imageChannelCatalog().filter(
      (ch) => ch.type === "*" || ch.type === tType || tType === "*",
    );
    return list.length ? list : [{ id: "", label: "—", type: "*" }];
  }

  function kindFromChannel(channel, fallback = "rgb") {
    const c = String(channel || "").toLowerCase();
    if (c.includes("depth")) return "depth";
    if (c.includes("mono") || c === "raw") return "mono";
    if (c.includes("compress")) return "rgb";
    return fallback === "depth" ? "depth" : "rgb";
  }

  function shortTopicLabel(topic) {
    const parts = String(topic || "").split("/").filter(Boolean);
    return parts.slice(-2).join("/") || topic || "image";
  }

  function imageSlotById(id) {
    return state.imageSlots.find((s) => s.id === id) || null;
  }

  function isDisplayAdded(id) {
    return state.displaySlots.includes(id);
  }

  function isDisplayOn(id) {
    return isDisplayAdded(id) && state.displayCfg[id]?.on !== false;
  }

  function displayColor(id) {
    return state.displayCfg[id]?.color || DISPLAY_DEFS.find((d) => d.id === id)?.color || "#4cc98c";
  }

  function displayTopic(id) {
    return state.displayCfg[id]?.topic || DISPLAY_DEFS.find((d) => d.id === id)?.topic || id;
  }

  function displayChannel(id) {
    const ch = state.displayCfg[id]?.channel;
    return typeof ch === "string" ? ch : "";
  }

  function displayDefById(id) {
    return DISPLAY_DEFS.find((d) => d.id === id) || null;
  }

  function imageTopic(id) {
    return imageSlotById(id)?.topic || id;
  }

  function imageChannel(id) {
    return imageSlotById(id)?.channel || "";
  }

  function imageKind(id) {
    const slot = imageSlotById(id);
    return slot?.kind || kindFromChannel(slot?.channel);
  }

  function hydrateDisplayCfg(parsed) {
    for (const d of DISPLAY_DEFS) {
      const cur = state.displayCfg[d.id] || {
        on: true,
        topic: d.topic,
        color: d.color,
        channel: "",
      };
      const v = parsed?.[d.id];
      if (typeof v === "boolean") {
        cur.on = v;
      } else if (v && typeof v === "object") {
        if (typeof v.on === "boolean") cur.on = v.on;
        if (typeof v.topic === "string" && v.topic) cur.topic = v.topic;
        if (typeof v.channel === "string") cur.channel = v.channel;
        if (typeof v.color === "string" && /^#?[0-9a-fA-F]{6}$/.test(v.color)) {
          cur.color = v.color.startsWith("#") ? v.color : `#${v.color}`;
        }
      }
      const allowed = channelsFor(d, cur.topic).map((c) => c.id);
      if (!allowed.includes(cur.channel)) cur.channel = "";
      state.displayCfg[d.id] = cur;
    }
  }

  function migrateLegacyDisplaySlots(parsed) {
    const slots = [];
    for (const d of DISPLAY_DEFS) {
      const v = parsed[d.id];
      if (v === false) continue;
      if (v && typeof v === "object" && v.on === false) continue;
      // Legacy: all defs were listed; only migrate those that were on.
      if (v === true || (v && typeof v === "object" && v.on !== false) || v == null) {
        if (v === true || (v && typeof v === "object")) slots.push(d.id);
      }
    }
    // If prefs only had __open and no per-id entries, treat as empty on-demand list.
    const hasAny = DISPLAY_DEFS.some((d) => parsed[d.id] !== undefined);
    return hasAny ? slots : [];
  }

  function loadDisplayPrefs() {
    state.displaySlots = [];
    try {
      const raw = localStorage.getItem(DISPLAYS_KEY);
      if (!raw) return;
      const parsed = JSON.parse(raw);
      if (!parsed || typeof parsed !== "object") return;
      hydrateDisplayCfg(parsed);
      if (Array.isArray(parsed.__slots)) {
        state.displaySlots = parsed.__slots.filter((id) => displayDefById(id));
      } else {
        state.displaySlots = migrateLegacyDisplaySlots(parsed);
      }
      if (typeof parsed.__open === "boolean") state.displaysOpen = parsed.__open;
    } catch (_) {}
  }

  function ensureDefaultDisplaySlots() {
    if (!state.displaySlots.length) {
      state.displaySlots = [...DEFAULT_DISPLAY_SLOTS];
      for (const id of state.displaySlots) {
        if (state.displayCfg[id]) state.displayCfg[id].on = true;
      }
      return;
    }
    if (!state.displaySlots.includes("grid")) {
      state.displaySlots.unshift("grid");
      if (state.displayCfg.grid) state.displayCfg.grid.on = true;
    }
  }

  function loadViewPrefs() {
    try {
      const v = localStorage.getItem(VIEW_KEY);
      if (v === "3d" || v === "2d") state.viewMode = v;
    } catch (_) {}
  }

  function saveViewPrefs() {
    try {
      localStorage.setItem(VIEW_KEY, state.viewMode);
    } catch (_) {}
  }

  function saveDisplayPrefs() {
    try {
      localStorage.setItem(
        DISPLAYS_KEY,
        JSON.stringify({
          ...state.displayCfg,
          __slots: state.displaySlots,
          __open: state.displaysOpen,
        }),
      );
    } catch (_) {}
  }

  function normalizeImageSlot(raw, idx = 0) {
    if (!raw || typeof raw !== "object") return null;
    const topic = typeof raw.topic === "string" && raw.topic ? raw.topic : "";
    if (!topic) return null;
    const allowed = imageChannelsFor(topic).map((c) => c.id);
    let channel = typeof raw.channel === "string" ? raw.channel : "";
    if (!allowed.includes(channel)) channel = allowed[0] || "";
    const id =
      typeof raw.id === "string" && raw.id
        ? raw.id
        : `img_${Date.now().toString(36)}_${idx}`;
    return {
      id,
      topic,
      channel,
      kind: kindFromChannel(channel, raw.kind || imageTopicType(topic)),
    };
  }

  function migrateLegacyImagePrefs(parsed) {
    const slots = [];
    for (const [key, v] of Object.entries(parsed)) {
      if (key.startsWith("__")) continue;
      if (!v || typeof v !== "object") continue;
      if (v.on === false) continue;
      const slot = normalizeImageSlot({ id: key, ...v }, slots.length);
      if (slot) slots.push(slot);
    }
    return slots;
  }

  function initImageState() {
    state.imageSlots = [];
    state.imageSeq = 0;
    // Optional seed from server config (only if user has no prefs yet).
    const seed =
      Array.isArray(cfg.images) && cfg.images.length
        ? cfg.images
            .filter((d) => d && d.on !== false)
            .map((d, i) => normalizeImageSlot(d, i))
            .filter(Boolean)
        : [];
    try {
      const raw = localStorage.getItem(IMAGES_KEY);
      if (!raw) {
        state.imageSlots = seed;
        return;
      }
      const parsed = JSON.parse(raw);
      if (!parsed || typeof parsed !== "object") {
        state.imageSlots = seed;
        return;
      }
      if (Array.isArray(parsed.__slots)) {
        state.imageSlots = parsed.__slots
          .map((s, i) => normalizeImageSlot(s, i))
          .filter(Boolean);
      } else {
        state.imageSlots = migrateLegacyImagePrefs(parsed);
      }
      if (typeof parsed.__open === "boolean") state.imagesOpen = parsed.__open;
      if (parsed.__layout === "grid" || parsed.__layout === "stack") {
        state.imageLayout = parsed.__layout;
      }
      if (typeof parsed.__seq === "number" && parsed.__seq >= 0) {
        state.imageSeq = parsed.__seq;
      } else {
        state.imageSeq = state.imageSlots.length;
      }
    } catch (_) {
      state.imageSlots = seed;
    }
  }

  function saveImagePrefs() {
    try {
      localStorage.setItem(
        IMAGES_KEY,
        JSON.stringify({
          __slots: state.imageSlots,
          __open: state.imagesOpen,
          __layout: state.imageLayout,
          __seq: state.imageSeq,
        }),
      );
    } catch (_) {}
  }

  function setStatus(msg, ok) {
    const el = document.getElementById("map-status");
    if (!el) return;
    el.textContent = msg;
    el.classList.toggle("ok", !!ok);
    el.classList.toggle("err", ok === false);
  }

  /** Local meters (sim frame) → lng/lat around ORIGIN. */
  function metersToLngLat(x, y) {
    const mPerDegLat = 111320;
    const mPerDegLng = 111320 * Math.cos((ORIGIN[1] * Math.PI) / 180);
    return [ORIGIN[0] + x / mPerDegLng, ORIGIN[1] + y / mPerDegLat];
  }

  function lngLatToMeters(lng, lat) {
    const mPerDegLat = 111320;
    const mPerDegLng = 111320 * Math.cos((ORIGIN[1] * Math.PI) / 180);
    return {
      x: (lng - ORIGIN[0]) * mPerDegLng,
      y: (lat - ORIGIN[1]) * mPerDegLat,
    };
  }

  function yawToDeg(yaw) {
    // sim yaw: 0 = +x east-ish; BicMap rotation typically degrees CW/CCW from north
    return ((90 - (yaw * 180) / Math.PI) % 360 + 360) % 360;
  }

  function rpcTarget() {
    const named = document.getElementById("named-target");
    const ota = document.getElementById("ota-target");
    return (named?.value || ota?.value || "localhost:50051").trim();
  }

  function setInteractHint(text) {
    const el = document.getElementById("map-interact-hint");
    if (el) el.textContent = text;
  }

  function pathToLngLat(path) {
    return (path || []).map((p) => metersToLngLat(p.x, p.y));
  }

  function containerVisible() {
    const el = document.getElementById("bicmap-container");
    if (!el) return false;
    const pane = el.closest(".tab-pane");
    if (pane?.hidden || pane?.classList.contains("active") === false) return false;
    const r = el.getBoundingClientRect();
    return r.width > 8 && r.height > 8;
  }

  function resizeMap() {
    if (!state.map) return false;
    try {
      state.map.resize();
      return true;
    } catch (_) {
      return false;
    }
  }

  /** Resize canvas to fill stage, then fit track / origin to center. */
  function fitMapView({ force = false } = {}) {
    if (!state.ready || !state.map) return;
    if (!containerVisible()) {
      state.needsRefit = true;
      return;
    }
    resizeMap();
    const run = () => {
      resizeMap();
      if (state.lastTrack) {
        applyTrack(state.lastTrack, { skipFit: false });
      } else {
        try {
          state.map.jumpTo?.({
            center: ORIGIN,
            zoom: 17.4,
            pitch: state.viewMode === "3d" ? 58 : 0,
            bearing: state.viewMode === "3d" ? state.map.getBearing?.() || -20 : 0,
          });
        } catch (_) {}
      }
      state.needsRefit = false;
    };
    if (force) {
      requestAnimationFrame(() => requestAnimationFrame(run));
    } else {
      requestAnimationFrame(run);
    }
  }

  function bindMapResize() {
    const el = document.getElementById("bicmap-container");
    if (!el || state.resizeObs || typeof ResizeObserver === "undefined") return;
    let timer = 0;
    state.resizeObs = new ResizeObserver(() => {
      if (!state.ready || !state.map) return;
      if (!containerVisible()) {
        state.needsRefit = true;
        return;
      }
      window.clearTimeout(timer);
      timer = window.setTimeout(() => {
        resizeMap();
        if (state.needsRefit) fitMapView({ force: true });
      }, 60);
    });
    state.resizeObs.observe(el);
    const stage = document.querySelector(".map-stage");
    if (stage && stage !== el) state.resizeObs.observe(stage);
  }

  function gridStepMeters() {
    const topic = displayTopic("grid") || "";
    const ch = displayChannel("grid") || "";
    if (topic.includes("0.5") || ch === "0.5m") return 0.5;
    return 1;
  }

  function buildGridGeoJSON({
    xmin = -24,
    xmax = 24,
    ymin = -24,
    ymax = 24,
    step = 1,
    majorEvery = 5,
  } = {}) {
    const features = [];
    const pushLine = (x0, y0, x1, y1, kind) => {
      features.push({
        type: "Feature",
        properties: { kind },
        geometry: {
          type: "LineString",
          coordinates: [metersToLngLat(x0, y0), metersToLngLat(x1, y1)],
        },
      });
    };
    for (let x = xmin; x <= xmax + 1e-9; x = Math.round((x + step) * 1000) / 1000) {
      const major = Math.abs(((x % majorEvery) + majorEvery) % majorEvery) < 1e-6;
      pushLine(x, ymin, x, ymax, major ? "major" : "minor");
    }
    for (let y = ymin; y <= ymax + 1e-9; y = Math.round((y + step) * 1000) / 1000) {
      const major = Math.abs(((y % majorEvery) + majorEvery) % majorEvery) < 1e-6;
      pushLine(xmin, y, xmax, y, major ? "major" : "minor");
    }
    pushLine(xmin, 0, xmax, 0, "axis-x");
    pushLine(0, ymin, 0, ymax, "axis-y");
    features.push({
      type: "Feature",
      properties: { kind: "ground" },
      geometry: {
        type: "Polygon",
        coordinates: [
          [
            metersToLngLat(xmin, ymin),
            metersToLngLat(xmax, ymin),
            metersToLngLat(xmax, ymax),
            metersToLngLat(xmin, ymax),
            metersToLngLat(xmin, ymin),
          ],
        ],
      },
    });
    return { type: "FeatureCollection", features };
  }

  function ensureGridLayer() {
    const map = state.map;
    if (!map || !map.getStyle) return;
    const data = buildGridGeoJSON({ step: gridStepMeters() });
    if (!map.getSource(GRID_SRC)) {
      map.addSource(GRID_SRC, { type: "geojson", data });
      map.addLayer({
        id: GRID_LAYER_GROUND,
        type: "fill",
        source: GRID_SRC,
        filter: ["==", ["get", "kind"], "ground"],
        paint: {
          "fill-color": "#2a3036",
          "fill-opacity": 0.55,
        },
      });
      map.addLayer({
        id: GRID_LAYER_MINOR,
        type: "line",
        source: GRID_SRC,
        filter: ["==", ["get", "kind"], "minor"],
        paint: {
          "line-color": "#4a5560",
          "line-width": 0.6,
          "line-opacity": 0.45,
        },
      });
      map.addLayer({
        id: GRID_LAYER_MAJOR,
        type: "line",
        source: GRID_SRC,
        filter: ["==", ["get", "kind"], "major"],
        paint: {
          "line-color": "#7a8690",
          "line-width": 1.15,
          "line-opacity": 0.7,
        },
      });
      map.addLayer({
        id: GRID_LAYER_AXES,
        type: "line",
        source: GRID_SRC,
        filter: [
          "any",
          ["==", ["get", "kind"], "axis-x"],
          ["==", ["get", "kind"], "axis-y"],
        ],
        paint: {
          "line-color": [
            "match",
            ["get", "kind"],
            "axis-x",
            "#c45c5c",
            "axis-y",
            "#5aaa7a",
            "#888",
          ],
          "line-width": 1.6,
          "line-opacity": 0.9,
        },
      });
    } else {
      map.getSource(GRID_SRC).setData(data);
    }
    state.gridReady = true;
    setGridVisible(isDisplayOn("grid"));
  }

  function setGridVisible(on) {
    const map = state.map;
    if (!map || !state.gridReady) return;
    const vis = on ? "visible" : "none";
    for (const id of [
      GRID_LAYER_GROUND,
      GRID_LAYER_MINOR,
      GRID_LAYER_MAJOR,
      GRID_LAYER_AXES,
    ]) {
      if (map.getLayer(id)) map.setLayoutProperty(id, "visibility", vis);
    }
  }

  function setViewMode(mode) {
    state.viewMode = mode === "3d" ? "3d" : "2d";
    saveViewPrefs();
    document.querySelectorAll("[data-map-view]").forEach((btn) => {
      btn.classList.toggle("active", btn.dataset.mapView === state.viewMode);
    });
    document.querySelector(".map-stage")?.classList.toggle("view-3d", state.viewMode === "3d");
    const map = state.map;
    if (!map) return;
    try {
      if (state.viewMode === "3d") {
        map.touchZoomRotate?.enableRotation?.();
        map.easeTo?.({ pitch: 58, bearing: map.getBearing?.() || -20, duration: 450 });
      } else {
        map.easeTo?.({ pitch: 0, bearing: 0, duration: 350 });
        map.touchZoomRotate?.disableRotation?.();
      }
    } catch (_) {}
    syncMapInteractionForMode();
    if (state.lastTrack) applyTrack(state.lastTrack, { skipFit: true });
  }

  function clearCloud() {
    try {
      state.cloudCtrl?.remove?.();
      state.cloudCtrl?.hide?.();
    } catch (_) {}
    state.cloudCtrl = null;
  }

  function updatePathCloud(track) {
    clearCloud();
    if (state.viewMode !== "3d" || !isDisplayOn("path") || !state.map) return;
    const path = track?.path || [];
    if (path.length < 2 || typeof bicMap.createPointCloud !== "function") return;
    const n = Math.max(path.length - 1, 1);
    const points = path.map((p, i) => {
      const ll = metersToLngLat(p.x, p.y);
      return { x: ll[0], y: ll[1], z: 0.15 + (i / n) * 1.6 };
    });
    try {
      state.cloudCtrl = bicMap.createPointCloud(state.map, points, {
        pointSize: 3.5,
        pointOpacity: 0.92,
        is3D: true,
        useColorMap: true,
        zRange: [0, 2],
        heightScale: 1,
      });
    } catch (err) {
      console.warn("createPointCloud", err);
    }
  }

  async function initMap() {
    const container = document.getElementById("bicmap-container");
    if (!container) return;
    if (typeof bicMap === "undefined") {
      setStatus("本地 BicMap 未加载（static/vendor/bic-map）", false);
      container.innerHTML =
        '<div class="map-fallback">无法加载本地 BicMap。<br/>请确认已打包 vendor：' +
        "<code>static/vendor/bic-map/bicMap.ext.min.js</code></div>";
      return;
    }
    try {
      setStatus("正在初始化 BicMap（本地源码构建）…");
      await bicMap.init();
      // Explicit blank style: no remote tiles, no glyph CDN.
      const offlineStyle = {
        version: 8,
        name: "BenchmarkOffline",
        sources: {},
        layers: [
          {
            id: "background",
            type: "background",
            paint: { "background-color": "#1f1b16" },
          },
        ],
      };
      state.map = bicMap.createMap({
        container: "bicmap-container",
        center: ORIGIN,
        zoom: 17.4,
        pitch: 0,
        bearing: 0,
        maxPitch: 75,
        antialias: true,
        backgroundColor: "#1f1b16",
        style: offlineStyle,
      });
      state.map.on("load", () => {
        try {
          bicMap.addZoomControl?.(state.map, "bottom-right");
        } catch (_) {
          /* optional */
        }
        state.ready = true;
        loadDisplayPrefs();
        ensureDefaultDisplaySlots();
        loadViewPrefs();
        ensureGridLayer();
        initImageState();
        bindPoseTool();
        bindDisplaysPanel();
        bindImagesPanel();
        bindMapResize();
        setDisplaysOpen(state.displaysOpen);
        setImagesOpen(state.imagesOpen);
        setMode(state.mode);
        setViewMode(state.viewMode);
        showSceneHint(state.sceneId);
        seedDemoPose();
        startImageDemoFeed();
        renderDisplaysList();
        saveDisplayPrefs();
        state.needsRefit = true;
        if (containerVisible()) fitMapView({ force: true });
      });
    } catch (err) {
      console.error(err);
      setStatus("BicMap 初始化失败: " + err, false);
    }
  }

  /** Keep map pan/rotate off while Goal Pose tool is active (left-drag = pose). */
  function syncMapInteractionForMode() {
    if (!state.map) return;
    try {
      if (state.mode === "navigate" && state.viewMode !== "3d") {
        state.map.dragPan?.disable?.();
        state.map.dragRotate?.disable?.();
      } else if (state.mode === "navigate" && state.viewMode === "3d") {
        // 3D: allow orbit; pose tool still uses left-drag when posing.
        if (!state.poseDraft?.dragging) {
          state.map.dragPan?.enable?.();
          state.map.dragRotate?.enable?.();
        }
      } else {
        state.map.dragPan?.enable?.();
        state.map.dragRotate?.enable?.();
      }
    } catch (_) {
      /* optional */
    }
  }

  /** Foxglove 2D Pose: press = position, drag = heading, release = publish. */
  function bindPoseTool() {
    if (!state.map || state.map.__bmPoseBound) return;
    state.map.__bmPoseBound = true;
    const canvas = state.map.getCanvas?.() || document.getElementById("bicmap-container");
    if (!canvas) return;

    const clientToLngLat = (clientX, clientY) => {
      const rect = canvas.getBoundingClientRect();
      if (typeof state.map.unproject !== "function") return null;
      const ll = state.map.unproject([clientX - rect.left, clientY - rect.top]);
      return ll ? { lng: ll.lng, lat: ll.lat } : null;
    };

    const updateDraftFromClient = (clientX, clientY) => {
      const draft = state.poseDraft;
      if (!draft?.dragging) return;
      const ll = clientToLngLat(clientX, clientY);
      if (!ll) return;
      const cur = lngLatToMeters(ll.lng, ll.lat);
      const dx = cur.x - draft.x;
      const dy = cur.y - draft.y;
      if (Math.hypot(dx, dy) > 0.08) {
        state.poseMoved = true;
        draft.yaw = Math.atan2(dy, dx);
        draft.screenAngleDeg = yawToScreenDeg(draft.yaw);
      }
      // Always show draft ghost while posing (ignore Displays toggle).
      showPoseGhost(draft.x, draft.y, draft.yaw, false);
      const deg = ((draft.yaw * 180) / Math.PI + 360) % 360;
      setStatus(
        `Goal Pose · (${draft.x.toFixed(2)}, ${draft.y.toFixed(2)}) · θ=${deg.toFixed(0)}°`,
        true,
      );
    };

    const onPointerDown = (ev) => {
      if (state.mode !== "navigate") return;
      if (ev.button != null && ev.button !== 0) return;
      if (ev.pointerType === "mouse" && ev.button !== 0) return;
      const ll = clientToLngLat(ev.clientX, ev.clientY);
      if (!ll) return;
      ev.preventDefault();
      ev.stopPropagation();
      try {
        canvas.setPointerCapture?.(ev.pointerId);
      } catch (_) {}
      const meters = lngLatToMeters(ll.lng, ll.lat);
      state.poseDraft = {
        x: meters.x,
        y: meters.y,
        yaw: state.pendingGoal?.yaw || defaultYawToward(meters.x, meters.y),
        screenAngleDeg: 0,
        dragging: true,
        pointerId: ev.pointerId,
      };
      state.poseDraft.screenAngleDeg = yawToScreenDeg(state.poseDraft.yaw);
      state.poseMoved = false;
      try {
        state.map.dragPan?.disable?.();
        state.map.dragRotate?.disable?.();
      } catch (_) {}
      document.querySelector(".map-stage")?.classList.add("posing");
      showPoseGhost(meters.x, meters.y, state.poseDraft.yaw, false);
      setInteractHint("拖动设置朝向 · Esc 取消");
      setStatus(`Goal Pose · (${meters.x.toFixed(2)}, ${meters.y.toFixed(2)})`, true);
    };

    const onPointerMove = (ev) => {
      if (!state.poseDraft?.dragging) return;
      if (
        state.poseDraft.pointerId != null &&
        ev.pointerId != null &&
        state.poseDraft.pointerId !== ev.pointerId
      ) {
        return;
      }
      ev.preventDefault();
      updateDraftFromClient(ev.clientX, ev.clientY);
    };

    const finishPose = (ev) => {
      const draft = state.poseDraft;
      if (!draft?.dragging) return;
      if (
        ev &&
        draft.pointerId != null &&
        ev.pointerId != null &&
        draft.pointerId !== ev.pointerId
      ) {
        return;
      }
      if (ev) {
        try {
          canvas.releasePointerCapture?.(ev.pointerId);
        } catch (_) {}
        if (ev.clientX != null) updateDraftFromClient(ev.clientX, ev.clientY);
      }
      draft.dragging = false;
      document.querySelector(".map-stage")?.classList.remove("posing");
      syncMapInteractionForMode();
      const yaw = state.poseMoved ? draft.yaw : defaultYawToward(draft.x, draft.y);
      draft.yaw = yaw;
      state.poseDraft = null;
      void sendNavigateGoal(draft.x, draft.y, yaw);
    };

    const onKey = (ev) => {
      if (ev.key !== "Escape") return;
      if (!state.poseDraft) return;
      cancelPoseDraft();
    };

    // Pointer on canvas (capture) — reliable drag; MapLibre mousedown races dragPan.
    canvas.addEventListener("pointerdown", onPointerDown, { capture: true });
    canvas.addEventListener("pointermove", onPointerMove, { capture: true });
    canvas.addEventListener("pointerup", finishPose, { capture: true });
    canvas.addEventListener("pointercancel", finishPose, { capture: true });
    window.addEventListener("keydown", onKey);
    state.map.on("move", () => {
      if (state.poseDraft) {
        showPoseGhost(state.poseDraft.x, state.poseDraft.y, state.poseDraft.yaw, false);
      } else if (state.pendingGoal && (isDisplayOn("ghost") || isDisplayOn("goal"))) {
        showPoseGhost(
          state.pendingGoal.x,
          state.pendingGoal.y,
          state.pendingGoal.yaw || 0,
          true,
        );
      }
    });
    syncMapInteractionForMode();
  }

  function defaultYawToward(x, y) {
    const pose = state.lastTrack?.pose;
    if (!pose) return 0;
    return Math.atan2(y - (pose.y || 0), x - (pose.x || 0));
  }

  function yawToScreenDeg(yaw) {
    // CSS 0° = +x screen right; map north is up → convert sim yaw (+x east) to screen.
    // In MapLibre screen: +x right, +y down; east is +lng ≈ right when north-up.
    return (-(yaw * 180) / Math.PI + 360) % 360;
  }

  function showPoseGhost(x, y, yaw, sticky = false) {
    const ghost = document.getElementById("map-pose-ghost");
    const label = document.getElementById("map-pose-label");
    const arrow = document.getElementById("map-pose-arrow");
    const stage = document.querySelector(".map-stage");
    if (!ghost || !stage || !state.map?.project) return;
    const ll = metersToLngLat(x, y);
    const pt = state.map.project(ll);
    const stageRect = stage.getBoundingClientRect();
    const canvas = state.map.getCanvas?.();
    const canvasRect = canvas?.getBoundingClientRect?.() || stageRect;
    const left = pt.x + (canvasRect.left - stageRect.left);
    const top = pt.y + (canvasRect.top - stageRect.top);
    const screenDeg = yawToScreenDeg(yaw);
    // Distinct palette: draft=green, committed goal=amber (ignore stale localStorage greens).
    const color = sticky ? "#e8a05c" : "#7ddea8";
    ghost.hidden = false;
    ghost.setAttribute("aria-hidden", "false");
    ghost.style.left = `${left}px`;
    ghost.style.top = `${top}px`;
    ghost.style.setProperty("--pose-color", color);
    ghost.classList.toggle("sticky", !!sticky);
    ghost.classList.toggle("draft", !sticky);
    if (arrow) arrow.style.transform = `rotate(${screenDeg}deg)`;
    if (label) {
      const deg = ((yaw * 180) / Math.PI + 360) % 360;
      const tag = sticky ? "GOAL" : "POSE";
      label.textContent = `${tag}  ${x.toFixed(2)}, ${y.toFixed(2)} · ${deg.toFixed(0)}°`;
    }
  }

  function hidePoseGhost() {
    const ghost = document.getElementById("map-pose-ghost");
    if (ghost) {
      ghost.hidden = true;
      ghost.setAttribute("aria-hidden", "true");
      ghost.classList.remove("sticky", "draft");
    }
  }

  function cancelPoseDraft({ silent = false } = {}) {
    const had = !!state.poseDraft;
    state.poseDraft = null;
    state.poseMoved = false;
    document.querySelector(".map-stage")?.classList.remove("posing");
    if (state.pendingGoal && (isDisplayOn("ghost") || isDisplayOn("goal"))) {
      showPoseGhost(
        state.pendingGoal.x,
        state.pendingGoal.y,
        state.pendingGoal.yaw || 0,
        true,
      );
    } else if (!state.pendingGoal) {
      hidePoseGhost();
    }
    syncMapInteractionForMode();
    if (had && !silent) {
      setInteractHint("按下定点 · 拖动定朝向 · 松开发送");
      setStatus("已取消 Goal Pose", true);
    }
  }

  function clearGoalMarker() {
    try {
      state.goalCtrl?.remove?.();
    } catch (_) {}
    state.goalCtrl = null;
    if (!state.poseDraft) hidePoseGhost();
  }

  function showGoalMarker(x, y, yaw = 0, _label = "goal") {
    if (!state.map) return;
    clearGoalMarker();
    state.pendingGoal = { x, y, yaw };
    // Use our Foxglove-style ghost only — BicMap POI bubbles look like a "navigate" CTA
    // and show lng/lat instead of map-frame meters.
    showPoseGhost(x, y, yaw, true);
  }

  function setDisplaysOpen(open) {
    state.displaysOpen = !!open;
    const panel = document.getElementById("map-displays");
    const fab = document.getElementById("btn-map-displays");
    const stage = document.querySelector(".map-stage");
    if (panel) panel.hidden = !state.displaysOpen;
    stage?.classList.toggle("displays-open", state.displaysOpen);
    if (fab) {
      fab.setAttribute("aria-expanded", state.displaysOpen ? "true" : "false");
      fab.classList.toggle("active", state.displaysOpen);
    }
    saveDisplayPrefs();
  }

  function topicOptionsHtml(def, selected) {
    const entries = (def.topics || [def.topic]).map((t) =>
      normalizeTopicEntry(t, def.type),
    );
    if (selected && !entries.some((e) => e.topic === selected)) {
      entries.unshift({ topic: selected, type: def.type || "*" });
    }
    return entries
      .map(
        (e) =>
          `<option value="${e.topic}" ${e.topic === selected ? "selected" : ""}>${e.topic}</option>`,
      )
      .join("");
  }

  function availableDisplayKinds() {
    return DISPLAY_DEFS.filter((d) => !isDisplayAdded(d.id));
  }

  function syncDisplayAddPicker() {
    const kindSel = document.getElementById("map-display-add-kind");
    const addBtn = document.getElementById("btn-map-display-add");
    if (!kindSel) return;
    const available = availableDisplayKinds();
    const prev = kindSel.value;
    const kindId = available.some((d) => d.id === prev)
      ? prev
      : available[0]?.id || "";
    kindSel.innerHTML = available.length
      ? available
          .map(
            (d) =>
              `<option value="${d.id}" ${d.id === kindId ? "selected" : ""}>${d.label} · ${d.topic}</option>`,
          )
          .join("")
      : '<option value="">全部已添加</option>';
    kindSel.disabled = !available.length;
    if (addBtn) addBtn.disabled = !available.length;
  }

  function addDisplaySlot(kindId) {
    const def = displayDefById(kindId);
    if (!def) {
      setStatus("请先选择 topic", false);
      return false;
    }
    if (isDisplayAdded(def.id)) {
      setStatus(`${def.label} 已添加`, false);
      return false;
    }
    const cfg = state.displayCfg[def.id];
    cfg.on = true;
    cfg.channel = "";
    state.displaySlots.push(def.id);
    saveDisplayPrefs();
    setStatus(`已添加 ${def.label}`, true);
    renderDisplaysList();
    refreshDisplays();
    return true;
  }

  function removeDisplaySlot(id) {
    const idx = state.displaySlots.indexOf(id);
    if (idx < 0) return;
    state.displaySlots.splice(idx, 1);
    if (state.displayCfg[id]) state.displayCfg[id].on = false;
    saveDisplayPrefs();
    renderDisplaysList();
    refreshDisplays();
  }

  function renderDisplaysList() {
    const host = document.getElementById("map-displays-list");
    if (!host) return;
    syncDisplayAddPicker();
    if (!state.displaySlots.length) {
      host.innerHTML =
        '<li class="map-displays-empty">选择 topic，点 Add 添加一层显示</li>';
      return;
    }
    host.innerHTML = state.displaySlots
      .map((id) => {
        const def = displayDefById(id);
        if (!def) return "";
        const on = isDisplayOn(id);
        const color = displayColor(id);
        const topic = displayTopic(id);
        return `<li class="map-display-row${on ? "" : " off"}" data-display-id="${id}" title="${def.label}">
          <label class="map-display-check">
            <input type="checkbox" ${on ? "checked" : ""} aria-label="显示 ${def.label}" />
          </label>
          <label class="map-display-color" title="${def.label} 颜色">
            <input type="color" value="${color}" aria-label="${def.label} 颜色" />
            <span class="map-display-swatch" style="background:${color}" aria-hidden="true"></span>
          </label>
          <select class="map-display-topic" aria-label="${def.label} topic" title="${def.label}">
            ${topicOptionsHtml(def, topic)}
          </select>
          <button type="button" class="map-display-remove" data-display-remove="${id}" title="移除" aria-label="移除 ${def.label}">×</button>
        </li>`;
      })
      .join("");
    host.querySelectorAll(".map-display-row").forEach((row) => {
      const id = row.dataset.displayId;
      const def = displayDefById(id);
      const cfg = state.displayCfg[id];
      const check = row.querySelector('input[type="checkbox"]');
      const colorIn = row.querySelector('input[type="color"]');
      const swatch = row.querySelector(".map-display-swatch");
      const topicSel = row.querySelector(".map-display-topic");
      check?.addEventListener("change", () => {
        cfg.on = !!check.checked;
        row.classList.toggle("off", !check.checked);
        saveDisplayPrefs();
        refreshDisplays();
      });
      colorIn?.addEventListener("input", () => {
        cfg.color = colorIn.value;
        if (swatch) swatch.style.background = cfg.color;
        saveDisplayPrefs();
        refreshDisplays();
      });
      topicSel?.addEventListener("change", () => {
        cfg.topic = topicSel.value;
        const allowed = channelsFor(def, cfg.topic).map((c) => c.id);
        if (!allowed.includes(cfg.channel)) cfg.channel = allowed[0] || "";
        saveDisplayPrefs();
        setStatus(`${def?.label || id} ← ${cfg.topic}`, true);
        refreshDisplays();
      });
    });
    host.querySelectorAll("[data-display-remove]").forEach((btn) => {
      btn.addEventListener("click", () => removeDisplaySlot(btn.dataset.displayRemove));
    });
  }

  function refreshDisplays() {
    if (state.gridReady) {
      ensureGridLayer();
      setGridVisible(isDisplayOn("grid"));
    }
    if (state.lastTrack) applyTrack(state.lastTrack, { skipFit: true });
    else {
      clearCloud();
      if (state.pendingGoal) {
        showGoalMarker(
          state.pendingGoal.x,
          state.pendingGoal.y,
          state.pendingGoal.yaw || 0,
        );
      } else if (!isDisplayOn("ghost")) {
        hidePoseGhost();
      }
    }
  }

  function bindDisplaysPanel() {
    renderDisplaysList();
    document.getElementById("btn-map-displays")?.addEventListener("click", () => {
      setDisplaysOpen(!state.displaysOpen);
    });
    document.getElementById("btn-map-displays-close")?.addEventListener("click", () => {
      setDisplaysOpen(false);
    });
    document.getElementById("btn-map-display-add")?.addEventListener("click", () => {
      const kind = document.getElementById("map-display-add-kind")?.value || "";
      addDisplaySlot(kind);
    });
  }

  function setImagesOpen(open) {
    state.imagesOpen = !!open;
    const panel = document.getElementById("map-images");
    const fab = document.getElementById("btn-map-images");
    const stage = document.querySelector(".map-stage");
    if (panel) panel.hidden = !state.imagesOpen;
    stage?.classList.toggle("images-open", state.imagesOpen);
    if (fab) {
      fab.setAttribute("aria-expanded", state.imagesOpen ? "true" : "false");
      fab.classList.toggle("active", state.imagesOpen);
    }
    saveImagePrefs();
    if (state.imagesOpen) paintAllImageTiles();
  }

  function setImageLayout(layout) {
    state.imageLayout = layout === "grid" ? "grid" : "stack";
    const body = document.getElementById("map-images-body");
    if (body) body.dataset.layout = state.imageLayout;
    document.querySelectorAll(".map-images-layout-btn").forEach((btn) => {
      btn.classList.toggle("active", btn.dataset.imageLayout === state.imageLayout);
    });
    saveImagePrefs();
  }

  function hashHue(id) {
    let h = 0;
    for (let i = 0; i < id.length; i++) h = (h * 31 + id.charCodeAt(i)) >>> 0;
    return h % 360;
  }

  function imageTopicOptionsHtml(selected) {
    const opts = [...imageTopicCatalog()];
    if (selected && !opts.includes(selected)) opts.unshift(selected);
    return opts
      .map(
        (t) =>
          `<option value="${t}" ${t === selected ? "selected" : ""}>${t}</option>`,
      )
      .join("");
  }

  function imageChannelOptionsHtml(topic, selected) {
    const channels = [...imageChannelsFor(topic)];
    const ids = channels.map((c) => c.id);
    if (selected && !ids.includes(selected)) {
      channels.unshift({ id: selected, label: selected, type: "*" });
    }
    return channels
      .map(
        (c) =>
          `<option value="${c.id}" ${c.id === selected ? "selected" : ""}>${c.label || c.id || "—"}</option>`,
      )
      .join("");
  }

  function syncImageAddPicker() {
    const topicSel = document.getElementById("map-image-add-topic");
    const channelSel = document.getElementById("map-image-add-channel");
    if (!topicSel || !channelSel) return;
    const prevTopic = topicSel.value;
    const topics = imageTopicCatalog();
    const topic = topics.includes(prevTopic) ? prevTopic : topics[0] || "";
    topicSel.innerHTML = imageTopicOptionsHtml(topic);
    const channels = imageChannelsFor(topicSel.value);
    const prevCh = channelSel.value;
    const channel = channels.some((c) => c.id === prevCh)
      ? prevCh
      : channels[0]?.id || "";
    channelSel.innerHTML = imageChannelOptionsHtml(topicSel.value, channel);
  }

  function addImageSlot(topic, channel) {
    const t = String(topic || "").trim();
    if (!t) {
      setStatus("请先选择 topic", false);
      return false;
    }
    const allowed = imageChannelsFor(t);
    let ch = String(channel || "").trim();
    if (!allowed.some((c) => c.id === ch)) ch = allowed[0]?.id || "";
    const dup = state.imageSlots.some((s) => s.topic === t && s.channel === ch);
    if (dup) {
      setStatus(`已添加 ${t} · ${ch || "—"}`, false);
      return false;
    }
    state.imageSeq += 1;
    const slot = {
      id: `img_${state.imageSeq}`,
      topic: t,
      channel: ch,
      kind: kindFromChannel(ch, imageTopicType(t)),
    };
    state.imageSlots.push(slot);
    saveImagePrefs();
    setStatus(`已添加 ${shortTopicLabel(t)} · ${ch || "—"}`, true);
    renderImagesPanel();
    return true;
  }

  function removeImageSlot(id) {
    const idx = state.imageSlots.findIndex((s) => s.id === id);
    if (idx < 0) return;
    state.imageSlots.splice(idx, 1);
    delete state.imageFrames[id];
    saveImagePrefs();
    renderImagesPanel();
  }

  /** Procedural placeholder frame (until live camera topics arrive). */
  function drawPlaceholderFrame(canvas, id, t) {
    if (!canvas) return;
    const w = canvas.width || 320;
    const h = canvas.height || 200;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    const topic = imageTopic(id);
    const channel = imageChannel(id);
    const kind = imageKind(id);
    const hue = hashHue(topic + channel);
    if (kind === "depth") {
      const g = ctx.createLinearGradient(0, 0, w, h);
      g.addColorStop(0, `hsl(${hue}, 35%, 18%)`);
      g.addColorStop(1, `hsl(${(hue + 40) % 360}, 55%, 42%)`);
      ctx.fillStyle = g;
      ctx.fillRect(0, 0, w, h);
      for (let y = 0; y < h; y += 6) {
        const v = 20 + ((y + t * 30) % 60);
        ctx.fillStyle = `hsla(200, 70%, ${v}%, 0.35)`;
        ctx.fillRect(0, y, w, 4);
      }
    } else if (kind === "mono") {
      const g = ctx.createLinearGradient(0, 0, w, h);
      g.addColorStop(0, "#1a1f24");
      g.addColorStop(1, "#3a424a");
      ctx.fillStyle = g;
      ctx.fillRect(0, 0, w, h);
      for (let i = 0; i < 18; i++) {
        const x = ((i * 37 + t * 40) % w);
        ctx.fillStyle = `rgba(255,255,255,${0.04 + (i % 5) * 0.02})`;
        ctx.fillRect(x, 0, 3, h);
      }
    } else {
      const g = ctx.createLinearGradient(0, 0, w, h);
      g.addColorStop(0, `hsl(${hue}, 28%, 14%)`);
      g.addColorStop(0.55, `hsl(${(hue + 25) % 360}, 40%, 28%)`);
      g.addColorStop(1, `hsl(${(hue + 55) % 360}, 45%, 20%)`);
      ctx.fillStyle = g;
      ctx.fillRect(0, 0, w, h);
      const cx = w * (0.35 + 0.15 * Math.sin(t));
      const cy = h * (0.45 + 0.1 * Math.cos(t * 0.8));
      ctx.beginPath();
      ctx.arc(cx, cy, Math.min(w, h) * 0.12, 0, Math.PI * 2);
      ctx.fillStyle = `hsla(${hue}, 70%, 60%, 0.55)`;
      ctx.fill();
      ctx.strokeStyle = "rgba(255,255,255,0.25)";
      ctx.strokeRect(8, 8, w - 16, h - 16);
    }
    ctx.fillStyle = "rgba(255,255,255,0.62)";
    ctx.font = "10px IBM Plex Mono, monospace";
    ctx.fillText(topic, 10, h - 22);
    ctx.fillStyle = "rgba(255,255,255,0.42)";
    ctx.fillText(`ch:${channel}`, 10, h - 8);
  }

  function renderImagesPanel() {
    const body = document.getElementById("map-images-body");
    if (!body) return;
    body.dataset.layout = state.imageLayout;
    syncImageAddPicker();
    if (!state.imageSlots.length) {
      body.innerHTML =
        '<p class="map-images-empty">选择 topic 与 channel，点 Add 添加一路图像</p>';
      return;
    }
    body.innerHTML = state.imageSlots
      .map((s) => {
        const label = shortTopicLabel(s.topic);
        const ch = s.channel || "—";
        return `<article class="map-image-tile" data-image-id="${s.id}">
          <header class="map-image-tile-head">
            <div class="map-image-meta">
              <strong title="${s.topic}">${label}</strong>
              <span title="${s.topic}">${ch}</span>
            </div>
            <button type="button" class="map-image-remove" data-image-remove="${s.id}" title="移除" aria-label="移除 ${label}">×</button>
          </header>
          <div class="map-image-frame">
            <canvas width="320" height="200" data-image-canvas="${s.id}"></canvas>
            <span class="map-image-badge" data-image-badge="${s.id}">idle</span>
          </div>
        </article>`;
      })
      .join("");
    body.querySelectorAll("[data-image-remove]").forEach((btn) => {
      btn.addEventListener("click", () => removeImageSlot(btn.dataset.imageRemove));
    });
    paintAllImageTiles();
  }

  function paintAllImageTiles() {
    const t = performance.now() / 1000;
    for (const slot of state.imageSlots) {
      const canvas = document.querySelector(`[data-image-canvas="${slot.id}"]`);
      const badge = document.querySelector(`[data-image-badge="${slot.id}"]`);
      const frame = state.imageFrames[slot.id];
      if (frame?.src && canvas) {
        const img = new Image();
        img.onload = () => {
          const ctx = canvas.getContext("2d");
          if (!ctx) return;
          ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
        };
        img.src = frame.src;
        if (badge) badge.textContent = frame.ts || "live";
      } else {
        drawPlaceholderFrame(canvas, slot.id, t + hashHue(slot.id) / 60);
        if (badge) badge.textContent = `${slot.channel || "—"} · demo`;
      }
    }
  }

  function startImageDemoFeed() {
    if (state.imageTimer) return;
    state.imageTimer = setInterval(() => {
      if (!state.imagesOpen) return;
      paintAllImageTiles();
    }, 400);
  }

  function updateImage(id, payload) {
    if (!id) return;
    if (!payload) {
      delete state.imageFrames[id];
    } else if (typeof payload === "string") {
      state.imageFrames[id] = { src: payload, ts: "live" };
    } else {
      state.imageFrames[id] = {
        src: payload.src || payload.url || "",
        ts: payload.ts || payload.stamp || "live",
      };
    }
    if (state.imagesOpen) paintAllImageTiles();
  }

  function bindImagesPanel() {
    renderImagesPanel();
    setImageLayout(state.imageLayout);
    document.getElementById("btn-map-images")?.addEventListener("click", () => {
      setImagesOpen(!state.imagesOpen);
    });
    document.getElementById("btn-map-images-close")?.addEventListener("click", () => {
      setImagesOpen(false);
    });
    document.querySelectorAll(".map-images-layout-btn").forEach((btn) => {
      btn.addEventListener("click", () => setImageLayout(btn.dataset.imageLayout));
    });
    document.getElementById("map-image-add-topic")?.addEventListener("change", () => {
      syncImageAddPicker();
    });
    document.getElementById("btn-map-image-add")?.addEventListener("click", () => {
      const topic = document.getElementById("map-image-add-topic")?.value || "";
      const channel = document.getElementById("map-image-add-channel")?.value || "";
      addImageSlot(topic, channel);
    });
  }

  function emitMapGoal(x, y, yaw) {
    try {
      window.dispatchEvent(
        new CustomEvent("benchmark:map-goal", {
          detail: { x, y, yaw, target: rpcTarget() },
        }),
      );
    } catch (_) {
      /* ignore */
    }
  }

  function shortRpcError(err) {
    const s = String(err || "");
    if (/UNAVAILABLE|Connection refused|failed to connect/i.test(s)) {
      return "机器人未连接";
    }
    if (/DEADLINE_EXCEEDED|timeout/i.test(s)) return "超时";
    const m = s.match(/failed:\s*([A-Z_]+)/);
    if (m) return m[1];
    return s.length > 80 ? `${s.slice(0, 77)}…` : s || "未知错误";
  }

  async function sendNavigateGoal(x, y, yaw) {
    state.pendingGoal = { x, y, yaw };
    showGoalMarker(x, y, yaw, "goal_pose");
    emitMapGoal(x, y, yaw);
    const deg = ((yaw * 180) / Math.PI + 360) % 360;
    setStatus(`发送 goal_pose · (${x.toFixed(2)}, ${y.toFixed(2)}) · θ=${deg.toFixed(0)}°…`);
    setInteractHint("正在发布 Navigate…");
    try {
      const res = await fetch("/api/map/navigate", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ x, y, yaw, target: rpcTarget() }),
      });
      const data = await res.json().catch(() => ({}));
      if (data.published === false || data.stored) {
        // Offline / soft-queue: goal kept on map + synced into 命令 → Navigate.
        setStatus(
          `目标已记录 · (${x.toFixed(2)}, ${y.toFixed(2)}) · θ=${deg.toFixed(0)}° · ${shortRpcError(data.error || data.warning)}`,
          false,
        );
        setInteractHint("已写入命令 Navigate · 接通机器人后可再发");
        return;
      }
      if (!res.ok || data.ok === false) {
        setStatus(
          `目标已记录 · 发布失败 · ${shortRpcError(data.error || res.status)}`,
          false,
        );
        setInteractHint("已写入命令 Navigate · 按下可重设目标点");
        return;
      }
      setStatus(
        `已发布 goal_pose · (${x.toFixed(2)}, ${y.toFixed(2)}) · θ=${deg.toFixed(0)}°`,
        true,
      );
      setInteractHint("按下定点 · 拖动定朝向 · 松开发送");
    } catch (err) {
      setStatus(`目标已记录 · 发布失败 · ${shortRpcError(err)}`, false);
      setInteractHint("已写入命令 Navigate · 按下可重设目标点");
    }
  }

  async function sendTeleop(action, vx = 0, wz = 0) {
    try {
      const res = await fetch("/api/map/teleop", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          action,
          linear_x: vx,
          angular_z: wz,
          target: rpcTarget(),
        }),
      });
      const data = await res.json().catch(() => ({}));
      if (!res.ok || data.ok === false) {
        if (action !== "twist") {
          setStatus(`遥操 ${action} 失败 · ${data.error || res.status}`, false);
        }
        return false;
      }
      return true;
    } catch (err) {
      if (action !== "twist") setStatus(`遥操失败 · ${err}`, false);
      return false;
    }
  }

  function setMode(mode) {
    state.mode = mode === "teleop" ? "teleop" : "navigate";
    const stage = document.querySelector(".map-stage");
    stage?.classList.toggle("mode-navigate", state.mode === "navigate");
    stage?.classList.toggle("mode-teleop", state.mode === "teleop");
    document.querySelectorAll(".map-mode-btn").forEach((btn) => {
      btn.classList.toggle("active", btn.dataset.mapMode === state.mode);
    });
    const pad = document.getElementById("map-teleop-pad");
    if (pad) pad.hidden = state.mode !== "teleop";
    if (state.mode === "navigate") {
      stopTeleopLoop(true);
      cancelPoseDraft({ silent: true });
      setInteractHint("按下定点 · 拖动定朝向 · 松开发送");
    } else {
      cancelPoseDraft({ silent: true });
      setInteractHint("拖动圆盘发送速度 · 松手停车");
    }
    syncMapInteractionForMode();
  }

  function sceneMeta(sceneId) {
    const scenes = cfg.scenes || [];
    return scenes.find((s) => s.id === sceneId) || null;
  }

  function showSceneHint(sceneId) {
    const meta = sceneMeta(sceneId);
    if (!meta) {
      setStatus("BicMap 就绪 · 运行任务后显示轨迹", true);
      return;
    }
    setStatus(`场景「${meta.label}」· ${meta.blurb || "运行任务后显示轨迹"}`, true);
  }

  function seedDemoPose() {
    applyTrack({
      task: "demo",
      mission: "navigate",
      path: [
        { x: 0, y: 0, yaw: 0 },
        { x: 2, y: 0.2, yaw: 0.1 },
        { x: 4, y: 0.5, yaw: 0.2 },
      ],
      start: { x: 0, y: 0, yaw: 0 },
      goal: { x: 4, y: 0.5, yaw: 0 },
      pose: { x: 2, y: 0.2, yaw: 0.1 },
    });
  }

  function clearLayers() {
    try {
      state.lineCtrl?.clear?.();
      state.lineCtrl?.remove?.();
    } catch (_) {}
    try {
      state.poiCtrl?.remove?.();
    } catch (_) {}
    try {
      state.robotCtrl?.clearRobots?.();
      state.robotCtrl?.remove?.();
    } catch (_) {}
    clearCloud();
    clearGoalMarker();
    state.lineCtrl = null;
    state.poiCtrl = null;
    state.robotCtrl = null;
  }

  function applyTrack(track, { skipFit = false } = {}) {
    if (!state.ready || !state.map || !track) return;
    state.lastTrack = track;
    const pathLL = pathToLngLat(track.path || []);
    const pose = track.pose || (track.path && track.path[track.path.length - 1]);
    const goal = track.goal;
    const start = track.start;
    const mission = track.mission || "";

    clearLayers();

    // Path polyline (BicMap createPolylines)
    if (
      isDisplayOn("path") &&
      pathLL.length >= 2 &&
      typeof bicMap.createPolylines === "function"
    ) {
      try {
        state.lineCtrl = bicMap.createPolylines(state.map, [
          {
            path: pathLL,
            color: displayColor("path"),
            width: 4,
            opacity: 0.9,
            dashType: "solid",
            showArrow: true,
            name: displayTopic("path") || track.task || "path",
          },
        ]);
      } catch (err) {
        console.warn("createPolylines", err);
      }
    }

    // Goal / start / dock / waypoints POI
    const pois = [];
    if (isDisplayOn("start") && start) {
      pois.push({
        id: "start",
        name: "起点",
        lngLat: metersToLngLat(start.x, start.y),
      });
    }
    if (goal) {
      const isDock = mission === "dock";
      if ((isDock && isDisplayOn("dock")) || (!isDock && isDisplayOn("goal"))) {
        pois.push({
          id: isDock ? "dock" : "goal",
          name: isDock ? "充电桩" : "目标",
          lngLat: metersToLngLat(goal.x, goal.y),
        });
      }
    }
    const wps = track.waypoints;
    if (isDisplayOn("waypoints") && Array.isArray(wps)) {
      wps.forEach((w, i) => {
        if (w && typeof w.x === "number") {
          pois.push({
            id: `wp-${i}`,
            name: `WP${i + 1}`,
            lngLat: metersToLngLat(w.x, w.y),
          });
        }
      });
    }
    if (pois.length && typeof bicMap.addBatchPOIMarkers === "function") {
      try {
        state.poiCtrl = bicMap.addBatchPOIMarkers(state.map, pois, {
          showLabels: true,
        });
      } catch (err) {
        console.warn("POI", err);
      }
    } else if (pois.length && typeof bicMap.addDirectionalMarker === "function") {
      pois.forEach((p) => {
        try {
          bicMap.addDirectionalMarker(state.map, {
            lngLat: p.lngLat,
            label: p.name,
          });
        } catch (_) {}
      });
    }

    // Robot marker — no BicMap HTML bubble (shows lng/lat + looks like a navigate CTA).
    if (isDisplayOn("robot") && pose && typeof bicMap.addRobotMarkers === "function") {
      try {
        state.robotCtrl = bicMap.addRobotMarkers(
          state.map,
          [
            {
              id: "benchmark-robot",
              name: "robot",
              lngLat: metersToLngLat(pose.x, pose.y),
              rotation: yawToDeg(pose.yaw || 0),
            },
          ],
          {
            svgPath: assets.robotSvg || "/static/assets/robot.svg",
            size: 34,
            showLabels: false,
          },
        );
      } catch (err) {
        console.warn("addRobotMarkers", err);
      }
    }

    if (state.poseDraft?.dragging) {
      showPoseGhost(state.poseDraft.x, state.poseDraft.y, state.poseDraft.yaw, false);
    } else if (state.pendingGoal) {
      showPoseGhost(
        state.pendingGoal.x,
        state.pendingGoal.y,
        state.pendingGoal.yaw || 0,
        true,
      );
    }

    updatePathCloud(track);
    setGridVisible(isDisplayOn("grid"));

    if (!skipFit) fitToTrack(pathLL, pose, goal, start);
    setStatus(
      `已渲染 · ${track.task || "—"} · 点位 ${pathLL.length} · ${track.status || ""}`,
      true,
    );
  }

  function fitToTrack(pathLL, pose, goal, start) {
    if (!state.map) return;
    const pts = [...pathLL];
    if (pose) pts.push(metersToLngLat(pose.x, pose.y));
    if (goal) pts.push(metersToLngLat(goal.x, goal.y));
    if (start) pts.push(metersToLngLat(start.x, start.y));
    if (!pts.length) return;
    try {
      if (typeof state.map.fitBounds === "function" && pts.length >= 2) {
        let minLng = pts[0][0];
        let minLat = pts[0][1];
        let maxLng = pts[0][0];
        let maxLat = pts[0][1];
        for (const [lng, lat] of pts) {
          minLng = Math.min(minLng, lng);
          minLat = Math.min(minLat, lat);
          maxLng = Math.max(maxLng, lng);
          maxLat = Math.max(maxLat, lat);
        }
        const pad = 0.00015;
        // Reserve space for floating toolbar / side panels so content looks centered.
        const stage = document.querySelector(".map-stage");
        const topChrome = Math.max(
          96,
          Math.round((stage?.querySelector(".map-overlay-top")?.getBoundingClientRect().height || 0) + 28),
        );
        const leftPad = state.displaysOpen ? 48 + 268 : 48;
        const rightPad = state.imagesOpen ? 48 + 300 : 48;
        const opts = {
          padding: { top: topChrome, bottom: 56, left: leftPad, right: rightPad },
          duration: 600,
        };
        if (state.viewMode === "3d") {
          opts.pitch = 58;
          opts.bearing = state.map.getBearing?.() ?? -20;
        }
        state.map.fitBounds(
          [
            [minLng - pad, minLat - pad],
            [maxLng + pad, maxLat + pad],
          ],
          opts,
        );
      } else if (pose) {
        state.map.flyTo?.({
          center: metersToLngLat(pose.x, pose.y),
          zoom: 18,
          duration: 500,
          pitch: state.viewMode === "3d" ? 58 : 0,
        });
      }
    } catch (err) {
      console.warn("fit", err);
    }
  }

  function selectScene(sceneId) {
    state.sceneId = sceneId;
    document.querySelectorAll(".scene-btn").forEach((btn) => {
      btn.classList.toggle("active", btn.dataset.scene === sceneId);
    });
    const sel = document.getElementById("scene-select");
    if (sel && [...sel.options].some((o) => o.value === sceneId)) {
      sel.value = sceneId;
    }
    const btn = document.querySelector(`.scene-btn[data-scene="${sceneId}"]`);
    const mission =
      btn?.dataset.mission ||
      sel?.selectedOptions?.[0]?.dataset?.mission;
    if (mission === "teleop") setMode("teleop");
    else if (mission === "navigate" || mission === "dock" || mission === "patrol") {
      setMode("navigate");
    }
    if (mission && window.BenchmarkMap?.getTrackForMission) {
      const t = window.BenchmarkMap.getTrackForMission(mission);
      if (t) {
        applyTrack(t);
        return;
      }
    }
    showSceneHint(sceneId);
  }

  function updateTeleopKnob(nx, ny) {
    const knob = document.getElementById("teleop-knob");
    const readout = document.getElementById("teleop-readout");
    const r = 42;
    if (knob) knob.style.transform = `translate(${nx * r}px, ${ny * r}px)`;
    if (readout) {
      readout.textContent = `${state.teleopVx.toFixed(2)} / ${state.teleopWz.toFixed(2)}`;
    }
  }

  function setTeleopFromPad(clientX, clientY) {
    const pad = document.getElementById("teleop-pad");
    if (!pad) return;
    const rect = pad.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    let dx = clientX - cx;
    let dy = clientY - cy;
    const maxR = Math.min(rect.width, rect.height) * 0.34;
    const dist = Math.hypot(dx, dy) || 1;
    if (dist > maxR) {
      dx = (dx / dist) * maxR;
      dy = (dy / dist) * maxR;
    }
    const nx = dx / maxR;
    const ny = dy / maxR;
    // screen: +y down → robot forward is -ny; +x right → +wz (CCW positive when left)
    state.teleopVx = -ny * MAX_V;
    state.teleopWz = -nx * MAX_W;
    if (Math.hypot(state.teleopVx, state.teleopWz) < 0.04) {
      state.teleopVx = 0;
      state.teleopWz = 0;
    }
    updateTeleopKnob(nx, ny);
  }

  function startTeleopLoop() {
    if (state.teleopTimer) return;
    state.teleopActive = true;
    state.teleopStarted = true;
    state.teleopTimer = setInterval(() => {
      if (!state.teleopActive) return;
      if (state.teleopVx === 0 && state.teleopWz === 0) return;
      void sendTeleop("twist", state.teleopVx, state.teleopWz).then((ok) => {
        if (ok) {
          setStatus(
            `遥操 · v=${state.teleopVx.toFixed(2)} ω=${state.teleopWz.toFixed(2)}`,
            true,
          );
        }
      });
    }, 1000 / TELEOP_HZ);
  }

  function stopTeleopLoop(sendStop) {
    state.teleopActive = false;
    if (state.teleopTimer) {
      clearInterval(state.teleopTimer);
      state.teleopTimer = null;
    }
    state.teleopVx = 0;
    state.teleopWz = 0;
    updateTeleopKnob(0, 0);
    if (sendStop && state.teleopStarted) {
      state.teleopStarted = false;
      void sendTeleop("stop");
      setStatus("遥操已停车", true);
    }
  }

  function bindTeleopPad() {
    const pad = document.getElementById("teleop-pad");
    if (!pad || pad.__bound) return;
    pad.__bound = true;
    const onMove = (ev) => {
      if (!state.teleopActive) return;
      const t = ev.touches ? ev.touches[0] : ev;
      if (!t) return;
      ev.preventDefault();
      setTeleopFromPad(t.clientX, t.clientY);
    };
    const onUp = (ev) => {
      if (!state.teleopActive) return;
      ev.preventDefault();
      stopTeleopLoop(true);
    };
    const onDown = (ev) => {
      if (state.mode !== "teleop") return;
      const t = ev.touches ? ev.touches[0] : ev;
      if (!t) return;
      ev.preventDefault();
      state.teleopActive = true;
      setTeleopFromPad(t.clientX, t.clientY);
      startTeleopLoop();
    };
    pad.addEventListener("pointerdown", onDown);
    window.addEventListener("pointermove", onMove);
    window.addEventListener("pointerup", onUp);
    window.addEventListener("pointercancel", onUp);
    pad.addEventListener("touchstart", onDown, { passive: false });
    pad.addEventListener("touchmove", onMove, { passive: false });
    pad.addEventListener("touchend", onUp, { passive: false });
  }

  // Public API for dashboard.js
  window.BenchmarkMap = {
    applyTrack,
    clear: () => {
      clearLayers();
      state.pendingGoal = null;
      cancelPoseDraft({ silent: true });
      hidePoseGhost();
      showSceneHint(state.sceneId);
    },
    fit: () => fitMapView({ force: true }),
    resize: () => {
      resizeMap();
      if (state.needsRefit) fitMapView({ force: true });
    },
    selectScene,
    showSceneHint,
    setStatus,
    setMode,
    setDisplaysOpen,
    setImagesOpen,
    updateImage,
    sendNavigateGoal,
    getTrackForMission: null, // filled by dashboard.js
    isReady: () => state.ready,
  };

  document.getElementById("btn-map-fit")?.addEventListener("click", () => {
    window.BenchmarkMap.fit();
  });
  document.getElementById("btn-map-clear")?.addEventListener("click", () => {
    window.BenchmarkMap.clear();
  });
  document.querySelectorAll(".scene-btn").forEach((btn) => {
    btn.addEventListener("click", () => selectScene(btn.dataset.scene));
  });
  document.querySelectorAll(".map-mode-btn[data-map-mode]").forEach((btn) => {
    btn.addEventListener("click", () => setMode(btn.dataset.mapMode));
  });
  document.querySelectorAll("[data-map-view]").forEach((btn) => {
    btn.addEventListener("click", () => setViewMode(btn.dataset.mapView));
  });
  document.getElementById("btn-teleop-stop")?.addEventListener("click", () => {
    stopTeleopLoop(true);
    void sendTeleop("cancel");
  });
  bindTeleopPad();

  window.BenchmarkMap.setViewMode = setViewMode;

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", initMap);
  } else {
    initMap();
  }
})();
