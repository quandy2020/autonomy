(() => {
  const $ = (id) => document.getElementById(id);
  const esc = (s) =>
    String(s ?? "")
      .replace(/&/g, "&amp;")
      .replace(/"/g, "&quot;")
      .replace(/</g, "&lt;");

  const SERIES_COLORS = {
    v: "#7dcea0",
    w: "#e0a070",
    a: "#7eb6ff",
    alpha: "#c9a0ff",
    jerk: "#e08a8a",
    jerk_angular: "#f0abfc",
  };

  const STORAGE_KEY = "benchmark.dashboard.prefs.v3";
  const DEFAULT_SERIES = ["v", "w", "a", "jerk"];

  const state = {
    catalog: null,
    orchSteps: [],
    configPath: "configs/default.yaml",
    curvesByTask: {},
    mapTracks: {},
    activeCurveTask: "",
    activeTab: "orchestrate",
    activeNav: "tasks",
    taskKind: "orchestration",
    dataMode: "record",
    paramDefaults: {},
    running: false,
    history: [],
    plot: null,
    sse: null,
    sseRetry: 0,
    sseTimer: null,
    otaTimer: null,
    otaBusy: false,
    scheduleSteps: [],
    namedSelected: new Set(),
    namedFocus: "",
    namedGroup: "", // "" = auto; "fav" | "recent" | "all" | group label
    namedRecent: [],
    namedFavs: [],
    namedCallHistory: [],
    namedSending: false,
    namedLinkTimer: null,
    namedLinkBusy: false,
  };

  const NAMED_RECENT_KEY = "benchmark.dashboard.named.recent.v1";
  const NAMED_RECENT_MAX = 8;
  const NAMED_FAV_KEY = "benchmark.dashboard.named.fav.v1";
  const NAMED_FAV_MAX = 24;
  const NAMED_HISTORY_KEY = "benchmark.dashboard.named.history.v1";
  const NAMED_HISTORY_MAX = 8;
  const NAMED_PICKER_W_KEY = "benchmark.dashboard.named.pickerW.v4";
  const NAMED_PICKER_W_MIN = 280;
  const NAMED_PICKER_W_MAX = 720;
  const NAMED_PICKER_W_DEFAULT = 440; // ~27.5rem

  const DATA_MODE_HINTS = {
    record: "采集 episode JSONL · automsgs",
    play: "回放数据集 · Autolink cmd_vel",
  };

  const NAV_DEFAULT_TAB = {
    settings: "params",
    tasks: "orchestrate",
    data: "data",
    test: "curves",
    ops: "ota",
  };

  const TAB_TO_NAV = {
    params: "settings",
    orchestrate: "tasks",
    schedule: "tasks",
    named: "tasks",
    map: "tasks",
    data: "data",
    curves: "test",
    results: "test",
    ota: "ops",
  };

  const TASK_KIND_TAB = {
    orchestration: "orchestrate",
    schedule: "schedule",
    named: "named",
  };

  function switchDataMode(mode) {
    if (!mode || (mode !== "record" && mode !== "play")) return;
    state.dataMode = mode;
    document.querySelectorAll(".data-mode-btn").forEach((btn) => {
      const on = btn.dataset.dataMode === mode;
      btn.classList.toggle("active", on);
      btn.setAttribute("aria-selected", on ? "true" : "false");
    });
    document.querySelectorAll(".data-pane").forEach((pane) => {
      const on = pane.dataset.dataPane === mode;
      pane.classList.toggle("active", on);
      pane.hidden = !on;
    });
    const hint = $("data-mode-hint");
    if (hint) hint.textContent = DATA_MODE_HINTS[mode] || "";
  }

  function syncStageTabsForNav(nav) {
    const tabs = [...document.querySelectorAll(".stage-tabs .tab")];
    const scoped = tabs.filter((btn) => btn.dataset.navScope === nav);
    tabs.forEach((btn) => {
      const show = btn.dataset.navScope === nav;
      btn.hidden = !show;
      btn.style.display = show ? "" : "none";
    });
    const chrome = document.querySelector(".stage-chrome");
    if (chrome) chrome.hidden = scoped.length <= 1;
    document.querySelectorAll(".stage-chrome-actions").forEach((el) => {
      const scope = el.dataset.chromeScope;
      el.hidden = !scope || scope !== nav || scoped.length === 0;
    });
  }

  function switchNav(nav, { tab } = {}) {
    if (!nav || !NAV_DEFAULT_TAB[nav]) return;
    state.activeNav = nav;
    // 设置 / 任务：窄图标栏 + 单主区；数据 / 测试 / 运维保留侧栏
    const compact = nav === "tasks" || nav === "settings";
    document.querySelector(".layout")?.classList.toggle("nav-compact", compact);
    document.querySelector(".sidebar")?.classList.toggle("compact", compact);
    document.querySelectorAll(".side-nav-btn").forEach((btn) => {
      const on = btn.dataset.nav === nav;
      btn.classList.toggle("active", on);
      btn.setAttribute("aria-selected", on ? "true" : "false");
    });
    document.querySelectorAll(".side-panel").forEach((panel) => {
      const on = panel.dataset.navPanel === nav;
      panel.classList.toggle("active", on);
      panel.hidden = !on;
    });
    syncStageTabsForNav(nav);
    const nextTab = tab || NAV_DEFAULT_TAB[nav];
    switchTab(nextTab, { fromNav: true });
    if (nav === "ops") {
      const t = $("ota-target")?.value;
      if ($("side-ota-target") && t) $("side-ota-target").textContent = t;
    }
  }

  function renderNamedLink(data) {
    const wrap = $("named-link");
    const text = $("named-link-text");
    const meta = $("named-link-meta");
    if (!wrap || !text) return;
    const stateKey = data?.state || (data?.linked ? "online" : "offline");
    wrap.dataset.state = stateKey;
    text.textContent = data?.label || "未知";
    const bits = [];
    if (data?.target) bits.push(data.target);
    if (data?.latency_ms != null) bits.push(`${data.latency_ms} ms`);
    if (data?.error && stateKey !== "online") bits.push(data.error);
    if (meta) meta.textContent = bits.join(" · ");
    wrap.title = data?.error
      ? `${data.label || ""} · ${data.error}`
      : `gRPC ${data?.label || ""} @ ${data?.target || ""}`;
  }

  async function refreshNamedLink() {
    if (state.namedLinkBusy) return;
    const target = ($("named-target")?.value || "").trim();
    const wrap = $("named-link");
    if (wrap) wrap.dataset.state = "checking";
    if ($("named-link-text")) $("named-link-text").textContent = "检测中…";
    if ($("named-link-meta")) $("named-link-meta").textContent = target || "";
    state.namedLinkBusy = true;
    try {
      const q = target ? `?target=${encodeURIComponent(target)}` : "";
      const res = await fetch(`/api/rpc/link${q}`);
      const data = await res.json();
      renderNamedLink(data);
    } catch (err) {
      renderNamedLink({
        state: "error",
        label: "检测失败",
        target,
        error: String(err),
      });
    } finally {
      state.namedLinkBusy = false;
    }
  }

  function startNamedLinkPolling() {
    stopNamedLinkPolling();
    refreshNamedLink();
    state.namedLinkTimer = setInterval(() => {
      if (state.activeTab === "named") refreshNamedLink();
    }, 8000);
  }

  function stopNamedLinkPolling() {
    if (state.namedLinkTimer) {
      clearInterval(state.namedLinkTimer);
      state.namedLinkTimer = null;
    }
  }

  function switchTab(name, { fromNav = false } = {}) {
    if (!name) return;
    state.activeTab = name;
    if (!fromNav) {
      const nav = TAB_TO_NAV[name];
      if (nav && nav !== state.activeNav) {
        switchNav(nav, { tab: name });
        return;
      }
    }
    document.querySelectorAll(".stage-tabs .tab").forEach((btn) => {
      const on = btn.dataset.tab === name;
      btn.classList.toggle("active", on);
      btn.setAttribute("aria-selected", on ? "true" : "false");
    });
    document.querySelectorAll(".tab-pane").forEach((pane) => {
      const on = pane.dataset.pane === name;
      pane.classList.toggle("active", on);
      pane.hidden = !on;
    });
    if (name === "curves") {
      const dot = $("curves-dot");
      if (dot) dot.hidden = true;
      requestAnimationFrame(() => drawCurves());
    }
    if (name === "map") {
      // MapLibre was often inited while the pane was hidden (0×0) — resize then center.
      requestAnimationFrame(() => {
        window.BenchmarkMap?.resize?.();
        requestAnimationFrame(() => window.BenchmarkMap?.fit?.());
      });
    }
    if (name === "ota") {
      refreshOtaSnapshot();
      startOtaPolling();
    } else {
      stopOtaPolling();
    }
    if (name === "named") {
      startNamedLinkPolling();
    } else {
      stopNamedLinkPolling();
    }
    const kindFromTab = Object.entries(TASK_KIND_TAB).find(([, tab]) => tab === name)?.[0];
    if (kindFromTab) state.taskKind = kindFromTab;
  }

  function otaTarget() {
    return ($("ota-target")?.value || "localhost:50051").trim();
  }

  function otaQuery() {
    return `?target=${encodeURIComponent(otaTarget())}`;
  }

  function setText(id, value) {
    const el = $(id);
    if (el) el.textContent = value == null || value === "" ? "—" : String(value);
  }

  function applyOtaStatus(ota) {
    if (!ota) return;
    const stateEl = $("ota-state");
    const label = ota.state_label || ota.state || "—";
    if (stateEl) {
      stateEl.textContent = label;
      stateEl.classList.remove("is-busy", "is-ok", "is-fail", "is-idle");
      const key = String(ota.state_key || ota.state || "");
      if (/FAILED|FAIL/i.test(key)) stateEl.classList.add("is-fail");
      else if (/COMMITTED/i.test(key)) stateEl.classList.add("is-ok");
      else if (/IDLE|UNKNOWN/i.test(key) || key === "1" || key === "0") stateEl.classList.add("is-idle");
      else stateEl.classList.add("is-busy");
    }
    const pct = Number(ota.progress_pct ?? (Number(ota.progress || 0) * 100));
    setText("ota-progress-label", `${Number.isFinite(pct) ? pct.toFixed(1) : 0}%`);
    const bar = $("ota-progress-bar");
    if (bar) bar.style.width = `${Math.max(0, Math.min(100, pct || 0))}%`;
    setText("ota-current", ota.currentVersion || ota.current_version);
    setText("ota-target-ver", ota.targetVersion || ota.target_version);
    setText("ota-detail", ota.detail);
  }

  function applyOtaSnapshot(data) {
    const foot = $("ota-footnote");
    if (!data?.ok) {
      if (foot) foot.textContent = data?.error || "OTA 不可用";
      const probe = data?.probe || {};
      if (foot && probe.target) {
        foot.textContent = `${data.error || "OTA 不可用"} · target=${probe.target}`;
      }
      return;
    }
    applyOtaStatus(data.ota);
    const info = data.info || {};
    const caps = data.capabilities || {};
    const supports =
      data.supports_ota ?? caps.supportsOta ?? caps.supports_ota;
    setText("ota-supports", supports == null ? "—" : supports ? "true" : "false");
    setText("side-ota-supports", supports == null ? "—" : supports ? "true" : "false");
    setText("side-ota-target", otaTarget());
    setText("ota-model", info.model);
    setText("ota-serial", info.serialNumber || info.serial_number);
    setText("ota-fw", info.firmwareVersion || info.firmware_version);
    setText("ota-sw", info.softwareVersion || info.software_version);
    setText("ota-autonomy", info.autonomyVersion || info.autonomy_version);
    const startBtn = $("btn-ota-start");
    if (startBtn) startBtn.disabled = supports === false;
    if (foot) {
      const p = data.probe || {};
      foot.textContent = `gRPC ${p.target || otaTarget()} · ${SYSTEM_HINT}`;
    }
  }

  const SYSTEM_HINT = "SystemService · Python gRPC + automsgs/proto/rpcs";

  async function refreshOtaSnapshot() {
    try {
      const res = await fetch(`/api/ota/snapshot${otaQuery()}`);
      const data = await res.json();
      applyOtaSnapshot(data);
    } catch (err) {
      const foot = $("ota-footnote");
      if (foot) foot.textContent = `刷新失败: ${err}`;
    }
  }

  function startOtaPolling() {
    stopOtaPolling();
    state.otaTimer = setInterval(() => {
      if (state.activeTab !== "ota" || state.otaBusy) return;
      refreshOtaSnapshot();
    }, 2000);
  }

  function stopOtaPolling() {
    if (state.otaTimer) {
      clearInterval(state.otaTimer);
      state.otaTimer = null;
    }
  }

  async function runOtaStart() {
    const uri = ($("ota-package-uri")?.value || "").trim();
    if (!uri) {
      toast("请填写 package_uri", "warn");
      return;
    }
    if (!window.confirm(`确认 StartOta？\n${uri}`)) return;
    state.otaBusy = true;
    try {
      const res = await fetch(`/api/ota/start${otaQuery()}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          target: otaTarget(),
          package_uri: uri,
          package_type: $("ota-package-type")?.value || "",
          prefer_delta: !!$("ota-prefer-delta")?.checked,
        }),
      });
      const data = await res.json();
      if (!data.ok) {
        toast(data.error || "StartOta 失败", "error");
        return;
      }
      applyOtaStatus(data.result);
      toast("StartOta 已提交", "ok");
      refreshOtaSnapshot();
    } catch (err) {
      toast(String(err), "error");
    } finally {
      state.otaBusy = false;
    }
  }

  async function runOtaAbort() {
    const reason = ($("ota-abort-reason")?.value || "operator").trim();
    state.otaBusy = true;
    try {
      const res = await fetch(`/api/ota/abort${otaQuery()}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ target: otaTarget(), reason }),
      });
      const data = await res.json();
      if (!data.ok) {
        toast(data.error || "AbortOta 失败", "error");
        return;
      }
      applyOtaStatus(data.result);
      toast("AbortOta 已发送", "warn");
      refreshOtaSnapshot();
    } catch (err) {
      toast(String(err), "error");
    } finally {
      state.otaBusy = false;
    }
  }

  async function runOtaRpc() {
    const method = ($("ota-rpc-method")?.value || "").trim();
    if (!method) {
      toast("请填写 method", "warn");
      return;
    }
    let data = {};
    const raw = $("ota-rpc-body")?.value || "{}";
    try {
      data = JSON.parse(raw);
    } catch {
      toast("JSON body 无效", "error");
      return;
    }
    try {
      const res = await fetch(`/api/ota/call${otaQuery()}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ target: otaTarget(), method, data }),
      });
      const payload = await res.json();
      const out = $("ota-rpc-out");
      if (out) {
        out.textContent = JSON.stringify(payload, null, 2);
      }
      if (!payload.ok) toast(payload.error || "调用失败", "error");
      else toast(`${method} ok`, "ok");
    } catch (err) {
      toast(String(err), "error");
    }
  }

  function markCurvesReady() {
    if (state.activeTab === "curves") return;
    const dot = $("curves-dot");
    if (dot) dot.hidden = false;
  }

  function toast(message, kind = "info") {
    const host = $("toast-host");
    if (!host || !message) return;
    const el = document.createElement("div");
    el.className = `toast ${kind}`;
    el.textContent = message;
    host.appendChild(el);
    requestAnimationFrame(() => el.classList.add("show"));
    setTimeout(() => {
      el.classList.remove("show");
      setTimeout(() => el.remove(), 220);
    }, 2400);
  }

  function setRunning(running) {
    state.running = !!running;
    const btn = $("btn-run");
    if (btn) {
      btn.disabled = state.running;
      btn.textContent = state.running ? "运行中…" : "运行";
      btn.classList.toggle("busy", state.running);
    }
  }

  function savePrefs() {
    try {
      localStorage.setItem(
        STORAGE_KEY,
        JSON.stringify({
          orchSteps: state.orchSteps,
          tasks: state.orchSteps.map((s) => s.task), // legacy
          orchExecution: $("orch-execution")?.value || "sync",
          orchEpisodes: $("orch-episodes")?.value || "3",
          orchStepInterval: $("orch-step-interval")?.value || "0",
          orchRepeat: $("orch-repeat")?.value || "1",
          config: state.configPath,
          record: !!$("record-enable")?.checked,
          params: collectParams(),
        }),
      );
    } catch {
      /* ignore */
    }
  }

  function loadPrefs() {
    try {
      return JSON.parse(localStorage.getItem(STORAGE_KEY) || "null");
    } catch {
      return null;
    }
  }

  function renderHistory(history) {
    const list = $("history-list");
    if (!list) return;
    const items = history || state.history || [];
    state.history = items;
    if (!items.length) {
      list.innerHTML = '<li class="empty">尚无历史</li>';
      return;
    }
    list.innerHTML = items
      .map(
        (h) => `<li class="history-item status-${h.status || "idle"}" data-job="${h.job_id || ""}" role="button" tabindex="0">
          <code>${h.job_id || "—"}</code>
          <span class="st">${h.status || "—"}</span>
        </li>`,
      )
      .join("");
    list.querySelectorAll(".history-item[data-job]").forEach((el) => {
      el.addEventListener("click", () => restoreHistory(el.dataset.job));
    });
  }

  function renderTaskProgress(job) {
    const host = $("task-progress");
    if (!host) return;
    const tasks = job?.tasks || [];
    const states = job?.task_states || {};
    if (!tasks.length) {
      host.innerHTML = "";
      return;
    }
    host.innerHTML = tasks
      .map((t) => {
        const st = states[t] || "pending";
        return `<div class="tp-item ${st}"><span class="tp-dot"></span>${t}</div>`;
      })
      .join("");
  }

  async function restoreHistory(jobId) {
    if (!jobId) return;
    try {
      const res = await fetch(`/api/history/${encodeURIComponent(jobId)}`);
      const data = await res.json();
      if (!data.ok || !data.job) {
        toast(data.error || "未找到", "warn");
        return;
      }
      const job = data.job;
      if (job.summary && Object.keys(job.summary).length) {
        $("summary-view").textContent = JSON.stringify(job.summary, null, 2);
        renderResultCards(job.summary);
      } else if (job.error) {
        $("summary-view").textContent = job.error;
        renderResultCards(null);
      }
      if (job.curves && Object.keys(job.curves).length) {
        state.curvesByTask = job.curves;
        refreshCurveTaskSelect();
        markCurvesReady();
        drawCurves();
      }
      if (job.map_tracks && Object.keys(job.map_tracks).length) {
        state.mapTracks = job.map_tracks;
        const keys = Object.keys(job.map_tracks);
        pushMapTrack(keys[keys.length - 1], job.map_tracks[keys[keys.length - 1]]);
        window.BenchmarkMap?.fit?.();
      }
      renderTaskProgress(job);
      $("live-job").textContent = job.job_id || "—";
      $("live-status").textContent = job.status || "—";
      $("job-meta").textContent = `history · ${job.status}`;
      updateDownloadLinks(job);
      switchTab("results");
    } catch (err) {
      toast(String(err), "error");
    }
  }

  function updateDownloadLinks(job) {
    const jobId = job?.job_id || "";
    const task = state.activeCurveTask || "";
    const sum = $("btn-dl-summary");
    const cur = $("btn-dl-curves");
    if (sum) {
      sum.href = "/api/export/summary";
      sum.classList.toggle("disabled", !jobId);
    }
    if (cur) {
      const q = task ? `?task=${encodeURIComponent(task)}` : "";
      cur.href = `/api/export/curves${q}`;
      cur.classList.toggle("disabled", !jobId);
    }
  }

  function pushMapTrack(task, track) {
    if (!track) return;
    state.mapTracks[task] = track;
    if (window.BenchmarkMap) {
      window.BenchmarkMap.getTrackForMission = (mission) => {
        const hit = Object.values(state.mapTracks).find(
          (t) => t.mission === mission || t.task === mission,
        );
        return hit || state.mapTracks[mission] || null;
      };
      window.BenchmarkMap.applyTrack(track);
      const sceneBtn = document.querySelector(
        `.scene-btn[data-mission="${track.mission || track.task}"]`,
      );
      if (sceneBtn) {
        window.BenchmarkMap.selectScene(sceneBtn.dataset.scene);
        const sel = $("scene-select");
        if (sel) sel.value = sceneBtn.dataset.scene;
      }
    }
  }

  function fmtTime(ts) {
    return new Date((ts || Date.now() / 1000) * 1000).toLocaleTimeString("zh-CN", {
      hour12: false,
    });
  }

  function setConn(mode) {
    const pill = $("conn-pill");
    if (!pill) return;
    const live = mode === true || mode === "live";
    const reconnecting = mode === "reconnecting";
    pill.classList.toggle("live", live);
    pill.classList.toggle("reconnecting", reconnecting);
    pill.classList.toggle("down", !live && !reconnecting);
    $("conn-label").textContent = live ? "live" : reconnecting ? "…" : "off";
  }

  function renderResultCards(results) {
    const host = $("result-cards");
    if (!host) return;
    if (!results || !Object.keys(results).length) {
      host.hidden = true;
      host.innerHTML = "";
      return;
    }
    host.hidden = false;
    host.innerHTML = Object.entries(results)
      .map(([task, block]) => {
        const summary = block?.summary || {};
        const eff = summary.efficiency || summary;
        const rate = eff.success_rate ?? summary.success_rate;
        const elapsed = block?.elapsed_s;
        const rateTxt = typeof rate === "number" ? `${(rate * 100).toFixed(0)}%` : "—";
        const elapTxt = typeof elapsed === "number" ? `${elapsed.toFixed(1)}s` : "—";
        return `<div class="result-card"><strong>${task}</strong><span>success <b>${rateTxt}</b></span><span>耗时 <b>${elapTxt}</b></span></div>`;
      })
      .join("");
  }

  function renderMetrics(metrics) {
    if (!metrics) return;
    if ($("m-uptime")) $("m-uptime").textContent = `${metrics.uptime_s ?? 0}s`;
    if ($("m-jobs")) $("m-jobs").textContent = String(metrics.jobs_total ?? 0);
    if ($("m-okfail")) $("m-okfail").textContent = `${metrics.jobs_ok ?? 0}/${metrics.jobs_failed ?? 0}`;
  }

  function renderJob(job) {
    if (!job) {
      $("live-job").textContent = "—";
      $("live-status").textContent = "idle";
      $("live-current").textContent = "—";
      $("live-progress").textContent = "0%";
      $("progress-bar").style.width = "0%";
      $("job-meta").textContent = "idle";
      $("btn-cancel").disabled = true;
      setRunning(false);
      updateDownloadLinks(null);
      renderTaskProgress(null);
      return;
    }
    const pct = Math.round((job.progress || 0) * 100);
    $("live-job").textContent = job.job_id || "—";
    $("live-status").textContent = job.status || "—";
    $("live-current").textContent = job.current_task || "—";
    $("live-progress").textContent = `${pct}%`;
    $("progress-bar").style.width = `${pct}%`;
    $("job-meta").textContent = `${job.status} · ${(job.tasks || []).join(" · ")}`;
    const active = ["queued", "running"].includes(job.status);
    $("btn-cancel").disabled = !active;
    setRunning(active);
    updateDownloadLinks(job);
    renderTaskProgress(job);
    if (job.summary && Object.keys(job.summary).length) {
      $("summary-view").textContent = JSON.stringify(job.summary, null, 2);
    }
    if (job.error) $("summary-view").textContent = job.error;
    if (job.curves && Object.keys(job.curves).length) {
      state.curvesByTask = job.curves;
      refreshCurveTaskSelect();
      drawCurves();
    }
    if (job.map_tracks && Object.keys(job.map_tracks).length) {
      state.mapTracks = job.map_tracks;
      const keys = Object.keys(job.map_tracks);
      pushMapTrack(keys[keys.length - 1], job.map_tracks[keys[keys.length - 1]]);
    }
  }

  function pushEvent(ev) {
    const log = $("event-log");
    if (!log) return;
    const li = document.createElement("li");
    li.innerHTML = `<span class="t">${fmtTime(ev.ts)}</span><span class="kind ${ev.kind}">${ev.kind}</span><span>${ev.message || ""}</span>`;
    log.prepend(li);
    while (log.children.length > 40) log.removeChild(log.lastChild);
  }

  function switchTaskKind(kind) {
    if (!TASK_KIND_TAB[kind]) return;
    state.taskKind = kind;
    switchTab(TASK_KIND_TAB[kind]);
  }

  const ORCH_PICKER_W_KEY = "benchmark.dashboard.orch.pickerW.v1";
  const ORCH_PICKER_W_MIN = 220;
  const ORCH_PICKER_W_MAX = 560;
  const ORCH_PICKER_W_DEFAULT = 352;

  function defaultOrchEpisodes(mission) {
    const fromUi = Number($("orch-episodes")?.value);
    if (Number.isFinite(fromUi) && fromUi >= 1) return Math.min(200, Math.floor(fromUi));
    return Number(mission?.default_episodes || mission?.episodes || 1) || 1;
  }

  function addOrchStep(taskId, episodes) {
    if (!taskId) return;
    const m = (state.catalog?.missions || []).find((x) => x.id === taskId);
    if (m && m.registered === false) return;
    const ep = episodes != null ? Number(episodes) : defaultOrchEpisodes(m);
    state.orchSteps.push({
      task: taskId,
      episodes: Number.isFinite(ep) && ep >= 1 ? Math.min(200, Math.floor(ep)) : 1,
    });
    renderOrchSteps();
    renderMissions();
    savePrefs();
    toast(`已添加：${taskId}`, "ok");
  }

  function renderOrchSteps() {
    const host = $("orch-steps");
    const count = $("orch-count");
    const hint = $("orch-steps-hint");
    if (count) count.textContent = `${state.orchSteps.length} 步`;
    if (hint) {
      hint.textContent = state.orchSteps.length
        ? `${state.orchSteps.length} 步 · 点击左侧继续添加`
        : "点击左侧添加 · 可同任务多步、改次数、排序";
    }
    if (!host) return;
    const missions = state.catalog?.missions || [];
    if (!state.orchSteps.length) {
      host.innerHTML = '<li class="empty">队列为空 · 从左侧点任务加入</li>';
      return;
    }
    host.innerHTML = state.orchSteps
      .map((step, i) => {
        const opts = missions
          .filter((m) => m.registered !== false)
          .map(
            (m) =>
              `<option value="${esc(m.id)}" ${m.id === step.task ? "selected" : ""}>${esc(m.label || m.id)}</option>`,
          )
          .join("");
        return `<li class="orch-step" data-idx="${i}">
          <span class="orch-idx">${i + 1}</span>
          <select data-orch-task aria-label="步骤 ${i + 1} 任务">${opts}</select>
          <input type="number" data-orch-ep min="1" max="200" value="${esc(step.episodes || 1)}" title="本步 episodes" aria-label="步骤 ${i + 1} 次数" />
          <div class="orch-step-actions">
            <button type="button" class="btn ghost sm" data-orch-up title="上移">↑</button>
            <button type="button" class="btn ghost sm" data-orch-down title="下移">↓</button>
            <button type="button" class="btn ghost sm" data-orch-del title="删除">×</button>
          </div>
        </li>`;
      })
      .join("");
    host.querySelectorAll(".orch-step").forEach((row) => {
      const idx = Number(row.dataset.idx);
      row.querySelector("[data-orch-task]")?.addEventListener("change", (e) => {
        state.orchSteps[idx].task = e.target.value;
        savePrefs();
        renderMissions();
      });
      row.querySelector("[data-orch-ep]")?.addEventListener("change", (e) => {
        const n = Math.max(1, Math.min(200, Number(e.target.value) || 1));
        state.orchSteps[idx].episodes = n;
        e.target.value = String(n);
        savePrefs();
      });
      row.querySelector("[data-orch-up]")?.addEventListener("click", () => {
        if (idx <= 0) return;
        const tmp = state.orchSteps[idx - 1];
        state.orchSteps[idx - 1] = state.orchSteps[idx];
        state.orchSteps[idx] = tmp;
        renderOrchSteps();
        savePrefs();
      });
      row.querySelector("[data-orch-down]")?.addEventListener("click", () => {
        if (idx >= state.orchSteps.length - 1) return;
        const tmp = state.orchSteps[idx + 1];
        state.orchSteps[idx + 1] = state.orchSteps[idx];
        state.orchSteps[idx] = tmp;
        renderOrchSteps();
        savePrefs();
      });
      row.querySelector("[data-orch-del]")?.addEventListener("click", () => {
        state.orchSteps.splice(idx, 1);
        renderOrchSteps();
        renderMissions();
        savePrefs();
      });
    });
  }

  function renderMissions(missions, filter = "") {
    const grid = $("mission-grid");
    if (!grid) return;
    const list = missions || state.catalog?.missions || [];
    const q = (filter || $("orch-filter")?.value || "").trim().toLowerCase();
    const view = list.filter((m) => {
      if (!q) return true;
      return (
        (m.id || "").toLowerCase().includes(q) ||
        (m.label || "").toLowerCase().includes(q) ||
        (m.description || "").toLowerCase().includes(q)
      );
    });
    const count = $("orch-picker-count");
    if (count) count.textContent = String(view.length);
    if (!view.length) {
      grid.innerHTML = '<div class="cmd-detail-empty">无匹配任务</div>';
      return;
    }
    const inQueue = {};
    state.orchSteps.forEach((s) => {
      inQueue[s.task] = (inQueue[s.task] || 0) + 1;
    });
    grid.innerHTML = view
      .map((m) => {
        const registered = m.registered !== false;
        const n = inQueue[m.id] || 0;
        const tip = `${m.description || ""}${m.rpc ? ` · ${m.rpc}` : ""}${registered ? " · 点击加入队列" : "（未注册，请用命令任务）"}`;
        return `<button type="button" class="orch-mission" data-orch-add="${esc(m.id)}" ${registered ? "" : "disabled"} role="listitem" title="${esc(tip)}">
          <strong>${esc(m.label || m.id)}${n ? ` <em>×${n}</em>` : ""}</strong>
          <span>${esc(m.description || (registered ? m.id : "未注册"))}</span>
        </button>`;
      })
      .join("");
    grid.querySelectorAll("[data-orch-add]").forEach((btn) => {
      btn.addEventListener("click", () => addOrchStep(btn.dataset.orchAdd));
    });
  }

  function initOrchPickerResize() {
    const body = $("orch-body");
    const handle = $("orch-resize");
    if (!body || !handle) return;
    const apply = (w, { persist } = {}) => {
      const clamped = Math.max(ORCH_PICKER_W_MIN, Math.min(ORCH_PICKER_W_MAX, w));
      body.style.setProperty("--orch-picker-w", `${clamped}px`);
      if (persist) {
        try {
          localStorage.setItem(ORCH_PICKER_W_KEY, String(clamped));
        } catch {
          /* ignore */
        }
      }
    };
    try {
      const saved = Number(localStorage.getItem(ORCH_PICKER_W_KEY));
      if (Number.isFinite(saved)) apply(saved);
      else apply(ORCH_PICKER_W_DEFAULT);
    } catch {
      apply(ORCH_PICKER_W_DEFAULT);
    }
    let dragging = false;
    let startX = 0;
    let startW = 0;
    const onMove = (e) => {
      if (!dragging) return;
      apply(startW + (e.clientX - startX));
    };
    const onUp = () => {
      if (!dragging) return;
      dragging = false;
      body.classList.remove("is-resizing");
      const cur = parseFloat(getComputedStyle(body).getPropertyValue("--orch-picker-w"));
      apply(cur, { persist: true });
      window.removeEventListener("pointermove", onMove);
      window.removeEventListener("pointerup", onUp);
    };
    handle.addEventListener("pointerdown", (e) => {
      dragging = true;
      startX = e.clientX;
      startW =
        parseFloat(getComputedStyle(body).getPropertyValue("--orch-picker-w")) ||
        ORCH_PICKER_W_DEFAULT;
      body.classList.add("is-resizing");
      window.addEventListener("pointermove", onMove);
      window.addEventListener("pointerup", onUp);
    });
  }

  function namedCatalog() {
    return state.catalog?.named_tasks || [];
  }

  function namedItemById(id) {
    return namedCatalog().find((x) => x.id === id) || null;
  }

  function pathParts(path) {
    return String(path || "")
      .split(".")
      .filter(Boolean)
      .map((p) => (/^\d+$/.test(p) ? Number(p) : p));
  }

  function getByPath(obj, path) {
    let cur = obj;
    for (const key of pathParts(path)) {
      if (cur == null) return undefined;
      cur = cur[key];
    }
    return cur;
  }

  function setByPath(obj, path, value) {
    const parts = pathParts(path);
    if (!parts.length) return obj;
    let cur = obj;
    for (let i = 0; i < parts.length - 1; i++) {
      const key = parts[i];
      const next = parts[i + 1];
      if (cur[key] == null || typeof cur[key] !== "object") {
        cur[key] = typeof next === "number" ? [] : {};
      }
      cur = cur[key];
    }
    cur[parts[parts.length - 1]] = value;
    return obj;
  }

  function readNamedBody() {
    try {
      const raw = JSON.parse($("named-body")?.value || "{}");
      return raw && typeof raw === "object" ? raw : {};
    } catch {
      return null;
    }
  }

  function fitNamedBody() {
    const el = $("named-body");
    if (!el) return;
    el.style.height = "0px";
    const line = 16;
    const pad = 18;
    const lines = Math.max(8, (el.value || "").split("\n").length + 1);
    const byContent = el.scrollHeight + 4;
    const byLines = lines * line + pad;
    const max = Math.floor(window.innerHeight * 0.7);
    const next = Math.min(max, Math.max(byContent, byLines, 128));
    el.style.height = `${next}px`;
  }

  function writeNamedBody(body) {
    const el = $("named-body");
    if (!el) return;
    el.value = JSON.stringify(body || {}, null, 2);
    requestAnimationFrame(fitNamedBody);
  }

  function syncNamedFormFromBody() {
    const body = readNamedBody();
    if (!body) return;
    document.querySelectorAll("#named-form [data-cmd-path]").forEach((el) => {
      const val = getByPath(body, el.dataset.cmdPath);
      if (val == null) {
        el.value = "";
        return;
      }
      if (typeof val === "boolean") {
        el.value = val ? "true" : "false";
        return;
      }
      el.value = String(val);
    });
  }

  function syncNamedBodyFromForm() {
    const body = readNamedBody();
    if (!body) {
      toast("请求 JSON 无效，请先修复高级参数", "error");
      return null;
    }
    document.querySelectorAll("#named-form [data-cmd-path]").forEach((el) => {
      const path = el.dataset.cmdPath;
      const type = el.dataset.cmdType || "text";
      let value = el.value;
      if (type === "number") {
        if (value === "" || value == null) return;
        value = Number(value);
        if (Number.isNaN(value)) return;
      } else if (type === "select" && (value === "true" || value === "false")) {
        value = value === "true";
      }
      setByPath(body, path, value);
    });
    writeNamedBody(body);
    return body;
  }

  function renderNamedForm(item, body) {
    const host = $("named-form");
    if (!host) return;
    const fields = Array.isArray(item?.fields) ? item.fields : [];
    if (!fields.length) {
      const keys =
        body && typeof body === "object" ? Object.keys(body).length : 0;
      host.innerHTML = keys
        ? '<p class="cmd-form-empty">无快捷字段 · 在下方 JSON 中编辑</p>'
        : '<p class="cmd-form-empty is-ready">无需请求参数<br/><span>可直接发送</span></p>';
      return;
    }
    host.innerHTML = fields
      .map((f) => {
        const path = f.path || "";
        const label = f.label || path;
        const type = f.type || "text";
        const cur = getByPath(body, path);
        const val = cur == null ? "" : String(cur);
        const wide = type === "text" && !path.includes(".") ? " cmd-field-wide" : "";
        if (type === "select") {
          const opts = (f.options || [])
            .map(
              (o) =>
                `<option value="${esc(o)}" ${String(o) === val ? "selected" : ""}>${esc(o)}</option>`,
            )
            .join("");
          return `<label class="cmd-field${wide}">
            <span>${esc(label)}</span>
            <select data-cmd-path="${esc(path)}" data-cmd-type="select">${opts}</select>
          </label>`;
        }
        const step = f.step != null ? ` step="${f.step}"` : "";
        const ph = f.placeholder ? ` placeholder="${esc(f.placeholder)}"` : "";
        return `<label class="cmd-field${wide}">
          <span>${esc(label)}</span>
          <input type="${type === "number" ? "number" : "text"}" data-cmd-path="${esc(path)}" data-cmd-type="${esc(type)}" value="${esc(val)}"${step}${ph} />
        </label>`;
      })
      .join("");
    host.querySelectorAll("[data-cmd-path]").forEach((el) => {
      el.addEventListener("input", () => syncNamedBodyFromForm());
      el.addEventListener("change", () => syncNamedBodyFromForm());
    });
  }

  function inferNamedStatusKind(status, payload) {
    if (status?.kind) return status.kind;
    if (payload && typeof payload === "object") {
      if (payload.ok === false || payload.error) return "fail";
      if (payload.ok === true) return "ok";
      const code = payload.code ?? payload.response?.code;
      if (code != null && code !== 0 && code !== "OK" && code !== "ok") return "fail";
    }
    return "";
  }

  function setNamedResponse(status, payload) {
    const statusEl = $("named-response-status");
    const pre = $("named-response");
    const kind = inferNamedStatusKind(status, payload);
    if (statusEl) {
      statusEl.textContent = status.label || "—";
      statusEl.className = `cmd-io-status ${kind}`.trim();
    }
    if (pre) {
      pre.classList.toggle("is-fail", kind === "fail");
      pre.classList.toggle("is-ok", kind === "ok");
      if (typeof payload === "string") pre.textContent = payload;
      else pre.textContent = JSON.stringify(payload ?? {}, null, 2);
    }
  }

  function setNamedSending(busy) {
    state.namedSending = !!busy;
    const sendBtn = $("btn-named-send");
    if (!sendBtn) return;
    const item = namedItemById(state.namedFocus);
    if (state.namedSending) {
      sendBtn.disabled = true;
      sendBtn.textContent = "发送中…";
      return;
    }
    if (!item) {
      sendBtn.disabled = true;
      sendBtn.classList.remove("danger");
      sendBtn.textContent = "发送";
      return;
    }
    sendBtn.disabled = false;
    sendBtn.classList.toggle("danger", !!item.danger);
    sendBtn.textContent = item.danger ? "确认急停" : "发送";
    sendBtn.title = item.danger ? "危险操作 · Enter 发送" : "发送 (Enter)";
  }

  function fillNamedEditor(item, { resetBody = true } = {}) {
    const empty = $("named-detail-empty");
    const main = $("named-detail-main");
    if (!item) {
      if (empty) empty.hidden = false;
      if (main) main.hidden = true;
      setNamedSending(false);
      renderNamedHistory();
      return;
    }
    if (empty) empty.hidden = true;
    if (main) main.hidden = false;
    const method = item.method || item.label || item.id;
    const zh =
      item.label && item.label !== method && item.label !== item.id ? item.label : "";
    if ($("named-title")) $("named-title").textContent = method;
    const desc = $("named-desc");
    if (desc) {
      const raw = (item.description || "").trim();
      const trivial =
        !raw ||
        raw === `${item.service}.${item.method}` ||
        raw === `${item.service}/${item.method}` ||
        raw === method;
      const text = trivial ? zh : raw;
      desc.textContent = text;
      desc.hidden = !text;
    }
    if ($("named-method")) {
      $("named-method").textContent = item.rpc || `${item.service}/${item.method}`;
    }
    if (resetBody) {
      const fromMap =
        item.id === "rpc.navigate" && state.mapGoalBody && typeof state.mapGoalBody === "object"
          ? state.mapGoalBody
          : null;
      writeNamedBody(fromMap || item.default_body || {});
      setNamedResponse({ label: "—", kind: "" }, "尚未发送");
    }
    const body = readNamedBody() || item.default_body || {};
    renderNamedForm(item, body);
    syncNamedFormFromBody();
    const advanced = $("named-advanced");
    const summary = $("named-json-summary");
    const hasFields = Array.isArray(item.fields) && item.fields.length > 0;
    const bodyKeys = body && typeof body === "object" ? Object.keys(body).length : 0;
    if (summary) summary.textContent = bodyKeys ? "JSON" : "JSON（空）";
    if (advanced) {
      advanced.open = !hasFields && bodyKeys > 0;
      if (advanced.open) requestAnimationFrame(fitNamedBody);
    }
    if (!state.namedSending) setNamedSending(false);
    renderNamedHistory();
  }

  function loadNamedRecent() {
    try {
      const raw = JSON.parse(localStorage.getItem(NAMED_RECENT_KEY) || "[]");
      state.namedRecent = Array.isArray(raw)
        ? raw.filter((id) => typeof id === "string").slice(0, NAMED_RECENT_MAX)
        : [];
    } catch {
      state.namedRecent = [];
    }
  }

  function pushNamedRecent(id) {
    if (!id) return;
    state.namedRecent = [id, ...state.namedRecent.filter((x) => x !== id)].slice(
      0,
      NAMED_RECENT_MAX,
    );
    try {
      localStorage.setItem(NAMED_RECENT_KEY, JSON.stringify(state.namedRecent));
    } catch {
      /* ignore */
    }
  }

  function loadNamedFavs() {
    try {
      const raw = JSON.parse(localStorage.getItem(NAMED_FAV_KEY) || "[]");
      state.namedFavs = Array.isArray(raw)
        ? raw.filter((id) => typeof id === "string").slice(0, NAMED_FAV_MAX)
        : [];
    } catch {
      state.namedFavs = [];
    }
  }

  function isNamedFav(id) {
    return state.namedFavs.includes(id);
  }

  function toggleNamedFav(id) {
    if (!id) return;
    if (isNamedFav(id)) {
      state.namedFavs = state.namedFavs.filter((x) => x !== id);
    } else {
      state.namedFavs = [id, ...state.namedFavs.filter((x) => x !== id)].slice(
        0,
        NAMED_FAV_MAX,
      );
    }
    try {
      localStorage.setItem(NAMED_FAV_KEY, JSON.stringify(state.namedFavs));
    } catch {
      /* ignore */
    }
    renderNamedTasks();
  }

  function loadNamedCallHistory() {
    try {
      const raw = JSON.parse(localStorage.getItem(NAMED_HISTORY_KEY) || "[]");
      state.namedCallHistory = Array.isArray(raw) ? raw.slice(0, NAMED_HISTORY_MAX) : [];
    } catch {
      state.namedCallHistory = [];
    }
  }

  function pushNamedCallHistory(entry) {
    if (!entry?.id) return;
    state.namedCallHistory = [entry, ...state.namedCallHistory].slice(0, NAMED_HISTORY_MAX);
    try {
      localStorage.setItem(NAMED_HISTORY_KEY, JSON.stringify(state.namedCallHistory));
    } catch {
      /* ignore */
    }
    renderNamedHistory();
  }

  function renderNamedHistory() {
    const host = $("named-history");
    if (!host) return;
    if (!state.namedCallHistory.length) {
      host.innerHTML = "";
      host.hidden = true;
      return;
    }
    host.hidden = false;
    host.innerHTML = `<div class="cmd-history-head">最近调用</div>
      <div class="cmd-history-list">
        ${state.namedCallHistory
          .map((h, i) => {
            const method = h.method || h.id;
            const stamp = h.at
              ? new Date(h.at).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit", second: "2-digit" })
              : "";
            return `<button type="button" class="cmd-history-item${h.ok ? " ok" : " fail"}" data-hist-idx="${i}" title="${esc(h.rpc || method)}">
              <strong>${esc(method)}</strong>
              <span>${h.ok ? "ok" : "fail"}${stamp ? ` · ${esc(stamp)}` : ""}</span>
            </button>`;
          })
          .join("")}
      </div>`;
    host.querySelectorAll("[data-hist-idx]").forEach((btn) => {
      btn.addEventListener("click", () => {
        const idx = Number(btn.dataset.histIdx);
        const h = state.namedCallHistory[idx];
        if (!h) return;
        const item = namedItemById(h.id);
        if (!item) {
          toast("命令已不在目录中", "warn");
          return;
        }
        state.namedSelected = new Set([h.id]);
        state.namedFocus = h.id;
        state.namedGroup = item.group || item.service || "all";
        pushNamedRecent(h.id);
        fillNamedEditor(item, { resetBody: false });
        writeNamedBody(h.body || item.default_body || {});
        renderNamedForm(item, h.body || {});
        syncNamedFormFromBody();
        setNamedResponse(
          {
            label: h.ok ? "历史 · 成功" : "历史 · 失败",
            kind: h.ok ? "ok" : "fail",
          },
          h.response != null
            ? { ok: h.ok, error: h.error, response: h.response }
            : h.error || "无保存的响应",
        );
        renderNamedTasks();
      });
    });
  }

  function namedGroupNames(list) {
    const names = [];
    const seen = new Set();
    for (const t of list) {
      const g = t.group || t.service || "其他";
      if (seen.has(g)) continue;
      seen.add(g);
      names.push(g);
    }
    return names;
  }

  function effectiveNamedGroup(list) {
    if (state.namedGroup) return state.namedGroup;
    if (state.namedFavs.some((id) => namedItemById(id))) return "fav";
    const focus = namedItemById(state.namedFocus);
    if (focus) return focus.group || focus.service || "all";
    const groups = namedGroupNames(list);
    return groups.includes("导航") ? "导航" : groups[0] || "all";
  }

  function namedItemButton(t, { showGroup = false } = {}) {
    const on = state.namedSelected.has(t.id) || state.namedFocus === t.id;
    const method = t.method || t.label || t.id;
    const zh = t.label && t.label !== method && t.label !== t.id ? t.label : "";
    const group = t.group || t.service || "";
    const tip = [zh || method, t.rpc || `${t.service}.${method}`, t.description]
      .filter(Boolean)
      .join(" · ");
    const sub = zh || (showGroup ? group : "");
    const fav = isNamedFav(t.id);
    return `<button type="button" class="named-item${on ? " selected" : ""}${t.danger ? " danger" : ""}" data-named-id="${t.id}" role="listitem" title="${esc(tip)}">
      <span class="named-meta">
        <strong class="named-method">${esc(method)}</strong>
        ${sub ? `<span class="named-alias">${esc(sub)}</span>` : ""}
      </span>
      <span class="named-star${fav ? " on" : ""}" data-named-fav="${esc(t.id)}" title="${fav ? "取消收藏" : "收藏"}" role="button" aria-label="${fav ? "取消收藏" : "收藏"}">${fav ? "★" : "☆"}</span>
    </button>`;
  }

  function renderNamedGroupTabs(list, activeGroup, q) {
    const host = $("named-group-tabs");
    if (!host) return;
    const groups = namedGroupNames(list);
    const favCount = state.namedFavs.filter((id) => namedItemById(id)).length;
    const recentCount = state.namedRecent.filter((id) => namedItemById(id)).length;
    const pinned = [
      { id: "fav", label: "收藏", count: favCount },
      { id: "recent", label: "最近", count: recentCount },
      { id: "all", label: "全部", count: list.length },
    ];
    const current = q ? "all" : activeGroup;
    const inPinned = pinned.some((t) => t.id === current);
    const chips = pinned
      .map((tab) => {
        const on = !q && tab.id === current;
        const dim = q && tab.id !== "all";
        return `<button type="button" class="cmd-group-tab${on ? " active" : ""}${dim ? " dim" : ""}" data-named-group="${esc(tab.id)}" title="${esc(tab.label)} · ${tab.count}">
          <span>${esc(tab.label)}</span>
          <em>${tab.count}</em>
        </button>`;
      })
      .join("");
    const opts = groups
      .map((g) => {
        const n = list.filter((t) => (t.group || t.service || "其他") === g).length;
        return `<option value="${esc(g)}" ${!q && current === g ? "selected" : ""}>${esc(g)} (${n})</option>`;
      })
      .join("");
    host.innerHTML = `${chips}
      <select class="cmd-group-select${!q && !inPinned ? " active" : ""}" id="named-group-select" aria-label="服务分组" title="按服务分组">
        <option value="">分组…</option>
        ${opts}
      </select>`;
    host.querySelectorAll("[data-named-group]").forEach((btn) => {
      btn.addEventListener("click", () => {
        state.namedGroup = btn.dataset.namedGroup || "all";
        if ($("named-filter")) $("named-filter").value = "";
        renderNamedTasks();
      });
    });
    $("named-group-select")?.addEventListener("change", (e) => {
      const v = e.target.value;
      if (!v) return;
      state.namedGroup = v;
      if ($("named-filter")) $("named-filter").value = "";
      renderNamedTasks();
    });
  }

  function selectNamedTask(id) {
    const item = namedItemById(id);
    if (!item) return;
    const changed = state.namedFocus !== id;
    state.namedSelected = new Set([id]);
    state.namedFocus = id;
    pushNamedRecent(id);
    fillNamedEditor(item, { resetBody: changed });
    renderNamedTasks();
  }

  function renderNamedTasks(items, filter = "") {
    const host = $("named-task-list");
    if (!host) return;
    const list = items || namedCatalog();
    loadNamedRecent();
    loadNamedFavs();
    const q = (filter || $("named-filter")?.value || "").trim().toLowerCase();
    const activeGroup = effectiveNamedGroup(list);
    renderNamedGroupTabs(list, activeGroup, q);

    const matchQuery = (t) =>
      (t.label || "").toLowerCase().includes(q) ||
      (t.description || "").toLowerCase().includes(q) ||
      (t.group || "").toLowerCase().includes(q) ||
      (t.service || "").toLowerCase().includes(q) ||
      (t.method || "").toLowerCase().includes(q) ||
      (t.rpc || "").toLowerCase().includes(q) ||
      (t.id || "").toLowerCase().includes(q);

    let view;
    if (q) {
      view = list.filter(matchQuery);
    } else if (activeGroup === "fav") {
      view = state.namedFavs.map((id) => namedItemById(id)).filter(Boolean);
    } else if (activeGroup === "recent") {
      view = state.namedRecent.map((id) => namedItemById(id)).filter(Boolean);
    } else if (activeGroup !== "all") {
      view = list.filter((t) => (t.group || t.service || "其他") === activeGroup);
    } else {
      view = list;
    }
    const count = $("named-picker-title");
    if (count) {
      count.textContent = q ? `${view.length}/${list.length}` : String(view.length);
      count.title = q
        ? `搜索匹配 ${view.length} / 共 ${list.length}`
        : activeGroup === "all"
          ? `共 ${list.length} 条`
          : `${activeGroup} · ${view.length} 条`;
    }

    if (!state.namedFocus && view[0]) {
      state.namedSelected = new Set([view[0].id]);
      state.namedFocus = view[0].id;
      fillNamedEditor(view[0], { resetBody: true });
    }
    if (state.namedFocus && !state.namedSelected.size) {
      state.namedSelected.add(state.namedFocus);
    }

    if (!view.length) {
      let emptyMsg = "无匹配";
      if (!q && activeGroup === "fav") emptyMsg = "暂无收藏 · 点 ★ 收藏常用命令";
      else if (!q && activeGroup === "recent") emptyMsg = "暂无最近使用 · 选一条命令后会出现在这里";
      host.innerHTML = `<div class="cmd-detail-empty">${emptyMsg}</div>`;
      if (!namedItemById(state.namedFocus)) fillNamedEditor(null);
      return;
    }

    if (!q && activeGroup === "all") {
      const groups = new Map();
      for (const t of view) {
        const g = t.group || t.service || "其他";
        if (!groups.has(g)) groups.set(g, []);
        groups.get(g).push(t);
      }
      host.innerHTML = [...groups.entries()]
        .map(
          ([group, rows]) =>
            `<details class="cmd-group" ${rows.some((t) => t.id === state.namedFocus) ? "open" : ""}>
              <summary class="cmd-group-label"><span>${esc(group)}</span><em>${rows.length}</em></summary>
              <div class="cmd-group-body">${rows.map((t) => namedItemButton(t)).join("")}</div>
            </details>`,
        )
        .join("");
    } else {
      const showGroup = !!q || activeGroup === "recent" || activeGroup === "fav";
      host.innerHTML = `<div class="cmd-group-body">${view
        .map((t) => namedItemButton(t, { showGroup }))
        .join("")}</div>`;
    }

    host.querySelectorAll(".named-item").forEach((btn) => {
      btn.addEventListener("click", (e) => {
        const star = e.target.closest?.("[data-named-fav]");
        if (star) {
          e.preventDefault();
          e.stopPropagation();
          toggleNamedFav(star.dataset.namedFav);
          return;
        }
        selectNamedTask(btn.dataset.namedId);
      });
    });
    fillNamedEditor(namedItemById(state.namedFocus), { resetBody: false });
  }

  const SCHED_STEPS_KEY = "benchmark.dashboard.sched.steps.v1";
  const SCHED_PICKER_W_KEY = "benchmark.dashboard.sched.pickerW.v1";
  const SCHED_PICKER_W_MIN = 220;
  const SCHED_PICKER_W_MAX = 560;
  const SCHED_PICKER_W_DEFAULT = 352;

  function syncSchedLoopUi() {
    const loopOn = !!$("sched-loop")?.checked;
    const repeatEl = $("sched-repeat");
    if (repeatEl) {
      repeatEl.disabled = loopOn;
      repeatEl.title = loopOn
        ? "无限循环已开启，循环次数无效"
        : "整条步骤队列重复几轮";
    }
  }

  function saveSchedSteps() {
    try {
      localStorage.setItem(
        SCHED_STEPS_KEY,
        JSON.stringify({
          steps: state.scheduleSteps,
          execution: $("sched-execution")?.value || "sync",
          interval: $("sched-interval")?.value || "0",
          repeat: $("sched-repeat")?.value || "1",
          episodes: $("sched-episodes")?.value || "1",
          delay: $("sched-delay")?.value || "0",
          startAt: $("sched-start-at")?.value || "",
          endAt: $("sched-end-at")?.value || "",
          loop: !!$("sched-loop")?.checked,
        }),
      );
    } catch {
      /* ignore */
    }
  }

  function loadSchedSteps() {
    try {
      const raw = JSON.parse(localStorage.getItem(SCHED_STEPS_KEY) || "null");
      if (!raw || typeof raw !== "object") return false;
      if (Array.isArray(raw.steps)) {
        state.scheduleSteps = raw.steps
          .filter((s) => s && typeof s.task === "string")
          .map((s) => ({ task: s.task }));
      }
      if ($("sched-execution") && raw.execution) $("sched-execution").value = raw.execution;
      if ($("sched-interval") && raw.interval != null) $("sched-interval").value = raw.interval;
      if ($("sched-repeat") && raw.repeat != null) $("sched-repeat").value = raw.repeat;
      if ($("sched-episodes") && raw.episodes != null) $("sched-episodes").value = raw.episodes;
      if ($("sched-delay") && raw.delay != null) $("sched-delay").value = raw.delay;
      if ($("sched-start-at") && raw.startAt != null) $("sched-start-at").value = raw.startAt;
      if ($("sched-end-at") && raw.endAt != null) $("sched-end-at").value = raw.endAt;
      if ($("sched-loop")) $("sched-loop").checked = !!raw.loop;
      syncSchedLoopUi();
      return true;
    } catch {
      return false;
    }
  }

  function addSchedStep(taskId) {
    if (!taskId) return;
    state.scheduleSteps.push({ task: taskId });
    renderSchedSteps();
    saveSchedSteps();
    toast(`已添加：${taskId}`, "ok");
  }

  function renderSchedMissions(filter = "") {
    const host = $("sched-mission-list");
    if (!host) return;
    const missions = state.catalog?.missions || [];
    const q = (filter || $("sched-filter")?.value || "").trim().toLowerCase();
    const view = missions.filter((m) => {
      if (!q) return true;
      return (
        (m.id || "").toLowerCase().includes(q) ||
        (m.label || "").toLowerCase().includes(q) ||
        (m.description || "").toLowerCase().includes(q)
      );
    });
    const count = $("sched-picker-count");
    if (count) count.textContent = String(view.length);
    if (!view.length) {
      host.innerHTML = '<div class="cmd-detail-empty">无匹配任务</div>';
      return;
    }
    host.innerHTML = view
      .map((m) => {
        const ok = m.registered !== false;
        return `<button type="button" class="sched-mission" data-sched-add="${esc(m.id)}" ${ok ? "" : "disabled"} role="listitem" title="${esc(m.description || m.id)}">
          <strong>${esc(m.label || m.id)}</strong>
          <span>${ok ? esc(m.description || m.id) : "未注册"}</span>
        </button>`;
      })
      .join("");
    host.querySelectorAll("[data-sched-add]").forEach((btn) => {
      btn.addEventListener("click", () => addSchedStep(btn.dataset.schedAdd));
    });
  }

  function renderSchedSteps() {
    const host = $("sched-steps");
    if (!host) return;
    const missions = state.catalog?.missions || [];
    const hint = $("sched-steps-hint");
    if (hint) {
      hint.textContent = state.scheduleSteps.length
        ? `${state.scheduleSteps.length} 步 · 点击左侧继续添加`
        : "点击左侧任务添加 · 可排序";
    }
    if (!state.scheduleSteps.length) {
      host.innerHTML = '<li class="empty">尚无步骤 · 点击左侧任务添加</li>';
      return;
    }
    host.innerHTML = state.scheduleSteps
      .map((step, i) => {
        const opts = missions
          .map(
            (m) =>
              `<option value="${m.id}" ${m.id === step.task ? "selected" : ""}>${m.label}${m.registered === false ? " (未注册)" : ""}</option>`,
          )
          .join("");
        return `<li class="sched-step" data-idx="${i}">
          <span class="sched-idx">${i + 1}</span>
          <select data-sched-task aria-label="步骤 ${i + 1} 任务">${opts}</select>
          <div class="sched-step-actions">
            <button type="button" class="btn ghost sm" data-sched-up title="上移">↑</button>
            <button type="button" class="btn ghost sm" data-sched-down title="下移">↓</button>
            <button type="button" class="btn ghost sm" data-sched-del title="删除">×</button>
          </div>
        </li>`;
      })
      .join("");
    host.querySelectorAll(".sched-step").forEach((row) => {
      const idx = Number(row.dataset.idx);
      row.querySelector("[data-sched-task]")?.addEventListener("change", (e) => {
        state.scheduleSteps[idx].task = e.target.value;
        saveSchedSteps();
      });
      row.querySelector("[data-sched-up]")?.addEventListener("click", () => {
        if (idx <= 0) return;
        const tmp = state.scheduleSteps[idx - 1];
        state.scheduleSteps[idx - 1] = state.scheduleSteps[idx];
        state.scheduleSteps[idx] = tmp;
        renderSchedSteps();
        saveSchedSteps();
      });
      row.querySelector("[data-sched-down]")?.addEventListener("click", () => {
        if (idx >= state.scheduleSteps.length - 1) return;
        const tmp = state.scheduleSteps[idx + 1];
        state.scheduleSteps[idx + 1] = state.scheduleSteps[idx];
        state.scheduleSteps[idx] = tmp;
        renderSchedSteps();
        saveSchedSteps();
      });
      row.querySelector("[data-sched-del]")?.addEventListener("click", () => {
        state.scheduleSteps.splice(idx, 1);
        renderSchedSteps();
        saveSchedSteps();
      });
    });
  }

  function initSchedPickerResize() {
    const body = $("sched-body");
    const handle = $("sched-resize");
    if (!body || !handle) return;

    const clamp = (px) => {
      const maxByParent = Math.max(
        SCHED_PICKER_W_MIN,
        Math.floor(body.getBoundingClientRect().width * 0.55),
      );
      return Math.round(
        Math.min(SCHED_PICKER_W_MAX, maxByParent, Math.max(SCHED_PICKER_W_MIN, px)),
      );
    };
    const apply = (px, persist = false) => {
      const w = clamp(px);
      body.style.setProperty("--sched-picker-w", `${w}px`);
      if (persist) {
        try {
          localStorage.setItem(SCHED_PICKER_W_KEY, String(w));
        } catch {
          /* ignore */
        }
      }
    };

    let saved = SCHED_PICKER_W_DEFAULT;
    try {
      const raw = Number(localStorage.getItem(SCHED_PICKER_W_KEY));
      if (Number.isFinite(raw) && raw > 0) saved = raw;
    } catch {
      /* ignore */
    }
    apply(saved);

    let dragging = false;
    let startX = 0;
    let startW = 0;
    handle.addEventListener("pointerdown", (e) => {
      if (e.button != null && e.button !== 0) return;
      e.preventDefault();
      dragging = true;
      startX = e.clientX;
      startW =
        parseFloat(getComputedStyle(body).getPropertyValue("--sched-picker-w")) ||
        SCHED_PICKER_W_DEFAULT;
      body.classList.add("is-resizing");
      handle.setPointerCapture?.(e.pointerId);
    });
    handle.addEventListener("pointermove", (e) => {
      if (!dragging) return;
      apply(startW + (e.clientX - startX));
    });
    const onUp = () => {
      if (!dragging) return;
      dragging = false;
      body.classList.remove("is-resizing");
      const cur = parseFloat(getComputedStyle(body).getPropertyValue("--sched-picker-w"));
      if (Number.isFinite(cur)) apply(cur, true);
    };
    handle.addEventListener("pointerup", onUp);
    handle.addEventListener("pointercancel", onUp);
  }

  function renderSelect(id, values, preferred) {
    const sel = $(id);
    if (!sel) return;
    const current = sel.value;
    const list = values || [];
    sel.innerHTML = list.map((v) => `<option value="${v}">${v}</option>`).join("");
    if (list.includes(current)) sel.value = current;
    else if (preferred && list.includes(preferred)) sel.value = preferred;
  }

  function syncSettingsSide() {
    /* side summary removed; keep hook for call sites */
  }

  function renderConfigs(configs) {
    const sel = $("config-select");
    if (!sel) return;
    sel.innerHTML = configs.map((c) => `<option value="${c.path}">${c.path}</option>`).join("");
    if ([...sel.options].some((o) => o.value === state.configPath)) sel.value = state.configPath;
    syncSettingsSide();
  }

  function syncHiddenFromParams() {
    const modeEl = document.querySelector('#param-fields [data-param="task.mode"]');
    const epEl = document.querySelector('#param-fields [data-param="task.episodes"]');
    if ($("mode-select") && modeEl) $("mode-select").value = modeEl.value;
    if ($("episodes-input") && epEl) $("episodes-input").value = epEl.value;
  }

  function applyShowWhen() {
    const host = $("param-fields");
    if (!host) return;
    const mode =
      host.querySelector('[data-param="task.mode"]')?.value ||
      $("mode-select")?.value ||
      "";
    host.querySelectorAll(".param-field[data-show-when]").forEach((el) => {
      let rule = {};
      try {
        rule = JSON.parse(el.dataset.showWhen || "{}");
      } catch {
        rule = {};
      }
      let visible = true;
      for (const [k, v] of Object.entries(rule)) {
        const cur =
          host.querySelector(`[data-param="${k}"]`)?.value ??
          (k === "task.mode" ? mode : "");
        if (String(cur) !== String(v)) {
          visible = false;
          break;
        }
      }
      el.classList.toggle("hidden-by-when", !visible);
    });
  }

  function collectParams() {
    const host = $("param-fields");
    const out = {};
    if (!host) return out;
    host.querySelectorAll("[data-param]").forEach((el) => {
      if (el.closest(".hidden-by-when")) return;
      const key = el.dataset.param;
      if (!key) return;
      const kind = el.dataset.kind || "text";
      if (kind === "bool") {
        out[key] = !!el.checked;
      } else if (kind === "int") {
        out[key] = Number.parseInt(el.value, 10);
      } else if (kind === "number" || kind === "range") {
        out[key] = Number.parseFloat(el.value);
      } else {
        out[key] = el.value;
      }
    });
    return out;
  }

  function updateSettingsParamCount(n) {
    const badge = $("settings-param-badge");
    if (badge) badge.textContent = String(n);
  }

  function renderParams(params) {
    const host = $("param-fields");
    if (!host) return;
    const fields = params?.fields || [];
    const values = params?.values || {};
    state.paramDefaults = { ...values };
    updateSettingsParamCount(fields.length);
    if (!fields.length) {
      host.innerHTML = `<div class="settings-empty">当前配置无白名单参数 · 可在 YAML 中扩展</div>`;
      return;
    }
    const groups = [];
    const byGroup = new Map();
    for (const f of fields) {
      const g = f.group || "参数";
      if (!byGroup.has(g)) {
        byGroup.set(g, []);
        groups.push(g);
      }
      byGroup.get(g).push(f);
    }
    host.innerHTML = groups
      .map((g) => {
        const items = byGroup.get(g) || [];
        const cells = items
          .map((f) => {
            const key = f.key;
            const val = values[key] ?? "";
            const showWhen = f.show_when
              ? ` data-show-when='${JSON.stringify(f.show_when)}'`
              : "";
            const wide = f.type === "range" ? " wide" : "";
            if (f.type === "bool") {
              return `<div class="param-field bool-field"${showWhen}>
                <label><input type="checkbox" data-param="${key}" data-kind="bool" ${val ? "checked" : ""} />${f.label}</label>
              </div>`;
            }
            if (f.type === "select") {
              const opts = (f.options || [])
                .map((o) => `<option value="${o}" ${String(o) === String(val) ? "selected" : ""}>${o}</option>`)
                .join("");
              return `<div class="param-field"${showWhen}>
                <span>${f.label}</span>
                <select data-param="${key}" data-kind="select">${opts}</select>
              </div>`;
            }
            if (f.type === "range") {
              return `<div class="param-field${wide}"${showWhen}>
                <span>${f.label}<i class="param-val" data-for="${key}">${val}</i></span>
                <input type="range" data-param="${key}" data-kind="range"
                  min="${f.min ?? 0}" max="${f.max ?? 1}" step="${f.step ?? 0.05}" value="${val}" />
                ${f.hint ? `<small class="param-hint">${f.hint}</small>` : ""}
              </div>`;
            }
            const kind = f.type === "int" ? "int" : "number";
            return `<div class="param-field"${showWhen}>
              <span>${f.label}</span>
              <input type="number" data-param="${key}" data-kind="${kind}"
                min="${f.min ?? ""}" max="${f.max ?? ""}" step="${f.step ?? 1}" value="${val}" />
            </div>`;
          })
          .join("");
        return `<section class="param-group">
          <header class="settings-group-head">
            <div><h3>${g}</h3></div>
            <span class="settings-group-badge">${items.length}</span>
          </header>
          <div class="param-grid settings-grid">${cells}</div>
        </section>`;
      })
      .join("");

    host.querySelectorAll("[data-param]").forEach((el) => {
      el.addEventListener("input", () => {
        if (el.dataset.kind === "range") {
          const label = host.querySelector(`.param-val[data-for="${el.dataset.param}"]`);
          if (label) label.textContent = el.value;
        }
        if (el.dataset.param === "task.mode") applyShowWhen();
        syncHiddenFromParams();
        savePrefs();
      });
      el.addEventListener("change", () => {
        if (el.dataset.param === "task.mode") applyShowWhen();
        syncHiddenFromParams();
        savePrefs();
      });
    });
    applyShowWhen();
    syncHiddenFromParams();
  }

  const MSG_ICONS = {
    pose: '<svg viewBox="0 0 16 16" aria-hidden="true"><circle cx="8" cy="8" r="2.2" fill="currentColor"/><path d="M8 2.2v2.2M8 11.6v2.2M2.2 8h2.2M11.6 8h2.2" stroke="currentColor" stroke-width="1.4" stroke-linecap="round"/><path d="M8 8l3.2-3.2" stroke="currentColor" stroke-width="1.4" stroke-linecap="round"/></svg>',
    twist: '<svg viewBox="0 0 16 16" aria-hidden="true"><path d="M3 11c2.2-4 7.8-4 10 0" fill="none" stroke="currentColor" stroke-width="1.4" stroke-linecap="round"/><path d="M11.2 8.6l1.8 2.6-2.8.2" fill="none" stroke="currentColor" stroke-width="1.4" stroke-linecap="round" stroke-linejoin="round"/><circle cx="4.2" cy="11" r="1.2" fill="currentColor"/></svg>',
    cmd: '<svg viewBox="0 0 16 16" aria-hidden="true"><path d="M3 8h7.5" stroke="currentColor" stroke-width="1.5" stroke-linecap="round"/><path d="M8.8 5.2L12.2 8l-3.4 2.8" fill="none" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/><circle cx="3" cy="8" r="1.3" fill="currentColor"/></svg>',
    scan: '<svg viewBox="0 0 16 16" aria-hidden="true"><path d="M8 12.5a5.2 5.2 0 0 1 0-10.4" fill="none" stroke="currentColor" stroke-width="1.35" stroke-linecap="round"/><path d="M8 12.5a3.2 3.2 0 0 1 0-6.4" fill="none" stroke="currentColor" stroke-width="1.35" stroke-linecap="round"/><circle cx="8" cy="12.5" r="1.2" fill="currentColor"/></svg>',
    image: '<svg viewBox="0 0 16 16" aria-hidden="true"><rect x="2.2" y="3.2" width="11.6" height="9.6" rx="1.4" fill="none" stroke="currentColor" stroke-width="1.35"/><circle cx="5.6" cy="6.4" r="1.1" fill="currentColor"/><path d="M2.8 11.2l3.2-2.8 2.2 1.8 2.4-2.4 2.6 3.4" fill="none" stroke="currentColor" stroke-width="1.2" stroke-linecap="round" stroke-linejoin="round"/></svg>',
    export: '<svg viewBox="0 0 16 16" aria-hidden="true"><path d="M8 2.5v7.2" stroke="currentColor" stroke-width="1.5" stroke-linecap="round"/><path d="M5.2 5.2L8 2.5l2.8 2.7" fill="none" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/><path d="M3 10.5v2.2h10v-2.2" fill="none" stroke="currentColor" stroke-width="1.4" stroke-linecap="round" stroke-linejoin="round"/></svg>',
    msg: '<svg viewBox="0 0 16 16" aria-hidden="true"><rect x="2.5" y="3" width="11" height="10" rx="1.5" fill="none" stroke="currentColor" stroke-width="1.35"/><path d="M5 6.2h6M5 8.5h4.5" stroke="currentColor" stroke-width="1.3" stroke-linecap="round"/></svg>',
  };

  function setChannelAttrs(channelId, on) {
    document
      .querySelectorAll(`#channel-list input[data-attr-channel="${channelId}"]`)
      .forEach((el) => {
        el.checked = !!on;
      });
  }

  function syncChannelCheckbox(channelWrap) {
    const channelBox = channelWrap.querySelector("input[data-channel]");
    const attrs = [...channelWrap.querySelectorAll("input[data-attr]")];
    if (!channelBox || !attrs.length) return;
    const checked = attrs.filter((a) => a.checked).length;
    channelBox.checked = checked === attrs.length;
    channelBox.indeterminate = checked > 0 && checked < attrs.length;
  }

  function bindChannelTree(root) {
    const host = root || $("channel-list");
    if (!host) return;

    host.querySelectorAll(".msg-row").forEach((row) => {
      row.addEventListener("click", (ev) => {
        if (ev.target.closest("input, label.msg-check")) return;
        const wrap = row.closest(".msg-channel");
        if (!wrap) return;
        wrap.classList.toggle("open");
      });
    });

    host.querySelectorAll("input[data-channel]").forEach((box) => {
      box.addEventListener("change", () => {
        setChannelAttrs(box.dataset.channel, box.checked);
        box.indeterminate = false;
      });
    });

    host.querySelectorAll("input[data-attr]").forEach((box) => {
      box.addEventListener("change", () => {
        const wrap = box.closest(".msg-channel");
        if (wrap) syncChannelCheckbox(wrap);
      });
    });
  }

  function setAllChannels(on) {
    const host = $("channel-list");
    if (!host) return;
    host.querySelectorAll("input[data-channel], input[data-attr]").forEach((el) => {
      el.checked = !!on;
      el.indeterminate = false;
    });
  }

  function renderChannelTree(tree) {
    const list = $("channel-list");
    if (!list) return;
    // Accept flat msg list, or legacy grouped tree.
    let channels = tree || [];
    if (channels.length && channels[0]?.children) {
      channels = channels.flatMap((g) => g.children || []);
    }
    if (!channels.length) {
      list.innerHTML = '<div class="empty">无通道定义</div>';
      return;
    }
    const esc = (s) =>
      String(s || "")
        .replace(/&/g, "&amp;")
        .replace(/"/g, "&quot;")
        .replace(/</g, "&lt;");
    list.innerHTML = channels
      .map((ch) => {
        const msg = ch.msg || ch.label || ch.id;
        const icon = MSG_ICONS[ch.icon] || MSG_ICONS.msg;
        const attrs = (ch.attrs || [])
          .map(
            (a) => `<label class="msg-attr" title="${esc(a.type || a.unit || "")}">
              <input class="tree-check" type="checkbox" data-attr="${esc(a.id)}"
                data-attr-channel="${esc(ch.id)}" ${a.enabled || ch.enabled ? "checked" : ""} />
              <span>${esc(a.label)}</span>${a.type ? `<em>${esc(a.type)}</em>` : ""}
            </label>`,
          )
          .join("");
        return `<div class="msg-channel" data-channel-wrap="${esc(ch.id)}">
          <div class="msg-row" role="treeitem" title="${esc(ch.description || msg)}">
            <button type="button" class="msg-twist" aria-label="展开字段" tabindex="-1"></button>
            <span class="msg-icon" data-icon="${esc(ch.icon || "msg")}">${icon}</span>
            <span class="msg-name">${esc(msg)}</span>
            <label class="msg-check" title="录制此消息">
              <input class="tree-check" type="checkbox" data-channel="${esc(ch.id)}"
                ${ch.enabled ? "checked" : ""} />
            </label>
          </div>
          ${attrs ? `<div class="msg-attrs">${attrs}</div>` : ""}
        </div>`;
      })
      .join("");
    bindChannelTree(list);
  }

  function renderCollect(collect) {
    if ($("collect-output")) $("collect-output").value = collect.output || "data/datasets/collect";
    if ($("collect-fps")) $("collect-fps").value = collect.fps ?? 10;
    if ($("collect-frames")) $("collect-frames").value = collect.episode_frames ?? 80;
    if ($("collect-episodes")) $("collect-episodes").value = collect.episodes ?? 3;
    renderChannelTree(collect.channel_tree || []);
  }

  function renderReplay(replay) {
    if ($("replay-source")) {
      $("replay-source").value = replay?.source || "data/datasets/collect";
    }
    if ($("replay-rate")) $("replay-rate").value = replay?.rate ?? 1;
    if ($("replay-loop")) $("replay-loop").checked = !!replay?.loop;
    if ($("replay-dry")) $("replay-dry").checked = !!replay?.dry_run;
  }

  function bindDatasetItems(root) {
    (root || document).querySelectorAll(".dataset-item[data-path]").forEach((el) => {
      const pick = () => {
        if ($("replay-source")) $("replay-source").value = el.dataset.path;
        toast(`已选 ${el.dataset.path}`, "info");
        switchTab("data");
        switchDataMode("play");
      };
      el.addEventListener("click", pick);
      el.addEventListener("keydown", (ev) => {
        if (ev.key === "Enter" || ev.key === " ") {
          ev.preventDefault();
          pick();
        }
      });
    });
  }

  function renderDatasets(datasets) {
    const list = $("dataset-list");
    const datalist = $("dataset-paths");
    if (datalist) {
      datalist.innerHTML = (datasets || [])
        .map((d) => `<option value="${d.path}"></option>`)
        .join("");
    }
    if (!list) return;
    if (!datasets?.length) {
      list.innerHTML = '<li class="empty">尚无数据集 · Record 后出现在这里</li>';
      return;
    }
    list.innerHTML = datasets
      .map(
        (d) => `<li class="dataset-item" data-path="${d.path}" role="button" tabindex="0">
          <code>${d.path}</code><span>${d.episodes} eps</span>
        </li>`,
      )
      .join("");
    bindDatasetItems(list);
  }

  async function refreshDatasets() {
    try {
      const res = await fetch("/api/datasets");
      const data = await res.json();
      renderDatasets(data.datasets || []);
    } catch (err) {
      toast(String(err), "error");
    }
  }

  function refreshCurveTaskSelect() {
    const sel = $("curve-task");
    if (!sel) return;
    const keys = Object.keys(state.curvesByTask);
    sel.innerHTML = keys.map((k) => `<option value="${k}">${k}</option>`).join("");
    if (!keys.length) {
      state.activeCurveTask = "";
      return;
    }
    if (!keys.includes(state.activeCurveTask)) state.activeCurveTask = keys[keys.length - 1];
    sel.value = state.activeCurveTask;
  }

  function selectedSeries() {
    const boxes = [...document.querySelectorAll(".curve-toolbar input[data-series]")];
    if (!boxes.length) return DEFAULT_SERIES;
    const picked = boxes.filter((el) => el.checked).map((el) => el.dataset.series);
    return picked.length ? picked : DEFAULT_SERIES;
  }

  function drawCurves() {
    const canvas = $("curve-canvas");
    const tip = $("curve-tip");
    if (!canvas) return;
    if (state.activeTab !== "curves") return;
    const wrap = canvas.parentElement;
    const ctx = canvas.getContext("2d");
    const dpr = window.devicePixelRatio || 1;
    const cssW = Math.max(320, canvas.clientWidth || wrap?.clientWidth || 900);
    const cssH = Math.max(240, wrap?.clientHeight ? wrap.clientHeight - 8 : 320);
    canvas.width = Math.floor(cssW * dpr);
    canvas.height = Math.floor(cssH * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, cssW, cssH);
    state.plot = null;
    if (tip) tip.hidden = true;

    const pack = state.curvesByTask[state.activeCurveTask];
    const latest = pack?.latest || pack || {};
    const t = latest.t || [];
    const stats = latest.stats || {};
    if ($("curve-stats")) {
      $("curve-stats").innerHTML = [
        ["v", stats.v_max],
        ["a", stats.a_max],
        ["jerk", stats.jerk_max],
      ]
        .map(([k, v]) => {
          const val = typeof v === "number" ? v.toFixed(2) : "—";
          return `<span class="curve-stat">${k} <b>${val}</b></span>`;
        })
        .join("");
    }

    if (t.length < 2) {
      ctx.fillStyle = "#6b7884";
      ctx.font = "500 13px IBM Plex Sans, sans-serif";
      ctx.fillText("运行后显示曲线", 20, cssH / 2);
      return;
    }

    const pad = { l: 40, r: 12, t: 10, b: 22 };
    const plotW = cssW - pad.l - pad.r;
    const plotH = cssH - pad.t - pad.b;
    const t0 = t[0];
    const t1 = t[t.length - 1] || 1;
    const seriesKeys = selectedSeries().filter((k) => Array.isArray(latest[k]));
    let ymin = 0;
    let ymax = 0;
    for (const key of seriesKeys) {
      for (const v of latest[key]) {
        ymin = Math.min(ymin, v);
        ymax = Math.max(ymax, v);
      }
    }
    if (Math.abs(ymax - ymin) < 1e-6) {
      ymax += 1;
      ymin -= 1;
    }
    const xOf = (ti) => pad.l + ((ti - t0) / (t1 - t0 || 1)) * plotW;
    const yOf = (v) => pad.t + (1 - (v - ymin) / (ymax - ymin)) * plotH;

    ctx.strokeStyle = "#2a343e";
    ctx.beginPath();
    ctx.moveTo(pad.l, pad.t);
    ctx.lineTo(pad.l, pad.t + plotH);
    ctx.lineTo(pad.l + plotW, pad.t + plotH);
    ctx.stroke();

    for (const key of seriesKeys) {
      const ys = latest[key];
      ctx.strokeStyle = SERIES_COLORS[key] || "#fff";
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      for (let i = 0; i < t.length; i++) {
        const x = xOf(t[i]);
        const y = yOf(ys[i]);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }

    ctx.fillStyle = "#6b7884";
    ctx.font = "500 10px IBM Plex Mono, monospace";
    ctx.fillText(`${t0.toFixed(1)}s`, pad.l, cssH - 6);
    ctx.fillText(`${t1.toFixed(1)}s`, pad.l + plotW - 36, cssH - 6);

    state.plot = { pad, plotW, plotH, t0, t1, t, latest, seriesKeys, xOf, yOf, cssW, cssH };
  }

  function onCurveMove(ev) {
    const tip = $("curve-tip");
    const canvas = $("curve-canvas");
    const plot = state.plot;
    if (!tip || !canvas || !plot || !plot.t.length) {
      if (tip) tip.hidden = true;
      return;
    }
    const rect = canvas.getBoundingClientRect();
    const x = ev.clientX - rect.left;
    if (x < plot.pad.l || x > plot.pad.l + plot.plotW) {
      tip.hidden = true;
      return;
    }
    const ratio = (x - plot.pad.l) / plot.plotW;
    const ti = plot.t0 + ratio * (plot.t1 - plot.t0 || 1);
    let best = 0;
    let bestDist = Infinity;
    for (let i = 0; i < plot.t.length; i++) {
      const d = Math.abs(plot.t[i] - ti);
      if (d < bestDist) {
        bestDist = d;
        best = i;
      }
    }
    const lines = [`t=${plot.t[best].toFixed(2)}s`];
    for (const key of plot.seriesKeys) {
      const v = plot.latest[key]?.[best];
      lines.push(`${key}=${typeof v === "number" ? v.toFixed(2) : "—"}`);
    }
    tip.hidden = false;
    tip.textContent = lines.join(" · ");
    tip.style.left = `${Math.min(rect.width - 12, Math.max(12, x + 8))}px`;
    tip.style.top = `${Math.max(6, ev.clientY - rect.top - 24)}px`;
  }

  async function loadConfigText(path) {
    const res = await fetch(`/api/config?path=${encodeURIComponent(path)}`);
    const data = await res.json();
    if ($("yaml-view")) {
      $("yaml-view").textContent = data.error || data.text || "";
    }
  }

  function applyCatalog(data, path) {
    state.catalog = data;
    state.configPath =
      path ||
      data.config_path ||
      "configs/default.yaml";
    loadNamedRecent();
    loadNamedFavs();
    loadNamedCallHistory();

    const prefs = loadPrefs();
    if (!state.orchSteps.length) {
      if (Array.isArray(prefs?.orchSteps) && prefs.orchSteps.length) {
        state.orchSteps = prefs.orchSteps
          .filter((s) => s && typeof s.task === "string")
          .map((s) => ({
            task: s.task,
            episodes: Math.max(1, Math.min(200, Number(s.episodes) || 1)),
          }));
      } else if (prefs?.tasks?.length) {
        const defEp = Number(prefs.orchEpisodes) || 3;
        state.orchSteps = prefs.tasks
          .filter((id) => typeof id === "string")
          .map((id) => ({ task: id, episodes: defEp }));
      } else {
        const registered = (data.missions || []).filter((m) => m.registered);
        if (registered[0]) {
          state.orchSteps = [
            {
              task: registered[0].id,
              episodes: Number(registered[0].default_episodes || registered[0].episodes || 3) || 3,
            },
          ];
        }
      }
    }
    if (prefs?.orchExecution && $("orch-execution")) $("orch-execution").value = prefs.orchExecution;
    if (prefs?.orchEpisodes != null && $("orch-episodes")) $("orch-episodes").value = prefs.orchEpisodes;
    if (prefs?.orchStepInterval != null && $("orch-step-interval")) {
      $("orch-step-interval").value = prefs.orchStepInterval;
    }
    if (prefs?.orchRepeat != null && $("orch-repeat")) $("orch-repeat").value = prefs.orchRepeat;
    if (typeof prefs?.record === "boolean" && $("record-enable")) {
      $("record-enable").checked = prefs.record;
    }

    renderConfigs(data.configs || []);
    renderMissions(data.missions || []);
    renderOrchSteps();
    renderNamedTasks(data.named_tasks || []);
    const hadSched = loadSchedSteps();
    if (!hadSched && !state.scheduleSteps.length && (data.missions || []).some((m) => m.registered)) {
      const first = (data.missions || []).find((m) => m.registered);
      if (first) state.scheduleSteps = [{ task: first.id }];
    }
    renderSchedMissions();
    renderSchedSteps();
    renderSelect("simulator-select", data.registry?.simulators || [], data.scene?.simulator);
    renderSelect("humans-select", data.registry?.humans || [], data.scene?.humans);
    renderParams(data.params || {});
    // Restore saved param overrides only for the same config file.
    if (
      prefs?.params &&
      typeof prefs.params === "object" &&
      prefs.config === state.configPath
    ) {
      const host = $("param-fields");
      Object.entries(prefs.params).forEach(([key, val]) => {
        const el = host?.querySelector(`[data-param="${key}"]`);
        if (!el) return;
        if (el.dataset.kind === "bool") el.checked = !!val;
        else el.value = val;
        if (el.dataset.kind === "range") {
          const label = host.querySelector(`.param-val[data-for="${key}"]`);
          if (label) label.textContent = String(val);
        }
      });
      applyShowWhen();
      syncHiddenFromParams();
    }
    renderCollect(data.collect || {});
    renderReplay(data.replay || {});
    renderDatasets(data.datasets || []);
    if (typeof data.config_text === "string" && data.config_text.length && $("yaml-view")) {
      $("yaml-view").textContent = data.config_text;
    }
    syncSettingsSide();
    drawCurves();
  }

  function resetParams() {
    const defaults = state.paramDefaults || state.catalog?.params?.values || {};
    renderParams({
      fields: state.catalog?.params?.fields || [],
      values: defaults,
    });
    savePrefs();
    toast("参数已重置", "info");
  }

  async function loadCatalog(path) {
    const q = path ? `?config=${encodeURIComponent(path)}` : "";
    const res = await fetch(`/api/catalog${q}`);
    const data = await res.json();
    applyCatalog(data, path);
    if (!data.config_text) {
      await loadConfigText(state.configPath);
    }
  }

  function bootstrapFromSSR() {
    const node = document.getElementById("bootstrap-catalog");
    if (!node?.textContent) return false;
    try {
      applyCatalog(JSON.parse(node.textContent), null);
      return true;
    } catch {
      return false;
    }
  }

  function collectPayload() {
    const channels = {};
    document.querySelectorAll("#channel-list input[data-channel]").forEach((el) => {
      // Channel on if checked or indeterminate (partial attrs).
      channels[el.dataset.channel] = !!(el.checked || el.indeterminate);
    });
    const attrs = {};
    document.querySelectorAll("#channel-list input[data-attr]").forEach((el) => {
      const ch = el.dataset.attrChannel;
      if (!attrs[ch]) attrs[ch] = [];
      if (el.checked) attrs[ch].push(el.dataset.attr);
    });
    return {
      output: $("collect-output")?.value || "data/datasets/collect",
      fps: Number($("collect-fps")?.value || 10),
      episode_frames: Number($("collect-frames")?.value || 80),
      episodes: Number($("collect-episodes")?.value || 3),
      pose: !!channels.pose,
      velocity: !!channels.velocity,
      action: !!channels.action,
      include_laser: !!channels.include_laser,
      include_image: !!channels.include_image,
      lerobot: !!channels.lerobot,
      attrs,
    };
  }

  function replayPayload() {
    return {
      source: $("replay-source")?.value || "data/datasets/collect",
      rate: Number($("replay-rate")?.value || 1),
      loop: !!$("replay-loop")?.checked,
      dry_run: !!$("replay-dry")?.checked,
    };
  }

  async function postRun(body) {
    if (state.running) {
      toast("已在运行", "warn");
      return null;
    }
    setRunning(true);
    const res = await fetch("/api/run", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const data = await res.json();
    if (!data.ok) {
      setRunning(false);
      toast(data.error || "启动失败", "error");
      return null;
    }
    toast("已排队", "ok");
    pushEvent({
      ts: Date.now() / 1000,
      kind: "info",
      message: `${data.job.job_id}: ${data.job.tasks.join(", ")}`,
    });
    return data;
  }

  function buildNamedCalls() {
    const catalog = state.catalog?.named_tasks || [];
    // Keep form fields → JSON before send.
    if (state.namedFocus) syncNamedBodyFromForm();
    const focusBody = readNamedBody();
    const target = ($("named-target")?.value || "").trim();
    return [...state.namedSelected].map((id) => {
      const item = catalog.find((x) => x.id === id) || {};
      const data =
        id === state.namedFocus && focusBody && typeof focusBody === "object"
          ? focusBody
          : item.default_body || {};
      return {
        id,
        method: item.method,
        rpc: item.rpc || `${item.service}/${item.method}`,
        data,
        target: target || undefined,
      };
    });
  }

  async function runSelected() {
    savePrefs();
    syncHiddenFromParams();
    const params = collectParams();
    const kind = state.taskKind || "orchestration";
    const base = {
      config: state.configPath,
      mode: params["task.mode"] || $("mode-select")?.value,
      simulator: $("simulator-select")?.value || undefined,
      humans: $("humans-select")?.value || undefined,
      record: !!$("record-enable")?.checked,
      collect: collectPayload(),
      replay: replayPayload(),
      params,
      kind,
    };

    if (kind === "named") {
      const named_calls = buildNamedCalls();
      if (!named_calls.length) {
        toast("请先选择一条命令", "warn");
        return;
      }
      if (focusBodyInvalid()) {
        toast("请求 JSON 无效，请检查发布参数", "error");
        return;
      }
      if (state.namedSending) {
        toast("发送中，请稍候", "warn");
        return;
      }
      const call0 = named_calls[0];
      const label = namedItemById(call0?.id)?.method || namedItemById(call0?.id)?.label || "命令";
      state._namedPending = {
        id: call0?.id,
        method: call0?.method,
        rpc: call0?.rpc,
        body: call0?.data || {},
      };
      setNamedSending(true);
      setNamedResponse(
        { label: "发送中…", kind: "pending" },
        { request: call0?.data || {}, rpc: call0?.rpc },
      );
      toast(`正在发送：${label}`, "info");
      try {
        await postRun({
          ...base,
          tasks: named_calls.map((c) => c.id),
          named_calls,
          execution: $("named-execution")?.value || "sync",
          record: false,
        });
      } catch (err) {
        setNamedSending(false);
        setNamedResponse({ label: "失败", kind: "fail" }, { error: String(err) });
        pushNamedCallHistory({
          ...(state._namedPending || {}),
          ok: false,
          error: String(err),
          at: Date.now(),
        });
      }
      return;
    }

    if (kind === "schedule") {
      const steps = state.scheduleSteps
        .map((s) => ({ task: s.task }))
        .filter((s) => s.task);
      if (!steps.length) {
        toast("请添加定时步骤", "warn");
        return;
      }
      const registered = new Set(
        (state.catalog?.missions || []).filter((m) => m.registered).map((m) => m.id),
      );
      const bad = steps.filter((s) => !registered.has(s.task));
      if (bad.length) {
        toast(`含未注册任务: ${bad.map((b) => b.task).join(", ")}`, "warn");
        return;
      }
      const loopForever = !!$("sched-loop")?.checked;
      const startAt = $("sched-start-at")?.value || "";
      const endAt = $("sched-end-at")?.value || "";
      if (startAt && endAt && endAt <= startAt) {
        toast("结束时间必须晚于开始时间", "warn");
        return;
      }
      await postRun({
        ...base,
        tasks: steps.map((s) => s.task),
        episodes: Number($("sched-episodes")?.value || 1),
        execution: $("sched-execution")?.value || "sync",
        schedule: {
          steps,
          interval_s: Number($("sched-interval")?.value || 0),
          repeat: loopForever ? 1 : Number($("sched-repeat")?.value || 1),
          delay_s: Number($("sched-delay")?.value || 0),
          start_at: startAt,
          end_at: endAt,
          loop: loopForever,
        },
      });
      return;
    }

    const steps = state.orchSteps
      .map((s) => ({
        task: s.task,
        episodes: Math.max(1, Math.min(200, Number(s.episodes) || 1)),
      }))
      .filter((s) => s.task);
    if (!steps.length) {
      toast("请添加编排步骤", "warn");
      return;
    }
    const registered = new Set(
      (state.catalog?.missions || []).filter((m) => m.registered).map((m) => m.id),
    );
    const bad = steps.filter((s) => !registered.has(s.task));
    if (bad.length) {
      toast(`含未注册任务: ${bad.map((b) => b.task).join(", ")}`, "warn");
      return;
    }
    const fallbackEp = Number(
      $("orch-episodes")?.value || params["task.episodes"] || $("episodes-input")?.value || 3,
    );
    if ($("episodes-input")) $("episodes-input").value = String(fallbackEp);
    await postRun({
      ...base,
      tasks: steps.map((s) => s.task),
      episodes: fallbackEp,
      execution: $("orch-execution")?.value || "sync",
      schedule: {
        steps,
        step_interval_s: Number($("orch-step-interval")?.value || 0),
        repeat: Number($("orch-repeat")?.value || 1),
        interval_s: 0,
      },
    });
  }

  function focusBodyInvalid() {
    try {
      JSON.parse($("named-body")?.value || "{}");
      return false;
    } catch {
      return true;
    }
  }

  async function runRecordOnly() {
    savePrefs();
    const collect = collectPayload();
    await postRun({
      tasks: ["collect"],
      config: state.configPath,
      episodes: collect.episodes,
      simulator: $("simulator-select")?.value || undefined,
      humans: $("humans-select")?.value || undefined,
      record: false,
      collect,
      params: collectParams(),
    });
  }

  async function runPlayOnly() {
    savePrefs();
    const replay = replayPayload();
    if (!replay.source) {
      toast("请填写数据源", "warn");
      return;
    }
    await postRun({
      tasks: ["replay"],
      config: state.configPath,
      simulator: $("simulator-select")?.value || undefined,
      replay,
      params: collectParams(),
    });
  }

  function scheduleSSEReconnect() {
    if (state.sseTimer) return;
    setConn("reconnecting");
    const delay = Math.min(12000, 600 * 2 ** Math.min(state.sseRetry, 4));
    state.sseRetry += 1;
    state.sseTimer = setTimeout(() => {
      state.sseTimer = null;
      connectSSE();
    }, delay);
  }

  function connectSSE() {
    if (state.sse) {
      try {
        state.sse.close();
      } catch {
        /* ignore */
      }
    }
    const es = new EventSource("/api/events");
    state.sse = es;
    es.onopen = () => {
      state.sseRetry = 0;
      setConn("live");
    };
    es.onerror = () => {
      try {
        es.close();
      } catch {
        /* ignore */
      }
      if (state.sse === es) state.sse = null;
      scheduleSSEReconnect();
    };
    es.onmessage = (msg) => {
      let ev;
      try {
        ev = JSON.parse(msg.data);
      } catch {
        return;
      }
      if (ev.kind === "snapshot" || ev.kind === "heartbeat") {
        renderMetrics(ev.payload?.metrics);
        renderJob(ev.payload?.job);
        if (ev.payload?.history) renderHistory(ev.payload.history);
        return;
      }
      pushEvent(ev);
      if (ev.kind === "task_start" || ev.kind === "task_done") {
        if (ev.payload?.task_states) {
          renderTaskProgress({
            tasks: Object.keys(ev.payload.task_states),
            task_states: ev.payload.task_states,
          });
        }
      }
      if (ev.kind === "task_done") {
        if (ev.payload?.curves) {
          state.curvesByTask[ev.payload.task] = { latest: ev.payload.curves };
          state.activeCurveTask = ev.payload.task;
          refreshCurveTaskSelect();
          markCurvesReady();
          drawCurves();
        }
        if (ev.payload?.map_track) {
          pushMapTrack(ev.payload.task, ev.payload.map_track);
          window.BenchmarkMap?.fit?.();
        }
        const result = ev.payload?.result;
        if (result && (state.taskKind === "named" || state.activeTab === "named")) {
          const ok = !!result.ok;
          setNamedSending(false);
          setNamedResponse(
            {
              label: ok
                ? `成功 · ${result.elapsed_s ?? "?"}s`
                : `失败 · ${result.elapsed_s ?? "?"}s`,
              kind: ok ? "ok" : "fail",
            },
            {
              method: result.method,
              ok,
              error: result.error || undefined,
              elapsed_s: result.elapsed_s,
              response: result.response ?? {},
            },
          );
          const pending = state._namedPending || {};
          pushNamedCallHistory({
            id: pending.id || result.id || state.namedFocus,
            method: result.method || pending.method,
            rpc: result.method || pending.rpc,
            body: pending.body || {},
            ok,
            error: result.error || undefined,
            response: result.response ?? {},
            at: Date.now(),
          });
          state._namedPending = null;
        }
      }
      if (ev.kind === "job_done") {
        toast("完成", "ok");
        if (state.namedSending) setNamedSending(false);
        if (ev.payload?.curves) {
          state.curvesByTask = ev.payload.curves;
          markCurvesReady();
        }
        if (ev.payload?.results) {
          $("summary-view").textContent = JSON.stringify(ev.payload.results, null, 2);
          renderResultCards(ev.payload.results);
          for (const [task, block] of Object.entries(ev.payload.results)) {
            if (block.map_track) pushMapTrack(task, block.map_track);
          }
          window.BenchmarkMap?.fit?.();
        }
        if (ev.payload?.history) renderHistory(ev.payload.history);
        refreshCurveTaskSelect();
        drawCurves();
        refreshDatasets();
      }
      if (ev.kind === "job_failed") {
        toast(ev.message || "失败", "error");
        if (state.namedSending || state._namedPending) {
          setNamedSending(false);
          setNamedResponse(
            { label: "失败", kind: "fail" },
            { error: ev.message || "job failed" },
          );
          if (state._namedPending) {
            pushNamedCallHistory({
              ...state._namedPending,
              ok: false,
              error: ev.message || "job failed",
              at: Date.now(),
            });
            state._namedPending = null;
          }
        }
      }
      if (ev.kind === "job_cancelled") {
        toast("已取消", "warn");
        if (state.namedSending) setNamedSending(false);
      }
      fetch("/api/status")
        .then((r) => r.json())
        .then((s) => {
          renderMetrics(s.metrics);
          renderJob(s.job);
          if (s.history) renderHistory(s.history);
        })
        .catch(() => {});
    };
  }

  $("config-select")?.addEventListener("change", async (e) => {
    state.configPath = e.target.value;
    state.orchSteps = [];
    syncSettingsSide();
    savePrefs();
    await loadCatalog(state.configPath);
  });

  $("simulator-select")?.addEventListener("change", () => {
    syncSettingsSide();
    savePrefs();
  });
  $("humans-select")?.addEventListener("change", () => {
    syncSettingsSide();
    savePrefs();
  });

  $("record-enable")?.addEventListener("change", savePrefs);

  $("btn-params-reset")?.addEventListener("click", resetParams);

  $("btn-record-run")?.addEventListener("click", runRecordOnly);
  $("btn-play-run")?.addEventListener("click", runPlayOnly);
  $("btn-refresh-datasets")?.addEventListener("click", refreshDatasets);
  $("btn-ch-all")?.addEventListener("click", () => setAllChannels(true));
  $("btn-ch-none")?.addEventListener("click", () => setAllChannels(false));
  document.querySelectorAll(".data-mode-btn").forEach((btn) => {
    btn.addEventListener("click", () => switchDataMode(btn.dataset.dataMode));
  });
  $("orch-filter")?.addEventListener("input", (e) => {
    renderMissions(state.catalog?.missions || [], e.target.value || "");
  });
  $("btn-orch-all")?.addEventListener("click", () => {
    const missions = (state.catalog?.missions || []).filter((m) => m.registered !== false);
    if (!missions.length) {
      toast("无可用任务", "warn");
      return;
    }
    for (const m of missions) {
      state.orchSteps.push({ task: m.id, episodes: defaultOrchEpisodes(m) });
    }
    renderOrchSteps();
    renderMissions();
    savePrefs();
  });
  $("btn-orch-none")?.addEventListener("click", () => {
    state.orchSteps = [];
    renderOrchSteps();
    renderMissions();
    savePrefs();
  });
  ["orch-execution", "orch-episodes", "orch-step-interval", "orch-repeat"].forEach((id) => {
    $(id)?.addEventListener("change", () => savePrefs());
  });

  $("btn-sched-add")?.addEventListener("click", () => {
    const missions = (state.catalog?.missions || []).filter((m) => m.registered !== false);
    const id = missions[0]?.id || state.catalog?.missions?.[0]?.id;
    if (!id) {
      toast("无可用任务", "warn");
      return;
    }
    addSchedStep(id);
  });
  $("btn-sched-clear")?.addEventListener("click", () => {
    state.scheduleSteps = [];
    renderSchedSteps();
    saveSchedSteps();
  });
  $("sched-filter")?.addEventListener("input", (e) => {
    renderSchedMissions(e.target.value || "");
  });
  ["sched-execution", "sched-interval", "sched-repeat", "sched-episodes", "sched-delay", "sched-start-at", "sched-end-at"].forEach(
    (id) => {
      $(id)?.addEventListener("change", () => saveSchedSteps());
    },
  );
  $("sched-loop")?.addEventListener("change", () => {
    syncSchedLoopUi();
    saveSchedSteps();
  });
  syncSchedLoopUi();
  bindDatasetItems(document);
  switchDataMode(state.dataMode);

  $("btn-ota-refresh")?.addEventListener("click", () => refreshOtaSnapshot());
  $("btn-ota-start")?.addEventListener("click", runOtaStart);
  $("btn-ota-abort")?.addEventListener("click", runOtaAbort);
  $("btn-ota-rpc")?.addEventListener("click", runOtaRpc);

  $("scene-select")?.addEventListener("change", (e) => {
    const id = e.target.value;
    window.BenchmarkMap?.selectScene?.(id);
  });

  $("btn-run")?.addEventListener("click", runSelected);
  function sendNamedCommand() {
    if (state.taskKind !== "named") state.taskKind = "named";
    if (!state.namedFocus) {
      toast("请先选择指令", "warn");
      return;
    }
    if (state.namedSending) {
      toast("发送中，请稍候", "warn");
      return;
    }
    runSelected();
  }
  $("btn-named-send")?.addEventListener("click", sendNamedCommand);

  /** Map Goal Pose → fill 命令 / Navigate waypoints (and orientation). */
  function yawToQuatJs(yaw) {
    const half = Number(yaw || 0) * 0.5;
    return { x: 0, y: 0, z: Math.sin(half), w: Math.cos(half) };
  }
  function applyMapGoalToNavigate(detail) {
    const x = Number(detail?.x);
    const y = Number(detail?.y);
    const yaw = Number(detail?.yaw || 0);
    if (!Number.isFinite(x) || !Number.isFinite(y)) return;
    const item = namedItemById("rpc.navigate");
    if (!item) return;
    const base =
      state.namedFocus === "rpc.navigate"
        ? readNamedBody() || {}
        : JSON.parse(JSON.stringify(item.default_body || {}));
    setByPath(base, "waypoints.0.pose.position.x", x);
    setByPath(base, "waypoints.0.pose.position.y", y);
    setByPath(base, "waypoints.0.pose.position.z", 0);
    setByPath(base, "waypoints.0.pose.orientation", yawToQuatJs(yaw));
    if (!getByPath(base, "waypoints.0.header.frame_id")) {
      setByPath(base, "waypoints.0.header.frame_id", "map");
    }
    state.mapGoalBody = base;
    if (state.namedFocus === "rpc.navigate") {
      writeNamedBody(base);
      syncNamedFormFromBody();
    }
  }
  window.addEventListener("benchmark:map-goal", (ev) => {
    applyMapGoalToNavigate(ev.detail || {});
  });
  $("btn-named-copy")?.addEventListener("click", async () => {
    const text = $("named-response")?.textContent || "";
    if (!text.trim()) {
      toast("无可复制内容", "warn");
      return;
    }
    try {
      await navigator.clipboard.writeText(text);
      toast("已复制响应", "ok");
    } catch {
      toast("复制失败", "error");
    }
  });
  $("btn-named-reset")?.addEventListener("click", () => {
    const item = namedItemById(state.namedFocus);
    if (!item) return;
    fillNamedEditor(item, { resetBody: true });
    toast("已填充默认参数", "info");
  });
  $("btn-named-link")?.addEventListener("click", (e) => {
    e.preventDefault();
    refreshNamedLink();
  });
  let namedTargetTimer = null;
  $("named-target")?.addEventListener("input", () => {
    if (namedTargetTimer) clearTimeout(namedTargetTimer);
    namedTargetTimer = setTimeout(() => {
      if (state.activeTab === "named") refreshNamedLink();
    }, 450);
  });
  $("named-target")?.addEventListener("change", () => refreshNamedLink());
  function placeNamedConnPanel() {
    const conn = $("named-conn");
    const panel = conn?.querySelector(".cmd-conn-panel");
    const anchor = $("named-link");
    if (!conn?.open || !panel || !anchor) return;
    const r = anchor.getBoundingClientRect();
    const gap = 8;
    const w = Math.min(264, window.innerWidth - 16);
    // Prefer open to the right into the detail pane; fall back left/below.
    let left = r.right + gap;
    if (left + w > window.innerWidth - 8) {
      left = Math.max(8, r.left - w - gap);
    }
    panel.style.width = `${w}px`;
    panel.style.visibility = "hidden";
    panel.style.left = `${left}px`;
    panel.style.top = `${r.bottom + gap}px`;
    const h = panel.getBoundingClientRect().height || 180;
    let top = r.bottom + gap;
    if (top + h > window.innerHeight - 8) {
      top = Math.max(8, r.top - h - gap);
    }
    panel.style.top = `${top}px`;
    panel.style.right = "auto";
    panel.style.visibility = "";
  }
  // 点击外部关闭连接浮层
  document.addEventListener("click", (e) => {
    const conn = $("named-conn");
    if (!conn?.open) return;
    if (!conn.contains(e.target)) conn.open = false;
  });
  $("named-conn")?.addEventListener("toggle", () => {
    if ($("named-conn")?.open) {
      placeNamedConnPanel();
      refreshNamedLink();
      requestAnimationFrame(placeNamedConnPanel);
    }
  });
  window.addEventListener("resize", () => {
    if ($("named-conn")?.open) placeNamedConnPanel();
  });
  document.addEventListener(
    "scroll",
    () => {
      if ($("named-conn")?.open) placeNamedConnPanel();
    },
    true,
  );
  $("named-body")?.addEventListener("change", () => syncNamedFormFromBody());
  $("named-body")?.addEventListener("blur", () => syncNamedFormFromBody());
  $("named-body")?.addEventListener("input", () => fitNamedBody());
  $("named-advanced")?.addEventListener("toggle", () => {
    if ($("named-advanced")?.open) requestAnimationFrame(fitNamedBody);
  });
  $("named-filter")?.addEventListener("input", (e) => {
    renderNamedTasks(namedCatalog(), e.target.value || "");
  });
  $("named-filter")?.addEventListener("keydown", (e) => {
    if (e.key === "Escape") {
      e.target.value = "";
      renderNamedTasks();
    }
  });
  // / 聚焦搜索；Enter 发送；↑↓ 在当前列表切换
  document.addEventListener("keydown", (e) => {
    if (state.activeTab !== "named") return;
    const tag = (e.target?.tagName || "").toLowerCase();
    const typing =
      tag === "input" || tag === "textarea" || tag === "select" || e.target?.isContentEditable;
    if (e.key === "/" && !typing && !e.metaKey && !e.ctrlKey && !e.altKey) {
      e.preventDefault();
      $("named-filter")?.focus();
      $("named-filter")?.select();
    }
  });
  $("tab-named")?.addEventListener("keydown", (e) => {
    if (state.activeTab !== "named") return;
    const tag = (e.target?.tagName || "").toLowerCase();
    const inTextarea = tag === "textarea";
    const inSearch = e.target?.id === "named-filter";
    const inGroupTab = e.target?.classList?.contains("cmd-group-tab");
    if (e.key === "Enter" && !e.shiftKey && !inTextarea && tag !== "button") {
      e.preventDefault();
      sendNamedCommand();
      return;
    }
    if (
      (e.key === "ArrowDown" || e.key === "ArrowUp") &&
      (inSearch || inGroupTab || (tag === "button" && e.target?.classList?.contains("named-item")))
    ) {
      const ids = [...document.querySelectorAll("#named-task-list .named-item")].map(
        (el) => el.dataset.namedId,
      );
      if (!ids.length) return;
      const cur = Math.max(0, ids.indexOf(state.namedFocus));
      const next =
        e.key === "ArrowDown"
          ? ids[Math.min(ids.length - 1, cur + 1)]
          : ids[Math.max(0, cur - 1)];
      if (next && next !== state.namedFocus) {
        e.preventDefault();
        selectNamedTask(next);
        document
          .querySelector(`#named-task-list .named-item[data-named-id="${next}"]`)
          ?.focus();
      }
    }
  });
  $("btn-cancel")?.addEventListener("click", async () => {
    const res = await fetch("/api/cancel", { method: "POST" });
    const data = await res.json().catch(() => ({}));
    toast(data.ok ? "已取消" : "无可取消", data.ok ? "warn" : "info");
  });

  $("curve-task")?.addEventListener("change", (e) => {
    state.activeCurveTask = e.target.value;
    updateDownloadLinks({
      job_id: $("live-job")?.textContent !== "—" ? $("live-job").textContent : "",
    });
    drawCurves();
  });

  document.querySelectorAll(".side-nav-btn").forEach((btn) => {
    btn.addEventListener("click", () => switchNav(btn.dataset.nav));
  });
  document.querySelectorAll(".stage-tabs .tab").forEach((btn) => {
    btn.addEventListener("click", () => switchTab(btn.dataset.tab));
  });
  document.querySelectorAll("[data-tab-jump]").forEach((btn) => {
    btn.addEventListener("click", () => switchTab(btn.dataset.tabJump));
  });
  document.querySelectorAll("[data-data-jump]").forEach((btn) => {
    btn.addEventListener("click", () => {
      switchNav("data");
      switchDataMode(btn.dataset.dataJump);
    });
  });
  switchNav(state.activeNav, { tab: state.activeTab });

  const canvas = $("curve-canvas");
  canvas?.addEventListener("mousemove", onCurveMove);
  canvas?.addEventListener("mouseleave", () => {
    if ($("curve-tip")) $("curve-tip").hidden = true;
  });
  window.addEventListener("resize", () => {
    if (state.activeTab === "curves") drawCurves();
  });

  window.addEventListener("keydown", (ev) => {
    const tag = (ev.target?.tagName || "").toLowerCase();
    if (tag === "input" || tag === "textarea" || tag === "select") return;
    if (ev.key === "Escape") {
      $("btn-cancel")?.click();
      return;
    }
    if ((ev.key === "r" || ev.key === "R") && !ev.metaKey && !ev.ctrlKey) {
      ev.preventDefault();
      runSelected();
    }
    if (ev.key === "1") switchNav("settings");
    if (ev.key === "2") switchNav("tasks");
    if (ev.key === "3") switchNav("data");
    if (ev.key === "4") switchNav("test");
    if (ev.key === "5") switchNav("ops");
  });

  function clampNamedPickerW(px) {
    const body = $("cmd-body");
    const maxByParent = body
      ? Math.max(NAMED_PICKER_W_MIN, Math.floor(body.getBoundingClientRect().width * 0.55))
      : NAMED_PICKER_W_MAX;
    return Math.round(
      Math.min(NAMED_PICKER_W_MAX, maxByParent, Math.max(NAMED_PICKER_W_MIN, px)),
    );
  }

  function applyNamedPickerWidth(px, { persist = false } = {}) {
    const body = $("cmd-body");
    if (!body) return;
    const w = clampNamedPickerW(px);
    body.style.setProperty("--cmd-picker-w", `${w}px`);
    const handle = $("cmd-resize");
    if (handle) {
      handle.setAttribute("aria-valuenow", String(w));
      handle.setAttribute("aria-valuemin", String(NAMED_PICKER_W_MIN));
      handle.setAttribute("aria-valuemax", String(NAMED_PICKER_W_MAX));
    }
    if (persist) {
      try {
        localStorage.setItem(NAMED_PICKER_W_KEY, String(w));
      } catch {
        /* ignore */
      }
    }
  }

  function initNamedPickerResize() {
    const body = $("cmd-body");
    const handle = $("cmd-resize");
    if (!body || !handle) return;

    let saved = NAMED_PICKER_W_DEFAULT;
    try {
      const raw = Number(localStorage.getItem(NAMED_PICKER_W_KEY));
      if (Number.isFinite(raw) && raw > 0) saved = raw;
    } catch {
      /* ignore */
    }
    applyNamedPickerWidth(saved);

    let dragging = false;
    let startX = 0;
    let startW = 0;

    const onMove = (clientX) => {
      if (!dragging) return;
      applyNamedPickerWidth(startW + (clientX - startX));
    };
    const onUp = () => {
      if (!dragging) return;
      dragging = false;
      body.classList.remove("is-resizing");
      document.body.style.removeProperty("cursor");
      const cur = parseFloat(getComputedStyle(body).getPropertyValue("--cmd-picker-w"));
      if (Number.isFinite(cur)) applyNamedPickerWidth(cur, { persist: true });
    };

    handle.addEventListener("pointerdown", (e) => {
      if (e.button != null && e.button !== 0) return;
      e.preventDefault();
      dragging = true;
      startX = e.clientX;
      startW =
        parseFloat(getComputedStyle(body).getPropertyValue("--cmd-picker-w")) ||
        body.querySelector(".cmd-picker")?.getBoundingClientRect().width ||
        NAMED_PICKER_W_DEFAULT;
      body.classList.add("is-resizing");
      document.body.style.cursor = "col-resize";
      handle.setPointerCapture?.(e.pointerId);
    });
    handle.addEventListener("pointermove", (e) => onMove(e.clientX));
    handle.addEventListener("pointerup", onUp);
    handle.addEventListener("pointercancel", onUp);
    handle.addEventListener("lostpointercapture", onUp);

    handle.addEventListener("keydown", (e) => {
      const step = e.shiftKey ? 32 : 12;
      const cur =
        parseFloat(getComputedStyle(body).getPropertyValue("--cmd-picker-w")) ||
        NAMED_PICKER_W_DEFAULT;
      if (e.key === "ArrowLeft") {
        e.preventDefault();
        applyNamedPickerWidth(cur - step, { persist: true });
      } else if (e.key === "ArrowRight") {
        e.preventDefault();
        applyNamedPickerWidth(cur + step, { persist: true });
      } else if (e.key === "Home") {
        e.preventDefault();
        applyNamedPickerWidth(NAMED_PICKER_W_MIN, { persist: true });
      } else if (e.key === "End") {
        e.preventDefault();
        applyNamedPickerWidth(NAMED_PICKER_W_MAX, { persist: true });
      }
    });

    window.addEventListener("resize", () => {
      const cur =
        parseFloat(getComputedStyle(body).getPropertyValue("--cmd-picker-w")) ||
        NAMED_PICKER_W_DEFAULT;
      applyNamedPickerWidth(cur);
    });
  }

  initNamedPickerResize();
  initSchedPickerResize();
  initOrchPickerResize();

  const boot = bootstrapFromSSR() ? Promise.resolve() : loadCatalog();
  boot
    .then(() => {
      connectSSE();
      return fetch("/api/status")
        .then((r) => r.json())
        .then((s) => {
          renderMetrics(s.metrics);
          renderJob(s.job);
          if (s.history) renderHistory(s.history);
        })
        .catch(() => {});
    })
    .catch((err) => {
      setConn(false);
      toast(String(err), "error");
    });
})();
