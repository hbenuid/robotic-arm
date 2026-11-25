// The MKS motor dashboard's page. It polls /api/state every 100 ms and sends every action to /api/cmd; the server
// (bench.py) checks every move and owns every safety rule - nothing here is trusted to keep the motor safe.
"use strict";

const TOKEN = document.querySelector('meta[name="dashboard-token"]').content;
const $ = (id) => document.getElementById(id);
const $$ = (sel) => Array.from(document.querySelectorAll(sel));
const HEADERS = { "Content-Type": "application/json", "X-Dashboard-Token": TOKEN };
const STATUS_ACTIVE = new Set([2, 3, 4, 5, 6]);

const S = {
  st: null,
  samples: [],
  sampleSeq: 0,
  logSeq: 0,
  noticeSeq: 0,
  logPaused: false,
  graphPaused: false,
  context: null,
  jogDir: 0,
  jogTimer: null,
  routineRows: null,
  routineKey: null,
  questionKey: null,
  findings: null,
  jointsFilled: false,
  failures: 0,
};

// --- talking to the server ----------------------------------------------------------------------------------------

async function api(action, body = {}) {
  try {
    const res = await fetch("/api/cmd", { method: "POST", headers: HEADERS, body: JSON.stringify({ action, ...body }) });
    if (res.status === 403) return { ok: false, error: "The dashboard restarted or the token is wrong: reload the page" };
    return await res.json();
  } catch (e) {
    return { ok: false, error: `no answer from the dashboard (${e.message}) - is it still running?` };
  }
}

function showMsg(el, text, ok) {
  if (!el) return;
  el.textContent = text || "";
  el.classList.toggle("err", ok === false);
  el.classList.toggle("ok", ok === true);
}

async function act(action, body, msgEl, okText) {
  const r = await api(action, body);
  showMsg(msgEl, r.ok ? (r.message || okText || "done") : r.error, r.ok);
  return r;
}

function toast(text, level = "info") {
  const el = document.createElement("div");
  el.className = `toast ${level === "warn" ? "warn" : level === "err" ? "err" : ""}`;
  el.textContent = text;
  $("toasts").prepend(el);
  setTimeout(() => el.remove(), level === "info" ? 6000 : 12000);
  while ($("toasts").children.length > 6) $("toasts").lastChild.remove();
}

async function download(path, name) {
  const res = await fetch(path, { headers: { "X-Dashboard-Token": TOKEN } });
  if (!res.ok) { toast(await res.text(), "err"); return; }
  const url = URL.createObjectURL(await res.blob());
  const a = document.createElement("a");
  a.href = url;
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

function stamp() {
  return new Date().toISOString().replace(/[:T]/g, "-").slice(0, 19);
}

async function poll() {
  try {
    const q = `samples=${S.sampleSeq}&log=${S.logSeq}&notices=${S.noticeSeq}`;
    const res = await fetch(`/api/state?${q}`, { headers: { "X-Dashboard-Token": TOKEN } });
    if (res.status === 403) throw new Error("the dashboard restarted: reload the page");
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const st = await res.json();
    S.failures = 0;
    ingest(st);
    render(st);
  } catch (e) {
    S.failures += 1;
    $("conn").textContent = `no answer: ${e.message}`;
    $("conn").classList.add("err");
    if (S.jogDir) jogStop(false);
  }
  setTimeout(poll, 100);
}

function ingest(st) {
  if (st.samples.length) {
    S.samples.push(...st.samples);
    S.sampleSeq = st.samples[st.samples.length - 1][0];
    const cutoff = S.samples[S.samples.length - 1][1] - 600;
    let drop = 0;
    while (drop < S.samples.length && (S.samples[drop][1] < cutoff || S.samples.length - drop > 60000)) drop++;
    if (drop) S.samples.splice(0, drop);
  }
  if (st.log.length) {
    S.logSeq = st.log[st.log.length - 1].seq;
    if (!S.logPaused) addLog(st.log);
  }
  for (const n of st.notices) {
    S.noticeSeq = Math.max(S.noticeSeq, n.seq);
    if (S.st) toast(n.text, n.level);   // not the ones from before this page loaded (they are in the log)
  }
}

// --- units ----------------------------------------------------------------------------------------------------------

function units(st) {
  const g = st.gear_ratio, cpd = st.counts_per_joint_deg, sign = st.sign;
  if (st.mode === "bench") {
    return { pos: (c) => (c * 360) / 16384, spd: (rpm) => rpm, posUnit: "° motor", spdUnit: "RPM", rel: false };
  }
  if (st.zero_deg === null) {
    return { pos: (c) => (sign * c) / cpd, spd: (rpm) => rpm * 6 * g, posUnit: "° from the encoder's zero",
             spdUnit: "°/s", rel: true };
  }
  return { pos: (c) => st.zero_deg + (sign * c) / cpd, spd: (rpm) => rpm * 6 * g, posUnit: "°", spdUnit: "°/s", rel: false };
}

const fmt = (v, d = 2) => (v === null || v === undefined || Number.isNaN(v) ? "—" : Number(v).toFixed(d));
const num = (id) => parseFloat($(id).value);
const int = (id) => parseInt($(id).value, 10);
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const round = (v, d = 1) => Math.round(v * 10 ** d) / 10 ** d;

// --- rendering ------------------------------------------------------------------------------------------------------

function render(st) {
  S.st = st;
  const u = units(st);
  const r = st.readout;
  const fresh = r.fresh || {};
  const joint = st.joints.find((j) => j.name === st.joint);

  $("conn").textContent = st.bus_error ? `bus error: ${st.bus_error}` : st.label;
  $("conn").classList.toggle("err", !!st.bus_error);
  const online = $("online");
  online.textContent = st.online ? `board ${st.can_id} online` : `board ${st.can_id} offline`;
  online.className = `pill ${st.online ? "ok" : "err"}`;

  if (!S.jointsFilled) {
    $("joint-select").innerHTML = st.joints.map((j) => `<option value="${j.name}">${j.name} (id ${j.can_id ?? "-"})</option>`).join("");
    $("set-82").innerHTML = Object.entries(st.work_modes).map(([k, v]) => `<option value="${k}">${k}: ${v}</option>`).join("");
    $("set-82").value = "5";
    S.jointsFilled = true;
  }

  const key = `${st.mode}|${st.joint}|${st.can_id}|${st.zero_deg === null}|${st.sign}`;
  if (key !== S.context) {
    S.context = key;
    fillDefaults(st, u);
  }

  $("mode-joint").classList.toggle("on", st.mode === "joint");
  $("mode-bench").classList.toggle("on", st.mode === "bench");
  $$(".joint-only").forEach((el) => el.classList.toggle("hidden", st.mode !== "joint"));
  $$(".bench-only").forEach((el) => el.classList.toggle("hidden", st.mode !== "bench"));
  $("bench-banner").classList.toggle("hidden", st.mode !== "bench");
  $("ref-banner").classList.toggle("hidden", !(st.mode === "joint" && st.zero_deg === null));
  $$(".unit-pos").forEach((el) => (el.textContent = st.mode === "joint" ? "(°)" : "(° motor)"));
  $$(".unit-speed").forEach((el) => (el.textContent = st.mode === "joint" ? "(°/s)" : "(RPM)"));
  $$(".joint-name").forEach((el) => (el.textContent = st.joint));

  // readouts
  $("pos-label").textContent = st.mode === "joint" ? (u.rel ? `${st.joint}: no reference` : st.joint) : "Motor shaft";
  const counts = r.counts;
  $("pos-value").textContent = counts === null ? "—" : fmt(u.pos(counts), 2);
  $("pos-unit").textContent = u.posUnit;
  $("pos-sub").textContent = counts === null ? "no encoder reading" :
    `${counts} counts · ${fmt(counts / 16384, 3)} motor turns` +
    (st.mode === "bench" ? ` · ${fmt((st.sign * counts) / st.counts_per_joint_deg, 2)}° of ${st.joint}` : "");
  const rpm = r.speed_rpm;
  $("speed-value").textContent = rpm === null ? "—" : fmt(u.spd(rpm), st.mode === "bench" ? 0 : 2);
  $("speed-unit").textContent = u.spdUnit;
  $("speed-sub").textContent = rpm === null ? "—" :
    st.mode === "bench" ? `${fmt(rpm * 6 * st.gear_ratio, 2)} °/s at ${st.joint} · positive = CCW (the manual)` :
    `${rpm} RPM at the motor · the board's sign: positive = CCW`;
  setDD("r-status", r.status_name ?? "—", fresh.status, STATUS_ACTIVE.has(r.status) ? "warn" : "");
  setDD("r-enabled", r.enabled === null ? "—" : r.enabled ? "enabled" : "disabled (free)", fresh.enabled, r.enabled ? "good" : "");
  setDD("r-stalled", r.stalled === null ? "—" : r.stalled ? "STALLED" : "no", fresh.stalled, r.stalled ? "bad" : "");
  const in1 = r.io === null ? null : r.io & 1;
  setDD("r-io", in1 === null ? "—" : in1 ? "high: no magnet" : "LOW: magnet seen", fresh.io, in1 === 0 ? "good" : "");
  setDD("r-volts", r.millivolts === null ? "n/a" : `${fmt(r.millivolts / 1000, 2)} V`, fresh.power);
  setDD("r-amps", r.milliamps === null ? "n/a" : `${r.milliamps} mA`, fresh.power);
  setDD("r-error", r.angle_error_deg === null ? "—" : `${fmt(r.angle_error_deg, 3)}° motor`, fresh.angle_error);
  const hs = r.home_status;
  const hsNames = { 0: "going", 1: "success", 2: "failed" };
  setDD("r-home", hs === null ? "—" : hs.length === 2 ? `zero ${hsNames[hs[0]] ?? hs[0]}, homing ${hsNames[hs[1]] ?? hs[1]}` :
    hs.map((x) => hsNames[x] ?? x).join(", "), fresh.home_status);
  setDD("r-version", r.version ?? "—", true);
  setDD("r-reference", st.mode === "bench" ? "bench: encoder zero" : st.reference, true, st.zero_deg === null && st.mode === "joint" ? "warn" : "");
  $("io-big").textContent = in1 === null ? "IN_1: —" : in1 ? "IN_1 high: no magnet" : "IN_1 LOW: magnet seen (switch on)";
  $("io-big").classList.toggle("on", in1 === 0);

  // motion
  const busy = st.motion || (STATUS_ACTIVE.has(r.status) ? r.status_name : null);
  $("motion-state").textContent = st.routine && ["running", "waiting"].includes(st.routine.status) ? `test: ${st.routine.title}` : busy || "idle";
  $("motion-state").className = `pill ${busy ? "busy" : ""}`;
  $("speed-cap-text").textContent = `(cap ${fmt(st.speed_cap, st.mode === "bench" ? 0 : 1)} ${u.spdUnit})`;
  const speedRange = $("speed-range");
  speedRange.max = st.speed_cap;
  speedRange.step = st.mode === "bench" ? 1 : 0.1;
  if (num("speed-num") > st.speed_cap) setSpeed(st.speed_cap);
  const noRef = st.mode === "joint" && st.zero_deg === null;
  const testing = st.routine && ["running", "waiting"].includes(st.routine.status);
  ["goto-btn", "step-minus", "step-plus", "jog-minus", "jog-plus", "run-cw", "run-ccw"].forEach((id) => ($(id).disabled = noRef || testing));
  $("hard-limits").textContent = `${st.hard_limits[0]}..${st.hard_limits[1]}°`;
  $("cap-text").textContent = `${st.max_joint_speed}`;
  $("bench-ceiling").textContent = `${st.bench_rpm_ceiling} RPM`;
  $("config-sign").textContent = `${st.config_sign > 0 ? "+1" : "−1"}`;
  $("sign-plus").classList.toggle("on", st.sign > 0);
  $("sign-minus").classList.toggle("on", st.sign < 0);
  $("home-deg").textContent = st.home_deg;
  $("ref-text").textContent = st.reference;
  const lim = st.limits;
  $("bench-joint-hint").textContent = joint ?
    `At ${st.joint} (${fmt(1 / joint.gear_ratio, 1)}:1) this window is ${fmt(lim.bench_min * joint.gear_ratio, 1)}..${fmt(lim.bench_max * joint.gear_ratio, 1)}° of the joint.` : "";

  // recording
  const rec = st.recording !== null;
  $("rec-toggle").textContent = rec ? `■ Stop (${st.recording})` : "● Record";
  $("rec-toggle").classList.toggle("recording", rec);

  // findings
  const findings = st.findings.join("\n");
  if (findings !== S.findings) {
    S.findings = findings;
    $("findings").value = findings;
  }

  // log footer
  const c = st.counts_bus;
  $("bus-counts").textContent = `sent ${c.tx ?? 0} · received ${c.rx ?? 0} · polls missed ${c.missed ?? 0} · unparsed ${c.bad ?? 0}` +
    (st.skipped_reads.length ? ` · not answered, skipped: ${st.skipped_reads.join(", ")}` : "");
  $("log-polls").checked = st.log_polls;

  renderRoutine(st);
  if (!S.graphPaused) drawGraph(st, u);
}

function setDD(id, text, fresh, cls = "") {
  const el = $(id);
  el.textContent = text;
  el.className = `${fresh === false || fresh === undefined ? "stale" : ""} ${cls}`.trim();
}

function setSpeed(v) {
  $("speed-range").value = v;
  $("speed-num").value = v;
}

function fillDefaults(st, u) {
  const cap = st.speed_cap;
  const [lo, hi] = st.window;
  const here = st.readout.counts === null || u.rel ? null : u.pos(st.readout.counts);
  const joint = st.mode === "joint";
  const speed = joint ? Math.min(5, cap) : Math.min(60, cap);
  setSpeed(round(speed, 1));
  $("lim-jmin").value = st.limits.joint_min;
  $("lim-jmax").value = st.limits.joint_max;
  $("lim-jspeed").value = st.limits.joint_speed;
  $("lim-bmin").value = st.limits.bench_min;
  $("lim-bmax").value = st.limits.bench_max;
  $("lim-brpm").value = st.limits.bench_rpm;
  if (document.activeElement !== $("canid-input")) $("canid-input").value = st.can_id;
  $("joint-select").value = st.joint;
  $("goto-value").value = here === null ? "" : round(here, 2);
  $("step-value").value = joint ? 5 : 90;
  const mid = here === null ? (lo + hi) / 2 : here;
  const span = joint ? Math.min(10, (hi - lo) / 4) : 90;
  const a = round(clamp(mid - span, lo, hi), 1), b = round(clamp(mid + span, lo, hi), 1);
  $("rep-a").value = a; $("rep-b").value = b; $("rep-speed").value = round(speed, 1);
  $("step-size").value = joint ? 5 : 90; $("step-speed").value = round(speed, 1);
  $("sw-a").value = a; $("sw-b").value = b;
  $("sw-speeds").value = (joint ? [0.2, 0.4, 0.6, 0.8, 1].map((f) => round(cap * f, 1)) :
    [30, 60, 120, 240].filter((s) => s <= cap)).join(" ");
  $("dir-amount").value = joint ? 5 : 90;
  $("dir-speed").value = joint ? round(Math.min(3, cap), 1) : Math.min(30, cap);
  if (joint) {
    $("sz-a").value = round(clamp(st.home_deg + 15, lo, hi), 1);
    $("sz-b").value = round(lo, 1);
    $("sz-speed").value = round(Math.min(2, cap), 1);
    $("ref-deg").value = st.home_deg;
    const dps = Math.min(5, cap);
    $("home-rpm").value = Math.max(1, Math.floor(dps / 6 / st.gear_ratio));
  } else {
    $("sz-a").value = 180; $("sz-b").value = -180; $("sz-speed").value = Math.min(30, cap);
    $("home-rpm").value = 30;
  }
  updateHomeHint();
}

function updateHomeHint() {
  const st = S.st;
  if (!st) return;
  const rpm = num("home-rpm");
  $("home-speed-hint").textContent = Number.isFinite(rpm) ? `= ${fmt(rpm * 6 * st.gear_ratio, 2)} °/s at ${st.joint}` : "";
}

// --- the graph ------------------------------------------------------------------------------------------------------

function niceStep(range, ticks) {
  const raw = range / ticks;
  const mag = 10 ** Math.floor(Math.log10(raw || 1));
  const n = raw / mag;
  return (n < 1.5 ? 1 : n < 3 ? 2 : n < 7 ? 5 : 10) * mag;
}

function prepCanvas(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.getAttribute("height") * 1;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
    canvas.style.height = `${h}px`;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  return { ctx, w, h };
}

function css(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

function pane(ctx, box, xs, ys, xr, yr, color, label, extra = []) {
  const { x, y, w, h } = box;
  ctx.strokeStyle = css("--grid");
  ctx.fillStyle = css("--muted");
  ctx.font = "11px system-ui";
  ctx.lineWidth = 1;
  let [y0, y1] = yr;
  if (y1 - y0 < 1e-9) { y0 -= 1; y1 += 1; }
  const pad = (y1 - y0) * 0.08;
  y0 -= pad; y1 += pad * 2.5;   // headroom for the label at the top left
  const sx = (v) => x + ((v - xr[0]) / (xr[1] - xr[0])) * w;
  const sy = (v) => y + h - ((v - y0) / (y1 - y0)) * h;
  const step = niceStep(y1 - y0, 4);
  for (let v = Math.ceil(y0 / step) * step; v <= y1; v += step) {
    ctx.beginPath(); ctx.moveTo(x, sy(v)); ctx.lineTo(x + w, sy(v)); ctx.stroke();
    ctx.fillText(fmt(v, step < 1 ? 2 : step < 10 ? 1 : 0), x + w + 4, sy(v) + 4);
  }
  for (const [v, c] of extra) {
    if (v < y0 || v > y1) continue;
    ctx.save(); ctx.setLineDash([5, 4]); ctx.strokeStyle = c;
    ctx.beginPath(); ctx.moveTo(x, sy(v)); ctx.lineTo(x + w, sy(v)); ctx.stroke(); ctx.restore();
  }
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.6;
  ctx.beginPath();
  let started = false;
  for (let i = 0; i < xs.length; i++) {
    if (ys[i] === null || Number.isNaN(ys[i])) { started = false; continue; }
    const px = sx(xs[i]), py = sy(ys[i]);
    if (!started) { ctx.moveTo(px, py); started = true; } else ctx.lineTo(px, py);
  }
  ctx.stroke();
  ctx.fillStyle = color;
  ctx.font = "600 12px system-ui";
  ctx.fillText(label, x + 4, y + 14);
}

function drawGraph(st, u) {
  const { ctx, w, h } = prepCanvas($("graph"));
  const span = num("graph-window");
  if (!S.samples.length) {
    ctx.fillStyle = css("--muted"); ctx.font = "13px system-ui";
    ctx.fillText("waiting for the board's encoder …", 16, h / 2);
    return;
  }
  const tEnd = S.samples[S.samples.length - 1][1];
  const t0 = tEnd - span;
  const pts = S.samples.filter((s) => s[1] >= t0 && s[2] !== null);
  const xs = pts.map((s) => s[1] - tEnd);
  const pos = pts.map((s) => u.pos(s[2]));
  const spd = pts.map((s) => (s[3] === null ? null : u.spd(s[3])));
  const right = 52, gap = 18;
  const ph = (h - gap - 18) * 0.62, sh = h - gap - 18 - ph;
  const finite = (a) => a.filter((v) => v !== null && Number.isFinite(v));
  const pr = finite(pos), sr = finite(spd);
  const extra = u.rel ? [] : [[st.window[0], css("--warn")], [st.window[1], css("--warn")]];
  pane(ctx, { x: 6, y: 4, w: w - right - 6, h: ph }, xs, pos, [-span, 0], [Math.min(...pr), Math.max(...pr)], css("--pos"),
    `position (${u.posUnit})`, extra);
  const slo = Math.min(0, ...sr), shi = Math.max(0, ...sr);
  pane(ctx, { x: 6, y: ph + gap, w: w - right - 6, h: sh }, xs, spd, [-span, 0], [slo, shi], css("--spd"), `speed (${u.spdUnit})`);
  ctx.fillStyle = css("--muted"); ctx.font = "11px system-ui";
  const tstep = niceStep(span, 6);
  for (let t = -Math.floor(span / tstep) * tstep; t <= 0; t += tstep) {
    const px = 6 + ((t + span) / span) * (w - right - 6);
    ctx.fillText(`${fmt(t, 0)} s`, px - 10, h - 3);
  }
}

function drawChart(canvas, chart) {
  const { ctx, w, h } = prepCanvas(canvas);
  const names = Object.keys(chart.series).filter((k) => chart.series[k].length);
  if (!names.length) return;
  const all = names.flatMap((k) => chart.series[k]);
  const xr = [Math.min(...all.map((p) => p[0])), Math.max(...all.map((p) => p[0]))];
  if (xr[1] - xr[0] < 1e-9) { xr[0] -= 1; xr[1] += 1; }
  const yr = [Math.min(...all.map((p) => p[1])), Math.max(...all.map((p) => p[1]))];
  const colors = [css("--pos"), css("--spd"), css("--ok"), css("--warn"), css("--err")];
  names.forEach((k, i) => {
    const pts = chart.series[k];
    pane(ctx, { x: 6, y: 18 + 0, w: w - 58, h: h - 40 }, pts.map((p) => p[0]), pts.map((p) => p[1]), xr, yr, colors[i % 5], "");
    ctx.fillStyle = colors[i % 5];
    ctx.font = "600 12px system-ui";
    ctx.fillText(k, 8 + i * 170, 12);
  });
  ctx.fillStyle = css("--muted"); ctx.font = "11px system-ui";
  ctx.fillText(`${chart.x}: ${fmt(xr[0], 2)} … ${fmt(xr[1], 2)}`, 8, h - 4);
}

// --- tests ----------------------------------------------------------------------------------------------------------

function renderRoutine(st) {
  const r = st.routine;
  const banner = $("question-banner");
  if (!r) {
    banner.classList.add("hidden");
    $("routine-cancel").disabled = true;
    return;
  }
  const active = ["running", "waiting"].includes(r.status);
  $("routine-title").textContent = r.title;
  $("routine-status").textContent = r.status;
  $("routine-status").className = `pill ${r.status === "done" ? "ok" : r.status === "failed" ? "err" : r.status === "cancelled" ? "warn" : "busy"}`;
  $("routine-cancel").disabled = !active;
  $("routine-progress").textContent = r.progress;
  showMsg($("routine-error"), r.error || "", r.error ? false : undefined);
  const key = `${r.started}|${r.status}|${r.summary.length}`;
  if (key !== S.routineKey) {
    S.routineKey = key;
    $("routine-summary").innerHTML = "";
    for (const line of r.summary) {
      const li = document.createElement("li");
      li.textContent = line;
      $("routine-summary").append(li);
    }
  }
  const tableKey = `${r.started}|${r.row_count}`;
  if (tableKey !== S.routineRows) {
    S.routineRows = tableKey;
    const thead = $("routine-table").querySelector("thead"), tbody = $("routine-table").querySelector("tbody");
    thead.innerHTML = "";
    const tr = document.createElement("tr");
    for (const c of r.columns) { const th = document.createElement("th"); th.textContent = c; tr.append(th); }
    thead.append(tr);
    tbody.innerHTML = "";
    for (const row of r.rows.slice(-400)) {
      const tr2 = document.createElement("tr");
      for (const v of row) { const td = document.createElement("td"); td.textContent = v === null ? "—" : v; tr2.append(td); }
      tbody.append(tr2);
    }
  }
  const chart = $("routine-chart");
  chart.classList.toggle("hidden", !r.chart);
  if (r.chart) drawChart(chart, r.chart);
  if (r.question) {
    banner.classList.remove("hidden");
    const qkey = `${r.started}|${r.question.text}`;
    if (qkey !== S.questionKey) {
      S.questionKey = qkey;
      $("question-text").textContent = `${r.title}: ${r.question.text}`;
      const box = $("question-options");
      box.innerHTML = "";
      for (const opt of r.question.options) {
        const b = document.createElement("button");
        b.className = "btn";
        b.textContent = opt;
        b.onclick = () => api("routine_answer", { answer: opt });
        box.append(b);
      }
    }
  } else {
    banner.classList.add("hidden");
    S.questionKey = null;
  }
}

function testParams(kind) {
  const acc = int("acc-num");
  switch (kind) {
    case "repeat": return { a: num("rep-a"), b: num("rep-b"), cycles: int("rep-cycles"), speed: num("rep-speed"), acc: int("rep-acc"), dwell: num("rep-dwell") };
    case "step": return { step: num("step-size"), speed: num("step-speed"), acc: int("step-acc"), duration: num("step-duration") };
    case "sweep": return { a: num("sw-a"), b: num("sw-b"), speeds: $("sw-speeds").value, acc: int("sw-acc") };
    case "direction": return { amount: num("dir-amount"), speed: num("dir-speed") };
    case "switch": return { a: num("sz-a"), b: num("sz-b"), speed: num("sz-speed"), acc };
    case "home": {
      const mode = $("home-mode").value;
      return { trigger: int("home-trig"), direction: int("home-dir"), rpm: int("home-rpm"), end_limit: int("home-end"), hm_mode: mode === "none" ? null : parseInt(mode, 10) };
    }
    default: return {};
  }
}

async function startTest(kind, msgEl) {
  const r = await api("routine", { kind, params: testParams(kind) });
  if (!r.ok) { toast(r.error, "err"); showMsg(msgEl, r.error, false); return; }
  S.routineRows = null;
  $("routine-card").scrollIntoView({ behavior: "smooth", block: "nearest" });
}

// --- the log --------------------------------------------------------------------------------------------------------

function addLog(entries) {
  const tbody = $("log-table").querySelector("tbody");
  const frag = document.createDocumentFragment();
  for (const e of entries.slice().reverse()) {
    const tr = document.createElement("tr");
    tr.className = e.direction;
    const t = new Date(e.t * 1000);
    const cells = [
      `${t.toLocaleTimeString([], { hour12: false })}.${String(t.getMilliseconds()).padStart(3, "0")}`,
      e.direction === "tx" ? "→" : e.direction === "rx" ? "←" : "•",
      e.can_id === null ? "" : `0x${e.can_id.toString(16).padStart(2, "0")}`,
      e.hex,
      e.text,
    ];
    cells.forEach((c, i) => {
      const td = document.createElement("td");
      td.textContent = c;
      if (i === 1) td.className = "dir";
      if (i === 4) td.className = "text";
      tr.append(td);
    });
    frag.append(tr);
  }
  tbody.prepend(frag);
  while (tbody.children.length > 1500) tbody.lastChild.remove();
}

// --- jog ------------------------------------------------------------------------------------------------------------

function jogStart(dir) {
  if (S.jogDir === dir) return;
  jogStop(true);
  S.jogDir = dir;
  $(dir > 0 ? "jog-plus" : "jog-minus").classList.add("held");
  const send = async () => {
    if (S.jogDir !== dir) return;
    const r = await api("jog", { direction: dir, speed: num("speed-num"), acc: int("acc-num") });
    if (!r.ok && S.jogDir === dir) {
      showMsg($("motion-msg"), r.error, false);
      jogStop(true);
    }
  };
  send();
  S.jogTimer = setInterval(send, 100);
}

function jogStop(release) {
  if (!S.jogDir) return;
  clearInterval(S.jogTimer);
  S.jogTimer = null;
  S.jogDir = 0;
  $("jog-plus").classList.remove("held");
  $("jog-minus").classList.remove("held");
  if (release) api("jog_release");
}

function bindJog(id, dir) {
  const el = $(id);
  el.addEventListener("pointerdown", (e) => { if (!el.disabled) { el.setPointerCapture(e.pointerId); jogStart(dir); } });
  for (const ev of ["pointerup", "pointercancel", "lostpointercapture"]) el.addEventListener(ev, () => jogStop(true));
}

function typing(e) {
  const t = e.target;
  return t && (t.tagName === "INPUT" || t.tagName === "TEXTAREA" || t.tagName === "SELECT");
}

async function halt() {
  jogStop(false);
  const r = await api("halt");
  toast(r.ok ? r.message : r.error, r.ok ? "warn" : "err");
}

// --- wiring ---------------------------------------------------------------------------------------------------------

function bind() {
  $("halt").onclick = halt;
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape") { e.preventDefault(); halt(); return; }
    if (typing(e)) return;
    if (e.key === " ") {
      e.preventDefault();
      if (document.activeElement && document.activeElement.blur) document.activeElement.blur();   // no button re-fires on keyup
      halt();
      return;
    }
    if (e.repeat) return;
    if (e.key === "ArrowRight" && !$("jog-plus").disabled) { e.preventDefault(); jogStart(1); }
    if (e.key === "ArrowLeft" && !$("jog-minus").disabled) { e.preventDefault(); jogStart(-1); }
  });
  document.addEventListener("keyup", (e) => {
    if (e.key === " " && !typing(e)) e.preventDefault();
    if ((e.key === "ArrowRight" && S.jogDir > 0) || (e.key === "ArrowLeft" && S.jogDir < 0)) jogStop(true);
  });
  window.addEventListener("blur", () => jogStop(true));
  document.addEventListener("visibilitychange", () => { if (document.hidden) jogStop(true); });

  $("board-apply").onclick = () => act("select", { joint: $("joint-select").value, can_id: int("canid-input") }, $("motion-msg"));
  $("joint-select").onchange = () => {
    const j = S.st && S.st.joints.find((x) => x.name === $("joint-select").value);
    if (j && j.can_id) $("canid-input").value = j.can_id;
  };
  $("mode-joint").onclick = () => act("mode", { mode: "joint" }, $("motion-msg"));
  $("mode-bench").onclick = () => act("mode", { mode: "bench" }, $("motion-msg"));
  $("rec-toggle").onclick = () => api("record", { on: !(S.st && S.st.recording !== null) });
  $("rec-download").onclick = () => download("/api/record.csv", `recording-${S.st ? S.st.joint : "board"}-${stamp()}.csv`);
  $("graph-pause").onclick = () => { S.graphPaused = !S.graphPaused; $("graph-pause").textContent = S.graphPaused ? "Resume" : "Pause"; };

  $("speed-range").oninput = () => ($("speed-num").value = $("speed-range").value);
  $("speed-num").oninput = () => ($("speed-range").value = $("speed-num").value);
  $("acc-range").oninput = () => ($("acc-num").value = $("acc-range").value);
  $("acc-num").oninput = () => ($("acc-range").value = $("acc-num").value);
  $("enable-on").onclick = () => act("enable", { on: true }, $("motion-msg"), "enabled");
  $("enable-off").onclick = () => {
    if (confirm("Disabling frees the shaft: a loaded joint falls. Support the arm first.\n\nDisable the motor?")) {
      act("enable", { on: false }, $("motion-msg"), "disabled");
    }
  };
  $("soft-stop").onclick = () => { jogStop(false); act("stop", {}, $("motion-msg"), "stopping"); };
  bindJog("jog-minus", -1);
  bindJog("jog-plus", 1);
  const move = (extra) => ({ speed: num("speed-num"), acc: int("acc-num"), ...extra });
  $("goto-btn").onclick = () => act("move", move({ value: num("goto-value") }), $("motion-msg"), "moving");
  $("step-minus").onclick = () => act("step", move({ delta: -num("step-value") }), $("motion-msg"), "moving");
  $("step-plus").onclick = () => act("step", move({ delta: num("step-value") }), $("motion-msg"), "moving");
  $("run-cw").onclick = () => act("run", { cw: true, rpm: num("run-rpm"), acc: int("acc-num") }, $("motion-msg"), "running CW");
  $("run-ccw").onclick = () => act("run", { cw: false, rpm: num("run-rpm"), acc: int("acc-num") }, $("motion-msg"), "running CCW");

  for (const tab of $$(".tab")) {
    tab.onclick = () => {
      $$(".tab").forEach((t) => t.classList.toggle("active", t === tab));
      $$(".tabpane").forEach((p) => p.classList.toggle("hidden", p.id !== `tab-${tab.dataset.tab}`));
    };
  }
  for (const tab of $$(".subtab")) {
    tab.onclick = () => {
      $$(".subtab").forEach((t) => t.classList.toggle("active", t === tab));
      $$(".testform").forEach((p) => p.classList.toggle("hidden", p.id !== `test-${tab.dataset.test}`));
    };
  }

  $("limits-apply").onclick = () => {
    const st = S.st;
    const limits = st.mode === "joint" ?
      { joint_min: num("lim-jmin"), joint_max: num("lim-jmax"), joint_speed: num("lim-jspeed") } :
      { bench_min: num("lim-bmin"), bench_max: num("lim-bmax"), bench_rpm: num("lim-brpm") };
    act("limits", { limits }, $("limits-msg"), "limits applied").then((r) => { if (r.ok) S.context = null; });
  };
  $("sign-plus").onclick = () => act("sign", { sign: 1 }, $("limits-msg"), "sign +1");
  $("sign-minus").onclick = () => act("sign", { sign: -1 }, $("limits-msg"), "sign −1");
  $("ref-set").onclick = () => act("reference", { deg: num("ref-deg") }, $("ref-msg"), "reference set");
  $("ref-clear").onclick = () => act("clear_reference", {}, $("ref-msg"), "reference cleared");

  $("home-rpm").oninput = updateHomeHint;
  $("home-go").onclick = () => {
    const st = S.st;
    const dir = $("home-dir").value === "1" ? "CCW" : "CW";
    if (confirm(`Home ${st.joint}: it turns ${dir} at ${$("home-rpm").value} RPM until the switch turns on.\n\nWatch it, and press STOP (or Esc) if it heads the wrong way.`)) {
      startTest("home", $("homing-msg"));
    }
  };
  $("zero-set").onclick = () => {
    if (confirm("Make the current position the board's encoder zero (0x92)? Joint mode's reference is kept.")) act("set_zero", {}, $("homing-msg"), "zero set");
  };
  $("stall-release").onclick = () => act("release_stall", {}, $("homing-msg"), "stall released");
  $("calibrate").onclick = () => {
    if (confirm("Calibration turns the motor by itself: it must be UNLOADED, out of the arm. Continue?")) startTest("calibrate", $("homing-msg"));
  };

  for (const b of $$(".start")) b.onclick = () => startTest(b.dataset.kind, null);
  $("routine-cancel").onclick = () => api("routine_cancel");
  $("routine-csv").onclick = () => download("/api/routine.csv", `${S.st && S.st.routine ? S.st.routine.kind : "test"}-${S.st ? S.st.joint : ""}-${stamp()}.csv`);

  $("scan-go").onclick = async () => {
    $("scan-result").textContent = "scanning …";
    const r = await api("scan", { first: int("scan-first"), last: int("scan-last") });
    if (!r.ok) { $("scan-result").textContent = r.error; return; }
    if (!r.found.length) { $("scan-result").textContent = "No board answered in that range."; return; }
    $("scan-result").innerHTML = "<table><thead><tr><th>CAN id</th><th>board</th><th>firmware</th><th>calibrated</th></tr></thead><tbody></tbody></table>";
    const tb = $("scan-result").querySelector("tbody");
    for (const f of r.found) {
      const tr = document.createElement("tr");
      for (const v of [f.can_id, f.board, f.firmware, f.calibrated ? "yes" : "NO"]) { const td = document.createElement("td"); td.textContent = v; tr.append(td); }
      tb.append(tr);
    }
  };
  $("findings-copy").onclick = async () => {
    try { await navigator.clipboard.writeText($("findings").value); toast("Findings copied"); } catch { $("findings").select(); }
  };
  $("findings-clear").onclick = () => api("clear_findings");

  $("settings-read").onclick = async () => {
    showMsg($("settings-msg"), "reading …");
    const r = await api("read_settings");
    if (!r.ok) { showMsg($("settings-msg"), r.error, false); return; }
    const names = { "82": "work mode", "83": "working current (mA)", "84": "microsteps", "88": "stall protection", "89": "MPlyer", "8A": "CAN bitrate (0-3: 125K/250K/500K/1M)", "8B": "CAN id", "8C": "response mode", "9E": "limit remap" };
    const tb = $("settings-table").querySelector("tbody");
    tb.innerHTML = "";
    for (const [code, v] of Object.entries(r.settings)) {
      const tr = document.createElement("tr");
      const shown = v === null ? "no reply (0x00 is V1.0.6+)" : v.value === null ? `not readable (${v.raw})` :
        code === "82" ? `${v.value}: ${S.st.work_modes[v.value] ?? "?"}` : `${v.value}  (${v.raw})`;
      for (const c of [`0x${code} ${names[code] ?? ""}`, shown]) { const td = document.createElement("td"); td.textContent = c; tr.append(td); }
      tb.append(tr);
      const input = $(`set-${code}`);
      if (input && v && v.value !== null && code !== "8C") input.value = code === "83" || code === "8B" ? v.value : String(v.value);
    }
    showMsg($("settings-msg"), "read", true);
  };
  for (const b of $$("[data-set]")) {
    b.onclick = () => {
      const code = b.dataset.set;
      const value = $(`set-${code}`).value;
      const info = S.st.settings[code];
      let warn = "";
      if (code === "8B") warn = "\n\nThe dashboard follows the board to its new id.";
      if (code === "82" && value !== "5") warn = "\n\nOnly the bus modes (3-5) take the motion commands: in a pulse mode nothing here moves the motor.";
      if (confirm(`Write ${info.name} = ${value} to board ${S.st.can_id}? It is saved on the board.${warn}`)) {
        act("setting", { code: `0x${code}`, value }, $("settings-msg"));
      }
    };
  }
  for (const b of $$("[data-action]")) {
    b.onclick = () => {
      const code = b.dataset.action;
      const text = { "3D": "Release the stall protection?", "41": "Restart the board (its settings are kept)?",
        "3F": "Restore FACTORY DEFAULTS? The board's CAN id goes back to 1, its work mode to CR_vFOC (a pulse mode) and every setting to the factory's." }[code];
      if (confirm(text)) act("setting", { code: `0x${code}` }, $("settings-msg"));
    };
  }

  $("log-polls").onchange = () => api("log_polls", { on: $("log-polls").checked });
  $("log-pause").onclick = () => { S.logPaused = !S.logPaused; $("log-pause").textContent = S.logPaused ? "Resume" : "Pause"; };
  $("log-clear").onclick = () => { $("log-table").querySelector("tbody").innerHTML = ""; };
  $("raw-send").onclick = async () => {
    const r = await api("raw", { hex: $("raw-hex").value });
    showMsg($("raw-msg"), r.ok ? `sent ${r.sent} → ${r.reply === null ? "no reply" : r.reply}` : r.error, r.ok);
  };
  $("raw-hex").addEventListener("keydown", (e) => { if (e.key === "Enter") $("raw-send").click(); });
}

const ping = new Worker("/ping.js");
ping.postMessage(TOKEN);
bind();
poll();
