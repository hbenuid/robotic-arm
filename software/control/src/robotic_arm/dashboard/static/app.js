// The motor dashboard's page. It polls /api/state every 100 ms and sends every action to /api/cmd. The server
// (bench.py) checks every move and owns every safety rule: nothing here is trusted to keep the motor safe.
"use strict";

const TOKEN = document.querySelector('meta[name="dashboard-token"]').content;
const $ = (id) => document.getElementById(id);
const $$ = (sel) => Array.from(document.querySelectorAll(sel));
const HEADERS = { "Content-Type": "application/json", "X-Dashboard-Token": TOKEN };
const MOVING = new Set([2, 3, 4, 5, 6]);
const TESTS = {
  repeat: "Back and forth between A and B: the error at each end, stalls.",
  step: "One step from where it is: rise, overshoot, settling.",
  sweep: "End to end at each speed: measured speed, current, angle error.",
  firmware: "Firmware, work mode and settings. Nothing moves.",
  direction: "A small step; you say which way it turned. That gives the sign.",
  switch: "Slowly through the home switch and back: where it turns on and off.",
};

const S = {
  st: null, samples: [], sampleSeq: 0, logSeq: 0, noticeSeq: 0,
  logPaused: false, graphPaused: false, graphSpan: 30,
  context: null, jogDir: 0, jogTimer: null,
  routineKey: null, routineTable: null, questionKey: null, findings: null, filled: false,
};

// --- server ---------------------------------------------------------------------------------------------------------

async function api(action, body = {}) {
  try {
    const res = await fetch("/api/cmd", { method: "POST", headers: HEADERS, body: JSON.stringify({ action, ...body }) });
    if (res.status === 403) return { ok: false, error: "The dashboard restarted: reload the page" };
    return await res.json();
  } catch (e) {
    return { ok: false, error: `no answer from the dashboard (${e.message})` };
  }
}

function say(el, text, ok) {
  if (!el) return;
  el.textContent = text || "";
  el.classList.toggle("err", ok === false);
  el.classList.toggle("ok", ok === true);
}

async function act(action, body, el, okText) {
  const r = await api(action, body);
  say(el, r.ok ? (r.message || okText || "done") : r.error, r.ok);
  return r;
}

function toast(text, level = "info") {
  const el = document.createElement("div");
  el.className = `toast ${level}`;
  el.textContent = text;
  $("toasts").prepend(el);
  setTimeout(() => el.remove(), level === "info" ? 5000 : 10000);
  while ($("toasts").children.length > 5) $("toasts").lastChild.remove();
}

async function download(path, name) {
  const res = await fetch(path, { headers: { "X-Dashboard-Token": TOKEN } });
  if (!res.ok) { toast(await res.text(), "err"); return; }
  const a = document.createElement("a");
  a.href = URL.createObjectURL(await res.blob());
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

const stamp = () => new Date().toISOString().replace(/[:T]/g, "-").slice(0, 19);

async function poll() {
  try {
    const res = await fetch(`/api/state?samples=${S.sampleSeq}&log=${S.logSeq}&notices=${S.noticeSeq}`,
      { headers: { "X-Dashboard-Token": TOKEN } });
    if (res.status === 403) throw new Error("the dashboard restarted: reload the page");
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const st = await res.json();
    ingest(st);
    render(st);
  } catch (e) {
    $("conn").textContent = `no answer: ${e.message}`;
    $("online-dot").classList.remove("on");
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
    if (S.st) toast(n.text, n.level);   // not the ones from before this page loaded
  }
}

// --- units ----------------------------------------------------------------------------------------------------------

function units(st) {
  const g = st.gear_ratio, cpd = st.counts_per_joint_deg, sign = st.sign;
  if (st.mode === "bench") return { pos: (c) => (c * 360) / 16384, spd: (r) => r, posUnit: "° motor", spdUnit: "RPM", rel: false };
  if (st.zero_deg === null) return { pos: (c) => (sign * c) / cpd, spd: (r) => r * 6 * g, posUnit: "° from encoder zero", spdUnit: "°/s", rel: true };
  return { pos: (c) => st.zero_deg + (sign * c) / cpd, spd: (r) => r * 6 * g, posUnit: "°", spdUnit: "°/s", rel: false };
}

const fmt = (v, d = 2) => (v === null || v === undefined || Number.isNaN(v) ? "—" : Number(v).toFixed(d));
const num = (id) => parseFloat($(id).value);
const int = (id) => parseInt($(id).value, 10);
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const round = (v, d = 1) => Math.round(v * 10 ** d) / 10 ** d;
const active = (r) => r && ["running", "waiting"].includes(r.status);

function chip(id, text, cls = "") {
  const el = $(id);
  const small = el.classList.contains("small-chip");
  el.textContent = text;
  el.className = ["chip", cls, small ? "small-chip" : ""].filter(Boolean).join(" ");
}

// --- render ---------------------------------------------------------------------------------------------------------

function render(st) {
  const first = !S.st;
  S.st = st;
  const u = units(st), r = st.readout, fresh = r.fresh || {};
  const joint = st.joints.find((j) => j.name === st.joint);
  const joint_mode = st.mode === "joint", noRef = joint_mode && st.zero_deg === null;
  const testing = active(st.routine);

  if (first) {
    $("joint-select").innerHTML = st.joints.map((j) => `<option value="${j.name}">${j.name}</option>`).join("");
    $("set-82").innerHTML = Object.entries(st.work_modes).map(([k, v]) => `<option value="${k}">${k} ${v}</option>`).join("");
    $("set-82").value = "5";
    showTest($("test-kind").value);
  }
  const key = `${st.mode}|${st.joint}|${st.can_id}|${st.zero_deg === null}|${st.sign}`;
  if (key !== S.context) { S.context = key; fillDefaults(st, u); }

  // identity, mode
  $("board-name").textContent = `${st.joint} · id ${st.can_id}`;
  $("conn").textContent = st.bus_error ? `bus error: ${st.bus_error}` : `${st.online ? "online" : "OFFLINE"} · ${st.label}`;
  $("online-dot").classList.toggle("on", st.online);
  $("mode-joint").classList.toggle("on", joint_mode);
  $("mode-bench").classList.toggle("on", !joint_mode);
  $$(".joint-only").forEach((el) => el.classList.toggle("hidden", !joint_mode));
  $$(".bench-only").forEach((el) => el.classList.toggle("hidden", joint_mode));
  $("bench-banner").classList.toggle("hidden", joint_mode);
  $$(".unit-pos").forEach((el) => (el.textContent = joint_mode ? "°" : "° motor"));
  $$(".unit-speed").forEach((el) => (el.textContent = joint_mode ? "°/s" : "RPM"));
  $$(".unit-pos-text").forEach((el) => (el.textContent = joint_mode ? "degrees" : "motor degrees"));

  // hero
  const counts = r.counts;
  $("pos-label").textContent = joint_mode ? (u.rel ? "Position (no reference)" : "Joint angle") : "Motor shaft";
  $("pos-value").textContent = counts === null ? "—" : fmt(u.pos(counts), 2);
  $("pos-unit").textContent = u.posUnit;
  $("pos-sub").textContent = counts === null ? "no encoder reading" : `${counts} counts · ${fmt(counts / 16384, 3)} motor turns`;
  const rpm = r.speed_rpm;
  $("speed-value").textContent = rpm === null ? "—" : fmt(u.spd(rpm), joint_mode ? 1 : 0);
  $("speed-unit").textContent = u.spdUnit;
  $("speed-sub").textContent = rpm === null ? "" : joint_mode ? `${rpm} RPM at the motor` : `${fmt(rpm * 6 * st.gear_ratio, 1)} °/s at the joint`;

  // chips
  const stale = (k) => fresh[k] !== true;
  chip("chip-enabled", r.enabled === null || stale("enabled") ? "motor ?" : r.enabled ? "motor on" : "motor off", r.enabled && !stale("enabled") ? "ok" : "");
  const moving = MOVING.has(r.status);
  chip("chip-status", r.status === null || stale("status") ? "state ?" : moving ? r.status_name : "stopped", moving ? "busy" : "");
  const in1 = r.io === null || stale("io") ? null : r.io & 1;
  chip("chip-switch", in1 === null ? "switch ?" : in1 ? "switch off" : "switch ON", in1 === 0 ? "ok" : "");
  chip("io-big", in1 === null ? "switch ?" : in1 ? "switch off" : "switch ON", in1 === 0 ? "ok" : "");
  chip("chip-stall", r.stalled === null || stale("stalled") ? "stall ?" : r.stalled ? "STALLED" : "no stall", r.stalled ? "err" : "");
  chip("chip-ref", !joint_mode ? "bench: encoder zero" : noRef ? "no reference" : st.reference.startsWith("homed") ? "homed" : "reference set",
    !joint_mode ? "" : noRef ? "warn" : "ok");

  // stats
  setStat("r-volts", r.millivolts === null ? "n/a" : `${fmt(r.millivolts / 1000, 2)} V`, fresh.power);
  setStat("r-amps", r.milliamps === null ? "n/a" : `${r.milliamps} mA`, fresh.power);
  setStat("r-error", r.angle_error_deg === null ? "—" : `${fmt(r.angle_error_deg, 3)}° motor`, fresh.angle_error);
  const hs = r.home_status, names = { 0: "going", 1: "ok", 2: "failed" };
  setStat("r-home", hs === null ? "—" : hs.map((x) => names[x] ?? x).join(" / "), fresh.home_status);
  setStat("r-version", r.version ?? "—", true);

  // drive panel
  const on = r.enabled === true;
  $("enable-toggle").classList.toggle("on", on);
  $("enable-toggle").setAttribute("aria-pressed", String(on));
  $("enable-toggle").querySelector(".tlabel").textContent = r.enabled === null ? "Motor ?" : on ? "Motor on" : "Motor off";
  $("ref-needed").classList.toggle("hidden", !noRef);
  const busy = testing ? `test: ${st.routine.title}` : st.motion || (moving ? r.status_name : null);
  $("motion-state").textContent = busy || "idle";
  $("motion-state").className = `pill ${busy ? "busy" : ""}`;
  $("speed-cap-text").textContent = `≤ ${fmt(st.speed_cap, joint_mode ? 1 : 0)} ${u.spdUnit}`;
  $("speed-range").max = st.speed_cap;
  $("speed-range").step = joint_mode ? 0.1 : 1;
  if (num("speed-num") > st.speed_cap) setSpeed(st.speed_cap);
  ["goto-btn", "step-minus", "step-plus", "jog-minus", "jog-plus", "run-cw", "run-ccw"].forEach((id) => ($(id).disabled = noRef || testing || !st.online));

  // setup
  $("cap-text").textContent = `≤ ${st.max_joint_speed}`;
  $("bench-ceiling").textContent = `≤ ${st.bench_rpm_ceiling}`;
  $("limits-hint").textContent = joint_mode ? `arm: ${st.hard_limits[0]} to ${st.hard_limits[1]}°` :
    joint ? `= ${fmt(st.limits.bench_min * joint.gear_ratio, 1)} to ${fmt(st.limits.bench_max * joint.gear_ratio, 1)}° at the joint` : "";
  $("config-sign").textContent = st.config_sign > 0 ? "+1" : "−1";
  $("sign-plus").classList.toggle("on", st.sign > 0);
  $("sign-minus").classList.toggle("on", st.sign < 0);
  $("ref-text").textContent = joint_mode ? `Reference: ${st.reference}` : "";

  // recording
  const rec = st.recording !== null;
  $("rec-toggle").textContent = rec ? `■ Stop (${st.recording})` : "● Record";
  $("rec-toggle").classList.toggle("danger", rec);

  // findings, log footer
  const findings = st.findings.join("\n");
  if (findings !== S.findings) {
    S.findings = findings;
    $("findings").value = findings;
    $("findings-count").textContent = st.findings.length ? `(${st.findings.length})` : "";
  }
  const c = st.counts_bus;
  $("bus-counts").textContent = `sent ${c.tx} · received ${c.rx} · missed ${c.missed} · unparsed ${c.bad}` +
    (st.skipped_reads.length ? ` · skipped (no answer): ${st.skipped_reads.join(", ")}` : "");
  $("log-polls").checked = st.log_polls;

  renderRoutine(st);
  if (!S.graphPaused) drawGraph(st, u);
}

function setStat(id, text, fresh) {
  const el = $(id);
  el.textContent = text;
  el.className = fresh ? "" : "stale";
}

function setSpeed(v) {
  $("speed-range").value = v;
  $("speed-num").value = v;
}

function fillDefaults(st, u) {
  const cap = st.speed_cap, [lo, hi] = st.window, joint = st.mode === "joint";
  const here = st.readout.counts === null || u.rel ? null : u.pos(st.readout.counts);
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
  $("sw-speeds").value = (joint ? [0.2, 0.4, 0.6, 0.8, 1].map((f) => round(cap * f, 1)) : [30, 60, 120, 240].filter((s) => s <= cap)).join(" ");
  $("dir-amount").value = joint ? 5 : 90;
  $("dir-speed").value = joint ? round(Math.min(3, cap), 1) : Math.min(30, cap);
  if (joint) {
    $("sz-a").value = round(clamp(st.home_deg + 15, lo, hi), 1);
    $("sz-b").value = round(lo, 1);
    $("sz-speed").value = round(Math.min(2, cap), 1);
    $("ref-deg").value = st.home_deg;
    $("ref-quick").value = st.home_deg;
    $("home-rpm").value = Math.max(1, Math.floor(Math.min(5, cap) / 6 / st.gear_ratio));
  } else {
    $("sz-a").value = 180; $("sz-b").value = -180; $("sz-speed").value = Math.min(30, cap);
    $("home-rpm").value = 30;
  }
  homeHint();
}

function homeHint() {
  const st = S.st, rpm = num("home-rpm");
  $("home-speed-hint").textContent = st && Number.isFinite(rpm) ? `= ${fmt(rpm * 6 * st.gear_ratio, 1)} °/s` : "";
}

// --- graph ----------------------------------------------------------------------------------------------------------

function niceStep(range, ticks) {
  const raw = range / ticks, mag = 10 ** Math.floor(Math.log10(raw || 1)), n = raw / mag;
  return (n < 1.5 ? 1 : n < 3 ? 2 : n < 7 ? 5 : 10) * mag;
}

function prep(canvas) {
  const dpr = window.devicePixelRatio || 1, w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  return { ctx, w, h };
}

const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

function pane(ctx, box, xs, ys, xr, yr, color, label, lines = []) {
  const { x, y, w, h } = box;
  let [y0, y1] = yr;
  if (!(y1 - y0 > 1e-9)) { y0 -= 1; y1 += 1; }
  const pad = (y1 - y0) * 0.1;
  y0 -= pad; y1 += pad * 2.2;
  const sx = (v) => x + ((v - xr[0]) / (xr[1] - xr[0])) * w;
  const sy = (v) => y + h - ((v - y0) / (y1 - y0)) * h;
  ctx.lineWidth = 1;
  ctx.strokeStyle = css("--grid");
  ctx.fillStyle = css("--faint");
  ctx.font = "11px system-ui";
  const step = niceStep(y1 - y0, 4);
  const digits = clamp(-Math.floor(Math.log10(step)), 0, 5);   // enough decimals to tell the ticks apart
  for (let v = Math.ceil(y0 / step) * step; v <= y1; v += step) {
    ctx.beginPath(); ctx.moveTo(x, sy(v)); ctx.lineTo(x + w, sy(v)); ctx.stroke();
    ctx.fillText(fmt(Math.abs(v) < step / 1e6 ? 0 : v, digits), x + w + 6, sy(v) + 4);
  }
  for (const [v, c] of lines) {
    if (v < y0 || v > y1) continue;
    ctx.save(); ctx.setLineDash([5, 4]); ctx.strokeStyle = c;
    ctx.beginPath(); ctx.moveTo(x, sy(v)); ctx.lineTo(x + w, sy(v)); ctx.stroke(); ctx.restore();
  }
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  let started = false;
  for (let i = 0; i < xs.length; i++) {
    if (ys[i] === null || Number.isNaN(ys[i])) { started = false; continue; }
    if (!started) { ctx.moveTo(sx(xs[i]), sy(ys[i])); started = true; } else ctx.lineTo(sx(xs[i]), sy(ys[i]));
  }
  ctx.stroke();
  if (label) {
    ctx.fillStyle = color;
    ctx.font = "600 12px system-ui";
    ctx.fillText(label, x + 4, y + 14);
  }
}

function drawGraph(st, u) {
  const { ctx, w, h } = prep($("graph"));
  if (!S.samples.length) {
    ctx.fillStyle = css("--muted"); ctx.font = "13px system-ui";
    ctx.fillText(st.online ? "waiting for the encoder …" : "board offline: no readings", 12, h / 2);
    return;
  }
  const span = S.graphSpan, tEnd = S.samples[S.samples.length - 1][1];
  const pts = S.samples.filter((s) => s[1] >= tEnd - span && s[2] !== null);
  const xs = pts.map((s) => s[1] - tEnd), pos = pts.map((s) => u.pos(s[2]));
  const spd = pts.map((s) => (s[3] === null ? null : u.spd(s[3])));
  const right = 54, gap = 16, ph = (h - gap - 16) * 0.64, sh = h - gap - 16 - ph;
  const ok = (a) => a.filter((v) => v !== null && Number.isFinite(v));
  const pr = ok(pos), sr = ok(spd);
  const lines = u.rel ? [] : [[st.window[0], css("--warn")], [st.window[1], css("--warn")]];
  pane(ctx, { x: 4, y: 2, w: w - right - 4, h: ph }, xs, pos, [-span, 0], [Math.min(...pr), Math.max(...pr)], css("--pos"),
    `position (${u.posUnit})`, lines);
  pane(ctx, { x: 4, y: ph + gap, w: w - right - 4, h: sh }, xs, spd, [-span, 0], [Math.min(0, ...sr), Math.max(0, ...sr)],
    css("--spd"), `speed (${u.spdUnit})`);
  ctx.fillStyle = css("--faint"); ctx.font = "11px system-ui";
  const tstep = niceStep(span, 6);
  for (let t = -Math.floor(span / tstep) * tstep; t <= 0; t += tstep) {
    ctx.fillText(`${fmt(t, 0)} s`, 4 + ((t + span) / span) * (w - right - 4) - 10, h - 2);
  }
}

function drawChart(canvas, chart) {
  const { ctx, w, h } = prep(canvas);
  const names = Object.keys(chart.series).filter((k) => chart.series[k].length);
  if (!names.length) return;
  const all = names.flatMap((k) => chart.series[k]);
  const xr = [Math.min(...all.map((p) => p[0])), Math.max(...all.map((p) => p[0]))];
  if (xr[1] - xr[0] < 1e-9) { xr[0] -= 1; xr[1] += 1; }
  const yr = [Math.min(...all.map((p) => p[1])), Math.max(...all.map((p) => p[1]))];
  const colors = [css("--pos"), css("--spd"), css("--ok"), css("--warn"), css("--err")];
  names.forEach((k, i) => {
    const pts = chart.series[k];
    pane(ctx, { x: 4, y: 18, w: w - 52, h: h - 36 }, pts.map((p) => p[0]), pts.map((p) => p[1]), xr, yr, colors[i % 5], "");
    ctx.fillStyle = colors[i % 5]; ctx.font = "600 11px system-ui";
    ctx.fillText(k, 6 + i * 150, 11);
  });
  ctx.fillStyle = css("--faint"); ctx.font = "11px system-ui";
  ctx.fillText(`${chart.x}: ${fmt(xr[0], 2)} … ${fmt(xr[1], 2)}`, 6, h - 2);
}

// --- tests ----------------------------------------------------------------------------------------------------------

function showTest(kind) {
  $$(".testform").forEach((f) => f.classList.toggle("hidden", f.dataset.kind !== kind));
  $("test-about").textContent = TESTS[kind] || "";
}

function renderRoutine(st) {
  const r = st.routine, dlg = $("question");
  $("routine-cancel").disabled = !active(r);
  $("test-start").disabled = active(r);
  if (!r) return;
  $("routine-box").classList.remove("hidden");
  $("routine-title").textContent = r.title;
  $("routine-status").textContent = r.status;
  $("routine-status").className = `pill ${r.status === "done" ? "ok" : r.status === "failed" ? "err" : r.status === "cancelled" ? "warn" : "busy"}`;
  $("routine-progress").textContent = active(r) ? r.progress : "";
  say($("routine-error"), r.error || "", r.error ? false : undefined);
  const key = `${r.started}|${r.status}|${r.summary.length}`;
  if (key !== S.routineKey) {
    S.routineKey = key;
    $("routine-summary").replaceChildren(...r.summary.map((line) => Object.assign(document.createElement("li"), { textContent: line })));
  }
  const tkey = `${r.started}|${r.row_count}`;
  if (tkey !== S.routineTable) {
    S.routineTable = tkey;
    const head = document.createElement("tr");
    r.columns.forEach((c) => head.append(Object.assign(document.createElement("th"), { textContent: c })));
    $("routine-table").querySelector("thead").replaceChildren(head);
    $("routine-table").querySelector("tbody").replaceChildren(...r.rows.slice(-400).map((row) => {
      const tr = document.createElement("tr");
      row.forEach((v) => tr.append(Object.assign(document.createElement("td"), { textContent: v === null ? "—" : v })));
      return tr;
    }));
  }
  $("routine-chart").classList.toggle("hidden", !r.chart);
  if (r.chart) drawChart($("routine-chart"), r.chart);
  if (r.question) {
    const qkey = `${r.started}|${r.question.text}`;
    if (qkey !== S.questionKey) {
      S.questionKey = qkey;
      $("question-title").textContent = r.title;
      $("question-text").textContent = r.question.text;
      $("question-options").replaceChildren(...r.question.options.map((opt, i) => {
        const b = document.createElement("button");
        b.className = i === 0 ? "btn" : "btn ghost";
        b.textContent = opt;
        b.onclick = () => { api("routine_answer", { answer: opt }); dlg.close(); };
        return b;
      }));
      if (!dlg.open) dlg.showModal();
    }
  } else {
    S.questionKey = null;
    if (dlg.open) dlg.close();
  }
}

function testParams(kind) {
  switch (kind) {
    case "repeat": return { a: num("rep-a"), b: num("rep-b"), cycles: int("rep-cycles"), speed: num("rep-speed"), acc: int("rep-acc"), dwell: num("rep-dwell") };
    case "step": return { step: num("step-size"), speed: num("step-speed"), acc: int("step-acc"), duration: num("step-duration") };
    case "sweep": return { a: num("sw-a"), b: num("sw-b"), speeds: $("sw-speeds").value, acc: int("sw-acc") };
    case "direction": return { amount: num("dir-amount"), speed: num("dir-speed") };
    case "switch": return { a: num("sz-a"), b: num("sz-b"), speed: num("sz-speed"), acc: int("acc-num") };
    case "home": {
      const mode = $("home-mode").value;
      return { trigger: int("home-trig"), direction: int("home-dir"), rpm: int("home-rpm"), end_limit: int("home-end"),
               hm_mode: mode === "none" ? null : parseInt(mode, 10) };
    }
    default: return {};
  }
}

async function startTest(kind, el) {
  const r = await api("routine", { kind, params: testParams(kind) });
  if (!r.ok) { toast(r.error, "err"); say(el, r.error, false); }
}

function homeNow(el) {
  const st = S.st;
  const dir = $("home-dir").value === "1" ? "CCW" : "CW";
  if (confirm(`Home ${st.joint}: it turns ${dir} at ${$("home-rpm").value} RPM until the switch turns on.\n\nWatch it, and press STOP if it heads the wrong way. (Homing settings: Setup tab.)`)) {
    startTest("home", el);
  }
}

// --- log ------------------------------------------------------------------------------------------------------------

function addLog(entries) {
  const tbody = $("log-table").querySelector("tbody");
  const rows = entries.slice().reverse().map((e) => {
    const tr = document.createElement("tr");
    tr.className = e.direction;
    const t = new Date(e.t * 1000);
    [`${t.toLocaleTimeString([], { hour12: false })}.${String(t.getMilliseconds()).padStart(3, "0")}`,
      e.direction === "tx" ? "→" : e.direction === "rx" ? "←" : "•",
      e.can_id === null ? "" : `0x${e.can_id.toString(16).padStart(2, "0")}`, e.hex, e.text].forEach((c, i) => {
      tr.append(Object.assign(document.createElement("td"), { textContent: c, className: i === 1 ? "dir" : i === 4 ? "text" : "" }));
    });
    return tr;
  });
  tbody.prepend(...rows);
  while (tbody.children.length > 1500) tbody.lastChild.remove();
}

// --- jog, stop ------------------------------------------------------------------------------------------------------

function jogStart(dir) {
  if (S.jogDir === dir) return;
  jogStop(true);
  S.jogDir = dir;
  $(dir > 0 ? "jog-plus" : "jog-minus").classList.add("held");
  const send = async () => {
    if (S.jogDir !== dir) return;
    const r = await api("jog", { direction: dir, speed: num("speed-num"), acc: int("acc-num") });
    if (!r.ok && S.jogDir === dir) { say($("motion-msg"), r.error, false); jogStop(true); }
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

const typing = (e) => e.target && ["INPUT", "TEXTAREA", "SELECT"].includes(e.target.tagName);

async function halt() {
  jogStop(false);
  if ($("question").open) $("question").close();
  const r = await api("halt");
  toast(r.ok ? r.message : r.error, r.ok ? "warn" : "err");
}

// --- wiring ---------------------------------------------------------------------------------------------------------

function bind() {
  $("halt").onclick = halt;
  $("q-stop").onclick = halt;   // the dialog's backdrop covers the panel's STOP
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
  $("question").addEventListener("cancel", (e) => { e.preventDefault(); halt(); });   // Esc in the dialog is STOP too

  $("mode-joint").onclick = () => act("mode", { mode: "joint" }, $("motion-msg"), "joint mode");
  $("mode-bench").onclick = () => act("mode", { mode: "bench" }, $("motion-msg"), "bench mode");
  $("enable-toggle").onclick = () => {
    const on = S.st && S.st.readout.enabled === true;
    if (on && !confirm("Turning the motor off frees the shaft: a loaded joint falls. Support the arm first.\n\nTurn it off?")) return;
    act("enable", { on: !on }, $("motion-msg"), on ? "motor off" : "motor on");
  };
  $("ref-home").onclick = () => homeNow($("motion-msg"));
  $("ref-quick-set").onclick = () => act("reference", { deg: num("ref-quick") }, $("motion-msg"), "reference set");
  bindJog("jog-minus", -1);
  bindJog("jog-plus", 1);
  $("speed-range").oninput = () => ($("speed-num").value = $("speed-range").value);
  $("speed-num").oninput = () => ($("speed-range").value = $("speed-num").value);
  $("acc-range").oninput = () => ($("acc-num").value = $("acc-range").value);
  $("acc-num").oninput = () => ($("acc-range").value = $("acc-num").value);
  const move = (extra) => ({ speed: num("speed-num"), acc: int("acc-num"), ...extra });
  $("goto-btn").onclick = () => act("move", move({ value: num("goto-value") }), $("motion-msg"), "moving");
  $("goto-value").addEventListener("keydown", (e) => { if (e.key === "Enter") $("goto-btn").click(); });
  $("step-minus").onclick = () => act("step", move({ delta: -num("step-value") }), $("motion-msg"), "moving");
  $("step-plus").onclick = () => act("step", move({ delta: num("step-value") }), $("motion-msg"), "moving");
  $("run-cw").onclick = () => act("run", { cw: true, rpm: num("run-rpm"), acc: int("acc-num") }, $("motion-msg"), "running CW");
  $("run-ccw").onclick = () => act("run", { cw: false, rpm: num("run-rpm"), acc: int("acc-num") }, $("motion-msg"), "running CCW");
  $("soft-stop").onclick = () => { jogStop(false); act("stop", {}, $("motion-msg"), "stopping"); };

  for (const b of $$("#graph-window button")) {
    b.onclick = () => { S.graphSpan = Number(b.dataset.s); $$("#graph-window button").forEach((x) => x.classList.toggle("on", x === b)); };
  }
  $("graph-pause").onclick = () => { S.graphPaused = !S.graphPaused; $("graph-pause").textContent = S.graphPaused ? "Resume" : "Pause"; };
  $("rec-toggle").onclick = () => api("record", { on: !(S.st && S.st.recording !== null) });
  $("rec-download").onclick = () => download("/api/record.csv", `recording-${S.st ? S.st.joint : "board"}-${stamp()}.csv`);

  for (const tab of $$("#tabs button")) {
    tab.onclick = () => {
      $$("#tabs button").forEach((t) => t.classList.toggle("active", t === tab));
      $$(".pane").forEach((p) => p.classList.toggle("hidden", p.id !== `tab-${tab.dataset.tab}`));
    };
  }

  $("board-apply").onclick = () => act("select", { joint: $("joint-select").value, can_id: int("canid-input") }, $("motion-msg"));
  $("joint-select").onchange = () => {
    const j = S.st && S.st.joints.find((x) => x.name === $("joint-select").value);
    if (j && j.can_id) $("canid-input").value = j.can_id;
  };
  $("scan-go").onclick = async () => {
    $("scan-result").textContent = "scanning ids 1–16 …";
    const r = await api("scan", { first: 1, last: 16 });
    $("scan-result").textContent = !r.ok ? r.error : !r.found.length ? "no board answered (ids 1–16)" :
      r.found.map((f) => `id ${f.can_id}: ${f.board}, fw ${f.firmware}`).join(" · ");
  };
  $("limits-apply").onclick = () => {
    const limits = S.st.mode === "joint" ?
      { joint_min: num("lim-jmin"), joint_max: num("lim-jmax"), joint_speed: num("lim-jspeed") } :
      { bench_min: num("lim-bmin"), bench_max: num("lim-bmax"), bench_rpm: num("lim-brpm") };
    act("limits", { limits }, $("limits-msg"), "applied").then((r) => { if (r.ok) S.context = null; });
  };
  $("sign-plus").onclick = () => act("sign", { sign: 1 }, $("ref-msg"), "sign +1");
  $("sign-minus").onclick = () => act("sign", { sign: -1 }, $("ref-msg"), "sign −1");
  $("ref-set").onclick = () => act("reference", { deg: num("ref-deg") }, $("ref-msg"), "reference set");
  $("ref-clear").onclick = () => act("clear_reference", {}, $("ref-msg"), "reference cleared");
  $("home-rpm").oninput = homeHint;
  $("home-go").onclick = () => homeNow($("homing-msg"));
  $("zero-set").onclick = () => { if (confirm("Make this position the board's encoder zero (0x92)?")) act("set_zero", {}, $("homing-msg"), "zero set"); };
  $("stall-release").onclick = () => act("release_stall", {}, $("homing-msg"), "stall released");
  $("calibrate").onclick = () => { if (confirm("Calibration turns the motor by itself: it must be unloaded, out of the arm. Continue?")) startTest("calibrate", $("homing-msg")); };

  $("test-kind").onchange = () => showTest($("test-kind").value);
  $("test-start").onclick = () => startTest($("test-kind").value, null);
  $("routine-cancel").onclick = () => api("routine_cancel");
  $("routine-csv").onclick = () => download("/api/routine.csv", `${S.st && S.st.routine ? S.st.routine.kind : "test"}-${S.st ? S.st.joint : ""}-${stamp()}.csv`);
  $("findings-copy").onclick = async () => {
    try { await navigator.clipboard.writeText($("findings").value); toast("Findings copied"); } catch { $("findings").select(); }
  };
  $("findings-clear").onclick = () => api("clear_findings");

  $("settings-read").onclick = async () => {
    say($("settings-msg"), "reading …");
    const r = await api("read_settings");
    if (!r.ok) { say($("settings-msg"), r.error, false); return; }
    const names = { "82": "work mode", "83": "current mA", "84": "microsteps", "88": "stall protection", "89": "MPlyer",
      "8A": "bitrate (2 = 500K)", "8B": "CAN id", "8C": "response mode", "9E": "limit remap" };
    $("settings-table").querySelector("tbody").replaceChildren(...Object.entries(r.settings).map(([code, v]) => {
      const tr = document.createElement("tr");
      const shown = v === null ? "no reply" : v.value === null ? "not readable" : code === "82" ? `${v.value} ${S.st.work_modes[v.value] ?? ""}` : String(v.value);
      [names[code] ?? `0x${code}`, shown].forEach((c) => tr.append(Object.assign(document.createElement("td"), { textContent: c })));
      const input = $(`set-${code}`);
      if (input && v && v.value !== null) input.value = String(v.value);
      return tr;
    }));
    say($("settings-msg"), "read", true);
  };
  for (const b of $$("[data-set]")) {
    b.onclick = () => {
      const code = b.dataset.set, value = $(`set-${code}`).value, info = S.st.settings[code];
      let warn = "";
      if (code === "8B") warn = "\n\nThe dashboard follows the board to its new id.";
      if (code === "82" && value !== "5") warn = "\n\nOnly the bus modes (3–5) take motion commands.";
      if (confirm(`Write ${info.name} = ${value} to board ${S.st.can_id}? It is saved on the board.${warn}`)) {
        act("setting", { code: `0x${code}`, value }, $("settings-msg"));
      }
    };
  }
  for (const b of $$("[data-action]")) {
    b.onclick = () => {
      const code = b.dataset.action;
      const text = { "3D": "Release the stall protection?", "41": "Restart the board (settings kept)?",
        "3F": "Factory reset? The CAN id goes back to 1, the work mode to CR_vFOC (a pulse mode) and every setting to the factory's." }[code];
      if (confirm(text)) act("setting", { code: `0x${code}` }, $("settings-msg"));
    };
  }
  $("raw-send").onclick = async () => {
    const r = await api("raw", { hex: $("raw-hex").value });
    say($("raw-msg"), r.ok ? `${r.sent} → ${r.reply === null ? "no reply" : r.reply}` : r.error, r.ok);
  };
  $("raw-hex").addEventListener("keydown", (e) => { if (e.key === "Enter") $("raw-send").click(); });
  $("log-polls").onchange = () => api("log_polls", { on: $("log-polls").checked });
  $("log-pause").onclick = () => { S.logPaused = !S.logPaused; $("log-pause").textContent = S.logPaused ? "Resume" : "Pause"; };
  $("log-clear").onclick = () => $("log-table").querySelector("tbody").replaceChildren();
}

const ping = new Worker("/ping.js");
ping.postMessage(TOKEN);
bind();
poll();
