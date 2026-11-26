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
  logPaused: false, graphPaused: false, graphSpan: 30, speedSrc: "encoder", showCmd: true, pidLoop: "vfoc",
  context: null, jogDir: 0, jogTimer: null, activeKey: null, cards: {}, jogMotor: null,
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
    const res = await fetch(`/api/state?samples=${S.sampleSeq}&log=${S.logSeq}&notices=${S.noticeSeq}&active=${S.activeKey ?? ""}`,
      { headers: { "X-Dashboard-Token": TOKEN } });
    if (res.status === 403) throw new Error("the dashboard restarted: reload the page");
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const st = await res.json();
    if (st.active !== S.activeKey) {   // another motor is active: its samples came from the start; its panels refill
      S.activeKey = st.active;
      S.samples = [];
      S.sampleSeq = 0;
      S.context = null;
      S.routineKey = S.routineTable = S.questionKey = S.findings = null;
    }
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
    $("pf-curve").innerHTML = st.curves.map((c) => `<option value="${c}">${c}</option>`).join("");
    $("set-9B").innerHTML = Array.from({ length: 9 }, (_, i) => `<option value="${i}">${(i + 1) * 10} %</option>`).join("");
    $("set-9B").value = "4";
    $("add-joint").innerHTML = st.joints.map((j) => `<option value="${j.name}">${j.name}</option>`).join("");
    const free = st.joints.find((j) => !st.motors.some((m) => m.joint === j.name)) || st.joints[0];
    $("add-joint").value = free.name;
    $("add-id").value = free.can_id ?? "";
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
  // the big number carries a short unit; "from encoder zero" leads the line under it (a long unit wrapped there)
  $("pos-unit").textContent = u.rel ? "°" : u.posUnit;
  $("pos-sub").textContent = counts === null ? "no encoder reading" :
    `${u.rel ? "from encoder zero · " : ""}${counts} counts · ${fmt(counts / 16384, 3)} motor turns`;
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
  const L = st.limits;
  const span = (v) => (v === null ? "open" : fmt(v, 1));
  chip("chip-limits", !L.on ? "limits OFF" : L.min === null && L.max === null ? "limits on · none set" : `limits ${span(L.min)} … ${span(L.max)}`,
    !L.on ? "warn" : L.min === null && L.max === null ? "" : "ok");

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
  renderLimits(st);
  renderProfile(st);
  const busy = testing ? `test: ${st.routine.title}` : st.motion || (moving ? r.status_name : null);
  $("motion-state").textContent = busy || "idle";
  $("motion-state").className = `pill ${busy ? "busy" : ""}`;
  $("speed-cap-text").textContent = `≤ ${fmt(st.speed_cap, joint_mode ? 1 : 0)} ${u.spdUnit}`;
  $("speed-range").max = st.speed_cap;
  $("speed-range").step = joint_mode ? 0.1 : 1;
  if (num("speed-num") > st.speed_cap) setSpeed(st.speed_cap);
  ["goto-btn", "step-minus", "step-plus", "jog-minus", "jog-plus", "run-cw", "run-ccw"].forEach((id) => ($(id).disabled = testing || !st.online));

  // setup
  $("cap-text").textContent = `max ${st.max_joint_speed}`;
  $("bench-ceiling").textContent = `max ${st.bench_rpm_ceiling}`;
  $("limits-hint").textContent = joint_mode ? `The arm's own range: ${st.hard_limits[0]} to ${st.hard_limits[1]}° (Use arm range, under Limits, sets it once the joint has a reference).` :
    joint ? `1 motor turn = ${fmt(360 * joint.gear_ratio, 1)}° at ${st.joint}.` : "";
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
  renderMotors(st);
  if (!S.graphPaused) drawGraph(st, u);
}

// --- the motor cards ------------------------------------------------------------------------------------------------

function renderMotors(st) {
  const box = $("motor-cards");
  const keys = st.motors.map((m) => m.key);
  for (const key of Object.keys(S.cards)) {
    if (!keys.includes(key)) { S.cards[key].remove(); delete S.cards[key]; }
  }
  st.motors.forEach((m, i) => {
    let card = S.cards[m.key];
    if (!card) {
      card = document.createElement("div");
      card.className = "mcard";
      card.dataset.key = m.key;
      card.innerHTML = `<div class="mc-head"><span class="mc-num"></span><span class="dot"></span><span class="mc-name"></span>` +
        `<span class="mc-id"></span><button class="mc-x" title="Take this motor off the page">×</button></div>` +
        `<div class="mc-value"><span class="v"></span><span class="u"></span></div><div class="mc-sub"></div>` +
        `<div class="mc-jog"><button data-dir="-1" title="Jog while held">◀ Jog</button><button data-dir="1" title="Jog while held">Jog ▶</button></div>`;
      S.cards[m.key] = card;
      bindCard(card, m.key);
    }
    if (box.children[i] !== card) box.insertBefore(card, box.children[i] || null);
    card.classList.toggle("active", m.active);
    card.querySelector(".mc-num").textContent = i + 1;
    card.querySelector(".dot").classList.toggle("on", m.online);
    card.querySelector(".mc-name").textContent = m.joint;
    card.querySelector(".mc-id").textContent = `id ${m.can_id}`;
    card.querySelector(".mc-x").classList.toggle("hidden", m.active || st.motors.length === 1);
    card.querySelector(".v").textContent = m.value === null ? "—" : fmt(m.value, 1);
    card.querySelector(".u").textContent = m.mode === "joint" && !m.referenced ? "° from zero" : m.unit;
    const state = !m.online ? `<span class="bad">offline</span>` : m.stalled ? `<span class="bad">STALLED</span>` :
      m.testing ? `<span class="busy">test</span>` : m.moving ? `<span class="busy">moving</span>` : "stopped";
    const power = m.enabled === null ? "?" : m.enabled ? `<span class="on">on</span>` : "off";
    card.querySelector(".mc-sub").innerHTML = `id ${m.can_id} · ${m.speed === null ? "—" : fmt(m.speed, m.mode === "joint" ? 1 : 0)} ${m.speed_unit} · ${power} · ${state}` +
      (m.limits_on ? "" : ` · <span class="bad">limits off</span>`);
  });
  $("halt").textContent = st.motors.length > 1 ? "STOP ALL" : "STOP";
  $("together-open").disabled = st.motors.length < 2;
}

async function selectMotor(key) {
  if (key === S.activeKey) return true;
  const r = await api("select_motor", { key });
  if (!r.ok) { toast(r.error, "err"); return false; }
  return true;
}

function bindCard(card, key) {
  card.addEventListener("click", (e) => { if (!e.target.closest("button")) selectMotor(key); });
  card.querySelector(".mc-x").onclick = async () => {
    if (!confirm("Take this motor off the page? (Its board keeps running as it is; STOP still reaches only the motors on the page.)")) return;
    const r = await api("remove_motor", { key });
    if (!r.ok) toast(r.error, "err");
  };
  for (const b of card.querySelectorAll(".mc-jog button")) {
    const dir = Number(b.dataset.dir);
    b.addEventListener("pointerdown", async (e) => {
      b.setPointerCapture(e.pointerId);
      b.classList.add("held");
      if (key !== S.activeKey && !(await selectMotor(key))) { b.classList.remove("held"); return; }
      jogStart(dir, key);   // this card's motor, whatever the page still shows
    });
    b.addEventListener("pointerup", () => { b.classList.remove("held"); jogEnd(); });
    b.addEventListener("pointercancel", () => { b.classList.remove("held"); jogEnd(); });
  }
}

function openTogether() {
  const st = S.st;
  $("together-rows").replaceChildren(...st.motors.map((m) => {
    const tr = document.createElement("tr");
    tr.innerHTML = `<td><b></b> <span class="muted small"></span></td><td class="muted small"></td><td><input type="number" step="0.1"></td><td class="muted small"></td>`;
    tr.querySelector("b").textContent = m.joint;
    tr.querySelector("span").textContent = `id ${m.can_id}`;
    tr.children[1].textContent = m.value === null ? "now —" : `now ${fmt(m.value, 2)}`;
    const input = tr.querySelector("input");
    input.dataset.key = m.key;
    input.value = m.value === null ? "" : round(m.value, 2);
    tr.children[3].textContent = m.mode === "joint" && !m.referenced ? "° from zero" : m.unit;
    return tr;
  }));
  $("together-acc").value = $("acc-num").value;
  say($("together-msg"), "");
  $("together").showModal();
}

function renderProfile(st) {
  const p = st.profile, sc = p.kind === "scurve";
  const set = (id, v) => { if (document.activeElement !== $(id)) $(id).value = v; };
  $$("#profile-kind button").forEach((b) => b.classList.toggle("on", b.dataset.kind === p.kind));
  $("acc-row").classList.toggle("hidden", sc);
  $("ease-row").classList.toggle("hidden", !sc);
  $("smooth-hint").textContent = sc ? `${p.curve} · ${p.ease_s} s to full speed · more in Tuning` : "acc: 0 = no ramp, larger = steeper";
  set("pf-kind", p.kind);
  set("pf-curve", p.curve);
  set("pf-ease", p.ease_s);
  set("pf-rate", p.rate_hz);
  set("pf-follow", p.follow_acc);
  set("pf-acc", $("acc-num").value);
  if (![$("ease-num"), $("ease-range")].includes(document.activeElement)) {
    $("ease-num").value = p.ease_s;
    $("ease-range").value = p.ease_s;
  }
  $$("#poll-ms button").forEach((b) => b.classList.toggle("on", Number(b.dataset.ms) === st.poll_ms));
  const m = st.microsteps;
  $("micro-note").textContent = m === null ? "Microsteps: not read yet (16 assumed)." :
    `Microsteps ${m}: the board's speed step is ${st.speed_factor === 1 ? "1 RPM" : `${st.speed_factor} RPM (${fmt(1 / st.speed_factor, 0)}× finer)`}` +
    (st.mode === "joint" ? `, ${fmt(st.speed_factor * 6 * st.gear_ratio, 3)} °/s at ${st.joint}.` : ".");
}

function renderLimits(st) {
  const L = st.limits;
  $("limits-box").classList.toggle("off", !L.on);
  const sw = $("limits-toggle");
  sw.classList.toggle("on", L.on);
  sw.setAttribute("aria-pressed", String(L.on));
  sw.querySelector(".slabel").textContent = L.on ? "ON" : "OFF";
  for (const side of ["min", "max"]) {
    const input = $(`lim-${side}`), v = L[side];
    if (document.activeElement !== input) input.value = v === null ? "" : round(v, 2);
    input.title = v === null ? "no limit this side" : "";
    $(`lim-${side}-clear`).disabled = v === null;
  }
  const arm = $("lim-arm"), [lo, hi] = st.hard_limits;
  arm.disabled = !L.arm_ready;
  arm.title = L.arm_ready ? `Set the limits to ${st.joint}'s range: ${lo} to ${hi}°` :
    "Needs a reference: home the joint, or set where it is (Setup)";
  $("limits-note").textContent = !L.on ? "Off: free movement. Only the speed cap and STOP apply." :
    L.min === null && L.max === null ? `None set: move to an end and press Here${L.arm_ready ? ", or Use arm range" : ""}.` : "";
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
  const cap = st.speed_cap, joint = st.mode === "joint";
  const here = st.readout.counts === null ? null : u.pos(st.readout.counts);
  const lo = st.window[0] ?? (here ?? 0) - (joint ? 40 : 360), hi = st.window[1] ?? (here ?? 0) + (joint ? 40 : 360);
  const speed = joint ? Math.min(5, cap) : Math.min(60, cap);
  setSpeed(round(speed, 1));
  $("lim-jspeed").value = st.limits.joint_speed;
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
    $("home-rpm").value = Math.max(1, Math.floor(Math.min(5, cap) / 6 / st.gear_ratio / st.speed_factor));
  } else {
    $("sz-a").value = 180; $("sz-b").value = -180; $("sz-speed").value = Math.min(30, cap);
    $("home-rpm").value = 30;
  }
  homeHint();
}

function homeHint() {
  const st = S.st, value = num("home-rpm");   // the board's speed value: RPM x speed_factor
  $("home-speed-hint").textContent = st && Number.isFinite(value) ? `= ${fmt(value * st.speed_factor * 6 * st.gear_ratio, 1)} °/s` : "";
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

// One plot: a y grid, dashed level `lines`, and each of `series` ({xs, ys, color, dash}) on shared axes.
function pane(ctx, box, series, xr, yr, label, lines = []) {
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
  for (const { xs, ys, color, dash } of series) {
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = dash ? 1.5 : 2;
    if (dash) ctx.setLineDash([6, 4]);
    ctx.beginPath();
    let started = false;
    for (let i = 0; i < xs.length; i++) {
      if (ys[i] === null || Number.isNaN(ys[i])) { started = false; continue; }
      if (!started) { ctx.moveTo(sx(xs[i]), sy(ys[i])); started = true; } else ctx.lineTo(sx(xs[i]), sy(ys[i]));
    }
    ctx.stroke();
    ctx.restore();
  }
  if (label) {
    ctx.fillStyle = series.length ? series[0].color : css("--muted");
    ctx.font = "600 12px system-ui";
    ctx.fillText(label, x + 4, y + 14);
  }
}

// Speed worked out from the encoder: the position's slope over about 50 ms each side, in the mode's speed unit.
function encoderSpeed(pts, pos, joint) {
  const n = pts.length, out = new Array(n).fill(null);
  if (n < 2) return out;
  const spacing = (pts[n - 1][1] - pts[0][1]) / (n - 1);
  const k = clamp(Math.round(0.05 / (spacing || 0.02)), 3, 25);
  for (let i = 0; i < n; i++) {
    const a = Math.max(0, i - k), b = Math.min(n - 1, i + k), dt = pts[b][1] - pts[a][1];
    if (dt > 0) out[i] = (pos[b] - pos[a]) / dt / (joint ? 1 : 6);   // deg/s at the joint; motor deg/s / 6 = RPM
  }
  return out;
}

// [lo, hi] widened about its middle to at least `span`.
function atLeast([lo, hi], span) {
  const mid = (lo + hi) / 2;
  return hi - lo >= span ? [lo, hi] : [mid - span / 2, mid + span / 2];
}

// The graph's tightest zoom. A still motor's encoder dithers by one count (0.022° at the motor), and an axis fitted
// to the data alone zooms in until that dither fills the pane. The position axis spans at least 2° in bench mode and,
// in joint mode, 0.1° or 2° of motor turn through the gear, whichever is more; the speed axis at least ±1 RPM in bench
// mode and, in joint mode, ±1 °/s or ±1 RPM at the motor, whichever is more. One count is then at most about 1/90 of
// the axis on every joint, and a still motor draws flat.
const POS_FLOOR = { motor: 2, joint: 0.1 }, SPD_FLOOR = 1;

function drawGraph(st, u) {
  const { ctx, w, h } = prep($("graph"));
  if (!S.samples.length) {
    ctx.fillStyle = css("--muted"); ctx.font = "13px system-ui";
    ctx.fillText(st.online ? "waiting for the encoder …" : "board offline: no readings", 12, h / 2);
    return;
  }
  const joint = st.mode === "joint";
  const span = S.graphSpan, tEnd = S.samples[S.samples.length - 1][1];
  const pts = S.samples.filter((s) => s[1] >= tEnd - span && s[2] !== null);
  const xs = pts.map((s) => s[1] - tEnd), pos = pts.map((s) => u.pos(s[2]));
  const spd = S.speedSrc === "encoder" ? encoderSpeed(pts, pos, joint) : pts.map((s) => (s[3] === null ? null : u.spd(s[3])));
  // the commanded speed: RPM signed with the encoder's count -> the shown unit, signed with the shown position
  const toUnit = (c) => (joint ? st.sign * c * 6 * st.gear_ratio : c);
  const cmd = pts.map((s) => (s[8] === null || s[8] === undefined ? null : toUnit(s[8])));
  const showCmd = S.showCmd && cmd.some((v) => v !== null);
  const right = 54, gap = 16, ph = (h - gap - 16) * 0.64, sh = h - gap - 16 - ph;
  const ok = (a) => a.filter((v) => v !== null && Number.isFinite(v));
  const pr = ok(pos), sr = ok(spd).concat(showCmd ? ok(cmd) : []);
  const lines = st.window.filter((v) => v !== null).map((v) => [v, css("--warn")]);
  const posFloor = joint ? Math.max(POS_FLOOR.joint, POS_FLOOR.motor * st.gear_ratio) : POS_FLOOR.motor;
  const spdFloor = joint ? Math.max(SPD_FLOOR, 6 * st.gear_ratio) : SPD_FLOOR;   // 1 RPM = 6 °/s at the motor
  pane(ctx, { x: 4, y: 2, w: w - right - 4, h: ph }, [{ xs, ys: pos, color: css("--pos") }], [-span, 0],
    atLeast([Math.min(...pr), Math.max(...pr)], posFloor), `position (${u.posUnit})`, lines);
  const speedSeries = [{ xs, ys: spd, color: css("--spd") }];
  if (showCmd) speedSeries.push({ xs, ys: cmd, color: css("--ok"), dash: true });
  pane(ctx, { x: 4, y: ph + gap, w: w - right - 4, h: sh }, speedSeries, [-span, 0], [Math.min(-spdFloor, ...sr), Math.max(spdFloor, ...sr)],
    `speed (${u.spdUnit}, ${S.speedSrc === "encoder" ? "from the encoder" : "board"})${showCmd ? "   - - commanded" : ""}`);
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
    pane(ctx, { x: 4, y: 18, w: w - 52, h: h - 36 }, [{ xs: pts.map((p) => p[0]), ys: pts.map((p) => p[1]), color: colors[i % 5] }],
      xr, yr, "");
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

// --- board settings -------------------------------------------------------------------------------------------------

async function readSettings(msgEl) {
  say(msgEl, "reading …");
  const r = await api("read_settings");
  if (!r.ok) { say(msgEl, r.error, false); return; }
  const names = { "82": "work mode", "83": "current mA", "84": "microsteps", "88": "stall protection", "89": "interpolation (MPlyer)",
    "8A": "bitrate (2 = 500K)", "8B": "CAN id", "8C": "response mode", "9B": "holding current (0-8 = 10-90 %)", "9E": "limit remap" };
  $("settings-table").querySelector("tbody").replaceChildren(...Object.entries(r.settings).map(([code, v]) => {
    const tr = document.createElement("tr");
    const shown = v === null ? "no reply" : v.value === null ? "not readable" : code === "82" ? `${v.value} ${S.st.work_modes[v.value] ?? ""}` : String(v.value);
    [names[code] ?? `0x${code}`, shown].forEach((c) => tr.append(Object.assign(document.createElement("td"), { textContent: c })));
    const input = $(`set-${code}`);
    if (input && v && v.value !== null) input.value = String(v.value);
    return tr;
  }));
  const missing = Object.values(r.settings).filter((v) => v === null).length;
  say(msgEl, missing === Object.keys(r.settings).length ? "no reply to 0x00 reads (firmware before V1.0.6?)" : "read", missing < Object.keys(r.settings).length);
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

function jogStart(dir, motor = S.activeKey) {
  if (S.jogDir === dir) return;
  jogStop(true);
  S.jogDir = dir;
  S.jogMotor = motor;
  $(dir > 0 ? "jog-plus" : "jog-minus").classList.add("held");
  const send = async () => {
    if (S.jogDir !== dir) return;
    const r = await api("jog", { direction: dir, speed: num("speed-num"), acc: int("acc-num"), motor });
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
  if (release) api("jog_release", { motor: S.jogMotor });
}

// Step and jog never share an input: ← / → (and the Step row's − / +) step by the Step amount, once per press;
// Shift + ← / → and the ◀ ▶ buttons jog while held.

function stepBy(dir, motor = S.activeKey) {
  act("step", { delta: dir * num("step-value"), speed: num("speed-num"), acc: int("acc-num"), motor }, $("motion-msg"),
    `step ${dir > 0 ? "+" : "−"}${$("step-value").value}`);
}

function jogEnd() {   // a released key or button, lost focus, STOP: end the jog
  jogStop(true);
}

function bindJog(id, dir) {
  const el = $(id);
  el.addEventListener("pointerdown", (e) => { if (!el.disabled) { el.setPointerCapture(e.pointerId); jogStart(dir); } });
  el.addEventListener("pointerup", jogEnd);
  el.addEventListener("pointercancel", jogEnd);
}

function nudgeSpeed(d) {
  const st = S.st;
  if (!st) return;
  const inc = st.mode === "bench" ? 10 : st.speed_cap <= 5 ? 0.5 : 1;
  const v = round(clamp(num("speed-num") + d * inc, inc, st.speed_cap), 1);
  setSpeed(v);
  say($("motion-msg"), `speed ${v} ${st.mode === "bench" ? "RPM" : "°/s"}`, true);
}

// Text and number boxes keep their keys; sliders and buttons do not (an arrow there moves the motor).
function typing(e) {
  const t = e.target;
  if (!t || !t.tagName) return false;
  if (t.tagName === "INPUT") return !["range", "checkbox", "button"].includes(t.type);
  return t.tagName === "TEXTAREA" || t.tagName === "SELECT";
}

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
  $("together-stop").onclick = halt;

  // motors: add, find, move together
  $("add-joint").onchange = () => {
    const j = S.st && S.st.joints.find((x) => x.name === $("add-joint").value);
    if (j && j.can_id) $("add-id").value = j.can_id;
  };
  $("add-go").onclick = async () => {
    const r = await api("add_motor", { joint: $("add-joint").value, can_id: int("add-id"), activate: true });
    toast(r.ok ? `${$("add-joint").value} added` : r.error, r.ok ? "info" : "err");
  };
  $("add-find").onclick = async () => {
    $("add-find").disabled = true;
    const r = await api("discover", { first: 1, last: 16 });
    $("add-find").disabled = false;
    toast(r.ok ? r.message : r.error, r.ok ? "info" : "err");
  };
  $("together-open").onclick = openTogether;
  $("together-cancel").onclick = () => $("together").close();
  $("together-pct").oninput = () => ($("together-pct-text").textContent = `${$("together-pct").value} % of each cap`);
  $("together").addEventListener("cancel", (e) => { e.preventDefault(); halt(); });   // Esc is STOP here too
  $("together-go").onclick = async () => {
    const targets = {};
    for (const input of $("together-rows").querySelectorAll("input")) if (input.value !== "") targets[input.dataset.key] = parseFloat(input.value);
    say($("together-msg"), "starting …");
    const r = await api("move_together", { targets, speed_pct: num("together-pct"), together: $("together-sync").checked, acc: int("together-acc") });
    say($("together-msg"), r.ok ? r.message : r.error, r.ok);
    if (r.ok) setTimeout(() => $("together").close(), 600);
  };
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape") { e.preventDefault(); halt(); return; }
    if (typing(e)) return;
    if (e.key === " ") {
      e.preventDefault();
      if (document.activeElement && document.activeElement.blur) document.activeElement.blur();   // no button re-fires on keyup
      halt();
      return;
    }
    if (/^[1-9]$/.test(e.key) && !e.repeat && !document.querySelector("dialog[open]")) {
      const m = S.st && S.st.motors[Number(e.key) - 1];
      if (m) { e.preventDefault(); jogEnd(); selectMotor(m.key); }
      return;
    }
    const dir = { ArrowRight: 1, ArrowLeft: -1 }[e.key];
    if (dir) {
      e.preventDefault();
      if (e.repeat || $(dir > 0 ? "jog-plus" : "jog-minus").disabled) return;   // one press, one step
      if (e.shiftKey) jogStart(dir); else stepBy(dir);
      return;
    }
    if (e.key === "ArrowUp" || e.key === "ArrowDown") { e.preventDefault(); nudgeSpeed(e.key === "ArrowUp" ? 1 : -1); }
  });
  document.addEventListener("keyup", (e) => {
    if (e.key === " " && !typing(e)) e.preventDefault();
    // letting go of the arrow or of Shift ends a keyboard jog
    if ((e.key === "ArrowRight" && S.jogDir > 0) || (e.key === "ArrowLeft" && S.jogDir < 0) || (e.key === "Shift" && S.jogDir)) jogEnd();
  });
  window.addEventListener("blur", jogEnd);
  document.addEventListener("visibilitychange", () => { if (document.hidden) jogEnd(); });
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
  $("step-minus").onclick = () => stepBy(-1);
  $("step-plus").onclick = () => stepBy(1);
  $("limits-toggle").onclick = () => {
    const on = !!(S.st && S.st.limits.on);
    act("limits_on", { on: !on }, $("motion-msg"), on ? "limits off: free movement" : "limits on");
  };
  for (const side of ["min", "max"]) {
    const input = $(`lim-${side}`);
    $(`lim-${side}-here`).onclick = () => act("limit", { side, here: true }, $("motion-msg"), `${side} limit set here`);
    $(`lim-${side}-clear`).onclick = () => act("limit", { side, value: null }, $("motion-msg"), `${side} cleared: open`);
    input.addEventListener("change", () => {
      const value = input.value === "" ? null : parseFloat(input.value);
      act("limit", { side, value }, $("motion-msg"), value === null ? `${side} cleared: open` : `${side} limit set`);
    });
    input.addEventListener("keydown", (e) => { if (e.key === "Enter") input.blur(); });
  }
  $("lim-arm").onclick = () => act("arm_limits", {}, $("motion-msg"), "limits set to the arm's range");
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
    const body = S.st.mode === "joint" ? { joint_speed: num("lim-jspeed") } : { bench_rpm: num("lim-brpm") };
    act("speed_limits", body, $("limits-msg"), "applied").then((r) => { if (r.ok) S.context = null; });
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

  $("settings-read").onclick = () => readSettings($("settings-msg"));
  $("tune-read").onclick = () => readSettings($("tune-msg"));

  // motion profile (left panel and Tuning tab)
  const sendProfile = (values, okText = "profile set") => act("profile", values, $("motion-msg"), okText);
  $$("#profile-kind button").forEach((b) => (b.onclick = () => sendProfile({ kind: b.dataset.kind }, b.textContent)));
  $("ease-range").oninput = () => ($("ease-num").value = $("ease-range").value);
  $("ease-range").onchange = () => sendProfile({ ease_s: num("ease-range") }, `ease ${num("ease-range")} s`);
  $("ease-num").onchange = () => { $("ease-range").value = $("ease-num").value; sendProfile({ ease_s: num("ease-num") }); };
  for (const [id, key, parse] of [["pf-kind", "kind", String], ["pf-curve", "curve", String], ["pf-ease", "ease_s", parseFloat],
    ["pf-rate", "rate_hz", parseFloat], ["pf-follow", "follow_acc", (v) => parseInt(v, 10)]]) {
    $(id).onchange = () => sendProfile({ [key]: parse($(id).value) });
  }
  $("pf-acc").oninput = () => { $("acc-num").value = $("pf-acc").value; $("acc-range").value = $("pf-acc").value; };
  const PRESETS = {
    snappy: { profile: { kind: "board" }, acc: 240 },
    balanced: { profile: { kind: "scurve", curve: "smoothstep", ease_s: 0.5, rate_hz: 50, follow_acc: 240 } },
    smooth: { profile: { kind: "scurve", curve: "quintic", ease_s: 1.2, rate_hz: 50, follow_acc: 220 } },
  };
  for (const b of $$("[data-preset]")) {
    b.onclick = () => {
      const p = PRESETS[b.dataset.preset];
      if (p.acc !== undefined) { $("acc-num").value = p.acc; $("acc-range").value = p.acc; }
      sendProfile(p.profile, `${b.textContent} preset`);
    };
  }

  // graph options
  for (const b of $$("#speed-src button")) {
    b.onclick = () => { S.speedSrc = b.dataset.src; $$("#speed-src button").forEach((x) => x.classList.toggle("on", x === b)); };
  }
  $("g-cmd").onchange = () => (S.showCmd = $("g-cmd").checked);
  for (const b of $$("#poll-ms button")) {
    b.onclick = () => act("poll_period", { ms: Number(b.dataset.ms) }, $("tune-msg"), `sampling every ${b.dataset.ms} ms`);
  }

  // PID (firmware V1.0.8+)
  for (const b of $$("#pid-loop button")) {
    b.onclick = () => { S.pidLoop = b.dataset.loop; $$("#pid-loop button").forEach((x) => x.classList.toggle("on", x === b)); };
  }
  $("pid-defaults").onclick = () => {
    const d = S.st.pid_defaults[S.pidLoop];
    for (const k of ["kp", "ki", "kd", "kv"]) $(`pid-${k}`).value = d[k];
    say($("pid-msg"), `${S.pidLoop} factory defaults filled in: Write sends them`);
  };
  $("pid-read").onclick = async () => {
    const r = await api("read_pid", { loop: S.pidLoop });
    say($("pid-msg"), r.ok ? `${S.pidLoop} raw: Kp Ki ${r.pid.kp_ki ?? "no reply"} · Kd Kv ${r.pid.kd_kv ?? "no reply"}` : r.error, r.ok);
  };
  $("pid-write").onclick = () => {
    const g = {};
    for (const k of ["kp", "ki", "kd", "kv"]) if ($(`pid-${k}`).value !== "") g[k] = parseInt($(`pid-${k}`).value, 10);
    const text = Object.entries(g).map(([k, v]) => `${k} ${v}`).join(", ");
    if (confirm(`Write the ${S.pidLoop} PID gains (${text}) to board ${S.st.can_id}? They are saved on the board.`)) {
      act("pid", { loop: S.pidLoop, ...g }, $("pid-msg"));
    }
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
