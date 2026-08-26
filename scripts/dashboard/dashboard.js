/**
 * PALLETRIS Dashboard — dashboard.js
 *
 * ROS 2 토픽을 rosbridge_server (WebSocket :9090) 를 통해 구독한다.
 *
 * 구독 토픽:
 *   /zone_a/status, /zone_b/status, /zone_c/status  → ZoneStatus (2 Hz)
 *   /robot/event                                      → RobotEvent (이벤트)
 *   /metrics/summary                                  → MetricsSummary (1 Hz)
 *   /algo/heightmap/a, /b, /c                         → sensor_msgs/Image (TRANSIENT_LOCAL)
 */

'use strict';

// ── 설정 ────────────────────────────────────────────────────────
const CONFIG = {
  rosbridgeUrl: 'ws://localhost:9090',
  maxEvents:    20,      // 이벤트 로그 최대 줄 수
  maxDataPts:   60,      // 추이 그래프 최대 데이터 포인트 (초)
  hmWidth:      120,     // heightmap 열 수 (10mm 격자 × 120 = 1200mm)
  hmHeight:     80,      // heightmap 행 수 (10mm 격자 × 80  = 800mm)
  maxHeightMm:  1300,
};

// ── 전역 상태 ────────────────────────────────────────────────────
const state = {
  zones: { a: null, b: null, c: null },
  metrics: null,
  events: [],
  throughputTs: [],    // 각 초의 타임스탬프
  throughputA:  [],
  throughputB:  [],
  throughputC:  [],
  placedA: 0, placedB: 0, placedC: 0,
  startTime: null,
  wallTimer: null,
};

// ── DOM 헬퍼 ────────────────────────────────────────────────────
const $ = (id) => document.getElementById(id);

// ── Chart.js 추이 그래프 ────────────────────────────────────────
let throughputChart = null;

function initThroughputChart() {
  const ctx = $('throughput-chart').getContext('2d');
  throughputChart = new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label: 'A', data: [], borderColor: '#3b82f6', backgroundColor: 'rgba(59,130,246,0.08)', tension: 0.4, pointRadius: 0, borderWidth: 2, fill: true },
        { label: 'B', data: [], borderColor: '#10b981', backgroundColor: 'rgba(16,185,129,0.08)', tension: 0.4, pointRadius: 0, borderWidth: 2, fill: true },
        { label: 'C', data: [], borderColor: '#a855f7', backgroundColor: 'rgba(168,85,247,0.08)', tension: 0.4, pointRadius: 0, borderWidth: 2, fill: true },
      ],
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: { duration: 300 },
      scales: {
        x: { display: false },
        y: {
          display: true,
          min: 0,
          ticks: { color: '#475569', font: { size: 10, family: 'JetBrains Mono' }, maxTicksLimit: 4 },
          grid: { color: 'rgba(100,160,255,0.06)' },
        },
      },
      plugins: {
        legend: {
          display: true,
          labels: { color: '#94a3b8', font: { size: 10 }, boxWidth: 12, padding: 12 },
        },
        tooltip: {
          backgroundColor: 'rgba(8,12,20,0.9)',
          borderColor: 'rgba(100,160,255,0.2)',
          borderWidth: 1,
          titleColor: '#e2e8f0',
          bodyColor: '#94a3b8',
        },
      },
    },
  });
}

function pushThroughputPoint() {
  const now = new Date();
  const label = now.toLocaleTimeString('ko-KR', { hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' });
  state.throughputTs.push(label);
  state.throughputA.push(state.placedA);
  state.throughputB.push(state.placedB);
  state.throughputC.push(state.placedC);

  // 슬라이딩 윈도우
  if (state.throughputTs.length > CONFIG.maxDataPts) {
    state.throughputTs.shift();
    state.throughputA.shift();
    state.throughputB.shift();
    state.throughputC.shift();
  }

  if (throughputChart) {
    throughputChart.data.labels = [...state.throughputTs];
    throughputChart.data.datasets[0].data = [...state.throughputA];
    throughputChart.data.datasets[1].data = [...state.throughputB];
    throughputChart.data.datasets[2].data = [...state.throughputC];
    throughputChart.update('none');
  }
}

// ── 링 차트 (단순 Canvas 원) ────────────────────────────────────
const ringCtxMap = {};

function initRings() {
  ['recognition', 'success', 'idle'].forEach(key => {
    const c = $(`ring-${key}`);
    if (c) ringCtxMap[key] = c.getContext('2d');
  });
}

function drawRing(key, value, color) {
  const ctx = ringCtxMap[key];
  if (!ctx) return;
  const w = 56, r = 22, lw = 6;
  ctx.clearRect(0, 0, w, w);
  // 배경 링
  ctx.beginPath();
  ctx.arc(w/2, w/2, r, 0, Math.PI * 2);
  ctx.strokeStyle = 'rgba(100,160,255,0.08)';
  ctx.lineWidth = lw;
  ctx.stroke();
  // 값 링
  if (value >= 0) {
    const end = -Math.PI / 2 + Math.PI * 2 * Math.min(value, 1.0);
    ctx.beginPath();
    ctx.arc(w/2, w/2, r, -Math.PI / 2, end);
    ctx.strokeStyle = color;
    ctx.lineWidth = lw;
    ctx.lineCap = 'round';
    ctx.stroke();
  }
}

// ── Zone 상태 갱신 ───────────────────────────────────────────────
const ZONE_COLORS = { a: '#3b82f6', b: '#10b981', c: '#a855f7' };

function updateZone(z, msg) {
  state.zones[z] = msg;
  const zl = z.toLowerCase();

  // placed 카운트 추적
  if (zl === 'a') state.placedA = msg.placed_count;
  else if (zl === 'b') state.placedB = msg.placed_count;
  else if (zl === 'c') state.placedC = msg.placed_count;

  // startTime 초기화
  if (!state.startTime && msg.placed_count > 0) {
    state.startTime = Date.now();
  }

  // 상태 뱃지
  const stateBadge = $(`z${zl}-state`);
  stateBadge.textContent = msg.state;
  stateBadge.className = `zone-state-badge ${msg.state}`;

  // accepting 표시
  const acceptEl = $(`z${zl}-accept`);
  if (msg.accepting) {
    acceptEl.textContent = '→';
    acceptEl.classList.remove('blocked');
    acceptEl.title = '수신 가능';
  } else {
    acceptEl.textContent = '✗';
    acceptEl.classList.add('blocked');
    acceptEl.title = '수신 중단';
  }

  // fill bar
  const pct = Math.min(Math.max(msg.fill_rate, 0), 1.0);
  const fillBar = $(`z${zl}-fill-bar`);
  fillBar.style.width = `${(pct * 100).toFixed(1)}%`;
  fillBar.classList.toggle('critical', pct >= 0.9);
  $(`z${zl}-fill-pct`).textContent = `${(pct * 100).toFixed(1)}%`;

  // stats
  $(`z${zl}-placed`).textContent = msg.placed_count;
  $(`z${zl}-rejected`).textContent = msg.rejected_count;
  $(`z${zl}-queue`).textContent = msg.queue_len;
  $(`z${zl}-height`).textContent = `${msg.max_height_mm} mm`;

  // 스파크라인
  updateSparkline(`z${zl}-sparkline`, pct, ZONE_COLORS[zl]);
}

// ── 스파크라인 (간단 Canvas 라인) ───────────────────────────────
const sparkHistory = { za: [], zb: [], zc: [] };

function updateSparkline(canvasId, value, color) {
  const key = canvasId.replace('-sparkline', '');
  const hist = sparkHistory[key] || (sparkHistory[key] = []);
  hist.push(value);
  if (hist.length > 40) hist.shift();

  const c = $(canvasId);
  if (!c) return;
  const ctx = c.getContext('2d');
  const w = c.offsetWidth || 200;
  const h = 40;
  ctx.clearRect(0, 0, w, h);

  if (hist.length < 2) return;

  ctx.beginPath();
  hist.forEach((v, i) => {
    const x = (i / (hist.length - 1)) * w;
    const y = h - v * (h - 4) - 2;
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.stroke();

  // 채우기
  ctx.lineTo(w, h);
  ctx.lineTo(0, h);
  ctx.closePath();
  ctx.fillStyle = color + '18';
  ctx.fill();
}

// ── KPI 갱신 ────────────────────────────────────────────────────
function updateKPI(msg) {
  state.metrics = msg;

  const fmt = (v, pct = true) => {
    if (v < 0) return '--';
    return pct ? `${(v * 100).toFixed(1)}%` : v.toFixed(1);
  };

  const colorize = (el, v, goodThresh = 0.85, warnThresh = 0.70) => {
    if (v < 0) { el.className = 'kpi-value mono'; return; }
    el.classList.remove('good', 'warn', 'bad');
    if (v >= goodThresh) el.classList.add('good');
    else if (v >= warnThresh) el.classList.add('warn');
    else el.classList.add('bad');
  };

  const recEl = $('kv-recognition');
  const sucEl = $('kv-success');
  const idleEl = $('kv-idle');

  recEl.textContent = fmt(msg.recognition_rate);
  colorize(recEl, msg.recognition_rate);
  drawRing('recognition', msg.recognition_rate, msg.recognition_rate >= 0.85 ? '#10b981' : '#f59e0b');

  sucEl.textContent = fmt(msg.place_success_rate);
  colorize(sucEl, msg.place_success_rate);
  drawRing('success', msg.place_success_rate, msg.place_success_rate >= 0.9 ? '#10b981' : '#ef4444');

  // 사이클 타임
  const cycleEl = $('kv-cycle');
  if (msg.cycle_avg_s >= 0) {
    cycleEl.textContent = `${msg.cycle_avg_s.toFixed(1)} s`;
    cycleEl.classList.remove('good', 'warn', 'bad');
    if (msg.cycle_avg_s <= 12) cycleEl.classList.add('good');
    else if (msg.cycle_avg_s <= 18) cycleEl.classList.add('warn');
    else cycleEl.classList.add('bad');
  } else {
    cycleEl.textContent = '-- s';
  }

  // 유휴율 (낮을수록 좋음)
  idleEl.textContent = fmt(msg.robot_idle_rate);
  idleEl.classList.remove('good', 'warn', 'bad');
  if (msg.robot_idle_rate >= 0) {
    if (msg.robot_idle_rate <= 0.25) idleEl.classList.add('good');
    else if (msg.robot_idle_rate <= 0.45) idleEl.classList.add('warn');
    else idleEl.classList.add('bad');
  }
  drawRing('idle', msg.robot_idle_rate >= 0 ? (1 - msg.robot_idle_rate) : -1, '#06b6d4');

  // 카운트
  $('kv-placed').textContent  = msg.placed_count;
  $('kv-scanned').textContent = msg.scanned_count;
  $('kv-failed').textContent  = msg.pick_failed_count;
  $('kv-rejected').textContent= msg.rejected_count;
  $('kv-queue').textContent   = msg.queue_len;

  // elapsed → 상단 헤더 타이머에 반영
  if (msg.elapsed_s > 0 && !state.startTime) {
    state.startTime = Date.now() - msg.elapsed_s * 1000;
  }
}

// ── 이벤트 로그 ─────────────────────────────────────────────────
function addEvent(msg) {
  // RobotEvent 에서 액션 타입 결정
  // msg.action: 0=PICK, 1=PLACE, 2=HOME, 3=STOP
  const ACTION_NAMES = ['PICK', 'PLACE', 'HOME', 'STOP'];
  const actionName = ACTION_NAMES[msg.action] ?? '??';

  let cls = 'other';
  let badge = actionName;
  if (msg.action === 1 && msg.ok)  { cls = 'placed'; badge = '✔ PLACED'; }
  if (msg.action === 1 && !msg.ok) { cls = 'failed'; badge = '✘ FAILED'; }
  if (msg.action === 0 && msg.ok)  { cls = 'picked'; badge = '↑ PICKED'; }

  const now = new Date();
  const timeStr = now.toLocaleTimeString('ko-KR', { hour12: false });

  let detail = '';
  if (msg.action === 1 && msg.ok && msg.has_actual) {
    detail = `pos (${msg.actual_x_mm}, ${msg.actual_y_mm}, ${msg.actual_z_mm}) mm  ${msg.duration_s.toFixed(1)} s`;
  } else if (!msg.ok && msg.reason) {
    detail = `사유: ${msg.reason}  ${msg.duration_s.toFixed(1)} s`;
  }

  const item = document.createElement('div');
  item.className = `event-item ${cls}`;
  item.innerHTML = `
    <div class="event-top">
      <span class="event-badge">${badge}</span>
      <span class="event-boxid">${msg.box_id || '—'}</span>
      <span class="event-time">${timeStr}</span>
    </div>
    ${detail ? `<div class="event-detail">${detail}</div>` : ''}
  `;

  const log = $('event-log');
  // placeholder 제거
  const ph = log.querySelector('.event-placeholder');
  if (ph) ph.remove();

  log.prepend(item);

  // 최대 개수 유지
  while (log.children.length > CONFIG.maxEvents) {
    log.removeChild(log.lastChild);
  }
}

// ── Heightmap 렌더링 ─────────────────────────────────────────────
// sensor_msgs/Image: data는 base64 인코딩, encoding='16UC1'
function renderHeightmap(zone, msg) {
  const canvasId = `hm-${zone}`;
  const canvas = $(canvasId);
  if (!canvas) return;

  // 16UC1 → Uint16Array
  let raw;
  try {
    const binaryStr = atob(msg.data);
    const bytes = new Uint8Array(binaryStr.length);
    for (let i = 0; i < binaryStr.length; i++) bytes[i] = binaryStr.charCodeAt(i);
    raw = new Uint16Array(bytes.buffer);
  } catch (e) {
    console.warn('[heatmap] decode error', e);
    return;
  }

  const w = msg.width  || CONFIG.hmWidth;
  const h = msg.height || CONFIG.hmHeight;

  canvas.width  = w;
  canvas.height = h;

  const ctx = canvas.getContext('2d');
  const imageData = ctx.createImageData(w, h);

  for (let i = 0; i < raw.length; i++) {
    const mm = raw[i];
    const t  = Math.min(mm / CONFIG.maxHeightMm, 1.0);
    const [r, g, b] = heightToColor(t);
    imageData.data[i * 4 + 0] = r;
    imageData.data[i * 4 + 1] = g;
    imageData.data[i * 4 + 2] = b;
    imageData.data[i * 4 + 3] = 255;
  }
  ctx.putImageData(imageData, 0, 0);
}

/**
 * t ∈ [0, 1]  →  [R, G, B]
 * 색상 스케일: 검정(0) → 남색(0.3) → 초록(0.7) → 밝은 연두(1.0)
 */
function heightToColor(t) {
  if (t <= 0) return [10, 14, 26];
  // 구간별 선형 보간
  const stops = [
    [0.00, [10,  14,  26]],
    [0.20, [30,  58, 138]],   // 남색
    [0.50, [5,  150,  105]],  // 초록
    [0.80, [16, 185, 129]],   // 에메랄드
    [1.00, [134, 239, 172]],  // 연두
  ];
  for (let i = 1; i < stops.length; i++) {
    const [t0, c0] = stops[i-1];
    const [t1, c1] = stops[i];
    if (t <= t1) {
      const f = (t - t0) / (t1 - t0);
      return c0.map((v, j) => Math.round(v + f * (c1[j] - v)));
    }
  }
  return stops[stops.length - 1][1];
}

// ── 경과 시간 타이머 ─────────────────────────────────────────────
function startWallTimer() {
  if (state.wallTimer) return;
  state.wallTimer = setInterval(() => {
    if (state.startTime) {
      const sec = Math.floor((Date.now() - state.startTime) / 1000);
      const h = String(Math.floor(sec / 3600)).padStart(2, '0');
      const m = String(Math.floor((sec % 3600) / 60)).padStart(2, '0');
      const s = String(sec % 60).padStart(2, '0');
      $('elapsed-display').textContent = `${h}:${m}:${s}`;
    }
    pushThroughputPoint();
  }, 1000);
}

// ── ROS 연결 ─────────────────────────────────────────────────────
function connect(url) {
  const ros = new ROSLIB.Ros({ url });

  ros.on('connection', () => {
    console.log('[ROS] connected:', url);
    setConnStatus(true);
    subscribeAll(ros);
  });

  ros.on('error', (e) => {
    console.warn('[ROS] error', e);
    setConnStatus(false);
  });

  ros.on('close', () => {
    console.warn('[ROS] closed — 3초 후 재연결');
    setConnStatus(false);
    setTimeout(() => connect(url), 3000);
  });
}

function setConnStatus(connected) {
  const badge = $('conn-status');
  const label = $('conn-label');
  badge.className = `conn-badge ${connected ? 'conn-connected' : 'conn-disconnected'}`;
  label.textContent = connected ? 'LIVE' : 'DISCONNECTED';
}

function subscribeAll(ros) {
  // Zone 상태 × 3
  ['a', 'b', 'c'].forEach(z => {
    const sub = new ROSLIB.Topic({
      ros,
      name: `/zone_${z}/status`,
      messageType: 'packing_msgs/msg/ZoneStatus',
    });
    sub.subscribe(msg => updateZone(z, msg));
  });

  // Robot Event
  const robotSub = new ROSLIB.Topic({
    ros,
    name: '/robot/event',
    messageType: 'packing_msgs/msg/RobotEvent',
  });
  robotSub.subscribe(msg => addEvent(msg));

  // Metrics Summary
  const metricsSub = new ROSLIB.Topic({
    ros,
    name: '/metrics/summary',
    messageType: 'packing_msgs/msg/MetricsSummary',
  });
  metricsSub.subscribe(msg => updateKPI(msg));

  // Heightmap × 3
  ['a', 'b', 'c'].forEach(z => {
    const sub = new ROSLIB.Topic({
      ros,
      name: `/algo/heightmap/${z}`,
      messageType: 'sensor_msgs/msg/Image',
    });
    sub.subscribe(msg => renderHeightmap(z, msg));
  });
}

// ── 초기화 ───────────────────────────────────────────────────────
function init() {
  initThroughputChart();
  initRings();
  startWallTimer();

  // WS URL 표시
  $('ws-url-display').textContent = CONFIG.rosbridgeUrl;

  // 오프라인 데모: ?demo=1 쿼리를 붙이면 더미 데이터로 실행
  const params = new URLSearchParams(window.location.search);
  if (params.get('demo') === '1') {
    runDemoMode();
    return;
  }

  connect(CONFIG.rosbridgeUrl);
}

// ── 데모 모드 (rosbridge 없이 UI 테스트용) ───────────────────────
function runDemoMode() {
  setConnStatus(true);
  $('conn-label').textContent = 'DEMO';
  console.log('[DEMO] 더미 데이터 모드로 실행 중');

  state.startTime = Date.now() - 120_000;  // 2분 전 시작

  const STATES = ['IDLE', 'FETCHING', 'PLANNING', 'PICKING', 'PLACING', 'UPDATING'];
  let tick = 0;

  setInterval(() => {
    tick++;
    ['a', 'b', 'c'].forEach((z, zi) => {
      const placed = Math.min(Math.floor(tick / 3) + zi * 2, 20);
      updateZone(z, {
        zone: z,
        state: STATES[(tick + zi * 2) % STATES.length],
        fill_rate: Math.min(placed / 20 + Math.sin(tick * 0.1 + zi) * 0.02, 1.0),
        placed_count: placed,
        rejected_count: Math.floor(tick / 15),
        queue_len: Math.max(0, 3 - (tick % 4)),
        max_height_mm: Math.min(placed * 60, 1300),
        accepting: placed < 19,
      });
    });

    // 가끔 이벤트 발생
    if (tick % 3 === 0) {
      const ok = Math.random() > 0.1;
      addEvent({
        action: tick % 3 === 0 ? 1 : 0,
        ok,
        box_id: `BOX-${String(100 + tick).padStart(3, '0')}`,
        reason: ok ? '' : 'GRASP_FAILED',
        has_actual: ok,
        actual_x_mm: 10500 + Math.floor(Math.random() * 100),
        actual_y_mm: 1800 + Math.floor(Math.random() * 80),
        actual_z_mm: 150 + (tick * 30),
        duration_s: 10.6 + (Math.random() * 3),
      });
    }

    // KPI
    updateKPI({
      elapsed_s: (Date.now() - state.startTime) / 1000,
      total_boxes: 40 + tick,
      scanned_count: 38 + tick,
      placed_count: Math.min(tick, 20) + 14,
      pick_failed_count: 2,
      rejected_count: 1,
      queue_len: 3,
      recognition_rate: 0.96 - tick * 0.001,
      place_success_rate: 0.943,
      cycle_avg_s: 10.6 + Math.sin(tick * 0.2) * 0.5,
      robot_idle_rate: 0.234,
      fill_rate: 0.67,
      max_height_mm: 870,
      zone: 'a',
    });

    // 더미 heightmap
    ['a', 'b', 'c'].forEach((z, zi) => {
      const w = CONFIG.hmWidth, h = CONFIG.hmHeight;
      const buf = new Uint16Array(w * h);
      for (let r = 0; r < h; r++) {
        for (let c = 0; c < w; c++) {
          const base = (tick * 10 + zi * 200 + r * 15 + c * 5) % 1300;
          buf[r * w + c] = Math.max(0, base + Math.floor(Math.random() * 80) - 40);
        }
      }
      // Uint16Array → base64
      const bytes = new Uint8Array(buf.buffer);
      let binary = '';
      bytes.forEach(b => { binary += String.fromCharCode(b); });
      renderHeightmap(z, { width: w, height: h, data: btoa(binary) });
    });

  }, 800);
}

// 페이지 로드 완료 후 실행
document.addEventListener('DOMContentLoaded', init);
