/**
 * 관제탑 대시보드 — 트럭 1대 관점.
 *
 * 전부 백엔드(server.py) 한 곳에서 받는다:
 *   /api/live      살아 있는 것 — 백엔드가 rclpy 로 /zone_{a,b,c}/status 를 구독한다
 *   /api/pallet …  쌓인 것     — palletris.db (읽기 전용)
 *
 * ★ rosbridge 를 쓰지 않는다. 브라우저가 ROS 를 직접 상대하면 토픽 이름과 QoS 를
 *   JS 문자열로 적게 되고, 기존 dashboard.js 가 그래서 /robot/event(발행자 0 개)를
 *   구독한 채 영영 빈 화면을 띄웠다. 백엔드가 rclpy 로 한 번 정확히 쓰면 끝난다.
 *   게다가 rosbridge 는 어느 package.xml 에도 없는 apt 의존이다.
 *
 * ★ 의존성 0. Chart.js·roslib CDN 을 쓰지 않는다 — 시연장 인터넷이 막히면
 *   그 스크립트 하나 때문에 화면이 통째로 안 뜬다 (기존 dashboard.js 의 실측 문제).
 *
 * ★ 값이 없을 때 0 을 그리지 않는다. `—` 로 둔다.
 *   "0 개" 와 "아직 모른다" 는 다르다 (metrics.py 의 UNKNOWN = -1.0 규약).
 */
'use strict';

const API = '';                                   // 같은 서버가 정적 파일도 준다
const ZONES = ['a', 'b', 'c'];
const POLL_MS = 2000;

const S = {
  zone: 'b',            // 선택된 팔레트
  view: 'iso',          // iso | heat
  truck: null,
  pallet: {},           // zone -> /api/pallet 응답
  status: {},           // zone -> ZoneStatus (rosbridge)
  ros: false,
  lastFetch: 0,         // 마지막으로 백엔드를 읽은 시각
  lastChange: 0,        // DB 값이 마지막으로 **바뀐** 시각
  sig: '',              // 바뀜 판정용 지문
};

const $ = (id) => document.getElementById(id);
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, c =>
  ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

/** -1.0 은 "잴 수 없음" 이다. 0 % 로 뭉개면 없는 문제를 쫓게 된다. */
const pct = (v) => (v === null || v === undefined || v <= -1) ? null : (v * 100);
const fmtPct = (v) => { const p = pct(v); return p === null ? '—' : p.toFixed(1) + '%'; };
const fmtSec = (v) => (v === null || v === undefined || v <= -1) ? '—' : v.toFixed(1) + '초';
const fmtMm  = (v) => (v === null || v === undefined) ? '—' : v.toLocaleString() + 'mm';

// ══════════════════════════════════════════════════════════════
//  데이터
// ══════════════════════════════════════════════════════════════
async function getJSON(path) {
  const r = await fetch(API + path, { cache: 'no-store' });
  const j = await r.json();
  if (!r.ok) throw new Error(j.error || r.statusText);
  return j;
}

async function refresh() {
  try {
    S.truck = await getJSON('/api/truck');
    S.pallet[S.zone] = await getJSON(`/api/pallet/${S.zone}`);
    S.lastFetch = Date.now();
    // ★ "읽은 시각" 과 "바뀐 시각" 을 따로 센다.
    //   DB 는 박스가 실제로 놓일 때만 바뀐다. 2 초마다 읽어도 값이 같으면 화면은
    //   가만히 있는데, 그게 "멈춘 것" 인지 "변화가 없는 것" 인지 구분이 안 됐다.
    const sig = JSON.stringify(S.truck.pallets) + S.truck.metrics.placed_count;
    // ★ 첫 읽기는 "바뀐 것" 이 아니다. 그걸 변화로 세면 관제탑이 꺼져 있는데도
    //   화면이 "진행 중" 이라고 말한다 — 실제로 그랬다.
    if (S.sig === '') S.sig = sig;                       // 기준만 잡고 넘어간다
    else if (sig !== S.sig) { S.sig = sig; S.lastChange = S.lastFetch; }
    $('note').textContent = '';
  } catch (e) {
    $('note').textContent = `백엔드를 못 읽었다 — ${e.message}`;
  }
  render();
}

/** "3초 전" 처럼 사람 말로. */
function ago(ms) {
  if (!ms) return '—';
  const s = Math.round((Date.now() - ms) / 1000);
  if (s < 2) return '방금';
  if (s < 60) return `${s}초 전`;
  const m = Math.floor(s / 60);
  return m < 60 ? `${m}분 전` : `${Math.floor(m / 60)}시간 전`;
}

// ══════════════════════════════════════════════════════════════
//  실시간 — 백엔드가 구독한 결과를 받아온다 (rosbridge 없음)
// ══════════════════════════════════════════════════════════════
async function pollLive() {
  try {
    const l = await getJSON('/api/live');
    // ★ 백엔드가 이미 오래된 값을 버린다 (5초). 여기서 또 안 남긴다 —
    //   "미연결" 이라 적힌 화면에 조금 전 값이 지금 값처럼 남으면 안 된다.
    S.status = l.zones || {};
    setConn(!!l.ok, l.reason);
  } catch {
    S.status = {}; setConn(false, '백엔드를 못 읽었다');
  }
  renderCards();
}

function setConn(live, reason) {
  S.ros = live;
  const el = $('conn');
  el.className = 'conn ' + (live ? 'live' : 'dead');
  $('conn-label').textContent = live
    ? '관제탑 실시간 연결'
    : `실시간 없음 · DB 화면만${reason ? ' — ' + reason : ''}`;
}

// ══════════════════════════════════════════════════════════════
//  카드 3장
// ══════════════════════════════════════════════════════════════
const STATE_CLASS = { IDLE: 'idle', DONE: 'done' };
function renderCards() {
  if (!S.truck) return;
  $('cards').innerHTML = ZONES.map(z => {
    const p = S.truck.pallets[z] || {};
    const st = S.status[z];
    const stateTxt = st ? st.state : '—';
    const cls = st ? (STATE_CLASS[st.state] || 'busy') : 'off';
    // ★ 충전율은 관제탑이 계산한 값만 쓴다. 여기서 다시 계산하면 두 숫자가 갈라진다.
    const fill = st ? pct(st.fill_rate) : null;
    const height = st ? st.max_height_mm : p.max_height_mm;
    return `
      <div class="card ${z === S.zone ? 'on' : ''}" data-z="${z}">
        <div class="card-head">
          <span class="card-name">팔레트 ${z.toUpperCase()}</span>
          <span class="state ${cls}">${esc(stateTxt)}</span>
        </div>
        <div class="card-row">
          <span>적재 <b>${p.count ?? '—'}</b></span>
          <span>충전율 <b>${fill === null ? '—' : fill.toFixed(0) + '%'}</b></span>
          <span>최고 <b>${fmtMm(height)}</b></span>
        </div>
        <div class="bar"><i style="width:${fill === null ? 0 : Math.min(100, fill)}%"></i></div>
        ${p.shipped ? `<div class="ship">트럭에 실려 나간 판 ${p.shipped}장 · 지금 ${p.pallet_index}번째</div>` : ''}
      </div>`;
  }).join('');
  $('cards').querySelectorAll('.card').forEach(c =>
    c.onclick = () => { S.zone = c.dataset.z; refresh(); });
}

// ══════════════════════════════════════════════════════════════
//  적재 상태 — 3D 아이소메트릭 / 높이 지도
// ══════════════════════════════════════════════════════════════
const TYPE_COLOR = { '3호': ['#85B7EB', '#4C9AFF', '#1F5FB0'],
                     '4호': ['#FFC94D', '#FFAB00', '#B87700'],
                     '5호': ['#6FD3AC', '#36B37E', '#1E7A54'] };
const FALLBACK = ['#c9ced6', '#aab2bd', '#7d8794'];

/** yaw 90 이면 가로세로를 맞바꾼다. yaw 는 0/90 뿐이라 회전행렬이 필요 없다. */
function fp(b) {
  let w = b.w, d = b.d;
  if (Math.abs(Math.abs(b.yaw || 0) % 180 - 90) < 45) { const t = w; w = d; d = t; }
  return { w, d };
}

function renderIso(P) {
  const B = P.boxes, PW = P.pallet_w_mm, PD = P.pallet_d_mm;
  if (!B.length) return '<div class="empty">아직 적재된 박스가 없다</div>';

  const top = Math.max(P.max_stack_mm, P.max_height_mm) || 1300;
  // 화면에 맞게 배율을 잡는다. 팔레트 크기가 설정마다 다르므로 매번 계산한다.
  const k = Math.min(560 / (PW + PD), 300 / ((PW + PD) / 2 + top) * 1.15);
  const px = (x, y, z) => [(x - y) * 0.5 * k, (x + y) * 0.25 * k - z * 0.5 * k];

  const corners = [[0,0,0],[PW,0,0],[0,PD,0],[PW,PD,0],[0,0,top],[PW,0,top],[0,PD,top],[PW,PD,top]]
    .map(c => px(...c));
  const xs = corners.map(c => c[0]), ys = corners.map(c => c[1]);
  const ox = -Math.min(...xs) + 14, oy = -Math.min(...ys) + 14;
  const W = Math.max(...xs) - Math.min(...xs) + 28;
  const H = Math.max(...ys) - Math.min(...ys) + 28;
  const pt = (x, y, z) => { const q = px(x, y, z); return `${(q[0]+ox).toFixed(1)},${(q[1]+oy).toFixed(1)}`; };

  let s = `<svg viewBox="0 0 ${W.toFixed(0)} ${H.toFixed(0)}" role="img" width="100%" height="300">`
        + `<title>팔레트 ${P.zone.toUpperCase()} 에 쌓인 박스 ${B.length}개</title>`
        + `<polygon points="${pt(0,0,0)} ${pt(PW,0,0)} ${pt(PW,PD,0)} ${pt(0,PD,0)}"`
        + ` fill="var(--pallet-top)" stroke="var(--pallet)" stroke-width="1.2"/>`;

  // 먼 것부터 그린다 (화가 알고리즘). 축정렬 직육면체라 x+y+z 순서면 맞는다.
  B.slice().sort((a, b) => (a.x+a.y+a.z) - (b.x+b.y+b.z)).forEach(b => {
    const { w, d } = fp(b);
    const c = TYPE_COLOR[b.type] || FALLBACK;
    const x0 = b.x - w/2, x1 = b.x + w/2, y0 = b.y - d/2, y1 = b.y + d/2;
    const z0 = b.z, z1 = b.z + b.h;
    const face = (p, fill) => `<polygon points="${p}" fill="${fill}" stroke="#2b2f36"`
                            + ` stroke-width="0.5" stroke-opacity="0.45"><title>`
                            + `${esc(b.box_id)} · ${esc(b.type)} · ${esc(b.address || '주소 없음')}`
                            + `</title></polygon>`;
    s += face(`${pt(x0,y0,z1)} ${pt(x1,y0,z1)} ${pt(x1,y1,z1)} ${pt(x0,y1,z1)}`, c[0]);
    s += face(`${pt(x1,y0,z1)} ${pt(x1,y1,z1)} ${pt(x1,y1,z0)} ${pt(x1,y0,z0)}`, c[1]);
    s += face(`${pt(x0,y1,z1)} ${pt(x1,y1,z1)} ${pt(x1,y1,z0)} ${pt(x0,y1,z0)}`, c[2]);
  });
  return s + '</svg>';
}

function renderHeat(P) {
  const B = P.boxes, PW = P.pallet_w_mm, PD = P.pallet_d_mm;
  if (!B.length) return '<div class="empty">아직 적재된 박스가 없다</div>';

  const CELL = P.cell_mm * 5;                 // 화면용으로 5칸씩 묶는다 (10mm -> 50mm)
  const NX = Math.ceil(PW / CELL), NY = Math.ceil(PD / CELL);
  const g = Array.from({ length: NX }, () => new Array(NY).fill(0));
  B.forEach(b => {
    const { w, d } = fp(b);
    for (let i = Math.max(0, Math.floor((b.x - w/2) / CELL)); i < Math.min(NX, Math.ceil((b.x + w/2) / CELL)); i++)
      for (let j = Math.max(0, Math.floor((b.y - d/2) / CELL)); j < Math.min(NY, Math.ceil((b.y + d/2) / CELL)); j++)
        g[i][j] = Math.max(g[i][j], b.z + b.h);
  });

  // 단일 색상 명도 램프 = 크기(높이)를 나타내는 표준 표현
  const RAMP = ['#e6f1fb', '#b5d4f4', '#85b7eb', '#4C9AFF', '#2b7bd0', '#185fa5', '#0c447c'];
  const cap = P.max_stack_mm || 1300;          // ★ 설정값. 코드에 안 박는다
  const scale = Math.min(560 / PW, 280 / PD);
  const W = PW * scale, H = PD * scale;

  let s = `<svg viewBox="0 0 ${(W+28).toFixed(0)} ${(H+28).toFixed(0)}" role="img" width="100%" height="300">`
        + `<title>팔레트 ${P.zone.toUpperCase()} 를 위에서 본 높이 지도</title>`
        + `<rect x="14" y="14" width="${W.toFixed(1)}" height="${H.toFixed(1)}"`
        + ` fill="var(--pallet-top)" stroke="var(--pallet)" stroke-width="1.2"/>`;
  for (let i = 0; i < NX; i++) for (let j = 0; j < NY; j++) {
    if (!g[i][j]) continue;
    const idx = Math.min(RAMP.length - 1, Math.floor(g[i][j] / cap * RAMP.length));
    s += `<rect x="${(14 + i*CELL*scale).toFixed(1)}" y="${(14 + j*CELL*scale).toFixed(1)}"`
       + ` width="${(CELL*scale + 0.4).toFixed(1)}" height="${(CELL*scale + 0.4).toFixed(1)}"`
       + ` fill="${RAMP[idx]}"><title>${g[i][j]}mm</title></rect>`;
  }
  return s + '</svg>';
}

function renderStack() {
  const P = S.pallet[S.zone];
  $('sel-zone').textContent = S.zone.toUpperCase();
  if (!P) { $('stage').innerHTML = '<div class="empty">불러오는 중…</div>'; return; }

  $('stage').innerHTML = S.view === 'iso' ? renderIso(P) : renderHeat(P);
  $('stack-sub').textContent =
    `박스 ${P.boxes.length}개 · 최고 ${P.max_height_mm}mm / ${P.max_stack_mm}mm`
    + ` · 팔레트 ${P.pallet_w_mm}×${P.pallet_d_mm}mm`
    + (P.pallet_index > 1 ? ` · ${P.pallet_index}번째 판` : '');

  $('legend').innerHTML = S.view === 'iso'
    ? Object.entries(TYPE_COLOR).map(([n, c]) =>
        `<span><i style="background:${c[1]}"></i>${n}</span>`).join('')
    : `<span>연할수록 낮고 진할수록 높다 · 0 ~ ${P.max_stack_mm}mm</span>`;

  // ★ 이상한 것을 숨기지 않고 알린다
  const w = $('stack-warn');
  const msgs = [];
  if (P.out_of_pallet > 0) {
    // 가장 흔한 원인이 설정 누락이라 그 얘기를 먼저 한다
    msgs.push(`박스 ${P.out_of_pallet}개가 팔레트(${P.pallet_w_mm}×${P.pallet_d_mm}mm) 밖으로 나갔다.`
      + (P.overlay ? '' : ' 서버를 PL_CONFIG_OVERLAY 없이 띄운 것 같다 —'
        + ' 시연 씬(test_3)은 팔레트가 90° 돌아 있어 800×1200 이다.'));
  }
  if (P.overlaps > 0) {
    msgs.push(`박스 ${P.overlaps}쌍이 서로 겹쳐 있다 — 한 팔레트에서는 불가능한 배치다.`
      + ` DB 에 판 번호가 없어 판을 못 가른 것일 수 있다.`);
  }
  w.hidden = msgs.length === 0;
  w.textContent = msgs.join(' ');
}

// ══════════════════════════════════════════════════════════════
//  꺼낼 순서
// ══════════════════════════════════════════════════════════════
async function renderUnload() {
  const el = $('unload');
  try {
    const rows = await getJSON(`/api/pallet/${S.zone}/unload`);
    el.innerHTML = rows.length ? rows.map(r => `
      <div class="u-row">
        <span class="u-seq">${r.seq}</span>
        <span class="u-layer">겹${r.layer}</span>
        <span class="u-addr" title="${esc(r.address || '')}">${esc(r.address || '(주소 없음)')}</span>
        <span class="u-top">${r.top_mm}mm</span>
      </div>`).join('') : '<div class="empty">적재된 박스가 없다</div>';
  } catch (e) { el.innerHTML = `<div class="empty">${esc(e.message)}</div>`; }
}

// ══════════════════════════════════════════════════════════════
//  사건 기록
//
//  ★ 왜 ROS 토픽이 아니라 백엔드(DB)에서 받나 — rosbag 실측 근거:
//    run3 에서 `/zone_*/robot/event` 2,634 건 중 **실제 사건은 27 건**(1.0 %)이었다.
//    같은 (seq, action) 이 수십 번 다시 오고, 나머지는 빈 하트비트다.
//    `/zone_*/box_at_pickup` 은 더 심해서 2,634 -> 45 건(1.7 %) 이다.
//    게다가 토픽은 지난 것을 다시 주지 않아 화면을 늦게 열면 그 전이 통째로 빈다.
//    -> 실시간 감각은 `/zone_b/status`(2 Hz)가 맡고, **사건 목록은 DB** 가 낸다.
// ══════════════════════════════════════════════════════════════
const BADGE = {
  PLACED: 'b-placed', PICKED: 'b-picked',
  PICK_FAILED: 'b-bad', REJECTED: 'b-bad',
  SCANNED: 'b-scan', VERIFIED: 'b-scan', DIVERTED: 'b-scan', SPAWNED: 'b-scan',
};

async function renderEvents() {
  const el = $('events');
  if (el.hidden) return;
  try {
    const rows = await getJSON('/api/events?limit=200');
    if (!rows.length) { el.innerHTML = '<div class="empty">아직 사건이 없다</div>'; return; }
    el.innerHTML = rows.map(e => `
      <div class="e-row" data-box="${esc(e.box_id)}">
        <span class="e-t">${e.t_rel === null ? '—' : e.t_rel.toFixed(0) + 's'}</span>
        <span class="e-z">${esc((e.zone || '').toUpperCase())}</span>
        <span class="e-id">${esc(e.box_id)}</span>
        <span class="e-what">
          <span class="e-badge ${BADGE[e.status] || 'b-scan'}">${esc(e.status)}</span>
          ${e.reason ? `<span class="e-why">${esc(e.reason)}</span> ` : ''}
          <span class="e-addr">${esc((e.address || '').replace('서울 구로구 ', ''))}</span>
        </span>
      </div>`).join('');
    el.querySelectorAll('.e-row').forEach(r =>
      r.onclick = () => showBox(r.dataset.box));
    $('right-sub').textContent = `이번 실행 ${rows.length}건 · 최신순 · 줄을 누르면 그 박스의 이력`;
  } catch (e) { el.innerHTML = `<div class="empty">${esc(e.message)}</div>`; }
}

// ── 박스 이력 (사건을 누르면) ──────────────────────────────────
async function showBox(id) {
  const m = $('modal'), body = $('mb-body');
  $('mb-title').textContent = id;
  body.innerHTML = '<div class="empty">불러오는 중…</div>';
  m.hidden = false;
  try {
    const b = await getJSON(`/api/box/${encodeURIComponent(id)}`);
    const at = b.master || {};
    const pos = (b.place_x_mm === null || b.place_x_mm === undefined)
      ? '아직 안 놓임'
      : `(${b.place_x_mm}, ${b.place_y_mm}, ${b.place_z_mm}) yaw ${(b.place_yaw_deg ?? 0).toFixed(1)}°`;
    const t0 = b.events.length ? b.events[0].ts : null;
    body.innerHTML = `
      <dl class="kv">
        <dt>구역</dt><dd>${esc((b.zone || '—').toUpperCase())}</dd>
        <dt>상태</dt><dd>${esc(b.status || '—')}</dd>
        <dt>크기</dt><dd>${b.size_w_mm}×${b.size_d_mm}×${b.size_h_mm}mm · ${b.mass_g}g</dd>
        <dt>놓인 자리</dt><dd>${esc(pos)}</dd>
        <dt>배송지</dt><dd>${esc(at.address || '—')}</dd>
      </dl>
      <div class="tl">${b.events.map(e => {
        const cls = e.status === 'PLACED' ? 'ok'
                  : (e.status === 'PICK_FAILED' || e.status === 'REJECTED') ? 'bad' : '';
        const dt = (t0 !== null && e.ts !== null) ? ` <span class="tl-t">+${(e.ts - t0).toFixed(1)}s</span>` : '';
        return `<div class="tl-row ${cls}">${esc(e.status)}`
             + `${e.reason ? ` · <span class="e-why">${esc(e.reason)}</span>` : ''}${dt}</div>`;
      }).join('')}</div>`;
  } catch (e) { body.innerHTML = `<div class="empty">${esc(e.message)}</div>`; }
}

// ══════════════════════════════════════════════════════════════
//  지표
// ══════════════════════════════════════════════════════════════
function renderKPI() {
  const m = S.truck?.metrics;
  if (!m) return;
  const grade = (v, good, bad) => {
    const p = pct(v); if (p === null) return 'na';
    return p >= good ? 'ok' : (p <= bad ? 'bad' : 'warn');
  };
  const items = [
    ['인식률',       fmtPct(m.recognition_rate),   grade(m.recognition_rate, 95, 70)],
    ['적재 완료',    `${S.truck.placed_total}개`,  ''],
    ['적재 성공률',  fmtPct(m.place_success_rate), grade(m.place_success_rate, 95, 80)],
    ['평균 사이클',  fmtSec(m.cycle_avg_s),        ''],
    ['로봇 유휴율',  fmtPct(m.robot_idle_rate),    ''],
  ];
  $('kpis').innerHTML = items.map(([l, v, c]) =>
    `<div class="kpi"><div class="kpi-l">${l}</div>
     <div class="kpi-v ${v === '—' ? 'na' : c}">${v}</div></div>`).join('');

  renderFresh();
  // ★ 정상일 때는 짧게. "실행 구간 10~1075초" 같은 진단값은 발표 보는 사람에게
  //   아무 의미가 없다 — 이상할 때만 말하게 한다 (아래 renderFresh 도 같은 원칙).
  const r = S.truck.run;
  const min = r ? r.elapsed_s / 60 : 0;
  const ship = S.truck.shipped_total
    ? ` · 트럭에 실린 판 ${S.truck.shipped_total}장` : '';
  // ★ "진행" 이라고만 쓰면 **지금 돌고 있다**는 뜻으로 읽힌다. 실제로는 DB 에 담긴
  //   기록의 길이일 뿐이고, 관제탑이 꺼져 있으면 그 숫자는 멈춰 있다.
  //   값이 최근에 실제로 바뀐 적이 있어야 "진행 중" 이라고 말한다.
  const moving = S.lastChange && (Date.now() - S.lastChange < 30000);
  const dur = min < 1 ? Math.round(r ? r.elapsed_s : 0) + '초' : min.toFixed(1) + '분';
  $('run').textContent = !r || r.elapsed_s <= 0
    ? '실행 기록 없음'
    : (moving ? `진행 중 · ${dur}째${ship}`
              : `멈춤 · ${dur} 분량의 기록${ship}`);
}

/** ★ 화면이 살아 있는지 / 값이 안 바뀌는 것뿐인지 구분해 준다. */
function renderFresh() {
  const el = $('fresh');
  if (!el) return;
  if (!S.lastFetch) { el.textContent = ''; return; }
  // ★ 정상이면 아무 말도 안 한다. 화면은 조용한 게 기본이고,
  //   **틀렸을 때만** 말해야 한다. 정상일 때까지 떠들면 진짜 경고가 묻힌다.
  const stale = Date.now() - S.lastFetch > 6000;   // 폴링 2초인데 6초 넘게 못 읽었다
  const run = S.truck && S.truck.run;
  const odd = run && run.elapsed_s > 86400;        // 하루 넘는 실행 = 시계 섞임
  el.className = 'fresh' + (stale || odd ? ' stale' : '');
  el.textContent = stale ? `⚠ 백엔드를 ${ago(S.lastFetch)} 부터 못 읽고 있다`
                 : odd   ? `⚠ 실행 구간이 ${(run.elapsed_s/86400).toFixed(0)}일이다 — `
                           + `DB 에 여러 실행이 섞여 있다. gen_master_db --reset 후 다시 볼 것`
                 : '';
}

function render() { renderCards(); renderStack(); renderUnload(); renderEvents(); renderKPI(); }

// ══════════════════════════════════════════════════════════════
function init() {
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => {
    document.querySelectorAll('.tab').forEach(x => x.classList.remove('on'));
    t.classList.add('on'); S.view = t.dataset.view; renderStack();
  });
  // 우측 패널 전환 — 꺼낼 순서 / 사건 기록
  document.querySelectorAll('.tab2').forEach(t => t.onclick = () => {
    document.querySelectorAll('.tab2').forEach(x => x.classList.remove('on'));
    t.classList.add('on');
    const ev = t.dataset.right === 'events';
    $('events').hidden = !ev; $('unload').hidden = ev;
    $('right-title').textContent = ev ? '사건 기록' : '꺼낼 순서 · 배송지';
    $('right-sub').textContent = ev ? '불러오는 중…' : '위에 아무것도 없는 것부터 한 겹씩';
    ev ? renderEvents() : renderUnload();
  });
  // 모달 닫기 — X · 바깥 · Esc
  $('mb-close').onclick = () => { $('modal').hidden = true; };
  $('modal').onclick = (e) => { if (e.target === $('modal')) $('modal').hidden = true; };
  document.addEventListener('keydown', e => { if (e.key === 'Escape') $('modal').hidden = true; });
  $('note').textContent = '';
  refresh();  pollLive();
  setInterval(refresh, POLL_MS);
  setInterval(pollLive, 1000);
  setInterval(renderFresh, 1000);     // 시간 표시는 값이 안 바뀌어도 흐른다        // 관제탑이 2 Hz 라 1 초면 충분하다
}
document.addEventListener('DOMContentLoaded', init);
