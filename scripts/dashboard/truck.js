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
const POLL_MS = 1000;   // 한 요청이면 되니 더 자주 물어도 된다 (관제탑이 2 Hz)
// ★ 백엔드의 --cam 기본값과 짝이다. 다른 카메라를 보려면 서버에 --cam /cam2 를 주고
//   여기를 'cam2' 로 바꾼다. 토픽 이름(/cam2/rgb)은 백엔드만 안다.
const CAM_NAME = 'cam1';

const S = {
  zone: 'b',            // 선택된 팔레트
  view: 'iso',          // iso | heat
  truck: null,
  pallet: {},           // zone -> /api/pallet 응답
  status: {},           // zone -> ZoneStatus
  live: null,           // /api/live 응답 전체 (cams 상태를 여기서 읽는다)
  ros: false,
  lastFetch: 0,         // 마지막으로 백엔드를 읽은 시각
  lastChange: 0,        // DB 값이 마지막으로 **바뀐** 시각
  sig: '',              // 바뀜 판정용 지문
  unload: [],           // 꺼낼 순서 (한 요청에 같이 온다)
  events: [],           // 사건 기록 (같음)
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
    // ★ 한 요청에 다 받는다. 전에는 truck·pallet·live·events·unload 를 따로 불러
    //   왕복이 다섯이었고, 각 응답이 서로 다른 순간의 DB 를 봐서 카드의 적재 수와
    //   3D 의 박스 수가 어긋나는 순간이 있었다.
    const st = await getJSON(`/api/state?zone=${S.zone}`);
    S.truck = st.truck;
    S.pallet[S.zone] = st.pallet;
    S.unload = st.unload;
    S.events = st.events;
    S.status = (st.live && st.live.zones) || {};
    S.live = st.live || null;              // cams(영상 상태) 를 여기서 읽는다
    setConn(!!(st.live && st.live.ok), st.live && st.live.reason);
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
    // ★ 여기서 연결 배지를 꺼야 한다 (2026-08-27). 예전엔 1초마다 도는 pollLive 가
    //   껐는데, API 를 /api/state 하나로 합치면서 그 함수가 없어졌다. 그 뒤로
    //   백엔드가 죽어도 배지가 **초록으로 남아** 화면이 거짓말을 했다.
    setConn(false, '백엔드를 못 읽었다');
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
          <span class="state ${cls}" data-tip="관제탑 상태머신. IDLE → FETCHING → PLANNING → PICKING → PLACING → UPDATING 을 돈다. '—' 는 실시간이 안 붙어 상태를 모른다는 뜻이다.">${esc(stateTxt)}</span>
        </div>
        <div class="card-row">
          <span data-tip="지금 이 팔레트에 올라가 있는 박스 수. 트럭으로 나간 판은 세지 않는다.">적재 <b>${p.count ?? '—'}</b></span>
          <span data-tip="관제탑이 계산해 보낸 값을 그대로 쓴다. 분모는 한도(1300mm)가 아니라 실제 더미 높이다 — 화면이 다시 계산하면 관제탑 숫자와 갈라진다.">충전율 <b>${fill === null ? '—' : fill.toFixed(0) + '%'}</b></span>
          <span data-tip="더미 꼭대기 높이 (팔레트 상면 기준). 한도는 1300mm 다.">최고 <b>${fmtMm(height)}</b></span>
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
  // ★ 가로축 부호 (2026-08-27 수정). 그리는 면(+X 면·+Y 면) · 그리는 순서(먼 것
  //   먼저) · 세로축은 셋 다 "카메라가 +X,+Y 위쪽에 있다" 는 그림인데 **가로축만
  //   반대**였다. 그래서 어떤 시점에서도 성립하지 않는, 좌우가 뒤집힌 그림이 나왔다.
  //   축정렬 더미라 그럴듯해 보여서 그림만 봐서는 안 드러난다 — Isaac 화면과
  //   나란히 놓아야 보인다 (CLAUDE.md §10-3).
  const px = (x, y, z) => [(y - x) * 0.5 * k, (x + y) * 0.25 * k - z * 0.5 * k];

  // 로봇이 선 자리 (팔레트 로컬 mm). 설정에서 온다 — 로봇은 팔레트 밖에 서므로
  // 여백 계산에 같이 넣어야 표시가 안 잘린다.
  const R = P.robot_local_mm;
  const corners = [[0,0,0],[PW,0,0],[0,PD,0],[PW,PD,0],[0,0,top],[PW,0,top],[0,PD,top],[PW,PD,top]]
    .concat(R ? [[R[0], R[1], 0]] : [])
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

  // ★ 방향 표시. 좌우가 뒤집혔는지는 그림만 봐서는 알 수 없다 — 로봇이 어느 변에
  //   서 있는지를 같이 그려야 Isaac 화면과 곧바로 대볼 수 있다.
  if (R) {
    const q = px(R[0], R[1], 0);
    const cx = q[0] + ox, cy = q[1] + oy;
    s += `<circle cx="${cx.toFixed(1)}" cy="${cy.toFixed(1)}" r="4.5"`
       + ` fill="var(--ink3)"><title>로봇 베이스 (팔레트 기준 ${R[0]}, ${R[1]}mm)</title></circle>`
       + `<text x="${cx.toFixed(1)}" y="${(cy - 8).toFixed(1)}" text-anchor="middle"`
       + ` font-size="11" fill="var(--ink3)">로봇</text>`;
  }
  return s + '</svg>';
}

function renderHeat(P) {
  const PW = P.pallet_w_mm, PD = P.pallet_d_mm;
  const cap = P.max_stack_mm || 1300;          // ★ 설정값. 코드에 안 박는다
  const RAMP = ['#e6f1fb', '#b5d4f4', '#85b7eb', '#4C9AFF', '#2b7bd0', '#185fa5', '#0c447c'];
  // ★ 3D 그림과 **같은 방향에서** 본다 (2026-08-27). 3D 는 월드 +Y 가 화면 오른쪽,
  //   +X 가 화면 안쪽(아래)인 시점이다. 지도도 그 방향으로 눕힌다 — 안 그러면
  //   두 탭에서 로봇이 서로 반대편에 있어서, 고쳐 놓고도 또 헷갈린다.
  //   즉 화면 가로 = 월드 +Y(PD) · 화면 세로 = 월드 +X(PW).
  const scale = Math.min(560 / PD, 280 / PW);
  const W = PD * scale, H = PW * scale;

  // ★ 두 출처가 있다.
  //   ① algo 가 직접 낸 격자 (`/algo/heightmap/<z>`) — **이게 진짜다**
  //   ② 여기서 DB 좌표로 다시 계산한 것 — algo 발행이 꺼져 있을 때
  //   ①이 있으면 그걸 그리고, 화면에 어느 쪽인지 적는다.
  const A = P.algo_heightmap;
  let NX, NY, cellW, cellH, at;
  if (A && A.grid && A.w && A.d) {
    // algo 격자: height=W(월드 +X), width=D(월드 +Y) — 발행 쪽 규약 그대로
    NX = A.w; NY = A.d;
    cellW = PW / NX; cellH = PD / NY;
    at = (i, j) => A.grid[i * NY + j];
  } else {
    const CELL = (P.cell_mm || 10) * 5;        // 화면용으로 5칸씩 묶는다
    NX = Math.ceil(PW / CELL); NY = Math.ceil(PD / CELL);
    const g = Array.from({ length: NX }, () => new Array(NY).fill(0));
    (P.boxes || []).forEach(b => {
      const { w, d } = fp(b);
      for (let i = Math.max(0, Math.floor((b.x - w/2) / CELL)); i < Math.min(NX, Math.ceil((b.x + w/2) / CELL)); i++)
        for (let j = Math.max(0, Math.floor((b.y - d/2) / CELL)); j < Math.min(NY, Math.ceil((b.y + d/2) / CELL)); j++)
          g[i][j] = Math.max(g[i][j], b.z + b.h);
    });
    cellW = CELL; cellH = CELL;
    at = (i, j) => g[i][j];
  }
  if (!A && !(P.boxes || []).length) return '<div class="empty">아직 적재된 박스가 없다</div>';

  // ★ 세로축을 뒤집는다 (2026-08-27 수정). SVG 는 아래가 +y 라 월드 +Y 를 그대로
  //   내려 그리면 **좌우가 뒤집힌 지도**가 된다 (위에서 내려다본 그림에서 +X 가
  //   오른쪽이면 +Y 는 위쪽이어야 한다). 3D 그림과 같은 종류의 실수였다.
  const R = P.robot_local_mm;
  const rx = R ? R[1] * scale : 0, ry = R ? R[0] * scale : 0;
  // 로봇은 팔레트 밖에 서므로 그림틀을 그만큼 넓힌다 (viewBox 는 음수 원점을 받는다)
  const x0 = Math.min(0, rx) - 14, x1 = Math.max(W, rx) + 14;
  const y0 = Math.min(0, ry) - 14, y1 = Math.max(H, ry) + 14;

  let s = `<svg viewBox="${x0.toFixed(1)} ${y0.toFixed(1)} ${(x1-x0).toFixed(1)} ${(y1-y0).toFixed(1)}"`
        + ` role="img" width="100%" height="300">`
        + `<title>팔레트 ${P.zone.toUpperCase()} 를 위에서 본 높이 지도</title>`
        + `<rect x="0" y="0" width="${W.toFixed(1)}" height="${H.toFixed(1)}"`
        + ` fill="var(--pallet-top)" stroke="var(--pallet)" stroke-width="1.2"/>`;
  for (let i = 0; i < NX; i++) for (let j = 0; j < NY; j++) {
    const v = at(i, j);
    if (!v) continue;
    const idx = Math.min(RAMP.length - 1, Math.floor(v / cap * RAMP.length));
    s += `<rect x="${(j*cellH*scale).toFixed(1)}" y="${(i*cellW*scale).toFixed(1)}"`
       + ` width="${(cellH*scale + 0.4).toFixed(1)}" height="${(cellW*scale + 0.4).toFixed(1)}"`
       + ` fill="${RAMP[idx]}"><title>${v}mm</title></rect>`;
  }
  if (R) {
    s += `<circle cx="${rx.toFixed(1)}" cy="${ry.toFixed(1)}" r="4.5" fill="var(--ink3)">`
       + `<title>로봇 베이스 (팔레트 기준 ${R[0]}, ${R[1]}mm)</title></circle>`
       + `<text x="${rx.toFixed(1)}" y="${(ry - 8).toFixed(1)}" text-anchor="middle"`
       + ` font-size="11" fill="var(--ink3)">로봇</text>`;
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
    : `<span>연할수록 낮고 진할수록 높다 · 0 ~ ${P.max_stack_mm}mm</span>`
      + (P.algo_heightmap
          ? ` <span style="color:var(--ok)">· 알고리즘이 낸 격자 ${P.algo_heightmap.w}×${P.algo_heightmap.d}</span>`
          : ` <span style="color:var(--ink3)">· DB 좌표로 재계산 (algo 발행 꺼짐)</span>`);

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
/** 꺼낼 순서 목록.
 *
 * ★ 2026-08-27 부터 오른쪽 패널의 그 탭은 **cam1 실시간 영상**으로 바뀌었다.
 *   데이터(`/api/state` 의 `unload`)와 이 함수는 그대로 두었다 — 되돌리려면
 *   truck.html 의 `<div class="cam" id="cam">` 자리에 `<div class="unload"
 *   id="unload">` 를 두고 탭 하나를 되살리면 된다. 서버는 손댈 것이 없다.
 */
function renderUnload() {
  const el = $('unload');
  if (!el) return;                 // 지금 화면에는 그 자리가 없다
  const rows = S.unload || [];
  el.innerHTML = rows.length ? rows.map(r => `
      <div class="u-row">
        <span class="u-seq" title="꺼내는 순서. 1번부터 차례로 빼면 위에 얹힌 박스를 건드리지 않는다.">${r.seq}</span>
        <span class="u-layer" title="몇 번째 겹에서 나오는가. 같은 겹끼리는 서로 위아래로 안 얹혀 있어 순서를 바꿔도 된다.">겹${r.layer}</span>
        <span class="u-addr" title="${esc(r.address || '주소 없음')}">${esc(r.address || '(주소 없음)')}</span>
        <span class="u-top" title="그 박스 윗면 높이 (팔레트 상면 기준).">${r.top_mm}mm</span>
      </div>`).join('') : '<div class="empty">적재된 박스가 없다</div>';
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

function renderEvents() {
  const el = $('events');
  if (el.hidden) return;
  const rows = S.events || [];
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
//  cam1 실시간 영상
//
//  ★ 브라우저가 ROS 를 직접 상대하지 않는다. 백엔드가 `/cam1/rgb` 를 구독해
//    최신 한 장만 들고 있다가 PNG 로 내주고, 여기서는 그냥 <img> 를 갈아 끼운다.
//    rosbridge 도, 브라우저용 이미지 디코더도 필요 없다.
//  ★ 왜 MJPEG 스트림이 아닌가: 한 연결을 계속 붙들면 파이썬 표준 HTTP 서버의
//    스레드 하나가 영영 묶인다. 탭을 두 개만 열어도 다른 요청이 밀린다.
//    한 장씩 받으면 연결이 매번 끝나고, 화면이 멈춰도 서버는 멀쩡하다.
// ══════════════════════════════════════════════════════════════
const CAM_MS = 250;                    // 4 Hz. 사람 눈에는 충분하고 회선은 여유롭다
let camTimer = null, camBusy = false, camStaleHits = 0;

function startCam() {
  if (camTimer) return;
  camTick();
  camTimer = setInterval(camTick, CAM_MS);
}
function stopCam() {
  if (camTimer) { clearInterval(camTimer); camTimer = null; }
  showEl($('cam-ov'), false);
}

function camTick() {
  // ★ 앞 장이 아직 안 왔으면 새로 요청하지 않는다. 느린 연결에서 요청만 쌓이면
  //   화면은 점점 과거를 보여주면서 회선만 먹는다.
  if (camBusy || document.hidden) return;
  camBusy = true;
  const img = $('cam-img'), off = $('cam-off');
  const next = new Image();
  next.onload = () => {
    camBusy = false;
    img.src = next.src; img.hidden = false; off.hidden = true;
    // ★ 영상이 새로 왔을 때만 QR 을 물어본다. 사진과 표시가 같은 프레임을 보게
    //   하려는 것이다 — 따로 돌리면 박스가 사진보다 한 박자 앞서거나 뒤진다.
    drawQR(next.naturalWidth, next.naturalHeight);
    const c = (S.live && S.live.cams && S.live.cams[CAM_NAME]) || null;
    // ★ 서버는 마지막 프레임을 계속 내준다 — 카메라가 멎어도 그림은 그대로 보인다.
    //   그래서 **몇 초 전 것인지**를 같이 적고, 오래되면 멈췄다고 분명히 말한다.
    //   (30 Hz 로 오는 토픽이라 3초면 확실히 이상한 것이다)
    // ★ 한 번 오래됐다고 바로 "멎었다" 라고 하지 않는다 (두 번 연속일 때만).
    //   탭을 다시 열면 백엔드가 그동안 안 받아 둔 마지막 프레임을 한 번 주는데,
    //   그걸 고장으로 읽으면 멀쩡한 카메라에 경고가 번쩍인다.
    camStaleHits = (c && c.age_s > 3) ? camStaleHits + 1 : 0;
    $('right-sub').textContent = !c ? '박스 등록·검증을 하는 그 화면'
      : camStaleHits >= 2 ? `⚠ 영상이 ${c.age_s.toFixed(0)}초째 안 바뀐다 — 카메라가 멎었다`
      : `${CAM_NAME} · ${c.w}×${c.h} · ${c.age_s.toFixed(1)}초 전 프레임`;
  };
  next.onerror = () => {
    camBusy = false;
    img.hidden = true; off.hidden = false; showEl($('cam-ov'), false);
    // 왜 안 오는지는 백엔드가 안다 (503 본문). 그걸 그대로 띄운다.
    fetch(`/api/cam/${CAM_NAME}.png`, { cache: 'no-store' })
      .then(r => r.json()).then(j => { off.textContent = j.error || '영상 없음'; })
      .catch(() => { off.textContent = '백엔드를 못 읽었다'; });
  };
  next.src = `/api/cam/${CAM_NAME}.png?w=640&t=${Date.now()}`;
}

/** ★ SVG 는 `el.hidden = false` 가 **안 먹는다.** `hidden` 은 HTMLElement 의
 *  프로퍼티라, SVG 요소에 대입하면 그냥 자바스크립트 변수 하나가 생길 뿐
 *  속성은 그대로 남는다 — 내용은 다 그려 놓고 화면에는 아무것도 안 보였다.
 *  속성을 직접 넣고 뺀다. */
function showEl(el, on) {
  if (on) el.removeAttribute('hidden'); else el.setAttribute('hidden', '');
}

/** QR 을 영상 위에 그린다 — 빨간 테두리 + 번호(+배송지).
 *
 *  ★ 좌표는 백엔드가 **0~1 비율**로 준다. 여기서 사진의 원래 크기(naturalWidth)를
 *    곱해 viewBox 좌표로 만든다. 창을 줄이거나 늘려도 SVG 가 사진과 같은 규칙으로
 *    맞춰지므로(둘 다 contain) 다시 계산할 일이 없다.
 */
async function drawQR(natW, natH) {
  const ov = $('cam-ov');
  if (!natW || !natH) { showEl(ov, false); return; }
  let r;
  try { r = await getJSON(`/api/cam/${CAM_NAME}/qr`); }
  catch { showEl(ov, false); return; }
  const qrs = (r && r.qrs) || [];
  if (!qrs.length) { showEl(ov, false); return; }
  // ★ 구역 색은 **씬이 박스를 칠하는 색** 그대로다 (A 빨강·B 초록·C 파랑).
  //   백엔드가 설정에서 유도해 보내 준다 — 화면에 색을 박아 두면 씬과 갈라진다.
  const ZC = (r && r.zone_colors) || {};

  ov.setAttribute('viewBox', `0 0 ${natW} ${natH}`);
  ov.innerHTML = qrs.map(q => {
    const pts = q.poly.map(([x, y]) => `${(x * natW).toFixed(1)},${(y * natH).toFixed(1)}`);
    const xs = q.poly.map(p => p[0] * natW), ys = q.poly.map(p => p[1] * natH);
    const x0 = Math.min(...xs), y0 = Math.min(...ys);
    // 번호 + (DB 에 있으면) 배송지. 못 찾으면 "등록 안 됨" 이라고 분명히 말한다 —
    // 빈칸으로 두면 "읽었는데 DB 에 없다" 와 "안 읽혔다" 가 화면에서 같아 보인다.
    const b = q.box;
    // ★ 배송지가 아니라 **구역**을 띄운다. 화면에서 궁금한 건 "저 박스가 어느
    //   팔레트로 가나" 이고, 아파트 이름은 길어서 영상을 가린다 (배송지는
    //   사건 기록·꺼낼 순서에 이미 있다)
    const z = (b && b.zone || '').toLowerCase();
    const label = (q.text || '읽음 (내용 없음)') + (z ? ` · ${z.toUpperCase()}구역` : ' · 등록 안 됨');
    const col = ZC[z] || '#6b7280';             // 모르는 구역은 회색
    const ty = y0 > 34 ? y0 - 10 : y0 + 30;     // 위가 좁으면 상자 안쪽에 붙인다
    // 폭·위치는 아래에서 **재서** 정한다 (글자 수로 어림하면 한글에서 어긋난다)
    return `<polygon class="qr-box" points="${pts.join(' ')}" stroke="${col}" fill="${col}"/>`
         + `<rect class="qr-tag" y="${(ty - 22).toFixed(1)}" height="28" rx="5" fill="${col}"/>`
         + `<text class="qr-txt" y="${ty.toFixed(1)}" data-x0="${x0.toFixed(1)}">${esc(label)}</text>`;
  }).join('');
  showEl(ov, true);

  // ★ 라벨 폭을 **직접 재서** 자리를 잡는다. 글자 수 × 상수로 어림하면 한글이
  //   섞였을 때 실제 폭의 절반밖에 안 나와, 오른쪽 끝 QR 의 번호가 잘려 나갔다.
  //   getComputedTextLength() 는 그려진 뒤의 진짜 폭이다.
  ov.querySelectorAll('text.qr-txt').forEach((t, i) => {
    const w = t.getComputedTextLength() + 18;
    const x = Math.max(0, Math.min(parseFloat(t.dataset.x0), natW - w));
    t.setAttribute('x', (x + 9).toFixed(1));
    const box = ov.querySelectorAll('rect.qr-tag')[i];
    box.setAttribute('x', x.toFixed(1));
    box.setAttribute('width', w.toFixed(1));
  });
}

// ══════════════════════════════════════════════════════════════
//  지표
// ══════════════════════════════════════════════════════════════
function renderKPI() {
  const m = S.truck?.metrics;
  if (!m) return;
  // ★ 네 번째 칸은 마우스를 올렸을 때 뜨는 설명이다. 지표 이름만으로는 분모가
  //   무엇인지 알 수 없어서 발표 때마다 말로 답해야 했다.
  const items = [
    // ★ 다섯 칸을 **같은 모양**으로 둔다 (2026-08-27). 두 칸만 색을 입히니
    //   나머지와 다른 물건처럼 보였다. 기준값은 말풍선이 그대로 알려 준다.
    ['인식률',       fmtPct(m.recognition_rate),   '',
     `cam1 이 읽은 박스 ${m.scanned_count}개 ÷ 라인에 올라온 박스 ${m.total_boxes}개(읽은 것 + 놓친 것 ${m.scan_miss}개). 95% 위면 초록, 70% 아래면 빨강.`],
    ['적재 완료',    `${S.truck.placed_total}개`,  '',
     '지금 세 팔레트에 올라가 있는 박스 수. 트럭으로 나간 판은 따로 센다.'],
    ['적재 성공률',  fmtPct(m.place_success_rate), '',
     `놓기 성공 ÷ 시도(성공 + 픽 실패 ${m.pick_failed_count}회). 지금까지 시도 ${m.place_attempts}회. 실패한 뒤 다시 집어 성공하면 시도는 2, 성공은 1 로 센다.`],
    ['평균 사이클',  fmtSec(m.cycle_avg_s),        '',
     '박스 하나를 집기 시작해서 내려놓을 때까지 걸린 평균 시간.'],
    ['로봇 유휴율',  fmtPct(m.robot_idle_rate),    '',
     '전체 실행 시간 중 로봇이 집지도 놓지도 않고 있던 시간의 비율. 높으면 로봇이 아니라 라인이 병목이라는 뜻이다.'],
  ];
  // 양 끝 칸은 말풍선을 가운데 정렬하면 화면 밖으로 나간다 — 끝에 붙인다
  $('kpis').innerHTML = items.map(([l, v, c, tip], i) =>
    `<div class="kpi tip-u ${i === 0 ? 'tip-l' : ''} ${i === items.length - 1 ? 'tip-r' : ''}" data-tip="${esc(tip)}">
     <div class="kpi-l">${l}</div>
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
    $('events').hidden = !ev; $('cam').hidden = ev;
    $('right-title').textContent = ev ? '사건 기록' : 'cam1 · 투입부 카메라';
    $('right-sub').textContent = ev ? '불러오는 중…' : '박스 등록·검증을 하는 그 화면';
    // ★ 보고 있지 않을 때는 영상을 안 받는다. 안 그러면 사건 기록을 보는 동안에도
    //   초당 몇 MB 를 계속 끌어와 시연 PC 가 괜히 바빠진다.
    ev ? (stopCam(), renderEvents()) : startCam();
  });
  // 모달 닫기 — X · 바깥 · Esc
  $('mb-close').onclick = () => { $('modal').hidden = true; };
  $('modal').onclick = (e) => { if (e.target === $('modal')) $('modal').hidden = true; };
  document.addEventListener('keydown', e => { if (e.key === 'Escape') $('modal').hidden = true; });
  $('note').textContent = '';
  startCam();                            // 처음 열리는 탭이 cam1 이다
  refresh();  setInterval(refresh, POLL_MS);
  setInterval(renderFresh, 1000);     // 시간 표시는 값이 안 바뀌어도 흐른다        // 관제탑이 2 Hz 라 1 초면 충분하다
}
document.addEventListener('DOMContentLoaded', init);
