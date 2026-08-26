"""결과물 한 벌을 한 번에 뽑는다 — 발표 자료 · Isaac 과 눈으로 대조용.

왜 따로 만드나
    지금까지 `viz` · `compare_baseline` · `robustness` 를 따로 돌렸다. 셋 다 돌려야
    한 세트가 되는데, 하나만 다시 돌리면 **표와 그림이 서로 다른 코드 상태**를
    가리킨다. 발표 직전에 이게 제일 무섭다. 한 명령으로 묶어 둔다.

내놓는 것 (`docs/트랙C_결과/`)
    01_stack.mp4              적재 과정 영상 (3D · heightmap · 적재율 곡선)
    02_stage_*.png            단계별 정지 화면 — Isaac 화면과 나란히 놓고 비교
    03_compare_baseline.png   제안 vs 베이스라인 (§13 "챌린지" 근거)
    04_robustness.png         pose 오차 견딤 (§13 "동적 대응" 근거)
    README.md                 무엇을 보라는지 + 숫자 표

실행
    ros2 run packing_algo report
    ros2 run packing_algo report --seeds 10 --boxes 40    # 빨리 한 번
"""
import argparse
import glob
import os
import statistics

import matplotlib

matplotlib.use("Agg")                       # pyplot 보다 먼저 (창 없는 환경)
import matplotlib.pyplot as plt             # noqa: E402

from packing_algo import compare_baseline as CB   # noqa: E402
from packing_algo.heightmap_core import HeightmapPacker  # noqa: E402
from packing_algo import robustness as RB         # noqa: E402
from packing_algo import viz                      # noqa: E402
from packing_common.config import load            # noqa: E402

# 정지 화면을 뽑을 지점 — 적재 개수 기준. 층이 올라가는 순간을 보여주려는 것이다.
STAGE_AT = (1, 5, 10, 15, 20, None)         # None = 마지막


def _stage_png(cfg, steps, k, path):
    """viz 와 **같은 그림**을 한 장 그린다. 영상과 정지화면이 달라 보이면 안 된다."""
    fills = [s["fill"] for s in steps]
    caps = [s["cap"] for s in steps]
    s = steps[k]
    fig = plt.figure(figsize=(12, 6.5))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.5, 1], hspace=0.45, wspace=0.25,
                          left=0.03, right=0.96, top=0.90, bottom=0.09)
    ax3d = fig.add_subplot(gs[:, 0], projection="3d")
    ax_hm = fig.add_subplot(gs[0, 1])
    ax_fr = fig.add_subplot(gs[1, 1])
    viz.draw_boxes3d(ax3d, s["placed"], cfg)
    viz.draw_heightmap(ax_hm, s["hm"], cfg)
    viz.draw_fill_curve(ax_fr, fills, caps, k)
    fig.suptitle(f"적재 {s['n_ok']}개 · 버림 {s['n_skip']}개   |   "
                 f"적재율 {s['fill'] * 100:.1f}%(실제높이) / {s['cap'] * 100:.1f}%(용량)   |   "
                 f"최고 {int(s['hm'].max())}mm / {cfg['pallet']['max_stack_mm']}mm",
                 fontsize=12)
    fig.savefig(path, dpi=120)
    plt.close(fig)


def _index_md(cfg, rows_cb, rows_rb, stages, last, n_boxes, seeds, seed_show):
    pal = cfg["pallet"]
    sat = rows_cb[0]
    L = [
        "# 트랙 C 결과 한 벌 — 패킹 알고리즘",
        "",
        f"팔레트 {pal['width_mm']}×{pal['depth_mm']} · 최대 적재 {pal['max_stack_mm']}mm · "
        f"격자 {cfg['heightmap']['cell_mm']}mm",
        "",
        "> 전부 `ros2 run packing_algo report` 한 번으로 다시 뽑힌다. "
        "표와 그림이 **같은 코드 상태**를 가리킨다는 뜻이다.",
        "",
        "## 1. 적재 과정 — Isaac 과 대조할 것",
        "",
        f"★ 시드 **{seed_show}** 으로 그렸다. 30시드 중 **적재율이 중앙값에 가장 가까운** "
        "판이다 — 기본 시드를 그냥 쓰면 그 판이 우연히 높거나 낮아 아래 표와 어긋난다. "
        "평균이 아니라 중앙값을 쓴 이유는, 평균에 맞추면 좋아 보이는 쪽으로 기울 수 있어서다.",
        "",
        "| 파일 | 무엇 |",
        "|---|---|",
        "| `01_stack.mp4` | 박스가 하나씩 놓이는 전 과정 |",
    ]
    for name, k in stages:
        L.append(f"| `{name}` | 적재 {k}개 시점 |")
    L += [
        "",
        "**Isaac 과 이렇게 비교한다.** 왼쪽 3D 그림의 박스 위치·색(3호 파랑 / 4호 주황 / "
        "5호 초록)이 Isaac 팔레트와 같아야 한다. 다르면 좌표 계약(`frames.py`)이나 "
        "실측 pose 회신을 의심한다.",
        "",
        "오른쪽 위 heightmap 은 **알고리즘이 믿고 있는 팔레트 상태**다. 실제 더미와 "
        "다르면 `update_heightmap` 에 계획값이 들어간 것이다 (계약: 실측값만).",
        "",
        "오른쪽 아래 곡선이 두 개인 이유 — 분모가 다르다:",
        "",
        "- **실제 높이 기준** (§7.8 정본): 층이 올라갈 때 분모가 뛰어 **뚝 떨어진다.** 정상이다.",
        "- **용량 기준** (max_stack 고정): 단조 증가. 발표에서 설명하기 쉽다.",
        f"- 포화까지 채우면 둘이 만난다 — 최종 {last['fill'] * 100:.1f}% / {last['cap'] * 100:.1f}%.",
        "",
        "## 2. 얼마나 잘 쌓나 — 베이스라인 대비 (`03_compare_baseline.png`)",
        "",
        f"시드 {seeds}개 · 팔레트 포화까지 · 두 패커에 **같은 박스 시퀀스**",
        "",
        "| 알고리즘 | 적재 개수 | 적재율 |",
        "|---|---|---|",
    ]
    for r in rows_cb:
        L.append(f"| {r['label']} | {r['placed']:.1f} | **{r['fill']:.1f}%** |")
    if len(rows_cb) >= 2:
        L.append("")
        L.append(f"**차이 +{rows_cb[0]['fill'] - rows_cb[1]['fill']:.1f}%p.** "
                 "베이스라인은 회전 없이 좌하단부터 처음 들어가는 자리에 넣는다 "
                 "— '아무 생각 없이 쌓기'의 대조군이다.")
    L += [
        "",
        "## 3. 오차를 얼마나 견디나 (`04_robustness.png`)",
        "",
        f"시드 {seeds}개 · 실측 pose 가 계획에서 벗어났을 때",
        "",
        "| 조건 | 적재율 | 오차없음 대비 |",
        "|---|---|---|",
    ]
    base = rows_rb[0]["fill"]
    for r in rows_rb:
        L.append(f"| {r['label']} | {r['fill']:.1f}% | {r['fill'] - base:+.1f}%p |")
    L += [
        "",
        "**위치는 널널하고 yaw 가 아프다.** `update()` 가 셀 격자(10mm)로 반올림하므로 "
        "위치 ±3mm 는 계획과 같은 칸에 떨어진다. 반면 yaw 는 허용치 "
        f"{cfg['heightmap']['yaw_tol_deg']:.0f}° 를 넘는 순간 외접 사각형으로 부풀려 "
        "기록돼 그 박스가 실제보다 넓은 자리를 먹는다.",
        "",
        "→ **로봇에 요구할 값: 놓는 순간 yaw 오차 ±5° 이내.** 위치는 ±10mm 여유.",
        "",
        "## 4. 실시간으로 보려면",
        "",
        "**ROS 모드 (기본).** 관제탑·로봇이 돌고 있으면 뷰어만 띄우면 붙는다 —",
        "`/robot/event` 의 실측 pose 와 `/zone_<z>/status` 를 듣는다.",
        "",
        "```bash",
        "# algo 에 heightmap 발행을 켜면 뷰어가 **algo 의 진짜 heightmap** 을 그린다.",
        "# (16UC1 sensor_msgs/Image · TRANSIENT_LOCAL — 뷰어를 늦게 띄워도 마지막 상태가 온다)",
        "ros2 run packing_algo algo_service_node --ros-args \\",
        "    -p use_sim_time:=true -p heightmap_topic:=/algo/heightmap",
        "ros2 run packing_algo live_view --ros-args -p use_sim_time:=true -p zone:=b",
        "```",
        "",
        "발행을 안 켜면 뷰어가 `/robot/event` 로 **재구성**해서 그린다. 둘 다 있으면",
        "algo 것을 그리고 재구성과 대조해, 어긋나면 화면에 경고를 띄운다",
        "(`UpdateHeightmap` 에 계획값이 들어갔다는 신호다 — §3-4 규약 3).",
        "",
        "⚠ 박스 3D 배치도는 `/robot/event` 로만 그린다. **로봇보다 먼저 띄울 것** —",
        "지난 이벤트는 다시 오지 않는다 (heightmap 은 TRANSIENT_LOCAL 이라 괜찮다).",
        "**파일 모드.** ROS 없이 알고리즘만 확인할 때.",
        "",
        "```bash",
        "ros2 run packing_algo algo_service_node --ros-args \\",
        "    -p use_sim_time:=true -p snapshot_path:=/tmp/algo_state.json",
        "python3 -m packing_algo.live_view --source file -p /tmp/algo_state.json -z b",
        "```",
        "",
        "위 그림과 같은 화면이 실시간으로 갱신된다. Isaac 창 옆에 띄워 두고 본다.",
    ]
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser(description="결과물 한 벌 생성 (발표·대조용)")
    ap.add_argument("--out", default="docs/트랙C_결과")
    ap.add_argument("--seeds", type=int, default=30)
    ap.add_argument("--boxes", type=int, default=60, help="영상에 흘려보낼 박스 수")
    ap.add_argument("--noise", type=int, default=3, help="영상의 실측 오차 ±mm")
    ap.add_argument("--fps", type=int, default=2)
    ap.add_argument("--target", type=float, default=70.0, help="적재율 목표 %%")
    args = ap.parse_args()

    cfg = load()
    out = args.out
    os.makedirs(out, exist_ok=True)
    seeds = list(range(1, args.seeds + 1))

    # ── 1. 적재 과정 ──────────────────────────────────────────
    #  ★ 어느 시드로 그릴 것인가. 기본 시드(42)를 그냥 쓰면 그 판이 우연히 낮게/높게
    #    나와 **영상과 표가 어긋난다** — 보는 사람은 체리피킹으로 읽는다.
    #    30시드를 먼저 돌려 **중앙값에 가장 가까운 시드**로 그린다. 평균에 제일
    #    가까운 것을 고르면 좋아 보이는 쪽으로 기울 수 있으니 중앙값을 쓴다.
    print(f"[1/4] 대표 시드 고르는 중 (시드 {args.seeds} 중 중앙값)…")
    fills = {s_: CB.run_once(HeightmapPacker, cfg, None, s_, s_ + 1000, args.noise)[3]
             for s_ in seeds}
    med = statistics.median(fills.values())
    seed_show = min(fills, key=lambda s_: abs(fills[s_] - med))
    print(f"      시드 {seed_show} (적재율 {fills[seed_show] * 100:.1f}% · "
          f"중앙값 {med * 100:.1f}%)")
    pk, steps = viz.simulate(cfg, n=args.boxes, noise=args.noise,
                             seed_box=seed_show, seed_noise=seed_show + 1000)
    viz.render_video(cfg, steps, os.path.join(out, "01_stack.mp4"), fps=args.fps)

    # ★ 옛 정지화면을 먼저 지운다. 파일 이름에 **적재 개수**가 들어가므로
    #   (`02_stage_21.png`) 알고리즘이 바뀌어 개수가 달라지면 옛 파일이 남는다.
    #   그러면 결과 폴더에 서로 다른 판본의 그림이 섞여 무엇이 최신인지 알 수 없다.
    for old_png in glob.glob(os.path.join(out, "02_stage_*.png")):
        os.remove(old_png)

    # 적재 개수가 목표에 처음 도달한 step 을 고른다 (버림 step 은 개수가 안 는다)
    stages = []
    for target in STAGE_AT:
        k = len(steps) - 1 if target is None else next(
            (i for i, s in enumerate(steps) if s["n_ok"] >= target), None)
        if k is None:
            continue
        name = f"02_stage_{steps[k]['n_ok']:02d}.png"
        if any(n == name for n, _ in stages):
            continue
        _stage_png(cfg, steps, k, os.path.join(out, name))
        stages.append((name, steps[k]["n_ok"]))
    print(f"      영상 + 정지화면 {len(stages)}장")

    # ★ 같은 시퀀스를 베이스라인에도 먹여 나란히 재생한다. 표의 "+%p" 는 숫자일 뿐
    #   무엇이 달라서 그런지 안 보인다 — 영상은 그걸 그대로 보여준다.
    from packing_algo.baseline import BaselinePacker
    _, steps_b = viz.simulate(cfg, n=args.boxes, noise=args.noise,
                              seed_box=seed_show, seed_noise=seed_show + 1000,
                              packer_cls=BaselinePacker)
    viz.render_compare_video(cfg, steps, steps_b,
                             os.path.join(out, "05_compare.mp4"), fps=args.fps)
    print(f"      나란히 비교 영상 (제안 {steps[-1]['n_ok']}개 vs "
          f"베이스라인 {steps_b[-1]['n_ok']}개)")

    # ── 2. 베이스라인 비교 ─────────────────────────────────────
    print(f"[2/4] 베이스라인 비교 (시드 {args.seeds})…")
    # counts=[] 로 부록(투입 개수별)은 건너뛴다 — 발표 지표는 "포화" 하나다
    sat, by_n = CB.sweep(cfg, [], seeds, args.noise)
    CB.plot(sat, seeds, args.noise,
            os.path.join(out, "03_compare_baseline.png"), args.target)
    with open(os.path.join(out, "03_compare_baseline.md"), "w") as f:
        f.write(CB.markdown_table(sat, by_n, [], seeds, args.noise, args.target) + "\n")
    rows_cb = [{"label": lb,
                "placed": sum(r[1] for r in rs) / len(rs),
                "fill": sum(r[3] for r in rs) / len(rs) * 100}
               for lb, rs in sat.items()]

    # ── 3. 오차 견딤 ──────────────────────────────────────────
    print(f"[3/4] 오차 견딤 (시드 {args.seeds})…")
    rows_rb = RB.sweep(cfg, seeds)
    RB.plot(rows_rb, os.path.join(out, "04_robustness.png"))
    with open(os.path.join(out, "04_robustness.md"), "w") as f:
        f.write(RB.markdown(rows_rb, seeds) + "\n")

    # ── 4. 색인 ───────────────────────────────────────────────
    print("[4/4] 색인…")
    last = steps[-1]
    with open(os.path.join(out, "README.md"), "w") as f:
        f.write(_index_md(cfg, rows_cb, rows_rb, stages, last,
                          args.boxes, args.seeds, seed_show) + "\n")

    print(f"\n완료 — {out}/")
    for n in sorted(os.listdir(out)):
        print(f"   {n}  ({os.path.getsize(os.path.join(out, n)) // 1024} KB)")


if __name__ == "__main__":
    main()
