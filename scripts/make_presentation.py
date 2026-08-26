#!/usr/bin/env python3
"""
Project Palletris 발표 PPT 생성 스크립트
템플릿: [협동3] 발표자료 KDT 양식 템플릿.pptx
출력:   Project_Palletris_발표.pptx
사용법: python3 scripts/make_presentation.py
"""
import copy, os, sys
from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR   = os.path.dirname(SCRIPT_DIR)
TEMPLATE   = os.path.join(ROOT_DIR, "[협동3] 발표자료 KDT 양식 템플릿.pptx")
OUTPUT     = os.path.join(ROOT_DIR, "Project_Palletris_발표.pptx")
IMG_DIR    = os.path.join(ROOT_DIR, "docs", "트랙C_결과")

def emu(inches):
    return Emu(int(inches * 914400))

def set_text(tf, text, font_size=None, bold=None, color=None, align=None):
    tf.clear()
    para = tf.paragraphs[0]
    if align is not None:
        para.alignment = align
    run = para.add_run()
    run.text = text
    if font_size:
        run.font.size = Pt(font_size)
    if bold is not None:
        run.font.bold = bold
    if color:
        run.font.color.rgb = RGBColor(*color)

def add_text_box(slide, x, y, w, h, text, font_size=14, bold=False,
                 color=(50,50,50), align=PP_ALIGN.LEFT, word_wrap=True):
    txBox = slide.shapes.add_textbox(emu(x), emu(y), emu(w), emu(h))
    tf = txBox.text_frame
    tf.word_wrap = word_wrap
    set_text(tf, text, font_size=font_size, bold=bold, color=color, align=align)
    return txBox

def add_bullet_box(slide, x, y, w, h, items, title=None, font_size=13,
                   title_size=16, color=(50,50,50), title_color=(0,84,166)):
    txBox = slide.shapes.add_textbox(emu(x), emu(y), emu(w), emu(h))
    tf = txBox.text_frame
    tf.word_wrap = True
    if title:
        para = tf.paragraphs[0]
        run = para.add_run()
        run.text = title
        run.font.size = Pt(title_size)
        run.font.bold = True
        run.font.color.rgb = RGBColor(*title_color)
    for item in items:
        para = tf.add_paragraph()
        para.space_before = Pt(3)
        run = para.add_run()
        run.text = "• " + item
        run.font.size = Pt(font_size)
        run.font.color.rgb = RGBColor(*color)
    return txBox

def add_table_box(slide, x, y, w, h, rows_data, header_color=(0,84,166), font_size=12):
    rows = len(rows_data)
    cols = len(rows_data[0])
    table = slide.shapes.add_table(rows, cols, emu(x), emu(y), emu(w), emu(h)).table
    table.first_row = True
    for r, row in enumerate(rows_data):
        for c, cell_text in enumerate(row):
            cell = table.cell(r, c)
            tf = cell.text_frame
            tf.word_wrap = True
            para = tf.paragraphs[0]
            para.alignment = PP_ALIGN.CENTER
            run = para.add_run()
            run.text = str(cell_text)
            run.font.size = Pt(font_size)
            if r == 0:
                run.font.bold = True
                run.font.color.rgb = RGBColor(255,255,255)
                cell.fill.solid()
                cell.fill.fore_color.rgb = RGBColor(*header_color)
    return table

def duplicate_slide(prs, slide_idx):
    template_slide = prs.slides[slide_idx]
    layout = template_slide.slide_layout
    new_slide = prs.slides.add_slide(layout)
    new_slide.shapes._spTree.clear()
    for child in template_slide.shapes._spTree:
        new_slide.shapes._spTree.append(copy.deepcopy(child))
    return new_slide

def build_presentation():
    prs = Presentation(TEMPLATE)

    # ── 슬라이드 01: 표지 ──
    s01 = duplicate_slide(prs, 0)
    for shape in s01.shapes:
        if not shape.has_text_frame: continue
        txt = shape.text_frame.text.strip()
        if "협동 - 3" in txt or "팀 프로젝트명" in txt:
            set_text(shape.text_frame,
                "Project Palletris\n스마트 물류창고 자율 팔레타이징 시스템",
                font_size=22, bold=True, color=(255,255,255))
        elif "어벤져스" in txt:
            set_text(shape.text_frame, "Palletris Team", font_size=16, color=(200,220,255))
        elif "그룹 - 조" in txt:
            set_text(shape.text_frame, "협동3조", font_size=14, color=(200,220,255))
    add_text_box(s01, 7.0, 5.0, 5.8, 0.5,
        "김학민(트랙A) · 김병직(트랙B) · 정정인(트랙C) · 민범진(트랙D)",
        font_size=13, color=(220,235,255))
    add_text_box(s01, 7.0, 5.55, 5.8, 0.4,
        "Isaac Sim 5.1 · ROS 2 Jazzy · 두산 H2017 · 6일 캡스톤",
        font_size=11, color=(180,200,240))
    print("OK 슬라이드 01 표지")

    # ── 슬라이드 02: 목차 ──
    s02 = duplicate_slide(prs, 1)
    add_text_box(s02, 1.0, 1.4, 11.0, 0.6, "목  차",
        font_size=28, bold=True, color=(0,84,166), align=PP_ALIGN.CENTER)
    toc = [
        ("01", "프로젝트 개요",           "무엇을 만들었나 · 문제 정의 · 시스템 구조"),
        ("02", "팀 구성 및 역할",          "트랙 A/B/C/D · 인프라"),
        ("03", "수행 절차 및 방법",        "6일 타임라인 · Day1 게이팅 검증"),
        ("04-1","알고리즘 — 온라인 3D 빈 패킹","Heightmap + DBLF + Lookahead + 낙하 완화"),
        ("04-2","로봇 제어 — 협곡 문제",  "접촉 감지 · 사이클 8.2s 개선"),
        ("04-3","비전 / 인식 / DB",        "카메라 4종 · ROS 2 파이프라인 · SQLite"),
        ("04-4","검증 결과",               "Isaac 실측 75.5% · 베이스라인 +30.6%p"),
        ("05",  "자체 평가 및 향후 계획", "남은 과제 · 개선 방향"),
    ]
    for i,(num,title,sub) in enumerate(toc):
        y = 2.2 + i*0.58
        add_text_box(s02, 0.8, y, 0.7, 0.48, num, font_size=13, bold=True, color=(0,84,166))
        add_text_box(s02, 1.6, y, 4.2, 0.48, title, font_size=13, bold=True, color=(20,20,20))
        add_text_box(s02, 5.9, y, 7.0, 0.48, sub, font_size=12, color=(100,100,100))
    print("OK 슬라이드 02 목차")

    # ── 슬라이드 03: 섹션01 표지 ──
    s03 = duplicate_slide(prs, 2)
    add_text_box(s03, 0.5, 2.5, 12.3, 1.0, "01  프로젝트 개요",
        font_size=36, bold=True, color=(255,255,255), align=PP_ALIGN.CENTER)
    add_text_box(s03, 0.5, 3.7, 12.3, 0.6, "무엇을 만들었나 · 문제 정의 · 시스템 구조",
        font_size=18, color=(200,220,255), align=PP_ALIGN.CENTER)
    print("OK 슬라이드 03 섹션01")

    # ── 슬라이드 04: 개요 수치 ──
    s04 = duplicate_slide(prs, 3)
    add_text_box(s04, 0.5, 0.2, 12.0, 0.5, "1-1. 프로젝트 한 줄 요약 & 핵심 수치",
        font_size=20, bold=True, color=(0,84,166))
    add_text_box(s04, 0.5, 0.85, 12.0, 0.6,
        "컨베이어로 오는 박스를 로봇 팔이 집어, 3D 빈패킹 알고리즘이 계산한 자리에 쌓고, 팔레트째 트럭에 넣는다.",
        font_size=15, bold=True, color=(30,30,30))
    kpi = [
        ["지표","결과","비고"],
        ["적재율 (Isaac 실측, 3시드)","75.5%","시드별 80.3 / 73.1 / 73.2%"],
        ["베이스라인 대비","+30.6%p","대조군 39.7% → 최종 70.3%"],
        ["사이클 타임","8.2~10.6 s","목표 10.0 s 달성"],
        ["종료 조건 (3시드 전부)","PALLET_FULL","빈자리 부족 아닌 팔레트 포화"],
        ["계획 재현성","4/4 동일","개수·적재율·좌표까지 완전 재현"],
    ]
    add_table_box(s04, 0.5, 1.65, 12.0, 2.7, kpi, font_size=13)
    add_text_box(s04, 0.5, 4.5, 12.0, 0.35,
        "★ 오늘 하루 궤적: 64.5% → 70.9% → 75.5% (낙하 완화 도입 전후)",
        font_size=12, bold=True, color=(180,50,50))
    add_bullet_box(s04, 0.5, 5.0, 12.0, 2.3,
        ["Isaac Sim 5.1 + ROS 2 Jazzy + 두산 H2017 (reach 1700mm)",
         "다중 PC ROS 2 통신 · ROS_DOMAIN_ID 115 통일",
         "박스 3종(3호 3kg / 4호 5kg / 5호 9kg) · 팔레트 1200×800mm · 최대 적재 1300mm",
         "random_seed=42 고정 → 베이스라인 비교에 필수"],
        title="환경 요약", font_size=12, title_size=14)
    print("OK 슬라이드 04 개요수치")

    # ── 슬라이드 05: 시스템 구조 ──
    s05 = duplicate_slide(prs, 3)
    add_text_box(s05, 0.5, 0.2, 12.0, 0.5, "1-2. 시스템 전체 구조",
        font_size=20, bold=True, color=(0,84,166))
    flow = [
        "박스 스폰 → 컨베이어 이송 (0.5m/s, 9.5m) → cam1 투입부 인식 (크기/ID) → cam2 검증",
        "→ 중재자 목적지 판단 (A/B/C 구역, 응답 0.6초 제한) → 스토퍼 정지",
        "→ pick_cam 픽업 좌표 실측 (x·y·yaw) → 벨트 정지",
        "→ 적재 알고리즘 자리 계산 (평균 163ms) → 로봇 집기·놓기",
        "→ 실측 pose로 heightmap 갱신 → 벨트 재가동 → 반복",
        "→ PALLET_FULL 시 트럭 상차",
    ]
    add_bullet_box(s05, 0.5, 0.85, 12.0, 2.4, flow,
        title="박스 1개 사이클 흐름", font_size=13, title_size=15)
    node_data = [
        ["노드","개수","담당","역할"],
        ["관제탑","3","트랙B","로봇 명령 유일 발신자 · 상태머신 7단계 (IDLE→FETCHING→PLANNING→PICKING→PLACING→UPDATING→DONE)"],
        ["구역 인식","3","트랙D","도착 감지 + 픽업 좌표 발행"],
        ["라인 인식","2","트랙D","cam1 판독 · cam2 검증"],
        ["중재자","1","트랙D","목적지 배분 · 되돌림 판단 · 버퍼 우회"],
        ["적재 알고리즘","1","트랙C","heightmap 3장 소유 · 자리 계산 (로봇 명령 금지)"],
        ["DB 서비스","1","트랙D","palletris.db 를 여는 유일한 프로세스"],
    ]
    add_table_box(s05, 0.5, 3.4, 12.0, 3.8, node_data, font_size=11)
    print("OK 슬라이드 05 시스템구조")

    # ── 슬라이드 06: 섹션02 표지 ──
    s06 = duplicate_slide(prs, 2)
    add_text_box(s06, 0.5, 2.5, 12.3, 1.0, "02  팀 구성 및 역할",
        font_size=36, bold=True, color=(255,255,255), align=PP_ALIGN.CENTER)
    add_text_box(s06, 0.5, 3.7, 12.3, 0.6, "트랙 A/B/C/D · 인프라 · 협업 방식",
        font_size=18, color=(200,220,255), align=PP_ALIGN.CENTER)
    print("OK 슬라이드 06 섹션02")

    # ── 슬라이드 07: 팀 구성 ──
    s07 = duplicate_slide(prs, 7)
    add_text_box(s07, 0.5, 0.2, 12.0, 0.5, "2-1. 트랙별 역할 분담",
        font_size=20, bold=True, color=(0,84,166))
    team_data = [
        ["트랙","담당자","영역","핵심 산출물"],
        ["A","김학민","Isaac 씬 / 에셋",
         "USD 씬 · 컨베이어 · 카메라 4종(cam1/2/pick_cam/wrist_cam) · tf/clock 발행"],
        ["B","김병직","로봇 제어",
         "관제탑 3대 · Lula IK · pick&place · 협곡 낙하 해결 · 사이클 8.2s"],
        ["C","정정인","패킹 알고리즘",
         "Heightmap + DBLF + Lookahead · 적재율 75.5% · 로봇 통로 검사(_corridor_ok)"],
        ["D","민범진","비전/인식/DB",
         "cam 파이프라인 · SQLite DB 서비스 · 중재자 · 인터페이스 설계"],
    ]
    add_table_box(s07, 0.5, 0.85, 12.0, 3.1, team_data, font_size=13)
    add_bullet_box(s07, 0.5, 4.1, 12.0, 3.2,
        ["4대 PC · 2.5G 스위치 · 고정 IP 10.10.0.1~4 · FastDDS 유선 화이트리스트 (ping 0.25ms)",
         "Isaac Sim 전용 1대 / DB·알고리즘·인식은 별도 PC (GPU 부하 분산, FPS 확보)",
         "USD 씬: 트랙A만 수정 — rsync scripts/sync_scene.sh 배포 (git에 USD 금지)",
         "공용 파일 수정 = 팀 전원 재빌드 · 반드시 보고",
         "ROS_DOMAIN_ID=115 · use_sim_time=true 전원 통일 (다르면 조용히 통신 불가)"],
        title="인프라 & 협업 규칙", font_size=12, title_size=14)
    print("OK 슬라이드 07 팀구성")

    # ── 슬라이드 08: 섹션03 표지 ──
    s08 = duplicate_slide(prs, 2)
    add_text_box(s08, 0.5, 2.5, 12.3, 1.0, "03  수행 절차 및 방법",
        font_size=36, bold=True, color=(255,255,255), align=PP_ALIGN.CENTER)
    add_text_box(s08, 0.5, 3.7, 12.3, 0.6, "6일 타임라인 · 인터페이스 설계 · Day1 게이팅 검증",
        font_size=18, color=(200,220,255), align=PP_ALIGN.CENTER)
    print("OK 슬라이드 08 섹션03")

    # ── 슬라이드 09: 타임라인 ──
    s09 = duplicate_slide(prs, 8)
    add_text_box(s09, 0.5, 0.2, 12.0, 0.5, "3-1. 6일 수행 타임라인",
        font_size=20, bold=True, color=(0,84,166))
    timeline_data = [
        ["일차","날짜","주요 활동","비고"],
        ["Day1","8/21","설계 결정서 확정 · G1~G7 게이팅 검증 · 인터페이스 합의","ROS_DOMAIN_ID 115 통일"],
        ["Day2","8/22","트랙별 기본 구현 · USD 씬 빌드 · DB 서비스 노드","Stage1 (B구역 단독)"],
        ["Day3","8/23","1차 통합 테스트 · 11박스 RESULT 통과 · 사이클 12.1s","확정본 영상 촬영"],
        ["Day4","8/24","코드 리뷰 7건 처리 · 적재율 수정 · 카메라 좌표 인식","2차 발표 준비"],
        ["Day5","8/25","협곡 낙하 해결 · 사이클 8.2s · 적재율 75.5%","Stage2 다중 존 병합"],
        ["Day6","8/26","최종 통합 · 관제탑 이중화 해소 · 발표 준비","최종 PPT 완성"],
    ]
    add_table_box(s09, 0.5, 0.85, 12.0, 4.0, timeline_data, font_size=12)
    add_bullet_box(s09, 0.5, 5.0, 12.0, 2.3,
        ["Day1 인터페이스 합의 = 4인 완전 독립 병렬 작업 가능 (서비스 5개만 합의)",
         "C·D 트랙은 Isaac GPU 없이 노트북에서 개발 (GPU 3대 대기 없음)",
         "USD는 git 불가 → rsync + scripts/sync_scene.sh 전용 배포",
         "진짜 리스크는 일정이 아니라 통합 — Day1 인터페이스가 핵심"],
        title="협업 전략 포인트", font_size=12, title_size=14)
    print("OK 슬라이드 09 타임라인")

    # ── 슬라이드 10: 섹션04 표지 ──
    s10 = duplicate_slide(prs, 2)
    add_text_box(s10, 0.5, 2.5, 12.3, 1.0, "04  프로젝트 수행 경과",
        font_size=36, bold=True, color=(255,255,255), align=PP_ALIGN.CENTER)
    add_text_box(s10, 0.5, 3.7, 12.3, 0.6, "알고리즘 · 로봇 제어 · 비전/DB · 검증 결과",
        font_size=18, color=(200,220,255), align=PP_ALIGN.CENTER)
    print("OK 슬라이드 10 섹션04")

    # ── 슬라이드 11: 알고리즘 ──
    s11 = duplicate_slide(prs, 9)
    add_text_box(s11, 0.5, 0.2, 12.0, 0.5, "4-1. 패킹 알고리즘 — 온라인 3D 빈 패킹",
        font_size=20, bold=True, color=(0,84,166))
    add_bullet_box(s11, 0.5, 0.85, 5.8, 2.6,
        ["박스 목록을 미리 모른다 (온라인 — 하나씩 도착)",
         "한 번 놓으면 되돌릴 수 없다",
         "6축 로봇 팔 — 못 가는 자리가 있다",
         "→ 순수 패킹 논문과 다른 3가지"],
        title="우리 문제 특수성", font_size=12, title_size=14)
    add_bullet_box(s11, 6.5, 0.85, 6.3, 2.6,
        ["Heightmap (10mm 격자 120x80) — 표면 높이 배열, plan() 평균 2ms",
         "DBLF + 가중합: w_height(0.6) · w_support(0.4) · w_wall(0.2) · w_far(0.1) · w_level(0.05)",
         "Lookahead Rollout (k=4, rollouts=3, depth=8) — plan() 163ms",
         "_corridor_ok — 그리퍼 Ø245mm 통로 검사 (핵심 차별점)",
         "낙하 완화 — drop_relief_mm=200, drop_max_kg=3.5"],
        title="알고리즘 구성", font_size=12, title_size=14)
    cmp = [
        ["계열","맞나","이유"],
        ["Wall building (offline)","X","박스 목록 사전 필요"],
        ["Layer building","△","같은 높이 박스가 모여야 함"],
        ["DBLF","대조군","베이스라인으로 채택"],
        ["Heightmap + 가중합","채택","팔레타이징 실무 표준"],
        ["심층강화학습 (DRL)","X","6일 일정 · 재현성 요구 불가"],
    ]
    add_table_box(s11, 0.5, 3.6, 12.0, 3.3, cmp, font_size=11)
    print("OK 슬라이드 11 알고리즘")

    # ── 슬라이드 12: 알고리즘 결과 (이미지) ──
    s12 = duplicate_slide(prs, 9)
    add_text_box(s12, 0.5, 0.2, 12.0, 0.5, "4-2. 알고리즘 결과 — 베이스라인 비교 & 강건성",
        font_size=20, bold=True, color=(0,84,166))
    compare_img = os.path.join(IMG_DIR, "03_compare_baseline.png")
    robust_img  = os.path.join(IMG_DIR, "04_robustness.png")
    if os.path.exists(compare_img):
        s12.shapes.add_picture(compare_img, emu(0.5), emu(0.9), emu(7.0), emu(3.4))
        print("  img: 베이스라인 비교")
    if os.path.exists(robust_img):
        s12.shapes.add_picture(robust_img, emu(7.7), emu(0.9), emu(5.3), emu(3.4))
        print("  img: 강건성")
    result_data = [
        ["알고리즘","적재 개수","적재율","표준편차","대조군 대비"],
        ["① 대조군 (DBLF, 회전 없음)","15.6","39.7%","+-8.9","—"],
        ["② Heightmap 그리디","23.1","62.1%","+-5.9","+22.4%p"],
        ["③ + Lookahead (rollout)","23.8","64.3%","+-5.9","+24.6%p"],
        ["④ + 낙하 완화 (최종)","26.5","70.3%","+-5.4","+30.6%p"],
    ]
    add_table_box(s12, 0.5, 4.45, 12.0, 2.5, result_data, font_size=12)
    add_text_box(s12, 0.5, 7.05, 12.0, 0.35,
        "★ 로봇 요구: 놓는 순간 yaw 오차 ±5° 이내 / 위치 ±10mm 여유  |  표준편차 8.9→5.4: 어떤 박스 순서에도 안정",
        font_size=11, bold=True, color=(180,50,50))
    print("OK 슬라이드 12 알고리즘결과")

    # ── 슬라이드 13: 로봇 제어 ──
    s13 = duplicate_slide(prs, 9)
    add_text_box(s13, 0.5, 0.2, 12.0, 0.5, "4-3. 로봇 제어 — 협곡 낙하 & 사이클 개선",
        font_size=20, bold=True, color=(0,84,166))
    add_bullet_box(s13, 0.5, 0.85, 5.8, 2.8,
        ["다층 적재 → 박스 사이 협곡(~250mm 슬롯) 발생",
         "그리퍼 몸통 Ø245mm → 팔이 못 내려감",
         "통로 반경 확대 시도: 26→22개 (-15%) 실패",
         "해결: 협곡 입구에서 놓고 낙하 (3.5kg 이하 3호 박스)",
         "기준: 박스 윗면을 (입구 - 그리퍼길이150mm)까지",
         "480mm 낙하(이탈 22.3mm) → 120mm 낙하(1.5mm)"],
        title="협곡 낙하 해결 (2026-08-25 신규)", font_size=12, title_size=14)
    add_bullet_box(s13, 6.5, 0.85, 6.3, 2.8,
        ["Lula IK: supports_collision_avoidance() = False",
         "해결: 툴 정지 감지 → 6틱 안에 지령을 현재 관절각으로 되돌림",
         "이전: 잔차<50mm 조건 → 55.6mm 막히면 2.5초 밀었음",
         "그 사이 이웃 박스 17.2mm 밀림"],
        title="접촉 감지 (Lula IK 한계 우회)", font_size=12, title_size=14)
    cycle_data = [
        ["개선 항목","사이클","근거"],
        ["초기","18.3 s","베이스라인"],
        ["도착 판정: 절대오차→수렴 정체","12.6 s","정착 잔차 1.03~2.05mm (임계 1mm가 최소보다 작았음)"],
        ["구간별 허용오차 (이송 8mm)","11.7 s","웨이포인트 8개 중 6개 이송인데 하강과 동일 정밀도 요구"],
        ["joint_speed_scale 0.8→1.0","10.6 s","정격 상한"],
        ["협곡 낙하 도입 후","8.2 s","픽→놓기 경로 단축"],
    ]
    add_table_box(s13, 0.5, 3.8, 12.0, 2.9, cycle_data, font_size=12)
    add_text_box(s13, 0.5, 6.85, 12.0, 0.5,
        "계측(--diag-time) 없이 엉뚱한 곳 최적화 방지 — 실제 관절 이동은 19.5s 사이클 중 4.8s뿐이었음",
        font_size=11, color=(100,100,100))
    print("OK 슬라이드 13 로봇제어")

    # ── 슬라이드 14: 비전/DB ──
    s14 = duplicate_slide(prs, 9)
    add_text_box(s14, 0.5, 0.2, 12.0, 0.5, "4-4. 비전 / 인식 / DB 파이프라인",
        font_size=20, bold=True, color=(0,84,166))
    cam_data = [
        ["카메라","위치","역할"],
        ["cam1","x=600 (투입부)","크기 측정 · QR 인식 → DB 등록"],
        ["cam2","x=1400 (검증)","cam1 결과 교차 검증"],
        ["pick_cam","스토퍼 위 (0,2300) 하향","픽업 좌표 실측 (x·y·yaw)"],
        ["wrist_cam","그리퍼 몸통 offset(170,0,30) tilt18도","집는 순간 hand-eye 확인"],
    ]
    add_table_box(s14, 0.5, 0.85, 12.0, 2.3, cam_data, font_size=12)
    add_bullet_box(s14, 0.5, 3.3, 5.8, 3.8,
        ["QoS: BEST_EFFORT (RELIABLE 시 이미지 수신 불가)",
         "오프스크린 렌더: 돔 라이트 꺼지면 카메라 캄캄 (강도 1000 필수)",
         "wrist_cam: 형상 파묻힘 → x 100→170mm 이동으로 해결",
         "해상도 1280x720 · HFOV 60° · 30Hz",
         "use_sim_time=true 미설정 시 tf lookup 전부 실패"],
        title="카메라 주의사항", font_size=12, title_size=14)
    add_bullet_box(s14, 6.5, 3.3, 6.3, 3.8,
        ["SQLite + WAL (0.25건/s → 수천건/s 여유. PostgreSQL 불필요)",
         "db_service_node가 DB를 여는 유일한 프로세스 (database is locked 방지)",
         "5개 서비스: register_box / verify_box / next_box / mark_placed / zone_queue",
         "다른 트랙이 스키마 몰라도 서비스만 호출하면 됨 (Day1 합의의 핵심)",
         "boxes_sorted.status로 전 생애 추적 → 발표 지표 SQL로 추출"],
        title="DB 설계 (SQLite)", font_size=12, title_size=14)
    print("OK 슬라이드 14 비전DB")

    # ── 슬라이드 15: 검증 결과 ──
    s15 = duplicate_slide(prs, 9)
    add_text_box(s15, 0.5, 0.2, 12.0, 0.5, "4-5. Isaac 실측 검증 결과",
        font_size=20, bold=True, color=(0,84,166))
    stage_img = os.path.join(IMG_DIR, "02_stage_21.png")
    if os.path.exists(stage_img):
        s15.shapes.add_picture(stage_img, emu(0.5), emu(0.85), emu(7.0), emu(3.9))
        print("  img: 최종 적재 상태")
    add_bullet_box(s15, 7.7, 0.85, 5.4, 3.9,
        ["적재 21개 · 최고 1300mm (팔레트 포화)",
         "적재율 60.5% / 60.5% (포화 시 두 기준 일치)",
         "",
         "Isaac 실측 3시드 최종:",
         "  시드42: 28/29 → 80.3%",
         "  시드 7: 28/32 → 73.1%",
         "  시드99: 29/32 → 73.2%",
         "  평균: 75.5%"],
        title="최종 적재 상태 (팔레트 포화)", font_size=11, title_size=13)
    repro_data = [
        ["실행","적재/투입","적재율","knock(mm)","팔레트밖(mm)","판정"],
        ["1회","28/29","80.3%","31.7","30.2","실패"],
        ["2회","28/29","80.3%","3.8","4.5","통과"],
        ["3회","28/29","80.3%","6.5","7.2","통과"],
        ["4회","28/29","80.3%","20.0","4.3","통과"],
    ]
    add_table_box(s15, 0.5, 4.9, 12.0, 2.2, repro_data, font_size=12)
    add_text_box(s15, 0.5, 7.18, 12.0, 0.28,
        "★ 계획은 4/4 완전 재현 (재현성 평가 충족) · 물리만 갈림 — 낙하 양의 되먹임이 원인",
        font_size=11, bold=True, color=(180,50,50))
    print("OK 슬라이드 15 검증결과")

    # ── 슬라이드 16: 검증 방법론 ──
    s16 = duplicate_slide(prs, 9)
    add_text_box(s16, 0.5, 0.2, 12.0, 0.5, "4-6. 검증 방법론 — 이 프로젝트에서 배운 것",
        font_size=20, bold=True, color=(0,84,166))
    lesson = [
        ["교훈","내용"],
        ["데스크 vs 시뮬 부호가 반대",
         "방향성 통로: 데스크 -0.8%p 예측 → Isaac +3개. 데스크 실패는 공짜지만 실측 실패 2건이 적재 3개를 깎음"],
        ["사람 눈 한 번은 필수",
         "돌아갈 때 이송 고도 버그 — 계측으론 숫자가 위장됨. 화면이 잡아냄. 고친 뒤 knock 700.8→0.8mm"],
        ["재현성: 계획 O, 물리 X",
         "계획 알고리즘은 결정적. 물리(낙하)만 갈림. 적재율은 신뢰값, 안정성 수치는 범위로 읽어야"],
        ["계측 없이 최적화 금지",
         "--diag-time 없었으면 엉뚱한 곳 최적화. 실제 관절 이동 4.8s / 전체 19.5s"],
    ]
    add_table_box(s16, 0.5, 0.85, 12.0, 3.9, lesson, font_size=11)
    add_bullet_box(s16, 0.5, 4.9, 12.0, 2.4,
        ["데스크 계산 스윕은 후보를 좁히는 용도 — 채택 판정은 Isaac이 한다",
         "계측은 '무엇이 얼마나'를 답하고, 화면은 '무엇이 빠졌는지'를 답한다",
         "빠진 것은 로그에 안 찍힌다 — 없는 동작은 없는 줄로 나타나지 않는다",
         "섞인 채로 종류별 최적 수준(72.5%)을 넘었다 — 75.5%"],
        title="핵심 원칙 (팀 합의)", font_size=13, title_size=15, title_color=(0,120,50))
    print("OK 슬라이드 16 검증방법론")

    # ── 슬라이드 17: 섹션05 표지 ──
    s17 = duplicate_slide(prs, 2)
    add_text_box(s17, 0.5, 2.5, 12.3, 1.0, "05  자체 평가 및 향후 계획",
        font_size=36, bold=True, color=(255,255,255), align=PP_ALIGN.CENTER)
    add_text_box(s17, 0.5, 3.7, 12.3, 0.6, "평가 항목 대응 · 남은 과제 · 개선 방향",
        font_size=18, color=(200,220,255), align=PP_ALIGN.CENTER)
    print("OK 슬라이드 17 섹션05")

    # ── 슬라이드 18: 자체 평가 ──
    s18 = duplicate_slide(prs, 16)
    add_text_box(s18, 0.5, 0.2, 12.0, 0.5, "5-1. 평가 항목 대응 & 자체 완성도",
        font_size=20, bold=True, color=(0,84,166))
    eval_data = [
        ["평가 항목","우리 대응","자체 점수"],
        ["ROS2 연동 (clock·이미지·tf)","Action Graph 3종 발행","9/10"],
        ["동적 대응 (랜덤 스폰)","스폰 y·yaw 랜덤화 — 미구현","6/10"],
        ["다중 로봇 조율","로봇 3대 + 관제탑 3개 + 중재자","9/10"],
        ["예외 처리 (크래시 없음)","입력 검증 · 타임아웃 20s · cancel_plan","9/10"],
        ["처리 효율 (추론 주기)","천장 카메라 도착 신호 시에만 추론","8/10"],
        ["검증 및 분석 (정량)","적재수·적재율·최고높이·소요시간 전부","10/10"],
        ["재현성 (Play/Stop)","random_seed=42 · 계획 4/4 완전 재현","9/10"],
        ["챌린지 (개선 실증)","베이스라인 대비 +30.6%p 실측","10/10"],
    ]
    add_table_box(s18, 0.5, 0.85, 12.0, 4.2, eval_data, font_size=12)
    remain = [
        ["남은 과제","담당","이유"],
        ["소켓→ROS2 전환","트랙B","관제탑이 localhost:8765 소켓. 다중 PC 실행 전제 조건"],
        ["spawn_y/yaw 랜덤화","팀","동적 대응 평가 항목 미충족 (6/10)"],
        ["system.yaml 정본화","팀","현재 값 6개가 런타임 오버라이드"],
        ["gripper_clearance 30→10mm","트랙B","적재율 +7.2%p 예상. 툴 단면 실측 필요"],
    ]
    add_table_box(s18, 0.5, 5.2, 12.0, 2.1, remain, font_size=11)
    print("OK 슬라이드 18 자체평가")

    # ── 슬라이드 19: 마무리 ──
    s19 = duplicate_slide(prs, 0)
    add_text_box(s19, 0.5, 2.2, 12.3, 1.2, "Thank you",
        font_size=48, bold=True, color=(255,255,255), align=PP_ALIGN.CENTER)
    add_text_box(s19, 0.5, 3.5, 12.3, 0.6,
        "Project Palletris — 스마트 물류창고 자율 팔레타이징 시스템",
        font_size=18, color=(200,220,255), align=PP_ALIGN.CENTER)
    add_text_box(s19, 0.5, 4.3, 12.3, 0.5,
        "Isaac Sim 5.1  ·  ROS 2 Jazzy  ·  두산 H2017  ·  적재율 75.5%",
        font_size=14, color=(180,210,255), align=PP_ALIGN.CENTER)
    add_text_box(s19, 0.5, 5.0, 12.3, 0.4,
        "김학민 · 김병직 · 정정인 · 민범진",
        font_size=14, color=(200,220,255), align=PP_ALIGN.CENTER)
    print("OK 슬라이드 19 마무리")

    # ── 템플릿 원본 슬라이드 17장 삭제 ──
    slide_part = prs.slides._sldIdLst
    ids = list(slide_part)
    for sld_id in ids[:17]:
        slide_part.remove(sld_id)

    # ── 저장 ──
    prs.save(OUTPUT)
    print(f"\n완성 -> {OUTPUT}")
    print(f"   총 슬라이드: {len(prs.slides)}장")

if __name__ == "__main__":
    build_presentation()
