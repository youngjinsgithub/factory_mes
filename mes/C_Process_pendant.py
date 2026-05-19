"""
═══════════════════════════════════════════════════════════════════════
  FAiCTORY MES — 공정 C (비전 검사 + 종합 처리)
  ※ 시연용 MVP 버전 — 공정 A/B와 동일한 CC-Link + 펜던트 IF 패턴
═══════════════════════════════════════════════════════════════════════

  [이 PC의 역할]
    - B130 트리거 감지 → 비전 검사 자동 실행 (YOLO)
    - 양품/불량 판정 → 공정 C PLC로 결과 통보 (M250/M260)
    - tbl_robot_c에 검사 결과 INSERT
    - M1120 종료 신호 → tbl_total INSERT + carrier_map RELEASE

  [로봇 제어 방식 — 공정 A/B와 동일]
    Vision PC → PLC(120)에 M250/M260 통보 (양품/불량만)
       ↓
    PLC 자체 센서/래더 로직 → 로봇 트리거
       ↓
    펜던트 IF로 로봇 C 자체 동작

    (CC-Link 기반, 검증된 안정적 패턴 — 시연 안정성 최우선)
    Vision PC는 로봇과 직접 통신하지 않음. PLC가 모든 로봇 기동을 처리.

  [연결 구성]
    DB     : 192.168.3.141 (별도 서버, guest 계정)
    PLC    : 192.168.3.120 (공정 C 메인)
             ※ 공정 C에는 PLC 110(컨베어)도 있으나 PLC끼리 통신,
               Vision PC는 메인 PLC 120만 접속
    카메라 : Intel RealSense D435 (vision_test_done 검증 구성)
    로봇 C : 192.168.3.5 — PLC가 자체 제어 (Vision PC 직접 접속 X)
    모델   : 6class_best.pt (vision_test_done 검증 모델)

  [B 디바이스 통신 - PLC 간 신호 전달]
    공정 B PLC(130)의 B120(="120으로 보낸다")
      ⟷ 공정 C PLC(120)의 B130(="130에서 받았다")
    B 디바이스(Link Relay)로 PLC 간 양방향 매핑.
    명명 규칙: 비트명에 상대방 PLC 번호를 넣어 신호 출처 표시.
    → 공정 C PC는 자기 PLC(120)만 폴링하면 모든 신호 받음

  [전체 시스템 구조]
    공정 A: PLC 150 (작업 + 컨베어) — 펜던트 IF 자체 동작
    공정 B: PLC 140 (메인) + PLC 130 (로봇/컨베어/종료) — 펜던트 IF 자체 동작
    공정 C: PLC 120 (메인) + PLC 110 (컨베어) — 펜던트 IF 자체 동작 ★
    관제:   PLC 160 (SCADA용, Vision PC 직접 접속 X)

  [비전 판정 방식 — Voting 다수결]
    B130 ON → IGNORE_DURATION(0.4s) 안정화 대기 → 매 프레임 추론
    첫 검출 시점부터 VOTING_DURATION(1.5s) 동안 결과 누적
    다수결로 최종 클래스 채택 → 양품/불량 판정 → PLC + DB (사이클당 1회)
    B130 OFF → ON 다시 들어오면 잠금 해제 → 재트리거 가능

  [TODO — 시연 후 확장 작업]
    Busan_Robot HMI 통합:
      - 펜던트 티칭 → Conty JSON export
      - Vision PC가 Busan_Robot 추상화 컨트롤러 통해 로봇 직접 제어
      - 양품/불량별 별도 JSON으로 분기 동작 (Robot_C_normal.json 등)
      - 데이터-로직 분리 아키텍처로 확장
      → 별도 파일 (C_Process_busanrobot.py 등) 으로 작성 예정
═══════════════════════════════════════════════════════════════════════
"""

import os
import time
import threading
import cv2
import numpy as np
import pyrealsense2 as rs
import pymysql
from collections import Counter
from datetime import datetime
from time import sleep
from pymcprotocol import Type3E
from ultralytics import YOLO

# ═══════════════════════════════════════════════════════════════════════
# 설정
# ═══════════════════════════════════════════════════════════════════════

# ───── DB 서버 ─────
# 운영 DB 서버 (192.168.3.141) — guest 계정
DB_CONFIG = {
    'host': '192.168.3.141',           # 🖥️ 서버 IP (고정)
    'port': 3306,
    'user': 'guest',                   # 👤 모든 PC가 동일
    'password': 'guest1234',           # 🔑 모든 PC가 동일
    'db': 'faictory_mes',
    'charset': 'utf8mb4',
    'autocommit': True,
    'use_unicode': True,
    'init_command': "SET NAMES utf8mb4"
}

# ───── PLC 설정 ─────
PLC_IP   = "192.168.3.120"   # 공정 C 메인 PLC (트리거 수신/통보)
PLC_PORT = 2000

PLC_MONITOR_IP = "192.168.3.160"   # 관제 PLC (M1120 종료 신호 폴링)

# ───── PLC 비트 정의 ─────
# 읽기 (PLC 120 → Vision C)
ADDR_VISION_TRIGGER = "B130"    # 비전 검사 시작 트리거
                                # 공정 B PLC(130)의 B120 ⟷ 공정 C PLC(120)의 B130

# 읽기 (PLC 160 관제 → Vision C)
ADDR_DONE_C         = "M1120"   # 공정 C 종료 신호 (관제 PLC 160에서 읽음)

# 쓰기 (Vision C → PLC 120)
ADDR_NORMAL    = "M250"     # 양품 신호 (PLC 래더가 인식 → 자체 분기 처리)
ADDR_CRACK     = "M260"     # 불량 신호 (PLC 래더가 인식 → 자체 분기 처리)
# ※ 로봇 동작 트리거(과거 M5)는 PLC 자체 센서 입력으로 대체됨.
#    Vision PC는 양품/불량만 통보하고, 로봇 기동은 PLC 래더가 자체 처리.

# ───── YOLO 모델 ─────
# vision_test_done.py 에서 검증된 모델 — C_VISION.pt에서는 detection이 잘 안 되어 교체
MODEL_PATH = r"C:\Users\user\Desktop\intel_cam_prj\intel_cam\intel_cam\models\6class_best.pt"
# 초기 conf 임계값 — 실행 중 트랙바로 실시간 조정 가능
INITIAL_CONF_THRESHOLD = 0.50
NORMAL_CLASSES = ['r_normal', 'g_normal', 'b_normal']
CRACK_CLASSES  = ['r_crack',  'g_crack',  'b_crack']

# B130 ON 직후 센서/조명 흔들림으로 인한 초기 쓰레기 데이터 무시 시간 (초)
# 이 시간 경과 후부터 매 프레임 추론을 시작.
IGNORE_DURATION = 0.4

# Voting 윈도우 — 첫 검출 시점부터 N초 동안 결과를 누적해 다수결로 최종 판정.
# 측정 결과: 트레이 시야 체류 ~2.7s, 검출 구간 ~24fps → 1.5s × 24fps ≈ 36프레임 누적
# 한 프레임 오인식에 흔들리지 않으면서 시연 속도도 빠르게 유지하는 균형값.
VOTING_DURATION = 1.5

# ───── 카메라 설정 (Intel RealSense D435) ─────
CAMERA_WIDTH  = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS    = 30

plc = None              # 공정 C PLC 120 (트리거 수신/통보)
plc_monitor = None      # 관제 PLC 160 (종료 신호 폴링)
yolo_model = None
cam_pipeline = None     # RealSense pipeline

# 폴링용 이전 상태 (상승 엣지 감지)
prev_vision_trigger = False
prev_done_c = False

# 검출 상태 — Voting 판정 모드
# B130 상승 엣지 → trigger_on_time 갱신 + voting/잠금 상태 초기화
# IGNORE_DURATION 경과 후 매 프레임 추론 시도
# 첫 검출 발생 → voting_started_at 기록 + voting_buffer에 결과 누적 시작
# VOTING_DURATION 경과 → 다수결로 최종 클래스 결정 → PLC + DB (한 사이클 1회)
# B130 하강 후 다시 ON 시 새 사이클 시작
trigger_on_time = 0.0
result_sent_this_cycle = False
last_view_frame = None        # cv2 창에 표시할 최근 어노테이션 프레임
last_diag_at = 0.0            # 진단 로그 직전 출력 시각 (1초마다 1번)

# Voting 상태
voting_started_at = 0.0       # 첫 검출 시각 (0이면 아직 검출 전)
voting_buffer = []            # [(class_name, confidence), ...]

# FPS 측정 — 0.5초마다 갱신
fps_frame_count = 0
fps_last_time = time.time()
current_fps = 0.0

# PLC 백그라운드 폴링 — PLC 네트워크 지연(개별 read 호출이 2s 가까이 걸리는 케이스 관측됨)이
# 메인 루프 FPS를 죽이는 문제 해결. 별도 스레드가 PLC를 지속적으로 read → 캐시 갱신하고,
# 메인 루프(monitor_plc_signals)는 캐시만 읽음. 캐시 read는 O(1)이라 FPS와 완전 분리.
plc_lock = threading.Lock()           # plc (192.168.3.120) 소켓 직렬화 (BG read + 메인 write 충돌 방지)
plc_monitor_lock = threading.Lock()   # plc_monitor (192.168.3.160) 소켓 직렬화
plc_cache = {
    'vision_trigger': False,
    'done_c': False,
    'last_update_at': 0.0,
}
plc_cache_lock = threading.Lock()
plc_thread_stop = threading.Event()
PLC_POLL_TARGET_INTERVAL = 0.05       # BG 스레드 목표 폴링 주기 (20Hz). PLC가 느리면 자연 throttle.

# BG 스레드 진단용 — 직전 1초 동안 각 PLC read 평균 시간 + 폴링 횟수
# 스레드 분리됨 (PLC 120 / PLC 160) — 한쪽이 느려도 다른쪽 폴링 주기 영향 X
plc_poll_diag = {
    'trigger_total_ms': 0.0,
    'trigger_count': 0,
    'done_total_ms': 0.0,
    'done_count': 0,
}
plc_poll_diag_lock = threading.Lock()

# 메인 루프 구간별 타이밍 진단 — 어디가 병목인지 1초마다 한 줄로 출력.
# capture: RealSense wait_for_frames + numpy 변환
# plc:     monitor_plc_signals (PLC 네트워크 read)
# vision:  vision_step (YOLO 추론 + voting)
# display: cv2.imshow + waitKey
section_times = {'capture': 0.0, 'plc': 0.0, 'vision': 0.0, 'display': 0.0}
section_counts = {'capture': 0, 'plc': 0, 'vision': 0, 'display': 0}
last_timing_print = time.time()

# 사이클 누적 카운터 — B130 ON→OFF 한 사이클 동안 통계 수집
# voting 윈도우 길이 결정을 위한 측정용. B130 OFF 시 요약 출력.
cycle_total_frames = 0          # 안정화 이후 추론된 총 프레임
cycle_detect_frames = 0         # 그 중 검출 성공 프레임 수
cycle_class_counter = Counter() # 검출된 클래스 분포
cycle_first_detect_at = 0.0     # 첫 검출 시각 (0이면 아직 검출 전)
cycle_last_detect_at = 0.0      # 마지막 검출 시각

WINDOW_NAME = 'Process C - Vision Inspection (Pendant Mode)'
TRACKBAR_NAME = 'Confidence (%)'


# ═══════════════════════════════════════════════════════════════════════
# DB 함수
# ═══════════════════════════════════════════════════════════════════════

def get_latest_active_info():
    """가장 최근 ACTIVE 트레이의 product_sn + tray_id 반환"""
    try:
        conn = pymysql.connect(**DB_CONFIG)
        with conn.cursor() as cur:
            cur.execute(
                "SELECT product_sn, tray_id FROM tbl_carrier_map "
                "WHERE status = 'ACTIVE' "
                "ORDER BY bound_at DESC LIMIT 1"
            )
            row = cur.fetchone()
        conn.close()
        return row if row else (None, None)
    except Exception as e:
        print(f"  ❌ 조회 에러: {e}")
        return (None, None)


def record_robot_c_result(product_sn, tray_sn, vision_result, defect_type=None):
    """비전 검사 결과 → tbl_robot_c INSERT"""
    try:
        conn = pymysql.connect(**DB_CONFIG)
        with conn.cursor() as cur:
            cur.execute(
                "INSERT INTO tbl_robot_c "
                "(product_sn, machine_name, recorded_at, tray_sn, vision_result, defect_type) "
                "VALUES (%s, %s, NOW(), %s, %s, %s)",
                (product_sn, 'Vision_PC_C', tray_sn, vision_result, defect_type)
            )
        conn.close()
        return True
    except Exception as e:
        print(f"  ❌ robot_c INSERT 에러: {e}")
        return False


def finalize_and_release(product_sn):
    """공정 C 종료 → tbl_total INSERT + carrier_map RELEASE

    1. 각 공정 결과(robot_a/b/c) 조회
    2. 최종 판정 결정 (하나라도 NG면 NG)
    3. tbl_total UPSERT
    4. carrier_map RELEASE
    """
    try:
        conn = pymysql.connect(**DB_CONFIG)
        with conn.cursor() as cur:
            # 1. 각 공정 결과 조회 (가장 최근)
            cur.execute(
                "SELECT vision_result FROM tbl_robot_a "
                "WHERE product_sn = %s ORDER BY recorded_at DESC LIMIT 1",
                (product_sn,)
            )
            row = cur.fetchone()
            result_a = row[0] if row else None

            cur.execute(
                "SELECT vision_result FROM tbl_robot_b "
                "WHERE product_sn = %s ORDER BY recorded_at DESC LIMIT 1",
                (product_sn,)
            )
            row = cur.fetchone()
            result_b = row[0] if row else None

            cur.execute(
                "SELECT vision_result FROM tbl_robot_c "
                "WHERE product_sn = %s ORDER BY recorded_at DESC LIMIT 1",
                (product_sn,)
            )
            row = cur.fetchone()
            result_c = row[0] if row else None

            # 2. 최종 판정
            results = [result_a, result_b, result_c]
            if any(r == 'NG' for r in results):
                final = 'NG'
            elif all(r == 'OK' for r in results):
                final = 'OK'
            else:
                final = None   # 일부 누락

            # 3. tbl_total INSERT (UPSERT)
            cur.execute(
                "INSERT INTO tbl_total "
                "(product_sn, process_a, process_b, process_c, final_result, completed_at) "
                "VALUES (%s, %s, %s, %s, %s, NOW()) "
                "ON DUPLICATE KEY UPDATE "
                "process_a=VALUES(process_a), process_b=VALUES(process_b), "
                "process_c=VALUES(process_c), final_result=VALUES(final_result), "
                "completed_at=VALUES(completed_at)",
                (product_sn, result_a, result_b, result_c, final)
            )

            # 4. carrier_map RELEASE
            cur.execute(
                "UPDATE tbl_carrier_map "
                "SET status='RELEASED', released_at=NOW() "
                "WHERE product_sn=%s AND status='ACTIVE'",
                (product_sn,)
            )
        conn.close()
        return final, result_a, result_b, result_c
    except Exception as e:
        print(f"  ❌ 종합 처리 에러: {e}")
        return None, None, None, None


def show_active_status():
    """현재 ACTIVE 트레이 조회"""
    try:
        conn = pymysql.connect(**DB_CONFIG)
        with conn.cursor() as cur:
            cur.execute(
                "SELECT tray_id, product_sn, bound_at "
                "FROM tbl_carrier_map "
                "WHERE status='ACTIVE' "
                "ORDER BY bound_at DESC"
            )
            rows = cur.fetchall()
        conn.close()
        print("\n  ┌─ 현재 ACTIVE 트레이 ─" + "─" * 30)
        if rows:
            for r in rows:
                print(f"  │ {r[0]} ← {r[1]}  (BIND: {r[2]})")
        else:
            print("  │ (모든 트레이 비어있음)")
        print("  └" + "─" * 50)
    except Exception as e:
        print(f"  ❌ 조회 에러: {e}")


# ═══════════════════════════════════════════════════════════════════════
# PLC 함수
# ═══════════════════════════════════════════════════════════════════════

def connect_plc():
    """공정 C PLC 120 연결 (트리거 수신/통보용)"""
    global plc
    try:
        plc = Type3E()
        plc.connect(PLC_IP, PLC_PORT)
        print(f"✅ PLC 연결 ({PLC_IP}:{PLC_PORT}) — 공정 C 메인")
        return True
    except Exception as e:
        print(f"⚠ PLC 연결 실패 — 시뮬 모드 ({e})")
        plc = None
        return False


def connect_plc_monitor():
    """관제 PLC 160 연결 (M1120 종료 신호 폴링용)"""
    global plc_monitor
    try:
        plc_monitor = Type3E()
        plc_monitor.connect(PLC_MONITOR_IP, PLC_PORT)
        print(f"✅ 관제 PLC 연결 ({PLC_MONITOR_IP}:{PLC_PORT}) — 종료 신호 폴링")
        return True
    except Exception as e:
        print(f"⚠ 관제 PLC 연결 실패 — 종료 폴링 비활성 ({e})")
        plc_monitor = None
        return False


def read_plc_bit(addr):
    """공정 C PLC(120) 비트 1개 읽기 — 트리거 비트용. plc_lock으로 BG 스레드/메인 write 직렬화."""
    if plc is None:
        return False
    with plc_lock:
        try:
            result = plc.batchread_bitunits(addr, 1)
            return bool(result[0])
        except Exception:
            return False


def read_done_bit():
    """공정 C 종료 비트 — 관제 PLC 160에서 읽기. plc_monitor_lock으로 직렬화."""
    if plc_monitor is None:
        return False
    with plc_monitor_lock:
        try:
            return bool(plc_monitor.batchread_bitunits(ADDR_DONE_C, 1)[0])
        except Exception:
            return False


def send_vision_result_to_plc(is_normal):
    """비전 결과 PLC 상태 통보 — 공정 A/B와 동일 패턴

    양품 → M250 펄스
    불량 → M260 펄스

    ※ 로봇 동작 기동은 PLC 자체 센서/래더 로직이 처리.
       Vision PC는 양품/불량 결과만 통보하면 됨.
    """
    target_addr = ADDR_NORMAL if is_normal else ADDR_CRACK
    status_text = f"양품({ADDR_NORMAL})" if is_normal else f"불량({ADDR_CRACK})"

    if plc is None:
        print(f"  [시뮬] PLC 없음 — {status_text} 가상 펄스 (0.5s)")
        sleep(0.5)
        return

    try:
        with plc_lock:
            plc.batchwrite_bitunits(target_addr, [1])
        print(f"  [PLC] {target_addr} ON ({status_text})")
        sleep(0.5)
        with plc_lock:
            plc.batchwrite_bitunits(target_addr, [0])
        print(f"  [PLC] {target_addr} OFF (초기화)")
    except Exception as e:
        print(f"  ❌ PLC 전송 실패: {e}")


def plc_polling_loop_120():
    """PLC 120 (B130 비전 트리거) 폴링 스레드 — 빠른 응답(~수ms), 짧은 주기로.

    공정 C 메인 PLC. B130 상승 엣지 감지 지연이 voting 시작 타이밍을 결정하므로
    M1120 폴링과 분리해서 독립적으로 빠르게 돌게 함.
    """
    while not plc_thread_stop.is_set():
        loop_start = time.time()
        t0 = time.time()
        vt = read_plc_bit(ADDR_VISION_TRIGGER)
        t1 = time.time()

        with plc_cache_lock:
            plc_cache['vision_trigger'] = vt
            plc_cache['last_update_at'] = t1

        with plc_poll_diag_lock:
            plc_poll_diag['trigger_total_ms'] += (t1 - t0) * 1000.0
            plc_poll_diag['trigger_count'] += 1

        elapsed = time.time() - loop_start
        remaining = PLC_POLL_TARGET_INTERVAL - elapsed
        if remaining > 0:
            plc_thread_stop.wait(remaining)


def plc_polling_loop_160():
    """관제 PLC 160 (M1120 종료 신호) 폴링 스레드 — 응답 느려도 무방.

    M1120은 long-lived 종료 신호라 폴링 주기 길어도 동작에 영향 없음.
    실측 결과 매 read 가 ~2s 타임아웃 — 별도 스레드로 분리해 PLC 120 폴링이 영향받지 않도록 함.
    """
    while not plc_thread_stop.is_set():
        loop_start = time.time()
        t0 = time.time()
        dc = read_done_bit()
        t1 = time.time()

        with plc_cache_lock:
            plc_cache['done_c'] = dc

        with plc_poll_diag_lock:
            plc_poll_diag['done_total_ms'] += (t1 - t0) * 1000.0
            plc_poll_diag['done_count'] += 1

        elapsed = time.time() - loop_start
        remaining = PLC_POLL_TARGET_INTERVAL - elapsed
        if remaining > 0:
            plc_thread_stop.wait(remaining)


def set_b130(val):
    """B130을 ON(1) 또는 OFF(0) 으로 직접 설정.

    공정 B PLC(130)가 보내는 시작 트리거를 흉내내는 수동 도구.
    val=1(ON) 후 monitor_plc_signals()의 상승 엣지 감지가 동작하여
    검출 모드에 진입함.

    재트리거하려면 val=0 으로 한 번 OFF 후 다시 val=1 (상승 엣지 필요).
    """
    if plc is None:
        print(f"  ⚠ PLC 미연결 — {ADDR_VISION_TRIGGER} 조작 불가")
        return
    try:
        with plc_lock:
            plc.batchwrite_bitunits(ADDR_VISION_TRIGGER, [val])
        state = "ON " if val else "OFF"
        print(f"  [TRIG] {ADDR_VISION_TRIGGER} {state}")
    except Exception as e:
        print(f"  ❌ {ADDR_VISION_TRIGGER} 조작 실패: {e}")


# ═══════════════════════════════════════════════════════════════════════
# YOLO 비전 검사 (vision_test_done 이식: 즉시 1프레임 판정 방식)
# ═══════════════════════════════════════════════════════════════════════

def load_yolo_model():
    """YOLO 모델 로드"""
    global yolo_model
    print(f"[INFO] 모델 로딩 중: {MODEL_PATH}")
    try:
        yolo_model = YOLO(MODEL_PATH)
        print(f"✅ YOLO 모델 로드 완료")
        print(f"  클래스: {yolo_model.names}")
        return True
    except Exception as e:
        print(f"⚠ YOLO 모델 로드 실패: {e}")
        yolo_model = None
        return False


def on_trackbar(_val):
    pass


def get_current_conf_threshold():
    """트랙바에서 현재 conf 임계값 (0.0~1.0)"""
    val = cv2.getTrackbarPos(TRACKBAR_NAME, WINDOW_NAME)
    return val / 100.0


def vision_inspect(frame, conf_thresh):
    """프레임 1장을 YOLO로 추론해서 판정 결과 반환.

    Returns:
        dict {
            'detected': bool,
            'class_name': str | None,
            'confidence': float,
            'is_normal': True/False/None,
            'annotated_frame': numpy array
        }
    """
    if yolo_model is None:
        return {'detected': False, 'class_name': None, 'confidence': 0.0,
                'is_normal': None, 'annotated_frame': frame}

    try:
        results = yolo_model(frame, conf=conf_thresh, verbose=False)
        result = results[0]
        annotated = result.plot()

        if len(result.boxes) == 0:
            return {'detected': False, 'class_name': None, 'confidence': 0.0,
                    'is_normal': None, 'annotated_frame': annotated}

        confidences = result.boxes.conf.cpu().numpy()
        class_ids = result.boxes.cls.cpu().numpy().astype(int)

        best_idx = confidences.argmax()
        best_conf = confidences[best_idx]
        best_class_name = yolo_model.names[class_ids[best_idx]]

        if best_class_name in NORMAL_CLASSES:
            is_normal = True
        elif best_class_name in CRACK_CLASSES:
            is_normal = False
        else:
            is_normal = None

        return {'detected': True, 'class_name': best_class_name,
                'confidence': float(best_conf), 'is_normal': is_normal,
                'annotated_frame': annotated}
    except Exception as e:
        print(f"  ❌ 비전 추론 에러: {e}")
        return {'detected': False, 'class_name': None, 'confidence': 0.0,
                'is_normal': None, 'annotated_frame': frame}


def commit_inspection_result(is_normal, class_name):
    """판정 결과 처리 공통 루틴 — PLC 양품/불량 통보 + tbl_robot_c INSERT."""
    judgment = "양품 ✅" if is_normal else "불량 ❌"
    print(f"  ⚡ [즉시 판정] 검출 클래스: {class_name} → {judgment}")

    # 1. PLC 양품/불량 통보 (로봇 기동은 PLC 자체 센서/래더가 처리)
    send_vision_result_to_plc(is_normal)

    # 2. tbl_robot_c INSERT
    product_sn, tray_sn = get_latest_active_info()
    if product_sn:
        vision_result = 'OK' if is_normal else 'NG'
        defect_type = None if is_normal else class_name
        if record_robot_c_result(product_sn, tray_sn, vision_result, defect_type):
            print(f"  ✅ tbl_robot_c INSERT: {product_sn} → {vision_result}")
    else:
        print(f"  ⚠ ACTIVE S/N 없음, DB 기록 스킵")


def execute_vision_inspection_manual(frame, conf_thresh):
    """V 키 — 수동 비전 검사 (PLC 우회, 디버그용)."""
    print(f"  🔍 [수동] 비전 검사 시작 (conf={conf_thresh:.2f})")
    result = vision_inspect(frame, conf_thresh)
    if not result['detected']:
        print(f"  ⚠ 객체 미검출")
        return result
    print(f"  검출: {result['class_name']} (신뢰도: {result['confidence']:.2f})")
    if result['is_normal'] is None:
        print(f"  ⚠ 알 수 없는 클래스: {result['class_name']}")
        return result
    commit_inspection_result(result['is_normal'], result['class_name'])
    return result


# ═══════════════════════════════════════════════════════════════════════
# PLC 신호 폴링 + 검출 단계 (vision_test_done 즉시 판정 방식)
# ═══════════════════════════════════════════════════════════════════════

def monitor_plc_signals():
    """B130(비전 트리거) + M1120(공정 C 종료) 폴링.

    B130 상승 엣지: 새 검사 사이클 시작 — trigger_on_time 갱신 + 잠금 해제
    B130 하강 엣지: 검출 모드 종료
    M1120 상승 엣지: tbl_total INSERT + carrier_map RELEASE
    """
    global prev_vision_trigger, prev_done_c
    global trigger_on_time, result_sent_this_cycle, last_view_frame
    global cycle_total_frames, cycle_detect_frames, cycle_class_counter
    global cycle_first_detect_at, cycle_last_detect_at
    global voting_started_at, voting_buffer

    if plc is None:
        return

    # BG 폴링 스레드가 갱신한 캐시에서 read (O(1), 메인 루프 FPS 영향 X)
    with plc_cache_lock:
        curr_vision_trigger = plc_cache['vision_trigger']
        curr_done_c = plc_cache['done_c']

    # ───── B130 상승 엣지 → 새 검사 사이클 시작 ─────
    if curr_vision_trigger and not prev_vision_trigger:
        trigger_on_time = time.time()
        result_sent_this_cycle = False
        # Voting 상태 리셋
        voting_started_at = 0.0
        voting_buffer = []
        # 사이클 누적 카운터 리셋
        cycle_total_frames = 0
        cycle_detect_frames = 0
        cycle_class_counter = Counter()
        cycle_first_detect_at = 0.0
        cycle_last_detect_at = 0.0
        print(f"\n[{datetime.now().strftime('%H:%M:%S')}] 🟢 {ADDR_VISION_TRIGGER} ON — 안정화 대기 ({IGNORE_DURATION}s)...")

    # ───── B130 하강 엣지 → 검출 모드 종료 + 사이클 요약 출력 ─────
    if not curr_vision_trigger and prev_vision_trigger:
        print(f"\n[{datetime.now().strftime('%H:%M:%S')}] 🔴 {ADDR_VISION_TRIGGER} OFF — 추론 대기모드")
        if voting_started_at > 0.0 and not result_sent_this_cycle:
            print(f"  ⚠ voting 미완료 상태로 OFF — 누적 {len(voting_buffer)}프레임 폐기")
        voting_started_at = 0.0
        voting_buffer = []
        if cycle_total_frames > 0:
            miss_frames = cycle_total_frames - cycle_detect_frames
            detect_ratio = cycle_detect_frames / cycle_total_frames * 100
            print(f"  ┌─ 📊 사이클 요약 (voting 윈도우 결정용)")
            print(f"  │ 총 추론 프레임 : {cycle_total_frames} (안정화 후)")
            print(f"  │ 검출 프레임    : {cycle_detect_frames} ({detect_ratio:.1f}%)")
            print(f"  │ 미검출 프레임  : {miss_frames}")
            if cycle_detect_frames > 0:
                in_view_dur = cycle_last_detect_at - cycle_first_detect_at
                det_fps = cycle_detect_frames / in_view_dur if in_view_dur > 0 else 0
                dist = ', '.join(f"{cn}={n}" for cn, n in cycle_class_counter.most_common())
                print(f"  │ 트레이 시야 체류: {in_view_dur:.2f}s (검출 구간 FPS≈{det_fps:.1f})")
                print(f"  │ 클래스 분포    : {dist}")
            print(f"  └" + "─" * 50)
        last_view_frame = None

    # ───── M1120 상승 엣지 (PLC 160) → 공정 C 종료 처리 ─────
    if curr_done_c and not prev_done_c:
        print(f"\n[{datetime.now().strftime('%H:%M:%S')}] 🟢 공정 C 종료 ({ADDR_DONE_C} ON, PLC 160)")
        product_sn, tray_sn = get_latest_active_info()
        if product_sn:
            final, result_a, result_b, result_c = finalize_and_release(product_sn)
            print(f"  ✅ tbl_total INSERT + carrier_map RELEASE")
            print(f"  📋 종합 결과:")
            print(f"     {product_sn} (Tray: {tray_sn})")
            print(f"     공정 A: {result_a or '미실시'}")
            print(f"     공정 B: {result_b or '미실시'}")
            print(f"     공정 C: {result_c or '미실시'}")
            print(f"     최종 판정: {final or '판정 불가'}")
        else:
            print(f"  ⚠ ACTIVE S/N 없음")

    prev_vision_trigger = curr_vision_trigger
    prev_done_c = curr_done_c


def vision_step(frame, conf_thresh):
    """매 프레임 호출 — B130 ON 동안 안정화 후 Voting 누적 판정.

    동작 흐름:
      1) B130 OFF: 원본 프레임 + Standby 텍스트
      2) B130 ON: IGNORE_DURATION 동안 안정화 표시 (추론 X)
      3) 안정화 시간 경과 후 매 프레임 추론
      4) 첫 검출 발생 → voting 시작 (VOTING_DURATION s 카운트)
      5) voting 윈도우 동안 매 검출 결과를 voting_buffer 에 누적
      6) VOTING_DURATION 경과 → 다수결로 최종 클래스 결정 → PLC + DB (사이클당 1회)
      7) result_sent_this_cycle=True 잠금 → B130 OFF→ON 시 해제
    """
    global last_view_frame, result_sent_this_cycle, last_diag_at
    global cycle_total_frames, cycle_detect_frames, cycle_class_counter
    global cycle_first_detect_at, cycle_last_detect_at
    global voting_started_at, voting_buffer

    # B130 OFF 상태 — 대기 화면
    if not prev_vision_trigger:
        view = frame.copy()
        cv2.putText(view, f"Standby [Thresh: {conf_thresh:.2f}]", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        last_view_frame = view
        return

    elapsed = time.time() - trigger_on_time

    # 안정화 대기 구간 — 추론 건너뛰고 표시만
    if elapsed < IGNORE_DURATION:
        view = frame.copy()
        cv2.putText(view, f"Filtering Initial Data... ({elapsed:.2f}s)", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        last_view_frame = view
        return

    # 추론 + 즉시 판정
    result = vision_inspect(frame, conf_thresh)
    view = result['annotated_frame']
    cv2.putText(view, f"Thresh: {conf_thresh:.2f} | SCANNING...", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    last_view_frame = view

    # 사이클 누적 카운터 갱신
    now = time.time()
    cycle_total_frames += 1
    if result['detected'] and result.get('class_name'):
        cycle_detect_frames += 1
        cycle_class_counter[result['class_name']] += 1
        if cycle_first_detect_at == 0.0:
            cycle_first_detect_at = now
        cycle_last_detect_at = now

    # 진단 로그 — 1초마다 1번 (현재 사이클 누적 + voting 진행 상태)
    if now - last_diag_at >= 1.0:
        cls_str = result.get('class_name') or '(객체 없음)'
        status = "✓ 검출" if result['detected'] else "× 미검출"
        voting_info = ""
        if voting_started_at > 0.0 and not result_sent_this_cycle:
            v_elapsed = now - voting_started_at
            voting_info = f"  | voting {v_elapsed:.1f}/{VOTING_DURATION}s ({len(voting_buffer)}프레임)"
        print(f"  [추론] {status}: class={cls_str}, conf={result.get('confidence', 0.0):.2f}"
              f"  | 누적: {cycle_detect_frames}/{cycle_total_frames}{voting_info}")
        last_diag_at = now

    # ───── Voting 누적 ─────
    if not result_sent_this_cycle:
        if result['detected'] and result['is_normal'] is not None:
            if voting_started_at == 0.0:
                voting_started_at = now
                voting_buffer = []
                print(f"  🔄 첫 검출 — {VOTING_DURATION}s voting 시작 "
                      f"(class={result['class_name']}, conf={result['confidence']:.2f})")
            voting_buffer.append((result['class_name'], result['confidence']))

        # ───── Voting 종료 판정 ─────
        if voting_started_at > 0.0 and (now - voting_started_at) >= VOTING_DURATION:
            total = len(voting_buffer)
            if total == 0:
                # 검출 직후 즉시 트레이가 사라진 극단 케이스 — 다음 검출 대기
                voting_started_at = 0.0
            else:
                counter = Counter(cn for cn, _ in voting_buffer)
                best_class, best_count = counter.most_common(1)[0]
                ratio = best_count / total * 100
                avg_conf = sum(c for cn, c in voting_buffer if cn == best_class) / best_count
                dist = ', '.join(f"{cn}={n}" for cn, n in counter.most_common())
                print(f"  📊 voting 결과 ({total}프레임 / {VOTING_DURATION}s): {dist}")
                print(f"     → 채택: {best_class} ({best_count}/{total}={ratio:.0f}%, 평균 conf={avg_conf:.2f})")

                if best_class in NORMAL_CLASSES:
                    is_normal = True
                elif best_class in CRACK_CLASSES:
                    is_normal = False
                else:
                    is_normal = None
                    print(f"  ⚠ 알 수 없는 클래스: {best_class} — PLC/DB 송신 스킵")

                if is_normal is not None:
                    commit_inspection_result(is_normal, best_class)
                result_sent_this_cycle = True


# ═══════════════════════════════════════════════════════════════════════
# 카메라 — Intel RealSense D435 (vision_test_done 이식)
# ═══════════════════════════════════════════════════════════════════════

def find_and_start_realsense(width=CAMERA_WIDTH, height=CAMERA_HEIGHT, fps=CAMERA_FPS):
    print("[INFO] 리얼센스 카메라 장치를 검색합니다...")
    ctx = rs.context()
    devices = ctx.query_devices()

    if len(devices) == 0:
        print("[ERROR] 연결된 리얼센스 카메라를 찾을 수 없습니다.")
        return None

    target_serial = None
    for dev in devices:
        name = dev.get_info(rs.camera_info.name)
        serial = dev.get_info(rs.camera_info.serial_number)
        if "D435" in name:
            target_serial = serial
            print(f"[SUCCESS] D435 발견! (S/N: {target_serial})")
            break

    pipeline = rs.pipeline()
    config = rs.config()
    if target_serial:
        config.enable_device(target_serial)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

    try:
        pipeline.start(config)
        print(f"[INFO] 카메라 파이프라인 시작 (해상도: {width}x{height})\n")
        return pipeline
    except Exception as e:
        print(f"[ERROR] 카메라 시작 실패: {e}")
        return None


# ═══════════════════════════════════════════════════════════════════════
# 초기화
# ═══════════════════════════════════════════════════════════════════════

# cv2 창 + 트랙바 — 트랙바는 namedWindow가 먼저 있어야 생성 가능
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WINDOW_NAME, 960, 720)
cv2.createTrackbar(TRACKBAR_NAME, WINDOW_NAME,
                   int(INITIAL_CONF_THRESHOLD * 100), 100, on_trackbar)

load_yolo_model()
connect_plc()
connect_plc_monitor()

# PLC 백그라운드 폴링 스레드 시작 — PLC 별로 분리해 한쪽이 느려도 다른쪽 영향 X
plc_poll_thread_120 = threading.Thread(target=plc_polling_loop_120, name='plc-poll-120', daemon=True)
plc_poll_thread_160 = threading.Thread(target=plc_polling_loop_160, name='plc-poll-160', daemon=True)
plc_poll_thread_120.start()
plc_poll_thread_160.start()
print(f"✅ PLC 백그라운드 폴링 스레드 시작 (PLC 120 / PLC 160 각 별도, 목표 {1.0/PLC_POLL_TARGET_INTERVAL:.0f}Hz)")

cam_pipeline = find_and_start_realsense()
if cam_pipeline is None:
    print("\n[FATAL] 카메라 초기화 실패 — 종료합니다.")
    raise SystemExit(1)


# ═══════════════════════════════════════════════════════════════════════
# 시작 안내
# ═══════════════════════════════════════════════════════════════════════

print("\n" + "=" * 60)
print("  공정 C — 비전 검사 + 종합 처리 (펜던트 IF 방식)")
print("=" * 60)
print("  [자동 동작 — Voting 다수결 판정 방식]")
print(f"    {ADDR_VISION_TRIGGER} 상승 엣지 → 안정화 {IGNORE_DURATION}s 대기 → 매 프레임 추론")
print(f"      └─ 첫 검출 발생 → {VOTING_DURATION}s 동안 결과 누적")
print(f"      └─ 다수결로 클래스 채택 → 양품/불량 판정 → PLC + DB")
print(f"      └─ 사이클당 1회 송신 (재트리거: {ADDR_VISION_TRIGGER} OFF→ON)")
print(f"    {ADDR_DONE_C} 감지 → tbl_total + carrier_map RELEASE")
print("")
print("  [수동 제어]")
print("    V  : 비전 검사 수동 실행 (PLC 우회, 디버그)")
print(f"    1  : {ADDR_VISION_TRIGGER} ON  (PLC 120 시작 트리거 흉내 — 공정 B 우회)")
print(f"    0  : {ADDR_VISION_TRIGGER} OFF (재트리거 준비)")
print("    S  : 현재 ACTIVE 조회")
print("    Q  : 종료")
print("    ※ 화면 상단 트랙바로 conf 임계값 실시간 조정 가능")
print("=" * 60)
print("")
print(f"  [DB 서버] {DB_CONFIG['host']}:{DB_CONFIG['port']}")
print(f"  [PLC]     {PLC_IP}:{PLC_PORT} (공정 C 메인)")
print(f"  [PLC 관제] {PLC_MONITOR_IP}:{PLC_PORT} (종료 신호 폴링)")
print(f"  [카메라]  Intel RealSense D435 ({CAMERA_WIDTH}x{CAMERA_HEIGHT} @ {CAMERA_FPS}fps)")
print(f"  [로봇 C]  192.168.3.5 (PLC 자체 제어 — 펜던트 IF)")
print("")
print("  [PLC 비트]")
print(f"    {ADDR_VISION_TRIGGER}     : 비전 트리거 (PLC 120, 공정 B ← B디바이스)")
print(f"    {ADDR_NORMAL}      : 양품 통보 (Vision → PLC 120)")
print(f"    {ADDR_CRACK}      : 불량 통보 (Vision → PLC 120)")
print(f"    {ADDR_DONE_C}  : 공정 C 종료 (PLC 160 → Vision)")
print("")
print("  [로봇 제어 방식]")
print("    CC-Link → PLC 자체 센서/래더 → 펜던트 IF (공정 A/B와 동일)")
print("    Vision PC는 양품/불량 통보만, 로봇 기동은 PLC가 자체 처리")
print("    ※ Busan_Robot HMI 통합은 시연 후 확장 작업 예정")
print("")
print("  [비전 클래스]")
print(f"    양품: {NORMAL_CLASSES}")
print(f"    불량: {CRACK_CLASSES}")
print(f"    초기 conf 임계값: {INITIAL_CONF_THRESHOLD}  |  안정화: {IGNORE_DURATION}s  |  Voting: {VOTING_DURATION}s")
print("=" * 60)

show_active_status()
print("")


# ═══════════════════════════════════════════════════════════════════════
# 메인 루프
# ═══════════════════════════════════════════════════════════════════════

try:
    while True:
        # ── [구간1] 카메라 캡처 ──
        _t0 = time.time()
        frames = cam_pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        frame = np.asanyarray(color_frame.get_data())
        _t1 = time.time()
        section_times['capture'] += _t1 - _t0
        section_counts['capture'] += 1

        # 트랙바에서 현재 conf 임계값
        conf_thresh = get_current_conf_threshold()

        # ── [구간2] PLC 신호 폴링 (캐시 read만 — 실제 PLC IO는 BG 스레드가 처리) ──
        monitor_plc_signals()
        _t2 = time.time()
        section_times['plc'] += _t2 - _t1
        section_counts['plc'] += 1

        # ── [구간3] 비전 검출 ──
        vision_step(frame, conf_thresh)
        _t3 = time.time()
        section_times['vision'] += _t3 - _t2
        section_counts['vision'] += 1

        # FPS 측정 — 0.5초 윈도우 평균
        fps_frame_count += 1
        _fps_now = time.time()
        if _fps_now - fps_last_time >= 0.5:
            current_fps = fps_frame_count / (_fps_now - fps_last_time)
            fps_frame_count = 0
            fps_last_time = _fps_now

        # ── [구간4] 화면 표시 — FPS 오버레이 (우상단) ──
        display_frame = last_view_frame if last_view_frame is not None else frame
        _w = display_frame.shape[1]
        cv2.putText(display_frame, f"FPS: {current_fps:.1f}",
                    (_w - 180, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow(WINDOW_NAME, display_frame)
        _t4 = time.time()
        section_times['display'] += _t4 - _t3
        section_counts['display'] += 1

        # 구간별 평균 시간 출력 (1초마다 1번) + BG PLC 폴링 진단
        if _t4 - last_timing_print >= 1.0:
            parts = []
            for k in ('capture', 'plc', 'vision', 'display'):
                cnt = max(1, section_counts[k])
                avg_ms = 1000.0 * section_times[k] / cnt
                parts.append(f"{k}={avg_ms:.1f}ms")

            with plc_poll_diag_lock:
                bg_t_count = plc_poll_diag['trigger_count']
                bg_d_count = plc_poll_diag['done_count']
                bg_t_avg = plc_poll_diag['trigger_total_ms'] / max(1, bg_t_count)
                bg_d_avg = plc_poll_diag['done_total_ms'] / max(1, bg_d_count)
                plc_poll_diag['trigger_total_ms'] = 0.0
                plc_poll_diag['trigger_count'] = 0
                plc_poll_diag['done_total_ms'] = 0.0
                plc_poll_diag['done_count'] = 0

            print(f"  [timing] " + " | ".join(parts) + f"  (프레임 {section_counts['capture']}장)")
            print(f"  [plc-bg] B130(120)={bg_t_avg:.1f}ms × {bg_t_count}/s"
                  f" | M1120(160)={bg_d_avg:.1f}ms × {bg_d_count}/s")

            for k in section_times:
                section_times[k] = 0.0
                section_counts[k] = 0
            last_timing_print = _t4

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('v'):
            print(f"\n[{datetime.now().strftime('%H:%M:%S')}] 🔍 [수동] 비전 검사")
            result = execute_vision_inspection_manual(frame, conf_thresh)
            if result.get('annotated_frame') is not None:
                cv2.imshow('Vision Result', result['annotated_frame'])
                cv2.waitKey(3000)
                cv2.destroyWindow('Vision Result')
        elif key == ord('1'):
            print(f"\n[{datetime.now().strftime('%H:%M:%S')}] 🟢 [수동] {ADDR_VISION_TRIGGER} ON")
            set_b130(1)
        elif key == ord('0'):
            print(f"\n[{datetime.now().strftime('%H:%M:%S')}] 🔴 [수동] {ADDR_VISION_TRIGGER} OFF")
            set_b130(0)
        elif key == ord('s'):
            show_active_status()

finally:
    # ═══════════════════════════════════════════════════════════════════════
    # 종료
    # ═══════════════════════════════════════════════════════════════════════
    print("\n[INFO] 프로그램을 안전하게 종료합니다.")
    # BG 폴링 스레드 정리 (PLC 소켓 종료 전에)
    plc_thread_stop.set()
    for _t in (plc_poll_thread_120, plc_poll_thread_160):
        try:
            _t.join(timeout=3.0)
        except Exception:
            pass
    print("  PLC 폴링 스레드 종료 (120 / 160)")
    try:
        cam_pipeline.stop()
    except Exception:
        pass
    cv2.destroyAllWindows()

    if plc:
        try:
            plc.close()
            print("  PLC (120) 연결 종료")
        except Exception:
            pass

    if plc_monitor:
        try:
            plc_monitor.close()
            print("  관제 PLC (160) 연결 종료")
        except Exception:
            pass

    print("\n" + "=" * 60)
    print("  공정 C 종료")
    print("=" * 60)
