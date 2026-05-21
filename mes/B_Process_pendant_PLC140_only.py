"""
═══════════════════════════════════════════════════════════════════════
  FAiCTORY MES — 공정 B (비전 검사 + 중간 공정)
  ※ 시연용 MVP 버전 — 공정 A/C와 동일한 CC-Link + 펜던트 IF 패턴
═══════════════════════════════════════════════════════════════════════

  [이 PC의 역할]
    - B150 트리거 감지 → 비전 검사 자동 실행 (YOLO Voting 다수결)
    - 양품/불량 판정 → 공정 B PLC로 결과 통보 (M250/M260)
    - M101 ON 후 DEFECT_TIMEOUT 안에 양품 미판정 → 불량 처리 (M260)
    - 비전 결과는 메모리에 보관, B1790(공정 A 종료) 시 B_Process INSERT
    - 불량 시 finalize_defect_at_b 로 tbl_total + carrier_map RELEASE 직접 처리
    - 양품 시 공정 C 가 최종 종합 처리

  [로봇 제어 방식 — 공정 A/C와 동일]
    Vision PC → PLC(140)에 양품/불량 통보
       ↓
    PLC 자체 센서/래더 로직 → 로봇 트리거
       ↓
    펜던트 IF로 로봇 B 자체 동작

    (CC-Link 기반, 검증된 안정적 패턴 — 시연 안정성 최우선)
    Vision PC는 로봇과 직접 통신하지 않음. PLC가 모든 로봇 기동을 처리.

  [연결 구성]
    DB     : 192.168.3.141 (운영 서버, guest 계정)
    PLC    : 192.168.3.140 (공정 B 메인 — 모든 read/write 단일 접속)
             ※ B150(트리거) + M101(부품 도착) + B1790(공정 A 종료) 모두 PLC 140 에서 폴링
             ※ 공정 B에는 PLC 130(로봇/컨베어/종료)도 있으나 PLC끼리 통신,
               Vision PC는 메인 PLC 140만 접속
    카메라 : 비전 검사용 (인덱스 0, cv2.VideoCapture)
    로봇 B : PLC가 자체 제어 (Vision PC 직접 접속 X)
    모델   : best.pt (프로젝트 루트, 차체조립 검사 모델)

  [B 디바이스 통신 - PLC 간 신호 전달]
    공정 A PLC(150)의 B140(="140으로 보낸다")
      ⟷ 공정 B PLC(140)의 B150(="150에서 받았다")
    공정 B PLC(140)의 B120(="120으로 보낸다")
      ⟷ 공정 C PLC(120)의 B140 (공정 B→C 트리거, 이 코드와 무관)

  [전체 시스템 구조]
    공정 A: PLC 150 (작업 + 컨베어) — 펜던트 IF 자체 동작
    공정 B: PLC 140 (메인) + PLC 130 (로봇/컨베어/종료) — 펜던트 IF 자체 동작 ★
    공정 C: PLC 120 (메인) + PLC 110 (컨베어) — 펜던트 IF 자체 동작
    관제:   PLC 160 (SCADA용, Vision PC 직접 접속 X)

  [비전 판정 방식 — Voting 다수결 (공정 C 와 동일) + M101 타임아웃 안전망]
    B150 ON → IGNORE_DURATION(0.4s) 안정화 대기 → 매 프레임 추론
    첫 검출 시점부터 VOTING_DURATION(1.5s) 동안 결과 누적
    다수결로 최종 클래스 채택 → 양품 판정 → M250 즉시 통보 + 메모리 저장
    M101 ON 후 DEFECT_TIMEOUT(3.0s) 안에 양품 미판정 → 불량 → M260 통보
    B1790 ON → 메모리 결과로 B_Process INSERT (+ NG면 finalize_defect_at_b)
    B150 OFF → ON 다시 들어오면 잠금 해제 → 재트리거 가능

  [B 와 C 의 차이 — 같은 비전 로직, 다른 DB 타이밍]
    공정 C : voting 완료 즉시 PLC + C_Process INSERT (사이클 단발성)
    공정 B : voting 완료 즉시 PLC, DB INSERT 는 B1790(공정 A 종료) 시점에 수행
             (PLC 작업이 끝난 뒤 결과 확정 — A_Process 와 동일한 패턴)
═══════════════════════════════════════════════════════════════════════
"""

import os
import time
import threading
import cv2
import numpy as np
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
PLC_IP   = "192.168.3.140"   # 공정 B 메인 PLC (모든 read/write 단일 접속)
PLC_PORT = 2000

# ───── PLC 비트 정의 ─────
# 읽기 (PLC 140 → Vision B) — 단일 PLC 에서 3비트 모두 폴링
ADDR_VISION_TRIGGER = "B150"    # 비전 검사 시작 트리거
                                # 공정 A PLC(150)의 B140 ⟷ 공정 B PLC(140)의 B150
ADDR_PART_PRESENT   = "M101"    # 부품 도착 신호 (여자 시 ON) — 비전 검출 기대 시작점.
                                # ON 후 DEFECT_TIMEOUT 안에 양품 판정 없으면 불량(M260) 처리.
ADDR_DONE_A         = "B1130"   # 공정 A 종료 신호 (PLC 140 의 B-디바이스로 들어옴)

# 쓰기 (Vision B → PLC 140)
ADDR_NORMAL    = "M250"     # 양품 신호 (PLC 래더가 인식 → 자체 분기 처리)
ADDR_CRACK     = "M260"     # 불량 신호 (PLC 래더가 인식 → 자체 분기 처리)
# ※ 로봇 동작 기동은 PLC 자체 센서 입력 + 래더가 처리.
#    Vision PC는 양품/불량만 통보.

# ───── YOLO 모델 ─────
# B 전용 모델 (프로젝트 루트의 best.pt — 차체조립 검사 모델)
# 모델 출력 클래스: {0: 'b', 1: 'g', 2: 'r'} — 차체 부품 색상 3종
# 차체조립 공정은 불량 클래스가 없으므로 검출된 b/g/r 전부 양품으로 매핑.
MODEL_PATH = os.path.join(os.path.dirname(__file__), '..', '..', 'best.pt')
# 초기 conf 임계값 — 실행 중 트랙바로 실시간 조정 가능
INITIAL_CONF_THRESHOLD = 0.50
NORMAL_CLASSES = ['b', 'g', 'r']   # 차체조립 — 세 부품 모두 정상 (양품 매핑)
CRACK_CLASSES  = []                # 차체조립에는 불량 클래스 없음 (M260 경로 미사용)

# B150 ON 직후 센서/조명 흔들림으로 인한 초기 쓰레기 데이터 무시 시간 (초)
# 이 시간 경과 후부터 매 프레임 추론을 시작.
IGNORE_DURATION = 0.4

# Voting 윈도우 — 첫 검출 시점부터 N초 동안 결과를 누적해 다수결로 최종 판정.
# C 공정 측정 결과: 트레이 시야 체류 ~2.7s, 검출 구간 ~24fps → 1.5s × 24fps ≈ 36프레임
# B 카메라 환경에서 부족하면 늘려 조정 (디버깅 md 의 "3초 voting" 참고).
VOTING_DURATION = 1.5

# M101 ON 후 양품 판정 없이 경과하면 불량으로 판정하는 타임아웃 (초)
# 차체조립에서 부품은 도착했는데(M101 ON) 모델이 b/g/r 어느 것도 검출 못한
# 케이스를 불량으로 분류 — voting 안전망. 만료 시 M260 펄스 + B_Process NG INSERT.
DEFECT_TIMEOUT    = 3.0
DEFECT_CLASS_NAME = 'no_detect'   # 타임아웃 시 defect_type 컬럼에 들어갈 값

# ───── 카메라 설정 ─────
CAMERA_INDEX = 0

# 카메라 회전 보정 (카메라가 가로/세로로 마운트된 경우)
# None : 회전 없음
# cv2.ROTATE_90_CLOCKWISE         : 시계방향 90°
# cv2.ROTATE_90_COUNTERCLOCKWISE  : 반시계방향 90°
# cv2.ROTATE_180                  : 180°
CAMERA_ROTATION = cv2.ROTATE_90_COUNTERCLOCKWISE

plc = None              # 공정 B PLC 140 (모든 read/write 단일 접속)
yolo_model = None

# 폴링용 이전 상태 (상승 엣지 감지)
prev_vision_trigger = False
prev_done_a = False
prev_part_present = False

# M101 ON 타임스탬프 — 0.0 이면 OFF 상태 또는 아직 ON 안 됨.
# B150 OFF→ON 또는 M101 OFF→ON 에서 갱신, 양품 판정/M101 OFF/B150 OFF 에서 리셋.
part_present_on_time = 0.0

# 검출 상태 — Voting 판정 모드
# B150 상승 엣지 → trigger_on_time 갱신 + voting/잠금 상태 초기화
# IGNORE_DURATION 경과 후 매 프레임 추론 시도
# 첫 검출 발생 → voting_started_at 기록 + voting_buffer 에 결과 누적 시작
# VOTING_DURATION 경과 → 다수결로 최종 클래스 결정 → PLC 통보 + 메모리 저장
# B150 하강 후 다시 ON 시 새 사이클 시작
trigger_on_time = 0.0
result_sent_this_cycle = False
last_view_frame = None        # cv2 창에 표시할 최근 어노테이션 프레임
last_diag_at = 0.0            # 진단 로그 직전 출력 시각 (1초마다 1번)

# Voting 상태
voting_started_at = 0.0       # 첫 검출 시각 (0이면 아직 검출 전)
voting_buffer = []            # [(class_name, confidence), ...]

# 최근 비전 검사 결과 (M1130 종료 시 B_Process INSERT 용)
# 예: {'vision_result': 'OK'/'NG', 'defect_type': str/None}
last_vision_result = None

# FPS 측정 — 0.5초마다 갱신
fps_frame_count = 0
fps_last_time = time.time()
current_fps = 0.0

# PLC 백그라운드 폴링 — PLC 네트워크 지연이 메인 루프 FPS 를 죽이는 문제 해결.
# 별도 스레드가 PLC 를 지속적으로 read → 캐시 갱신, 메인 루프는 캐시만 read (O(1)).
plc_lock = threading.Lock()           # plc (140) 소켓 직렬화 (BG read + 메인 write 충돌 방지)
plc_cache = {
    'vision_trigger': False,   # B150 — BG 스레드(PLC 140)가 갱신
    'part_present': False,     # M101 — BG 스레드(PLC 140)가 갱신
    'done_a': False,           # B1790 공정 A 종료 — BG 스레드(PLC 140)가 갱신
    'last_update_at': 0.0,
}
plc_cache_lock = threading.Lock()
plc_thread_stop = threading.Event()
PLC_POLL_TARGET_INTERVAL = 0.05       # BG 스레드 목표 폴링 주기 (20Hz). PLC가 느리면 자연 throttle.

# BG 스레드 진단용 — 직전 1초 동안 PLC 140 read 평균 시간 + 폴링 횟수
# 한 사이클당 B150 + M101 + B1790 3비트 read 총 시간을 누적.
plc_poll_diag = {
    'trigger_total_ms': 0.0,
    'trigger_count': 0,
}
plc_poll_diag_lock = threading.Lock()

# 메인 루프 구간별 타이밍 진단 — 어디가 병목인지 1초마다 한 줄로 출력.
# capture: cv2 카메라 read + 회전 보정
# plc:     monitor_plc_signals (캐시 read만, O(1))
# vision:  vision_step (YOLO 추론 + voting)
# display: cv2.imshow + waitKey
section_times = {'capture': 0.0, 'plc': 0.0, 'vision': 0.0, 'display': 0.0}
section_counts = {'capture': 0, 'plc': 0, 'vision': 0, 'display': 0}
last_timing_print = time.time()

# 사이클 누적 카운터 — B150 ON→OFF 한 사이클 동안 통계 수집
cycle_total_frames = 0          # 안정화 이후 추론된 총 프레임
cycle_detect_frames = 0         # 그 중 검출 성공 프레임 수
cycle_class_counter = Counter() # 검출된 클래스 분포
cycle_first_detect_at = 0.0     # 첫 검출 시각 (0이면 아직 검출 전)
cycle_last_detect_at = 0.0      # 마지막 검출 시각

WINDOW_NAME = 'Process B - Vision Inspection (Pendant Mode)'
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


def record_robot_b_result(product_sn, tray_sn, vision_result, defect_type=None):
    """비전 검사 결과 → B_Process INSERT"""
    try:
        conn = pymysql.connect(**DB_CONFIG)
        with conn.cursor() as cur:
            cur.execute(
                "INSERT INTO B_Process "
                "(product_sn, machine_name, recorded_at, tray_sn, vision_result, defect_type) "
                "VALUES (%s, %s, NOW(), %s, %s, %s)",
                (product_sn, 'Vision_PC_B', tray_sn, vision_result, defect_type)
            )
        conn.close()
        return True
    except Exception as e:
        print(f"  ❌ robot_b INSERT 에러: {e}")
        return False


def finalize_defect_at_b(product_sn):
    """공정 B 불량 판정 시 → tbl_total UPSERT(NG) + carrier_map RELEASE.

    공정 C 로 안 넘어가는 시나리오에서 트레이가 영원히 ACTIVE 로 묶이지 않게
    B 단계에서 자체적으로 종합 처리 마무리.

    1. A_Process (이전 공정 결과) 조회
    2. B_Process 는 방금 INSERT 한 NG (자체 조회)
    3. process_c 는 NULL (공정 C 안 거침)
    4. final = NG (B 가 불량이므로 자동 NG)
    5. tbl_total UPSERT + carrier_map RELEASE
    """
    try:
        conn = pymysql.connect(**DB_CONFIG)
        with conn.cursor() as cur:
            cur.execute(
                "SELECT vision_result FROM A_Process "
                "WHERE product_sn = %s ORDER BY recorded_at DESC LIMIT 1",
                (product_sn,)
            )
            row = cur.fetchone()
            result_a = row[0] if row else None

            cur.execute(
                "SELECT vision_result FROM B_Process "
                "WHERE product_sn = %s ORDER BY recorded_at DESC LIMIT 1",
                (product_sn,)
            )
            row = cur.fetchone()
            result_b = row[0] if row else 'NG'

            result_c = None   # 공정 C 안 거침
            final = 'NG'      # B 가 불량이라 자동 NG

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

            cur.execute(
                "UPDATE tbl_carrier_map "
                "SET status='RELEASED', released_at=NOW() "
                "WHERE product_sn=%s AND status='ACTIVE'",
                (product_sn,)
            )
        conn.close()
        return final, result_a, result_b
    except Exception as e:
        print(f"  ❌ B 불량 종합 처리 에러: {e}")
        return None, None, None


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
    """공정 B PLC 140 연결 (트리거 수신/통보용)"""
    global plc
    try:
        plc = Type3E()
        plc.connect(PLC_IP, PLC_PORT)
        print(f"✅ PLC 연결 ({PLC_IP}:{PLC_PORT}) — 공정 B 메인")
        return True
    except Exception as e:
        print(f"⚠ PLC 연결 실패 — 시뮬 모드 ({e})")
        plc = None
        return False


def read_plc_bit(addr):
    """공정 B PLC(140) 비트 1개 읽기. plc_lock 으로 BG/메인 write 직렬화."""
    if plc is None:
        return False
    with plc_lock:
        try:
            result = plc.batchread_bitunits(addr, 1)
            return bool(result[0])
        except Exception:
            return False


def send_vision_result_to_plc(is_normal):
    """비전 결과 PLC 상태 통보 — 공정 A/C 와 동일 패턴

    양품 → M250 펄스
    불량 → M260 펄스

    ※ 로봇 동작 기동은 PLC 자체 센서/래더 로직이 처리.
       Vision PC 는 양품/불량 결과만 통보하면 됨.
    """
    target_addr = ADDR_NORMAL if is_normal else ADDR_CRACK
    status_text = f"양품({ADDR_NORMAL})" if is_normal else f"불량({ADDR_CRACK})"

    if plc is None:
        print(f"  [시뮬] PLC 없음 — {status_text} 가상 펄스 (1.0s)")
        sleep(1.0)
        return

    try:
        with plc_lock:
            plc.batchwrite_bitunits(target_addr, [1])
        print(f"  [PLC] {target_addr} ON ({status_text})")
        sleep(1.0)
        with plc_lock:
            plc.batchwrite_bitunits(target_addr, [0])
        print(f"  [PLC] {target_addr} OFF (초기화)")
    except Exception as e:
        print(f"  ❌ PLC 전송 실패: {e}")


def plc_polling_loop_140():
    """PLC 140 폴링 스레드 — 3비트(B150 + M101 + B1790) 통합 폴링.

    공정 B 메인 PLC 단일 접속으로 다음 비트를 모두 읽음:
      B150 : 비전 트리거 (voting 시작 타이밍 결정)
      M101 : 부품 도착 (불량 타임아웃 시작점)
      B1790: 공정 A 종료 신호 (B_Process INSERT 트리거)

    셋 다 다른 디바이스 영역(B/M)이라 batch read 불가 → 3회 순차 read.
    측정 시 1회 read 가 너무 느리면 PLC_POLL_TARGET_INTERVAL 조정.
    """
    while not plc_thread_stop.is_set():
        loop_start = time.time()
        t0 = time.time()
        vt = read_plc_bit(ADDR_VISION_TRIGGER)
        pp = read_plc_bit(ADDR_PART_PRESENT)
        da = read_plc_bit(ADDR_DONE_A)
        t1 = time.time()

        with plc_cache_lock:
            plc_cache['vision_trigger'] = vt
            plc_cache['part_present'] = pp
            plc_cache['done_a'] = da
            plc_cache['last_update_at'] = t1

        with plc_poll_diag_lock:
            plc_poll_diag['trigger_total_ms'] += (t1 - t0) * 1000.0
            plc_poll_diag['trigger_count'] += 1

        elapsed = time.time() - loop_start
        remaining = PLC_POLL_TARGET_INTERVAL - elapsed
        if remaining > 0:
            plc_thread_stop.wait(remaining)


def set_b150(val):
    """B150 을 ON(1) 또는 OFF(0) 으로 직접 설정.

    공정 A PLC(150)가 보내는 시작 트리거를 흉내내는 수동 도구.
    val=1(ON) 후 monitor_plc_signals() 의 상승 엣지 감지가 동작하여
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
# YOLO 비전 검사 (C 공정 Voting 다수결 이식)
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
    """프레임 1장을 YOLO 로 추론해서 판정 결과 반환.

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
    """판정 결과 처리 — PLC 즉시 통보 + 비전 결과 메모리 저장.

    공정 C 와 달리 B 는 DB INSERT 를 M1130 종료 신호 시점에 수행.
    여기서는 PLC 양품/불량 펄스만 보내고, 결과는 last_vision_result 에 보관.
    """
    global last_vision_result

    judgment = "양품 ✅" if is_normal else "불량 ❌"
    print(f"  ⚡ [voting 판정] 채택 클래스: {class_name} → {judgment}")

    # 1. PLC 양품/불량 통보 (로봇 기동은 PLC 자체 센서/래더가 처리)
    send_vision_result_to_plc(is_normal)

    # 2. 비전 결과를 메모리에 저장 — 실제 DB INSERT 는 M1130 시점에 수행
    vision_result = 'OK' if is_normal else 'NG'
    defect_type = None if is_normal else class_name
    last_vision_result = {
        'vision_result': vision_result,
        'defect_type': defect_type
    }
    print(f"  💾 비전 결과 메모리 저장 — {ADDR_DONE_A}(공정 완료) 시 DB INSERT 예정 (현재: {vision_result})")


def commit_defect_timeout():
    """M101 ON 후 DEFECT_TIMEOUT 안에 양품 voting 미완료 → 불량 판정 (voting 없이 직접).

    차체조립에서 부품은 도착했는데(M101 ON) b/g/r 어느 것도 검출 못한 케이스.
    M260 펄스 + 메모리 저장 → B_Process INSERT (NG, defect_type=DEFECT_CLASS_NAME).
    commit_inspection_result(False, ...) 와 동일 효과지만 'voting 판정' 로그 대신
    타임아웃 로그를 남겨 원인 추적이 쉬워짐.
    """
    global last_vision_result
    print(f"  ⏰ [타임아웃 불량] {ADDR_PART_PRESENT} ON 후 {DEFECT_TIMEOUT}s 양품 판정 없음 → 불량 ❌")
    send_vision_result_to_plc(False)   # 기존 M260 펄스 로직 재사용
    last_vision_result = {
        'vision_result': 'NG',
        'defect_type': DEFECT_CLASS_NAME,
    }
    print(f"  💾 불량 결과 메모리 저장 — {ADDR_DONE_A}(공정 완료) 시 DB INSERT 예정 (defect_type={DEFECT_CLASS_NAME})")


def execute_vision_inspection_manual(frame, conf_thresh):
    """V 키 — 수동 비전 검사 (PLC 우회, 디버그용).

    voting 없이 단일 프레임으로 즉시 판정 → PLC 통보 + 메모리 저장.
    """
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
# PLC 신호 폴링 + 검출 단계 (C 공정 Voting 다수결 이식)
# ═══════════════════════════════════════════════════════════════════════

def monitor_plc_signals():
    """B150(비전 트리거) + M101(부품 도착) + B1790(공정 A 종료) 폴링 — 모두 PLC 140.

    B150 상승 엣지: 새 검사 사이클 시작 — trigger_on_time 갱신 + 잠금 해제
    B150 하강 엣지: 검출 모드 종료 + 사이클 요약
    M101 상승 엣지: 불량 타임아웃 카운터 시작 (DEFECT_TIMEOUT 후 양품 없으면 M260)
    M101 하강 엣지: 불량 타임아웃 카운터 리셋 (부품 사라짐)
    B1790 상승 엣지: B_Process INSERT (+ NG 면 finalize_defect_at_b)
    """
    global prev_vision_trigger, prev_done_a, prev_part_present
    global trigger_on_time, result_sent_this_cycle, last_view_frame
    global last_vision_result, part_present_on_time
    global cycle_total_frames, cycle_detect_frames, cycle_class_counter
    global cycle_first_detect_at, cycle_last_detect_at
    global voting_started_at, voting_buffer

    if plc is None:
        return

    # BG 폴링 스레드가 갱신한 캐시에서 read (O(1), 메인 루프 FPS 영향 X)
    with plc_cache_lock:
        curr_vision_trigger = plc_cache['vision_trigger']
        curr_done_a = plc_cache['done_a']
        curr_part_present = plc_cache['part_present']

    # ───── B150 상승 엣지 → 새 검사 사이클 시작 ─────
    if curr_vision_trigger and not prev_vision_trigger:
        trigger_on_time = time.time()
        result_sent_this_cycle = False
        last_vision_result = None   # 새 사이클 — 이전 비전 결과 초기화
        # Voting 상태 리셋
        voting_started_at = 0.0
        voting_buffer = []
        # M101 타임아웃 리셋 — 사이클 시작 시점에 M101이 이미 ON이면 지금부터 카운트
        part_present_on_time = time.time() if curr_part_present else 0.0
        # 사이클 누적 카운터 리셋
        cycle_total_frames = 0
        cycle_detect_frames = 0
        cycle_class_counter = Counter()
        cycle_first_detect_at = 0.0
        cycle_last_detect_at = 0.0
        print(f"\n[{datetime.now().strftime('%H:%M:%S')}] 🟢 {ADDR_VISION_TRIGGER} ON — 안정화 대기 ({IGNORE_DURATION}s)...")
        if curr_part_present:
            print(f"  🟡 {ADDR_PART_PRESENT} 이미 ON — 불량 타임아웃 카운트 시작 ({DEFECT_TIMEOUT}s)")

    # ───── M101 상승 엣지 → 불량 타임아웃 카운터 시작 ─────
    if curr_part_present and not prev_part_present:
        # B150 ON 상태에서만 의미 있음. B150 OFF면 사이클 시작 시 다시 처리됨.
        if curr_vision_trigger and not result_sent_this_cycle:
            part_present_on_time = time.time()
            print(f"  🟡 {ADDR_PART_PRESENT} ON — 부품 도착, 불량 타임아웃 {DEFECT_TIMEOUT}s 카운트 시작")

    # ───── M101 하강 엣지 → 불량 타임아웃 카운터 리셋 ─────
    if not curr_part_present and prev_part_present:
        if part_present_on_time > 0.0 and not result_sent_this_cycle:
            elapsed_pp = time.time() - part_present_on_time
            print(f"  ⚫ {ADDR_PART_PRESENT} OFF — 타임아웃 카운터 리셋 (경과 {elapsed_pp:.2f}s)")
        part_present_on_time = 0.0

    # ───── B150 하강 엣지 → 검출 모드 종료 + 사이클 요약 출력 ─────
    if not curr_vision_trigger and prev_vision_trigger:
        print(f"\n[{datetime.now().strftime('%H:%M:%S')}] 🔴 {ADDR_VISION_TRIGGER} OFF — 추론 대기모드")
        if voting_started_at > 0.0 and not result_sent_this_cycle:
            print(f"  ⚠ voting 미완료 상태로 OFF — 누적 {len(voting_buffer)}프레임 폐기")
        voting_started_at = 0.0
        voting_buffer = []
        part_present_on_time = 0.0   # 사이클 종료 — 타임아웃 카운터도 해제
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

    # ───── B1790 상승 엣지 (PLC 140) → 공정 A 종료 → B_Process INSERT ─────
    # 양품 → B_Process INSERT 만 (종합 처리는 공정 C 가 담당)
    # 불량 → B_Process INSERT + tbl_total NG + carrier_map RELEASE (C 로 안 넘어감)
    if curr_done_a and not prev_done_a:
        product_sn, tray_sn = get_latest_active_info()
        sn_str = f"{product_sn} (Tray: {tray_sn})" if product_sn else "(ACTIVE S/N 없음)"
        print(f"\n[{datetime.now().strftime('%H:%M:%S')}] 🟢 공정 A 종료 ({ADDR_DONE_A} ON, PLC 140) — {sn_str}")

        if product_sn:
            # 비전 결과로 INSERT (없으면 default 'OK' 로 양품 가정)
            if last_vision_result is not None:
                vr = last_vision_result['vision_result']
                dt = last_vision_result.get('defect_type')
            else:
                vr, dt = 'OK', None
                print(f"  ℹ 비전 결과 없음 — default OK 로 INSERT")

            if record_robot_b_result(product_sn, tray_sn, vr, dt):
                print(f"  ✅ B_Process INSERT: {product_sn} → {vr}")

            # 양품/불량 분기
            if vr == 'NG':
                # 공정 B 불량 → C 로 안 넘어감 → B 에서 종합 처리 마무리
                print(f"  ⚠ 불량 판정 — 공정 C 미진행 → B 에서 종합 처리")
                final, ra, rb = finalize_defect_at_b(product_sn)
                if final:
                    print(f"  ✅ tbl_total INSERT (final={final}) + carrier_map RELEASE")
                    print(f"     공정 A: {ra or '미실시'}, 공정 B: {rb}, 공정 C: 미실시")
                else:
                    print(f"  ❌ 종합 처리 실패 — carrier_map 수동 RELEASE 필요")
            else:
                # 양품 → C 로 넘어감 (C 의 M1120 이 최종 종합)
                print(f"  ✓ 양품 — 공정 C 로 진행 (C 에서 최종 tbl_total + RELEASE)")

            # 다음 사이클을 위해 비전 결과 정리
            last_vision_result = None
        else:
            print(f"  ⚠ ACTIVE S/N 없음, INSERT 스킵")

    prev_vision_trigger = curr_vision_trigger
    prev_done_a = curr_done_a
    prev_part_present = curr_part_present


def vision_step(frame, conf_thresh):
    """매 프레임 호출 — B150 ON 동안 안정화 후 Voting 누적 판정 (+ M101 타임아웃 불량).

    동작 흐름:
      1) B150 OFF: 원본 프레임 + Standby 텍스트
      2) B150 ON: IGNORE_DURATION 동안 안정화 표시 (추론 X)
      3) 안정화 시간 경과 후 매 프레임 추론
      4) 첫 검출 발생 → voting 시작 (VOTING_DURATION s 카운트)
      5) voting 윈도우 동안 매 검출 결과를 voting_buffer 에 누적
      6) VOTING_DURATION 경과 → 다수결로 최종 클래스 결정 → PLC + 메모리 저장 (사이클당 1회)
      7) result_sent_this_cycle=True 잠금 → B150 OFF→ON 시 해제

    M101 타임아웃 (안전망):
      M101 ON 상태에서 DEFECT_TIMEOUT 안에 양품 voting 미완료 시 → 불량 판정 + M260.
      안정화/voting 단계와 독립적으로 매 프레임 체크 (IGNORE_DURATION 중에도 동작).
    """
    global last_view_frame, result_sent_this_cycle, last_diag_at
    global cycle_total_frames, cycle_detect_frames, cycle_class_counter
    global cycle_first_detect_at, cycle_last_detect_at
    global voting_started_at, voting_buffer

    # B150 OFF 상태 — 대기 화면
    if not prev_vision_trigger:
        view = frame.copy()
        cv2.putText(view, f"Standby [Thresh: {conf_thresh:.2f}]", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        last_view_frame = view
        return

    now = time.time()

    # ─── M101 타임아웃 체크 — 안정화/voting 단계와 독립 ───
    # M101 ON 후 DEFECT_TIMEOUT 안에 양품 판정 없으면 즉시 불량 처리.
    if not result_sent_this_cycle and part_present_on_time > 0.0:
        pp_elapsed = now - part_present_on_time
        if pp_elapsed >= DEFECT_TIMEOUT:
            commit_defect_timeout()
            result_sent_this_cycle = True
            view = frame.copy()
            cv2.putText(view, f"DEFECT (M101 timeout {pp_elapsed:.1f}s)", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            last_view_frame = view
            return

    elapsed = now - trigger_on_time

    # 안정화 대기 구간 — 추론 건너뛰고 표시만
    if elapsed < IGNORE_DURATION:
        view = frame.copy()
        cv2.putText(view, f"Filtering Initial Data... ({elapsed:.2f}s)", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        last_view_frame = view
        return

    # 추론 + voting 누적
    result = vision_inspect(frame, conf_thresh)
    view = result['annotated_frame']
    # M101 타임아웃 진행률을 라벨에 함께 표시 (남은 시간 가시화)
    if part_present_on_time > 0.0 and not result_sent_this_cycle:
        pp_remain = max(0.0, DEFECT_TIMEOUT - (now - part_present_on_time))
        label = f"Thresh: {conf_thresh:.2f} | SCANNING... | timeout in {pp_remain:.1f}s"
    else:
        label = f"Thresh: {conf_thresh:.2f} | SCANNING..."
    cv2.putText(view, label, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    last_view_frame = view

    # 사이클 누적 카운터 갱신
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
                    print(f"  ⚠ 알 수 없는 클래스: {best_class} — PLC/메모리 송신 스킵")

                if is_normal is not None:
                    commit_inspection_result(is_normal, best_class)
                result_sent_this_cycle = True


# ═══════════════════════════════════════════════════════════════════════
# 초기화
# ═══════════════════════════════════════════════════════════════════════

# cv2 창 + 트랙바 — 트랙바는 namedWindow 가 먼저 있어야 생성 가능
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WINDOW_NAME, 960, 720)
cv2.createTrackbar(TRACKBAR_NAME, WINDOW_NAME,
                   int(INITIAL_CONF_THRESHOLD * 100), 100, on_trackbar)

load_yolo_model()
connect_plc()

# PLC 백그라운드 폴링 스레드 시작 — PLC 140 단일 접속으로 3비트(B150/M101/B1790) 통합 폴링
plc_poll_thread_140 = threading.Thread(target=plc_polling_loop_140, name='plc-poll-140', daemon=True)
plc_poll_thread_140.start()
print(f"✅ PLC 140 백그라운드 폴링 스레드 시작 (목표 {1.0/PLC_POLL_TARGET_INTERVAL:.0f}Hz)")

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
if not cap.isOpened():
    print(f"\n[FATAL] 카메라(인덱스 {CAMERA_INDEX}) 초기화 실패 — 종료합니다.")
    raise SystemExit(1)


# ═══════════════════════════════════════════════════════════════════════
# 시작 안내
# ═══════════════════════════════════════════════════════════════════════

print("\n" + "=" * 60)
print("  공정 B — 비전 검사 + 중간 공정 (펜던트 IF 방식)")
print("=" * 60)
print("  [자동 동작 — Voting 다수결 판정 방식 (C 공정 이식) + M101 불량 타임아웃]")
print(f"    {ADDR_VISION_TRIGGER} 상승 엣지 → 안정화 {IGNORE_DURATION}s 대기 → 매 프레임 추론")
print(f"      └─ 첫 검출 발생 → {VOTING_DURATION}s 동안 결과 누적")
print(f"      └─ 다수결로 클래스 채택 → 양품(M250) / 불량(M260) 통보 + 메모리 저장")
print(f"      └─ 사이클당 1회 송신 (재트리거: {ADDR_VISION_TRIGGER} OFF→ON)")
print(f"    {ADDR_PART_PRESENT} ON → {DEFECT_TIMEOUT}s 안에 양품 미검출 시 → 불량(M260, defect_type={DEFECT_CLASS_NAME})")
print(f"    {ADDR_DONE_A} 감지 → B_Process INSERT (NG 면 finalize_defect_at_b)")
print("")
print("  [수동 제어]")
print("    V  : 비전 검사 수동 실행 (PLC 우회, 디버그)")
print(f"    1  : {ADDR_VISION_TRIGGER} ON  (PLC 140 시작 트리거 흉내 — 공정 A 우회)")
print(f"    0  : {ADDR_VISION_TRIGGER} OFF (재트리거 준비)")
print("    S  : 현재 ACTIVE 조회")
print("    Q  : 종료")
print("    ※ 화면 상단 트랙바로 conf 임계값 실시간 조정 가능")
print("=" * 60)
print("")
print(f"  [DB 서버] {DB_CONFIG['host']}:{DB_CONFIG['port']}")
print(f"  [PLC]     {PLC_IP}:{PLC_PORT} (공정 B 메인 — 3비트 통합 폴링)")
print(f"  [카메라]  cv2.VideoCapture(index={CAMERA_INDEX}, rotation={CAMERA_ROTATION})")
print(f"  [로봇 B]  PLC 자체 제어 — 펜던트 IF")
print("")
print("  [PLC 비트]")
print(f"    {ADDR_VISION_TRIGGER}     : 비전 트리거 (공정 A PLC 150 ← B디바이스)")
print(f"    {ADDR_PART_PRESENT}     : 부품 도착 (PLC 140 → Vision, 불량 타임아웃 기준)")
print(f"    {ADDR_NORMAL}      : 양품 통보 (Vision → PLC 140)")
print(f"    {ADDR_CRACK}      : 불량 통보 (Vision → PLC 140)")
print(f"    {ADDR_DONE_A}  : 공정 A 종료 (PLC 140 B-디바이스 → Vision)")
print("")
print("  [로봇 제어 방식]")
print("    CC-Link → PLC 자체 센서/래더 → 펜던트 IF (공정 A/C 와 동일)")
print("    Vision PC 는 양품/불량 통보만, 로봇 기동은 PLC 가 자체 처리")
print("")
print("  [비전 클래스]")
print(f"    양품: {NORMAL_CLASSES}")
print(f"    불량: {CRACK_CLASSES}")
print(f"    초기 conf 임계값: {INITIAL_CONF_THRESHOLD}  |  안정화: {IGNORE_DURATION}s  |  Voting: {VOTING_DURATION}s  |  불량 타임아웃({ADDR_PART_PRESENT}): {DEFECT_TIMEOUT}s")
print("=" * 60)

show_active_status()
print("")


# ═══════════════════════════════════════════════════════════════════════
# 메인 루프
# ═══════════════════════════════════════════════════════════════════════

try:
    while True:
        # ── [구간1] 카메라 캡처 + 회전 보정 ──
        _t0 = time.time()
        ret, frame = cap.read()
        if not ret:
            break
        if CAMERA_ROTATION is not None:
            frame = cv2.rotate(frame, CAMERA_ROTATION)
        _t1 = time.time()
        section_times['capture'] += _t1 - _t0
        section_counts['capture'] += 1

        # 트랙바에서 현재 conf 임계값
        conf_thresh = get_current_conf_threshold()

        # ── [구간2] PLC 신호 폴링 (캐시 read만 — 실제 PLC IO 는 BG 스레드가 처리) ──
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
                bg_t_avg = plc_poll_diag['trigger_total_ms'] / max(1, bg_t_count)
                plc_poll_diag['trigger_total_ms'] = 0.0
                plc_poll_diag['trigger_count'] = 0

            print(f"  [timing] " + " | ".join(parts) + f"  (프레임 {section_counts['capture']}장)")
            print(f"  [plc-bg] PLC140 3비트(B150+M101+B1790)={bg_t_avg:.1f}ms × {bg_t_count}/s")

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
            set_b150(1)
        elif key == ord('0'):
            print(f"\n[{datetime.now().strftime('%H:%M:%S')}] 🔴 [수동] {ADDR_VISION_TRIGGER} OFF")
            set_b150(0)
        elif key == ord('s'):
            show_active_status()

finally:
    # ═══════════════════════════════════════════════════════════════════════
    # 종료
    # ═══════════════════════════════════════════════════════════════════════
    print("\n[INFO] 프로그램을 안전하게 종료합니다.")
    # BG 폴링 스레드 정리 (PLC 소켓 종료 전에)
    plc_thread_stop.set()
    try:
        plc_poll_thread_140.join(timeout=3.0)
    except Exception:
        pass
    print("  PLC 140 폴링 스레드 종료")
    try:
        cap.release()
    except Exception:
        pass
    cv2.destroyAllWindows()

    if plc:
        try:
            plc.close()
            print("  PLC (140) 연결 종료")
        except Exception:
            pass

    print("\n" + "=" * 60)
    print("  공정 B 종료")
    print("=" * 60)
