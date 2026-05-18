"""
═══════════════════════════════════════════════════════════════════════
  FAiCTORY MES — 공정 C (비전 검사 + 종합 처리)
  ※ 시연용 MVP 버전 — 공정 A/B와 동일한 CC-Link + 펜던트 IF 패턴
═══════════════════════════════════════════════════════════════════════

  [이 PC의 역할]
    - B130 트리거 감지 → 비전 검사 자동 실행 (YOLO)
    - 양품/불량 판정 → 공정 C PLC로 결과 통보 (X0/X10)
    - tbl_robot_c에 검사 결과 INSERT
    - M1120 종료 신호 → tbl_total INSERT + carrier_map RELEASE

  [로봇 제어 방식 — 공정 A/B와 동일]
    Vision PC → PLC(120)에 X0/X10 통보 (양품/불량만)
       ↓
    PLC 자체 센서/래더 로직 → 로봇 트리거
       ↓
    펜던트 IF로 로봇 C 자체 동작

    (CC-Link 기반, 검증된 안정적 패턴 — 시연 안정성 최우선)
    Vision PC는 로봇과 직접 통신하지 않음. PLC가 모든 로봇 기동을 처리.
  
  [연결 구성]
    DB     : 192.168.3.45 (별도 서버, guest 계정)
    PLC    : 192.168.3.120 (공정 C 메인)
             ※ 공정 C에는 PLC 110(컨베어)도 있으나 PLC끼리 통신,
               Vision PC는 메인 PLC 120만 접속
    카메라 : 비전 검사용 (인덱스 1)
    로봇 C : 192.168.3.5 — PLC가 자체 제어 (Vision PC 직접 접속 X)
    모델   : C_VISION.pt (프로젝트 루트, 6 클래스: r/g/b_normal/crack)
  
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
import cv2
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
# 프로토타입 단계 — DB가 이 Vision PC와 같은 노트북에 있어 localhost + root 사용.
# 시연/실 운영으로 분리될 때 별도 계정(예: faictory_mes 전용 사용자) 생성 권장.
DB_CONFIG = {
    'host': 'localhost',
    'port': 3306,
    'user': 'root',
    'password': '1234',
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
ADDR_CRACK     = "M260"    # 불량 신호 (PLC 래더가 인식 → 자체 분기 처리)
# ※ 로봇 동작 트리거(과거 M5)는 PLC 자체 센서 입력으로 대체됨.
#    Vision PC는 양품/불량만 통보하고, 로봇 기동은 PLC 래더가 자체 처리.

# ───── YOLO 모델 ─────
# 모델 파일은 프로젝트 루트(MES의 상위)에 있는 C_VISION.pt
MODEL_PATH = os.path.join(os.path.dirname(__file__), '..', 'C_VISION.pt')
# ultralytics predict()의 conf 임계값 — 이 값 이상의 박스만 결과로 반환됨.
# 기본값(0.25)을 명시적으로 설정해 코드만 봐도 임계값이 분명하게.
CONF_THRESHOLD = 0.25
NORMAL_CLASSES = ['r_normal', 'g_normal', 'b_normal']
CRACK_CLASSES  = ['r_crack',  'g_crack',  'b_crack']

# ───── 카메라 설정 ─────
CAMERA_INDEX = 2

# 카메라 회전 보정 (카메라가 가로/세로로 마운트된 경우)
# None : 회전 없음
# cv2.ROTATE_90_CLOCKWISE         : 시계방향 90°
# cv2.ROTATE_90_COUNTERCLOCKWISE  : 반시계방향 90°
# cv2.ROTATE_180                  : 180°
CAMERA_ROTATION = cv2.ROTATE_90_COUNTERCLOCKWISE

plc = None              # 공정 C PLC 120 (트리거 수신/통보)
plc_monitor = None      # 관제 PLC 160 (종료 신호 폴링)
yolo_model = None

# 폴링용 이전 상태 (상승 엣지 감지)
prev_vision_trigger = False
prev_done_c = False

# 비전 검출 폴링 상태
# B130 ON 감지 → vision_active=True 진입 → 매 프레임 추론 시도
# 첫 검출 발생 → 3초 voting 시작 → 누적 결과로 다수결 판정 → PLC + DB
vision_active = False          # 현재 검출 모드 중인지
last_diag_at = 0.0             # 진단 로그 직전 출력 시각 (1초마다 1번만 출력)
last_annotated_frame = None    # YOLO가 박스 그린 최근 프레임 (검출 중 시각화용)

# 다수결 voting 설정 — 튀는 값/오인식 방지00
VOTING_DURATION = 3.0          # 첫 검출 후 누적 윈도우 (초)
voting_started_at = 0.0        # 첫 검출 시각 (0이면 아직 검출 전)
voting_buffer = []             # 누적 결과 [(class_name, confidence), ...]


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
    """공정 C PLC(120) 비트 1개 읽기 — 트리거 비트용"""
    if plc is None:
        return False
    try:
        result = plc.batchread_bitunits(addr, 1)
        return bool(result[0])
    except Exception:
        return False


def read_done_bit():
    """공정 C 종료 비트 — 관제 PLC 160에서 읽기"""
    if plc_monitor is None:
        return False
    try:
        return bool(plc_monitor.batchread_bitunits(ADDR_DONE_C, 1)[0])
    except Exception:
        return False


def send_vision_result_to_plc(is_normal):
    """비전 결과 PLC 상태 통보 — 공정 A/B와 동일 패턴

    양품 → X0 ON
    불량 → X10 ON

    ※ 로봇 동작 기동은 PLC 자체 센서/래더 로직이 처리.
       Vision PC는 양품/불량 결과만 통보하면 됨.
    """
    if plc is None:
        result_text = "양품" if is_normal else "불량"
        print(f"  [시뮬] PLC 없음 — {result_text} 가상 전송")
        return

    try:
        if is_normal:
            plc.batchwrite_bitunits(ADDR_NORMAL, [1])
            print(f"  [PLC] {ADDR_NORMAL} ON (양품)")
            sleep(1.0)
            plc.batchwrite_bitunits(ADDR_NORMAL, [0])
            print(f"  [PLC] {ADDR_NORMAL} OFF")
        else:
            plc.batchwrite_bitunits(ADDR_CRACK, [1])
            print(f"  [PLC] {ADDR_CRACK} ON (불량)")
            sleep(1.0)
            plc.batchwrite_bitunits(ADDR_CRACK, [0])
            print(f"  [PLC] {ADDR_CRACK} OFF")
    except Exception as e:
        print(f"  ❌ PLC 전송 실패: {e}")


def set_b130(val):
    """B130을 ON(1) 또는 OFF(0) 으로 직접 설정.

    공정 B PLC(130)가 보내는 시작 트리거를 흉내내는 수동 도구.
    val=1(ON) 후 monitor_plc_signals()의 상승 엣지 감지가 동작하여
    execute_vision_inspection() 자동 호출됨.

    재트리거하려면 val=0 으로 한 번 OFF 후 다시 val=1 (상승 엣지 필요).
    """
    if plc is None:
        print(f"  ⚠ PLC 미연결 — {ADDR_VISION_TRIGGER} 조작 불가")
        return
    try:
        plc.batchwrite_bitunits(ADDR_VISION_TRIGGER, [val])
        state = "ON " if val else "OFF"
        print(f"  [TRIG] {ADDR_VISION_TRIGGER} {state}")
    except Exception as e:
        print(f"  ❌ {ADDR_VISION_TRIGGER} 조작 실패: {e}")


# ═══════════════════════════════════════════════════════════════════════
# YOLO 비전 검사
# ═══════════════════════════════════════════════════════════════════════

def load_yolo_model():
    """YOLO 모델 로드"""
    global yolo_model
    try:
        yolo_model = YOLO(MODEL_PATH)
        print(f"✅ YOLO 모델 로드 완료 ({MODEL_PATH})")
        print(f"  클래스: {yolo_model.names}")
        return True
    except Exception as e:
        print(f"⚠ YOLO 모델 로드 실패: {e}")
        yolo_model = None
        return False


def vision_inspect(frame):
    """프레임을 YOLO로 추론해서 판정 결과 반환
    
    Returns:
        dict {
            'detected': bool,
            'class_name': str,
            'confidence': float,
            'is_normal': True/False/None,
            'annotated_frame': numpy array
        }
    """
    if yolo_model is None:
        return {'detected': False, 'is_normal': None, 'annotated_frame': frame}
    
    try:
        results = yolo_model.predict(frame, conf=CONF_THRESHOLD, verbose=False)
        result = results[0]
        annotated = result.plot()
        
        if len(result.boxes) == 0:
            return {
                'detected': False,
                'class_name': None,
                'confidence': 0,
                'is_normal': None,
                'annotated_frame': annotated
            }
        
        # 가장 신뢰도 높은 박스 1개
        boxes = result.boxes
        confidences = boxes.conf.cpu().numpy()
        class_ids = boxes.cls.cpu().numpy().astype(int)

        best_idx = confidences.argmax()
        best_conf = confidences[best_idx]
        best_class_id = class_ids[best_idx]
        best_class_name = yolo_model.names[best_class_id]

        # 임계값 필터링 없음 — YOLO 기본값(conf=0.25)이 이미 ultralytics 내부에서 적용됨.
        # 모델이 박스를 반환했으면 그대로 검출로 인정.

        # 양품/불량 판정
        if best_class_name in NORMAL_CLASSES:
            is_normal = True
        elif best_class_name in CRACK_CLASSES:
            is_normal = False
        else:
            is_normal = None
        
        return {
            'detected': True,
            'class_name': best_class_name,
            'confidence': float(best_conf),
            'is_normal': is_normal,
            'annotated_frame': annotated
        }
    
    except Exception as e:
        print(f"  ❌ 비전 추론 에러: {e}")
        return {'detected': False, 'is_normal': None, 'annotated_frame': frame}


def execute_vision_inspection(frame):
    """비전 검사 실행 → PLC 양품/불량 통보 → tbl_robot_c INSERT"""
    print(f"  🔍 비전 검사 시작")
    
    result = vision_inspect(frame)
    
    if not result['detected']:
        print(f"  ⚠ 객체 미검출 또는 신뢰도 낮음")
        return result
    
    print(f"  검출: {result['class_name']} (신뢰도: {result['confidence']:.2f})")
    
    if result['is_normal'] is None:
        print(f"  ⚠ 알 수 없는 클래스: {result['class_name']}")
        return result
    
    judgment = "양품 ✅" if result['is_normal'] else "불량 ❌"
    print(f"  판정: {judgment}")
    
    # 1. PLC 양품/불량 통보 (로봇 기동은 PLC 자체 센서/래더가 처리)
    send_vision_result_to_plc(result['is_normal'])
    
    # 2. tbl_robot_c INSERT
    product_sn, tray_sn = get_latest_active_info()
    if product_sn:
        vision_result = 'OK' if result['is_normal'] else 'NG'
        defect_type = None if result['is_normal'] else result['class_name']
        if record_robot_c_result(product_sn, tray_sn, vision_result, defect_type):
            print(f"  ✅ tbl_robot_c INSERT: {product_sn} → {vision_result}")
    else:
        print(f"  ⚠ ACTIVE S/N 없음, DB 기록 스킵")
    
    return result


# ═══════════════════════════════════════════════════════════════════════
# PLC 신호 폴링 (상승 엣지 감지)
# ═══════════════════════════════════════════════════════════════════════

def monitor_plc_signals():
    """B130(비전 트리거) + M1120(공정 C 종료) 폴링.

    B130 상승 엣지: 검출 폴링 모드 진입 (실제 추론은 vision_polling_step(frame)이 매 프레임)
    B130 하강 엣지: 검출 폴링 모드 중단 (외부에서 OFF 시킨 경우)
    M1120 상승 엣지: tbl_total INSERT + carrier_map RELEASE
    """
    global prev_vision_trigger, prev_done_c, vision_active, last_annotated_frame
    global voting_started_at, voting_buffer

    if plc is None:
        return

    curr_vision_trigger = read_plc_bit(ADDR_VISION_TRIGGER)
    curr_done_c = read_done_bit()   # 종료 비트는 관제 PLC 160에서 읽음

    # ───── B130 상승 엣지 → 검출 폴링 모드 시작 ─────
    if curr_vision_trigger and not prev_vision_trigger:
        print(f"\n[{datetime.now().strftime('%H:%M:%S')}] 🔍 {ADDR_VISION_TRIGGER} ON 감지 — 검출 폴링 시작 (voting {VOTING_DURATION}s)")
        vision_active = True
        # 새 사이클 — voting 초기화
        voting_started_at = 0.0
        voting_buffer = []

    # ───── B130 하강 엣지 → 검출 폴링 중단 (외부에서 OFF) ─────
    if not curr_vision_trigger and prev_vision_trigger and vision_active:
        print(f"  ⏹ {ADDR_VISION_TRIGGER} OFF — 검출 폴링 중단 (voting 누적 {len(voting_buffer)}프레임 폐기)")
        vision_active = False
        voting_started_at = 0.0
        voting_buffer = []
        last_annotated_frame = None

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


def vision_polling_step(frame):
    """vision_active 모드일 때 매 프레임마다 호출 — 추론 + 3초 voting + 결과 처리.

    동작 흐름:
      1) B130 ON 후 매 프레임 추론
      2) 미검출 → 조용히 다음 프레임 (트레이 도착 대기)
      3) 첫 검출 발생 → voting 시작 (3초 카운트)
      4) 3초 동안 매 프레임 결과를 voting_buffer 에 누적
      5) 3초 경과 → 다수결로 최종 클래스 결정 → 양품/불량 판정 → PLC + DB
      6) 모드 해제 (B130 OFF는 PLC/사용자가 처리)

    튀는 값/오인식 방지: 한 프레임이 잘못 분류해도 3초 평균으로 보정.
    컨베이어 시간 무관: B130 ON 동안 트레이가 카메라 앞에 도착하면 즉시 voting 시작.
    """
    global vision_active, last_diag_at, last_annotated_frame
    global voting_started_at, voting_buffer

    if not vision_active:
        return

    # 한 프레임 추론 시도
    result = vision_inspect(frame)
    # YOLO가 박스 그린 프레임 저장 → 메인 cv2 창에 실시간 표시용
    last_annotated_frame = result.get('annotated_frame')

    detected = result.get('detected', False)
    cls = result.get('class_name')
    conf = result.get('confidence', 0.0)

    # ── 진단 로그 — 1초마다 1번 추론 상태 출력 ──
    now = time.time()
    if now - last_diag_at >= 1.0:
        cls_str = cls or '(객체 없음)'
        status = "✓ 검출" if detected else "× 미검출"
        voting_info = ""
        if voting_started_at > 0.0:
            elapsed = now - voting_started_at
            voting_info = f" | voting {elapsed:.1f}s / {VOTING_DURATION}s ({len(voting_buffer)}프레임)"
        print(f"  [추론] {status}: class={cls_str}, conf={conf:.2f}{voting_info}")
        last_diag_at = now

    # ── 검출 결과를 voting 버퍼에 누적 ──
    if detected and cls is not None:
        # 첫 검출이면 voting 카운트 시작
        if voting_started_at == 0.0:
            voting_started_at = now
            voting_buffer = []
            print(f"  🔄 첫 검출 — {VOTING_DURATION}s voting 시작 (class={cls}, conf={conf:.2f})")
        voting_buffer.append((cls, conf))

    # ── voting 종료 판정 ──
    if voting_started_at == 0.0:
        return   # 아직 첫 검출 전

    elapsed = now - voting_started_at
    if elapsed < VOTING_DURATION:
        return   # 누적 중

    # ─────────────────────────────────────────────
    # 3초 경과 → 다수결로 최종 판정
    # ─────────────────────────────────────────────
    total = len(voting_buffer)
    if total == 0:
        print(f"  ⚠ voting 윈도우 끝났지만 누적 결과 없음 — 다시 대기")
        voting_started_at = 0.0
        return

    counter = Counter(cn for cn, _ in voting_buffer)
    best_class, best_count = counter.most_common(1)[0]
    ratio = best_count / total * 100
    avg_conf = sum(c for cn, c in voting_buffer if cn == best_class) / best_count

    # 전체 분포 표시 (디버그)
    dist = ', '.join(f"{cn}={n}" for cn, n in counter.most_common())
    print(f"  📊 voting 결과 ({total}프레임): {dist}")
    print(f"     → 채택: {best_class} ({best_count}/{total} = {ratio:.0f}%, 평균 conf={avg_conf:.2f})")

    # 양품/불량 판정
    if best_class in NORMAL_CLASSES:
        is_normal = True
    elif best_class in CRACK_CLASSES:
        is_normal = False
    else:
        print(f"  ⚠ 알 수 없는 클래스: {best_class} — DB 기록 스킵")
        is_normal = None

    if is_normal is not None:
        judgment = "양품 ✅" if is_normal else "불량 ❌"
        print(f"  최종 판정: {judgment}")

        # 1. PLC 양품/불량 통보
        send_vision_result_to_plc(is_normal)

        # 2. tbl_robot_c INSERT
        product_sn, tray_sn = get_latest_active_info()
        if product_sn:
            vision_result = 'OK' if is_normal else 'NG'
            defect_type = None if is_normal else best_class
            if record_robot_c_result(product_sn, tray_sn, vision_result, defect_type):
                print(f"  ✅ tbl_robot_c INSERT: {product_sn} → {vision_result}")
        else:
            print(f"  ⚠ ACTIVE S/N 없음, DB 기록 스킵")

    # 모드 + voting 상태 해제 (B130 OFF는 PLC 래더 또는 사용자가 0 키로 처리)
    vision_active = False
    voting_started_at = 0.0
    voting_buffer = []
    last_annotated_frame = None
    print(f"  ✓ 검출 완료 — {ADDR_VISION_TRIGGER} OFF는 별도 처리 필요 (다음 트리거 위해)")


# ═══════════════════════════════════════════════════════════════════════
# 초기화
# ═══════════════════════════════════════════════════════════════════════

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# cv2 창을 사용자가 마우스로 크기 조절 가능하게 (잘림 방지)
cv2.namedWindow('Process C - Vision Inspection (Pendant Mode)', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Process C - Vision Inspection (Pendant Mode)', 960, 720)

load_yolo_model()
connect_plc()
connect_plc_monitor()


# ═══════════════════════════════════════════════════════════════════════
# 시작 안내
# ═══════════════════════════════════════════════════════════════════════

print("\n" + "=" * 60)
print("  공정 C — 비전 검사 + 종합 처리 (펜던트 IF 방식)")
print("=" * 60)
print("  [자동 동작]")
print(f"    {ADDR_VISION_TRIGGER} 상승 엣지 → 검출 폴링 (검출될 때까지 또는 OFF까지)")
print(f"      └─ 첫 검출 발생 → {VOTING_DURATION}s voting (튀는 값 방지)")
print(f"      └─ 다수결 결정 → PLC 양품/불량 통보 + DB 기록")
print(f"      └─ {ADDR_VISION_TRIGGER} OFF는 PLC/사용자가 처리 (다음 트리거 위해 필요)")
print(f"    {ADDR_DONE_C} 감지 → tbl_total + carrier_map RELEASE")
print("")
print("  [수동 제어]")
print("    V  : 비전 검사 수동 실행 (PLC 우회, 디버그)")
print(f"    1  : {ADDR_VISION_TRIGGER} ON  (PLC 120 시작 트리거 흉내 — 공정 B 우회)")
print(f"    0  : {ADDR_VISION_TRIGGER} OFF (재트리거 준비)")
print("    S  : 현재 ACTIVE 조회")
print("    Q  : 종료")
print("=" * 60)
print("")
print(f"  [DB 서버] {DB_CONFIG['host']}:{DB_CONFIG['port']}")
print(f"  [PLC]     {PLC_IP}:{PLC_PORT} (공정 C 메인)")
print(f"  [PLC 관제] {PLC_MONITOR_IP}:{PLC_PORT} (종료 신호 폴링)")
print(f"  [로봇 C]  192.168.3.5 (PLC 자체 제어 — 펜던트 IF)")
print("")
print("  [PLC 비트]")
print(f"    {ADDR_VISION_TRIGGER}     : 비전 트리거 (PLC 120, 공정 B ← B디바이스)")
print(f"    {ADDR_NORMAL}      : 양품 통보 (Vision → PLC 120)")
print(f"    {ADDR_CRACK}     : 불량 통보 (Vision → PLC 120)")
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
print(f"    conf 임계값: {CONF_THRESHOLD}  |  voting 윈도우: {VOTING_DURATION}s")
print("=" * 60)

show_active_status()
print("")


# ═══════════════════════════════════════════════════════════════════════
# 메인 루프
# ═══════════════════════════════════════════════════════════════════════

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 카메라 회전 보정 (가로 마운트된 카메라 → 정방향 영상)
    if CAMERA_ROTATION is not None:
        frame = cv2.rotate(frame, CAMERA_ROTATION)

    # PLC 신호 폴링 (B130 / M1120 상승 엣지 감지 → 검출 모드 진입)
    monitor_plc_signals()

    # 검출 폴링 — vision_active 모드일 때만 동작. annotated_frame 갱신.
    vision_polling_step(frame)

    # 화면 표시 — vision_active 중이면 YOLO 박스 그린 프레임, 아니면 원본
    display_frame = last_annotated_frame if (vision_active and last_annotated_frame is not None) else frame
    cv2.imshow('Process C - Vision Inspection (Pendant Mode)', display_frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break
    elif key == ord('v'):
        # 수동 비전 검사 (PLC 우회, 디버그용)
        print(f"\n[{datetime.now().strftime('%H:%M:%S')}] 🔍 [수동] 비전 검사")
        result = execute_vision_inspection(frame)
        if result.get('annotated_frame') is not None:
            cv2.imshow('Vision Result', result['annotated_frame'])
            cv2.waitKey(3000)
            cv2.destroyWindow('Vision Result')
    elif key == ord('1'):
        # B130 ON — PLC 120 시작 트리거 흉내 (공정 B 우회 디버그)
        # monitor_plc_signals이 다음 폴링에서 상승 엣지 감지 → execute_vision_inspection 자동 호출
        print(f"\n[{datetime.now().strftime('%H:%M:%S')}] 🟢 [수동] {ADDR_VISION_TRIGGER} ON")
        set_b130(1)
    elif key == ord('0'):
        # B130 OFF — 재트리거 준비 (다음 1 키로 새 상승 엣지 만들기)
        print(f"\n[{datetime.now().strftime('%H:%M:%S')}] 🔴 [수동] {ADDR_VISION_TRIGGER} OFF")
        set_b130(0)
    elif key == ord('s'):
        show_active_status()


# ═══════════════════════════════════════════════════════════════════════
# 종료
# ═══════════════════════════════════════════════════════════════════════

cap.release()
cv2.destroyAllWindows()

if plc:
    try:
        plc.close()
        print("\n  PLC (120) 연결 종료")
    except:
        pass

if plc_monitor:
    try:
        plc_monitor.close()
        print("  관제 PLC (160) 연결 종료")
    except:
        pass

print("\n" + "=" * 60)
print("  공정 C 종료")
print("=" * 60)