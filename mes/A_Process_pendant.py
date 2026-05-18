"""
═══════════════════════════════════════════════════════════════════════
  FAiCTORY MES — 공정 A (QR 스캐너 + 차종 트리거 + 종료 모니터링)
  ※ 시연용 MVP — 공정 A/B/C 동일 CC-Link + 펜던트 IF 패턴
═══════════════════════════════════════════════════════════════════════

  [이 PC의 역할]
    - QR 스캔으로 트레이 인식
    - tbl_carrier_map에 BIND 등록 (S/N 생성)
    - 차종 비트(M50/M60) → 공정 A PLC(150)로 펄스 전송
    - PLC160의 M1150 폴링 → tbl_robot_a 결과 기록

  [PLC 구성 — 두 PLC 접속]
    PLC 150 (공정 A 메인): 차종 트리거 쓰기 (M50/M60)
    PLC 160 (관제):         M1150 (A 공정 종료 신호) 읽기
    ※ 종료 신호 비트(M1150/M1130/M1120)는 모두 관제 PLC 160에 모여 있음

  [연결 구성]
    DB     : 192.168.3.141 (운영 서버, guest 계정)
    PLC    : 192.168.3.150 (공정 A)
    PLC 관제: 192.168.3.160 (종료 신호 모음)
    카메라 : QR 인식용 (인덱스 1)

  [전체 시스템 구조]
    공정 A: PLC 150 (작업 + 컨베어) — 펜던트 IF 자체 동작 ★
    공정 B: PLC 140 (메인) + PLC 130 (로봇/컨베어/종료) — 펜던트 IF
    공정 C: PLC 120 (메인) + PLC 110 (컨베어) — 펜던트 IF
    관제:   PLC 160 (SCADA + 종료 신호 통합) — Vision PC가 종료 비트만 폴링
═══════════════════════════════════════════════════════════════════════
"""

import cv2
import pymysql
from datetime import datetime
from time import sleep
from pymcprotocol import Type3E

# ═══════════════════════════════════════════════════════════════════════
# 설정
# ═══════════════════════════════════════════════════════════════════════

# ───── DB 서버 ─────
# 운영 DB 서버 (192.168.3.141) — guest 계정
DB_CONFIG = {
    'host': '192.168.3.141',
    'port': 3306,
    'user': 'guest',
    'password': 'guest1234',
    'db': 'faictory_mes',
    'charset': 'utf8mb4',
    'autocommit': True,
    'use_unicode': True,
    'init_command': "SET NAMES utf8mb4"
}

# ───── PLC 설정 ─────
PLC_IP   = "192.168.3.150"   # 공정 A 메인 PLC (트리거 쓰기)
PLC_PORT = 2000

PLC_MONITOR_IP = "192.168.3.160"   # 관제 PLC (M1150 종료 신호 폴링)

# ───── PLC 비트 정의 ─────
# 쓰기 (Vision A → PLC 150)
ADDR_RED  = "M250"   # 빨간차 시작 트리거
ADDR_BLUE = "M260"   # 파란차 시작 트리거

# 읽기 (PLC 160 → Vision A)
ADDR_DONE_A = "M1150"   # 공정 A 종료 신호 (관제 PLC 160에서 읽음)

# ───── 카메라 설정 ─────
CAMERA_INDEX = 1   # QR 카메라

plc = None              # 공정 A PLC (트리거 쓰기)
plc_monitor = None      # 관제 PLC 160 (종료 신호 폴링)
prev_done_a = False     # M1150 상승 엣지 감지용


# ═══════════════════════════════════════════════════════════════════════
# DB 함수
# ═══════════════════════════════════════════════════════════════════════

def get_tray_info(tray_id):
    """Tray 마스터 조회"""
    try:
        conn = pymysql.connect(**DB_CONFIG)
        with conn.cursor() as cur:
            cur.execute(
                "SELECT tray_type, status FROM tbl_tray_master WHERE tray_id = %s",
                (tray_id,)
            )
            row = cur.fetchone()
        conn.close()
        return {'tray_type': row[0], 'status': row[1]} if row else None
    except Exception as e:
        print(f"DB 조회 에러: {e}")
        return None


def has_active_binding(tray_id):
    """이 Tray에 이미 ACTIVE 매핑이 있는지 확인 (중복 BIND 방지)"""
    try:
        conn = pymysql.connect(**DB_CONFIG)
        with conn.cursor() as cur:
            cur.execute(
                "SELECT product_sn FROM tbl_carrier_map "
                "WHERE tray_id = %s AND status = 'ACTIVE'",
                (tray_id,)
            )
            row = cur.fetchone()
        conn.close()
        return row[0] if row else None
    except Exception as e:
        print(f"활성 매핑 조회 에러: {e}")
        return None


def generate_product_sn(tray_type):
    """오늘 일일 순번 + 차종 코드로 S/N 생성

    형식: YYMMDD-RD/BL-NNNN
    """
    today = datetime.now().strftime('%y%m%d')
    type_code = 'RD' if tray_type == 'RED' else 'BL'
    try:
        conn = pymysql.connect(**DB_CONFIG)
        with conn.cursor() as cur:
            cur.execute(
                "SELECT COUNT(*) FROM tbl_carrier_map "
                "WHERE DATE(bound_at) = CURDATE() "
                "AND product_sn LIKE %s",
                (f'%-{type_code}-%',)
            )
            count = cur.fetchone()[0] + 1
        conn.close()
        return f'{today}-{type_code}-{count:04d}'
    except Exception as e:
        print(f"S/N 생성 에러: {e}")
        return None


def register_carrier_mapping(tray_id, product_sn):
    """tbl_carrier_map에 매핑 등록 — 공정 시작점"""
    try:
        conn = pymysql.connect(**DB_CONFIG)
        with conn.cursor() as cur:
            cur.execute(
                "INSERT INTO tbl_carrier_map (tray_id, product_sn, status, bound_at) "
                "VALUES (%s, %s, 'ACTIVE', NOW())",
                (tray_id, product_sn)
            )
        conn.close()
        return True
    except Exception as e:
        print(f"매핑 INSERT 에러: {e}")
        return False


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


def record_robot_a_result(product_sn, tray_sn, vision_result='OK', defect_type=None):
    """공정 A 완료 → tbl_robot_a INSERT"""
    try:
        conn = pymysql.connect(**DB_CONFIG)
        with conn.cursor() as cur:
            cur.execute(
                "INSERT INTO tbl_robot_a "
                "(product_sn, machine_name, recorded_at, tray_sn, vision_result, defect_type) "
                "VALUES (%s, %s, NOW(), %s, %s, %s)",
                (product_sn, 'RobotA_Indy7', tray_sn, vision_result, defect_type)
            )
        conn.close()
        return True
    except Exception as e:
        print(f"  ❌ robot_a INSERT 에러: {e}")
        return False


# ═══════════════════════════════════════════════════════════════════════
# Manual Override (테스트/디버깅용)
# ═══════════════════════════════════════════════════════════════════════

def release_latest_active():
    """가장 최근 ACTIVE 트레이 1개 강제 RELEASE"""
    try:
        conn = pymysql.connect(**DB_CONFIG)
        with conn.cursor() as cur:
            cur.execute(
                "SELECT id, tray_id, product_sn FROM tbl_carrier_map "
                "WHERE status = 'ACTIVE' "
                "ORDER BY bound_at DESC LIMIT 1"
            )
            row = cur.fetchone()
            if row is None:
                print("  ℹ DB에 ACTIVE 트레이가 없음")
                conn.close()
                return False
            row_id, tray_id, product_sn = row
            cur.execute(
                "UPDATE tbl_carrier_map "
                "SET status='RELEASED', released_at=NOW() WHERE id=%s",
                (row_id,)
            )
        conn.close()
        print(f"  🔧 가장 최근 ACTIVE 해제: {tray_id} ({product_sn})")
        return True
    except Exception as e:
        print(f"  ❌ 에러: {e}")
        return False


def manual_release_all():
    """모든 ACTIVE 매핑 강제 RELEASE"""
    try:
        conn = pymysql.connect(**DB_CONFIG)
        with conn.cursor() as cur:
            cur.execute(
                "UPDATE tbl_carrier_map "
                "SET status='RELEASED', released_at=NOW() "
                "WHERE status='ACTIVE'"
            )
            affected = cur.rowcount
        conn.close()
        if affected > 0:
            print(f"  🔧 전체 강제 해제됨 ({affected}건)")
        else:
            print(f"  ℹ ACTIVE 트레이가 없음")
        return affected
    except Exception as e:
        print(f"  ❌ 에러: {e}")
        return 0


def show_active_status():
    """현재 ACTIVE 상태 모든 매핑 조회"""
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
    """공정 A PLC 연결 (트리거 쓰기용)"""
    global plc
    try:
        plc = Type3E()
        plc.connect(PLC_IP, PLC_PORT)
        print(f"✅ PLC 연결 ({PLC_IP}:{PLC_PORT}) — 공정 A 메인")
        return True
    except Exception as e:
        print(f"⚠ PLC 연결 실패 — 시뮬 모드 ({e})")
        plc = None
        return False


def connect_plc_monitor():
    """관제 PLC 160 연결 (M1150 종료 신호 폴링용)"""
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


def trigger_plc(tray_type):
    """QR 인식 → 차종 비트 1초 펄스 (Vision → PLC 150)"""
    if plc is None:
        print(f"  [시뮬] PLC 없음 — '{tray_type}' 가상 전송")
        return
    addr = ADDR_RED if tray_type == 'RED' else ADDR_BLUE
    try:
        plc.batchwrite_bitunits(addr, [1])
        print(f"  [PLC] {addr} ON")
        sleep(1.0)
        plc.batchwrite_bitunits(addr, [0])
        print(f"  [PLC] {addr} OFF")
    except Exception as e:
        print(f"  ❌ PLC 전송 실패: {e}")


def read_done_bit():
    """공정 A 종료 비트 — 관제 PLC 160에서 읽기"""
    if plc_monitor is None:
        return False
    try:
        return bool(plc_monitor.batchread_bitunits(ADDR_DONE_A, 1)[0])
    except Exception:
        return False


# ═══════════════════════════════════════════════════════════════════════
# 공정 A 종료 신호 폴링 (상승 엣지 감지) — PLC 160에서
# ═══════════════════════════════════════════════════════════════════════

def monitor_process_a_completion():
    """관제 PLC 160의 M1150 폴링해서 공정 A 종료 시 자동 DB 기록"""
    global prev_done_a

    if plc_monitor is None:
        return

    curr_done_a = read_done_bit()

    # 상승 엣지 감지
    if curr_done_a and not prev_done_a:
        print(f"\n[{datetime.now().strftime('%H:%M:%S')}] 🟢 공정 A 종료 ({ADDR_DONE_A} ON, PLC 160)")
        product_sn, tray_sn = get_latest_active_info()
        if product_sn:
            if record_robot_a_result(product_sn, tray_sn, 'OK'):
                print(f"  ✅ tbl_robot_a INSERT: {product_sn} (Tray: {tray_sn})")
        else:
            print(f"  ⚠ ACTIVE S/N 없음, INSERT 스킵")

    prev_done_a = curr_done_a


# ═══════════════════════════════════════════════════════════════════════
# 초기화
# ═══════════════════════════════════════════════════════════════════════

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
detector = cv2.QRCodeDetector()

# cv2 창을 사용자가 마우스로 크기 조절 가능하게
cv2.namedWindow('Process A - QR + Trigger (Pendant Mode)', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Process A - QR + Trigger (Pendant Mode)', 960, 720)

connect_plc()
connect_plc_monitor()

last_decoded = ""


# ═══════════════════════════════════════════════════════════════════════
# 시작 안내
# ═══════════════════════════════════════════════════════════════════════

print("\n" + "=" * 60)
print("  공정 A — QR 스캐너 + 차종 트리거 (펜던트 IF 방식)")
print("=" * 60)
print("  [자동 동작]")
print("    QR 갖다 대면 → carrier_map BIND + 차종 신호 PLC 150 전송")
print(f"    {ADDR_DONE_A} 감지 (PLC 160) → tbl_robot_a INSERT")
print("")
print("  [수동 제어]")
print("    R  : 가장 최근 ACTIVE 트레이 1개 RELEASE")
print("    A  : 모든 ACTIVE 트레이 RELEASE")
print("    S  : 현재 ACTIVE 조회")
print("    Q  : 종료")
print("=" * 60)
print("")
print(f"  [DB 서버] {DB_CONFIG['host']}:{DB_CONFIG['port']}")
print(f"  [PLC]     {PLC_IP}:{PLC_PORT} (공정 A 메인)")
print(f"  [PLC 관제] {PLC_MONITOR_IP}:{PLC_PORT} (종료 신호 폴링)")
print("")
print("  [PLC 비트]")
print(f"    {ADDR_RED}    : RED 차종 시작 펄스 (Vision → PLC 150)")
print(f"    {ADDR_BLUE}    : BLUE 차종 시작 펄스 (Vision → PLC 150)")
print(f"    {ADDR_DONE_A} : 공정 A 종료 (PLC 160 → Vision)")
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

    display_frame = frame.copy()

    # ───── QR 인식 ─────
    data, bbox, _ = detector.detectAndDecode(frame)

    if bbox is not None and data:
        data = data.strip()
        bbox = bbox.astype(int)
        for i in range(len(bbox[0])):
            cv2.line(display_frame, tuple(bbox[0][i]),
                     tuple(bbox[0][(i + 1) % len(bbox[0])]),
                     (255, 255, 255), 3)
        cv2.putText(display_frame, data, (30, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        if data != last_decoded:
            print(f"\n[{datetime.now().strftime('%H:%M:%S')}] QR 감지: {data}")

            info = get_tray_info(data)
            if info is None:
                print(f"  ⚠ 미등록 Tray")
                last_decoded = data
                continue

            tray_type = info['tray_type']
            status = info['status']
            print(f"  차종: {tray_type}")
            print(f"  상태: {status}")

            if status != 'IN_USE':
                print(f"  ⚠ 사용 불가")
                last_decoded = data
                continue

            existing_sn = has_active_binding(data)
            if existing_sn:
                print(f"  ⚠ 이미 사용 중: {existing_sn}")
                print(f"  💡 강제 해제: 'R' / 'A'")
                last_decoded = data
                continue

            product_sn = generate_product_sn(tray_type)
            print(f"  S/N: {product_sn}")

            if register_carrier_mapping(data, product_sn):
                print(f"  ✅ tbl_carrier_map INSERT 완료")
                trigger_plc(tray_type)
            else:
                print(f"  ❌ INSERT 실패")

            last_decoded = data

    # ★ 매 프레임마다 공정 A 종료 폴링 (PLC 160의 M1150)
    monitor_process_a_completion()

    cv2.imshow('Process A - QR + Trigger (Pendant Mode)', display_frame)

    # ───── 키 입력 ─────
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break
    elif key == ord('r'):
        print(f"\n🔧 [MANUAL OVERRIDE] 가장 최근 ACTIVE RELEASE")
        release_latest_active()
        last_decoded = ""
    elif key == ord('a'):
        print(f"\n🔧 [MANUAL OVERRIDE] 전체 ACTIVE RELEASE")
        manual_release_all()
        last_decoded = ""
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
        print("\n  PLC (150) 연결 종료")
    except:
        pass

if plc_monitor:
    try:
        plc_monitor.close()
        print("  관제 PLC (160) 연결 종료")
    except:
        pass

print("\n" + "=" * 60)
print("  공정 A 종료")
print("=" * 60)
show_active_status()
print("")
