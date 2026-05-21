# C_Process_pendant.py — 공정 C (비전 검사 + 종합 처리)

## 1. 역할

공정 C Vision PC 가 담당하는 일 — **3공정 마지막 단계**:
- B 에서 넘어온 트레이의 최종 비전 검사 (YOLO + 1.5s voting)
- 양품/불량 판정 → PLC 120 에 M250/M260 통보
- `tbl_robot_c` 에 검사 결과 INSERT
- M1120 종료 신호 → `tbl_total` 종합 UPSERT + `carrier_map RELEASE` (**모든 경우**)

> ⚠️ 공정 B 와 달리 C 는 **양품/불량 분기 없이 항상 종합 처리**. 마지막 공정이라 트레이가 반드시 RELEASE 되어야 함.

## 2. 연결 구성

| 항목 | 주소 |
|---|---|
| DB 서버 | `192.168.3.141:3306` / `guest` / `guest1234` |
| 공정 C PLC | `192.168.3.120:2000` (트리거 수신/양품·불량 통보) |
| 관제 PLC 160 | `192.168.3.160:2000` (M1120 종료 신호 폴링) |
| 카메라 | **Intel RealSense D435** (인덱스 미사용, `pyrealsense2`) |
| YOLO 모델 | `C_Vision_6class.pt` (vision_test_done 검증 모델, `factory_mes/models/` 안에서 repo 관리) |

## 3. PLC 비트 매핑

| 비트 | 방향 | 의미 |
|---|---|---|
| `B1390` | PLC 120 → Vision | 비전 검사 시작 트리거 (공정 B → C B디바이스 매핑) |
| `M250` | Vision → PLC 120 | 양품 통보 |
| `M260` | Vision → PLC 120 | 불량 통보 |
| `M1120` | PLC 160 → Vision | 공정 C 종료 신호 (관제 PLC 에서 통합 폴링) |

## 4. ⭐ Voting 다수결 (튀는 값 방지)

```
B1390 ON (상승 엣지)
  ↓
[IGNORE_DURATION = 0.4s 안정화 대기]
  센서/조명 흔들림으로 인한 초기 쓰레기 데이터 무시
  ↓
매 프레임 YOLO 추론 시도
  ↓
첫 검출 발생 → voting 시작
  ↓
[VOTING_DURATION = 1.5s 동안 매 프레임 결과 누적]
  voting_buffer 에 (class_name, conf) 누적
  ↓
1.5s 경과 → 다수결로 최종 클래스 채택
  예: r_normal 28회 / r_crack 3회 → r_normal
  ↓
PLC 양품/불량 통보 + tbl_robot_c INSERT (사이클당 1회)
  ↓
B1390 OFF → ON 들어오면 다시 사이클 시작
```

### 주요 상수

| 상수 | 값 | 의미 |
|---|---|---|
| `IGNORE_DURATION` | `0.4` | B1390 ON 직후 무시 시간 (s) |
| `VOTING_DURATION` | `1.5` | voting 윈도우 (s) |
| `INITIAL_CONF_THRESHOLD` | `0.50` | YOLO conf 초기값 (실행 중 트랙바로 조정) |

## 5. 동작 흐름

```
   [공정 B 종료 + 컨베어 운반]
        │
        ▼ B1390 ON
   PLC 120 트리거 수신
                                          ┌────────────────────────────┐
                                          │  Vision PC                 │
                                          │  1) B1390 상승 엣지 감지    │
                                          │  2) 0.4s 안정화 대기       │
                                          │  3) 검출 폴링              │
                                          │  4) 첫 검출 → 1.5s voting  │
                                          │  5) 다수결 클래스 채택     │
                                          │  6) PLC 120 통보 (M250/M260)│
                                          │  7) 결과 메모리 저장       │
                                          └────────────────────────────┘
                                                       │
                                          [PLC 작업 진행 + 컨베어]
                                                       │
        ▼ M1120 ON (PLC 160 의 관제 비트)              │
                                          ┌────────────────────────────┐
                                          │  Vision PC                 │
                                          │  1) M1120 상승 엣지 감지   │
                                          │  2) finalize_and_release() │
                                          │     ├─ tbl_robot_a 조회    │
                                          │     ├─ tbl_robot_b 조회    │
                                          │     ├─ tbl_robot_c INSERT  │
                                          │     ├─ tbl_total UPSERT    │
                                          │     │   (NG 하나라도면 NG) │
                                          │     └─ carrier_map RELEASE │
                                          │  3) 종합 결과 콘솔 출력    │
                                          └────────────────────────────┘
                                                  ↓
                                          [트레이 RELEASED]
                                          [사이클 완료]
```

## 6. 양품/불량 처리 (공정 C 는 분기 없음)

C 는 **마지막 공정** 이라 양품/불량 무관하게 항상 `tbl_total` UPSERT + `RELEASE`:

```python
# C 의 M1120 핸들러 (단순화)
final = compute_final()   # A=OK, B=OK, C=OK → OK / 하나라도 NG → NG
tbl_total UPSERT
carrier_map RELEASE
```

→ 공정 B 와 다른 점: **B 는 양품일 때 C 로 떠넘기고, 불량일 때 자체 마무리**. C 는 그런 분기 필요 없음.

## 7. 트러블슈팅

### A. "트레이가 영원히 ACTIVE 로 묶임"
**원인**: M1120 신호 못 받음, 또는 `finalize_and_release` 에러.

```sql
-- DB 상태 확인
SELECT * FROM tbl_carrier_map WHERE status='ACTIVE';
SELECT * FROM tbl_total ORDER BY completed_at DESC LIMIT 5;
```

| 시나리오 | 진단 |
|---|---|
| ACTIVE 트레이 있는데 tbl_total 빔 | M1120 신호 못 받음 → test_plc_signal.py 로 PLC 160 M1120 직접 확인 |
| tbl_total 있는데 carrier_map 그대로 | UPDATE 권한 부족 → SHOW GRANTS 확인 |

### B. "검출은 되는데 voting 결과가 이상"
- `VOTING_DURATION` 너무 짧음 → 1.5s 를 2.0s 로 늘려보기
- `IGNORE_DURATION` 너무 짧음 → 카메라 안정화 더 기다리기

### C. "Intel RealSense D435 연결 안 됨"

```python
# pyrealsense2 import 에러 → 설치
pip install pyrealsense2
```

USB 케이블 / 드라이버 (Intel RealSense SDK 2.0) 확인.

### D. "모델 경로 못 찾음"

```python
MODEL_PATH = os.path.join(os.path.dirname(__file__), '..', 'models', 'C_Vision_6class.pt')
```

→ 이제 **repo 내부 상대 경로**. 어느 PC에서든 `factory_mes/` 만 clone 하면 동작.
→ 모델 파일 동봉: `C_Vision_6class.pt` (PyTorch), `C_Vision_6class.onnx` (ONNX 백업), `C_Vision_6class_labels.txt` (라벨).

### E. "M250/M260 PLC 측에서 안 보임"
- `test_plc_signal.py` 의 sweep 모드 (`s`) 로 M250 쓰기 가능한지 확인
- PLC 래더가 M250 OUT 으로 덮어쓰는지 확인 (그러면 다른 자유 M 비트로 변경)

## 8. 키 매뉴얼 (cv2 창에서)

| 키 | 동작 |
|---|---|
| **V** | 비전 검사 수동 실행 (PLC 우회, 디버그) |
| **1** | B1390 ON (PLC B 트리거 흉내, 검출 폴링 시작) |
| **0** | B1390 OFF (재트리거 준비) |
| **S** | 현재 ACTIVE 트레이 조회 |
| **Q** | 종료 |

### conf 트랙바 (실시간 조정)
cv2 창 상단에 conf 임계값 슬라이더가 있어요. 시연 중 검출 안 되면 트랙바를 낮추면 즉시 반영됨.

## 9. 실행

```powershell
conda activate robot
cd MES
python C_Process_pendant.py
```

부팅 시 콘솔 출력 예시:
```
✅ Intel RealSense D435 연결 성공
✅ YOLO 모델 로드 완료 (6class_best.pt)
✅ PLC 연결 (192.168.3.120:2000) — 공정 C 메인
✅ 관제 PLC 연결 (192.168.3.160:2000) — 종료 신호 폴링
...
공정 C — 비전 검사 + 종합 처리 (펜던트 IF 방식)
```

## 10. FPS 안정화 — PLC 폴링 백그라운드 분리 (히스토리)

### 10.1 증상
voting 적용 후 컨베이어 테스트에서 FPS가 갑자기 **0.5fps**까지 떨어지는 현상. 같은 코드인데 세션마다 결과가 달라짐 — voting 자체는 무관, 외부 요인(네트워크/PLC 응답) 의심.

| 시도 | FPS | 비고 |
|------|------|------|
| 1차 (즉시 판정) | 검출 구간 24fps | 정상 동작 |
| 2차 (voting 적용 후) | 0.5fps | 갑자기 죽음 |
| 3차 (재실행) | 35fps | 우연히 정상 |
| 4차 | 0.5fps | 재발 |

### 10.2 진단 — 구간별 타이밍 로그
메인 루프 각 단계 시간을 1초마다 출력:

```python
section_times = {'capture': 0.0, 'plc': 0.0, 'vision': 0.0, 'display': 0.0}
# 매 프레임 _t0~_t4 측정 → 1초마다 구간별 평균 ms 출력
```

**범인 발견:**
```
[timing] capture=232.0ms | plc=2007.2ms | vision=1.0ms | display=2.0ms  (프레임 1장)
```
- PLC 폴링이 매 호출 **2초**(=mcprotocol TCP 타임아웃)
- 메인 루프가 PLC 응답 기다리느라 0.5fps
- YOLO/카메라/디스플레이는 모두 정상

### 10.3 1차 시도 — Throttle (실패)
PLC 폴링을 매 프레임이 아니라 100ms 간격으로만 호출:

```python
PLC_POLL_INTERVAL = 0.1
if _t1 - last_plc_poll_at >= PLC_POLL_INTERVAL:
    monitor_plc_signals()
```

**효과 없음:** throttle은 "호출 빈도"만 줄임. **개별 호출이 2초 블로킹**되는 문제는 해결 X.

### 10.4 2차 시도 — 단일 백그라운드 스레드
PLC IO를 별도 스레드로 분리. 메인 루프는 캐시만 read:

```python
plc_cache = {'vision_trigger': False, 'done_c': False}
plc_cache_lock = threading.Lock()
plc_lock = threading.Lock()           # 소켓 직렬화 (BG read vs 메인 write 충돌 방지)
plc_monitor_lock = threading.Lock()

def plc_polling_loop():
    while not plc_thread_stop.is_set():
        vt = read_plc_bit(ADDR_VISION_TRIGGER)   # PLC 120
        dc = read_done_bit()                      # PLC 160
        with plc_cache_lock:
            plc_cache['vision_trigger'] = vt
            plc_cache['done_c'] = dc
```

**효과:** 메인 FPS 30fps 회복. 진단으로 **PLC 120=3ms / PLC 160=2008ms** 확인 — 관제 PLC 160(M1120)이 범인.

**남은 문제:** BG 스레드가 1개라 PLC 120 폴링도 PLC 160 뒤에 묶임 → B1390 트리거 감지 최대 2초 지연.

### 10.5 3차 시도 (최종) — PLC별 스레드 분리
각 PLC가 독립 소켓이므로 스레드도 분리:

```python
def plc_polling_loop_120():    # PLC 120 (B1390) — 빠른 응답, 50ms 주기
def plc_polling_loop_160():    # PLC 160 (M1120) — 느려도 OK, 자기 페이스
```

**최종 결과:**
```
[timing] capture=23.6ms | plc=0.0ms | vision=1.0ms | display=0.9ms  (프레임 31장)
[plc-bg] B1390(120)=3.2ms × 16/s | M1120(160)=0.0ms × 16/s
```
- 메인 FPS = **30fps 안정**
- B1390 폴링 16~27Hz (50ms 응답성 보장)
- M1120이 느려도 B1390 영향 없음
- voting 결과: **47프레임 / 1.5s** 누적 → 다수결 100% 일관

### 10.6 핵심 교훈
- **외부 시스템(PLC/네트워크)을 메인 루프 동기 호출하지 말 것** — 응답 지연이 FPS 손실로 직결
- **Throttle vs 비동기는 다름** — throttle은 호출 빈도만 줄임, 비동기(스레드)만 블로킹을 진짜 분리
- **자원별로 스레드 분리** — 한 스레드에 빠른/느린 IO 섞으면 빠른 쪽도 느린 쪽에 묶임
- **타이밍 로그 먼저** — 추측 X, 구간별 ms 단위로 측정 후 결정
- **lock 두 종류 분리** — 캐시 lock(짧게 보호) vs 소켓 lock(IO 호출 전체 보호)

---

## 11. 관련 파일

- [C_Process_pendant.py](C_Process_pendant.py) — 본 코드
- `../models/C_Vision_6class.pt`, `.onnx`, `_labels.txt` — YOLO 가중치/라벨
- [B_Process_pendant.py](B_Process_pendant.py), [B_Process_디버깅.md](B_Process_디버깅.md) — 인접 공정 (양품/불량 분기 다름)
- [A_Process_pendant.py](A_Process_pendant.py) — 시작 공정 (QR 스캔)
- [../db/schema.sql](../db/schema.sql), [../db/SETUP.md](../db/SETUP.md)
