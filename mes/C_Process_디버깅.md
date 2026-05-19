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
| YOLO 모델 | `6class_best.pt` (vision_test_done 검증 모델, 외부 절대 경로) |

## 3. PLC 비트 매핑

| 비트 | 방향 | 의미 |
|---|---|---|
| `B130` | PLC 120 → Vision | 비전 검사 시작 트리거 (공정 B → C B디바이스 매핑) |
| `M250` | Vision → PLC 120 | 양품 통보 |
| `M260` | Vision → PLC 120 | 불량 통보 |
| `M1120` | PLC 160 → Vision | 공정 C 종료 신호 (관제 PLC 에서 통합 폴링) |

## 4. ⭐ Voting 다수결 (튀는 값 방지)

```
B130 ON (상승 엣지)
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
B130 OFF → ON 들어오면 다시 사이클 시작
```

### 주요 상수

| 상수 | 값 | 의미 |
|---|---|---|
| `IGNORE_DURATION` | `0.4` | B130 ON 직후 무시 시간 (s) |
| `VOTING_DURATION` | `1.5` | voting 윈도우 (s) |
| `INITIAL_CONF_THRESHOLD` | `0.50` | YOLO conf 초기값 (실행 중 트랙바로 조정) |

## 5. 동작 흐름

```
   [공정 B 종료 + 컨베어 운반]
        │
        ▼ B130 ON
   PLC 120 트리거 수신
                                          ┌────────────────────────────┐
                                          │  Vision PC                 │
                                          │  1) B130 상승 엣지 감지    │
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
MODEL_PATH = r"C:\Users\user\Desktop\intel_cam_prj\intel_cam\intel_cam\models\6class_best.pt"
```

→ 이 절대 경로는 **개발 PC 전용**. 다른 PC 에 배포 시 경로 수정 또는 모델 파일 복사 필요.

### E. "M250/M260 PLC 측에서 안 보임"
- `test_plc_signal.py` 의 sweep 모드 (`s`) 로 M250 쓰기 가능한지 확인
- PLC 래더가 M250 OUT 으로 덮어쓰는지 확인 (그러면 다른 자유 M 비트로 변경)

## 8. 키 매뉴얼 (cv2 창에서)

| 키 | 동작 |
|---|---|
| **V** | 비전 검사 수동 실행 (PLC 우회, 디버그) |
| **1** | B130 ON (PLC B 트리거 흉내, 검출 폴링 시작) |
| **0** | B130 OFF (재트리거 준비) |
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

## 10. 관련 파일

- [C_Process_pendant.py](C_Process_pendant.py) — 본 코드
- [B_Process_pendant.py](B_Process_pendant.py), [B_Process_디버깅.md](B_Process_디버깅.md) — 인접 공정 (양품/불량 분기 다름)
- [A_Process_pendant.py](A_Process_pendant.py) — 시작 공정 (QR 스캔)
- [../db/schema.sql](../db/schema.sql), [../db/SETUP.md](../db/SETUP.md)
