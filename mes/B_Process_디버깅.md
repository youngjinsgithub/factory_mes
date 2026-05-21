# B_Process_pendant.py — 공정 B (비전 검사 + 중간 공정)

## 1. 역할

공정 B Vision PC 가 담당하는 일:
- M101 ON → 비전 검사 자동 실행 (YOLO + 1.5s voting 다수결, GPU 추론)
- 양품/불량 판정 → PLC 140 에 M250/M260 통보
- **voting 완료 직후** `B_Process` 에 검사 결과 INSERT (공정 C 와 동일 패턴)
- **불량 시** → 공정 C 진행 불가하므로 B 단에서 `tbl_total` + `carrier_map RELEASE` 직접 처리
- 양품 시 → 공정 C 가 최종 종합 처리
- M101 OFF 시 voting 못 끝낸 채 종료 / 타임아웃 / 알 수 없는 클래스 → 검출 실패로 간주 → 자동 release

## 2. 연결 구성

| 항목 | 주소 / 값 |
|---|---|
| DB 서버 | `192.168.3.141:3306` / `guest` / `guest1234` / DB `faictory_mes` |
| 공정 B PLC | `192.168.3.140:2000` (M101 트리거 수신 + M250/M260 통보) |
| 관제 PLC 160 | `192.168.3.160:2000` (M1130 종료 신호 — 현재 로그용) |
| 카메라 | `cv2.VideoCapture(index=0, CAP_DSHOW)` — MJPG 지원 시 1280x720, 미지원 시 640x480 YUY2 로 폴백 |
| 카메라 회전 | **None** (모델이 회전 없는 원본 영상으로 학습됨 — 회전 적용 시 검출 실패) |
| YOLO 모델 | `B_Vision.pt` (프로젝트 루트, **3 클래스**: `b`, `g`, `r`, 차체조립 검사용) |
| Python 환경 | `.venv39` (Python 3.9.13 + torch 1.10.2+cu102, GPU 사용) |

## 3. PLC 비트 매핑

| 비트 | PLC | 방향 | 의미 |
|---|---|---|---|
| `M101` | 140 | → Vision | 비전 검사 시작 트리거 = 부품 도착 신호 (같은 비트 통합) |
| `M250` | 140 | Vision → | 양품 통보 (1초 펄스) |
| `M260` | 140 | Vision → | 불량 통보 (1초 펄스) |
| `M1130` | 160 | → Vision | 공정 B 종료 신호 — 현재 로그 출력만, DB INSERT 는 voting 시점에 이미 처리됨 |

## 4. ⭐ Voting 다수결 + 검출 실패 안전망 (이번 세션 핵심)

```
M101 ON (상승 엣지)
  ↓
[IGNORE_DURATION = 0.4s 안정화 대기]
  ↓
매 프레임 YOLO 추론 (GPU 약 15ms/프레임)
  ↓
첫 검출 발생 → voting 시작
  ↓
[VOTING_DURATION = 1.5s 동안 결과 누적]
  ↓
voting 완료 → 다수결 클래스 채택
  ├─ 채택 클래스 ∈ NORMAL_CLASSES (b, r)  → 양품 → M250 + B_Process INSERT(OK)
  ├─ 채택 클래스 ∈ CRACK_CLASSES (g)      → 불량 → M260 + B_Process INSERT(NG, defect_type='g') + finalize_defect_at_b (RELEASE)
  └─ 알 수 없는 클래스                    → 검출 실패 → M260 + INSERT(NG, 'no_detect') + finalize_defect_at_b
```

### 검출 실패 통합 — `_commit_detection_failure(reason)`

세 가지 케이스 모두 같은 NG + RELEASE 경로:
1. M101 ON 후 `DEFECT_TIMEOUT = 3.0s` 안에 양품 voting 시작도 못 함 (`no_detect` 타임아웃)
2. voting 완료했는데 채택 클래스가 NORMAL/CRACK 어디에도 없음
3. M101 OFF 됐는데 voting 못 끝낸 채 사이클 종료

→ 모두 `M260 펄스 + B_Process INSERT(NG, 'no_detect') + finalize_defect_at_b` 자동 실행 → carrier_map RELEASE 됨.

### 주요 상수

| 상수 | 값 | 의미 |
|---|---|---|
| `IGNORE_DURATION` | `0.4` | M101 ON 직후 안정화 대기 (s) |
| `VOTING_DURATION` | `1.5` | voting 누적 윈도우 (s) |
| `DEFECT_TIMEOUT` | `3.0` | M101 ON 후 양품 미판정 시 자동 NG (s) |
| `INITIAL_CONF_THRESHOLD` | `0.80` | YOLO conf 초기값 (트랙바로 실시간 조정) |
| `NORMAL_CLASSES` | `['b', 'r']` | 정상 차종 → 양품 |
| `CRACK_CLASSES` | `['g']` | 불량 차종 → M260 + RELEASE |
| `DEFECT_CLASS_NAME` | `'no_detect'` | 검출 실패 시 defect_type 값 |
| `CAMERA_INDEX` | `0` | cv2.VideoCapture index |
| `CAMERA_ROTATION` | `None` | **회전 적용 금지** — 모델 학습 방향과 안 맞음 |

## 5. DB 적재 타이밍 (공정 C 와 동일)

```
voting 완료 (또는 검출 실패)
   ↓
commit_inspection_result() / _commit_detection_failure()
   ├─ PLC M250/M260 펄스
   ├─ B_Process INSERT (즉시)         ← C 공정과 동일 패턴
   └─ NG 면 → finalize_defect_at_b 까지 (tbl_total + carrier_map RELEASE 즉시)

(나중에)
M1130 ON → 로그만 출력, DB 동작 없음 (INSERT 는 이미 voting 시점에 완료)
```

> 이전 버전은 M1130 시점에 INSERT 했었음 → 관제 PLC 160 연결 실패 시 영영 INSERT 안 됨 → C 패턴(즉시 INSERT)으로 통합.

## 6. ⭐ 이번 세션에서 해결한 큰 이슈들

### 6-1. FPS 5fps 정체 → 30fps 회복
근본 원인 3종 콤보:

| 병목 | 원인 | 해결 |
|---|---|---|
| YOLO 추론 200ms | PyTorch 가 CPU 빌드(`2.12.0+cpu`)로 깔려있었음 | **`.venv39` 새로 만들어서 `torch 1.10.2+cu102` 설치** — GTX 1050 + 구형 NVIDIA 드라이버 441.87 호환 |
| 첫 추론 2.6s 멈춤 | CUDA JIT 컴파일 (warmup 필요) | `load_yolo_model()` 에서 dummy 2회 추론 미리 돌려서 warmup |
| 카메라 read 175ms | 1080p YUY2 raw 송출 → USB 2.0 대역폭 한계 | **카메라 캡처 BG 스레드 분리** + 640x480 MJPG 우선 / YUY2 폴백 |

결과: capture 0.2ms / vision 14ms / display 0.5ms / 총 ~30fps 안정

### 6-2. 박스 안 그려지는 이유 — 카메라 회전 보정
- 이전: `CAMERA_ROTATION = cv2.ROTATE_90_COUNTERCLOCKWISE`
- 문제: 모델이 회전 없는 원본으로 학습되어, 회전된 프레임에선 conf 가 거의 0
- 해결: **`CAMERA_ROTATION = None`** — 그대로 YOLO 에 넣어야 검출됨

검증: `vision_test.py` 로 단순 YOLO 카메라 추론 돌렸을 때 `g=0.986` 검출 → 모델은 정상, 회전이 범인이었음.

### 6-3. voting 도중 M101 타임아웃 발동 → 결과 가로채임
- voting 시작했는데 첫 추론이 warmup 으로 2.6s 걸리는 동안 M101 3s 타임아웃 도달
- 해결: voting 시작했으면(`voting_started_at > 0.0`) 타임아웃 무시
- 또 GPU warmup 으로 첫 추론도 15ms 로 단축

### 6-4. 검출 실패 시 carrier_map 영원히 ACTIVE
- 이전: voting 결과가 unknown class 거나 M101 OFF 시 voting 미완료 → 그냥 잠금만, release 안 됨
- 해결: `_commit_detection_failure(reason)` 헬퍼로 세 가지 실패 케이스 통합 처리 → 항상 NG + RELEASE

### 6-5. X 버튼으로 창 닫을 때 traceback
- 원인: `cv2.getTrackbarPos` 가 창 닫힌 후 호출되며 NULL window 에러
- 해결: try/except 가드 + `cv2.getWindowProperty(WND_PROP_VISIBLE) < 1` 체크로 메인 루프 깔끔히 break

## 7. 트러블슈팅

### A. "이 가상환경으로 돌렸는데 ModuleNotFoundError"
프롬프트에 `(.venv39)` 떠있어도 VSCode 가 시스템 Python 경로로 명령어 만들어 실행하면 venv 무시됨.
```powershell
# 반드시 venv Python 경로 명시
.\.venv39\Scripts\python.exe .\factory_mes\mes\B_Process_pendant.py
```
또는 VSCode `Ctrl+Shift+P` → `Python: Select Interpreter` → **`.venv39\Scripts\python.exe`** 선택.

### B. "FPS 가 갑자기 떨어지거나 박스 안 보임"
1. 화면 상단 텍스트 확인:
   - `Standby` → M101 OFF 상태 (정상)
   - `Filtering Initial Data...` → 안정화 0.4s 대기 (정상)
   - `SCANNING...` → 추론 중인데 박스 없으면 모델이 시야 안 인식
2. 트랙바 conf 임계값 낮춰보기 (예: 80 → 20 → 5)
3. `[timing]` 라인 확인 — `vision=14ms` 정상 / `vision=200ms+` 이면 CPU 로 돌고 있음 (venv 잘못 됨)

### C. "카메라 인덱스 0 으로 열기 실패"
이전 인스턴스가 카메라 점유 중일 가능성:
```powershell
taskkill /F /IM python.exe /IM pythonw.exe
```
한 번 친 뒤 재실행. DirectShow 백엔드는 한 프로세스만 카메라 잡을 수 있음.

### D. "트레이가 영원히 ACTIVE 로 묶임"

```sql
SELECT * FROM tbl_carrier_map WHERE status='ACTIVE';
```

| 원인 | 진단 방법 |
|---|---|
| voting OK 였는데 공정 C 가 미진행 | `C_Process_pendant.py` 가 떠있는지 + B130 신호 들어오는지 |
| DB UPDATE 권한 부족 | `SHOW GRANTS FOR CURRENT_USER` — guest 는 DELETE 만 가능 |
| 코드 버그로 finalize 건너뜀 | 콘솔에 `⏰ [검출 실패] ... → 불량` / `⚡ [voting 판정]` 메시지 떴는지 확인 |

수동 정리:
```sql
-- 특정 product 만 release
UPDATE tbl_carrier_map SET status='RELEASED', released_at=NOW()
WHERE product_sn='260520-RD-0001' AND status='ACTIVE';

-- 전체 초기화 (시연 전)
.\.venv39\Scripts\python.exe -X utf8 .\db_clear.py --force
```

### E. "비전 검출 자체가 안 됨 (vision_test.py 로 검증)"
모델 동작 자체를 본 코드 우회로 검증:
```powershell
.\.venv39\Scripts\python.exe -u .\vision_test.py
```
콘솔에 `상위 3: g=0.986` 식으로 conf 출력. 여기서 0 이면 모델이 정말 못 보는 것 → 카메라 시야/조명/모델 문제.

### F. "model 클래스 매핑이 이상함"
`B_Vision.pt` 는 `color` 데이터셋(`/content/machineVisionRobotics/datasets/color/`)으로 학습된 **단색 영역 검출 모델**. 디테일 많은 객체는 conf 떨어짐.

| 단색 색상 | 정상 conf |
|---|---|
| 빨강 | ~0.92 |
| 초록 | ~0.80 |
| 파랑 | ~0.02 (학습 부족 — 시연 시 b 의존 회피 권장) |

`g` 가 잡히면 자동 NG 처리되니까 빨강/파랑 차로 시연하면 양품 라인 검증 가능.

## 8. 키 매뉴얼 (cv2 창에서)

| 키 | 동작 |
|---|---|
| **V** | 비전 검사 수동 실행 (PLC 우회, 디버그) |
| **1** | M101 ON 수동 트리거 (PLC 자체 센서 흉내) |
| **0** | M101 OFF (재트리거 준비) |
| **S** | 현재 ACTIVE 트레이 조회 |
| **Q** | 종료 (X 버튼도 안전하게 종료됨) |
| 트랙바 | conf 임계값 0~100 실시간 조정 |

## 9. 실행

```powershell
# 권장 — venv Python 명시
.\.venv39\Scripts\python.exe .\factory_mes\mes\B_Process_pendant.py

# 콘솔 없이 cv2 창만
.\.venv39\Scripts\pythonw.exe .\factory_mes\mes\B_Process_pendant.py
```

부팅 로그 정상 예시:
```
✅ YOLO 모델 로드 완료 (GPU: GeForce GTX 1050)
✅ GPU warmup 완료 — 첫 사이클부터 fast inference
✅ PLC 연결 (192.168.3.140:2000) — 공정 B 메인
✅ 관제 PLC 연결 (192.168.3.160:2000) — 종료 신호 폴링
✅ PLC 백그라운드 폴링 스레드 시작 (PLC 140 / PLC 160 각 별도, 목표 20Hz)
[INFO] 카메라: 640x480 @ YUY2 (CAP_DSHOW)
✅ 카메라 캡처 백그라운드 스레드 시작 — 메인 루프와 카메라 read 분리
[timing] capture=0.2ms | plc=0.0ms | vision=14ms | display=0.3ms  (프레임 30장)
```

## 10. DB 스키마 변경 (이번 세션 적용)

테이블명 통일:
- `tbl_robot_a` → **`A_Process`**
- `tbl_robot_b` → **`B_Process`**
- `tbl_robot_c` → **`C_Process`**

A/B/C 모두 6축 관절 좌표 `a1, a2, a3, a4, a5, a6` FLOAT 칼럼 포함. M1130 시 Indy7(192.168.3.6) SDK 로 좌표 전송 예정 (SDK 도착 후 작업).

## 11. 관련 파일

- [B_Process_pendant.py](B_Process_pendant.py) — 본 코드
- [B_Process_pendant_PLC140_only.py](B_Process_pendant_PLC140_only.py) — PLC 160 제거 단일 PLC 버전 (예비)
- [../../vision_test.py](../../vision_test.py) — 최소 카메라+YOLO 검증 스크립트
- [../../db_clear.py](../../db_clear.py) — DB 5개 테이블 일괄 초기화 (Python)
- [../db/schema.sql](../db/schema.sql) — DB 스키마 (새 이름 + a1~a6 반영)
- [A_Process_pendant.py](A_Process_pendant.py), [C_Process_pendant.py](C_Process_pendant.py) — 인접 공정
- [C_Process_디버깅.md](C_Process_디버깅.md)
