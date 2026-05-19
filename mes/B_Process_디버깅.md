# B_Process_pendant.py — 공정 B (비전 검사 + 중간 공정)

## 1. 역할

공정 B Vision PC 가 담당하는 일:
- 공정 A 에서 넘어온 트레이의 비전 검사 (YOLO, 3초 voting)
- 양품/불량 판정 → PLC 140 에 M250/M260 통보
- `tbl_robot_b` 에 검사 결과 INSERT
- **불량 시** → 공정 C 진행 불가하므로 B 단에서 `tbl_total` + `carrier_map RELEASE` 직접 처리
- 양품 시 → 공정 C 가 최종 종합 처리

## 2. 연결 구성

| 항목 | 주소 |
|---|---|
| DB 서버 | `192.168.3.141:3306` / `guest` / `guest1234` |
| 공정 B PLC | `192.168.3.140:2000` (트리거 수신/양품·불량 통보) |
| 관제 PLC 160 | `192.168.3.160:2000` (M1130 종료 신호 폴링) |
| 카메라 | 비전 검사 (인덱스 1) |
| YOLO 모델 | `C_VISION.pt` (6 클래스: r/g/b_normal · r/g/b_crack) |

## 3. PLC 비트 매핑

| 비트 | 방향 | 의미 |
|---|---|---|
| `B150` | PLC 140 → Vision | 비전 검사 시작 트리거 (공정 A → B B디바이스 매핑) |
| `M250` | Vision → PLC 140 | 양품 통보 |
| `M260` | Vision → PLC 140 | 불량 통보 |
| `M1130` | PLC 160 → Vision | 공정 B 종료 신호 (관제 PLC 에서 통합 폴링) |

## 4. 동작 흐름

```
   [공정 A 종료]                           [공정 B 시작]
        │                                       │
        ▼ B150 ON                                │
   PLC 140 트리거 수신 ──────────────────────────▶
                                          ┌────────────────────────────┐
                                          │  Vision PC                 │
                                          │  1) B150 상승 엣지 감지    │
                                          │  2) 검출 폴링 (3초 voting) │
                                          │  3) 양품/불량 판정         │
                                          │  4) PLC 140 통보 (M250/M260)│
                                          │  5) 결과 메모리 저장        │
                                          └────────────────────────────┘
                                                       │
                                          [PLC 작업 진행 + 컨베어]
                                                       │
        ▼ M1130 ON (PLC 160 의 관제 비트)              │
                                          ┌────────────────────────────┐
                                          │  Vision PC                 │
                                          │  1) M1130 상승 엣지 감지   │
                                          │  2) tbl_robot_b INSERT     │
                                          │     (메모리 결과 사용)     │
                                          │  3) 양품/불량 분기 ↓       │
                                          └────────────────────────────┘
                                          ┌───────────┐  ┌─────────────┐
                                          │  양품 →   │  │  불량 →     │
                                          │  공정 C 로│  │  B 에서     │
                                          │  넘어감   │  │  tbl_total +│
                                          │  (C가 종합)│  │  RELEASE    │
                                          └───────────┘  └─────────────┘
```

## 5. ⭐ 양품 / 불량 분기 (핵심)

### 양품 (vision_result = 'OK')

```
M1130 감지
  → tbl_robot_b INSERT (OK)
  → 메시지: "양품 — 공정 C 로 진행"
  → (트레이는 ACTIVE 그대로 유지)
  → 컨베어가 트레이를 공정 C 로 운반
  → 공정 C 가 끝나면 C 의 M1120 이 ON → C 가 tbl_total + RELEASE
```

### 불량 (vision_result = 'NG')

```
M1130 감지
  → tbl_robot_b INSERT (NG, defect_type=r_crack 등)
  → 메시지: "불량 판정 — 공정 C 미진행 → B 에서 종합 처리"
  → finalize_defect_at_b() 호출:
      ├─ tbl_robot_a 결과 조회 (있으면)
      ├─ tbl_robot_b 결과 = 방금 INSERT 한 NG
      ├─ tbl_robot_c = NULL (안 거침)
      ├─ final = NG (B 가 NG 라 자동 NG)
      ├─ tbl_total UPSERT
      └─ carrier_map status='RELEASED'
  → 트레이가 ACTIVE 에서 빠짐
```

## 6. 트러블슈팅 — "트레이가 영원히 ACTIVE 로 묶임"

### 증상
`tbl_carrier_map` 에 ACTIVE 상태 트레이가 누적되고 RELEASED 안 됨.

### 원인 분류

| 원인 | 진단 방법 |
|---|---|
| B 불량 후 finalize_defect_at_b 미실행 | 콘솔에서 "B 에서 종합 처리" 메시지 떴는지 확인 |
| DB INSERT/UPDATE 권한 부족 | `SHOW GRANTS FOR CURRENT_USER` 로 UPDATE 권한 확인 |
| 양품인데 공정 C 가 비전 검사 미실시 | C_Process_pendant.py 가 떠있는지 + B130 신호 들어오는지 |
| M1130 신호 자체가 안 옴 | test_plc_signal.py 로 PLC 160 의 M1130 직접 확인 |

### 수동 RELEASE
```python
# 콘솔 `S` 키 — 현재 ACTIVE 트레이 조회
# 코드 안에는 강제 RELEASE 헬퍼 없음 (A_Process_pendant.py 의 `R`/`A` 키 또는 직접 SQL)
```

```sql
-- 직접 SQL 로 RELEASE
UPDATE tbl_carrier_map
SET status='RELEASED', released_at=NOW()
WHERE product_sn = 'PROD_XXX' AND status='ACTIVE';
```

## 7. 키 매뉴얼 (cv2 창에서)

| 키 | 동작 |
|---|---|
| **V** | 비전 검사 수동 실행 (PLC 우회, 디버그) |
| **1** | B150 ON (PLC A 트리거 흉내, 검출 폴링 시작) |
| **0** | B150 OFF (재트리거 준비) |
| **S** | 현재 ACTIVE 트레이 조회 |
| **Q** | 종료 |

## 8. 실행

```powershell
conda activate robot
cd MES
python B_Process_pendant.py
```

부팅 시 콘솔 출력 예시:
```
✅ YOLO 모델 로드 완료 (C_VISION.pt)
✅ PLC 연결 (192.168.3.140:2000) — 공정 B 메인
✅ 관제 PLC 연결 (192.168.3.160:2000) — 종료 신호 폴링
...
공정 B — 비전 검사 + 중간 공정 (펜던트 IF 방식)
```

## 9. 관련 파일

- [B_Process_pendant.py](B_Process_pendant.py) — 본 코드
- [../db/schema.sql](../db/schema.sql) — DB 스키마
- [../db/SETUP.md](../db/SETUP.md) — DB 셋업 가이드
- [A_Process_pendant.py](A_Process_pendant.py), [C_Process_pendant.py](C_Process_pendant.py) — 인접 공정 코드
