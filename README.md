# FAiCTORY MES

부산 인텔 DX 부트캠프 — Indy7 협동로봇 + 미쓰비시 PLC + YOLO 비전 검사 기반 3공정 스마트팩토리 MES.

## 📊 시스템 구조

```
공정 A (PLC 150)           공정 B (PLC 140 + 130)         공정 C (PLC 120 + 110)
 ─ QR 스캐너               ─ YOLO 비전 검사              ─ YOLO 비전 검사
 ─ 차종 트리거             ─ 양품/불량 분기              ─ 양품/불량 + 종합 처리
       │                          │                            │
       └──────────────────────────┴────────────────────────────┘
                                  │
                          ┌───────┴────────┐
                          │  PLC 160 (관제) │  ← 종료 신호 통합
                          │  M1150 / M1130  │
                          │  M1120          │
                          └───────┬────────┘
                                  │
                          ┌───────┴────────┐
                          │   MySQL DB     │
                          │   faictory_mes │
                          └────────────────┘
```

각 공정 PC는 **자기 공정 PLC** (트리거/통보) + **관제 PLC 160** (종료 신호 폴링) 두 개에 접속.

## 📂 폴더 구조

```
factory_mes/
├── requirements.txt
├── db/
│   └── schema.sql                   # MySQL 스키마 (DDL)
└── mes/
    ├── A_Process_pendant.py         # 공정 A — QR + 차종 트리거
    ├── B_Process_pendant.py         # 공정 B — 비전 검사
    └── C_Process_pendant.py         # 공정 C — 비전 검사 + 종합 처리
```

## 🚀 설치 + 실행

### 1. 의존성 설치
```powershell
pip install -r requirements.txt
```

### 2. DB 초기화 (최초 1회)
```powershell
mysql -u root -p faictory_mes < db/schema.sql
```

### 3. 각 공정 PC에서 실행
```powershell
# 공정 A
python mes/A_Process_pendant.py

# 공정 B
python mes/B_Process_pendant.py

# 공정 C
python mes/C_Process_pendant.py
```

## ⌨️ 키 매뉴얼 (B, C 공정)

| 키 | 동작 |
|---|---|
| **V** | 비전 검사 수동 실행 (PLC 우회) |
| **1** | 비전 트리거 ON (PLC 자동 우회 디버그) |
| **0** | 비전 트리거 OFF (재트리거 준비) |
| **S** | ACTIVE 트레이 조회 |
| **Q** | 종료 |

## 🔌 연결 정보

| 항목 | 주소 |
|---|---|
| DB 서버 | `192.168.3.45:3306` / guest / guest1234 / `faictory_mes` |
| 공정 A PLC | `192.168.3.150:2000` |
| 공정 B PLC | `192.168.3.140:2000` |
| 공정 C PLC | `192.168.3.120:2000` |
| 관제 PLC 160 | `192.168.3.160:2000` |

DB 셋업 + 계정 권한 부여는 [db/SETUP.md](db/SETUP.md) 참조.

## 🎯 비전 검사 — 3초 voting

```
트리거 ON → 매 프레임 YOLO 추론 → 첫 검출부터 3초 누적
            → 다수결 채택 (예: r_normal 82회 / r_crack 5회 → r_normal)
            → PLC 양품/불량 통보 + DB INSERT
```

- 모델: `C_VISION.pt` (별도 제공, 6 클래스: r/g/b_normal · r/g/b_crack)
- 임계값: ultralytics 기본 `conf=0.25`

## 🗃 DB 핵심 테이블

| 테이블 | 용도 |
|---|---|
| `tbl_carrier_map` | 트레이 ↔ product_sn (ACTIVE/RELEASED) |
| `tbl_robot_a/b/c` | 공정별 결과 이력 |
| `tbl_total` | 3공정 종합 (NG 하나라도면 NG) |
| `tbl_tray_master` | 트레이 마스터 |
