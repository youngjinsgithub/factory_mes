# 공정 A — 디버깅 가이드

`A_Process_pendant.py` 실행 중 자주 만나는 에러와 원인/해결 정리.

## 1. 통신 토폴로지

```
[이 PC]  ──MC Protocol(TCP/2000)──▶  PLC 150 (192.168.3.150)   ← 차종 트리거 쓰기
        ──MC Protocol(TCP/2000)──▶  PLC 160 (192.168.3.160)   ← M1150 종료 신호 읽기
        ──MySQL(TCP/3306)───────▶   DB     (192.168.3.141)
```

각 라인이 끊기면 증상이 다르게 나옵니다 — 아래 표 참고.

---

## 2. 자주 보는 에러와 원인

### 2-1. `PLC 전송 실패: [WinError 10054]`

| 항목 | 내용 |
|---|---|
| 의미 | TCP 소켓이 원격에서 RST로 강제 종료됨 |
| 직접 원인 | PLC가 **idle 소켓**을 끊음 (Open Setting Existence Check 타임아웃) |
| 또는 | 이전 비정상 종료된 Python 세션이 PLC 슬롯에 좀비로 남음 |
| 해결 | ① 모든 Python 프로세스 종료 (`Q`키로 깔끔히) <br> ② 30~60초 대기 (PLC 좀비 슬롯 회수) <br> ③ 재실행 <br> ④ 그래도 안 되면 GX Works → Ethernet 진단 → 해당 행 "선택 행 강제 무효화" |

### 2-2. `PLC 전송 실패: timed out`

| 항목 | 내용 |
|---|---|
| 의미 | TCP는 붙었는데 MC Protocol 응답이 안 옴 |
| 진단 시작점 | `✅ PLC 연결` 메시지는 **TCP 3-way 성공만 의미**. 실 데이터 통신 보장 아님 |
| 원인 후보 (확률 순) | ① **Open Setting의 교신 상대 IP가 다른 PC로 잠김** (이 PC IP는 `ipconfig`로 확인) <br> ② 동시 접속 슬롯 소진 <br> ③ 교신 데이터 코드 미스매치 (Binary ↔ ASCII) <br> ④ PLC가 STOP 모드 <br> ⑤ Open Setting 자체가 없음 |
| 진단 방법 | GX Works → 진단 → Ethernet 진단 → **커넥션별 상태** 탭에서 이 PC IP 행의 "최신 에러 코드" 확인 (예: `4183` = 요청 거부 응답) |

### 2-3. `관제 PLC 연결 실패 — 종료 폴링 비활성 (timed out)`

| 항목 | 내용 |
|---|---|
| 의미 | `192.168.3.160:2000` MC Protocol TCP 자체 실패 |
| 원인 | PLC 160에는 MC Protocol Open Setting이 없거나 다른 포트로 설정됨 |
| 영향 | M1150 (공정 A 종료) 자동 감지가 동작 안 함. QR 적재는 정상 |
| 해결 | PLC 160의 Open Setting에 MC Protocol/TCP/2000 슬롯 추가 → 파라미터 PLC로 쓰기 → 리셋 |

### 2-4. `Table 'faictory_mes.tbl_robot_a' doesn't exist`

| 항목 | 내용 |
|---|---|
| 의미 | 코드가 참조한 테이블이 DB에 없음 |
| 원인 | `schema.sql`에는 `tbl_robot_a`로 정의돼 있으나, 실제 운영 DB는 **`A_Process`** 테이블 사용 |
| 해결 | `record_robot_a_result()`의 INSERT 테이블명을 `A_Process`로 (현재 코드 적용됨) |

### 2-5. `⚠ 미등록 Tray`

| 항목 | 내용 |
|---|---|
| 의미 | QR로 읽은 텍스트가 `tbl_tray_master`에 없음 |
| 해결 | DB에 트레이 행 추가 (아래 5절 참고) |

### 2-6. `⚠ 이미 사용 중: ...`

| 항목 | 내용 |
|---|---|
| 의미 | 해당 트레이에 ACTIVE 매핑이 이미 존재 (이전 사이클이 RELEASED 안 됨) |
| 정상 흐름 | PLC 160의 M1150 신호 → 종료 처리 시 자동 해제 |
| 수동 해제 | `R` (가장 최근 1건) 또는 `A` (전체 ACTIVE) 키 |

---

## 3. GX Works Ethernet 진단 읽는 법

진단 화면 (PLC 150 예시) → **커넥션별 상태** 탭:

| 컬럼 | 의미 |
|---|---|
| 자국 포트 번호 | PLC가 listen 중인 포트. MC Protocol이면 보통 2000 |
| 교신 상대 IP | **누가 이 슬롯에 붙어 있는지** (← 본인 PC IP 확인 핵심) |
| 프로토콜 | MELSOFT / MC 프로토콜 / FTP 서버 등 |
| TCP 상태 | 접속 중 / 절단 |
| **최신 에러 코드** | **빈 칸이 정상**. 값이 있으면 마지막 요청에서 PLC가 에러 응답함 |
| 해제 이상 횟수 | 비정상 끊김 누적 |

**본인 IP 확인** — PowerShell에서:

```powershell
Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.IPAddress -like "192.168.3.*" } | Select-Object IPAddress, InterfaceAlias
```

이 IP가 진단 화면의 MC 프로토콜 행에 안 보이면, **본인 PC는 슬롯에 들어가지도 못한 상태**입니다.

---

## 4. PLC 비트 매핑 (공정 A)

| 비트 | 방향 | 의미 |
|---|---|---|
| `M250` | Vision → PLC 150 | RED 차종 시작 펄스 (1초) |
| `M260` | Vision → PLC 150 | BLUE 차종 시작 펄스 (1초) |
| `M270` | Vision → PLC 150 | GREEN 차종 시작 펄스 (1초) |
| `M1150` | PLC 160 → Vision | 공정 A 종료 신호 (상승 엣지 감지) |

---

## 5. 트레이 / 차종 추가 절차

### 5-1. 같은 차종의 새 트레이 (코드 수정 불필요)

```python
import pymysql
c = pymysql.connect(host='192.168.3.141', port=3306, user='guest', password='guest1234',
                    db='faictory_mes', charset='utf8mb4', autocommit=True)
c.cursor().execute(
    "INSERT INTO tbl_tray_master (tray_id, tray_type, status) "
    "VALUES ('T-0004', 'RED', 'IN_USE')")
c.close()
```

### 5-2. 새로운 차종 (코드 + DB)

예: BLACK 차종을 `M280`으로 추가하려면 `A_Process_pendant.py`에서:

| 위치 | 변경 |
|---|---|
| `ADDR_RED`/`BLUE`/`GREEN` 인근 | `ADDR_BLACK = "M280"` 추가 |
| `generate_product_sn()` 의 type_code dict | `'BLACK': 'BK'` 추가 |
| `trigger_plc()` 의 addr dict | `'BLACK': ADDR_BLACK` 추가 |
| 시작 안내 print | M280 라인 추가 |

그리고 DB에 트레이 1행 INSERT (tray_type='BLACK').

---

## 6. 현재 등록된 트레이 (참고)

```powershell
factory_mes\.venv\Scripts\python.exe -c "import pymysql; c=pymysql.connect(host='192.168.3.141',port=3306,user='guest',password='guest1234',db='faictory_mes',charset='utf8mb4'); cur=c.cursor(); cur.execute('SELECT * FROM tbl_tray_master ORDER BY tray_id'); [print(r) for r in cur.fetchall()]; c.close()"
```

| tray_id (=QR 텍스트) | tray_type | PLC 비트 | S/N 접두 |
|---|---|---|---|
| `T-0001` | RED | M250 | `YYMMDD-RD-NNNN` |
| `T-0002` | BLUE | M260 | `YYMMDD-BL-NNNN` |
| `T-0003` | GREEN | M270 | `YYMMDD-GN-NNNN` |

> QR 텍스트 = `tray_id`. 코드에는 tray_id가 어디에도 하드코딩돼 있지 않고, 모든 매핑은 `tbl_tray_master` DB 한 곳에 있음.

---

## 7. 종료 시 주의

`Q` 키로 종료해야 `plc.close()`, `plc_monitor.close()`가 호출되어 PLC 슬롯이 깔끔하게 회수됩니다. **창 X 버튼 / Ctrl+C / 강제 종료는 좀비 세션을 남길 수 있음** — 다음 실행 시 10054 또는 timeout으로 나타남.
