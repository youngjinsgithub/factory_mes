# DB 설정 가이드

## 1. 현재 운영 환경

| 항목 | 값 |
|---|---|
| Host | `192.168.3.45` |
| Port | `3306` |
| User | `guest` |
| Password | `guest1234` |
| Database | `faictory_mes` |

각 펜던트 파일의 `DB_CONFIG`에 이 값들이 이미 적용되어 있습니다.

---

## 2. 서버에 처음 셋업하는 경우

### 2-1. DB + 테이블 생성

```bash
mysql -u root -p < schema.sql
```

`faictory_mes` DB가 생성되고 테이블 + 시드 데이터(T-0001/T-0002)가 입력됩니다.

### 2-2. guest 계정 생성 + 권한 부여

서버 MySQL에 **root로 접속**해서:

```sql
-- guest 계정 생성 (어디서든 접속 가능)
CREATE USER 'guest'@'%' IDENTIFIED BY 'guest1234';

-- 또는 공정 PC IP만 허용 (보안 강화)
-- CREATE USER 'guest'@'192.168.3.50' IDENTIFIED BY 'guest1234';

-- 필요한 권한만 부여 (DROP/CREATE 같은 위험 권한 제외)
GRANT SELECT, INSERT, UPDATE, DELETE ON faictory_mes.* TO 'guest'@'%';
FLUSH PRIVILEGES;

-- 확인
SHOW GRANTS FOR 'guest'@'%';
```

### 2-3. 접속 테스트

```bash
mysql -h 192.168.3.45 -u guest -pguest1234 faictory_mes -e "SHOW TABLES;"
```

위 명령으로 테이블 6개(`tbl_carrier_map`, `tbl_robot_a/b/c`, `tbl_tray_master`, `tbl_total`) 보이면 OK.

---

## 3. 노트북에 남아있던 guest 계정 정리 (선택)

프로토타입 단계에서 노트북 MySQL에 만들었던 guest 계정이 있다면 정리:

```sql
-- 노트북 MySQL 에 root 로 접속

-- 확인
SELECT user, host FROM mysql.user WHERE user = 'guest';

-- 삭제
DROP USER IF EXISTS 'guest'@'localhost';
DROP USER IF EXISTS 'guest'@'%';
DROP USER IF EXISTS 'guest'@'127.0.0.1';

FLUSH PRIVILEGES;

-- 0행 확인
SELECT user, host FROM mysql.user WHERE user = 'guest';
```

---

## 4. 보안 강화 (권장)

현재 `guest / guest1234`는 시연용. 운영 안정화 단계에서는 다음 권장:

### 비밀번호 환경변수로 분리

```python
import os
DB_CONFIG = {
    'host': os.environ.get('FAICTORY_DB_HOST', '192.168.3.45'),
    'user': os.environ.get('FAICTORY_DB_USER', 'guest'),
    'password': os.environ['FAICTORY_DB_PASSWORD'],   # 필수
    'db': 'faictory_mes',
    ...
}
```

실행:

```powershell
# PowerShell
$env:FAICTORY_DB_PASSWORD = "guest1234"
python mes/C_Process_pendant.py
```

```bash
# Bash
export FAICTORY_DB_PASSWORD=guest1234
python mes/C_Process_pendant.py
```

비밀번호가 git 히스토리에 남지 않고 PC마다 다른 값 설정 가능.

### 또는 더 강한 비밀번호로 교체

```sql
ALTER USER 'guest'@'%' IDENTIFIED BY '새로운_강한_비밀번호';
FLUSH PRIVILEGES;
```

코드의 `DB_CONFIG['password']` 또는 환경변수도 같이 업데이트.
