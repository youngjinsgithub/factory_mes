# DB 설정 가이드

## 1. 초기 셋업 (스키마 적용)

```bash
mysql -u root -p < schema.sql
```

`faictory_mes` DB가 생성되고 테이블 + 시드 데이터(T-0001/T-0002)가 입력됩니다.

---

## 2. 노트북 (프로토타입) → 서버 이전 시

### 2-1. 노트북의 guest 계정 정리

프로토타입 단계에서 만든 `guest` 계정이 남아있으면 정리:

```sql
-- root 로 접속
SELECT user, host FROM mysql.user WHERE user = 'guest';   -- 현재 확인

DROP USER IF EXISTS 'guest'@'localhost';
DROP USER IF EXISTS 'guest'@'%';
DROP USER IF EXISTS 'guest'@'127.0.0.1';

FLUSH PRIVILEGES;
SELECT user, host FROM mysql.user WHERE user = 'guest';   -- 0행이면 완료
```

### 2-2. 서버에 운영용 전용 계정 만들기

`root` 직접 사용은 비추천. 별도 계정을 권장합니다.

```sql
-- 서버 MySQL 에 root 로 접속

-- DB 생성
CREATE DATABASE IF NOT EXISTS faictory_mes DEFAULT CHARACTER SET utf8mb4;

-- 운영용 계정 (모든 호스트 허용)
CREATE USER 'faictory_app'@'%' IDENTIFIED BY '강한_비밀번호';

-- 또는 공정 PC IP 만 허용 (보안 강화)
-- CREATE USER 'faictory_app'@'192.168.3.50' IDENTIFIED BY '...';

-- 필요한 권한만 부여 (DROP/CREATE 같은 위험 권한 제외)
GRANT SELECT, INSERT, UPDATE, DELETE ON faictory_mes.* TO 'faictory_app'@'%';
FLUSH PRIVILEGES;

-- 스키마 적용
USE faictory_mes;
SOURCE /경로/schema.sql;
```

### 2-3. 펜던트 코드의 DB_CONFIG 변경

3개 파일의 `DB_CONFIG` 를 서버 정보로 변경:

- `mes/A_Process_pendant.py`
- `mes/B_Process_pendant.py`
- `mes/C_Process_pendant.py`

```python
# 변경 전 (프로토타입)
DB_CONFIG = {
    'host': 'localhost',
    'user': 'root',
    'password': '1234',
    ...
}

# 변경 후 (서버 운영)
DB_CONFIG = {
    'host': '서버_IP',           # 예: '192.168.3.45'
    'port': 3306,
    'user': 'faictory_app',
    'password': '강한_비밀번호',
    'db': 'faictory_mes',
    'charset': 'utf8mb4',
    'autocommit': True,
}
```

---

## 3. 보안 강화 (권장)

### 비밀번호 환경변수로 분리

코드에 비밀번호 하드코딩 대신 환경변수 사용:

```python
import os
DB_CONFIG = {
    'host': os.environ.get('FAICTORY_DB_HOST', 'localhost'),
    'user': os.environ.get('FAICTORY_DB_USER', 'faictory_app'),
    'password': os.environ['FAICTORY_DB_PASSWORD'],   # 필수
    'db': 'faictory_mes',
    ...
}
```

실행:

```powershell
# PowerShell
$env:FAICTORY_DB_HOST = "192.168.3.45"
$env:FAICTORY_DB_PASSWORD = "강한_비밀번호"
python mes/C_Process_pendant.py
```

```bash
# Bash
export FAICTORY_DB_HOST=192.168.3.45
export FAICTORY_DB_PASSWORD=강한_비밀번호
python mes/C_Process_pendant.py
```

비밀번호가 git 히스토리에 안 남고 PC마다 다른 값 가능.
