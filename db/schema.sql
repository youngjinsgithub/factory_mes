-- ═══════════════════════════════════════════════════════
-- FAiCTORY MES — DB 셋업 (지금 단계 최소 버전)
-- ═══════════════════════════════════════════════════════

-- 1. DB
CREATE DATABASE IF NOT EXISTS faictory_mes
  DEFAULT CHARACTER SET utf8mb4;
USE faictory_mes;

-- 2. Tray 마스터
CREATE TABLE tbl_tray_master (
    tray_id VARCHAR(10) PRIMARY KEY,
    tray_type VARCHAR(20) NOT NULL,
    status VARCHAR(20) DEFAULT 'IN_USE',
    registered_at DATETIME DEFAULT CURRENT_TIMESTAMP
);

-- 3. Tray ↔ Product 매핑
CREATE TABLE tbl_carrier_map (
    id INT AUTO_INCREMENT PRIMARY KEY,
    tray_id VARCHAR(10) NOT NULL,
    product_sn VARCHAR(20),
    status VARCHAR(20) DEFAULT 'ACTIVE',
    bound_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    released_at DATETIME NULL,
    INDEX idx_tray_status (tray_id, status)
);

-- 4. 공정 A — 차체검사
CREATE TABLE tbl_robot_a (
    num INT AUTO_INCREMENT PRIMARY KEY,
    product_sn VARCHAR(20) NOT NULL,
    machine_name VARCHAR(30) DEFAULT 'RobotA_Indy7',
    recorded_at DATETIME NOT NULL,
    tray_sn VARCHAR(10),
    vision_result VARCHAR(5) NOT NULL,
    defect_type VARCHAR(30) NULL,
    INDEX idx_sn (product_sn),
    INDEX idx_tray (tray_sn),
    INDEX idx_date (recorded_at)
);

-- 5. 공정 B — 유리검사 + 6축
CREATE TABLE tbl_robot_b (
    num INT AUTO_INCREMENT PRIMARY KEY,
    product_sn VARCHAR(20) NOT NULL,
    machine_name VARCHAR(30) DEFAULT 'RobotB_Indy7',
    recorded_at DATETIME NOT NULL,
    tray_sn VARCHAR(10),
    a1 FLOAT, a2 FLOAT, a3 FLOAT,
    a4 FLOAT, a5 FLOAT, a6 FLOAT,
    vision_result VARCHAR(5) NOT NULL,
    defect_type VARCHAR(30) NULL,
    INDEX idx_sn (product_sn),
    INDEX idx_tray (tray_sn)
);

-- 6. 공정 C — 분류적재
CREATE TABLE tbl_robot_c (
    num INT AUTO_INCREMENT PRIMARY KEY,
    product_sn VARCHAR(20) NOT NULL,
    machine_name VARCHAR(30) DEFAULT 'RobotC_Indy7',
    recorded_at DATETIME NOT NULL,
    tray_sn VARCHAR(10),
    vision_result VARCHAR(5) NOT NULL,
    box_sn VARCHAR(15),
    defect_type VARCHAR(30) NULL,
    INDEX idx_sn (product_sn),
    INDEX idx_tray (tray_sn)
);

-- 7. 종합 집계
CREATE TABLE tbl_total (
    num INT AUTO_INCREMENT PRIMARY KEY,
    product_sn VARCHAR(20) NOT NULL UNIQUE,
    process_a VARCHAR(5),
    process_b VARCHAR(5),
    process_c VARCHAR(5),
    final_result VARCHAR(5),
    completed_at DATETIME,
    INDEX idx_date (completed_at)
);

-- 8. 시드 — Tray 2개 등록
INSERT INTO tbl_tray_master (tray_id, tray_type) VALUES
('T-0001', 'RED'),
('T-0002', 'BLUE');

-- 9. 확인
SHOW TABLES;
SELECT * FROM tbl_tray_master;
