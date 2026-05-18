-- ═══════════════════════════════════════════════════════════════════════
-- FAiCTORY MES — Database Schema
-- ═══════════════════════════════════════════════════════════════════════
-- Database: faictory_mes
-- Charset:  utf8mb4 / utf8mb4_0900_ai_ci
-- Exported: mysqldump (MySQL 8.0), structure only (no data)
-- ═══════════════════════════════════════════════════════════════════════
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `robot_realtime_status` (
  `robot_id` varchar(50) NOT NULL,
  `j1_deg` float DEFAULT NULL,
  `j2_deg` float DEFAULT NULL,
  `j3_deg` float DEFAULT NULL,
  `j4_deg` float DEFAULT NULL,
  `j5_deg` float DEFAULT NULL,
  `j6_deg` float DEFAULT NULL,
  `t1_nm` float DEFAULT NULL,
  `t2_nm` float DEFAULT NULL,
  `t3_nm` float DEFAULT NULL,
  `t4_nm` float DEFAULT NULL,
  `t5_nm` float DEFAULT NULL,
  `t6_nm` float DEFAULT NULL,
  `updated_at` datetime DEFAULT NULL,
  PRIMARY KEY (`robot_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `robot_task_history` (
  `id` bigint NOT NULL AUTO_INCREMENT,
  `robot_id` varchar(50) NOT NULL,
  `action_type` varchar(20) DEFAULT NULL,
  `pos_x` float DEFAULT NULL,
  `pos_y` float DEFAULT NULL,
  `pos_z` float DEFAULT NULL,
  `completed_at` datetime DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `idx_robot_time` (`robot_id`,`completed_at`),
  KEY `idx_completed` (`completed_at`)
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `tbl_carrier_map` (
  `id` int NOT NULL AUTO_INCREMENT,
  `tray_id` varchar(10) NOT NULL,
  `product_sn` varchar(20) DEFAULT NULL,
  `status` varchar(20) DEFAULT 'ACTIVE',
  `bound_at` datetime DEFAULT CURRENT_TIMESTAMP,
  `released_at` datetime DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `idx_tray_status` (`tray_id`,`status`)
) ENGINE=InnoDB AUTO_INCREMENT=15 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `tbl_robot_a` (
  `num` int NOT NULL AUTO_INCREMENT,
  `product_sn` varchar(20) NOT NULL,
  `machine_name` varchar(30) DEFAULT 'RobotA_Indy7',
  `recorded_at` datetime NOT NULL,
  `tray_sn` varchar(10) DEFAULT NULL,
  `vision_result` varchar(5) NOT NULL,
  `defect_type` varchar(30) DEFAULT NULL,
  PRIMARY KEY (`num`),
  KEY `idx_sn` (`product_sn`),
  KEY `idx_tray` (`tray_sn`),
  KEY `idx_date` (`recorded_at`)
) ENGINE=InnoDB AUTO_INCREMENT=2 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `tbl_robot_b` (
  `num` int NOT NULL AUTO_INCREMENT,
  `product_sn` varchar(20) NOT NULL,
  `machine_name` varchar(30) DEFAULT 'RobotB_Indy7',
  `recorded_at` datetime NOT NULL,
  `tray_sn` varchar(10) DEFAULT NULL,
  `a1` float DEFAULT NULL,
  `a2` float DEFAULT NULL,
  `a3` float DEFAULT NULL,
  `a4` float DEFAULT NULL,
  `a5` float DEFAULT NULL,
  `a6` float DEFAULT NULL,
  `vision_result` varchar(5) NOT NULL,
  `defect_type` varchar(30) DEFAULT NULL,
  PRIMARY KEY (`num`),
  KEY `idx_sn` (`product_sn`),
  KEY `idx_tray` (`tray_sn`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `tbl_robot_c` (
  `num` int NOT NULL AUTO_INCREMENT,
  `product_sn` varchar(20) NOT NULL,
  `machine_name` varchar(30) DEFAULT 'RobotC_Indy7',
  `recorded_at` datetime NOT NULL,
  `tray_sn` varchar(10) DEFAULT NULL,
  `vision_result` varchar(5) NOT NULL,
  `box_sn` varchar(15) DEFAULT NULL,
  `defect_type` varchar(30) DEFAULT NULL,
  PRIMARY KEY (`num`),
  KEY `idx_sn` (`product_sn`),
  KEY `idx_tray` (`tray_sn`)
) ENGINE=InnoDB AUTO_INCREMENT=18 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `tbl_total` (
  `num` int NOT NULL AUTO_INCREMENT,
  `product_sn` varchar(20) NOT NULL,
  `process_a` varchar(5) DEFAULT NULL,
  `process_b` varchar(5) DEFAULT NULL,
  `process_c` varchar(5) DEFAULT NULL,
  `final_result` varchar(5) DEFAULT NULL,
  `completed_at` datetime DEFAULT NULL,
  PRIMARY KEY (`num`),
  UNIQUE KEY `product_sn` (`product_sn`),
  KEY `idx_date` (`completed_at`)
) ENGINE=InnoDB AUTO_INCREMENT=3 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `tbl_tray_master` (
  `tray_id` varchar(10) NOT NULL,
  `tray_type` varchar(20) NOT NULL,
  `status` varchar(20) DEFAULT 'IN_USE',
  `registered_at` datetime DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`tray_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

