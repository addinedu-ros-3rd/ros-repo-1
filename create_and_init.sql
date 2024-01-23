-- MySQL dump 10.13  Distrib 8.0.35, for Linux (x86_64)
--
-- Host: database-1.cktq4evzprzg.ap-northeast-2.rds.amazonaws.com    Database: nursing_home_robot
-- ------------------------------------------------------
-- Server version	8.0.33

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!50503 SET NAMES utf8mb4 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `place`
--

DROP TABLE IF EXISTS `place`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `place` (
  `id` int NOT NULL AUTO_INCREMENT,
  `name` varchar(16) NOT NULL,
  `ko_name` varchar(16) NOT NULL,
  `x` float NOT NULL,
  `y` float NOT NULL,
  `z` float NOT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=11 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `place`
--

LOCK TABLES `place` WRITE;
/*!40000 ALTER TABLE `place` DISABLE KEYS */;
INSERT INTO `place` VALUES (1,'MEAL','배식대',0.268526,-0.70297,0),(2,'LAUNDRY','다용도실',1.75687,-0.429889,0),(3,'INITIAL_POSE_1','로봇1_시작점',0,-0.5,0),(4,'INITIAL_POSE_2','로봇2_시작점',0,0,0),(5,'INITIAL_POSE_3','로봇3_시작점',0,0.5,0),(6,'1-A','1-A',1.87399,-2.25,0),(7,'1-B','1-B',1.81452,-1.54887,0),(8,'2-A','2-A',0.25,-1.60994,0),(9,'2-B','2-B',0.25,-2.18115,0),(10,'DESK','데스크',0.132795,0.85,0);
/*!40000 ALTER TABLE `place` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `robot`
--

DROP TABLE IF EXISTS `robot`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `robot` (
  `id` int NOT NULL AUTO_INCREMENT,
  `battery` int NOT NULL,
  `ros_domain_id` int NOT NULL,
  `robot_status_id` int NOT NULL,
  `robot_work_mode_id` int NOT NULL,
  PRIMARY KEY (`id`),
  KEY `fk_robot_status` (`robot_status_id`),
  KEY `fk_robot_work_mode` (`robot_work_mode_id`),
  CONSTRAINT `fk_robot_status` FOREIGN KEY (`robot_status_id`) REFERENCES `robot_status` (`id`),
  CONSTRAINT `fk_robot_work_mode` FOREIGN KEY (`robot_work_mode_id`) REFERENCES `robot_work_mode` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `robot`
--

LOCK TABLES `robot` WRITE;
/*!40000 ALTER TABLE `robot` DISABLE KEYS */;
INSERT INTO `robot` VALUES (1,100,93,2,1),(2,100,94,2,1),(3,90,97,2,1);
/*!40000 ALTER TABLE `robot` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `robot_status`
--

DROP TABLE IF EXISTS `robot_status`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `robot_status` (
  `id` int NOT NULL AUTO_INCREMENT,
  `name` varchar(16) NOT NULL,
  `meaning` varchar(16) NOT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `uq_name` (`name`)
) ENGINE=InnoDB AUTO_INCREMENT=5 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `robot_status`
--

LOCK TABLES `robot_status` WRITE;
/*!40000 ALTER TABLE `robot_status` DISABLE KEYS */;
INSERT INTO `robot_status` VALUES (1,'CHARGING','충전중'),(2,'WAIT','대기중'),(3,'WORK','업무중'),(4,'ERROR','업무 수행 불가');
/*!40000 ALTER TABLE `robot_status` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `robot_work_mode`
--

DROP TABLE IF EXISTS `robot_work_mode`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `robot_work_mode` (
  `id` int NOT NULL AUTO_INCREMENT,
  `name` varchar(16) NOT NULL,
  `meaning` varchar(16) NOT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `uq_name` (`name`)
) ENGINE=InnoDB AUTO_INCREMENT=5 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `robot_work_mode`
--

LOCK TABLES `robot_work_mode` WRITE;
/*!40000 ALTER TABLE `robot_work_mode` DISABLE KEYS */;
INSERT INTO `robot_work_mode` VALUES (1,'NONE','업무중 아님'),(2,'AUTO','자율주행'),(3,'FOLLOW','팔로잉'),(4,'TALK','대화중');
/*!40000 ALTER TABLE `robot_work_mode` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `task`
--

DROP TABLE IF EXISTS `task`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `task` (
  `id` int NOT NULL AUTO_INCREMENT,
  `task_type_id` int NOT NULL,
  `robot_id` int DEFAULT NULL,
  `requested_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP,
  `started_at` timestamp NULL DEFAULT NULL,
  `ETA` timestamp NULL DEFAULT NULL,
  `finished_at` timestamp NULL DEFAULT NULL,
  `goal_point` json DEFAULT NULL,
  `place` varchar(16) DEFAULT NULL,
  `remarks` text,
  PRIMARY KEY (`id`),
  KEY `fk_task_robot` (`robot_id`),
  KEY `fk_task_task_type` (`task_type_id`),
  CONSTRAINT `fk_task_robot` FOREIGN KEY (`robot_id`) REFERENCES `robot` (`id`),
  CONSTRAINT `fk_task_task_type` FOREIGN KEY (`task_type_id`) REFERENCES `task_type` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=62 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Table structure for table `task_type`
--

DROP TABLE IF EXISTS `task_type`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `task_type` (
  `id` int NOT NULL AUTO_INCREMENT,
  `name` varchar(16) NOT NULL,
  `meaning` varchar(16) NOT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `uq_name` (`name`)
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `task_type`
--

LOCK TABLES `task_type` WRITE;
/*!40000 ALTER TABLE `task_type` DISABLE KEYS */;
INSERT INTO `task_type` VALUES (1,'MEAL','식사'),(2,'TISSUE','휴지'),(3,'LAUNDRY','빨래');
/*!40000 ALTER TABLE `task_type` ENABLE KEYS */;
UNLOCK TABLES;

CREATE PROCEDURE IF NOT EXISTS `give_robot_task` (IN robotId INT, IN taskId INT)
BEGIN
	UPDATE task SET robot_id = robotId, started_at = now() WHERE id = taskId;
	UPDATE robot SET robot_status_id = 3, robot_work_mode_id = 2 WHERE id = robotId;
END;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2024-01-23 11:15:05
