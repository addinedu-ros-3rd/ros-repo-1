# 1조 쉬러왔서영

---

## Index
- 프로젝트 소개
- 프로젝트 설계
  - 시스템 구성
  - 시나리오
  - Map
  - 기술 스택
- 다중 로봇 제어
  - Multi-Robot Control
  - Task Planning
- Navigation
  - A* Path Planning
  - 이상 상황 Management
  - Behavior Tree
- 딥러닝 요소
  - Following: 사람 추적 기능
  - 안전 기능: 쓰러진 보행자 인식
- 결론
  - 결과 요약
  - 팀원 역할
  - 회고
---

## 프로젝트 소개
요양원 업무 보조를 위한 <b>다중 주행 로봇 제어 시스템</b>
![시연gif](./images/play.gif)

---

## 프로젝트 설계

### 기술 스택
|   |   |
|---|---|
|개발환경|![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=Ubuntu&logoColor=white) ![Visual Studio Code](https://img.shields.io/badge/Visual%20Studio%20Code-007ACC?style=for-the-badge&logo=Visual%20Studio%20Code&logoColor=white) ![Git](https://img.shields.io/badge/Git-F05032?style=for-the-badge&logo=Git&logoColor=white) ![Github](https://img.shields.io/badge/GitHub-181717?style=for-the-badge&logo=GitHub&logoColor=white) ![RDS](https://img.shields.io/badge/AWS%20RDS-527FFF?style=for-the-badge&logo=Amazon%20RDS&logoColor=white)||
|기술|![Python](https://img.shields.io/badge/python-3776AB?style=for-the-badge&logo=python&logoColor=white) ![PyTorch](https://img.shields.io/badge/PyTorch-EE4C2C?style=for-the-badge&logo=PyTorch&logoColor=white) ![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=OpenCV&logoColor=white) ![ROS2](https://img.shields.io/badge/ROS2-22314E?style=for-the-badge&logo=ROS&logoColor=white) ![Mysql](https://img.shields.io/badge/mysql-4479A1?style=for-the-badge&logo=mysql&logoColor=white) ![Qt](https://img.shields.io/badge/Qt-41CD52?style=for-the-badge&logo=Qt&logoColor=white)|
|커뮤니케이션|![Jira](https://img.shields.io/badge/Jira-0052CC?style=for-the-badge&logo=Jira&logoColor=white) ![Confluence](https://img.shields.io/badge/Confluence-172B4D?style=for-the-badge&logo=Confluence&logoColor=white) ![Slack](https://img.shields.io/badge/slack-4A154B?style=for-the-badge&logo=slack&logoColor=white)|

### 시스템 구성 
![시스템구성도](./images/system_architecture.png)

### 기능 리스트 
![기능리스트](./images/functions.png)

### Map
![맵](./images/map.PNG)

---

## 다중 로봇 제어

### Multi-Robot Control
![Multi Robot Control](./images/multi_robot_control.PNG)

### Task Planning
#### 작업 요청 시나리오
![Task Req](./images/task_req.png)
#### 작업 스케줄링 시나리오
![Task Skd](./images/task_skd.png)
#### 작업 수행 시나리오
![Task Run](./images/task_run.png)

---

## Navigation

### A* Path Planning
![Path Planning](./images/path_planning.png)

### 이슈 처리

---

## 딥러닝 요소

### Following: 사람 추적 기능


### 안전 기능: 쓰러진 보행자 인식

---

## 결론

### 결과 요약

### 팀원 소개 및 역할
|구분|이름|역할|
|---|---|---|
|팀장|조태상||
|팀원|강한얼||
|팀원|오윤||
|팀원|문서영||
|팀원|조홍기||
|팀원|강소희||

### 회고