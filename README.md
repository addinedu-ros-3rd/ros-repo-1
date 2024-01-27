# 1조 쉬러왔서영

---

## Index
- [프로젝트 소개](#프로젝트-소개)
  - [프로젝트 기간](#프로젝트-기간)
  - [기술 스택](#기술-스택)
  - [팀원 역할](#팀원-역할)
- [프로젝트 설계](#프로젝트-설계)
  - [시스템 구성](#시스템-구성)
  - [기능 리스트](#기능-리스트)
  - [Map](#map)
  - [데이터 구조](#데이터-구조)
  - [GUI](#GUI)
- [다중 로봇 제어](#다중-로봇-제어)
  - [Multi-Robot Control](#multi-robot-control)
  - [Task Planning](#task-planning)
    - [작업 요청 시나리오](#작업-요청-시나리오)
    - [작업 스케줄링 시나리오](#작업-스케줄링-시나리오)
    - [작업 수행 시나리오](#작업-수행-시나리오)
- [Navigation](#navigation)
  - [A* Path Planning](#a-path-planning)
  - [이슈 처리](#이슈-처리)
- [딥러닝 요소](#딥러닝-요소)
  - [Following: 사람 추적 기능](#following-사람-추적-기능)
  - [안전 기능: 쓰러진 보행자 인식](#안전-기능-쓰러진-보행자-인식)
- [결론](#결론)
  - [시연 영상](#시연-영상)
  - [결과 요약](#결과-요약)
  - [회고](#회고)
  - [발표자료](#발표자료)
- [실행 방법](#실행-방법)
  - [공통](#공통)
  - [다중 로봇](#다중-로봇)
  - [Following](#Following)
  - [안전 기능](#안전-기능)

---

## 프로젝트 소개
요양원 업무 보조를 위한 <b>다중 주행 로봇 제어 시스템</b>
<p align=center width="98%">
  <img src="./images/play.gif">
</p>

### 프로젝트 기간
2023.12.26 ~ 2024.01.25

### 기술 스택
|   |   |
|---|---|
|개발환경|![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=Ubuntu&logoColor=white) ![Visual Studio Code](https://img.shields.io/badge/Visual%20Studio%20Code-007ACC?style=for-the-badge&logo=Visual%20Studio%20Code&logoColor=white) ![Git](https://img.shields.io/badge/Git-F05032?style=for-the-badge&logo=Git&logoColor=white) ![Github](https://img.shields.io/badge/GitHub-181717?style=for-the-badge&logo=GitHub&logoColor=white) ![RDS](https://img.shields.io/badge/AWS%20RDS-527FFF?style=for-the-badge&logo=Amazon%20RDS&logoColor=white)||
|기술|![Python](https://img.shields.io/badge/python-3776AB?style=for-the-badge&logo=python&logoColor=white) ![PyTorch](https://img.shields.io/badge/PyTorch-EE4C2C?style=for-the-badge&logo=PyTorch&logoColor=white) ![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=OpenCV&logoColor=white) ![ROS2](https://img.shields.io/badge/ROS2-22314E?style=for-the-badge&logo=ROS&logoColor=white) ![Mysql](https://img.shields.io/badge/mysql-4479A1?style=for-the-badge&logo=mysql&logoColor=white) ![Qt](https://img.shields.io/badge/Qt-41CD52?style=for-the-badge&logo=Qt&logoColor=white)|
|커뮤니케이션|![Jira](https://img.shields.io/badge/Jira-0052CC?style=for-the-badge&logo=Jira&logoColor=white) ![Confluence](https://img.shields.io/badge/Confluence-172B4D?style=for-the-badge&logo=Confluence&logoColor=white) ![Slack](https://img.shields.io/badge/slack-4A154B?style=for-the-badge&logo=slack&logoColor=white)|

### 팀원 역할

|구분|이름|역할|
|---|---|---|
|팀장|조태상|Map 구축, SLAM, Camera Calibration, Aruco Marker Detection|
|팀원|강소희|GUI 설계 및 디자인, Map 구축|
|팀원|강한얼|딥러닝 활용 Following mode, 보행자 쓰러짐 인식 기능 개발|
|팀원|문서영|Map 구축, SLAM, 딥러닝 활용 Following mode|
|팀원|오윤|Multi Robot Control, Task Planning, DB Query/SP 작성, GUI 경로 표시|
|팀원|조홍기|A* Path Planning, 로봇 주행, Multi Robot Spawn 시도|

---

## 프로젝트 설계

### 시스템 구성
<p align=center width="98%">
  <img src="./images/system_architecture.png">
</p>

### 기능 리스트
<p align=center width="98%">
  <img src="./images/functions.png">
</p>

### Map
<p align=center width="98%">
  <img src="./images/map.PNG">
</p>

### 데이터 구조
<p align=center width="98%">
  <img src="./images/erd.png">
</p>

### GUI
<p align=center width="98%">
  <img src="./images/gui.png">
</p>

---

## 다중 로봇 제어

### Multi-Robot Control
<p align=center width="98%">
  <img src="./images/multi_robot_control.PNG">
</p>

### Task Planning
#### 작업 요청 시나리오
<p align=center width="98%">
  <img src="./images/task_req.png">
</p>

#### 작업 스케줄링 시나리오
<p align=center width="98%">
  <img src="./images/task_skd.png">
</p>

#### 작업 수행 시나리오
<p align=center width="98%">
  <img src="./images/task_run.png">
</p>

---

## Navigation

### A* Path Planning

- Sequence Diagram
<p align=center width="98%">
  <img src="./images/path_planning.png">
</p>

### 이슈 처리

- 안전 주행을 위한 장애물 Padding 처리
<p align=center width="98%">
  <img src="./images/path_planning_padding.png">
</p>
- Behavior Tree 개선 및 cmd_vel 조정을 통한 Timeout 상태 탈출
<p align=center width="98%">
  <img src="./images/BT.png">
</p>

---

## 딥러닝 요소

### Following: 사람 추적 기능

### 안전 기능: 쓰러진 보행자 인식

---

## 결론

### 시연 영상
<p align=center>
  <a href="https://youtu.be/rqWVjeEV688?feature=shared">
    <img src="https://i.ytimg.com/vi/rqWVjeEV688/maxresdefault.jpg" width="80%">
  </a>
</p>

### 결과 요약
- __Navigation
  - A* Path Planning
  - 장애물 Padding, 오차 보정을 통한 안전하고 정확한 주행
  - Behavior Tree를 사용하여 동적/정적 장애물 회피 및 고착 상태 탈출


- __다중 로봇 제어__
  - 서로 다른 도메인ID 간 통신 가능하게 하여 하나의 맵에서 다수의 로봇을 제어
  - 로봇 상태별 스케줄링

- __딥러닝 요소__
  - 딥러닝 기반의 영상 인식을 활용한 사람 추적 및 보행자 쓰러짐 인식 기능

### 회고
- 보다 다양한 시나리오에서 장애물 회피 로직을 구현해보고 싶습니다.
- 라이다와 엔코더 이외에도 여러 개의 센서를 사용하여 Localization에 활용해보고 싶습니다.
- Jira, Confluence 등의 협업 툴을 사용해보는 기회가 되었습니다.
- ROS2 topic을 통해 GUI로 다양한 기능을 보여줄 수 있어 재밌었습니다.

### 발표자료
https://docs.google.com/presentation/d/1JA80CWg-Doe3NlXZKzBihne2yhNvz5CSX7nk5qTw1c4/edit?usp=sharing

---

## 실행 방법

### 공통
- ROS2의 humble 버전을 설치하여 사용했습니다.
- 핑크랩의 미니봇 하드웨어와 소스코드(https://github.com/PinkWink/pinklab_minibot_robot) 를 사용했습니다.
  - minibot_bringup/scripts 파일로 udev를 설정했습니다.
- 모터 컨트롤을 위해 핑크랩의 아두이노 코드(https://github.com/PinkWink/pinklab_minibot_robot_firmware) 를 사용했습니다.
- LiDAR 사용을 위해 안병규님이 수정하신 driver(https://github.com/byeongkyu/ydlidar_ros2_driver) 를 사용했습니다.
- 미니봇을 구동하기 위해 Ubuntu 22.04를 설치한 라즈베리파이와 PC를 사용했습니다.
- 사용되는 모든 PC/라즈베리파이는 ROS2 통신을 위해 동일한 네트워크에 접속했습니다.
- 프로젝트를 클론한 후, nursing_home 디렉토리에서 빌드해주세요.
```
git clone https://github.com/addinedu-ros-3rd/ros-repo-1.git
cd ros-repo-1/nursing_home
colcon build
source ./install/local_setup.bash
```

### 다중 로봇
#### 1) DB 접속<br>
- nursing_home/src/main_pkg/utils 경로에 config.ini 파일 생성
```
[dev]
host = DB Host
port = DB port
user = DB user
password = DB password
database = DB database name
```

- 해당 DB에 Table/Procedure 가져오기
```
source create_and_init.sql
```

#### 2) ROS_DOMAIN_ID 및 ROS2 환경 접속 설정<br>
프로젝트 루트에 있는 bridge_config.yaml 설정을 그대로 사용한다면<br>
다음 내용을 ~/.bashrc 파일에 추가한 다음 ```source ~/.bashrc``` 하면 됩니다.<br>

- 관제 PC
```
export ROS_DOMAIN_ID = 91
source /opt/ros/humble/setup.bash
source ~/nursing_home/install/local_setup.bash
```

- 로봇1 라즈베리파이
```
export ROS_DOMAIN_ID = 93
source /opt/ros/humble/setup.bash
source ~/pinkbot/install/local_setup.bash
```

- 로봇1 PC
```
export ROS_DOMAIN_ID = 93
source /opt/ros/humble/setup.bash
source ~/pinkbot/install/local_setup.bash
source ~/nursing_home/install/local_setup.bash
```

- 로봇2 라즈베리파이
```
export ROS_DOMAIN_ID = 94
source /opt/ros/humble/setup.bash
source ~/pinkbot/install/local_setup.bash
```

- 로봇2 PC
```
export ROS_DOMAIN_ID = 94
source /opt/ros/humble/setup.bash
source ~/pinkbot/install/local_setup.bash
source ~/nursing_home/install/local_setup.bash
```

- 로봇3 라즈베리파이
```
export ROS_DOMAIN_ID = 97
source /opt/ros/humble/setup.bash
source ~/pinkbot/install/local_setup.bash
```

- 로봇3 PC
```
export ROS_DOMAIN_ID = 97
source /opt/ros/humble/setup.bash
source ~/pinkbot/install/local_setup.bash
source ~/nursing_home/install/local_setup.bash
```

#### 3) 라즈베리파이: 로봇 실행 + 카메라 실행
- 터미널 1
```
ros2 launch minibot_bringup bringup_robot.launch.py
```

- 터미널 2
```
sudo chmod 777 /dev/video0
ros2 run v4l2_camera v4l2_camera_node
```

#### 4) 로봇PC: 주행 실행(yaml파일 경로: nursing_home/src/main_pkg/map)
- 터미널 1
```
ros2 launch minibot_navigation2 bringup_launch.py map:=home.yaml
```

- 터미널 2
```
ros2 run robot_pkg robot_controller
```

#### 5) 관제PC: GUI/메인컨트롤러 실행
- 터미널 1
```
ros2 run main_pkg main_controller
```

- 터미널 2
```
ros2 run ui_pkg monitoring
```

- 터미널 3
```
ros2 run domain_bridge domain_bridge bridge_config.yaml
```

### Following

### 안전 기능
