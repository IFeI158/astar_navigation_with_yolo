# 🧭 Integrated Navigation with YOLO (ROS2)

이 프로젝트는 **ROS2 기반 자율주행 로봇 내비게이션 시스템**으로,
A* 경로 계획과 YOLO 객체 감지를 통합한 노드입니다.

---

## 📌 주요 기능

- 🗺️ OccupancyGrid 기반 A* 경로 계획
- 📍 AMCL 기반 로봇 위치 추정
- 🤖 Pure Pursuit 방식 경로 추종
- 👁️ YOLO 객체 감지 (Compressed Image 입력)
- 🚨 객체 감지 시 로봇 정지 및 재개
- 🖼️ YOLO 디버그 이미지 시각화

---

## 📁 패키지 구조 예시

📦 img_pkg
 ┣ 📂 src
 ┃ ┗ cho_test.py
 ┣ 📂 launch
 ┃ ┗ tb3_localization.launch.py
 ┣ best.pt
 ┣ package.xml
 ┗ setup.py
---

## 🧠 노드 설명 (IntegratedNavigation)

### 구독 토픽 (Subscribers)

- /map (nav_msgs/OccupancyGrid)
- /amcl_pose (PoseWithCovarianceStamped)
- /goal_pose (PoseStamped)
- /image_raw/compressed (sensor_msgs/CompressedImage)

---

### 발행 토픽 (Publishers)

- /cmd_vel (geometry_msgs/Twist)
- /planned_path (nav_msgs/Path)
- /yolo_debug (sensor_msgs/Image)

---

### YOLO 기반 정지 로직

- 객체가 1개 이상 감지되면 로봇 즉시 정지
- 객체가 사라지면 자동 주행 재개

---

## 🚀 Launch 파일 설명
(tb3_localization.launch.py)

### 포함 구성

- Nav2 Localization (AMCL + Map Server)
- RViz2
- 사용자 A* + YOLO 노드 실행

---

## ⚙️ 필요 요구 사항

### 시스템 환경

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10 이상

---

### ROS2 패키지

sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-cv-bridge
sudo apt install ros-humble-image-transport

---

### Python 라이브러리

pip install ultralytics
pip install opencv-python
pip install numpy

---

### YOLO 모델

- YOLOv8 기반 best.pt 필요
- 코드 내 경로 예시:

/home/dev/ros-cv_ws/img_pkg/best.pt

---

## ▶️ 실행 가이드

### 1. 워크스페이스 빌드

cd ~/ros-cv_ws
colcon build
source install/setup.bash

---

### 2. Launch 실행

ros2 launch img_pkg tb3_localization.launch.py

---

### 3. 목표 지점 설정

- RViz에서 "2D Nav Goal" 클릭
- 로봇이 경로 생성 후 이동

---

### 4. YOLO 디버그 확인

rqt_image_view 실행 후
/yolo_debug 토픽 선택

---

## ✅ 테스트 체크리스트

- map 정상 수신
- amcl_pose 갱신
- 카메라 이미지 수신
- YOLO 박스 표시
- 객체 감지 시 정지
- 목표 도달 시 정지

---

## 📌 참고 사항

- Nav2 Global Planner 미사용
- 자체 A* 알고리즘 사용
- YOLO는 회피가 아닌 정지 트리거 용도

---

## ✨ 향후 개선

- YOLO 결과를 costmap에 반영
- 동적 재경로 계획
- 객체 클래스별 행동 분기
