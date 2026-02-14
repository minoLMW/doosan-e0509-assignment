# ROS2 Doosan E0509 MoveIt2 Assignment (Humble)

## 1. 프로젝트 개요
ROS2 Humble 환경에서 Doosan E0509 로봇암 + MoveIt2를 이용해 사용자가 입력한 목표 좌표(x,y,z)로 End-Effector(EE)를 이동시키는 시뮬레이션/제어 프로그램입니다.

- 목표 좌표는 1개 이상 입력
- 절대좌표(ABS) / 상대좌표(REL) 선택 가능
- 속도/가속도 스케일링(0.0~1.0) 입력 반영
- PyQt5 GUI(코드로 UI 구성) 제공
- 실행/정지 버튼은 별도 스레드(QThread)로 동작
- 실시간 상태(/assignment/status)로 연결/동작/로그/joint/EE pose 모니터링

---

## 2. 개발 환경
- OS: Ubuntu 22.04 (Jammy)
- ROS2: Humble
- MoveIt2: Humble
- Robot: Doosan E0509 (doosan-robot2)
- Language: Python (rclpy)
- GUI: PyQt5

---

## 3. 패키지 구조
권장 제출 구조(워크스페이스 기준):

ros2_ws/
├── src/
│ ├── my_ros2_assignment/
│ │ ├── package.xml
│ │ ├── setup.py
│ │ ├── setup.cfg
│ │ └── my_ros2_assignment/
│ │ ├── movegroup_sequence.py # 목표 좌표 시퀀스 실행 노드
│ │ ├── assignment_status_node.py # 상태 publish 노드
│ │ └── assignment_gui.py # PyQt5 GUI
│ └── my_ros2_assignment_msgs/
│ ├── CMakeLists.txt
│ ├── package.xml
│ └── msg/AssignmentStatus.msg
├── Presentation.pptx
├── readme.md
└── requirements.txt


---

## 4. 전체 시스템 구성(토픽/액션/서비스)

### 4.1 MoveIt2 / ros2_control
- MoveIt2 Action:
  - `/move_action` (moveit_msgs/action/MoveGroup)
- ros2_control controller:
  - `dsr_moveit_controller` (JointTrajectoryController) active
  - `joint_state_broadcaster` active
  - Follow joint trajectory action:
    - `/dsr_moveit_controller/follow_joint_trajectory`

### 4.2 과제 노드 인터페이스
- `/assignment/status` (my_ros2_assignment_msgs/msg/AssignmentStatus)
  - 연결상태(move_group/controller)
  - 동작상태(IDLE/PLANNING/EXECUTING/STOPPING/ERROR)
  - 최근 로그(recent_logs)
  - joint_state
  - ee_pose_base (base_link 기준)
- `/assignment/state_event` (std_msgs/String)
  - 실행 노드가 상태 이벤트를 k=v;... 형태로 publish
  - status 노드가 이를 파싱하여 motion/log를 갱신
- `/assignment/stop` (std_srvs/Trigger)
  - 실행 중 goal cancel 및 시퀀스 중지

---

## 5. 동작 로직

### 5.1 목표 입력/좌표 처리
- Targets 입력 포맷: `x,y,z; x,y,z; ...`
- ABS: 입력 좌표를 base_link 기준 절대 목표로 사용
- REL: 현재 EE pose(base_link 기준)를 읽어 입력 벡터를 더해 목표 생성

### 5.2 MoveGroup 액션 기반 이동
- MoveGroup ActionClient로 `/move_action`에 goal 전송
- MotionPlanRequest에
  - group_name = `manipulator`
  - max_velocity_scaling_factor = vel
  - max_acceleration_scaling_factor = acc
  - goal_constraints:
    - 기본: position-only constraint (성공률↑)
    - 옵션 keep_orientation: pos+ori로 1차 시도 후 실패 시 pos-only로 fallback
- 각 target을 순서대로 실행하고, 결과 error_code를 기록

### 5.3 상태 관리(/assignment/status)
- StatusNode가 주기적으로 publish
- 포함 내용:
  - 연결 상태: `/move_action`, `/<controller>/follow_joint_trajectory` 서버 준비 여부(server_is_ready)
  - joint_state: `/joint_states` 구독
  - EE pose: TF(base_link -> link_6) lookup
  - 이벤트 로그: `/assignment/state_event` 수신 후 최근 N개 버퍼로 저장

---

## 6. GUI 구성 (PyQt5, 코드 UI)
GUI는 Qt Designer 파일 없이 코드로 레이아웃을 구성합니다.

### 6.1 좌측(Control)
- ABS/REL 선택
- Targets 입력(PlainTextEdit)
- vel/acc 입력(LineEdit)
- 옵션 체크:
  - keep_orientation
  - plan_only
- RUN/STOP 버튼
  - RUN은 QThread에서 `movegroup_sequence` subprocess 실행
  - STOP은 `/assignment/stop` 서비스 호출 + SIGINT로 subprocess 종료

### 6.2 우측(Status/Monitor)
- 연결 상태(move_group / controller)
- 동작 상태(IDLE/PLANNING/EXECUTING 등)
- goal_id, target 진행률
- joint 6축 값 실시간 테이블
- EE Pose(base_link 기준) 표시
- 로그(recent_logs diff 방식으로 새 로그만 추가)

---

## 7. 실행 방법

### 7.1 빌드
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select my_ros2_assignment_msgs my_ros2_assignment
source install/setup.bash

### 7.2 MoveIt2 demo 실행
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch dsr_moveit_config_e0509 demo.launch.py

### 7.3 컨트롤러 확인/스폰(필요 시)
ros2 run controller_manager spawner dsr_moveit_controller --controller-manager /controller_manager
ros2 control list_controllers -c /controller_manager

### 7.4 상태 노드 실행
ros2 run my_ros2_assignment assignment_status_node

### 7.5 GUI 실행
(사전 설치)
sudo apt install -y python3-pyqt5

(실행)
python3 ~/ros2_ws/src/my_ros2_assignment/my_ros2_assignment/assignment_gui.py

### One-click bringup (optional)
chmod +x start.sh stop.sh
./start.sh
tmux attach -t doosan_assign
### stop: ./stop.sh


---

## 8. 트러블슈팅

### 8.1 CONTROL_FAILED(-4) / 실행 실패
- 원인: `follow_joint_trajectory` action server가 0개(컨트롤러 비활성)
- 해결:
ros2 run controller_manager spawner dsr_moveit_controller --controller-manager /controller_manager
ros2 control list_controllers -c /controller_manager

### 8.2 패키지 not found / 환경 꼬임
- build/install 정리 후 재빌드:
cd ~/ros2_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --packages-up-to dsr_moveit_config_e0509 --packages-skip dsr_gazebo2
source install/setup.bash

### 8.3 dsr_gazebo2 빌드 실패(gazebo_ros_pkgs)
- 현 과제는 MoveIt2 demo 기반으로 진행 → gazebo 패키지 빌드는 skip
colcon build --packages-skip dsr_gazebo2

### 8.4 /assignment/status publisher가 2개로 보일 때
- status 노드 중복 실행 상태. 하나만 남기기:
pkill -f assignment_status_node
ros2 run my_ros2_assignment assignment_status_node
