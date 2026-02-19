# Doosan E0509 Robot Arm Control GUI

ROS2 기반 두산 E0509 협동로봇 시뮬레이션 제어 프로젝트

## 프로젝트 개요

사용자가 목표 좌표를 입력하면 로봇암의 End-Effector가 해당 위치로 이동하는 ROS2 기반 시뮬레이션입니다.

- **로봇 모델**: Doosan E0509 (6축 협동로봇)
- **시뮬레이션**: Gazebo + 두산 에뮬레이터
- **GUI**: PyQt5 (코드 직접 구현)
- **ROS2 버전**: Humble
- **운영체제**: Ubuntu 22.04

---

## 실행 환경

### 시스템 요구사항
- Ubuntu 22.04 LTS
- ROS2 Humble
- Docker (에뮬레이터 실행용)
- Python 3.10+

### 의존성 설치
```bash
# requirements.txt 참조
sudo apt install -y ros-humble-desktop \
                    ros-humble-moveit \
                    ros-humble-gazebo-ros-pkgs \
                    ros-humble-joint-state-publisher \
                    ros-humble-robot-state-publisher \
                    ros-humble-xacro \
                    ros-humble-controller-manager \
                    ros-humble-ros2-control \
                    ros-humble-ros2-controllers

pip3 install pyqt5 numpy
```

---

## 실행 매뉴얼

### 1. Docker 컨테이너 접속
```bash
# 컨테이너 시작
docker start ros2-doosan

# 터미널 1: 컨테이너 접속
docker exec -it ros2-doosan bash
```

### 2. 두산 에뮬레이터 실행 (터미널 1)
```bash
# 컨테이너 내부에서
docker run -d --name dsr01_emulator --network host doosanrobot/dsr_emulator:3.0.1
```

### 3. Gazebo 시뮬레이션 실행 (터미널 2)
```bash
docker exec -it ros2-doosan bash
ros2 launch dsr_bringup2 dsr_bringup2_gazebo.launch.py model:=e0509
```

### 4. GUI 실행 (터미널 3)
```bash
docker exec -it ros2-doosan bash
cd ~/ros2_ws
colcon build --packages-select my_ros2_assignment
source install/setup.bash
ros2 run my_ros2_assignment my_node
```

### 단일 명령 실행 (Launch 파일)
```bash
ros2 launch my_ros2_assignment full_simulation.launch.py
```

---

## 작동 로직

### 전체 시스템 구조
```
┌─────────────────────────────────────────────────────────────┐
│                      사용자 (GUI)                            │
└─────────────────────┬───────────────────────────────────────┘
                      │ PyQt5 Signal/Slot
┌─────────────────────▼───────────────────────────────────────┐
│                   MainWindow (GUI)                          │
│  - 좌표 입력/목록 관리                                        │
│  - 옵션 설정 (절대/상대, 속도, 가속도)                         │
│  - 상태 모니터링 (연결, 관절, 위치, 로그)                      │
└─────────────────────┬───────────────────────────────────────┘
                      │ QThread
┌─────────────────────▼───────────────────────────────────────┐
│                   MoveThread (워커 스레드)                    │
│  - 좌표 목록 순회                                            │
│  - 비동기 이동 명령 실행                                      │
│  - 정지 플래그 감시                                          │
└─────────────────────┬───────────────────────────────────────┘
                      │ ROS2 Service Call
┌─────────────────────▼───────────────────────────────────────┐
│                 RobotController                             │
│  - /dsr01/motion/move_line (직선 이동)                       │
│  - /dsr01/motion/move_joint (관절 이동)                      │
│  - /dsr01/motion/move_stop (즉시 정지)                       │
│  - /dsr01/motion/ikin (역기구학 검증)                        │
│  - /dsr01/joint_states (관절 상태 구독)                      │
└─────────────────────┬───────────────────────────────────────┘
                      │ ROS2 Topic/Service
┌─────────────────────▼───────────────────────────────────────┐
│              Doosan Robot Emulator + Gazebo                 │
└─────────────────────────────────────────────────────────────┘
```

### 이동 프로세스
1. 사용자가 목표 좌표 (X, Y, Z, RX, RY, RZ) 입력
2. 역기구학(Ikin) 서비스로 도달 가능 여부 검증
3. 검증 통과 시 좌표 목록에 추가
4. "실행" 클릭 시 MoveThread 시작
5. MoveLine 서비스로 순차 이동
6. 실시간 관절/위치 상태 업데이트

---

## 기능 설명

### 1. 좌표 입력
| 항목 | 설명 | 단위 |
|------|------|------|
| X, Y, Z | End-Effector 위치 | mm |
| RX, RY, RZ | End-Effector 자세 (Roll, Pitch, Yaw) | degree |

### 2. 옵션 설정
| 옵션 | 설명 |
|------|------|
| 절대 좌표 | 로봇 베이스 기준 위치 |
| 상대 좌표 | 현재 위치 기준 오프셋 |
| 속도 | End-Effector 이동 속도 (mm/s) |
| 가속도 | End-Effector 가속도 (mm/s²) |

### 3. 상태 모니터
| 항목 | 설명 |
|------|------|
| 로봇 연결 상태 | 에뮬레이터 연결 여부 (3초 타임아웃) |
| 동작 상태 | 대기 / 이동 중 / 완료 / 정지 / 오류 |
| 관절 각도 | J1~J6 현재 각도 (degree) |
| End-Effector 위치 | Base 기준 절대 좌표 (X, Y, Z mm) |
| 실시간 로그 | 시스템 메시지 및 이동 상태 |

### 4. 프리셋 위치
미리 정의된 안전한 위치로 빠르게 이동:
- 홈 위치, 전방, 좌측, 우측, 높은 위치, 낮은 위치

### 5. 정지 기능
- 정지 버튼 클릭 시 **현재 위치에서 즉시 정지**
- MoveStop 서비스 호출로 로봇 동작 중단

---

## UI 구성

```
┌─────────────────────────────────────────────────────────────────┐
│  Doosan E0509 Robot Control                              [?]    │
├────────────────────────────┬────────────────────────────────────┤
│     제어 패널 (입력/실행)    │           상태 모니터               │
├────────────────────────────┼────────────────────────────────────┤
│ ┌─ 프리셋 위치 ───────────┐ │ ┌─ 연결 상태 ─────────────────────┐│
│ │ [홈][전방][좌측][우측]  │ │ │ 로봇: ● 연결됨                  ││
│ │ [높음][낮음]            │ │ │ [작업 자세로 전환]              ││
│ └─────────────────────────┘ │ └────────────────────────────────┘│
│                            │                                    │
│ ┌─ 목표 좌표 입력 ────────┐ │ ┌─ 동작 상태 ─────────────────────┐│
│ │ X: [350.0] mm           │ │ │ 상태: 대기                      ││
│ │ Y: [0.0] mm             │ │ └────────────────────────────────┘│
│ │ Z: [400.0] mm           │ │                                    │
│ │ RX: [0.0] RY: [180.0]   │ │ ┌─ 현재 관절 각도 (deg) ──────────┐│
│ │ RZ: [0.0]               │ │ │ J1: 0.00  J2: -20.00  J3: 90.00││
│ │ [좌표 추가]             │ │ │ J4: 0.00  J5: 70.00   J6: 0.00 ││
│ └─────────────────────────┘ │ └────────────────────────────────┘│
│                            │                                    │
│ ┌─ 좌표 목록 ─────────────┐ │ ┌─ End-Effector 위치 (mm) ─────────┐│
│ │ 1. pos(350, 0, 400)     │ │ │ X: 350.000  Y: 0.000  Z: 400.000││
│ │ 2. pos(400, 100, 300)   │ │ └────────────────────────────────┘│
│ │ [선택 항목 삭제]        │ │                                    │
│ └─────────────────────────┘ │ ┌─ 실시간 로그 ────────────────────┐│
│                            │ │ [10:30:01] 시스템 초기화 완료   ││
│ ┌─ 옵션 설정 ─────────────┐ │ │ [10:30:05] 로봇 연결됨          ││
│ │ ○ 절대 좌표 ○ 상대 좌표│ │ │ [10:30:10] 작업 자세 전환 완료  ││
│ │ 속도: [100] mm/s        │ │ │                                 ││
│ │ 가속도: [100] mm/s²     │ │ │                                 ││
│ └─────────────────────────┘ │ │                                 ││
│                            │ │                                 ││
│ ┌─ 실행 ─────────────────┐ │ │                                 ││
│ │ [▶ 실행]  [■ 정지]     │ │ │                                 ││
│ │ [목록 초기화]          │ │ │                                 ││
│ └─────────────────────────┘ │ └────────────────────────────────┘│
└────────────────────────────┴────────────────────────────────────┘
```

---

## 프레임워크 및 기술 스택

| 구분 | 기술 |
|------|------|
| ROS2 | Humble Hawksbill |
| 로봇 패키지 | doosan-robot2 (humble-devel) |
| GUI | PyQt5 5.15+ |
| 스레딩 | QThread (PyQt5) |
| 통신 | ROS2 Service (dsr_msgs2) |
| 시뮬레이션 | Gazebo + Doosan Emulator |

### 사용된 ROS2 서비스
| 서비스 | 용도 |
|--------|------|
| `/dsr01/motion/move_line` | 직선 이동 (MoveLine) |
| `/dsr01/motion/move_joint` | 관절 이동 (MoveJoint) |
| `/dsr01/motion/move_stop` | 즉시 정지 (MoveStop) |
| `/dsr01/motion/ikin` | 역기구학 검증 (Ikin) |
| `/dsr01/motion/fkin` | 순기구학 (Fkin) |
| `/dsr01/aux_control/get_current_posx` | 현재 위치 조회 |

### 사용된 ROS2 토픽
| 토픽 | 용도 |
|------|------|
| `/dsr01/joint_states` | 관절 상태 구독 |

---

## 패키지 구조

```
my_ros2_assignment/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── requirements.txt
├── resource/
│   └── my_ros2_assignment
├── launch/
│   └── full_simulation.launch.py
└── my_ros2_assignment/
    ├── __init__.py
    ├── main_node.py           # 진입점 (ROS2 + Qt 통합)
    ├── gui/
    │   ├── __init__.py
    │   └── main_window.py     # PyQt5 GUI
    ├── robot/
    │   ├── __init__.py
    │   └── controller.py      # 로봇 제어 (서비스 클라이언트)
    └── threads/
        ├── __init__.py
        └── move_thread.py     # QThread 이동 처리
```

---

## 문제 해결

### 로봇 연결 안됨
```bash
# 에뮬레이터 실행 확인
docker ps | grep emulator

# 에뮬레이터 재시작
docker stop dsr01_emulator
docker rm dsr01_emulator
docker run -d --name dsr01_emulator --network host doosanrobot/dsr_emulator:3.0.1
```

### 빌드 오류
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select my_ros2_assignment
```

### GUI 표시 안됨 (Docker)
```bash
# X11 포워딩 설정
xhost +local:docker
docker exec -it -e DISPLAY=$DISPLAY ros2-doosan bash
```

---

## 라이선스

이 프로젝트는 교육 목적으로 제작되었습니다.
- Doosan Robotics: [doosan-robot2](https://github.com/DoosanRobotics/doosan-robot2)
- MoveIt2: [moveit2](https://github.com/moveit/moveit2)
