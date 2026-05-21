# ISR Navigation

ISR 로봇을 위한 Nav2 기반 자율주행 패키지. Nav2 스택을 활용한 경로 계획 및 추종, 웨이포인트 기반 자율주행 기능을 제공합니다.

## 개요

ISR Navigation은 Nav2(Navigation2) 프레임워크를 기반으로 ISR 로봇의 자율주행을 지원하며 다음 기능을 제공합니다:
- **Nav2 통합**: MPPI 컨트롤러, NavFn 플래너, Collision Monitor 등 최적화된 Nav2 파라미터 설정
- **웨이포인트 네비게이션**: 사용자 정의 웨이포인트 순회를 통한 자율주행
- **다양한 주행 모드**: 단일 주행, 순환(Loop), 왕복(Round Trip) 모드 지원
- **GUI 제어**: Qt5 기반 GUI를 통한 직관적인 웨이포인트 관리 및 네비게이션 제어
- **실시간 시각화**: RViz2를 통한 웨이포인트 및 경로 시각화
- **웨이포인트 저장/불러오기**: YAML 파일을 통한 웨이포인트 영구 저장

## 구성 요소

### 1. Nav2 Launch (`navigation_launch.py`)
Nav2 스택의 핵심 노드들을 실행하는 런치 파일입니다.

**실행되는 노드:**
- `controller_server`: MPPI 컨트롤러를 사용한 로컬 경로 추종
- `planner_server`: NavFn(A*) 알고리즘 기반 전역 경로 계획
- `bt_navigator`: Behavior Tree 기반 네비게이션 관리
- `smoother_server`: 경로 스무딩
- `behavior_server`: 복구 행동 (Spin, BackUp, Wait 등)
- `waypoint_follower`: 웨이포인트 순차 추종
- `velocity_smoother`: 속도 명령 스무딩
- `collision_monitor`: 실시간 충돌 회피
- `lifecycle_manager`: 노드 라이프사이클 관리

**구현 코드 위치:**
- 런치 파일: `launch/navigation_launch.py`
- 파라미터 파일: `config/params.yaml`

### 2. Goal Publisher Node (`goal_publisher`)
웨이포인트를 관리하고 Nav2에 목표 위치를 발행하는 노드입니다.

**주요 기능:**
- **웨이포인트 등록**: RViz의 Publish Point 도구로 `/clicked_point` 토픽을 통한 웨이포인트 추가
- **자동 네비게이션**: 등록된 웨이포인트를 순차적으로 추종
- **다중 주행 모드**:
  - `OFF`: 모든 웨이포인트 1회 순회 후 종료
  - `LOOP`: 마지막 웨이포인트 도달 후 첫 번째 웨이포인트로 순환
  - `ROUND_TRIP`: 마지막 웨이포인트 도달 후 역순으로 복귀, 시작 위치로 돌아온 뒤 다시 순방향 주행
- **웨이포인트 저장/불러오기**: YAML 파일로 웨이포인트 영구 저장 및 로드
- **실시간 시각화**: RViz 마커를 통한 웨이포인트 및 경로 표시

**작동 방식:**
1. RViz에서 "Publish Point" 도구로 웨이포인트 등록 또는 YAML 파일에서 로드
2. `/auto_nav` 서비스 호출로 자동 네비게이션 시작
3. TF를 통해 로봇 위치 모니터링 및 도착 판정
4. 도착 시 다음 웨이포인트로 `/goal_pose` 발행
5. 주행 모드에 따라 순환 또는 종료

**구현 코드 위치:**
- 헤더: `include/nav2_navigation/goal_publisher.hpp`
- 구현: `src/goal_publisher.cpp`
- 파라미터: `config/goal_publisher_params.yaml`

### 3. Goal Publisher GUI Node (`goal_publisher_gui`)
Qt5 기반 GUI를 통해 Goal Publisher를 제어하는 노드입니다.

**주요 기능:**
- **네비게이션 제어**: Start/Stop 버튼으로 자동 네비게이션 시작/중지
- **모드 전환**: OFF → LOOP → ROUND_TRIP 순환 토글
- **웨이포인트 관리**: 목록 조회, 마지막 삭제, 전체 삭제
- **파일 저장/불러오기**: 파일 다이얼로그를 통한 웨이포인트 YAML 저장/로드
- **상태 모니터링**: 실시간 상태 메시지 표시

**구현 코드 위치:**
- 헤더: `include/nav2_navigation/goal_publisher_gui.hpp`
- 구현: `src/goal_publisher_gui.cpp`

### 4. Rosbag Visualize Launch (`rosbag_visualize_launch.py`)
Rosbag 파일을 재생하면서 지도와 함께 시각화하는 런치 파일입니다. 데이터 검증, 디버깅, 결과 확인 등에 활용됩니다.

**실행되는 노드:**
- `static_transform_publisher` (map → odom): 정적 TF 발행
- `static_transform_publisher` (base_link → velodyne): 정적 TF 발행
- `map_server`: 2D 지도 로드 및 발행
- `lifecycle_manager`: map_server 라이프사이클 관리
- `rviz2`: 시각화
- `ros2 bag play`: rosbag 재생

**런치 파라미터:**
| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `rosbag_path` | (필수) | 재생할 rosbag 파일 경로 (쉼표로 구분하여 여러 개 지정 가능) |
| `map_file` | `map_2D/campus/campus.yaml` | 지도 YAML 파일 경로 |
| `rviz_config` | `lidar_localization/config/rviz2.rviz` | RViz 설정 파일 경로 |
| `use_sim_time` | `true` | 시뮬레이션 시간 사용 여부 |
| `rate` | `1.0` | 재생 속도 (0.5: 절반 속도, 2.0: 2배속) |

**구현 코드 위치:**
- 런치 파일: `launch/rosbag_visualize_launch.py`

## 설치

### 사전 요구사항
- Ubuntu 22.04
- ROS2 Humble
- Nav2 패키지
- Qt5

### 빌드

```bash
# 워크스페이스로 이동
cd ~/your_workspace

# 시스템 패키지 업데이트 및 필수 패키지 설치
# nav2-bringup이 controller, planner, bt_navigator, smoother, behaviors,
# waypoint_follower, velocity_smoother, lifecycle_manager 등을 의존성으로 포함
# nav2-map-server, nav2-collision-monitor는 별도 설치 필요
sudo apt update
sudo apt install ros-humble-nav2-bringup
sudo apt install qtbase5-dev


# ROS 의존성 설치
rosdep update
rosdep install --from-paths src/nav2_navigation --ignore-src -r -y

# 빌드
colcon build --packages-select nav2_navigation

# Source
source install/setup.bash
```

### 의존성

**ROS 패키지 (rosdep 자동 설치):**
- rclcpp - ROS2 C++ 클라이언트 라이브러리
- geometry_msgs - 기하학적 메시지 타입
- sensor_msgs - 센서 메시지 타입
- visualization_msgs - RViz 마커 메시지 타입
- tf2_ros, tf2_geometry_msgs - 좌표 변환 처리
- std_srvs - 표준 서비스 타입
- yaml-cpp - YAML 파일 파싱

**Nav2 패키지:**
- nav2_bringup - Nav2 런치 파일 (아래 패키지들을 의존성으로 포함)
  - nav2_bt_navigator - Behavior Tree 네비게이터
  - nav2_controller - 로컬 컨트롤러 서버 (MPPI)
  - nav2_planner - 전역 플래너 서버 (NavFn)
  - nav2_smoother - 경로 스무딩
  - nav2_behaviors - 복구 행동 (Spin, BackUp, Wait)
  - nav2_waypoint_follower - 웨이포인트 추종
  - nav2_velocity_smoother - 속도 스무딩
  - nav2_costmap_2d - 코스트맵 관리
  - nav2_lifecycle_manager - 라이프사이클 관리
- nav2_map_server - 맵 서버 (별도 설치 필요)
- nav2_collision_monitor - 실시간 충돌 모니터 (별도 설치 필요)

**Qt5 패키지:**
- qtbase5-dev - Qt5 개발 라이브러리

**워크스페이스 내 의존성:**
- lidar_localization - LiDAR 기반 위치 추정

## 사용법

### 1. 통합 실행 (권장)
Nav2, Map Server, Goal Publisher를 한 번에 실행합니다.

```bash
ros2 launch nav2_navigation nav2_launch.py
```

**런치 파라미터:**
```bash
# 커스텀 맵 파일 사용
ros2 launch nav2_navigation nav2_launch.py \
  map:=/path/to/your/map.yaml

# 커스텀 파라미터 파일 사용
ros2 launch nav2_navigation nav2_launch.py \
  params_file:=/path/to/custom_params.yaml

# 시뮬레이션 시간 사용
ros2 launch nav2_navigation nav2_launch.py \
  use_sim_time:=True
```

### 2. 개별 노드 실행

#### Goal Publisher만 실행
```bash
ros2 launch nav2_navigation goal_publisher_launch.py
```

#### Navigation 노드들만 실행
```bash
ros2 launch nav2_navigation navigation_launch.py \
  params_file:=/path/to/params.yaml
```

### 3. 웨이포인트 네비게이션 사용법

#### Step 1: 웨이포인트 등록
RViz2에서 상단 툴바의 "Publish Point" 버튼을 클릭한 후 맵에서 원하는 위치를 클릭합니다.

또는 GUI에서 "Load WP" 버튼으로 기존 YAML 파일 로드:
```bash
# GUI가 자동으로 파일 다이얼로그를 열어줍니다
```

또는 서비스 호출로 로드:
```bash
ros2 service call /load_waypoints nav2_navigation/srv/LoadWaypoints \
  "{file_path: '/path/to/waypoints.yaml'}"
```

#### Step 2: 주행 모드 설정 (선택사항)
GUI에서 "Mode" 버튼 클릭 (OFF → LOOP → ROUND_TRIP → OFF 순환)

또는 서비스 호출:
```bash
ros2 service call /toggle_loop_mode std_srvs/srv/Trigger
```

#### Step 3: 자동 네비게이션 시작
GUI에서 "Start Nav" 버튼 클릭

또는 서비스 호출:
```bash
ros2 service call /auto_nav std_srvs/srv/Trigger
```

#### Step 4: 네비게이션 중지 (필요시)
GUI에서 "Stop Nav" 버튼 클릭

또는 서비스 호출:
```bash
ros2 service call /stop_nav std_srvs/srv/Trigger
```

#### Step 5: 웨이포인트 저장
GUI에서 "Save WP" 버튼 클릭

또는 서비스 호출:
```bash
ros2 service call /save_waypoints nav2_navigation/srv/SaveWaypoints \
  "{file_path: '/path/to/save/waypoints.yaml'}"
```

### 4. Rosbag 시각화
Rosbag 데이터를 지도와 함께 시각화합니다. 데이터 검증 및 결과 확인에 유용합니다.

```bash
# 기본 실행
ros2 launch nav2_navigation rosbag_visualize_launch.py \
  rosbag_path:=/path/to/your/rosbag

# 커스텀 지도 파일 사용
ros2 launch nav2_navigation rosbag_visualize_launch.py \
  rosbag_path:=/path/to/your/rosbag \
  map_file:=/path/to/custom/map.yaml

# 모든 파라미터 지정
ros2 launch nav2_navigation rosbag_visualize_launch.py \
  rosbag_path:=/path/to/your/rosbag \
  map_file:=/path/to/map.yaml \
  rviz_config:=/path/to/custom.rviz \
  use_sim_time:=true
```

#### 여러 Rosbag 연속 재생
쉼표로 구분하여 여러 rosbag 파일을 순차적으로 재생할 수 있습니다. 첫 번째 bag 재생이 끝나면 자동으로 다음 bag이 재생됩니다.

```bash
ros2 launch nav2_navigation rosbag_visualize_launch.py \
  rosbag_path:="/path/to/rosbag1,/path/to/rosbag2,/path/to/rosbag3"
```

#### 재생 속도 조절
`rate` 파라미터로 재생 속도를 조절할 수 있습니다.

```bash
# 2배속 재생
ros2 launch nav2_navigation rosbag_visualize_launch.py \
  rosbag_path:=/path/to/your/rosbag \
  rate:=2.0
```

## 파라미터 튜닝 가이드

파라미터 파일: `config/params.yaml`
아래 상황별 예시를 참고하여 파라미터 튜닝 가능

### 1. 로봇 속도 조절

**더 빠르게 주행하고 싶을 때:**
```yaml
# config/params.yaml
controller_server:
  ros__parameters:
    FollowPath:
      vx_max: 0.8           # 최대 전진 속도 증가 (기본: 0.5 m/s)
      vx_min: -0.5          # 최대 후진 속도 증가 (기본: -0.35 m/s)
      ax_max: 3.0           # 가속도 증가 (기본: 2.0 m/s^2)

velocity_smoother:
  ros__parameters:
    max_velocity: [0.8, 0.0, 1.5]   # [x, y, theta] 속도 제한
    max_accel: [3.0, 0.0, 4.0]      # 가속도 제한
```

**더 천천히 안전하게 주행하고 싶을 때:**
```yaml
controller_server:
  ros__parameters:
    FollowPath:
      vx_max: 0.3
      wz_max: 1.5           # 회전 속도도 줄임

velocity_smoother:
  ros__parameters:
    max_velocity: [0.3, 0.0, 0.8]
```

---

### 2. 장애물 회피 강화

**장애물에 더 민감하게 반응하고 싶을 때:**
```yaml
# Collision Monitor 영역 확대
collision_monitor:
  ros__parameters:
    StopCircle:
      points: [0.5, 0.5, 0.5, -0.5, 0.0, -0.5, 0.0, 0.5]  # 정지 영역 확대
    SlowCircle:
      points: [1.5, 0.6, 1.5, -0.6, 0.0, -0.6, 0.0, 0.6]  # 감속 영역 확대
      slowdown_ratio: 0.3   # 더 많이 감속 (기본: 0.5)

# 코스트맵 inflation 증가
local_costmap:
  local_costmap:
    ros__parameters:
      inflation_layer:
        inflation_radius: 1.0   # 장애물 주변 회피 영역 확대 (기본: 0.85 m)
        cost_scaling_factor: 4.0  # 장애물 근처 비용 급격히 증가 (기본 3.0)

# MPPI 장애물 회피 비용 증가
controller_server:
  ros__parameters:
    FollowPath:
      CostCritic:
        cost_weight: 25.0       # 장애물 회피 가중치 증가 (기본: 15.0)
```

**좁은 공간을 통과해야 할 때:**
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.25        # 로봇 반경 축소 (기본 0.3)
      inflation_layer:
        inflation_radius: 0.5   # inflation 축소

global_costmap:
  global_costmap:
    ros__parameters:
      inflation_layer:
        inflation_radius: 0.45  # inflation 축소(기본 0.5)
```

---

### 3. 경로 추종 강화

**글로벌 경로를 더 정확히 따라가게 하고 싶을 때:**
```yaml
controller_server:
  ros__parameters:
    FollowPath:
      PathFollowCritic:
        cost_weight: 10.0       # 경로 추종 가중치 증가 (기본: 5.0)
      PathAlignCritic:
        cost_weight: 5.0        # 경로 정렬 가중치 증가 (기본: 2.0)
      PathAngleCritic:
        cost_weight: 5.0        # 경로 각도 가중치 증가 (기본: 2.0)
      CostCritic:
        cost_weight: 8.0       # 장애물 회피 가중치 감소 (기본: 15.0)

# 코스트맵 inflation 감소
local_costmap:
  local_costmap:
    ros__parameters:
      inflation_layer:
        inflation_radius: 0.7   # 장애물 주변 회피 영역 축소 (기본: 0.85 m)
        cost_scaling_factor: 2.0  # 장애물 근처 비용 감소(기본 3.0)
```

---

### 4. 로봇이 자주 멈추거나 Stuck 판정될 때

**Progress checker 완화:**
```yaml
controller_server:
  ros__parameters:
    progress_checker:
      required_movement_radius: 0.1   # 필요 이동 거리 축소 (기본: 0.3 m)
      movement_time_allowance: 30.0   # 허용 시간 증가 (기본: 15.0 sec)
```

**Collision Monitor가 너무 민감할 때:**
```yaml
collision_monitor:
  ros__parameters:
    StopCircle:
      min_points: 3             # 최소 감지 포인트 증가 (기본: 1)
    SlowCircle:
      slowdown_ratio: 0.7       # 감속 비율 완화 (기본: 0.5)
```

---

### 웨이포인트 파일 형식 (`waypoints/*.yaml`)

```yaml
waypoints:
  - id: 1
    x: 123.0
    y: 322.0
  - id: 2
    x: 139.0
    y: 322.0
  - id: 3
    x: 157.0
    y: 365.0
```

**파라미터 설명:**
- `id`: 웨이포인트 고유 식별자 (표시용, 순서는 리스트 순서 따름)
- `x`: 맵 좌표계 X 위치 (미터)
- `y`: 맵 좌표계 Y 위치 (미터)

## 토픽

### Goal Publisher

#### 구독 토픽
- `/clicked_point` (`geometry_msgs/PointStamped`)
  - RViz의 Publish Point 도구에서 발행하는 포인트
  - 웨이포인트 등록에 사용

#### 발행 토픽
- `/goal_pose` (`geometry_msgs/PoseStamped`)
  - Nav2 BT Navigator에 전달되는 목표 위치
  - QoS: Reliable, Volatile

- `/waypoint_markers` (`visualization_msgs/MarkerArray`)
  - RViz 시각화용 마커 배열
  - 웨이포인트 위치, 경로 라인, 번호 라벨, 현재 목표 하이라이트 포함
  - 발행 주기: 100ms (10Hz)

### Nav2 관련 토픽

#### 주요 입력 토픽
- `/scan` (`sensor_msgs/LaserScan`)
  - LiDAR 스캔 데이터 (장애물 감지용)

- `/map` (`nav_msgs/OccupancyGrid`)
  - 정적 맵 데이터

- `/odometry/imu` (`nav_msgs/Odometry`)
  - 오도메트리 데이터

#### 주요 출력 토픽
- `/cmd_vel` (`geometry_msgs/Twist`)
  - 최종 속도 명령 (Collision Monitor 출력)

- `/cmd_vel_nav` (`geometry_msgs/Twist`)
  - 네비게이션 속도 명령 (Controller 출력)

- `/local_costmap/costmap` (`nav_msgs/OccupancyGrid`)
  - 로컬 코스트맵

- `/global_costmap/costmap` (`nav_msgs/OccupancyGrid`)
  - 전역 코스트맵

## 서비스

### Goal Publisher 서비스

| 서비스 이름 | 타입 | 설명 |
|------------|------|------|
| `/auto_nav` | `std_srvs/Trigger` | 자동 네비게이션 시작 |
| `/stop_nav` | `std_srvs/Trigger` | 네비게이션 중지 |
| `/toggle_loop_mode` | `std_srvs/Trigger` | 주행 모드 전환 (OFF→LOOP→ROUND_TRIP→OFF) |
| `/list_waypoints` | `std_srvs/Trigger` | 등록된 웨이포인트 목록 출력 |
| `/remove_last_waypoint` | `std_srvs/Trigger` | 마지막 웨이포인트 삭제 |
| `/clear_all_waypoints` | `std_srvs/Trigger` | 모든 웨이포인트 삭제 |
| `/save_waypoints` | `nav2_navigation/SaveWaypoints` | 웨이포인트를 YAML 파일로 저장 |
| `/load_waypoints` | `nav2_navigation/LoadWaypoints` | YAML 파일에서 웨이포인트 로드 |

### 커스텀 서비스 메시지

#### SaveWaypoints.srv / LoadWaypoints.srv
```
# Request
string file_path
---
# Response
bool success
string message
```

## TF 프레임

Nav2 자율주행을 위해 필요한 TF 변환:
- `map` → `odom`: 동일 좌표계 (Static Transform)
- `odom` → `base_link`: 로봇의 위치 (위치 추정 노드에서 발행)

## 지원 맵

패키지에 포함된 2D 맵 디렉토리:
- `map_2D/campus/` - 캠퍼스 맵

각 맵 디렉토리에는 다음 파일들이 포함됩니다:
- `map.yaml` - 맵 메타데이터
- `map.pgm` 또는 `map.png` - 맵 이미지

## RViz 시각화

### 웨이포인트 마커 색상
- **녹색 구체**: 등록된 웨이포인트 위치
- **주황색 선**: 시작 위치와 웨이포인트를 연결하는 경로
- **흰색 텍스트**: 웨이포인트 번호 (1, 2, 3, ...)
- **빨간색 반투명 구체**: 현재 목표 웨이포인트 (네비게이션 중)

### 권장 RViz 설정
1. MarkerArray 추가 → Topic: `/waypoint_markers`
2. Map 추가 → Topic: `/map`
3. LaserScan 추가 → Topic: `/scan`
4. Path 추가 → Topic: `/plan` (전역 경로)
5. Polygon 추가 → Topic: `/polygon_stop`, `/polygon_slowdown` (충돌 감지 영역)

## 문제 해결

### 진단 플로우차트

로봇이 움직이지 않을 때 아래 순서로 점검하세요:

```
[로봇이 움직이지 않음]
         │
         ▼
┌─────────────────────────┐
│ 1. TF 트리 정상?        │
│ map → odom → base_link  │
└───────────┬─────────────┘
            │
     No ◄───┴───► Yes
      │            │
      ▼            ▼
  [TF 문제]   ┌─────────────────────┐
  → 1번 참조  │ 2. Nav2 노드 active? │
              └──────────┬──────────┘
                         │
                  No ◄───┴───► Yes
                   │            │
                   ▼            ▼
              [Lifecycle]  ┌──────────────────┐
              → 2번 참조   │ 3. 경로 생성됨?   │
                           │ /plan 토픽 확인  │
                           └────────┬─────────┘
                                    │
                             No ◄───┴───► Yes
                              │            │
                              ▼            ▼
                         [플래너 문제] ┌──────────────────┐
                         → 3번 참조   │ 4. cmd_vel 발행? │
                                      └────────┬─────────┘
                                               │
                                        No ◄───┴───► Yes
                                         │            │
                                         ▼            ▼
                                   [컨트롤러]    [하드웨어]
                                   → 4번 참조   → 5번 참조
```

### 공통 문제 

#### 1. TF 트리 문제
**증상:** "Could not get transform" 경고 메시지, 로봇 위치가 RViz에 표시되지 않음

**해결 방법:**
```bash
# TF 트리 확인 (map->odom->base_link 연결 확인)
ros2 run tf2_tools view_frames

# 특정 변환 실시간 확인
ros2 run tf2_ros tf2_echo map base_link
```

**체크리스트:**
- Localization 노드 (NDT)가 실행 중인지 확인
- `map` → `odom` → `base_link` 체인이 끊김 없이 연결되어 있는지 확인
- TF 발행 주기가 너무 느리지 않은지 확인

---

#### 2. Nav2 Lifecycle 문제
**증상:** Nav2 노드가 `inactive` 상태, 네비게이션 명령에 반응 없음

**해결 방법:**
```bash
# 각 Nav2 노드 상태 확인
ros2 lifecycle list /controller_server
ros2 lifecycle list /planner_server
ros2 lifecycle list /bt_navigator

# 노드가 inactive면 수동으로 활성화
ros2 lifecycle set /controller_server activate
ros2 lifecycle set /planner_server activate
```

**체크리스트:**
- `lifecycle_manager`가 정상 실행 중인지 확인
- 모든 Nav2 노드가 `active` 상태인지 확인
- 런치 파일에서 `autostart: True` 설정 확인

---

#### 3. 경로 생성 문제 (플래너)
**증상:** 목표 설정 후 경로가 표시되지 않음, "No valid path" 오류

**해결 방법:**
```bash
# 경로 토픽 확인
ros2 topic echo /plan

# 전역 코스트맵 확인
ros2 topic echo /global_costmap/costmap --once

# 맵 서버 상태 확인
ros2 service call /map_server/get_state lifecycle_msgs/srv/GetState
```

**체크리스트:**
- 맵이 정상적으로 로드되었는지 확인
- 목표 위치가 맵 내 유효한 (occupied가 아닌) 영역인지 확인
- 코스트맵의 inflation_radius가 너무 크지 않은지 확인
- 시작점과 목표점 사이에 경로가 존재하는지 확인

---

#### 4. 컨트롤러/cmd_vel 문제
**증상:** 경로는 생성되지만 로봇이 움직이지 않음

**해결 방법:**
```bash
# cmd_vel 토픽 발행 확인
ros2 topic echo /cmd_vel

# 컨트롤러 서버 상태 확인
ros2 topic echo /local_plan

# Collision Monitor 상태 확인
ros2 topic echo /collision_monitor_state
```

**체크리스트:**
- `/cmd_vel` 토픽에 속도 명령이 발행되는지 확인
- Collision Monitor가 로봇을 정지시키고 있지 않은지 확인
- 로컬 코스트맵에 잘못된 장애물이 없는지 확인
- 로봇 드라이버가 `/cmd_vel`을 구독하고 있는지 확인

---

#### 5. 하드웨어 연결 문제
**증상:** cmd_vel은 발행되지만 실제 로봇이 움직이지 않음

**해결 방법:**
```bash
# 로봇 드라이버 노드 확인
ros2 node list | grep driver

# 토픽 연결 상태 확인
ros2 topic info /cmd_vel
```

**체크리스트:**
- 로봇 드라이버 노드가 실행 중인지 확인
- `/cmd_vel` 토픽의 subscriber가 있는지 확인
- 로봇과의 통신 (시리얼/CAN 등) 상태 확인
- E-Stop이 눌려있지 않은지 확인

---

#### 6. GUI가 서비스에 연결되지 않음
**증상:** GUI 버튼 클릭 시 "Service not available" 메시지

**해결 방법:**
```bash
# 서비스 목록 확인
ros2 service list | grep -E "(auto_nav|stop_nav|waypoints)"

# goal_publisher 노드 실행 확인
ros2 node list | grep goal_publisher
```

**체크리스트:**
- `goal_publisher` 노드가 먼저 실행되었는지 확인
- 네임스페이스가 올바른지 확인
