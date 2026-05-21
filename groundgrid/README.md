# GroundGrid

LiDAR 포인트 클라우드 기반 2D occupancy grid 지도 생성 ROS2 패키지. Ground segmentation과 장애물 검출을 통해 자율주행용 2D 지도를 생성합니다.

**참고:** 전체 시스템 통합 실행에 대한 자세한 내용은 [워크스페이스 메인 README](../../README.md)를 참고하세요.

## 개요

GroundGrid는 LiDAR 스캔과 로봇 pose를 입력으로 받아 2D occupancy grid 지도를 생성합니다:
- **Ground Segmentation**: LiDAR 포인트 클라우드에서 지면과 장애물 분리
- **Occupancy Map 생성**: 지면은 free space, 장애물은 occupied로 표시하여 2D 지도 생성
- **Nav2 호환 출력**: `map.pgm` + `map.yaml` 형식으로 저장

## 구성 요소

### 1. GroundGrid Node
LiDAR 포인트 클라우드에서 지면을 분할하고 terrain estimation을 수행합니다.

**작동 방식:**
1. LiDAR 스캔과 odometry를 입력으로 받음
2. Grid 기반 ground segmentation 수행
3. 지면/비지면 포인트 분류 결과와 grid map 발행

**구현 코드 위치:** `src/GroundGrid.cpp`, `src/GroundSegmentation.cpp`

### 2. OccupancyMapGenerator Node
Ground segmentation 결과를 누적하여 2D occupancy grid 지도를 생성합니다.

**작동 방식:**
1. GroundGrid에서 발행하는 grid map과 segmented cloud 수신
2. 지면 영역을 free space로, 장애물을 occupied로 누적
3. 주기적으로 `map.pgm` + `map.yaml` 파일 저장 (10초 간격)

**구현 코드 위치:** `src/OccupancyMapGenerator.cpp`


## 설치

### 사전 요구사항
- Ubuntu 22.04
- ROS2 Humble

### 빌드

```bash
# 워크스페이스로 이동
cd <your_workspace>

# ROS 의존성 설치
rosdep update
rosdep install --from-paths src/groundgrid --ignore-src -r -y

# 빌드
colcon build --packages-select groundgrid

# Source
source install/setup.bash
```


## 사용법

고주파수 SLAM pose (`/ground_truth` 토픽)가 포함된 rosbag을 사용하여 2D 지도를 생성합니다.

**터미널 1: GroundGrid 및 OccupancyMapGenerator 실행**
```bash
ros2 launch groundgrid groundgrid_with_occupancy.launch.py \
  save_path:=<map_save_directory> \
  x_min:=<x_min> x_max:=<x_max> \
  y_min:=<y_min> y_max:=<y_max>
```

**터미널 2: rosbag 재생**
```bash
ros2 bag play <rosbag_path> --clock --topics /velodyne_points /ground_truth /tf
```

**출력:**
- `<map_save_directory>/map.pgm`: 2D occupancy grid 이미지
- `<map_save_directory>/map.yaml`: Nav2 호환 지도 메타데이터

### 런치 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `odom_topic` | `/ground_truth` | Odometry 토픽 |
| `pointcloud_topic` | `/velodyne_points` | LiDAR 포인트 클라우드 토픽 |
| `sensor_frame` | `velodyne` | LiDAR 센서 프레임 |
| `resolution` | `0.1` | 지도 해상도 (m/pixel) |
| `save_path` | `/data2/` | 지도 저장 경로 |
| `x_min`, `x_max` | `0.0`, `400.0` | X축 범위 (m) |
| `y_min`, `y_max` | `0.0`, `400.0` | Y축 범위 (m) |


## 파라미터 설정

알고리즘 파라미터는 `config/groundgrid.yaml`에서 설정합니다. 전체 파라미터 목록은 해당 파일을 참고하세요.

### GroundGrid 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `thread_count` | 8 | 병렬 처리 스레드 수 (CPU 코어 수에 맞게 조정) |
| `miminum_point_height_threshold` | 0.05 | ground + threshold 이하를 ground로 분류 [m] |
| `minimum_point_height_obstacle_threshold` | 0.02 | 장애물 판정 최소 높이 [m] |
| `outlier_tolerance` | 0.03 | Outlier 검출 허용 오차 [m] |

### OccupancyMapGenerator 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `resolution` | 0.1 | 지도 해상도 [m/pixel] |
| `obstacle_max_height` | 1.5 | 센서 기준 최대 장애물 높이 [m] (이 높이 이상은 무시) |
| `obstacle_min_radius` | 3.0 | 센서 기준 최소 장애물 거리 [m] (이 거리 이내는 무시) |
| `obstacle_count_threshold` | 3 | 셀을 occupied로 표시하기 위한 최소 장애물 포인트 수 |

### 런타임 파라미터 변경

```bash
# GroundGrid 파라미터
ros2 param set /groundgrid thread_count 16
ros2 param set /groundgrid miminum_point_height_threshold 0.08

# OccupancyMapGenerator 파라미터
ros2 param set /occupancy_map_generator obstacle_max_height 3.0
ros2 param set /occupancy_map_generator obstacle_min_radius 2.0
```


## 토픽

### GroundGrid Node

#### 발행 토픽
- `groundgrid/grid_map` (`grid_map_msgs/GridMap`)
  - Ground segmentation 결과 grid map

- `groundgrid/segmented_cloud` (`sensor_msgs/PointCloud2`)
  - 지면/비지면 분류된 포인트 클라우드

#### 구독 토픽
- `pointcloud_topic` (`sensor_msgs/PointCloud2`)
  - LiDAR 포인트 클라우드

- `odom_topic` (`nav_msgs/Odometry`)
  - 로봇 odometry

- `/tf` (`tf2_msgs/TFMessage`)
  - 좌표 변환 정보 (map → base_link, map → sensor_frame)

### OccupancyMapGenerator Node

#### 발행 토픽
- `occupancy_map/grid` (`nav_msgs/OccupancyGrid`)
  - 실시간 occupancy grid 지도

- `occupancy_map/image` (`sensor_msgs/Image`)
  - Occupancy map 이미지

#### 구독 토픽
- `groundgrid/grid_map` (`grid_map_msgs/GridMap`)
  - GroundGrid의 ground segmentation 결과

- `groundgrid/segmented_cloud` (`sensor_msgs/PointCloud2`)
  - GroundGrid의 segmented 포인트 클라우드

- `odom_topic` (`nav_msgs/Odometry`)
  - 로봇 odometry (기본값: `/ground_truth`)


## 문제 해결

### 1. 지도가 생성되지 않는 경우
**증상:** `map.pgm` 파일이 생성되지 않거나 빈 이미지

**체크리스트:**
- `/ground_truth` 토픽이 정상적으로 발행되는지 확인
- `/velodyne_points` 토픽이 정상적으로 발행되는지 확인
- `save_path` 경로에 쓰기 권한이 있는지 확인

```bash
# 토픽 확인
ros2 topic echo /ground_truth --once
ros2 topic echo /velodyne_points --once
```

### 2. 지도 범위가 맞지 않는 경우
**증상:** 지도가 잘리거나 너무 큰 경우

**해결 방법:**
- `x_min`, `x_max`, `y_min`, `y_max` 런치 파라미터 조정

