# Lidar Localization

LiDAR 기반 3D 지도 작성 및 위치추정 ROS2 패키지. LIO-SAM 기반의 SLAM과 NDT 기반 맵 매칭을 통한 실시간 위치추정 기능을 제공합니다.

**참고:** 전체 시스템 통합 실행에 대한 자세한 내용은 [워크스페이스 메인 README](../../README.md)를 참고하세요.

## 개요

Lidar Localization은 자율주행 로봇 및 차량을 위한 LiDAR 기반 위치추정 시스템으로 다음 기능을 제공합니다:
- **3D 지도 작성 (Mapping)**: LiDAR-Inertial SLAM을 통한 고정밀 3D point cloud 지도 생성
- **고주파수 SLAM Pose 생성 (Pose Interpolation)**: SLAM으로 생성된 sparse한 pose를 odometry 주기에 맞춰 dense하게 보간
- **3D 지도 기반 위치추정 (Localization)**: 사전 제작된 3D 지도와 NDT 매칭을 통한 실시간 위치추정

## 구성 요소

### 1. 3D 지도 작성 모듈 (Mapping)
LIO-SAM 기반의 LiDAR-Inertial SLAM으로 3D point cloud 지도를 생성합니다.

**주요 기능:**
- **LiDAR-Inertial Odometry**: LiDAR 스캔 매칭과 IMU 사전적분을 결합한 odometry 생성
- **Loop Closure**: 이전에 방문한 장소 재인식을 통한 drift 보정
- **Graph SLAM**: GTSAM 기반 factor graph 최적화

**작동 방식:**
1. LiDAR 스캔과 IMU 데이터를 입력으로 받아 LiDAR-Inertial odometry 생성
2. Loop closure 검출 시 factor graph에 constraint 추가
3. Factor graph 최적화를 통해 전체 trajectory 보정
4. 최적화된 pose들로부터 3D point cloud 지도 생성

**구현 코드 위치:**
- IMU 사전적분: `src/imuPreintegration.cpp`
- 포인트 클라우드 처리: `src/imageProjection.cpp`
- 특징점 추출: `src/featureExtraction.cpp`
- 그래프 최적화: `src/mapOptmization.cpp`

### 2. 고주파수 SLAM Pose 생성 모듈 (Pose Interpolation)
SLAM으로 생성된 저주파수의 keyframe pose를 고주파수 odometry 주기에 맞춰 보간합니다.

**작동 방식:**
1. 입력 rosbag에서 SLAM path와 odometry 메시지 읽기
2. Odometry 간 상대 변환을 SLAM pose에 누적하여 고주파수 pose 생성
3. 보간된 pose를 출력 rosbag에 저장

**구현 코드 위치:** `src/utils/rosbag_with_gt.cpp`

### 3. 3D 지도 기반 위치추정 모듈 (Localization)
사전 제작된 3D point cloud 지도와 NDT 매칭을 통해 실시간 위치를 추정합니다.

**주요 기능:**
- **전역 지도 서버 (Global Map Server)**: PCD/PLY 형식의 3D 지도 로딩 및 배포
- **NDT 맵 매칭 (Map Matching)**: OpenMP 가속 NDT 알고리즘으로 scan-to-map 정합
- **초기 위치 설정**: RViz의 2D Pose Estimate를 통한 초기 위치 설정
- **위치추정 최적화**: IMU 사전적분과 NDT 결과를 융합한 최적화

**작동 방식:**
1. Global Map Server가 PCD 파일에서 3D 지도를 로딩하여 배포
2. RViz 또는 외부 소스로부터 초기 위치 (initial pose) 수신
3. LiDAR 스캔과 전역 지도 간 NDT 정합으로 위치 추정
4. IMU 사전적분으로 scan 사이의 고주파수 odometry 제공
5. GTSAM factor graph에 NDT 결과와 IMU factor 추가하여 최적화

**구현 코드 위치:**
- 전역 지도 서버: `src/utils/globalmapServer.cpp`
- NDT 맵 매칭: `src/mapMatching.cpp`
- 위치추정 최적화: `src/localizationOptmization.cpp`
- IMU 사전적분, 포인트 클라우드 처리, 특징점 추출: Mapping 모듈과 공유


## 설치

### 사전 요구사항
- Ubuntu 22.04
- ROS2 Humble

### 외부 라이브러리 설치

#### GTSAM
```bash
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

### 빌드

```bash
# 워크스페이스로 이동
cd <your_workspace>

# ROS 의존성 설치
rosdep update
rosdep install --from-paths src/lidar_localization --ignore-src -r -y

# 빌드
colcon build --packages-select lidar_localization

# Source
source install/setup.bash
```


## 사용법

### 1. 3D 지도 작성 (Mapping)

LiDAR와 IMU 데이터가 포함된 rosbag을 재생하면서 SLAM을 실행합니다.

**터미널 1: SLAM 실행**
```bash
ros2 launch lidar_localization run_mapping_launch.py
```

**터미널 2: SLAM 결과 녹화**
```bash
ros2 bag record /velodyne_points /odometry/imu /lidar_localization/mapping/path \
  --use-sim-time -o <output_bag_path>
```

**터미널 3: 센서 데이터 재생**
```bash
ros2 bag play <input_bag_path> --clock \
  --topics /velodyne_points /imu/data /gps/fix
```
- `/gps/fix`: GPS 토픽 (선택사항)

**터미널 4: 지도 저장** (rosbag 재생 완료 후)
```bash
ros2 service call /lidar_localization/save_map lidar_localization/srv/SaveMap \
  "{resolution: 0.01, destination: '<save_directory>'}"
```

**출력:**
- `<output_bag_path>/`: SLAM pose가 포함된 rosbag 파일
- `<save_directory>/GlobalMap.pcd`: 3D point cloud 지도 파일

### 2. 고주파수 SLAM Pose 생성

SLAM이 완료된 rosbag에서 고주파수로 보간된 SLAM pose를 생성합니다.

```bash
ros2 run lidar_localization lidar_localization_rosbagWithGt \
  <input_bag_path> \
  <output_bag_path> \
  [slam_path_topic] \
  [odometry_topic]
```

**파라미터:**
- `input_bag_path`: SLAM path가 포함된 입력 rosbag 경로
- `output_bag_path`: 보간된 pose가 추가된 출력 rosbag 경로
- `slam_path_topic`: (선택) SLAM path 토픽 (기본값: `/lidar_localization/mapping/path`)
- `odometry_topic`: (선택) Odometry 토픽 (기본값: `/odometry/imu`)

**예시:**
```bash
ros2 run lidar_localization lidar_localization_rosbagWithGt \
  ~/bags/slam_result \
  ~/bags/with_interpolated_pose \
  /lidar_localization/mapping/path \
  /odometry/imu
```

**출력:**
- `<output_bag_path>/`: 고주파수로 보간된 pose (`/ground_truth` 토픽)가 추가된 rosbag 파일

### 3. 3D 지도 기반 위치추정 (Localization)

사전 제작된 지도를 로딩하고 실시간 위치추정을 수행합니다.

#### 온라인 (실시간 센서 데이터)

```bash
ros2 launch lidar_localization run.launch.py \
  map_file:=/path/to/GlobalMap.pcd \
  use_sim_time:=false
```

#### 오프라인 (rosbag 테스트)

**터미널 1: 위치추정 실행**
```bash
ros2 launch lidar_localization run.launch.py \
  map_file:=/path/to/GlobalMap.pcd \
  use_sim_time:=true \
  rviz_config:=$(ros2 pkg prefix lidar_localization)/share/lidar_localization/config/rviz2_localization.rviz
```
**터미널 2: Static TF 실행**
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link velodyne
```

**터미널 3: rosbag 재생**
```bash
ros2 bag play <rosbag_path> --clock --topics /velodyne_points /imu/data
```

#### 초기 위치 설정
RViz2에서 "2D Pose Estimate" 도구를 사용하여 초기 위치를 설정합니다.


## 설정

### 매핑 설정 파일 (`config/params_mapping.yaml`)

#### 센서 설정
```yaml
# Topics
pointCloudTopic: "/velodyne_points"    # LiDAR 포인트 클라우드 토픽
imuTopic: "/imu/data"                  # IMU 데이터 토픽

# Sensor Settings
sensor: velodyne                        # LiDAR 센서 타입 (velodyne, ouster, livox)
N_SCAN: 16                              # LiDAR 채널 수
Horizon_SCAN: 1800                      # 수평 해상도
lidarMinRange: 1.0                      # 최소 거리 (m)
lidarMaxRange: 1000.0                   # 최대 거리 (m)
```

#### GPS 설정 (선택사항)
GPS를 사용하는 경우, 원점 좌표 및 토픽 설정은 launch 파일(`launch/run_mapping_launch.py`)에서 수정합니다.
```python
# GPS Transformation node
Node(
    package='lidar_localization',
    executable='lidar_localization_gpsTransformation',
    parameters=[
        {'gps_origin_lat': 37.58091508003259},   # GPS 원점 위도
        {'gps_origin_long': 127.02640766847915}, # GPS 원점 경도
        {'gps_origin_alt': 25.077},              # GPS 원점 고도
        {'gps_origin_yaw': 0.505401129400864},   # GPS 원점 yaw (rad)
    ],
    remappings=[
        ('/gps/fix', '/gps/fix'),      # 입력: GPS NavSatFix 토픽
        ('/odom/gps', '/odom/gps_rtk') # 출력: GPS odometry 토픽
    ],
)
```

#### IMU 설정
```yaml
# IMU Noise Parameters
imuAccNoise: 3.9939570888238808e-03     # 가속도 노이즈
imuGyrNoise: 1.5636343949698187e-03     # 자이로 노이즈
imuAccBiasN: 6.4356659353532566e-05     # 가속도 바이어스 노이즈
imuGyrBiasN: 3.5640318696367613e-05     # 자이로 바이어스 노이즈
imuGravity: 9.80511                     # 중력 가속도

# Extrinsic: LiDAR-IMU 변환
extrinsicTrans: [0.0, 0.0, 0.0]         # 변환 벡터 [x, y, z]
extrinsicRot: [1, 0, 0, 0, 1, 0, 0, 0, 1]  # 회전 행렬
```

#### 루프 클로저 설정
```yaml
loopClosureEnableFlag: true             # 루프 클로저 활성화
loopClosureFrequency: 0.5               # 루프 검출 주기 (Hz)
historyKeyframeSearchRadius: 30.0       # 루프 검색 반경 (m)
historyKeyframeFitnessScore: 0.2        # ICP fitness score 임계값
```

### 위치추정 설정 파일 (`config/params.yaml`)

센서 설정, IMU 설정은 매핑 설정 파일과 동일합니다.

#### NDT 매칭 설정
```yaml
# NDT Parameters
ndt_neighbor_search_method: "DIRECT7"   # 이웃 검색 방법 (KDTREE, DIRECT1, DIRECT7, DIRECT26)
number_of_threads_ndt: 8                # NDT 스레드 수
ndt_resolution: 1.0                     # NDT 해상도 (m)
lidar_downsample_resolution: 0.2        # LiDAR 다운샘플 해상도 (m)
```

#### 맵 매칭 설정
```yaml
map_matching_update_interval: 1.0       # 맵 매칭 주기 (sec)
matching_covariance_const: [0.1, 0.1, 0.01, 0.0001, 0.0001, 0.001]  # 공분산 상수
overlap_ratio_threshold: 0.5            # 초기 pose 추정 시 overlap 비율 임계값
map_matching_stale_time_threshold: 20.0 # 맵 매칭 stale 시간 임계값 (sec)
```


## 토픽

### 3D 지도 작성 (Mapping)

#### 발행 토픽
- `/lidar_localization/mapping/odometry` (`nav_msgs/Odometry`)
  - 최적화된 현재 pose 정보

- `/lidar_localization/mapping/path` (`nav_msgs/Path`)
  - 전체 경로 trajectory

- `/odometry/imu` (`nav_msgs/Odometry`)
  - IMU 사전적분 기반 고주파수 odometry

#### 구독 토픽
- `pointCloudTopic` (`sensor_msgs/PointCloud2`)
  - LiDAR 포인트 클라우드 데이터

- `imuTopic` (`sensor_msgs/Imu`)
  - IMU 센서 데이터

- `/gps/fix` (`sensor_msgs/NavSatFix`)
  - GPS 데이터 (선택사항, gpsTransformation 노드에서 사용)

### 3D 지도 기반 위치추정 (Localization)

#### 발행 토픽
- `/lidar_localization/mapping/odometry` (`nav_msgs/Odometry`)
  - 위치추정 결과 pose 정보 (LiDAR scan 주기 기준)

- `/lidar_localization/mapping/path` (`nav_msgs/Path`)
  - 추정된 경로 trajectory

- `/odometry/imu` (`nav_msgs/Odometry`)
  - IMU 사전적분 기반 고주파수 odometry

- `/tf` (`tf2_msgs/TFMessage`)
  - 좌표 변환 정보 (odom → base_link)

- `/globalmap` (`sensor_msgs/PointCloud2`)
  - 로딩된 전역 3D 지도 (transient local QoS)

#### 구독 토픽
- `pointCloudTopic` (`sensor_msgs/PointCloud2`)
  - LiDAR 포인트 클라우드 데이터

- `imuTopic` (`sensor_msgs/Imu`)
  - IMU 센서 데이터

- `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`)
  - RViz에서 설정하는 초기 위치

- `/globalmap` (`sensor_msgs/PointCloud2`)
  - 전역 지도 수신


## 문제 해결

### 빌드 및 실행 오류

#### GTSAM 관련 오류
```bash
# GTSAM 라이브러리 경로 확인
echo $LD_LIBRARY_PATH

# 경로가 설정되지 않은 경우
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

### 런타임 문제

#### 1. 지도 및 위치추정 시 Large velocity, reset IMU-preintegration! 가 출력되면서 위치가 발산하는 경우
**증상:** SLAM 결과 지도가 왜곡되거나 위치추정 발산이 발생

**해결 방법:**
- IMU 캘리브레이션 파라미터 확인
  ```yaml
  imuAccNoise: <calibrated_value>
  imuGyrNoise: <calibrated_value>
  ```
- LiDAR-IMU extrinsic 변환 확인

#### 2. 위치추정이 발산하는 경우
**증상:** 위치추정 결과가 지도와 맞지 않거나 발산

**체크리스트:**
- 초기 위치가 올바르게 설정되었는지 확인 (3D 상에서 위치가 초기화가 잘 되었는지 확인)
- 전역 지도가 정상적으로 로딩되었는지 확인
- LiDAR 데이터나 IMU 데이터가 정상적으로 수신되는지 확인

#### 3. 전역 지도가 표시되지 않는 경우
**증상:** RViz에서 전역 지도가 보이지 않음

**해결 방법:**
```bash
# 지도 토픽 확인
ros2 topic echo /globalmap --once

# 노드 상태 확인
ros2 node info /globalmap_server
```

**체크리스트:**
- PCD 파일 경로가 올바른지 확인
- RViz의 PointCloud2 디스플레이 설정 확인
- Fixed frame이 "map"으로 설정되어 있는지 확인

#### 4. TF 변환 오류
**증상:** TF 관련 경고 또는 오류 메시지 출력

**해결 방법:**
```bash
# TF 트리 확인
ros2 run tf2_tools view_frames

# 특정 변환 확인
ros2 run tf2_ros tf2_echo map base_link
```

**체크리스트:**
- `map` → `odom` → `base_link` → `velodyne` 변환 체인 확인
- Static transform publisher가 실행 중인지 확인
