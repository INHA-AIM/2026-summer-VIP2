# 트러블슈팅 (팀 공유)

실습·리허설에서 반복된 문제만 모았다.

## 빌드 / 환경

### `Unable to locate package ros-noetic-...`
호스트가 Ubuntu 20.04가 아니다 (예: WSL 26.04).  
→ `docs/setup.md` Docker 절. 호스트에서 `./init_setting.sh` 하지 말 것.

### 컨테이너 recreate 후 Camera/YOLO 실패
pip 설치는 이미지에 안 남아 있을 수 있다.  
→ `./init_setting.sh` 또는 `pip3 install -r requirements-python.txt`

### `Cannot load message class for morai_msgs/...`
```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash   # 또는 source scripts/ws_env.sh
```

## MORAI ↔ ROS

### `connected_clients` 가 비어 있음
1. `./scripts/bridge.sh 9090` 가 **하나**만 떠 있는지  
2. Docker 포트 `0.0.0.0:9090->9090` (`docker ps`)  
3. MORAI Bridge `127.0.0.1:9090` Disconnect → Connect  
4. Windows: `netstat -an | findstr 9090`

### Connected인데 `/gps` `/lidar3D` 없음
Sensor 장착 ≠ Network Publisher.  
Publisher 목록에서 토픽명을 `/gps`, `/imu`, `/lidar3D`, `/image_jpeg/compressed`, `/Ego_topic`으로 ON.

### `client_count` 가 30+
MORAI가 토픽마다 WebSocket을 연다. **정상**. 스택 중복이 아님.  
`rosnode list`에 `lattice`/`rosbridge`가 각각 1개인지 보면 된다.

### bridge를 두 번 켬
로그에 `new node registered with same name` → 기존 브리지 종료·전 클라이언트 끊김.  
→ bridge 하나만, MORAI 재Connect.

### 센서 50 Hz → 코스 이탈 / `TF_REPEATED_DATA` 폭주
원인: Camera·LiDAR를 **50 Hz**로내면 rosbridge(WebSocket JSON)와 YOLO/costmap이 따라가지 못한다.  
메시지가 쌓이거나 지연·중복 타임스탬프가 나고, planner(약 10 Hz)가 보는 GPS/자세/costmap이 어긋나 **시나리오 시작 직후 경로 이탈**처럼 보인다.

대응 (팀 표준):

| 센서 | 권장 |
|------|------|
| Camera / LiDAR / IMU | **10 Hz** (`sensorPeriod` 0.1) |
| GPS | **5 Hz** (`sensorPeriod` 0.2) |

프리셋: [`data/sensors/24.R2.H2/VIP2.json`](../data/sensors/24.R2.H2/VIP2.json)  
확인: `rostopic hz /lidar3D /image_jpeg/compressed /imu /gps`

## 주행 / 제어

### 목표 속도를 넘겨 계속 가속 (accel≈1.0)
원인: `ego.vel` 피드백이 0에 가까움 (`/Ego_topic` velocity).  
대응: `main.cpp` egoCallback에서 속도 크기 + 위치 차분 보정. lattice 재빌드·재시작 후:

```bash
rostopic echo /Ego_topic      # 달릴 때 position이 변하는지
rostopic echo /ctrl_cmd_0     # 속도 오르면 accel이 줄어야 함
```

### costmap / lattice 대기
`/lidar3D` hz 확인. Lidar 노드 출력 없음은 정상(콜백형).

### 직진만 / 경로 무시
GPS·IMU Publisher, `ref.txt`/`track_log_recorded_final.csv`와 하계 map_init 좌표 정합 확인.  
[tuning-notes.md](./tuning-notes.md)

## RViz

```bash
# 컨테이너, planner 기동 후 (DISPLAY는 ws_env.sh 기본값)
source scripts/ws_env.sh
./scripts/rviz_visualization.sh
```

- 호스트: `xhost +local:` (WSLg)  
- `DISPLAY=host.docker.internal:0` → Windows X(VcXsrv) 필요할 때  
- GLX 실패 시 Foxglove → `ws://localhost:9090`
