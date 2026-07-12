# 팀원 셋업 가이드

> WSL/Windows + MORAI로 대회 스택을 돌리기 위한 **팀 공통 절차**.  
> 실습에서 막혔던 내용(Ubuntu 26.04, Docker Desktop 포트, rosbridge, 속도 피드백)을 반영했다.

---

## 0. 환경 한 줄 요약

| 환경 | 권장 |
|------|------|
| Windows + WSL (Ubuntu 22/24/**26**) | **Docker Noetic만** 사용. 호스트에 `ros-noetic-*` apt 설치 불가 |
| Ubuntu **20.04** 네이티브 | 호스트 Noetic 또는 Docker 모두 가능 |
| MORAI | Windows, Bridge **`127.0.0.1:9090`** (rosbridge WebSocket) |

레포 루트 = catkin workspace. lattice는 **반드시 레포 루트 cwd**에서 실행.

---

## 1. clone

```bash
git clone https://github.com/ahnsh03/26-summer-VIP2.git
cd 26-summer-VIP2
git checkout main && git pull
git checkout -b feature/<이름>-<기능>
```

---

## 2. Docker (권장 — WSL 26.04 포함)

Docker Desktop → **Settings → Resources → WSL integration** 에서 사용 distro Enable.

```bash
cd /path/to/26-summer-VIP2
export VIP2="$(pwd)"
cd docker
docker compose -f docker-compose.pc.yaml up -d --build
docker ps --filter name=vip2-noetic-pc   # 0.0.0.0:9090->9090 확인
docker exec -it vip2-noetic-pc bash
```

컨테이너 안 (최초 1회):

```bash
cd /root/vip2
./init_setting.sh    # rosbridge/pcl + Python(YOLO) — Noetic 있는 환경에서만
./build.sh
```

> `docker compose ... --force-recreate` 하면 **컨테이너 안 apt/pip 설치가 날아갈 수 있다.**  
> Dockerfile에 rosbridge/pcl은 포함되어 있다. Python은 `./init_setting.sh`를 다시 실행.

상세: [../docker/readme.txt](../docker/readme.txt)

---

## 3. MORAI 네트워크·센서

1. 시나리오: `data/scenarios/test_scenario_summer.json`  
2. Map init: `data/scenarios/map_init_default.json`  
3. **Ego Network**
   - Middleware: **ROS**
   - Bridge IP: `127.0.0.1` / PORT: **`9090`**
   - Connect (초록 Connected)
4. Publisher 토픽 (이름 일치 필수)

| 센서 | Topic |
|------|--------|
| GPS | `/gps` |
| IMU | `/imu` |
| Ego | `/Ego_topic` |
| LiDAR | `/lidar3D` |
| Camera | `/image_jpeg/compressed` |
| 제어(권장) | `/ctrl_cmd_0` (`morai_msgs/CtrlCmd`) |

Sensor Setting에 센서를 붙인 것과 Network **Publisher ON**은 다르다.  
`client_count`가 20~40이어도 정상(토픽별 WebSocket). bridge를 **두 번** 켜면 기존 연결이 끊긴다.

---

## 4. 실행 순서 (컨테이너 터미널 여러 개)

각 터미널:

```bash
docker exec -it vip2-noetic-pc bash
cd /root/vip2
source scripts/ws_env.sh
```

| 순서 | 명령 | 주의 |
|------|------|------|
| 1 | `./scripts/bridge.sh 9090` | **하나만**. 유지 후 MORAI Connect |
| 2 | `./scripts/camera.sh` | YOLO 필요 시 init_setting 선행 |
| 3 | `./scripts/lidar.sh` | 출력 거의 없음 = 정상 |
| 4 | `source set_mode.sh va` → `./scripts/lattice_planner.sh` | cwd=`/root/vip2` |
| 확인 | 아래 진단 | |

```bash
rostopic echo /connected_clients          # clients 비어 있으면 MORAI 미연결
rostopic hz /gps /imu /Ego_topic /lidar3D /costmap
rostopic echo /ctrl_cmd_0                 # accel이 항상 1.0이면 속도 피드백 의심
```

메시지 클래스 오류 시:

```bash
source /opt/ros/noetic/setup.bash
source /root/vip2/devel/setup.bash
```

정리: `./scripts/cleanup_ros.sh`

---

## 5. RViz

Docker Desktop + Windows에서는 OpenGL(GLX) 때문에 RViz가 자주 실패한다.

**WSLg 시도** (호스트 `xhost +` 후 컨테이너):

```bash
export DISPLAY=:0
export QT_X11_NO_MITSHM=1
unset LIBGL_ALWAYS_INDIRECT
rviz -d src/main/config/lattice_planner.rviz
```

`host.docker.internal:0` 은 VcXsrv 등 Windows X 서버가 있을 때만.  
대안: Windows **Foxglove Studio** → `ws://localhost:9090`.

---

## 6. 알려진 이슈 (팀 공통)

| 증상 | 원인 / 대응 |
|------|-------------|
| `Unable to locate package ros-noetic-*` | 호스트가 20.04가 아님 → Docker 사용 |
| MORAI Connected인데 `/gps` 없음 | Publisher 토픽명/ON 확인, bridge 단일 실행 |
| costmap 안 나옴 | `/lidar3D` Publishers 확인 |
| 목표 속도 지나 계속 가속 | `/Ego_topic` velocity≈0 → 위치 차분 보정으로 보완함. 재빌드·lattice 재시작 |
| bridge 재실행 후 clients=[] | 중복 bridge로 끊김 → Disconnect/Connect |
| 기본 목표 속도 | 리허설용 **20 km/h** (`parameter_loader.cpp`) |

더 자세한 조정: [tuning-notes.md](./tuning-notes.md) · [troubleshooting.md](./troubleshooting.md)

---

## 7. 일상 워크플로

1. `git checkout main && git pull`  
2. `git checkout -b feature/<이름>-<기능>`  
3. 수정 · 시뮬 검증  
4. PR  

[collaboration.md](./collaboration.md)
