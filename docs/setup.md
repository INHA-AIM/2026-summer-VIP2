# 팀원 셋업 가이드

> PC(WSL)에서 팀 레포를 clone하고, MORAI로 검증하기까지의 **공유 절차**.

---

## 1. 저장소 clone

```bash
git clone https://github.com/ahnsh03/26-summer-VIP2.git
cd 26-summer-VIP2
git checkout main && git pull
git checkout -b feature/<이름>-<기능>
```

GitHub 권한이 없으면 owner([@ahnsh03](https://github.com/ahnsh03))에게 collaborator 추가를 요청한다.

---

## 2. 필수 도구

| 도구 | 용도 |
|------|------|
| Git / `gh` | 브랜치·PR |
| **MORAI SIM 24.R2.H2** | 시뮬 (Windows) |
| **ROS Noetic** | 스택 빌드·실행 (Ubuntu 20.04 또는 Docker) |

### ROS (호스트)

```bash
./init_setting.sh   # tf2-sensor-msgs, rosbridge, ultralytics 등
./build.sh          # catkin_make (레포 루트 = workspace)
```

### ROS (Docker)

```bash
# 상세: docker/readme.txt
export VIP2="$(pwd)"
cd docker
docker compose -f docker-compose.pc.yaml up -d --build
docker exec -it vip2-noetic-pc bash
# 컨테이너: cd /root/vip2 && ./init_setting.sh && ./build.sh
```

---

## 3. 시나리오·맵 데이터

팀 레포에 포함됨:

| 경로 | 용도 |
|------|------|
| [`data/scenarios/test_scenario_summer.json`](../data/scenarios/test_scenario_summer.json) | Test Scenario |
| [`data/scenarios/map_init_default.json`](../data/scenarios/map_init_default.json) | Map Init |
| [`data/hdmap/R_KR_PG_KATRI/`](../data/hdmap/R_KR_PG_KATRI/) | HD 맵 참조 |

MORAI에 시나리오·map init을 로드한 뒤 ROS 스택을 켠다.

글로벌 주행 경로(플래너용): `src/main/config/track_log_recorded_final.csv` + `ref.txt`  
(`parameter_loader.cpp`가 레포 루트 기준 상대경로로 읽음 → **lattice는 반드시 레포 루트 cwd**에서 실행)

---

## 4. 실행 순서

별도 터미널에서:

```bash
cd /path/to/26-summer-VIP2
source scripts/ws_env.sh   # 또는 ./scripts/... 가 자동 source

./scripts/bridge.sh          # 1) ROS bridge (또는 ./scripts/grpc_bridge.sh)
./scripts/camera.sh          # 2) Camera
./scripts/lidar.sh           # 3) Lidar
source set_mode.sh va        # 4) Lattice 모드
./scripts/lattice_planner.sh # 5) PlanningControl → /ctrl_cmd_0
```

한 번에 센서 쪽만: `./scripts/start_system.sh` 후 lattice를 따로 실행.

정리: `./scripts/cleanup_ros.sh`

---

## 5. 일상 워크플로

1. `git checkout main && git pull`
2. `git checkout -b feature/<이름>-<기능>`
3. 코드 수정 · 시뮬 검증
4. `git push` → PR
5. merge 후 `main` pull

상세: [collaboration.md](./collaboration.md)

---

## 6. 로컬 루트 vs 팀 레포

| | 로컬 상위 폴더 | **이 레포** |
|--|----------------|-------------|
| Git | 없음 | 있음 |
| 내용 | PDF, external 참조 clone | docs + **대회 코드·data** |

셸 변수 예:

```bash
export VIP2_PROJECT="$HOME/projects/2026-summer-Vertically Integrated Project 2"
export VIP2="$VIP2_PROJECT/26-summer-VIP2"
```
