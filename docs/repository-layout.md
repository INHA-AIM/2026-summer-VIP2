# 저장소 레이아웃

레포 루트 = **catkin 워크스페이스 루트**.  
하드코딩된 `src/main/config/...` 상대경로가 cwd=레포 루트일 때 동작한다.

## 디렉터리

| 경로 | 역할 |
|------|------|
| `data/scenarios/` | `test_scenario_summer.json`, `map_init_default.json` (MORAI 로드) |
| `data/hdmap/R_KR_PG_KATRI/` | HD 맵 JSON 참조 |
| `src/morai_msgs/` | MORAI 메시지 (`CtrlCmd`, GPS, Ego 등) |
| `src/main/config/` | 글로벌 경로·zone·yaml·rviz |
| `src/main/src/Camera/` | YOLO 차선 + camera costmap → `/lane/path`, `/costmap/camera` |
| `src/main/src/Lidar/` | PCL → `/costmap` |
| `src/main/src/PlanningControl/` | Lattice / Jamming / Control → `/ctrl_cmd_0` |
| `scripts/` | 실행 스크립트 (`ws_env.sh`가 `WS_ROOT` 설정) |
| `grpc_inha_univ/` | ROS `/ctrl_cmd_0` → MORAI gRPC |
| `docker/` | ROS Noetic 컨테이너 |

## 데이터 흐름

```text
MORAI (scenarios + sensors)
  → Camera / Lidar
  → PlanningControl (+ src/main/config Path)
  → /ctrl_cmd_0
  → (optional) grpc_bridge → MORAI
```

## 실험 모드 (`set_mode.sh`)

| 인자 | `EXPERIMENT_MODE` | 내용 |
|------|-------------------|------|
| `static` / `0` | 0 | Static Lattice |
| `va` / `1` | 1 | VA-MDS Lattice |
| `fusion` / `2` | 2 | VA + Fusion Costmap |
| `original` / `3` | 3 | Dynamic LD |

## 제외한 레거시

`learning_by_cheating`, 실험 CSV 덤프, `*.zip`, 중복 YOLO 가중치(`best.pt` 등)는 이식하지 않았다.  
참고 clone: 상위 폴더 `external/aim_ws-va_seoyeon/`.
