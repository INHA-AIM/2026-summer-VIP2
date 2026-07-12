# src/ — 대회 자율주행 스택

catkin 패키지 소스. 레포 루트에서 `./build.sh`로 빌드한다.

| 패키지 | 경로 | 역할 |
|--------|------|------|
| `morai_msgs` | `morai_msgs/` | MORAI 메시지 |
| `Camera` | `main/src/Camera/` | 차선·카메라 costmap |
| `Lidar` | `main/src/Lidar/` | LiDAR costmap |
| `PlanningControl` | `main/src/PlanningControl/` | Lattice · GPS 재밍 · `ctrl_cmd` |

설정·경로는 `main/config/`. 제어 출력은 **`/ctrl_cmd_0`만** (규정: `ctrl_cmd` / `MoraiEventCmdSrv`).

레이아웃 상세: [../docs/repository-layout.md](../docs/repository-layout.md)
