# 하계 시나리오 맞춤 — 이식 직후 점검 메모

호스트에 ROS Noetic이 없어 `catkin_make` 스모크는 Docker/팀원 PC에서 수행한다.  
코드·경로·미션 훅은 아래처럼 확인했다.

## 확인됨

| 항목 | 상태 |
|------|------|
| GPS 재밍 | `gpsCallback`: `status==0` → `JammingPlanningProcess()` |
| 제어 출력 | `/ctrl_cmd_0` (`morai_msgs/CtrlCmd`)만 advertise |
| 정지 좌표 훅 | `LatticePlanning.cpp` `STOP_X/Y = (47.33, -96.62)` (ENU, 동계 코스값) |
| 시나리오 파일 | `data/scenarios/` 포함 |
| YOLO | `new_best.pt` + `rospkg`로 패키지 경로 탐색 |

## 시뮬 리허설 전 조정 포인트

1. **글로벌 경로** — `map_init` ego ≈ `(-12.4, 977.7)` (맵 절대좌표). 플래너 경로는 `ref.txt` GPS 원점 기준 ENU(`track_log_recorded_final.csv` 선두 ≈ `(-0.7, -78)`). 좌표계는 설계상 분리되어 있으나, **하계 코스와 track CSV가 동일 코스인지** MORAI에서 ego vs local path를 맞춰 볼 것. 어긋나면 `src/main/config/` Path/track/ref 교체.
2. **정지 미션** — `STOP_X/Y`는 구 코스 하드코딩. 하계 “정지 표시 장애물” 위치에 맞게 갱신하거나, costmap 근접 정차로 대체.
3. **속도** — `parameter_loader.cpp` `target_vel`/`curve_vel` (현재 60/40 km/h 급). Ioniq5·대회 코스에 맞게 소폭 하향 권장.
4. **MoraiEventCmdSrv** — 현재 미사용. 기어/자율모드 전환이 필요하면 허용 서비스만 추가.

## 빌드 스모크

호스트에 ROS Noetic 없음. Docker `ros:noetic-ros-base`에서:

- **성공:** `morai_msgs` · `PlanningControl` (`lattice_test_node`) · `Camera` (`camera_costmap`)
- **보류:** `Lidar` — 스모크 환경에서 `ros-noetic-pcl-ros` apt 미러(mesa) 실패. 팀원 Noetic/Docker desktop 이미지에서는 `./build.sh`로 전체 빌드.

```bash
export VIP2="$(pwd)"
cd docker && docker compose -f docker-compose.pc.yaml up -d --build
docker exec -it vip2-noetic-pc bash -lc 'cd /root/vip2 && ./init_setting.sh && ./build.sh'
```