# 하계 시나리오 · 튜닝 메모

## 이식 범위 (검토 요약)

| 포함 | 제외 |
|------|------|
| PlanningControl, Lidar, Camera, morai_msgs | `learning_by_cheating/` |
| config Path/zone/yaml, `best.pt`(차선) + `new_best.pt`(road) | `*.zip`, 실험 CSV |
| 시나리오 JSON, KATRI HD맵 | Unity `.meta` |
| scripts, docker, grpc_inha_univ | MPC 브랜치 |

하드코딩 `src/main/config/...`를 깨지 않기 위해 **레포 루트 = catkin ws** 구조를 유지했다.

## 런타임 확인됨

| 항목 | 상태 |
|------|------|
| GPS 재밍 | `status==0` → `JammingPlanningProcess()` |
| 제어 | `/ctrl_cmd_0` only |
| 정지 훅 | 경로 **마지막 웨이포인트** 기준 — 25 m 이내 15 km/h, 6 m 이내 정지+latch (`applyPathEndStop`) |
| 목표 속도 | external과 동일 **60 km/h** / 곡선 40 km/h |
| 속도 피드백 | `egoCallback`: `ego.vel = msg->velocity.x` |

## 조정 포인트

1. **글로벌 경로** — `map_init` ≈ `(-12.4, 977.7)`. track CSV는 ENU/`ref.txt` 기준. 코스 불일치 시 `src/main/config/` 교체.
2. **코스 종료 정지** — CSV 끝점이 곧 정지점. 감속/정지 거리는 `applyPathEndStop`의 `SLOW_DIST`/`STOP_DIST`.
3. **속도** — `parameter_loader.cpp` / `yaml_hybrid.yaml`.
4. **MoraiEventCmdSrv** — 기어 등이 필요하면 허용 서비스만 추가.

운영 FAQ: [troubleshooting.md](./troubleshooting.md)
