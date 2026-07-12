# 하계 시나리오 · 튜닝 메모

## 이식 범위 (검토 요약)

| 포함 | 제외 |
|------|------|
| PlanningControl, Lidar, Camera, morai_msgs | `learning_by_cheating/` |
| config Path/zone/yaml, `new_best.pt` | `*.zip`, 실험 CSV, 중복 `.pt` |
| 시나리오 JSON, KATRI HD맵 | Unity `.meta` |
| scripts, docker, grpc_inha_univ | MPC 브랜치 |

하드코딩 `src/main/config/...`를 깨지 않기 위해 **레포 루트 = catkin ws** 구조를 유지했다.

## 런타임 확인됨

| 항목 | 상태 |
|------|------|
| GPS 재밍 | `status==0` → `JammingPlanningProcess()` |
| 제어 | `/ctrl_cmd_0` only |
| 정지 훅 | `STOP_X/Y` 동계 코스값 — 하계 재조정 필요 |
| 리허설 목표 속도 | **20 km/h** / 곡선 12 km/h |
| 속도 피드백 | `egoCallback`: \|v\| + position 차분 (velocity≈0 과속 방지) |

## 조정 포인트

1. **글로벌 경로** — `map_init` ≈ `(-12.4, 977.7)`. track CSV는 ENU/`ref.txt` 기준. 코스 불일치 시 `src/main/config/` 교체.
2. **정지 미션** — `LatticePlanning.cpp` `STOP_X/Y` 하계 좌표로 갱신.
3. **속도** — `parameter_loader.cpp` / `yaml_hybrid.yaml`.
4. **MoraiEventCmdSrv** — 기어 등이 필요하면 허용 서비스만 추가.

운영 FAQ: [troubleshooting.md](./troubleshooting.md)
