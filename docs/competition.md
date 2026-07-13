# 모라이 내부대회 — 팀 공유 요약

> **대회:** 2026 하계 모라이 내부대회  
> **일시·장소:** **2026.07.13** / 학생회관 **513**  
> **원본 PDF**는 개인 작업 폴더 `docs/모라이_내부대회_규정집.pdf`에 보관. 공지 변경 시 이 문서도 갱신.

하계 알파프로젝트 2 (06.22 ~ 07.15) 동안 이 대회를 준비한다.

---

## 1. 시뮬 · 차량 · 통신

| 항목 | 규정 |
|------|------|
| Simulator | **MORAI 24.R2.H2** |
| 허용 통신 | **`ctrl_cmd`**, **`MoraiEventCmdSrv`만** |
| 차량 | **2023_Hyundai_ioniq5** |
| GPS / IMU | 각 **1개** |
| Camera / LiDAR | **자유** |
| 센서 장착 | 차체 기준 **0.4 m 이내** |

코드를 추가할 때 제어·이벤트 경로가 위 두 인터페이스를 벗어나지 않는지 PR에서 확인한다.

---

## 2. 미션

1. **주행 경로** — 지정 코스 자율주행  
2. **GPS 재밍** — 해당 구간 GPS **값 0**, **타 센서만**으로 주행 / 구간 좌표는 추후 재공지 / 재밍 구간 **장애물 없음**  
3. **정지** — 정지 표시 장애물 앞 정지  
4. **동적 장애물**  
5. **정적 장애물**  
6. **동·정적 혼합 구간**

---

## 3. 시나리오 파일

| 파일 | 용도 | 팀 레포 경로 |
|------|------|-------------|
| `test_scenario_summer.json` | Test Scenario | [`data/scenarios/test_scenario_summer.json`](../data/scenarios/test_scenario_summer.json) |
| `map_init_default.json` | Map Init (초기 위치·자세) | [`data/scenarios/map_init_default.json`](../data/scenarios/map_init_default.json) |

HD 맵 참조: [`data/hdmap/R_KR_PG_KATRI/`](../data/hdmap/R_KR_PG_KATRI/)  
플래너 글로벌 경로: `src/main/config/` (`track_log_recorded_final.csv`, `ref.txt`, zone CSV)

MORAI Network: Bridge `127.0.0.1:9090`, 센서 Publisher 토픽 `/gps` `/imu` `/Ego_topic` `/lidar3D` `/image_jpeg/compressed`, 제어 `/ctrl_cmd_0`.  
센서 프리셋(10 Hz): [`data/sensors/24.R2.H2/VIP2.json`](../data/sensors/24.R2.H2/VIP2.json).  
셋업·트러블슈팅: [setup.md](./setup.md), [troubleshooting.md](./troubleshooting.md)

원본 PDF는 개인 작업 폴더에 보관. 공지 변경 시 이 문서도 갱신.

---

## 4. 개발 체크리스트

- [ ] 24.R2.H2 + Ioniq5 센서 구성
- [ ] `ctrl_cmd` / `MoraiEventCmdSrv` only
- [ ] 시나리오·map init으로 코스 리허설
- [ ] GPS=0 fallback
- [ ] 정지 / 정적 / 동적 / 혼합 각각 통과

협업 절차: [collaboration.md](./collaboration.md)
