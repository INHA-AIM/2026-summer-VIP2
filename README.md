# 26-summer-VIP2

**2026 하계 알파프로젝트 2** — A.I.M 팀 통합 저장소  
모라이 내부대회(2026.07.13) 자율주행 코드·팀 공유 문서를 담는다.

| | |
|---|---|
| **GitHub** | https://github.com/ahnsh03/26-summer-VIP2 |
| **하계학기** | 2026.06.22 ~ 07.15 |
| **대회** | 2026.07.13 / 학생회관 513 |
| **시뮬** | MORAI 24.R2.H2 · 차량 `2023_Hyundai_ioniq5` |
| **스택** | ROS Noetic · Lattice Planning (aim_ws `va_seoyeon` 기반) |

---

## 팀원이 먼저 읽을 문서

| 문서 | 내용 |
|------|------|
| **[docs/collaboration.md](docs/collaboration.md)** | Git 브랜치 · PR · 충돌 방지 |
| **[docs/competition.md](docs/competition.md)** | 모라이 내부대회 규정 요약 |
| **[docs/setup.md](docs/setup.md)** | clone · 빌드 · 실행 |
| **[docs/repository-layout.md](docs/repository-layout.md)** | 디렉터리·패키지 역할 |
| **[docs/tuning-notes.md](docs/tuning-notes.md)** | 하계 시나리오 조정 포인트 |

---

## 빠른 시작

```bash
git clone https://github.com/ahnsh03/26-summer-VIP2.git
cd 26-summer-VIP2
git checkout -b feature/<이름>-<기능>

# ROS Noetic 환경에서
./init_setting.sh   # 최초 1회
./build.sh

# 터미널별 실행 순서
./scripts/bridge.sh          # 또는 ./scripts/grpc_bridge.sh
./scripts/camera.sh
./scripts/lidar.sh
source set_mode.sh va
./scripts/lattice_planner.sh # cwd=레포 루트 필수 (config 상대경로)
```

시나리오 파일: [`data/scenarios/`](data/scenarios/) — MORAI에 로드.

협업 규칙: [docs/collaboration.md](docs/collaboration.md).

---

## 저장소 구조

```
26-summer-VIP2/          # = catkin workspace root
├── data/scenarios/      # Test Scenario · Map Init
├── data/hdmap/          # R_KR_PG_KATRI
├── docs/
├── docker/
├── grpc_inha_univ/      # /ctrl_cmd_0 → MORAI gRPC
├── scripts/             # bridge · camera · lidar · lattice …
├── src/
│   ├── morai_msgs/
│   └── main/
│       ├── config/      # Path · zone · yaml
│       └── src/         # Camera · Lidar · PlanningControl
├── build.sh · init_setting.sh · set_mode.sh
└── README.md
```

상세: [docs/repository-layout.md](docs/repository-layout.md)

---

## 제어 인터페이스 (규정)

허용: **`ctrl_cmd`**, **`MoraiEventCmdSrv`** 만.  
플래너 출력 토픽: `/ctrl_cmd_0` (`morai_msgs/CtrlCmd`).  
상세: [docs/competition.md](docs/competition.md)
