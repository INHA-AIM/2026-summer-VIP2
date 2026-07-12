# 26-summer-VIP2

**2026 하계 알파프로젝트 2** — A.I.M 팀 통합 저장소  
모라이 내부대회 자율주행 코드·팀 공유 문서.

| | |
|---|---|
| **GitHub** | https://github.com/ahnsh03/26-summer-VIP2 |
| **시뮬** | MORAI 24.R2.H2 · `2023_Hyundai_ioniq5` |
| **스택** | ROS Noetic · Lattice (`aim_ws` `va_seoyeon` 정리 이식) |

---

## 팀원이 먼저 읽을 문서

| 문서 | 내용 |
|------|------|
| **[docs/setup.md](docs/setup.md)** | clone · **Docker 권장** · MORAI · 실행 순서 ★ |
| **[docs/troubleshooting.md](docs/troubleshooting.md)** | 막히는 문제 FAQ ★ |
| **[docs/collaboration.md](docs/collaboration.md)** | 브랜치 · PR |
| **[docs/competition.md](docs/competition.md)** | 대회 규정 요약 |
| **[docs/repository-layout.md](docs/repository-layout.md)** | 디렉터리·패키지 |
| **[docs/tuning-notes.md](docs/tuning-notes.md)** | 경로·속도·미션 조정 |

---

## 빠른 시작 (WSL / Windows — Docker)

호스트 Ubuntu 26.04 등에는 ROS Noetic apt가 **없다**. Docker를 쓴다.

```bash
git clone https://github.com/ahnsh03/26-summer-VIP2.git
cd 26-summer-VIP2
export VIP2="$(pwd)"
cd docker && docker compose -f docker-compose.pc.yaml up -d --build
docker exec -it vip2-noetic-pc bash
# 컨테이너:
cd /root/vip2 && ./init_setting.sh && ./build.sh
./scripts/bridge.sh 9090          # 하나만 — MORAI Bridge 127.0.0.1:9090
# 다른 터미널: camera → lidar → set_mode → lattice_planner
```

시나리오: [`data/scenarios/`](data/scenarios/).  
상세·토픽표·RViz: [docs/setup.md](docs/setup.md).

---

## 저장소 구조

```
26-summer-VIP2/          # = catkin workspace root
├── data/scenarios/      # Test Scenario · Map Init
├── data/hdmap/          # R_KR_PG_KATRI
├── docs/
├── docker/              # Noetic + 9090 publish (MORAI rosbridge)
├── grpc_inha_univ/
├── scripts/
├── src/morai_msgs/
├── src/main/{config,src/{Camera,Lidar,PlanningControl}}
├── requirements-python.txt
├── build.sh · init_setting.sh · set_mode.sh
└── README.md
```

이식 시 제외: LBC, 실험 CSV, zip, 중복 YOLO 가중치.  
참고 clone(로컬): `../external/aim_ws-va_seoyeon/`.

---

## 제어 인터페이스 (규정)

허용: **`ctrl_cmd`**, **`MoraiEventCmdSrv`** 만.  
출력 토픽: `/ctrl_cmd_0`.  
리허설 기본 목표 속도: **20 km/h** (`parameter_loader.cpp`).
