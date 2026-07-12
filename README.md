# 26-summer-VIP2

**2026 하계 알파프로젝트 2** — A.I.M 팀 통합 저장소  
모라이 내부대회(2026.07.13) 자율주행 코드·팀 공유 문서를 담는다.

| | |
|---|---|
| **GitHub** | https://github.com/ahnsh03/26-summer-VIP2 |
| **하계학기** | 2026.06.22 ~ 07.15 |
| **대회** | 2026.07.13 / 학생회관 513 |
| **지도** | 원종훈 교수 (Multi-Agent Mobility Simulator) |
| **시뮬** | MORAI 24.R2.H2 · 차량 `2023_Hyundai_ioniq5` |

> **개인 PC 상위 폴더**(PDF·시나리오 원본·참조 clone):  
> `~/projects/2026-summer-Vertically Integrated Project 2/` — git 없음, 팀 repo와 역할이 다름.

---

## 팀원이 먼저 읽을 문서

| 문서 | 내용 |
|------|------|
| **[docs/collaboration.md](docs/collaboration.md)** | Git 브랜치 · PR · 충돌 방지 ★ |
| **[docs/competition.md](docs/competition.md)** | 모라이 내부대회 규정 요약 ★ |
| **[docs/setup.md](docs/setup.md)** | clone · 로컬 경로 · 개발 환경 |

---

## 빠른 시작

```bash
git clone https://github.com/ahnsh03/26-summer-VIP2.git
cd 26-summer-VIP2

# 작업은 항상 feature 브랜치에서
git checkout main && git pull
git checkout -b feature/<이름>-<기능>
```

협업 규칙은 [docs/collaboration.md](docs/collaboration.md) 필독.

---

## 저장소 구조 (초기)

```
26-summer-VIP2/
├── README.md
├── docs/                 # 팀 공유 문서
│   ├── collaboration.md
│   ├── competition.md
│   └── setup.md
└── src/                  # 대회 코드 (통합 예정)
```

코드 레이아웃은 통합이 진행되면 `docs/`에 repository-layout을 추가한다.

---

## 제어 인터페이스 (규정)

허용: **`ctrl_cmd`**, **`MoraiEventCmdSrv`** 만.  
상세: [docs/competition.md](docs/competition.md)
