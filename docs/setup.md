# 팀원 셋업 가이드

> PC(WSL)에서 팀 레포를 clone하고, MORAI로 검증하기까지의 **공유 절차**.  
> 개인 PC 경로·자격증명은 각자 `DEV-ENVIRONMENT.md`에 두고, **이 문서에는 팀 공통만** 적는다.

---

## 1. 저장소 clone

```bash
# 예시 A — 상위 프로젝트 폴더 안에 두는 경우 (안승현 PC 관행)
mkdir -p ~/projects
cd ~/projects
# 이미 로컬 루트가 있다면 그 아래로
cd "2026-summer-Vertically Integrated Project 2"   # 없으면 생략하고 원하는 경로로
git clone https://github.com/ahnsh03/26-summer-VIP2.git
cd 26-summer-VIP2
```

```bash
# 예시 B — 홈 또는 임의 경로에 단독 clone
git clone https://github.com/ahnsh03/26-summer-VIP2.git
cd 26-summer-VIP2
```

GitHub 권한이 없으면 레포 owner([@ahnsh03](https://github.com/ahnsh03))에게 **collaborator** 추가를 요청한다.

인증(Windows Credential Manager 예):

```bash
git -c credential.helper='!"/mnt/c/Program Files/Git/mingw64/bin/git-credential-manager.exe"' \
  pull
```

---

## 2. 필수 도구

| 도구 | 용도 |
|------|------|
| Git | 브랜치·커밋 |
| GitHub CLI (`gh`) | PR 생성 (권장) — [collaboration.md §1.7](./collaboration.md) |
| Cursor 등 IDE | 코드 편집 (WSL Remote 권장) |
| **MORAI SIM 24.R2.H2** | 시뮬 실행 (Windows) |

ROS/Docker 사용 여부는 스택이 확정되면 이 절에 팀 표준을 추가한다.  
참고: 동아리 모빌리티 대회 쪽은 ROS Noetic + Docker 패턴을 쓴 바 있음.

---

## 3. 시나리오·맵 데이터

대회 시나리오:

- `test_scenario_summer.json`
- `map_init_default.json`

배포 방식이 정해지기 전에는 팀 채널/개인 `data/`로 공유할 수 있다.  
시뮬에 로드할 때는 **규정집과 동일한 파일**을 쓴다.

---

## 4. 일상 워크플로

1. `git checkout main && git pull`
2. `git checkout -b feature/<이름>-<기능>`
3. 코드 수정 · 로컬/시뮬 검증
4. `git push` → `gh pr create` (또는 웹 PR)
5. 리뷰·merge 후 `main` pull, feature 브랜치 삭제

상세: [collaboration.md](./collaboration.md)

---

## 5. 로컬 루트 vs 팀 레포 (역할 구분)

| | 로컬 상위 폴더 | **이 레포 (`26-summer-VIP2`)** |
|--|----------------|--------------------------------|
| Git | 없음 | **있음** (팀 공유) |
| 내용 | PDF, 개인 메모, external 참조 clone, 임시 data | 공유 docs, **대회 코드** |
| 팀원 | 각자 선택 | **전원 clone·PR** |

개인 메모를 이 레포에 올리지 않는다. 팀이 알아야 할 내용만 `docs/`에 PR로 넣는다.

---

## 6. (선택) 셸 변수 예시

안승현 PC (`~/.bashrc`):

```bash
export VIP2_PROJECT="$HOME/projects/2026-summer-Vertically Integrated Project 2"
export VIP2="$VIP2_PROJECT/26-summer-VIP2"
```

팀원은 자기 경로에 맞게 설정하면 된다.
