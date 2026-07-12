# 협업 가이드 (브랜치 · PR · 충돌 방지)

> **목표**: PC에서 개발 → GitHub PR → MORAI 시뮬 검증 → 대회 스택 안정화  
> **필독**: 코드 수정 전 **§1 Git 규약**을 따른다.  
> 환경·clone: [setup.md](./setup.md) · 대회 제약: [competition.md](./competition.md)

---

## 1. 팀 Git 규약 (필수)

### 1.1 기본 원칙

| 규칙 | 설명 |
|------|------|
| **`main` 직접 push 금지** | 안정 브랜치는 **Pull Request merge로만** 반영 |
| **브랜치에서만 개발** | `main`에서 바로 코딩하지 않음 |
| **작은 PR** | 한 PR = 한 기능 또는 한 버그 수정 |
| **merge 후 정리** | 로컬 feature 브랜치 삭제, `main` pull |
| **이 repo에서만 대회 코드 누적** | 개인 실험 repo에 최종본을 흩어 두지 않음 — 통합은 여기로 |

```
main (안정 — 시뮬·대회 기준)
  └── feature/<이름>-<기능>
         ├── 개발 · 커밋
         ├── push
         ├── Pull Request
         ├── (팀) 리뷰
         └── merge → main
```

> **요약**: `브랜치 생성 → 작업 → commit → push → PR → (리뷰) → merge`  
> **`main`에 직접 push하지 않는다.**

### 1.2 표준 개발 절차

```bash
# 0. 저장소 이동 (경로는 팀원 PC마다 다를 수 있음 — setup.md 참고)
cd ~/projects/2026-summer-Vertically\ Integrated\ Project\ 2/26-summer-VIP2
# 또는: cd "$VIP2"

# 1. main 최신화
git checkout main
git pull origin main

# 2. 작업 브랜치 생성
git checkout -b feature/seunghyun-gps-fallback

# 3. 담당 범위만 수정 후 커밋
git add src/...
git commit -m "feat(localization): GPS jamming fallback skeleton"

# 4. push
git push -u origin feature/seunghyun-gps-fallback

# 5. Pull Request (GitHub CLI 권장)
gh pr create \
  --title "feat(localization): GPS jamming fallback skeleton" \
  --body "$(cat <<'EOF'
## Summary
- GPS=0 구간용 fallback localization 골격

## 테스트
- [ ] MORAI 24.R2.H2에서 시나리오 로드
- [ ] GPS 강제 0 시 주행 유지 여부
EOF
)"

# 6. merge 후 로컬 정리
git checkout main
git pull origin main
git branch -d feature/seunghyun-gps-fallback
```

PR은 GitHub 웹에서 만들어도 된다. merge는 **팀장/합의된 담당자**가 수행.

### 1.3 브랜치 이름 규칙

| 패턴 | 예 | 사용 시점 |
|------|-----|-----------|
| `feature/<이름>-<기능>` | `feature/seunghyun-lidar-cluster` | **기능 개발 (기본)** |
| `fix/<이름>-<이슈>` | `fix/wontae-stop-overshoot` | 버그 수정 |
| `docs/<이름>-<주제>` | `docs/seunghyun-setup` | 문서만 |

**비권장**

- `main`에서 직접 commit & push
- 개인 이름 브랜치 하나(`seunghyun` 등)에 모든 작업 누적
- 한 PR에 서로 다른 담당 모듈을 대량으로 섞기

### 1.4 커밋 메시지

```
feat(scope): 한 줄 요약
fix(scope): 버그 수정
docs: 문서만 변경
chore: 빌드·설정 등
```

`scope` 예: `perception`, `localization`, `planning`, `control`, `bridge`, `docker`, `mission`

예: `feat(control): ctrl_cmd throttle limit for stop mission`

### 1.5 PR 타이밍

| 상황 | 행동 |
|------|------|
| 동작하는 최소 단위 완성 | PR 생성 |
| 방향 피드백 필요·미완 | **Draft PR** |
| 리뷰·merge 가능 | Ready for review |

### 1.6 도구 역할

| 도구 | 역할 |
|------|------|
| **Cursor / 터미널** | 브랜치, commit, push, `gh pr create` |
| **GitHub CLI (`gh`)** | PR 생성·상태·CI 확인 |
| **GitHub 웹** | 리뷰·merge |
| **Sublime Merge 등** | 히스토리·diff·merge conflict |
| **MORAI SIM (Windows)** | 시뮬 검증 |

### 1.7 GitHub CLI (`gh`) — 권장

```bash
gh --version
gh auth login          # 최초 1회
gh auth status

git push -u origin HEAD
gh pr create --title "..." --body "..."
gh pr status
```

레포 collaborator 권한이 없으면 팀장([@ahnsh03](https://github.com/ahnsh03))에게 초대 요청.

---

## 2. 코드 수정 규칙 (충돌 방지)

초기에는 모듈 담당이 유동적일 수 있다. 합의 전까지:

| 규칙 | 설명 |
|------|------|
| **담당 파일만 수정** | 남의 WIP 파일은 건드리지 않음 — 필요하면 이슈/채팅으로 합의 |
| **공통 인터페이스는 합의 후** | bridge, launch, 메시지 정의, `ctrl_cmd` 발행 지점 |
| **규정 위반 API 금지** | `ctrl_cmd` / `MoraiEventCmdSrv` 외 제어·이벤트 사용하지 않음 |
| **시크릿 금지** | `.env`, 라이선스 키, 계정 정보를 commit하지 않음 |

모듈별 디렉터리·담당자 표는 코드 골격이 잡히면 이 문서 §2에 추가한다.

---

## 3. 개발·검증 환경 (요약)

| 환경 | 역할 |
|------|------|
| **WSL (Ubuntu)** | 코드 편집, Git, (선택) ROS Docker |
| **Windows + MORAI 24.R2.H2** | 시뮬 실행·시나리오 검증 |
| **Docker ROS Noetic** | 팀 합의 시 bridge/ROS 노드 빌드 (모빌리티 대회 스택과 유사할 수 있음) |

상세 경로·자격 증명은 개인 `DEV-ENVIRONMENT.md`를 따르고, **팀에 공유할 절차만** [setup.md](./setup.md)에 적는다.

시나리오 파일(`test_scenario_summer.json`, `map_init_default.json`)은 용량·배포 방식에 따라 repo 또는 별도 공유일 수 있다. 현재 개인 로컬에는 상위 폴더 `data/`에 있다.

---

## 4. 충돌이 날 때

```bash
git checkout feature/my-branch
git fetch origin
git rebase origin/main
# conflict 해결 → git add → git rebase --continue
git push --force-with-lease
```

같은 파일을 두 명이 수정 중이면 **파일/모듈 분리**를 먼저 합의한다.

---

## 5. PR 체크리스트 (요약)

- [ ] `main`에서 feature 브랜치를 분기했는가
- [ ] **`main`에 직접 push하지 않았는가**
- [ ] 커밋 메시지·PR 제목이 변경 의도를 나타내는가
- [ ] 제어 출력이 `ctrl_cmd` / `MoraiEventCmdSrv`만 쓰는가
- [ ] (가능하면) MORAI에서 해당 미션 구간을 돌려 보았는가
- [ ] merge 후 로컬 feature 브랜치를 삭제했는가

---

## 관련 문서

- [competition.md](./competition.md) — 대회 규정
- [setup.md](./setup.md) — 셋업
- [README.md](../README.md) — 저장소 개요
