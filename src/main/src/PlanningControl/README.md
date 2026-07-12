# AIM Autonomous Vehicle Planning & Control System

자율주행 차량의 경로 계획 및 제어를 위한 ROS 기반 통합 시스템입니다. 카메라, LiDAR 센서 입력을 활용하여 격자 기반 경로 계획(Lattice Planning)을 수행하고, 제어 신호를 생성합니다.

---

## 📋 프로젝트 개요

### 주요 기능

- **격자 기반 경로 계획 (Lattice Planning)**: 다중 경로 후보 생성 및 최적 경로 선택
- **다중 센서 융합**: 카메라 및 LiDAR 데이터 처리
- **Costmap 기반 충돌 회피**: 동적 장애물 회피
- **추월 구간 인식**: 추월 금지/가능 구간에 따른 모드 전환
- **속도 제어**: 곡률 기반 적응형 속도 조절
- **RViz 시각화**: 경로, costmap, 차량 상태 실시간 시각화
- **UI 기반 모드 선택**: 표준/매드맥스 모드 전환

---

## 📁 프로젝트 구조

```
aim_ws/
├── src/
│   ├── main/                          # 메인 패키지
│   │   ├── src/
│   │   │   ├── Camera/                # 카메라 센서 처리
│   │   │   ├── Lidar/                 # LiDAR 센서 처리
│   │   │   └── PlanningControl/       # 경로 계획 및 제어
│   │   │       ├── Planning/          # 격자 경로 계획 (LatticePlanning.cpp)
│   │   │       ├── Control/           # 제어 로직
│   │   │       ├── Global/            # 전역 데이터 구조
│   │   │       └── Visualizer/        # 시각화 모듈
│   │   ├── scripts/
│   │   │   └── driving_mode_ui.py     # UI 모드 선택 인터페이스
│   │   └── config/
│   │       ├── lattice_planner.rviz   # RViz 설정
│   │       ├── yaml_hybrid.yaml       # 경로 계획 설정
│   │       ├── Path.txt               # 기준 경로
│   │       └── *.csv                  # 추월 구간, 비용맵 설정
│   └── MORAI-ROS_morai_msgs/          # 메시지 정의
├── devel/                              # 빌드 산출물
├── build/                              # CMake 빌드 디렉토리
├── launch scripts                      # 실행 스크립트들
└── visualization_*.rviz                # RViz 설정 파일들
```

---

## 🚀 시작 가이드

### 사전 요구사항

- **OS**: Ubuntu 20.04 LTS
- **ROS**: ROS Noetic
- **Python**: Python 3.8+
- **필수 패키지**:
  - ros-noetic-tf2-sensor-msgs
  - ros-noetic-rosbridge-server
  - ultralytics (YOLOv8)

### 1️⃣ 초기 설정

```bash
cd ~/aim_ws

# 필수 의존성 설치
./init_setting.sh
```

**설치 내용:**
- ROS 센서 메시지 라이브러리
- ROS Bridge 서버 (웹 통신용)
- YOLOv8 (객체 감지)

### 2️⃣ 빌드

```bash
./build.sh
```

또는 수동 빌드:
```bash
cd ~/aim_ws
source devel/setup.bash
catkin_make
```

### 3️⃣ 시스템 실행

프로젝트는 여러 모듈로 구성되어 있으며, 각 모듈을 개별 터미널에서 실행합니다.

#### **터미널 1: 데이터 브리지 시작**
```bash
./bridge.sh
```
- ROS Bridge WebSocket 서버 실행
- 웹/외부 통신 담당

#### **터미널 2: 카메라 센서**
```bash
./camera.sh
```
- 카메라 입력 수집
- 프레임 퍼블리시

#### **터미널 3: LiDAR 센서**
```bash
./lidar.sh
```
- LiDAR 포인트 클라우드 수집
- 3D 데이터 퍼블리시

#### **터미널 4: 경로 계획 및 제어**
```bash
./lattice_planner.sh
```
- 격자 기반 경로 계획 실행
- Costmap 업데이트
- 제어 신호 생성

#### **터미널 5: RViz 시각화**
```bash
./rviz_visualization.sh
```
- 경로, 센서, 차량 상태 시각화
- 실시간 모니터링

#### **터미널 6: UI 모드 선택 (선택사항)**
```bash
./driving_mode_ui.sh
```
- 표준 / 매드맥스 모드 선택
- 주행 전략 변경

---

## 🎛️ 주행 모드

### 1. **표준 모드 (Standard)**
- 안정적인 경로 추종
- 일반적인 속도 제어
- 보수적인 회피 전략

### 2. **매드맥스 모드 (Mad Max)**
- 공격적인 경로 계획
- 고속 주행
- 적극적인 추월 시도

---

## 📊 주요 설정 파일

### `config/yaml_hybrid.yaml`
- 격자 경로 계획 파라미터
- 다항식 차수, 샘플링 간격 등

### `config/Path.txt`
- 기준 경로점 (위도, 경도)
- 글로벌 네비게이션 기준

### `config/*.csv`
- **No_CameraCostmap_zone.csv**: 카메라 비용맵 제외 구간
- **No_LidarCostmap_zone.csv**: LiDAR 비용맵 제외 구간
- **overtaking_zone.csv**: 추월 가능/불가 구간 정의

### `config/lattice_planner.rviz`
- RViz 시각화 설정
- 표시 레이어, 카메라 각도 등

---

## 🔍 핵심 모듈

### Planning (경로 계획)
**파일**: `src/main/src/PlanningControl/Planning/LatticePlanning.cpp`

**주요 함수:**
- `LatticePlanningProcess()`: 메인 경로 계획 루프
- `generateOffsetGoals()`: 다중 경로 목표 생성
- `computeAllPolynomialPaths()`: 다항식 경로 계산
- `evaluateAllCandidates()`: 경로 비용 평가
- `selectBestPath()`: 최적 경로 선택
- `checkOvertakingZone()`: 추월 구간 인식

**특징:**
- 5차 다항식 경로 생성
- 차량 역학 제약 고려
- 동적 costmap 기반 충돌 회피

### Camera (카메라 센서)
**위치**: `src/main/src/Camera/`

**기능:**
- 이미지 입력 수집
- YOLOv8 기반 객체 감지
- 가능한 영역 맵 생성

### Lidar (LiDAR 센서)
**위치**: `src/main/src/Lidar/`

**기능:**
- 포인트 클라우드 처리
- 장애물 탐지
- 3D 비용맵 생성

### Control (제어)
**위치**: `src/main/src/PlanningControl/Control/`

**기능:**
- Stanley 제어기 / Pure Pursuit 등
- 속도 제어
- 스티어링 명령 생성

---

## 📝 로그 및 데이터

### 경로 기록
- `config/track_log.csv`: 실시간 주행 기록
- `config/track_log_recorded.csv`: 기록된 전체 경로
- `config/track_log_recorded_final.csv`: 최종 경로

### ROS Topics
주요 퍼블리시 토픽:
- `/lattice/path`: 계획된 경로
- `/lattice/costmap`: 충돌 회피용 비용맵
- `/odom`: 차량 위치/자세
- `/cmd_vel`: 속도 명령
- `/imu`: IMU 데이터

---

## 🛠️ 트러블슈팅

### 문제: RViz가 실행되지 않음
```bash
export DISPLAY=:0
export QT_X11_NO_MITSHM=1
```

### 문제: Costmap 수신 실패
- 카메라/LiDAR 노드가 실행 중인지 확인
- ROS 마스터 연결 상태 확인
```bash
rosnode list
rostopic list
```

### 문제: 경로 계획이 느림
- `yaml_hybrid.yaml`에서 샘플링 포인트 감소
- 평가 함수 최적화

---

## 📦 빌드 및 설치

### Docker 환경 (권장)
```bash
cd docker-noetic
docker-compose -f docker-compose.pc.yaml up -d
```

### 직접 설치
1. ROS Noetic 설치: http://wiki.ros.org/noetic/Installation
2. 의존성 설치: `./init_setting.sh`
3. 빌드: `./build.sh`

---

## 📄 라이선스 및 참고

- **플랫폼**: ROS (Robot Operating System)
- **기반 기술**: Lattice Planner (경로 계획)
- **센서**: 카메라 + LiDAR 다중 센서 융합
- **시뮬레이터**: MORAI 시뮬레이터 연동 (메시지 정의)

---

## 👥 개발 및 기여

이 프로젝트는 자율주행 차량의 경로 계획 및 제어 연구를 위해 개발되었습니다.

개선 사항 및 버그 보고는 환영합니다.

---

## 🧪 성능 검증 실험 (Research Paper Evaluation)

본 시스템의 성능을 검증하기 위해 다음과 같은 실험 절차를 제시합니다. 논문의 **<표1> Camera-Lidar Fusion Costmap 검증** 및 **<표2> VA-MDS Lattice Planner 성능 비교**를 재현할 수 있습니다.

### 🔬 실험 개요

**4가지 비교군 설정:**
1. **Baseline 0 (Static)**: 고정 LD (10m) + 고정 해상도 (52개 샘플)
2. **Baseline 1 (Dynamic LD Only)**: 속도 비례 LD + 고정 해상도
3. **Baseline 2 (Max High-Res)**: 고정 LD (15m) + 최대 해상도 (128개 샘플)
4. **Proposed (VA-MDS)**: 속도 적응 LD + 동적 해상도 - **제안 방법**

**평가 지표:**
- RMS Offset (m): 횡방향 오차
- Max Offset (m): 최대 편차
- Steering Variance (deg²): 조향각 분산
- Intrusion Ratio (%): 차도 침범률
- Computation Time (ms): 실시간성

### 📋 Phase 1: Camera-Lidar Fusion Costmap 검증

**목표**: Camera 기반 주행 가능 영역 코스트맵의 효과 입증

#### M1: 직선 구간 - Static (기준)
```bash
cd /root/aim_ws
export EXPERIMENT_MODE=0
export SCENARIO_NAME="straight_line"
./run_experiment.sh
```
**예상 결과**: RMS Offset ~4.1m, Intrusion Ratio ~91.6%

#### M2: 직선 구간 - VA-MDS (Camera 포함)
```bash
cd /root/aim_ws
export EXPERIMENT_MODE=3
export SCENARIO_NAME="straight_line"
./run_experiment.sh
```
**예상 결과**: RMS Offset ~1.7m, Intrusion Ratio 0%

#### M3: 곡선 구간 - Static (기준)
```bash
cd /root/aim_ws
export EXPERIMENT_MODE=0
export SCENARIO_NAME="curve_line"
./run_experiment.sh
```
**예상 결과**: RMS Offset ~3.4m, Intrusion Ratio ~85.0%

#### M4: 곡선 구간 - VA-MDS (Camera 포함)
```bash
cd /root/aim_ws
export EXPERIMENT_MODE=3
export SCENARIO_NAME="curve_line"
./run_experiment.sh
```
**예상 결과**: RMS Offset ~1.6m, Intrusion Ratio 0%

---

### 📋 Phase 2: VA-MDS Lattice Planner 시나리오 검증

**목표**: 속도 적응형 LD와 동적 해상도의 성능 입증

#### Scenario 1: 저속 협소 구간 (속도 5~15 km/h, 폭 4m 골목길)

**Baseline 0: Static**
```bash
export EXPERIMENT_MODE=0
export SCENARIO_NAME="scenario1_narrow_slow"
./run_experiment.sh
```
**예상 결과**: ❌ **충돌** (해상도 부족)

**Baseline 1: Dynamic LD Only**
```bash
export EXPERIMENT_MODE=1
export SCENARIO_NAME="scenario1_narrow_slow"
./run_experiment.sh
```
**예상 결과**: ❌ **충돌** (LD 부족)

**Baseline 2: Max High-Res**
```bash
export EXPERIMENT_MODE=2
export SCENARIO_NAME="scenario1_narrow_slow"
./run_experiment.sh
```
**예상 결과**: ✅ **통과** (높은 계산량)

**Proposed: VA-MDS** ⭐
```bash
export EXPERIMENT_MODE=3
export SCENARIO_NAME="scenario1_narrow_slow"
./run_experiment.sh
```
**예상 결과**: ✅ **통과** (저속에서 해상도 128개로 자동 증가, 효율적)

---

#### Scenario 2: 고속 회피 구간 (속도 50~60 km/h, 2차로, 전방 장애물)

**Baseline 0: Static**
```bash
export EXPERIMENT_MODE=0
export SCENARIO_NAME="scenario2_fast_avoidance"
./run_experiment.sh
```
**예상 결과**: ✅ 통과 (조향 분산 18.42 deg² - **불안정**)

**Baseline 1: Dynamic LD Only**
```bash
export EXPERIMENT_MODE=1
export SCENARIO_NAME="scenario2_fast_avoidance"
./run_experiment.sh
```
**예상 결과**: ✅ 통과 (조향 분산 1.85 deg², RMS Offset 큼)

**Baseline 2: Max High-Res**
```bash
export EXPERIMENT_MODE=2
export SCENARIO_NAME="scenario2_fast_avoidance"
./run_experiment.sh
```
**예상 결과**: ❌ **제어 불안정** (조향 분산 35.15 deg² - 최악)

**Proposed: VA-MDS** ⭐
```bash
export EXPERIMENT_MODE=3
export SCENARIO_NAME="scenario2_fast_avoidance"
./run_experiment.sh
```
**예상 결과**: ✅ **통과** (조향 분산 1.95 deg² - **최적**, LD 확장으로 예측성 확보)

---

#### Scenario 3: 속도 가변 연결 구간 (속도 10→60→20 km/h, 직선→곡선→협소)

**Baseline 0: Static**
```bash
export EXPERIMENT_MODE=0
export SCENARIO_NAME="scenario3_variable_speed"
./run_experiment.sh
```
**예상 결과**: ❌ **실패** (전환 구간에서 제어 불안정)

**Baseline 1: Dynamic LD Only**
```bash
export EXPERIMENT_MODE=1
export SCENARIO_NAME="scenario3_variable_speed"
./run_experiment.sh
```
**예상 결과**: ❌ **실패** (저속 구간 협소 통과 불가)

**Baseline 2: Max High-Res**
```bash
export EXPERIMENT_MODE=2
export SCENARIO_NAME="scenario3_variable_speed"
./run_experiment.sh
```
**예상 결과**: ❌ **실패** (고속 구간 진입 후 제어 불안정)

**Proposed: VA-MDS** ⭐
```bash
export EXPERIMENT_MODE=3
export SCENARIO_NAME="scenario3_variable_speed"
./run_experiment.sh
```
**예상 결과**: ✅ **통과** (전 구간 안정적 궤적 추종)

---

### 🚀 자동화 스크립트 (모든 실험 일괄 실행)

```bash
#!/bin/bash
# filepath: /root/aim_ws/run_all_experiments.sh

cd /root/aim_ws

echo "=========================================="
echo "  Phase 1: Camera Costmap Validation"
echo "=========================================="

# Phase 1: Camera Costmap 검증
for scenario in "straight_line" "curve_line"; do
    for mode in 0 3; do
        export EXPERIMENT_MODE=$mode
        export SCENARIO_NAME=$scenario
        mode_names=("Static" "DynamicLD" "MaxHighRes" "VA-MDS")
        echo "[$(date '+%H:%M:%S')] Running Mode $mode (${mode_names[$mode]}), Scenario: $scenario"
        ./run_experiment.sh
        sleep 5
    done
done

echo ""
echo "=========================================="
echo "  Phase 2: VA-MDS Scenario Validation"
echo "=========================================="

# Phase 2: Scenario 검증
scenarios=("scenario1_narrow_slow" "scenario2_fast_avoidance" "scenario3_variable_speed")
scenario_desc=("Narrow-Low Speed" "Fast-Avoidance" "Variable Speed")

for i in "${!scenarios[@]}"; do
    scenario="${scenarios[$i]}"
    desc="${scenario_desc[$i]}"
    
    echo ""
    echo "--- $desc: $scenario ---"
    
    for mode in 0 1 2 3; do
        export EXPERIMENT_MODE=$mode
        export SCENARIO_NAME=$scenario
        mode_names=("Static" "DynamicLD" "MaxHighRes" "VA-MDS")
        echo "[$(date '+%H:%M:%S')] Running Mode $mode (${mode_names[$mode]})"
        ./run_experiment.sh
        sleep 5
    done
done

echo ""
echo "=========================================="
echo "  ✅ All experiments completed!"
echo "=========================================="
echo "Results saved in:"
echo "  - Experiment_Mode0_Static_*.csv"
echo "  - Experiment_Mode1_DynamicLD_*.csv"
echo "  - Experiment_Mode2_MaxHighRes_*.csv"
echo "  - Experiment_Mode3_VA-MDS_*.csv"
```

**실행:**
```bash
chmod +x run_all_experiments.sh
./run_all_experiments.sh
```

---

### 📁 실험 데이터 저장 위치

자동으로 생성되는 폴더 구조:
```
/root/aim_ws/src/main/
├── Experiment_Mode0_Static_straight_line/
│   └── run_01_staticLattice.csv
├── Experiment_Mode3_VA-MDS_straight_line/
│   └── run_01_vaMds.csv
├── Experiment_Mode0_Static_curve_line/
│   └── run_01_staticLattice.csv
├── Experiment_Mode3_VA-MDS_curve_line/
│   └── run_01_vaMds.csv
├── Experiment_Mode0_Static_scenario1_narrow_slow/
│   └── run_01_staticLattice.csv
├── Experiment_Mode1_DynamicLD_scenario1_narrow_slow/
│   └── run_01_dynamicLattice.csv
├── Experiment_Mode2_MaxHighRes_scenario1_narrow_slow/
│   └── run_01_maxHighRes.csv
├── Experiment_Mode3_VA-MDS_scenario1_narrow_slow/
│   └── run_01_vaMds.csv
...
```

각 CSV 파일에는 다음 정보가 기록됩니다:
- 차량 위치 (x, y)
- 속도 (velocity)
- 조향각 (steering angle)
- RMS Offset, Max Offset
- 충돌 여부 (collision flag)
- 계산 시간 (computation time)

---

### 📊 결과 분석

**Excel/Python으로 데이터 수집:**
```python
import pandas as pd
import glob

# 모든 결과 파일 수집
results = {}
for mode in range(4):
    for scenario in ["scenario1_narrow_slow", "scenario2_fast_avoidance", "scenario3_variable_speed"]:
        folder = f"Experiment_Mode{mode}_*/Experiment_Mode{mode}*{scenario}"
        csv_files = glob.glob(f"/root/aim_ws/src/main/{folder}/*.csv")
        if csv_files:
            df = pd.read_csv(csv_files[0])
            results[(mode, scenario)] = df

# 통계 출력
for (mode, scenario), df in results.items():
    print(f"Mode {mode} - {scenario}:")
    print(f"  RMS Offset: {df['rms_offset'].mean():.3f}m")
    print(f"  Steering Variance: {df['steering_angle'].var():.3f} deg²")
    print(f"  Success: {(df['collision'] == 0).sum() / len(df) * 100:.1f}%")
```

---

## 📞 주요 실행 명령 요약

| 작업 | 명령어 |
|-----|--------|
| **초기 설정** | `./init_setting.sh` |
| **빌드** | `./build.sh` |
| **데이터 브리지** | `./bridge.sh` |
| **카메라 실행** | `./camera.sh` |
| **LiDAR 실행** | `./lidar.sh` |
| **경로 계획** | `./lattice_planner.sh` |
| **시각화** | `./rviz_visualization.sh` |
| **UI 모드 선택** | `./driving_mode_ui.sh` |
| **모든 실험 실행** | `./run_all_experiments.sh` |

---

**마지막 업데이트**: 2026년 3월
