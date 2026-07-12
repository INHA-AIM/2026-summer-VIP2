#include "Global.hpp"
#include "Planning.hpp"

#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
using namespace std;

// ========================================
// Static Lattice Planning Process
// [빠이스라인] 정적 LD로 인한 자연스러운 성능 제약:
// 1. 고속에서 부족한 예측 거리
// 2. 저속에서 비효율적인 과도한 예측
// 3. 단순한 오프셋 생성 전략
// ========================================
void StaticLatticePlannerProcess() {

    lock_guard<std::mutex> lock(costmap_mutex);
    
    // 시작 시점의 모드를 저장 (변경 방지)
    int current_mode = driving_mode;

    checkOvertakingZone(ego);
    if (!checkCostmapAvailable()) {
        ROS_WARN_THROTTLE(1.0, "[StaticLattice] Waiting for costmap...");
        return;
    }
    findClosestWaypoint(ego, lattice_ctrl.close_idx);
    findStaticLookaheadGoal(ego, lattice_ctrl.close_idx, lattice_ctrl);  // Static version
    generateConservativeOffsetGoals(lattice_ctrl);  // 보수적 오프셋 생성
    transformOffsetGoalsToBaselink(lattice_ctrl, ego);
    computeAllPolynomialPaths(lattice_ctrl);
    sampleAllCandidatePaths(lattice_ctrl);
    evaluateAllCandidates(lattice_ctrl, current_mode);
    selectBestPath(lattice_ctrl);
    getTargetLocalPathIdx(lattice_ctrl, ctrl.ld, ctrl.lookahead_idx);
    getMaxCurvature(ctrl.close_idx, ctrl.lookahead_idx * 3, ego.max_curvature);
    getTargetSpeed(ego.max_curvature, ctrl.target_vel, ctrl.lookahead_idx, current_mode, ego);
}

// ========================================
// Static Lookahead Goal 찾기 (속도 무관)
// ========================================
void findStaticLookaheadGoal(const VehicleState& ego, int close_idx, LatticeControl& lattice_ctrl) {
    // 정적 LD - 속도에 무관하게 고정된 거리 사용
    // 자연스러운 성능 제약: 모든 속도에서 동일한 LD 사용
    double ld_short = lattice_ctrl.ld_short;        // 5m (고정)
    double ld_medium = lattice_ctrl.ld_medium;      // 10m (고정)
    double ld_long = lattice_ctrl.ld_long;          // 15m (고정)
    double ld_very_long = lattice_ctrl.ld_very_long; // 20m (고정)
    
    // 고속에서는 예측 거리 부족, 저속에서는 과도한 예측으로 비효율

    int target_idx_short = close_idx;
    int target_idx_medium = close_idx;
    int target_idx_long = close_idx;
    int target_idx_very_long = close_idx;

    for (int i = close_idx; i < (int)waypoints.size(); i++) {
        double dx = waypoints[i].x - ego.x;
        double dy = waypoints[i].y - ego.y;
        double dist = std::sqrt(dx*dx + dy*dy);

        // ld_short보다 큰 첫 번째 idx
        if (dist >= ld_short && target_idx_short == close_idx) {
            target_idx_short = i;
        }

        // ld_medium보다 큰 첫 번째 idx
        if (dist >= ld_medium && target_idx_medium == close_idx) {
            target_idx_medium = i;
        }

        // ld_long보다 큰 첫 번째 idx
        if (dist >= ld_long && target_idx_long == close_idx) {
            target_idx_long = i;
        }

        // ld_very_long보다 큰 첫 번째 idx (이것을 찾으면 종료)
        if (dist >= ld_very_long && target_idx_very_long == close_idx) {
            target_idx_very_long = i;
            break;
        }
    }

    lattice_ctrl.target_idx_short = target_idx_short;
    lattice_ctrl.target_idx_medium = target_idx_medium;
    lattice_ctrl.target_idx_long = target_idx_long;
    lattice_ctrl.target_idx_very_long = target_idx_very_long;

    // [디버깅] Static LD 사용 확인
    ROS_DEBUG_THROTTLE(1.0, "[StaticLattice] Fixed LD used (vel=%.2f): Short=%.1f, Medium=%.1f, Long=%.1f, VeryLong=%.1f", 
             ego.vel, ld_short, ld_medium, ld_long, ld_very_long);
}

// ========================================
// 보수적 오프셋 생성 (속도 비적응성으로 인한 자연스러운 성능 제약)
// ========================================
void generateConservativeOffsetGoals(LatticeControl& lattice_ctrl) {
    lattice_ctrl.offset_goals.clear();
    
    // 빠이스라인: 속도와 무관하게 고정된 파라미터 사용
    // VA와 달리 속도별 최적화 없음
    int n = planner_params.num_offsets;  // 고정된 오프셋 개수
    
    // 단순한 오프셋 생성 로직 (단일 거리만 사용)
    auto add_simple_set = [&](int idx, const char* name) {
        if (idx >= (int)waypoints.size()) {
            idx = (int)waypoints.size() - 1;
        }
        
        double goal_ref_x = waypoints[idx].x;
        double goal_ref_y = waypoints[idx].y;
        double dx = 0.0, dy = 0.0;
        if (idx < (int)waypoints.size() - 1) {
            dx = waypoints[idx + 1].x - waypoints[idx].x;
            dy = waypoints[idx + 1].y - waypoints[idx].y;
        }
        double len = std::max(1e-6, std::sqrt(dx*dx + dy*dy));
        double dir_x = dx / len, dir_y = dy / len;
        double norm_x = -dir_y, norm_y = dir_x;
        double yaw_global = std::atan2(dir_y, dir_x);

        for (int i = 0; i < n; i++) {
            double offset = -planner_params.lateral_offset_step * (n - 1) / 2.0 + 
                             planner_params.lateral_offset_step * i;
            OffsetGoal goal;
            goal.global_x = goal_ref_x + offset * norm_x;
            goal.global_y = goal_ref_y + offset * norm_y;
            goal.global_yaw = yaw_global;
            goal.offset = offset;
            lattice_ctrl.offset_goals.push_back(goal);
        }
    };

    // 베이스라인: 기본적인 다중 거리 사용 (VA의 복잡한 Multi-Distance Sampling 없이)
    // 원래 Lattice와 동일한 4개 거리 사용하지만 속도별 최적화는 없음
    add_simple_set(lattice_ctrl.target_idx_very_long, "very_long");
    add_simple_set(lattice_ctrl.target_idx_medium, "medium");    
    add_simple_set(lattice_ctrl.target_idx_long, "long");      
    add_simple_set(lattice_ctrl.target_idx_short, "short");     
    
    ROS_DEBUG_THROTTLE(1.0, "[StaticLattice] Conservative offset generation: %zu goals (no velocity adaptation)", 
                      lattice_ctrl.offset_goals.size());
}
