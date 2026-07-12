#include "Planning.hpp"
#include "Global.hpp"
#include <ros/ros.h>
#include <algorithm>
#include <cmath>

using namespace std;

// ========================================
// Baseline 0: Static Lattice Planner
// (고정 LD + 고정 해상도)
// ========================================
void DynamicLDOnlyProcess() {
    lock_guard<std::mutex> lock(costmap_mutex);
    
    int current_mode = driving_mode;
    
    checkOvertakingZone(ego);
    if (!checkCostmapAvailable()) {
        ROS_WARN_THROTTLE(1.0, "[DynamicLD] Waiting for costmap...");
        return;
    }
    
    findClosestWaypoint(ego, lattice_ctrl.close_idx);
    
    // ✨ Baseline 특성: 속도 비례 LD
    double speed_scale_ld = 1.0;
    if (ego.vel > 3.0) {
        speed_scale_ld = 1.0 + (ego.vel - 3.0) * 0.15;  // v 증가시 LD 증가
    }
    double dynamic_ld = 10.0 * speed_scale_ld;
    
    findLookaheadGoal(ego, lattice_ctrl.close_idx, lattice_ctrl);
    
    // ✨ Baseline 특성: 고정 해상도 (52개) - 적응성 없음
    generateConservativeOffsetGoals(lattice_ctrl);
    
    transformOffsetGoalsToBaselink(lattice_ctrl, ego);
    computeAllPolynomialPaths(lattice_ctrl);
    sampleAllCandidatePaths(lattice_ctrl);
    evaluateAllCandidates(lattice_ctrl, current_mode);
    selectBestPath(lattice_ctrl);
    getTargetLocalPathIdx(lattice_ctrl, ctrl.ld, ctrl.lookahead_idx);
    getMaxCurvature(ctrl.close_idx, ctrl.lookahead_idx * 3, ego.max_curvature);
    getTargetSpeed(ego.max_curvature, ctrl.target_vel, ctrl.lookahead_idx, 
                   current_mode, ego);
    
    ROS_DEBUG_THROTTLE(1.0, "[DynamicLD] Speed: %.1f m/s | Dynamic LD: %.2f m | Fixed Resolution: 52", 
                       ego.vel, dynamic_ld);
}

// ========================================
// Baseline 2: Max High-Res
// (고정 LD + 항상 최대 해상도)
// ========================================
void MaxHighResProcess() {
    lock_guard<std::mutex> lock(costmap_mutex);
    
    int current_mode = driving_mode;
    
    checkOvertakingZone(ego);
    if (!checkCostmapAvailable()) {
        ROS_WARN_THROTTLE(1.0, "[MaxHighRes] Waiting for costmap...");
        return;
    }
    
    findClosestWaypoint(ego, lattice_ctrl.close_idx);
    
    // ✨ Baseline 특성: 고정 LD (15m)
    findStaticLookaheadGoal(ego, lattice_ctrl.close_idx, lattice_ctrl);
    
    // ✨ Baseline 특성: 항상 최대 해상도 (128개) - 속도 무관
    generateOffsetGoalsMaxResolution(lattice_ctrl);
    
    transformOffsetGoalsToBaselink(lattice_ctrl, ego);
    computeAllPolynomialPaths(lattice_ctrl);
    sampleAllCandidatePaths(lattice_ctrl);
    evaluateAllCandidates(lattice_ctrl, current_mode);
    selectBestPath(lattice_ctrl);
    getTargetLocalPathIdx(lattice_ctrl, ctrl.ld, ctrl.lookahead_idx);
    getMaxCurvature(ctrl.close_idx, ctrl.lookahead_idx * 3, ego.max_curvature);
    getTargetSpeed(ego.max_curvature, ctrl.target_vel, ctrl.lookahead_idx, 
                   current_mode, ego);
    
    ROS_DEBUG_THROTTLE(1.0, "[MaxHighRes] Fixed LD: 15.0m | Max Resolution: 128 samples");
}

// ========================================
// Helper Functions
// ========================================

/**
 * @brief 최대 해상도 오프셋 생성 (항상 128개 샘플)
 */
void generateOffsetGoalsMaxResolution(LatticeControl& lattice_ctrl) {
    lattice_ctrl.offset_goals.clear();
    
    const int MAX_NUM_SAMPLES = 32;  // 최대 해상도
    const double MAX_OFFSET = 8.0;
    
    auto generate_for_target = [&](int target_idx, const char* name) {
        if (target_idx < 0 || target_idx >= (int)waypoints.size()) return;
        
        double goal_ref_x = waypoints[target_idx].x;
        double goal_ref_y = waypoints[target_idx].y;
        double dx = 0.0, dy = 0.0;
        
        if (target_idx < (int)waypoints.size() - 1) {
            dx = waypoints[target_idx + 1].x - waypoints[target_idx].x;
            dy = waypoints[target_idx + 1].y - waypoints[target_idx].y;
        }
        
        double len = std::max(1e-6, std::sqrt(dx*dx + dy*dy));
        double dir_x = dx / len, dir_y = dy / len;
        double norm_x = -dir_y, norm_y = dir_x;
        double yaw_global = std::atan2(dir_y, dir_x);
        
        int num_samples = MAX_NUM_SAMPLES;
        if (num_samples % 2 == 0) num_samples += 1;  // 홀수 보장
        
        for (int j = 0; j < num_samples; j++) {
            double offset = -MAX_OFFSET + (2.0 * MAX_OFFSET) * j / (num_samples - 1);
            if (std::abs(offset) < 0.1) offset = 0.0;
            
            OffsetGoal goal;
            goal.global_x = goal_ref_x + offset * norm_x;
            goal.global_y = goal_ref_y + offset * norm_y;
            goal.global_yaw = yaw_global;
            goal.offset = offset;
            
            lattice_ctrl.offset_goals.push_back(goal);
        }
    };
    
    generate_for_target(lattice_ctrl.target_idx_short, "Short");
    generate_for_target(lattice_ctrl.target_idx_medium, "Medium");
    generate_for_target(lattice_ctrl.target_idx_long, "Long");
    generate_for_target(lattice_ctrl.target_idx_very_long, "VeryLong");
}
