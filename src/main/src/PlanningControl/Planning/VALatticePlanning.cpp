#include "Global.hpp"
#include "Planning.hpp"

#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
using namespace std;

// ============================================================================
// VA-MDS Parameters (논문 수식 기반)
// ============================================================================
struct VAMDSParams {
    // 물리 파라미터
    double mu = 0.7;              // 노면 마찰계수 (아스팔트)
    double g = 9.81;              // 중력가속도
    double a_lat_max = 1.5;       // 최대 허용 횡가속도 (m/s²)
    
    // 속도 임계값 계수 (식 2, 3 수정)
    double alpha = 0.25;          // v_low = alpha * v_max
    double beta = 0.70;           // v_high = beta * v_max
    double S_min = 0.4;           // 최소 스케일 팩터
    
    // LD 파라미터 (식 4)
    double L_min = 10.0;           // 최소 주시 거리 (m)
    double k_v = 0.6;             // 속도 가중치
    
    // 샘플링 파라미터
    vector<int> base_samples = {38, 33, 30, 25,20};  // 거리별 기본 샘플 수
    double D_max = 8.0;           // 최대 횡방향 오프셋 (m)
} va_params;

// ============================================================================
// Forward declarations
// ============================================================================
double estimatePathCurvature(int close_idx);
void findLookaheadGoalVA(const VehicleState& ego, int close_idx, LatticeControl& lattice_ctrl);
void generateMultiDistanceAdaptiveGoals(LatticeControl& lattice_ctrl);
void evaluateAllCandidates_VA(LatticeControl& lattice_ctrl, int mode);
void getTargetSpeedVA(double max_curvature, double& out_target_vel, const VehicleState& ego);
void calculatePathValidityRatiosVA(LatticeControl& lattice_ctrl);

// ============================================================================
// MAIN: VALatticePlanningProcess
// ============================================================================
void VALatticePlanningProcess() {
    lock_guard<std::mutex> lock(costmap_mutex);
    
    int current_mode = driving_mode;

    checkOvertakingZone(ego);
    if (!checkCostmapAvailable()) {
        ROS_WARN_THROTTLE(1.0, "[VA-Lattice] Waiting for costmap...");
        return;
    }
    findClosestWaypoint(ego, lattice_ctrl.close_idx);
    findLookaheadGoalVA(ego, lattice_ctrl.close_idx, lattice_ctrl);
    generateMultiDistanceAdaptiveGoals(lattice_ctrl);
    transformOffsetGoalsToBaselink(lattice_ctrl, ego);
    computeAllPolynomialPaths(lattice_ctrl);
    sampleAllCandidatePaths(lattice_ctrl);
    evaluateAllCandidates_VA(lattice_ctrl, current_mode);
    selectBestPath(lattice_ctrl);
    calculatePathValidityRatiosVA(lattice_ctrl);  // VA 특화: 비율 계산
    getTargetLocalPathIdx(lattice_ctrl, ctrl.ld, ctrl.lookahead_idx);
    getMaxCurvature(ctrl.close_idx, ctrl.lookahead_idx * 3, ego.max_curvature);
    getTargetSpeedVA(ego.max_curvature, ctrl.target_vel, ego);
}
// ========================================
// 경로 평가 (costmap query는 map 좌표로 변환 후)
// ========================================
void evaluateAllCandidates_VA(LatticeControl& lattice_ctrl, int mode) {
    double front_offset = planner_params.vehicle_front_offset; // 4.0m
    double width = 2.0; // 차폭
    double half_width = width / 2.0;

    // [추가 1] 현재 게이트 존(No Lidar Zone)인지 확인
    // 이 함수는 Global.hpp에 extern 선언되어 있어야 합니다.
    bool ignore_lidar = isInsideNoLidarZone(); 

    for (auto& path : lattice_ctrl.candidates) {
        path.obstacle_cost = 0.0;
        path.lane_cost = 0.0; 
        path.valid = true;
        
        int lethal_count = 0;
        int valid_point_count = 0;
        double lane_penalty_sum = 0.0; 

        for (size_t i = 0; i < path.points.size(); ++i) {
            Point2D pt_rear = path.points[i]; // base_link 좌표

            // 헤딩 계산
            double heading = 0.0;
            if (i + 1 < path.points.size()) {
                heading = atan2(path.points[i+1].y - pt_rear.y, 
                               path.points[i+1].x - pt_rear.x);
            } else if (i > 0) {
                heading = atan2(pt_rear.y - path.points[i-1].y, 
                               pt_rear.x - path.points[i-1].x);
            }
            double cos_heading = cos(heading);
            double sin_heading = sin(heading);

            // ====================================================
            // 1. LiDAR 기반 물리적 장애물 검사 (Hard Constraint)
            // ====================================================
            int max_cost_in_step = 0;
            bool inside_map = false;

            // [추가 2] 게이트 존이 아닐 때만 라이다 검사 수행
            if (!ignore_lidar) { 
                // d는 차량 길이 방향 거리 (0m ~ 4.0m)
                for (double front_dx = 0.0; front_dx <= front_offset; front_dx += 0.5) {
                    double cx = pt_rear.x + front_dx * cos_heading;
                    double cy = pt_rear.y + front_dx * sin_heading;

                    // 좌/우/중앙 3점 검사
                    std::vector<Point2D> body_points;
                    body_points.push_back({cx, cy}); // 중앙
                    body_points.push_back({cx - half_width * sin_heading, cy + half_width * cos_heading}); // 왼쪽
                    body_points.push_back({cx + half_width * sin_heading, cy - half_width * cos_heading}); // 오른쪽

                    for (const auto& body_point : body_points) {
                        // LiDAR Costmap 직접 조회
                        if (checkCostmapAvailable()) { // 안전 장치
                            int cost = getCostmapCost(body_point.x, body_point.y);
                            if (cost > max_cost_in_step) {
                                max_cost_in_step = cost;
                            }
                            // 하나라도 맵 안에 있으면 유효한 포인트로 간주
                            if (cost >= 0) inside_map = true;
                        }
                    }
                }
            } else {
                // 게이트 존 내부라면 장애물 코스트는 0, 맵은 항상 유효한 것으로 처리
                max_cost_in_step = 0;
                inside_map = true; 
            }

            // ====================================================
            // 2. Camera 기반 차선 이탈 검사 (Soft Constraint)
            // ====================================================
            // 게이트 존이라도 차선(혹은 유도선)은 지키고 싶다면 이 부분은 유지
            // 만약 게이트 존에서 차선도 무시해야 한다면 if (!ignore_lidar) 안에 넣으세요.
            int cam_cost = getCameraCost(pt_rear.x, pt_rear.y); 
            lane_penalty_sum += cam_cost; 

            if (!inside_map) continue;

            valid_point_count++;

            // [LiDAR] 치명적 장애물(100)이면 즉시 카운트
            // ignore_lidar가 true면 max_cost_in_step이 0이므로 여기 안 걸림
            if (max_cost_in_step >= (int)planner_params.lethal_cost_threshold) {
                lethal_count++;
            }
            path.obstacle_cost += max_cost_in_step / 100.0;
        }

        // [LiDAR] 장애물 충돌 시 즉시 경로 폐기 (Hard Constraint)
        if (lethal_count > 0) {
            path.valid = false;
            path.cost = 1e10;
            continue;
        }

        if (valid_point_count > 0) {
            path.obstacle_cost /= valid_point_count;
            // [Camera] 차선 비용 평균 계산
            path.lane_cost = lane_penalty_sum / valid_point_count; 
        }
        
        // 곡률 비용 계산
        path.curvature_cost = 0.0;
        if (path.points.size() >= 3) {
            for (size_t i = 1; i < path.points.size() - 1; i++) {
                double x0 = path.points[i-1].x, y0 = path.points[i-1].y;
                double x1 = path.points[i].x,   y1 = path.points[i].y;
                double x2 = path.points[i+1].x, y2 = path.points[i+1].y;

                double dx1 = x1 - x0, dy1 = y1 - y0;
                double dx2 = x2 - x1, dy2 = y2 - y1;

                double denom = (std::sqrt(dx1*dx1 + dy1*dy1) * std::sqrt(dx2*dx2 + dy2*dy2) + 1e-6);
                double curvature = std::fabs((dx1*dy2 - dy1*dx2) / denom);

                path.points[i].curvature = curvature;
                path.curvature_cost = std::max(path.curvature_cost, curvature);
            }
        }

        path.offset_cost = std::fabs(path.offset);
        path.offset_change_cost = std::fabs(path.offset - last_selected_offset);
        
        path.cost = 0.0; 
    }
    
    // ========================================
    // 모든 코스트 정규화 (0~1 범위)
    // ========================================
    double max_obstacle = 0.0;
    double max_lane = 0.0; 
    double max_offset = 0.0;
    double max_curvature = 0.0;
    double max_offset_change = 0.0;
    
    for (const auto& path : lattice_ctrl.candidates) {
        if (!path.valid) continue;
        max_obstacle = std::max(max_obstacle, path.obstacle_cost);
        max_lane = std::max(max_lane, path.lane_cost);
        max_offset = std::max(max_offset, path.offset_cost);
        max_curvature = std::max(max_curvature, path.curvature_cost);
        max_offset_change = std::max(max_offset_change, path.offset_change_cost);
    }
    
    // 0으로 나누기 방지
    if (max_obstacle < 1e-6) max_obstacle = 1.0;
    if (max_lane < 1e-6) max_lane = 1.0;
    if (max_offset < 1e-6) max_offset = 1.0;
    if (max_curvature < 1e-6) max_curvature = 1.0;
    if (max_offset_change < 1e-6) max_offset_change = 1.0;
    
    // 정규화된 코스트 계산 (각 항목 0~1 범위)
    for (auto& path : lattice_ctrl.candidates) {
        if (!path.valid) {
            path.cost = 1e10;
            continue;
        }
        
        double norm_obstacle = path.obstacle_cost / max_obstacle;
        double norm_lane = path.lane_cost / max_lane; 
        double norm_offset = path.offset_cost / max_offset;
        double norm_curvature = path.curvature_cost / max_curvature;
        double norm_offset_change = path.offset_change_cost / max_offset_change;
        


        // 중심선 보너스 계산
        double center_bonus = 0.0;
        if (std::fabs(path.offset) < 0.1) {  // 10cm 이내
            center_bonus = 0.5;  // 강한 보너스
        } else if (std::fabs(path.offset) < 0.3) {  // 30cm 이내
            center_bonus = 0.2;  // 중간 보너스
        }
        
        // 일반 모드: 장애물 회피 + 차선 유지 + 중심선 선호도 강화
        path.cost = norm_obstacle * 0.40 +        
                    norm_lane * 0.25 +            
                    norm_curvature * 0.05 +       
                    norm_offset * 0.05 +          // 중심선 선호도 강화 (0.15 → 0.20)
                    norm_offset_change * 0.30 -   // 급격한 변화 억제 (0.15 → 0.30)
                    center_bonus;                 // 중심선 보너스 적용

    }
}

// ============================================================================
// 식 (1): 최대 선회 속도 계산
// v_max = sqrt(μg / κ)
// ============================================================================
double computeVmax(double curvature) {
    if (curvature < 1e-6) {
        return 100.0;  // 직선 구간: 매우 큰 값 반환
    }
    return std::sqrt(va_params.mu * va_params.g / curvature);
}

// ============================================================================
// 식 (2), (3) 수정: 동적 속도 임계값 계산
// v_low = α * v_max,  v_high = β * v_max
// ============================================================================
void computeDynamicSpeedThresholds(double curvature, double& v_low, double& v_high) {
    double v_max = computeVmax(curvature);
    
    v_low = va_params.alpha * v_max;
    v_high = va_params.beta * v_max;
    
    // 최소값 보장 (정지 상태 방지)
    v_low = std::max(v_low, 2.0);   // 최소 2 m/s (7.2 km/h)
    v_high = std::max(v_high, 5.0); // 최소 5 m/s (18 km/h)
    
    ROS_DEBUG_THROTTLE(1.0, "[VA] κ=%.4f → v_max=%.1f, v_low=%.1f, v_high=%.1f", 
                       curvature, v_max, v_low, v_high);
}

// ============================================================================
// 식 (4): 동적 LD 계산
// L_d = L_min + k_v * v
// ============================================================================
double computeDynamicLD(double velocity) {
    return va_params.L_min + va_params.k_v * velocity;
}

// ============================================================================
// 식 (5): 속도 스케일 팩터 계산 (동적 임계값 사용)
// ============================================================================
double computeSpeedScaleFactor(double velocity, double curvature) {
    double v_low, v_high;
    computeDynamicSpeedThresholds(curvature, v_low, v_high);
    
    if (velocity <= v_low) {
        return 1.0;  // 저속: 100% 샘플
    } 
    else if (velocity < v_high) {
        // 선형 보간: 식 (5)
        double ratio = (v_high - velocity) / (v_high - v_low);
        return va_params.S_min + (1.0 - va_params.S_min) * ratio;
    } 
    else {
        return va_params.S_min;  // 고속: 최소 샘플
    }
}

// ============================================================================
// 식 (6): 최종 샘플 개수 (홀수 보장)
// N_final = 2 * floor(N_sample * S_f / 2) + 1
// ============================================================================
int computeFinalSampleCount(int base_samples, double scale_factor) {
    int scaled = (int)(base_samples * scale_factor);
    int N_final = 2 * (scaled / 2) + 1;  // 홀수 보장
    return std::max(3, N_final);         // 최소 3개
}

// ============================================================================
// 식 (7): 횡방향 오프셋 계산
// d_i = D_max * i / ((N_final - 1) / 2)
// ============================================================================
double computeLateralOffset(int i, int N_final) {
    int half = (N_final - 1) / 2;
    if (half == 0) return 0.0;
    return va_params.D_max * (double)(i - half) / (double)half;
}

// ============================================================================
// 식 (10): 곡률 기반 한계 속도
// v_lim = sqrt(a_lat_max / κ_max)
// ============================================================================
double computeCurvatureLimitSpeed(double max_curvature) {
    if (max_curvature < 1e-6) {
        return 100.0;  // 직선: 제한 없음
    }
    return std::sqrt(va_params.a_lat_max / max_curvature);
}

// ============================================================================
// Lookahead Goal 찾기 (동적 LD 적용)
// ============================================================================
void findLookaheadGoalVA(const VehicleState& ego, int close_idx, LatticeControl& lattice_ctrl) {
    // 식 (4) 적용: 동적 LD
    double base_ld = computeDynamicLD(ego.vel);
    
    double ld_short = base_ld * 0.8;       // 50%
    double ld_medium = base_ld * 1.5;            // 100%
    double ld_long = base_ld * 1.8;        // 150%
    double ld_very_long = base_ld * 2.0;   // 200%

    int target_idx_short = close_idx;
    int target_idx_medium = close_idx;
    int target_idx_long = close_idx;
    int target_idx_very_long = close_idx;

    for (int i = close_idx; i < (int)waypoints.size(); i++) {
        double dx = waypoints[i].x - ego.x;
        double dy = waypoints[i].y - ego.y;
        double dist = std::sqrt(dx*dx + dy*dy);

        if (dist >= ld_short && target_idx_short == close_idx) {
            target_idx_short = i;
        }
        if (dist >= ld_medium && target_idx_medium == close_idx) {
            target_idx_medium = i;
        }
        if (dist >= ld_long && target_idx_long == close_idx) {
            target_idx_long = i;
        }
        if (dist >= ld_very_long && target_idx_very_long == close_idx) {
            target_idx_very_long = i;
            break;
        }
    }

    lattice_ctrl.target_idx_short = target_idx_short;
    lattice_ctrl.target_idx_medium = target_idx_medium;
    lattice_ctrl.target_idx_long = target_idx_long;
    lattice_ctrl.target_idx_very_long = target_idx_very_long;
    
    ROS_DEBUG_THROTTLE(1.0, "[VA-LD] v=%.1f → base_ld=%.1f, range=[%.1f, %.1f, %.1f, %.1f]",
                       ego.vel, base_ld, ld_short, ld_medium, ld_long, ld_very_long);
}

// ============================================================================
// Multi-Distance Adaptive Goals 생성 (논문 수식 완전 반영)
// ============================================================================
void generateMultiDistanceAdaptiveGoals(LatticeControl& lattice_ctrl) {
    lattice_ctrl.offset_goals.clear();
    
    // 현재 경로 곡률 추정 (waypoint 기반)
    double path_curvature = estimatePathCurvature(lattice_ctrl.close_idx);
    
    // 식 (5): 동적 스케일 팩터
    double speed_scale = computeSpeedScaleFactor(ego.vel, path_curvature);
    
    int total_samples = 0;
    
    auto generate_for_target = [&](int target_idx, int distance_priority) {
        if (target_idx < 0 || target_idx >= (int)waypoints.size()) return;
        
        double goal_ref_x = waypoints[target_idx].x;
        double goal_ref_y = waypoints[target_idx].y;
        double dx = 0.0, dy = 0.0;
        
        if (target_idx < (int)waypoints.size() - 1) {
            dx = waypoints[target_idx + 1].x - waypoints[target_idx].x;
            dy = waypoints[target_idx + 1].y - waypoints[target_idx].y;
        }
        
        double len = std::max(1e-6, std::sqrt(dx*dx + dy*dy));
        double norm_x = -dy / len, norm_y = dx / len;
        double yaw_global = std::atan2(dy, dx);
        
        // 식 (6): 최종 샘플 개수
        int N_final = computeFinalSampleCount(
            va_params.base_samples[distance_priority], speed_scale);
        
        // 식 (7): 횡방향 오프셋 목표점 생성
        for (int i = 0; i < N_final; i++) {
            double offset = computeLateralOffset(i, N_final);
            
            OffsetGoal goal;
            goal.global_x = goal_ref_x + offset * norm_x;
            goal.global_y = goal_ref_y + offset * norm_y;
            goal.global_yaw = yaw_global;
            goal.offset = offset;
            
            lattice_ctrl.offset_goals.push_back(goal);
            total_samples++;
        }
    };
    
    // 거리별 생성 (속도별 우선순위 조정)
    double speed_threshold = 8.5;  // m/s = 30.0 km/h
    
    if (ego.vel < speed_threshold) {
        // 저속 (장애물 많음): 근거리 우선 (타이트한 회피)
        generate_for_target(lattice_ctrl.target_idx_short, 0);
        generate_for_target(lattice_ctrl.target_idx_medium, 1);
        generate_for_target(lattice_ctrl.target_idx_long, 2);
        generate_for_target(lattice_ctrl.target_idx_very_long, 3);
        ROS_INFO_THROTTLE(2.0, "[VA-MDS] LOW_SPEED(%.1f km/h) - Close-first priority | κ=%.4f | S_f=%.2f | Total=%d", 
                          ego.vel * 3.6, path_curvature, speed_scale, total_samples);
    } else {
        // 고속 (장애물 적음): 원거리 우선 (부드러운 궤적)
        generate_for_target(lattice_ctrl.target_idx_very_long, 3);
        generate_for_target(lattice_ctrl.target_idx_long, 2);
        generate_for_target(lattice_ctrl.target_idx_medium, 1);
        generate_for_target(lattice_ctrl.target_idx_short, 0);
        ROS_INFO_THROTTLE(2.0, "[VA-MDS] HIGH_SPEED(%.1f km/h) - Far-first priority | κ=%.4f | S_f=%.2f | Total=%d", 
                          ego.vel * 3.6, path_curvature, speed_scale, total_samples);
    }
}

// ============================================================================
// VA 특화: 차량 폭 기반 경로 유효성 비율 계산
// ============================================================================
void calculatePathValidityRatiosVA(LatticeControl& lattice_ctrl) {
    // 차량 폭 기반 검사 범위 계산
    double vehicle_width = 2.7;  // 차량 폭 (m)
    double costmap_resolution = 0.05;  // Costmap 해상도 (m)
    int cells_per_meter = (int)(1.0 / costmap_resolution);  // 20 cells/m
    int vehicle_width_cells = (int)(vehicle_width * cells_per_meter);  // ~54 cells
    
    // 거리별 검사 범위 (수직 오프셋)
    int close_search_radius = vehicle_width_cells;  // 근거리: ±차량폭
    int mid_search_radius = (int)(vehicle_width_cells * 0.75);  // 중거리: ±0.75*차량폭
    int long_search_radius = (int)(vehicle_width_cells * 0.5);  // 원거리: ±0.5*차량폭
    
    // selectBestPath에서 선택된 best_path 인덱스를 찾기
    int best_idx = -1;
    for (int i = 0; i < (int)lattice_ctrl.candidates.size(); i++) {
        if (lattice_ctrl.candidates[i].offset == lattice_ctrl.best_path.offset &&
            lattice_ctrl.candidates[i].valid == lattice_ctrl.best_path.valid) {
            best_idx = i;
            break;
        }
    }
    
    if (best_idx == -1) {
        lattice_ctrl.ego_path_ratio = 0.0;
        lattice_ctrl.valid_path_ratio = 0.0;
        lattice_ctrl.very_long_path_ratio = 0.0;
        return;
    }
    
    // [1] ego_path_ratio: 선택 경로 ±차량폭 범위 (근거리)
    {
        int start_idx = std::max(0, best_idx - close_search_radius);
        int end_idx = std::min((int)lattice_ctrl.candidates.size() - 1, best_idx + close_search_radius);
        
        int valid_count = 0;
        int search_range = end_idx - start_idx + 1;
        
        for (int i = start_idx; i <= end_idx; i++) {
            if (lattice_ctrl.candidates[i].valid) {
                valid_count++;
            }
        }
        
        lattice_ctrl.ego_path_ratio = (double)valid_count / (double)search_range;
        ROS_DEBUG_THROTTLE(1.0, "[VA-Ratio] ego_path_ratio (근거리±%.2fm): %.2f (%d/%d)", 
                          vehicle_width, lattice_ctrl.ego_path_ratio, valid_count, search_range);
    }
    
    // [2] valid_path_ratio: 선택 경로 ±0.75*차량폭 범위 (중거리)
    {
        int start_idx = std::max(0, best_idx - mid_search_radius);
        int end_idx = std::min((int)lattice_ctrl.candidates.size() - 1, best_idx + mid_search_radius);
        
        int valid_count = 0;
        int search_range = end_idx - start_idx + 1;
        
        for (int i = start_idx; i <= end_idx; i++) {
            if (lattice_ctrl.candidates[i].valid) {
                valid_count++;
            }
        }
        
        lattice_ctrl.valid_path_ratio = (double)valid_count / (double)search_range;
        ROS_DEBUG_THROTTLE(1.0, "[VA-Ratio] valid_path_ratio (중거리±%.2fm): %.2f (%d/%d)", 
                          vehicle_width * 0.75, lattice_ctrl.valid_path_ratio, valid_count, search_range);
    }
    
    // [3] very_long_path_ratio: 선택 경로 ±0.5*차량폭 범위 (원거리)
    {
        int start_idx = std::max(0, best_idx - long_search_radius);
        int end_idx = std::min((int)lattice_ctrl.candidates.size() - 1, best_idx + long_search_radius);
        
        int valid_count = 0;
        int search_range = end_idx - start_idx + 1;
        
        for (int i = start_idx; i <= end_idx; i++) {
            if (lattice_ctrl.candidates[i].valid) {
                valid_count++;
            }
        }
        
        lattice_ctrl.very_long_path_ratio = (double)valid_count / (double)search_range;
        ROS_DEBUG_THROTTLE(1.0, "[VA-Ratio] very_long_path_ratio (원거리±%.2fm): %.2f (%d/%d)", 
                          vehicle_width * 0.5, lattice_ctrl.very_long_path_ratio, valid_count, search_range);
    }
}

// ============================================================================
// 경로 곡률 추정 (waypoint 기반)
// ============================================================================
double estimatePathCurvature(int close_idx) {
    if (waypoints.size() < 3) return 0.0;
    
    int lookahead = std::min(20, (int)waypoints.size() - close_idx - 2);
    double max_curvature = 0.0;
    
    for (int i = close_idx; i < close_idx + lookahead - 1; i++) {
        double x0 = waypoints[i].x,   y0 = waypoints[i].y;
        double x1 = waypoints[i+1].x, y1 = waypoints[i+1].y;
        double x2 = waypoints[i+2].x, y2 = waypoints[i+2].y;
        
        double dx1 = x1 - x0, dy1 = y1 - y0;
        double dx2 = x2 - x1, dy2 = y2 - y1;
        
        double cross = dx1 * dy2 - dy1 * dx2;
        double len1 = std::sqrt(dx1*dx1 + dy1*dy1);
        double len2 = std::sqrt(dx2*dx2 + dy2*dy2);
        
        if (len1 > 0.1 && len2 > 0.1) {
            double curvature = std::abs(cross) / (len1 * len2);
            max_curvature = std::max(max_curvature, curvature);
        }
    }
    
    return max_curvature;
}

// ============================================================================
// 목표 속도 계산 (곡률 제한 + 장애물 기반 감속)
// ============================================================================
void getTargetSpeedVA(double max_curvature, double& out_target_vel, const VehicleState& ego) {
    if (applyPathEndStop(ego, out_target_vel)) {
        return;
    }

    double ego_ratio = lattice_ctrl.ego_path_ratio;             // 근거리: 내 차선 유효성
    double valid_ratio = lattice_ctrl.valid_path_ratio;         // 중거리: 선택 경로 주변 유효성
    double very_long_ratio = lattice_ctrl.very_long_path_ratio; // 원거리: 먼 거리 중앙 유효성
    // ====================================================
    // 1단계: 곡률 기반 최대속도 제한 (식 10)
    // v_lim = sqrt(a_lat_max / κ_max)
    // ====================================================
    double v_lim = computeCurvatureLimitSpeed(max_curvature);
    double base_vel = std::min(target_vel, v_lim);
    
    ROS_DEBUG_THROTTLE(1.0, "[VA-Speed] κ_max=%.4f → v_lim=%.1f km/h", 
                       max_curvature, v_lim * 3.6);
    
    // ====================================================
    // 2단계: 장애물 기반 감속
    // ====================================================
    
    // [근거리 장애물] 내 차선 주변 경로 유효성 (최우선)
    if (ego_ratio < 0.7) {
        base_vel *= 0.6;  // 30% 수준
        ROS_WARN_THROTTLE(1.0, "[VA-Speed] [Obstacle] CRITICAL - Ego lane blocked: %.1f km/h (ratio: %.2f)", 
                        base_vel * 3.6, ego_ratio);
    } 
    // [중거리 장애물] 선택된 경로 주변 유효성 (근거리 안전하면 판단)
    else if (valid_ratio < 0.5) {
        base_vel *= 0.7;  // 40% 수준
        ROS_ERROR_THROTTLE(1.0, "[VA-Speed] [Obstacle] EMERGENCY - Path completely blocked: %.1f km/h (ratio: %.2f)", 
                        base_vel * 3.6, valid_ratio);
    } 
    // [원거리 장애물] 먼 거리 중앙 유효성 (근거리/중거리 안전하면 판단)
    else if (very_long_ratio < 0.8) {
        base_vel *= 0.8;  // 80% 수준
        ROS_WARN_THROTTLE(1.0, "[VA-Speed] [Obstacle] VeryLong: %.1f km/h (ratio: %.2f)", 
                        base_vel * 3.6, very_long_ratio);
    }
    out_target_vel = base_vel;
    ROS_DEBUG_THROTTLE(1.0, "[VA-Speed] Final target speed: %.1f km/h (%.2f m/s)", base_vel * 3.6, base_vel);

}