#include "SteeringMonitor.hpp"
#include <cstdlib>
#include <iostream>

// Global instance definition
SteeringContinuityMonitor steering_monitor;

SteeringContinuityMonitor::SteeringContinuityMonitor() {
    // 로그 디렉터리 생성 (워크스페이스 cwd = 레포 루트)
    system("mkdir -p logs");
    
    log_file.open("logs/steering_continuity.csv");
    log_file << "timestamp,steering_angle,steering_rate,velocity,planner_mode,status\n";
    log_file.flush();
    
    ROS_INFO("[SteeringMonitor] Initialized - logging to steering_continuity.csv");
}

SteeringContinuityMonitor::~SteeringContinuityMonitor() {
    if (log_file.is_open()) {
        log_file.close();
    }
}

void SteeringContinuityMonitor::updateSteering(double steering_angle_deg, double velocity_ms, int mode) {
    ros::Time current_time = ros::Time::now();
    
    if (!initialized) {
        last_steering_angle = steering_angle_deg;
        last_update_time = current_time;
        initialized = true;
        return;
    }
    
    double dt = (current_time - last_update_time).toSec();
    
    // 유효한 시간 간격 체크
    if (dt > 0.001 && dt < 1.0) {
        // 조향 변화율 계산 (deg/s)
        double steering_rate = (steering_angle_deg - last_steering_angle) / dt;
        
        SteeringData data;
        data.timestamp = current_time.toSec();
        data.steering_angle = steering_angle_deg;
        data.steering_rate = steering_rate;
        data.velocity = velocity_ms * 3.6;  // km/h
        data.planner_mode = mode;
        
        // 연속성 상태 평가
        std::string status = evaluateContinuity(steering_rate);
        
        // 기록 저장
        steering_history.push_back(data);
        if (steering_history.size() > (size_t)params.history_size) {
            steering_history.pop_front();
        }
        
        // CSV 로그
        log_file << std::fixed << std::setprecision(3)
                << data.timestamp << ","
                << data.steering_angle << ","
                << data.steering_rate << ","
                << data.velocity << ","
                << data.planner_mode << ","
                << status << "\n";
        log_file.flush();
        
        // 실시간 경고
        if (std::abs(steering_rate) > params.max_rate_critical) {
            ROS_WARN("[Steering] CRITICAL rate: %.2f deg/s (mode=%d)", steering_rate, mode);
        } else if (std::abs(steering_rate) > params.max_rate_warning) {
            ROS_DEBUG("[Steering] WARNING rate: %.2f deg/s (mode=%d)", steering_rate, mode);
        }
    }
    
    last_steering_angle = steering_angle_deg;
    last_update_time = current_time;
}

std::string SteeringContinuityMonitor::evaluateContinuity(double rate) {
    double abs_rate = std::abs(rate);
    if (abs_rate > params.max_rate_critical) return "CRITICAL";
    if (abs_rate > params.max_rate_warning) return "WARNING";
    return "NORMAL";
}

void SteeringContinuityMonitor::printStatistics() {
    if (steering_history.size() < 10) return;
    
    std::map<int, std::vector<double>> mode_rates;
    for (const auto& data : steering_history) {
        mode_rates[data.planner_mode].push_back(std::abs(data.steering_rate));
    }
    
    ROS_INFO("\n=== Steering Continuity Statistics ===");
    for (auto& pair : mode_rates) {
        int mode = pair.first;
        std::vector<double>& rates = pair.second;
        
        if (rates.empty()) continue;
        
        std::sort(rates.begin(), rates.end());
        double mean = std::accumulate(rates.begin(), rates.end(), 0.0) / rates.size();
        double max_rate = rates.back();
        double p95 = rates[static_cast<size_t>(rates.size() * 0.95)];
        
        std::string mode_name = (mode == 0) ? "Static" : 
                              (mode == 1) ? "VA-MDS" : 
                              (mode == 3) ? "VA-NonMDS" : "Unknown";
        
        ROS_INFO("[%s] Samples: %zu, Mean: %.2f, Max: %.2f, P95: %.2f deg/s", 
                 mode_name.c_str(), rates.size(), mean, max_rate, p95);
    }
    ROS_INFO("======================================");
}