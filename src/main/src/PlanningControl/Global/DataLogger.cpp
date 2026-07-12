#include "DataLogger.hpp"
#include "Global.hpp"

#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <cmath>
#include <sys/stat.h>
#include <dirent.h>

// ========================================
// 내부 변수
// ========================================
static std::ofstream log_file;
static double prev_steering = 0.0;
static double prev_timestamp = 0.0;
static bool logger_initialized = false;
static double last_log_timestamp = 0.0;  // 3초 간격 저장용

// ========================================
// 폴더 내 기존 csv 파일 수 카운트 (run_id 자동 부여용)
// ========================================
static int countExistingRuns(const std::string& dir) {
    int count = 0;
    DIR* dp = opendir(dir.c_str());
    if (dp == nullptr) return 0;

    struct dirent* entry;
    while ((entry = readdir(dp)) != nullptr) {
        std::string name(entry->d_name);
        // "run_" 로 시작하고 ".csv"로 끝나는 파일만 카운트
        if (name.find("run_") == 0 && name.find(".csv") != std::string::npos) {
            count++;
        }
    }
    closedir(dp);
    return count;
}

// ========================================
// 폴더 생성 (없으면)
// ========================================
static void ensureDirectory(const std::string& dir) {
    struct stat st;
    if (stat(dir.c_str(), &st) != 0) {
        mkdir(dir.c_str(), 0777);
        ROS_INFO("[Logger] Created directory: %s", dir.c_str());
    }
}

// ========================================
// 초기화
// ========================================
void initDataLogger(const std::string& save_dir) {
    ensureDirectory(save_dir);

    int run_id = countExistingRuns(save_dir) + 1;

    std::stringstream ss;
    ss << save_dir << "/run_" << std::setfill('0') << std::setw(2) << run_id << ".csv";
    std::string file_path = ss.str();

    log_file.open(file_path);
    if (!log_file.is_open()) {
        ROS_ERROR("[Logger] Failed to open: %s", file_path.c_str());
        return;
    }

    // CSV 헤더
    log_file << "timestamp,"
             << "ego_x,ego_y,ego_yaw,ego_vel,ego_steering,"
             << "target_local_path_x,target_local_path_y,"
             << "steering_rate,planner_mode,status"
             << std::endl;

    prev_steering = 0.0;
    prev_timestamp = 0.0;
    logger_initialized = true;

    ROS_INFO("[Logger] Logging to: %s (Run #%d)", file_path.c_str(), run_id);
}

// ========================================
// 매 timestep 데이터 기록 (3초 간격)
//  26.3.27 에 추가 -> LEARNING BY CHEATING
// ========================================
void logData() {
    if (!logger_initialized || !log_file.is_open()) return;

    double timestamp = ros::Time::now().toSec();
    
    // 1초 간격으로만 저장 // ** 
    if (timestamp - last_log_timestamp < 1.0) {
        return;
    }
    last_log_timestamp = timestamp;

    // 선택된 경로 정보
    double path_offset = lattice_ctrl.best_path.offset;
    double path_cost = lattice_ctrl.best_path.cost;

    // steering은 ctrl에서 가져옴
    double steering = ctrl.steering;

    // Steering rate 계산 (deg/s)
    double steering_rate = 0.0;
    if (prev_timestamp > 0.0) {
        double dt = timestamp - prev_timestamp;
        if (dt > 0.0) {
            steering_rate = (steering - prev_steering) * 180.0 / M_PI / dt; // rad/s -> deg/s
        }
    }

    // Planner mode (기본값 1, 실제 플래너에 따라 수정 필요)
    int planner_mode = 1; // 0: Static, 1: VA-MDS, 3: VA-NonMDS

    // Status 판단 (steering rate 기반)
    std::string status;
    double abs_steering_rate = std::abs(steering_rate);
    if (abs_steering_rate >= 50.0) {
        status = "CRITICAL";
    } else if (abs_steering_rate >= 30.0) {
        status = "WARNING";
    } else {
        status = "NORMAL";
    }

    log_file << std::fixed << std::setprecision(6)
             << timestamp << ","
             << ego.x << "," << ego.y << "," << ego.yaw << ","
             << std::setprecision(4) // 소수점, 
             << ego.vel << "," << steering << ","
             << std::setprecision(2)
             << target_local_path_x << ","  // ← Learning by cheating 추가
             << target_local_path_y << ","  // ← Learning by cheating 추가
             << steering_rate << ","
             << planner_mode << ","
             << status
             << std::endl;

    prev_steering = steering;
    prev_timestamp = timestamp;
}

// ========================================
// 종료
// ========================================
void closeDataLogger() {
    if (log_file.is_open()) {
        log_file.close();
        ROS_INFO("[Logger] File closed.");
    }
    logger_initialized = false;
}