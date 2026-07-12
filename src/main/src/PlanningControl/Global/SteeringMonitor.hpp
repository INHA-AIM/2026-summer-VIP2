#ifndef STEERING_MONITOR_HPP
#define STEERING_MONITOR_HPP

#include <ros/ros.h>
#include <deque>
#include <fstream>
#include <map>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iomanip>

struct SteeringData {
    double timestamp;
    double steering_angle;
    double steering_rate;
    double velocity;
    int planner_mode;
};

class SteeringContinuityMonitor {
private:
    std::deque<SteeringData> steering_history;
    std::ofstream log_file;
    ros::Time last_update_time;
    double last_steering_angle = 0.0;
    bool initialized = false;
    
    struct {
        double max_rate_warning = 30.0;    // deg/s
        double max_rate_critical = 50.0;   // deg/s
        int history_size = 1000;
    } params;

public:
    SteeringContinuityMonitor();
    ~SteeringContinuityMonitor();
    
    void updateSteering(double steering_angle_deg, double velocity_ms, int mode);
    void printStatistics();
    std::string evaluateContinuity(double rate);
};

// Global instance declaration
extern SteeringContinuityMonitor steering_monitor;

#endif