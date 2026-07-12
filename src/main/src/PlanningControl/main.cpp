#include "Global.hpp"
#include "Planning.hpp"
#include "Visualizer.hpp"
#include "Control.hpp"
#include "parameter_loader.hpp"
#include "SteeringMonitor.hpp"
#include "DataLogger.hpp"

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <morai_msgs/GPSMessage.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <morai_msgs/CtrlCmd.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>

using namespace std;
mutex costmap_mutex;

// ========================================
// Callback
// ========================================

void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {

    lock_guard<std::mutex> lock(costmap_mutex);

    costmap_info.msg = msg; // 안전하게 shared_ptr 저장
    costmap_info.origin_x = msg->info.origin.position.x;
    costmap_info.origin_y = msg->info.origin.position.y;
    costmap_info.resolution = msg->info.resolution;
    costmap_info.width = (int)msg->info.width;
    costmap_info.height = (int)msg->info.height;

    ROS_INFO_ONCE("[Costmap] Received! %dx%d, res: %.2fm",
                  costmap_info.width, costmap_info.height, costmap_info.resolution);
}

void CameraCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    // 굳이 mutex 잠글 필요까진 없지만 안전하게 하려면 사용
    // lock_guard<std::mutex> lock(costmap_mutex); 
    Camera_costmap_info.msg = msg;
    Camera_costmap_info.origin_x = msg->info.origin.position.x;
    Camera_costmap_info.origin_y = msg->info.origin.position.y;
    Camera_costmap_info.resolution = msg->info.resolution;
    Camera_costmap_info.width = (int)msg->info.width;
    Camera_costmap_info.height = (int)msg->info.height;
}


void gpsCallback(const morai_msgs::GPSMessage::ConstPtr& msg) {

    if (msg ->status ==0)
    { gps_jamming_perception = true; return;}
    else
    { gps_jamming_perception = false;}
    
    gps_state.last_valid_gps_time = ros::Time::now();

     // 2. 좌표계 초기화 (기존 코드 유지)
    if (!coord_ref_initialized) {
        coord_ref.lat0 = msg->latitude;
        coord_ref.lon0 = msg->longitude;
        coord_ref.h0 = msg->altitude;
        wgs84ToECEF(coord_ref.lat0, coord_ref.lon0, coord_ref.h0,
                    coord_ref.x0_ecef, coord_ref.y0_ecef, coord_ref.z0_ecef);
        coord_ref_initialized = true;
        ROS_INFO("[GPS] Reference point set: lat=%.6f, lon=%.6f", 
                 coord_ref.lat0, coord_ref.lon0);
    }

    // 4. 좌표 변환 (기존 코드 유지)
    double x, y, z;
    wgs84ToENU(msg->latitude, msg->longitude, msg->altitude,
               coord_ref, x, y, z);
    ego.x = x;
    ego.y = y;
}

void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
    ego.vel = msg->velocity.x;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ego.yaw = quaternionToYaw(msg->orientation.x,
                              msg->orientation.y,
                              msg->orientation.z,
                              msg->orientation.w);
}

void laneCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {

    lane.offset = msg->data[0];
    lane.angle = msg->data[1];
}

void driving_modeCallback(const std_msgs::Int32::ConstPtr& msg) {
    driving_mode = msg->data;  // 0: Normal, 1: Overtaking
    ROS_INFO("[Mode] Driving mode changed to: %d", driving_mode);
}

void mainControlLoop(const ros::TimerEvent&) {
    static ros::Time last_stats_time = ros::Time::now();

    if (gps_jamming_perception) {
        JammingPlanningProcess();
        ControlProcess();
    } else {
        // 실험 모드 선택
        // Mode 0: Static (Fixed LD + Fixed Resolution)
        // Mode 1: Dynamic LD Only (Speed-proportional LD + Fixed Resolution)
        // Mode 2: Max High-Res (Fixed LD + Max Resolution)
        // Mode 3: VA-MDS (Proposed - Speed-adaptive LD + Dynamic Resolution)
        switch(EXPERIMENT_MODE) {
            case 0:
                StaticLatticePlannerProcess();  // Baseline 0: Static LD
                break;
            case 1:
                DynamicLDOnlyProcess();         // Baseline 1: Dynamic LD only
                break;
            case 2:
                MaxHighResProcess();            // Baseline 2: Max High-Res
                break;
            case 3:
                VALatticePlanningProcess();     // Proposed: VA-MDS (Full adaptation)
                break;
            default:
                StaticLatticePlannerProcess();  // Default to baseline
                break;
        }
        ControlProcess();
        
        //  조향 모니터링 추가
        double steering_angle_deg = ctrl.steering * 180.0 / M_PI;
        steering_monitor.updateSteering(steering_angle_deg, ego.vel, EXPERIMENT_MODE);
        
        //
        logData(); // <- 데이터 기록용 추가 
        publishCandidatePaths();
        publishVehicleFootprint();
        publishLocalPath();
        publishCostmapGrid();  // 코스트맵 격자 시각화
    }
    
    // 주기적 통계 출력 (15초마다)
    if ((ros::Time::now() - last_stats_time).toSec() > 15.0) {
        steering_monitor.printStatistics();
        last_stats_time = ros::Time::now();
    }
}

// ========================================
// main
// ========================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "lattice_planning_node");
    ros::NodeHandle nh;

    // 실험 모드 설정 (환경변수 또는 ROS 파라미터)
    if (char* env_mode = getenv("EXPERIMENT_MODE")) {
        EXPERIMENT_MODE = std::stoi(env_mode);
    } else {
        nh.param<int>("experiment_mode", EXPERIMENT_MODE, 0);
    }
    
    // 시나리오 설정 (환경변수 또는 ROS 파라미터)
    std::string scenario_name = "default";
    if (char* env_scenario = getenv("SCENARIO_NAME")) {
        scenario_name = std::string(env_scenario);
    } else {
        nh.param<std::string>("scenario_name", scenario_name, "default");
    }
    
    // 실험 모드 이름 매핑
    std::string mode_names[] = {"Static", "DynamicLD", "MaxHighRes", "VA-MDS"};
    std::string mode_name = (EXPERIMENT_MODE < 4) ? mode_names[EXPERIMENT_MODE] : "Unknown";
    
    // 런 로그: logs/runs/<scenario>_<timestamp> (gitignore)
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream time_ss;
    time_ss << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S");
    
    std::string log_folder = "logs/runs/" + mode_name + "_" + scenario_name + "_" + time_ss.str();
    system("mkdir -p logs/runs");
    
    ROS_INFO("[EXPERIMENT] Mode %d (%s) | Scenario: %s", EXPERIMENT_MODE, mode_name.c_str(), scenario_name.c_str());
    ROS_INFO("[LOG] Data will be saved to: %s", log_folder.c_str());

    // Waypoints 로드
    loadWaypoints();
    load_overtakingZone();
    loadNoCameraZones();
    loadNoLidarZones();

    // 파라미터 초기화
    initializePlannerParameters();
    initializeControlParameters();

    // DATA 기록 경로 (동적 설정)
    initDataLogger(log_folder);
    
    
    // 초기값
    ego.x = 0.0;
    ego.y = 0.0;
    ego.yaw = 0.0;
    ego.vel = 0.0;
    
    // Subscriber
    ros::Subscriber gps_sub = nh.subscribe("/gps", 1, gpsCallback);
    ros::Subscriber imu_sub = nh.subscribe("/imu", 1, imuCallback);
    ros::Subscriber ego_sub = nh.subscribe("/Ego_topic", 1, egoCallback);
    ros::Subscriber costmap_sub = nh.subscribe("/costmap", 1, costmapCallback);
    // ros::Subscriber lane_sub = nh.subscribe<camera::LaneInfo>("/lane/path", 1, laneCallback);
    // path_msg = Float32MultiArray()
    ros::Subscriber lane_sub = nh.subscribe<std_msgs::Float32MultiArray>("/lane/path", 1, laneCallback);
    ros::Subscriber camera_costmap_sub = nh.subscribe("/costmap/camera", 1, CameraCostmapCallback);
    ros::Subscriber mode_sub = nh.subscribe("/driving_mode", 10, driving_modeCallback);
    // Publisher
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/lattice/paths", 1);
    local_path_pub = nh.advertise<visualization_msgs::MarkerArray>("/local_path", 1);
    cmd_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd_0", 1);

    // Timer (10Hz)
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), mainControlLoop);
    
    ROS_INFO("Node Started! (Visualization Only - No Control)");
    ROS_INFO("========================================");
    
    ros::AsyncSpinner spinner(4); // 일꾼 4명 고용 // 4개의 스레드를 할당하여 전역 콜백 큐(Global Callback Queue)를 병렬로 처리하는 스피너 생성
    spinner.start();              // 일꾼들 투입 (백그라운드에서 돔) // 스피너를 백그라운드 스레드에서 비동기적으로 시작 (메인 스레드는 차단되지 않음)
    ros::waitForShutdown();       // 메인 스레드는 여기서 프로그램 안 꺼지게 대기
 
    return 0;
}