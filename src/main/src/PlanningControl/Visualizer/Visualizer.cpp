#include "Visualizer.hpp"
#include "Global.hpp"
#include <geometry_msgs/Point.h>

void publishCandidatePaths() {
    visualization_msgs::MarkerArray marker_array;
    
    double best_offset = lattice_ctrl.best_path.offset;

    for (size_t i = 0; i < lattice_ctrl.candidates.size(); i++) {
        const auto& candidate = lattice_ctrl.candidates[i];
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link"; 
        marker.header.stamp = ros::Time::now();
        marker.ns = "candidate_paths";
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0; 
        
        if (!candidate.valid) {
            marker.scale.x = 0.03; 
            marker.color.r = 0.5; marker.color.g = 0.5; marker.color.b = 0.5; marker.color.a = 0.5;
        } 
        else if (fabs(candidate.offset - best_offset) < 0.01) {
            marker.scale.x = 0.15; 
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
        } 
        else {
            marker.scale.x = 0.05; 
            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 0.6;
        }
        
        for (size_t j = 0; j < candidate.points.size(); j++) {
            geometry_msgs::Point p_visual;
            p_visual.x = candidate.points[j].x;
            p_visual.y = candidate.points[j].y;
            p_visual.z = 0.0; 
            marker.points.push_back(p_visual);
        }
        
        marker_array.markers.push_back(marker);
    }
    
    marker_pub.publish(marker_array);
}

void publishVehicleFootprint() { // 내 차(노란 박스) 그리기
    visualization_msgs::Marker car_marker;
    car_marker.header.frame_id = "base_link";
    car_marker.header.stamp = ros::Time::now();
    car_marker.ns = "ego_shape";
    car_marker.id = 0;
    car_marker.type = visualization_msgs::Marker::CUBE;
    car_marker.action = visualization_msgs::Marker::ADD;

    double front_len = planner_params.vehicle_front_offset; 
    double rear_len  = 1.0; 
    double width     = 2.0; 

    car_marker.pose.position.x = (front_len - rear_len) / 2.0;
    car_marker.pose.position.y = 0.0;
    car_marker.pose.position.z = 0.5; 
    car_marker.pose.orientation.x = 0.0;
    car_marker.pose.orientation.y = 0.0;
    car_marker.pose.orientation.z = 0.0;
    car_marker.pose.orientation.w = 1.0;

    car_marker.scale.x = front_len + rear_len; 
    car_marker.scale.y = width;
    car_marker.scale.z = 1.5; 

    car_marker.color.r = 1.0;
    car_marker.color.g = 1.0;
    car_marker.color.b = 0.0;
    car_marker.color.a = 0.5; 

    visualization_msgs::MarkerArray arr;
    arr.markers.push_back(car_marker);
    marker_pub.publish(arr);
}

void publishLocalPath() { // 로컬 경로를 파란색 MarkerArray로 발행
    if (lattice_ctrl.best_path.points.empty()) {
        return;
    }
    
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "base_link";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "local_path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    
    path_marker.pose.orientation.x = 0.0;
    path_marker.pose.orientation.y = 0.0;
    path_marker.pose.orientation.z = 0.0;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.3;
    
    path_marker.color.r = 0.0;
    path_marker.color.g = 1.0;
    path_marker.color.b = 0.0;
    path_marker.color.a = 0.6;
    
    for (const auto& point : lattice_ctrl.best_path.points) {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0;
        path_marker.points.push_back(p);
    }
    
    visualization_msgs::MarkerArray arr;
    arr.markers.push_back(path_marker);
    local_path_pub.publish(arr);
}

void publishCostmapGrid() {
    // Costmap 격자 시각화
    visualization_msgs::MarkerArray grid_markers;
    
    // 격자 크기 설정 (예시: 100m x 100m 영역을 1m 간격으로)
    const float GRID_SIZE = 1.0f;        // 격자 간격 (m)
    const float GRID_RANGE = 50.0f;      // 격자 범위 (±50m)
    const float LINE_WIDTH = 0.02f;      // 선 두께
    
    int marker_id = 0;
    
    // 세로선 (X 방향으로 평행한 선들)
    for (float y = -GRID_RANGE; y <= GRID_RANGE; y += GRID_SIZE) {
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "base_link";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "costmap_grid";
        line_marker.id = marker_id++;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        
        line_marker.pose.orientation.w = 1.0;
        line_marker.scale.x = LINE_WIDTH;
        
        // 격자 색상 (연한 회색)
        line_marker.color.r = 0.5;
        line_marker.color.g = 0.5; 
        line_marker.color.b = 0.5;
        line_marker.color.a = 0.3;
        
        // 선의 시작점과 끝점
        geometry_msgs::Point p1, p2;
        p1.x = -GRID_RANGE; p1.y = y; p1.z = 0.0;
        p2.x = GRID_RANGE;  p2.y = y; p2.z = 0.0;
        
        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);
        
        grid_markers.markers.push_back(line_marker);
    }
    
    // 가로선 (Y 방향으로 평행한 선들)  
    for (float x = -GRID_RANGE; x <= GRID_RANGE; x += GRID_SIZE) {
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "base_link";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "costmap_grid";
        line_marker.id = marker_id++;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        
        line_marker.pose.orientation.w = 1.0;
        line_marker.scale.x = LINE_WIDTH;
        
        // 격자 색상 (연한 회색)
        line_marker.color.r = 0.5;
        line_marker.color.g = 0.5;
        line_marker.color.b = 0.5; 
        line_marker.color.a = 0.3;
        
        // 선의 시작점과 끝점
        geometry_msgs::Point p1, p2;
        p1.x = x; p1.y = -GRID_RANGE; p1.z = 0.0;
        p2.x = x; p2.y = GRID_RANGE;  p2.z = 0.0;
        
        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);
        
        grid_markers.markers.push_back(line_marker);
    }
    
    // 중심축 강조 (X축 - 빨간색)
    visualization_msgs::Marker x_axis;
    x_axis.header.frame_id = "base_link";
    x_axis.header.stamp = ros::Time::now();
    x_axis.ns = "costmap_grid";
    x_axis.id = marker_id++;
    x_axis.type = visualization_msgs::Marker::LINE_STRIP;
    x_axis.action = visualization_msgs::Marker::ADD;
    
    x_axis.pose.orientation.w = 1.0;
    x_axis.scale.x = LINE_WIDTH * 2;
    x_axis.color.r = 1.0; x_axis.color.g = 0.0; x_axis.color.b = 0.0; x_axis.color.a = 0.8;
    
    geometry_msgs::Point px1, px2;
    px1.x = -GRID_RANGE; px1.y = 0; px1.z = 0.0;
    px2.x = GRID_RANGE;  px2.y = 0; px2.z = 0.0;
    x_axis.points.push_back(px1);
    x_axis.points.push_back(px2);
    grid_markers.markers.push_back(x_axis);
    
    // 중심축 강조 (Y축 - 녹색)
    visualization_msgs::Marker y_axis;
    y_axis.header.frame_id = "base_link";
    y_axis.header.stamp = ros::Time::now();
    y_axis.ns = "costmap_grid";
    y_axis.id = marker_id++;
    y_axis.type = visualization_msgs::Marker::LINE_STRIP;
    y_axis.action = visualization_msgs::Marker::ADD;
    
    y_axis.pose.orientation.w = 1.0;
    y_axis.scale.x = LINE_WIDTH * 2;
    y_axis.color.r = 0.0; y_axis.color.g = 1.0; y_axis.color.b = 0.0; y_axis.color.a = 0.8;
    
    geometry_msgs::Point py1, py2;
    py1.x = 0; py1.y = -GRID_RANGE; py1.z = 0.0;
    py2.x = 0; py2.y = GRID_RANGE;  py2.z = 0.0;
    y_axis.points.push_back(py1);
    y_axis.points.push_back(py2);
    grid_markers.markers.push_back(y_axis);
    
    marker_pub.publish(grid_markers);
}