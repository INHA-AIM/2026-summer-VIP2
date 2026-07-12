// #include "Kalman.hpp"
// #include <iostream>
// #include <cmath>

// // ===========================================================================================
// // EKFTracker - CTRA 모델 기반 (위치 변화로 속도 추정 버전)
// // ===========================================================================================

// EKFTracker::EKFTracker(int id, float init_x, float init_y, float init_v, const Eigen::MatrixXd& shared_Q, const Eigen::MatrixXd& shared_R) {
//     this->id = id;
//     this->px = init_x;
//     this->py = init_y;
//     this->theta = 0.0; 
//     this->v = init_v;  
//     this->a = 0.0;
//     this->omega = 0.0;

//     this->miss_count = 0;
//     this->real_cluster_count = 1;

//     this->x = Eigen::VectorXd::Zero(6);
//     this->x << px, py, theta, v, a, omega;

//     // P 초기값: 속도(v), 가속도(a), 각속도(omega)에 높은 불확실성을 주어 위치 변화를 빠르게 흡수
//     this->P = Eigen::MatrixXd::Identity(6, 6) *1.0;
//     this->P(0,0) = 1.0; 
//     this->P(1,1) = 1.0; 
//     this->P(2,2) = 0.1;
//     this->P(3,3) = 100.0;  // 속도 초기 불확실성
//     this->P(4,4) = 100.0;  // 가속도 초기 불확실성
//     this->P(5,5) = 100.0;  // 각속도 초기 불확실성
    
//     this->Q = shared_Q;
//     this->R = shared_R;
// }

// void EKFTracker::predict(double dt) {

//         // ── [추가] Kinematic Model: heading 방향 속도만 유지 ──────────────
//     // vx, vy 대신 스칼라 v와 theta를 쓰는 구조이므로
//     // theta 방향 성분만 남기고 측면 속도 누적 방지
//     double cur_v_signed = x(3);  // 현재 속도 (부호 있음)
//     // theta 방향으로만 v를 유지 (측면 드리프트 제거)
//     // v는 스칼라이므로 크기만 clamp
//     // 저속에서 omega 감쇠 (yaw drift 방지)
//     if (std::abs(cur_v_signed) < 0.5) {
//         x(5) *= 0.5;  // omega 감쇠
//     }
//     // ──────────────────────────────────────────────────────────────────

//     Eigen::MatrixXd F = calculateJacobianF(dt);

//     double cur_px = x(0);
//     double cur_py = x(1);
//     double cur_theta = x(2);
//     double cur_v = x(3);
//     double cur_a = x(4);
//     double cur_omega = x(5);

//     Eigen::VectorXd x_next = Eigen::VectorXd::Zero(6);

//     // CTRA 비선형 예측 모델
//     if (std::abs(cur_omega) < 0.0001) { 
//         double dist = cur_v * dt + 0.5 * cur_a * dt * dt;
//         x_next(0) = cur_px + dist * std::cos(cur_theta);
//         x_next(1) = cur_py + dist * std::sin(cur_theta);
//     } 
//     else {
//         double v_final = cur_v + cur_a * dt;
//         double theta_final = cur_theta + cur_omega * dt;
//         double o2 = cur_omega * cur_omega;

//         x_next(0) = cur_px + (v_final * std::sin(theta_final) - cur_v * std::sin(cur_theta)) / cur_omega 
//                     + (cur_a * (std::cos(theta_final) - std::cos(cur_theta))) / o2;
//         x_next(1) = cur_py - (v_final * std::cos(theta_final) - cur_v * std::cos(cur_theta)) / cur_omega 
//                     + (cur_a * (std::sin(theta_final) - std::sin(cur_theta))) / o2;
//     }
//     x_next(2) = cur_theta + cur_omega * dt;
//     x_next(3) = cur_v + cur_a * dt;
//     x_next(4) = cur_a;
//     x_next(5) = cur_omega;

//     // 각도 정규화
//     while (x_next(2) >  M_PI) x_next(2) -= 2.0 * M_PI;
//     while (x_next(2) < -M_PI) x_next(2) += 2.0 * M_PI;

//     x = x_next;
//     px = x(0); py = x(1); theta = x(2); v = x(3); a = x(4); omega = x(5);

//     P = F * P * F.transpose() + Q;
    
//     // P 행렬의 nan/inf 검사
//     for (int i = 0; i < 6; ++i) {
//         for (int j = 0; j < 6; ++j) {
//             if (!std::isfinite(P(i, j))) {
//                 std::cerr << "[WARN] P(" << i << "," << j << ") is not finite: " << P(i,j) << std::endl;
//                 P = Eigen::MatrixXd::Identity(6, 6);
//                 break;
//             }
//         }
//     }
// }


// Eigen::MatrixXd EKFTracker::calculateJacobianF(double dt) {
//     Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
//     double th = theta;
//     double vel = v;
//     double acc = a;
//     double yr = omega;

//     if (std::abs(yr) < 0.0001) {
//         F(0, 2) = -(vel * dt + 0.5 * acc * dt * dt) * std::sin(th);
//         F(0, 3) = dt * std::cos(th);
//         F(0, 4) = 0.5 * dt * dt * std::cos(th); // 가속도가 x에 주는 영향
//         F(1, 2) = (vel * dt + 0.5 * acc * dt * dt) * std::cos(th);
//         F(1, 3) = dt * std::sin(th);
//         F(1, 4) = 0.5 * dt * dt * std::sin(th); // 가속도가 y에 주는 영향
//     } else {
//         double th_n = th + yr * dt;
//         double v_n = vel + acc * dt;
//         double yr2 = yr * yr;
//         double yr3 = yr2 * yr;

//         F(0, 2) = (v_n * std::cos(th_n) - vel * std::cos(th)) / yr + (acc * (std::sin(th) - std::sin(th_n))) / yr2;
//         F(1, 2) = (v_n * std::sin(th_n) - vel * std::sin(th)) / yr + (acc * (std::cos(th_n) - std::cos(th))) / yr2;
//         F(0, 3) = (std::sin(th_n) - std::sin(th)) / yr;
//         F(1, 3) = (-std::cos(th_n) + std::cos(th)) / yr;
//         F(0, 4) = (dt * yr * std::sin(th_n) + std::cos(th_n) - std::cos(th)) / yr2;
//         F(1, 4) = (-dt * yr * std::cos(th_n) + std::sin(th_n) - std::sin(th)) / yr2;
//         F(0, 5) = ( (vel*dt*yr + acc*dt)*std::cos(th_n) - (v_n*std::sin(th_n) - vel*std::sin(th)) ) / yr2 
//                   - ( 2*acc*(std::cos(th_n) - std::cos(th)) + acc*dt*yr*std::sin(th_n) ) / yr3;
//         F(1, 5) = ( (vel*dt*yr + acc*dt)*std::sin(th_n) + (v_n*std::cos(th_n) - vel*std::cos(th)) ) / yr2 
//                   - ( 2*acc*(std::sin(th_n) - std::sin(th)) - acc*dt*yr*std::cos(th_n) ) / yr3;
//     }

//     F(2, 5) = dt;     // theta는 omega에 의해 변함
//     F(3, 4) = dt;     // 가속도가 속도에 미치는 영향
    
//     return F;
// }

// void EKFTracker::update(double measured_x, double measured_y, double measured_yaw) {
//     double dx = measured_x - x(0);
//     double dy = measured_y - x(1);
//     double dist = std::sqrt(dx*dx + dy*dy);

//     // ── 180° ambiguity 보정 ──────────────────────────────
//     // L-shape fitting은 앞/뒤 구분 못하므로 EKF 트랙 yaw와 비교해서 보정
//     double yaw_diff = measured_yaw - x(2);
//     while (yaw_diff >  M_PI) yaw_diff -= 2.0 * M_PI;
//     while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;

//     if (std::abs(yaw_diff) > M_PI / 2.0) {
//         measured_yaw += M_PI;  // 180° 반전
//         while (measured_yaw >  M_PI) measured_yaw -= 2.0 * M_PI;
//         while (measured_yaw < -M_PI) measured_yaw += 2.0 * M_PI;
//     }

//     // ── 이동방향으로 yaw 초기 수렴 유도 (주행 중일 때) ──────
//     // 트랙 초기 수렴이 느릴 수 있으므로 충분히 움직였으면 이동방향 yaw 우선
//     if (dist > 0.3 && real_cluster_count < 5) {
//         measured_yaw = std::atan2(dy, dx);
//     }

//     // ── 3D update: z = [x, y, yaw] ──────────────────────
//     Eigen::VectorXd z(3);
//     z << measured_x, measured_y, measured_yaw;

//     Eigen::VectorXd h_x(3);
//     h_x << x(0), x(1), x(2);

//     Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
//     H(0, 0) = 1.0;
//     H(1, 1) = 1.0;
//     H(2, 2) = 1.0;

//     Eigen::MatrixXd R_ext = Eigen::MatrixXd::Zero(3, 3);
//     R_ext(0, 0) = R(0, 0);
//     R_ext(1, 1) = R(1, 1);
//     R_ext(2, 2) = 0.1;  // yaw 측정 노이즈 (튜닝 가능)

//     Eigen::MatrixXd S = H * P * H.transpose() + R_ext;
//     S += Eigen::MatrixXd::Identity(3, 3) * 1e-6;
//     Eigen::MatrixXd K = P * H.transpose() * S.inverse();

//     Eigen::VectorXd innov = z - h_x;
//     // innovation yaw 정규화
//     while (innov(2) >  M_PI) innov(2) -= 2.0 * M_PI;
//     while (innov(2) < -M_PI) innov(2) += 2.0 * M_PI;

//     x = x + K * innov;

//     Eigen::MatrixXd I_mat = Eigen::MatrixXd::Identity(6, 6);
//     P = (I_mat - K * H) * P;

//     // ── 정지 상태 처리 ────────────────────────────────────
//     if (dist < 0.05) {
//         x(3) = 0.0;  // v
//         x(5) = 0.0;  // omega
//         P(2, 2) = std::min(P(2, 2), 0.01);  // yaw 공분산 고정
//     }

//     // ── 멤버변수 동기화 ───────────────────────────────────
//     px = x(0); py = x(1); theta = x(2); v = x(3); a = x(4); omega = x(5);

//     while (x(2) >  M_PI) x(2) -= 2.0 * M_PI;
//     while (x(2) < -M_PI) x(2) += 2.0 * M_PI;
//     theta = x(2);
// }


// Eigen::MatrixXd EKFTracker::calculateJacobianH() {
//     Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 6);
//     H(0, 0) = 1.0; 
//     H(1, 1) = 1.0; 
//     return H;
// }

// double EKFTracker::getMahalanobisDistance(double measured_x, double measured_y) {
//     Eigen::VectorXd z(2);
//     z << measured_x, measured_y;
//     Eigen::VectorXd h_x(2);
//     h_x << x(0), x(1);

//     Eigen::MatrixXd H = calculateJacobianH();
//     Eigen::MatrixXd S = H * P * H.transpose() + R;
    
//     double regularization = 1e-6;
//     S = S + Eigen::MatrixXd::Identity(2, 2) * regularization;

//     Eigen::LLT<Eigen::MatrixXd> llt(S);
//     if (llt.info() != Eigen::Success) {
//         return 9999.0;  // 매칭 불가 처리
//     }

    
//     Eigen::VectorXd y = z - h_x;
    
//     double val = y.transpose() * S.inverse() * y;

//     // 음수/nan/inf 방어
//     if (!std::isfinite(val) || val < 0.0) {
//         return 9999.0;
//     }

//     return std::sqrt(val);
// }

// // ===========================================================================================
// // MultiObjectTracker
// // ===========================================================================================

// // ── [추가] 속도/가속도 상한값 정의 ──────────────────────────────────
// static constexpr double MAX_TRACK_VEL = 50.0;   // 50 m/s = 180 km/h
// static constexpr double MAX_TRACK_ACC = 20.0;   // 20 m/s²
// // ────────────────────────────────────────────────────────────────────


// MultiObjectTracker::MultiObjectTracker() : next_id(0), last_timestamp(0.0) {
//     // Q: 예측 모델의 노이즈 설정 (낮을수록 모델을 신뢰)
//     this->Q = Eigen::MatrixXd::Identity(6, 6);
//     this->Q(0,0) = 0.05; this->Q(1,1) = 0.05; // 위치 예측 오차
//     this->Q(2,2) = 0.01; // 헤딩 예측 오차
//     this->Q(3,3) = 0.01;  // 제약 강하면 실제 속도까지 수렴 느려짐 0.1 -> 0.05 (정지했을 때 속도벡터 줄어드는 현상 완화)
//     this->Q(4,4) = 1.0;  // 가속도 변화 허용 (속도 급격한 변화 방지)
//     this->Q(5,5) = 0.01;  // 각속도 변화 허용 1.0 -> 0.01 (회전이 많은 상황에서 트랙 유지 개선)

//     this->R = Eigen::MatrixXd::Identity(2, 2);
//     this->R << 0.001, 0, 
//                0, 0.001; // 0.05 -> 속도 오차 심함
// }

// void MultiObjectTracker::updateTracks(Lidar& st_Lidar, double current_time) {
//     st_Lidar.vec_kalman_clusters.clear();

//     // 1. dt 계산
//     if (last_timestamp <= 0.0) dt = 0.1;
//     else dt = std::max(0.001, current_time - last_timestamp);

//     // 2. Prediction (안전하게 인덱스 접근)
//     for (size_t i = 0; i < vec_EKFtracks.size(); ++i) {
//         vec_EKFtracks[i].predict(dt);
//     }

//     // 3. Association
//     int num_tracks = vec_EKFtracks.size();
//     int num_clusters = st_Lidar.vec_clusters.size();

//     std::cout << "[DEBUG] num_tracks: " << num_tracks 
//             << " num_clusters: " << num_clusters << std::endl;

    
//     std::vector<bool> track_updated(num_tracks, false);
//     std::vector<bool> cluster_matched(num_clusters, false);

//     if (num_tracks > 0 && num_clusters > 0) {
//         std::vector<std::vector<double>> cost_matrix(num_tracks, std::vector<double>(num_clusters));
//         for (int i = 0; i < num_tracks; ++i) {
//             for (int j = 0; j < num_clusters; ++j) {

//                 // -----------------추가 ------------------------
//                 // miss_count 일정 이상이면 cost matrix에서 제외
//                 if (vec_EKFtracks[i].miss_count > 3) { // 3회 이상 놓친 트랙은 매칭에서 제외
//                     cost_matrix[i][j] = 9999.0; // 매칭 불가 처리
//                     continue;
//                 }
//                 else
//                 {
//                     cost_matrix[i][j] = vec_EKFtracks[i].getMahalanobisDistance(
//                     st_Lidar.vec_clusters[j].centroid_x, 
//                     st_Lidar.vec_clusters[j].centroid_y);
//                 }                
//                 // --------------------------------------------
//             }
//         }

//         std::vector<int> assignment;
//         HungarianAlgorithm hungarian_solver;
//         hungarian_solver.Solve(cost_matrix, assignment);

//         // assignment 벡터 크기 검증
//         if (assignment.size() != num_tracks) {
//             std::cerr << "[ERROR] Assignment size mismatch: " << assignment.size() << " vs " << num_tracks << std::endl;
//             assignment.clear();
//             assignment.resize(num_tracks, -1);
//         }

//         for (int i = 0; i < num_tracks; ++i) {
//             int matched_idx = assignment[i];
//             if (matched_idx >= 0 && matched_idx < num_clusters) {
//                 if (cost_matrix[i][matched_idx] < 100.0) { 
//                     vec_EKFtracks[i].update(
//                         st_Lidar.vec_clusters[matched_idx].centroid_x,
//                         st_Lidar.vec_clusters[matched_idx].centroid_y,
//                         st_Lidar.vec_clusters[matched_idx].heading_theta);

//                     // -------------- 추가 --------------
//                     // OBB 클러스터 크기 정보 트랙에 같이 저장해서 업데이트
//                     vec_EKFtracks[i].last_length = st_Lidar.vec_clusters[matched_idx].length;
//                     vec_EKFtracks[i].last_width = st_Lidar.vec_clusters[matched_idx].width;
//                     // ----------------------------------

//                     vec_EKFtracks[i].real_cluster_count++;
//                     vec_EKFtracks[i].miss_count = 0;
//                     cluster_matched[matched_idx] = true;
//                     track_updated[i] = true;
//                 }
//             }
//         }
//     }

//     // 4. Track Management (Miss Count)
//     // for (int i = 0; i < num_tracks; ++i) {
//     //     if (!track_updated[i]) vec_EKFtracks[i].miss_count++;
//     // }

//     //     // ↓ 여기에 MISS 로그 추가
//     // for (int i = 0; i < num_tracks; ++i) {
//     //     if (!track_updated[i]) {
//     //         std::cout << "[MISS] ID: " << vec_EKFtracks[i].id 
//     //                   << " miss_count: " << vec_EKFtracks[i].miss_count << std::endl;
//     //     }
//     // }

//     // // 5. New Track Creation (반복문 밖에서 수행하여 Segfault 방지)
//     // for (int j = 0; j < num_clusters; ++j) {
//     //     if (!cluster_matched[j]) {
//     //         std::cout << "[NEW TRACK] id: " << next_id
//     //                 << " x: " << st_Lidar.vec_clusters[j].centroid_x
//     //                 << " y: " << st_Lidar.vec_clusters[j].centroid_y << std::endl;

//     //         vec_EKFtracks.emplace_back(next_id++, 
//     //             st_Lidar.vec_clusters[j].centroid_x, 
//     //             st_Lidar.vec_clusters[j].centroid_y, 
//     //             0.0, this->Q, this->R);
//     //     }
//     // }
//     for (int i = 0; i < num_tracks; ++i) {
//         if (!track_updated[i]) {
//             vec_EKFtracks[i].miss_count++;
//             std::cout << "[MISS] ID: " << vec_EKFtracks[i].id 
//                       << " miss_count: " << vec_EKFtracks[i].miss_count << std::endl;
//         }

//         // ── [추가] 속도/가속도 폭발 감지 후 상태 리셋 ──────────────
//         double vel = std::abs(vec_EKFtracks[i].x(3));
//         double acc = std::abs(vec_EKFtracks[i].x(4));

//         if (vel > MAX_TRACK_VEL || acc > MAX_TRACK_ACC) {
//             std::cerr << "[WARN] Track ID: " << vec_EKFtracks[i].id
//                       << " velocity/accel overflow. vel=" << vel
//                       << " acc=" << acc << " → reset state" << std::endl;

//             vec_EKFtracks[i].x(3) = 0.0;   // v 리셋
//             vec_EKFtracks[i].x(4) = 0.0;   // a 리셋
//             vec_EKFtracks[i].x(5) = 0.0;   // omega 리셋

//             // P도 함께 리셋 (다시 빠르게 수렴하도록)
//             vec_EKFtracks[i].P(3,3) = 100.0;
//             vec_EKFtracks[i].P(4,4) = 100.0;
//             vec_EKFtracks[i].P(5,5) = 100.0;
//         }
//         // ────────────────────────────────────────────────────────────
//     }
//     // 5. New Track Creation
//     for (int j = 0; j < num_clusters; ++j) {
//         if (!cluster_matched[j]) {
//             std::cout << "[NEW TRACK] id: " << next_id
//                       << " x: " << st_Lidar.vec_clusters[j].centroid_x
//                       << " y: " << st_Lidar.vec_clusters[j].centroid_y << std::endl;

//             vec_EKFtracks.emplace_back(next_id++, 
//                 st_Lidar.vec_clusters[j].centroid_x, 
//                 st_Lidar.vec_clusters[j].centroid_y, 
//                 0.0, this->Q, this->R);
//         }
//     }

//     // 6. Delete Old Tracks
//     vec_EKFtracks.erase(std::remove_if(vec_EKFtracks.begin(), vec_EKFtracks.end(),
//         [](const EKFTracker& t){ return t.miss_count > 10; }), // 트랙 삭제 임계값 5 -> 10
//         vec_EKFtracks.end());

//     // 7. Data Packing
//     for (const auto& t : vec_EKFtracks) {
//         std::cout << "[TRACK] ID: " << t.id 
//                 << " miss: " << t.miss_count 
//                 << " count: " << t.real_cluster_count << std::endl;

//         if (t.miss_count > 2 || t.real_cluster_count < 3) continue;
//         // 트랙 놓쳤을 때도 2회까지는 허용, 실제 클러스터와 3회 이상 매칭된 트랙만 시각화

//         double velocity_kmh = t.x(3) * 3.6;
//         if (std::abs(velocity_kmh) > 1.0) { 
//             std::cout << ">>> ID: " << t.id << " | Speed: " << velocity_kmh << " km/h" << std::endl;
//         }

//         KalmanDetection res;
//         res.id = t.id;
//         res.x = t.x(0); res.y = t.x(1); res.yaw = t.x(2);
//         res.v = t.x(3); res.a = t.x(4);

//         std::cout << "[EKF] ID: " << t.id 
//           << " | theta: " << t.x(2) 
//           << " | omega: " << t.x(5)
//           << " | v: " << t.x(3) << std::endl;


//         res.is_confirmed = true;
//         res.yaw_x = std::cos(t.x(2)); 
//         res.yaw_y = std::sin(t.x(2));

//         res.length = t.last_length;  // ← 추가
//         res.width  = t.last_width;   // ← 추가

//         st_Lidar.vec_kalman_clusters.push_back(res);
//         }
    
//     last_timestamp = current_time;
// }

