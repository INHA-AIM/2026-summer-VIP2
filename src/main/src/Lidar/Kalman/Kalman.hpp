// #ifndef KALMAN_HPP
// #define KALMAN_HPP

// #include "../Global/Global.hpp"
// #include "../Hungarian/Hungarian.hpp"
// #include <Eigen/Dense>
// #include <vector>

// // ===========================================================================================
// // EKFTracker - CTRA 모델 구현 (Constant Turn Rate and Acceleration)
// // ===========================================================================================
// class EKFTracker {
// public:
//     int id;
//     int miss_count;
//     int real_cluster_count;
    
//     // 상태 변수: 위치(px, py), 헤딩(theta), 속도(v), 가속도(a), 각속도(omega)
//     double px, py, theta, v, a, omega;
    
//     float last_length = 2.0f;  // ← 추가 (기본값: 차량 길이 추정값)
//     float last_width  = 1.0f;  // ← 추가 (기본값: 차량 폭 추정값)

//     Eigen::VectorXd x; // 상태 벡터 [6x1]: px, py, theta, v, a, omega
//     Eigen::MatrixXd P; // 오차 공분산 [6x6]
//     Eigen::MatrixXd Q; // 프로세스 노이즈 [6x6]
//     Eigen::MatrixXd R; // 측정 노이즈 [3x3] (x, y, theta만 측정하므로)

//     // =================== 0. 초기화 ======================
//     // init_v: 초기 속도 (Lidar가 주지 않으므로 보통 0.0으로 시작)
//     EKFTracker(int id, float init_x, float init_y, float init_v, const Eigen::MatrixXd& shared_Q, const Eigen::MatrixXd& shared_R);

//     // =================== 마할라노비스 거리 ======================
//     // Lidar 클러스터는 속도값이 없으므로 x, y, theta만 사용하여 거리를 계산합니다.
//     double getMahalanobisDistance(double measured_x, double measured_y);

//     // =============== 1. 예측 단계 (Prediction) ===============
//     // 식 (40): 시스템 모델을 통해 다음 시점의 상태를 통계적으로 예측
//     void predict(double dt);
    
//     // 1. 상태 x 예측 : 비선형 CTRA 모델을 적용해 다음 위치/속도 계산
//     // 2. 자코비안 F 계산 : 비선형 모션 모델을 선형화하여 오차 전파 준비
//     // 3. 오차 공분산 P 예측 : P = F*P*F' + Q (시간이 흐름에 따라 불확실성 증가)

//     // =============== 2. 보정 단계 (Update/Correction) ===============
//     // 식 (41): 실제 센서 측정값을 반영하여 예측값의 오차를 수정
//     void update(double measured_x, double measured_y);

//     // 1. 센서 측정값 벡터 구성 (z_t) : [measured_x, measured_y] (2차원)
//     // 2. 자코비안 H 계산 : 측정 방식(x, y 추출)에 따른 선형화 기울기 [2x6]
//     // 3. 칼만 게인 K 계산 : 예측값(모델)과 측정값(센서) 중 무엇을 더 믿을지 비중 결정
//     // 4. 상태 업데이트 : K를 이용해 위치 변화량을 속도(v)와 가속도(a)로 전이시켜 수정
//     // 5. 오차 공분산 업데이트 : 측정이 완료되었으므로 해당 객체의 불확실성(P) 감소

// private:
//     // 자코비안 F_t 계산 (상태 전이 행렬의 선형화)
//     Eigen::MatrixXd calculateJacobianF(double dt);
    
//     // 자코비안 H_t 계산 (측정 행렬의 선형화 - 2x6 행렬 반환)
//     Eigen::MatrixXd calculateJacobianH();
// };

// // ===========================================================================================
// // MultiObjectTracker - 여러 객체의 생명 주기 및 데이터 연관 관리
// // ===========================================================================================
// class MultiObjectTracker {
// public:
//     MultiObjectTracker();
//     std::vector<EKFTracker> vec_EKFtracks;

//     // 메인 업데이트 루프: Lidar 클러스터 데이터를 받아 트래킹 수행
//     void updateTracks(Lidar& st_Lidar, double current_time);
    
// private:
//     int next_id;
//     double last_timestamp;
//     double dt;
    
//     // 모든 트랙이 공유하는 노이즈 설정 (가속도 추정 감도 조절)
//     Eigen::MatrixXd Q; // 프로세스 노이즈: 모델이 얼마나 변할 수 있는지 (가속도 항 중요)
//     Eigen::MatrixXd R; // 측정 노이즈: 센서가 얼마나 정확한지 (x, y, theta 정밀도)
// };

// #endif