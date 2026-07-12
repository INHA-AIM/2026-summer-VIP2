#ifndef DATA_LOGGER_HPP
#define DATA_LOGGER_HPP

#include <string>

// ========================================
// 실험 데이터 로거
// ========================================

// 초기화: 파일 경로에 새 csv 파일 생성 + 헤더 작성
// run_id는 자동 증가 (같은 폴더 내 기존 파일 수 기반)
void initDataLogger(const std::string& save_dir);

// 매 timestep 호출: 현재 전역 변수에서 데이터 읽어서 csv에 한 줄 추가
void logData();

// 종료 시 호출: 파일 닫기
void closeDataLogger();

#endif // DATA_LOGGER_HPP