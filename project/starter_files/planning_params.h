// VELOCITY_PROFILE_GENERATOR.CPP → 차량 속도, 가속도, 저크(승차감) 관련 값들이 사용됨.
// BEHAVIOR_PLANNER_FSM.CPP → 차량의 경로 탐색 거리, 정지 관련 변수 사용됨.
// MOTION_PLANNER.CPP → 경로 최적화 및 기동 수행 시간 관련 값들이 사용됨.
// COST_FUNCTIONS.CPP → 원 충돌 검사 및 비용 평가 값들 사용됨.




/*
주요 로직
입방 나선 생성: motion_planner.cpp 파일에서 구현된 입방 나선을 사용하여 목표를 따라가며 경로를 생성.
충돌 검사: cost_functions.cpp에서 원 기반 충돌 검사 알고리즘을 구현.
FSM 로직: behavior_planner_FSM.cpp에서 상태 전환과 로직을 설계.


파일: planning_params.h
해야 할 TODO:

경로(목표) 수 설정: P_NUM_PATHS 값 선택.
나선 내 지점 수 설정: P_NUM_POINTS_IN_SPIRAL 값 선택.

도움말:
적절한 P_NUM_PATHS 값을 설정하여 계산 부하를 줄이셈.
나선 지점 수를 조정해 시뮬레이션 성능을 최적.

*/



/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file planning_param.h
 **/

#pragma once

#include <array>
// ---------------------------------------------------------
//                     Planning Constants
// ---------------------------------------------------------

// Planning Constants
#define P_NUM_PATHS 1                  // TODO - Num of paths (goals), 차량이 한 번의 경로 생성 시 몇 개의 후보 경로를 생성할지 결정 --->MOTION_PLANNER.CPP
                                       //값이 크면: 다양한 선택지가 많아 최적 경로를 찾을 가능성이 높아짐
                                       //값이 작으면: 선택지가 적어짐 (보통 최소 3~5개를 사용함)
                                       //일반적으로 3~7 정도가 적절한 값 (좁은 길에서는 3, 넓은 길에서는 5~7)
                                       // 값이 작으면 차량이 차선 변경 등의 유연한 움직임을 하기 어려울 수 있음

#define P_LOOKAHEAD_MIN 8.0            // m  // 최소 전방 탐색 거리 (m) --> BEHAVIOR_PLANNER_FSM.CPP

#define P_LOOKAHEAD_MAX 20.0           // m  // 최대 전방 탐색 거리 (m) --> BEHAVIOR_PLANNER_FSM.CPP

#define P_LOOKAHEAD_TIME 1.5           // s  // 전방 탐색 시간 (s)  -->BEHAVIOR_PLANNER_FSM.CPP

#define P_GOAL_OFFSET 1.0              // m // 목표 위치 오프셋 (m)  -->MOTION_PLANNER.CPP


#define P_ERR_TOLERANCE 0.1            // m  // 허용 오차 (m) -->MOTION_PLANNER.CPP

#define P_TIME_GAP 1.0                 // s // 차량 간 최소 시간 간격 (s)  -->MOTION_PLANNER.CPP

#define P_MAX_ACCEL 1.5                // m/s^2 // 최대 가속도 (m/s^2)  -->VELOCITY_PROFILE_GENERATOR.CPP


#define P_SLOW_SPEED 1.0               // m/s   // 저속 주행 기준 속도 (m/s)  -->VELOCITY_PROFILE_GENERATOR.CPP

#define P_SPEED_LIMIT 3.0              // m/s    // 속도 제한 (m/s) -->BEHAVIOR_PLANNER_FSM.CPP, VELOCITY_PROFILE_GENERATOR.CPP

// ---------------------------------------------------------
//                     Stop Line Settings
// ---------------------------------------------------------

#define P_STOP_LINE_BUFFER 0.5         // m  // 정지선과의 최소 거리 버퍼 (m) -->BEHAVIOR_PLANNER_FSM.CPP

#define P_STOP_THRESHOLD_SPEED 0.02    // m/s // 정지 기준 속도 (m/s) , 차량이 정지하는 기준 설정 --->VELOCITY_PROFILE_GENERATOR.CPP 

#define P_REQ_STOPPED_TIME 1.0         // secs // 정지 유지 시간 (s) , 차량이 정지하는 기준 설정 --->VELOCITY_PROFILE_GENERATOR.CPP 

// ---------------------------------------------------------
//                   Collision Avoidance
// ---------------------------------------------------------

#define P_LEAD_VEHICLE_LOOKAHEAD 20.0  // m // 선행 차량 탐지 범위 (m) -->BEHAVIOR_PLANNER_FSM.CPP

#define P_REACTION_TIME 0.25           // secs // 차량 반응 시간 (s) -->BEHAVIOR_PLANNER_FSM.CPP

#define P_NUM_POINTS_IN_SPIRAL 2       // TODO - Num of points in the spiral, 생성되는 나선형 궤적(spiral path) 에 몇 개의 점을 포함할지를 결정 --> COST_FUNCTIONS.CPP , MOTION_PLANNER.CPP
                                       //값이 크면: 더 부드러운 경로가 생성되지만 계산량이 증가, 값이 작으면: 경로가 부자연스러워질 수 있음
                                       //보통 10~20 정도가 적절한 값으로 사용됨! , 값을 너무 작게 설정하면 차량이 순간이동하듯 움직일 수 있음

#define P_STOP_THRESHOLD_DISTANCE \ //--> 차량이 정지 상태(STOPPED)로 전환할 때 사용하는 거리 기준 --->BEHAVIOR_PLANNER_FSM.CPP
  P_LOOKAHEAD_MIN / P_NUM_POINTS_IN_SPIRAL * 2  // m   // 정지 거리 기준 계산 (m)

constexpr std::array<float, 3> CIRCLE_OFFSETS = {-1.0, 1.0, 3.0};  // m // 차량 주변 원 위치 (m) -->OST_FUNCTIONS.CPP
 
constexpr std::array<float, 3> CIRCLE_RADII = {1.5, 1.5, 1.5};     // m // 원 반지름 (m) -->OST_FUNCTIONS.CPP


// ---------------------------------------------------------
//                   Simulation Settings
// ---------------------------------------------------------

constexpr double dt = 0.05; // 시뮬레이션 업데이트 주기 (50ms) -->모든 CPP 파일 (BEHAVIOR_PLANNER_FSM.CPP, MOTION_PLANNER.CPP, VELOCITY_PROFILE_GENERATOR.CPP, COST_FUNCTIONS.CPP)
// Standard deviation parameters for x, x_dot, x_double_dot
// to generate appropriate perturbed goals. EGO REF FRAME


// ---------------------------------------------------------
//                   Goal Perturbation
// ---------------------------------------------------------
//목표 샘플링을 위한 표준 편차 설정
constexpr std::array<float, 3> SIGMA_X = {4, 1.0, 2.0};  // X 방향 표준 편차 -->MOTION_PLANNER.CPP
// Standard devaition parameters for y, y_dot, y_double_dot 
// to generate appropriate perturbed goals. EGO REF FRAME
constexpr std::array<float, 3> SIGMA_Y = {0.5, 1.0, 0.5};  // Y 방향 표준 편차 -->MOTION_PLANNER.CPP
constexpr std::array<float, 3> SIGMA_YAW = {0.17, 1.0, 1.0}; // Yaw 방향 표준 편차 -->MOTION_PLANNER.CPP



// Standard deviation for time (as in the time
// taken to finish the maneuver)
// 주행 기동 수행 시간의 표준 편차 (초)
// 차량의 동작 예측 및 샘플링 과정에서 사용
constexpr double SIGMA_T = 0.5; //기동(maneuver) 수행 시간의 표준 편차를 정의하는 상수
                                //기동 시간 변동성: 차량이 특정한 동작(예: 정지, 차선 변경)을 수행할 때 예상 시간의 변동성을 표현
                                //샘플링 및 예측: 경로 계획에서 미래 동작을 예측하거나 시뮬레이션할 때, 기동 시간의 불확실성을 모델링
                                //안정성 및 승차감: 시간 변동성을 통해 차량의 움직임을 더 부드럽고 인간처럼 보이게 조정
                                //주로 Trajectory Planning 또는 Simulation Settings 섹션에서 사용
                                // -->MOTION_PLANNER.CPP, VELOCITY_PROFILE_GENERATOR.CPP

// ---------------------------------------------------------
//                     Comfort Settings
// ---------------------------------------------------------
// 모두 VELOCITY_PROFILE_GENERATOR.CPP 파일에 연결됨
// This would be the filtered jerk over one sec
constexpr double CONFORT_MAX_LAT_JERK = 0.9;               // m/s3 // 측면 저크 최대값 (m/s^3)
constexpr double CONFORT_MAX_LON_JERK = 1.5;               // m/s3   // 종방향 저크 최대값 (m/s^3)
constexpr double CONFORT_ACCUM_LON_JERK_IN_ONE_SEC = 3.0;  // m/s3 // 1초 동안 종방향 누적 저크
constexpr double CONFORT_ACCUM_LAT_JERK_IN_ONE_SEC = 2.0;  // m/s3  // 1초 동안 측면 누적 저크

constexpr double CONFORT_ACCUM_LON_ACC_IN_ONE_SEC = 1.0;  // m/s2 // 1초 동안 종방향 누적 가속도
constexpr double CONFORT_ACCUM_LAT_ACC_IN_ONE_SEC = 0.6;  // m/s2  // 1초 동안 측면 누적 가속도

constexpr double CONFORT_MAX_LON_ACCEL = 3.0;  // m/s2  // 최대 종방향 가속도 (m/s^2)
constexpr double CONFORT_MAX_LAT_ACCEL = 1.0;  // m/s2 / 최대 측면 가속도 (m/s^2)


// ---------------------------------------------------------
//                   Maneuver Time Settings
// ---------------------------------------------------------
constexpr double MIN_MANEUVER_TIME = dt * 10;  // min steps // 최소 기동 시간 (s) -->MOTION_PLANNER.CPP
constexpr double MAX_MANEUVER_TIME = dt * 75;  // max steps // 최대 기동 시간 (s) -->MOTION_PLANNER.CPP
