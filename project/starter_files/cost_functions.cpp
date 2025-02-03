/*
주요 로직
입방 나선 생성: motion_planner.cpp 파일에서 구현된 입방 나선을 사용하여 목표를 따라가며 경로를 생성.
충돌 검사: cost_functions.cpp에서 원 기반 충돌 검사 알고리즘을 구현.
FSM 로직: behavior_planner_FSM.cpp에서 상태 전환과 로직을 설계.


파일: cost_functions.cpp
해야 할 TODO:

원 배치: CIRCLE_OFFSETS를 사용하여 원의 중심 위치 계산.
원과 장애물/행위자 간 거리 계산: 각 원의 중심과 장애물 사이의 거리 평가.
나선의 끝점과 주 목표 간 거리 계산: Spiral의 마지막 지점과 목표 지점의 거리 계산.

도움말:
CIRCLE_OFFSETS를 활용할 때, sin과 cos로 x, y 좌표를 계산.
나선 궤적의 끝점이 실제 목표와 가까운지 확인하는 것이 중요.

*/





/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file cost_functions.cpp
 **/

#include "cost_functions.h"

using namespace std;

namespace cost_functions {
// COST FUNCTIONS


// 차량의 궤적과 목표 지점 사이의 차이를 기반으로 비용을 계산하는 함수
double diff_cost(vector<double> coeff, double duration,
                 std::array<double, 3> goals, std::array<float, 3> sigma,
                 double cost_weight) {
  /*
  Penalizes trajectories whose coordinate(and derivatives)
  differ from the goal.
  */
  double cost = 0.0;

  // 궤적의 계수(coeff)와 시간(duration)을 기반으로 함수 값과 2차 도함수까지 계산
  vector<double> evals = evaluate_f_and_N_derivatives(coeff, duration, 2);
  //////////////cout << "26 - Evaluating f and N derivatives Done. Size:" <<
  /// evals.size() << endl;

// 계산된 함수 값과 목표 값 사이의 차이를 비용으로 누적
  for (size_t i = 0; i < evals.size(); i++) {
    double diff = fabs(evals[i] - goals[i]);  // 절대값 차이 계산
    cost += logistic(diff / sigma[i]); // 차이를 로지스틱 함수로 변환하여 비용에 추가
  }
  ////////////cout << "diff_coeff Cost Calculated " << endl;

  // 최종 비용에 가중치를 곱하여 반환
  return cost_weight * cost;
}

// 나선 궤적과 장애물 간의 충돌 여부를 판단하고 비용을 계산하는 함수
double collision_circles_cost_spiral(const std::vector<PathPoint>& spiral,
                                     const std::vector<State>& obstacles) {
  bool collision{false};  // 충돌 여부를 저장하는 플래그
  auto n_circles = CIRCLE_OFFSETS.size();  // 원의 개수 (CIRCLE_OFFSETS는 원의 중심 위치 오프셋)

// 나선 궤적의 각 웨이포인트를 순회하며 충돌 검사
  for (auto wp : spiral) {
    if (collision) { 
      // LOG(INFO) << " ***** COLLISION DETECTED *********" << std::endl;
       // 충돌이 발생한 경우 루프 종료
      break;
    }


    // 현재 웨이포인트의 위치와 방향 (yaw)을 가져옴
    double cur_x = wp.x;
    double cur_y = wp.y;
    double cur_yaw = wp.theta;  // This is already in rad.  // yaw는 라디안 단위


// 각 원의 중심 위치를 계산하고 장애물과의 충돌 여부를 검사
    for (size_t c = 0; c < n_circles && !collision; ++c) {
      // TODO-Circle placement: Where should the circles be at? The code below --(TO DO 부분 :원 배치 (TODO-Circle placement))
      // is NOT complete. HINT: use CIRCLE_OFFSETS[c], sine and cosine to
      // calculate x and y: cur_y + CIRCLE_OFFSETS[c] * std::sin/cos(cur_yaw)
      //auto circle_center_x = 0;  // <- Update 
      //auto circle_center_y = 0;  // <- Update 

      // 원을 차량의 진행 방향(cur_yaw)에 맞춰 배치
      //차량 진행 방향(yaw)을 고려하여 원을 배치
      //CIRCLE_OFFSETS[c] 값만큼 이동한 좌표가 원의 중심
       // 원의 중심 위치 계산 (차량의 진행 방향을 고려하여 CIRCLE_OFFSETS[c]만큼 이동)
       //차량 주변에 원(CIRCLE)을 배치하여 충돌검사 수행
       //차량의 위치(cur_x, cur_y)를 기준으로 CIRCLE_OFFSETS[c] 만큼 떨어진 위치에 원을 배치
      auto circle_center_x = cur_x + CIRCLE_OFFSETS[c] * std::cos(cur_yaw); //차량 진행 방향을 고려해 원의 x 좌표 계산
      auto circle_center_y = cur_y + CIRCLE_OFFSETS[c] * std::sin(cur_yaw); //차량 진행 방향을 고려해 원의 y 좌표 계산
                                                                            //cur_x, cur_y → 차량의 현재 위치
                                                                            //cur_yaw → 차량이 바라보는 방향 (yaw, radian 단위)
                                                                            //CIRCLE_OFFSETS[c] → 원을 배치할 거리(반지름 역할) --?	원을 차량의 진행 방향으로 떨어진 거리만큼 배치
                                                                            //cos(cur_yaw), sin(cur_yaw)을 이용해 yaw 방향으로 거리 만큼 떨어진 점을 계산
/* 
 차량 진행 방향 예시
📌 차량이 90도(yaw = 90° = π/2 rad) 방향을 바라보고 있을 때 :
cos(90°) = 0,   sin(90°) = 1
이 경우, 원의 중심은 x 좌표는 그대로, y 좌표만 이동
circle_center_x = cur_x + 0;
circle_center_y = cur_y + CIRCLE_OFFSETS[c];
즉, 차량이 위쪽(북쪽) 으로 가는 경우, 원은 차량 앞쪽(y축 방향) 에 배치됨.

📌 차량이 0도(yaw = 0 rad) 방향을 바라보고 있을 때:
cos(0°) = 1,   sin(0°) = 0
이 경우, 원의 중심은 x 좌표만 이동, y 좌표는 그대로
circle_center_x = cur_x + CIRCLE_OFFSETS[c];
circle_center_y = cur_y + 0;
즉, 차량이 오른쪽(동쪽) 으로 가는 경우, 원은 차량 오른쪽(x축 방향) 에 배치됨.

*/


      // 모든 장애물에 대해 충돌 검사
      for (auto obst : obstacles) {
        if (collision) {
          break;
        }

        // 장애물의 방향 (yaw)을 가져옴
        auto actor_yaw = obst.rotation.yaw;

         // 장애물의 원 중심 위치 계산
        for (size_t c2 = 0; c2 < n_circles && !collision; ++c2) {
          auto actor_center_x =
              obst.location.x + CIRCLE_OFFSETS[c2] * std::cos(actor_yaw);
          auto actor_center_y =
              obst.location.y + CIRCLE_OFFSETS[c2] * std::sin(actor_yaw);

          // TODO-Distance from circles to obstacles/actor: How do you calculate --(TO DO 부분 : 원과 장애물/행위자 간 거리 계산)
          //거리가 원 반지름보다 작으면 충돌 발생으로 간주.
          // the distance between the center of each circle and the
          // obstacle/actor
          //double dist = 0;  // <- Update
          // 유클리드 거리 계산 (장애물과 원의 중심 간 거리)
             
             // // 원과 장애물 간의 거리 계산 (유클리드 거리)
             double dist = std::sqrt(std::pow(circle_center_x - actor_center_x, 2) + std::pow(circle_center_y - actor_center_y, 2));
           //원 중심(circle_center_x, circle_center_y)과 장애물 중심(actor_center_x, actor_center_y) 사이의 유클리드 거리 계산
           //이 값이 CIRCLE_RADII[c] + CIRCLE_RADII[c2] 보다 작으면 충돌 발생


          collision = (dist < (CIRCLE_RADII[c] + CIRCLE_RADII[c2]));  // 거리가 두 원의 반지름 합보다 작으면 충돌로 판단
        }
      }
    }
  }
  return (collision) ? COLLISION : 0.0; // 충돌이 발생한 경우 COLLISION 비용 반환, 아니면 0 반환
}



// 나선 궤적의 마지막 지점과 주 목표 지점 사이의 거리를 기반으로 비용을 계산하는 함수
double close_to_main_goal_cost_spiral(const std::vector<PathPoint>& spiral,
                                      State main_goal) {
  // The last point on the spiral should be used to check how close we are to
  // the Main (center) goal. That way, spirals that end closer to the lane
  // center-line, and that are collision free, will be prefered.
  
  
  // 나선 궤적의 마지막 지점을 사용하여 주 목표와의 거리를 계산
  auto n = spiral.size();

  // TODO-distance between last point on spiral and main goal: How do we //TO DO 부분 :나선의 마지막 지점과 목표 지점 거리 계산
 //나선 경로(spiral)의 마지막 지점과 주 목표(main_goal) 간 거리 계산.
 //이 값이 작을수록 도로 중심선에 가까운 좋은 경로라고 판단됨.

  // calculate the distance between the last point on the spiral (spiral[n-1])
  // and the main goal (main_goal.location). Use spiral[n - 1].x, spiral[n -
  // 1].y and spiral[n - 1].z.
  // Use main_goal.location.x, main_goal.location.y and main_goal.location.z
  // Ex: main_goal.location.x - spiral[n - 1].x
  //auto delta_x = 0;  // <- Update
  //auto delta_y = 0;  // <- Update
  //auto delta_z = 0;  // <- Update

  // 나선의 마지막 지점과 목표 지점의 차이 계산
  // 나선의 마지막 지점과 주 목표 지점의 차이 계산
    auto delta_x = main_goal.location.x - spiral[n - 1].x;
    auto delta_y = main_goal.location.y - spiral[n - 1].y;
    auto delta_z = main_goal.location.z - spiral[n - 1].z;
  //나선(spiral)의 마지막 지점과 목표 지점(main_goal.location)의 거리 차이를 계산
  //유클리드 거리(dist = sqrt(delta_x² + delta_y² + delta_z²))를 이용해 목표와의 거리를 평가



  // 유클리드 거리 계산 (목표와의 거리)
  auto dist = std::sqrt((delta_x * delta_x) + (delta_y * delta_y) +
                        (delta_z * delta_z));

  auto cost = logistic(dist);   // 거리를 로지스틱 함수로 변환하여 비용 계산
  // LOG(INFO) << "distance to main goal: " << dist;
  // LOG(INFO) << "cost (log): " << cost;
  return cost;
}
}  // namespace cost_functions
