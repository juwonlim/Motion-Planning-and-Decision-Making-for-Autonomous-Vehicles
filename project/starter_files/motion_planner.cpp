/*

주요 로직
입방 나선 생성: motion_planner.cpp 파일에서 구현된 입방 나선을 사용하여 목표를 따라가며 경로를 생성.
충돌 검사: cost_functions.cpp에서 원 기반 충돌 검사 알고리즘을 구현.
FSM 로직: behavior_planner_FSM.cpp에서 상태 전환과 로직을 설계.


파일: motion_planner.cpp
해야 할 TODO:

목표 위치의 수직 방향 계산: 목표 yaw에 π/2를 추가하여 수직 방향 계산.
오프셋 목표 위치 계산: yaw 방향과 offset 값을 활용하여 새로운 목표 위치 설정.

도움말:
목표 yaw와 offset 계산을 정확히 구현
경로의 타당성을 확인하기 위해 충돌 검사를 활용


*/


/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

#include "motion_planner.h"

MotionPlanner::~MotionPlanner() {}

State MotionPlanner::get_goal_state_in_ego_frame(const State& ego_state,
                                                 const State& goal_state) {
  // Let's start by making a copy of the goal state (global reference frame)
  auto goal_state_ego_frame = goal_state;  // 글로벌 좌표계의 목표 상태 복사

  // Translate so the ego state is at the origin in the new frame.
  // This is done by subtracting the ego_state from the goal_ego_.
 
  // Ego 차량의 현재 위치를 원점(0,0,0)으로 변환
  goal_state_ego_frame.location.x -= ego_state.location.x;
  goal_state_ego_frame.location.y -= ego_state.location.y;
  goal_state_ego_frame.location.z -= ego_state.location.z;

  /* Rotate such that the ego state has zero heading/yaw in the new frame.
     We are rotating by -ego_state "yaw" to ensure the ego vehicle's
     current yaw corresponds to theta = 0 in the new local frame.

     Recall that the general rotation matrix around the Z axix is:
     [cos(theta) -sin(theta)
     sin(theta)  cos(theta)]
  */

  // 차량의 현재 방향을 기준으로 목표 지점을 회전 변환 (yaw=0 기준)
  auto theta_rad = -ego_state.rotation.yaw;
  auto cos_theta = std::cos(theta_rad);
  auto sin_theta = std::sin(theta_rad);

  goal_state_ego_frame.location.x =
      cos_theta * goal_state_ego_frame.location.x -
      sin_theta * goal_state_ego_frame.location.y;
  goal_state_ego_frame.location.y =
      sin_theta * goal_state_ego_frame.location.x +
      cos_theta * goal_state_ego_frame.location.y;

  // Compute the goal yaw in the local frame by subtracting off the
  // current ego yaw from the goal waypoint heading/yaw.
  goal_state_ego_frame.rotation.yaw += theta_rad; // 목표 yaw도 ego 기준으로 변환

  // Ego speed is the same in both coordenates
  // the Z coordinate does not get affected by the rotation.

  // Let's make sure the yaw is within [-180, 180] or [-pi, pi] so the optimizer
  // works.

  // yaw 값을 [-π, π] 범위로 유지
  goal_state_ego_frame.rotation.yaw = utils::keep_angle_range_rad(
      goal_state_ego_frame.rotation.yaw, -M_PI, M_PI);
  // if (goal_state_ego_frame.rotation.yaw < -M_PI) {
  //   goal_state_ego_frame.rotation.yaw += (2 * M_PI);
  // } else if (goal_state_ego_frame.rotation.yaw > M_PI) {
  //   goal_state_ego_frame.rotation.yaw -= (2 * M_PI);
  // }

  return goal_state_ego_frame;
} //이 부분은 자차(Ego) 기준 좌표계를 만들어 경로를 쉽게 계산하도록 변환하는 역할을 함





std::vector<State> MotionPlanner::generate_offset_goals_ego_frame(
    const State& ego_state, const State& goal_state) {
  // Let's transform the "main" goal (goal state) into ego reference frame
  auto goal_state_ego_frame =
      get_goal_state_in_ego_frame(ego_state, goal_state);

  return generate_offset_goals(goal_state_ego_frame);
}

std::vector<State> MotionPlanner::generate_offset_goals_global_frame(
    const State& goal_state) {
  return generate_offset_goals(goal_state);
}

std::vector<State> MotionPlanner::generate_offset_goals(
    const State& goal_state) {
  // Now we need to gernerate "_num_paths" goals offset from the center goal at
  // a distance "_goal_offset".
  std::vector<State> goals_offset;

  // the goals will be aligned on a perpendiclular line to the heading of the
  // main goal. To get a perpendicular angle, just add 90 (or PI/2) to the main
  // goal heading.

  // TODO-Perpendicular direction: ADD pi/2 to the goal yaw
  // (goal_state.rotation.yaw)
  //auto yaw = ;  // <- Fix This

    //yaw = goal_state.rotation.yaw + M_PI / 2를 통해 주 목표 지점의 방향에 수직인 방향을 계산
    auto yaw = goal_state.rotation.yaw + M_PI / 2;  // 주 목표 지점의 yaw에 π/2를 더함 -->// 목표 yaw에 π/2를 추가하여 수직 방향을 계산
                                                    //이 부분은 주 목표 지점(goal_state)의 방향(yaw)에 수직인 방향을 계산하는 부분
                                                    //주 목표 지점의 방향에 90도(π/2 라디안)를 더하면 수직 방향이 됨
                                                    //이 수직 방향은 오프셋 목표 지점을 생성할 때 사용

  // LOG(INFO) << "MAIN GOAL";
  // LOG(INFO) << "x: " << goal_state.location.x << " y: " <<
  // goal_state.location.y
  //          << " z: " << goal_state.location.z
  //          << " yaw (rad): " << goal_state.rotation.yaw;
  // LOG(INFO) << "OFFSET GOALS";
  // LOG(INFO) << "ALL offset yaw (rad): " << yaw;


// 오프셋 목표 지점 생성
  for (int i = 0; i < _num_paths; ++i) { // _num_paths: 생성할 오프셋 목표 지점의 개수
    auto goal_offset = goal_state;
    float offset = (i - (int)(_num_paths / 2)) * _goal_offset; //_goal_offset: 각 오프셋 목표 지점 사이의 거리
    // 오프셋 목표지점 생성 : offset은 (i - (int)(_num_paths / 2)) * _goal_offset로 계산
                           // std::cos(yaw)와 std::sin(yaw)를 사용하여 수직 방향으로의 이동량을 계산


    // LOG(INFO) << "Goal: " << i + 1;
    // LOG(INFO) << "(int)(_num_paths / 2): " << (int)(_num_paths / 2);
    // LOG(INFO) << "(i - (int)(_num_paths / 2)): " << (i - (int)(_num_paths /
    // 2)); LOG(INFO) << "_goal_offset: " << _goal_offset;

    // LOG(INFO) << "offset: " << offset;

    // TODO-offset goal location: calculate the x and y position of the offset
    // goals using "offset" (calculated above) and knowing that the goals should
    // lie on a perpendicular line to the direction (yaw) of the main goal. You
    // calculated this direction above (yaw_plus_90). HINT: use
    // std::cos(yaw_plus_90) and std::sin(yaw_plus_90)
    

    
    // goal_offset.location.x += ;  // <- Fix This
    // goal_offset.location.y += ;  // <- Fix This

    //이 부분은 주 목표 지점에서 수직 방향으로 오프셋을 적용하여 새로운 목표 지점의 좌표를 계산하는 부분
    //offset은 오프셋 거리이며, yaw_plus_90은 수직 방향의 각도
    //std::cos(yaw_plus_90)와 std::sin(yaw_plus_90)를 사용하여 수직 방향으로의 이동량을 계산
   // 오프셋 적용하여 목표 위치 변경
    goal_offset.location.x += offset * std::cos(yaw);  // x 좌표 업데이트
    goal_offset.location.y += offset * std::sin(yaw);  // y 좌표 업데이트
    // std::cos(yaw)와 std::sin(yaw)를 사용하여 수직 방향으로의 이동량을 계산
    //이 값을 주 목표 지점의 좌표에 더하여 오프셋 목표 지점을 생성


    // LOG(INFO) << "x: " << goal_offset.location.x
    //          << " y: " << goal_offset.location.y
    //          << " z: " << goal_offset.location.z
    //          << " yaw (rad): " << goal_offset.rotation.yaw;

//생성된 오프셋 목표 지점이 유효한지 확인한 후(valid_goal), 유효한 경우 목표 목록에 추가
 // 생성된 오프셋 목표가 유효한지 검사 후 추가   
    if (valid_goal(goal_state, goal_offset)) { //valid_goal: 생성된 오프셋 목표 지점이 주 목표 지점에서 너무 멀리 떨어져 있지 않은지 확인하는 함수
      goals_offset.push_back(goal_offset);
    } //목표 지점에 수직 방향(π/2) 오프셋을 적용하여 다양한 경로를 생성하는 역할
  }
  return goals_offset;
}

bool MotionPlanner::valid_goal(const State& main_goal,
                               const State& offset_goal) {
  auto max_offset = ((int)(_num_paths / 2) + 1) * _goal_offset;
  // LOG(INFO) << "max offset: " << max_offset;
  auto dist = utils::magnitude(main_goal.location - offset_goal.location);
  // LOG(INFO) << "distance from main goal to offset goal: " << dist;
  return dist < max_offset;
}

/*
📌 3️⃣ 나선형 경로(spirals) 생성 및 변환
*/
std::vector<int> MotionPlanner::get_best_spiral_idx(
    const std::vector<std::vector<PathPoint>>& spirals,
    const std::vector<State>& obstacles, const State& goal_state) {
  // LOG(INFO) << "Num Spirals: " << spirals.size();
  double best_cost = DBL_MAX;
  std::vector<int> collisions;
  int best_spiral_idx = -1;
  for (size_t i = 0; i < spirals.size(); ++i) {
    double cost = calculate_cost(spirals[i], obstacles, goal_state);

    if (cost < best_cost) {
      best_cost = cost;
      best_spiral_idx = i;
    }
    if (cost > DBL_MAX) {
      collisions.push_back(i);
    }
  }
  if (best_spiral_idx != -1) {
    collisions.push_back(best_spiral_idx);
    return collisions;
  }
  std::vector<int> noResults;
  return noResults;
}

std::vector<std::vector<PathPoint>>
MotionPlanner::transform_spirals_to_global_frame(
    const std::vector<std::vector<PathPoint>>& spirals,
    const State& ego_state) {
  std::vector<std::vector<PathPoint>> transformed_spirals;
  for (auto spiral : spirals) {
    std::vector<PathPoint> transformed_single_spiral;
    for (auto path_point : spiral) {
      PathPoint new_path_point;
      new_path_point.x = ego_state.location.x +
                         path_point.x * std::cos(ego_state.rotation.yaw) -
                         path_point.y * std::sin(ego_state.rotation.yaw);
      new_path_point.y = ego_state.location.y +
                         path_point.x * std::sin(ego_state.rotation.yaw) +
                         path_point.y * std::cos(ego_state.rotation.yaw);
      new_path_point.theta = path_point.theta + ego_state.rotation.yaw;

      transformed_single_spiral.emplace_back(new_path_point);
    }
    transformed_spirals.emplace_back(transformed_single_spiral);
  }
  return transformed_spirals;
}

std::vector<std::vector<PathPoint>> MotionPlanner::generate_spirals(
    const State& ego_state, const std::vector<State>& goals) {
  // Since we are on Ego Frame, the start point is always at 0, 0, 0,

  //Ego 차량의 현재 위치를 기준으로 시작점 설정
  PathPoint start;
  start.x = ego_state.location.x;
  start.y = ego_state.location.y;
  start.z = ego_state.location.z;
  start.theta = ego_state.rotation.yaw;
  start.kappa = 0.0;
  start.s = 0.0;
  start.dkappa = 0.0;
  start.ddkappa = 0.0;

  std::vector<std::vector<PathPoint>> spirals;
  for (auto goal : goals) {
    PathPoint end;
    end.x = goal.location.x;
    end.y = goal.location.y;
    end.z = goal.location.z;
    end.theta = goal.rotation.yaw;
    end.kappa = 0.0;
    end.s = std::sqrt((end.x * end.x) + (end.y * end.y));
    end.dkappa = 0.0;
    end.ddkappa = 0.0;


    // 나선형 경로 생성
    //목표 지점으로 향하는 다양한 나선형 경로를 생성하는 코드
    //경로가 유효한지 검토 후 선택하는 과정 포함
    
    if (_cubic_spiral.GenerateSpiral(start, end)) {
      std::vector<PathPoint>* spiral = new std::vector<PathPoint>;
      auto ok = _cubic_spiral.GetSampledSpiral(P_NUM_POINTS_IN_SPIRAL, spiral);

     // 생성된 나선이 유효한지 확인 후 저장
      if (ok && valid_spiral(*spiral, goal)) {
        // LOG(INFO) << "Spiral Valid ";
        spirals.push_back(*spiral);
      } else {
        // LOG(INFO) << "Spiral Invalid ";
      }
    } else {
      // LOG(INFO) << "Spiral Generation FAILED! ";
    }
  }
  return spirals;
}

bool MotionPlanner::valid_spiral(const std::vector<PathPoint>& spiral,
                                 const State& offset_goal) {
  auto n = spiral.size();
  auto delta_x = (offset_goal.location.x - spiral[n - 1].x);
  auto delta_y = (offset_goal.location.y - spiral[n - 1].y);
  auto dist = std::sqrt((delta_x * delta_x) + (delta_y * delta_y));
  // auto dist = utils::magnitude(spiral[spiral.size() - 1].location -
  //                              offset_goal.location);
  // LOG(INFO) << "Distance from Spiral end to offset_goal: " << dist;
  return (dist < 0.1);
}

float MotionPlanner::calculate_cost(const std::vector<PathPoint>& spiral,
                                    const std::vector<State>& obstacles,
                                    const State& goal) {
  // LOG(INFO) << "Starting spiral cost calc";
  // Initialize cost to 0.0
  float cost = 0.0;
  cost += cf::collision_circles_cost_spiral(spiral, obstacles);

  cost += cf::close_to_main_goal_cost_spiral(spiral, goal);

  // LOG(INFO) << "Path Cost: " << cost;
  return cost;
}
