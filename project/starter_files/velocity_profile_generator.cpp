/*
주요 로직
입방 나선 생성: motion_planner.cpp 파일에서 구현된 입방 나선을 사용하여 목표를 따라가며 경로를 생성.
충돌 검사: cost_functions.cpp에서 원 기반 충돌 검사 알고리즘을 구현.
FSM 로직: behavior_planner_FSM.cpp에서 상태 전환과 로직을 설계.

파일: velocity_profile_generator.cpp
해야 할 TODO:

거리 계산: 등가속도 공식을 사용해 이동 거리 계산.
최종 속도 계산: 정지선까지의 최종 속도를 계산.

도움말:
거리 계산 시, 초기 속도와 최종 속도를 기반으로 등가속도를 사용해야 함.
정지선을 목표로 하여 부드러운 감속 궤적을 생성

*/





/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file velocity_trajectory_generator.cpp
 **/

#include "velocity_profile_generator.h"

#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <thread>

// 생성자
VelocityProfileGenerator::VelocityProfileGenerator() {}

// 소멸자
VelocityProfileGenerator::~VelocityProfileGenerator() {}


// 초기 설정 함수 (시간 간격, 최대 가속도, 저속 주행 속도 설정)
void VelocityProfileGenerator::setup(const double& time_gap,
                                     const double& a_max,
                                     const double& slow_speed) {
  _time_gap = time_gap; // 시간 간격 설정
  _a_max = a_max;   // 최대 가속도 설정
  _slow_speed = slow_speed;  // 저속 주행 속도 설정
};

/*
This class computes a velocity trajectory from a starting speed to a desired
speed. It works in unison with the Behavioral plannner  as it needs to build a
velocity profile for each of the states that the vehicle can be in.
In the "Follow_lane" state we need to either speed up or speed down to maintain
a speed target. In the "decel_to_stop" state we need to create a profile that
allows us to decelerate smoothly to a stop line.

The order of precedence for handling these cases is stop sign handling and then
nominal lane maintenance. In a real velocity planner you would need to handle
the coupling between these states, but for simplicity this project can be
implemented by isolating each case.

For all trajectories, the required acceleration is given by _a_max (confortable
accel).
Look at the structs.h for details on the types of manuevers/states that the
behavior planner can be in.
*/

/*
이 클래스는 시작 속도에서 원하는 속도까지의 속도 궤적을 계산합니다.
Behavioral Planner와 함께 작동하며, 차량이 있을 수 있는 각 상태에 대한 속도 프로파일을 생성해야 합니다.
"Follow_lane" 상태에서는 속도 목표를 유지하기 위해 가속하거나 감속해야 합니다.
"decel_to_stop" 상태에서는 정지선까지 부드럽게 감속할 수 있는 프로파일을 생성해야 합니다.

이러한 경우를 처리하기 위한 우선순위는 정지 신호 처리 후 일반 차선 유지입니다.
실제 속도 플래너에서는 이러한 상태 간의 결합을 처리해야 하지만, 이 프로젝트에서는 단순화를 위해 각 경우를 분리하여 구현할 수 있습니다.

모든 궤적에 대해 필요한 가속도는 _a_max (편안한 가속도)로 주어집니다.
Behavioral Planner가 있을 수 있는 상태/기동 유형에 대한 자세한 내용은 structs.h를 참조하세요.
*/

// 궤적 생성 함수
std::vector<TrajectoryPoint> VelocityProfileGenerator::generate_trajectory(
    const std::vector<PathPoint>& spiral, const double& desired_speed,
    const State& ego_state, const State& lead_car_state,
    const Maneuver& maneuver) const {
  // LOG(INFO) << "Lead car x: " << lead_car_state.location.x;

  std::vector<TrajectoryPoint> trajectory; // 궤적을 저장할 벡터
  double start_speed = utils::magnitude(ego_state.velocity); // 초기 속도 계산

  // LOG(INFO) << "Start Speed (m/s): " << start_speed;
  // LOG(INFO) << "Desired Speed (m/s): " << desired_speed;

  // Generate a trapezoidal trajectory to decelerate to stop.
  // DECEL_TO_STOP 상태인 경우, 정지까지 감속 궤적 생성
  if (maneuver == DECEL_TO_STOP) {
    // LOG(INFO) << "Generating velocity trajectory for DECEL_TO_STOP";
    trajectory = decelerate_trajectory(spiral, start_speed);
  }
  // If we need to follow the lead vehicle, make sure we decelerate to its speed
  // by the time we reach the time gap point.
   // FOLLOW_VEHICLE 상태인 경우, 선행 차량을 따라가는 궤적 생성
  else if (maneuver == FOLLOW_VEHICLE) {
    // LOG(INFO) << "Generating velocity trajectory for FOLLOW_VEHICLE";
    trajectory =
        follow_trajectory(spiral, start_speed, desired_speed, lead_car_state);
  }

  // Otherwise, compute the trajectory to reach our desired speed.
   // 일반 주행 상태인 경우, 원하는 속도로 주행하는 궤적 생성
  else {
    // LOG(INFO) << "Generating velocity trajectory for NOMINAL TRAVEL";
    trajectory = nominal_trajectory(spiral, start_speed, desired_speed);
  }
  // Interpolate between the zeroth state and the first state.
  // This prevents the controller from getting stuck at the zeroth state.

    // 궤적의 첫 번째와 두 번째 상태 사이를 보간하여 컨트롤러가 첫 번째 상태에서 멈추지 않도록 함
  if (trajectory.size() > 1) {
    TrajectoryPoint interpolated_state;
    interpolated_state.path_point.x =
        (trajectory[1].path_point.x - trajectory[0].path_point.x) * 0.1 +
        trajectory[0].path_point.x;
    interpolated_state.path_point.y =
        (trajectory[1].path_point.y - trajectory[0].path_point.y) * 0.1 +
        trajectory[0].path_point.y;
    interpolated_state.path_point.z =
        (trajectory[1].path_point.z - trajectory[0].path_point.z) * 0.1 +
        trajectory[0].path_point.z;
    interpolated_state.v =
        (trajectory[1].v - trajectory[0].v) * 0.1 + trajectory[0].v;
    trajectory[0] = interpolated_state;
  }

  return trajectory;
}

// Computes a velocity trajectory for deceleration to a full stop.
// 정지까지 감속 궤적 생성 함수
std::vector<TrajectoryPoint> VelocityProfileGenerator::decelerate_trajectory(
    const std::vector<PathPoint>& spiral, const double& start_speed) const {
  std::vector<TrajectoryPoint> trajectory; // 궤적을 저장할 벡터

  // Using d = (v_f^2 - v_i^2) / (2 * a)
  // 등가속도 공식을 사용하여 감속 거리 계산
  auto decel_distance = calc_distance(start_speed, _slow_speed, -_a_max);
  auto brake_distance = calc_distance(_slow_speed, 0, -_a_max);


  // 나선 경로의 총 길이 계산
  auto path_length{0.0};
  auto stop_index{spiral.size() - 1};
  for (size_t i = 0; i < stop_index; ++i) {
    path_length += utils::distance(spiral[i + 1], spiral[i]);
  }

  /* If the brake distance exceeds the length of the path, then we cannot
  perform a smooth deceleration and require a harder deceleration.  Build the path
  up in reverse to ensure we reach zero speed at the required time.
  */

   // 감속 거리가 경로 길이를 초과하는 경우, 더 강한 감속이 필요함
  if (brake_distance + decel_distance > path_length) {
    std::vector<double> speeds; // 속도 값을 저장할 벡터
    auto vf{0.0}; // 최종 속도 (정지 상태)


    // Let's add the last point, i.e at the stopping line we should have speed
    // 0.0.

     // 정지선에서의 속도를 0으로 설정
    auto it = speeds.begin();
    speeds.insert(it, 0.0);

    // Let's now go backwards until we get to the very beginning of the path
    // 경로의 끝에서 시작점까지 역순으로 속도 계산
    for (int i = stop_index - 1; i >= 0; --i) {
      auto dist = utils::distance(spiral[i + 1], spiral[i]);
      auto vi = calc_final_speed(vf, -_a_max, dist);
      if (vi > start_speed) {
        vi = start_speed;
      }
      // Let's add it
      // 계산된 속도를 벡터에 추가
      auto it = speeds.begin();
      speeds.insert(it, vi);
      vf = vi;
    }

    // At this point we have all the speeds. Now we need to create the
    // trajectory
    // 속도 값을 기반으로 궤적 생성
    double time_step{0.0};
    double time{0.0};
    for (size_t i = 0; i < speeds.size() - 1; ++i) {
      TrajectoryPoint traj_point;
      traj_point.path_point = spiral[i];
      traj_point.v = speeds[i];
      traj_point.relative_time = time;
      trajectory.push_back(traj_point);
      time_step = std::fabs(speeds[i] - speeds[i + 1]) / _a_max;  // Doubt!
      time += time_step;
    }


    // We still need to add the last one
    // 마지막 점 추가    
    auto i = spiral.size() - 1;
    TrajectoryPoint traj_point;
    traj_point.path_point = spiral[i];
    traj_point.v = speeds[i];
    traj_point.relative_time = time;
    trajectory.push_back(traj_point);

    // If the brake distance DOES NOT exceed the length of the path
  } else { // 감속 거리가 경로 길이를 초과하지 않는 경우
    auto brake_index{stop_index};
    auto temp_dist{0.0};
    
    // Compute the index at which to start braking down to zero.
     // 정지선까지의 감속 시작 지점 계산
    while ((brake_index > 0) and (temp_dist < brake_distance)) {
      temp_dist +=
          utils::distance(spiral[brake_index], spiral[brake_index - 1]);
      --brake_index;
    }
   
    // Compute the index to stop decelerating to the slow speed.
    // 저속 주행까지의 감속 시작 지점 계산
    uint decel_index{0};
    temp_dist = 0.0;
    while ((decel_index < brake_index) and (temp_dist < decel_distance)) {
      temp_dist +=
          utils::distance(spiral[decel_index + 1], spiral[decel_index]);
      ++decel_index;
    }
   
   
    // At this point we have all the speeds. Now we need to create the
    // trajectory
    // 궤적 생성
    double time_step{0.0};
    double time{0.0};
    auto vi{start_speed};
    for (size_t i = 0; i < decel_index; ++i) {
      auto dist = utils::distance(spiral[i + 1], spiral[i]);
      auto vf = calc_final_speed(vi, -_a_max, dist);
      if (vf < _slow_speed) {
        vf = _slow_speed;
      }
      TrajectoryPoint traj_point;
      traj_point.path_point = spiral[i];
      traj_point.v = vi;
      traj_point.relative_time = time;
      trajectory.push_back(traj_point);
      time_step = std::fabs(vf - vi) / _a_max;
      time += time_step;
      vi = vf;
    }

    // 저속 주행 구간
    for (size_t i = decel_index; i < brake_index; ++i) {
      TrajectoryPoint traj_point;
      traj_point.path_point = spiral[i];
      traj_point.v = vi;
      traj_point.relative_time = time;
      trajectory.push_back(traj_point);
      auto dist = utils::distance(spiral[i + 1], spiral[i]);  // ??
      if (dist > DBL_EPSILON)
        time_step = vi / dist;
      else
        time_step = 0.00;

      time += time_step;
    }

    //// 정지선까지의 감속 구간
    for (size_t i = brake_index; i < stop_index; ++i) {
      auto dist = utils::distance(spiral[i + 1], spiral[i]);
      auto vf = calc_final_speed(vi, -_a_max, dist);
      TrajectoryPoint traj_point;
      traj_point.path_point = spiral[i];
      traj_point.v = vi;
      traj_point.relative_time = time;
      trajectory.push_back(traj_point);
      time_step = std::fabs(vf - vi) / _a_max;
      time += time_step;
      vi = vf;
    }


    // Now we just need to add the last point.
    //마지막 점 추가 (정지 상태)
    auto i = stop_index;
    TrajectoryPoint traj_point;
    traj_point.path_point = spiral[i];
    traj_point.v = 0.0;
    traj_point.relative_time = time;
    trajectory.push_back(traj_point);
  }
  return trajectory;
}

// Computes a velocity trajectory for following a lead vehicle
// 선행 차량을 따라가는 궤적 생성 함수
std::vector<TrajectoryPoint> VelocityProfileGenerator::follow_trajectory(
    const std::vector<PathPoint>& spiral, const double& start_speed,
    const double& desired_speed, const State& lead_car_state) const {
  std::vector<TrajectoryPoint> trajectory;
  return trajectory;
}

// Computes a velocity trajectory for nominal speed tracking, a.k.a. Lane Follow
// or Cruise Control
// 일반 주행 궤적 생성 함수
std::vector<TrajectoryPoint> VelocityProfileGenerator::nominal_trajectory(
    const std::vector<PathPoint>& spiral, const double& start_speed,
    double const& desired_speed) const {
  std::vector<TrajectoryPoint> trajectory;
  double accel_distance;


  // LOG(INFO) << "MAX_ACCEL: " << _a_max;
// 가속 또는 감속 거리 계산
  if (desired_speed < start_speed) {
    // LOG(INFO) << "decelerate";
    accel_distance = calc_distance(start_speed, desired_speed, -_a_max);
  } else {
    // LOG(INFO) << "accelerate";
    accel_distance = calc_distance(start_speed, desired_speed, _a_max);
  }


// 가속/감속 구간의 끝 지점 계산
  size_t ramp_end_index{0};
  double distance{0.0};
  while (ramp_end_index < (spiral.size() - 1) && (distance < accel_distance)) {
    distance +=
        utils::distance(spiral[ramp_end_index], spiral[ramp_end_index + 1]);
    ramp_end_index += 1;
  }
  // LOG(INFO) << "ramp_end_index:" << ramp_end_index;

 // 궤적 생성
  double time_step{0.0};
  double time{0.0};
  double vi{start_speed};

  for (size_t i = 0; i < ramp_end_index; ++i) {
    auto dist = utils::distance(spiral[i], spiral[i + 1]);
    double vf;
    if (desired_speed < start_speed) {
      vf = calc_final_speed(vi, -_a_max, dist);

      if (vf < desired_speed) {
        vf = desired_speed;
      }
    } else {
      vf = calc_final_speed(vi, _a_max, dist);

      if (vf > desired_speed) {
        vf = desired_speed;
      }
    }
    TrajectoryPoint traj_point;
    traj_point.path_point = spiral[i];
    traj_point.v = vi;
    traj_point.relative_time = time;
    trajectory.push_back(traj_point);

    // LOG(INFO) << i << "- x: " << traj_point.path_point.x
    //          << ", y: " << traj_point.path_point.y
    //          << ", th: " << traj_point.path_point.theta
    //          << ", v: " << traj_point.v << ", t: " <<
    //          traj_point.relative_time;
    time_step = std::fabs(vf - vi) / _a_max;
    time += time_step;
    vi = vf;
  }

// 가속/감속 구간 이후의 궤적 생성
  for (size_t i = ramp_end_index; i < spiral.size() - 1; ++i) {
    TrajectoryPoint traj_point;
    traj_point.path_point = spiral[i];
    traj_point.v = desired_speed;
    traj_point.relative_time = time;
    trajectory.push_back(traj_point);

    auto dist = utils::distance(spiral[i], spiral[i + 1]);
    // This should never happen in a "nominal_trajectory", but it's a sanity
    // check
    if (std::abs(desired_speed) < DBL_EPSILON) {
      time_step = 0.0;
    } else {
      time_step = dist / desired_speed;
    }
    time += time_step;

    // LOG(INFO) << i << "- x: " << traj_point.path_point.x
    //          << ", y: " << traj_point.path_point.y
    //          << ", th: " << traj_point.path_point.theta
    //          << ", v: " << traj_point.v << ", t: " <<
    //          traj_point.relative_time;
  }

  // Add last point
  // 마지막 점 추가
  auto i = spiral.size() - 1;
  TrajectoryPoint traj_point;
  traj_point.path_point = spiral[i];
  traj_point.v = desired_speed;
  traj_point.relative_time = time;
  trajectory.push_back(traj_point);
  // LOG(INFO) << i << "- x: " << traj_point.path_point.x
  //          << ", y: " << traj_point.path_point.y
  //          << ", th: " << traj_point.path_point.theta
  //          << ", v: " << traj_point.v << ", t: " << traj_point.relative_time;

  // LOG(INFO) << "Trajectory Generated";
  return trajectory;
}

/*
Using d = (v_f^2 - v_i^2) / (2 * a), compute the distance
required for a given acceleration/deceleration.

Inputs: v_i - the initial speed in m/s.
        v_f - the final speed in m/s.
        a - the acceleration in m/s^2.
        */

// 등가속도 공식을 사용하여 이동 거리 계산
double VelocityProfileGenerator::calc_distance(const double& v_i,
                                               const double& v_f,
                                               const double& a) const {
  double d{0.0};

  // 가속도가 0일 경우, 나누기 오류 방지
  if (std::abs(a) < DBL_EPSILON) { 
    d = std::numeric_limits<double>::infinity(); // 무한대 거리 반환
  } else {
    // TODO-calc distance: use one of the common rectilinear accelerated ---TO DO 영역,calc_distance 함수 수정 (TODO-calc distance)
    /*
    해야 할 일:

    등가속도 운동 공식을 사용하여 주어진 가속도 a로 초기 속도 v_i에서 최종 속도 v_f까지 도달하는 데 필요한 거리 d를 계산
    공식을 d = (v_f^2 - v_i^2) / (2 * a) 사용
    가속도 a가 0일 경우 나누기 오류를 방지해야 함
        */

    // equations of motion to calculate the distance traveled while going from
    // v_i (initial velocity) to v_f (final velocity) at a constant
    // acceleration/deceleration "a". HINT look at the description of this
    // function. Make sure you handle div by 0
    
    
    //d = 0;  // <- Update, 주석처리된 원래 코드
    // 등가속도 운동 공식: d = (v_f^2 - v_i^2) / (2 * a)
    d = (std::pow(v_f, 2) - std::pow(v_i, 2)) / (2.0 * a);
     
     // 가속도가 0이면 나누기 오류 방지 (정지할 수 없음) 
     // 등가속도 운동 공식을 이용하여 이동 거리 계산 

  }
  return d;
}

/*
Using v_f = sqrt(v_i ^ 2 + 2ad), compute the final speed for a given
acceleration across a given distance, with initial speed v_i.
Make sure to check the discriminant of the radical. If it is negative,
return zero as the final speed.
Inputs : v_i - the initial speed in m / s.
v_f - the ginal speed in m / s.
a - the acceleration in m / s ^ 2.
*/

// 등가속도 공식을 사용하여 최종 속도 계산
double VelocityProfileGenerator::calc_final_speed(const double& v_i,
                                                  const double& a,
                                                  const double& d) const {
  double v_f{0.0};
  // TODO-calc final speed: Calculate the final distance. HINT: look at the --TO DO 부분 ,calc_final_speed 함수 수정 (TODO-calc final speed)
  // description of this function. Make sure you handle negative discriminant
  // and make v_f = 0 in that case. If the discriminant is inf or nan return
  // infinity
/*
 해야 할 일:

등가속도 운동 공식을 사용하여 주어진 거리 d와 가속도 a로 최종 속도 v_f를 계산
공식을 v_f = sqrt(v_i^2 + 2 * a * d) 사용
루트 내부(판별식)가 음수면 v_f = 0 반환
결과가 inf 또는 nan이면 무한대 반환

*/


  // double disc = 0;  // <- Fix this,원래 것 주석처리
 
  // 판별식 (루트 내부 값) 계산
  double disc = v_i * v_i + 2 * a * d; // 판별식 (루트 내부 값)
// 판별식 (루트 내부 값) 계산: v_f = sqrt(v_i^2 + 2ad)

 // 음수 판별식 처리 (속도는 음수가 될 수 없음)
 // 음수 판별식이 나오면 속도를 0으로 설정
  if (disc <= 0.0) {
    v_f = 0.0;


  // Inf 또는 NaN 값이 나오면 무한대 반환
  } else if (disc == std::numeric_limits<double>::infinity() ||
             std::isnan(disc)) {
    v_f = std::numeric_limits<double>::infinity();

     // 정상적인 경우 루트 값 계산
  } else {
    v_f = std::sqrt(disc);
  }
  //   std::cout << "v_i, a, d: " << v_i << ", " << a << ", " << d
  //             << ",  v_f: " << v_f << std::endl;
  return v_f;
}




  