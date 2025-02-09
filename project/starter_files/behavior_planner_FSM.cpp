



/* 
주요 로직
입방 나선 생성: motion_planner.cpp 파일에서 구현된 입방 나선을 사용하여 목표를 따라가며 경로를 생성.
충돌 검사: cost_functions.cpp에서 원 기반 충돌 검사 알고리즘을 구현.
FSM 로직: behavior_planner_FSM.cpp에서 상태 전환과 로직을 설계.



파일: behavior_planner_FSM.cpp
해야 할 TODO:

전방 살피기: 정지점까지의 타당한 전방 거리 계산.
정지점 뒤에 목표 생성: _stop_line_buffer를 사용하여 정지점을 기준으로 목표 위치 설정.
목표 속도 설정:
정지점에서의 목표 속도.
공칭 상태에서의 목표 속도.
상태 전환 로직 구현:
"decel_to_stop" → "stopped".
"stopped" → "follow_lane".
거리 기반 로직 추가: 속도 대신 거리 정보를 활용하여 DECEL_TO_STOP 상태에서 정지로 전환.
정지 상태 유지: "stopped" 상태에서 동일 목표 유지.

*/



/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file behavior_planner_FSM.cpp
 @brief 자율주행차의 상태 기계(Finite State Machine, FSM) 로직을 구현하는 파일

@details 차량의 현재 상태를 바탕으로 목표를 설정하고, 상태를 전환하는 역할을 수행함.

 **/


#include "behavior_planner_FSM.h"
//@brief 차량이 주행하는 도로에서 가장 가까운 목표 웨이포인트를 찾는 함수
//@param ego_state 차량의 현재 상태
//@param map 도로 지도 객체
//@param lookahead_distance 전방 탐색 거리
//@param is_goal_junction 목표 지점이 교차로인지 여부
//@return 가장 가까운 웨이포인트 (State 형식)
//State BehaviorPlannerFSM::get_closest_waypoint_goal(const State& ego_state, const SharedPtrcc::Map& map,const float& lookahead_distance, bool& is_goal_junction) {
// -->차량의 현재 위치에서 가장 가까운 웨이포인트를 찾음  auto waypoint_0 = map->GetWaypoint(ego_state.location);







// 가장 가까운 웨이포인트를 찾아 목표로 설정하는 함수
State BehaviorPlannerFSM::get_closest_waypoint_goal( //get_closest_waypoint_goal:차량의 현재 위치에서 가장 가까운 웨이포인트를 찾아 목표로 설정합니다. 
                                                     //특히, DECEL_TO_STOP 또는 STOPPED 상태에서는 현재 웨이포인트를 그대로 유지
    const State& ego_state, const SharedPtr<cc::Map>& map,
    const float& lookahead_distance, bool& is_goal_junction) {
  // Nearest waypoint on the center of a Driving Lane.
   // 현재 차량 위치에서 가장 가까운 웨이포인트를 찾음
  auto waypoint_0 = map->GetWaypoint(ego_state.location);

 // 현재 상태가 DECEL_TO_STOP 또는 STOPPED인 경우, 현재 웨이포인트를 그대로 반환
  if (_active_maneuver == DECEL_TO_STOP || _active_maneuver == STOPPED) {
    State waypoint;
    auto wp_transform = waypoint_0->GetTransform();
    waypoint.location = wp_transform.location;
    waypoint.rotation.yaw = utils::deg2rad(wp_transform.rotation.yaw);
    waypoint.rotation.pitch = utils::deg2rad(wp_transform.rotation.pitch);
    waypoint.rotation.roll = utils::deg2rad(wp_transform.rotation.roll);
    return waypoint;
  }

  // Waypoints at a lookahead distance
  // NOTE: "GetNext(d)" creates a list of waypoints at an approximate distance
  // "d" in the direction of the lane. The list contains one waypoint for each
  // deviation possible.

  // NOTE 2: GetNextUntilLaneEnd(d) returns a list of waypoints a distance "d"
  // apart. The list goes from the current waypoint to the end of its
  // lane.

// lookahead_distance만큼 앞에 있는 웨이포인트들을 가져옴
  auto lookahead_waypoints = waypoint_0->GetNext(lookahead_distance);
  auto n_wp = lookahead_waypoints.size();
  if (n_wp == 0) {
    // LOG(INFO) << "Goal wp is a nullptr";
    State waypoint;
    return waypoint;
  }
  // LOG(INFO) << "BP - Num of Lookahead waypoints: " << n_wp;

// 가장 먼 웨이포인트를 목표로 설정
  waypoint_0 = lookahead_waypoints[lookahead_waypoints.size() - 1];

// 목표 웨이포인트가 교차로인지 확인
  is_goal_junction = waypoint_0->IsJunction();
  // LOG(INFO) << "BP - Is Last wp in junction? (0/1): " << is_goal_junction;
  auto cur_junction_id = waypoint_0->GetJunctionId();
  if (is_goal_junction) {
    if (cur_junction_id == _prev_junction_id) {
      // LOG(INFO) << "BP - Last wp is in same junction as ego. Junction ID: "
      //          << _prev_junction_id;
      is_goal_junction = false;
    } else {
      // LOG(INFO) << "BP - Last wp is in different junction than ego. Junction
      // ID: "
      //          << cur_junction_id;
      _prev_junction_id = cur_junction_id;
    }
  }

   // 목표 웨이포인트의 상태를 반환
  State waypoint;
  auto wp_transform = waypoint_0->GetTransform();
  waypoint.location = wp_transform.location;
  waypoint.rotation.yaw = utils::deg2rad(wp_transform.rotation.yaw);
  waypoint.rotation.pitch = utils::deg2rad(wp_transform.rotation.pitch);
  waypoint.rotation.roll = utils::deg2rad(wp_transform.rotation.roll);
  return waypoint;
}

// 차량의 속도와 가속도를 기반으로 lookahead_distance를 계산하는 함수
double BehaviorPlannerFSM::get_look_ahead_distance(const State& ego_state) { 
                           //get_look_ahead_distance :차량의 속도와 가속도를 기반으로 lookahead_distance를 계산합니다. 이 거리는 차량이 안전하게 정지할 수 있는 거리를 고려해야
  auto velocity_mag = utils::magnitude(ego_state.velocity);
  auto accel_mag = utils::magnitude(ego_state.acceleration);

  // TODO-Lookahead: One way to find a reasonable lookahead distance is to find
  // the distance you will need to come to a stop while traveling at speed V and
  // using a comfortable deceleration.
  
  // TODO: 차량이 현재 속도로 주행 중일 때, 안전하게 정지할 수 있는 거리를 계산해야 함
  auto look_ahead_distance = 1.0;  // <- Fix This

  // LOG(INFO) << "Calculated look_ahead_distance: " << look_ahead_distance;

  
  // lookahead_distance를 최소 및 최대 값으로 제한
  // 차량이 현재 속도로 주행 중일 때, 안전하게 정지할 수 있는 거리를 계산해야 함
  look_ahead_distance =
      std::min(std::max(look_ahead_distance, _lookahead_distance_min),
               _lookahead_distance_max);  // _lookahead_distance_min, _lookahead_distance_max가  planning_params.h의 P_LOOKAHEAD_MIN 및 P_LOOKAHEAD_MAX와 연결됨.




  // LOG(INFO) << "Final look_ahead_distance: " << look_ahead_distance;

  return look_ahead_distance;
}



// 차량의 현재 상태와 맵을 기반으로 목표 상태를 계산하는 함수
State BehaviorPlannerFSM::get_goal(const State& ego_state, 
                           //get_goal: 차량의 현재 상태와 맵을 기반으로 목표 상태를 계산. 이 함수는 lookahead_distance를 계산하고, 가장 가까운 웨이포인트를 찾아 목표로 설정
                                   SharedPtr<cc::Map> map) {
  // Get look-ahead distance based on Ego speed

  // 차량의 속도에 기반하여 lookahead_distance를 계산
  auto look_ahead_distance = get_look_ahead_distance(ego_state);

  // Nearest waypoint on the center of a Driving Lane.
  // 가장 가까운 웨이포인트를 찾아 목표로 설정
  bool is_goal_in_junction{false};
  auto goal_wp = get_closest_waypoint_goal(ego_state, map, look_ahead_distance,
                                           is_goal_in_junction);

  // LOG(INFO) << "Is the FINAL goal on a junction: " << is_goal_in_junction;
   // 상태 전환 로직을 통해 최종 목표 상태를 결정
  string tl_state = "none";
  State goal =
      state_transition(ego_state, goal_wp, is_goal_in_junction, tl_state);
      //state_transition:상태 전환 로직을 처리. 
      //FOLLOW_LANE, DECEL_TO_STOP, STOPPED 상태에 따라 목표 위치와 속도를 조정하고, 상태 전환을 수행
      //특히, 정지선까지의 거리를 기반으로 상태를 전환하는 로직이 포함되어 있음

  return goal;
}

// 상태 전환 로직을 처리하는 함수
State BehaviorPlannerFSM::state_transition(const State& ego_state, State goal,
                                           bool& is_goal_in_junction,
                                           string tl_state) {
  // Check with the Behavior Planner to see what we are going to do and
  // where our next goal is
  //
// 목표 상태의 가속도를 0으로 설정
  goal.acceleration.x = 0;
  goal.acceleration.y = 0;
  goal.acceleration.z = 0;

// 현재 상태가 FOLLOW_LANE인 경우
  if (_active_maneuver == FOLLOW_LANE) {
    // LOG(INFO) << "BP- IN FOLLOW_LANE STATE";
    if (is_goal_in_junction) {
        // 목표가 교차로인 경우, DECEL_TO_STOP 상태로 전환
      // LOG(INFO) << "BP - goal in junction";

      _active_maneuver = DECEL_TO_STOP;
      // LOG(INFO) << "BP - changing to DECEL_TO_STOP";

      // Let's backup a "buffer" distance behind the "STOP" point
      // LOG(INFO) << "BP- original STOP goal at: " << goal.location.x << ", "
      //          << goal.location.y;

      // TODO-goal behind the stopping point: put the goal behind the stopping --->todo 영역
      // point (i.e the actual goal location) by "_stop_line_buffer". HINTS:
      // remember that we need to go back in the opposite direction of the
      // goal/road, i.e you should use: ang = goal.rotation.yaw + M_PI and then
      // use cosine and sine to get x and y
      
       // 정지선 뒤에 목표 위치를 설정
      auto ang = goal.rotation.yaw + M_PI; 
      //goal.location.x += 1.0;  // <- Fix This, 이 값을 바꿔라
      //goal.location.y += 1.0;  // <- Fix This


      //이 부분은 차량이 정지선에서 너무 앞으로 나가지 않게 하기 위해 살짝 뒤쪽으로 목표를 조정하는 코드
      goal.location.x -= _stop_line_buffer * std::cos(ang); //차량이 정지선에서 약간 뒤쪽으로 멈추도록 조정
      goal.location.y -= _stop_line_buffer * std::sin(ang); //	정지선 기준으로 목표 위치를 후진 방향으로 이동
                                                             //_stop_line_buffer가 planning_params.h의 P_STOP_LINE_BUFFER와 연결됨.


      //goal.location : 차량이 멈춰야할 목표지점
      //_stop_line_buffer는 정지선보다 약간 뒤쪽에서 멈추도록 조정하는 거리 값
      //ang = goal.rotation.yaw + M_PI 이므로 현재 방향(yaw)에서 180도 반대 방향을 계산
      //cos(ang) 및 sin(ang)을 사용하여 정지선 기준으로 후진 방향으로 이동

      //실행시 _stop_line_buffer 값이 너무 크거나 작으면 차량이 정지선과 너무 멀거나 너무 가까울수 있음.
      //이 경우 planning_params.h에서 _stop_line_buffer값을 조정하면서 테스트 필요함
      //기본적으로 1.0~2.5정도 값을 권장함

      // LOG(INFO) << "BP- new STOP goal at: " << goal.location.x << ", "
      //          << goal.location.y;

      // TODO-goal speed at stopping point: What should be the goal speed?? -->todo 영역
      //goal.velocity.x = 1.0;  // <- Fix This ,이 값을 바꿔라
      //goal.velocity.y = 1.0;  // <- Fix This
      //goal.velocity.z = 1.0;  // <- Fix This


 // 정지선에서의 목표 속도를 0으로 설정
      goal.velocity.x = 0; 
      goal.velocity.y = 0;
      goal.velocity.z = 0;

    } else {
      // TODO-goal speed in nominal state: What should be the goal speed now --todo영역
      // that we know we are in nominal state and we can continue freely?
      // Remember that the speed is a vector
      // HINT: _speed_limit * std::sin/cos (goal.rotation.yaw);
      //goal.velocity.x = 1.0;  // <- Fix This , 이 값을 바꿔라
      //goal.velocity.y = 1.0;  // <- Fix This
      //goal.velocity.z = 0;

   // 일반 주행 상태에서의 목표 속도를 설정
   //차량이 현재 주행해야 할 방향(yaw)에 맞춰 목표 속도를 설정하는 코드
      goal.velocity.x = _speed_limit * std::cos(goal.rotation.yaw); //차량이 도로 방향을 따라 속도를 설정
      goal.velocity.y = _speed_limit * std::sin(goal.rotation.yaw); //차량이 도로 진행 방향으로 자연스럽게 움직이도록 설정
      //_speed_limit은 차량의 목표 속도
      //_speed_limit가 planning_params.h의  P_SPEED_LIMIT와 연결됨.


      //goal.rotation.yaw는 차량이 향하는 방향 (도로 진행 방향)
      //cos(goal.rotation.yaw), sin(goal.rotation.yaw)을 사용하여 x축, y축 방향 속도 계산


      goal.velocity.z = 0;
    }

  } else if (_active_maneuver == DECEL_TO_STOP) {

    
    // LOG(INFO) << "BP- IN DECEL_TO_STOP STATE";
    // TODO-maintain the same goal when in DECEL_TO_STOP state: Make sure the -- todo영역
    // new goal is the same as the previous goal (_goal). That way we
    // keep/maintain the goal at the stop line.
       //goal = ;  // <- Fix This --이 값을 바꿔라??

        // DECEL_TO_STOP 상태에서는 이전 목표를 유지
       goal = _goal; // 이전 목표 유지  (stopped상태(일정시간 정지선에서 정지, 차량이 멈추려면 현재 위치에서 새로운 목표를 설정하면 안됨))
                     // 즉, goal을 새롭게 계산하는 것이 아니라 이전 목표 _goal을 그대로 유지해야함.
                    // 그렇지 않으면 정지선을 지나서 움직이거나 이상한 위치로 이동할 수 있음
                    //만약 차량이 멈추지 않고 움직인다면, planning_params.h에서  _stop_threshold_speed값을 낮추면 멈추는 시간이 길어짐
                    //(예: _stop_threshold_speed =0.5(기본)--> 0.2로 변경시 더 빨리 정지

    // TODO: It turns out that when we teleport, the car is always at speed
    // zero. In this the case, as soon as we enter the DECEL_TO_STOP state,
    // the condition that we are <= _stop_threshold_speed is ALWAYS true and we
    // move straight to "STOPPED" state. To solve this issue (since we don't
    // have a motion controller yet), you should use "distance" instead of
    // speed. Make sure the distance to the stopping point is <=
    // P_STOP_THRESHOLD_DISTANCE. Uncomment the line used to calculate the
    // distance


     // 정지선까지의 거리를 계산
    auto distance_to_stop_sign =
        utils::magnitude(goal.location - ego_state.location);
    // LOG(INFO) << "Ego distance to stop line: " << distance_to_stop_sign;

    // TODO-use distance rather than speed: Use distance rather than speed... --> 
    //if (utils::magnitude(ego_state.velocity) <= //  속도 기반 비교, 이 부분을 거리 기반 비교로 변경해야함, 그래서 주석처리
    //    _stop_threshold_speed) {  // -> Fix this //그래서 주석처리
      
      // 정지선까지의 거리가 임계값 이하인 경우, STOPPED 상태로 전환
      if (distance_to_stop_sign <= P_STOP_THRESHOLD_DISTANCE) { //(거리비교 코드를 주석해제) , planning_params.h의 P_STOP_THRESHOLD_DISTANCE가 차량이 정지할 거리 기준으로 사용됨.
      // TODO-move to STOPPED state: Now that we know we are close or at the
      // stopping point we should change state to "STOPPED"
      //_active_maneuver = ;  // <- Fix This
      _active_maneuver = STOPPED; //Stopped상태로 이동
      


      _start_stop_time = std::chrono::high_resolution_clock::now(); //이 부분은 그대로 사용
      // LOG(INFO) << "BP - changing to STOPPED";
    }
  } else if (_active_maneuver == STOPPED) {
    // LOG(INFO) << "BP- IN STOPPED STATE";
    // TODO-maintain the same goal when in STOPPED state: Make sure the new goal
    // is the same as the previous goal. That way we keep/maintain the goal at
    // the stop line. goal = ...;
       //goal = ;  // Keep previous goal. Stay where you are. // <- Fix This

       // STOPPED 상태에서는 이전 목표를 유지
       goal = _goal; // 이전 목표 유지  (stopped상태(일정시간 정지선에서 정지, 차량이 멈추려면 현재 위치에서 새로운 목표를 설정하면 안됨))
                     // 즉, goal을 새롭게 계산하는 것이 아니라 이전 목표 _goal을 그대로 유지해야함.
                    // 그렇지 않으면 정지선을 지나서 움직이거나 이상한 위치로 이동할 수 있음

                    //하지만 차량이 STOPPED상태에서 FOllow_Lane으로 정상적으로 이동하는지 확인필요
                    //아래의 if (stopped_secs >= _req_stop_time && tl_state.compare("Red") != 0) 부분이 그 부분임


  // 정지 상태 유지 시간을 계산
    long long stopped_secs =
        std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::high_resolution_clock::now() - _start_stop_time)
            .count();
    // LOG(INFO) << "BP- Stopped for " << stopped_secs << " secs";

    
    // 정지 시간이 요구 시간을 초과하고 신호등이 빨간불이 아닌 경우, FOLLOW_LANE 상태로 전환
    if (stopped_secs >= _req_stop_time && tl_state.compare("Red") != 0) { //이 부분은 그대로 사용
                                                                         // _req_stop_time = 2.0 ~ 3.0초 정도로 설정하는 것이 적절
                                                                         //_req_stop_time이  planning_params.h의 P_REQ_STOPPED_TIME과 연결됨.

      // TODO-move to FOLLOW_LANE state: What state do we want to move to, when --> 이 조건에서 Follow_Lane으로 전환해야함
      // we are "done" at the STOPPED state?
      //_active_maneuver = ;  // <- Fix This //_active_maneuver = FOLLOW_LANE;를 추가해야 함.
       _active_maneuver = FOLLOW_LANE;  // FOLLOW_LANE 상태로 이동
        
      // LOG(INFO) << "BP - changing to FOLLOW_LANE";
    }
  }
  _goal = goal;   // 최종 목표 상태를 저장하고 반환
  return goal;
}





