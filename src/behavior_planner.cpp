#include "behavior_planner.h"
#include <math.h>

BehaviorPlanner::BehaviorPlanner() {}

BehaviorPlanner::~BehaviorPlanner() {}

int BehaviorPlanner::planLane(double car_x, double car_y,
  double car_s, double car_d, double car_yaw, double car_speed,
  std::vector<double> previous_path_x, std::vector<double> previous_path_y,
  double end_path_s, double end_path_d, std::vector<std::vector<double>> sensor_fusion, double max_v) {

  this-> currentLane = ceil(car_d/4)-1;
  if(prevTargetLane == -1) this->prevTargetLane = currentLane;
  if(targetLane == -1) this->targetLane = currentLane;

  //unsafe assumption for now, just estimate forward 1 second
  // [lane, car_x, cary_y, car_s, car_d, car_speed, cost]
  std::vector<std::vector<double>> vehicle_states;
  for (int lane = laneMin; lane <= laneMax; lane++) {
    double d = 2.0+lane*4;
    std::vector<double> xy = getXY(car_s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    //put cost as last items and as 0 for now
    std::vector<double> v {1.0*lane, xy[0], xy[1], car_s, d, car_speed, 0.0};
    vehicle_states.push_back(v);
  }

  // std::cout << "[id, x, y, vx, vy, s, d]" << std::endl;
  // for(int i = 0; i < sensor_fusion.size(); i++) {
  // 	std::cout << "[";
	// 	for(auto val : sensor_fusion[i]) {
	// 		std::cout << val << ", ";
	// 	}
	// 	std::cout << "]" << std::endl;
  // }


  double collision_cost = 300;
  double unclear_lane_cost = 100;

  std::cout << "car_s:" << car_s << " pathS:" << end_path_s << " v:" << car_speed << std::endl;

  //loop through our car states
  for (int state_index = 0; state_index < vehicle_states.size(); state_index++) {
    std::vector<double>& vehicle_state = vehicle_states[state_index];
    bool laneUnclear = false;
    // Sensor Fusion: [id, x, y, vx, vy, s, d]
    for(int oc_index = 0; oc_index < sensor_fusion.size(); oc_index++) {
      std::vector<double>& otherCar = sensor_fusion[oc_index];
      double newX = otherCar[1] + otherCar[3];
      double newY = otherCar[2] + otherCar[4];
      double& otherCarOldS = otherCar[5];
      double otherCarLane = ceil(otherCar[6]/4)-1; //ceil(frenet[1]/4)-1;
      double otherCarVs = distance(newX, newY, otherCar[1], otherCar[2]);//otherCarS - otherCarOldS;
      double otherCarS = otherCarOldS + otherCarVs; //frenet[0];

      bool logging = false;

      // if lane is the same and the other carS is going to be between where we are and where we'll be
      if(otherCarLane == vehicle_state[0]) {
        //logging
        if(otherCarOldS > car_s - 10 && otherCarS < end_path_s + 70) {
          logging = true;
          std::cout << "Lane:" << otherCarLane << " car_s:" << car_s << " pathS:";
          std::cout << end_path_s << " s:" << otherCarOldS; //<< " ID:" << otherCar[0] << " d:" << otherCar[6]
          std::cout << " newS:" << otherCarS << " S2:" << (otherCarS + otherCarVs) << " V:" << otherCarVs << " d:" << otherCarLane;
        }

        //calculate collision cost
        if(otherCarOldS >= car_s-7 && (otherCarS-5) <= (car_s + car_speed)) {
        // if(otherCarOldS >= car_s-7 && (otherCarOldS) <= (car_s + 10)) {
          std::cout << " COLLISION ";
          vehicle_state[6] += collision_cost;
        }
        //looking much further ahead to attempt to find a clear lane with a speed bias
        if(otherCarOldS > car_s && (otherCarS) <= (end_path_s + max_v*3)) {
          std::cout << " UNCLEAR";
          laneUnclear = true; //mark lane as not being clear
        }

        //record velocity if other car is in sight and is slower
        if(otherCarOldS >= car_s && (otherCarS) <= (end_path_s + max_v*3)) {
          if(otherCarVs < vehicle_state[5]) {
            vehicle_state[5] = otherCarVs;
          }
          std::cout << " VELOCITY";
        }
      }
      if(logging) std::cout << std::endl;
    }

    //check lane is clear and calculate cost that way
    if(laneUnclear) {
      vehicle_state[6] += unclear_lane_cost;
    }
  }

  //calculate velocity cost
  for (int state_index = 0; state_index < vehicle_states.size(); state_index++) {
    std::vector<double>& vehicle_state = vehicle_states[state_index];
    double velocity_cost = 0.0;
    double velocity_weight = 100.0;
    if(vehicle_state[5] < max_v) {
      velocity_cost = 1.0 * (max_v - vehicle_state[5]) / max_v;
    }
    // if(velocity_cost <= 1) velocity_cost = 0;
    // std::cout << "velocity_cost:" << velocity_cost << std::endl;
    vehicle_state[6] += velocity_cost*velocity_weight;
  }

  int tempTargetLane = currentLane;
  double min_cost = 999999999999.0;
  for(int i=0; i<vehicle_states.size(); i++) {
    double& cost = vehicle_states[i][6];
    double& lane = vehicle_states[i][0];
    //TODO: fix this duplication
    if(cost <= min_cost && lane == prevTargetLane) {
      // std::cout << "best cost:" << cost << " lane:" << lane << std::endl;
      min_cost = cost;
      tempTargetLane = lane;
    } else if(cost < min_cost-5) { //don't change lanes just because of minor cost diff
      // std::cout << "best cost:" << cost << " lane:" << lane << std::endl;
      min_cost = cost;
      tempTargetLane = lane;
    }
  }

  // std::cout << "[lane, car_x, cary_y, car_s, car_d, car_speed, cost]" << std::endl;
  for(int i = 0; i < vehicle_states.size(); i++) {
    std::cout << "[";
    for(auto val : vehicle_states[i]) {
      std::cout << val << ", ";
    }
    std::cout << "]" << std::endl;
  }

  if(tempTargetLane != currentLane) {
    std::cout << "=================================changing lanes " << currentLane << " -> " << tempTargetLane << "====================================" << std::endl;
  }

  //we haven't finished changing lanes yet
  // if(tempTargetLane != targetLane && targetLane != currentLane) {
  //   //keep going
  // } else
  //smooth targetLane and don't change if we're going to crash, unless we're already started the lane change process
  // at which point the path planner should slow us down
  if(tempTargetLane < currentLane && (vehicle_states[currentLane-1][6] < collision_cost || targetLane==tempTargetLane)) {
    this->targetLane = currentLane-1;
    // std::cout << "======================================changing lanes=======================================" << std::endl;
  }
  else if(tempTargetLane > currentLane && (vehicle_states[currentLane+1][6] < collision_cost || targetLane==tempTargetLane)) {
    this->targetLane = currentLane+1;
    // std::cout << "======================================changing lanes=======================================" << std::endl;
  }
  else if(tempTargetLane == currentLane) this->targetLane = currentLane;

  std::cout << "targetLane: " << targetLane << std::endl;

  this->prevTargetLane = targetLane;



  return this->targetLane;
}
