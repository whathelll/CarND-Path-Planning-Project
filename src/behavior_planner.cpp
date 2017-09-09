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

  //unsafe assumption for now, just estimate forward 1 second
  // [lane, car_x, cary_y, car_s, car_d, car_speed, cost]
  std::vector<std::vector<double>> vehicle_states;
  for (int lane = laneMin; lane <= laneMax; lane++) {
    double d = 2.0+lane*4;
    std::vector<double> xy = getXY(car_s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    std::vector<double> v {1.0*lane, xy[0], xy[1], car_s, d, car_speed, 0.0}; //put cost as last item and as 0 for now
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

  // Sensor Fusion: [id, x, y, vx, vy, s, d]
  for(int oc_index = 0; oc_index < sensor_fusion.size(); oc_index++) {
    std::vector<double>& otherCar = sensor_fusion[oc_index];
    double newX = otherCar[1] + otherCar[3]*1;
    double newY = otherCar[2] + otherCar[4]*1;
    double& otherCarOldS = otherCar[5];
    double angle = atan2(newX-otherCar[1],newY-otherCar[2]);
    std::vector<double> frenet = getFrenet(newX, newY, angle, map_waypoints_x, map_waypoints_y);
    double otherCarLane = ceil(frenet[1]/4)-1;
    double& otherCarS = frenet[0];
    double otherCarVs = otherCarS - otherCarOldS;

    bool logging = false;
    if(otherCarOldS > car_s - 30 && otherCarS < end_path_s + 30) {
      logging = true;
      std::cout << "Lane:" << otherCarLane << " ID:" << otherCar[0] << " d:" << otherCar[6] << " s:" << otherCarOldS ;
      std::cout << " newS:" << otherCarS << " S2:" << (otherCarS + otherCarVs*2.5) << " V:" << otherCarVs;
    }

    //loop through our car states
    for (int state_index = 0; state_index < vehicle_states.size(); state_index++) {
      std::vector<double>& vehicle_state = vehicle_states[state_index];
      // if lane is the same and the other carS is going to be between where we are and where we'll be
      if(otherCarLane == vehicle_state[0]) {
        //calculate collision cost
        if(otherCarOldS >= car_s-20 && otherCarS <= (car_s + car_speed)) {
          // std::cout << "collision:" << otherCarLane << " ID:" << otherCar[0] << " s:" << otherCarOldS << " d:" << otherCar[6];
          // std::cout << " oldS:" << otherCarOldS; //" oldX:" << otherCar[1] << " oldY:" << otherCar[2];
          // std::cout << std::endl;
          std::cout << " COLLISION ";
          vehicle_state[6] += collision_cost;
        }
        //looking much further ahead to attempt to find a clear lane with a speed bias
        if(otherCarOldS > car_s && (otherCarS + otherCarVs*2.5) <= (end_path_s + car_speed*2.5)) {
          // std::cout << "lane not clear:" << otherCarLane << " ID:" << otherCar[0] << " s:" << frenet[0] << " d:" << frenet[1];
          // std::cout << " oldS:" << otherCarOldS; // << " oldX:" << otherCar[1] << " oldY:" << otherCar[2];
          // std::cout << std::endl;
          std::cout << " UNCLEAR";
          // double velocity_s = otherCarS - otherCarOldS;  //TODO change this calculation
          vehicle_state[6] += unclear_lane_cost;// + (max_v - velocity_s);
        }

        //calculate velocity cost if other car is in sight
        if(otherCarOldS >= car_s && (otherCarS + otherCarVs*2.5) <= (end_path_s + car_speed*2.5)) {
          double velocity_cost = 0.0;
          double velocity_weight = 100.0;
          double velocity_s = otherCarS - otherCarOldS;
          if(velocity_s < max_v) {
            velocity_cost = 1.0 * (max_v - velocity_s) / max_v;
          }
          // std::cout << "velocity slow lane:" << otherCarLane << " ID:" << vehicle_state[0] << " velocity cost: " << velocity_cost*velocity_weight << std::endl;
          std::cout << " VELOCITY";
          vehicle_state[6] += velocity_cost*velocity_weight;
        }
      }
    }
    if(logging) std::cout << std::endl;
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
    } else if(cost < min_cost) {
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
  //smooth targetLane and don't change if we're going to crash
  if(tempTargetLane < currentLane && vehicle_states[currentLane-1][6] < collision_cost) {
    this->targetLane = currentLane-1;
    std::cout << "======================================changing lanes=======================================" << std::endl;
  }
  else if(tempTargetLane > currentLane && vehicle_states[currentLane+1][6] < collision_cost) {
    this->targetLane = currentLane+1;
    std::cout << "======================================changing lanes=======================================" << std::endl;
  }
  else if(tempTargetLane == currentLane) this->targetLane = currentLane;

  std::cout << "targetLane: " << targetLane << std::endl;

  this->prevTargetLane = targetLane;



  return this->targetLane;
}
