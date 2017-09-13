#include "path_planner.h"
#include "spline.h"
#include <math.h>

PathPlanner::PathPlanner() {}

PathPlanner::~PathPlanner() {}

std::vector<std::vector<double>> PathPlanner::planPath(double car_x, double car_y,
  double car_s, double car_d, double car_yaw, double car_speed,
  std::vector<double> previous_path_x, std::vector<double> previous_path_y,
  double end_path_s, double end_path_d, std::vector<std::vector<double>> sensor_fusion, int lane) {

  car_speed = car_speed  / 0.62137 * 1000 / 3600;  // meters per second
  // max speed in meters  = ~22m/s
  double max_v = (49.8 / 0.62137) * 1000 / 3600;
  // max acceleration 10m/s
  double max_v_dot = 8.5; //for some reason the simulator sometimes blips a accel violation, lowering this to avoid it.
  double currentLane = ceil(car_d/4)-1;

  // Calculate max_v to match car in front of us.
  // Sensor Fusion: [id, x, y, vx, vy, s, d]
  for(int oc_index = 0; oc_index < sensor_fusion.size(); oc_index++) {
    std::vector<double>& otherCar = sensor_fusion[oc_index];
    double& otherCarOldS = otherCar[5];
    double otherCarLane = ceil(otherCar[6]/4)-1;
    double otherCarVx = otherCar[3];
    double otherCarVy = otherCar[4];
    double otherCarV = sqrt(otherCarVx*otherCarVx + otherCarVy*otherCarVy);

    //predict the future D 1 second later, on rare occassions they make dangerous lane changes
    double newX = otherCar[1] + otherCar[3]*1;
    double newY = otherCar[2] + otherCar[4]*1;
    double angle = atan2(newX-otherCar[1],newY-otherCar[2]);
    std::vector<double> frenet = getFrenet(newX, newY, angle, map_waypoints_x, map_waypoints_y);
    double otherCarFutureLane = ceil(frenet[1]/4)-1;


    if(otherCarLane == currentLane || otherCarLane==lane || otherCarFutureLane==currentLane) {
      //calculate collision and match speed
      if(otherCarOldS >= car_s && (otherCarOldS+otherCarV*0.2) <= (end_path_s + car_speed*0.2) && otherCarV < max_v) {
        max_v = otherCarV*0.95; //aim for slightly slower than them
        std::cout << "Slowing down for ID:"  << otherCar[0] << " otherCarV:" << otherCarV << " max_v:" << max_v;
        std::cout << std::endl;
      }
    }
  }


  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  int path_size = previous_path_x.size();
  double timeStep = 0.02;

  double v_dot;
  double v;
  double s;
  double angle;
  std::vector<double> ref_path_x;
  std::vector<double> ref_path_y;
    // s = vt + 1/2 * a * t^2
  // double dist_inc = 0.43;

  if(path_size <= 2) {
    v = car_speed;
    s = car_s;
    end_path_s = car_s;
    end_path_d = car_d;
    angle = deg2rad(car_yaw);

    double prev_car_x = car_x - cos(angle);
    double prev_car_y = car_y - sin(angle);
    ref_path_x.push_back(prev_car_x);
    ref_path_y.push_back(prev_car_y);

    ref_path_x.push_back(car_x);
    ref_path_y.push_back(car_y);

  } else {
    double pos_x = previous_path_x[path_size-1];
    double pos_y = previous_path_y[path_size-1];

    double pos_x2 = previous_path_x[path_size-2];
    double pos_y2 = previous_path_y[path_size-2];
    angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
    s = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y)[0];
    v = distance(pos_x, pos_y, pos_x2, pos_y2)/0.02;
    ref_path_x.push_back(pos_x2);
    ref_path_y.push_back(pos_y2);
    ref_path_x.push_back(pos_x);
    ref_path_y.push_back(pos_y);
  }
  //
  // // add 2 more ref points
  std::vector<double> ref_pt1 = getXY(s + 30.0, 2+lane*4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  std::vector<double> ref_pt2 = getXY(s + 90.0, 2+lane*4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  ref_path_x.push_back(ref_pt1[0]);
  ref_path_y.push_back(ref_pt1[1]);
  ref_path_x.push_back(ref_pt2[0]);
  ref_path_y.push_back(ref_pt2[1]);

  // for(int i=0; i<ref_path_x.size(); i++) {
  //   std::cout << ref_path_x[i] << ", " << ref_path_y[i] << std::endl;
  // }

  // // use the 2nd point as point of where the car will be. shift points to
  // // use car as centre
  double ref_x = ref_path_x[1];
  double ref_y = ref_path_y[1];
  for(int i = 0; i < ref_path_x.size(); i++) {
    double shift_x = ref_path_x[i] - ref_x;
    double shift_y = ref_path_y[i] - ref_y;
    ref_path_x[i] = (shift_x * cos(0-angle) - shift_y*sin(0-angle));
    ref_path_y[i] = (shift_x * sin(0-angle) + shift_y*cos(0-angle));
  }
  // std::cout << "RefX:" << ref_path_x[1] << " RefY:" << ref_path_y[1] << std::endl;
  tk::spline spline;
  spline.set_points(ref_path_x, ref_path_y);


  for(int i = 0; i < path_size; i++) {
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
  }

  double target_x = 30.0;
  double target_y = spline(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  //calc required acceleration
  int num_paths_to_calculate = 50 - path_size;
  double t = (num_paths_to_calculate*0.02);
  v_dot = (max_v - v) / t;
  if(v_dot > 0 && v_dot > max_v_dot) v_dot = max_v_dot;
  if(v_dot < 0 && v_dot < -max_v_dot) v_dot = -max_v_dot;

  // generate the additional points
  for(int i=1; i<=50-path_size; i++) {
    double t_temp = i*0.02;
    double distance_covered = v * t_temp + 0.5 * v_dot * t_temp*t_temp;
    double distance_ratio = (distance_covered/target_dist); // (0.002/5) (5/30)
    double x_point = target_x*distance_ratio;
    double y_point = spline(x_point);
    // std::cout << "t:" << t_temp << " distance_covered:" << distance_covered;
    // std::cout << " distance_ratio:" << distance_ratio << " xPoint:" << x_point << std::endl;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = x_ref*cos(angle) - y_ref*sin(angle);
    y_point = x_ref*sin(angle) + y_ref*cos(angle);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
    // std::cout << "X:" << x_point << " Y:" << y_point << " vdot:" << v_dot << " v:" << v+v_dot*t_temp << std::endl;
  }




  // std::cout << "size:" << this->map_waypoints_x.size() << std::endl;

  std::vector<std::vector<double>> response;
  response.push_back(next_x_vals);
  response.push_back(next_y_vals);
  return response;
}
