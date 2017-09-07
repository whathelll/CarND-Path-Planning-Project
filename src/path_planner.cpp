#include "path_planner.h"

PathPlanner::PathPlanner() {}

PathPlanner::~PathPlanner() {}

std::vector<std::vector<double>> PathPlanner::planPath(double car_x, double car_y,
  double car_s, double car_d, double car_yaw, double car_speed,
  std::vector<double> previous_path_x, std::vector<double> previous_path_y,
  double end_path_s, double end_path_d, int lane) {

  car_speed = car_speed  / 0.62137 * 1000 / 3600;  // meters per second
  // max speed in meters  = ~22m/s
  double max_v = (50 / 0.62137) * 1000 / 3600;
  // max acceleration 10m/s
  double max_v_dot = 10;


  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  int path_size = previous_path_x.size();
  double timeStep = 0.02;

  // double v_dot;
  // double v;
  double s;
  double angle;
  std::vector<double> ref_path_x;
  std::vector<double> ref_path_y;
    // s = vt + 1/2 * a * t^2
  double dist_inc = 0.43;

  if(path_size == 0) {
    // v = car_speed;
    s = car_s;
    end_path_s = car_s;
    end_path_d = car_d;
    angle = deg2rad(car_yaw);
    // ref_path_x.push_back(previous_path_x);
    // ref_path_y.push_back(previous_path_y);
    // ref_path_x.push_back(car_x);
    // ref_path_y.push_back(car_y);
  } else {
    double pos_x = previous_path_x[path_size-1];
    double pos_y = previous_path_y[path_size-1];

    double pos_x2 = previous_path_x[path_size-2];
    double pos_y2 = previous_path_y[path_size-2];
    angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
    s = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y)[0];
    // ref_path_x.push_back(pos_x2);
    // ref_path_y.push_back(pos_y2);
    // double s2 = getFrenet(pos_x2, pos_y2, angle, map_waypoints_x, map_waypoints_y)[0];
    // v = (s - s2) / timeStep;
  }
  //
  // // add 2 more ref points
  // vector<double> ref_pt1 = getXY(s + 15.0, 2+lane*4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  // vector<double> ref_pt2 = getXY(s + 30.0, 2+lane*4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  // ref_path_x.push_back(ref_pt1[0]);
  // ref_path_y.push_back(ref_pt1[1]);
  // ref_path_x.push_back(ref_pt2[0]);
  // ref_path_y.push_back(ref_pt2[1]);
  //
  // // use the 2nd point as point of where the car will be. shift points to
  // // use car as centre
  // double ref_x = ref_path_x[1];
  // double ref_y = ref_path_y[1];
  // for(int i = 0; i < ref_path_x.size(); i++) {
  //   double shift_x = ref_path_x[i] - ref_x;
  //   double shift_y = ref_path_y[i] - ref_y;
  //   ref_path_x[i] = (shift_x * cos(0-angle) - shift_y*sin(0-angle));
  //   ref_path_y[i] = (shift_x * sin(0-angle) + shift_y*cos(0-angle));
  // }
  // std::cout << "RefX:" << ref_path_x[0] << " RefY:" << ref_path_y[0] << std::endl;
  // tk::spline spline;
  // spline.set_points(ref_path_x, ref_path_y);


  for(int i = 0; i < path_size; i++) {
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
  }


  // generate the additional points
  for(int i=0; i<50-path_size; i++) {
    double next_s = end_path_s + (i+1)*dist_inc;
    double next_d = 2+lane*4;
    std::vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    next_x_vals.push_back(xy[0]);
    next_y_vals.push_back(xy[1]);
    //std::cout << "S:" << s << " nextS:" << next_s << " vdot:" << v_dot << " v:" << v+v_dot*timeStep << std::endl;
  }





  // std::cout << "size:" << this->map_waypoints_x.size() << std::endl;

  std::vector<std::vector<double>> response;
  response.push_back(next_x_vals);
  response.push_back(next_y_vals);
  return response;
}
