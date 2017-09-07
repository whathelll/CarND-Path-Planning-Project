#include "gtest/gtest.h"
#include "../src/path_planner.h"
#include <iostream>
#include <fstream>

using namespace std;

PathPlanner buildPlanner() {
	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	std::vector<double> map_waypoints_x;
	std::vector<double> map_waypoints_y;
	std::vector<double> map_waypoints_s;
	std::vector<double> map_waypoints_dx;
	std::vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line)) {
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	PathPlanner planner(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
	return planner;
}




TEST(PathPlanner, init) {
	PathPlanner planner = buildPlanner();

	double car_x = 909.48;
	double car_y = 1128.67;
	double car_s = 124.834;
	double car_d = 6.16483;
	double car_yaw = 0;
	double car_speed = 0;
	std::vector<double> previous_path_x;
	std::vector<double> previous_path_y;
	double end_path_s = 0;
	double end_path_d = 0;
	int lane = 1;

	std::vector<std::vector<double>> response = planner.planPath(car_x, car_y,
	  car_s, car_d, car_yaw, car_speed,
	  previous_path_x, previous_path_y,
	  end_path_s, end_path_d, lane);

	std::vector<double> next_x_vals = response[0];
	std::vector<double> next_y_vals = response[1];

	// std::cout << "waypoints: " << std::endl;
	// for(int i = 0; i < next_x_vals.size(); i++) {
		// std::cout << next_x_vals[i] << ", " << next_y_vals[i] <<  std::endl;
	// }


	// std::cout << "Y: ";
	// for(int i = 0; i < next_y_vals.size(); i++) {
	// 	std::cout << next_y_vals[i] << "|";
	// }
	// std::cout << std::endl;

	EXPECT_TRUE(true);
}
