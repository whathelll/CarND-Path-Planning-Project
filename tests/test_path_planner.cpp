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

double calculateDistance(double x1, double y1, double x2, double y2) {
	std::cout << "x:" << x1 << " x2:" << x2 << std::endl;
	std::cout << "y:" << y1 << " y2:" << y2 << std::endl;
	double xdiff = x1 - x2;
	double ydiff = y1 - y2;
	std::cout << "xdiff:" << xdiff << " ydiff:" << ydiff << std::endl;
	return sqrt(xdiff*xdiff + ydiff*ydiff);
}

TEST(PathPlanner, collision) {
	PathPlanner planner = buildPlanner();


	double car_x = 1583.14;
	double car_y = 1156.93;
	double car_s = 809.553;
	double car_d = 5.9743;
	double car_yaw = 355.868;
	double car_speed = 22.2561;
	std::vector<double> previous_path_x;
	std::vector<double> previous_path_y;
	double end_path_s = 831.369;
	double end_path_d = 6.25158;
	int lane = 1;

	previous_path_x.push_back(1583.586); previous_path_y.push_back(1156.903);
	previous_path_x.push_back(1584.03); previous_path_y.push_back(1156.871);
	previous_path_x.push_back(1584.474); previous_path_y.push_back(1156.839);
	previous_path_x.push_back(1584.918); previous_path_y.push_back(1156.807);
	previous_path_x.push_back(1585.362); previous_path_y.push_back(1156.775);
	previous_path_x.push_back(1585.806); previous_path_y.push_back(1156.743);
	previous_path_x.push_back(1586.25); previous_path_y.push_back(1156.711);
	previous_path_x.push_back(1586.694); previous_path_y.push_back(1156.679);
	previous_path_x.push_back(1587.138); previous_path_y.push_back(1156.647);
	previous_path_x.push_back(1587.582); previous_path_y.push_back(1156.615);
	previous_path_x.push_back(1588.026); previous_path_y.push_back(1156.583);
	previous_path_x.push_back(1588.47); previous_path_y.push_back(1156.551);
	previous_path_x.push_back(1588.914); previous_path_y.push_back(1156.519);
	previous_path_x.push_back(1589.358); previous_path_y.push_back(1156.487);
	previous_path_x.push_back(1589.802); previous_path_y.push_back(1156.455);
	previous_path_x.push_back(1590.246); previous_path_y.push_back(1156.423);
	previous_path_x.push_back(1590.69); previous_path_y.push_back(1156.391);
	previous_path_x.push_back(1591.134); previous_path_y.push_back(1156.359);
	previous_path_x.push_back(1591.578); previous_path_y.push_back(1156.327);
	previous_path_x.push_back(1592.022); previous_path_y.push_back(1156.295);
	previous_path_x.push_back(1592.466); previous_path_y.push_back(1156.263);
	previous_path_x.push_back(1592.91); previous_path_y.push_back(1156.231);
	previous_path_x.push_back(1593.354); previous_path_y.push_back(1156.199);
	previous_path_x.push_back(1593.798); previous_path_y.push_back(1156.167);

	// std::cout << "Sensor Fusion: [id, x, y, vx, vy, s, d]";
	std::vector<std::vector<double>> sensor_fusion;
	sensor_fusion.push_back({0, 1437.145, 1167.685, 19.7403, -2.58999, 663.6522, 10.01648});
	sensor_fusion.push_back({1, 1489.001, 1169.233, 14.72473, -1.87942, 714.9138, 1.979843});
	sensor_fusion.push_back({2, 1531.837, 1156.961, 15.72414, -1.209547, 758.4755, 10.08306});
	sensor_fusion.push_back({3, 1429.132, 1176.812, 19.66341, -2.789587, 654.5749, 1.964186});
	sensor_fusion.push_back({4, 1512.378, 1158.64, 15.58602, -1.533883, 739.1954, 10.0585});
	sensor_fusion.push_back({5, 1554.114, 1155.229, 15.37079, -1.308317, 780.967, 9.988347});
	sensor_fusion.push_back({6, 1487.291, 1161.665, 14.26104, 0.1989829, 714.1722, 9.702814});
	sensor_fusion.push_back({7, 1604.598, 1155.424, 18.14621, -1.338722, 831.2211, 5.959428});
	sensor_fusion.push_back({8, 1388.058, 1173.823, 19.60996, -1.888519, 613.9006, 9.966961});
	sensor_fusion.push_back({9, 1462.6, 1172.143, 12.16166, -2.524842, 688.3531, 2.418325});
	sensor_fusion.push_back({10, 1544.995, 1164.011, 15.60225, -1.108997, 771.0563, 2.048462});
	sensor_fusion.push_back({11, 1669.919, 1144.991, 17.76959, -1.081872, 896.994, 10.15911});

	std::vector<std::vector<double>> response = planner.planPath(car_x, car_y,
		car_s, car_d, car_yaw, car_speed,
		previous_path_x, previous_path_y,
		end_path_s, end_path_d, sensor_fusion, lane);

		std::vector<double> next_x_vals = response[0];
		std::vector<double> next_y_vals = response[1];

		std::cout << "waypoints: " << std::endl;
		for(int i = 0; i < next_x_vals.size()/2; i++) {
			std::cout << next_x_vals[i] << ", " << next_y_vals[i] <<  std::endl;
		}

		// double s = calculateDistance(next_x_vals[1], next_y_vals[1], next_x_vals[0], next_y_vals[0]);
		// double s_expected = car_speed + 0.5 * 10 * 0.02*0.02;
		// d = vt + 0.5at^2
		// v = v + at
		// double new_v = car_speed + 0.5 * a * 0.02*0.02;
		// a =
		// std::cout << "Dist travelled: " << s << " Expected: " << s_expected << std::endl;
		// EXPECT_TRUE(s <= s_expected);


		// s = calculateDistance(next_x_vals[2], next_y_vals[2], next_x_vals[0], next_y_vals[0])/0.02;
		// s_expected = car_speed + 0.5 * 10 * 0.04*0.04;
		// std::cout << "Dist travelled: " << s << " Expected: " << s_expected << std::endl;


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


		// std::cout << "Sensor Fusion: [id, x, y, vx, vy, s, d]";
		std::vector<std::vector<double>> sensor_fusion;
		sensor_fusion.push_back({0,1000.267,1144.747,19.38602,6.413828,216.0355,2.460715});
		sensor_fusion.push_back({1,1138.506,1184.167,14.11496,1.019762,362.3357,5.794508});
		sensor_fusion.push_back({2,937.3333,1125.296,18.7618,1.038109,151.6654,9.958471});
		sensor_fusion.push_back({3,1175.495,1190.462,17.13626,0.7762172,399.7549,1.881828});
		sensor_fusion.push_back({4,1173.929,1182.325,16.45432,0.7817263,397.8073,9.936078});
		sensor_fusion.push_back({5,1110.398,1177.318,15.06555,2.288945,333.2551,9.543228});
		sensor_fusion.push_back({6,974.1444,1129.928,18.33414,3.774951,187.6157,10.27336});
		sensor_fusion.push_back({7,1157.824,1185.469,14.93461,0.9289353,381.6972,5.958156});
		sensor_fusion.push_back({8,1142.51,1180.474,15.60957,1.087982,366.0486,9.780029});
		sensor_fusion.push_back({9,1005.769,1138.489,19.31873,7.203657,218.6866,10.36017});
		sensor_fusion.push_back({10,1180.635,1186.646,15.73803,0.6883883,404.7097,5.935398});
		sensor_fusion.push_back({11,958.7037,1135.268,17.82439,2.420322,173.9939,2.370735});

		std::vector<std::vector<double>> response = planner.planPath(car_x, car_y,
		  car_s, car_d, car_yaw, car_speed,
		  previous_path_x, previous_path_y,
		  end_path_s, end_path_d, sensor_fusion, lane);

		std::vector<double> next_x_vals = response[0];
		std::vector<double> next_y_vals = response[1];

		// std::cout << "waypoints: " << std::endl;
		// for(int i = 0; i < next_x_vals.size()/2; i++) {
		// 	std::cout << next_x_vals[i] << ", " << next_y_vals[i] <<  std::endl;
		// }

		double s = calculateDistance(next_x_vals[1], next_y_vals[1], next_x_vals[0], next_y_vals[0]);
		double s_expected = car_speed + 0.5 * 10 * 0.02*0.02;
		// d = vt + 0.5at^2
		// v = v + at
		// double new_v = car_speed + 0.5 * a * 0.02*0.02;
		// a =
		std::cout << "Dist travelled: " << s << " Expected: " << s_expected << std::endl;

		EXPECT_TRUE(s <= s_expected);

		s = calculateDistance(next_x_vals[2], next_y_vals[2], next_x_vals[0], next_y_vals[0])/0.02;
		s_expected = car_speed + 0.5 * 10 * 0.04*0.04;
		std::cout << "Dist travelled: " << s << " Expected: " << s_expected << std::endl;





	}
