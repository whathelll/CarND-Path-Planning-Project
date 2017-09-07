#include "gtest/gtest.h"
#include "../src/behavior_planner.h"
#include <fstream>

using namespace std;

BehaviorPlanner buildBehaviorPlanner() {
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

	BehaviorPlanner planner(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
	return planner;
}


TEST(BehaviorPlanner, Init) {
	BehaviorPlanner planner = buildBehaviorPlanner();

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
	double max_v = (50 / 0.62137) * 1000 / 3600;

	// std::cout << "Sensor Fusion: [id, x, y, vx, vy, s, d]";
	std::vector<std::vector<double>> sensor_fusion;
	sensor_fusion.push_back({0, 775.99, 1421.6, 0, 0, 6721.839, -277.6729});
	sensor_fusion.push_back({1, 775.8, 1425.2, 0, 0, 6719.219, -280.1494});
	sensor_fusion.push_back({2, 775.8, 1429, 0, 0, 6716.599, -282.9019});
	sensor_fusion.push_back({3, 775.8, 1432.9, 0, 0, 6713.911, -285.7268});
	sensor_fusion.push_back({4, 775.8, 1436.3, 0, 0, 6711.566, -288.1896});
	sensor_fusion.push_back({5, 775.8, 1441.7, 0, 0, 6661.772, -291.7797});
	sensor_fusion.push_back({6, 762.1, 1421.6, 0, 0, 6711.778, -268.0964});
	sensor_fusion.push_back({7, 762.1, 1425.2, 0, 0, 6709.296, -270.7039});
	sensor_fusion.push_back({8, 762.1, 1429, 0, 0, 6663.543, -273.1828});
	sensor_fusion.push_back({9, 762.1, 1432.9, 0, 0, 6660.444, -275.5511});
	sensor_fusion.push_back({10, 762.1, 1436.3, 0, 0, 6657.743, -277.6157});
	sensor_fusion.push_back({11, 762.1, 1441.7, 0, 0, 6653.453, -280.8947});

	int targetLane = planner.planLane(car_x, car_y,
	  car_s, car_d, car_yaw, car_speed,
	  previous_path_x, previous_path_y,
	  end_path_s, end_path_d, sensor_fusion, max_v);

	// std::cout << "Sensor Fusion: [id, x, y, vx, vy, s, d]";
	// for(int i = 0; i < sensor_fusion.size(); i++) {
	// 	std::cout << std::endl << "[";
	// 	for(auto val : sensor_fusion[i]) {
	// 		std::cout << val << ",";
	// 	}
	// 	std::cout << "]";
	// }
	// std::cout << std::endl;

	EXPECT_TRUE(targetLane==1);
}


TEST(BehaviorPlanner, collision) {
	BehaviorPlanner planner = buildBehaviorPlanner();

	double car_x = 1032.05;
	double car_y = 1154.2;
	double car_s = 249.058;
	double car_d = 5.99958;
	double car_yaw = 22.2076;
	double car_speed = 14.9634;
	std::vector<double> previous_path_x;
	std::vector<double> previous_path_y;
	double end_path_s = 263.741;
	double end_path_d = 5.99995;
	double max_v = (50 / 0.62137) * 1000 / 3600;

	// std::cout << "Sensor Fusion: [id, x, y, vx, vy, s, d]";
	std::vector<std::vector<double>> sensor_fusion;
	sensor_fusion.push_back({0, 952.1817, 1134.414, 17.74544, 2.059165, 167.4176, 2.510718});
	sensor_fusion.push_back({1, 1191.069, 1183.093, 16.56019, 0.7313032, 414.9735, 9.968277});
	sensor_fusion.push_back({2, 991.1567, 1133.893, 18.03671, 4.611553, 205.0838, 10.21543});
	sensor_fusion.push_back({3, 1119.949, 1178.554, 15.13209, 1.720415, 342.885, 9.547203});
	sensor_fusion.push_back({4, 1156.551, 1181.393, 16.76942, 1.023441, 380.119, 9.926159});
	sensor_fusion.push_back({5, 1015.124, 1142.319, 17.30066, 7.380551, 228.7945, 10.44951});
	sensor_fusion.push_back({6, 956.2703, 1127.047, 19.94033, 2.408248, 170.6805, 10.27851});
	sensor_fusion.push_back({7, 1205.458, 1187.727, 17.88137, 0.764839, 429.5611, 6.004757});
	sensor_fusion.push_back({8, 1174.92, 1190.438, 17.55774, 0.8113426, 399.1795, 1.878143});
	sensor_fusion.push_back({9, 911.4869, 1124.844, 15.27545, 0.09530624, 126.8074, 10.0076});
	sensor_fusion.push_back({10, 1152.687, 1185.138, 15.0628, 0.9971056, 376.5489, 5.89957});
	sensor_fusion.push_back({11, 993.0543, 1142.567, 18.77731, 5.1254, 208.874, 2.185807});

	int targetLane = planner.planLane(car_x, car_y,
	  car_s, car_d, car_yaw, car_speed,
	  previous_path_x, previous_path_y,
	  end_path_s, end_path_d, sensor_fusion, max_v);

	// std::cout << "Sensor Fusion: [id, x, y, vx, vy, s, d]";
	// for(int i = 0; i < sensor_fusion.size(); i++) {
	// 	std::cout << std::endl << "[";
	// 	for(auto val : sensor_fusion[i]) {
	// 		std::cout << val << ",";
	// 	}
	// 	std::cout << "]";
	// }
	// std::cout << std::endl;

	EXPECT_TRUE(targetLane==1);
}



TEST(BehaviorPlanner, collision_from_behind) {
	BehaviorPlanner planner = buildBehaviorPlanner();

 	double car_x = 1019.91;
	double car_y = 1149.15;
	double car_s = 235.855;
	double car_d = 5.99933;
	double car_yaw = 22.8353;
	double car_speed = 15.0379;
	std::vector<double> previous_path_x;
	std::vector<double> previous_path_y;
	double end_path_s = 250.249;
	double end_path_d = 5.99901;
	double max_v = (50 / 0.62137) * 1000 / 3600;

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

	int targetLane = planner.planLane(car_x, car_y,
	  car_s, car_d, car_yaw, car_speed,
	  previous_path_x, previous_path_y,
	  end_path_s, end_path_d, sensor_fusion, max_v);

	// std::cout << "Sensor Fusion: [id, x, y, vx, vy, s, d]";
	// for(int i = 0; i < sensor_fusion.size(); i++) {
	// 	std::cout << std::endl << "[";
	// 	for(auto val : sensor_fusion[i]) {
	// 		std::cout << val << ",";
	// 	}
	// 	std::cout << "]";
	// }
	// std::cout << std::endl;

	EXPECT_TRUE(targetLane==1);
}
