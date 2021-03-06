#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H
#include <iostream>
#include <vector>
#include <math.h>

class PathPlanner {
public:
  PathPlanner();
  PathPlanner(std::vector<double> map_waypoints_x, std::vector<double> map_waypoints_y, std::vector<double> map_waypoints_s, std::vector<double> map_waypoints_dx, std::vector<double> map_waypoints_dy) {
    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
    this->map_waypoints_s = map_waypoints_s;
    this->map_waypoints_dx = map_waypoints_dx;
    this->map_waypoints_dy = map_waypoints_dy;
  };

  virtual ~PathPlanner();

  std::vector<std::vector<double>> planPath(double car_x, double car_y,
    double car_s, double car_d, double car_yaw, double car_speed,
    std::vector<double> previous_path_x, std::vector<double> previous_path_y,
    double end_path_s, double end_path_d, std::vector<std::vector<double>> sensor_fusion, int lane);




private:
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;
  double distance(double x1, double y1, double x2, double y2)
  {
  	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
  }
  double deg2rad(double x) { return x * M_PI / 180; }
  double rad2deg(double x) { return x * 180 / M_PI; }
  int ClosestWaypoint(double x, double y, std::vector<double> maps_x, std::vector<double> maps_y)
  {

  	double closestLen = 100000; //large number
  	int closestWaypoint = 0;

  	for(int i = 0; i < maps_x.size(); i++)
  	{
  		double map_x = maps_x[i];
  		double map_y = maps_y[i];
  		double dist = distance(x,y,map_x,map_y);
  		if(dist < closestLen)
  		{
  			closestLen = dist;
  			closestWaypoint = i;
  		}

  	}

  	return closestWaypoint;

  }
  std::vector<double> getXY(double s, double d, std::vector<double> maps_s, std::vector<double> maps_x, std::vector<double> maps_y)
  {
  	int prev_wp = -1;

  	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  	{
  		prev_wp++;
  	}

  	int wp2 = (prev_wp+1)%maps_x.size();

  	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  	// the x,y,s along the segment
  	double seg_s = (s-maps_s[prev_wp]);

  	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  	double perp_heading = heading-M_PI/2;

  	double x = seg_x + d*cos(perp_heading);
  	double y = seg_y + d*sin(perp_heading);

  	return {x,y};

  }
  int NextWaypoint(double x, double y, double theta, std::vector<double> maps_x, std::vector<double> maps_y)
  {

  	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  	double map_x = maps_x[closestWaypoint];
  	double map_y = maps_y[closestWaypoint];

  	double heading = atan2( (map_y-y),(map_x-x) );

  	double angle = abs(theta-heading);

  	if(angle > M_PI/4)
  	{
  		closestWaypoint++;
  	}

  	return closestWaypoint;

  }
  std::vector<double> getFrenet(double x, double y, double theta, std::vector<double> maps_x, std::vector<double> maps_y)
  {
  	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  	int prev_wp;
  	prev_wp = next_wp-1;
  	if(next_wp == 0)
  	{
  		prev_wp  = maps_x.size()-1;
  	}

  	double n_x = maps_x[next_wp]-maps_x[prev_wp];
  	double n_y = maps_y[next_wp]-maps_y[prev_wp];
  	double x_x = x - maps_x[prev_wp];
  	double x_y = y - maps_y[prev_wp];

  	// find the projection of x onto n
  	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  	double proj_x = proj_norm*n_x;
  	double proj_y = proj_norm*n_y;

  	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  	//see if d value is positive or negative by comparing it to a center point

  	double center_x = 1000-maps_x[prev_wp];
  	double center_y = 2000-maps_y[prev_wp];
  	double centerToPos = distance(center_x,center_y,x_x,x_y);
  	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  	if(centerToPos <= centerToRef)
  	{
  		frenet_d *= -1;
  	}

  	// calculate s value
  	double frenet_s = 0;
  	for(int i = 0; i < prev_wp; i++)
  	{
  		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  	}

  	frenet_s += distance(0,0,proj_x,proj_y);

  	return {frenet_s,frenet_d};

  }

};




#endif
