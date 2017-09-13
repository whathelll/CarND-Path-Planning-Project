# Model Reflection
I decided to split my car model for path generation into 2 separate files. BehaviorPlanner and PathPlanner, mainly so I can write code and some simple tests without having to run the simulator. In hindsight they both could have been in the same class since all the helper functions I had to extract from main.cpp was the same. I also had to repeat some calculations in the PathPlanner as it needed to look at sensor data and make sure velocity could match cars in front.

## BehaviorPlanner ([behavior_planner.cpp](src/behavior_planner.cpp))
The BehaviorPlanner's role was to determine what lane we should be in. the outcome of this is to feed a target lane to the path planner.

I start by generating a vector of doubles to store calculations for each lane scenario with the cost being the last value. It turned out I didn't need most of the values other than car_speed and cost. I haven't cleaned this up yet.

```c++
std::vector<std::vector<double>> vehicle_states;
for (int lane = laneMin; lane <= laneMax; lane++) {
  double d = 2.0+lane*4;
  std::vector<double> xy = getXY(car_s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  //put cost as last items and as 0 for now
  std::vector<double> v {1.0*lane, xy[0], xy[1], car_s, d, car_speed, 0.0};
  vehicle_states.push_back(v);
}
```


For each of the above state I loop through the sensor fusion (loop is more efficient the other way around but it's easier for logging). If the other car's lane matched the vehicle state then we check for cost conditions to assess how desirable this lane is.

If the car is within 5 meters of our car we add a collision_cost of 300. This is used to stop us from changing lanes and hitting them. I believe this became redundant after I added the unclear lane calculation that varies based on distance. While I haven't tweaked that to take into consideration cars slightly behind us I'm keeping it.
```c++
//calculate collision cost
if(otherCarOldS >= car_s-7 && (otherCarOldS) <= (car_s + 10)) {
  vehicle_state[6] += collision_cost;
}
```


Look ahead about 2.5 seconds at max speed to find clear lanes. Use a cost function that heavily penaltises lanes with cars closer in front of us. In hindsight I should have implemented this earlier and it would have saved me mucking around trying different distance horizons.
```c++
if(otherCarOldS > car_s && (otherCarS) <= (end_path_s + max_v*2.5)) {
  laneUnclear = true; //mark lane as not being clear
  double distance = (otherCarOldS-car_s)/15; //make it a ratio of 30 so 30/30 gives 1.57... 2 gives 1.15, 4*30/30 gives 1
  double cost = 1 / (1-exp(-distance)) * 100; // so 2 gives 115 cost
  if(distance > 0 && cost > laneUnclearCost) laneUnclearCost = cost;
}
```

Save the slowest speed so we can calculate a velocity cost for the lane later.
```c++
//record velocity if other car is in sight and is slower
if(otherCarOldS >= car_s && (otherCarS) <= (end_path_s + max_v*2.5)) {
  if(otherCarVs < vehicle_state[5]) {
    vehicle_state[5] = otherCarVs;  //record this speed so we can calculate velocity cost
  }
  std::cout << " VELOCITY";
}
```
velocity cost is calculated as
```c++
if(vehicle_state[5] < max_v) {
  velocity_cost = (max_v - vehicle_state[5]) / max_v;
}
vehicle_state[6] += velocity_cost*100;
```

Once the cost for each potential lane is calculated, we determine if we should change lanes or not. If changing lanes will cause a collision then we'll stay in our lane.



## PathPlanner ([path_planner.cpp](src/path_planner.cpp))
I started off using the spline library as show in the project walkthrough, I found that to be a sufficient for generating a smooth path.

I added to this the conecpt of required acceleration, "v_dot", based on the difference between my target velocity (max_v) over time and limiting this to be no larger than max_v_dot.

```c++
int num_paths_to_calculate = 50 - path_size;
double t = (num_paths_to_calculate*0.02);
v_dot = (max_v - v) / t;
if(v_dot > 0 && v_dot > max_v_dot) v_dot = max_v_dot;
if(v_dot < 0 && v_dot < -max_v_dot) v_dot = -max_v_dot;
```

Once the v_dot is determined, I used the forumla s = v*t + 1/2at^2 to gauge where the car should end up and then I proportion the points along the spline.
```c++
// generate the additional points
for(int i=1; i<=50-path_size; i++) {
  double t_temp = i*0.02;
  double distance_covered = v * t_temp + 0.5 * v_dot * t_temp*t_temp;
  double distance_ratio = (distance_covered/target_dist); // (0.002/5) (5/30)
  double x_point = target_x*distance_ratio;
  double y_point = spline(x_point);

  double x_ref = x_point;
  double y_ref = y_point;

  x_point = x_ref*cos(angle) - y_ref*sin(angle);
  y_point = x_ref*sin(angle) + y_ref*cos(angle);

  x_point += ref_x;
  y_point += ref_y;

  next_x_vals.push_back(x_point);
  next_y_vals.push_back(y_point);
}
```

To make sure I don't hit other cars I modify the max_v to match cars within the short range in the current lane

```c++
for(int oc_index = 0; oc_index < sensor_fusion.size(); oc_index++) {
  std::vector<double>& otherCar = sensor_fusion[oc_index];
  double& otherCarOldS = otherCar[5];
  double otherCarLane = ceil(otherCar[6]/4)-1;
  double otherCarVx = otherCar[3];
  double otherCarVy = otherCar[4];
  double otherCarV = sqrt(otherCarVx*otherCarVx + otherCarVy*otherCarVy);

  //predict the future D 1 second later, on rare occassions they make dangerous lane changes
  double newX = otherCar[1] + otherCar[3];
  double newY = otherCar[2] + otherCar[4];
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
```
