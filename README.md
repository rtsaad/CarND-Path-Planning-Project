# Path Planning for the Udacity Simulator

This project consists of a c++ implementation of a Path Planner to control the car using the Udacity self-driving simulator. The main goal of this project is to develop a c++ path planner that successfully navigates the vehicle around the virtual highway (Udacity simulator) without collide with any other vehicle, respecting the speed limit of 50 mph and with maximum acceleration and jerk of 10 m/s² and 50 m/s³, respectively. Figure 1 depicts an example of the car being controlled by the controller. 
![alt text][image1]



[//]: # (Image References)

[image1]: images/simulator.gif "Path planning working with the Simulator."
[image2]: images/behavior.png "Finite State Machine."
[image3]: images/spline.png "Spline equidistant points."

## 1.Access 

The source code for this project is available at [project code](https://github.com/rtsaad/CarND-Path-Planning-Project).

## 2.Files

The following files are part of this project:  
* main.cpp:  main file that integrates the controller with the simulator;
* vehicle.cpp: vehicle class which defines the Finite State Machine;
* cost_function.cpp: cost_function class to compute the optimun path;


### 2.1 Dependency

This project requires the following packages to work:
* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## 3.How to use this project

To run this project, you first need to compile the code. After the code is compiled, please, run the Udacity simulator and the path planner binary created at the build folder.

### 3.1 Compiling and Running

The main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./path_planning
6. Run the Udacity Simulator (./term3_simulator)

## 4 Behavior planning

The behavior planner is responsible to generate the path and the speed to be followed by the car. From a high-level perspective, the planner has to decide if the car should stay or change lane. From a low level, the planer has to generate the map's coordinates (path) to be followed and decide at which speed the car should run. In order to achieve such a behavior, our self-driving car implements a Finite State Machine (FMS) with 5 states, which the transition is realized through the minimization of a cost function. Finally, the path is generated using the spline math tool

### 4.1 Finite State Machine

Figure 2 depicts our FSM for our behavior planner. The FSM starts with the Keep Lane (KP) state. Depending on the system context (highway), the FSM may stay at KP state or change to Prepare to Change Lane Left or Right. At each state, all possible states are evaluated using a cost function and the state with the minimum cost is selected. The FSM machine works as follow:

* At initial state (KL) and according to the cost function, the FSM can stay at the same state or prepare to change lanes (PLCL  or  PLCR). However, only the possible lane change (PLCL or PLCR) is available if the car is in one of the lateral lanes (lanes 0 or 2); The car will stay in the same lane (KL) if there is no other vehicle that prevents it from reaching the maximum legal speed limit of the road.

* If the PLCL or PLCR are selected, the car prepares to change lane. The preparation checks if the car speed and the buffer space are safe to change lance. The car may stay in the PLC_ state until the buffer is safe enough to change lane or even decide to return to state KL (same lane) if the cost to change the lane is no longer relevant;

* When there is enough buffer space to change lane, the FSM will transition to LCL/LCR states. The FSM returns to state KL as soon the lane change is over (car is between the lane lines).

![alt text][image2]

#### 4.1.2 Cost Function

The cost function evaluates the cost for the FSM to change state. It evaluates different metrics trying to identify the next optimal state. Below, the metrics and how they are evaluated  are presented in detail:

1. Change Lane: 

The change lane cost function adds a "comfort" constant penalty if the vehicle decides to change lane (see file 'cost_function.cpp' lines 41-51). 

```cpp
if(start_lane != end_lane){
    cost += COMFORT;
  }
```

2. Buffer:

The buffer cost function computes how long it has to the vehicle in front. It is computed by dividing the distance from the vehicle in front by the current speed of the ego car. Note that the cost is smaller if the vehicle in question is behind. It helps the ego car to choose a lane with no traffic in the front (see file 'cost_function.cpp' lines 80-101). .

```cpp
double time_steps = abs(vehicle->collider.closest_approach)/(abs(vehicle->speed)*MPH_TO_MS);  
 
if(time_steps > DESIRED_BUFFER){
  return 0;
}

double multiplier = 1.0 - pow((time_steps / DESIRED_BUFFER),2);
cost = multiplier * DANGER;
if(vehicle->collider.closest_approach < 0){
  //car in the back
  cost /= 10;
}      
```

3. Inefficiency:

This function evaluates the vehicle speed defined in this state in relation to the maximum speed allowed. States with speeds closer to the maximum speed are more efficient (lower cost), in contrast, states with lower speeds are less efficient (higher cost).

```cpp
double diff = (49.5 - vehicle->update.target_v)/49.5;
cost = pow(diff,2) * EFFICIENCY;
```

4. Target:

The target cost evaluates the speed comparison between the ego car and the vehicle in front (possible collision). If all lanes are blocked (possible collision), this function helps the car to choose a lane which the speed of the vehicle in question matches closer to the ego car (see file 'cost_function.cpp' lines 29-39). . 

```cpp
if(!vehicle->collider.collision){
  //no possible collision, no cost
  return 0;
}
double diff = (vehicle->collider.target_speed - vehicle->speed)/vehicle->collider.target_speed;
cost = pow(diff,2) * EFFICIENCY;
```

5. Collision: 

The collision cost is the most important function. It strongly penalizes the states which the risk of collision is more imminent. However, in order to force the car to escape from heavy traffic, the collision cost is smaller whenever is safe to change lane. We found out that it helps the car to find a more appropriate situation, instead of just following the car ahead until it opens a passageway (see file 'cost_function.cpp' lines 61-77). .

```cpp
double time_to_collide = abs(vehicle->collider.distance)/(abs(vehicle->speed)*MPH_TO_MS);
cost = exp(-pow(time_to_collide,2))*COLLISION;
if(vehicle->trajectory.lane_end != vehicle->trajectory.lane_start){
  if(time_to_collide > DESIRED_BUFFER){
    //safe to change lane
  cost /= 10;
  }
} 
```

### 4.3 Path Generation and Speed control

The FSM defines, from a high-level point of view, the lane and the speed to be followed for every state. For instance, if the car is changing lane, the FSM will return a different one from the current lane. The same is for speed, the FSM will return for every state the target speed the ego car should follow (see file 'main.cpp' lines 255-260). . 

```cpp
vehicle.Update(car_x, car_y, car_s, car_d, car_yaw, car_speed, lane, ref_vel, prev_size*.02);
vehicle.NextState(sensor_fusion);
// new lane
lane = vehicle.update.lane;
// target speed
ref_vel = vehicle.update.ref_v;		
```

After the FSM returns the lane and speed to follow, the controller generates up to 50 map coordinates back to the simulator. These points define the path to be followed by the car.

#### 4.3.1 Acceleration and Jerk control 

The requirements of this project state that the acceleration and jerk should not exceed 10 m/s² and 50m/s³, respectively. In order to meet this requirement, the car acceleration is increased or decreased by steps of 0.224 m/s². The limit 0.224 is computed as follows: max acceleration of 10 m/s², with delta time of 0.02 seconds in miles per hour:  ![equation](http://latex.codecogs.com/gif.latex?%5Cfrac%7B2.24%7D%7B10%7D)

```cpp
if(!collider.collision && ref_speed < update.target_v && ref_speed < 49.5){
  update.ref_v += 0.224;
} else if(ref_speed > update.target_v && ref_speed > 0){
  update.ref_v -= 0.224;
}
```

#### 4.3.2 Path: Spline

Spline is a piecewise "polynomial" parametric curve. They are popular for their simplicity of use and accuracy. Our path planner uses the Spline mathematical function for curve fitting the generated map coordinates. The spline helps to define a smooth path for the car (see file 'main.cpp' lines 260-372). . 

The path generation is an elaborate set of tasks. First, our planner has to generate equally spaced map coordinates. We use the helper function "getXY" to generate points from Freenet to Cartesian coordinates. 

```cpp
//In Frenet
vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);

ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);
```

After, we shift the orientation to the ego car for simplicity and feed the points to the spline generator. We used the [Cubic Spline library ](http://kluge.in-chemnitz.de/opensource/spline/) to generate the spline curve.

```cpp

for(int i=0; i<ptsx.size(); i++){
  //shift car reference angle to 0 degrees
  double shift_x = ptsx[i] - ref_x;
  double shift_y = ptsy[i] - ref_y;

  ptsx[i] = (shift_x *cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
  ptsy[i] = (shift_x *sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
}

//create a spline
tk::spline s;

//set (x,y) points to the spline
s.set_points(ptsx, ptsy);
```

After, with the spline function already done, we have to recompute the map points back from the curve. This task is accomplished by breaking up the spline into equidistant points that respect the desired speed (see Figure X). 

![alt text][image3]

Considering the time interval of 20 ms, the travel distance of 30 meters on x-axis with the speed "ref_speed", we have:

```cpp
//calculate how to break up spline points so that we travel at our desired reference velocity
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt((target_x) * (target_x) + (target_y)*(target_y));

//N distance
double N = (target_dist/(.02*ref_vel));
```

Finally, the final part is to compute the coordinates from spline and shift its orientation back.

```cpp

x_add_on = x_point;
for(int i = 0; i < 50-previous_path_x.size(); i++){
  //map X coordinate distance for every spline point
  double x_point = x_add_on + (target_x)/N;
  double y_point = s(x_point);
  //increment for next spline point
  x_add_on = x_point;

  double x_ref = x_point;
  double y_ref = y_point;

  //rotate back to normal after rotation it earlier
  x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
  y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

  x_point += ref_x;
  y_point += ref_y;

  //return to simulator
  next_x_vals.push_back(x_point);
  next_y_vals.push_back(y_point);
}

```






