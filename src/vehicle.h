#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>


using namespace std;

enum States { KL, LCL, LCR, PLCL, PLCR };

class Vehicle {
 public:

  States state = KL;

  struct update{
    double ref_v = 0;
    double target_v = 49.50;
    int lane = 0;
  } update;

  struct collider{
    bool collision = false;
    double distance = 0;
    double closest_approach = 1000;
    double target_speed = 0;
  } collider;

  struct trajectory{
    int lane_start = 0;
    int lane_end = 0;
    double target_speed=0;
  } trajectory;

  double ref_speed = 0;
  int ref_lane = 0;
  
  double x = 0;
  double y = 0;
  double s = 0;
  double d = 0;
  double yaw = 0;
  double speed = 0;

  double delta_time = 0;

  
  /**
  * Constructor
  */
  Vehicle(int lane, double target_speed);

  /*
   *
   */
  void Update(double ax, double ay, double as, double ad, double ayaw, double aspeed, int lane, double target_speed, double delta);
  
  void NextState(vector<vector<double>> sensor);

 private:
  void _realise_state(States state, vector<vector<double>> sensor_fusion);
  void _reset_data();

};

#endif
