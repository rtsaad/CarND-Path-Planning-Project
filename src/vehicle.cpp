#include "vehicle.h"
#include <iostream>
#include <iostream>
#include <math.h>
#include "cost_function.h"



Vehicle::Vehicle(int lane, double target_speed){
    ref_speed = target_speed;
    ref_lane = lane;
}
  
void Vehicle::Update(double ax, double ay, double as, double ad, double ayaw, double aspeed, int lane, double target_speed, double delta){
  //update raw data
  x = ax;
  y = ay;
  s = as;
  d = ad;
  yaw = ayaw;
  speed = aspeed;
  delta_time = delta;

  ref_speed = target_speed;
  ref_lane = lane;

  //clean data
  _reset_data();
}

void Vehicle::_reset_data(){
  //clean data

  //reset trajectory
  trajectory.lane_start = ref_lane;
  trajectory.lane_end = ref_lane;
  trajectory.target_speed = ref_speed;

  //reset update
  update.ref_v = ref_speed;
  update.lane = ref_lane;
  update.target_v = 49.50;
  collider.collision = false;
  collider.distance = 10000;
  collider.closest_approach = 10000;
  collider.target_speed = 0;
}


void Vehicle::NextState(vector<vector<double>> sensor){
  States current_state = state;
  vector<States> states;
  states.push_back(KL);
  if(ref_lane != 0){
    //check if lane change is over before LCL again
    if(d<(2+4*(ref_lane)+2) && d>(2+4*(ref_lane)-2)){
      //inside lane
      std::cout << "INSIDE LANE\n";
      states.push_back(LCL);
    }
    states.push_back(PLCL);    
  }
  if(ref_lane != 2){
    if(d<(2+4*(ref_lane)+2) && d>(2+4*(ref_lane)-2)){
            std::cout << "INSIDE LANE\n";
      states.push_back(LCR);
    }
    states.push_back(PLCR);    
  }
  
  States min_state = KL;
  double  min_cost = 10000000;
  
  for(int i =0; i<states.size(); i++){
    States n_state = states[i];
    //prepare state
    _reset_data();
    _realise_state(n_state);
    std::cout << "STATE " << state << "\n";
    CostFunction cost = CostFunction(this, sensor);
    double value = cost.Compute();
    std::cout << "Cost Compare " << value << " < " << min_cost << "\n";
    if(value < min_cost){
      std::cout << "VALUE " << value << " " << n_state << "\n";
      min_state = n_state;
      min_cost = value;
    }
  }
  //update state
  std::cout << "##################### FINAL STATE " << min_state << "\n";
  state = min_state;
  if(state==PLCL || state==PLCR){
    std::cout << "$$$$$$$$$&&&&&&&&&&&*********((((((((())))))))) PLCL PLCR";
  }
  _reset_data();
  _realise_state(state);
  //update speed
  CostFunction cost = CostFunction(this, sensor);
  float v = cost.Compute();
  if(!collider.collision && speed < update.target_v && ref_speed < 49.5){
    update.ref_v += 0.224;
  } else if(collider.collision){
    
    std::cout << "#############################\nSpeeds " << collider.target_speed << " " << speed << "\n";
    if(abs(collider.target_speed - speed) > 2){
      std::cout << "REDUCE SPEED \n";
      update.ref_v -= 0.224;
    }
  }
  std::cout << state << " " << v << " " << update.ref_v << " " << collider.collision << " " << ref_speed << " " << update.ref_v  << "\n";
}

void Vehicle::_realise_state(States astate){
  state = astate;
  switch(state){
  case KL: {
    //same lane
    trajectory.lane_start = ref_lane;
    trajectory.lane_end = ref_lane;
    update.lane = ref_lane;
    break;    
  }
  case PLCL:{
    //same lane
    trajectory.lane_start = ref_lane;
    trajectory.lane_end = ref_lane - 1;
    update.lane = ref_lane;
    break;
  }
  case LCL:{
    //same lane
    trajectory.lane_start = ref_lane;
    trajectory.lane_end = ref_lane - 1;
    update.lane = ref_lane - 1;
    break;
  }    
  case PLCR:{
    //same lane
    trajectory.lane_start = ref_lane;
    trajectory.lane_end = ref_lane + 1;
    update.lane = ref_lane;
    break;
  }
  case LCR:{
    //same lane
    trajectory.lane_start = ref_lane;
    trajectory.lane_end = ref_lane + 1;
    update.lane = ref_lane + 1;
    break;
  } 
  default:
    std::cout << "STATE ERROR\n";
  }

  //check lane
  if(trajectory.lane_end < 0){
    trajectory.lane_end = 0;
  } else if( trajectory.lane_end > 2){
    trajectory.lane_end = 2;
  }

  if(trajectory.lane_start < 0){
    trajectory.lane_start = 0;
  } else if( trajectory.lane_start > 2){
    trajectory.lane_start = 2;
  }

  if(update.lane < 0){
    update.lane = 0;
  } else if(update.lane > 2){
    update.lane = 2;
  }
}
