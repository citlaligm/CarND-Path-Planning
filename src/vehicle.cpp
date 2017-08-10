#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"
//#include "json.hpp"
//#include "spline.h"
#include "vehicle.h"
#include "Snapshot.h"

#include <string>
#include <algorithm>
#include <iterator>
#include <tuple>
#include <map>




using namespace std;
//Vehicle::Vehicle(){}


Vehicle::Vehicle(double x, double y, double vx, double vy, double s, double d, double distance, double acceleration)
  {
  	_x = x;
  	_y = y;
  	_vx = vx;
  	_vy = vy;
  	_v = sqrt(pow(vx,2)+pow(vy,2));
  	_s = s;
  	_d = d;

  	if (d >= 0.0 && d<4.0){_lane = 0;}
  	else if (d >= 4.0 && d<8.0){_lane = 1;}
  	else if (d >= 8.0 && d<=12.0){_lane = 2;}
  	else{_lane = 100;}
  	_acc = acceleration;
  	_state = "CS";
  	_target_speed = NULL;
  	_lanes_available = NULL;
  	_max_acc = NULL;




  }

//Vehicle::Vehicle(double x, double y, double vx, double vy, double s, double d, double distance, double acceleration=0)
//  {
//  	_x = x;
//  	_y = y;
//  	_vx = vx;
//  	_vy = vy;
//  	_v = sqrt(pow(vx,2)+pow(vy,2));
//  	_s = s;
//  	_d = d;
//  	if (d >= 0.0 && d<4.0){_lane = 0;}
//  	else if (d >= 4.0 && d<8.0){_lane = 1;}
//  	else if (d >= 8.0 && d<=12.0){_lane = 2;}
//  	_acc = acceleration;
//  	_state = "CS";
//
//  }


/*
 * Getters
 */
double Vehicle::get_s(){return _s;}
string Vehicle::get_state(){return _state;}
int Vehicle::get_lane(){return _lane;}
double Vehicle::get_acc(){return _acc;}
double Vehicle::get_velocity(){return _v;}
double Vehicle::get_target_speed(){return _target_speed;}
double Vehicle::get_max_acc(){return _max_acc;}


/*
 * Setters
 */

void Vehicle::set_lane(int lane){_lane = lane;}
void Vehicle::set_s(double s){_s = s;}
void Vehicle::set_velocity(double velocity){_v = velocity;}
void Vehicle::set_acc(double acc){_acc = acc;}
void Vehicle::set_state(string state){_state = state;}
void Vehicle::set_target_speed(double speed){_target_speed = speed;}

void Vehicle::configure(double target_speed, int lanes_available, double max_acceleration)
{
  _target_speed = target_speed;
  _lanes_available = lanes_available;
  _max_acc = max_acceleration;
}



void Vehicle::restore_from_snapshot(Snapshot snapshot)
{
  Snapshot s = snapshot;
  set_lane(s._lane);
  set_s(s._s);
  set_velocity(s._v);
  set_acc(s._a);
  set_state(s._state);

}

Snapshot Vehicle::take_snapshot(){
  return Snapshot(get_lane(),get_s(),get_velocity(), get_acc(),get_state());
}


double Vehicle::max_accel_for_lane(vector<map<string,double>>predictions, int lane, double s)
{
  vector<Vehicle>in_front;
  double delta_v_til_target = get_target_speed() - get_velocity();
  double max_acc = min(get_max_acc(), delta_v_til_target);
  for(auto pred: predictions)
  {
    double s = pred.at("s");
    int lane = pred.at("lane");


  }

  return 0.0;

}


void Vehicle::execute_constant_speed(){set_velocity(0.0);}
void Vehicle::execute_keep_lane(vector<map<string,double>>predictions){}
void Vehicle::execute_lane_change(vector<map<string,double>>predictions, string dir)
{
  int delta = -1;
  if(dir == "R"){delta = 1;}
  set_lane(get_lane()+ delta) ;




}
void Vehicle::execute_prep_lane_change(vector<map<string,double>>predictions, string dir){}




void Vehicle::execute_state(vector<map<string,double>>predictions)
{
  string state = get_state();
  if (state == "CS") {execute_constant_speed();}
  else if (state == "KL") {execute_keep_lane(predictions);}
  else if (state == "LCL") {execute_lane_change(predictions, "L");}
  else if (state == "LCR") {execute_lane_change(predictions, "R");}
  else if (state == "PLCL") {execute_prep_lane_change(predictions, "L");}
  else if (state == "PLCR") {execute_prep_lane_change(predictions, "R");}


}


void Vehicle::_trajectory_for_state(string state,vector<map<string,double>>predictions, int horizon){
  Snapshot snapshot = take_snapshot();


  vector<Snapshot> trajectory = {snapshot} ;


  for (int i = 0; i < horizon; ++i)
  {
    restore_from_snapshot(snapshot);
    set_state(state);




  }


}


string Vehicle::_get_next_state(map<int,vector<map<string,double>>> predictions)
{
  //cout<<"***_get_next_state***"<<endl;
  vector<string> states = {"KL","LCL","LCR"};
  int current_lane = get_lane();
  if(current_lane == 0){
    auto p = find (states.begin(), states.end(),"LCL");
    states.erase(p);


  }
  else if(current_lane == 2){
    auto p = find (states.begin(), states.end(),"LCR");
    states.erase(p);
  }

  vector<double> costs;
  cout<<"***************"<<endl;
  for(auto state:states)
  {

    cout<<state<<endl;

  }
  cout<<"***************"<<endl;


  //for(auto state:states){cout<<state<<endl;}
  return states[2];
}





//calculate state of car at certain time
vector<double> Vehicle::state_at(double t){
  double s =  get_s() +  get_velocity()*t + get_acc()*t*t/2.0;
  double v = get_velocity()+ get_acc()*t;
  return vector<double> {double(get_lane()), s, v, get_acc()};
}

//Generate predictions of where other cars will be in a horizon
std::vector<map<string,double>> Vehicle::generate_predictions(double horizon){
  std::vector<map<string,double>> predictions;
   for(int i=0; i<horizon;i++){
     int lane = int(state_at(i)[0]);
     double s = state_at(i)[1];
     map<string,double> data = {{"s",0},{"lane",0}};
     data.at("s") = s;
     data.at("lane") = double(lane);
     predictions.push_back(data);
   }
  return predictions;
}


void Vehicle::update_state(map<int,vector<map<string,double>>>  predictions){
  //cout<<"***update_state***"<<endl;
  string state = Vehicle::_get_next_state(predictions);
  Vehicle::set_state (state);

}
