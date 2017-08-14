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
#include "cf.h"

#include <string>
#include <algorithm>
#include <iterator>
#include <tuple>
#include <map>




//double SPEED_LIMIT = 49.5;

Vehicle::Vehicle(){}

using namespace std;


void Vehicle::Init(double x, double y, double vx, double vy, double s, double d, double distance, double acceleration)
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
  	_speed_limit = NULL;
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
double Vehicle::get_speed_limit(){return _speed_limit;}
double Vehicle::get_max_acc(){return _max_acc;}


/*
 * Setters
 */

void Vehicle::set_lane(int lane){_lane = lane;}
void Vehicle::set_s(double s){_s = s;}
void Vehicle::set_velocity(double velocity){_v = velocity;}
void Vehicle::set_acc(double acc){_acc = acc;}
void Vehicle::set_state(string state){_state = state;}
void Vehicle::set_speed_limit(double speed){_speed_limit = speed;}

void Vehicle::configure(double speed_limit, int lanes_available, double max_acceleration)
{
  _speed_limit = speed_limit;
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


double Vehicle::max_accel_for_lane(map<int,vector<map<string,double>>>&predictions, int lane, double s)
{
  //cout<<"max_acc"<<endl;
  vector<map<string, double>>in_front;
  double delta_v_til_target = get_speed_limit() - get_velocity();
  double max_acc = min(get_max_acc(), delta_v_til_target);
  //cout<<predictions.size()<<endl;


  vector<map<string,double>> preds;


  for (auto p: predictions){
    //cout<<"for"<<endl;
    vector<std::map<string, double>> vec;
    int id = p.first;
    vec = p.second;
    //cout<<vec.size()<<endl;
    std::map<string, double> first_veh = vec[0];

    int s = first_veh.at("s");
    int lane = int(first_veh.at("lane"));

    if(get_lane() != lane || s < get_s())
      {
        predictions.erase(id);
        
      }
    } 
    double min_s = 100000;
    int index = 0;
    vector<std::map<string, double>> leading_vec;

    if(predictions.size()>0)
    {
      for (auto& x: predictions)
      {
        vector<std::map<string, double>> v;
        int id = x.first;
        v = x.second;
        std::map<string, double> map2 = v[0];

        double s = map2.at("s");
        int lane = map2.at("lane");

        double diff = s - get_s();
        if (diff < min_s)
          {
            min_s = s;
            leading_vec = v;
          }

      }

    }

    //cout<<"min: "<< min_s<<"\t my_s: "<<get_s()<<"\t diff: "<<min_s - s<<endl;
    
    // for(auto&v : leading_vec)
    // {
    //   std::map<string, double> map2 = v;
    //   int s = map2.at("s");
    //   int lane = map2.at("lane");
    //   //cout<<"s: "<<s<<"\t lane: "<<lane<<endl;
    // }
    //cout<<"size: "<<leading_vec.size()<<endl;
    if (leading_vec.size()>0){
      //cout<<leading_vec.size()<<endl;
      std::map<string, double> leading = leading_vec[1];
      double next_position = leading.at("s");
      double my_next_position = get_s() + get_velocity();
      double separation_next = next_position - my_next_position;
      double available_room = separation_next - preferred_buffer;
      max_acc = min(max_acc, available_room);
      //cout<<max_acc<<endl;
    }

  return max_acc;
}

void Vehicle::increment(int dt){
  set_s(get_s()+(get_velocity()*dt));
  set_velocity(get_velocity()+(get_acc()*dt));

}


void Vehicle::execute_constant_speed(){
  set_velocity(49.5);

}

void Vehicle::execute_keep_lane(map<int,vector<map<string,double>>> &predictions )
{
  set_acc(max_accel_for_lane(predictions, get_lane(), get_s()));
}

void Vehicle::execute_lane_change(map<int,vector<map<string,double>>> &predictions, string dir)
{
  int delta = -1;
  if(dir == "R"){delta = 1;}
  set_lane(get_lane()+ delta) ;
  set_acc(max_accel_for_lane(predictions, get_lane(), get_s()));
}

void Vehicle::execute_prep_lane_change(map<int,vector<map<string,double>>> &predictions, string dir)
{
  



}




void Vehicle::execute_state(map<int,vector<map<string,double>>> &predictions)
{
  string state = get_state();
  if (state == "CS") {execute_constant_speed();}
  else if (state == "KL") {execute_keep_lane(predictions);}
  else if (state == "LCL") {execute_lane_change(predictions, "L");}
  else if (state == "LCR") {execute_lane_change(predictions, "R");}
  else if (state == "PLCL") {execute_prep_lane_change(predictions, "L");}
  else if (state == "PLCR") {execute_prep_lane_change(predictions, "R");}


}


vector<Snapshot> Vehicle::_trajectory_for_state(string state,map<int,vector<map<string,double>>> &predictions, int horizon){
  Snapshot snapshot = take_snapshot();


  vector<Snapshot> trajectory = {snapshot} ;


  for (int i = 0; i < horizon; ++i)
  {
    restore_from_snapshot(snapshot);
    set_state(state);
    execute_state(predictions);
    increment();
    trajectory.push_back(take_snapshot());

    //remove first prediction for each vehicle
    for (auto p: predictions){
        //cout<<"for"<<endl;
        vector<std::map<string, double>> vec;
        int id = p.first;
        vec = p.second;
        vec.erase(vec.begin(),vec.begin()+1);
     }
   }

  restore_from_snapshot(snapshot);
  //cout<<"size traj: "<<trajectory.size()<<endl;
  return trajectory;
}

cf c_f;

string Vehicle::_get_next_state(map<int,vector<map<string,double>>> &predictions)
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

  vector<map<string,double>> costs;
  double a = max_accel_for_lane(predictions, get_lane(), get_s());

  //cout<<"a: "<<a<<endl;

  // //cout<<"***************"<<endl;
  // // for(auto state:states)
  // // {

  // //   cout<<state<<endl;

  // // }
  // //cout<<"***************"<<endl;


  for(auto state:states)
    {
      vector<Snapshot> trajectory = _trajectory_for_state(state, predictions);
      //cout<<"p_size: "<<predictions.size()<<endl;
      double cost = c_f.calculate_cost(*this, trajectory, predictions);
      //cout<<"state: "<<state<<"\tcost: "<<cost<<endl;
      map<string,double> state_cost;
      state_cost.emplace(state, cost);
      costs.push_back(state_cost);

    }

  double min_cost = pow(10,20);
  string min_state = get_state();

  for (auto v : costs)
  {
    // j is each std::pair<string,string> in each map
    for (auto s_c : v) 
    {
      string state = s_c.first; 
      double s_cost = s_c.second;
      if(s_cost < min_cost)
      {
        min_state = state;
        min_cost = s_cost;
      }
    } 
  }

  cout<<"min_state: "<<min_state<<"\tcost: "<<min_cost<<endl;
  return min_state;
}





//calculate state of car at certain time
vector<double> Vehicle::state_at(double t){
  double s =  get_s() +  get_velocity()*t + get_acc()*t*t/2.0;
  double v = get_velocity()+ get_acc()*t;
  //cout<<"lane_func: "<<get_lane()<<endl;
  return vector<double> {double(get_lane()), s, v, get_acc()};
}

//Generate predictions of where other cars will be in a horizon
std::vector<map<string,double>> Vehicle::generate_predictions(double horizon){
  std::vector<map<string,double>> predictions;
  //cout<<"lane_gen: "<<get_lane()<<endl;
   for(int i=0; i<horizon;i++){
     double lane = double(get_lane());//int(state_at(i)[0]);
     double s = state_at(i)[1];
     
     map<string,double> data;
     data.emplace("s",s);
     data.emplace("lane", lane);
     predictions.push_back(data);
   }
  return predictions;
}


void Vehicle::update_state(map<int,vector<map<string,double>>> &predictions){
  //cout<<"***update_state***"<<endl;
  string state = Vehicle::_get_next_state(predictions);
  Vehicle::set_state (state);

}
