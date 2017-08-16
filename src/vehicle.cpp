#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"
//#include "json.hpp"
//#include "spline.h"
#include "vehicle.h"

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
    double LANE_WIDTH = 4.0;
  	_x = x;
  	_y = y;
  	_vx = vx;
  	_vy = vy;
  	_v = sqrt(pow(vx,2)+pow(vy,2));
  	_s = s;
  	_d = d;

  	_lane = int(floor(_d/LANE_WIDTH));
  	_acc = acceleration;
  	_state = "KL";
  	_speed_limit = 49.5;
  	_lanes_available = 3;
  	_max_acc = 0.5;




  }


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

