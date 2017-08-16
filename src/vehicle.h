#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include <string>
#include <map>


using namespace std;

class Vehicle {
public:


  double _x;
  double _y;
  double _v;
  double _vx;
  double _vy;
  double _s;
  double _d;
  int _lane;
  double _acc;
  string _state;
  double _speed_limit;
  double _max_acc;
  int _lanes_available;


  /**
  * Constructor.
  */
  Vehicle();

  /**
 	The data format for each car looks like this, [ id, x, y, vx, vy, s, d]. 
 	The id is a unique identifier.
 	The x, y values are in global map coordinates.
 	The vx, vy values are the velocity components in reference to the global map. 
 	The s and d values are the Frenet coordinates for that car.
  **/


  //Vehicle(double x, double y, double v, double s, double d, double distance, double acceleration=0);

  //Vehicle();
  void Init(double x, double y, double vx, double vy, double s, double d, double distance, double acceleration=0);

  double get_s();
  string get_state();
  int get_lane();
  double get_acc();
  double get_velocity();
  double get_speed_limit();
  double get_max_acc();



  void set_lane(int lane);
  void set_s(double s);
  void set_velocity(double velocity);
  void set_acc(double acc);
  void set_state(string state);
  void set_speed_limit(double speed_limit);



  /**
  * Destructor.
  */
  virtual ~Vehicle(){};

};

#endif /* CAR_H_ */
