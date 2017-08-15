#ifndef VEHICLE_H_
#define VEHICLE_H_

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
#include <string>
#include <map>
#include "Snapshot.h"





using namespace std;

class Vehicle {
public:
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

  double preferred_buffer = 6.0;
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


  void configure(double speed_limit, int lanes_available, double max_acceleration);

  //TODO: Refactor comment
  /*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is 
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions 
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the 
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    """

  */
  void increment(int dt=1);
  void update_state(map<int,vector<map<string,double>>>&predictions);

  void restore_from_snapshot(Snapshot& snapshot);

  Snapshot take_snapshot();
  double max_accel_for_lane(map<int,vector<map<string,double>>>predictions, int lane, double s);

  void execute_constant_speed();
  void execute_init_ramp();
  void execute_keep_lane(map<int,vector<map<string,double>>>&predictions );
  void execute_lane_change(map<int,vector<map<string,double>>>&predictions , string dir);
  void execute_prep_lane_change(map<int,vector<map<string,double>>>&predictions , string dir);

  void execute_state(map<int,vector<map<string,double>>>&predictions);

  vector<Snapshot> _trajectory_for_state(string state,map<int,vector<map<string,double>>>&predictions, int horizon=5);

  string _get_next_state(map<int,vector<map<string,double>>>&predictions);



  /*
  *     Predicts state of vehicle in t seconds (assuming constant acceleration)
  */
  vector<double> state_at(double i);

  /*
  * Generate predictions of where other cars will be in a horizont.
  */
  vector<map<string,double>> generate_predictions(double horizont=10);



  /**
  * Destructor.
  */
  virtual ~Vehicle(){};


//private:

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
  double goal_s = 10000;




};

#endif /* CAR_H_ */
