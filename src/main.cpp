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
#include "vehicle.h"
#include <map>
#include <iostream>
#include <string>
#include "road.h"

using namespace std;

// for convenience
using json = nlohmann::json;

int lane = 1;

double ref_vel = 0; //mph

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// For converting back and forth between mph and meters per second.
double mph2mtps(double x) { return x * 1609 /3600; }
double mtps2mph(double x) { return x * 3600 /1609; }

//Convert d to lane and back


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
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

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
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

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

vector<int> filter_predictions_by_lane(map<int,Vehicle>& vehicles, int current_lane)
{
  vector<int>id_vector;
  for(auto&v : vehicles)
  {
      int id = v.first;
      Vehicle boring_car = v.second;

      if (boring_car._lane == current_lane)
      {
        id_vector.push_back(id);
      }
  }

  return id_vector;
}

double check_collision(map<int,Vehicle>& vehicles, vector<int>ids_vector, Vehicle& AV, double distance, double current_s)
{
  double closest = 9999;
  for(auto&v : vehicles)
  {
    int id = v.first;
    Vehicle boring_car = v.second;

    if(std::find(ids_vector.begin(), ids_vector.end(), id)!=ids_vector.end())
    {
      double speed = boring_car._v;
      double s_start = boring_car._s;
      double s_end = s_start + distance * speed;

      double diff_start = fabs(s_start - AV._s);

      if(diff_start<closest){closest = diff_start;}

      double diff_end = fabs(s_end - AV._s);

      if(diff_end<closest){closest = diff_end;}

    }

  }
  return closest;
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  Vehicle autonomus_car;
  double speed_limit = 49.5;
  int lanes_available =  3;
  double max_acceleration = 0.5; 

  Vehicle old_car;


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &autonomus_car, &old_car](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    bool flag =true;





    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          


        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];


            double car_vx = car_speed*cos(car_yaw);
            double car_vy = car_speed*sin(car_yaw);
            
            /*
              --------------AV configuration---------------------
            */
            autonomus_car.Init(car_x, car_y, car_vx, car_vy, car_s, car_d, 0.0, 0.5);
            lane = autonomus_car.get_lane();
            /*
              --d ------------AV configuration---------------------
            */


            int prev_size = previous_path_x.size();
            map<int,Vehicle> vehicles;

            vector<double> speed_per_lane = {0.0,0.0,0.0,0.0,0.0,0.0};
            vector<int> vehicles_per_lane = {0,0,0,0,0,0};
            road simulator;
            if(prev_size > 0)
            {
              autonomus_car.set_s(end_path_s);
              car_s = autonomus_car.get_s();
            }


            bool too_close = false;

            /*
             --------------SENSOR FUSION---------------------
             */
            for(int i=0; i < sensor_fusion.size(); i++)
            {
              int id = sensor_fusion[i][0];
              double x = sensor_fusion[i][1];
              double y = sensor_fusion[i][2];
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double s = sensor_fusion[i][5]; 
              double d = sensor_fusion[i][6];  


              //cout<<"lane_d: "<<d<<endl;
              old_car.Init(x, y, vx, vy, s, d, 0.0, 0.0);

              vehicles.emplace(id,old_car);

              int old_lane = old_car._lane;

              //at the beginning there are some random numbers, so make sure it makes d value makes sense
              if(old_lane>=0 && old_lane <=2)
              {
                simulator.speed_per_lane[old_lane] += mtps2mph(old_car._v);
                simulator.vehicles_per_lane[old_lane] += 1;
              }
            }


            simulator.avg_speed_lanes();

            /*
            --------------SENSOR FUSION---------------------
            */


            //check which vehicles are in my path
            double closest_speed = autonomus_car._speed_limit;
            for(auto&v : vehicles)
            {
                int id = v.first;
                Vehicle boring_car = v.second;
                boring_car._s = boring_car._s + ((double)prev_size*0.02*boring_car._v);

                if(lane == boring_car._lane )
                {
                  if(boring_car._s > car_s && (boring_car._s - car_s) < 30.0 )
                  {
                    too_close = true;
                    closest_speed = boring_car._v;
                  }
                }
             }


            if (too_close)
             {

               //Remove states that are not possible.
               vector<string> states = {"KL","LCL","LCR"};
               if(lane == 0)
               {
                 auto p = find (states.begin(), states.end(),"LCL");
                 states.erase(p);
               }
               else if(lane == 2)
               {
                   auto p = find (states.begin(), states.end(),"LCR");
                   states.erase(p);
               }

               int best_lane = autonomus_car._lane;
               double best_cost = 1.79769e+308; //maximum number represented by a double
               string best_state ="";

                /*
                 * COST FUNCTIONS
                 */
               for(auto&state : states)
               {

                 int current_lane;
                 if (state == "KL"){current_lane = autonomus_car._lane;}
                 else if (state == "LCL"){current_lane = autonomus_car._lane -1;}
                 else if (state == "LCR"){current_lane = autonomus_car._lane + 1;}
                 else{current_lane = 100; cout<<"error"<<endl;}


                 double cost = 0.0;
                 double cost_change_lane = 0;
                 double cost_nearest = 0;

                 //Penalizes changes of lanes
                 if(autonomus_car._state != state)
                 {
                   cost_change_lane = 1000;
                   cost += cost_change_lane;
                 }


                 //Penalizes speeding
                 double speed_in_lane = simulator.speed_per_lane[current_lane];
                 double cost_speed = ((speed_in_lane - ref_vel)/speed_in_lane)*2;
                 double cost_speed_normalized = 2/(1.0+exp(-cost_speed))-1.0;
                 cost += cost_speed_normalized * 1000;


                 //Penalizes collision and distance less than the desired buffer
                 vector<int>ids_vector = filter_predictions_by_lane(vehicles,current_lane);

                 //Check if theres is a possible collision
                 double nearest = check_collision(vehicles,ids_vector, autonomus_car , 0.02*prev_size, autonomus_car._s);

                 double buffer = 10;
                 //If it's too close, better
                 if(nearest < buffer)
                 {
                   cost_nearest = pow(10,5);
                   cost+= cost_nearest;

                 }

                 double cost_collision_buffer = (2*buffer/nearest);
                 double cost_collision_buffer_normalized = 2/(1.0+exp(-cost_collision_buffer))-1.0;
                 cost += 1000*cost_collision_buffer_normalized;

                 //if current state has the lowest cost then choose it
                 if (cost< best_cost)
                 {
                   best_lane = current_lane;
                   best_cost = cost;
                   best_state = state;

                 }
               }
               cout<<"+++++++++++++++++++++++++++++++++++++++++"<<endl;
               cout<<"state: "<<best_state<<"  cost:"<<best_cost<<"  lane: "<<best_lane<<endl;


               //if the current state is the best and speed of the current lane is lower than the current speed or the speed of the car in front
               // SLOW DOWN!
               if(best_lane == lane && (ref_vel > simulator.vehicles_per_lane[lane] || closest_speed))
               {
                 ref_vel-= 0.5;
                 lane = best_lane;
               }

               //The best is to change lane
               lane = best_lane;
             }

             //Let's go. Nothing is going on, keep going at speed limit.
             else if(ref_vel < autonomus_car._speed_limit)
             {
               ref_vel += 0.5;
             }


            //create a list of widely spaced (x,y) waypoints, evenly distributed at 30m
            //later we will interpolate these waypoints with a spline and fill it with more points that control
            //
            std::vector<double> ptsx;
            std::vector<double> ptsy;

            //reference x,y, yaw state
            //either we will reference the starting point as where the car is or at the previous paths end point

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            //if previous path size is almost empty, use car as starting reference
            if(prev_size < 2)
            {
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsy.push_back(prev_car_y);

            }
            else
            {
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];

              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];

              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              //use two points that make the path tangent to the previous path's end point
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
            }

            //in Frenet add evenly 30m spaced pints ahead of the starting reference
            double space = 30.0;
            double target_lane = 2.0 + 4.0*lane; //4 meters width plus half lane

            std::vector<double> next_wp0 = getXY(car_s + space, target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            std::vector<double> next_wp1 = getXY(car_s + 2*space, target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            std::vector<double> next_wp2 = getXY(car_s+ 3*space, target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            
            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);


            for (int i = 0; i<ptsx.size(); i++)
            {
              //shift car reference angle to 0 degrees
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
              ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
              
            }

            //create spline
            tk::spline s;

            //set (x,y) points to the spline
            s.set_points(ptsx,ptsy);

          	json msgJson;

            
            //Define the actual (x,y) points we will use for the planner
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;



            //start with all of the previous path points from last time
            for(int i = 0; i < previous_path_x.size(); i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            //calculate how to break up spline points so that we travel at out desire reference velocity
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

            double x_add_on = 0;


            //fill up the rest of our our path planner after filling it with previous points, here we will always output
            //50 points


            for(int i = 0; i <= 50-previous_path_x.size(); i++)
            {
              //quantity of marks
              double N = target_dist/(0.02*mph2mtps(ref_vel));
              double x_point = x_add_on + (target_x)/N;

              //where in the spline is the mark
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              //rotate back to normal after rotating it earlier
              x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);


            }

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































