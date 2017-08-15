/*
 * road.cpp
 *
 *  Created on: Aug 14, 2017
 *      Author: ggonzalez
 */

#include "road.h"
#include <vector>


using namespace std;

road::road() {
  // TODO Auto-generated constructor stub
  int num_lanes = 6;
  vector<double> speed_per_lane = {0.0,0.0,0.0,0.0,0.0,0.0};
  vector<int> vehicles_per_lane = {0,0,0,0,0,0};
  double max_speed = 49.5;
}

void road::avg_speed_lanes()
{
  double avg_speed = 0;

  for(int i=0; i<speed_per_lane.size(); ++i)
   {
    int num_cars = vehicles_per_lane[i];
    if (num_cars>0)
    {
      avg_speed = speed_per_lane[i]/num_cars;

    } else {
      avg_speed = max_speed;
    }

    speed_per_lane[i] = avg_speed;
   }

}




road::~road() {
  // TODO Auto-generated destructor stub
}

