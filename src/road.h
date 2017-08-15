/*
 * road.h
 *
 *  Created on: Aug 14, 2017
 *      Author: ggonzalez
 */

#ifndef ROAD_H_
#define ROAD_H_

#include <vector>
using namespace std;


class road {
public:

  int num_lanes = 6;
  vector<double> speed_per_lane = {0.0,0.0,0.0};
  vector<int> vehicles_per_lane = {0,0,0};
  double max_speed = 49.5;
  road();
  virtual ~road();


  /*
   *Calculate the average speed of the cars in each lane
  */
  void avg_speed_lanes();

};

#endif /* ROAD_H_ */
