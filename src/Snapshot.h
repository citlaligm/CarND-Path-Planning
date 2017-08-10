/*
 * Snapshot.h
 *
 *  Created on: Aug 9, 2017
 *      Author: ggonzalez
 */

#ifndef SNAPSHOT_H_
#define SNAPSHOT_H_
#include <string>

using namespace std;

class Snapshot {
public:
  int _lane;
  double _s;
  double _v;
  double _a;
  string _state;

  Snapshot(double lane, double s, double v, double a, string state);
  virtual ~Snapshot();
};

#endif /* SNAPSHOT_H_ */
