/*
 * Snapshot.cpp
 *
 *  Created on: Aug 9, 2017
 *      Author: ggonzalez
 */

#include "Snapshot.h"
#include <string>

Snapshot::Snapshot(double lane, double s, double v, double a, string state) {
  _lane = lane;
  _s = s;
  _v = v;
  _a = a;
  _state = state;

}

Snapshot::~Snapshot() {}

