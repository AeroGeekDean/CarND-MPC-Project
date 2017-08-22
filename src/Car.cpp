/*
 * Car.cpp
 *
 *  Created on: Aug 21, 2017
 *      Author: deanliu
 */

#include <math.h>
#include "Car.h"

Car::Car() {
  // TODO Auto-generated constructor stub

}

Car::~Car() {
  // TODO Auto-generated destructor stub
}

Car::Coord Car::map2body(Car::Coord in_map) {
  double dxm = in_map.x - myMapCoord.x;
  double dym = in_map.y - myMapCoord.y;
  Coord out_body;
  out_body.x = dxm*cos(myPsi) + dym*sin(myPsi);
  out_body.y =-dxm*sin(myPsi) + dym*cos(myPsi);
  return out_body;
}
