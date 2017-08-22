/*
 * Car.h
 *
 *  Created on: Aug 21, 2017
 *      Author: deanliu
 */

#ifndef CAR_H_
#define CAR_H_

class Car {
 public:

  struct Coord {
    double x;
    double y;
  };

  Car();

  virtual ~Car();

  Car::Coord map2body(Car::Coord in_map);

  void set_location(Car::Coord loc_in);
  void set_psi(double psi_in);

  Car::Coord get_location();
  double get_psi();

 private:

  // car's own state (in map coordinate system)
  Coord myMapCoord; // [m], (+) east/north
//  double myX;   // [m], (+) east
//  double myY;   // [m], (+) north
  double myPsi; // [rad], (+) CCW from east
  double myV;   // [mph], (+) forward

};

inline void Car::set_location(Car::Coord loc_in) {
  myMapCoord.x = loc_in.x;
  myMapCoord.y = loc_in.y;
}

inline void Car::set_psi(double psi_in) {
  myPsi = psi_in;
}

inline Car::Coord Car::get_location() {
  return myMapCoord;
}

inline double Car::get_psi() {
  return myPsi;
}
#endif /* CAR_H_ */
