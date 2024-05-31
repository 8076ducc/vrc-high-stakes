#ifndef _8076_PURSUIT_ODOM_HPP_
#define _8076_PURSUIT_ODOM_HPP_
#include "point.hpp"

extern Point position;
extern double angle;
extern double bearing;

double degToRad(double deg);

double radToDeg(double rad);

double boundRad(double rad);

void setBearing(double bear);

void odometryControl(void *ignore);

#endif