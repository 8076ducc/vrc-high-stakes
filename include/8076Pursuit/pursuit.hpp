#ifndef _8076_PURSUIT_PURSUIT_HPP_
#define _8076_PURSUIT_PURSUIT_HPP_
#include <vector>
#include "point.hpp"
#include "path.hpp"

extern bool isPursuing;

void setVolts(double left, double right);

double autoAim();

void basePursue(std::vector<Point> iwaypoints, double iweightdata, double iweightsmooth, double ilookahead, bool ireverse = false);

void basePursue(double dist, double iweightdata, double iweightsmooth, double ilookahead);

void baseTurn(double bear, double min, double max);

void baseTurn(double x, double y, double min, double max);

void leftTurn(double bear, double min, double max);

void rightTurn(double bear, double min, double max);

void waitPursuit(double timer);

void waitTurn(double timer);

void baseControl(void *ignore);

#endif 