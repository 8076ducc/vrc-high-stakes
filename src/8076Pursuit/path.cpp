#include "8076Pursuit/path.hpp"
#include "main.h"
#include "pursuit.hpp"
#include <algorithm>

//most constants here r scuffed 

double k = 0.02 ; //max curvature of path

double maxVelRPM = 600.0;
double maxAccelRPM = 0.7; //lmao

double globalMaxVel = maxVelRPM * RPMToInPerMillis;
double globalMaxAccel = maxAccelRPM * RPMToInPerMillis;

#define spacing 1 //spacing between each point
#define tolerance 0.001 //tolerance for distance between each point

void setMaxRPM(double rpm) {
    maxVelRPM = rpm;
    globalMaxVel = maxVelRPM * RPMToInPerMillis;
}

void setMaxAccel(double accel) {
    maxAccelRPM = accel;
    globalMaxAccel = maxAccelRPM * RPMToInPerMillis;
}

Path::Path(): waypoints{} {}

Path::Path(std::vector<Point> iwaypoints): waypoints(iwaypoints) {}

Point Path::getSmoothedWaypoints(int i) {
    return smoothedWaypoints[i];
}

double Path::getMaxVel(int i) {
    return maxVel[i];
}

double Path::getTargetVel(int i) {
    return targetVel[i];
}

int Path::getLength() {
    return length;
}

double Path::getLookAhead() {
    return lookAhead;
}

void Path::inject() {
    injectedWaypoints.clear();
    for (int i = 0; i <= (waypoints.size() - 2); ++i) {
        Point start = waypoints[i];
        Point end = waypoints[i + 1];
        Point diff = end - start;
        Point step = diff.normalise()*spacing;
        int maxAmt = ceil(diff.magnitude()/spacing);
        for (int j = 0; j < maxAmt; ++j) {
            injectedWaypoints.push_back(start + step*j);
        }
    }
    injectedWaypoints.push_back(waypoints[waypoints.size() - 1]);
    length = injectedWaypoints.size();
}

void Path::smooth() {
    smoothedWaypoints = injectedWaypoints;
    double change = tolerance;
    while (change >= tolerance) {
        change = 0.0;
        for (int i = 1; i < length-1; ++i) {
            Point aux = smoothedWaypoints[i];
            smoothedWaypoints[i] = aux + (injectedWaypoints[i] - smoothedWaypoints[i])*weightData + (smoothedWaypoints[i-1] + smoothedWaypoints[i+1] - smoothedWaypoints[i]*2)*weightSmooth;
            Point diff = smoothedWaypoints[i] - aux;
            change += (fabs(diff.getX()) + fabs(diff.getY()));
        }
    }
}

void Path::findDistance() {
    Point previousWaypoint = smoothedWaypoints[0];
    double previousDistance = 0;
    for (auto smoothedWaypoint: smoothedWaypoints) {
        Point diff = smoothedWaypoint - previousWaypoint;
        double changeDistance = diff.magnitude();
        double newDistance = previousDistance + changeDistance;
        distance.push_back(newDistance);
        previousDistance = newDistance;
        previousWaypoint = smoothedWaypoint;
    }
}

void Path::findCurvature() {
    curvature.push_back(0);
    for (int i = 1; i <= length - 2; ++i) {
        double x1 = smoothedWaypoints[i].getX(), y1 = smoothedWaypoints[i].getY();
        double x2 = smoothedWaypoints[i-1].getX(), y2 = smoothedWaypoints[i-1].getY();
        double x3 = smoothedWaypoints[i+1].getX(), y3 = smoothedWaypoints[i+1].getY();
        if (x1 == x2) x1 = 1/INFINITY;
        double k1 = 0.5*(pow(x1, 2) + pow(y1, 2) - pow(x2, 2) - pow (y2, 2))/(x1 - x2);
        double k2 = (y1 - y2)/(x1 - x2);

        double b = 0.5*(pow(x2, 2) - 2*x2*k1 + pow(y2, 2) - pow(x3, 2) + 2*x3*k1 - pow(y3, 2))/(x3*k2 - y3 + y2 - x2*k2);
        double a = k1 - k2 * b;

        double r = sqrt(pow((x1 - a), 2) + pow((y1 - b), 2));
        curvature.push_back(1/r);
    }
    curvature.push_back(0);
}

void Path::findMaxVel() {
    maxVel.clear();
    for (int i = 0; i < length; i++) {
        maxVel.push_back(std::min(globalMaxVel, k/curvature[i]));
    }
}

void Path::findTargetVel() {
    targetVel.clear();
    targetVel.resize(length, 0);
    targetVel[length - 1] = 0;
    for (int i = length - 2; i >= 0; --i) {
        Point diff = smoothedWaypoints[i+1] - smoothedWaypoints[i];
        double s = diff.magnitude();
        targetVel[i] = std::min(maxVel[i], sqrt(targetVel[i+1]*targetVel[i+1] + 2*globalMaxAccel*s));
    }
}

void Path::setWaypoints(std::vector<Point> iwaypoints, double iweightdata, double iweightsmooth, double ilookahead) {
    waypoints = iwaypoints;
    weightData = iweightdata;
    weightSmooth = iweightsmooth;
    lookAhead = ilookahead;
    this -> inject();
    this -> smooth();
    this -> findDistance();
    this -> findCurvature();
    this -> findMaxVel();
    this -> findTargetVel();
    isPursuing = true;
}