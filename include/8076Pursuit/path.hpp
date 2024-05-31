#ifndef _8076_PURSUIT_PATH_HPP_
#define _8076_PURSUIT_PATH_HPP_
#include <vector>
#include "point.hpp"

#define inPerDeg 0.02268928027*1.2

#define RPMToInPerMillis 1/60/1000*480*inPerDeg
#define inPerMillisToRPM 1/inPerDeg*1000*60/480 

extern double globalMaxVel, globalMaxAccel;

void setMaxRPM(double rpm);

void setMaxAccel(double accel);

class Path{
    private:
    std::vector<Point> waypoints;
    std::vector<Point> injectedWaypoints;
    std::vector<Point> smoothedWaypoints;

    double weightSmooth, weightData;
    double lookAhead;
    int length;

    std::vector<double> distance;
    std::vector<double> curvature;
    std::vector<double> maxVel;
    std::vector<double> targetVel;

    public:
    Path();
    Path(std::vector<Point> iwaypoints);
    Point getSmoothedWaypoints(int i);
    double getMaxVel(int i);
    double getTargetVel(int i);
    int getLength();
    double getLookAhead();
    void inject();
    void smooth();
    void findDistance();
    void findCurvature();
    void findMaxVel();
    void findTargetVel();
    void setWaypoints(std::vector<Point> iwaypoints, double iweightdata, double iweightsmooth, double ilookahead);
};

#endif