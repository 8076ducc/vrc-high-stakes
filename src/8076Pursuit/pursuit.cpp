#include "main.h"
#include "path.hpp"
#include "pursuit.hpp"

#define PI 3.1415926535897932384626433

#define kV 1700
#define kA 5000
#define kP 500
double slewPow = 15.0*RPMToInPerMillis; //slew rate
#define pursuitThres 2

#define turnkP 0.0022
#define turnkD 0.022
#define turnThres 1

#define R_DIS 4 //basewidth from wheel to wheel divided by 2

int closestPointIndex = 0;
double lastFractionalIndex = 0;

double targetBearing = 0, turnMaxVel, turnMinVel;
int turnL = 1, turnR = 1;
double prevError = 0;
bool atThresh = false;

bool isPursuing = false;

Path currentPath;
bool reverse = false;

bool voltageOverride = false;
double leftVolts = 0;
double rightVolts = 0;

Motor left_1(-1, MOTOR_GEAR_BLUE);
Motor right_1(8, MOTOR_GEAR_BLUE);
MotorGroup left_mg({-1, 2, -3});	
MotorGroup right_mg({8, -9, 10});

double distance(Point p1, Point p2) {
    Point diff = p2 - p1;
    return diff.magnitude();
}

int sgn(double e) {
    if (e >= 0) {
        return 1;
    } else return -1;
}

double cap(double a, double b) {
    if (a > b) return b;
    else if (a < -b) return -b;
    else return a;
}

double minAngle(double targ, double curr) {
    double turnAngle = targ - curr;
    if (turnAngle > 180 || turnAngle < -180) {
        turnAngle = -1 * sgn(turnAngle) * (360 - fabs(turnAngle));
    }
    return turnAngle + curr;
}

double boundDeg(double deg){
  double res = fmod(deg, 360);
  if(res < 0) res += 360;
  return res;
}

void drive(double left, double right) {
    left_mg.move(left);
    right_mg.move(right);
}

void setVolts(double left, double right) {
    leftVolts = left;
    rightVolts = right;
    if (left || right) voltageOverride = true;
    else voltageOverride = false;
}

void resetBase() {
    closestPointIndex = 0;
    lastFractionalIndex = 0;
    targetBearing = radToDeg(bearing);
}

void basePursue(std::vector<Point> iwaypoints, double iweightdata, double iweightsmooth, double ilookahead, bool ireverse) {
    currentPath.setWaypoints(iwaypoints, iweightdata, iweightsmooth, ilookahead);
    reverse = ireverse;
    lcd::set_text(7, "Pursuing");
}

void basePursue(double dist, double iweightdata, double iweightsmooth, double ilookahead) {
    basePursue({position, position + Point(dist*sin(bearing), dist*cos(bearing))}, iweightdata, iweightsmooth, ilookahead, dist < 0);
}

void baseTurn(double bear, double min, double max) {
    targetBearing = minAngle(bear, radToDeg(bearing));
    turnMaxVel = max*RPMToInPerMillis;
    turnMinVel = min*RPMToInPerMillis;
    turnL = 1; 
    turnR = 1;
    atThresh = false;
    lcd::set_text(7, "Turning");
}

void baseTurn(double x, double y, double min, double max) {
    double turnBearing = radToDeg(atan2(x - position.getX(), y - position.getY()));
    double diffBearing = boundDeg(turnBearing - radToDeg(bearing));
    if (diffBearing > 180.0) diffBearing = diffBearing - 360.0;
    targetBearing = radToDeg(bearing) + diffBearing;
    turnMaxVel = max*RPMToInPerMillis;
    turnMinVel = min*RPMToInPerMillis;
    turnL = 1;
    turnR = 1;
    atThresh = false;
    lcd::set_text(7, "Turning");
}

void leftTurn(double bear, double min, double max) {
    targetBearing = minAngle(bear, radToDeg(bearing));
    turnMaxVel = max*RPMToInPerMillis;
    turnMinVel = min*RPMToInPerMillis;
    turnL = 1; 
    turnR = 0;
    atThresh = false;
    lcd::set_text(7, "Turning");
}

void rightTurn(double bear, double min, double max) {
    targetBearing = minAngle(bear, radToDeg(bearing));
    turnMaxVel = max*RPMToInPerMillis;
    turnMinVel = min*RPMToInPerMillis;
    turnL = 0; 
    turnR = 1;
    atThresh = false;
    lcd::set_text(7, "Turning");
}

void waitPursuit(double timer) {
    double start = millis();
    Point target = currentPath.getSmoothedWaypoints(currentPath.getLength() - 1);
    while (distance(target, position) >= pursuitThres && (millis() - start) < timer) delay(5);
    resetBase();
    isPursuing = false;
    reverse = false;
    lcd::clear_line(7);
}

void waitTurn(double timer) {
    double start = millis();
    while (!atThresh && (millis() - start < timer)) delay(5);
    resetBase();
    lcd::clear_line(7);
}

void baseControl(void *ignore) {
    IMU inertial (6);
    left_mg.set_brake_modes(MOTOR_BRAKE_BRAKE);
    right_mg.set_brake_modes(MOTOR_BRAKE_BRAKE);
    Point lookAheadPoint;
    double targV = 0, targVL = 0, targVR = 0;
    double prevTargVL = 0, prevTargVR = 0;
    double targAL = 0, targAR = 0;
    double measuredVL = 0, measuredVR = 0;

    while(true) {
        if (isPursuing) {
            double minDist = INFINITY;
            for (int i = closestPointIndex; i < currentPath.getLength(); ++i) {
                double d = distance(position, currentPath.getSmoothedWaypoints(i)); 
                if(d < minDist){
                    minDist = d;
                    closestPointIndex = i;
                }
            }

            for (int i = floor(lastFractionalIndex); i <= currentPath.getLength()-2; ++i) {
                Point start = currentPath.getSmoothedWaypoints(i);
                Point end = currentPath.getSmoothedWaypoints(i + 1);
                std::vector<double> l = position.findLookAhead(start, end, currentPath.getLookAhead());
                if (l[0]) {
                    double fractionalIndex = i + l[1];
                    if(fractionalIndex >= lastFractionalIndex){
                        lookAheadPoint = start + (end - start) * l[1];
                        lastFractionalIndex = fractionalIndex;
                        break;
                    }
                }
            }

            double practicalAngle = reverse ? angle+PI:angle;

            double a = -tan(practicalAngle);
            double b = 1;
            double c = tan(practicalAngle)*position.getX() - position.getY();
            double xabs = fabs(a * lookAheadPoint.getX() + b * lookAheadPoint.getY() + c)/sqrt(a*a + b*b);
            double crossProduct = sin(practicalAngle)*(lookAheadPoint.getX() - position.getX()) - cos(practicalAngle)*(lookAheadPoint.getY() - position.getY());
            double sign = crossProduct >= 0 ? 1 : -1;
            double moveCurvature = sign*2*xabs/(currentPath.getLookAhead()*currentPath.getLookAhead());

            double targVClosest = reverse ? -currentPath.getTargetVel(closestPointIndex) : currentPath.getTargetVel(closestPointIndex);
            double changeV = targVClosest - targV;
            if (fabs(targVClosest) > fabs(targV)) {
                targV = targV + cap(changeV, slewPow);
            } else {
                targV = targVClosest;
            }
            targVL = targV*(2 + moveCurvature*2*R_DIS)/2;
            targVR = targV*(2 - moveCurvature*2*R_DIS)/2;

        }
        else {
            double error = targetBearing - radToDeg(bearing);
            double derivative = error - prevError;

            double output = error*turnkP + derivative*turnkD;

            if (fabs(output) > turnMaxVel) {
                output = turnMaxVel*sgn(output);
            } else if (fabs(output) < turnMinVel) {
                output = turnMinVel*sgn(output);
            }

            if (fabs(error) > turnThres) {
                targVL = (output) * turnL;
                targVR = (-output) * turnR;
                atThresh = false;
            } else if (fabs(error) <= turnThres) {
                targVL = 0;
                targVR = 0;
                atThresh = true;
            }
            prevError = error;
            lcd::print(4, "Error: %.5f", error);
            lcd::print(5, "Output: %.5f", output);
        }

        targAL = (targVL - prevTargVL)/5;
        targAR = (targVR - prevTargVR)/5;

        double ffL = kV * targVL + kA * targAL;
        double ffR = kV * targVR + kA * targAL;

        double fbL;
        double fbR;

        measuredVL = left_1.get_actual_velocity()*RPMToInPerMillis;
        measuredVR = right_1.get_actual_velocity()*RPMToInPerMillis;

        if(reverse) {
            fbL = kP * (targVL - measuredVR);
            fbR = kP * (targVR - measuredVL);
        }else {
            fbL = kP * (targVL - measuredVL);
            fbR = kP * (targVR - measuredVR);
        }
        if (voltageOverride) {
            drive(leftVolts, rightVolts);
        }
        else if(reverse) {
            drive((ffR + fbR), (ffL + fbL));
        }else {
            drive((ffL + fbL), (ffR + fbR));
        }

        prevTargVL = targVL;
        prevTargVR = targVR;
        
        delay(5);
    }
}