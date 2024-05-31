#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"

#define PI 3.1415926535897932384626433
#define inchesPerDeg 0.017*1.25 //inches per unit of rotation of the encoders
#define hardOffset 0.0

Point position = {0.0,0.0};
double angle;
double bearing;

double degToRad(double deg) {
    return deg*PI/180;
}

double radToDeg(double rad) {
    return rad/PI*180;
}

double boundRad(double rad){
  double res = fmod(rad, PI*2);
  if(res < 0) res += (PI*2);
  return res;
}

double EMABearing = 0, prevEMABearing = 0;

// void updateBearingEMA() {
//     IMU inertial (16);
//     double currentBearing = inertial.is_calibrating()? 0 : degToRad(inertial.get_rotation() - offset);
//     double EMAConstant = 2.0/4.0;
//     EMABearing = (currentBearing * EMAConstant) + (prevEMABearing * (1 - EMAConstant));
//     prevEMABearing = EMABearing; 
// }

double diffBearing = 0;

void setBearing(double bear) {
    diffBearing = bear - radToDeg(bearing);
}

void odometryControl(void *ignore) {
    IMU inertial(6);
    pros::Motor L1 (1, E_MOTOR_GEAR_BLUE, true);
	Motor R1 (8, E_MOTOR_GEAR_BLUE, false);
    double encdL = 0, encdR = 0;
    double prevEncdL = 0, prevEncdR = 0, prevBearing = 0;
    L1.tare_position();
    R1.tare_position();
    position.setPoint(0, 0);
    inertial.set_heading(0);
    while (true) {
        encdL = L1.get_position();
        encdR = R1.get_position();
        bearing = inertial.is_calibrating() ? 0 : degToRad(inertial.get_rotation() + diffBearing);
        angle = boundRad(PI/2 - bearing);

        double encdChangeL = encdL - prevEncdL;
        double encdChangeR = encdR - prevEncdR;

        double distance = (encdChangeL + encdChangeR)/2*inchesPerDeg;
        Point change = {distance*cos(angle), distance*sin(angle)};
        position = position + change;

        prevEncdL = encdL;
        prevEncdR = encdR;
        
        delay(5);
    }
}

// #include "main.h"
// #include "pros/adi.hpp"
// #include "pros/llemu.hpp"
// #include "pros/motors.h"

// #define PI 3.1415926535897932384626433
// #define S_DIS 5.5
// #define R_DIS 1
// #define inchesPerDeg 0.0223
// #define inchesPerDeg2 0.0002441
// #define hardOffset 0.0

// Point position = {0.0,0.0};
// double angle;
// double bearing;

// double degToRad(double deg) {
//     return deg*PI/180;
// }

// double radToDeg(double rad) {
//     return rad/PI*180;
// }

// double boundRad(double rad){
//   double res = fmod(rad, PI*2);
//   if(res < 0) res += (PI*2);
//   return res;
// }

// double EMABearing = 0, prevEMABearing = 0;

// // void updateBearingEMA() {
// //     IMU inertial (16);
// //     double currentBearing = inertial.is_calibrating()? 0 : degToRad(inertial.get_rotation() - offset);
// //     double EMAConstant = 2.0/4.0;
// //     EMABearing = (currentBearing * EMAConstant) + (prevEMABearing * (1 - EMAConstant));
// //     prevEMABearing = EMABearing; 
// // }

// double diffBearing = 0;

// void setBearing(double bear) {
//     diffBearing = bear - radToDeg(bearing);
// }

// void odometryControl(void *ignore) {
//     Rotation R (19, true);
//     Rotation S (13, true);
//     R.set_data_rate(5);
//     S.set_data_rate(5);
//     IMU inertial(5);
//     inertial.set_data_rate(5);
//     R.reset_position();
//     S.reset_position();
//     position.setPoint(0, 0);
//     inertial.set_heading(0);
//     double prevEncdR = 0, prevEncdS = 0, prevBearing = degToRad(inertial.get_rotation());
//     while(true){
//         double encdR = R.get_position()*inchesPerDeg2;
//         double encdS = S.get_position()*inchesPerDeg2/3*5;
//         bearing = inertial.is_calibrating() ? 0 : degToRad(inertial.get_rotation() + diffBearing);
//         angle = boundRad(PI/2 - bearing);
//         double encdChangeR = encdR-prevEncdR;
//         double encdChangeS = encdS-prevEncdS;
//         double bearingChange = bearing - prevBearing;

// 		prevEncdR = encdR;
// 		prevEncdS = encdS;
//         prevBearing = bearing;

//         Point localOffset;
//         if(bearingChange) {
//             localOffset = Point(encdChangeS/bearingChange + S_DIS, encdChangeR/bearingChange + R_DIS) * 2*sin(bearingChange/2);
//         }else {
//             localOffset = Point(encdChangeS, encdChangeR);
//         }

//         double avgBearing = prevBearing + bearingChange/2;
//         Point rotatedOffset = Point(cos(-avgBearing)*localOffset.getX() - sin(-avgBearing)*localOffset.getY(),
//                               sin(-avgBearing)*localOffset.getX() + cos(-avgBearing)*localOffset.getY());
//         position = position + rotatedOffset;
//         delay(10);
//     }
//     // Rotation R (12, true);
//     // R.set_data_rate(5);
//     // S.set_data_rate(5);
//     // inertial.set_data_rate(5);
//     // R.reset_position();
//     // S.reset_position();
//     // position.setPoint(0, 0);
//     // inertial.set_heading(0);
//     // double prevEncdR = 0, prevEncdS = 0, prevBearing = degToRad(inertial.get_rotation());
//     // while(true){
//     //     // updateBearingEMA();
//     //     double encdR = R.get_position()*inchesPerDeg;
//     //     double encdS = S.get_position()*inchesPerDeg;
//     //     bearing = inertial.is_calibrating() ? 0 : degToRad(inertial.get_rotation() - hardOffset + diffBearing);
//     //     angle = boundRad(PI/2 - bearing);
//     //     double encdChangeR = encdR-prevEncdR;
//     //     double encdChangeS = encdS-prevEncdS;
//     //     double bearingChange = bearing - prevBearing;

// 	// 	prevEncdR = encdR;
// 	// 	prevEncdS = encdS;
//     //     prevBearing = bearing;

//     //     Point localOffset;
//     //     if(bearingChange) {
//     //         localOffset = Point(encdChangeS/bearingChange + S_DIS, encdChangeR/bearingChange + R_DIS) * 2*sin(bearingChange/2);
//     //     }else {
//     //         localOffset = Point(encdChangeS, encdChangeR);
//     //     }

//     //     double avgBearing = prevBearing + bearingChange/2;
//     //     Point rotatedOffset = Point(cos(-avgBearing)*localOffset.getX() - sin(-avgBearing)*localOffset.getY(),
//     //                           sin(-avgBearing)*localOffset.getX() + cos(-avgBearing)*localOffset.getY());
//     //     position = position + rotatedOffset;
//     //     delay(10);
//     // }

//     // IMU inertial(21);
//     // double encdL = 0, encdR = 0;
//     // double prevEncdL = 0, prevEncdR = 0, prevBearing = 0;
//     // FL.tare_position();
//     // CL.tare_position();
//     // BL.tare_position();
//     // FR.tare_position();
//     // CR.tare_position();
//     // BR.tare_position();
//     // lcd::initialize();
//     // while (true) {
//     //     encdL = (BL.get_position()+FL.get_position()+CL.get_position())/3;
//     //     encdR = (BR.get_position()+FR.get_position()+CR.get_position())/3;
//     //     bearing = inertial.get_rotation();
//     //     angle = PI/2 - degToRad(bearing);

//     //     double encdChangeL = encdL - prevEncdL;
//     //     double encdChangeR = encdR - prevEncdR;

//     //     double distance = (encdChangeL + encdChangeR)/2*inchesPerDeg;
//     //     Point change = {distance*cos(angle), distance*sin(angle)};
//     //     position = position + change;

//     //     prevEncdL = encdL;
//     //     prevEncdR = encdR;
        
//     //     delay(5);
//     // }
// }