#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include <iostream>
#include <ostream>
#include <string>

IMU inertial (6);
int autonnum = 5;

bool unlocked = false;

void controllerScreenControl(void *ignore) {
	Controller master (E_CONTROLLER_MASTER);
	bool alreadyRumbled = false;
	while (true) {
		if (competition::is_autonomous() || competition::is_disabled()) {
			master.print(0, 0, "X: %.4f", position.getX());
			delay(50);
			master.print(1, 0, "Y: %.4f", position.getY());
			delay(50);
			master.print(2, 0, "Bearing: %.4f", radToDeg(bearing));
			delay(50);
		} else {
			delay(50);
		}
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
	Task odomTask (odometryControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "odom");
	Task baseTask(baseControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "YEE");
	Task displayTask(displayControl);
	Task screenTask(controllerScreenControl, (void*) "PROS", TASK_PRIORITY_MIN, TASK_STACK_DEPTH_DEFAULT, "controller screen");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() {
	double start = millis();

	switch(autonnum) {
		case 1:
		break;

		case 3:
		delay(5);
		break;
	}
	screen::print(TEXT_MEDIUM, 300, 200, "Time elapsed: %f", (millis() - start)/1000);
}	

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	MotorGroup left_mg({-1, 2, -3});	
	MotorGroup right_mg({8, -9, 10});
	bool leftArcade = false;

	while (true)
	{
		if (master.get_digital_new_press(DIGITAL_A))
		{
			leftArcade = !leftArcade;
		}

		int dir, turn;

		if (leftArcade)
		{
			dir = master.get_analog(ANALOG_LEFT_Y);	  // Gets amount forward/backward from left joystick
			turn = master.get_analog(ANALOG_RIGHT_X); // Gets the turn left/right from right joystick
		}
		else
		{
			dir = master.get_analog(ANALOG_RIGHT_Y); // Gets amount forward/backward from right joystick
			turn = master.get_analog(ANALOG_LEFT_X); // Gets the turn left/right from left joystick
		}

		left_mg.move(dir + turn);
		right_mg.move(dir - turn); 
		if (master.get_digital(DIGITAL_L1))
		{
			runIntake(127);
		}
		else if (master.get_digital(DIGITAL_L2))
		{					   
			runIntake(-127);
		}
		else
		{
			runIntake(0);
		}

		if (master.get_digital_new_press(DIGITAL_R1)) {
			toggleMogoState();
		}
		pros::delay(10);
	}
}