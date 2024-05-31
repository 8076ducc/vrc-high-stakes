#ifndef _MECHCONTROL_HPP_
#define _MECHCONTROL_HPP_

void runIntake(int speed);

void setArmTarg(int targ);

void armControl(void *ignore);

void setMogoState(bool state);

void toggleMogoState();

void setClimbState(bool state);

void toggleClimbState();

#endif 