#include "main.h"

Motor intake(-4);
Motor arm(7);
ADIDigitalOut mogo(1);
ADIDigitalOut climb(2);

void runIntake(int speed) {
    intake.move(speed);
}

int armTarg = 0;

void setArmTarg(int targ) {
    armTarg = targ;
}

void armControl(void *ignore) {
    arm.tare_position();
    while (true) {
        int error = armTarg - arm.get_position();
        arm.move(error * 2);
        delay(10);
    }
}

bool mogoState = false;

void setMogoState(bool state) {
    mogo.set_value(state);
    mogoState = state;
}

void toggleMogoState() {
    mogoState = !mogoState;
    mogo.set_value(mogoState);
}

bool climbState = false;

void setClimbState(bool state) {
    climb.set_value(state);
    climbState = state;
}

void toggleClimbState() {
    climbState = !climbState;
    climb.set_value(climbState);
}