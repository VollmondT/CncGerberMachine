#include <hal.h>
#include "laser.h"

unsigned LASER_POWER = 5; //percents

void LaserEnable(void) {
	pwmEnableChannel(&PWMD2, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD2, LASER_POWER*100));
}

void LaserDisable(void) {
	pwmDisableChannel(&PWMD2, 1);
}
