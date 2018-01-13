#include "motor.h"

#define STEP_MEANDR 20
#define STEP_WAIT 200

static int motor_x_delta, motor_y_delta;

BSEMAPHORE_DECL(motor_sem, TRUE);

static void MotorDriverSetPad(MotorDriver* drv, MotorPadsIndex pad, int set) {
	if( set ) {
		palSetPad(drv->m_pads[pad].m_port, drv->m_pads[pad].m_pad);
	} else {
		palClearPad(drv->m_pads[pad].m_port, drv->m_pads[pad].m_pad);
	}
}


static void motor_cb(GPTDriver* gptp) {
	osalSysLockFromISR();

	motor_x_delta--;
	motor_y_delta--;

	if( motor_x_delta > 0 || motor_y_delta > 0) {
		int tick = motor_x_delta & 1; // odd number, no x meaning
		if( motor_x_delta > 0 ) {
			MotorDriverSetPad(MOTOR_X, PadStep, tick);
		}
		if( motor_y_delta > 0 ) {
			MotorDriverSetPad(MOTOR_Y, PadStep, tick);
		}
		gptChangeIntervalI(gptp, tick ? STEP_MEANDR : STEP_WAIT);
	} else {
		gptStopTimerI(gptp);
		chBSemSignalI(&motor_sem);
	}

	osalSysUnlockFromISR();
}

void MotorDriverSetEnabled(MotorDriver* drv, int val) {
	MotorDriverSetPad(drv, PadEnable, !val);
}

void MotorDriverSetSleep(MotorDriver* drv, int val) {
	MotorDriverSetPad(drv, PadSleep, !val);
}

void MotorDriverSetReset(MotorDriver* drv, int val) {
	MotorDriverSetPad(drv, PadReset, !val);
}

void MotorDriverSetStepping(MotorDriver* drv, MotorStepping val) {
	unsigned s = 0;

	switch(val) {
	case sHalf:
		s = 1;
		break;
		
	case s1_4:
		s = 2;
		break;
		
	case s1_8:
		s = 3;
		break;
		
	default:
		s = 0;
	}

	MotorDriverSetPad(drv, PadM0, s & 1);
	MotorDriverSetPad(drv, PadM1, s & 2);
}

void MotorDriverSetDirection(MotorDriver* drv, int val) {
	MotorDriverSetPad(drv, PadDir, val);
}

void MotorDriverInit(MotorDriver* drv) {
	for( int i = 0; i < PadsEnd; ++i ) {
		palSetPadMode(drv->m_pads[i].m_port, drv->m_pads[i].m_pad, PAL_MODE_OUTPUT_PUSHPULL);
	}
	MotorDriverSetPad(drv, PadStep, 0);
	MotorDriverSetEnabled(drv, false);
	MotorDriverSetSleep(drv, true);
	MotorDriverSetReset(drv, true);
	MotorDriverSetStepping(drv, MOTOR_MICROSTEPPING);
}

MotorDriver DRV1 = {
	.m_pads = {
		{GPIOB, 9},
		{GPIOB, 8},
		{GPIOB, 7},
		{GPIOB, 6},
		{GPIOB, 5},
		{GPIOB, 4},
		{GPIOB, 3}
	},
};

MotorDriver DRV2 = {
	.m_pads = {
		{GPIOA, 15},
		{GPIOA, 12},
		{GPIOA, 11},
		{GPIOA, 10},
		{GPIOA, 9},
		{GPIOA, 8},
		{GPIOB, 15},
	},
};


void MotorGroupMakeSteps(const int x_count, const int y_count) {
	motor_x_delta = x_count*MOTOR_MICROSTEPPING*2;
	motor_y_delta = y_count*MOTOR_MICROSTEPPING*2;
	gptStartContinuous(MOTOR_TIMER, STEP_WAIT);
	chBSemWait(&motor_sem);
}
