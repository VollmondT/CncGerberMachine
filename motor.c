#include "motor.h"

#define STEP_MEANDR 20
#define STEP_WAIT 200

static void MotorDriverSetPad(MotorDriver* drv, MotorPadsIndex pad, int set) {
	if( set ) {
		palSetPad(drv->m_pads[pad].m_port, drv->m_pads[pad].m_pad);
	} else {
		palClearPad(drv->m_pads[pad].m_port, drv->m_pads[pad].m_pad);
	}
}

typedef void(*Stepfunction)(GPTDriver*);

static int motor_x_delta, motor_y_delta; // total steps count in any direction
static unsigned motor_microsteps; // how many microsteps we need to do at current full step
static int motor_x_involved, motor_y_involved; // involved on next full step

static unsigned motor_steps_ticker;
Stepfunction motor_step_next_stage, motor_step_function;

BSEMAPHORE_DECL(motor_sem, TRUE);

static void MotorStepStageMakeMicrostep(GPTDriver* gptp);

static void MotorStepStageOnMeandrGenerated(GPTDriver* gptp) {
	MotorDriverSetPad(MOTOR_X, PadStep, 0);
	MotorDriverSetPad(MOTOR_Y, PadStep, 0);
	--motor_microsteps;
	motor_step_next_stage = motor_step_function;
	gptChangeIntervalI(gptp, STEP_WAIT);
}

void MotorStepStagePrepareFullStep(GPTDriver* gptp) {
	if( motor_steps_ticker == 0 ) {
		// finished
		gptStopTimerI(gptp);
		chBSemSignalI(&motor_sem);
		return;
	}

	--motor_steps_ticker;

	if( MOTOR_MICROSTEPPING == sFull ) {
		motor_step_function = MotorStepStagePrepareFullStep;
	} else {
		motor_microsteps = MOTOR_MICROSTEPPING;
		motor_step_function = MotorStepStageMakeMicrostep;
	}

	//
	if( motor_x_delta ) {
		--motor_x_delta;
		motor_x_involved = 1;
		MotorDriverSetPad(MOTOR_X, PadStep, 1);
	} else {
		motor_x_involved = 0;
	}

	if( motor_y_delta ) {
		--motor_y_delta;
		motor_y_involved = 1;
		MotorDriverSetPad(MOTOR_Y, PadStep, 1);
	} else {
		motor_y_involved = 0;
	}
	//

	motor_step_next_stage = MotorStepStageOnMeandrGenerated;
	gptChangeIntervalI(gptp, STEP_MEANDR);
}

void MotorStepStageMakeMicrostep(GPTDriver* gptp) {
	if( motor_x_involved ) {
		MotorDriverSetPad(MOTOR_X, PadStep, 1);
	}
	if( motor_y_involved ) {
		MotorDriverSetPad(MOTOR_Y, PadStep, 1);
	}
	if( motor_microsteps == 1 ) {
		motor_step_function = MotorStepStagePrepareFullStep;
	}
	motor_step_next_stage = MotorStepStageOnMeandrGenerated;
	gptChangeIntervalI(gptp, STEP_MEANDR);
}


static void MotorCallback(GPTDriver* gptp) {
	osalSysLockFromISR();
	motor_step_next_stage(gptp);
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


void MotorGroupMakeSteps(const unsigned x_count, const unsigned y_count) {
	motor_x_delta = x_count;
	motor_y_delta = y_count;
	motor_steps_ticker = x_count > y_count ? x_count : y_count;

	motor_step_next_stage = MotorStepStagePrepareFullStep;
	gptStartContinuous(MOTOR_TIMER, STEP_MEANDR);
	chBSemWait(&motor_sem);
}
