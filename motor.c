#include "motor.h"

#define STEP_MEANDR 20
#define STEP_WAIT 500

static void MotorDriverSetPad(MotorDriver* drv, MotorPadsIndex pad, int set) {
	if( set ) {
		palSetPad(drv->m_pads[pad].m_port, drv->m_pads[pad].m_pad);
	} else {
		palClearPad(drv->m_pads[pad].m_port, drv->m_pads[pad].m_pad);
	}
}

typedef void(*Stepfunction)(GPTDriver*);

static unsigned motor_movement_x1, motor_movement_x2, motor_movement_y1, motor_movement_y2;
static int motor_x_delta, motor_y_delta; // total steps count in any direction
static int motor_movement_interpolation_error;
static uint16_t motor_microsteps;
static unsigned char motor_x_involved, motor_y_involved; // involved on next full step
static int motor_move_silent;
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

void MotorStepStagePrepareFullStepSilent(GPTDriver* gptp) {
	motor_x_involved = motor_movement_x1 != motor_movement_x2 ? 1 : 0;
	if( motor_x_involved ) {
		++motor_movement_x1;
		MotorDriverSetPad(MOTOR_X, PadStep, 1);
	}

	motor_y_involved = motor_movement_y1 != motor_movement_y2 ? 1 : 0;
	if( motor_y_involved ) {
		++motor_movement_y1;
		MotorDriverSetPad(MOTOR_Y, PadStep, 1);
	}

	if( !(motor_x_involved || motor_y_involved) ) {
		// finished
		gptStopTimerI(gptp);
		chBSemSignalI(&motor_sem);
		return;
	}

	if( MOTOR_MICROSTEPPING == sFull ) {
		motor_step_function = MotorStepStagePrepareFullStepSilent;
	} else {
		motor_microsteps = MOTOR_MICROSTEPPING;
		motor_step_function = MotorStepStageMakeMicrostep;
	}

	motor_step_next_stage = MotorStepStageOnMeandrGenerated;
	gptChangeIntervalI(gptp, STEP_MEANDR);
}

void MotorStepStagePrepareFullStep(GPTDriver* gptp) {
	if( motor_movement_x1 == motor_movement_x2 && motor_movement_y1 == motor_movement_y2 ) {
		// finished
		gptStopTimerI(gptp);
		chBSemSignalI(&motor_sem);
		return;
	}

	if( MOTOR_MICROSTEPPING == sFull ) {
		motor_step_function = MotorStepStagePrepareFullStep;
	} else {
		motor_microsteps = MOTOR_MICROSTEPPING;
		motor_step_function = MotorStepStageMakeMicrostep;
	}

	const int error = motor_movement_interpolation_error * 2;
	if(error > -motor_y_delta) {
		motor_movement_interpolation_error -= motor_y_delta;
		++motor_movement_x1;
		motor_x_involved = 1;
		MotorDriverSetPad(MOTOR_X, PadStep, 1);
	} else {
		motor_x_involved = 0;
	}
	
	if(error < motor_x_delta) {
		motor_movement_interpolation_error += motor_x_delta;
		++motor_movement_y1;
		motor_y_involved = 1;
		MotorDriverSetPad(MOTOR_Y, PadStep, 1);
	} else {
		motor_y_involved = 0;
	}

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
		motor_step_function = motor_move_silent ? MotorStepStagePrepareFullStepSilent : MotorStepStagePrepareFullStep;
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
	MotorDriverSetEnabled(drv, true);
	MotorDriverSetSleep(drv, true);
	MotorDriverSetReset(drv, false);
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


void MotorGroupMakeSteps(const unsigned x_count, const unsigned y_count, int silent) {
	motor_movement_x1 = 0;
	motor_movement_y1 = 0;
	motor_movement_x2 = x_count;
	motor_movement_y2 = y_count;
	motor_x_delta = x_count;
	motor_y_delta = y_count;
	motor_movement_interpolation_error = motor_x_delta - motor_y_delta;
	motor_move_silent = silent;

	motor_step_next_stage = silent ? MotorStepStagePrepareFullStepSilent: MotorStepStagePrepareFullStep;
	gptStartContinuous(MOTOR_TIMER, STEP_MEANDR);
	chBSemWait(&motor_sem);
}
