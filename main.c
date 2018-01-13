#include <ch.h>
#include <hal.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include <shell.h>
#include <chprintf.h>
#include <stdlib.h>

#include "board.c"
#include "laser.c"
#include "motor.c"
#include "gerber.c"


#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

static PWMConfig pwmcfg = {
	50000,
	100,
	NULL,
	{
		{PWM_OUTPUT_DISABLED, NULL},
		{PWM_OUTPUT_ACTIVE_HIGH, NULL},
		{PWM_OUTPUT_DISABLED, NULL},
		{PWM_OUTPUT_DISABLED, NULL}
	},
	0,
	0,
	#if STM32_PWM_USE_ADVANCED
	0
	#endif
};

static void cmd_start(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argc;
	(void)argv;
	(void)chp;

	MotorDriverSetEnabled(MOTOR_X, true);
	MotorDriverSetSleep(MOTOR_X, false);
	
	
	MotorDriverSetEnabled(MOTOR_Y, true);
	MotorDriverSetSleep(MOTOR_Y, false);
}

static void cmd_stop(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argc;
	(void)argv;
	(void)chp;
	
	MoveTo(0,0);

	MotorDriverSetEnabled(MOTOR_X, false);
	MotorDriverSetSleep(MOTOR_X, true);
	
	MotorDriverSetEnabled(MOTOR_Y, false);
	MotorDriverSetSleep(MOTOR_Y, true);
}

static void cmd_moveto(BaseSequentialStream *chp, int argc, char *argv[]) {
	if( argc < 2 ) {
		chprintf(chp, "move XPOS YPOS\r\n");
		return;
	}
	MoveTo(atoi(argv[0]), atoi(argv[1]));
}

static void cmd_movetol(BaseSequentialStream *chp, int argc, char *argv[]) {
	if( argc < 2 ) {
		chprintf(chp, "movel XPOS YPOS\r\n");
		return;
	}

	LaserEnable();
	MoveTo(atoi(argv[0]), atoi(argv[1]));
	LaserDisable();
}

static void cmd_origin(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argc;
	(void)argv;
	(void)chp;
	MoveTo(0, 0);
}

static void cmd_ping(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argc;
	(void)argv;
	chprintf(chp, "pong\r\n");
}

static void cmd_lamp(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)chp;
	
	int timeout = argc > 0 ? atoi(argv[0]) : 1;
	LaserEnable();
	while(timeout--) {
		chThdSleepSeconds(1);
	}
	LaserDisable();
}

static void cmd_laserpower(BaseSequentialStream *chp, int argc, char *argv[]) {
	if( argc > 0 ) {
		LASER_POWER = atoi(argv[0]);
		return;
	}
	chprintf(chp, "%d%%\r\n", LASER_POWER);
}

static GerberContext* gbr = NULL;

static void cmd_gerber_start(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argc;
	(void)argv;
	if( gbr ) {
		chprintf(chp, "Gerber machine is already in progress\r\n");
		return;
	}
	gbr = GerberContextNew();
}

static void cmd_gerber_finish(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argc;
	(void)argv;
	if( !gbr ) {
		chprintf(chp, "Gerber machine is already free\r\n");
		return;
	}
	GerberContextFree(gbr);
	gbr = NULL;
}

static void cmd_gerber(BaseSequentialStream *chp, int argc, char *argv[]) {
	if( !gbr) {
		chprintf(chp, "No Gerber machine was activated\r\n");
	}
	if( argc < 1 ) {
		chprintf(chp, "Wrong Gerber command\r\n");
		return;
	}
	GerberAcceptCommand(gbr, argc, argv);
}

static const ShellCommand commands[] = {
	{"start", cmd_start},
	{"stop", cmd_stop},
	{"lamp", cmd_lamp},
	{"laserpower", cmd_laserpower},
	{"move", cmd_moveto},
	{"movel", cmd_movetol},
	{"origin", cmd_origin},
	{"ping", cmd_ping},
	{"gerber_start", cmd_gerber_start},
	{"gerber_finish", cmd_gerber_finish},
	{"gerber", cmd_gerber},
	{NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
	(BaseSequentialStream *)&SD3,
	commands
};

static const GPTConfig gpt_motor = {
	1000000,
	MotorCallback,
	0,
	0
};

int main(void) {
	halInit();
	chSysInit();

	palSetPadMode(GPIOA, 1, PAL_MODE_STM32_ALTERNATE_PUSHPULL);

	palSetPadMode(GPIOB, 10, PAL_MODE_STM32_ALTERNATE_PUSHPULL);

        sdStart(&SD3, NULL);

	pwmStart(&PWMD2, &pwmcfg);
	
	gptStart(MOTOR_TIMER, &gpt_motor);

	MotorDriverInit(MOTOR_X);
	MotorDriverSetReset(MOTOR_X, false);

	MotorDriverInit(MOTOR_Y);
	MotorDriverSetReset(MOTOR_Y, false);
	
	while( 1 ) {
		thread_t *shelltp = chThdCreateFromHeap(
			NULL,
			SHELL_WA_SIZE,
			"shell",
			NORMALPRIO + 1,
			shellThread, (void *)&shell_cfg1
		);
		chThdWait(shelltp);
	}
}
