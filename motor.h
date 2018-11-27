#ifndef _MOTOR_H
#define _MOTOR_H

#include <hal.h>
#include "board.h"

#define MOTOR_MICROSTEPPING s1_8
#define MOTOR_TIMER (&GPTD1)

typedef struct {
	GPIO_TypeDef* m_port;
	uint32_t m_pad;
} Pad;

typedef enum {
	PadEnable = 0,
	PadM0 = 1,
	PadM1 = 2,
	PadReset = 3,
	PadSleep = 4,
	PadStep = 5,
	PadDir = 6,
	PadsEnd
} MotorPadsIndex;


typedef enum {
	sFull = 1,
	sHalf = 2,
	s1_4 = 4,
	s1_8 = 8
} MotorStepping;


typedef struct {
	Pad m_pads[7];
} MotorDriver;

void MotorDriverSetEnabled(MotorDriver* drv, int val);
void MotorDriverSetSleep(MotorDriver* drv, int val);
void MotorDriverSetReset(MotorDriver* drv, int val);
void MotorDriverSetStepping(MotorDriver* drv, MotorStepping val);
void MotorDriverSetDirection(MotorDriver* drv, int val);
void MotorDriverInit(MotorDriver* drv);

#define MOTOR_GROUP_MAX_SIZE 2
#define MOTOR_GROUP_CLEAR(group) (group)->m_count = 0
#define MOTOR_GROUP_ADD(group, driver) (group)->m_drivers[(group)->m_count++] = driver

typedef struct MotorGroup {
	int m_count;
	MotorDriver* m_drivers[MOTOR_GROUP_MAX_SIZE];
} MotorGroup;

void MotorGroupMakeSteps(const unsigned x_count, const unsigned y_count, int silent);

extern MotorDriver DRV1;
extern MotorDriver DRV2;

#define MOTOR_X (&DRV1)
#define MOTOR_X_DIRECTION_PLUS 0
#define MOTOR_X_DIRECTION_MINUS 1

#define MOTOR_Y (&DRV2)
#define MOTOR_Y_DIRECTION_PLUS 0
#define MOTOR_Y_DIRECTION_MINUS 1

#endif // _MOTOR_H

