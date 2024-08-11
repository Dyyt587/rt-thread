#include "motor.h"


void motor_set_pos_plan(int id,float targetPos,float stepPos,float flexible,int maxTimes)
{
	motor_t* motor = motor_get(id);
	motor->ops->curve->intervel = 2;
	motor->ops->curve->aTimes = 0;			  // 当前时间步
	motor->ops->curve->targetPos = targetPos*motor->ratio;
	motor->ops->curve->startPos = motor->cur_pos;
	motor->ops->curve->currentPos = motor->cur_pos;
	motor->ops->curve->maxTimes = maxTimes;
	motor->ops->curve->stepPos = stepPos;
	motor->ops->curve->flexible = flexible;
	motor->ops->curve->maxTimes = maxTimes;
	motor->ops->curve->curveMode = CURVE_TRAP;
	motor->tar_pos = motor_planning(motor->ops->curve);
}
