/**
 * @file motor.h
 * @author Dyyt587 (805207319@qq.com)
 * @brief 电机通用驱动
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef MOTOR_H
#define MOTOR_H
#if defined(__cplusplus)
extern "C"
{
#endif
#include "apid.h"
#include "stdbool.h"
#include "motor_cfg.h"
#include "motor_def.h"
#include <stdint.h>
#include <stdlib.h>
#include "Trajectory_planning.h"
//#include "motor_planning.h"

/* Compiler Related Definitions */
#include "rtcompiler.h"


    motor_t *motor_get(int id);
    int motor_get_id(const char *name);

    int motor_handle(int id, float cycle);
    void motor_init(void);

    int motor_read_feedback(int id, int cycle);

    void motor_set_passive_feedback(int id, bool is_true);

    void motor_set_ratio(int id, float ratio);


    int motor_feedback_torque(int id, float value);
    int motor_feedback_speed(int id, float value);
    int motor_feedback_pos(int id, float value);

    int motor_stop(int id);
    int motor_start(int id);

    int motor_set_speed(int id, float value);
    int motor_set_relative_speed(int id, float value);

    int motor_set_pos(int id, float value);
    int motor_set_relative_pos(int id, float value);
    void motor_set_pos_plan(int id,float targetPos,float stepPos,float flexible,int maxTimes);

    int motor_set_torque(int id, float value);

    float motor_get_speed(int id);

    float motor_get_pos(int id);

    float motor_get_torque(int id);

    apid_t *motor_get_pid_torque(int id);
    apid_t *motor_get_pid_speed(int id);
    apid_t *motor_get_pid_pos(int id);

    void motor_shakdown(int id);

    int motor_control(int id, MOTOR_VALUE_TYPE mode, void *data);

    void motor_set_pid_speed_ratio(int id,uint8_t ratio_speed,uint8_t ratio_pos);

#if defined(__cplusplus)
}
#endif
#endif