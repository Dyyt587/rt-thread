/*
 * @Author: Dyyt587 67887002+Dyyt587@users.noreply.github.com
 * @Date: 2024-06-19 18:15:02
 * @LastEditors: Dyyt587 67887002+Dyyt587@users.noreply.github.com
 * @LastEditTime: 2024-08-08 01:36:32
 * @FilePath: \project\applications\motor_dj_rm_driver_cfg.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef MOTOR_MIT_DRIVER_CFG_H
#define MOTOR_MIT_DRIVER_CFG_H
#include "motor_mit_driver.h"
#include "motor_cfg.h"

#ifdef __cplusplus
extern "C"{
#endif

#define MIT_DISCONNECT_LEVEL 200
    /**
     * @brief 电机抽象层电机操作函数定义
     */
#ifdef MOTOR_MIT_ID1_CAN1
#define MOTOR_MIT_ID1_CAN1_OPS           \
    {                                         \
        .curve = MOTOR_OPS_CURVE_INIT(),      \
        .driver = motor_mit_driver,            \
        .control = motor_mit_ctr,              \
        .user_data = &mit_motors[MIT_M_CAN1_1], \
    }
#endif

#ifdef MOTOR_MIT_ID2_CAN1
#define MOTOR_MIT_ID2_CAN1_OPS           \
    {                                         \
        .curve = MOTOR_OPS_CURVE_INIT(),      \
        .driver = motor_mit_driver,            \
        .control = motor_mit_ctr,              \
        .user_data = &mit_motors[MIT_M_CAN1_2], \
    }
#endif
#ifdef __cplusplus
}
#endif
#endif
