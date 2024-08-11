/*
 * @Author: Dyyt587 67887002+Dyyt587@users.noreply.github.com
 * @Date: 2024-06-19 17:16:21
 * @LastEditors: Dyyt587 67887002+Dyyt587@users.noreply.github.com
 * @LastEditTime: 2024-06-19 20:17:44
 * @FilePath: \project\applications\motor_def.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef MOTOR_DEF_H
#define MOTOR_DEF_H
#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>
#include "Trajectory_planning.h"
#include "apid.h"

#define MOTOR_USING_AUTO_INIT   /* 自动初始化 */
#define MOTOR_DEBUGING_AUTO_INIT  

    typedef enum
    {
        MOTOR_MODE_IDEL = 0U,   /* 空闲 */
        MOTOR_MODE_TORQUE = 1U, /* 力矩 */
        MOTOR_MODE_SPEED = 2U,  /* 速度 rpm 转每分钟 */
        MOTOR_MODE_POS = 3U,    /* 位置 rad */
        MOTOR_MODE_VOLTAGE,     /* 电压 mv 毫伏 */
        MOTOR_MODE_TEMP,        /* 温度 °C */

        MOTOR_MODE_ACC,        /* 加速度 */
        MOTOR_MODE_DAC,        /* 减速度 */
        MOTOR_MODE_MAX_V,      /* 最大速度，通常用于位置模式限制速度 */


        MOTOR_MODE_SAFETY_STOP,  /* 紧急关闭 */
        MOTOR_MODE_SAFETY_START, /* 安全启动 */
        MOTOR_MODE_MAX,
    } MOTOR_VALUE_TYPE;

    enum
    {
        MOTOR_CONTROL_SUPPORT_NONE = 0, // 硬件支持
        MOTOR_CONTROL_SUPPORT_TORQUE,   // 硬件支持
        MOTOR_CONTROL_SUPPORT_SPEED,    // 速度支持
        MOTOR_CONTROL_SUPPORT_POS,      // 位置支持
    };

    typedef struct motor motor_t;
    typedef int (*motor_driver)(int id, uint16_t mode, float *value, void *user_data);
    typedef int (*motor_ctr)(int id, uint16_t mode, float *data);
    typedef int (*motor_behiver)(int id, uint16_t mode, void *data, void *user_data);
    typedef void (*motor_shakedown)(int id, motor_t *motor);

    typedef struct
    {
        motor_driver driver;
        motor_ctr control;
        CurveObjectType* curve;

        void *user_data;//电机驱动私有数据
    } motor_ops_t;
    struct motor
    {
        motor_ops_t *ops;
        int id;
        const char *name;
        
        long long time;
        uint8_t ratio_pos:4;
        uint8_t ratio_speed:4;
        // uint8_t torque_tick:2;
        apid_t *pid_speed;
        apid_t *pid_pos;
        apid_t *pid_torque;

        float tar_speed;
        float tar_pos;
        float tar_torque;

        float cur_speed;
        float cur_pos;
        float cur_torque;

        uint8_t flag_run_mode : 2;         // 记录当前运行模式
        uint8_t flag_out_mode : 2;         // 记录当前输出模式
        uint8_t flag_accept_level : 2;     // 记录允许的模式 0 - 3
        uint8_t flag_passive_feedback : 1; // 用户自行反馈当前值
        uint8_t flag_is_stop : 1;          // 记录支持的模式 0 - 3

        motor_behiver behaver;
        float acc_out;

        uint8_t timeout_cnt;

        float ratio;/*减速比*/

        
    };

#ifdef __cplusplus
}
#endif
#endif
