/*
 * @Author: Dyyt587 67887002+Dyyt587@users.noreply.github.com
 * @Date: 2024-06-19 18:15:02
 * @LastEditors: Dyyt587 67887002+Dyyt587@users.noreply.github.com
 * @LastEditTime: 2024-06-19 19:46:39
 * @FilePath: \project\applications\motor_dj_rm_driver_cfg.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef MOTOR_DJ_RM_DRIVER_CFG_H
#define MOTOR_DJ_RM_DRIVER_CFG_H
#include "motor_dj_rm_driver.h"
#include "motor_cfg.h"

#ifdef __cplusplus
extern "C"{
#endif
    /**
     * @brief 电机抽象层电机操作函数定义
     */
#ifdef MOTOR_DJ_M3508_ID1_CAN1
#define MOTOR_DJ_M3508_ID1_CAN1_OPS           \
    {                                         \
        .curve = MOTOR_OPS_CURVE_INIT(),      \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN1_1], \
    }
#endif
#ifdef MOTOR_DJ_M3508_ID2_CAN1
#define MOTOR_DJ_M3508_ID2_CAN1_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN1_2], \
    }
#endif
#ifdef MOTOR_DJ_M3508_ID3_CAN1
#define MOTOR_DJ_M3508_ID3_CAN1_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN1_3], \
    }
#endif
#ifdef MOTOR_DJ_M3508_ID4_CAN1

#define MOTOR_DJ_M3508_ID4_CAN1_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN1_4], \
    }
#endif
#ifdef MOTOR_DJ_M3508_ID5_CAN1
#define MOTOR_DJ_M3508_ID5_CAN1_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN1_5], \
    }
#endif

#ifdef MOTOR_DJ_M3508_ID6_CAN1
#define MOTOR_DJ_M3508_ID6_CAN1_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN1_6], \
    }

#endif
#ifdef MOTOR_DJ_M3508_ID7_CAN1
#define MOTOR_DJ_M3508_ID7_CAN1_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN1_7], \
    }
#endif
#ifdef MOTOR_DJ_M3508_ID8_CAN1
#define MOTOR_DJ_M3508_ID8_CAN1_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN1_8], \
    }
#endif
#ifdef MOTOR_DJ_M3508_ID1_CAN2
#define MOTOR_DJ_M3508_ID1_CAN2_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN2_1], \
    }
#endif
#ifdef MOTOR_DJ_M3508_ID2_CAN2
#define MOTOR_DJ_M3508_ID2_CAN2_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN2_2], \
    }
#endif

#ifdef MOTOR_DJ_M3508_ID3_CAN2
#define MOTOR_DJ_M3508_ID3_CAN2_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN2_3], \
    }
#endif
#ifdef MOTOR_DJ_M3508_ID4_CAN2
#define MOTOR_DJ_M3508_ID4_CAN2_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN2_4], \
    }
#endif
#ifdef MOTOR_DJ_M3508_ID5_CAN2
#define MOTOR_DJ_M3508_ID5_CAN2_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN2_5], \
    }
#endif
#ifdef MOTOR_DJ_M3508_ID6_CAN2
#define MOTOR_DJ_M3508_ID6_CAN2_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN2_6], \
    }
#endif
#ifdef MOTOR_DJ_M3508_ID7_CAN2
#define MOTOR_DJ_M3508_ID7_CAN2_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN2_7], \
    }
#endif
#ifdef MOTOR_DJ_M3508_ID8_CAN2
#define MOTOR_DJ_M3508_ID8_CAN2_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN2_8], \
    }
#endif

#ifdef MOTOR_DJ_M2006_ID1_CAN1
#define MOTOR_DJ_M2006_ID1_CAN1_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN1_1], \
    }
#endif
#ifdef MOTOR_DJ_M2006_ID2_CAN1
#define MOTOR_DJ_M2006_ID2_CAN1_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN1_2], \
    }
#endif
#ifdef MOTOR_DJ_M2006_ID3_CAN1
#define MOTOR_DJ_M2006_ID3_CAN1_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN1_3], \
    }
#endif
#ifdef MOTOR_DJ_M2006_ID4_CAN1
#define MOTOR_DJ_M2006_ID4_CAN1_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN1_4], \
    }
#endif
#ifdef MOTOR_DJ_M2006_ID5_CAN1
#define MOTOR_DJ_M2006_ID5_CAN1_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN1_5], \
    }
#endif
#ifdef MOTOR_DJ_M2006_ID6_CAN1
#define MOTOR_DJ_M2006_ID6_CAN1_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN1_6], \
    }
#endif
#ifdef MOTOR_DJ_M2006_ID7_CAN1
#define MOTOR_DJ_M2006_ID7_CAN1_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN1_7], \
    }
#endif
#ifdef MOTOR_DJ_M2006_ID8_CAN1
#define MOTOR_DJ_M2006_ID8_CAN1_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN1_8], \
    }
#endif
#ifdef MOTOR_DJ_M2006_ID1_CAN2
#define MOTOR_DJ_M2006_ID1_CAN2_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN2_1], \
    }
#endif
#ifdef MOTOR_DJ_M2006_ID2_CAN2
#define MOTOR_DJ_M2006_ID2_CAN2_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN2_2], \
    }
#endif
#ifdef MOTOR_DJ_M2006_ID3_CAN2
#define MOTOR_DJ_M2006_ID3_CAN2_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN2_3], \
    }
#endif
#ifdef MOTOR_DJ_M2006_ID4_CAN2
#define MOTOR_DJ_M2006_ID4_CAN2_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN2_4], \
    }
#endif
#ifdef MOTOR_DJ_M2006_ID5_CAN2
#define MOTOR_DJ_M2006_ID5_CAN2_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN2_5], \
    }
#endif
#ifdef MOTOR_DJ_M2006_ID6_CAN2
#define MOTOR_DJ_M2006_ID6_CAN2_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN2_6], \
    }
#endif
#ifdef MOTOR_DJ_M2006_ID7_CAN2
#define MOTOR_DJ_M2006_ID7_CAN2_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN2_7], \
    }
#endif
#ifdef MOTOR_DJ_M2006_ID8_CAN2
#define MOTOR_DJ_M2006_ID8_CAN2_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN2_8], \
    }
#endif
#ifdef MOTOR_DJ_M6020_ID1_CAN1
#define MOTOR_DJ_M6020_ID1_CAN1_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN1_5], \
    }
#endif
#ifdef MOTOR_DJ_M6020_ID2_CAN1
#define MOTOR_DJ_M6020_ID2_CAN1_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN1_6], \
    }
#endif
#ifdef MOTOR_DJ_M6020_ID3_CAN1
#define MOTOR_DJ_M6020_ID3_CAN1_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN1_7], \
    }
#endif
#ifdef MOTOR_DJ_M6020_ID4_CAN1
#define MOTOR_DJ_M6020_ID4_CAN1_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN1_8], \
    }
#endif
#ifdef MOTOR_DJ_M6020_ID5_CAN1
#define MOTOR_DJ_M6020_ID5_CAN1_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN1_9], \
    }
#endif
#ifdef MOTOR_DJ_M6020_ID6_CAN1
#define MOTOR_DJ_M6020_ID6_CAN1_OPS            \
    {                                          \
        .driver = motor_dj_driver,             \
        .control = motor_dj_ctr,               \
        .user_data = &dj_motors[DJ_M_CAN1_10], \
    }
#endif
#ifdef MOTOR_DJ_M6020_ID7_CAN1
#define MOTOR_DJ_M6020_ID7_CAN1_OPS            \
    {                                          \
        .driver = motor_dj_driver,             \
        .control = motor_dj_ctr,               \
        .user_data = &dj_motors[DJ_M_CAN1_11], \
    }
#endif
#ifdef MOTOR_DJ_M6020_ID8_CAN1
#define MOTOR_DJ_M6020_ID8_CAN1_OPS            \
    {                                          \
        .driver = motor_dj_driver,             \
        .control = motor_dj_ctr,               \
        .user_data = &dj_motors[DJ_M_CAN1_12], \
    }
#endif
#ifdef MOTOR_DJ_M6020_ID1_CAN2
#define MOTOR_DJ_M6020_ID1_CAN2_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN2_5], \
    }
#endif
#ifdef MOTOR_DJ_M6020_ID2_CAN2
#define MOTOR_DJ_M6020_ID2_CAN2_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN2_6], \
    }
#endif
#ifdef MOTOR_DJ_M6020_ID3_CAN2
#define MOTOR_DJ_M6020_ID3_CAN2_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN2_7], \
    }
#endif
#ifdef MOTOR_DJ_M6020_ID4_CAN2
#define MOTOR_DJ_M6020_ID4_CAN2_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN2_8], \
    }
#endif
#ifdef MOTOR_DJ_M6020_ID5_CAN2
#define MOTOR_DJ_M6020_ID5_CAN2_OPS           \
    {                                         \
        .driver = motor_dj_driver,            \
        .control = motor_dj_ctr,              \
        .user_data = &dj_motors[DJ_M_CAN2_9], \
    }
#endif
#ifdef MOTOR_DJ_M6020_ID6_CAN2
#define MOTOR_DJ_M6020_ID6_CAN2_OPS            \
    {                                          \
        .driver = motor_dj_driver,             \
        .control = motor_dj_ctr,               \
        .user_data = &dj_motors[DJ_M_CAN2_10], \
    }
#endif
#ifdef MOTOR_DJ_M6020_ID7_CAN2
#define MOTOR_DJ_M6020_ID7_CAN2_OPS            \
    {                                          \
        .driver = motor_dj_driver,             \
        .control = motor_dj_ctr,               \
        .user_data = &dj_motors[DJ_M_CAN2_11], \
    }
#endif
#ifdef MOTOR_DJ_M6020_ID8_CAN2
#define MOTOR_DJ_M6020_ID8_CAN2_OPS            \
    {                                          \
        .driver = motor_dj_driver,             \
        .control = motor_dj_ctr,               \
        .user_data = &dj_motors[DJ_M_CAN2_12], \
    }
#endif

#ifdef __cplusplus
}
#endif
#endif
