/**
 * @file motor_cfg.h
 * @author Dyyt587 (67887002+Dyyt587@users.noreply.github.com)
 * @brief
 * @version 0.1
 * @date 2024-03-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef _MOTOR_CFG_H
#define _MOTOR_CFG_H
#ifdef __cplusplus
extern "C"
{
#endif

#ifndef NULL
#define NULL 0
#endif
/**
 * @brief 用于初始化电机的操作函数和支持功能的宏函数
 *
 */
#define MOTOR_INIT_OPS_PID_ALL(_index, __ops, __level, _torque_pid, _speed_pid, _pos_pid) [_index] = {.name=#_index, .ops = (&((motor_ops_t)__ops)), .flag_accept_level = __level, .pid_torque = _torque_pid, .pid_speed = _speed_pid, .pid_pos = _pos_pid}

#define MOTOR_INIT_OPS(_index, _ops, _level)  MOTOR_INIT_OPS_PID_ALL(_index, _ops, (_level), (NULL), (NULL), (NULL))
#define MOTOR_INIT_OPS_PID_POS(index, __ops, __level, _pos_pid) MOTOR_INIT_OPS_PID_ALL(index, __ops, __level, (NULL), (NULL), _pos_pid)
#define MOTOR_INIT_OPS_PID_SPEED(index, __ops, __level, _speed_pid, _pos_pid) MOTOR_INIT_OPS_PID_ALL(index, __ops, __level, (NULL), _speed_pid, _pos_pid)

#define MOTOR_ID_GET_OFFSET(index,start) ((index) - (start)-1)

#define MOTOR_OPS_CURVE_INIT() (&((CurveObjectType){.aTimes=0,\
}))

	#define MOTOR_DJ_MOTOR 0
  #define MOTOR_EMM_V5_MOTOR 0
  #define MOTOR_MIT_MOTOR 1
    /**
     * @brief 配置电机是否启用的宏，如果启用则定义，否则注释
     *
     */
# if MOTOR_MIT_MOTOR
     #define MOTOR_MIT_ID1_CAN1
     #define MOTOR_MIT_ID2_CAN1
//     #define MOTOR_MIT_ID3_CAN1
#endif
#if MOTOR_DJ_MOTOR
     #define MOTOR_DJ_M3508_ID1_CAN1
     #define MOTOR_DJ_M3508_ID2_CAN1
     #define MOTOR_DJ_M3508_ID3_CAN1
     #define MOTOR_DJ_M3508_ID4_CAN1
     #define MOTOR_DJ_M3508_ID5_CAN1
     #define MOTOR_DJ_M3508_ID6_CAN1
     #define MOTOR_DJ_M3508_ID7_CAN1
    // #define MOTOR_DJ_M3508_ID8_CAN1
    // #define MOTOR_DJ_M3508_ID1_CAN2
    // #define MOTOR_DJ_M3508_ID2_CAN2
    // #define MOTOR_DJ_M3508_ID3_CAN2
    // #define MOTOR_DJ_M3508_ID4_CAN2
    // #define MOTOR_DJ_M3508_ID5_CAN2
    // #define MOTOR_DJ_M3508_ID6_CAN2
    // #define MOTOR_DJ_M3508_ID7_CAN2
    // #define MOTOR_DJ_M3508_ID8_CAN2

// #define MOTOR_DJ_M2006_ID1_CAN1
// #define MOTOR_DJ_M2006_ID2_CAN1
// #define MOTOR_DJ_M2006_ID3_CAN1
// #define MOTOR_DJ_M2006_ID4_CAN1
// #define MOTOR_DJ_M2006_ID5_CAN1
    // #define MOTOR_DJ_M2006_ID6_CAN1
    // #define MOTOR_DJ_M2006_ID7_CAN1
    // #define MOTOR_DJ_M2006_ID8_CAN1
    // #define MOTOR_DJ_M2006_ID1_CAN2
    // #define MOTOR_DJ_M2006_ID2_CAN2
    // #define MOTOR_DJ_M2006_ID3_CAN2
    // #define MOTOR_DJ_M2006_ID4_CAN2
    // #define MOTOR_DJ_M2006_ID5_CAN2
    // #define MOTOR_DJ_M2006_ID6_CAN2
    // #define MOTOR_DJ_M2006_ID7_CAN2
    // #define MOTOR_DJ_M2006_ID8_CAN2

    // #define MOTOR_DJ_M6020_ID1_CAN1
    // #define MOTOR_DJ_M6020_ID2_CAN1
    // #define MOTOR_DJ_M6020_ID3_CAN1
    // #define MOTOR_DJ_M6020_ID4_CAN1

    // #define MOTOR_DJ_M6020_ID5_CAN1
    // #define MOTOR_DJ_M6020_ID6_CAN1
    // #define MOTOR_DJ_M6020_ID7_CAN1
    // #define MOTOR_DJ_M6020_ID8_CAN1

    // #define MOTOR_DJ_M6020_ID1_CAN2
    // #define MOTOR_DJ_M6020_ID2_CAN2
    // #define MOTOR_DJ_M6020_ID3_CAN2
    // #define MOTOR_DJ_M6020_ID4_CAN2

    // #define MOTOR_DJ_M6020_ID5_CAN2
    // #define MOTOR_DJ_M6020_ID6_CAN2
    // #define MOTOR_DJ_M6020_ID7_CAN2
    // #define MOTOR_DJ_M6020_ID8_CAN2
    /**
     * 配置项目结束
     */
#endif
#if MOTOR_EMM_V5_MOTOR
     #define MOTOR_EMMV5_ID1_SERIAL1
//      #define MOTOR_EMMV5_ID2_SERIAL1
//      #define MOTOR_EMMV5_ID3_SERIAL1
//      #define MOTOR_EMMV5_ID4_SERIAL1
//      #define MOTOR_EMMV5_ID5_SERIAL1
//      #define MOTOR_EMMV5_ID6_SERIAL1

#endif

    /**
     * @brief 电机抽象层电机id索引表
     */
    enum
    {
#if MOTOR_DJ_MOTOR
			
#ifdef MOTOR_DJ_M3508_ID1_CAN1
        M3508_1_CAN1,
#endif
#ifdef MOTOR_DJ_M3508_ID2_CAN1
        M3508_2_CAN1,
#endif
#ifdef MOTOR_DJ_M3508_ID3_CAN1
        M3508_3_CAN1,
#endif
#ifdef MOTOR_DJ_M3508_ID4_CAN1
        M3508_4_CAN1,
#endif
#ifdef MOTOR_DJ_M3508_ID5_CAN1
        M3508_5_CAN1,
#endif
#ifdef MOTOR_DJ_M3508_ID6_CAN1
        M3508_6_CAN1,
#endif
#ifdef MOTOR_DJ_M3508_ID7_CAN1
        M3508_7_CAN1,
#endif
#ifdef MOTOR_DJ_M3508_ID8_CAN1
        M3508_8_CAN1,
#endif
#ifdef MOTOR_DJ_M3508_ID1_CAN2
        M3508_1_CAN2,
#endif
#ifdef MOTOR_DJ_M3508_ID2_CAN2
        M3508_2_CAN2,
#endif
#ifdef MOTOR_DJ_M3508_ID3_CAN2
        M3508_3_CAN2,
#endif
#ifdef MOTOR_DJ_M3508_ID4_CAN2
        M3508_4_CAN2,
#endif
#ifdef MOTOR_DJ_M3508_ID5_CAN2
        M3508_5_CAN2,
#endif
#ifdef MOTOR_DJ_M3508_ID6_CAN2
        M3508_6_CAN2,
#endif
#ifdef MOTOR_DJ_M3508_ID7_CAN2
        M3508_7_CAN2,
#endif
#ifdef MOTOR_DJ_M3508_ID8_CAN2
        M3508_8_CAN2,
#endif

#ifdef MOTOR_DJ_M2006_ID1_CAN1
        M2006_1_CAN1,
#endif
#ifdef MOTOR_DJ_M2006_ID2_CAN1
        M2006_2_CAN1,
#endif
#ifdef MOTOR_DJ_M2006_ID3_CAN1
        M2006_3_CAN1,
#endif
#ifdef MOTOR_DJ_M2006_ID4_CAN1
        M2006_4_CAN1,
#endif
#ifdef MOTOR_DJ_M2006_ID5_CAN1
        M2006_5_CAN1,
#endif
#ifdef MOTOR_DJ_M2006_ID6_CAN1
        M2006_6_CAN1,
#endif
#ifdef MOTOR_DJ_M2006_ID7_CAN1
        M2006_7_CAN1,
#endif
#ifdef MOTOR_DJ_M2006_ID8_CAN1
        M2006_8_CAN1,
#endif

#ifdef MOTOR_DJ_M2006_ID1_CAN2
        M2006_1_CAN2,
#endif
#ifdef MOTOR_DJ_M2006_ID2_CAN2
        M2006_2_CAN2,
#endif
#ifdef MOTOR_DJ_M2006_ID3_CAN2
        M2006_3_CAN2,
#endif
#ifdef MOTOR_DJ_M2006_ID4_CAN2
        M2006_4_CAN2,
#endif
#ifdef MOTOR_DJ_M2006_ID5_CAN2
        M2006_5_CAN2,
#endif
#ifdef MOTOR_DJ_M2006_ID6_CAN2
        M2006_6_CAN2,
#endif
#ifdef MOTOR_DJ_M2006_ID7_CAN2
        M2006_7_CAN2,
#endif
#ifdef MOTOR_DJ_M2006_ID8_CAN2
        M2006_8_CAN2,
#endif

#ifdef MOTOR_DJ_M6020_ID1_CAN1
        M6020_1_CAN1,
#endif
#ifdef MOTOR_DJ_M6020_ID2_CAN1
        M6020_2_CAN1,
#endif
#ifdef MOTOR_DJ_M6020_ID3_CAN1
        M6020_3_CAN1,
#endif
#ifdef MOTOR_DJ_M6020_ID4_CAN1
        M6020_4_CAN1,
#endif
#ifdef MOTOR_DJ_M6020_ID5_CAN1
        M6020_5_CAN1,
#endif
#ifdef MOTOR_DJ_M6020_ID6_CAN1
        M6020_6_CAN1,
#endif
#ifdef MOTOR_DJ_M6020_ID7_CAN1
        M6020_7_CAN1,
#endif
#ifdef MOTOR_DJ_M6020_ID8_CAN1
        M6020_8_CAN1,
#endif

#ifdef MOTOR_DJ_M6020_ID1_CAN2
        M6020_1_CAN2,
#endif
#ifdef MOTOR_DJ_M6020_ID2_CAN2
        M6020_2_CAN2,
#endif
#ifdef MOTOR_DJ_M6020_ID3_CAN2
        M6020_3_CAN2,
#endif
#ifdef MOTOR_DJ_M6020_ID4_CAN2
        M6020_4_CAN2,
#endif
#ifdef MOTOR_DJ_M6020_ID5_CAN2
        M6020_5_CAN2,
#endif
#ifdef MOTOR_DJ_M6020_ID6_CAN2
        M6020_6_CAN2,
#endif
#ifdef MOTOR_DJ_M6020_ID7_CAN2
        M6020_7_CAN2,
#endif
#ifdef MOTOR_DJ_M6020_ID8_CAN2
        M6020_8_CAN2,
#endif
#endif

#if MOTOR_EMM_V5_MOTOR
#ifdef MOTOR_EMMV5_ID1_SERIAL1
    MSP_1_SERIAL1,
#endif
#ifdef MOTOR_EMMV5_ID2_SERIAL1
    MSP_2_SERIAL1,
#endif
#ifdef MOTOR_EMMV5_ID3_SERIAL1
    MSP_3_SERIAL1,
#endif
#ifdef MOTOR_EMMV5_ID4_SERIAL1
    MSP_4_SERIAL1,
#endif
#ifdef MOTOR_EMMV5_ID5_SERIAL1
    MSP_5_SERIAL1,
#endif
#ifdef MOTOR_EMMV5_ID6_SERIAL1
    MSP_6_SERIAL1,
#endif
#endif
#if MOTOR_MIT_MOTOR
#ifdef MOTOR_MIT_ID1_CAN1
    MMIT_1_CAN1,
#endif
#ifdef MOTOR_MIT_ID2_CAN1
    MMIT_2_CAN1,
#endif
#endif
        MOTOR_NUM
    };


#ifdef __cplusplus
}
#endif
#endif
