/*
 * @Author: Dyyt587 805207319@qq.com
 * @Date: 2024-03-03 17:44:36
 * @LastEditors: Dyyt587 67887002+Dyyt587@users.noreply.github.com
 * @LastEditTime: 2024-06-19 17:40:48
 * @FilePath: \project\applications\motor_dj_rm_driver.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef MOTOR_DJ_RM_DRIVER_H
#define MOTOR_DJ_RM_DRIVER_H
#if defined(__cplusplus)
extern "C" {
#endif
#include "apid.h"
#include "motor.h"
#include <stdint.h>

#define DJ_MOTOR_NUMBER 11
#define FILTER_BUF_LEN        5

enum{
    #if defined(MOTOR_DJ_M3508_ID1_CAN1) || defined(MOTOR_DJ_M2006_ID1_CAN1)
        DJ_M_CAN1_1,
    #endif
    #if defined(MOTOR_DJ_M3508_ID2_CAN1) || defined(MOTOR_DJ_M2006_ID2_CAN1)
        DJ_M_CAN1_2,
    #endif
    #if defined(MOTOR_DJ_M3508_ID3_CAN1) || defined(MOTOR_DJ_M2006_ID3_CAN1)
        DJ_M_CAN1_3,
    #endif
    
    #if defined(MOTOR_DJ_M3508_ID4_CAN1) || defined(MOTOR_DJ_M2006_ID4_CAN1)
        DJ_M_CAN1_4,
    #endif
    #if defined(MOTOR_DJ_M3508_ID5_CAN1) || defined(MOTOR_DJ_M2006_ID5_CAN1)||defined(MOTOR_DJ_M6020_ID1_CAN1)
        DJ_M_CAN1_5,
    #endif
    #if defined(MOTOR_DJ_M3508_ID6_CAN1) || defined(MOTOR_DJ_M2006_ID6_CAN1)||defined(MOTOR_DJ_M6020_ID2_CAN1)
        DJ_M_CAN1_6,
    #endif
    #if defined(MOTOR_DJ_M3508_ID7_CAN1) || defined(MOTOR_DJ_M2006_ID7_CAN1)||defined(MOTOR_DJ_M6020_ID3_CAN1)
        DJ_M_CAN1_7,
    #endif
    #if defined(MOTOR_DJ_M3508_ID8_CAN1) || defined(MOTOR_DJ_M2006_ID8_CAN1)||defined(MOTOR_DJ_M6020_ID4_CAN1)
        DJ_M_CAN1_8,
    #endif

    #if defined(MOTOR_DJ_M6020_ID5_CAN1)
        DJ_M_CAN1_9,
    #endif
    #if defined(MOTOR_DJ_M6020_ID6_CAN1)
        DJ_M_CAN1_10,
    #endif
    #if defined(MOTOR_DJ_M6020_ID7_CAN1)
        DJ_M_CAN1_11,
    #endif
    #if defined(MOTOR_DJ_M6020_ID8_CAN1)
        DJ_M_CAN1_12,
    #endif
    
    #if defined(MOTOR_DJ_M3508_ID1_CAN2) || defined(MOTOR_DJ_M2006_ID1_CAN2)
        DJ_M_CAN2_1,
    #endif
    #if defined(MOTOR_DJ_M3508_ID2_CAN2) || defined(MOTOR_DJ_M2006_ID2_CAN2)
        DJ_M_CAN2_2,
    #endif
    #if defined(MOTOR_DJ_M3508_ID3_CAN2) || defined(MOTOR_DJ_M2006_ID3_CAN2)
        DJ_M_CAN2_3,
    #endif
    #if defined(MOTOR_DJ_M3508_ID4_CAN2) || defined(MOTOR_DJ_M2006_ID4_CAN2)
        DJ_M_CAN2_4,
    #endif

    #if defined(MOTOR_DJ_M3508_ID5_CAN2) || defined(MOTOR_DJ_M2006_ID5_CAN2)||defined(MOTOR_DJ_M6020_ID1_CAN2)
        DJ_M_CAN2_5,
    #endif
    #if defined(MOTOR_DJ_M3508_ID6_CAN2) || defined(MOTOR_DJ_M2006_ID6_CAN2)||defined(MOTOR_DJ_M6020_ID2_CAN2)
        DJ_M_CAN2_6,
    #endif
    #if defined(MOTOR_DJ_M3508_ID7_CAN2) || defined(MOTOR_DJ_M2006_ID7_CAN2)||defined(MOTOR_DJ_M6020_ID3_CAN2)
        DJ_M_CAN2_7,
    #endif
    #if defined(MOTOR_DJ_M3508_ID8_CAN2) || defined(MOTOR_DJ_M2006_ID8_CAN2)||defined(MOTOR_DJ_M6020_ID4_CAN2)
        DJ_M_CAN2_8,
    #endif

    #if defined(MOTOR_DJ_M6020_ID5_CAN2)
        DJ_M_CAN2_9,
    #endif
    #if defined(MOTOR_DJ_M6020_ID6_CAN2)
        DJ_M_CAN2_10,
    #endif
    #if defined(MOTOR_DJ_M6020_ID7_CAN2)
        DJ_M_CAN2_11,
    #endif
    #if defined(MOTOR_DJ_M6020_ID8_CAN2)
        DJ_M_CAN2_12,
    #endif

    DJ_M_NUM

};

typedef struct {
    uint8_t id;           //对于motor抽象层的id
    uint16_t can_id;
    uint16_t angle;         //abs angle range:[0,8191] 角度范围[0,8191]
    uint8_t msg_cnt;            //初始化计数 小于50清零,同时兼顾数据更新标准位


    uint16_t last_angle;    //abs angle range:[0,8191]
    uint16_t offset_angle;  //偏差
   //uint8_t temperature;    //温度

    int32_t round_cnt;      //电机转的圈数
    int32_t total_angle;    //当前总角度 圈数

} motor_measure_t;
extern motor_measure_t dj_motors[DJ_M_NUM] ;

/*CAN发送或是接收的ID*/
typedef enum {

    CAN_Motor_ALL_ID = 0x200,
    CAN_Motor1_ID = 0x201,
    CAN_Motor2_ID = 0x202,
    CAN_Motor3_ID = 0x203,
    CAN_Motor4_ID = 0x204,

    CAN_Motor_ALL_ID2 = 0x1FF,
    CAN_Motor5_ID = 0x205,
    CAN_Motor6_ID = 0x206,
    CAN_Motor7_ID = 0x207,
    CAN_Motor8_ID = 0x208,

    CAN_6020_ALL_ID = 0X1FF,
    CAN_6020_ID1 = 0x205,
    CAN_6020_ID2 = 0x206,
    CAN_6020_ID3 = 0x207,
    CAN_6020_ID4 = 0x208,
    //6020的id 1234和 2006&3508的id5678重复
    CAN_6020_ALL_ID2 = 0X2FF,
    CAN_6020_ID5 = 0x209,
    CAN_6020_ID6 = 0x20A,
    CAN_6020_ID7 = 0x20B,
    CAN_6020_ID8 = 0x20C,



} DJ_Motor_Message_ID;
// can1的电机
extern motor_measure_t motor_can1[DJ_MOTOR_NUMBER]; 
// can2的电机
extern motor_measure_t motor_can2[DJ_MOTOR_NUMBER];

int motor_dj_ctr(int id,uint16_t mode,float*data);
int motor_dj_driver (int id, uint16_t mode,float* value, void* user_data);

#if defined(__cplusplus)
}
#endif
#endif