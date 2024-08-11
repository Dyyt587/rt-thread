/**
 * @file motor_dj_rm_driver.c
 * @author Dyyt587 (805207319@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-03
 *
 * @copyright Copyright (c) 2024
 *
 */

/*
 * 程序清单：这是一个 CAN 设备使用例程
 * 例程导出了 can_test 命令到控制终端
 * 命令调用格式：can_test can1
 * 命令解释：命令第二个参数是要使用的 CAN 设备名称，为空则使用默认的 CAN 设备
 * 程序功能：通过 CAN 设备发送一帧，并创建一个线程接收数据然后打印输出。
 */

#include <board.h>
#include <rtthread.h>
#include <drv_gpio.h>
#ifndef RT_USING_NANO
#include <rtdevice.h>
#endif /* RT_USING_NANO */
#include "apid.h"
#include "motor_dj_rm_driver.h"
#include "ulog.h"
#include "motor_cfg.h"
#include "perf_counter.h"

#define pi (3.1415926f)
#define CAN_DEV_NAME "can1" /* CAN 设备名称 */

static struct rt_semaphore rx_sem; /* 用于接收消息的信号量 */
static rt_device_t can_dev;        /* CAN 设备句柄 */
static struct rt_ringbuffer *dj_m_ringfifo;
static rt_timer_t tmr1 = RT_NULL;
extern motor_t motor_list[MOTOR_NUM];
#define DJ_MOTOR_MOTOR_ID(index, __id, __can_id) [index] = {             \
                                                     .id = __id,         \
                                                     .can_id = __can_id, \
}
// #define DJ_MOTOR_MOTOR_ID(index, __id, __can_id) [index] = {             \
//                                                      .can_id = __can_id, \
//                                                      .motor = &motor_list[__id],\
// }


motor_measure_t dj_motors[DJ_M_NUM] = {
#ifdef MOTOR_DJ_M3508_ID1_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_1, M3508_1_CAN1, CAN_Motor1_ID),
#endif
#ifdef MOTOR_DJ_M3508_ID2_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_2, M3508_2_CAN1, CAN_Motor2_ID),
#endif
#ifdef MOTOR_DJ_M3508_ID3_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_3, M3508_3_CAN1, CAN_Motor3_ID),
#endif
#ifdef MOTOR_DJ_M3508_ID4_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_4, M3508_4_CAN1, CAN_Motor4_ID),
#endif
#ifdef MOTOR_DJ_M3508_ID5_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_5, M3508_5_CAN1, CAN_Motor5_ID),
#endif

#ifdef MOTOR_DJ_M3508_ID6_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_6, M3508_6_CAN1, CAN_Motor6_ID),
#endif
#ifdef MOTOR_DJ_M3508_ID7_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_7, M3508_7_CAN1, CAN_Motor7_ID),
#endif
#ifdef MOTOR_DJ_M3508_ID8_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_8, M3508_8_CAN1, CAN_Motor8_ID),
#endif
#ifdef MOTOR_DJ_M3508_ID1_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_1, M3508_1_CAN2, CAN_Motor1_ID),
#endif
#ifdef MOTOR_DJ_M3508_ID2_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_2, M3508_2_CAN2, CAN_Motor2_ID),
#endif
#ifdef MOTOR_DJ_M3508_ID3_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_3, M3508_3_CAN2, CAN_Motor3_ID),
#endif
#ifdef MOTOR_DJ_M3508_ID4_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_4, M3508_4_CAN2, CAN_Motor4_ID),
#endif
#ifdef MOTOR_DJ_M3508_ID5_CAN2

    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_5, M3508_5_CAN2, CAN_Motor5_ID),
#endif
#ifdef MOTOR_DJ_M3508_ID6_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_6, M3508_6_CAN2, CAN_Motor6_ID),
#endif
#ifdef MOTOR_DJ_M3508_ID7_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_7, M3508_7_CAN2, CAN_Motor7_ID),
#endif
#ifdef MOTOR_DJ_M3508_ID8_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_8, M3508_8_CAN2, CAN_Motor8_ID),
#endif
#ifdef MOTOR_DJ_M2006_ID1_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_1, M2006_1_CAN1, CAN_Motor1_ID),
#endif
#ifdef MOTOR_DJ_M2006_ID2_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_2, M2006_2_CAN1, CAN_Motor2_ID),
#endif
#ifdef MOTOR_DJ_M2006_ID3_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_3, M2006_3_CAN1, CAN_Motor3_ID),
#endif
#ifdef MOTOR_DJ_M2006_ID4_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_4, M2006_4_CAN1, CAN_Motor4_ID),
#endif
#ifdef MOTOR_DJ_M2006_ID5_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_5, M2006_5_CAN1, CAN_Motor5_ID),
#endif
#ifdef MOTOR_DJ_M2006_ID6_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_6, M2006_6_CAN1, CAN_Motor6_ID),
#endif
#ifdef MOTOR_DJ_M2006_ID7_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_7, M2006_7_CAN1, CAN_Motor7_ID),
#endif
#ifdef MOTOR_DJ_M2006_ID8_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_8, M2006_8_CAN1, CAN_Motor8_ID),
#endif

#ifdef MOTOR_DJ_M2006_ID1_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_1, M2006_1_CAN2, CAN_Motor1_ID),
#endif
#ifdef MOTOR_DJ_M2006_ID2_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_2, M2006_2_CAN2, CAN_Motor2_ID),
#endif
#ifdef MOTOR_DJ_M2006_ID3_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_3, M2006_3_CAN2, CAN_Motor3_ID),
#endif
#ifdef MOTOR_DJ_M2006_ID4_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_4, M2006_4_CAN2, CAN_Motor4_ID),
#endif
#ifdef MOTOR_DJ_M2006_ID5_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_5, M2006_5_CAN2, CAN_Motor5_ID),
#endif
#ifdef MOTOR_DJ_M2006_ID6_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_6, M2006_6_CAN2, CAN_Motor6_ID),
#endif
#ifdef MOTOR_DJ_M2006_ID7_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_7, M2006_7_CAN2, CAN_Motor7_ID),
#endif
#ifdef MOTOR_DJ_M2006_ID8_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_8, M2006_8_CAN2, CAN_Motor8_ID),
#endif

#ifdef MOTOR_DJ_M6020_ID1_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_5, M6020_1_CAN1, CAN_6020_ID1),
#endif
#ifdef MOTOR_DJ_M6020_ID2_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_6, M6020_2_CAN1, CAN_6020_ID2),
#endif
#ifdef MOTOR_DJ_M6020_ID3_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_7, M6020_3_CAN1, CAN_6020_ID3),
#endif
#ifdef MOTOR_DJ_M6020_ID4_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_8, M6020_4_CAN1, CAN_6020_ID4),
#endif
#ifdef MOTOR_DJ_M6020_ID5_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_9, M6020_5_CAN1, CAN_6020_ID5),
#endif
#ifdef MOTOR_DJ_M6020_ID6_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_10, M6020_6_CAN1, CAN_6020_ID6),
#endif
#ifdef MOTOR_DJ_M6020_ID7_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_11, M6020_7_CAN1, CAN_6020_ID7),
#endif
#ifdef MOTOR_DJ_M6020_ID8_CAN1
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN1_12, M6020_8_CAN1, CAN_6020_ID8),
#endif

#ifdef MOTOR_DJ_M6020_ID1_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_5, M6020_1_CAN2, CAN_6020_ID1),
#endif
#ifdef MOTOR_DJ_M6020_ID2_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_6, M6020_2_CAN2, CAN_6020_ID2),
#endif
#ifdef MOTOR_DJ_M6020_ID3_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_7, M6020_3_CAN2, CAN_6020_ID3),
#endif
#ifdef MOTOR_DJ_M6020_ID4_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_8, M6020_4_CAN2, CAN_6020_ID4),
#endif
#ifdef MOTOR_DJ_M6020_ID5_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_9, M6020_5_CAN2, CAN_6020_ID5),
#endif
#ifdef MOTOR_DJ_M6020_ID6_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_10, M6020_6_CAN2, CAN_6020_ID6),
#endif
#ifdef MOTOR_DJ_M6020_ID7_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_11, M6020_7_CAN2, CAN_6020_ID7),
#endif
#ifdef MOTOR_DJ_M6020_ID8_CAN2
    DJ_MOTOR_MOTOR_ID(DJ_M_CAN2_12, M6020_8_CAN2, CAN_6020_ID8),
#endif

};
// 写一个二分查找,通过can_id查找到电机对应的结构体
motor_measure_t *motor_get_by_canid(uint16_t can_id)
{
    uint8_t left = 0, right = DJ_M_NUM - 1;
    uint8_t mid;
    while (left <= right)
    {
        mid = (left + right) / 2;
        if (dj_motors[mid].can_id == can_id)
            return &dj_motors[mid];
        else if (dj_motors[mid].can_id < can_id)
            left = mid + 1;
        else
            right = mid - 1;
    }
    LOG_E("can_id:%d not found", can_id);
    return RT_NULL;
}

// can1 的
int16_t iq1_low[4], iq1_high[4], iq1_uhigh[4];
// can2 的
int16_t iq2_low[4], iq2_high[4], iq2_uhigh[4];

int motor_dj_driver(int id, uint16_t mode, float *value, void *user_data)
{
    motor_t *motor = motor_get(id);
    struct rt_can_msg msg = {0};
    motor_measure_t *__motor = (motor_measure_t *)motor->ops->user_data;
    int16_t tmpout = *value;
    int time = 0;
    // LOG_D("id:%d mode:%d",id,mode);

    switch (mode)
    {
    case MOTOR_MODE_TORQUE:
        /*设置力矩/电流值*/
        /* 发送一帧 CAN 数据 */
        msg.ide = RT_CAN_STDID; /* 标准格式 */
        msg.rtr = RT_CAN_DTR;   /* 数据帧 */
        msg.len = 8;            /* 数据长度为 8 */
        uint16_t tt = (__motor->can_id - CAN_Motor1_ID) % 4;
        // LOG_D("canid:%d index %d out %f",__motor->can_id,tt,value);
        static int time = 0;
        msg.id = __motor->can_id;
        // LOG_D("rang%d",(msg.id - CAN_Motor1_ID));
        if ((msg.id - CAN_Motor1_ID) >= 8) // 6020
        {
            LOG_W("id%d not support 6020",msg.id);
        }
        else if ((msg.id - CAN_Motor1_ID) >= 4) // 6020 3508 2006
        {
            iq1_high[tt] = tmpout;
            // LOG_D("dewhdu");
            // LOG_D("cnt:%d",dj_motors[DJ_M_CAN1_5].msg_cnt);

            if (
#ifdef MOTOR_DJ_M2006_ID5_CAN1
                (dj_motors[DJ_M_CAN1_5].msg_cnt == 0xff) &&
#endif
#ifdef MOTOR_DJ_M2006_ID6_CAN1
                dj_motors[DJ_M_CAN1_6].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M2006_ID7_CAN1
                dj_motors[DJ_M_CAN1_7].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M2006_ID8_CAN1
                dj_motors[DJ_M_CAN1_8].msg_cnt == 0xff &&
#endif

#ifdef MOTOR_DJ_M2006_ID5_CAN2
                dj_motors[DJ_M_CAN2_5].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M2006_ID6_CAN2
                dj_motors[DJ_M_CAN2_6].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M2006_ID7_CAN2
                dj_motors[DJ_M_CAN2_7].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M2006_ID8_CAN2
                dj_motors[DJ_M_CAN2_8].msg_cnt == 0xff &&
#endif

#ifdef MOTOR_DJ_M3508_ID5_CAN1
                dj_motors[DJ_M_CAN1_5].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M3508_ID6_CAN1
                dj_motors[DJ_M_CAN1_6].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M3508_ID7_CAN1
                dj_motors[DJ_M_CAN1_7].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M3508_ID8_CAN1
                dj_motors[DJ_M_CAN1_8].msg_cnt == 0xff &&
#endif

#ifdef MOTOR_DJ_M3508_ID5_CAN2
                dj_motors[DJ_M_CAN2_5].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M3508_ID6_CAN2
                dj_motors[DJ_M_CAN2_6].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M3508_ID7_CAN2
                dj_motors[DJ_M_CAN2_7].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M3508_ID8_CAN2
                dj_motors[DJ_M_CAN2_8].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M6020_ID1_CAN1
                dj_motors[DJ_M_CAN1_9].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M6020_ID2_CAN1
                dj_motors[DJ_M_CAN1_10].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M6020_ID3_CAN1
                dj_motors[DJ_M_CAN1_11].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M6020_ID4_CAN1
                dj_motors[DJ_M_CAN1_12].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M6020_ID5_CAN1
                dj_motors[DJ_M_CAN1_13].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M6020_ID6_CAN1
                dj_motors[DJ_M_CAN1_14].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M6020_ID7_CAN1
                dj_motors[DJ_M_CAN1_15].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M6020_ID8_CAN1
                dj_motors[DJ_M_CAN1_16].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M6020_ID1_CAN2
                dj_motors[DJ_M_CAN2_9].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M6020_ID2_CAN2
                dj_motors[DJ_M_CAN2_10].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M6020_ID3_CAN2
                dj_motors[DJ_M_CAN2_11].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M6020_ID4_CAN2
                dj_motors[DJ_M_CAN2_12].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M6020_ID5_CAN2
                dj_motors[DJ_M_CAN2_13].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M6020_ID6_CAN2
                dj_motors[DJ_M_CAN2_14].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M6020_ID7_CAN2
                dj_motors[DJ_M_CAN2_15].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M6020_ID8_CAN2
                dj_motors[DJ_M_CAN2_16].msg_cnt == 0xff &&
#endif
                1)
            {
                // LOG_D("%d",msg.id);

                /* 待发送的 8 字节数据 */
                msg.id = CAN_Motor_ALL_ID2;
                msg.data[0] = iq1_high[0] >> 8;
                msg.data[1] = iq1_high[0];
                msg.data[2] = iq1_high[1] >> 8;
                msg.data[3] = iq1_high[1];
                msg.data[4] = iq1_high[2] >> 8;
                msg.data[5] = iq1_high[2];
                msg.data[6] = iq1_high[3] >> 8;
                msg.data[7] = iq1_high[3];
                rt_ssize_t size;
                size = rt_device_write(can_dev, 0, &msg, sizeof(msg));
                if (size != sizeof(msg))
                {
                    LOG_E("can dev write data failed! %d\n", size);
                }
                else
                {
                    return 0;
                }
            }
            else
            {
                // TODO: 如果超时则汇报是那个电机超时并且出现问题
            }
        }
        else
        {
            if (
#ifdef MOTOR_DJ_M2006_ID1_CAN1
                dj_motors[DJ_M_CAN1_1].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M2006_ID2_CAN1
                dj_motors[DJ_M_CAN1_2].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M2006_ID3_CAN1
                    dj_motors[DJ_M_CAN1_3].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M2006_ID4_CAN1
                    dj_motors[DJ_M_CAN1_4].msg_cnt == 0xff &&
#endif

#ifdef MOTOR_DJ_M2006_ID1_CAN2
                    dj_motors[DJ_M_CAN2_1].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M2006_ID2_CAN2
                    dj_motors[DJ_M_CAN2_2].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M2006_ID3_CAN2
                    dj_motors[DJ_M_CAN2_3].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M2006_ID4_CAN2
                    dj_motors[DJ_M_CAN2_4].msg_cnt == 0xff &&
#endif

#ifdef MOTOR_DJ_M3508_ID1_CAN1
                    dj_motors[DJ_M_CAN1_1].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M3508_ID2_CAN1
                    dj_motors[DJ_M_CAN1_2].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M3508_ID3_CAN1
                    dj_motors[DJ_M_CAN1_3].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M3508_ID4_CAN1
                    dj_motors[DJ_M_CAN1_4].msg_cnt == 0xff &&
#endif

#ifdef MOTOR_DJ_M3508_ID1_CAN2
                    dj_motors[DJ_M_CAN2_1].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M3508_ID2_CAN2
                    dj_motors[DJ_M_CAN2_2].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M3508_ID3_CAN2
                    dj_motors[DJ_M_CAN2_3].msg_cnt == 0xff &&
#endif
#ifdef MOTOR_DJ_M3508_ID4_CAN2
                    dj_motors[DJ_M_CAN2_4].msg_cnt == 0xff &&
#endif
                    1)
            {
                iq1_low[tt] = tmpout;
                /* 待发送的 8 字节数据 */
                msg.id = CAN_Motor_ALL_ID;
                msg.data[0] = iq1_low[0] >> 8;
                msg.data[1] = iq1_low[0];
                msg.data[2] = iq1_low[1] >> 8;
                msg.data[3] = iq1_low[1];
                msg.data[4] = iq1_low[2] >> 8;
                msg.data[5] = iq1_low[2];
                msg.data[6] = iq1_low[3] >> 8;
                msg.data[7] = iq1_low[3];
                rt_ssize_t size;
                size = rt_device_write(can_dev, 0, &msg, sizeof(msg));
                if (size != sizeof(msg))
                {
                    LOG_E("can dev write failed!%d\n", size);
                }
                else
                {
                    return 0;
                }
                for (int i = 0; i < DJ_M_NUM; ++i)
                {
                    dj_motors[i].msg_cnt = 50;
                }
            }
        }

        break;
    case MOTOR_MODE_SPEED:
    case MOTOR_MODE_POS:
    default:
        LOG_E("invlide motor mode");
        break;
    }
    return 0;
}
/**
 * @brief 针对2006电机的控制
 *
 * @param id 电机抽象层id单号
 * @param cmd 控制命令
 * @param arg 命令参数
 * @return int
 */
int motor_dj_ctr(int id, uint16_t cmd, float *arg)
{
    motor_t *motor = motor_get(id);
    motor_measure_t *__motor = (motor_measure_t *)motor->ops->user_data;
    LOG_D("not used");
    switch (cmd)
    {

    case MOTOR_MODE_SAFETY_STOP:
        motor_stop(id);
        break;
    case MOTOR_MODE_SAFETY_START:
        motor_start(id);

        break;
    case MOTOR_MODE_TORQUE:
        /*返回力矩/电流值*/
		
        // *arg = __motor->real_current;
        break;
    case MOTOR_MODE_SPEED:
        // /*返回速度值r/min rpm*/
        // *arg = __motor->speed_rpm;
        break;
    case MOTOR_MODE_POS:
        // /*返回位置 rad*/
        //*arg = (float)__motor->total_angle * 0.0007669f;
        break;
    case MOTOR_MODE_TEMP:
        // /*返回温度*/
        *arg = 42.f;
        break;

    default:
        break;
    }
    return 0;
}

/* 接收数据回调函数 */
static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size);

/**
    Rx_Data[0] 转子机械角度高8位
    Rx_Data[1] 转子机械角度低8位
    Rx_Data[2] 转子转速高8位
    Rx_Data[3] 转子转速低8位
    Rx_Data[4] 实际转矩电流高8位
    Rx_Data[5] 实际转矩电流低8位
    Rx_Data[6] 电机温度
    Rx_Data[7] 空Null

    发送频率 1KHz
    转子机械角度 0 ~ 8191 (对应转子角度0~360)
    转子转速单位 RPM
    电机温度单位 °C

last_angle  上次角度更新
angle       转子机械角度高8位和第八位
speed_rpm   转子转速高8位和低八位
real_current实际输出转矩高8位和低8位
temperature 温度
total_angle 总角度
 */
void dj_motor_BackToZero(motor_measure_t *motor)
{
    motor->offset_angle = motor->angle;
    motor->round_cnt = 0;
}
// rt_mailbox_t dj_m_mailbox;
rt_err_t ind_dj_can_motor_callback(rt_device_t dev, void *args, rt_int32_t hdr, rt_size_t size)
{
    /* CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    // rt_pin_write(GET_PIN(I, 0), 1 - rt_pin_read(GET_PIN(I, 0)));
    struct rt_can_msg rxmsg = {0};

    /* 从 CAN 读取一帧数据 */
    rt_device_read(can_dev, 0, &rxmsg, sizeof(rxmsg));
    /* 打印数据 ID 及内容 */
    // rt_kprintf("ID:%x ", rxmsg.id);

    motor_measure_t *motor_measure = motor_get_by_canid(rxmsg.id);

    if (motor_measure->msg_cnt < 50) // 上电后接收50次矫正 50次之后正常接收数据
    {
        motor_measure->msg_cnt++;
        motor_measure->offset_angle = motor_measure->angle = (uint16_t)(rxmsg.data[0] << 8 | rxmsg.data[1]);
    }
    else
    {
        motor_measure->last_angle = motor_measure->angle;                      // 上次角度更新
        motor_measure->angle = (uint16_t)(rxmsg.data[0] << 8 | rxmsg.data[1]); // 转子机械角度高8位和第八位
                                                                               // 温度     //Null

        if (motor_measure->angle - motor_measure->last_angle > 4096)
        {
            motor_measure->round_cnt--;
            // LOG_D("id %d,total_angle %d ", motor_measure->id, motor_measure->round_cnt);
        }
        else if (motor_measure->angle - motor_measure->last_angle < -4096)
        {
            motor_measure->round_cnt++;
            // LOG_D("id %d,total_angle %d ", motor_measure->id, motor_measure->round_cnt);
        }

        motor_measure->msg_cnt = 0xff;
        motor_measure->total_angle = motor_measure->round_cnt * 8191 + motor_measure->angle - motor_measure->offset_angle;
        // LOG_D("id %d,cnt %d angle %d %d", motor_measure->id, motor_measure->round_cnt,motor_measure->round_cnt * 8191 + motor_measure->angle - motor_measure->offset_angle);
    }

    int id = motor_measure->id;
    uint16_t pos = 0;
    // 9.549279f*功率/转速=扭矩
    motor_feedback_speed(id, ((float)((int16_t)(rxmsg.data[2] << 8 | rxmsg.data[3]))));
    motor_feedback_torque(id, ((int16_t)(rxmsg.data[4] << 8 | rxmsg.data[5])));
    // 0.894为实际测量后的修正系数
#define PI_2 6.28318530719f
#define KK PI_2 / 8192.f /*弧度制*/
    // #define KK (360.f/ 8192.f)/*角度制*/
    motor_feedback_pos(id, ((float)(motor_measure->total_angle)) * KK);
    // rt_pin_write(GET_PIN(I, 0), 1 - rt_pin_read(GET_PIN(I, 0)));

    rt_ringbuffer_putchar_force(dj_m_ringfifo, id);
    // rt_sem_release(&rx_sem);

    // int8_t tt = rt_mb_send(dj_m_mailbox, id);
    // if (tt != RT_EOK)
    // {
    //     LOG_E("dj_m_mailbox send failed %d",-tt);
    // }
    return RT_EOK;
}
static void can_rx_thread1(void *parameter)
{
    //motor_t *motor = motor_get(M3508_7_CAN1);
    motor_t *motor = motor_get(0);
//    var_register(&(motor->tar_speed), "tarspeed", _f);
//    var_register(&(motor->pid_speed->parameter.kp), "kp", _f);
//    var_register(&(motor->pid_speed->parameter.ki), "ki", _f);
//    var_register(&(motor->pid_speed->parameter.kd), "kd", _f);
//    var_register(&(motor->tar_pos), "tarpos", _f);
//    var_register(&(motor->pid_pos->parameter.kp), "kp1", _f);
//    var_register(&(motor->pid_pos->parameter.ki), "ki1", _f);
//    var_register(&(motor->pid_pos->parameter.kd), "kd1", _f);

    //  motor_set_pos(M3508_1_CAN1, 0);
    //  motor_set_speed(M3508_1_CAN1, 200);
    //   motor_set_pos(M2006_1_CAN1,0);
    //   motor_set_pos(M2006_2_CAN1,0);
    //   motor_set_pos(M2006_3_CAN1,0);
    //   motor_set_pos(M2006_4_CAN1,0);
    //   motor_set_speed(M2006_5_CAN1,0);
    motor_t *motor1 = motor_get(0);
    motor_t *motor2= motor_get(1);
    motor_t *motor3 = motor_get(2);
    motor_t *motor4 = motor_get(3);

    while (1)
    {
        // motor_shakdown(0);
       //motor_shakdown(M3508_7_CAN1);
        // motor_set_pos(M2006_5_CAN1,-100);
//			LOG_D("m1:%.2f,m2:%.2f,m3:%.2f,m4:%.2f",motor1->pid_speed->parameter.out,motor2->pid_speed->parameter.out,motor3->pid_speed->parameter.out,motor4->pid_speed->parameter.out);
//			LOG_D("current2:%.2f,%.2f,%.2f,%.2f",motor1->pid_speed->parameter.target,motor2->pid_speed->parameter.target,motor3->pid_speed->parameter.target,motor4->pid_speed->parameter.target);
        rt_thread_delay(20);
    }
}
/******************************************************************************
 * @ 函数名  ： timer1_callback
 * @ 功  能  ： 定时器1回调函数
 * @ 参  数  ： parameter 外部传入的参数
 * @ 返回值  ： 无
 ******************************************************************************/
static void timer1_callback(void *parameter)
{
    rt_uint32_t tick_num1 = 0;
}
static void can_rx_thread(void *parameter)
{
    static rt_uint8_t i;
    rt_err_t res;
    struct rt_can_msg msg = {0};
    rt_pin_mode(GET_PIN(I, 0), PIN_MODE_OUTPUT);
    rt_pin_mode(GET_PIN(I, 2), PIN_MODE_OUTPUT);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(can_dev, can_rx_call);
#ifdef RT_CAN_USING_HDR
    struct rt_can_filter_item items[] = {
        {.id = 0x200, .ide = 0, .rtr = 0, .mode = 0, .mask = 0x7f0, .hdr_bank = 0, .rxfifo = CAN_RX_FIFO0, .ind = ind_dj_can_motor_callback, .args = RT_NULL}};

    struct rt_can_filter_config cfg = {sizeof(items) / sizeof(struct rt_can_filter_item), 1, items}; /* 过滤表 */
    /* 设置硬件过滤表 */
    res = rt_device_control(can_dev, RT_CAN_CMD_SET_FILTER, &cfg);
    RT_ASSERT(res == RT_EOK);
#endif
    static int timeout_counter = 0;
    while (1)
    {

        if (rt_ringbuffer_getchar(dj_m_ringfifo, &i) == 0)
        {
            timeout_counter++;
            if (timeout_counter > 4)
            {
                // LOG_W("We have lost too many can motor device (last received id %d) data, the motor may be lose position,please check!",i);
            }
            rt_thread_mdelay(1);
            // LOG_E("dj_m_ringfifo get failed");
        }
        else
        {
            timeout_counter = 0;
            motor_handle(i, 1);
        }

        // LOG_D("id %d,total_angle %f angle %d count %d", motor_measure->id, motor_get_pos(id),
        // motor_measure->angle,motor_measure->round_cnt);
        // rt_pin_write(GET_PIN(I, 2), 1 - rt_pin_read(GET_PIN(I, 2)));
        static uint32_t last_time = 0;
        // motor_handle(i, get_system_ms()-last_time);
        // motor_handle(i, 1);
        last_time = get_system_ms();
        // rt_pin_write(GET_PIN(I, 2), 1 - rt_pin_read(GET_PIN(I, 2)));
    }
}
static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
{

    LOG_W("can_rx_call unknown id data");
    return RT_EOK;
}
static void set_motor_passive_feedback(void);

int motor_dj_init(void)
{
    struct rt_can_msg msg = {0};
    rt_err_t res;
    rt_size_t size;
    rt_thread_t thread;

    motor_init();
    set_motor_passive_feedback();
    /* 查找 CAN 设备 */
    can_dev = rt_device_find(CAN_DEV_NAME);
    if (!can_dev)
    {
        rt_kprintf("find %s failed!\n", CAN_DEV_NAME);
        return -RT_ERROR;
    }

    /* 初始化 CAN 接收信号量 */
    rt_sem_init(&rx_sem, "can_m_dj_sem", 0, RT_IPC_FLAG_PRIO);
    //    dj_m_mailbox = rt_mb_create("dj_m_rx_mailbox", 4096, RT_IPC_FLAG_PRIO);

    dj_m_ringfifo = rt_ringbuffer_create(4096);
    /* 以中断接收及中断发送方式打开 CAN 设备 */
    res = rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    RT_ASSERT(res == RT_EOK);
    /* 设置 CAN 通信的波特率为 1Mbit/s*/
    res = rt_device_control(can_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN1MBaud);

    RT_ASSERT(res == RT_EOK);
    res = rt_device_control(can_dev, RT_CAN_CMD_SET_PRIV, (void *)1);
    RT_ASSERT(res == RT_EOK);

    // 创建一个软件定时器
    tmr1 = rt_timer_create("tmr1",          // 软件定时器名称
                           timer1_callback, // 软件定时器超时函数
                           RT_NULL,         // 超时函数参数
                           10,              // 超时时间
                           RT_TIMER_FLAG_ONE_SHOT |
                               RT_TIMER_FLAG_SOFT_TIMER); // 软件定时器模式，一次模式

    // // 启动定时器
    // if(tmr1 != RT_NULL)
    // 	rt_timer_start(tmr1);

    /* 创建数据接收线程 */
    thread = rt_thread_create("m_dj_driver", can_rx_thread, RT_NULL, 4096 * 2, 3, 10);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        rt_kprintf("create can_rx thread failed!\n");
    }

    thread = rt_thread_create("m_test", can_rx_thread1, RT_NULL, 4096 * 2, 6, 10);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        rt_kprintf("create can_rx thread failed!\n");
    }
    return 0;
}
INIT_COMPONENT_EXPORT(motor_dj_init);

#define DJ_DEFAULT_CFG_3508(id)                                \
    do                                                         \
    {                                                          \
        motor_set_passive_feedback(id, 1);                     \
        motor_set_ratio(id, 19.20204388f);                     \
        motor_set_pid_speed_ratio(id,1,2);                     \
        APID_Set_Out_Limit(motor_get_pid_speed(id), 20000);    \
        APID_Set_Integral_Limit(motor_get_pid_speed(id), 2000); \
        APID_Set_Bias_Dead_Zone(motor_get_pid_speed(id), 40);  \
        APID_Set_Bias_Limit(motor_get_pid_speed(id), 2000);    \
			\
			        APID_Set_Out_Limit(motor_get_pid_pos(id), 600);    \
        APID_Set_Integral_Limit(motor_get_pid_pos(id), 200);   \
        APID_Set_Bias_Dead_Zone(motor_get_pid_pos(id), 0.06);  \
    } while (0)

static void set_motor_passive_feedback(void)
{
#if defined(MOTOR_DJ_M3508_ID1_CAN1)
    DJ_DEFAULT_CFG_3508(M3508_1_CAN1);

//    //////////////////////////////////参数覆盖//////////////////////////////////////////////////////
//    motor_set_passive_feedback(M3508_1_CAN1, 1);
//    motor_set_ratio(M3508_1_CAN1, 19.20204388f); // TODO:大概是19，不准,100圈校准值19.20204388f
//    ////////////////////////////speed///////////////////////////////
//    APID_Set_Out_Limit(motor_get_pid_speed(M3508_1_CAN1), 20000);
//    APID_Set_Integral_Limit(motor_get_pid_speed(M3508_1_CAN1), 2000);
//    APID_Set_Bias_Dead_Zone(motor_get_pid_speed(M3508_1_CAN1), 40);
//    APID_Set_Bias_Limit(motor_get_pid_speed(M3508_1_CAN1), 2000);
//    // APID_Set_Target_Limit(motor_get_pid_speed(M3508_1_CAN1), 70);

//    ////////////////////////////position///////////////////////////
//    // APID_Set_Out_Limit(motor_get_pid_pos(M3508_1_CAN1),      20000);
//    // APID_Set_Out_Limit(motor_get_pid_pos(M3508_1_CAN1), 50);
//    APID_Set_Integral_Limit(motor_get_pid_pos(M3508_1_CAN1), 200);
//    APID_Set_Bias_Dead_Zone(motor_get_pid_pos(M3508_1_CAN1), 0.02);
//    // APID_Set_Bias_Limit(motor_get_pid_pos(M3508_1_CAN1), 2000);
//    // APID_Set_Target_Limit(motor_get_pid_pos(M3508_1_CAN1), 70);
//    ////////////////////////////////////////////////////////////////////////////////////////////////

#endif
#if defined(MOTOR_DJ_M3508_ID2_CAN1)
    DJ_DEFAULT_CFG_3508(M3508_2_CAN1);
#endif
#if defined(MOTOR_DJ_M3508_ID3_CAN1)
    DJ_DEFAULT_CFG_3508(M3508_3_CAN1);
#endif
#if defined(MOTOR_DJ_M3508_ID4_CAN1)
    DJ_DEFAULT_CFG_3508(M3508_4_CAN1);
#endif
#if defined(MOTOR_DJ_M3508_ID5_CAN1)
    DJ_DEFAULT_CFG_3508(M3508_5_CAN1);
#endif
#if defined(MOTOR_DJ_M3508_ID6_CAN1)
    DJ_DEFAULT_CFG_3508(M3508_6_CAN1);
#endif
#if defined(MOTOR_DJ_M3508_ID7_CAN1)
    DJ_DEFAULT_CFG_3508(M3508_7_CAN1);
									
		APID_Set_Out_Limit(motor_get_pid_speed(M3508_7_CAN1), 20000);    
        APID_Set_Integral_Limit(motor_get_pid_speed(M3508_7_CAN1), 2000); 
        APID_Set_Bias_Dead_Zone(motor_get_pid_speed(M3508_7_CAN1), 40);  
        APID_Set_Bias_Limit(motor_get_pid_speed(M3508_7_CAN1), 2000);   
			
			  APID_Set_Out_Limit(motor_get_pid_pos(M3508_7_CAN1), 3000);  
       // APID_Set_Bias_Limit(motor_get_pid_pos(M3508_7_CAN1), 3.1415926/4);   				
        APID_Set_Integral_Limit(motor_get_pid_pos(M3508_7_CAN1), 200);   
        APID_Set_Bias_Dead_Zone(motor_get_pid_pos(M3508_7_CAN1), 0.06);  
#endif
#if defined(MOTOR_DJ_M3508_ID8_CAN1)
    DJ_DEFAULT_CFG_3508(M3508_8_CAN1);
#endif
#if defined(MOTOR_DJ_M3508_ID1_CAN2)
    DJ_DEFAULT_CFG_3508(M3508_1_CAN2);
#endif
#if defined(MOTOR_DJ_M3508_ID2_CAN2)
    DJ_DEFAULT_CFG_3508(M3508_2_CAN2);
#endif
#if defined(MOTOR_DJ_M3508_ID3_CAN2)
    DJ_DEFAULT_CFG_3508(M3508_3_CAN2);
#endif
#if defined(MOTOR_DJ_M3508_ID4_CAN2)
    DJ_DEFAULT_CFG_3508(M3508_4_CAN2);
#endif
#if defined(MOTOR_DJ_M3508_ID5_CAN2)
    DJ_DEFAULT_CFG_3508(M3508_5_CAN2);
#endif
#if defined(MOTOR_DJ_M3508_ID6_CAN2)
    DJ_DEFAULT_CFG_3508(M3508_6_CAN2);
#endif
#if defined(MOTOR_DJ_M3508_ID7_CAN2)
    DJ_DEFAULT_CFG_3508(M3508_7_CAN2);
#endif
#if defined(MOTOR_DJ_M3508_ID8_CAN2)
    DJ_DEFAULT_CFG_3508(M3508_8_CAN2);
#endif

#if defined(MOTOR_DJ_M2006_ID1_CAN1)
    motor_set_passive_feedback(M2006_1_CAN1, 1);
    APID_Set_Out_Limit(motor_get_pid_speed(M2006_1_CAN1), 30000);
    APID_Set_Integral_Limit(motor_get_pid_speed(M2006_1_CAN1), 200);
    APID_Set_Bias_Dead_Zone(motor_get_pid_speed(M2006_1_CAN1), 20);
    APID_Set_Bias_Limit(motor_get_pid_speed(M2006_1_CAN1), 2000);
    APID_Set_Target_Limit(motor_get_pid_speed(M2006_1_CAN1), 70);

    // APID_Set_Out_Limit(motor_get_pid_pos(M2006_1_CAN1), 20000);
    APID_Set_Out_Limit(motor_get_pid_pos(M2006_1_CAN1), 50);
    APID_Set_Integral_Limit(motor_get_pid_pos(M2006_1_CAN1), 200);
    // APID_D_PART(motor_get_pid_pos(M2006_1_CAN1), 0.7);

#endif
#if defined(MOTOR_DJ_M2006_ID2_CAN1)
    motor_set_passive_feedback(M2006_2_CAN1, 1);
    APID_Set_Out_Limit(motor_get_pid_speed(M2006_2_CAN1), 30000);
    APID_Set_Integral_Limit(motor_get_pid_speed(M2006_2_CAN1), 200);
    APID_Set_Bias_Dead_Zone(motor_get_pid_speed(M2006_2_CAN1), 20);
    APID_Set_Bias_Limit(motor_get_pid_speed(M2006_2_CAN1), 2000);
    APID_Set_Target_Limit(motor_get_pid_speed(M2006_2_CAN1), 70);

    APID_Set_Out_Limit(motor_get_pid_pos(M2006_2_CAN1), 50);
    APID_Set_Integral_Limit(motor_get_pid_pos(M2006_2_CAN1), 200);
    // APID_D_PART(motor_get_pid_pos(M2006_2_CAN1), 0.7);
#endif
#if defined(MOTOR_DJ_M2006_ID3_CAN1)
    motor_set_passive_feedback(M2006_3_CAN1, 1);
    APID_Set_Out_Limit(motor_get_pid_speed(M2006_3_CAN1), 30000);
    APID_Set_Integral_Limit(motor_get_pid_speed(M2006_3_CAN1), 200);
    APID_Set_Bias_Dead_Zone(motor_get_pid_speed(M2006_3_CAN1), 20);
    APID_Set_Target_Limit(motor_get_pid_speed(M2006_3_CAN1), 70);

    APID_Set_Out_Limit(motor_get_pid_pos(M2006_3_CAN1), 50);
    APID_Set_Integral_Limit(motor_get_pid_pos(M2006_3_CAN1), 200);
    // APID_D_PART(motor_get_pid_pos(M2006_3_CAN1), 0.7);

#endif
#if defined(MOTOR_DJ_M2006_ID4_CAN1)
    motor_set_passive_feedback(M2006_4_CAN1, 1);
    APID_Set_Out_Limit(motor_get_pid_speed(M2006_4_CAN1), 30000);
    APID_Set_Integral_Limit(motor_get_pid_speed(M2006_4_CAN1), 200);
    APID_Set_Bias_Dead_Zone(motor_get_pid_speed(M2006_4_CAN1), 20);
    APID_Set_Target_Limit(motor_get_pid_speed(M2006_4_CAN1), 70);

    APID_Set_Out_Limit(motor_get_pid_pos(M2006_4_CAN1), 50);
    APID_Set_Integral_Limit(motor_get_pid_pos(M2006_4_CAN1), 200);
    // APID_D_PART(motor_get_pid_pos(M2006_4_CAN1), 0.7);

#endif
#if defined(MOTOR_DJ_M2006_ID5_CAN1)
    motor_set_passive_feedback(M2006_5_CAN1, 1);
    APID_Set_Out_Limit(motor_get_pid_speed(M2006_5_CAN1), 3500);
    APID_Set_Integral_Limit(motor_get_pid_speed(M2006_5_CAN1), 200);
    APID_Set_Bias_Dead_Zone(motor_get_pid_speed(M2006_5_CAN1), 20);
    APID_Set_Target_Limit(motor_get_pid_speed(M2006_5_CAN1), 60);

    // APID_Set_Out_Limit(motor_get_pid_pos(M2006_5_CAN1), 50);
    APID_Set_Integral_Limit(motor_get_pid_pos(M2006_5_CAN1), 200);
    // APID_D_PART(motor_get_pid_pos(M2006_5_CAN1), 0.7);
#endif
#if defined(MOTOR_DJ_M2006_ID6_CAN1)
    motor_set_passive_feedback(M2006_1_CAN1, 1);
#endif
#if defined(MOTOR_DJ_M2006_ID7_CAN1)
    motor_set_passive_feedback(M2006_1_CAN1, 1);
#endif
#if defined(MOTOR_DJ_M2006_ID8_CAN1)
    motor_set_passive_feedback(M2006_1_CAN1, 1);
#endif
#if defined(MOTOR_DJ_M2006_ID1_CAN2)

    motor_set_passive_feedback(M2006_1_CAN2, 1);
#endif
#if defined(MOTOR_DJ_M2006_ID2_CAN2)
    motor_set_passive_feedback(M2006_1_CAN2, 1);
#endif
#if defined(MOTOR_DJ_M2006_ID3_CAN2)
    motor_set_passive_feedback(M2006_1_CAN2, 1);
#endif
#if defined(MOTOR_DJ_M2006_ID4_CAN2)
    motor_set_passive_feedback(M2006_1_CAN2, 1);
#endif
#if defined(MOTOR_DJ_M2006_ID5_CAN2)
    motor_set_passive_feedback(M2006_1_CAN2, 1);
#endif
#if defined(MOTOR_DJ_M2006_ID6_CAN2)
    motor_set_passive_feedback(M2006_1_CAN2, 1);
#endif
#if defined(MOTOR_DJ_M2006_ID7_CAN2)
    motor_set_passive_feedback(M2006_1_CAN2, 1);
#endif
#if defined(MOTOR_DJ_M2006_ID8_CAN2)
    motor_set_passive_feedback(M2006_1_CAN2, 1);
#endif

#if defined(MOTOR_DJ_M6020_ID1_CAN1)
    motor_set_passive_feedback(M6020_1_CAN1, 1);
#endif
#if defined(MOTOR_DJ_M6020_ID2_CAN1)
    motor_set_passive_feedback(M6020_2_CAN1, 1);
#endif
#if defined(MOTOR_DJ_M6020_ID3_CAN1)
    motor_set_passive_feedback(M6020_3_CAN1, 1);
#endif
#if defined(MOTOR_DJ_M6020_ID4_CAN1)
    motor_set_passive_feedback(M6020_4_CAN1, 1);
#endif
#if defined(MOTOR_DJ_M6020_ID5_CAN1)
    motor_set_passive_feedback(M6020_5_CAN1, 1);
#endif
#if defined(MOTOR_DJ_M6020_ID6_CAN1)
    motor_set_passive_feedback(M6020_6_CAN1, 1);
#endif
#if defined(MOTOR_DJ_M6020_ID7_CAN1)
    motor_set_passive_feedback(M6020_7_CAN1, 1);
#endif
#if defined(MOTOR_DJ_M6020_ID8_CAN1)
    motor_set_passive_feedback(M6020_8_CAN1, 1);
#endif
#if defined(MOTOR_DJ_M6020_ID1_CAN2)
    motor_set_passive_feedback(M6020_1_CAN2, 1);
#endif
#if defined(MOTOR_DJ_M6020_ID2_CAN2)
    motor_set_passive_feedback(M6020_2_CAN2, 1);
#endif
#if defined(MOTOR_DJ_M6020_ID3_CAN2)
    motor_set_passive_feedback(M6020_3_CAN2, 1);
#endif
#if defined(MOTOR_DJ_M6020_ID4_CAN2)
    motor_set_passive_feedback(M6020_4_CAN2, 1);
#endif
#if defined(MOTOR_DJ_M6020_ID5_CAN2)
    motor_set_passive_feedback(M6020_5_CAN2, 1);
#endif
#if defined(MOTOR_DJ_M6020_ID6_CAN2)
    motor_set_passive_feedback(M6020_6_CAN2, 1);
#endif
#if defined(MOTOR_DJ_M6020_ID7_CAN2)
    motor_set_passive_feedback(M6020_7_CAN2, 1);
#endif
#if defined(MOTOR_DJ_M6020_ID8_CAN2)
    motor_set_passive_feedback(M6020_8_CAN2, 1);
#endif
}
