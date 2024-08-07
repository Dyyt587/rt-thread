/*
 * @Author: Dyyt587 67887002+Dyyt587@users.noreply.github.com
 * @Date: 2024-08-06 23:50:22
 * @LastEditors: Dyyt587 67887002+Dyyt587@users.noreply.github.com
 * @LastEditTime: 2024-08-07 20:04:49
 * @FilePath: \stm32h7b0-fk7b0m1-vbt6c:\Users\80520\Documents\GitHub\rt-thread\bsp\stm32\libraries\HAL_Drivers\drivers\drv_fdcan.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-08-05     Xeon Xu      the first version
 * 2019-01-22     YLZ          port from stm324xx-HAL to bsp stm3210x-HAL
 * 2019-01-26     YLZ          redefine `struct stm32_drv_can` add member `Rx1Message`
 * 2019-02-19     YLZ          port to BSP [stm32]
 * 2019-06-17     YLZ          modify struct stm32_drv_can.
 */

#ifndef __DRV_CAN_H__
#define __DRV_CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <board.h>
#include <rtdevice.h>

#define BS1SHIFT        16
#define BS2SHIFT        20
#define RRESCLSHIFT     0
#define SJWSHIFT        24
#define BS1MASK         ((0x0F) << BS1SHIFT )
#define BS2MASK         ((0x07) << BS2SHIFT )
#define RRESCLMASK      (0x3FF << RRESCLSHIFT )
#define SJWMASK         (0x3 << SJWSHIFT )

struct stm32_baud_rate_tab
{
    rt_uint32_t baud_rate;

    rt_uint32_t DataPrescaler:5;                /*!< Specifies the value by which the oscillator frequency is
                                                divided for generating the data bit time quanta.
                                                This parameter must be a number between 1 and 32             */

    rt_uint32_t DataSyncJumpWidth:4;            /*!< Specifies the maximum number of time quanta the FDCAN
                                                hardware is allowed to lengthen or shorten a data bit to
                                                perform resynchronization.
                                                This parameter must be a number between 1 and 16             */
    rt_uint32_t DataTimeSeg1:5;                 /*!< Specifies the number of time quanta in Data Bit Segment 1.
                                                This parameter must be a number between 1 and 32             */

    rt_uint32_t DataTimeSeg2:4;                 /*!< Specifies the number of time quanta in Data Bit Segment 2.
                                                This parameter must be a number between 1 and 16  */ 
                                                
    rt_uint32_t NominalPrescaler :9;            /*!< Specifies the value by which the oscillator frequency is
                                                divided for generating the nominal bit time quanta.
                                                This parameter must be a number between 1 and 512            */

    rt_uint32_t NominalSyncJumpWidth :7;         /*!< Specifies the maximum number of time quanta the FDCAN
                                                hardware is allowed to lengthen or shorten a bit to perform
                                                resynchronization.
                                                This parameter must be a number between 1 and 128            */

    rt_uint32_t NominalTimeSeg1:8;              /*!< Specifies the number of time quanta in Bit Segment 1.
                                                This parameter must be a number between 2 and 256            */

    rt_uint32_t NominalTimeSeg2:7;              /*!< Specifies the number of time quanta in Bit Segment 2.
                                                This parameter must be a number between 2 and 128            */
};
#define BAUD_DATA(TYPE,NO)       ((can_baud_rate_tab[NO].TYPE))

/* stm32 can device */
struct stm32_can
{
    char *name;
    FDCAN_RxHeaderTypeDef RxHeader;
	FDCAN_TxHeaderTypeDef TxHeader;
	uint8_t u8RxDataBuffer[8];
	uint8_t u8TxDataBuufer[8];  
    FDCAN_HandleTypeDef CanHandle;
    FDCAN_FilterTypeDef FilterConfig;
    struct rt_can_device device;     /* inherit from can device */
};

int rt_hw_can_init(void);

#ifdef __cplusplus
}
#endif

#endif /*__DRV_CAN_H__ */

/************************** end of file ******************/
