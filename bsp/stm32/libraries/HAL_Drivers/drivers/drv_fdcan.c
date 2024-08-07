/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#include "drv_fdcan.h"


// #define RT_USING_CAN
// #define BSP_USING_CAN
// #define BSP_USING_FDCAN1




#ifdef BSP_USING_FDCAN

#define LOG_TAG    "drv_can"
#include <drv_log.h>
#include <stm32h7xx_hal_fdcan.h>
#include <board.h>
#include <rtdevice.h>


#define BSP_FDCAN_CLOCK 120000000
#define BSP_USING_CAN2

#ifndef BSP_FDCAN_CLOCK
#error please define BSP_FDCAN_CLOCK in rtconfig.h to calculate the baud rate.
#endif
#if defined (SOC_SERIES_STM32H7)/* APB1 25MHz(max) */
static const struct stm32_baud_rate_tab can_baud_rate_tab[] =
{
    #if BSP_FDCAN_CLOCK == 120000000 /* 120MHz */
    {CAN1MBaud,    5, 1, 15, 8,  12, 1, 5, 4},
    {CAN800kBaud,  5, 2, 18, 10, 6, 1, 15, 9}, 
    {CAN500kBaud,  10, 1, 15, 8, 12, 1, 10, 9},
    {CAN250kBaud,  20, 1, 15, 8, 12, 1, 20, 19},
//    {CAN125kBaud,  40, 1, 15, 8, 12, 1, 40, 39},
//    {CAN100kBaud,  50, 1, 15, 8, 12, 1, 15, 9},
//    {CAN50kBaud,   100, 1, 15, 8, 12, 1, 100, 99},
//    {CAN20kBaud,   250, 1, 15, 8, 24, 1, 150, 99},
//    {CAN10kBaud,   500, 1, 15, 8, 48, 1, 150, 99},
    #elif BSP_FDCAN_CLOCK == 60000000 /* 60MHz */
    {CAN1MBaud,    5, 1, 7, 4, 1, 1, 1, 1},
    {CAN800kBaud,  5, 1, 9, 5, 1, 1, 1, 1}, 
    {CAN500kBaud,  5, 1, 15, 8, 1, 1, 1, 1},
    {CAN250kBaud,  10, 1, 15, 8, 1, 1, 1, 1},
    {CAN125kBaud,  20, 1, 15, 8, 1, 1, 1, 1},
    {CAN100kBaud,  25, 1, 15, 8, 1, 1, 1, 1},
    {CAN50kBaud,   50, 1, 15, 8, 1, 1, 1, 1},
    {CAN20kBaud,   125, 1, 15, 8, 1, 1, 1, 1},
    {CAN10kBaud,   250, 1, 15, 8, 1, 1, 1, 1},
    #else
    #error please change BSP_FDCAN_CLOCK and CubeMX config or define new table for this clock. 
    #endif
};
#endif

#ifdef BSP_USING_CAN1
static struct stm32_can drv_can1 =
{
    .name = "can1",
    .CanHandle.Instance = FDCAN1,
};
#endif

#ifdef BSP_USING_CAN2
static struct stm32_can drv_can2 =
{
    "can2",
    .CanHandle.Instance = FDCAN2,
};
#endif

static rt_uint32_t get_can_baud_index(rt_uint32_t baud)
{
    rt_uint32_t len, index;

    len = sizeof(can_baud_rate_tab) / sizeof(can_baud_rate_tab[0]);
    for (index = 0; index < len; index++)
    {
        if (can_baud_rate_tab[index].baud_rate == baud)
            return index;
    }

    return 0; /* default baud is CAN1MBaud */
}

static rt_err_t _can_config(struct rt_can_device *can, struct can_configure *cfg)
{
    struct stm32_can *drv_can;
    rt_uint32_t baud_index;

    RT_ASSERT(can);
    RT_ASSERT(cfg);
    drv_can = (struct stm32_can *)can->parent.user_data;
    RT_ASSERT(drv_can);

    drv_can->CanHandle.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    drv_can->CanHandle.Init.Mode = FDCAN_MODE_NORMAL;
    drv_can->CanHandle.Init.AutoRetransmission = ENABLE;
    drv_can->CanHandle.Init.TransmitPause = DISABLE;
    drv_can->CanHandle.Init.ProtocolException = DISABLE;

    drv_can->CanHandle.Init.DataPrescaler = 1;
    drv_can->CanHandle.Init.DataSyncJumpWidth = 15;
    drv_can->CanHandle.Init.DataTimeSeg1 = 1;
    drv_can->CanHandle.Init.DataTimeSeg2 = 1;

	if(drv_can->CanHandle.Instance == FDCAN1)
	{
		drv_can->CanHandle.Init.MessageRAMOffset = 0;					
	}
	else
	{
		drv_can->CanHandle.Init.MessageRAMOffset = 1280;					
	}


    drv_can->CanHandle.Init.StdFiltersNbr = 2;
    drv_can->CanHandle.Init.ExtFiltersNbr = 2;
    drv_can->CanHandle.Init.RxFifo0ElmtsNbr = 1;
    drv_can->CanHandle.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    drv_can->CanHandle.Init.RxFifo1ElmtsNbr = 0;
    drv_can->CanHandle.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    drv_can->CanHandle.Init.RxBuffersNbr = 0;
    drv_can->CanHandle.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    drv_can->CanHandle.Init.TxEventsNbr = 0;
    drv_can->CanHandle.Init.TxBuffersNbr = 3;
    drv_can->CanHandle.Init.TxFifoQueueElmtsNbr = 0;
    drv_can->CanHandle.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    drv_can->CanHandle.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
    switch (cfg->mode)
    {
    case RT_CAN_MODE_NORMAL:
        drv_can->CanHandle.Init.Mode = FDCAN_MODE_NORMAL;
        break;
    case RT_CAN_MODE_LISTEN:
        drv_can->CanHandle.Init.Mode = FDCAN_MODE_RESTRICTED_OPERATION;
        break;
    case RT_CAN_MODE_LOOPBACK:
        drv_can->CanHandle.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
        break;
    case RT_CAN_MODE_LOOPBACKANLISTEN:
        drv_can->CanHandle.Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;
        break;
    }

    baud_index = get_can_baud_index(cfg->baud_rate);   
    drv_can->CanHandle.Init.NominalPrescaler     = BAUD_DATA(NominalPrescaler, baud_index);
    drv_can->CanHandle.Init.NominalSyncJumpWidth = BAUD_DATA(NominalSyncJumpWidth, baud_index);
    drv_can->CanHandle.Init.NominalTimeSeg1      = BAUD_DATA(NominalTimeSeg1, baud_index);
    drv_can->CanHandle.Init.NominalTimeSeg2      = BAUD_DATA(NominalTimeSeg2, baud_index);

    drv_can->CanHandle.Init.DataPrescaler        = BAUD_DATA(DataPrescaler, baud_index);
    drv_can->CanHandle.Init.DataSyncJumpWidth    = BAUD_DATA(DataSyncJumpWidth, baud_index);
    drv_can->CanHandle.Init.DataTimeSeg1         = BAUD_DATA(DataTimeSeg1, baud_index);
    drv_can->CanHandle.Init.DataTimeSeg2         = BAUD_DATA(DataTimeSeg2, baud_index);
    /* init can */
    if (HAL_FDCAN_Init(&drv_can->CanHandle) != HAL_OK)
    {
        return -RT_ERROR;
    }

    /* default filter config */
    HAL_FDCAN_ConfigFilter(&drv_can->CanHandle, &drv_can->FilterConfig);
    /* can start */
    HAL_FDCAN_Start(&drv_can->CanHandle);

    return RT_EOK;
}

static rt_err_t _can_control(struct rt_can_device *can, int cmd, void *arg)
{

	rt_uint32_t argval;
    struct stm32_can *pdrv_can;

	struct rt_can_filter_config *filter_cfg;

	RT_ASSERT(can != RT_NULL);
	pdrv_can = (struct stm32_can *)can->parent.user_data;
	RT_ASSERT(pdrv_can != RT_NULL);

	switch (cmd)
	{

	case RT_DEVICE_CTRL_CLR_INT:
		argval = (rt_uint32_t) arg;
		if (argval == RT_DEVICE_FLAG_INT_RX)
		{
			HAL_FDCAN_DeactivateNotification(&pdrv_can->CanHandle,  FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
		}
		else if (argval == RT_DEVICE_FLAG_INT_TX)
		{
			HAL_FDCAN_DeactivateNotification(&pdrv_can->CanHandle,  FDCAN_IT_TX_FIFO_EMPTY);
			HAL_FDCAN_DeactivateNotification(&pdrv_can->CanHandle,  FDCAN_IT_TX_COMPLETE);
		}
		else if (argval == RT_DEVICE_CAN_INT_ERR)
		{
			HAL_FDCAN_DeactivateNotification(&pdrv_can->CanHandle,  FDCAN_IT_ERROR_WARNING);
			HAL_FDCAN_DeactivateNotification(&pdrv_can->CanHandle,  FDCAN_IT_ERROR_PASSIVE);
			HAL_FDCAN_DeactivateNotification(&pdrv_can->CanHandle,  FDCAN_IT_ERROR_LOGGING_OVERFLOW);
			HAL_FDCAN_DeactivateNotification(&pdrv_can->CanHandle,  FDCAN_IT_BUS_OFF);
			HAL_FDCAN_DeactivateNotification(&pdrv_can->CanHandle,  FDCAN_IT_ARB_PROTOCOL_ERROR);
		}
		break;
	case RT_DEVICE_CTRL_SET_INT:
		argval = (rt_uint32_t) arg;
		if (argval == RT_DEVICE_FLAG_INT_RX)
		{
			HAL_FDCAN_ConfigInterruptLines(&pdrv_can->CanHandle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);
			HAL_FDCAN_ActivateNotification(&pdrv_can->CanHandle,  FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

			if(FDCAN1 == pdrv_can->CanHandle.Instance)
			{
				HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 1);
				HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
			}
			else
			{
				HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 0, 1);
				HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
			}

		}
		else if (argval == RT_DEVICE_FLAG_INT_TX)
		{
			HAL_FDCAN_ConfigInterruptLines(&pdrv_can->CanHandle, FDCAN_IT_TX_COMPLETE, FDCAN_INTERRUPT_LINE1);
			HAL_FDCAN_ActivateNotification(&pdrv_can->CanHandle,  FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0);
			HAL_FDCAN_ActivateNotification(&pdrv_can->CanHandle,  FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER1);
			HAL_FDCAN_ActivateNotification(&pdrv_can->CanHandle,  FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER2);

			if(FDCAN1 == pdrv_can->CanHandle.Instance)
			{
				HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 0, 2);
				HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
			}
			else
			{
				HAL_NVIC_SetPriority(FDCAN2_IT1_IRQn, 0, 2);
				HAL_NVIC_EnableIRQ(FDCAN2_IT1_IRQn);
			}
		}
		else if (argval == RT_DEVICE_CAN_INT_ERR)
		{
			HAL_FDCAN_ConfigInterruptLines(&pdrv_can->CanHandle, FDCAN_IT_BUS_OFF, FDCAN_INTERRUPT_LINE1);
			HAL_FDCAN_ConfigInterruptLines(&pdrv_can->CanHandle, FDCAN_IT_ERROR_WARNING, FDCAN_INTERRUPT_LINE1);
			HAL_FDCAN_ConfigInterruptLines(&pdrv_can->CanHandle, FDCAN_IT_ERROR_PASSIVE, FDCAN_INTERRUPT_LINE1);
			HAL_FDCAN_ConfigInterruptLines(&pdrv_can->CanHandle, FDCAN_IT_ARB_PROTOCOL_ERROR, FDCAN_INTERRUPT_LINE1);

			HAL_FDCAN_ActivateNotification(&pdrv_can->CanHandle,  FDCAN_IT_BUS_OFF, 0);
			HAL_FDCAN_ActivateNotification(&pdrv_can->CanHandle,  FDCAN_IT_ERROR_WARNING, 0);
			HAL_FDCAN_ActivateNotification(&pdrv_can->CanHandle,  FDCAN_IT_ERROR_PASSIVE, 0);
			HAL_FDCAN_ActivateNotification(&pdrv_can->CanHandle,  FDCAN_IT_ARB_PROTOCOL_ERROR, 0);
			if(FDCAN1 == pdrv_can->CanHandle.Instance)
			{
				HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 0, 2);
				HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
			}
			else
			{
				HAL_NVIC_SetPriority(FDCAN2_IT1_IRQn, 0, 2);
				HAL_NVIC_EnableIRQ(FDCAN2_IT1_IRQn);
			}
		}
		break;
	case RT_CAN_CMD_SET_FILTER:
		if (RT_NULL == arg)
		{
			/* default filter config */
			HAL_FDCAN_ConfigFilter(&pdrv_can->CanHandle , &pdrv_can->FilterConfig);
		}
		else
		{
			filter_cfg = (struct rt_can_filter_config *)arg;
			_inline_can_filter_config(pdrv_can, filter_cfg);
		}
		break;
	case RT_CAN_CMD_SET_MODE:
		argval = (rt_uint32_t) arg;
		if (argval != RT_CAN_MODE_NORMAL &&
			argval != RT_CAN_MODE_LISEN &&
			argval != RT_CAN_MODE_LOOPBACK &&
			argval != RT_CAN_MODE_LOOPBACKANLISEN)
		{
			return -RT_ERROR;
		}
		if (argval != pdrv_can->device.config.mode)
		{
			pdrv_can->device.config.mode = argval;
			return _inline_can_config(&pdrv_can->device, &pdrv_can->device.config);
		}
		break;
	case RT_CAN_CMD_SET_BAUD:
		argval = (rt_uint32_t ) arg;
		/*just low to 50kbit/s*/
		if (argval != CAN1MBaud &&
			argval != CAN800kBaud &&
			argval != CAN500kBaud &&
			argval != CAN250kBaud &&
			argval != CAN125kBaud &&
			argval != CAN100kBaud &&
			argval != CAN50kBaud  &&
			argval != CAN20kBaud  &&
			argval != CAN10kBaud)
		{
			return -RT_ERROR;
		}
		if (argval != pdrv_can->device.config.baud_rate)
		{
			pdrv_can->device.config.baud_rate = argval;
			return _inline_can_config(&pdrv_can->device, &pdrv_can->device.config);
		}
		break;

	case RT_CAN_CMD_SET_PRIV:
		argval = (rt_uint32_t) arg;
		if (argval != RT_CAN_MODE_PRIV &&
				argval != RT_CAN_MODE_NOPRIV)
		{
			return -RT_ERROR;
		}
		if (argval != pdrv_can->device.config.privmode)
		{
			pdrv_can->device.config.privmode = argval;

			return RT_EOK;
		}
		break;
        case RT_CAN_CMD_GET_STATUS:
    {

        FDCAN_ProtocolStatusTypeDef ProtocolStatus;
        FDCAN_ErrorCountersTypeDef ErrorCounters;
        HAL_FDCAN_GetProtocolStatus(&pdrv_can->CanHandle, &ProtocolStatus);
        HAL_FDCAN_GetErrorCounters(&pdrv_can->CanHandle, &ErrorCounters);
        pdrv_can->device.status.rcverrcnt = ErrorCounters.RxErrorCnt+(ErrorCounters.RxErrorPassive<<8);
        pdrv_can->device.status.snderrcnt = ErrorCounters.TxErrorCnt;
        pdrv_can->device.status.lasterrtype = ProtocolStatus.LastErrorCode;
        pdrv_can->device.status.errcode = READ_REG(drv_can->CanHandle.Instance->PSR);

        // rt_uint32_t errtype;
        // errtype = drv_can->CanHandle.Instance->ESR;
        // drv_can->device.status.rcverrcnt = errtype >> 24;
        // drv_can->device.status.snderrcnt = (errtype >> 16 & 0xFF);
        // drv_can->device.status.lasterrtype = errtype & 0x70;
        // drv_can->device.status.errcode = errtype & 0x07;

        rt_memcpy(arg, &pdrv_can->device.status, sizeof(pdrv_can->device.status));
    }
    break;
    }

    return RT_EOK;
}

static int _can_sendmsg(struct rt_can_device *can, const void *buf, rt_uint32_t box_num)
{

    struct stm32_can *pdrv_can;
	struct rt_can_msg *pmsg;
	uint32_t tmp_u32DataLen;
	RT_ASSERT(can);
	RT_ASSERT(buf);

	pdrv_can = (_stm32_fdcan_t *)can->parent.user_data;

	RT_ASSERT(pdrv_can);

	pmsg = (struct rt_can_msg *) buf;

	/* Check the parameters */
	if(pmsg->len > 8)
	{
		tmp_u32DataLen = 8;
	}
	else
	{
		tmp_u32DataLen = pmsg->len;
	}
	tmp_u32DataLen <<=16;

	if(pmsg->ide == RT_CAN_EXTID)
	{
		pdrv_can->TxHeader.IdType = FDCAN_EXTENDED_ID;
	}
	else
	{
		pdrv_can->TxHeader.IdType = FDCAN_STANDARD_ID;
	}
	if (RT_CAN_DTR == pmsg->rtr)
	{
		pdrv_can->TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	}
	else
	{
		pdrv_can->TxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
	}

	pdrv_can->TxHeader.Identifier = pmsg->id;
	pdrv_can->TxHeader.DataLength = tmp_u32DataLen;
	if(HAL_FDCAN_AddMessageToTxBuffer(&pdrv_can->CanHandle, &pdrv_can->TxHeader, pmsg->data, FDCAN_TX_BUFFER0+box_num) != HAL_OK)
	{
		return -RT_ERROR;
	}
	else
	{
		/* Request transmission */
		HAL_FDCAN_EnableTxBufferRequest(&pdrv_can->CanHandle,FDCAN_TX_BUFFER0+box_num);
		return RT_EOK;
	}
}

static int _can_recvmsg(struct rt_can_device *can, void *buf, rt_uint32_t fifo)
{


    struct rt_can_msg *pmsg;
    struct stm32_can *pdrv_can;

    RT_ASSERT(can);
    RT_ASSERT(buf);

    pdrv_can = (struct stm32_can *)can->parent.user_data;
    pmsg = (struct rt_can_msg *) buf;
    if(HAL_FDCAN_GetRxMessage(&pdrv_can->CanHandle,FDCAN_RX_FIFO0+fifo, &pdrv_can->RxHeader, pmsg->data) != HAL_OK)
    {
    	return -RT_ERROR;
    }
    else
    {
    	if(pdrv_can->RxHeader.IdType == FDCAN_EXTENDED_ID)
    	{
    		pmsg->ide = RT_CAN_EXTID;
    	}
    	else
    	{
    		pmsg->ide = RT_CAN_STDID;
    	}

    	if(pdrv_can->RxHeader.RxFrameType == FDCAN_DATA_FRAME)
    	{
    		pmsg->rtr = RT_CAN_DTR;
		}
		else
		{
			pmsg->rtr = RT_CAN_RTR;
		}
    	pmsg->id = pdrv_can->RxHeader.Identifier;

    	pmsg->len = (pdrv_can->RxHeader.DataLength>>16)&0x0f;

    	pmsg->hdr = pdrv_can->RxHeader.FilterIndex;
    	return RT_EOK;
    }
}


static const struct rt_can_ops _can_ops =
{
    _can_config,
    _can_control,
    _can_sendmsg,
    _can_recvmsg,
};

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  	{
  		if(hfdcan->Instance == FDCAN1)
  		{
#ifdef BSP_USING_FDCAN1
  			//CAN1
  			/* Retreive Rx messages from RX FIFO0 */
  			rt_hw_can_isr(&st_DrvCan1.device, RT_CAN_EVENT_RX_IND | 0 << 8);
#endif
  		}
		else
		{
#ifdef BSP_USING_FDCAN2
			//CAN2
			/* Retreive Rx messages from RX FIFO0 */
			rt_hw_can_isr(&st_DrvCan2.device, RT_CAN_EVENT_RX_IND | 0 << 8);
#endif
		}
	}
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
	if(hfdcan->Instance == FDCAN1)
	{
#ifdef BSP_USING_FDCAN1
		//can1
		rt_hw_can_isr(&st_DrvCan1.device, RT_CAN_EVENT_TX_DONE | ((BufferIndexes-1) << 8));
#endif
	}
	else
	{
#ifdef BSP_USING_FDCAN2
		//can2
		rt_hw_can_isr(&st_DrvCan2.device, RT_CAN_EVENT_TX_DONE | ((BufferIndexes-1) << 8));
#endif
	}

}


void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
	if(hfdcan->Instance == FDCAN1)
	{
		//can1
	}
	else
	{
		//can2
	}
}

void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{

}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
	rt_uint32_t tmp_u32Errcount;
	rt_uint32_t tmp_u32status;
	uint32_t ret = HAL_FDCAN_GetError(hfdcan);

	if(hfdcan->Instance == FDCAN1)
	{
#ifdef BSP_USING_FDCAN1
		//can1
		if(	(ret & FDCAN_IT_ARB_PROTOCOL_ERROR) &&
			(hfdcan->Instance->CCCR & FDCAN_CCCR_INIT_Msk))
		{
			//hfdcan->Instance->CCCR |= FDCAN_CCCR_CCE_Msk;
			hfdcan->Instance->CCCR &= ~FDCAN_CCCR_INIT_Msk;
			st_DrvCan1.device.status.errcode = 0xff;
		}
		else
		{
			tmp_u32Errcount = st_DrvCan1.CanHandle.Instance->ECR;
			tmp_u32status = st_DrvCan1.CanHandle.Instance->PSR;

			st_DrvCan1.device.status.rcverrcnt = (tmp_u32Errcount>>8)&0x000000ff;
			st_DrvCan1.device.status.snderrcnt = (tmp_u32Errcount)&0x000000ff;
			st_DrvCan1.device.status.lasterrtype = tmp_u32status&0x000000007;
		}
#endif /*BSP_USING_FDCAN1*/
	}
	else
	{
#ifdef BSP_USING_FDCAN2
		if(	(ret & FDCAN_IT_ARB_PROTOCOL_ERROR) &&
			(hfdcan->Instance->CCCR & FDCAN_CCCR_INIT_Msk))
		{
			//hfdcan->Instance->CCCR |= FDCAN_CCCR_CCE_Msk;
			hfdcan->Instance->CCCR &= ~FDCAN_CCCR_INIT_Msk;
			st_DrvCan2.device.status.errcode = 0xff;
		}
		else
		{
			//can2
			tmp_u32Errcount = st_DrvCan2.CanHandle.Instance->ECR;
			tmp_u32status = st_DrvCan2.CanHandle.Instance->PSR;
			st_DrvCan2.device.status.rcverrcnt = (tmp_u32Errcount>>8)&0x000000ff;
			st_DrvCan2.device.status.snderrcnt = (tmp_u32Errcount)&0x000000ff;
			st_DrvCan2.device.status.lasterrtype = tmp_u32status&0x000000007;
		}
#endif /*BSP_USING_FDCAN2*/
	}
}

#ifdef BSP_USING_CAN1
// /**
//  * @brief This function handles FDCAN1 TX interrupts. transmit fifo0/1/2 is empty can trigger this interrupt
//  */
// void CAN1_TX_IRQHandler(void)
// {
//     rt_interrupt_enter();
//     _can_tx_isr(&drv_can1.device);
//     rt_interrupt_leave();
// }

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */
    rt_interrupt_enter();

  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&drv_can1.CanHandle);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */
    rt_interrupt_leave();

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}
/**
  * @brief This function handles FDCAN1 interrupt 1.
  */
void FDCAN1_IT1_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT1_IRQn 0 */
    rt_interrupt_enter();

  /* USER CODE END FDCAN1_IT1_IRQn 0 */
  HAL_FDCAN_IRQHandler(&drv_can1.CanHandle);
  /* USER CODE BEGIN FDCAN1_IT1_IRQn 1 */
    rt_interrupt_leave();

  /* USER CODE END FDCAN1_IT1_IRQn 1 */
}

#endif /* BSP_USING_CAN1 */

#ifdef BSP_USING_CAN2


/**
  * @brief This function handles FDCAN2 interrupt 0.
  */
void FDCAN2_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN2_IT0_IRQn 0 */
    rt_interrupt_enter();

  /* USER CODE END FDCAN2_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&drv_can2.CanHandle);
  /* USER CODE BEGIN FDCAN2_IT0_IRQn 1 */
    rt_interrupt_leave();

  /* USER CODE END FDCAN2_IT0_IRQn 1 */
}


/**
  * @brief This function handles FDCAN2 interrupt 1.
  */
void FDCAN2_IT1_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN2_IT1_IRQn 0 */
    rt_interrupt_enter();

  /* USER CODE END FDCAN2_IT1_IRQn 0 */
  HAL_FDCAN_IRQHandler(&drv_can2.CanHandle);
  /* USER CODE BEGIN FDCAN2_IT1_IRQn 1 */
    rt_interrupt_leave();

  /* USER CODE END FDCAN2_IT1_IRQn 1 */
}



#endif /* BSP_USING_CAN2 */


/**
  * @brief This function handles FDCAN calibration unit interrupt.
  */
void FDCAN_CAL_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN_CAL_IRQn 0 */
    rt_interrupt_enter();

  /* USER CODE END FDCAN_CAL_IRQn 0 */
    #ifdef BSP_USING_CAN1
  HAL_FDCAN_IRQHandler(&drv_can1.CanHandle);
  #endif
  #ifdef BSP_USING_CAN2

  HAL_FDCAN_IRQHandler(&drv_can2.CanHandle);
  #endif
  /* USER CODE BEGIN FDCAN_CAL_IRQn 1 */
    rt_interrupt_leave();

  /* USER CODE END FDCAN_CAL_IRQn 1 */
}
// /**
//  * @brief  Error CAN callback.
//  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
//  *         the configuration information for the specified CAN.
//  * @retval None
//  */
// void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
// {
//     __HAL_CAN_ENABLE_IT(hcan, CAN_IT_ERROR_WARNING |
//                         CAN_IT_ERROR_PASSIVE |
//                         CAN_IT_BUSOFF |
//                         CAN_IT_LAST_ERROR_CODE |
//                         CAN_IT_ERROR |
//                         CAN_IT_RX_FIFO0_MSG_PENDING |
//                         CAN_IT_RX_FIFO0_OVERRUN |
//                         CAN_IT_RX_FIFO0_FULL |
//                         CAN_IT_RX_FIFO1_MSG_PENDING |
//                         CAN_IT_RX_FIFO1_OVERRUN |
//                         CAN_IT_RX_FIFO1_FULL |
//                         CAN_IT_TX_MAILBOX_EMPTY);
// }

int rt_hw_can_init(void)
{
    struct can_configure config = CANDEFAULTCONFIG;
    config.baud_rate = CAN250kBaud;
    config.msgboxsz = 48;
    config.sndboxnumber = 1;
    config.mode = RT_CAN_MODE_NORMAL;
    config.privmode = RT_CAN_MODE_NOPRIV;
    config.ticks = 50;
     /* config default filter */
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0;
    sFilterConfig.FilterID2 = 0;

#ifdef BSP_USING_CAN1
    filterConf.FilterIndex = 0;

    drv_can1.FilterConfig = sFilterConfig;
    drv_can1.device.config = config;
    /* register FDCAN1 device */
    rt_hw_can_register(&drv_can1.device,
                       drv_can1.name,
                       &_can_ops,
                       &drv_can1);
#endif /* BSP_USING_CAN1 */

#ifdef BSP_USING_CAN2
    //filterConf.FilterIndex = filterConf.SlaveStartFilterBank;

    drv_can2.FilterConfig = sFilterConfig;
    drv_can2.device.config = config;
    /* register FDCAN2 device */
    rt_hw_can_register(&drv_can2.device,
                       drv_can2.name,
                       &_can_ops,
                       &drv_can2);
#endif /* BSP_USING_CAN2 */

    return 0;
}

INIT_BOARD_EXPORT(rt_hw_can_init);

#endif /* BSP_USING_CAN */

/************************** end of file ******************/
