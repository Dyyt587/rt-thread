#include "FDILink.h"
#include "string.h"
#include "main.h"
#include "fdilink_decode.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <rtdbg.h>

extern uint8_t CRC8_Table(uint8_t* p, uint8_t counter);
extern uint16_t CRC16_Table(uint8_t* p, uint8_t counter);

#ifndef FDI_ASSERT
#define FDI_ASSERT(x)
#endif

FDILink_Status_t FDILink;
FDILink_AHRSData_Packet_t AHRSData;
static rt_device_t serial;
static rt_size_t rec_size;
struct rt_semaphore fdi_sem1;
	uint8_t buf_fdi;

#define FDI_UART_NAME       "uart2"      /* 串口设备名称 */

static void imu_handle(void*params);
static inline int FDILink_Receive(FDILink_Status_t* FDILink, uint8_t value);

int Drv_FDI_init(void)
{

		
  rt_thread_t tid_chassis = RT_NULL;

    /* 创建线程， 名称是 thread_test， 入口是 thread_entry*/
    tid_chassis = rt_thread_create("imu",
                                   imu_handle, RT_NULL,
                                   4096,
                                   8, 1);

    /* 线程创建成功，则启动线程 */
    if (tid_chassis != RT_NULL)
    {
        rt_thread_startup(tid_chassis);
    }
    return 0;
}INIT_DEVICE_EXPORT(Drv_FDI_init);

rt_err_t fdi_uart_rx_ind(rt_device_t dev, rt_size_t size)
{
	//LOG_D("ded %d ",size);
		rec_size = size;
		rt_sem_release(&fdi_sem1);

		return RT_EOK;
}


 void imu_handle(void*params)
 {
	 	/* 查找串口设备 */
    serial = rt_device_find(FDI_UART_NAME);
    if (!serial)
    {
        rt_kprintf("find %s(imu) failed!\n", FDI_UART_NAME);
        return ;
    }

	if (rt_device_open(serial, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_RX_NON_BLOCKING) != RT_EOK)
	{
		LOG_E("%s(imu) open failed",FDI_UART_NAME);
		return ;
	}
//			DATA_BITS_8,				/* 8 databits */
//		STOP_BITS_2,				/* 1 stopbit */
//		PARITY_EVEN,				/* No parity  */
	struct serial_configure config = {
		BAUD_RATE_921600,						/* 115200 bits/s */
		DATA_BITS_8,				/* 8 databits */
		STOP_BITS_1,				/* 1 stopbit */
		PARITY_NONE,				/* No parity  */
		BIT_ORDER_LSB,				/* LSB first sent */
		NRZ_NORMAL,					/* Normal mode */
		4096,		/* rxBuf size */
		4096,		/* txBuf size */
		RT_SERIAL_FLOWCONTROL_NONE, /* Off flowcontrol */
		0};
//	if (RT_EOK != rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config))
//	{
//		rt_kprintf("change %s(imu) failed!\n", serial->parent.name);
//	}
		rt_sem_init(&fdi_sem1, "imu_sem", 0, RT_IPC_FLAG_FIFO);

		if (rt_device_set_rx_indicate(serial, fdi_uart_rx_ind) != RT_EOK)
	{
		LOG_E("%s(imu) set rx indicate failed",serial->parent.name);
		//return -1;
	}
		while(1)
		{
			//LOG_D("euwh");
			if(rt_device_read(serial,-1,&buf_fdi,1)==0)
			{
				rt_sem_take(&fdi_sem1,RT_WAITING_FOREVER);
				continue;
			}
			FDILink_Receive(&FDILink,buf_fdi);		
			
			//rt_thread_mdelay(10);
		}
 }

void FDILink_Decode(int type,void* buf)
{
	switch(type)
	{
		case FDILINK_IMUDATA_PACKET_ID:  //MSG_IMU
		{
			FDILink_IMUData_Packet_t IMUData;
			memcpy(&IMUData,buf,sizeof(FDILink_IMUData_Packet_t));

			break;
		}
		case FDILINK_AHRSDATA_PACKET_ID:   //MSG_AHRS
		{
			memcpy(&AHRSData,buf,sizeof(FDILink_AHRSData_Packet_t));
		LOG_D("test");
			break;
		}
		case FDILINK_INSGPSDATA_PACKET_ID:   //MSG_INS/GPS
		{
			FDILink_INSGPSData_Packet_t data;
			memcpy(&data,buf,sizeof(FDILink_INSGPSData_Packet_t));
			
			break;
		}
		case System_State_Packet_ID:  //MSG_SYS_STATE
		{
			System_State_Packet_t data;
			memcpy(&data,buf,sizeof(System_State_Packet_t));
			
			break;
		}
		case Raw_GNSS_Packet_ID:   //MSG_RAW_GNSS
		{
			Raw_GNSS_Packet_t data;
			memcpy(&data,buf,sizeof(Raw_GNSS_Packet_t));
			
			
			break;
		}
	}
}



void FDILink_Error(FDILink_Status_t* FDILink)
{
	FDILink->RxStatus = FDILink_Frame_Start;
}
void FDILink_Insert(FDILink_Status_t* FDILink, uint8_t value)
{
	if (FDILink->RxDataLeft <= 0)
	{
		FDILink_Error(FDILink);
		return;
	}
	if (FDILink->RxStatus != FDILink_Frame_Data)
	{
		FDILink_Error(FDILink);
		return;
	}
	FDILink->Buffer[FDILink->BufferIndex++] = value;  //将元素放入队列尾部
	if (FDILink->BufferIndex >= 256)
	{
		FDILink_Error(FDILink);
		return;
	}
	FDILink->RxDataLeft--;
}
void FDILink_Reset(FDILink_Status_t* FDILink)
{
	FDILink->RxStatus = FDILink_Frame_Start;
	FDILink->RxDataLeft = 0;
	FDILink->RxType = 0;
	FDILink->BufferIndex = 0;
	FDILink->TxNumber = 0;
}
int FDILink_RunningData(FDILink_Status_t* FDILink, uint8_t value)
{
	if (FDILink->RxStatus < FDILink_Frame_Start || FDILink->RxStatus > FDILink_Frame_End)
	{
		FDILink_Error(FDILink);
		return -3;
	}
	FDILink->FDILink_Frame_Buffer[FDILink->RxStatus] = value;
	switch (FDILink->RxStatus)
	{
		case FDILink_Frame_Start:
			FDILink_Reset(FDILink);
			if (value == FDILink_Connect_Flag)
			{
				return 0;
			}
			if (value != FDILink_STX_Flag)
			{
				FDILink_Error(FDILink);
				return -1;
			}
			FDILink->RxStatus = FDILink_Frame_CMD;
			break;
		case FDILink_Frame_CMD:
			FDILink->RxType = value;
			FDILink->RxStatus = FDILink_Frame_Length;
			break;
		case FDILink_Frame_Length:
			FDILink->RxDataLeft = value;
			FDILink->RxStatus = FDILink_Frame_SerialNumber;
			break;
		case FDILink_Frame_SerialNumber:
			FDILink->RxNumber = value;
			FDILink->RxStatus = FDILink_Frame_CRC8;
			break;
		case FDILink_Frame_CRC8:
			FDILink->CRC8_Verify = value;
			if (CRC8_Table(FDILink->FDILink_Frame_Buffer, FDILink_Frame_CRC8) != FDILink->CRC8_Verify)
			{
				FDILink_Error(FDILink);
				return -1;
			}
			if(FDILink->RxDataLeft == 0)
			{
				FDILink->RxStatus = FDILink_Frame_Start;
				return 1;
			}
			FDILink->RxStatus = FDILink_Frame_CRC16H;
			break;
		case FDILink_Frame_CRC16H:
			FDILink->CRC16_Verify = value;
			FDILink->RxStatus = FDILink_Frame_CRC16L;
			break;
		case FDILink_Frame_CRC16L:
			FDILink->CRC16_Verify = (FDILink->CRC16_Verify << 8) | value;
			FDILink->RxStatus = FDILink_Frame_Data;
			break;
		case FDILink_Frame_Data:
			if (FDILink->RxDataLeft)
			{
				FDILink_Insert(FDILink,value);
				if (FDILink->RxDataLeft == 0)
				{
					FDILink->RxStatus = FDILink_Frame_End;
				}
				break;
			}
			else
			{
				FDILink->RxStatus = FDILink_Frame_End;
			}

		case FDILink_Frame_End:
		{
			if (value != FDILink_EDX_Flag)
			{
				FDILink_Error(FDILink);
				return -1;
			}
			uint16_t CRC16 = CRC16_Table(FDILink->Buffer, FDILink->BufferIndex);
			if (CRC16 != FDILink->CRC16_Verify)
			{
				FDILink_Error(FDILink);
				return -1;
			}
			FDILink->RxStatus = FDILink_Frame_Start;
			return 1;
		}
		default:
			FDILink_Error(FDILink);
			return -1;
	}
	return 0;
}
int FDILink_CheckData(FDILink_Status_t* FDILink, uint8_t len)
{
	if (FDILink->BufferIndex != len)
	{
		FDILink_Error(FDILink);
		return -1;
	}
	return 0;
}
extern void FDILink_Decode(int type,void* buf);
void FDILink_Effect(FDILink_Status_t* FDILink)
{
	FDILink_Decode(FDILink->RxType,FDILink->Buffer);
}
int FDILink_Send(FDILink_Status_t* FDILink, uint8_t type, uint8_t * buf, int len)
{
	uint8_t buffer[256];
	FDI_ASSERT(len < 248);
	buffer[FDILink_Frame_Start] = FDILink_STX_Flag;
	buffer[FDILink_Frame_CMD] = type;
	buffer[FDILink_Frame_Length] = len;
	buffer[FDILink_Frame_SerialNumber] = FDILink->TxNumber++;
	uint8_t CRC8 = CRC8_Table(buffer, FDILink_Frame_CRC8);
	buffer[FDILink_Frame_CRC8] = CRC8;
	if(len == 0)
	{
		//没有CRC16校验和结束符
		return Serial_Send(buffer,FDILink_Frame_CRC8 + 1);
	}
	else
	{
		uint8_t* buf_data = buffer + FDILink_Frame_Data;
		memcpy(buf_data,buf,len);
		uint16_t CRC16 = CRC16_Table(buf_data, len);
		buffer[FDILink_Frame_CRC16H] = (CRC16 >> 8);
		buffer[FDILink_Frame_CRC16L] = (CRC16 & 0xff);
		buffer[FDILink_Frame_End + len - 1] = FDILink_EDX_Flag;
		return Serial_Send(buffer,FDILink_Frame_End + len);
	}
}

int FDILink_Pack(uint8_t* buffer, FDILink_Status_t* FDILink, uint8_t type, void* buf, int len)
{
	FDI_ASSERT(len < 248);
	buffer[FDILink_Frame_Start] = FDILink_STX_Flag;
	buffer[FDILink_Frame_CMD] = type;
	buffer[FDILink_Frame_Length] = len;
	buffer[FDILink_Frame_SerialNumber] = FDILink->TxNumber++;
	uint8_t CRC8 = CRC8_Table(buffer, FDILink_Frame_CRC8);
	buffer[FDILink_Frame_CRC8] = CRC8;

	uint8_t* buf_data = buffer + FDILink_Frame_Data;
	//memcpy(buf_data,buf,len);
	for(int i = 0;i < len;i++)
	{
		buf_data[i] = ((uint8_t*)buf)[i];
	}
	uint16_t CRC16 = CRC16_Table(buf_data, len);
	buffer[FDILink_Frame_CRC16H] = (CRC16 >> 8);
	buffer[FDILink_Frame_CRC16L] = (CRC16 & 0xff);
	buffer[FDILink_Frame_End + len - 1] = FDILink_EDX_Flag;
	return FDILink_Frame_End + len;
}

static inline int FDILink_Receive(FDILink_Status_t* FDILink, uint8_t value)
{
	FDI_ASSERT(FDILink->BootStatus == FDILink_Status_Running);
	uint8_t result = FDILink_RunningData(FDILink, value);
	if (result == 1)
	{
		FDILink_Effect(FDILink);
	}
	return result;
}

int FDILink_Init(FDILink_Status_t* FDILink)
{
	FDILink->BufferIndex = 0;
	FDILink->BootStatus = FDILink_Status_Running;
	FDILink->RxStatus = FDILink_Frame_Start;
	for (int i = 0; i < 256; i++)
	{
		FDILink->Buffer[i] = 0;
	}
	return 0;
}


int FDILink_Recv(FDILink_Status_t* FDILink, uint8_t * buf, int len)
{
	for(int i = 0;i < len;i++)
	{
		FDILink_Receive(FDILink, buf[i]);
	}
	return 0;
}

//extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart2;
//extern uint8_t 	USART2_RX_BUF[256];
//extern uint16_t    USART2_RX_STA;
//extern uint16_t    USART2_RX_LEN;
//static FDILink_Status_t _FDILink;
//uint8_t p_buff[256];

/*!
 *  FDILink_SDK说明
 *  此SDK配置了DETA系列主串口（MAIN）可以接收的各式指令，通过串口收发来实现
 *  HAL_UART_Transmit_IT	用户可以根据设备变化修改此处的串口发送函数来适配
 *  HAL_UART_RxCpltCallback		此处为串口的中断接收函数，用户参考此函数来适配
 */


///*!
// *  配置外部SLAM1数据包，所有外部数据包依照此格式配置（所有结构体在fdilink_decode.h中）
// *	Configure external SLAM1 packets. All external packets are configured in this format.
// *	\param[out]	None
// *	\param[in]	None
// *	\return		None
// */
//void fdiSendSLAM1(void)
//{
//	External_SLAM1_Packet_t SLAMData;
//	
//	SLAMData.Position_X 				   = 1.23456;
//	SLAMData.Position_Y 				   = 2.23456;
//	SLAMData.Position_Z 				   = 3.23456;
//	SLAMData.Velocity_X 				   = 4.23456;
//	SLAMData.Velocity_Y 				   = 5.23456;
//	SLAMData.Velocity_Z					   = 6.23456;
//	SLAMData.Roll       				   = 7.23456;
//	SLAMData.Pitch     					   = 8.23456;
//	SLAMData.Yaw        				   = 9.23456;
//	SLAMData.Position_X_standard_deviation = 0.23456;
//	SLAMData.Position_Y_standard_deviation = 0.23456;
//	SLAMData.Position_Z_standard_deviation = 0.23456;
//	SLAMData.Velocity_X_standard_deviation = 0.23456;
//	SLAMData.Velocity_Y_standard_deviation = 0.23456;
//	SLAMData.Velocity_Z_standard_deviation = 0.23456;
//	SLAMData.Roll_standard_deviation       = 2.23456;
//	SLAMData.Pitch_standard_deviation      = 2.23456;
//	SLAMData.Yaw_standard_deviation        = 2.23456;
//	
//	fdiSendExternalData(External_SLAM1_Packet_ID, &SLAMData, sizeof(External_SLAM1_Packet_t));
//}

///*!
// *  发送配置后的外部数据包给DETA10，type为数据包ID，buffer为配置后的数据包，length为buffer包含字节的长度
// *	Send configured external packets.
// *	\param[out]	None
// *	\param[in]	ID - the data frame ID number to be requested.
// *	\return		FDI_NO_ERROR if we have entered configuration mode.
// */
//int fdiSendExternalData(int type, void* buffer, int length)
//{
//	uint8_t buf[256];
//	FDILink_Pack(buf, &_FDILink, type, buffer, length);
//	HAL_Delay(10);
//	HAL_UART_Transmit_IT(&huart2, buf, strlen(buf));
//	HAL_UART_Transmit_IT(&huart1, buf, strlen(buf));
//	//USART2_RX_STA = 0;
//	return FDI_NO_ERROR;
//}

///*!
// *  请求数据帧并解析，ID为要请求的数据帧ID编号。系统会返回当前时刻对应数据输出，如果该帧被设置为固定频率输出，则会持续输出。
// *  使用此指令会自动解析要获得的数据至构建好的结构体中，FDILink_Decode中配置了部分数据包，用户可自行添加需要解析的数据包
// *	Request and analysis data frame.
// *	\param[out]	None
// *	\param[in]	ID - the data frame ID number to be requested.
// *	\return		FDI_NO_ERROR if we have entered configuration mode.
// */
//int fdiGetPacket(uint8_t ID)
//{
//	
//	uint8_t buffer[4];
//	buffer[0] = ID;
//	buffer[1] = 0;
//	buffer[2] = 0;
//	buffer[3] = 0;
//	FDILink_Pack(p_buff, &_FDILink, 0xA0, buffer, sizeof(buffer));
//	for(int i =0;i<150;i++)
//	{
//		HAL_Delay(10);
//		HAL_UART_Transmit_IT(&huart2, p_buff, 12);
////		USART2_RX_STA = 0;
////		HAL_Delay(10);
//		FDILink_Recv(&_FDILink, USART2_RX_BUF, USART2_RX_STA);
//	}
//	return FDI_NO_ERROR;
//}

///*!
// *  进入配置模式
// *	Enter configuration mode.
// *	\param[out]	None
// *	\param[in]	None
// *	\return		FDI_NO_ERROR if we have entered configuration mode.
// */
//int fdiSendConfig(void)
//{
//	HAL_UART_Transmit_IT(&huart1,"#fconfig\r\n",sizeof("#fconfig\r\n"));
//	HAL_Delay(1000);
//	HAL_UART_Transmit_IT(&huart2,"#fconfig\r\n",sizeof("#fconfig\r\n"));
//	HAL_Delay(500);
//	HAL_UART_Transmit_IT(&huart2,"#fconfig\r\n",sizeof("#fconfig\r\n"));
//	HAL_Delay(500);
//	HAL_UART_Transmit_IT(&huart2,"#fconfig\r\n",sizeof("#fconfig\r\n"));
//	USART2_RX_STA = 0;
//	HAL_Delay(200);

//	HAL_UART_Transmit_IT(&huart1,USART2_RX_BUF,USART2_RX_STA);
//	while(huart1.gState != HAL_UART_STATE_READY){};
//	
//	int res = strstr((char*)USART2_RX_BUF , "*#OK") != 0 ? 1 : 0;
//	if(res)
//		return FDI_NO_ERROR;
//	else
//		return FDI_ERROR;
//	HAL_Delay(100);
//}

///*!
// *  退出配置模式
// *	Exit configuration mode.
// *	\param[out]	None
// *	\param[in]	None
// *	\return		FDI_NO_ERROR if we have exited configuration mode.
// */
//int fdiSendDeconfig(void)
//{
//	HAL_UART_Transmit_IT(&huart1,"#fdeconfig\r\n",sizeof("#fdeconfig\r\n"));
//	HAL_Delay(1000);
//	HAL_UART_Transmit_IT(&huart2,"#fdeconfig\r\n",sizeof("#fdeconfig\r\n"));
//	USART2_RX_STA = 0;

//	HAL_UART_Transmit_IT(&huart1,USART2_RX_BUF,USART2_RX_STA);
//	while(huart1.gState != HAL_UART_STATE_READY){};
//	int res = strstr((char*)USART2_RX_BUF , "*#OK") != 0 ? 1 : 0;
//	if(res)
//		return FDI_NO_ERROR;
//	else
//		return FDI_ERROR;
//	HAL_Delay(100);
//}

///*!
// *  重启设备（重要数据的更新需要重启设备）
// *	Restart the device (the device needs to be restarted for the update of important data).
// *	\param[out]	None
// *	\param[in]	None
// *	\return		FDI_NO_ERROR if we have restarted the device.
// */
//int fdiSendReboot(void)
//{
//	HAL_UART_Transmit_IT(&huart1,"#freboot\r\n",sizeof("#freboot\r\n"));
//	HAL_Delay(1000);
//	HAL_UART_Transmit_IT(&huart2,"#freboot\r\n",sizeof("#freboot\r\n"));
//	HAL_Delay(1500);
//	HAL_UART_Transmit_IT(&huart2,"y\r\n",sizeof("y\r\n"));
//	USART2_RX_STA = 0;
//	HAL_Delay(200);

//	HAL_UART_Transmit_IT(&huart1,USART2_RX_BUF,USART2_RX_STA);
//	while(huart1.gState != HAL_UART_STATE_READY){};
//	
//	int res = strstr((char*)USART2_RX_BUF , "*#OK") != 0 ? 1 : 0;
//	if(res)
//		return FDI_NO_ERROR;
//	else
//		return FDI_ERROR;
//	HAL_Delay(100);
//}

///*!
// *  恢复出厂设置
// *	Restore factory settings.
// *	\param[out]	None
// *	\param[in]	None
// *	\return		FDI_NO_ERROR if we have restored factory settings.
// */
//int fdiSendReset(void)
//{
//	HAL_UART_Transmit_IT(&huart1,"#freset\r\n",sizeof("#freset\r\n"));
//	HAL_Delay(1000);
//	HAL_UART_Transmit_IT(&huart2,"#freset\r\n",sizeof("#freset\r\n"));
//	HAL_Delay(1500);
//	HAL_UART_Transmit_IT(&huart2,"y\r\n",sizeof("y\r\n"));
//	USART2_RX_STA = 0;
//	HAL_Delay(200);

//	HAL_UART_Transmit_IT(&huart1,USART2_RX_BUF,USART2_RX_STA);
//	while(huart1.gState != HAL_UART_STATE_READY){};
//	
//	int res = strstr((char*)USART2_RX_BUF , "*#OK") != 0 ? 1 : 0;
//	if(res)
//		return FDI_NO_ERROR;
//	else
//		return FDI_ERROR;
//	HAL_Delay(100);
//}

///*!
// *  查询设备的安装方向
// *	Query the installation direction of the equipment.
// *	\param[out]	None
// *	\param[in]	None
// *	\return		FDI_NO_ERROR if we have gotten the info.
// */
//int fdiGetAxis(void)
//{
//	HAL_UART_Transmit_IT(&huart1,"#faxis\r\n",sizeof("#faxis\r\n"));
//	HAL_Delay(1000);
//	HAL_UART_Transmit_IT(&huart2,"#faxis\r\n",sizeof("#faxis\r\n"));
//	USART2_RX_STA = 0;
//	HAL_Delay(100);

//	HAL_UART_Transmit_IT(&huart1,USART2_RX_BUF,USART2_RX_STA);
//	while(huart1.gState != HAL_UART_STATE_READY){};
//	return FDI_NO_ERROR;
//	HAL_Delay(100);
//}

///*!
// *  设置设备的安装方向：flip可选为绕x、y、z轴旋转，rot（0-360度）为从设备顶部看去顺时针为正方向
// *	Set the installation direction of the equipment.
// *	\param[out]	None
// *	\param[in]	flip - choose one of value from x, y or z; rot - range from 0 to 360(degree)
// *	\return		FDI_NO_ERROR if we have set the param.
// */
//int fdiSendAxis(char* flip, int rot)
//{
//	char send_buff[128];
//	sprintf(send_buff, "#faxis %s %d\r\n", flip, rot);
//	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
//	HAL_Delay(1000);
//	HAL_UART_Transmit_IT(&huart2,(uint8_t*)send_buff,strlen(send_buff));
//	USART2_RX_STA = 0;
//	HAL_Delay(100);

//	HAL_UART_Transmit_IT(&huart1,USART2_RX_BUF,USART2_RX_STA);
//	while(huart1.gState != HAL_UART_STATE_READY){};
//		
//	int res = strstr((char*)USART2_RX_BUF , "*#OK") != 0 ? 1 : 0;
//	if(res)
//		return FDI_NO_ERROR;
//	else
//		return FDI_ERROR;
//	HAL_Delay(100);
//}

///*!
// *  查询双天线航向与载体前向夹角（主天线到从天线为基线矢量正方向，从IMU正上方看去，顺时针为天线航向偏角的正方向）
// *	Query the included angle between dual antenna heading and carrier forward.
// *	\param[out]	None
// *	\param[in]	None
// *	\return		FDI_NO_ERROR if we have set the param.
// */
//int fdiGetAnte(void)
//{
//	HAL_UART_Transmit_IT(&huart1,"#fante\r\n",sizeof("#fante\r\n"));
//	HAL_Delay(1000);
//	HAL_UART_Transmit_IT(&huart2,"#fante\r\n",sizeof("#fante\r\n"));
//	USART2_RX_STA = 0;
//	HAL_Delay(100);

//	HAL_UART_Transmit_IT(&huart1,USART2_RX_BUF,USART2_RX_STA);
//	while(huart1.gState != HAL_UART_STATE_READY){};
//	return FDI_NO_ERROR;
//	HAL_Delay(100);
//}

///*!
// *  配置双天线航向偏角，其中angle为角度值（0-360度）
// *	Configure dual antenna heading angle.
// *	\param[out]	None
// *	\param[in]	angle - dual antenna heading angle.
// *	\return		FDI_NO_ERROR if we have set the param.
// */
//int fdiSendAnteHeadbias(int angle)
//{
//	char send_buff[128];
//	sprintf(send_buff, "#fanteheadbias %d\r\n", angle);
//	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
//	HAL_Delay(1000);
//	HAL_UART_Transmit_IT(&huart2,(uint8_t*)send_buff,strlen(send_buff));
//	USART2_RX_STA = 0;
//	HAL_Delay(100);

//	HAL_UART_Transmit_IT(&huart1,USART2_RX_BUF,USART2_RX_STA);
//	while(huart1.gState != HAL_UART_STATE_READY){};
//		
//	int res = strstr((char*)USART2_RX_BUF , "*#OK") != 0 ? 1 : 0;
//	if(res)
//		return FDI_NO_ERROR;
//	else
//		return FDI_ERROR;
//	HAL_Delay(100);
//}

///*!
//*  配置双天线之间的基线长度，length单位为米（m）
// *	Configure dual antenna baseline length.
// *	\param[out]	None
// *	\param[in]	length - dual antenna baseline length.
// *	\return		FDI_NO_ERROR if we have set the param.
// */
//int fdiSendAnteBaseline(int length)
//{
//	char send_buff[128];
//	sprintf(send_buff, "#fantebaseline %d\r\n", length);
//	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
//	HAL_Delay(1000);
//	HAL_UART_Transmit_IT(&huart2,(uint8_t*)send_buff,strlen(send_buff));
//	USART2_RX_STA = 0;
//	HAL_Delay(100);

//	HAL_UART_Transmit_IT(&huart1,USART2_RX_BUF,USART2_RX_STA);
//	while(huart1.gState != HAL_UART_STATE_READY){};
//		
//	int res = strstr((char*)USART2_RX_BUF , "*#OK") != 0 ? 1 : 0;
//	if(res)
//		return FDI_NO_ERROR;
//	else
//		return FDI_ERROR;
//	HAL_Delay(100);
//}

///*!
// *  配置GNSS主天线到IMU的杆臂命令
// *	Configure lever arm command from GNSS main antenna to IMU.
// *	\param[out]	None
// *	\param[in]	(x,y,z)
// *	\return		FDI_NO_ERROR if we have set the param.
// */
//int fdiSendAnteArm(int x, int y, int z)
//{
//	char send_buff[128];
//	sprintf(send_buff, "#fantearm %d %d %d\r\n", x, y, z);
//	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
//	HAL_Delay(1000);
//	HAL_UART_Transmit_IT(&huart2,(uint8_t*)send_buff,strlen(send_buff));
//	USART2_RX_STA = 0;
//	HAL_Delay(100);

//	HAL_UART_Transmit_IT(&huart1,USART2_RX_BUF,USART2_RX_STA);
//	while(huart1.gState != HAL_UART_STATE_READY){};
//		
//	int res = strstr((char*)USART2_RX_BUF , "*#OK") != 0 ? 1 : 0;
//	if(res)
//		return FDI_NO_ERROR;
//	else
//		return FDI_ERROR;
//	HAL_Delay(100);
//}

///*!
// *  校准陀螺仪、加表的常值零偏以及调平，Level和Acce需要在水平静止的状态下执行该命令，Gyro只需要模块保持静止
// *	Calibrate the constant zero bias and leveling of gyroscope and meter.
// *	\param[out]	None
// *	\param[in]	Level, Acce or Gyro
// *	\return		FDI_NO_ERROR if we have set the param.
// */
//int fdiSendImucal(int num)
//{
//	char send_buff[128];
//	switch(num)
//	{
//		case Level:
//			sprintf(send_buff, "#fimucal_level\r\n");//将IMU坐标系调至水平面，不改变陀螺和加表零偏
//		case Acce:
//			sprintf(send_buff, "#fimucal_acce\r\n");//执行加速度计零偏校准
//		case Gyro:
//			sprintf(send_buff, "#fimucal_gyro\r\n");//执行陀螺仪零偏校准
//		default:
//			break;
//	}
//	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
//	HAL_Delay(1000);
//	HAL_UART_Transmit_IT(&huart2,(uint8_t*)send_buff,strlen(send_buff));
//	USART2_RX_STA = 0;
//	HAL_Delay(100);

//	HAL_UART_Transmit_IT(&huart1,USART2_RX_BUF,USART2_RX_STA);
//	while(huart1.gState != HAL_UART_STATE_READY){};
//		
//	int res = strstr((char*)USART2_RX_BUF , "*#OK") != 0 ? 1 : 0;
//	if(res)
//		return FDI_NO_ERROR;
//	else
//		return FDI_ERROR;
//	HAL_Delay(100);
//}

///*!
// *  配置发送的数据内容，msg为2位16进制数字表示数据包ID，freq为设置指定数据包的发送频率
// *	Configure the data content to be sent.
// *	\param[out]	None
// *	\param[in]	msg - Represents the packet ID; freq - Packet transmit frequency.
// *	\return		FDI_NO_ERROR if we have set the param.
// */
//int fdiSendMsg(char* msg, int freq)
//{
//	char send_buff[128];
//	sprintf(send_buff, "#fmsg %s %d\r\n", msg, freq);
//	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
//	HAL_Delay(1000);
//	HAL_UART_Transmit_IT(&huart2,(uint8_t*)send_buff,strlen(send_buff));
//	USART2_RX_STA = 0;
//	HAL_Delay(100);

//	HAL_UART_Transmit_IT(&huart1,USART2_RX_BUF,USART2_RX_STA);
//	while(huart1.gState != HAL_UART_STATE_READY){};
//	return FDI_NO_ERROR;
//	HAL_Delay(100);
//}

///*!
// *  读取参数，paramName为需要获取的参数名称
// *	Read parameters.
// *	\param[out]	None
// *	\param[in]	paramName - Parameter name to be obtained.
// *	\return		FDI_NO_ERROR if we have set the param.
// */
//int fdiGetParam(char* paramName)
//{
//	char send_buff[128];
//	sprintf(send_buff, "#fparam get %s\r\n", paramName);
//	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
//	HAL_Delay(1000);
//	HAL_UART_Transmit_IT(&huart2,(uint8_t*)send_buff,strlen(send_buff));
//	USART2_RX_STA = 0;
//	HAL_Delay(100);

//	HAL_UART_Transmit_IT(&huart1,USART2_RX_BUF,USART2_RX_STA);
//	while(huart1.gState != HAL_UART_STATE_READY){};
//	return FDI_NO_ERROR;
//	HAL_Delay(100);
//}

///*!
//*  配置参数，paramName为需要设置的参数名称，paramValue为设置参数的数值（10进制）
// *	configuration parameter.
// *	\param[out]	None
// *	\param[in]	paramName - Parameter name to be obtained; paramValue - Set the value of the parameter.
// *	\return		FDI_NO_ERROR if we have set the param.
// */
//int fdiSendParam(char* paramName, int paramValue)
//{
//	char send_buff[128];
//	sprintf(send_buff, "#fparam set %s %d\r\n", paramName, paramValue);
//	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
//	HAL_Delay(1000);
//	HAL_UART_Transmit_IT(&huart2,(uint8_t*)send_buff,strlen(send_buff));
//	USART2_RX_STA = 0;
//	HAL_Delay(100);

//	HAL_UART_Transmit_IT(&huart1,USART2_RX_BUF,USART2_RX_STA);
//	while(huart1.gState != HAL_UART_STATE_READY){};
//	return FDI_NO_ERROR;
//	HAL_Delay(100);
//}

///*!
// *  保存已修改的配置（重要的数据更新需要保存）
// *	Save modified configuration.
// *	\param[out]	None
// *	\param[in]	None
// *	\return		FDI_NO_ERROR if we have set the param.
// */
//int fdiSendSave(void)
//{
//	HAL_UART_Transmit_IT(&huart1,"#fsave\r\n",sizeof("#fsave\r\n"));
//	HAL_Delay(1000);
//	HAL_UART_Transmit_IT(&huart2,"#fsave\r\n",sizeof("#fsave\r\n"));
//	USART2_RX_STA = 0;
//	HAL_Delay(200);

//	HAL_UART_Transmit_IT(&huart1,USART2_RX_BUF,USART2_RX_STA);
//	while(huart1.gState != HAL_UART_STATE_READY){};
//	
//	int res = strstr((char*)USART2_RX_BUF , "*#OK") != 0 ? 1 : 0;
//	if(res)
//		return FDI_NO_ERROR;
//	else
//		return FDI_ERROR;
//	HAL_Delay(100);
//}
