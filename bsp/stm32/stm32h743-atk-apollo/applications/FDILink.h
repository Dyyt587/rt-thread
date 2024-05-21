#ifndef __SERIAL_BOOT_H
#define __SERIAL_BOOT_H
#include <stdint.h>
//未初始化
#define FDILink_Status_Uninitialized 		0
//运行
#define FDILink_Status_Running 				2
//正忙
#define FDILink_Status_Busy 				3

#define FDILink_Frame_Start					0
#define FDILink_Frame_CMD					1
#define FDILink_Frame_Length				2
#define FDILink_Frame_SerialNumber			3
#define FDILink_Frame_CRC8					4
#define FDILink_Frame_CRC16H				5
#define FDILink_Frame_CRC16L				6
#define FDILink_Frame_Data					7
#define FDILink_Frame_End					8
typedef struct
{
	uint8_t Start;
	uint8_t CMD;
	uint8_t Length;
	uint8_t SerialNumber;
	uint8_t CRC8;
	union
	{
		uint16_t CRC16;
		struct
		{
			uint8_t CRC16L;
			uint8_t CRC16H;
		};
	};
	uint8_t Data[0];
}__attribute__((packed)) FDILink_Frame_t;

#define Storage_Type_Only_Code		0x0800
#define Storage_Type_Only_Data		0x0400
#define Storage_Type_Both			0x0000

#define Storage_Type_NONE			0x0000
#define Storage_Type_NOR_FLASH		0x0010
#define Storage_Type_NAND_FLASH		0x0020
#define Storage_Type_SRAM			0x0030
#define Storage_Type_DRAM			0x0040
#define Storage_Type_EEPROM			0x0050
#define Storage_Type_FRAM			0x0060
#define Storage_Type_ROM			0x0070
#define Storage_Type_UNKOWN			0x0080

#define Storage_Type_ExNOR_FLASH	0x8010
#define Storage_Type_ExNAND_FLASH	0x8020
#define Storage_Type_ExSRAM			0x8030
#define Storage_Type_ExDRAM			0x8040
#define Storage_Type_ExEEPROM		0x8050
#define Storage_Type_ExFRAM			0x8060
#define Storage_Type_ExUNKOWN		0x8080



typedef struct{
	uint32_t		Address;
	uint32_t		Length;
	uint32_t		Type;
}Storage_Info_t;



typedef struct{
	uint32_t		Address;
	uint32_t		Length;
	int32_t			Checked;
	uint32_t		Version;
}Firmware_Info_t;


typedef struct
{
	uint32_t 		Version_List_ID;			//列表版本ID
	uint32_t		Board_ID;					//PCB的ID
	uint32_t		Boot_Address;				//单位字节
	uint32_t		Boot_Length;				//单位字节
	uint32_t		APP_Entry;					//应用程序入口
	uint32_t		Chip_ID;					//芯片缩略ID
	uint32_t		Full_ID[4];
	Firmware_Info_t	Firmware[3];				//固件区
	Storage_Info_t	Storage[4];					//存储器
}__attribute__((packed)) FDILink_Version_t;
/***************************************************************
 *	下行指令
 **************************************************************/
#define FDILink_FrameCMD_Version			0x0c	//	获得版本
#define FDILink_FrameCMD_Keep 				0x0d	//	保持连接
#define FDILink_FrameCMD_Unlock 			0x0e	//	解锁
#define FDILink_FrameCMD_Erasure 			0x0f	//	擦除
#define FDILink_FrameCMD_Download 			0x10	//	下载
#define FDILink_FrameCMD_Upload 			0x11	//	上传
#define FDILink_FrameCMD_Verify 			0x12	//	存储校验
#define FDILink_FrameCMD_Reset 				0x13	//	跳转
#define FDILink_FrameCMD_Firmware			0x14	//	获得固件信息
#define FDILink_FrameCMD_Download_Message 	0x16	//	下载

/***************************************************************
 *	上行指令
 **************************************************************/
#define FDILink_FrameCMD_Catalog 			0x40	//	目录
#define FDILink_FrameCMD_Status 			0x41	//	当前状态
#define FDILink_FrameCMD_Error 				0x42	//	错误

#define FDILink_StatusType_Reconnecting		0x01
#define FDILink_StatusType_Processing		0x02
#define FDILink_StatusType_Idle				0x03
#define FDILink_StatusType_Break			0x04

#define FDILink_Return_OK					0
#define FDILink_Error_InvalidStart			-10	//无效的Start符
#define FDILink_Error_InvalidStatus			-11	//无效的状态
#define FDILink_Error_CRC8					-12	//错误的CRC8
#define FDILink_Error_CRC16					-13	//错误的CRC16

/***************************************************************
*	帧符号
***************************************************************/

#define FDILink_STX_Flag 0xFC
#define FDILink_EDX_Flag 0xFD
#define FDILink_Connect_Flag 0xFD

typedef struct FDILink_Status
{
	int 				BootStatus;
	int					RxStatus;
	int 				RxType;
	int 				RxDataLeft;
	int		 			RxNumber;
	int		 			TxNumber;
	int 				CRC8_Verify;
	int 				CRC16_Verify;
	uint32_t 			BufferIndex;
	uint8_t 			FDILink_Frame_Buffer[12];
	uint8_t 			Buffer[256];
}FDILink_Status_t;

#define Level    0
#define Acce     1
#define Gyro     2

#define FDI_NO_ERROR 0
#define FDI_ERROR    1


int FDILink_Init(FDILink_Status_t* FDILink);
int FDILink_Send(FDILink_Status_t* FDILink, uint8_t type, uint8_t * buf, int len);
int FDILink_Pack(uint8_t* buffer, FDILink_Status_t* FDILink, uint8_t type, void* buf, int len);
int FDILink_Recv(FDILink_Status_t* FDILink, uint8_t * buf, int len);

void fdiSendSLAM1(void);
int fdiSendExternalData(int type, void* buffer, int length);
int fdiGetPacket(uint8_t ID);
int fdiSendConfig(void);
int fdiSendDeconfig(void);
int fdiSendReboot(void);
int fdiSendReset(void);
int fdiGetAxis(void);
int fdiSendAxis(char* flip, int rot);
int fdiGetAnte(void);
int fdiSendAnteHeadbias(int angle);
int fdiSendAnteBaseline(int length);
int fdiSendAnteArm(int x, int y, int z);
int fdiSendImucal(int num);
int fdiSendMsg(char* msg, int freq);
int fdiGetParam(char* paramName);
int fdiSendParam(char* paramName, int paramValue);
int fdiSendSave(void);

#define MSG_IMU                "40"
#define MSG_AHRS               "41"
#define MSG_INSGPS             "42"
#define MSG_SYS_STATE          "50"
#define MSG_UNIX_TIME          "51"
#define MSG_FORMAT_TIME        "52"
#define MSG_STATUS             "53"
#define MSG_POS_STD_DEV        "54"
#define MSG_VEL_STD_DEV        "55"
#define MSG_EULER_ORIEN_STD_DEV "56"
#define MSG_QUAT_ORIEN_STD_DEV "57"
#define MSG_RAW_SENSORS        "58"
#define MSG_RAW_GNSS           "59"
#define MSG_SATELLITE          "5a"
#define MSG_DETAILED_SATELLITE "5b"
#define MSG_GEODETIC_POS       "5c"
#define MSG_ECEF_POS           "5d"
#define MSG_UTM_POS            "5e"
#define MSG_NED_VEL            "5f"
#define MSG_BODY_VEL           "60"
#define MSG_ACCELERATION       "61"
#define MSG_BODY_ACCELERATION  "62"
#define MSG_EULER_ORIEN        "63"
#define MSG_QUAT_ORIEN         "64"
#define MSG_DCM_ORIEN          "65"
#define MSG_ANGULAR_VEL        "66"
#define MSG_ANGULAR_ACC        "67"
#define MSG_RUNNING_TIME       "6d"
#define MSG_LOCAL_MAG_FIELD    "6e"
#define MSG_ODOMETER_STATE     "6f"
#define MSG_GEOID_HEIGHT       "72"
#define MSG_RTCM_CORRECTIONS   "73"
#define MSG_WIND               "75"
#define MSG_HEAVE              "76"
#define MSG_RAW_SATELLITE      "77"
#define MSG_GNSS_DUAL_ANT      "78"
#define MSG_GIMBAL_STATE       "7a"
#define MSG_AUTOMOTIVE         "7b"
#define MSG_PACKET_TIMER_PERIOD "7c"
#define MSG_PACKETS_PERIOD     "7d"
#define MSG_INSTALL_ALIGN      "80"
#define MSG_FILTER_OPTIONS     "81"
#define MSG_GPIO_CONFIG        "82"
#define MSG_MAG_CALI_VALUES    "83"
#define MSG_MAG_CALI_CONFIG    "84"
#define MSG_MAG_CALI_STATUS    "85"
#define MSG_ODOMETER_CONFIG    "86"
#define MSG_SET_ZERO_ORIENT_ALIGN "87"
#define MSG_REF_POINT_OFFSET   "88"
#define MSG_USER_DATA          "8a"
#define MSG_BAUD_RATES         "a0"
#define MSG_SENSOR_RANGES      "a1"
#define MSG_GPIO_OUTPUT_CONFIG "a2"
#define MSG_GPIO_INPUT_CONFIG  "a3"
#define MSG_DUAL_ANT           "a4"

//定义串口发送函数
//格式 int Serial_Send(void* buffer,int length)
#define Serial_Send(...) 0

#endif
