#ifndef __MOTOR_MIT_DRIVER_H__
#define __MOTOR_MIT_DRIVER_H__
#include "main.h"
#include "motor.h"

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPEED_MODE		0x200

//???DM4310???,???????????????
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f



/**
 * @brief mit 电机内部驱动id
 * 
 */
enum{
    #if defined(MOTOR_MIT_ID1_CAN1)  
        MIT_M_CAN1_1,
    #endif
    #if defined(MOTOR_MIT_ID2_CAN1)
        MIT_M_CAN1_2,
    #endif
    #if defined(MOTOR_MIT_ID3_CAN1)
        MIT_M_CAN1_3,
    #endif
   
    MIT_M_NUM

};
/**
 * @brief 定义每个电机对应的can id
 * 
 */
enum{
	MIT_M_CAN1_1_TX_ID = 0x01,
	MIT_M_CAN1_2_TX_ID = 0x02,
	MIT_M_CAN1_3_TX_ID = 0x03,
};

enum{
	MIT_M_CAN1_1_RX_ID = 0x11,
	MIT_M_CAN1_2_RX_ID = 0x12,
	MIT_M_CAN1_3_RX_ID = 0x13,
};


typedef struct 
{
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;

	float pos;
	float vel;
	float tor;

	float Tmos;
	float Tcoil;
}motor_fbpara_t;


typedef struct
{
    uint8_t id;           //对于motor抽象层的id
    uint16_t can_tx_id;      //CAN设备控制的 id
    uint16_t can_rx_id;      //CAN设备回传的 id
	rt_device_t* can_dev;
	uint16_t mode;
	uint8_t connect_level;
	motor_fbpara_t para;
}mit_motor_measure_t ;

extern mit_motor_measure_t mit_motors[MIT_M_NUM] ;


//extern void dm4310_fbdata(mit_motor_measure_t *motor, uint8_t *rx_data,uint32_t data_len);


//extern void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
//extern void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

////????
//extern void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
//extern void pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel);
//extern void speed_ctrl(hcan_t* hcan,uint16_t motor_id, float _vel);

/**
 * @brief 电机控制函数，通常用于获取温度，位置等数据
 * 
 * @param id 
 * @param mode 
 * @param data 
 * @return int 
 */
int motor_mit_ctr(int id,uint16_t mode,float*data);
/**
 * @brief 电机驱动函数，通常用于控制电机
 * 
 * @param id 统一的电机id
 * @param mode 
 * @param value 
 * @param user_data 
 * @return int 
 */
int motor_mit_driver (int id, uint16_t mode,float* value, void* user_data);


extern void joint_motor_init(mit_motor_measure_t *motor,uint16_t id,uint16_t mode);

	
extern float Hex_To_Float(uint32_t *Byte,int num);//????????
extern uint32_t FloatTohex(float HEX);//??????????

extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int float_to_uint(float x_float, float x_min, float x_max, int bits);

#endif /* __MOTOR_MIT_DRIVER_H__ */

