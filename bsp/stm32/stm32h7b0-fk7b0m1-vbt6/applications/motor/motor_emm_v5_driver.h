/*
 * @Author: Dyyt587 67887002+Dyyt587@users.noreply.github.com
 * @Date: 2024-06-19 17:37:20
 * @LastEditors: Dyyt587 67887002+Dyyt587@users.noreply.github.com
 * @LastEditTime: 2024-06-19 18:06:59
 * @FilePath: \project\applications\motor_emm_v5_driver.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef MOTOR_EMM_V5_DRIVER_H
#define MOTOR_EMM_V5_DRIVER_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdbool.h>

#include "motor_def.h"

	typedef enum
	{
		S_VER = 0,	  /* 读取固件版本和对应的硬件版本 */
		S_RL = 1,	  /* 读取读取相电阻和相电感 */
		S_PID = 2,	  /* 读取PID参数 */
		S_VBUS = 3,	  /* 读取总线电压 */
		S_CPHA = 5,	  /* 读取相电流 */
		S_ENCL = 7,	  /* 读取经过线性化校准后的编码器值 */
		S_TPOS = 8,	  /* 读取电机目标位置角度 */
		S_VEL = 9,	  /* 读取电机实时转速 */
		S_CPOS = 10,  /* 读取电机实时位置角度 */
		S_PERR = 11,  /* 读取电机位置误差角度 */
		S_FLAG = 13,  /* 读取使能/到位/堵转状态标志位 */
		S_Conf = 14,  /* 读取驱动参数 */
		S_State = 15, /* 读取系统状态参数 */
		S_ORG = 16,	  /* 读取正在回零/回零失败状态标志位 */
	} SysParams_t;

	typedef struct stepper_motor
	{
		uint8_t stepper_motor_id;									/* 电机ID */
		rt_device_t* serial;
		motor_t* motor; /* 电机结构体指针 */


		uint8_t stepper_motor_target_dir;							/* 电机旋转方向 */
		uint32_t stepper_motor_target_angle; /* 电机目标位置角度 */ // 3200个脉冲一圈
		uint16_t stepper_motor_target_speed;						/* 电机目标转速 */

		uint16_t stepper_motor_speed; /* 电机实时转速 */
		float stepper_motor_angle;	  /* 电机实时位置角度 */
		float stepper_motor_err;	  /* 电机位置误差角度 */

		uint8_t stepper_motor_reachflag; /* 到位状态标志位 */
		uint8_t stepper_motor_stallflag; /* 堵转状态标志位 */
		uint8_t stepper_motor_enflag;	 /* 使能状态标志位 */

		uint8_t stepper_motor_encokflag;		   /* 编码器就绪标志位 */
		uint8_t stepper_motor_calibrationflag;	   /* 校准就绪标志位 */
		uint8_t stepper_motor_returnzeroingflag;   /* 正在回零状态标志位 */
		uint8_t stepper_motor_returnzero_failflag; /* 回零失败状态标志位 */

		uint16_t stepper_motor_current; /* 读取相电流 */
	} stepper_motor_t;

#define STEPPER_MOTOR_NUM 11
extern stepper_motor_t stepper_motor[STEPPER_MOTOR_NUM];

	/**********************************************************
	*** 注意：每个函数的参数的具体说明，请查阅下方的函数的注释说明
	**********************************************************/
	void Emm_V5_Reset_CurPos_To_Zero(stepper_motor_t *motor); // 将当前位置清零
	void Emm_V5_Reset_Clog_Pro(stepper_motor_t *motor);		  // 解除堵转保护

	void Emm_V5_Read_Sys_Params(stepper_motor_t *motor, SysParams_t s); // 读取参数

	void Emm_V5_Modify_Ctrl_Mode(stepper_motor_t *motor, bool svF, uint8_t ctrl_mode);				   // 发送命令修改开环/闭环控制模式
	void Emm_V5_En_Control(stepper_motor_t *motor, bool state, bool snF);							   // 电机使能控制
	void Emm_V5_Vel_Control(stepper_motor_t *motor, uint8_t dir, uint16_t vel, uint8_t acc, bool snF); // 速度模式控制

	void Emm_V5_Pos_Control(stepper_motor_t *motor, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF); // 位置模式控制

	void Emm_V5_Stop_Now(stepper_motor_t *motor, bool snF);																																		  // 让电机立即停止运动
	void Emm_V5_Synchronous_motion(stepper_motor_t *motor);																																		  // 触发多机同步开始运动
	void Emm_V5_Origin_Set_O(stepper_motor_t *motor, bool svF);																																	  // 设置单圈回零的零点位置
	void Emm_V5_Origin_Modify_Params(stepper_motor_t *motor, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF); // 修改回零参数
	void Emm_V5_Origin_Trigger_Return(stepper_motor_t *motor, uint8_t o_mode, bool snF);																										  // 发送命令触发回零
	void Emm_V5_Origin_Interrupt(stepper_motor_t *motor);																																		  // 强制中断并退出回零

	void Emm_V5_Pos_moveok(void);
#ifdef __cplusplus
}
#endif
#endif
