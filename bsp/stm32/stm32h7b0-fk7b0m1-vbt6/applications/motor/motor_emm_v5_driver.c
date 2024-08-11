#include <rtthread.h>
#include <rtdevice.h>
#include <rtdbg.h>
#define DBG_TAG "drv.step_motor"
#define DBG_LVL DBG_LOG
#include "motor.h"
#include "motor_emm_v5_driver.h"
#include "motor_emm_v5_driver_cfg.h"
#include "math.h"
#include "rtconfig.h"
#include "board.h"

#ifndef ABS
#define ABS(x) ((x) > 0 ? (x) : -(x))
#endif
    #define EMM_SERIAL  "uart8"

rt_mutex_t mutex_step = RT_NULL; // 互斥锁
rt_sem_t emm_rx_sem = RT_NULL;
int Emm_rx_size = 0;
uint8_t Emm_rx_buf[64] = {0};

rt_device_t Emm_serial1 = RT_NULL;
stepper_motor_t motor1;
stepper_motor_t motor2;

#define STEPPER_INIT(index, _serial, _id) [index] = {                  \
                                              .serial = &_serial,      \
                                              .stepper_motor_id = _id, \
}

stepper_motor_t stepper_motor[STEPPER_MOTOR_NUM] =
    {
#ifdef MOTOR_EMMV5_ID1_SERIAL1
        STEPPER_INIT(SP_M_ID1_SERIAL1, Emm_serial1, 1),
#endif
#ifdef MOTOR_EMMV5_ID2_SERIAL1
        STEPPER_INIT(SP_M_ID2_SERIAL1, Emm_serial1, 2),
#endif
#ifdef MOTOR_EMMV5_ID3_SERIAL1
        STEPPER_INIT(SP_M_ID3_SERIAL1, Emm_serial1, 3),
#endif
#ifdef MOTOR_EMMV5_ID4_SERIAL1
        STEPPER_INIT(SP_M_ID4_SERIAL1, Emm_serial1, 4),
#endif
#ifdef MOTOR_EMMV5_ID5_SERIAL1
        STEPPER_INIT(SP_M_ID5_SERIAL1, Emm_serial1, 5),
#endif
#ifdef MOTOR_EMMV5_ID6_SERIAL1
        STEPPER_INIT(SP_M_ID6_SERIAL1, Emm_serial1, 6),
#endif

};

int motor_emm_v5_driver(int id, uint16_t mode, float *value, void *user_data)
{
    motor_t *motor = motor_get(id);
    struct rt_can_msg msg = {0};
    stepper_motor_t *__motor = (stepper_motor_t *)motor->ops->user_data;
    int16_t tmpout = *value;
    int time = 0;
    LOG_D("id:%d mode:%d", id, mode);
    int acc =100;
    int max_v =1000;
    switch (mode)
    {
    case MOTOR_MODE_TORQUE:
        break;
    case MOTOR_MODE_SPEED:
        Emm_V5_Vel_Control( __motor, 1,tmpout, acc, false);
        break;
    case MOTOR_MODE_POS:
        Emm_V5_Pos_Control(__motor, 1, max_v, acc, tmpout, true, false);
        break;
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
int motor_emm_v5_ctr(int id, uint16_t cmd, float *arg)
{
    motor_t *motor = motor_get(id);
    stepper_motor_t *__motor = (stepper_motor_t *)motor->ops->user_data;
    LOG_D("not used");
    switch (cmd)
    {

    case MOTOR_MODE_SAFETY_STOP:
        Emm_V5_En_Control(__motor, false, false);
        break;
    case MOTOR_MODE_SAFETY_START:
        Emm_V5_En_Control(__motor, true, false);
        break;
    case MOTOR_MODE_TORQUE:
        /*返回力矩/电流值*/
        *arg = __motor->stepper_motor_current;
        break;
    case MOTOR_MODE_SPEED:
        // /*返回速度值r/min rpm*/
        *arg = __motor->stepper_motor_speed;
        break;
    case MOTOR_MODE_POS:
        // /*返回位置 rad*/
        *arg = (float)__motor->stepper_motor_angle;
        break;
    case MOTOR_MODE_TEMP:
        // /*返回温度*/
        *arg = 0.f;
        break;
    case MOTOR_MODE_ACC: /* 加速度 */

        break;
    case MOTOR_MODE_DAC: /* 减速度 */

        break;
    case MOTOR_MODE_MAX_V: /* 最大速度，通常用于位置模式限制速度 */

        break;
    default:
        break;
    }
    return 0;
}

int motor_acc = 225;
int motor_vel = 50;
// #define motor_acc 0xff
// #define motor_vel 150
// v=0.1 m/s
int gap_time = 10;

void emm_transmit(uint8_t *data, uint8_t len)
{
    rt_device_write(Emm_serial1, 0, data, len);
}

int emm_get_cmd_receive_size(uint8_t cmd)
{
    switch (cmd)
    {
    case 0x35: // v
        return 6;
    case 0x36: // p
        return 8;
    default:
        return 4;
    }
}

void emm_cmd_ana(stepper_motor_t *motor, uint8_t cmd, uint8_t *buf)
{
    // LOG_D("cmd:%d",cmd);
    switch (cmd)
    {
    case 0x35: // v
        if (Emm_rx_buf[1] == 0x35)
        {

            // 拼接成uint16_t类型数据
            uint16_t vel = (uint16_t)(((uint16_t)Emm_rx_buf[3] << 8) |
                                      ((uint16_t)Emm_rx_buf[4] << 0));
            motor->stepper_motor_speed = vel;

            // 符号
            if (Emm_rx_buf[2])
            {
                motor->stepper_motor_speed = -motor->stepper_motor_speed;
            }
            //     LOG_D("emm_cmd_ana v:%d", motor->stepper_motor_speed);
            return;
        }
        else
            return;
    case 0x36: // p
        if (Emm_rx_buf[1] == 0x36)
        {
            // 获取绝对值信息
            float pos = (uint32_t)(((uint32_t)Emm_rx_buf[3] << 24) |
                                   ((uint32_t)Emm_rx_buf[4] << 16) |
                                   ((uint32_t)Emm_rx_buf[5] << 8) |
                                   ((uint32_t)Emm_rx_buf[6] << 0));
            // 转换为角度
            motor->stepper_motor_angle = (float)pos * 360.0f / 65536.0f;

            // 修正电机正反位置
            if (Emm_rx_buf[2])
            {
                motor->stepper_motor_angle = -motor->stepper_motor_angle;
            }
            //   LOG_I("stepperMotor_%d_Cur_Pos: %f\n",motor->stepper_motor_id , motor->stepper_motor_angle);
            return;
        }
        else
            return;
    default:
        if (Emm_rx_buf[3] != 0x6B)
            for (int i = 0; i < 4; i++)
            {
                LOG_D("Emm_rx_buf[%d]:%x", i, Emm_rx_buf[i]);
            }
        //  LOG_W("emm_cmd answer nuknown");
        return;
    }
}
void emm_wait_for_ack(stepper_motor_t *motor, uint8_t cmd)
{
    uint8_t will_rx_size = emm_get_cmd_receive_size(cmd);
    // LOG_D("will_rx_size:%d",will_rx_size);
    uint8_t Emm_rx_index = 0;
    uint8_t ch = 0;
    while (1)
    {
        while (rt_device_read(Emm_serial1, -1, &ch, 1) != 1)
        {
            rt_err_t ret = rt_sem_take(emm_rx_sem, 100);
            if (ret != RT_EOK)
            {
                LOG_E("emm_wait_for_ack timeout");
                rt_mutex_release(mutex_step);
                return;
            }
            else
            {
            }
        }
        // 获取到字符
        Emm_rx_buf[Emm_rx_index++] = ch;
        if (Emm_rx_index >= will_rx_size)
        {
            emm_cmd_ana(motor, cmd, Emm_rx_buf);
            rt_mutex_release(mutex_step);
            return;
        }
    }

    rt_mutex_release(mutex_step);
}
/**
 * @brief    将当前位置清零
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Reset_CurPos_To_Zero(stepper_motor_t *motor)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[0] = motor->stepper_motor_id; // 地址
    cmd[1] = 0x0A;                    // 功能码
    cmd[2] = 0x6D;                    // 辅助码
    cmd[3] = 0x6B;                    // 校验字节

    // 发送命令
    emm_transmit(cmd, 4);
    emm_wait_for_ack(motor, cmd[1]);
}

/**
 * @brief    解除堵转保护
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Reset_Clog_Pro(stepper_motor_t *motor)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[0] = motor->stepper_motor_id; // 地址
    cmd[1] = 0x0E;                    // 功能码
    cmd[2] = 0x52;                    // 辅助码
    cmd[3] = 0x6B;                    // 校验字节

    // 发送命令
    emm_transmit(cmd, 4);
    emm_wait_for_ack(motor, cmd[1]);
}

/**
 * @brief    读取系统参数
 * @param    addr  ：电机地址
 * @param    s     ：系统参数类型
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Read_Sys_Params(stepper_motor_t *motor, SysParams_t s)
{
    uint8_t i = 0;
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[i] = motor->stepper_motor_id;
    ++i; // 地址

    switch (s) // 功能码
    {
    case S_VER:
        cmd[i] = 0x1F;
        ++i;
        break;
    case S_RL:
        cmd[i] = 0x20;
        ++i;
        break;
    case S_PID:
        cmd[i] = 0x21;
        ++i;
        break;
    case S_VBUS:
        cmd[i] = 0x24;
        ++i;
        break;
    case S_CPHA:
        cmd[i] = 0x27;
        ++i;
        break;
    case S_ENCL:
        cmd[i] = 0x31;
        ++i;
        break;
    case S_TPOS:
        cmd[i] = 0x33;
        ++i;
        break;
    case S_VEL:
        cmd[i] = 0x35;
        ++i;
        break;
    case S_CPOS:
        cmd[i] = 0x36;
        ++i;
        break;
    case S_PERR:
        cmd[i] = 0x37;
        ++i;
        break;
    case S_FLAG:
        cmd[i] = 0x3A;
        ++i;
        break;
    case S_ORG:
        cmd[i] = 0x3B;
        ++i;
        break;
    case S_Conf:
        cmd[i] = 0x42;
        ++i;
        cmd[i] = 0x6C;
        ++i;
        break;
    case S_State:
        cmd[i] = 0x43;
        ++i;
        cmd[i] = 0x7A;
        ++i;
        break;
    default:
        break;
    }

    cmd[i] = 0x6B;
    ++i; // 校验字节

    // 发送命令
    emm_transmit(cmd, i);
    emm_wait_for_ack(motor, cmd[1]);
}

/**
 * @brief    修改开环/闭环控制模式
 * @param    addr     ：电机地址
 * @param    svF      ：是否存储标志，false为不存储，true为存储
 * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Modify_Ctrl_Mode(stepper_motor_t *motor, bool svF, uint8_t ctrl_mode)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[0] = motor->stepper_motor_id; // 地址
    cmd[1] = 0x46;                    // 功能码
    cmd[2] = 0x69;                    // 辅助码
    cmd[3] = svF;                     // 是否存储标志，false为不存储，true为存储
    cmd[4] = ctrl_mode;               // 控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
    cmd[5] = 0x6B;                    // 校验字节

    // 发送命令
    emm_transmit(cmd, 6);
    emm_wait_for_ack(motor, cmd[1]);
}

/**
 * @brief    使能信号控制
 * @param    addr  ：电机地址
 * @param    state ：使能状态     ，true为使能电机，false为关闭电机
 * @param    snF   ：多机同步标志 ，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_En_Control(stepper_motor_t *motor, bool state, bool snF)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[0] = motor->stepper_motor_id; // 地址
    cmd[1] = 0xF3;                    // 功能码
    cmd[2] = 0xAB;                    // 辅助码
    cmd[3] = (uint8_t)state;          // 使能状态
    cmd[4] = snF;                     // 多机同步运动标志
    cmd[5] = 0x6B;                    // 校验字节

    // 发送命令
    emm_transmit(cmd, 6);
    emm_wait_for_ack(motor, cmd[1]);
}

/**
 * @brief    速度模式
 * @param    addr：电机地址
 * @param    dir ：方向       ，0为CW，其余值为CCW
 * @param    vel ：速度       ，范围0 - 5000RPM
 * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
 * @param    snF ：多机同步标志，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Vel_Control(stepper_motor_t *motor, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[0] = motor->stepper_motor_id; // 地址
    cmd[1] = 0xF6;                    // 功能码
    cmd[2] = dir;                     // 方向
    cmd[3] = (uint8_t)(vel >> 8);     // 速度(RPM)高8位字节
    cmd[4] = (uint8_t)(vel >> 0);     // 速度(RPM)低8位字节
    cmd[5] = acc;                     // 加速度，注意：0是直接启动
    cmd[6] = snF;                     // 多机同步运动标志
    cmd[7] = 0x6B;                    // 校验字节

    // 发送命令
    emm_transmit(cmd, 8);
    emm_wait_for_ack(motor, cmd[1]);
}

/**
 * @brief    位置模式
 * @param    addr：电机地址
 * @param    dir ：方向        ，0为CW，其余值为CCW
 * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
 * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
 * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
 * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
 * @param    snF ：多机同步标志 ，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Pos_Control(stepper_motor_t *motor, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[0] = motor->stepper_motor_id; // 地址
    cmd[1] = 0xFD;                    // 功能码
    cmd[2] = dir;                     // 方向
    cmd[3] = (uint8_t)(vel >> 8);     // 速度(RPM)高8位字节
    cmd[4] = (uint8_t)(vel >> 0);     // 速度(RPM)低8位字节
    cmd[5] = acc;                     // 加速度，注意：0是直接启动
    cmd[6] = (uint8_t)(clk >> 24);    // 脉冲数(bit24 - bit31)
    cmd[7] = (uint8_t)(clk >> 16);    // 脉冲数(bit16 - bit23)
    cmd[8] = (uint8_t)(clk >> 8);     // 脉冲数(bit8  - bit15)
    cmd[9] = (uint8_t)(clk >> 0);     // 脉冲数(bit0  - bit7 )
    cmd[10] = raF;                    // 相位/绝对标志，false为相对运动，true为绝对值运动
    cmd[11] = snF;                    // 多机同步运动标志，false为不启用，true为启用
    cmd[12] = 0x6B;                   // 校验字节

    // 发送命令
    emm_transmit(cmd, 13);
    emm_wait_for_ack(motor, cmd[1]);
}

/**
 * @brief    立即停止（所有控制模式都通用）
 * @param    addr  ：电机地址
 * @param    snF   ：多机同步标志，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Stop_Now(stepper_motor_t *motor, bool snF)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[0] = motor->stepper_motor_id; // 地址
    cmd[1] = 0xFE;                    // 功能码
    cmd[2] = 0x98;                    // 辅助码
    cmd[3] = snF;                     // 多机同步运动标志
    cmd[4] = 0x6B;                    // 校验字节

    // 发送命令
    emm_transmit(cmd, 5);
    emm_wait_for_ack(motor, cmd[1]);
}

/**
 * @brief    多机同步运动
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Synchronous_motion(stepper_motor_t *motor)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[0] = motor->stepper_motor_id; // 地址
    cmd[1] = 0xFF;                    // 功能码
    cmd[2] = 0x66;                    // 辅助码
    cmd[3] = 0x6B;                    // 校验字节

    // 发送命令
    emm_transmit(cmd, 4);
    emm_wait_for_ack(motor, cmd[1]);
}

/**
 * @brief    设置单圈回零的零点位置
 * @param    addr  ：电机地址
 * @param    svF   ：是否存储标志，false为不存储，true为存储
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Origin_Set_O(stepper_motor_t *motor, bool svF)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[0] = motor->stepper_motor_id; // 地址
    cmd[1] = 0x93;                    // 功能码
    cmd[2] = 0x88;                    // 辅助码
    cmd[3] = svF;                     // 是否存储标志，false为不存储，true为存储
    cmd[4] = 0x6B;                    // 校验字节

    // 发送命令
    emm_transmit(cmd, 5);
    emm_wait_for_ack(motor, cmd[1]);
}

/**
 * @brief    修改回零参数
 * @param    addr  ：电机地址
 * @param    svF   ：是否存储标志，false为不存储，true为存储
 * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
 * @param    o_dir  ：回零方向，0为CW，其余值为CCW
 * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
 * @param    o_tm   ：回零超时时间，单位：毫秒
 * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
 * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
 * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
 * @param    potF   ：上电自动触发回零，false为不使能，true为使能
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Origin_Modify_Params(stepper_motor_t *motor, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
    uint8_t cmd[32] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[0] = motor->stepper_motor_id; // 地址
    cmd[1] = 0x4C;                    // 功能码
    cmd[2] = 0xAE;                    // 辅助码
    cmd[3] = svF;                     // 是否存储标志，false为不存储，true为存储
    cmd[4] = o_mode;                  // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
    cmd[5] = o_dir;                   // 回零方向
    cmd[6] = (uint8_t)(o_vel >> 8);   // 回零速度(RPM)高8位字节
    cmd[7] = (uint8_t)(o_vel >> 0);   // 回零速度(RPM)低8位字节
    cmd[8] = (uint8_t)(o_tm >> 24);   // 回零超时时间(bit24 - bit31)
    cmd[9] = (uint8_t)(o_tm >> 16);   // 回零超时时间(bit16 - bit23)
    cmd[10] = (uint8_t)(o_tm >> 8);   // 回零超时时间(bit8  - bit15)
    cmd[11] = (uint8_t)(o_tm >> 0);   // 回零超时时间(bit0  - bit7 )
    cmd[12] = (uint8_t)(sl_vel >> 8); // 无限位碰撞回零检测转速(RPM)高8位字节
    cmd[13] = (uint8_t)(sl_vel >> 0); // 无限位碰撞回零检测转速(RPM)低8位字节
    cmd[14] = (uint8_t)(sl_ma >> 8);  // 无限位碰撞回零检测电流(Ma)高8位字节
    cmd[15] = (uint8_t)(sl_ma >> 0);  // 无限位碰撞回零检测电流(Ma)低8位字节
    cmd[16] = (uint8_t)(sl_ms >> 8);  // 无限位碰撞回零检测时间(Ms)高8位字节
    cmd[17] = (uint8_t)(sl_ms >> 0);  // 无限位碰撞回零检测时间(Ms)低8位字节
    cmd[18] = potF;                   // 上电自动触发回零，false为不使能，true为使能
    cmd[19] = 0x6B;                   // 校验字节

    // 发送命令
    emm_transmit(cmd, 20);
    emm_wait_for_ack(motor, cmd[1]);
}

/**
 * @brief    触发回零
 * @param    addr   ：电机地址
 * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
 * @param    snF   ：多机同步标志，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Origin_Trigger_Return(stepper_motor_t *motor, uint8_t o_mode, bool snF)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[0] = motor->stepper_motor_id; // 地址
    cmd[1] = 0x9A;                    // 功能码
    cmd[2] = o_mode;                  // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
    cmd[3] = snF;                     // 多机同步运动标志，false为不启用，true为启用
    cmd[4] = 0x6B;                    // 校验字节

    // 发送命令
    emm_transmit(cmd, 5);
    emm_wait_for_ack(motor, cmd[1]);
}

/**
 * @brief    强制中断并退出回零
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Origin_Interrupt(stepper_motor_t *motor)
{
    uint8_t cmd[16] = {0};
    rt_mutex_take(mutex_step, RT_WAITING_FOREVER);

    // 装载命令
    cmd[0] = motor->stepper_motor_id; // 地址
    cmd[1] = 0x9C;                    // 功能码
    cmd[2] = 0x48;                    // 辅助码
    cmd[3] = 0x6B;                    // 校验字节

    // 发送命令
    emm_transmit(cmd, 4);
    emm_wait_for_ack(motor, cmd[1]);
}
rt_err_t emm_uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    Emm_rx_size = size;
    rt_sem_release(emm_rx_sem);
    return RT_EOK;
}

void drv_emm_v5_entry(void *t)
{
    // rt_thread_mdelay(2000); //等待步进上电
    /* 查找系统中的串口设备 */
    Emm_serial1 = rt_device_find(EMM_SERIAL);
    if (Emm_serial1 == RT_NULL)
        return;
    char ch = 0;
    /* 初始化信号量 */
    emm_rx_sem = rt_sem_create("emm_rx_sem", 0, RT_IPC_FLAG_FIFO);
    /* 以阻塞接收及轮询发送模式打开串口设备 */
    rt_device_open(Emm_serial1, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
		struct serial_configure config = {
		BAUD_RATE_115200,						/* 115200 bits/s */
		DATA_BITS_8,				/* 8 databits */
		STOP_BITS_1,				/* 1 stopbit */
		PARITY_NONE,				/* No parity  */
		BIT_ORDER_LSB,				/* LSB first sent */
		NRZ_NORMAL,					/* Normal mode */
		BSP_UART7_RX_BUFSIZE,		/* rxBuf size */
		BSP_UART7_TX_BUFSIZE,		/* txBuf size */
		RT_SERIAL_FLOWCONTROL_NONE, /* Off flowcontrol */
		0};
	if (RT_EOK != rt_device_control(Emm_serial1, RT_DEVICE_CTRL_CONFIG, &config))
	{
		rt_kprintf("change %s(emmv5) failed!\n", Emm_serial1->parent.name);
	}
    if (rt_device_set_rx_indicate(Emm_serial1, emm_uart_rx_ind) != RT_EOK)
    {
        LOG_E("uart1 set rx indicate failed");
        return;
    }

#define Origin_vel 180
#define timeout 30000
// 这里的堵转速度直接给最大值 就不用管这个参数了
#define sl_vel 500
// 用大电流 小时间来60
#define sl_ma 200
#define sl_ms 10
    // 回零调参 电机有存储的 没必要每次都重新设置
    /*
     * @param    addr  ：电机地址
     * @param    svF   ：是否存储标志，false为不存储，true为存储
     * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
     * @param    o_dir  ：回零方向，0为CW，其余值为CCW
     * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
     * @param    o_tm   ：回零超时时间，单位：毫秒
     * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
     * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
     * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
     * @param    potF   ：上电自动触发回零，false为不使能，true为使能
     * */

    // Emm_V5_Origin_Modify_Params(&left_stepper, 1, 2, 0, Origin_vel, timeout, sl_vel, sl_ma, sl_ms, 0);//参数4是方向   倒数第三个参数是电流值   这是控制大臂的电机（螺丝很长的一边）  方向1 是控制往上抬
    // Emm_V5_Origin_Modify_Params(&right_stepper, 1, 2, 0, Origin_vel, timeout, sl_vel, sl_ma, sl_ms, 0);//参数4是方向 倒数第三个参数是电流值    这是控制小臂的电机（多一个件的一边）  第四个参数 方向0 是控制往上抬

    // rt_thread_mdelay(100); //设置参数之后需要延时！！！！！！！！！延时等待闭环步进参数设置完成（写入flash）
motor1.stepper_motor_id=1;
motor2.stepper_motor_id=2;

motor1.serial = &Emm_serial1;
motor2.serial = &Emm_serial1;
    Emm_V5_En_Control(&motor1, 1, 0);//一个一个回零
    Emm_V5_En_Control(&motor2, 1, 0);//一个一个回零
		
		

    // Emm_V5_Origin_Trigger_Return(&left_stepper, 2, 0);
    // rt_thread_mdelay(5000);

    // Emm_V5_En_Control(&left_stepper, 0, 0);//一个一个回零
    // Emm_V5_Origin_Trigger_Return(&right_stepper, 2, 0);
    // rt_thread_mdelay(5000);

    // Emm_V5_Reset_CurPos_To_Zero(&left_stepper);
    // Emm_V5_Reset_CurPos_To_Zero(&right_stepper);

    // Emm_V5_En_Control(&left_stepper, 1, 0);
    // Emm_V5_En_Control(&right_stepper, 1, 0);
    // corexy.x=0;
    // corexy.y=0;
//    Emm_V5_En_Control(&motor1, 0, 0);//一个一个回零
//    Emm_V5_En_Control(&motor2, 0, 0);//一个一个回零
//		
    while (1)
    {
//		Emm_V5_Vel_Control(&motor1,0,500,100,0);
//		Emm_V5_Vel_Control(&motor2,0,100,100,0);
//			
			
			        					Emm_V5_Pos_Control(&motor1, 1, motor_vel, motor_acc, 3200*3*2, 1, 0);

			        					Emm_V5_Pos_Control(&motor2, 0, motor_vel, motor_acc, 3200*3*2, 1, 0);

        //			if(left_stepper_pulse>=0) //对电机来说 默认逆时针为正
        //			{
        //					Emm_V5_Pos_Control(&left_stepper, 1, motor_vel, motor_acc, left_stepper_pulse, 1, 0);
        //			}
        //			else
        //			{
        //					Emm_V5_Pos_Control(&left_stepper, 0, motor_vel, motor_acc, -left_stepper_pulse, 1, 0);
        //			}
        //			if(right_stepper_pulse>=0)
        //			{
        //					Emm_V5_Pos_Control(&right_stepper, 1, motor_vel, motor_acc, right_stepper_pulse, 1, 0);
        //			}
        //			else
        //			{
        //					Emm_V5_Pos_Control(&right_stepper, 0, motor_vel, motor_acc, -right_stepper_pulse, 1, 0);
        //			}

        static int time = 0;

//                Emm_V5_Read_Sys_Params(&motor1, S_CPOS);
//                Emm_V5_Read_Sys_Params(&motor2, S_CPOS);
//								LOG_D("dw%f",motor1.stepper_motor_angle);
        rt_thread_mdelay(10); // 150有点震
    }
}

int emm_v5_init(void)
{

    /*互斥锁*/
    mutex_step = rt_mutex_create("mutex_emm", RT_IPC_FLAG_PRIO);
    if (mutex_step == RT_NULL)
        return -RT_ERROR;
    rt_thread_t drv_emm_v5_tb = RT_NULL;

    drv_emm_v5_tb = rt_thread_create("drv_emm_v5",
                                     drv_emm_v5_entry, RT_NULL,
                                     8192,
                                     15, 1);

    if (drv_emm_v5_tb != RT_NULL)
    {
        rt_thread_startup(drv_emm_v5_tb);
    }
    return 0;
}
//INIT_COMPONENT_EXPORT(emm_v5_init);