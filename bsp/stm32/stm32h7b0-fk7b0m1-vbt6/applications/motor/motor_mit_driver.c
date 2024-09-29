#include "motor_mit_driver.h"
#include <board.h>
#include <rtthread.h>
#include <drv_gpio.h>
#ifndef RT_USING_NANO
#include <rtdevice.h>
#endif /* RT_USING_NANO */
#include "apid.h"
#include "ulog.h"
#include "motor_cfg.h"
#include "perf_counter.h"
#include "math.h"
#include "motor_mit_driver_cfg.h"
#include "all_check.h"
#define CAN2_DEV_NAME "can2"
static rt_device_t can1_dev;        /* CAN 设备句柄 */
static rt_device_t can2_dev;        /* CAN 设备句柄 */

float Hex_To_Float(uint32_t *Byte, int num) //????????
{
    return *((float *)Byte);
}

uint32_t FloatTohex(float HEX) //??????????
{
    return *(uint32_t *)&HEX;
}
void canx_send_data(rt_device_t hcan,uint32_t id,uint8_t* data, uint8_t size)
{
		struct rt_can_msg msg;
    msg.id = 0x78;              /* ID 为 0x78 */
    msg.ide = RT_CAN_STDID;     /* 标准格式 */
    msg.rtr = RT_CAN_DTR;       /* 数据帧 */
    msg.len = 8;                /* 数据长度为 8 */

    /* 待发送的 8 字节数据 */
		int i=0;
		for(i=0;i<size;++i){
		   msg.data[i] = data[i];
		}
		
		for(;i<8;++i){
		   msg.data[i] = 0;
		}
    /* 发送一帧 CAN 数据 */
    size = rt_device_write(hcan, 0, &msg, sizeof(msg));

    if (size == 0)
    {
        rt_kprintf("can dev write data failed!\n");
    }

}

/**
************************************************************************
* @brief:      	float_to_uint: ?????????????
* @param[in]:   x_float:	???????
* @param[in]:   x_min:		?????
* @param[in]:   x_max:		?????
* @param[in]:   bits: 		??????????
* @retval:     	???????
* @details:    	??????? x ????? [x_min, x_max] ???????,?????????????????
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span   = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
/**
************************************************************************
* @brief:      	uint_to_float: ?????????????
* @param[in]:   x_int: ?????????
* @param[in]:   x_min: ?????
* @param[in]:   x_max: ?????
* @param[in]:   bits:  ????????
* @retval:     	?????
* @details:    	????????? x_int ????? [x_min, x_max] ???????,??????????
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span   = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void joint_motor_init(mit_motor_measure_t *motor, uint16_t id, uint16_t mode)
{
    motor->mode    = mode;
}

/**
************************************************************************
* @brief:      	mit_read_data: ??DM4310????????
* @param[in]:   motor:    ??motor_t?????,?????????????
* @param[in]:   rx_data:  ?????????????
* @param[in]:   data_len: ????
* @retval:     	void
* @details:    	??????????DM4310???????,????ID?
*               ????????????????????????
************************************************************************
**/
void mit_read_data(mit_motor_measure_t *motor, uint8_t *rx_data, uint32_t data_len)
{
    if (data_len == FDCAN_DLC_BYTES_8) { //??????8???
       // motor->para.id    = (rx_data[0]) & 0x0F;
        motor->para.state = (rx_data[0]) >> 4;
        motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
        motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
        motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
        motor->para.pos   = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); //
        motor->para.vel   = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); //
        motor->para.tor   = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12); //
        motor->para.Tmos  = (float)(rx_data[6]);
        motor->para.Tcoil = (float)(rx_data[7]);
    }
}

void enable_motor_mode(rt_device_t hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;

    canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	disable_motor_mode: ????????
* @param[in]:   hcan:     ??CAN_HandleTypeDef?????
* @param[in]:   motor_id: ??ID,??????
* @param[in]:   mode_id:  ??ID,????????
* @retval:     	void
* @details:    	??CAN??????????????????
************************************************************************
**/
void disable_motor_mode(rt_device_t hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFD;

    canx_send_data(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	mit_ctrl: MIT??????????
* @param[in]:   hcan:			??CAN_HandleTypeDef?????,????CAN??
* @param[in]:   motor_id:	??ID,??????
* @param[in]:   pos:			?????
* @param[in]:   vel:			?????
* @param[in]:   kp:				??????
* @param[in]:   kd:				??????
* @param[in]:   torq:			?????
* @retval:     	void
* @details:    	??CAN???????MIT????????
************************************************************************
**/
void mit_ctrl(rt_device_t hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq)
{
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint16_t id = motor_id + MIT_MODE;

    //?????????????
    pos_tmp = float_to_uint(pos, P_MIN, P_MAX, 16);  //(-12.5~12.5)
    vel_tmp = float_to_uint(vel, V_MIN, V_MAX, 12);  //(-30.0~30.0)
    kp_tmp  = float_to_uint(kp, KP_MIN, KP_MAX, 12); //(0.0~500.0)
    kd_tmp  = float_to_uint(kd, KD_MIN, KD_MAX, 12); //(0.0~5.0)
    tor_tmp = float_to_uint(torq, T_MIN, T_MAX, 12); //(-10.0~10.0)

    data[0] = (pos_tmp >> 8); // data[0]??pos_tmp???
    data[1] = pos_tmp;        // data[1]??pos_tmp???
    data[2] = (vel_tmp >> 4); // data[2]??vel_tmp11??4?
    data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;

    //??can?? ????????
    canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	pos_speed_ctrl: ????????
* @param[in]:   hcan:			??CAN_HandleTypeDef?????,????CAN??
* @param[in]:   motor_id:	??ID,??????
* @param[in]:   vel:			?????
* @retval:     	void
* @details:    	??CAN???????????????
************************************************************************
**/
void pos_speed_ctrl(rt_device_t hcan, uint16_t motor_id, float pos, float vel)
{
    uint16_t id;
    uint8_t *pbuf, *vbuf;
    uint8_t data[8];

    id   = motor_id + POS_MODE;
    pbuf = (uint8_t *)&pos;
    vbuf = (uint8_t *)&vel;

    data[0] = *pbuf;
    data[1] = *(pbuf + 1);
    data[2] = *(pbuf + 2);
    data[3] = *(pbuf + 3);

    data[4] = *vbuf;
    data[5] = *(vbuf + 1);
    data[6] = *(vbuf + 2);
    data[7] = *(vbuf + 3);

    canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	speed_ctrl: ??????
* @param[in]:   hcan: 		??CAN_HandleTypeDef?????,????CAN??
* @param[in]:   motor_id: ??ID,??????
* @param[in]:   vel: 			?????
* @retval:     	void
* @details:    	??CAN?????????????
************************************************************************
**/
void speed_ctrl(rt_device_t hcan, uint16_t motor_id, float vel)
{
    uint16_t id;
    uint8_t *vbuf;
    uint8_t data[4];

    id   = motor_id + SPEED_MODE;
    vbuf = (uint8_t *)&vel;

    data[0] = *vbuf;
    data[1] = *(vbuf + 1);
    data[2] = *(vbuf + 2);
    data[3] = *(vbuf + 3);

    canx_send_data(hcan, id, data, 4);
}




extern motor_t motor_list[MOTOR_NUM];
#define MIT_MOTOR_MOTOR_ID(index,_can_dev, __id, __can_txid,__can_rxid) [index] = {  	  \
                                                     .id = __id,         			\
                                                     .can_tx_id = __can_txid, \
                                                     .can_rx_id = __can_rxid, \
                                                     .can_dev = _can_dev,     \
}

// #define DJ_MOTOR_MOTOR_ID(index, __id, __can_id) [index] = {             \
//                                                      .can_id = __can_id, \
//                                                      .motor = &motor_list[__id],\
// }


mit_motor_measure_t mit_motors[MIT_M_NUM] = {
#ifdef MOTOR_MIT_ID1_CAN1
    MIT_MOTOR_MOTOR_ID(MIT_M_CAN1_1,&can2_dev, MMIT_1_CAN1, MIT_M_CAN1_1_TX_ID,MIT_M_CAN1_1_RX_ID),
#endif
#ifdef MOTOR_MIT_ID2_CAN1
    MIT_MOTOR_MOTOR_ID(MIT_M_CAN1_2,&can2_dev, MMIT_2_CAN1, MIT_M_CAN1_2_TX_ID, MIT_M_CAN1_2_RX_ID),
#endif
// #ifdef MOTOR_MIT_ID3_CAN1
//     MIT_MOTOR_MOTOR_ID(MIT_M_CAN1_3, MMIT_3_CAN1, CAN_Motor3_ID),
// #endif

};

















/**
 * @brief 电机控制函数，通常用于获取温度，位置等数据
 *
 * @param id
 * @param mode
 * @param data
 * @return int
 */
int motor_mit_ctr(int id, uint16_t cmd, float *data)
{
    motor_t *motor               = motor_get(id);
    mit_motor_measure_t *__motor = (mit_motor_measure_t *)motor->ops->user_data;
    //LOG_D("not used");
    switch (cmd) {

        case MOTOR_MODE_SAFETY_STOP:
            motor_stop(id);
            disable_motor_mode(*__motor->can_dev,__motor->can_tx_id,MIT_MODE);
            break;
        case MOTOR_MODE_SAFETY_START:
            motor_start(id);
            enable_motor_mode(*__motor->can_dev,__motor->can_tx_id,MIT_MODE);

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
            *data = 42.f;
            break;

        default:
            break;
    }
    return 0;
}
/**
 * @brief 电机驱动函数，通常用于控制电机
 *
 * @param id 统一的电机id
 * @param mode
 * @param value
 * @param user_data
 * @return int
 */
int motor_mit_driver(int id, uint16_t mode, float *value, void *user_data)
{
    motor_t *motor           = motor_get(id);
    struct rt_can_msg msg    = {0};
    mit_motor_measure_t *__motor = (mit_motor_measure_t *)motor->ops->user_data;
    int16_t tmpout           = *value;

    // LOG_D("id:%d mode:%d",id,mode);

    switch (mode) {
        case MOTOR_MODE_TORQUE:
        	mit_ctrl(*__motor->can_dev,__motor->can_tx_id,0,0,0,0.0f,tmpout);
            break;
        case MOTOR_MODE_SPEED:
		    mit_ctrl(*__motor->can_dev,__motor->can_tx_id,0,tmpout,0,1.0f,0);
            break;
        case MOTOR_MODE_POS:
        	mit_ctrl(*__motor->can_dev,__motor->can_tx_id,tmpout,1.0f,0,0,0);
            break;
        default:
            LOG_E("invlide motor mode");
            break;
    }
    return 0;
}

mit_motor_measure_t *mit_motor_get_by_rxcanid(int id)
{

	for(int i=0;i<MIT_M_NUM;i++){
		if(mit_motors[i].can_rx_id == id){
			return &mit_motors[i];
		}
	}
	return 0;
}
// rt_mailbox_t dj_m_mailbox;
rt_err_t ind_mit_can_motor_callback(rt_device_t dev, void *args, rt_int32_t hdr, rt_size_t size)
{
    /* CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    // rt_pin_write(GET_PIN(I, 0), 1 - rt_pin_read(GET_PIN(I, 0)));
    struct rt_can_msg rxmsg = {0};

    /* 从 CAN 读取一帧数据 */
    rt_device_read(dev, 0, &rxmsg, sizeof(rxmsg));
    /* 打印数据 ID 及内容 */
     LOG_D("ID:%x ", rxmsg.id);

    mit_motor_measure_t *motor_measure = mit_motor_get_by_rxcanid(rxmsg.id);
		if(motor_measure){
			mit_read_data(motor_measure,rxmsg.data,rxmsg.len);
			motor_measure->connect_level=0;
		}

		
    // rt_sem_release(&rx_sem);

    // int8_t tt = rt_mb_send(dj_m_mailbox, id);
    // if (tt != RT_EOK)
    // {
    //     LOG_E("dj_m_mailbox send failed %d",-tt);
    // }
    return RT_EOK;
}
static void can_rx_thread(void *parameter)
{
    static rt_uint8_t i;
    rt_err_t res;
    struct rt_can_msg msg = {0};
    // rt_pin_mode(GET_PIN(I, 0), PIN_MODE_OUTPUT);
    // rt_pin_mode(GET_PIN(I, 2), PIN_MODE_OUTPUT);
    /* 设置接收回调函数 */
    //rt_device_set_rx_indicate(can2_dev, ind_mit_can_motor_callback);
#ifdef RT_CAN_USING_HDR
    struct rt_can_filter_item items[] = {
        {.id = 0x200, .ide = 0, .rtr = 0, .mode = 0, .mask = 0x7f0, .hdr_bank = 0, .rxfifo = CAN_RX_FIFO0, .ind = ind_mit_can_motor_callback, .args = RT_NULL}};

    struct rt_can_filter_config cfg = {sizeof(items) / sizeof(struct rt_can_filter_item), 1, items}; /* 过滤表 */
    /* 设置硬件过滤表 */
    res = rt_device_control(can2_dev, RT_CAN_CMD_SET_FILTER, &cfg);
    RT_ASSERT(res == RT_EOK);
#endif
    static int timeout_counter = 0;
    while (1)
    {

		for(int i=0;i<MIT_M_NUM;i++){
			if(mit_motors[i].connect_level< MIT_DISCONNECT_LEVEL){
				motor_handle(i, 1);
			}
		}
		motor_start(0);
		//rt_thread_delay_until(rt_tick_get(),1);
		rt_thread_mdelay(1);

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


int motor_mit_init(void)
{
    struct rt_can_msg msg = {0};
    rt_err_t res;
    rt_size_t size;
    rt_thread_t thread;

    motor_init();
//    set_motor_passive_feedback();
    /* 查找 CAN 设备 */
    can2_dev = rt_device_find(CAN2_DEV_NAME);
    if (!can2_dev)
    {
        rt_kprintf("find %s failed!\n", CAN2_DEV_NAME);
        return -RT_ERROR;
    }

    // /* 初始化 CAN 接收信号量 */
    // rt_sem_init(&rx_sem, "can_m_dj_sem", 0, RT_IPC_FLAG_PRIO);
    // //    dj_m_mailbox = rt_mb_create("dj_m_rx_mailbox", 4096, RT_IPC_FLAG_PRIO);

   // dj_m_ringfifo = rt_ringbuffer_create(4096);
    /* 以中断接收及中断发送方式打开 CAN 设备 */
    res = rt_device_open(can2_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    RT_ASSERT(res == RT_EOK);
    /* 设置 CAN 通信的波特率为 1Mbit/s*/
    res = rt_device_control(can2_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN1MBaud);

    RT_ASSERT(res == RT_EOK);
//    res = rt_device_control(can2_dev, RT_CAN_CMD_SET_PRIV, (void *)1);
//    RT_ASSERT(res == RT_EOK);


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

//    thread = rt_thread_create("m_test", can_rx_thread1, RT_NULL, 4096 * 2, 6, 10);
//    if (thread != RT_NULL)
//    {
//        rt_thread_startup(thread);
//    }
//    else
//    {
//        rt_kprintf("create can_rx thread failed!\n");
//    }
    return 0;
}
INIT_COMPONENT_EXPORT(motor_mit_init);

