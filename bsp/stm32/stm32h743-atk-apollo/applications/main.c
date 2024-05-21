/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-03-05     whj4674672   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <lcd_st7789.h>

/* defined the LED0 pin: PC13 */
#define LED0_PIN    GET_PIN(C, 13)

int main(void)
{
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
	   //
 spi_lcd_init(200);

		LCD_Clear(0xffff);
//	rt_thread_mdelay(500);
//		LCD_Clear(0xf000);
//	rt_thread_mdelay(500);
//		LCD_Clear(0xffff);
//	rt_thread_mdelay(500);
//		LCD_Clear(0xf000);
//	rt_thread_mdelay(500);
//			LCD_Clear(0xffff);
//	rt_thread_mdelay(500);
//		LCD_Clear(0xf000);
//	rt_thread_mdelay(500);
	LCD_Fill(0,0,240,320,0xff00);
    while (1)
    {
//					LCD_Clear(0xffff);
//				LCD_Fill(0,0,240,320,0xff00);
//rt_thread_mdelay(1000);
			
			

        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);

        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
			
    }
}
