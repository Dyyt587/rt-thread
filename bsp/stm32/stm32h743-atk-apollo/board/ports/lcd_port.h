/*
 * @Author: Dyyt587 67887002+Dyyt587@users.noreply.github.com
 * @Date: 2024-02-03 00:00:00
 * @LastEditors: Dyyt587 67887002+Dyyt587@users.noreply.github.com
 * @LastEditTime: 2024-05-20 01:36:44
 * @FilePath: \ports\lcd_port.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-01-08     zylx         first version
 */

#ifndef __LCD_PORT_H__
#define __LCD_PORT_H__

/* atk 4.3 inch screen, 480 * 272 */
#define LCD_WIDTH           480
#define LCD_HEIGHT          272
#define LCD_BITS_PER_PIXEL  16
#define LCD_BUF_SIZE        (LCD_WIDTH * LCD_HEIGHT * LCD_BITS_PER_PIXEL / 8)
#define LCD_PIXEL_FORMAT    RTGRAPHIC_PIXEL_FORMAT_RGB565

#define LCD_HSYNC_WIDTH     1
#define LCD_VSYNC_HEIGHT    1
#define LCD_HBP             40
#define LCD_VBP             8
#define LCD_HFP             5
#define LCD_VFP             8

#define LCD_BACKLIGHT_USING_GPIO
#define LCD_BL_GPIO_NUM     GET_PIN(B, 5)
#define LCD_DISP_GPIO_NUM   GET_PIN(B, 0)
/* atk 4.3 inch screen, 480 * 272 */

#endif /* __LCD_PORT_H__ */
