/***********************************************************************************************************************
 * Copyright [2020-2023] Renesas Electronics Corporation and/or its affiliates.  All Rights Reserved.
 *
 * This software and documentation are supplied by Renesas Electronics America Inc. and may only be used with products
 * of Renesas Electronics Corp. and its affiliates ("Renesas").  No other uses are authorized.  Renesas products are
 * sold pursuant to Renesas terms and conditions of sale.  Purchasers are solely responsible for the selection and use
 * of Renesas products and Renesas assumes no liability.  No license, express or implied, to any intellectual property
 * right is granted by Renesas. This software is protected under all applicable laws, including copyright laws. Renesas
 * reserves the right to change or discontinue this software and/or this documentation. THE SOFTWARE AND DOCUMENTATION
 * IS DELIVERED TO YOU "AS IS," AND RENESAS MAKES NO REPRESENTATIONS OR WARRANTIES, AND TO THE FULLEST EXTENT
 * PERMISSIBLE UNDER APPLICABLE LAW, DISCLAIMS ALL WARRANTIES, WHETHER EXPLICITLY OR IMPLICITLY, INCLUDING WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT, WITH RESPECT TO THE SOFTWARE OR
 * DOCUMENTATION.  RENESAS SHALL HAVE NO LIABILITY ARISING OUT OF ANY SECURITY VULNERABILITY OR BREACH.  TO THE MAXIMUM
 * EXTENT PERMITTED BY LAW, IN NO EVENT WILL RENESAS BE LIABLE TO YOU IN CONNECTION WITH THE SOFTWARE OR DOCUMENTATION
 * (OR ANY PERSON OR ENTITY CLAIMING RIGHTS DERIVED FROM YOU) FOR ANY LOSS, DAMAGES, OR CLAIMS WHATSOEVER, INCLUDING,
 * WITHOUT LIMITATION, ANY DIRECT, CONSEQUENTIAL, SPECIAL, INDIRECT, PUNITIVE, OR INCIDENTAL DAMAGES; ANY LOST PROFITS,
 * OTHER ECONOMIC DAMAGE, PROPERTY DAMAGE, OR PERSONAL INJURY; AND EVEN IF RENESAS HAS BEEN ADVISED OF THE POSSIBILITY
 * OF SUCH LOSS, DAMAGES, CLAIMS OR COSTS.
 **********************************************************************************************************************/

#ifndef BSP_MCU_API_H
#define BSP_MCU_API_H

/** Common macro for FSP header files. There is also a corresponding FSP_FOOTER macro at the end of this file. */
FSP_HEADER

typedef struct st_bsp_event_info
{
    IRQn_Type   irq;
    elc_event_t event;
} bsp_event_info_t;

typedef enum e_bsp_clocks_octaclk_div
{
    BSP_CLOCKS_OCTACLK_DIV_1 = 0,      ///< Divide OCTA source clock by 1
    BSP_CLOCKS_OCTACLK_DIV_2,          ///< Divide OCTA source clock by 2
    BSP_CLOCKS_OCTACLK_DIV_4,          ///< Divide OCTA source clock by 4
    BSP_CLOCKS_OCTACLK_DIV_6,          ///< Divide OCTA source clock by 6
    BSP_CLOCKS_OCTACLK_DIV_8           ///< Divide OCTA source clock by 8
} bsp_clocks_octaclk_div_t;

typedef enum e_bsp_clocks_source
{
    BSP_CLOCKS_CLOCK_HOCO = 0,         ///< The high speed on chip oscillator.
    BSP_CLOCKS_CLOCK_MOCO,             ///< The middle speed on chip oscillator.
    BSP_CLOCKS_CLOCK_LOCO,             ///< The low speed on chip oscillator.
    BSP_CLOCKS_CLOCK_MAIN_OSC,         ///< The main oscillator.
    BSP_CLOCKS_CLOCK_SUBCLOCK,         ///< The subclock oscillator.
    BSP_CLOCKS_CLOCK_PLL,              ///< The PLL oscillator.
    BSP_CLOCKS_CLOCK_PLL2,             ///< The PLL2 oscillator.
} bsp_clocks_source_t;

typedef struct st_bsp_octaclk_settings
{
    bsp_clocks_source_t      source_clock; ///< OCTACLK source clock
    bsp_clocks_octaclk_div_t divider;      ///< OCTACLK divider
} bsp_octaclk_settings_t;

void      R_BSP_RegisterProtectEnable(bsp_reg_protect_t regs_to_protect);
void      R_BSP_RegisterProtectDisable(bsp_reg_protect_t regs_to_unprotect);
fsp_err_t R_BSP_GroupIrqWrite(bsp_grp_irq_t irq, void (* p_callback)(bsp_grp_irq_t irq));
void      R_BSP_OctaclkUpdate(bsp_octaclk_settings_t * p_octaclk_setting);
void      R_BSP_SoftwareDelay(uint32_t delay, bsp_delay_units_t units);

/** Common macro for FSP header files. There is also a corresponding FSP_HEADER macro at the top of this file. */
FSP_FOOTER

#endif
