/****************************************************************************
 * boards/arm/imxrt/imxrt1060-evk/include/board.h
 *
 *   Copyright (C) 2018-2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *            Dave Marples <dave@marples.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_IMXRT_IMXRT1060_EVK_INCLUDE_BOARD_H
#define __BOARDS_ARM_IMXRT_IMXRT1060_EVK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Do not include i.MXRT header files here. */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Set VDD_SOC to 1.25V */

#define IMXRT_VDD_SOC (0x12)

/* Set Arm PLL (PLL1) to  fOut    = (24Mhz * ARM_PLL_DIV_SELECT/2) / ARM_PODF_DIVISOR
 *                        600Mhz  = (24Mhz * ARM_PLL_DIV_SELECT/2) / ARM_PODF_DIVISOR
 *                        ARM_PLL_DIV_SELECT = 100
 *                        ARM_PODF_DIVISOR   = 2
 *                        600Mhz  = (24Mhz * 100/2) / 2
 *
 *     AHB_CLOCK_ROOT             = PLL1fOut / IMXRT_AHB_PODF_DIVIDER
 *     1Hz to 600 Mhz             = 600Mhz / IMXRT_ARM_CLOCK_DIVIDER
 *                        IMXRT_ARM_CLOCK_DIVIDER = 1
 *                        600Mhz  = 600Mhz / 1
 *
 *     PRE_PERIPH_CLK_SEL         = PRE_PERIPH_CLK_SEL_PLL1
 *     PERIPH_CLK_SEL             = 1 (0 select PERIPH_CLK2_PODF,
 *                                     1 select PRE_PERIPH_CLK_SEL_PLL1)
 *     PERIPH_CLK                 = 600Mhz
 *
 *     IPG_CLOCK_ROOT             = AHB_CLOCK_ROOT / IMXRT_IPG_PODF_DIVIDER
 *                       IMXRT_IPG_PODF_DIVIDER = 4
 *                       150Mhz = 600Mhz / 4
 *
 *     PRECLK_CLOCK_ROOT          = IPG_CLOCK_ROOT / IMXRT_PERCLK_PODF_DIVIDER
 *                       IMXRT_PERCLK_PODF_DIVIDER = 9
 *                       16.6Mhz  = 150Mhz / 9
 *
 *     SEMC_CLK_ROOT              = 600Mhz / IMXRT_SEMC_PODF_DIVIDER
 *                                  (labeled AIX_PODF in 18.2)
 *                       IMXRT_SEMC_PODF_DIVIDER = 8
 *                       75Mhz    = 600Mhz / 8
 *
 * Set Sys PLL (PLL2) to  fOut    = (24Mhz * (20+(2*(DIV_SELECT)))
 *                        528Mhz  = (24Mhz * (20+(2*(1)))
 *
 * Set USB1 PLL (PLL3) to fOut    = (24Mhz * 20)
 *                         480Mhz = (24Mhz * 20)
 *
 * Set LPSPI PLL3 PFD0 to fOut    = (480Mhz / 12 * 18)
 *                        720Mhz  = (480Mhz / 12 * 18)
 *                         90Mhz  = (720Mhz / LSPI_PODF_DIVIDER)
 *
 * Set LPI2C PLL3 / 8 to   fOut   = (480Mhz / 8)
 *                         60Mhz  = (480Mhz / 8)
 *                         12Mhz  = (60Mhz / LSPI_PODF_DIVIDER)
 *
 * These clock frequencies can be verified via the CCM_CLKO1 pin and sending
 * the appropriate clock to it with something like;
 *
 *   putreg32( <Clk number> | CCM_CCOSR_CLKO1_EN ,   IMXRT_CCM_CCOSR);
 *   imxrt_config_gpio(GPIO_CCM_CLKO1);
 */

#define BOARD_XTAL_FREQUENCY       24000000
#define IMXRT_PRE_PERIPH_CLK_SEL   CCM_CBCMR_PRE_PERIPH_CLK_SEL_PLL1
#define IMXRT_PERIPH_CLK_SEL       CCM_CBCDR_PERIPH_CLK_SEL_PRE_PERIPH
#define IMXRT_ARM_PLL_DIV_SELECT   100
#define IMXRT_ARM_PODF_DIVIDER     2
#define IMXRT_AHB_PODF_DIVIDER     1
#define IMXRT_IPG_PODF_DIVIDER     4
#define IMXRT_PERCLK_CLK_SEL       CCM_CSCMR1_PERCLK_CLK_SEL_IPG_CLK_ROOT
#define IMXRT_PERCLK_PODF_DIVIDER  9
#define IMXRT_SEMC_PODF_DIVIDER    8

#define IMXRT_LPSPI_CLK_SELECT     CCM_CBCMR_LPSPI_CLK_SEL_PLL3_PFD0
#define IMXRT_LSPI_PODF_DIVIDER    8

#define IMXRT_LPI2C_CLK_SELECT     CCM_CSCDR2_LPI2C_CLK_SEL_PLL3_60M
#define IMXRT_LSI2C_PODF_DIVIDER   5

#define IMXRT_SYS_PLL_SELECT       CCM_ANALOG_PLL_SYS_DIV_SELECT_22

#define BOARD_CPU_FREQUENCY \
  (BOARD_XTAL_FREQUENCY * (IMXRT_ARM_PLL_DIV_SELECT / 2)) / IMXRT_ARM_PODF_DIVIDER

/* LED definitions **********************************************************/

/* There are four LED status indicators located on the EVK Board.
 * The functions of these LEDs include:
 *
 *   - Main Power Supply(D3)
 *     Green: DC 5V main supply is normal.
 *     Red:   J2 input voltage is over 5.6V.
 *     Off:   The board is not powered.
 *   - Reset RED LED(D15)
 *   - OpenSDA LED(D16)
 *   - USER LED(D18)
 *
 * Only a single LED, D18, is under software control.
 */

/* LED index values for use with board_userled() */

#define BOARD_USERLED     0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_USERLED_BIT (1 << BOARD_USERLED)

/* This LED is not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/imxrt_autoleds.c. The LED is used to encode
 * OS-related events as follows:
 *
 *   -------------------- ----------------------------- ------
 *   SYMBOL                   Meaning                   LED
 *   -------------------- ----------------------------- ------
 */

#define LED_STARTED       0  /* NuttX has been started  OFF    */
#define LED_HEAPALLOCATE  0  /* Heap has been allocated OFF    */
#define LED_IRQSENABLED   0  /* Interrupts enabled      OFF    */
#define LED_STACKCREATED  1  /* Idle stack created      ON     */
#define LED_INIRQ         2  /* In an interrupt         N/C    */
#define LED_SIGNAL        2  /* In a signal handler     N/C    */
#define LED_ASSERTION     2  /* An assertion failed     N/C    */
#define LED_PANIC         3  /* The system has crashed  FLASH  */
#undef  LED_IDLE             /* Not used                       */

/* Thus if the LED is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  If the LED is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

/* Touchscreen definitions **************************************************/

/* The IMXRT 1050/1060 have connectors for the LCD model RK043FN02H-CT.
 * It comes with the FT5336GQQ (FT5X06) touchscreen chip integrated.
 * FT5X06 is connected to the LPI2C1 bus.
 */

/* LPI2C address of the FT5336GQQ touchscreen chip */

#define FT5X06_I2C_ADDRESS  0x38

/* Button definitions *******************************************************/

/* The IMXRT board has one external user button
 *
 * 1. SW8 (IRQ88)   GPIO5-00
 */

#define BUTTON_SW8        0
#define NUM_BUTTONS       1

#define BUTTON_SW8_BIT    (1 << BUTTON_SW8)

/* SDIO *********************************************************************/

/* Pin drive characteristics - drive strength in particular may need tuning
 * for specific boards, but has been checked by scope on the EVKB to make
 * sure shapes are square with minimal ringing.
 */

#define PIN_USDHC1_D0     (GPIO_USDHC1_DATA0 | IOMUX_USDHC1_DATAX_DEFAULT)
#define PIN_USDHC1_D1     (GPIO_USDHC1_DATA1 | IOMUX_USDHC1_DATAX_DEFAULT)
#define PIN_USDHC1_D2     (GPIO_USDHC1_DATA2 | IOMUX_USDHC1_DATAX_DEFAULT)
#define PIN_USDHC1_D3     (GPIO_USDHC1_DATA3 | IOMUX_USDHC1_DATAX_DEFAULT)
#define PIN_USDHC1_DCLK   (GPIO_USDHC1_CLK   | IOMUX_USDHC1_CLK_DEFAULT)
#define PIN_USDHC1_CMD    (GPIO_USDHC1_CMD   | IOMUX_USDHC1_CMD_DEFAULT)
#define PIN_USDHC1_CD     (GPIO_USDHC1_CD_2  | IOMUX_USDHC1_CLK_DEFAULT)

/* 386 KHz for initial inquiry stuff */

#define BOARD_USDHC_IDMODE_PRESCALER    USDHC_SYSCTL_SDCLKFS_DIV256
#define BOARD_USDHC_IDMODE_DIVISOR      USDHC_SYSCTL_DVS_DIV(2)

/* 24.8MHz for other modes */

#define BOARD_USDHC_MMCMODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_MMCMODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

#define BOARD_USDHC_SD1MODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_SD1MODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

#define BOARD_USDHC_SD4MODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_SD4MODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

/* Buttons ****************************************************************/

#define GPIO_SW        (GPIO_INTERRUPT | GPIO_INT_FALLINGEDGE | \
                        IOMUX_SW_DEFAULT | \
                        GPIO_PORT5 | GPIO_PIN0 | )              /* WAKEUP */

/* Test Pins **************************************************************/

#define BOARD_NGPIOIN   0 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT  4 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT  0 /* Amount of GPIO Input w/ Interruption pins */

#define GPIO_GOUT1      (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | IOMUX_GOUT_DEFAULT | \
                         GPIO_PORT1 | GPIO_PIN19)

#define GPIO_GOUT2      (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | IOMUX_GOUT_DEFAULT | \
                         GPIO_PIN18 | GPIO_PORT1)

#define GPIO_GOUT3      (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | IOMUX_GOUT_DEFAULT | \
                         GPIO_PIN10 | GPIO_PORT1)

#define GPIO_GOUT4      (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | IOMUX_GOUT_DEFAULT | \
                         GPIO_PIN9 | GPIO_PORT1)

/* LED Disambiguation *******************************************************/

#ifdef CONFIG_ARCH_LEDS
#define GPIO_LED        (GPIO_OUTPUT | IOMUX_LED_DEFAULT | \
                         GPIO_OUTPUT_ZERO | GPIO_PORT1 | GPIO_PIN9)  /* AD_BO_09 */
#endif

/* LCD *********************************************************************/

#ifdef CONFIG_IMXRT_LCD
/* Backlight */

#  define GPIO_LCD_BL        (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GPIO_PORT2 | \
                              GPIO_PIN31 | IOMUX_LCD_BL_DEFAULT)

/* LCD controller */

#  define GPIO_LCD_DATA23    GPIO_LCD_DATA23_1
#  define GPIO_LCD_DATA22    GPIO_LCD_DATA22_1
#  define GPIO_LCD_DATA21    GPIO_LCD_DATA21_1
#  define GPIO_LCD_DATA20    GPIO_LCD_DATA20_1
#  define GPIO_LCD_DATA19    GPIO_LCD_DATA19_1
#  define GPIO_LCD_DATA18    GPIO_LCD_DATA18_1
#  define GPIO_LCD_DATA17    GPIO_LCD_DATA17_1
#  define GPIO_LCD_DATA16    GPIO_LCD_DATA16_1
#  define GPIO_LCD_DATA15    GPIO_LCD_DATA15_1
#  define GPIO_LCD_DATA14    GPIO_LCD_DATA14_1
#  define GPIO_LCD_DATA13    GPIO_LCD_DATA13_1
#  define GPIO_LCD_DATA12    GPIO_LCD_DATA12_1
#  define GPIO_LCD_DATA11    GPIO_LCD_DATA11_1
#  define GPIO_LCD_DATA10    GPIO_LCD_DATA10_1
#  define GPIO_LCD_DATA09    GPIO_LCD_DATA09_1
#  define GPIO_LCD_DATA08    GPIO_LCD_DATA08_1
#  define GPIO_LCD_DATA07    GPIO_LCD_DATA07_1
#  define GPIO_LCD_DATA06    GPIO_LCD_DATA06_1
#  define GPIO_LCD_DATA05    GPIO_LCD_DATA05_1
#  define GPIO_LCD_DATA04    GPIO_LCD_DATA04_1
#  define GPIO_LCD_DATA03    GPIO_LCD_DATA03_1
#  define GPIO_LCD_DATA02    GPIO_LCD_DATA02_1
#  define GPIO_LCD_DATA01    GPIO_LCD_DATA01_1
#  define GPIO_LCD_DATA00    GPIO_LCD_DATA00_1
#  define GPIO_LCD_ENABLE    GPIO_LCD_ENABLE_1
#  define GPIO_LCD_HSYNC     GPIO_LCD_HSYNC_1
#  define GPIO_LCD_VSYNC     GPIO_LCD_VSYNC_1
#  define GPIO_LCD_CLK       GPIO_LCD_CLK_1
#endif

/* ETH Disambiguation *******************************************************/

#define GPIO_ENET_TX_DATA00  (GPIO_ENET_TX_DATA00_1| \
                              IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_07 */
#define GPIO_ENET_TX_DATA01  (GPIO_ENET_TX_DATA01_1| \
                              IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_08 */
#define GPIO_ENET_RX_DATA00  (GPIO_ENET_RX_DATA00_1| \
                              IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_04 */
#define GPIO_ENET_RX_DATA01  (GPIO_ENET_RX_DATA01_1| \
                              IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_05 */
#define GPIO_ENET_MDIO       (GPIO_ENET_MDIO_3|IOMUX_ENET_MDIO_DEFAULT)   /* GPIO_EMC_41 */
#define GPIO_ENET_MDC        (GPIO_ENET_MDC_3|IOMUX_ENET_MDC_DEFAULT)     /* GPIO_EMC_40 */
#define GPIO_ENET_RX_EN      (GPIO_ENET_RX_EN_1|IOMUX_ENET_EN_DEFAULT)    /* GPIO_B1_06 */
#define GPIO_ENET_RX_ER      (GPIO_ENET_RX_ER_1|IOMUX_ENET_RXERR_DEFAULT) /* GPIO_B1_11 */
#define GPIO_ENET_TX_CLK     (GPIO_ENET_REF_CLK_2|\
                              IOMUX_ENET_TX_CLK_DEFAULT)                  /* GPIO_B1_10 */
#define GPIO_ENET_TX_EN      (GPIO_ENET_TX_EN_1|IOMUX_ENET_EN_DEFAULT)    /* GPIO_B1_09 */
#define GPIO_ENET_INT        (IOMUX_ENET_INT_DEFAULT | \
                              GPIO_PORT1 | GPIO_PIN10)                    /* AD_B0_10 */
#define GPIO_ENET_IRQ         IMXRT_IRQ_GPIO1_10
#define GPIO_ENET_RST        (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                              GPIO_PORT1 | GPIO_PIN9 | IOMUX_ENET_RST_DEFAULT) /* AD_B0_09 */

#ifdef CONFIG_ETH0_PHY_KSZ8081
#  ifdef GPIO_LED
#    warning LED interferes with ETH reset unless R323 is removed.
#  endif
#endif

/* PIO Disambiguation *******************************************************/

/* LPUARTs
 *
 * Virtual console port provided by OpenSDA on UART1 and
 * Arduino RS-232 Shield on UART3.
 *
 */

#define GPIO_LPUART1_RX      (GPIO_LPUART1_RX_1|IOMUX_UART_DEFAULT) /* GPIO_AD_B0_13 */
#define GPIO_LPUART1_TX      (GPIO_LPUART1_TX_1|IOMUX_UART_DEFAULT) /* GPIO_AD_B0_12 */
#define GPIO_LPUART3_RX      (GPIO_LPUART3_RX_1|IOMUX_UART_DEFAULT) /* GPIO_AD_B1_07 */
#define GPIO_LPUART3_TX      (GPIO_LPUART3_TX_1|IOMUX_UART_DEFAULT) /* GPIO_AD_B1_06 */

/* LPI2Cs
 *
 * Arduino Connector LPI2C1 and audio/gyro IO on LPI2C3.
 */

#define GPIO_LPI2C1_SDA      (GPIO_LPI2C1_SDA_2|IOMUX_LPI2C_DEFAULT) /* GPIO_AD_B1_01 */
#define GPIO_LPI2C1_SCL      (GPIO_LPI2C1_SCL_2|IOMUX_LPI2C_DEFAULT) /* GPIO_AD_B1_00 */
#define GPIO_LPI2C3_SDA      (GPIO_LPI2C3_SDA_2|IOMUX_LPI2C_DEFAULT) /* GPIO_AD_B1_01 */
#define GPIO_LPI2C3_SCL      (GPIO_LPI2C3_SCL_2|IOMUX_LPI2C_DEFAULT) /* GPIO_AD_B1_00 */

/* LPSPI
 *
 * Arduino Connector
 *
 *   J24 D09   GPIO_AD_B0_02  LPSPI3_SDI
 *   J24 D14   GPIO_AD_B0_01  LPSPI3_SDO
 *   J24 D15   GPIO_AD_B0_00  LPSPI3_SCK
 */

#define GPIO_LPSPI3_SCK      (GPIO_LPSPI3_SCK_2|IOMUX_LPSPI_DEFAULT) /* GPIO_AD_B0_00 */
#define GPIO_LPSPI3_MISO     (GPIO_LPSPI3_SDI_2|IOMUX_LPSPI_DEFAULT) /* GPIO_AD_B0_02 */
#define GPIO_LPSPI3_MOSI     (GPIO_LPSPI3_SDO_2|IOMUX_LPSPI_DEFAULT) /* GPIO_AD_B0_01 */
#define IOMUX_LPSPI3_CS      (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                              IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                              _IOMUX_PULL_ENABLE)
#define GPIO_LPSPI3_CS       (GPIO_OUTPUT | GPIO_OUTPUT_ONE | \
                              GPIO_PORT1 | GPIO_PIN3 | IOMUX_LPSPI3_CS) /* GPIO_AD_B0_03 */

/* LPSPI1 CS:  GPIO_SD_B0_01 */

#define IOMUX_LPSPI1_CS      (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                              IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                              _IOMUX_PULL_ENABLE)
#define GPIO_LPSPI1_CS       (GPIO_OUTPUT | GPIO_OUTPUT_ONE | \
                              GPIO_PORT3 | GPIO_PIN13 | IOMUX_LPSPI1_CS)

#define IOMUX_MMCSD_EN       (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                              IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                              _IOMUX_PULL_ENABLE)
#define GPIO_MMCSD_EN        (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                              GPIO_PORT3 | GPIO_PIN2 | IOMUX_MMCSD_EN)

#define GPIO_FT5X06_INTR     IMXRT_IRQ_GPIO1_11

#define IOMUX_FT5X06_RST     (IOMUX_PULL_NONE | IOMUX_CMOS_OUTPUT | \
                              IOMUX_DRIVE_40OHM | IOMUX_SPEED_MEDIUM | \
                              IOMUX_SLEW_SLOW)                            /* AD_B0_11 */
#define GPIO_FT5X06_CTRSTn   (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                              GPIO_PORT1 | GPIO_PIN2 | IOMUX_FT5X06_RST)  /* AD_B0_02 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_IMXRT_IMXRT1060_EVK_INCLUDE_BOARD_H */
