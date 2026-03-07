/**
 * @file jtag_process_new.c
 * @brief 
 * 
 * Copyright (c) 2021 Sipeed team
 * Copyright (C) 2026 Sergey Koshkin <koshkin.sergey@gmail.com>
 * 
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 * 
 *   http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 * 
 */

#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#include "jtag_process.h"
#include "usb_dc.h"
#include "hal_gpio.h"
#include "hal_common.h"
#include "bl702_gpio.h"
#include "bl702_pwm.h"
#include "bl702_glb.h"
#include "io_cfg.h"

#define GOWIN_INT_FLASH_QUIRK     0
#define PWM_CH                    3

#define GPIO_IN_ADDR              ((volatile uint32_t *)0x40000180)
#define GPIO_OUT_ADDR             ((volatile uint32_t *)0x40000188)
#define TDO                       (*GPIO_IN_ADDR & (TDO_PIN_MASK))

#define MPSSE_IDLE                0
#define MPSSE_RCV_LENGTH_1        1
#define MPSSE_RCV_LENGTH_2        2
#define MPSSE_RCV_VALUE_L         3
#define MPSSE_RCV_VALUE_H         4
#define MPSSE_TRANSMIT_BYTE       5
#define MPSSE_TRANSMIT_BIT        6
#define MPSSE_RUN_TEST            7
#define MPSSE_SET_VALUE           8
#define MPSSE_SET_DIRECTION       9

/* Data Shifting Command Bit Definitions */
#define DSC_NVE_CLK_ON_WR         (1UL << 0)
#define DSC_BIT_MODE              (1UL << 1)
#define DSC_NVE_CLK_ON_RD         (1UL << 2)
#define DSC_LSB_FIRST             (1UL << 3)
#define DSC_WRITE_TDI             (1UL << 4)
#define DSC_READ_TDO              (1UL << 5)
#define DSC_WRITE_TMS             (1UL << 6)
#define DSC_INSTRACTION           (1UL << 7)

#define TMS_PIN_MASK              (1 << TMS_PIN)
#define TCK_PIN_MASK              (1 << TCK_PIN)
#define TDI_PIN_MASK              (1 << TDI_PIN)
#define TDO_PIN_MASK              (1 << TDO_PIN)

#define JTAG_TX_BUFFER_SIZE       (1024)
#define JTAG_RX_BUFFER_SIZE       (4 * 1024)

// 6.94 ns every "nop"
// 20.82 ns every one PIN_DELAY()
//#define DELAY_IMPULSE()
#define DELAY_IMPULSE()           PIN_DELAY(delay_val)
//#define DELAY_RUN_TEST()          PIN_DELAY(20)
#define PIN_DELAY_CALC(mhz, div)  ((div) * (1000 / PIN_DELAY_NS) / (mhz))
#define CLK_MHZ_DEFAULT           (12U)
#define CLK_DIV_DEFAULT           (0U)

static uint8_t jtag_tx_buffer[JTAG_TX_BUFFER_SIZE] __attribute__((section(".tcm_data")));
Ring_Buffer_Type jtag_tx_rb;

static uint8_t jtag_rx_buffer[JTAG_RX_BUFFER_SIZE] __attribute__((section(".tcm_data")));
Ring_Buffer_Type jtag_rx_rb;

static uint32_t clk_mhz __attribute__((section(".tcm_data"))) = CLK_MHZ_DEFAULT;
static uint16_t clk_div __attribute__((section(".tcm_data"))) = CLK_DIV_DEFAULT;
static uint32_t delay_val __attribute__((section(".tcm_data"))) = PIN_DELAY_CALC(CLK_MHZ_DEFAULT, CLK_DIV_DEFAULT);
static uint16_t mpsse_length __attribute__((section(".tcm_data")));
static uint32_t mpsse_status __attribute__((section(".tcm_data"))) = MPSSE_IDLE;
static uint8_t mpsse_cmd __attribute__((section(".tcm_data")));
static uint32_t output_pin_mask __attribute__((section(".tcm_data")));

static
void jtag_write(uint8_t data)
{
  Ring_Buffer_Write_Byte(&jtag_tx_rb, data);
}

static
void ringbuffer_lock(void)
{
  cpu_global_irq_disable();
}

static
void ringbuffer_unlock(void)
{
  cpu_global_irq_enable();
}

void jtag_ringbuffer_init(void)
{
  memset(jtag_tx_buffer, 0, JTAG_TX_BUFFER_SIZE);
  memset(jtag_rx_buffer, 0, JTAG_RX_BUFFER_SIZE);

  /* init ring_buffer */
  Ring_Buffer_Init(&jtag_tx_rb, jtag_tx_buffer, JTAG_TX_BUFFER_SIZE,
                   ringbuffer_lock, ringbuffer_unlock);
  Ring_Buffer_Init(&jtag_rx_rb, jtag_rx_buffer, JTAG_RX_BUFFER_SIZE,
                   ringbuffer_lock, ringbuffer_unlock);
}

#if GOWIN_INT_FLASH_QUIRK
static void pwm_start(void)
{
    GLB_GPIO_Cfg_Type gpio_cfg;

    gpio_cfg.drive = 0;
    gpio_cfg.smtCtrl = 1;
    gpio_cfg.gpioMode = GPIO_MODE_AF;
    gpio_cfg.pullType = GPIO_PULL_DOWN;
    gpio_cfg.gpioFun = GPIO_FUN_PWM;
    gpio_cfg.gpioPin = TCK_PIN;
    GLB_GPIO_Init(&gpio_cfg);
    PWM_Channel_Enable(PWM_CH);
}

static void pwm_stop(void)
{
    PWM_Channel_Disable(PWM_CH);

    GLB_GPIO_Cfg_Type gpio_cfg;
    gpio_cfg.drive = 0;
    gpio_cfg.smtCtrl = 1;
    gpio_cfg.gpioMode = GPIO_MODE_OUTPUT;
    gpio_cfg.pullType = GPIO_PULL_NONE;
    gpio_cfg.gpioFun = GPIO_FUN_GPIO;
    gpio_cfg.gpioPin = TCK_PIN;
    GLB_GPIO_Init(&gpio_cfg);
}

void pwm_init(void)
{
    static PWM_CH_CFG_Type pwmCfg =
    {
        .ch = PWM_CH,
        .clk = PWM_CLK_BCLK,
        .stopMode = PWM_STOP_GRACEFUL,
        .pol = PWM_POL_NORMAL,
        .clkDiv = 1,
        .period = 28,
        .threshold1 = 0,
        .threshold2 = 14,
        .intPulseCnt = 0,
    };
    PWM_Channel_Init(&pwmCfg);
}
#endif

void jtag_gpio_init(void)
{
  gpio_write(TMS_PIN, 0U);
  gpio_set_mode(TMS_PIN, GPIO_OUTPUT_MODE);
  gpio_write(TDI_PIN, 0U);
  gpio_set_mode(TDI_PIN, GPIO_OUTPUT_MODE);
  gpio_write(TCK_PIN, 1U);
  gpio_set_mode(TCK_PIN, GPIO_OUTPUT_MODE);
  gpio_set_mode(TDO_PIN, GPIO_INPUT_MODE);

#if GOWIN_INT_FLASH_QUIRK 
  pwm_init();
#endif
}

static
ATTR_CLOCK_SECTION void transmit_bits(uint8_t cmd, uint8_t rx_data, uint32_t cnt)
{
  register uint8_t tx_data;
  register uint32_t bitbang;
  register uint32_t i;
  register uint32_t bit;
  register uint32_t isLSB;
  volatile uint32_t *gpio_out = GPIO_OUT_ADDR;

  tx_data = 0U;
  bit = 0U;
  isLSB = (cmd & DSC_LSB_FIRST) != 0U;
  bitbang = *gpio_out;

  if (output_pin_mask == TMS_PIN_MASK) {
    if ((rx_data & (1 << 7)) != 0U) {
      bitbang |= TDI_PIN_MASK;
    }
    else {
      bitbang &= ~TDI_PIN_MASK;
    }
  }

  do {
    i = isLSB ? bit : 7U - bit;

    if (rx_data & (1 << i)) {
      bitbang |= output_pin_mask;
    }
    else {
      bitbang &= ~output_pin_mask;
    }

    //TCK_LOW;
    bitbang &= ~TCK_PIN_MASK;
    *gpio_out = bitbang;
    DELAY_IMPULSE();

    //TCK_HIGH;
    bitbang |= TCK_PIN_MASK;
    *gpio_out = bitbang;
    DELAY_IMPULSE();

    if (TDO != 0U) {
      tx_data |= 1 << i;
    }
  } while (bit++ < cnt);

  if ((cmd & DSC_READ_TDO) != 0U) {
    jtag_write(tx_data);
  }
}

ATTR_CLOCK_SECTION void jtag_process(void)
{
  register uint8_t rx_data;
  register uint32_t jtag_rx_pos;
  register uint32_t jtag_rx_len;
  static uint8_t rx_buf[JTAG_RX_BUFFER_SIZE] __attribute__((section(".tcm_data")));

  jtag_rx_len = Ring_Buffer_Read(&jtag_rx_rb, rx_buf, sizeof(rx_buf));
  if (jtag_rx_len == 0U) {
    return;
  }

  jtag_rx_pos = 0U;
  cpu_global_irq_disable();

  while (jtag_rx_pos < jtag_rx_len) {
    rx_data = rx_buf[jtag_rx_pos++];

    switch (mpsse_status) {
      case MPSSE_IDLE:
        mpsse_cmd = rx_data;
        if ((mpsse_cmd & DSC_INSTRACTION) == 0U) {
          if ((mpsse_cmd & (DSC_READ_TDO|DSC_WRITE_TDI|DSC_WRITE_TMS)) != 0U && (mpsse_cmd & DSC_NVE_CLK_ON_WR) != 0U) {
            if ((mpsse_cmd & DSC_WRITE_TMS) != 0U) {
              output_pin_mask = TMS_PIN_MASK;
            }
            else if ((mpsse_cmd & DSC_WRITE_TDI) != 0U) {
              output_pin_mask = TDI_PIN_MASK;
            }
            else {
              output_pin_mask = 0U;
            }

            mpsse_status = MPSSE_RCV_LENGTH_1;
          }
        }
        else {
          switch (mpsse_cmd) {
            /* Instructions*/
            case 0x80:  //This will setup the direction of the first 8 lines and force a value on the bits that are set as output.
            case 0x82:  //This will setup the direction of the high 8 lines and force a value on the bits that are set as output.
              mpsse_status = MPSSE_SET_VALUE;
              break;
            case 0x81:  //This will read the current state of the first 8 pins and send back 1 byte.
            case 0x83:  //This will read the current state of the high 8 pins and send back 1 byte.
              jtag_write(0U);
              break;
            case 0x84:  //Connect TDI to TDO for Loopback
            case 0x85:  //Disconnect TDI to TDO for Loopback
              __NOP();
              break;
            case 0x86:  //This will set the clock divisor.
              mpsse_status = MPSSE_RCV_VALUE_L;
              break;
            case 0x87:  //Send immediate. This will make the chip flush its buffer back to the PC.
              __NOP();
              break;
            case 0x8a:  //Disables the clk divide by 5 to allow for a 60MHz master clock
              clk_mhz = 60U;
              delay_val = PIN_DELAY_CALC(clk_mhz, clk_div);
              break;
            case 0x8b:  //Enables the clk divide by 5 to allow for backward compatibility with FT2232D
              clk_mhz = 12U;
              delay_val = PIN_DELAY_CALC(clk_mhz, clk_div);
              break;
            default:
              jtag_write(0xFA);
              jtag_write(mpsse_cmd);
              break;
          }
        }
        break;

      case MPSSE_RCV_LENGTH_1:
        mpsse_length = rx_data;
        if ((mpsse_cmd & DSC_BIT_MODE) == 0U) {
          mpsse_status = MPSSE_RCV_LENGTH_2;
        }
        else {
          mpsse_status = MPSSE_TRANSMIT_BIT;
        }
        break;

      case MPSSE_RCV_LENGTH_2:
        mpsse_length |= rx_data << 8;
#if GOWIN_INT_FLASH_QUIRK
        if ((mpsse_length >=8000) && (mpsse_cmd & DSC_READ_TDO) == 0) {
          pwm_start();
          mpsse_status = MPSSE_RUN_TEST;
        }
        else
#endif
        mpsse_status = MPSSE_TRANSMIT_BYTE;
        break;

      case MPSSE_RCV_VALUE_L:
        clk_div = rx_data;
        mpsse_status = MPSSE_RCV_VALUE_H;
        break;

      case MPSSE_RCV_VALUE_H:
        clk_div |= rx_data << 8;
        delay_val = PIN_DELAY_CALC(clk_mhz, clk_div);
        mpsse_status = MPSSE_IDLE;
        break;

      case MPSSE_TRANSMIT_BYTE:
        transmit_bits(mpsse_cmd, rx_data, 7U);

        if (mpsse_length > 0U) {
          --mpsse_length;
        }
        else {
          mpsse_status = MPSSE_IDLE;
        }
        break;

      case MPSSE_TRANSMIT_BIT:
        transmit_bits(mpsse_cmd, rx_data, mpsse_length);
        mpsse_status = MPSSE_IDLE;
        break;

      case MPSSE_SET_VALUE:
        mpsse_status = MPSSE_SET_DIRECTION;
        break;

      case MPSSE_SET_DIRECTION:
        mpsse_status = MPSSE_IDLE;
        break;

#if GOWIN_INT_FLASH_QUIRK
      case MPSSE_RUN_TEST:
        if (mpsse_length == 0) {
          mpsse_status = MPSSE_IDLE;
          pwm_stop();
        }

        DELAY_RUN_TEST();

        jtag_rx_pos++;
        mpsse_length--;
        break;
#endif

      default:
        mpsse_status = MPSSE_IDLE;
        break;
    }
  }

  cpu_global_irq_enable();
}
