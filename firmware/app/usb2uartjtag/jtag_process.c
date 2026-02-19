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

#include "ring_buffer.h"
#include "usb_dc.h"
#include "hal_gpio.h"
#include "hal_common.h"
#include "bl702_gpio.h"
#include "bl702_pwm.h"
#include "bl702_glb.h"
#include "io_cfg.h"
#include "usbd_ftdi.h"

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

#define JTAG_TX_BUFFER_SIZE       (1 * 1024)
#define JTAG_RX_BUFFER_SIZE       (64)

// 6.94 ns every "nop"
// 20.82 ns every one PIN_DELAY()
#define DELAY_LOW()               PIN_DELAY(delay_val)
#define DELAY_HIGH()              PIN_DELAY(delay_val)
#define DELAY_RUN_TEST()          PIN_DELAY(60)

extern void led_set(uint8_t idx, uint8_t status);

static uint32_t delay_val;
static uint8_t jtag_tx_buffer[JTAG_TX_BUFFER_SIZE] __attribute__((section(".tcm_data")));
static Ring_Buffer_Type jtag_tx_rb;

static uint8_t jtag_rx_buffer[JTAG_RX_BUFFER_SIZE] __attribute__((section(".tcm_data")));
static volatile uint32_t jtag_rx_len = 0;
static volatile uint32_t jtag_rx_pos __attribute__((section(".tcm_data")));

static uint16_t mpsse_length __attribute__((section(".tcm_data")));
static uint16_t mpsse_value  __attribute__((section(".tcm_data")));
static uint32_t mpsse_status __attribute__((section(".tcm_data"))) = MPSSE_IDLE;
static uint32_t jtag_cmd __attribute__((section(".tcm_data")));
static volatile uint32_t jtag_received_flag __attribute__((section(".tcm_data"))) = false;

extern struct device *usb_fs;

static void jtag_write(uint8_t data)
{
  Ring_Buffer_Write_Byte(&jtag_tx_rb, data);
}

void jtag_ringbuffer_init(void)
{
  memset(jtag_tx_buffer, 0, JTAG_TX_BUFFER_SIZE);
  /* init ring_buffer */
  Ring_Buffer_Init(&jtag_tx_rb, jtag_tx_buffer, JTAG_TX_BUFFER_SIZE, NULL, NULL);
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
        .period = 29,
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
  gpio_write(TCK_PIN, 0U);
  gpio_set_mode(TCK_PIN, GPIO_OUTPUT_MODE);
  gpio_set_mode(TDO_PIN, GPIO_INPUT_MODE);

#if GOWIN_INT_FLASH_QUIRK 
  pwm_init();
#endif
}

void usbd_cdc_jtag_out(uint8_t ep)
{
  (void) ep;

  jtag_received_flag = true;
}

extern uint16_t usb_dc_ftdi_send_from_ringbuffer(struct device *dev, Ring_Buffer_Type *rb, uint8_t ep);
void usbd_cdc_jtag_in(uint8_t ep)
{
  if (jtag_rx_len == 0U) {
    usb_dc_ftdi_send_from_ringbuffer(usb_fs, &jtag_tx_rb, ep);
  }
}

ATTR_CLOCK_SECTION void jtag_process(void)
{
  uint32_t chunk;
  uint8_t tx_data;
  uint8_t rx_data;
  uint32_t output_pin_mask;
  uint32_t bitbang;
  volatile uint32_t *gpio_out = GPIO_OUT_ADDR;

  if (jtag_received_flag == false) {
    return;
  }

  usbd_ep_read(JTAG_OUT_EP, jtag_rx_buffer, JTAG_RX_BUFFER_SIZE, &chunk);
  if (chunk == 0U) {
    return;
  }

  jtag_rx_len = chunk;
  jtag_rx_pos = 0;
  jtag_received_flag = false;
  usbd_ep_read(JTAG_OUT_EP, NULL, 0, NULL);

  led_set(0, 1);

  while (jtag_rx_pos < jtag_rx_len) {
    switch (mpsse_status) {
      case MPSSE_IDLE:
        jtag_cmd = jtag_rx_buffer[jtag_rx_pos++];
        if ((jtag_cmd & DSC_INSTRACTION) == 0U) {
          mpsse_status = MPSSE_RCV_LENGTH_1;
        }
        else {
          switch (jtag_cmd) {
            /* Instructions*/
            case 0x80:
            case 0x82:/* set data bit as output, we just nop it */
              jtag_rx_pos += 2U;
              break;
            case 0x81:
            case 0x83:/* dummy read back 8pins data, and send to usb */
              jtag_write(0U);
              break;
            case 0x86: /* Clock Divisor, 0xValueL, 0xValueH */
              mpsse_status = MPSSE_RCV_VALUE_L;
              break;
            case 0x8e:	//Allows for a clock to be output without transferring data. Commonly used in the JTAG state machine. Clocks counted in terms of numbers of bit
              jtag_rx_pos += 1U;
              break;
            case 0x8f:	//Allows for a clock to be output without transferring data. Commonly used in the JTAG state machine. Clocks counted in terms of numbers of bytes
            case 0x9c:	//Allows for a clock to be output without transferring data until a logic 1 input on GPIOL1 stops the clock or a set number of clock pulses are sent. Clocks counted in terms of numbers of bytes
            case 0x9d:	//Allows for a clock to be output without transferring data until a logic 0 input on GPIOL1 stops the clock or a set number of clock pulses are sent. Clocks counted in terms of numbers of bytes
              jtag_rx_pos += 2U;
              break;
            case 0x84:  //This will connect the TDI/DO output to the TDO/DI input for loopback testing.
            case 0x85:  //This will disconnect the TDI output from the TDO input for loopback testing.
            case 0x87:  //Send immediate. This will make the chip flush its buffer back to the PC.
            case 0x88:  //This will cause the MPSSE controller to wait until GPIOL1 (JTAG) or I/O1 (CPU) is high.
            case 0x89:  //This will cause the controller to wait until GPIOL1 (JTAG) or I/O1 (CPU) is low.
            case 0x8a:	//Disables the clk divide by 5 to allow for a 60MHz master clock
            case 0x8b:	//Enables the clk divide by 5 to allow for backward compatibility with FT2232D
            case 0x8c:	//Enables 3 phase data clocking. Used by I2C interfaces to allow data on both clock edges.
            case 0x8d:	//Disables 3 phase data clocking.
            case 0x94:	//Allows for a clock to be output without transferring data until a logic 1 input on GPIOL1 stops the clock
            case 0x95:	//Allows for a clock to be output without transferring data until a logic 0 input on GPIOL1 stops the clock.
            case 0x96:	//Allows for a clock to be output without transferring data until a logic 0 input on GPIOL1 stops the clock.
            case 0x97:	//Disable adaptive clocking
              break;
            default:
              jtag_write(0xFA);
              jtag_write(jtag_cmd);
              break;
          }
        }
        break;

      case MPSSE_RCV_LENGTH_1:
        mpsse_length = jtag_rx_buffer[jtag_rx_pos++];
        if ((jtag_cmd & DSC_BIT_MODE) == 0U) {
          mpsse_status = MPSSE_RCV_LENGTH_2;
        }
        else {
          mpsse_status = MPSSE_TRANSMIT_BIT;
        }
        break;

      case MPSSE_RCV_LENGTH_2:
        mpsse_length |= jtag_rx_buffer[jtag_rx_pos++] << 8;
#if GOWIN_INT_FLASH_QUIRK
        if ((mpsse_length >=2000) && (jtag_cmd & DSC_READ_TDO) == 0) {
          pwm_start();
          mpsse_status = MPSSE_RUN_TEST;
        }
        else
#endif
        mpsse_status = MPSSE_TRANSMIT_BYTE;
        break;

      case MPSSE_RCV_VALUE_L:
        mpsse_value = jtag_rx_buffer[jtag_rx_pos++];
        mpsse_status = MPSSE_RCV_VALUE_H;
        break;

      case MPSSE_RCV_VALUE_H:
        mpsse_value |= jtag_rx_buffer[jtag_rx_pos++] << 8;
        delay_val = (mpsse_value + 1U) * 1000 / (12 * PIN_DELAY_NS);
        mpsse_status = MPSSE_IDLE;
        break;

      case MPSSE_TRANSMIT_BYTE:
//        cpu_global_irq_disable();

        rx_data = jtag_rx_buffer[jtag_rx_pos++];
        tx_data = 0;
        bitbang = *gpio_out;

        for (uint32_t bit = 0U; bit < 8U; ++bit) {
          uint32_t i;

          if ((jtag_cmd & DSC_LSB_FIRST) == 0U) {
            i = 7U - bit;
          }
          else {
            i = bit;
          }

          if (rx_data & (1 << i)) {
            //TDI_HIGH;
            bitbang |= TDI_PIN_MASK;
          } else {
            //TDI_LOW;
            bitbang &= ~TDI_PIN_MASK;
          }
          *gpio_out = bitbang;
          DELAY_LOW();

          //TCK_HIGH;
          bitbang |= TCK_PIN_MASK;
          *gpio_out = bitbang;
          DELAY_HIGH();

          if (TDO != 0U) {
            tx_data |= 1 << i;
          }

          //TCK_LOW;
          bitbang &= ~TCK_PIN_MASK;
          *gpio_out = bitbang;
        }

//        cpu_global_irq_enable();

        if ((jtag_cmd & DSC_READ_TDO) != 0U) {
          jtag_write(tx_data);
        }

        if (mpsse_length > 0U) {
          --mpsse_length;
        }
        else {
          mpsse_status = MPSSE_IDLE;
        }
        break;

      case MPSSE_TRANSMIT_BIT:
//        cpu_global_irq_disable();

        rx_data = jtag_rx_buffer[jtag_rx_pos++];
        tx_data = 0;
        bitbang = *gpio_out;

        if ((jtag_cmd & DSC_WRITE_TMS) != 0U) {
          output_pin_mask = TMS_PIN_MASK;
          if ((rx_data & (1 << 7)) != 0U) {
            //TDI_HIGH;
            bitbang |= TDI_PIN_MASK;
          }
          else {
            //TDI_LOW;
            bitbang &= ~TDI_PIN_MASK;
          }
        }
        else {
          output_pin_mask = TDI_PIN_MASK;
        }

        for (uint32_t bit = 0; bit <= mpsse_length; ++bit) {
          uint32_t i;

          if ((jtag_cmd & DSC_LSB_FIRST) == 0U) {
            i = 7U - bit;
          }
          else {
            i = bit;
          }

          if (rx_data & (1 << i)) {
            //TDI_HIGH;
            bitbang |= output_pin_mask;
          } else {
            //TDI_LOW;
            bitbang &= ~output_pin_mask;
          }
          *gpio_out = bitbang;
          DELAY_LOW();

          //TCK_HIGH;
          bitbang |= TCK_PIN_MASK;
          *gpio_out = bitbang;
          DELAY_HIGH();

          if (TDO != 0U) {
            tx_data |= 1 << i;
          }

          //TCK_LOW;
          bitbang &= ~TCK_PIN_MASK;
          *gpio_out = bitbang;
        };

//        cpu_global_irq_enable();

        if ((jtag_cmd & DSC_READ_TDO) != 0U) {
          jtag_write(tx_data);
        }

        mpsse_status = MPSSE_IDLE;
        break;

#if GOWIN_INT_FLASH_QUIRK
      case MPSSE_RUN_TEST:
        if (mpsse_length == 0) {
          mpsse_status = MPSSE_IDLE;
          pwm_stop();
        }
        jtag_rx_pos++;
        mpsse_length--;
        DELAY_RUN_TEST();
        break;
#endif

      default:
        mpsse_status = MPSSE_IDLE;
        break;
    }
  }

  jtag_rx_len = 0U;
  led_set(0, 0);
}
