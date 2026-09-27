/**
 * @file main.c
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

#include "usbd_core.h"
#include "hal_usb.h"
#include "usbd_ftdi.h"
#include "usb_descriptor.h"
#include "bl702_ef_ctrl.h"
#include "bl702_glb.h"
#include "hal_gpio.h"
#include "io_cfg.h"

extern struct device* usb_dc_init(void);

static struct device* usb_fs;

static
void led_gpio_init(void)
{
  gpio_set_mode(LED0_PIN, GPIO_OUTPUT_MODE);
}

void led_set(uint8_t status)
{
  gpio_write(LED0_PIN, !status);
}

void led_toggle(void)
{
  gpio_toggle(LED0_PIN);
}

int main(void)
{
  uint8_t chipid[8];

  /* disable debug for uart0 */
  bflb_platform_print_set(1);
  GLB_Select_Internal_Flash();
  bflb_platform_init(0);
  led_gpio_init();
  led_set(1);
  EF_Ctrl_Read_Chip_ID(chipid);
  usb_descriptor_register(chipid);

  usbd_ftdi_init();

  usb_fs = usb_dc_init();
  if (usb_fs) {
    device_control(usb_fs,
                   DEVICE_CTRL_SET_INT,
                   (void*)(USB_SOF_IT           |
                           USB_EP1_DATA_IN_IT   |
                           USB_EP2_DATA_OUT_IT  |
                           USB_EP3_DATA_IN_IT   |
                           USB_EP4_DATA_OUT_IT));
  }

  while (!usb_device_is_configured()) {
    __NOP();
  };

  led_set(0);

  for (;;) {
    usbd_ftdi_process();
  }

  return (0);
}
