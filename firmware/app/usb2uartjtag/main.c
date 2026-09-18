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

#include "hal_usb.h"
#include "usbd_core.h"
#include "usbd_ftdi.h"
#include "usb_descriptor.h"
#include "uart_interface.h"
#include "jtag_process.h"
#include "bl702_ef_ctrl.h"
#include "bl702_usb.h"
#include "bl702_glb.h"
#include "hal_gpio.h"
#include "io_cfg.h"

/*
 UART:
 RXD	-> ringbuffer -> usbd_cdc_acm_bulk_in -> CDC_IN_EP
 UART <---------------------------------------------------> USB
 TXD <- ringbuffer <- usbd_cdc_acm_bulk_out<- CDC_OUT_EP

 JTAG:
 jtag_rx_buffer[jtag_rx_pos] -> jtag_cmd -> mpsse status machine
 MPSSE_TRANSMIT_BYTE/BIT MSB/LSB MPSSE_TMS_OUT
 bitbang simulate clk rate about 5MHz
 */

extern struct device* usb_dc_init(void);

static usbd_class_t     cdc_class0;
static usbd_interface_t cdc_data_intf0;
static usbd_class_t     cdc_class1;
static usbd_interface_t cdc_data_intf1;
static struct device    *usb_fs;

/************************  led ctrl functions  ************************/
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

/************************  API for usbd_ftdi  ************************/
// USB -> UART out
static
void usbd_cdc_acm_bulk_out(uint8_t ep)
{
  usbd_ftdi_receive_to_ringbuffer(ep, &usb_rx_rb);
}

// UART -> USB in
static
void usbd_cdc_acm_bulk_in(uint8_t ep)
{
  usbd_ftdi_send_from_ringbuffer(ep, &uart1_rx_rb);
}

static
void usbd_cdc_jtag_out(uint8_t ep)
{
  usbd_ftdi_receive_to_ringbuffer(ep, &jtag_rx_rb);
}

static
void usbd_cdc_jtag_in(uint8_t ep)
{
  usbd_ftdi_send_from_ringbuffer(ep, &jtag_tx_rb);
}

/************************  endpoint definition  ************************/
//For UART
usbd_endpoint_t cdc_out_ep1 = {
  .ep_addr  = CDC_OUT_EP,
  .ep_cb    = usbd_cdc_acm_bulk_out
};

usbd_endpoint_t cdc_in_ep1 = {
  .ep_addr  = CDC_IN_EP,
  .ep_cb    = usbd_cdc_acm_bulk_in
};

//For JTAG
usbd_endpoint_t cdc_out_ep0 = {
  .ep_addr  = JTAG_OUT_EP,
  .ep_cb    = usbd_cdc_jtag_out
};

usbd_endpoint_t cdc_in_ep0 = {
  .ep_addr  = JTAG_IN_EP,
  .ep_cb    = usbd_cdc_jtag_in
};

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

  usbd_ftdi_add_interface(&cdc_class0, &cdc_data_intf0);
  usbd_interface_add_endpoint(&cdc_data_intf0, &cdc_out_ep0);
  usbd_interface_add_endpoint(&cdc_data_intf0, &cdc_in_ep0);

  usbd_ftdi_add_interface(&cdc_class1, &cdc_data_intf1);
  usbd_interface_add_endpoint(&cdc_data_intf1, &cdc_out_ep1);
  usbd_interface_add_endpoint(&cdc_data_intf1, &cdc_in_ep1);

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
    uart_send_from_ringbuffer();
    jtag_process();
  }

  return (0);
}
