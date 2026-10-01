/**
 * @file usbd_ftdi.c
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
#include "usbd_ftdi.h"
#include "hal_usb.h"
#include "hal_mtimer.h"
#include "bl702_usb.h"
#include "uart_interface.h"
#include "jtag_process.h"

/* USB Endpoint Number */
#define JTAG_IN_EP                    0x81
#define JTAG_OUT_EP                   0x02
#define CDC_IN_EP                     0x83
#define CDC_OUT_EP                    0x04

/* Requests */
#define SIO_RESET_REQUEST             0x00 /* Reset the port */
#define SIO_SET_MODEM_CTRL_REQUEST    0x01 /* Set the modem control register */
#define SIO_SET_FLOW_CTRL_REQUEST     0x02 /* Set flow control register */
#define SIO_SET_BAUDRATE_REQUEST      0x03 /* Set baud rate */
#define SIO_SET_DATA_REQUEST          0x04 /* Set the data characteristics of the port */
#define SIO_POLL_MODEM_STATUS_REQUEST 0x05
#define SIO_SET_EVENT_CHAR_REQUEST    0x06
#define SIO_SET_ERROR_CHAR_REQUEST    0x07
#define SIO_SET_LATENCY_TIMER_REQUEST 0x09
#define SIO_GET_LATENCY_TIMER_REQUEST 0x0A
#define SIO_SET_BITMODE_REQUEST       0x0B
#define SIO_READ_PINS_REQUEST         0x0C
#define SIO_READ_EEPROM_REQUEST       0x90
#define SIO_WRITE_EEPROM_REQUEST      0x91
#define SIO_ERASE_EEPROM_REQUEST      0x92

#define SIO_RESET_VALUE_SIO           0
#define SIO_RESET_VALUE_PURGE_RX      1
#define SIO_RESET_VALUE_PURGE_TX      2

#define SIO_DISABLE_FLOW_CTRL         0x0
#define SIO_RTS_CTS_HS                (0x1 << 8)
#define SIO_DTR_DSR_HS                (0x2 << 8)
#define SIO_XON_XOFF_HS               (0x4 << 8)

#define SIO_SET_DTR_MASK              0x1
#define SIO_SET_DTR_HIGH              ( 1 | ( SIO_SET_DTR_MASK  << 8))
#define SIO_SET_DTR_LOW               ( 0 | ( SIO_SET_DTR_MASK  << 8))
#define SIO_SET_RTS_MASK              0x2
#define SIO_SET_RTS_HIGH              ( 2 | ( SIO_SET_RTS_MASK << 8 ))
#define SIO_SET_RTS_LOW               ( 0 | ( SIO_SET_RTS_MASK << 8 ))
#define SIO_RTS_CTS_HS                (0x1 << 8)

#define FTDI_USB_CLK                  48000000

#define time_after_eq(a,b)            ((int32_t)(a) - (int32_t)(b) >= 0)

static void usbd_cdc_acm_bulk_in(uint8_t ep);
static void usbd_cdc_acm_bulk_out(uint8_t ep);
static void usbd_cdc_jtag_in(uint8_t ep);
static void usbd_cdc_jtag_out(uint8_t ep);
static uint8_t* GetLineModemStatus(void);
static int receive_to_ringbuffer(uint8_t ep, Ring_Buffer_Type *rb);
static int send_from_ringbuffer(uint8_t ep, Ring_Buffer_Type *rb);

static usbd_class_t     ftdi_class;
static usbd_interface_t ftdi_intf;

static uint8_t ftdi_modem_status[2];
static uint32_t sof_tick;
static uint8_t latency_timer;
static uint32_t latency_timeout;
static bool send_immediate;
static bool jtag_enable;

// Endpoints for JTAG
static usbd_endpoint_t jtag_in_ep = {
  .ep_addr  = JTAG_IN_EP,
  .ep_cb    = usbd_cdc_jtag_in
};

static usbd_endpoint_t jtag_out_ep = {
  .ep_addr  = JTAG_OUT_EP,
  .ep_cb    = usbd_cdc_jtag_out
};

// Endpoints for UART
static usbd_endpoint_t uart_in_ep = {
  .ep_addr  = CDC_IN_EP,
  .ep_cb    = usbd_cdc_acm_bulk_in
};

static usbd_endpoint_t uart_out_ep = {
  .ep_addr  = CDC_OUT_EP,
  .ep_cb    = usbd_cdc_acm_bulk_out
};

static const uint16_t ftdi_eeprom_info[] = {
  0x0800, 0x0403, 0x6010, 0x0500, 0x3280, 0x0000, 0x0200, 0x0E96,
  0x1AA4, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0X0000, 0x0000, 0x0000, 0x030E, 0x0053, 0x0069, 0x0070, 0x0065,
  0x0065, 0x0064, 0x031A, 0x0055, 0x0053, 0x0042, 0x0020, 0x0044,
  0x0065, 0x0062, 0x0075, 0x0067, 0x0067, 0x0065, 0x0072, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x4E1B
};

// USB -> UART out
static
void usbd_cdc_acm_bulk_out(uint8_t ep)
{
  receive_to_ringbuffer(ep, &usb_rx_rb);
}

// UART -> USB in
static
void usbd_cdc_acm_bulk_in(uint8_t ep)
{
  send_from_ringbuffer(ep, &uart1_rx_rb);
}

static
void usbd_cdc_jtag_out(uint8_t ep)
{
  receive_to_ringbuffer(ep, &jtag_rx_rb);
}

static
void usbd_cdc_jtag_in(uint8_t ep)
{
  send_from_ringbuffer(ep, &jtag_tx_rb);
}

static
void usbd_ftdi_reset(void)
{
  latency_timer = 16;  // ms
  sof_tick = 0U;
  latency_timeout = latency_timer;
  send_immediate = false;
  jtag_enable = false;

  uart_ringbuffer_init();
  uart1_init();
}

static
void ftdi_set_baudrate(uint32_t itdf_divisor, uint32_t *actual_baudrate)
{
  int baudrate;
  uint8_t frac[] = {0, 8, 4, 2, 6, 10, 12, 14};
  int divisor = itdf_divisor & 0x3fff;

  divisor <<= 4;
  divisor |= frac[(itdf_divisor >> 14) & 0x07];

  if (itdf_divisor == 0) {
    baudrate = 3000000;
  }
  else if (itdf_divisor == 1) {
    baudrate = 2000000;
  }
  else {
    baudrate = FTDI_USB_CLK / divisor;
  }

  if (baudrate > 10000 && baudrate < 12000) {
    *actual_baudrate = (baudrate - 10000) * 10000;
  } else
    *actual_baudrate = baudrate;
}

static
void usbd_ftdi_set_line_coding(uint32_t baudrate, uint8_t databits,
                                      uint8_t parity, uint8_t stopbits)
{
  uart_databits_t uart_databits;
  uart_parity_t uart_parity;
  uart_stopbits_t uart_stopbits;

  switch (databits) {
    case 5:
      uart_databits = UART_DATA_LEN_5;
      break;
    case 6:
      uart_databits = UART_DATA_LEN_6;
      break;
    case 7:
      uart_databits = UART_DATA_LEN_7;
      break;
    case 8:
    default:
      uart_databits = UART_DATA_LEN_8;
      break;
  }

  switch (parity) {
    default:
    case 0:
      uart_parity = UART_PAR_NONE;
      break;
    case 1:
      uart_parity = UART_PAR_ODD;
      break;
    case 2:
      uart_parity = UART_PAR_EVEN;
      break;
  }

  switch (stopbits) {
    default:
    case 0:
      uart_stopbits = UART_STOP_ONE;
      break;
    case 1:
      uart_stopbits = UART_STOP_ONE_D_FIVE;
      break;
    case 2:
      uart_stopbits = UART_STOP_TWO;
      break;
  }

  uart1_config(baudrate, uart_databits, uart_parity, uart_stopbits);
}

static
void usbd_ftdi_set_dtr(bool dtr)
{

}

static
void usbd_ftdi_set_rts(bool rts)
{

}

static
int ftdi_vendor_request_handler(struct usb_setup_packet *pSetup,
                                       uint8_t **data, uint32_t *len)
{
  static uint32_t actual_baudrate = 1200;

  switch (pSetup->bRequest) {
    case SIO_RESET_REQUEST:
      switch (pSetup->wValueL) {
        case SIO_RESET_VALUE_SIO:
          usbd_ftdi_reset();
          break;
        case SIO_RESET_VALUE_PURGE_RX:
          Ring_Buffer_Reset(&jtag_rx_rb);
          break;
        case SIO_RESET_VALUE_PURGE_TX:
          Ring_Buffer_Reset(&jtag_tx_rb);
          break;
      }
      break;

    case SIO_SET_MODEM_CTRL_REQUEST:
      if (pSetup->wValue == SIO_SET_DTR_HIGH) {
        usbd_ftdi_set_dtr(true);
      } else if (pSetup->wValue == SIO_SET_DTR_LOW) {
        usbd_ftdi_set_dtr(false);
      } else if (pSetup->wValue == SIO_SET_RTS_HIGH) {
        usbd_ftdi_set_rts(true);
      } else if (pSetup->wValue == SIO_SET_RTS_LOW) {
        usbd_ftdi_set_rts(false);
      }
      break;

    case SIO_SET_FLOW_CTRL_REQUEST:

      break;

    case SIO_SET_BAUDRATE_REQUEST:
      ftdi_set_baudrate(pSetup->wValue | (pSetup->wIndexH << 16), &actual_baudrate);
      usbd_ftdi_set_line_coding(actual_baudrate, 8, 0, 0);
      break;

    case SIO_SET_DATA_REQUEST:
      /**
       * D0-D7 databits  BITS_7=7, BITS_8=8
       * D8-D10 parity  NONE=0, ODD=1, EVEN=2, MARK=3, SPACE=4
       * D11-D12 		STOP_BIT_1=0, STOP_BIT_15=1, STOP_BIT_2=2
       * D14  		BREAK_OFF=0, BREAK_ON=1
       **/
      usbd_ftdi_set_line_coding(actual_baudrate,
                                pSetup->wValueL,
                                pSetup->wValueH & 0x07,
                                (pSetup->wValueH >> 3) & 0x03);
      break;

    case SIO_POLL_MODEM_STATUS_REQUEST:
      /*     Poll modem status information

       This function allows the retrieve the two status bytes of the device.
       The device sends these bytes also as a header for each read access
       where they are discarded by ftdi_read_data(). The chip generates
       the two stripped status bytes in the absence of data every 40 ms.

       Layout of the first byte:
       - B0..B3 - must be 0
       - B4       Clear to send (CTS)
       0 = inactive
       1 = active
       - B5       Data set ready (DTS)
       0 = inactive
       1 = active
       - B6       Ring indicator (RI)
       0 = inactive
       1 = active
       - B7       Receive line signal detect (RLSD)
       0 = inactive
       1 = active

       Layout of the second byte:
       - B0       Data ready (DR)
       - B1       Overrun error (OE)
       - B2       Parity error (PE)
       - B3       Framing error (FE)
       - B4       Break interrupt (BI)
       - B5       Transmitter holding register (THRE)
       - B6       Transmitter empty (TEMT)
       - B7       Error in RCVR FIFO */
      *data = GetLineModemStatus();
      *len  = sizeof(ftdi_modem_status);
      break;

    case SIO_SET_EVENT_CHAR_REQUEST:

      break;

    case SIO_SET_ERROR_CHAR_REQUEST:

      break;

    case SIO_SET_LATENCY_TIMER_REQUEST:
      latency_timer = pSetup->wValueL;
      break;

    case SIO_GET_LATENCY_TIMER_REQUEST:
      *data = &latency_timer;
      *len = sizeof(latency_timer);
      break;

    case SIO_SET_BITMODE_REQUEST:
      switch (pSetup->wValueH) {
        case 0x00:
          jtag_enable = false;
          break;

        case 0x02U:
          jtag_ringbuffer_init();
          jtag_init();
          jtag_enable = true;
          break;
      }
      break;

    case SIO_READ_EEPROM_REQUEST:
      *data = (uint8_t*)&ftdi_eeprom_info[pSetup->wIndexL];
      *len  = sizeof(ftdi_eeprom_info[0]);
      break;

    default:
      return (-1);
  }

  return (0);
}
static
void ftdi_notify_handler(uint8_t event, void *arg)
{
  switch (event) {
    case USB_EVENT_RESET:
      usbd_ftdi_reset();
      break;
    case USB_EVENT_SOF:
      sof_tick++;
      break;
    default:
      break;
  }
}

static
uint8_t* GetLineModemStatus(void)
{
  ftdi_modem_status[0] = 0x01;

  if (Ring_Buffer_Get_Status(&jtag_rx_rb) == RING_BUFFER_EMPTY) {
    ftdi_modem_status[1] = 0x60;
  }
  else {
    ftdi_modem_status[1] = 0x00;
  }

  return (ftdi_modem_status);
}

static
int receive_to_ringbuffer(uint8_t ep, Ring_Buffer_Type *rb)
{
  uint8_t ep_idx;
  uint32_t recv_len;
  uint32_t timeout = 0x00FFFFFF;

  /* Check if OUT ep */
  if (USB_EP_GET_DIR(ep) != USB_EP_DIR_OUT) {
    return (-USB_DC_EP_DIR_ERR);
  }

  ep_idx = USB_EP_GET_IDX(ep);

  while (!USB_Is_EPx_RDY_Free(ep_idx)) {
    timeout--;
    if (!timeout) {
      return (-USB_DC_EP_TIMEOUT_ERR);
    }
  }

  recv_len = USB_Get_EPx_RX_FIFO_CNT(ep_idx);

  /*if rx fifo count equal 0,it means last is send nack and ringbuffer is smaller than 64,
   * so,if ringbuffer is larger than 64,set ack to recv next data.
   */
  if (recv_len == 0U) {
    if (Ring_Buffer_Get_Empty_Length(rb) >= USB_FS_MAX_PACKET_SIZE) {
      USB_Set_EPx_Rdy(ep_idx);
    }
  } else {
    uint32_t addr = USB_BASE + 0x11C + (ep_idx - 1) * 0x10;
    Ring_Buffer_Write_Callback(rb, recv_len, fifocopy_to_mem, (void*)addr);

    if (Ring_Buffer_Get_Empty_Length(rb) < USB_FS_MAX_PACKET_SIZE) {
      return (-USB_DC_RB_SIZE_SMALL_ERR);
    }

    USB_Set_EPx_Rdy(ep_idx);
  }

  return (USB_DC_OK);
}

static
int send_from_ringbuffer(uint8_t ep, Ring_Buffer_Type *rb)
{
  uint8_t ep_idx;
  static bool zlp_flag = false;
  uint32_t timeout = 0x00FFFFFF;

  ep_idx = USB_EP_GET_IDX(ep);

  /* Check if IN ep */
  if (USB_EP_GET_DIR(ep) != USB_EP_DIR_IN) {
    return (-USB_DC_EP_DIR_ERR);
  }

  while (!USB_Is_EPx_RDY_Free(ep_idx)) {
    timeout--;
    if (!timeout) {
      return (-USB_DC_EP_TIMEOUT_ERR);
    }
  }

  if (zlp_flag == true) {
    zlp_flag = false;
    USB_Set_EPx_Rdy(ep_idx);
    return (-USB_DC_ZLP_ERR);
  }

  if (!USB_Get_EPx_TX_FIFO_Status(ep_idx, USB_FIFO_EMPTY)) {
    return (-USB_DC_RB_SIZE_SMALL_ERR);
  }

  uint32_t addr = USB_BASE + 0x118 + (ep_idx - 1) * 0x10;

  if ((Ring_Buffer_Get_Length(rb) >= USB_FS_MAX_PACKET_SIZE - sizeof(ftdi_modem_status)) ||
      time_after_eq(sof_tick, latency_timeout) || (send_immediate == true)) {
    memcopy_to_fifo((void *)addr,
                    GetLineModemStatus(),
                    sizeof(ftdi_modem_status));
    Ring_Buffer_Read_Callback(rb,
                              USB_FS_MAX_PACKET_SIZE - sizeof(ftdi_modem_status),
                              memcopy_to_fifo,
                              (void *)addr);

    if (Ring_Buffer_Get_Length(rb) == 0U && USB_Get_EPx_TX_FIFO_Status(ep_idx, USB_FIFO_FULL)) {
      zlp_flag = true;
    }

    USB_Set_EPx_Rdy(ep_idx);
    latency_timeout = sof_tick + latency_timer;
    send_immediate = false;
  }
  else {
    return (-USB_DC_RB_SIZE_SMALL_ERR);
  }

  return (USB_DC_OK);
}

void usbd_ftdi_init(void)
{
  ftdi_intf.class_handler = NULL;
  ftdi_intf.custom_handler = NULL;
  ftdi_intf.notify_handler = ftdi_notify_handler;
  ftdi_intf.vendor_handler = ftdi_vendor_request_handler;

  usbd_class_register(&ftdi_class);
  usbd_class_add_interface(&ftdi_class, &ftdi_intf);
  usbd_interface_add_endpoint(&ftdi_intf, &jtag_out_ep);
  usbd_interface_add_endpoint(&ftdi_intf, &jtag_in_ep);
  usbd_interface_add_endpoint(&ftdi_intf, &uart_out_ep);
  usbd_interface_add_endpoint(&ftdi_intf, &uart_in_ep);
}

void usbd_ftdi_process(void)
{
  uart_send_from_ringbuffer();

  if (jtag_enable == true) {
    jtag_process();
  }
}

void usbd_ftdi_send_immediate(void)
{
  send_immediate = true;
}
