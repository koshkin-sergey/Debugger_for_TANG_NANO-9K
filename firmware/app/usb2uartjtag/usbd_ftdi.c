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

#include "usbd_ftdi.h"
#include "hal_usb.h"
#include "hal_mtimer.h"
#include "bl702_usb.h"

static const uint8_t ftdi_modem_status[2] = {0x01, 0x60};
static volatile uint32_t sof_tick;
static uint8_t Latency_Timer;
static volatile uint32_t last_send;

const char *stop_name[] = {"1", "1.5", "2"};
const char *parity_name[] = {"N", "O", "E", "M", "S"};

static const uint16_t ftdi_eeprom_info[] = {
  0x0800, 0x0403, 0x6010, 0x0500, 0x3280, 0x0000, 0x0200, 0x1096,
  0x1aa6, 0x0000, 0x0046, 0x0310, 0x004f, 0x0070, 0x0065, 0x006e,
  0x002d, 0x0045, 0x0043, 0x031a, 0x0055, 0x0053, 0x0042, 0x0020,
  0x0044, 0x0065, 0x0062, 0x0075, 0x0067, 0x0067, 0x0065, 0x0072,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x1027
};

static void usbd_ftdi_reset(void)
{
  Latency_Timer = 16;  // ms
  sof_tick = 0;
}

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

#define SIO_DISABLE_FLOW_CTRL 0x0
#define SIO_RTS_CTS_HS (0x1 << 8)
#define SIO_DTR_DSR_HS (0x2 << 8)
#define SIO_XON_XOFF_HS (0x4 << 8)

#define SIO_SET_DTR_MASK 0x1
#define SIO_SET_DTR_HIGH ( 1 | ( SIO_SET_DTR_MASK  << 8))
#define SIO_SET_DTR_LOW  ( 0 | ( SIO_SET_DTR_MASK  << 8))
#define SIO_SET_RTS_MASK 0x2
#define SIO_SET_RTS_HIGH ( 2 | ( SIO_SET_RTS_MASK << 8 ))
#define SIO_SET_RTS_LOW ( 0 | ( SIO_SET_RTS_MASK << 8 ))

#define SIO_RTS_CTS_HS (0x1 << 8)

#define FTDI_USB_CLK 48000000

static void ftdi_set_baudrate(uint32_t itdf_divisor, uint32_t *actual_baudrate)
{
  int baudrate;
  uint8_t frac[] = {0, 8, 4, 2, 6, 10, 12, 14};
  int divisor = itdf_divisor & 0x3fff;
  divisor <<= 4;
  divisor |= frac[(itdf_divisor >> 14) & 0x07];

  if (itdf_divisor == 0x01) {
    baudrate = 2000000;
  } else if (itdf_divisor == 0x00) {
    baudrate = 3000000;
  } else {
    baudrate = FTDI_USB_CLK / divisor;
  }
  if (baudrate > 10000 && baudrate < 12000) {
    *actual_baudrate = (baudrate - 10000) * 10000;
  } else
    *actual_baudrate = baudrate;
}

//static char datatmp[2]={0x32, 0x60};
static int ftdi_vendor_request_handler(struct usb_setup_packet *pSetup,
                                       uint8_t **data, uint32_t *len)
{
  static uint32_t actual_baudrate = 1200;

  switch (pSetup->bRequest) {
    case SIO_READ_EEPROM_REQUEST:
      *data = (uint8_t*)&ftdi_eeprom_info[pSetup->wIndexL];
      *len  = sizeof(ftdi_eeprom_info[0]);
      break;

    case SIO_RESET_REQUEST:
//      usbd_ftdi_reset();
      break;

    case SIO_SET_MODEM_CTRL_REQUEST:
      if (pSetup->wValue == SIO_SET_DTR_HIGH) {
        //USBD_LOG("DTR 1\r\n");
        usbd_ftdi_set_dtr(true);
      } else if (pSetup->wValue == SIO_SET_DTR_LOW) {
        //USBD_LOG("DTR 0\r\n");
        usbd_ftdi_set_dtr(false);
      } else if (pSetup->wValue == SIO_SET_RTS_HIGH) {
        //USBD_LOG("RTS 1\r\n");
        usbd_ftdi_set_rts(true);
      } else if (pSetup->wValue == SIO_SET_RTS_LOW) {
        //USBD_LOG("RTS 0\r\n");
        usbd_ftdi_set_rts(false);
      }
      break;

    case SIO_SET_FLOW_CTRL_REQUEST:

      break;

    case SIO_SET_BAUDRATE_REQUEST: //wValue，2个字节波特率
    {
      uint8_t baudrate_high = (pSetup->wIndex >> 8);
      ftdi_set_baudrate(pSetup->wValue | (baudrate_high << 16),
          &actual_baudrate);
      if (actual_baudrate != 1200) {
        usbd_ftdi_set_line_coding(actual_baudrate, 8, 0, 0);
      }
      break;
    }

    case SIO_SET_DATA_REQUEST:
      /**
       * D0-D7 databits  BITS_7=7, BITS_8=8
       * D8-D10 parity  NONE=0, ODD=1, EVEN=2, MARK=3, SPACE=4
       * D11-D12 		STOP_BIT_1=0, STOP_BIT_15=1, STOP_BIT_2=2
       * D14  		BREAK_OFF=0, BREAK_ON=1
       **/
      if (actual_baudrate != 1200) {
        //USBD_LOG("CDC_SET_LINE_CODING <%d %d %s %s>\r\n",actual_baudrate,(uint8_t)pSetup->wValue,parity_name[(uint8_t)(pSetup->wValue>>8)],stop_name[(uint8_t)(pSetup->wValue>>11)]);
        usbd_ftdi_set_line_coding(actual_baudrate, (uint8_t)pSetup->wValue,
            (uint8_t)(pSetup->wValue >> 8), (uint8_t)(pSetup->wValue >> 11));
      }
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
      *data = (uint8_t *)&ftdi_modem_status[0];
      *len  = sizeof(ftdi_modem_status);
      break;

    case SIO_SET_EVENT_CHAR_REQUEST:

      break;

    case SIO_SET_ERROR_CHAR_REQUEST:

      break;

    case SIO_SET_LATENCY_TIMER_REQUEST:
      Latency_Timer = pSetup->wValueL;
      break;

    case SIO_GET_LATENCY_TIMER_REQUEST:
      *data = &Latency_Timer;
      *len = sizeof(Latency_Timer);
      break;

    case SIO_SET_BITMODE_REQUEST:

      break;

    default:
      USBD_LOG_DBG("CDC ACM request 0x%x, value 0x%x\r\n",
          pSetup->bRequest, pSetup->wValue);
      return (-1);
  }

  return (0);
}
static void ftdi_notify_handler(uint8_t event, void *arg)
{
  switch (event) {
    case USB_EVENT_RESET:
      usbd_ftdi_reset();
      break;
    case USB_EVENT_SOF:
      sof_tick++;
      USBD_LOG_DBG("tick: %d\r\n", sof_tick);
      break;
    default:
      break;
  }
}

__weak void usbd_ftdi_set_line_coding(uint32_t baudrate, uint8_t databits,
    uint8_t parity, uint8_t stopbits)
{

}
__weak void usbd_ftdi_set_dtr(bool dtr)
{

}
__weak void usbd_ftdi_set_rts(bool rts)
{

}

void usbd_ftdi_add_interface(usbd_class_t *class, usbd_interface_t *intf)
{
  static usbd_class_t *last_class = NULL;

  if (last_class != class) {
    last_class = class;
    usbd_class_register(class);
  }

  intf->class_handler = NULL;
  intf->custom_handler = NULL;
  intf->vendor_handler = ftdi_vendor_request_handler;
  intf->notify_handler = ftdi_notify_handler;
  usbd_class_add_interface(class, intf);
}

int usbd_ftdi_receive_to_ringbuffer(uint8_t ep, Ring_Buffer_Type *rb)
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

int usbd_ftdi_send_from_ringbuffer(uint8_t ep, Ring_Buffer_Type *rb)
{
  uint8_t ep_idx;
  static bool zlp_flag = false;
  static uint32_t send_total_len = 0;
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
    send_total_len = 0;
    USB_Set_EPx_Rdy(ep_idx);
    return (-USB_DC_ZLP_ERR);
  }

  if (USB_Get_EPx_TX_FIFO_CNT(ep_idx) != USB_FS_MAX_PACKET_SIZE) {
    return (-USB_DC_RB_SIZE_SMALL_ERR);
  }

  uint32_t addr = USB_BASE + 0x118 + (ep_idx - 1) * 0x10;

  if ((Ring_Buffer_Get_Length(rb) == USB_FS_MAX_PACKET_SIZE - sizeof(ftdi_modem_status)) ||
      (sof_tick - last_send >= Latency_Timer)) {
    memcopy_to_fifo((void *)addr, (uint8_t *)&ftdi_modem_status[0], sizeof(ftdi_modem_status));
    send_total_len += sizeof(ftdi_modem_status);
    send_total_len += Ring_Buffer_Read_Callback(rb,
                                                USB_FS_MAX_PACKET_SIZE - sizeof(ftdi_modem_status),
                                                memcopy_to_fifo,
                                                (void *)addr);

    if (Ring_Buffer_Get_Length(rb) == 0U && (send_total_len % USB_FS_MAX_PACKET_SIZE) == 0U) {
      zlp_flag = true;
    }

    USB_Set_EPx_Rdy(ep_idx);
    last_send = sof_tick;
  }
  else {
    return (-USB_DC_RB_SIZE_SMALL_ERR);
  }

  return (USB_DC_OK);
}
