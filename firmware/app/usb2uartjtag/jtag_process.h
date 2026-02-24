/*
 * Copyright (C) 2026 Sergey Koshkin <koshkin.sergey@gmail.com>
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef JTAG_PROCESS_H_
#define JTAG_PROCESS_H_

#include "ring_buffer.h"

extern Ring_Buffer_Type jtag_tx_rb;
extern Ring_Buffer_Type jtag_rx_rb;

void jtag_process(void);
void jtag_ringbuffer_init(void);
void jtag_gpio_init(void);

#endif /* JTAG_PROCESS_H_ */
