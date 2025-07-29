// Copyright (C) 2025 b2(shengpengxiang1@outlook.com)
// This This program is free software:
// you can redistribute it and/or modify it under the terms of the
// GNU General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.

#include "contral_DR16.h"
#include "stm32f4xx_hal.h"
#include "CMSIS_os.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

struct ControlDR16Data processed_data;

void ContralDR16GetOrigin(uint8_t rx_Buffer[18])
{
  while (1)
  {
    xSemaphoreGiveFromISR(xBinarySemaphorel, NULL);

    // Process the received data

    processed_data.ch0 = ((int16_t)rx_Buffer[0] | ((int16_t)rx_Buffer[1] << 8)) & 0x07FF;
    processed_data.ch1 = (((int16_t)rx_Buffer[1] >> 3) | ((int16_t)rx_Buffer[2] << 5)) & 0x07FF;
    processed_data.ch2 = (((int16_t)rx_Buffer[2] >> 6) | ((int16_t)rx_Buffer[3] << 2) | ((int16_t)rx_Buffer[4] << 10)) & 0x07FF;
    processed_data.ch3 = (((int16_t)rx_Buffer[4] >> 1) | ((int16_t)rx_Buffer[5] << 7)) & 0x07FF;
    processed_data.s1 = ((rx_Buffer[5] >> 4) & 0x000C) >> 2;
    processed_data.s2 = ((rx_Buffer[5] >> 4) & 0x0003);
    processed_data.mouse_x_axis = ((int16_t)rx_Buffer[6]) | ((int16_t)rx_Buffer[7] << 8);
    processed_data.mouse_y_axis = ((int16_t)rx_Buffer[8]) | ((int16_t)rx_Buffer[9] << 8);
    processed_data.mouse_z_axis = ((int16_t)rx_Buffer[10]) | ((int16_t)rx_Buffer[11] << 8);
    processed_data.mouse_button_L = rx_Buffer[12];
    processed_data.mouse_button_R = rx_Buffer[13];
    processed_data.buttons = ((int16_t)rx_Buffer[14]) | ((int16_t)rx_Buffer[15] << 8);
    // processed_data.reserved = control_dr16_data.data2 & 0xFFFF;
  }
}

void ControlDR16GetValue(struct ControlDR16Data *data)
{
  *data = processed_data;
}
