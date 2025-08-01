#ifndef CONTROL_DR16_H
#define CONTROL_DR16_H
#include "stm32f4xx_hal.h"
#include "CMSIS_os.h"
#include "FreeRTOS.h"

//this struct is used to hold the processed data from DR16
//ch0-3 reflect the control channel 1-4,range from 364 to 1684
//s1, s2 reflect the control transmitter s1, s2,1-up,2-down,3-middle
//mouse_x_axis, mouse_y_axis, mouse_z_axis reflect the mouse speed in x, y, z axis
//range from -32768 to 32767
//mouse_button_L, mouse_button_R, reflect the mouse button state
//0 is not pressed, 1 is pressed
//button reflect to W, A, S, D, Q, E, R, F, G, H in the keyboard
struct ControlDR16Data
{
  uint16_t ch0;
  uint16_t ch1;
  uint16_t ch2;
  uint16_t ch3;
  char s1;
  char s2;
  int16_t mouse_x_axis;
  int16_t mouse_y_axis;
  int16_t mouse_z_axis;
  char mouse_button_L;
  char mouse_button_R;
  uint16_t buttons;
  uint16_t reserved;
};

void ControlDR16Init(UART_HandleTypeDef *huart);
void ControlDR16GetValue(struct ControlDR16Data *data);



#endif
