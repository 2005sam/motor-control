#include "contral_DR16.h"
#include "stm32f4xx_hal.h"
#include "CMSIS_os.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
// #include "motor_rm3508"

// this file is used to get information from DR16 and process it
// uint8_t rx_Buffer[18];
UART_HandleTypeDef *huart_local;
 SemaphoreHandle_t xBinarySemaphorel;

uint8_t rx_Buffer[18];
//SemaphoreHandle_t temp;
int16_t result;

struct ControlDR16Data processed_data;

xQueueHandle control_dr16_queue_isr;
void ControlDR16Process(void *argument);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  //result = (((int16_t)rx_Buffer[0] | (int16_t)rx_Buffer[1] << 8) & 0x07FF - 1024);
  xSemaphoreGiveFromISR(xBinarySemaphorel, NULL);
  // MotorRm3508Set(0, result);
  // HAL_UART_Receive_IT(&huart3, rx_Buffer, 18);
}
/*
void DR16Process(void *argument)
{
  while (1)
  {
    xSemaphoreTake(temp, portMAX_DELAY);

    //MotorRm3508Set(0, result);
    HAL_UART_Receive_IT(huart_local, rx_Buffer, 18);
  }
}
*/
void ContralDR16InitTask(void *argument);

void ControlDR16Init(UART_HandleTypeDef *param_huart)
{
  huart_local = param_huart;
  xTaskCreate(ContralDR16InitTask, "ContralDR16InitTask", 512, NULL, 0, NULL);
  // HAL_UART_Receive_IT(&huart, rx_Buffer, 18);
  //  Initialize the UART for receiving data from DR16
  //  HAL_UART_Receive_IT(&huart, rx_Buffer, 18);
  //  xBinarySemaphorel = xSemaphoreCreateBinary();
  //  xTaskCreate(ControlDR16Process, "ControlDR16Process", 512, NULL, 1, NULL);
}

void ContralDR16InitTask(void *argument)
{
  xBinarySemaphorel = xSemaphoreCreateBinary();
  HAL_UART_Receive_IT(huart_local, rx_Buffer, 18);
	xTaskCreate(ControlDR16Process, "ControlDR16Process", 512, NULL, 1, NULL);
  vTaskDelete(NULL);
}


void ControlDR16Process(void *argument)
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
      HAL_UART_Receive_IT(huart_local, rx_Buffer, 18);

  }
}


void ControlDR16GetValue(struct ControlDR16Data *data)
{
  *data = processed_data;
}

/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    processed_data.ch0 = ((int16_t)rx_Buffer[0] | ((int16_t)rx_Buffer[1] << 8)) & 0x07FF;
    xSemaphoreGive(xBinarySemaphorel);
    //HAL_UART_Receive_IT(huart, rx_Buffer, 18);
    portYIELD_FROM_ISR(pdTRUE);
  }
}

*/