#include "contral_DR16.h"
// this file is used to get information from DR16 and process it
uint8_t rxBuffer[18];
UART_HandleTypeDef huart;
QueueHandle_t control_dr16_queue_isr;
struct ControlDR16Data processed_data;
struct PreControlDR16Data
{
  uint64_t data0;
  uint64_t data1;
  uint16_t data2;
};
void ControlDR16Process(void *argument);

void ControlDR16Init(UART_Typedef *param_huart)
{
  huart = *param_huart;
  // Initialize the UART for receiving data from DR16
  HAL_UART_Receive_IT(&huart, (uint8_t *)rxBuffer, sizeof(rxBuffer));
  // Create a queue to hold the received data
  control_dr16_queue_isr = xQueueCreate(2, sizeof(struct PreControlDR16Data));
  xTaskCreate(ControlDR16Process, "ControlDR16Process", 256, NULL, 1, NULL);
}

void ControlDR16Process(void *argument)
{
  struct PreControlDR16Data control_dr16_data;
  while (1)
  {
    xQueueReceive(control_dr16_queue_isr, &control_dr16_data, portMAX_DELAY);
    // Process the received data

    processed_data.ch0 = control_dr16_data.data0 & 0x007F;
    processed_data.ch1 = (control_dr16_data.data0 >> 11) & 0x007F;
    processed_data.ch2 = (control_dr16_data.data0 >> 22) & 0x007F;
    processed_data.ch3 = (control_dr16_data.data0 >> 33) & 0x007F;
    processed_data.s1 = (control_dr16_data.data0 >> 44) & 0x01;
    processed_data.s2 = (control_dr16_data.data0 >> 46) & 0x01;
    processed_data.mouse_x_axis = (control_dr16_data.data0 >> 48) & 0xFFFF;
    processed_data.mouse_y_axis = (control_dr16_data.data1 & 0xFFFF);
    processed_data.mouse_z_axis = (control_dr16_data.data1 >> 16) & 0xFFFF;
    processed_data.mouse_button_L = (control_dr16_data.data1 >> 32) & 0x00FF;
    processed_data.mouse_button_R = (control_dr16_data.data1 >> 40) & 0x00FF;
    processed_data.buttons = (control_dr16_data.data1 >> 48) & 0xFFFF;
    processed_data.reserved = control_dr16_data.data2 & 0xFFFF;
  }
}

void ControlDR16GetValue(struct ControlDR16Data *data)
{
  *data = processed_data;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    struct PreControlDR16Data control_dr16_data;
    control_dr16_data.data0 = rx_Buffer[0] |
                              rx_Buffer[1] << 8 |
                              rx_Buffer[2] << 16 |
                              rx_Buffer[3] << 24 |
                              rx_Buffer[4] << 32 |
                              rx_Buffer[5] << 40 |
                              rx_Buffer[6] << 48 |
                              rx_Buffer[7] << 56;
    control_dr16_data.data1 = rx_Buffer[8] |
                              rx_Buffer[9] << 8 |
                              rx_Buffer[10] << 16 |
                              rx_Buffer[11] << 24 |
                              rx_Buffer[12] << 32 |
                              rx_Buffer[13] << 40 |
                              rx_Buffer[14] << 48 |
                              rx_Buffer[15] << 56;
    control_dr16_data.data2 = rx_Buffer[16] | (rx_Buffer[17] << 8);
    xQueueSendFromISR(control_dr16_queue_isr, control_dr16_data, NULL);
    HAL_UART_Receive_IT(&huart, (uint8_t *)rxBuffer, sizeof(rxBuffer));
  }
}
