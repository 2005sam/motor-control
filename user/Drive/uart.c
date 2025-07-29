#include "uart.h"
#include "contral_DR16.h"
uint8_t rx_Buffer[18];
extern UART_HandleTypeDef huart3;
SemaphoreHandle_t xBinarySemaphorelUart3;
void ControlDR16Init(void);

void ControlDR16Init(void)
{
  xBinarySemaphorelUart3 = xSemaphoreCreateBinary();
  HAL_UART_Receive_IT(&huart3, rx_Buffer, 18);
  1 xTaskCreate(Uart3CallbackProcess, "Uart3CallbackProcess", 64, NULL, 1, NULL);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (*huart == huart3)
    xSemaphoreGiveFromISR(xBinarySemaphorelUart3, NULL);
}

void Uart3CallbackProcess(void *argument)
{
  xSemaphoreTake(xBinarySemaphorelUart3, 0);
  ContralDR16GetOrigin(rx_Buffer);
  HAL_UART_Receive_IT(&huart3, rx_Buffer, 18);
}
