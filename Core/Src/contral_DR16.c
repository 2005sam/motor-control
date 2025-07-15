#include "contral_DR16.h"
// this file is used to get information from DR16 and process it
uint8_t rxBuffer[18];
UART_HandleTypeDef huart;
QueueHandle_t control_dr16_queue_isr;
struct PreControlDR16Data
{
    uint64_t data0;
    uint64_t data1;
    uint16_t data2;
};
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
    char mouse_button_M;
    uint16_t buttons;
    uin
};

void ControlDR16Init(void)
{
    // Initialize the UART peripheral
    huart.Instance = USART3;
    huart.Init.BaudRate = 115200;
    huart.Init.WordLength = UART_WORDLENGTH_8B;
    huart.Init.StopBits = UART_STOPBITS_1;
    huart.Init.Parity = UART_PARITY_NONE;
    huart.Init.Mode = UART_MODE_TX_RX;
    huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart);
    HAL_UART_Receive_IT(&huart, (uint8_t *)rxBuffer, sizeof(rxBuffer));

    // Create a queue to hold the received data
    control_dr16_queue_isr = xQueueCreate(2, sizeof(struct PreControlDR16Data));
}

void ControlDR16Process(void *argument)
{
    struct PreControlDR16Data control_dr16_data;
    while (1)
    {
        xQueueReceive(control_dr16_queue_isr, &control_dr16_data, portMAX_DELAY);
        // Process the received data
    }

    void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
    {
        if (huart->Instance == USART3)
        {
            struct PreControlDR16Data control_dr16_data;
            control_dr16_data.data0 = rxBuffer[0] || rxBuffer[1] << 8 || rxBuffer[2] << 16 || rxBuffer[3] << 24;
            control_dr16_data.data1 = rxBuffer[4] || rxBuffer[5] << 8 || rxBuffer[6] << 16 || rxBuffer[7] << 24;
            control_dr16_data.data2 = rxBuffer[16] || rxBuffer[17] << 8;
            xQueueSendFromISR(control_dr16_queue_isr, control_dr16_data, NULL);
            HAL_UART_Receive_IT(&huart, (uint8_t *)rxBuffer, sizeof(rxBuffer));
        }
    }