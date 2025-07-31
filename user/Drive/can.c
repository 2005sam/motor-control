#include "can.h"
#include "freertos.h"
#include "cmsis_os.h"
#include "motor_rm3508.h"
#include "queue.h"

// Global variables for CAN communication

extern CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef tx_header;
uint32_t Rxfifo;
char fifo_number;
QueueHandle_t get_data_queue;
struct RevciveData
{
	CAN_RxHeaderTypeDef rxheader;
	uint8_t data[8];
};

void TxHeaderSet(void);
void sFilterConfigSet(void);
void ReceiveDataProcess(void *argument);

char CanInit(void)
{
	fifo_number = 0;
	TxHeaderSet();
	sFilterConfigSet();
	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		return -1;
	}
	if (fifo_number == 0)
	{
		Rxfifo = CAN_RX_FIFO0;
		if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
		{
			return -2;
		}
	}
	else
	{
		Rxfifo = CAN_RX_FIFO1;
		if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
		{
			return -3;
		}
	}
	get_data_queue = xQueueCreate(4, sizeof(struct RevciveData));
	xTaskCreate(ReceiveDataProcess, "ReceiveDataProcess", 32, NULL, 1, NULL);

	return 0;
}

/**
 * @brief Set up CAN TX header configuration
 *
 * Set the standard identifier of the message to 0x200, data length to 8 bytes,
 * and disable the global time stamp. The message is a data frame. The
 * identifier is a standard identifier.
 */
void TxHeaderSet(void)
{
	tx_header.StdId = 0x200;
	tx_header.ExtId = 0;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 8;
	tx_header.IDE = CAN_ID_STD;
	tx_header.TransmitGlobalTime = DISABLE;
}

/**
 * @brief Set up CAN filter configuration
 *
 * Filter configuration for CAN messages from ID 0x200 to 0x2FF
 * to be stored in FIFO0. Mask is set to 0x00FF, which means that
 * only the lower 8 bits of the standard identifier need to match
 * the filter ID.
 */
void sFilterConfigSet(void)
{
	CAN_FilterTypeDef s_filter_config;
	s_filter_config.FilterActivation = ENABLE;
	s_filter_config.FilterBank = 0;
	s_filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	s_filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
	s_filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
	s_filter_config.FilterIdHigh = 0x200 << 5;
	s_filter_config.FilterIdLow = 0x0000;
	s_filter_config.FilterMaskIdHigh = 0x000 << 5;
	s_filter_config.FilterMaskIdLow = 0x0000;
	HAL_CAN_ConfigFilter(&hcan1, &s_filter_config);
}

/**
 * @brief  Sends a CAN message using the predefined transmission header.
 * @param  data: Array of 8 bytes containing the payload to be sent.
 * @retval 0 if the message is successfully queued for transmission,
 *         -1 if the transmission fails.
 */

char CanSend(uint8_t data[8])
{
	uint32_t mailbox;
	if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &mailbox) != HAL_OK)
	{
		return -1; // CAN transmission failed
	}
	return 0;
}

void ReceiveDataProcess(void *argument)
{
	struct RevciveData receive_date;

	xQueueReceive(get_data_queue, &receive_date, portMAX_DELAY);
	switch (receive_date.rxheader.StdId)
	{
	case 0x201:
	case 0x202:
	case 0x203:
	case 0x204:
		MotorRm3508GetOriginalData(receive_date.data, receive_date.rxheader.StdId - 0x201);
		break;
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	struct RevciveData reveive_data;
	CAN_RxHeaderTypeDef rx_header;
	uint8_t reve_data[8];
	HAL_CAN_GetRxMessage(hcan, Rxfifo, &rx_header, reveive_data.data);
	reveive_data.rxheader = rx_header;
	xQueueSendFromISR(get_data_queue, &reveive_data, NULL);
	portYIELD_FROM_ISR(pdTRUE);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	struct RevciveData reveive_data;
	CAN_RxHeaderTypeDef rx_header;
	uint8_t reve_data[8];
	HAL_CAN_GetRxMessage(hcan, Rxfifo, &rx_header, reveive_data.data);
	reveive_data.rxheader = rx_header;
	xQueueSendFromISR(get_data_queue, &reveive_data, NULL);
	portYIELD_FROM_ISR(pdTRUE);
}
