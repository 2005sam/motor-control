#include "motor_rm3508.h"
#include "queue.h"

struct SetData
{
	char motor_number;
	int16_t motor_current;
};
struct RxData
{
	char motor_number;
	uint8_t *motor_data;
};

CAN_HandleTypeDef hcan;
CAN_RxHeaderTypeDef rx_header[4];
CAN_TxHeaderTypeDef tx_header;
QueueHandle_t set_data_queue;
QueueHandle_t get_data_queue;
struct MotorRm3508ReturnData received_data[4];
uint8_t tx_data[8] = {0};
char fifo_number;
static uint32_t Rxfifo;

void RxHeaderSet(void);
void TxHeaderSet(void);
void sFilterConfigSet(void);
void SendDataUpdate(void *argument);
void SendData(void *argument);
void ReceiveDataProcess(void *argument);

// this function is used to initialize the motor RM3508
// input hcan1 is the can handle
// return 0 if success
char MotorRm3508Init(CAN_HandleTypeDef *hcan1)
{
	hcan = *hcan1;
	fifo_number = 0;
	if (fifo_number == 0)
	{
		Rxfifo = CAN_RX_FIFO0;
		HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	}
	else
	{
		Rxfifo = CAN_RX_FIFO1;
		HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
	}

	RxHeaderSet();
	TxHeaderSet();
	sFilterConfigSet();
	HAL_CAN_Start(&hcan);

	get_data_queue = xQueueCreate(4, sizeof(struct RxData));
	set_data_queue = xQueueCreate(4, sizeof(struct SetData));
	xTaskCreate(SendDataUpdate, "SendDataUpdate", 128, NULL, 1, NULL);
	xTaskCreate(SendData, "SendData", 128, NULL, 1, NULL);

	xTaskCreate(ReceiveDataProcess, "ReceiveDataProcess", 128, NULL, 1, NULL);
	return 0;
}

void RxHeaderSet(void)
{
	for (int i = 0; i < 4; i++)
	{
		rx_header[i].StdId = 0x201 + i;
		rx_header[i].ExtId = 0;
		rx_header[i].RTR = CAN_RTR_DATA;
		rx_header[i].DLC = 8;
		rx_header[i].IDE = CAN_ID_STD;
	}
}

void TxHeaderSet(void)
{
	tx_header.StdId = 0x200;
	tx_header.ExtId = 0;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 8;
	tx_header.IDE = CAN_ID_STD;
	tx_header.TransmitGlobalTime = DISABLE;
}

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
	s_filter_config.FilterMaskIdHigh = 0x7fe << 5;
	s_filter_config.FilterMaskIdLow = 0x0000;
	HAL_CAN_ConfigFilter(&hcan, &s_filter_config);
}

// function to set the motor current
//  input motor_number is the motor number,0-3,corresponding to motor 1-4
//  input motor_current is the motor current,range is -32768 to 32767
//  return 0 if success
char MotorRm3508Set(char motor_number, int16_t motor_current)
{
	struct SetData set_data;
	set_data.motor_number = motor_number;
	set_data.motor_current = motor_current;
	xQueueSend(set_data_queue, &set_data, 0);
	return 0;
}

// function to get the motor current
//  input motor_number is the motor number,0-3,corresponding to motor 1-4
//  input kpdata is the pointer to the struct to store the motor data
// pdata is the pointer to the struct to store the motor data
//  return 0 if success
char MotorRm3508Get(char motor_number, struct MotorRm3508ReturnData *kpdata)
{
	*kpdata = received_data[motor_number];
	return 0;
}

// function is used to updata the tx_data when a set data is received
void SendDataUpdate(void *argument)
{
	struct SetData set_data;
	xQueueReceive(set_data_queue, &set_data, portMAX_DELAY);
	tx_data[set_data.motor_number * 2] = set_data.motor_current >> 8;
	tx_data[set_data.motor_number * 2 + 1] = set_data.motor_current & 0xFF;
}

// function is used to send the tx_data to the motor
void SendData(void *argument)
{
	while (1)
	{
		HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, NULL);
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

// function is used to receive the data from the motor and store it in the received_data structure
// it also frees the memory rx_data.motor_data
void ReceiveDataProcess(void *argument)
{
	struct RxData rx_data;
	while (1)
	{
		xQueueReceive(get_data_queue, &rx_data, portMAX_DELAY);
		received_data[rx_data.motor_number].angle = (rx_data.motor_data[0] << 8 | rx_data.motor_data[1]) / 8191.0f;
		received_data[rx_data.motor_number].rpm = (rx_data.motor_data[2] << 8 | rx_data.motor_data[3]);
		received_data[rx_data.motor_number].current = (rx_data.motor_data[4] << 8 | rx_data.motor_data[5]);
		received_data[rx_data.motor_number].temperture = rx_data.motor_data[6];
		vPortFree(rx_data.motor_data);
	}
}

// two functions are used to receive the data from the motor
// warning: this function is called in the interrupt, so it should not use any blocking functions
// a memory is allocated to store the received data, and it should be freed after use
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (fifo_number == 1)
		return;
	struct RxData rx_data;
	uint8_t *recv_data = pvPortMalloc(sizeof(uint8_t) * 8);
	for (int i = 0; i < 4; i++)
	{
		if (HAL_CAN_GetRxMessage(hcan, Rxfifo, &rx_header[i], recv_data) == HAL_OK)
		{
			rx_data.motor_number = i;
			rx_data.motor_data = recv_data;
			if (xQueueSendToBackFromISR(get_data_queue, &rx_data, NULL) != pdPASS)
				vPortFree(recv_data);

			return;
		}
	}
	vPortFree(recv_data);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (fifo_number == 0)
		return;
	struct RxData rx_data;
	uint8_t *recv_data = pvPortMalloc(sizeof(uint8_t) * 8);
	for (int i = 0; i < 4; i++)
	{
		if (HAL_CAN_GetRxMessage(hcan, Rxfifo, &rx_header[i], recv_data) == HAL_OK)
		{
			rx_data.motor_number = i;
			rx_data.motor_data = recv_data;
			if (xQueueSendToBackFromISR(get_data_queue, &rx_data, NULL) != pdPASS)
				vPortFree(recv_data);

			return;
		}
	}
	vPortFree(recv_data);
}