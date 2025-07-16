#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "MotorRM3508Drive.h"
// this file is used to receive and send RM3508 motor data through CAN bus
// receive data is processed and sent to the motor_rm3508_rx_queue
// receive data is in the form of struct rx_date_motor_rm3508_struct
// send data is received from the motor_rm3508_tx_queue
// send data is in the form of struct usr_tx_data

#define motor_rx_fifo(num) CAN_RX_FIFO##num
#define motor_rx_number(i, j) i##j
#define motor_tx_number(i)                              \
	do                                                    \
	{                                                     \
		tx_data[2 * i - 2] = motor_send_data.motor##i >> 8; \
		tx_data[2 * i - 1] = motor_send_data.motor##i;      \
	} while (0)

#define motor_active_it(i) CAN_IT_RX_FIFO##i##_MSG_PENDING
// this define is not working
#define CAN_Rx_FifoMsg_PendingCallback(i) void CAN_Rx_Fifo##i##_Msg_PendingCallback(CAN_HandleTypeDef *hcan)

// the can communication protocol for RM3508 motor
// define motor RM3508 tx massage can communication protocol
#define motor_RM3508_tx_header(motor)   \
	do                                    \
	{                                     \
		motor.StdId = 0x200;                \
		motor.ExtId = 0;                    \
		motor.RTR = CAN_RTR_DATA;           \
		motor.DLC = 8;                      \
		motor.IDE = CAN_ID_STD;             \
		motor.TransmitGlobalTime = DISABLE; \
	} while (0)

// define motor RM3508 each rx header can communication protocol
//  the date is the motor number,0-3,corresponding to motor 1-4
#define motor_RM3508_each_rx_header(motor, date) \
	do                                             \
	{                                              \
		motor.StdId = 0x201 + date;                  \
		motor.ExtId = 0;                             \
		motor.RTR = CAN_RTR_DATA;                    \
		motor.DLC = 8;                               \
		motor.IDE = CAN_ID_STD;                      \
	} while (0)

// define can filter configuration for RM3508 motor
#define motor_RM3508_sFilterConfig(sFilterConfig)          \
	do                                                       \
	{                                                        \
		sFilterConfig.FilterActivation = ENABLE;               \
		sFilterConfig.FilterBank = 0;                          \
		sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; \
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;      \
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;     \
		sFilterConfig.FilterIdHigh = 0x200 << 5;               \
		sFilterConfig.FilterIdLow = 0x0000;                    \
		sFilterConfig.FilterMaskIdHigh = 0x7fe << 5;           \
		sFilterConfig.FilterMaskIdLow = 0x0000;                \
	} while (0)

static CAN_TxHeaderTypeDef tx_header_motor;
static CAN_RxHeaderTypeDef rx_header_motor[4];
static CAN_FilterTypeDef sFilterConfig;
static CAN_HandleTypeDef hcan;
static uint32_t Rxfifo;
static char fifo_number;
struct tx_data_motor_rm3508_struct motor_send_data;

struct rx_date_motor_rm3508_struct motor_rx_date;
struct temp_rx_data
{
	char motor;
	uint8_t rx_data[8];
} temp_rx_data;
static QueueHandle_t motor_rm3508_rx_queue;
static QueueHandle_t motor_rm3508_tx_queue;
QueueHandle_t temp_data;

// CAN1 init function
void motor_RM3508_Init(CAN_HandleTypeDef *hcan1, char fifo_number_input)
{
	// Initialize the CAN filter configuration
	fifo_number = fifo_number_input;
	if (fifo_number)
	{
		Rxfifo = motor_rx_fifo(1);
	}
	else
	{
		Rxfifo = motor_rx_fifo(0);
	}
	hcan = *hcan1;
	int temp = 0;
	while (temp < 4)
	{
		motor_RM3508_each_rx_header(rx_header_motor[temp], temp);
		temp++;
	}
	motor_RM3508_tx_header(tx_header_motor);
	motor_RM3508_sFilterConfig(sFilterConfig);
	if (fifo_number)
	{
		HAL_CAN_ActivateNotification(&hcan, motor_active_it(1));
	}
	else
	{
		HAL_CAN_ActivateNotification(&hcan, motor_active_it(0));
	}
	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
	HAL_CAN_Start(&hcan);

	// Create the queue for temporary RX data
	temp_data = xQueueCreate(10, sizeof(struct temp_rx_data));
	motor_rm3508_tx_queue = xQueueCreate(1, sizeof(struct tx_data_motor_rm3508_struct));
	motor_rm3508_rx_queue = xQueueCreate(1, sizeof(struct rx_date_motor_rm3508_struct));

	xTaskCreate(moter_rm3508_tx_massage, "moter_rm3508_tx_massage", 128, NULL, 1, NULL);
	xTaskCreate(motor_rm3508_rx_massage, "motor_rm3508_rx_massage", 128, NULL, 1, NULL);

	motor_send_data.motor1 = 0;
	motor_send_data.motor2 = 0;
	motor_send_data.motor3 = 0;
	motor_send_data.motor4 = 0;
}

// thread to send data to the motor
void moter_rm3508_tx_massage(void *argument)
{
	// get the data from the motor_rm3508_tx_queue and store it in the motor_send_data
	while (1)
	{
		struct usr_tx_data motor;
		xQueueReceive(motor_rm3508_tx_queue, &motor, portMAX_DELAY);
		switch (motor.motor)
		{
		case 1:
			motor_send_data.motor1 = motor.rx_data;
			break;
		case 2:
			motor_send_data.motor2 = motor.rx_data;
			break;
		case 3:
			motor_send_data.motor3 = motor.rx_data;
			break;
		case 4:
			motor_send_data.motor4 = motor.rx_data;
			break;
		default:
			break;
		}

		// prepare the tx header and data
		uint32_t tx_mailbox;
		uint8_t tx_data[8] = {0};
		motor_tx_number(1);
		motor_tx_number(2);
		motor_tx_number(3);
		motor_tx_number(4);

		// send the data to the motor
		HAL_CAN_AddTxMessage(&hcan, &tx_header_motor, tx_data, &tx_mailbox);
	}
}

// Function to process the received data from the moto,and return the processed data
void motor_rm3508_rx_massage(void *argument)
{
	while (1)
	{
		struct temp_rx_data temp;
		xQueueReceive(temp_data, &temp, portMAX_DELAY);
		motor_rx_date.angle = (temp.rx_data[0] << 8 | temp.rx_data[1]) / 8191.0f;
		motor_rx_date.rpm = (temp.rx_data[2] << 8 | temp.rx_data[3]);
		motor_rx_date.current = (temp.rx_data[4] << 8 | temp.rx_data[5]);
		motor_rx_date.temperture = temp.rx_data[6];
		motor_rx_date.motor_number = temp.motor;
		xQueueOverwrite(motor_rm3508_rx_queue, &motor_rx_date);
	}
}

// Callback function for CAN message pending in FIFO 0/1
struct rx_date_motor_rm3508_struct motor_rx_date_it[4];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (fifo_number == 1)
		return;
	for (int i = 0; i < 4; i++)
	{
		if (HAL_CAN_GetRxMessage(hcan, Rxfifo, &rx_header_motor[i], temp_rx_data.rx_data) == HAL_OK)
		{
			temp_rx_data.motor = i;
			xQueueSendFromISR(temp_data, &temp_rx_data, NULL);
			break;
		}
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (fifo_number == 0)
		return;

	for (int i = 0; i < 4; i++)
	{
		if (HAL_CAN_GetRxMessage(hcan, Rxfifo, &rx_header_motor[i], temp_rx_data.rx_data) == HAL_OK)
		{
			temp_rx_data.motor = i;
			xQueueSendFromISR(temp_data, &temp_rx_data, NULL);
			break;
		}
	}
}
