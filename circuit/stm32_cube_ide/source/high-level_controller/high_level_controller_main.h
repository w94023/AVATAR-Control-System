#ifndef __HIGH_LEVEL_CONTROLLER_MAIN__
#define __HIGH_LEVEL_CONTROLLER_MAIN__

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <string.h>

#define CAN_ENABLED

// FreeRTOS 사용
#include "cmsis_os.h"

#include "packet.h"

// FreeRTOS task
extern osThreadId uartTaskHandle;
extern osThreadId canTaskHandle;

////////////////////////////////////////////////
//                    CAN                     //
////////////////////////////////////////////////

// CAN 설정
#define CAN_PORT hcan
#define CAN_DATA_SIZE 8
#define CAN_BUFFER_SIZE 42

// 포트 설정
extern CAN_HandleTypeDef CAN_PORT;

// 수신 데이터 정의
CAN_RxHeaderTypeDef can_rx_header;
uint8_t can_rx_buffer[CAN_DATA_SIZE] = {0};
uint8_t can_rx_data[CAN_BUFFER_SIZE] = {0};
uint8_t can_rx_data_buffer[CAN_BUFFER_SIZE] = {0};
volatile uint8_t can_rx_buffer_count = 0;
volatile uint8_t can_rx_buffer_lock = 0;

// 송신 데이터 정의
CAN_TxHeaderTypeDef can_tx_header;
uint32_t can_tx_mailbox;
uint8_t can_tx_data[CAN_BUFFER_SIZE] = {0};
volatile uint16_t can_tx_count = 0;

////////////////////////////////////////////////
//                   UART                     //
////////////////////////////////////////////////

// UART 설정
#define UART_PORT huart1
#define UART_INSTANCE USART1

#define UART_RX_TIMER htim1
#define UART_RX_TIMEOUT 100 // ms

#define UART_BUFFER_SIZE 42

// 포트 설정
extern UART_HandleTypeDef UART_PORT;

// 타이머 설정
extern TIM_HandleTypeDef UART_RX_TIMER;

// 송신 데이터 설정
uint8_t uart_tx_flag = 0;
uint8_t uart_tx_buffer[UART_BUFFER_SIZE] = {0};

// 수신 데이터 설정
uint8_t uart_rx_buffer[UART_BUFFER_SIZE] = {0};
uint8_t uart_rx_data[UART_BUFFER_SIZE] = {0};
volatile uint8_t uart_rx_buffer_lock = 0;
volatile uint16_t uart_rx_timer_count = 0;

////////////////////////////////////////////////
//                   기타                     //
////////////////////////////////////////////////

// 송신 표시 LED 정의
#define UART_LED_GPIO_PORT GPIOA
#define UART_LED_GPIO_PIN GPIO_PIN_5
uint16_t uart_tx_led_count = 0;
uint16_t uart_tx_led_count_interval = 20;

// Auto rotate 플래그
volatile uint8_t auto_rotate_flag = 0;

// Auto rotate 방향 설정
volatile int32_t direction = 1;

// 메인 루프 카운터
volatile int32_t main_loop_count = 0;

// Auto rotate 시, 너무 잦은 CAN 송신 방지를 위한 카운터
volatile uint8_t can_tx_wait_count = 0;

////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////

// CAN 수신 이벤트
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    // CAN 데이터 수신
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_header, can_rx_buffer) == HAL_OK) {
        
        // 순번 확인
        if (can_rx_buffer[0] == can_rx_buffer_count) {
            // 체크섬 검사
            uint8_t checksum = Calculate_Checksum8(can_rx_buffer, 0, 6);
            if (checksum == can_rx_buffer[7]) {
                memcpy(can_rx_data_buffer + can_rx_buffer_count * 6, can_rx_buffer + 1, 6);
                can_rx_buffer_count++;
            }
        }

        // 40바이트 : 7개의 8바이트 CAN패킷으로 전송
        if (can_rx_buffer_count == 7) {
            if (can_rx_buffer_lock == 0) {
                can_rx_buffer_lock = 1;
                memset(can_rx_data, '\0', CAN_BUFFER_SIZE);
                memcpy(can_rx_data, can_rx_data_buffer, CAN_BUFFER_SIZE);

                osSignalSet(canTaskHandle, 0x01);
            }

            can_rx_buffer_count = 0;
        }
    }
}

// UART 수신 이벤트
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART_INSTANCE) {
        HAL_TIM_Base_Stop_IT(&UART_RX_TIMER); // 타임아웃 타이머 중지
        uart_rx_timer_count = 0;

        if (uart_rx_buffer_lock == 0) {
            uart_rx_buffer_lock = 1;

            memset(uart_rx_data, '\0', UART_BUFFER_SIZE);
            memcpy(uart_rx_data, uart_rx_buffer, UART_BUFFER_SIZE);

            osSignalSet(uartTaskHandle, 0x01);
        }

        HAL_UART_Receive_IT(&UART_PORT, uart_rx_buffer, UART_BUFFER_SIZE);
        // 타임아웃 타이머 시작 (예: 100ms 타임아웃)
        HAL_TIM_Base_Start_IT(&UART_RX_TIMER);
    }
}

void On_Timer_Tick(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == UART_RX_TIMER.Instance) {    
        uart_rx_timer_count++;
        if (uart_rx_timer_count == UART_RX_TIMEOUT) {
            HAL_TIM_Base_Stop_IT(&UART_RX_TIMER);
            HAL_UART_AbortReceive_IT(&UART_PORT);
            
            HAL_UART_Receive_IT(&UART_PORT, uart_rx_buffer, UART_BUFFER_SIZE);
            HAL_TIM_Base_Start_IT(&UART_RX_TIMER);
            uart_rx_timer_count = 0;
        }
    }
}

void Start()
{
    // CAN filter 선언
    CAN_FilterTypeDef can_filter;
    can_filter.FilterBank = 0;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh = 0x0000;
    can_filter.FilterIdLow = 0x0000;
    can_filter.FilterMaskIdHigh = 0x0000;
    can_filter.FilterMaskIdLow = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterActivation = ENABLE;
    can_filter.SlaveStartFilterBank = 14;

    // CAN filter 적용
    HAL_CAN_ConfigFilter(&CAN_PORT, &can_filter);

    // CAN 수신 콜백 시작
    HAL_CAN_ActivateNotification(&CAN_PORT, CAN_IT_RX_FIFO0_MSG_PENDING);

    // CAN 통신 시작
    HAL_CAN_Start(&CAN_PORT);

    // UART 수신 시작
    HAL_UART_Receive_IT(&UART_PORT, uart_rx_buffer, UART_BUFFER_SIZE);
    // 타임아웃 타이머 시작 (예: 100ms 타임아웃)
    HAL_TIM_Base_Start_IT(&UART_RX_TIMER);
}

#endif // __HIGH_LEVEL_CONTROLLER_MAIN__