#ifndef __LOW_LEVEL_CONTROLLER_MASTER_MAIN__
#define __LOW_LEVEL_CONTROLLER_MASTER_MAIN__

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <string.h>

// CAN 관련 메서드 사용을 위한 플래그 설정
#define CAN_ENABLED

// 로보티즈 모터 패킷 송수신을 위한 라이브러리 참조
#include "robotis.h"

// 패킷 분석을 위한 라이브러리 참조
#include "packet.h"

// FreeRTOS 사용
#include "cmsis_os.h"

// FreeRTOS task
extern osThreadId spiTaskHandle;
extern osThreadId canRxTaskHandle;
extern osThreadId canTxTaskHandle;

////////////////////////////////////////////////
//                    SPI                     //
////////////////////////////////////////////////

// SPI 설정
#define SPI_PORT hspi2
#define SPI_INSTANCE SPI2
#define SPI_BUFFER_SIZE 42

// 포트 설정
extern SPI_HandleTypeDef SPI_PORT;

// 수신 버퍼 정의
uint8_t spi_rx_data[SPI_BUFFER_SIZE];
uint8_t spi_rx_buffer[SPI_BUFFER_SIZE];

// 송신 버퍼 정의
uint8_t spi_tx_data[SPI_BUFFER_SIZE];
uint8_t spi_tx_buffer[SPI_BUFFER_SIZE];
uint8_t spi_tx_dummy[SPI_BUFFER_SIZE];
uint8_t spi_flag = 1; // 송신 완료 플래그

volatile uint8_t spi_tx_lock = 0;
volatile uint8_t spi_rx_buffer_lock = 0;

////////////////////////////////////////////////
//                    CAN                     //
////////////////////////////////////////////////

// CAN 설정
#define CAN_PORT hcan
#define CAN_RX_DATA_SIZE 8
#define CAN_TX_DATA_SIZE 30

#define CAN_RX_TIMER htim1
#define CAN_RX_TIMEOUT 100 // ms

// 포트 설정
extern CAN_HandleTypeDef CAN_PORT;
CAN_RxHeaderTypeDef can_rx_header;

// 타이머 설정
extern TIM_HandleTypeDef CAN_RX_TIMER;
volatile uint16_t can_rx_timer_count = 0;

// 수신 버퍼 정의
uint8_t can_rx_buffer[CAN_RX_DATA_SIZE] = {0};
uint8_t can_rx_data_buffer[SPI_BUFFER_SIZE] = {0};
uint8_t can_rx_data[SPI_BUFFER_SIZE] = {0};
volatile uint8_t can_rx_buffer_lock = 0;
volatile uint8_t can_rx_buffer_count;

// 송신 버퍼 정의
uint8_t can_tx_data[SPI_BUFFER_SIZE] = {0};
uint8_t can_tx_buffer[SPI_BUFFER_SIZE] = {0};
volatile uint8_t can_tx_buffer_lock = 0;
volatile uint8_t can_tx_count = 0;

////////////////////////////////////////////////
//                    LED                     //
////////////////////////////////////////////////

// SPI 수신 완료 표시용 LED
#define SPI_LED_GPIO_PORT GPIOA
#define SPI_LED_GPIO_PIN GPIO_PIN_8
uint16_t spi_rx_count = 0;
uint16_t spi_rx_count_interval = 500;

// CAN 수신 완료 표시용 LED
#define CAN_LED_GPIO_PORT GPIOA
#define CAN_LED_GPIO_PIN GPIO_PIN_9
uint16_t can_rx_count = 0;
uint16_t can_rx_count_interval = 10;

////////////////////////////////////////////////
//                PPS counter                 //
////////////////////////////////////////////////

// PPS 주기 계산
volatile uint16_t pps_timer_count = 0;

// UART PPS counter
uint16_t uart_work_counting_flag = 0;
uint16_t uart_work_count = 0;

// ADC PPS counter
uint16_t adc_work_counting_flag = 0;
uint16_t adc_work_count = 0;

////////////////////////////////////////////////
//                   기타                     //
////////////////////////////////////////////////

// Request 수신 여부 플래그
volatile uint8_t on_response_received = 1;

// Request data 임시 저장용 버퍼
uint8_t requested_data[SPI_BUFFER_SIZE] = {0};

////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////

// SPI 에러 이벤트
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    
    // DMA 전송 중단
    HAL_DMA_Abort(hspi->hdmatx);
    HAL_DMA_Abort(hspi->hdmarx);

    // SPI 초기화 해제
    HAL_SPI_DeInit(hspi);

    // SPI 재초기화
    HAL_SPI_Init(hspi);

    // SPI 통신 시작
    HAL_SPI_TransmitReceive_DMA(hspi, spi_tx_dummy, spi_rx_buffer, SPI_BUFFER_SIZE);
}

volatile uint8_t spi_delay_tick = 0;
volatile uint8_t spi_count_start = 0;
volatile GPIO_PinState spi_slave_respone = GPIO_PIN_SET;

// SPI 수신 이벤트
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI_INSTANCE) {

        // LED 토글
        if (spi_rx_count == spi_rx_count_interval) {
            HAL_GPIO_TogglePin(SPI_LED_GPIO_PORT, SPI_LED_GPIO_PIN);
            spi_rx_count = 0;
        }
        spi_rx_count = spi_rx_count + 1;

        // SPI 데이터 버퍼링
        if (spi_rx_buffer_lock == 0) {
            spi_rx_buffer_lock = 1;
            memset(spi_rx_data, '\0', SPI_BUFFER_SIZE);
            memcpy(spi_rx_data, spi_rx_buffer, SPI_BUFFER_SIZE);

            osSignalSet(spiTaskHandle, 0x01);
        }
    }

    // 슬레이브 보드 준비 완료까지 대기
    while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) != spi_slave_respone);
    if (spi_slave_respone == GPIO_PIN_SET) {
        spi_slave_respone = GPIO_PIN_RESET;
    }
    else {
        spi_slave_respone = GPIO_PIN_SET;
    }

    if (spi_flag == 0) {
        // 송신 데이터가 존재할 경우, 데이터 전송
        HAL_SPI_TransmitReceive_DMA(&SPI_PORT, spi_tx_data, spi_rx_buffer, SPI_BUFFER_SIZE);
        spi_flag = 1;
    }
    else {
        // 송신 데이터가 존재하지 않을 경우, 더미 데이터 전송 (수신을 위함)
        HAL_SPI_TransmitReceive_DMA(&SPI_PORT, spi_tx_dummy, spi_rx_buffer, SPI_BUFFER_SIZE);
    }
}

// CAN 수신 이벤트
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
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
                memset(can_rx_data, '\0', SPI_BUFFER_SIZE);
                memcpy(can_rx_data, can_rx_data_buffer, SPI_BUFFER_SIZE);

                osSignalSet(canRxTaskHandle, 0x01);
            }

            can_rx_buffer_count = 0;
            can_rx_timer_count = 0;
        }
    }
}

// 1ms마다 호출되는 메서드
void On_Timer_Tick(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == CAN_RX_TIMER.Instance) {    
        can_rx_timer_count++;
        pps_timer_count++;
        if (spi_count_start == 1) {
            spi_delay_tick++;
        }

        // CAN 타임아웃 적용
        if (can_rx_timer_count == CAN_RX_TIMEOUT) {
            can_rx_timer_count = 0;
            can_rx_buffer_count = 0;
        }

        // 1초마다 PPS 업데이트
        if (pps_timer_count == 1000) { // 1초 주기로 동작
            adc_work_count = adc_work_counting_flag;
            adc_work_counting_flag = 0;

            uart_work_count = uart_work_counting_flag;
            uart_work_counting_flag = 0;

            pps_timer_count = 0;
        }
    }
}

void Start()
{
    // CAN 수신 필터 구성
    CAN_FilterTypeDef canFilter;
    canFilter.FilterBank = 0;
    canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilter.FilterIdHigh = 0x0000;
    canFilter.FilterIdLow = 0x0000;
    canFilter.FilterMaskIdHigh = 0x0000;
    canFilter.FilterMaskIdLow = 0x0000;
    canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilter.FilterActivation = ENABLE;
    canFilter.SlaveStartFilterBank = 14;

    // CAN 설정
    HAL_CAN_ConfigFilter(&hcan, &canFilter);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan);

    // 타이머 시작
    HAL_TIM_Base_Start_IT(&CAN_RX_TIMER);

    // 슬레이브 보드 준비 완료 시 SPI 통신 시작
    while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) != GPIO_PIN_SET);
    spi_slave_respone = GPIO_PIN_RESET;
    HAL_SPI_TransmitReceive_DMA(&SPI_PORT, spi_tx_dummy, spi_rx_buffer, SPI_BUFFER_SIZE);
}

#endif // __LOW_LEVEL_CONTROLLER_MASTER_MAIN__