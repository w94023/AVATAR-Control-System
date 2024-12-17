#ifndef __LOW_LEVEL_CONTROLLER_SLAVE_MAIN__
#define __LOW_LEVEL_CONTROLLER_SLAVE_MAIN__

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <string.h>

// UART 관련 메서드 사용을 위한 플래그 설정
#define UART_ENABLED

// 로보티즈 모터 패킷 송수신을 위한 라이브러리 참조
#include "robotis.h"

// 패킷 분석을 위한 라이브러리 참조
#include "packet.h"

// 로보티즈 라이브러리
#include "robotis.h"

// FreeRTOS 사용
#include "cmsis_os.h"

// FreeRTOS task
extern osThreadId spiTaskHandle;
extern osThreadId uartTaskHandle;

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
uint8_t spi_rx_data[SPI_BUFFER_SIZE] = {0};
uint8_t spi_rx_buffer[SPI_BUFFER_SIZE] = {0};
volatile uint8_t spi_rx_buffer_lock = 0;

// 송신 버퍼 정의
uint8_t spi_tx_buffer[SPI_BUFFER_SIZE] = {0};
uint8_t spi_tx_dummy[SPI_BUFFER_SIZE] = {0};
volatile uint8_t spi_tx_lock = 0;

// 슬레이브 보드 준비상태 알림을 위한 GPIO핀 설정
#define SPI_COMM_FLAG_PORT GPIOB
#define SPI_COMM_FLAG_PIN GPIO_PIN_12

////////////////////////////////////////////////
//                   UART                     //
////////////////////////////////////////////////

// UART 설정
#define UART_PORT huart1
#define UART_INSTANCE USART1
#define UART_TX_SET_BUFFER_SIZE 16
#define UART_TX_STATE_BUFFER_SIZE 14
#define UART_RX_BUFFER_SIZE 30
#define UART_RX_TIMER htim1
#define UART_RX_TIMEOUT 10 // ms

// 포트 설정
extern UART_HandleTypeDef UART_PORT;

// 타이머 설정
extern TIM_HandleTypeDef UART_RX_TIMER;
volatile uint16_t uart_rx_timer_count = 0;

// 송신 버퍼 정의
uint8_t uart_motor_set_request_packet[UART_TX_SET_BUFFER_SIZE] = {0};
uint8_t uart_motor_state_request_packet[UART_TX_STATE_BUFFER_SIZE] = {0};

// 수신 버퍼 정의
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE] = {0};
uint8_t uart_rx_data[UART_RX_BUFFER_SIZE] = {0};
volatile uint8_t uart_rx_buffer_lock = 0;

// UART 수신 완료 플래그
uint8_t uart_flag = 1;

////////////////////////////////////////////////
//                    ADC                     //
////////////////////////////////////////////////

// ADC 설정
#define ADC_PORT hadc1
#define ADC_INSTANCE ADC1

// 포트 설정
extern ADC_HandleTypeDef ADC_PORT;

// ADC 데이터
uint32_t adc_value[10] = {0};
uint32_t adc_value_buffer[10] = {0};
volatile uint8_t adc_value_lock = 0;

// ADC 수신 완료 플래그
uint8_t adc_flag = 1;

////////////////////////////////////////////////
//                     LED                    //
////////////////////////////////////////////////

// UART LED
#define UART_LED_GPIO_PORT GPIOA
#define UART_LED_GPIO_PIN GPIO_PIN_9
uint16_t uart_rx_led_count = 0;
uint16_t uart_rx_led_count_interval = 200;

// SPI 에러 LED
#define SPI_LED_GPIO_PORT GPIOA
#define SPI_LED_GPIO_PIN GPIO_PIN_8
uint16_t spi_count = 0;
uint16_t spi_count_interval = 500;

////////////////////////////////////////////////
//                   제어                     //
////////////////////////////////////////////////

// 모터 설정
volatile uint8_t robotis_protocol_version = 2;
volatile uint8_t motor_stop_request = 0;
volatile uint8_t motor_id = 1;

// 모터 제어
volatile int32_t target_position_value = 0;
volatile uint8_t target_torque_enable = 0;
volatile uint8_t target_led_enable = 0;
volatile uint8_t target_control_request = 0;

// 모터 상황
int16_t present_current = 0;
int32_t present_velocity = 0;
int32_t present_position = 0;

// 모터 정지 (emergency stop) 설정
volatile uint8_t target_emergency_stop_enabel = 0;
volatile uint16_t target_sensor_value = 0;

// 모터 response 수신 플래그
volatile uint8_t motor_response_received = 1;

// 마스터보드의 request instruction
volatile uint8_t requested_instruction = 0;

////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////

// SPI 에러 이벤트
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    // 에러 LED ON
    HAL_GPIO_WritePin(SPI_LED_GPIO_PORT, SPI_LED_GPIO_PIN, GPIO_PIN_SET);

    // DMA 전송 중단
    HAL_DMA_Abort(hspi->hdmatx);
    HAL_DMA_Abort(hspi->hdmarx);

    // SPI 통신 재시작
    HAL_SPI_TransmitReceive_DMA(&SPI_PORT, spi_tx_dummy, spi_rx_buffer, SPI_BUFFER_SIZE);
}

void Set_Start_Of_Packet(uint8_t instruction)
{
    // 헤더 설정
    spi_tx_buffer[0] = 0xFF;
    spi_tx_buffer[1] = 0xFF;
    spi_tx_buffer[2] = 0xFD;
    spi_tx_buffer[3] = 0x00;

    // ID 설정
    spi_tx_buffer[4] = motor_id;

    // Instruction 설정
    spi_tx_buffer[5] = instruction;
}

void Send_SPI_Packet()
{
    // Checksum 계산
    spi_tx_buffer[41] = Calculate_Checksum8(spi_tx_buffer, 0, 40);

    // 데이터 송신
    HAL_SPI_TransmitReceive_DMA(&SPI_PORT, spi_tx_buffer, spi_rx_buffer, SPI_BUFFER_SIZE);

    // 마스터 보드로 통신 준비 상태 송신
    // HAL_GPIO_WritePin(SPI_COMM_FLAG_PORT, SPI_COMM_FLAG_PIN, GPIO_PIN_SET);
    HAL_GPIO_TogglePin(SPI_COMM_FLAG_PORT, SPI_COMM_FLAG_PIN);
}

// SPI 송수신 완료 이벤트
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI_INSTANCE) {
        // 마스터 보드로 통신 준비 상태 송신
        // HAL_GPIO_WritePin(SPI_COMM_FLAG_PORT, SPI_COMM_FLAG_PIN, GPIO_PIN_RESET);

        // 에러 LED OFF
        HAL_GPIO_WritePin(SPI_LED_GPIO_PORT, SPI_LED_GPIO_PIN, GPIO_PIN_RESET);

        // SPI 수신 데이터 버퍼링
        if (spi_rx_buffer_lock == 0) {
            spi_rx_buffer_lock = 1;
            memset(spi_rx_data, '\0', SPI_BUFFER_SIZE);
            memcpy(spi_rx_data, spi_rx_buffer, SPI_BUFFER_SIZE);

            osSignalSet(spiTaskHandle, 0x01);
        }
        
        // Request가 있었을 경우, response 패킷 우선 송신
        if (requested_instruction > 0) {
            if (requested_instruction == 5) {
                Set_Start_Of_Packet(requested_instruction);

                spi_tx_buffer[6] = target_emergency_stop_enabel;
                spi_tx_buffer[7] = target_sensor_value & 0xFF;
                spi_tx_buffer[8] = (target_sensor_value >> 8) & 0xFF;

                Send_SPI_Packet();
                requested_instruction = 0;
                return;
            }
        }

        // UART 및 ADC 데이터 수신 시 마스터 보드로 송신
        if (uart_flag == 0 || adc_flag == 0) {        
            Set_Start_Of_Packet((!uart_flag << 7) | (!adc_flag << 6));

            // UART 데이터 설정
            if (uart_flag == 0) {
                Int32_To_Packet(spi_tx_buffer, 6, 9, &present_position);
                Int32_To_Packet(spi_tx_buffer, 10, 13, &present_velocity);
                Int16_To_Packet(spi_tx_buffer, 14, 15, &present_current);
            }
            
            // ADC 데이터 설정
            if (adc_flag == 0) {
                for (size_t i = 0; i < 10; i++) {
                    spi_tx_buffer[16 + 2*i]     = adc_value[i] & 0xFF;
                    spi_tx_buffer[16 + 2*i + 1] = (adc_value[i] >> 8) & 0xFF;
                }
            }
            
            Send_SPI_Packet();

            // 플래그 초기화
            uart_flag = 1;
            adc_flag = 1;
            return;
        }

        // 준비된 데이터가 없을 경우 더미 데이터 송신 (수신을 위해)
        HAL_SPI_TransmitReceive_DMA(&SPI_PORT, spi_tx_dummy, spi_rx_buffer, SPI_BUFFER_SIZE);

        // 마스터 보드로 통신 준비 상태 송신
        // HAL_GPIO_WritePin(SPI_COMM_FLAG_PORT, SPI_COMM_FLAG_PIN, GPIO_PIN_SET);
        HAL_GPIO_TogglePin(SPI_COMM_FLAG_PORT, SPI_COMM_FLAG_PIN);
    }
}

// ADC 에러 이벤트
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    // 내부 오류 처리
    if (hadc->ErrorCode & HAL_ADC_ERROR_INTERNAL) {
        HAL_ADC_DeInit(hadc);
        HAL_ADC_Init(hadc);
    }

    // 오버런 오류 처리
    if (hadc->ErrorCode & HAL_ADC_ERROR_OVR) {
        HAL_DMA_Abort(hadc->DMA_Handle);
        HAL_DMA_Init(hadc->DMA_Handle);
    }

    // DMA 오류 처리
    if (hadc->ErrorCode & HAL_ADC_ERROR_DMA) {
        HAL_DMA_Abort(hadc->DMA_Handle);
        HAL_DMA_Init(hadc->DMA_Handle);
    }

    // ADC 변환 재시작
    HAL_ADC_Start_DMA(hadc, adc_value_buffer, 10);
}

// ADC 수신 이벤트
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC_INSTANCE) {

        // Emergency stop이 활성화 된 경우
        if (target_emergency_stop_enabel == 1) {
            // ADC 9번 채널의 값이 지정된 값 이상일 경우, 모터 정지
            if (adc_value_buffer[9] > target_sensor_value) {
                motor_stop_request = 1;
            }
            else {
                motor_stop_request = 0;
            }
        }

        // ADC 데이터 버퍼링
        if (adc_flag == 1) {
            adc_flag = 0;
            memset(adc_value, '\0', sizeof(adc_value));
            memcpy(adc_value, adc_value_buffer, sizeof(adc_value));
        }

        // ADC 데이터 수신 요청
        HAL_ADC_Start_DMA(&ADC_PORT, adc_value_buffer, 10);
    }
}

// UART 에러 이벤트
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART_INSTANCE) {
        
        // RX 데이터 오버런 시
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) {

            // RX FIFO 초기화
            Flush_UART_RX_FIFO(huart, 10);

            // 플래그 초기화
            __HAL_UART_CLEAR_OREFLAG(huart);
        }
    }
}

// UART 수신 이벤트
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART_INSTANCE) {
        // 타임아웃 타이머 중지
        HAL_TIM_Base_Stop_IT(&UART_RX_TIMER); 
        uart_rx_timer_count = 0;

        // UART 수신 데이터 버퍼링
        if (uart_rx_buffer_lock == 0) {
            uart_rx_buffer_lock = 1;
            memset(uart_rx_data, '\0', UART_RX_BUFFER_SIZE);
            memcpy(uart_rx_data, uart_rx_buffer, UART_RX_BUFFER_SIZE);

            osSignalSet(uartTaskHandle, 0x01);
        }
        
        // 모터 응답 플래그 활성화
        motor_response_received = 1;
    }
}

// 1ms마다 호출되는 메서드
void On_Timer_Tick(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == UART_RX_TIMER.Instance) {    
        uart_rx_timer_count++;
        
        // 설정된 타임아웃 도달 시
        if (uart_rx_timer_count == UART_RX_TIMEOUT) {

            // 타이머 정지
            HAL_TIM_Base_Stop_IT(&UART_RX_TIMER);

            // UART 수신 대기 정지
            HAL_UART_AbortReceive_IT(&UART_PORT);

            // UART RX FIFO 초기화
            Flush_UART_RX_FIFO(&UART_PORT, 10);

            // 타임아웃에 의한 모터 응답 플래그 활성화
            motor_response_received = 1;
            uart_rx_timer_count = 0;
        }
    }
}

void Start()
{
    // SPI 송수신 태스크 시작
    HAL_SPI_TransmitReceive_DMA(&SPI_PORT, spi_tx_dummy, spi_rx_buffer, SPI_BUFFER_SIZE);

    // SPI 송수신 준비 상태 전송
    HAL_GPIO_WritePin(SPI_COMM_FLAG_PORT, SPI_COMM_FLAG_PIN, GPIO_PIN_SET);

    // ADC 수신 태스크 시작
    HAL_ADC_Start_DMA(&ADC_PORT, adc_value_buffer, 10);

    // 모터 상태 요청 패킷 설정
    Motor_State_Request_Protocol_V2(uart_motor_state_request_packet, sizeof(uart_motor_state_request_packet), motor_id);
}

#endif // __LOW_LEVEL_CONTROLLER_SLAVE_MAIN__