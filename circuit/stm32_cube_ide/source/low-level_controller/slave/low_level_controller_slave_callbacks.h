#ifndef __LOW_LEVEL_CONTROLLER_SLAVE_CALLBACKS__
#define __LOW_LEVEL_CONTROLLER_SLAVE_CALLBACKS__

#include "low_level_controller_slave_main.h"
#include "packet.h"

void Analyze_SPI_Packet()
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < SPI_BUFFER_SIZE; i++) {
        sum += spi_rx_data[i];
    }
    if (sum == 0) {
        return;
    }

    if (Check_Packet_Validity(spi_rx_data, SPI_BUFFER_SIZE, 1, 0) == 0) {
        return;
    }

    // 모터 ID 설정
    motor_id = spi_rx_data[4];
    
    // Data 설정
    if (spi_rx_data[5] == 0x01) {
        // Goal position

        // SPI를 통해 수신한 target position : 501923 (180도) ~ -501923 (-180도)로 표현
        // 양수 : Little endian 기준 16 바이트 변환 (501923 --> 0xA3, 0xA8, 0x07, 0x00)
        // 음수 : Little endian 기준 16 바이트 변환 후, 모든 비트 반전 (-501923 --> 501923 변환 (0xA3, 0xA8, 0x07, 0x07) 후, 모든 비트 반전 (0x5C, 0x57, 0xF8, 0xFF))

        int32_t target_position = 0;
        target_position |= spi_rx_data[6];
        target_position |= (spi_rx_data[7] << 8);
        target_position |= (spi_rx_data[8] << 16);
        target_position |= (spi_rx_data[9] << 24);

        target_position_value = target_position;
        target_control_request |= (1 << 1);
    }
    else if (spi_rx_data[5] == 0x03) {
        // Torque enable
        target_torque_enable = spi_rx_data[6];
        target_control_request |= (1 << 0);
    }
    else if (spi_rx_data[5] == 0x04) {
        // LED enable
        target_led_enable = spi_rx_data[6];
        target_control_request |= (1 << 2);
    }
    else if (spi_rx_data[5] == 0x05) {
        // Emergency stop set
        requested_instruction = 5;
        target_emergency_stop_enabel = spi_rx_data[6];
        target_sensor_value = spi_rx_data[7] | (spi_rx_data[8] << 8);
    }

    return;
}

void SPI_Callback()
{
    osSignalWait(0x01, osWaitForever);

    // 수신 SPI 패킷으로부터 모터 제어 정보 획득
    Analyze_SPI_Packet();

    // 모터가 제어 중일 경우
    if (motor_response_received == 0) {
        spi_rx_buffer_lock = 0;
        return;
    }

    if (target_control_request > 0) {
        // 모터 제어 전송 요청이 있었을 경우, 모터 제어 요청 전송

        // Torque enable 제어
        if ((target_control_request & (1 << 0)) != 0) {
            motor_response_received = 0;
            Motor_Torque_Enable_Set_Protocol_V2(&UART_PORT, motor_id, target_torque_enable, uart_motor_set_request_packet, UART_TX_SET_BUFFER_SIZE, uart_rx_buffer, UART_RX_BUFFER_SIZE);
            HAL_TIM_Base_Start_IT(&UART_RX_TIMER);
        }    
        // Target position 제어
        else if ((target_control_request & (1 << 1)) != 0) {
            if (motor_stop_request == 1) {
                // Motor stop request가 있을 때는 현재 positon으로 제어
                motor_response_received = 0;
                Motor_Position_Set_Protocol_V2(&UART_PORT, motor_id, present_position, uart_motor_set_request_packet, UART_TX_SET_BUFFER_SIZE, uart_rx_buffer, UART_RX_BUFFER_SIZE);
                HAL_TIM_Base_Start_IT(&UART_RX_TIMER);
            }
            else if (motor_stop_request == 0) {
                // Motor stop request가 없을 때는 수신한 position으로 제어
                motor_response_received = 0;
                Motor_Position_Set_Protocol_V2(&UART_PORT, motor_id, target_position_value, uart_motor_set_request_packet, UART_TX_SET_BUFFER_SIZE, uart_rx_buffer, UART_RX_BUFFER_SIZE);
                HAL_TIM_Base_Start_IT(&UART_RX_TIMER);
            }
            
        }
        // LED enable 제어 
        else if ((target_control_request & (1 << 2)) != 0) {
            motor_response_received = 0;
            Motor_LED_Enable_Set_Protocol_V2(&UART_PORT, motor_id, target_led_enable, uart_motor_set_request_packet, UART_TX_SET_BUFFER_SIZE, uart_rx_buffer, UART_RX_BUFFER_SIZE);
            HAL_TIM_Base_Start_IT(&UART_RX_TIMER);
        }
    }
    else {
        // 모터 제어 전송 요청이 없었을 경우, 모터 상태 확인 요청 전송
        motor_response_received = 0;
        memset(uart_rx_buffer, '\0', UART_RX_BUFFER_SIZE);
        Flush_UART_RX_FIFO(&UART_PORT, 10);
        HAL_UART_Receive_IT(&UART_PORT, uart_rx_buffer, 21);
        HAL_UART_Transmit_IT(&UART_PORT, uart_motor_state_request_packet, sizeof(uart_motor_state_request_packet));
        HAL_TIM_Base_Start_IT(&UART_RX_TIMER);
    }

    // 버퍼 잠금 해제
    spi_rx_buffer_lock = 0;
}

void UART_Callback()
{
    osSignalWait(0x01, osWaitForever);

    // LED 토글
    uart_rx_led_count++;
    if (uart_rx_led_count == uart_rx_led_count_interval) {
        uart_rx_led_count = 0;
        HAL_GPIO_TogglePin(UART_LED_GPIO_PORT, UART_LED_GPIO_PIN);
    }

    if (target_control_request > 0) {
        // 모터 제어 패킷에 대한 회신일 경우
        // 모터 제어 성공 패킷이 확인될 경우 플래그 초기화
        if ((target_control_request & (1 << 0)) != 0) {
            if (Get_Motor_Response_Protocol_V2(uart_rx_data, UART_RX_BUFFER_SIZE, motor_id) == 1) {
                target_control_request &= ~(1 << 0);
            }
        }
        else if ((target_control_request & (1 << 1)) != 0) {
            if (Get_Motor_Response_Protocol_V2(uart_rx_data, UART_RX_BUFFER_SIZE, motor_id) == 1) {
                target_control_request &= ~(1 << 1);
            }
        }
        else if ((target_control_request & (1 << 2)) != 0) {
            if (Get_Motor_Response_Protocol_V2(uart_rx_data, UART_RX_BUFFER_SIZE, motor_id) == 1) {
                target_control_request &= ~(1 << 2);
            }
        }
    }
    else {
        // 모터 상태 회신 요청에 대한 회신일 경우
        int16_t current = 0;
        int32_t velocity = 0;
        int32_t position = 0;

        // 모터 상태 저장
        if (Get_Motor_State_Protocol_V2(uart_rx_data, UART_RX_BUFFER_SIZE, motor_id, &current, &velocity, &position) == 1) {
            if (uart_flag == 1) {
                present_current = current;
                present_velocity = velocity;
                present_position = position;

                uart_flag = 0;
            }
        }
    }

    // 버퍼 락 해제
    uart_rx_buffer_lock = 0;
}

#endif // __LOW_LEVEL_CONTROLLER_SLAVE_CALLBACKS__