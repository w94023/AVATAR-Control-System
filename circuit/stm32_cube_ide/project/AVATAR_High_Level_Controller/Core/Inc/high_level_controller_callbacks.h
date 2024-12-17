#ifndef __HIGH_LEVEL_CONTROLLER_CALLBACKS__
#define __HIGH_LEVEL_CONTROLLER_CALLBACKS__

#include "high_level_controller_main.h"
#include "packet.h"

// UART 수신 콜백
void UART_Callback()
{
    osSignalWait(0x01, osWaitForever);

    // UART 패킷 유효성 검사
    if (Check_Packet_Validity(uart_rx_data, UART_BUFFER_SIZE, 2, 1) == 1) {
        // LED 토글
        HAL_GPIO_TogglePin(UART_LED_GPIO_PORT, UART_LED_GPIO_PIN);

        // Auto rotate request 수신
        if (uart_rx_data[5] == 0x02) {
            auto_rotate_flag = uart_rx_data[6];
        }
        // Goal position 수신
        else if (uart_rx_data[5] == 0x01) {
            // Auto rotate 중이 아닐 때에만 제어 정보 송신
            if (auto_rotate_flag == 0) {
                // HAL_GPIO_TogglePin(UART_LED_GPIO_PORT, UART_LED_GPIO_PIN);
                Transmit_MultiByte_Through_CAN(&CAN_PORT, uart_rx_data, UART_BUFFER_SIZE);
            }
        }
        else {
            // 제어 정보 CAN으로 송신
            Transmit_MultiByte_Through_CAN(&CAN_PORT, uart_rx_data, UART_BUFFER_SIZE);
        }
    }
    
    uart_rx_buffer_lock = 0;
}

// CAN 수신 콜백
void CAN_Callback()
{
    osSignalWait(0x01, osWaitForever);

    // CRC
    uint16_t crc16 = Calculate_CRC16_UMTS(can_rx_data, 0, 39);
    if (can_rx_data[40] == (crc16 & 0xFF) && can_rx_data[41] == ((crc16 >> 8) & 0xFF)) {
        can_tx_count++;
        if (can_tx_count == 5) {
            HAL_UART_Transmit(&UART_PORT, can_rx_data, CAN_BUFFER_SIZE, HAL_MAX_DELAY);
            can_tx_count = 0;
        }
    }

    can_rx_buffer_lock = 0;
}

// 동작 중엔 지속적으로 호출되는 메서드
void Update()
{
    // Auto rotate 제어
    if (auto_rotate_flag == 1) {
        main_loop_count += direction * 50;

        // 헤더 설정
        can_tx_data[0] = 0xFF;
        can_tx_data[1] = 0xFF;
        can_tx_data[2] = 0xFD;

        // Reserved
        can_tx_data[3] = 0x00;

        // ID
        can_tx_data[4] = 0x01;

        // Instruction
        can_tx_data[5] = 0x01;

        // Data
        for (uint8_t i = 0; i < 4; i++) {
            can_tx_data[6 + i] = (main_loop_count >> 8 * i) & 0xFF;
        }

        // CRC
        uint16_t crc16 = Calculate_CRC16_UMTS(can_tx_data, 0, 39);
        can_tx_data[40] = (uint8_t)(crc16 & 0xFF); // LSB
        can_tx_data[41] = (uint8_t)((crc16 >> 8) & 0xFF); // LSB

        if(can_tx_wait_count >= 20) {
            Transmit_MultiByte_Through_CAN(&CAN_PORT, can_tx_data, UART_BUFFER_SIZE);
            can_tx_wait_count = 0;
        }


        if (main_loop_count >= 200000) {
            direction = -1;
        }

        if (main_loop_count <= -200000) {
            direction = 1;
        }

        can_tx_wait_count++;
    }
}

#endif // __HIGH_LEVEL_CONTROLLER_CALLBACKS__