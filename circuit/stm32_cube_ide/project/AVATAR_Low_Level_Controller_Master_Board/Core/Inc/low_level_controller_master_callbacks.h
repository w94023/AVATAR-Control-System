#ifndef __LOW_LEVEL_CONTROLLER_MASTER_CALLBACKS__
#define __LOW_LEVEL_CONTROLLER_MASTER_CALLBACKS__

#include "low_level_controller_master_main.h"

void CAN_RX_Callback()
{
    osSignalWait(0x01, osWaitForever);

    if (Check_Packet_Validity(can_rx_data, SPI_BUFFER_SIZE, 2, 1) == 1) {
        if (spi_flag == 1) {
            // Master에게 부여된 request인 경우
            if (can_rx_data[5] == 0x05) {
                // 이전 요청에 대한 수신이 완료된 경우에만 송신
                if (on_response_received == 1) {
                    on_response_received = 0;
                    memset(requested_data, '\0', SPI_BUFFER_SIZE);
                    memcpy(requested_data, can_rx_data + 6, 3);

                    memset(spi_tx_data, '\0', SPI_BUFFER_SIZE);
                    memcpy(spi_tx_data, can_rx_data, SPI_BUFFER_SIZE);

                    // Checksum 계산
                    spi_tx_data[41] = Calculate_Checksum8(spi_tx_data, 0, 40);

                    spi_flag = 0;
                }
            }
            else {
                memset(spi_tx_data, '\0', SPI_BUFFER_SIZE);
                memcpy(spi_tx_data, can_rx_data, SPI_BUFFER_SIZE);

                // Checksum 계산
                spi_tx_data[41] = Calculate_Checksum8(spi_tx_data, 0, 40);

                spi_flag = 0;
            }
        }

        HAL_GPIO_TogglePin(CAN_LED_GPIO_PORT, CAN_LED_GPIO_PIN);
    }

    can_rx_buffer_lock = 0;    
}

uint8_t Analyze_SPI_Packet()
{
    uint8_t result = 0;

    // Dummy data 확인
    uint32_t sum = 0;
    for (size_t i = 0; i < SPI_BUFFER_SIZE; i++) {
        sum += spi_rx_data[i];
    }
    if (sum == 0) {
        return result;
    }

    if (Check_Packet_Validity(spi_rx_data, SPI_BUFFER_SIZE, 1, 0) == 0) {
        return result;
    }

    if (spi_rx_data[5] == 0x05) {
        if (Check_Packet_Matched(spi_rx_data, 6, requested_data, 0, 3)) {
            // Emegency stop 세팅 완료 패킷 수신
            on_response_received = 1;
        }
    }

    // Pacekt count stack
    if ((spi_rx_data[5] & 0b10000000) != 0) {
        uart_work_counting_flag++;
    }
    if ((spi_rx_data[5] & 0b01000000) != 0) {
        adc_work_counting_flag++;
    }

    result = 1;
    return result;
}

volatile uint16_t test_count = 0;

void SPI_Callback()
{
    // SPI 콜백 신호 대기
    osSignalWait(0x01, osWaitForever);

    uint8_t result = Analyze_SPI_Packet();
    
    if (can_tx_buffer_lock == 0 && result == 1) {

        // Instruction filtering
        if (spi_rx_data[5] != 0x40 && spi_rx_data[5] != 0x80 && spi_rx_data[5] != 0xC0) {
            spi_rx_buffer_lock = 0;
            return;
        }

        can_tx_buffer_lock = 1;
        memset(can_tx_data, '\0', SPI_BUFFER_SIZE);
        memcpy(can_tx_data, spi_rx_data, SPI_BUFFER_SIZE);
        osSignalSet(canTxTaskHandle, 0x01);
    }
    
    spi_rx_buffer_lock = 0;
}

void Can_TX_Callback()
{
    osSignalWait(0x01, osWaitForever);

    // 헤더 설정
    can_tx_buffer[0] = 0xFF;
    can_tx_buffer[1] = 0xFF;
    can_tx_buffer[2] = 0xFD;
    
    // 모터 ID 설정
    can_tx_buffer[3] = can_tx_data[4];
    
    // 컨트롤러 ID 설정
    can_tx_buffer[4] = 0x01;

    // Motor position
    can_tx_buffer[5] = 0x01;

    // Motor state data
    memcpy(can_tx_buffer + 6, can_tx_data + 6, 10);

    // ADC data
    memcpy(can_tx_buffer + 16, can_tx_data + 16, 20);

    // PPS
    can_tx_buffer[36] = (uint8_t)(uart_work_count & 0xFF);
    can_tx_buffer[37] = (uint8_t)((uart_work_count >> 8) & 0xFF);
    can_tx_buffer[38] = (uint8_t)(adc_work_count & 0xFF);
    can_tx_buffer[39] = (uint8_t)((adc_work_count >> 8) & 0xFF);

    // CRC
    uint16_t crc16 = Calculate_CRC16_UMTS(can_tx_buffer, 0, 39);
    can_tx_buffer[40] = crc16 & 0xFF;        // MSB
    can_tx_buffer[41] = (crc16 >> 8) & 0xFF; // LSB

    Transmit_MultiByte_Through_CAN(&hcan, can_tx_buffer, SPI_BUFFER_SIZE);

    can_tx_buffer_lock = 0;
}

#endif // __LOW_LEVEL_CONTROLLER_MASTER_CALLBACKS__