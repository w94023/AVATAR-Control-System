#ifndef __ROBOTIS__
#define __ROBOTIS__

#include "packet.h"

uint8_t robotis_motor_type = 0;
#define H54_200_S500 1
#define PH42_020_S300 2
#define MX_106 3

#define POSITION_4BYTE_MIN -501923
#define POSITION_4BYTE_MAX  501923
#define POSITION_2BYTE_MIN  0
#define POSITION_2BYTE_MAX  4096

#define VELOCITY_4BYTE_MIN -501923
#define VELOCITY_4BYTE_MAX  501923
#define VELOCITY_2BYTE_MIN  -1024
#define VELOCITY_2BYTE_MAX  1024

////////////////////////////////////////////////
//                  COMMON                    //
////////////////////////////////////////////////

uint16_t Map_4Byte_Position_To_2Byte(int32_t value)
{
    int64_t mapped_value = 0;
    mapped_value = POSITION_2BYTE_MIN + 
           (int64_t)(value - POSITION_4BYTE_MIN) * (POSITION_2BYTE_MAX - POSITION_2BYTE_MIN) / (POSITION_4BYTE_MAX - POSITION_4BYTE_MIN);

    if (mapped_value < POSITION_2BYTE_MIN) {
        mapped_value = POSITION_2BYTE_MIN;
    }
    else if (mapped_value > POSITION_2BYTE_MAX) {
        mapped_value = POSITION_2BYTE_MAX;
    }

    return mapped_value;
}

int32_t Map_2Byte_Position_To_4Byte(uint16_t value)
{
    int64_t mapped_value = 0;
    mapped_value = POSITION_4BYTE_MIN + 
           (int64_t)(value - POSITION_2BYTE_MIN) * (POSITION_4BYTE_MAX - POSITION_4BYTE_MIN) / (POSITION_2BYTE_MAX - POSITION_2BYTE_MIN);

    if (mapped_value < POSITION_4BYTE_MIN) {
        mapped_value = POSITION_4BYTE_MIN;
    }
    else if (mapped_value > POSITION_4BYTE_MAX) {
        mapped_value = POSITION_4BYTE_MAX;
    }

    return mapped_value;
}

int16_t Map_4Byte_Velocity_To_2Byte(int32_t value)
{
    int64_t mapped_value = 0;
    mapped_value = VELOCITY_2BYTE_MIN + 
           (int64_t)(value - VELOCITY_4BYTE_MIN) * (VELOCITY_2BYTE_MAX - VELOCITY_2BYTE_MIN) / (VELOCITY_4BYTE_MAX - VELOCITY_4BYTE_MIN);

    if (mapped_value < VELOCITY_2BYTE_MIN) {
        mapped_value = VELOCITY_2BYTE_MIN;
    }
    else if (mapped_value > VELOCITY_2BYTE_MAX) {
        mapped_value = VELOCITY_2BYTE_MAX;
    }

    return mapped_value;
}

////////////////////////////////////////////////
//                    V1                      //
////////////////////////////////////////////////

void Set_Position_Protocol_V1(uint8_t* packet, size_t length, uint8_t id, int32_t target_position)
{
    // 데이터 길이 부족
    if (length < 9) {
        return;
    }

    // -501922~501922(-180도~180도)로 표현되는 target position을 0~4096(0도 ~360도)으로 변환
    uint16_t mapped_position = Map_4Byte_Position_To_2Byte(target_position);

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = 0x04;
    packet[4] = 0x03;
    packet[5] = 0x1E;
    packet[6] = mapped_position & 0xFF;
    packet[7] = (mapped_position >> 8) & 0xFF;
    packet[8] = Calculate_Checksum8(packet, 2, 7);
}

////////////////////////////////////////////////
//                    V2                      //
////////////////////////////////////////////////

uint8_t Motor_Set_Request_Protocol_V2(uint8_t* packet, size_t length, uint8_t id, uint16_t address, int32_t value, uint8_t value_size)
{
    uint8_t result_packet_length = 0;

    // 데이터 길이 부족
    if (length < 10 + value_size + 2) {
        return result_packet_length;
    }

    // Header
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;

    // Reserved
    packet[3] = 0x00;

    // ID
    packet[4] = id;

    // Length (Parameters + 3)
    // Parameters : address + data
    uint16_t data_length = 3 + 2 + value_size;
    packet[5] = data_length & 0xFF; // LSB
    packet[6] = (data_length >> 8) & 0xFF; // MSB

    // Instruction
    packet[7] = 0x03; // write

    // Address
    packet[8] = address & 0xFF;        // LSB
    packet[9] = (address >> 8) & 0xFF; // MSB

    // // Data
    for (uint8_t i = 0; i < value_size; i++) {
        packet[10 + i] = (value >> 8 * i) & 0xFF;
    }

    // CRC
    uint16_t crc16 = Calculate_CRC16_UMTS(packet, 0, 10 + value_size - 1);
    packet[10 + value_size]     = (uint8_t)(crc16 & 0xFF); // LSB
    packet[10 + value_size + 1] = (uint8_t)((crc16 >> 8) & 0xFF); // LSB

    return 10 + value_size + 2;
}

uint8_t Get_Motor_Response_Protocol_V2(uint8_t* packet, size_t length, uint8_t id)
{
    uint8_t result = 0;

    // 패킷 길이 부족
    if (length < 11) {
        return result;
    }

    uint8_t header[] = {0xFF, 0xFF, 0xFD, 0x00};
    int8_t header_index = Find_Header_Index(packet, length, header, sizeof(header));

    // 헤더 찾지 못함
    if (header_index < 0) {
        return result;
    }

    // 헤더 이후로 유호한 패킷이 없음
    if (header_index + 11 > length) {
        return result;
    }

    // CRC 확인
    uint16_t crc = Calculate_CRC16_UMTS(packet, header_index, header_index + 8);
    if ((crc & 0xFF) != packet[header_index + 9]) {
        return result;
    }
    if (((crc >> 8) & 0xFF) != packet[header_index + 10]) {
        return result;
    }

    // ID 확인
    if (packet[header_index + 4] != id) {
        return result;
    }

    // Data length 확인
    uint8_t data_length = packet[header_index + 5] | (packet[header_index + 6] >> 8);
    if (data_length != 4) {
        return result;
    }

    // Status response 확인
    if (packet[header_index + 7] != 0x55) {
        return result;
    }

    // 에러 확인
    if (packet[header_index + 8] > 0x00 && packet[header_index + 8] <= 0x3F) {
        return result;
    }

    result = 1;
    return result;
}

void Motor_State_Request_Protocol_V2(uint8_t* packet, size_t length, uint8_t id)
{
    // 데이터 길이 부족
    if (length < 14) {
        return;
    }

    // Header
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;

    // Reserved
    packet[3] = 0x00;

    // ID
    packet[4] = id;

    // Length (7)
    packet[5] = 0x07;
    packet[6] = 0x00;

    // Instruction (read)
    packet[7] = 0x02;

    // Address (574)
    packet[8] = 0x3E; // LSB
    packet[9] = 0x02; // MSB

    // Target byte (12)
    packet[10] = 0x0A;
    packet[11] = 0x00;

    // CRC
    uint16_t crc16 = Calculate_CRC16_UMTS(packet, 0, 11);
    packet[12] = (uint8_t)(crc16 & 0xFF);        // LSB
    packet[13] = (uint8_t)((crc16 >> 8) & 0xFF); // MSB
}

#ifdef UART_ENABLED

void Motor_Position_Set_Protocol_V2(UART_HandleTypeDef* uart_port, uint8_t id, int32_t position, uint8_t* tx_packet, size_t tx_packet_length, uint8_t* rx_packet, size_t rx_packet_length)
{
    memset(rx_packet, '\0', rx_packet_length);
    uint8_t valid_tx_packet_length = Motor_Set_Request_Protocol_V2(tx_packet, tx_packet_length, id, 564, position, 4);
    Flush_UART_RX_FIFO(uart_port, 10);
    HAL_UART_Receive_IT(uart_port, rx_packet, 11);
    HAL_UART_Transmit_IT(uart_port, tx_packet, valid_tx_packet_length);
}

void Motor_Torque_Enable_Set_Protocol_V2(UART_HandleTypeDef* uart_port, uint8_t id, uint8_t enable, uint8_t* tx_packet, size_t tx_packet_length, uint8_t* rx_packet, size_t rx_packet_length)
{
    memset(rx_packet, '\0', rx_packet_length);
    uint8_t valid_tx_packet_length = Motor_Set_Request_Protocol_V2(tx_packet, tx_packet_length, id, 512, enable, 1);
    Flush_UART_RX_FIFO(uart_port, 10);
    HAL_UART_Receive_IT(uart_port, rx_packet, 11);
    HAL_UART_Transmit_IT(uart_port, tx_packet, valid_tx_packet_length);
}

void Motor_LED_Enable_Set_Protocol_V2(UART_HandleTypeDef* uart_port, uint8_t id, uint8_t enable, uint8_t* tx_packet, size_t tx_packet_length, uint8_t* rx_packet, size_t rx_packet_length)
{
    // Red LED만 사용
    memset(rx_packet, '\0', rx_packet_length);
    uint8_t valid_tx_packet_length = Motor_Set_Request_Protocol_V2(tx_packet, tx_packet_length, id, 513, enable, 1);
    Flush_UART_RX_FIFO(uart_port, 10);
    HAL_UART_Receive_IT(uart_port, rx_packet, 11);
    HAL_UART_Transmit_IT(uart_port, tx_packet, valid_tx_packet_length);
}

#endif

// uint8_t Set_Motor_State_Request_Protocol_V2(uint8_t* packet, size_t length, uint8_t id, uint8_t type)
// {
//     // Type : bit로 계산
//     // index 0 : current, velocity, position
//     // index 1 : torque enable
//     // 나머지 : 사용하지 않음
//     // 모든 인덱스의 값이 1이면, 최상위 인덱스 결과만 반환

//     // 헤더 설정
//     packet[0] = 0xFF;
//     packet[1] = 0xFF;
//     packet[2] = 0xFD;
//     packet[3] = 0x00;

//     // ID 설정
//     packet[4] = id;

//     // Data length 설정

//     // Instruction 설정
//     packet[7] = 0x02; // read

//     uint16_t start_address = 0;

//     if ((type & 0b00000010) != 0) {
//         // Torque enable 설정

//         // Address 설정
//         start_address = 512;
//         packet[8] = start_address & 0xFF;
//         packet[9] = (start_address >> 8) & 0xFF;
//     }

// }
uint8_t Get_Motor_State_Protocol_V2(uint8_t* packet, size_t length, uint8_t id, int16_t *current, int32_t *velocity, int32_t *position)
{
    uint8_t result = 0;

    // 데이터 길이 부족
    if (length < 21) {
        return result;
    }

    if (packet[0] != 0xFF || packet[1] != 0xFF || packet[2] != 0xFD || packet[3] != 0x00) {
        return result;
    }

    // ID 확인
    if (packet[4] != id) {
        return result;
    }

    uint16_t data_length = packet[5] | (packet[6] >> 8);

    // Data length 확인
    if (data_length != 14) {
        return result;
    }

    // Response 확인
    if (packet[7] != 0x55) {
        return result;
    }
    
    // 에러 확인
    if (packet[8] > 0x00 && packet[8] <= 0x3F) {
        return result;
    }

    // CRC 확인
    uint16_t crc = Calculate_CRC16_UMTS(packet, 0, 18);
    if ((crc & 0xFF) != packet[19]) {
        return result;
    }
    if (((crc >> 8) & 0xFF) != packet[20]) {
        return result;
    }

    *current = packet[9] | (packet[10] << 8);
    *velocity = packet[11] | (packet[12] << 8) | (packet[13] << 16) | (packet[14] << 24);
    *position = packet[15] | (packet[16] << 8) | (packet[17] << 16) | (packet[18] << 24);

    result = 1;
    return result;
}

#endif // __ROBOTIS__