#ifndef __PACKET__
#define __PACKET__

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

////////////////////////////////////////////////
//                유효성 검사                 //
////////////////////////////////////////////////

uint8_t Calculate_CRC8(uint8_t *data, uint8_t start_index, uint8_t end_index)
{
    // CRC-8 계산 함수
    uint8_t crc = 0x00; // 초기값 설정

    for (uint8_t i = start_index; i < end_index + 1; i++) {
        crc ^= data[i]; // 데이터 바이트를 CRC와 XOR
        for (uint8_t j = 0; j < 8; j++) { // 각 비트를 반복적으로 처리
            if (crc & 0x80) { // 최상위 비트가 1인지 확인
                crc = (crc << 1) ^ 0x07; // 다항식 XOR
            } else {
                crc <<= 1;
            }
        }
    }
    return crc; // 계산된 CRC 값 반환
}

uint16_t Calculate_CRC16_UMTS(uint8_t *data, uint8_t start_index, uint8_t end_index)
{
    uint16_t crc = 0x0000;       // 초기 CRC 값
    uint16_t polynomial = 0x8005; // 다항식

    for (size_t i = start_index; i < end_index + 1; i++) {
        crc ^= (uint16_t)(data[i] << 8); // MSB 위치에 XOR

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) { // MSB가 1인지 확인
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
        }

        crc &= 0xFFFF; // 16비트로 유지
    }

    return crc;
}

uint8_t Calculate_Checksum8(uint8_t *data, size_t start_index, size_t end_index)
{
    uint32_t checksum = 0;
    for (size_t i = start_index; i < end_index+1; i++) {
        checksum = checksum + data[i];
    }

    return ~(checksum & 0xFF);
}

int8_t Find_Header_Index(uint8_t* packet, size_t packet_length, uint8_t* header, size_t header_length)
{
    int8_t header_index = -1;
    
    // Search for the first and second occurrences of 0xFF, 0xFF
    for (size_t i = 0; i < packet_length - header_length; i++) {
        size_t true_value = 0;
        for (size_t j = 0; j < header_length; j++) {
            if (packet[i + j] == header[j]) {
                true_value++;
            }
        }

        if (true_value == header_length) {
            header_index = i;
            break;
        }
    }

    return header_index;
}

uint8_t Check_Packet_Matched(uint8_t* packet1, size_t start_index1, uint8_t* packet2, size_t start_index2, size_t length)
{
    for (size_t i = 0; i < length; i++) {
        if (packet1[i + start_index1] != packet2[i + start_index2]) {
            return 0;
        }
    }

    return 1;
}

uint8_t Check_Packet_Validity(uint8_t* packet, size_t packet_length, uint8_t byte, uint8_t type)
{
    // byte : 유효성 검사에 사용될 패킷 바이트 수
    // type : 0 > checksum, 1 > CRC
    uint8_t result = 0;

    // 헤더 + CRC16 바이트 수보다 작으면 오류 반환
    if (packet_length < 6) {
        return result;
    } 

    // 헤더 확인
    uint8_t header[] = {0xFF, 0xFF, 0xFD};
    int8_t header_index = Find_Header_Index(packet, packet_length, header, sizeof(header));
    if (header_index != 0) {
        return result;
    }

    if (type == 0) {
        // Checksum
        if (byte == 1){
            
            // Checksum8
            if (packet[packet_length-1] != Calculate_Checksum8(packet, 0, packet_length-2)) {
                return result;
            }
        }
        else {
            // 오류
            return result;
        }
    }
    else if (type == 1) {
        // CRC
        if (byte == 2) {

            // CRC16
            uint16_t crc16 = Calculate_CRC16_UMTS(packet, 0, packet_length-3);
            if (packet[packet_length-2] != (crc16 & 0xFF)) {
                return result;
            }
            if (packet[packet_length-1] != ((crc16 >> 8) & 0xFF)) {
                return result;
            }
        }

        else {
            // 오류
            return result;
        }
    }
    else {
        // 오류
        return result;
    }

    result = 1;
    return result;
}

////////////////////////////////////////////////
//                 패킷 구성                  //
////////////////////////////////////////////////

void Delete_Packet_From_Buffer(uint8_t* buffer, size_t buffer_length, size_t start_index, size_t end_index)
{
    size_t shift_length = end_index - start_index + 1;

    // 데이터 앞으로 당기기
    memmove(buffer + start_index, buffer + end_index + 1, buffer_length - end_index - 1);
    // 남은 영역을 '\0'으로 채우기
    memset(buffer + buffer_length - shift_length, '\0', shift_length);
}

void Int32_To_Packet(uint8_t* packet, size_t start_index, size_t end_index, int32_t* value)
{
    // int32_t를 uint8_t 배열에 작성하기 위해서는 4자리가 필요
    if ((end_index - start_index + 1) != 4) {
        return;
    }

    packet[start_index + 0] = *value & 0xFF;
    packet[start_index + 1] = (*value >> 8) & 0xFF;
    packet[start_index + 2] = (*value >> 16) & 0xFF;
    packet[start_index + 3] = (*value >> 24) & 0xFF;
}

void Int16_To_Packet(uint8_t* packet, size_t start_index, size_t end_index, int16_t* value)
{
    // int16_t를 uint8_t 배열에 작성하기 위해서는 2자리가 필요
    if ((end_index - start_index + 1) != 2) {
        return;
    }

    packet[start_index + 0] = *value & 0xFF;
    packet[start_index + 1] = (*value >> 8) & 0xFF;
}

int32_t Packet_To_Int32(uint8_t* packet, size_t start_index, size_t end_index)
{
    int32_t value = 0;

    // int32_t를 uint8_t 배열에 작성하기 위해서는 4자리가 필요
    if ((end_index - start_index + 1) != 4) {
        return value;
    }

    value |= packet[start_index + 0];
    value |= (packet[start_index + 1] << 8);
    value |= (packet[start_index + 2] << 16);
    value |= (packet[start_index + 3] << 24);

    return value;
}

int16_t Packet_To_Int16(uint8_t* packet, size_t start_index, size_t end_index)
{
    int32_t value = 0;

    // int16_t를 uint8_t 배열에 작성하기 위해서는 2자리가 필요
    if ((end_index - start_index + 1) != 2) {
        return value;
    }

    value |= packet[start_index + 0];
    value |= (packet[start_index + 1] << 8);

    return value;
}

////////////////////////////////////////////////
//                 CAN 관련                   //
////////////////////////////////////////////////

#ifdef CAN_ENABLED

// CAN으로는 8바이트 데이터만 송신 가능
// 입력된 데이터를 8바이트로 쪼개서 송신하는 메서드
void Transmit_MultiByte_Through_CAN(CAN_HandleTypeDef* hcan, uint8_t* packet, size_t packet_length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8];
    uint16_t offset = 0;

    // 송신 헤더 설정
    tx_header.StdId = 0x123;          // 표준 ID
    tx_header.ExtId = 0x01;           // 확장 ID (사용하지 않음)
    tx_header.RTR = CAN_RTR_DATA;     // 데이터 프레임
    tx_header.IDE = CAN_ID_STD;       // 표준 ID
    tx_header.DLC = 8; // 송신 데이터 바이트
    tx_header.TransmitGlobalTime = DISABLE;

    uint8_t count = 0;
    uint8_t data_size = 6; // 1byte for id, 1byte for checksum

    while (offset < packet_length) {
        // 전송할 데이터 추출 (8바이트씩)
        uint16_t chunk_size = (packet_length - offset > data_size) ? data_size : packet_length - offset;

        memset(tx_data, '\0', sizeof(tx_data)); // 나머지 공간 0으로 초기화

        tx_data[0] = count;
        memcpy(tx_data + 1, packet + offset, chunk_size);
        tx_data[7] = Calculate_Checksum8(tx_data, 0, 6);

         // 현재 송신 데이터 길이

        // HAL_CAN_AddTxMessage(hcan, &txHeader, tx_data, &txMailbox);
        // CAN 송신
        if (HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
            // 송신 실패 처리
            return;
        }

        // 송신 완료 대기
        while (HAL_CAN_IsTxMessagePending(hcan, tx_mailbox));

        offset += chunk_size; // 다음 데이터로 이동
        count++;
    }
}

#endif

////////////////////////////////////////////////
//                UART 관련                   //
////////////////////////////////////////////////

#ifdef UART_ENABLED

// RX FIFO 데이터 초기화하는 메서드
void Flush_UART_RX_FIFO(UART_HandleTypeDef *huart, size_t timeout)
{
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
        uint8_t dummy;
        HAL_UART_Receive(huart, &dummy, 1, timeout); // 수신된 데이터 읽어서 버림
    }
}

#endif

#endif // __PACKET__