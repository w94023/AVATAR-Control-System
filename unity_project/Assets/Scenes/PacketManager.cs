using System;
using System.Text;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PacketManager : MonoBehaviour
{
    public SerialDeviceHandler deviceHandle;

    public int packetLength = 42;
    // private byte[] _packet;
    private byte[] _packet = new byte[42];

    private void Awake()
    {
        _packet = new byte[packetLength];
    }

    ////////////////////////////////////////////////
    //                  COMMON                    //
    ////////////////////////////////////////////////

    private byte CalculateChecksum8(byte[] packet, int startIndex, int endIndex)
    {
        int checksum = 0;
        for (int i = startIndex; i < endIndex + 1; i++) {
            checksum += packet[i];
        }
        return (byte)((~(checksum & 0xFF)) & 0xFF);
    }

    private int CalculateCRC16(byte[] data, int startIndex, int endIndex)
    {
        ushort crc = 0x0000;        // 초기 CRC 값
        ushort polynomial = 0x8005; // 다항식

        for (int i = startIndex; i <= endIndex; i++) // 범위 내의 데이터 처리
        {
            crc ^= (ushort)(data[i] << 8); // MSB 위치에 XOR

            for (int bit = 0; bit < 8; bit++)
            {
                if ((crc & 0x8000) != 0) // MSB가 1인지 확인
                {
                    crc = (ushort)((crc << 1) ^ polynomial);
                }
                else
                {
                    crc <<= 1;
                }
            }

            crc &= 0xFFFF; // 16비트로 유지
        }

        return (int)crc;
    }

    private void SetHeader()
    {
        _packet[0] = 0xFF;
        _packet[1] = 0xFF;
        _packet[2] = 0xFD;
        _packet[3] = 0x00;
    }

    private void SetCRC16()
    {
        int crc16 = CalculateCRC16(_packet, 0, 39);
        _packet[40] = (byte)(crc16 & 0xFF);
        _packet[41] = (byte)((crc16 >> 8) & 0xFF);
    }

    ////////////////////////////////////////////////
    //                   Write                    //
    ////////////////////////////////////////////////

    public void SetTargetPosition(byte id, int value)
    {
        SetHeader();
        
        // ID
        _packet[4] = id;
        // Instruction
        _packet[5] = 0x01;
        // Data
        _packet[6] = (byte)(value & 0xFF);
        _packet[7] = (byte)((value >> 8) & 0xFF);
        _packet[8] = (byte)((value >> 16) & 0xFF);
        _packet[9] = (byte)((value >> 24) & 0xFF);

        SetCRC16();

        // 전송
        deviceHandle.SendPacket(_packet);
    }

    public void SetAutoRotate(byte id, byte value)
    {
        SetHeader();
        
        // ID
        _packet[4] = id;
        // Instruction
        _packet[5] = 0x02;
        // Data
        _packet[6] = value;

        SetCRC16();

        // 전송
        deviceHandle.SendPacket(_packet);
    }

    public void SetTorqueEnable(byte id, byte value)
    {
        SetHeader();
        
        // ID
        _packet[4] = id;
        // Instruction
        _packet[5] = 0x03;
        // Data
        _packet[6] = value;

        SetCRC16();

        // 전송
        deviceHandle.SendPacket(_packet);
    }

    public void SetLEDEnable(byte id, byte value)
    {
        SetHeader();
        
        // ID
        _packet[4] = id;
        // Instruction
        _packet[5] = 0x04;
        // Data
        _packet[6] = value;

        SetCRC16();

        // 전송
        deviceHandle.SendPacket(_packet);
    }

    public void SetEmergencyStop(byte id, byte enable, int sensorValue)
    {
        SetHeader();
        
        // ID
        _packet[4] = id;
        // Instruction
        _packet[5] = 0x05;
        // Data
        _packet[6] = enable;
        _packet[7] = (byte)(sensorValue & 0xFF);
        _packet[8] = (byte)((sensorValue >> 8) & 0xFF);

        SetCRC16();

        // 전송
        deviceHandle.SendPacket(_packet);
    }

    ////////////////////////////////////////////////
    //                    Read                    //
    ////////////////////////////////////////////////

    public bool CheckPacketValidity(string packet, ref int[] intTokens)
    {
        bool result = false;

        string[] tokens = packet.Split('-');

        // 패킷 길이 검사
        if (tokens.Length != packetLength) {
            return result;
        }

        // 헤더 검사
        if (tokens[packetLength-3] != "FF" || tokens[packetLength-2] != "FF" || tokens[packetLength-1] != "FD") {
            return result;
        }

        // intTokens 길이 검사
        if (intTokens.Length != packetLength) {
            return result;
        }

        // intTokens에 패킷 데이터 변환해서 붙여넣기
        for (int i = 0; i < tokens.Length; i++) {
            intTokens[i] = Convert.ToInt32(tokens[i], 16);
        }

        // CRC 검사
        byte[] crcPacket = new byte[packetLength-2];
        crcPacket[0] = (byte)0xFF;
        crcPacket[1] = (byte)0xFF;
        crcPacket[2] = (byte)0xFD;
        for (int i = 3; i < packetLength-2; i++) {
            crcPacket[i] = (byte)intTokens[i-3];
        }
        int crc16 = CalculateCRC16(crcPacket, 0, packetLength-3);
        if (((byte)(crc16 & 0xFF)) != (byte)intTokens[packetLength-5]) {
            return result;
        }
        if (((byte)((crc16 >> 8) & 0xFF)) != (byte)intTokens[packetLength-4]) {
            return result;
        }

        // 결과 반환
        result = true;
        return result;
    }
}
