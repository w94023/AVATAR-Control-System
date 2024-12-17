using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityUI;

using System.Text;

public class Controller : MonoBehaviour
{
    // SceneManager 인스턴스 저장
    public SceneManager sceneManager;

    // PacketManager 인스턴스 저장
    public PacketManager packetManager;

    [Space(20)]

    // Vibrator 강도 조절할 슬라이더 저장
    public Slider motorIntensitySlider;

    [Space(20)]
    public GameObject sensorStopLine;

    [Space(20)]
    public ToggleButton torqueEnableToggleButton;
    public ToggleButton ledEnableToggleButton;
    public ToggleButton autoRotateToggleButton;
    public ToggleButton emergencyStopToggleButton;

    // Auto rotate 선택 버튼
    
    public bool _useAutoRotation = false;
    public bool _currentAutoRotation = false;

    private byte _motor_id = 0x01;

    private void Awake()
    {
        // 이벤트 연결
        sceneManager.onConnected += OnConnected;

        // 모터 intensity 변경 이벤트 연결
        motorIntensitySlider.onValueChanged.AddListener(OnSliderValueChanged);

        // Auto rotate toggle 이벤트 연결
        torqueEnableToggleButton.onClick.AddListener(ChangeTorqueEnableState);
        ledEnableToggleButton.onClick.AddListener(ChangeLedEnableState);
        autoRotateToggleButton.onClick.AddListener(ChangeAutoRotateState);
        emergencyStopToggleButton.onClick.AddListener(ChangeEmergencyStopState);
    }


    private void OnConnected()
    {
        ChangeTorqueEnableState();
        ChangeLedEnableState();
        ChangeAutoRotateState();
        ChangeEmergencyStopState();
    }

    private void OnSliderValueChanged(float value)
    {
        // 회전 각도 : -501923~501923으로 사용 (4byte)
        // Max limit이 501433에 걸려있음
        int goalPositionValue = (int)(value / 180  * 501433);
        packetManager.SetTargetPosition(_motor_id, goalPositionValue);
    }

    private void ChangeAutoRotateState()
    {
        byte useAutoRotation = (autoRotateToggleButton.value == 1) ? (byte)0x01 : (byte)0x00;
        packetManager.SetAutoRotate(_motor_id, useAutoRotation);
    }

    private void ChangeTorqueEnableState()
    {
        byte torqueEnable = (torqueEnableToggleButton.value == 1) ? (byte)0x01 : (byte)0x00;
        packetManager.SetTorqueEnable(_motor_id, torqueEnable);
    }

    private void ChangeLedEnableState()
    {
        byte ledEnable = (ledEnableToggleButton.value == 1) ? (byte)0xFF : (byte)0x00;
        packetManager.SetLEDEnable(_motor_id, ledEnable);
    }

    private void ChangeEmergencyStopState()
    {
        byte useEmergencyStop = (emergencyStopToggleButton.value == 1) ? (byte)0x01 : (byte)0x00;
        if (emergencyStopToggleButton.value == 1) {
            sensorStopLine.SetActive(true);
        }
        else {
            sensorStopLine.SetActive(false);
        }

        int sensorStopValue = 2048;

        packetManager.SetEmergencyStop(_motor_id, useEmergencyStop, sensorStopValue);
    }
}
