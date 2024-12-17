using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityPlotter;

public class SensorDataPlotter : MonoBehaviour
{
    // SceneManager 인스턴스 참조
    public SceneManager sceneManager;

    [Space(20)]

    // 센서 애니메이션 오브젝트 참조
    public GameObject sensorObject;

    [Space(20)]

    // Plotter 인스턴스 참조
    public Plotter sensorDataPlotter;
    
    [Space(20)]

    // PPS 정보 표시를 위한 텍스트 참조
    public Text ppsText;

    // USB로 수신한 센서 정보
    private int _sensorData = 0;
    private List<int> _sensorDataStored = new List<int>(); // moving average 필터 적용을 위한 저장 데이터
    private int _windowSize = 10; // moving average 필터 window size

    // Plot data
    private List<float> _sensorPlotdata = new List<float>();
    private int _plotLimit = 500;

    private void Awake()
    {
        // 이벤트 연결
        sceneManager.onConnected    += Clear;
        sceneManager.onDisconnected += Clear;
        sceneManager.onDataReceived += OnDataReceived;

        // Plotter 초기화
        Clear();
    }

    private void Clear()
    {
        // Plot data 초기화
        _sensorPlotdata.Clear();

        // 센서 오브젝트 크기 초기화
        sensorObject.transform.localScale = new Vector3(100f, 100f, 100f);

        // Plotter 초기화
        sensorDataPlotter.Clear();

        // PPS 텍스트 초기화
        ppsText.text = "";
    }

    private int BytesToInt(int[] packet)
    {
        int value = 0;
        for (int i = 0; i < packet.Length; i++) {
            value |= packet[i] << (8 * i);
        }
        return value;
    }

    private void SetPlotLimitToData(List<float> data)
    {
        // Plot data가 plotter의 limit을 넘었을 경우 가장 첫 번재 데이터를 삭제하는 메서드
        if (data.Count > _plotLimit) {
            data.RemoveAt(0);
        }
    }

    float CalculateAverage(List<int> list)
    {
        if (list == null || list.Count == 0) {
            return 0f;
        }

        // 합계를 구하고 float로 나누기
        float sum = 0f;
        foreach (int number in list) {
            sum += number;
        }

        return sum / list.Count;
    }

    private void OnDataReceived(int[] data)
    {
        // 수신한 패킷으로 부터 센서 데이터 획득
        _sensorData = BytesToInt(new int[] { data[28], data[29] }); // ADC CH9
        _sensorDataStored.Add(_sensorData);
        if (_sensorDataStored.Count > _windowSize) {
            _sensorDataStored.RemoveAt(0);
        }

        float sensorData = CalculateAverage(_sensorDataStored);

        // 수신한 패킷으로 부터 PPS 정보 획득
        int pps = BytesToInt(new int[] { data[32], data[33] });

        // 센서 오브젝트 scale 조절
        float scale = (sensorData - 900f) / (2500f - 900f) * 100f + 100f;
        if (scale < 100f) scale = 100f;
        if (scale > 200f) scale = 200f;
        sensorObject.transform.localScale = new Vector3(scale, 100f, 100f);

        // Plot 데이터 추가
        _sensorPlotdata.Add(sensorData);

        // Plotter limit 적용
        SetPlotLimitToData(_sensorPlotdata);

        // Plotter 설정
        sensorDataPlotter.Plot(_sensorPlotdata.ToArray(), lineWidth : 2.5f);
        sensorDataPlotter.Draw();

        // PPS text 설정
        ppsText.text = pps.ToString("0000");
    }
}
