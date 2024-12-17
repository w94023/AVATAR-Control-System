using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityPlotter;

public class MotorStatePlotter : MonoBehaviour
{
    // SceneManager 인스턴스 참조
    public SceneManager sceneManager;

    [Space(20)]

    // Plotter 인스턴스 참조
    public Plotter positionPlotter;
    public Plotter velocityPlotter;
    public Plotter loadPlotter;

    [Space(20)]

    // PPS 정보 표시를 위한 텍스트 참조
    public Text ppsText;

    // USB로 수신한 모터 정보
    private int[] _motorState = new int[3];

    // Plot data
    private List<float> _positionPlotData = new List<float>();
    private List<float> _velocityPlotData = new List<float>();
    private List<float> _loadPlotData     = new List<float>();
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
        _positionPlotData.Clear();
        _velocityPlotData.Clear();
        _loadPlotData.Clear();

        // Plotter 초기화
        positionPlotter.Clear();
        velocityPlotter.Clear();
        loadPlotter.Clear();

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

    private int CombineBytesToInt16(int[] packet)
    {
        if (packet.Length != 2) {
            return 0;
        }

        // 16비트 정수 구성
        short result = (short)((packet[1] << 8) | packet[0]);
        return (int)result;
    }

    private void SetPlotLimitToData(List<float> data)
    {
        // Plot data가 plotter의 limit을 넘었을 경우 가장 첫 번재 데이터를 삭제하는 메서드
        if (data.Count > _plotLimit) {
            data.RemoveAt(0);
        }
    }

    private void OnDataReceived(int[] data)
    {
        // 수신한 패킷으로 부터 모터 정보 획득
        _motorState[0] = BytesToInt(new int[] { data[0], data[1], data[2], data[3] });
        _motorState[1] = BytesToInt(new int[] { data[4], data[5], data[6], data[7] });
        _motorState[2] = CombineBytesToInt16(new int[] { data[8], data[9] } );

        // 수신한 패킷으로 부터 PPS 정보 획득
        int pps = BytesToInt(new int[] { data[30], data[31] });

        // Plot 데이터 추가
        _positionPlotData.Add(_motorState[0]);
        _velocityPlotData.Add(_motorState[1]);
        _loadPlotData.Add(_motorState[2]);

        // Plotter limit 적용
        SetPlotLimitToData(_positionPlotData);
        SetPlotLimitToData(_velocityPlotData);
        SetPlotLimitToData(_loadPlotData);

        // Plotter 설정
        positionPlotter.Plot(_positionPlotData.ToArray(), lineWidth : 1f);
        positionPlotter.Draw();
        velocityPlotter.Plot(_velocityPlotData.ToArray(), lineWidth : 1f);
        velocityPlotter.Draw();
        loadPlotter.Plot(_loadPlotData.ToArray(), lineWidth : 1f);
        loadPlotter.Draw();

        // PPS text 설정
        ppsText.text = pps.ToString("000");
    }
}
