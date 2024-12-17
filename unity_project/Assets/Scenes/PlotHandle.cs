using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityPlotter;

public class PlotHandle : MonoBehaviour
{
    // sceneManager 인스턴스 저장
    public SceneManager sceneManager;
    public SerialDeviceHandler deviceHandler;

    // 데이터 플랏 인스턴스 저장
    public Plotter plotter;

    // 플랏 데이터
    private List<float> _plotData = new List<float>();
    private List<float> _plotData2 = new List<float>();
    private List<int> _stackData = new List<int>();
    private int _dataCountLimit = 500;

    public int[] receivedData = new int[15];

    private void Awake()
    {
        sceneManager.onConnected    += Clear;
        sceneManager.onDisconnected += Clear;
        // sceneManager.onDataReceived += AnalyzePacket;

        Clear();
    }

    private void Clear()
    {
        // plot data initialization
        _plotData.Clear();
        _plotData2.Clear();
        plotter.Clear();
    }

    // 두 16진수 문자열을 하나의 정수로 조합
    public int CombineLSBMSB(string lsb, string msb)
    {
        // LSB와 MSB를 정수로 변환
        int lsbValue = Convert.ToInt32(lsb, 16); // 하위 바이트
        int msbValue = Convert.ToInt32(msb, 16); // 상위 바이트

        // MSB를 상위 8비트로 이동시키고 LSB를 결합
        return (msbValue << 8) | lsbValue;
    }

    private void PlotData()
    {
        _plotData.Add((float)receivedData[4]);
        _plotData2.Add((float)deviceHandler.targetPosition);

        if (_plotData.Count > _dataCountLimit) {
            _plotData.RemoveAt(0);
            _plotData2.RemoveAt(0);
        }

        plotter.Plot(_plotData.ToArray(), lineWidth : 5f);
        plotter.Plot(_plotData2.ToArray(), lineWidth : 5f);
        plotter.Draw();
    }
}
