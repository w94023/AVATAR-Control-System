using UnityEngine;
using UnityEditor;
using SerialManager;

[CustomEditor(typeof(SerialHandleUnity))]
internal class SerialHandleUnityEditor : Editor
{
    private string      dataToSend     = "";
    private float       CPUUsage       = 0;
    private float       memoryRemained = 0;

	public override void OnInspectorGUI()
    {   
        base.OnInspectorGUI();
        serializedObject.Update();

        SerialHandleUnity _target = (SerialHandleUnity)target;

        EditorGUILayout.Space();
        GUILayout.Label("Device options", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("deviceType"), new GUIContent("deviceType"), true);
        EditorGUI.indentLevel = 2;
        if (_target.deviceType == SerialHandleUnity.DeviceType.USB) {
            EditorGUILayout.PropertyField(serializedObject.FindProperty("portName"), new GUIContent("portName"), true);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("baudRate"), new GUIContent("baudRate"), true);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("dataBit"), new GUIContent("dataBit"), true);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("parity"), new GUIContent("parity"), true);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("stopBit"), new GUIContent("stopBit"), true);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("flowControl"), new GUIContent("flowControl"), true);
        }
        else if (_target.deviceType == SerialHandleUnity.DeviceType.DAQ) {
            EditorGUILayout.PropertyField(serializedObject.FindProperty("deviceName"), new GUIContent("deviceName"), true);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("AIPorts"), new GUIContent("AIPorts"), true);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("AOPorts"), new GUIContent("AOPorts"), true);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("DPorts"),  new GUIContent("DPorts"), true);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("lines"),   new GUIContent("lines"), true);
        }
        else if (_target.deviceType == SerialHandleUnity.DeviceType.BTClassic) {
            EditorGUILayout.PropertyField(serializedObject.FindProperty("deviceName"), new GUIContent("deviceName"), true);
        }
        else if (_target.deviceType == SerialHandleUnity.DeviceType.BLE) {
            EditorGUILayout.PropertyField(serializedObject.FindProperty("deviceName"), new GUIContent("deviceName"), true);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("uuidService"), new GUIContent("uuidService"), true);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("uuidTx"), new GUIContent("uuidTx"), true);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("uuidRx"), new GUIContent("uuidRx"), true);
        }
        else if (_target.deviceType == SerialHandleUnity.DeviceType.TCP) {
            EditorGUILayout.PropertyField(serializedObject.FindProperty("socketType"), new GUIContent("socketType"), true);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("port"), new GUIContent("port"), true);
        }
        EditorGUI.indentLevel = 0;

        EditorGUILayout.Space();
        GUILayout.Label("Logging option", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_logLevel"), new GUIContent("logLevel"), true);
        EditorGUI.indentLevel = 0;

        EditorGUILayout.Space();
        GUILayout.Label("Connection option", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("__isConnected"), new GUIContent("isConnected"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("useCloseMessage"), new GUIContent("useCloseMessage"), true);
        if (_target.useCloseMessage) EditorGUILayout.PropertyField(serializedObject.FindProperty("closeMessage"), new GUIContent("closeMessage"), true);
        EditorGUI.indentLevel = 0;

        EditorGUILayout.Space();
        GUILayout.Label("Communication options", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("getPPSOnDataReceived"), new GUIContent("getPPSOnDataReceived"), true);
        if (_target.getPPSOnDataReceived) EditorGUILayout.PropertyField(serializedObject.FindProperty("__PPS"), new GUIContent("PPS"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_getReceivedBufferLength"), new GUIContent("getReceivedBufferLength"), true);
        if (_target.getReceivedBufferLength) EditorGUILayout.PropertyField(serializedObject.FindProperty("__receivedBufferLength"), new GUIContent("receivedBufferLength"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_flushRatio"), new GUIContent("flushRatio"), true);
        EditorGUI.indentLevel = 0;

        EditorGUILayout.Space();
        GUILayout.Label("Packet options", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("receiveByteSize"),   new GUIContent("receiveByteSize"),   true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("receiveBufferSize"), new GUIContent("receiveBufferSize"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_usePacketLength"),  new GUIContent("usePacketLength"),   true);
        if (_target.usePacketLength) {
            EditorGUILayout.PropertyField(serializedObject.FindProperty("packetLength"),  new GUIContent("packetLength"),      true);
        }
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_useStopByte"),      new GUIContent("useStopByte"),       true);
        if (_target.useStopByte) {
            EditorGUILayout.PropertyField(serializedObject.FindProperty("stopByte"),      new GUIContent("stopByte"),          true);
        }
        EditorGUILayout.PropertyField(serializedObject.FindProperty("encodingType"),      new GUIContent("encodingType"),      true);
        EditorGUI.indentLevel = 0;

        EditorGUILayout.Space();
        GUILayout.Label("Resources management options", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_usePPSLimit"),    new GUIContent("usePPSLimit"),    true);
        if (_target.usePPSLimit) {
            EditorGUILayout.PropertyField(serializedObject.FindProperty("_PPSLimit"), new GUIContent("PPSLimit"), true);
        }
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_useCPULimit"),    new GUIContent("useCPULimit"),    true);
        if (_target.useCPULimit) {
            EditorGUILayout.PropertyField(serializedObject.FindProperty("_CPULimit"), new GUIContent("CPULimit"), true);
        }
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_useMemoryLimit"), new GUIContent("useMemoryLimit"), true);
        if (_target.useMemoryLimit) {
            EditorGUILayout.PropertyField(serializedObject.FindProperty("_memoryLimit"), new GUIContent("memoryLimit"), true);
        }
        EditorGUI.indentLevel = 0;

        EditorGUILayout.Space();
        GUILayout.Label("Methods", EditorStyles.boldLabel);
        
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.BeginHorizontal();
        GUILayout.Space(15f);
        if (GUILayout.Button("ScanDevices")) {
            _target.ScanDevices();
        }
        EditorGUILayout.EndHorizontal();
        EditorGUILayout.BeginHorizontal();
        GUILayout.Space(15f);
        if (GUILayout.Button("Connect")) {
            _target.Connect();
        }
        EditorGUILayout.EndHorizontal();
        EditorGUILayout.BeginHorizontal();
        GUILayout.Space(15f);
        if (GUILayout.Button("Disconnect")) {
            _target.Disconnect();
        }
        EditorGUILayout.EndHorizontal();
        dataToSend = EditorGUILayout.TextField("Data to send", dataToSend);
        EditorGUILayout.BeginHorizontal();
        GUILayout.Space(15f);
        if (GUILayout.Button("SendData")) {
            _target.SendData(dataToSend);
        }
        EditorGUILayout.EndHorizontal();
        EditorGUILayout.TextField("CPU usage (%)",     CPUUsage.ToString("0.00"));
        EditorGUILayout.TextField("RAM remained (MB)", memoryRemained.ToString("0"));
        EditorGUILayout.BeginHorizontal();
        GUILayout.Space(15f);
        if (GUILayout.Button("CheckResources")) {
            float[] resources = _target.CheckResources();
            CPUUsage = resources[0];
            memoryRemained = resources[1];
        }
        EditorGUILayout.EndHorizontal();
        EditorGUI.indentLevel = 0;

        EditorGUILayout.Space();
        GUILayout.Label("Events", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("onScanEnded"), new GUIContent("onScanEnded"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("onConnected"), new GUIContent("onConnected"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("onConnectionFailed"), new GUIContent("onConnectionFailed"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("onDisconnected"), new GUIContent("onDisconnected"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("onDataReceived"), new GUIContent("onDataReceived"), true);
        EditorGUI.indentLevel = 0;

        serializedObject.ApplyModifiedProperties();
    }
}
