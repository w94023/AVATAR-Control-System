using System;
using System.Threading;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using SerialManager;

namespace SerialManager
{
    public class SerialHandleUnity : MonoBehaviour
    {
        public enum DeviceType
        {
            USB,
            DAQ,
            BTClassic,
            BLE,
            TCP,
        }

        public enum SocketType
        {
            Server
        }

        private SerialHandle _manager;

        private bool _isAlive = false;

        [HideInInspector] public DeviceType deviceType = DeviceType.USB;
        [HideInInspector] public SocketType socketType = SocketType.Server;
        // USB option
        [HideInInspector] public string      portName    = "";
        [HideInInspector] public int         baudRate    = 9600;
        [HideInInspector] public DataBit     dataBit     = DataBit.bit8;
        [HideInInspector] public Parity      parity      = Parity.None;
        [HideInInspector] public StopBit     stopBit     = StopBit.bit1;
        [HideInInspector] public FlowControl flowControl = FlowControl.None;
        // BT options
        [HideInInspector] public string deviceName = "";
        [HideInInspector] public string uuidService = "{0000FFE0-0000-1000-8000-00805F9B34FB}";
        [HideInInspector] public string uuidTx      = "{0000FFE1-0000-1000-8000-00805F9B34FB}";
        [HideInInspector] public string uuidRx      = "{0000FFE1-0000-1000-8000-00805F9B34FB}";
        // DAQ options
        [HideInInspector] public int[] AIPorts;
        [HideInInspector] public int[] AOPorts;
        [HideInInspector] public int[] DPorts;
        [HideInInspector] public int[] lines;
        // TCP options
        [HideInInspector] public int port;

        [HideInInspector]                 public  bool         getPPSOnDataReceived = false;
                                          public  int          PPS { get { return _PPS; } }
                                          private int         _PPS = 0;
        [SerializeField, HideInInspector] private int        __PPS = 0;
        public bool getReceivedBufferLength { get { return _getReceivedBufferLength; } set { _getReceivedBufferLength = value; _manager.getReceivedBufferLength = value; } }
        [SerializeField, HideInInspector] private bool  _getReceivedBufferLength = false;
                                          public  int          receivedBufferLength { get { return _receivedBufferLength; } }
                                          private int         _receivedBufferLength = 0;
        [SerializeField, HideInInspector] private int        __receivedBufferLength = 0;
        public float flushRatio { get { return _flushRatio; } set { _flushRatio = GetClampedFloat(value, 0.01f, 1.0f); _manager.flushRatio = _flushRatio; } }
        [SerializeField, HideInInspector] private float _flushRatio = 0.9f;

                                          public  bool         isConnected { get { return _isConnected; } }
                                          private bool        _isConnected = false;
        [SerializeField, HideInInspector] private bool       __isConnected = false;
        [HideInInspector] public bool   useCloseMessage = false;
        [HideInInspector] public byte[] closeMessage;

        [HideInInspector]                  public  int  logLevel { get { return _logLevel; } set { _logLevel = value; if (_manager != null) _manager.logLevel = value; } }
        [SerializeField, HideInInspector]  private int _logLevel = 1;

        // Packet options
        [HideInInspector] public  int          receiveByteSize      = 1;
        [HideInInspector] public  int          receiveBufferSize    = 10000;
        [HideInInspector] public  byte[]       stopByte             = new byte[0];
        [HideInInspector] public  int          packetLength         = 0;
        [HideInInspector] public  EncodingType encodingType         = EncodingType.ASCII;
        public bool usePacketLength { get { return _usePacketLength; } set { _usePacketLength = value; _manager.usePacketLength = _usePacketLength; } }
        public bool useStopByte     { get { return _useStopByte;     } set { _useStopByte     = value; _manager.useStopByte     = _useStopByte;     } }
        [SerializeField, HideInInspector] private bool _usePacketLength = false;
        [SerializeField, HideInInspector] private bool _useStopByte     = false;

        // Resource limit options
        public bool   usePPSLimit    { get { return _usePPSLimit;    } set { _usePPSLimit    = value; _manager.usePPSLimit    = _usePPSLimit;    } }
        public bool   useCPULimit    { get { return _useCPULimit;    } set { _useCPULimit    = value; _manager.useCPULimit    = _useCPULimit;    } }
        public bool   useMemoryLimit { get { return _useMemoryLimit; } set { _useMemoryLimit = value; _manager.useMemoryLimit = _useMemoryLimit; } }
        public int    PPSLimit       { get { return _PPSLimit;       } set { _PPSLimit       = value; _manager.PPSLimit       = _PPSLimit;       } }
        public double CPULimit       { get { return _CPULimit;       } set { _CPULimit       = value; _manager.CPULimit       = _CPULimit;       } } 
        public int    memoryLimit    { get { return _memoryLimit;    } set { _memoryLimit    = value; _manager.memoryLimit    = _memoryLimit;    } }
        [SerializeField, HideInInspector] bool   _usePPSLimit    = false;
        [SerializeField, HideInInspector] bool   _useCPULimit    = false;
        [SerializeField, HideInInspector] bool   _useMemoryLimit = false;
        [SerializeField, HideInInspector] int    _PPSLimit       = 300;
        [SerializeField, HideInInspector] double _CPULimit       = 95;
        [SerializeField, HideInInspector] int    _memoryLimit    = 200;

        [HideInInspector] public UnityEvent<SerialLog>          onScanEnded;
        [HideInInspector] public UnityEvent                     onConnected;
        [HideInInspector] public UnityEvent                     onConnectionFailed;
        [HideInInspector] public UnityEvent                     onDisconnected;
        [HideInInspector] public UnityEvent<double, SerialData> onDataReceived;

        private SynchronizationContext _mainThreadContext;

        private void Awake()
        {
            _isAlive = true;
            _manager = new SerialHandle();
            _manager.onScanEnded        += OnScanEnded;
            _manager.onLogReceived      += OnLogReceived;
            _manager.onConnected        += OnConnected;
            _manager.onConnectionFailed += OnConnectionFailed;
            _manager.onDisconnected     += OnDisconnected;
            _manager.onDataReceived     += OnDataReceived;

            SetManagerConfig();

            _mainThreadContext = SynchronizationContext.Current;
        }

        private float GetClampedFloat(float value, float minValue, float maxValue)
        {
            if (value < minValue) return minValue;
            if (value > maxValue) return maxValue;
            return value;
        }

        private void SetManagerConfig()
        {
            if (_manager == null) return;
            _manager.getReceivedBufferLength = _getReceivedBufferLength;
            _flushRatio = GetClampedFloat(_flushRatio, 0.01f, 1.0f);
            _manager.flushRatio = _flushRatio;

            _manager.usePacketLength      = _usePacketLength;
            _manager.useStopByte          = _useStopByte;

            _manager.usePPSLimit          = _usePPSLimit;
            _manager.useCPULimit          = _useCPULimit;
            _manager.useMemoryLimit       = _useMemoryLimit;
            _manager.PPSLimit             = _PPSLimit;
            _manager.CPULimit             = _CPULimit;
            _manager.memoryLimit          = _memoryLimit;
        }

        private void OnValidate()
        {
            SetManagerConfig();

            if (_manager != null) {
                if (_manager.logLevel != _logLevel) {
                    _manager.logLevel = _logLevel;
                }
            }
            
            if (!_isConnected) {
                __PPS = 0;
                __receivedBufferLength = 0;
                __isConnected = false;
            }
            else {
                __PPS = _PPS;
                __receivedBufferLength = _receivedBufferLength;
                __isConnected = true;
            }
        }

        public void ScanDevices()
        {
            _manager.ScanDevices();
        }

        public void Connect()
        {
            _manager.logLevel             = _logLevel;
            _manager.getPPSOnDataReceived = getPPSOnDataReceived;
            _manager.receiveByteSize      = receiveByteSize;
            _manager.receiveBufferSize    = receiveBufferSize;
            _manager.encodingType         = encodingType;
            _manager.stopByte             = stopByte;
            _manager.packetLength         = packetLength;

            switch (deviceType) {
                case DeviceType.USB:       _manager.Connect(portName, baudRate, dataBit, parity, stopBit, flowControl); break;
                case DeviceType.DAQ:       _manager.Connect(deviceName, AIPorts, AOPorts, DPorts, lines); break;
                case DeviceType.BTClassic: _manager.Connect(deviceName); break;
                // case DeviceType.BLE:       
                //     UUID uuid = new UUID();
                //     uuid.service = uuidService;
                //     uuid.tx      = uuidTx;
                //     uuid.rx      = uuidRx;
                //     _manager.Connect(deviceName, uuid); break;
                case DeviceType.TCP : _manager.Connect(0, port); break;
            }
        }

        public void Disconnect()
        {
            if (useCloseMessage) _manager.SendData(closeMessage);
            _manager.Disconnect();
        }

        public void SendData(string data)
        {
            if (!_isAlive) return;
            _manager.SendData(data);
        }

        public void SendData(byte[] data)
        {
            if (!_isAlive) return;
            _manager.SendData(data);
        }

        public void SendData(double[] data)
        {
            if (!_isAlive) return;
            _manager.SendData(data);
        }

        public void SendData(bool[] data)
        {
            if (!_isAlive) return;
            _manager.SendData(data);
        }

        public float[] CheckResources()
        {
            if(!_isAlive) return new float[] { -1f, -1f };
            else          return _manager.CheckResources();
        }

        private void OnLogReceived(SerialLog e)
        {
            if (!_isAlive) return;
            _mainThreadContext.Post(state => { 
                if (e.log.Contains("Error")) {
                    Debug.LogError(e.log);
                }
                else {
                    Debug.Log(e.log);
                }
            }, null);
        }

        private void OnScanEnded(SerialLog e)
        {
            if (!_isAlive) return;
            onScanEnded?.Invoke(e);
        }

        private void OnConnected()
        {
            if (!_isAlive) return;
            onConnected?.Invoke();
            _isConnected = true;
            __isConnected = _isConnected;
        }

        private void OnConnectionFailed()
        {
            if (!_isAlive) return;
            onConnectionFailed?.Invoke();
            _isConnected = false;
            __isConnected = _isConnected;
        }

        private void OnDisconnected()
        {
            if (!_isAlive) return;
            onDisconnected?.Invoke();
            _isConnected = false;
            __isConnected = _isConnected;
        }

        private void OnDataReceived(double time, SerialData e)
        {
            if (!_isAlive) return;
            if (getPPSOnDataReceived) {
                _PPS  = _manager.PPS;
                __PPS = _manager.PPS;
            }
            if (getReceivedBufferLength) {
                _receivedBufferLength  = _manager.receivedBufferLength;
                __receivedBufferLength = _manager.receivedBufferLength;
                Debug.Log(_manager.receivedBufferLength);
            }
            
            onDataReceived?.Invoke(time, e);
        }

        private void OnDestroy()
        {
            _isAlive = false;
            if (useCloseMessage) _manager.SendData(closeMessage);
            _manager.Disconnect();
        }
    }
}
