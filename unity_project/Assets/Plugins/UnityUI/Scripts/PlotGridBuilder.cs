using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityUI;

namespace UnityUI
{
    [RequireComponent(typeof(RectTransform))]
    [ExecuteInEditMode]
    public class PlotGridBuilder : MonoBehaviour
    {
        public enum Mode
        {
            Horizontal,
            Vertical
        };

        [SerializeField, HideInInspector] private Mode  _mode              = Mode.Horizontal; public Mode  mode              { set { _mode              = value; Refresh(); } get { return _mode;              } }
        [SerializeField, HideInInspector] private bool  _useAxisLine       = true;            public bool  useAxisLine       { set { _useAxisLine       = value; Refresh(); } get { return _useAxisLine;       } }
        [SerializeField, HideInInspector] private bool  _useGridLine       = true;            public bool  useGridLine       { set { _useGridLine       = value; Refresh(); } get { return _useGridLine;       } }
        [SerializeField, HideInInspector] private float _gridMin           = 0f;              public float gridMin           { set { _gridMin           = value; Refresh(); } get { return _gridMin;           } }
        [SerializeField, HideInInspector] private float _gridMax           = 100f;            public float gridMax           { set { _gridMax           = value; Refresh(); } get { return _gridMax;           } }
        [SerializeField, HideInInspector] private float _gridSpan          = 10f;             public float gridSpan          { set { _gridSpan          = value; Refresh(); } get { return _gridSpan;          } }
        [SerializeField, HideInInspector] private float _axisLineOffset    = 0f;              public float axisLineOffset    { set { _axisLineOffset    = value; Refresh(); } get { return _axisLineOffset;    } }
        [SerializeField, HideInInspector] private float _gridLineOffset    = 0f;              public float gridLineOffset    { set { _gridLineOffset    = value; Refresh(); } get { return _gridLineOffset;    } }
        [SerializeField, HideInInspector] private float _axisLineThickness = 1f;              public float axisLineThickness { set { _axisLineThickness = value; Refresh(); } get { return _axisLineThickness; } }
        [SerializeField, HideInInspector] private float _gridLineThickness = 1f;              public float gridLineThickness { set { _gridLineThickness = value; Refresh(); } get { return _gridLineThickness; } }
        [SerializeField, HideInInspector] private Color _axisLineColor     = Color.white;     public Color axisLineColor     { set { _axisLineColor     = value; Refresh(); } get { return _axisLineColor;     } }
        [SerializeField, HideInInspector] private Color _gridLineColor     = Color.white;     public Color gridLineColor     { set { _gridLineColor     = value; Refresh(); } get { return _gridLineColor;     } }
        
        [SerializeField, HideInInspector] private Font  _textFont;                   public Font  textFont     { set { _textFont     = value; Refresh(); } get { return _textFont;     } }
        [SerializeField, HideInInspector] private int   _textSize;                   public int   textSize     { set { _textSize     = value; Refresh(); } get { return _textSize;     } }
        [SerializeField, HideInInspector] private Color _textColor    = Color.white; public Color textColor    { set { _textColor    = value; Refresh(); } get { return _textColor;    } }
        [SerializeField, HideInInspector] private float _textOffset   = 0f;          public float textOffset   { set { _textOffset   = value; Refresh(); } get { return _textOffset;   } }
        [SerializeField, HideInInspector] private int   _digitPlace   = 1;           public int   digitPlace   { set { _digitPlace   = value; Refresh(); } get { return _digitPlace;   } }
        [SerializeField, HideInInspector] private int   _decimalPlace = 1;           public int   decimalPlace { set { _decimalPlace = value; Refresh(); } get { return _decimalPlace; } }

        private RectTransform       _rect;
        private GameObject          _axisLineObject;
        private Image               _axisLineImage;
        private RectTransform       _axisLineRect;
        private GameObject          _gridLineObject;
        private GridLine            _gridLineComp;
        private RectTransform       _gridLineRect;
        private GameObject          _axisTextObject;
        private Text                _axisTextComp;
        private RectTransform       _axisTextRect;
        private GameObject          _gridTextObject;
        private RectTransform       _gridTextObjRect;
        private List<Text>          _gridTextComps = new List<Text>();
        private List<RectTransform> _gridTextRects = new List<RectTransform>();

        // 오브젝트 상태
        private bool _isInitialized  = false;
        private bool _isValueChanged = false;
        private bool _isEnabled = false;
        // Grid mode
        private Mode _modeHist      = Mode.Horizontal;
        private bool _isModeChanged = false;
        // Grid option
        private int   _gridCount  = 0;
        private float _gridDist   = 0f;
        // Text option
        private string _textForm = "0";

        private void OnEnable()
        {
            _isEnabled = true;
            Initialize();
            Refresh();
        }

        private void OnDisable()
        {
            _isEnabled = false;
        }
        
        private void Reset()
        {
            Initialize();
            Refresh();
        }

        private void Initialize()
        {
            _rect = GetComponent<RectTransform>();

            _axisLineObject  = _axisLineObject.InitializeObject(this.gameObject, "Axis Line");
            _axisLineImage   = _axisLineObject.InitializeComponent<Image>();
            _axisLineRect    = _axisLineObject.InitializeComponent<RectTransform>();
            _gridLineObject  = _gridLineObject.InitializeObject(this.gameObject, "Grid Line");
            _gridLineComp    = _gridLineObject.InitializeComponent<GridLine>();
            _gridLineRect    = _gridLineObject.InitializeComponent<RectTransform>();
            _axisTextObject  = _axisTextObject.InitializeObject(_axisLineObject, "Axis Text");
            _axisTextComp    = _axisTextObject.InitializeComponent<Text>();
            _axisTextRect    = _axisTextObject.InitializeComponent<RectTransform>();

            _axisLineRect.pivot     = new Vector2(0f, 0f);
            _axisLineRect.anchorMin = new Vector2(0f, 0f);
            _axisLineRect.anchorMax = new Vector2(0f, 0f);

            _gridLineRect.pivot             = new Vector2(0f, 0f);
            _gridLineRect.anchorMin         = new Vector2(0f, 0f);
            _gridLineRect.anchorMax         = new Vector2(0f, 0f);
            _gridLineComp.axisLineColor     = new Color(0f, 0f, 0f, 0f);
            _gridLineComp.direction         = GridLine.Direction.Left;
            _gridLineComp.showMajorTick     = false;
            _gridLineComp.axisLineThickness = 0f;

            Text[] gridTextComps = _gridLineObject.GetComponentsInChildren<Text>();
            if (_gridTextComps.Count != gridTextComps.Length) {
                _gridTextComps.Clear();
                _gridTextComps.AddRange(gridTextComps);

                _gridTextRects.Clear();
                for (int i = 0; i < _gridTextComps.Count; i++) {
                    _gridTextRects.Add(_gridTextComps[i].gameObject.GetComponent<RectTransform>());
                }
            }

            _isInitialized = true;
        }

        private void OnValidate()
        {
            _isValueChanged = true;
        }

        private void LateUpdate()
        {
            if (_isValueChanged && !_isEnabled)
            {
                Refresh();
                _isValueChanged = false;
            }
        }

        private void OnRectTransformDimensionsChange()
        {
            Refresh();
        }

        private void Refresh()
        {
            if (!_isEnabled) return;
            if (!_isInitialized) Initialize();

            if (_gridMax <= _gridMin)                    _gridMax           = _gridMin + 1f;
            if (_gridSpan < 0f)                          _gridSpan          = 0f;
            // 그리드 최대 생성 수 100개로 고정
            if (_gridSpan < (_gridMax - _gridMin) / 99f) _gridSpan          = (_gridMax - _gridMin) / 99f;
            if (_axisLineThickness < 0f)                 _axisLineThickness = 0f;
            if (_gridLineThickness < 0f)                 _gridLineThickness = 0f;
            if (_textSize < 0)                           _textSize          = 0;
            if (_digitPlace < 0)                         _digitPlace        = 0;
            if (_decimalPlace < 0)                       _decimalPlace      = 0;

            // Axis, grid line 설정
            if (_useAxisLine) _axisLineImage.color        = _axisLineColor;
            else              _axisLineImage.color        = new Color(0f, 0f, 0f, 0f);
            if (_useGridLine) _gridLineComp.gridLineColor = _gridLineColor;
            else              _gridLineComp.gridLineColor = new Color(0f, 0f, 0f, 0f);

            // 첫 번째 라인 제외하고 grid line 생성
            // > 첫 번째 라인은 axis line과 겹치기 때문
            _gridCount = (int)Mathf.Floor((_gridMax - _gridMin) / _gridSpan);
            _gridLineComp.gridLineAmounts = _gridCount;
            _gridLineComp.gridLineThickness = _gridLineThickness;
            
            if (_rect == null) return;

            Vector2 rectSize = _rect.GetRectSize();
            if (_mode == Mode.Horizontal) {
                _gridDist   = (rectSize.y - _axisLineThickness - _gridCount * _gridLineThickness) / _gridCount;
                _axisLineRect.anchoredPosition = new Vector2(_axisLineOffset, 0f);
                _axisLineRect.sizeDelta        = new Vector2(rectSize.x - _axisLineOffset, _axisLineThickness);
                _gridLineRect.localScale       = new Vector3(1f, 1f, 1f);
                _gridLineRect.localRotation    = Quaternion.Euler(new Vector3(0f, 0f, 0f));
                _gridLineRect.anchoredPosition = new Vector2(_gridLineOffset, _axisLineThickness + _gridDist);
                _gridLineRect.sizeDelta        = new Vector2(rectSize.x - _gridLineRect.anchoredPosition.x, rectSize.y - _gridLineRect.anchoredPosition.y);
                _gridLineComp.gridLineLength   = _gridLineRect.sizeDelta.x;
                if (_modeHist == Mode.Vertical) {
                    _isModeChanged = true;
                }
            }
            else {
                _gridDist   = (rectSize.x - _axisLineThickness - _gridCount * _gridLineThickness) / _gridCount;
                _axisLineRect.anchoredPosition = new Vector2(0f, _axisLineOffset);
                _axisLineRect.sizeDelta        = new Vector2(_axisLineThickness, rectSize.y - _axisLineOffset);
                _gridLineRect.localScale       = new Vector3(-1f, 1f, 1f);
                _gridLineRect.localRotation    = Quaternion.Euler(new Vector3(0f, 0f, -90f));
                _gridLineRect.anchoredPosition = new Vector2(_axisLineRect.sizeDelta.x + _gridDist, _gridLineOffset);
                _gridLineRect.sizeDelta        = new Vector2(rectSize.y - _gridLineRect.anchoredPosition.y, rectSize.x - _gridLineRect.anchoredPosition.x);
                _gridLineComp.gridLineLength   = _gridLineRect.sizeDelta.x;
                if (_modeHist == Mode.Horizontal) {
                    _isModeChanged = true;
                }
            }

            // Axis, grid text 설정
            UpdateTextObjectsNum();

            string textForm = "";
            for (int i = 0; i < _digitPlace; i++) textForm += "0";
            textForm += ".";
            for (int i = 0; i < _decimalPlace; i++) textForm += "0";
            _textForm = textForm;

            string   axisTextStr = _gridMin.ToString(_textForm);
            string[] gridTextStr = new string[_gridTextComps.Count];
            for (int i = 0; i < _gridTextComps.Count; i++) {
                gridTextStr[i] = (_gridMin + _gridSpan * (i + 1)).ToString(_textForm);
            }

            TextAnchor textAlignment;
            Quaternion gridTextRot;
            Vector3    gridTextScale;
            Vector2    axisTextPos;
            Vector2[]  gridTextPos = new Vector2[_gridTextComps.Count];
            // for (int i = 0; i < _gridTextComps.Count; i++) {
            //     gridTextPos[i] = new Vector2(-_textOffset - _gridLineOffset, _gridLineThickness / 2f + (_gridLineRect.sizeDelta.y - _gridLineThickness) / (_gridTextComps.Count - 1) * i);
            // }

            if (_mode == Mode.Horizontal) {
                textAlignment = TextAnchor.MiddleRight;
                gridTextRot   = Quaternion.Euler(new Vector3(0f, 0f, 0f));
                gridTextScale = new Vector3(1f, 1f, 1f);
                axisTextPos   = new Vector2(-_textOffset - _axisLineOffset, _axisLineThickness / 2f);
                for (int i = 0; i < _gridTextComps.Count; i++) {
                    gridTextPos[i] = new Vector2(-_textOffset - _gridLineOffset, _gridLineThickness / 2f + (_gridLineRect.sizeDelta.y - _gridLineThickness) / (_gridTextComps.Count - 1) * i);
                }
            }
            else {
                textAlignment = TextAnchor.MiddleCenter;
                gridTextRot   = Quaternion.Euler(new Vector3(0f, 0f, -90f));
                gridTextScale = new Vector3(-1f, 1f, 1f);
                axisTextPos   = new Vector2(_axisLineThickness / 2f, -_textOffset - _axisLineOffset - _textSize / 2f);
                for (int i = 0; i < _gridTextComps.Count; i++) {
                    gridTextPos[i] = new Vector2(-_textOffset - _gridLineOffset - _textSize / 2f, _gridLineThickness / 2f + (_gridLineRect.sizeDelta.y - _gridLineThickness) / (_gridTextComps.Count - 1) * i);
                }
            }
            
            SetText(_axisTextComp, _axisTextRect, axisTextStr, axisTextPos, _axisTextRect.localRotation, _axisTextRect.localScale, textAlignment);
            for (int i = 0; i < _gridTextComps.Count; i++) {
                SetText(_gridTextComps[i], _gridTextRects[i], gridTextStr[i], gridTextPos[i], gridTextRot, gridTextScale, textAlignment);
            }  

            _modeHist = mode;
            if (_isModeChanged) {
                _isModeChanged = false;
                Refresh();
            }
        }

        private void SetText(Text text, RectTransform rect, string textStr, Vector2 pos, Quaternion rot, Vector3 scale, TextAnchor alignment)
        {
            if (_useGridLine) text.color = _textColor;
            else              text.color = new Color(0f, 0f, 0f, 0f);

            text.font               = _textFont;
            text.fontSize           = _textSize;
            text.text               = textStr;
            text.horizontalOverflow = HorizontalWrapMode.Overflow;
            text.verticalOverflow   = VerticalWrapMode.Overflow;
            text.alignment          = alignment;
            rect.anchorMin          = new Vector2(0f, 0f);
            rect.anchorMax          = new Vector2(0f, 0f);
            rect.sizeDelta          = new Vector2(0f, 0f);
            rect.anchoredPosition   = pos;
            rect.localRotation      = rot;
            rect.localScale         = scale;
        }

        private void UpdateTextObjectsNum()
        {
            int initialCount = _gridTextComps.Count;
            if (initialCount > _gridCount) {
                for (int i = 0; i < initialCount - _gridCount; i++) {
                    DestroyImmediate(_gridTextComps[_gridTextComps.Count - 1].gameObject);
                    _gridTextRects.RemoveAt(_gridTextComps.Count - 1);
                    _gridTextComps.RemoveAt(_gridTextComps.Count - 1);
                }
            }
            else if (initialCount < _gridCount) {
                for (int i = 0; i < _gridCount - initialCount; i++) {
                    GameObject textObject = _gridLineObject.AddObjectAsChild("Grid Text");
                    _gridTextComps.Add(textObject.InitializeComponent<Text>());
                    _gridTextRects.Add(textObject.InitializeComponent<RectTransform>());
                }
            }

            if (_gridTextComps.Count != _gridCount) {
                UpdateTextObjectsNum();
            }
        }
    }
}
