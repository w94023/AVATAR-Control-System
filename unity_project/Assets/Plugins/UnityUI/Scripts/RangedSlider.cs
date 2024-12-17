using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;
#if UNITY_EDITOR
using UnityEditor.Events;
#endif

namespace UnityUI
{
    [RequireComponent(typeof(Slider))]
    [ExecuteInEditMode]
    public class RangedSlider : MonoBehaviour
    {
        public UnityEvent<Vector2> onRangeChanged;

        public Slider slider;
        public GameObject backgroundObject;
        public GameObject fillObject;
        public GameObject rangedBackgroundObject;
        public GameObject rangedFillObject;
        public GameObject handleObject;

        [SerializeField, HideInInspector] private RectTransform _sliderRect;
        [SerializeField, HideInInspector] private RectTransform _backgroundRect;
        [SerializeField, HideInInspector] private Image         _backgroundImage;
        [SerializeField, HideInInspector] private Image         _fillImage;
        [SerializeField, HideInInspector] private RectTransform _rangedBackgroundRect;
        [SerializeField, HideInInspector] private Image         _rangedBackgroundImage;
        [SerializeField, HideInInspector] private RectTransform _rangedFillRect;
        [SerializeField, HideInInspector] private Image         _rangedFillImage;
        [SerializeField, HideInInspector] private RectTransform _handleRect;

        [SerializeField, HideInInspector] private bool _isEventRegistered = false;

        [Space(20)]
        [SerializeField] private bool    _useRange     = false;               public bool    useRange { set { _useRange = value; Update(); } get { return _useRange; } } 
        [SerializeField] private Vector2 _range        = new Vector2(0f, 0f); public Vector2 range    { set { _range    = value; Update(); } get { return _range;    } }
        
        [SerializeField] private Color _backgroundColor      = new Color(230f/255f, 230f/255f, 230f/255f, 1f); public Color backgroundColor      { set { _backgroundColor      = value; OnValidate(); } get { return _backgroundColor;      }}
        [SerializeField] private Color _fillColor            = new Color(1f,        1f,        1f,        1f); public Color fillColor            { set { _fillColor            = value; OnValidate(); } get { return _fillColor;            }}
        [SerializeField] private Color _rangeBackgroundColor = new Color(230f/255f, 0f,        0f,        1f); public Color rangeBackgroundColor { set { _rangeBackgroundColor = value; OnValidate(); } get { return _rangeBackgroundColor; }}
        [SerializeField] private Color _rangeFillColor       = new Color(1f,        0f,        0f,        1f); public Color rangeFillColor       { set { _rangeFillColor       = value; OnValidate(); } get { return _rangeFillColor;       }}

        [SerializeField, HideInInspector] private bool    _useRangeHist   = false;
        [SerializeField, HideInInspector] private Vector2 _rangeHist      = new Vector2(0f, 0f);
        [SerializeField, HideInInspector] private Vector2 _sliderSize     = new Vector2(0f, 0f);
        [SerializeField, HideInInspector] private float   _handleSize     = 0f;
        [SerializeField, HideInInspector] private float   _sliderMinValue = 0f;
        [SerializeField, HideInInspector] private float   _sliderMaxValue = 0f;

        private void InitializeImage(RectTransform targetRect, Image targetImage)
        {
            targetImage.sprite       = _backgroundImage.sprite;
            targetImage.type         = _backgroundImage.type;
            targetRect.pivot         = _backgroundRect.pivot;
            targetRect.anchorMin     = _backgroundRect.anchorMin;
            targetRect.anchorMax     = _backgroundRect.anchorMax;
            targetRect.sizeDelta     = _backgroundRect.sizeDelta;
            targetRect.localPosition = _backgroundRect.localPosition;
        }

        private void OnEnable()
        {
            slider      = this.gameObject.GetComponent<Slider>();
            _sliderRect = this.gameObject.GetComponent<RectTransform>();

            if (backgroundObject == null) {
                backgroundObject = ComponentHelper.FindObject(this.gameObject, "Background");
            }
            _backgroundRect   = backgroundObject.GetComponent<RectTransform>();
            _backgroundImage  = backgroundObject.GetComponent<Image>();
            
            // if (fillObject == null) {
            //     GameObject fillAreaObject = ComponentHelper.FindObject(this.gameObject, "Fill Area");
            //     fillObject = ComponentHelper.FindObject(fillAreaObject, "Fill");
            // }
            if (fillObject != null) _fillImage = fillObject.GetComponent<Image>();

            if (rangedBackgroundObject == null) {
                rangedBackgroundObject = ComponentHelper.CreateEmptyObject(this.gameObject, "Ranged Background");
                rangedBackgroundObject.transform.SetSiblingIndex(1);
                rangedBackgroundObject.AddComponent<Image>();
            }
            _rangedBackgroundRect   = rangedBackgroundObject.GetComponent<RectTransform>();
            _rangedBackgroundImage  = rangedBackgroundObject.GetComponent<Image>();
            InitializeImage(_rangedBackgroundRect, _rangedBackgroundImage);
            
            if (rangedFillObject == null) {
                rangedFillObject = ComponentHelper.CreateEmptyObject(this.gameObject, "Ranged Fill");
                rangedFillObject.transform.SetSiblingIndex(3);
                rangedFillObject.AddComponent<Image>();
            }
            _rangedFillRect = rangedFillObject.GetComponent<RectTransform>();
            _rangedFillImage = rangedFillObject.GetComponent<Image>();
            InitializeImage(_rangedFillRect, _rangedFillImage);
            
            if (handleObject == null) {
                List<GameObject> sliderObjects = this.gameObject.GetAllChildren();
                handleObject = sliderObjects.Find("Handle");
            }
            _handleRect = handleObject.GetComponent<RectTransform>();
            
            _sliderSize = _sliderRect.GetRectSize();
            _handleSize = _handleRect.sizeDelta.x;
            _sliderMinValue = slider.minValue;
            _sliderMaxValue = slider.maxValue;

            SetRange();
            #if UNITY_EDITOR
            if (!_isEventRegistered) {
                UnityEventTools.AddPersistentListener(slider.onValueChanged, OnValueChanged);
                _isEventRegistered = true;
            }
            #endif
        }

        public void Clear()
        {
            useRange = false;
            range    = new Vector2(0f, 0f);
            slider.minValue = 0f;
            slider.maxValue = 1f;
            slider.value = 0f;
        }

        private void OnValidate()
        {
            if (_backgroundImage       != null) _backgroundImage.color = backgroundColor;
            if (_fillImage             != null) _fillImage.color = fillColor;
            if (slider                 != null) SetHandleColor(fillColor);
            if (_rangedBackgroundImage != null) _rangedBackgroundImage.color = rangeBackgroundColor;
            if (_rangedFillImage       != null) _rangedFillImage.color = rangeFillColor;
        }

        private void OnRectTransformDimensionsChange()
        {
            _sliderSize = _sliderRect.GetRectSize();
            SetRange();
        }

        private void Update()
        {
            if (
                _range == _rangeHist && 
                _useRange == _useRangeHist &&
                _handleSize == _handleRect.sizeDelta.x &&
                _sliderMinValue == slider.minValue &&
                _sliderMaxValue == slider.maxValue
            ) 
            return;

            SetRange();
        }

        private void SetHandleColor(Color color)
        {
            ColorBlock colors       = slider.colors;
            colors.normalColor      = color;
            colors.highlightedColor = color;
            colors.pressedColor     = color;
            colors.selectedColor    = color;
            slider.colors           = colors;
        }

        public void SetRange()
        {
            _useRangeHist   = _useRange;
            _rangeHist      = _range;
            _handleSize     = _handleRect.sizeDelta.x;
            _sliderMinValue = slider.minValue;
            _sliderMaxValue = slider.maxValue;
            
            if (slider == null) return;

            if (_range[0] > _range[1]) {
                float maxRangeTemp = _range[1];
                _range[1] = _range[0];
                _range[0] = maxRangeTemp;
            }
            if (_range[0] < _sliderMinValue) _range[0] = _sliderMinValue;
            if (_range[1] > _sliderMaxValue) _range[1] = _sliderMaxValue;

            if (_useRange) {
                Vector2 normalizedRange = new Vector2(0f, 0f);
                normalizedRange[0] = (_range[0] - _sliderMinValue) / (_sliderMaxValue - _sliderMinValue);
                normalizedRange[1] = (_range[1] - _sliderMinValue) / (_sliderMaxValue - _sliderMinValue);
                if (_rangedBackgroundRect != null) _rangedBackgroundRect.offsetMin = new Vector2(normalizedRange[0] * _sliderSize.x, _rangedBackgroundRect.offsetMin.y);
                if (_rangedBackgroundRect != null) _rangedBackgroundRect.offsetMax = new Vector2(-((1f - normalizedRange[1]) * (_sliderSize.x - _handleSize) + _handleSize / 2f), _rangedBackgroundRect.offsetMax.y);
                if (_rangedFillRect       != null) _rangedFillRect.offsetMin       = new Vector2(normalizedRange[0] * _sliderSize.x, _rangedFillRect.offsetMin.y);
            }
            else {
                _rangedBackgroundRect.offsetMin = new Vector2(+_sliderSize.x / 2f, _rangedBackgroundRect.offsetMin.y);
                _rangedBackgroundRect.offsetMax = new Vector2(-_sliderSize.x / 2f, _rangedBackgroundRect.offsetMax.y);
                _rangedFillRect.offsetMin       = new Vector2(+_sliderSize.x / 2f, _rangedFillRect.offsetMin.y);
            }
            
            onRangeChanged?.Invoke(_range);
            OnValueChanged(slider.value);
        }

        private void OnValueChanged(float value)
        {
            if (slider == null) return;

            if (!_useRange) {
                SetHandleColor(fillColor);
                _rangedFillRect.offsetMax = new Vector2(-_sliderSize.x / 2f, _rangedFillRect.offsetMax.y);
                return;
            }

            if (value >= _range[0] && value <= _range[1]) {
                SetHandleColor(rangeFillColor);

                float normalizedValue = (value - _sliderMinValue) / (_sliderMaxValue - _sliderMinValue);
                if (_rangedFillRect != null) _rangedFillRect.offsetMax = new Vector2(-((1f - normalizedValue) * (_sliderSize.x - _handleSize) + _handleSize / 2f), _rangedFillRect.offsetMax.y);
            }
            else {
                SetHandleColor(fillColor);

                if (value < _range[0]) value = _range[0];
                else                   value = _range[1];
                float normalizedValue = (value - _sliderMinValue) / (_sliderMaxValue - _sliderMinValue);
                if (_rangedFillRect != null) _rangedFillRect.offsetMax = new Vector2(-((1f - normalizedValue) * (_sliderSize.x - _handleSize) + _handleSize / 2f), _rangedFillRect.offsetMax.y);
            }
        }
    }
}