using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;
using UnityEngine.EventSystems;
using UnityUI;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace UnityUI
{
    [ExecuteInEditMode]
    [RequireComponent(typeof(RectTransform))]
    public class ToggleButton : MonoBehaviour, IPointerDownHandler
    {   
                                          public  int  stepAmount   { set { _stepAmount = value; Refresh(false); } get { return _stepAmount; } }
        [SerializeField, HideInInspector] private int _stepAmount   = 2; 
                                          public  int  value        { set { _value = value; Refresh(false); } get { return _value; } }
        [SerializeField, HideInInspector] private int _value        = 0;   
        [SerializeField, HideInInspector] private int _currentValue = 0;

                                          public  float  handleEdgeThickness { set { _handleEdgeThickness = value; Refresh(false); } get { return _handleEdgeThickness; } }
        [SerializeField, HideInInspector] private float _handleEdgeThickness = 0.2f; 

                                          public  Color    backgroundColor  { set { _backgroundColor = value; Refresh(false); } get { return _backgroundColor; } }
        [SerializeField, HideInInspector] private Color   _backgroundColor  = new Color(8f/255f, 9f/255f, 11f/255f, 1f);
                                          public  Color    sliderColor      { set { _sliderColor = value; Refresh(false); } get { return _sliderColor; } }
        [SerializeField, HideInInspector] private Color   _sliderColor      = new Color(35f/255f, 38f/255f, 51f/255f, 1f);  
                                          public  Color    handleBaseColor  { set { _handleBaseColor = value; Refresh(false); } get { return _handleBaseColor; } }
        [SerializeField, HideInInspector] private Color   _handleBaseColor  = new Color(8f/255f, 9f/255f, 11f/255f, 1f);
                                          public  Color[]  handleEdgeColors { set { _handleEdgeColors = value; Refresh(false); } get { return _handleEdgeColors; } }
        [SerializeField, HideInInspector] private Color[] _handleEdgeColors = new Color[2] { new Color(30f/255f, 215f/255f, 171f/255f, 1f), new Color(2f/255f, 167f/255f, 248f/255f, 1f) };

        [SerializeField, HideInInspector] private string _labelText = "";
        [SerializeField, HideInInspector] private Color  _labelColor = new Color(110f/255f, 111f/255f, 120f/255f, 1f);
        [SerializeField, HideInInspector] private int    _labelFontSize = 15;
        [SerializeField, HideInInspector] private float  _labelSpacing = 10;

        [SerializeField, HideInInspector] private float _toggleAnimationTime = 0.25f;

        [HideInInspector] public UnityEvent onClick = new UnityEvent();

        private GameObject _label;
        private GameObject _background;
        private GameObject _slider;
        private GameObject _handle;

        private RectTransform _rect;
        private RectTransform _labelRect;
        private RectTransform _backgroundRect;
        private RectTransform _sliderRect;
        private RectTransform _handleRect;

        private Text _labelTextComp;
        private Image _backgroundImage;
        private Image _sliderImage;
        private Image _handleImage;

        private ImageWithRoundedCorners _backgroundImageCorners;
        private ImageWithRoundedCorners _sliderImageCorners;
        private ImageWithRoundedCorners _handleImageCorners;

        private bool _isInitialized = false;
        private bool _isAnimationPlaying = false;

        private void OnEnable()
		{
			Initialize();
            Refresh(false);
            UpdateGraphics();
		}

		private void Reset()
		{
			Initialize();
            Refresh(false);
            UpdateGraphics();
		}

        private void Initialize()
		{
            _label      = _label.     InitializeObject(this.gameObject, "Label");
            _background = _background.InitializeObject(this.gameObject, "Background");
            _slider     = _slider.    InitializeObject(this.gameObject, "Slider");
            _handle     = _handle.    InitializeObject(this.gameObject, "Handle");

            _rect           =             GetComponent<RectTransform>();
            _labelRect      = _label.     GetComponent<RectTransform>();
            _backgroundRect = _background.GetComponent<RectTransform>();
            _sliderRect     = _slider.    GetComponent<RectTransform>();
            _handleRect     = _handle.    GetComponent<RectTransform>();

            _labelTextComp = _label.InitializeComponent<Text>();

            _backgroundImage = _background.InitializeComponent<Image>();
            _sliderImage     = _slider.    InitializeComponent<Image>();
            _handleImage     = _handle.    InitializeComponent<Image>();

            _backgroundImageCorners = _background.InitializeComponent<ImageWithRoundedCorners>();
            _sliderImageCorners     = _slider.    InitializeComponent<ImageWithRoundedCorners>();
            _handleImageCorners     = _handle.    InitializeComponent<ImageWithRoundedCorners>();

            _labelTextComp.alignment          = TextAnchor.MiddleRight;
            _labelTextComp.horizontalOverflow = HorizontalWrapMode.Overflow;
            _labelTextComp.verticalOverflow   = VerticalWrapMode.Overflow;
            _labelRect.sizeDelta              = new Vector2(0f, 1f);
            _labelRect.anchorMin              = new Vector2(0f, 0.5f);
            _labelRect.anchorMax              = new Vector2(0f, 0.5f);
            _labelRect.pivot                  = new Vector2(1f, 0.5f);

            _backgroundImageCorners.edgeSoftness       = 0.1f;
            _sliderImageCorners.edgeSoftness           = 0.1f;
            _handleImageCorners.edgeSoftness           = 0.2f;
            _handleImageCorners.edgeTransitionSoftness = 0.2f;

            _isInitialized = true;
		}

        private void OnValidate()
        {
            Refresh(true);
        }

        private void Refresh(bool invokeEvent)
        {
            if (!_isInitialized) Initialize();

            _value = Mathf.Clamp(_value, 0, _stepAmount - 1);

            _labelTextComp.text = _labelText;
            _labelTextComp.color = _labelColor;
            _labelTextComp.fontSize = _labelFontSize;
            _labelRect.anchoredPosition =  new Vector3(-_labelSpacing, 0f, 0f);

            if (_handleEdgeColors.Length > _stepAmount) {
                List<Color> newColors = new List<Color>();
                for (int i = 0; i < _stepAmount; i++) {
                    newColors.Add(_handleEdgeColors[i]);
                }
                _handleEdgeColors = newColors.ToArray();
            }
            else if (_handleEdgeColors.Length < _stepAmount) {
                List<Color> newColors = new List<Color>();
                for (int i = 0; i < _handleEdgeColors.Length; i++) {
                    newColors.Add(_handleEdgeColors[i]);
                }
                for (int i = 0; i < _stepAmount - _handleEdgeColors.Length; i++) {
                    newColors.Add(Color.white);
                }
                _handleEdgeColors = newColors.ToArray();
            }

            _backgroundImage.color        = _backgroundColor;
            _sliderImage.color            = _sliderColor;
            _handleImage.color            = _handleBaseColor;
            _handleImageCorners.edgeColor = _handleEdgeColors[_value];

            _handleEdgeThickness = Mathf.Max(0f, _handleEdgeThickness);
            _handleEdgeThickness = Mathf.Min(1f, _handleEdgeThickness);
            _handleImageCorners.edgeThickness = _handleEdgeThickness;

            // 유니티 에디터에서 플레이 중이지 않은 경우를 제외하면 코루틴을 통한 애니메이션 재생
            // 유니티 에디터에서 플레이 중이지 않은 상태에서 value가 변경된 경우 코루틴 사용 X
            // (플레이 중이지 않은 상태에서는 코루틴 사용 불가)
        #if UNITY_EDITOR
            if (EditorApplication.isPlaying)
            {
                PlayClickAnimation(_value);
            }
            else {
                float handleStartPos = -_sliderRect.GetRectSize().x/2f;
                float handleEndPos   = -handleStartPos;
                Vector3 targetPos    = new Vector3((handleEndPos - handleStartPos) * _value/(_stepAmount - 1) + handleStartPos, 0f, 0f);
                _handleRect.anchoredPosition = targetPos;
            }
        #else
                PlayClickAnimation(_value);
        #endif

            
            if (invokeEvent) {
                // value가 변경되었으면 이벤트 활성화
                if (_value != _currentValue) {
                    onClick?.Invoke();
                }
            }

            _currentValue = _value;
        }

        private void OnRectTransformDimensionsChange()
        { 
            UpdateGraphics();
        }

        private void UpdateGraphics()
        {
            if (!_isInitialized) Initialize();

            Vector2 rectSize = _rect.GetRectSize();
            _backgroundRect.sizeDelta = new Vector2(rectSize.x,              rectSize.y);
            _sliderRect.sizeDelta     = new Vector2(rectSize.x - rectSize.y, rectSize.y / 6f);
            _handleRect.sizeDelta     = new Vector2(rectSize.y - 10f,        rectSize.y - 10f);

            _backgroundImageCorners.radius = _backgroundRect.GetRectSize().x;
            _sliderImageCorners.radius = _sliderRect.GetRectSize().x;
            _handleImageCorners.radius = _handleRect.GetRectSize().x;

            float handleStartPos = -_sliderRect.GetRectSize().x/2f;
            float handleEndPos   = -handleStartPos;
            Vector3 targetPos    = new Vector3((handleEndPos - handleStartPos) * _value/(_stepAmount - 1) + handleStartPos, 0f, 0f);
            _handleRect.anchoredPosition = targetPos;
        }

        public void Toggle()
        {
            OnClicked(false);
        }

        public void OnPointerDown(PointerEventData pointerEventData)
        {
            OnClicked(true);
        }

        private void OnClicked(bool invokeEvent)
        {
            _value++;
            if(_value >= _stepAmount) _value = 0; 

            _value = Mathf.Clamp(_value, 0, _stepAmount - 1);

            Refresh(invokeEvent);
        }

        public void PlayClickAnimation(int targetValue)
        {
            if (_isAnimationPlaying) return;
            StartCoroutine(ChangeHandlePosition(targetValue, _toggleAnimationTime));
        }

        private IEnumerator ChangeHandlePosition(int targetValue, float targetPlaySeconds)
        {
            float handleStartPos = -_sliderRect.GetRectSize().x/2f;
            float handleEndPos   = -handleStartPos;
            Vector3 startPos     = _handleRect.anchoredPosition;
            Vector3 targetPos    = new Vector3((handleEndPos - handleStartPos) * targetValue/(_stepAmount - 1) + handleStartPos, 0f, 0f);

            float chkTime = Time.realtimeSinceStartup;
            while (true)
            {
                float timeSpent = Time.realtimeSinceStartup - chkTime;
                float normalizedProgress = timeSpent/targetPlaySeconds;
                if (normalizedProgress >= 1f) {
                    normalizedProgress = 1f;
                }

                _isAnimationPlaying = true;
                Vector3 currentPos = Vector3.Lerp(startPos, targetPos, normalizedProgress);
                _handleRect.anchoredPosition = currentPos;
                if (normalizedProgress >= 1f) {
                    _isAnimationPlaying = false;
                    _handleImageCorners.edgeColor = _handleEdgeColors[_value];
                    if (_value != targetValue) {
                        StartCoroutine(ChangeHandlePosition(_value, targetPlaySeconds));
                    }
                    break;
                }
                yield return new WaitForSeconds(0.005f);
            }
            _handleImageCorners.edgeColor = _handleEdgeColors[_value];
            _isAnimationPlaying = false;
        }
    }
}
