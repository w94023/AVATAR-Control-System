using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace UnityUI
{
    [ExecuteInEditMode]
    [RequireComponent(typeof(RectTransform))]
    public class ScrollViewCreater : MonoBehaviour
    {
        public  Color  backgroundColor          { get { return _backgroundColor; }          set { _backgroundColor = value;          Refresh(); } }
        public  Color  textColor                { get { return _textColor; }                set { _textColor = value;                Refresh(); } }
        public  Color  scrollbarBackgroundColor { get { return _scrollbarBackgroundColor; } set { _scrollbarBackgroundColor = value; Refresh(); } }
        public  Color  scrollbarHandleColor     { get { return _scrollbarHandleColor; }     set { _scrollbarHandleColor = value;     Refresh(); } }
        [SerializeField] private Color _backgroundColor          = new Color(110f/255f, 111f/255f, 120f/255f, 29f/255f);
        [SerializeField] private Color _textColor                = new Color(       1f,        1f,        1f,       1f);
        [SerializeField] private Color _scrollbarBackgroundColor = new Color( 25f/255f,  26f/255f,  33f/255f,       1f);
        [SerializeField] private Color _scrollbarHandleColor     = new Color(       1f,        1f,        1f,       1f);

        public int scrollSensitivity { get { return _scrollSensitivity; } set { _scrollSensitivity = value; Refresh(); } }
        public int fontSize          { get { return _fontSize; }          set { _fontSize          = value; Refresh(); } }
        [Space(20)]
        [SerializeField] private int _scrollSensitivity = 30;
        [SerializeField] private int _fontSize          = 20;

        [SerializeField, HideInInspector] private Sprite _backgroundSprite;
        [SerializeField, HideInInspector] private Sprite _uiMaskSprite;
        [SerializeField, HideInInspector] private Sprite _uiSprite;
        [SerializeField, HideInInspector] private Font   _arialFont;

                                          public  ScrollRect   scrollRect;
                                          public  Text         text;
        [SerializeField, HideInInspector] private Image       _scrollViewImage;
        [SerializeField, HideInInspector] private List<Image> _scrollbarImage       = new List<Image>();
        [SerializeField, HideInInspector] private List<Image> _scrollbarHandleImage = new List<Image>();

        private enum ScrollbarType
        {
            Vertical,
            Horizontal
        }

        private void OnEnable()
        {
            #if UNITY_EDITOR
            // Scroll view 생성에 필요한 sprite 불러오기
            _backgroundSprite = AssetDatabase.GetBuiltinExtraResource<Sprite>("UI/Skin/Background.psd");
            _uiMaskSprite     = AssetDatabase.GetBuiltinExtraResource<Sprite>("UI/Skin/UIMask.psd");
            _uiSprite         = AssetDatabase.GetBuiltinExtraResource<Sprite>("UI/Skin/UISprite.psd");
            _arialFont        = Resources.GetBuiltinResource<Font>("Arial.ttf");
            #endif
            if (_backgroundSprite == null || _uiMaskSprite == null || _uiSprite == null || _arialFont == null) {
                Debug.LogError("Failed to find srpite/font to craete scroll view");
            }

            // Scrill view 생성
            if (scrollRect == null) {
                scrollRect = CreateScrollView();
            }
        }

        #if UNITY_EDITOR
        private void Reset()
        {
            OnEnable();
        }

        private void OnValidate()
        {
            Refresh();   
        }
        #endif

        private void Refresh()
        {
            scrollRect.scrollSensitivity = _scrollSensitivity;
            _scrollViewImage.color       = _backgroundColor;
            text.color                   = _textColor;
            text.fontSize                = _fontSize;
            for (int i = 0; i < _scrollbarImage.Count; i++) {
                _scrollbarImage[i].color = _scrollbarBackgroundColor;
            }
            for (int i = 0; i < _scrollbarHandleImage.Count; i++) {
                _scrollbarHandleImage[i].color = _scrollbarHandleColor;
            }
        }

        private Scrollbar CreateScrollbar(GameObject root, ScrollbarType type)
        {
            string scrollbarName = "";
            if (type == ScrollbarType.Vertical) {
                scrollbarName = "Scrollbar Vertical";
            }
            else if (type == ScrollbarType.Horizontal) {
                scrollbarName = "Scrollbar Horizontal";
            }
            else {
                goto Fail;
            }

            // 1. Scrollbar 객체 생성
            GameObject scrollbarObject = ComponentHelper.CreateEmptyObject(root, scrollbarName);
            var scrollbarRect  = scrollbarObject.InitializeComponent<RectTransform>();
            var scrollbarImage = scrollbarObject.InitializeComponent<Image>();
            var scrollbar      = scrollbarObject.InitializeComponent<Scrollbar>();
            scrollbarImage.sprite = _backgroundSprite;
            scrollbarImage.type   = Image.Type.Sliced;
            scrollbarImage.color  = _scrollbarBackgroundColor;
            scrollbar.size        = 1f;
            if (type == ScrollbarType.Vertical) {
                scrollbarRect.anchorMin = new Vector2(1f, 0f);
                scrollbarRect.anchorMax = new Vector2(1f, 1f);
                scrollbarRect.pivot     = new Vector2(1f, 1f);
                scrollbarRect.sizeDelta = new Vector2(20f, scrollbarRect.sizeDelta.y);
                scrollbar.direction = Scrollbar.Direction.BottomToTop;
            }
            else if (type == ScrollbarType.Horizontal) {
                scrollbarRect.anchorMin = new Vector2(0f, 0f);
                scrollbarRect.anchorMax = new Vector2(1f, 0f);
                scrollbarRect.pivot     = new Vector2(0f, 0f);
                scrollbarRect.sizeDelta = new Vector2(scrollbarRect.sizeDelta.x, 20f);
                scrollbar.direction = Scrollbar.Direction.LeftToRight;
            }
            else {
                goto Fail;
            }
            _scrollbarImage.Add(scrollbarImage);

            // 2. Scrollbar의 Sliding Area 추가
            GameObject slidingArea = ComponentHelper.CreateEmptyObject(scrollbarObject, "Sliding Area");
            var slidingAreaRect = slidingArea.InitializeComponent<RectTransform>();
            slidingAreaRect.anchorMin        = Vector2.zero;
            slidingAreaRect.anchorMax        = Vector2.one;
            slidingAreaRect.pivot            = new Vector2(0.5f, 0.5f);
            slidingAreaRect.anchoredPosition = Vector2.zero;
            slidingAreaRect.sizeDelta        = Vector2.zero;
            slidingAreaRect.offsetMin        = new Vector2( 10f,  10f);
            slidingAreaRect.offsetMax        = new Vector2(-10f, -10f);

            // 3. Scrollbar의 Handle 추가
            GameObject handleObject = ComponentHelper.CreateEmptyObject(slidingArea, "Handle");
            var handleRect  = handleObject.InitializeComponent<RectTransform>();
            var handleImage = handleObject.InitializeComponent<Image>();
            handleRect.sizeDelta        = new Vector2(20, 100); // Handle 크기 설정
            handleRect.anchorMin        = Vector2.zero;
            handleRect.anchorMax        = Vector2.one;
            handleRect.pivot            = new Vector2(0.5f, 0.5f);
            handleRect.anchoredPosition = Vector2.zero;
            handleRect.offsetMin        = new Vector2(-10f, -10f);
            handleRect.offsetMax        = new Vector2( 10f,  10f);
            handleImage.sprite          = _uiSprite;
            handleImage.type            = Image.Type.Sliced;
            scrollbar.targetGraphic     = handleImage;
            scrollbar.handleRect        = handleRect;
            _scrollbarHandleImage.Add(handleImage);

            return scrollbar;

            Fail:
            Debug.LogError("Failed to create scroll bar in scroll view");
            return null;
        }

        public ScrollRect CreateScrollView()
        {
            // 1. ScrollView component 생성
            var scrollViewRect  = this.gameObject.InitializeComponent<RectTransform>();
               _scrollViewImage = this.gameObject.InitializeComponent<Image>();
            var scrollRect      = this.gameObject.InitializeComponent<ScrollRect>();
            scrollViewRect.sizeDelta        = new Vector2(300, 400);
            scrollViewRect.anchorMin        = new Vector2(0.5f, 0.5f);
            scrollViewRect.anchorMax        = new Vector2(0.5f, 0.5f);
            scrollViewRect.pivot            = new Vector2(0.5f, 0.5f);
            scrollViewRect.anchoredPosition = Vector2.zero;
            _scrollViewImage.color  = _backgroundColor;
            _scrollViewImage.sprite = _backgroundSprite;
            _scrollViewImage.type   = Image.Type.Sliced;
            scrollRect.verticalScrollbarVisibility   = ScrollRect.ScrollbarVisibility.AutoHideAndExpandViewport;
            scrollRect.horizontalScrollbarVisibility = ScrollRect.ScrollbarVisibility.AutoHideAndExpandViewport;
            scrollRect.scrollSensitivity             = _scrollSensitivity;

            // 2. Viewport 객체 생성
            GameObject viewportObject = ComponentHelper.CreateEmptyObject(this.gameObject, "Viewport");
            var viewportRect  = viewportObject.InitializeComponent<RectTransform>();
            var viewportImage = viewportObject.InitializeComponent<Image>();
            var viewportMask  = viewportObject.InitializeComponent<Mask>();
            viewportRect.pivot = new Vector2(0f, 1f);
            viewportImage.sprite = _uiMaskSprite;
            viewportImage.type   = Image.Type.Sliced;
            viewportMask.showMaskGraphic = false;
            
            // 3. Content 객체 생성
            GameObject contentObject = ComponentHelper.CreateEmptyObject(viewportObject, "Content");
            var contentRect        = contentObject.InitializeComponent<RectTransform>();
            var contentSizeFitter  = contentObject.InitializeComponent<ContentSizeFitter>();
            var contentLayoutGroup = contentObject.InitializeComponent<VerticalLayoutGroup>();
            contentRect.anchorMin = new Vector2(0f, 1f);
            contentRect.anchorMax = new Vector2(1f, 1f);
            contentRect.pivot     = new Vector2(0f, 1f);
            contentSizeFitter.horizontalFit = ContentSizeFitter.FitMode.PreferredSize;
            contentSizeFitter.verticalFit   = ContentSizeFitter.FitMode.PreferredSize;
            contentLayoutGroup.childScaleWidth  = true;
            contentLayoutGroup.childScaleHeight = true;

            // 4. Text 객체 생성
            GameObject textObject = ComponentHelper.CreateEmptyObject(contentObject, "Text");
            var textRect        = textObject.InitializeComponent<RectTransform>();
                text            = textObject.InitializeComponent<Text>();
            var textSizeFiltter = textObject.InitializeComponent<ContentSizeFitter>();
            textRect.pivot                = new Vector2(0f, 1f);
            textSizeFiltter.horizontalFit = ContentSizeFitter.FitMode.PreferredSize;
            textSizeFiltter.verticalFit   = ContentSizeFitter.FitMode.PreferredSize;
            text.color    = _textColor;
            text.font     = _arialFont;
            text.fontSize = _fontSize;

            _scrollbarImage.Clear();
            _scrollbarHandleImage.Clear();
            var scrollbarVertical   = CreateScrollbar(this.gameObject, ScrollbarType.Vertical);
            var scrollbarHorizontal = CreateScrollbar(this.gameObject, ScrollbarType.Horizontal);

            // ScrollRect 할당
            scrollRect.content             = contentRect;
            scrollRect.viewport            = viewportRect;
            scrollRect.verticalScrollbar   = scrollbarVertical;
            scrollRect.horizontalScrollbar = scrollbarHorizontal;

            return scrollRect;
        }
    }
}