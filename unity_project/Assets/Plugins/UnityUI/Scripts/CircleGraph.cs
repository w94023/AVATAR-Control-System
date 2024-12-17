using UnityEngine;
using UnityEngine.UI;
using UnityUI;

namespace UnityUI
{
    [RequireComponent(typeof(RectTransform))]
    [ExecuteInEditMode]
	public class CircleGraph : MonoBehaviour
    {
        [HideInInspector, SerializeField] private float _thickness       = 10f;                                           public float thickness       { get { return _thickness;       } set { _thickness       = value; Refresh(); } }
        [HideInInspector, SerializeField] private Color _backgroundColor = new Color(25f/255f, 26f/255f, 33f/255f, 1f);   public Color backgroundColor { get { return _backgroundColor; } set { _backgroundColor = value; Refresh(); } }
        [HideInInspector, SerializeField] private Color _fillColor       = new Color(30f/255f, 215f/255f, 171f/255f, 1f); public Color fillColor       { get { return _fillColor;       } set { _fillColor       = value; Refresh(); } }
        [HideInInspector, SerializeField] private bool  _reverse         = false;                                         public bool  reverse         { get { return _reverse;         } set { _reverse         = value; Refresh(); } }
        [Range(0f, 1f)]
        [HideInInspector, SerializeField] private float _value = 1f; public float value { get { return _value; } set { _value  = Mathf.Clamp(value, 0f, 1f); Refresh(); } }
        

        private RectTransform _rect;
        private GameObject    _backgroundObject;
        private RectTransform _backgroundRect;
        private Image         _backgroundImage;
        private GameObject    _fillObject;
        private RectTransform _fillRect;
        private Image         _fillImage;

        private Material      _objectMaterial;
        private Shader        _ringMask;
        private Sprite        _UIPanel;

        private bool _isInitialized  = false;
        private bool _isValueChanged = false;

        private void Reset()
        {
            Initialize();
            Refresh();
        }

        private void Initialize()
        {
            _rect = gameObject.GetComponent<RectTransform>();

            _backgroundObject = _backgroundObject.InitializeObject(this.gameObject, "Background");
		    _fillObject       = _fillObject.      InitializeObject(this.gameObject, "Fill");
            
            _backgroundImage = _backgroundObject.InitializeComponent<Image>();
            _backgroundRect  = _backgroundObject.InitializeComponent<RectTransform>();
            _fillImage       = _fillObject.      InitializeComponent<Image>();
            _fillRect        = _fillObject.      InitializeComponent<RectTransform>();

            _ringMask       = Shader.Find("UI/CircleGraph/RingMask");
            _UIPanel        = Resources.Load<Sprite>("UIPanel");
            _objectMaterial = new Material(_ringMask);
            _objectMaterial.SetFloat("_Hardness", 2f);

            _backgroundImage.material = _objectMaterial;
            _fillImage.sprite         = _UIPanel;
            _fillImage.material       = _objectMaterial;
            _fillImage.type           = Image.Type.Filled;
            _fillImage.fillMethod     = Image.FillMethod.Radial360;
            _fillImage.fillOrigin     = (int)Image.Origin360.Bottom;

            _isInitialized = true;
        }

        private void OnValidate()
        {
            _isValueChanged = true;
        }

        private void OnRectTransformDimensionsChange()
        {
            Refresh();
        }

        private void LateUpdate()
        {
            if (_isValueChanged) {
                Refresh();
                _isValueChanged = false;
            }
        }

        private void Refresh()
        {
            if (!_isInitialized) Initialize();

            Vector2 rectSize = _rect.GetRectSize();

            float newThickness = _thickness/((rectSize.x + rectSize.y) / 2f) * 0.35f;
            _objectMaterial.SetFloat("_Thickness", newThickness);

            _backgroundImage.color   = _backgroundColor;
            _fillImage.color         = _fillColor;
            _fillImage.fillAmount    = value;
            _fillImage.fillClockwise = !_reverse;

            _backgroundRect.sizeDelta = rectSize;
            _fillRect.      sizeDelta = rectSize;
        }
    }
}
