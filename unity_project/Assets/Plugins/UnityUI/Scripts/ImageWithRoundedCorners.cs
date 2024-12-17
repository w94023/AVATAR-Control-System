using UnityEngine;
using UnityEngine.UI;
using UnityUI;

namespace UnityUI
{
	[ExecuteInEditMode]
	[RequireComponent(typeof(RectTransform))]
	[RequireComponent(typeof(Image))]
	public class ImageWithRoundedCorners : MonoBehaviour
	{
										               public  float  radius { get { return _radius; } set { _radius = value; Refresh(); } }
		[SerializeField, HideInInspector             ] private float _radius = 0f;
										               public  float  edgeSoftness { get { return _edgeSoftness; } set { _edgeSoftness = value; Refresh(); } }
		[SerializeField, HideInInspector             ] private float _edgeSoftness = 0f;
						 				               public  float  edgeTransitionSoftness { get { return _edgeTransitionSoftness; } set { _edgeTransitionSoftness = value; Refresh(); } }
		[SerializeField, HideInInspector             ] private float _edgeTransitionSoftness = 0f;
										  		       public  float  edgeThickness { get { return _edgeThickness; } set { _edgeThickness = value; Refresh(); } }
		[SerializeField, HideInInspector, Range(0, 1)] private float _edgeThickness = 0f;
						 			                   public  Color  edgeColor { get { return _edgeColor; } set { _edgeColor = value; Refresh(); } }		      
		[SerializeField, HideInInspector             ] private Color _edgeColor = Color.white;

		private RectTransform _rect;
		private Image         _image;
        private Material      _material;

		private bool _isInitialized = false;

		private void OnEnable()
		{
			Initialize();
			Refresh();
		}

		private void Reset()
		{
			Initialize();
			Refresh();
		}

		private void Initialize()
		{
			_rect  = GetComponent<RectTransform>();
			_image = GetComponent<Image>();

            _material = new Material(Shader.Find("UI/UnityUI/RoundedCornersWithEdge"));
			_image.material = _material;

			_isInitialized = true;
		}
		
		private void OnValidate()
		{
			Refresh();
		}

		private void OnRectTransformDimensionsChange()
        {
			Refresh();
        }

		private void Refresh()
		{
			if (!_isInitialized) Initialize();

			_edgeThickness = Mathf.Min(_edgeThickness, 1f);
			_edgeThickness = Mathf.Max(_edgeThickness, 0f);

			Vector2 rectSize = _rect.GetRectSize();

			_material.SetFloat("_Aspect", rectSize.x/rectSize.y);
			// EdgeSoftness보다 커야 투명해지지 않음
			_material.SetFloat("_Radius", Mathf.Max(_radius/Mathf.Max(rectSize.x, rectSize.y), _edgeSoftness));
			_material.SetFloat("_EdgeSoftness", _edgeSoftness);
			_material.SetFloat("_EdgeThickness", _edgeThickness);
			_material.SetColor("_EdgeColor", _edgeColor);
			_material.SetColor("_MainColor", _image.color);
			if (_edgeThickness > 0) _material.SetFloat("_EdgeTransitionSoftness", _edgeTransitionSoftness);
			else				    _material.SetFloat("_EdgeTransitionSoftness", 0f);
		}
	}	
}