using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace UnityUI
{
    [ExecuteInEditMode]
    [RequireComponent(typeof(RectTransform))]
    public class GridLine : CustomMeshManager
    {
        public enum Direction
        { 
            Center = 1, 
            Left   = 0, 
            Right  = 2
        }

        [SerializeField, HideInInspector] private Direction _direction                     = Direction.Center; public Direction direction                     { get { return _direction;                     } set { _direction                     = value; Refresh(); } }
        [SerializeField, HideInInspector] private bool      _includeAxisLineInShowingRatio = true;             public bool      includeAxisLineInShowingRatio { get { return _includeAxisLineInShowingRatio; } set { _includeAxisLineInShowingRatio = value; Refresh(); } }
        [SerializeField, HideInInspector] private float     _showingRatio                  = 1f;               public float     showingRatio                  { get { return _showingRatio;                  } set { _showingRatio                  = value; Refresh(); } }

        [SerializeField, HideInInspector] private float _axisLineThickness = 2f;      public float axisLineThickness { get { return _axisLineThickness; } set { _axisLineThickness = value; Refresh(); } } 
        [SerializeField, HideInInspector] private Color _axisLineColor = Color.white; public Color axisLineColor     { get { return _axisLineColor;     } set { _axisLineColor     = value; Refresh(); } }

        [SerializeField, HideInInspector] private int   _gridLineAmounts   = 21;          public int   gridLineAmounts   { get { return _gridLineAmounts;   } set { _gridLineAmounts   = value; Refresh(); } }
        [SerializeField, HideInInspector] private float _gridLineLength    = 5f;          public float gridLineLength    { get { return _gridLineLength;    } set { _gridLineLength    = value; Refresh(); } }
        [SerializeField, HideInInspector] private float _gridLineThickness = 2f;          public float gridLineThickness { get { return _gridLineThickness; } set { _gridLineThickness = value; Refresh(); } }
        [SerializeField, HideInInspector] private Color _gridLineColor     = Color.white; public Color gridLineColor     { get { return _gridLineColor;     } set { _gridLineColor     = value; Refresh(); } }
        
        [SerializeField, HideInInspector] private bool  _showMajorTick        = true; public bool  showMajorTick        { get { return _showMajorTick;        } set { _showMajorTick        = value; Refresh(); } }
        [SerializeField, HideInInspector] private int   _majorTickSpan        = 4;    public int   majorTickSpan        { get { return _majorTickSpan;        } set { _majorTickSpan        = value; Refresh(); } }
        [SerializeField, HideInInspector] private float _majorTickScaleFactor = 2f;   public float majorTickScaleFactor { get { return _majorTickScaleFactor; } set { _majorTickScaleFactor = value; Refresh(); } }

        private RectTransform  _rect;

        private List<float>    _gridVerticesXStartPoints = new List<float>();
        private List<float>    _gridVerticesYStartPoints = new List<float>();
        private List<float>    _gridVerticesYEndPoints   = new List<float>();

        private List<CustomUIVertex> _UIvertices = new List<CustomUIVertex>();

        private bool _isInitialized = false;
        private bool _isEnabled     = true;

        private void OnEnable()
        {
            _isEnabled = true;
            Initialize();
            Refresh();
        }

        private void OnDisable()
        {
            _isEnabled = false;
            Initialize();
            Clear();
        }

        private void Reset()
        {
            Initialize();
            Refresh();
        }

        internal override void Initialize()
        {
            base.Initialize();
            _rect = GetComponent<RectTransform>();
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
            if (!_isEnabled) return;

            if (!_isInitialized) Initialize();
            if (_rect == null) return;
            if (_axisLineThickness < 0f)    _axisLineThickness    = 0f;
            if (_gridLineAmounts < 1)       _gridLineAmounts      = 1;
            if (_gridLineThickness < 0f)    _gridLineThickness    = 0f;
            if (_gridLineLength < 0f)       _gridLineLength       = 0f;
            if (_showingRatio < 0f)         _showingRatio         = 0f;
            if (_showingRatio > 1f)         _showingRatio         = 1f;
            if (_majorTickSpan < 1)         _majorTickSpan        = 1;
            if (_majorTickScaleFactor < 0f) _majorTickScaleFactor = 0f;

            Draw();
        }

        private void Clear()
        {
            _UIvertices.Clear();
            ClearMesh();
        }

        private void Draw()
        {
            _UIvertices.Clear();

            GenerateGridMesh();

            if (_UIvertices.Count == 0) {
                Clear();
                return;
            }

            for (int i = 0; i < _UIvertices.Count; i++) {
                Vector3 uvPoint = _UIvertices[i].vertex.LocalPosToNormalizedPos(_rect);
                _UIvertices[i].uv = new Vector2(uvPoint.x, uvPoint.y);
            }

            GenerateMesh(_UIvertices);
        }

        private void GenerateLeftSideGridMesh(float showingRatio, float axisLineCeneterPoint, float gridLineLengthMultiplier, float gridLineLength)
        {
            GridMeshBuildHelper gridLineBuilder = new GridMeshBuildHelper(_rect, axisLineCeneterPoint, _axisLineThickness, -1, gridLineLength, _gridLineThickness, _gridLineAmounts, 
                                                                          _majorTickSpan, gridLineLengthMultiplier, _axisLineColor, _gridLineColor);
            _UIvertices.AddRange(gridLineBuilder.CreateVertices(showingRatio));
        }

        private void GenerateRightSideGridMesh(float showingRatio, float axisLineCeneterPoint, float gridLineLengthMultiplier, float gridLineLength)
        {
            GridMeshBuildHelper gridLineBuilder = new GridMeshBuildHelper(_rect, axisLineCeneterPoint, _axisLineThickness, 1, gridLineLength, _gridLineThickness, _gridLineAmounts, 
                                                                          _majorTickSpan, gridLineLengthMultiplier, _axisLineColor, _gridLineColor);
            _UIvertices.AddRange(gridLineBuilder.CreateVertices(showingRatio));
        }

        private void GenerateGridMesh()
        {
            if (_rect == null) return;
            
            Vector2 rectSize = _rect.GetRectSize();
            float leftSideXPos             = -rectSize.x * _rect.pivot.x;
            float axisLineCeneterPoint     = 0f;
            float gridLineLengthMultiplier = (_showMajorTick) ? _majorTickScaleFactor : 1f;

            switch (_direction) {
                case Direction.Left:
                    axisLineCeneterPoint = leftSideXPos + _axisLineThickness / 2f;
                    GenerateRightSideGridMesh(_showingRatio, axisLineCeneterPoint, gridLineLengthMultiplier, _gridLineLength);
                    break;
                case Direction.Right:
                    axisLineCeneterPoint = leftSideXPos + rectSize.x - _axisLineThickness / 2f;
                    GenerateLeftSideGridMesh(_showingRatio, axisLineCeneterPoint, gridLineLengthMultiplier, _gridLineLength);
                    break;
                case Direction.Center:
                    axisLineCeneterPoint = leftSideXPos + rectSize.x / 2f;
                    GenerateLeftSideGridMesh (_showingRatio, axisLineCeneterPoint, gridLineLengthMultiplier, _gridLineLength / 2f);
                    GenerateRightSideGridMesh(_showingRatio, axisLineCeneterPoint, gridLineLengthMultiplier, _gridLineLength / 2f);
                    break;
            }
        }
    }

    internal class GridMeshBuildHelper
    {
        internal RectTransform rect;

        internal float axisLineCeneterPoint;
        internal float axisLineThickness;
        internal int   gridLineDirection;
        internal float gridLineLength;
        internal float gridLineThickness;
        internal int   gridLineAmounts;
        internal int   gridLineMajorTickSpan;
        internal float gridLineMajorTickScaleFactor;
        internal Color axisLineColor;
        internal Color gridLineColor;

        internal GridMeshBuildHelper(RectTransform rect, float axisLineCeneterPoint, float axisLineThickness,
                                    int gridLineDirection, float gridLineLength, float gridLineThickness, int gridLineAmounts,
                                    int gridLineMajorTickSpan, float gridLineMajorTickScaleFactor,
                                    Color axisLineColor, Color gridLineColor)
        {
            this.rect                         = rect;
            this.axisLineCeneterPoint         = axisLineCeneterPoint;
            this.axisLineThickness            = axisLineThickness;
            this.gridLineDirection            = gridLineDirection;
            this.gridLineLength               = gridLineLength;
            this.gridLineThickness            = gridLineThickness;
            this.gridLineAmounts              = gridLineAmounts;
            this.gridLineMajorTickSpan        = gridLineMajorTickSpan;
            this.gridLineMajorTickScaleFactor = gridLineMajorTickScaleFactor;
            this.axisLineColor                = axisLineColor;
            this.gridLineColor                = gridLineColor;
        }

        // gridLineDirection : 0 --> 왼쪽을 향하는 grid line
        // gridLineDirection : 1 --> 오른쪽을 향하는 grid line
        private float GetGridLineVertexXStartPoint()
        { 
            return axisLineCeneterPoint + gridLineDirection * axisLineThickness / 2f; 
        }

        private float GetGridLineVertexXEndPoint()
        { 
            return axisLineCeneterPoint + gridLineDirection * (axisLineThickness / 2f + gridLineLength); 
        }

        private float GetGridMajorLineVertexXEndPoint()
        {
            return axisLineCeneterPoint + gridLineDirection * (axisLineThickness / 2f + gridLineLength * gridLineMajorTickScaleFactor); 
        }

        private float GetAxisLineVertexXStartPoint()
        {
            return axisLineCeneterPoint - gridLineDirection * axisLineThickness / 2f;
        }

        private float GetAxisLineVertexXEndPoint()
        {
            return GetGridLineVertexXStartPoint();
        }   

        private Vector2[] GetGridLineYPoints()
        {
            if (gridLineAmounts < 1) return null;

            Vector2 rectSize = rect.GetRectSize();
            float lineSpan = (rectSize.y - gridLineThickness) / (gridLineAmounts - 1);
            Vector2[] gridLineYPoints = new Vector2[gridLineAmounts];
            for (int i = 0; i < gridLineAmounts; i++) {
                gridLineYPoints[i][0] = -rectSize.y * rect.pivot.y + gridLineThickness / 2f - gridLineThickness / 2f + lineSpan * i; // start point
                gridLineYPoints[i][1] = -rectSize.y * rect.pivot.y + gridLineThickness / 2f + gridLineThickness / 2f + lineSpan * i; // end point
            }

            return gridLineYPoints;
        }

        private int GetOptimizedGridLineThickness(Vector2[] gridLineYPoints)
        {
            // Vertex 생성 시 똑같은 line thickness를 적용했더라도, pixel은 discrete하게 계산되기 때문에
            // 실제 렌더링된 화면에서는 line thickness가 다르게 보일 수 있음
            // > 동일한 line thickness를 얻기 위해 pixel에 대한 계산 수행
            // > 라인의 위치 및 두께에 따라 라인 별 pixel 수가 달라질 수 있기 때문에, 최소 pixel 수로 line thickness를 고정함

            int optimizedLineThickness = 0;
            for (int i = 0; i < gridLineYPoints.Length; i++) {
                Vector2 gridLineYStartPointOnScreen = new Vector2(0f, gridLineYPoints[i][0]).LocalPosToScreenPos(rect);
                Vector2 gridLineYEndPointOnScreen   = new Vector2(0f, gridLineYPoints[i][1]).LocalPosToScreenPos(rect);
                gridLineYStartPointOnScreen = new Vector2((int)gridLineYStartPointOnScreen.x, (int)gridLineYStartPointOnScreen.y);   
                gridLineYEndPointOnScreen   = new Vector2((int)gridLineYEndPointOnScreen.x,   (int)gridLineYEndPointOnScreen.y);

                if (i == 0) optimizedLineThickness = (int)Vector2.Distance(gridLineYStartPointOnScreen, gridLineYEndPointOnScreen);
                else        optimizedLineThickness = Mathf.Min(optimizedLineThickness, (int)Vector2.Distance(gridLineYStartPointOnScreen, gridLineYEndPointOnScreen));
            }

            return optimizedLineThickness;
        }

        private void CreateVertex(List<CustomUIVertex> UIvertices, Vector2 startPoint, Vector2 sizeDelta, Color color)
        {
            int vertexCount = UIvertices.Count;

            Vector2 screenSpaceStartPoint = new Vector2(startPoint.x, startPoint.y).LocalPosToScreenPos(rect);
            Vector2 newStartPoint = new Vector2((int)screenSpaceStartPoint.x, (int)screenSpaceStartPoint.y).ScreenPosToLocalPos(rect);

            UIvertices.Add(new CustomUIVertex(new Vector3(newStartPoint.x,               newStartPoint.y,               0f)));
            UIvertices.Add(new CustomUIVertex(new Vector3(newStartPoint.x + sizeDelta.x, newStartPoint.y,               0f)));
            UIvertices.Add(new CustomUIVertex(new Vector3(newStartPoint.x + sizeDelta.x, newStartPoint.y + sizeDelta.y, 0f)));

            UIvertices.Add(new CustomUIVertex(new Vector3(newStartPoint.x,               newStartPoint.y,               0f)));
            UIvertices.Add(new CustomUIVertex(new Vector3(newStartPoint.x,               newStartPoint.y + sizeDelta.y, 0f)));
            UIvertices.Add(new CustomUIVertex(new Vector3(newStartPoint.x + sizeDelta.x, newStartPoint.y + sizeDelta.y, 0f)));

            for (int i = vertexCount; i < UIvertices.Count; i++) {
                UIvertices[i].color = color;
            }
        }

        internal List<CustomUIVertex> CreateVertices(float showingRatio)
        {
            List<CustomUIVertex> UIvertices = new List<CustomUIVertex>();

            if (gridLineAmounts < 1) return UIvertices;
            if (showingRatio == 0)   return UIvertices;
            
            Vector2[] gridLineYPoints = GetGridLineYPoints();

            int newGridLineThickness  = GetOptimizedGridLineThickness(gridLineYPoints);
            float gridLineXStartPoint = GetGridLineVertexXStartPoint();
            float gridLineXEndPoint;

            for (int i = 0; i < gridLineYPoints.Length; i++) {
                if ((float)i / (gridLineYPoints.Length - 1) > showingRatio) break;
                if (i % gridLineMajorTickSpan == 0) {
                    gridLineXEndPoint = GetGridMajorLineVertexXEndPoint();
                }
                else {
                    gridLineXEndPoint = GetGridLineVertexXEndPoint();
                }
                Vector2 gridLineStartPoint = new Vector2(gridLineXStartPoint, gridLineYPoints[i][0]);
                Vector2 gridLineSizeDelta  = new Vector2(gridLineXEndPoint - gridLineXStartPoint, newGridLineThickness);

                CreateVertex(UIvertices, gridLineStartPoint, gridLineSizeDelta, gridLineColor);
            }

            float axisLineXStartPoint = GetAxisLineVertexXStartPoint();
            float axisLineXEndPoint   = GetAxisLineVertexXEndPoint();
            // Axis line이 Grid line의 끝을 넘지 않도록 조정
            // (Grid line의 두께 및 위치가 픽셀 기준으로 수정되었기 때문에, 항상 위쪽 끝에 위치하지 않을 수 있음)
            float axisLineYEndPoint   = Mathf.Min(rect.GetRectSize().y * showingRatio + gridLineYPoints[0][0], gridLineYPoints[gridLineYPoints.Length - 1][0] + newGridLineThickness);
            CreateVertex(UIvertices, new Vector2(axisLineXStartPoint, gridLineYPoints[0][0]), new Vector2(axisLineXEndPoint - axisLineXStartPoint, axisLineYEndPoint - gridLineYPoints[0][0]), axisLineColor);
            
            return UIvertices;
        }
    }
}