using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityUI;

namespace UnityUI
{
    [RequireComponent(typeof(CanvasRenderer))]
    [ExecuteInEditMode]
    public class GradientMesh : BaseMeshEffect
    {
        [SerializeField, HideInInspector] private Color _startColor = new Color(0.1176f, 0.8431f, 0.6706f); public Color startColor { set { _startColor = value; Refresh(); } get { return _startColor; } }
        [SerializeField, HideInInspector] private Color _endColor   = new Color(0.0078f, 0.6549f, 0.9725f); public Color endColor   { set { _endColor   = value; Refresh(); } get { return _endColor;   } }
        [Range(-180f, 180f)]
        [SerializeField, HideInInspector] private float _angle = 0f; public float angle { set { _angle = value; Refresh(); } get { return _angle; } }

        private CanvasRenderer    _canvasRenderer;
        private CustomMeshManager _customMeshManager;
        private Graphic           _graphic;

        private List<Vector3>  _vertices   = new List<Vector3>();
        private List<Color>    _colors     = new List<Color>();
        private List<Vector2>  _uv         = new List<Vector2>();
        private List<int>      _triangles  = new List<int>();

        private bool _isInitialized       = false;
        private bool _isCustomMeshBuilder = false;
        private bool _isValid             = false;

        protected override void OnEnable()
        {
            Initialize();
            Refresh();
        }

        protected override void OnDisable()
        {
            if (_isCustomMeshBuilder && _isValid) {
                _customMeshManager.onMeshModified -= UpdateCustomMesh;
            }
        }

    #if UNITY_EDITOR
        protected override void Reset()
        {
            Initialize();
            Refresh();
        }
    #endif

        private void RemoveMethodInAction(Action<Mesh> action, Action<Mesh> targetMethod)
        {
            if (action == null) return;

            foreach (var existingMethod in action.GetInvocationList()) {
                if (existingMethod.Method == targetMethod.Method) {
                    action -= (Action<Mesh>)existingMethod;
                }
            }
        }

        public void Set(CustomMeshManager customMeshManager)
        {
            _customMeshManager = customMeshManager;
            _isInitialized = true;
            _isValid = true;
            _isCustomMeshBuilder = true;
            RemoveMethodInAction(_customMeshManager.onMeshModified, UpdateCustomMesh);
            _customMeshManager.onMeshModified += UpdateCustomMesh;
        }

        private void Initialize()
        {
            _canvasRenderer    = GetComponent<CanvasRenderer>();
            _customMeshManager = GetComponent<CustomMeshManager>();
            _graphic           = GetComponent<Graphic>();

            _isInitialized = true;

            if (_customMeshManager == null && _graphic == null) {
                _isValid = false;
                return;
            }
            else {
                _isValid = true;
                if (_customMeshManager != null) {
                    _isCustomMeshBuilder = true;
                    RemoveMethodInAction(_customMeshManager.onMeshModified, UpdateCustomMesh);
                    _customMeshManager.onMeshModified += UpdateCustomMesh;
                }
                else {
                    _isCustomMeshBuilder = false;
                    _graphic.SetVerticesDirty();
                    _graphic.SetLayoutDirty();
                }
            }
        }

        private void UpdateCustomMesh(Mesh mesh)
        {
            if (mesh == null)        return;
            if (mesh.uv.Length == 0) return;
            mesh.colors = GetColorFromUVs(new List<Vector2>(mesh.uv)).ToArray();
            _customMeshManager.ReGenerateMesh(mesh);
        }

        private void Refresh()
        {
            if (!_isInitialized) Initialize();
            if (!_isValid)       return;

            if (_isCustomMeshBuilder) {
                UpdateCustomMesh(_customMeshManager.mesh);
            }
            else {
                if (_vertices.Count == 0) return;
                Mesh mesh = new Mesh();
                mesh.vertices  = _vertices.ToArray();
                mesh.triangles = _triangles.ToArray();
                mesh.colors    = GetColorFromUVs(_uv).ToArray();
                mesh.uv        = _uv.ToArray();
                mesh.RecalculateNormals();
                _canvasRenderer.SetMesh(mesh);
            }
        }

    #if UNITY_EDITOR
        protected override void OnValidate()
        {
            Refresh();
        }
    #endif

        public override void ModifyMesh(VertexHelper vh)
        {
            _vertices.Clear();
            _colors.Clear();
            _uv.Clear();
            _triangles.Clear();

            List<UIVertex> vertices = new List<UIVertex>();
            vh.GetUIVertexStream(vertices);

            for (int i = 0; i < vertices.Count; i++)
            {
                UIVertex vertex = vertices[i];
                Vector2 uv = new Vector2(vertex.uv0.x, vertex.uv0.y);
                vertex.color = GetGradientColor(new Vector2(vertex.uv0.x, vertex.uv0.y)); // 색상 변경
                vertices[i] = vertex;

                _vertices.Add(vertex.position);
                _uv.      Add(uv);
            }
            _triangles = BuildTrianglesFromVertices(_vertices);

            vh.Clear(); // 기존 메쉬 데이터 클리어
            vh.AddUIVertexTriangleStream(vertices); // 수정된 메쉬 데이터 추가
        }

        private List<int> BuildTrianglesFromVertices(List<Vector3> vertices)
        {
            if (vertices.Count % 3 > 0) return null;
            
            List<int> triangles = new List<int>();
            for (int i = 0; i < vertices.Count; i++) {
                triangles.Add(i);
            }

            return triangles;
        }

        private Color GetGradientColor(Vector2 uv)
        {
            // uv 포인트 -1 ~ 1 범위로 변경
            float xPoint = -1 + 2 * uv.x;
            float yPoint = -1 + 2 * uv.y;
            // 포인트의 길이 및 기울기 계산
            float vLength = new Vector2(xPoint, yPoint).magnitude;
            float vSlope  = Mathf.Atan2(yPoint, xPoint);
            // _Angle을 이용한 오프셋 적용
            float newXPoint = vLength * Mathf.Sin(vSlope + _angle / 180f * Mathf.PI);
            float newYPoint = vLength * Mathf.Cos(vSlope + _angle / 180f * Mathf.PI);

            float dist = 0.5f * newYPoint + 0.5f;
            Color col = Color.Lerp(_startColor, _endColor, dist);
            return col;
        }

        private List<Color> GetColorFromUVs(List<Vector2> uv)
        {
            List<Color> colors = new List<Color>();
            if (uv.Count == 0) return colors;

            for (int i = 0 ; i < uv.Count; i++) {
                colors.Add(GetGradientColor(uv[i]));
            }

            return colors;
        }
    }
}