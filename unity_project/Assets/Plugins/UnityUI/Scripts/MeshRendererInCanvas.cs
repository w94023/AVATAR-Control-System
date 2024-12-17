using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace Plugin.UI
{
    [ExecuteInEditMode]
    [RequireComponent(typeof(CanvasRenderer))]
    public class MeshRendererInCanvas : MonoBehaviour
    {
        public Mesh     _mesh;     public Mesh     mesh     { set { _mesh     = value; OnPropertyChanged(); } get { return _mesh    ; } }
        public Material _material; public Material material { set { _material = value; OnPropertyChanged(); } get { return _material; } }
        // public Vector3  _angle;    public Vector3  angle    { set { _angle    = value; OnPropertyChanged(); } get { return _angle   ; } }

        private Mesh _copiedMesh;
        private Vector3[] vertices;
        private CanvasRenderer _canvasRenderer;
        // private bool isHolding = false;

        private bool _isInitialized = false;

        private void Initialize()
        {
            // 메쉬 생성
            _copiedMesh = new Mesh();

            // CanvasRenderer 참조
            _canvasRenderer = GetComponent<CanvasRenderer>();

            _isInitialized = true;
        }

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
        
        private void Awake()
        {
            Initialize();
            Refresh();
        }

        private void OnValidate()
		{
			Refresh();
		}

        private void Refresh()
        {
            if (!_isInitialized) {
                Initialize();
            }

            // 주어진 Mesh가 없을 경우 작업 종료
            if (_mesh == null) {
                return;
            }

            // 주어진 Material이 없을 경우 작업 종료
            if (_material == null) {
                return;
            }

            // Material 설정
            _canvasRenderer.SetMaterial(_material, null);

            // 주어진 angle을 2D vertex에 반영
            // Quaternion rotation = Quaternion.Euler(_angle);

            // Mesh 이름 생성
            _copiedMesh.name = _mesh.name + " Copy";

            // 버텍스 복사
            // vertices = _mesh.vertices;
            // for (int i = 0; i < vertices.Length; i++) {
            //     vertices[i] = rotation * vertices[i];
            // }

            // 메쉬 설정
            _copiedMesh.vertices     = _mesh.vertices;
            _copiedMesh.normals      = _mesh.normals;
            _copiedMesh.tangents     = _mesh.tangents;
            _copiedMesh.uv           = _mesh.uv;
            _copiedMesh.uv2          = _mesh.uv2;
            _copiedMesh.colors       = _mesh.colors;
            _copiedMesh.colors32     = _mesh.colors32;
            _copiedMesh.subMeshCount = _mesh.subMeshCount;

            SetMesh();
        }

        public void SetMesh()
        {
            // 설정된 메쉬를 화면상에 보이도록 설정
            for (int i = 0; i < _mesh.subMeshCount; i++) {
                _copiedMesh.SetTriangles(_mesh.GetTriangles(i), i);
            }
            _canvasRenderer.SetMesh(_copiedMesh);
        }

        public void OnPropertyChanged()
        {
        #if UNITY_EDITOR
            Refresh();
        #endif
        }

        // public void HoldCurrentMeshPosture()
        // {
        //     if (vertices == null) return;
        //     isHolding = true;
        //     for (int i = 0; i < vertices.Length; i++) {
        //         vertices[i].z = 0f;
        //     }
        //     _copiedMesh.vertices = vertices;
        //     _canvasRenderer.SetMesh(_copiedMesh);
        // }

        // public void Release()
        // {
        //     isHolding = false;
        // }
    }
}

