using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UnityUI 
{
    [RequireComponent(typeof(CanvasRenderer))]
    [ExecuteInEditMode]
    public class CustomMeshManager : MonoBehaviour
    {
        internal Mesh           mesh;
        internal CanvasRenderer canvasRenderer;
        internal Material       canvasMaterial;
        internal Action<Mesh>   onMeshModified;

        internal virtual void Initialize()
        {
            mesh           = new Mesh();
            canvasMaterial = new Material(Shader.Find("Sprites/Default"));
            canvasRenderer = GetComponent<CanvasRenderer>();
            canvasRenderer.SetMaterial(canvasMaterial, null);
        }

        internal virtual Mesh GetMesh()
        {
            return mesh;
        }

        internal virtual void SetMesh(Mesh mesh)
        {
            this.mesh = mesh;
        }

        internal virtual void GenerateMesh(List<CustomUIVertex> UIvertices, Color color = default(Color))
        {
            GenerateMesh(UIvertices.ToArray(), color);
        }

        internal virtual void GenerateMesh(CustomUIVertex[] UIvertices, Color color = default(Color))
        {
            if (canvasRenderer == null) Initialize();

            List<Vector3> vertices   = new List<Vector3>();
            List<Color>   colors     = new List<Color>();
            List<Vector2> uv         = new List<Vector2>();
            List<int>     triangles  = new List<int>();
            
            for (int i = 0; i < UIvertices.Length; i++) {
                CustomUIVertex UIvertex = UIvertices[i];

                vertices. Add(UIvertex.vertex);
                uv.       Add(UIvertex.uv);
                triangles.Add(i);

                if (color == default(Color)) colors.Add(UIvertex.color);
                else                         colors.Add(color);
            }

            Mesh newMesh = new Mesh();
            newMesh.vertices  = vertices.ToArray();
            newMesh.colors    = colors.ToArray();
            newMesh.uv        = uv.ToArray();
            newMesh.triangles = triangles.ToArray();
            newMesh.RecalculateNormals();
            mesh = newMesh;

            canvasRenderer.SetMesh(mesh);
            onMeshModified?.Invoke(mesh);
        }

        internal virtual void GenerateMesh(Mesh mesh)
        {
            canvasRenderer.SetMesh(mesh);
            // onMeshModified?.Invoke(mesh);
        }

        internal virtual void ReGenerateMesh(Mesh mesh)
        {
            ClearMesh();
            mesh.RecalculateNormals();
            canvasRenderer.SetMesh(mesh);
        }

        internal void ClearMesh()
        {
            if (canvasRenderer == null) canvasRenderer = GetComponent<CanvasRenderer>();
            canvasRenderer.Clear();
            canvasRenderer.SetMaterial(canvasMaterial, null);
        }
    }
}