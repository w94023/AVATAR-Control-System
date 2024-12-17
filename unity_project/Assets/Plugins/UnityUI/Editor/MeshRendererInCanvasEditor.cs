using UnityEngine;
using UnityEditor;
using Plugin.UI;

[CustomEditor(typeof(MeshRendererInCanvas))]
[CanEditMultipleObjects]
public class MeshRendererInCanvasEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();
        serializedObject.Update();

        MeshRendererInCanvas script = (MeshRendererInCanvas)target;

        EditorGUILayout.BeginHorizontal();
        if(GUILayout.Button("Set mesh")) {
            script.SetMesh();
        }
        EditorGUILayout.EndHorizontal();

        serializedObject.ApplyModifiedProperties();
    }
}
