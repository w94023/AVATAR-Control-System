using UnityEngine;
using UnityEditor;
using UnityUI;

[CustomEditor(typeof(GradientMesh))]
[CanEditMultipleObjects]
public class GradientMeshEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();
        serializedObject.Update();

        EditorGUILayout.Space();
        GUILayout.Label("Gradient color options", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_startColor"), new GUIContent("startColor"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_endColor"),   new GUIContent("endColor"),   true);
        EditorGUI.indentLevel = 0;

        EditorGUILayout.Space();
        GUILayout.Label("Gradient angle option", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_angle"), new GUIContent("angle (deg)"), true);
        EditorGUI.indentLevel = 0;

        serializedObject.ApplyModifiedProperties();
    }
}
