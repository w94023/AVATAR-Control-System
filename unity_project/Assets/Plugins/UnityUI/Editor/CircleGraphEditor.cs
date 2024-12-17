using UnityEngine;
using UnityEditor;
using UnityUI;

[CustomEditor(typeof(CircleGraph))]
[CanEditMultipleObjects]
public class CircleGraphEditor : Editor
{
    public override void OnInspectorGUI()
    { 
        base.OnInspectorGUI();
        serializedObject.Update();

        EditorGUILayout.Space();
        GUILayout.Label("Graphic options", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_thickness"), new GUIContent("thickness"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_backgroundColor"), new GUIContent("backgroundColor"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_fillColor"), new GUIContent("fillColor"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_value"), new GUIContent("value"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_reverse"), new GUIContent("reverse"), true);
        EditorGUI.indentLevel = 0;

        serializedObject.ApplyModifiedProperties();
    }
}
