using UnityEngine;
using UnityEditor;
using UnityUI;

[CustomEditor(typeof(ToggleButton))]
[CanEditMultipleObjects]
public class ToggleButtonEditor : Editor
{
    public override void OnInspectorGUI()
    {   
        base.OnInspectorGUI();
        serializedObject.Update();

        EditorGUILayout.Space();
        GUILayout.Label("Graphic options", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_handleEdgeThickness"), new GUIContent("handleEdgeThickness"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_backgroundColor"), new GUIContent("backgroundColor"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_sliderColor"), new GUIContent("sliderColor"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_handleBaseColor"), new GUIContent("handleBaseColor"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_handleEdgeColors"), new GUIContent("handleEdgeColors"), true);
        EditorGUI.indentLevel = 0;

        EditorGUILayout.Space();
        GUILayout.Label("Label graphic options", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_labelText"), new GUIContent("labelText"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_labelColor"), new GUIContent("labelColor"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_labelFontSize"), new GUIContent("labelFontSize"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_labelSpacing"), new GUIContent("labelSpacing"), true);
        EditorGUI.indentLevel = 0;

        EditorGUILayout.Space();
        GUILayout.Label("Toggle options", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_stepAmount"), new GUIContent("stepAmount"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_value"), new GUIContent("value"), true);
        EditorGUI.indentLevel = 0;

        EditorGUILayout.Space();
        GUILayout.Label("Events", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("onClick"), new GUIContent("onClick"), true);
        EditorGUI.indentLevel = 0;

        serializedObject.ApplyModifiedProperties();
    }
}
