using UnityEngine;
using UnityEditor;
using UnityUI;

[CustomEditor(typeof(GridLine))]
[CanEditMultipleObjects]
public class GridLineEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();
        serializedObject.Update();

        GridLine _target = (GridLine)target;

        EditorGUILayout.Space();
        GUILayout.Label("Object graphic options", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_direction"),                     new GUIContent("direction"),                     true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_includeAxisLineInShowingRatio"), new GUIContent("includeAxisLineInShowingRatio"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_showingRatio"),                  new GUIContent("showingRatio"),                  true);
        EditorGUI.indentLevel = 0;

        EditorGUILayout.Space();
        GUILayout.Label("Axis graphic options", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_axisLineThickness"), new GUIContent("axisLineThickness"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_axisLineColor"),     new GUIContent("axisLineColor"), true);
        EditorGUI.indentLevel = 0;

        EditorGUILayout.Space();
        GUILayout.Label("Grid graphic options", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_gridLineAmounts"),   new GUIContent("gridLineAmounts"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_gridLineLength"),    new GUIContent("gridLineLength"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_gridLineThickness"), new GUIContent("gridLineThickness"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_gridLineColor"),     new GUIContent("gridLineColor"), true);
        EditorGUI.indentLevel = 0;

        EditorGUILayout.Space();
        GUILayout.Label("Major tick graphic options", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_showMajorTick"), new GUIContent("showMajorTick"), true);
        if (_target.showMajorTick) {
            EditorGUILayout.PropertyField(serializedObject.FindProperty("_majorTickSpan"),        new GUIContent("majorTickSpan"), true);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("_majorTickScaleFactor"), new GUIContent("majorTickScaleFactor"), true);
        }
        EditorGUI.indentLevel = 0;

        serializedObject.ApplyModifiedProperties();
    }
}
