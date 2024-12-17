using UnityEngine;
using UnityEditor;
using UnityUI;

[CustomEditor(typeof(PlotGridBuilder))]
[CanEditMultipleObjects]
public class PlotGridBuilderEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();
        serializedObject.Update();

        EditorGUILayout.Space();
        GUILayout.Label("Mode option", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_mode"),        new GUIContent("mode"),        true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_useAxisLine"), new GUIContent("useAxisLine"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_useGridLine"), new GUIContent("useGridLine"), true);
        EditorGUI.indentLevel = 0;

        EditorGUILayout.Space();
        GUILayout.Label("Grid scale options", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_gridMin"),  new GUIContent("gridMin"),  true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_gridMax"),  new GUIContent("gridMax"),  true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_gridSpan"), new GUIContent("gridSpan"), true);
        EditorGUI.indentLevel = 0;

        EditorGUILayout.Space();
        GUILayout.Label("Graphic options", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_axisLineOffset"),    new GUIContent("axisLineOffset"),    true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_gridLineOffset"),    new GUIContent("gridLineOffset"),    true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_axisLineThickness"), new GUIContent("axisLineThickness"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_gridLineThickness"), new GUIContent("gridLineThickness"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_axisLineColor"),     new GUIContent("axisLineColor"),     true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_gridLineColor"),     new GUIContent("gridLineColor"),     true);
        EditorGUI.indentLevel = 0;

        EditorGUILayout.Space();
        GUILayout.Label("Text options", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_textFont"),     new GUIContent("textFont"),     true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_textSize"),     new GUIContent("textSize"),     true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_textColor"),    new GUIContent("textColor"),    true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_textOffset"),   new GUIContent("textOffset"),   true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_digitPlace"),   new GUIContent("digitPlace"),   true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_decimalPlace"), new GUIContent("decimalPlace"), true);
        EditorGUI.indentLevel = 0;

        serializedObject.ApplyModifiedProperties();
    }
}