using UnityEngine;
using UnityEditor;
using UnityUI;

[CustomEditor(typeof(PanelFader))]
[CanEditMultipleObjects]
public class PanelFaderEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();
        serializedObject.Update();
        
        PanelFader _target = (PanelFader)target;

        EditorGUILayout.Space();
        GUILayout.Label("Fade target objects", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_targetObjects"), new GUIContent("targetObjects"), true);
        EditorGUI.indentLevel = 0;

        EditorGUILayout.Space();
        GUILayout.Label("Showing options", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_animationCurve"), new GUIContent("animationCurve"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_animatingTime"), new GUIContent("animatingTime"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_showingPanelIndex"), new GUIContent("showingPanelIndex"), true);
        EditorGUI.indentLevel = 0;

        EditorGUILayout.Space();
        GUILayout.Label("Methods", EditorStyles.boldLabel);
        EditorGUILayout.BeginHorizontal();
        GUILayout.Space(15f);
        if (GUILayout.Button("Fade")) {
            _target.Fade();
        }
        EditorGUILayout.EndHorizontal();
        
        serializedObject.ApplyModifiedProperties();
    }
}
