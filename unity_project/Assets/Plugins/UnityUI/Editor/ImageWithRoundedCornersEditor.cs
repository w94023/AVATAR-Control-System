using UnityEngine;
using UnityEditor;
using UnityUI;

[CustomEditor(typeof(ImageWithRoundedCorners))]
[CanEditMultipleObjects]
public class ImageWithRoundedCornersEditor : Editor
{
    public override void OnInspectorGUI()
    {  
        base.OnInspectorGUI();
        serializedObject.Update();

        ImageWithRoundedCorners _target = (ImageWithRoundedCorners)target;

        EditorGUILayout.Space();
        GUILayout.Label("Options", EditorStyles.boldLabel);
        // 들여쓰기 설정
        EditorGUI.indentLevel = 1;
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_radius"),                 new GUIContent("radius"),                 true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_edgeSoftness"),           new GUIContent("edgeSoftness"),           true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_edgeTransitionSoftness"), new GUIContent("edgeTransitionSoftness"), true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_edgeThickness"),          new GUIContent("edgeThickness"),          true);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("_edgeColor"),              new GUIContent("edgeColor"),              true);
        EditorGUI.indentLevel = 0;

        serializedObject.ApplyModifiedProperties();
    }
}
