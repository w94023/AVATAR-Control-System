#if UNITY_EDITOR
using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityUI;

public class ResourceManager : IPreprocessBuildWithReport
{
    public int callbackOrder => 0;

    private string[] shaderList = new string[] {
        "GradientMesh"
    };

    public void OnPreprocessBuild(BuildReport report)
    {
        // 유니티 빌드 시작 전에 UI/UnityUI/GradientMesh 쉐이더 Resources 폴더로 복사

        // 복사될 경로
        string targetPath = "Assets/Resources/"; 
        if (!Directory.Exists(targetPath)) {
            Directory.CreateDirectory(targetPath);
        }

        // 프로젝트 내의 모든 쉐이더 찾기
        string[] guids = AssetDatabase.FindAssets("t:Shader");
        foreach (string guid in guids) {
            string path = AssetDatabase.GUIDToAssetPath(guid);
            Shader shader = AssetDatabase.LoadAssetAtPath<Shader>(path);

            if (shader != null) {
                if (path.Contains("Resources")) {
                    // 이미 Resources 폴더에 저장된 것은 건너뜀
                    continue;
                }
                // 쉐이더 이름 확인
                if (shader.name.Contains("UnityUI")) {
                    Debug.Log(shader.name);
                    Debug.Log(path);
                    string[] shaderName = shader.name.Split('/');
                    string shaderTargetPath = targetPath + shaderName[shaderName.Length - 1] + ".shader";
                    // 파일이 이미 존재하면 삭제
                    if (File.Exists(shaderTargetPath)) {
                        File.Delete(shaderTargetPath);
                    }

                    File.Copy(path, shaderTargetPath);
                    Debug.Log(shaderTargetPath + " : saved");
                }
            }
        }

        AssetDatabase.Refresh();
    }
}

#endif
