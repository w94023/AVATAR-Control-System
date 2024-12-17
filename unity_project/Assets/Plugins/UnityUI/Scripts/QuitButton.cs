using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;
#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.Events;
#endif

namespace UnityUI
{
    [ExecuteInEditMode]
    [RequireComponent(typeof(RectTransform))]
    public class QuitButton : MonoBehaviour
    {
        [SerializeField, HideInInspector] private Image  _image;
        [SerializeField, HideInInspector] private Button _button;

        private void OnEnable()
        {
            if (_image == null) {
                _image = this.gameObject.InitializeComponent<Image>();
            }
            if (_button == null) {
                _button = this.gameObject.InitializeComponent<Button>();
            }

            // 아래 코드는 유니티 에디터 환경에서만 동작함
            #if UNITY_EDITOR
            if (_image != null) {
                // UIQuit sprite를 Assets/Plugins/UnityUI/Resources 경로에서 찾아서 할당
                // 파일명이나 경로가 바뀌게 될 경우 적절히 수정해야 함
                string[] guids = AssetDatabase.FindAssets("UIQuit", new[] { "Assets/Plugins/UnityUI/Resources" });
                if (guids.Length > 0) {
                    string assetPath = AssetDatabase.GUIDToAssetPath(guids[0]);
                    Sprite loadedSprite = AssetDatabase.LoadAssetAtPath<Sprite>(assetPath);
                    if (loadedSprite != null) {
                        _image.sprite = loadedSprite;
                    }
                    else {
                        Debug.LogError("Failed to load icon image for quit button");
                    }
                }
                else {
                    Debug.LogError("Failed to load icon image for quit button");
                }
            }
            if (_button != null) {
                // 버튼 이벤트 등록
                UnityAction action = QuitApplication;
               // 이미 이벤트가 등록되어 있는지 확인합니다.
                bool alreadyRegistered = false;
                int persistentEventCount = _button.onClick.GetPersistentEventCount();
                for (int i = 0; i < persistentEventCount; i++) {
                    if ((object)_button.onClick.GetPersistentTarget(i) == action.Target &&
                        (object)_button.onClick.GetPersistentMethodName(i) == action.Method.Name) {
                        alreadyRegistered = true;
                        break;
                    }
                }
                if (!alreadyRegistered) {
                    // Button의 onClick 이벤트에 새로운 이벤트를 영구적으로 등록합니다.
                    UnityEventTools.AddPersistentListener(_button.onClick, action);
                    EditorUtility.SetDirty(_button);  // 변경된 내용을 저장
                }
            }
            #endif
        }

        private void Reset()
        {
            OnEnable();
        }

        private void Update()
        {
            #if UNITY_EDITOR
            if (Application.isPlaying) {
                // 에디터에서 PlayMode일 때만 실행되는 코드
                goto Quit;
            }
            #else
            // 빌드된 상태에서 실행 중일 때만 실행되는 코드
            goto Quit;
            #endif

            Quit:
            if (Input.GetKeyDown(KeyCode.Escape)) {
                QuitApplication();
            }
        }

        private void QuitApplication()
        {
            // 유니티 에디터에서 실행 중이라면 에디터 종료
            #if UNITY_EDITOR
            UnityEditor.EditorApplication.isPlaying = false;
            #else
            // 빌드된 게임이라면 어플리케이션 종료
            Application.Quit();
            #endif
        }
    }
}
