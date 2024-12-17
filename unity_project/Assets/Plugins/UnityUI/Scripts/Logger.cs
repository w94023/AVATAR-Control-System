using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace UnityUI
{
    [ExecuteInEditMode]
    [RequireComponent(typeof(RectTransform))]
    public class Logger : MonoBehaviour
    {
        [SerializeField, HideInInspector] private GameObject _logObject;
        [SerializeField, HideInInspector] private ScrollRect _log;
        [SerializeField, HideInInspector] private Text       _text;

        public Color normalLogColor  = Color.white;
        public Color warningLogColor = new Color(237f/255f, 177f/255f, 32f/255f, 1f);
        public Color errorLogColor   = new Color(195f/255f, 75f /255f, 73f/255f, 1f);

        private int _maxLength = 10000;  // 최대 길이 설정

        private void OnEnable()
        {
            if (_log == null) {
                var _logObject = ComponentHelper.CreateEmptyObject(this.gameObject, "Scroll view");
                var scrollViewRect   = _logObject.InitializeComponent<RectTransform>();
                var scrollViewCreate = _logObject.InitializeComponent<ScrollViewCreater>();
                scrollViewRect.anchorMin = Vector2.zero;
                scrollViewRect.anchorMax = Vector2.one;
                scrollViewRect.offsetMin = Vector2.zero;
                scrollViewRect.offsetMax = Vector2.zero;
                _log  = scrollViewCreate.scrollRect;
                _text = scrollViewCreate.text;
            }
            Application.logMessageReceived += HandleLog;
        }

        private void OnDisable()
        {
            Application.logMessageReceived -= HandleLog;
        }

        private void HandleLog(string logString, string stackTrace, LogType type)
        {
            #if UNITY_EDITOR
            if (Application.isPlaying) {
                // 에디터에서 PlayMode일 때만 실행되는 코드
                goto Logging;
            }
            #else
            // 빌드된 상태에서 실행 중일 때만 실행되는 코드
            goto Logging;
            #endif

            Logging:
            if (type == LogType.Warning) {
                _text.text += ColorText(logString, warningLogColor);
            }
            else if (type == LogType.Error) {
                _text.text += ColorText(logString, errorLogColor);
            }
            else {
                _text.text += ColorText(logString, normalLogColor);
            }
            _text.text += "\n";
            // _text.text += logString + "\n";

            // text가 최대 길이를 넘겼을 시 먼저 받은 로그를 삭제하는 기능
            if (_text.text.Length > _maxLength) {
                string textTemp = _text.text.Substring(_text.text.Length - _maxLength);
                string[] splitText = textTemp.Split('\n');
                string[] newSplitText = new string[splitText.Length - 1];
                System.Array.Copy(splitText, 1, newSplitText, 0, newSplitText.Length);
                _text.text = string.Join("\n", newSplitText);
            }
            
            StartCoroutine(SetSliderInZeroPoisition());
        }

        string ColorText(string text, Color color)
        {
            // Color 타입을 HTML 색상 코드 형식으로 변환
            string colorString = ColorUtility.ToHtmlStringRGBA(color);
            return "<color=#" + colorString + ">" + text + "</color>";
        }

        private IEnumerator SetSliderInZeroPoisition()
        {
            yield return null;
            _log.verticalNormalizedPosition = 0f;
        }
    }
}

