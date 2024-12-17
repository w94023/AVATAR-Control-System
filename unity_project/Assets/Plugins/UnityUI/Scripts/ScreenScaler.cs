using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace UnityUI
{
    [ExecuteInEditMode]
    public class ScreenScaler : MonoBehaviour
    {
        public GameObject      canvas;
        private RectTransform _canvasRect;

        [Space(20)]
        // 스케일 조정 대상이 되는 씬의 배경
        public GameObject      targetBackground;
        private RectTransform _targetBackgroundRect;
        // 스케일 조정 시 빈 공간을 채울 배경
        public GameObject      baseBackground;
        private RectTransform _baseBackgroundRect;
        

        [Space(20)]
        public Vector2 targetScreenSize;

        void OnEnable()
        {
            if (canvas == null)           return;
            if (targetBackground == null) return;
            if (baseBackground == null)   return;

            if (_canvasRect == null) {
                _canvasRect = canvas.GetComponent<RectTransform>();
            }
            if (_targetBackgroundRect == null) {
                _targetBackgroundRect = targetBackground.GetComponent<RectTransform>();
            }
            if (_baseBackgroundRect == null) {
                _baseBackgroundRect = baseBackground.GetComponent<RectTransform>();
            }

            SetConfig();
        }

        void SetConfig()
        {
            if (_targetBackgroundRect == null) return;

            if (_targetBackgroundRect.sizeDelta != targetScreenSize) {
                _targetBackgroundRect.sizeDelta = targetScreenSize;
            }

            _baseBackgroundRect.anchorMin = new Vector2(0, 0);  // 왼쪽 아래 모서리
            _baseBackgroundRect.anchorMax = new Vector2(1, 1);  // 오른쪽 위 모서리
            _baseBackgroundRect.offsetMin = Vector2.zero;      // 왼쪽 아래 Offset 초기화
            _baseBackgroundRect.offsetMax = Vector2.zero;      // 오른쪽 위 Offset 초기화
        }

        void Update()
        {
            if (_canvasRect == null || _targetBackgroundRect == null || _baseBackgroundRect == null) OnEnable();

            SetConfig();

            float widthRatio  = _canvasRect.rect.width / targetScreenSize.x;
            float heightRatio = _canvasRect.rect.height / targetScreenSize.y;

            if (widthRatio > heightRatio) {
                _targetBackgroundRect.localScale = new Vector3(heightRatio, heightRatio, 1f);
            }
            else {
                _targetBackgroundRect.localScale = new Vector3(widthRatio, widthRatio, 1f);
            }

            // Debug.Log(_canvasRect.rect.width.ToString() + ", " + _canvasRect.rect.height.ToString());
        }
        
    }
}
