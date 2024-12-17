using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace UnityUI
{
    [ExecuteInEditMode]
    public class PanelFader : MonoBehaviour
    {
                                          public  List<GameObject>  targetObject      { get { return _targetObjects; } set { _targetObjects = value; Refresh(); }}
        [SerializeField, HideInInspector] private List<GameObject> _targetObjects     = new List<GameObject>();
        [SerializeField, HideInInspector] private AnimationCurve   _animationCurve    = AnimationCurve.Linear(0, 0, 1, 1);
        [SerializeField, HideInInspector] private float            _animatingTime     = 0.25f; // seconds
                                          public  int               showingPanelIndex { get { return _showingPanelIndex; } set { _showingPanelIndex = value; Refresh(); }}
        [SerializeField, HideInInspector] private int              _showingPanelIndex = 0;

        private List<int>         _validIndex;
        private List<CanvasGroup> _canvasGroups;

        private Coroutine _animationCoroutine;
        private bool      _isValueChanged = false;

        private void Start()
        {
            Refresh();
        }

        private void OnValidate()
        {
            _isValueChanged = true;
        }

        private void LateUpdate()
        {
            if (_isValueChanged) {
                Refresh();
                _isValueChanged = false;
            }
        }

        private void Refresh()
        {
            if (_targetObjects == null)    { _showingPanelIndex = 0; return; }
            if (_targetObjects.Count == 0) { _showingPanelIndex = 0; return; }

            _canvasGroups = new List<CanvasGroup>();
            _validIndex   = new List<int>();
            for (int i = 0; i < _targetObjects.Count; i++) {
                if (_targetObjects[i] == null) continue;
                _validIndex.Add(i);
                _canvasGroups.Add(_targetObjects[i].InitializeComponent<CanvasGroup>());
            }

            if (_validIndex.Count == 0) { _showingPanelIndex = 0; return; }
            if (!_validIndex.Contains(_showingPanelIndex)) _showingPanelIndex = _validIndex[0];

            //  Update alpha
            for (int i = 0; i < _validIndex.Count; i++) {
                if (_showingPanelIndex == _validIndex[i]) {
                    _canvasGroups[i].interactable = true;
                    _canvasGroups[i].blocksRaycasts = true;
                    _canvasGroups[i].alpha = 1f;
                }
                else {
                    _canvasGroups[i].interactable = false;
                    _canvasGroups[i].blocksRaycasts = false;
                    _canvasGroups[i].alpha = 0f;
                }
            }
        }

        private void IncreaseShowingIndex()
        {
            _showingPanelIndex++;
            if (_showingPanelIndex > _validIndex[_validIndex.Count - 1]) _showingPanelIndex = _validIndex[0];
            if (!_validIndex.Contains(_showingPanelIndex)) IncreaseShowingIndex();
        }

        private void SetTargetIndex(int targetIndex)
        {
            if (targetIndex > _validIndex[_validIndex.Count - 1]) return;
            if (!_validIndex.Contains(targetIndex)) return;

            _showingPanelIndex = targetIndex;
        }

        public void Fade()
        {
            if (_targetObjects == null)    { _showingPanelIndex = 0; return; }
            if (_targetObjects.Count == 0) { _showingPanelIndex = 0; return; }
            if (_validIndex.Count == 0)    { _showingPanelIndex = 0; return; }

        #if UNITY_EDITOR
            if (EditorApplication.isPlaying)
            {
                if (_animationCoroutine != null) StopFading();
                _animationCoroutine = StartCoroutine(FadeAnimation());
            }
            else {
                IncreaseShowingIndex();
                Refresh();
            }
        #else
            if (_animationCoroutine != null) StopFading();
            _animationCoroutine = StartCoroutine(FadeAnimation());
        #endif
        }

        public void Fade(int targetIndex)
        {
            if (_targetObjects == null) return;
            if (_targetObjects.Count == 0) return;
            if (_validIndex.Count == 0) return;
            if (_showingPanelIndex == targetIndex) return;

            if (_animationCoroutine != null) StopFading();
            _animationCoroutine = StartCoroutine(FadeAnimation(targetIndex : targetIndex));
        }

        public void Fade(int targetIndex, Action action)
        {
            if (_targetObjects == null) return;
            if (_targetObjects.Count == 0) return;
            if (_validIndex.Count == 0) return;
            if (_showingPanelIndex == targetIndex) {
                action?.Invoke();
                return;
            }

            if (_animationCoroutine != null) StopFading();
            _animationCoroutine = StartCoroutine(FadeAnimation(targetIndex : targetIndex, action : action));
        }

        private void StopFading()
        {
            StopCoroutine(_animationCoroutine);
            for (int i = 0; i < _validIndex.Count; i++) {
                if (i == _validIndex.IndexOf(_showingPanelIndex)) _canvasGroups[i].alpha = 1f;
                else                                              _canvasGroups[i].alpha = 0f;
            }
        }

        private IEnumerator FadeAnimation(int targetIndex = -1, Action action = null)
        {
            for (int i = 0; i < 2; i++) {
                // i = 0 : 현재 패널을 투명하게
                // i = 1 : 다음 패널을 불투명하게
                int sign = (i == 0) ? -1 : 1;
                int   showingPanelIndex = _validIndex.IndexOf(_showingPanelIndex);
                float initialAlpha      = _canvasGroups[showingPanelIndex].alpha;
                float chkTime           = Time.realtimeSinceStartup;
            
                if (i == 0) {
                    if (targetIndex < 0) IncreaseShowingIndex();
                    else                 SetTargetIndex(targetIndex);
                }
                
                for (int j = 0; j < 1000; j++) {
                    float timeSpent = Time.realtimeSinceStartup - chkTime;
                    _canvasGroups[showingPanelIndex].alpha = initialAlpha + sign * _animationCurve.Evaluate( timeSpent/_animatingTime );
                    if (sign == -1) {
                        if (_canvasGroups[showingPanelIndex].alpha == 0) {
                            _canvasGroups[showingPanelIndex].interactable = false;
                            _canvasGroups[showingPanelIndex].blocksRaycasts = false;
                            break;
                        }
                    }
                    else if (sign == 1) {
                        if (_canvasGroups[showingPanelIndex].alpha > 0) {
                            _canvasGroups[showingPanelIndex].interactable = true;
                            _canvasGroups[showingPanelIndex].blocksRaycasts = true;
                        }
                        if (_canvasGroups[showingPanelIndex].alpha == 1) {
                            break;
                        }
                    }
                    
                    yield return new WaitForSeconds(0.005f);
                }
            }

            action?.Invoke();
            _animationCoroutine = null;
        }
    }
}

