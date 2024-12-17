using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;
using UnityEngine.EventSystems;

namespace UnityUI
{
    public class UIExtension : MonoBehaviour
    {
        public UnityEvent onPointerClick;
        public UnityEvent onPointerUp;
        public UnityEvent onPointerDown;
        
        private void Awake()
        {
            // EventTrigger 컴포넌트 추가
            EventTrigger trigger = gameObject.AddComponent<EventTrigger>();

            // PointerClick 이벤트 설정
            EventTrigger.Entry clickEntry = new EventTrigger.Entry { eventID = EventTriggerType.PointerClick };
            clickEntry.callback.AddListener((eventData) => { OnPointerClick(); });
            trigger.triggers.Add(clickEntry);

            // PointerDown 이벤트 설정
            EventTrigger.Entry downEntry = new EventTrigger.Entry { eventID = EventTriggerType.PointerDown };
            downEntry.callback.AddListener((eventData) => { OnPointerDown(); });
            trigger.triggers.Add(downEntry);

            // PointerUp 이벤트 설정
            EventTrigger.Entry upEntry = new EventTrigger.Entry { eventID = EventTriggerType.PointerUp };
            upEntry.callback.AddListener((eventData) => { OnPointerUp(); });
            trigger.triggers.Add(upEntry);
        }

        public void OnPointerClick()
        {
            onPointerClick?.Invoke();
        }

        public void OnPointerUp()
        {
            onPointerUp?.Invoke();
        }

        public void OnPointerDown()
        {
            onPointerDown?.Invoke();
        }
    }
}
