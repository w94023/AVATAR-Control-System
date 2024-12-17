using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class BlinkImage : MonoBehaviour
{
    public float blinkingSpeed = 0.05f;

    private Image     _target;
    private Color     _initialColor;
    private Coroutine _routine;

    private void Start()
    {
        _target = this.gameObject.GetComponent<Image>();
        if (_target != null) {
            _initialColor = _target.color;   
        }
    }

    public void Blink(float targetAlpha)
    {
        if (_target == null) {
            Debug.Log("Blink target is not set");
            return;
        }

        if (targetAlpha > 1f) targetAlpha = 1f;
        if (targetAlpha < 0f) targetAlpha = 0f;

        if (_routine != null) {
            StopCoroutine(_routine);
        }
        _routine = StartCoroutine(StartBlinkAnimation(targetAlpha));
    }

    private IEnumerator StartBlinkAnimation(float targetAlpha)
    {
        float sign;
        if (_initialColor.a > targetAlpha) {
            sign = -1f;
        }
        else {
            sign = +1f;
        }
        
        Color color = new Color(_initialColor.r, _initialColor.g, _initialColor.b, _initialColor.a);
        bool isReversed = false;
        while (true) {
            color.a += sign * blinkingSpeed;
            _target.color = color;
            if (sign == -1f) {
                if (_target.color.a <= targetAlpha && !isReversed) {
                    isReversed = true;
                    sign = 1f;
                    continue;
                }
                if (_target.color.a <= _initialColor.a && isReversed) {
                    _target.color = _initialColor;
                    break;
                }
            }
            else {
                if (_target.color.a >= targetAlpha && !isReversed) {
                    isReversed = true;
                    sign = -1f;
                    continue;
                }
                if (_target.color.a >= _initialColor.a && isReversed) {
                    _target.color = _initialColor;
                    break;
                }
            }
            yield return new WaitForSeconds(0.005f);
        }
    }
}
