using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityUI;

[RequireComponent(typeof(CircleGraph))]
public class ProgressIndicator : MonoBehaviour
{
    private CircleGraph   circleGraph;
    private RectTransform rect;
    private Coroutine     routine;

    public Color baseColor  = new Color(30f/255f, 215f/255f, 171f/255f, 1f);
    public Color errorColor = new Color(255f/255f, 133f/255f, 133f/255f, 1f);

    private void Start()
    {
        circleGraph       = this.gameObject.GetComponent<CircleGraph>();
        circleGraph.value = 0f;
        rect              = circleGraph.GetComponent<RectTransform>();
    }

    public void StartProgressing()
    {
        if (routine != null) { 
            StopCoroutine(routine);
        }
        circleGraph.fillColor = baseColor;
        routine = StartCoroutine(StartProgressingAnimtaion());
    }

    public void StopProgressing(bool isConnected)
    {
        if (routine != null) { 
            StopCoroutine(routine);
        }
        if (isConnected) {
            circleGraph.fillColor = baseColor;
            routine = StartCoroutine(StopProgressingAnimation(true));
        }
        else {
            circleGraph.fillColor = errorColor;
            routine = StartCoroutine(StartBlinkAnimation());
        }
        
    }

    public void HoldProgressing()
    {
        if (routine != null) {
            StopCoroutine(routine);
        }
    }

    public void ClearProgressing()
    {
        if (routine != null) { 
            StopCoroutine(routine);
        }
        circleGraph.fillColor = baseColor;
        routine = StartCoroutine(StopProgressingAnimation(false));
    }

    private IEnumerator StartBlinkAnimation()
    {
        Color initialColor = circleGraph.fillColor;
        initialColor.a = 0f;
        circleGraph.value = 1f;
        circleGraph.fillColor = initialColor;

        bool  isLighted = false;
        float sign = 1f;

        while (true) {
            initialColor.a += sign * 0.05f;
            if (initialColor.a >= 1f) {
                isLighted = true;
                sign = -1f;
            }
            else if (initialColor.a <= 0f && isLighted) {
                circleGraph.fillColor = initialColor;
                break;
            }
            circleGraph.fillColor = initialColor;
            yield return new WaitForSeconds(0.005f);
        }

        circleGraph.value = 0f;
    }

    private IEnumerator StartProgressingAnimtaion()
    {
        float valueAdder = 0f;
        float rotAdder = 0f;
        int direction = 0;
        circleGraph.reverse = false;

        float fillingSpeed = 0.002f;
        float rotatingSpeed = 0.1f;

        float fullPointOffset = 0.7f;
        float emptyPointOffset = 0.05f;

        float delayPoint = -0.5f;
        float delayTime = 0.5f;

        float chkTime = Time.realtimeSinceStartup;
        while (true) {
                float diff = Mathf.Abs((1f - fullPointOffset) / 2f - (circleGraph.value - emptyPointOffset)) + 1f;
                valueAdder = fillingSpeed * Mathf.Pow(diff, 3);
                rotAdder = rotatingSpeed * Mathf.Pow(diff, 2);

                float currentTime = Time.realtimeSinceStartup;

                if ((currentTime - delayPoint) > delayTime) {
                    if (direction == 0) {
                        circleGraph.value += valueAdder;
                    }
                    else {
                        circleGraph.value -= valueAdder;
                    }
                }
                
                rect.eulerAngles -= new Vector3(0f, 0f, rotAdder * 2 * Mathf.PI);

                if (circleGraph.value >= fullPointOffset) {
                    if (direction == 0) {
                        direction = 1;
                        valueAdder = 0f;
                        rotAdder = 0f;

                        circleGraph.reverse = true;
                        rect.eulerAngles -= new Vector3(0f, 0f, 360f * circleGraph.value);
                        delayPoint = Time.realtimeSinceStartup;
                    }
                }
                else if (circleGraph.value <= emptyPointOffset) {
                    if (direction == 1) {
                        direction = 0;
                        valueAdder = 0f;
                        rotAdder = 0f;

                        circleGraph.reverse = false;
                        rect.eulerAngles += new Vector3(0f, 0f, 360f * circleGraph.value);
                        delayPoint = Time.realtimeSinceStartup;
                    }
                }

                fillingSpeed = (currentTime - chkTime) / 5f * 2f;
                rotatingSpeed = fillingSpeed * 50f;
                chkTime = Time.realtimeSinceStartup; 
            yield return new WaitForSeconds(0.005f);
        }
    }

    private IEnumerator StopProgressingAnimation(bool isConnected)
    {
        float fillingSpeed = 0.01f;
        float valueAdder = 0f;

        float chkTime = Time.realtimeSinceStartup;
        while (true) {
            float diff = Mathf.Abs(0.5f - circleGraph.value) + 1f;
            valueAdder = fillingSpeed * Mathf.Pow(diff, 2);

            if (isConnected) {
                circleGraph.value += valueAdder;

                if (circleGraph.value >= 1f) {
                    break;
                }
            }
            else {
                circleGraph.value -= valueAdder;

                if (circleGraph.value <= 0f) {
                    break;
                }
            }

            fillingSpeed = (Time.realtimeSinceStartup - chkTime) * 2f;
            chkTime = Time.realtimeSinceStartup;
            yield return new WaitForSeconds(0.005f);
        }

        rect.eulerAngles = new Vector3(0f, 0f, 0f);
    }
}
