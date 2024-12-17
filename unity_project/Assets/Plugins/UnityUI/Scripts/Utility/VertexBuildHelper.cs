using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UnityUI
{
    internal class CustomUIVertex
    {
        internal Vector3 vertex;
        internal Color   color;
        internal Vector2 uv;

        internal CustomUIVertex()
        {
            this.vertex = new Vector3(0f, 0f, 0f);
            this.color  = Color.white;
            this.uv     = new Vector2(0.5f, 0.5f);
        }

        internal CustomUIVertex(Vector3 vertex)
        {
            this.vertex = vertex;
            this.color  = Color.white;
            this.uv     = new Vector2(0.5f, 0.5f);
        }
    };

    internal static class VertexBuildHelper
    {
        internal static Vector2 LocalPosToScreenPos(this Vector2 localPoint, RectTransform rect)
        {
            Vector3 worldPoint = rect.TransformPoint(localPoint);
            Vector2 screenPoint = RectTransformUtility.WorldToScreenPoint(null, worldPoint);

            return screenPoint;
        }

        internal static Vector2 ScreenPosToLocalPos(this Vector2 screenPoint, RectTransform rect)
        {
            RectTransformUtility.ScreenPointToLocalPointInRectangle(rect, screenPoint, null, out Vector2 localPoint);
            return localPoint;
        }

        internal static Vector3 NormalizedPosToLocalPos(this Vector3 normalizedPos, RectTransform rect, bool applyPivot = true)
        {
            // 현재 오브젝트의 RectTransform 정보 확인
            // AnchorMin.x와 AnchorMax.x, AnchorMin.y와 AnchorMin.y가 서로 다를 경우
            // Strecth모드가 되기 때문에 스크립트에서 넓이와 높이를 확인할 수 없음
            // > 이럴 경우 부모 오브젝트의 넓이와 높이로부터 계산함

            Vector3 localPos = new Vector3(0f, 0f, 0f);
            Vector2 rectSize = rect.GetRectSize();
            Vector2 pivot    = new Vector2(0.5f, 0.5f);
            if (applyPivot) pivot = rect.pivot;

            localPos.x = Mathf.Lerp(- rectSize.x * pivot.x, - rectSize.x * pivot.x + rectSize.x, normalizedPos.x);
            localPos.y = Mathf.Lerp(- rectSize.y * pivot.y, - rectSize.y * pivot.y + rectSize.y, normalizedPos.y);
            localPos.z = normalizedPos.z;
            return localPos;
        }

        internal static Vector3 LocalPosToNormalizedPos(this Vector3 localPos, RectTransform rect, bool applyPivot = true)
        {
            Vector3 normalizedPos = new Vector3(0f, 0f, 0f);
            Vector2 rectSize = rect.GetRectSize();
            Vector2 pivot    = new Vector2(0.5f, 0.5f);
            if (applyPivot) pivot = rect.pivot;

            normalizedPos.x = Mathf.InverseLerp(- rectSize.x * pivot.x, - rectSize.x * pivot.x + rectSize.x, localPos.x);
            normalizedPos.y = Mathf.InverseLerp(- rectSize.y * pivot.y, - rectSize.y * pivot.y + rectSize.y, localPos.y);
            normalizedPos.z = localPos.z;
            return normalizedPos;
        }

        internal static Vector3 GetProjectedPosition(this Vector3 position, Vector3 eulerRotation)
        {
            Quaternion quaternionRotation = Quaternion.Euler(-eulerRotation.x, -eulerRotation.y, eulerRotation.z);
            Vector3 projectedLocalPosition = quaternionRotation * new Vector3(-position.x, -position.y, position.z);

            return new Vector3(-projectedLocalPosition.x, -projectedLocalPosition.y, projectedLocalPosition.z);
        }

        private static bool CheckRangeOut(this RectTransform range, Vector3 point)
        {
            Vector2 rectSize = range.GetRectSize();
            bool isContained = true;
            float offset = 5f;
            if (Mathf.Abs(point.x + range.localPosition.x) > (rectSize.x / 2f + offset)) isContained = false;
            if (Mathf.Abs(point.y + range.localPosition.y) > (rectSize.y / 2f + offset)) isContained = false;

            return isContained;
        }

        private static Vector3 GetInterpolatedPoint(this RectTransform range, Vector3 startPoint, Vector3 endPoint)
        {
            Vector2 rectSize = range.GetRectSize();
            Vector3 newEndPoint = endPoint;
            int sign = 0; float t = 0f;

            // X 좌표 베이스로 interpolated point 계산
            if (Mathf.Abs(endPoint.x + range.localPosition.x) > rectSize.x / 2f) {
                sign = (endPoint.x + range.localPosition.x > 0) ? 1 : -1;
                t = Mathf.InverseLerp(startPoint.x + range.localPosition.x, endPoint.x + range.localPosition.x, sign * rectSize.x / 2f);
                newEndPoint = startPoint + t * (endPoint - startPoint);
            }
            
            // 계산된 point가 범위를 벗어났을 경우 y 좌표 베이스로 interpolated point 계산
            if (range.CheckRangeOut(newEndPoint)) return newEndPoint;

            if (Mathf.Abs(endPoint.y + range.localPosition.y) > rectSize.y / 2f) {
                sign = (endPoint.y + range.localPosition.y > 0) ? 1 : -1;
                t = Mathf.InverseLerp(startPoint.y + range.localPosition.y, endPoint.y + range.localPosition.y, sign * rectSize.y / 2f);
                newEndPoint = startPoint + t * (endPoint - startPoint);
            }

            return newEndPoint;
        }

        internal static void GetVerticesWithinRange(this List<Vector3> vertices, RectTransform range, out List<Vector3> intersectionPoints, out List<int> triangles)
        {
            intersectionPoints = new List<Vector3>();
            triangles = new List<int>();

            for (int i = 0; i < vertices.Count; i++) {
                int nextIndex = i + 1;
                if (nextIndex == vertices.Count) nextIndex = 0; 
                if (range.CheckRangeOut(vertices[i])) {
                    intersectionPoints.Add(vertices[i]);

                    if (!range.CheckRangeOut(vertices[nextIndex])) {
                        intersectionPoints.Add(range.GetInterpolatedPoint(vertices[i], vertices[nextIndex]));
                    }
                }
                else {
                    if (range.CheckRangeOut(vertices[nextIndex])) {
                        intersectionPoints.Add(range.GetInterpolatedPoint(vertices[nextIndex], vertices[i]));
                    }
                    else {
                        // start, end 포인트 모두 range out인데, 중간은 화면 내로 들어오는 경우
                        Vector3 newEndPoint = range.GetInterpolatedPoint(vertices[i], vertices[nextIndex]);
                        if (range.CheckRangeOut(newEndPoint)) {
                            intersectionPoints.Add(newEndPoint);
                            intersectionPoints.Add(range.GetInterpolatedPoint(newEndPoint, vertices[i]));
                        }
                    }
                }
            }

            if (intersectionPoints.Count < 3) {
                intersectionPoints.Clear();
                return;
            }

            int trianglesCount = intersectionPoints.Count - 2;
            for (int i = 0; i < trianglesCount; i++)
            {
                triangles.Add(0);
                triangles.Add(i + 1);
                triangles.Add(i + 2);
            }
        }

        internal static Gradient CopyGradient(this Gradient originalGradient)
        {
            if (originalGradient == null) return null;
            // Gradient newGradient = originalGradient
            // > newGradient는 originalGradient의 colorKeys, alphaKeys, mode를 참조
            // > 새로운 Gradient를 만들어서 복사하기 위해서는 아래 코드가 필요
            Gradient newGradient = new Gradient();

            // 색상 키와 알파 키 복사
            newGradient.colorKeys = originalGradient.colorKeys;
            newGradient.alphaKeys = originalGradient.alphaKeys;

            // 모드 복사
            newGradient.mode = originalGradient.mode;

            return newGradient;
        }
    }
}

