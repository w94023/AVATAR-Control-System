using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UnityUI
{
    public static class ColorTemplate
    {
        public enum ColorType {
            Background,
            Point,
        }
        public static List<Color> backgroundColors = new List<Color>() {
            new Color(32f/255f, 33f/255f, 42f/255f, 255f/255f),
        };
        public static List<Color> pointColors = new List<Color>() {
            new Color(30f/255f, 215f/255f, 171f/255f, 255f/255f),
            new Color(2f/255f, 167f/255f, 248f/255f, 255f/255f),
            new Color(159f/255f, 143f/255f, 198f/255f, 255f/255f),
        };

        public static List<Color> GetColor(ColorType colorType)
        {
            if (colorType == ColorType.Background)
                return backgroundColors;
            if (colorType == ColorType.Point)
                return pointColors;
            
            return null;
        }

        public static List<Gradient> GetGradient(ColorType colorType)
        {
            List<Gradient> gradient = new List<Gradient>();
            if (colorType == ColorType.Background) {
                for (int i = 0; i < backgroundColors.Count; i++) {
                    Gradient _gradient = new Gradient();
                    _gradient.SetKeys(
                        new GradientColorKey[] { new GradientColorKey(backgroundColors[i], 0.0f),  new GradientColorKey(backgroundColors[i], 1.0f) },
                        new GradientAlphaKey[] { new GradientAlphaKey(1f, 0f),                     new GradientAlphaKey(1f, 1.0f)                  }
                    );
                    gradient.Add(_gradient);
                }  
            }
            if (colorType == ColorType.Point) {
                for (int i = 0; i < pointColors.Count; i++) {
                    Gradient _gradient = new Gradient();
                    _gradient.SetKeys(
                        new GradientColorKey[] { new GradientColorKey(pointColors[i], 0.0f),  new GradientColorKey(pointColors[i], 1.0f) },
                        new GradientAlphaKey[] { new GradientAlphaKey(1f, 0f),                new GradientAlphaKey(1f, 1.0f)             }
                    );
                    gradient.Add(_gradient);
                }  
            }
            
            return gradient;
        }
    }
}
