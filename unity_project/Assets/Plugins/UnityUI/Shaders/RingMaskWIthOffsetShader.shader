Shader "UI/CircleGraph/RingMaskWithOffset"
{
    Properties
    {
        [HideInInspector] _MainTex ("Texture", 2D) = "white" {}
        // --- Mask support ---
        [HideInInspector] _StencilComp ("Stencil Comparison", Float) = 8
        [HideInInspector] _Stencil ("Stencil ID", Float) = 0
        [HideInInspector] _StencilOp ("Stencil Operation", Float) = 0
        [HideInInspector] _StencilWriteMask ("Stencil Write Mask", Float) = 255
        [HideInInspector] _StencilReadMask ("Stencil Read Mask", Float) = 255
        [HideInInspector] _ColorMask ("Color Mask", Float) = 15
        [HideInInspector] _UseUIAlphaClip ("Use Alpha Clip", Float) = 0

        [HideInInspector] _BackColor ("Back Color", Color) = (0,0,0,0)
        _FracStart ("Start point of circle", Range(0,1)) = 0.0
        _FracEnd ("End point of circle", Range(0,1)) = 1.0
        _Thickness("Thickness", float) = 0.1
        _Hardness("Harndess", float) = 2
    }
    SubShader
    {
        Tags
        {
            "RenderType"="Transparent"
            "Queue"="Transparent"
        }
        // --- Mask support ---
        Stencil
        {
            Ref [_Stencil]
            Comp [_StencilComp]
            Pass [_StencilOp]
            ReadMask [_StencilReadMask]
            WriteMask [_StencilWriteMask]
        }
        Cull Off
        Lighting Off
        ZTest [unity_GUIZTestMode]
        ColorMask [_ColorMask]
        // ---
        
        Blend SrcAlpha OneMinusSrcAlpha
        ZWrite Off
 
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
             
            #include "UnityCG.cginc"
            // #include "ShaderSetup.cginc"
 
            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
                float4 color : COLOR;
            };
 
            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
                float4 color : COLOR;
            };
 
            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                o.color = v.color;
                return o;
            }
             
            sampler2D _MainTex;
            half _FracStart;
            half _FracEnd;
            fixed4 _BackColor;
            float _Radius;
            float _Hardness;
            float _CenterX;
            float _CenterY;
            float _SizeX;
            float _SizeY;
            float _Thickness;
 
            fixed4 frag (v2f i) : SV_Target
            {
                // Receive color profile from image
                fixed4 c = tex2D(_MainTex, i.uv)*i.color;
                // Set default value
                _CenterX = 0.5; _CenterY = 0.5;
                _SizeX = 1.0+_Thickness; _SizeY = 1.0+_Thickness;
                _Radius = 0.5;
                // Calculate ring alpha map
                float dist = length(float2(i.uv.x - _CenterX, i.uv.y - _CenterY) * float2(_SizeX, _SizeY));
                float rd = _Thickness/2;
                float rc = _Radius - rd;
                float circle = saturate(abs(dist - rc) / _Thickness);
                float circleAlpha = pow(circle, pow(_Hardness, 2));
                float a = 1 - circleAlpha;
                // Apply alpha map
                fixed4 colTemp1 = lerp(_BackColor, c, a);

                half angle = atan2(i.uv.x-_CenterX, i.uv.y-_CenterY);
                // rescale -π to +π range to 0.0 to 1.0
                half gradient = angle / (UNITY_PI * 2.0)+0.5;
                // screen space derivatives scaled to 1.5 for the smoothstep
                half gradientDeriv = fwidth(gradient) * 1.5;
                // smoothstep for a smooth but sharp edge on the progress bar
                half barProgress1 = smoothstep(_FracEnd, _FracEnd + gradientDeriv, gradient);
                half barProgress2 = 1-smoothstep(_FracStart, _FracStart + gradientDeriv, gradient);
                // lerp between colors
                fixed4 colTemp2 = lerp(colTemp1, _BackColor, barProgress1);
                fixed4 col = lerp(colTemp2, _BackColor, barProgress2);
                
                return col;
            }
            ENDCG
        }
    }
}