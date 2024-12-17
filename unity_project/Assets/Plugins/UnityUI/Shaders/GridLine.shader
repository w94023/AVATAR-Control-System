Shader "UI/UnityUI/GridLine"
{
    Properties
    {
        [HideInInspector] _MainTex ("Texture", 2D) = "white" {}
        [HideInInspector] _BackColor ("Color", Color) = (0, 0, 0, 0)
        _Thickness("Thickness", Range(0, 1)) = 0.1
        _Amount("Amount", int) = 10
        _Ratio("Ratio", Range(0, 1)) = 1

        // --- Mask support ---
        [HideInInspector] _StencilComp ("Stencil Comparison", Float) = 8
        [HideInInspector] _Stencil ("Stencil ID", Float) = 0
        [HideInInspector] _StencilOp ("Stencil Operation", Float) = 0
        [HideInInspector] _StencilWriteMask ("Stencil Write Mask", Float) = 255
        [HideInInspector] _StencilReadMask ("Stencil Read Mask", Float) = 255
        [HideInInspector] _ColorMask ("Color Mask", Float) = 15
        [HideInInspector] _UseUIAlphaClip ("Use Alpha Clip", Float) = 0
    }
    SubShader
    {
        Tags 
        {
            "Queue"="Transparent"
            "RenderType"="Transparent"
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

            struct appdata
            {
                float4 vertex : POSITION;
                float4 color : COLOR;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float4 vertex : SV_POSITION;
                float4 color : COLOR;
                float2 uv : TEXCOORD0;
            };

            sampler2D _MainTex;
            float4 _MainTex_ST;
            float4 _BackColor;
            float _Radius;
            float _Thickness;
            int _Amount;
            float _Ratio;

            v2f vert(appdata IN)
            {
                v2f OUT;
                OUT.vertex = UnityObjectToClipPos(IN.vertex);
                OUT.color = IN.color;
                OUT.uv = IN.uv;
                return OUT;
            }

            float4 frag(v2f i) : SV_Target
            {
                if (_Amount < 0) { _Amount = 0; }
                fixed4 c = tex2D(_MainTex, i.uv)*i.color;
                float pivot = 0; // Bottom of image
                float dist = i.uv.y - pivot;
                float span = ((1 - _Thickness/2) - (0 + _Thickness/2))/(_Amount-1);
                bool draw = false;

                float alpha = 0;
                for (int i = 0; i < _Amount; i++) {
                    float ratio = span*i;
                    float centerOfTick = _Thickness/2 + ratio;
                    if (ratio < _Ratio) {
                        if (_Amount <= 1) { centerOfTick = 0.5; }
                        // antialiasing
                        float smoothness = 0.005;
                        if (dist <= centerOfTick - _Thickness/2 && dist > centerOfTick - _Thickness/2 - smoothness) {
                            alpha = smoothstep(centerOfTick - _Thickness/2 - smoothness, centerOfTick - _Thickness/2, dist);
                        }
                        else if (dist <= centerOfTick + _Thickness/2 && dist > centerOfTick - _Thickness/2) {
                            alpha = 1;
                        }
                        else if (dist <= centerOfTick + _Thickness/2 + smoothness && dist > centerOfTick + _Thickness/2){
                            alpha = 1 - smoothstep(centerOfTick + _Thickness/2, centerOfTick + _Thickness/2 + smoothness, dist);
                        }
                    }   
                }
                // float a = draw ? 1 : 0;
                fixed4 col = lerp(_BackColor, c, alpha);

                return col;
            }
            ENDCG
        }
    }
}
