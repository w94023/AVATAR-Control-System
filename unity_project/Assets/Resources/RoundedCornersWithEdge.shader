Shader "UI/UnityUI/RoundedCornersWithEdge" {
    
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

        _Aspect ("Aspect", Float) = 1
        _Radius ("Radius", Range(0, 1)) = 0.5
        _EdgeSoftness ("Edge Softness", Float) = 0.05
        _EdgeThickness ("Edge Thickness", Range(0, 1)) = 0
        _EdgeTransitionSoftness ("Edge Transition Softness", Float) = 0.05
        _MainColor ("Main Color", Color) = (1, 1, 1, 1)
        _EdgeColor ("Edge Color", Color) = (1, 1, 1, 0)
    }
    
    SubShader
    {
        Tags { 
            "RenderType" = "Transparent"
            "Queue" = "Transparent" 
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
            
            #include "UnityCG.cginc"
            
            #pragma vertex vert
            #pragma fragment frag
            
            sampler2D _MainTex;
            float _Aspect;
            float _Radius;
            float _EdgeSoftness;
            float _EdgeTransitionSoftness;
            float _EdgeThickness;
            fixed4 _EdgeColor;
            fixed4 _MainColor;

            struct appdata {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
                float4 color : COLOR;  // set from Image component property
            };

            struct v2f {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
                float4 color : COLOR;
            };

            v2f vert (appdata v) {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                o.color = v.color;
                return o;
            }
            
            fixed4 frag (v2f i) : SV_Target
            {
                _Aspect = max(0.0, _Aspect);
                _EdgeSoftness = max(0.0, _EdgeSoftness);
                _EdgeTransitionSoftness = max(0.0, _EdgeTransitionSoftness);

                // 텍스처 샘플링
                fixed4 col = tex2D(_MainTex, i.uv) * i.color;

                // 모서리 둥글기 계산
                float2 uv = i.uv * 2 - 1; // UV 좌표를 -1에서 1 사이로 조정
                float2 fromCenter = abs(uv); // 중앙으로부터의 거리
                float2 fromCenterByAspect = fromCenter - 1; // 물체의 비율 반영
                if (_Aspect <= 1) {
                    fromCenterByAspect[1] *= 1/_Aspect; // _Aspect : 가로/세로 비율
                }
                else {
                    fromCenterByAspect[0] *= _Aspect; // _Aspect : 가로/세로 비율
                }
                float cornerDist = length(max(fromCenterByAspect + _Radius, 0)); // 모서리까지의 거리

                // 외곽선 효과
                float edgeStart = _Radius - _EdgeThickness;
                float alpha = 1.0 - smoothstep(edgeStart - _EdgeTransitionSoftness, edgeStart, cornerDist);
                fixed4 edgeCol = lerp(_EdgeColor * col.a, col, alpha);

                // 알파값을 부드럽게 조정하여 흐릿한 가장자리 생성
                float blurAlpha = 1.0 - smoothstep(_Radius - _EdgeSoftness, _Radius, cornerDist);
                edgeCol.a *= blurAlpha;

                return edgeCol;
            }
            
            ENDCG
        }
    }
}
