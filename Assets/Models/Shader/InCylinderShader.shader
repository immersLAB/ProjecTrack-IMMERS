Shader "Unlit/InCylinderShader"
{
    Properties
    {
        _Color ("Color", Color) = (1,1,1,1)
        _CylinderBase ("Cylinder Base", Vector) = (0,0,0,0)
        _CylinderHeight ("Cylinder Height", Float) = 2.0
        _CylinderRadius ("Cylinder Radius", Float) = 0.5
    }
    SubShader
    {
        Tags { "RenderType"="Transparent" }
        LOD 200

        Pass
        {
            Blend SrcAlpha OneMinusSrcAlpha
            ZWrite On

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
            };

            struct v2f
            {
                float4 pos : POSITION;
                float3 worldPos : TEXCOORD0;
            };

            float4 _CylinderBase;
            float _CylinderHeight;
            float _CylinderRadius;
            float4 _Color;

            v2f vert(appdata v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.worldPos = mul(unity_ObjectToWorld, v.vertex).xyz;
                return o;
            }

            fixed4 frag(v2f i) : SV_Target
            {
                            // Calculate the distance from the point to the cylinder's central axis
                float3 relativePos = i.worldPos - _CylinderBase.xyz;
                float distToAxis = length(float2(relativePos.x, relativePos.z));

                            // Check if within cylinder radius and height
                bool insideCylinder = (distToAxis < _CylinderRadius) &&
                                                  (relativePos.y > 0.0) &&
                                                  (relativePos.y < _CylinderHeight);

                if (insideCylinder)
                {
                    return _Color;
                }
                else
                {
                    return float4(0, 0, 0, 0);
                }
            }
            ENDCG
        }
    }
FallBack"Transparent/VertexLit"
}