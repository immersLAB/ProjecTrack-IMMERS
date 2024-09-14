Shader"Unlit/InConeShader"
{
    Properties
    {
        _Color ("Color", Color) = (1,1,1,1)
        _ConeVertex ("Cone Vertex", Vector) = (0,0,0,0)
        _ConeAxis ("Cone Axis", Vector) = (0,1,0,0)
        _ConeAngle ("Cone Angle", Float) = 45.0 // Apex angle in degrees
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

            float4 _ConeVertex;
            float4 _ConeAxis;
            float _ConeAngle;
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
                float3 coneVertex = _ConeVertex.xyz;
                float3 coneAxis = normalize(_ConeAxis.xyz);
                float coneAngle = radians(_ConeAngle);

                            // Calculate the vector from the cone vertex to the fragment position
                float3 toPoint = i.worldPos - coneVertex;

                            // Project the vector onto the cone axis
                float3 projected = dot(toPoint, coneAxis) * coneAxis;

                            // Calculate the angle between the projected vector and the vector to the point
                float3 toPointProjected = normalize(toPoint - projected);
                float angle = acos(dot(toPoint, coneAxis) / length(toPoint));
                
                            // Check if the angle is within the cone's apex angle
                bool insideCone = angle < coneAngle;

                if (insideCone)
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
