Shader "Custom/HeadLamp"
{
    Properties
    {
        // _MainTex ("Base Texture", 2D) = "white" {}
        // _HoleTex ("Hole Texture", 2D) = "black" {}
        _HolePosition ("Hole Position", Vector) = (0,0,0,0)
        _HoleSize ("Hole Size", Float) = 0.1
    }
    SubShader
    {
        Tags { "RenderType" = "Transparent" "Queue" = "Transparent"}
        Blend SrcAlpha OneMinusSrcAlpha
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata_t
            {
	            float4 vertex : POSITION;
	            float2 uv : TEXCOORD0;
            }; // Defines the input to the vertex shader, including the position and UV coordinates.

            struct v2f
            {
	            float4 pos : POSITION;
	            float2 uv : TEXCOORD0;
            }; // Defines the output from the vertex shader, including transformed position and UV coordinates.

            // sampler2D _MainTex;
            // sampler2D _HoleTex;
            float4 _HolePosition;
            float _HoleSize;

            v2f vert(appdata_t v)
            {
	            v2f o;
	            o.pos = UnityObjectToClipPos(v.vertex);
	            o.uv = v.uv;
	            return o;
            } // Transforms the vertex position from object space to clip space, pass to fragment shader

            half4 frag(v2f i) : SV_Target
            {
	            half4 col = (1, 1, 1, 1); // tex2D(_MainTex, i.uv);
	            float dist = distance(i.uv, _HolePosition.xy); // red channel
	            if (dist < _HoleSize)
	            {
		            col.r = 0.0;
		            col.a = 0.0; // Fully transparent where the hole texture is white
	            }
	            return col; 
            }
                ENDCG
            }
        }
    FallBack "Diffuse"
}
