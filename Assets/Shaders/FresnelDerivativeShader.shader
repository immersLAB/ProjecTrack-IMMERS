Shader "Custom/FresnelDerivativeShader"
{
	Properties
	{
		[HDR] _AlbedoEdgeColor("Albedo Edge Color", Color) = (1,1,1,1)

		// Fresnel Shader Properties
		_FresnelEdgeWidth("Fresnel Edge Width", Range(0.1,100.0)) = 5.0

		// Derivative Shader Properties
		_DerivativeEdgeOffset("Derivative Edge Offset", Range(0.0,1.0)) = 0.15
		
		// Shader Mode Selection
		[Toggle(ADDITIVE_MODE_SELECTION)]
		_AdditiveModeSelection("Additive Mode Selection", Float) = 0

		// Intensity Multiplicative Factor
		_IntensityFactor("Intensity Factor Value", Range(1.0,100.0)) = 1.0
	}

	SubShader
	{
		Tags{ "Queue" = "Transparent" "IgnoreProjector" = "True" "RenderType" = "Transparent" }
		LOD 200

		// extra pass that renders to depth buffer only
		Pass
		{
			ZWrite On
			ColorMask 0
			ZTest Always

		}

		Pass
		{
			ZWrite On
			ColorMask 0
		}

		Pass
		{
			Cull Back
			ZWrite On

			Blend SrcAlpha OneMinusSrcAlpha

			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
			#pragma shader_feature ADDITIVE_MODE_SELECTION
			
			#include "UnityCG.cginc"


struct appdata
{
    float4 vertex : POSITION;
    float3 normal : TEXCOORD0;

    UNITY_VERTEX_INPUT_INSTANCE_ID //Insert
};

			struct v2f
			{
				float3 worldNormal : TEXCOORD0;
				float3 worldViewDirection : TEXCOORD1;
				float4 pos : SV_POSITION;
    UNITY_VERTEX_OUTPUT_STEREO
};

			float _FresnelEdgeWidth;
			float4 _AlbedoEdgeColor;
			float _DerivativeEdgeOffset;
			float _AdditiveModeSelection;
			float _IntensityFactor;

			// Vertex Shader
			v2f vert(appdata v)
			{
				v2f o;
	
	
    UNITY_SETUP_INSTANCE_ID(v);
    UNITY_INITIALIZE_OUTPUT(v2f, o);
    UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO();

	
				o.pos = UnityObjectToClipPos(v.vertex);
				o.worldNormal = UnityObjectToWorldNormal(v.normal);

				float3 worldPos = mul(unity_ObjectToWorld, v.vertex).xyz;
				// Compute world space view direction
				o.worldViewDirection = normalize(UnityWorldSpaceViewDir(worldPos));

				return o;
			}

			fixed4 frag(v2f i) : SV_Target
			{
				fixed4 c = 0;
				c = _AlbedoEdgeColor;

				// Fresnel Component
				float fresnelComponent = pow(clamp((1.0 - dot(i.worldNormal, i.worldViewDirection)), 0, 1), _FresnelEdgeWidth);

				// Derivative Component
				float vertical = clamp(ddx(i.worldNormal.x) + ddy(i.worldNormal.y), 0, 1);
				float horizontal = clamp(ddx(1 - i.worldNormal.x) + ddy(1 - i.worldNormal.y), 0, 1);
				float derivativeComponent = clamp(vertical + horizontal, 0, 1);

				[branch] if (derivativeComponent <= _DerivativeEdgeOffset)
				{
					derivativeComponent = 0;
				}
				
				// Shader Selection
				float alphaValue = 1;
				#ifdef ADDITIVE_MODE_SELECTION
					alphaValue = _IntensityFactor * (fresnelComponent + derivativeComponent);
				#else
					alphaValue = _IntensityFactor * (fresnelComponent * derivativeComponent);
				#endif
				
				c.a = alphaValue;

				return c;
			}
			ENDCG
		}
	}
}