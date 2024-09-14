//Based on the shader available at: http://wiki.unity3d.com/index.php/Silhouette-Outlined_Diffuse

Shader "Custom/SilhouetteShader"
{
	Properties
	{
		_OutlineColor("Outline Color", Color) = (0,0,0,1)
		_Outline("Outline Width", Range(0.000001, 0.001)) = 0.00001
	}

	CGINCLUDE
	#include "UnityCG.cginc"

	struct appdata
	{
		float4 vertex : POSITION;
		float3 normal : NORMAL;
	};

	struct v2f
	{
		float4 pos : POSITION;
		float4 color : COLOR;
	};

	uniform float4 _OutlineColor;
	uniform float4 _InsideColor;
	uniform float _Outline;
	
	v2f vert(appdata v)
	{
		// Scales vertex data according to normal direction
		v2f o;
		o.pos = UnityObjectToClipPos(v.vertex);

		float3 norm = mul((float3x3)UNITY_MATRIX_IT_MV, v.normal);
		float2 offset = TransformViewToProjection(norm.xy);

		o.pos.xy += offset * _Outline;
		o.color = _OutlineColor;
		return o;
	}

	ENDCG

	SubShader
	{
		Tags{ "Queue" = "Transparent" "RenderType" = "Transparent"  "IgnoreProjector" = "True" }
		
		// Brings the object to the front
		Pass
		{
			ColorMask 0
			ZTest Always
		}

		Pass 
		{
			Name "BASE"
			Cull Back
			Blend Zero One
 
			SetTexture [_OutlineColor] 
			{
				ConstantColor (0,0,0,0)
				Combine constant
			}
		}

		Pass
		{
			Name "OUTLINE"
			Cull Front

			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag

			half4 frag(v2f i) :COLOR
			{
				return i.color;
			}
			ENDCG
		}

		// This GrabPass Stores the inner details of the model
		GrabPass
		{
			"_BackgroundTexture"
		}

		// This pass renders the objects considering the expandend vertex creating the edges of the object.
		Pass
		{
			Name "OUTLINE"
			Cull Front
			
			ZTest Always

			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag

			half4 frag(v2f i) :COLOR
			{
				return i.color;
			}
			ENDCG
		}
		
		// This pass combines the inner details with the edges ob the object
		Pass
		{
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
			#include "UnityCG.cginc"

			struct v2f_Original
			{
				float4 grabPos : TEXCOORD0;
				float4 pos : SV_POSITION;
			};

			v2f_Original vert(appdata_base v) 
			{
				v2f_Original o;
				// Use UnityObjectToClipPos from UnityCG.cginc to calculate the clip-space of the vertex
				o.pos = UnityObjectToClipPos(v.vertex);
				// use ComputeGrabScreenPos function from UnityCG.cginc to get the correct texture coordinate
				o.grabPos = ComputeGrabScreenPos(o.pos);
				return o;
			}

			sampler2D _BackgroundTexture;

			half4 frag(v2f_Original i) : SV_Target
			{
				half4 bgcolor = tex2Dproj(_BackgroundTexture, i.grabPos);
				return bgcolor;
			}
			ENDCG
		}
	}
	//Fallback "Diffuse"
}