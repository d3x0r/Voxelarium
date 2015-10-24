#define MORE_ROUNDED
#if !USE_GLES2
using OpenTK.Graphics.OpenGL;
#else
using OpenTK.Graphics.ES20;
#endif
using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.UI.Shaders
{
	internal class VoxelGeometryShader : Shader
	{
		internal int vertex_attrib_id;
		internal int texture_attrib_id;
		internal int use_texture_id;
		internal int flat_color_id;
		internal int mod_attrib_id;
		internal int color_id;
		internal int face_color_id;
		internal int power_id;
		internal int texture_id;

		const string Vertex_Simple =
				@"precision mediump float;
			precision mediump int;
			uniform mat4 modelView;
			uniform mat4 worldView;
			uniform mat4 Projection;
			attribute vec4 vPosition;
			attribute vec2 in_Texture;
			attribute  vec4 in_Color;
			attribute  float in_Pow;
			attribute  int in_use_texture;
			attribute  int in_flat_color;
			attribute  vec2 in_Modulous;
			varying vec4 ex_Color;
			varying vec2 ex_Texture;
			varying  float ex_Pow;
			varying  int ex_use_texture;
			varying  int ex_flat_color;
			varying  vec2 ex_Modulous;
			void main(void) {
			//  gl_Position = Projection * worldView * modelView * vPosition;
			  gl_Position = Projection * worldView * vPosition;
			  ex_Texture = in_Texture;
			  ex_Color = in_Color;
			  ex_Pow = in_Pow;
			  ex_use_texture = in_use_texture;
			  ex_flat_color = in_flat_color;
			  ex_Modulous = in_Modulous;
			}"
			;

		const string Fragment_Simple =
			@"#version 130
			precision mediump float;
			precision mediump int;
			varying vec2 ex_Texture;
			varying vec4 ex_Color;
			varying  float ex_Pow;
			varying  int ex_use_texture;
			varying  int ex_flat_color;
			varying  vec2 ex_Modulous;
			uniform tex;
			void main(void) {
			  vec4 
			  if( ex_use_texture )
				{
					gl_FragColor = ex_Color * texture2D( tex, ex_texCoord );
				}
				else if( ex_flat_color )
				{
					gl_FragColor = ex_Color;
				}
				else
				{
			  float a = ex_Modulous.x - round(ex_Modulous.x );
			  float b = ex_Modulous.y - round(ex_Modulous.y );
			  float g;
			  float h;
			  vec3 white;
			  a = 4*(0.25-a*a);
			  b = 4*(0.25-b*b);
			  a = pow( a, ex_Pow );
			  b = pow( b, ex_Pow );"
#if !MORE_ROUNDED
			  +@"g = sqrt((a*a+b*b)/2);
			  h = pow(g,200.0) * 0.5;  // up to 600 even works...
			  g = pow( ( max(a,b)),400);
			  h = (g+h);
			  gl_FragColor = vec4( h * in_Color.rgb, in_Color.a ) ;"
#else
			+ @"//  g = pow( ( max(a,b)),in_Pow);
			//  h = pow( ( a*b),in_Pow/4);
			  g = min(1,b+a);
			  h = max((b+a)-1,0)/3;
			  white = vec3(1,1,1) * max(ex_Color.r,max(ex_Color.g,ex_Color.b));
			  gl_FragColor = vec4( h * white + (g * ex_Color.rgb), ex_Color.a ) ;
			//  gl_FragColor = vec4( g * ex_Color.rgb, ex_Color.a ) ;"
#endif
			 +@" } 
				}";


		internal override void Compile()
		{
			if( Compile( Vertex_Simple, Fragment_Simple ) )
			{
				vertex_attrib_id = GL.GetAttribLocation( Program, "vPosition" );
				texture_attrib_id = GL.GetAttribLocation( Program, "in_Texture" );
				use_texture_id = GL.GetAttribLocation( Program, "use_texture" );
				flat_color_id = GL.GetAttribLocation( Program, "flat_color" );
				mod_attrib_id = GL.GetAttribLocation( Program, "modulous" );

				color_id = GL.GetAttribLocation( Program, "in_Color" );
				face_color_id = GL.GetAttribLocation( Program, "in_FaceColor" );
				power_id = GL.GetUniformLocation( Program, "in_Pow" );
			}
		}

	}
}
