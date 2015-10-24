using System;
using System.Collections.Generic;
using System.Text;
#if !USE_GLES2
using OpenTK.Graphics.OpenGL;
#else
using OpenTK.Graphics.ES20;
#endif
using OpenTK;

namespace Voxelarium.Core.UI.Shaders
{
	internal class SimpleEdgeShader : Shader
	{
		int vertex_attrib_id;
		int texture_attrib_id;
		int color_id;
		int face_color_id;
		int power_id;

		const string Vertex_Simple =
				"precision mediump float;\n"
			 + "precision mediump int;\n"
			+ "uniform mat4 modelView;\n"
			+ "uniform mat4 worldView;\n"
			+ "uniform mat4 Projection;\n"
			+ "attribute vec4 vPosition;\n"
			+ "attribute vec2 in_Texture;\n"
			//+ "uniform  vec4 in_Color;\n"
			//+ "varying vec4 ex_Color;\n"
			+ "varying vec2 ex_Texture;\n"
			+ "void main(void) {"
			+ "  ex_Texture = in_Texture;\n"
			//+ "  gl_Position = Projection * worldView * modelView * vPosition;"
			+ "  gl_Position = Projection * worldView * vPosition;"
			//+ "  ex_Color = in_Color;"
			+ "}"
			;

		const string Fragment_Simple =
			"#version 130\n" 
			+"precision mediump float;\n"
			+ "precision mediump int;\n"
			+ "uniform  vec4 in_Color;\n"
			+ "uniform  vec4 in_FaceColor;\n"
			+ "uniform  float in_Pow;\n"
			+ "varying vec2 ex_Texture;"
			+ "varying vec4 ex_Color;"
			+ "void main(void) {"
			+ "  float a = ex_Texture.x - round(ex_Texture.x );\n"
			+ "  float b = ex_Texture.y - round(ex_Texture.y );\n"
			+ "  float g;\n"
			+ "  float h;\n"
			+ "  vec3 white;\n"
			+ "  a = 4*(0.25-a*a);\n"
			+ "  b = 4*(0.25-b*b);\n"
			+ "  a = pow( a, in_Pow );\n"
			+ "  b = pow( b, in_Pow );\n"
#if MORE_ROUNDED
			+ "  g = sqrt((a*a+b*b)/2);\n"
			+ "  h = pow(g,200.0) * 0.5;\n"  // up to 600 even works...
			+ "  g = pow( ( max(a,b)),400);\n"
			+ "  h = (g+h);"
			+ "  gl_FragColor = vec4( h * in_Color.rgb, in_Color.a ) ;"
#else
			//+ "  g = pow( ( max(a,b)),in_Pow);\n"
			+ "  g = min(1,b+a);\n"
			+ "  h = max((b+a)-1,0)/3;\n"
			//+ "  gl_FragColor = vec4( a,b,0,1) ;"
			+ "  white = vec3(1,1,1) * max(in_Color.r,max(in_Color.g,in_Color.b));\n"
			+ "  gl_FragColor = vec4( (1-g)*in_FaceColor.rgb + h * ( white - in_FaceColor.rgb )+ (g * in_Color.rgb), in_Color.a ) ;"
#endif
			+ "}";


		internal override void Compile()
		{
			if( Compile( Vertex_Simple, Fragment_Simple ) )
			{
				vertex_attrib_id = GL.GetAttribLocation( Program, "vPosition" );
				texture_attrib_id = GL.GetAttribLocation( Program, "in_Texture" );
				color_id = GL.GetUniformLocation( Program, "in_Color" );
				face_color_id = GL.GetUniformLocation( Program, "in_FaceColor" );
				power_id = GL.GetUniformLocation( Program, "in_Pow" );
			}
		}

		internal void DrawQuad( Vector3[] verts, Vector2[] texture, ref Vector4 face_color, ref Vector4 color , float power = 400.0f )	
		{
			GL.Uniform1( power_id, power );
			Display.CheckErr();
			GL.Uniform4( face_color_id, ref face_color );
			Display.CheckErr();
			GL.Uniform4( color_id, ref color );
			Display.CheckErr();
			GL.EnableVertexAttribArray( vertex_attrib_id );
			GL.EnableVertexAttribArray( texture_attrib_id );
			Display.CheckErr();
			GL.VertexAttribPointer( vertex_attrib_id, 3, VertexAttribPointerType.Float, false, 0, verts );
			Display.CheckErr();
			GL.VertexAttribPointer( texture_attrib_id, 2, VertexAttribPointerType.Float, false, 0, texture );
			Display.CheckErr();
			GL.DrawArrays( PrimitiveType.TriangleStrip, 0, 4 );
			Display.CheckErr();
		}

	}

	internal class ColorEdgeShader : Shader
	{
		int vertex_attrib_id;
		int texture_attrib_id;
		int color_id;
		int power_id;

		const string Vertex_Simple =
				"precision mediump float;\n"
			 + "precision mediump int;\n"
			+ "uniform mat4 modelView;\n"
			+ "uniform mat4 worldView;\n"
			+ "uniform mat4 Projection;\n"
			+ "attribute vec4 vPosition;\n"
			+ "attribute vec2 in_Texture;\n"
			+ "attribute  vec4 in_Color;\n"
			+ "varying vec4 ex_Color;\n"
			+ "varying vec2 ex_Texture;\n"
			+ "void main(void) {"
			+ "  ex_Texture = in_Texture;\n"
			//+ "  gl_Position = Projection * worldView * modelView * vPosition;"
			+ "  gl_Position = Projection * worldView * vPosition;"
			+ "  ex_Color = in_Color;"
			+ "}"
			;

		const string Fragment_Simple =
			"#version 130\n"
			+ "precision mediump float;\n"
			+ "precision mediump int;\n"
			+ "uniform  float in_Pow;\n"
			+ "varying vec2 ex_Texture;"
			+ "varying vec4 ex_Color;"
			+ "void main(void) {"
			+ "  float a = ex_Texture.x - round(ex_Texture.x );\n"
			+ "  float b = ex_Texture.y - round(ex_Texture.y );\n"
			+ "  float g;\n"
			+ "  float h;\n"
			+ "  vec3 white;\n"
			+ "  a = 4*(0.25-a*a);\n"
			+ "  b = 4*(0.25-b*b);\n"
			+ "  a = pow( a, in_Pow );\n"
			+ "  b = pow( b, in_Pow );\n"
#if MORE_ROUNDED
			+ "  g = sqrt((a*a+b*b)/2);\n"
			+ "  h = pow(g,200.0) * 0.5;\n"  // up to 600 even works...
			+ "  g = pow( ( max(a,b)),400);\n"
			+ "  h = (g+h);"
			+ "  gl_FragColor = vec4( h * in_Color.rgb, in_Color.a ) ;"
#else
			//+ "  g = pow( ( max(a,b)),in_Pow);\n"
			//+ "  h = pow( ( a*b),in_Pow/4);\n"
			+ "  g = min(1,b+a);\n"
			+ "  h = max((b+a)-1,0)/3;\n"
			+ "  white = vec3(1,1,1) * max(ex_Color.r,max(ex_Color.g,ex_Color.b));\n"
			+ "  gl_FragColor = vec4( h * white + (g * ex_Color.rgb), ex_Color.a ) ;"
			//+ "  gl_FragColor = vec4( g * ex_Color.rgb, ex_Color.a ) ;"
#endif
			+ "}";


		internal override void Compile()
		{
			if( Compile( Vertex_Simple, Fragment_Simple ) )
			{
				vertex_attrib_id = GL.GetAttribLocation( Program, "vPosition" );
				texture_attrib_id = GL.GetAttribLocation( Program, "in_Texture" );
				color_id = GL.GetAttribLocation( Program, "in_Color" );
				power_id = GL.GetUniformLocation( Program, "in_Pow" );
			}
		}

		internal void DrawQuad( Vector3[] verts, Vector2[] texture, ref Vector4 face_color, Vector4[] colors, float power = 400.0f )
		{
			GL.Uniform1( power_id, power );
			Display.CheckErr();
			GL.EnableVertexAttribArray( vertex_attrib_id );
			GL.EnableVertexAttribArray( texture_attrib_id );
			GL.EnableVertexAttribArray( color_id );
			Display.CheckErr();
			GL.VertexAttribPointer( vertex_attrib_id, 3, VertexAttribPointerType.Float, false, 0, verts );
			Display.CheckErr();
			GL.VertexAttribPointer( texture_attrib_id, 2, VertexAttribPointerType.Float, false, 0, texture );
			Display.CheckErr();
			GL.VertexAttribPointer( color_id, 4, VertexAttribPointerType.Float, false, 0, colors );
			Display.CheckErr();
			GL.DrawArrays( PrimitiveType.TriangleStrip, 0, 4 );
			Display.CheckErr();
		}

	}


}
