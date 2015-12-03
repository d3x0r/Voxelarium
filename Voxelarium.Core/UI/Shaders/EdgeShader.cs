/*
 * This file is part of Voxelarium.
 *
 * Copyright 2015-2016 James Buckeyne  *** Added 11/22/2015
 *
 * Voxelarium is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Voxelarium is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
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
				"uniform mat4 modelView;\n"
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
#if !USE_GLES2
			"#version 130\n" 
			+ 
#endif
			   "uniform  vec4 in_Color;\n"
			+ "uniform  vec4 in_FaceColor;\n"
			+ "uniform  float in_Pow;\n"
			+ "varying vec2 ex_Texture;"
			+ "varying vec4 ex_Color;"
			+ "void main(void) {"
			+
#if USE_GLES2
			@"float a = mod(ex_Texture.x +0.5f, 1.0f )-0.5f;
			  float b = mod(ex_Texture.y +0.5f, 1.0f )-0.5f;" +
#else
			@"float a = ex_Texture.x - round(ex_Texture.x );
			  float b = ex_Texture.y - round(ex_Texture.y );" +
			#endif
			 @"  float g;
				  float h;
 				vec3 white;
				a = 4.0f*(0.25-a*a);
				b = 4.0f*(0.25-b*b);
				a = pow( a, in_Pow );
				b = pow( b, in_Pow );" +
#if MORE_ROUNDED
			+ "  g = sqrt((a*a+b*b)/2);\n"
			+ "  h = pow(g,200.0) * 0.5;\n"  // up to 600 even works...
			+ "  g = pow( ( max(a,b)),400);\n"
			+ "  h = (g+h);"
			+ "  gl_FragColor = vec4( h * in_Color.rgb, in_Color.a ) ;"
#else
			//+ "  g = pow( ( max(a,b)),in_Pow);\n"
			@"  g = min(1.0f,b+a);
		       h = max((b+a)-1.0f,0.0f)/3.0f;
			//  gl_FragColor = vec4( a,b,0,1) ;
			  white = vec3(1,1,1) * max(in_Color.r,max(in_Color.g,in_Color.b));
			  gl_FragColor = vec4( (1.0f-g)*in_FaceColor.rgb + h * ( white - in_FaceColor.rgb )+ (g * in_Color.rgb), in_Color.a ) ;"
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
			Display.CheckErr();
			GL.EnableVertexAttribArray( texture_attrib_id );
			Display.CheckErr();
			GL.VertexAttribPointer( vertex_attrib_id, 3, VertexAttribPointerType.Float, false, 0, verts );
			Display.CheckErr();
			GL.VertexAttribPointer( texture_attrib_id, 2, VertexAttribPointerType.Float, false, 0, texture );
			Display.CheckErr();
			GL.DrawArrays( BeginMode.TriangleStrip, 0, 4 );
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
				"uniform mat4 modelView;\n"
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
#if !USE_GLES2
			"#version 130\n"
			+ 
#endif
			  @"uniform  float in_Pow;
			    varying vec2 ex_Texture;
			    varying vec4 ex_Color;
			    void main(void) {" +
#if USE_GLES2
			@"float a = mod(ex_Texture.x +0.5f, 1.0f )-0.5f;
			  float b = mod(ex_Texture.y +0.5f, 1.0f )-0.5f;" +
#else
			@"float a = ex_Texture.x - round(ex_Texture.x );
			  float b = ex_Texture.y - round(ex_Texture.y );" +
			#endif
			  @"float g;
			  float h;
			  vec3 white;
			  a = 4.0f*(0.25-a*a);
			  b = 4.0f*(0.25-b*b);
			  a = pow( a, in_Pow );
			  b = pow( b, in_Pow );"+
#if MORE_ROUNDED
			   @" g = sqrt((a*a+b*b)/2);
			    h = pow(g,200.0) * 0.5;  // up to 600 even works...
			    g = pow( ( max(a,b)),400);
			    h = (g+h);
			    gl_FragColor = vec4( h * in_Color.rgb, in_Color.a ) ;"
#else
			//+ "  g = pow( ( max(a,b)),in_Pow);\n"
			//+ "  h = pow( ( a*b),in_Pow/4);\n"
			  @" g = min(1.0f,b+a);
			   h = max((b+a)-1.0f,0.0f)/3.0f;
			   white = vec3(1.0f,1.0f,1.0f) * max(ex_Color.r,max(ex_Color.g,ex_Color.b));
			   gl_FragColor = vec4( h * white + (g * ex_Color.rgb), ex_Color.a ) ;"
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
			Display.CheckErr();
			GL.EnableVertexAttribArray( texture_attrib_id );
			Display.CheckErr();
			GL.EnableVertexAttribArray( color_id );
			Display.CheckErr();
			GL.VertexAttribPointer( vertex_attrib_id, 3, VertexAttribPointerType.Float, false, 0, verts );
			Display.CheckErr();
			GL.VertexAttribPointer( texture_attrib_id, 2, VertexAttribPointerType.Float, false, 0, texture );
			Display.CheckErr();
			GL.VertexAttribPointer( color_id, 4, VertexAttribPointerType.Float, false, 0, colors );
			Display.CheckErr();
#if !USE_GLES2
			GL.DrawArrays( PrimitiveType.TriangleStrip, 0, 4 );
#else
			GL.DrawArrays( BeginMode.TriangleStrip, 0, 4 );
#endif
			Display.CheckErr();
			GL.DisableVertexAttribArray( vertex_attrib_id );
			Display.CheckErr();
			GL.DisableVertexAttribArray( texture_attrib_id );
			Display.CheckErr();
			GL.DisableVertexAttribArray( color_id );
			Display.CheckErr();
		}

	}


}
