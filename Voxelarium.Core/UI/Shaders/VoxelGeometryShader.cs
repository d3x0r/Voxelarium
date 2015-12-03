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
#define MORE_ROUNDED
#if !USE_GLES2
using OpenTK.Graphics.OpenGL;
#else
using OpenTK.Graphics.ES20;
#endif
using System;
using System.Collections.Generic;
using System.Diagnostics;
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
		internal int decal_texture_id; 

		const string Vertex_Simple =
			#if !USE_GLES2
				@"#version 130
			"+
			#endif
			@"uniform mat4 modelView;
			uniform mat4 worldView;
			uniform mat4 Projection;
			attribute vec4 vPosition;
			attribute vec2 in_Texture;
			attribute  vec4 in_Color;
			attribute  vec4 in_FaceColor;
			attribute  float in_Pow;"+
		#if USE_GLES2
		@"
			attribute  float in_use_texture;
			attribute  float in_flat_color;
			attribute  float in_decal_texture;"+
		#else
		@"
		attribute  int in_use_texture;
		attribute  int in_flat_color;
		attribute  int in_decal_texture;"+
		#endif
		
		@"	attribute  vec2 in_Modulous;
			varying vec4 ex_Color;
			varying vec2 ex_texCoord;
			varying  float ex_Pow;"+
#if USE_GLES2
			@"float ex_use_texture;
			float ex_flat_color;
			float ex_decal_texture;
			varying vec4 ex_FaceColor;"+
#else
			@"flat out  int ex_use_texture;
			flat out  int ex_flat_color;
			flat out  int ex_decal_texture;
			out vec4 ex_FaceColor;"+
#endif
			@"varying  vec2 ex_Modulous;
			void main(void) {
			//  gl_Position = Projection * worldView * modelView * vPosition;
				gl_Position = Projection * worldView * vPosition;"+
#if USE_GLES2
				@"ex_texCoord = in_Texture/32768.0f;
				ex_Color = in_Color/255.0f;
				ex_FaceColor = in_FaceColor/255.0f;"+
#else
				@"ex_texCoord = in_Texture/32768;
				ex_Color = in_Color/256;
				ex_FaceColor = in_FaceColor/256;" +
#endif
				@"ex_Pow = in_Pow;
				ex_use_texture = in_use_texture;
				ex_flat_color = in_flat_color;
				ex_Modulous = in_Modulous;
			}"
			;

		const string Fragment_Simple =
#if !USE_GLES2
			@"#version 130
			"+
#endif
			@"varying vec2 ex_texCoord;
			varying vec4 ex_Color;" +
#if USE_GLES2
		@"varying float ex_Pow;
			float ex_use_texture;
			float ex_flat_color;
			varying vec2 ex_Modulous;
			varying vec4 ex_FaceColor;
			uniform sampler2D tex;
			void main(void) {
			  if( ex_use_texture > 0.5f )
				{
					gl_FragColor = ex_Color * texture2D( tex, ex_texCoord );
				}
				else if( ex_flat_color > 0.5f )
				{
					gl_FragColor =vec4(1,0,1,1);// ex_Color;
				}
				else
				{
			" +
#else
			@"in float ex_Pow;
			flat in  int ex_use_texture;
			flat in  int ex_flat_color;
			in  vec2 ex_Modulous;
			in  vec4 ex_FaceColor;
		    uniform sampler2D tex;
		void main(void) {
		if( ex_use_texture != 0 )
		{
		gl_FragColor = ex_Color * texture2D( tex, ex_texCoord );
		}
		else if( ex_flat_color != 0 )
		{
		gl_FragColor =vec4(1,0,1,1);// ex_Color;
		}
		else
		{
		" +
#endif
#if USE_GLES2
				@"float a = mod(ex_Modulous.x +0.5f, 1.0f )-0.5f;
				float b = mod(ex_Modulous.y +0.5f, 1.0f )-0.5f;
				"+
#else
				@"float a = ex_Modulous.x - round(ex_Modulous.x );
				float b = ex_Modulous.y - round(ex_Modulous.y );
				"+
#endif
				@"float g;
				float h;
				vec3 white;
				a = 4.0f*(0.25f-a*a);
				b = 4.0f*(0.25f-b*b);
				a = pow( a, ex_Pow );
				b = pow( b, ex_Pow );
			"
#if !MORE_ROUNDED
			  +@"g = sqrt((a*a+b*b)/2);
			  h = pow(g,200.0) * 0.5;  // up to 600 even works...
			  g = pow( ( max(a,b)),400);
			  h = (g+h);
			  gl_FragColor = vec4( h * in_Color.rgb, in_Color.a ) ;"
#else
			+ @"//g = pow( ( max(a,b)),in_Pow);
				//h = pow( ( a*b),in_Pow/4);
				g = min(1.0f,b+a);
				h = max((b+a)-1.0f,0.0f)/3.0f;
				white = vec3(1.0f,1.0f,1.0f) * max(ex_Color.r,max(ex_Color.g,ex_Color.b));
			//	gl_FragColor = vec4( h * white + (g * ex_Color.rgb), ex_Color.a ) ;
			//  gl_FragColor = vec4( g * ex_Color.rgb, ex_Color.a ) ;
			    gl_FragColor = vec4( (1.0f-g)*ex_FaceColor.rgb + h* ( white - ex_FaceColor.rgb )+ (g* ex_Color.rgb), ex_Color.a ) ;
			"
#endif
			 + @" } 
				}";


		internal override void Compile()
		{
			if( Compile( Vertex_Simple, Fragment_Simple ) )
			{
				texture_id = GL.GetUniformLocation( Program, "tex" );
				vertex_attrib_id = GL.GetAttribLocation( Program, "vPosition" );
				texture_attrib_id = GL.GetAttribLocation( Program, "in_Texture" );
				use_texture_id = GL.GetAttribLocation( Program, "in_use_texture" );
				flat_color_id = GL.GetAttribLocation( Program, "in_flat_color" );
				mod_attrib_id = GL.GetAttribLocation( Program, "in_Modulous" );

				color_id = GL.GetAttribLocation( Program, "in_Color" );
				face_color_id = GL.GetAttribLocation( Program, "in_FaceColor" );
				power_id = GL.GetAttribLocation( Program, "in_Pow" );
				decal_texture_id = GL.GetAttribLocation( Program, "in_decal_texture" );
			}
			else
				Debug.Assert( false, "Shader didn't compile" );
		}

	}
}
