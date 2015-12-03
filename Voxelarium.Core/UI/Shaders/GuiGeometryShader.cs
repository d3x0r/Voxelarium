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
 *
 * Created : 2015/12/02 James Buckeyne
 *
*/
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
	internal class GuiGeometryShader : Shader
	{
		internal struct GuiGeometryVertex {
			internal const int VertexSize = 4 * 4 + 4 * 4; /* 32 */

			/* 0 offset */
			internal byte bar1, bar2, bar3;
			/* 3 offset */
			internal byte shade_method;  // 0 = straight copy
										 //, 0.25/*64*/ = mono shade, 
										 // 0.5/*127*/ = tri-shade 
										 // 0.75/*196*/ = portion_tri_shade*/;
			/* 4 offset */
			internal byte cb1r, cb1g, cb1b;
			/* 7 offset */
			internal byte translation; /* 0 = no transform, use literal vertex
								, 0.25 [64] = projection&world
								, 0.5 [128] = projection&world&instance */
			/* 8 offset */
			internal byte cb2r, cb2g, cb2b;
			/* 11 offset */  
			internal byte texture_select;  // 0 = font, 0.5 [128] = gui resource
			/* 12 offset */
			internal byte cb3r, cb3g, cb4g, unused3;

			/* 16 offset */
			internal float vx, vy, vz;
			/* 28 offset */
			internal ushort u, v;
		}

		internal class GuiGeometry : GeometryBuffer<GuiGeometryVertex>
		{
			static GuiGeometryShader GuiShader;
			internal GuiGeometry( int InitialCapacity ) : base( InitialCapacity )
			{
			}
			internal GuiGeometry() : this( 100 )
			{
			}

			internal static void DoSetupBuffer( IntPtr vertbuf )
			{
				GL.VertexAttribPointer( GuiShader.vertex_attrib_id, 3
					, VertexAttribPointerType.Float, false, GuiGeometryVertex.VertexSize, vertbuf + 16 );
				Display.CheckErr();
				GL.EnableVertexAttribArray( GuiShader.vertex_attrib_id );
				Display.CheckErr();

				if( GuiShader.texture_attrib_id >= 0 )
				{
					GL.EnableVertexAttribArray( GuiShader.texture_attrib_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GuiShader.texture_attrib_id, 2
						, VertexAttribPointerType.UnsignedShort, false, GuiGeometryVertex.VertexSize, vertbuf + 28 );
					Display.CheckErr();
				}
				if( GuiShader.color_translation_id >= 0 )
				{
					GL.EnableVertexAttribArray( GuiShader.color_translation_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GuiShader.color_translation_id, 1
						, VertexAttribPointerType.Byte, false, GuiGeometryVertex.VertexSize, vertbuf + 3 );
					Display.CheckErr();
				}
				if( GuiShader.vertex_translation_id >= 0 )
				{
					GL.EnableVertexAttribArray( GuiShader.vertex_translation_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GuiShader.vertex_translation_id, 1
						, VertexAttribPointerType.Byte, false, GuiGeometryVertex.VertexSize, vertbuf + 7 );
					Display.CheckErr();
				}
				if( GuiShader.texture_select_id >= 0 )
				{
					GL.EnableVertexAttribArray( GuiShader.texture_select_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GuiShader.texture_select_id, 1
						, VertexAttribPointerType.Byte, false, GuiGeometryVertex.VertexSize, vertbuf + 11 );
					Display.CheckErr();
				}
				if( GuiShader.color1_id >= 0 )
				{
					GL.EnableVertexAttribArray( GuiShader.color1_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GuiShader.color1_id, 3
						, VertexAttribPointerType.Byte, false, GuiGeometryVertex.VertexSize, vertbuf + 4 );
					Display.CheckErr();
				}
				if( GuiShader.color2_id >= 0 )
				{
					GL.EnableVertexAttribArray( GuiShader.color2_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GuiShader.color2_id, 3
						, VertexAttribPointerType.Byte, false, GuiGeometryVertex.VertexSize, vertbuf + 8 );
					Display.CheckErr();
				}
				if( GuiShader.color3_id >= 0 )
				{
					GL.EnableVertexAttribArray( GuiShader.color3_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GuiShader.color3_id, 3
						, VertexAttribPointerType.Byte, false, GuiGeometryVertex.VertexSize, vertbuf + 12 );
					Display.CheckErr();
				}
			}

			protected override void SetupBuffer()
			{
				GL.VertexAttribPointer( GuiShader.vertex_attrib_id, 3
					, VertexAttribPointerType.Float, false, GuiGeometryVertex.VertexSize, 16 );
				Display.CheckErr();
				GL.EnableVertexAttribArray( GuiShader.vertex_attrib_id );
				Display.CheckErr();

				if( GuiShader.texture_attrib_id >= 0 )
				{
					GL.EnableVertexAttribArray( GuiShader.texture_attrib_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GuiShader.texture_attrib_id, 2
						, VertexAttribPointerType.UnsignedShort, false, GuiGeometryVertex.VertexSize, 28 );
					Display.CheckErr();
				}
				if( GuiShader.color_translation_id >= 0 )
				{
					GL.EnableVertexAttribArray( GuiShader.color_translation_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GuiShader.color_translation_id, 1
						, VertexAttribPointerType.Byte, false, GuiGeometryVertex.VertexSize, 3 );
					Display.CheckErr();
				}
				if( GuiShader.vertex_translation_id >= 0 )
				{
					GL.EnableVertexAttribArray( GuiShader.vertex_translation_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GuiShader.vertex_translation_id, 1
						, VertexAttribPointerType.Byte, false, GuiGeometryVertex.VertexSize, 7 );
					Display.CheckErr();
				}
				if( GuiShader.texture_select_id >= 0 )
				{
					GL.EnableVertexAttribArray( GuiShader.texture_select_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GuiShader.texture_select_id, 1
						, VertexAttribPointerType.Byte, false, GuiGeometryVertex.VertexSize, 11 );
					Display.CheckErr();
				}
				if( GuiShader.color1_id >= 0 )
				{
					GL.EnableVertexAttribArray( GuiShader.color1_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GuiShader.color1_id, 3
						, VertexAttribPointerType.Byte, false, GuiGeometryVertex.VertexSize, 4 );
					Display.CheckErr();
				}
				if( GuiShader.color2_id >= 0 )
				{
					GL.EnableVertexAttribArray( GuiShader.color2_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GuiShader.color2_id, 3
						, VertexAttribPointerType.Byte, false, GuiGeometryVertex.VertexSize, 8 );
					Display.CheckErr();
				}
				if( GuiShader.color3_id >= 0 )
				{
					GL.EnableVertexAttribArray( GuiShader.color3_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GuiShader.color3_id, 3
						, VertexAttribPointerType.Byte, false, GuiGeometryVertex.VertexSize, 12 );
					Display.CheckErr();
				}
			}

			protected override void FillBuffer()
			{
				unsafe
				{
					fixed ( byte* data = &buffer[0].bar1 )
					{
						GL.BufferData( BufferTarget.ArrayBuffer, (IntPtr)( GuiGeometryVertex.VertexSize * used ), (IntPtr)data
#if USE_GLES2
									, BufferUsage.StaticDraw
#else
									, BufferUsageHint.StaticDraw
#endif
								);
						Display.CheckErr();
					}
				}
			}

			protected override void ShaderActivate()
			{
				GuiShader.Activate();

				GL.Uniform1( GuiShader.font_texture_id, 0 );
				GL.Uniform1( GuiShader.texture_id, 2 );
				Display.CheckErr();
			}


		}

		internal int vertex_attrib_id;
		internal int texture_attrib_id;
		internal int color_translation_id;
		internal int vertex_translation_id;
		internal int color1_id;
		internal int color2_id;
		internal int color3_id;
		internal int texture_select_id;
		internal int texture_id;
		internal int font_texture_id;

		const string Vertex =
			  @"
				uniform mat4 modelView;
				uniform mat4 worldView;
				uniform mat4 Projection;
				attribute vec3 vPosition;
				attribute vec2 in_texCoord;
				attribute int in_VertexTranslation;
				attribute int in_ColorTranslation;
				attribute vec3 in_Color1;
				attribute vec3 in_Color2;
				attribute vec3 in_Color3;
				attribute int in_TextureSelect;
				varying vec2 ex_texCoord;
				" +
#if USE_GLES2
			  @"float ex_ColorTranslation;
				vec3 ex_Color1;
				vec3 ex_Color2;
				vec3 ex_Color3;
				float ex_TextureSelect;
			  "+
#else
			  @"flat out int ex_ColorTranslation;
				flat out vec3 ex_Color1;
				flat out vec3 ex_Color2;
				flat out vec3 ex_Color3;
				flat out int ex_TextureSelect;
			  " +
#endif
			  @"void main() {
					if( in_VertexTranslation == 0 )
						gl_Position = vPosition;
					if( in_VertexTranslation == 64 )
						gl_Position = Projection * worldView * vPosition;
					if( in_VertexTranslation == 128 )
						gl_Position = Projection * worldView * modelView * vPosition;

					ex_texCoord = in_texCoord;
					ex_Color1 = in_Color1;
					ex_Color2 = in_Color2;
					ex_Color3 = in_Color3;
					ex_ColorTranslation = in_ColorTranslation;
					ex_TextureSelect = in_TextureSelect;
				}";

		const string Fragment =
			  @" varying vec2 ex_texCoord;
				in vec3 in_Color1;
				in vec3 in_Color2;
				in vec3 in_Color3;
				uniform sampler2D tex;
				uniform sampler2D fontTex;
			  " +
#if USE_GLES2
#else
			  @"flat in int ex_ColorTranslation;
			  " +
#endif
			  @"void main(void) {
					vec4 tmpColor;
					if( ex_textureSelect == 0 )
						tmpColor = texture2D( fontTex, ex_texCoord );
					else
						tmpColor = texture2D( tex, ex_texCoord );
					if( ex_ColorTranslation < 64 ) {
						glFragData[0] = texture2D( tex, ex_texCoord );
					} else if( ex_ColorTransation < 128 ) {
						glFragData[0] = ex_Color1 * texture2D( tex, ex_texCoord );
					} else if( ex_ColorTranslation < 192 ) {
						vec4 tmpColor = texture2D( tex, ex_texCoord );
						glFragData[0] = vec4( tmpColor.r * ex_Color1
						                    + tmpColor.g * ex_Color2
						                    + tmpColor.b * ex_Color3, tmpColor.a );
					} else {
						
						vec4 tmpColor = texture2D( tex, ex_texCoord );
						vec4 outColor = 0;
						if( tmpColor.r > 0 && tmpColor.r <= bar1 )
							outColor = vec4( ex_Color1, tmpColor.a );
						else if( tmpColor.g > 0 && tmpColor.g <= bar2 )
							outColor = vec4( ex_Color2, tmpColor.a );
						else if( tmpColor.b > 0 && tmpColor.b <= bar3 )
							outColor = vec4( ex_Color3, tmpColor.a );
						glFragData[0] = outColor;
					}
				}";


		internal override void Compile()
		{
			if( Compile( Vertex, Fragment ) )
			{
				vertex_attrib_id = GL.GetAttribLocation( Program, "vPosition" );
				texture_attrib_id = GL.GetAttribLocation( Program, "in_texCoord" );
				vertex_translation_id = GL.GetAttribLocation( Program, "in_VertexTranslation" );
				color_translation_id = GL.GetAttribLocation( Program, "in_ColorTranslation" );
				color1_id = GL.GetAttribLocation( Program, "in_Color1" );
				color2_id = GL.GetAttribLocation( Program, "in_Color2" );
				color3_id = GL.GetAttribLocation( Program, "in_Color3" );
				texture_select_id = GL.GetAttribLocation( Program, "textureSelect" );
				texture_id = GL.GetUniformLocation( Program, "tex" );
				font_texture_id = GL.GetUniformLocation( Program, "fontTex" );
			}
		}

		internal void SetUniforms( int texture, int font )
		{
			Shader.BindTexture( 2, texture );
			Display.CheckErr();
			GL.Uniform1( texture_id, 2 );
			Display.CheckErr();
			Shader.BindTexture( 0, texture );
			Display.CheckErr();
			GL.Uniform1( font_texture_id, 0 );
			Display.CheckErr();
		}

		internal void DrawQuad( float[] verts, float[] texture_uv )
		{
			GuiGeometryVertex[] Verts = new GuiGeometryVertex[4];
			Verts[0].u = (ushort)(texture_uv[0] * 32768);
			Verts[0].v = (ushort)(texture_uv[1] * 32768);
			Verts[0].vx = verts[0];
			Verts[0].vy = verts[1];
			Verts[0].vz = verts[2];
			Verts[1].u = (ushort)( texture_uv[2] * 32768 );
			Verts[1].v = (ushort)( texture_uv[3] * 32768 );
			Verts[1].vx = verts[3];
			Verts[1].vy = verts[4];
			Verts[1].vz = verts[5];
			Verts[2].u = (ushort)( texture_uv[4] * 32768 );
			Verts[2].v = (ushort)( texture_uv[5] * 32768 );
			Verts[2].vx = verts[6];
			Verts[2].vy = verts[7];
			Verts[2].vz = verts[8];
			Verts[3].u = (ushort)( texture_uv[6] * 32768 );
			Verts[3].v = (ushort)( texture_uv[7] * 32768 );
			Verts[3].vx = verts[9];
			Verts[3].vy = verts[10];
			Verts[3].vz = verts[11];
			unsafe
			{
				fixed ( byte* vertbuf = &Verts[0].bar1 )
					GuiGeometry.DoSetupBuffer( (IntPtr)vertbuf );
			}
#if !USE_GLES2
			GL.DrawArrays( PrimitiveType.TriangleStrip, 0, 4 );
#else
			GL.DrawArrays( BeginMode.TriangleStrip, 0, 4 );
#endif
			Display.CheckErr();
		}
		internal void DrawQuadTris( float[] verts, float[] texture_uv )
		{
			GuiGeometryVertex[] Verts = new GuiGeometryVertex[4];
			Verts[0].u = (ushort)( texture_uv[0] * 32768 );
			Verts[0].v = (ushort)( texture_uv[1] * 32768 );
			Verts[0].vx = verts[0];
			Verts[0].vy = verts[1];
			Verts[0].vz = verts[2];
			Verts[1].u = (ushort)( texture_uv[2] * 32768 );
			Verts[1].v = (ushort)( texture_uv[3] * 32768 );
			Verts[1].vx = verts[3];
			Verts[1].vy = verts[4];
			Verts[1].vz = verts[5];
			Verts[2].u = (ushort)( texture_uv[4] * 32768 );
			Verts[2].v = (ushort)( texture_uv[5] * 32768 );
			Verts[2].vx = verts[6];
			Verts[2].vy = verts[7];
			Verts[2].vz = verts[8];
			Verts[3].u = (ushort)( texture_uv[6] * 32768 );
			Verts[3].v = (ushort)( texture_uv[7] * 32768 );
			Verts[3].vx = verts[9];
			Verts[3].vy = verts[10];
			Verts[3].vz = verts[11];
			unsafe
			{
				fixed ( byte* vertbuf = &Verts[0].bar1 )
					GuiGeometry.DoSetupBuffer( (IntPtr)vertbuf );
			}
#if !USE_GLES2
			GL.DrawArrays( PrimitiveType.TriangleStrip, 0, 6 );
#else
			GL.DrawArrays( BeginMode.TriangleStrip, 0, 6 );
#endif
			Display.CheckErr();
		}


	}
}
