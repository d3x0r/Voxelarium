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
using OpenTK;
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
	internal class SimpleTextureShader : Shader
	{
		internal int vertex_attrib_id;
		internal int texture_attrib_id;
		internal int color_id;
		internal int texture_id;

		const string Vertex =
				  "attribute vec4 vPosition;\n"
				+ "attribute vec2 in_texCoord;\n"
				+ "uniform mat4 modelView;\n"
				+ "uniform mat4 worldView;\n"
				+ "uniform mat4 Projection;\n"
				+ " varying vec2 out_texCoord;\n"
				+ "void main() {\n"
				//+"  gl_Position = Projection * worldView * modelView * vPosition;\n"
				+ "  gl_Position = Projection * worldView * vPosition;\n"
				+ "  out_texCoord = in_texCoord;\n"
				+ "}\n";

		const string Fragment = " varying vec2 out_texCoord;\n"
			+ "uniform  vec4 in_Color;\n"
			+ " uniform sampler2D tex;\n"
			//+ "varying vec4 ex_Color;"
			+ "void main(void) {"
			//+ " vec4 tmp = texture2D( tex, out_texCoord );\n"
			+ "  gl_FragColor = in_Color * texture2D( tex, out_texCoord );\n"
			//+ "  gl_FragColor = vec4( out_texCoord.x, out_texCoord.y,tmp.r,1) ;"//*in_Color;"// * texture2D( tex, out_texCoord );\n"
			+ "}";


		internal override void Compile()
		{
			if( Compile( Vertex, Fragment ) )
			{
				vertex_attrib_id = GL.GetAttribLocation( Program, "vPosition" );
				texture_attrib_id = GL.GetAttribLocation( Program, "in_texCoord" );
				color_id = GL.GetUniformLocation( Program, "in_Color" );
				texture_id = GL.GetUniformLocation( Program, "tex" );
			}
		}

		internal void SetUniforms( int texture, ref Vector4 color )
		{
			GL.Uniform4( color_id, ref color );
			Display.CheckErr();
			//GL.ActiveTexture( TextureUnit.Texture0 );
			//Display.CheckErr();
			int texture_unit = Shader.BindTexture( texture );
			Display.CheckErr();
			GL.Uniform1( texture_id, texture_unit );
			Display.CheckErr();
		}

		internal void DrawQuad( float[] verts, float[] texture_uv )
		{
			GL.EnableVertexAttribArray( vertex_attrib_id );
			Display.CheckErr();
			GL.EnableVertexAttribArray( texture_attrib_id );
			Display.CheckErr();
			GL.VertexAttribPointer( vertex_attrib_id, 3, VertexAttribPointerType.Float, false, 0, verts );
			Display.CheckErr();
			GL.VertexAttribPointer( texture_attrib_id, 2, VertexAttribPointerType.Float, false, 0, texture_uv );
			Display.CheckErr();
#if !USE_GLES2
			GL.DrawArrays( PrimitiveType.TriangleStrip, 0, 4 );
#else
			GL.DrawArrays( BeginMode.TriangleStrip, 0, 4 );
#endif
			Display.CheckErr();
			GL.DisableVertexAttribArray( vertex_attrib_id );
			GL.DisableVertexAttribArray( texture_attrib_id );
		}
		internal void DrawQuadTris( float[] verts, float[] texture_uv )
		{
			GL.EnableVertexAttribArray( vertex_attrib_id );
			Display.CheckErr();
			GL.EnableVertexAttribArray( texture_attrib_id );
			Display.CheckErr();
			GL.VertexAttribPointer( vertex_attrib_id, 3, VertexAttribPointerType.Float, false, 0, verts );
			Display.CheckErr();
			GL.VertexAttribPointer( texture_attrib_id, 2, VertexAttribPointerType.Float, false, 0, texture_uv );
			Display.CheckErr();
#if !USE_GLES2
			GL.DrawArrays( PrimitiveType.Triangles, 0, 6 );
#else
			GL.DrawArrays( BeginMode.Triangles, 0, 6 );
#endif
			Display.CheckErr();
			GL.DisableVertexAttribArray( vertex_attrib_id );
			GL.DisableVertexAttribArray( texture_attrib_id );
		}

	}
}
