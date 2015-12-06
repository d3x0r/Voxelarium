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
using Voxelarium.LinearMath;
using Voxelarium.Core.Support;
using Voxelarium.Common;
#if !USE_GLES2
using OpenTK.Graphics.OpenGL;
#else
using OpenTK.Graphics.ES20;
#endif
using OpenTK;

namespace Voxelarium.Core.UI.Shaders
{
	internal class SimpleVertShader : Shader
	{
		internal int vertex_attrib_id;
		internal int color_attrib_id;

		const string Vertex =
				"uniform mat4 modelView;\n"
			+ "uniform mat4 worldView;\n"
			+ "uniform mat4 Projection;\n"
			+ "attribute vec4 vPosition;"
			+ "attribute vec4 in_Color;\n"
			+ "varying vec4 ex_Color;\n"
			+ "void main(void) {"
			//+ "  gl_Position = Projection * worldView * modelView * vPosition;"
			+ "  gl_Position = Projection * worldView  * vPosition;"
			//+ "  gl_Position = vPosition;"
			+ "  ex_Color = in_Color;"
			+ "}"
			;

		const string Fragment = //"uniform  vec4 in_Color;\n"
			"varying vec4 ex_Color;"
			+ "void main(void) {"
			+ "  gl_FragColor = ex_Color;"
			+ "}";


		internal override void Compile()
		{
			if( Compile( Vertex, Fragment ) )
			{
				vertex_attrib_id = GL.GetAttribLocation( Program, "vPosition" );
				color_attrib_id = GL.GetAttribLocation( Program, "in_Color" );
			}
		}

		internal void DrawQuad( Vector3[] verts, ref Vector4[] colors  )	
		{
			//GL.Uniform4( color_id, ref color );
			GL.EnableVertexAttribArray( color_attrib_id );
			Display.CheckErr();
			GL.EnableVertexAttribArray( vertex_attrib_id );
			Display.CheckErr();
			GL.VertexAttribPointer( vertex_attrib_id, 3, VertexAttribPointerType.Float, false, 0, verts );
			Display.CheckErr();
			GL.VertexAttribPointer( vertex_attrib_id, 4, VertexAttribPointerType.Float, false, 0, colors );
			Display.CheckErr();
#if !USE_GLES2
			GL.DrawArrays( PrimitiveType.TriangleStrip, 0, 4 );
#else
			GL.DrawArrays( BeginMode.TriangleStrip, 0, 4 );
#endif
			Display.CheckErr();
		}

		internal void DrawLine( Vector3[] verts, ref Vector4[] colors )
		{
			GL.EnableVertexAttribArray( color_attrib_id );
			Display.CheckErr();
			GL.EnableVertexAttribArray( vertex_attrib_id );
			Display.CheckErr();
			GL.VertexAttribPointer( vertex_attrib_id, 3, VertexAttribPointerType.Float, false, 0, verts );
			Display.CheckErr();
			GL.VertexAttribPointer( vertex_attrib_id, 4, VertexAttribPointerType.Float, false, 0, colors );
			Display.CheckErr();
			#if !USE_GLES2
			GL.DrawArrays( PrimitiveType.Lines, 0, 2 );
			#else
			GL.DrawArrays( BeginMode.Lines, 0, 2 );
			#endif
			Display.CheckErr();

		}

		internal unsafe void DrawBox( float[] verts, ref Vector4[] colors )
		{
			GL.EnableVertexAttribArray( color_attrib_id );
			Display.CheckErr();

			GL.EnableVertexAttribArray( vertex_attrib_id );
			Display.CheckErr();
			GL.VertexAttribPointer( vertex_attrib_id, 3, VertexAttribPointerType.Float, false, 0, verts );
			Display.CheckErr();
			GL.VertexAttribPointer( vertex_attrib_id, 4, VertexAttribPointerType.Float, false, 0, colors );
			Display.CheckErr();
			GL.LineWidth( 0.5f );
			Display.CheckErr();
			try
			{
#if !USE_GLES2
				GL.DrawArrays( PrimitiveType.Lines, 0, 24 );
#else
				GL.DrawArrays( BeginMode.Lines, 0, 24 );
#endif
				Display.CheckErr();
			}
			catch( Exception e )
			{
					GL.UseProgram( 0 );
					Shader.prior_activated = 0;
				Log.log( "Draw exception in lines : " + e.Message );
			}
		}
	}
}
