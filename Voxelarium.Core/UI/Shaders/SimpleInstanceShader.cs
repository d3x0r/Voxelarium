#if !USE_GLES2
using Bullet.LinearMath;
using OpenTK;
using OpenTK.Graphics.OpenGL;
#else
using OpenTK.Graphics.ES20;
#endif
using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.UI.Shaders
{
	internal class SimpleInstanceShader: Shader
	{
		int vertex_attrib_id;
		int color_id;

		const string Vertex =
				"precision mediump float;\n"
			 + "precision mediump int;\n"
			+ "uniform mat4 modelView;\n"
			+ "uniform mat4 worldView;\n"
			+ "uniform mat4 Projection;\n"
			+ "attribute vec4 vPosition;"
			//+ "attribute vec4 in_Color;\n"
			//+ "varying vec4 ex_Color;\n"
			+ "void main(void) {"
			//+ "  gl_Position = modelView  * vPosition;"
			+ "  gl_Position = Projection * worldView * modelView  * vPosition;"
			//+ "  gl_Position = Projection * worldView  * vPosition;"
			//+ "  gl_Position = vPosition;"
			//+ "  ex_Color = in_Color;"
			+ "}"
			;

		const string Fragment = "precision mediump float;\n"
			+ "precision mediump int;\n"
			+ "uniform  vec4 in_Color;\n"
			//+ "varying vec4 ex_Color;"
			+ "void main(void) {"
			+ "  gl_FragColor = in_Color;"
			+ "}";


		internal override void Compile()
		{
			if( Compile( Vertex, Fragment ) )
			{
				vertex_attrib_id = GL.GetAttribLocation( Program, "vPosition" );
				color_id = GL.GetUniformLocation( Program, "in_Color" );
			}
		}

		internal unsafe void SetModelMatrix( ref btTransform m )
		{
			btMatrix3x3 tmp;
			m.GetGLMatrix( out tmp );
			float* matrix_ptr = &tmp.m_el0.x;
			//fixed ( float* matrix_ptr = &Display.worldview.m_el0.x )
			{
				GL.UniformMatrix4( modelview_id, 1, false, matrix_ptr );
			}
		}

		internal void DrawQuad( Vector3[] verts, ref Vector4 color )
		{
			GL.Uniform4( color_id, ref color );
			Display.CheckErr();
			GL.EnableVertexAttribArray( vertex_attrib_id );
			Display.CheckErr();
			GL.VertexAttribPointer( vertex_attrib_id, 3, VertexAttribPointerType.Float, false, 0, verts );
			Display.CheckErr();
			GL.DrawArrays( PrimitiveType.TriangleStrip, 0, 4 );
			Display.CheckErr();
		}

		internal unsafe void DrawBox( float[] verts, ref Vector4 color )
		{
			GL.Uniform4( color_id, ref color );

			GL.EnableVertexAttribArray( vertex_attrib_id );
			Display.CheckErr();
			GL.VertexAttribPointer( vertex_attrib_id, 3, VertexAttribPointerType.Float, false, 0, verts );
			Display.CheckErr();
			GL.LineWidth( 0.5f );
			GL.DrawArrays( PrimitiveType.Lines, 0, 24 );
		}
	}
}
