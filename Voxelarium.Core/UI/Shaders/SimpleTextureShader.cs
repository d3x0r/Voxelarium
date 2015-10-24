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
		int vertex_attrib_id;
		int texture_attrib_id;
		int color_id;
		int texture_id;

		const string Vertex =
				  "precision mediump float;\n"
				+ "precision mediump int;\n"
				+ "attribute vec4 vPosition;\n"
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

		const string Fragment = "precision mediump float;\n"
			+ "precision mediump int;\n"
			+ " varying vec2 out_texCoord;\n"
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
			GL.BindTexture( TextureTarget.Texture2D, texture );
			Display.CheckErr();
			GL.Uniform1( texture_id, 0 );
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
			GL.DrawArrays( PrimitiveType.TriangleStrip, 0, 4 );
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
			GL.DrawArrays( PrimitiveType.Triangles, 0, 6 );
			Display.CheckErr();
			GL.DisableVertexAttribArray( vertex_attrib_id );
			GL.DisableVertexAttribArray( texture_attrib_id );
		}

	}
}
