//#define USE_GLES2
#if !USE_GLES2
using OpenTK.Graphics.OpenGL;
#else
using OpenTK.Graphics.ES20;
#endif

using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
using Voxelarium.Core.UI.Shaders;

namespace Voxelarium.Core.Voxels.UI
{
	internal class VoxelGeometry : IDisposable
	{
		const int VertexSize = 80; // 4*4 * 5
		[StructLayout( LayoutKind.Sequential, Pack=1 )]
		internal struct Vertex{
			// first 4 (16)
			internal float p1, p2, p3, p4; 
			// second 4
			internal ushort u, v;
#if USE_GLES2
			internal byte use_texture;
			internal byte flat_color;
			internal short power;
#else
			internal byte use_texture;
			internal byte flat_color;
			internal short power;
#endif
			internal float m1, m2;
			// third 4
			internal float c1r, c1g, c1b, c1a;
			// fourth 4
			internal float c2r, c2g, c2b, c2a;
			// 5 total
		}

		static VoxelGeometryShader Shader = new VoxelGeometryShader();
		Vertex[] buffer;
		int vbo_solid;
		int vao_solid;
		int vbo_transparent;
		int vao_transparent;

		bool dirty;
		int available;
		int used;

		internal VoxelGeometry()
		{
			vbo_solid = -1;
			available = 100;
			buffer = new Vertex[available];
		}

		public void Dispose()
		{
			buffer = null;
		}

		void LoadBuffer()
		{
			if( vbo_solid == -1 )
				vbo_solid = GL.GenBuffer();

			if( vao_solid == -1 )
			{
				vao_solid = GL.GenVertexArray();

#if USE_GLES2
				GL.Oes.BindVertexArray( vao_solid );
#else
				GL.BindVertexArray( vao_solid );
#endif
				GL.BindBuffer( BufferTarget.ArrayBuffer, vbo_solid );
				GL.BufferData( BufferTarget.ArrayBuffer, VertexSize * used, buffer, BufferUsageHint.StaticDraw );

				GL.EnableVertexAttribArray( Shader.vertex_attrib_id );
				GL.VertexAttribPointer( 1, 4, VertexAttribPointerType.Float, false, VertexSize, 0 );
				GL.EnableVertexAttribArray( Shader.texture_attrib_id );
				GL.VertexAttribPointer( 1, 2, VertexAttribPointerType.UnsignedShort, false, VertexSize, 16 );
				GL.EnableVertexAttribArray( Shader.use_texture_id );
				GL.VertexAttribPointer( 1, 1, VertexAttribPointerType.Byte, false, VertexSize, 20 );
				GL.EnableVertexAttribArray( Shader.flat_color_id );
				GL.VertexAttribPointer( 1, 1, VertexAttribPointerType.Byte, false, VertexSize, 21 );
				GL.EnableVertexAttribArray( Shader.power_id );
				GL.VertexAttribPointer( 1, 1, VertexAttribPointerType.Short, false, VertexSize, 28 );
				GL.EnableVertexAttribArray( Shader.mod_attrib_id );
				GL.VertexAttribPointer( 1, 2, VertexAttribPointerType.Float, false, VertexSize, 32 );
				GL.EnableVertexAttribArray( Shader.color_id );
				GL.VertexAttribPointer( 1, 4, VertexAttribPointerType.Float, false, VertexSize, 48 );
				GL.EnableVertexAttribArray( Shader.face_color_id );
				GL.VertexAttribPointer( 1, 4, VertexAttribPointerType.Float, false, VertexSize, 64 );
#if USE_GLES2
				GL.Oes.BindVertexArray( 0 );
#else
				GL.BindVertexArray( 0 );
#endif
				GL.BindBuffer( BufferTarget.ArrayBuffer, 0 );
			}

			if( dirty )
			{
				GL.BindBuffer( BufferTarget.ArrayBuffer, vbo_solid );
				GL.BufferData( BufferTarget.ArrayBuffer, VertexSize * used, buffer, BufferUsageHint.StaticDraw );
			}
			//GL.
			//void glBindBuffer​(enum target, uint bufferName)
		}

		internal void DrawBuffer( bool transparent, int atlas_id )
		{
			Shader.Activate();
			LoadBuffer();

			if( transparent )
			{
#if USE_GLES2
				GL.Oes.BindVertexArray( vao_transparent );
#else
				GL.BindVertexArray( vao_transparent );
#endif
			}
			else
			{
#if USE_GLES2
				GL.Oes.BindVertexArray( vao_solid );
#else
				GL.BindVertexArray( vao_solid );
#endif
			}
			GL.Uniform1( Shader.texture_id, atlas_id );
			GL.DrawArrays( PrimitiveType.Triangles, 0, used * 3 );
		}

		void expand()
		{
			int new_count = available * 2;
			Vertex[] new_buffer = new Vertex[new_count];
			for( int n = 0; n < used; n++ )
			{
				new_buffer[n] = buffer[n];
			}
			available = new_count;
			buffer = new_buffer;
		}

		void Reset()
		{
			dirty = true;
			used = 0;
		}

		void AddTriangle( float[] tri_points, float[] color )
		{
			int n;
			int b;
			int point_base;
			if( used == available ) expand();
			dirty = true;
			for( n = 0; n < 3; n++ )
			{
				buffer[used].use_texture = 0;
				b = used * 3;
				point_base = n * 3;
				buffer[used].p1 = tri_points[point_base + 0];
				buffer[used].p2 = tri_points[point_base + 1];
				buffer[used].p3 = tri_points[point_base + 2];
				b = used * 4;
				buffer[used].c1r = color[point_base + 0];
				buffer[used].c1g = color[point_base + 1];
				buffer[used].c1b = color[point_base + 2];
				buffer[used].c1a = color[point_base + 3];
				used++;
			}
		}

		static int[] quad_base = { 0, 1, 2, 2, 1, 3 };
		void AddQuad( float[] quad_points, float[] texture )
		{
			int point_base;
			if( used == available ) expand();
			dirty = true;
			for( int n = 0; n < 6; n++ )
			{
				buffer[used].use_texture = 0;

				point_base = quad_base[n] * 3;
				buffer[used].p1 = quad_points[point_base + 0];
				buffer[used].p2 = quad_points[point_base + 1];
				buffer[used].p3 = quad_points[point_base + 2];

				point_base = quad_base[n] * 2;
				buffer[used].u = (ushort)(texture[point_base + 0] * 65535.0f);
				buffer[used].v = (ushort)( texture[point_base + 1] * 65535.0f);

				buffer[used].c1r = 1;
				buffer[used].c1g = 1;
				buffer[used].c1b = 1;
				buffer[used].c1a = 1;

				used++;
			}
		}

		void AddSimpleColorQuad( float[] quad_points, float[] color )
		{
			int point_base;
			if( used == available ) expand();
			dirty = true;
			for( int n = 0; n < 6; n++ )
			{
				buffer[used].use_texture = 0;
				buffer[used].flat_color = 1;
				point_base = quad_base[n] * 3;
				buffer[used].p1 = quad_points[point_base + 0];
				buffer[used].p2 = quad_points[point_base + 1];
				buffer[used].p3 = quad_points[point_base + 2];

				buffer[used].c1r = color[0];
				buffer[used].c1g = color[1];
				buffer[used].c1b = color[2];
				buffer[used].c1a = color[3];
			}
		}

	}
}
