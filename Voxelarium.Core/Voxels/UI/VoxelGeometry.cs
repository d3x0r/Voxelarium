//#define USE_GLES2
#if !USE_GLES2
using OpenTK;
using OpenTK.Graphics.OpenGL;
#else
using OpenTK.Graphics.ES20;
#endif

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Runtime.InteropServices;
using System.Text;
using Voxelarium.Core.Support;
using Voxelarium.Core.UI;
using Voxelarium.Core.UI.Shaders;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.Voxels.UI
{
	internal class VoxelGeometry : IDisposable
	{
		const int VertexSize = 36; // 4*4 * 5
		[StructLayout( LayoutKind.Sequential, Pack=1 )]
		internal struct Vertex{
			// 0 first 4 (16)
			internal float p1, p2, p3, p4; 
			// 16 second 4 (16)
			internal ushort u, v;
#if USE_GLES2
			internal byte use_texture;
			internal byte flat_color;
			internal short power;
#else
			// 20  ...
			internal byte use_texture;
			//internal byte use_texture_overlay;
			// 21
			internal byte flat_color;
			// 22
			internal byte decal_texture;
			// 23 unused
			internal byte unused;
			// 24
			internal short power;
#endif
			// 26
			internal byte m1, m2;
			// 28 third 4 (16)
			internal byte c1r, c1g, c1b, c1a;
			// 32 fourth 4 (16)
			internal byte c2r, c2g, c2b, c2a;
			// 5 total
		}

		static VoxelGeometryShader Shader = new VoxelGeometryShader();
		Vertex[] solid_buffer;
		Vertex[] transparent_buffer;
		bool prior_transparent;
		bool prior_solid;
		Vertex[] buffer;
		int vbo_solid;
		int vao_solid;
		int vbo_transparent;
		int vao_transparent;

		bool dirty;
		bool solid_dirty;
		bool transparent_dirty;
		int transparent_available;
		int solid_available;
		int available;
		int solid_used;
		int transparent_used;
		int used;

		internal VoxelGeometry()
		{
			vbo_solid = -1;
			vao_solid = -1;
			vbo_transparent = -1;
			vao_transparent = -1;
			solid_available = 100;
			solid_buffer = new Vertex[solid_available];
			transparent_available = 100;
			transparent_buffer = new Vertex[transparent_available];
		}

		public void SetSolid()
		{
			if( prior_transparent )
			{
				transparent_dirty = dirty;
				transparent_used = used;
				transparent_buffer = buffer;
				transparent_available = available;
			}
			else if( prior_solid )
			{
				solid_dirty = dirty;
				solid_used = used;
				solid_buffer = buffer;
				solid_available = available;
			}
			prior_transparent = false;
			prior_solid = true;
			available = solid_available;
			buffer = solid_buffer;
			used = solid_used;
			dirty = solid_dirty;
		}
		public void SetTransparent()
		{
			if( !prior_transparent )
			{
				transparent_dirty = dirty;
				transparent_used = used;
				transparent_buffer = buffer;
				transparent_available = available;
			}
			else if( prior_solid )
			{
				solid_dirty = dirty;
				solid_used = used;
				solid_buffer = buffer;
				solid_available = available;
			}
			prior_transparent = true;
			prior_solid = false;
			available = transparent_available;
			buffer = transparent_buffer;
			used = transparent_used;
			dirty = transparent_dirty;
		}

		public void Dispose()
		{
			solid_buffer = null;
			transparent_buffer = null;
			buffer = null;
		}

		public void Clear()
		{
			used = 0;
		}

		void LoadBuffer( bool transparent )
		{
			bool setup_array = false;
			if( used == 0 )
				return;

			if( !transparent )
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
					Display.CheckErr();
					GL.BindBuffer( BufferTarget.ArrayBuffer, vbo_solid );
					Display.CheckErr();
					setup_array = true;

					solid_dirty = false;
					unsafe
					{
						fixed ( float* data = &solid_buffer[0].p1 )
						{
							GL.BufferData( BufferTarget.ArrayBuffer, (IntPtr)( VertexSize * solid_used), (IntPtr)data, BufferUsageHint.StaticDraw );
							Display.CheckErr();
						}
					}
				}
			}
			else
			{
				if( vbo_transparent == -1 )
					vbo_transparent = GL.GenBuffer();

				if( vao_transparent == -1 )
				{
					vao_transparent = GL.GenVertexArray();

#if USE_GLES2
					GL.Oes.BindVertexArray( vao_solid );
#else
					GL.BindVertexArray( vao_transparent );
#endif
					Display.CheckErr();
					GL.BindBuffer( BufferTarget.ArrayBuffer, vbo_transparent );
					Display.CheckErr();
					setup_array = true;

					transparent_dirty = false;
					unsafe
					{
						fixed ( float* data = &solid_buffer[0].p1 )
						{
							GL.BufferData( BufferTarget.ArrayBuffer, (IntPtr)( VertexSize * transparent_used), (IntPtr)data, BufferUsageHint.StaticDraw );
							Display.CheckErr();
						}
					}
				}
			}

			if( setup_array )
			{
				GL.VertexAttribPointer( Shader.vertex_attrib_id, 3, VertexAttribPointerType.Float, false, VertexSize, 0 );
				if( Display.CheckErr() )
				{
				}
				GL.EnableVertexAttribArray( Shader.vertex_attrib_id );
				if( Display.CheckErr() )
				{
					Log.log( "Geometry vertex attrib failed to get set." );
					GL.BindVertexArray( 0 );
					return;
				}
				if( Shader.texture_attrib_id >= 0 )
				{
					GL.EnableVertexAttribArray( Shader.texture_attrib_id );
					Display.CheckErr();
					GL.VertexAttribPointer( Shader.texture_attrib_id, 2, VertexAttribPointerType.UnsignedShort, false, VertexSize, 16 );
					Display.CheckErr();
				}
				if( Shader.use_texture_id >= 0 )
				{
					GL.EnableVertexAttribArray( Shader.use_texture_id );
					Display.CheckErr();
					GL.VertexAttribPointer( Shader.use_texture_id, 1, VertexAttribPointerType.Byte, false, VertexSize, 20 );
					Display.CheckErr();
				}
				if( Shader.flat_color_id >= 0 )
				{
					GL.EnableVertexAttribArray( Shader.flat_color_id );
					Display.CheckErr();
					GL.VertexAttribPointer( Shader.flat_color_id, 1, VertexAttribPointerType.Byte, false, VertexSize, 21 );
					Display.CheckErr();
				}
				if( Shader.decal_texture_id >= 0 )
				{
					GL.EnableVertexAttribArray( Shader.flat_color_id );
					Display.CheckErr();
					GL.VertexAttribPointer( Shader.flat_color_id, 1, VertexAttribPointerType.Byte, false, VertexSize, 22 );
					Display.CheckErr();
				}
				if( Shader.power_id >= 0 )
				{
					GL.EnableVertexAttribArray( Shader.power_id );
					Display.CheckErr();
					GL.VertexAttribPointer( Shader.power_id, 1, VertexAttribPointerType.Short, false, VertexSize, 24 );
					Display.CheckErr();
				}
				if( Shader.mod_attrib_id >= 0 )
				{
					GL.EnableVertexAttribArray( Shader.mod_attrib_id );
					Display.CheckErr();
					GL.VertexAttribPointer( Shader.mod_attrib_id, 2, VertexAttribPointerType.UnsignedByte, false, VertexSize, 26 );
					Display.CheckErr();
				}
				if( Shader.color_id >= 0 )
				{
					GL.EnableVertexAttribArray( Shader.color_id );
					Display.CheckErr();
					GL.VertexAttribPointer( Shader.color_id, 4, VertexAttribPointerType.UnsignedByte, false, VertexSize, 28 );
					Display.CheckErr();
				}
				if( Shader.face_color_id >= 0 )
				{
					GL.EnableVertexAttribArray( Shader.face_color_id );
					Display.CheckErr();
					GL.VertexAttribPointer( Shader.face_color_id, 4, VertexAttribPointerType.UnsignedByte, false, VertexSize, 32 );
					Display.CheckErr();
				}
#if USE_GLES2
				GL.Oes.BindVertexArray( 0 );
#else
				GL.BindVertexArray( 0 );
#endif
				GL.BindBuffer( BufferTarget.ArrayBuffer, 0 );
			}

			if( !transparent )
			{
				if( transparent_dirty )
				{
					GL.BindBuffer( BufferTarget.ArrayBuffer, vbo_solid );
					unsafe
					{
						fixed ( float* data = &solid_buffer[0].p1 )
						{
							GL.BufferData( BufferTarget.ArrayBuffer, (IntPtr)( VertexSize * solid_used ), (IntPtr)data, BufferUsageHint.StaticDraw );
							Display.CheckErr();
						}
					}
				}
			}
			else
			{
				if( solid_dirty )
				{
					GL.BindBuffer( BufferTarget.ArrayBuffer, vbo_solid );
					unsafe
					{
						fixed ( float* data = &solid_buffer[0].p1 )
						{
							GL.BufferData( BufferTarget.ArrayBuffer, (IntPtr)( VertexSize * transparent_used ), (IntPtr)data, BufferUsageHint.StaticDraw );
							Display.CheckErr();
						}
					}
				}
			}
			//GL.
			//void glBindBuffer​(enum target, uint bufferName)
		}

		internal void DrawBuffer( bool transparent, int atlas_id )
		{
			if( transparent && transparent_used == 0 )
				return;
			if( !transparent && solid_used == 0 )
				return;
			Shader.Activate();
			LoadBuffer( transparent );
			GL.BindTexture( TextureTarget.Texture2D, atlas_id );
			Display.CheckErr();
			GL.Uniform1( Shader.texture_id, 0 );
			Display.CheckErr();

			if( transparent )
			{
#if USE_GLES2
				GL.Oes.BindVertexArray( vao_transparent );
#else
				GL.BindVertexArray( vao_transparent );
#endif
				GL.DrawArrays( PrimitiveType.Triangles, 0, transparent_used * 3 );
			}
			else
			{
#if USE_GLES2
				GL.Oes.BindVertexArray( vao_solid );
#else
				GL.BindVertexArray( vao_solid );
#endif
				GL.DrawArrays( PrimitiveType.Triangles, 0, solid_used * 3 );
			}
			GL.BindVertexArray( 0 );
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

		internal void Reset()
		{
			solid_dirty = true;
			transparent_dirty = true;
			used = 0;
		}

		internal void AddTriangle( float[] tri_points, float[] color )
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
				buffer[used].c1r = (byte)(color[point_base + 0]*255 );
				buffer[used].c1g = (byte)( color[point_base + 1] * 255 );
				buffer[used].c1b = (byte)( color[point_base + 2] * 255 );
				buffer[used].c1a = (byte)( color[point_base + 3] * 255);
				used++;
			}
		}

		static int[] quad_base = { 0, 1, 2, 2, 1, 3 };
		internal void AddQuad( float[] quad_points, float[] texture )
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
				buffer[used].u = (ushort)(texture[point_base + 0] );
				buffer[used].v = (ushort)( texture[point_base + 1] );

				buffer[used].c1r = 255;
				buffer[used].c1g = 255;
				buffer[used].c1b = 255;
				buffer[used].c1a = 255;

				used++;
			}
		}

		internal void AddQuad( ref Vector3 P1, ref Vector3 P2, ref Vector3 P3, ref Vector3 P4, ref Box2D texture )
		{
			if( (used+6) >= available ) expand();
			dirty = true;
			//for( int n = 0; n < 6; n++ )
			{
				buffer[used].use_texture = 1;
				buffer[used].u = (ushort)( texture.Position.X  );
				buffer[used].v = (ushort)( texture.Position.Y  );

				buffer[used].p1 = P1.X;
				buffer[used].p2 = P1.Y;
				buffer[used].p3 = P1.Z;

				buffer[used].c1r = 255;
				buffer[used].c1g = 255;
				buffer[used].c1b = 255;
				buffer[used].c1a = 255;

				used++;

				buffer[used].use_texture = 1;
				buffer[used].u = (ushort)( ( texture.Position.X + texture.Size.X )  );
				buffer[used].v = (ushort)( texture.Position.Y  );

				buffer[used].p1 = P2.X;
				buffer[used].p2 = P2.Y;
				buffer[used].p3 = P2.Z;

				buffer[used].c1r = 255;
				buffer[used].c1g = 255;
				buffer[used].c1b = 255;
				buffer[used].c1a = 255;
				used++;

				buffer[used].use_texture = 1;
				buffer[used].u = (ushort)( ( texture.Position.X  )  );
				buffer[used].v = (ushort)( ( texture.Position.Y +texture.Size.Y )  );

				buffer[used].p1 = P3.X;
				buffer[used].p2 = P3.Y;
				buffer[used].p3 = P3.Z;

				buffer[used].c1r = 255;
				buffer[used].c1g = 255;
				buffer[used].c1b = 255;
				buffer[used].c1a = 255;
				used++;

				buffer[used].use_texture = 1;
				buffer[used].u = (ushort)( ( texture.Position.X + texture.Size.X )  );
				buffer[used].v = (ushort)( texture.Position.Y  );

				buffer[used].p1 = P2.X;
				buffer[used].p2 = P2.Y;
				buffer[used].p3 = P2.Z;

				buffer[used].c1r = 255;
				buffer[used].c1g = 255;
				buffer[used].c1b = 255;
				buffer[used].c1a = 255;
				used++;

				buffer[used].use_texture = 1;
				buffer[used].u = (ushort)( ( texture.Position.X )  );
				buffer[used].v = (ushort)( ( texture.Position.Y + texture.Size.Y )  );

				buffer[used].p1 = P3.X;
				buffer[used].p2 = P3.Y;
				buffer[used].p3 = P3.Z;

				buffer[used].c1r = 255;
				buffer[used].c1g = 255;
				buffer[used].c1b = 255;
				buffer[used].c1a = 255;
				used++;

				buffer[used].use_texture = 1;
				buffer[used].u = (ushort)( ( texture.Position.X + texture.Size.Y )  );
				buffer[used].v = (ushort)( ( texture.Position.Y + texture.Size.Y )  );

				buffer[used].p1 = P4.X;
				buffer[used].p2 = P4.Y;
				buffer[used].p3 = P4.Z;

				buffer[used].c1r = 255;
				buffer[used].c1g = 255;
				buffer[used].c1b = 255;
				buffer[used].c1a = 255;
				used++;
			}
		}

		internal void AddQuad( ref Vector3 P1, ref Vector3 P2, ref Vector3 P3, ref Vector3 P4, Color face, Color edge, short power )
		{
			if( ( used + 6 ) >= available ) expand();
			dirty = true;
			//for( int n = 0; n < 6; n++ )
			{
				buffer[used].use_texture = 0;
				buffer[used].decal_texture = 0;
				buffer[used].flat_color = 0;
				buffer[used].m1 = ( 0 );
				buffer[used].m2 = ( 0 );

				buffer[used].p1 = P1.X;
				buffer[used].p2 = P1.Y;
				buffer[used].p3 = P1.Z;

				buffer[used].c1r = edge.R;
				buffer[used].c1g = edge.G;
				buffer[used].c1b = edge.B;
				buffer[used].c1a = edge.A;
				buffer[used].c2r = face.R;
				buffer[used].c2g = face.G;
				buffer[used].c2b = face.B;
				buffer[used].c2a = face.A;
				buffer[used].power = power;

				used++;

				buffer[used].use_texture = 0;
				buffer[used].decal_texture = 0;
				buffer[used].flat_color = 0;
				buffer[used].m1 = ( 1 );
				buffer[used].m2 = ( 0 );

				buffer[used].p1 = P2.X;
				buffer[used].p2 = P2.Y;
				buffer[used].p3 = P2.Z;

				buffer[used].c1r = edge.R;
				buffer[used].c1g = edge.G;
				buffer[used].c1b = edge.B;
				buffer[used].c1a = edge.A;
				buffer[used].c2r = face.R;
				buffer[used].c2g = face.G;
				buffer[used].c2b = face.B;
				buffer[used].c2a = face.A;
				buffer[used].power = power;
				used++;

				buffer[used].use_texture = 0;
				buffer[used].decal_texture = 0;
				buffer[used].flat_color = 0;
				buffer[used].m1 = ( 0 );
				buffer[used].m2 = ( 1 );

				buffer[used].p1 = P3.X;
				buffer[used].p2 = P3.Y;
				buffer[used].p3 = P3.Z;

				buffer[used].c1r = edge.R;
				buffer[used].c1g = edge.G;
				buffer[used].c1b = edge.B;
				buffer[used].c1a = edge.A;
				buffer[used].c2r = face.R;
				buffer[used].c2g = face.G;
				buffer[used].c2b = face.B;
				buffer[used].c2a = face.A;
				buffer[used].power = power;
				used++;

				buffer[used].use_texture = 0;
				buffer[used].decal_texture = 0;
				buffer[used].flat_color = 0;
				buffer[used].m1 = ( 1 );
				buffer[used].m2 = ( 0 );

				buffer[used].p1 = P2.X;
				buffer[used].p2 = P2.Y;
				buffer[used].p3 = P2.Z;

				buffer[used].c1r = edge.R;
				buffer[used].c1g = edge.G;
				buffer[used].c1b = edge.B;
				buffer[used].c1a = edge.A;
				buffer[used].c2r = face.R;
				buffer[used].c2g = face.G;
				buffer[used].c2b = face.B;
				buffer[used].c2a = face.A;
				buffer[used].power = power;
				used++;

				buffer[used].use_texture = 0;
				buffer[used].decal_texture = 0;
				buffer[used].flat_color = 0;
				buffer[used].m1 = ( 0 );
				buffer[used].m2 = ( 1 );

				buffer[used].p1 = P3.X;
				buffer[used].p2 = P3.Y;
				buffer[used].p3 = P3.Z;

				buffer[used].c1r = edge.R;
				buffer[used].c1g = edge.G;
				buffer[used].c1b = edge.B;
				buffer[used].c1a = edge.A;
				buffer[used].c2r = face.R;
				buffer[used].c2g = face.G;
				buffer[used].c2b = face.B;
				buffer[used].c2a = face.A;
				buffer[used].power = power;
				used++;

				buffer[used].use_texture = 0;
				buffer[used].decal_texture = 0;
				buffer[used].flat_color = 0;
				buffer[used].m1 = ( 1 );
				buffer[used].m2 = ( 1 );

				buffer[used].p1 = P4.X;
				buffer[used].p2 = P4.Y;
				buffer[used].p3 = P4.Z;

				buffer[used].c1r = edge.R;
				buffer[used].c1g = edge.G;
				buffer[used].c1b = edge.B;
				buffer[used].c1a = edge.A;
				buffer[used].c2r = face.R;
				buffer[used].c2g = face.G;
				buffer[used].c2b = face.B;
				buffer[used].c2a = face.A;
				buffer[used].power = power;
				used++;
			}
		}

		internal void AddSimpleColorQuad( float[] quad_points, float[] color )
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

				buffer[used].c1r = (byte)(color[0] * 255 );
				buffer[used].c1g = (byte)( color[1] * 255 );
				buffer[used].c1b = (byte)( color[2] * 255 );
				buffer[used].c1a = (byte)( color[3]*255);
			}
		}

	}
}
