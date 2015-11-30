//#define USE_GLES2
#if !USE_GLES2
using OpenTK.Graphics.OpenGL;
#else
using OpenTK.Graphics.ES20;
#endif

using OpenTK;
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
			// 20  ...
			internal byte use_texture;
			// 21
			internal byte flat_color;
			// 22
			internal byte decal_texture;
			// 23 unused
			internal byte unused;
			// 24
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

		static VoxelGeometryShader GeometryShader = new VoxelGeometryShader();
		Vertex[] buffer;
		int vbo_solid;
		int vao_solid;
		int vbo_transparent;
		int vao_transparent;
		int vbo_custom;
		int vao_custom;

		bool dirty;

		bool prior_solid;
		Vertex[] solid_buffer;
		bool solid_dirty;
		int solid_available;
		int solid_used;
		int available;

		bool prior_transparent;
		Vertex[] transparent_buffer;
		bool transparent_dirty;
		int transparent_available;
		int transparent_used;

		bool prior_custom;
		Vertex[] custom_buffer;
		bool custom_dirty;
		int custom_available;
		int custom_used;
		int used;

		// this is the last direction that the transparent render was sorted.
		internal VoxelSector.RelativeVoxelOrds transparent_render_sorting;
		internal int sortedX, sortedY, sortedZ;

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

		void UpdateInternals()
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
			else if( prior_custom )
			{
				custom_dirty = dirty;
				custom_used = used;
				custom_buffer = buffer;
				custom_available = available;
			}
		}

		public void SetCustom()
		{
			UpdateInternals();
			prior_custom = true;
			prior_transparent = false;
			prior_solid = false;
			available = custom_available;
			buffer = custom_buffer;
			used = custom_used;
			dirty = custom_dirty;
		}

		public void SetSolid()
		{
			UpdateInternals();
			prior_custom = false;
			prior_transparent = false;
			prior_solid = true;
			available = solid_available;
			buffer = solid_buffer;
			used = solid_used;
			dirty = solid_dirty;
		}
		public void SetTransparent()
		{
			UpdateInternals();
			prior_custom = false;
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

		void SetupBuffer()
		{
			GL.VertexAttribPointer( GeometryShader.vertex_attrib_id, 3, VertexAttribPointerType.Float, false, VertexSize, 0 );
			Display.CheckErr();
			GL.EnableVertexAttribArray( GeometryShader.vertex_attrib_id );
			Display.CheckErr();
			if( GeometryShader.texture_attrib_id >= 0 )
			{
				GL.EnableVertexAttribArray( GeometryShader.texture_attrib_id );
				Display.CheckErr();
				GL.VertexAttribPointer( GeometryShader.texture_attrib_id, 2, VertexAttribPointerType.UnsignedShort, false, VertexSize, 16 );
				Display.CheckErr();
			}
			if( GeometryShader.use_texture_id >= 0 )
			{
				GL.EnableVertexAttribArray( GeometryShader.use_texture_id );
				Display.CheckErr();
				GL.VertexAttribPointer( GeometryShader.use_texture_id, 1, VertexAttribPointerType.Byte, false, VertexSize, 20 );
				Display.CheckErr();
			}
			if( GeometryShader.flat_color_id >= 0 )
			{
				GL.EnableVertexAttribArray( GeometryShader.flat_color_id );
				Display.CheckErr();
				GL.VertexAttribPointer( GeometryShader.flat_color_id, 1, VertexAttribPointerType.Byte, false, VertexSize, 21 );
				Display.CheckErr();
			}
			if( GeometryShader.decal_texture_id >= 0 )
			{
				GL.EnableVertexAttribArray( GeometryShader.flat_color_id );
				Display.CheckErr();
				GL.VertexAttribPointer( GeometryShader.flat_color_id, 1, VertexAttribPointerType.Byte, false, VertexSize, 22 );
				Display.CheckErr();
			}
			if( GeometryShader.power_id >= 0 )
			{
				GL.EnableVertexAttribArray( GeometryShader.power_id );
				Display.CheckErr();
				GL.VertexAttribPointer( GeometryShader.power_id, 1, VertexAttribPointerType.Short, false, VertexSize, 24 );
				Display.CheckErr();
			}
			if( GeometryShader.mod_attrib_id >= 0 )
			{
				GL.EnableVertexAttribArray( GeometryShader.mod_attrib_id );
				Display.CheckErr();
				GL.VertexAttribPointer( GeometryShader.mod_attrib_id, 2, VertexAttribPointerType.UnsignedByte, false, VertexSize, 26 );
				Display.CheckErr();
			}
			if( GeometryShader.color_id >= 0 )
			{
				GL.EnableVertexAttribArray( GeometryShader.color_id );
				Display.CheckErr();
				GL.VertexAttribPointer( GeometryShader.color_id, 4, VertexAttribPointerType.UnsignedByte, false, VertexSize, 28 );
				Display.CheckErr();
			}
			if( GeometryShader.face_color_id >= 0 )
			{
				GL.EnableVertexAttribArray( GeometryShader.face_color_id );
				Display.CheckErr();
				GL.VertexAttribPointer( GeometryShader.face_color_id, 4, VertexAttribPointerType.UnsignedByte, false, VertexSize, 32 );
				Display.CheckErr();
			}
		}

		void LoadBuffer( bool transparent, bool custom )
		{
			bool setup_array = false;

			if( !transparent && !custom )
			{
				if( solid_used == 0 )
					return;
				if( vbo_solid == -1 )
#if USE_GLES2
					GL.GenBuffers( 1, out vbo_solid );
#else
					vbo_solid = GL.GenBuffer();
#endif
				if( vao_solid == -1 )
				{
#if USE_GLES3
					GL.GenVertexArrays();
#elif !USE_GLES2
					vao_solid = GL.GenVertexArray();
#else
					GL.Oes.GenVertexArrays( 1, out vao_solid);
#endif

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
							GL.BufferData( BufferTarget.ArrayBuffer, (IntPtr)( VertexSize * solid_used), (IntPtr)data
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
			}
			else if( transparent && !custom )
			{
				if( transparent_used == 0 )
					return;
				if( vbo_transparent == -1 )
#if USE_GLES2
					GL.GenBuffers( 1, out vbo_transparent );
#else
					vbo_transparent = GL.GenBuffer();
#endif
				if( vao_transparent == -1 )
				{
#if USE_GLES3
#elif !USE_GLES2
					vao_transparent = GL.GenVertexArray();
#else
					GL.Oes.GenVertexArrays( 1, out vao_transparent);
#endif

#if USE_GLES2
					GL.Oes.BindVertexArray( vao_transparent );
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
						fixed ( float* data = &transparent_buffer[0].p1 )
						{
							GL.BufferData( BufferTarget.ArrayBuffer, (IntPtr)( VertexSize * transparent_used), (IntPtr)data
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
			}
			else if( !transparent && custom )
			{
				if( custom_used == 0 )
					return;
				if( vbo_custom == -1 )
#if USE_GLES2
					GL.GenBuffers( 1, out vbo_custom );
#else
					vbo_custom = GL.GenBuffer();
#endif

				if( vao_custom == -1 )
				{
#if USE_GLES3
					vao_custom = GL.GenVertexArray();
#elif !USE_GLES2
					vao_custom = GL.GenVertexArray();
#else
					GL.Oes.GenVertexArrays( 1, out vao_custom);
#endif

#if USE_GLES2
					GL.Oes.BindVertexArray( vao_custom );
#else
					GL.BindVertexArray( vao_custom );
#endif
					Display.CheckErr();
					GL.BindBuffer( BufferTarget.ArrayBuffer, vbo_custom );
					Display.CheckErr();
					setup_array = true;

					custom_dirty = false;
					unsafe
					{
						fixed ( float* data = &custom_buffer[0].p1 )
						{
							GL.BufferData( BufferTarget.ArrayBuffer, (IntPtr)( VertexSize * custom_used ), (IntPtr)data
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
			}

			if( setup_array )
			{
				GL.VertexAttribPointer( GeometryShader.vertex_attrib_id, 3, VertexAttribPointerType.Float, false, VertexSize, 0 );
				Display.CheckErr();
				GL.EnableVertexAttribArray( GeometryShader.vertex_attrib_id );
				Display.CheckErr();
				if( GeometryShader.texture_attrib_id >= 0 )
				{
					GL.EnableVertexAttribArray( GeometryShader.texture_attrib_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GeometryShader.texture_attrib_id, 2, VertexAttribPointerType.UnsignedShort, false, VertexSize, 16 );
					Display.CheckErr();
				}
				if( GeometryShader.use_texture_id >= 0 )
				{
					GL.EnableVertexAttribArray( GeometryShader.use_texture_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GeometryShader.use_texture_id, 1, VertexAttribPointerType.Byte, false, VertexSize, 20 );
					Display.CheckErr();
				}
				if( GeometryShader.flat_color_id >= 0 )
				{
					GL.EnableVertexAttribArray( GeometryShader.flat_color_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GeometryShader.flat_color_id, 1, VertexAttribPointerType.Byte, false, VertexSize, 21 );
					Display.CheckErr();
				}
				if( GeometryShader.decal_texture_id >= 0 )
				{
					GL.EnableVertexAttribArray( GeometryShader.flat_color_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GeometryShader.flat_color_id, 1, VertexAttribPointerType.Byte, false, VertexSize, 22 );
					Display.CheckErr();
				}
				if( GeometryShader.power_id >= 0 )
				{
					GL.EnableVertexAttribArray( GeometryShader.power_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GeometryShader.power_id, 1, VertexAttribPointerType.Short, false, VertexSize, 24 );
					Display.CheckErr();
				}
				if( GeometryShader.mod_attrib_id >= 0 )
				{
					GL.EnableVertexAttribArray( GeometryShader.mod_attrib_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GeometryShader.mod_attrib_id, 2, VertexAttribPointerType.UnsignedByte, false, VertexSize, 26 );
					Display.CheckErr();
				}
				if( GeometryShader.color_id >= 0 )
				{
					GL.EnableVertexAttribArray( GeometryShader.color_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GeometryShader.color_id, 4, VertexAttribPointerType.UnsignedByte, false, VertexSize, 28 );
					Display.CheckErr();
				}
				if( GeometryShader.face_color_id >= 0 )
				{
					GL.EnableVertexAttribArray( GeometryShader.face_color_id );
					Display.CheckErr();
					GL.VertexAttribPointer( GeometryShader.face_color_id, 4, VertexAttribPointerType.UnsignedByte, false, VertexSize, 32 );
					Display.CheckErr();
				}
#if USE_GLES2
				GL.Oes.BindVertexArray( 0 );
#else
				GL.BindVertexArray( 0 );
#endif
				GL.BindBuffer( BufferTarget.ArrayBuffer, 0 );
				Display.CheckErr();
			}

			if( !transparent && !custom )
			{
				if( solid_dirty )
				{
					GL.BindBuffer( BufferTarget.ArrayBuffer, vbo_solid );
					Display.CheckErr();
					unsafe
					{
						fixed ( float* data = &solid_buffer[0].p1 )
						{
							GL.BufferData( BufferTarget.ArrayBuffer, (IntPtr)( VertexSize * solid_used ), (IntPtr)data
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
			}
			else if( transparent && !custom )
			{
				if( transparent_dirty )
				{
					GL.BindBuffer( BufferTarget.ArrayBuffer, vbo_transparent );
					Display.CheckErr();
					unsafe
					{
						fixed ( float* data = &transparent_buffer[0].p1 )
						{
							GL.BufferData( BufferTarget.ArrayBuffer, (IntPtr)( VertexSize * transparent_used ), (IntPtr)data
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
			}
			else if( !transparent && custom )
			{
				if( custom_dirty )
				{
					GL.BindBuffer( BufferTarget.ArrayBuffer, vbo_custom );
					Display.CheckErr();
					unsafe
					{
						fixed ( float* data = &custom_buffer[0].p1 )
						{
							GL.BufferData( BufferTarget.ArrayBuffer, (IntPtr)( VertexSize * custom_used ), (IntPtr)data
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
			}
			GL.BindBuffer( BufferTarget.ArrayBuffer, 0 );

			//GL.
			//void glBindBuffer​(enum target, uint bufferName)
		}

		internal void DrawBuffer( bool transparent
				, bool custom
				, int atlas_id )
		{
			if( ( !transparent && custom ) && custom_used == 0 )
				return;
			if( ( transparent && !custom ) && transparent_used == 0 )
				return;
			if( ( !custom && !transparent ) && solid_used == 0 )
				return;

			GeometryShader.Activate();
			LoadBuffer( transparent, custom );
			Shader.BindTexture( 1, atlas_id );
			GL.Uniform1( GeometryShader.texture_id, 1 );
			Display.CheckErr();

			if( transparent )
			{
#if USE_GLES2
				GL.Oes.BindVertexArray( vao_transparent );
#else
				GL.BindVertexArray( vao_transparent );
#endif
				Display.CheckErr();
#if USE_GLES2
				GL.DrawArrays( BeginMode.Triangles, 0, transparent_used * 3 );
#else
				GL.DrawArrays( PrimitiveType.Triangles, 0, transparent_used * 3 );
#endif
				Display.CheckErr();
			}
			else if( custom )
			{
#if USE_GLES2
				GL.Oes.BindVertexArray( vao_custom );
#else
				GL.BindVertexArray( vao_custom );
#endif
				Display.CheckErr();
#if USE_GLES2
				GL.DrawArrays( BeginMode.Triangles, 0, custom_used * 3 );
#else
				GL.DrawArrays( PrimitiveType.Triangles, 0, custom_used * 3 );
#endif
				Display.CheckErr();
			}
			else 
			{
#if USE_GLES2
				GL.Oes.BindVertexArray( vao_solid );
#else
				GL.BindVertexArray( vao_solid );
#endif
				Display.CheckErr();
#if USE_GLES2
				GL.DrawArrays( BeginMode.Triangles, 0, solid_used * 3 );
#else
				GL.DrawArrays( PrimitiveType.Triangles, 0, solid_used * 3 );
#endif
				Display.CheckErr();
			}

#if USE_GLES2
			GL.Oes.BindVertexArray( 0 );
#else
			GL.BindVertexArray( 0 );
#endif
			Display.CheckErr();
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
