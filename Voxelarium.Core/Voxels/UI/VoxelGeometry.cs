/*
 * 
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

using OpenTK;
using System;
using System.Drawing;
using Voxelarium.Core.UI;
using Voxelarium.Core.UI.Shaders;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.Voxels.UI
{

	internal struct VoxelVertex
	{
		internal const int VertexSize = 36; // 4*4 * 5
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


	internal class VoxelGeometry : GeometryBuffer<VoxelVertex>, IDisposable
	{

		static VoxelGeometryShader GeometryShader = new VoxelGeometryShader();

		internal VoxelGeometry( int InitialCapacity ) : base( InitialCapacity )
		{
		}
		internal VoxelGeometry() : this( 100 )
		{
		}

		protected override void SetupBuffer()
		{
			GL.VertexAttribPointer( GeometryShader.vertex_attrib_id, 3, VertexAttribPointerType.Float, false, VoxelVertex.VertexSize, 0 );
			Display.CheckErr();
			GL.EnableVertexAttribArray( GeometryShader.vertex_attrib_id );
			Display.CheckErr();
			if( GeometryShader.texture_attrib_id >= 0 )
			{
				GL.EnableVertexAttribArray( GeometryShader.texture_attrib_id );
				Display.CheckErr();
				GL.VertexAttribPointer( GeometryShader.texture_attrib_id, 2, VertexAttribPointerType.UnsignedShort, false, VoxelVertex.VertexSize, 16 );
				Display.CheckErr();
			}
			if( GeometryShader.use_texture_id >= 0 )
			{
				GL.EnableVertexAttribArray( GeometryShader.use_texture_id );
				Display.CheckErr();
				GL.VertexAttribPointer( GeometryShader.use_texture_id, 1, VertexAttribPointerType.Byte, false, VoxelVertex.VertexSize, 20 );
				Display.CheckErr();
			}
			if( GeometryShader.flat_color_id >= 0 )
			{
				GL.EnableVertexAttribArray( GeometryShader.flat_color_id );
				Display.CheckErr();
				GL.VertexAttribPointer( GeometryShader.flat_color_id, 1, VertexAttribPointerType.Byte, false, VoxelVertex.VertexSize, 21 );
				Display.CheckErr();
			}
			if( GeometryShader.decal_texture_id >= 0 )
			{
				GL.EnableVertexAttribArray( GeometryShader.flat_color_id );
				Display.CheckErr();
				GL.VertexAttribPointer( GeometryShader.flat_color_id, 1, VertexAttribPointerType.Byte, false, VoxelVertex.VertexSize, 22 );
				Display.CheckErr();
			}
			if( GeometryShader.power_id >= 0 )
			{
				GL.EnableVertexAttribArray( GeometryShader.power_id );
				Display.CheckErr();
				GL.VertexAttribPointer( GeometryShader.power_id, 1, VertexAttribPointerType.Short, false, VoxelVertex.VertexSize, 24 );
				Display.CheckErr();
			}
			if( GeometryShader.mod_attrib_id >= 0 )
			{
				GL.EnableVertexAttribArray( GeometryShader.mod_attrib_id );
				Display.CheckErr();
				GL.VertexAttribPointer( GeometryShader.mod_attrib_id, 2, VertexAttribPointerType.UnsignedByte, false, VoxelVertex.VertexSize, 26 );
				Display.CheckErr();
			}
			if( GeometryShader.color_id >= 0 )
			{
				GL.EnableVertexAttribArray( GeometryShader.color_id );
				Display.CheckErr();
				GL.VertexAttribPointer( GeometryShader.color_id, 4, VertexAttribPointerType.UnsignedByte, false, VoxelVertex.VertexSize, 28 );
				Display.CheckErr();
			}
			if( GeometryShader.face_color_id >= 0 )
			{
				GL.EnableVertexAttribArray( GeometryShader.face_color_id );
				Display.CheckErr();
				GL.VertexAttribPointer( GeometryShader.face_color_id, 4, VertexAttribPointerType.UnsignedByte, false, VoxelVertex.VertexSize, 32 );
				Display.CheckErr();
			}
		}

		protected override void FillBuffer()
		{
			unsafe
			{
				fixed ( float* data = &buffer[0].p1 )
				{
					GL.BufferData( BufferTarget.ArrayBuffer, (IntPtr)( VoxelVertex.VertexSize * used ), (IntPtr)data
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


		internal void SetupUniforms( int atlas_id )
		{
			Shader.BindTexture( 1, atlas_id );
		}

		protected override void ShaderActivate()
		{
			GeometryShader.Activate();

			GL.Uniform1( GeometryShader.texture_id, 1 );
			Display.CheckErr();
		}

		internal void AddTriangle( float[] tri_points, byte[] color )
		{
			int n;
			int b;
			int point_base;
			if( used == available )
				expand();
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
				buffer[used].c1r = ( color[point_base + 0] );
				buffer[used].c1g = ( color[point_base + 1] );
				buffer[used].c1b = ( color[point_base + 2] );
				buffer[used].c1a = ( color[point_base + 3]);
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

		internal void AddQuad( ref Vector3 P1, ref Vector3 P2, ref Vector3 P3, ref Vector3 P4
		                     , Color face, Color edge, short power )
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

		internal void AddSimpleColorQuad( float[] quad_points, byte[] color )
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

				buffer[used].c1r = (color[0] );
				buffer[used].c1g = ( color[1]);
				buffer[used].c1b = ( color[2] );
				buffer[used].c1a = ( color[3]);
			}
		}

	}
}
