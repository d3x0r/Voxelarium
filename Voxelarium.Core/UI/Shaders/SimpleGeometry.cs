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

namespace Voxelarium.Core.UI.Shaders
{
	[StructLayout( LayoutKind.Sequential, Pack =1)]
    internal struct SimpleGeometryVertex {
		internal const int VertexSize = 4 + 3 * 4; /* 16 */

		internal byte cr, cg, cb, ca;
		/* 4 offset */
		internal float vx, vy, vz;
	}

	internal class SimpleGeometry : GeometryBuffer<SimpleGeometryVertex>
	{
		static SimpleVertShader VertShader = Display.simpleVertShader;

		internal SimpleGeometry( int InitialCapacity ) : base( InitialCapacity )
		{
		}
		internal SimpleGeometry() : this( 100 )
		{
		}

		internal static void DoSetupBuffer( IntPtr vertbuf )
		{
			GL.VertexAttribPointer( VertShader.vertex_attrib_id, 3
				, VertexAttribPointerType.Float, false, SimpleGeometryVertex.VertexSize, vertbuf + 4 );
			Display.CheckErr();
			GL.EnableVertexAttribArray( VertShader.vertex_attrib_id );
			Display.CheckErr();

			if( VertShader.color_attrib_id >= 0 )
			{
				GL.EnableVertexAttribArray( VertShader.color_attrib_id );
				Display.CheckErr();
				GL.VertexAttribPointer( VertShader.color_attrib_id, 4
					, VertexAttribPointerType.UnsignedByte, true, SimpleGeometryVertex.VertexSize, vertbuf );
				Display.CheckErr();
			}
		}

		protected override void SetupBuffer()
		{
			GL.VertexAttribPointer( VertShader.vertex_attrib_id, 3
				, VertexAttribPointerType.Float, false, SimpleGeometryVertex.VertexSize, 4 );
			Display.CheckErr();
			GL.EnableVertexAttribArray( VertShader.vertex_attrib_id );
			Display.CheckErr();

			if( VertShader.color_attrib_id >= 0 )
			{
				GL.EnableVertexAttribArray( VertShader.color_attrib_id );
				Display.CheckErr();
				GL.VertexAttribPointer( VertShader.color_attrib_id, 4
					, VertexAttribPointerType.UnsignedByte, true, SimpleGeometryVertex.VertexSize, 0 );
				Display.CheckErr();
			}
		}

		protected override void FillBuffer()
		{
			unsafe
			{
				fixed ( byte* data = &buffer[0].cr )
				{
					GL.BufferData( BufferTarget.ArrayBuffer, (IntPtr)( SimpleGeometryVertex.VertexSize * used ), (IntPtr)data
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
			VertShader.Activate();

			Display.CheckErr();
		}


		internal void AddLine( Vector3[] verts, ref Vector4 color )
		{
			int n;
			if( ( used + 2 ) >= available )
				expand();
			dirty = true;
			{
				buffer[used].cr = (byte)( 255 * color.X );
				buffer[used].cg = (byte)( 255 * color.Y );
				buffer[used].cb = (byte)( 255 * color.Z );
				buffer[used].ca = (byte)( 255 * color.W );
				buffer[used].vx = verts[0].X;
				buffer[used].vy = verts[0].Y;
				buffer[used].vz = verts[0].Z;
				used++;
				buffer[used].cr = (byte)( 255 * color.X );
				buffer[used].cg = (byte)( 255 * color.Y );
				buffer[used].cb = (byte)( 255 * color.Z );
				buffer[used].ca = (byte)( 255 * color.W );
				buffer[used].vx = verts[1].X;
				buffer[used].vy = verts[1].Y;
				buffer[used].vz = verts[1].Z;
				used++;
			}
		}

		internal void AddLine( ref Vector3 from, ref Vector3 to, Color colorFrom, Color colorTo )
		{
			int n;
			if( ( used + 2 ) >= available )
				expand();
			dirty = true;
			{
				buffer[used].cr = colorFrom.R;
				buffer[used].cg = colorFrom.G;
				buffer[used].cb = colorFrom.B;
				buffer[used].ca = colorFrom.A;
				buffer[used].vx = from.X;
				buffer[used].vy = from.Y;
				buffer[used].vz = from.Z;
				used++;
				buffer[used].cr = colorTo.R;
				buffer[used].cg = colorTo.G;
				buffer[used].cb = colorTo.B;
				buffer[used].ca = colorTo.A;
				buffer[used].vx = to.X;
				buffer[used].vy = to.Y;
				buffer[used].vz = to.Z;
				used++;
			}

		}
	}
}
