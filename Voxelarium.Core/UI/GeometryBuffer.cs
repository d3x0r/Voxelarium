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

using System;
using Voxelarium.Core.Voxels;

namespace Voxelarium.Core.UI
{
	/// <summary>
	/// General utility for an expanding array of geometry data
	/// </summary>
	/// <typeparam name="VertexType">Type of the vertex for interleaved data</typeparam>
	internal abstract class GeometryBuffer<VertexType> : IDisposable where VertexType : struct
	{
		protected VertexType[] buffer;
		int vbo;
		int vao;
		protected bool dirty;
		protected int used;
		protected int available;

		// this is the last direction that the transparent render was sorted.
		internal VoxelSector.RelativeVoxelOrds transparent_render_sorting;
		internal int sortedX, sortedY, sortedZ;

		internal GeometryBuffer( int InitialCapacity )
		{
			Display.OnInvalidate += Display_OnInvalidate;
			vbo = -1;
			vao = -1;
		
			available = InitialCapacity;
			buffer = new VertexType[available];
		}
		internal GeometryBuffer() : this( 100 )
		{
		}

		void Display_OnInvalidate ()
		{
			vbo = -1;
			vao = -1;
		}

		public void Dispose()
		{
			buffer = null;
		}

		public void Clear()
		{
			used = 0;
		}

		protected abstract void SetupBuffer();
		protected abstract void FillBuffer();

		void LoadBuffer()
		{
			bool setup_array = false;

			{
				if( used == 0 )
					return;
				if( vbo == -1 )
#if USE_GLES2
					GL.GenBuffers( 1, out vbo );
#else
					vbo = GL.GenBuffer();
#endif
				if( vao == -1 )
				{
#if USE_GLES3
					GL.GenVertexArrays();
#elif !USE_GLES2
					vao = GL.GenVertexArray();
#else
					GL.Oes.GenVertexArrays( 1, out vao);
#endif

#if USE_GLES2
					GL.Oes.BindVertexArray( vao );
#else
					GL.BindVertexArray( vao );
#endif
					Display.CheckErr();
					GL.BindBuffer( BufferTarget.ArrayBuffer, vbo );
					Display.CheckErr();
					setup_array = true;

					dirty = true;
				}
			}

			if( setup_array )
			{
				SetupBuffer();
#if USE_GLES2
				GL.Oes.BindVertexArray( 0 );
#else
				GL.BindVertexArray( 0 );
#endif
				GL.BindBuffer( BufferTarget.ArrayBuffer, 0 );
				Display.CheckErr();
			}

			if( dirty && used > 0 )
			{
				GL.BindBuffer( BufferTarget.ArrayBuffer, vbo );
				Display.CheckErr();
				FillBuffer();
				GL.BindBuffer( BufferTarget.ArrayBuffer, 0 );
			}

			//GL.
			//void glBindBuffer​(enum target, uint bufferName)
		}

		protected abstract void ShaderActivate();

		internal void DrawBuffer()
		{
			if( used == 0 )
				return;

			ShaderActivate();
			// setup and possibly re-load data into buffer...
			// handles dirty buffer reload
			LoadBuffer();

			{
#if USE_GLES2
				GL.Oes.BindVertexArray( vao );
#else
				GL.BindVertexArray( vao );
#endif
				Display.CheckErr();
#if USE_GLES2
				GL.DrawArrays( BeginMode.Triangles, 0, used * 3 );
#else
				GL.DrawArrays( PrimitiveType.Triangles, 0, used * 3 );
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

		protected void expand()
		{
			int new_count = available * 2;
			VertexType[] new_buffer = new VertexType[new_count];
			for( int n = 0; n < used; n++ )
			{
				new_buffer[n] = buffer[n];
			}
			available = new_count;
			buffer = new_buffer;
		}

		internal void Reset()
		{
			dirty = true;
			used = 0;
		}

	}
}
