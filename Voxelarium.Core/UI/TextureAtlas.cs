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
#if !USE_GLES2
using OpenTK.Graphics.OpenGL;
#else
using Android.Graphics;
using OpenTK.Graphics.ES20;
#endif
using System;
using System.Collections.Generic;
using System.Drawing;
// really this is IF_ANDROID_PORT
#if !USE_GLES2
using System.Drawing.Imaging;
#endif
using System.Text;
using Voxelarium.Common;
using Voxelarium.Core.Support;
using Voxelarium.Core.UI.Shaders;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.UI
{
	internal class TextureAtlas
	{
		int texture_count;
		int texture_size;
		Bitmap atlas;
		int x_ofs, y_ofs;

		int _OpenGl_TextureRef = -1;
		bool invalidated;

		internal int OpenGl_TextureRef
		{
			get
			{
				if( invalidated ) {
					if( _OpenGl_TextureRef != -1 )
						_OpenGl_TextureRef = -1;
				}
				if( _OpenGl_TextureRef == -1 ) {
					Log.log( "Load atlas texture  (check thread)" );

					GL.ActiveTexture( TextureUnit.Texture0 );
					Display.CheckErr();

					GL.GenTextures( 1, out _OpenGl_TextureRef );
					Display.CheckErr();

					int atlas_unit = Voxelarium.Core.UI.Shaders.Shader.BindTexture( _OpenGl_TextureRef );
					Display.CheckErr();

					// if (i & 1) glTexParameteri(GL_TEXTURE_2D, 0x84FE /*TEXTURE_MAX_ANISOTROPY_EXT*/, 8);
					//GL.TexParameterI( TextureTarget.Texture2D, TextureParameterName. 0x84FE /*TEXTURE_MAX_ANISOTROPY_EXT*/, 8 );
#if USE_GLES2
					//Log.log( " 2 Generate texture {0} {1} {2}", _OpenGl_TextureRef, All.Texture2D, TextureTarget.Texture2D );
					Android.Opengl.GLUtils.TexImage2D( (int)TextureTarget.Texture2D, 0, atlas, 0 );
#else
					BitmapData data = atlas.LockBits(
						new Rectangle( 0, 0, atlas.Width, atlas.Height )
						, System.Drawing.Imaging.ImageLockMode.ReadOnly
						, atlas.PixelFormat );
					GL.TexImage2D( TextureTarget.Texture2D, 0, PixelInternalFormat.Rgba
						, data.Width, data.Height
						, 0, OpenTK.Graphics.OpenGL.PixelFormat.Bgra
						, PixelType.UnsignedByte
						, data.Scan0
						);
#endif
					Display.CheckErr();
					GL.TexParameter( TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.Linear );
					Display.CheckErr();
					GL.TexParameter( TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Nearest );
					Display.CheckErr();
#if !USE_GLES2
					atlas.UnlockBits( data );
#endif
				}
				return _OpenGl_TextureRef;
			}
		}


		internal TextureAtlas( int count, int size )
		{
			int needed_size;
			texture_count = count;
			texture_size = size;
			if( ( Display.max_texture_size / texture_count ) > texture_size )
				needed_size = texture_count * texture_size;
			else
			{
				texture_size = Display.max_texture_size / texture_count;
				needed_size = Display.max_texture_size;
			}
#if USE_GLES2
			atlas = Bitmap.CreateBitmap( needed_size, needed_size, Bitmap.Config.Argb8888 );
#else
			atlas = new Bitmap( needed_size, needed_size );
#endif
			Display.OnInvalidate += Display_OnInvalidate;
		}

		void Display_OnInvalidate ()
		{
			invalidated = true;
		}

		internal void AddTexture( Bitmap image, out Box2D coord, out float[] uvs )
		{
			uvs = new float[8];
			if( y_ofs >= 32 )
			{
				for( int n = 0; n < 8; n++ )
					uvs[n] = 0;
				coord.Position.X = 0;
				coord.Position.Y = 0;
				coord.Size.X = 0;
				coord.Size.Y = 0;
				return;
			}
			float scalar = 32768 / texture_count;
			//Log.log( "output texture to atlas... {0} {1}", x_ofs, y_ofs );
			coord.Position.X = ( scalar ) * x_ofs;
			coord.Position.Y = ( scalar ) * y_ofs;
			coord.Size.X = ( scalar );
			coord.Size.Y = ( scalar );

			uvs[0 * 2 + 0] = ( scalar ) * x_ofs;
			uvs[0 * 2 + 1] = ( scalar ) * y_ofs;
			uvs[1 * 2 + 0] = uvs[0 * 2 + 0] + ( scalar );
			uvs[1 * 2 + 1] = uvs[0 * 2 + 1];
			uvs[2 * 2 + 0] = uvs[0 * 2 + 0];
			uvs[2 * 2 + 1] = uvs[0 * 2 + 1] + ( scalar );
			uvs[2 * 2 + 0] = uvs[1 * 2 + 0];
			uvs[2 * 2 + 1] = uvs[2 * 2 + 1];

#if USE_GLES2
			Canvas canvas = new Canvas( atlas );
			Paint paint = new Paint();
			canvas.DrawBitmap( image
				, new Rect( 0, 0, texture_size, texture_size )
				, new Rect( texture_size * x_ofs, texture_size * y_ofs, texture_size, texture_size )
				, paint
				);
#else

			Graphics g = Graphics.FromImage( atlas );
			g.DrawImage( image, new Rectangle( texture_size * x_ofs, texture_size * y_ofs, texture_size, texture_size ) );
			g.Dispose();
#endif
			x_ofs++;
			if( x_ofs == texture_count )
			{
				x_ofs = 0;
				y_ofs++;
			}
		}



	}
}
