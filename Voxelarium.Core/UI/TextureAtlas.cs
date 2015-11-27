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
using OpenTK.Graphics.ES20;
#endif
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
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

		internal int OpenGl_TextureRef
		{
			get
			{
				if( _OpenGl_TextureRef == -1 )
				{
					Log.log( "Load atlas texture  (check thread)" );

					GL.ActiveTexture( TextureUnit.Texture0 );
					Display.CheckErr();

					GL.GenTextures( 1, out _OpenGl_TextureRef );
					Display.CheckErr();
					Shader.BindTexture( 0, _OpenGl_TextureRef );
					Display.CheckErr();
					// if (i & 1) glTexParameteri(GL_TEXTURE_2D, 0x84FE /*TEXTURE_MAX_ANISOTROPY_EXT*/, 8);
					//GL.TexParameterI( TextureTarget.Texture2D, TextureParameterName. 0x84FE /*TEXTURE_MAX_ANISOTROPY_EXT*/, 8 );
					BitmapData data = atlas.LockBits(
						new Rectangle( 0, 0, atlas.Width, atlas.Height )
						, System.Drawing.Imaging.ImageLockMode.ReadOnly
						, atlas.PixelFormat );
#if USE_GLES2
						GL.TexImage2D( TextureTarget2d.Texture2D, 0, TextureComponentCount.Rgba
							, data.Width, data.Height
							, 0, OpenTK.Graphics.ES20.PixelFormat.Rgba
							, PixelType.UnsignedByte
							, data.Scan0
							);
#else
					GL.TexImage2D( TextureTarget.Texture2D, 0, PixelInternalFormat.Rgba
						, data.Width, data.Height
						, 0, OpenTK.Graphics.OpenGL.PixelFormat.Bgra
						, PixelType.UnsignedByte
						, data.Scan0
						);
#endif
					Display.CheckErr();
					GL.TexParameter( TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.Nearest );
					Display.CheckErr();
					GL.TexParameter( TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Nearest );
					Display.CheckErr();

					atlas.UnlockBits( data );
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
			atlas = new Bitmap( needed_size, needed_size );
		}

		internal void AddTexture( Bitmap image, out Box2D coord )
		{
			if( y_ofs >= 32 )
			{
				coord.Position.X = 0;
				coord.Position.Y = 0;
				coord.Size.X = 0;
				coord.Size.Y = 0;
				return;
			}

			//Log.log( "output texture to atlas... {0} {1}", x_ofs, y_ofs );
			coord.Position.X = ( 65535.0f / texture_count ) * x_ofs;
			coord.Position.Y = ( 65535.0f / texture_count ) * y_ofs;
			coord.Size.X = ( 65535.0f / texture_count );
			coord.Size.Y = ( 65535.0f / texture_count );

			Graphics g = Graphics.FromImage( atlas );
			g.DrawImage( image, new Rectangle( texture_size * x_ofs, texture_size * y_ofs, texture_size, texture_size ) );
			g.Dispose();
			x_ofs++;
			if( x_ofs == texture_count )
			{
				x_ofs = 0;
				y_ofs++;
			}
		}
	}
}
