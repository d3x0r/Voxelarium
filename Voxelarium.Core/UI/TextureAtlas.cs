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
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.UI
{
	internal class TextureAtlas
	{
		int texture_count;
		int texture_size;
		Bitmap atlas;
		int size;
		int x_ofs, y_ofs;

		int _OpenGl_TextureRef;

		internal int OpenGl_TextureRef
		{
			get
			{
				if( _OpenGl_TextureRef == -1 )
				{
					GL.ActiveTexture( TextureUnit.Texture0 );
					Display.CheckErr();

					GL.GenTextures( 1, out _OpenGl_TextureRef );
					Display.CheckErr();
					GL.BindTexture( TextureTarget.Texture2D, _OpenGl_TextureRef );
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
					GL.TexParameter( TextureTarget.Texture2D
						, TextureParameterName.TextureMinFilter
						, (int)TextureMinFilter.Linear );
					int param = (int)TextureMinFilter.Nearest;
					GL.TexParameter( TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, param ); // GL_LINEAR GL_NEAREST
					param = (int)TextureMagFilter.Nearest;
					GL.TexParameter( TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, param );
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

		void AddTexture( Bitmap image, out Box2D coord )
		{
			coord.Position.X = ( 1.0f / texture_count ) * x_ofs;
			coord.Position.Y = ( 1.0f / texture_count ) * y_ofs;
			coord.Size.X = ( 1.0f / texture_count );
			coord.Size.Y = ( 1.0f / texture_count );

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
