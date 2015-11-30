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
using Voxelarium.Common;


#if !USE_GLES2
using OpenTK.Graphics.OpenGL;
using System.Drawing.Imaging;
#else
using Android.Graphics;
using OpenTK.Graphics.ES20;
#endif
using OpenTK;

using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Text;
using TrueTypeSharp;
using Voxelarium.Core.UI.Shaders;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.UI
{
	public class FontRenderer
	{
		public class FontCharacter
		{
			internal int width, height;
			internal int xOffset, yOffset;
			internal int x, y; // x/y pixel offset on bitmap
			public short u, v, uw, vh;
			internal float[] uvs;
			internal int advanceWidth, leftSideBearing, topSideBearing;
		}

		Rectangle fontrect;
		Bitmap fontmap;
		int line_height; // standard spacing for lines.
		int ascent, descent;  // otherwise is baseline...

		int _OpenGl_TextureRef = -1;
		bool dirty;
		bool invalidated;
		TrueTypeFont ttf;
		Dictionary<uint, FontCharacter> charmap = new Dictionary<uint, FontCharacter>();
		List<int> line_heights = new List<int>();
		List<int> line_offsets = new List<int>();
		float scalex, scaley;

		public int OpenGl_TextureRef
		{
			get
			{
				if( invalidated ) {
					invalidated = false;
					_OpenGl_TextureRef = -1;
				}
				if( _OpenGl_TextureRef == -1 )
				{
					GL.ActiveTexture( TextureUnit.Texture0 );
					Display.CheckErr();

					GL.GenTextures( 1, out _OpenGl_TextureRef );
					Display.CheckErr();
					dirty = true;
				}
				if( dirty )
				{
					GL.ActiveTexture( TextureUnit.Texture0 );
					Display.CheckErr();

					Voxelarium.Core.UI.Shaders.Shader.BindTexture( 0, _OpenGl_TextureRef );
					Display.CheckErr();
					// if (i & 1) glTexParameteri(GL_TEXTURE_2D, 0x84FE /*TEXTURE_MAX_ANISOTROPY_EXT*/, 8);
					//GL.TexParameterI( TextureTarget.Texture2D, TextureParameterName. 0x84FE /*TEXTURE_MAX_ANISOTROPY_EXT*/, 8 );
#if USE_GLES2
					//Log.log( "0 Generate texture {0} {1} {2}", _OpenGl_TextureRef, All.Texture2D, TextureTarget.Texture2D );
					Android.Opengl.GLUtils.TexImage2D( (int)TextureTarget.Texture2D, 0, fontmap, 0 );
#else
					BitmapData data = fontmap.LockBits(
						fontrect
						, System.Drawing.Imaging.ImageLockMode.ReadOnly
						, fontmap.PixelFormat );
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

					int param = (int)TextureMinFilter.Linear;// Nearest;
					GL.TexParameter( TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, param ); // GL_LINEAR GL_NEAREST
					param = (int)TextureMagFilter.Linear;
					GL.TexParameter( TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, param );

#if !USE_GLES2
					fontmap.UnlockBits( data );
#endif
					dirty = false;
				}
				return _OpenGl_TextureRef;
			}
		}

		void ExpandMap()
		{
#if USE_GLES2
			Bitmap newmap = Bitmap.CreateBitmap( fontmap.Width * 2, fontmap.Height * 2, Bitmap.Config.Argb8888 );
			Canvas canvas = new Canvas( newmap );
			canvas.DrawBitmap( fontmap, 0, 0, new Paint() );
			canvas.Dispose();
#else
			Bitmap newmap = new Bitmap( fontmap.Width * 2, fontmap.Height * 2 );

			Graphics g = Graphics.FromImage( newmap );
			g.DrawImage( fontmap, 0, 0, fontrect, GraphicsUnit.Pixel );
			g.Dispose();
#endif

			fontmap = newmap;
			fontrect = new Rectangle( 0, 0, fontmap.Width, fontmap.Height );

			foreach( KeyValuePair<uint, FontCharacter> chp in charmap )
			{
				FontCharacter ch = chp.Value;
				ch.u = (short)( 65536 * ch.x / fontrect.Width );
				ch.v = (short)( 65536 * ch.y / fontrect.Height );
				ch.uw = (short)( 65536 * ( ch.x + ch.width ) / fontrect.Width );
				ch.vh = (short)( 65536 * ( ch.y + ch.height ) / fontrect.Height );
				ch.uvs[0 * 2 + 0] = (float)ch.x / fontrect.Width + (float)ch.width / fontrect.Width;
				ch.uvs[0 * 2 + 1] = (float)ch.y / fontrect.Height + (float)ch.height / fontrect.Height;
				ch.uvs[1 * 2 + 0] = (float)ch.x / fontrect.Width + (float)ch.width / fontrect.Width;
				ch.uvs[1 * 2 + 1] = (float)ch.y / fontrect.Height;
				ch.uvs[2 * 2 + 0] = (float)ch.x / fontrect.Width;
				ch.uvs[2 * 2 + 1] = (float)ch.y / fontrect.Height + (float)ch.height / fontrect.Height;
				ch.uvs[3 * 2 + 0] = (float)ch.x / fontrect.Width;
				ch.uvs[3 * 2 + 1] = (float)ch.y / fontrect.Height;
			}

		}

		void FindSpot( FontCharacter fc )
		{
			int width = fc.width;
			int height = fc.height;
			int row_start = 0;
			int row;
			for( row = 0; row < line_heights.Count; row++ )
			{
				int h = line_heights[row];
				if( h == -1 )
				{
					line_heights[row] = height+1;
					line_heights.Add( -1 );
					line_offsets.Add( 0 );
					break;
				}
				if( (height+1) <= h )
				{
					if( width <= ( fontmap.Width - line_offsets[row] ) )
						break;
				}
				row_start += h;
			}
			if( row_start + height > fontmap.Height )
			{
				ExpandMap();
			}
			fc.x = line_offsets[row];
			fc.y = row_start;
			line_offsets[row] += width+1;

			fc.u = (short)( 65536 * fc.x / fontrect.Width );
			fc.v = (short)( 65536 * fc.y / fontrect.Height );
			fc.uw = (short)( 65536 * fc.width / fontrect.Width );
			fc.vh = (short)( 65536 * fc.height / fontrect.Height );
			fc.uvs = new float[8];
			fc.uvs[2 * 2 + 0] = (float)fc.x / fontrect.Width + (float)fc.width / fontrect.Width;
			fc.uvs[2 * 2 + 1] = (float)fc.y / fontrect.Height + (float)fc.height / fontrect.Height;
			fc.uvs[3 * 2 + 0] = (float)fc.x / fontrect.Width + (float)fc.width / fontrect.Width;
			fc.uvs[3 * 2 + 1] = (float)fc.y / fontrect.Height;
			fc.uvs[0 * 2 + 0] = (float)fc.x / fontrect.Width;
			fc.uvs[0 * 2 + 1] = (float)fc.y / fontrect.Height + (float)fc.height / fontrect.Height;
			fc.uvs[1 * 2 + 0] = (float)fc.x / fontrect.Width;
			fc.uvs[1 * 2 + 1] = (float)fc.y / fontrect.Height;
		}

		public FontRenderer( string font, int width, int height )
		{
			string tmp;
			int location;
			if( Display.FileExists( font, out location ) ) {
				ttf = new TrueTypeFont( Display.FileReadAllBytes( location, font ), 0 );
			}
			else if( Display.FileExists( tmp = ("c:/windows/fonts/" + font), out location ) )
				ttf = new TrueTypeFont( Display.FileReadAllBytes( location, tmp ), 0 );
			else if( Display.FileExists( tmp = ("Content/fonts/" + font), out location ) )
				ttf = new TrueTypeFont( Display.FileReadAllBytes( location, tmp ), 0 );
			else if( Display.FileExists( tmp = ( "/usr/share/fonts/TTF/" + font ), out location ) )
				ttf = new TrueTypeFont( Display.FileReadAllBytes( location, tmp ), 0 );
			else
				throw new Exception( "Font not found:" + font );
			scalex = ttf.GetScaleForPixelHeight( width );
			scaley = ttf.GetScaleForPixelHeight( height );

			int lineGap;
			ttf.GetFontVMetrics( out ascent, out descent, out lineGap );
			line_height = (int)Math.Ceiling( ( lineGap + ( ascent - descent ) ) * scaley );
			ascent = (int)( ascent * scaley );
			descent = (int)( descent * scaley );

			fontrect = new Rectangle( 0, 0, 256, 256 );
#if !USE_GLES2
			fontmap = new Bitmap( 256, 256 );
			Graphics g = Graphics.FromImage( fontmap );
			Brush b = new SolidBrush( Color.FromArgb( 0 ) );
			g.FillRectangle( b, fontrect );
			g.Dispose();
#else
			fontmap = Bitmap.CreateBitmap( 256, 256, Bitmap.Config.Argb8888 );
			Canvas canvas = new Canvas( fontmap );
			canvas.DrawColor( Android.Graphics.Color.Transparent );
			canvas.Dispose();
#endif
			charmap = new Dictionary<uint, FontCharacter>();

			line_heights = new List<int>();
			line_heights.Add( -1 );
			line_offsets = new List<int>();
			line_offsets.Add( 0 );

			Display.OnInvalidate += Display_OnInvalidate;
		}

		void Display_OnInvalidate ()
		{
			invalidated = true;
		}

		public FontCharacter GetCharacter( uint code )
		{
			uint index = ttf.FindGlyphIndex( code );
			FontCharacter c;
			if( !charmap.TryGetValue( index, out c ) )
			{
				c = new FontCharacter();
				ttf.GetGlyphHMetrics( index, out c.advanceWidth, out c.leftSideBearing );
				c.advanceWidth = (int)Math.Ceiling(c.advanceWidth* scalex );
				c.leftSideBearing = (int)( c.leftSideBearing * scalex );
				int a, b, cv, d;
				ttf.GetGlyphBox( index, out a, out c.topSideBearing, out cv, out d );
				c.topSideBearing = (int)( c.topSideBearing * scaley );
                byte[] data = ttf.GetGlyphBitmap( index, scalex, scaley,
					out c.width, out c.height, out c.xOffset, out c.yOffset );

				//static void SaveBitmap( byte[] data, int x0, int y0, int x1, int y1,
				//int stride, string filename )
				{
					if( c.width == 0 || c.height == 0 )
					{
						charmap.Add( index, c );
						return c;
					}
					//Color WHITE = Color.White;
					FindSpot( c );
					//BitmapData bm_data = fontmap.LockBits( fontrect, System.Drawing.Imaging.ImageLockMode.ReadWrite, fontmap.PixelFormat );
					for( int by = 0; by < c.height; by++ )
					{
						for( int bx = 0; bx < c.width; bx++ )
						{
							byte opacity = data[by * c.width + bx];
							fontmap.SetPixel( c.x + bx, c.y + by
#if USE_GLES2
								, Android.Graphics.Color.Argb( opacity, 0xFF, 0xFF, 0xFF )
#else
								, Color.FromArgb( opacity, 0xFF, 0xFF, 0xFF ) 
#endif
								);
							//bm_data.Scan0.bitmap.SetPixel( x - x0, y - y0, Color.FromArgb( opacity, 0x00, 0x00, 0x00 ) );
						}
					}
					//fontmap.UnlockBits( bm_data );
				}
				dirty = true;
				charmap.Add( index, c );
			}
			return c;
		}

		uint GetUnicode( ref int c, string text )
		{
			uint rune;
			char ch = text[c];

			if( ( ( ch & 0xFC00 ) >= 0xD800 ) &&
				( ( ch & 0xFC00 ) <= 0xDF00 ) )
			{
				rune = 0x10000U + ( ( ch & 0x3FFU ) << 10 ) + ( text[c + 1] & 0x3FFU );
				c++;
			}
			else
				rune = ch;

			return rune;
		}

		public void GetFontRenderSize( string text, out Vector2 size )
		{
			int c;
			char ch;
			uint rune;
			int line_width = 0;
			size.X = 0;
			size.Y = line_height;
			for( c = 0; c < text.Length; c++ )
			{
				rune = GetUnicode( ref c, text );
				if( rune == '\n' )
				{
					size.Y += line_height;
					if( line_width > size.X )
						size.X = line_width;
					line_width = 0;
				}
				else
				{
					FontCharacter fontchar = GetCharacter( rune );
					line_width += fontchar.advanceWidth;
				}
			}
			if( line_width > size.X )
				size.X = line_width;
		}


		internal void RenderFont( Display render, ref Box EffectivePosition
			, float scale, string text
			, ref Vector4 color )
		{
			int c;
			uint rune;
			GL.Enable( EnableCap.Blend );
			GL.BlendFunc( BlendingFactorSrc.SrcAlpha, BlendingFactorDest.OneMinusSrcAlpha );
			render.simple_gui_texture.Activate();
			render.simple_gui_texture.SetUniforms( OpenGl_TextureRef, ref color );

			float[] quad = new float[12];
			float[] uvs = new float[8];
			float x_left, y_top;
			float x = EffectivePosition.Position.X;
			float y = EffectivePosition.Position.Y;
			float z = EffectivePosition.Position.Z + EffectivePosition.Size.Z;
			float xp, yp;
			FontCharacter ch;
            for( c = 0; c < text.Length; c++ )
			{
				rune = GetUnicode( ref c, text );
				if( rune == '\n' )
					y++;
				else
				{
					ch = GetCharacter( rune );
					if( ch.width > 0 )
					{
						x_left = x + ( ch.leftSideBearing * scale / line_height );
						y_top = y + ( ch.topSideBearing * scale / line_height );//( ( ( line_height + ch.yOffset ) * scale ) / line_height );
						xp = x_left + ( ch.width * scale / line_height );
						yp = y_top + ( ch.height * scale / line_height );

						quad[0*3+0] = x_left;
						quad[0*3+1] = y_top;
						quad[0 * 3 + 2] = z;
						quad[1 * 3 + 0] = x_left;
						quad[1 * 3 + 1] = yp;
						quad[1 * 3 + 2] = z;
						quad[2 * 3 + 0] = xp;
						quad[2 * 3 + 1] = y_top;
						quad[2 * 3 + 2] = z;
						quad[3 * 3 + 0] = xp;
						quad[3 * 3 + 1] = yp;
						quad[3 * 3 + 2] = z;

						render.simple_gui_texture.DrawQuad( quad, ch.uvs );
					}
					x = x + ( ch.advanceWidth * scale / line_height );
				}
			}
		}

		void RenderFont( Display render, ref Vector3 normal, ref Vector3 anchor, string text
			, float scale
			, ref Vector4 color )
		{
			int c;
			uint rune;
			for( c = 0; c < text.Length; c++ )
			{
				rune = GetUnicode( ref c, text );

			}
		}
	}
}
