﻿/*
 * Before porting, this header appeared inmost sources.  Of course
 * the change from C++ to C# required significant changes an no part
 * is entirely original.
 * 
 * This file is part of Blackvoxel. (Now Voxelarium)
 *
 * Copyright 2010-2014 Laurent Thiebaut & Olivia Merle
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
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
#endif
using System.Text;
using Voxelarium.Common;
using Voxelarium.Core.Support;
using Voxelarium.Core.UI.Shaders;
using Voxelarium.Core.Voxels;

namespace Voxelarium.Core.UI
{
	internal enum TextureID
	{
		MainMenuBackground
			, TitleBanner
			, Panel
			, OldFont
			, TitleA
			, TitleB
			, OldGuiTiles
			, DialogBackground
			, DialogVoxelTypeBackground
			, Button
			, Title2
			, Title3
			, Contribute
	}
	internal class TextureManager : List<TextureManager.Texture_Entry>
	{
		internal class Texture_Entry
		{
			internal Bitmap Texture;
			internal bool LinearInterpolation;
			int _OpenGl_TextureRef;
			internal string FileName;
			bool dirty;
			bool invalidated;
			internal int OpenGl_TextureRef
			{
				get
				{
					if( invalidated ) {
						Log.log( "Texture has been invalidated... try reload" );
						_OpenGl_TextureRef = -1;
						invalidated = false;
					}
					if( _OpenGl_TextureRef == -1 ) {
						GL.ActiveTexture( TextureUnit.Texture0 );
						Display.CheckErr();

						GL.GenTextures( 1, out _OpenGl_TextureRef );
						Display.CheckErr();
						dirty = true;
					}
					if( dirty)
					{
						dirty = false;
						Voxelarium.Core.UI.Shaders.Shader.BindTexture( _OpenGl_TextureRef );
						Display.CheckErr();
						// if (i & 1) glTexParameteri(GL_TEXTURE_2D, 0x84FE /*TEXTURE_MAX_ANISOTROPY_EXT*/, 8);
						//GL.TexParameterI( TextureTarget.Texture2D, TextureParameterName. 0x84FE /*TEXTURE_MAX_ANISOTROPY_EXT*/, 8 );
#if USE_GLES2
						//Log.log( "Generate texture {0} {1} {2}", _OpenGl_TextureRef, All.Texture2D, TextureTarget.Texture2D );
						Android.Opengl.GLUtils.TexImage2D( (int)TextureTarget.Texture2D, 0, Texture, 0 );
#else
						BitmapData data = Texture.LockBits(
							new Rectangle( 0, 0, Texture.Width, Texture.Height )
							, System.Drawing.Imaging.ImageLockMode.ReadOnly
							, Texture.PixelFormat );
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
						if( LinearInterpolation )
						{
							int param = (int)TextureMinFilter.Nearest;
							GL.TexParameter( TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, param ); // GL_LINEAR GL_NEAREST
							param = (int)TextureMagFilter.Linear;
							GL.TexParameter( TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, param );
						}
						else
						{
							int param = (int)TextureMinFilter.Nearest;
							GL.TexParameter( TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, param ); // GL_LINEAR GL_NEAREST
							param = (int)TextureMagFilter.Nearest;
							GL.TexParameter( TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, param );
						}
#if !USE_GLES2
						Texture.UnlockBits( data );
#endif
					}
					return _OpenGl_TextureRef;
				}
			}

			internal Texture_Entry()
			{
				Texture = null;
				LinearInterpolation = false;
				_OpenGl_TextureRef = -1;
				Display.OnInvalidate += Display_OnInvalidate;
			}

			void Display_OnInvalidate ()
			{
				invalidated = true;
			}
			~Texture_Entry()
			{
				Texture = null;
				LinearInterpolation = false;
				if( !VoxelGlobalSettings.Exiting )
				{
					if( _OpenGl_TextureRef != -1 )
					{
						GL.DeleteTexture( _OpenGl_TextureRef );
						_OpenGl_TextureRef = -1;
					}
                }
			}
		};


		internal bool LoadTexture( string FileSpec, TextureID TextureNum, bool LinearInterpolation = true )
		{
			Bitmap NewImage;
			Texture_Entry Texture = new Texture_Entry();

			try
			{
				Texture.FileName = FileSpec;
				NewImage = Display.LoadBitmap( 0, FileSpec );
			}
			catch( Exception e )
			{
				Log.log( "Failed to load GUI texture: " + FileSpec + " : " + e.Message );
				return false;
			}

#if !USE_GLES2
			if( VoxelGlobalSettings.COMPILEOPTION_LOWRESTEXTURING )
				if( NewImage.Width > 128 )
				{
					Bitmap newImage = new Bitmap( 128, 128 );
					using( Graphics gr = Graphics.FromImage( newImage ) )
					{
						gr.SmoothingMode = SmoothingMode.HighQuality;
						gr.InterpolationMode = InterpolationMode.HighQualityBicubic;
						gr.PixelOffsetMode = PixelOffsetMode.HighQuality;
						gr.DrawImage( NewImage, new Rectangle( 0, 0, 128, 128 ) );
					}
					NewImage = newImage;
				}
#endif

			Texture.Texture = NewImage;
			Texture.LinearInterpolation = LinearInterpolation;
			while( Count < ( (int)TextureNum + 1 ) )
				this.Add( null );

			this[(int)TextureNum] = Texture;

			return ( true );
		}

		internal int GetTexture_Count()
		{
			return ( Count );
		}

		internal Texture_Entry GetTextureEntry( TextureID TextureNum )
		{
			return this[(int)TextureNum];
		}
	}
}
