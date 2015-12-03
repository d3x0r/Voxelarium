/*
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
using OpenTK;
using System.Collections.Generic;
using System.Drawing;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.UI
{
	class TileSet 
	{
		internal class TileStyle 
		{
			public TileSet TileSet;
			public float HSizeFactor;
			public float VSizeFactor;
			public float Interligne_sup;
			public float CharSpacing_Sup;
		};

		internal class TileSetStyles : List<TileStyle>
		{

			internal void CreateStyle( int StyleNum, TileSet TileSet, float HSizeFactor, float VSizeFactor, float Interligne_sup = 0.0f, float CharSpacing_Sup = 0.0f )
			{
				TileStyle Style;

				Style = new TileStyle();

				Style.TileSet = TileSet;
				Style.HSizeFactor = HSizeFactor;
				Style.VSizeFactor = VSizeFactor;
				Style.Interligne_sup = Interligne_sup;
				Style.CharSpacing_Sup = CharSpacing_Sup;
				while( Count < StyleNum + 1 )
					Add( null );
				this[StyleNum] = Style;
			}

			internal TileStyle GetStyle( int StyleNum ) { return this[StyleNum]; }

		};

		Vector4 DefaultDrawColor;

		internal struct TileCoord
		{
			internal float TopLeft_x;
			internal float TopLeft_y;
			internal float BottomRight_x;
			internal float BottomRight_y;
			internal float Tile_Width;
			internal float Tile_Height;
			internal float[] texture;
		};

		public TextureManager TextureManager;
		public List<TileCoord> CoordTable;
		public TextureID TextureNum;

		//
		int Texture_Width, Texture_Height;
		float TileSlot_Width, TileSlot_Height;
		float Tile_Width, Tile_Height;
		int TileOffset_x, TileOffset_y;
		int TilesPerLine;

		internal TileSet()
		{
			CoordTable = new List<TileCoord>( 256 );
			TextureNum = 0;

			Texture_Width = 128;
			Texture_Height = 128;
			TileSlot_Width = 8;
			TileSlot_Height = 8;
			Tile_Width = 8;
			Tile_Height = 8;
			TileOffset_x = 0;
			TileOffset_y = 0;
			TilesPerLine = 16;
			DefaultDrawColor = new Vector4( 1 );
		}
		~TileSet()
		{
			CoordTable = null;
		}

		internal void SetTextureManager( TextureManager TextureManager ) { this.TextureManager = TextureManager; }
		internal void SetTextureNum( TextureID TextureNum ) { this.TextureNum = TextureNum; }


		// Prepare tiling
		internal void SetTextureSize( int Width, int Height ) { Texture_Width = Width; Texture_Height = Height; }
		internal void SetTileSlotSize( float Width, float Height ) { TileSlot_Width = Width; TileSlot_Height = Height; }
		internal void SetTileSize( float Width, float Height ) { Tile_Width = Width; Tile_Height = Height; }
		internal void SetTilesPerLine( int TilesPerLine ) { this.TilesPerLine = TilesPerLine; }
		internal void SetTileOffset( int x, int y ) { TileOffset_x = x; TileOffset_y = y; }
		internal void ComputeTileCoords()
		{
			int i;
			float x1, y1, x2, y2;

			for( i = 0; i < 256; i++ )
			{
				x1 = ( i % TilesPerLine ) * TileSlot_Width + TileOffset_x;
				y1 = ( i / TilesPerLine ) * TileSlot_Height + TileOffset_y;
				x2 = x1 + TileSlot_Width;
				y2 = y1 + TileSlot_Height;
				TileCoord tmp;
				tmp.TopLeft_x = (float)x1 / (float)Texture_Width;
				tmp.TopLeft_y = (float)y1 / (float)Texture_Height;
				tmp.BottomRight_x = (float)x2 / (float)Texture_Width;
				tmp.BottomRight_y = (float)y2 / (float)Texture_Height;
				tmp.Tile_Width = (float)Tile_Width;
				tmp.Tile_Height = (float)Tile_Height;
				tmp.texture = new float[8];
				tmp.texture[0 * 2 + 0] = tmp.TopLeft_x;
				tmp.texture[0 * 2 + 1] = tmp.BottomRight_y;
				tmp.texture[1 * 2 + 0] = tmp.TopLeft_x;
				tmp.texture[1 * 2 + 1] = tmp.TopLeft_y;
				tmp.texture[2 * 2 + 0] = tmp.BottomRight_x;
				tmp.texture[2 * 2 + 1] = tmp.BottomRight_y;
				tmp.texture[3 * 2 + 0] = tmp.BottomRight_x;
				tmp.texture[3 * 2 + 1] = tmp.TopLeft_y;
				while( CoordTable.Count < ( i + 1 ) )
					CoordTable.Add( tmp );
				CoordTable[i] = tmp;
			}
		}


#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal TileCoord GetTileCoord( int TileNum ) { return ( CoordTable[TileNum] ); }

		internal float GetTileWidth( int TileNum ) { return ( CoordTable[TileNum].Tile_Width ); }
		internal float GetTileHeight( int TileNum ) { return ( CoordTable[TileNum].Tile_Height ); }

		float[] position = new float[4 * 3];

		internal void RenderTile( Display render, ref Vector3 TopLeft, ref Vector3 BottomRight, int TileNum, ref Vector4 Color )
		{
			TileCoord Coord;

			Coord = CoordTable[TileNum];

			int textureRef = TextureManager.GetTextureEntry( TextureNum ).OpenGl_TextureRef;
			//glBindTexture(GL_TEXTURE_2D,TextureManager.GetTextureEntry(TextureNum).OpenGl_TextureRef[psvInit] );
			position[0 * 3 + 0] = TopLeft.X;
			position[0 * 3 + 1] = TopLeft.Y;
			position[0 * 3 + 2] = TopLeft.Z;
			position[1 * 3 + 0] = TopLeft.X;
			position[1 * 3 + 1] = BottomRight.Y;
			position[1 * 3 + 2] = TopLeft.Z;
			position[2 * 3 + 0] = BottomRight.X;
			position[2 * 3 + 1] = TopLeft.Y;
			position[2 * 3 + 2] = TopLeft.Z;
			position[3 * 3 + 0] = BottomRight.X;
			position[3 * 3 + 1] = BottomRight.Y;
			position[3 * 3 + 2] = TopLeft.Z;

			render.simple_texture.Activate();
			render.simple_texture.SetUniforms( textureRef, ref Color );
			render.simple_texture.DrawQuad( position, Coord.texture );

			render.simple_gui_texture.Activate();
			render.simple_gui_texture.SetUniforms( textureRef, ref Color );
			render.simple_gui_texture.DrawQuad( position, Coord.texture);
		}

		internal void RenderTile( Display render, Vector3 TopLeft, Vector3 BottomRight, int TileNum )
		{
			RenderTile( render, ref TopLeft, ref BottomRight, TileNum, ref DefaultDrawColor );
		}

		internal void RenderFont( Display render, TileStyle TileStyle, ref Box DrawBox
					, string TextToRender, ref Vector4 Color )
		{
			float x, y, xp, yp, DimX, DimY, LimitX;// LimitY;


			int i;
			TileSet TileSet;
			TileCoord Coord;
			int c;

			TileSet = TileStyle.TileSet;
			x = DrawBox.Position.X;
			y = DrawBox.Position.Y;
			LimitX = x + DrawBox.Size.X;
			//LimitY = y + DrawBox.Height;
			int TextureRef = TextureManager.GetTextureEntry( TileSet.TextureNum ).OpenGl_TextureRef;
			float z = DrawBox.Position.Z + DrawBox.Size.Z;
			render.simple_gui_texture.Activate();
			//glBindTexture(GL_TEXTURE_2D,TextureManager.GetTextureEntry(TileSet.TextureNum).OpenGl_TextureRef[psvInit] );
			//glColor3f(Color.r, Color.v, Color.b);
			for( i = 0; i < TextToRender.Length; i++ )
			{
				c = (byte)( TextToRender[i] );
				Coord = TileSet.CoordTable[c];
				DimX = Coord.Tile_Width * TileStyle.HSizeFactor;
				DimY = Coord.Tile_Height * TileStyle.VSizeFactor;
				//lprintf( "%g %g", TileStyle.HSizeFactor, TileStyle.VSizeFactor );
				xp = x + DimX;
				yp = y + DimY;
				if( xp > LimitX )
				{
					x = DrawBox.Size.X;
					y += DimY + TileStyle.Interligne_sup;
					xp = x + DimX;
					yp = y + DimY;
				}
				{
					position[0 * 3 + 0] = x;
					position[0 * 3 + 1] = y;
					position[0 * 3 + 2] = z;
					position[1 * 3 + 0] = x;
					position[1 * 3 + 1] = yp;
					position[1 * 3 + 2] = z;
					position[2 * 3 + 0] = xp;
					position[2 * 3 + 1] = y;
					position[2 * 3 + 2] = z;
					position[3 * 3 + 0] = xp;
					position[3 * 3 + 1] = yp;
					position[3 * 3 + 2] = z;
					//	lprintf( "pos %3d %c %g,%g,%g %g,%g,%g  %gx%g   %gx%g", TextureRef, c, x*1920.0, y*1080.0, 0.0, xp*1920.0, yp*1080.0, DrawBox.Position_z, (xp - x ) * 1920, ( yp-y) * 1080, (Coord.BottomRight_x -Coord.TopLeft_x) *Texture_Width, (Coord.BottomRight_y -Coord.TopLeft_y) *Texture_Height  );
					//	lprintf( "tex %3d %g,%g,%g %g,%g,%g", TextureRef, Coord.TopLeft_x, Coord.TopLeft_y, 0.0, Coord.BottomRight_x, Coord.BottomRight_y, 0.0 );
					render.simple_texture.Activate();
					render.simple_texture.SetUniforms( TextureRef, ref Color );
					render.simple_texture.DrawQuad( position, Coord.texture );
					render.simple_gui_texture.Activate();
					render.simple_gui_texture.SetUniforms( TextureRef, ref Color );
					render.simple_gui_texture.DrawQuad( position, Coord.texture );
				}
				x += DimX + TileStyle.CharSpacing_Sup;
			}
			//glColor3f(1.0,1.0,1.0);
		}

		internal void RenderFont( Display render, TileStyle TileStyle, ref Box DrawBox, string TextToRender )
		{
			RenderFont( render, TileStyle, ref DrawBox, TextToRender, ref DefaultDrawColor );
		}


		internal void GetFontRenderSize( TileStyle TileStyle, string TextToRender, out Vector2 OutSize )
		{
			float x, y, DimX, DimY;

			int i;
			TileSet TileSet;
			//TileCoord Coord;
			byte c;

			TileSet = TileStyle.TileSet;
			x = 0;
			y = 0;

			for( i = 0; i < TextToRender.Length; i++ )
			{
				c = (byte)TextToRender[i];
				DimX = TileSet.CoordTable[(int)c].Tile_Width * TileStyle.HSizeFactor;
				DimY = TileSet.CoordTable[(int)c].Tile_Height * TileStyle.VSizeFactor;
				x += DimX + TileStyle.CharSpacing_Sup;
				if( DimY > y ) y = DimY;
			}
			OutSize.X = x;
			OutSize.Y = y;
		}

#if GET_TILE_PIXEL
		internal int GetTilePixel( int TileNum, int x, int y )
		{
			Image image;
			int Image_x, Image_y;

			image = TextureManager.GetTextureEntry( TextureNum ).Texture;
			Bitmap bitmap = image as Bitmap;
			if( bitmap != null )
			{
				Image_x = (int)( x + ( TileNum % TilesPerLine ) * TileSlot_Width );
				Image_y = (int)( y + ( TileNum / TilesPerLine ) * TileSlot_Height );

				return ( bitmap.GetPixel( Image_x, Image_y ).ToArgb() );
			}
			return 0;
		}
#endif
	};
}
