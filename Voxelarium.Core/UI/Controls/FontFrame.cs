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
using Voxelarium.Core.Voxels;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.UI
{
	class FontFrame : Frame
	{
		string TextToDisplay;
		//TileSet.TileStyle TileStyle;
		FontRenderer font;
		float line_height;
		internal FontFrame()
		{
			FrameType = VoxelGlobalSettings.MulticharConst( 'T', 'E', 'X', 'T' );
			TextToDisplay = "";
		}
		internal virtual void SetDisplayText( string TextToDisplay ) { this.TextToDisplay = TextToDisplay; }
		//internal virtual void Style( TileSet.TileStyle TileStyle ) { this.TileStyle = TileStyle; }
		internal virtual FontRenderer Font { set { font = value; } }
		internal virtual float FontSize { set { line_height = value; } get { return line_height; } }
		internal virtual void GetTextDisplaySize( out Vector2 OutSize )
		{
			if( font != null )
			{
				Vector2 target_size;
				font.GetFontRenderSize( TextToDisplay, out target_size );
				OutSize.X = ( target_size.X * line_height ) / target_size.Y;
				OutSize.Y = ( line_height );
			}
			else
				OutSize = new Vector2( 0 );
		}

		internal override void Render( Display render, ref Box ParentPosition )
		{
			if( Flag_Show_Master )
			{
				EffectivePosition.Position = ParentPosition.Position + Dimensions.Position;
				EffectivePosition.Position.Z -= 0.1f;
                EffectivePosition.Size = Dimensions.Size;
				if( Flag_Show_Frame )
				{
					if( font != null )
						font.RenderFont( render, ref EffectivePosition, line_height
                            , this.TextToDisplay, ref DrawColor );
				}

				// Render child frames
				if( Flag_Show_Childs )
				{
					//SubFrameList.Dump();
					foreach( Frame frame in SubFrameList )
					{
						frame.Render( render, ref EffectivePosition );
					}
				}

			}

		}
	}
}
