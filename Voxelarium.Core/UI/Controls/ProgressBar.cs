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
using System;
using System.Collections.Generic;
using System.Text;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.UI
{
	internal class ProgressBar : Frame
	{
		TileSet TileSet;
		float Factor;

		internal ProgressBar()
		{
			//FrameType = MulticharConst( 'P', 'B', 'A', 'R' ); // = InventoryBox;
			Flag_Cap_Dragable = false;
		}

		internal void SetCompletion( float Completion ) { this.Factor = Completion; }
		//internal override void SetTileSet( TileSet TileSet ) { this.TileSet = TileSet; }
		internal override void Render( Display render, ref Box ParentPosition )
		{
			Vector3 TopLeft, BottomRight, BarBottomRight;

			// Frame Position Computing
			if( Flag_Show_Master )
			{
				EffectivePosition.Position = ParentPosition.Position + Dimensions.Position;
				EffectivePosition.Size = Dimensions.Size;

				// Render this frame

				if( Flag_Show_Frame )
				{
					TopLeft = EffectivePosition.Position;
					BottomRight = EffectivePosition.Position + EffectivePosition.Size;
					BarBottomRight = BottomRight;
                    BarBottomRight.X = TopLeft.X + ( ( BottomRight.X - TopLeft.X ) * this.Factor / 100.0f );

			        {
						Vector3[] coords = new Vector3[4];
						coords[0] = TopLeft;
						coords[1] = TopLeft;
						coords[1].Y = BottomRight.Y;
						coords[2] = TopLeft;
						coords[2].X = BarBottomRight.X;
						coords[3] = BarBottomRight;
						coords[3].X = BarBottomRight.X;

						Vector4 c = new Vector4( 1 );
						render.simple_gui.Activate();
						render.simple_gui.DrawQuad( coords, ref c );
					}

					if( TileSet != null )
					{
						//TopLeft.z -= 1.0f;
						TileSet.RenderTile( render, ref TopLeft, ref BarBottomRight, 10, ref DrawColor );

						//TileSet.RenderTile(&TopLeft, &BottomRight, 8, &DrawColor);
					}
				}


				// Render child frames

				if( Flag_Show_Childs )
				{
					foreach( Frame frame in SubFrameList )
					{
						frame.Render( render, ref EffectivePosition );
					}
				}

			}

		}
	}
}
