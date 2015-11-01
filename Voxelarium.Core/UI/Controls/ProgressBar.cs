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
