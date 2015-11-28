/*
 * This file is part of Voxelarium.
 *
 * Copyright 2010-2014 Laurent Thiebaut & Olivia Merle
 * Copyright 2015-2016 James Buckeyne
 *
 * Blackvoxel is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Blackvoxel is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
 * ZGui_InventoryBox.cpp
 *
 *  Created on: 4 juil. 2011
 *      Author: laurent
 *  Ported on 27 nov, 2015
 *      Porter: James Buckeyne
 */

using OpenTK;
using OpenTK.Input;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Voxelarium.Core.UI;
using Voxelarium.Core.Voxels;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.Game.GameWindows
{
	internal class InventoryBox : Frame
	{
		ushort VoxelType;
		int Quantity;
		VoxelTypeManager VoxelTypeManager;
		TileSet.TileStyle TileStyle;


		internal InventoryBox()
		{
			FrameType = VoxelGlobalSettings.MulticharConst( 'I', 'B', 'O', 'X' ); // = InventoryBox;
			Flag_Cap_Dragable = true;
			VoxelType = 0;
		}

		internal virtual void SetVoxelType( ushort VoxelType ) { this.VoxelType = VoxelType; }
		internal virtual void SetQuantity( int Quantity ) { this.Quantity = Quantity; }
		internal virtual void SetTileStyle( TileSet.TileStyle Style ) { this.TileStyle = Style; }
		internal virtual ushort GetVoxelType() { return ( VoxelType ); }
		internal virtual int GetQuantity() { return ( Quantity ); }
		internal virtual void SetVoxelTypeManager( VoxelTypeManager VoxelTypeManager ) { this.VoxelTypeManager = VoxelTypeManager; }

		internal override void Render( Display render, ref Box ParentPosition )
		{

			Vector3 TopLeft, BottomRight;
			//ListItem Item;
			Frame Frame;

			// Frame Position Computing
			if( Flag_Show_Master )
			{
				EffectivePosition.Position.X = ParentPosition.Position.X + Dimensions.Position.Y;
				EffectivePosition.Position.Y = ParentPosition.Position.Y + Dimensions.Position.Y;
				EffectivePosition.Position.Z = ParentPosition.Position.Z + Dimensions.Position.Z;
				EffectivePosition.Size.X = Dimensions.Size.X;
				EffectivePosition.Size.Y = Dimensions.Size.Y;
				EffectivePosition.Size.Z = Dimensions.Size.Z;

				if( this.Flag_Draging ) { EffectivePosition.Position.X = DragAbsolutePosition.X; EffectivePosition.Position.Y = DragAbsolutePosition.Y; EffectivePosition.Position.Z = ParentPosition.Position.Z + Dimensions.Position.Z + 2.0f; }

				// Render this frame

				if( Flag_Show_Frame )
				{
					TopLeft = EffectivePosition.Position;
					BottomRight = EffectivePosition.Position + EffectivePosition.Size;


					// if (TileSet) TileSet.RenderTile(&TopLeft, &BottomRight, Tile, &DrawColor);

					Vector2 P1, P2, P3, P4;

					P1.X = TopLeft.X; P1.Y = TopLeft.Y;
					P2.X = BottomRight.X; P2.Y = TopLeft.Y;
					P3.X = BottomRight.X; P3.Y = BottomRight.Y;
					P4.X = TopLeft.X; P4.Y = BottomRight.Y;


					int texture;
					Vector4 color;
					// Render
					float[] uvs = new float[2 * 4];
					if( VoxelType != 0 )
					{
						texture = render.game.World.TextureAtlas.OpenGl_TextureRef;
						uvs = VoxelTypeManager.VoxelTable[VoxelType].TextureUVs;
					}
					else
						texture = 0;
					if( VoxelType != 0 ) color = Vector4.One;
					else color = new Vector4( 0.4f, 0.4f, 0.4f, 0.6f ); //  glColor4f(0.7f, 0.7f, 0.7f, 0.9f);
					float[] verts = new float[3 * 4];
					verts[0 * 3 + 0] = P1.X;
					verts[0 * 3 + 1] = P1.Y;
					verts[0 * 3 + 2] = TopLeft.Z;
					verts[0 * 3 + 0] = P2.X;
					verts[0 * 3 + 1] = P2.Y;
					verts[0 * 3 + 2] = TopLeft.Z;
					verts[0 * 3 + 0] = P3.X;
					verts[0 * 3 + 1] = P3.Y;
					verts[0 * 3 + 2] = BottomRight.Z;
					verts[0 * 3 + 0] = P4.X;
					verts[0 * 3 + 1] = P4.Y;
					verts[0 * 3 + 2] = BottomRight.Z;
					render.simple_gui_texture.DrawQuad( verts, uvs );
					// Rendering Quantity


					if( Quantity > 1 )
					{
						String QuantityText;

						QuantityText = Quantity.ToString();

						Box FontBox;
						Vector2 Dim;

						TileStyle.TileSet.GetFontRenderSize( TileStyle, QuantityText, out Dim );

						FontBox.Position.X = BottomRight.X - Dim.X - 1.0f;
						FontBox.Position.Y = BottomRight.Y - Dim.Y - 1.0f;
						FontBox.Position.Z = TopLeft.Z + 2.0f;
						FontBox.Size.X = Dim.X;
						FontBox.Size.Y = Dim.Y;
						FontBox.Size.Z = 0.0f;
						Vector4 Color;
						Vector4 BkColor;
						Color = Vector4.One;
						BkColor = Vector4.Zero;
						TileStyle.TileSet.RenderFont( render, TileStyle, ref FontBox, QuantityText, ref Color );
						FontBox.Position.Z -= 1.0f;
						FontBox.Position.X -= Display.SclX(1.0f );
						FontBox.Position.Y -= Display.SclY(1.0f );

						TileStyle.TileSet.RenderFont( render, TileStyle, ref FontBox, QuantityText, ref BkColor );
					}
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


		internal void DropItem( Display render, Frame Item, ushort nButton )
		{
			if( Item.FrameType == VoxelGlobalSettings.MulticharConst( 'I', 'B', 'O', 'X' ) )
			{
				InventoryBox IbItem;
				ushort In_VoxelType, Temp_VoxelType;
				int In_Quantity, Temp_Quantity;

				IbItem = (InventoryBox)Item;

				In_VoxelType = IbItem.GetVoxelType();
				In_Quantity = IbItem.GetQuantity();

				if( In_VoxelType != VoxelType && Quantity > 0 )
				{
					if( nButton == 1 )
					{
						Temp_VoxelType = In_VoxelType;
						Temp_Quantity = In_Quantity;
						IbItem.SetVoxelType( VoxelType );
						IbItem.SetQuantity( Quantity );
						VoxelType = Temp_VoxelType;
						Quantity = Temp_Quantity;
					}
				}

				else if( In_VoxelType == VoxelType || Quantity == 0 )
				{
					if( nButton == 1 )
					{
						Quantity += In_Quantity;
						VoxelType = In_VoxelType;
						IbItem.SetVoxelType( 0 );
						IbItem.SetQuantity( 0 );
					}
					if( nButton == 3 )
					{
						int TransfertQuantity = 1;
						if( render.Keyboard[ Key.ShiftLeft] ||
							render.Keyboard[Key.ShiftRight] )
							TransfertQuantity = 10;
						if( render.Keyboard[Key.ControlLeft] ||
							render.Keyboard[Key.ControlRight] )
							TransfertQuantity = 100;
						if( render.Keyboard[Key.AltLeft] ||
							render.Keyboard[Key.AltRight] )
							TransfertQuantity = 1000;
						if( TransfertQuantity > In_Quantity ) TransfertQuantity = In_Quantity;
						if( In_Quantity > 0 )
						{
							( Quantity ) += TransfertQuantity;
							IbItem.SetQuantity( In_Quantity - TransfertQuantity );
							VoxelType = In_VoxelType;
						}
					}
				}

			}
			// printf("Dropped : %lx in %lx\n",(ULong)Item, (ULong)this);
		}
	}
}
