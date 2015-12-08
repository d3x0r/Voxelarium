/*
 * Before porting, this header appeared inmost sources.  Of course
 * the change from C++ to C# required significant changes an no part
 * is entirely original.
 * 
 * This file is part of Blackvoxel. (Now Voxelarium)
 *
 * Copyright 2010-2014 Laurent Thiebaut & Olivia Merle
 * Copyright 2015-2016 James Buckeyne  *** Added 12/08/2015
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
 *
 * ZGameWindow_VoxelTypeBar.cpp
 *
 *  Created on: 12 juil. 2011
 *      Author: laurent
 *
 * ported on 07 Dec. 2015
 *      Porter: James Buckeyne
*/
using OpenTK;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Voxelarium.Core.UI;

namespace Voxelarium.Core.Game.GameWindows
{
	internal class GameWindow_VoxelTypeBar
	{
		VoxelGameEnvironment GameEnv;
		Frame MainWindow = new Frame();
		bool Flag_Shown;


		InventoryBox[] MainInventory;

		internal GameWindow_VoxelTypeBar()
		{
			MainInventory = new InventoryBox[128];
			Flag_Shown = false;
		}

		~GameWindow_VoxelTypeBar()
		{
			MainInventory = null;
		}

		internal void SetGameEnv( VoxelGameEnvironment GameEnv ) { this.GameEnv = GameEnv; }
		internal InventoryBox GetInventoryBox( int BoxNum ) { return ( MainInventory[BoxNum] ); }

		internal void Show_Slot( int SlotNum ) { MainInventory[SlotNum].Show( true ); }
		internal void Hide_Slot( int SlotNum ) { MainInventory[SlotNum].Show( false ); }

		internal bool Is_Shown() { return ( Flag_Shown ); }
		internal void Show()
		{
			Vector2 Rp, Ip;
			Actor Actor;
			int Y;

			Actor = GameEnv.GetActiveActor(); if( Actor == null ) return;

			// Running position computing

			Ip.X = Display.SclX( 18.0f ); Ip.Y = Display.SclY( 25.0f );
			Rp.X = Ip.X; Rp.Y = Ip.Y;

			// Main Window

			Vector2 MainWindow_Pos, MainWindow_Size;
			MainWindow_Size.X = Display.SclX( 48.0f ); MainWindow_Size.Y = Display.SclY( 600.0f );
			MainWindow_Pos.X = ( (float)2.0f - MainWindow_Size.X );
			MainWindow_Pos.Y = ( (float)2.0f - MainWindow_Size.Y ) / 2.0f;

			MainWindow.SetPosition( MainWindow_Pos.X, MainWindow_Pos.Y );
			MainWindow.SetSize( MainWindow_Size.X, MainWindow_Size.Y );
			MainWindow.SetTexture( TextureID.Button );

			GameEnv.GuiManager.AddFrame( MainWindow );


			// Inventory main title

			// Textures

			for( Y = 0; Y < 14; Y++ )
			{
				int Indice = Y;
				Inventory.Entry Entry = Actor.Inventory.GetSlotRef( Indice );
				MainInventory[Indice].SetVoxelTypeManager( GameEnv.VoxelTypeManager );
				if( Y != 6 )
				{
					MainInventory[Indice].SetPosition( 8.0f, Rp.Y );
					MainInventory[Indice].SetSize( 32.0f, 32.0f );
					Rp.Y += 32.0f + 6.0f;
				}
				else
				{
					MainInventory[Indice].SetPosition( -24.0f, Rp.Y );
					MainInventory[Indice].SetSize( 64.0f, 64.0f );
					Rp.Y += 64.0f + 6.0f;
				}
				MainInventory[Indice].SetEntry( Entry );
				MainInventory[Indice].SetColor( 255.0f, 255.0f, 255.0f );
				MainWindow.AddFrame( MainInventory[Indice] );

			}

			Flag_Shown = true;
		}

		internal void Hide()
		{
			GameEnv.GuiManager.RemoveFrame( MainWindow );

			Flag_Shown = false;
		}
	}
}
