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
 * ZGameWindow_Storage.cpp
 *
 *  Created on: 30 sept. 2011
 *      Author: laurent
 *
 * ZGame_GUI.cpp
 *
 *  Created on: 2 juil. 2011
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
using Voxelarium.Core.Voxels;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.Game.GameWindows
{
	class GameWindow_Storage
	{
		Frame MainWindow;
		InventoryBox IBox;
		InventoryBox IBox2;
		InventoryBox IBox3;
		VoxelGameEnvironment GameEnv;
		bool Flag_Shown;

		FontFrame MainTitle = new FontFrame();
		FontFrame StorageTitle = new FontFrame();
		FontFrame InventoryTitle = new FontFrame();

		InventoryBox[] MainStorage;

		string Text_MainTitle;
		string Text_StorageTitle;
		string Text_InventoryTitle;

		IVoxelExtension_Storage VoxelExtension_Storage;

		public GameWindow_Storage()
		{
			MainStorage = new InventoryBox[128];
			Flag_Shown = false;
			Text_MainTitle = "STORAGE";
			Text_StorageTitle = "STORAGE";
			Text_InventoryTitle = "INVENTORY";
			VoxelExtension_Storage = null;
		}

		~GameWindow_Storage()
		{
			MainStorage = null;
		}

		internal void SetGameEnv( VoxelGameEnvironment GameEnv ) { this.GameEnv = GameEnv; }

		internal void SetVoxelExtension( VoxelExtension Extension ) { VoxelExtension_Storage = (IVoxelExtension_Storage)Extension; }


		internal bool Is_Shown() { return ( Flag_Shown ); }
		internal void Show()
		{
			Vector2 Rp, Ip, Size;
			Actor Actor;
			int X, Y;

			Actor = GameEnv.GetActiveActor(); if( Actor == null ) return;

			// Running position computing

			Ip.X = Display.SclX( 32.0f ); Ip.Y = Display.SclY( 25.0f );
			Rp.X = Ip.X; Rp.Y = Ip.Y;

			// Main Window

			Vector2 MainWindow_Pos, MainWindow_Size;
			MainWindow_Size.X = Display.SclX( 758.0f ); MainWindow_Size.Y = Display.SclY( 600.0f );
			MainWindow_Pos.X = ( (float)2.0f - MainWindow_Size.X ) / 2.0f;
			MainWindow_Pos.Y = ( (float)2.0f - MainWindow_Size.Y ) / 2.0f;

			MainWindow.SetPosition( MainWindow_Pos.X, MainWindow_Pos.Y );
			MainWindow.SetSize( MainWindow_Size.X, MainWindow_Size.Y );
			MainWindow.SetTexture( TextureID.DialogVoxelTypeBackground );
			GameEnv.GuiManager.AddFrame( MainWindow );

			// Inventory main title

			//MainTitle.SetStyle( GameEnv.TileSetStyles.GetStyle( ZGame::FONTSIZE_3 ) );
			MainTitle.Font = GameEnv.default_font;
			MainTitle.FontSize = 1f/40.0f;
			MainTitle.SetDisplayText( Text_MainTitle );
			MainTitle.GetTextDisplaySize( out Size );
			MainTitle.SetPosition( MainWindow_Size.X / 2.0f - Size.X / 2.0f, Rp.Y );
			MainTitle.SetSize( Size.X, Size.Y );
			MainTitle.SetColor( 255.0f, 255.0f, 255.0f );
			MainWindow.AddFrame( MainTitle );
			Rp.Y += Size.Y + 20.0f;

			// Tools Title
			StorageTitle.Font = GameEnv.default_font;
			StorageTitle.FontSize = 1f / 60.0f;
			StorageTitle.SetDisplayText( Text_StorageTitle );
			StorageTitle.GetTextDisplaySize( out Size );
			StorageTitle.SetPosition( Rp.X, Rp.Y );
			StorageTitle.SetSize( Size.X, Size.Y );
			StorageTitle.SetColor( 255.0f, 255.0f, 255.0f );
			MainWindow.AddFrame( StorageTitle );
			Rp.Y += Size.Y + Display.SclY( 5.0f );

			// Tools

			for( Y = 0; Y < 4; Y++ )
			{
				for( X = 0; X < 10; X++ )
				{
					int Indice = 0 + X + Y * 10;
					MainStorage[Indice].SetVoxelTypeManager( GameEnv.VoxelTypeManager );
					MainStorage[Indice].SetPosition( Display.SclX( 32.0f ) + X * Display.SclX( 70.0f ), Rp.Y );
					MainStorage[Indice].SetSize( Display.SclX( 48.0f ), Display.SclY( 48.0f ) );
					MainStorage[Indice].SetTileStyle( GameEnv.TileSetStyles.GetStyle( 3 ) );
					MainStorage[Indice].SetEntry( VoxelExtension_Storage.GetStorageEntry(Indice) );
					MainStorage[Indice].SetColor( 128.0f, 128.0f, 128.0f );
					MainWindow.AddFrame( MainStorage[Indice] );
				}
				Rp.Y += Display.SclY( 48.0f + 6.0f);
			}
			Rp.Y += Display.SclY( 12.0f );

			// Inventory Zone Title
			InventoryTitle.Font = GameEnv.default_font;
			InventoryTitle.FontSize = 1 / 40f;
			//InventoryTitle.SetStyle( GameEnv.TileSetStyles.GetStyle( ZGame::FONTSIZE_1 ) );
			InventoryTitle.SetDisplayText( Text_InventoryTitle );
			InventoryTitle.GetTextDisplaySize( out Size );
			InventoryTitle.SetPosition( Rp.X, Rp.Y );
			InventoryTitle.SetSize( Size.X, Size.Y );
			InventoryTitle.SetColor( 255.0f, 255.0f, 255.0f );
			MainWindow.AddFrame( InventoryTitle );
			Rp.Y += Size.Y + Display.SclY( 5.0f );


			// Inventory

			for( Y = 0; Y < 4; Y++ )
			{
				for( X = 0; X < 10; X++ )
				{
					int Indice = (int)Inventory.SlotType.Inventory_StartSlot + X + Y * 10;
					//Inventory.Entry Entry = Actor.Inventory.GetSlotRef( Indice );
					MainStorage[Indice + 40].SetVoxelTypeManager( GameEnv.VoxelTypeManager );
					MainStorage[Indice + 40].SetPosition( 32.0f + X * 70.0f, Rp.Y );
					MainStorage[Indice + 40].SetSize( 48.0f, 48.0f );
					MainStorage[Indice + 40].SetTileStyle( GameEnv.TileSetStyles.GetStyle( 3 ) );
					MainStorage[Indice + 40].SetEntry( Actor.Inventory.SlotTable[Indice] );
					MainStorage[Indice + 40].SetColor( 128.0f, 128.0f, 128.0f );
					MainWindow.AddFrame( MainStorage[Indice + 40] );
				}
				Rp.Y += Display.SclY( 48.0f + 6.0f );
			}
			Rp.Y += Display.SclY( 12.0f );

			Display.ShowMouse();

			GameEnv.Game_Events.SetDisableMouseEvents();
			Flag_Shown = true;
		}

		internal void Hide()
		{
			GameEnv.GuiManager.RemoveFrame( MainWindow );
			Display.HideMouse();

			Flag_Shown = false;
		}
	}
}
