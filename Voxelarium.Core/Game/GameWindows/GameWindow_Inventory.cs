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
 * ZGame_GUI.cpp
 *
 *  Created on: 2 juil. 2011
 *      Author: laurent
 *  Ported on 27 nov. 2015
 *      Porter: James Buckeyne
 */

using OpenTK;
using System;
using System.Collections.Generic;

using System.Text;
using Voxelarium.Core.UI;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.Game.GameWindows
{
	internal class GameWindow_Inventory
	{
		Frame MainWindow = new Frame();
		InventoryBox IBox;
		InventoryBox IBox2;
		InventoryBox IBox3;
		VoxelGameEnvironment GameEnv;
		bool Flag_Shown;

		FontFrame MainTitle = new FontFrame();
		FontFrame ToolTitle = new FontFrame();
		FontFrame InventoryTitle = new FontFrame();
		FontFrame PowerTitle = new FontFrame();
		InventoryBox[] MainInventory;

		string Text_MainTitle;
		string Text_ToolTitle;
		string Text_InventoryTitle;
		string Text_PowerTitle;


		ushort i1, i2, i3;
		ulong Q1, Q2, Q3;
		
		internal GameWindow_Inventory()
		{
			MainInventory = new InventoryBox[128];
			Flag_Shown = false;
			Text_MainTitle = "INVENTORY";
			Text_ToolTitle = "TOOLS";
			Text_InventoryTitle = "INVENTORY";
			Text_PowerTitle = "POWERS";
		}

		~GameWindow_Inventory()
		{
			if( MainInventory != null ) MainInventory = null;
		}

		internal void SetGameEnv( VoxelGameEnvironment GameEnv ) { this.GameEnv = GameEnv; }


		internal bool Is_Shown() { return ( Flag_Shown ); }

		internal void Show()
		{
			Vector2 Rp, Ip, Size;
			Actor Actor;
			int x, y;

			Actor = GameEnv.GetActiveActor(); if( Actor == null ) return;

			// Running position computing

			Ip.X = 32.0f * 2 / 1024; Ip.Y = 25.0f * 2/768;
			Rp.X = Ip.X; Rp.Y = Ip.Y;

			// Main Window

			Vector2 MainWindow_Pos, MainWindow_Size;
			MainWindow_Size.X = 758.0f / 1024; MainWindow_Size.Y = 600.0f / 768;
			MainWindow_Pos.X = ( 2 - MainWindow_Size.X ) / 2.0f;
			MainWindow_Pos.Y = ( 2 - MainWindow_Size.Y ) / 2.0f;

			MainWindow.SetPosition( MainWindow_Pos.X, MainWindow_Pos.Y );
			MainWindow.SetSize( MainWindow_Size.X, MainWindow_Size.Y );
			MainWindow.SetTexture( 8 );
			GameEnv.GuiManager.AddFrame( MainWindow );

			// Inventory main title

			MainTitle.Font = GameEnv.default_font;
			MainTitle.FontSize = 2.0f / 60f;
			MainTitle.SetDisplayText( Text_MainTitle );
			MainTitle.GetTextDisplaySize( out Size );
			MainTitle.SetPosition( MainWindow_Size.X / 2.0f - Size.X / 2.0f, Rp.Y );
			MainTitle.SetSize( Size.X, Size.Y );
			MainTitle.SetColor( 255.0f, 255.0f, 255.0f );
			MainWindow.AddFrame( MainTitle );
			Rp.Y += Size.Y + Display.SclY( 20.0f );

			// Tools Title

			ToolTitle.Font = GameEnv.default_font;
			ToolTitle.FontSize = 2.0f / 80f;
			ToolTitle.SetDisplayText( Text_ToolTitle );
			ToolTitle.GetTextDisplaySize( out Size );
			ToolTitle.SetPosition( Rp.X, Rp.Y );
			ToolTitle.SetSize( Size.X, Size.Y );
			ToolTitle.SetColor( 255.0f, 255.0f, 255.0f );
			MainWindow.AddFrame( ToolTitle );
			Rp.Y += Size.Y + Display.SclY( 5.0f );

			// Tools

			for( x = 0; x < 10; x++ )
			{
				int Indice = (int)Inventory.SlotType.Tools_StartSlot + x;
				int Entry = Actor.Inventory.GetSlotRef( Indice );
				MainInventory[Indice].SetVoxelTypeManager( GameEnv.VoxelTypeManager );
				MainInventory[Indice].SetPosition( 32.0f + x * 70.0f, Rp.Y );
				MainInventory[Indice].SetSize( 64.0f, 64.0f );
				MainInventory[Indice].SetTileStyle( GameEnv.TileSetStyles.GetStyle( 3 ) );
				MainInventory[Indice].SetVoxelType( Actor.Inventory.SlotTable[Entry].VoxelType );
				MainInventory[Indice].SetQuantity( Actor.Inventory.SlotTable[Entry].Quantity );
				MainInventory[Indice].SetColor( 255.0f, 255.0f, 255.0f );
				MainWindow.AddFrame( MainInventory[Indice] );
			}
			Rp.Y += 64.0f + 12.0f;

			// Inventory Zone Title

			InventoryTitle.Font = GameEnv.default_font;
			InventoryTitle.FontSize = 2.0f / 80;
			InventoryTitle.SetDisplayText( Text_InventoryTitle );
			InventoryTitle.GetTextDisplaySize( out Size );
			InventoryTitle.SetPosition( Rp.X, Rp.Y );
			InventoryTitle.SetSize( Size.X, Size.Y );
			InventoryTitle.SetColor( 255.0f, 255.0f, 255.0f );
			MainWindow.AddFrame( InventoryTitle );
			Rp.Y += Size.Y + Display.SclY( 5.0f );


			// Inventory

			for( y = 0; y < 4; y++ )
			{
				for( x = 0; x < 10; x++ )
				{
					int Indice = (int)Inventory.SlotType.Inventory_StartSlot + x + y * 10;
					int Entry = Actor.Inventory.GetSlotRef( Indice );
					MainInventory[Indice].SetVoxelTypeManager( GameEnv.VoxelTypeManager );
					MainInventory[Indice].SetPosition( 32.0f + x * 70.0f, Rp.Y );
					MainInventory[Indice].SetSize( 64.0f, 64.0f );
					MainInventory[Indice].SetTileStyle( GameEnv.TileSetStyles.GetStyle( 3 ) );
					MainInventory[Indice].SetVoxelType( Actor.Inventory.SlotTable[Entry].VoxelType );
					MainInventory[Indice].SetQuantity( Actor.Inventory.SlotTable[Entry].Quantity );
					MainInventory[Indice].SetColor( 128.0f, 128.0f, 128.0f );
					MainWindow.AddFrame( MainInventory[Indice] );
				}
				Rp.Y += 64.0f + 6.0f;
			}
			Rp.Y += 12.0f;

			// Powers Zone Title
			PowerTitle.Font = GameEnv.default_font;
			PowerTitle.FontSize = 2.0f / 60;
			PowerTitle.SetDisplayText( Text_PowerTitle );
			PowerTitle.GetTextDisplaySize( out Size );
			PowerTitle.SetPosition( Rp.X, Rp.Y );
			PowerTitle.SetSize( Size.X, Size.Y );
			PowerTitle.SetColor( 255.0f, 255.0f, 255.0f );
			MainWindow.AddFrame( PowerTitle );
			Rp.Y += Size.Y + 5.0f;

			// Powers

			for( x = 0; x < 10; x++ )
			{
				int Indice = (int)Inventory.SlotType.Powers_StartSlot + x;
				int Entry = Actor.Inventory.GetSlotRef( Indice );
				MainInventory[Indice].SetVoxelTypeManager( GameEnv.VoxelTypeManager );
				MainInventory[Indice].SetPosition( 32.0f + x * 70.0f, Rp.Y );
				MainInventory[Indice].SetSize( 64.0f, 64.0f );
				MainInventory[Indice].SetTileStyle( GameEnv.TileSetStyles.GetStyle( 3 ) );
				MainInventory[Indice].SetVoxelType( Actor.Inventory.SlotTable[Entry].VoxelType );
				MainInventory[Indice].SetQuantity( Actor.Inventory.SlotTable[Entry].Quantity );
				MainInventory[Indice].SetColor( 255.0f, 255.0f, 255.0f );
				MainWindow.AddFrame( MainInventory[Indice] );
			}
			Rp.Y += 64.0f + 12.0f;

			Display.ShowMouse();

			GameEnv.Game_Events.SetDisableMouseEvents();
			Flag_Shown = true;
		}

		internal void Hide()
		{
			Display.HideMouse();
			GameEnv.Game_Events.SetEnableMouseEvents();
			Flag_Shown = false;
		}

	}
}
