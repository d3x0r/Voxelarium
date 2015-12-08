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
 * ZGameWindow_ProgressBar.cpp
 *
 *  Created on: 21 oct. 2011
 *      Author: laurent
 *  Ported on 08 dec. 2015
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
	internal class GameWindow_ProgressBar
	{
		Frame MainWindow;
		VoxelGameEnvironment GameEnv;
		bool Flag_Shown;

		ProgressBar Bar = new ProgressBar();


		internal GameWindow_ProgressBar()
		{
			Flag_Shown = false;
		}

		internal void SetGameEnv( VoxelGameEnvironment GameEnv ) { this.GameEnv = GameEnv; }

		internal void SetCompletion( float CompletionPercent )
		{
			Bar.SetCompletion( CompletionPercent );
		}


		internal bool Is_Shown() { return ( Flag_Shown ); }
		internal void Show()
		{
			// Main Window
			if( MainWindow == null )
			{
				MainWindow = new Frame();

				Vector2 MainWindow_Pos, MainWindow_Size;
				MainWindow_Size.X = Display.SclX( 200.0f ); MainWindow_Size.Y = Display.SclY( 15.0f );
				MainWindow_Pos.X = ( (float)2.0f / 2.0f - MainWindow_Size.X / 2.0f );
				MainWindow_Pos.Y = ( (float)2.0f * 0.98f - ( MainWindow_Size.Y ) );

				MainWindow.SetPosition( MainWindow_Pos.X, MainWindow_Pos.Y );
				MainWindow.SetSize( MainWindow_Size.X, MainWindow_Size.Y );
				MainWindow.SetTexture( TextureID.Button );

				// Inventory main title

				//Bar.SetTileSet( GameEnv.GuiTileset );
				Bar.SetPosition( Display.SclX( 5.0f ), Display.SclY( 5.0f ) );
				Bar.SetSize( MainWindow_Size.X - Display.SclX( 10.0f ), MainWindow_Size.Y- Display.SclY( 10.0f ) );
				Bar.SetCompletion( 50.0f );

			}
			GameEnv.GuiManager.AddFrame( MainWindow );
			MainWindow.AddFrame( Bar );

			Flag_Shown = true;
		}

		internal void Hide()
		{
			GameEnv.GuiManager.RemoveFrame( MainWindow );

			Flag_Shown = false;
		}

	}
}
