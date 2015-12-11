/*
 * This file is part of Voxelarium.
 *
 * Copyright 2015-2016 James Buckeyne  
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
 * Created 2015/12/01 d3x0r
*/
using OpenTK;
using System;
using System.Collections.Generic;
using System.Text;
using Voxelarium.Core.UI;

namespace Voxelarium.Core.Game.Screens
{
	internal class ScreenConnecting : Screen
	{
		FontFrame Frame_Connecting;
		ProgressBar Frame_ProgressBar;

		internal ScreenConnecting( VoxelGameEnvironment.Pages page_id ) : base( page_id )
		{
		}


		internal override ScreenChoices ProcessScreen( VoxelGameEnvironment GameEnv )
		{
			if( GameEnv.page_up != page_id )
			{
				GameEnv.page_up = page_id;
				GameEnv.GuiManager.RemoveAllFrames();
				if( Frame_Connecting == null )
				{
					Frame_Connecting = new FontFrame();
					Frame_Connecting.Text = "Connecting...";
					Frame_Connecting.Font = GameEnv.menu_font;
					Frame_Connecting.FontSize = ( 2.0f / 10 );
					Frame_Connecting.SetPosition( 1 - Frame_Connecting.Dimensions.Size.X / 2.0f
												, 1 - Frame_Connecting.Dimensions.Size.Y / 2.0f );
				}

				if( Frame_ProgressBar == null )
				{
					Frame_ProgressBar = new ProgressBar();
					Frame_ProgressBar.SetPosition( 0.5f, 0.5f );
					Frame_ProgressBar.SetSize( 1.0f, 0.2f );
					Frame_ProgressBar.SetCompletion( 50 );
				}

				GameEnv.GuiManager.AddFrame( Frame_Connecting );
				Frame_Connecting.AddFrame( Frame_ProgressBar );

			}
			if( GameEnv.Master_Server_Connection.PercentToFail == 100 )
				return ScreenChoices.CHOICE_RETURN;
			if( GameEnv.Master_Server_Connection.Connected )
				return ScreenChoices.SELECT_SERVER;
			Frame_ProgressBar.SetCompletion( GameEnv.Master_Server_Connection.PercentToFail );

			return ResultCode;
		}
	}
}
