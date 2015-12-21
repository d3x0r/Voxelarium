/*
 * This file is part of Voxelarium.
 *
 * Copyright 2015-2016 James Buckeyne  *** Added 18/12/2015
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
using System;
using System.Collections.Generic;
using System.Text;
using Voxelarium.Core.UI;
using Voxelarium.Core.UI.Controls;

namespace Voxelarium.Core.Game.Screens
{
	internal class ScreenSelectServer : Screen
	{
		internal class ServerSelectItem : FrameListbox.ListboxItem
		{
			string ServerName;
			string ServerConnections;

			internal override void Render()
			{
				
			}
		}
		FrameListbox ServerList;

		internal ScreenSelectServer( VoxelGameEnvironment.Pages page_id ) : base( page_id )
		{
		}

		internal override ScreenChoices ProcessScreen( VoxelGameEnvironment GameEnv )
		{
			if( GameEnv.page_up != page_id )
			{
				GameEnv.page_up = page_id;
				GameEnv.GuiManager.RemoveAllFrames();

				if( ServerList == null )
				{
					ServerList = new FrameListbox();
					ServerList.EffectivePosition.Position.X = Display.SclX( 100 );
					ServerList.EffectivePosition.Position.Y = Display.SclY( 100 );
					ServerList.EffectivePosition.Size.X = Display.SclX( 1920 - 200 );
					ServerList.EffectivePosition.Size.Y = Display.SclY( 1080 - 200 );
					ServerList.TextureNum = TextureID.DialogBackground;
				}

				GameEnv.GuiManager.AddFrame( ServerList );

			}
			return ResultCode;
		}
	}
}
