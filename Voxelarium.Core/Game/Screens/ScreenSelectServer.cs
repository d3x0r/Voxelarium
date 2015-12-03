using System;
using System.Collections.Generic;
using System.Text;
using Voxelarium.Core.UI;
using Voxelarium.Core.UI.Controls;

namespace Voxelarium.Core.Game.Screens
{
	internal class ScreenSelectServer : Screen
	{
		FrameListbox ServerList;

		internal ScreenSelectServer( VoxelGameEnvironment.Pages page_id ) : base( page_id )
		{
		}

		internal override ScreenChoices ProcessScreen( VoxelGameEnvironment GameEnv )
		{
			if( GameEnv.page_up != page_id )
			{
				GameEnv.page_up = page_id;
				GameEnv.active_screen = this;
				GameEnv.GuiManager.RemoveAllFrames();

				if( ServerList == null )
				{

				}
			}
			return ResultCode;
		}
	}
}
