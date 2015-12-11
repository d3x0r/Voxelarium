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
