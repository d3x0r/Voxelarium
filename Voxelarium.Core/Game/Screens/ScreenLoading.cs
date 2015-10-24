using OpenTK;
using System;
using System.Collections.Generic;
using Voxelarium.Core.UI;

namespace Voxelarium.Core.Game.Screens
{
	internal class ScreenLoading : Screen
	{
		FontFrame Frame_Loading;
		Vector2 Loading_Size;
		internal ScreenLoading( VoxelGameEnvironment.Pages page_id ) : base( page_id )
		{
		}

		internal override ScreenChoices ProcessScreen( VoxelGameEnvironment GameEnv )
		{
			if( GameEnv.page_up != page_id )
			{
				GameEnv.page_up = page_id;
				GameEnv.GuiManager.RemoveAllFrames();

				if( Frame_Loading == null )
				{
					Frame_Loading = new FontFrame();
					Frame_Loading.SetDisplayText( "LOADING..." );
					Frame_Loading.SetStyle( GameEnv.TileSetStyles.GetStyle( 1 ) );
					Frame_Loading.GetTextDisplaySize( out Loading_Size );
					Frame_Loading.SetPosition(  1-Loading_Size.X / 2.0f, 1-Loading_Size.Y / 2.0f );
					Frame_Loading.SetSize( Loading_Size.X + 1.0f, Loading_Size.Y );
				}
				Frame_Loading.TextureNum = 3;

				//TitleBackground.AddFrame(&Frame_PlayGame);
				GameEnv.GuiManager.AddFrame( Frame_Loading );
			}
			return ( ScreenChoices.NONE );
		}
	}
}
