/*
 * Before porting, this header appeared inmost sources.  Of course
 * the change from C++ to C# required significant changes an no part
 * is entirely original.
 * 
 * This file is part of Blackvoxel. (Now Voxelarium)
 *
 * Copyright 2010-2014 Laurent Thiebaut & Olivia Merle
 * Copyright 2015-2016 James Buckeyne  *** Added 11/22/2015
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
using OpenTK;
using System;
using System.Collections.Generic;
using System.Text;
using Voxelarium.Core.UI;
using Voxelarium.Core.Voxels;

namespace Voxelarium.Core.Game.Screens
{
	internal class ScreenPlaySelect : Screen
	{
		Frame TitleBackground = new Frame();
		Frame Title = new Frame();
		Vector2 Title_Size;
		FontFrame Frame_Version = new FontFrame();
		Vector2 Version_Size;
		FontFrame Frame_PlayGame = new FontFrame();
		Vector2 PlayGame_Size;
		FontFrame Frame_Options = new FontFrame();
		Vector2 Options_Size;
		FontFrame Frame_Quit = new FontFrame();
		Vector2 Quit_Size;

		internal ScreenPlaySelect( VoxelGameEnvironment.Pages page_id ) : base( page_id )
		{
		}


		internal override ScreenChoices ProcessScreen( VoxelGameEnvironment GameEnv )
		{
			ResultCode = ScreenChoices.NONE;
			if( GameEnv.page_up != page_id )
			{
				float screen_x = GameEnv.GuiManager.FirstFrame.Dimensions.Size.X;
				float screen_y = GameEnv.GuiManager.FirstFrame.Dimensions.Size.Y;
				GameEnv.page_up = page_id;
				GameEnv.active_screen = this;
				GameEnv.GuiManager.RemoveAllFrames();

				TitleBackground.SetPosition( 0, 0 );
				TitleBackground.SetSize( screen_x, screen_y );
				TitleBackground.SetTexture( 0 );
				GameEnv.GuiManager.AddFrame( TitleBackground );



				Title_Size.X = 0.55f; Title_Size.X = 0.1f;
				Title.SetPosition( ( screen_x - Title_Size.X ) / 2.0f, screen_y - 1 / 8.0f );
				Title.SetSize( Title_Size.X, Title_Size.Y );
				Title.SetTexture( TextureID.TitleBanner );
				TitleBackground.AddFrame( Title );


				Version_Size.X = 0.25f; Version_Size.Y = 0.010f;
				Frame_Version.SetPosition( screen_x - Version_Size.X, Version_Size.Y );
				Frame_Version.SetSize( SclX( 53.0f * 8.0f + 1.0f ), SclY( 100.0f ) );
				Frame_Version.SetDisplayText( VoxelGlobalSettings.COMPILEOPTION_VERSIONSTRING );
				Frame_Version.TextureNum = TextureID.OldFont;
				Frame_Version.Font = GameEnv.default_font;// SetStyle( GameEnv.TileSetStyles.GetStyle( 0 ) );
				Frame_Version.FontSize = ( 2.0f / 60 );
				TitleBackground.AddFrame( Frame_Version );


				Frame_PlayGame.Text = "Single Player";
				Frame_PlayGame.Font = GameEnv.menu_font;// .SetStyle( GameEnv.TileSetStyles.GetStyle( 1 ) );
				Frame_PlayGame.FontSize = ( 2.0f / 10 );
				Frame_PlayGame.GetTextDisplaySize( out PlayGame_Size );
				Frame_PlayGame.SetPosition( screen_x / 2.0f - PlayGame_Size.X / 2.0f
											, screen_y *3f/ 4f );
				Frame_PlayGame.SetSize( PlayGame_Size.X + SclX( 128.0f ), PlayGame_Size.Y );
				//Frame_PlayGame.TextureNum = TextureID.OldFont;
				TitleBackground.AddFrame( Frame_PlayGame );

				Frame_Options.SetDisplayText( "Multiplayer" );
				Frame_Options.Font = GameEnv.menu_font;// .SetStyle( GameEnv.TileSetStyles.GetStyle( 1 ) );
				Frame_Options.FontSize = ( 2.0f / 10 );
				Frame_Options.GetTextDisplaySize( out Options_Size );
				Frame_Options.SetPosition( screen_x / 2.0f - Options_Size.X / 2.0f
									,  screen_y *3f/4f - PlayGame_Size.Y );
				Frame_Options.SetSize( Options_Size.X + 1.0f, Options_Size.Y );
				//Frame_Options.TextureNum = TextureID.OldFont;
				TitleBackground.AddFrame( Frame_Options );


				TitleBackground.AddFrame( Frame_Quit );
				//printf("FrameAdress : %lx\n",(unsigned int)&Frame_PlayGame);
			}
			{
				if( Frame_PlayGame.Is_MouseIn() ) { Frame_PlayGame.SetColor( 0.5f, 0.5f, 1.0f ); }
				if( Frame_PlayGame.Is_MouseOut() ) { Frame_PlayGame.SetColor( 1.0f, 1.0f, 1.0f ); }
				if( Frame_Options.Is_MouseIn() ) { Frame_Options.SetColor( 0.5f, 0.5f, 1.0f ); }
				if( Frame_Options.Is_MouseOut() ) { Frame_Options.SetColor( 1.0f, 1.0f, 1.0f ); }
				//if( Frame_Quit.Is_MouseIn() ) { Frame_Quit.SetColor( 0.5f, 0.5f, 1.0f ); }
				//if( Frame_Quit.Is_MouseOut() ) { Frame_Quit.SetColor( 1.0f, 1.0f, 1.0f ); }
				if( Frame_PlayGame.Is_MouseClick() ) { ResultCode = ScreenChoices.PLAYGAME; }
				if( Frame_Options.Is_MouseClick() ) { ResultCode = ScreenChoices.OPTIONS; }
				//if( Frame_Quit.Is_MouseClick() ) { ResultCode = ScreenChoices.QUIT; }
			}

			return ( ResultCode );
		}
	}
}
