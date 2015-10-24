using OpenTK;
using System;
using System.Collections.Generic;
using System.Text;
using Voxelarium.Core.UI;
using Voxelarium.Core.Voxels;

namespace Voxelarium.Core.Game.Screens
{
	internal class ScreenMain : Screen
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

		internal ScreenMain(VoxelGameEnvironment.Pages page_id ) : base( page_id )
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
				Title.SetPosition( screen_x - Title_Size.X / 2.0f, screen_y / 8.0f );
				Title.SetSize( Title_Size.X, Title_Size.Y );
				Title.SetTexture( 1 );
				TitleBackground.AddFrame( Title );


				Version_Size.X = 0.25f; Version_Size.Y = 0.010f;
				Frame_Version.SetPosition( screen_x - Version_Size.X, Version_Size.Y );
				Frame_Version.SetSize( SclX( 53.0f * 8.0f + 1.0f ), SclY( 100.0f ) );
				Frame_Version.SetDisplayText( VoxelGlobalSettings.COMPILEOPTION_VERSIONSTRING );
				Frame_Version.TextureNum = 3;
				Frame_Version.SetStyle( GameEnv.TileSetStyles.GetStyle( 0 ) );
				TitleBackground.AddFrame( Frame_Version );



				PlayGame_Size.X = SclX( 9.0f * 32.0f + 1.0f ); PlayGame_Size.Y = SclY( 32.0f );
				Frame_PlayGame.SetDisplayText( "PLAY GAME" );
				Frame_PlayGame.SetStyle( GameEnv.TileSetStyles.GetStyle( 1 ) );
				Frame_PlayGame.GetTextDisplaySize( out PlayGame_Size );
				Frame_PlayGame.SetPosition( screen_x / 2.0f - PlayGame_Size.X / 2.0f, screen_y / 1.64f + SclY( 32.0f ) );
				Frame_PlayGame.SetSize( PlayGame_Size.X + SclX( 128.0f ), PlayGame_Size.Y );
				Frame_PlayGame.TextureNum = 3;
				TitleBackground.AddFrame( Frame_PlayGame );


				Frame_Options.SetDisplayText( "OPTIONS" );
				Frame_Options.SetStyle( GameEnv.TileSetStyles.GetStyle( 1 ) );
				Frame_Options.GetTextDisplaySize( out Options_Size );
				Frame_Options.SetPosition( screen_x / 2.0f - Options_Size.X / 2.0f, screen_y / 1.64f - SclY( 32.0f ) );
				Frame_Options.SetSize( Options_Size.X + 1.0f, Options_Size.Y );
				Frame_Options.TextureNum = 3;
				TitleBackground.AddFrame( Frame_Options );


				Frame_Quit.SetDisplayText( "QUIT" );
				Frame_Quit.SetStyle( GameEnv.TileSetStyles.GetStyle( 1 ) );
				Frame_Quit.GetTextDisplaySize( out Quit_Size );
				Frame_Quit.SetPosition( screen_x / 2.0f - Quit_Size.X / 2.0f, screen_y / 1.64f - SclY( 96.0f ) );
				Frame_Quit.SetSize( Quit_Size.X + 1.0f, Quit_Size.Y );
				Frame_Quit.TextureNum = 3;

				TitleBackground.AddFrame( Frame_Quit );
				//printf("FrameAdress : %lx\n",(unsigned int)&Frame_PlayGame);
			}
			{

				if( Frame_PlayGame.Is_MouseIn() ) { Frame_PlayGame.SetColor( 0.5f, 0.5f, 1.0f ); }
				if( Frame_PlayGame.Is_MouseOut() ) { Frame_PlayGame.SetColor( 1.0f, 1.0f, 1.0f ); }
				if( Frame_Options.Is_MouseIn() ) { Frame_Options.SetColor( 0.5f, 0.5f, 1.0f ); }
				if( Frame_Options.Is_MouseOut() ) { Frame_Options.SetColor( 1.0f, 1.0f, 1.0f ); }
				if( Frame_Quit.Is_MouseIn() ) { Frame_Quit.SetColor( 0.5f, 0.5f, 1.0f ); }
				if( Frame_Quit.Is_MouseOut() ) { Frame_Quit.SetColor( 1.0f, 1.0f, 1.0f ); }
				if( Frame_PlayGame.Is_MouseClick() ) { ResultCode = ScreenChoices.PLAYGAME; }
				if( Frame_Options.Is_MouseClick() ) { ResultCode = ScreenChoices.OPTIONS; }
				if( Frame_Quit.Is_MouseClick() ) { ResultCode = ScreenChoices.QUIT; }
			}

			return ( ResultCode );
		}
	}
}
