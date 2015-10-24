using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using Voxelarium.Core.Game.Screens;
using Voxelarium.Core.Support;
using Voxelarium.Core.UI;

namespace Voxelarium.Core
{
	public partial class VoxelGameEnvironment
	{
		internal enum Pages
		{
			NONE = -1
			, MAIN_MENU
			, SETTINGS
			, SELECT_UNIVERSE
			, LOADING_SCREEN
			, SETTINGS_DISPLAY
			, SETTINGS_SOUND
			, SETTINGS_MOUSE
			, SETTINGS_KEYMAP
			, GAME_WORLD_1
			, SAVE_GAME
		};
		//HighPerfTimer Timer;
		HighPerfTimer Timer_Draw;
		HighPerfTimer PhysicsTimer;
		HighPerfTimer PhysicsTimer_Compute;

		//		Screen_ChooseOption Screen_ChooseOption;
		//Screen_SlotSelection Screen_SlotSelection;
		//Screen_Loading Screen_Loading;
		//Screen_Saving Screen_Saving;
		//Screen_Options_Display Screen_Options_Display;

		//Screen_Options_Sound Screen_Options_Sound;

		//Screen_Options_Game Screen_Options_Mouse;

		//Screen_Options_Keymap Screen_Options_Keymap;
		ScreenLoading Screen_Loading;
		ScreenMain Screen_Main;
		internal ScreenSlotSelect Screen_SlotSelection; 

		Screen _active_screen;
		Screen prior_active_screen;
		internal Screen active_screen
		{
			set { prior_active_screen = _active_screen; _active_screen = value; }
			get { return _active_screen; }
		}

		static void LoadGame( object o )
		{
			VoxelGameEnvironment game = o as VoxelGameEnvironment;
			if( !game.Start_Game() )
				game.active_screen = game.Screen_Main;
		}

		// return false to Exit()
		public bool Update()
		{
			//Timer.Start();
			if( _active_screen != null )
			{
				Screen.ScreenChoices Result = _active_screen.ProcessScreen( this );
				if( Result >= Screen.ScreenChoices.SlotChoice1 && Result <= Screen.ScreenChoices.SlotChoice16 )
				{
					active_screen = Screen_Loading;
					Thread thread = new Thread( LoadGame );
					thread.Start( this );
				}
                else switch( Result )
				{
					case Screen.ScreenChoices.CHOICE_RETURN:
						_active_screen = prior_active_screen;
						break;
					case Screen.ScreenChoices.PLAYGAME:
						active_screen = Screen_SlotSelection;
						break;
					case Screen.ScreenChoices.NONE:
						break;
					case Screen.ScreenChoices.SAME_SCREEN:
						break;
					case Screen.ScreenChoices.QUIT:
						return false; // return false to Exit();
				}
			}
#if PHYSICS_COMPLETED
			if( PhysicEngine != null )
			{
				// Game Events.
				PhysicsTimer_Compute.Start();

				GameEventSequencer.ProcessEvents( PhysicEngine.GetSelectedActor().Time_TotalGameTime );

				PhysicsTimer.End();
				FrameTime = Time_GameLoop = PhysicsTimer.GetResult() / 1000.0;
				Time_FrameTime = PhysicsTimer.GetResult();
				if( Time_GameLoop > 64.0 ) Time_GameLoop = 64.0; // Game cannot make too long frames because inaccuracy. In this case, game must slow down.
				Time_GameElapsedTime += Time_FrameTime;
				PhysicEngine.DoPhysic( Time_FrameTime );

				// Voxel Processor Get Player Position.
				if( VoxelProcessor )
				{
					ZActor* Actor;
					Actor = PhysicEngine.GetSelectedActor();
					VoxelProcessor.SetPlayerPosition( Actor.ViewDirection.x(), Actor.ViewDirection.y(), Actor.ViewDirection.z() );
				}
				GameStat.FrameTime = (ULong)FrameTime;
				GameStat.DoLogRecord();

				PhysicsTimer_Compute.End();

				PhysicsTimer.Start();
			}
			if( Basic_Renderer )
			{
				if( Basic_Renderer.Camera )
				{
					for( int n = 0; n < 16; n++ )
						( (float*)origin )[n] = (float)Basic_Renderer.Camera.orientation.m[0][n];
				}
				ReadableDisplayCounter += FrameTime;
				if( GameWindow_DisplayInfos
					&& VoxelProcessor
					&& GameWindow_DisplayInfos.Is_Shown() )
				{
					if( ReadableDisplayCounter > 500.0 )
					{
						ReadableDisplayCounter = 0.0;
						ZString As;

						As = "FPS: "; As << (ULong)( 1000.0 * frames / ( timeGetTime() - frame_start ) ) << " FTM: " << ( Timer_Draw.GetResult() / 1000.0 );
						As << " MVI Time: " << (ULong)( VoxelProcessor.Timer.GetResult() / 1000.0 ) << " " << (ULong)( VoxelProcessor.Timer_Compute.GetResult() / 1000.0 );
						As << " PT: " << (ULong)( PhysicsTimer_Compute.GetResult() / 1000.0 );
						frame_start = timeGetTime();
						frames = 0;
						GameWindow_DisplayInfos.SetText( &As );
						As = "Direction: ";
						As << PhysicEngine.GetSelectedActor().Camera.orientation.yaw();
						As << " ";
						As << PhysicEngine.GetSelectedActor().Camera.orientation.pitch();
						As << " ";
						As << PhysicEngine.GetSelectedActor().Camera.orientation.roll();
						As << " ";
						As << PhysicEngine.GetSelectedActor().Camera.orientation.m[0][0];
						GameWindow_DisplayInfos.SetText2( &As );
						As = "Origin: ";
						As << PhysicEngine.GetSelectedActor().Camera.orientation.origin().x;
						As << " ";
						As << PhysicEngine.GetSelectedActor().Camera.orientation.origin().y;
						As << " ";
						As << PhysicEngine.GetSelectedActor().Camera.orientation.origin().z;
						As << " ";
						As << PhysicEngine.GetSelectedActor().Camera.orientation.m[0][0];
						GameWindow_DisplayInfos.SetText3( &As );
					}
				}

			}
#endif
			return true; // allow continued play.
		}
		public bool Draw( Display display )
		{
			Screen.ScreenChoices Result = Screen.ScreenChoices.SAME_SCREEN;
			Timer_Draw.Start();

			if( frames == 0 )
				frame_start = HighPerfTimer.GetActualTime();
			frames++;


			switch( page_up )
			{
				case 0:
					switch( Result )
					{
						case Screen.ScreenChoices.QUIT:     // Quit the game
							display.Exit();
							break;

						case Screen.ScreenChoices.OPTIONS:  // Option Section
							page_up = Pages.SETTINGS;
							break;

						case Screen.ScreenChoices.PLAYGAME: // Play the game
							page_up = Pages.SELECT_UNIVERSE;
							break;
					}
					break;
				case Pages.SETTINGS:
					{
#if asdfasdf
						Screen_ChooseOption.ProcessScreen( Ge );
						switch( Screen_ChooseOption.ResultCode )
						{
							case Screen.ScreenChoices.QUIT: { page_up = Pages.MAIN_MENU; break; }
							case Screen.ScreenChoices.DISPLAY: { page_up = Pages.SETTINGS_DISPLAY; break; }
							case Screen.ScreenChoices.SOUND: { page_up = Pages.SETTINGS_SOUND; break; }
							case Screen.ScreenChoices.MOUSE: { page_up = Pages.SETTINGS_MOUSE; break; }
							case Screen.ScreenChoices.KEYMAP: { page_up = Pages.SETTINGS_KEYMAP; break; }
						}
#endif
					}
					break;
				case Pages.SAVE_GAME:
					//Screen_Saving.ProcessScreen( Ge );
					break;
				case Pages.SETTINGS_DISPLAY:
					{
#if asdfasdf
						if( Screen_Options_Display.ProcessScreen( Ge ) == Screen.ScreenChoices.QUIT )
							page_up = Pages.MAIN_MENU;
#endif
						break;
					}

				case Pages.SETTINGS_SOUND:
					{
#if asdfasdf
						if( Screen_Options_Sound.ProcessScreen( Ge ) == Screen.ScreenChoices.QUIT )
							page_up = Pages.MAIN_MENU;
#endif
						break;
					}

				case Pages.SETTINGS_MOUSE:
					{
#if asdfasdf
						if( Screen_Options_Mouse.ProcessScreen( Ge ) == Screen.ScreenChoices.QUIT )
							page_up = Pages.MAIN_MENU;
#endif
						break;
					}

				case Pages.SETTINGS_KEYMAP:
					{
#if asdfasdf
						if( Screen_Options_Keymap.ProcessScreen( Ge ) == Screen.ScreenChoices.QUIT )
							page_up = Pages.MAIN_MENU;
#endif
						break;
					}
				case Pages.GAME_WORLD_1:
					if( prior_page_up != page_up )
					{
						prior_page_up = page_up;
						GuiManager.RemoveAllFrames();
					}
					break;
				case Pages.SELECT_UNIVERSE:

#if asdfasdf
					UniverseNum = Screen_SlotSelection.ProcessScreen( Ge );
					if( UniverseNum )
						page_up = Pages.LOADING_SCREEN;
#endif
					break;
				case Pages.LOADING_SCREEN:
#if asdfasdf

					Screen_Loading.ProcessScreen( Ge );
					( *pStartGame ) = true;
#endif
					break;
			}

#if asdfasdf
			Basic_Renderer.Aspect_Ratio = sack_aspect[psvInit - 1];
#endif
			if( Game_Run )
			{
				// Rendering
#if asdfasdf
				if( Basic_Renderer )
				{
						GameWindow_Advertising.Advertising_Actions( (double)FrameTime );
						ToolManager.ProcessAndDisplay();
					Basic_Renderer.Render( true );
				}
#endif
			}

			GuiManager.Render( display );
			//Timer.End();
			Timer_Draw.End();

			return true;
		}
	}
}
