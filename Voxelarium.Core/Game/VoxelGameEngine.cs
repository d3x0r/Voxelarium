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
using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using Voxelarium.Core.Game.Screens;
using Voxelarium.Core.Support;
using Voxelarium.Core.UI;
using Voxelarium.Common;
using OpenTK.Input;
using Voxelarium.Core.Networking;

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
			, SELECT_PLAY_TYPE
			, CONNECTING_TO_MASTER_SERVER
			, SELECT_SERVER
		};
		//HighPerfTimer Timer;
		HighPerfTimer Timer_Draw;
		//HighPerfTimer PhysicsTimer;
		//HighPerfTimer PhysicsTimer_Compute;
		//uint FrameTime;
		//		Screen_ChooseOption Screen_ChooseOption;
		//Screen_SlotSelection Screen_SlotSelection;
		//Screen_Saving Screen_Saving;
		//Screen_Options_Display Screen_Options_Display;

		//Screen_Options_Sound Screen_Options_Sound;

		//Screen_Options_Game Screen_Options_Mouse;

		//Screen_Options_Keymap Screen_Options_Keymap;
		ScreenLoading Screen_Loading;
		ScreenMain Screen_Main;
		ScreenPlaySelect Screen_Play_Select;
		ScreenConnecting Screen_Connecting;
		internal MasterServerConnection Master_Server_Connection;
		ScreenSelectServer Screen_Select_Server;
		internal ScreenSlotSelect Screen_SlotSelection; 

		Screen _active_screen;
		Screen prior_active_screen;
		internal Screen active_screen
		{
			set { prior_active_screen = _active_screen; _active_screen = value;
				if( value == null ) GuiManager.RemoveAllFrames();
					}
			get { return _active_screen; }
		}

		static void LoadGame( object o )
		{
			VoxelGameEnvironment game = o as VoxelGameEnvironment;
			if( !game.Start_Game() )
			{
				game.active_screen = game.Screen_Main;
			}
			else
			{
				game.Game_Run = true;
				game.active_screen = null;
			}
		}

		public delegate void OnUpdate( double timeDelta );
		event OnUpdate Update;

		// return false to Exit()
		internal bool DoUpdate( KeyboardDevice keyboard, double timeDelta )
		{
			//Timer.Start();
			if( Update != null )
				Update( timeDelta );

			if( Game_Events != null )
				Game_Events.Process_StillEvents( keyboard ); // Process repeating checked events.

			if( _active_screen != null )
			{
				Screen.ScreenChoices Result = _active_screen.ProcessScreen( this );
				if( Result >= Screen.ScreenChoices.SlotChoice1 && Result <= Screen.ScreenChoices.SlotChoice16 )
				{
					Display.HideMouse();
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
						active_screen = Screen_Play_Select;
						break;
					case Screen.ScreenChoices.PLAYLOCAL:
						active_screen = Screen_SlotSelection;
						break;
					case Screen.ScreenChoices.PLAYREMOTE:
						Master_Server_Connection = new MasterServerConnection();
						active_screen = Screen_Connecting;
						break;
					case Screen.ScreenChoices.SELECT_SERVER:
						active_screen = Screen_Select_Server;
						break;
                    case Screen.ScreenChoices.NONE:
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
		public bool Draw( Display display, float game_time )
		{
			Timer_Draw.Start();

			if( frames == 0 )
				frame_start = HighPerfTimer.GetActualTime();
			frames++;

			if( Game_Run )
			{
				// Rendering
				if( Basic_Renderer != null )
				{
					// Process Input events (Mouse, Keyboard)
					// this is handeld from callbacks
					//EventManager.ProcessEvents();       // Process incoming events.


					// Process incoming sectors from the make/load working thread
					World.ProcessNewLoadedSectors( );
					// Sector Ejection processing.
					World.ProcessOldEjectedSectors();

					// if (MoveShipCounter>125 ) {GameEnv.MoveShip(); MoveShipCounter = 0; }
					// Player physics
					// PhysicEngine.DoPhysic( GameEnv.Time_FrameTime );

					// Voxel Processor Get Player Position.
					//ZActor* Actor;
					//Actor = GameEnv.PhysicEngine.GetSelectedActor();
					//GameEnv.VoxelProcessor.SetPlayerPosition( Actor.Location.x, Actor.Location.y, Actor.Location.z );

					// Advertising messages
					GameWindow_Advertising.Advertising_Actions( game_time );
					//ToolManager.ProcessAndDisplay();
					Basic_Renderer.Render( display, this, World );
					//if( Ge && Ge->Gui )
				}
			}

			if( GuiManager != null )
				GuiManager.Render( display );

			//Timer.End();
			Timer_Draw.End();

			return true;
		}
	}
}
