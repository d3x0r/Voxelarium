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
using System.Diagnostics;
using System.IO;
using System.Text;
using Voxelarium.Common;
using Voxelarium.Core.Game;
using Voxelarium.Core.Game.GameWindows;
using Voxelarium.Core.Game.Screens;
using Voxelarium.Core.Support;
using Voxelarium.Core.UI;
using Voxelarium.Core.Voxels;
using Voxelarium.Core.Voxels.IO;
using Voxelarium.Core.Voxels.Physics;
using Voxelarium.Core.Voxels.Types;
using Voxelarium.Core.Voxels.UI;
using Voxelarium.LinearMath;

namespace Voxelarium.Core
{
	public partial class VoxelGameEnvironment
	{

		internal int percent_done;
		//ZVector3L ShipCenter; // Test

		public VoxelGameEnvironment()
		{
#if BUILD_ANDROID
			try
			{
				default_font = new FontRenderer( "/system/fonts/DroidSans.ttf", 24, 24 );
			}
			catch( Exception e )
			{
				Log.log( "font load failed : " + e.Message );
			}
			try
			{
				menu_font = new FontRenderer( "/system/fonts/DroidSans.ttf", 48, 48 );
			}
			catch( Exception e )
			{
			}
#else
			try
			{
				default_font = new FontRenderer( "Roboto-Regular.ttf", 24, 24 );
			}
			catch( Exception e )
			{
				Log.log( "font load failed : " + e.Message );
				try
				{
					default_font = new FontRenderer( "DroidSans.ttf", 24, 24 );
				}
				catch( Exception e2 )
				{
					Log.log( "font load failed : " + e2.Message );
				}
			}
			try
			{
				menu_font = new FontRenderer( "Roboto-Regular.ttf", 64, 64 );
			}
			catch( Exception e )
			{
				try
				{
					menu_font = new FontRenderer( "DroidSans.ttf", 32, 32 );
				}
				catch( Exception e2 )
				{
				}
			}
#endif
			percent_done = 1;
			Initialized_SDL =
			Initialized_GraphicMode =
			Initialized_TextureManager =
			Initialized_GuiManager =
			Initialized_OpenGLGameSettings =
			Initialized_Glew =
			Initialized_VoxelTypeManager =
			Initialized_EventManager =
			Initialized_TileSetsAndFonts =
			Initialized_Settings =
			Initialized_Renderer =
			Initialized_Game_Events =
			Initialized_Sound =
			Initialized_World =
			Initialized_PhysicEngine =
			Initialized_SectorLoader =
			Initialized_VoxelProcessor =
			Initialized_RendererSettings =
			Initialized_GameWindows =
			Initialized_ToolManager =
			Initialized_UserDataStorage =
			Initialized_WorldInfo =
			Initialized_GameEventSequencer = false;
			Font_1 = null;
			GuiTileset = null;
			//Settings_Hardware = null;
			UniverseNum = 1;
			Enable_LoadNewSector = true;

			TileSetStyles = null;
			//Time_GameLoop = 16.0;

#if FINISHED_PORTING
				for( int r = 0; r < 6; r++ )
					sack_camera[r] = 0;
				display_index = 0;
#endif
			//Menu_Up = false;
			//OptionScreen_Up = false;
			Game_Run = false;
				Initialized_GameStats = false;
				Enable_MVI = true;
				Enable_NewSectorRendering = true;
				frames = 0;
				frame_start = 0;
				//Time_FrameTime = 20;
				//Time_GameElapsedTime = 0;
				//VFov = 63.597825649;
				Machine_Serial = 1;
				Stop_Programmable_Robots = false;
		}
		~VoxelGameEnvironment() { UniverseNum = 0; }


		// Usefull directory

		string Path_UserData;

		string Path_Universes;
		string Path_ActualUniverse;
		string Path_UserTextures;
		string Path_UserScripts;
		string Path_UserScripts_UserData;

		// Flags

		internal bool Enable_MVI;           // Enable or disable massive voxel interraction and animation processing.
		internal bool Enable_LoadNewSector; // Enable new sector loading and rendering. Disable Locks to only loaded sectors.
		internal bool Enable_NewSectorRendering; // Enable to make display lists for new incoming sectors.
		internal bool Stop_Programmable_Robots; // This flag signal to user programmable robots to stop running as soon as possible.

		/* controlled directly by screens */
		Pages _page_up;
		internal Pages page_up
		{
			set { prior_page_up = _page_up; _page_up = value; }
			get { return _page_up; }
		}
		internal Pages prior_page_up;

		internal bool Game_Run;

		internal VoxelWorldProcessor VoxelProcessor;
		internal VoxelTypeManager VoxelTypeManager;
		internal VoxelTypeManager GetVoxelTypeManager() { return VoxelTypeManager; }
		internal TextureManager TextureManager;
		internal GraphicUserManager GuiManager;
		internal EventManager EventManager = new EventManager();
		internal TileSet.TileSetStyles TileSetStyles;
		internal FontRenderer default_font;
		internal FontRenderer menu_font;
		internal RenderInterface Basic_Renderer;
		internal Sound Sound;
		internal PhysicsEngine Engine;


		Actor ActiveActor;
		internal Actor GetActiveActor() { return ActiveActor; }

		/* Started things */
		internal EventSequencer GameEventSequencer;  // internal timer system
		internal GameStats GameStat;  // timing stats for rendering
		internal Game_Events Game_Events; // this is user input event dispatch

		// Game objects
		internal VoxelWorld World;
		SectorLoader SectorLoader;
#if FINISHED_PORTING
		// Game objects

			int VoxelBlockSize;
			// Jeu proprement dit

			ZActorPhysicEngine PhysicEngine;
			ZToolManager ToolManager;
			ZWorldInfo WorldInfo;

			// Game Windows
			ZGameWindow_VoxelTypeBar VoxelTypeBar;
			ZGameWindow_Storage GameWindow_Storage;
			ZGameWindow_Programmable GameWindow_Programmable;
			ZGameWindow_UserTextureTransformer GameWindow_UserTextureTransformer;
			ZGameWindow_ProgressBar GameProgressBar;
			ZGameWindow_DisplayInfos GameWindow_DisplayInfos;
			ZGameWindow_Sequencer GameWindow_Sequencer;
#endif
		GameWindow_Inventory GameWindow_Inventory;
		GameWindow_Advertising GameWindow_Advertising;

		bool Initialized_UserDataStorage;
		bool Initialized_Settings;
		bool Initialized_Glew;
		bool Initialized_SDL;
		bool Initialized_GraphicMode;
		bool Initialized_TextureManager;
		bool Initialized_EventManager;
		bool Initialized_GuiManager;
		bool Initialized_OpenGLGameSettings;
		bool Initialized_VoxelTypeManager;
		bool Initialized_TileSetsAndFonts;
		bool Initialized_Renderer;
		bool Initialized_Game_Events;
		bool Initialized_Sound;
		bool Initialized_World;
		bool Initialized_PhysicEngine;
		bool Initialized_SectorLoader;
		bool Initialized_VoxelProcessor;
		bool Initialized_RendererSettings;
		bool Initialized_GameWindows;
		bool Initialized_ToolManager;
		bool Initialized_GameStats;
		bool Initialized_WorldInfo;
		bool Initialized_GameEventSequencer;

		// timers

		//double Time_GameLoop;
		long frame_start;
		uint frames;
		//ulong Time_FrameTime; // Same as Time_GameLoop but in integer format;
		//ulong Time_GameElapsedTime;

		// Values

		uint Machine_Serial;       // Serial number for robots and/or machines that needs it.
		uint Previous_GameVersion; // Game version of the loaded world file.

		// General Inits
		int steps = 8;
		int step = 0;
		public bool Init( bool nogui = false )
		{
			bool result;
			result = Init_UserDataStorage(); if( !result ) return ( false );
			result = Init_Settings(); if( !result ) return ( false );
			percent_done = 1 + ( ++step * 100 ) / steps;
			result = Init_VoxelTypeManager( nogui ); if( !result ) return ( false );
			percent_done = 1 + ( ++step * 100 ) / steps;
			if( !nogui )
			{
				result = Init_TextureManager(); if( !result ) return ( false );
				percent_done = 1 + ( ++step * 100 ) / steps;
				result = Init_GuiManager(); if( !result ) return ( false );
				percent_done = 1 + ( ++step * 100 ) / steps;
				result = Init_TileSetsAndFonts(); if( !result ) return ( false );
				percent_done = 1 + ( ++step * 100 ) / steps;
				result = Init_Renderer(); if( !result ) return ( false );
				percent_done = 1 + ( ++step * 100 ) / steps;
				result = Init_Sound(); if( !result ) return ( false );
				percent_done = 1 + ( ++step * 100 ) / steps;
				InitScreens();
				percent_done = 1 + ( ++step * 100 ) / steps;
			}

			if( VoxelGlobalSettings.COMPILEOPTION_FABDATABASEOUTPUT )
			{
				Log.log( "**** Debug FabInfos incomplete ****" );
				//VoxelTypeManager.OutFabInfos();
				//VoxelTypeManager.FindFabConflics();
			}

			return ( true );
		}

		void InitScreens()
		{
			Screen_Main = new ScreenMain( Pages.MAIN_MENU );
			Screen_SlotSelection = new ScreenSlotSelect( Pages.SELECT_UNIVERSE );
			Screen_Loading = new ScreenLoading( Pages.LOADING_SCREEN );
			//Screen_ChooseOption = new ;
			//Screen_SlotSelection Screen_SlotSelection;
			//Screen_Loading Screen_Loading;
			//Screen_Saving Screen_Saving;
			//Screen_Options_Display Screen_Options_Display;

			//			Screen_Options_Sound Screen_Options_Sound;
			//
			//		Screen_Options_Game Screen_Options_Mouse;
			//
			//	Screen_Options_Keymap Screen_Options_Keymap;
			page_up = Pages.NONE;
			active_screen = Screen_Main;
			Basic_Renderer = new Render_Basic();
			Basic_Renderer.GameEnv = this;
		}

		internal int start_percent;
		int start_steps = 10;
		int start_step = 0;
		public bool Start_Game( bool nogui = false, bool client_networked = false )
		{
			bool result;
			start_step = 0;
			start_percent = 0;
			//Log.log( "Start Game 1" );
			result = Start_PersistGameEnv(); if( !result ) return ( false );
			//Log.log( "Start Game 2" );
			result = Start_GameEventSequencer(); if( !result ) return ( false );
			//    result = Start_WorldInfo();          if(!result) return(false);
			//Log.log( "Start Game 3" );
			result = Start_Game_Stats(); if( !result ) return ( false );
			//Log.log( "Start Game 4" );
			result = Start_Game_Events(); if( !result ) return ( false );

			//Log.log( "Start Game 5" );
			result = Start_World( nogui ); if( !result ) return ( false );
			start_percent = ( ++start_step * 100 ) / start_steps;
			
			//Log.log( "Start Game 6" );
			result = Start_SectorLoader( false, null ); if( !result ) return ( false );
			start_percent = ( ++start_step * 100 ) / start_steps;
			//Log.log( "Start Game 7" );
			result = Start_PhysicEngine(); if( !result ) return ( false );

			// preload sectors immeidately around the player
			//Log.log( "Start Game 8" );
			result = Start_WorldSectors( nogui, ref start_percent, ref start_step, ref start_steps ); if( !result ) return ( false );

			start_percent = ( ++start_step * 100 ) / start_steps;
			//Log.log( "Start Game 9" );
			result = Start_VoxelProcessor(); if( !result ) return ( false );
#if FINISHED_PORTING
				result = Start_ToolManager(); if( !result ) return ( false );
#endif
			//Log.log( "Start Game 10" );
			if( !nogui )
			{
				result = Start_RendererSettings(); if( !result ) return ( false );
				result = Start_GameWindows(); if( !result ) return ( false );
			}
			start_percent = ( ++start_step * 100 ) / start_steps;
			return ( true );
		}

		bool End_Game()
		{
			if( Initialized_GameWindows ) End_GameWindows();

			if( Initialized_SectorLoader ) End_SectorLoader();
			if( Initialized_VoxelProcessor ) End_VoxelProcessor();
			if( Initialized_PhysicEngine ) End_PhysicsEngine();
#if FINISHED_PORTING
				if( Initialized_RendererSettings ) End_RendererSettings();
				if( Initialized_ToolManager ) End_ToolManager();
				if( Initialized_World ) End_World();
				if( Initialized_Game_Events ) End_Game_Events();
				if( Initialized_GameStats ) End_Game_Stats();
				//    if (Initialized_WorldInfo)         End_WorldInfo();
				if( GameEventSequencer ) End_GameEventSequencer();
				End_PersistGameEnv();
				Sound.Stop_AllSounds();
#endif
			return ( true );
		}


		bool End()
		{
#if FINISHED_PORTING
				if( Initialized_Sound ) Cleanup_Sound( InitLog.Sec( 2120 ) );
				if( Initialized_Renderer ) Cleanup_Renderer( InitLog.Sec( 2110 ) );
				if( Initialized_GuiManager ) Cleanup_GuiManager( InitLog.Sec( 2090 ) );
				if( Initialized_EventManager ) Cleanup_EventManager( InitLog.Sec( 2080 ) );
				if( Initialized_OpenGLGameSettings ) Cleanup_OpenGLGameSettings( InitLog.Sec( 2070 ) );
				if( Initialized_TextureManager ) Cleanup_TextureManager( InitLog.Sec( 2060 ) );
				if( Initialized_VoxelTypeManager ) Cleanup_VoxelTypeManager( InitLog.Sec( 2050 ) );
				if( Initialized_Glew ) Cleanup_Glew( InitLog.Sec( 2040 ) );
				if( Initialized_GraphicMode ) Cleanup_GraphicMode( InitLog.Sec( 2030 ) );
				if( Initialized_SDL ) Cleanup_SDL( InitLog.Sec( 2020 ) );
				if( Initialized_TileSetsAndFonts ) Cleanup_TileSetsAndFonts( InitLog.Sec( 2100 ) );
				if( Initialized_UserDataStorage ) Cleanup_UserDataStorage( InitLog.Sec( 2000 ) );
#endif
			return ( true );
		}

		// TileSets

		internal TileSet Font_1;
		TileSet GuiTileset;

		enum FontSizes
		{
			FONTSIZE_1 = 0,
			FONTSIZE_2 = 3,
			FONTSIZE_3 = 4,
			FONTSIZE_4 = 1,
			FONTSIZE_5 = 2
		};
		enum TileTypes
		{
			Something
		}

		// InGame

		public int UniverseNum;
#if false
		void MoveShip()
			{
				ZVector3L VoxelCoords, Vx;
				VoxelLocation Loc;
				ZVector3d NewLocation;

				if( ShipCenter.x == 0 && ShipCenter.y == 0 && ShipCenter.z == 0 ) return;
				VoxelCoords = ShipCenter;



				for( Vx.x = VoxelCoords.x - 5; Vx.x < VoxelCoords.x + 5; Vx.x++ )
					for( Vx.z = VoxelCoords.z - 5; Vx.z < VoxelCoords.z + 5; Vx.z++ )
						for( Vx.y = VoxelCoords.y; Vx.y < VoxelCoords.y + 5; Vx.y++ )
						{
							World.MoveVoxel( Vx.x, Vx.y, Vx.z, Vx.x, Vx.y, Vx.z - 1, 0, true );
							World.GetVoxelLocation( &Loc, Vx.x, Vx.y, Vx.z );
							Loc.Sector.Flag_HighPriorityRefresh = true;
							World.GetVoxelLocation( &Loc, Vx.x, Vx.y, Vx.z - 1 );
							Loc.Sector.Flag_HighPriorityRefresh = true;
						}
				/
					World.GetVoxelLocation( &Loc, Vx.x+5, Vx.y, Vx.z );
					Loc.Sector.Flag_HighPriorityRefresh = true;
					World.GetVoxelLocation( &Loc, Vx.x-5, Vx.y, Vx.z );
					Loc.Sector.Flag_HighPriorityRefresh = true;
					World.GetVoxelLocation( &Loc, Vx.x, Vx.y, Vx.z+5 );
					Loc.Sector.Flag_HighPriorityRefresh = true;
					World.GetVoxelLocation( &Loc, Vx.x, Vx.y, Vx.z-5 );
					Loc.Sector.Flag_HighPriorityRefresh = true;
				/
				PhysicEngine.GetSelectedActor().ViewDirection.get_origin( NewLocation );
				NewLocation.z -= 256.0;
				PhysicEngine.GetSelectedActor().ViewDirection.translate( NewLocation );
			}


		};
#endif

		bool Init_UserDataStorage()
		{

			Log.log( "Starting : UserDataStorage Initialization" );

			// Directory for user data

			if( VoxelGlobalSettings.COMPILEOPTION_USEHOMEDIRSTORAGE )
			{
				Path_UserData = VStreamFile.Get_Directory_UserData();
			}
			else
				Path_UserData = ".";

			Path_UserData += "/" + VoxelGlobalSettings.COMPILEOPTION_SAVEFOLDERNAME;
			System.IO.Directory.CreateDirectory( Path_UserData );


			// Subdirectories

			Path_Universes = Path_UserData + "/Universes";
			Path_UserTextures = Path_UserData + "/UserTextures";

			System.IO.Directory.CreateDirectory( Path_Universes );
			System.IO.Directory.CreateDirectory( Path_UserTextures );

			Initialized_UserDataStorage = true;
			//Log.log( "Ended OK : UserDataStorage Initialization" );
			return ( true );
		}

		bool Init_VoxelTypeManager( bool nogui )
		{
			string Msg;
			//Log.log( "Starting : VoxelTypeManager Init" );
			//if (!Initialized_GraphicMode)    {ZString Err ="Can't init VoxelTypeManager : GraphicMode init not completed"; InitLog.Log(3, ZLog::FAIL, Err); return(false);}
			//if (!Initialized_Glew)           {ZString Err ="Can't init VoxelTypeManager : Glew init not completed"; InitLog.Log(4, ZLog::FAIL, Err); return(false);}
			VoxelTypeManager = new VoxelTypeManager();
			VoxelTypeManager.SetGameEnv( this );
			if( !VoxelTypeManager.LoadVoxelTypes( nogui ) ) { string Err = "Can't init VoxelTypeManager."; Log.log( Err ); return ( false ); }
			//Log.log( "Loaded " + VoxelTypeManager.GetTexturesCount() + " Voxel Textures." );
			if( VoxelTypeManager.GetTexturesCount() < 10 ) { string Err; Err = "Missing Texture files (count : " + VoxelTypeManager.GetTexturesCount() + ")"; Log.log( Err ); return ( false ); }
			if( null != VoxelTypeManager.GetVoxelType( 50 ) ) VoxelTypeManager.FillZeroSlots( 50 );
			else return ( false );
			Initialized_VoxelTypeManager = true;
			Log.log( "Ended Ok : VoxelTypeManager Init" );
			return ( true );
		}

		bool Init_TextureManager()
		{
			bool Result;
			string Path;
			string Err, ErrMsg;

			Log.log( "Starting : Texture Manager Init" );
			TextureManager = new TextureManager();
			Result = true;
			ErrMsg = "*** ERROR : Missing gui texture file ";

			/*0 */Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH + "gui/title_1_2.bmp"; Result = TextureManager.LoadTexture( Path, TextureID.MainMenuBackground ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			/*1 */Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH + "gui/blackvoxel_title_1_3.png"; Result = TextureManager.LoadTexture( Path, TextureID.TitleBanner ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			/*2 */Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH + "gui/panel_2.bmp"; Result = TextureManager.LoadTexture( Path, TextureID.Panel ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			/*3 */Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH + "gui/font_1_1.bmp"; Result = TextureManager.LoadTexture( Path, TextureID.OldFont, false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			///*4*/Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPAth+= "gui/title_1_2.bmp"; Result = TextureManager.LoadTexture( Path, TextureID., false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			/*5 */Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH + "gui/tile_1_1.bmp"; Result = TextureManager.LoadTexture( Path, TextureID.TitleA, false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			/*6 */Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH + "gui/tile_2_1.bmp"; Result = TextureManager.LoadTexture( Path, TextureID.TitleB, false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			/*7 */Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH + "gui/gui_tiles_1_2.bmp"; Result = TextureManager.LoadTexture( Path, TextureID.OldGuiTiles, false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			/*8 */Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH + "gui/dialog_background_1_2.bmp"; Result = TextureManager.LoadTexture( Path, TextureID.DialogBackground, false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			/*9 */Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH + "gui/dialog_voxeltype_1_1.bmp"; Result = TextureManager.LoadTexture( Path, TextureID.DialogVoxelTypeBackground, false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			/*10*/Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH + "gui/button_1_5.bmp"; Result = TextureManager.LoadTexture( Path, TextureID.Button, false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			/*11*/Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH + "gui/tile_2_2.bmp"; Result = TextureManager.LoadTexture( Path, TextureID.Title2, false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			/*12*/Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH + "gui/tile_3_1.bmp"; Result = TextureManager.LoadTexture( Path, TextureID.Title3, false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			/*13*/Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH + "gui/contribute_1_1.png"; Result = TextureManager.LoadTexture( Path, TextureID.Contribute, false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }

			Initialized_TextureManager = true;
			Log.log( "Ended Ok : Texture Manager Init" );
			return ( true );
		}

		bool Init_GuiManager()
		{
			Log.log( "Starting : GuiManager Init" );
			//if( !Initialized_TextureManager ) { Err = "Can't init GuiManager : TextureManager init not completed"; InitLog.Log( 4, ZLog::FAIL, Err ); return ( false ); }
			GuiManager = new GraphicUserManager();
            GuiManager.SetTextureManager( TextureManager );
			GuiManager.SetEventManager( EventManager );

			GuiManager.SetScreenDimensions( 0, 0, (float)2.0, (float)2.0 );
			//GuiManager.SetScreenDimensions(0,0,(float)ScreenResolution.x,(float)ScreenResolution.y);
			EventManager.AddConsumer_ToTail( GuiManager );

			Initialized_GuiManager = true;
			Log.log( "Ended Ok : GuiManager Init" );
			return ( true );
		}

		bool Init_TileSetsAndFonts()
		{
			Log.log( "Starting : Tilesets and Fonts Init" );
			if( !Initialized_TextureManager ) { string Err = "Can't init TileSets : TextureManager init not completed\n"; Log.log( Err ); return ( false ); }

			TileSetStyles = new TileSet.TileSetStyles();

			// Tile 1 : Font 1 Little Old

			Font_1 = new TileSet();
			Font_1.SetTextureManager( TextureManager );
			Font_1.SetTextureNum( TextureID.OldFont );
			Font_1.SetTextureSize( 128, 128 );
			Font_1.SetTileSlotSize( 8, 8 );
			Font_1.SetTileSize( 2 * ( 8.0f / 1920.0f ), 2 * ( 8.0f / 1080.0f ) );
			Font_1.SetTileOffset( 0, 0 );
			Font_1.SetTilesPerLine( 16 );
			Font_1.ComputeTileCoords();

			// Tile 2 Gui Tileset

			GuiTileset = new TileSet();
			GuiTileset.SetTextureManager( TextureManager );
			GuiTileset.SetTextureNum( TextureID.OldGuiTiles );
			GuiTileset.SetTextureSize( 512, 512 );
			GuiTileset.SetTileSlotSize( 32, 32 );
			GuiTileset.SetTileSize( 2 * ( 32.0f / 1920.0f ), 2 * ( 32.0f / 1080.0f ) );
			GuiTileset.SetTileOffset( 0, 0 );
			GuiTileset.SetTilesPerLine( 16 );
			GuiTileset.ComputeTileCoords();

			// Font Styles

			TileSetStyles.CreateStyle( 0, Font_1, 1, 1, 0, 0 );
			TileSetStyles.CreateStyle( 1, Font_1, 4, 4, 0, 0 );
			TileSetStyles.CreateStyle( 2, Font_1, 5, 4, 0, 0 );
			TileSetStyles.CreateStyle( 3, Font_1, 2, 2, 0, 0 );
			TileSetStyles.CreateStyle( 4, Font_1, 3, 3, 0, 0 );

			Initialized_TileSetsAndFonts = true;
			Log.log( "Ended Ok : Tilesets and Fonts Init" );
			return ( true );
		}

		bool Init_Renderer()
		{
			Log.log( "Starting : Renderer Init" );
			//ZRender_Basic Basic_Renderer;

#if FINISHED_WORLD_OUTPUT_RENDER
			//Basic_Renderer = new ZRender_Basic( World );
			Basic_Renderer = new ZRender_Smooth( World );

			Basic_Renderer.SetGameEnv( this );
			Basic_Renderer.Init();
			//Basic_Renderer.SetCamera(&Player);
			Basic_Renderer.SetVoxelTypeManager( &VoxelTypeManager );
			Basic_Renderer.SetTextureManager( &TextureManager );
			Basic_Renderer.current_gl_camera = psvInit - 1;
#endif
			Initialized_Renderer = true;
			Log.log( "Ended Ok : Renderer Init" );
			return ( true );
		}

		bool Init_Sound()
		{
			Log.log( "Starting : Sound Init" );
			Sound = new Sound();

			Sound.LoadSoundFiles();
			/*
			Msg.Clear() << "Loaded " << Sound.GetSampleCount() << " Sound samples."; InitLog.Log( 3, ZLog::INFO, Msg );
			if( Sound.GetSampleCount() < 8 ) { ZString Err; Err << "Missing Sound Sample Files (count : " << Sound.GetSampleCount() << ")"; InitLog.Log( 4, ZLog::FAIL, Err ); return ( false ); }

			Sound.SampleModify_Volume( 4, 0.5 );
			Sound.SampleModify_Volume( 5, 0.03 ); // Vrilleuse d'oreilles (0.1)
			Sound.SampleModify_Volume( 6, 0.3 ); // Bloc Break (0.3)
			Sound.SampleModify_Volume( 7, 0.3 ); // Bloc Place (0.3)

			*/
			Initialized_Sound = true;
			Log.log( "Ended Ok : Sound Init" );
			return ( true );
		}

		bool Start_GameEventSequencer()
		{
			GameEventSequencer = new EventSequencer();
			// GameEventSequencer.AddEvent(30000,10000,1,true, 600000);
			return  Initialized_GameEventSequencer = true;
		}

		bool Start_PersistGameEnv()
		{
			// Init the path to actual universe

			Path_ActualUniverse = Path_Universes; Path_ActualUniverse += "/" + UniverseNum;
			Path_UserScripts = Path_ActualUniverse; Path_UserScripts += "/Scripts";
			//Path_UserScripts_Squirrel = Path_UserScripts; Path_UserScripts_Squirrel.AddToPath( "Squirrel" );
			Path_UserScripts_UserData = Path_UserScripts; Path_UserScripts_UserData += "/UserData";

			// Create the directory for this universe if not exists.
			Directory.CreateDirectory( Path_ActualUniverse );
			Directory.CreateDirectory( Path_UserScripts );
			Directory.CreateDirectory( Path_UserScripts_UserData );

			// Loadind the worldinfo
			string Data = Path_ActualUniverse + "/WorldInfo.dat";
			if( File.Exists( Data ) )
				try
				{
					FileStream fs = new FileStream( Data, FileMode.Open );
					BinaryReader br = new BinaryReader( fs );

					uint MagicCookie = br.ReadUInt32();
					if( MagicCookie != 0xB14C08E1 ) { br.Close(); fs.Dispose();  return ( false ); }
					Previous_GameVersion = br.ReadUInt32();

					uint Compatibility_Class = br.ReadUInt32();
					if( Compatibility_Class >= 2 )
					{
						Machine_Serial = br.ReadUInt32();
					}
					br.Close();
					fs.Dispose();
				}
				catch( Exception e )
				{
					Log.log( "Unexpected exception reading universe data:" + e.Message );
				}
			return ( true );
		}

		bool Start_Game_Stats()
		{
			GameStat = new GameStats();
			//if( !GameStat ) { Initialized_GameStats = false; return ( false ); }

			return ( Initialized_GameStats = GameStat.Start() );
		}

		bool Start_Game_Events()
		{
			Game_Events = new Game_Events();

			Game_Events.SetEventManager( EventManager );
			Game_Events.SetGameEnv( this );

			EventManager.AddConsumer_ToTail( Game_Events );

			Initialized_Game_Events = true;
			return ( true );
		}
		bool Start_GameWindows()
		{
			GameWindow_Inventory = new GameWindow_Inventory(); GameWindow_Inventory.SetGameEnv( this );
			//VoxelTypeBar = new ZGameWindow_VoxelTypeBar; VoxelTypeBar.SetGameEnv( this );
			//GameWindow_Storage = new ZGameWindow_Storage; GameWindow_Storage.SetGameEnv( this );
			//GameWindow_Programmable = new ZGameWindow_Programmable; GameWindow_Programmable.SetGameEnv( this );
			//GameProgressBar = new ZGameWindow_ProgressBar; GameProgressBar.SetGameEnv( this );
			GameWindow_Advertising = new GameWindow_Advertising(); GameWindow_Advertising.SetGameEnv( this );
			//GameWindow_UserTextureTransformer = new ZGameWindow_UserTextureTransformer; GameWindow_UserTextureTransformer.SetGameEnv( this );
			//GameWindow_DisplayInfos = new ZGameWindow_DisplayInfos; GameWindow_DisplayInfos.SetGameEnv( this );
			//GameWindow_Sequencer = new ZGameWindow_Sequencer; GameWindow_Sequencer.SetGameEnv( this );

			GameWindow_Advertising.Show();
			Initialized_GameWindows = true;
			return ( true );
		}
		bool End_GameWindows()
		{
			// saving should be the screen up, and don't remove his frames.
			//while( prior_page_up != page_up )
			//	Relinquish();
			//GuiManager.RemoveAllFrames();
			/*
			VoxelTypeBar = null;
			GameWindow_Storage = null;
			GameWindow_UserTextureTransformer = null;
			GameWindow_Inventory = null;
			*/
			Initialized_GameWindows = false;
			return ( true );
		}
		bool Start_World( bool nogui )
		{
			World = new VoxelWorld( nogui, this );
			Render_Basic.BuildSortList( ref start_percent, ref start_step, ref start_steps );

			World.renderer = Basic_Renderer;

			World.SetUniverseNum( UniverseNum );
			start_step++;
			int tmp1 = 0, tmp2 = 0, tmp3 = 1;
			World.SetVoxelTypeManager( VoxelTypeManager, ref tmp1, ref tmp2, ref tmp3 );


			Initialized_World = true;
			return ( true );
		}
		bool Start_WorldSectors( bool nogui, ref int start_percent, ref int start_step, ref int start_steps  )
		{
			if( !nogui )
				World.CreateDemoWorld( ref start_percent, ref start_step, ref start_steps ); // force load initial zone.
			//World.SetVoxel( 0, 0, 0, 1 );// World.SetVoxel(0,1,0,1); World.SetVoxel(1,0,0,1); World.SetVoxel(0,2,0,2); World.SetVoxel(0,2,1,2);
			//World.SetVoxel(0,2,-1,2);World.SetVoxel(10,0,10,1); World.SetVoxel(10,1,10,1); World.SetVoxel(10,2,10,1); World.SetVoxel(5,3,0,1); World.SetVoxel(0,3,5,2);
			Initialized_World = true;
			return ( true );
		}

		bool Start_SectorLoader( bool networked, Networking.ClientConnection connection )
		{
			if( networked )
			{
				SectorLoader = new NetworkSectorLoader( this, connection );
				SectorLoader.SetVoxelTypeManager( VoxelTypeManager );
				SectorLoader.SetUniverseNum( UniverseNum );
			}
			else
			{
				IWorldGenesis genesis;
				genesis = new Genesis();
				//IWorldGenesis genesis = Compiler.LoadGenesisCode();
				if( genesis != null )
				{
					SectorLoader = new FileSectorLoader( this, genesis );
					SectorLoader.SetVoxelTypeManager( VoxelTypeManager );
					SectorLoader.SetUniverseNum( UniverseNum );
					if( !SectorLoader.Init( ref start_percent, ref start_step, ref start_steps ) ) return ( false );
					World.SetSectorLoader( SectorLoader );
					Initialized_SectorLoader = true;
					return ( true );
				}
			}
			return false;
		}
		internal bool End_SectorLoader()
		{
			if( SectorLoader != null )
			{
				SectorLoader.Cleanup();
				Initialized_SectorLoader = false;
				SectorLoader = null;
			}
			return ( true );
		}

		bool Start_VoxelProcessor()
		{
			// Compute sector ejection distance.

			float EjectionDistance, h, v;

			h = (float)Settings_Hardware.RenderingDistance_Horizontal;
			v = (float)Settings_Hardware.RenderingDistance_Vertical;
			EjectionDistance = (float)(Math.Sqrt( ( h * h ) + ( h * h ) + ( v * v ) ) * 1.5);

			// Init the voxel processor

			VoxelProcessor = new VoxelWorldProcessor();

			VoxelProcessor.SetWorld( World );
			VoxelProcessor.SetPlayerPosition( ref btVector3.Zero );
			VoxelProcessor.SetSectorEjectDistance( EjectionDistance );
			VoxelProcessor.SetGameEnv( this );
			VoxelProcessor.Start();

			Initialized_VoxelProcessor = true;
			return ( true );
		}

		bool End_VoxelProcessor()
		{
			if( VoxelProcessor != null )
			{
				VoxelProcessor.End();
				Initialized_VoxelProcessor = false;
				VoxelProcessor = null;
			}
			return ( true );
		}

		bool Start_PhysicEngine()
		{
			Engine = new PhysicsEngine();
			Update += VoxelGameEnvironment_Update;
			Initialized_PhysicEngine = true;
			return true;
		}

		private void VoxelGameEnvironment_Update( double timeDelta )
		{
			if( Game_Run )
				Engine.Step( (float)timeDelta );
        }

		bool End_PhysicsEngine()
		{
			Engine.Dispose();
			Engine = null;
			return true;
		}

		bool Init_Settings(  )
		{
			bool Res;

			Log.log( "Starting : Settings" );
			Res = Settings_Hardware.Load(); // If loading fail, continue anyway with default.

			if( Res ) Log.log( "Info : Hardware Settings Loaded From File" );
			else Log.log( "Info : Can't load Settings from file, default settings taken." );

			Initialized_Settings = true;

			Log.log( "Ended OK : Settings" );
			return ( true );
		}

		bool Start_RendererSettings()
		{
			Basic_Renderer.SetRenderSectorRadius( Settings_Hardware.RenderingDistance_Horizontal, Settings_Hardware.RenderingDistance_Vertical );
			//Basic_Renderer.SetWorld( World );
			//Basic_Renderer.SetActor( PhysicEngine.GetSelectedActor() );

			//Basic_Renderer.SetCamera(&PhysicEngine.GetSelectedActor().Camera);

			//Basic_Renderer.SetPointedVoxel( &PhysicEngine.GetSelectedActor().PointedVoxel );
			//Basic_Renderer.SetViewportResolution( ScreenResolution );
			Basic_Renderer.SetPixelAspectRatio( Settings_Hardware.PixelAspectRatio );
			Basic_Renderer.SetSectorCullingOptimisationFactor( Settings_Hardware.Opt_SectCFactor );
			Initialized_RendererSettings = true;
			return ( true );
		}

		bool End_RendererSettings()
		{
			//Basic_Renderer.SetWorld( null );
			//Basic_Renderer.SetCamera( null );
			//Basic_Renderer.SetActor( null );
			//Basic_Renderer.SetPointedVoxel( null );
			Initialized_RendererSettings = false;
			return ( false );
		}

	}
}
