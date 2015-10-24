using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using Voxelarium.Core.Game;
using Voxelarium.Core.Game.Screens;
using Voxelarium.Core.Support;
using Voxelarium.Core.UI;
using Voxelarium.Core.Voxels;
using Voxelarium.Core.Voxels.IO;
using Voxelarium.Core.Voxels.Types;
using Voxelarium.Core.Voxels.UI;

namespace Voxelarium.Core
{
	public partial class VoxelGameEnvironment
	{

		internal int percent_done;
		internal int percent_started;
		//ZVector3L ShipCenter; // Test

		public VoxelGameEnvironment()
		{
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
#if FINISHED_PORTING
			TileSetStyles = null;
				Time_GameLoop = 16.0;

				for( int r = 0; r < 6; r++ )
					sack_camera[r] = 0;
				//Menu_Up = false;
				//OptionScreen_Up = false;
				Game_Run = false;
				display_index = 0;
				GameWindow_Advertising = 0;
				Sound = 0;
				GameWindow_UserTextureTransformer = 0;
				GameWindow_Sequencer = 0;
				GameStat = 0;
				Initialized_GameStats = false;
				WorldInfo = 0;
				Mouse_captured = false;
				Mouse_relative = false;
				Enable_MVI = true;
				Enable_LoadNewSector = true;
				Enable_NewSectorRendering = true;
				GameEventSequencer = 0;
				frames = 0;
				frame_start = 0;
				Time_FrameTime = 20;
				Time_GameElapsedTime = 0;
				VFov = 63.597825649;
				Machine_Serial = 1;
				Stop_Programmable_Robots = false;
#endif
		}
		~VoxelGameEnvironment() { UniverseNum = 0; }

		//ZLog InitLog;

		//
		SaltyRandomGenerator Random;

		//ZPointList PointList;

		// Usefull directory

		string Path_GameFiles;
		string Path_UserData;

		string Path_Universes;
		string Path_ActualUniverse;
		string Path_UserTextures;
		string Path_UserScripts;
		string Path_UserScripts_Squirrel;
		string Path_UserScripts_UserData;

		// Flags

		internal bool Enable_MVI;           // Enable or disable massive voxel interraction and animation processing.
		internal bool Enable_LoadNewSector; // Enable new sector loading and rendering. Disable Locks to only loaded sectors.
		internal bool Enable_NewSectorRendering; // Enable to make display lists for new incoming sectors.
		internal bool Stop_Programmable_Robots; // This flag signal to user programmable robots to stop running as soon as possible.


		// Game Loop continue flag
		bool Mouse_relative;
		bool Mouse_captured;

		/* controlled directly by screens */
		Pages _page_up;
		internal Pages page_up
		{
			set { prior_page_up = _page_up; _page_up = value; }
			get { return _page_up; }
		}
		internal Pages prior_page_up;

		bool Game_Run;

		internal VoxelTypeManager VoxelTypeManager;
		internal VoxelTypeManager GetVoxelTypeManager() { return VoxelTypeManager; }
        internal TextureManager TextureManager;
		internal GraphicUserManager GuiManager;
		internal EventManager EventManager = new EventManager();
		internal TileSet.TileSetStyles TileSetStyles;
		internal RenderInterface Basic_Renderer;
		internal Sound Sound;
		Actor ActiveActor;
		internal Actor GetActiveActor() { return ActiveActor; }

		/* Started things */
		EventSequencer GameEventSequencer;  // internal timer system
		internal GameStats GameStat;  // timing stats for rendering
		Game_Events Game_Events; // this is user input event dispatch

		// Game objects
		internal VoxelWorld World;
		FileSectorLoader SectorLoader;
#if FINISHED_PORTING
		// Game objects

			int VoxelBlockSize;
			// Jeu proprement dit

			ZActorPhysicEngine PhysicEngine;
			ZVoxelProcessor VoxelProcessor;
			ZToolManager ToolManager;
			ZWorldInfo WorldInfo;

			// Game Windows
			ZGameWindow_Inventory GameWindow_Inventory;
			ZGameWindow_VoxelTypeBar VoxelTypeBar;
			ZGameWindow_Storage GameWindow_Storage;
			ZGameWindow_Programmable GameWindow_Programmable;
			ZGameWindow_UserTextureTransformer GameWindow_UserTextureTransformer;
			ZGameWindow_ProgressBar GameProgressBar;
			ZGameWindow_Advertising GameWindow_Advertising;
			ZGameWindow_DisplayInfos GameWindow_DisplayInfos;
			ZGameWindow_Sequencer GameWindow_Sequencer;
#endif

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

		// Screen Informations

		double VFov;               // Vertical Fov;

		// timers

		double Time_GameLoop;
		long frame_start;
		uint frames;
		ulong Time_FrameTime; // Same as Time_GameLoop but in integer format;
		ulong Time_GameElapsedTime;

		// Values

		uint Machine_Serial;       // Serial number for robots and/or machines that needs it.
		uint Previous_GameVersion; // Game version of the loaded world file.

		// General Inits
#if FINISHED_PORTING

			bool Cleanup_UserDataStorage( ZLog InitLog );
			bool Cleanup_TextureManager( ZLog InitLog );
			bool Cleanup_VoxelTypeManager( ZLog InitLog );
			bool Cleanup_GuiManager( ZLog InitLog );
			bool Cleanup_EventManager( ZLog InitLog );
			bool Cleanup_OpenGLGameSettings( ZLog InitLog );
			bool Cleanup_TileSetsAndFonts( ZLog InitLog );
			bool Cleanup_Renderer( ZLog InitLog );
			bool Cleanup_Sound( ZLog InitLog );

			// Specific game Settings.

			bool Start_Game_Events();
			bool Start_World();
			bool Start_PhysicEngine();
			bool Start_SectorLoader();
			bool Start_VoxelProcessor();
			bool Start_RendererSettings();
			bool Start_GameWindows();
			bool Start_ToolManager();
			bool Start_PersistGameEnv();

			bool End_WorldInfo();
			bool End_Game_Events();
			bool End_World();
			bool End_PhysicEngine();
			void SaveWorld();

			bool End_SectorLoader();
			bool End_VoxelProcessor();
			bool End_RendererSettings();
			bool End_GameWindows();
			bool End_ToolManager();
			bool End_Game_Stats();
			bool End_GameEventSequencer();
			bool End_PersistGameEnv();
#endif
		int steps = 8;
		int step = 0;
		internal bool Init()
		{
			bool result;
			result = Init_UserDataStorage(); if( !result ) return ( false );
			percent_done = 1 + ( ++step * 100 ) / steps;
			result = Init_VoxelTypeManager(); if( !result ) return ( false );
			percent_done = 1 + ( ++step * 100 ) / steps;
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
        }

		internal bool Start_Game()
		{
			bool result;

			result = Start_PersistGameEnv(); if( !result ) return ( false );
			result = Start_GameEventSequencer(); if( !result ) return ( false );
			//    result = Start_WorldInfo();          if(!result) return(false);
			result = Start_Game_Stats(); if( !result ) return ( false );
			result = Start_Game_Events(); if( !result ) return ( false );
			result = Start_World(); if( !result ) return ( false );

			result = Start_SectorLoader(); if( !result ) return ( false );
#if FINISHED_PORTING
				result = Start_ToolManager(); if( !result ) return ( false );
				result = Start_PhysicEngine(); if( !result ) return ( false );
				result = Start_VoxelProcessor(); if( !result ) return ( false );
				result = Start_RendererSettings(); if( !result ) return ( false );
#endif
			result = Start_GameWindows(); if( !result ) return ( false );
			return ( true );
		}

		bool End_Game()
		{
#if FINISHED_PORTING
				if( Initialized_GameWindows ) End_GameWindows();
				if( Initialized_VoxelProcessor ) End_VoxelProcessor();
				if( Initialized_RendererSettings ) End_RendererSettings();
				if( Initialized_SectorLoader ) End_SectorLoader();
				if( Initialized_PhysicEngine ) End_PhysicEngine();
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
				if( Initialized_Settings ) Cleanup_Settings( InitLog.Sec( 2010 ) );
				if( Initialized_UserDataStorage ) Cleanup_UserDataStorage( InitLog.Sec( 2000 ) );
#endif
			return ( true );
		}

		// TileSets

		TileSet Font_1;
		TileSet GuiTileset;

		enum FontSizes
		{
			FONTSIZE_1 = 0,
			FONTSIZE_2 = 3,
			FONTSIZE_3 = 4,
			FONTSIZE_4 = 1,
			FONTSIZE_5 = 2
		};


		// InGame

		internal int UniverseNum;
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
			string ErrorMsg;

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

			// Directory for game files

			Path_GameFiles = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH;

			// Subdirectories

			Path_Universes = Path_UserData + "/Universes";
			Path_UserTextures = Path_UserData + "/UserTextures";

			System.IO.Directory.CreateDirectory( Path_Universes );
			System.IO.Directory.CreateDirectory( Path_UserTextures );

			Initialized_UserDataStorage = true;
			Log.log( "Ended OK : UserDataStorage Initialization" );
			return ( true );
		}

		bool Init_VoxelTypeManager()
		{
			string Msg;
			Log.log( "Starting : VoxelTypeManager Init" );
			//if (!Initialized_GraphicMode)    {ZString Err ="Can't init VoxelTypeManager : GraphicMode init not completed"; InitLog.Log(3, ZLog::FAIL, Err); return(false);}
			//if (!Initialized_Glew)           {ZString Err ="Can't init VoxelTypeManager : Glew init not completed"; InitLog.Log(4, ZLog::FAIL, Err); return(false);}
			VoxelTypeManager = new VoxelTypeManager();
			VoxelTypeManager.SetGameEnv( this );
			if( !VoxelTypeManager.LoadVoxelTypes() ) { string Err = "Can't init VoxelTypeManager."; Log.log( Err ); return ( false ); }
			Msg = "Loaded " + VoxelTypeManager.GetTexturesCount() + " Voxel Textures.";
			Log.log( Msg );
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

			Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH; Path += "/gui/title_1_2.bmp"; Result = TextureManager.LoadTexture( Path, 0 ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH; Path += "/gui/blackvoxel_title_1_3.bmp"; Result = TextureManager.LoadTexture( Path, 1 ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH; Path += "/gui/panel_2.bmp"; Result = TextureManager.LoadTexture( Path, 2 ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH; Path += "/gui/font_1_1.bmp"; Result = TextureManager.LoadTexture( Path, 3, false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH; Path += "/gui/title_1_2.bmp"; Result = TextureManager.LoadTexture( Path, 4, false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH; Path += "/gui/tile_1_1.bmp"; Result = TextureManager.LoadTexture( Path, 5, false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH; Path += "/gui/tile_2_1.bmp"; Result = TextureManager.LoadTexture( Path, 6, false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH; Path += "/gui/gui_tiles_1_2.bmp"; Result = TextureManager.LoadTexture( Path, 7, false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH; Path += "/gui/dialog_background_1_2.bmp"; Result = TextureManager.LoadTexture( Path, 8, false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH; Path += "/gui/dialog_voxeltype_1_1.bmp"; Result = TextureManager.LoadTexture( Path, 9, false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH; Path += "/gui/button_1_5.bmp"; Result = TextureManager.LoadTexture( Path, 10, false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH; Path += "/gui/tile_2_2.bmp"; Result = TextureManager.LoadTexture( Path, 11, false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH; Path += "/gui/tile_3_1.bmp"; Result = TextureManager.LoadTexture( Path, 12, false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }
			Path = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH; Path += "/gui/contribute_1_1.bmp"; Result = TextureManager.LoadTexture( Path, 13, false ); if( !Result ) { Err = ErrMsg + Path; Log.log( Err ); return ( false ); }

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
			Font_1.SetTextureNum( 3 );
			Font_1.SetTextureSize( 128, 128 );
			Font_1.SetTileSlotSize( 8, 8 );
			Font_1.SetTileSize( 2 * ( 8.0f / 1920.0f ), 2 * ( 8.0f / 1080.0f ) );
			Font_1.SetTileOffset( 0, 0 );
			Font_1.SetTilesPerLine( 16 );
			Font_1.ComputeTileCoords();

			// Tile 2 Gui Tileset

			GuiTileset = new TileSet();
			GuiTileset.SetTextureManager( TextureManager );
			GuiTileset.SetTextureNum( 7 );
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

			Basic_Renderer->SetGameEnv( this );
			Basic_Renderer->Init();
			//Basic_Renderer.SetCamera(&Player);
			Basic_Renderer->SetVoxelTypeManager( &VoxelTypeManager );
			Basic_Renderer->SetTextureManager( &TextureManager );
			Basic_Renderer->current_gl_camera = psvInit - 1;
			Basic_Renderer->LoadVoxelTexturesToGPU( psvInit - 1 );
			Basic_Renderer->LoadTexturesToGPU( psvInit - 1 );
#endif
			Initialized_Renderer = true;
			Log.log( "Ended Ok : Renderer Init" );
			return ( true );
		}

		bool Init_Sound()
		{
			string Msg;
			Log.log( "Starting : Sound Init" );
			Sound = new Sound();

			Sound.LoadSoundFiles();
			/*
			Msg.Clear() << "Loaded " << Sound->GetSampleCount() << " Sound samples."; InitLog->Log( 3, ZLog::INFO, Msg );
			if( Sound->GetSampleCount() < 8 ) { ZString Err; Err << "Missing Sound Sample Files (count : " << Sound->GetSampleCount() << ")"; InitLog->Log( 4, ZLog::FAIL, Err ); return ( false ); }

			Sound->SampleModify_Volume( 4, 0.5 );
			Sound->SampleModify_Volume( 5, 0.03 ); // Vrilleuse d'oreilles (0.1)
			Sound->SampleModify_Volume( 6, 0.3 ); // Bloc Break (0.3)
			Sound->SampleModify_Volume( 7, 0.3 ); // Bloc Place (0.3)

			*/
			Initialized_Sound = true;
			Log.log( "Ended Ok : Sound Init" );
			return ( true );
		}

		bool Start_GameEventSequencer()
		{
			GameEventSequencer = new EventSequencer();
			// GameEventSequencer->AddEvent(30000,10000,1,true, 600000);
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
			//GameWindow_Inventory = new ZGameWindow_Inventory; GameWindow_Inventory->SetGameEnv( this );
			//VoxelTypeBar = new ZGameWindow_VoxelTypeBar; VoxelTypeBar->SetGameEnv( this );
			//GameWindow_Storage = new ZGameWindow_Storage; GameWindow_Storage->SetGameEnv( this );
			//GameWindow_Programmable = new ZGameWindow_Programmable; GameWindow_Programmable->SetGameEnv( this );
			//GameProgressBar = new ZGameWindow_ProgressBar; GameProgressBar->SetGameEnv( this );
			//GameWindow_Advertising = new ZGameWindow_Advertising; GameWindow_Advertising->SetGameEnv( this );
			//GameWindow_UserTextureTransformer = new ZGameWindow_UserTextureTransformer; GameWindow_UserTextureTransformer->SetGameEnv( this );
			//GameWindow_DisplayInfos = new ZGameWindow_DisplayInfos; GameWindow_DisplayInfos->SetGameEnv( this );
			//GameWindow_Sequencer = new ZGameWindow_Sequencer; GameWindow_Sequencer->SetGameEnv( this );

			//GameWindow_Advertising->Show();
			Initialized_GameWindows = true;
			return ( true );
		}
		bool Start_World()
		{
			World = new VoxelWorld( this );
			//Basic_Renderer.SetWorld( World );

			World.SetUniverseNum( UniverseNum );
			World.SetVoxelTypeManager( VoxelTypeManager );
			World.CreateDemoWorld();
			World.SetVoxel( 0, 0, 0, 1 );// World.SetVoxel(0,1,0,1); World.SetVoxel(1,0,0,1); World.SetVoxel(0,2,0,2); World.SetVoxel(0,2,1,2);
										  //World.SetVoxel(0,2,-1,2);World.SetVoxel(10,0,10,1); World.SetVoxel(10,1,10,1); World.SetVoxel(10,2,10,1); World.SetVoxel(5,3,0,1); World.SetVoxel(0,3,5,2);
			World.Load();
			// GameEnv.VoxelTypeManager.DumpInfos();
			World.WorldUpdateFaceCulling();


			Initialized_World = true;
			return ( true );
		}

		bool Start_SectorLoader()
		{
			IWorldGenesis genesis = Compiler.LoadGenesisCode();
			if( genesis != null )
			{
				SectorLoader = new FileSectorLoader( this, genesis );
				SectorLoader.SetVoxelTypeManager( VoxelTypeManager );
				SectorLoader.SetUniverseNum( UniverseNum );
				if( !SectorLoader.Init() ) return ( false );
				World.SetSectorLoader( SectorLoader );
				Initialized_SectorLoader = true;
				return ( true );
			}
			return false;
		}
	}
}
