
using OpenTK;
using System.IO;
using Voxelarium.Core.UI;
using Voxelarium.Core.Voxels;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.Game.Screens
{
	class ScreenSlotSelect : Screen
	{
		Frame[] Slot = new Frame[16];
		FontFrame[] SlotTitle = new FontFrame[16];
		FontFrame[] SlotStatus = new FontFrame[16];
		string[] SlotString = new string[16];
		string[] StatusString = new string[16];
		bool[] SlotUsed = new bool[16];
		string[] FileName = new string[16];
		//FILE* fh;
		string Directory;
		Frame LogoBlackvoxel;
		FontFrame Frame_MainTitle;

		public ScreenSlotSelect( VoxelGameEnvironment.Pages page_id ) : base( page_id )
		{
			LogoBlackvoxel = new Frame();
			Frame_MainTitle = new FontFrame();
			for( int i = 0; i < 16; i++ )
			{
				Slot[i] = new Frame();
				SlotTitle[i] = new FontFrame();
				SlotStatus[i] = new FontFrame();
			}
		}

		internal override ScreenMain.ScreenChoices ProcessScreen( VoxelGameEnvironment GameEnv )
		{
			Vector2 Size;
			int i;
			ScreenMain.ScreenChoices ChoosedSlot = ScreenChoices.NONE;

			// Effacer l'Ã©cran
			if( GameEnv.page_up != page_id )
			{
				GameEnv.page_up = page_id;
				// RÃ©gler la transparence

				// Enlever toutes les boites affichÃ©es

				GameEnv.GuiManager.RemoveAllFrames();

				// DÃ©finition et rÃ©glage de la boite du fond


				// DÃ©finition du titre

				Size.Y = 0.2f;
				Size.X = Size.Y * 10.0f;
				LogoBlackvoxel.SetPosition( 0, 2-0.2f );
				LogoBlackvoxel.SetSize( Size.X, Size.Y );
				LogoBlackvoxel.SetTexture( 1 );
				GameEnv.GuiManager.AddFrame( LogoBlackvoxel );

				// DÃ©finition et rÃ©glage du sous-titre

				Frame_MainTitle.SetDisplayText( "GAME UNIVERSE SELECTION" );
				Frame_MainTitle.Font = GameEnv.default_font;// .SetStyle( GameEnv.TileSetStyles.GetStyle( 1 ) );
				Frame_MainTitle.FontSize = ( 2.0f / 20 );
				Frame_MainTitle.GetTextDisplaySize( out Size );
				Frame_MainTitle.SetPosition( 1 - Size.X/2, 2-0.4f );
				//Frame_MainTitle.SetPosition(2.0f / 2.0f - Size.X/2.0f, 2.0f / 2.0f - Size.Y / 2.0f);
				Frame_MainTitle.SetSize( Size.X, Size.Y );
				GameEnv.GuiManager.AddFrame( Frame_MainTitle );
				//Background.AddFrame(Frame_MainTitle);

				// DÃ©finition des variables et objets utilisÃ©s pour la suite


				// RÃ©cupÃ©ration du chemin du rÃ©pertoire de stockage des univers

				Directory = VStreamFile.Get_Directory_UserData();
				Directory += "/" + VoxelGlobalSettings.COMPILEOPTION_SAVEFOLDERNAME;
				Directory += "/Universes";

				// Boucle de rÃ©glages des boites de choix d'univers

				for( i = 0; i < 16; i++ )
				{
					// Tester si le slot est utilisÃ© / Test if GameSlot is used

					FileName[i] = Directory;
					FileName[i] = Directory + "/" + (i+1) + "/PlayerInfo.dat";
					SlotUsed[i] = File.Exists( FileName[i] );

					// RÃ©gler les paramÃ¨tres de la boite du slot univers

					Vector2 SlotPos, SlotSize;
					SlotPos.X = ( (float)( i % 4 ) ) * ( ( 2 ) / 4.25f ) + (float)2.0f * 0.045f;
					SlotPos.Y = 0.6f - ( i / 4 * ( 2.0f / 6.5f ) ) + (float)2.0f * 0.3f;
					//SlotPos.Y = (i / 4 * (2.0f / 4.3f)) + ( i/ 8 * (2.0f * 0.05f) ) + (float)2.0f * 0.025f;
					SlotSize.X = 2.0f / 5.0f;
					SlotSize.Y = 2.0f / 10.0f;
					Slot[i].SetTexture( 11 );

					if( SlotUsed[i] ) Slot[i].SetColor( 1.0f, 1.0f, 1.0f );
					else Slot[i].SetColor( 1.0f, 1.0f, 1.0f );

					Slot[i].SetPosition( SlotPos.X, SlotPos.Y );
					Slot[i].SetSize( SlotSize.X, SlotSize.Y );
					GameEnv.GuiManager.AddFrame( Slot[i] );
					//Background.AddFrame(Slot[i]);

					// RÃ©gler les paramÃ¨tres de la boite du texte slot univers

					SlotString[i] = "UNIVERSE " + (i+1);
					SlotTitle[i].SetDisplayText( SlotString[i] );
					SlotTitle[i].Font = GameEnv.default_font;//.SetStyle( GameEnv.TileSetStyles.GetStyle( 3 ) );
					SlotTitle[i].FontSize = ( 2.0f / 30 );
					SlotTitle[i].SetTexture( 3 );
					SlotTitle[i].GetTextDisplaySize( out Size );
					SlotTitle[i].SetPosition( SlotSize.X * 0.07f, SlotSize.Y * 0.2f );
					// SlotTitle[i].SetPosition(SlotSize.X / 2 - Size.X/2, SlotSize.Y * 0.2f);
					SlotTitle[i].SetSize( Size.X + 1.0f, Size.Y );
					SlotTitle[i].SetColor( 1.0f, 1.0f, 1.0f );
					Slot[i].AddFrame( SlotTitle[i] );

					// RÃ©gler les paramÃ¨tres de la boite du texte Used/Free

					if( SlotUsed[i] ) StatusString[i] = "USED";
					else StatusString[i] = "FREE";
					SlotStatus[i].SetDisplayText( StatusString[i] );
					SlotStatus[i].Font = GameEnv.default_font;//.SetStyle( GameEnv.TileSetStyles.GetStyle( 3 ) );
					SlotStatus[i].FontSize = ( 2.0f / 30 );
					SlotStatus[i].SetTexture( 3 );
					SlotStatus[i].GetTextDisplaySize( out Size );
					SlotStatus[i].SetPosition( SlotSize.X * 0.65f, SlotSize.Y * 0.65f );
					// SlotStatus[i].SetPosition(SlotSize.X / 2 - Size.X/2, SlotSize.Y * 0.45f);
					SlotStatus[i].SetSize( Size.X + 1.0f, Size.Y );
					if( SlotUsed[i] ) SlotStatus[i].SetColor( 1.0f, 0.0f, 0.0f );
					else SlotStatus[i].SetColor( 0.0f, 1.0f, 0.0f );
					// else            SlotStatus[i].SetColor(0.5f,0.5f,0.5f);
					Slot[i].AddFrame( SlotStatus[i] );
				}
			}
			// Boucle d'affichage et de gestion des Ã©vÃ¨nements

			// for (Loop = true; Loop; )
			{
				// Effacer l'Ã©cran

				// glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

				// GÃ©rer les Ã©vÃ¨nements entrants (clics, dÃ©placemets souris, frappe clavier)

				//Loop = GameEnv.EventManager.ProcessEvents();

				// DÃ©tecter les actions de l'utilisateur sur chaque boite slot univers et prendre les actions appropriÃ©es

				for( i = 0; i < 16; i++ )
				{
					if( Slot[i].Is_MouseIn() ) Slot[i].SetTexture( 12 );
					if( Slot[i].Is_MouseOut() ) Slot[i].SetTexture( 11 );
					if( Slot[i].Is_MouseClick() ) { ChoosedSlot = ScreenChoices.SlotChoice1+i; }
				}

				// Demander au gestionnaire de boite d'effectuer le rendu graphique

				//GameEnv.GuiManager.Render(  );

				// On montre Ã  l'utilisateur ce qui a Ã©tÃ© rendu

				//SDL_GL_SwapBuffers();
				//SDL_GL_SwapWindow(GameEnv.screen);

				// On met le programme en pause pendant 10 milliÃ¨mes de seconde

				//SDL_Delay(10);
			}

			// Enlever toutes les boites de l'affichage

			//GameEnv.GuiManager.RemoveAllFrames();

			// Retourner Ã  l'appelant et lui donner le numÃ©ro de l'univers choisis

			return ( ChoosedSlot );
		}

	}
}
