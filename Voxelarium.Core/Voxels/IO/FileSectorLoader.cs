using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using Voxelarium.Core.Support;

namespace Voxelarium.Core.Voxels.IO
{
	internal class FileSectorLoader : SectorLoader
	{
		private VoxelGameEnvironment GameEnv;

		Thread Thread;
		bool ThreadContinue;

		SectorRequestRingList[] RequestList = new SectorRequestRingList[8];
		SectorTagHash RequestTag = new SectorTagHash();
		SectorRingList ReadySectorList;
		SectorRingList EjectedSectorList;
		SectorRingList SectorRecycling;

		VoxelTypeManager VoxelTypeManager;

		internal static VoxelSector WorkingEmptySector;
		internal static VoxelSector WorkingFullSector;

		int UniverseNum;

		IWorldGenesis SectorCreator;

		internal void SetVoxelTypeManager( VoxelTypeManager VoxelTypeManager ) { this.VoxelTypeManager = VoxelTypeManager; }
		internal void SetUniverseNum( int UniverseNum ) { this.UniverseNum = UniverseNum; }
		public bool Is_EjectFileNotFull() { return ( EjectedSectorList.IsNotFull() ); }


		//ZMonoSizeMemoryPool ZSectorTagHash::DefaultMemoryPool;


		void thread_func( object Data )
		{
			FileSectorLoader SectorLoader = (FileSectorLoader)Data;
			while( !VoxelGlobalSettings.Exiting && SectorLoader.ThreadContinue )
			{
				SectorLoader.MakeTasks();
				System.Threading.Thread.Sleep( 10 );
				//Thread.
				//WakeableSleep( 20 );
				//SDL_Delay(10);
			}
		}


		internal FileSectorLoader( VoxelGameEnvironment GameEnv, IWorldGenesis Genesis )
		{
			for( int i = 0; i < 8; i++ )
				RequestList[i] = new SectorRequestRingList();
			this.GameEnv = GameEnv;
			SectorCreator = Genesis;
			ReadySectorList = new SectorRingList( 1024 * 1024 );
			EjectedSectorList = new SectorRingList( 1024 * 1024 );
			SectorRecycling = new SectorRingList( 1024 * 1024 );
			VoxelTypeManager = null;
			UniverseNum = 1;
			WorkingEmptySector = new VoxelSector( GameEnv.World );
			GameEnv.Basic_Renderer.GetCuller().InitFaceCullData( WorkingEmptySector );
			WorkingEmptySector.Fill( 0 );
			WorkingFullSector = new VoxelSector( GameEnv.World );
			GameEnv.Basic_Renderer.GetCuller().InitFaceCullData( WorkingFullSector );
			WorkingFullSector.Fill( 1 );
			Thread = null;
			ThreadContinue = false;
		}

		~FileSectorLoader()
		{
			WorkingEmptySector.Dispose();
			WorkingEmptySector = null;
			WorkingFullSector.Dispose();

			WorkingFullSector = null;

			VoxelSector Sector;

			if( ReadySectorList != null )
			{
				while( ( Sector = ReadySectorList.PullFromList() ) != null )
				{
					if( VoxelGlobalSettings.COMPILEOPTION_ALLOWSAVE )
						Sector.Save( UniverseNum );
					Sector.Dispose();
				}
				//delete ReadySectorList;
				ReadySectorList = null;
			}
			if( EjectedSectorList != null )
			{
				EjectedSectorList.FreeRemainingContent();
				//delete EjectedSectorList;
				EjectedSectorList = null;
			}
			UniverseNum = 0;
		}

		public bool Init()
		{
			ThreadContinue = true;
			Thread = new System.Threading.Thread( thread_func );
			Thread.Start( this );
			if( !SectorCreator.LoadTemplateImages( VoxelTypeManager ) ) return ( false );
			return ( true );
		}

		public bool LoadSector( int x, int y, int z )
		{
			VoxelSector NewSector;
			bool Redo, TryLoad;

			if( !RequestTag.find( x, y, z ) )
			{
				NewSector = SectorRecycling.PullFromList(); // Try recycling some old used sector.
				if( NewSector == null )
				{
					NewSector = new VoxelSector( GameEnv.World );//, GameEnv.Basic_Renderer.GetCuller() );
				}

				TryLoad = true;
				do
				{
					Redo = false;

					NewSector.Pos_x = x; NewSector.Pos_y = y; NewSector.Pos_z = z;
					NewSector.SetVoxelTypeManager( VoxelTypeManager );

					if( TryLoad && NewSector.Load( UniverseNum ) )
					{
						// Does this sector must be regenerated ?
						if( 0 == ( NewSector.Flag_IsModified & VoxelSector.ModifiedFieldFlags.SAVEMASK ) )
						{
							if( VoxelGlobalSettings. COMPILEOPTION_ALLOWSAVE ) NewSector.DeleteSave( UniverseNum );
							NewSector.ReinitSector();
							Redo = true;
							TryLoad = false;
							continue;
						}
					}
					else
					{
						SectorCreator.GenerateSector( NewSector );
					}
				} while( Redo );

				// Set the options for further edge faceculling.

				NewSector.Flag_NeedFullCulling = false;
				NewSector.PartialCulling = VoxelSector.FACEDRAW_Operations.ALL;

				// Add it in the tag

				RequestTag.Add( x, y, z );

				// Partial face culling on the sector. Edges must be completed later when other sectors will be availlable.

				LimitedUpdateFaceCulling( NewSector );
				//NoDrawFaceCulling(NewSector);

				// Push it to the list for integration in the world on the main thread.

				ReadySectorList.PushToList( NewSector );
				return ( true );
			}
			return ( false );
		}

		int debug_deletecount = 0;

		internal void MakeTasks()
		{
			int x, y, z;
			byte Pri;

			// Sector Loading

			while( true )
			{
				if( RequestList[5].PullFromList( out x, out y, out z ) ) { Pri = 4; }
				else if( RequestList[4].PullFromList( out x, out y, out z ) ) { Pri = 4; }
				else if( RequestList[3].PullFromList( out x, out y, out z ) ) { Pri = 3; }
				else if( RequestList[2].PullFromList( out x, out y, out z ) ) { Pri = 2; }
				else if( RequestList[1].PullFromList( out x, out y, out z ) ) { Pri = 1; }
				else if( RequestList[0].PullFromList( out x, out y, out z ) ) { Pri = 0; }
				else break;

				if( LoadSector( x, y, z ) /*&& Pri<4*/) break;
			}

			// Sector Unloading

			VoxelSector Sector;

			while( ( Sector = EjectedSectorList.PullFromList() ) != null )
			{
				RequestTag.Remove( Sector.Pos_x, Sector.Pos_y, Sector.Pos_z );
				Log.log( "Deleted : {0}, {1} L2 Start:{2} End:{3} nEntries:{4}\n", Sector, ++debug_deletecount, EjectedSectorList.debug_getstart(), EjectedSectorList.debug_GetEnd(), EjectedSectorList.debug_GetnEntries() );
				if( VoxelGlobalSettings.COMPILEOPTION_ALLOWSAVE )
				{
					if( !VoxelGlobalSettings.COMPILEOPTION_SAVEONLYMODIFIED || Sector.IsMustBeSaved() )
					{
						Sector.Save( UniverseNum );
					}
				}
				//delete Sector;

				Sector.ReinitSector();
				SectorRecycling.PushToList( Sector );
			}

			return;

		}

		public void Request_Sector( int x, int y, int z, int Priority )
		{
			RequestList[Priority].PushToList( x, y, z );
			//printf("Request :%ld,%ld,%ld\n",x,y,z);
		}

		public void Eject_Sector( VoxelSector Sector )
		{
			if( !EjectedSectorList.PushToList( Sector ) ) Log.log( "Ejection Stall" );
		}

		public VoxelSector GetRequested()
		{
			return ( ReadySectorList.PullFromList() );
		}

		public void Cleanup()
		{
			ThreadContinue = false;
			while( Thread.ThreadState == ThreadState.Running )
				System.Threading.Thread.Sleep( 10 );
		}


		ushort[][] BlocMatrix = new ushort[3][] { new ushort[9], new ushort[9], new ushort[9] };
		ushort[] tmpp;
		VoxelSector[] SectorTable = new VoxelSector[27];
		VoxelType[] Vt = new VoxelType[6 + 12];

		void LimitedUpdateFaceCulling( VoxelSector Sector )
		{
			//VoxelSector[] SectorTable = new VoxelSector[27];
			VoxelType[] VoxelTypeTable;

			ushort Temp;
			uint i;

			for( i = 0; i < 27; i++ ) SectorTable[i] = WorkingFullSector;
			SectorTable[0] = Sector;
			//SectorTable[1] = WorkingFullSector;
			//SectorTable[2] = WorkingFullSector;
			//SectorTable[3] = WorkingFullSector;
			//SectorTable[6] = WorkingFullSector;
			//SectorTable[9] = WorkingFullSector;
			//SectorTable[18]= WorkingFullSector;

			int xc, yc, zc;
			int xp, yp, zp;
			int xpp, ypp, zpp;
			VoxelSector.FACEDRAW_Operations info;
			bool TransparentVoxel;

			VoxelTypeTable = VoxelTypeManager.VoxelTable;

			for( xc = 0; xc < VoxelSector.ZVOXELBLOCSIZE_X; xc++ )
			{
				xp = xc + 1; xpp = xc + 2;
				for( zc = 0; zc < VoxelSector.ZVOXELBLOCSIZE_Z; zc++ )
				{
					zp = zc + 1; zpp = zc + 2;

					// Prefetching the bloc matrix (only 2 rows)
					// left/ahead (below)
					BlocMatrix[1][0] = SectorTable[( VoxelSector.STableX[xc] + VoxelSector.STableY[0] + VoxelSector.STableZ[zc] )].Data.Data[VoxelSector.OfTableX[xc] + VoxelSector.OfTableY[0] + VoxelSector.OfTableZ[zc]];
					// center/ahead (below)
					BlocMatrix[1][1] = SectorTable[( VoxelSector.STableX[xp] + VoxelSector.STableY[0] + VoxelSector.STableZ[zc] )].Data.Data[VoxelSector.OfTableX[xp] + VoxelSector.OfTableY[0] + VoxelSector.OfTableZ[zc]];
					// right/ahead (below)
					BlocMatrix[1][2] = SectorTable[( VoxelSector.STableX[xpp] + VoxelSector.STableY[0] + VoxelSector.STableZ[zc] )].Data.Data[VoxelSector.OfTableX[xpp] + VoxelSector.OfTableY[0] + VoxelSector.OfTableZ[zc]];
					// left/center (below)
					BlocMatrix[1][3] = SectorTable[( VoxelSector.STableX[xc] + VoxelSector.STableY[0] + VoxelSector.STableZ[zp] )].Data.Data[VoxelSector.OfTableX[xc] + VoxelSector.OfTableY[0] + VoxelSector.OfTableZ[zp]];
					// cneter/center (below)
					BlocMatrix[1][4] = SectorTable[( VoxelSector.STableX[xp] + VoxelSector.STableY[0] + VoxelSector.STableZ[zp] )].Data.Data[VoxelSector.OfTableX[xp] + VoxelSector.OfTableY[0] + VoxelSector.OfTableZ[zp]];
					// right/center (below)
					BlocMatrix[1][5] = SectorTable[( VoxelSector.STableX[xpp] + VoxelSector.STableY[0] + VoxelSector.STableZ[zp] )].Data.Data[VoxelSector.OfTableX[xpp] + VoxelSector.OfTableY[0] + VoxelSector.OfTableZ[zp]];
					// left/behind (below)
					BlocMatrix[1][6] = SectorTable[( VoxelSector.STableX[xc] + VoxelSector.STableY[0] + VoxelSector.STableZ[zpp] )].Data.Data[VoxelSector.OfTableX[xc] + VoxelSector.OfTableY[0] + VoxelSector.OfTableZ[zpp]];
					// center/behind (below)
					BlocMatrix[1][7] = SectorTable[( VoxelSector.STableX[xp] + VoxelSector.STableY[0] + VoxelSector.STableZ[zpp] )].Data.Data[VoxelSector.OfTableX[xp] + VoxelSector.OfTableY[0] + VoxelSector.OfTableZ[zpp]];
					// right/behind (below)
					BlocMatrix[1][8] = SectorTable[( VoxelSector.STableX[xpp] + VoxelSector.STableY[0] + VoxelSector.STableZ[zpp] )].Data.Data[VoxelSector.OfTableX[xpp] + VoxelSector.OfTableY[0] + VoxelSector.OfTableZ[zpp]];

					BlocMatrix[2][0] = SectorTable[( VoxelSector.STableX[xc] + VoxelSector.STableY[1] + VoxelSector.STableZ[zc] )].Data.Data[VoxelSector.OfTableX[xc] + VoxelSector.OfTableY[1] + VoxelSector.OfTableZ[zc]];
					BlocMatrix[2][1] = SectorTable[( VoxelSector.STableX[xp] + VoxelSector.STableY[1] + VoxelSector.STableZ[zc] )].Data.Data[VoxelSector.OfTableX[xp] + VoxelSector.OfTableY[1] + VoxelSector.OfTableZ[zc]];
					BlocMatrix[2][2] = SectorTable[( VoxelSector.STableX[xpp] + VoxelSector.STableY[1] + VoxelSector.STableZ[zc] )].Data.Data[VoxelSector.OfTableX[xpp] + VoxelSector.OfTableY[1] + VoxelSector.OfTableZ[zc]];
					BlocMatrix[2][3] = SectorTable[( VoxelSector.STableX[xc] + VoxelSector.STableY[1] + VoxelSector.STableZ[zp] )].Data.Data[VoxelSector.OfTableX[xc] + VoxelSector.OfTableY[1] + VoxelSector.OfTableZ[zp]];
					BlocMatrix[2][4] = SectorTable[( VoxelSector.STableX[xp] + VoxelSector.STableY[1] + VoxelSector.STableZ[zp] )].Data.Data[VoxelSector.OfTableX[xp] + VoxelSector.OfTableY[1] + VoxelSector.OfTableZ[zp]];
					BlocMatrix[2][5] = SectorTable[( VoxelSector.STableX[xpp] + VoxelSector.STableY[1] + VoxelSector.STableZ[zp] )].Data.Data[VoxelSector.OfTableX[xpp] + VoxelSector.OfTableY[1] + VoxelSector.OfTableZ[zp]];
					BlocMatrix[2][6] = SectorTable[( VoxelSector.STableX[xc] + VoxelSector.STableY[1] + VoxelSector.STableZ[zpp] )].Data.Data[VoxelSector.OfTableX[xc] + VoxelSector.OfTableY[1] + VoxelSector.OfTableZ[zpp]];
					BlocMatrix[2][7] = SectorTable[( VoxelSector.STableX[xp] + VoxelSector.STableY[1] + VoxelSector.STableZ[zpp] )].Data.Data[VoxelSector.OfTableX[xp] + VoxelSector.OfTableY[1] + VoxelSector.OfTableZ[zpp]];
					BlocMatrix[2][8] = SectorTable[( VoxelSector.STableX[xpp] + VoxelSector.STableY[1] + VoxelSector.STableZ[zpp] )].Data.Data[VoxelSector.OfTableX[xpp] + VoxelSector.OfTableY[1] + VoxelSector.OfTableZ[zpp]];

					for( yc = 0; yc < VoxelSector.ZVOXELBLOCSIZE_Y; yc++ )
					{
						yp = yc + 1; ypp = yc + 2;

						// Scrolling bloc matrix by exchanging references.
						tmpp = BlocMatrix[0];
						BlocMatrix[0] = BlocMatrix[1];
						BlocMatrix[1] = BlocMatrix[2];
						BlocMatrix[2] = tmpp;

						// Fetching a new bloc of data slice;

						BlocMatrix[2][0] = SectorTable[( VoxelSector.STableX[xc] + VoxelSector.STableY[ypp] + VoxelSector.STableZ[zc] )].Data.Data[VoxelSector.OfTableX[xc] + VoxelSector.OfTableY[ypp] + VoxelSector.OfTableZ[zc]];
						BlocMatrix[2][1] = SectorTable[( VoxelSector.STableX[xp] + VoxelSector.STableY[ypp] + VoxelSector.STableZ[zc] )].Data.Data[VoxelSector.OfTableX[xp] + VoxelSector.OfTableY[ypp] + VoxelSector.OfTableZ[zc]];
						BlocMatrix[2][2] = SectorTable[( VoxelSector.STableX[xpp] + VoxelSector.STableY[ypp] + VoxelSector.STableZ[zc] )].Data.Data[VoxelSector.OfTableX[xpp] + VoxelSector.OfTableY[ypp] + VoxelSector.OfTableZ[zc]];
						BlocMatrix[2][3] = SectorTable[( VoxelSector.STableX[xc] + VoxelSector.STableY[ypp] + VoxelSector.STableZ[zp] )].Data.Data[VoxelSector.OfTableX[xc] + VoxelSector.OfTableY[ypp] + VoxelSector.OfTableZ[zp]];
						BlocMatrix[2][4] = SectorTable[( VoxelSector.STableX[xp] + VoxelSector.STableY[ypp] + VoxelSector.STableZ[zp] )].Data.Data[VoxelSector.OfTableX[xp] + VoxelSector.OfTableY[ypp] + VoxelSector.OfTableZ[zp]];
						BlocMatrix[2][5] = SectorTable[( VoxelSector.STableX[xpp] + VoxelSector.STableY[ypp] + VoxelSector.STableZ[zp] )].Data.Data[VoxelSector.OfTableX[xpp] + VoxelSector.OfTableY[ypp] + VoxelSector.OfTableZ[zp]];
						BlocMatrix[2][6] = SectorTable[( VoxelSector.STableX[xc] + VoxelSector.STableY[ypp] + VoxelSector.STableZ[zpp] )].Data.Data[VoxelSector.OfTableX[xc] + VoxelSector.OfTableY[ypp] + VoxelSector.OfTableZ[zpp]];
						BlocMatrix[2][7] = SectorTable[( VoxelSector.STableX[xp] + VoxelSector.STableY[ypp] + VoxelSector.STableZ[zpp] )].Data.Data[VoxelSector.OfTableX[xp] + VoxelSector.OfTableY[ypp] + VoxelSector.OfTableZ[zpp]];
						BlocMatrix[2][8] = SectorTable[( VoxelSector.STableX[xpp] + VoxelSector.STableY[ypp] + VoxelSector.STableZ[zpp] )].Data.Data[VoxelSector.OfTableX[xpp] + VoxelSector.OfTableY[ypp] + VoxelSector.OfTableZ[zpp]];

						info = 0;
						if( BlocMatrix[1][4] > 0 )
						{
							Vt[0] = VoxelTypeTable[BlocMatrix[1][1]];
							Vt[1] = VoxelTypeTable[BlocMatrix[1][7]];
							Vt[2] = VoxelTypeTable[BlocMatrix[1][3]];
							Vt[3] = VoxelTypeTable[BlocMatrix[1][5]];
							Vt[4] = VoxelTypeTable[BlocMatrix[0][4]];
							Vt[5] = VoxelTypeTable[BlocMatrix[2][4]];

							Vt[6] = VoxelTypeTable[BlocMatrix[0][1]];
							Vt[7] = VoxelTypeTable[BlocMatrix[0][7]];
							Vt[8] = VoxelTypeTable[BlocMatrix[2][1]];
							Vt[9] = VoxelTypeTable[BlocMatrix[2][7]];
							Vt[10] = VoxelTypeTable[BlocMatrix[0][3]];
							Vt[11] = VoxelTypeTable[BlocMatrix[0][5]];
							Vt[12] = VoxelTypeTable[BlocMatrix[2][3]];
							Vt[13] = VoxelTypeTable[BlocMatrix[2][5]];
							Vt[14] = VoxelTypeTable[BlocMatrix[1][0]];
							Vt[15] = VoxelTypeTable[BlocMatrix[1][2]];
							Vt[16] = VoxelTypeTable[BlocMatrix[1][6]];
							Vt[17] = VoxelTypeTable[BlocMatrix[1][8]];

							Temp = BlocMatrix[1][4];
							TransparentVoxel = VoxelTypeTable[Temp].properties.Draw_TransparentRendering;

							info |= ( Vt[0].properties.Draw_FullVoxelOpacity || ( TransparentVoxel && Vt[0].properties.Draw_TransparentRendering ) ) ? 0 : VoxelSector.FACEDRAW_Operations.AHEAD;
							info |= ( Vt[1].properties.Draw_FullVoxelOpacity || ( TransparentVoxel && Vt[1].properties.Draw_TransparentRendering ) ) ? 0 : VoxelSector.FACEDRAW_Operations.BEHIND;
							info |= ( Vt[2].properties.Draw_FullVoxelOpacity || ( TransparentVoxel && Vt[2].properties.Draw_TransparentRendering ) ) ? 0 : VoxelSector.FACEDRAW_Operations.LEFT;
							info |= ( Vt[3].properties.Draw_FullVoxelOpacity || ( TransparentVoxel && Vt[3].properties.Draw_TransparentRendering ) ) ? 0 : VoxelSector.FACEDRAW_Operations.RIGHT;
							info |= ( Vt[4].properties.Draw_FullVoxelOpacity || ( TransparentVoxel && Vt[4].properties.Draw_TransparentRendering ) ) ? 0 : VoxelSector.FACEDRAW_Operations.BELOW;
							info |= ( Vt[5].properties.Draw_FullVoxelOpacity || ( TransparentVoxel && Vt[5].properties.Draw_TransparentRendering ) ) ? 0 : VoxelSector.FACEDRAW_Operations.ABOVE;

							if( Temp != 0 )
							{
								//if( info & VoxelSector.FACEDRAW_Operations.AHEAD )
								info |= ( Vt[6].properties.Draw_FullVoxelOpacity ) ? VoxelSector.FACEDRAW_Operations.BELOW_HAS_AHEAD : 0;
								info |= ( Vt[7].properties.Draw_FullVoxelOpacity ) ? VoxelSector.FACEDRAW_Operations.BELOW_HAS_BEHIND : 0;
								info |= ( Vt[8].properties.Draw_FullVoxelOpacity ) ? VoxelSector.FACEDRAW_Operations.ABOVE_HAS_AHEAD : 0;
								info |= ( Vt[9].properties.Draw_FullVoxelOpacity ) ? VoxelSector.FACEDRAW_Operations.ABOVE_HAS_BEHIND : 0;
								info |= ( Vt[10].properties.Draw_FullVoxelOpacity ) ? VoxelSector.FACEDRAW_Operations.BELOW_HAS_LEFT : 0;
								info |= ( Vt[11].properties.Draw_FullVoxelOpacity ) ? VoxelSector.FACEDRAW_Operations.BELOW_HAS_RIGHT : 0;
								info |= ( Vt[12].properties.Draw_FullVoxelOpacity ) ? VoxelSector.FACEDRAW_Operations.ABOVE_HAS_LEFT : 0;
								info |= ( Vt[13].properties.Draw_FullVoxelOpacity ) ? VoxelSector.FACEDRAW_Operations.ABOVE_HAS_RIGHT : 0;

								info |= ( Vt[14].properties.Draw_FullVoxelOpacity ) ? VoxelSector.FACEDRAW_Operations.LEFT_HAS_AHEAD : 0;
								info |= ( Vt[15].properties.Draw_FullVoxelOpacity ) ? VoxelSector.FACEDRAW_Operations.RIGHT_HAS_AHEAD : 0;
								info |= ( Vt[16].properties.Draw_FullVoxelOpacity ) ? VoxelSector.FACEDRAW_Operations.LEFT_HAS_BEHIND : 0;
								info |= ( Vt[17].properties.Draw_FullVoxelOpacity ) ? VoxelSector.FACEDRAW_Operations.RIGHT_HAS_BEHIND : 0;
							}
						}

						// if ( (y==-1) && (yc==63) ) info = 255;
						/*
						if (BlocMatrix[1][4]>0)
						{
						  if (VoxelTypeTable[BlocMatrix[1][4]].properties.Draw_TransparentRendering) SectorTable[0].Flag_Void_Transparent = false;
						  else                                                             SectorTable[0].Flag_Void_Regular = false;
						}
						*/
						// Write face culling info to face culling table
						if( SectorTable[0].Culler != null )
							SectorTable[0].Culler.setFaceCulling( SectorTable[0], VoxelSector.OfTableX[xp] + VoxelSector.OfTableY[yp] + VoxelSector.OfTableZ[zp], info );

					}
				}
			}

		}


		internal void NoDrawFaceCulling( VoxelSector Sector )
		{
			int i;

			for( i = 0; i < ( VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_Z ); i++ )
			{
				Sector.Culler.setFaceCulling( Sector, i, VoxelSector.FACEDRAW_Operations.NONE );
			}
		}

	}
}
