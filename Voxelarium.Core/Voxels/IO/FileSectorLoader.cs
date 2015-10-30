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
		ushort[][] SectorDataTable = new ushort[27][];
		VoxelType[] Vt = new VoxelType[6 + 12];

		void LimitedUpdateFaceCulling( VoxelSector Sector )
		{
			//VoxelSector[] SectorTable = new VoxelSector[27];
			VoxelType[] VoxelTypeTable;

			ushort Temp;
			uint i;

			for( i = 0; i < 27; i++ )
			{
				SectorDataTable[i] = WorkingFullSector.Data.Data;
			}
			SectorDataTable[0] = Sector.Data.Data;
			//SectorTable[1] = WorkingFullSector;
			//SectorTable[2] = WorkingFullSector;
			//SectorTable[3] = WorkingFullSector;
			//SectorTable[6] = WorkingFullSector;
			//SectorTable[9] = WorkingFullSector;
			//SectorTable[18]= WorkingFullSector;

			int xc, yc, zc;
			int xp, yp, zp;
			int xpp, ypp, zpp;
			int sec_xc, sec_zc;
			int sec_xp, sec_zp;
			int sec_xpp, sec_ypp, sec_zpp;
			int off_xc, off_zc;
			int off_xp, off_zp;
			int off_xpp, off_ypp, off_zpp;
			VoxelSector.FACEDRAW_Operations info;
			bool TransparentVoxel;

			VoxelTypeTable = VoxelTypeManager.VoxelTable;

			for( xc = 0; xc < VoxelSector.ZVOXELBLOCSIZE_X; xc++ )
			{
				xp = xc + 1; xpp = xc + 2;
				sec_xc = VoxelSector.STableX[xc]; sec_xp = VoxelSector.STableX[xp]; sec_xpp = VoxelSector.STableX[xpp];
				off_xc = VoxelSector.OfTableX[xc]; off_xp = VoxelSector.OfTableX[xp]; off_xpp = VoxelSector.OfTableX[xpp];
				for( zc = 0; zc < VoxelSector.ZVOXELBLOCSIZE_Z; zc++ )
				{
					zp = zc + 1; zpp = zc + 2;
					sec_zc = VoxelSector.STableZ[zc]; sec_zp = VoxelSector.STableZ[zp]; sec_zpp = VoxelSector.STableZ[zpp];
					off_zc = VoxelSector.OfTableZ[zc]; off_zp = VoxelSector.OfTableZ[zp]; off_zpp = VoxelSector.OfTableZ[zpp];
					byte sec_y0 = VoxelSector.STableY[0];
					byte sec_y1 = VoxelSector.STableY[1];
					ushort off_y0 = VoxelSector.OfTableY[0];
					ushort off_y1 = VoxelSector.OfTableY[1];
					// Prefetching the bloc matrix (only 2 rows)
					// left/ahead (below)
					//BlocMatrix[1][0] = SectorDataTable[( sec_xc + sec_y0 + sec_zc )][xc + off_y0 + off_zc];
					// center/ahead (below)
					BlocMatrix[1][1] = SectorDataTable[( sec_xp + sec_y0 + sec_zc )][xp + off_y0 + off_zc];
					// right/ahead (below)
					//BlocMatrix[1][2] = SectorDataTable[( sec_xpp + sec_y0 + sec_zc )][xpp + off_y0 + off_zc];
					// left/center (below)
					BlocMatrix[1][3] = SectorDataTable[( sec_xc + sec_y0 + sec_zp )][xc + off_y0 + off_zp];
					// cneter/center (below)
					BlocMatrix[1][4] = SectorDataTable[( sec_xp + sec_y0 + sec_zp )][xp + off_y0 + off_zp];
					// right/center (below)
					BlocMatrix[1][5] = SectorDataTable[( sec_xpp + sec_y0 + sec_zp )][xpp + off_y0 + off_zp];
					// left/behind (below)
					//BlocMatrix[1][6] = SectorDataTable[( sec_xc + sec_y0 + sec_zpp )][xc + off_y0 + off_zpp];
					// center/behind (below)
					BlocMatrix[1][7] = SectorDataTable[( sec_xp + sec_y0 + sec_zpp )][xp + off_y0 + off_zpp];
					// right/behind (below)
					//BlocMatrix[1][8] = SectorDataTable[( sec_xpp + sec_y0 + sec_zpp )][xpp + off_y0 + off_zpp];

					//BlocMatrix[2][0] = SectorDataTable[( sec_xc + sec_y1 + sec_zc )][xc + off_y1 + off_zc];
					BlocMatrix[2][1] = SectorDataTable[( sec_xp + sec_y1 + sec_zc )][xp + off_y1 + off_zc];
					//BlocMatrix[2][2] = SectorDataTable[( sec_xpp + sec_y1 + sec_zc )][xpp + off_y1 + off_zc];
					BlocMatrix[2][3] = SectorDataTable[( sec_xc + sec_y1 + sec_zp )][xc + off_y1 + off_zp];
					BlocMatrix[2][4] = SectorDataTable[( sec_xp + sec_y1 + sec_zp )][xp + off_y1 + off_zp];
					BlocMatrix[2][5] = SectorDataTable[( sec_xpp + sec_y1 + sec_zp )][xpp + off_y1 + off_zp];
					//BlocMatrix[2][6] = SectorDataTable[( sec_xc + sec_y1 + sec_zpp )][xc + off_y1 + off_zpp];
					BlocMatrix[2][7] = SectorDataTable[( sec_xp + sec_y1 + sec_zpp )][xp + off_y1 + off_zpp];
					//BlocMatrix[2][8] = SectorDataTable[( sec_xpp + sec_y1 + sec_zpp )][xpp + off_y1 + off_zpp];

					for( yc = 0; yc < VoxelSector.ZVOXELBLOCSIZE_Y; yc++ )
					{
						yp = yc + 1; ypp = yc + 2;
						sec_ypp = VoxelSector.STableY[ypp];
						off_ypp = VoxelSector.OfTableY[ypp];
						// Scrolling bloc matrix by exchanging references.
						tmpp = BlocMatrix[0];
						BlocMatrix[0] = BlocMatrix[1];
						BlocMatrix[1] = BlocMatrix[2];
						BlocMatrix[2] = tmpp;

						// Fetching a new bloc of data slice;

						//BlocMatrix[2][0] = SectorDataTable[( sec_xc + sec_ypp + sec_zc )][xc + off_ypp + off_zc];
						BlocMatrix[2][1] = SectorDataTable[( sec_xp + sec_ypp + sec_zc )][xp + off_ypp + off_zc];
						//BlocMatrix[2][2] = SectorDataTable[( sec_xpp + sec_ypp + sec_zc )][xpp + off_ypp + off_zc];
						BlocMatrix[2][3] = SectorDataTable[( sec_xc + sec_ypp + sec_zp )][xc + off_ypp + off_zp];
						BlocMatrix[2][4] = SectorDataTable[( sec_xp + sec_ypp + sec_zp )][xp + off_ypp + off_zp];
						BlocMatrix[2][5] = SectorDataTable[( sec_xpp + sec_ypp + sec_zp )][xpp + off_ypp + off_zp];
						//BlocMatrix[2][6] = SectorDataTable[( sec_xc + sec_ypp + sec_zpp )][xc + off_ypp + off_zpp];
						BlocMatrix[2][7] = SectorDataTable[( sec_xp + sec_ypp + sec_zpp )][xp + off_ypp + off_zpp];
						//BlocMatrix[2][8] = SectorDataTable[( sec_xpp + sec_ypp + sec_zpp )][xpp + off_ypp + off_zpp];

						info = 0;
						if( BlocMatrix[1][4] > 0 )
						{
							Vt[0] = VoxelTypeTable[BlocMatrix[1][1]];
							Vt[1] = VoxelTypeTable[BlocMatrix[1][7]];
							Vt[2] = VoxelTypeTable[BlocMatrix[1][3]];
							Vt[3] = VoxelTypeTable[BlocMatrix[1][5]];
							Vt[4] = VoxelTypeTable[BlocMatrix[0][4]];
							Vt[5] = VoxelTypeTable[BlocMatrix[2][4]];
#if asdfasdf
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
#endif

							Temp = BlocMatrix[1][4];
							TransparentVoxel = VoxelTypeTable[Temp].properties.Draw_TransparentRendering;

							info |= ( Vt[0].properties.Draw_FullVoxelOpacity || ( TransparentVoxel && Vt[0].properties.Draw_TransparentRendering ) ) ? 0 : VoxelSector.FACEDRAW_Operations.AHEAD;
							info |= ( Vt[1].properties.Draw_FullVoxelOpacity || ( TransparentVoxel && Vt[1].properties.Draw_TransparentRendering ) ) ? 0 : VoxelSector.FACEDRAW_Operations.BEHIND;
							info |= ( Vt[2].properties.Draw_FullVoxelOpacity || ( TransparentVoxel && Vt[2].properties.Draw_TransparentRendering ) ) ? 0 : VoxelSector.FACEDRAW_Operations.LEFT;
							info |= ( Vt[3].properties.Draw_FullVoxelOpacity || ( TransparentVoxel && Vt[3].properties.Draw_TransparentRendering ) ) ? 0 : VoxelSector.FACEDRAW_Operations.RIGHT;
							info |= ( Vt[4].properties.Draw_FullVoxelOpacity || ( TransparentVoxel && Vt[4].properties.Draw_TransparentRendering ) ) ? 0 : VoxelSector.FACEDRAW_Operations.BELOW;
							info |= ( Vt[5].properties.Draw_FullVoxelOpacity || ( TransparentVoxel && Vt[5].properties.Draw_TransparentRendering ) ) ? 0 : VoxelSector.FACEDRAW_Operations.ABOVE;

#if asdfasdf
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
#endif
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
						if( Sector.Culler != null )
							Sector.Culler.setFaceCulling( Sector, off_xp + VoxelSector.OfTableY[yp] + off_zp, info );

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
