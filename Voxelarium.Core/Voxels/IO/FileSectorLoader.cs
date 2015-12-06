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
using Voxelarium.Common;
using Voxelarium.Core.Support;

namespace Voxelarium.Core.Voxels.IO
{
	internal class FileSectorLoader : SectorLoader
	{
		private VoxelGameEnvironment GameEnv;

		Thread Thread;
		AutoResetEvent SleepEvent;
		bool ThreadContinue;

		SectorRequestRingList[] RequestList = new SectorRequestRingList[8];
		SectorTagHash RequestTag = new SectorTagHash();
		SectorRingList ReadySectorList;
		SectorRingList EjectedSectorList;
		Stack<VoxelSector> SectorRecycling;

		VoxelTypeManager VoxelTypeManager;

		internal static VoxelSector WorkingEmptySector;
		internal static VoxelSector WorkingFullSector;

		int UniverseNum;

		IWorldGenesis SectorCreator;

		public void SetVoxelTypeManager( VoxelTypeManager VoxelTypeManager ) { this.VoxelTypeManager = VoxelTypeManager; }
		public void SetUniverseNum( int UniverseNum ) { this.UniverseNum = UniverseNum; }
		public bool Is_EjectFileNotFull() { return ( EjectedSectorList.IsNotFull() ); }


		//ZMonoSizeMemoryPool ZSectorTagHash::DefaultMemoryPool;

		void thread_func( object Data )
		{
			FileSectorLoader SectorLoader = (FileSectorLoader)Data;
			while( !VoxelGlobalSettings.Exiting && SectorLoader.ThreadContinue )
			{
				SectorLoader.MakeTasks();
				SleepEvent.WaitOne( 25 );
				//Thread.
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
			SectorRecycling = new Stack<VoxelSector>( 100 );
			VoxelTypeManager = null;
			UniverseNum = 1;
			WorkingEmptySector = new VoxelSector( null, (VoxelWorld)null );
			//GameEnv.Basic_Renderer.GetCuller().InitFaceCullData( WorkingEmptySector );
			WorkingEmptySector.Fill( 0 );
			WorkingFullSector = new VoxelSector( null, (VoxelWorld)null );
			//GameEnv.Basic_Renderer.GetCuller().InitFaceCullData( WorkingFullSector );
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
			ThreadContinue = false;
		}

		public bool Init( ref int start_percent, ref int start_step, ref int start_steps )
		{
			ThreadContinue = true;
			SleepEvent = new AutoResetEvent( false );
			Thread = new System.Threading.Thread( thread_func );
			Thread.Start( this );
			if( !SectorCreator.LoadTemplateImages( VoxelTypeManager, ref  start_percent, ref  start_step, ref  start_steps ) ) return ( false );
			return ( true );
		}

		public bool LoadSector( int x, int y, int z, EventWaitHandle wait_event )
		{
			VoxelSector NewSector;
			bool Redo, TryLoad;

			if( !RequestTag.find( x, y, z ) )
			{
				if( SectorRecycling.Count > 0 )
				{
					//Log.log( "Have something to recycle..." );
					NewSector = SectorRecycling.Pop(); // Try recycling some old used sector.
					NewSector.SetPos( x, y, z );
				}
				else
					NewSector = new VoxelSector( GameEnv, GameEnv.World, x, y, z );//, GameEnv.Basic_Renderer.GetCuller() );

				TryLoad = true;
				do
				{
					Redo = false;

					//NewSector.Pos_x = x; NewSector.Pos_y = y; NewSector.Pos_z = z;
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

				//Log.log( "Push ReadySector..." + x + " " + y + " " + z );
				// Push it to the list for integration in the world on the main thread.
				ReadySectorList.PushToList( NewSector );
				if( wait_event != null )
					wait_event.Set();
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
				EventWaitHandle wait_event;
				if( RequestList[5].PullFromList( out x, out y, out z, out wait_event ) ) { Pri = 4; }
				else if( RequestList[4].PullFromList( out x, out y, out z, out wait_event ) ) { Pri = 4; }
				else if( RequestList[3].PullFromList( out x, out y, out z, out wait_event ) ) { Pri = 3; }
				else if( RequestList[2].PullFromList( out x, out y, out z, out wait_event ) ) { Pri = 2; }
				else if( RequestList[1].PullFromList( out x, out y, out z, out wait_event ) ) { Pri = 1; }
				else if( RequestList[0].PullFromList( out x, out y, out z, out wait_event ) ) { Pri = 0; }
				else break;

				if( LoadSector( x, y, z, wait_event ) /*&& Pri<4*/) break;
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
				SectorRecycling.Push( Sector );
			}

			return;

		}

		public void Request_Sector( int x, int y, int z, int Priority, EventWaitHandle wait_event )
		{
			RequestList[Priority].PushToList( x, y, z, wait_event );
			SleepEvent.Set();
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
			{
				SleepEvent.Set();
				System.Threading.Thread.Sleep( 10 );
			}
		}

	}
}
