using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using Voxelarium.Core.Support;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.Voxels
{
	internal class VoxelWorldProcessor
	{
		public bool ThreadProcessing; // set when thread is dispatched processing
		VoxelWorld World;
		//ZActorPhysicEngine* PhysicEngine;
		bool ThreadContinue;
		Thread thread;
		VoxelGameEnvironment GameEnv;
		VoxelReactor VoxelReactor;
		//EgmyScatter EgmyScatter;
		public HighPerfTimer Timer;
		public HighPerfTimer Timer_Compute;

		//ULong RNG_z = 0;

		ZVector3L Player_Sector;
		ZVector3f Player_Position;
		ZVector3L Player_Voxel;

		float SectorEjectDistance;
		uint debug_DeleteRequests = 0;

		public VoxelWorldProcessor()
		{
			SectorEjectDistance = 1000000.0f;
		}

		~VoxelWorldProcessor()
		{

		}

		internal void SetWorld( VoxelWorld World ) { this.World = World; }
		internal void SetGameEnv( VoxelGameEnvironment GameEnv ) { this.GameEnv = GameEnv; }
		internal void SetSectorEjectDistance( float SectorEjectDistance ) { this.SectorEjectDistance = SectorEjectDistance; }

		internal void SetPlayerPosition( float x, float y, float z )
		{
			{
				Player_Position.x = x;
				Player_Position.y = y;
				Player_Position.z = z;
				Player_Sector.x = (int)( x / ( VoxelGlobalSettings.WorldVoxelBlockSize * 16.0 ) );
				Player_Sector.y = (int)( y / ( VoxelGlobalSettings.WorldVoxelBlockSize * 64.0 ) );
				Player_Sector.z = (int)( z / ( VoxelGlobalSettings.WorldVoxelBlockSize * 16.0 ) );
				Player_Voxel.x = (int)( x / VoxelGlobalSettings.WorldVoxelBlockSize );
				Player_Voxel.y = (int)( y / VoxelGlobalSettings.WorldVoxelBlockSize );
				Player_Voxel.z = (int)( z / VoxelGlobalSettings.WorldVoxelBlockSize );
			}
		}

		internal void Start()
		{
			VoxelReactor = new VoxelReactor();
			VoxelReactor.Init( this.GameEnv );
			ThreadContinue = true;
			thread = new Thread( thread_func );
			//Thread[1] = (SDL_Thread * )SDL_CreateThread(thread_func, "thread_func", this);
			//Thread[2] = (SDL_Thread * )SDL_CreateThread(thread_func, "thread_func", this);
			//Thread[3] = (SDL_Thread * )SDL_CreateThread(thread_func, "thread_func", this);
		}

		internal void End()
		{
			ThreadContinue = false;
			while( ThreadProcessing )
				Thread.Sleep( 10 );
			VoxelReactor = null;
		}

		static void thread_func( object Data )
		{
			VoxelWorldProcessor VoxelProcessor = (VoxelWorldProcessor)Data;
			while( VoxelProcessor.ThreadContinue )
			{
				VoxelProcessor.ThreadProcessing = true;
				VoxelProcessor.MakeTasks();
				VoxelProcessor.ThreadProcessing = false;
			}
		}


		internal void MakeTasks()
		{
			// Block processing
			VoxelSector Sector;
			int cnt;

			cnt = 0;

			Timer.Start();
			Timer_Compute.Start();

			// Sector Tasks like face culling.
			Sector = World.SectorList;
			while( Sector != null )
			{
				if( !Sector.Flag_DeletePending ) MakeSectorTasks( Sector );
				Sector = Sector.GlobalList_Next;
				cnt++;
			}

			if( GameEnv.Enable_MVI )
				VoxelReactor.ProcessSectors( ( (float)Timer.GetResult() ) / 1000.0f );

			Timer_Compute.End();
			// printf("Processed: %lu Sectors\n", cnt);
			if( ThreadContinue )
			{
				int tmp;
				if( ( tmp = ( Timer_Compute.GetResult() / 1000 ) ) < 20 )
					Thread.Sleep( 20 - tmp );
				else
					Thread.Sleep( 0 );
			}
			Timer.End();
			//SDL_Delay(200); // 2
		}


		internal void MakeSectorTasks( VoxelSector Sector )
		{


			// Sector Unloading.
			//
			// Compute distance of the sector from the player position sector.
			// If sector is too far, don't process further and send sector to unloading list.

			float xdist = Sector.Pos_x - Player_Sector.x;
			float ydist = Sector.Pos_y - Player_Sector.y;
			float zdist = Sector.Pos_z - Player_Sector.z;
			float Dist = (float)Math.Sqrt( xdist * xdist + ydist * ydist + zdist * zdist );

			if( Dist > SectorEjectDistance && !Sector.Flag_KeepInMemory )
			{
				if( World.RequestSectorEject( Sector ) )
				{
					Sector.Flag_DeletePending = true;
					//printf("EjectDemand : %lx L1 Start:%lu End:%lu nEntries:%lu\n",Sector,World.SectorEjectList.debug_getstart(),World.SectorEjectList.debug_GetEnd(),World.SectorEjectList.debug_GetnEntries());
					debug_DeleteRequests++;
					// printf("EjectDemand : %lx %lu\n",Sector,debug_DeleteRequests);
				}
			} // 14

			// **************************** Sector face culling ***********************

			//uint64_t CullingResult;

			if( Sector.PartialCulling != 0 )
			{
				//CullingResult = 
				Sector.Culler.CullSector( Sector, false, Sector.PartialCulling );
				//Sector.PartialCulling ^= CullingResult & (DRAWFACE_ABOVE | DRAWFACE_BELOW | DRAWFACE_LEFT | DRAWFACE_RIGHT | DRAWFACE_AHEAD | DRAWFACE_BEHIND);
				//Sector.PartialCulling &= (DRAWFACE_ABOVE | DRAWFACE_BELOW | DRAWFACE_LEFT | DRAWFACE_RIGHT | DRAWFACE_AHEAD | DRAWFACE_BEHIND);
				//if (CullingResult) Sector.Flag_Render_Dirty = true;
				// printf("Cull %ld,%ld,%ld :%lx (%lx)\n", Sector.Pos_x, Sector.Pos_y, Sector.Pos_z, CullingResult, (ULong)Sector.PartialCulling);
			}

			// **************************** Egmy Scattering ****************************
#if asdf
			if( GameEnv.GameEventSequencer.SlotIsActivated( 1 ) )
			{
				EgmyScatter.ScatterEgmys_T1( Sector );
			}
			else { EgmyScatter.ResetWave(); }
#endif
		}
	}
}
