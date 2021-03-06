﻿/*
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
using Voxelarium.Core.Game;
using Voxelarium.Core.Support;
using Voxelarium.Core.UI;
using Voxelarium.Core.Voxels.Types;
using Voxelarium.LinearMath;

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
		//ZVector3f Player_Position;
		//ZVector3L Player_Voxel;

		int SectorEjectDistance;
		uint debug_DeleteRequests = 0;

		public VoxelWorldProcessor()
		{
			SectorEjectDistance = (int)( Math.Max( Settings_Hardware.RenderingDistance_Horizontal, Settings_Hardware.RenderingDistance_Vertical ) + 2);
		}

		~VoxelWorldProcessor()
		{

		}

		internal void SetWorld( VoxelWorld World ) { this.World = World; }
		internal void SetGameEnv( VoxelGameEnvironment GameEnv ) { this.GameEnv = GameEnv; }
		internal void SetSectorEjectDistance( int SectorEjectDistance ) { this.SectorEjectDistance = SectorEjectDistance; }

		internal void SetPlayerPosition( VoxelWorld world, ref btVector3 v )
		{
			{
				//Player_Position.x = v.x;
				//Player_Position.y = v.y;
				//Player_Position.z = v.z;
				Player_Sector.x = (int)( v.x / ( world.VoxelBlockSize * VoxelSector.ZVOXELBLOCSIZE_X ) );
				Player_Sector.y = (int)( v.y / ( world.VoxelBlockSize * VoxelSector.ZVOXELBLOCSIZE_Y ) );
				Player_Sector.z = (int)( v.z / ( world.VoxelBlockSize * VoxelSector.ZVOXELBLOCSIZE_Z ) );
				//Player_Voxel.x = (int)( v.x / world.VoxelBlockSize );
				//Player_Voxel.y = (int)( v.y / world.VoxelBlockSize );
				//Player_Voxel.z = (int)( v.z / world.VoxelBlockSize );
			}
		}

		internal void Start()
		{
			Display.AtExit += Display_AtExit;
			VoxelReactor = new VoxelReactor();
			VoxelReactor.Init( this.GameEnv );
			ThreadContinue = true;
			thread = new Thread( thread_func );
			thread.Start( this );
			//Thread[1] = (SDL_Thread * )SDL_CreateThread(thread_func, "thread_func", this);
			//Thread[2] = (SDL_Thread * )SDL_CreateThread(thread_func, "thread_func", this);
			//Thread[3] = (SDL_Thread * )SDL_CreateThread(thread_func, "thread_func", this);
		}

		private void Display_AtExit()
		{
			ThreadContinue = false;
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

			int xdist = Sector.Pos_x - Player_Sector.x;
			int ydist = Sector.Pos_y - Player_Sector.y;
			int zdist = Sector.Pos_z - Player_Sector.z;
			int Dist = ( xdist * xdist + ydist * ydist + zdist * zdist );
			if( Dist > SectorEjectDistance )
				if( Sector.Flag_KeepInMemory )
					Log.log( "Prevented eject; is processing in reactor" );
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
			// handled during normal culling...
			//uint64_t CullingResult;
			/*
						if( !Sector.Flag_DeletePending && Sector.PartialCulling != 0 )
						{
							//CullingResult = 
							Sector.Culler.CullSector( Sector, false, Sector.PartialCulling );
							//Sector.PartialCulling ^= CullingResult & (DRAWFACE_ABOVE | DRAWFACE_BELOW | DRAWFACE_LEFT | DRAWFACE_RIGHT | DRAWFACE_AHEAD | DRAWFACE_BEHIND);
							//Sector.PartialCulling &= (DRAWFACE_ABOVE | DRAWFACE_BELOW | DRAWFACE_LEFT | DRAWFACE_RIGHT | DRAWFACE_AHEAD | DRAWFACE_BEHIND);
							//if (CullingResult) Sector.Flag_Render_Dirty = true;
							// printf("Cull %ld,%ld,%ld :%lx (%lx)\n", Sector.Pos_x, Sector.Pos_y, Sector.Pos_z, CullingResult, (ULong)Sector.PartialCulling);
						}
						*/
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
