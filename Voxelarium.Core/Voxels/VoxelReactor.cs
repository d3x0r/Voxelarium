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
using Voxelarium.LinearMath;
using OpenTK;
using System;
using Voxelarium.Core.Game;
using Voxelarium.Common;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.Voxels
{
	public class VoxelReactor
	{
		public static SaltyRandomGenerator Random;

		Vector3 PlayerPosition;
		VoxelGameEnvironment GameEnv;
		VoxelWorld World;
		VoxelTypeManager VoxelTypeManager;
		//ZEgmyTargetManager EgmyWaveManager;
		uint CycleNum;

#if asdfasdfasdf
		public:
		public struct ZBlocPos { public byte x; byte y; byte z; };
		static ZBlocPos bfta[26];  // bloc flow table (air)
		static ZBlocPos bfts[18];  // bloc flow table (smoothing)
		static ZBlocPos bp6[6];   // Bloc positions with 6 slots around main cube.
		static ZBlocPos bft[8];   // Bloc fall test positions with 4 slots around and 4 slots under;
		static ZBlocPos bft6[10]; // Bloc fall test positions with 6 slots around main cube and 4 slots under (Special case for acid).
		static UByte BlocOpposite[6];
		static ZBlocPos xbp6[6];  // Bloc positions with 6 slots around main cube. ( New standardised robot order.).
		static ZBlocPos xbp6_opposing[6];  // Bloc positions with 6 slots around main cube. ( New standardised robot order.).
		static RelativeVoxelOrds x6_opposing_escape[6,5];  // Bloc positions with 6 slots around main cube. ( New standardised robot order.).
	static ZBlocPosN xbp6_opposing_escape[6,5]	;
    static ZBlocPosN xbp6_nc[6];// same as xbp6 with -1,+1 range
#endif

		// Time remaining on FireMine action
		uint FireMineTime;

		public
			void Init( VoxelGameEnvironment GameEnv )
		{
			this.GameEnv = GameEnv;
			this.World = GameEnv.World;
			this.VoxelTypeManager = GameEnv.VoxelTypeManager;
			PlayerPosition.X = PlayerPosition.Y = PlayerPosition.Z = 0;
		}

		internal VoxelReactor()
		{
			if( Random == null )
			{
				// one-time inits for static members.
				Random = new SaltyRandomGenerator();
			}
		}


		void UpdatePlayerPosition( ref Vector3 PlayerPosition )
		{
			this.PlayerPosition = PlayerPosition;
		}

#if asdfasdf
		void LightTransmitter_FindEndPoints( ZVector3L* Location, ZVector3L* NewCommingDirection );
		void LightTransmitter_FollowTransmitter( ZVector3L* Location, ZVector3L* FollowingDirection );
		bool VoxelFluid_ComputeVolumePressure( ZVector3L* Location, UShort VoxelType, bool EvenCycle );
		void VoxelFluid_ComputeVolumePressure_Recurse( ZVector3L* Location, ZonePressure* Pr );
		void VoxelFluid_SetVolumePressure_Recurse( ZVector3L* Location, ZonePressure* Pr );
#endif

		internal static bool StepOne = true;

		internal void ProcessSectors( float LastLoopTime )
		{
			int x, y, z;

			uint MainOffset;
			ushort VoxelType;
			VoxelSector Sector;
			bool LowActivityTrigger;
			Actor SelectedActor;

			btVector3 PlayerLocation;
			//Log.log( "Begin Reaction Processing" );
			int Sectors_processed = 0;
			int Voxels_Processed = 0;
			// FireMine
			if( FireMineTime > 0 ) FireMineTime--;

			// Get the player location (in multithreading friendly way)

			//do
			//{
			SelectedActor = GameEnv.GetActiveActor();
			if( SelectedActor != null )
				PlayerLocation = SelectedActor.ViewDirection.m_origin;
			//} while( PlayerLocation != GameEnv.PhysicEngine.GetSelectedActor().ViewDirection.origin() );

			// Cycle Counter is incremented at each MVI's cycle. This is used in cycle dependent operations.

			CycleNum++;

			// Egmy Wave Manager
			//EgmyWaveManager.SwapList();
			//

			Sector = World.SectorList;

			if( !StepOne )
				return;
			while( ( Sector ) != null )
			{
				Sectors_processed++;

				LowActivityTrigger = Sector.Flag_IsActiveLowRefresh
					&& ( ( ( CycleNum ) & Sector.LowRefresh_Mask ) == 0 );
				Sector.ModifTracker.SetActualCycleNum( CycleNum );
				if( Sector.Flag_IsActiveVoxels | LowActivityTrigger )
				{
					for( x = 0; x <= 6; x++ )
					{
						if( Sector.near_sectors[x] != null )
							Sector.near_sectors[x].ModifTracker.SetActualCycleNum( CycleNum );
					}

					VoxelExtension[] Extension = Sector.Data.OtherInfos;
					ushort[] VoxelP = Sector.Data.Data;
					VoxelType[] VoxelTable = VoxelTypeManager.VoxelTable;
					int zofs, xofs;
					bool IsActiveVoxels = false;
					MainOffset = 0;
					int RSx = Sector.Pos_x << VoxelSector.ZVOXELBLOCSHIFT_X;
					int RSy = Sector.Pos_y << VoxelSector.ZVOXELBLOCSHIFT_Y;
					int RSz = Sector.Pos_z << VoxelSector.ZVOXELBLOCSHIFT_Z;
					VoxelRef vref;
					FastBit_Array_32k sleep = Sector.Data.SleepState;
                    vref.World = World;
					vref.VoxelTypeManager = VoxelTypeManager;
					vref.Sector = Sector;
					for( z = 0, zofs = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++, zofs += (int)(Sector.Size_x*Sector.Size_y) )
						for( xofs = 0, x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++, xofs += (int)Sector.Size_y )
							for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
							{
								MainOffset = (uint)(zofs + xofs + y);

								if( sleep.Get( (ushort)MainOffset ) )
									continue;

								vref.VoxelExtension = Extension[MainOffset];
								VoxelType = VoxelP[MainOffset];

								if( ( vref.Type = VoxelTable[VoxelType] ).properties.Is_Active )
								{

									if( !Sector.ModifTracker.Get( MainOffset ) ) // If voxel is already processed, don't process it once more in the same cycle.
									{
										Voxels_Processed++;
										vref.wx = RSx + ( vref.x = (byte)x );
										vref.wy = RSy + ( vref.y = (byte)y );
										vref.wz = RSz + ( vref.z = (byte)z );
										vref.Offset = MainOffset;
										//St[i].ModifTracker.Set(SecondaryOffset[i]);
										try
										{
											if( vref.Type.React( ref vref, LastLoopTime ) )
												IsActiveVoxels = true;
											else
												vref.Sector.Data.SleepState.Set( (int)vref.Offset, true );

										}
										catch( Exception e )
										{
											Log.log( "Voxel Reaction Exception : {0}", e.Message );
										}
									}
								}
								else
									sleep.Set( (ushort)MainOffset, true );
							}
					Sector.Flag_IsActiveVoxels = IsActiveVoxels;
				}
				Sector = Sector.GlobalList_Next;
			}
			//StepOne = false;
			//Log.log( "Finish Reaction Processing {0} {1} ", Sectors_processed, Voxels_Processed );
		}
	}
}