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
using System.Collections.Generic;
using System.Text;
using Voxelarium.Core.Game;
using Voxelarium.Core.Voxels.Types;
using Voxelarium.Common;

namespace Voxelarium.Core.Voxels
{
	public class VoxelReactor
	{

		//public static ZLightSpeedRandom Random;

		public static SaltyRandomGenerator Random;

		public static VoxelSector DummySector;

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

		// Fast computing offsets;
		static public byte[] Of_x = new byte[VoxelSector.ZVOXELBLOCSIZE_X + 2];
		static public byte[] Of_y = new byte[VoxelSector.ZVOXELBLOCSIZE_Y + 2];
		static public byte[] Of_z = new byte[VoxelSector.ZVOXELBLOCSIZE_Z + 2];
		static public uint[] If_x = new uint[VoxelSector.ZVOXELBLOCSIZE_X + 2];
		static public uint[] If_y = new uint[VoxelSector.ZVOXELBLOCSIZE_Y + 2];
		static public uint[] If_z = new uint[VoxelSector.ZVOXELBLOCSIZE_Z + 2];

		// DirCodes

		public static byte[] DirCodeTable = new byte[16];

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
			int i;

			if( Random == null )
			{
				// one-time inits for static members.
				Random = new SaltyRandomGenerator();
				//Random.Init( 0 );
				// Dummy Sector

				DummySector = new VoxelSector( null, (VoxelWorld)null );
				DummySector.Fill( 0xFFFF );

				// Multiplexing Sector Tables for fast access to voxels
				Of_x[0] = 0; Of_x[VoxelSector.ZVOXELBLOCSIZE_X + 1] = 2; for( i = 1; i <= VoxelSector.ZVOXELBLOCSIZE_X; i++ ) Of_x[i] = 1;
				Of_y[0] = 0; Of_y[VoxelSector.ZVOXELBLOCSIZE_Y + 1] = 8; for( i = 1; i <= VoxelSector.ZVOXELBLOCSIZE_Y; i++ ) Of_y[i] = 4;
				Of_z[0] = 0; Of_z[VoxelSector.ZVOXELBLOCSIZE_Z + 1] = 32; for( i = 1; i <= VoxelSector.ZVOXELBLOCSIZE_Z; i++ ) Of_z[i] = 16;

				// Multiplexing Voxel Tables for fast access to voxels

				If_x[0] = ( VoxelSector.ZVOXELBLOCSIZE_X - 1 ) * VoxelSector.ZVOXELBLOCSIZE_Y;
				If_x[VoxelSector.ZVOXELBLOCSIZE_X + 1] = 0;
				for( i = 0; i < VoxelSector.ZVOXELBLOCSIZE_X; i++ ) If_x[i + 1] = (uint)i * VoxelSector.ZVOXELBLOCSIZE_Y;
				If_y[0] = ( VoxelSector.ZVOXELBLOCSIZE_Y - 1 );
				If_y[VoxelSector.ZVOXELBLOCSIZE_Y + 1] = 0;
				for( i = 0; i < VoxelSector.ZVOXELBLOCSIZE_Y; i++ ) If_y[i + 1] = (uint)i;
				If_z[0] = ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X;
				If_z[VoxelSector.ZVOXELBLOCSIZE_Z + 1] = 0;
				for( i = 0; i < VoxelSector.ZVOXELBLOCSIZE_Z; i++ ) If_z[i + 1] = (uint)i * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X;

			}

#if asfasdf
			// Reaction table
			ReactionTable = new VoxelReaction[65536];
			for( i = 0; i < 65536; i++ ) ReactionTable[i] = 0;
			// Green acid reaction
			ReactionTable[86] = new ZVoxelReaction( 89, 0 );
			// ReactionTable[86].SetReaction(1,10,10);
#endif
			//ReactionTable[86].Set(1,10);
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

		internal void ProcessSectors( float LastLoopTime )
		{
			int x, y, z;

			uint MainOffset;
			ushort VoxelType;
			VoxelSector Sector;
			bool LowActivityTrigger;
			Actor SelectedActor;

			btVector3 PlayerLocation;
			Log.log( "Begin Reaction Processing" );
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
					int zofs, xofs;
					bool IsActiveVoxels = false;
					MainOffset = 0;
					int RSx = Sector.Pos_x << VoxelSector.ZVOXELBLOCSHIFT_X;
					int RSy = Sector.Pos_y << VoxelSector.ZVOXELBLOCSHIFT_Y;
					int RSz = Sector.Pos_z << VoxelSector.ZVOXELBLOCSHIFT_Z;
					VoxelRef vref;
					vref.World = World;
					vref.VoxelTypeManager = VoxelTypeManager;
					vref.Sector = Sector;
					for( z = 0, zofs = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++, zofs += (int)(Sector.Size_x*Sector.Size_y) )
						for( xofs = 0, x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++, xofs += (int)Sector.Size_y )
							for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
							{
								MainOffset = (uint)(zofs + xofs + y);
                                VoxelType = VoxelP[MainOffset];
								if( VoxelTypeManager[VoxelType].properties.Is_Active )
								{
									if( !Sector.ModifTracker.Get( MainOffset ) ) // If voxel is already processed, don't process it once more in the same cycle.
									{
										Voxels_Processed++;
										vref.wx = RSx + ( vref.x = (byte)x );
										vref.wy = RSy + ( vref.y = (byte)y );
										vref.wz = RSz + ( vref.z = (byte)z );
										vref.Offset = If_x[x + 1] + If_y[y + 1] + If_z[z + 1];
										vref.Type = VoxelTypeManager.VoxelTable[VoxelType];

										IsActiveVoxels = true;
										vref.VoxelExtension = Extension[MainOffset];
										//St[i].ModifTracker.Set(SecondaryOffset[i]);
										try
										{
											IsActiveVoxels = vref.Type.React( ref vref, LastLoopTime );
										}
										catch( Exception e )
										{
											Log.log( "Voxel Reaction Exception : {0}", e.Message );
										}
									}
								}
							}
				}
				Sector = Sector.GlobalList_Next;
			}
			Log.log( "Finish Reaction Processing {0} {1} ", Sectors_processed, Voxels_Processed );
		}
	}
}