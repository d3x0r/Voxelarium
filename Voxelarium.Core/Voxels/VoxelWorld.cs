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
using System;
using System.Collections.Generic;
using System.Text;
using Voxelarium.Core.Support;
using Voxelarium.Core.UI;
using Voxelarium.Core.Voxels.IO;
using Voxelarium.Core.Voxels.UI;
using System.Threading;

namespace Voxelarium.Core.Voxels
{
	public class VoxelWorld
	{
		public static VoxelSector WorkingFullSector;
		public static VoxelSector WorkingEmptySector;
		public static VoxelSector WorkingScratchSector;

		VoxelGameEnvironment GameEnv;
		internal SectorRingList SectorEjectList;
		public VoxelSector SectorList;

		internal RenderInterface renderer;
		VoxelSector[] SectorTable;
		internal VoxelTypeManager VoxelTypeManager;
		internal SectorLoader SectorLoader;

			// world has voxel size... all blocks in a 'world' are constant
		public int VoxelBlockSizeBits = 0;
		public int VoxelBlockSize = 1 ;

		const int TableSize = SectorHashSize_x * SectorHashSize_y * SectorHashSize_z;
		public const int SectorHashSize_x = 32;
		public const int SectorHashSize_y = 32;
		public const int SectorHashSize_z = 32;
		public btMatrix3x3 orientation;
		public int UniverseNum;

		internal TextureAtlas TextureAtlas;

		static VoxelWorld()
		{
			WorkingFullSector = new VoxelSector( (VoxelWorld)null );
			//GameEnv.Basic_Renderer.GetCuller().InitFaceCullData( WorkingFullSector );
			WorkingFullSector.Fill( 0x0001 );
			WorkingEmptySector = new VoxelSector( (VoxelWorld)null );
			//GameEnv.Basic_Renderer.GetCuller().InitFaceCullData( WorkingEmptySector );
			WorkingEmptySector.Fill( 0 );
			WorkingScratchSector = new VoxelSector( (VoxelWorld)null );
			//GameEnv.Basic_Renderer.GetCuller().InitFaceCullData( WorkingScratchSector );
		}

		public VoxelWorld( VoxelGameEnvironment GameEnv )
		{
			uint i;
			this.GameEnv = GameEnv;
			SectorEjectList = new SectorRingList( 256 * 256 * 32/*65536*/);
			TextureAtlas = new TextureAtlas( 32, 64 );
			
			SectorTable = new VoxelSector[TableSize];

			for( i = 0; i < TableSize; i++ ) SectorTable[i] = null;


			SectorList = null;
			UniverseNum = 1;
			VoxelTypeManager = null;
		}

		~VoxelWorld()
		{
			VoxelSector Sector, NewSector;

			Sector = SectorList;

			while( Sector != null )
			{
				NewSector = Sector.GlobalList_Next;
				if( VoxelGlobalSettings.COMPILEOPTION_ALLOWSAVE )
				{
					if( VoxelGlobalSettings.COMPILEOPTION_SAVEONLYMODIFIED )
					{
						//Log.log( " *** SAVE INCOMPLETE *** " );

						if( Sector.IsMustBeSaved() )
						{
							Sector.Save( UniverseNum );
						}
					}
				}
				Sector.Dispose();
				Sector = NewSector;
			}

			if( SectorTable != null ) { SectorTable = null; }

			if( WorkingFullSector != null ) { WorkingFullSector.Dispose(); WorkingFullSector = null; }
			if( WorkingEmptySector != null ) { WorkingEmptySector.Dispose(); WorkingEmptySector = null; }
			if( WorkingScratchSector != null ) { WorkingScratchSector.Dispose(); WorkingScratchSector = null; }
			SectorList = null;
			UniverseNum = 0;
			if( SectorEjectList != null ) SectorEjectList.Dispose();
			SectorEjectList = null;
		}

		internal VoxelSector FindSector( int x, int y, int z )
		{
			int xs, ys, zs, Offset;
			VoxelSector SectorPointer;

			xs = x % SectorHashSize_x;
			ys = y % SectorHashSize_y;
			zs = z % SectorHashSize_z;

			xs &= 0x1f;
			ys &= 0x1f;
			zs &= 0x1f;

			Offset = xs + ys * SectorHashSize_x + ( zs * SectorHashSize_x * SectorHashSize_y );

			SectorPointer = SectorTable[Offset];
			while( SectorPointer != null )
			{
				if( ( SectorPointer.Pos_x == x ) && ( SectorPointer.Pos_y == y ) && ( SectorPointer.Pos_z == z ) ) return ( SectorPointer );
				SectorPointer = SectorPointer.Next;
			}
			return null;
		}

		VoxelSector FindSector_Secure( int x, int y, int z ) // Create sector if not in memory.
		{
			int xs, ys, zs, Offset;
			VoxelSector SectorPointer;

			xs = x % SectorHashSize_x;
			ys = y % SectorHashSize_y;
			zs = z % SectorHashSize_z;

			xs &= 0x1f;
			ys &= 0x1f;
			zs &= 0x1f;

			Offset = xs + ys * SectorHashSize_x + ( zs * SectorHashSize_x * SectorHashSize_y );
			bool requested = false;

			while( true )
			{
				AutoResetEvent wait_event = new AutoResetEvent( false );
				SectorPointer = SectorTable[Offset];
				while( SectorPointer != null )
				{
					if( ( SectorPointer.Pos_x == x ) && ( SectorPointer.Pos_y == y ) && ( SectorPointer.Pos_z == z ) ) return ( SectorPointer );
					SectorPointer = SectorPointer.Next;
				}
				if( !requested )
				{
					RequestSector( x, y, z, 5, wait_event );
					requested = true;
				}
				// need a while to load a sector...
				wait_event.WaitOne( -1 );
				//System.Threading.Thread.Sleep( 30 );
				ProcessNewLoadedSectors();
            }
		}

		internal void ProcessNewLoadedSectors()
		{
			VoxelSector Sector, AdjSector;
			//if( SectorLoader == null ) return;

			while( ( Sector = SectorLoader.GetRequested() ) != null )
			{
				if( FindSector( Sector.Pos_x, Sector.Pos_y, Sector.Pos_z ) == null )
				{
					AddSector( Sector );
					Sector.Culler = renderer.GetCuller( );
					Sector.Culler.InitFaceCullData( Sector );

					Sector.Culler.CullSector( Sector, true, 0 );

					Sector.Flag_Void_Regular = false;
					Sector.Flag_Void_Transparent = false;

					Sector.Flag_Render_Dirty = true;
					//printf("AddSector: %ld,%ld,%ld\n",Sector.Pos_x, Sector.Pos_y, Sector.Pos_z);

					// Partial face culing for adjacent sectors
					AdjSector = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1]; // find to the left... update its right
					if( AdjSector != null ) {
						AdjSector.Culler.CullSector( AdjSector, false, VoxelSector.FACEDRAW_Operations.RIGHT );
					}
					AdjSector = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1]; // find to the right... update its left
					if( AdjSector != null ) {
						AdjSector.Culler.CullSector( AdjSector, false, VoxelSector.FACEDRAW_Operations.LEFT );
					}
					AdjSector = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND - 1]; // behind 'behind' update its ahead...
					if( AdjSector != null ) {
						AdjSector.Culler.CullSector( AdjSector, false, VoxelSector.FACEDRAW_Operations.AHEAD );
					}
					AdjSector = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD - 1]; // found to the front, update its behind
					if( AdjSector != null ) {
						AdjSector.Culler.CullSector( AdjSector, false, VoxelSector.FACEDRAW_Operations.BEHIND );
					}
					AdjSector = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1]; // found below update its above
					if( AdjSector != null ) {
						AdjSector.Culler.CullSector( AdjSector, false, VoxelSector.FACEDRAW_Operations.ABOVE );
					}
					AdjSector = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1]; // found above, udpate its below
					if( AdjSector != null ) {
						AdjSector.Culler.CullSector( AdjSector, false, VoxelSector.FACEDRAW_Operations.BELOW );
					}
				}
				else { Sector.Dispose(); Log.log( "Loading already used sector***\n" ); }
			}
		}

		void AddSector( VoxelSector Sector )
		{
			int x, y, z, Offset;
			VoxelSector SectorPointer;

			{
				int n;
				for( n = 0; n < 6; n++ )
				{
					VoxelSector near_sec = FindSector( Sector.Pos_x + VoxelSector.NormalBasePosition[n].x
													  , Sector.Pos_y + VoxelSector.NormalBasePosition[n].y
													  , Sector.Pos_z + VoxelSector.NormalBasePosition[n].z );
					if( near_sec != null )
					{
						Sector.near_sectors[n] = near_sec;
						near_sec.near_sectors[n ^ 1] = Sector;
					}
				}
			}

			// Adding to fast access hash

			x = Sector.Pos_x & (SectorHashSize_x - 1);
			y = Sector.Pos_y & (SectorHashSize_y - 1);
			z = Sector.Pos_z & (SectorHashSize_z - 1);

			Offset = x + y * SectorHashSize_x + ( z * SectorHashSize_x * SectorHashSize_y );

			if( SectorTable[Offset] == null )
			{
				SectorTable[Offset] = Sector; Sector.Next = null; Sector.Pred = null;
			}
			else
			{
				SectorPointer = SectorTable[Offset];
				while( SectorPointer.Next != null ) SectorPointer = SectorPointer.Next;
				SectorPointer.Next = Sector;
				Sector.Next = null;
				Sector.Pred = SectorPointer;
			}

			// Adding to sequential access global list

			if( SectorList == null )
			{
				SectorList = Sector;
				Sector.GlobalList_Next = null;
				Sector.GlobalList_Pred = null;
			}
			else
			{
				Sector.GlobalList_Next = SectorList;
				Sector.GlobalList_Pred = null;
				SectorList.GlobalList_Pred = Sector;
				SectorList = Sector;
			}
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public bool GetVoxelRef( out VoxelRef result, int x, int y, int z )
		{
			result.Sector = FindSector( x >> VoxelSector.ZVOXELBLOCSHIFT_X, y >> VoxelSector.ZVOXELBLOCSHIFT_Y, z >> VoxelSector.ZVOXELBLOCSHIFT_Z );
			result.wx = x;
			result.wy = y;
			result.wz = z;
			result.Offset = (uint)( ( result.y = (byte)(y & VoxelSector.ZVOXELBLOCMASK_Y) )
				   + ( ( result.x = (byte)( x & VoxelSector.ZVOXELBLOCMASK_X) ) << VoxelSector.ZVOXELBLOCSHIFT_Y )
				   + ( ( result.z = (byte)( z & VoxelSector.ZVOXELBLOCMASK_Z) ) << ( VoxelSector.ZVOXELBLOCSHIFT_Y + VoxelSector.ZVOXELBLOCSHIFT_X ) ) );

			result.World = this;
			result.VoxelTypeManager = VoxelTypeManager;
			if( result.Sector == null )
			{
				result.Type = 0;
				result.VoxelExtension = null;
				return false;
			}
			result.Type = result.Sector.Data.Data[result.Offset];
			result.VoxelExtension = result.Sector.Data.OtherInfos[result.Offset];

			return true;
		}

		internal void SetUniverseNum( int UniverseNum ) { this.UniverseNum = UniverseNum; }
		internal void SetVoxelTypeManager( VoxelTypeManager Manager, ref int percent, ref int step, ref int steps )
		{
			VoxelTypeManager = Manager;
			Manager.LoadTexturesToAtlas( TextureAtlas, ref percent, ref step, ref steps );
        }

		internal void CreateDemoWorld()
		{
			int x, y, z;
			for( x = -2; x <= 1; x++ )
			{
				for( y = -2; y <= 1; y++ )
				{
					for( z = -2; z <= 1; z++ )
					{
						FindSector_Secure( x, y, z );
					}
				}
			}
		}

		internal void SetVoxel( int x, int y, int z, int VoxelValue )
		{
			VoxelSector Sector;
			Sector = FindSector( x >> VoxelSector.ZVOXELBLOCSHIFT_X, y >> VoxelSector.ZVOXELBLOCSHIFT_Y, z >> VoxelSector.ZVOXELBLOCSHIFT_Z );
			if( Sector == null ) return;
			Sector.SetCube( x, y, z, VoxelValue );
		}

		internal void RequestSector( int x, int y, int z, int Priority, EventWaitHandle wait_event = null )
		{
			if( SectorLoader == null ) return;

			SectorLoader.Request_Sector( x, y, z, Priority, wait_event );
		}

		internal void SetSectorLoader( SectorLoader SectorLoader )
		{
			this.SectorLoader = SectorLoader;
		}
		internal bool RequestSectorEject( VoxelSector SectorToEject )
		{ return ( SectorEjectList.PushToList( SectorToEject ) ); }

		internal void ProcessOldEjectedSectors()
		{
			VoxelSector Sector;

			if( SectorLoader == null ) return;

			while( SectorLoader.Is_EjectFileNotFull() )
			{
				if( ( Sector = SectorEjectList.PullFromList() ) == null ) break;

				// printf("EjectPass : %lx %lu\n",Sector,++debug_ejectpass);
				RemoveSector( Sector );
				SectorLoader.Eject_Sector( Sector );
				Sector.Dispose();
			}
		}

		void RemoveSector( VoxelSector Sector )
		{
			int x, y, z, Offset;
			VoxelSector SectorPointer;

			// Finding sector in hash

			x = ( Sector.Pos_x % SectorHashSize_x ) & 0xff;
			y = ( Sector.Pos_y % SectorHashSize_y ) & 0xFF;
			z = ( Sector.Pos_z % SectorHashSize_z ) & 0xff;
			Offset = x + y * SectorHashSize_x + ( z * SectorHashSize_x * SectorHashSize_y );
			SectorPointer = SectorTable[Offset];
			while( SectorPointer != Sector )
			{
				if( SectorPointer == null ) return;
				SectorPointer = SectorPointer.Next;
			}

			// Remove from hash
			if( SectorPointer == SectorTable[Offset] ) SectorTable[Offset] = Sector.Next;
			else Sector.Pred.Next = Sector.Next;
			if( Sector.Next != null ) Sector.Next.Pred = Sector.Pred;

			// Remove from global list

			if( Sector.GlobalList_Pred != null ) Sector.GlobalList_Pred.GlobalList_Next = Sector.GlobalList_Next;
			else { SectorList = Sector.GlobalList_Next; }
			if( Sector.GlobalList_Next != null ) Sector.GlobalList_Next.GlobalList_Pred = Sector.GlobalList_Pred;

			// Zeroing fields

			Sector.Next = null; Sector.Pred = null; Sector.GlobalList_Next = null; Sector.GlobalList_Pred = null;

		}
	}
}
