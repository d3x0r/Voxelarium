using Bullet.LinearMath;
using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Voxels
{
	public class VoxelWorld
	{

		public VoxelSector WorkingFullSector;
		public VoxelSector WorkingEmptySector;
		public VoxelSector WorkingScratchSector;
		VoxelGameEnvironment GameEnv;
	    //public SectorRingList SectorEjectList;
		public VoxelSector SectorList;

		VoxelSector[] SectorTable;
		public VoxelTypeManager VoxelTypeManager;
		//public SectorLoader SectorLoader;

		uint TableSize;
		public int Size_x;
		public int Size_y;
		public int Size_z;
		public btMatrix3x3 orientation;
		public uint UniverseNum;


		VoxelSector FindSector( int x, int y, int z )
		{
			int xs, ys, zs, Offset;
			VoxelSector SectorPointer;

			xs = x % Size_x;
			ys = y % Size_y;
			zs = z % Size_z;

			xs &= 0xff;
			ys &= 0xf;
			zs &= 0xff;

			Offset = xs + ys * Size_x + ( zs * Size_x * Size_y );

			SectorPointer = SectorTable[Offset];
			while( SectorPointer != null )
			{
				if( ( SectorPointer.Pos_x == x ) && ( SectorPointer.Pos_y == y ) && ( SectorPointer.Pos_z == z ) ) return ( SectorPointer );
				SectorPointer = SectorPointer.Next;
			}
			return null;
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
			result.Offset = ( result.y = y & VoxelSector.ZVOXELBLOCMASK_Y )
				   + ( ( result.x = x & VoxelSector.ZVOXELBLOCMASK_X ) << VoxelSector.ZVOXELBLOCSHIFT_Y )
				   + ( ( result.z = z & VoxelSector.ZVOXELBLOCMASK_Z ) << ( VoxelSector.ZVOXELBLOCSHIFT_Y + VoxelSector.ZVOXELBLOCSHIFT_X ) );

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
	}
}
