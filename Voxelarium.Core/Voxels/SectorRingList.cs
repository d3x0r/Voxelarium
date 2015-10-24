using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Voxels
{
	class SectorRingList: IDisposable
	{
		VoxelSector[] SectorList;
		volatile int ListSize;
		volatile int WrapMask;
		volatile int Start;
		volatile int End;
		volatile int nEntries;

		public SectorRingList( int SizeInPowerOfTwo )
		{
			ListSize = SizeInPowerOfTwo;
			WrapMask = ListSize - 1;
			Start = End = nEntries = 0;
			SectorList = new VoxelSector[ListSize];
		}

		public void Dispose()
		{
			SectorList = null;
			ListSize = WrapMask = Start = End = nEntries = 0;
		}
		~SectorRingList()
		{
			Dispose();
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal bool PushToList( VoxelSector Sector )
		{
			if( nEntries > ( ListSize - 4 ) ) return ( false );
			SectorList[End] = Sector;
			End = ( End + 1 ) & WrapMask;
			nEntries++;
			// printf("Push");
			return ( true );
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal bool IsNotFull()
		{
			if( nEntries > ( ListSize - 4 ) ) return ( false );
			return ( true );
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal VoxelSector PullFromList()
		{
			VoxelSector Sector;

			if( Start == End ) return ( null );

			Sector = SectorList[Start]; SectorList[Start] = null;

			Start = ( Start + 1 ) & WrapMask;
			if( nEntries != 0 ) nEntries--;

			// printf("Pull");

			return ( Sector );
		}

		internal void FreeRemainingContent()
		{
			VoxelSector Sector;
			while( ( Sector = PullFromList() ) != null )
			{
				Sector.Dispose();
			}
		}

		internal int debug_getstart() { return ( Start ); }
		internal int debug_GetEnd() { return ( End ); }
		internal int debug_GetnEntries() { return ( nEntries ); }
	}
}
