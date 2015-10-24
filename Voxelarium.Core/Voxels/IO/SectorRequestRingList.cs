using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Voxels.IO
{
	internal class SectorRequestRingList
	{
		internal struct SectorRequest { internal int x, y, z; };
		internal SectorRequest[] RequestList;
		internal uint ListSize;
		internal uint WrapMask;
		internal volatile uint Start, End, nEntries;

		internal SectorRequestRingList()
		{
			ListSize = 128;
			WrapMask = ListSize - 1;
			Start = End = nEntries = 0;
			RequestList = new SectorRequest[ListSize];

		}

		 ~SectorRequestRingList()
		{
			RequestList = null;
			ListSize = WrapMask = Start = End = nEntries = 0;
		}


#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal void PushToList( int x, int y, int z )
		{
			// if (nEntries > (ListSize - 4)) return;
			if( ( ( End + 1 ) & WrapMask ) == Start ) return;
			RequestList[End].x = x; RequestList[End].y = y; RequestList[End].z = z;
			End = ( End + 1 ) & WrapMask;
			// nEntries ++;
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal bool PullFromList( out int x, out int y, out int z )
		{
			if( Start == End )
			{
				x = y = z = 0;
				return ( false );
			}
			x = RequestList[Start].x; y = RequestList[Start].y; z = RequestList[Start].z;
			Start = ( Start + 1 ) & WrapMask;
			// nEntries --;
			return ( true );
		}

		internal bool IsDataReady() { return ( Start != End ); }
}
}
