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

namespace Voxelarium.Core.Voxels.IO
{
	internal class SectorRequestRingList
	{
		internal struct SectorRequest { internal int x, y, z; internal EventWaitHandle wait_event; };
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
		internal void PushToList( int x, int y, int z, EventWaitHandle wait_event )
		{
			// if (nEntries > (ListSize - 4)) return;
			if( ( ( End + 1 ) & WrapMask ) == Start ) return;
			RequestList[End].x = x; RequestList[End].y = y; RequestList[End].z = z; RequestList[End].wait_event = wait_event;
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

		internal bool PullFromList( out int x, out int y, out int z, out EventWaitHandle wait_event )
		{
			if( Start == End )
			{
				x = y = z = 0;
				wait_event = null;
				return ( false );
			}
			x = RequestList[Start].x; y = RequestList[Start].y; z = RequestList[Start].z;
			wait_event = RequestList[Start].wait_event;
			Start = ( Start + 1 ) & WrapMask;
			// nEntries --;
			return ( true );
		}

		internal bool IsDataReady() { return ( Start != End ); }
}
}
