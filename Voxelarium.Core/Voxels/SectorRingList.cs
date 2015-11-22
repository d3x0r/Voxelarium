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
