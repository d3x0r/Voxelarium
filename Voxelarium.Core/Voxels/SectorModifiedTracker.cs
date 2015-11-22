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
using System.Runtime.CompilerServices;
using System.Text;

namespace Voxelarium.Core.Voxels
{
	public struct SectorModifiedTracker
	{
		uint[] Storage;
		uint ActualCycleNum;
		uint LastUpdateCycleNum;
		uint StorageSize_Num_uints;
		uint BitSize;

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public void Init( uint SizeInBits )
		{
			BitSize = SizeInBits;
			StorageSize_Num_uints = ( BitSize + 1 ) >> 5;
			Storage = new uint[StorageSize_Num_uints];
			//if( sizeof( uint ) != 4 ) throw; // uint Needs to be 32 bits
			ActualCycleNum = 0xFFFFFFFF;
			LastUpdateCycleNum = 0;
		}


		public SectorModifiedTracker( uint SizeInBits )
		{
			BitSize = SizeInBits;
			StorageSize_Num_uints = ( BitSize + 1 ) >> 5;
			Storage = new uint[StorageSize_Num_uints];
			//if( sizeof( uint ) != 4 ) throw; // uint Needs to be 32 bits
			ActualCycleNum = 0xFFFFFFFF;
			LastUpdateCycleNum = 0;
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public void SetActualCycleNum( uint ActualCycleNum )
		{
			this.ActualCycleNum = ActualCycleNum;
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public void Clear() // Warning, before using this funcion, you must SetActualCycleNum(...)
		{
			uint i;
			for( i = 0; i < StorageSize_Num_uints; i++ ) Storage[i] = 0;
			LastUpdateCycleNum = ActualCycleNum;
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public void Set( uint Index )
		{
			byte Remain;
			uint Offset;

			if( LastUpdateCycleNum != ActualCycleNum ) { Clear(); }

			Remain = (byte)( Index & 0x1f );
			Offset = Index >> 5;
			Storage[Offset] |= 1U << Remain;
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public bool Get( uint Index )
		{
			byte Remain;
			uint Offset;

			if( LastUpdateCycleNum != ActualCycleNum ) return ( false );

			Remain = (byte)( Index & 0x1f );
			Offset = Index >> 5;
			return ( Storage[Offset] & ( 1 << Remain ) ) != 0;
		}
	}
}
