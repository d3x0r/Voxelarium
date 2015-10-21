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
