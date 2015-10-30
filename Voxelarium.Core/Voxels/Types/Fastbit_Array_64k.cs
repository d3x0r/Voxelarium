using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Voxels.Types
{
	public class FastBit_Array_64k
	{
		internal uint[] Storage = new uint[2048];

		~FastBit_Array_64k()
		{
			Storage = null;
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal void Clear()
		{
			ushort i;
			for( i = 0; i < 2048; i++ ) Storage[i] = 0;
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal void Set( int Index, bool Data )
		{
			int Remain;
			int Offset;
			Remain = Index;
			Offset = Index >> 5;
			Remain &= 0x1f;
			if( Data ) Storage[Offset] |= (uint)1 << Remain;
			else Storage[Offset] &= ~( (uint)1 << Remain );
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal bool Get( ushort Index )
		{
			ushort Remain;
			int Offset;
			Remain = Index;
			Offset = Index >> 5;
			Remain &= 0x1f;
			return ( Storage[Offset] & ( (uint)1 << Remain ) ) != 0;
		}
	}
}
