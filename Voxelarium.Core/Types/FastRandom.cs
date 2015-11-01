using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Types
{
	public class FastRandom
	{
		uint State;

		public FastRandom()
		{
			State = 1;
		}

		public void SetSeed( uint Seed )
		{
			State = Seed;
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public uint GetNumber()
		{
			State = ( State * 16807 ) % 2147483647;
			return ( State );
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public uint GetNumber( uint Seed )
		{
			return ( ( Seed * 16807 ) % 2147483647 );
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public bool RandomProbability( uint Seed, uint Proba32 )
		{

			if( Seed < Proba32 ) return ( true );
			return ( false );
		}
	}
}
