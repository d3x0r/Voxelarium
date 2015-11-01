using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Types
{
	public class Random_LFSR
	{
		uint State;

		public Random_LFSR()
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
			uint i;

			for( i = 0; i < 4; i++ )
			{
				State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
				State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
				State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
				State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
				State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
				State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
				State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
				State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
			}
			return State;
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public uint Get8Bits()
		{
			State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
			State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
			State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
			State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
			State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
			State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
			State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
			State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );

			return ( State >> 24 );
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public bool RandomProbability( uint Seed, uint Proba32 )
		{
			uint i;

			for( i = 0; i < 40; i++ )
			{
				State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
				State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
				State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
				State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
				State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
				State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
				State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
				State = (uint)( ( State >> 1 ) ^ ( -(int)( State & 1u ) & 0xD0000001u ) );
			}
			if( Seed < Proba32 ) return ( true );
			return ( false );
		}

	};

}

