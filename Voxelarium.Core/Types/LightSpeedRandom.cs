using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using Voxelarium.Core.Support;
using Voxelarium.Core.Voxels;

namespace Voxelarium.Core.Types
{
	class LightSpeedRandom
	{
		// Pooled random

		const int ZLIGHTSPEEDRANDOM_POOLLEN = 1024 * 1024;
		const int ZLIGHTSPEEDRANDOM_LENMASK = 0xFFFFF;

		public static uint[] Pool;
		public static uint PoolLen;

		public uint Position;


		public LightSpeedRandom()
		{
			if( Pool == null )
			{
				try
				{
					FileStream fs = new FileStream( VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH + "/randomnum.dat", FileMode.Open );
					BinaryReader br = new BinaryReader( fs );

					Pool = new uint[ZLIGHTSPEEDRANDOM_POOLLEN];
					byte[] bytes = br.ReadBytes( 4 * ZLIGHTSPEEDRANDOM_POOLLEN );
					for( int i = 0; i < ZLIGHTSPEEDRANDOM_POOLLEN; i++ )
						Pool[i] = BitConverter.ToUInt32( bytes, i * 4 );
					PoolLen = ZLIGHTSPEEDRANDOM_POOLLEN;
					br.Close();
				}
				catch( Exception e )
				{
					Log.log( "LightSpeed Random init failed: " + e.Message );
				}
			}
			Position = 0;
		}

		~LightSpeedRandom()
		{
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public uint GetNumber()
		{
			return Pool[Position++ & ( ZLIGHTSPEEDRANDOM_POOLLEN - 1 )];
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public uint GetNumber( uint Seed )
		{
			return ( Pool[Seed & ( ZLIGHTSPEEDRANDOM_POOLLEN - 1 )] );
		}

		public bool RandomProbability( uint Seed, uint Proba32 )
		{
			Seed &= ( ZLIGHTSPEEDRANDOM_POOLLEN - 1 );
			uint RandomNum = Pool[Seed];
			bool ret = RandomNum < Proba32;
			return ( ret );
			// return( Pool[ Seed & (ZLIGHTSPEEDRANDOM_POOLLEN - 1)] < Proba32 );
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public void Init( uint Seed = 0 )
		{
			Position = Seed;
		}
	}

}
