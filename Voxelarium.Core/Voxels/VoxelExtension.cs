using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace Voxelarium.Core.Voxels
{
	public abstract class VoxelExtension : IDisposable
	{
		public ExtensionTypes ExtensionType;

		public enum ExtensionTypes
		{
			Extension_None = 0
			, Extension_Storage = 1
			, Extension_UserTextureTransformer = 2
			, Extension_PlaneZ1 = 3
			, Extension_TransformationMachine = 4
			, Extension_Programmable = 5
			, Extension_FusionElement = 6
			, Extension_BlastFurnace = 7
			, Extension_MiningRobot_xr1 = 8
			  , Extension_Sequencer = 9
			  , Extension_Egmy_T1 = 10
			  , Extension_FertileGround
			  , Extension_Food
			  , Extension_Animal
			  , Extension_Aroma
			  , Extension_AromaGenerator
			  , Extension_Count
		};

		//public static ULong[] ExtensionCharCodes;

		//protected:
		//	bool _ThrowExtension( ZStream_SpecialRamStream* Stream, ZMemSize ExtensionSize );

		public static uint MulticharConst( char a, char b, char c, char d ) { return ( (uint)a ) | ( (uint)b << 9) | ( (uint)c << 16 ) | ( (uint)d << 24 ); }
		public virtual uint GetExtensionID() { return ( MulticharConst( 'N', 'S', 'P', 'C' ) ); }
		public abstract bool Save( Stream Stream );
		public abstract bool Load( Stream Stream );
		//public virtual void SetGameEnv( VoxelGameEnvironment GameEnv ) { }
		public virtual VoxelExtension GetNewCopy() { return ( null ); }
		public void Dispose() { }

		bool _ThrowExtension( Stream Stream, uint ExtensionSize )
		{
			ExtensionSize -= 2;
			try
			{
				for( uint i = 0; i < ExtensionSize; i++ ) Stream.ReadByte();
			}
			catch( Exception e )
			{
				return false;
			}

			return true;
		}


		public static uint[] ExtensionCharCodes = {
			 0
			, MulticharConst( 'S', 'T', 'O', 'R' )
			, MulticharConst( 'U', 'T', 'T', 'R' )
			, MulticharConst( 'P', 'L', 'Z', '1' )
			, MulticharConst( 'F', 'M', 'C', 'H' )
			, MulticharConst( 'P', 'R', 'O', 'G' )
			, MulticharConst( 'F', 'U', 'S', 'E' )
			, MulticharConst( 'B', 'F', 'U', 'R' ) // 7 blst furnace
			, MulticharConst( 'M', 'R', 'X', '1' )  //8
			, MulticharConst( 'S', 'E', 'Q', 'U' )
			, MulticharConst( 'E', 'M', 'Y', '1' ) // 10 egmy
			, MulticharConst( 'B', 'F', 'G', 'R' ) // fertile ground
			, MulticharConst( 'F', 'O', 'O', 'D' )
			, MulticharConst( 'B', 'A', 'N', 'I' )  // animal
			, MulticharConst( 'A', 'R', 'M', 'A' )  // aroma
			, MulticharConst( 'A', 'R', 'M', 'G' )   // aroma generator
		//, // count unused
	};

#if asdfasdf
		public inline void* operator new ( size_t Size )
		{
			return ( ZMemPool_Optimized::GetMainPublicPool()->AllocMem( Size ) );
		}

		inline void operator delete ( void* Memory )
		{
			ZMemPool_Optimized::GetMainPublicPool()->FreeMem( Memory );
		}
#endif
}
}
