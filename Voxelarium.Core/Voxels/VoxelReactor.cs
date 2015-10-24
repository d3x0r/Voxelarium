using OpenTK;
using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Voxels
{
	class VoxelReactor
	{

		//public static ZLightSpeedRandom Random;

		public static SaltyRandomGenerator Random2;

		public static VoxelSector DummySector;

		Vector3 PlayerPosition;
		VoxelGameEnvironment GameEnv;
		VoxelWorld World;
		VoxelTypeManager VoxelTypeManager;
		//ZEgmyTargetManager EgmyWaveManager;
		ulong CycleNum;


		//ZVoxelReaction** ReactionTable;
#if asdfasdfasdf
		public:
		public struct ZBlocPos { public byte x; byte y; byte z; };
		public struct ZBlocPosN { public sbyte x; sbyte y; sbyte z; };
		static ZBlocPos bfta[26];  // bloc flow table (air)
		static ZBlocPos bfts[18];  // bloc flow table (smoothing)
		static ZBlocPos bp6[6];   // Bloc positions with 6 slots around main cube.
		static ZBlocPos bft[8];   // Bloc fall test positions with 4 slots around and 4 slots under;
		static ZBlocPos bft6[10]; // Bloc fall test positions with 6 slots around main cube and 4 slots under (Special case for acid).
		static UByte BlocOpposite[6];
		static ZBlocPosN nbp6[6];
		static ZBlocPos xbp6[6];  // Bloc positions with 6 slots around main cube. ( New standardised robot order.).
		static ZBlocPos xbp6_opposing[6];  // Bloc positions with 6 slots around main cube. ( New standardised robot order.).
		static RelativeVoxelOrds x6_opposing_escape[6,5];  // Bloc positions with 6 slots around main cube. ( New standardised robot order.).
	static ZBlocPosN xbp6_opposing_escape[6,5]	;
    static ZBlocPosN xbp6_nc[6];// same as xbp6 with -1,+1 range
#endif

		// Fast computing offsets;
		static public byte[] Of_x = new byte[VoxelSector.ZVOXELBLOCSIZE_X + 2];
		static public byte[] Of_y = new byte[VoxelSector.ZVOXELBLOCSIZE_Y + 2];
		static public byte[] Of_z = new byte[VoxelSector.ZVOXELBLOCSIZE_Z + 2];
		static public uint[] If_x = new uint[VoxelSector.ZVOXELBLOCSIZE_X + 2];
		static public uint[] If_y = new uint[VoxelSector.ZVOXELBLOCSIZE_Y + 2];
		static public uint[] If_z = new uint[VoxelSector.ZVOXELBLOCSIZE_Z + 2];

		// DirCodes

		public static byte[] DirCodeTable = new byte[16];

#if asdfasdf
		// Time remaining on FireMine action
		ULong FireMineTime;
#endif


		public
			void Init( VoxelGameEnvironment GameEnv )
		{
			this.GameEnv = GameEnv;
			this.World = GameEnv.World;
			this.VoxelTypeManager = GameEnv.VoxelTypeManager;
			PlayerPosition.X = PlayerPosition.Y = PlayerPosition.Z = 0;

		}


		internal VoxelReactor()
		{
			int i;

			if( Random2 == null )
			{
				// one-time inits for static members.
				Random2 = new SaltyRandomGenerator();
				//Random.Init( 0 );
				// Dummy Sector

				DummySector = new VoxelSector( ( VoxelWorld ) null );
				DummySector.Fill( 0xFFFF );

				// Multiplexing Sector Tables for fast access to voxels
				Of_x[0] = 0; Of_x[VoxelSector.ZVOXELBLOCSIZE_X + 1] = 2; for( i = 1; i <= VoxelSector.ZVOXELBLOCSIZE_X; i++ ) Of_x[i] = 1;
				Of_y[0] = 0; Of_y[VoxelSector.ZVOXELBLOCSIZE_Y + 1] = 8; for( i = 1; i <= VoxelSector.ZVOXELBLOCSIZE_Y; i++ ) Of_y[i] = 4;
				Of_z[0] = 0; Of_z[VoxelSector.ZVOXELBLOCSIZE_Z + 1] = 32; for( i = 1; i <= VoxelSector.ZVOXELBLOCSIZE_Z; i++ ) Of_z[i] = 16;

				// Multiplexing Voxel Tables for fast access to voxels

				If_x[0] = ( VoxelSector.ZVOXELBLOCSIZE_X - 1 ) * VoxelSector.ZVOXELBLOCSIZE_Y;
				If_x[VoxelSector.ZVOXELBLOCSIZE_X + 1] = 0;
				for( i = 0; i < VoxelSector.ZVOXELBLOCSIZE_X; i++ ) If_x[i + 1] = (uint)i * VoxelSector.ZVOXELBLOCSIZE_Y;
				If_y[0] = ( VoxelSector.ZVOXELBLOCSIZE_Y - 1 );
				If_y[VoxelSector.ZVOXELBLOCSIZE_Y + 1] = 0;
				for( i = 0; i < VoxelSector.ZVOXELBLOCSIZE_Y; i++ ) If_y[i + 1] = (uint)i;
				If_z[0] = ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X;
				If_z[VoxelSector.ZVOXELBLOCSIZE_Z + 1] = 0;
				for( i = 0; i < VoxelSector.ZVOXELBLOCSIZE_Z; i++ ) If_z[i + 1] = (uint)i * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X;

			}

#if asfasdf
			// Reaction table
			ReactionTable = new VoxelReaction[65536];
			for( i = 0; i < 65536; i++ ) ReactionTable[i] = 0;

			// Green acid reaction
			ReactionTable[86] = new ZVoxelReaction( 89, 0 );
			// ReactionTable[86].SetReaction(1,10,10);
#endif

			//ReactionTable[86].Set(1,10);
		}
		~VoxelReactor()
		{
			//ActiveTable = null;
			if( DummySector != null ) { DummySector.Dispose(); DummySector = null; }
		}


		void UpdatePlayerPosition( ref Vector3 PlayerPosition )
		{
			this.PlayerPosition = PlayerPosition;
		}


#if asdfasdf
		void LightTransmitter_FindEndPoints( ZVector3L* Location, ZVector3L* NewCommingDirection );
		void LightTransmitter_FollowTransmitter( ZVector3L* Location, ZVector3L* FollowingDirection );
		bool VoxelFluid_ComputeVolumePressure( ZVector3L* Location, UShort VoxelType, bool EvenCycle );
		void VoxelFluid_ComputeVolumePressure_Recurse( ZVector3L* Location, ZonePressure* Pr );
		void VoxelFluid_SetVolumePressure_Recurse( ZVector3L* Location, ZonePressure* Pr );
		void ProcessSectors( double LastLoopTime );
#endif

	}
}
