
using Bullet.LinearMath;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using Voxelarium.Core.Support;
using Voxelarium.Core.Voxels.Types;
using Voxelarium.Core.Voxels.UI;

namespace Voxelarium.Core.Voxels
{

	public class VoxelSector : VObject, IDisposable
	{
		public const int ZVOXELBLOCSHIFT_X = 5;
		public const int ZVOXELBLOCSHIFT_Y = 5;
		public const int ZVOXELBLOCSHIFT_Z = 5;
		public const uint ZVOXELBLOCSIZE_X = 1 << ZVOXELBLOCSHIFT_X;
		public const uint ZVOXELBLOCSIZE_Y = 1 << ZVOXELBLOCSHIFT_Y;
		public const uint ZVOXELBLOCSIZE_Z = 1 << ZVOXELBLOCSHIFT_Z;
		public const uint ZVOXELBLOCMASK_X = ZVOXELBLOCSIZE_X - 1;
		public const uint ZVOXELBLOCMASK_Y = ZVOXELBLOCSIZE_Y - 1;
		public const uint ZVOXELBLOCMASK_Z = ZVOXELBLOCSIZE_Z - 1;

		static bool Initialized;
		public static byte[] STableX = new byte[ZVOXELBLOCSIZE_X + 2];//= { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2 };
		public static byte[] STableZ = new byte[ZVOXELBLOCSIZE_Z + 2];// { 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6 };
		public static byte[] STableY = new byte[ZVOXELBLOCSIZE_Y + 2];//{9,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
																	  //0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
																	  //0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
																	  //0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
																	  //0,18};
		public static ushort[] OfTableY = new ushort[ZVOXELBLOCSIZE_Y + 2];//= {63, 0,1,2,3, 4,5,6,7, 8,9,10,11, 12,13,14,15,
																		   //16,17,18,19, 20,21,22,23, 24,25,26,27, 28,29,30,31,
																		   //32,33,34,35, 36,37,38,39, 40,41,42,43, 44,45,46,47,
																		   //48,49,50,51, 52,53,54,55, 56,57,58,59, 60,61,62,63,
																		   //0};
		public static ushort[] OfTableX = new ushort[ZVOXELBLOCSIZE_X + 2];//{  15*ZVOXELBLOCSIZE_Y,
																		   //	0*ZVOXELBLOCSIZE_Y,  1*ZVOXELBLOCSIZE_Y,  2*ZVOXELBLOCSIZE_Y, 3*ZVOXELBLOCSIZE_Y,
																		   //	4*ZVOXELBLOCSIZE_Y,  5*ZVOXELBLOCSIZE_Y,  6*ZVOXELBLOCSIZE_Y, 7*ZVOXELBLOCSIZE_Y,
																		   //	8*ZVOXELBLOCSIZE_Y,  9*ZVOXELBLOCSIZE_Y, 10*ZVOXELBLOCSIZE_Y,11*ZVOXELBLOCSIZE_Y,
																		   //   12*ZVOXELBLOCSIZE_Y, 13*ZVOXELBLOCSIZE_Y, 14*ZVOXELBLOCSIZE_Y,15*ZVOXELBLOCSIZE_Y,
																		   //	0*ZVOXELBLOCSIZE_Y };
		public static ushort[] OfTableZ = new ushort[ZVOXELBLOCSIZE_Z + 2]; //{  15*ZVOXELBLOCSIZE_Y*ZVOXELBLOCSIZE_X,
					   //0 *ZVOXELBLOCSIZE_Y*ZVOXELBLOCSIZE_X, 1 *ZVOXELBLOCSIZE_Y*ZVOXELBLOCSIZE_X, 2*ZVOXELBLOCSIZE_Y*ZVOXELBLOCSIZE_X, 3*ZVOXELBLOCSIZE_Y*ZVOXELBLOCSIZE_X,
					   //4 *ZVOXELBLOCSIZE_Y*ZVOXELBLOCSIZE_X, 5 *ZVOXELBLOCSIZE_Y*ZVOXELBLOCSIZE_X, 6*ZVOXELBLOCSIZE_Y*ZVOXELBLOCSIZE_X, 7*ZVOXELBLOCSIZE_Y*ZVOXELBLOCSIZE_X,
					   //8 *ZVOXELBLOCSIZE_Y*ZVOXELBLOCSIZE_X, 9 *ZVOXELBLOCSIZE_Y*ZVOXELBLOCSIZE_X,10*ZVOXELBLOCSIZE_Y*ZVOXELBLOCSIZE_X,11*ZVOXELBLOCSIZE_Y*ZVOXELBLOCSIZE_X,
					   //12*ZVOXELBLOCSIZE_Y*ZVOXELBLOCSIZE_X, 13*ZVOXELBLOCSIZE_Y*ZVOXELBLOCSIZE_X,14*ZVOXELBLOCSIZE_Y*ZVOXELBLOCSIZE_X,15*ZVOXELBLOCSIZE_Y*ZVOXELBLOCSIZE_X,
					   //0 *ZVOXELBLOCSIZE_Y*ZVOXELBLOCSIZE_X};


		// 14 bits requires for position
		// 18 bits left
		// 0x3FFF
		[Flags]
		public enum FACEDRAW_Operations
		{
			LEFT = 0x00001
		, RIGHT = 0x00002
		, AHEAD = 0x00004
		, BEHIND = 0x00008
		, ABOVE = 0x00010
		, BELOW = 0x00020
		, LEFT_HAS_ABOVE = 0x00000400
		, LEFT_HAS_BELOW = 0x00000800
		, LEFT_HAS_AHEAD = 0x00001000
		, LEFT_HAS_BEHIND = 0x00002000
		, RIGHT_HAS_ABOVE = 0x00004000
		, RIGHT_HAS_BELOW = 0x00008000
		, RIGHT_HAS_AHEAD = 0x00010000
		, RIGHT_HAS_BEHIND = 0x00020000
		, ABOVE_HAS_LEFT = LEFT_HAS_ABOVE
		, ABOVE_HAS_RIGHT = RIGHT_HAS_ABOVE
		, ABOVE_HAS_AHEAD = 0x00040000
		, ABOVE_HAS_BEHIND = 0x00080000
		, BELOW_HAS_LEFT = LEFT_HAS_BELOW
		, BELOW_HAS_RIGHT = RIGHT_HAS_BELOW
		, BELOW_HAS_AHEAD = 0x00100000
		, BELOW_HAS_BEHIND = 0x00200000
		, AHEAD_HAS_LEFT = LEFT_HAS_AHEAD
		, AHEAD_HAS_RIGHT = RIGHT_HAS_AHEAD
		, AHEAD_HAS_ABOVE = ABOVE_HAS_AHEAD
		, AHEAD_HAS_BELOW = BELOW_HAS_AHEAD
		, BEHIND_HAS_LEFT = LEFT_HAS_BEHIND
		, BEHIND_HAS_RIGHT = RIGHT_HAS_BEHIND
		, BEHIND_HAS_ABOVE = ABOVE_HAS_BEHIND
		, BEHIND_HAS_BELOW = BELOW_HAS_BEHIND


		, ALL = ( LEFT | RIGHT | AHEAD | BEHIND | ABOVE | BELOW )
		, NONE = 0x00000
		, FLANK = ( LEFT | RIGHT | AHEAD | BEHIND )
		, UD = ( ABOVE | BELOW )
		, ALL_BITS = 0x3FFFFF
		};

		public enum RelativeVoxelOrds
		{
			INCENTER //0
		   , LEFT     //1
		   , RIGHT //2
		   , INFRONT //3
		   , AHEAD = INFRONT // 3
		   , BEHIND  //4
		   , ABOVE   //5
		   , BELOW   //6

		   , LEFT_ABOVE //7
		   , ABOVE_LEFT = LEFT_ABOVE
		   , RIGHT_ABOVE //8
		   , ABOVE_RIGHT = RIGHT_ABOVE

		   , INFRONT_ABOVE //9
		   , AHEAD_ABOVE = INFRONT_ABOVE
		   , ABOVE_AHEAD = INFRONT_ABOVE

		   , BEHIND_ABOVE  //10
		   , ABOVE_BEHIND = BEHIND_ABOVE

		   , LEFT_AHEAD   //11
		   , AHEAD_LEFT = LEFT_AHEAD

		   , RIGHT_AHEAD   //12
		   , AHEAD_RIGHT = RIGHT_AHEAD

		   , LEFT_BELOW //13
		   , BELOW_LEFT = LEFT_BELOW
		   , RIGHT_BELOW //14
		   , BELOW_RIGHT = RIGHT_BELOW
		   , INFRONT_BELOW //15
		   , AHEAD_BELOW = INFRONT_BELOW
		   , BELOW_AHEAD = INFRONT_BELOW
		   , BEHIND_BELOW  //16
		   , BELOW_BEHIND = BEHIND_BELOW

		   , LEFT_BEHIND   //17
		   , BEHIND_LEFT = LEFT_BEHIND
		   , BEHIND_RIGHT   //18
		   , RIGHT_BEHIND = BEHIND_RIGHT


		   , LEFT_AHEAD_ABOVE   // 19
		   , RIGHT_AHEAD_ABOVE  // 20
		   , LEFT_AHEAD_BELOW   // 21
		   , RIGHT_AHEAD_BELOW  // 22
		   , LEFT_BEHIND_ABOVE  // 23
		   , RIGHT_BEHIND_ABOVE // 24
		   , LEFT_BEHIND_BELOW  // 25
		   , RIGHT_BEHIND_BELOW // 26

		   , LEFT_ABOVE_AHEAD = LEFT_AHEAD_ABOVE   // 19
		   , RIGHT_ABOVE_AHEAD = RIGHT_AHEAD_ABOVE   // 20
		   , LEFT_BELOW_AHEAD = LEFT_AHEAD_BELOW     // 21
		   , RIGHT_BELOW_AHEAD = RIGHT_AHEAD_BELOW   // 22
		   , LEFT_ABOVE_BEHIND = LEFT_BEHIND_ABOVE  // 23
		   , RIGHT_ABOVE_BEHIND = RIGHT_BEHIND_ABOVE // 24
		   , LEFT_BELOW_BEHIND = LEFT_BEHIND_BELOW  // 25
		   , RIGHT_BELOW_BEHIND = RIGHT_BEHIND_BELOW // 26

		   , ABOVE_AHEAD_LEFT = LEFT_AHEAD_ABOVE   // 19
		   , ABOVE_AHEAD_RIGHT = RIGHT_AHEAD_ABOVE   // 20
		   , BELOW_AHEAD_LEFT = LEFT_AHEAD_BELOW     // 21
		   , BELOW_AHEAD_RIGHT = RIGHT_AHEAD_BELOW   // 22
		   , ABOVE_BEHIND_LEFT = LEFT_BEHIND_ABOVE  // 23 
		   , ABOVE_BEHIND_RIGHT = RIGHT_BEHIND_ABOVE // 24
		   , BELOW_BEHIND_LEFT = LEFT_BEHIND_BELOW  // 25 
		   , BELOW_BEHIND_RIGHT = RIGHT_BEHIND_BELOW // 26

		   , AHEAD_ABOVE_LEFT = LEFT_AHEAD_ABOVE   // 19
		   , AHEAD_ABOVE_RIGHT = RIGHT_AHEAD_ABOVE   // 20
		   , AHEAD_BELOW_LEFT = LEFT_AHEAD_BELOW     // 21
		   , AHEAD_BELOW_RIGHT = RIGHT_AHEAD_BELOW   // 22
		   , BEHIND_ABOVE_LEFT = LEFT_BEHIND_ABOVE  // 23 
		   , BEHIND_ABOVE_RIGHT = RIGHT_BEHIND_ABOVE // 24
		   , BEHIND_BELOW_LEFT = LEFT_BEHIND_BELOW  // 25 
		   , BEHIND_BELOW_RIGHT = RIGHT_BEHIND_BELOW // 26

		   , ABOVE_LEFT_AHEAD = LEFT_AHEAD_ABOVE   // 19
		   , ABOVE_RIGHT_AHEAD = RIGHT_AHEAD_ABOVE   // 20
		   , BELOW_LEFT_AHEAD = LEFT_AHEAD_BELOW     // 21
		   , BELOW_RIGHT_AHEAD = RIGHT_AHEAD_BELOW   // 22
		   , ABOVE_LEFT_BEHIND = LEFT_BEHIND_ABOVE  // 23 
		   , ABOVE_RIGHT_BEHIND = RIGHT_BEHIND_ABOVE // 24
		   , BELOW_LEFT_BEHIND = LEFT_BEHIND_BELOW  // 25 
		   , BELOW_RIGHT_BEHIND = RIGHT_BEHIND_BELOW // 26

		   , AHEAD_LEFT_ABOVE = LEFT_AHEAD_ABOVE   // 19
		   , AHEAD_RIGHT_ABOVE = RIGHT_AHEAD_ABOVE   // 20
		   , AHEAD_LEFT_BELOW = LEFT_AHEAD_BELOW     // 21
		   , AHEAD_RIGHT_BELOW = RIGHT_AHEAD_BELOW   // 22
		   , BEHIND_LEFT_ABOVE = LEFT_BEHIND_ABOVE  // 23 
		   , BEHIND_RIGHT_ABOVE = RIGHT_BEHIND_ABOVE // 24
		   , BEHIND_LEFT_BELOW = LEFT_BEHIND_BELOW  // 25 
		   , BEHIND_RIGHT_BELOW = RIGHT_BEHIND_BELOW // 26

		};



		public static uint[] RelativeVoxelOffsets_Unwrapped = new uint[27];
		public static FACEDRAW_Operations[] RelativeVoxelOffset_Fixups = new FACEDRAW_Operations[27]; // (ordered the same way voxel order is?)... but as a bitmask of draw ops
		public static uint[] RelativeVoxelOffsets_Wrapped = new uint[27];
		static public RelativeVoxelOrds[] VoxelSectorReactorMapTemp = new RelativeVoxelOrds[] {
						RelativeVoxelOrds.LEFT_BELOW_BEHIND
			, RelativeVoxelOrds.BELOW_BEHIND
			, RelativeVoxelOrds.RIGHT_BELOW_BEHIND
			, 0
			, RelativeVoxelOrds.LEFT_BEHIND
			, RelativeVoxelOrds.BEHIND
			, RelativeVoxelOrds.RIGHT_BEHIND
			, 0
			, RelativeVoxelOrds.LEFT_ABOVE_BEHIND
			, RelativeVoxelOrds.ABOVE_BEHIND
			, RelativeVoxelOrds.RIGHT_ABOVE_BEHIND
			, 0
			, 0, 0, 0, 0
			, RelativeVoxelOrds.LEFT_BELOW
			, RelativeVoxelOrds.BELOW
			, RelativeVoxelOrds.RIGHT_BELOW
			, 0
			, RelativeVoxelOrds.LEFT
			, RelativeVoxelOrds.INCENTER
			, RelativeVoxelOrds.RIGHT
			, 0
			, RelativeVoxelOrds.LEFT_ABOVE
			, RelativeVoxelOrds.ABOVE
			, RelativeVoxelOrds.RIGHT_ABOVE
			, 0
			, 0, 0, 0, 0
			, RelativeVoxelOrds.LEFT_BELOW_AHEAD
			, RelativeVoxelOrds.BELOW_AHEAD
			, RelativeVoxelOrds.RIGHT_BELOW_AHEAD
			, 0
			, RelativeVoxelOrds.LEFT_AHEAD
			, RelativeVoxelOrds.AHEAD
			, RelativeVoxelOrds.RIGHT_AHEAD
			, 0
			, RelativeVoxelOrds.LEFT_ABOVE_AHEAD
			, RelativeVoxelOrds.ABOVE_AHEAD
			, RelativeVoxelOrds.RIGHT_ABOVE_AHEAD
			, 0
			, 0, 0, 0, 0}; // for debugging purposes... map reactor maps to physical 27 3x3x3 maps
		public static RelativeVoxelOrds[,] VoxelFaceGroups = new RelativeVoxelOrds[6, 9] { {  RelativeVoxelOrds.LEFT,
			RelativeVoxelOrds.LEFT_ABOVE,
			RelativeVoxelOrds.LEFT_BELOW,
			RelativeVoxelOrds.LEFT_AHEAD,
			RelativeVoxelOrds.LEFT_BEHIND,
			RelativeVoxelOrds.LEFT_ABOVE_AHEAD,
			RelativeVoxelOrds.LEFT_ABOVE_BEHIND,
			RelativeVoxelOrds.LEFT_BELOW_AHEAD,
			RelativeVoxelOrds.LEFT_BELOW_BEHIND },
			{ RelativeVoxelOrds.RIGHT,
			 RelativeVoxelOrds.RIGHT_ABOVE,
			 RelativeVoxelOrds.RIGHT_BELOW,
			 RelativeVoxelOrds.RIGHT_AHEAD,
			 RelativeVoxelOrds.RIGHT_BEHIND,
			 RelativeVoxelOrds.RIGHT_ABOVE_AHEAD,
			 RelativeVoxelOrds.RIGHT_ABOVE_BEHIND,
			 RelativeVoxelOrds.RIGHT_BELOW_AHEAD,
			 RelativeVoxelOrds.RIGHT_BELOW_BEHIND },
			{ RelativeVoxelOrds.AHEAD,
			RelativeVoxelOrds.AHEAD_ABOVE,
			RelativeVoxelOrds.AHEAD_BELOW,
			RelativeVoxelOrds.AHEAD_LEFT,
			RelativeVoxelOrds.AHEAD_RIGHT,
			RelativeVoxelOrds.AHEAD_ABOVE_LEFT,
			RelativeVoxelOrds.AHEAD_ABOVE_RIGHT,
			RelativeVoxelOrds.AHEAD_BELOW_LEFT,
			RelativeVoxelOrds.AHEAD_BELOW_RIGHT } ,
			{ RelativeVoxelOrds.BEHIND,
			RelativeVoxelOrds.BEHIND_ABOVE,
			RelativeVoxelOrds.BEHIND_BELOW,
			RelativeVoxelOrds.BEHIND_LEFT,
			RelativeVoxelOrds.BEHIND_RIGHT,
			RelativeVoxelOrds.BEHIND_ABOVE_LEFT,
			RelativeVoxelOrds.BEHIND_ABOVE_RIGHT,
			RelativeVoxelOrds.BEHIND_BELOW_LEFT,
			RelativeVoxelOrds.BEHIND_BELOW_RIGHT } ,
			 { RelativeVoxelOrds.ABOVE,
			 RelativeVoxelOrds.ABOVE_LEFT,
			 RelativeVoxelOrds.ABOVE_RIGHT,
			 RelativeVoxelOrds.ABOVE_AHEAD,
			 RelativeVoxelOrds.ABOVE_BEHIND,
			 RelativeVoxelOrds.ABOVE_LEFT_AHEAD,
			 RelativeVoxelOrds.ABOVE_LEFT_BEHIND,
			 RelativeVoxelOrds.ABOVE_RIGHT_AHEAD,
			 RelativeVoxelOrds.ABOVE_RIGHT_BEHIND },

			 { RelativeVoxelOrds.BELOW,
			 RelativeVoxelOrds.BELOW_LEFT,
			 RelativeVoxelOrds.BELOW_RIGHT,
			 RelativeVoxelOrds.BELOW_AHEAD,
			 RelativeVoxelOrds.BELOW_BEHIND,
			 RelativeVoxelOrds.BELOW_LEFT_AHEAD,
			 RelativeVoxelOrds.BELOW_LEFT_BEHIND,
			 RelativeVoxelOrds.BELOW_RIGHT_AHEAD,
			 RelativeVoxelOrds.BELOW_RIGHT_BEHIND } ,
		};

		public int SectorsInMemory;

		internal VoxelTypeManager VoxelTypeManager;

		public VoxelSector Next;
		public VoxelSector Pred;

		public VoxelSector GlobalList_Next;
		public VoxelSector GlobalList_Pred;

		public VoxelSector[] near_sectors = new VoxelSector[27];

		short Handle_x, Handle_y, Handle_z;
		public int Pos_x, Pos_y, Pos_z;
		internal uint Size_x, Size_y, Size_z;

		// Version control : Added for handling better world evolution.
		ushort ZoneType;     // The type of the zone.
		ushort ZoneVersion;  // The version of the generator used for this zone.
		ushort GeneratorVersion; // Main generator version. Updated at world change.
		ushort RingNum;

		internal VoxelWorld world;

		// Is_Modified field Values.
		[Flags]
		public enum ModifiedFieldFlags
		{
			NONE = 0,         // No changes. Assigned at sector creation.
			SAVEMASK = 28,    // If one of theses bits are set, sector must be saved.
			BITSECTORMODIFIED = 1, // If sector is modified for any reason, this bit is set.
			BITDISCARDABLE = 2, // Change aren't important and must be discarded at saving.
			BITUNIMPORTANT = 4, // Change aren't important but must be saved anyway. The sector can be discarded if changes are made to the world.
			BITIMPORTANT = 8, // The changes are important and must be preserved.
			BITCRITICAL = 16,// The changes are critical as they are done directly by player. These sectors must be preserved at all cost.

			// These values is what you MUST use with functions using "ImportanceFactor" field

			DISCARDABLE = BITDISCARDABLE | BITSECTORMODIFIED,  // Changes aren't important at all and MUST be discarded at saving.
			UNIMPORTANT = BITUNIMPORTANT | BITSECTORMODIFIED,   // Changes aren't important BUT MUST be saved anyway. Can be discarded if a new game version change zone disposition.
			IMPORTANT = BITIMPORTANT | BITSECTORMODIFIED,   // Changes are important and bust be preserved.
			CRITICAL = BITCRITICAL | BITSECTORMODIFIED    // Critical changes : work done directly by the player. Must be preserved at all cost.
		};

		public bool Flag_Void_Regular;
		public bool Flag_Void_Transparent;
		public bool Flag_Render_Dirty;
		public bool Flag_HighPriorityRefresh;
		public bool Flag_IsVisibleAtLastRendering;
		public bool Flag_DeletePending;
		public bool Flag_NeedFullCulling;
		public bool Flag_KeepInMemory;
		public ModifiedFieldFlags Flag_IsModified;        // This sector has been modified. Values are bitfields (See above).
		public bool Flag_IsSlowGeneration;   // This sector was generated using a very slow algorithm. Based on processer power, it may be a good idea to save it to disk rather than redoing the generation.
		public bool Flag_IsActiveVoxels;     // Active voxels in this sector needs voxel processor attention.
		public bool Flag_IsActiveLowRefresh; // Voxel processor will get activity in low frequency mode. Use LowRefresh_Mask to specify frequency;
		public bool Flag_IsDebug;            // Debug flag
		public bool Flag_NotStandardSize;
		public bool Flag_NeedSortedRendering; // Activate new rendering code for better speed in some zones.

		//bool Flag_NeedPartialCulling;
		//byte PartialCulling;
		public FACEDRAW_Operations PartialCulling;

		// Data stored by block
		public uint DataSize;
		public class VoxelData
		{
			public ushort[] Data = new ushort[ZVOXELBLOCSIZE_X * ZVOXELBLOCSIZE_Y * ZVOXELBLOCSIZE_Z];  // voxel type index
			public ushort[] TempInfos = new ushort[ZVOXELBLOCSIZE_X * ZVOXELBLOCSIZE_Y * ZVOXELBLOCSIZE_Z];// TempÃ©rature des voxels
			public VoxelExtension[] OtherInfos = new VoxelExtension[ZVOXELBLOCSIZE_X * ZVOXELBLOCSIZE_Y * ZVOXELBLOCSIZE_Z];// Informations autres
			//public byte[] FaceCulling = new byte[ZVOXELBLOCSIZE_X * ZVOXELBLOCSIZE_Y * ZVOXELBLOCSIZE_Z];
        }
		public VoxelData Data;

		internal VoxelCuller Culler;

		//public VObject DisplayData;
		internal VoxelGeometry geometry;
		public SectorModifiedTracker ModifTracker;

		public int RefreshWaitCount;

		public int LowRefresh_Mask;

#if delete_this
		void InitSector(); // Fill sector content for reuse.

		void CleanupSector(); //

		void Compress_Short_RLE( VoxelData* Data, void* Stream );
		void Compress_OtherInfos_RLE( VoxelData* Data, ushort* VoxelData, void* Stream );
		//void Compress_FaceCulling_RLE(byte * Data, void  * Stream);
		void Compress_Temperatures_RLE( VoxelData* Data, void* Stream );

		bool Decompress_Short_RLE( VoxelData* Data, void* Stream );
		//bool Decompress_FaceCulling_RLE(byte * Data, void * Stream);
		bool Decompress_OtherInfos_RLE( VoxelData* Data, void* Stream );
		bool Decompress_Temperatures_RLE( VoxelData* Data, void* Stream );
#endif

		internal void SetVoxelTypeManager( VoxelTypeManager VoxelTypeManager ) { this.VoxelTypeManager = VoxelTypeManager; }

		static void InitStatics()
		{
			if( !Initialized )
			{
				STableX[0] = 1;
				STableX[ZVOXELBLOCSIZE_X + 1] = 2;
				STableZ[0] = 3;
				STableZ[ZVOXELBLOCSIZE_Z + 1] = 6;
				STableY[0] = 9;
				STableY[ZVOXELBLOCSIZE_Y + 1] = 18;
				Initialized = false;

			}
		}

		public void ReinitSector()
		{
			CleanupSector(); InitSector();
#if VOXEL_CULLER
			Culler.InitFaceCullData( this );
#endif
		}
		public void SetPos( int x, int y, int z )
		{
			this.Pos_x = x;
			this.Pos_y = y;
			this.Pos_z = z;
		}

		// Handle point is set relative to default point.
		public void SetHandle( short x, short y, short z )
		{
			Handle_x = x;
			Handle_y = y;
			Handle_z = z;
		}

		public void SetNotStandardSize( bool NotStandardSize = true )
		{
			Flag_NotStandardSize = NotStandardSize;
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public void SetCube( int x, int y, int z, int CubeValue )
		{
			int Offset;
			Offset = ( y & (int)ZVOXELBLOCMASK_Y )
				+( ( x & (int)ZVOXELBLOCMASK_X ) * (int)ZVOXELBLOCSIZE_Y )
				+ ( ( z & (int)ZVOXELBLOCMASK_Z ) * ( (int)ZVOXELBLOCSIZE_Y * (int)ZVOXELBLOCSIZE_X ) );
			Data.Data[Offset] = (ushort)CubeValue;
			Data.OtherInfos[Offset] = null;
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		void SetCube_WithExtension( uint x, uint y, uint z, byte CubeValue, VoxelExtension Extension )
		{
			uint Offset;
			Offset = ( y & ZVOXELBLOCMASK_Y )
				+ ( ( x & ZVOXELBLOCMASK_X ) * ZVOXELBLOCSIZE_Y )
				+ ( ( z & ZVOXELBLOCMASK_Z ) * ( ZVOXELBLOCSIZE_Y * ZVOXELBLOCSIZE_X ) );
			Data.Data[Offset] = CubeValue;
			Data.OtherInfos[Offset] = Extension;
		}

		ushort GetCube( uint x, uint y, uint z )
		{
			uint Offset;
			Offset = ( y & ZVOXELBLOCMASK_Y )
				+ ( ( x & ZVOXELBLOCMASK_X ) * ZVOXELBLOCSIZE_Y )
				+ ( ( z & ZVOXELBLOCMASK_Z ) * ( ZVOXELBLOCSIZE_Y * ZVOXELBLOCSIZE_X ) );
			return ( Data.Data[Offset] );
		}

		void MakeSector()
		{
			uint x, y, z;
			byte Cnt;

			if( Pos_y < 0 ) { Cnt = 1; Flag_Void_Regular = false; Flag_Void_Transparent = true; }
			else { Cnt = 0; Flag_Void_Regular = true; Flag_Void_Transparent = true; }

			for( z = 0; z < Size_x; z++ )
			{
				for( y = 0; y < Size_y; y++ )
				{
					for( x = 0; x < Size_x; x++ )
					{
						SetCube( (int)x, (int)y, (int)z, Cnt );
					}
				}
			}
		}

		internal void Fill( ushort VoxelType )
		{
			uint i;

			for( i = 0; i < DataSize; i++ )
			{
				Data.Data[i] = VoxelType;
				//FaceCulling[i] = 0x3FFFFF;
			}
		}

		// Update control

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal bool IsMustBeSaved()
		{

			bool IsModified = /*(Flag_IsModified & BITSECTORMODIFIED) && */ ( Flag_IsModified & ModifiedFieldFlags.SAVEMASK ) != 0;

			return ( IsModified ); // Save only if sector is modified AND if modifications are rated important enough.
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public static void GetNearVoxel( VoxelSector Sector, uint origin_offset
					, out VoxelSector SectorOut, out uint offsetOut, RelativeVoxelOrds direction )
		{
			SectorOut = Sector;
			offsetOut = origin_offset + RelativeVoxelOffsets_Unwrapped[(int)direction];
			FACEDRAW_Operations fixup = RelativeVoxelOffset_Fixups[(int)direction];
			if( ( ( fixup & FACEDRAW_Operations.LEFT ) != 0 )
				&& ( origin_offset & ( ZVOXELBLOCMASK_X << ZVOXELBLOCSHIFT_Y ) ) == 0 )
			{
				offsetOut += ( ZVOXELBLOCSIZE_X ) * ZVOXELBLOCSIZE_Y;
				SectorOut = SectorOut.near_sectors[(int)RelativeVoxelOrds.LEFT - 1];
			}
			if( ( fixup & FACEDRAW_Operations.RIGHT ) != 0 && ( ( ( origin_offset & ( ZVOXELBLOCMASK_X << ZVOXELBLOCSHIFT_Y ) ) ^ ( ZVOXELBLOCMASK_X << ZVOXELBLOCSHIFT_Y ) ) == 0 ) )
			{
				if( SectorOut != null )
				{
					offsetOut -= ( ZVOXELBLOCSIZE_X ) * ZVOXELBLOCSIZE_Y;
					SectorOut = SectorOut.near_sectors[(int)RelativeVoxelOrds.RIGHT - 1];
				}
			}
			if( ( fixup & FACEDRAW_Operations.AHEAD ) != 0 && ( ( ( origin_offset & ( ZVOXELBLOCMASK_Z << ( ZVOXELBLOCSHIFT_X + ZVOXELBLOCSHIFT_Y ) ) ^ ( ZVOXELBLOCMASK_Z << ( ZVOXELBLOCSHIFT_X + ZVOXELBLOCSHIFT_Y ) ) ) ) == 0 ) )
			{
				if( SectorOut != null )
				{
					offsetOut -= ( ZVOXELBLOCSIZE_Z ) * ZVOXELBLOCSIZE_Y * ZVOXELBLOCSIZE_X;
					SectorOut = SectorOut.near_sectors[(int)RelativeVoxelOrds.AHEAD - 1];
				}
			}
			if( ( fixup & FACEDRAW_Operations.BEHIND ) != 0 && ( ( origin_offset & ( ZVOXELBLOCMASK_Z << ( ZVOXELBLOCSHIFT_X + ZVOXELBLOCSHIFT_Y ) ) ) == 0 ) )
			{
				if( SectorOut != null )
				{
					offsetOut += ( ZVOXELBLOCSIZE_Z ) * ZVOXELBLOCSIZE_Y * ZVOXELBLOCSIZE_X;
					SectorOut = SectorOut.near_sectors[(int)RelativeVoxelOrds.BEHIND - 1];
				}
			}
			if( ( fixup & FACEDRAW_Operations.ABOVE ) != 0 && ( ( ( origin_offset & ( ZVOXELBLOCMASK_Y ) ) ^ ( ZVOXELBLOCMASK_Y ) ) == 0 ) )
			{
				if( SectorOut != null )
				{
					offsetOut -= ZVOXELBLOCSIZE_Y;
					SectorOut = SectorOut.near_sectors[(int)RelativeVoxelOrds.ABOVE - 1];
				}
			}
			if( ( fixup & FACEDRAW_Operations.BELOW ) != 0 && ( ( origin_offset & ( ZVOXELBLOCMASK_Y ) ) == 0 ) )
			{
				if( SectorOut != null )
				{
					offsetOut += ZVOXELBLOCSIZE_Y;
					SectorOut = SectorOut.near_sectors[(int)RelativeVoxelOrds.BELOW - 1];
				}
			}
		}


		static uint OffsetDelta( int x, int y, int z )
		{
			return (uint)( ( ( x ) * ZVOXELBLOCSIZE_Y ) + ( y ) + ( ( z ) * ZVOXELBLOCSIZE_Y * ZVOXELBLOCSIZE_X ) );
		}
		static uint OffsetDeltaWrapped( int x, int y, int z )
		{
			return (uint)( ( ( ( ( x ) > 0 ? ( -( ZVOXELBLOCSIZE_X ) ) : ( x ) < 0 ? ( ZVOXELBLOCSIZE_X ) : 0 ) ) * ZVOXELBLOCSIZE_Y )
                   + ( ( ( ( y ) > 0 ? ( -( ZVOXELBLOCSIZE_Y ) ) : ( y ) < 0 ? ( ZVOXELBLOCSIZE_Y ) : 0 ) ) )
					+ ( ( ( ( ( z ) > 0 ? ( -( ZVOXELBLOCSIZE_Z ) ) : ( z ) < 0 ? ( ZVOXELBLOCSIZE_Z ) : 0 ) ) ) * ZVOXELBLOCSIZE_Y * ZVOXELBLOCSIZE_X ) );
		}
		void DefaultInit()
		{
			if( RelativeVoxelOffsets_Unwrapped[1] == 0 )
			{
				// these should have been done in-line... but forgot; and it became long serial code 
				{
					int n, f;
					for( n = 0; n < 6; n++ )
						for( f = 0; f < 9; f++ )
							RelativeVoxelOffset_Fixups[(int)VoxelFaceGroups[n, f]] |= (FACEDRAW_Operations)( 1 << n );
				}

				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.LEFT] = OffsetDelta( -1, 0, 0 );
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.RIGHT] = OffsetDelta( 1, 0, 0 );

				// x not on bound, y not on bound.
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.ABOVE] = OffsetDelta( 0, 1, 0 );
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.BELOW] = OffsetDelta( 0, -1, 0 );
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.LEFT_ABOVE] = OffsetDelta( -1, 1, 0 );
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.LEFT_BELOW] = OffsetDelta( -1, -1, 0 );
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.RIGHT_ABOVE] = OffsetDelta( 1, 1, 0 );
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.RIGHT_BELOW] = OffsetDelta( 1, -1, 0 );

				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.AHEAD] = OffsetDelta( 0, 0, 1 );
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.BEHIND] = OffsetDelta( 0, 0, -1 );
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.AHEAD_LEFT] = OffsetDelta( -1, 0, 1 );
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.AHEAD_RIGHT] = OffsetDelta( 1, 0, 1 );
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.BEHIND_LEFT] = OffsetDelta( -1, 0, -1 );
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.BEHIND_RIGHT] = OffsetDelta( 1, 0, -1 );

				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.ABOVE_AHEAD] = OffsetDelta( 0, 1, 1 );
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.ABOVE_BEHIND] = OffsetDelta( 0, 1, -1 );
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.BELOW_AHEAD] = OffsetDelta( 0, -1, 1 );
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.BELOW_BEHIND] = OffsetDelta( 0, -1, -1 );

				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = OffsetDelta( -1, 1, 1 );
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.LEFT_BELOW_AHEAD] = OffsetDelta( -1, -1, 1 );
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = OffsetDelta( 1, 1, 1 );
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = OffsetDelta( 1, -1, 1 );
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = OffsetDelta( -1, 1, -1 );
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.LEFT_BELOW_BEHIND] = OffsetDelta( -1, -1, -1 );
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = OffsetDelta( 1, 1, -1 );
				RelativeVoxelOffsets_Unwrapped[(int)RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = OffsetDelta( 1, -1, -1 );

				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.LEFT] = OffsetDeltaWrapped( -1, 0, 0 );
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.RIGHT] = OffsetDeltaWrapped( 1, 0, 0 );
				// x not on bound, y not on bound.
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.ABOVE] = OffsetDeltaWrapped( 0, 1, 0 );
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.BELOW] = OffsetDeltaWrapped( 0, -1, 0 );
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.LEFT_ABOVE] = OffsetDeltaWrapped( -1, 1, 0 );
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.LEFT_BELOW] = OffsetDeltaWrapped( -1, -1, 0 );
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.RIGHT_ABOVE] = OffsetDeltaWrapped( 1, 1, 0 );
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.RIGHT_BELOW] = OffsetDeltaWrapped( 1, -1, 0 );

				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.AHEAD] = OffsetDeltaWrapped( 0, 0, 1 );
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.BEHIND] = OffsetDeltaWrapped( 0, 0, -1 );
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.AHEAD_LEFT] = OffsetDeltaWrapped( -1, 0, 1 );
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.AHEAD_RIGHT] = OffsetDeltaWrapped( 1, 0, 1 );
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.BEHIND_LEFT] = OffsetDeltaWrapped( -1, 0, -1 );
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.BEHIND_RIGHT] = OffsetDeltaWrapped( 1, 0, -1 );

				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.ABOVE_AHEAD] = OffsetDeltaWrapped( 0, 1, 1 );
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.ABOVE_BEHIND] = OffsetDeltaWrapped( 0, 1, -1 );
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.BELOW_AHEAD] = OffsetDeltaWrapped( 0, -1, 1 );
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.BELOW_BEHIND] = OffsetDeltaWrapped( 0, -1, -1 );

				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = OffsetDeltaWrapped( -1, 1, 1 );
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.LEFT_BELOW_AHEAD] = OffsetDeltaWrapped( -1, -1, 1 );
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = OffsetDeltaWrapped( 1, 1, 1 );
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = OffsetDeltaWrapped( 1, -1, 1 );
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = OffsetDeltaWrapped( -1, 1, -1 );
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.LEFT_BELOW_BEHIND] = OffsetDeltaWrapped( -1, -1, -1 );
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = OffsetDeltaWrapped( 1, -1, -1 );
				RelativeVoxelOffsets_Wrapped[(int)RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = OffsetDeltaWrapped( 1, -1, -1 );


			}
			VoxelTypeManager = null;
			Size_x = ZVOXELBLOCSIZE_X;
			Size_y = ZVOXELBLOCSIZE_Y;
			Size_z = ZVOXELBLOCSIZE_Z;
			Handle_x = Handle_y = Handle_z = 0;

			DataSize = (uint)( Size_x * Size_y * Size_z );
			//DisplayData = null;
			//Data        = new VoxelData();
			//FaceCulling = new int [DataSize];
			//OtherInfos  = new uint[DataSize];
			//TempInfos   = new ushort[DataSize];

			Next = null;
			Pred = null;
			GlobalList_Next = null;
			GlobalList_Pred = null;

			InitSector();

			SectorsInMemory++;
		}

		public VoxelSector( VoxelWorld world )
		{
			this.world = world;
			ModifTracker.Init( ZVOXELBLOCSIZE_X * ZVOXELBLOCSIZE_Y * ZVOXELBLOCSIZE_Z );
			DefaultInit();
		}
#if VOXEL_CULLER
		public ZVoxelSector( ZVoxelCuller culler )
		{
			ModifTracker.Init( ZVOXELBLOCSIZE_X * ZVOXELBLOCSIZE_Y * ZVOXELBLOCSIZE_Z );
			DefaultInit();
			culler.InitFaceCullData( this );
		}
#endif
		public VoxelSector( VoxelSector Sector )
		{
			uint DataSize;

			DataSize = Sector.DataSize;

			Data = new VoxelData();
			//OtherInfos  = new uint[DataSize];
			//TempInfos   = new ushort[DataSize];
			for( int i = 0; i < DataSize; i++ )
			{
				Data.Data[i] = Sector.Data.Data[i];
				Data.TempInfos[i] = Sector.Data.TempInfos[i];
			}
			//memcpy(FaceCulling, Sector.FaceCulling, DataSize);
			//memcpy(OtherInfos, Sector.OtherInfos, DataSize * sizeof(uint));
			//memcpy(TempInfos, Sector.TempInfos, DataSize << 2);

			VoxelTypeManager = null;
			Next = Pred = GlobalList_Next = GlobalList_Pred = null;
			Handle_x = Sector.Handle_x;
			Handle_y = Sector.Handle_y;
			Handle_z = Sector.Handle_z;
			Pos_x = Sector.Pos_x;
			Pos_y = Sector.Pos_y;
			Pos_z = Sector.Pos_z;
			Size_x = Sector.Size_x;
			Size_y = Sector.Size_y;
			Size_z = Sector.Size_z;

			Flag_Void_Regular = Sector.Flag_Void_Regular;
			Flag_Void_Transparent = Sector.Flag_Void_Transparent;
			Flag_Render_Dirty = Sector.Flag_Render_Dirty;

			Flag_HighPriorityRefresh = Sector.Flag_HighPriorityRefresh;
			Flag_IsVisibleAtLastRendering = Sector.Flag_IsVisibleAtLastRendering;
			Flag_DeletePending = Sector.Flag_DeletePending;
			Flag_NeedFullCulling = Sector.Flag_NeedFullCulling;
			Flag_KeepInMemory = Sector.Flag_KeepInMemory;
			Flag_IsModified = Sector.Flag_IsModified;
			Flag_IsSlowGeneration = Sector.Flag_IsSlowGeneration;
			Flag_IsActiveVoxels = Sector.Flag_IsActiveVoxels;
			Flag_IsActiveLowRefresh = Sector.Flag_IsActiveLowRefresh;
			Flag_IsDebug = Sector.Flag_IsDebug;
			Flag_NotStandardSize = Sector.Flag_NotStandardSize;
			Flag_NeedSortedRendering = Sector.Flag_NeedSortedRendering;
			PartialCulling = Sector.PartialCulling;

			RefreshWaitCount = Sector.RefreshWaitCount;
			LowRefresh_Mask = Sector.LowRefresh_Mask;
		}

		void InitSector()
		{
			int i;

			Pos_x = 0; Pos_y = 0; Pos_z = 0;
			Handle_x = Handle_y = Handle_z = 0;
			ZoneType = 0;
			ZoneVersion = 0;
			GeneratorVersion = 0;
			RingNum = 65535;
#if VOXEL_CULLER
			Culling = 0;
#endif
			Data = new VoxelData();
			Data.Data = new ushort[DataSize];
			Data.TempInfos = new ushort[DataSize];
			Data.OtherInfos = new VoxelExtension[DataSize];
			//for( i = 0; i < DataSize; i++ ) Data.OtherInfos[i] = null;
			for( i = 0; i < DataSize; i++ ) Data.TempInfos[i] = 273 + 20;

			for( int r = 0; r < 6; r++ )
			{
				near_sectors[r] = null;
			}
			Flag_Render_Dirty = true;
			Flag_HighPriorityRefresh = false;
			Flag_Void_Regular = true;
			Flag_Void_Transparent = true;
			Flag_IsVisibleAtLastRendering = false;
			Flag_DeletePending = false;
			Flag_NeedFullCulling = false;
			Flag_KeepInMemory = false;
			Flag_IsModified = ModifiedFieldFlags.NONE;
			Flag_IsSlowGeneration = false;
			Flag_IsActiveVoxels = false;
			Flag_IsActiveLowRefresh = false;
			Flag_IsDebug = false;
			Flag_NotStandardSize = false;
			Flag_NeedSortedRendering = false;
			PartialCulling = 0;
			RefreshWaitCount = 0;
			LowRefresh_Mask = 0x0F;
		}

		void CleanupSector()
		{
			int i;
			VoxelExtension Infos;

			for( i = 0; i < DataSize; i++ )
			{
				if( ( Infos = Data.OtherInfos[i] ) != null )
				{
					Infos.Dispose();
					Data.OtherInfos[i] = null;
				}
			}
		} //

		public void Dispose()
		{
			int i;
			if( VoxelTypeManager != null )
			{
				for( i = 0; i < DataSize; i++ )
				{
					if( Data.OtherInfos[i] != null )
						Data.OtherInfos[i].Dispose();
					Data.OtherInfos[i] = null;
				}
			}
			Data = null;
			// Delete memory zones

			//if (Data)        {delete [] Data;        Data = 0;        }
#if VOXEL_CULLER
			if( Culling ) { delete[] Culling; Culling = 0; }
#endif
			geometry.Dispose();
			//DisplayData = null;
			//if (OtherInfos)  {delete [] OtherInfos;  OtherInfos  = 0; }
			//if (TempInfos)   {delete [] TempInfos;   TempInfos   = 0; }
			world = null;

			SectorsInMemory--;
		}

		~VoxelSector()
		{
		}

		bool GetSectorBaseDirectory( out string OutDirectory )
		{
			if( VoxelGlobalSettings.COMPILEOPTION_USEHOMEDIRSTORAGE )
			{
				OutDirectory = VStreamFile.Get_Directory_UserData() + "/" + VoxelGlobalSettings.COMPILEOPTION_SAVEFOLDERNAME;
			}
			else
				OutDirectory = ".";
			OutDirectory += "/Universes";

			return ( true );
		}


		bool GetSectorFileName( int UniverseNum, int Pos_x, int Pos_y, int Pos_z, string BaseDirectory, out string OutBuffer )
		{
			OutBuffer = String.Format( "{0}/{1}/TL1_{2}_{3}_{4}/TL2_{5}_{6}_{7}/TL3_{8}_{9}_{10}/Sector_{11}_{12}_{13}.vox",
					   BaseDirectory,
					   (uint)UniverseNum,
					   (uint)( Pos_x >> 12 ), (uint)( Pos_y >> 12 ), (uint)( Pos_z >> 12 ),
					   (uint)( Pos_x >> 8 ), (uint)( Pos_y >> 8 ), (uint)( Pos_z >> 8 ),
					   (uint)( Pos_x >> 4 ), (uint)( Pos_y >> 4 ), (uint)( Pos_z >> 4 ),
					   (uint)( Pos_x ), (uint)( Pos_y ), (uint)( Pos_z )
					 );

			return ( true );
		}

		public bool DeleteSave( int UniverseNum ) // Delete file on disk.
		{
			string Directory;
			string FileName;

			GetSectorBaseDirectory( out Directory );
			GetSectorFileName( UniverseNum, Pos_x, Pos_y, Pos_z, Directory, out FileName );
			if( System.IO.File.Exists( FileName ) )
				System.IO.File.Delete( FileName );
			return ( true );
		}

		bool CreateSectorPathSubstructure( string SectorSaveBaseDirectory, int UniverseNum, int Pos_x, int Pos_y, int Pos_z )
		{
			string DirName;
			DirName = String.Format( "{0}/{1}/TL1_{2}_{3}_{4}/TL2_{5}_{6}_{7}/TL3_{8}_{9}_{10}",
					   SectorSaveBaseDirectory,
					   (uint)UniverseNum,
					   (uint)( Pos_x >> 12 ), (uint)( Pos_y >> 12 ), (uint)( Pos_z >> 12 ),
					   (uint)( Pos_x >> 8 ), (uint)( Pos_y >> 8 ), (uint)( Pos_z >> 8 ),
					   (uint)( Pos_x >> 4 ), (uint)( Pos_y >> 4 ), (uint)( Pos_z >> 4 ) );
			System.IO.Directory.CreateDirectory( DirName );
			return ( true );
		}

		internal bool Save( int UniverseNum, string OptFileName = null )
		{
			Log.log( "VoxelSector.Save is incomplete" );
			return true;
#if FINISHED_SAVE
			ZStream_File OutStream;
		ZStream_SpecialRamStream Rs;
		int* Size, StartLen, i;
		string Directory;

		int DataSize;

	// Make sector path and FileName

	if( !OptFileName )
	{
		char FileName[FILENAME_MAX];
        GetSectorBaseDirectory( out Directory );
        GetSectorFileName( UniverseNum, Pos_x, Pos_y, Pos_z, Directory, FileName );
		OutStream.SetFileName( FileName );
	}
	else OutStream.SetFileName( OptFileName );

	// Open filename. If it doesn't work, try creating the right directory substructure.

	if( !OutStream.OpenWrite() )
	{
		if( OptFileName ) return ( false );
        CreateSectorPathSubstructure( Directory, UniverseNum, Pos_x, Pos_y, Pos_z );
		if( !OutStream.OpenWrite() ) return ( false );
	}

DataSize = Size_x* Size_y * Size_z;

Rs.SetStream( &OutStream );

	Rs.OpenWrite();
	Rs.PutString( "BLACKSEC" );

	Rs.Put( (ushort)4 ); // Version
	Rs.Put( (ushort)4 ); // Compatibility Class;

	// Sector Informations

	Rs.PutString( "SECTINFO" );
	Size = Rs.GetPointer_int();
	Rs.Put( 0xA600DBEDu );
	StartLen = Rs.GetActualBufferLen();
	Rs.Put( (ushort)6 ); // Version
	Rs.Put( (ushort)Size_x ); Rs.Put( (ushort)Size_y ); Rs.Put( (ushort)Size_z ); // Voxel Sector Dimensions
	Rs.Put( (short)Handle_x ); Rs.Put( (short)Handle_y ); Rs.Put( (short)Handle_z );
	Rs.Put( (byte)Flag_NeedFullCulling );
	Rs.Put( PartialCulling ); // Partial Culling data
	Rs.Put( (byte)Flag_IsModified );
	Rs.Put( (byte)Flag_IsSlowGeneration );
	Rs.Put( (byte)Flag_IsActiveVoxels );       // V2
	Rs.Put( (byte)Flag_IsActiveLowRefresh );   // V5
	Rs.Put( (int)LowRefresh_Mask );          // V5
	Rs.Put( (ushort)ZoneType );                // V6
	Rs.Put( (ushort)ZoneVersion );             // V6
	Rs.Put( (ushort)GeneratorVersion );        // V6
	Rs.Put( (ushort)RingNum );                  // V6
	Rs.Put( (byte)Flag_NeedSortedRendering ); // V6
	Rs.Put( (byte)Flag_NotStandardSize );     // V6
    * Size = Rs.GetActualBufferLen() - StartLen;
Rs.FlushBuffer();

	// Voxel Data

	Rs.PutString( "VOXELDAT" );
	Size = Rs.GetPointer_int();
	Rs.Put( 0xA600DBEDu );
	StartLen = Rs.GetActualBufferLen();
	Rs.Put( (ushort)1 ); // Version
    Compress_Short_RLE( &this.Data, &Rs );
    * Size = Rs.GetActualBufferLen() - StartLen;
Rs.FlushBuffer();

	// Face Culling Info

	Rs.PutString( "FACECULL" );
	Size = Rs.GetPointer_int();
	Rs.Put( 0xA600DBEDu );
	StartLen = Rs.GetActualBufferLen();
	Rs.Put( (ushort)1 ); // Version
	Culler.Compress_RLE( this, &Rs );
	//  Compress_FaceCulling_RLE(FaceCulling, &Rs);
	*Size = Rs.GetActualBufferLen() - StartLen;
Rs.FlushBuffer();

// Other infos Data

Rs.PutString( "OTHERINF" );
Size = Rs.GetPointer_int();
Rs.Put( 0xA600DBEDu );
StartLen = Rs.GetActualBufferLen();
Rs.Put( (ushort)1 ); // Version
Compress_OtherInfos_RLE( &this.Data, this.Data.Data, &Rs );
	*Size = Rs.GetActualBufferLen() - StartLen;
Rs.FlushBuffer();

// Temperature infos

Rs.PutString( "TEMPDATA" );
Size = Rs.GetPointer_int();
Rs.Put( 0xA600DBEDu );
StartLen = Rs.GetActualBufferLen();
Rs.Put( (ushort)1 ); // Version
Compress_Temperatures_RLE( &this.Data, &Rs );
	*Size = Rs.GetActualBufferLen() - StartLen;
Rs.FlushBuffer();

	// Voxel Extensions

	ushort Voxel;
ZVoxelType* VoxelType;
	int* ExtensionCount;
ZVoxelExtension* VoxelExtension;

Rs.PutString( "VOXELEXT" );
Size = Rs.GetPointer_int();
Rs.Put( 0xA600DBEDu );
StartLen = Rs.GetActualBufferLen();
Rs.Put( (ushort)1 );
ExtensionCount = Rs.GetPointer_int();
Rs.Put( 0u );

	for( i = 0; i < DataSize; i++ )
    {
	Voxel = Data.Data[i];
	VoxelType = VoxelTypeManager.VoxelTable[Voxel];
	if( VoxelType.Is_HasAllocatedMemoryExtension )
	{
		( *ExtensionCount )++;
		Rs.Put( i ); // Voxel Offset;
		VoxelExtension = (ZVoxelExtension*)Data.OtherInfos[i];
		Rs.Put( VoxelExtension.GetExtensionID() );

		VoxelExtension.Save( &Rs );
	}
}
	*Size = Rs.GetActualBufferLen() - StartLen;
Rs.FlushBuffer();






//if (DataSize != fwrite(Data, sizeof(ushort),DataSize,fh)) {fclose (fh);return(false);}
//if (DataSize != fwrite(Data, sizeof(byte) ,DataSize,fh)) {fclose (fh);return(false);}
//fclose (fh);

OutStream.Close();

	return ( true );
#endif
		}

		/*
		Bool Save(char const * FileName)
		{
		  FILE * fh;
		  int DataSize;

		  fh = fopen(FileName, "wb");

		  DataSize = ZVOXELBLOCSIZE_X * ZVOXELBLOCSIZE_Y * ZVOXELBLOCSIZE_Z;

		  if (DataSize != fwrite(Data, sizeof(ushort),DataSize,fh)) {fclose (fh);return(false);}
		  if (DataSize != fwrite(Data, sizeof(byte) ,DataSize,fh)) {fclose (fh);return(false);}
		  fclose (fh);

		  return(true);
		}
		*/


		internal bool Load( int UniverseNum, string OptFileName = null )
		{
			Log.log( "*** Sector load incomplete " );
			return true;
#if asdfsadf
			ZTestMemoryPool MemPool;
			ZStream_File InStream;
			ZStream_SpecialRamStream Rs;
			string String;
			//String.SetMemPool(&MemPool);
			string SectionName;
			//SectionName.SetMemPool(&MemPool);
			//InStream.FileName.SetMemPool(&MemPool);

			bool Ok;
			string Directory;
			int i, j;

			//for(i=0;i<DataSize;i++) Data[i]=0;

			if( OptFileName )
			{
				InStream.SetFileName( OptFileName );
			}
			else
			{
				char FileName[FILENAME_MAX];

				GetSectorBaseDirectory( Directory );
				GetSectorFileName( UniverseNum, Pos_x, Pos_y, Pos_z, Directory, FileName );
				InStream.SetFileName( FileName );
			}

			if( !InStream.OpenRead() ) return ( false );

			Rs.SetStream( &InStream );
			Rs.OpenRead();

			//
			// Sector Header
			ushort Version;
			ushort Compatibility_Class;

			String.SetLen( 8 );
			Rs.GetStringFixedLen( String.String, 8 );
			if( String != "BLACKSEC" ) { printf( "Sector Loading Error (%ld,%ld,%ld): Header Missing, File is not a Sector regular format.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }
			Ok = Rs.Get( Version );
			Ok &= Rs.Get( Compatibility_Class ); if( !Ok ) { printf( "Sector Loading Error (%ld,%ld,%ld): Can't read header informations.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }

			if( Compatibility_Class > 4 ) { printf( "Sector Loading Error (%ld,%ld,%ld): Incompatible format version.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }

			// Sector Informations


			ushort Section_Version;
			int Section_Len;
			ushort Size_x = 0, Size_y = 0, Size_z = 0;
			byte Temp_Byte;
			//bool    SectInfoReaded;

			Section_Version = 0;
			Temp_Byte = 0;
			//SectInfoReaded = false;
			SectionName.SetLen( 8 );
			while( Rs.GetRemainBytesToRead() != 0 )
			{
				if( !Rs.GetStringFixedLen( SectionName.String, 8 ) ) { printf( "Sector Loading Error (%ld,%ld,%ld): Unable to read section name.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }

				if( SectionName == "SECTINFO" )
				{
					Ok = Rs.Get( Section_Len );
					Ok &= Rs.Get( Section_Version );
					if( Section_Version >= 1 )
					{
						Ok &= Rs.Get( Size_x );
						Ok &= Rs.Get( Size_y );
						Ok &= Rs.Get( Size_z );
						if( ( Size_x != this.Size_x ) || ( Size_y != this.Size_y ) || ( Size_z != this.Size_z ) )
							ChangeSize( Size_x, Size_y, Size_z );
						if( Section_Version >= 4 )
						{
							Ok &= Rs.Get( Handle_x );
							Ok &= Rs.Get( Handle_y );
							Ok &= Rs.Get( Handle_z );
						}
						Ok &= Rs.Get( Temp_Byte ); Flag_NeedFullCulling = ( Temp_Byte ) ? true : false;
						Ok &= Rs.Get( Temp_Byte ); PartialCulling = Temp_Byte;
					}
					if( Section_Version >= 2 )
					{
						if( Section_Version >= 6 ) { Ok &= Rs.Get( Flag_IsModified ); Flag_IsModified &= ( 0xFF ^ BITSECTORMODIFIED ); }
						else { Ok &= Rs.Get( Temp_Byte ); Flag_IsModified = ( Temp_Byte ) ? IMPORTANT : NONE; }
						Ok &= Rs.Get( Temp_Byte ); Flag_IsSlowGeneration = ( Temp_Byte ) ? true : false;
					}
					if( Section_Version >= 3 )
					{
						Ok &= Rs.Get( Temp_Byte ); Flag_IsActiveVoxels = ( Temp_Byte ) ? true : false;
					}

					if( Section_Version >= 5 )
					{
						Ok &= Rs.Get( Temp_Byte ); Flag_IsActiveLowRefresh = ( Temp_Byte ) ? true : false;
						Ok &= Rs.Get( LowRefresh_Mask );
					}

					if( Section_Version >= 6 )
					{
						Ok &= Rs.Get( ZoneType );
						Ok &= Rs.Get( ZoneVersion );
						Ok &= Rs.Get( GeneratorVersion );
						Ok &= Rs.Get( RingNum );
						Ok &= Rs.Get( Temp_Byte ); Flag_NeedSortedRendering = ( Temp_Byte ) ? true : false;
						Ok &= Rs.Get( Temp_Byte ); Flag_NotStandardSize = ( Temp_Byte ) ? true : false;
					}
					//
					if( !Ok ) { printf( "Sector Loading Error (%ld,%ld,%ld): Can't read SECTOR INFO section informations.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }

					if( !Flag_NotStandardSize ) if( ( Size_x != ZVOXELBLOCSIZE_X ) || ( Size_y != ZVOXELBLOCSIZE_Y ) || ( Size_z != ZVOXELBLOCSIZE_Z ) ) { printf( "Sector Loading Error (%ld,%ld,%ld): Incompatible sector dimension.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }
					if( Section_Version > 6 ) { printf( "Sector Loading Error (%ld,%ld,%ld): Incompatible format in SECTOR INFO section.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }
				}
				else if( SectionName == "VOXELDAT" )
				{
					Ok = Rs.Get( Section_Len );
					Ok &= Rs.Get( Section_Version );
					if( !Ok ) { printf( "Sector Loading Error (%ld,%ld,%ld): Can't read VOXEL DATA section informations.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }
					if( !Decompress_Short_RLE( &Data, &Rs ) ) { printf( "Sector Loading Error (%ld,%ld,%ld): Can't read and decompress VOXEL DATA section data.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }
				}
				else if( SectionName == "FACECULL" )
				{
					Ok = Rs.Get( Section_Len );
					Ok &= Rs.Get( Section_Version );
					if( !Ok ) { printf( "Sector Loading Error (%ld,%ld,%ld): Can't read VOXEL DATA section informations.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }
					if( !Culler.Decompress_RLE( this, &Rs ) ) { printf( "Sector Loading Error (%ld,%ld,%ld): Can't read and decompress FACE CULLING section data.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }
				}
				else if( SectionName == "OTHERINF" )
				{
					Ok = Rs.Get( Section_Len );
					Ok &= Rs.Get( Section_Version );
					if( !Ok ) { printf( "Sector Loading Error (%ld,%ld,%ld): Can't read VOXEL DATA section informations.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }
					if( !Decompress_OtherInfos_RLE( &Data, &Rs ) ) { printf( "Sector Loading Error (%ld,%ld,%ld): Can't read and decompress FACE CULLING section data.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }
				}
				else if( SectionName == "TEMPDATA" )
				{
					Ok = Rs.Get( Section_Len );
					Ok &= Rs.Get( Section_Version );
					if( !Ok ) { printf( "Sector Loading Error (%ld,%ld,%ld): Can't read VOXEL DATA section informations.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }
					if( !Decompress_Temperatures_RLE( &Data, &Rs ) ) { printf( "Sector Loading Error (%ld,%ld,%ld): Can't read and decompress FACE CULLING section data.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }
				}
				else if( SectionName == "VOXELEXT" )
				{
					// Voxel Extensions

					ushort Voxel;
					ZVoxelType* VoxelType;
					ZVoxelExtension* VoxelExtension;
					int RemainingExtensionsInFile;
					int VoxelOffset, ExtensionID, ExtensionLen;
					bool PassExtension;
					RemainingExtensionsInFile = 0;

					Ok = Rs.Get( Section_Len );
					Ok &= Rs.Get( Section_Version );
					Ok &= Rs.Get( RemainingExtensionsInFile );


					if( !Ok ) { printf( "Sector Loading Error (%ld,%ld,%ld): Can't read VOXEL EXTENSION section informations.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }

					for( i = 0; i < DataSize; i++ )
					{
						Voxel = Data.Data[i];
						VoxelType = VoxelTypeManager.VoxelTable[Voxel];
						if( VoxelType.Is_HasAllocatedMemoryExtension )
						{
							VoxelExtension = (ZVoxelExtension*)VoxelType.CreateVoxelExtension( true );
							if( !VoxelExtension ) { printf( "Sector Loading Error (%ld,%ld,%ld): Can't read VOXEL EXTENSION section / Can't create voxel extension.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }

							do
							{
								PassExtension = false;
								Ok = Rs.Get( VoxelOffset ); if( !Ok ) { printf( "Sector Loading Error (%ld,%ld,%ld): Can't read VOXEL EXTENSION section / VoxelOffset.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }
								if( !RemainingExtensionsInFile ) { printf( "Sector Loading Error (%ld,%ld,%ld): Can't read VOXEL EXTENSION section / Missing voxel extensions in file.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }
								Ok = Rs.Get( ExtensionID ); if( !Ok ) { printf( "Sector Loading Error (%ld,%ld,%ld): Can't read VOXEL EXTENSION section / ExtensionID.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }

								// If there is unsupported extensions in file, read them and throw there contents out.
								if( VoxelOffset != i )
								{
									PassExtension = true; // After throwing extension, we must try next.
									Ok = Rs.Get( ExtensionLen ); if( !Ok ) { printf( "Sector Loading Error (%ld,%ld,%ld): Can't read VOXEL EXTENSION section / ExtensionLen.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }
									for( j = 0; j < ExtensionLen; j++ ) Ok &= Rs.Get( Temp_Byte );
									if( !Ok ) { printf( "Sector Loading Error (%ld,%ld,%ld): Can't read VOXEL EXTENSION section / Thrown extension data.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }

								}
								else if( ExtensionID != VoxelExtension.GetExtensionID() ) { printf( "Sector Loading Error (%ld,%ld,%ld): Can't read VOXEL EXTENSION section / Extension type doesn't match.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }
								RemainingExtensionsInFile--;
							} while( PassExtension );

							Data.OtherInfos[i] = (uint)VoxelExtension;
							if( !VoxelExtension.Load( &Rs ) ) { printf( "Sector Loading Error (%ld,%ld,%ld): Can't read VOXEL EXTENSION section / Can't read voxel extension subdata.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }
						}
					}
					// End of Section
				}
				else // Unknow Section
				{
					Ok = Rs.Get( Section_Len );
					if( !Ok ) { printf( "Sector Loading Error (%ld,%ld,%ld): Can't read UNKNOWN section informations.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }
					for( i = 0; i < Section_Len; i++ ) if( !Rs.Get( Temp_Byte ) ) { printf( "Sector Loading Error (%ld,%ld,%ld): Can't read UNKNOWN section data.\n", (uint)Pos_x, (uint)Pos_y, (uint)Pos_z ); Rs.Close(); InStream.Close(); return ( false ); }
				}
			}

			// Voxel Data

			/*

			Rs.PutString("VOXELDAT");
			Size = Rs.GetPointer_int();
			Rs.Put(0xA600DBEDUL);
			StartLen = Rs.GetActualBufferLen();
			Rs.Put((ushort)1); // Version
			Compress_Short_RLE(Data, &Rs);
			*Size = Rs.GetActualBufferLen() - StartLen;
			Rs.FlushBuffer();

			// Face Culling Info

			Rs.PutString("FACECULL");
			Size = Rs.GetPointer_int();
			Rs.Put(0xA600DBEDUL);
			StartLen = Rs.GetActualBufferLen();
			Rs.Put((ushort)1); // Version
			Compress_FaceCulling_RLE(FaceCulling, &Rs);
			*Size = Rs.GetActualBufferLen() - StartLen;
			Rs.FlushBuffer();

			// Other infos Data

			Rs.PutString("OTHERINF");
			Size = Rs.GetPointer_int();
			Rs.Put(0xA600DBEDUL);
			StartLen = Rs.GetActualBufferLen();
			Rs.Put((ushort)1); // Version
			Compress_OtherInfos_RLE(OtherInfos, &Rs);
			*Size = Rs.GetActualBufferLen() - StartLen;
			Rs.FlushBuffer();

			// Temperature infos

			Rs.PutString("TEMPDATA");
			Size = Rs.GetPointer_int();
			Rs.Put(0xA600DBEDUL);
			StartLen = Rs.GetActualBufferLen();
			Rs.Put((ushort)1); // Version
			Compress_Temperatures_RLE(TempInfos, &Rs);
			*Size = Rs.GetActualBufferLen() - StartLen;
			Rs.FlushBuffer();

			}

			  //if (DataSize != fwrite(Data, sizeof(ushort),DataSize,fh)) {fclose (fh);return(false);}
			  //if (DataSize != fwrite(Data, sizeof(byte) ,DataSize,fh)) {fclose (fh);return(false);}
			  //fclose (fh);
		  */
			InStream.Close();

			return ( true );
#endif

		}

		/*
		Bool Load(char const * FileName)
		{
		  FILE * fh;
		  int DataSize;

		  fh = fopen(FileName, "rb");

		  if (!fh) return(false);

		  DataSize = ZVOXELBLOCSIZE_X * ZVOXELBLOCSIZE_Y * ZVOXELBLOCSIZE_Z;

		  if (DataSize != fread(Data, sizeof(ushort),DataSize,fh)) {fclose (fh);return(false);}
		  if (DataSize != fread(Data, sizeof(byte) ,DataSize,fh)) {fclose (fh);return(false);}
		  fclose (fh);

		  return(true);
		}
		*/
#if OUTPUT_ENABLED
		void DebugOutFCInfo( string FileName )
		{
			int x, y, z;
			byte Voxel;
			int Car;
			FILE* fp;

			fp = fopen( FileName, "wb" );
			if( !fp ) return;

			for( y = 0; y < Size_y; y++ )
			{
				for( z = 0; z < Size_x; z++ )
				{
					for( x = 0; x < Size_z; x++ )
					{
						Voxel = this.Culler.getFaceCulling( this, y + ( x * Size_y ) + ( z * ( Size_y * Size_x ) ) );
						Car = Voxel + 'A';
						fputc( Car, fp );
					}
					fputc( 0x0d, fp );
					fputc( 0x0a, fp );
				}
				fputc( 0x0d, fp );
				fputc( 0x0a, fp );
				fputc( 0x0d, fp );
				fputc( 0x0a, fp );
			}

			fclose( fp );
		}

		void Compress_Short_RLE( VoxelData* Data, void* Stream )
		{
			ZStream_SpecialRamStream* Rs = (ZStream_SpecialRamStream*)Stream;
			ushort MagicToken = 65535;
			ushort Last, Actual;
			int Point = 0;
			int SameCount = 0;
			int i;
			bool Continue;

			Last = Data.Data[Point++];
			Continue = true;
			while( Continue )
			{
				if( Point != DataSize ) Actual = Data.Data[Point++];
				else { Actual = Last - 1; Continue = false; }
				if( Last == Actual )
				{
					SameCount++;
				}
				else
				{
					if( SameCount )
					{
						if( SameCount < 3 )
						{
							if( Last == MagicToken ) { Rs.Put( MagicToken ); Rs.Put( MagicToken ); Rs.Put( (ushort)( SameCount + 1 ) ); }
							else { for( i = 0; i <= SameCount; i++ ) Rs.Put( Last ); }
						}
						else
						{
							Rs.Put( MagicToken );
							Rs.Put( Last );
							Rs.Put( (ushort)( SameCount + 1 ) );
						}
						SameCount = 0;
					}
					else
					{
						if( Last == MagicToken ) { Rs.Put( MagicToken ); Rs.Put( Last ); Rs.Put( (ushort)1 ); }
						else { Rs.Put( Last ); }
					}
				}
				Last = Actual;
			}

		}

		void Compress_OtherInfos_RLE( VoxelData* Data, ushort* VoxelData, void* Stream )
		{
			ZStream_SpecialRamStream* Rs = (ZStream_SpecialRamStream*)Stream;
			int MagicToken = 0xA600DBED;
			int Last, Actual;
			int Point = 0;
			int SameCount = 0;
			int i;
			bool Continue;

			Last = Data.OtherInfos[Point];
			if( VoxelTypeManager.VoxelTable[VoxelData[Point++]].Is_HasAllocatedMemoryExtension ) Last = 0;

			Continue = true;
			while( Continue )
			{
				if( Point != DataSize )
				{
					Actual = this.Data.OtherInfos[Point];
					if( VoxelTypeManager.VoxelTable[VoxelData[Point++]].Is_HasAllocatedMemoryExtension ) Actual = 0;
				}
				else { Actual = Last - 1; Continue = false; }
				if( Last == Actual )
				{
					SameCount++;
				}
				else
				{
					if( SameCount )
					{
						if( SameCount < 3 )
						{
							if( Last == MagicToken ) { Rs.Put( MagicToken ); Rs.Put( MagicToken ); Rs.Put( (ushort)( SameCount + 1 ) ); }
							else { for( i = 0; i <= SameCount; i++ ) Rs.Put( Last ); }
						}
						else
						{
							Rs.Put( MagicToken );
							Rs.Put( Last );
							Rs.Put( (ushort)( SameCount + 1 ) );
						}
						SameCount = 0;
					}
					else
					{
						if( Last == MagicToken ) { Rs.Put( MagicToken ); Rs.Put( Last ); Rs.Put( (ushort)1 ); }
						else { Rs.Put( Last ); }
					}
				}
				Last = Actual;
			}
		}
		/*
		void Compress_FaceCulling_RLE(byte * Data, void  * Stream)
		{
		  ZStream_SpecialRamStream * Rs = (ZStream_SpecialRamStream *)Stream;
		  byte MagicToken = 0xFF;
		  int Last, Actual;
		  int Point = 0;
		  int SameCount = 0;
		  int i;
		  bool Continue;

		  Last = OtherInfos[Point++];

		  Continue = true;
		  while (Continue)
		  {
			if (Point != DataSize) {Actual = OtherInfos[Point++]; }
			else                   {Actual = Last - 1; Continue = false; }
			if (Last == Actual)
			{
			  SameCount ++;
			}
			else
			{
			  if (SameCount)
			  {
				if (SameCount < 3)
				{
				  if   (Last == MagicToken) { Rs.Put(MagicToken); Rs.Put(MagicToken); Rs.Put((ushort)(SameCount+1)); }
				  else                 { for (i=0;i<=SameCount;i++) Rs.Put(Last); }
				}
				else
				{
				  Rs.Put(MagicToken);
				  Rs.Put(Last);
				  Rs.Put((ushort)(SameCount+1));
				}
				SameCount = 0;
			  }
			  else
			  {
				if (Last == MagicToken) {Rs.Put(MagicToken); Rs.Put(Last); Rs.Put((ushort)1); }
				else               {Rs.Put(Last);}
			  }
			}
			Last = Actual;
		  }
		}
		*/

		void Compress_Temperatures_RLE( VoxelData* Data, void* Stream )
		{
			ZStream_SpecialRamStream* Rs = (ZStream_SpecialRamStream*)Stream;
			ushort MagicToken = 55871;
			ushort Last, Actual;
			int Point = 0;
			int SameCount = 0;
			int i;
			bool Continue;

			Last = Data.TempInfos[Point++];
			Continue = true;
			while( Continue )
			{
				if( Point != DataSize ) Actual = Data.TempInfos[Point++];
				else { Actual = Last - 1; Continue = false; }
				if( Last == Actual )
				{
					SameCount++;
				}
				else
				{
					if( SameCount )
					{
						if( SameCount < 3 )
						{
							if( Last == MagicToken ) { Rs.Put( MagicToken ); Rs.Put( MagicToken ); Rs.Put( (ushort)( SameCount + 1 ) ); }
							else { for( i = 0; i <= SameCount; i++ ) Rs.Put( Last ); }
						}
						else
						{
							Rs.Put( MagicToken );
							Rs.Put( Last );
							Rs.Put( (ushort)( SameCount + 1 ) );
						}
						SameCount = 0;
					}
					else
					{
						if( Last == MagicToken ) { Rs.Put( MagicToken ); Rs.Put( Last ); Rs.Put( (ushort)1 ); }
						else { Rs.Put( Last ); }
					}
				}
				Last = Actual;
			}

		}

		bool Decompress_Short_RLE( VoxelData* Data, void* Stream )
		{
			ZStream_SpecialRamStream* Rs = (ZStream_SpecialRamStream*)Stream;
			ushort MagicToken = 0xFFFF;
			ushort Actual;
			int Pointer;
			ushort nRepeat;

			Pointer = 0;
			while( Pointer < DataSize )
			{
				if( !Rs.Get( Actual ) ) return ( false );
				if( Actual == MagicToken )
				{
					if( !Rs.Get( Actual ) ) return ( false );
					if( !Rs.Get( nRepeat ) ) return ( false );
					if( ( (int)nRepeat ) > ( DataSize - Pointer ) )
					{
						return ( false );
					}

					while( nRepeat-- ) { Data.Data[Pointer++] = Actual; }
				}
				else
				{
					Data.Data[Pointer++] = Actual;
				}
			}

			return ( true );
		}

#if false
bool Decompress_FaceCulling_RLE(byte * Data, void * Stream)
{
  ZStream_SpecialRamStream * Rs = (ZStream_SpecialRamStream *)Stream;
  byte MagicToken = 0xFF;
  int Actual;
  int Pointer;
  ushort nRepeat;

  Pointer = 0;
  while (Pointer<DataSize)
  {
    if (!Rs.Get(Actual)) return(false);
    if (Actual == MagicToken)
    {
      if (!Rs.Get(Actual))  return(false);
      if (!Rs.Get(nRepeat)) return(false);
      if ( ((int)nRepeat) > (DataSize - Pointer))
      {
        return(false);
      }

      while (nRepeat--) {Data[Pointer++] = Actual;}
    }
    else
    {
      Data[Pointer++] = Actual;
    }
  }

  return(true);
}
#endif

		bool Decompress_OtherInfos_RLE( VoxelData* Data, void* Stream )
		{
			ZStream_SpecialRamStream* Rs = (ZStream_SpecialRamStream*)Stream;
			int MagicToken = 0xA600DBED; ;
			int Actual;
			int Pointer;
			ushort nRepeat;

			Pointer = 0;
			while( Pointer < DataSize )
			{
				if( !Rs.Get( Actual ) ) return ( false );
				if( Actual == MagicToken )
				{
					if( !Rs.Get( Actual ) ) return ( false );
					if( !Rs.Get( nRepeat ) ) return ( false );
					if( ( (int)nRepeat ) > ( DataSize - Pointer ) )
					{
						return ( false );
					}

					while( nRepeat-- ) { Data.OtherInfos[Pointer++] = Actual; }
				}
				else
				{
					Data.OtherInfos[Pointer++] = Actual;
				}
			}

			return ( true );
		}

		bool Decompress_Temperatures_RLE( VoxelData* Data, void* Stream )
		{
			ZStream_SpecialRamStream* Rs = (ZStream_SpecialRamStream*)Stream;
			ushort MagicToken = 55871;
			ushort Actual;
			int Pointer;
			ushort nRepeat;

			Pointer = 0;
			while( Pointer < DataSize )
			{
				if( !Rs.Get( Actual ) ) return ( false );
				if( Actual == MagicToken )
				{
					if( !Rs.Get( Actual ) ) return ( false );
					if( !Rs.Get( nRepeat ) ) return ( false );
					if( ( (int)nRepeat ) > ( DataSize - Pointer ) )
					{
						return ( false );
					}

					while( nRepeat-- ) { Data.TempInfos[Pointer++] = Actual; }
				}
				else
				{
					Data.TempInfos[Pointer++] = Actual;
				}
			}

			return ( true );
		}
#endif

		public void Draw_safe_Sphere( double x, double y, double z, double Radius, ushort VoxelType, bool DrawIfVoid = true )
		{
			uint sx, sy, sz;
			uint ex, ey, ez;
			uint nx, ny, nz;
			double r;
			double dx, dy, dz;

			if( Radius < 0 ) Radius = -Radius;

			sx = (uint)Math.Floor( x - Radius ); ex = (uint)Math.Ceiling( x + Radius );
			sy = (uint)Math.Floor( y - Radius ); ey = (uint)Math.Ceiling( y + Radius );
			sz = (uint)Math.Floor( z - Radius ); ez = (uint)Math.Ceiling( z + Radius );

			if( sx < 0 ) sx = 0; if( sx >= Size_x ) sx = Size_x - 1;
			if( sy < 0 ) sy = 0; if( sy >= Size_y ) sy = Size_y - 1;
			if( sz < 0 ) sz = 0; if( sz >= Size_z ) sz = Size_z - 1;

			if( ex < 0 ) ex = 0; if( ex >= Size_x ) ex = Size_x - 1;
			if( ey < 0 ) ey = 0; if( ey >= Size_y ) ey = Size_y - 1;
			if( ez < 0 ) ez = 0; if( ez >= Size_z ) ez = Size_z - 1;

			for( nz = sz; nz <= ez; nz++ )
				for( nx = sx; nx <= ex; nx++ )
					for( ny = sy; ny <= ey; ny++ )
					{
						dx = x - (double)nx;
						dy = y - (double)ny;
						dz = z - (double)nz;
						r = Math.Sqrt( dx * dx + dy * dy + dz * dz );
						if( r <= Radius ) Draw_safe_SetVoxel( nx, ny, nz, VoxelType, DrawIfVoid );
					}

		}

		public void Draw_safe_SetVoxel( uint x, uint y, uint z, ushort VoxelType, bool DrawIfVoid = true )
		{
			uint Pointer;

			ushort OldVoxel;

			if( x >= Size_x || y >= Size_y || z >= Size_z || x < 0 || y < 0 || z < 0 ) return;

			Pointer = y + x * Size_y + z * ( Size_y * Size_x );

			// Si le Voxel StockÃ© dispose d'une extension, la libÃ©rer.

			OldVoxel = Data.Data[Pointer];

			if( OldVoxel != 0 && !DrawIfVoid ) return;
			if( VoxelTypeManager.VoxelTable[OldVoxel].properties.Is_HasAllocatedMemoryExtension )
			{
				VoxelTypeManager.VoxelTable[Data.Data[Pointer]].DeleteVoxelExtension( Data.OtherInfos[Pointer] );
				Data.OtherInfos[Pointer] = null;
			}

			// Stocke le voxel

			Data.Data[Pointer] = VoxelType;

			// Si le voxel a stocker comporte une partie extension, la crÃ©er et l'enregistrer.

			if( VoxelTypeManager.VoxelTable[VoxelType].properties.Is_HasAllocatedMemoryExtension )
			{
				Data.OtherInfos[Pointer] = VoxelTypeManager.VoxelTable[VoxelType].CreateVoxelExtension();
			}
		}

		public void Purge( ushort VoxelType )
		{
			int i;
			ushort Voxel;

			for( i = 0; i < ( ZVOXELBLOCSIZE_X * ZVOXELBLOCSIZE_Y * ZVOXELBLOCSIZE_Z ); i++ )
			{
				Voxel = this.Data.Data[i];
				if( Voxel == VoxelType )
				{
					if( VoxelTypeManager.VoxelTable[Voxel].properties.Is_HasAllocatedMemoryExtension )
					{
						VoxelTypeManager.VoxelTable[Voxel].DeleteVoxelExtension( Data.OtherInfos[i] );
					}
					Data.Data[i] = 0;
					Data.OtherInfos[i] = null;
					Data.TempInfos[i] = 0;
				}
			}
		}

		public void Draw_safe_VoxelLine2( ref ZRect3L_2 LineCoords, ref ZRect1d Thickness, ushort VoxelType )
		{
			int Dx, Dy, Dz, TempMax, NumSteps, i;

			double x, y, z, Thick, Stepx, Stepy, Stepz, StepThick;

			Dx = LineCoords.ex - LineCoords.sx;
			Dy = LineCoords.ey - LineCoords.sy;
			Dz = LineCoords.ez - LineCoords.sz;

			TempMax = ( Math.Abs( Dx ) > Math.Abs( Dy ) ) ? Math.Abs( Dx ) : Math.Abs( Dy );
			NumSteps = ( TempMax > Math.Abs( Dz ) ) ? TempMax : Math.Abs( Dz );
			if( NumSteps <= 0 ) Draw_safe_SetVoxel( (uint)LineCoords.sx, (uint)LineCoords.sy, (uint)LineCoords.sz, VoxelType );

			x = LineCoords.sx; y = LineCoords.sy; z = LineCoords.sz; Thick = Thickness.Start;
			Stepx = ( (double)Dx ) / NumSteps; Stepy = ( (double)Dy ) / NumSteps; Stepz = ( (double)Dz ) / NumSteps; StepThick = ( Thickness.End - Thickness.Start ) / NumSteps;

			for( i = 0; i <= NumSteps; i++ )
			{
				if( Thick == 0.5 ) Draw_safe_SetVoxel( (uint)Math.Floor( x + 0.5 ), (uint)Math.Floor( y + 0.5 ), (uint)Math.Floor( z + 0.5 ), VoxelType );
				else Draw_safe_Sphere( x + 0.5, y + 0.5, z + 0.5, Thick, VoxelType );

				x += Stepx;
				y += Stepy;
				z += Stepz;
				Thick += StepThick;
			}
		}

		public void Draw_safe_VoxelLine( ref ZRect3L LineCoords, ref ZRect1f Thickness, ushort VoxelType )
		{
			int Dx, Dy, Dz, TempMax, NumSteps, i;

			double x, y, z, Thick, Stepx, Stepy, Stepz, StepThick;

			// Do not draw if line is entirely outside the sector.

			int MaxThickness;
			int ThickSize_x, ThickSize_y, ThickSize_z;
			MaxThickness = (int)Thickness.End;
			if( Thickness.Start > MaxThickness ) MaxThickness = (int)Thickness.Start;
			MaxThickness += MaxThickness; // Convert Ray to diameter.
			ThickSize_x = (int)Size_x + MaxThickness;
			ThickSize_y = (int)Size_y + MaxThickness;
			ThickSize_z = (int)Size_z + MaxThickness;

			MaxThickness = -MaxThickness; // We needs negative value for next tests;

			if( LineCoords.Start.x < MaxThickness && LineCoords.End.x < MaxThickness ) return;
			if( LineCoords.Start.x >= ThickSize_x && LineCoords.End.x >= ThickSize_x ) return;
			if( LineCoords.Start.y < MaxThickness && LineCoords.End.y < MaxThickness ) return;
			if( LineCoords.Start.y >= ThickSize_y && LineCoords.End.y >= ThickSize_y ) return;
			if( LineCoords.Start.z < MaxThickness && LineCoords.End.z < MaxThickness ) return;
			if( LineCoords.Start.z >= ThickSize_z && LineCoords.End.z >= ThickSize_z ) return;

			//

			Dx = LineCoords.End.x - LineCoords.Start.x;
			Dy = LineCoords.End.y - LineCoords.Start.y;
			Dz = LineCoords.End.z - LineCoords.Start.z;

			TempMax = ( Math.Abs( Dx ) > Math.Abs( Dy ) ) ? Math.Abs( Dx ) : Math.Abs( Dy );
			NumSteps = ( TempMax > Math.Abs( Dz ) ) ? TempMax : Math.Abs( Dz );
			if( NumSteps <= 0 ) Draw_safe_SetVoxel( (uint)LineCoords.Start.x, (uint)LineCoords.Start.y, (uint)LineCoords.Start.z, VoxelType );

			x = LineCoords.Start.x; y = LineCoords.Start.y; z = LineCoords.Start.z; Thick = Thickness.Start;
			Stepx = ( (double)Dx ) / NumSteps; Stepy = ( (double)Dy ) / NumSteps; Stepz = ( (double)Dz ) / NumSteps; StepThick = ( Thickness.End - Thickness.Start ) / NumSteps;

			for( i = 0; i <= NumSteps; i++ )
			{
				if( Thick == 0.5 ) Draw_safe_SetVoxel( (uint)Math.Floor( x + 0.5 ), (uint)Math.Floor( y + 0.5 ), (uint)Math.Floor( z + 0.5 ), VoxelType );
				else Draw_safe_Sphere( x + 0.5, y + 0.5, z + 0.5, Thick, VoxelType );

				x += Stepx;
				y += Stepy;
				z += Stepz;
				Thick += StepThick;
			}
		}

		public void Draw_safe_VoxelLine_TickCtl( ref ZRect3L LineCoords, double[] ThicknessTable, int nThickIndices, ushort VoxelType )
		{
			int Index1, Index2;
			int Dx, Dy, Dz, TempMax, NumSteps, i;


			double x, y, z, ThickIndex, Thick, Thick1, Thick2, Stepx, Stepy, Stepz, StepThick, Temp, Modulator;

			if( nThickIndices < 2 || ( ThicknessTable == null ) ) return;

			Dx = LineCoords.End.x - LineCoords.Start.x;
			Dy = LineCoords.End.y - LineCoords.Start.y;
			Dz = LineCoords.End.z - LineCoords.Start.z;

			TempMax = ( Math.Abs( Dx ) > Math.Abs( Dy ) ) ? Math.Abs( Dx ) : Math.Abs( Dy );
			NumSteps = ( TempMax > Math.Abs( Dz ) ) ? TempMax : Math.Abs( Dz );
			if( NumSteps <= 0 ) Draw_safe_SetVoxel( (uint)LineCoords.Start.x, (uint)LineCoords.Start.y, (uint)LineCoords.Start.z, VoxelType );

			x = LineCoords.Start.x; y = LineCoords.Start.y; z = LineCoords.Start.z;
			Stepx = ( (double)Dx ) / NumSteps; Stepy = ( (double)Dy ) / NumSteps; Stepz = ( (double)Dz ) / NumSteps;
			StepThick = ( (double)( nThickIndices - 1 ) ) / ( NumSteps ); ThickIndex = 0;

			for( i = 0; i <= NumSteps; i++ )
			{
				Modulator = ThickIndex - ( Temp = Math.Floor( ThickIndex ) );
				Index1 = (int)Math.Floor( ThickIndex );
				Index2 = (int)Math.Ceiling( ThickIndex );
				if( Index2 >= nThickIndices ) Index2 = nThickIndices - 1;
				Thick1 = ThicknessTable[Index1];
				Thick2 = ThicknessTable[Index2];

				Thick = ( Thick1 * ( 1.0 - Modulator ) ) + ( Thick2 * Modulator );

				if( Thick <= 1.0 ) Draw_safe_SetVoxel( (uint)( x + 0.5 ), (uint)( y + 0.5 ), (uint)( z + 0.5 ), VoxelType );
				else Draw_safe_Sphere( x + 0.5, y + 0.5, z + 0.5, Thick, VoxelType );

				x += Stepx;
				y += Stepy;
				z += Stepz;
				ThickIndex += StepThick;
			}
		}

		public void Draw_safe_3DBlit( VoxelSector SourceSector, ref ZVector3L DestinationPoint, ref ZVector3L SourcePoint, ref ZVector3L Size )
		{
			ZVector3L Dp;
			ZVector3L Sp;
			ZVector3L Sz;

			Dp.x = DestinationPoint.x; Dp.y = DestinationPoint.y; Dp.z = DestinationPoint.z;
			Sp.x = SourcePoint.x; Sp.y = SourcePoint.y; Sp.z = SourcePoint.z;
			Sz.x = Size.x; Sz.y = Size.y; Sz.z = Size.z;

			// Clipping du destination point englobant le volume.

			if( Dp.x < 0 ) { Sp.x -= Dp.x; Sz.x -= Dp.x; Dp.x = 0; }
			if( Dp.y < 0 ) { Sp.y -= Dp.y; Sz.y -= Dp.y; Dp.y = 0; }
			if( Dp.z < 0 ) { Sp.z -= Dp.z; Sz.z -= Dp.z; Dp.z = 0; }

			if( Dp.x > this.Size_x || Dp.y > this.Size_y || Dp.z > this.Size_z ) return;
			if( Sp.x > SourceSector.Size_x || Sp.y > SourceSector.Size_y || Sp.z > SourceSector.Size_z ) return;
			if( Sp.x < 0 || Sp.y < 0 || Sp.z < 0 ) return;

			if( ( Sp.x + Sz.x ) > SourceSector.Size_x ) Sz.x -= ( Sp.x + Sz.x - (int)SourceSector.Size_x );
			if( ( Sp.y + Sz.y ) > SourceSector.Size_y ) Sz.y -= ( Sp.y + Sz.y - (int)SourceSector.Size_y );
			if( ( Sp.z + Sz.z ) > SourceSector.Size_z ) Sz.z -= ( Sp.z + Sz.z - (int)SourceSector.Size_z );

			int x, y, z;
			ushort Voxel;

			for( z = 0; z < Sz.z; z++ )
				for( x = 0; x < Sz.x; x++ )
					for( y = 0; y < Sz.y; y++ )
					{
						Voxel = SourceSector.GetCube( (uint)(Sp.x + x), (uint)(Sp.y + y ), (uint)( Sp.z + z ));
						if( Voxel != 0 ) Draw_safe_SetVoxel( (uint)( Dp.x + x ), (uint)( Dp.y + y ), (uint)( Dp.z + z ), Voxel );
					}
		}

		public void Draw_subtree_1( VoxelSector Sector, ref btVector3 Point, ref ZPolar3f Direction, Random Random, double TotalLen )
		{
			btVector3 BranchVector, NewPoint;
			ZPolar3f NewDirection, NewDirection2;
			double angle, angle2;
			ZRect3L_2 Rect;
			ZRect1d Thickness;

			BranchVector.x = (float)( Direction.Len * Math.Sin( Direction.yaw / 57.295779506 ) * Math.Cos( Direction.pitch / 57.295779506 ) );
			BranchVector.y = (float)( Direction.Len * Math.Sin( Direction.yaw / 57.295779506 ) * Math.Sin( Direction.pitch / 57.295779506 ) );
			BranchVector.z = (float)( Direction.Len * Math.Cos( Direction.yaw / 57.295779506 ) );

			NewPoint.x = Point.x + BranchVector.x;
			NewPoint.y = Point.y + BranchVector.y;
			NewPoint.z = Point.z + BranchVector.z;
			NewPoint.w = 0;

			NewDirection.Len = Direction.Len;
			NewDirection.pitch = Direction.pitch;
			NewDirection.roll = Direction.roll;
			NewDirection.yaw = Direction.yaw;
			NewDirection2.Len = Direction.Len;
			NewDirection2.pitch = Direction.pitch;
			NewDirection2.roll = Direction.roll;
			NewDirection2.yaw = Direction.yaw;

			TotalLen += Direction.Len;

			angle = (double)( (int)Random.Next() ) / 23860929 / 3;
			angle2 = (double)( (int)Random.Next() ) / 23860929 / 3;
			NewDirection.pitch += (float)angle;
			NewDirection.yaw += (float)angle2;
			NewDirection2.pitch -= (float)angle;
			NewDirection2.yaw -= (float)angle2;

			Rect.sx = (int)Math.Floor( Point.x + 0.5 );
			Rect.sy = (int)Math.Floor( Point.y + 0.5 );
			Rect.sz = (int)Math.Floor( Point.z + 0.5 );
			Rect.ex = (int)Math.Floor( NewPoint.x + 0.5 );
			Rect.ey = (int)Math.Floor( NewPoint.y + 0.5 );
			Rect.ez = (int)Math.Floor( NewPoint.z + 0.5 );
			Thickness.Start = 1;
			Thickness.End = 1;
			Draw_safe_VoxelLine2( ref Rect, ref Thickness, 10 );

			if( TotalLen < 128.0 )
			{
				Draw_subtree_1( Sector, ref NewPoint, ref NewDirection, Random, TotalLen );
				Draw_subtree_1( Sector, ref NewPoint, ref NewDirection2, Random, TotalLen );
			}
		}

		public void Draw_safe_Tree_Type_1( VoxelSector Sector, ref ZVector3L Coords )
		{
			btVector3 Point;
			Random Random = new Random();
			ZPolar3f Direction;

			Point.x = Coords.x;
			Point.y = Coords.y;
			Point.z = Coords.z;
			Point.w = 0;

			Direction.Len = 10.0f;
			Direction.pitch = 90.0f;
			Direction.yaw = 90.0f;
			Direction.roll = 0;

			Draw_subtree_1( Sector, ref Point, ref Direction, Random, 0.0 );
		}

		public void Draw_subtree_2( ref btVector3 Point, ref ZPolar3f Direction, Random Random, double TotalLen )
		{
			btVector3 BranchVector, NewPoint;
			ZPolar3f NewDirection, NewDirection2;
			double angle, angle2;
			ZRect3L_2 Rect;
			ZRect1d Thickness;

			BranchVector.x = (float)( Direction.Len * Math.Sin( Direction.yaw / 57.295779506 ) * Math.Cos( Direction.pitch / 57.295779506 ) );
			BranchVector.y = (float)( Direction.Len * Math.Sin( Direction.yaw / 57.295779506 ) * Math.Sin( Direction.pitch / 57.295779506 ) );
			BranchVector.z = (float)( Direction.Len * Math.Cos( Direction.yaw / 57.295779506 ) );

			NewPoint.x = Point.x + BranchVector.x;
			NewPoint.y = Point.y + BranchVector.y;
			NewPoint.z = Point.z + BranchVector.z;
			NewPoint.w = 0;

			NewDirection.Len = Direction.Len;
			NewDirection.pitch = Direction.pitch;
			NewDirection.roll = Direction.roll;
			NewDirection.yaw = Direction.yaw;
			NewDirection2.Len = Direction.Len;
			NewDirection2.pitch = Direction.pitch;
			NewDirection2.roll = Direction.roll;
			NewDirection2.yaw = Direction.yaw;

			TotalLen += Direction.Len;

			angle = (double)( (int)Random.Next() ) / 23860929 / 3;
			angle2 = (double)( (int)Random.Next() ) / 23860929 / 3;
			NewDirection.pitch += (float)angle;
			NewDirection.yaw += (float)angle2;
			NewDirection2.pitch -= (float)angle;
			NewDirection2.yaw -= (float)angle2;

			Rect.sx = (int)Math.Floor( Point.x + 0.5 );
			Rect.sy = (int)Math.Floor( Point.y + 0.5 );
			Rect.sz = (int)Math.Floor( Point.z + 0.5 );
			Rect.ex = (int)Math.Floor( NewPoint.x + 0.5 );
			Rect.ey = (int)Math.Floor( NewPoint.y + 0.5 );
			Rect.ez = (int)Math.Floor( NewPoint.z + 0.5 );
			Thickness.Start = 1;
			Thickness.End = 1;
			Draw_safe_VoxelLine2( ref Rect, ref Thickness, 68 );


			if( TotalLen > 40.0 ) this.Draw_safe_Sphere( NewPoint.x, NewPoint.y, NewPoint.z, 10.0, 67, false );
			if( TotalLen < 64.0 )
			{
				Draw_subtree_2( ref NewPoint, ref NewDirection, Random, TotalLen );
				Draw_subtree_2( ref NewPoint, ref NewDirection2, Random, TotalLen );
			}
			else
			{
				this.Draw_safe_Sphere( NewPoint.x, NewPoint.y, NewPoint.z, 10.0, 67, false );
			}


		}

		public void Draw_safe_Tree_Type_2( ref ZVector3L Coords )
		{
			btVector3 Point;
			Random Random = new Random();
			ZPolar3f Direction;

			Point.x = Coords.x;
			Point.y = Coords.y;
			Point.z = Coords.z;
			Point.w = 0;

			Direction.Len = 10.0f;
			Direction.pitch = 90.0f;
			Direction.yaw = 90.0f;
			Direction.roll = 0;

			this.Draw_subtree_2( ref Point, ref Direction, Random, 0.0 );
		}

		public void Draw_subtree_3( ref btVector3 Point, ref ZPolar3f Direction, Random Random, double TotalLen )
		{
			btVector3 BranchVector, NewPoint;
			ZPolar3f NewDirection, NewDirection2;
			double angle, angle2;
			ZRect3L_2 Rect;
			ZRect1d Thickness;

			BranchVector.x = (float)( Direction.Len * Math.Sin( Direction.yaw / 57.295779506 ) * Math.Cos( Direction.pitch / 57.295779506 ) );
			BranchVector.y = (float)( Direction.Len * Math.Sin( Direction.yaw / 57.295779506 ) * Math.Sin( Direction.pitch / 57.295779506 ) );
			BranchVector.z = (float)( Direction.Len * Math.Cos( Direction.yaw / 57.295779506 ) );

			NewPoint.x = Point.x + BranchVector.x;
			NewPoint.y = Point.y + BranchVector.y;
			NewPoint.z = Point.z + BranchVector.z;
			NewPoint.w = 0;

			NewDirection.Len = Direction.Len;
			NewDirection.pitch = Direction.pitch;
			NewDirection.roll = Direction.roll;
			NewDirection.yaw = Direction.yaw;
			NewDirection2.Len = Direction.Len;
			NewDirection2.pitch = Direction.pitch;
			NewDirection2.roll = Direction.roll;
			NewDirection2.yaw = Direction.yaw;

			if( NewDirection2.Len > 10.0 ) NewDirection2.Len = 10.0f;
			if( NewDirection.Len > 10.0 ) NewDirection.Len = 10.0f;

			TotalLen += Direction.Len;

			angle = (double)( (int)Random.Next() ) / 23860929 / 3;
			angle2 = (double)( (int)Random.Next() ) / 23860929 / 3;
			NewDirection.pitch += (float)angle;
			NewDirection.yaw += (float)angle2;
			NewDirection2.pitch -= (float)angle;
			NewDirection2.yaw -= (float)angle2;

			Rect.sx = (int)Math.Floor( Point.x + 0.5 );
			Rect.sy = (int)Math.Floor( Point.y + 0.5 );
			Rect.sz = (int)Math.Floor( Point.z + 0.5 );
			Rect.ex = (int)Math.Floor( NewPoint.x + 0.5 );
			Rect.ey = (int)Math.Floor( NewPoint.y + 0.5 );
			Rect.ez = (int)Math.Floor( NewPoint.z + 0.5 );
			Thickness.Start = 5 - ( ( TotalLen - Direction.Len ) / 24 );
			Thickness.End = 5 - ( TotalLen / 24 );
			Draw_safe_VoxelLine2( ref Rect, ref Thickness, 68 );


			if( TotalLen > 100.0 ) this.Draw_safe_Sphere( NewPoint.x, NewPoint.y, NewPoint.z, 10.0, 70, false );
			if( TotalLen < 120.0 )
			{
				Draw_subtree_3( ref NewPoint, ref NewDirection, Random, TotalLen );
				Draw_subtree_3( ref NewPoint, ref NewDirection2, Random, TotalLen );
			}
			else
			{
				this.Draw_safe_Sphere( NewPoint.x, NewPoint.y, NewPoint.z, 10.0, 70, false );
			}


		}

		public void Draw_safe_Tree_Type_3( ref ZVector3L Coords )
		{
			btVector3 Point;
			Random Random = new Random();
			ZPolar3f Direction;

			Point.x = (float)Coords.x;
			Point.y = (float)Coords.y;
			Point.z = (float)Coords.z;
			Point.w = 0;

			Direction.Len = 40.0f;
			Direction.pitch = 90.0f;
			Direction.yaw = 90.0f;
			Direction.roll = 0;

			Draw_subtree_3( ref Point, ref Direction, Random, 0.0 );
		}


		public void BlitSector( VoxelSector Source, ref ZVector3L Offset )
		{
			ZVector3L Size;
			int t;
			int xs, zs, xd, zd;
			ZVector3L SStart, SEnd, DStart, DEnd, SSize, DSize;
			int SOffsetStep_x, SOffsetStep_z, DOffsetStep_z, DOffsetStep_x;
			int So, Do, SoEnd;

			SSize.x = (int)Source.Size_x; SSize.y = (int)Source.Size_y; SSize.z = (int)Source.Size_z;
			DSize.x = (int)Size_x; DSize.y = (int)Size_y; DSize.z = (int)Size_z;

			SStart.x = SStart.y = SStart.z = 0;
			SEnd = SSize;
			DStart = Offset;
			DEnd = SSize;
			DEnd.x += Offset.x; DEnd.y += Offset.y; DEnd.z += Offset.z;

			if( ( t = DEnd.x - DSize.x ) > 0 ) { DEnd.x -= t; SEnd.x -= t; }
			if( ( t = DEnd.y - DSize.y ) > 0 ) { DEnd.y -= t; SEnd.y -= t; }
			if( ( t = DEnd.z - DSize.z ) > 0 ) { DEnd.z -= t; SEnd.z -= t; }

			if( ( t = -DStart.x ) > 0 ) { DStart.x = 0; SStart.x += t; }
			if( ( t = -DStart.y ) > 0 ) { DStart.y = 0; SStart.y += t; }
			if( ( t = -DStart.z ) > 0 ) { DStart.z = 0; SStart.z += t; }

			if( SStart.x >= SEnd.x ) return;
			if( SStart.y >= SEnd.y ) return;
			if( SStart.z >= SEnd.z ) return;

			Size.x = (int)Source.Size_x;
			Size.y = (int)Source.Size_y;
			Size.z = (int)Source.Size_z;



			SOffsetStep_x = (int)Source.Size_y;
			SOffsetStep_z = (int)Source.Size_y * (int)Source.Size_x;
			DOffsetStep_x = (int)Size_y;
			DOffsetStep_z = (int)Size_y * (int)Size_x;

			int SOffsetStart_x, SOffsetStart_y, SOffsetStart_z;
			int SOffsetEnd_x, SOffsetEnd_z;
			int DOffsetStart_x, DOffsetStart_y, DOffsetStart_z;

			SOffsetStart_x = SStart.x * SOffsetStep_x;
			SOffsetEnd_x = ( SEnd.x ) * SOffsetStep_x;
			DOffsetStart_x = DStart.x * DOffsetStep_x;

			SOffsetStart_y = SStart.y;
			DOffsetStart_y = DStart.y;

			SOffsetStart_z = SStart.z * SOffsetStep_z;
			SOffsetEnd_z = ( SEnd.z ) * SOffsetStep_z;
			DOffsetStart_z = DStart.z * DOffsetStep_z;



			for( zs = SOffsetStart_z, zd = DOffsetStart_z; zs < SOffsetEnd_z; zs += SOffsetStep_z, zd += DOffsetStep_z )
			{
				for( xs = SOffsetStart_x, xd = DOffsetStart_x; xs < SOffsetEnd_x; xs += SOffsetStep_x, xd += DOffsetStep_x )
				{
					So = zs + xs + SOffsetStart_y;
					Do = zd + xd + DOffsetStart_y;
					SoEnd = zs + xs + SEnd.y;
					for( ; So < SoEnd; So++, Do++ )
					{
						ushort VoxelType;
						VoxelType = Source.Data.Data[So];

						if( VoxelType != 0 )
						{
							// Bound checking check
							if( VoxelGlobalSettings.COMPILEOPTION_BOUNDCHECKINGSLOW )
							{
								if( So > Source.DataSize ) Debugger.Break();
								if( Do > DataSize ) Debugger.Break();
							}

							Data.Data[Do] = VoxelType;
							if( ( Data.OtherInfos[Do] = Source.Data.OtherInfos[So] ) != null )
							{
								if( VoxelTypeManager.VoxelTable[Data.Data[So]].properties.Is_HasAllocatedMemoryExtension )
								{
									Data.OtherInfos[Do] = Data.OtherInfos[So].GetNewCopy();
								}
							}
						}
					}
				}
			}
		}

		public void Subst( ushort Source_VoxelType, ushort Dest_VoxelType )
		{
			int i;
			for( i = 0; i < DataSize; i++ )
			{
				if( Data.Data[i] == Source_VoxelType ) Data.Data[i] = Dest_VoxelType;
			}

		}

	}
}
