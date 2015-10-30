using Bullet.LinearMath;
using System;
using System.Collections.Generic;
using System.Text;
using Voxelarium.Core.Support;
using Voxelarium.Core.UI;
using Voxelarium.Core.Voxels.IO;
using Voxelarium.Core.Voxels.UI;

namespace Voxelarium.Core.Voxels
{
	public class VoxelWorld
	{

		public VoxelSector WorkingFullSector;
		public VoxelSector WorkingEmptySector;
		public VoxelSector WorkingScratchSector;
		VoxelGameEnvironment GameEnv;
		internal SectorRingList SectorEjectList;
		public VoxelSector SectorList;

		internal RenderInterface renderer;
		VoxelSector[] SectorTable;
		internal VoxelTypeManager VoxelTypeManager;
		internal SectorLoader SectorLoader;

			// world has voxel size... all blocks in a 'world' are constant
		public int VoxelBlockSizeBits = 2;
		public int VoxelBlockSize = 1 << 2;

		int TableSize;
		public int Size_x;
		public int Size_y;
		public int Size_z;
		public btMatrix3x3 orientation;
		public int UniverseNum;

		internal TextureAtlas TextureAtlas;


		public VoxelWorld( VoxelGameEnvironment GameEnv )
		{
			uint i;
			this.GameEnv = GameEnv;
			SectorEjectList = new SectorRingList( 256 * 256 * 32/*65536*/);
			TextureAtlas = new TextureAtlas( 32, 64 );
			Size_x = 0x20;
			Size_y = 0x20;
			Size_z = 0x20;

			TableSize = Size_x * Size_y * Size_z;
			SectorTable = new VoxelSector[TableSize];

			for( i = 0; i < TableSize; i++ ) SectorTable[i] = null;

			WorkingFullSector = new VoxelSector( this );
			GameEnv.Basic_Renderer.GetCuller().InitFaceCullData( WorkingFullSector );
			WorkingFullSector.Fill( 0x0001 );
			WorkingEmptySector = new VoxelSector( this );
			GameEnv.Basic_Renderer.GetCuller().InitFaceCullData( WorkingEmptySector );
			WorkingEmptySector.Fill( 0 );
			WorkingScratchSector = new VoxelSector( this );
			GameEnv.Basic_Renderer.GetCuller().InitFaceCullData( WorkingScratchSector );

			SectorList = null;
			UniverseNum = 1;
			VoxelTypeManager = null;
		}

		~VoxelWorld()
		{
			VoxelSector Sector, NewSector;

			Sector = SectorList;

			while( Sector != null )
			{
				NewSector = Sector.GlobalList_Next;
				if( VoxelGlobalSettings.COMPILEOPTION_ALLOWSAVE )
				{
					if( VoxelGlobalSettings.COMPILEOPTION_SAVEONLYMODIFIED )
					{
						Log.log( " *** SAVE INCOMPLETE *** " );
/*
						if( Sector.IsMustBeSaved() )
						{
							Sector.Save( UniverseNum );
						}
						*/
					}
				}
				Sector.Dispose();
				Sector = NewSector;
			}

			if( SectorTable != null ) { SectorTable = null; TableSize = 0; }


			if( WorkingFullSector != null ) { WorkingFullSector.Dispose(); WorkingFullSector = null; }
			if( WorkingEmptySector != null ) { WorkingEmptySector.Dispose(); WorkingEmptySector = null; }
			if( WorkingScratchSector != null ) { WorkingScratchSector.Dispose(); WorkingScratchSector = null; }
			SectorList = null;
			UniverseNum = 0;
			if( SectorEjectList != null ) SectorEjectList.Dispose();
			SectorEjectList = null;
		}

		internal VoxelSector FindSector( int x, int y, int z )
		{
			int xs, ys, zs, Offset;
			VoxelSector SectorPointer;

			xs = x % Size_x;
			ys = y % Size_y;
			zs = z % Size_z;

			xs &= 0x1f;
			ys &= 0x1f;
			zs &= 0x1f;

			Offset = xs + ys * Size_x + ( zs * Size_x * Size_y );

			SectorPointer = SectorTable[Offset];
			while( SectorPointer != null )
			{
				if( ( SectorPointer.Pos_x == x ) && ( SectorPointer.Pos_y == y ) && ( SectorPointer.Pos_z == z ) ) return ( SectorPointer );
				SectorPointer = SectorPointer.Next;
			}
			return null;
		}

		VoxelSector FindSector_Secure( int x, int y, int z ) // Create sector if not in memory.
		{
			int xs, ys, zs, Offset;
			VoxelSector SectorPointer;

			xs = x % Size_x;
			ys = y % Size_y;
			zs = z % Size_z;

			xs &= 0x1f;
			ys &= 0x1f;
			zs &= 0x1f;

			Offset = xs + ys * Size_x + ( zs * Size_x * Size_y );

			while( true )
			{
				SectorPointer = SectorTable[Offset];
				while( SectorPointer != null )
				{
					if( ( SectorPointer.Pos_x == x ) && ( SectorPointer.Pos_y == y ) && ( SectorPointer.Pos_z == z ) ) return ( SectorPointer );
					SectorPointer = SectorPointer.Next;
				}
				RequestSector( x, y, z, 5 );
				System.Threading.Thread.Sleep( 2 );
				ProcessNewLoadedSectors();
			}
		}

		internal void ProcessNewLoadedSectors()
		{
			VoxelSector Sector, AdjSector;
			//if( SectorLoader == null ) return;

			while( ( Sector = SectorLoader.GetRequested() ) != null )
			{
				if( FindSector( Sector.Pos_x, Sector.Pos_y, Sector.Pos_z ) == null )
				{
					AddSector( Sector );
					Sector.Culler = renderer.GetCuller( );
					Sector.Culler.InitFaceCullData( Sector );

					Sector.Culler.CullSector( Sector, true, 0 );

					Sector.Flag_Void_Regular = false;
					Sector.Flag_Void_Transparent = false;

					Sector.Flag_Render_Dirty = true;
					//printf("AddSector: %ld,%ld,%ld\n",Sector.Pos_x, Sector.Pos_y, Sector.Pos_z);

					// Partial face culing for adjacent sectors
					AdjSector = FindSector( Sector.Pos_x - 1, Sector.Pos_y, Sector.Pos_z );
					if( AdjSector != null ) { AdjSector.PartialCulling |= VoxelSector.FACEDRAW_Operations.LEFT; }
					AdjSector = FindSector( Sector.Pos_x + 1, Sector.Pos_y, Sector.Pos_z );
					if( AdjSector != null ) { AdjSector.PartialCulling |= VoxelSector.FACEDRAW_Operations.RIGHT; }
					AdjSector = FindSector( Sector.Pos_x, Sector.Pos_y, Sector.Pos_z - 1 );
					if( AdjSector != null ) { AdjSector.PartialCulling |= VoxelSector.FACEDRAW_Operations.AHEAD; }
					AdjSector = FindSector( Sector.Pos_x, Sector.Pos_y, Sector.Pos_z + 1 );
					if( AdjSector != null ) { AdjSector.PartialCulling |= VoxelSector.FACEDRAW_Operations.BEHIND; }
					AdjSector = FindSector( Sector.Pos_x, Sector.Pos_y - 1, Sector.Pos_z );
					if( AdjSector != null ) { AdjSector.PartialCulling |= VoxelSector.FACEDRAW_Operations.BELOW; }
					AdjSector = FindSector( Sector.Pos_x, Sector.Pos_y + 1, Sector.Pos_z );
					if( AdjSector != null ) { AdjSector.PartialCulling |= VoxelSector.FACEDRAW_Operations.ABOVE; }

				}
				else { Sector.Dispose(); Log.log( "Loading already used sector***\n" ); }
			}
		}

		void AddSector( VoxelSector Sector )
		{
			int x, y, z, Offset;
			VoxelSector SectorPointer;

			{
				int n;
				for( n = 0; n < 6; n++ )
				{
					VoxelSector near_sec = FindSector( Sector.Pos_x + VoxelReactor.nbp6[n].x
													  , Sector.Pos_y + VoxelReactor.nbp6[n].y
													  , Sector.Pos_z + VoxelReactor.nbp6[n].z );
					if( near_sec != null )
					{
						Sector.near_sectors[n] = near_sec;
						near_sec.near_sectors[n ^ 1] = Sector;
					}
				}
			}

			// Adding to fast access hash

			x = ( Sector.Pos_x % Size_x ) & 0x1f;
			y = ( Sector.Pos_y % Size_y ) & 0x1f;
			z = ( Sector.Pos_z % Size_z ) & 0x1f;

			Offset = x + y * Size_x + ( z * Size_x * Size_y );

			if( SectorTable[Offset] == null )
			{
				SectorTable[Offset] = Sector; Sector.Next = null; Sector.Pred = null;
			}
			else
			{
				SectorPointer = SectorTable[Offset];
				while( SectorPointer.Next != null ) SectorPointer = SectorPointer.Next;
				SectorPointer.Next = Sector;
				Sector.Next = null;
				Sector.Pred = SectorPointer;
			}

			// Adding to sequential access global list

			if( SectorList == null )
			{
				SectorList = Sector;
				Sector.GlobalList_Next = null;
				Sector.GlobalList_Pred = null;
			}
			else
			{
				Sector.GlobalList_Next = SectorList;
				Sector.GlobalList_Pred = null;
				SectorList.GlobalList_Pred = Sector;
				SectorList = Sector;
			}
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public bool GetVoxelRef( out VoxelRef result, int x, int y, int z )
		{
			result.Sector = FindSector( x >> VoxelSector.ZVOXELBLOCSHIFT_X, y >> VoxelSector.ZVOXELBLOCSHIFT_Y, z >> VoxelSector.ZVOXELBLOCSHIFT_Z );
			result.wx = x;
			result.wy = y;
			result.wz = z;
			result.Offset = (uint)( ( result.y = (byte)(y & VoxelSector.ZVOXELBLOCMASK_Y) )
				   + ( ( result.x = (byte)( x & VoxelSector.ZVOXELBLOCMASK_X) ) << VoxelSector.ZVOXELBLOCSHIFT_Y )
				   + ( ( result.z = (byte)( z & VoxelSector.ZVOXELBLOCMASK_Z) ) << ( VoxelSector.ZVOXELBLOCSHIFT_Y + VoxelSector.ZVOXELBLOCSHIFT_X ) ) );

			result.World = this;
			result.VoxelTypeManager = VoxelTypeManager;
			if( result.Sector == null )
			{
				result.Type = 0;
				result.VoxelExtension = null;
				return false;
			}
			result.Type = result.Sector.Data.Data[result.Offset];
			result.VoxelExtension = result.Sector.Data.OtherInfos[result.Offset];

			return true;
		}

		internal void SetUniverseNum( int UniverseNum ) { this.UniverseNum = UniverseNum; }
		internal void SetVoxelTypeManager( VoxelTypeManager Manager )
		{
			VoxelTypeManager = Manager;

			Manager.LoadTexturesToAtlas( TextureAtlas );
        }
		internal void CreateDemoWorld()
		{
			return;
			int x, y, z;
			VoxelSector Sector;
			/*
			for( x = -1; x < 1; x++ )
			{
				for( y = -1; y < 1; y++ )
				{
					for( z = -1; z < 1; z++ )
					{
						Sector = new ZVoxelSector();
						Sector.SetVoxelTypeManager( VoxelTypeManager );
						Sector.SetPos( x, y, z );
						Sector.MakeSector();

						AddSector( Sector );
					}
				}
			}
			*/
		}

		internal void SetVoxel( int x, int y, int z, int VoxelValue )
		{
			VoxelSector Sector;
			Sector = FindSector( x >> VoxelSector.ZVOXELBLOCSHIFT_X, y >> VoxelSector.ZVOXELBLOCSHIFT_Y, z >> VoxelSector.ZVOXELBLOCSHIFT_Z );
			if( Sector == null ) return;
			Sector.SetCube( x, y, z, VoxelValue );
		}

		internal bool Load()
		{
			VoxelSector Sector;
			bool Result;
			Result = true;
			Sector = SectorList;

			while( Sector != null )
			{
				if( !Sector.Load( UniverseNum ) ) Result = false;
				Sector = Sector.GlobalList_Next;
			}
			return ( Result );
		}


		void SectorUpdateFaceCulling( int x, int y, int z, bool Isolated = false )
		{
			VoxelSector[] SectorTable = new VoxelSector[27];
			VoxelType[] VoxelTypeTable;
			VoxelSector MissingSector;

			ushort[][] BlocMatrix = new ushort[3][];
			ushort[] tmpp;
			int i;

			ushort[] s1 = new ushort[9];
			ushort[] s2 = new ushort[9];
			ushort[] s3 = new ushort[9];

			BlocMatrix[0] = s1;
			BlocMatrix[1] = s2;
			BlocMatrix[2] = s3;


			if( Isolated ) MissingSector = WorkingEmptySector;
			else MissingSector = WorkingFullSector;

			/*
			  if (x==0 && y== 0 && z==0)
			  {
				printf("Entering..");
			  }
			*/

			// (VoxelSector.FACEDRAW_Operations.ABOVE | VoxelSector.FACEDRAW_Operations.BELOW | VoxelSector.FACEDRAW_Operations.LEFT | VoxelSector.FACEDRAW_Operations.RIGHT | VoxelSector.FACEDRAW_Operations.AHEAD | VoxelSector.FACEDRAW_Operations.BEHIND);
			for( i = 0; i < 27; i++ ) SectorTable[i] = MissingSector;
			SectorTable[0] = FindSector( x, y, z ); if( SectorTable[0] == null ) { return; }
			SectorTable[1] = FindSector( x - 1, y, z ); if( SectorTable[1] == null ) { SectorTable[1] = MissingSector; SectorTable[0].PartialCulling |= VoxelSector.FACEDRAW_Operations.LEFT; }
			SectorTable[2] = FindSector( x + 1, y, z ); if( SectorTable[2] == null ) { SectorTable[2] = MissingSector; SectorTable[0].PartialCulling |= VoxelSector.FACEDRAW_Operations.RIGHT; }
			SectorTable[3] = FindSector( x, y, z - 1 ); if( SectorTable[3] == null ) { SectorTable[3] = MissingSector; SectorTable[0].PartialCulling |= VoxelSector.FACEDRAW_Operations.AHEAD; }
			SectorTable[6] = FindSector( x, y, z + 1 ); if( SectorTable[6] == null ) { SectorTable[6] = MissingSector; SectorTable[0].PartialCulling |= VoxelSector.FACEDRAW_Operations.BEHIND; }
			SectorTable[9] = FindSector( x, y - 1, z ); if( SectorTable[9] == null ) { SectorTable[9] = MissingSector; SectorTable[0].PartialCulling |= VoxelSector.FACEDRAW_Operations.BELOW; }
			SectorTable[18] = FindSector( x, y + 1, z ); if( SectorTable[18] == null ) { SectorTable[18] = MissingSector; SectorTable[0].PartialCulling |= VoxelSector.FACEDRAW_Operations.ABOVE; }


			int xc, yc, zc;
			int xp, yp, zp;
			int xpp, ypp, zpp;
			VoxelSector.FACEDRAW_Operations info;
			int MainVoxelDrawInfo;

			//SectorTable[0].Flag_Void_Regular = true;
			//SectorTable[0].Flag_Void_Transparent = true;
			VoxelTypeTable = this.VoxelTypeManager.VoxelTable;

			for( xc = 0; xc < VoxelSector.ZVOXELBLOCSIZE_X; xc++ )
			{
				xp = xc + 1; xpp = xc + 2;
				for( zc = 0; zc < VoxelSector.ZVOXELBLOCSIZE_Z; zc++ )
				{
					zp = zc + 1; zpp = zc + 2;

					// Prefetching the bloc matrix (only 2 rows)
					//    BlocMatrix[1][0] = SectorTable[(VoxelSector.STableX[xc ]+VoxelSector.STableY[0]+VoxelSector.STableZ[zc ])].Data.Data[VoxelSector.OfTableX[xc]+VoxelSector.OfTableY[0]+VoxelSector.OfTableZ[zc]];
					BlocMatrix[1][1] = SectorTable[( VoxelSector.STableX[xp] + VoxelSector.STableY[0] + VoxelSector.STableZ[zc] )].Data.Data[VoxelSector.OfTableX[xp] + VoxelSector.OfTableY[0] + VoxelSector.OfTableZ[zc]];
					//    BlocMatrix[1][2] = SectorTable[(VoxelSector.STableX[xpp]+VoxelSector.STableY[0]+VoxelSector.STableZ[zc ])].Data.Data[VoxelSector.OfTableX[xpp]+VoxelSector.OfTableY[0]+VoxelSector.OfTableZ[zc]];
					BlocMatrix[1][3] = SectorTable[( VoxelSector.STableX[xc] + VoxelSector.STableY[0] + VoxelSector.STableZ[zp] )].Data.Data[VoxelSector.OfTableX[xc] + VoxelSector.OfTableY[0] + VoxelSector.OfTableZ[zp]];
					BlocMatrix[1][4] = SectorTable[( VoxelSector.STableX[xp] + VoxelSector.STableY[0] + VoxelSector.STableZ[zp] )].Data.Data[VoxelSector.OfTableX[xp] + VoxelSector.OfTableY[0] + VoxelSector.OfTableZ[zp]];
					BlocMatrix[1][5] = SectorTable[( VoxelSector.STableX[xpp] + VoxelSector.STableY[0] + VoxelSector.STableZ[zp] )].Data.Data[VoxelSector.OfTableX[xpp] + VoxelSector.OfTableY[0] + VoxelSector.OfTableZ[zp]];
					//    BlocMatrix[1][6] = SectorTable[(VoxelSector.STableX[xc ]+VoxelSector.STableY[0]+VoxelSector.STableZ[zpp])].Data.Data[VoxelSector.OfTableX[xc]+VoxelSector.OfTableY[0]+VoxelSector.OfTableZ[zpp]];
					BlocMatrix[1][7] = SectorTable[( VoxelSector.STableX[xp] + VoxelSector.STableY[0] + VoxelSector.STableZ[zpp] )].Data.Data[VoxelSector.OfTableX[xp] + VoxelSector.OfTableY[0] + VoxelSector.OfTableZ[zpp]];
					//    BlocMatrix[1][8] = SectorTable[(VoxelSector.STableX[xpp]+VoxelSector.STableY[0]+VoxelSector.STableZ[zpp])].Data.Data[VoxelSector.OfTableX[xpp]+VoxelSector.OfTableY[0]+VoxelSector.OfTableZ[zpp]];

					//    BlocMatrix[2][0] = SectorTable[(VoxelSector.STableX[xc ]+VoxelSector.STableY[1]+VoxelSector.STableZ[zc ])].Data.Data[VoxelSector.OfTableX[xc ]+VoxelSector.OfTableY[1]+VoxelSector.OfTableZ[zc ]];
					BlocMatrix[2][1] = SectorTable[( VoxelSector.STableX[xp] + VoxelSector.STableY[1] + VoxelSector.STableZ[zc] )].Data.Data[VoxelSector.OfTableX[xp] + VoxelSector.OfTableY[1] + VoxelSector.OfTableZ[zc]];
					//    BlocMatrix[2][2] = SectorTable[(VoxelSector.STableX[xpp]+VoxelSector.STableY[1]+VoxelSector.STableZ[zc ])].Data.Data[VoxelSector.OfTableX[xpp]+VoxelSector.OfTableY[1]+VoxelSector.OfTableZ[zc ]];
					BlocMatrix[2][3] = SectorTable[( VoxelSector.STableX[xc] + VoxelSector.STableY[1] + VoxelSector.STableZ[zp] )].Data.Data[VoxelSector.OfTableX[xc] + VoxelSector.OfTableY[1] + VoxelSector.OfTableZ[zp]];
					BlocMatrix[2][4] = SectorTable[( VoxelSector.STableX[xp] + VoxelSector.STableY[1] + VoxelSector.STableZ[zp] )].Data.Data[VoxelSector.OfTableX[xp] + VoxelSector.OfTableY[1] + VoxelSector.OfTableZ[zp]];
					BlocMatrix[2][5] = SectorTable[( VoxelSector.STableX[xpp] + VoxelSector.STableY[1] + VoxelSector.STableZ[zp] )].Data.Data[VoxelSector.OfTableX[xpp] + VoxelSector.OfTableY[1] + VoxelSector.OfTableZ[zp]];
					//    BlocMatrix[2][6] = SectorTable[(VoxelSector.STableX[xc ]+VoxelSector.STableY[1]+VoxelSector.STableZ[zpp])].Data.Data[VoxelSector.OfTableX[xc ]+VoxelSector.OfTableY[1]+VoxelSector.OfTableZ[zpp]];
					BlocMatrix[2][7] = SectorTable[( VoxelSector.STableX[xp] + VoxelSector.STableY[1] + VoxelSector.STableZ[zpp] )].Data.Data[VoxelSector.OfTableX[xp] + VoxelSector.OfTableY[1] + VoxelSector.OfTableZ[zpp]];
					//    BlocMatrix[2][8] = SectorTable[(VoxelSector.STableX[xpp]+VoxelSector.STableY[1]+VoxelSector.STableZ[zpp])].Data.Data[VoxelSector.OfTableX[xpp]+VoxelSector.OfTableY[1]+VoxelSector.OfTableZ[zpp]];

					for( yc = 0; yc < VoxelSector.ZVOXELBLOCSIZE_Y; yc++ )
					{
						yp = yc + 1; ypp = yc + 2;

						// Scrolling bloc matrix by exchangingypp references.
						tmpp = BlocMatrix[0];
						BlocMatrix[0] = BlocMatrix[1];
						BlocMatrix[1] = BlocMatrix[2];
						BlocMatrix[2] = tmpp;

						// Fetching a new bloc of data slice;

						//      BlocMatrix[2][0] = SectorTable[(VoxelSector.STableX[xc ]+STableY[ypp]+VoxelSector.STableZ[zc ])].Data.Data[VoxelSector.OfTableX[xc ]+VoxelSector.OfTableY[ypp]+VoxelSector.OfTableZ[zc ]];
						BlocMatrix[2][1] = SectorTable[( VoxelSector.STableX[xp] + VoxelSector.STableY[ypp] + VoxelSector.STableZ[zc] )].Data.Data[VoxelSector.OfTableX[xp] + VoxelSector.OfTableY[ypp] + VoxelSector.OfTableZ[zc]];
						//      BlocMatrix[2][2] = SectorTable[(VoxelSector.STableX[xpp]+STableY[ypp]+VoxelSector.STableZ[zc ])].Data.Data[VoxelSector.OfTableX[xpp]+VoxelSector.OfTableY[ypp]+VoxelSector.OfTableZ[zc ]];
						BlocMatrix[2][3] = SectorTable[( VoxelSector.STableX[xc] + VoxelSector.STableY[ypp] + VoxelSector.STableZ[zp] )].Data.Data[VoxelSector.OfTableX[xc] + VoxelSector.OfTableY[ypp] + VoxelSector.OfTableZ[zp]];
						BlocMatrix[2][4] = SectorTable[( VoxelSector.STableX[xp] + VoxelSector.STableY[ypp] + VoxelSector.STableZ[zp] )].Data.Data[VoxelSector.OfTableX[xp] + VoxelSector.OfTableY[ypp] + VoxelSector.OfTableZ[zp]];
						BlocMatrix[2][5] = SectorTable[( VoxelSector.STableX[xpp] + VoxelSector.STableY[ypp] + VoxelSector.STableZ[zp] )].Data.Data[VoxelSector.OfTableX[xpp] + VoxelSector.OfTableY[ypp] + VoxelSector.OfTableZ[zp]];
						//      BlocMatrix[2][6] = SectorTable[(VoxelSector.STableX[xc ]+VoxelSector.STableY[ypp]+VoxelSector.STableZ[zpp])].Data.Data[VoxelSector.OfTableX[xc ]+VoxelSector.OfTableY[ypp]+VoxelSector.OfTableZ[zpp]];
						BlocMatrix[2][7] = SectorTable[( VoxelSector.STableX[xp] + VoxelSector.STableY[ypp] + VoxelSector.STableZ[zpp] )].Data.Data[VoxelSector.OfTableX[xp] + VoxelSector.OfTableY[ypp] + VoxelSector.OfTableZ[zpp]];
						//      BlocMatrix[2][8] = SectorTable[(VoxelSector.STableX[xpp]+VoxelSector.STableY[ypp]+VoxelSector.STableZ[zpp])].Data.Data[VoxelSector.OfTableX[xpp]+VoxelSector.OfTableY[ypp]+VoxelSector.OfTableZ[zpp]];

						// Compute face culling info
						/*
								if (x==0 && y== 0 && z==0)
								{
								  if (xc == 15 && yc == 0 && zc ==0)
								  {
									printf("Gotcha\n");
								  }

								}
						*/
						info = 0;
						if( BlocMatrix[1][4] > 0 )
						{
							MainVoxelDrawInfo = VoxelTypeTable[BlocMatrix[1][4]].properties.DrawInfo;
							VoxelSector.FACEDRAW_Operations[] SubTable = RenderInterface.IntFaceStateTable[MainVoxelDrawInfo];
							// {
							/*
							UByte Bm = BlocMatrix[1][1];
							ZVoxelType * Vt = VoxelTypeTable[Bm];
							UByte Di = Vt.DrawInfo;
							UShort st = SubTable[ Di ];
							info |= ( st ) & VoxelSector.FACEDRAW_Operations.AHEAD;
							*/
							// }

							info |= ( ( SubTable[VoxelTypeTable[BlocMatrix[1][1]].properties.DrawInfo] ) & VoxelSector.FACEDRAW_Operations.AHEAD );
							info |= ( ( SubTable[VoxelTypeTable[BlocMatrix[1][7]].properties.DrawInfo] ) & VoxelSector.FACEDRAW_Operations.BEHIND );
							info |= ( ( SubTable[VoxelTypeTable[BlocMatrix[1][3]].properties.DrawInfo] ) & VoxelSector.FACEDRAW_Operations.LEFT );
							info |= ( ( SubTable[VoxelTypeTable[BlocMatrix[1][5]].properties.DrawInfo] ) & VoxelSector.FACEDRAW_Operations.RIGHT );
							info |= ( ( SubTable[VoxelTypeTable[BlocMatrix[0][4]].properties.DrawInfo] ) & VoxelSector.FACEDRAW_Operations.BELOW );
							info |= ( ( SubTable[VoxelTypeTable[BlocMatrix[2][4]].properties.DrawInfo] ) & VoxelSector.FACEDRAW_Operations.ABOVE );
						}

						// Write face culling info to face culling table

						SectorTable[0].Culler.setFaceCulling( SectorTable[0], VoxelSector.OfTableX[xp] + VoxelSector.OfTableY[yp] + VoxelSector.OfTableZ[zp], info );

					}
				}
			}
		}
		internal void WorldUpdateFaceCulling()
		{
			VoxelSector Sector;

			Sector = SectorList;

			while( Sector != null )
			{
				SectorUpdateFaceCulling( Sector.Pos_x, Sector.Pos_y, Sector.Pos_z );
				Sector = Sector.GlobalList_Next;
			}
		}
		internal void RequestSector( int x, int y, int z, int Priority )
		{
			if( SectorLoader == null ) return;

			SectorLoader.Request_Sector( x, y, z, Priority );
		}
		internal void SetSectorLoader( SectorLoader SectorLoader )
		{
			this.SectorLoader = SectorLoader;
		}
		internal bool RequestSectorEject( VoxelSector SectorToEject )
		{ return ( SectorEjectList.PushToList( SectorToEject ) ); }

		internal void ProcessOldEjectedSectors()
		{
			VoxelSector Sector;

			if( SectorLoader == null ) return;

			while( SectorLoader.Is_EjectFileNotFull() )
			{
				if( ( Sector = SectorEjectList.PullFromList() ) == null ) break;

				// printf("EjectPass : %lx %lu\n",Sector,++debug_ejectpass);
				RemoveSector( Sector );
				SectorLoader.Eject_Sector( Sector );
				Sector.Dispose();
			}
		}

		void RemoveSector( VoxelSector Sector )
		{
			int x, y, z, Offset;
			VoxelSector SectorPointer;

			// Finding sector in hash

			x = ( Sector.Pos_x % Size_x ) & 0xff;
			y = ( Sector.Pos_y % Size_y ) & 0xf;
			z = ( Sector.Pos_z % Size_z ) & 0xff;
			Offset = x + y * Size_x + ( z * Size_x * Size_y );
			SectorPointer = SectorTable[Offset];
			while( SectorPointer != Sector )
			{
				if( SectorPointer == null ) return;
				SectorPointer = SectorPointer.Next;
			}

			// Remove from hash
			if( SectorPointer == SectorTable[Offset] ) SectorTable[Offset] = Sector.Next;
			else Sector.Pred.Next = Sector.Next;
			if( Sector.Next != null ) Sector.Next.Pred = Sector.Pred;

			// Remove from global list

			if( Sector.GlobalList_Pred != null ) Sector.GlobalList_Pred.GlobalList_Next = Sector.GlobalList_Next;
			else { SectorList = Sector.GlobalList_Next; }
			if( Sector.GlobalList_Next != null ) Sector.GlobalList_Next.GlobalList_Pred = Sector.GlobalList_Pred;

			// Zeroing fields

			Sector.Next = null; Sector.Pred = null; Sector.GlobalList_Next = null; Sector.GlobalList_Pred = null;

		}
	}
}
