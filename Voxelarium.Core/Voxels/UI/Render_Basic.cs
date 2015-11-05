using Voxelarium.LinearMath;
using OpenTK;
using OpenTK.Graphics.OpenGL;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Text;
using Voxelarium.Core.Support;
using Voxelarium.Core.UI;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.Voxels.UI
{
	internal class Render_Basic : RenderInterface
	{

		internal class BasicVoxelCuller : VoxelCuller
		{
			internal byte[] FaceCulling;
			VoxelWorld world;

			internal override int getFaceCulling( VoxelSector Sector, int offset )
			{
				return FaceCulling[offset];
			}
			internal override void setFaceCulling( VoxelSector Sector, int offset, VoxelSector.FACEDRAW_Operations value )
			{
				FaceCulling[offset] = (byte)value;
			}

			internal override void InitFaceCullData( VoxelSector Sector )
			{
				Sector.Culler = this;
				this.world = Sector.world;
				FaceCulling = new byte[Sector.DataSize];
				int n;
				for( n = 0; n < Sector.DataSize; n++ )
					FaceCulling[n] = 0xFF;
			}
			internal override byte[] GetData()
			{
				return FaceCulling;
			}

			internal override void CullSector( VoxelSector Sector, bool internal_faces, VoxelSector.FACEDRAW_Operations interesting_faces )
			{
				if( world == null )
					return;// false;
				if( internal_faces )
				{
					Render_Basic.SectorUpdateFaceCulling( world, Sector, false );
				}
				else
				{
					Render_Basic.SectorUpdateFaceCulling_Partial( world, Sector, interesting_faces, false );
				}
			}
			void CullSingleVoxel( VoxelSector _Sector, uint offset )
			{

				//bool ZVoxelWorld::SetVoxelSector.RelativeVoxelOrds.WithCullingUpdate(int x, int y, int z, ushort VoxelValue, byte ImportanceFactor, bool CreateExtension, VoxelLocation * Location)
				//{
				uint[] Offset = new uint[19];
				ushort[] VoxelState = new ushort[19];
				ushort Voxel;
				VoxelSector[] Sector = new VoxelSector[19];
				VoxelType[] VoxelTypeTable;
				VoxelType VoxelType;

				VoxelSector.FACEDRAW_Operations[] ExtFaceState;
				VoxelSector.FACEDRAW_Operations[] IntFaceState;
				//if( !world ) 
				//		return;// false;

				VoxelTypeTable = world.VoxelTypeManager.VoxelTable;

				// Fetching sectors

				if( null == ( Sector[(int)VoxelSector.RelativeVoxelOrds.INCENTER] = _Sector ) ) return;
				Offset[(int)VoxelSector.RelativeVoxelOrds.INCENTER] = offset;

				Offset[(int)VoxelSector.RelativeVoxelOrds.LEFT] = offset - ( 1 * VoxelSector.ZVOXELBLOCSIZE_Y );
				Offset[(int)VoxelSector.RelativeVoxelOrds.RIGHT] = offset + ( 1 * VoxelSector.ZVOXELBLOCSIZE_Y );
				Offset[(int)VoxelSector.RelativeVoxelOrds.INFRONT] = offset + ( 1 * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y );
				Offset[(int)VoxelSector.RelativeVoxelOrds.BEHIND] = offset - ( 1 * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y );
				Offset[(int)VoxelSector.RelativeVoxelOrds.ABOVE] = offset + ( 1 );
				Offset[(int)VoxelSector.RelativeVoxelOrds.BELOW] = offset - ( 1 );

				if( 0 == ( offset & ( VoxelSector.ZVOXELBLOCMASK_X << VoxelSector.ZVOXELBLOCSHIFT_Y ) ) )
				{
					if( null == ( Sector[(int)VoxelSector.RelativeVoxelOrds.LEFT] = _Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1] ) )
						Sector[(int)VoxelSector.RelativeVoxelOrds.LEFT] = VoxelWorld.WorkingScratchSector;
					Offset[(int)VoxelSector.RelativeVoxelOrds.LEFT] += ( VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y );
				}
				else
					Sector[(int)VoxelSector.RelativeVoxelOrds.LEFT] = _Sector;

				if( 0 == ( ( offset & ( VoxelSector.ZVOXELBLOCMASK_X << VoxelSector.ZVOXELBLOCSHIFT_Y ) ) ^ ( VoxelSector.ZVOXELBLOCMASK_X << VoxelSector.ZVOXELBLOCSHIFT_Y ) ) )
				{
					if( null == ( Sector[(int)VoxelSector.RelativeVoxelOrds.RIGHT] = _Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1] ) )
						Sector[(int)VoxelSector.RelativeVoxelOrds.RIGHT] = VoxelWorld.WorkingScratchSector;
					Offset[(int)VoxelSector.RelativeVoxelOrds.RIGHT] -= ( VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y );
				}
				else
					Sector[(int)VoxelSector.RelativeVoxelOrds.RIGHT] = _Sector;

				if( 0 == ( ( offset & ( VoxelSector.ZVOXELBLOCMASK_Z << ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y ) ) ^ ( VoxelSector.ZVOXELBLOCMASK_Z << ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y ) ) ) ) )
				{
					if( null == ( Sector[(int)VoxelSector.RelativeVoxelOrds.INFRONT] = _Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.INFRONT - 1] ) )
						Sector[(int)VoxelSector.RelativeVoxelOrds.INFRONT] = VoxelWorld.WorkingScratchSector;
					Offset[(int)VoxelSector.RelativeVoxelOrds.INFRONT] -= ( VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_Z );
				}
				else
					Sector[(int)VoxelSector.RelativeVoxelOrds.INFRONT] = _Sector;

				if( 0 == ( offset & ( VoxelSector.ZVOXELBLOCMASK_Z << ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y ) ) ) )
				{
					if( null == ( Sector[(int)VoxelSector.RelativeVoxelOrds.BEHIND] = _Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND - 1] ) )
						Sector[(int)VoxelSector.RelativeVoxelOrds.BEHIND] = VoxelWorld.WorkingScratchSector;
					Offset[(int)VoxelSector.RelativeVoxelOrds.BEHIND] += ( VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_Z );
				}
				else
					Sector[(int)VoxelSector.RelativeVoxelOrds.BEHIND] = _Sector;

				if( 0 == ( ( offset & ( VoxelSector.ZVOXELBLOCMASK_Y ) ) ^ ( VoxelSector.ZVOXELBLOCMASK_Y ) ) )
				{
					if( null == ( Sector[(int)VoxelSector.RelativeVoxelOrds.ABOVE] = _Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1] ) )
						Sector[(int)VoxelSector.RelativeVoxelOrds.ABOVE] = VoxelWorld.WorkingScratchSector;
					Offset[(int)VoxelSector.RelativeVoxelOrds.ABOVE] -= ( VoxelSector.ZVOXELBLOCSIZE_Y );
				}
				else
					Sector[(int)VoxelSector.RelativeVoxelOrds.ABOVE] = _Sector;

				if( 0 == ( offset & ( VoxelSector.ZVOXELBLOCMASK_Y ) ) )
				{
					if( null == ( Sector[(int)VoxelSector.RelativeVoxelOrds.BELOW] = _Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1] ) )
						Sector[(int)VoxelSector.RelativeVoxelOrds.BELOW] = VoxelWorld.WorkingScratchSector;
					Offset[(int)VoxelSector.RelativeVoxelOrds.BELOW] += ( VoxelSector.ZVOXELBLOCSIZE_Y );
				}
				else
					Sector[(int)VoxelSector.RelativeVoxelOrds.BELOW] = _Sector;

				// Computing absolute memory pointer of blocks
				for( int i = 0; i < 7; i++ )
				{
					//Voxel_Address[i] = Sector[i].Data.Data[Offset[i]];
					Voxel = ( Sector[i].Data.Data[Offset[i]] ); VoxelType = VoxelTypeTable[Voxel];
					VoxelState[i] = (ushort)(( ( Voxel == 0 ) ? 1 : 0 )
						   | ( VoxelType.properties. Draw_FullVoxelOpacity ? 2 : 0 )
						   | ( VoxelType.properties.Draw_TransparentRendering ? 4 : 0 ));
				}

				Voxel = Sector[(int)VoxelSector.RelativeVoxelOrds.INCENTER].Data.Data[Offset[(int)VoxelSector.RelativeVoxelOrds.INCENTER]] ;

				// Storing Extension

				VoxelType = VoxelTypeTable[Voxel];

				// Storing Voxel

				if( VoxelType.properties.Is_Active ) Sector[(int)VoxelSector.RelativeVoxelOrds.INCENTER].Flag_IsActiveVoxels = true;

				// Getting case subtables.

				ExtFaceState = ExtFaceStateTable[VoxelState[(int)VoxelSector.RelativeVoxelOrds.INCENTER]];
				IntFaceState = IntFaceStateTable[VoxelState[(int)VoxelSector.RelativeVoxelOrds.INCENTER]];

				// Computing face culling for center main stored voxel.

				FaceCulling[Offset[(int)VoxelSector.RelativeVoxelOrds.INCENTER]] = (byte)(( IntFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.LEFT]] & VoxelSector.FACEDRAW_Operations.LEFT )
													   | ( IntFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.RIGHT]] & VoxelSector.FACEDRAW_Operations.RIGHT )
													   | ( IntFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.INFRONT]] & VoxelSector.FACEDRAW_Operations.AHEAD )
													   | ( IntFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.BEHIND]] & VoxelSector.FACEDRAW_Operations.BEHIND )
													   | ( IntFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.ABOVE]] & VoxelSector.FACEDRAW_Operations.ABOVE )
													   | ( IntFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.BELOW]] & VoxelSector.FACEDRAW_Operations.BELOW ))
													   ;

				// Computing face culling for nearboring voxels faces touching center voxel.
#if asdfsadf
				unchecked
				{
					FaceCulling[Offset[(int)VoxelSector.RelativeVoxelOrds.LEFT]] &= (byte)( VoxelSector.FACEDRAW_Operations.ALL_BITS ^ VoxelSector.FACEDRAW_Operations.RIGHT );
					FaceCulling[Offset[(int)VoxelSector.RelativeVoxelOrds.LEFT]] |= (byte)(ExtFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.LEFT]] & VoxelSector.FACEDRAW_Operations.RIGHT );
					FaceCulling[Offset[(int)VoxelSector.RelativeVoxelOrds.RIGHT]] &= (byte)( VoxelSector.FACEDRAW_Operations.ALL_BITS ^ VoxelSector.FACEDRAW_Operations.LEFT );
					FaceCulling[Offset[(int)VoxelSector.RelativeVoxelOrds.RIGHT]] |= (byte)( ExtFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.RIGHT]] & VoxelSector.FACEDRAW_Operations.LEFT );
					FaceCulling[Offset[(int)VoxelSector.RelativeVoxelOrds.INFRONT]] &= (byte)( VoxelSector.FACEDRAW_Operations.ALL_BITS ^ VoxelSector.FACEDRAW_Operations.BEHIND );
					FaceCulling[Offset[(int)VoxelSector.RelativeVoxelOrds.INFRONT]] |= (byte)( ExtFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.INFRONT]] & VoxelSector.FACEDRAW_Operations.BEHIND );
					FaceCulling[Offset[(int)VoxelSector.RelativeVoxelOrds.BEHIND]] &= (byte)( VoxelSector.FACEDRAW_Operations.ALL_BITS ^ VoxelSector.FACEDRAW_Operations.AHEAD );
					FaceCulling[Offset[(int)VoxelSector.RelativeVoxelOrds.BEHIND]] |= (byte)( ExtFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.BEHIND]] & VoxelSector.FACEDRAW_Operations.AHEAD );
					FaceCulling[Offset[(int)VoxelSector.RelativeVoxelOrds.ABOVE]] &= (byte)( VoxelSector.FACEDRAW_Operations.ALL_BITS ^ VoxelSector.FACEDRAW_Operations.BELOW );
					FaceCulling[Offset[(int)VoxelSector.RelativeVoxelOrds.ABOVE]] |= (byte)( ExtFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.ABOVE]] & VoxelSector.FACEDRAW_Operations.BELOW );
					FaceCulling[Offset[(int)VoxelSector.RelativeVoxelOrds.BELOW]] &= (byte)( VoxelSector.FACEDRAW_Operations.ALL_BITS ^ VoxelSector.FACEDRAW_Operations.ABOVE );
					FaceCulling[Offset[(int)VoxelSector.RelativeVoxelOrds.BELOW]] |= (byte)( ExtFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.BELOW]] & VoxelSector.FACEDRAW_Operations.ABOVE);
				}
#endif
				// printf("State[Center]:%x [Left]%x [Right]%x [INFRONT]%x [BEHIND]%x [ABOVE]%x [BELOW]%x\n",VoxelState[VoxelSector.RelativeVoxelOrds.INCENTER],VoxelState[VoxelSector.RelativeVoxelOrds.LEFT],VoxelState[VoxelSector.RelativeVoxelOrds.RIGHT],VoxelState[VoxelSector.RelativeVoxelOrds.INFRONT],VoxelState[VoxelSector.RelativeVoxelOrds.BEHIND],VoxelState[VoxelSector.RelativeVoxelOrds.ABOVE],VoxelState[VoxelSector.RelativeVoxelOrds.BELOW]);

				// Updating sector status rendering flag status
				for( int i = 0; i < 6; i++ )
				{
					Sector[i].Flag_Render_Dirty = true;
				}

			}

			void CullSingleVoxel( int x, int y, int z )
			{
				VoxelSector sector = world.FindSector( ( x >> VoxelSector.ZVOXELBLOCSHIFT_X ), y >> VoxelSector.ZVOXELBLOCSHIFT_Y, z >> VoxelSector.ZVOXELBLOCSHIFT_Z );
				uint offset = (uint)( ( ( x & VoxelSector.ZVOXELBLOCMASK_X ) << VoxelSector.ZVOXELBLOCSHIFT_Y ) 
								+ ( y & VoxelSector.ZVOXELBLOCMASK_Y ) 
								+ ( ( z & VoxelSector.ZVOXELBLOCMASK_Z ) << ( VoxelSector.ZVOXELBLOCSHIFT_Y + VoxelSector.ZVOXELBLOCSHIFT_X ) ) );
				CullSingleVoxel( sector,(uint)offset );
			}
#if asdfasdf
			bool ZVoxelCuller_Basic::Decompress_RLE( VoxelSector Sector, void* Stream )
			{
				byte* Data = (byte*)Sector.Culling;
				ZStream_SpecialRamStream* Rs = (ZStream_SpecialRamStream*)Stream;
				byte MagicToken = 0xFF;
				byte Actual;
				uint Pointer;
				ushort nRepeat;

				Pointer = 0;
				while( Pointer < Sector.DataSize )
				{
					if( !Rs.Get( Actual ) ) return ( false );
					if( Actual == MagicToken )
					{
						if( !Rs.Get( Actual ) ) return ( false );
						if( !Rs.Get( nRepeat ) ) return ( false );
						if( ( (uint)nRepeat ) > ( Sector.DataSize - Pointer ) )
						{
							return ( false );
						}

						while( nRepeat-- ) { Data[Pointer++] = Actual; }
					}
					else
					{
						Data[Pointer++] = Actual;
					}
				}

				return ( true );
			}

			void ZVoxelCuller_Basic::Compress_RLE( VoxelSector Sector, void* Stream )
			{
				ZStream_SpecialRamStream* Rs = (ZStream_SpecialRamStream*)Stream;
				byte MagicToken = 0xFF;
				byte* Data = (byte*)Sector.Culling;
				byte Last, Actual;
				uint Point = 0;
				uint SameCount = 0;
				uint i;
				bool Continue;

				Last = Data[Point++];

				Continue = true;
				while( Continue )
				{
					if( Point != Sector.DataSize ) { Actual = Data[Point++]; }
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
#endif
		}



		//VoxelWorld world;
		//BasicVoxelCuller Culler = new BasicVoxelCuller();

		internal override VoxelCuller GetCuller()
		{
			return new BasicVoxelCuller();
		}


		static VoxelSector[] SectorTable = new VoxelSector[27];
		static ushort[][] SectorDataTable = new ushort[27][];
		static ushort[][] BlocMatrix = new ushort[3][] { new ushort[9], new ushort[9], new ushort[9] };

		internal static void SectorUpdateFaceCulling( VoxelWorld world, VoxelSector Sector, bool Isolated )
		{
			VoxelType[] VoxelTypeTable;
			VoxelSector MissingSector;

			ushort[] tmpp;
			int i;

			if( Isolated ) MissingSector = VoxelWorld.WorkingEmptySector;
			else MissingSector = VoxelWorld.WorkingFullSector;

			/*
			  if (x==0 && y== 0 && z==0)
			  {
				printf("Entering..");
			  }
			*/

			// (VoxelSector.FACEDRAW_Operations.ABOVE | VoxelSector.FACEDRAW_Operations.BELOW | VoxelSector.FACEDRAW_Operations.LEFT | VoxelSector.FACEDRAW_Operations.RIGHT | VoxelSector.FACEDRAW_Operations.AHEAD | VoxelSector.FACEDRAW_Operations.BEHIND);
			for( i = 0; i < 27; i++ ) SectorTable[i] = MissingSector;
			SectorTable[0] = world.FindSector( Sector.Pos_x, Sector.Pos_y, Sector.Pos_z ); if( SectorTable[0] == null ) { return; }
			SectorTable[1] = world.FindSector( Sector.Pos_x - 1, Sector.Pos_y, Sector.Pos_z ); if( SectorTable[1] == null ) { SectorTable[1] = MissingSector; SectorTable[0].PartialCulling |= VoxelSector.FACEDRAW_Operations.LEFT; }
			SectorTable[2] = world.FindSector( Sector.Pos_x + 1, Sector.Pos_y, Sector.Pos_z ); if( SectorTable[2] == null ) { SectorTable[2] = MissingSector; SectorTable[0].PartialCulling |= VoxelSector.FACEDRAW_Operations.RIGHT; }
			SectorTable[3] = world.FindSector( Sector.Pos_x, Sector.Pos_y, Sector.Pos_z - 1 ); if( SectorTable[3] == null ) { SectorTable[3] = MissingSector; SectorTable[0].PartialCulling |= VoxelSector.FACEDRAW_Operations.AHEAD; }
			SectorTable[6] = world.FindSector( Sector.Pos_x, Sector.Pos_y, Sector.Pos_z + 1 ); if( SectorTable[6] == null ) { SectorTable[6] = MissingSector; SectorTable[0].PartialCulling |= VoxelSector.FACEDRAW_Operations.BEHIND; }
			SectorTable[9] = world.FindSector( Sector.Pos_x, Sector.Pos_y - 1, Sector.Pos_z ); if( SectorTable[9] == null ) { SectorTable[9] = MissingSector; SectorTable[0].PartialCulling |= VoxelSector.FACEDRAW_Operations.BELOW; }
			SectorTable[18] = world.FindSector( Sector.Pos_x, Sector.Pos_y + 1, Sector.Pos_z ); if( SectorTable[18] == null ) { SectorTable[18] = MissingSector; SectorTable[0].PartialCulling |= VoxelSector.FACEDRAW_Operations.ABOVE; }
			for( i = 0; i < 27; i++ ) SectorDataTable[i] = SectorTable[i].Data.Data;


			int xc, yc, zc;
			int xp, yp, zp;
			int xpp, ypp, zpp;
			VoxelSector.FACEDRAW_Operations info;
				int MainVoxelDrawInfo;

			//SectorTable[0].Flag_Void_Regular = true;
			//SectorTable[0].Flag_Void_Transparent = true;
			VoxelTypeTable = world.VoxelTypeManager.VoxelTable;

			for( xc = 0; xc < VoxelSector.ZVOXELBLOCSIZE_X; xc++ )
			{
				xp = xc + 1; xpp = xc + 2;
				for( zc = 0; zc < VoxelSector.ZVOXELBLOCSIZE_Z; zc++ )
				{
					byte[] STableX = VoxelSector.STableX;
					byte[] STableY = VoxelSector.STableY;
					byte[] STableZ = VoxelSector.STableZ;
					ushort[] OfTableX = VoxelSector.OfTableX;
					ushort[] OfTableY = VoxelSector.OfTableY;
					ushort[] OfTableZ = VoxelSector.OfTableZ;
					zp = zc + 1; zpp = zc + 2;

					// Prefetching the bloc matrix (only 2 rows)
					//    BlocMatrix[1][0] = SectorTable[(VoxelSector.STableX[xc ]+STableY[0]+STableZ[zc ])].Data[OfTableX[xc]+OfTableY[0]+OfTableZ[zc]];
					BlocMatrix[1][1] = SectorDataTable[( STableX[xp] + STableY[0] + STableZ[zc] )][OfTableX[xp] + OfTableY[0] + OfTableZ[zc]];
					//    BlocMatrix[1][2] = SectorDataTable[(STableX[xpp]+STableY[0]+STableZ[zc ])].Data;	   [OfTableX[xpp]+OfTableY[0]+OfTableZ[zc ]]
					BlocMatrix[1][3] = SectorDataTable[( STableX[xc] + STableY[0] + STableZ[zp] )][OfTableX[xc] + OfTableY[0] + OfTableZ[zp]];
					BlocMatrix[1][4] = SectorDataTable[( STableX[xp] + STableY[0] + STableZ[zp] )][OfTableX[xp] + OfTableY[0] + OfTableZ[zp]];
					BlocMatrix[1][5] = SectorDataTable[( STableX[xpp] + STableY[0] + STableZ[zp] )][OfTableX[xpp] + OfTableY[0] + OfTableZ[zp]];
					//    BlocMatrix[1][6] = SectorDataTable[(STableX[xc ]+STableY[0]+STableZ[zpp])].Data;	   [OfTableX[xc ]+OfTableY[0]+OfTableZ[zpp]]
					BlocMatrix[1][7] = SectorDataTable[( STableX[xp] + STableY[0] + STableZ[zpp] )][OfTableX[xp] + OfTableY[0] + OfTableZ[zpp]];
					//    BlocMatrix[1][8] = SectorDataTable[(STableX[xpp]+STableY[0]+STableZ[zpp])].Data;	   [OfTableX[xpp]+OfTableY[0]+OfTableZ[zpp]]

					//    BlocMatrix[2][0] = SectorDataTable[(STableX[xc ]+STableY[1]+STableZ[zc ])].Data;	   [OfTableX[xc ]+OfTableY[1]+OfTableZ[zc ]]
					BlocMatrix[2][1] = SectorDataTable[( STableX[xp] + STableY[1] + STableZ[zc] )][OfTableX[xp] + OfTableY[1] + OfTableZ[zc]];
					//    BlocMatrix[2][2] = SectorDataTable[(STableX[xpp]+STableY[1]+STableZ[zc ])].Data;	   [OfTableX[xpp]+OfTableY[1]+OfTableZ[zc ]]
					BlocMatrix[2][3] = SectorDataTable[( STableX[xc] + STableY[1] + STableZ[zp] )][OfTableX[xc] + OfTableY[1] + OfTableZ[zp]];
					BlocMatrix[2][4] = SectorDataTable[( STableX[xp] + STableY[1] + STableZ[zp] )][OfTableX[xp] + OfTableY[1] + OfTableZ[zp]];
					BlocMatrix[2][5] = SectorDataTable[( STableX[xpp] + STableY[1] + STableZ[zp] )][OfTableX[xpp] + OfTableY[1] + OfTableZ[zp]];
					//    BlocMatrix[2][6] = SectorDataTable[(STableX[xc ]+STableY[1]+STableZ[zpp])].Data;	   [OfTableX[xc ]+OfTableY[1]+OfTableZ[zpp]]
					BlocMatrix[2][7] = SectorDataTable[( STableX[xp] + STableY[1] + STableZ[zpp] )][OfTableX[xp] + OfTableY[1] + OfTableZ[zpp]];
					//    BlocMatrix[2][8] = SectorDataTable[(STableX[xpp]+STableY[1]+STableZ[zpp])].Data;	   [OfTableX[xpp]+OfTableY[1]+OfTableZ[zpp]]

					for( yc = 0; yc < VoxelSector.ZVOXELBLOCSIZE_Y; yc++ )
					{
						yp = yc + 1; ypp = yc + 2;

						// Scrolling bloc matrix by exchangingypp references.
						tmpp = BlocMatrix[0];
						BlocMatrix[0] = BlocMatrix[1];
						BlocMatrix[1] = BlocMatrix[2];
						BlocMatrix[2] = tmpp;

						// Fetching a new bloc of data slice;

						//      BlocMatrix[2][0] = SectorDataTable[(STableX[xc ]+STableY[ypp]+STableZ[zc ])].Data;    [OfTableX[xc ]+OfTableY[ypp]+OfTableZ[zc ]]
						BlocMatrix[2][1] = SectorDataTable[( STableX[xp] + STableY[ypp] + STableZ[zc] )][OfTableX[xp] + OfTableY[ypp] + OfTableZ[zc]];
						//      BlocMatrix[2][2] = SectorDataTable[(STableX[xpp]+STableY[ypp]+STableZ[zc ])].Data;	   [OfTableX[xpp]+OfTableY[ypp]+OfTableZ[zc ]]
						BlocMatrix[2][3] = SectorDataTable[( STableX[xc] + STableY[ypp] + STableZ[zp] )][OfTableX[xc] + OfTableY[ypp] + OfTableZ[zp]];
						BlocMatrix[2][4] = SectorDataTable[( STableX[xp] + STableY[ypp] + STableZ[zp] )][OfTableX[xp] + OfTableY[ypp] + OfTableZ[zp]];
						BlocMatrix[2][5] = SectorDataTable[( STableX[xpp] + STableY[ypp] + STableZ[zp] )][OfTableX[xpp] + OfTableY[ypp] + OfTableZ[zp]];
						//      BlocMatrix[2][6] = SectorDataTable[(STableX[xc ]+STableY[ypp]+STableZ[zpp])].Data;	   [OfTableX[xc ]+OfTableY[ypp]+OfTableZ[zpp]]
						BlocMatrix[2][7] = SectorDataTable[( STableX[xp] + STableY[ypp] + STableZ[zpp] )][OfTableX[xp] + OfTableY[ypp] + OfTableZ[zpp]];
						//      BlocMatrix[2][8] = SectorDataTable[(STableX[xpp]+STableY[ypp]+STableZ[zpp])].Data;	   [OfTableX[xpp]+OfTableY[ypp]+OfTableZ[zpp]]

						// Compute face culling info
						info = 0;
						if( BlocMatrix[1][4] > 0 )
						{

							MainVoxelDrawInfo = VoxelTypeTable[BlocMatrix[1][4]].properties.DrawInfo;
							VoxelSector.FACEDRAW_Operations[] SubTable = IntFaceStateTable[MainVoxelDrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS];

							info |= ( ( SubTable[VoxelTypeTable[BlocMatrix[1][1]].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS] ) & VoxelSector.FACEDRAW_Operations.AHEAD );
							info |= ( ( SubTable[VoxelTypeTable[BlocMatrix[1][7]].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS] ) & VoxelSector.FACEDRAW_Operations.BEHIND );
							info |= ( ( SubTable[VoxelTypeTable[BlocMatrix[1][3]].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS] ) & VoxelSector.FACEDRAW_Operations.LEFT );
							info |= ( ( SubTable[VoxelTypeTable[BlocMatrix[1][5]].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS] ) & VoxelSector.FACEDRAW_Operations.RIGHT );
							info |= ( ( SubTable[VoxelTypeTable[BlocMatrix[0][4]].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS] ) & VoxelSector.FACEDRAW_Operations.BELOW );
							info |= ( ( SubTable[VoxelTypeTable[BlocMatrix[2][4]].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS] ) & VoxelSector.FACEDRAW_Operations.ABOVE );
						}

						// Write face culling info to face culling table

						Sector.Culler.setFaceCulling( Sector, OfTableX[xp] + OfTableY[yp] + OfTableZ[zp], info );

					}
				}
			}

		}


		internal static VoxelSector.FACEDRAW_Operations SectorUpdateFaceCulling_Partial( VoxelWorld world, VoxelSector Sector
					, VoxelSector.FACEDRAW_Operations FacesToDraw, bool Isolated )
		{
			VoxelType[] VoxelTypeTable;
			VoxelSector MissingSector;
			VoxelSector Sector_In, Sector_Out;
			int i;
			VoxelSector.FACEDRAW_Operations CuledFaces;
			int Off_Ip, Off_In, Off_Op, Off_Out, Off_Aux;
			VoxelSector.VoxelData VoxelData_In, VoxelData_Out;
			byte[] VoxelFC_In;
			int x, y, z;
			VoxelSector.FACEDRAW_Operations FaceState;
			//extern ushort IntFaceStateTable[][8];

			x = Sector.Pos_x;
			y = Sector.Pos_y;
			z = Sector.Pos_z;

			if( Isolated ) MissingSector = VoxelWorld.WorkingEmptySector;
			else MissingSector = VoxelWorld.WorkingFullSector;

			VoxelTypeTable = world.VoxelTypeManager.VoxelTable;

			Sector_In = Sector; if( Sector_In == null ) return ( 0 );
			CuledFaces = 0;

			// Top Side

			if( ( FacesToDraw & VoxelSector.FACEDRAW_Operations.ABOVE ) != 0 )
				if( ( Sector_Out = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1] ) != null )
				{
					VoxelData_In = Sector_In.Data;
					VoxelData_Out = Sector_Out.Data;
					VoxelFC_In = Sector_In.Culler.GetData();

					for( Off_Ip = (int)VoxelSector.ZVOXELBLOCSIZE_Y - 1, Off_Op = 0; 
						Off_Ip < ( VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X ); 
						Off_Ip += (int)VoxelSector.ZVOXELBLOCSIZE_Y, 
						Off_Op += (int)VoxelSector.ZVOXELBLOCSIZE_Y ) // x (0..15)
					{
						for( Off_Aux = 0; 
							Off_Aux < ( VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_Z ); 
							Off_Aux += (int)( VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y ) ) // z (0..15)
						{
							Off_In = Off_Ip + Off_Aux;
							Off_Out = Off_Op + Off_Aux;
							FaceState = IntFaceStateTable[VoxelTypeTable[VoxelData_In.Data[Off_In]].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS][VoxelTypeTable[VoxelData_Out.Data[Off_Out]].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS];
							if( FaceState != 0 )
							{
								VoxelFC_In[Off_In] |= (byte)VoxelSector.FACEDRAW_Operations.ABOVE;
							}
							else
							{
								VoxelFC_In[Off_In] &= (byte)( ( ~(int)VoxelSector.FACEDRAW_Operations.ABOVE ) & 0xFF );
							}
						}
					}
					CuledFaces |= VoxelSector.FACEDRAW_Operations.ABOVE;
				}
			// Bottom Side

			if( ( FacesToDraw & VoxelSector.FACEDRAW_Operations.BELOW ) != 0 )
				if( ( Sector_Out = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1] ) != null )
				{
					VoxelData_In = Sector_In.Data;
					VoxelData_Out = Sector_Out.Data;
					VoxelFC_In = Sector_In.Culler.GetData();

					for( Off_Ip = 0, Off_Op = (int)VoxelSector.ZVOXELBLOCSIZE_Y - 1; Off_Ip < ( VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_Z ); Off_Ip += (int)VoxelSector.ZVOXELBLOCSIZE_Y, Off_Op += (int)VoxelSector.ZVOXELBLOCSIZE_Y ) // x (0..15)
					{
						for( Off_Aux = 0; Off_Aux < ( VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_Z ); Off_Aux += (int)( VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y ) ) // z (0..15)
						{
							Off_In = Off_Ip + Off_Aux;
							Off_Out = Off_Op + Off_Aux;
							ushort Voxel_In = VoxelData_In.Data[Off_In];
							ushort Voxel_Out = VoxelData_Out.Data[Off_Out];
							//ZVoxelType * VtIn =  VoxelTypeTable[ Voxel_In ];
							//ZVoxelType * VtOut = VoxelTypeTable[ Voxel_Out ];


							FaceState = IntFaceStateTable[VoxelTypeTable[Voxel_In].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS][VoxelTypeTable[Voxel_Out].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS];

							//FaceState = IntFaceStateTable[ VoxelTypeTable[ VoxelData_In.Data[Off_In] ].DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS ][ VoxelTypeTable[ VoxelData_Out.Data[Off_Out] ].DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS ];
							if( FaceState != 0 ) VoxelFC_In[Off_In] |= (byte)VoxelSector.FACEDRAW_Operations.BELOW; else VoxelFC_In[Off_In] &= (byte)( (int)~VoxelSector.FACEDRAW_Operations.BELOW & 0xFF );
						}
					}
					CuledFaces |= VoxelSector.FACEDRAW_Operations.BELOW;
				}
					// Left Side

					if( ( FacesToDraw & VoxelSector.FACEDRAW_Operations.LEFT ) != 0 )
				if( ( Sector_Out = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1] ) != null )
				{
					VoxelData_In = Sector_In.Data;
					VoxelData_Out = Sector_Out.Data;
					VoxelFC_In = Sector_In.Culler.GetData();
					// VoxelData_In[63]=1;
					// VoxelData_In[63 + VoxelSector.ZVOXELBLOCSIZE_Y*15 ]=14; // x
					// VoxelData_In[63 + VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X * 15] = 13; // z

					for( Off_Ip = 0, Off_Op = (int)(VoxelSector.ZVOXELBLOCSIZE_Y * ( VoxelSector.ZVOXELBLOCSIZE_X - 1 )); 
						Off_Ip < ( VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Z ); 
						Off_Ip += (int)( VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X ), 
						Off_Op += (int)( VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X ) ) // z (0..15)
					{
						for( Off_Aux = 0; Off_Aux < VoxelSector.ZVOXELBLOCSIZE_Y; Off_Aux++ ) // y (0..63)
						{
							Off_In = Off_Ip + Off_Aux;
							Off_Out = Off_Op + Off_Aux;
							//VoxelData_In[Off_In]=1; VoxelData_Out[Off_Out]=14;
							FaceState = IntFaceStateTable[VoxelTypeTable[VoxelData_In.Data[Off_In]].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS][VoxelTypeTable[VoxelData_Out.Data[Off_Out]].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS];
							if( FaceState != 0 ) VoxelFC_In[Off_In] |= (byte)VoxelSector.FACEDRAW_Operations.LEFT; else VoxelFC_In[Off_In] &= (byte)( (int)~VoxelSector.FACEDRAW_Operations.LEFT & 0xFF );


						}
					}
					CuledFaces |= VoxelSector.FACEDRAW_Operations.LEFT;
				}

			// Right Side

			if( ( FacesToDraw & VoxelSector.FACEDRAW_Operations.RIGHT ) != 0 )
				if( ( Sector_Out = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1] ) != null )
				{
					VoxelData_In = Sector_In.Data;
					VoxelData_Out = Sector_Out.Data;
					VoxelFC_In = Sector_In.Culler.GetData();

					for( Off_Ip = (int)( VoxelSector.ZVOXELBLOCSIZE_Y * ( VoxelSector.ZVOXELBLOCSIZE_X - 1 )), Off_Op = 0; 
						Off_Op < ( VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Z ); 
						Off_Ip += (int)( VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X ), Off_Op += (int)( VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X ) ) // z (0..15)
					{
						for( Off_Aux = 0; Off_Aux < VoxelSector.ZVOXELBLOCSIZE_Y; Off_Aux++ ) // y (0..63)
						{
							Off_In = Off_Ip + Off_Aux;
							Off_Out = Off_Op + Off_Aux;
							FaceState = IntFaceStateTable[VoxelTypeTable[VoxelData_In.Data[Off_In]].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS][VoxelTypeTable[VoxelData_Out.Data[Off_Out]].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS];
							if( FaceState != 0 ) VoxelFC_In[Off_In] |= (byte)VoxelSector.FACEDRAW_Operations.RIGHT; else VoxelFC_In[Off_In] &= (byte)( (int)~VoxelSector.FACEDRAW_Operations.RIGHT & 0xFF );
						}
					}
					CuledFaces |= VoxelSector.FACEDRAW_Operations.RIGHT;
				}

			// Front Side

			if( ( FacesToDraw & VoxelSector.FACEDRAW_Operations.AHEAD ) != 0 )
				if( ( Sector_Out = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD - 1] ) != null )
				{
					VoxelData_In = Sector_In.Data;
					VoxelData_Out = Sector_Out.Data;
					VoxelFC_In = Sector_In.Culler.GetData();

					for( Off_Ip = (int)( VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X * ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) ), Off_Op = 0;
						Off_Op < ( VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X );
						Off_Ip += (int)VoxelSector.ZVOXELBLOCSIZE_Y, Off_Op += (int)VoxelSector.ZVOXELBLOCSIZE_Y ) // x (0..15)
					{
						for( Off_Aux = 0; Off_Aux < VoxelSector.ZVOXELBLOCSIZE_Y; Off_Aux++ ) // y (0..63)
						{
							Off_In = Off_Ip + Off_Aux;
							Off_Out = Off_Op + Off_Aux;
							FaceState = IntFaceStateTable[VoxelTypeTable[VoxelData_In.Data[Off_In]].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS][VoxelTypeTable[VoxelData_Out.Data[Off_Out]].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS];
							if( FaceState != 0 ) VoxelFC_In[Off_In] |= (byte)VoxelSector.FACEDRAW_Operations.BEHIND; else VoxelFC_In[Off_In] &= (byte)( (int)~VoxelSector.FACEDRAW_Operations.BEHIND & 0xFF );
						}
					}
					CuledFaces |= VoxelSector.FACEDRAW_Operations.BEHIND;
				}

			// Back Side

			if( ( FacesToDraw & VoxelSector.FACEDRAW_Operations.BEHIND ) != 0 )
				if( ( Sector_Out = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND - 1] ) != null )
						{
							VoxelData_In = Sector_In.Data;
							VoxelData_Out = Sector_Out.Data;
							VoxelFC_In = Sector_In.Culler.GetData();
							for( Off_Ip = 0, Off_Op = (int)( VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X * ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) );
								Off_Ip < ( VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X );
								Off_Ip += (int)VoxelSector.ZVOXELBLOCSIZE_Y, Off_Op += (int)VoxelSector.ZVOXELBLOCSIZE_Y ) // x (0..15)
							{
								for( Off_Aux = 0; Off_Aux < VoxelSector.ZVOXELBLOCSIZE_Y; Off_Aux++ ) // y (0..63)
								{
									Off_In = Off_Ip + Off_Aux;
									Off_Out = Off_Op + Off_Aux;
									FaceState = IntFaceStateTable[VoxelTypeTable[VoxelData_In.Data[Off_In]].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS][VoxelTypeTable[VoxelData_Out.Data[Off_Out]].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS];
									if( FaceState != 0 ) VoxelFC_In[Off_In] |= (byte)VoxelSector.FACEDRAW_Operations.AHEAD; else VoxelFC_In[Off_In] &= (byte)( (int)~VoxelSector.FACEDRAW_Operations.AHEAD & 0xFF );
								}
							}
							CuledFaces |= VoxelSector.FACEDRAW_Operations.AHEAD;
						}

			//Sector.PartialCulling ^= CuledFaces & ( VoxelSector.FACEDRAW_Operations.ABOVE | VoxelSector.FACEDRAW_Operations.BELOW | VoxelSector.FACEDRAW_Operations.LEFT | VoxelSector.FACEDRAW_Operations.RIGHT | VoxelSector.FACEDRAW_Operations.AHEAD | VoxelSector.FACEDRAW_Operations.BEHIND );
			//Sector.PartialCulling &= ( VoxelSector.FACEDRAW_Operations.ABOVE | VoxelSector.FACEDRAW_Operations.BELOW | VoxelSector.FACEDRAW_Operations.LEFT | VoxelSector.FACEDRAW_Operations.RIGHT | VoxelSector.FACEDRAW_Operations.AHEAD | VoxelSector.FACEDRAW_Operations.BEHIND );
			if( CuledFaces != 0 )
			{
				//Log.log( "Sector {0} {1} {2} is dirty", Sector.Pos_x, Sector.Pos_y, Sector.Pos_z );
				Sector.Flag_Render_Dirty = true;
			}

			return ( CuledFaces );
		}


		internal static bool SetVoxel_WithCullingUpdate( VoxelWorld world, VoxelSector sector, byte x, byte y, byte z, byte VoxelValue, VoxelSector.ModifiedFieldFlags ImportanceFactor, bool CreateExtension
							, ref VoxelRef Location )
		{
			if( sector == null )
			{
				return false;
			}
			//ushort[] Voxel_Address[19];
			uint[] Offset;
			VoxelSector[] Sector;
			byte[][] FaceCulling_Address = new byte[7][];
			int[] VoxelState = new int[7];
			ushort Voxel;
			VoxelType[] VoxelTypeTable;
			VoxelType VoxelType;
			Location = new VoxelRef( world, null, x, y, z, sector, 0 );
			Location.GetVoxelRefs( out Sector, out Offset, true );

			VoxelSector.FACEDRAW_Operations[] ExtFaceState;
			VoxelSector.FACEDRAW_Operations[] IntFaceState;
			VoxelExtension OtherInfos;
			VoxelTypeTable = world.VoxelTypeManager.VoxelTable;

			// Fetching sectors

			if( null == ( Sector[(int)VoxelSector.RelativeVoxelOrds.INCENTER] ) ) return ( false );
			if( null == ( Sector[(int)VoxelSector.RelativeVoxelOrds.LEFT] ) ) Sector[(int)VoxelSector.RelativeVoxelOrds.LEFT] = VoxelWorld.WorkingScratchSector;
			if( null == ( Sector[(int)VoxelSector.RelativeVoxelOrds.RIGHT] ) ) Sector[(int)VoxelSector.RelativeVoxelOrds.RIGHT] = VoxelWorld.WorkingScratchSector;
			if( null == ( Sector[(int)VoxelSector.RelativeVoxelOrds.INFRONT] ) ) Sector[(int)VoxelSector.RelativeVoxelOrds.INFRONT] = VoxelWorld.WorkingScratchSector;
			if( null == ( Sector[(int)VoxelSector.RelativeVoxelOrds.BEHIND] ) ) Sector[(int)VoxelSector.RelativeVoxelOrds.BEHIND] = VoxelWorld.WorkingScratchSector;
			if( null == ( Sector[(int)VoxelSector.RelativeVoxelOrds.ABOVE] ) ) Sector[(int)VoxelSector.RelativeVoxelOrds.ABOVE] = VoxelWorld.WorkingScratchSector;
			if( null == ( Sector[(int)VoxelSector.RelativeVoxelOrds.BELOW] ) ) Sector[(int)VoxelSector.RelativeVoxelOrds.BELOW] = VoxelWorld.WorkingScratchSector;

			FaceCulling_Address[0] = sector.Culler.GetData();
			// Computing absolute memory pointer of blocks
			for( int i = 1; i < 7; i++ )
			{
				FaceCulling_Address[i] = Sector[i].Culler.GetData();
				Voxel = Sector[i].Data.Data[Offset[i]];
				VoxelType = VoxelTypeTable[Voxel];
				VoxelState[i] = ( ( Voxel == 0 ) ? 1 : 0 )
					   | ( VoxelType.properties.Draw_FullVoxelOpacity ? 2 : 0 )
					   | ( VoxelType.properties.Draw_TransparentRendering ? 4 : 0 );
			}
			// Delete Old voxel extended informations if any

			Voxel = Sector[(int)VoxelSector.RelativeVoxelOrds.INCENTER].Data.Data[Offset[(int)VoxelSector.RelativeVoxelOrds.INCENTER]];
			OtherInfos = Sector[(int)VoxelSector.RelativeVoxelOrds.INCENTER].Data.OtherInfos[Offset[(int)VoxelSector.RelativeVoxelOrds.INCENTER]];

			if( OtherInfos != null )
			{
				VoxelType = VoxelTypeTable[Voxel];
				if( VoxelType.properties.Is_HasAllocatedMemoryExtension ) VoxelType.DeleteVoxelExtension( OtherInfos );
			}

			// Storing Extension

			VoxelType = VoxelTypeTable[VoxelValue];
			if( CreateExtension )
			{
				Sector[(int)VoxelSector.RelativeVoxelOrds.INCENTER].Data.Data[Offset[(int)VoxelSector.RelativeVoxelOrds.INCENTER]] = 0; // Temporary set to 0 to prevent VoxelReactor for crashing while loading the wrong extension.
				Sector[(int)VoxelSector.RelativeVoxelOrds.INCENTER].Data.OtherInfos[Offset[(int)VoxelSector.RelativeVoxelOrds.INCENTER]] = VoxelType.CreateVoxelExtension();
			}

			// Storing Voxel

			Sector[(int)VoxelSector.RelativeVoxelOrds.INCENTER].Data.Data[Offset[(int)VoxelSector.RelativeVoxelOrds.INCENTER]] = VoxelValue;
			VoxelState[(int)VoxelSector.RelativeVoxelOrds.INCENTER] = ( ( VoxelValue == 0 ) ? 1 : 0 ) 
					| ( VoxelType.properties.Draw_FullVoxelOpacity ? 2 : 0 ) | ( VoxelType.properties.Draw_TransparentRendering ? 4 : 0 );


			if( VoxelTypeTable[VoxelValue].properties.Is_Active ) Sector[(int)VoxelSector.RelativeVoxelOrds.INCENTER].Flag_IsActiveVoxels = true;

			// Getting case subtables.

			ExtFaceState = ExtFaceStateTable[VoxelState[(int)VoxelSector.RelativeVoxelOrds.INCENTER]];
			IntFaceState = IntFaceStateTable[VoxelState[(int)VoxelSector.RelativeVoxelOrds.INCENTER]];

			// Computing face culling for center main stored voxel.

			FaceCulling_Address[(int)VoxelSector.RelativeVoxelOrds.INCENTER][Offset[(int)VoxelSector.RelativeVoxelOrds.INCENTER]]
				= (byte)( ( IntFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.LEFT]] & VoxelSector.FACEDRAW_Operations.LEFT )
						| ( IntFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.RIGHT]] & VoxelSector.FACEDRAW_Operations.RIGHT )
						| ( IntFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.INFRONT]] & VoxelSector.FACEDRAW_Operations.AHEAD )
						| ( IntFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.BEHIND]] & VoxelSector.FACEDRAW_Operations.BEHIND )
						| ( IntFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.ABOVE]] & VoxelSector.FACEDRAW_Operations.ABOVE )
						| ( IntFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.BELOW]] & VoxelSector.FACEDRAW_Operations.BELOW ) )
						;

			// Computing face culling for nearboring voxels faces touching center voxel.
			unchecked
			{
				FaceCulling_Address[(int)VoxelSector.RelativeVoxelOrds.LEFT][Offset[(int)VoxelSector.RelativeVoxelOrds.LEFT]] &= (byte)VoxelSector.FACEDRAW_Operations.ALL_BITS ^ (byte)VoxelSector.FACEDRAW_Operations.RIGHT;
				FaceCulling_Address[(int)VoxelSector.RelativeVoxelOrds.LEFT][Offset[(int)VoxelSector.RelativeVoxelOrds.LEFT]] |= (byte)(ExtFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.LEFT]] & VoxelSector.FACEDRAW_Operations.RIGHT );
				FaceCulling_Address[(int)VoxelSector.RelativeVoxelOrds.RIGHT][Offset[(int)VoxelSector.RelativeVoxelOrds.RIGHT]] &= (byte)VoxelSector.FACEDRAW_Operations.ALL_BITS ^ (byte)VoxelSector.FACEDRAW_Operations.LEFT;
				FaceCulling_Address[(int)VoxelSector.RelativeVoxelOrds.RIGHT][Offset[(int)VoxelSector.RelativeVoxelOrds.RIGHT]] |= (byte)( ExtFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.RIGHT]] & VoxelSector.FACEDRAW_Operations.LEFT );
				FaceCulling_Address[(int)VoxelSector.RelativeVoxelOrds.INFRONT][Offset[(int)VoxelSector.RelativeVoxelOrds.INFRONT]] &= (byte)VoxelSector.FACEDRAW_Operations.ALL_BITS ^ (byte)VoxelSector.FACEDRAW_Operations.BEHIND;
				FaceCulling_Address[(int)VoxelSector.RelativeVoxelOrds.INFRONT][Offset[(int)VoxelSector.RelativeVoxelOrds.INFRONT]] |= (byte)( ExtFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.INFRONT]] & VoxelSector.FACEDRAW_Operations.BEHIND );
				FaceCulling_Address[(int)VoxelSector.RelativeVoxelOrds.BEHIND][Offset[(int)VoxelSector.RelativeVoxelOrds.BEHIND]] &= (byte)VoxelSector.FACEDRAW_Operations.ALL_BITS ^ (byte)VoxelSector.FACEDRAW_Operations.AHEAD;
				FaceCulling_Address[(int)VoxelSector.RelativeVoxelOrds.BEHIND][Offset[(int)VoxelSector.RelativeVoxelOrds.BEHIND]] |= (byte)( ExtFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.BEHIND]] & VoxelSector.FACEDRAW_Operations.AHEAD );
				FaceCulling_Address[(int)VoxelSector.RelativeVoxelOrds.ABOVE][Offset[(int)VoxelSector.RelativeVoxelOrds.ABOVE]] &= (byte)VoxelSector.FACEDRAW_Operations.ALL_BITS ^ (byte)VoxelSector.FACEDRAW_Operations.BELOW;
				FaceCulling_Address[(int)VoxelSector.RelativeVoxelOrds.ABOVE][Offset[(int)VoxelSector.RelativeVoxelOrds.ABOVE]] |= (byte)( ExtFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.ABOVE]] & VoxelSector.FACEDRAW_Operations.BELOW );
				FaceCulling_Address[(int)VoxelSector.RelativeVoxelOrds.BELOW][Offset[(int)VoxelSector.RelativeVoxelOrds.BELOW]] &= (byte)VoxelSector.FACEDRAW_Operations.ALL_BITS ^ (byte)VoxelSector.FACEDRAW_Operations.ABOVE;
				FaceCulling_Address[(int)VoxelSector.RelativeVoxelOrds.BELOW][Offset[(int)VoxelSector.RelativeVoxelOrds.BELOW]] |= (byte)( ExtFaceState[VoxelState[(int)VoxelSector.RelativeVoxelOrds.BELOW]] & VoxelSector.FACEDRAW_Operations.ABOVE );
			}

			// printf("State[Center]:%x [Left]%x [Right]%x [INFRONT]%x [BEHIND]%x [ABOVE]%x [BELOW]%x\n",VoxelState[VoxelSector.RelativeVoxelOrds.INCENTER],VoxelState[VoxelSector.RelativeVoxelOrds.LEFT],VoxelState[VoxelSector.RelativeVoxelOrds.RIGHT],VoxelState[VoxelSector.RelativeVoxelOrds.INFRONT],VoxelState[VoxelSector.RelativeVoxelOrds.BEHIND],VoxelState[VoxelSector.RelativeVoxelOrds.ABOVE],VoxelState[VoxelSector.RelativeVoxelOrds.BELOW]);

			// Updating sector status rendering flag status
			for( int i = 0; i < 7; i++ )
			{
				for( int r = 0; r < 6; r++ )
					Sector[i].Flag_Render_Dirty = true;
			}

			Sector[(int)VoxelSector.RelativeVoxelOrds.INCENTER].Flag_IsModified |= ImportanceFactor;
			return ( true );
		}



		internal override void Render( Display display, VoxelWorld world )
		{
			HighPerfTimer Timer = new HighPerfTimer();
			HighPerfTimer Timer_SectorRefresh = new HighPerfTimer();
			long Time;
			uint RenderedSectors;
			int i;

			Timer.Start();

			Stat_RenderDrawFaces = 0;
			Stat_FaceTop = 0;
			Stat_FaceBottom = 0;
			Stat_FaceLeft = 0;
			Stat_FaceRight = 0;
			Stat_FaceFront = 0;
			Stat_FaceBack = 0;

			// Stats reset
			GameStats Stat = GameEnv.GameStat;

			if( Stat == null )
				return;

			// Precomputing values for faster math

			// Update per cycle.
			int UpdatePerCycle = 2;
			int n;

			if( Stat_RefreshWaitingSectorCount < 50 ) UpdatePerCycle = 1;
			if( Stat_RefreshWaitingSectorCount < 500 ) UpdatePerCycle = 2;
			else if( Stat.SectorRefresh_TotalTime < 32 ) UpdatePerCycle = 5;
			Stat_RefreshWaitingSectorCount = 0;

			// Stat Reset

			Stat.SectorRefresh_Count = 0;
			Stat.SectorRefresh_TotalTime = 0;
			Stat.SectorRefresh_MinTime = 0;
			Stat.SectorRefresh_MaxTime = 0;
			Stat.SectorRender_Count = 0;
			Stat.SectorRender_TotalTime = 0;
			Stat.SectorRefresh_Waiting = 0;

			// Renderwaiting system

			for( i = 0; i < 64; i++ ) RefreshToDo[i] = 0;
			for( i = 63; i > 0; i-- )
			{
				n = RefreshWaiters[i];
				if( n > UpdatePerCycle ) n = UpdatePerCycle;
				UpdatePerCycle -= n;
				RefreshToDo[i] = n;
			}
			RefreshToDo[0] = UpdatePerCycle;

			for( i = 0; i < 64; i++ ) RefreshWaiters[i] = 0;

			// Computing Frustum and Setting up Projection

			//Frustum_CullingLimit = ((Frustum_H > Frustum_V) ? Frustum_H : Frustum_V) * Optimisation_FCullingFactor;

			//glAlphaFunc(GL_GREATER, 0.2);
			//glEnable(GL_ALPHA_TEST);

			// int Start_x,Start_y,Start_z;
			int Sector_x, Sector_y, Sector_z;
			// int End_x, End_y, End_z;
			int x, y, z;

			VoxelSector Sector;
			int Priority, PriorityBoost;
			uint Sector_Refresh_Count;

			// Transforming Camera coords to sector coords. One Voxel is 256 observer units. One sector is 16x16x32.
			btVector3 origin;
			Camera.location.getOrigin( out origin );
			Sector_x = (int)( (long)origin.x >> ( world.VoxelBlockSizeBits + VoxelSector.ZVOXELBLOCSHIFT_X ));
			Sector_y = (int)( (long)origin.y >> ( world.VoxelBlockSizeBits + VoxelSector.ZVOXELBLOCSHIFT_Y ));
			Sector_z = (int)( (long)origin.z >> ( world.VoxelBlockSizeBits + VoxelSector.ZVOXELBLOCSHIFT_Z ));

			// Rendering loop

			// printf("x: %lf, y: %lf, z: %lf Pitch: %lf Yaw: %lf \n",Camera.x, Camera.y, Camera.z, Camera.Pitch, Camera.Yaw);

			// Preparation and first rendering pass

			RenderedSectors = 0;
			Sector_Refresh_Count = 0;
			int voxelSizeBits = world.VoxelBlockSizeBits;
			int voxelSize = world.VoxelBlockSize;
			SectorSphere.SphereEntry SectorSphereEntry;
			uint SectorsToProcess = SectorSphere.GetEntryCount();

			btVector3 Cv;
			btVector3 Cv2;
			Cv2.w = 0;
			Cv.w = 0;

			for( int Entry = 0; Entry < SectorsToProcess; Entry++ )
			{
				SectorSphere.GetEntry( Entry, out SectorSphereEntry );

				x = SectorSphereEntry.x + Sector_x;
				y = SectorSphereEntry.y + Sector_y;
				z = SectorSphereEntry.z + Sector_z;

				// for (x = Start_x ; x <= End_x ; x++)
				// for (y = Start_y; y <= End_y ; y++)
				// for (z = Start_z; z <= End_z ; z++)

				// try to see if sector is visible

				bool SectorVisible;

				Cv.x = (float)( (x ) << ( voxelSizeBits + VoxelSector.ZVOXELBLOCSHIFT_X ) );
				Cv.y = (float)( (y ) << ( voxelSizeBits + VoxelSector.ZVOXELBLOCSHIFT_Y ) );
				Cv.z = (float)( (z ) << ( voxelSizeBits + VoxelSector.ZVOXELBLOCSHIFT_Z ) );

				SectorVisible = false;
				Cv2.x = ( 0 * VoxelSector.ZVOXELBLOCSIZE_X * voxelSize ); Cv2.y = ( 0 * VoxelSector.ZVOXELBLOCSIZE_Y * voxelSize ); Cv2.z = ( 0 * VoxelSector.ZVOXELBLOCSIZE_Z * voxelSize );
				Cv2.Add( ref Cv, out Cv2 ); SectorVisible |= Is_PointVisible( ref Camera.location, ref Cv2 );
				Cv2.x = ( 1 * VoxelSector.ZVOXELBLOCSIZE_X * voxelSize ); Cv2.y = ( 0 * VoxelSector.ZVOXELBLOCSIZE_Y * voxelSize ); Cv2.z = ( 0 * VoxelSector.ZVOXELBLOCSIZE_Z * voxelSize ); 
				Cv2.Add( ref Cv, out Cv2 ); SectorVisible |= Is_PointVisible( ref Camera.location, ref Cv2 );
				Cv2.x = ( 1 * VoxelSector.ZVOXELBLOCSIZE_X * voxelSize ); Cv2.y = ( 0 * VoxelSector.ZVOXELBLOCSIZE_Y * voxelSize ); Cv2.z = ( 1 * VoxelSector.ZVOXELBLOCSIZE_Z * voxelSize ); 
				Cv2.Add( ref Cv, out Cv2 ); SectorVisible |= Is_PointVisible( ref Camera.location, ref Cv2 );
				Cv2.x = ( 0 * VoxelSector.ZVOXELBLOCSIZE_X * voxelSize ); Cv2.y = ( 0 * VoxelSector.ZVOXELBLOCSIZE_Y * voxelSize ); Cv2.z = ( 1 * VoxelSector.ZVOXELBLOCSIZE_Z * voxelSize ); 
				Cv2.Add( ref Cv, out Cv2 ); SectorVisible |= Is_PointVisible( ref Camera.location, ref Cv2 );
				Cv2.x = ( 0 * VoxelSector.ZVOXELBLOCSIZE_X * voxelSize ); Cv2.y = ( 1 * VoxelSector.ZVOXELBLOCSIZE_Y * voxelSize ); Cv2.z = ( 0 * VoxelSector.ZVOXELBLOCSIZE_Z * voxelSize ); 
				Cv2.Add( ref Cv, out Cv2 ); SectorVisible |= Is_PointVisible( ref Camera.location, ref Cv2 );
				Cv2.x = ( 1 * VoxelSector.ZVOXELBLOCSIZE_X * voxelSize ); Cv2.y = ( 1 * VoxelSector.ZVOXELBLOCSIZE_Y * voxelSize ); Cv2.z = ( 0 * VoxelSector.ZVOXELBLOCSIZE_Z * voxelSize ); 
				Cv2.Add( ref Cv, out Cv2 ); SectorVisible |= Is_PointVisible( ref Camera.location, ref Cv2 );
				Cv2.x = ( 1 * VoxelSector.ZVOXELBLOCSIZE_X * voxelSize ); Cv2.y = ( 1 * VoxelSector.ZVOXELBLOCSIZE_Y * voxelSize ); Cv2.z = ( 1 * VoxelSector.ZVOXELBLOCSIZE_Z * voxelSize ); 
				Cv2.Add( ref Cv, out Cv2 ); SectorVisible |= Is_PointVisible( ref Camera.location, ref Cv2 );
				Cv2.x = ( 0 * VoxelSector.ZVOXELBLOCSIZE_X * voxelSize ); Cv2.y = ( 1 * VoxelSector.ZVOXELBLOCSIZE_Y * voxelSize ); Cv2.z = ( 1 * VoxelSector.ZVOXELBLOCSIZE_Z * voxelSize ); 
				Cv2.Add( ref Cv, out Cv2 ); SectorVisible |= Is_PointVisible( ref Camera.location, ref Cv2 );

				Sector = world.FindSector( x, y, z );
				Priority = RadiusZones.GetZone( x - Sector_x, y - Sector_y, z - Sector_z );
				PriorityBoost = ( SectorVisible && Priority <= 2 ) ? 1 : 0;
				// Go = true;

				if( Sector != null )
				{
					Sector.Flag_IsVisibleAtLastRendering = SectorVisible || Priority >= 3;
					// Display lists preparation
					if( Sector.Flag_Render_Dirty && GameEnv.Enable_NewSectorRendering )
					{
						if( Sector.Flag_IsDebug )
						{
							//printf( "Debug\n" );
							//Sector.Flag_IsDebug = false;
						}

						// if (Sector_Refresh_Count < 5 || Priority==4)
						if( ( RefreshToDo[Sector.RefreshWaitCount] != 0 ) || Sector.Flag_HighPriorityRefresh )
						{

							if ( VoxelGlobalSettings.COMPILEOPTION_FINETIMINGTRACKING )
								Timer_SectorRefresh.Start();

							RefreshToDo[Sector.RefreshWaitCount]--;
							Sector.Flag_HighPriorityRefresh = false;

							//Log.log( "Draw sector geometry {0} {1} {2}", Sector.Pos_x, Sector.Pos_y, Sector.Pos_z );
							if( Sector.Flag_NeedSortedRendering ) MakeSectorRenderingData_Sorted( Sector );
							else MakeSectorRenderingData( Sector );

							Sector_Refresh_Count++;
							Sector.RefreshWaitCount = 0;
							Stat.SectorRefresh_Count++;

							if( VoxelGlobalSettings.COMPILEOPTION_FINETIMINGTRACKING )
							{
								Timer_SectorRefresh.End(); Time = Timer_SectorRefresh.GetResult(); Stat.SectorRefresh_TotalTime += (uint)Time;
								if( Time < Stat.SectorRefresh_MinTime ) Stat.SectorRefresh_MinTime = (uint)Time;
								if( Time > Stat.SectorRefresh_MaxTime ) Stat.SectorRefresh_MaxTime = (uint)Time;
							}
						}
						else
						{
							Sector.RefreshWaitCount++;
							if( Sector.RefreshWaitCount > 31 ) Sector.RefreshWaitCount = 31;
							if( Priority == 4 ) Sector.RefreshWaitCount++;
							RefreshWaiters[Sector.RefreshWaitCount]++;
							Stat_RefreshWaitingSectorCount++;
							Stat.SectorRefresh_Waiting++;
						}

					}


					// Rendering first pass
					if( Sector.Flag_IsVisibleAtLastRendering
						&& ( !Sector.Flag_Void_Regular )
						)
					{
						if( VoxelGlobalSettings.COMPILEOPTION_FINETIMINGTRACKING )
							Timer_SectorRefresh.Start();

						Sector.geometry.DrawBuffer( false, world.TextureAtlas.OpenGl_TextureRef );
						//glCallList( ( (ZRender_Interface_displaydata*)Sector.DisplayData ).DisplayList_Regular[current_gl_camera] );

						Stat.SectorRender_Count++; RenderedSectors++;

						if( VoxelGlobalSettings.COMPILEOPTION_FINETIMINGTRACKING ) {
							Timer_SectorRefresh.End(); Time = Timer_SectorRefresh.GetResult(); Stat.SectorRender_TotalTime += (uint)Time;
						}
					}
				}
				else
				{
					if( GameEnv.Enable_LoadNewSector )
						world.RequestSector( x, y, z, Priority + PriorityBoost );
					GL.Disable( EnableCap.DepthTest );
					if( SectorVisible )
						Render_EmptySector( display, world, x, y, z, 1.0f, 0.3f, 0.1f );
					GL.Enable( EnableCap.DepthTest );
					//return;

				}

			}

			// Second pass rendering
			//GL.Disable( EnableCap.DepthTest );
			//GL.DepthMask( false );
			//glDepthMask( GL_FALSE );
			GL.AlphaFunc( AlphaFunction.Greater, 0.2f );

			GL.Enable( EnableCap.AlphaTest );
			GL.Enable( EnableCap.Blend );
			GL.BlendFunc( BlendingFactorSrc.SrcAlpha, BlendingFactorDest.OneMinusSrcAlpha );

			for( int Entry = 0; Entry < SectorsToProcess; Entry++ )
			{
				SectorSphere.GetEntry( Entry, out SectorSphereEntry );

				x = SectorSphereEntry.x + Sector_x;
				y = SectorSphereEntry.y + Sector_y;
				z = SectorSphereEntry.z + Sector_z;

				Sector = world.FindSector( x, y, z );
				// printf("Sector : %ld %ld %ld %lu\n", x, y, z, (uint)(Sector != 0));9
				if( Sector != null )
				{
					if( Sector.Flag_IsVisibleAtLastRendering
					   && ( !Sector.Flag_Void_Transparent )
					   )
					{
						if( VoxelGlobalSettings.COMPILEOPTION_FINETIMINGTRACKING )
							Timer_SectorRefresh.Start();

						Sector.geometry.DrawBuffer( true, world.TextureAtlas.OpenGl_TextureRef );
						//glCallList( ( (ZRender_Interface_displaydata*)Sector.DisplayData ).DisplayList_Transparent[current_gl_camera] );
						Stat.SectorRender_Count++;

						if( VoxelGlobalSettings.COMPILEOPTION_FINETIMINGTRACKING )
						{
							Timer_SectorRefresh.End(); Time = Timer_SectorRefresh.GetResult();
							Stat.SectorRender_TotalTime += (uint)Time;
						}
					}

				}
			}
			GL.Enable( EnableCap.DepthTest );
			GL.Disable( EnableCap.Blend );
			//GL.DepthMask( true );
			//glDepthMask( GL_TRUE );

#if asdfasdf
			// ***************************
			// Cube designation
			// ***************************

			ZRayCast_in In;

			In.Camera = Camera;
			In.MaxCubeIterations = 150;
			In.PlaneCubeDiff = 100;
			In.MaxDetectionDistance = 30000.0;

			//   if (World.RayCast( &In, PointedVoxel ))
			//   {
			// Render_VoxelSelector( &PointedVoxel.PointedVoxel, 1.0,1.0,1.0 );
			//Render_VoxelSelector( &PointedVoxel.PredPointedVoxel, 1.0, 0.0, 0.0);
			//    }

			// Debug ****************************************************

			ZVector3d Norm, Tmp;
			Norm.x = 0; Norm.y = 0; Norm.z = 1;
			Camera.orientation.ApplyRotation( Tmp, Norm );
			// X axis rotation
			//Tmp.y = Norm.y * cos(-Camera.Pitch/57.295779513) - Norm.z * sin(-Camera.Pitch/57.295779513);
			//Tmp.z = Norm.y * sin(-Camera.Pitch/57.295779513) + Norm.z * cos(-Camera.Pitch/57.295779513);
			//Norm.y = Tmp.y; Norm.z = Tmp.z;
			// Y axis rotation
			//Tmp.x = Norm.z*sin(Camera.Yaw/57.295779513) + Norm.x * cos(Camera.Yaw/57.295779513);
			//Tmp.z = Norm.z*cos(Camera.Yaw/57.295779513) - Norm.x * sin(Camera.Yaw/57.295779513);
			//Norm.x = Tmp.x; Norm.z = Tmp.z;
			//Norm.y = Tmp.y;
			// printf("Norm(%lf %lf %lf)\n",Norm.x,Norm.y,Norm.z);

			In.MaxCubeIterations = 150;
			In.MaxDetectionDistance = 1536;//1000000.0;

			//ZVector3d CamPoint(Camera.x(),Camera.y(),Camera.z());
			ZVector3d Zp;
			Zp = PointedVoxel.CollisionPoint; Zp.y = PointedVoxel.CollisionPoint.y + 100.0;

			if( World.RayCast_Vector( Camera.orientation, Tmp, &In, PointedVoxel ) )
			{
				if( PointedVoxel.CollisionDistance < In.MaxDetectionDistance )
				{
					PointedVoxel.Collided = true;
					if( BvProp_DisplayVoxelSelector )
						Render_VoxelSelector( &PointedVoxel.PointedVoxel, 1.0, 1.0, 1.0 );
				}
				else
				{
					ZVector3d a = Camera.orientation.origin() +
						Camera.orientation.z_axis() * ( VoxelGlobalSettings.WorldVoxelBlockSize * ( Actor.VoxelSelectDistance ) );

					ZVoxelRef v;
					if( World.GetVoxelRefPlayerCoord( v, a.x, a.y, a.z ) )
					{
						//World.RayCast_Vector(Camera.orientation, Tmp, &In, PointedVoxel);
						PointedVoxel.PredPointedVoxel.x = v.x + ( v.Sector.Pos_x << VoxelSector.ZVOXELBLOCSHIFT_X );
						PointedVoxel.PredPointedVoxel.y = v.y + ( v.Sector.Pos_y << VoxelSector.ZVOXELBLOCSHIFT_Y );
						PointedVoxel.PredPointedVoxel.z = v.z + ( v.Sector.Pos_z << VoxelSector.ZVOXELBLOCSHIFT_Z );
						PointedVoxel.Collided = true;
					}
					if( BvProp_DisplayVoxelSelector )
						Render_VoxelSelector( &PointedVoxel.PredPointedVoxel, 0.2, 1.0, 0.1 );
				}
			}


			// ***************************
			// RÃ©ticule
			// ***************************
			DrawReticule();
			// Voile colorÃ©
			DrawColorOverlay();
#endif
			Timer.End();

			/*printf("Frame Time : %lu Rend Sects: %lu Draw Faces :%lu Top:%lu Bot:%lu Le:%lu Ri:%lu Front:%lu Back:%lu\n",Timer.GetResult(), RenderedSectors, Stat_RenderDrawFaces, Stat_FaceTop, Stat_FaceBottom,
				   Stat_FaceLeft,Stat_FaceRight,Stat_FaceFront,Stat_FaceBack);*/

			//printf("RenderedSectors : %lu\n",RenderedSectors);
			//SDL_GL_SwapBuffers( );
		}


		void MakeSectorRenderingData( VoxelSector Sector )
		{
			Color face = Color.Black, edge = Color.Red;
			short power = 400;
			int x, y, z;
			VoxelSector.FACEDRAW_Operations info;
			ushort cube, prevcube;
			/* build sector geometry */

			uint Offset;
			float cubx, cuby, cubz;
			int Sector_Display_x, Sector_Display_y, Sector_Display_z;
			uint Pass;
			bool Draw;
			VoxelTypeManager VoxelTypeManager = Sector.VoxelTypeManager;
			VoxelType[] VoxelTypeTable = VoxelTypeManager.VoxelTable;
			Vector3 P0, P1, P2, P3, P4, P5, P6, P7;

			//Log.log( "Building sector {0} {1} {2}", Sector.Pos_x, Sector.Pos_y, Sector.Pos_z );
			// Display list creation or reuse.

			if( Sector.Flag_Render_Dirty )
			{
				VoxelGeometry geometry = Sector.geometry;
				float voxelSize = Sector.world.VoxelBlockSize;
				byte[] FaceCulling = ( Sector.Culler as BasicVoxelCuller ).FaceCulling;
				Sector_Display_x = (int)(Sector.Pos_x * Sector.Size_x * voxelSize );
				Sector_Display_y = (int)( Sector.Pos_y * Sector.Size_y * voxelSize );
				Sector_Display_z = (int)( Sector.Pos_z * Sector.Size_z * voxelSize );

				Sector.Flag_Void_Regular = true;
				Sector.Flag_Void_Transparent = true;

				for( Pass = 0; Pass < 2; Pass++ )
				{
					switch( Pass )
					{
						case 0: geometry.SetSolid(); break;// glNewList( DisplayData.DisplayList_Regular[current_gl_camera], GL_COMPILE ); break;
						case 1: geometry.SetTransparent(); break;//glNewList( DisplayData.DisplayList_Transparent[current_gl_camera], GL_COMPILE ); break;
					}
					prevcube = 0;
					for( z = 0; z < Sector.Size_z; z++ )
					{
						for( x = 0; x < Sector.Size_x; x++ )
						{
							for( y = 0; y < Sector.Size_y; y++ )
							{
								Offset = (uint)(y + ( x * Sector.Size_y ) + ( z * ( Sector.Size_y * Sector.Size_x ) ));
								cube = Sector.Data.Data[Offset];
								info = (VoxelSector.FACEDRAW_Operations)FaceCulling[Offset];


								if( cube > 0 && info != VoxelSector.FACEDRAW_Operations.NONE )
								{
									switch( Pass )
									{
										default:
										case 0:
											if( VoxelTypeTable[cube].properties.Draw_TransparentRendering ) { Draw = false; Sector.Flag_Void_Transparent = false; }
											else { Draw = true; Sector.Flag_Void_Regular = false; }
											break;
										case 1: Draw = ( VoxelTypeTable[cube].properties.Draw_TransparentRendering ) ? true : false; break;
									}
								}
								else Draw = false;

								if( Draw )
								{
									Box2D box = VoxelTypeTable[cube].TextureCoords;
									bool face_is_shaded;
									if( ( VoxelTypeTable[cube].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_SHADER ) != 0 )
									{
										face = VoxelTypeTable[cube].properties.FaceColor;
										edge = VoxelTypeTable[cube].properties.EdgeColor;
										power = VoxelTypeTable[cube].properties.EdgePower;
										face_is_shaded = true;
									}
									else
										face_is_shaded = false;
									prevcube = cube;

									cubx = (float)( x * Sector.world.VoxelBlockSize + Sector_Display_x );
									cuby = (float)( y * Sector.world.VoxelBlockSize + Sector_Display_y );
									cubz = (float)( z * Sector.world.VoxelBlockSize + Sector_Display_z );

									if( 0 != ( VoxelTypeTable[cube].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_SPECIALRENDERING ))
									{ VoxelTypeTable[cube].SpecialRender( cubx, cuby, cubz ); continue; }

									P0.X = cubx; P0.Y = cuby; P0.Z = cubz;
									P1.X = cubx + voxelSize; P1.Y = cuby; P1.Z = cubz;
									P2.X = cubx + voxelSize; P2.Y = cuby; P2.Z = cubz + voxelSize;
									P3.X = cubx; P3.Y = cuby; P3.Z = cubz + voxelSize;
									P4.X = cubx; P4.Y = cuby + voxelSize; P4.Z = cubz;
									P5.X = cubx + voxelSize; P5.Y = cuby + voxelSize; P5.Z = cubz;
									P6.X = cubx + voxelSize; P6.Y = cuby + voxelSize; P6.Z = cubz + voxelSize;
									P7.X = cubx; P7.Y = cuby + voxelSize; P7.Z = cubz + voxelSize;

									//Left
									if( ( info & VoxelSector.FACEDRAW_Operations.LEFT ) != 0 )
									{
										Stat_RenderDrawFaces++;
										Stat_FaceLeft++;
										//Log.log( "Add {0} {1} {2} {3}", P4 , P0, P3, P7 );
										if( face_is_shaded )
											geometry.AddQuad( ref P0, ref P3, ref P4, ref P7, face, edge, power  );
										else
											geometry.AddQuad( ref P0, ref P3, ref P4, ref P7, ref VoxelTypeTable[cube].TextureCoords );
									}

									// Right
									if( ( info & VoxelSector.FACEDRAW_Operations.RIGHT ) != 0 )
									{
										Stat_RenderDrawFaces++;
										Stat_FaceRight++;
										//Log.log( "Add {0} {1} {2} {3}", P5, P6, P2, P1 );
										if( face_is_shaded )
											geometry.AddQuad( ref P1, ref P2, ref P5, ref P6, face, edge, power );
										else
											geometry.AddQuad( ref P1, ref P2, ref P5, ref P6, ref VoxelTypeTable[cube].TextureCoords );
									}
									//Front
									if( ( info & VoxelSector.FACEDRAW_Operations.AHEAD ) != 0 )
									{
										Stat_RenderDrawFaces++;
										Stat_FaceFront++;
										//Log.log( "Add {0} {1} {2} {3}", P0, P4, P5, P1 );
										if( face_is_shaded )
											geometry.AddQuad( ref P0, ref P1, ref P4, ref P5, face, edge, power );
										else
											geometry.AddQuad( ref P0, ref P1, ref P4, ref P5, ref VoxelTypeTable[cube].TextureCoords );
									}

									//Back
									if( ( info & VoxelSector.FACEDRAW_Operations.BEHIND ) != 0 )
									{
										Stat_RenderDrawFaces++;
										Stat_FaceBack++;
										//Log.log( "Add {0} {1} {2} {3}", P2, P6, P3, P7 );
										if( face_is_shaded )
											geometry.AddQuad( ref P2, ref P3, ref P6, ref P7, face, edge, power );
										else
											geometry.AddQuad( ref P2, ref P3, ref P6, ref P7, ref VoxelTypeTable[cube].TextureCoords );
									}

									// Top
									if( ( info & VoxelSector.FACEDRAW_Operations.ABOVE ) != 0 )
									{
										Stat_RenderDrawFaces++;
										Stat_FaceTop++;
										//Log.log( "Add {0} {1} {2} {3}", P4, P7, P5, P6 );
										if( face_is_shaded )
											geometry.AddQuad( ref P4, ref P7, ref P5, ref P6, face, edge, power );
										else
											geometry.AddQuad( ref P4, ref P7, ref P5, ref P6, ref VoxelTypeTable[cube].TextureCoords );
									}

									// Bottom
									if( ( info & VoxelSector.FACEDRAW_Operations.BELOW ) != 0 )
									{
										Stat_RenderDrawFaces++;
										Stat_FaceBottom++;
										//Log.log( "Add {0} {1} {2} {3}", P0, P1, P3, P2 );
										if( face_is_shaded )
											geometry.AddQuad( ref P0, ref P2, ref P1, ref P3, face, edge, power );
										else
											geometry.AddQuad( ref P0, ref P2, ref P1, ref P3, ref VoxelTypeTable[cube].TextureCoords );
									}
								}
							}
						}
					}
					// if in the first pass, the sector has no transparent block, the second pass is cancelled.
					if( Sector.Flag_Void_Transparent ) break;
				}
				geometry.SetSolid();
				Sector.Flag_Render_Dirty = false;
			}
		}




		void MakeSectorRenderingData_Sorted( VoxelSector Sector )
		{
#if asdfasdfasdf
			int x, y, z;
			uint info, i;
			ushort VoxelType, prevVoxelType;
			// uint Offset;
			float cubx, cuby, cubz;
			int Sector_Display_x, Sector_Display_y, Sector_Display_z;
			ZRender_Interface_displaydata* DisplayData;
			uint Pass;
			ZVoxelType** VoxelTypeTable = VoxelTypeManager.VoxelTable;
			ZVector3f P0, P1, P2, P3, P4, P5, P6, P7;

			ZRender_Sorter::RenderBucket* RenderBucket;

			// Set flags

			Sector.Flag_Void_Regular = true;
			Sector.Flag_Void_Transparent = true;
			Sector.Flag_Render_Dirty[current_gl_camera] = false;

			// Render sorter action

			RenderSorter.ProcessSector( Sector );
			if( !RenderSorter.GetBucketCount() )
				return;

			// Check what blocktypes

			RenderSorter.Rendering_Start();
			for( i = 0; i < RenderSorter.GetBucketCount(); i++ )
			{
				VoxelType = RenderSorter.Rendering_GetNewBucket().VoxelType;

				if( VoxelTypeTable[VoxelType].Draw_TransparentRendering ) Sector.Flag_Void_Transparent = false;
				else Sector.Flag_Void_Regular = false;
			}

			// Display list creation or reuse.

			if( Sector.DisplayData == 0 ) { Sector.DisplayData = new ZRender_Interface_displaydata; }
			DisplayData = (ZRender_Interface_displaydata*)Sector.DisplayData;
			if( ( !Sector.Flag_Void_Regular ) && ( DisplayData.DisplayList_Regular[current_gl_camera] == 0 ) ) DisplayData.DisplayList_Regular[current_gl_camera] = glGenLists( 1 );
			if( ( !Sector.Flag_Void_Transparent ) && ( DisplayData.DisplayList_Transparent[current_gl_camera] == 0 ) ) DisplayData.DisplayList_Transparent[current_gl_camera] = glGenLists( 1 );

			// Computing Sector Display coordinates;

			Sector_Display_x = ( Sector.Pos_x * Sector.Size_x ) << VoxelGlobalSettings.WorldVoxelBlockSizeBits;
			Sector_Display_y = ( Sector.Pos_y * Sector.Size_y ) << VoxelGlobalSettings.WorldVoxelBlockSizeBits;
			Sector_Display_z = ( Sector.Pos_z * Sector.Size_z ) << VoxelGlobalSettings.WorldVoxelBlockSizeBits;

			for( Pass = 0; Pass < 2; Pass++ )
			{
				if( !Pass ) { if( Sector.Flag_Void_Regular ) continue; glNewList( DisplayData.DisplayList_Regular[current_gl_camera], GL_COMPILE ); }
				else { if( Sector.Flag_Void_Transparent ) continue; glNewList( DisplayData.DisplayList_Transparent[current_gl_camera], GL_COMPILE ); }

				prevVoxelType = 0;

				RenderSorter.Rendering_Start();

				while( ( RenderBucket = RenderSorter.Rendering_GetNewBucket() ) )
				{
					VoxelType = RenderBucket.VoxelType;

					// Is it the right voxel transparency type for that rendering pass ?

					if( ( Pass > 0 ) != VoxelTypeTable[VoxelType].Draw_TransparentRendering ) continue;

					// Render one RenderBucket.

					for( i = 0; i < RenderBucket.VoxelCount; i++ )
					{
						register uint64_t Pck;

						// Gettint Voxel Informations from the table.

						Pck = RenderBucket.RenderTable[i].PackedInfos;

						// Unpacking voxel infos

						info = Pck & ( ( 1 << ( 64 - ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y + VoxelSector.ZVOXELBLOCSHIFT_Z ) ) ) - 1 );
						z = ( Pck >> ( 64 - ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y + VoxelSector.ZVOXELBLOCSHIFT_Z ) ) ) & VoxelSector.ZVOXELBLOCMASK_Z;
						y = ( Pck >> ( 64 - ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y ) ) ) & VoxelSector.ZVOXELBLOCMASK_Y;
						x = ( Pck >> ( 64 - ( VoxelSector.ZVOXELBLOCSHIFT_X ) ) ) & VoxelSector.ZVOXELBLOCMASK_X;
						//info = Pck & 0xFF;
						//z    = Pck >> 8 & 0xFF;
						//y    = Pck >> 16 & 0xFF;
						//x    = Pck >> 24 & 0xFF;

						// Offset = y + ( x << VoxelSector.ZVOXELBLOCSHIFT_Y )+ (z << (VoxelSector.ZVOXELBLOCSHIFT_Y + VoxelSector.ZVOXELBLOCSHIFT_X));

						// glTexEnvf(0x8500 /* TEXTURE_FILTER_CONTROL_EXT */, 0x8501 /* TEXTURE_LOD_BIAS_EXT */,VoxelTypeManager.VoxelTable[VoxelType].TextureLodBias);
						if( VoxelType != prevVoxelType ) glBindTexture( GL_TEXTURE_2D, VoxelTypeManager.VoxelTable[VoxelType].OpenGl_TextureRef[current_gl_camera] );
						{
							int x; if( x = glGetError() )
								printf( "glerror(%d): %d\n", __LINE__, x );
						}
						prevVoxelType = VoxelType;
						cubx = (float)( x * VoxelGlobalSettings.WorldVoxelBlockSize + Sector_Display_x );
						cuby = (float)( y * VoxelGlobalSettings.WorldVoxelBlockSize + Sector_Display_y );
						cubz = (float)( z * VoxelGlobalSettings.WorldVoxelBlockSize + Sector_Display_z );

						if( VoxelTypeTable[VoxelType].DrawInfo & VoxelSector.RelativeVoxelOrds.DRAWINFO_SPECIALRENDERING ) { VoxelTypeTable[VoxelType].SpecialRender( cubx, cuby, cubz ); continue; }

						P0.x = cubx; P0.y = cuby; P0.z = cubz;
						P1.x = cubx + VoxelGlobalSettings.WorldVoxelBlockSize; P1.y = cuby; P1.z = cubz;
						P2.x = cubx + VoxelGlobalSettings.WorldVoxelBlockSize; P2.y = cuby; P2.z = cubz + VoxelGlobalSettings.WorldVoxelBlockSize;
						P3.x = cubx; P3.y = cuby; P3.z = cubz + VoxelGlobalSettings.WorldVoxelBlockSize;
						P4.x = cubx; P4.y = cuby + VoxelGlobalSettings.WorldVoxelBlockSize; P4.z = cubz;
						P5.x = cubx + VoxelGlobalSettings.WorldVoxelBlockSize; P5.y = cuby + VoxelGlobalSettings.WorldVoxelBlockSize; P5.z = cubz;
						P6.x = cubx + VoxelGlobalSettings.WorldVoxelBlockSize; P6.y = cuby + VoxelGlobalSettings.WorldVoxelBlockSize; P6.z = cubz + VoxelGlobalSettings.WorldVoxelBlockSize;
						P7.x = cubx; P7.y = cuby + VoxelGlobalSettings.WorldVoxelBlockSize; P7.z = cubz + VoxelGlobalSettings.WorldVoxelBlockSize;

						//Left
						if( info & VoxelSector.FACEDRAW_Operations.LEFT )
						{
							Stat_RenderDrawFaces++;
							Stat_FaceLeft++;
							glBegin( GL_TRIANGLES );
							glTexCoord2f( 0.25, 0.25 ); glVertex3f( P4.x, P4.y, P4.z );
							glTexCoord2f( 0.25, 0.0 ); glVertex3f( P0.x, P0.y, P0.z );
							glTexCoord2f( 0.50, 0.0 ); glVertex3f( P3.x, P3.y, P3.z );
							glTexCoord2f( 0.50, 0.0 ); glVertex3f( P3.x, P3.y, P3.z );
							glTexCoord2f( 0.50, 0.25 ); glVertex3f( P7.x, P7.y, P7.z );
							glTexCoord2f( 0.25, 0.25 ); glVertex3f( P4.x, P4.y, P4.z );
							glEnd();
						}
						{
							int x; if( x = glGetError() )
								printf( "glerror(%d): %d\n", __LINE__, x );
						}

						// Right
						if( info & VoxelSector.FACEDRAW_Operations.RIGHT )
						{
							Stat_RenderDrawFaces++;
							Stat_FaceRight++;
							glBegin( GL_TRIANGLES );
							glTexCoord2f( 0.25, 0.50 ); glVertex3f( P5.x, P5.y, P5.z );
							glTexCoord2f( 0.50, 0.50 ); glVertex3f( P6.x, P6.y, P6.z );
							glTexCoord2f( 0.50, 0.75 ); glVertex3f( P2.x, P2.y, P2.z );
							glTexCoord2f( 0.50, 0.75 ); glVertex3f( P2.x, P2.y, P2.z );
							glTexCoord2f( 0.25, 0.75 ); glVertex3f( P1.x, P1.y, P1.z );
							glTexCoord2f( 0.25, 0.50 ); glVertex3f( P5.x, P5.y, P5.z );
							glEnd();
						}
						{
							int x; if( x = glGetError() )
								printf( "glerror(%d): %d\n", __LINE__, x );
						}

						//Front
						if( info & VoxelSector.FACEDRAW_Operations.AHEAD )
						{
							Stat_RenderDrawFaces++;
							Stat_FaceFront++;
							glBegin( GL_TRIANGLES );
							glTexCoord2f( 0.0, 0.25 ); glVertex3f( P0.x, P0.y, P0.z );
							glTexCoord2f( 0.25, 0.25 ); glVertex3f( P4.x, P4.y, P4.z );
							glTexCoord2f( 0.25, 0.50 ); glVertex3f( P5.x, P5.y, P5.z );
							glTexCoord2f( 0.25, 0.50 ); glVertex3f( P5.x, P5.y, P5.z );
							glTexCoord2f( 0.0, 0.50 ); glVertex3f( P1.x, P1.y, P1.z );
							glTexCoord2f( 0.0, 0.25 ); glVertex3f( P0.x, P0.y, P0.z );
							glEnd();
						}
						{
							int x; if( x = glGetError() )
								printf( "glerror(%d): %d\n", __LINE__, x );
						}

						//Back
						if( info & VoxelSector.FACEDRAW_Operations.BEHIND )
						{
							Stat_RenderDrawFaces++;
							Stat_FaceBack++;
							glBegin( GL_TRIANGLES );
							glTexCoord2f( 0.75, 0.50 ); glVertex3f( P2.x, P2.y, P2.z );
							glTexCoord2f( 0.50, 0.50 ); glVertex3f( P6.x, P6.y, P6.z );
							glTexCoord2f( 0.75, 0.25 ); glVertex3f( P3.x, P3.y, P3.z );
							glTexCoord2f( 0.75, 0.25 ); glVertex3f( P3.x, P3.y, P3.z );
							glTexCoord2f( 0.50, 0.50 ); glVertex3f( P6.x, P6.y, P6.z );
							glTexCoord2f( 0.50, 0.25 ); glVertex3f( P7.x, P7.y, P7.z );
							glEnd();
						}
						{
							int x; if( x = glGetError() )
								printf( "glerror(%d): %d\n", __LINE__, x );
						}

						// Top
						if( info & VoxelSector.FACEDRAW_Operations.ABOVE )
						{
							Stat_RenderDrawFaces++;
							Stat_FaceTop++;
							glBegin( GL_TRIANGLES );
							glTexCoord2f( 0.25, 0.25 ); glVertex3f( P4.x, P4.y, P4.z );
							glTexCoord2f( 0.50, 0.25 ); glVertex3f( P7.x, P7.y, P7.z );
							glTexCoord2f( 0.25, 0.50 ); glVertex3f( P5.x, P5.y, P5.z );
							glTexCoord2f( 0.25, 0.50 ); glVertex3f( P5.x, P5.y, P5.z );
							glTexCoord2f( 0.50, 0.25 ); glVertex3f( P7.x, P7.y, P7.z );
							glTexCoord2f( 0.50, 0.50 ); glVertex3f( P6.x, P6.y, P6.z );
							glEnd();
						}
						{
							int x; if( x = glGetError() )
								printf( "glerror(%d): %d\n", __LINE__, x );
						}

						// Bottom
						if( info & VoxelSector.FACEDRAW_Operations.BELOW )
						{
							Stat_RenderDrawFaces++;
							Stat_FaceBottom++;
							glBegin( GL_TRIANGLES );
							glTexCoord2f( 1.0, 0.25 ); glVertex3f( P0.x, P0.y, P0.z );
							glTexCoord2f( 1.0, 0.50 ); glVertex3f( P1.x, P1.y, P1.z );
							glTexCoord2f( 0.75, 0.25 ); glVertex3f( P3.x, P3.y, P3.z );
							glTexCoord2f( 0.75, 0.25 ); glVertex3f( P3.x, P3.y, P3.z );
							glTexCoord2f( 1.0, 0.50 ); glVertex3f( P1.x, P1.y, P1.z );
							glTexCoord2f( 0.75, 0.50 ); glVertex3f( P2.x, P2.y, P2.z );
							glEnd();
						}
						{
							int x; if( x = glGetError() )
								printf( "glerror(%d): %d\n", __LINE__, x );
						}


					}
				}
				glEndList();
				{
					int x; if( x = glGetError() )
						printf( "glerror(%d): %d\n", __LINE__, x );
				}
			}
#endif
		}

	}
}
