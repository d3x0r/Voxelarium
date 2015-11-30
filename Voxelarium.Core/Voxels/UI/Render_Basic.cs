/*
 * Before porting, this header appeared inmost sources.  Of course
 * the change from C++ to C# required significant changes an no part
 * is entirely original.
 * 
 * This file is part of Blackvoxel. (Now Voxelarium)
 *
 * Copyright 2010-2014 Laurent Thiebaut & Olivia Merle
 * Copyright 2015-2016 James Buckeyne  *** Added 11/22/2015
 *
 * Voxelarium is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Voxelarium is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
using Voxelarium.LinearMath;
using OpenTK;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Text;
using Voxelarium.Core.Support;
using Voxelarium.Core.UI;
using Voxelarium.Core.Voxels.Types;
using Voxelarium.Core.Voxels.Physics;
using System.Diagnostics;
using Voxelarium.Core.UI.Shaders;
using Voxelarium.Common;
#if !USE_GLES2
using OpenTK.Graphics.OpenGL;
#else
using OpenTK.Graphics.ES20;
#endif

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
					VoxelState[i] = (ushort)( ( ( Voxel == 0 ) ? 1 : 0 )
						   | ( VoxelType.properties.Draw_FullVoxelOpacity ? 2 : 0 )
						   | ( VoxelType.properties.Draw_TransparentRendering ? 4 : 0 ) );
				}

				Voxel = Sector[(int)VoxelSector.RelativeVoxelOrds.INCENTER].Data.Data[Offset[(int)VoxelSector.RelativeVoxelOrds.INCENTER]];

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
			SectorTable[0] = Sector; if( SectorTable[0] == null ) { return; }
			SectorTable[1] = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1]; if( SectorTable[1] == null ) { SectorTable[1] = MissingSector; SectorTable[0].PartialCulling |= VoxelSector.FACEDRAW_Operations.LEFT; }
			SectorTable[2] = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1]; if( SectorTable[2] == null ) { SectorTable[2] = MissingSector; SectorTable[0].PartialCulling |= VoxelSector.FACEDRAW_Operations.RIGHT; }
			SectorTable[3] = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND - 1]; if( SectorTable[3] == null ) { SectorTable[3] = MissingSector; SectorTable[0].PartialCulling |= VoxelSector.FACEDRAW_Operations.BEHIND; }
			SectorTable[6] = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD - 1]; if( SectorTable[6] == null ) { SectorTable[6] = MissingSector; SectorTable[0].PartialCulling |= VoxelSector.FACEDRAW_Operations.AHEAD; }
			SectorTable[9] = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1]; if( SectorTable[9] == null ) { SectorTable[9] = MissingSector; SectorTable[0].PartialCulling |= VoxelSector.FACEDRAW_Operations.BELOW; }
			SectorTable[18] = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1]; if( SectorTable[18] == null ) { SectorTable[18] = MissingSector; SectorTable[0].PartialCulling |= VoxelSector.FACEDRAW_Operations.ABOVE; }
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
					//    BlocMatrix[1][0] = SectorDataTable[(VoxelSector.STableX[xc ]+STableY[0]+STableZ[zc ])][OfTableX[xc]+OfTableY[0]+OfTableZ[zc]];
					BlocMatrix[1][1] = SectorDataTable[( STableX[xp] + STableY[0] + STableZ[zc] )][OfTableX[xp] + OfTableY[0] + OfTableZ[zc]];
					//    BlocMatrix[1][2] = SectorDataTable[(STableX[xpp]+STableY[0]+STableZ[zc ])][OfTableX[xpp]+OfTableY[0]+OfTableZ[zc ]]
					BlocMatrix[1][3] = SectorDataTable[( STableX[xc] + STableY[0] + STableZ[zp] )][OfTableX[xc] + OfTableY[0] + OfTableZ[zp]];
					BlocMatrix[1][4] = SectorDataTable[( STableX[xp] + STableY[0] + STableZ[zp] )][OfTableX[xp] + OfTableY[0] + OfTableZ[zp]];
					BlocMatrix[1][5] = SectorDataTable[( STableX[xpp] + STableY[0] + STableZ[zp] )][OfTableX[xpp] + OfTableY[0] + OfTableZ[zp]];
					//    BlocMatrix[1][6] = SectorDataTable[(STableX[xc ]+STableY[0]+STableZ[zpp])][OfTableX[xc ]+OfTableY[0]+OfTableZ[zpp]]
					BlocMatrix[1][7] = SectorDataTable[( STableX[xp] + STableY[0] + STableZ[zpp] )][OfTableX[xp] + OfTableY[0] + OfTableZ[zpp]];
					//    BlocMatrix[1][8] = SectorDataTable[(STableX[xpp]+STableY[0]+STableZ[zpp])][OfTableX[xpp]+OfTableY[0]+OfTableZ[zpp]]

					//    BlocMatrix[2][0] = SectorDataTable[(STableX[xc ]+STableY[1]+STableZ[zc ])][OfTableX[xc ]+OfTableY[1]+OfTableZ[zc ]]
					BlocMatrix[2][1] = SectorDataTable[( STableX[xp] + STableY[1] + STableZ[zc] )][OfTableX[xp] + OfTableY[1] + OfTableZ[zc]];
					//    BlocMatrix[2][2] = SectorDataTable[(STableX[xpp]+STableY[1]+STableZ[zc ])][OfTableX[xpp]+OfTableY[1]+OfTableZ[zc ]]
					BlocMatrix[2][3] = SectorDataTable[( STableX[xc] + STableY[1] + STableZ[zp] )][OfTableX[xc] + OfTableY[1] + OfTableZ[zp]];
					BlocMatrix[2][4] = SectorDataTable[( STableX[xp] + STableY[1] + STableZ[zp] )][OfTableX[xp] + OfTableY[1] + OfTableZ[zp]];
					BlocMatrix[2][5] = SectorDataTable[( STableX[xpp] + STableY[1] + STableZ[zp] )][OfTableX[xpp] + OfTableY[1] + OfTableZ[zp]];
					//    BlocMatrix[2][6] = SectorDataTable[(STableX[xc ]+STableY[1]+STableZ[zpp])][OfTableX[xc ]+OfTableY[1]+OfTableZ[zpp]]
					BlocMatrix[2][7] = SectorDataTable[( STableX[xp] + STableY[1] + STableZ[zpp] )][OfTableX[xp] + OfTableY[1] + OfTableZ[zpp]];
					//    BlocMatrix[2][8] = SectorDataTable[(STableX[xpp]+STableY[1]+STableZ[zpp])][OfTableX[xpp]+OfTableY[1]+OfTableZ[zpp]]

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

						Sector.Culler.setFaceCulling( Sector
									, OfTableX[xp] + OfTableY[yp] + OfTableZ[zp], info );

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
			Sector_Out = null;  // again a redundant assignment
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
							if( FaceState != 0 ) VoxelFC_In[Off_In] |= (byte)VoxelSector.FACEDRAW_Operations.ABOVE;
							else VoxelFC_In[Off_In] &= (byte)( ( ~(int)VoxelSector.FACEDRAW_Operations.ABOVE ) & 0xFF );
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

					for( Off_Ip = 0, Off_Op = (int)VoxelSector.ZVOXELBLOCSIZE_Y - 1; 
						 Off_Ip < ( VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_Z ); 
						 Off_Ip += (int)VoxelSector.ZVOXELBLOCSIZE_Y, 
						 Off_Op += (int)VoxelSector.ZVOXELBLOCSIZE_Y ) // x (0..15)
					{
						for( Off_Aux = 0; 
							 Off_Aux < ( VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_Z ); 
							 Off_Aux += (int)( VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y ) ) // z (0..15)
						{
							Off_In = Off_Ip + Off_Aux;
							Off_Out = Off_Op + Off_Aux;
							ushort Voxel_In = VoxelData_In.Data[Off_In];
							ushort Voxel_Out = VoxelData_Out.Data[Off_Out];
							//ZVoxelType * VtIn =  VoxelTypeTable[ Voxel_In ];
							//ZVoxelType * VtOut = VoxelTypeTable[ Voxel_Out ];


							FaceState = IntFaceStateTable[VoxelTypeTable[Voxel_In].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS][VoxelTypeTable[Voxel_Out].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS];

							//FaceState = IntFaceStateTable[ VoxelTypeTable[ VoxelData_In.Data[Off_In] ].DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS ][ VoxelTypeTable[ VoxelData_Out.Data[Off_Out] ].DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_CULLINGBITS ];
							if( FaceState != 0 ) VoxelFC_In[Off_In] |= (byte)VoxelSector.FACEDRAW_Operations.BELOW;
							else VoxelFC_In[Off_In] &= (byte)( (int)~VoxelSector.FACEDRAW_Operations.BELOW & 0xFF );
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
							if( FaceState != 0 ) VoxelFC_In[Off_In] |= (byte)VoxelSector.FACEDRAW_Operations.LEFT;
							else VoxelFC_In[Off_In] &= (byte)( (int)~VoxelSector.FACEDRAW_Operations.LEFT & 0xFF );
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
							if( FaceState != 0 ) VoxelFC_In[Off_In] |= (byte)VoxelSector.FACEDRAW_Operations.BEHIND;
							else VoxelFC_In[Off_In] &= (byte)( (int)~VoxelSector.FACEDRAW_Operations.BEHIND & 0xFF );
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
				Sector_Out.Flag_Render_Dirty = true;
                Sector_In.Flag_Render_Dirty = true;
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



		internal override void Render( Display display, VoxelGameEnvironment game, VoxelWorld world )
		{
			HighPerfTimer Timer = new HighPerfTimer();
			HighPerfTimer Timer_SectorRefresh = new HighPerfTimer();
			long Time;
			uint RenderedSectors;
			int i;
			int in_centerX, in_centerY, in_centerZ;

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
			int Sector_x, Sector_y, Sector_z;
			int x, y, z;

			VoxelSector Sector;
			int Priority, PriorityBoost;
			uint Sector_Refresh_Count;

			// Transforming Camera coords to sector coords. One Voxel is 256 observer units. One sector is 16x16x32.
			btVector3 origin;
			Camera.location.getOrigin( out origin );

			Sector_x = ( (int)origin.x >> ( world.VoxelBlockSizeBits + VoxelSector.ZVOXELBLOCSHIFT_X ) );
			Sector_y = ( (int)origin.y >> ( world.VoxelBlockSizeBits + VoxelSector.ZVOXELBLOCSHIFT_Y ) );
			Sector_z = ( (int)origin.z >> ( world.VoxelBlockSizeBits + VoxelSector.ZVOXELBLOCSHIFT_Z ) );

			in_centerX = (int)origin.x & VoxelSector.ZVOXELBLOCMASK_X;
			in_centerY = (int)origin.y & VoxelSector.ZVOXELBLOCMASK_Y;
			in_centerZ = (int)origin.z & VoxelSector.ZVOXELBLOCMASK_Z;


			// Rendering loop
			// Preparation and first rendering pass

			RenderedSectors = 0;
			Sector_Refresh_Count = 0;
			int voxelSizeBits = world.VoxelBlockSizeBits;
			float voxelSize = world.VoxelBlockSize;
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

				Cv.x = (float)( ( x ) << ( voxelSizeBits + VoxelSector.ZVOXELBLOCSHIFT_X ) );
				Cv.y = (float)( ( y ) << ( voxelSizeBits + VoxelSector.ZVOXELBLOCSHIFT_Y ) );
				Cv.z = (float)( ( z ) << ( voxelSizeBits + VoxelSector.ZVOXELBLOCSHIFT_Z ) );

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

							MakeSectorRenderingData( Sector );

							MakeSectorRenderingData_Sorted( Sector, SectorSphereEntry.relative_pos
														, in_centerX, in_centerY, in_centerZ );
							//Log.log( "Drew sector geometry {0} {1} {2}", Sector.Pos_x, Sector.Pos_y, Sector.Pos_z );

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

						Sector.geometry.DrawBuffer( false, false, world.TextureAtlas.OpenGl_TextureRef );
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

#if !USE_GLES2
			GL.AlphaFunc( AlphaFunction.Greater, 0.2f );
			GL.Enable( EnableCap.AlphaTest );
#endif

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

						Sector.geometry.DrawBuffer( true, false, world.TextureAtlas.OpenGl_TextureRef );
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

			Timer.End();

			/*printf("Frame Time : %lu Rend Sects: %lu Draw Faces :%lu Top:%lu Bot:%lu Le:%lu Ri:%lu Front:%lu Back:%lu\n",Timer.GetResult(), RenderedSectors, Stat_RenderDrawFaces, Stat_FaceTop, Stat_FaceBottom,
				   Stat_FaceLeft,Stat_FaceRight,Stat_FaceFront,Stat_FaceBack);*/

			//printf("RenderedSectors : %lu\n",RenderedSectors);
			//SDL_GL_SwapBuffers( );
		}


		void MakeSectorRenderingData( VoxelSector Sector )
		{
			Color face = Color.Red, edge = Color.Black;
			short power = 400;
			int x, y, z;
			int ofs;
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
				//Log.log( "Is Dirty Building sector {0} {1} {2}", Sector.Pos_x, Sector.Pos_y, Sector.Pos_z );
				Sector_Display_x = (int)( Sector.Pos_x * Sector.Size_x * voxelSize );
				Sector_Display_y = (int)( Sector.Pos_y * Sector.Size_y * voxelSize );
				Sector_Display_z = (int)( Sector.Pos_z * Sector.Size_z * voxelSize );

				Sector.Flag_Void_Regular = true;
				Sector.physics.Empty = true;
				Sector.Flag_Void_Transparent = true;
				Sector.Flag_Void_Custom = true;
				//for( Pass = 0; Pass < 2; Pass++ )
				Pass = 0;
				{
					geometry.SetSolid();
					geometry.Clear();
					/*
					Log.log( "Sector is {6} {7} {8} near l{0} r{1} ab{2} bl{3} bh{4} ah{5}"
						, ( Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT-1] != null ) ? "Yes" : "no"
						, ( Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1] != null ) ? "Yes" : "no"
						, ( Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1] != null ) ? "Yes" : "no"
						, ( Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1] != null ) ? "Yes" : "no"
						, ( Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND - 1] != null ) ? "Yes" : "no"
						, ( Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD - 1] != null ) ? "Yes" : "no" 
						, Sector.Pos_x, Sector.Pos_y, Sector.Pos_z 
						);
					*/
					for( z = 0; z < Sector.Size_z; z++ )
					{
						for( x = 0; x < Sector.Size_x; x++ )
						{
							for( y = 0; y < Sector.Size_y; y++ )
							{
								Offset = (uint)( y + ( x * Sector.Size_y ) + ( z * ( Sector.Size_y * Sector.Size_x ) ) );
								cube = Sector.Data.Data[Offset];
								info = (VoxelSector.FACEDRAW_Operations)FaceCulling[Offset];

								if( cube > 0 && info != VoxelSector.FACEDRAW_Operations.NONE )
								{
									if( VoxelTypeTable[cube].properties.Draw_TransparentRendering )
										{ Draw = false; Sector.Flag_Void_Transparent = false; }
									else
										{ Draw = true; Sector.Flag_Void_Regular = false; }
								}
								else
									Draw = false;

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
										face_is_shaded = false; // uses texture instead of algorithm

									if( info != 0 )
									{
										//Log.log( "Set sector {0} {1} {2} offset {3}   {4:x}", Sector.Pos_x, Sector.Pos_y, Sector.Pos_z, Offset, info );
										Sector.physics.SetVoxel( Offset );
									}
									else
									{
										Sector.physics.ClearVoxel( Offset );
										continue;
									}

									cubx = (float)( x * voxelSize + Sector_Display_x );
									cuby = (float)( y * voxelSize + Sector_Display_y );
									cubz = (float)( z * voxelSize + Sector_Display_z );

									if( 0 != ( VoxelTypeTable[cube].properties.DrawInfo & VoxelGlobalSettings.ZVOXEL_DRAWINFO_SPECIALRENDERING ))
									{ /*VoxelTypeTable[cube].SpecialRender( cubx, cuby, cubz ); */
										Log.log( "Need to add custom pass for special render" );
										continue;
									}

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
											geometry.AddQuad( ref P3, ref P7, ref P0, ref P4, face, edge, power );
										else
											geometry.AddQuad( ref P3, ref P7, ref P0, ref P4, ref VoxelTypeTable[cube].TextureCoords );
									}

									// Right
									if( ( info & VoxelSector.FACEDRAW_Operations.RIGHT ) != 0 )
									{
										Stat_RenderDrawFaces++;
										Stat_FaceRight++;
										//Log.log( "Add {0} {1} {2} {3}", P5, P6, P2, P1 );
										if( face_is_shaded )
											geometry.AddQuad( ref P1, ref P5, ref P2, ref P6, face, edge, power );
										else
											geometry.AddQuad( ref P1, ref P5, ref P2, ref P6, ref VoxelTypeTable[cube].TextureCoords );
									}
									//Front
									if( ( info & VoxelSector.FACEDRAW_Operations.AHEAD ) != 0 )
									{
										Stat_RenderDrawFaces++;
										Stat_FaceFront++;
										//Log.log( "Add {0} {1} {2} {3}", P0, P4, P5, P1 );
										if( face_is_shaded )
											geometry.AddQuad( ref P0, ref P4, ref P1, ref P5, face, edge, power );
										else
											geometry.AddQuad( ref P0, ref P4, ref P1, ref P5, ref VoxelTypeTable[cube].TextureCoords );
									}

									//Back
									if( ( info & VoxelSector.FACEDRAW_Operations.BEHIND ) != 0 )
									{
										Stat_RenderDrawFaces++;
										Stat_FaceBack++;
										//Log.log( "Add {0} {1} {2} {3}", P2, P6, P3, P7 );
										if( face_is_shaded )
											geometry.AddQuad( ref P2, ref P6, ref P3, ref P7, face, edge, power );
										else
											geometry.AddQuad( ref P2, ref P6, ref P3, ref P7, ref VoxelTypeTable[cube].TextureCoords );
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
											geometry.AddQuad( ref P3, ref P0, ref P2, ref P1, face, edge, power );
										else
											geometry.AddQuad( ref P3, ref P0, ref P2, ref P1, ref VoxelTypeTable[cube].TextureCoords );
									}
								}
								else
									Sector.physics.ClearVoxel( Offset );
							}
						}
					}
					// if in the first pass, the sector has no transparent block, the second pass is cancelled.
					//if( Sector.Flag_Void_Transparent ) break;
				}
				geometry.SetSolid();
				Sector.Flag_Render_Dirty = false;
			}
		}


		void MakeSectorRenderingData_Sorted( VoxelSector Sector, VoxelSector.RelativeVoxelOrds viewed_as
					, int centerX, int centerY, int centerZ )
		{
			if( Sector.Flag_Void_Transparent )
				return;

            Color face = Color.Red, edge = Color.Black;
			short power = 400;
			int x, y, z;
			VoxelSector.FACEDRAW_Operations info;
			VoxelSector.FACEDRAW_Operations sorted_draw_info = sorted_draw_infos[(int)viewed_as];
			ushort cube, prevcube;
			/* build sector geometry */

			uint Offset;
			float cubx, cuby, cubz;
			int Sector_Display_x, Sector_Display_y, Sector_Display_z;
			bool Draw;
			VoxelTypeManager VoxelTypeManager = Sector.VoxelTypeManager;
			VoxelType[] VoxelTypeTable = VoxelTypeManager.VoxelTable;
			Vector3 P0, P1, P2, P3, P4, P5, P6, P7;

			//Log.log( "Building sector {0} {1} {2}", Sector.Pos_x, Sector.Pos_y, Sector.Pos_z );
			// Display list creation or reuse.
			VoxelGeometry geometry = Sector.geometry;
			float voxelSize = Sector.world.VoxelBlockSize;
			byte[] FaceCulling = ( Sector.Culler as BasicVoxelCuller ).FaceCulling;
			Sector_Display_x = (int)( Sector.Pos_x * Sector.Size_x * voxelSize );
			Sector_Display_y = (int)( Sector.Pos_y * Sector.Size_y * voxelSize );
			Sector_Display_z = (int)( Sector.Pos_z * Sector.Size_z * voxelSize );

			ushort[] SectorIndexes = SortedSectorIndexes[(int)viewed_as];
			if( geometry.transparent_render_sorting != viewed_as )
			{
				geometry.transparent_render_sorting = viewed_as;
				Sector.Flag_Render_Dirty_Transparent = true;
				geometry.sortedX = -1;
			}
			if( viewed_as == VoxelSector.RelativeVoxelOrds.INCENTER )
			{
				if( center_sorted_x != centerX 
					|| center_sorted_y != centerY 
					|| center_sorted_z != centerZ )
				{
					Sector.Flag_Render_Dirty_Transparent = true;
					BuildSortListInSector( centerX, centerY, centerZ );
				}
			}

			if( Sector.Flag_Render_Dirty_Transparent )
			{
				{
					geometry.SetTransparent(); 

					geometry.Clear();

					prevcube = 0;
					for( int OffsetIndex = 0; OffsetIndex < VoxelSector.ZVOXELBLOCKCOUNT; OffsetIndex++ )
					{
						{
							{
								Offset = SectorIndexes[OffsetIndex];

								cube = Sector.Data.Data[Offset];
								info = (VoxelSector.FACEDRAW_Operations)FaceCulling[Offset] & sorted_draw_info;

								if( cube > 0 && ( info )!= VoxelSector.FACEDRAW_Operations.NONE )
								{
									Draw = VoxelTypeTable[cube].properties.Draw_TransparentRendering;
								}
								else
									Draw = false;

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

									cubx = (float)( ( ( Offset >> VoxelSector.ZVOXELBLOCSHIFT_Y ) & VoxelSector.ZVOXELBLOCMASK_X) * voxelSize + Sector_Display_x );
									cuby = (float)( ( ( Offset ) & VoxelSector.ZVOXELBLOCMASK_Y ) * voxelSize + Sector_Display_y );
									cubz = (float)( ( ( Offset >> (VoxelSector.ZVOXELBLOCSHIFT_Y+VoxelSector.ZVOXELBLOCSHIFT_X) ) & VoxelSector.ZVOXELBLOCMASK_Z ) * voxelSize + Sector_Display_z );

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
											geometry.AddQuad( ref P3, ref P7, ref P0, ref P4, face, edge, power );
										else
											geometry.AddQuad( ref P3, ref P7, ref P0, ref P4, ref VoxelTypeTable[cube].TextureCoords );
									}

									// Right
									if( ( info & VoxelSector.FACEDRAW_Operations.RIGHT ) != 0 )
									{
										Stat_RenderDrawFaces++;
										Stat_FaceRight++;
										//Log.log( "Add {0} {1} {2} {3}", P5, P6, P2, P1 );
										if( face_is_shaded )
											geometry.AddQuad( ref P1, ref P5, ref P2, ref P6, face, edge, power );
										else
											geometry.AddQuad( ref P1, ref P5, ref P2, ref P6, ref VoxelTypeTable[cube].TextureCoords );
									}
									//Front
									if( ( info & VoxelSector.FACEDRAW_Operations.AHEAD ) != 0 )
									{
										Stat_RenderDrawFaces++;
										Stat_FaceFront++;
										//Log.log( "Add {0} {1} {2} {3}", P0, P4, P5, P1 );
										if( face_is_shaded )
											geometry.AddQuad( ref P0, ref P4, ref P1, ref P5, face, edge, power );
										else
											geometry.AddQuad( ref P0, ref P4, ref P1, ref P5, ref VoxelTypeTable[cube].TextureCoords );
									}

									//Back
									if( ( info & VoxelSector.FACEDRAW_Operations.BEHIND ) != 0 )
									{
										Stat_RenderDrawFaces++;
										Stat_FaceBack++;
										//Log.log( "Add {0} {1} {2} {3}", P2, P6, P3, P7 );
										if( face_is_shaded )
											geometry.AddQuad( ref P2, ref P6, ref P3, ref P7, face, edge, power );
										else
											geometry.AddQuad( ref P2, ref P6, ref P3, ref P7, ref VoxelTypeTable[cube].TextureCoords );
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
											geometry.AddQuad( ref P3, ref P0, ref P2, ref P1, face, edge, power );
										else
											geometry.AddQuad( ref P3, ref P0, ref P2, ref P1, ref VoxelTypeTable[cube].TextureCoords );
									}
								}
							}
						}
					}
					// if in the first pass, the sector has no transparent block, the second pass is cancelled.
					//if( Sector.Flag_Void_Transparent ) break;
				}
				geometry.SetSolid();
				Sector.Flag_Render_Dirty_Transparent = false;
			}
		}



		static VoxelSector.FACEDRAW_Operations[] sorted_draw_infos = new VoxelSector.FACEDRAW_Operations[27];
        static ushort[][] SortedSectorIndexes = new ushort[27][];
		static SortingTree sorter = new SortingTree( VoxelSector.ZVOXELBLOCKCOUNT );
		static int center_sorted_x = -1, center_sorted_y, center_sorted_z;

		/// <summary>
		/// Build the INCENTER order list.
		/// </summary>
		/// <param name="x">voxel position of viewpoint</param>
		/// <param name="y">voxel position of viewpoint</param>
		/// <param name="z">voxel position of viewpoint</param>
		static void BuildSortListInSector( int eye_x, int eye_y, int eye_z )
		{
			int x, y, z;
			int tmpx, tmpy, tmpz;
			int d;
			ushort val;
			if( center_sorted_x != eye_x || center_sorted_y != eye_y || center_sorted_z != eye_z )
			{
				int n;
				center_sorted_x = eye_x;
				center_sorted_y = eye_y;
				center_sorted_z = eye_z;
				// x, y, z po
				for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
					for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
						for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
						{
							val = (ushort)( x * VoxelSector.ZVOXELBLOCSIZE_Y + y + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y );
							tmpx = x - eye_x;
							tmpy = y - eye_y;
							tmpz = z - eye_z;
							d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
							sorter.Add( d, val );
						}

				ushort[] Indexes;
				Indexes = SortedSectorIndexes[0];
				n = VoxelSector.ZVOXELBLOCKCOUNT;
				foreach( ushort index in sorter )
				{
					// retrieved closest to furthest so... reverse storing it.
					Indexes[--n] = index;
				}
			}
		}

		struct BinaryQueueNode {
			internal int minx, maxx, midx;
			internal int miny, maxy, midy;
			internal int minz, maxz, midz;
		}


		static void BuildBinaryListIndexes( ushort[] list, ref int index )
		{
			Queue<BinaryQueueNode> values = new Queue<BinaryQueueNode>( VoxelSector.ZVOXELBLOCKCOUNT );
			BinaryQueueNode bqn;
			BinaryQueueNode bqn_add;

			bool[] used = new bool[VoxelSector.ZVOXELBLOCKCOUNT];
			int next_ofs;
            bqn.minx = 0;
			bqn.maxx = VoxelSector.ZVOXELBLOCSIZE_X - 1;
			bqn.midx = ( bqn.maxx + bqn.minx ) / 2;
			bqn.miny = 0;
			bqn.maxy = VoxelSector.ZVOXELBLOCSIZE_Y - 1;
			bqn.midy = ( bqn.maxy + bqn.miny ) / 2;
			bqn.minz = 0;
			bqn.maxz = VoxelSector.ZVOXELBLOCSIZE_Z - 1;
			bqn.midz = ( bqn.maxz + bqn.minz ) / 2;
			values.Enqueue( bqn );
			while( values.Count > 0 )
			{
				bqn = values.Dequeue();
				list[index++] = (ushort)( bqn.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn.midy + bqn.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X );

				if( bqn.midx > bqn.minx )
				{
					if( bqn.midy > bqn.miny )
					{
						if( bqn.midz > bqn.minz )
						{
							bqn_add.minx = bqn.minx;
							bqn_add.maxx = bqn.midx - 1;
							bqn_add.midx = ( bqn_add.maxx + bqn_add.minx ) / 2;
							bqn_add.miny = bqn.miny;
							bqn_add.maxy = bqn.midy - 1;
							bqn_add.midy = ( bqn_add.maxy + bqn_add.miny ) / 2;
							bqn_add.minz = bqn.minz;
							bqn_add.maxz = bqn.midz - 1;
							bqn_add.midz = ( bqn_add.maxz + bqn_add.minz ) / 2;
				
							next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
							if (!used[next_ofs])
							{
								used[next_ofs] = true;
								//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
								values.Enqueue(bqn_add);
							}
						}
						if( bqn.maxz > bqn.midz )
						{
							bqn_add.minx = bqn.minx;
							bqn_add.maxx = bqn.midx - 1;
							bqn_add.midx = ( bqn_add.maxx + bqn_add.minx ) / 2;
							bqn_add.miny = bqn.miny;
							bqn_add.maxy = bqn.midy - 1;
							bqn_add.midy = ( bqn_add.maxy + bqn_add.miny ) / 2;
							bqn_add.minz = bqn.midz + 1;
							bqn_add.maxz = bqn.maxz;
							bqn_add.midz = ( bqn_add.maxz + bqn_add.minz ) / 2;
							next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
							if (!used[next_ofs])
							{
								used[next_ofs] = true;
								//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
								values.Enqueue(bqn_add);
							}
						}
						bqn_add.minx = bqn.minx;
						bqn_add.maxx = bqn.midx - 1;
						bqn_add.midx = ( bqn_add.maxx + bqn_add.minx ) / 2;
						bqn_add.miny = bqn.miny;
						bqn_add.maxy = bqn.midy - 1;
						bqn_add.midy = ( bqn_add.maxy + bqn_add.miny ) / 2;
						bqn_add.minz = bqn.minz;
						bqn_add.maxz = bqn.maxz;
						bqn_add.midz = bqn.midz;
						next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
						if (!used[next_ofs])
						{
							used[next_ofs] = true;
							//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
							values.Enqueue(bqn_add);
						}
					}
					if ( bqn.maxy > bqn.midy )
					{
						if( bqn.midz > bqn.minz )
						{
							bqn_add.minx = bqn.minx;
							bqn_add.maxx = bqn.midx - 1;
							bqn_add.midx = ( bqn_add.maxx + bqn_add.minx ) / 2;
							bqn_add.miny = bqn.midy + 1;
							bqn_add.maxy = bqn.maxy;
							bqn_add.midy = ( bqn_add.maxy + bqn_add.miny ) / 2;
							bqn_add.minz = bqn.minz;
							bqn_add.maxz = bqn.midz - 1;
							bqn_add.midz = ( bqn_add.maxz + bqn_add.minz ) / 2;
							next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
							if (!used[next_ofs])
							{
								used[next_ofs] = true;
								//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
								values.Enqueue(bqn_add);
							}
						}
						if ( bqn.maxz > bqn.midz )
						{
							bqn_add.minx = bqn.minx;
							bqn_add.maxx = bqn.midx - 1;
							bqn_add.midx = ( bqn_add.maxx + bqn_add.minx ) / 2;
							bqn_add.miny = bqn.midy + 1;
							bqn_add.maxy = bqn.maxy;
							bqn_add.midy = ( bqn_add.maxy + bqn_add.miny ) / 2;
							bqn_add.minz = bqn.midz + 1;
							bqn_add.maxz = bqn.maxz;
							bqn_add.midz = ( bqn_add.maxz + bqn_add.minz ) / 2;
							next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
							if (!used[next_ofs])
							{
								used[next_ofs] = true;
								//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
								values.Enqueue(bqn_add);
							}
						}
						bqn_add.minx = bqn.minx;
						bqn_add.maxx = bqn.midx - 1;
						bqn_add.midx = ( bqn_add.maxx + bqn_add.minx ) / 2;
						bqn_add.miny = bqn.miny;
						bqn_add.maxy = bqn.midy - 1;
						bqn_add.midy = ( bqn_add.maxy + bqn_add.miny ) / 2;
						bqn_add.minz = bqn.minz;
						bqn_add.maxz = bqn.maxz;
						bqn_add.midz = bqn.midz;
						next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
						if (!used[next_ofs])
						{
							used[next_ofs] = true;
							//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
							values.Enqueue(bqn_add);
						}
					}
				}
				if( bqn.maxx > bqn.midx )
				{
					if( bqn.midy > bqn.miny )
					{
						if( bqn.midz > bqn.minz )
						{
							bqn_add.minx = bqn.midx + 1;
							bqn_add.maxx = bqn.maxx;
							bqn_add.midx = ( bqn_add.maxx + bqn_add.minx ) / 2;
							bqn_add.miny = bqn.miny;
							bqn_add.maxy = bqn.midy - 1;
							bqn_add.midy = ( bqn_add.maxy + bqn_add.miny ) / 2;
							bqn_add.minz = bqn.minz;
							bqn_add.maxz = bqn.midz - 1;
							bqn_add.midz = ( bqn_add.maxz + bqn_add.minz ) / 2;
							next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
							if (!used[next_ofs])
							{
								used[next_ofs] = true;
								//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
								values.Enqueue(bqn_add);
							}
						}
						if ( bqn.maxz > bqn.midz )
						{
							bqn_add.minx = bqn.midx + 1;
							bqn_add.maxx = bqn.maxx;
							bqn_add.midx = ( bqn_add.maxx + bqn_add.minx ) / 2;
							bqn_add.miny = bqn.miny;
							bqn_add.maxy = bqn.midy - 1;
							bqn_add.midy = ( bqn_add.maxy + bqn_add.miny ) / 2;
							bqn_add.minz = bqn.midz + 1;
							bqn_add.maxz = bqn.maxz;
							bqn_add.midz = ( bqn_add.maxz + bqn_add.minz ) / 2;
							next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
							if (!used[next_ofs])
							{
								used[next_ofs] = true;
								//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
								values.Enqueue(bqn_add);
							}
						}
						bqn_add.minx = bqn.midx + 1;
						bqn_add.maxx = bqn.maxx;
						bqn_add.midx = ( bqn_add.maxx + bqn_add.minx ) / 2;
						bqn_add.miny = bqn.miny;
						bqn_add.maxy = bqn.midy - 1;
						bqn_add.midy = ( bqn_add.maxy + bqn_add.miny ) / 2;
						bqn_add.minz = bqn.minz;
						bqn_add.maxz = bqn.maxz;
						bqn_add.midz = bqn.midz;
						next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
						if (!used[next_ofs])
						{
							used[next_ofs] = true;
							//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
							values.Enqueue(bqn_add);
						}
					}
					if ( bqn.maxy > bqn.midy )
					{
						if( bqn.midz > bqn.minz )
						{
							bqn_add.minx = bqn.midx + 1;
							bqn_add.maxx = bqn.maxx;
							bqn_add.midx = ( bqn_add.maxx + bqn_add.minx ) / 2;
							bqn_add.miny = bqn.midy + 1;
							bqn_add.maxy = bqn.maxy;
							bqn_add.midy = ( bqn_add.maxy + bqn_add.miny ) / 2;
							bqn_add.minz = bqn.minz;
							bqn_add.maxz = bqn.midz - 1;
							bqn_add.midz = ( bqn_add.maxz + bqn_add.minz ) / 2;
							next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
							if (!used[next_ofs])
							{
								used[next_ofs] = true;
								//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
								values.Enqueue(bqn_add);
							}
						}
						if ( bqn.maxz > bqn.midz )
						{
							bqn_add.minx = bqn.midx + 1;
							bqn_add.maxx = bqn.maxx;
							bqn_add.midx = ( bqn_add.maxx + bqn_add.minx ) / 2;
							bqn_add.miny = bqn.midy + 1;
							bqn_add.maxy = bqn.maxy;
							bqn_add.midy = ( bqn_add.maxy + bqn_add.miny ) / 2;
							bqn_add.minz = bqn.midz + 1;
							bqn_add.maxz = bqn.maxz;
							bqn_add.midz = ( bqn_add.maxz + bqn_add.minz ) / 2;
							next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
							if (!used[next_ofs])
							{
								used[next_ofs] = true;
								//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
								values.Enqueue(bqn_add);
							}
						}
						bqn_add.minx = bqn.midx + 1;
						bqn_add.maxx = bqn.maxx;
						bqn_add.midx = ( bqn_add.maxx + bqn_add.minx ) / 2;
						bqn_add.miny = bqn.midy + 1;
						bqn_add.maxy = bqn.maxy;
						bqn_add.midy = ( bqn_add.maxy + bqn_add.miny ) / 2;
						bqn_add.minz = bqn.minz;
						bqn_add.maxz = bqn.maxz;
						bqn_add.midz = bqn.midz;
						next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
						if (!used[next_ofs])
						{
							used[next_ofs] = true;
							//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
							values.Enqueue(bqn_add);
						}
					}
				}

				if( bqn.midy > bqn.miny )
				{
					if( bqn.midz > bqn.minz )
					{
						bqn_add.minx = bqn.minx;
						bqn_add.maxx = bqn.maxx;
						bqn_add.midx = bqn.midx;
						bqn_add.miny = bqn.miny;
						bqn_add.maxy = bqn.midy - 1;
						bqn_add.midy = ( bqn_add.maxy + bqn_add.miny ) / 2;
						bqn_add.minz = bqn.minz;
						bqn_add.maxz = bqn.midz - 1;
						bqn_add.midz = ( bqn_add.maxz + bqn_add.minz ) / 2;
						next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
						if (!used[next_ofs])
						{
							used[next_ofs] = true;
							//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
							values.Enqueue(bqn_add);
						}
					}
					if ( bqn.maxz > bqn.midz )
					{
						bqn_add.minx = bqn.minx;
						bqn_add.maxx = bqn.maxx;
						bqn_add.midx = bqn.midx;
						bqn_add.miny = bqn.miny;
						bqn_add.maxy = bqn.midy - 1;
						bqn_add.midy = ( bqn_add.maxy + bqn_add.miny ) / 2;
						bqn_add.minz = bqn.midz + 1;
						bqn_add.maxz = bqn.maxz;
						bqn_add.midz = ( bqn_add.maxz + bqn_add.minz ) / 2;
						next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
						if (!used[next_ofs])
						{
							used[next_ofs] = true;
							//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
							values.Enqueue(bqn_add);
						}
					}
					bqn_add.minx = bqn.minx;
					bqn_add.maxx = bqn.maxx;
					bqn_add.midx = bqn.midx;
					bqn_add.miny = bqn.miny;
					bqn_add.maxy = bqn.midy - 1;
					bqn_add.midy = ( bqn_add.maxy + bqn_add.miny ) / 2;
					bqn_add.minz = bqn.minz;
					bqn_add.maxz = bqn.maxz;
					bqn_add.midz = bqn.midz;
					next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
					if (!used[next_ofs])
					{
						used[next_ofs] = true;
						//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
						values.Enqueue(bqn_add);
					}
				}
				if ( bqn.maxy > bqn.midy )
				{
					if( bqn.midz > bqn.minz )
					{
						bqn_add.minx = bqn.minx;
						bqn_add.maxx = bqn.maxx;
						bqn_add.midx = bqn.midx;
						bqn_add.miny = bqn.midy + 1;
						bqn_add.maxy = bqn.maxy;
						bqn_add.midy = ( bqn_add.maxy + bqn_add.miny ) / 2;
						bqn_add.minz = bqn.minz;
						bqn_add.maxz = bqn.midz - 1;
						bqn_add.midz = ( bqn_add.maxz + bqn_add.minz ) / 2;
						next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
						if (!used[next_ofs])
						{
							used[next_ofs] = true;
							//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
							values.Enqueue(bqn_add);
						}
					}
					if ( bqn.maxz > bqn.midz )
					{
						bqn_add.minx = bqn.minx;
						bqn_add.maxx = bqn.maxx;
						bqn_add.midx = bqn.midx;
						bqn_add.miny = bqn.midy + 1;
						bqn_add.maxy = bqn.maxy;
						bqn_add.midy = ( bqn_add.maxy + bqn_add.miny ) / 2;
						bqn_add.minz = bqn.midz + 1;
						bqn_add.maxz = bqn.maxz;
						bqn_add.midz = ( bqn_add.maxz + bqn_add.minz ) / 2;
						next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
						if (!used[next_ofs])
						{
							used[next_ofs] = true;
							//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
							values.Enqueue(bqn_add);
						}
					}
					bqn_add.minx = bqn.minx;
					bqn_add.maxx = bqn.maxx;
					bqn_add.midx = bqn.midx;
					bqn_add.miny = bqn.midy + 1;
					bqn_add.maxy = bqn.maxy;
					bqn_add.midy = ( bqn_add.maxy + bqn_add.miny ) / 2;
					bqn_add.minz = bqn.minz;
					bqn_add.maxz = bqn.maxz;
					bqn_add.midz = bqn.midz;
					next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
					if (!used[next_ofs])
					{
						used[next_ofs] = true;
						//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
						values.Enqueue(bqn_add);
					}
				}
				if ( bqn.midx > bqn.minx )
				{
					bqn_add.minx = bqn.minx;
					bqn_add.maxx = bqn.midx - 1;
					bqn_add.midx = ( bqn_add.maxx + bqn_add.minx ) / 2;
					bqn_add.miny = bqn.miny;
					bqn_add.maxy = bqn.maxy;
					bqn_add.midy = bqn.midy;
					bqn_add.minz = bqn.minz;
					bqn_add.maxz = bqn.maxz;
					bqn_add.midz = bqn.midz;
					next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
					if (!used[next_ofs])
					{
						used[next_ofs] = true;
						//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
						values.Enqueue(bqn_add);
					}
				}
				if ( bqn.maxx > bqn.midx )
				{
					bqn_add.minx = bqn.midx + 1;
					bqn_add.maxx = bqn.maxx;
					bqn_add.midx = ( bqn_add.maxx + bqn_add.minx ) / 2;
					bqn_add.miny = bqn.miny;
					bqn_add.maxy = bqn.maxy;
					bqn_add.midy = bqn.midy;
					bqn_add.minz = bqn.minz;
					bqn_add.maxz = bqn.maxz;
					bqn_add.midz = bqn.midz;
					next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
					if (!used[next_ofs])
					{
						used[next_ofs] = true;
						//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
						values.Enqueue(bqn_add);
					}
				}

				if ( bqn.midz > bqn.minz )
				{
					bqn_add.minx = bqn.minx;
					bqn_add.maxx = bqn.maxx;
					bqn_add.midx = bqn.midx;
					bqn_add.miny = bqn.miny;
					bqn_add.maxy = bqn.maxy;
					bqn_add.midy = bqn.midy;
					bqn_add.minz = bqn.minz;
					bqn_add.maxz = bqn.midz - 1;
					bqn_add.midz = ( bqn_add.maxz + bqn_add.minz ) / 2;
					next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
					if (!used[next_ofs])
					{
						used[next_ofs] = true;
						//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
						values.Enqueue(bqn_add);
					}
				}
				if ( bqn.maxz > bqn.midz )
				{
					bqn_add.minx = bqn.minx;
					bqn_add.maxx = bqn.maxx;
					bqn_add.midx = bqn.midx;
					bqn_add.miny = bqn.miny;
					bqn_add.maxy = bqn.maxy;
					bqn_add.midy = bqn.midy;
					bqn_add.minz = bqn.midz + 1;
					bqn_add.maxz = bqn.maxz;
					bqn_add.midz = ( bqn_add.maxz + bqn_add.minz ) / 2;
					next_ofs = (ushort)(bqn_add.midx * VoxelSector.ZVOXELBLOCSIZE_Y + bqn_add.midy + bqn_add.midz * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X);
					if (!used[next_ofs])
					{
						used[next_ofs] = true;
						//Log.log("Added {0} {1} {2}", bqn_add.midx, bqn_add.midy, bqn_add.midz);
						values.Enqueue(bqn_add);
					}
				}
			}
		}

		internal static void BuildSortList( ref int start_percent, ref int start_step, ref int start_steps )
		{
			int x, y, z;
			int tmpx, tmpy, tmpz;
			int d;
			ushort val;
			ushort[] binaryOutput = new ushort[VoxelSector.ZVOXELBLOCKCOUNT];
			int binaryOutputIndex = 0;
			start_steps += 27;
			sorter.AutoBalance = true;
			/*
			for (int n = 0; n < VoxelSector.ZVOXELBLOCKCOUNT; n++)
			{
				tmpx = ( n >> VoxelSector.ZVOXELBLOCSHIFT_Y ) & VoxelSector.ZVOXELBLOCMASK_X;
				tmpx -= VoxelSector.ZVOXELBLOCSIZE_X;
				tmpy = ( n >> 0 ) & VoxelSector.ZVOXELBLOCMASK_Y;
				tmpy -= VoxelSector.ZVOXELBLOCSIZE_Y;
				tmpz = ( n >> ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y ) ) & VoxelSector.ZVOXELBLOCMASK_Z;
				tmpz -= VoxelSector.ZVOXELBLOCSIZE_Z;

				d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
				sorter.Add(d, (ushort)n);
			}
			//BuildBinaryListIndexes( binaryOutput, ref binaryOutputIndex );
			sorter.FillBinaryLiteral(binaryOutput, ref binaryOutputIndex, -1);
			*/
			/*
			int ofs, check;
			for( ofs = 0; ofs < VoxelSector.ZVOXELBLOCKCOUNT; ofs++ )
			{
				for( check = 0; check < VoxelSector.ZVOXELBLOCKCOUNT; check++ )
				{
					if( binaryOutput[check] == ofs )
						break;
				}
				if( check == VoxelSector.ZVOXELBLOCKCOUNT )
					Debugger.Break();
			}
			*/
			//sorter.AutoBalance = false;

			VoxelSector.RelativeVoxelOrds i;
			for( i = VoxelSector.RelativeVoxelOrds.INCENTER;
						i <= VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT_BELOW; i++ )
			{
				ushort[] Indexes;
				int n, m;
				int xval, zval;
				Indexes = SortedSectorIndexes[(int)i] = new ushort[VoxelSector.ZVOXELBLOCKCOUNT];
				n = 0;
				m = VoxelSector.ZVOXELBLOCKCOUNT - 1;
				sorter.Clear();
				switch( i )
				{
					case VoxelSector.RelativeVoxelOrds.INCENTER:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL;
						// no information; this one has to be done custom.
						break;
					case VoxelSector.RelativeVoxelOrds.LEFT:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ VoxelSector.FACEDRAW_Operations.LEFT;
						//for( n = 0; n < VoxelSector.ZVOXELBLOCKCOUNT; n++ )
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)(zval + y); //(ushort)( binaryOutput[n] );
									//x = ( val >> VoxelSector.ZVOXELBLOCSHIFT_Y ) & VoxelSector.ZVOXELBLOCMASK_X;
									//y = ( val  ) & VoxelSector.ZVOXELBLOCMASK_Y;
									//z = ( val >> ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y ) ) & VoxelSector.ZVOXELBLOCMASK_Z;
									tmpx = VoxelSector.ZVOXELBLOCSIZE_X - x;
									tmpy = y - ( VoxelSector.ZVOXELBLOCSIZE_Y / 2 );
									tmpz = z - ( VoxelSector.ZVOXELBLOCSIZE_Z / 2 );
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									//Log.log("Add {0} {1} {2}  {3}", x, y, z, d);
									sorter.Add( d, val );
								}
							}
						}
						break;
					case VoxelSector.RelativeVoxelOrds.RIGHT:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ VoxelSector.FACEDRAW_Operations.RIGHT;
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = x;
									tmpy = y - ( VoxelSector.ZVOXELBLOCSIZE_Y / 2 );
									tmpz = z - ( VoxelSector.ZVOXELBLOCSIZE_Z / 2 );
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;
					case VoxelSector.RelativeVoxelOrds.ABOVE:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ VoxelSector.FACEDRAW_Operations.ABOVE;
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = x - ( VoxelSector.ZVOXELBLOCSIZE_X / 2 );
									tmpy = y;
									tmpz = z - ( VoxelSector.ZVOXELBLOCSIZE_Z / 2 );
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;
					case VoxelSector.RelativeVoxelOrds.BELOW:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ VoxelSector.FACEDRAW_Operations.BELOW;
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = x - ( VoxelSector.ZVOXELBLOCSIZE_X / 2 );
									tmpy = VoxelSector.ZVOXELBLOCSIZE_Y - y;
									tmpz = z - ( VoxelSector.ZVOXELBLOCSIZE_Z / 2 );
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;
					case VoxelSector.RelativeVoxelOrds.AHEAD:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ VoxelSector.FACEDRAW_Operations.AHEAD;
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = x - ( VoxelSector.ZVOXELBLOCSIZE_X / 2 );
									tmpy = y - ( VoxelSector.ZVOXELBLOCSIZE_Y / 2 );
									tmpz = z;
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;
					case VoxelSector.RelativeVoxelOrds.BEHIND:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ VoxelSector.FACEDRAW_Operations.BEHIND;
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = x - ( VoxelSector.ZVOXELBLOCSIZE_X / 2 );
									tmpy = y - ( VoxelSector.ZVOXELBLOCSIZE_Y / 2 );
									tmpz = VoxelSector.ZVOXELBLOCSIZE_Z - z;
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;

					case VoxelSector.RelativeVoxelOrds.LEFT_AHEAD:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ (VoxelSector.FACEDRAW_Operations.LEFT|VoxelSector.FACEDRAW_Operations.AHEAD);
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = VoxelSector.ZVOXELBLOCSIZE_X - x;
									tmpy = y - ( VoxelSector.ZVOXELBLOCSIZE_Y / 2 );
									tmpz = z;
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;
					case VoxelSector.RelativeVoxelOrds.RIGHT_AHEAD:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ ( VoxelSector.FACEDRAW_Operations.RIGHT | VoxelSector.FACEDRAW_Operations.AHEAD );
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = x;
									tmpy = y - ( VoxelSector.ZVOXELBLOCSIZE_Y / 2 );
									tmpz = z;
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;
					case VoxelSector.RelativeVoxelOrds.LEFT_BEHIND:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ ( VoxelSector.FACEDRAW_Operations.LEFT | VoxelSector.FACEDRAW_Operations.BEHIND );
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = VoxelSector.ZVOXELBLOCSIZE_X - x;
									tmpy = y - ( VoxelSector.ZVOXELBLOCSIZE_Y / 2 );
									tmpz = VoxelSector.ZVOXELBLOCSIZE_Z - z;
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;
					case VoxelSector.RelativeVoxelOrds.RIGHT_BEHIND:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ ( VoxelSector.FACEDRAW_Operations.RIGHT | VoxelSector.FACEDRAW_Operations.BEHIND );
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = x;
									tmpy = y - ( VoxelSector.ZVOXELBLOCSIZE_Y / 2 );
									tmpz = VoxelSector.ZVOXELBLOCSIZE_Z - z;
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;

					case VoxelSector.RelativeVoxelOrds.ABOVE_LEFT:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ ( VoxelSector.FACEDRAW_Operations.LEFT | VoxelSector.FACEDRAW_Operations.ABOVE );
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = VoxelSector.ZVOXELBLOCSIZE_X - x;
									tmpy = y;
									tmpz = z - ( VoxelSector.ZVOXELBLOCSIZE_Z / 2 );
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;
					case VoxelSector.RelativeVoxelOrds.ABOVE_RIGHT:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ ( VoxelSector.FACEDRAW_Operations.RIGHT | VoxelSector.FACEDRAW_Operations.ABOVE );
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = x;
									tmpy = y;
									tmpz = z - ( VoxelSector.ZVOXELBLOCSIZE_Z / 2 );
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;
					case VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ ( VoxelSector.FACEDRAW_Operations.BEHIND | VoxelSector.FACEDRAW_Operations.ABOVE );
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = x - ( VoxelSector.ZVOXELBLOCSIZE_X / 2 );
									tmpy = y;
									tmpz = VoxelSector.ZVOXELBLOCSIZE_Z - z;
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;
					case VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ ( VoxelSector.FACEDRAW_Operations.AHEAD | VoxelSector.FACEDRAW_Operations.ABOVE );
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									val = (ushort)( x * VoxelSector.ZVOXELBLOCSIZE_Y + y + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y );
									tmpx = x - ( VoxelSector.ZVOXELBLOCSIZE_X / 2 );
									tmpy = y;
									tmpz = z;
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;

					case VoxelSector.RelativeVoxelOrds.ABOVE_LEFT_AHEAD:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ ( VoxelSector.FACEDRAW_Operations.LEFT | VoxelSector.FACEDRAW_Operations.AHEAD | VoxelSector.FACEDRAW_Operations.ABOVE );
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = VoxelSector.ZVOXELBLOCSIZE_X - x;
									tmpy = y;
									tmpz = z;
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;
					case VoxelSector.RelativeVoxelOrds.ABOVE_RIGHT_AHEAD:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ ( VoxelSector.FACEDRAW_Operations.RIGHT | VoxelSector.FACEDRAW_Operations.AHEAD | VoxelSector.FACEDRAW_Operations.ABOVE );
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = x;
									tmpy = y;
									tmpz = z;
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;
					case VoxelSector.RelativeVoxelOrds.ABOVE_LEFT_BEHIND:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ ( VoxelSector.FACEDRAW_Operations.LEFT | VoxelSector.FACEDRAW_Operations.BEHIND |VoxelSector.FACEDRAW_Operations.ABOVE );
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									val = (ushort)( x * VoxelSector.ZVOXELBLOCSIZE_Y + y + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y );
									tmpx = VoxelSector.ZVOXELBLOCSIZE_X - x;
									tmpy = y;
									tmpz = VoxelSector.ZVOXELBLOCSIZE_Z - z;
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;
					case VoxelSector.RelativeVoxelOrds.ABOVE_RIGHT_BEHIND:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ ( VoxelSector.FACEDRAW_Operations.RIGHT | VoxelSector.FACEDRAW_Operations.BEHIND |VoxelSector.FACEDRAW_Operations.ABOVE );
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = x;
									tmpy = y;
									tmpz = VoxelSector.ZVOXELBLOCSIZE_Z - z;
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;

					case VoxelSector.RelativeVoxelOrds.BELOW_LEFT:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ ( VoxelSector.FACEDRAW_Operations.LEFT | VoxelSector.FACEDRAW_Operations.BELOW );
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = VoxelSector.ZVOXELBLOCSIZE_X - x;
									tmpy = VoxelSector.ZVOXELBLOCSIZE_Y - y;
									tmpz = z - ( VoxelSector.ZVOXELBLOCSIZE_Z / 2 );
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;
					case VoxelSector.RelativeVoxelOrds.BELOW_RIGHT:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ ( VoxelSector.FACEDRAW_Operations.RIGHT | VoxelSector.FACEDRAW_Operations.BELOW );
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = x;
									tmpy = VoxelSector.ZVOXELBLOCSIZE_Y - y;
									tmpz = z - ( VoxelSector.ZVOXELBLOCSIZE_Z / 2 );
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;
					case VoxelSector.RelativeVoxelOrds.BELOW_BEHIND:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ ( VoxelSector.FACEDRAW_Operations.BEHIND | VoxelSector.FACEDRAW_Operations.BELOW );
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = x - ( VoxelSector.ZVOXELBLOCSIZE_X / 2 );
									tmpy = VoxelSector.ZVOXELBLOCSIZE_Y - y;
									tmpz = VoxelSector.ZVOXELBLOCSIZE_Z - z;
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;
					case VoxelSector.RelativeVoxelOrds.BELOW_AHEAD:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ ( VoxelSector.FACEDRAW_Operations.AHEAD | VoxelSector.FACEDRAW_Operations.BELOW );
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = x - ( VoxelSector.ZVOXELBLOCSIZE_X / 2 );
									tmpy = VoxelSector.ZVOXELBLOCSIZE_Y - y;
									tmpz = z;
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;

					case VoxelSector.RelativeVoxelOrds.BELOW_LEFT_AHEAD:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ ( VoxelSector.FACEDRAW_Operations.LEFT | VoxelSector.FACEDRAW_Operations.BELOW | VoxelSector.FACEDRAW_Operations.AHEAD );
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = VoxelSector.ZVOXELBLOCSIZE_X - x;
									tmpy = VoxelSector.ZVOXELBLOCSIZE_Y - y;
									tmpz = z;
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;
					case VoxelSector.RelativeVoxelOrds.BELOW_RIGHT_AHEAD:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ ( VoxelSector.FACEDRAW_Operations.RIGHT | VoxelSector.FACEDRAW_Operations.BELOW | VoxelSector.FACEDRAW_Operations.AHEAD );
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = x;
									tmpy = VoxelSector.ZVOXELBLOCSIZE_Y - y;
									tmpz = z;
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;
					case VoxelSector.RelativeVoxelOrds.BELOW_LEFT_BEHIND:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ ( VoxelSector.FACEDRAW_Operations.LEFT | VoxelSector.FACEDRAW_Operations.BELOW | VoxelSector.FACEDRAW_Operations.BEHIND );
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = VoxelSector.ZVOXELBLOCSIZE_X - x;
									tmpy = VoxelSector.ZVOXELBLOCSIZE_Y - y;
									tmpz = VoxelSector.ZVOXELBLOCSIZE_Z - z;
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;
					case VoxelSector.RelativeVoxelOrds.BELOW_RIGHT_BEHIND:
						sorted_draw_infos[(int)i] = VoxelSector.FACEDRAW_Operations.ALL ^ ( VoxelSector.FACEDRAW_Operations.RIGHT | VoxelSector.FACEDRAW_Operations.BELOW | VoxelSector.FACEDRAW_Operations.BEHIND );
						for( x = 0; x < VoxelSector.ZVOXELBLOCSIZE_X; x++ )
						{
							xval = x * VoxelSector.ZVOXELBLOCSIZE_Y;
							for( z = 0; z < VoxelSector.ZVOXELBLOCSIZE_Z; z++ )
							{
								zval = xval + z * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
								for( y = 0; y < VoxelSector.ZVOXELBLOCSIZE_Y; y++ )
								{
									val = (ushort)( zval + y );
									tmpx = x;
									tmpy = VoxelSector.ZVOXELBLOCSIZE_Y - y;
									tmpz = VoxelSector.ZVOXELBLOCSIZE_Z - z;
									d = ( tmpx * tmpx ) + ( tmpy * tmpy ) + ( tmpz * tmpz );
									sorter.Add( d, val );
								}
							}
						}
						break;
				}

				n = VoxelSector.ZVOXELBLOCKCOUNT;
				foreach( ushort index in sorter )
				{
					// retrieved closest to furthest so... reverse storing it.
					//Log.log( "index is {0} {1} {2}"
					//	, ( index >> VoxelSector.ZVOXELBLOCSHIFT_Y ) & VoxelSector.ZVOXELBLOCMASK_X
					//	, index & VoxelSector.ZVOXELBLOCMASK_Y
					//	, ( index >> ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y ) ) & VoxelSector.ZVOXELBLOCMASK_Z );
					Indexes[--n] = index;
				}
				start_percent = ( ++start_step * 100 ) / start_steps;

				
				// strong test to make sure every offset is represented once.
				// paranoid debugging.
				/*
				if( i != VoxelSector.RelativeVoxelOrds.INCENTER )
				{
					int ofs, check;
					for( ofs = 0; ofs < VoxelSector.ZVOXELBLOCKCOUNT; ofs++ )
					{
						for( check = 0; check < VoxelSector.ZVOXELBLOCKCOUNT; check++ )
						{
							if( Indexes[check] == ofs )
								break;
						}
						if( check == VoxelSector.ZVOXELBLOCKCOUNT )
							Debugger.Break();
					}
				}
				*/
			}
		}

		void DetermineStaticGridBoundaries( Camera camera
										  , VoxelSector.RelativeVoxelOrds relpos
										  , VoxelSector sector )
		{
			switch( relpos )
			{

			}
		}



	}
}
