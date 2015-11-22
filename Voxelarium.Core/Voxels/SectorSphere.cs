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
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace Voxelarium.Core.Voxels
{
	internal class SectorSphere
	{
		public struct SphereEntry
		{
			public int x, y, z;
			public float SectorDistance;
			public VoxelSector.RelativeVoxelOrds relative_pos;
		};

		static SphereEntry[] SectorList;
		static uint nSlots;

		public static uint GetEntryCount() { return nSlots; }
		public static void GetEntry( int EntryNum, out SphereEntry result ) { result = SectorList[EntryNum]; }
		
		static void PartSort( uint Start, uint ItemCount, SphereEntry[] SortBuffer )
		{
			uint i, FirstPartCount, SecondPartCount, FirstPartStart, SecondPartStart, EndPart;
			if( ItemCount <= 1 )
			{
				return;
			}
			SecondPartCount = ItemCount / 2;
			FirstPartCount = ItemCount - SecondPartCount;
			FirstPartStart = Start;
			SecondPartStart = Start + FirstPartCount;
			EndPart = FirstPartStart + ItemCount;

			// Sort subtables

			PartSort( FirstPartStart, FirstPartCount, SortBuffer );
			PartSort( SecondPartStart, SecondPartCount, SortBuffer );

			// Copy first partition into buffer

			for( i = Start; i < SecondPartStart; i++ )
			{
				SortBuffer[i] = SectorList[i];
			}

			// Make partition fusion

			for( i = Start; i < EndPart; i++ )
			{
				if( FirstPartCount > 0 && SecondPartCount > 0 )
				{
					if( SortBuffer[FirstPartStart].SectorDistance <= SectorList[SecondPartStart].SectorDistance ) { SectorList[i] = SortBuffer[FirstPartStart++]; FirstPartCount--; }
					else { SectorList[i] = SectorList[SecondPartStart++]; SecondPartCount--; }
				}
				else
				{
					if( FirstPartCount != 0 ) { SectorList[i] = SortBuffer[FirstPartStart++]; FirstPartCount--; }
					else { SectorList[i] = SectorList[SecondPartStart++]; SecondPartCount--; }
				}
			}
		}

		static void Sort()
		{
			SphereEntry[] SortBuffer;
			SortBuffer = new SphereEntry[nSlots];
			if( nSlots > 0 ) PartSort( 0, nSlots, SortBuffer );
			//SortBuffer;
		}

		public static void Init( uint Render_Distance_h, uint Render_Distance_v )
		{
			int x, y, z;
			ulong Offset;

			float dist_x, dist_y, dist_z;

			nSlots = ( Render_Distance_h * 2 + 1 ) * ( Render_Distance_h * 2 + 1 ) * ( Render_Distance_v * 2 + 1 );

			SectorList = new SphereEntry[nSlots];

			Offset = 0;
			for( x = -(int)Render_Distance_h; x <= Render_Distance_h; x++ )
				for( y = -(int)Render_Distance_v; y <= Render_Distance_v; y++ )
					for( z = -(int)Render_Distance_h; z <= Render_Distance_h; z++ )
					{
						SectorList[Offset].x = x;
						SectorList[Offset].y = y;
						SectorList[Offset].z = z;
						dist_x = ( (float)( ( ( (long)x ) << ( VoxelGlobalSettings.WorldVoxelBlockSizeBits + VoxelSector.ZVOXELBLOCSHIFT_X ) ) ) );
						dist_y = ( (float)( ( ( (long)y ) << ( VoxelGlobalSettings.WorldVoxelBlockSizeBits + VoxelSector.ZVOXELBLOCSHIFT_Y ) ) ) );
						dist_z = ( (float)( ( ( (long)z ) << ( VoxelGlobalSettings.WorldVoxelBlockSizeBits + VoxelSector.ZVOXELBLOCSHIFT_Z ) ) ) );
						SectorList[Offset].SectorDistance = (float)Math.Sqrt( dist_x * dist_x + dist_y * dist_y + dist_z * dist_z );
						if( x < 0 )
							if( y < 0 )
								if( z < 0 )      SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.BEHIND_BELOW_LEFT;
								else if( z > 0 ) SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.AHEAD_BELOW_LEFT;
								else             SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.BELOW_LEFT;
							else if( y > 0 )
								if( z < 0 )      SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.BEHIND_ABOVE_LEFT;
								else if( z > 0 ) SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.AHEAD_ABOVE_LEFT;
								else             SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.ABOVE_LEFT;
							else
								if( z < 0 )      SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.BEHIND_LEFT;
								else if( z > 0 ) SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.AHEAD_LEFT;
								else             SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.LEFT;
						else if( x > 0 )
							if( y < 0 )
								if( z < 0 )      SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.BEHIND_BELOW_RIGHT;
								else if( z > 0 ) SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.AHEAD_BELOW_RIGHT;
								else             SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.BELOW_RIGHT;
							else if( y > 0 )
								if( z < 0 )      SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.BEHIND_ABOVE_RIGHT;
								else if( z > 0 ) SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.AHEAD_ABOVE_RIGHT;
								else             SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.ABOVE_RIGHT;
							else
								if( z < 0 )      SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT;
								else if( z > 0 ) SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT;
								else             SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.RIGHT;
						else
							if( y < 0 )
								if( z < 0 )      SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.BEHIND_BELOW;
								else if( z > 0 ) SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.AHEAD_BELOW;
								else             SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.BELOW;
							else if( y > 0 )
								if( z < 0 )      SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.BEHIND_ABOVE;
								else if( z > 0 ) SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.AHEAD_ABOVE;
								else             SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.ABOVE;
							else
								if( z < 0 )      SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.BEHIND;
								else if( z > 0 ) SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.AHEAD;
								else             SectorList[Offset].relative_pos = VoxelSector.RelativeVoxelOrds.INCENTER;
						Offset++;
					}

			// Sort the list;

			Sort();

		}

		public void debugout( string FileSpec )
		{
			StreamWriter sw = new StreamWriter( FileSpec, false );
			int i;
			for( i = 0; i < nSlots; i++ )
			{
				sw.WriteLine( "X:{0} Y:{1} Z:{2} Dist: {3}"
						, SectorList[i].x, SectorList[i].y, SectorList[i].z, SectorList[i].SectorDistance );
			}
			sw.Dispose();
		}

	}
}
