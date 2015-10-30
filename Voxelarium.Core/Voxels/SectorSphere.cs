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

		~SectorSphere()
		{
			SectorList = null;
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
