using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace Voxelarium.Core.Voxels
{
	class SectorSphere
	{
		public struct SphereEntry
		{
			public long x, y, z;
			public double SectorDistance;
		};

		protected SphereEntry[] SectorList;
		protected int nSlots;

		public int GetEntryCount() { return nSlots; }
		public void GetEntry( int EntryNum, out SphereEntry result ) { result = SectorList[EntryNum]; }

		
		protected void PartSort( int Start, int ItemCount, SphereEntry[] SortBuffer )
		{
			int i, FirstPartCount, SecondPartCount, FirstPartStart, SecondPartStart, EndPart;
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

		protected void Sort()
		{
			SphereEntry[] SortBuffer;
			SortBuffer = new SphereEntry[nSlots];
			if( nSlots > 0 ) PartSort( 0, nSlots, SortBuffer );
			//SortBuffer;
		}

		public SectorSphere()
		{
			SectorList = null;
			nSlots = 0;
		}

		~SectorSphere()
		{
			SectorList = null;
		}

		public void Init( int Render_Distance_h, int Render_Distance_v )
		{
			long x, y, z;
			ulong Offset;

			double dist_x, dist_y, dist_z;

			nSlots = ( Render_Distance_h * 2 + 1 ) * ( Render_Distance_h * 2 + 1 ) * ( Render_Distance_v * 2 + 1 );

			SectorList = new SphereEntry[nSlots];

			Offset = 0;
			for( x = -Render_Distance_h; x <= Render_Distance_h; x++ )
				for( y = -Render_Distance_v; y <= Render_Distance_v; y++ )
					for( z = -Render_Distance_h; z <= Render_Distance_h; z++ )
					{
						SectorList[Offset].x = x;
						SectorList[Offset].y = y;
						SectorList[Offset].z = z;
						dist_x = ( (double)( ( ( (long)x ) << ( VoxelGlobalSettings.VoxelBlockSizeBits + 4 ) ) ) );
						dist_y = ( (double)( ( ( (long)y ) << ( VoxelGlobalSettings.VoxelBlockSizeBits + 6 ) ) ) );
						dist_z = ( (double)( ( ( (long)z ) << ( VoxelGlobalSettings.VoxelBlockSizeBits + 4 ) ) ) );
						SectorList[Offset].SectorDistance = Math.Sqrt( dist_x * dist_x + dist_y * dist_y + dist_z * dist_z );
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
