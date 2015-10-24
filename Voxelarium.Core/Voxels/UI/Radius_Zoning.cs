using Bullet.LinearMath;
using System;
using System.Collections.Generic;
using System.Text;
using Voxelarium.Core.Support;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.Voxels.UI
{
	internal class Radius_Zoning
	{
		byte[] ZoneData;
		uint ZoneSize;
		ZVector3L Size;
		ZVector3L ArraySize;

		internal Radius_Zoning()
		{
			ZoneData = null;
			ZoneSize = 0;
		}

		~Radius_Zoning()
		{
			ZoneData = null;
		}

		internal void SetSize( int Size_x, int Size_y, int Size_z )
		{
			int i;

			Size.x = Size_x; Size.y = Size_y; Size.z = Size_z;
			ArraySize.x = ( Size.x << 1 ) + 1;
			ArraySize.y = ( Size.y << 1 ) + 1;
			ArraySize.z = ( Size.z << 1 ) + 1;

			ZoneSize = (uint)( ArraySize.x * ArraySize.y * ArraySize.z );
			ZoneData = new byte[ZoneSize];

			for( i = 0; i < ZoneSize; i++ ) ZoneData[i] = 0;
		}



		internal void DrawZones( double ZoneRadius, byte ZoneMark )
		{
			int x, y, z;
			uint Off_y, Off_z;
			uint Inc_y, Inc_z;
			btVector3 Vector;
			double Distance;

			Inc_y = (uint)ArraySize.x;
			Inc_z = (uint)(ArraySize.x * ArraySize.y);

			for( x = 0; x < ArraySize.x; x++ )
				for( y = 0, Off_y = 0; y < ArraySize.y; y++, Off_y += Inc_y )
					for( z = 0, Off_z = 0; z < ArraySize.z; z++, Off_z += Inc_z )
					{
						Vector.x = x - Size.x; Vector.y = y - Size.y; Vector.z = z - Size.z;
						Distance = Math.Sqrt( Vector.x * Vector.x + Vector.y * Vector.y + Vector.z * Vector.z );
						if( Distance < ZoneRadius ) ZoneData[x + Off_y + Off_z] = ZoneMark;
					}

		}

		/*
			void DrawZones( double Zone1, double Zone2, double Zone3, double Zone4)
			{
			  double x,y,z;
			  ULong  xe,ye,ze;
			  double sx = (double)Size_x,sy = (double)Size_y,sz = (double)Size_z;
			  double mpx = sx / 2 - 0.5, mpy = sy / 2 - 0.5, mpz = sz / 2 - 0.5;
			  double pointdist;
			  UByte  Zone;
			  ULong  Offset;

			  double Probe[20];

			  for (x=0,xe=0 ; xe<Size_x ; x++,xe++)
				for (y=0,ye=0 ; ye<Size_y ; y++,ye++)
				  for (z=0,ze=0 ; ze<Size_z ; z++,ze++)
				  {
					pointdist = sqrt(  ((mpx-x) * (mpx-x)) + ((mpy-y) * (mpy-y)) + ((mpz-z) * (mpz-z)) );
					if ( y == (Size_y/2)  && x == (Size_x/2))
					{
					  Probe[ze] = pointdist;
					}
					if      (pointdist <Zone4 ) Zone = 4;
					else if (pointdist <Zone3 ) Zone = 3;
					else if (pointdist <Zone2 ) Zone = 2;
					else if (pointdist <Zone1 ) Zone = 1;
					else                        Zone = 0;

					Offset = xe + ye * Size_x + ze * (Size_x * Size_y);
					ZoneData[Offset] = Zone;
				  }

			  // printf("Probe:");
			  // for ( Offset=0 ; Offset<20 ; Offset++) printf(" %ld:[%lf]", Offset, Probe[Offset]);
			}
		*/
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal int GetZone( int x, int y, int z )
		{
			int Offset = Size.x + x + ( Size.y + y ) * ArraySize.x + ( Size.z + z ) * ( ArraySize.x * ArraySize.y );

			return ( ZoneData[Offset] );
		}

		internal void DebugOut()
		{
			int x, y, z;
			uint Offset;
#if asdfasdf
			Log.log( "---------------------------------------------------\n" );
			y = ArraySize.y / 2;
			for( x = 0; x < ArraySize.x; x++ )
			{
				for( z = 0; z < ArraySize.z; z++ )
				{
					Offset = x + y * ArraySize.x + z * ( ArraySize.x * ArraySize.y );
					printf( "%d", ZoneData[Offset] );
				}
				printf( "\n" );
			}

			z = ArraySize.z / 2;
			for( x = 0; x < ArraySize.x; x++ )
			{
				for( y = 0; y < ArraySize.y; y++ )
				{
					Offset = x + y * ArraySize.x + z * ( ArraySize.x * ArraySize.y );
					printf( "%d", ZoneData[Offset] );
				}
				printf( "\n" );
			}
#endif
		}
	}
}
