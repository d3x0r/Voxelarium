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
using System.Text;

namespace Voxelarium.Core.Voxels.Types
{
	class LinearInterpolator
	{
		float[] ValuesTable;
		uint nValues;
		float Start, End;
		float DeltaVal, Factor;

		public LinearInterpolator() { ValuesTable = null; nValues = 0; Start = 0; End = 0; }
		public LinearInterpolator( float[] ValuesTable, uint ValuesCount ) { this.ValuesTable = ValuesTable; nValues = ValuesCount; Start = 0; End = 0; }

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public void SetValuesTable( float[] ValuesTable, uint ValuesCount ) { this.ValuesTable = ValuesTable; nValues = ValuesCount; }

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public void SetBounds( float Start, float End )
		{
			this.Start = Start;
			this.End = End;
			DeltaVal = End - Start;
			Factor = ( (float)( nValues - 1 ) ) / DeltaVal;
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public float Interpolate( float Position )
		{
			float TablePos, FracPos;

			uint P1, P2;
			float Value1, Value2, FinalValue;

			if( Position < Start ) Position = Start;
			if( Position > End ) Position = End;

			TablePos = ( Position - Start ) * Factor;

			P1 = (uint)Math.Floor( TablePos );
			P2 = (uint)Math.Ceiling( TablePos );
			FracPos = TablePos - (float)( TablePos );
			if( P2 >= nValues ) P2 = nValues - 1;

			Value1 = ValuesTable[P1];
			Value2 = ValuesTable[P2];

			FinalValue = (float)( Value1 * ( 1.0 - FracPos ) ) + ( Value2 * FracPos );

			return ( FinalValue );
		}

	}
}
