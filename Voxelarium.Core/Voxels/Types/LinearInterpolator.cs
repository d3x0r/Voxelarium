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
