//#define BT_USE_DOUBLE_PRECISION

using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text;

/*
Copyright (c) 2003-2009 Erwin Coumans  http://bullet.googlecode.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 
 This source version has been altered significantly from C++ to C#

 Now part of Voxelarium.
*/


namespace Voxelarium.LinearMath
{
	public interface btIScalar
	{
#if BT_USE_DOUBLE_PRECISION
		float
#else
			float
#endif		
				Value{ get; set; }
	}

	public struct btScalar : btIScalar
	{
#if BT_USE_DOUBLE_PRECISION
		float v;
		public const double SIMD_EPSILON = ( 0.0000001 );// double.Epsilon;
		public const double SIMD_LARGE_EPSILON = 0.0001;
        public const double SIMD_INFINITY = double.MaxValue;
		public const double SIMD_NEG_INFINITY = double.MinValue;
		public const double SIMDSQRT12 = 0.7071067811865475244008443621048490;
		public const double SIMD_PI = Math.PI;
		public const double SIMD_2_PI = ( 2.0 * SIMD_PI );
		public const double SIMD_HALF_PI = ( SIMD_PI * 0.5 );
		public const double SIMD_RADS_PER_DEG = ( SIMD_2_PI / 360.0 );
		public const double SIMD_DEGS_PER_RAD = ( 360.0 / SIMD_2_PI );
		public const double BT_MIN_FLOAT = double.MinValue;
		public const double BT_MAX_FLOAT = double.MaxValue;

		public const double BT_LARGE_FLOAT = 1e30;

		public const double BT_QUARTER = 0.25;
		public const double BT_HALF = 0.5;
		public const double BT_ZERO = 0.0;
		public const double BT_ZERO_NINE = 0.9;
		public const double BT_ZERO_THREE = 0.3;
		public const double BT_ONE = 1.0;
		public const double BT_NEG_ONE = 1.0;
		public const double BT_TWO = 2.0;
		public const double BT_TEN = 10.0;
		public const double BT_ONE_OVER_SIXTY = 1.0 / 60.0;

#else
		float v;
		public const float SIMD_EPSILON = float.Epsilon;
		public const float SIMD_INFINITY = float.MaxValue;
		public const float SIMDSQRT12 = 0.7071067811865475244008443621048490f;
		public const float SIMD_PI = (float)Math.PI;
		public const float SIMD_2_PI = ( 2.0f * SIMD_PI );
		public const float SIMD_HALF_PI = ( SIMD_PI * 0.5f );
		public const float SIMD_RADS_PER_DEG = ( SIMD_2_PI / 360.0f );
		public const float SIMD_DEGS_PER_RAD = ( 360.0f / SIMD_2_PI );

		public const float BT_MIN_FLOAT = float.MinValue;
		public const float BT_MAX_FLOAT = float.MaxValue;
		public const float BT_LARGE_FLOAT = 1e18f;

		public const float BT_QUARTER = 0.25f;
		public const float BT_HALF = 0.5f;
		public const float BT_ZERO = 0.0f;
		public const float BT_ONE = 1.0f;
		public const float BT_TWO = 2.0f;
		public const float BT_TEN = 10.0f;
		public const float BT_ONE_OVER_SIXTY = 1.0f / 60.0f;

#endif

#if !DISABLE_OPERATOR
		public static implicit operator btScalar( float v ) { btScalar tmp = new btScalar(); tmp.v = v; return tmp; }
		public static implicit operator float( btScalar v ) { return v.v; }
		public float Value { get { return v; } set { v = value; } }
#endif
		//use this, in case there are clashes (such as xnamath.h)

		//[MethodImpl( MethodImplOptions.AggressiveInlining )]
		public static float btSqrt( float x )
		{
			return (float)Math.Sqrt( x );
		}

		public static float btFabs( float x ) { return (float)Math.Abs( x ); }
		public static float btCos( float x ) { return (float)Math.Cos( x ); }
		public static float btSin( float x ) { return (float)Math.Sin( x ); }
		public static float btTan( float x ) { return (float)Math.Tan( x ); }

		public static float btAcos( float x )
		{
			if( x < -1 ) return SIMD_PI;
			if( x > 1 ) return 0;
			return (float)Math.Acos( x );
		}
		public static float btAsin( float x )
		{
			if( x < -1 ) return -SIMD_HALF_PI;
			if( x > 1 ) return SIMD_HALF_PI;
			return (float)Math.Asin( x );
		}
		public static float btAtan( float x ) { return (float)Math.Atan( x ); }
		public static float btAtan2( float x, float y ) { return (float)Math.Atan2( x, y ); }
		public static float btExp( float x ) { return (float)Math.Exp( x ); }
		public static float btLog( float x ) { return (float)Math.Log( x ); }
		public static float btPow( float x, float y ) { return (float)Math.Pow( x, y ); }
		public static float btFmod( float x, float y ) { return x % y; }

		public static void btSetMax( ref float a, float b )
		{
			if( a < b )
			{
				a = b;
			}
		}
		public static void btSetMin( ref float a, float b )
		{
			if( a > b )
			{
				a = b;
			}
		}
		public static float btRecipSqrt( float x )
		{
			return ( BT_ONE / (float)btSqrt( x ) );     // reciprocal square root 
		}

		public static float btMin( float a, float b )
		{
			return a < b ? a : b;
		}

		public static int btMin( int a, int b )
		{
			return a < b ? a : b;
		}

		public static float btMax( float a, float b )
		{
			return a > b ? a : b;
		}


		public static float btClamped( float a, float lb, float ub )
		{
			return a < lb ? lb : ( ub < a ? ub : a );
		}

		public static void btClamp( ref float a, float lb, float ub )
		{
			if( a < lb )
			{
				a = lb;
			}
			else if( ub < a )
			{
				a = ub;
			}
		}

		public static float btFsels( float a, float b, float c )
		{
			return a >= 0 ? b : c;
		}

		public static float btRecip( float a )
		{
			return (float)1.0 / a;
		}


		public static float btAtan2Fast( float y, float x )
		{
			float coeff_1 = SIMD_PI / 4.0f;
			float coeff_2 = 3.0f * coeff_1;
			float abs_y = btFabs( y );
			float angle;
			if( x >= 0.0f )
			{
				float r = ( x - abs_y ) / ( x + abs_y );
				angle = coeff_1 - coeff_1 * r;
			}
			else
			{
				float r = ( x + abs_y ) / ( abs_y - x );
				angle = coeff_2 - coeff_1 * r;
			}
			return ( y < 0.0f ) ? -angle : angle;
		}

		public static bool btFuzzyZero( float x ) { return btFabs( x ) < SIMD_EPSILON; }

		public static bool btEqual( float a, float eps )
		{
			return ( ( ( a ) <= eps ) & !( ( a ) < -eps ) );
		}
		public static bool btGreaterEqual( float a, float eps )
		{
			return ( !( ( a ) <= eps ) );
		}


		public static int btIsNegative( float x )
		{
			return x < 0.0 ? 1 : 0;
		}

		public static float btRadians( float x ) { return x * SIMD_RADS_PER_DEG; }
		public static float btDegrees( float x ) { return x * SIMD_DEGS_PER_RAD; }

		public static bool btMachineIsLittleEndian()
		{
			return BitConverter.IsLittleEndian;
		}



		///btSelect avoids branches, which makes performance much better for consoles like Playstation 3 and XBox 360
		///Thanks Phil Knight. See also http://www.cellperformance.com/articles/2006/04/more_techniques_for_eliminatin_1.html
		public static uint btSelect( uint condition, uint valueIfConditionNonZero, uint valueIfConditionZero )
		{
			// Set testNz to 0xFFFFFFFF if condition is nonzero, 0x00000000 if condition is zero
			// Rely on positive value or'ed with its negative having sign bit on
			// and zero value or'ed with its negative (which is still zero) having sign bit off 
			// Use arithmetic shift right, shifting the sign bit through all 32 bits
			uint testNz = (uint)( ( (int)condition | -(int)condition ) >> 31 );
			uint testEqz = ~testNz;
			return ( ( valueIfConditionNonZero & testNz ) | ( valueIfConditionZero & testEqz ) );
		}
		int btSelect( uint condition, int valueIfConditionNonZero, int valueIfConditionZero )
		{
			uint testNz = (uint)( ( (int)condition | -(int)condition ) >> 31 );
			uint testEqz = ~testNz;
			return (int)( ( valueIfConditionNonZero & testNz ) | ( valueIfConditionZero & testEqz ) );
		}
		public static float btSelect( uint condition, float valueIfConditionNonZero, float valueIfConditionZero )
		{
			return (float)btFsels( (float)condition - (float)( 1.0f ), valueIfConditionNonZero, valueIfConditionZero );
		}

		public static void btSwap<T>( ref T a, ref T b )
		{
			T tmp = a;
			a = b;
			b = tmp;
		}

#if ENDIAN_SWAP_ALLOW
		//PCK: endian swapping functions
		uint btSwapEndian( uint val )
		{
			return ( ( ( val & 0xff000000 ) >> 24 ) | ( ( val & 0x00ff0000 ) >> 8 ) | ( ( val & 0x0000ff00 ) << 8 ) | ( ( val & 0x000000ff ) << 24 ) );
		}

		ushort btSwapEndian( ushort val )
		{
			return (ushort)( ( ( val & 0xff00 ) >> 8 ) | ( ( val & 0x00ff ) << 8 ) );
		}

		uint btSwapEndian( int val )
		{
			return btSwapEndian( (uint)val );
		}

		ushort btSwapEndian( short val )
		{
			return btSwapEndian( (ushort)val );
		}

		///btSwapFloat uses using char pointers to swap the endianness
		////btSwapFloat/btSwapDouble will NOT return a float, because the machine might 'correct' invalid floating point values
		///Not all values of sign/exponent/mantissa are valid floating point numbers according to IEEE 754. 
		///When a floating point unit is faced with an invalid value, it may actually change the value, or worse, throw an exception. 
		///In most systems, running user mode code, you wouldn't get an exception, but instead the hardware/os/runtime will 'fix' the number for you. 
		///so instead of returning a float/float, we return integer/long long integer
		unsafe uint btSwapEndianFloat( float d )
		{
			uint a = 0;
			byte* dst = (byte*)&a;
			byte* src = (byte*)&d;

			dst[0] = src[3];
			dst[1] = src[2];
			dst[2] = src[1];
			dst[3] = src[0];
			return a;
		}

		// unswap using char pointers
		unsafe float btUnswapEndianFloat( uint a )
		{
			float d = 0.0f;
			byte* src = (byte*)a;
			byte* dst = (byte*)&d;

			dst[0] = src[3];
			dst[1] = src[2];
			dst[2] = src[1];
			dst[3] = src[0];

			return d;
		}


		// swap using char pointers
		unsafe void btSwapEndianDouble( float d, byte* dst )
		{
			byte* src = (byte*)&d;

			dst[0] = src[7];
			dst[1] = src[6];
			dst[2] = src[5];
			dst[3] = src[4];
			dst[4] = src[3];
			dst[5] = src[2];
			dst[6] = src[1];
			dst[7] = src[0];

		}

		// unswap using char pointers
		unsafe float btUnswapEndianDouble( byte* src )
		{
			float d = 0.0;
			byte* dst = (byte*)&d;

			dst[0] = src[7];
			dst[1] = src[6];
			dst[2] = src[5];
			dst[3] = src[4];
			dst[4] = src[3];
			dst[5] = src[2];
			dst[6] = src[1];
			dst[7] = src[0];

			return d;
		}
#endif
#if asdfasdf
		unsafe void btSetZero<T>( void* a, int n )
		{
			T* acurr = a;
			uint ncurr = n;
			while( ncurr > 0 )
			{
				( acurr++ ) = 0;
				--ncurr;
			}
		}
#endif
#if UNSAFE_ALLOWED
		unsafe float btLargeDot( float* a, float* b, int n )
		{
			float p0, q0, m0, p1, q1, m1, sum;
			sum = 0;
			n -= 2;
			while( n >= 0 )
			{
				p0 = a[0]; q0 = b[0];
				m0 = p0 * q0;
				p1 = a[1]; q1 = b[1];
				m1 = p1 * q1;
				sum += m0;
				sum += m1;
				a += 2;
				b += 2;
				n -= 2;
			}
			n += 2;
			while( n > 0 )
			{
				sum += ( a[0] ) * ( b[0] );
				a++;
				b++;
				n--;
			}
			return sum;
		}
#endif

		// returns normalized value in range [-SIMD_PI, SIMD_PI]
		public static float btNormalizeAngle( float angleInRadians )
		{
			angleInRadians = btFmod( angleInRadians, SIMD_2_PI );
			if( angleInRadians < -SIMD_PI )
			{
				return angleInRadians + SIMD_2_PI;
			}
			else if( angleInRadians > SIMD_PI )
			{
				return angleInRadians - SIMD_2_PI;
			}
			else
			{
				return angleInRadians;
			}
		}
	}
}
