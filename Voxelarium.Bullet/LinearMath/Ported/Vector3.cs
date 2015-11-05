//#define DISABLE_OPERATORS
/*
 Copyright (c) 2011 Apple Inc.
 http://continuousphysics.com/Bullet/
 
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose, 
 including commercial applications, and to alter it and redistribute it freely, 
 subject to the following restrictions:
 
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 
 This source version has been altered.
 */

using System;
using System.Diagnostics;

namespace Bullet.LinearMath
{
	/*
    // how-to-inline C# 4.5 (mono-2.0?) 2012
    [MethodImpl( MethodImplOptions.AggressiveInlining )]
    */
#if IMPLEMENT_AS_ABSTRACT
		public abstract class
#else
	public interface
#endif
		btIVector3
	{
		double this[int n] { get; set; }
		double X { get; set; }
		double Y { get; set; }
		double Z { get; set; }
		double W { get; set; }

		double R { get; set; }
		double G { get; set; }
		double B { get; set; }
		double A { get; set; }
		void Copy( out btVector3 result );

		/*@brief Add a vector to this one 
          @param The vector to add to this one */
		void Add( ref btVector3 b, out btVector3 result );
		void AddScale( ref btVector3 b, double s, out btVector3 result );
		void AddAndScale( ref btVector3 b, double s, out btVector3 result );

		void Sub( ref btVector3 b, out btVector3 result );
		void Sub( btIVector3 b, out btVector3 result );

		void SubScale( ref btVector3 b, double s, out btVector3 result );
		void SubAndScale( ref btVector3 b, double s, out btVector3 result );

		void Mult( double b, out btVector3 result );

		void Div( double b, out btVector3 result );

		/*@brief Return the dot product
          @param v The other vector in the dot product */
		double dot( ref btVector3 v );

		/*@brief Return the length of the vector squared */
		double length2();

		/*@brief Return the length of the vector */
		double length();

		/*@brief Return the norm (length); of the vector */
		double norm();

		/*@brief Return the distance squared between the ends of this and another vector
          This is symantically treating the vector like a point */
		double distance2( ref btVector3 v );

		/*@brief Return the distance between the ends of this and another vector
          This is symantically treating the vector like a point */
		double distance( ref btVector3 v );

		btVector3 safeNormalize();

		void setValue( double _x, double _y, double _z );

		/*@brief Return a vector will the absolute values of each element */
		btVector3 absolute();

		/*@brief Normalize this vector 
          x^2 + y^2 + z^2 = 1 */
		btVector3 normalize();

		bool IsAlmostZero();

		bool fuzzyZero();

		/*@brief Return a normalized version of this vector */
		void normalized( out btVector3 result );
		/*@brief Return a rotated version of this vector
          @param wAxis The axis to rotate about 
          @param angle The angle to rotate by */
		void rotate( ref btVector3 wAxis, double angle, out btVector3 result );
		/*@brief Return the cross product between this and another vector 
          @param v The other vector */
		void cross( ref btVector3 v, out btVector3 result );

		/*@brief Return the angle between this and another vector
          @param v The other vector */
		double angle( ref btVector3 v );


		double triple( ref btVector3 v1, ref btVector3 v2 );

		/*@brief Return the axis with the smallest value 
          Note return values are 0,1,2 for x, y, or z */
		int minAxis();

		/*@brief Return the axis with the largest value 
          Note return values are 0,1,2 for x, y, or z */
		int maxAxis();
		int furthestAxis();

		int closestAxis();


		/*@brief Return the linear interpolation between this and another vector 
          @param v The other vector 
          @param t The ration of this to v (t = 0 => return this, t=1 => return other); */
		void lerp( ref btVector3 v, double t, out btVector3 result );

		bool Equals( ref btVector3 other );

		/*@brief Set each element to the max of the current values and the values of another btVector3
		  @param other The other btVector3 to compare with 
		 */
		void setMax( ref btVector3 other );

		/*@brief Set each element to the min of the current values and the values of another btVector3
		  @param other The other btVector3 to compare with 
		 */
		void setMin( ref btVector3 other );

		void getSkewSymmetricMatrix( out btVector3 v0, out btVector3 v1, out btVector3 v2 );

		void setZero();

		bool isZero();


		/* create a vector as  btVector3( this.dot( btVector3 v0 );, this.dot( btVector3 v1);, this.dot( btVector3 v2 ););  */
		void dot3( ref btVector3 v0, ref btVector3 v1, ref btVector3 v2, out btVector3 result );
		void dot3( ref btMatrix3x3 m, out btVector3 result );

		/*@brief Return the elementwise product of two vectors */
		void Mult( ref btVector3 v2, out btVector3 result );

		void Invert( out btVector3 result );


		/*@brief Return the vector inversely scaled by s */
		void Div( ref btVector3 v2, out btVector3 result );

		/*@brief Return the angle between two vectors */
		double btAngle( ref btVector3 v1, ref btVector3 v2 );


		long maxDot( btVector3[] array, long array_count, out double dotOut );

		long minDot( btVector3[] array, long array_count, ref double dotOut );

	}


	public struct btVector3 : btIVector3
	{
		public double x;
		public double y;
		public double z;
		public double w;

		public double X { get { return x; } set { x = value; } }
		public double Y { get { return y; } set { y = value; } }
		public double Z { get { return z; } set { z = value; } }
		public double W { get { return w; } set { w = value; } }

		public double R { get { return x; } set { x = value; } }
		public double G { get { return y; } set { y = value; } }
		public double B { get { return z; } set { x = value; } }
		public double A { get { return w; } set { w = value; } }

		public void Copy( out btVector3 result ) { result = this; }

		public static btVector3 Zero = new btVector3();
		public static btVector3 Right = new btVector3( 1, 0, 0 );
		public static btVector3 Up = new btVector3( 0, 1, 0 );
		public static btVector3 Forward = new btVector3( 0, 0, 1 );
		public static btVector3 One = new btVector3( 1, 1, 1 );
		public static btVector3 NegOne = new btVector3( -1, -1, -1 );
		public static btVector3 Max = new btVector3( btScalar.BT_MAX_FLOAT, btScalar.BT_MAX_FLOAT, btScalar.BT_MAX_FLOAT );
		public static btVector3 Min = new btVector3( btScalar.BT_MIN_FLOAT, btScalar.BT_MIN_FLOAT, btScalar.BT_MIN_FLOAT );
		public static btVector3 xAxis = new btVector3( 1, 0, 0 );
		public static btVector3 yAxis = new btVector3( 0, 1, 0 );
		public static btVector3 zAxis = new btVector3( 0, 0, 1 );
		public static btVector3 wAxis = new btVector3( 0, 0, 0, 1 );

		public btVector3( double s )
		{
			x = s; y = s; z = s; w = s;
		}

		public btVector3( double x, double y, double z )
		{
			this.x = x; this.y = y; this.z = z; this.w = 0;
		}
		public btVector3( double x, double y, double z, double w )
		{
			this.x = x; this.y = y; this.z = z; this.w = w;
		}
		public btVector3( ref btVector3 v )
		{
			this.x = v.x; this.y = v.y; this.z = v.z; this.w = v.w;
		}
		public double this[int n]
		{
			get
			{
				switch( n )
				{
					default:
					case 0: return x;
					case 1: return y;
					case 2: return z;
					case 3: return w;
				}
			}
			set
			{
				switch( n )
				{
					default:
					case 0: x = value; break;
					case 1: y = value; break;
					case 2: z = value; break;
					case 3: w = value; break;
				}
			}
		}

		/*@brief Add a vector to this one 
          @param The vector to add to this one */
		public void Add( ref btVector3 b, out btVector3 result )
		{
			result.x = x + b.x;
			result.y = y + b.y;
			result.z = z + b.z;
			result.w = w + b.w;
		}
		public void Add( btIVector3 b, out btVector3 result )
		{
			result.x = x + b.X;
			result.y = y + b.Y;
			result.z = z + b.Z;
			result.w = w + b.W;
		}
		public void AddScale( ref btVector3 b, double s, out btVector3 result )
		{
			result.x = x + b.x * s;
			result.y = y + b.y * s;
			result.z = z + b.z * s;
			result.w = w + b.w * s;
		}
		public void AddScale( btIVector3 b, double s, out btVector3 result )
		{
			result.x = x + b[0] * s;
			result.y = y + b[1] * s;
			result.z = z + b[2] * s;
			result.w = w + b[3] * s;
		}
		public void AddAndScale( ref btVector3 b, double s, out btVector3 result )
		{
			result.x = ( x + b.x ) * s;
			result.y = ( y + b.y ) * s;
			result.z = ( z + b.z ) * s;
			result.w = ( w + b.w ) * s;
		}

		public void Sub( ref btVector3 b, out btVector3 result )
		{
			result.x = x - b.x;
			result.y = y - b.y;
			result.z = z - b.z;
			result.w = w - b.w;
		}

		public void Sub( btIVector3 b, out btVector3 result )
		{
			result.x = x - b.X;
			result.y = y - b.Y;
			result.z = z - b.Z;
			result.w = w - b.W;
		}

		public void SubScale( ref btVector3 b, double s, out btVector3 result )
		{
			result.x = x - b.x * s;
			result.y = y - b.y * s;
			result.z = z - b.z * s;
			result.w = w - b.w * s;
		}
		public void SubAndScale( ref btVector3 b, double s, out btVector3 result )
		{
			result.x = ( x - b.x ) * s;
			result.y = ( y - b.y ) * s;
			result.z = ( z - b.z ) * s;
			result.w = ( w - b.w ) * s;
		}

		public void Mult( double b, out btVector3 result )
		{
			result.x = x * b;
			result.y = y * b;
			result.z = z * b;
			result.w = w * b;
		}

		public void Div( double b, out btVector3 result )
		{
#if PARANOID_ASSERTS
			Debug.Assert( b != 0 );
#endif
			Mult( ( 1 / b ), out result );
		}

		/*@brief Return the dot product
          @param v The other vector in the dot product */
		public double dot( ref btVector3 v )
		{
			return x * v.x +
					y * v.y +
					z * v.z;
		}
		public double dot( btIVector3 v )
		{
			return x * v.X +
					y * v.Y +
					z * v.Z;
		}
		public static double dot( ref btVector3 a, ref btVector3 b )
		{
			return a.x * b.x +
					a.y * b.y +
					a.z * b.z;
		}

		/*@brief Return the length of the vector squared */
		public double length2()
		{
			return dot( ref this );
		}

		public static double BetweenLength2( ref btVector3 min, ref btVector3 max )
		{
			btVector3 tmp;
			max.Sub( ref min, out tmp );
			return tmp.dot( ref tmp );
		}
		public static double btDelLength2( ref btVector3 max, ref btVector3 min )
		{
			btVector3 tmp;
			max.Sub( ref min, out tmp );
			return tmp.dot( ref tmp );
		}
		/*@brief Return the length of the vector */
		public double length()
		{
			return btScalar.btSqrt( length2() );
		}

		/*@brief Return the norm (length) of the vector */
		public double norm()
		{
			return length();
		}

		/*@brief Return the distance squared between the ends of this and another vector
          This is symantically treating the vector like a point */
		public double distance2( ref btVector3 v )
		{
			btVector3 tmp;
			v.Sub( ref this, out tmp );
			return tmp.length2();
		}

		/*@brief Return the distance between the ends of this and another vector
          This is symantically treating the vector like a point */
		public double distance( ref btVector3 v )
		{
			btVector3 tmp;
			v.Sub( ref this, out tmp );
			return tmp.length();
		}

		public btVector3 safeNormalize()
		{
			btVector3 absVec = this.absolute();
			int maxIndex = absVec.maxAxis();
			if( absVec[maxIndex] > 0 )
			{
				this.Div( absVec[maxIndex], out this );
				this.Div( length(), out this );
				return this;
			}
			setValue( 1, 0, 0 );
			return this;
		}

		public void setValue( double _x, double _y, double _z )
		{
			x = _x;
			y = _y;
			z = _z;
			w = 0;
		}

		public static void setValue( out btVector3 result, double _x, double _y, double _z )
		{
			result.x = _x;
			result.y = _y;
			result.z = _z;
			result.w = 0;
		}
		public static void setValue( out btVector3 result, double _x, double _y, double _z, double _w )
		{
			result.x = _x;
			result.y = _y;
			result.z = _z;
			result.w = _w;
		}

		/*@brief Return a vector will the absolute values of each element */
		public btVector3 absolute()
		{
			return new btVector3( btScalar.btFabs( x ), btScalar.btFabs( y ), btScalar.btFabs( z ) );
		}

		/*@brief Normalize this vector 
          x^2 + y^2 + z^2 = 1 */
		public btVector3 normalize()
		{
			Debug.Assert( !fuzzyZero() );
			this.Div( length(), out this );
			return this;
		}

		public static bool IsAlmostZero( ref btVector3 v )
		{
			if( btScalar.btFabs( v.x ) > 1e-6 || btScalar.btFabs( v.y ) > 1e-6 || btScalar.btFabs( v.z ) > 1e-6 ) return false;
			return true;
		}
		public bool IsAlmostZero()
		{
			if( btScalar.btFabs( x ) > 1e-6 || btScalar.btFabs( y ) > 1e-6 || btScalar.btFabs( z ) > 1e-6 ) return false;
			return true;
		}

		public bool fuzzyZero()
		{
			return length2() < btScalar.SIMD_EPSILON * btScalar.SIMD_EPSILON;
		}

		/*@brief Return a normalized version of this vector */
		public void normalized( out btVector3 result )
		{
			btVector3 nrm = new btVector3( ref this );
			result = nrm.normalize();
		}

		/*@brief Return a rotated version of this vector
          @param wAxis The axis to rotate about 
          @param angle The angle to rotate by */
		public void rotate( ref btVector3 wAxis, double angle, out btVector3 result )
		{
			btVector3 o;
			wAxis.Mult( wAxis.dot( ref this ), out o );
			btVector3 _x;
			this.Sub( ref o, out _x );
			btVector3 _y;

			wAxis.cross( ref this, out _y );
			btVector3 tmp;
			btVector3 tmp2;
			_x.Mult( btScalar.btCos( angle ), out tmp );
			o.Add( ref tmp, out tmp2 );
			_y.Mult( btScalar.btSin( angle ), out tmp );
			tmp2.Add( ref tmp, out result );
		}
		/*@brief Return the cross product between this and another vector 
          @param v The other vector */
		public void cross( ref btVector3 v, out btVector3 result )
		{
			result.x = y * v.z - z * v.y;
			result.y = z * v.x - x * v.z;
			result.z = x * v.y - y * v.x;
			result.w = 0;
		}

		/*@brief Return the angle between this and another vector
          @param v The other vector */
		public double angle( ref btVector3 v )
		{
			double s = btScalar.btSqrt( ( length2() * v.length2() ) );
#if PARANOID_ASSERTS
			Debug.Assert( s != (double)(0.0));
#endif
			return btScalar.btAcos( ( dot( ref v ) / s ) );
		}


		public double triple( ref btVector3 v1, ref btVector3 v2 )
		{
			return
				x * ( v1.y * v2.z - v1.z * v2.y ) +
				y * ( v1.z * v2.x - v1.x * v2.z ) +
				z * ( v1.x * v2.y - v1.y * v2.x );
		}

		/*@brief Return the axis with the smallest value 
          Note return values are 0,1,2 for x, y, or z */
		public int minAxis()
		{
			return x < y ? ( x < z ? 0 : 2 ) : ( y < z ? 1 : 2 );
		}

		/*@brief Return the axis with the largest value 
          Note return values are 0,1,2 for x, y, or z */
		public int maxAxis()
		{
			return x < y ? ( y < z ? 2 : 1 ) : ( x < z ? 2 : 0 );
		}

		public int furthestAxis()
		{
			return absolute().minAxis();
		}

		public int closestAxis()
		{
			return absolute().maxAxis();
		}

		public static void setInterpolate3( out btVector3 result, ref btVector3 v0, ref btVector3 v1, double rt )
		{
			double s = 1.0 - rt;
			result.x = s * v0.x + rt * v1.x;
			result.y = s * v0.y + rt * v1.y;
			result.z = s * v0.z + rt * v1.z;
			result.w = 0;
			//don't do the unused w component
			//		m_co[3] = s  v0[3] + rt  v1[3];
		}

		public static void setInterpolate3( out btVector3 result, btIVector3 v0, btIVector3 v1, double rt )
		{
			double s = 1.0 - rt;
			result.x = s * v0.X + rt * v1.X;
			result.y = s * v0.Y + rt * v1.Y;
			result.z = s * v0.Z + rt * v1.Z;
			result.w = 0;
			//don't do the unused w component
			//		m_co[3] = s  v0[3] + rt  v1[3];
		}

		public void setInterpolate3( ref btVector3 v0, ref btVector3 v1, double rt )
		{
			double s = 1.0 - rt;
			x = s * v0.x + rt * v1.x;
			y = s * v0.y + rt * v1.y;
			z = s * v0.z + rt * v1.z;
			w = 0;
			//don't do the unused w component
			//		m_co[3] = s  v0[3] + rt  v1[3];
		}

		public void setInterpolate3( btIVector3 v0, btIVector3 v1, double rt )
		{
			double s = 1.0 - rt;
			x = s * v0.X + rt * v1.X;
			y = s * v0.Y + rt * v1.Y;
			z = s * v0.Z + rt * v1.Z;
			w = 0;
			//don't do the unused w component
			//		m_co[3] = s  v0[3] + rt  v1[3];
		}

		/*@brief Return the linear interpolation between this and another vector 
          @param v The other vector 
          @param t The ration of this to v (t = 0 => return this, t=1 => return other) */
		public void lerp( ref btVector3 v, double t, out btVector3 result )
		{
			result.x = x + ( v.x - x ) * t;
			result.y = y + ( v.y - y ) * t;
			result.z = z + ( v.z - z ) * t;
			result.w = 0;
		}

		public bool Equals( ref btVector3 other )
		{
			return ( ( w == other.w ) &&
					( z == other.z ) &&
					( y == other.y ) &&
					( x == other.x ) );
		}


		/*@brief Set each element to the max of the current values and the values of another btVector3
		  @param other The other btVector3 to compare with 
		 */
		public void setMax( ref btVector3 other )
		{
			btScalar.btSetMax( ref x, other.x );
			btScalar.btSetMax( ref y, other.y );
			btScalar.btSetMax( ref z, other.z );
			btScalar.btSetMax( ref w, other.w );
		}
		public static void setMax( ref btVector3 a, ref btVector3 b, out btVector3 result )
		{
			result.x = Math.Max( a.x, b.x );
			result.y = Math.Max( a.y, b.y );
			result.z = Math.Max( a.z, b.z );
			result.w = Math.Max( a.w, b.w );
		}


		/*@brief Set each element to the min of the current values and the values of another btVector3
		  @param other The other btVector3 to compare with 
		 */
		public void setMin( ref btVector3 other )
		{
			btScalar.btSetMin( ref x, other.x );
			btScalar.btSetMin( ref y, other.y );
			btScalar.btSetMin( ref z, other.z );
			btScalar.btSetMin( ref w, other.w );
		}
		public static void setMin( ref btVector3 a, ref btVector3 b, out btVector3 result )
		{
			result.x = Math.Min( a.x, b.x );
			result.y = Math.Min( a.y, b.y );
			result.z = Math.Min( a.z, b.z );
			result.w = Math.Min( a.w, b.w );
		}


		public void getSkewSymmetricMatrix( out btVector3 v0, out btVector3 v1, out btVector3 v2 )
		{
			btVector3.setValue( out v0, 0, -z, y );
			btVector3.setValue( out v1, z, 0, -x );
			btVector3.setValue( out v2, -y, x, 0 );
		}

		public void setZero()
		{
			setValue( 0, 0, 0 );
		}

		public bool isZero()
		{
			return x == 0 && y == 0 && z == 0;
		}


		/* create a vector as  btVector3( this.dot( btVector3 v0 ), this.dot( btVector3 v1), this.dot( btVector3 v2 ))  */
		public void dot3( ref btVector3 v0, ref btVector3 v1, ref btVector3 v2, out btVector3 result )
		{
			result.x = dot( ref v0 );
			result.y = dot( ref v1 );
			result.z = dot( ref v2 );
			result.w = 0;
		}
		public void dot3( ref btMatrix3x3 m, out btVector3 result )
		{
			result.x = dot( ref m.m_el0 );
			result.y = dot( ref m.m_el1 );
			result.z = dot( ref m.m_el2 );
			result.w = 0;
		}


		/*@brief Return the elementwise product of two vectors */
		public static void Mult( ref btVector3 v1, ref btVector3 v2, out btVector3 result )
		{
			result.x = v1.x * v2.x;
			result.y = v1.y * v2.y;
			result.z = v1.z * v2.z;
			result.w = v1.w * v2.w;
		}
		/*@brief Return the elementwise product of two vectors */
		public void Mult( ref btVector3 v2, out btVector3 result )
		{
			result.x = x * v2.x;
			result.y = y * v2.y;
			result.z = z * v2.z;
			result.w = w * v2.w;
		}


		/*@brief Return the negative of the vector */
		public static void Invert( ref btVector3 v, out btVector3 result )
		{
			result.x = -v.x;
			result.y = -v.y;
			result.z = -v.z;
			result.w = -v.w;
		}
		public void Invert( out btVector3 result )
		{
			result.x = -x;
			result.y = -y;
			result.z = -z;
			result.w = -w;
		}

		/*@brief Return the vector scaled by s */
		public static void Mult( double s, ref btVector3 v, out btVector3 result )
		{
			v.Mult( s, out result );
		}

		/*@brief Return the vector inversely scaled by s */
		public static void Div( ref btVector3 v1, ref btVector3 v2, out btVector3 result )
		{
			result.x = v1.x / v2.x;
			result.y = v1.y / v2.y;
			result.z = v1.z / v2.z;
			result.w = v1.w / v2.w;
		}

		/*@brief Return the vector inversely scaled by s */
		public void Div( ref btVector3 v2, out btVector3 result )
		{
			result.x = x / v2.x;
			result.y = y / v2.y;
			result.z = z / v2.z;
			result.w = w / v2.w;
		}

		/*@brief Return the dot product between two vectors */
		public static double btDot( ref btVector3 v1, ref btVector3 v2 )
		{
			return v1.dot( ref v2 );
		}


		/*@brief Return the distance squared between two vectors */
		public static double btDistance2( ref btVector3 v1, ref btVector3 v2 )
		{
			return v1.distance2( ref v2 );
		}


		/*@brief Return the distance between two vectors */
		public static double btDistance( ref btVector3 v1, ref btVector3 v2 )
		{
			return v1.distance( ref v2 );
		}

		/*@brief Return the angle between two vectors */
		public double btAngle( ref btVector3 v1, ref btVector3 v2 )
		{
			return v1.angle( ref v2 );
		}

		/*@brief Return the cross product of two vectors */
		public static void btCross( ref btVector3 v1, ref btVector3 v2, out btVector3 result )
		{
			v1.cross( ref v2, out result );
		}

		public static void btCross2Del( ref btVector3 a, ref btVector3 b, ref btVector3 c, ref btVector3 d, out btVector3 result )
		{
			btVector3 tmp1;
			btVector3 tmp2;
			a.Sub( ref b, out tmp1 );
			c.Sub( ref d, out tmp2 );
			btCross( ref tmp1, ref tmp2, out result );
		}

		public static double btTriple( ref btVector3 v1, ref btVector3 v2, ref btVector3 v3 )
		{
			return v1.triple( ref v2, ref v3 );
		}

		/*@brief Return the linear interpolation between two vectors
		  @param v1 One vector 
		  @param v2 The other vector 
		  @param t The ration of this to v (t = 0 => return v1, t=1 => return v2) */
		public static void lerp( btVector3 v1, btVector3 v2, double t, out btVector3 result )
		{
			v1.lerp( ref v2, t, out result );
		}



		public long maxDot( btVector3[] array, long array_count, out double dotOut )
		{
			{
				double maxDot1 = -btScalar.SIMD_INFINITY;
				int i = 0;
				int ptIndex = -1;
				for( i = 0; i < array_count; i++ )
				{
					double dot = array[i].dot( ref this );

					if( dot > maxDot1 )
					{
						maxDot1 = dot;
						ptIndex = i;
					}
				}

				dotOut = maxDot1;
				return ptIndex;
			}
		}

		public long minDot( btVector3[] array, long array_count, ref double dotOut )
		{
			{
				double minDot = btScalar.SIMD_INFINITY;
				int i = 0;
				int ptIndex = -1;

				for( i = 0; i < array_count; i++ )
				{
					double dot = array[i].dot( ref this );

					if( dot < minDot )
					{
						minDot = dot;
						ptIndex = i;
					}
				}

				dotOut = minDot;

				return ptIndex;
			}
		}

		public static void btPlaneSpace1( ref btVector3 n, out btVector3 p, out btVector3 q )
		{
			if( btScalar.btFabs( n.z ) > btScalar.SIMDSQRT12 )
			{
				// choose p in y-z plane
				double a = n.y * n.y + n.z * n.z;
				double k = btScalar.btRecipSqrt( a );
				p.x = 0;
				p.y = -n[2] * k;
				p.z = n[1] * k;
				// set q = n x p
				q.x = a * k;
				q.y = -n.x * p.z;
				q.z = n.x * p.y;
			}
			else
			{
				// choose p in x-y plane
				double a = n.x * n.x + n.y * n.y;
				double k = btScalar.btRecipSqrt( a );
				p.x = -n.y * k;
				p.y = n.x * k;
				p.z = 0;
				// set q = n x p
				q.x = -n.z * p.x;
				q.y = n.z * p.x;
				q.z = a * k;
			}
			q.w = 0;
			p.w = 0;
		}

		public static btVector3 btAabbSupport( ref btVector3 halfExtents, ref btVector3 supportDir )
		{
			return new btVector3( ( supportDir.x < 0.0 ) ? (double)-halfExtents.x : (double)halfExtents.x,
			  ( supportDir.y < 0.0 ) ? (double)-halfExtents.y : (double)halfExtents.y,
			  ( supportDir.z < 0 ) ? (double)-halfExtents.z : (double)halfExtents.z );
		}

		public static void getHalfExtent( ref btVector3 min, ref btVector3 max, out btVector3 half )
		{
			half.x = ( max.x - min.x ) * btScalar.BT_HALF;
			half.y = ( max.y - min.y ) * btScalar.BT_HALF;
			half.z = ( max.z - min.z ) * btScalar.BT_HALF;
			half.w = ( max.w - min.w ) * btScalar.BT_HALF; ;
		}
		public static void getCenter( ref btVector3 min, ref btVector3 max, out btVector3 half )
		{
			half.x = ( min.x + max.x ) * btScalar.BT_HALF;
			half.y = ( min.y + max.y ) * btScalar.BT_HALF;
			half.z = ( min.z + max.z ) * btScalar.BT_HALF;
			half.w = ( min.w + max.w ) * btScalar.BT_HALF; ;
		}

		public static double btDelLength( ref btVector3 linVelB, ref btVector3 linVelA )
		{
			btVector3 tmp;
			linVelB.Sub( linVelA, out tmp );
			return tmp.length();
		}

#if !DISABLE_OPERATORS
		public static btVector3 operator +( btVector3 a, btVector3 b )
		{
			return new btVector3( a.x + b.x, a.y + b.y, a.z + b.z );
		}
		public static btVector3 operator -( btVector3 a, btVector3 b )
		{
			return new btVector3( a.x - b.x, a.y - b.y, a.z - b.z );
		}
		public static btVector3 operator -( btVector3 a, btIVector3 b )
		{
			return new btVector3( a.x - b[0], a.y - b[1], a.z - b[2] );
		}
		public static btVector3 operator -( btVector3 a )
		{
			return new btVector3( -a.x, -a.y, -a.z );
		}
		public static btVector3 operator *( btVector3 a, btVector3 b )
		{
			return new btVector3( a.x * b.x, a.y * b.y, a.z * b.z );
		}
		public static btVector3 operator *( btVector3 a, double b )
		{
			return new btVector3( a.x * b, a.y * b, a.z * b );
		}
		public static btVector3 operator /( btVector3 a, double b )
		{
			b = 1.0 / b;
			return new btVector3( a.x * b, a.y * b, a.z * b );
		}
		public static btVector3 operator *( btVector3 a, btIVector3 b )
		{
			return new btVector3( a.x * b[0], a.y * b[1], a.z * b[2] );
		}
		public static btVector3 operator *( btIMatrix3x3 m, btVector3 v )
		{
			return new btVector3( m[0].dot( ref v ),
					m[1].dot( ref v ),
					m[2].dot( ref v ) );
		}
		public static btVector3 operator *( btVector3 v, btIMatrix3x3 m )
		{
			return new btVector3( m.tdotx( ref v ),
					m.tdoty( ref v ),
					m.tdotz( ref v ) );
		}


		public btVector3 cross( btVector3 v )
		{
			return new btVector3( y * v.z - z * v.y,
				z * v.x - x * v.z,
				x * v.y - y * v.x );
		}
		public double dot( btVector3 v )
		{
			return x * v.x +
					y * v.y +
					z * v.z;
		}

#endif
		public override string ToString()
		{
			return String.Format( "({0:g6},{1:g6},{2:g6})", x, y, z );
		}


	}


	public struct btVector4
	{
		public double x;
		public double y;
		public double z;
		public double w;


		public btVector4( double _x, double _y, double _z, double _w )
		{
			x = _x; y = _y; z = _z;
			w = _w;
		}


		btVector4 absolute4()
		{
			return new btVector4(
				btScalar.btFabs( x ),
				btScalar.btFabs( y ),
				btScalar.btFabs( z ),
				btScalar.btFabs( w ) );
		}


		double getW() { return w; }


		int maxAxis4()
		{
			int maxIndex = -1;
			double maxVal = -btScalar.BT_LARGE_FLOAT;
			if( x > maxVal )
			{
				maxIndex = 0;
				maxVal = x;
			}
			if( y > maxVal )
			{
				maxIndex = 1;
				maxVal = y;
			}
			if( z > maxVal )
			{
				maxIndex = 2;
				maxVal = z;
			}
			if( w > maxVal )
			{
				maxIndex = 3;
				maxVal = w;
			}

			return maxIndex;
		}


		int minAxis4()
		{
			int minIndex = -1;
			double minVal = btScalar.BT_LARGE_FLOAT;
			if( x < minVal )
			{
				minIndex = 0;
				minVal = x;
			}
			if( y < minVal )
			{
				minIndex = 1;
				minVal = y;
			}
			if( z < minVal )
			{
				minIndex = 2;
				minVal = z;
			}
			if( w < minVal )
			{
				minIndex = 3;
				minVal = w;
			}

			return minIndex;
		}


		public int closestAxis4()
		{
			return absolute4().maxAxis4();
		}




		/*@brief Set x,y,z and zero w 
		  @param x Value of x
		  @param y Value of y
		  @param z Value of z
		 */


		/*		void getValue(double m) 
				{
					m[0] = x;
					m[1] = y;
					m[2] =z;
				}
		*/
		/*@brief Set the values 
		    @param x Value of x
		    @param y Value of y
		    @param z Value of z
		    @param w Value of w
		   */
		void setValue( double _x, double _y, double _z, double _w )
		{
			x = _x;
			y = _y;
			z = _z;
			w = _w;
		}


		/*
				///btSwapVector3Endian swaps vector endianness, useful for network and cross-platform serialization
				void btSwapScalarEndian( double sourceVal, double destVal)
				{
# ifdef BT_USE_DOUBLE_PRECISION
					byte dest = ( byte) destVal;
					byte src = ( byte) &sourceVal;
					dest = src[7];
					dest[1] = src[6];
					dest[2] = src[5];
					dest[3] = src[4];
					dest[4] = src[3];
					dest[5] = src[2];
					dest[6] = src[1];
					dest[7] = src[0];
#else
	byte dest = ( byte) destVal;
					byte src = ( byte) &sourceVal;
					dest = src[3];
					dest[1] = src[2];
					dest[2] = src[1];
					dest[3] = src[0];
		#endif //BT_USE_DOUBLE_PRECISION
				}
				///btSwapVector3Endian swaps vector endianness, useful for network and cross-platform serialization
				void btSwapVector3Endian( btVector3 sourceVec, ref btVector3 destVec)
				{
					for( int i = 0; i < 4; i++ )
					{
						btSwapScalarEndian( sourceVec[i], destVec[i] );
					}

				}

				///btUnSwapVector3Endian swaps vector endianness, useful for network and cross-platform serialization
				void btUnSwapVector3Endian( ref btVector3 vector)
				{

					btVector3 swappedVec;
					for( int i = 0; i < 4; i++ )
					{
						btSwapScalarEndian( vector[i], swappedVec[i] );
					}
					vector = swappedVec;
				}
		*/


		/*
				void btVector3::serializeFloat(struct btVector3FloatData& dataOut)
				{
					///could also do a memcpy, check if it is worth it
					for (int i = 0; i<4;i++)
						dataOut.m_floats[i] = float(m_floats[i]);
				}

			void btVector3::deSerializeFloat(stringstruct btVector3FloatData& dataIn)
			{
				for (int i = 0; i<4;i++)
					m_floats[i] = (double)( dataIn.m_floats[i]);
		}


		void btVector3::serializeDouble(struct btVector3DoubleData& dataOut)
			{
				///could also do a memcpy, check if it is worth it
				for (int i = 0; i<4;i++)
					dataOut.m_floats[i] = double(m_floats[i]);
			}

			 void btVector3::deSerializeDouble(stringstruct btVector3DoubleData& dataIn)
			{
				for (int i = 0; i<4;i++)
					m_floats[i] = (double)( dataIn.m_floats[i]);
			}


			 void btVector3::serialize(struct btVector3Data& dataOut)
			{
				///could also do a memcpy, check if it is worth it
				for (int i = 0; i<4;i++)
					dataOut.m_floats[i] = m_floats[i];
			}

			 void btVector3::deSerialize(stringstruct btVector3Data& dataIn)
			{
				for (int i = 0; i<4;i++)
					m_floats[i] = dataIn.m_floats[i];
			}
			*/
		public bool Equals( ref btVector3 q )
		{
			return ( q.x == x ) && ( q.y == y ) && ( q.z == z ) && ( q.w == w );
		}

	}

}
