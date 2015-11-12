//#define DEBUG_CONVEX_HULL
//#define SHOW_ITERATIONS



//http://code.google.com/p/bullet/issues/detail?id=275
/*
Copyright (c) 2011 Ole Kniemeyer, MAXON, www.maxon.net

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

using System;
using System.Collections.Generic;
using System.Diagnostics;
using Bullet.LinearMath;
using Bullet.Types;

namespace Bullet.LinearMath
{

	public abstract class PoolNode<T>
	{
		public T next;
		internal abstract void Free();
	}

	// Convex hull implementation based on Preparata and Hong
	// Ole Kniemeyer, MAXON Computer GmbH
	public class btConvexHullInternal
	{

		public struct Point64
		{
			public long x;
			public long y;
			public long z;

			public Point64( long x, long y, long z )
			{
				this.x = x; this.y = y; this.z = z;
			}

			public bool isZero()
			{
				return ( x == 0 ) && ( y == 0 ) && ( z == 0 );
			}

			public long dot( ref Point64 b )
			{
				return x * b.x + y * b.y + z * b.z;
			}
		};

		public struct Point32
		{
			public int x;
			public int y;
			public int z;
			public int index;


			public Point32( int x, int y, int z )
			{
				this.x = x; this.y = y; this.z = z;
				this.index = -1;
			}

			bool Equals( ref Point32 b )
			{
				return ( x == b.x ) && ( y == b.y ) && ( z == b.z );
			}

			public bool isZero()
			{
				return ( x == 0 ) && ( y == 0 ) && ( z == 0 );
			}

			public Point64 cross( ref Point32 b )
			{
				return new Point64( y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x );
			}

			public Point64 cross( ref Point64 b )
			{
				return new Point64( y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x );
			}

			public long dot( ref Point32 b )
			{
				return x * b.x + y * b.y + z * b.z;
			}

			public long dot( ref Point64 b )
			{
				return x * b.x + y * b.y + z * b.z;
			}

			public Point32 Add( ref Point32 b )
			{
				return new Point32( x + b.x, y + b.y, z + b.z );
			}

			public Point32 Sub( ref Point32 b )
			{
				return new Point32( x - b.x, y - b.y, z - b.z );
			}
			public static Point32 operator +( Point32 a, Point32 b )
			{
				return new Point32( a.x + b.x, a.y + b.y, a.z + b.z );
			}

			public static Point32 operator -( Point32 a, Point32 b )
			{
				return new Point32( a.x - b.x, a.y - b.y, a.z - b.z );
			}
		};

		public struct Int128
		{
			public ulong low;
			public ulong high;

			public Int128( ulong low, ulong high )
			{
				this.low = low;
				this.high = high;
			}

			public Int128( ulong low )
			{
				this.low = low;
				this.high = 0;
			}

			public Int128( long value )
			{
				this.low = (ulong)value;
				this.high = ( value >= 0 ) ? 0 : 0xFFFFFFFFFFFFFFFFUL;
			}

			public void Negative( out Int128 result )
			{
				result.low = (ulong)-(long)low;
				result.high = ~high + ( ( low == 0 ) ? (ulong)1 : (ulong)0 );
			}

			public Int128 Add( ref Int128 b )
			{
				ulong lo = low + b.low;
				return new Int128( lo, high + b.high + ( ( lo < low ) ? (ulong)1 : (ulong)0 ) );
			}

			public Int128 Sub( ref Int128 b )
			{
				Int128 tmp;
				b.Negative( out tmp );
				return this.Add( ref tmp );
			}

			public static Int128 operator +( Int128 a, Int128 b )
			{
				ulong lo = a.low + b.low;
				return new Int128( lo, a.high + b.high + ( ( lo < a.low ) ? (ulong)1 : (ulong)0 ) );
			}

			public static Int128 operator +( Int128 a, ulong b )
			{
				ulong lo = a.low + b;
				return new Int128( lo, a.high + 0 + ( ( lo < a.low ) ? (ulong)1 : (ulong)0 ) );
			}
			public static Int128 operator +( Int128 a, long b )
			{
				ulong lo = a.low + (ulong)b;
				return new Int128( lo, a.high + 0 + ( ( lo < a.low ) ? (ulong)1 : (ulong)0 ) );
			}

			public static Int128 operator -( Int128 a, Int128 b )
			{
				Int128 tmp;
				b.Negative( out tmp );
				return a.Add( ref tmp );
			}


			public void Inc()
			{
				if( ++low == 0 )
				{
					++high;
				}
			}

			//public Int128 operator *( long b );

			public double toScalar()
			{
				if( ( high & 0x8000000000000000UL ) == 0 )
					return (double)( high ) * ( (double)( 0x100000000L ) * (double)( 0x100000000L ) ) + (double)( low );
				else
				{
					Int128 tmp;
					Negative( out tmp );
					return -( tmp ).toScalar();
				}
			}

			public int getSign()
			{
				return ( (long)high < 0 ) ? -1 : ( ( high != 0 ) || ( low != 0 ) ) ? 1 : 0;
			}

			public bool LessThan( ref Int128 b )
			{
				return ( high < b.high ) || ( ( high == b.high ) && ( low < b.low ) );
			}
			public static bool operator <( Int128 a, Int128 b )
			{
				return ( a.high < b.high ) || ( ( a.high == b.high ) && ( a.low < b.low ) );
			}
			public static bool operator >( Int128 a, Int128 b )
			{
				return ( a.high > b.high ) || ( ( a.high == b.high ) && ( a.low > b.low ) );
			}

			public int ucmp( ref Int128 b )
			{
				if( high < b.high )
				{
					return -1;
				}
				if( high > b.high )
				{
					return 1;
				}
				if( low < b.low )
				{
					return -1;
				}
				if( low > b.low )
				{
					return 1;
				}
				return 0;
			}
			public Int128 Mul( long b )
			{
				bool negative = (long)high < 0;
				Int128 a;
				if( negative )
					this.Negative( out a );
				else
					a = this;
				if( b < 0 )
				{
					negative = !negative;
					b = -b;
				}
				Int128 result = mul( a.low, (ulong)b );
				result.high += a.high * (ulong)b;
				if( negative )
				{
					Int128 tmp;
					result.Negative( out tmp );
					return tmp;
				}
				else
					return result;
			}

			public static Int128 operator *( Int128 a, long b )
			{
				return a.Mul( b );
			}
			public static Int128 mul( long a, long b )
			{
				Int128 result;

				bool negative = a < 0;
				if( negative )
				{
					a = -a;
				}
				if( b < 0 )
				{
					negative = !negative;
					b = -b;
				}
				DMul.mul_ulong( (ulong)a, (ulong)b, out result.low, out result.high );
				if( negative )
				{
					Int128 tmp;
					result.Negative( out tmp );
					return tmp;
				}
				return result;
			}

			public static Int128 mul( ulong a, ulong b )
			{
				Int128 result;

				DMul.mul_ulong( a, b, out result.low, out result.high );

				return result;
			}
		};


		public struct Rational64
		{
			ulong m_numerator;
			ulong m_denominator;
			int sign;

			public
				Rational64( long numerator, long denominator )
			{
				if( numerator > 0 )
				{
					sign = 1;
					m_numerator = (ulong)numerator;
				}
				else if( numerator < 0 )
				{
					sign = -1;
					m_numerator = (ulong)-numerator;
				}
				else
				{
					sign = 0;
					m_numerator = 0;
				}
				if( denominator > 0 )
				{
					m_denominator = (ulong)denominator;
				}
				else if( denominator < 0 )
				{
					sign = -sign;
					m_denominator = (ulong)-denominator;
				}
				else
				{
					m_denominator = 0;
				}
			}

			public bool isNegativeInfinity()
			{
				return ( sign < 0 ) && ( m_denominator == 0 );
			}

			public bool isNaN()
			{
				return ( sign == 0 ) && ( m_denominator == 0 );
			}

			public int compare( ref Rational64 b )
			{
				if( sign != b.sign )
				{
					return sign - b.sign;
				}
				else if( sign == 0 )
				{
					return 0;
				}

				//	return (numerator * b.denominator > b.numerator * denominator) ? sign : (numerator * b.denominator < b.numerator * denominator) ? -sign : 0;
				Int128 tmp = Int128.mul( m_denominator, b.m_numerator );
				return sign * Int128.mul( m_numerator, b.m_denominator ).ucmp( ref tmp );
			}


			public double toScalar()
			{
				return sign * ( ( m_denominator == 0 ) ? btScalar.SIMD_INFINITY : (double)m_numerator / m_denominator );
			}
		};


		public class Rational128
		{
			Int128 numerator;
			Int128 denominator;
			int sign;
			bool isInt64;

			public
				Rational128( long value )
			{
				if( value > 0 )
				{
					sign = 1;
					this.numerator = new Int128( value );
				}
				else if( value < 0 )
				{
					sign = -1;
					this.numerator = new Int128( -value );
				}
				else
				{
					sign = 0;
					this.numerator = new Int128( (ulong)0 );
				}
				this.denominator = new Int128( (ulong)1 );
				isInt64 = true;
			}

			public Rational128( Int128 numerator, Int128 denominator )
			{
				sign = numerator.getSign();
				if( sign >= 0 )
				{
					this.numerator = numerator;
				}
				else
				{
					//tmp
					numerator.Negative( out this.numerator );
					//this.numerator = -numerator;
				}
				int dsign = denominator.getSign();
				if( dsign >= 0 )
				{
					this.denominator = denominator;
				}
				else
				{
					sign = -sign;
					denominator.Negative( out this.denominator );
					//this.denominator = -denominator;
				}
				isInt64 = false;
			}

			public int compare( ref Rational128 b )
			{
				if( sign != b.sign )
				{
					return sign - b.sign;
				}
				else if( sign == 0 )
				{
					return 0;
				}
				if( isInt64 )
				{
					return -b.compare( sign * (long)numerator.low );
				}

				Int128 nbdLow, nbdHigh, dbnLow, dbnHigh;
				DMul.mul_128( numerator, b.denominator, out nbdLow, out nbdHigh );
				DMul.mul_128( denominator, b.numerator, out dbnLow, out dbnHigh );

				int cmp = nbdHigh.ucmp( ref dbnHigh );
				if( cmp != 0 )
				{
					return cmp * sign;
				}
				return nbdLow.ucmp( ref dbnLow ) * sign;
			}

			public int compare( long b )
			{
				if( isInt64 )
				{
					long a = sign * (long)numerator.low;
					return ( a > b ) ? 1 : ( a < b ) ? -1 : 0;
				}
				if( b > 0 )
				{
					if( sign <= 0 )
					{
						return -1;
					}
				}
				else if( b < 0 )
				{
					if( sign >= 0 )
					{
						return 1;
					}
					b = -b;
				}
				else
				{
					return sign;
				}

				Int128 tmp = denominator.Mul( b );
				return numerator.ucmp( ref tmp ) * sign;
			}

			public double toScalar()
			{
				return sign * ( ( denominator.getSign() == 0 ) ? btScalar.SIMD_INFINITY : numerator.toScalar() / denominator.toScalar() );
			}
		};

		public class PointR128
		{
			public Int128 x;
			public Int128 y;
			public Int128 z;
			public Int128 denominator;

			public PointR128( Int128 x, Int128 y, Int128 z, Int128 denominator )
			{
				this.x = x; this.y = ( y ); this.z = ( z ); this.denominator = ( denominator );

			}

			public double xvalue()
			{
				return x.toScalar() / denominator.toScalar();
			}

			public double yvalue()
			{
				return y.toScalar() / denominator.toScalar();
			}

			public double zvalue()
			{
				return z.toScalar() / denominator.toScalar();
			}
		};


		public class Vertex : PoolNode<Vertex>
		{
			//public Vertex next;
			public Vertex prev;
			public Edge edges;
			public Face firstNearbyFace;
			public Face lastNearbyFace;
			public PointR128 point128;
			public Point32 point;
			internal btConvexHullComputer.Edge copy;

			public Vertex()
			{
				//copy = null;
			}

			internal override void Free()
			{
			}

#if DEBUG_CONVEX_HULL
				void print()
				{
					Console.WriteLine("V{0} ({1}, {2}, {3})", point.index, point.x, point.y, point.z);
				}

				void printGraph()
				{
					print();
					Console.WriteLine("\nEdges\n");
					Edge* e = edges;
					if (e)
					{
						do
						{
							e.print();
							Console.WriteLine("\n");
							e = e.next;
						} while (e != edges);
						do
						{
							Vertex* v = e.target;
							if (v.copy != copy)
							{
								v.copy = copy;
								v.printGraph();
							}
							e = e.next;
						} while (e != edges);
					}
				}
#endif

			public static Point32 operator -( Vertex a, Vertex b )
			{
				return a.point - b.point;
			}

			public Rational128 dot( ref Point64 b )
			{
				return ( point.index >= 0 ) ? new Rational128( point.dot( ref b ) )
					: new Rational128( point128.x * b.x + point128.y * b.y + point128.z * b.z, point128.denominator );
			}

			public double xvalue()
			{
				return ( point.index >= 0 ) ? (double)( point.x ) : point128.xvalue();
			}

			public double yvalue()
			{
				return ( point.index >= 0 ) ? (double)( point.y ) : point128.yvalue();
			}

			public double zvalue()
			{
				return ( point.index >= 0 ) ? (double)( point.z ) : point128.zvalue();
			}

			public void receiveNearbyFaces( Vertex src )
			{
				if( lastNearbyFace != null )
				{
					lastNearbyFace.nextWithSameNearbyVertex = src.firstNearbyFace;
				}
				else
				{
					firstNearbyFace = src.firstNearbyFace;
				}
				if( src.lastNearbyFace != null )
				{
					lastNearbyFace = src.lastNearbyFace;
				}
				for( Face f = src.firstNearbyFace; f != null; f = f.nextWithSameNearbyVertex )
				{
					Debug.Assert( f.nearbyVertex == src );
					f.nearbyVertex = this;
				}
				src.firstNearbyFace = null;
				src.lastNearbyFace = null;
			}
		};


		public class Edge : PoolNode<Edge>
		{
			//public Edge next;
			public Edge prev;
			public Edge reverse;
			public Vertex target;
			public Face face;
			internal btConvexHullComputer.Edge copy;

			internal override void Free()
			{
				next = null;
				prev = null;
				reverse = null;
				target = null;
				face = null;
			}

			public void link( Edge n )
			{
				Debug.Assert( reverse.target == n.reverse.target );
				next = n;
				n.prev = this;
			}

#if DEBUG_CONVEX_HULL
				void print()
				{
					Console.WriteLine("E{0} : {1} . {2},  n={3} p={4}   (0 {0}\t{6}\t{7}) . ({8} {9} {10})", this, reverse.target.point.index, target.point.index, next, prev,
								 reverse.target.point.x, reverse.target.point.y, reverse.target.point.z, target.point.x, target.point.y, target.point.z);
				}
#endif
		};

		public class Face : PoolNode<Face>
		{
			//public Face next;
			public Vertex nearbyVertex;
			public Face nextWithSameNearbyVertex;
			public Point32 origin;
			public Point32 dir0;
			public Point32 dir1;

			public Face()
			{
			}

			internal override void Free()
			{

			}

			public void init( Vertex a, Vertex b, Vertex c )
			{
				nearbyVertex = a;
				origin = a.point;
				dir0 = b - a;
				dir1 = c - a;
				if( a.lastNearbyFace != null )
				{
					a.lastNearbyFace.nextWithSameNearbyVertex = this;
				}
				else
				{
					a.firstNearbyFace = this;
				}
				a.lastNearbyFace = this;
			}

			public Point64 getNormal()
			{
				return dir0.cross( ref dir1 );
			}
		};

		public class DMul
		{
			static uint high( ulong value )
			{
				return (uint)( value >> 32 );
			}

			static uint low( ulong value )
			{
				return (uint)value;
			}

			static ulong mul( uint a, uint b )
			{
				return (ulong)a * (ulong)b;
			}

			static void shlHalf( ref ulong value )
			{
				value <<= 32;
			}

			static ulong high( Int128 value )
			{
				return value.high;
			}

			static ulong low( Int128 value )
			{
				return value.low;
			}

			static Int128 mul( ulong a, ulong b )
			{
				return Int128.mul( a, b );
			}

			static void shlHalf( ref Int128 value )
			{
				value.high = value.low;
				value.low = 0;
			}

			public static void mul_ulong( ulong a, ulong b, out ulong resLow, out ulong resHigh )
			{
				ulong p00 = mul( low( a ), low( b ) );
				ulong p01 = mul( low( a ), high( b ) );
				ulong p10 = mul( high( a ), low( b ) );
				ulong p11 = mul( high( a ), high( b ) );
				ulong p0110 = (ulong)( low( p01 ) ) + (ulong)( low( p10 ) );
				p11 += high( p01 );
				p11 += high( p10 );
				p11 += high( p0110 );
				shlHalf( ref p0110 );
				p00 += p0110;
				if( p00 < p0110 )
				{
					++p11;
				}
				resLow = (uint)p00;
				resHigh = (uint)p11;
			}
			public static void mul_128( Int128 a, Int128 b, out Int128 resLow, out Int128 resHigh )
			{
				Int128 p00 = mul( low( a ), low( b ) );
				Int128 p01 = mul( low( a ), high( b ) );
				Int128 p10 = mul( high( a ), low( b ) );
				Int128 p11 = mul( high( a ), high( b ) );
				Int128 p0110 = new Int128( low( p01 ) ) + new Int128( low( p10 ) );
				p11 += high( p01 );
				p11 += high( p10 );
				p11 += high( p0110 );
				shlHalf( ref p0110 );
				p00 += p0110;
				if( p00 < p0110 )
				{
					p11 += 1;
					//++p11;
				}
				resLow = p00;
				resHigh = p11;
			}
		};


		public struct IntermediateHull
		{
			public Vertex minXy;
			public Vertex maxXy;
			public Vertex minYx;
			public Vertex maxYx;

#if DEBUG_CONVEX_HULL
void print()
{
	Console.WriteLine("    Hull\n");
	for (Vertex* v = minXy; v; )
	{
		Console.WriteLine("      ");
		v.print();
		if (v == maxXy)
		{
			Console.WriteLine(" maxXy");
		}
		if (v == minYx)
		{
			Console.WriteLine(" minYx");
		}
		if (v == maxYx)
		{
			Console.WriteLine(" maxYx");
		}
		if (v.next.prev != v)
		{
			Console.WriteLine(" Inconsistency");
		}
		Console.WriteLine("\n");
		v = v.next;
		if (v == minXy)
		{
			break;
		}
	}
	if (minXy)
	{		
		minXy.copy = (minXy.copy == -1) ? -2 : -1;
		minXy.printGraph();
	}
}

#endif

		};

		public enum Orientation { NONE, CLOCKWISE, COUNTER_CLOCKWISE };

		public class PoolArray<T> where T : PoolNode<T>, new()
		{
			T[] array;
			int size;

			public
				PoolArray<T> next;

			public PoolArray( int size )
			{
				this.size = ( size ); this.next = null;
				array = new T[size];
			}

			~PoolArray()
			{
				array = null;
				//btAlignedFree( array );
			}

			public T init()
			{
				array[0] = new T();
				for( int i = 0; i < size; i++ )
				{
					if( i + 1 < size )
					{
						array[i].next = array[i + 1] = new T();
					}
				}
				return array[0];
			}
		};

		public class Pool<T> where T : PoolNode<T>, new()
		{
			PoolArray<T> arrays;
			PoolArray<T> nextArray;
			T freeObjects;
			int arraySize;

			public
				Pool()
			{
				arraySize = 256;
			}

			~Pool()
			{
				while( arrays != null )
				{
					PoolArray<T> p = arrays;
					arrays = p.next;
					p.next = null;
				}
			}

			public void reset()
			{
				nextArray = arrays;
				freeObjects = null;
			}

			public void setArraySize( int arraySize )
			{
				this.arraySize = arraySize;
			}

			public T newObject()
			{
				T o = freeObjects;
				if( o == null )
				{
					PoolArray<T> p = nextArray;
					if( p != null )
					{
						nextArray = p.next;
					}
					else
					{
						p = new PoolArray<T>( arraySize );// ( btAlignedAlloc( sizeof( PoolArray<T> ), 16 ) ) PoolArray<T>( arraySize );
						p.next = arrays;
						arrays = p;
					}
					o = p.init();
				}
				freeObjects = o.next;
				return o;
			}

			public void freeObject( T o )
			{
				o.Free();
				o.next = freeObjects;
				freeObjects = o;
			}
		};

		btVector3 scaling;
		btVector3 center;
		Pool<Vertex> vertexPool = new Pool<Vertex>();
		Pool<Edge> edgePool = new Pool<Edge>();
		Pool<Face> facePool = new Pool<Face>();
		btList<Vertex> originalVertices = new btList<Vertex>();
		btConvexHullComputer.Edge mergeStamp;
		int minAxis;
		int medAxis;
		int maxAxis;
		int usedEdgePairs;
		int maxUsedEdgePairs;
		public Vertex vertexList;

		//Edge* findMaxAngle( bool ccw, Vertex start, ref Point32 s, ref Point64 rxs, ref Point64 sxrxs, ref Rational64 minCot);
		//void findEdgeForCoplanarFaces( Vertex c0, Vertex c1, Edge e0, Edge e1, Vertex stop0, Vertex stop1 );

		//Edge* newEdgePair( Vertex* from, Vertex* to );

		void removeEdgePair( Edge edge )
		{
			Edge n = edge.next;
			Edge r = edge.reverse;

			Debug.Assert( edge.target != null && r.target != null );

			if( n != edge )
			{
				n.prev = edge.prev;
				edge.prev.next = n;
				r.target.edges = n;
			}
			else
			{
				r.target.edges = null;
			}

			n = r.next;

			if( n != r )
			{
				n.prev = r.prev;
				r.prev.next = n;
				edge.target.edges = n;
			}
			else
			{
				edge.target.edges = null;
			}

			edgePool.freeObject( edge );
			edgePool.freeObject( r );
			usedEdgePairs--;
		}





		Edge newEdgePair( Vertex from, Vertex to )
		{
			Debug.Assert( from != null && to != null );
			Edge e = edgePool.newObject();
			Edge r = edgePool.newObject();
			e.reverse = r;
			r.reverse = e;
			e.copy = mergeStamp;
			r.copy = mergeStamp;
			e.target = to;
			r.target = from;
			e.face = null;
			r.face = null;
			usedEdgePairs++;
			if( usedEdgePairs > maxUsedEdgePairs )
			{
				maxUsedEdgePairs = usedEdgePairs;
			}
			return e;
		}

		bool mergeProjection( ref IntermediateHull h0, ref IntermediateHull h1, out Vertex c0, out Vertex c1 )
		{
			Vertex v0 = h0.maxYx;
			Vertex v1 = h1.minYx;
			if( ( v0.point.x == v1.point.x ) && ( v0.point.y == v1.point.y ) )
			{
				Debug.Assert( v0.point.z < v1.point.z );
				Vertex v1p = v1.prev;
				if( v1p == v1 )
				{
					c0 = v0;
					if( v1.edges != null )
					{
						Debug.Assert( v1.edges.next == v1.edges );
						v1 = v1.edges.target;
						Debug.Assert( v1.edges.next == v1.edges );
					}
					c1 = v1;
					return false;
				}
				Vertex v1n = v1.next;
				v1p.next = v1n;
				v1n.prev = v1p;
				if( v1 == h1.minXy )
				{
					if( ( v1n.point.x < v1p.point.x ) || ( ( v1n.point.x == v1p.point.x ) && ( v1n.point.y < v1p.point.y ) ) )
					{
						h1.minXy = v1n;
					}
					else
					{
						h1.minXy = v1p;
					}
				}
				if( v1 == h1.maxXy )
				{
					if( ( v1n.point.x > v1p.point.x ) || ( ( v1n.point.x == v1p.point.x ) && ( v1n.point.y > v1p.point.y ) ) )
					{
						h1.maxXy = v1n;
					}
					else
					{
						h1.maxXy = v1p;
					}
				}
			}

			v0 = h0.maxXy;
			v1 = h1.maxXy;
			Vertex v00 = null;
			Vertex v10 = null;
			int sign = 1;

			for( int side = 0; side <= 1; side++ )
			{
				int dx = ( v1.point.x - v0.point.x ) * sign;
				if( dx > 0 )
				{
					while( true )
					{
						int dy = v1.point.y - v0.point.y;

						Vertex w0 = ( side != 0 ) ? v0.next : v0.prev;
						if( w0 != v0 )
						{
							int dx0 = ( w0.point.x - v0.point.x ) * sign;
							int dy0 = w0.point.y - v0.point.y;
							if( ( dy0 <= 0 ) && ( ( dx0 == 0 ) || ( ( dx0 < 0 ) && ( dy0 * dx <= dy * dx0 ) ) ) )
							{
								v0 = w0;
								dx = ( v1.point.x - v0.point.x ) * sign;
								continue;
							}
						}

						Vertex w1 = ( side != 0 ) ? v1.next : v1.prev;
						if( w1 != v1 )
						{
							int dx1 = ( w1.point.x - v1.point.x ) * sign;
							int dy1 = w1.point.y - v1.point.y;
							int dxn = ( w1.point.x - v0.point.x ) * sign;
							if( ( dxn > 0 ) && ( dy1 < 0 ) && ( ( dx1 == 0 ) || ( ( dx1 < 0 ) && ( dy1 * dx < dy * dx1 ) ) ) )
							{
								v1 = w1;
								dx = dxn;
								continue;
							}
						}

						break;
					}
				}
				else if( dx < 0 )
				{
					while( true )
					{
						int dy = v1.point.y - v0.point.y;

						Vertex w1 = ( side != 0 ) ? v1.prev : v1.next;
						if( w1 != v1 )
						{
							int dx1 = ( w1.point.x - v1.point.x ) * sign;
							int dy1 = w1.point.y - v1.point.y;
							if( ( dy1 >= 0 ) && ( ( dx1 == 0 ) || ( ( dx1 < 0 ) && ( dy1 * dx <= dy * dx1 ) ) ) )
							{
								v1 = w1;
								dx = ( v1.point.x - v0.point.x ) * sign;
								continue;
							}
						}

						Vertex w0 = ( side != 0 ) ? v0.prev : v0.next;
						if( w0 != v0 )
						{
							int dx0 = ( w0.point.x - v0.point.x ) * sign;
							int dy0 = w0.point.y - v0.point.y;
							int dxn = ( v1.point.x - w0.point.x ) * sign;
							if( ( dxn < 0 ) && ( dy0 > 0 ) && ( ( dx0 == 0 ) || ( ( dx0 < 0 ) && ( dy0 * dx < dy * dx0 ) ) ) )
							{
								v0 = w0;
								dx = dxn;
								continue;
							}
						}

						break;
					}
				}
				else
				{
					int x = v0.point.x;
					int y0 = v0.point.y;
					Vertex w0 = v0;
					Vertex t;
					while( ( ( t = ( side != 0 ) ? w0.next : w0.prev ) != v0 ) && ( t.point.x == x ) && ( t.point.y <= y0 ) )
					{
						w0 = t;
						y0 = t.point.y;
					}
					v0 = w0;

					int y1 = v1.point.y;
					Vertex w1 = v1;
					while( ( ( t = ( side != 0 ) ? w1.prev : w1.next ) != v1 ) && ( t.point.x == x ) && ( t.point.y >= y1 ) )
					{
						w1 = t;
						y1 = t.point.y;
					}
					v1 = w1;
				}

				if( side == 0 )
				{
					v00 = v0;
					v10 = v1;

					v0 = h0.minXy;
					v1 = h1.minXy;
					sign = -1;
				}
			}

			v0.prev = v1;
			v1.next = v0;

			v00.next = v10;
			v10.prev = v00;

			if( h1.minXy.point.x < h0.minXy.point.x )
			{
				h0.minXy = h1.minXy;
			}
			if( h1.maxXy.point.x >= h0.maxXy.point.x )
			{
				h0.maxXy = h1.maxXy;
			}

			h0.maxYx = h1.maxYx;

			c0 = v00;
			c1 = v10;

			return true;
		}

		void computeInternal( int start, int end, out IntermediateHull result )
		{
			int n = end - start;
			switch( n )
			{
				case 0:
					result.minXy = null;
					result.maxXy = null;
					result.minYx = null;
					result.maxYx = null;
					return;
				case 2:
					{
						Vertex v = originalVertices[start];
						Vertex w = originalVertices[start + 1];
						if( v.point.Equals( w.point ) )
						{
							int dx = v.point.x - w.point.x;
							int dy = v.point.y - w.point.y;

							if( ( dx == 0 ) && ( dy == 0 ) )
							{
								if( v.point.z > w.point.z )
								{
									Vertex t = w;
									w = v;
									v = t;
								}
								Debug.Assert( v.point.z < w.point.z );
								v.next = v;
								v.prev = v;
								result.minXy = v;
								result.maxXy = v;
								result.minYx = v;
								result.maxYx = v;
							}
							else
							{
								v.next = w;
								v.prev = w;
								w.next = v;
								w.prev = v;

								if( ( dx < 0 ) || ( ( dx == 0 ) && ( dy < 0 ) ) )
								{
									result.minXy = v;
									result.maxXy = w;
								}
								else
								{
									result.minXy = w;
									result.maxXy = v;
								}

								if( ( dy < 0 ) || ( ( dy == 0 ) && ( dx < 0 ) ) )
								{
									result.minYx = v;
									result.maxYx = w;
								}
								else
								{
									result.minYx = w;
									result.maxYx = v;
								}
							}

							Edge e = newEdgePair( v, w );
							e.link( e );
							v.edges = e;

							e = e.reverse;
							e.link( e );
							w.edges = e;

							return;
						}
					}
					goto case 1;
				// lint -fallthrough
				case 1:
					{
						Vertex v = originalVertices[start];
						v.edges = null;
						v.next = v;
						v.prev = v;

						result.minXy = v;
						result.maxXy = v;
						result.minYx = v;
						result.maxYx = v;

						return;
					}
			}

			int split0 = start + n / 2;
			Point32 p = originalVertices[split0 - 1].point;
			int split1 = split0;
			while( ( split1 < end ) && ( originalVertices[split1].point.Equals( p ) ) )
			{
				split1++;
			}
			computeInternal( start, split0, out result );
			IntermediateHull hull1;
			computeInternal( split1, end, out hull1 );
#if DEBUG_CONVEX_HULL
	Console.WriteLine("\n\nMerge\n");
	result.print();
	hull1.print();
#endif
			merge( ref result, ref hull1 );
#if DEBUG_CONVEX_HULL
	Console.WriteLine("\n  Result\n");
	result.print();
#endif
		}

		//static Orientation getOrientation( Edge prev, Edge next, out Point32 s, out Point32 t );
		btConvexHullInternal.Orientation getOrientation( Edge prev, Edge next, ref Point32 s, ref Point32 t )
		{
			Debug.Assert( prev.reverse.target == next.reverse.target );
			if( prev.next == next )
			{
				if( prev.prev == next )
				{
					Point64 n = t.cross( ref s );
					Point32 tmp = next.target - next.reverse.target;
					Point64 m = ( prev.target - next.reverse.target ).cross( ref tmp );
					Debug.Assert( !m.isZero() );
					long dot = n.dot( ref m );
					Debug.Assert( dot != 0 );
					return ( dot > 0 ) ? Orientation.COUNTER_CLOCKWISE : Orientation.CLOCKWISE;
				}
				return Orientation.COUNTER_CLOCKWISE;
			}
			else if( prev.prev == next )
			{
				return Orientation.CLOCKWISE;
			}
			else
			{
				return Orientation.NONE;
			}
		}

		Edge findMaxAngle( bool ccw, Vertex start, ref Point32 s, ref Point64 rxs, ref Point64 sxrxs, ref Rational64 minCot )
		{
			Edge minEdge = null;

#if DEBUG_CONVEX_HULL
	Console.WriteLine("find max edge for %d\n", start.point.index);
#endif
			Edge e = start.edges;
			if( e != null )
			{
				do
				{
					if( e.copy.id > mergeStamp.id )
					{
						Point32 t = e.target - start;
						Rational64 cot = new Rational64( t.dot( ref sxrxs ), t.dot( ref rxs ) );
#if DEBUG_CONVEX_HULL
				Console.WriteLine("      Angle is %f (%d) for ", (float) btAtan(cot.toScalar()), (int) cot.isNaN());
				e.print();
#endif
						if( cot.isNaN() )
						{
							Debug.Assert( ccw ? ( t.dot( ref s ) < 0 ) : ( t.dot( ref s ) > 0 ) );
						}
						else
						{
							int cmp;
							if( minEdge == null )
							{
								minCot = cot;
								minEdge = e;
							}
							else if( ( cmp = cot.compare( ref minCot ) ) < 0 )
							{
								minCot = cot;
								minEdge = e;
							}
							else if( ( cmp == 0 ) && ( ccw == ( getOrientation( minEdge, e, ref s, ref t ) == Orientation.COUNTER_CLOCKWISE ) ) )
							{
								minEdge = e;
							}
						}
#if DEBUG_CONVEX_HULL
				Console.WriteLine("\n");
#endif
					}
					e = e.next;
				} while( e != start.edges );
			}
			return minEdge;
		}

		void findEdgeForCoplanarFaces( Vertex c0, Vertex c1, Edge e0, Edge e1, Vertex stop0, Vertex stop1 )
		{
			Edge start0 = e0;
			Edge start1 = e1;
			Point32 et0 = start0 != null ? start0.target.point : c0.point;
			Point32 et1 = start1 != null ? start1.target.point : c1.point;
			Point32 s = c1.point - c0.point;
			Point64 normal = ( ( start0 != null ? start0 : start1 ).target.point - c0.point ).cross( ref s );
			long dist = c0.point.dot( ref normal );
			Debug.Assert( start1 == null || ( start1.target.point.dot( ref normal ) == dist ) );
			Point64 perp = s.cross( ref normal );
			Debug.Assert( !perp.isZero() );

#if DEBUG_CONVEX_HULL
	Console.WriteLine("   Advancing %d %d  (%p %p, %d %d)\n", c0.point.index, c1.point.index, start0, start1, start0 ? start0.target.point.index : -1, start1 ? start1.target.point.index : -1);
#endif

			long maxDot0 = et0.dot( ref perp );
			if( e0 != null )
			{
				while( e0.target != stop0 )
				{
					Edge e = e0.reverse.prev;
					if( e.target.point.dot( ref normal ) < dist )
					{
						break;
					}
					Debug.Assert( e.target.point.dot( ref normal ) == dist );
					if( e.copy == mergeStamp )
					{
						break;
					}
					long dot = e.target.point.dot( ref perp );
					if( dot <= maxDot0 )
					{
						break;
					}
					maxDot0 = dot;
					e0 = e;
					et0 = e.target.point;
				}
			}

			long maxDot1 = et1.dot( ref perp );
			if( e1 != null )
			{
				while( e1.target != stop1 )
				{
					Edge e = e1.reverse.next;
					if( e.target.point.dot( ref normal ) < dist )
					{
						break;
					}
					Debug.Assert( e.target.point.dot( ref normal ) == dist );
					if( e.copy == mergeStamp )
					{
						break;
					}
					long dot = e.target.point.dot( ref perp );
					if( dot <= maxDot1 )
					{
						break;
					}
					maxDot1 = dot;
					e1 = e;
					et1 = e.target.point;
				}
			}

#if DEBUG_CONVEX_HULL
	Console.WriteLine("   Starting at %d %d\n", et0.index, et1.index);
#endif

			long dx = maxDot1 - maxDot0;
			if( dx > 0 )
			{
				while( true )
				{
					long dy = ( et1 - et0 ).dot( ref s );

					if( e0 != null && ( e0.target != stop0 ) )
					{
						Edge f0 = e0.next.reverse;
						if( f0.copy.id > mergeStamp.id )
						{
							long dx0 = ( f0.target.point - et0 ).dot( ref perp );
							long dy0 = ( f0.target.point - et0 ).dot( ref s );
							Rational64 tmp = new Rational64( dy, dx );
							if( ( dx0 == 0 ) ? ( dy0 < 0 ) : ( ( dx0 < 0 ) && ( new Rational64( dy0, dx0 ).compare( ref tmp ) >= 0 ) ) )
							{
								et0 = f0.target.point;
								dx = ( et1 - et0 ).dot( ref perp );
								e0 = ( e0 == start0 ) ? null : f0;
								continue;
							}
						}
					}

					if( e1 != null && ( e1.target != stop1 ) )
					{
						Edge f1 = e1.reverse.next;
						if( f1.copy.id > mergeStamp.id )
						{
							Point32 d1 = f1.target.point - et1;
							if( d1.dot( ref normal ) == 0 )
							{
								long dx1 = d1.dot( ref perp );
								long dy1 = d1.dot( ref s );
								long dxn = ( f1.target.point - et0 ).dot( ref perp );
								Rational64 tmp = new Rational64( dy, dx );
								if( ( dxn > 0 ) && ( ( dx1 == 0 ) ? ( dy1 < 0 ) : ( ( dx1 < 0 ) && ( new Rational64( dy1, dx1 ).compare( ref tmp ) > 0 ) ) ) )
								{
									e1 = f1;
									et1 = e1.target.point;
									dx = dxn;
									continue;
								}
							}
							else
							{
								Debug.Assert( ( e1 == start1 ) && ( d1.dot( ref normal ) < 0 ) );
							}
						}
					}

					break;
				}
			}
			else if( dx < 0 )
			{
				while( true )
				{
					long dy = ( et1 - et0 ).dot( ref s );

					if( e1 != null && ( e1.target != stop1 ) )
					{
						Edge f1 = e1.prev.reverse;
						if( f1.copy.id > mergeStamp.id )
						{
							long dx1 = ( f1.target.point - et1 ).dot( ref perp );
							long dy1 = ( f1.target.point - et1 ).dot( ref s );
							Rational64 tmp = new Rational64( dy, dx );
							if( ( dx1 == 0 ) ? ( dy1 > 0 ) : ( ( dx1 < 0 ) && ( new Rational64( dy1, dx1 ).compare( ref tmp ) <= 0 ) ) )
							{
								et1 = f1.target.point;
								dx = ( et1 - et0 ).dot( ref perp );
								e1 = ( e1 == start1 ) ? null : f1;
								continue;
							}
						}
					}

					if( e0 != null && ( e0.target != stop0 ) )
					{
						Edge f0 = e0.reverse.prev;
						if( f0.copy.id > mergeStamp.id )
						{
							Point32 d0 = f0.target.point - et0;
							if( d0.dot( ref normal ) == 0 )
							{
								long dx0 = d0.dot( ref perp );
								long dy0 = d0.dot( ref s );
								long dxn = ( et1 - f0.target.point ).dot( ref perp );
								Rational64 tmp = new Rational64( dy, dx );
								if( ( dxn < 0 ) && ( ( dx0 == 0 ) ? ( dy0 > 0 ) : ( ( dx0 < 0 ) && ( new Rational64( dy0, dx0 ).compare( ref tmp ) < 0 ) ) ) )
								{
									e0 = f0;
									et0 = e0.target.point;
									dx = dxn;
									continue;
								}
							}
							else
							{
								Debug.Assert( ( e0 == start0 ) && ( d0.dot( ref normal ) < 0 ) );
							}
						}
					}

					break;
				}
			}
#if DEBUG_CONVEX_HULL
	Console.WriteLine("   Advanced edges to %d %d\n", et0.index, et1.index);
#endif
		}


		void merge( ref IntermediateHull h0, ref IntermediateHull h1 )
		{
			if( h1.maxXy == null )
			{
				return;
			}
			if( h0.maxXy == null )
			{
				h0 = h1;
				return;
			}

			Debugger.Break();
			// mergeStamp is uninitialized.
			//mergeStamp--;
			mergeStamp = mergeStamp.reverse;

			Vertex c0 = null;
			Edge toPrev0 = null;
			Edge firstNew0 = null;
			Edge pendingHead0 = null;
			Edge pendingTail0 = null;
			Vertex c1 = null;
			Edge toPrev1 = null;
			Edge firstNew1 = null;
			Edge pendingHead1 = null;
			Edge pendingTail1 = null;
			Point32 prevPoint;

			if( mergeProjection( ref h0, ref h1, out c0, out c1 ) )
			{
				Point32 s = c1 - c0;
				Point64 normal = new Point32( 0, 0, -1 ).cross( ref s );
				Point64 t = s.cross( ref normal );
				Debug.Assert( !t.isZero() );

				Edge e = c0.edges;
				Edge start0 = null;
				if( e != null )
				{
					do
					{
						long dot = ( e.target - c0 ).dot( ref normal );
						Debug.Assert( dot <= 0 );
						if( ( dot == 0 ) && ( ( e.target - c0 ).dot( ref t ) > 0 ) )
						{
							Point32 tmp = new Point32( 0, 0, -1 );
							if( start0 == null || ( getOrientation( start0, e, ref s, ref tmp ) == Orientation.CLOCKWISE ) )
							{
								start0 = e;
							}
						}
						e = e.next;
					} while( e != c0.edges );
				}

				e = c1.edges;
				Edge start1 = null;
				if( e != null )
				{
					do
					{
						long dot = ( e.target - c1 ).dot( ref normal );
						Debug.Assert( dot <= 0 );
						if( ( dot == 0 ) && ( ( e.target - c1 ).dot( ref t ) > 0 ) )
						{
							Point32 tmp = new Point32( 0, 0, -1 );
							if( start1 == null || ( getOrientation( start1, e, ref s, ref tmp ) == Orientation.COUNTER_CLOCKWISE ) )
							{
								start1 = e;
							}
						}
						e = e.next;
					} while( e != c1.edges );
				}

				if( start0 != null || start1 != null )
				{
					findEdgeForCoplanarFaces( c0, c1, start0, start1, null, null );
					if( start0 != null )
					{
						c0 = start0.target;
					}
					if( start1 != null )
					{
						c1 = start1.target;
					}
				}

				prevPoint = c1.point;
				prevPoint.z++;
			}
			else
			{
				prevPoint = c1.point;
				prevPoint.x++;
			}

			Vertex first0 = c0;
			Vertex first1 = c1;
			bool firstRun = true;

			while( true )
			{
				Point32 s = c1 - c0;
				Point32 r = prevPoint - c0.point;
				Point64 rxs = r.cross( ref s );
				Point64 sxrxs = s.cross( ref rxs );

#if DEBUG_CONVEX_HULL
		Console.WriteLine("\n  Checking %d %d\n", c0.point.index, c1.point.index);
#endif
				Rational64 minCot0 = new Rational64( 0, 0 );
				Edge min0 = findMaxAngle( false, c0, ref s, ref rxs, ref sxrxs, ref minCot0 );
				Rational64 minCot1 = new Rational64( 0, 0 );
				Edge min1 = findMaxAngle( true, c1, ref s, ref rxs, ref sxrxs, ref minCot1 );
				if( min0 == null && min1 == null )
				{
					Edge e = newEdgePair( c0, c1 );
					e.link( e );
					c0.edges = e;

					e = e.reverse;
					e.link( e );
					c1.edges = e;
					return;
				}
				else
				{
					int cmp = ( min0 == null ) ? 1 : ( min1 == null ) ? -1 : minCot0.compare( ref minCot1 );
#if DEBUG_CONVEX_HULL
			Console.WriteLine("    . Result %d\n", cmp);
#endif
					if( firstRun || ( ( cmp >= 0 ) ? !minCot1.isNegativeInfinity() : !minCot0.isNegativeInfinity() ) )
					{
						Edge e = newEdgePair( c0, c1 );
						if( pendingTail0 != null )
						{
							pendingTail0.prev = e;
						}
						else
						{
							pendingHead0 = e;
						}
						e.next = pendingTail0;
						pendingTail0 = e;

						e = e.reverse;
						if( pendingTail1 != null )
						{
							pendingTail1.next = e;
						}
						else
						{
							pendingHead1 = e;
						}
						e.prev = pendingTail1;
						pendingTail1 = e;
					}

					Edge e0 = min0;
					Edge e1 = min1;

#if DEBUG_CONVEX_HULL
			Console.WriteLine("   Found min edges to %d %d\n", e0 ? e0.target.point.index : -1, e1 ? e1.target.point.index : -1);
#endif

					if( cmp == 0 )
					{
						findEdgeForCoplanarFaces( c0, c1, e0, e1, null, null );
					}

					if( ( cmp >= 0 ) && e1 != null )
					{
						if( toPrev1 != null )
						{
							for( Edge e = toPrev1.next, n = null; e != min1; e = n )
							{
								n = e.next;
								removeEdgePair( e );
							}
						}

						if( pendingTail1 != null )
						{
							if( toPrev1 != null )
							{
								toPrev1.link( pendingHead1 );
							}
							else
							{
								min1.prev.link( pendingHead1 );
								firstNew1 = pendingHead1;
							}
							pendingTail1.link( min1 );
							pendingHead1 = null;
							pendingTail1 = null;
						}
						else if( toPrev1 == null )
						{
							firstNew1 = min1;
						}

						prevPoint = c1.point;
						c1 = e1.target;
						toPrev1 = e1.reverse;
					}

					if( ( cmp <= 0 ) && e0 != null )
					{
						if( toPrev0 != null )
						{
							for( Edge e = toPrev0.prev, n = null; e != min0; e = n )
							{
								n = e.prev;
								removeEdgePair( e );
							}
						}

						if( pendingTail0 != null )
						{
							if( toPrev0 != null )
							{
								pendingHead0.link( toPrev0 );
							}
							else
							{
								pendingHead0.link( min0.next );
								firstNew0 = pendingHead0;
							}
							min0.link( pendingTail0 );
							pendingHead0 = null;
							pendingTail0 = null;
						}
						else if( toPrev0 == null )
						{
							firstNew0 = min0;
						}

						prevPoint = c0.point;
						c0 = e0.target;
						toPrev0 = e0.reverse;
					}
				}

				if( ( c0 == first0 ) && ( c1 == first1 ) )
				{
					if( toPrev0 == null )
					{
						pendingHead0.link( pendingTail0 );
						c0.edges = pendingTail0;
					}
					else
					{
						for( Edge e = toPrev0.prev, n = null; e != firstNew0; e = n )
						{
							n = e.prev;
							removeEdgePair( e );
						}
						if( pendingTail0 != null )
						{
							pendingHead0.link( toPrev0 );
							firstNew0.link( pendingTail0 );
						}
					}

					if( toPrev1 == null )
					{
						pendingTail1.link( pendingHead1 );
						c1.edges = pendingTail1;
					}
					else
					{
						for( Edge e = toPrev1.next, n = null; e != firstNew1; e = n )
						{
							n = e.next;
							removeEdgePair( e );
						}
						if( pendingTail1 != null )
						{
							toPrev1.link( pendingHead1 );
							pendingTail1.link( firstNew1 );
						}
					}

					return;
				}

				firstRun = false;
			}
		}

		class pointCmp : IComparer<Point32>
		{

			int IComparer<Point32>.Compare( Point32 p, Point32 q )
			{
				if( ( p.y < q.y ) || ( ( p.y == q.y ) && ( ( p.x < q.x ) || ( ( p.x == q.x ) && ( p.z < q.z ) ) ) ) )
					return -1;
				if( ( p.y > q.y ) || ( ( p.y == q.y ) && ( ( p.x > q.x ) || ( ( p.x == q.x ) && ( p.z > q.z ) ) ) ) )
					return 1;
				return 0;
			}
		};

		public void compute( btList<btVector3> coords, int count )
		{
			btVector3 min = new btVector3( (double)( 1e30 ), (double)( 1e30 ), (double)( 1e30 ) );
			btVector3 max = new btVector3( (double)( -1e30 ), (double)( -1e30 ), (double)( -1e30 ) );
			{
				for( int i = 0; i < count; i++ )
				{
					btVector3 p = coords[i];
					//ptr += stride;
					min.setMin( ref p );
					max.setMax( ref p );
				}
			}

			btVector3 s; max.Sub( ref min, out s );
			maxAxis = s.maxAxis();
			minAxis = s.minAxis();
			if( minAxis == maxAxis )
			{
				minAxis = ( maxAxis + 1 ) % 3;
			}
			medAxis = 3 - maxAxis - minAxis;

			s.Div( (double)( 10216 ), out s );
			if( ( ( medAxis + 1 ) % 3 ) != maxAxis )
			{
				s.Mult( -1, out s );
				//s *= -1;
			}
			scaling = s;

			if( s[0] != 0 )
			{
				s[0] = (double)( 1 ) / s[0];
			}
			if( s[1] != 0 )
			{
				s[1] = (double)( 1 ) / s[1];
			}
			if( s[2] != 0 )
			{
				s[2] = (double)( 1 ) / s[2];
			}

			btVector3 tmp;
			min.Add( ref max, out tmp );
			tmp.Mult( 0.5, out center );
			//center = ( min + max ) * (double)( 0.5 );

			btList<Point32> points = new btList<Point32>( count );
			Point32[] arr_points = points.InternalArray;
			//ptr = (string)coords;
			{
				for( int i = 0; i < count; i++ )
				{
					btVector3 p = coords[i];
					//double v = ( double *) ptr;
					btVector3 tmp2;
					p.Sub( ref center, out tmp2 );
					tmp2.Mult( ref s, out p );
					arr_points[i].x = (int)p[medAxis];
					arr_points[i].y = (int)p[maxAxis];
					arr_points[i].z = (int)p[minAxis];
					arr_points[i].index = i;
				}
			}
			points.Sort( new pointCmp() );

			vertexPool.reset();
			vertexPool.setArraySize( count );
			originalVertices.Count = originalVertices.Capacity = ( count );
			for( int i = 0; i < count; i++ )
			{
				Vertex v = vertexPool.newObject();
				v.edges = null;
				v.point = points[i];
				v.copy = null;
				originalVertices[i] = v;
			}

			points.Clear();

			edgePool.reset();
			edgePool.setArraySize( 6 * count );

			usedEdgePairs = 0;
			maxUsedEdgePairs = 0;

			mergeStamp = null;

			IntermediateHull hull;
			computeInternal( 0, count, out hull );
			vertexList = hull.minXy;
#if DEBUG_CONVEX_HULL
	Console.WriteLine("max. edges {0} (3v = {1})", maxUsedEdgePairs, 3 * count);
#endif
		}

		btVector3 toBtVector( ref Point32 v )
		{
			btVector3 p = btVector3.Zero;
			p[medAxis] = (double)( v.x );
			p[maxAxis] = (double)( v.y );
			p[minAxis] = (double)( v.z );
			p.Mult( ref scaling, out p );
			return p;
		}

		btVector3 getBtNormal( Face face )
		{
			btVector3 tmp = toBtVector( ref face.dir1 );
			btVector3 tmp2;
			toBtVector( ref face.dir0 ).cross( ref tmp, out tmp2 );
			tmp2.normalized( out tmp2 );
			return tmp2;
		}

		internal btVector3 getCoordinates( Vertex v )
		{
			btVector3 p = btVector3.Zero;
			p[medAxis] = v.xvalue();
			p[maxAxis] = v.yvalue();
			p[minAxis] = v.zvalue();
			btVector3 tmp;
			p.Mult( ref scaling, out tmp );
			tmp.Add( ref center, out tmp );
			return tmp;
		}

		internal double shrink( double amount, double clampAmount )
		{
			if( vertexList == null )
			{
				return 0;
			}
			Debugger.Break();
			// this is setup, but I don't know that it's at all right.
			btConvexHullComputer.Edge stamp = mergeStamp.reverse;
			btList<Vertex> stack = new btList<Vertex>();
			vertexList.copy = stamp;
			stack.Add( vertexList );
			btList<Face> faces = new btList<Face>();

			Point32 r = vertexList.point;
			Int128 hullCenterX = new Int128( 0, 0);
			Int128 hullCenterY = new Int128( 0, 0);
			Int128 hullCenterZ = new Int128( 0, 0);
			Int128 volume = new Int128( 0, 0);

			while( stack.Count > 0 )
			{
				Vertex v = stack[stack.Count - 1];
				stack.RemoveAt( stack.Count - 1 );
				Edge e = v.edges;
				if( e!= null )
				{
					do
					{
						if( e.target.copy != stamp )
						{
							e.target.copy = stamp;
							stack.Add( e.target );
						}
						if( e.copy != stamp )
						{
							Face face = facePool.newObject();
							face.init( e.target, e.reverse.prev.target, v );
							faces.Add( face );
							Edge f = e;

							Vertex a = null;
							Vertex b = null;
							do
							{
								if( a != null && b != null )
								{
									Point32 tmp = b.point - r;
                                    Point64 tmp2 = ( a.point - r ).cross( ref tmp );
                                    long vol = ( v.point - r).dot(ref tmp2 );
									Debug.Assert( vol >= 0 );
									Point32 c = v.point + a.point + b.point + r;
									hullCenterX += vol * c.x;
									hullCenterY += vol * c.y;
									hullCenterZ += vol * c.z;
									volume += vol;
								}

								Debug.Assert( f.copy != stamp );
								f.copy = stamp;
								f.face = face;

								a = b;
								b = f.target;

								f = f.reverse.prev;
							} while( f != e );
						}
						e = e.next;
					} while( e != v.edges );
				}
			}

			if( volume.getSign() <= 0 )
			{
				return 0;
			}

			btVector3 hullCenter = new btVector3();
			hullCenter[medAxis] = hullCenterX.toScalar();
			hullCenter[maxAxis] = hullCenterY.toScalar();
			hullCenter[minAxis] = hullCenterZ.toScalar();
			hullCenter.Div( 4 * volume.toScalar(), out hullCenter );
			hullCenter.Mult( ref scaling, out hullCenter );

			int faceCount = faces.Count;

			if( clampAmount > 0 )
			{
				double minDist = btScalar.SIMD_INFINITY;
				for( int i = 0; i < faceCount; i++ )
				{
					btVector3 normal = getBtNormal( faces[i] );
					btVector3 tmp; toBtVector( ref faces[i].origin ).Sub( ref hullCenter, out tmp );
                    double dist = normal.dot( ref tmp );
					if( dist < minDist )
					{
						minDist = dist;
					}
				}

				if( minDist <= 0 )
				{
					return 0;
				}

				amount = btScalar.btMin( amount, minDist * clampAmount );
			}

			uint seed = 243703;
			Face[] arrFaces = faces.InternalArray;
			for( int i = 0; i < faceCount; i++, seed = 1664525 * seed + 1013904223 )
			{
				btScalar.btSwap( ref arrFaces[i], ref arrFaces[seed % faceCount] );
			}

			for( int i = 0; i < faceCount; i++ )
			{
				if( !shiftFace( faces[i], amount, stack ) )
				{
					return -amount;
				}
			}

			return amount;
		}

		bool shiftFace( Face face, double amount, btList<Vertex> stack )
		{
			btVector3 origShift; getBtNormal( face ).Mult( -amount, out origShift );
			if( scaling[0] != 0 )
			{
				origShift[0] /= scaling[0];
			}
			if( scaling[1] != 0 )
			{
				origShift[1] /= scaling[1];
			}
			if( scaling[2] != 0 )
			{
				origShift[2] /= scaling[2];
			}
			Point32 shift = new Point32( (int)origShift[medAxis], (int)origShift[maxAxis], (int)origShift[minAxis]);
			if( shift.isZero() )
			{
				return true;
			}
			Point64 normal = face.getNormal();
#if DEBUG_CONVEX_HULL
	Console.WriteLine("\nShrinking face (%d %d %d) (%d %d %d) (%d %d %d) by (%d %d %d)\n",
				 face.origin.x, face.origin.y, face.origin.z, face.dir0.x, face.dir0.y, face.dir0.z, face.dir1.x, face.dir1.y, face.dir1.z, shift.x, shift.y, shift.z);
#endif
			long origDot = face.origin.dot( ref normal );
			Point32 shiftedOrigin = face.origin + shift;
			long shiftedDot = shiftedOrigin.dot( ref normal );
			Debug.Assert( shiftedDot <= origDot );
			if( shiftedDot >= origDot )
			{
				return false;
			}

			Edge intersection = null;

			Edge startEdge = face.nearbyVertex.edges;
#if DEBUG_CONVEX_HULL
	Console.WriteLine("Start edge is ");
	startEdge.print();
	Console.WriteLine(", normal is (%lld %lld %lld), shifted dot is %lld\n", normal.x, normal.y, normal.z, shiftedDot);
#endif
			Rational128 optDot = face.nearbyVertex.dot( ref normal );
			int cmp = optDot.compare( shiftedDot );
#if SHOW_ITERATIONS
	int n = 0;
#endif
			if( cmp >= 0 )
			{
				Edge e = startEdge;
				do
				{
#if SHOW_ITERATIONS
			n++;
#endif
					Rational128 dot = e.target.dot( ref normal );
					Debug.Assert( dot.compare( origDot ) <= 0 );
#if DEBUG_CONVEX_HULL
			Console.WriteLine("Moving downwards, edge is ");
			e.print();
			Console.WriteLine(", dot is %f (%f %lld)\n", (float) dot.toScalar(), (float) optDot.toScalar(), shiftedDot);
#endif
					if( dot.compare( ref optDot ) < 0 )
					{
						int c = dot.compare( shiftedDot );
						optDot = dot;
						e = e.reverse;
						startEdge = e;
						if( c < 0 )
						{
							intersection = e;
							break;
						}
						cmp = c;
					}
					e = e.prev;
				} while( e != startEdge );

				if( intersection == null )
				{
					return false;
				}
			}
			else
			{
				Edge e = startEdge;
				do
				{
#if SHOW_ITERATIONS
			n++;
#endif
					Rational128 dot = e.target.dot( ref normal );
					Debug.Assert( dot.compare( origDot ) <= 0 );
#if DEBUG_CONVEX_HULL
			Console.WriteLine("Moving upwards, edge is ");
			e.print();
			Console.WriteLine(", dot is %f (%f %lld)\n", (float) dot.toScalar(), (float) optDot.toScalar(), shiftedDot);
#endif
					if( dot.compare( ref optDot ) > 0 )
					{
						cmp = dot.compare( shiftedDot );
						if( cmp >= 0 )
						{
							intersection = e;
							break;
						}
						optDot = dot;
						e = e.reverse;
						startEdge = e;
					}
					e = e.prev;
				} while( e != startEdge );

				if( intersection == null )
				{
					return true;
				}
			}

#if SHOW_ITERATIONS
	Console.WriteLine("Needed %d iterations to find initial intersection\n", n);
#endif

			if( cmp == 0 )
			{
				Edge e = intersection.reverse.next;
#if SHOW_ITERATIONS
		n = 0;
#endif
				while( e.target.dot( ref normal ).compare( shiftedDot ) <= 0 )
				{
#if SHOW_ITERATIONS
			n++;
#endif
					e = e.next;
					if( e == intersection.reverse )
					{
						return true;
					}
#if DEBUG_CONVEX_HULL
			Console.WriteLine("Checking for outwards edge, current edge is ");
			e.print();
			Console.WriteLine("\n");
#endif
				}
#if SHOW_ITERATIONS
		Console.WriteLine("Needed %d iterations to check for complete containment\n", n);
#endif
			}

			Edge firstIntersection = null;
			Edge faceEdge = null;
			Edge firstFaceEdge = null;

#if SHOW_ITERATIONS
	int m = 0;
#endif
			while( true )
			{
#if SHOW_ITERATIONS
		m++;
#endif
#if DEBUG_CONVEX_HULL
		Console.WriteLine("Intersecting edge is ");
		intersection.print();
		Console.WriteLine("\n");
#endif
				if( cmp == 0 )
				{
					Edge e2 = intersection.reverse.next;
					startEdge = e2;
#if SHOW_ITERATIONS
			n = 0;
#endif
					while( true )
					{
#if SHOW_ITERATIONS
				n++;
#endif
						if( e2.target.dot( ref normal ).compare( shiftedDot ) >= 0 )
						{
							break;
						}
						intersection = e2.reverse;
						e2 = e2.next;
						if( e2 == startEdge )
						{
							return true;
						}
					}
#if SHOW_ITERATIONS
			Console.WriteLine("Needed %d iterations to advance intersection\n", n);
#endif
				}

#if DEBUG_CONVEX_HULL
		Console.WriteLine("Advanced intersecting edge to ");
		intersection.print();
		Console.WriteLine(", cmp = %d\n", cmp);
#endif

				if( firstIntersection == null )
				{
					firstIntersection = intersection;
				}
				else if( intersection == firstIntersection )
				{
					break;
				}

				int prevCmp = cmp;
				Edge prevIntersection = intersection;
				Edge prevFaceEdge = faceEdge;

				Edge e = intersection.reverse;
#if SHOW_ITERATIONS
		n = 0;
#endif
				while( true )
				{
#if SHOW_ITERATIONS
			n++;
#endif
					e = e.reverse.prev;
					Debug.Assert( e != intersection.reverse );
					cmp = e.target.dot( ref normal ).compare( shiftedDot );
#if DEBUG_CONVEX_HULL
			Console.WriteLine("Testing edge ");
			e.print();
			Console.WriteLine(" . cmp = %d\n", cmp);
#endif
					if( cmp >= 0 )
					{
						intersection = e;
						break;
					}
				}
#if SHOW_ITERATIONS
		Console.WriteLine("Needed %d iterations to find other intersection of face\n", n);
#endif

				if( cmp > 0 )
				{
					Vertex removed = intersection.target;
					e = intersection.reverse;
					if( e.prev == e )
					{
						removed.edges = null;
					}
					else
					{
						removed.edges = e.prev;
						e.prev.link( e.next );
						e.link( e );
					}
#if DEBUG_CONVEX_HULL
			Console.WriteLine("1: Removed part contains (%d %d %d)\n", removed.point.x, removed.point.y, removed.point.z);
#endif

					Point64 n0 = intersection.face.getNormal();
					Point64 n1 = intersection.reverse.face.getNormal();
					long m00 = face.dir0.dot( ref n0 );
					long m01 = face.dir1.dot( ref n0 );
					long m10 = face.dir0.dot( ref n1 );
					long m11 = face.dir1.dot( ref n1 );
					long r0 = ( intersection.face.origin - shiftedOrigin ).dot( ref n0 );
					long r1 = ( intersection.reverse.face.origin - shiftedOrigin ).dot( ref n1 );
					Int128 det = Int128.mul( m00, m11 ) - Int128.mul( m01, m10 );
					Debug.Assert( det.getSign() != 0 );
					Vertex v = vertexPool.newObject();
					v.point.index = -1;
					v.copy = null;
					v.point128 = new PointR128( Int128.mul( face.dir0.x * r0, m11 ) - Int128.mul( face.dir0.x * r1, m01 )
																	+ Int128.mul( face.dir1.x * r1, m00 ) - Int128.mul( face.dir1.x * r0, m10 ) + det * shiftedOrigin.x,
																	Int128.mul( face.dir0.y * r0, m11 ) - Int128.mul( face.dir0.y * r1, m01 )
																	+ Int128.mul( face.dir1.y * r1, m00 ) - Int128.mul( face.dir1.y * r0, m10 ) + det * shiftedOrigin.y,
																	Int128.mul( face.dir0.z * r0, m11 ) - Int128.mul( face.dir0.z * r1, m01 )
																	+ Int128.mul( face.dir1.z * r1, m00 ) - Int128.mul( face.dir1.z * r0, m10 ) + det * shiftedOrigin.z,
																	det );
					v.point.x = (int)v.point128.xvalue();
					v.point.y = (int)v.point128.yvalue();
					v.point.z = (int)v.point128.zvalue();
					intersection.target = v;
					v.edges = e;

					stack.Add( v );
					stack.Add( removed );
					stack.Add( (Vertex)null );
				}

				if( cmp != 0 || prevCmp != 0 || ( prevIntersection.reverse.next.target != intersection.target ) )
				{
					faceEdge = newEdgePair( prevIntersection.target, intersection.target );
					if( prevCmp == 0 )
					{
						faceEdge.link( prevIntersection.reverse.next );
					}
					if( ( prevCmp == 0 ) || prevFaceEdge != null )
					{
						prevIntersection.reverse.link( faceEdge );
					}
					if( cmp == 0 )
					{
						intersection.reverse.prev.link( faceEdge.reverse );
					}
					faceEdge.reverse.link( intersection.reverse );
				}
				else
				{
					faceEdge = prevIntersection.reverse.next;
				}

				if( prevFaceEdge != null )
				{
					if( prevCmp > 0 )
					{
						faceEdge.link( prevFaceEdge.reverse );
					}
					else if( faceEdge != prevFaceEdge.reverse )
					{
						stack.Add( prevFaceEdge.target );
						while( faceEdge.next != prevFaceEdge.reverse )
						{
							Vertex removed = faceEdge.next.target;
							removeEdgePair( faceEdge.next );
							stack.Add( removed );
#if DEBUG_CONVEX_HULL
					Console.WriteLine("2: Removed part contains (%d %d %d)\n", removed.point.x, removed.point.y, removed.point.z);
#endif
						}
						stack.Add( (Vertex)null );
					}
				}
				faceEdge.face = face;
				faceEdge.reverse.face = intersection.face;

				if( firstFaceEdge == null )
				{
					firstFaceEdge = faceEdge;
				}
			}
#if SHOW_ITERATIONS
	Console.WriteLine("Needed %d iterations to process all intersections\n", m);
#endif

			if( cmp > 0 )
			{
				firstFaceEdge.reverse.target = faceEdge.target;
				firstIntersection.reverse.link( firstFaceEdge );
				firstFaceEdge.link( faceEdge.reverse );
			}
			else if( firstFaceEdge != faceEdge.reverse )
			{
				stack.Add( faceEdge.target );
				while( firstFaceEdge.next != faceEdge.reverse )
				{
					Vertex removed = firstFaceEdge.next.target;
					removeEdgePair( firstFaceEdge.next );
					stack.Add( removed );
#if DEBUG_CONVEX_HULL
			Console.WriteLine("3: Removed part contains (%d %d %d)\n", removed.point.x, removed.point.y, removed.point.z);
#endif
				}
				stack.Add( (Vertex)null );
			}

			Debug.Assert( stack.Count > 0 );
			vertexList = stack[0];

#if DEBUG_CONVEX_HULL
	Console.WriteLine("Removing part\n");
#endif
#if SHOW_ITERATIONS
	n = 0;
#endif
			int pos = 0;
			while( pos < stack.Count )
			{
				int end = stack.Count;
				while( pos < end )
				{
					Vertex kept = stack[pos++];
#if DEBUG_CONVEX_HULL
			kept.print();
#endif
					bool deeper = false;
					Vertex removed;
					while( ( removed = stack[pos++] ) != null )
					{
#if SHOW_ITERATIONS
				n++;
#endif
						kept.receiveNearbyFaces( removed );
						while( removed.edges != null )
						{
							if( !deeper )
							{
								deeper = true;
								stack.Add( kept );
							}
							stack.Add( removed.edges.target );
							removeEdgePair( removed.edges );
						}
					}
					if( deeper )
					{
						stack.Add( (Vertex)null );
					}
				}
			}
#if SHOW_ITERATIONS
	Console.WriteLine("Needed %d iterations to remove part\n", n);
#endif

			stack.Clear();
			face.origin = shiftedOrigin;

			return true;
		}


		internal static int getVertexCopy( Vertex vertex, btList<Vertex> vertices)
		{
			int index = vertex.copy == null?-1:vertex.copy.id;
			if( index < 0 )
			{
				index = vertices.Count;
				throw new Exception( "This is broken; porting changed from int to class reference broke this" );
				vertex.copy = null;// edges[index];
				vertices.Add( vertex );
#if DEBUG_CONVEX_HULL
				Console.WriteLine("Vertex %d gets index *%d\n", vertex.point.index, index);
#endif
			}
			return index;
		}

		/// Convex hull implementation based on Preparata and Hong
		/// See http://code.google.com/p/bullet/issues/detail?id=275
		/// Ole Kniemeyer, MAXON Computer GmbH


	}
	public class btConvexHullComputer
	{

		internal class Edge
		{
			internal Edge next;
			internal Edge reverse;
			internal int targetVertex;
			internal int id;

			public int getSourceVertex()
			{
				return ( reverse ).targetVertex;
			}

			public int getTargetVertex()
			{
				return targetVertex;
			}

			public Edge getNextEdgeOfVertex() // clockwise list of all edges of a vertex
			{
				return next;
			}

			public Edge getNextEdgeOfFace() // counter-clockwise list of all edges of a face
			{
				return ( reverse ).getNextEdgeOfVertex();
			}

			public Edge getReverseEdge()
			{
				return reverse;
			}
		};


		// Vertices of the output hull
		internal btList<btVector3> vertices = new btList<btVector3>();

		// Edges of the output hull
		internal btList<Edge> edges = new btList<Edge>();

		// Faces of the convex hull. Each entry is an index into the "edges" array pointing to an edge of the face. Faces are planar n-gons
		internal btList<Edge> faces = new btList<Edge>();

		/*
		Compute convex hull of "count" vertices stored in "coords". "stride" is the difference in bytes
		between the addresses of consecutive vertices. If "shrink" is positive, the convex hull is shrunken
		by that amount (each face is moved by "shrink" length units towards the center along its normal).
		If "shrinkClamp" is positive, "shrink" is clamped to not exceed "shrinkClamp * innerRadius", where "innerRadius"
		is the minimum distance of a face to the center of the convex hull.

		The returned value is the amount by which the hull has been shrunken. If it is negative, the amount was so large
		that the resulting convex hull is empty.

		The output convex hull can be found in the member variables "vertices", "edges", "faces".
		*/
		internal double compute( btList<btVector3> coords, int count, double shrink, double shrinkClamp )
		{
			if( count <= 0 )
			{
				vertices.Clear();
				edges.Clear();
				faces.Clear();
				return 0;
			}

			btConvexHullInternal hull = new btConvexHullInternal();
			hull.compute( coords, count );

			double shift = 0;
			if( ( shrink > 0 ) && ( ( shift = hull.shrink( shrink, shrinkClamp ) ) < 0 ) )
			{
				vertices.Clear();
				edges.Clear();
				faces.Clear();
				return shift;
			}

			vertices.Count = ( 0 );
			edges.Count = ( 0 );
			faces.Count = ( 0 );

			btList<btConvexHullInternal.Vertex> oldVertices = new btList<btConvexHullInternal.Vertex>();
			btConvexHullInternal.getVertexCopy( hull.vertexList, oldVertices );
			int copied = 0;
			while( copied < oldVertices.Count )
			{
				btConvexHullInternal.Vertex v = oldVertices[copied];
				vertices.Add( hull.getCoordinates( v ) );
				btConvexHullInternal.Edge firstEdge = v.edges;
				if( firstEdge != null )
				{
					Edge firstCopy = null;
					Edge prevCopy = null;
					btConvexHullInternal.Edge e = firstEdge;
					do
					{
						if( e.copy == null )
						{
							int s = edges.Count;
							edges.Add( new Edge() );
							edges.Add( new Edge() );
							Edge c = edges[s];
							Edge r = edges[s + 1];
							c.id = s;
							r.id = s + 1;
							e.copy = c;
							e.reverse.copy = r;
							c.reverse = r;
							r.reverse = c;
							c.targetVertex = btConvexHullInternal.getVertexCopy( e.target, oldVertices );
							r.targetVertex = copied;
#if DEBUG_CONVEX_HULL
					Console.WriteLine("      CREATE: Vertex *%d has edge to *%d\n", copied, c.getTargetVertex());
#endif
						}
						if( prevCopy != null )
						{
							e.copy.next = prevCopy;
						}
						else
						{
							firstCopy = e.copy;
						}
						prevCopy = e.copy;
						e = e.next;
					} while( e != firstEdge );
					firstCopy.next = prevCopy;
				}
				copied++;
			}

			for( int i = 0; i < copied; i++ )
			{
				btConvexHullInternal.Vertex v = oldVertices[i];
				btConvexHullInternal.Edge firstEdge = v.edges;
				if( firstEdge != null )
				{
					btConvexHullInternal.Edge e = firstEdge;
					do
					{
						if( e.copy != null )
						{
#if DEBUG_CONVEX_HULL
					Console.WriteLine("Vertex *%d has edge to *%d\n", i, edges[e.copy].getTargetVertex());
#endif
							faces.Add( e.copy );
							btConvexHullInternal.Edge f = e;
							do
							{
#if DEBUG_CONVEX_HULL
						Console.WriteLine("   Face *%d\n", edges[f.copy].getTargetVertex());
#endif
								f.copy = null;
								f = f.reverse.prev;
							} while( f != e );
						}
						e = e.next;
					} while( e != firstEdge );
				}
			}

			return shift;
		}


	};
}
