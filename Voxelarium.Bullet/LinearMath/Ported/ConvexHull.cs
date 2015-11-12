/*
Stan Melax Convex Hull Computation
Copyright (c) 2003-2006 Stan Melax http://www.melax.com/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

using System.Collections.Generic;
using System.Diagnostics;
using Bullet.Types;

namespace Bullet.LinearMath
{
	//----------------------------------

	internal class int3
	{
		public int x, y, z;
		public int3( int _x, int _y, int _z ) { x = _x; y = _y; z = _z; }
		public int this[int i]
		{
			get { switch( i ) { default: case 0: return x; case 1: return y; case 2: return z; } }
			set { switch( i ) { case 0: x = value; return; case 1: y = value; return; case 2: z = value; return; } }
		}
		public static bool Equals( int3 a, int3 b )
		{
			if( a.x != b.x ) return false;
			if( a.y != b.y ) return false;
			if( a.z != b.z ) return false;
			return true;
		}
		public bool Equals( int3 b )
		{
			if( x != b.x ) return false;
			if( y != b.y ) return false;
			if( z != b.z ) return false;
			return true;
		}

	};


	//------- btPlane ----------




	//--------- Utility Functions ------
	public partial struct btPlane
	{
		//btVector3 PlaneLineIntersection( btPlane &plane, btVector3 p0, btVector3 p1);
		//btVector3 PlaneProject( btPlane &plane, btVector3 point);

		void ThreePlaneIntersection( ref btPlane p0
			, ref btPlane p1
			, ref btPlane p2
			, out btVector3 result )
		{

			btVector3 n2n3; p1.normal.cross( ref p2.normal, out n2n3 );
			btVector3 n3n1; p2.normal.cross( ref p0.normal, out n3n1 );
			btVector3 n1n2; p0.normal.cross( ref p1.normal, out n1n2 );

			double quotient = ( p0.normal.dot( ref n2n3 ) );

			Debug.Assert( btScalar.btFabs( quotient ) > 0.000001 );

			quotient = -1.0 / quotient;
			n2n3.Mult( p0.dist, out n2n3 );
			n3n1.Mult( p1.dist, out n3n1 );
			n1n2.Mult( p2.dist, out n1n2 );
			btVector3 potentialVertex;
			n2n3.Add( ref n3n1, out potentialVertex );
			potentialVertex.Add( ref n3n1, out potentialVertex );
			potentialVertex.Add( ref n1n2, out potentialVertex );
			potentialVertex.Mult( quotient, out result );
		}


		public void PlaneLineIntersection( ref btVector3 p0, ref btVector3 p1, out btVector3 result )
		{
			// returns the point where the line p0-p1 intersects the plane n&d
			btVector3 dif;
			p1.Sub( ref p0, out dif );
			double dn = normal.dot( ref dif );
			double t = -( dist + normal.dot( ref p0 ) ) / dn;
			dif.Mult( t, out dif );
			p0.Add( ref dif, out result );
		}

		public void PlaneProject( btVector3 point, out btVector3 result )
		{
			btVector3 tmp;
			normal.Mult( ( point.dot( ref normal ) + dist ), out tmp );
			point.Sub( ref tmp, out result );
		}

		public static void TriNormal( ref btVector3 v0, ref btVector3 v1, ref btVector3 v2, out btVector3 result )
		{
			// return the normal of the triangle
			// inscribed by v0, v1, and v2
			btVector3 tmp1;
			btVector3 tmp2;
			v1.Sub( ref v0, out tmp1 );
			v2.Sub( ref v1, out tmp2 );
			btVector3 cp;
			tmp1.cross( ref tmp2, out cp );
			double m = cp.length();
			if( m == 0 )
			{
				result = btVector3.xAxis;
				return;
			}
			cp.Mult( ( 1.0 / m ), out result );
		}

		public static double DistanceBetweenLines( ref btVector3 ustart, ref btVector3 udir
			, ref btVector3 vstart, ref btVector3 vdir
			, out btVector3 upoint, out btVector3 vpoint )
		{
			btVector3 cp;
			udir.cross( ref vdir, out cp );
			cp.normalize();

			double distu = -cp.dot( ref ustart );
			double distv = -cp.dot( ref vstart );
			double dist = btScalar.btFabs( distu - distv );
			btPlane plane;
			btVector3 tmp;
			//if( upoint )
			{
				vdir.cross( ref cp, out plane.normal );
				plane.normal.normalize();
				plane.dist = -plane.normal.dot( ref vstart );
				ustart.Add( ref udir, out tmp );
				plane.PlaneLineIntersection( ref ustart, ref tmp, out upoint );
			}
			//if( vpoint )
			{
				udir.cross( ref cp, out plane.normal );
				plane.normal.normalize();
				plane.dist = -plane.normal.dot( ref ustart );
				ustart.Add( ref vdir, out tmp );
				plane.PlaneLineIntersection( ref vstart, ref tmp, out vpoint );
			}
			return dist;
		}
	}




	internal partial class ConvexH
	{
		internal enum PlaneTest
		{
			COPLANAR = ( 0 ),
			UNDER = ( 1 ),
			OVER = ( 2 ),
			SPLIT = ( OVER | UNDER )
		};

		readonly static double PAPERWIDTH = 0.001;
		readonly static double planetestepsilon = PAPERWIDTH;

		public ConvexH( int vertices_size, int edges_size, int facets_size )
		{
			vertices = new btList<btVector3>( vertices_size );
			edges = new btList<HalfEdge>( edges_size );
			facets = new btList<btPlane>( facets_size );
		}


		PlaneTest DoPlaneTest( ref btPlane p, ref btVector3 v )
		{
			double a = v.dot( ref p.normal ) + p.dist;
			PlaneTest flag = ( a > planetestepsilon )
				? PlaneTest.OVER
				: ( ( a < -planetestepsilon )
						? PlaneTest.UNDER
						: PlaneTest.COPLANAR );
			return flag;
		}

		PlaneTest SplitTest( ConvexH convex, btPlane plane )
		{
			PlaneTest flag = 0;
			int c = convex.vertices.Count;
			btVector3[] array = convex.vertices.InternalArray;
			for( int i = 0; i < c; i++ )
			{
				flag |= DoPlaneTest( ref plane, ref array[i] );
			}
			return flag;
		}

		/*
		internal struct VertFlag
		{
			public byte planetest;
			public byte junk;
			public byte undermap;
			public byte overmap;
		};
		internal struct EdgeFlag
		{
			public byte planetest;
			public byte fixes;
			public short undermap;
			public short overmap;
		};
		internal struct PlaneFlag
		{
			public byte undermap;
			public byte overmap;
		};
		internal struct Coplanar
		{
			public ushort ea;
			public byte v0;
			public byte v1;
		};
		*/



		static int maxdirfiltered( btVector3[] p, int count, ref btVector3 dir, int[] allow )
		{
			Debug.Assert( count >= 0 );
			int m = -1;
			for( int i = 0; i < count; i++ )
				if( allow[i] != 0 )
				{
					if( m == -1 || p[i].dot( ref dir ) > p[m].dot( ref dir ) )
						m = i;
				}
			Debug.Assert( m != -1 );
			return m;
		}

		static void orth( ref btVector3 v, out btVector3 result )
		{
			btVector3 a; v.cross( ref btVector3.Forward, out a );
			btVector3 b; v.cross( ref btVector3.yAxis, out b );
			if( a.length() > b.length() )
			{
				a.normalized( out result );
			}
			else
			{
				b.normalized( out result );
			}
		}


		static int maxdirsterid( btVector3[] p, int count, ref btVector3 dir, int[] allow )
		{
			int m = -1;
			while( m == -1 )
			{
				m = maxdirfiltered( p, count, ref dir, allow );
				if( allow[m] == 3 ) return m;
				btVector3 u; orth( ref dir, out u );
				btVector3 v; u.cross( ref dir, out v );
				int ma = -1;
				for( double x = 0.0; x <= 360.0; x += 45.0 )
				{
					double s = btScalar.btSin( btScalar.SIMD_RADS_PER_DEG * ( x ) );
					double c = btScalar.btCos( btScalar.SIMD_RADS_PER_DEG * ( x ) );
					btVector3 tmp;
					btVector3 tmp2;
					u.Mult( s, out tmp );
					v.Mult( c, out tmp2 );
					tmp.Add( ref tmp2, out tmp );
					tmp.Mult( 0.025, out tmp );
					tmp.Add( ref dir, out tmp );
					int mb = maxdirfiltered( p, count, ref tmp, allow );
					if( ma == m && mb == m )
					{
						allow[m] = 3;
						return m;
					}
					if( ma != -1 && ma != mb )  // Yuck - this is really ugly
					{
						int mc = ma;
						for( double xx = x - 40.0; xx <= x; xx += 5.0 )
						{
							double s2 = btScalar.btSin( btScalar.SIMD_RADS_PER_DEG * ( xx ) );
							double c2 = btScalar.btCos( btScalar.SIMD_RADS_PER_DEG * ( xx ) );

							u.Mult( s, out tmp );
							v.Mult( c, out tmp2 );
							tmp.Add( ref tmp2, out tmp );
							tmp.Mult( 0.025, out tmp );
							tmp.Add( ref dir, out tmp );
							int md = maxdirfiltered( p, count, ref tmp, allow );
							if( mc == m && md == m )
							{
								allow[m] = 3;
								return m;
							}
							mc = md;
						}
					}
					ma = mb;
				}
				allow[m] = 0;
				m = -1;
			}
			Debug.Assert( false );
			return m;
		}



		static bool above( btVector3[] vertices, int3 t, ref btVector3 p, double epsilon )
		{
			btVector3 n;
			btPlane.TriNormal( ref vertices[t[0]], ref vertices[t[1]], ref vertices[t[2]], out n );
			p.Sub( ref vertices[t[0]], out p );
			return ( n.dot( ref p ) > epsilon ); // btScalar.SIMD_EPSILON???
		}

		static bool hasedge( int3 t, int a, int b )
		{
			for( int i = 0; i < 3; i++ )
			{
				int i1 = ( i + 1 ) % 3;
				if( t[i] == a && t[i1] == b ) return true;
			}
			return false;
		}

		static bool hasvert( int3 t, int v )
		{
			return ( t[0] == v || t[1] == v || t[2] == v );
		}

		int shareedge( int3 a, int3 b )
		{
			int i;
			for( i = 0; i < 3; i++ )
			{
				int i1 = ( i + 1 ) % 3;
				if( hasedge( a, b[i1], b[i] ) ) return 1;
			}
			return 0;
		}



		internal class btHullTriangle
		{
			public int3 v;
			public int3 n;
			public int id;
			public int vmax;
			public double rise;
			public btHullTriangle( int a, int b, int c )
			{
				v = new int3( a, b, c );
				n = new int3( -1, -1, -1 );
				vmax = -1;
				rise = 0.0;
			}

			//public int neib( int a, int b );
			public int neib(int a, int b)
			{
				{
					int er = -1;
					int i;
					for( i = 0; i < 3; i++ )
					{
						int i1 = ( i + 1 ) % 3;
						int i2 = ( i + 2 ) % 3;
						if( v[i] == a && ( v )[i1] == b ) return n[i2];
						if( ( v )[i] == b && ( v )[i1] == a ) return n[i2];
					}
					Debug.Assert( false );
					return er;
				}
			}
			public void neib( int a, int b, int val )
			{
				{
					//int er = -1;
					int i;
					for( i = 0; i < 3; i++ )
					{
						int i1 = ( i + 1 ) % 3;
						int i2 = ( i + 2 ) % 3;
						if( v[i] == a && ( v )[i1] == b ) n[i2] = val;
						if( ( v )[i] == b && ( v )[i1] == a ) n[i2] = val;
					}
					Debug.Assert( false );
					//return er;
				}
			}
		};


		internal partial class HullLibrary
		{
			btList<btHullTriangle> m_tris = new btList<btHullTriangle>();
			public btList<int> m_vertexIndexMapping = new btList<int>();

			void b2bfix( btHullTriangle s, btHullTriangle t )
			{
				int i;
				for( i = 0; i < 3; i++ )
				{
					int i1 = ( i + 1 ) % 3;
					int i2 = ( i + 2 ) % 3;
					int a = ( s.v )[i1];
					int b = ( s.v )[i2];
					Debug.Assert( m_tris[s.neib( a, b )].neib( b, a ) == s.id );
					Debug.Assert( m_tris[t.neib( a, b )].neib( b, a ) == t.id );
					m_tris[s.neib( a, b )].neib( b, a, t.neib( b, a ) );
					m_tris[t.neib( b, a )].neib( a, b, s.neib( a, b ) );
				}
			}

			void removeb2b( btHullTriangle s, btHullTriangle t )
			{
				b2bfix( s, t );
				deAllocateTriangle( s );

				deAllocateTriangle( t );
			}

			void checkit( btHullTriangle t )
			{
				int i;
				Debug.Assert( m_tris[t.id] == t );
				for( i = 0; i < 3; i++ )
				{
					int i1 = ( i + 1 ) % 3;
					int i2 = ( i + 2 ) % 3;
					int a = ( t.v )[i1];
					int b = ( t.v )[i2];

					// release compile fix

					Debug.Assert( a != b );
					Debug.Assert( m_tris[t.n[i]].neib( b, a ) == t.id );
				}
			}

			btHullTriangle allocateTriangle( int a, int b, int c )
			{
				btHullTriangle tr = new btHullTriangle( a, b, c );
				tr.id = m_tris.Count;
				m_tris.Add( tr );

				return tr;
			}

			void deAllocateTriangle( btHullTriangle tri )
			{
				Debug.Assert( m_tris[tri.id] == tri );
				m_tris[tri.id] = null;
			}


			void extrude( btHullTriangle t0, int v )
			{
				int3 t = t0.v;
				int n = m_tris.Count;
				btHullTriangle ta = allocateTriangle( v, t[1], t[2] );
				ta.n = new int3( t0.n[0], n + 1, n + 2 );
				m_tris[t0.n[0]].neib( t[1], t[2] , n + 0 );
				btHullTriangle tb = allocateTriangle( v, t[2], t[0] );
				tb.n = new int3( t0.n[1], n + 2, n + 0 );
				m_tris[t0.n[1]].neib( t[2], t[0] , n + 1 );
				btHullTriangle tc = allocateTriangle( v, t[0], t[1] );
				tc.n = new int3( t0.n[2], n + 0, n + 1 );
				m_tris[t0.n[2]].neib( t[0], t[1] , n + 2 );
				checkit( ta );
				checkit( tb );
				checkit( tc );
				if( hasvert( m_tris[ta.n[0]].v, v ) ) removeb2b( ta, m_tris[ta.n[0]] );
				if( hasvert( m_tris[tb.n[0]].v, v ) ) removeb2b( tb, m_tris[tb.n[0]] );
				if( hasvert( m_tris[tc.n[0]].v, v ) ) removeb2b( tc, m_tris[tc.n[0]] );
				deAllocateTriangle( t0 );

			}

			btHullTriangle extrudable( double epsilon )
			{
				int i;
				btHullTriangle t = null;
				for( i = 0; i < m_tris.Count; i++ )
				{
					if( t == null || ( m_tris[i] != null && t.rise < m_tris[i].rise ) )
					{
						t = m_tris[i];
					}
				}
				return ( t.rise > epsilon ) ? t : null;
			}


			int4 FindSimplex( btVector3[] verts, int verts_count, int[] allow)
			{
				btVector3[] basis = new btVector3[3];
				basis[0] = new btVector3( 0.01, 0.02, 1.0 );
				int p0 = maxdirsterid( verts, verts_count, ref basis[0], allow );
				btVector3 tmp; basis[0].Invert( out tmp );
				int p1 = maxdirsterid( verts, verts_count, ref tmp, allow );
				verts[p0].Sub( ref  verts[p1], out basis[0] );
				//basis[0] = verts[p0] - verts[p1];
				if( p0 == p1 || basis[0].isZero() )
					return new int4( -1, -1, -1, -1 );
				tmp = new btVector3( (double)( 1 ), 0.02, (double)( 0 ) );
                btVector3.btCross( ref tmp, ref basis[0], out basis[1] );
				tmp = new btVector3( (double)( -0.02 ), (double)( 1 ), (double)( 0 ) );
                btVector3.btCross( ref tmp, ref basis[0], out basis[2] );
				if( basis[1].length() > basis[2].length() )
				{
					basis[1].normalize();
				}
				else
				{
					basis[1] = basis[2];
					basis[1].normalize();
				}
				int p2 = maxdirsterid( verts, verts_count, ref basis[1], allow );
				if( p2 == p0 || p2 == p1 )
				{
					basis[1].Invert( out tmp );
					p2 = maxdirsterid( verts, verts_count, ref tmp, allow );
				}
				if( p2 == p0 || p2 == p1 )
					return new int4( -1, -1, -1, -1 );
				verts[p2].Sub( ref verts[p0], out basis[1] );
				btVector3.btCross( ref basis[1], ref basis[0], out tmp );
                tmp.normalized( out basis[2] );
				int p3 = maxdirsterid( verts, verts_count, ref basis[2], allow );
				basis[2].Invert( out tmp );
				if( p3 == p0 || p3 == p1 || p3 == p2 ) p3 = maxdirsterid( verts, verts_count, ref tmp, allow );
				if( p3 == p0 || p3 == p1 || p3 == p2 )
					return new int4( -1, -1, -1, -1 );
				Debug.Assert( !( p0 == p1 || p0 == p2 || p0 == p3 || p1 == p2 || p1 == p3 || p2 == p3 ) );
				btVector3 tmp2;
				verts[p1].Sub( ref verts[p0], out tmp );
				verts[p2].Sub( ref verts[p0], out tmp2 );
				btVector3 tmp3;
				btVector3.btCross( ref tmp, ref tmp2, out tmp3 );
				verts[p3].Sub( ref verts[p0], out tmp );
                if( btVector3.btDot( ref tmp, ref tmp3  ) < 0 ) { btScalar.btSwap( ref p2, ref p3 ); }
				return new int4( p0, p1, p2, p3 );
			}

			int calchullgen( btVector3[] verts, int verts_count, int vlimit )
			{
				if( verts_count < 4 ) return 0;
				if( vlimit == 0 ) vlimit = 1000000000;
				int j;
				btVector3 bmin = verts[0], bmax= verts[0];
				btList<int> isextreme = new btList<int>( verts_count );
				btList<int> allow = new btList<int>( verts_count );

				for( j = 0; j < verts_count; j++ )
				{
					allow.Add( 1 );
					isextreme.Add( 0 );
					bmin.setMin( ref verts[j] );
					bmax.setMax( ref verts[j] );
				}
				btVector3 del;
				bmax.Sub( ref bmin, out del );
				double epsilon = ( del ).length() * 0.001;
				Debug.Assert( epsilon != 0.0 );


				int4 p = FindSimplex( verts, verts_count, allow.InternalArray );
				if( p.x == -1 ) return 0; // simplex failed


				btVector3 tmp;
				btVector3 center; //= ( verts[p[0]] + verts[p[1]] + verts[p[2]] + verts[p[3]] ) / 4.0;  // a valid interior point
				verts[p[0]].Add( ref verts[p[1]], out tmp );
				tmp.Add( ref verts[p[2]], out tmp );
				tmp.Add( ref verts[p[3]], out tmp );
				tmp.Div( 4, out center );
				btHullTriangle t0 = allocateTriangle( p[2], p[3], p[1] ); t0.n = new int3( 2, 3, 1 );
				btHullTriangle t1 = allocateTriangle( p[3], p[2], p[0] ); t1.n = new int3( 3, 2, 0 );
				btHullTriangle t2 = allocateTriangle( p[0], p[1], p[3] ); t2.n = new int3( 0, 1, 3 );
				btHullTriangle t3 = allocateTriangle( p[1], p[0], p[2] ); t3.n = new int3( 1, 0, 2 );
				isextreme[p[0]] = isextreme[p[1]] = isextreme[p[2]] = isextreme[p[3]] = 1;
				checkit( t0 ); checkit( t1 ); checkit( t2 ); checkit( t3 );

				for( j = 0; j < m_tris.Count; j++ )
				{
					btHullTriangle t = m_tris[j];
					Debug.Assert( t != null );
					Debug.Assert( t.vmax < 0 );
					btVector3 n; btPlane.TriNormal( ref verts[ t.v[0]], ref verts[ t.v[1]], ref verts[ t.v[2]], out n );
					t.vmax = maxdirsterid( verts, verts_count, ref n, allow.InternalArray );
					btVector3 tmp2;
					verts[t.vmax].Sub( ref verts[( t ).v[0]], out tmp2 );
					t.rise = btVector3.btDot( ref n, ref tmp );
				}
				btHullTriangle te;
				vlimit -= 4;
				while( vlimit > 0 && ( ( te = extrudable( epsilon ) ) != null ) )
				{
					//int3 ti=te;
					int v = te.vmax;
					Debug.Assert( v != -1 );
					Debug.Assert( isextreme[v] == 0 );  // wtf we've already done this vertex
					isextreme[v] = 1;
					//if(v==p0 || v==p1 || v==p2 || v==p3) continue; // done these already
					j = m_tris.Count;
					while( j-- != 0  )
					{
						if( m_tris[j] == null ) continue;
						int3 t = m_tris[j].v;
						if( above( verts, t, ref verts[v], 0.01 * epsilon ) )
						{
							extrude( m_tris[j], v );
						}
					}
					// now check for those degenerate cases where we have a flipped triangle or a really skinny triangle
					j = m_tris.Count;
					while( j-- != 0 )
					{
						if( m_tris[j] == null ) continue;
						if( !hasvert( m_tris[j].v, v ) ) break;
						int3 nt = m_tris[j].v;
						btVector3 tmp2;
						btVector3 tmp3;
						verts[nt[1]].Sub( ref verts[nt[0]], out tmp );
						verts[nt[2]].Sub( ref verts[nt[1]], out tmp2 );
                        btVector3.btCross( ref tmp, ref tmp2, out tmp3 );
                        if( above( verts, nt, ref center, 0.01 * epsilon ) || 
							tmp3.length() < epsilon * epsilon * 0.1 )
						{
							btHullTriangle nb = m_tris[m_tris[j].n[0]];
							Debug.Assert( nb != null ); Debug.Assert( !hasvert( nb.v, v ) ); Debug.Assert( nb.id < j );
							extrude( nb, v );
							j = m_tris.Count;
						}
					}
					j = m_tris.Count;
					while( j-- != 0 )
					{
						btHullTriangle t = m_tris[j];
						if( t == null ) continue;
						if( t.vmax >= 0 ) break;
						btVector3 n; btPlane.TriNormal( ref verts[( t ).v[0]], ref verts[( t ).v[1]], ref verts[( t ).v[2]], out n );
						t.vmax = maxdirsterid( verts, verts_count, ref n, allow.InternalArray );
						if( isextreme[t.vmax] != 0 )
						{
							t.vmax = -1; // already done that vertex - algorithm needs to be able to terminate.
						}
						else
						{
							verts[t.vmax].Sub( ref verts[( t ).v[0]], out tmp );
                            t.rise = btVector3.btDot( ref n, ref tmp );
						}
					}
					vlimit--;
				}
				return 1;
			}

			int calchull( btVector3[] verts, int verts_count, out TUIntArray tris_out, out int tris_count, int vlimit )
			{
				int rc = calchullgen( verts, verts_count, vlimit );
				if( rc == 0 )
				{
					tris_out = null;
					tris_count = 0;
					return 0;
				}
				btList<int> ts = new btList<int>();
				int i;

				for( i = 0; i < m_tris.Count; i++ )
				{
					if( m_tris[i] != null )
					{
						for( int j = 0; j < 3; j++ )
							ts.Add( ( m_tris[i].v)[j] );
						deAllocateTriangle( m_tris[i] );
					}
				}
				tris_count = ts.Count / 3;
				tris_out = new TUIntArray( ts.Count );

				for( i = 0; i < ts.Count; i++ )
				{
					tris_out[i] = (uint)( ts[i] );
				}
				m_tris.Count = ( 0 );

				return 1;
			}





			bool ComputeHull( int vcount, btVector3[] vertices, PHullResult result, int vlimit )
			{

				int tris_count;
				int ret = calchull( vertices, (int)vcount, out result.m_Indices, out tris_count,  (int)vlimit  );
				if( ret == 0 ) {
					return false;
				}
				result.mIndexCount = ( tris_count * 3 );
				result.mFaceCount = tris_count;
				result.mVertices.Add( vertices );
				result.mVcount = vcount;
				return true;

			}


			void ReleaseHull( PHullResult result)
			{
				if( result.m_Indices.Count > 0 )
				{
					result.m_Indices.Clear();
				}
				result.mVcount = 0;
				result.mIndexCount = 0;
				result.mVertices = null;
			}


			//*****************************
			//*****************************
			//*[]*  HullLib header
			//*****************************
			//*****************************

			//*****************************
			//*****************************
			//*[]*  HullLib implementation
			//*****************************
			//*****************************

			HullError CreateConvexHull( HullDesc       desc,           // describes the input request
										HullResult           result)         // contains the resulst
			{
				HullError ret = HullError.QE_FAIL;

				PHullResult hr = new PHullResult();

				int vcount = desc.mVcount;
				if( vcount < 8 ) vcount = 8;

				btList<btVector3> vertexSource = new btList<btVector3>();
				vertexSource.Count = vertexSource.Capacity = ( (int)vcount  );

				btVector3 scale = btVector3.Zero;

				int ovcount;

				bool ok = CleanupVertices( desc.mVcount, desc.mVertices.InternalArray, desc.mVertexStride
						, out ovcount, vertexSource, desc.mNormalEpsilon, ref scale ); // normalize point cloud, remove duplicates!

				if( ok )
				{


					//		if ( 1 ) // scale vertices back to their original size.
					{
						for( uint i = 0; i < ovcount; i++ )
						{
							btVector3 v = vertexSource[(int)( i )];
							v[0] = scale[0];
							v[1] = scale[1];
							v[2] = scale[2];
						}
					}

					ok = ComputeHull( ovcount, vertexSource.InternalArray, hr, desc.mMaxVertices );

					if( ok )
					{

						// re-index triangle mesh so it refers to only used vertices, rebuild a new vertex table.
						btList<btVector3> vertexScratch = new btList<btVector3>( (int)hr.mVcount );

						BringOutYourDead( hr.mVertices, hr.mVcount, vertexScratch, out ovcount, hr.m_Indices, hr.mIndexCount );

						ret = HullError.QE_OK;

						if( desc.HasHullFlag( HullFlag.QF_TRIANGLES ) ) // if he wants the results as triangle!
						{
							result.mPolygons = false;
							result.mNumOutputVertices = ovcount;
							result.m_OutputVertices.Count = result.m_OutputVertices.Capacity = ovcount;
							result.mNumFaces = hr.mFaceCount;
							result.mNumIndices = hr.mIndexCount;

							result.m_Indices.Count = result.m_Indices.Capacity = hr.mIndexCount;
							for( int j = 0; j < ovcount; j++ )
								result.m_OutputVertices[j] = vertexScratch[j];
							//memcpy( result.m_OutputVertices, vertexScratch, sizeof( btVector3 ) * ovcount );

							uint[] source = hr.m_Indices.InternalArray;
							uint[] dest = result.m_Indices.InternalArray;
							if( desc.HasHullFlag( HullFlag.QF_REVERSE_ORDER ) )
							{


								for( uint i = 0; i < hr.mFaceCount; i++ )
								{
									//dest = source;
									dest[i*3+0] = source[i*3+2];
									dest[i * 3 + 1] = source[i * 3 + 1];
									dest[i * 3 + 2] = source[i * 3 + 0];
									//dest += 3;
									//source += 3;
								}

							}
							else
							{
								for( int i = 0; i < 3*hr.mIndexCount; i++ )
									dest[i] = source[i];
								//memcpy( result.m_Indices, hr.m_Indices, sizeof( uint ) * hr.mIndexCount );
							}
						}
						else
						{

							result.mPolygons = true;
							result.mNumOutputVertices = ovcount;
							result.m_OutputVertices.Count = result.m_OutputVertices.Capacity = ovcount;
							result.mNumFaces = hr.mFaceCount;
							result.mNumIndices = hr.mIndexCount + hr.mFaceCount;
							result.m_Indices.Count = result.m_Indices.Capacity = ( result.mNumIndices );
							{
								btVector3[] dest = result.m_OutputVertices.InternalArray;
								btVector3[] source = vertexScratch.InternalArray;
								for( int i = 0; i < 3 * hr.mIndexCount; i++ )
									dest[i] = source[i];
							}
							//memcpy( result.m_OutputVertices, vertexScratch, sizeof( btVector3 ) * ovcount );

							//				if ( 1 )
							{
								uint[] source = hr.m_Indices.InternalArray;
								uint[] dest = result.m_Indices.InternalArray;
								for( uint i = 0; i < hr.mFaceCount; i++ )
								{
									dest[i*4 + 0] = 3;
									if( desc.HasHullFlag( HullFlag.QF_REVERSE_ORDER ) )
									{
										dest[i * 4 + 1] = source[i * 3 + 2];
										dest[i * 4 + 2] = source[i * 3 + 1];
										dest[i * 4 + 3] = source[i * 3 + 0];
									}
									else
									{
										dest[i * 4 + 1] = source[i * 3 + 0];
										dest[i * 4 + 2] = source[i * 3 + 1];
										dest[i * 4 + 3] = source[i*3+2];
									}

									//dest += 4;
									//source += 3;
								}
							}
						}
						ReleaseHull( hr );
					}
				}

				return ret;
			}



			HullError ReleaseResult( HullResult result) // release memory allocated for this result, we are done with it.
			{
				if( result.m_OutputVertices.Count != 0 )
				{
					result.mNumOutputVertices = 0;
					result.m_OutputVertices.Clear();
				}
				if( result.m_Indices.Count > 0 )
				{
					result.mNumIndices = 0;
					result.m_Indices.Clear();
				}
				return HullError.QE_OK;
			}


			static void addPoint( ref int vcount, btList<btVector3> p, double x, double y, double z )
			{
				// XXX, might be broken
				p.InternalArray[vcount].x = x;
				p.InternalArray[vcount].y = y;
				p.InternalArray[vcount].z = z;
				vcount++;
			}

			double GetDist( double px, double py, double pz, ref btVector3 p2 )
			{

				double dx = px - p2.x;
				double dy = py - p2.y;
				double dz = pz - p2.z;

				return dx * dx + dy * dy + dz * dz;
			}



			bool CleanupVertices( int svcount,
							   btVector3[] svertices,
							   int stride,
							   out int vcount,       // output number of vertices
							   btList<btVector3> vertices,                 // location to store the results.
							   double normalepsilon,
							   ref btVector3 scale)
			{
				if( svcount == 0 ) {
					vcount = 0;
					return false;
				}

				m_vertexIndexMapping.Count =( 0 );

				vcount = 0;

				double[] recip = new double[3];

				//if( scale )
				{
					scale[0] = 1;
					scale[1] = 1;
					scale[2] = 1;
				}

				double[] bmin = new double[3] { btScalar.BT_MAX_FLOAT, btScalar.BT_MAX_FLOAT, btScalar.BT_MAX_FLOAT };
				double[] bmax = new double[3] { btScalar.BT_MIN_FLOAT, btScalar.BT_MIN_FLOAT, btScalar.BT_MIN_FLOAT };

				//char vtx = (char)svertices;

				//	if ( 1 )
				{
					for( uint i = 0; i < svcount; i++ )
					{						
						for( int j = 0; j < 3; j++ )
						{
							if( svertices[i][j] < bmin[j] ) bmin[j] = svertices[i][j];
							if( svertices[i][j] > bmax[j] ) bmax[j] = svertices[i][j];
						}
					}
				}

				double dx = bmax[0] - bmin[0];
				double dy = bmax[1] - bmin[1];
				double dz = bmax[2] - bmin[2];

				btVector3 center = new btVector3( dx * (double)( 0.5 ) + bmin[0],
					dy * (double)( 0.5 ) + bmin[1],
					dz * (double)( 0.5 ) + bmin[2] );

				if( dx < btScalar.SIMD_EPSILON || dy < btScalar.SIMD_EPSILON || dz < btScalar.SIMD_EPSILON || svcount < 3 )
				{

					double len = btScalar.BT_MAX_FLOAT;

					if( dx > btScalar.SIMD_EPSILON & dx < len ) len = dx;
					if( dy > btScalar.SIMD_EPSILON && dy < len ) len = dy;
					if( dz > btScalar.SIMD_EPSILON && dz < len ) len = dz;

					if( len == btScalar.BT_MAX_FLOAT )
					{
						dx = dy = dz = (double)( 0.01 ); // one centimeter
					}
					else
					{
						if( dx < btScalar.SIMD_EPSILON ) dx = len * (double)( 0.05 ); // 1/5th the shortest non-zero edge.
						if( dy < btScalar.SIMD_EPSILON ) dy = len * (double)( 0.05 );
						if( dz < btScalar.SIMD_EPSILON ) dz = len * (double)( 0.05 );
					}

					double x1 = center.x - dx;
					double x2 = center.x + dx;

					double y1 = center.y - dy;
					double y2 = center.y + dy;

					double z1 = center.z - dz;
					double z2 = center.z + dz;

					addPoint( ref vcount, vertices, x1, y1, z1 );
					addPoint( ref vcount, vertices, x2, y1, z1 );
					addPoint( ref vcount, vertices, x2, y2, z1 );
					addPoint( ref vcount, vertices, x1, y2, z1 );
					addPoint( ref vcount, vertices, x1, y1, z2 );
					addPoint( ref vcount, vertices, x2, y1, z2 );
					addPoint( ref vcount, vertices, x2, y2, z2 );
					addPoint( ref vcount, vertices, x1, y2, z2 );

					return true; // return cube


				}
				else
				{
					//if( scale )
					{
						scale[0] = dx;
						scale[1] = dy;
						scale[2] = dz;

						recip[0] = 1 / dx;
						recip[1] = 1 / dy;
						recip[2] = 1 / dz;

						center.x = recip[0];
						center.y = recip[1];
						center.z = recip[2];

					}

				}



				//vtx = (stringchar)svertices;

				for( uint i = 0; i < svcount; i++ )
				{
					//btVector3 p = (btVector3)vtx;
					//vtx += stride;

					double px = svertices[i].x;
					double py = svertices[i].y;
					double pz = svertices[i].z;

					//if( scale )
					{
						px = px * recip[0]; // normalize
						py = py * recip[1]; // normalize
						pz = pz * recip[2]; // normalize
					}

					//		if ( 1 )
					{
						int j;

						btVector3[] v = vertices.InternalArray;
						for( j = 0; j < vcount; j++ )
						{
							/// XXX might be broken

							double x = v[j][0];
							double y = v[j][1];
							double z = v[j][2];

							dx = btScalar.btFabs( x - px );
							dy = btScalar.btFabs( y - py );
							dz = btScalar.btFabs( z - pz );

							if( dx < normalepsilon && dy < normalepsilon && dz < normalepsilon )
							{
								// ok, it is close enough to the old one
								// now let us see if it is further from the center of the point cloud than the one we already recorded.
								// in which case we keep this one instead.

								double dist1 = GetDist( px, py, pz, ref center );
								double dist2 = GetDist( v[j][0], v[j][1], v[j][2], ref center );

								if( dist1 > dist2 )
								{
									v[j][0] = px;
									v[j][1] = py;
									v[j][2] = pz;

								}

								break;
							}
						}

						if( j == vcount )
						{
							vertices.InternalArray[vcount][0] = px;
							vertices.InternalArray[vcount][1] = py;
							vertices.InternalArray[vcount][2] = pz;
							vcount++;
						}
						m_vertexIndexMapping.Add( j );
					}
				}

				// ok..now make sure we didn't prune so many vertices it is now invalid.
				//	if ( 1 )
				{
					/* reset min max */
					bmin = new double[] { btScalar.BT_MAX_FLOAT, btScalar.BT_MAX_FLOAT, btScalar.BT_MAX_FLOAT };
					bmax = new double[] { btScalar.BT_MIN_FLOAT, btScalar.BT_MIN_FLOAT, btScalar.BT_MIN_FLOAT };

					for( int i = 0; i < vcount; i++ )
					{
						btVector3 p = vertices[i];
						for( int j = 0; j < 3; j++ )
						{
							if( p[j] < bmin[j] ) bmin[j] = p[j];
							if( p[j] > bmax[j] ) bmax[j] = p[j];
						}
					}

					dx = bmax[0] - bmin[0];
					dy = bmax[1] - bmin[1];
					dz = bmax[2] - bmin[2];

					if( dx < btScalar.SIMD_EPSILON || dy < btScalar.SIMD_EPSILON || dz < btScalar.SIMD_EPSILON || vcount < 3 )
					{
						double cx = dx * 0.5 + bmin[0];
						double cy = dy * 0.5 + bmin[1];
						double cz = dz * 0.5 + bmin[2];

						double len = btScalar.BT_MAX_FLOAT;

						if( dx >= btScalar.SIMD_EPSILON && dx < len ) len = dx;
						if( dy >= btScalar.SIMD_EPSILON && dy < len ) len = dy;
						if( dz >= btScalar.SIMD_EPSILON && dz < len ) len = dz;

						if( len == btScalar.BT_MAX_FLOAT )
						{
							dx = dy = dz = (double)( 0.01 ); // one centimeter
						}
						else
						{
							if( dx < btScalar.SIMD_EPSILON ) dx = len * (double)( 0.05 ); // 1/5th the shortest non-zero edge.
							if( dy < btScalar.SIMD_EPSILON ) dy = len * (double)( 0.05 );
							if( dz < btScalar.SIMD_EPSILON ) dz = len * (double)( 0.05 );
						}

						double x1 = cx - dx;
						double x2 = cx + dx;

						double y1 = cy - dy;
						double y2 = cy + dy;

						double z1 = cz - dz;
						double z2 = cz + dz;

						vcount = 0; // add box

						addPoint( ref vcount, vertices, x1, y1, z1 );
						addPoint( ref vcount, vertices, x2, y1, z1 );
						addPoint( ref vcount, vertices, x2, y2, z1 );
						addPoint( ref vcount, vertices, x1, y2, z1 );
						addPoint( ref vcount, vertices, x1, y1, z2 );
						addPoint( ref vcount, vertices, x2, y1, z2 );
						addPoint( ref vcount, vertices, x2, y2, z2 );
						addPoint( ref vcount, vertices, x1, y2, z2 );

						return true;
					}
				}

				return true;
			}

			void BringOutYourDead( btList<btVector3> verts, int vcount, btList<btVector3> overts, out int ocount, TUIntArray indices, int indexcount )
			{
				btList<int> tmpIndices = new btList<int>( m_vertexIndexMapping.Count );
				int i;

				for( i = 0; i < m_vertexIndexMapping.Count; i++ )
				{
					tmpIndices[i] = m_vertexIndexMapping[i];
				}

				TUIntArray usedIndices = new TUIntArray( (int)vcount );
				//usedIndices.Capacity( (int)vcount ) );
				//memset( usedIndices, 0, sizeof( uint ) * vcount );

				ocount = 0;

				for( i = 0; i < (int)indexcount; i++ )
				{
					int v = (int)indices[i]; // original array index

					Debug.Assert( v >= 0 && v < vcount );

					if( usedIndices[(int) v] != 0 ) // if already remapped
					{
						indices[i] = usedIndices[v] - 1; // index to new array
					}
					else
					{

						indices[i] = (uint)ocount;      // new index mapping

						overts[ocount] = verts[v]; // copy old vert to new vert array

						for( int k = 0; k < m_vertexIndexMapping.Count; k++ )
						{
							if( tmpIndices[k] ==  v )
								m_vertexIndexMapping[k] = ocount;
						}

						ocount++; // increment output vert count

						Debug.Assert( ocount >= 0 && ocount <= vcount );

						usedIndices[v] = (uint)ocount; // assign new index remapping


					}
				}
			}

		}

	}
}