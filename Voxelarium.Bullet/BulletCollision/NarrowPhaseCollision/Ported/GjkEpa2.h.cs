/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the
use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
claim that you wrote the original software. If you use this software in a
product, an acknowledgment in the product documentation would be appreciated
but is not required.
2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
GJK-EPA collision solver by Nathanael Presson, 2008
*/
using System.Runtime.CompilerServices;
using Bullet.Collision.Shapes;
using Bullet.LinearMath;

namespace Bullet.Collision.NarrowPhase
{

	public class btGjkEpaSolver2
	{
		///btGjkEpaSolver contributed under zlib by Nathanael Presson
		public struct sResults
		{
			public enum eStatus
			{
				Separated,      /* Shapes doesnt penetrate												*/
				Penetrating,    /* Shapes are penetrating												*/
				GJK_Failed,     /* GJK phase fail, no big issue, shapes are probably just 'touching'	*/
				EPA_Failed      /* EPA phase fail, bigger problem, need to save parameters, and debug	*/
			};
			public eStatus status;
			//public btVector3[] witnesses = new btVector3[2];
			public btVector3 witness0;
			public btVector3 witness1;
			public btVector3 normal;
			public double distance;
		};
#if false
		public static int StackSizeRequirement()
		{
			return ( sizeof( GJK ) + sizeof( EPA ) );
		}
#endif
		// Config

		/* GJK	*/
		const int GJK_MAX_ITERATIONS = 128;
		const double GJK_ACCURARY = ( (double)0.0001 );
		const double GJK_MIN_DISTANCE = ( (double)0.0001 );
		const double GJK_DUPLICATED_EPS = ( (double)0.0001 );
		const double GJK_SIMPLEX2_EPS = ( (double)0.0 );
		const double GJK_SIMPLEX3_EPS = ( (double)0.0 );
		const double GJK_SIMPLEX4_EPS = ( (double)0.0 );

		/* EPA	*/
		const int EPA_MAX_VERTICES = 64;
		const int EPA_MAX_FACES = ( EPA_MAX_VERTICES * 2 );
		const int EPA_MAX_ITERATIONS = 255;
		const double EPA_ACCURACY = ( (double)0.0001 );
		const double EPA_FALLBACK = ( 10 * EPA_ACCURACY );
		const double EPA_PLANE_EPS = ( (double)0.00001 );
		const double EPA_INSIDE_EPS = ( (double)0.01 );

		// Shorthands

		// MinkowskiDiff
		internal class MinkowskiDiff
		{
			public btConvexShape[] m_shapes = new btConvexShape[2];
			public btMatrix3x3 m_toshape1;
			public btTransform m_toshape0;
			bool m_enableMargin;

			internal void EnableMargin( bool enable )
			{
				m_enableMargin = enable;
			}

			void Support0( ref btVector3 d, out btVector3 result )
			{
				if( m_enableMargin )
				{
					m_shapes[0].localGetSupportVertexNonVirtual( ref d, out result );
				}
				else
				{
					m_shapes[0].localGetSupportVertexWithoutMarginNonVirtual( ref d, out result );
				}
			}
			void Support1( ref btVector3 d, out btVector3 result )
			{
				btVector3 tmp;
				btVector3 tmp2;
				if( m_enableMargin )
				{
					m_toshape1.Apply( ref d, out tmp );
					m_shapes[1].localGetSupportVertexNonVirtual( ref tmp, out tmp2 );
					m_toshape0.Apply( ref tmp2, out result );
				}
				else
				{
					m_toshape1.Apply( ref d, out tmp );
					m_shapes[1].localGetSupportVertexWithoutMarginNonVirtual( ref tmp, out tmp2 );
					m_toshape0.Apply( ref tmp2, out result );
					//return m_toshape0.Apply( ( m_shapes[1]->localGetSupportVertexWithoutMarginNonVirtual( m_toshape1 * d ) );
				}
			}


			//[MethodImpl( MethodImplOptions.AggressiveInlining )]
			internal void Support( ref btVector3 d, out btVector3 result )
			{
				btVector3 tmp, tmp2;
				Support0( ref d, out tmp );
				Support1( ref d, out tmp2 );
				tmp2.Invert( out tmp2 );
				tmp.Sub( ref tmp2, out result );
				//return ( Support0( d ) - Support1( -d ) );
			}

			internal void Support( ref btVector3 d, uint index, out btVector3 result )
			{
				if( index != 0 )
					Support1( ref d, out result );
				else
					Support0( ref d, out result );
			}
		};

		internal class tShape : MinkowskiDiff
		{
		}


		internal class sSV
		{
			internal btVector3 d, w;
		};

		// GJK
		internal class GJK
		{
			/* Types		*/
			internal class sSimplex
			{
				internal sSV[] c = new sSV[4];
				internal double[] p = new double[4];
				internal uint rank;
			};
			internal struct eStatus
			{
				internal enum _
				{
					Valid,
					Inside,
					Failed
				};
			};
			/* Fields		*/
			internal tShape m_shape;
			internal btVector3 m_ray;
			internal double m_distance;
			internal sSimplex[] m_simplices = new sSimplex[2];
			internal sSV[] m_free = new sSV[4];
			internal uint m_nfree;
			internal uint m_current;
			internal sSimplex m_simplex;
			internal eStatus._ m_status;
			/* Methods		*/
			internal GJK()
			{
				Initialize();
			}
			void Initialize()
			{
				m_ray = btVector3.Zero;
				m_nfree = 0;
				m_status = eStatus._.Failed;
				m_current = 0;
				m_distance = 0;
			}
			internal eStatus._ Evaluate( tShape shapearg, ref btVector3 guess )
			{
				uint iterations = 0;
				double sqdist = 0;
				double alpha = 0;
				btVector3[] lastw = new btVector3[4];
				uint clastw = 0;
				/* Initialize solver		*/
				m_free[0] = new sSV();
				m_free[1] = new sSV();
				m_free[2] = new sSV();
				m_free[3] = new sSV();
				m_nfree = 4;
				m_current = 0;
				m_status = eStatus._.Valid;
				m_shape = shapearg;
				m_distance = 0;
				/* Initialize simplex		*/
				m_simplices[0].rank = 0;
				m_ray = guess;
				double sqrl = m_ray.length2();
				btVector3 tmp;
				if( sqrl > 0 )
					m_ray.Invert( out tmp );
				else
					tmp = btVector3.xAxis;
				appendvertice( m_simplices[0], ref tmp );
				m_simplices[0].p[0] = 1;
				m_ray = m_simplices[0].c[0].w;
				sqdist = sqrl;
				lastw[0] =
					lastw[1] =
					lastw[2] =
					lastw[3] = m_ray;
				/* Loop						*/
				do
				{
					uint next = 1 - m_current;
					sSimplex cs = m_simplices[m_current];
					sSimplex ns = m_simplices[next];
					/* Check zero							*/
					double rl = m_ray.length();
					if( rl < GJK_MIN_DISTANCE )
					{/* Touching or inside				*/
						m_status = eStatus._.Inside;
						break;
					}
					/* Append new vertice in -'v' direction	*/
					m_ray.Invert( out tmp );
					appendvertice( cs, ref tmp );
					btVector3 w = cs.c[cs.rank - 1].w;
					bool found = false;
					for( uint i = 0; i < 4; ++i )
					{
						w.Sub( ref lastw[i], out tmp );
						if( tmp.length2() < GJK_DUPLICATED_EPS )
						{ found = true; break; }
					}
					if( found )
					{/* Return old simplex				*/
						removevertice( m_simplices[m_current] );
						break;
					}
					else
					{/* Update lastw					*/
						lastw[clastw = ( clastw + 1 ) & 3] = w;
					}
					/* Check for termination				*/
					double omega = btVector3.btDot( ref m_ray, ref w ) / rl;
					alpha = btScalar.btMax( omega, alpha );
					if( ( ( rl - alpha ) - ( GJK_ACCURARY * rl ) ) <= 0 )
					{/* Return old simplex				*/
						removevertice( m_simplices[m_current] );
						break;
					}
					/* Reduce simplex						*/
					double[] weights = new double[4];
					uint mask = 0;
					switch( cs.rank )
					{
						case 2:
							sqdist = projectorigin( ref cs.c[0].w,
								ref cs.c[1].w,
								weights, out mask ); break;
						case 3:
							sqdist = projectorigin( ref cs.c[0].w,
								ref cs.c[1].w,
								ref cs.c[2].w,
								weights, out mask ); break;
						case 4:
							sqdist = projectorigin( ref cs.c[0].w,
								ref cs.c[1].w,
								ref cs.c[2].w,
								ref cs.c[3].w,
								weights, out mask ); break;
					}
					if( sqdist >= 0 )
					{/* Valid	*/
						ns.rank = 0;
						m_ray = btVector3.Zero;
						m_current = next;
						for( int i = 0, ni = (int)cs.rank; i < ni; ++i )
						{
							if( ( mask & ( (uint)1 << i ) ) != 0 )
							{
								ns.c[ns.rank] = cs.c[i];
								ns.p[ns.rank++] = weights[i];
								btVector3 tmp2;
								cs.c[i].w.Mult( weights[i], out tmp2 );
								m_ray.Add( ref tmp2, out m_ray );
							}
							else
							{
								m_free[m_nfree++] = cs.c[i];
							}
						}
						if( mask == 15 ) m_status = eStatus._.Inside;
					}
					else
					{/* Return old simplex				*/
						removevertice( m_simplices[m_current] );
						break;
					}
					m_status = ( ( ++iterations ) < GJK_MAX_ITERATIONS ) ? m_status : eStatus._.Failed;
				} while( m_status == eStatus._.Valid );
				m_simplex = m_simplices[m_current];
				switch( m_status )
				{
					case eStatus._.Valid: m_distance = m_ray.length(); break;
					case eStatus._.Inside: m_distance = 0; break;
					default:
						break;
				}
				return ( m_status );
			}
			internal bool EncloseOrigin()
			{
				switch( m_simplex.rank )
				{
					case 1:
						{
							for( int i = 0; i < 3; ++i )
							{
								btVector3 axis = btVector3.Zero;
								axis[i] = 1;
								appendvertice( m_simplex, ref axis );
								if( EncloseOrigin() ) return ( true );
								removevertice( m_simplex );
								btVector3 tmp2;
								axis.Invert( out tmp2 );
								appendvertice( m_simplex, ref tmp2 );
								if( EncloseOrigin() ) return ( true );
								removevertice( m_simplex );
							}
						}
						break;
					case 2:
						{
							btVector3 d; m_simplex.c[1].w.Sub( ref m_simplex.c[0].w, out d );
							for( int i = 0; i < 3; ++i )
							{
								btVector3 axis = btVector3.Zero;
								axis[i] = 1;
								btVector3 p; btVector3.btCross( ref d, ref axis, out p );
								if( p.length2() > 0 )
								{
									appendvertice( m_simplex, ref p );
									if( EncloseOrigin() ) return ( true );
									removevertice( m_simplex );
									btVector3 tmp2;
									p.Invert( out tmp2 );
									appendvertice( m_simplex, ref tmp2 );
									if( EncloseOrigin() ) return ( true );
									removevertice( m_simplex );
								}
							}
						}
						break;
					case 3:
						{
							btVector3 n; btVector3.btCross2Del( ref m_simplex.c[1].w, ref m_simplex.c[0].w,
								ref m_simplex.c[2].w, ref m_simplex.c[0].w, out n );
							if( n.length2() > 0 )
							{
								appendvertice( m_simplex, ref n );
								if( EncloseOrigin() ) return ( true );
								removevertice( m_simplex );
								btVector3 tmp2;
								n.Invert( out tmp2 );
								appendvertice( m_simplex, ref tmp2 );
								if( EncloseOrigin() ) return ( true );
								removevertice( m_simplex );
							}
						}
						break;
					case 4:
						{
							if( btScalar.btFabs( det( ref m_simplex.c[0].w, ref m_simplex.c[3].w,
								ref m_simplex.c[1].w, ref m_simplex.c[3].w,
								ref m_simplex.c[2].w, ref m_simplex.c[3].w ) ) > 0 )
								return ( true );
						}
						break;
				}
				return ( false );
			}
			/* Internals	*/
			internal void getsupport( ref btVector3 d, sSV sv )
			{
				d.normalized( out sv.d );
				//sv.d = d / d.length();
				m_shape.Support( ref sv.d, out sv.w );
				//sv.w = ;
			}
			internal void removevertice( sSimplex simplex )
			{
				m_free[m_nfree++] = simplex.c[--simplex.rank];
			}
			internal void appendvertice( sSimplex simplex, ref btVector3 v )
			{
				simplex.p[simplex.rank] = 0;
				simplex.c[simplex.rank] = m_free[--m_nfree];
				getsupport( ref v, simplex.c[simplex.rank++] );
			}
			internal static double det( ref btVector3 a, ref btVector3 b, ref btVector3 c )
			{
				return ( a.y * b.z * c.x + a.z * b.x * c.y -
					a.x * b.z * c.y - a.y * b.x * c.z +
					a.x * b.y * c.z - a.z * b.y * c.x );
			}
			internal static double det( ref btVector3 a, ref btVector3 a2, ref btVector3 b
					, ref btVector3 b2, ref btVector3 c, ref btVector3 c2 )
			{
				return ( ( a.y - a2.y ) * ( b.z - b2.z ) * ( c.x - c2.x ) + ( a.z - a2.z ) * ( b.x - b2.x ) * ( c.y - c2.y ) -
					( a.x - a2.x ) * ( b.z - b2.z ) * ( c.y - c2.y ) - ( a.y - a2.y ) * ( b.x - b2.x ) * ( c.z - c2.z ) +
					( a.x - a2.x ) * ( b.y - b2.y ) * ( c.z - c2.z ) - ( a.z - a2.z ) * ( b.y - b2.y ) * ( c.x - c2.x ) );
			}
			static double projectorigin( ref btVector3 a,
				ref btVector3 b,
				double[] w, out uint m )
			{
				btVector3 d; b.Sub( ref a, out d );
				double l = d.length2();
				if( l > GJK_SIMPLEX2_EPS )
				{
					double t = ( l > 0 ? -btVector3.btDot( ref a, ref d ) / l : 0 );
					if( t >= 1 ) { w[0] = 0; w[1] = 1; m = 2; return ( b.length2() ); }
					else if( t <= 0 ) { w[0] = 1; w[1] = 0; m = 1; return ( a.length2() ); }
					else
					{
						w[0] = 1 - ( w[1] = t ); m = 3;
						btVector3 result;
						a.AddScale( ref d, t, out result );
						return ( result.length2() );
					}
				}
				m = 10;
				return ( -1 );
			}

			static double projectorigin( ref btVector3 a,
				ref btVector3 b,
				ref btVector3 c,
				double[] w, out uint m )
			{
				m = 100;
				int[] imd3 = { 1, 2, 0 };
				btVector3[] vt = { a, b, c };
				btVector3[] dl = new btVector3[3];
				a.Sub( ref b, out dl[0] );
				b.Sub( ref c, out dl[1] );
				c.Sub( ref a, out dl[2] );

				btVector3 n; btVector3.btCross( ref dl[0], ref dl[1], out n );
				double l = n.length2();
				if( l > GJK_SIMPLEX3_EPS )
				{
					double mindist = -1;
					double[] subw = { 0, 0 };
					uint subm = ( 0 );
					for( int i = 0; i < 3; ++i )
					{
						btVector3 tmp;
						btVector3.btCross( ref dl[i], ref n, out tmp );
						if( btVector3.btDot( ref vt[i], ref tmp ) > 0 )
						{
							int j = imd3[i];
							double subd = ( projectorigin( ref vt[i], ref vt[j], subw, out subm ) );
							if( ( mindist < 0 ) || ( subd < mindist ) )
							{
								mindist = subd;
								m = (uint)( ( ( subm & 1 ) != 0 ? 1 << i : 0 ) + ( ( subm & 2 ) != 0 ? 1 << j : 0 ) );
								w[i] = subw[0];
								w[j] = subw[1];
								w[imd3[j]] = 0;
							}
						}
					}
					if( mindist < 0 )
					{
						double d = btVector3.btDot( ref a, ref n );
						double s = btScalar.btSqrt( l );
						btVector3 p; n.Mult( ( d / l ), out p );
						mindist = p.length2();
						m = 7;
						btVector3 tmp, tmp2;
						b.Sub( ref p, out tmp );
						dl[1].cross( ref tmp, out tmp2 );
						w[0] = ( tmp2 ).length() / s;
						c.Sub( ref p, out tmp );
						dl[2].cross( ref tmp, out tmp2 );
						w[1] = ( tmp2 ).length() / s;
						w[2] = 1 - ( w[0] + w[1] );
					}
					return ( mindist );
				}
				return ( -1 );
			}
			static double projectorigin( ref btVector3 a,
				ref btVector3 b,
				ref btVector3 c,
				ref btVector3 d,
				double[] w, out uint m )
			{
				m = 100;
				uint[] imd3 = { 1, 2, 0 };
				btVector3[] vt = { a, b, c, d };
				btVector3[] dl = new btVector3[3];// { a - d, b - d, c - d };
				a.Sub( ref d, out dl[0] );
				b.Sub( ref d, out dl[1] );
				c.Sub( ref d, out dl[2] );
				double vl = det( ref dl[0], ref dl[1], ref dl[2] );
				btVector3 tmp;
				btVector3.btCross2Del( ref b, ref c, ref a, ref b, out tmp );
				bool ng = ( vl * btVector3.btDot( ref a, ref tmp ) ) <= 0;
				if( ng && ( btScalar.btFabs( vl ) > GJK_SIMPLEX4_EPS ) )
				{
					double mindist = -1;
					double[] subw = { 0, 0, 0 };
					uint subm = ( 0 );
					for( int i = 0; i < 3; ++i )
					{
						uint j = imd3[i];
						btVector3 tmp2;
						btVector3.btCross( ref dl[i], ref dl[j], out tmp2 );
						double s = vl * btVector3.btDot( ref d, ref tmp2 );
						if( s > 0 )
						{
							double subd = projectorigin( ref vt[i], ref vt[j], ref d, subw, out subm );
							if( ( mindist < 0 ) || ( subd < mindist ) )
							{
								mindist = subd;
								m = (uint)( ( ( ( subm & 1 ) != 0 ) ? 1 << i : 0 ) +
											( ( ( subm & 2 ) != 0 ) ? 1 << (int)j : 0 ) +
											( ( ( subm & 4 ) != 0 ) ? 8 : 0 ) );
								w[i] = subw[0];
								w[j] = subw[1];
								w[imd3[j]] = 0;
								w[3] = subw[2];
							}
						}
					}
					if( mindist < 0 )
					{
						mindist = 0;
						m = 15;
						w[0] = det( ref c, ref b, ref d ) / vl;
						w[1] = det( ref a, ref c, ref d ) / vl;
						w[2] = det( ref b, ref a, ref d ) / vl;
						w[3] = 1 - ( w[0] + w[1] + w[2] );
					}
					return ( mindist );
				}
				return ( -1 );
			}
		};

		// EPA
		internal class EPA
		{
			/* Types		*/
			internal class sFace
			{
				internal btVector3 n;
				internal double d;
				internal sSV[] c = new sSV[3];
				internal sFace[] f = new sFace[3];
				internal sFace[] l = new sFace[2];
				internal byte[] e = new byte[3];
				internal byte pass;
				internal sFace( sFace clone )
				{
					n = clone.n;
					d = clone.d;
					int i;
					for( i = 0; i < 3; i++ )
					{
						c[i] = clone.c[i];
						f[i] = clone.f[i];
						e[i] = clone.e[i];
					}
					for( i = 0; i < 2; i++ )
					{
						l[i] = clone.l[i];
					}
					pass = clone.pass;
				}
			}
			internal class sList
			{
				internal sFace root;
				internal uint count;
				internal sList() { root = null; count = 0; }
			};
			internal class sHorizon
			{
				internal sFace cf;
				internal sFace ff;
				internal uint nf;
				internal sHorizon() { cf = ( null ); ff = ( null ); nf = ( 0 ); }
			};
			internal struct eStatus
			{
				internal enum _
				{
					Valid,
					Touching,
					Degenerated,
					NonConvex,
					InvalidHull,
					OutOfFaces,
					OutOfVertices,
					AccuraryReached,
					FallBack,
					Failed
				};
			};
			/* Fields		*/
			eStatus._ m_status;
			internal GJK.sSimplex m_result;
			internal btVector3 m_normal;
			internal double m_depth;
			sSV[] m_sv_store = new sSV[EPA_MAX_VERTICES];
			sFace[] m_fc_store = new sFace[EPA_MAX_FACES];
			uint m_nextsv;
			sList m_hull;
			sList m_stock;
			/* Methods		*/
			internal EPA()
			{
				Initialize();
			}


			static void bind( sFace fa, uint ea, sFace fb, uint eb )
			{
				fa.e[ea] = (byte)eb; fa.f[ea] = fb;
				fb.e[eb] = (byte)ea; fb.f[eb] = fa;
			}
			static void append( sList list, sFace face )
			{
				face.l = null;
				face.l[1] = list.root;
				if( list.root != null ) list.root.l[0] = face;
				list.root = face;
				++list.count;
			}
			static void remove( sList list, sFace face )
			{
				if( face.l[1] != null ) face.l[1].l[0] = face.l[0];
				if( face.l[0] != null ) face.l[0].l[1] = face.l[1];
				if( face == list.root ) list.root = face.l[1];
				--list.count;
			}


			void Initialize()
			{
				m_status = eStatus._.Failed;
				m_normal = btVector3.Zero;
				m_depth = 0;
				m_nextsv = 0;
				for( int i = 0; i < EPA_MAX_FACES; ++i )
				{
					append( m_stock, m_fc_store[EPA_MAX_FACES - i - 1] );
				}
			}

			internal eStatus._ Evaluate( GJK gjk, ref btVector3 guess )
			{
				GJK.sSimplex simplex = gjk.m_simplex;
				if( ( simplex.rank > 1 ) && gjk.EncloseOrigin() )
				{

					/* Clean up				*/
					while( m_hull.root != null )
					{
						sFace f = m_hull.root;
						remove( m_hull, f );
						append( m_stock, f );
					}
					m_status = eStatus._.Valid;
					m_nextsv = 0;
					/* Orient simplex		*/
					if( GJK.det( ref simplex.c[0].w, ref simplex.c[3].w,
						ref simplex.c[1].w, ref simplex.c[3].w,
						ref simplex.c[2].w, ref simplex.c[3].w ) < 0 )
					{
						btScalar.btSwap( ref simplex.c[0], ref simplex.c[1] );
						btScalar.btSwap( ref simplex.p[0], ref simplex.p[1] );
					}
					/* Build initial hull	*/
					sFace[] tetra = {newface(simplex.c[0],simplex.c[1],simplex.c[2],true),
						newface(simplex.c[1],simplex.c[0],simplex.c[3],true),
						newface(simplex.c[2],simplex.c[1],simplex.c[3],true),
						newface(simplex.c[0],simplex.c[2],simplex.c[3],true)};
					if( m_hull.count == 4 )
					{
						sFace best = findbest();
						sFace outer = new sFace( best );
						uint pass = 0;
						uint iterations = 0;
						bind( tetra[0], 0, tetra[1], 0 );
						bind( tetra[0], 1, tetra[2], 0 );
						bind( tetra[0], 2, tetra[3], 0 );
						bind( tetra[1], 1, tetra[3], 2 );
						bind( tetra[1], 2, tetra[2], 1 );
						bind( tetra[2], 2, tetra[3], 1 );
						m_status = eStatus._.Valid;
						for( ; iterations < EPA_MAX_ITERATIONS; ++iterations )
						{
							if( m_nextsv < EPA_MAX_VERTICES )
							{
								sHorizon horizon = new sHorizon();
								sSV w = m_sv_store[m_nextsv++];
								bool valid = true;
								best.pass = (byte)( ++pass );
								gjk.getsupport( ref best.n, w );
								double wdist = btVector3.btDot( ref best.n, ref w.w ) - best.d;
								if( wdist > EPA_ACCURACY )
								{
									for( uint j = 0; ( j < 3 ) && valid; ++j )
									{
										valid &= expand( pass, w,
											best.f[j], best.e[j],
											horizon );
									}
									if( valid & ( horizon.nf >= 3 ) )
									{
										bind( horizon.cf, 1, horizon.ff, 2 );
										remove( m_hull, best );
										append( m_stock, best );
										best = findbest();
										outer = new sFace( best );
									}
									else { m_status = eStatus._.InvalidHull; break; }
								}
								else { m_status = eStatus._.AccuraryReached; break; }
							}
							else { m_status = eStatus._.OutOfVertices; break; }
						}
						btVector3 projection; outer.n.Mult( outer.d, out projection );
						m_normal = outer.n;
						m_depth = outer.d;
						m_result.rank = 3;
						m_result.c[0] = outer.c[0];
						m_result.c[1] = outer.c[1];
						m_result.c[2] = outer.c[2];
						btVector3 tmp;
						btVector3.btCross2Del( ref outer.c[1].w, ref projection,
							ref outer.c[2].w, ref projection, out tmp );
						m_result.p[0] = tmp.length();
						btVector3.btCross2Del( ref outer.c[2].w, ref projection,
							ref outer.c[0].w, ref projection, out tmp );
						m_result.p[1] = tmp.length();
						btVector3.btCross2Del( ref outer.c[0].w, ref projection,
							ref outer.c[1].w, ref projection, out tmp );
						m_result.p[2] = tmp.length();
						double sum = m_result.p[0] + m_result.p[1] + m_result.p[2];
						m_result.p[0] /= sum;
						m_result.p[1] /= sum;
						m_result.p[2] /= sum;
						return ( m_status );
					}
				}
				/* Fallback		*/
				m_status = eStatus._.FallBack;
				guess.Invert( out m_normal );
				//m_normal = -guess;
				double nl = m_normal.length();
				if( nl > 0 )
					m_normal.Div( nl, out m_normal );
				else
					m_normal = btVector3.xAxis;
				m_depth = 0;
				m_result.rank = 1;
				m_result.c[0] = simplex.c[0];
				m_result.p[0] = 1;
				return ( m_status );
			}
			bool getedgedist( sFace face, sSV a, sSV b, double dist )
			{
				btVector3 ba; b.w.Sub( ref a.w, out ba );
				btVector3 n_ab; btVector3.btCross( ref ba, ref face.n, out n_ab ); // Outward facing edge normal direction, on triangle plane
				double a_dot_nab = btVector3.btDot( ref a.w, ref n_ab ); // Only care about the sign to determine inside/outside, so not normalization required

				if( a_dot_nab < 0 )
				{
					// Outside of edge a.b

					double ba_l2 = ba.length2();
					double a_dot_ba = btVector3.btDot( ref a.w, ref ba );
					double b_dot_ba = btVector3.btDot( ref b.w, ref ba );

					if( a_dot_ba > 0 )
					{
						// Pick distance vertex a
						dist = a.w.length();
					}
					else if( b_dot_ba < 0 )
					{
						// Pick distance vertex b
						dist = b.w.length();
					}
					else
					{
						// Pick distance to edge a.b
						double a_dot_b = btVector3.btDot( ref a.w, ref b.w );
						dist = btScalar.btSqrt( btScalar.btMax( ( a.w.length2() * b.w.length2() - a_dot_b * a_dot_b ) / ba_l2, (double)0 ) );
					}

					return true;
				}

				return false;
			}
			sFace newface( sSV a, sSV b, sSV c, bool forced )
			{
				if( m_stock.root != null )
				{
					sFace face = m_stock.root;
					remove( m_stock, face );
					append( m_hull, face );
					face.pass = 0;
					face.c[0] = a;
					face.c[1] = b;
					face.c[2] = c;
					btVector3.btCross2Del( ref b.w, ref a.w, ref c.w, ref a.w, out face.n );
					double l = face.n.length();
					bool v = l > EPA_ACCURACY;

					if( v )
					{
						if( !( getedgedist( face, a, b, face.d ) ||
							 getedgedist( face, b, c, face.d ) ||
							 getedgedist( face, c, a, face.d ) ) )
						{
							// Origin projects to the interior of the triangle
							// Use distance to triangle plane
							face.d = btVector3.btDot( ref a.w, ref face.n ) / l;
						}

						face.n.Div( l, out face.n );
						if( forced || ( face.d >= -EPA_PLANE_EPS ) )
						{
							return face;
						}
						else
							m_status = eStatus._.NonConvex;
					}
					else
						m_status = eStatus._.Degenerated;

					remove( m_hull, face );
					append( m_stock, face );
					return null;

				}
				m_status = m_stock.root != null ? eStatus._.OutOfVertices : eStatus._.OutOfFaces;
				return null;
			}
			sFace findbest()
			{
				sFace minf = m_hull.root;
				double mind = minf.d * minf.d;
				for( sFace f = minf.l[1]; f != null; f = f.l[1] )
				{
					double sqd = f.d * f.d;
					if( sqd < mind )
					{
						minf = f;
						mind = sqd;
					}
				}
				return ( minf );
			}
			bool expand( uint pass, sSV w, sFace f, uint e, sHorizon horizon )
			{
				int[] i1m3 = { 1, 2, 0 };
				int[] i2m3 = { 2, 0, 1 };
				if( f.pass != pass )
				{
					int e1 = i1m3[e];
					if( ( btVector3.btDot( ref f.n, ref w.w ) - f.d ) < -EPA_PLANE_EPS )
					{
						sFace nf = newface( f.c[e1], f.c[e], w, false );
						if( nf != null )
						{
							bind( nf, 0, f, e );
							if( horizon.cf != null ) bind( horizon.cf, 1, nf, 2 ); else horizon.ff = nf;
							horizon.cf = nf;
							++horizon.nf;
							return ( true );
						}
					}
					else
					{
						int e2 = i2m3[e];
						f.pass = (byte)pass;
						if( expand( pass, w, f.f[e1], f.e[e1], horizon ) &&
							expand( pass, w, f.f[e2], f.e[e2], horizon ) )
						{
							remove( m_hull, f );
							append( m_stock, f );
							return ( true );
						}
					}
				}
				return ( false );
			}

		};

		//
		static void Initialize( btConvexShape shape0, ref btTransform wtrs0,
			btConvexShape shape1, ref btTransform wtrs1,
			btGjkEpaSolver2.sResults results,
			tShape shape,
			bool withmargins )
		{
			/* Results		*/
			results.witness0 =
				results.witness1 = btVector3.Zero;
			results.status = btGjkEpaSolver2.sResults.eStatus.Separated;
			/* Shape		*/
			shape.m_shapes[0] = shape0;
			shape.m_shapes[1] = shape1;
			wtrs1.m_basis.transposeTimes( ref wtrs0.m_basis, out shape.m_toshape1 );
			wtrs0.inverseTimes( ref wtrs1, out shape.m_toshape0 );
			shape.EnableMargin( withmargins );
		}


		//
		// Api
		//


		//

		//
		public static bool Distance( btConvexShape shape0,
											  ref btTransform wtrs0,
											  btConvexShape shape1,
											  ref btTransform wtrs1,
											  ref btVector3 guess,
											  sResults results )
		{
			tShape shape = new tShape();
			Initialize( shape0, ref wtrs0, shape1, ref wtrs1, results, shape, false );
			GJK gjk = new GJK();
			GJK.eStatus._ gjk_status = gjk.Evaluate( shape, ref guess );
			if( gjk_status == GJK.eStatus._.Valid )
			{
				btVector3 w0 = btVector3.Zero;
				btVector3 w1 = btVector3.Zero;
				for( uint i = 0; i < gjk.m_simplex.rank; ++i )
				{
					double p = gjk.m_simplex.p[i];
					btVector3 tmp;
					shape.Support( ref gjk.m_simplex.c[i].d, 0, out tmp );
					w0.AddScale( ref tmp, p, out w0 );

					gjk.m_simplex.c[i].d.Invert( out tmp );
					shape.Support( ref tmp, 1, out tmp );
					w1.AddScale( ref tmp, p, out w1 );
				}
				wtrs0.Apply( ref w0, out results.witness0 );
				wtrs0.Apply( ref w1, out results.witness1 );
				w0.Sub( ref w1, out results.normal );
				results.distance = results.normal.length();
				results.normal.Div( ( results.distance > GJK_MIN_DISTANCE ) ? results.distance : 1, out results.normal );
				return ( true );
			}
			else
			{
				results.status = gjk_status == GJK.eStatus._.Inside
					? sResults.eStatus.Penetrating
					: sResults.eStatus.GJK_Failed;
				return ( false );
			}
		}

		//
		public static bool Penetration( btConvexShape shape0,
											 ref btTransform wtrs0,
											 btConvexShape shape1,
											 ref btTransform wtrs1,
											 ref btVector3 guess,
											 sResults results,
											 bool usemargins = false )
		{
			tShape shape = new tShape();
			Initialize( shape0, ref wtrs0, shape1, ref wtrs1, results, shape, usemargins );
			GJK gjk = new GJK();
			btVector3 tmp;
			guess.Invert( out tmp );
			GJK.eStatus._ gjk_status = gjk.Evaluate( shape, ref tmp );
			switch( gjk_status )
			{
				case GJK.eStatus._.Inside:
					{
						EPA epa = new EPA();
						EPA.eStatus._ epa_status = epa.Evaluate( gjk, ref tmp );
						if( epa_status != EPA.eStatus._.Failed )
						{
							btVector3 w0 = btVector3.Zero;
							for( uint i = 0; i < epa.m_result.rank; ++i )
							{
								shape.Support( ref epa.m_result.c[i].d, 0, out tmp );
								w0.AddScale( ref tmp, epa.m_result.p[i], out w0 );
							}
							results.status = sResults.eStatus.Penetrating;
							wtrs0.Apply( ref w0, out results.witness0 );
							w0.SubScale( ref epa.m_normal, epa.m_depth, out tmp );
							wtrs0.Apply( ref tmp, out results.witness1 );
							epa.m_normal.Invert( out results.normal );
							results.distance = -epa.m_depth;
							return ( true );
						}
						else results.status = sResults.eStatus.EPA_Failed;
					}
					break;
				case GJK.eStatus._.Failed:
					results.status = sResults.eStatus.GJK_Failed;
					break;
				default:
					break;
			}
			return ( false );
		}

		//
		public static double SignedDistance( ref btVector3 position,
													double margin,
													btConvexShape shape0,
													ref btTransform wtrs0,
													sResults results )
		{
			tShape shape = new tShape();
			btSphereShape shape1 = new btSphereShape( margin );
			btTransform wtrs1 = new btTransform( ref btQuaternion.Zero, ref position );
			Initialize( shape0, ref wtrs0, shape1, ref wtrs1, results, shape, false );
			GJK gjk = new GJK();
			GJK.eStatus._ gjk_status = gjk.Evaluate( shape, ref btVector3.One );
			if( gjk_status == GJK.eStatus._.Valid )
			{
				btVector3 w0 = btVector3.Zero;
				btVector3 w1 = btVector3.Zero;
				for( uint i = 0; i < gjk.m_simplex.rank; ++i )
				{
					double p = gjk.m_simplex.p[i];
					btVector3 tmp;
					shape.Support( ref gjk.m_simplex.c[i].d, 0, out tmp );
					w0.AddScale( ref tmp, p, out w0 );
					btVector3 tmp2;
					gjk.m_simplex.c[i].d.Invert( out tmp2 );
					shape.Support( ref tmp2, 1, out tmp );
					w1.AddScale( ref tmp, p, out w1 );
				}
				wtrs0.Apply( ref w0, out results.witness0 );
				wtrs0.Apply( ref w1, out results.witness1 );
				btVector3 delta; results.witness1.Sub( ref results.witness0, out delta );
				margin = shape0.getMarginNonVirtual() +
					shape1.getMarginNonVirtual();
				double length = delta.length();
				delta.Div( length, out results.normal );
				results.witness0.AddScale( ref results.normal, margin, out results.witness0 );
				return ( length - margin );
			}
			else
			{
				if( gjk_status == GJK.eStatus._.Inside )
				{
					if( Penetration( shape0, ref wtrs0, shape1, ref wtrs1, ref gjk.m_ray, results ) )
					{
						btVector3 delta; results.witness0.Sub(
							ref results.witness1, out delta );
						double length = delta.length();
						if( length >= btScalar.SIMD_EPSILON )
							delta.Div( length, out results.normal );
						return ( -length );
					}
				}
			}
			return ( btScalar.SIMD_INFINITY );
		}

		//
		public static bool SignedDistance( btConvexShape shape0,
												ref btTransform wtrs0,
												btConvexShape shape1,
												ref btTransform wtrs1,
												ref btVector3 guess,
												sResults results )
		{
			if( !Distance( shape0, ref wtrs0, shape1, ref wtrs1, ref guess, results ) )
				return ( Penetration( shape0, ref wtrs0, shape1, ref wtrs1, ref guess, results, false ) );
			else
				return ( true );
		}

	};

}

