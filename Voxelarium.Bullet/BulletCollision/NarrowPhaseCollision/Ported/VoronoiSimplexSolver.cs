#define CATCH_DEGENERATE_TETRAHEDRON
#define BT_USE_EQUAL_VERTEX_THRESHOLD
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

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
using System.Diagnostics;
using Bullet.LinearMath;

namespace Bullet.Collision.NarrowPhase
{


	///disable next define, or use defaultCollisionConfiguration.getSimplexSolver().setEqualVertexThreshold(0) to disable/configure

	[Flags]
	public enum btUsageBitfield
	{
		usedVertexA = 1,
		usedVertexB = 2,
		usedVertexC = 4,
		usedVertexD = 8,
			usedAll = 0xF
	}


	internal struct btSubSimplexClosestResult
	{

		internal btVector3 m_closestPointOnSimplex;
		//MASK for m_usedVertices
		//stores the simplex vertex-usage, using the MASK, 
		// if m_usedVertices & MASK then the related vertex is used
		internal btUsageBitfield m_usedVertices;
		internal double m_barycentricCoord0;
		internal double m_barycentricCoord1;
		internal double m_barycentricCoord2;
		internal double m_barycentricCoord3;

		internal bool m_degenerate;

		internal void reset()
		{
			m_degenerate = false;
			setBarycentricCoordinates();
			m_usedVertices = 0;//.reset();
		}
		internal bool isValid()
		{
			bool valid = ( m_barycentricCoord0 >= btScalar.BT_ZERO ) &&
				( m_barycentricCoord1 >= btScalar.BT_ZERO ) &&
				( m_barycentricCoord2 >= btScalar.BT_ZERO ) &&
				( m_barycentricCoord3 >= btScalar.BT_ZERO );


			return valid;
		}
		internal void setBarycentricCoordinates( double a = btScalar.BT_ZERO, double b = btScalar.BT_ZERO, double c = btScalar.BT_ZERO, double d = btScalar.BT_ZERO )
		{
			m_barycentricCoord0 = a;
			m_barycentricCoord1 = b;
			m_barycentricCoord2 = c;
			m_barycentricCoord3 = d;
		}

	};

	/// btVoronoiSimplexSolver is an implementation of the closest point distance algorithm from a 1-4 points simplex to the origin.
	/// Can be used with GJK, as an alternative to Johnson distance algorithm.
	internal class btVoronoiSimplexSolver : btSimplexSolverInterface, IDisposable
	{
		const int VORONOI_SIMPLEX_MAX_VERTS = 5;
		const double VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD = 0.0001;

		public int m_numVertices;

		public btVector3[] m_simplexVectorW = new btVector3[VORONOI_SIMPLEX_MAX_VERTS];
		public btVector3[] m_simplexPointsP = new btVector3[VORONOI_SIMPLEX_MAX_VERTS];
		public btVector3[] m_simplexPointsQ = new btVector3[VORONOI_SIMPLEX_MAX_VERTS];



		public btVector3 m_cachedP1;
		public btVector3 m_cachedP2;
		public btVector3 m_cachedV;
		public btVector3 m_lastW;

		public double m_equalVertexThreshold;
		public bool m_cachedValidClosest;


		public btSubSimplexClosestResult m_cachedBC = new btSubSimplexClosestResult();

		public bool m_needsUpdate;

		public void Dispose()
		{
			BulletGlobals.VoronoiSimplexSolverPool.Free( this );
		}

		public int numVertices()
		{
			return m_numVertices;
		}

		public btVoronoiSimplexSolver()
		{
			m_equalVertexThreshold = ( VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD );
		}


		public void setEqualVertexThreshold( double threshold )
		{
			m_equalVertexThreshold = threshold;
		}

		public double getEqualVertexThreshold()
		{
			return m_equalVertexThreshold;
		}


		public bool fullSimplex()
		{
			return ( m_numVertices == 4 );
		}


		public void removeVertex( int index )
		{

			Debug.Assert( m_numVertices > 0 );
			m_numVertices--;
			m_simplexVectorW[index] = m_simplexVectorW[m_numVertices];
			m_simplexPointsP[index] = m_simplexPointsP[m_numVertices];
			m_simplexPointsQ[index] = m_simplexPointsQ[m_numVertices];
		}

		public void reduceVertices( btUsageBitfield usedVerts )
		{
			if( ( m_numVertices >= 4 ) && ( usedVerts & btUsageBitfield.usedVertexD ) == 0 )
				removeVertex( 3 );

			if( ( m_numVertices >= 3 ) && ( usedVerts & btUsageBitfield.usedVertexC ) == 0 )
				removeVertex( 2 );

			if( ( m_numVertices >= 2 ) && ( usedVerts & btUsageBitfield.usedVertexB ) == 0 )
				removeVertex( 1 );

			if( ( m_numVertices >= 1 ) && ( usedVerts & btUsageBitfield.usedVertexA ) == 0 )
				removeVertex( 0 );

		}



		//clear the simplex, remove all the vertices
		public void reset()
		{
			m_cachedValidClosest = false;
			m_numVertices = 0;
			m_needsUpdate = true;
			m_lastW = new btVector3( (double)( btScalar.BT_LARGE_FLOAT ), (double)( btScalar.BT_LARGE_FLOAT ), (double)( btScalar.BT_LARGE_FLOAT ) );
			m_cachedBC.reset();
		}



		//add a vertex
		public void addVertex( ref btVector3 w, ref btVector3 p, ref btVector3 q )
		{
			m_lastW = w;
			m_needsUpdate = true;

			m_simplexVectorW[m_numVertices] = w;
			m_simplexPointsP[m_numVertices] = p;
			m_simplexPointsQ[m_numVertices] = q;

			m_numVertices++;
		}

		public bool updateClosestVectorAndPoints()
		{

			if( m_needsUpdate )
			{
				m_cachedBC.reset();

				m_needsUpdate = false;

				switch( m_numVertices )
				{
					case 0:
						m_cachedValidClosest = false;
						break;
					case 1:
						{
							m_cachedP1 = m_simplexPointsP[0];
							m_cachedP2 = m_simplexPointsQ[0];
							m_cachedP1.Sub( ref m_cachedP2, out m_cachedV ); //== m_simplexVectorW[0]
							m_cachedBC.reset();
							m_cachedBC.setBarycentricCoordinates( btScalar.BT_ONE, btScalar.BT_ZERO, btScalar.BT_ZERO, btScalar.BT_ZERO );
							m_cachedValidClosest = m_cachedBC.isValid();
							break;
						};
					case 2:
						{
							//closest point origin from line segment
							//btVector3 from = ;
							//btVector3 to = m_simplexVectorW[1];
							btVector3 nearest;

							btVector3 p = new btVector3( btScalar.BT_ZERO, btScalar.BT_ZERO, btScalar.BT_ZERO );
							btVector3 diff; p.Sub( ref m_simplexVectorW[0], out diff );
							btVector3 v; m_simplexVectorW[1].Sub( ref m_simplexVectorW[0], out v );
							double t = v.dot( ref diff );
							btVector3 tmp;

							if( t > 0 )
							{
								double dotVV = v.dot( ref v );
								if( t < dotVV )
								{
									t /= dotVV;
									v.Mult( t, out tmp );
									diff.Sub( ref tmp, out diff );// -= t * v;
									m_cachedBC.m_usedVertices |= btUsageBitfield.usedVertexA;
									m_cachedBC.m_usedVertices |= btUsageBitfield.usedVertexB;
								}
								else
								{
									t = 1;
									diff.Sub( ref v, out diff );
									//reduce to 1 point
									m_cachedBC.m_usedVertices |= btUsageBitfield.usedVertexB;
								}
							}
							else
							{
								t = 0;
								//reduce to 1 point
								m_cachedBC.m_usedVertices |= btUsageBitfield.usedVertexA;
							}
							m_cachedBC.setBarycentricCoordinates( 1 - t, t );

							v.Mult( t, out tmp );
							m_simplexVectorW[0].Add( ref tmp, out nearest );

							m_simplexPointsP[1].Sub( ref m_simplexPointsP[0], out tmp );
							tmp.Mult( t, out tmp );
							m_simplexPointsP[0].Add( ref tmp, out m_cachedP1 );

							m_simplexPointsQ[1].Sub( ref tmp, out m_simplexPointsQ[0] );
							tmp.Mult( t, out tmp );
							m_simplexPointsQ[0].Add( ref tmp, out m_cachedP2 );

							m_cachedP1.Sub( ref m_cachedP2, out m_cachedV );

							reduceVertices( m_cachedBC.m_usedVertices );

							m_cachedValidClosest = m_cachedBC.isValid();
							break;
						}
					case 3:
						{
							//closest point origin from triangle 
							btVector3 p = btVector3.Zero;

							closestPtPointTriangle( ref p, ref m_simplexVectorW[0], ref m_simplexVectorW[1], ref m_simplexVectorW[2], m_cachedBC );
							btVector3 tmp, tmp2;
							m_simplexPointsP[0].Mult( m_cachedBC.m_barycentricCoord0, out tmp );
							m_simplexPointsP[1].Mult( m_cachedBC.m_barycentricCoord1, out tmp2 );
							tmp.Add( ref tmp2, out tmp );
							m_simplexPointsP[2].Mult( m_cachedBC.m_barycentricCoord2, out tmp2 );
							tmp.Add( ref tmp2, out m_cachedP1 );

							//m_cachedP1 = m_simplexPointsP[0] * m_cachedBC.m_barycentricCoord0 +
							//	m_simplexPointsP[1] * m_cachedBC.m_barycentricCoord1 +
							//	m_simplexPointsP[2] * m_cachedBC.m_barycentricCoord2;

							m_simplexPointsQ[0].Mult( m_cachedBC.m_barycentricCoord0, out tmp );
							m_simplexPointsQ[1].Mult( m_cachedBC.m_barycentricCoord1, out tmp2 );
							tmp.Add( ref tmp2, out tmp );
							m_simplexPointsQ[2].Mult( m_cachedBC.m_barycentricCoord2, out tmp2 );
							tmp.Add( ref tmp2, out m_cachedP2 );
							//m_cachedP2 = m_simplexPointsQ[0] * m_cachedBC.m_barycentricCoord0 +
							//	m_simplexPointsQ[1] * m_cachedBC.m_barycentricCoord1 +
							//	m_simplexPointsQ[2] * m_cachedBC.m_barycentricCoord2;

							m_cachedP1.Sub( ref m_cachedP2, out m_cachedV );
							//m_cachedV = m_cachedP1 - m_cachedP2;

							reduceVertices( m_cachedBC.m_usedVertices );
							m_cachedValidClosest = m_cachedBC.isValid();

							break;
						}
					case 4:
						{
							btVector3 p = btVector3.Zero;

							bool hasSeperation = closestPtPointTetrahedron( ref p, ref m_simplexVectorW[0], ref m_simplexVectorW[1], ref m_simplexVectorW[2], ref m_simplexVectorW[3], m_cachedBC );

							if( hasSeperation )
							{
								btVector3 tmp, tmp2;
								m_simplexPointsP[0].Mult( m_cachedBC.m_barycentricCoord0, out tmp );
								m_simplexPointsP[1].Mult( m_cachedBC.m_barycentricCoord1, out tmp2 );
								tmp.Add( ref tmp2, out tmp );
								m_simplexPointsP[2].Mult( m_cachedBC.m_barycentricCoord2, out tmp2 );
								tmp.Add( ref tmp2, out tmp );
								m_simplexPointsP[3].Mult( m_cachedBC.m_barycentricCoord3, out tmp2 );
								tmp.Add( ref tmp2, out m_cachedP1 );
								//m_cachedP1 = m_simplexPointsP[0] * m_cachedBC.m_barycentricCoord0 +
								//	m_simplexPointsP[1] * m_cachedBC.m_barycentricCoord1 +
								//	m_simplexPointsP[2] * m_cachedBC.m_barycentricCoord2 +
								//	m_simplexPointsP[3] * m_cachedBC.m_barycentricCoord3;

								m_simplexPointsQ[0].Mult( m_cachedBC.m_barycentricCoord0, out tmp );
								m_simplexPointsQ[1].Mult( m_cachedBC.m_barycentricCoord1, out tmp2 );
								tmp.Add( ref tmp2, out tmp );
								m_simplexPointsQ[2].Mult( m_cachedBC.m_barycentricCoord2, out tmp2 );
								tmp.Add( ref tmp2, out tmp );
								m_simplexPointsQ[3].Mult( m_cachedBC.m_barycentricCoord3, out tmp2 );
								tmp.Add( ref tmp2, out m_cachedP2 );
								//m_cachedP2 = m_simplexPointsQ[0] * m_cachedBC.m_barycentricCoord0 +
								//	m_simplexPointsQ[1] * m_cachedBC.m_barycentricCoord1 +
								//	m_simplexPointsQ[2] * m_cachedBC.m_barycentricCoord2 +
								//	m_simplexPointsQ[3] * m_cachedBC.m_barycentricCoord3;

								m_cachedP1.Sub( ref m_cachedP2, out m_cachedV );
								//m_cachedV = m_cachedP1 - m_cachedP2;
								reduceVertices( m_cachedBC.m_usedVertices );
							}
							else
							{
								//					Console.WriteLine("sub distance got penetration\n");

								if( m_cachedBC.m_degenerate )
								{
									m_cachedValidClosest = false;
								}
								else
								{
									m_cachedValidClosest = true;
									//degenerate case == false, penetration = true + zero
									m_cachedV.setValue( btScalar.BT_ZERO, btScalar.BT_ZERO, btScalar.BT_ZERO );
								}
								break;
							}

							m_cachedValidClosest = m_cachedBC.isValid();

							//closest point origin from tetrahedron
							break;
						}
					default:
						{
							m_cachedValidClosest = false;
						}
						break;
				};
			}

			return m_cachedValidClosest;

		}

		//return/calculate the closest vertex
		public bool closest( out btVector3 v )
		{
			bool succes = updateClosestVectorAndPoints();
			v = m_cachedV;
			return succes;
		}



		public double maxVertex()
		{
			int i, numverts = m_numVertices;
			double maxV = btScalar.BT_ZERO;
			for( i = 0; i < numverts; i++ )
			{
				double curLen2 = m_simplexVectorW[i].length2();
				if( maxV < curLen2 )
					maxV = curLen2;
			}
			return maxV;
		}



		//return the current simplex
		public int getSimplex( btVector3[] pBuf, btVector3[] qBuf, btVector3[] yBuf )
		{
			int i;
			for( i = 0; i < m_numVertices; i++ )
			{
				yBuf[i] = m_simplexVectorW[i];
				pBuf[i] = m_simplexPointsP[i];
				qBuf[i] = m_simplexPointsQ[i];
			}
			return m_numVertices;
		}




		public bool inSimplex( ref btVector3 w )
		{
			bool found = false;
			int i, numverts = m_numVertices;
			//double maxV = btScalar.BT_ZERO;

			//w is in the current (reduced) simplex
			for( i = 0; i < numverts; i++ )
			{
#if BT_USE_EQUAL_VERTEX_THRESHOLD
				if( m_simplexVectorW[i].distance2( ref w ) <= m_equalVertexThreshold )
#else
		if (m_simplexVectorW[i] == w)
#endif
					found = true;
			}

			//check in case lastW is already removed
			if( w.Equals( ref m_lastW ) )
				return true;

			return found;
		}

		public void backup_closest( ref btVector3 v )
		{
			v = m_cachedV;
		}


		public bool emptySimplex()
		{
			return ( m_numVertices == 0 );

		}

		public void compute_points( out btVector3 p1, out btVector3 p2 )
		{
			updateClosestVectorAndPoints();
			p1 = m_cachedP1;
			p2 = m_cachedP2;

		}




		public bool closestPtPointTriangle( ref btVector3 p, ref btVector3 a, ref btVector3 b, ref btVector3 c, btSubSimplexClosestResult result )
		{
			result.m_usedVertices = 0;//.reset();

			// Check if P in vertex region outside A
			btVector3 ab; b.Sub( ref a, out ab );
			btVector3 ac; c.Sub( ref a, out ac );
			btVector3 ap; p.Sub( ref a, out ap );
			double d1 = ab.dot( ref ap );
			double d2 = ac.dot( ref ap );
			if( d1 <= (double)( 0.0 ) && d2 <= (double)( 0.0 ) )
			{
				result.m_closestPointOnSimplex = a;
				result.m_usedVertices |= btUsageBitfield.usedVertexA;
				result.setBarycentricCoordinates( 1, 0, 0 );
				return true;// a; // barycentric coordinates (1,0,0)
			}

			// Check if P in vertex region outside B
			btVector3 bp; p.Sub( ref b, out bp );
			double d3 = ab.dot( ref bp );
			double d4 = ac.dot( ref bp );
			if( d3 >= (double)( 0.0 ) && d4 <= d3 )
			{
				result.m_closestPointOnSimplex = b;
				result.m_usedVertices |= btUsageBitfield.usedVertexB;
				result.setBarycentricCoordinates( 0, 1, 0 );

				return true; // b; // barycentric coordinates (0,1,0)
			}
			// Check if P in edge region of AB, if so return projection of P onto AB
			double vc = d1 * d4 - d3 * d2;
			btVector3 tmp;
			if( vc <= (double)( 0.0 ) && d1 >= (double)( 0.0 ) && d3 <= (double)( 0.0 ) )
			{
				double v = d1 / ( d1 - d3 );
				ab.Mult( v, out tmp );
				a.Add( ref tmp, out result.m_closestPointOnSimplex );
				result.m_usedVertices |= btUsageBitfield.usedVertexA;
				result.m_usedVertices |= btUsageBitfield.usedVertexB;
				result.setBarycentricCoordinates( 1 - v, v, 0 );
				return true;
				//return a + v * ab; // barycentric coordinates (1-v,v,0)
			}

			// Check if P in vertex region outside C
			btVector3 cp; p.Sub( ref c, out cp );
			double d5 = ab.dot( ref cp );
			double d6 = ac.dot( ref cp );
			if( d6 >= (double)( 0.0 ) && d5 <= d6 )
			{
				result.m_closestPointOnSimplex = c;
				result.m_usedVertices |= btUsageBitfield.usedVertexC;
				result.setBarycentricCoordinates( 0, 0, 1 );
				return true;//c; // barycentric coordinates (0,0,1)
			}

			// Check if P in edge region of AC, if so return projection of P onto AC
			double vb = d5 * d2 - d1 * d6;
			if( vb <= (double)( 0.0 ) && d2 >= (double)( 0.0 ) && d6 <= (double)( 0.0 ) )
			{
				double w = d2 / ( d2 - d6 );
				ac.Mult( w, out tmp );
				a.Add( ref tmp, out result.m_closestPointOnSimplex );
				result.m_usedVertices |= btUsageBitfield.usedVertexA;
				result.m_usedVertices |= btUsageBitfield.usedVertexC;
				result.setBarycentricCoordinates( 1 - w, 0, w );
				return true;
				//return a + w * ac; // barycentric coordinates (1-w,0,w)
			}

			// Check if P in edge region of BC, if so return projection of P onto BC
			double va = d3 * d6 - d5 * d4;
			if( va <= (double)( 0.0 ) && ( d4 - d3 ) >= (double)( 0.0 ) && ( d5 - d6 ) >= (double)( 0.0 ) )
			{
				double w = ( d4 - d3 ) / ( ( d4 - d3 ) + ( d5 - d6 ) );
				btVector3 tmp2;
				c.Sub( ref b, out tmp2 );
				tmp2.Mult( w, out tmp );
				b.Add( ref tmp, out result.m_closestPointOnSimplex );
				//result.m_closestPointOnSimplex = b + w * ( c - b );
				result.m_usedVertices |= btUsageBitfield.usedVertexB;
				result.m_usedVertices |= btUsageBitfield.usedVertexC;
				result.setBarycentricCoordinates( 0, 1 - w, w );
				return true;
				// return b + w * (c - b); // barycentric coordinates (0,1-w,w)
			}

			// P inside face region. Compute Q through its barycentric coordinates (u,v,w)
			{
				double denom = (double)( 1.0 ) / ( va + vb + vc );
				double v = vb * denom;
				double w = vc * denom;
				ab.Mult( v, out ab );
				ac.Mult( w, out ac );
				a.Add( ref ab, out tmp );
				tmp.Add( ref ac, out result.m_closestPointOnSimplex );

				//result.m_closestPointOnSimplex = a + ab * v + ac * w;
				result.m_usedVertices |= btUsageBitfield.usedVertexA;
				result.m_usedVertices |= btUsageBitfield.usedVertexB;
				result.m_usedVertices |= btUsageBitfield.usedVertexC;
				result.setBarycentricCoordinates( 1 - v - w, v, w );
			}
			return true;
			//	return a + ab * v + ac * w; // = u*a + v*b + w*c, u = va * denom = (double)(1.0) - v - w

		}





		/// Test if point p and d lie on opposite sides of plane through abc
		public int pointOutsideOfPlane( ref btVector3 p, ref btVector3 a, ref btVector3 b, ref btVector3 c, ref btVector3 d )
		{
			btVector3 tmp, tmp2;
			b.Sub( ref a, out tmp );
			c.Sub( ref a, out tmp2 );
			btVector3 normal; ( tmp ).cross( ref tmp2, out normal );
			p.Sub( ref a, out tmp );
			double signp = ( tmp ).dot( ref normal ); // [AP AB AC]
			d.Sub( ref a, out tmp );
			double signd = ( tmp ).dot( ref normal ); // [AD AB AC]

#if CATCH_DEGENERATE_TETRAHEDRON
#if BT_USE_DOUBLE_PRECISION
if (signd * signd < ((double)(1e-8) * (double)(1e-8)))
	{
		return -1;
	}
#else
			if( signd * signd < ( (double)( 1e-4 ) * (double)( 1e-4 ) ) )
			{
				//		Console.WriteLine("affine dependent/degenerate\n");//
				return -1;
			}
#endif

#endif
			// Points on opposite sides if expression signs are opposite
			return ( signp * signd < btScalar.BT_ZERO ) ? 1 : 0;
		}


		public bool closestPtPointTetrahedron( ref btVector3 p, ref btVector3 a, ref btVector3 b, ref btVector3 c, ref btVector3 d, btSubSimplexClosestResult finalResult )
		{
			btSubSimplexClosestResult tempResult = new btSubSimplexClosestResult();

			// Start out assuming point inside all halfspaces, so closest to itself
			finalResult.m_closestPointOnSimplex = p;
			finalResult.m_usedVertices = 0;
			finalResult.m_usedVertices |= btUsageBitfield.usedVertexA;
			finalResult.m_usedVertices |= btUsageBitfield.usedVertexB;
			finalResult.m_usedVertices |= btUsageBitfield.usedVertexC;
			finalResult.m_usedVertices |= btUsageBitfield.usedVertexD;

			int pointOutsideABC = pointOutsideOfPlane( ref p, ref a, ref b, ref c, ref d );
			int pointOutsideACD = pointOutsideOfPlane( ref p, ref a, ref c, ref d, ref b );
			int pointOutsideADB = pointOutsideOfPlane( ref p, ref a, ref d, ref b, ref c );
			int pointOutsideBDC = pointOutsideOfPlane( ref p, ref b, ref d, ref c, ref a );

			if( pointOutsideABC < 0 || pointOutsideACD < 0 || pointOutsideADB < 0 || pointOutsideBDC < 0 )
			{
				finalResult.m_degenerate = true;
				return false;
			}

			if( pointOutsideABC == 0 && pointOutsideACD == 0 && pointOutsideADB == 0 && pointOutsideBDC == 0 )
			{
				return false;
			}


			double bestSqDist = btScalar.BT_MAX_FLOAT;
			// If point outside face abc then compute closest point on abc
			btVector3 tmp;

			if( pointOutsideABC != 0 )
			{
				closestPtPointTriangle( ref p, ref a, ref b, ref c, tempResult );
				//btVector3 q = tempResult.m_closestPointOnSimplex;
				tempResult.m_closestPointOnSimplex.Sub( ref p, out tmp );
				double sqDist = ( tmp ).dot( ref tmp );
				// Update best closest point if (squared) distance is less than current best
				if( sqDist < bestSqDist )
				{
					bestSqDist = sqDist;
					finalResult.m_closestPointOnSimplex = tempResult.m_closestPointOnSimplex;
					//convert result bitmask!
					finalResult.m_usedVertices = 0;
					finalResult.m_usedVertices = tempResult.m_usedVertices
							& ( btUsageBitfield.usedVertexA | btUsageBitfield.usedVertexB | btUsageBitfield.usedVertexC );
					finalResult.setBarycentricCoordinates(
							tempResult.m_barycentricCoord0,
							tempResult.m_barycentricCoord1,
							tempResult.m_barycentricCoord2,
							0
					);

				}
			}


			// Repeat test for face acd
			if( pointOutsideACD != 0 )
			{
				closestPtPointTriangle( ref p, ref a, ref c, ref d, tempResult );
				//btVector3 q = tempResult.m_closestPointOnSimplex;
				//convert result bitmask!

				tempResult.m_closestPointOnSimplex.Sub( ref p, out tmp );
				double sqDist = ( tmp ).dot( ref tmp );
				//double sqDist = ( q - p ).dot( q - p );
				if( sqDist < bestSqDist )
				{
					bestSqDist = sqDist;
					finalResult.m_closestPointOnSimplex = tempResult.m_closestPointOnSimplex;
					finalResult.m_usedVertices = 0;
					finalResult.m_usedVertices = tempResult.m_usedVertices & ( btUsageBitfield.usedVertexA );

					finalResult.m_usedVertices |= ((tempResult.m_usedVertices & btUsageBitfield.usedVertexB) != 0)? btUsageBitfield.usedVertexC : 0;
                    finalResult.m_usedVertices |= ((tempResult.m_usedVertices & btUsageBitfield.usedVertexC) != 0)? btUsageBitfield.usedVertexD : 0;
					finalResult.setBarycentricCoordinates(
							tempResult.m_barycentricCoord0,
							0,
							tempResult.m_barycentricCoord1,
							tempResult.m_barycentricCoord2
					);

				}
			}
			// Repeat test for face adb


			if( pointOutsideADB != 0 )
			{
				closestPtPointTriangle( ref p, ref a, ref d, ref b, tempResult );
				//btVector3 q = tempResult.m_closestPointOnSimplex;
				//convert result bitmask!

				tempResult.m_closestPointOnSimplex.Sub( ref p, out tmp );
				double sqDist = ( tmp ).dot( ref tmp );
				//double sqDist = ( q - p ).dot( q - p );
				if( sqDist < bestSqDist )
				{
					bestSqDist = sqDist;
					finalResult.m_closestPointOnSimplex = tempResult.m_closestPointOnSimplex;
					finalResult.m_usedVertices = 0;
					finalResult.m_usedVertices = tempResult.m_usedVertices & ( btUsageBitfield.usedVertexA );
					finalResult.m_usedVertices |= ( ( tempResult.m_usedVertices & btUsageBitfield.usedVertexC ) != 0 ) ? btUsageBitfield.usedVertexB : 0;
					finalResult.m_usedVertices |= ( ( tempResult.m_usedVertices & btUsageBitfield.usedVertexB ) != 0 ) ? btUsageBitfield.usedVertexD : 0;
					finalResult.setBarycentricCoordinates(
							tempResult.m_barycentricCoord0,
							tempResult.m_barycentricCoord2,
							0,
							tempResult.m_barycentricCoord3
					);

				}
			}
			// Repeat test for face bdc


			if( pointOutsideBDC != 0 )
			{
				closestPtPointTriangle( ref p, ref b, ref d, ref c, tempResult );
				//btVector3 q = tempResult.m_closestPointOnSimplex;
				//convert result bitmask!
				tempResult.m_closestPointOnSimplex.Sub( ref p, out tmp );
				double sqDist = ( tmp ).dot( ref tmp );
				//double sqDist = ( q - p ).dot( q - p );
				if( sqDist < bestSqDist )
				{
					bestSqDist = sqDist;
					finalResult.m_closestPointOnSimplex = tempResult.m_closestPointOnSimplex;
					finalResult.m_usedVertices = 0;
					//
					finalResult.m_usedVertices |= ( ( tempResult.m_usedVertices & btUsageBitfield.usedVertexA ) != 0 ) ? btUsageBitfield.usedVertexB : 0;
					finalResult.m_usedVertices |= ( ( tempResult.m_usedVertices & btUsageBitfield.usedVertexC ) != 0 ) ? btUsageBitfield.usedVertexC : 0;
					finalResult.m_usedVertices |= ( ( tempResult.m_usedVertices & btUsageBitfield.usedVertexB ) != 0 ) ? btUsageBitfield.usedVertexD : 0;

					finalResult.setBarycentricCoordinates(
							0,
							tempResult.m_barycentricCoord0,
							tempResult.m_barycentricCoord2,
							tempResult.m_barycentricCoord2
					);

				}
			}

			//help! we ended up full !

			if( ( finalResult.m_usedVertices & btUsageBitfield.usedAll ) == btUsageBitfield.usedAll )
			{
				return true;
			}

			return true;
		}


	};

}
