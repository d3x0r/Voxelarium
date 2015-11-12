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

using Bullet.Collision.Shapes;
using Bullet.LinearMath;
using System;

namespace Bullet.Collision.NarrowPhase
{

	/// btSubsimplexConvexCast implements Gino van den Bergens' paper
	///"Ray Casting against bteral Convex Objects with Application to Continuous Collision Detection"
	/// GJK based Ray Cast, optimized version
	/// Objects should not start in overlap, otherwise results are not defined.
	internal class btSubsimplexConvexCast : btConvexCast, IDisposable
	{

#if BT_USE_DOUBLE_PRECISION
		const int MAX_ITERATIONS =64;
#else
		const int MAX_ITERATIONS = 32;
#endif
		btSimplexSolverInterface m_simplexSolver;
		btConvexShape m_convexA;
		btConvexShape m_convexB;

		public void Dispose()
		{
			BulletGlobals.SubSimplexConvexCastPool.Free( this );
		}
		public btSubsimplexConvexCast()
		{
		}
		public void Initialize( btConvexShape convexA, btConvexShape convexB, btSimplexSolverInterface simplexSolver )
		{
			m_simplexSolver = ( simplexSolver );
			m_convexA = ( convexA );
			m_convexB = ( convexB );
		}


		///SimsimplexConvexCast calculateTimeOfImpact calculates the time of impact+normal for the linear cast (sweep) between two moving objects.
		///Precondition is that objects should not penetration/overlap at the start from the interval. Overlap can be tested using btGjkPairDetector.
		/// 
		///Typically the conservative advancement reaches solution in a few iterations, clip it to 32 for degenerate cases.
		///See discussion about this here http://continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=565
		internal override bool calcTimeOfImpact(
				ref btTransform fromA,
				ref btTransform toA,
				ref btTransform fromB,
				ref btTransform toB,
				CastResult result )
		{

			m_simplexSolver.reset();

			btVector3 linVelA, linVelB;
			toA.m_origin.Sub( ref fromA.m_origin, out linVelA );
			toB.m_origin.Sub( ref fromB.m_origin, out linVelB );

			double lambda = btScalar.BT_ZERO;

			btTransform interpolatedTransA; fromA.Get( out interpolatedTransA );
			btTransform interpolatedTransB; fromB.Get( out interpolatedTransB );

			///take relative motion
			btVector3 r; linVelA.Sub( ref linVelB, out r );
			btVector3 v;
			btVector3 tmp;
			btVector3 tmp2;

			r.Invert( out v );
			fromA.m_basis.ApplyInverse( ref v, out tmp );
			m_convexA.localGetSupportingVertex( ref tmp, out tmp2 );
			btVector3 supVertexA; fromA.Apply( ref tmp2, out supVertexA );
			//btVector3 supVertexA = fromA( m_convexA.localGetSupportingVertex( -r * fromA.getBasis() ) );

			fromB.m_basis.ApplyInverse( ref r, out tmp );
			m_convexB.localGetSupportingVertex( ref tmp, out tmp2 );
			btVector3 supVertexB; fromB.Apply( ref tmp2, out supVertexB );
			//btVector3 supVertexB = fromB( m_convexB.localGetSupportingVertex( r * fromB.getBasis() ) );
			supVertexA.Sub( ref supVertexB, out v );
			//v = supVertexA - supVertexB;
			int maxIter = MAX_ITERATIONS;

			btVector3 n = btVector3.Zero;

			//btVector3 c;


			double dist2 = v.length2();
			btVector3 w;
			double VdotR;

			while( ( dist2 > btScalar.SIMD_LARGE_EPSILON ) && ( maxIter-- != 0 ) )
			{
				//btVector3 tmp, tmp2;
				v.Invert( out tmp );
				interpolatedTransA.m_basis.ApplyInverse( ref tmp, out tmp2 );
				m_convexA.localGetSupportingVertex( ref tmp2, out tmp );
				interpolatedTransA.Apply( ref tmp, out supVertexA );
				//supVertexA = interpolatedTransA( m_convexA.localGetSupportingVertex( -v * interpolatedTransA.getBasis() ) );
				interpolatedTransB.m_basis.ApplyInverse( ref v, out tmp2 );
				m_convexB.localGetSupportingVertex( ref tmp2, out tmp );
				interpolatedTransB.Apply( ref tmp, out supVertexB );
				//supVertexB = interpolatedTransB( m_convexB.localGetSupportingVertex( v * interpolatedTransB.getBasis() ) );
				supVertexA.Sub( ref supVertexB, out w );
				//w = supVertexA - supVertexB;

				double VdotW = v.dot( ref w );

				if( lambda > (double)( 1.0 ) )
				{
					return false;
				}

				if( VdotW > btScalar.BT_ZERO )
				{
					VdotR = v.dot( ref r );

					if( VdotR >= -( btScalar.SIMD_EPSILON * btScalar.SIMD_EPSILON ) )
						return false;
					else
					{
						lambda = lambda - VdotW / VdotR;
						//interpolate to next lambda
						//	x = s + lambda * r;
						interpolatedTransA.m_origin.setInterpolate3( ref fromA.m_origin, ref toA.m_origin, lambda );
						interpolatedTransB.m_origin.setInterpolate3( ref fromB.m_origin, ref toB.m_origin, lambda );
						//m_simplexSolver.reset();
						//check next line
						supVertexA.Sub( ref supVertexB, out w );
						//w = supVertexA - supVertexB;

						n = v;

					}
				}
				///Just like regular GJK only add the vertex if it isn't already (close) to current vertex, it would lead to divisions by zero and NaN etc.
				if( !m_simplexSolver.inSimplex( ref w ) )
					m_simplexSolver.addVertex( ref w, ref supVertexA, ref supVertexB );

				if( m_simplexSolver.closest( out v ) )
				{
					dist2 = v.length2();

					//todo: check this normal for validity
					//n=v;
					//Console.WriteLine("V=%f , %f, %f\n",v,v[1],v[2]);
					//Console.WriteLine("DIST2=%f\n",dist2);
					//Console.WriteLine("numverts = %i\n",m_simplexSolver.numVertices());
				}
				else
				{
					dist2 = btScalar.BT_ZERO;
				}
			}

			//int numiter = MAX_ITERATIONS - maxIter;
			//	Console.WriteLine("number of iterations: %d", numiter);

			//don't report a time of impact when moving 'away' from the hitnormal


			result.m_fraction = lambda;
			if( n.length2() >= ( btScalar.SIMD_EPSILON * btScalar.SIMD_EPSILON ) )
				n.normalized( out result.m_normal );
			else
				result.m_normal = btVector3.Zero;

			//don't report time of impact for motion away from the contact normal (or causes minor penetration)
			if( result.m_normal.dot( ref r ) >= -result.m_allowedPenetration )
				return false;

			btVector3 hitA, hitB;
			m_simplexSolver.compute_points( out hitA, out hitB );
			result.m_hitPoint = hitB;
			return true;
		}
	};
}

