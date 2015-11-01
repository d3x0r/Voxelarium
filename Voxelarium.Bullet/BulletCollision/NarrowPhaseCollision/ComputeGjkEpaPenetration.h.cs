/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2014 Erwin Coumans http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

using Bullet.LinearMath;

namespace Bullet.Collision.NarrowPhase
{

	static class btGjkEpaUtil
	{
		bool btGjkEpaCalcPenDepth<btConvexTemplate>( btConvexTemplate a, btConvexTemplate b,
						  btGjkCollisionDescription colDesc,
						  ref btVector3 v, out btVector3 wWitnessOnA, out btVector3 wWitnessOnB ) where btConvexTemplate : 
	{
		//(void)v;

		//	double				radialmargin(btScalar.BT_ZERO);

		btVector3 guessVector( b.getWorldTransform().getOrigin() - a.getWorldTransform().getOrigin() );//?? why not use the GJK input?

		btGjkEpaSolver3::sResults results;


		if( btGjkEpaSolver3_Penetration( a, b, guessVector, results ) )

		{
			//	debugDraw.drawLine(results.witnesses[1],results.witnesses[1]+results.normal,btVector3(255,0,0));
			//resultOut.addContactPoint(results.normal,results.witnesses[1],-results.depth);
			wWitnessOnA = results.witnesses[0];
			wWitnessOnB = results.witnesses[1];
			v = results.normal;
			return true;
		} else
		{
			if( btGjkEpaSolver3_Distance( a, b, guessVector, results ) )
			{
				wWitnessOnA = results.witnesses[0];
				wWitnessOnB = results.witnesses[1];
				v = results.normal;
				return false;
			}
		}
		return false;
	}

	//template<typename btConvexTemplate, typename btGjkDistanceTemplate>
int btComputeGjkEpaPenetration<btConvexTemplate,btGjkDistanceTemplate>( btConvexTemplate a, btConvexTemplate b, btGjkCollisionDescription& colDesc, btVoronoiSimplexSolver& simplexSolver, btGjkDistanceTemplate* distInfo )
	{

		bool m_catchDegeneracies = true;
		double m_cachedSeparatingDistance = 0;

		double distance = btScalar.BT_ZERO;
		btVector3 normalInB( btScalar.BT_ZERO, btScalar.BT_ZERO, btScalar.BT_ZERO);

		btVector3 pointOnA, pointOnB;
		btTransform localTransA = a.getWorldTransform();
		btTransform localTransB = b.getWorldTransform();

		double marginA = a.getMargin();
		double marginB = b.getMargin();

		int m_curIter = 0;
		int gGjkMaxIter = colDesc.m_maxGjkIterations;//this is to catch invalid input, perhaps check for #NaN?
		btVector3 m_cachedSeparatingAxis = colDesc.m_firstDir;

		bool isValid = false;
		bool checkSimplex = false;
		bool checkPenetration = true;
		int m_degenerateSimplex = 0;

		int m_lastUsedMethod = -1;

		{
			double squaredDistance = BT_LARGE_FLOAT;
			double delta = btScalar.BT_ZERO;

			double margin = marginA + marginB;



			simplexSolver.reset();

			for( ;;)
			//while (true)
			{

				btVector3 seperatingAxisInA = ( -m_cachedSeparatingAxis ) * localTransA.getBasis();
				btVector3 seperatingAxisInB = m_cachedSeparatingAxis * localTransB.getBasis();

				btVector3 pInA = a.getLocalSupportWithoutMargin( seperatingAxisInA );
				btVector3 qInB = b.getLocalSupportWithoutMargin( seperatingAxisInB );

				btVector3 pWorld = localTransA( pInA );
				btVector3 qWorld = localTransB( qInB );



				btVector3 w = pWorld - qWorld;
				delta = m_cachedSeparatingAxis.dot( w );

				// potential exit, they don't overlap
				if( ( delta > (double)( 0.0 ) ) && ( delta * delta > squaredDistance * colDesc.m_maximumDistanceSquared ) )
				{
					m_degenerateSimplex = 10;
					checkSimplex = true;
					//checkPenetration = false;
					break;
				}

				//exit 0: the new point is already in the simplex, or we didn't come any closer
				if( simplexSolver.inSimplex( w ) )
				{
					m_degenerateSimplex = 1;
					checkSimplex = true;
					break;
				}
				// are we getting any closer ?
				double f0 = squaredDistance - delta;
				double f1 = squaredDistance * colDesc.m_gjkRelError2;

				if( f0 <= f1 )
				{
					if( f0 <= btScalar.BT_ZERO )
					{
						m_degenerateSimplex = 2;
					} else
					{
						m_degenerateSimplex = 11;
					}
					checkSimplex = true;
					break;
				}

				//add current vertex to simplex
				simplexSolver.addVertex( w, pWorld, qWorld );
				btVector3 newCachedSeparatingAxis;

				//calculate the closest point to the origin (update vector v)
				if( !simplexSolver.closest( newCachedSeparatingAxis ) )
				{
					m_degenerateSimplex = 3;
					checkSimplex = true;
					break;
				}

				if( newCachedSeparatingAxis.length2() < colDesc.m_gjkRelError2 )
				{
					m_cachedSeparatingAxis = newCachedSeparatingAxis;
					m_degenerateSimplex = 6;
					checkSimplex = true;
					break;
				}

				double previousSquaredDistance = squaredDistance;
				squaredDistance = newCachedSeparatingAxis.length2();
#if 0
            ///warning: this termination condition leads to some problems in 2d test case see Bullet/Demos/Box2dDemo
            if (squaredDistance>previousSquaredDistance)
            {
                m_degenerateSimplex = 7;
                squaredDistance = previousSquaredDistance;
                checkSimplex = false;
                break;
            }
#endif //


				//redundant m_simplexSolver.compute_points(pointOnA, pointOnB);

				//are we getting any closer ?
				if( previousSquaredDistance - squaredDistance <= SIMD_EPSILON * previousSquaredDistance )
				{
					//				m_simplexSolver.backup_closest(m_cachedSeparatingAxis);
					checkSimplex = true;
					m_degenerateSimplex = 12;

					break;
				}

				m_cachedSeparatingAxis = newCachedSeparatingAxis;

				//degeneracy, this is typically due to invalid/uninitialized worldtransforms for a btCollisionObject
				if( m_curIter++ > gGjkMaxIter )
				{
#if defined(DEBUG) || defined (_DEBUG)
                
                Console.WriteLine("btGjkPairDetector maxIter exceeded:%i\n",m_curIter);
                Console.WriteLine("sepAxis=(%f,%f,%f), squaredDistance = %f\n",
                       m_cachedSeparatingAxis.x,
                       m_cachedSeparatingAxis.y,
                       m_cachedSeparatingAxis.z,
                       squaredDistance);
#endif

					break;

				}


				bool check = ( !simplexSolver.fullSimplex() );
				//bool check = (!m_simplexSolver.fullSimplex() && squaredDistance > SIMD_EPSILON * m_simplexSolver.maxVertex());

				if( !check )
				{
					//do we need this backup_closest here ?
					//				m_simplexSolver.backup_closest(m_cachedSeparatingAxis);
					m_degenerateSimplex = 13;
					break;
				}
			}

			if( checkSimplex )
			{
				simplexSolver.compute_points( pointOnA, pointOnB );
				normalInB = m_cachedSeparatingAxis;

				double lenSqr = m_cachedSeparatingAxis.length2();

				//valid normal
				if( lenSqr < 0.0001 )
				{
					m_degenerateSimplex = 5;
				}
				if( lenSqr > SIMD_EPSILON * SIMD_EPSILON )
				{
					double rlen = btScalar.BT_ONE / btSqrt( lenSqr );
					normalInB *= rlen; //normalize

					double s = btSqrt( squaredDistance );

					Debug.Assert( s > (double)( 0.0 ) );
					pointOnA -= m_cachedSeparatingAxis * ( marginA / s );
					pointOnB += m_cachedSeparatingAxis * ( marginB / s );
					distance = ( ( btScalar.BT_ONE / rlen ) - margin );
					isValid = true;

					m_lastUsedMethod = 1;
				} else
				{
					m_lastUsedMethod = 2;
				}
			}

			bool catchDegeneratePenetrationCase =
			( m_catchDegeneracies && m_degenerateSimplex && ( ( distance + margin ) < 0.01 ) );

			//if (checkPenetration && !isValid)
			if( checkPenetration && ( !isValid || catchDegeneratePenetrationCase ) )
			{
				//penetration case

				//if there is no way to handle penetrations, bail out

				// Penetration depth case.
				btVector3 tmpPointOnA, tmpPointOnB;

				m_cachedSeparatingAxis.setZero();

				bool isValid2 = btGjkEpaCalcPenDepth( a, b,
													 colDesc,
													 m_cachedSeparatingAxis, tmpPointOnA, tmpPointOnB );

				if( isValid2 )
				{
					btVector3 tmpNormalInB = tmpPointOnB - tmpPointOnA;
					double lenSqr = tmpNormalInB.length2();
					if( lenSqr <= ( SIMD_EPSILON * SIMD_EPSILON ) )
					{
						tmpNormalInB = m_cachedSeparatingAxis;
						lenSqr = m_cachedSeparatingAxis.length2();
					}

					if( lenSqr > ( SIMD_EPSILON * SIMD_EPSILON ) )
					{
						tmpNormalInB /= btSqrt( lenSqr );
						double distance2 = -( tmpPointOnA - tmpPointOnB ).length();
						//only replace valid penetrations when the result is deeper (check)
						if( !isValid || ( distance2 < distance ) )
						{
							distance = distance2;
							pointOnA = tmpPointOnA;
							pointOnB = tmpPointOnB;
							normalInB = tmpNormalInB;

							isValid = true;
							m_lastUsedMethod = 3;
						} else
						{
							m_lastUsedMethod = 8;
						}
					} else
					{
						m_lastUsedMethod = 9;
					}
				} else

				{
					///this is another degenerate case, where the initial GJK calculation reports a degenerate case
					///EPA reports no penetration, and the second GJK (using the supporting vector without margin)
					///reports a valid positive distance. Use the results of the second GJK instead of failing.
					///thanks to Jacob.Langford for the reproduction case
					///http://code.google.com/p/bullet/issues/detail?id=250


					if( m_cachedSeparatingAxis.length2() > btScalar.BT_ZERO )
					{
						double distance2 = ( tmpPointOnA - tmpPointOnB ).length() - margin;
						//only replace valid distances when the distance is less
						if( !isValid || ( distance2 < distance ) )
						{
							distance = distance2;
							pointOnA = tmpPointOnA;
							pointOnB = tmpPointOnB;
							pointOnA -= m_cachedSeparatingAxis * marginA;
							pointOnB += m_cachedSeparatingAxis * marginB;
							normalInB = m_cachedSeparatingAxis;
							normalInB.normalize();

							isValid = true;
							m_lastUsedMethod = 6;
						} else
						{
							m_lastUsedMethod = 5;
						}
					}
				}
			}
		}



		if( isValid && ( ( distance < 0 ) || ( distance * distance < colDesc.m_maximumDistanceSquared ) ) )
		{

			m_cachedSeparatingAxis = normalInB;
			m_cachedSeparatingDistance = distance;
			distInfo.m_distance = distance;
			distInfo.m_normalBtoA = normalInB;
			distInfo.m_pointOnB = pointOnB;
			distInfo.m_pointOnA = pointOnB + normalInB * distance;
			return 0;
		}
		return -m_lastUsedMethod;
	}



}
