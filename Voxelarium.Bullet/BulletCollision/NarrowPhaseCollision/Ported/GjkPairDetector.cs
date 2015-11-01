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
using Bullet.Collision.BroadPhase;
using Bullet.Collision.Shapes;
using Bullet.LinearMath;

namespace Bullet.Collision.NarrowPhase
{


	/// btGjkPairDetector uses GJK to implement the btDiscreteCollisionDetectorInterface
	public class btGjkPairDetector : btDiscreteCollisionDetectorInterface
	{


		btVector3 m_cachedSeparatingAxis;
		btConvexPenetrationDepthSolver m_penetrationDepthSolver;
		btSimplexSolverInterface m_simplexSolver;
		btConvexShape m_minkowskiA;
		btConvexShape m_minkowskiB;
		BroadphaseNativeTypes m_shapeTypeA;
		BroadphaseNativeTypes m_shapeTypeB;
		double m_marginA;
		double m_marginB;

		bool m_ignoreMargin;
		double m_cachedSeparatingDistance;




		//some debugging to fix degeneracy problems
		public int m_lastUsedMethod;
		public int m_curIter;
		public int m_degenerateSimplex;
		public int m_catchDegeneracies;
		public int m_fixContactNormalDirection;


		public void setMinkowskiA( btConvexShape minkA )
		{
			m_minkowskiA = minkA;
		}

		public void setMinkowskiB( btConvexShape minkB )
		{
			m_minkowskiB = minkB;
		}
		public void setCachedSeperatingAxis( ref btVector3 seperatingAxis )
		{
			m_cachedSeparatingAxis = seperatingAxis;
		}

		public btVector3 getCachedSeparatingAxis()
		{
			return m_cachedSeparatingAxis;
		}
		public double getCachedSeparatingDistance()
		{
			return m_cachedSeparatingDistance;
		}

		internal void setPenetrationDepthSolver( btConvexPenetrationDepthSolver penetrationDepthSolver )
		{
			m_penetrationDepthSolver = penetrationDepthSolver;
		}

		///don't use setIgnoreMargin, it's for Bullet's internal use
		public void setIgnoreMargin( bool ignoreMargin )
		{
			m_ignoreMargin = ignoreMargin;
		}
		//must be above the machine epsilon
		const double REL_ERROR2 = (double)( 1.0e-6 );

		//temp globals, to improve GJK/EPA/penetration calculations
		public int gNumDeepPenetrationChecks = 0;
		public int gNumGjkChecks = 0;


		public btGjkPairDetector()
		{
		}

		internal void Initialize( btConvexShape objectA, btConvexShape objectB, btSimplexSolverInterface simplexSolver
			, btConvexPenetrationDepthSolver penetrationDepthSolver )
		{
			m_cachedSeparatingAxis = btVector3.yAxis;
			m_penetrationDepthSolver = ( penetrationDepthSolver );
			m_simplexSolver = ( simplexSolver );
			m_minkowskiA = ( objectA );
			m_minkowskiB = ( objectB );
			m_shapeTypeA = ( objectA.getShapeType() );
			m_shapeTypeB = ( objectB.getShapeType() );
			m_marginA = ( objectA.getMargin() );
			m_marginB = ( objectB.getMargin() );
			m_ignoreMargin = ( false );
			m_lastUsedMethod = ( -1 );
			m_catchDegeneracies = ( 1 );
			m_fixContactNormalDirection = ( 1 );
		}

		internal void Initialize( btConvexShape objectA, btConvexShape objectB
			, BroadphaseNativeTypes shapeTypeA, BroadphaseNativeTypes shapeTypeB, double marginA, double marginB
			, btSimplexSolverInterface simplexSolver, btConvexPenetrationDepthSolver penetrationDepthSolver )
		{
			m_cachedSeparatingAxis = btVector3.yAxis;
			m_penetrationDepthSolver = ( penetrationDepthSolver );
			m_simplexSolver = ( simplexSolver );
			m_minkowskiA = ( objectA );
			m_minkowskiB = ( objectB );
			m_shapeTypeA = ( shapeTypeA );
			m_shapeTypeB = ( shapeTypeB );
			m_marginA = ( marginA );
			m_marginB = ( marginB );
			m_ignoreMargin = ( false );
			m_lastUsedMethod = ( -1 );
			m_catchDegeneracies = ( 1 );
			m_fixContactNormalDirection = ( 1 );
		}

		internal override void getClosestPoints( btDiscreteCollisionDetectorInterface.ClosestPointInput input
											, btDiscreteCollisionDetectorInterface.Result output
											, btIDebugDraw debugDraw, bool swapResults = false )
		{
			//(void)swapResults;

			getClosestPointsNonVirtual( input, output, debugDraw );
		}

#if __SPU__
void btGjkPairDetector::getClosestPointsNonVirtual(string losestPointInput& input,Result& output,btIDebugDraw debugDraw)
#else
		internal  void getClosestPointsNonVirtual( btDiscreteCollisionDetectorInterface.ClosestPointInput input
			, btDiscreteCollisionDetectorInterface.Result output, btIDebugDraw debugDraw )
#endif
		{
			m_cachedSeparatingDistance = 0;

			double distance = btScalar.BT_ZERO;
			btVector3 normalInB = btVector3.Zero;

			btVector3 pointOnA, pointOnB = btVector3.Zero;
			btTransform localTransA = input.m_transformA;
			btTransform localTransB = input.m_transformB;
			btVector3 originA, originB;
			input.m_transformB.getOrigin( out originB );
			input.m_transformA.getOrigin( out originA );
			btVector3 positionOffset; originA.Add( ref originB, out positionOffset );
			positionOffset.Mult( (double)( 0.5 ), out positionOffset );

			originA.Sub( ref positionOffset, out originA );
			localTransA.setOrigin( ref originA );
			//localTransA.m_origin.Sub( ref positionOffset, out localTransA.m_origin );
			originB.Sub( ref positionOffset, out originB );
			localTransB.setOrigin( ref originB );
			//localTransB.m_origin.Sub( ref positionOffset, out localTransA.m_origin );

			bool check2d = m_minkowskiA.isConvex2d() && m_minkowskiB.isConvex2d();

			double marginA = m_marginA;
			double marginB = m_marginB;

			gNumGjkChecks++;

			//for CCD we don't use margins
			if( m_ignoreMargin )
			{
				marginA = btScalar.BT_ZERO;
				marginB = btScalar.BT_ZERO;
			}

			m_curIter = 0;
			int gGjkMaxIter = 1000;//this is to catch invalid input, perhaps check for #NaN?
			m_cachedSeparatingAxis.setValue( 0, 1, 0 );

			bool isValid = false;
			bool checkSimplex = false;
			bool checkPenetration = true;
			m_degenerateSimplex = 0;

			m_lastUsedMethod = -1;

			{
				double squaredDistance = btScalar.BT_LARGE_FLOAT;
				double delta = btScalar.BT_ZERO;

				double margin = marginA + marginB;



				m_simplexSolver.reset();

				for( ;;)
				//while (true)
				{

					btVector3 tmp;
					m_cachedSeparatingAxis.Invert( out tmp );

					btVector3 seperatingAxisInA; input.m_transformA.Basis.ApplyInverse( ref tmp, out seperatingAxisInA );
					btVector3 seperatingAxisInB; input.m_transformB.Basis.ApplyInverse( ref m_cachedSeparatingAxis, out seperatingAxisInB );


					btVector3 pInA; m_minkowskiA.localGetSupportVertexWithoutMarginNonVirtual( ref seperatingAxisInA, out pInA );
					btVector3 qInB; m_minkowskiB.localGetSupportVertexWithoutMarginNonVirtual( ref seperatingAxisInB, out qInB );

					btVector3 pWorld; input.m_transformA.Apply( ref pInA, out pWorld );
					btVector3 qWorld; input.m_transformB.Apply( ref qInB, out qWorld );


					if( check2d )
					{
						pWorld[2] = 0;
						qWorld[2] = 0;
					}

					btVector3 w; pWorld.Sub( ref qWorld, out w );
					delta = m_cachedSeparatingAxis.dot( ref w );

					// potential exit, they don't overlap
					if( ( delta > (double)( 0.0 ) ) && ( delta * delta > squaredDistance * input.m_maximumDistanceSquared ) )
					{
						m_degenerateSimplex = 10;
						checkSimplex = true;
						//checkPenetration = false;
						break;
					}

					//exit 0: the new point is already in the simplex, or we didn't come any closer
					if( m_simplexSolver.inSimplex( ref w ) )
					{
						m_degenerateSimplex = 1;
						checkSimplex = true;
						break;
					}
					// are we getting any closer ?
					double f0 = squaredDistance - delta;
					double f1 = squaredDistance * REL_ERROR2;

					if( f0 <= f1 )
					{
						if( f0 <= btScalar.BT_ZERO )
						{
							m_degenerateSimplex = 2;
						}
						else
						{
							m_degenerateSimplex = 11;
						}
						checkSimplex = true;
						break;
					}

					//add current vertex to simplex
					m_simplexSolver.addVertex( ref w, ref pWorld, ref qWorld );
					btVector3 newCachedSeparatingAxis;

					//calculate the closest point to the origin (update vector v)
					if( !m_simplexSolver.closest( out newCachedSeparatingAxis ) )
					{
						m_degenerateSimplex = 3;
						checkSimplex = true;
						break;
					}

					if( newCachedSeparatingAxis.length2() < REL_ERROR2 )
					{
						m_cachedSeparatingAxis = newCachedSeparatingAxis;
						m_degenerateSimplex = 6;
						checkSimplex = true;
						break;
					}

					double previousSquaredDistance = squaredDistance;
					squaredDistance = newCachedSeparatingAxis.length2();
#if asdfasdf
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
					if( previousSquaredDistance - squaredDistance <= btScalar.SIMD_EPSILON * previousSquaredDistance )
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
#if DEBUG
						Console.WriteLine( "btGjkPairDetector maxIter exceeded:{0}", m_curIter );
						Console.WriteLine( "sepAxis=({0},{1},{2}), squaredDistance = {3}, shapeTypeA={4},shapeTypeB={5}\n",
							  m_cachedSeparatingAxis.x,
							  m_cachedSeparatingAxis.y,
							  m_cachedSeparatingAxis.z,
							  squaredDistance,
							  m_minkowskiA.getShapeType(),
							  m_minkowskiB.getShapeType() );

#endif
						break;

					}


					bool check = ( !m_simplexSolver.fullSimplex() );
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
					m_simplexSolver.compute_points( out pointOnA, out pointOnB );
					normalInB = m_cachedSeparatingAxis;

					double lenSqr = m_cachedSeparatingAxis.length2();

					//valid normal
					if( lenSqr < 0.0001 )
					{
						m_degenerateSimplex = 5;
					}
					if( lenSqr > btScalar.SIMD_EPSILON * btScalar.SIMD_EPSILON )
					{
						double rlen = btScalar.BT_ONE / btScalar.btSqrt( lenSqr );
						normalInB.Mult( rlen, out normalInB );
						//normalInB *= rlen; //normalize

						double s = btScalar.btSqrt( squaredDistance );

						Debug.Assert( s > (double)( 0.0 ) );
						pointOnA.SubScale( ref m_cachedSeparatingAxis, ( marginA / s ), out pointOnA );
						pointOnB.AddScale( ref m_cachedSeparatingAxis, ( marginB / s ), out pointOnB );
						//pointOnA -= m_cachedSeparatingAxis * ( marginA / s );
						//pointOnB += m_cachedSeparatingAxis * ( marginB / s );
						distance = ( ( btScalar.BT_ONE / rlen ) - margin );
						isValid = true;

						m_lastUsedMethod = 1;
					}
					else
					{
						m_lastUsedMethod = 2;
					}
				}

				bool catchDegeneratePenetrationCase =
					( m_catchDegeneracies != 0 
					  && m_penetrationDepthSolver != null 
					  && m_degenerateSimplex != 0 
					  && ( ( distance + margin ) < 0.01 ) );

				//if (checkPenetration && !isValid)
				if( checkPenetration && ( !isValid || catchDegeneratePenetrationCase ) )
				{
					//penetration case

					//if there is no way to handle penetrations, bail out
					if( m_penetrationDepthSolver != null )
					{
						// Penetration depth case.
						btVector3 tmpPointOnA, tmpPointOnB;

						gNumDeepPenetrationChecks++;
						m_cachedSeparatingAxis.setZero();

						bool isValid2 = m_penetrationDepthSolver.calcPenDepth(
							m_simplexSolver,
							m_minkowskiA, m_minkowskiB,
							ref input.m_transformA, ref input.m_transformB,
							ref m_cachedSeparatingAxis, out tmpPointOnA, out tmpPointOnB,
							debugDraw
							);


						if( isValid2 )
						{
							btVector3 tmpNormalInB; tmpPointOnB.Sub( ref tmpPointOnA, out tmpNormalInB );
							double lenSqr = tmpNormalInB.length2();
							if( lenSqr <= ( btScalar.SIMD_EPSILON * btScalar.SIMD_EPSILON ) )
							{
								tmpNormalInB = m_cachedSeparatingAxis;
								lenSqr = m_cachedSeparatingAxis.length2();
							}

							if( lenSqr > ( btScalar.SIMD_EPSILON * btScalar.SIMD_EPSILON ) )
							{
								tmpNormalInB.Mult( btScalar.btSqrt( lenSqr ), out tmpNormalInB );
								btVector3 tmp;
								tmpPointOnA.Sub( ref tmpPointOnB, out tmp );
								double distance2 = -tmp.length();
								//only replace valid penetrations when the result is deeper (check)
								if( !isValid || ( distance2 < distance ) )
								{
									distance = distance2;
									pointOnA = tmpPointOnA;
									pointOnB = tmpPointOnB;
									normalInB = tmpNormalInB;

									isValid = true;
									m_lastUsedMethod = 3;
								}
								else
								{
									m_lastUsedMethod = 8;
								}
							}
							else
							{
								m_lastUsedMethod = 9;
							}
						}
						else

						{
							///this is another degenerate case, where the initial GJK calculation reports a degenerate case
							///EPA reports no penetration, and the second GJK (using the supporting vector without margin)
							///reports a valid positive distance. Use the results of the second GJK instead of failing.
							///thanks to Jacob.Langford for the reproduction case
							///http://code.google.com/p/bullet/issues/detail?id=250


							if( m_cachedSeparatingAxis.length2() > btScalar.BT_ZERO )
							{
								btVector3 tmp;
								tmpPointOnA.Sub( ref tmpPointOnB, out tmp );
								double distance2 = tmp.length() - margin;
								//only replace valid distances when the distance is less
								if( !isValid || ( distance2 < distance ) )
								{
									distance = distance2;
									pointOnA = tmpPointOnA;
									pointOnB = tmpPointOnB;
									pointOnA.SubScale( ref m_cachedSeparatingAxis, marginA, out pointOnA );
									pointOnB.AddScale( ref m_cachedSeparatingAxis, marginB, out pointOnA );
									//pointOnA -= m_cachedSeparatingAxis * marginA;
									//pointOnB += m_cachedSeparatingAxis * marginB;
									normalInB = m_cachedSeparatingAxis;
									normalInB.normalize();

									isValid = true;
									m_lastUsedMethod = 6;
								}
								else
								{
									m_lastUsedMethod = 5;
								}
							}
						}

					}

				}
			}



			if( isValid && ( ( distance < 0 ) || ( distance * distance < input.m_maximumDistanceSquared ) ) )
			{

				m_cachedSeparatingAxis = normalInB;
				m_cachedSeparatingDistance = distance;
				btVector3 tmp;
				pointOnB.Add( ref positionOffset, out tmp );
				output.addContactPoint(
					ref normalInB,
					ref tmp,
					distance );

			}


		}


	}
}


