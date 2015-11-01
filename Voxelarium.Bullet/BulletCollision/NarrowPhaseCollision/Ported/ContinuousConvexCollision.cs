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

namespace Bullet.Collision.NarrowPhase
{

	/// btContinuousConvexCollision implements angular and linear time of impact for convex objects.
	/// Based on Brian Mirtich's Conservative Advancement idea (PhD thesis).
	/// Algorithm operates in worldspace, in order to keep inbetween motion globally consistent.
	/// It uses GJK at the moment. Future improvement would use minkowski sum / supporting vertex, merging innerloops
	class btContinuousConvexCollision : btConvexCast
	{
		const int MAX_ITERATIONS = 64;

		btSimplexSolverInterface m_simplexSolver;
		btConvexPenetrationDepthSolver m_penetrationDepthSolver;
		btConvexShape m_convexA;
		//second object is either a convex or a plane (code sharing)
		btConvexShape m_convexB1;
		btStaticPlaneShape m_planeShape;

		public btContinuousConvexCollision() { }

		internal void Initialize( btConvexShape convexA, btStaticPlaneShape plane )
		{
			m_simplexSolver = ( null );
			m_penetrationDepthSolver = ( null );
			m_convexA = ( convexA ); m_convexB1 = ( null ); m_planeShape = ( plane );
		}

		internal void Initialize( btConvexShape convexA, btConvexShape convexB, btSimplexSolverInterface simplexSolver, btConvexPenetrationDepthSolver penetrationDepthSolver )
		{
			m_simplexSolver = ( simplexSolver );
			m_penetrationDepthSolver = ( penetrationDepthSolver );
			m_convexA = ( convexA ); m_convexB1 = ( convexB ); m_planeShape = null;
		}


		public btContinuousConvexCollision( btConvexShape convexA, btStaticPlaneShape plane )
		{
			m_simplexSolver = ( null );
			m_penetrationDepthSolver = ( null );
			m_convexA = ( convexA ); m_convexB1 = ( null ); m_planeShape = ( plane );
		}


		/// This maximum should not be necessary. It allows for untested/degenerate cases in production code.
		/// You don't want your game ever to lock-up.

		void computeClosestPoints( btITransform transA, btITransform transB, btPointCollector pointCollector)
		{
			if( m_convexB1 != null)
			{
				m_simplexSolver.reset();
				btGjkPairDetector gjk = BulletGlobals.GjkPairDetectorPool.Get();
				gjk.Initialize( m_convexA, m_convexB1, m_convexA.getShapeType(), m_convexB1.getShapeType(), m_convexA.getMargin(), m_convexB1.getMargin(), m_simplexSolver, m_penetrationDepthSolver);
				btGjkPairDetector.ClosestPointInput input = new btDiscreteCollisionDetectorInterface.ClosestPointInput();
				input.m_transformA = transA.T;
				input.m_transformB = transB.T;
				gjk.getClosestPoints( input, pointCollector, null );
				BulletGlobals.GjkPairDetectorPool.Free( gjk );
			}
			else
			{
				//convex versus plane
				btConvexShape convexShape = m_convexA;
				btStaticPlaneShape planeShape = m_planeShape;

				btVector3 planeNormal; planeShape.getPlaneNormal( out planeNormal );
				double planeConstant = planeShape.getPlaneConstant();

				//btTransform convexWorldTransform = transA;
				btTransform convexInPlaneTrans;
				btTransform tmpInv;
				transB.inverse( out tmpInv );
				tmpInv.Apply( transA, out convexInPlaneTrans );
				btTransform planeInConvex;
				convexInPlaneTrans.inverse( out tmpInv );
				tmpInv.Apply( transB, out planeInConvex );
				//planeInConvex = convexWorldTransform.inverse() * transB;

				btVector3 tmp;
				planeInConvex.getBasis().Mult( ref planeNormal, out tmp );
				tmp.Invert( out tmp );
                btVector3 vtx; convexShape.localGetSupportingVertex( ref tmp, out vtx );

				btVector3 vtxInPlane; convexInPlaneTrans.Apply( vtx, out vtxInPlane );
				double distance = ( planeNormal.dot( vtxInPlane ) - planeConstant );

				btVector3 vtxInPlaneProjected;// = vtxInPlane - distance * planeNormal;
				vtxInPlane.SubScale( ref planeNormal, distance, out vtxInPlaneProjected );
				btVector3 vtxInPlaneWorld; transB.Apply(ref  vtxInPlaneProjected, out vtxInPlaneWorld );
				btVector3 normalOnSurfaceB = transB.getBasis() * planeNormal;

				pointCollector.addContactPoint(
					ref normalOnSurfaceB,
					ref vtxInPlaneWorld,
					distance );
			}
		}

		internal override bool calcTimeOfImpact(
						btITransform fromA,
						btITransform toA,
						btITransform fromB,
						btITransform toB,
						CastResult result)
		{


			/// compute linear and angular velocity for this interval, to interpolate
			btVector3 linVelA, angVelA, linVelB, angVelB;
			btTransformUtil.calculateVelocity( fromA, toA, btScalar.BT_ONE, out linVelA, out angVelA );
			btTransformUtil.calculateVelocity( fromB, toB, btScalar.BT_ONE, out linVelB, out angVelB );


			double boundingRadiusA = m_convexA.getAngularMotionDisc();
			double boundingRadiusB = m_convexB1 != null ? m_convexB1.getAngularMotionDisc() : 0;

			double maxAngularProjectedVelocity = angVelA.length() * boundingRadiusA + angVelB.length() * boundingRadiusB;
			btVector3 relLinVel = ( linVelB - linVelA );

			double relLinVelocLength = btVector3.btDelLength( ref linVelB,ref linVelA );

			if( ( relLinVelocLength + maxAngularProjectedVelocity ) == 0 )
				return false;



			double lambda = btScalar.BT_ZERO;
			btVector3 v = btVector3.xAxis;

			int maxIter = MAX_ITERATIONS;

			btVector3 n = btVector3.Zero;
			bool hasResult = false;
			btVector3 c;

			double lastLambda = lambda;
			//double epsilon = (double)(0.001);

			int numIter = 0;
			//first solution, using GJK


			double radius = 0.001f;
			//	result.drawCoordSystem(sphereTr);

			btPointCollector pointCollector1 = new btPointCollector();

			{

				computeClosestPoints( fromA, fromB, pointCollector1 );

				hasResult = pointCollector1.m_hasResult;
				c = pointCollector1.m_pointInWorld;
			}

			if( hasResult )
			{
				double dist;
				dist = pointCollector1.m_distance + result.m_allowedPenetration;
				n = pointCollector1.m_normalOnBInWorld;
				double projectedLinearVelocity = relLinVel.dot( n );
				if( ( projectedLinearVelocity + maxAngularProjectedVelocity ) <= btScalar.SIMD_EPSILON )
					return false;

				//not close enough
				while( dist > radius )
				{
					if( result.m_debugDrawer != null )
					{
						result.m_debugDrawer.drawSphere( ref c, 0.2f, ref btVector3.One );
					}
					double dLambda = btScalar.BT_ZERO;

					projectedLinearVelocity = relLinVel.dot( n );


					//don't report time of impact for motion away from the contact normal (or causes minor penetration)
					if( ( projectedLinearVelocity + maxAngularProjectedVelocity ) <= btScalar.SIMD_EPSILON )
						return false;

					dLambda = dist / ( projectedLinearVelocity + maxAngularProjectedVelocity );



					lambda = lambda + dLambda;

					if( lambda > btScalar.BT_ONE )
						return false;

					if( lambda < btScalar.BT_ZERO )
						return false;


					//todo: next check with relative epsilon
					if( lambda <= lastLambda )
					{
						return false;
					}
					lastLambda = lambda;



					//interpolate to next lambda
					btTransform interpolatedTransA, interpolatedTransB, relativeTrans;

					btTransformUtil.integrateTransform( fromA, linVelA, angVelA, lambda, out interpolatedTransA );
					btTransformUtil.integrateTransform( fromB, linVelB, angVelB, lambda, out interpolatedTransB );
					interpolatedTransB.inverseTimes( ref interpolatedTransA, out relativeTrans );

					if( result.m_debugDrawer != null )
					{
						result.m_debugDrawer.drawSphere( ref interpolatedTransA.m_origin, 0.2f, ref btVector3.xAxis );
					}

					result.DebugDraw( lambda );

					btPointCollector pointCollector = new btPointCollector();
					computeClosestPoints( interpolatedTransA, interpolatedTransB, pointCollector );

					if( pointCollector.m_hasResult )
					{
						dist = pointCollector.m_distance + result.m_allowedPenetration;
						c = pointCollector.m_pointInWorld;
						n = pointCollector.m_normalOnBInWorld;
					}
					else
					{
						result.reportFailure( -1, numIter );
						return false;
					}

					numIter++;
					if( numIter > maxIter )
					{
						result.reportFailure( -2, numIter );
						return false;
					}
				}

				result.m_fraction = lambda;
				result.m_normal = n;
				result.m_hitPoint = c;
				return true;
			}

			return false;

		}


	};


}
