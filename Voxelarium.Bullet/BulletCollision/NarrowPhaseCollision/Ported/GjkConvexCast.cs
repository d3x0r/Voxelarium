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

	///GjkConvexCast performs a raycast on a convex object using support mapping.
	public class btGjkConvexCast : btConvexCast
	{
#if BT_USE_DOUBLE_PRECISION
		const int MAX_ITERATIONS = 64;
#else
		const int MAX_ITERATIONS = 32;
#endif
		btSimplexSolverInterface m_simplexSolver;
		btConvexShape m_convexA;
		btConvexShape m_convexB;



		public btGjkConvexCast(btConvexShape convexA, btConvexShape convexB, btSimplexSolverInterface simplexSolver)
		{
			m_simplexSolver = ( simplexSolver );
			m_convexA = ( convexA );
			m_convexB = ( convexB );
        }

		/// cast a convex against another convex object
		public override bool calcTimeOfImpact(
							ref btTransform fromA,
							ref btTransform toA,
							ref btTransform fromB,
							ref btTransform toB,
							CastResult result)
		{


			m_simplexSolver.reset();

			/// compute linear velocity for this interval, to interpolate
			//assume no rotation/angular velocity, assert here?
			btVector3 linVelA, linVelB;
			toA.m_origin.Sub( ref fromA.m_origin, out linVelA  );
			toB.m_origin.Sub(ref fromB.m_origin, out linVelB );

			double radius = (double)( 0.001 );
			double lambda = btScalar.BT_ZERO;
			btVector3 v =  btVector3.xAxis;

			int maxIter = MAX_ITERATIONS;

			btVector3 n = btVector3.Zero;
			bool hasResult = false;
			btVector3 c;
			btVector3 r; linVelA.Sub( ref linVelB, out r  );

			double lastLambda = lambda;
			//double epsilon = (double)(0.001);

			int numIter = 0;
			//first solution, using GJK


			//	result.drawCoordSystem(sphereTr);

			btPointCollector pointCollector = new btPointCollector();


			btGjkPairDetector gjk = new btGjkPairDetector( m_convexA, m_convexB, m_simplexSolver, null);//m_penetrationDepthSolver);		
			btGjkPairDetector.ClosestPointInput input = new btDiscreteCollisionDetectorInterface.ClosestPointInput();

			//we don't use margins during CCD
			//	gjk.setIgnoreMargin(true);

			input.m_transformA = fromA;
			input.m_transformB = fromB;
			gjk.getClosestPoints( input, pointCollector, null );

			hasResult = pointCollector.m_hasResult;
			c = pointCollector.m_pointInWorld;

			if( hasResult )
			{
				double dist;
				dist = pointCollector.m_distance;
				n = pointCollector.m_normalOnBInWorld;



				//not close enough
				while( dist > radius )
				{
					numIter++;
					if( numIter > maxIter )
					{
						return false; //todo: report a failure
					}
					double dLambda = btScalar.BT_ZERO;

					double projectedLinearVelocity = r.dot( ref n );

					dLambda = dist / ( projectedLinearVelocity );

					lambda = lambda - dLambda;

					if( lambda > btScalar.BT_ONE )
						return false;

					if( lambda < btScalar.BT_ZERO )
						return false;

					//todo: next check with relative epsilon
					if( lambda <= lastLambda )
					{
						return false;
						//n.setValue(0,0,0);
						//break;
					}
					lastLambda = lambda;

					//interpolate to next lambda
					result.DebugDraw( lambda );
					btVector3 tmp;
					btVector3.setInterpolate3( ref fromA.m_origin, ref toA.m_origin, lambda, out tmp );
					input.m_transformA.setOrigin( ref tmp );
					btVector3.setInterpolate3( ref fromB.m_origin, ref toB.m_origin, lambda, out tmp );
					input.m_transformB.setOrigin( ref tmp );

					gjk.getClosestPoints( input, pointCollector, null );
					if( pointCollector.m_hasResult )
					{
						if( pointCollector.m_distance < btScalar.BT_ZERO )
						{
							result.m_fraction = lastLambda;
							n = pointCollector.m_normalOnBInWorld;
							result.m_normal = n;
							result.m_hitPoint = pointCollector.m_pointInWorld;
							return true;
						}
						c = pointCollector.m_pointInWorld;
						n = pointCollector.m_normalOnBInWorld;
						dist = pointCollector.m_distance;
					}
					else
					{
						//??
						return false;
					}

				}

				//is n normalized?
				//don't report time of impact for motion away from the contact normal (or causes minor penetration)
				if( n.dot( ref r ) >= -result.m_allowedPenetration )
					return false;

				result.m_fraction = lambda;
				result.m_normal = n;
				result.m_hitPoint = c;
				return true;
			}

			return false;


		}

	};

}
