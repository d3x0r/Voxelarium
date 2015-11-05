//#define USE_SUBSIMPLEX_CONVEX_CAST
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

	internal abstract class btTriangleRaycastCallback : btTriangleCallback
	{

		//input
		public btVector3 m_from;
		public btVector3 m_to;

		//@BP Mod - allow backface filtering and unflipped normals
		public enum EFlags
		{
			kF_None = 0,
			kF_FilterBackfaces = 1 << 0,
			kF_KeepUnflippedNormal = 1 << 1,   // Prevents returned face normal getting flipped when a ray hits a back-facing triangle
											   ///SubSimplexConvexCastRaytest is the default, even if kF_None is set.
			kF_UseSubSimplexConvexCastRaytest = 1 << 2,   // Uses an approximate but faster ray versus convex intersection algorithm
			kF_UseGjkConvexCastRaytest = 1 << 3,
			kF_Terminator = -1
		};
		public EFlags m_flags;

		public double m_hitFraction;


		internal abstract double reportHit( ref btVector3 hitNormalLocal, double hitFraction, int partId, int triangleIndex );

		public btTriangleRaycastCallback( ref btVector3 from, ref btVector3 to, EFlags flags = EFlags.kF_None )
		{
			m_from = ( from );
			m_to = ( to );
			//@BP Mod
			m_flags = ( flags );
			m_hitFraction = ( btScalar.BT_ONE );
		}


		public void processTriangle( btVector3[] triangle, int partId, int triangleIndex )
		{
			btVector3 vert0 = triangle[0];
			btVector3 vert1 = triangle[1];
			btVector3 vert2 = triangle[2];

			btVector3 v10; vert1.Sub( ref vert0, out v10 );
			btVector3 v20; vert2.Sub( ref vert0, out v20 );

			btVector3 triangleNormal; v10.cross( ref v20, out triangleNormal );

			double dist = vert0.dot( ref triangleNormal );
			double dist_a = triangleNormal.dot( ref m_from );
			dist_a -= dist;
			double dist_b = triangleNormal.dot( ref m_to );
			dist_b -= dist;

			if( dist_a * dist_b >= (double)( 0.0 ) )
			{
				return; // same sign
			}

			if( ( ( m_flags & EFlags.kF_FilterBackfaces ) != 0 ) && ( dist_a <= (double)( 0.0 ) ) )
			{
				// Backface, skip check
				return;
			}


			double proj_length = dist_a - dist_b;
			double distance = ( dist_a ) / ( proj_length );
			// Now we have the intersection point on the plane, we'll see if it's inside the triangle
			// Add an epsilon as a tolerance for the raycast,
			// in case the ray hits exacly on the edge of the triangle.
			// It must be scaled for the triangle size.

			if( distance < m_hitFraction )
			{

				double edge_tolerance = triangleNormal.length2();
				edge_tolerance *= (double)( -0.0001 );
				btVector3 point; btVector3.setInterpolate3( out point, ref m_from, ref m_to, distance );
				{
					btVector3 v0p; vert0.Sub( ref point, out v0p );
					btVector3 v1p; vert1.Sub( ref point, out v1p );
					btVector3 cp0; v0p.cross( ref v1p, out cp0 );

					if( (double)( cp0.dot( ref triangleNormal ) ) >= edge_tolerance )
					{


						btVector3 v2p; vert2.Sub( ref point, out v2p );
						btVector3 cp1;
						v1p.cross( ref v2p, out cp1 );
						if( (double)( cp1.dot( ref triangleNormal ) ) >= edge_tolerance )
						{
							btVector3 cp2;
							v2p.cross( ref v0p, out cp2 );

							if( (double)( cp2.dot( ref triangleNormal ) ) >= edge_tolerance )
							{
								//@BP Mod
								// Triangle normal isn't normalized
								triangleNormal.normalize();

								//@BP Mod - Allow for unflipped normal when raycasting against backfaces
								if( ( ( m_flags & EFlags.kF_KeepUnflippedNormal ) == 0 ) && ( dist_a <= (double)( 0.0 ) ) )
								{
									btVector3 tmp;
									triangleNormal.Invert( out tmp );
									m_hitFraction = reportHit( ref tmp, distance, partId, triangleIndex );
								}
								else
								{
									m_hitFraction = reportHit( ref triangleNormal, distance, partId, triangleIndex );
								}
							}
						}
					}
				}
			}
		}


	};

	internal abstract class btTriangleConvexcastCallback : btTriangleCallback
	{
		public btConvexShape m_convexShape;
		public btTransform m_convexShapeFrom;
		public btTransform m_convexShapeTo;
		public btTransform m_triangleToWorld;
		public double m_hitFraction;
		public double m_triangleCollisionMargin;
		public double m_allowedPenetration;



		internal abstract double reportHit( ref btVector3 hitNormalLocal, ref btVector3 hitPointLocal, double hitFraction, int partId, int triangleIndex );

		public btTriangleConvexcastCallback() { }

		internal void Initialize( btConvexShape convexShape, ref btTransform convexShapeFrom, ref btTransform convexShapeTo, btITransform triangleToWorld, double triangleCollisionMargin )
		{
			m_convexShape = convexShape;
			m_convexShapeFrom = convexShapeFrom;
			m_convexShapeTo = convexShapeTo;
			m_triangleToWorld = triangleToWorld.T;
			m_hitFraction = 1.0f;
			m_triangleCollisionMargin = triangleCollisionMargin;
			m_allowedPenetration = 0;
		}

		public void processTriangle( btVector3[] triangle, int partId, int triangleIndex )
		{
			using( btTriangleShape triangleShape = BulletGlobals.TriangleShapePool.Get() )
			{
				triangleShape.Initialize( ref triangle[0], ref triangle[1], ref triangle[2] );
				triangleShape.setMargin( m_triangleCollisionMargin );

				btVoronoiSimplexSolver simplexSolver = new btVoronoiSimplexSolver();
				btGjkEpaPenetrationDepthSolver gjkEpaPenetrationSolver = new btGjkEpaPenetrationDepthSolver();

				//#define  USE_SUBSIMPLEX_CONVEX_CAST 1
				//if you reenable USE_SUBSIMPLEX_CONVEX_CAST see commented out code below
#if USE_SUBSIMPLEX_CONVEX_CAST
	btSubsimplexConvexCast convexCaster(m_convexShape, &triangleShape, &simplexSolver);
#else
				//btGjkConvexCast	convexCaster(m_convexShape,&triangleShape,&simplexSolver);
				btContinuousConvexCollision convexCaster = BulletGlobals.ContinuousConvexCollisionPool.Get();
				convexCaster.Initialize( m_convexShape, triangleShape, simplexSolver, gjkEpaPenetrationSolver );
#endif //#USE_SUBSIMPLEX_CONVEX_CAST

				btConvexCast.CastResult castResult = BulletGlobals.CastResultPool.Get();
				castResult.Initialize();
				castResult.m_fraction = btScalar.BT_ONE;
				castResult.m_allowedPenetration = m_allowedPenetration;
				if( convexCaster.calcTimeOfImpact( m_convexShapeFrom, m_convexShapeTo, m_triangleToWorld, m_triangleToWorld, castResult ) )
				{
					//add hit
					if( castResult.m_normal.length2() > (double)( 0.0001 ) )
					{
						if( castResult.m_fraction < m_hitFraction )
						{
							/* btContinuousConvexCast's normal is already in world space */
							/*
							#if USE_SUBSIMPLEX_CONVEX_CAST
											//rotate normal into worldspace
											castResult.m_normal = m_convexShapeFrom.getBasis() * castResult.m_normal;
							#endif //USE_SUBSIMPLEX_CONVEX_CAST
							*/
							castResult.m_normal.normalize();

							reportHit( ref castResult.m_normal,
										ref castResult.m_hitPoint,
										castResult.m_fraction,
										partId,
										triangleIndex );
						}
					}
				}
				BulletGlobals.CastResultPool.Free( castResult );
				BulletGlobals.ContinuousConvexCollisionPool.Free( convexCaster );
			}
		}
	};
}
