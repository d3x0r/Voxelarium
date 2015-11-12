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

using Bullet.Collision.NarrowPhase;
using Bullet.Collision.Shapes;
using Bullet.LinearMath;

namespace Bullet.Collision.Dispatch
{


	/// sphere-triangle to match the btDiscreteCollisionDetectorInterface
	internal class SphereTriangleDetector : btDiscreteCollisionDetectorInterface
	{

		btSphereShape m_sphere;
		btTriangleShape m_triangle;
		double m_contactBreakingThreshold;

		public SphereTriangleDetector() { }

		internal void Initialize( btSphereShape sphere, btTriangleShape triangle, double contactBreakingThreshold )
		{
			m_sphere = ( sphere );
			m_triangle = ( triangle );
			m_contactBreakingThreshold = ( contactBreakingThreshold );
		}

		internal override void getClosestPoints( ClosestPointInput input, Result output, btIDebugDraw debugDraw, bool swapResults = false )
		{

			//(void)debugDraw;
			//btTransform transformA = input.m_transformA;
			//btTransform transformB = input.m_transformB;

			btVector3 point, normal;
			double timeOfImpact = btScalar.BT_ONE;
			double depth = btScalar.BT_ZERO;
			//	output.m_distance = (double)(BT_LARGE_FLOAT);
			//move sphere into triangle space
			btTransform sphereInTr; input.m_transformB.inverseTimes( ref input.m_transformA, out sphereInTr );

			if( collide( ref sphereInTr.m_origin, out point, out normal, out depth, timeOfImpact, m_contactBreakingThreshold ) )
			{
				if( swapResults )
				{

					btVector3 normalOnB; input.m_transformB.m_basis.Apply( ref normal, out normalOnB );
					btVector3 normalOnA; normalOnB.Invert( out normalOnA ); 
					btVector3 tmp;
					input.m_transformB.Apply( ref point, out tmp );

					btVector3 pointOnA; tmp.AddScale( ref normalOnB, depth, out pointOnA );
					output.addContactPoint( ref normalOnA, ref pointOnA, depth );
				}
				else
				{
					btVector3 tmp, tmp2;
					input.m_transformB.m_basis.Apply( ref normal, out tmp );
					input.m_transformB.Apply( ref point, out tmp2 );
					output.addContactPoint( ref tmp, ref tmp2, depth );
				}
			}

		}



		// See also geometrictools.com
		// Basic idea: D = |p - (lo + t0*lv)| where t0 = lv . (p - lo) / lv . lv
		double SegmentSqrDistance( ref btVector3 from, ref btVector3 to, ref btVector3 p, out btVector3 nearest )
		{
			btVector3 diff = p - from;
			btVector3 v = to - from;
			double t = v.dot( diff );

			if( t > 0 )
			{
				double dotVV = v.dot( v );
				if( t < dotVV )
				{
					t /= dotVV;
					diff.AddScale( ref v, -t, out diff );
					//diff -= t * v;
				}
				else
				{
					t = 1;
					diff.Sub( ref v, out diff );
					//diff -= v;
				}
			}
			else
				t = 0;

			from.AddScale( ref v, t, out nearest );
			//nearest = from + t * v;
			return diff.dot( ref diff );
		}


		bool collide( ref btVector3 sphereCenter, out btVector3 point, out btVector3 resultNormal, out double depth, double timeOfImpact, double contactBreakingThreshold )
		{

			//btVector3[] vertices = m_triangle.getVertexPtr( 0 );

			double radius = m_sphere.getRadius();
			double radiusWithThreshold = radius + contactBreakingThreshold;


			btVector3 normal;
			btVector3.btCross2Del( ref m_triangle.m_vertices2, ref m_triangle.m_vertices1
				, ref m_triangle.m_vertices3, ref m_triangle.m_vertices1
				, out normal );
			normal.normalize();

			btVector3 p1ToCentre; sphereCenter.Sub( ref m_triangle.m_vertices1, out p1ToCentre );
			double distanceFromPlane = p1ToCentre.dot( ref normal );

			if( distanceFromPlane < btScalar.BT_ZERO )
			{
				//triangle facing the other way
				distanceFromPlane *= btScalar.BT_NEG_ONE;
				normal *= btScalar.BT_NEG_ONE;
			}

			bool isInsideContactPlane = distanceFromPlane < radiusWithThreshold;

			// Check for contact / intersection
			bool hasContact = false;
			btVector3 contactPoint = btVector3.Zero;
			if( isInsideContactPlane )
			{
				if( pointInTriangle(
					ref m_triangle.m_vertices1
					, ref m_triangle.m_vertices2
					, ref m_triangle.m_vertices3
					, ref normal
					, ref sphereCenter
					) )
				{
					// Inside the contact wedge - touches a point on the shell plane
					hasContact = true;
					contactPoint = sphereCenter - normal * distanceFromPlane;
				}
				else
				{
					// Could be inside one of the contact capsules
					double contactCapsuleRadiusSqr = radiusWithThreshold * radiusWithThreshold;
					btVector3 nearestOnEdge;
					for( int i = 0; i < m_triangle.getNumEdges(); i++ )
					{

						btVector3 pa;
						btVector3 pb;

						m_triangle.getEdge( i, out pa, out pb );

						double distanceSqr = SegmentSqrDistance( ref pa, ref pb, ref sphereCenter, out nearestOnEdge );
						if( distanceSqr < contactCapsuleRadiusSqr )
						{
							// Yep, we're inside a capsule
							hasContact = true;
							contactPoint = nearestOnEdge;
						}

					}
				}
			}

			if( hasContact )
			{
				btVector3 contactToCentre; sphereCenter.Sub( ref contactPoint, out contactToCentre );
				double distanceSqr = contactToCentre.length2();

				if( distanceSqr < radiusWithThreshold * radiusWithThreshold )
				{
					if( distanceSqr > btScalar.SIMD_EPSILON )
					{
						double distance = btScalar.btSqrt( distanceSqr );
						resultNormal = contactToCentre;
						resultNormal.normalize();
						point = contactPoint;
						depth = -( radius - distance );
					}
					else
					{
						resultNormal = normal;
						point = contactPoint;
						depth = -radius;
					}
					return true;
				}
			}
			depth = 0;
			point = btVector3.Zero;
			resultNormal = btVector3.Zero;
			return false;
		}


		bool pointInTriangle(
			ref btVector3 vertice0,
			ref btVector3 vertice1,
			ref btVector3 vertice2,
			ref btVector3 normal, ref btVector3 p )
		{

			btVector3 edge1; vertice1.Sub( ref vertice0, out edge1 );// ( *p2 - *p1 );
			btVector3 edge2; vertice2.Sub( ref vertice1, out edge2 );//( *p3 - *p2 );
			btVector3 edge3; vertice0.Sub( ref vertice2, out edge3 );//( *p1 - *p3 );

			btVector3 p1_to_p; p.Sub( ref vertice0, out p1_to_p );// ( *p - *p1 );
			btVector3 p2_to_p; p.Sub( ref vertice1, out p2_to_p );// ( *p - *p2 );
			btVector3 p3_to_p; p.Sub( ref vertice2, out p3_to_p );// ( *p - *p3 );

			btVector3 edge1_normal; edge1.cross( ref normal, out edge1_normal );
			btVector3 edge2_normal; edge2.cross( ref normal, out edge2_normal );
			btVector3 edge3_normal; edge3.cross( ref normal, out edge3_normal );

			double r1, r2, r3;
			r1 = edge1_normal.dot( ref p1_to_p );
			r2 = edge2_normal.dot( ref p2_to_p );
			r3 = edge3_normal.dot( ref p3_to_p );
			if( ( r1 > 0 && r2 > 0 && r3 > 0 ) ||
				 ( r1 <= 0 && r2 <= 0 && r3 <= 0 ) )
				return true;
			return false;

		}

	};

}


