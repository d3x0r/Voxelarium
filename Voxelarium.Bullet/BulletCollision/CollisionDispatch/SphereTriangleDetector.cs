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

#include "LinearMath/double.h"
#include "SphereTriangleDetector.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"


SphereTriangleDetector::SphereTriangleDetector(btSphereShape* sphere,btTriangleShape* triangle,double contactBreakingThreshold)
:m_sphere(sphere),
m_triangle(triangle),
m_contactBreakingThreshold(contactBreakingThreshold)
{

}

void	SphereTriangleDetector::getClosestPoints(string losestPointInput& input,Result& output,btIDebugDraw debugDraw,bool swapResults)
{

	(void)debugDraw;
	ref btTransform transformA = input.m_transformA;
	ref btTransform transformB = input.m_transformB;

	btVector3 point,normal;
	double timeOfImpact = btScalar.BT_ONE;
	double depth = btScalar.BT_ZERO;
//	output.m_distance = (double)(BT_LARGE_FLOAT);
	//move sphere into triangle space
	btTransform	sphereInTr = transformB.inverseTimes(transformA);

	if (collide(sphereInTr.getOrigin(),point,normal,depth,timeOfImpact,m_contactBreakingThreshold))
	{
		if (swapResults)
		{
			btVector3 normalOnB = transformB.getBasis()*normal;
			btVector3 normalOnA = -normalOnB;
			btVector3 pointOnA = transformB*point+normalOnB*depth;
			output.addContactPoint(normalOnA,pointOnA,depth);
		} else
		{
			output.addContactPoint(transformB.getBasis()*normal,transformB*point,depth);
		}
	}

}



// See also geometrictools.com
// Basic idea: D = |p - (lo + t0*lv)| where t0 = lv . (p - lo) / lv . lv
double SegmentSqrDistance(ref btVector3 from, ref btVector3 to,btVector3 p, btVector3 nearest);

double SegmentSqrDistance(ref btVector3 from, ref btVector3 to,btVector3 p, btVector3 nearest) {
	btVector3 diff = p - from;
	btVector3 v = to - from;
	double t = v.dot(diff);
	
	if (t > 0) {
		double dotVV = v.dot(v);
		if (t < dotVV) {
			t /= dotVV;
			diff -= t*v;
		} else {
			t = 1;
			diff -= v;
		}
	} else
		t = 0;

	nearest = from + t*v;
	return diff.dot(diff);	
}

bool SphereTriangleDetector::facecontains(btVector3 p,btVector3* vertices,ref btVector3 normal)  {
	btVector3 lp(p);
	btVector3 lnormal(normal);
	
	return pointInTriangle(vertices, lnormal, &lp);
}

bool SphereTriangleDetector::collide(ref btVector3 sphereCenter,btVector3 point, ref btVector3 resultNormal, double depth, double &timeOfImpact, double contactBreakingThreshold)
{

	btVector3* vertices = &m_triangle.getVertexPtr(0);
	
	double radius = m_sphere.getRadius();
	double radiusWithThreshold = radius + contactBreakingThreshold;

	btVector3 normal = (vertices[1]-vertices[0]).cross(vertices[2]-vertices[0]);
	normal.normalize();
	btVector3 p1ToCentre = sphereCenter - vertices[0];
	double distanceFromPlane = p1ToCentre.dot(normal);

	if (distanceFromPlane < btScalar.BT_ZERO)
	{
		//triangle facing the other way
		distanceFromPlane *= (double)(-1.);
		normal *= (double)(-1.);
	}

	bool isInsideContactPlane = distanceFromPlane < radiusWithThreshold;
	
	// Check for contact / intersection
	bool hasContact = false;
	btVector3 contactPoint;
	if (isInsideContactPlane) {
		if (facecontains(sphereCenter,vertices,normal)) {
			// Inside the contact wedge - touches a point on the shell plane
			hasContact = true;
			contactPoint = sphereCenter - normal*distanceFromPlane;
		} else {
			// Could be inside one of the contact capsules
			double contactCapsuleRadiusSqr = radiusWithThreshold*radiusWithThreshold;
			btVector3 nearestOnEdge;
			for (int i = 0; i < m_triangle.getNumEdges(); i++) {
				
				btVector3 pa;
				btVector3 pb;
				
				m_triangle.getEdge(i,pa,pb);

				double distanceSqr = SegmentSqrDistance(pa,pb,sphereCenter, nearestOnEdge);
				if (distanceSqr < contactCapsuleRadiusSqr) {
					// Yep, we're inside a capsule
					hasContact = true;
					contactPoint = nearestOnEdge;
				}
				
			}
		}
	}

	if (hasContact) {
		btVector3 contactToCentre = sphereCenter - contactPoint;
		double distanceSqr = contactToCentre.length2();

		if (distanceSqr < radiusWithThreshold*radiusWithThreshold)
		{
			if (distanceSqr>SIMD_EPSILON)
			{
				double distance = btSqrt(distanceSqr);
				resultNormal = contactToCentre;
				resultNormal.normalize();
				point = contactPoint;
				depth = -(radius-distance);
			} else
			{
				resultNormal = normal;
				point = contactPoint;
				depth = -radius;
			}
			return true;
		}
	}
	
	return false;
}


bool SphereTriangleDetector::pointInTriangle(btVector3 vertices[], btVector3 normal, btVector3 *p )
{
	btVector3* p1 = vertices;
	btVector3* p2 = &vertices[1];
	btVector3* p3 = &vertices[2];

	btVector3 edge1( *p2 - *p1 );
	btVector3 edge2( *p3 - *p2 );
	btVector3 edge3( *p1 - *p3 );

	btVector3 p1_to_p( *p - *p1 );
	btVector3 p2_to_p( *p - *p2 );
	btVector3 p3_to_p( *p - *p3 );

	btVector3 edge1_normal( edge1.cross(normal));
	btVector3 edge2_normal( edge2.cross(normal));
	btVector3 edge3_normal( edge3.cross(normal));
	
	double r1, r2, r3;
	r1 = edge1_normal.dot( p1_to_p );
	r2 = edge2_normal.dot( p2_to_p );
	r3 = edge3_normal.dot( p3_to_p );
	if ( ( r1 > 0 && r2 > 0 && r3 > 0 ) ||
	     ( r1 <= 0 && r2 <= 0 && r3 <= 0 ) )
		return true;
	return false;

}
