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

#if ! BT_SPHERE_TRIANGLE_DETECTOR_H
#define BT_SPHERE_TRIANGLE_DETECTOR_H

#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"



/// sphere-triangle to match the btDiscreteCollisionDetectorInterface
struct SphereTriangleDetector : btDiscreteCollisionDetectorInterface
{
	virtual void	getClosestPoints(string losestPointInput& input,Result& output,btIDebugDraw debugDraw,bool swapResults=false);

	SphereTriangleDetector(btSphereShape* sphere,btTriangleShape* triangle, double contactBreakingThreshold);

	virtual ~SphereTriangleDetector() {};

	bool collide(ref btVector3 sphereCenter,btVector3 point, ref btVector3 resultNormal, double depth, double &timeOfImpact, double	contactBreakingThreshold);

private:

	
	bool pointInTriangle(btVector3 vertices[], btVector3 normal, btVector3 *p );
	bool facecontains(btVector3 p,btVector3* vertices,ref btVector3 normal);

	btSphereShape* m_sphere;
	btTriangleShape* m_triangle;
	double	m_contactBreakingThreshold;
	
};
#endif //BT_SPHERE_TRIANGLE_DETECTOR_H

