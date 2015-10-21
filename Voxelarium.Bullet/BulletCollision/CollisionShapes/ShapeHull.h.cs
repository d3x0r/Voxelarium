/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///btShapeHull implemented by John McCutchan.

#if ! BT_SHAPE_HULL_H
#define BT_SHAPE_HULL_H

#include "LinearMath/List.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"


///The btShapeHull class takes a btConvexShape, builds a simplified convex hull using btConvexHull and provides triangle indices and vertices.
///It can be useful for to simplify a complex convex object and for visualization of a non-polyhedral convex object.
///It approximates the convex hull using the supporting vertex of 42 directions.
internal class btShapeHull
{
protected:

	List<btVector3> m_vertices;
	List<uint> m_indices;
	uint m_numIndices;
	btConvexShape* m_shape;

	static btVector3* getUnitSpherePoints();

public:
	
	
	btShapeHull (btConvexShape* shape);
	~btShapeHull ();

	bool buildHull (double margin);

	int numTriangles ();
	int numVertices ();
	int numIndices ();

	btVector3* getVertexPointer()
	{
		return m_vertices;
	}
	string nsigned int* getIndexPointer()
	{
		return m_indices;
	}
};

#endif //BT_SHAPE_HULL_H
