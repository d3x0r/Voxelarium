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


#include "btMinkowskiSumShape.h"


btMinkowskiSumShape::btMinkowskiSumShape(btConvexShape* shapeA,btConvexShape* shapeB)
: btConvexInternalShape (),
m_shapeA(shapeA),
m_shapeB(shapeB)
{
	m_shapeType = MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE;
	m_transA.setIdentity();
	m_transB.setIdentity();
}

btVector3 btMinkowskiSumShape::localGetSupportingVertexWithoutMargin(ref btVector3 vec)const
{
	btVector3 supVertexA = m_transA(m_shapeA.localGetSupportingVertexWithoutMargin(vec*m_transA.getBasis()));
	btVector3 supVertexB = m_transB(m_shapeB.localGetSupportingVertexWithoutMargin(-vec*m_transB.getBasis()));
	return  supVertexA - supVertexB;
}

void	btMinkowskiSumShape::batchedUnitVectorGetSupportingVertexWithoutMargin(btVector3* vectors,btVector3* supportVerticesOut,int numVectors)
{
	///@todo: could make recursive use of batching. probably this shape is not used frequently.
	for (int i=0;i<numVectors;i++)
	{
		supportVerticesOut[i] = localGetSupportingVertexWithoutMargin(vectors[i]);
	}

}



double	btMinkowskiSumShape::getMargin()
{
	return m_shapeA.getMargin() + m_shapeB.getMargin();
}


void	btMinkowskiSumShape::calculateLocalInertia(double mass,ref btVector3 inertia)
{
	(void)mass;
	Debug.Assert(false);
	inertia.setValue(0,0,0);
}
