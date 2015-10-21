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

#if ! BT_MULTI_SPHERE_MINKOWSKI_H
#define BT_MULTI_SPHERE_MINKOWSKI_H

#include "btConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types
#include "LinearMath/List.h"
#include "LinearMath/btAabbUtil2.h"



///The btMultiSphereShape represents the convex hull of a collection of spheres. You can create special capsules or other smooth volumes.
///It is possible to animate the spheres for deformation, but call 'recalcLocalAabb' after changing any sphere position/radius
internal class btMultiSphereShape : btConvexInternalAabbCachingShape
{
	
	List<btVector3> m_localPositionArray;
	List<double>  m_radiArray;
	
public:
	
	
	btMultiSphereShape (btVector3* positions,double* radi,int numSpheres);

	///CollisionShape Interface
	virtual void	calculateLocalInertia(double mass,ref btVector3 inertia);

	/// btConvexShape Interface
	virtual btVector3	localGetSupportingVertexWithoutMargin(ref btVector3 vec)const;

	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(btVector3* vectors,btVector3* supportVerticesOut,int numVectors);
	
	int	getSphereCount()
	{
		return m_localPositionArray.Count;
	}

	ref btVector3	getSpherePosition(int index)
	{
		return m_localPositionArray[index];
	}

	double	getSphereRadius(int index)
	{
		return m_radiArray[index];
	}


	public override string ToString()   	{
		return "MultiSphere";
	}

	virtual	int	calculateSerializeBufferSize();

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	string	serialize(object dataBuffer, btSerializer* serializer);


};


struct	btPositionAndRadius
{
	btVector3FloatData	m_pos;
	float		m_radius;
};

struct	btMultiSphereShapeData
{
	btConvexInternalShapeData	m_convexInternalShapeData;

	btPositionAndRadius	*m_localPositionArrayPtr;
	int				m_localPositionArraySize;
	char	m_padding[4];
};



public	int	btMultiSphereShape::calculateSerializeBufferSize()
{
	return sizeof(btMultiSphereShapeData);
}



#endif //BT_MULTI_SPHERE_MINKOWSKI_H
