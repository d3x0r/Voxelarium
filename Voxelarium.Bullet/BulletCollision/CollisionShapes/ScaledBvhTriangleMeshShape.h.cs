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

#if ! BT_SCALED_BVH_TRIANGLE_MESH_SHAPE_H
#define BT_SCALED_BVH_TRIANGLE_MESH_SHAPE_H

#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"


///The btScaledBvhTriangleMeshShape allows to instance a scaled version of an existing btBvhTriangleMeshShape.
///Note that each btBvhTriangleMeshShape still can have its own local scaling, independent from this btScaledBvhTriangleMeshShape 'localScaling'
internal class btScaledBvhTriangleMeshShape : btConcaveShape
{
	
	
	btVector3	m_localScaling;

	btBvhTriangleMeshShape*	m_bvhTriMeshShape;

public:

	


	btScaledBvhTriangleMeshShape(btBvhTriangleMeshShape* childShape,ref btVector3 localScaling);

	virtual ~btScaledBvhTriangleMeshShape();


	virtual void getAabb(ref btTransform t,ref btVector3 aabbMin,ref btVector3 aabbMax);
	virtual void	setLocalScaling(ref btVector3 scaling);
	virtual ref btVector3 getLocalScaling();
	virtual void	calculateLocalInertia(double mass,ref btVector3 inertia);

	virtual void	processAllTriangles(btTriangleCallback* callback,ref btVector3 aabbMin,ref btVector3 aabbMax);

	btBvhTriangleMeshShape*	getChildShape()
	{
		return m_bvhTriMeshShape;
	}

	btBvhTriangleMeshShape*	getChildShape()
	{
		return m_bvhTriMeshShape;
	}

	//debugging
	public override string ToString()   return "SCALEDBVHTRIANGLEMESH";}

	virtual	int	calculateSerializeBufferSize();

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	string	serialize(object dataBuffer, btSerializer* serializer);

};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btScaledTriangleMeshShapeData
{
	btTriangleMeshShapeData	m_trimeshShapeData;

	btVector3FloatData	m_localScaling;
};


public	int	btScaledBvhTriangleMeshShape::calculateSerializeBufferSize()
{
	return sizeof(btScaledTriangleMeshShapeData);
}


///fills the dataBuffer and returns the struct name (and 0 on failure)
public	string	btScaledBvhTriangleMeshShape::serialize(object dataBuffer, btSerializer* serializer)
{
	btScaledTriangleMeshShapeData* scaledMeshData = (btScaledTriangleMeshShapeData*) dataBuffer;
	m_bvhTriMeshShape.serialize(&scaledMeshData.m_trimeshShapeData,serializer);
	scaledMeshData.m_trimeshShapeData.m_collisionShapeData.m_shapeType = SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE;
	m_localScaling.serializeFloat(scaledMeshData.m_localScaling);
	return "btScaledTriangleMeshShapeData";
}


#endif //BT_SCALED_BVH_TRIANGLE_MESH_SHAPE_H
