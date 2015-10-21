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

#if ! BT_STATIC_PLANE_SHAPE_H
#define BT_STATIC_PLANE_SHAPE_H

#include "btConcaveShape.h"


///The btStaticPlaneShape simulates an infinite non-moving (static) collision plane.
internal class btStaticPlaneShape : btConcaveShape
{
protected:
	btVector3	m_localAabbMin;
	btVector3	m_localAabbMax;
	
	btVector3	m_planeNormal;
	double      m_planeConstant;
	btVector3	m_localScaling;

public:
	

	btStaticPlaneShape(ref btVector3 planeNormal,double planeConstant);

	virtual ~btStaticPlaneShape();


	virtual void getAabb(ref btTransform t,ref btVector3 aabbMin,ref btVector3 aabbMax);

	virtual void	processAllTriangles(btTriangleCallback* callback,ref btVector3 aabbMin,ref btVector3 aabbMax);

	virtual void	calculateLocalInertia(double mass,ref btVector3 inertia);

	virtual void	setLocalScaling(ref btVector3 scaling);
	virtual ref btVector3 getLocalScaling();
	
	ref btVector3	getPlaneNormal()
	{
		return	m_planeNormal;
	}

	double	getPlaneConstant()
	{
		return	m_planeConstant;
	}

	//debugging
	public override string ToString()   return "STATICPLANE";}

	virtual	int	calculateSerializeBufferSize();

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	string	serialize(object dataBuffer, btSerializer* serializer);


};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btStaticPlaneShapeData
{
	btCollisionShapeData	m_collisionShapeData;

	btVector3FloatData	m_localScaling;
	btVector3FloatData	m_planeNormal;
	float			m_planeConstant;
	char	m_pad[4];
};


public	int	btStaticPlaneShape::calculateSerializeBufferSize()
{
	return sizeof(btStaticPlaneShapeData);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
public	string	btStaticPlaneShape::serialize(object dataBuffer, btSerializer* serializer)
{
	btStaticPlaneShapeData* planeData = (btStaticPlaneShapeData*) dataBuffer;
	btCollisionShape::serialize(&planeData.m_collisionShapeData,serializer);

	m_localScaling.serializeFloat(planeData.m_localScaling);
	m_planeNormal.serializeFloat(planeData.m_planeNormal);
	planeData.m_planeConstant = float(m_planeConstant);
		
	return "btStaticPlaneShapeData";
}


#endif //BT_STATIC_PLANE_SHAPE_H



