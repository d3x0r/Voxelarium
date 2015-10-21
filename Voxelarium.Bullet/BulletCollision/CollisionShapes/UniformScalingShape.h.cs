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

#if ! BT_UNIFORM_SCALING_SHAPE_H
#define BT_UNIFORM_SCALING_SHAPE_H

#include "btConvexShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types

///The btUniformScalingShape allows to re-use uniform scaled instances of btConvexShape in a memory efficient way.
///Istead of using btUniformScalingShape, it is better to use the non-uniform setLocalScaling method on convex shapes that implement it.
internal class btUniformScalingShape : btConvexShape
{
	btConvexShape*	m_childConvexShape;

	double	m_uniformScalingFactor;
	
	public:
	
	
	
	btUniformScalingShape(	btConvexShape* convexChildShape, double uniformScalingFactor);
	
	virtual ~btUniformScalingShape();
	
	virtual btVector3	localGetSupportingVertexWithoutMargin(ref btVector3 vec)const;

	virtual btVector3	localGetSupportingVertex(ref btVector3 vec)const;

	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(btVector3* vectors,btVector3* supportVerticesOut,int numVectors);

	virtual void	calculateLocalInertia(double mass,ref btVector3 inertia);

	double	getUniformScalingFactor()
	{
		return m_uniformScalingFactor;
	}

	btConvexShape*	getChildShape() 
	{
		return m_childConvexShape;
	}

	btConvexShape*	getChildShape()
	{
		return m_childConvexShape;
	}

	public override string ToString()   	{
		return "UniformScalingShape";
	}
	


	///////////////////////////


	///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	void getAabb(ref btTransform t,ref btVector3 aabbMin,ref btVector3 aabbMax);

	virtual void getAabbSlow(ref btTransform t,ref btVector3 aabbMin,ref btVector3 aabbMax);

	virtual void	setLocalScaling(ref btVector3 scaling) ;
	virtual ref btVector3 getLocalScaling() string 

	virtual void	setMargin(double margin);
	virtual double	getMargin();

	virtual int		getNumPreferredPenetrationDirections();
	
	virtual void	getPreferredPenetrationDirection(int index, ref btVector3 penetrationVector);


};

#endif //BT_UNIFORM_SCALING_SHAPE_H
