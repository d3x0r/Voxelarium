/*
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "btGImpactShape.h"
#include "btGImpactMassUtil.h"


#define CALC_EXACT_INERTIA 1


void btGImpactCompoundShape::calculateLocalInertia(double mass,ref btVector3 inertia)
{
	lockChildShapes();
#if CALC_EXACT_INERTIA
	inertia.setValue(0,0,0);

	int i = this.getNumChildShapes();
	double shapemass = mass/(double)(i);

	while(i--)
	{
		btVector3 temp_inertia;
		m_childShapes[i].calculateLocalInertia(shapemass,temp_inertia);
		if(childrenHasTransform())
		{
			inertia = gim_inertia_add_transformed( inertia,temp_inertia,m_childTransforms[i]);
		}
		else
		{
			inertia = gim_inertia_add_transformed( inertia,temp_inertia,btTransform::getIdentity());
		}

	}

#else

	// Calc box inertia

	double lx= m_localAABB.m_max[0] - m_localAABB.m_min[0];
	double ly= m_localAABB.m_max[1] - m_localAABB.m_min[1];
	double lz= m_localAABB.m_max[2] - m_localAABB.m_min[2];
	double x2 = lx*lx;
	double y2 = ly*ly;
	double z2 = lz*lz;
	double scaledmass = mass * (double)(0.08333333);

	inertia = scaledmass * (btVector3(y2+z2,x2+z2,x2+y2));

#endif
	unlockChildShapes();
}



void btGImpactMeshShapePart::calculateLocalInertia(double mass,ref btVector3 inertia)
{
	lockChildShapes();


#if CALC_EXACT_INERTIA
	inertia.setValue(0,0,0);

	int i = this.getVertexCount();
	double pointmass = mass/(double)(i);

	while(i--)
	{
		btVector3 pointintertia;
		this.getVertex(i,pointintertia);
		pointintertia = gim_get_point_inertia(pointintertia,pointmass);
		inertia+=pointintertia;
	}

#else

	// Calc box inertia

	double lx= m_localAABB.m_max[0] - m_localAABB.m_min[0];
	double ly= m_localAABB.m_max[1] - m_localAABB.m_min[1];
	double lz= m_localAABB.m_max[2] - m_localAABB.m_min[2];
	double x2 = lx*lx;
	double y2 = ly*ly;
	double z2 = lz*lz;
	double scaledmass = mass * (double)(0.08333333);

	inertia = scaledmass * (btVector3(y2+z2,x2+z2,x2+y2));

#endif

	unlockChildShapes();
}

void btGImpactMeshShape::calculateLocalInertia(double mass,ref btVector3 inertia)
{

#if CALC_EXACT_INERTIA
	inertia.setValue(0,0,0);

	int i = this.getMeshPartCount();
	double partmass = mass/(double)(i);

	while(i--)
	{
		btVector3 partinertia;
		getMeshPart(i).calculateLocalInertia(partmass,partinertia);
		inertia+=partinertia;
	}

#else

	// Calc box inertia

	double lx= m_localAABB.m_max[0] - m_localAABB.m_min[0];
	double ly= m_localAABB.m_max[1] - m_localAABB.m_min[1];
	double lz= m_localAABB.m_max[2] - m_localAABB.m_min[2];
	double x2 = lx*lx;
	double y2 = ly*ly;
	double z2 = lz*lz;
	double scaledmass = mass * (double)(0.08333333);

	inertia = scaledmass * (btVector3(y2+z2,x2+z2,x2+y2));

#endif
}

void btGImpactMeshShape::rayTest(ref btVector3 rayFrom, ref btVector3 rayTo, btCollisionWorld::RayResultCallback& resultCallback)
{
}

void btGImpactMeshShapePart::processAllTrianglesRay(btTriangleCallback* callback,ref btVector3 rayFrom, ref btVector3 rayTo)
{
	lockChildShapes();

	List<int> collided;
	btVector3 rayDir(rayTo - rayFrom);
	rayDir.normalize();
	m_box_set.rayQuery(rayDir, rayFrom, collided);

	if(collided.Count==0)
	{
		unlockChildShapes();
		return;
	}

	int part = (int)getPart();
	btPrimitiveTriangle triangle;
	int i = collided.Count;
	while(i--)
	{
		getPrimitiveTriangle(collided[i],triangle);
		callback.processTriangle(triangle.m_vertices,part,collided[i]);
	}
	unlockChildShapes();
}

void btGImpactMeshShapePart::processAllTriangles(btTriangleCallback* callback,ref btVector3 aabbMin,ref btVector3 aabbMax)
{
	lockChildShapes();
	btAABB box;
	box.m_min = aabbMin;
	box.m_max = aabbMax;

	List<int> collided;
	m_box_set.boxQuery(box,collided);

	if(collided.Count==0)
	{
		unlockChildShapes();
		return;
	}

	int part = (int)getPart();
	btPrimitiveTriangle triangle;
	int i = collided.Count;
	while(i--)
	{
		this.getPrimitiveTriangle(collided[i],triangle);
		callback.processTriangle(triangle.m_vertices,part,collided[i]);
	}
	unlockChildShapes();

}

void btGImpactMeshShape::processAllTriangles(btTriangleCallback* callback,ref btVector3 aabbMin,ref btVector3 aabbMax)
{
	int i = m_mesh_parts.Count;
	while(i--)
	{
		m_mesh_parts[i].processAllTriangles(callback,aabbMin,aabbMax);
	}
}

void btGImpactMeshShape::processAllTrianglesRay(btTriangleCallback* callback,ref btVector3 rayFrom, ref btVector3 rayTo)
{
	int i = m_mesh_parts.Count;
	while(i--)
	{
		m_mesh_parts[i].processAllTrianglesRay(callback, rayFrom, rayTo);
	}
}


///fills the dataBuffer and returns the struct name (and 0 on failure)
string	btGImpactMeshShape::serialize(object dataBuffer, btSerializer* serializer)
{
	btGImpactMeshShapeData* trimeshData = (btGImpactMeshShapeData*) dataBuffer;

	btCollisionShape::serialize(&trimeshData.m_collisionShapeData,serializer);

	m_meshInterface.serialize(&trimeshData.m_meshInterface, serializer);

	trimeshData.m_collisionMargin = float(m_collisionMargin);

	localScaling.serializeFloat(trimeshData.m_localScaling);

	trimeshData.m_gimpactSubType = int(getGImpactShapeType());

	return "btGImpactMeshShapeData";
}

