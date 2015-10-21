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
#if ! BT_CONVEX_TRIANGLEMESH_SHAPE_H
#define BT_CONVEX_TRIANGLEMESH_SHAPE_H


#include "btPolyhedralConvexShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types


/// The btConvexTriangleMeshShape is a convex hull of a triangle mesh, but the performance is not as good as btConvexHullShape.
/// A small benefit of this class is that it uses the btStridingMeshInterface, so you can avoid the duplication of the triangle mesh data. Nevertheless, most users should use the much better performing btConvexHullShape instead.
internal class btConvexTriangleMeshShape : btPolyhedralConvexAabbCachingShape
{

	class btStridingMeshInterface*	m_stridingMesh;

public:
	
	
	btConvexTriangleMeshShape(btStridingMeshInterface* meshInterface, bool calcAabb = true);

	class btStridingMeshInterface*	getMeshInterface()
	{
		return m_stridingMesh;
	}
	string lass btStridingMeshInterface* getMeshInterface()
	{
		return m_stridingMesh;
	}
	
	virtual btVector3	localGetSupportingVertex(ref btVector3 vec)const;
	virtual btVector3	localGetSupportingVertexWithoutMargin(ref btVector3 vec)const;
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(btVector3* vectors,btVector3* supportVerticesOut,int numVectors);
	
	//debugging
	public override string ToString()   return "ConvexTrimesh";}
	
	virtual int	getNumVertices();
	virtual int getNumEdges();
	virtual void getEdge(int i,ref btVector3 pa,ref btVector3 pb);
	virtual void getVertex(int i,ref btVector3 vtx);
	virtual int	getNumPlanes();
	virtual void getPlane(ref btVector3 planeNormal,ref btVector3 planeSupport,int i );
	virtual	bool isInside(ref btVector3 pt,double tolerance);

	
	virtual void	setLocalScaling(ref btVector3 scaling);
	virtual ref btVector3 getLocalScaling();

	///computes the exact moment of inertia and the transform from the coordinate system defined by the principal axes of the moment of inertia
	///and the center of mass to the current coordinate system. A mass of 1 is assumed, for other masses just multiply the computed "inertia"
	///by the mass. The resulting transform "principal" has to be applied inversely to the mesh in order for the local coordinate system of the
	///shape to be centered at the center of mass and to coincide with the principal axes. This also necessitates a correction of the world transform
	///of the collision object by the principal transform. This method also computes the volume of the convex mesh.
	void calculatePrincipalAxisTransform(ref btTransform principal, ref btVector3 inertia, double volume);

};



#endif //BT_CONVEX_TRIANGLEMESH_SHAPE_H



