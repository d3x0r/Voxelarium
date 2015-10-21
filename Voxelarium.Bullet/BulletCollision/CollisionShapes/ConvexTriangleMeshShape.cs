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

#include "btConvexTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"

#include "LinearMath/btQuaternion.h"
#include "BulletCollision/CollisionShapes/btStridingMeshInterface.h"


btConvexTriangleMeshShape ::btConvexTriangleMeshShape (btStridingMeshInterface* meshInterface, bool calcAabb)
: btPolyhedralConvexAabbCachingShape(), m_stridingMesh(meshInterface)
{
	m_shapeType = CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE;
	if ( calcAabb )
		recalcLocalAabb();
}




///It's not nice to have all this virtual function overhead, so perhaps we can also gather the points once
///but then we are duplicating
class LocalSupportVertexCallback: btInternalTriangleIndexCallback
{

	btVector3 m_supportVertexLocal;
public:

	double m_maxDot;
	btVector3 m_supportVecLocal;

	LocalSupportVertexCallback(ref btVector3 supportVecLocal)
		: m_supportVertexLocal(btScalar.BT_ZERO,btScalar.BT_ZERO,btScalar.BT_ZERO),
		m_maxDot((double)(-BT_LARGE_FLOAT)),
                m_supportVecLocal(supportVecLocal)
	{
	}

	virtual void internalProcessTriangleIndex(btVector3* triangle,int partId,int  triangleIndex)
	{
		(void)triangleIndex;
		(void)partId;

		for (int i=0;i<3;i++)
		{
			double dot = m_supportVecLocal.dot(triangle[i]);
			if (dot > m_maxDot)
			{
				m_maxDot = dot;
				m_supportVertexLocal = triangle[i];
			}
		}
	}
	
	btVector3	GetSupportVertexLocal()
	{
		return m_supportVertexLocal;
	}

};





btVector3	btConvexTriangleMeshShape::localGetSupportingVertexWithoutMargin(ref btVector3 vec0)const
{
	btVector3 supVec(btScalar.BT_ZERO,btScalar.BT_ZERO,btScalar.BT_ZERO);

	btVector3 vec = vec0;
	double lenSqr = vec.length2();
	if (lenSqr < (double)(0.0001))
	{
		vec.setValue(1,0,0);
	} else
	{
		double rlen = btScalar.BT_ONE / btSqrt(lenSqr );
		vec *= rlen;
	}

	LocalSupportVertexCallback	supportCallback(vec);
	btVector3 aabbMax((double)(BT_LARGE_FLOAT),(double)(BT_LARGE_FLOAT),(double)(BT_LARGE_FLOAT));
	m_stridingMesh.InternalProcessAllTriangles(&supportCallback,-aabbMax,aabbMax);
	supVec = supportCallback.GetSupportVertexLocal();

	return supVec;
}

void	btConvexTriangleMeshShape::batchedUnitVectorGetSupportingVertexWithoutMargin(btVector3* vectors,btVector3* supportVerticesOut,int numVectors)
{
	//use 'w' component of supportVerticesOut?
	{
		for (int i=0;i<numVectors;i++)
		{
			supportVerticesOut[i][3] = (double)(-BT_LARGE_FLOAT);
		}
	}
	
	///@todo: could do the batch inside the callback!


	for (int j=0;j<numVectors;j++)
	{
		ref btVector3 vec = vectors[j];
		LocalSupportVertexCallback	supportCallback(vec);
		btVector3 aabbMax((double)(BT_LARGE_FLOAT),(double)(BT_LARGE_FLOAT),(double)(BT_LARGE_FLOAT));
		m_stridingMesh.InternalProcessAllTriangles(&supportCallback,-aabbMax,aabbMax);
		supportVerticesOut[j] = supportCallback.GetSupportVertexLocal();
	}
	
}
	


btVector3	btConvexTriangleMeshShape::localGetSupportingVertex(ref btVector3 vec)const
{
	btVector3 supVertex = localGetSupportingVertexWithoutMargin(vec);

	if ( getMargin()!=btScalar.BT_ZERO )
	{
		btVector3 vecnorm = vec;
		if (vecnorm .length2() < (SIMD_EPSILON*SIMD_EPSILON))
		{
			vecnorm.setValue((double)(-1.),(double)(-1.),(double)(-1.));
		} 
		vecnorm.normalize();
		supVertex+= getMargin() * vecnorm;
	}
	return supVertex;
}









//currently just for debugging (drawing), perhaps future support for algebraic continuous collision detection
//Please note that you can debug-draw btConvexTriangleMeshShape with the Raytracer Demo
int	btConvexTriangleMeshShape::getNumVertices()
{
	//cache this?
	return 0;
	
}

int btConvexTriangleMeshShape::getNumEdges()
{
	return 0;
}

void btConvexTriangleMeshShape::getEdge(int ,ref btVector3 ,ref btVector3 )
{
	Debug.Assert(false);	
}

void btConvexTriangleMeshShape::getVertex(int ,ref btVector3 )
{
	Debug.Assert(false);
}

int	btConvexTriangleMeshShape::getNumPlanes()
{
	return 0;
}

void btConvexTriangleMeshShape::getPlane(ref btVector3 ,ref btVector3 ,int  )
{
	Debug.Assert(false);
}

//not yet
bool btConvexTriangleMeshShape::isInside(ref btVector3 ,double )
{
	Debug.Assert(false);
	return false;
}



void	btConvexTriangleMeshShape::setLocalScaling(ref btVector3 scaling)
{
	m_stridingMesh.setScaling(scaling);
	
	recalcLocalAabb();
	
}


ref btVector3 btConvexTriangleMeshShape::getLocalScaling()
{
	return m_stridingMesh.getScaling();
}

void btConvexTriangleMeshShape::calculatePrincipalAxisTransform(ref btTransform principal, ref btVector3 inertia, double volume)
{
   class CenterCallback: btInternalTriangleIndexCallback
   {
      bool first;
      btVector3 ref;
      btVector3 sum;
      double volume;

   public:

      CenterCallback() : first(true), ref(0, 0, 0), sum(0, 0, 0), volume(0)
      {
      }

      virtual void internalProcessTriangleIndex(btVector3* triangle, int partId, int triangleIndex)
      {
         (void) triangleIndex;
         (void) partId;
         if (first)
         {
            ref = triangle[0];
            first = false;
         }
         else
         {
            double vol = btFabs((triangle[0] - ref).triple(triangle[1] - ref, triangle[2] - ref));
            sum += ((double)(0.25) * vol) * ((triangle[0] + triangle[1] + triangle[2] + ref));
            volume += vol;
         }
      }
      
      btVector3 getCenter()
      {
         return (volume > 0) ? sum / volume : ref;
      }

      double getVolume()
      {
         return volume * (double)(1. / 6);
      }

   };

   class InertiaCallback: btInternalTriangleIndexCallback
   {
      btMatrix3x3 sum;
      btVector3 center;

   public:

      InertiaCallback(ref btVector3 center) : sum(0, 0, 0, 0, 0, 0, 0, 0, 0), center(center)
      {
      }

      virtual void internalProcessTriangleIndex(btVector3* triangle, int partId, int triangleIndex)
      {
         (void) triangleIndex;
         (void) partId;
         btMatrix3x3 i;
         btVector3 a = triangle[0] - center;
         btVector3 b = triangle[1] - center;
         btVector3 c = triangle[2] - center;
         double volNeg = -btFabs(a.triple(b, c)) * (double)(1. / 6);
         for (int j = 0; j < 3; j++)
         {
            for (int k = 0; k <= j; k++)
            {
               i[j][k] = i[k][j] = volNeg * ((double)(0.1) * (a[j] * a[k] + b[j] * b[k] + c[j] * c[k])
                  + (double)(0.05) * (a[j] * b[k] + a[k] * b[j] + a[j] * c[k] + a[k] * c[j] + b[j] * c[k] + b[k] * c[j]));
            }
         }
         double i00 = -i[0][0];
         double i11 = -i[1][1];
         double i22 = -i[2][2];
         i[0][0] = i11 + i22; 
         i[1][1] = i22 + i00; 
         i[2][2] = i00 + i11;
         sum[0] += i[0];
         sum[1] += i[1];
         sum[2] += i[2];
      }
      
      btMatrix3x3 getInertia()
      {
         return sum;
      }

   };

   CenterCallback centerCallback;
   btVector3 aabbMax((double)(BT_LARGE_FLOAT),(double)(BT_LARGE_FLOAT),(double)(BT_LARGE_FLOAT));
   m_stridingMesh.InternalProcessAllTriangles(&centerCallback, -aabbMax, aabbMax);
   btVector3 center = centerCallback.getCenter();
   principal.setOrigin(center);
   volume = centerCallback.getVolume();

   InertiaCallback inertiaCallback(center);
   m_stridingMesh.InternalProcessAllTriangles(&inertiaCallback, -aabbMax, aabbMax);

   btMatrix3x3& i = inertiaCallback.getInertia();
   i.diagonalize(principal.getBasis(), (double)(0.00001), 20);
   inertia.setValue(i[0], i[1][1], i[2][2]);
   inertia /= volume;
}

