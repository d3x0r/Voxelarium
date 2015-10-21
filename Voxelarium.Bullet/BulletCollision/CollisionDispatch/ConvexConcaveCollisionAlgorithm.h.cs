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


namespace Bullet.Collision.Dispatch
{

///For each triangle in the concave mesh that overlaps with the AABB of a convex (m_convexProxy), processTriangle is called.
class btConvexTriangleCallback : btTriangleCallback
{
	btCollisionObjectWrapper* m_convexBodyWrap;
	btCollisionObjectWrapper* m_triBodyWrap;

	btVector3	m_aabbMin;
	btVector3	m_aabbMax ;


	btManifoldResult* m_resultOut;
	btDispatcher*	m_dispatcher;
	btDispatcherInfo* m_dispatchInfoPtr;
	double m_collisionMarginTriangle;
	
public:
int	m_triangleCount;
	
	btPersistentManifold*	m_manifoldPtr;

	btConvexTriangleCallback(btDispatcher* dispatcher,btCollisionObjectWrapper* body0Wrap,btCollisionObjectWrapper* body1Wrap,bool isSwapped);

	void	setTimeStepAndCounters(double collisionMarginTriangle,btDispatcherInfo& dispatchInfo,btCollisionObjectWrapper* convexBodyWrap, btCollisionObjectWrapper* triBodyWrap, btManifoldResult* resultOut);

	void	clearWrapperData()
	{
		m_convexBodyWrap = 0;
		m_triBodyWrap = 0;
	}
	virtual ~btConvexTriangleCallback();

	virtual void processTriangle(btVector3* triangle, int partId, int triangleIndex);
	
	void clearCache();

	public ref btVector3 getAabbMin()
	{
		return m_aabbMin;
	}
	public ref btVector3 getAabbMax()
	{
		return m_aabbMax;
	}

};




/// btConvexConcaveCollisionAlgorithm  supports collision between convex shapes and (concave) trianges meshes.
class btConvexConcaveCollisionAlgorithm  : btActivatingCollisionAlgorithm
{

	bool	m_isSwapped;

	btConvexTriangleCallback m_btConvexTriangleCallback;



public:

	btConvexConcaveCollisionAlgorithm( btCollisionAlgorithmConstructionInfo& ci,btCollisionObjectWrapper* body0Wrap,btCollisionObjectWrapper* body1Wrap,bool isSwapped);

	virtual ~btConvexConcaveCollisionAlgorithm();

	virtual void processCollision (btCollisionObjectWrapper* body0Wrap,btCollisionObjectWrapper* body1Wrap,btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	double	calculateTimeOfImpact(btCollisionObject body0,btCollisionObject body1,btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(btManifoldArray&	manifoldArray);
	
	void	clearCache();

	struct CreateFunc :public 	btCollisionAlgorithmCreateFunc
	{
		virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, btCollisionObjectWrapper* body0Wrap,btCollisionObjectWrapper* body1Wrap)
		{
			object mem = ci.m_dispatcher1.allocateCollisionAlgorithm(sizeof(btConvexConcaveCollisionAlgorithm));
			return new(mem) btConvexConcaveCollisionAlgorithm(ci,body0Wrap,body1Wrap,false);
		}
	};

	struct SwappedCreateFunc :public 	btCollisionAlgorithmCreateFunc
	{
		virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, btCollisionObjectWrapper* body0Wrap,btCollisionObjectWrapper* body1Wrap)
		{
			object mem = ci.m_dispatcher1.allocateCollisionAlgorithm(sizeof(btConvexConcaveCollisionAlgorithm));
			return new(mem) btConvexConcaveCollisionAlgorithm(ci,body0Wrap,body1Wrap,true);
		}
	};

};

}
