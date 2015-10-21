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

namespace Bullet.Collision.BroadPhase
{

	public class btSapBroadphaseArray : btList<btBroadphaseInterface>
	{
	}

///The btMultiSapBroadphase is a research project, not recommended to use in production. Use btAxisSweep3 or btDbvtBroadphase instead.
///The btMultiSapBroadphase is a broadphase that contains multiple SAP broadphases.
///The user can add SAP broadphases that cover the world. A btBroadphaseProxy can be in multiple child broadphases at the same time.
///A btQuantizedBvh acceleration structures finds overlapping SAPs for each btBroadphaseProxy.
///See http://www.continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=328
///and http://www.continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=1329
/// 
public class btMultiSapBroadphase : btBroadphaseInterface
{
	btSapBroadphaseArray	m_sapBroadphases;
	
	btSimpleBroadphase		m_simpleBroadphase;

	btOverlappingPairCache	m_overlappingPairs;

	class btQuantizedBvh*			m_optimizedAabbTree;


	bool					m_ownsPairCache;
	
	btOverlapFilterCallback*	m_filterCallback;

	int			m_invalidPair;

	struct	btBridgeProxy
	{
		btBroadphaseProxy*		m_childProxy;
		btBroadphaseInterface*	m_childBroadphase;
	};


public:

	struct	btMultiSapProxy	: btBroadphaseProxy
	{

		///array with all the entries that this proxy belongs to
		List<btBridgeProxy*> m_bridgeProxies;
		btVector3	m_aabbMin;
		btVector3	m_aabbMax;

		int	m_shapeType;

/*		object	m_userPtr;
		short int	m_collisionFilterGroup;
		short int	m_collisionFilterMask;
*/
		btMultiSapProxy(ref btVector3 aabbMin,  ref btVector3 aabbMax,int shapeType,object userPtr, short int collisionFilterGroup,short int collisionFilterMask)
			:btBroadphaseProxy(aabbMin,aabbMax,userPtr,collisionFilterGroup,collisionFilterMask),
			m_aabbMin(aabbMin),
			m_aabbMax(aabbMax),
			m_shapeType(shapeType)
		{
			m_multiSapParentProxy =this;
		}

		
	};

protected:


	List<btMultiSapProxy*> m_multiSapProxies;

public:

	btMultiSapBroadphase(int maxProxies = 16384,btOverlappingPairCache* pairCache=0);


	btSapBroadphaseArray&	getBroadphaseArray()
	{
		return m_sapBroadphases;
	}

	btSapBroadphaseArray&	getBroadphaseArray()
	{
		return m_sapBroadphases;
	}

	virtual ~btMultiSapBroadphase();

	virtual btBroadphaseProxy*	createProxy(  ref btVector3 aabbMin,  ref btVector3 aabbMax,int shapeType,object userPtr, short int collisionFilterGroup,short int collisionFilterMask, btDispatcher* dispatcher,object multiSapProxy);
	virtual void	destroyProxy(btBroadphaseProxy* proxy,btDispatcher* dispatcher);
	virtual void	setAabb(btBroadphaseProxy* proxy,ref btVector3 aabbMin,ref btVector3 aabbMax, btDispatcher* dispatcher);
	virtual void	getAabb(btBroadphaseProxy* proxy,ref btVector3 aabbMin, ref btVector3 aabbMax );

	virtual void	rayTest(ref btVector3 rayFrom,ref btVector3 rayTo, btBroadphaseRayCallback& rayCallback,ref btVector3 aabbMin=btVector3(0,0,0),ref btVector3 aabbMax=btVector3(0,0,0));

	void	addToChildBroadphase(btMultiSapProxy* parentMultiSapProxy, btBroadphaseProxy* childProxy, btBroadphaseInterface*	childBroadphase);

	///calculateOverlappingPairs is optional: incremental algorithms (sweep and prune) might do it during the set aabb
	virtual void	calculateOverlappingPairs(btDispatcher* dispatcher);

	bool	testAabbOverlap(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1);

	virtual	btOverlappingPairCache*	getOverlappingPairCache()
	{
		return m_overlappingPairs;
	}
	virtual	btOverlappingPairCache*	getOverlappingPairCache()
	{
		return m_overlappingPairs;
	}

	///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
	///will add some transform later
	virtual void getBroadphaseAabb(ref btVector3 aabbMin,ref btVector3 aabbMax)
	{
		aabbMin.setValue(-BT_LARGE_FLOAT,-BT_LARGE_FLOAT,-BT_LARGE_FLOAT);
		aabbMax.setValue(BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT);
	}

	void	buildTree(ref btVector3 bvhAabbMin,ref btVector3 bvhAabbMax);

	virtual void	printStats();

	void quicksort (btBroadphasePairArray& a, int lo, int hi);

	///reset broadphase internal structures, to ensure determinism/reproducability
	virtual void resetPool(btDispatcher* dispatcher);

};

}
