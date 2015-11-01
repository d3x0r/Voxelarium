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


	struct btSimpleBroadphaseProxy : btBroadphaseProxy
	{
		int m_nextFree;

		//	int			m_handleId;


		//	public btSimpleBroadphaseProxy() {}

		btSimpleBroadphaseProxy( ref btVector3 minpt, ref btVector3 maxpt, int shapeType, object userPtr, short int collisionFilterGroup, short int collisionFilterMask, object multiSapProxy )
		: base( ref minpt, maxpt, userPtr, collisionFilterGroup, collisionFilterMask, multiSapProxy )
		{
			//(void)shapeType;
		}


		public void SetNextFree( int next ) { m_nextFree = next; }
		public int GetNextFree() { return m_nextFree; }



	};

	///The SimpleBroadphase is just a unit-test for btAxisSweep3, bt32BitAxisSweep3, or btDbvtBroadphase, so use those classes instead.
	///It is a brute force aabb culling broadphase based on O(n^2) aabb checks
	class btSimpleBroadphase : btBroadphaseInterface
	{

		protected:

	int m_numHandles;                       // number of active handles
		int m_maxHandles;                       // max number of handles
		int m_LastHandleIndex;

		btSimpleBroadphaseProxy* m_pHandles;                        // handles pool

		object m_pHandlesRawPtr;
		int m_firstFreeHandle;      // free handles list

		int allocHandle()
		{
			Debug.Assert( m_numHandles < m_maxHandles );
			int freeHandle = m_firstFreeHandle;
			m_firstFreeHandle = m_pHandles[freeHandle].GetNextFree();
			m_numHandles++;
			if( freeHandle > m_LastHandleIndex )
			{
				m_LastHandleIndex = freeHandle;
			}
			return freeHandle;
		}

		void freeHandle( btSimpleBroadphaseProxy* proxy )
		{
			int handle = int( proxy - m_pHandles );
			Debug.Assert( handle >= 0 && handle < m_maxHandles );
			if( handle == m_LastHandleIndex )
			{
				m_LastHandleIndex--;
			}
			proxy.SetNextFree( m_firstFreeHandle );
			m_firstFreeHandle = handle;

			proxy.m_clientObject = 0;

			m_numHandles--;
		}

		btOverlappingPairCache* m_pairCache;
		bool m_ownsPairCache;

		int m_invalidPair;



		inline btSimpleBroadphaseProxy*	getSimpleProxyFromProxy( btBroadphaseProxy* proxy )
		{
			btSimpleBroadphaseProxy* proxy0 = static_cast<btSimpleBroadphaseProxy*>( proxy );
			return proxy0;
		}

		inline btSimpleBroadphaseProxy*	getSimpleProxyFromProxy( btBroadphaseProxy* proxy )
		{
			btSimpleBroadphaseProxy* proxy0 = static_cast<btSimpleBroadphaseProxy*>( proxy );
			return proxy0;
		}

		///reset broadphase internal structures, to ensure determinism/reproducability
		virtual void resetPool( btDispatcher* dispatcher );


		void validate();

		protected:


	

public:
	btSimpleBroadphase( int maxProxies = 16384, btOverlappingPairCache* overlappingPairCache = 0 );
		virtual ~btSimpleBroadphase();


		static bool aabbOverlap( btSimpleBroadphaseProxy* proxy0, btSimpleBroadphaseProxy* proxy1 );


		virtual btBroadphaseProxy* createProxy( ref btVector3 aabbMin, ref btVector3 aabbMax, int shapeType, object userPtr, short int collisionFilterGroup, short int collisionFilterMask, btDispatcher* dispatcher, object multiSapProxy );

		virtual void calculateOverlappingPairs( btDispatcher* dispatcher );

		virtual void destroyProxy( btBroadphaseProxy* proxy, btDispatcher* dispatcher );
		virtual void setAabb( btBroadphaseProxy* proxy, ref btVector3 aabbMin, ref btVector3 aabbMax, btDispatcher* dispatcher );
		virtual void getAabb( btBroadphaseProxy* proxy, ref btVector3 aabbMin, ref btVector3 aabbMax );

		virtual void rayTest( ref btVector3 rayFrom, ref btVector3 rayTo, btBroadphaseRayCallback& rayCallback, ref btVector3 aabbMin = btVector3( 0, 0, 0 ), ref btVector3 aabbMax = btVector3( 0, 0, 0 ) );
		virtual void aabbTest( ref btVector3 aabbMin, ref btVector3 aabbMax, btBroadphaseAabbCallback& callback);

		btOverlappingPairCache* getOverlappingPairCache()
		{
			return m_pairCache;
		}
		btOverlappingPairCache* getOverlappingPairCache()
		{
			return m_pairCache;
		}

		bool testAabbOverlap( btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1 );


		///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
		///will add some transform later
		virtual void getBroadphaseAabb( ref btVector3 aabbMin, ref btVector3 aabbMax )
		{
			aabbMin.setValue( -BT_LARGE_FLOAT, -BT_LARGE_FLOAT, -BT_LARGE_FLOAT );
			aabbMax.setValue( BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT );
		}

		virtual void printStats()
		{
			//		Console.WriteLine("btSimpleBroadphase.h\n");
			//		Console.WriteLine("numHandles = %d, maxHandles = %d\n",m_numHandles,m_maxHandles);
		}
	};



}

