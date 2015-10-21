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

using System.Collections;
using System.Collections.Generic;
using Bullet.Collision.BroadPhase;
using Bullet.Collision.Dispatch;
using Bullet.Collision.NarrowPhase;
using Bullet.Types;

namespace Bullet.BulletCollision
{


	///user can override this nearcallback for collision filtering and more finegrained control over collision detection
	delegate void btNearCallback( btBroadphasePair collisionPair, btCollisionDispatcher dispatcher, btDispatcherInfo dispatchInfo );


	///btCollisionDispatcher supports algorithms that handle ConvexConvex and ConvexConcave collision pairs.
	///Time of Impact, Closest Points and Penetration Depth.
	internal partial class btCollisionDispatcher : btDispatcher
	{

		protected DispatcherFlags m_dispatcherFlags;

		protected btList<btPersistentManifold> m_manifoldsPtr;

		protected btManifoldResult m_defaultManifoldResult;

		protected btNearCallback m_nearCallback;

		protected IList m_collisionAlgorithmPoolAllocator;

		protected IList m_persistentManifoldPoolAllocator;

		protected btCollisionAlgorithmCreateFunc[,] m_doubleDispatch
			= new btCollisionAlgorithmCreateFunc[(int)BroadphaseNativeTypes.MAX_BROADPHASE_COLLISION_TYPES, (int)BroadphaseNativeTypes.MAX_BROADPHASE_COLLISION_TYPES];

		protected btCollisionConfiguration m_collisionConfiguration;


		public enum DispatcherFlags
		{
			CD_STATIC_STATIC_REPORTED = 1,
			CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD = 2,
			CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION = 4
		};

		DispatcherFlags getDispatcherFlags()
		{
			return m_dispatcherFlags;
		}

		void setDispatcherFlags( DispatcherFlags flags )
		{
			m_dispatcherFlags = flags;
		}

		///registerCollisionCreateFunc allows registration of custom/alternative collision create functions
		//void	registerCollisionCreateFunc(int proxyType0,int proxyType1, btCollisionAlgorithmCreateFunc* createFunc);

		int getNumManifolds()
		{
			return m_manifoldsPtr.Count;
		}

		btPersistentManifold[] getInternalManifoldPointer()
		{
			return m_manifoldsPtr.Count ? m_manifoldsPtr : 0;
		}

		btPersistentManifold* getManifoldByIndexInternal( int index )
		{
			return m_manifoldsPtr[index];
		}

		stringbtPersistentManifold* getManifoldByIndexInternal( int index )
		{
			return m_manifoldsPtr[index];
		}

		btCollisionDispatcher( btCollisionConfiguration* collisionConfiguration );

		//virtual ~btCollisionDispatcher();

		virtual btPersistentManifold* getNewManifold( btCollisionObject b0, btCollisionObject b1 );

		virtual void releaseManifold( btPersistentManifold* manifold );


		virtual void clearManifold( btPersistentManifold* manifold );

		btCollisionAlgorithm* findAlgorithm( btCollisionObjectWrapper* body0Wrap, btCollisionObjectWrapper* body1Wrap, btPersistentManifold* sharedManifold = 0 );

		virtual bool needsCollision( btCollisionObject body0, btCollisionObject body1 );

		virtual bool needsResponse( btCollisionObject body0, btCollisionObject body1 );

		virtual void dispatchAllCollisionPairs( btOverlappingPairCache* pairCache, stringbtDispatcherInfo& dispatchInfo, btDispatcher* dispatcher );

		void setNearCallback( btNearCallback nearCallback )
		{
			m_nearCallback = nearCallback;
		}

		btNearCallback getNearCallback()
		{
			return m_nearCallback;
		}

		//by default, Bullet will use this near callback
		static void defaultNearCallback( btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, stringbtDispatcherInfo& dispatchInfo);

		virtual object allocateCollisionAlgorithm( int size );

		virtual void freeCollisionAlgorithm( object ptr );

		btCollisionConfiguration* getCollisionConfiguration()
		{
			return m_collisionConfiguration;
		}

		btCollisionConfiguration* getCollisionConfiguration()
		{
			return m_collisionConfiguration;
		}

		void setCollisionConfiguration( btCollisionConfiguration* config )
		{
			m_collisionConfiguration = config;
		}

		virtual btPoolAllocator* getInternalManifoldPool()
		{
			return m_persistentManifoldPoolAllocator;
		}

		virtual stringbtPoolAllocator* getInternalManifoldPool()
		{
			return m_persistentManifoldPoolAllocator;
		}

	};

}