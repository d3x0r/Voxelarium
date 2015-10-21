/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

using Bullet.Types;

namespace Bullet.Collision.Dispatch
{


	///The btGhostObject can keep track of all objects that are overlapping
	///By default, this overlap is based on the AABB
	///This is useful for creating a character controller, collision sensors/triggers, explosions etc.
	///We plan on adding rayTest and other queries for the btGhostObject
	internal class btGhostObject : btCollisionObject
	{
		

		protected btList<btCollisionObject> m_overlappingObjects;

		public:

	btGhostObject();

		virtual ~btGhostObject();

		void convexSweepTest( string lass btConvexShape* castShape, ref btTransform convexFromWorld, ref btTransform convexToWorld, btCollisionWorld::ConvexResultCallback& resultCallback, double allowedCcdPenetration = 0 );

		void rayTest( ref btVector3 rayFromWorld, ref btVector3 rayToWorld, btCollisionWorld::RayResultCallback& resultCallback);

		///this method is mainly for expert/internal use only.
		virtual void addOverlappingObjectInternal( btBroadphaseProxy* otherProxy, btBroadphaseProxy* thisProxy = 0 );
		///this method is mainly for expert/internal use only.
		virtual void removeOverlappingObjectInternal( btBroadphaseProxy* otherProxy, btDispatcher* dispatcher, btBroadphaseProxy* thisProxy = 0 );

		int getNumOverlappingObjects()
		{
			return m_overlappingObjects.Count;
		}

		btCollisionObject getOverlappingObject( int index )
		{
			return m_overlappingObjects[index];
		}

		btList<btCollisionObject>	getOverlappingPairs()
		{
			return m_overlappingObjects;
		}

		//
		// internal cast
		//

		static btGhostObject	upcast( btCollisionObject colObj)
		{
			if( colObj.getInternalType() == CO_GHOST_OBJECT )
				return (btGhostObject *)colObj;
			return 0;
		}
		static btGhostObject* upcast( btCollisionObject colObj )
		{
			if( colObj.getInternalType() == CO_GHOST_OBJECT )
				return (btGhostObject*)colObj;
			return 0;
		}

	};

	class btPairCachingGhostObject : btGhostObject
	{
		btHashedOverlappingPairCache* m_hashPairCache;

		public:

	btPairCachingGhostObject();

		virtual ~btPairCachingGhostObject();

		///this method is mainly for expert/internal use only.
		virtual void addOverlappingObjectInternal( btBroadphaseProxy* otherProxy, btBroadphaseProxy* thisProxy = 0 );

		virtual void removeOverlappingObjectInternal( btBroadphaseProxy* otherProxy, btDispatcher* dispatcher, btBroadphaseProxy* thisProxy = 0 );

		btHashedOverlappingPairCache* getOverlappingPairCache()
		{
			return m_hashPairCache;
		}

	};



	///The btGhostPairCallback interfaces and forwards adding and removal of overlapping pairs from the btBroadphaseInterface to btGhostObject.
	class btGhostPairCallback : btOverlappingPairCallback
	{

		public:
	btGhostPairCallback()
		{
		}

		virtual ~btGhostPairCallback()
		{

		}

		virtual btBroadphasePair* addOverlappingPair( btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1 )
		{
			btCollisionObject colObj0 = (btCollisionObject)proxy0.m_clientObject;
			btCollisionObject colObj1 = (btCollisionObject)proxy1.m_clientObject;
			btGhostObject* ghost0 = btGhostObject::upcast( colObj0 );
			btGhostObject* ghost1 = btGhostObject::upcast( colObj1 );
			if( ghost0 )
				ghost0.addOverlappingObjectInternal( proxy1, proxy0 );
			if( ghost1 )
				ghost1.addOverlappingObjectInternal( proxy0, proxy1 );
			return 0;
		}

		virtual object removeOverlappingPair( btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1, btDispatcher* dispatcher )
		{
			btCollisionObject colObj0 = (btCollisionObject)proxy0.m_clientObject;
			btCollisionObject colObj1 = (btCollisionObject)proxy1.m_clientObject;
			btGhostObject* ghost0 = btGhostObject::upcast( colObj0 );
			btGhostObject* ghost1 = btGhostObject::upcast( colObj1 );
			if( ghost0 )
				ghost0.removeOverlappingObjectInternal( proxy1, dispatcher, proxy0 );
			if( ghost1 )
				ghost1.removeOverlappingObjectInternal( proxy0, dispatcher, proxy1 );
			return 0;
		}

		virtual void removeOverlappingPairsContainingProxy( btBroadphaseProxy* /*proxy0*/, btDispatcher* /*dispatcher*/)
		{
			Debug.Assert( false );
			//need to keep track of all ghost objects and call them here
			//m_hashPairCache.removeOverlappingPairsContainingProxy(proxy0,dispatcher);
		}



	};

}