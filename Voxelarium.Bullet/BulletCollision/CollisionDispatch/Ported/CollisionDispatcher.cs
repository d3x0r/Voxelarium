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
using Bullet.Collision.NarrowPhase;
using Bullet.Types;
using System.Diagnostics;
using Bullet.LinearMath;
using System;

namespace Bullet.Collision.Dispatch
{


	///user can override this nearcallback for collision filtering and more finegrained control over collision detection
	delegate void btNearCallback( btBroadphasePair collisionPair, btCollisionDispatcher dispatcher, btDispatcherInfo dispatchInfo );


	///btCollisionDispatcher supports algorithms that handle ConvexConvex and ConvexConcave collision pairs.
	///Time of Impact, Closest Points and Penetration Depth.
	internal partial class btCollisionDispatcher : btDispatcher
	{
		static int gNumManifold = 0;

		protected DispatcherFlags m_dispatcherFlags;

		protected btList<btPersistentManifold> m_manifoldsPtr = new btList<btPersistentManifold>();

		protected btManifoldResult m_defaultManifoldResult = new btManifoldResult();

		protected btNearCallback m_nearCallback;

		//protected IList m_collisionAlgorithmPoolAllocator;

		//protected IList m_persistentManifoldPoolAllocator;

		protected btCollisionAlgorithmCreateFunc[,] m_doubleDispatch
			= new btCollisionAlgorithmCreateFunc[(int)BroadphaseNativeTypes.MAX_BROADPHASE_COLLISION_TYPES, (int)BroadphaseNativeTypes.MAX_BROADPHASE_COLLISION_TYPES];

		protected btCollisionConfiguration m_collisionConfiguration;


		public enum DispatcherFlags
		{
			CD_STATIC_STATIC_REPORTED = 1,
			CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD = 2,
			CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION = 4
		};

		public DispatcherFlags getDispatcherFlags()
		{
			return m_dispatcherFlags;
		}

		public void setDispatcherFlags( DispatcherFlags flags )
		{
			m_dispatcherFlags = flags;
		}

		///registerCollisionCreateFunc allows registration of custom/alternative collision create functions
		//void	registerCollisionCreateFunc(int proxyType0,int proxyType1, btCollisionAlgorithmCreateFunc* createFunc);

		internal override int getNumManifolds()
		{
			return m_manifoldsPtr.Count;
		}

		internal override btPersistentManifold[] getInternalManifoldPointer()
		{
			return m_manifoldsPtr.Count != 0 ? m_manifoldsPtr.InternalArray : null;
		}

		internal override btPersistentManifold getManifoldByIndexInternal( int index )
		{
			return m_manifoldsPtr[index];
		}

		void setNearCallback( btNearCallback nearCallback )
		{
			m_nearCallback = nearCallback;
		}

		btNearCallback getNearCallback()
		{
			return m_nearCallback;
		}

		//by default, Bullet will use this near callback
		public btCollisionConfiguration getCollisionConfiguration()
		{
			return m_collisionConfiguration;
		}

		void setCollisionConfiguration( btCollisionConfiguration config )
		{
			m_collisionConfiguration = config;
		}

		internal btCollisionDispatcher( btCollisionConfiguration collisionConfiguration )
		{
			int i;
			m_dispatcherFlags = DispatcherFlags.CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD;
			m_collisionConfiguration = collisionConfiguration;

			setNearCallback( defaultNearCallback );

			//m_collisionAlgorithmPoolAllocator = collisionConfiguration.getCollisionAlgorithmPool();

			//m_persistentManifoldPoolAllocator = collisionConfiguration.getPersistentManifoldPool();

			for( i = 0; i < (int)BroadphaseNativeTypes.MAX_BROADPHASE_COLLISION_TYPES; i++ )
			{
				for( int j = 0; j < (int)BroadphaseNativeTypes.MAX_BROADPHASE_COLLISION_TYPES; j++ )
				{
					m_doubleDispatch[i, j] = m_collisionConfiguration.getCollisionAlgorithmCreateFunc( (BroadphaseNativeTypes)i, (BroadphaseNativeTypes)j );
					Debug.Assert( m_doubleDispatch[i, j] != null );
				}
			}
		}


		void registerCollisionCreateFunc( int proxyType0, int proxyType1, btCollisionAlgorithmCreateFunc createFunc )
		{
			m_doubleDispatch[proxyType0, proxyType1] = createFunc;
		}

		internal override btPersistentManifold getNewManifold( btCollisionObject body0, btCollisionObject body1 )
		{
			gNumManifold++;

			//Debug.Assert(gNumManifold < 65535);
			//optional relative contact breaking threshold, turned on by default (use setDispatcherFlags to switch off feature for improved performance)
			double contactBreakingThreshold = ( m_dispatcherFlags & DispatcherFlags.CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD ) != 0 ?
				btScalar.btMin( body0.getCollisionShape().getContactBreakingThreshold( btPersistentManifold.gContactBreakingThreshold ), body1.getCollisionShape().getContactBreakingThreshold( btPersistentManifold.gContactBreakingThreshold ) )
				: btPersistentManifold.gContactBreakingThreshold;

			double contactProcessingThreshold = btScalar.btMin( body0.getContactProcessingThreshold(), body1.getContactProcessingThreshold() );

			btPersistentManifold manifold = BulletGlobals.PersistentManifoldPool.Get();
			manifold.Initialize( body0, body1, 0, contactBreakingThreshold, contactProcessingThreshold );
			manifold.m_index1a = m_manifoldsPtr.Count;
			btScalar.Dbg( "add a manifold (getNewManifold)" );
			m_manifoldsPtr.Add( manifold );

			return manifold;
		}

		internal override void clearManifold( btPersistentManifold manifold )
		{
			manifold.clearManifold();
		}


		internal override void releaseManifold( btPersistentManifold manifold )
		{
			gNumManifold--;
			//Console.WriteLine("releaseManifold: gNumManifold %d\n",gNumManifold);
			clearManifold( manifold );
			btScalar.Dbg( "Remove Manifold from dispatcher" );
			m_manifoldsPtr.Remove( manifold );
			BulletGlobals.PersistentManifoldPool.Free( manifold );
		}



		internal override btCollisionAlgorithm findAlgorithm( btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap, btPersistentManifold sharedManifold = null )
		{
			btCollisionAlgorithmConstructionInfo ci;

			ci.m_dispatcher1 = this;
			ci.m_manifold = sharedManifold;
			btCollisionAlgorithm algo = m_doubleDispatch[(int)body0Wrap.getCollisionShape().getShapeType(), (int)body1Wrap.getCollisionShape().getShapeType()].CreateCollisionAlgorithm( ci, body0Wrap, body1Wrap );

			return algo;
		}




		internal override bool needsResponse( btCollisionObject body0, btCollisionObject body1 )
		{
			//here you can do filtering
			bool hasResponse =
				( body0.hasContactResponse() && body1.hasContactResponse() );
			//no response between two static/kinematic bodies:
			hasResponse = hasResponse &&
				( ( !body0.isStaticOrKinematicObject() ) || ( !body1.isStaticOrKinematicObject() ) );
			return hasResponse;
		}

		internal override bool needsCollision( btCollisionObject body0, btCollisionObject body1 )
		{
			Debug.Assert( body0 != null );
			Debug.Assert( body1 != null );

			bool needsCollision = true;

#if DEBUG
			if( ( m_dispatcherFlags & DispatcherFlags.CD_STATIC_STATIC_REPORTED ) == 0 )
			{
				//broadphase filtering already deals with this
				if( body0.isStaticOrKinematicObject() && body1.isStaticOrKinematicObject() )
				{
					m_dispatcherFlags |= DispatcherFlags.CD_STATIC_STATIC_REPORTED;
					Console.WriteLine( "warning needsCollision: static-static collision!\n" );
				}
			}
#endif //BT_DEBUG

			if( ( !body0.isActive() ) && ( !body1.isActive() ) )
				needsCollision = false;
			else if( ( !body0.checkCollideWith( body1 ) ) || ( !body1.checkCollideWith( body0 ) ) )
				needsCollision = false;

			return needsCollision;

		}



		///interface for iterating all overlapping collision pairs, no matter how those pairs are stored (array, set, map etc)
		///this is useful for the collision dispatcher.
		internal struct btCollisionPairCallback : btOverlapCallback
		{
			btDispatcherInfo m_dispatchInfo;
			btCollisionDispatcher m_dispatcher;

			internal btCollisionPairCallback( btDispatcherInfo dispatchInfo, btCollisionDispatcher dispatcher )
			{
				m_dispatchInfo = ( dispatchInfo );
				m_dispatcher = ( dispatcher );
			}

			public bool processOverlap( btBroadphasePair pair )
			{
				m_dispatcher.m_nearCallback( pair, m_dispatcher, m_dispatchInfo );
				return false;
			}
		};



		internal override void dispatchAllCollisionPairs( btOverlappingPairCache pairCache, btDispatcherInfo dispatchInfo, btDispatcher dispatcher )
		{
			//m_blockedForChanges = true;
			btCollisionPairCallback collisionCallback = new btCollisionPairCallback( dispatchInfo, this );
			pairCache.processAllOverlappingPairs( collisionCallback, dispatcher );
			//m_blockedForChanges = false;
		}




		//by default, Bullet will use this near callback
		void defaultNearCallback( btBroadphasePair collisionPair, btCollisionDispatcher dispatcher, btDispatcherInfo dispatchInfo )
		{
			btCollisionObject colObj0 = (btCollisionObject)collisionPair.m_pProxy0.m_clientObject;
			btCollisionObject colObj1 = (btCollisionObject)collisionPair.m_pProxy1.m_clientObject;

			if( dispatcher.needsCollision( colObj0, colObj1 ) )
			{
				btCollisionObjectWrapper obj0Wrap = BulletGlobals.CollisionObjectWrapperPool.Get();
				obj0Wrap.Initialize( null, colObj0.getCollisionShape(), colObj0, colObj0.getWorldTransform(), -1, -1 );
				btCollisionObjectWrapper obj1Wrap = BulletGlobals.CollisionObjectWrapperPool.Get();
				obj1Wrap.Initialize( null, colObj1.getCollisionShape(), colObj1, colObj1.getWorldTransform(), -1, -1 );


				//dispatcher will keep algorithms persistent in the collision pair
				if( collisionPair.m_algorithm == null )
				{
					collisionPair.m_algorithm = dispatcher.findAlgorithm( obj0Wrap, obj1Wrap );
				}

				if( collisionPair.m_algorithm != null )
				{
					btManifoldResult contactPointResult = BulletGlobals.ManifoldResultPool.Get();
					contactPointResult.Initialize( obj0Wrap, obj1Wrap );

					if( dispatchInfo.m_dispatchFunc == btDispatcherInfo.DispatchFunc.DISPATCH_DISCRETE )
					{
						//discrete collision detection query

						collisionPair.m_algorithm.processCollision( obj0Wrap, obj1Wrap, dispatchInfo, contactPointResult );
					}
					else
					{
						//continuous collision detection query, time of impact (toi)
						double toi = collisionPair.m_algorithm.calculateTimeOfImpact( colObj0, colObj1, dispatchInfo, contactPointResult );
						if( dispatchInfo.m_timeOfImpact > toi )
							dispatchInfo.m_timeOfImpact = toi;
					}
					BulletGlobals.ManifoldResultPool.Free( contactPointResult );
				}
				BulletGlobals.CollisionObjectWrapperPool.Free( obj0Wrap );
				BulletGlobals.CollisionObjectWrapperPool.Free( obj1Wrap );
			}

		}

		object allocateCollisionAlgorithm( int size )
		{
			return null;
		}

		internal override void freeCollisionAlgorithm( btCollisionAlgorithm ptr )
		{
			ptr.Cleanup();
		}

	};

}