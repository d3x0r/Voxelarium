#define USE_PERSISTENT_CONTACTS
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

using Bullet.Collision.BroadPhase;
using Bullet.Collision.NarrowPhase;
using Bullet.Collision.Shapes;
using Bullet.LinearMath;

namespace Bullet.Collision.Dispatch
{
	///box-box collision detection
	internal class btBoxBoxCollisionAlgorithm : btActivatingCollisionAlgorithm
	{
		bool m_ownManifold;
		btPersistentManifold m_manifoldPtr;

		public btBoxBoxCollisionAlgorithm() { }

		new public void Initialize( btCollisionAlgorithmConstructionInfo ci )
		{
			base.Initialize( ci );
		}

		/*
	virtual void processCollision (btCollisionObjectWrapper* body0Wrap,btCollisionObjectWrapper* body1Wrap,btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	virtual double calculateTimeOfImpact(btCollisionObject body0,btCollisionObject body1,btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	btBoxBoxCollisionAlgorithm(btPersistentManifold* mf,btCollisionAlgorithmConstructionInfo& ci,btCollisionObjectWrapper* body0Wrap,btCollisionObjectWrapper* body1Wrap);

	virtual ~btBoxBoxCollisionAlgorithm();
	*/
		internal override void getAllContactManifolds( btManifoldArray manifoldArray )
		{
			if( m_manifoldPtr != null && m_ownManifold )
			{
				manifoldArray.Add( m_manifoldPtr );
			}
		}


		void Initialize( btPersistentManifold mf, btCollisionAlgorithmConstructionInfo ci, btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap )
		{
			base.Initialize( ci, body0Wrap, body1Wrap );
			m_ownManifold = ( false );
			m_manifoldPtr = ( mf );
			if( m_manifoldPtr == null && m_dispatcher.needsCollision( body0Wrap.getCollisionObject(), body1Wrap.getCollisionObject() ) )
			{
				m_manifoldPtr = m_dispatcher.getNewManifold( body0Wrap.getCollisionObject(), body1Wrap.getCollisionObject() );
				m_ownManifold = true;
			}
		}

		internal override void Cleanup()
		{
			if( m_ownManifold )
			{
				if( m_manifoldPtr != null )
					m_dispatcher.releaseManifold( m_manifoldPtr );
			}
		}

		internal override void processCollision( btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap, btDispatcherInfo dispatchInfo, btManifoldResult resultOut )
		{
			if( m_manifoldPtr == null )
				return;


			btBoxShape box0 = (btBoxShape)body0Wrap.getCollisionShape();
			btBoxShape box1 = (btBoxShape)body1Wrap.getCollisionShape();



			/// report a contact. internally this will be kept persistent, and contact reduction is done
			resultOut.setPersistentManifold( m_manifoldPtr );
#if !USE_PERSISTENT_CONTACTS
	m_manifoldPtr.clearManifold();
#endif //USE_PERSISTENT_CONTACTS

			btDiscreteCollisionDetectorInterface.ClosestPointInput input = new btDiscreteCollisionDetectorInterface.ClosestPointInput();
			input.m_maximumDistanceSquared = btScalar.BT_LARGE_FLOAT;
			input.m_transformA = body0Wrap.m_worldTransform.T;
			input.m_transformB = body1Wrap.m_worldTransform.T;

			//btBoxBoxDetector detector = BulletGlobals.
			//	new btBoxBoxDetectors( box0, box1 );
			btBoxBoxDetector.getClosestPoints( box0, box1, input, resultOut, dispatchInfo.m_debugDraw );

#if USE_PERSISTENT_CONTACTS
			//  refreshContactPoints is only necessary when using persistent contact points. otherwise all points are newly added
			if( m_ownManifold )
			{
				resultOut.refreshContactPoints();
			}
#endif //USE_PERSISTENT_CONTACTS

		}

		internal override double calculateTimeOfImpact( btCollisionObject body0, btCollisionObject body1, btDispatcherInfo dispatchInfo, btManifoldResult resultOut)
		{
			//not yet
			return 1;
		}


		internal class CreateFunc : btCollisionAlgorithmCreateFunc
		{
			internal override btCollisionAlgorithm CreateCollisionAlgorithm( btCollisionAlgorithmConstructionInfo ci, btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap )
			{
				btBoxBoxCollisionAlgorithm ca = BulletGlobals.BoxBoxCollisionAlgorithmPool.Get();
				ca.Initialize( ci, body0Wrap, body1Wrap );
				return ca;
			}
		};

	};


}
