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

using System;
using Bullet.Collision.BroadPhase;
using Bullet.Collision.NarrowPhase;
using Bullet.Collision.Shapes;
using Bullet.LinearMath;

namespace Bullet.Collision.Dispatch
{

	/// btSphereSphereCollisionAlgorithm  provides sphere-sphere collision detection.
	/// Other features are frame-coherency (persistent data) and collision response.
	/// Also provides the most basic sample for custom/user btCollisionAlgorithm
	internal class btSphereSphereCollisionAlgorithm : btActivatingCollisionAlgorithm
	{
		bool m_ownManifold;
		btPersistentManifold m_manifoldPtr;

		internal override void getAllContactManifolds( btManifoldArray	manifoldArray)
		{
			if( m_manifoldPtr != null && m_ownManifold )
			{
				manifoldArray.Add( m_manifoldPtr );
			}
		}

		//	virtual ~btSphereSphereCollisionAlgorithm();

		internal class CreateFunc : btCollisionAlgorithmCreateFunc
		{
			internal override btCollisionAlgorithm CreateCollisionAlgorithm( btCollisionAlgorithmConstructionInfo ci, btCollisionObjectWrapper col0Wrap, btCollisionObjectWrapper col1Wrap )
			{
				btSphereSphereCollisionAlgorithm ca = BulletGlobals.SphereSphereCollisionAlgorithmPool.Get();
				ca.Initialize( null, ci, col0Wrap, col1Wrap );
				return ca;
			}
		};

		internal override void Cleanup()
		{
			if( m_ownManifold )
			{
				if( m_manifoldPtr != null )
					m_dispatcher.releaseManifold( m_manifoldPtr );
			}
			BulletGlobals.SphereSphereCollisionAlgorithmPool.Free( this );
		}

		public btSphereSphereCollisionAlgorithm() { }

		//btSphereSphereCollisionAlgorithm( btCollisionAlgorithmConstructionInfo ci )
		//		: btActivatingCollisionAlgorithm( ci) { }

		internal void Initialize( btPersistentManifold mf, btCollisionAlgorithmConstructionInfo ci, btCollisionObjectWrapper col0Wrap, btCollisionObjectWrapper col1Wrap )
		{
			base.Initialize( ci, col0Wrap, col1Wrap );
			m_ownManifold = ( false );
			m_manifoldPtr = ( mf );
			if( m_manifoldPtr == null )
			{
				m_manifoldPtr = m_dispatcher.getNewManifold( col0Wrap.m_collisionObject, col1Wrap.m_collisionObject );
				m_ownManifold = true;
			}
		}

		internal override void processCollision( btCollisionObjectWrapper col0Wrap, btCollisionObjectWrapper col1Wrap, btDispatcherInfo dispatchInfo, btManifoldResult resultOut )
		{
			//(void)dispatchInfo;

			if( m_manifoldPtr == null )
				return;

			resultOut.setPersistentManifold( m_manifoldPtr );

			btSphereShape sphere0 = (btSphereShape)col0Wrap.getCollisionShape();
			btSphereShape sphere1 = (btSphereShape)col1Wrap.getCollisionShape();

			btVector3 diff; col0Wrap.m_collisionObject.m_worldTransform.m_origin.Sub( ref col1Wrap.m_collisionObject.m_worldTransform.m_origin, out diff );
			double len = diff.length();
			double radius0 = sphere0.getRadius();
			double radius1 = sphere1.getRadius();

#if CLEAR_MANIFOLD
	m_manifoldPtr.clearManifold(); //don't do this, it disables warmstarting
#endif

			///iff distance positive, don't generate a new contact
			if( len > ( radius0 + radius1 ) )
			{
#if !CLEAR_MANIFOLD
				resultOut.refreshContactPoints();
#endif //CLEAR_MANIFOLD
				return;
			}
			///distance (negative means penetration)
			double dist = len - ( radius0 + radius1 );

			btVector3 normalOnSurfaceB = btVector3.xAxis;
			if( len > btScalar.SIMD_EPSILON )
			{
				normalOnSurfaceB = diff / len;
			}

			///point on A (worldspace)
			///btVector3 pos0 = col0.getWorldTransform().getOrigin() - radius0 * normalOnSurfaceB;
			///point on B (worldspace)
			btVector3 pos1; col1Wrap.m_collisionObject.m_worldTransform.m_origin.AddScale( ref normalOnSurfaceB, radius1, out pos1 );

			/// report a contact. internally this will be kept persistent, and contact reduction is done
			resultOut.addContactPoint( ref normalOnSurfaceB, ref pos1, dist );

#if !CLEAR_MANIFOLD
			resultOut.refreshContactPoints();
#endif //CLEAR_MANIFOLD

		}

		internal override double calculateTimeOfImpact( btCollisionObject col0, btCollisionObject col1, btDispatcherInfo dispatchInfo, btManifoldResult resultOut )
		{
			//not yet
			return btScalar.BT_ONE;
		}

	};

}
