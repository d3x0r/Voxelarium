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
	class btSphereTriangleCollisionAlgorithm : btActivatingCollisionAlgorithm
	{
		bool m_ownManifold;
		btPersistentManifold m_manifoldPtr;
		internal bool m_swapped;

		internal override void getAllContactManifolds( btManifoldArray manifoldArray )
		{
			if( m_manifoldPtr != null && m_ownManifold )
			{
				manifoldArray.Add( m_manifoldPtr );
			}
		}


		internal class CreateFunc : btCollisionAlgorithmCreateFunc
		{
			internal override btCollisionAlgorithm CreateCollisionAlgorithm( btCollisionAlgorithmConstructionInfo ci, btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap )
			{
				btSphereTriangleCollisionAlgorithm ca = BulletGlobals.SphereTriangleCollisionAlgorithmPool.Get();
				ca.Initialize( ci.m_manifold, ci, body0Wrap, body1Wrap, m_swapped );
				return ca;
			}
		};

		public btSphereTriangleCollisionAlgorithm() { }

		internal void Initialize( btPersistentManifold mf, btCollisionAlgorithmConstructionInfo ci, btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap, bool swapped )
		{
			base.Initialize( ci, body0Wrap, body1Wrap );
			m_ownManifold = ( false );
			m_manifoldPtr = ( mf );
			m_swapped = ( swapped );
			if( m_manifoldPtr == null )
			{
				m_manifoldPtr = m_dispatcher.getNewManifold( body0Wrap.m_collisionObject, body1Wrap.m_collisionObject );
				m_ownManifold = true;
			}
		}

		void cleanManifold()
		{
			if( m_ownManifold )
			{
				if( m_manifoldPtr != null )
					m_dispatcher.releaseManifold( m_manifoldPtr );
			}
		}

		internal override void Cleanup()
		{
			cleanManifold();
			BulletGlobals.SphereTriangleCollisionAlgorithmPool.Free( this );
		}


		internal override void processCollision( btCollisionObjectWrapper col0Wrap
			, ref btTransform body0Transform
			, btCollisionObjectWrapper col1Wrap
			, ref btTransform body1Transform
			, btDispatcherInfo dispatchInfo, btManifoldResult resultOut )
		{
			if( m_manifoldPtr == null )
				return;

			btCollisionObjectWrapper sphereObjWrap = m_swapped ? col1Wrap : col0Wrap;
			btCollisionObjectWrapper triObjWrap = m_swapped ? col0Wrap : col1Wrap;

			btSphereShape sphere = (btSphereShape)sphereObjWrap.getCollisionShape();
			btTriangleShape triangle = (btTriangleShape)triObjWrap.getCollisionShape();

			/// report a contact. internally this will be kept persistent, and contact reduction is done
			resultOut.setPersistentManifold( m_manifoldPtr );
			SphereTriangleDetector detector = BulletGlobals.SphereTriangleDetectorPool.Get();
			detector.Initialize( sphere, triangle, m_manifoldPtr.getContactBreakingThreshold());

			btDiscreteCollisionDetectorInterface.ClosestPointInput input = BulletGlobals.ClosestPointInputPool.Get();

			input.m_maximumDistanceSquared = btScalar.BT_LARGE_FLOAT;///@todo: tighter bounds
			if( m_swapped )
			{
				input.m_transformA = body1Transform;
				input.m_transformB = body0Transform;
			}
			else
			{
				input.m_transformA = body0Transform;
				input.m_transformB = body1Transform;
			}

			bool swapResults = m_swapped;

			detector.getClosestPoints( input, resultOut, dispatchInfo.m_debugDraw, swapResults );

			BulletGlobals.ClosestPointInputPool.Free( input );
			BulletGlobals.SphereTriangleDetectorPool.Free( detector );
			if( m_ownManifold )
				resultOut.refreshContactPoints();

		}

		internal override double calculateTimeOfImpact( btCollisionObject col0, btCollisionObject col1, btDispatcherInfo dispatchInfo, btManifoldResult resultOut )
		{
			//not yet
			return btScalar.BT_ONE;
		}

	};

}
