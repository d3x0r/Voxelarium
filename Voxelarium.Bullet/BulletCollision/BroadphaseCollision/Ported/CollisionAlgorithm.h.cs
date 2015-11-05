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

using Bullet.Collision.Dispatch;
using Bullet.Collision.NarrowPhase;
using Bullet.LinearMath;
using Bullet.Types;

namespace Bullet.Collision.BroadPhase
{
	public class btManifoldArray : btList<btPersistentManifold>
	{
	}

	public struct btCollisionAlgorithmConstructionInfo
	{
		internal btDispatcher m_dispatcher1;
		public btPersistentManifold m_manifold;
		internal btCollisionAlgorithmConstructionInfo( btDispatcher dispatcher, int temp )
		{
			m_dispatcher1 = dispatcher;
			m_manifold = null;
		}

	};


	///btCollisionAlgorithm is an collision interface that is compatible with the Broadphase and btDispatcher.
	///It is persistent over frames
	public abstract class btCollisionAlgorithm
	{
		protected btDispatcher m_dispatcher;

		//	protected int	getDispatcherId();
		public btCollisionAlgorithm() { }

		internal void Initialize( btCollisionAlgorithmConstructionInfo ci)
		{
			m_dispatcher = ci.m_dispatcher1;
		}
		internal abstract void Cleanup();
		internal abstract void processCollision( btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap, btDispatcherInfo dispatchInfo, btManifoldResult resultOut );

		internal abstract double calculateTimeOfImpact( btCollisionObject body0, btCollisionObject body1, btDispatcherInfo dispatchInfo, btManifoldResult resultOut );

		internal abstract void getAllContactManifolds( btManifoldArray	manifoldArray);
	};

}
