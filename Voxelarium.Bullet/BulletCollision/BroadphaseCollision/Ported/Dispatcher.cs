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

namespace Bullet.Collision.BroadPhase
{

	public class btDispatcherInfo
	{
		public enum DispatchFunc
		{
			DISPATCH_DISCRETE = 1,
			DISPATCH_CONTINUOUS
		};

		btDispatcherInfo()
		{
			m_timeStep = 0;
			m_stepCount = 0;
			m_dispatchFunc = DispatchFunc.DISPATCH_DISCRETE;
			m_timeOfImpact = 1;
			m_useContinuous = true;
			m_debugDraw = null;
			m_enableSatConvex = false;
			m_enableSPU = true;
			m_useEpa = true;
			m_allowedCcdPenetration = 0.04;
			m_useConvexConservativeDistanceUtil = false;
			m_convexConservativeDistanceThreshold = 0.0;

		}
		public double m_timeStep;
		public int m_stepCount;
		public DispatchFunc m_dispatchFunc;
		public double m_timeOfImpact;
		public bool m_useContinuous;
		public btIDebugDraw m_debugDraw;
		public bool m_enableSatConvex;
		public bool m_enableSPU;
		public bool m_useEpa;
		public double m_allowedCcdPenetration;
		public bool m_useConvexConservativeDistanceUtil;
		public double m_convexConservativeDistanceThreshold;
	};

	///The btDispatcher interface class can be used in combination with broadphase to dispatch calculations for overlapping pairs.
	///For example for pairwise collision detection, calculating contact points stored in btPersistentManifold or user callbacks (game logic).
	internal abstract class btDispatcher
	{
		internal abstract btCollisionAlgorithm findAlgorithm( btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap, btPersistentManifold sharedManifold );
		internal abstract btPersistentManifold getNewManifold( btCollisionObject b0, btCollisionObject b1 );
		internal abstract void releaseManifold( btPersistentManifold manifold );
		internal abstract void clearManifold( btPersistentManifold manifold );
		internal abstract bool needsCollision( btCollisionObject body0, btCollisionObject body1 );
		internal abstract bool needsResponse( btCollisionObject body0, btCollisionObject body1 );
		internal abstract void dispatchAllCollisionPairs( btOverlappingPairCache pairCache, btDispatcherInfo dispatchInfo, btDispatcher dispatcher );
		internal abstract int getNumManifolds();
		internal abstract btPersistentManifold getManifoldByIndexInternal( int index );
		internal abstract btPersistentManifold[] getInternalManifoldPointer();
		//btPoolAllocator getInternalManifoldPool();
		//btCollisionAlgorithm allocateCollisionAlgorithm( int size );
		internal abstract void freeCollisionAlgorithm( btCollisionAlgorithm ptr );
	};

}
