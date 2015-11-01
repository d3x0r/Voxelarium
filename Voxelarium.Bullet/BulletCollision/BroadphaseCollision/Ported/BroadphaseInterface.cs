using Bullet.LinearMath;
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


	public interface btBroadphaseAabbCallback
	{
		bool process( btBroadphaseProxy proxy );
	};


	public abstract class btBroadphaseRayCallback : btBroadphaseAabbCallback
	{
		///added some cached data to accelerate ray-AABB tests
		internal btVector3 m_rayDirectionInverse;
		internal uint[] m_signs = new uint[3];
		internal double m_lambda_max;

		public abstract bool process( btBroadphaseProxy proxy );
	};

	///The btBroadphaseInterface class provides an interface to detect aabb-overlapping object pairs.
	///Some implementations for this broadphase interface include btAxisSweep3, bt32BitAxisSweep3 and btDbvtBroadphase.
	///The actual overlapping pair management, storage, adding and removing of pairs is dealt by the btOverlappingPairCache class.
	internal interface btBroadphaseInterface
	{

		btBroadphaseProxy createProxy( ref btVector3 aabbMin, ref btVector3 aabbMax, BroadphaseNativeTypes shapeType
					, object userPtr
					, btBroadphaseProxy.CollisionFilterGroups collisionFilterGroup
					, btBroadphaseProxy.CollisionFilterGroups collisionFilterMask
					, btDispatcher dispatcher, object multiSapProxy );
		void destroyProxy( btBroadphaseProxy proxy, btDispatcher dispatcher );
		void setAabb( btBroadphaseProxy proxy, ref btVector3 aabbMin, ref btVector3 aabbMax, btDispatcher dispatcher );
		void getAabb( btBroadphaseProxy proxy, ref btVector3 aabbMin, ref btVector3 aabbMax );

		void rayTest( ref btVector3 rayFrom, ref btVector3 rayTo, btBroadphaseRayCallback rayCallback, ref btVector3 aabbMin, ref btVector3 aabbMax );
		void rayTest( ref btVector3 rayFrom, ref btVector3 rayTo, btBroadphaseRayCallback rayCallback );

		void aabbTest( ref btVector3 aabbMin, ref btVector3 aabbMax, btBroadphaseAabbCallback callback );

		///calculateOverlappingPairs is optional: incremental algorithms (sweep and prune) might do it during the set aabb
		void calculateOverlappingPairs( btDispatcher dispatcher );

		btOverlappingPairCache getOverlappingPairCache();

		///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
		///will add some transform later
		void getBroadphaseAabb( out btVector3 aabbMin, out btVector3 aabbMax );

		///reset broadphase internal structures, to ensure determinism/reproducability
		void resetPool( btDispatcher dispatcher );// { (void) dispatcher; };

		void printStats();

	};

	internal abstract class btBroadphaseDefault : btBroadphaseInterface
	{
		public abstract void aabbTest( ref btVector3 aabbMin, ref btVector3 aabbMax, btBroadphaseAabbCallback callback );
		public abstract void calculateOverlappingPairs( btDispatcher dispatcher );
		public abstract btBroadphaseProxy createProxy( ref btVector3 aabbMin, ref btVector3 aabbMax, BroadphaseNativeTypes shapeType, object userPtr, btBroadphaseProxy.CollisionFilterGroups collisionFilterGroup, btBroadphaseProxy.CollisionFilterGroups collisionFilterMask, btDispatcher dispatcher, object multiSapProxy );
		public abstract void destroyProxy( btBroadphaseProxy proxy, btDispatcher dispatcher );
		public abstract void getAabb( btBroadphaseProxy proxy, ref btVector3 aabbMin, ref btVector3 aabbMax );
		public abstract void getBroadphaseAabb( out btVector3 aabbMin, out btVector3 aabbMax );
		public abstract btOverlappingPairCache getOverlappingPairCache();
		public abstract void printStats();
		public virtual void rayTest( ref btVector3 rayFrom, ref btVector3 rayTo, btBroadphaseRayCallback rayCallback )
		{
			rayTest( ref rayFrom, ref rayTo, rayCallback, ref btVector3.Min, ref btVector3.Max );
		}
		public abstract void rayTest( ref btVector3 rayFrom, ref btVector3 rayTo, btBroadphaseRayCallback rayCallback, ref btVector3 aabbMin, ref btVector3 aabbMax );
		public abstract void resetPool( btDispatcher dispatcher );
		public abstract void setAabb( btBroadphaseProxy proxy, ref btVector3 aabbMin, ref btVector3 aabbMax, btDispatcher dispatcher );
	}
}