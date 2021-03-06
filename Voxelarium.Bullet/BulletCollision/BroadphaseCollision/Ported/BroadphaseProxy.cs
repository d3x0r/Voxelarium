using Bullet.Collision.Dispatch;
using Bullet.Dynamics;
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
using System.Collections.Generic;

namespace Bullet.Collision.BroadPhase
{


	/// btDispatcher uses these types
	/// IMPORTANT NOTE:The types are ordered polyhedral, implicit convex and concave
	/// to facilitate type checking
	/// CUSTOM_POLYHEDRAL_SHAPE_TYPE,CUSTOM_CONVEX_SHAPE_TYPE and CUSTOM_CONCAVE_SHAPE_TYPE can be used to extend Bullet without modifying source code
	public enum BroadphaseNativeTypes
	{
		// polyhedral convex shapes
		BOX_SHAPE_PROXYTYPE,
		TRIANGLE_SHAPE_PROXYTYPE,
		TETRAHEDRAL_SHAPE_PROXYTYPE,
		CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE,
		CONVEX_HULL_SHAPE_PROXYTYPE,
		CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE,
		CUSTOM_POLYHEDRAL_SHAPE_TYPE,
		//implicit convex shapes
		IMPLICIT_CONVEX_SHAPES_START_HERE,
		SPHERE_SHAPE_PROXYTYPE,
		MULTI_SPHERE_SHAPE_PROXYTYPE,
		CAPSULE_SHAPE_PROXYTYPE,
		CONE_SHAPE_PROXYTYPE,
		CONVEX_SHAPE_PROXYTYPE,
		CYLINDER_SHAPE_PROXYTYPE,
		UNIFORM_SCALING_SHAPE_PROXYTYPE,
		MINKOWSKI_SUM_SHAPE_PROXYTYPE,
		MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE,
		BOX_2D_SHAPE_PROXYTYPE,
		CONVEX_2D_SHAPE_PROXYTYPE,
		CUSTOM_CONVEX_SHAPE_TYPE,
		//concave shapes
		CONCAVE_SHAPES_START_HERE,
		//keep all the convex shapetype below here, for the check IsConvexShape in broadphase proxy!
		TRIANGLE_MESH_SHAPE_PROXYTYPE,
		SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE,
		///used for demo integration FAST/Swift collision library and Bullet
		FAST_CONCAVE_MESH_PROXYTYPE,
		//terrain
		TERRAIN_SHAPE_PROXYTYPE,
		///Used for GIMPACT Trimesh integration
		GIMPACT_SHAPE_PROXYTYPE,
		///Multimaterial mesh
		MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE,

		EMPTY_SHAPE_PROXYTYPE,
		STATIC_PLANE_PROXYTYPE,
		CUSTOM_CONCAVE_SHAPE_TYPE,
		CONCAVE_SHAPES_END_HERE,

		COMPOUND_SHAPE_PROXYTYPE,

		SOFTBODY_SHAPE_PROXYTYPE,
		HFFLUID_SHAPE_PROXYTYPE,
		HFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE,
		INVALID_SHAPE_PROXYTYPE,

		MAX_BROADPHASE_COLLISION_TYPES
	};


	///The btBroadphaseProxy is the main class that can be used with the Bullet broadphases. 
	///It stores collision shape type information, collision filter information and a client object, typically a btCollisionObject or btRigidBody.
	public class btBroadphaseProxy
	{

		///optional filtering to cull potential collisions
		public enum CollisionFilterGroups
		{
			DefaultFilter = 1,
			StaticFilter = 2,
			KinematicFilter = 4,
			DebrisFilter = 8,
			SensorTrigger = 16,
			CharacterFilter = 32,
			AllFilter = -1 //all bits sets: DefaultFilter | StaticFilter | KinematicFilter | DebrisFilter | SensorTrigger
		};

		//Usually the client btCollisionObject or Rigidbody class
		public btCollisionObject m_clientObject;
		public btBroadphaseProxy.CollisionFilterGroups m_collisionFilterGroup;
		public btBroadphaseProxy.CollisionFilterGroups m_collisionFilterMask;
		public object m_multiSapParentProxy;

		//internal IntPtr m_uniqueId;//m_uniqueId is introduced for paircache. could get rid of this, by calculating the address offset etc.

		internal btVector3 m_aabbMin;
		internal btVector3 m_aabbMax;

		public static int IMPLICIT_CONVEX_SHAPES_START_HERE { get; private set; }

		//used for memory pools
		public btBroadphaseProxy()
		{
		}

		public void Initialize( ref btVector3 aabbMin, ref btVector3 aabbMax
				, btCollisionObject userPtr, CollisionFilterGroups collisionFilterGroup
				, CollisionFilterGroups collisionFilterMask )
		{
			//m_uniqueId = 0;
			m_clientObject = userPtr;
			m_collisionFilterGroup = collisionFilterGroup;
			m_collisionFilterMask = collisionFilterMask;
			m_aabbMax = aabbMax;
			m_aabbMin = aabbMin;
			m_multiSapParentProxy = null;
		}


		 btBroadphaseProxy( ref btVector3 aabbMin, ref btVector3 aabbMax
				, btCollisionObject userPtr, CollisionFilterGroups collisionFilterGroup
				, CollisionFilterGroups collisionFilterMask, object multiSapParentProxy )
		{
			//m_uniqueId = 0;
			m_clientObject = userPtr;
			m_collisionFilterGroup = collisionFilterGroup;
			m_collisionFilterMask = collisionFilterMask;
			m_aabbMax = aabbMax;
			m_aabbMin = aabbMin;
			m_multiSapParentProxy = multiSapParentProxy;
		}



		static public bool isPolyhedral( BroadphaseNativeTypes proxyType )
		{
			return ( proxyType < BroadphaseNativeTypes.IMPLICIT_CONVEX_SHAPES_START_HERE );
		}

		static public bool isConvex( BroadphaseNativeTypes proxyType )
		{
			return ( proxyType < BroadphaseNativeTypes.CONCAVE_SHAPES_START_HERE );
		}

		static public bool isNonMoving( BroadphaseNativeTypes proxyType )
		{
			return ( isConcave( proxyType ) && !( proxyType == BroadphaseNativeTypes.GIMPACT_SHAPE_PROXYTYPE ) );
		}

		static public bool isConcave( BroadphaseNativeTypes proxyType )
		{
			return ( ( proxyType > BroadphaseNativeTypes.CONCAVE_SHAPES_START_HERE ) &&
				( proxyType < BroadphaseNativeTypes.CONCAVE_SHAPES_END_HERE ) );
		}
		static public bool isCompound( BroadphaseNativeTypes proxyType )
		{
			return ( proxyType == BroadphaseNativeTypes.COMPOUND_SHAPE_PROXYTYPE );
		}

		static public bool isSoftBody( BroadphaseNativeTypes proxyType )
		{
			return ( proxyType == BroadphaseNativeTypes.SOFTBODY_SHAPE_PROXYTYPE );
		}

		static public bool isInfinite( BroadphaseNativeTypes proxyType )
		{
			return ( proxyType == BroadphaseNativeTypes.STATIC_PLANE_PROXYTYPE );
		}

		static public bool isConvex2d( BroadphaseNativeTypes proxyType )
		{
			return ( proxyType == BroadphaseNativeTypes.BOX_2D_SHAPE_PROXYTYPE )
						|| ( proxyType == BroadphaseNativeTypes.CONVEX_2D_SHAPE_PROXYTYPE );
		}

	}



	///The btBroadphasePair class contains a pair of aabb-overlapping objects.
	///A btDispatcher can search a btCollisionAlgorithm that performs exact/narrowphase collision detection on the actual collision shapes.
	internal class btBroadphasePair
	{
		internal btBroadphaseProxy m_pProxy0;
		internal btBroadphaseProxy m_pProxy1;

		internal btCollisionAlgorithm m_algorithm;
		internal object m_internalInfo1;
		public btBroadphasePair(  )
		{
		}
        internal btBroadphasePair( ref btBroadphasePair other )
		{
			m_pProxy0 = other.m_pProxy0;
			m_pProxy1 = other.m_pProxy1;
			m_algorithm = other.m_algorithm;
			m_internalInfo1 = other.m_internalInfo1;
		}

		public void Initialize( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 )
		{

			//keep them sorted, so the std::set operations work
			m_pProxy0 = proxy0;
			m_pProxy1 = proxy1;

			m_algorithm = null;
			m_internalInfo1 = 0;

		}
		internal btBroadphasePair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 )
		{
			Initialize( proxy0, proxy1 );
		}
		internal bool Equals( btBroadphasePair a, btBroadphasePair b )
		{
			return ( ( a.m_pProxy0 == b.m_pProxy0 ) && ( a.m_pProxy1 == b.m_pProxy1 ) )
			    || ( ( a.m_pProxy0 == b.m_pProxy1 ) && ( a.m_pProxy1 == b.m_pProxy0 ) )
				;
		}

		internal static bool qsCompare( btBroadphasePair a, btBroadphasePair b )
		{
			int uidA0 = 0;// a.m_pProxy0 != null ? a.m_pProxy0.m_uniqueId : -1;
			int uidB0 = 0;// b.m_pProxy0 != null ? b.m_pProxy0.m_uniqueId : -1;
			int uidA1 = 0;// a.m_pProxy1 != null ? a.m_pProxy1.m_uniqueId : -1;
			int uidB1 = 0;// b.m_pProxy1 != null ? b.m_pProxy1.m_uniqueId : -1;

			return uidA0 > uidB0 ||
			   ( a.m_pProxy0.Equals( b.m_pProxy0 ) && uidA1 > uidB1 ) ||
			   ( a.m_pProxy0 == b.m_pProxy0
			   && a.m_pProxy1 == b.m_pProxy1
			   && a.m_algorithm != b.m_algorithm
			   );
		}

	}//don't use this data, it will be removed in future version.


	/*
	//comparison for set operation, see Solid DT_Encounter
	public bool operator<(btBroadphasePair& a, btBroadphasePair& b) 
	{ 
		return a.m_pProxy0 < b.m_pProxy0 || 
			(a.m_pProxy0 == b.m_pProxy0 && a.m_pProxy1 < b.m_pProxy1); 
	}
	*/
/* this is in broadphase as qaCompare 
	internal class btBroadphasePairSortPredicate : IComparer<btBroadphasePair>
	{
		public int Compare( btBroadphasePair a, btBroadphasePair b )
		{
			int uidA0 = a.m_pProxy0.m_uniqueId;
			int uidB0 = b.m_pProxy0.m_uniqueId;
			int uidA1 = a.m_pProxy1.m_uniqueId;
			int uidB1 = b.m_pProxy1.m_uniqueId;

			if( uidA0 > uidB0 ||
			   ( a.m_pProxy0.Equals( b.m_pProxy0 ) && uidA1 > uidB1 ) ||
			   ( a.m_pProxy0 == b.m_pProxy0
			   && a.m_pProxy1 == b.m_pProxy1
			   //&& a.m_algorithm > b.m_algorithm
			   ) )
				return 0;
			return -1;
		}
	};
	*/

}


