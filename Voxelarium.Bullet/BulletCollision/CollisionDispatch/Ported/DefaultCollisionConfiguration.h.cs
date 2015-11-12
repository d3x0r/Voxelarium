#define REMOVE_OBSOLETE_SOLVER
//#define USE_BUGGY_SPHERE_BOX_ALGORITHM
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

namespace Bullet.Collision.Dispatch
{

	class btDefaultCollisionConstructionInfo
	{
		//public btPoolAllocator* m_persistentManifoldPool;
		//public btPoolAllocator* m_collisionAlgorithmPool;
		//public int m_defaultMaxPersistentManifoldPoolSize;
		//public int m_defaultMaxCollisionAlgorithmPoolSize;
		//public int m_customCollisionAlgorithmMaxElementSize;
		public bool m_useEpaPenetrationAlgorithm;

		public btDefaultCollisionConstructionInfo()
		{
			//m_persistentManifoldPool( 0 );
			//m_collisionAlgorithmPool( 0 );
			//m_defaultMaxPersistentManifoldPoolSize( 4096 );
			//m_defaultMaxCollisionAlgorithmPoolSize( 4096 );
			//m_customCollisionAlgorithmMaxElementSize( 0 );
			m_useEpaPenetrationAlgorithm = ( true );
		}
	};



	///btCollisionConfiguration allows to configure Bullet collision detection
	///stack allocator, pool memory allocators
	///@todo: describe the meaning
	class btDefaultCollisionConfiguration : btCollisionConfiguration
	{
		//int m_persistentManifoldPoolSize;

		//btPoolAllocator* m_persistentManifoldPool;
		//bool m_ownsPersistentManifoldPool;

		//btPoolAllocator* m_collisionAlgorithmPool;
		//bool m_ownsCollisionAlgorithmPool;

		//default simplex/penetration depth solvers

		btVoronoiSimplexSolver m_simplexSolver;
		btConvexPenetrationDepthSolver m_pdSolver;


		//default CreationFunctions, filling the m_doubleDispatch table
		btCollisionAlgorithmCreateFunc m_convexConvexCreateFunc;
		btCollisionAlgorithmCreateFunc m_convexConcaveCreateFunc;
		btCollisionAlgorithmCreateFunc m_swappedConvexConcaveCreateFunc;
		btCollisionAlgorithmCreateFunc m_compoundCreateFunc;
		btCollisionAlgorithmCreateFunc m_compoundCompoundCreateFunc;

		btCollisionAlgorithmCreateFunc m_swappedCompoundCreateFunc;
		btCollisionAlgorithmCreateFunc m_emptyCreateFunc;
		btCollisionAlgorithmCreateFunc m_sphereSphereCF;
#if USE_BUGGY_SPHERE_BOX_ALGORITHM
		btCollisionAlgorithmCreateFunc m_sphereBoxCF;
		btCollisionAlgorithmCreateFunc m_boxSphereCF;
#endif

		btCollisionAlgorithmCreateFunc m_boxBoxCF;
		btCollisionAlgorithmCreateFunc m_sphereTriangleCF;
		btCollisionAlgorithmCreateFunc m_triangleSphereCF;
		btCollisionAlgorithmCreateFunc m_planeConvexCF;
		btCollisionAlgorithmCreateFunc m_convexPlaneCF;



		///memory pools
		/*
		public virtual btPoolAllocator getPersistentManifoldPool()
		{
			return m_persistentManifoldPool;
		}

		public virtual btPoolAllocator getCollisionAlgorithmPool()
		{
			return m_collisionAlgorithmPool;
		}
		*/

		public virtual btVoronoiSimplexSolver getSimplexSolver()
		{
			return m_simplexSolver;
		}


		// pass null to use default construction info
		public btDefaultCollisionConfiguration( btDefaultCollisionConstructionInfo constructionInfo )
		//btDefaultCollisionConfiguration(btStackAlloc*	stackAlloc,btPoolAllocator*	persistentManifoldPool,btPoolAllocator*	collisionAlgorithmPool)
		{
			if( constructionInfo == null )
				constructionInfo = new btDefaultCollisionConstructionInfo();

			m_simplexSolver = BulletGlobals.VoronoiSimplexSolverPool.Get();

#if !REMOVE_OBSOLETE_SOLVER
			if( constructionInfo.m_useEpaPenetrationAlgorithm )
#endif
			{
				m_pdSolver = BulletGlobals.GjkEpaPenetrationDepthSolverPool.Get();
			}
#if !REMOVE_OBSOLETE_SOLVER
			else
			{
				mem = btAlignedAlloc( sizeof( btMinkowskiPenetrationDepthSolver ), 16 );
				m_pdSolver = 
					new btMinkowskiPenetrationDepthSolver;
			}
#endif

			//default CreationFunctions, filling the m_doubleDispatch table
			m_convexConvexCreateFunc = new btConvexConvexAlgorithm.CreateFunc( m_simplexSolver, m_pdSolver );
			m_convexConcaveCreateFunc = new btConvexConcaveCollisionAlgorithm.CreateFunc();
			m_swappedConvexConcaveCreateFunc = new btConvexConcaveCollisionAlgorithm.SwappedCreateFunc();
			m_compoundCreateFunc = new btCompoundCollisionAlgorithm.CreateFunc();

			m_compoundCompoundCreateFunc = new btCompoundCompoundCollisionAlgorithm.CreateFunc();

			m_swappedCompoundCreateFunc = new btCompoundCollisionAlgorithm.SwappedCreateFunc();
			m_emptyCreateFunc = new btEmptyAlgorithm.CreateFunc();

			m_sphereSphereCF = new btSphereSphereCollisionAlgorithm.CreateFunc();
#if USE_BUGGY_SPHERE_BOX_ALGORITHM
			m_sphereBoxCF = new(mem) btSphereBoxCollisionAlgorithm.CreateFunc;
			m_boxSphereCF = new (mem)btSphereBoxCollisionAlgorithm.CreateFunc;
			m_boxSphereCF.m_swapped = true;
#endif //USE_BUGGY_SPHERE_BOX_ALGORITHM

			m_sphereTriangleCF = new btSphereTriangleCollisionAlgorithm.CreateFunc();
			m_triangleSphereCF = new btSphereTriangleCollisionAlgorithm.CreateFunc();
			m_triangleSphereCF.m_swapped = true;

			m_boxBoxCF = new btBoxBoxCollisionAlgorithm.CreateFunc();

			//convex versus plane
			m_convexPlaneCF = new  btConvexPlaneCollisionAlgorithm.CreateFunc();
			m_planeConvexCF = new  btConvexPlaneCollisionAlgorithm.CreateFunc();
			m_planeConvexCF.m_swapped = true;

			///calculate maximum element size, big enough to fit any collision algorithm in the memory pool
			/*
			int maxSize = sizeof( btConvexConvexAlgorithm );
			int maxSize2 = sizeof( btConvexConcaveCollisionAlgorithm );
			int maxSize3 = sizeof( btCompoundCollisionAlgorithm );
			int maxSize4 = sizeof( btCompoundCompoundCollisionAlgorithm );

			int collisionAlgorithmMaxElementSize = btScalar.btMax( maxSize, constructionInfo.m_customCollisionAlgorithmMaxElementSize );
			collisionAlgorithmMaxElementSize = btScalar.btMax( collisionAlgorithmMaxElementSize, maxSize2 );
			collisionAlgorithmMaxElementSize = btScalar.btMax( collisionAlgorithmMaxElementSize, maxSize3 );
			collisionAlgorithmMaxElementSize = btScalar.btMax( collisionAlgorithmMaxElementSize, maxSize4 );
			*/

			/*
			if( constructionInfo.m_persistentManifoldPool )
			{
				m_ownsPersistentManifoldPool = false;
				m_persistentManifoldPool = constructionInfo.m_persistentManifoldPool;
			}
			else
			{
				m_ownsPersistentManifoldPool = true;
				object mem = btAlignedAlloc( sizeof( btPoolAllocator ), 16 );
				m_persistentManifoldPool = new  btPoolAllocator( sizeof( btPersistentManifold ), constructionInfo.m_defaultMaxPersistentManifoldPoolSize );
			}

			if( constructionInfo.m_collisionAlgorithmPool )
			{
				m_ownsCollisionAlgorithmPool = false;
				m_collisionAlgorithmPool = constructionInfo.m_collisionAlgorithmPool;
			}
			else
			{
				m_ownsCollisionAlgorithmPool = true;
				object mem = btAlignedAlloc( sizeof( btPoolAllocator ), 16 );
				m_collisionAlgorithmPool = new btPoolAllocator( collisionAlgorithmMaxElementSize, constructionInfo.m_defaultMaxCollisionAlgorithmPoolSize );
			}
			*/

		}


		internal override btCollisionAlgorithmCreateFunc getCollisionAlgorithmCreateFunc( BroadphaseNativeTypes proxyType0, BroadphaseNativeTypes proxyType1 )
		{

			if( ( proxyType0 == BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE ) && ( proxyType1 == BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE ) )
			{
				return m_sphereSphereCF;
			}
#if USE_BUGGY_SPHERE_BOX_ALGORITHM
	if ((proxyType0 == SPHERE_SHAPE_PROXYTYPE) && (proxyType1==BOX_SHAPE_PROXYTYPE))
	{
		return	m_sphereBoxCF;
	}

	if ((proxyType0 == BOX_SHAPE_PROXYTYPE ) && (proxyType1==SPHERE_SHAPE_PROXYTYPE))
	{
		return	m_boxSphereCF;
	}
#endif //USE_BUGGY_SPHERE_BOX_ALGORITHM


			if( ( proxyType0 == BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE ) && ( proxyType1 == BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE ) )
			{
				return m_sphereTriangleCF;
			}

			if( ( proxyType0 == BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE ) && ( proxyType1 == BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE ) )
			{
				return m_triangleSphereCF;
			}

			if( ( proxyType0 == BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE ) && ( proxyType1 == BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE ) )
			{
				return m_boxBoxCF;
			}

			if( btBroadphaseProxy.isConvex( proxyType0 ) && ( proxyType1 == BroadphaseNativeTypes.STATIC_PLANE_PROXYTYPE ) )
			{
				return m_convexPlaneCF;
			}

			if( btBroadphaseProxy.isConvex( proxyType1 ) && ( proxyType0 == BroadphaseNativeTypes.STATIC_PLANE_PROXYTYPE ) )
			{
				return m_planeConvexCF;
			}



			if( btBroadphaseProxy.isConvex( proxyType0 ) && btBroadphaseProxy.isConvex( proxyType1 ) )
			{
				return m_convexConvexCreateFunc;
			}

			if( btBroadphaseProxy.isConvex( proxyType0 ) && btBroadphaseProxy.isConcave( proxyType1 ) )
			{
				return m_convexConcaveCreateFunc;
			}

			if( btBroadphaseProxy.isConvex( proxyType1 ) && btBroadphaseProxy.isConcave( proxyType0 ) )
			{
				return m_swappedConvexConcaveCreateFunc;
			}


			if( btBroadphaseProxy.isCompound( proxyType0 ) && btBroadphaseProxy.isCompound( proxyType1 ) )
			{
				return m_compoundCompoundCreateFunc;
			}

			if( btBroadphaseProxy.isCompound( proxyType0 ) )
			{
				return m_compoundCreateFunc;
			}
			else
			{
				if( btBroadphaseProxy.isCompound( proxyType1 ) )
				{
					return m_swappedCompoundCreateFunc;
				}
			}

			//failed to find an algorithm
			return m_emptyCreateFunc;
		}

		///Use this method to allow to generate multiple contact points between at once, between two objects using the generic convex-convex algorithm.
		///By default, this feature is disabled for best performance.
		///@param numPerturbationIterations controls the number of collision queries. Set it to zero to disable the feature.
		///@param minimumPointsPerturbationThreshold is the minimum number of points in the contact cache, above which the feature is disabled
		///3 is a good value for both params, if you want to enable the feature. This is because the default contact cache contains a maximum of 4 points, and one collision query at the unperturbed orientation is performed first.
		///See Bullet/Demos/CollisionDemo for an example how this feature gathers multiple points.
		///@todo we could add a per-object setting of those parameters, for level-of-detail collision detection.
		void setConvexConvexMultipointIterations( int numPerturbationIterations = 3, int minimumPointsPerturbationThreshold = 3 )
		{
			btConvexConvexAlgorithm.CreateFunc convexConvex = (btConvexConvexAlgorithm.CreateFunc)m_convexConvexCreateFunc;
			convexConvex.m_numPerturbationIterations = numPerturbationIterations;
			convexConvex.m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;
		}

		void setPlaneConvexMultipointIterations( int numPerturbationIterations = 3, int minimumPointsPerturbationThreshold = 3 )
		{
			btConvexPlaneCollisionAlgorithm.CreateFunc cpCF = (btConvexPlaneCollisionAlgorithm.CreateFunc)m_convexPlaneCF;
			cpCF.m_numPerturbationIterations = numPerturbationIterations;
			cpCF.m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;

			btConvexPlaneCollisionAlgorithm.CreateFunc pcCF = (btConvexPlaneCollisionAlgorithm.CreateFunc)m_planeConvexCF;
			pcCF.m_numPerturbationIterations = numPerturbationIterations;
			pcCF.m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;
		}

	};

}
