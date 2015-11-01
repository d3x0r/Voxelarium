using Bullet.Collision.NarrowPhase;
using Bullet.Collision;
using Bullet.LinearMath;
using Bullet.Types;
using System;
using System.Collections.Generic;
using System.Text;
using Bullet.Collision.Shapes;
using Bullet.Collision.BroadPhase;
using Bullet.Collision.Dispatch;
using Bullet.Dynamics;

namespace Bullet
{
	public class BulletGlobals
	{
		public static bool gDisableDeactivation = false;
		//public static int gOverlappingPairs = 0;
		public static float gDeactivationTime = 2f;
		/* git ef5650b67990814e332bb132efb5461fb8b8ee39 */
		public static string Version = "Bullet#-2.8.4-git";

		public static int BT_BULLET_VERSION = 284;

		public static int RAND_MAX = int.MaxValue;

		public static float gContactBreakingThreshold = .02f;

		public static Random gRandom = new Random();

		//public static IContactAddedCallback gContactAddedCallback;

		public static btIDebugDraw gDebugDraw;

		//public static StreamWriter g_streamWriter;
		/*
				public static IProfileManager g_profileManager;

				public static void StartProfile( String name )
				{
					if( g_profileManager != null )
					{
						g_profileManager.Start_Profile( name );
					}

				}
				public static void StopProfile()
				{
					if( g_profileManager != null )
					{
						g_profileManager.Stop_Profile();
					}
				}

				public static void ResetProfile()
				{
					if( g_profileManager != null )
					{
						g_profileManager.Reset();
					}
				}
				*/

		//public static ManifoldPoint GetManifoldPoint()
		//{
		//    if (m_pointStack.Count == 0)
		//    {
		//        m_pointStack.Push(new ManifoldPoint());
		//    }
		//    ++m_assignedPointCount;
		//    return m_pointStack.Pop();
		//}

		//public static void ReleaseManifoldPoint(ManifoldPoint mp)
		//{
		//    --m_assignedPointCount;
		//    m_pointStack.Push(mp);
		//}

		//private static Stack<ManifoldPoint> m_pointStack = new Stack<ManifoldPoint>(20);
		//private static int m_assignedPointCount = 0;

		internal static PooledType<btDbvtStackDataBlock> DbvtStackDataBlockPool = new PooledType<btDbvtStackDataBlock>();
		internal static PooledType<btDbvtNode> DbvtNodePool = new PooledType<btDbvtNode>();

		//#if UNUSED_GLOBAL
		internal static PooledType<btVoronoiSimplexSolver> VoronoiSimplexSolverPool = new PooledType<btVoronoiSimplexSolver>();
		internal static PooledType<btSubsimplexConvexCast> SubSimplexConvexCastPool = new PooledType<btSubsimplexConvexCast>();
		internal static PooledType<btManifoldPoint> ManifoldPointPool = new PooledType<btManifoldPoint>();
		internal static PooledType<btConvexCast.CastResult> CastResultPool = new PooledType<btConvexCast.CastResult>();
		internal static PooledType<btSphereShape> SphereShapePool = new PooledType<btSphereShape>();
		internal static PooledType<btBroadphasePair> BroadphasePairPool = new PooledType<btBroadphasePair>();
		internal static PooledType<btCollisionWorld.btSingleRayCallback> SingleRayCallbackPool = new PooledType<btCollisionWorld.btSingleRayCallback>();
		internal static PooledType<btSubSimplexClosestResult> SubSimplexClosestResultPool = new PooledType<btSubSimplexClosestResult>();
		internal static PooledType<btGjkPairDetector> GjkPairDetectorPool = new PooledType<btGjkPairDetector>();
		internal static PooledType<btDbvtBroadphase.btDbvtTreeCollider> DbvtTreeColliderPool = new PooledType<btDbvtBroadphase.btDbvtTreeCollider>();
		internal static PooledType<btCollisionWorld.btSingleSweepCallback> SingleSweepCallbackPool = new PooledType<btCollisionWorld.btSingleSweepCallback>();
		internal static PooledType<btDbvtBroadphase.BroadphaseRayTester> BroadphaseRayTesterPool = new PooledType<btDbvtBroadphase.BroadphaseRayTester>();
		internal static PooledType<btDiscreteDynamicsWorld.btClosestNotMeConvexResultCallback> ClosestNotMeConvexResultCallbackPool = new PooledType<btDiscreteDynamicsWorld.btClosestNotMeConvexResultCallback>();
		internal static PooledType<btGjkEpaPenetrationDepthSolver> GjkEpaPenetrationDepthSolverPool = new PooledType<btGjkEpaPenetrationDepthSolver>();
		internal static PooledType<btContinuousConvexCollision> ContinuousConvexCollisionPool = new PooledType<btContinuousConvexCollision>();

		internal static PooledType<btBoxBoxCollisionAlgorithm> BoxBoxCollisionAlgorithmPool = new PooledType<btBoxBoxCollisionAlgorithm>();
		//internal static PooledType<btCompoundCollisionAlgorithm> CompoundCollisionAlgorithmPool = new PooledType<btCompoundCollisionAlgorithm>();
		//internal static PooledType<btConvexConcaveCollisionAlgorithm> ConvexConcaveCollisionAlgorithmPool = new PooledType<btConvexConcaveCollisionAlgorithm>();
		internal static PooledType<btConvexConvexAlgorithm> ConvexConvexAlgorithmPool = new PooledType<btConvexConvexAlgorithm>();
		//internal static PooledType<btConvexPlaneCollisionAlgorithm> ConvexPlaneAlgorithmPool = new PooledType<btConvexPlaneCollisionAlgorithm>();
		//internal static PooledType<btSphereBoxCollisionAlgorithm> SphereBoxCollisionAlgorithmPool = new PooledType<btSphereBoxCollisionAlgorithm>();
		//internal static PooledType<btSphereSphereCollisionAlgorithm> SphereSphereCollisionAlgorithmPool = new PooledType<btSphereSphereCollisionAlgorithm>();
		//internal static PooledType<btSphereTriangleCollisionAlgorithm> SphereTriangleCollisionAlgorithmPool = new PooledType<btSphereTriangleCollisionAlgorithm>();
		//internal static PooledType<btGImpactCollisionAlgorithm> GImpactCollisionAlgorithmPool = new PooledType<btGImpactCollisionAlgorithm>();
		//internal static PooledType<btGjkEpaSolver2MinkowskiDiff> GjkEpaSolver2MinkowskiDiffPool = new PooledType<btGjkEpaSolver2MinkowskiDiff>();
		internal static PooledType<btPersistentManifold> PersistentManifoldPool = new PooledType<btPersistentManifold>();
		internal static PooledType<btManifoldResult> ManifoldResultPool = new PooledType<btManifoldResult>();
		internal static PooledType<btGjkEpaSolver2.GJK> GJKPool = new PooledType<btGjkEpaSolver2.GJK>();
		//internal static PooledType<GIM_ShapeRetriever> GIM_ShapeRetrieverPool = new PooledType<GIM_ShapeRetriever>();
		internal static PooledType<btTriangleShape> TriangleShapePool = new PooledType<btTriangleShape>();
		//internal static PooledType<SphereTriangleDetector> SphereTriangleDetectorPool = new PooledType<SphereTriangleDetector>();
		//internal static PooledType<CompoundLeafCallback> CompoundLeafCallbackPool = new PooledType<CompoundLeafCallback>();
		internal static PooledType<btGjkConvexCast> GjkConvexCastPool = new PooledType<btGjkConvexCast>();
		internal static PooledType<btCollisionWorld.btBridgedManifoldResult> BridgedManifoldResultPool = new PooledType<btCollisionWorld.btBridgedManifoldResult>();
		//internal static PooledType<LocalTriangleSphereCastCallback> LocalTriangleSphereCastCallbackPool = new PooledType<LocalTriangleSphereCastCallback>();
		//internal static PooledType<BridgeTriangleRaycastCallback> BridgeTriangleRaycastCallbackPool = new PooledType<BridgeTriangleRaycastCallback>();
		//internal static PooledType<BridgeTriangleConcaveRaycastCallback> BridgeTriangleConcaveRaycastCallbackPool = new PooledType<BridgeTriangleConcaveRaycastCallback>();
		internal static PooledType<btCollisionWorld.BridgeTriangleConvexcastCallback> BridgeTriangleConvexcastCallbackPool = new PooledType<btCollisionWorld.BridgeTriangleConvexcastCallback>();
		//internal static PooledType<MyNodeOverlapCallback> MyNodeOverlapCallbackPool = new PooledType<MyNodeOverlapCallback>();
		internal static PooledType<btCollisionWorld.ClosestRayResultCallback> ClosestRayResultCallbackPool = new PooledType<btCollisionWorld.ClosestRayResultCallback>();
		//internal static PooledType<DebugDrawcallback> DebugDrawcallbackPool = new PooledType<DebugDrawcallback>();
		internal static PooledType<btCollisionObjectWrapper> CollisionObjectWrapperPool = new PooledType<btCollisionObjectWrapper>();
//#endif

		public const bool debugRigidBody = true;
		public const bool debugCollisionWorld = false;
		public const bool debugConstraints = false;
		public const bool debugDiscreteDynamicsWorld = true;
		public const bool debugBoxBoxDetector = false;
		public const bool debugIslands = false;
		public const bool debugBVHTriangleMesh = false;
		public const bool debugConvexHull = false;
		public const bool debugConvexShape = false;
		public const bool debugShapeHull = false;
		public const bool debugStridingMesh = false;
		public const bool debugGJK = false;
		public const bool debugGJKDetector = false;
		public const bool debugPersistentManifold = false;
		public const bool debugVoronoiSimplex = false;
		public const bool debugSolver = true;
		public const bool debugBroadphase = false;
		public const bool debugBoxShape = false;
		public const bool debugGimpactShape = false;
		public const bool debugGimpactAlgo = false;
		public const bool debugGimpactBVH = false;
		public const bool debugPairCache = false;
		public const bool debugDispatcher = false;
		public const bool debugManifoldResult = false;


		// tracking
		public static int s_collisionAlgorithmInstanceCount = 0;

	}
}
