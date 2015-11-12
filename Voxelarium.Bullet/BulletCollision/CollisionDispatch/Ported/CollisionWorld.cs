//#define DISABLE_DBVT_COMPOUNDSHAPE_RAYCAST_ACCELERATION
//#define USE_BRUTEFORCE_RAYBROADPHASE 
//#define SUPPORT_TRIANGLE_MESH // requires BVH to be finished.
//# define USE_SUBSIMPLEX_CONVEX_CAST
//RECALCULATE_AABB is slower, but benefit is that you don't need to call 'stepSimulation'  or 'updateAabbs' before using a rayTest
//#define RECALCULATE_AABB_RAYCAST 1

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



//When the user doesn't provide dispatcher or broadphase, create basic versions (and delete them in destructor)

using System.Diagnostics;
using Bullet.Collision.BroadPhase;
using Bullet.Collision.NarrowPhase;
using Bullet.Collision.Shapes;
using Bullet.LinearMath;

namespace Bullet.Collision.Dispatch
{
	///for debug drawing

	//for debug rendering
	public partial class btCollisionWorld
	{
		internal btCollisionWorld()
		{
		}

		internal btCollisionWorld( btDispatcher dispatcher
						, btBroadphaseInterface pairCache
						, btCollisionConfiguration collisionConfiguration )
		{
			Initialize( dispatcher, pairCache, collisionConfiguration );
		}

		internal void Initialize( btDispatcher dispatcher
						, btBroadphaseInterface pairCache
						, btCollisionConfiguration collisionConfiguration )
		{
			m_dispatcher1 = dispatcher;
			m_broadphasePairCache = pairCache;
			m_debugDrawer = null;
			m_forceUpdateAllAabbs = true;
		}


		~btCollisionWorld()
		{

			//clean up remaining objects
			//int i;
			foreach( btCollisionObject collisionObject in m_collisionObjects )
			{
				btBroadphaseProxy bp = collisionObject.getBroadphaseHandle();
				if( bp != null )
				{
					//
					// only clear the cached algorithms
					//

					//getBroadphase().getOverlappingPairCache().cleanProxyFromPairs( bp, m_dispatcher1 );
					getBroadphase().destroyProxy( bp, m_dispatcher1 );

					collisionObject.setBroadphaseHandle( null );
				}
			}


		}

		//ConvexCast::CastResult
		internal class BridgeTriangleRaycastCallback : btTriangleRaycastCallback
		{
			RayResultCallback m_resultCallback;
			btCollisionObject m_collisionObject;
			btConcaveShape m_triangleMesh;

			btTransform m_colObjWorldTransform;

			internal BridgeTriangleRaycastCallback( ref btVector3 from, ref btVector3 to
				, RayResultCallback resultCallback
				, btCollisionObject collisionObject, btConcaveShape triangleMesh
				, ref btTransform colObjWorldTransform )
						: base( ref from, ref to, (btTriangleRaycastCallback.EFlags)resultCallback.m_flags )
			{
				m_resultCallback = ( resultCallback );
				m_collisionObject = ( collisionObject );
				m_triangleMesh = ( triangleMesh );
				m_colObjWorldTransform = ( colObjWorldTransform );
			}


			internal override double reportHit( ref btVector3 hitNormalLocal, double hitFraction, int partId, int triangleIndex )
			{
				LocalShapeInfo shapeInfo = new LocalShapeInfo();
				shapeInfo.m_shapePart = partId;
				shapeInfo.m_triangleIndex = triangleIndex;

				btVector3 hitNormalWorld; m_colObjWorldTransform.m_basis.Apply( ref hitNormalLocal, out hitNormalWorld );

				LocalRayResult rayResult = new LocalRayResult
									( m_collisionObject,
									  shapeInfo,
					hitNormalWorld,
					hitFraction );

				bool normalInWorldSpace = true;
				return m_resultCallback.addSingleResult( rayResult, normalInWorldSpace );
			}

		};

		internal class LocalInfoAdder2 : RayResultCallback
		{
			public RayResultCallback m_userCallback;
			public int m_i;

			internal LocalInfoAdder2( int i, RayResultCallback user )
			{
				m_userCallback = ( user ); m_i = ( i );
				m_closestHitFraction = m_userCallback.m_closestHitFraction;
				m_flags = m_userCallback.m_flags;
			}
			public override bool needsCollision( btBroadphaseProxy p )
			{
				return m_userCallback.needsCollision( p );
			}

			public override double addSingleResult( LocalRayResult r, bool b )
			{
				LocalShapeInfo shapeInfo = new LocalShapeInfo();
				shapeInfo.m_shapePart = -1;
				shapeInfo.m_triangleIndex = m_i;
				if( r.m_localShapeInfo == null )
					r.m_localShapeInfo = shapeInfo;

				btScalar result = m_userCallback.addSingleResult( r, b );
				m_closestHitFraction = m_userCallback.m_closestHitFraction;
				return result;
			}
		};

		internal struct RayTester : btDbvt.ICollide
		{
			btCollisionObject m_collisionObject;
			btCompoundShape m_compoundShape;
			btTransform m_colObjWorldTransform;
			btTransform m_rayFromTrans;
			btTransform m_rayToTrans;
			RayResultCallback m_resultCallback;

			internal RayTester( btCollisionObject collisionObject,
					btCompoundShape compoundShape,
					ref btTransform colObjWorldTransform,
					ref btTransform rayFromTrans,
					ref btTransform rayToTrans,
					RayResultCallback resultCallback )
			{
				m_collisionObject = ( collisionObject );
				m_compoundShape = ( compoundShape );
				m_colObjWorldTransform = ( colObjWorldTransform );
				m_rayFromTrans = ( rayFromTrans );
				m_rayToTrans = ( rayToTrans );
				m_resultCallback = ( resultCallback );

			}

			public void Process( int i )
			{
				btCollisionShape childCollisionShape = m_compoundShape.getChildShape( i );
				btTransform childTrans = m_compoundShape.getChildTransform( i );
				btTransform childWorldTrans; m_colObjWorldTransform.Apply( ref childTrans, out childWorldTrans );

				btCollisionObjectWrapper tmpOb = BulletGlobals.CollisionObjectWrapperPool.Get();
				tmpOb.Initialize( null, childCollisionShape, m_collisionObject, ref childWorldTrans, -1, i );
				// replace collision shape so that callback can determine the triangle

				LocalInfoAdder2 my_cb = new LocalInfoAdder2( i, m_resultCallback );

				rayTestSingleInternal(
					ref m_rayFromTrans,
					ref m_rayToTrans,
					tmpOb,
					my_cb );

				BulletGlobals.CollisionObjectWrapperPool.Free( tmpOb );
			}

			public void Process( btDbvt.btDbvtNode leaf )
			{
				Process( leaf.dataAsInt );
			}
			public void Process( btDbvt.btDbvtNode n, btDbvt.btDbvtNode n2 ) { }

			public void Process( btDbvt.btDbvtNode n, double f )
			{
				Process( n );
			}
			public bool Descent( btDbvt.btDbvtNode n )
			{
				return true;
			}
			public bool AllLeaves( btDbvt.btDbvtNode n )
			{
				return true;
			}

		};

		public virtual void addCollisionObject( btCollisionObject collisionObject
			, btBroadphaseProxy.CollisionFilterGroups collisionFilterGroup = btBroadphaseProxy.CollisionFilterGroups.DefaultFilter
			, btBroadphaseProxy.CollisionFilterGroups collisionFilterMask = btBroadphaseProxy.CollisionFilterGroups.AllFilter )
		{
			Debug.Assert( collisionObject != null );

			//check that the object isn't already added
			Debug.Assert( !m_collisionObjects.Contains( collisionObject ) );

			m_collisionObjects.Add( collisionObject );

			//calculate new AABB

			btVector3 minAabb;
			btVector3 maxAabb;
			collisionObject.getCollisionShape().getAabb( ref collisionObject.m_worldTransform, out minAabb, out maxAabb );

			BroadphaseNativeTypes type = collisionObject.getCollisionShape().getShapeType();
			collisionObject.setBroadphaseHandle( getBroadphase().createProxy(
				ref minAabb,
				ref maxAabb,
				type,
				collisionObject,
				collisionFilterGroup,
				collisionFilterMask,
				m_dispatcher1, null
				) );
		}


		static bool reportMe = true;

		void updateSingleAabb( btCollisionObject colObj )
		{
			btVector3 minAabb, maxAabb;
			colObj.getCollisionShape().getAabb( ref colObj.m_worldTransform, out minAabb, out maxAabb );
			//need to increase the aabb for contact thresholds
			btVector3 contactThreshold = new btVector3( btPersistentManifold.gContactBreakingThreshold, btPersistentManifold.gContactBreakingThreshold, btPersistentManifold.gContactBreakingThreshold );
			minAabb.Sub( ref contactThreshold, out minAabb );
			maxAabb.Add( ref contactThreshold, out maxAabb );
			//minAabb -= contactThreshold;
			//maxAabb += contactThreshold;

			if( getDispatchInfo().m_useContinuous
				&& colObj.getInternalType() == btCollisionObject.CollisionObjectTypes.CO_RIGID_BODY
				&& !colObj.isStaticOrKinematicObject() )
			{
				btVector3 minAabb2, maxAabb2;
				colObj.getCollisionShape().getAabb( ref colObj.m_interpolationWorldTransform, out minAabb2, out maxAabb2 );
				minAabb2.Sub( ref contactThreshold, out minAabb2 );
				maxAabb2.Add( ref contactThreshold, out maxAabb2 );
				minAabb.setMin( ref minAabb2 );
				maxAabb.setMax( ref maxAabb2 );
			}

			btBroadphaseInterface bp = (btBroadphaseInterface)m_broadphasePairCache;

			//moving objects should be moderately sized, probably something wrong if not
			if( colObj.isStaticObject()
				|| ( btVector3.BetweenLength2( ref minAabb, ref maxAabb ) < (double)( 1e12 ) ) )
			{
				bp.setAabb( colObj.getBroadphaseHandle(), ref minAabb, ref maxAabb, m_dispatcher1 );
			}
			else
			{
				//something went wrong, investigate
				//this assert is unwanted in 3D modelers (danger of loosing work)
				colObj.setActivationState( ActivationState.DISABLE_SIMULATION );

				if( reportMe && m_debugDrawer != null )
				{
					reportMe = false;
					m_debugDrawer.reportErrorWarning( "Overflow in AABB, object removed from simulation" );
					m_debugDrawer.reportErrorWarning( "If you can reproduce this, please email bugs@continuousphysics.com\n" );
					m_debugDrawer.reportErrorWarning( "Please include above information, your Platform, version of OS.\n" );
					m_debugDrawer.reportErrorWarning( "Thanks.\n" );
				}
			}
		}

		public virtual void updateAabbs()
		{
			CProfileSample sample = new CProfileSample( "updateAabbs" );

			//btTransform predictedTrans;
			for( int i = 0; i < m_collisionObjects.Count; i++ )
			{
				btCollisionObject colObj = m_collisionObjects[i];

				//only update aabb of active objects
				if( m_forceUpdateAllAabbs || colObj.isActive() )
				{
					updateSingleAabb( colObj );
				}
			}
		}


		///the computeOverlappingPairs is usually already called by performDiscreteCollisionDetection (or stepSimulation)
		///it can be useful to use if you perform ray tests without collision detection/simulation
		public virtual void computeOverlappingPairs()
		{
			CProfileSample sample = new CProfileSample( "calculateOverlappingPairs" );
			m_broadphasePairCache.calculateOverlappingPairs( m_dispatcher1 );
		}

		public virtual void performDiscreteCollisionDetection()
		{
			CProfileSample sample = new CProfileSample( "performDiscreteCollisionDetection" );

			//btDispatcherInfo dispatchInfo = m_dispatchInfo;

			updateAabbs();

			computeOverlappingPairs();

			//btDispatcher dispatcher = m_dispatcher1;
			{
				CProfileSample sample2 = new CProfileSample( "dispatchAllCollisionPairs" );
				if( m_dispatcher1 != null )
					m_dispatcher1.dispatchAllCollisionPairs( m_broadphasePairCache.getOverlappingPairCache(), m_dispatchInfo, m_dispatcher1 );
			}
		}



		public virtual void removeCollisionObject( btCollisionObject collisionObject )
		{


			//bool removeFromBroadphase = false;

			{

				btBroadphaseProxy bp = collisionObject.getBroadphaseHandle();
				if( bp != null )
				{
					//
					// only clear the cached algorithms
					//
					getBroadphase().getOverlappingPairCache().cleanProxyFromPairs( bp, m_dispatcher1 );
					getBroadphase().destroyProxy( bp, m_dispatcher1 );
					collisionObject.setBroadphaseHandle( null );
				}
			}


			//swapremove
			m_collisionObjects.Remove( collisionObject );

		}


		/// rayTestSingle performs a raycast call and calls the resultCallback. It is used internally by rayTest.
		/// In a future implementation, we consider moving the ray test as a virtual method in btCollisionShape.
		/// This allows more customization.
		public static void rayTestSingle( ref btTransform rayFromTrans, ref btTransform rayToTrans,
										btCollisionObject collisionObject,
											btCollisionShape collisionShape,
											ref btTransform colObjWorldTransform,
											RayResultCallback resultCallback )
		{
			btCollisionObjectWrapper colObWrap = BulletGlobals.CollisionObjectWrapperPool.Get();
			colObWrap.Initialize( null, collisionShape, collisionObject, ref colObjWorldTransform, -1, -1 );
			rayTestSingleInternal( ref rayFromTrans, ref rayToTrans, colObWrap, resultCallback );
			BulletGlobals.CollisionObjectWrapperPool.Free( colObWrap );
		}





		public static void rayTestSingleInternal( ref btTransform rayFromTrans, ref btTransform rayToTrans,
												btCollisionObjectWrapper collisionObjectWrap,
												RayResultCallback resultCallback )
		{
			btSphereShape pointShape = BulletGlobals.SphereShapePool.Get();
			pointShape.Initialize( btScalar.BT_ZERO );
			pointShape.setMargin( 0f );
			btConvexShape castShape = pointShape;
			btCollisionShape collisionShape = collisionObjectWrap.getCollisionShape();
			btTransform colObjWorldTransform = collisionObjectWrap.m_worldTransform;

			if( collisionShape.isConvex() )
			{
				//		CProfileSample sample = new CProfileSample("rayTestConvex");
				btConvexCast.CastResult castResult = new btConvexCast.CastResult();

				castResult.m_fraction = resultCallback.m_closestHitFraction;

				btConvexShape convexShape = (btConvexShape)collisionShape;
				btVoronoiSimplexSolver simplexSolver = BulletGlobals.VoronoiSimplexSolverPool.Get();
				//new btVoronoiSimplexSolver();
				btSubsimplexConvexCast subSimplexConvexCaster = BulletGlobals.SubSimplexConvexCastPool.Get();
				subSimplexConvexCaster.Initialize( castShape, convexShape, simplexSolver );

				btGjkConvexCast gjkConvexCaster = BulletGlobals.GjkConvexCastPool.Get();
				gjkConvexCaster.Initialize( castShape, convexShape, simplexSolver );
				BulletGlobals.SubSimplexConvexCastPool.Free( subSimplexConvexCaster );
				BulletGlobals.VoronoiSimplexSolverPool.Free( simplexSolver );

				//btContinuousConvexCollision convexCaster(castShape,convexShape,&simplexSolver,0);

				btConvexCast convexCasterPtr = null;
				//use kF_UseSubSimplexConvexCastRaytest by default
				if( ( resultCallback.m_flags & (uint)btTriangleRaycastCallback.EFlags.kF_UseGjkConvexCastRaytest ) != 0 )
					convexCasterPtr = gjkConvexCaster;
				else
					convexCasterPtr = subSimplexConvexCaster;

				btConvexCast convexCaster = convexCasterPtr;

				if( convexCaster.calcTimeOfImpact( ref rayFromTrans, ref rayToTrans, ref collisionObjectWrap.m_worldTransform, ref collisionObjectWrap.m_worldTransform, castResult ) )
				{
					//add hit
					if( castResult.m_normal.length2() > (double)( 0.0001 ) )
					{
						if( castResult.m_fraction < resultCallback.m_closestHitFraction )
						{
							//todo: figure out what this is about. When is rayFromTest.getBasis() not identity?
#if USE_SUBSIMPLEX_CONVEX_CAST
							//rotate normal into worldspace
							castResult.m_normal = rayFromTrans.getBasis()  castResult.m_normal;
#endif //USE_SUBSIMPLEX_CONVEX_CAST

							castResult.m_normal.normalize();
							LocalRayResult localRayResult = new LocalRayResult
								(
								collisionObjectWrap.m_collisionObject,
								null,
								castResult.m_normal,
								castResult.m_fraction
								);

							bool normalInWorldSpace = true;
							resultCallback.addSingleResult( localRayResult, normalInWorldSpace );

						}
					}
				}
				BulletGlobals.GjkConvexCastPool.Free( gjkConvexCaster );
			}
			else
			{
				if( collisionShape.isConcave() )
				{
					btTransform worldTocollisionObject; colObjWorldTransform.inverse( out worldTocollisionObject );
					btVector3 tmp;
					rayFromTrans.getOrigin( out tmp );
					btVector3 rayFromLocal; worldTocollisionObject.Apply( ref tmp, out rayFromLocal );
					rayToTrans.getOrigin( out tmp );
					btVector3 rayToLocal; worldTocollisionObject.Apply( ref tmp, out rayToLocal );

					//			CProfileSample sample = new CProfileSample("rayTestConcave");
#if SUPPORT_TRIANGLE_MESH
					if( collisionShape.getShapeType() == BroadphaseNativeTypes.TRIANGLE_MESH_SHAPE_PROXYTYPE )
					{
						///optimized version for btBvhTriangleMeshShape
						btBvhTriangleMeshShape triangleMesh = (btBvhTriangleMeshShape)collisionShape;

						BridgeTriangleRaycastCallback rcb( rayFromLocal, rayToLocal,&resultCallback, collisionObjectWrap.getCollisionObject(), triangleMesh, colObjWorldTransform);
						rcb.m_hitFraction = resultCallback.m_closestHitFraction;
						triangleMesh.performRaycast( &rcb, rayFromLocal, rayToLocal );
					}
					else
#endif
					{
						//generic (slower) case
						btConcaveShape concaveShape = (btConcaveShape)collisionShape;

						//ConvexCast::CastResult


						BridgeTriangleRaycastCallback rcb = new BridgeTriangleRaycastCallback( ref rayFromLocal, ref rayToLocal, resultCallback
							, collisionObjectWrap.m_collisionObject, concaveShape, ref colObjWorldTransform );
						rcb.m_hitFraction = resultCallback.m_closestHitFraction;

						btVector3 rayAabbMinLocal = rayFromLocal;
						rayAabbMinLocal.setMin( ref rayToLocal );
						btVector3 rayAabbMaxLocal = rayFromLocal;
						rayAabbMaxLocal.setMax( ref rayToLocal );

						concaveShape.processAllTriangles( rcb, ref rayAabbMinLocal, ref rayAabbMaxLocal );
					}
				}
				else
				{
					//			CProfileSample sample = new CProfileSample("rayTestCompound");
					if( collisionShape.isCompound() )
					{

						btCompoundShape compoundShape = (btCompoundShape)( collisionShape );
						btDbvt dbvt = compoundShape.getDynamicAabbTree();


						RayTester rayCB = new RayTester(
							collisionObjectWrap.m_collisionObject,
									compoundShape,
									ref colObjWorldTransform,
									ref rayFromTrans,
									ref rayToTrans,
									resultCallback );
#if !DISABLE_DBVT_COMPOUNDSHAPE_RAYCAST_ACCELERATION
						if( dbvt != null )
						{
							btTransform tmp;
							colObjWorldTransform.inverseTimes( ref rayFromTrans, out tmp );
							btVector3 localRayFrom; tmp.getOrigin( out localRayFrom );
							colObjWorldTransform.inverseTimes( ref rayToTrans, out tmp );
							btVector3 localRayTo; tmp.getOrigin( out localRayTo );
							btDbvt.rayTest( dbvt.m_root, ref localRayFrom, ref localRayTo, rayCB );
						}
						else
#endif //DISABLE_DBVT_COMPOUNDSHAPE_RAYCAST_ACCELERATION
						{
							for( int i = 0, n = compoundShape.getNumChildShapes(); i < n; ++i )
							{
								rayCB.Process( i );
							}
						}
					}
				}
			}
			BulletGlobals.SphereShapePool.Free( pointShape );
		}

		internal class BridgeTriangleConvexcastCallback : btTriangleConvexcastCallback
		{
			ConvexResultCallback m_resultCallback;
			btCollisionObject m_collisionObject;
			btConcaveShape m_triangleMesh;

			public BridgeTriangleConvexcastCallback() { }

			internal void Initialize( btConvexShape castShape, ref btTransform from, ref btTransform to,
				ConvexResultCallback resultCallback, btCollisionObject collisionObject
				, btConcaveShape triangleMesh, ref btTransform triangleToWorld )
			{
				base.Initialize( castShape, ref from, ref to, ref triangleToWorld, triangleMesh.getMargin() );
				m_resultCallback = ( resultCallback );
				m_collisionObject = ( collisionObject );
				m_triangleMesh = ( triangleMesh );
			}


			internal override double reportHit( ref btVector3 hitNormalLocal, ref btVector3 hitPointLocal
				, double hitFraction, int partId, int triangleIndex )
			{
				LocalShapeInfo shapeInfo = new LocalShapeInfo();
				shapeInfo.m_shapePart = partId;
				shapeInfo.m_triangleIndex = triangleIndex;
				if( hitFraction <= m_resultCallback.m_closestHitFraction )
				{

					LocalConvexResult convexResult = new LocalConvexResult
											( m_collisionObject,
											 shapeInfo,
						ref hitNormalLocal,
						ref hitPointLocal,
						hitFraction );

					bool normalInWorldSpace = true;


					return m_resultCallback.addSingleResult( ref convexResult, normalInWorldSpace );
				}
				return hitFraction;
			}

		};

		public static void objectQuerySingle( btConvexShape castShape, ref btTransform convexFromTrans, ref btTransform convexToTrans,
													btCollisionObject collisionObject,
													btCollisionShape collisionShape,
													ref btTransform colObjWorldTransform,
													ConvexResultCallback resultCallback, double allowedPenetration )
		{
			btCollisionObjectWrapper tmpOb = BulletGlobals.CollisionObjectWrapperPool.Get();
			tmpOb.Initialize( null, collisionShape, collisionObject, ref colObjWorldTransform, -1, -1 );
			objectQuerySingleInternal( castShape, ref convexFromTrans, ref convexToTrans, tmpOb, resultCallback, allowedPenetration );
			BulletGlobals.CollisionObjectWrapperPool.Free( tmpOb );
		}

		internal class LocalInfoAdder : ConvexResultCallback
		{
			ConvexResultCallback m_userCallback;
			int m_i;

			internal LocalInfoAdder( int i, ConvexResultCallback user )
			{
				m_userCallback = ( user ); m_i = ( i );
				m_closestHitFraction = m_userCallback.m_closestHitFraction;
			}
			public override bool needsCollision( btBroadphaseProxy p )
			{
				return m_userCallback.needsCollision( p );
			}
			public override double addSingleResult( ref LocalConvexResult r, bool b )
			{
				LocalShapeInfo shapeInfo = new LocalShapeInfo();
				shapeInfo.m_shapePart = -1;
				shapeInfo.m_triangleIndex = m_i;
				if( r.m_localShapeInfo == null )
					r.m_localShapeInfo = shapeInfo;
				btScalar result = m_userCallback.addSingleResult( ref r, b );
				m_closestHitFraction = m_userCallback.m_closestHitFraction;
				return result;
			}
		};


		public static void objectQuerySingleInternal( btConvexShape castShape, ref btTransform convexFromTrans, ref btTransform convexToTrans,
														btCollisionObjectWrapper colObjWrap,
														ConvexResultCallback resultCallback, double allowedPenetration )
		{
			btCollisionShape collisionShape = colObjWrap.getCollisionShape();
			//btTransform colObjWorldTransform = colObjWrap.m_worldTransform;

			if( collisionShape.isConvex() )
			{
				//CProfileSample sample = new CProfileSample("convexSweepConvex");
				btConvexCast.CastResult castResult = new btConvexCast.CastResult();
				castResult.m_allowedPenetration = allowedPenetration;
				castResult.m_fraction = resultCallback.m_closestHitFraction;//btScalar.BT_ONE;//??

				btConvexShape convexShape = (btConvexShape)collisionShape;
				btVoronoiSimplexSolver simplexSolver = BulletGlobals.VoronoiSimplexSolverPool.Get();

				btGjkEpaPenetrationDepthSolver gjkEpaPenetrationSolver = BulletGlobals.GjkEpaPenetrationDepthSolverPool.Get();
				//	new btGjkEpaPenetrationDepthSolver();

				btContinuousConvexCollision convexCaster1 = BulletGlobals.ContinuousConvexCollisionPool.Get();
				convexCaster1.Initialize( castShape, convexShape, simplexSolver, gjkEpaPenetrationSolver );
				//btGjkConvexCast convexCaster2(castShape,convexShape,&simplexSolver);
				//btSubsimplexConvexCast convexCaster3(castShape,convexShape,&simplexSolver);

				btConvexCast castPtr = convexCaster1;

				if( castPtr.calcTimeOfImpact( ref convexFromTrans, ref convexToTrans, ref colObjWrap.m_worldTransform, ref colObjWrap.m_worldTransform, castResult ) )
				{
					//add hit
					if( castResult.m_normal.length2() > (double)( 0.0001 ) )
					{
						if( castResult.m_fraction < resultCallback.m_closestHitFraction )
						{
							castResult.m_normal.normalize();
							LocalConvexResult localConvexResult =
								new LocalConvexResult
								(
								colObjWrap.m_collisionObject,
								null,
								ref castResult.m_normal,
								ref castResult.m_hitPoint,
								castResult.m_fraction
								);

							bool normalInWorldSpace = true;
							resultCallback.addSingleResult( ref localConvexResult, normalInWorldSpace );

						}
					}
				}
				BulletGlobals.GjkEpaPenetrationDepthSolverPool.Free( gjkEpaPenetrationSolver );
				BulletGlobals.VoronoiSimplexSolverPool.Free( simplexSolver );
				BulletGlobals.ContinuousConvexCollisionPool.Free( convexCaster1 );
			}
			else
			{
				if( collisionShape.isConcave() )
				{
					if( collisionShape.getShapeType() == BroadphaseNativeTypes.TRIANGLE_MESH_SHAPE_PROXYTYPE )
					{
						//CProfileSample sample = new CProfileSample("convexSweepbtBvhTriangleMesh");
						btConcaveShape triangleMesh = (btConcaveShape)collisionShape;
						btTransform worldTocollisionObject; colObjWrap.m_worldTransform.inverse( out worldTocollisionObject );
						btVector3 convexFromLocal; worldTocollisionObject.Apply( ref convexFromTrans.m_origin, out convexFromLocal );
						btVector3 convexToLocal; worldTocollisionObject.Apply( ref convexToTrans.m_origin, out convexToLocal );
						// rotation of box in local mesh space = MeshRotation^-1  ConvexToRotation
						btTransform rotationXform; worldTocollisionObject.m_basis.Apply( ref convexToTrans.m_basis, out rotationXform.m_basis );
						rotationXform.m_origin = btVector3.Zero;
						//ConvexCast::CastResult

						BridgeTriangleConvexcastCallback tccb = BulletGlobals.BridgeTriangleConvexcastCallbackPool.Get();
						tccb.Initialize( castShape, ref convexFromTrans, ref convexToTrans
							, resultCallback, colObjWrap.m_collisionObject
							, triangleMesh, ref colObjWrap.m_worldTransform );
						tccb.m_hitFraction = resultCallback.m_closestHitFraction;
						tccb.m_allowedPenetration = allowedPenetration;
						btVector3 boxMinLocal, boxMaxLocal;
						castShape.getAabb( ref rotationXform, out boxMinLocal, out boxMaxLocal );
						triangleMesh.performConvexcast( tccb, ref convexFromLocal, ref convexToLocal, ref boxMinLocal, ref boxMaxLocal );

						BulletGlobals.BridgeTriangleConvexcastCallbackPool.Free( tccb );
					}
					else
					{
						if( collisionShape.getShapeType() == BroadphaseNativeTypes.STATIC_PLANE_PROXYTYPE )
						{
							btConvexCast.CastResult castResult = BulletGlobals.CastResultPool.Get();
							castResult.m_allowedPenetration = allowedPenetration;
							castResult.m_fraction = resultCallback.m_closestHitFraction;
							btStaticPlaneShape planeShape = (btStaticPlaneShape)collisionShape;
							btContinuousConvexCollision convexCaster1 = BulletGlobals.ContinuousConvexCollisionPool.Get();
							convexCaster1.Initialize( castShape, planeShape );
							btConvexCast castPtr = convexCaster1;

							if( castPtr.calcTimeOfImpact(ref  convexFromTrans, ref convexToTrans, ref colObjWrap.m_worldTransform, ref colObjWrap.m_worldTransform, castResult ) )
							{
								//add hit
								if( castResult.m_normal.length2() > (double)( 0.0001 ) )
								{
									if( castResult.m_fraction < resultCallback.m_closestHitFraction )
									{
										castResult.m_normal.normalize();
										LocalConvexResult localConvexResult = new LocalConvexResult
											(
											colObjWrap.m_collisionObject,
											null,
											ref castResult.m_normal,
											ref castResult.m_hitPoint,
											castResult.m_fraction
											);

										bool normalInWorldSpace = true;
										resultCallback.addSingleResult( ref localConvexResult, normalInWorldSpace );
									}
								}
							}

						}
						else
						{
							//CProfileSample sample = new CProfileSample("convexSweepConcave");
							btConcaveShape concaveShape = (btConcaveShape)collisionShape;
							btTransform worldTocollisionObject; colObjWrap.m_worldTransform.inverse( out worldTocollisionObject );
							btVector3 convexFromLocal; worldTocollisionObject.Apply( ref convexFromTrans.m_origin, out convexFromLocal );
							btVector3 convexToLocal; worldTocollisionObject.Apply( ref convexToTrans.m_origin, out convexToLocal );
							// rotation of box in local mesh space = MeshRotation^-1  ConvexToRotation
							btTransform rotationXform; worldTocollisionObject.m_basis.Apply( ref convexToTrans.m_basis, out rotationXform.m_basis );
							rotationXform.m_origin = btVector3.Zero;


							BridgeTriangleConvexcastCallback tccb = BulletGlobals.BridgeTriangleConvexcastCallbackPool.Get();
							tccb.Initialize( castShape, ref convexFromTrans, ref convexToTrans, resultCallback, colObjWrap.m_collisionObject
								, concaveShape, ref colObjWrap.m_worldTransform );
							tccb.m_hitFraction = resultCallback.m_closestHitFraction;
							tccb.m_allowedPenetration = allowedPenetration;
							btVector3 boxMinLocal, boxMaxLocal;
							castShape.getAabb( ref rotationXform, out boxMinLocal, out boxMaxLocal );

							btVector3 rayAabbMinLocal = convexFromLocal;
							rayAabbMinLocal.setMin( ref convexToLocal );
							btVector3 rayAabbMaxLocal = convexFromLocal;
							rayAabbMaxLocal.setMax( ref convexToLocal );
							rayAabbMinLocal += boxMinLocal;
							rayAabbMaxLocal += boxMaxLocal;
							concaveShape.processAllTriangles( tccb, ref rayAabbMinLocal, ref rayAabbMaxLocal );
							BulletGlobals.BridgeTriangleConvexcastCallbackPool.Free( tccb );
						}
					}
				}
				else
				{
					///@todo : use AABB tree or other BVH acceleration structure!
					if( collisionShape.isCompound() )
					{
						CProfileSample sample = new CProfileSample( "convexSweepCompound" );
						btCompoundShape compoundShape = (btCompoundShape)( collisionShape );
						int i = 0;
						for( i = 0; i < compoundShape.getNumChildShapes(); i++ )
						{
							//btTransform childTrans = compoundShape.getChildTransform( i );
							btCollisionShape childCollisionShape = compoundShape.getChildShape( i );
							btTransform childWorldTrans; colObjWrap.m_worldTransform.Apply( ref  compoundShape.m_children.InternalArray[i].m_transform
										, out childWorldTrans );


							LocalInfoAdder my_cb = new LocalInfoAdder( i, resultCallback );

							btCollisionObjectWrapper tmpObj = BulletGlobals.CollisionObjectWrapperPool.Get();
							tmpObj.Initialize( colObjWrap, childCollisionShape, colObjWrap.m_collisionObject, ref childWorldTrans, -1, i );

							objectQuerySingleInternal( castShape, ref convexFromTrans, ref convexToTrans,
								tmpObj, my_cb, allowedPenetration );
							BulletGlobals.CollisionObjectWrapperPool.Free( tmpObj );
						}
					}
				}
			}
		}


		internal class btSingleRayCallback : btBroadphaseRayCallback
		{
			btVector3 m_rayFromWorld;
			btVector3 m_rayToWorld;
			btTransform m_rayFromTrans;
			btTransform m_rayToTrans;

			btCollisionWorld m_world;
			RayResultCallback m_resultCallback;

			public btSingleRayCallback()
			{
			}


			internal void Initialize( ref btVector3 rayFromWorld, ref btVector3 rayToWorld, btCollisionWorld world, RayResultCallback resultCallback )
			{
				m_rayFromWorld = ( rayFromWorld );
				m_rayToWorld = ( rayToWorld );
				m_world = ( world );
				m_resultCallback = ( resultCallback );
				m_rayFromTrans.setIdentity();
				m_rayFromTrans.setOrigin( ref m_rayFromWorld );
				m_rayToTrans.setIdentity();
				m_rayToTrans.setOrigin( ref m_rayToWorld );

				btVector3 rayDir = ( rayToWorld - rayFromWorld );

				rayDir.normalize();
				///what about division by zero? -. just set rayDirection[i] to INF/BT_LARGE_FLOAT
				m_rayDirectionInverse[0] = rayDir[0] == btScalar.BT_ZERO ? (double)( btScalar.BT_LARGE_FLOAT ) : (double)( 1.0 ) / rayDir[0];
				m_rayDirectionInverse[1] = rayDir[1] == btScalar.BT_ZERO ? (double)( btScalar.BT_LARGE_FLOAT ) : (double)( 1.0 ) / rayDir[1];
				m_rayDirectionInverse[2] = rayDir[2] == btScalar.BT_ZERO ? (double)( btScalar.BT_LARGE_FLOAT ) : (double)( 1.0 ) / rayDir[2];
				m_signs[0] = m_rayDirectionInverse[0] < 0.0 ? 1U : 0;
				m_signs[1] = m_rayDirectionInverse[1] < 0.0 ? 1U : 0;
				m_signs[2] = m_rayDirectionInverse[2] < 0.0 ? 1U : 0;
				m_lambda_max = rayDir.dot( m_rayToWorld - m_rayFromWorld );
			}


			public override bool process( btBroadphaseProxy proxy )
			{
				///terminate further ray tests, once the closestHitFraction reached zero
				if( m_resultCallback.m_closestHitFraction == (double)( 0f ) )
					return false;

				btCollisionObject collisionObject = (btCollisionObject)proxy.m_clientObject;

				//only perform raycast if filterMask matches
				if( m_resultCallback.needsCollision( collisionObject.getBroadphaseHandle() ) )
				{
					//RigidcollisionObject collisionObject = ctrl.GetRigidcollisionObject();
					//btVector3 collisionObjectAabbMin,collisionObjectAabbMax;
#if false
#if RECALCULATE_AABB
			btVector3 collisionObjectAabbMin,collisionObjectAabbMax;
			collisionObject.getCollisionShape().getAabb(collisionObject.getWorldTransform(),collisionObjectAabbMin,collisionObjectAabbMax);
#else
					//getBroadphase().getAabb(collisionObject.getBroadphaseHandle(),collisionObjectAabbMin,collisionObjectAabbMax);
					btVector3 collisionObjectAabbMin = collisionObject.getBroadphaseHandle().m_aabbMin;
					btVector3 collisionObjectAabbMax = collisionObject.getBroadphaseHandle().m_aabbMax;
#endif
#endif
					//double hitLambda = m_resultCallback.m_closestHitFraction;
					//culling already done by broadphase
					//if (btRayAabb(m_rayFromWorld,m_rayToWorld,collisionObjectAabbMin,collisionObjectAabbMax,hitLambda,m_hitNormal))
					{
						rayTestSingle( ref m_rayFromTrans, ref m_rayToTrans,
							collisionObject,
							collisionObject.getCollisionShape(),
							ref collisionObject.m_worldTransform,
							m_resultCallback );
					}
				}
				return true;
			}
		};

		/// rayTest performs a raycast on all objects in the btCollisionWorld, and calls the resultCallback
		/// This allows for several queries: first hit, all hits, any hit, dependent on the value returned by the callback.
		void rayTest( btVector3 rayFromWorld, btVector3 rayToWorld, RayResultCallback resultCallback )
		{
			//CProfileSample sample = new CProfileSample("rayTest");
			/// use the broadphase to accelerate the search for objects, based on their aabb
			/// and for each object with ray-aabb overlap, perform an exact ray test
			btSingleRayCallback rayCB = BulletGlobals.SingleRayCallbackPool.Get();
			rayCB.Initialize( ref rayFromWorld, ref rayToWorld, this, resultCallback );

#if !USE_BRUTEFORCE_RAYBROADPHASE
			m_broadphasePairCache.rayTest( ref rayFromWorld, ref rayToWorld, rayCB );
#else
	for( int i = 0; i < this.getNumCollisionObjects(); i++ )
	{
		rayCB.process( m_collisionObjects[i].getBroadphaseHandle() );
	}
#endif //USE_BRUTEFORCE_RAYBROADPHASE
			BulletGlobals.SingleRayCallbackPool.Free( rayCB );
		}


		internal class btSingleSweepCallback : btBroadphaseRayCallback
		{

			btTransform m_convexFromTrans;
			btTransform m_convexToTrans;
			//btVector3 m_hitNormal;
			btCollisionWorld m_world;
			ConvexResultCallback m_resultCallback;
			double m_allowedCcdPenetration;
			btConvexShape m_castShape;

			public btSingleSweepCallback() { }

			internal void Initialize( btConvexShape castShape, ref btTransform convexFromTrans, ref btTransform convexToTrans, btCollisionWorld world, ConvexResultCallback resultCallback, double allowedPenetration )
			{
				m_convexFromTrans = ( convexFromTrans );
				m_convexToTrans = ( convexToTrans );
				m_world = ( world );
				m_resultCallback = ( resultCallback );
				m_allowedCcdPenetration = ( allowedPenetration );
				m_castShape = ( castShape );
				btVector3 unnormalizedRayDir; m_convexToTrans.m_origin.Sub( ref m_convexFromTrans.m_origin, out unnormalizedRayDir );
				btVector3 rayDir; unnormalizedRayDir.normalized( out rayDir );
				///what about division by zero? -. just set rayDirection[i] to INF/BT_LARGE_FLOAT
				m_rayDirectionInverse[0] = rayDir[0] == btScalar.BT_ZERO ? btScalar.BT_LARGE_FLOAT : (double)( 1.0 ) / rayDir[0];
				m_rayDirectionInverse[1] = rayDir[1] == btScalar.BT_ZERO ? btScalar.BT_LARGE_FLOAT : (double)( 1.0 ) / rayDir[1];
				m_rayDirectionInverse[2] = rayDir[2] == btScalar.BT_ZERO ? btScalar.BT_LARGE_FLOAT : (double)( 1.0 ) / rayDir[2];
				m_signs[0] = m_rayDirectionInverse[0] < 0.0 ? 1U : 0;
				m_signs[1] = m_rayDirectionInverse[1] < 0.0 ? 1U : 0;
				m_signs[2] = m_rayDirectionInverse[2] < 0.0 ? 1U : 0;

				m_lambda_max = rayDir.dot( unnormalizedRayDir );

			}

			public override bool process( btBroadphaseProxy proxy )
			{
				///terminate further convex sweep tests, once the closestHitFraction reached zero
				if( m_resultCallback.m_closestHitFraction == (double)( 0f ) )
					return false;

				btCollisionObject collisionObject = (btCollisionObject)proxy.m_clientObject;

				//only perform raycast if filterMask matches
				if( m_resultCallback.needsCollision( collisionObject.getBroadphaseHandle() ) )
				{
					//RigidcollisionObject collisionObject = ctrl.GetRigidcollisionObject();
					objectQuerySingle( m_castShape, ref m_convexFromTrans, ref m_convexToTrans,
						collisionObject,
						collisionObject.getCollisionShape(),
						ref collisionObject.m_worldTransform,
						m_resultCallback,
						m_allowedCcdPenetration );
				}

				return true;
			}
		};



		/// convexTest performs a swept convex cast on all objects in the btCollisionWorld, and calls the resultCallback
		/// This allows for several queries: first hit, all hits, any hit, dependent on the value return by the callback.
		internal void convexSweepTest( btConvexShape castShape, ref btTransform convexFromWorld, ref btTransform convexToWorld, ConvexResultCallback resultCallback, double allowedCcdPenetration = btScalar.BT_ZERO )
		{

			CProfileSample sample = new CProfileSample( "convexSweepTest" );
			/// use the broadphase to accelerate the search for objects, based on their aabb
			/// and for each object with ray-aabb overlap, perform an exact ray test
			/// unfortunately the implementation for rayTest and convexSweepTest duplicated, albeit practically identical



			btTransform convexFromTrans, convexToTrans;
			convexFromTrans = convexFromWorld;
			convexToTrans = convexToWorld;
			btVector3 castShapeAabbMin, castShapeAabbMax;
			/* Compute AABB that encompasses angular movement */
			{
				btVector3 linVel, angVel;
				btTransformUtil.calculateVelocity( ref convexFromWorld, ref convexToWorld, 1.0f, out linVel, out angVel );
				btVector3 zeroLinVel = btVector3.Zero;
				btTransform R = btTransform.Identity;
				R.m_basis = convexFromWorld.m_basis;
				castShape.calculateTemporalAabb( ref R, ref zeroLinVel, ref angVel, 1.0f, out castShapeAabbMin, out castShapeAabbMax );
			}

#if !USE_BRUTEFORCE_RAYBROADPHASE

			btSingleSweepCallback convexCB = BulletGlobals.SingleSweepCallbackPool.Get();
			convexCB.Initialize( castShape, ref convexFromWorld, ref convexToWorld, this, resultCallback, allowedCcdPenetration );

			m_broadphasePairCache.rayTest( ref convexFromTrans.m_origin, ref convexToTrans.m_origin, convexCB, ref castShapeAabbMin, ref castShapeAabbMax );
			BulletGlobals.SingleSweepCallbackPool.Free( convexCB );
#else
	/// go over all objects, and if the ray intersects their aabb + cast shape aabb,
	// do a ray-shape query using convexCaster (CCD)
	int i;
	for( i = 0; i < m_collisionObjects.Count; i++ )
	{
		btCollisionObject collisionObject = m_collisionObjects[i];
		//only perform raycast if filterMask matches
		if( resultCallback.needsCollision( collisionObject.getBroadphaseHandle() ) )
		{
			//RigidcollisionObject collisionObject = ctrl.GetRigidcollisionObject();
			btVector3 collisionObjectAabbMin, collisionObjectAabbMax;
			collisionObject.getCollisionShape().getAabb( collisionObject.getWorldTransform(), collisionObjectAabbMin, collisionObjectAabbMax );
			AabbExpand( collisionObjectAabbMin, collisionObjectAabbMax, castShapeAabbMin, castShapeAabbMax );
			double hitLambda = (double)( 1.0 ); //could use resultCallback.m_closestHitFraction, but needs testing
			btVector3 hitNormal;
			if( btRayAabb( convexFromWorld.getOrigin(), convexToWorld.getOrigin(), collisionObjectAabbMin, collisionObjectAabbMax, hitLambda, hitNormal ) )
			{
				objectQuerySingle( castShape, convexFromTrans, convexToTrans,
					collisionObject,
					collisionObject.getCollisionShape(),
					collisionObject.getWorldTransform(),
					resultCallback,
					allowedCcdPenetration );
			}
		}
	}
#endif //USE_BRUTEFORCE_RAYBROADPHASE
		}



		internal class btBridgedManifoldResult : btManifoldResult
		{

			ContactResultCallback m_resultCallback;
			public btBridgedManifoldResult() { }


			internal void Initialize( btCollisionObjectWrapper obj0Wrap, btCollisionObjectWrapper obj1Wrap, ContactResultCallback resultCallback )
			{
				base.Initialize( obj0Wrap, obj1Wrap );
				m_resultCallback = ( resultCallback );
			}

			public override void addContactPoint( ref btVector3 normalOnBInWorld, ref btVector3 pointInWorld, double depth )
			{
				bool isSwapped = m_manifoldPtr.m_body0 != m_body0Wrap.m_collisionObject;
				btVector3 pointA; pointInWorld.AddScale( ref normalOnBInWorld, depth, out pointA );
				btVector3 localA;
				btVector3 localB;
				if( isSwapped )
				{
					m_body1Wrap.m_collisionObject.m_worldTransform.invXform( ref pointA, out localA );
					m_body0Wrap.m_collisionObject.m_worldTransform.invXform( ref pointInWorld, out localB );
				}
				else
				{
					m_body0Wrap.m_collisionObject.m_worldTransform.invXform( ref pointA, out localA );
					m_body1Wrap.m_collisionObject.m_worldTransform.invXform( ref pointInWorld, out localB );
				}

				btManifoldPoint newPt = BulletGlobals.ManifoldPointPool.Get();
				newPt.Initialize( ref localA, ref localB, ref normalOnBInWorld, depth );
				newPt.m_positionWorldOnA = pointA;
				newPt.m_positionWorldOnB = pointInWorld;

				//BP mod, store contact triangles.
				if( isSwapped )
				{
					newPt.m_partId0 = m_partId1;
					newPt.m_partId1 = m_partId0;
					newPt.m_index0 = m_index1;
					newPt.m_index1 = m_index0;
				}
				else
				{
					newPt.m_partId0 = m_partId0;
					newPt.m_partId1 = m_partId1;
					newPt.m_index0 = m_index0;
					newPt.m_index1 = m_index1;
				}

				//experimental feature info, for per-triangle material etc.
				btCollisionObjectWrapper obj0Wrap = isSwapped ? m_body1Wrap : m_body0Wrap;
				btCollisionObjectWrapper obj1Wrap = isSwapped ? m_body0Wrap : m_body1Wrap;
				m_resultCallback.addSingleResult( newPt, obj0Wrap, newPt.m_partId0, newPt.m_index0, obj1Wrap, newPt.m_partId1, newPt.m_index1 );

			}

		};



		struct btSingleContactCallback : btBroadphaseAabbCallback
		{

			btCollisionObject m_collisionObject;
			btCollisionWorld m_world;
			ContactResultCallback m_resultCallback;


			internal btSingleContactCallback( btCollisionObject collisionObject, btCollisionWorld world, ContactResultCallback resultCallback )
			{
				m_collisionObject = ( collisionObject );
				m_world = ( world );
				m_resultCallback = ( resultCallback );
			}

			public bool process( btBroadphaseProxy proxy )
			{
				btCollisionObject collisionObject = (btCollisionObject)proxy.m_clientObject;
				if( collisionObject == m_collisionObject )
					return true;

				//only perform raycast if filterMask matches
				if( m_resultCallback.needsCollision( collisionObject.getBroadphaseHandle() ) )
				{
					btCollisionObjectWrapper ob0 = BulletGlobals.CollisionObjectWrapperPool.Get();
					ob0.Initialize( null, m_collisionObject.getCollisionShape(), m_collisionObject, ref m_collisionObject.m_worldTransform, -1, -1 );
					btCollisionObjectWrapper ob1 = BulletGlobals.CollisionObjectWrapperPool.Get();
					ob1.Initialize( null, collisionObject.getCollisionShape(), collisionObject, ref collisionObject.m_worldTransform, -1, -1 );

					btCollisionAlgorithm algorithm = m_world.m_dispatcher1.findAlgorithm( ob0, ob1, null );
					if( algorithm != null )
					{
						btBridgedManifoldResult contactPointResult = BulletGlobals.BridgedManifoldResultPool.Get();

						contactPointResult.Initialize( ob0, ob1, m_resultCallback );
						//discrete collision detection query

						algorithm.processCollision( ob0, ob1, m_world.getDispatchInfo(), contactPointResult );

						//algorithm.~btCollisionAlgorithm();
						m_world.m_dispatcher1.freeCollisionAlgorithm( algorithm );
						BulletGlobals.BridgedManifoldResultPool.Free( contactPointResult );
					}
					BulletGlobals.CollisionObjectWrapperPool.Free( ob0 );
					BulletGlobals.CollisionObjectWrapperPool.Free( ob1 );
				}
				return true;
			}
		};


		///contactTest performs a discrete collision test against all objects in the btCollisionWorld, and calls the resultCallback.
		///it reports one or more contact points for every overlapping object (including the one with deepest penetration)
		void contactTest( btCollisionObject colObj, ContactResultCallback resultCallback )
		{
			btVector3 aabbMin, aabbMax;
			colObj.getCollisionShape().getAabb( ref colObj.m_worldTransform, out aabbMin, out aabbMax );
			btSingleContactCallback contactCB = new btSingleContactCallback( colObj, this, resultCallback );

			m_broadphasePairCache.aabbTest( ref aabbMin, ref aabbMax, contactCB );
		}


		///contactTest performs a discrete collision test between two collision objects and calls the resultCallback if overlap if detected.
		///it reports one or more contact points (including the one with deepest penetration)
		void contactPairTest( btCollisionObject colObjA, btCollisionObject colObjB, ContactResultCallback resultCallback )
		{
			btCollisionObjectWrapper obA = BulletGlobals.CollisionObjectWrapperPool.Get();
			obA.Initialize( null, colObjA.getCollisionShape(), colObjA, ref colObjA.m_worldTransform, -1, -1 );
			btCollisionObjectWrapper obB = BulletGlobals.CollisionObjectWrapperPool.Get();
			obB.Initialize( null, colObjB.getCollisionShape(), colObjB, ref colObjB.m_worldTransform, -1, -1 );

			btCollisionAlgorithm algorithm = m_dispatcher1.findAlgorithm( obA, obB, null );
			if( algorithm != null )
			{
				btBridgedManifoldResult contactPointResult = BulletGlobals.BridgedManifoldResultPool.Get();
				contactPointResult.Initialize( obA, obB, resultCallback );
				//discrete collision detection query
				algorithm.processCollision( obA, obB, getDispatchInfo(), contactPointResult );

				//algorithm.~btCollisionAlgorithm();
				m_dispatcher1.freeCollisionAlgorithm( algorithm );
				BulletGlobals.BridgedManifoldResultPool.Free( contactPointResult );
			}
			BulletGlobals.CollisionObjectWrapperPool.Free( obA );
			BulletGlobals.CollisionObjectWrapperPool.Free( obB );
		}


		internal class DebugDrawcallback : btInternalTriangleIndexCallback, btTriangleCallback
		{
			btIDebugDraw m_debugDrawer;
			btVector3 m_color;
			btTransform m_worldTrans;

			public DebugDrawcallback( btIDebugDraw debugDrawer, ref btTransform worldTrans, ref btVector3 color )
			{
				m_debugDrawer = ( debugDrawer );
				m_color = ( color );
				m_worldTrans = ( worldTrans );
			}

			internal override void internalProcessTriangleIndex( btVector3[] triangle, int partId, int triangleIndex )
			{
				processTriangle( triangle, partId, triangleIndex );
			}

			public void processTriangle( btVector3[] triangle, int partId, int triangleIndex )
			{
				//(void)partId;
				//(void)triangleIndex;

				btVector3 wv0, wv1, wv2;
				m_worldTrans.Apply( ref triangle[0], out wv0 );
				m_worldTrans.Apply( ref triangle[1], out wv1 );
				m_worldTrans.Apply( ref triangle[2], out wv2 );

				btVector3 center;
				wv0.Add( ref wv1, out center );
				center.Add( ref wv2, out center );
				center.Mult( (double)( 1.0 / 3.0 ), out center );

				if( ( m_debugDrawer.getDebugMode() & btIDebugDraw.DebugDrawModes.DBG_DrawNormals ) != 0 )
				{
					btVector3 normal = ( wv1 - wv0 ).cross( wv2 - wv0 );
					normal.normalize();
					btVector3 normalColor = new btVector3( 1, 1, 0, 1 );
					btVector3 tmp; center.Add( ref normal, out tmp );
					m_debugDrawer.drawLine( ref center, ref tmp, ref normalColor );
				}
				m_debugDrawer.drawLine( ref wv0, ref wv1, ref m_color );
				m_debugDrawer.drawLine( ref wv1, ref wv2, ref m_color );
				m_debugDrawer.drawLine( ref wv2, ref wv0, ref m_color );
			}
		};


		public virtual void debugDrawObject( ref btTransform worldTransform, btCollisionShape shape, btVector3 color )
		{
			// Draw a small simplex at the center of the object
			if( m_debugDrawer != null && ( ( m_debugDrawer.getDebugMode() & btIDebugDraw.DebugDrawModes.DBG_DrawFrames ) != 0 ) )
			{
				m_debugDrawer.drawTransform( ref worldTransform, 1 );
			}

			if( shape.getShapeType() == BroadphaseNativeTypes.COMPOUND_SHAPE_PROXYTYPE )
			{
				btCompoundShape compoundShape = (btCompoundShape)( shape );
				for( int i = compoundShape.getNumChildShapes() - 1; i >= 0; i-- )
				{
					//btITransform childTrans = compoundShape.getChildTransform( i );
					btCollisionShape colShape = compoundShape.getChildShape( i );
					btTransform tmp; worldTransform.Apply( ref compoundShape.m_children.InternalArray[i].m_transform, out tmp );
					debugDrawObject( ref tmp, colShape, color );
				}
			}
			else
			{

				switch( shape.getShapeType() )
				{

					case BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE:
						{
							btBoxShape boxShape = (btBoxShape)( shape );
							btVector3 halfExtents; boxShape.getHalfExtentsWithMargin( out halfExtents );
							btVector3 tmp; halfExtents.Invert( out tmp );
							m_debugDrawer.drawBox( ref tmp, ref halfExtents, ref worldTransform, ref color );
							break;
						}

					case BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE:
						{
							btSphereShape sphereShape = (btSphereShape)( shape );
							double radius = sphereShape.getMargin();//radius doesn't include the margin, so draw with margin

							m_debugDrawer.drawSphere( radius, ref worldTransform, ref color );
							break;
						}
					case BroadphaseNativeTypes.MULTI_SPHERE_SHAPE_PROXYTYPE:
						{
							btMultiSphereShape multiSphereShape = (btMultiSphereShape)( shape );

							btTransform childTransform = btTransform.Identity;

							for( int i = multiSphereShape.getSphereCount() - 1; i >= 0; i-- )
							{
								multiSphereShape.getSpherePosition( i, out childTransform.m_origin );
								btTransform tmp;
								worldTransform.Apply( ref childTransform, out tmp );
								m_debugDrawer.drawSphere( multiSphereShape.getSphereRadius( i ), ref tmp, ref color );
							}

							break;
						}
					case BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE:
						{
							btCapsuleShape capsuleShape = (btCapsuleShape)( shape );

							double radius = capsuleShape.getRadius();
							double halfHeight = capsuleShape.getHalfHeight();

							int upAxis = capsuleShape.getUpAxis();
							m_debugDrawer.drawCapsule( radius, halfHeight, upAxis, ref worldTransform, ref color );
							break;
						}
					case BroadphaseNativeTypes.CONE_SHAPE_PROXYTYPE:
						{
							btConeShape coneShape = (btConeShape)( shape );
							double radius = coneShape.getRadius();//+coneShape.getMargin();
							double height = coneShape.getHeight();//+coneShape.getMargin();

							int upAxis = coneShape.getConeUpIndex();
							m_debugDrawer.drawCone( radius, height, upAxis, ref worldTransform, ref color );
							break;

						}
					case BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE:
						{
							btCylinderShape cylinder = (btCylinderShape)( shape );
							int upAxis = cylinder.getUpAxis();
							double radius = cylinder.getRadius();
							btVector3 tmp;
							cylinder.getHalfExtentsWithMargin( out tmp );
							double halfHeight = tmp[upAxis];
							m_debugDrawer.drawCylinder( radius, halfHeight, upAxis, ref worldTransform, ref color );
							break;
						}

					case BroadphaseNativeTypes.STATIC_PLANE_PROXYTYPE:
						{
							btStaticPlaneShape staticPlaneShape = (btStaticPlaneShape)( shape );
							double planeConst = staticPlaneShape.getPlaneConstant();
							btVector3 planeNormal; staticPlaneShape.getPlaneNormal( out planeNormal );
							m_debugDrawer.drawPlane( ref planeNormal, planeConst, ref worldTransform, ref color );
							break;

						}
					default:
						{

							/// for polyhedral shapes
							if( shape.isPolyhedral() )
							{
								btPolyhedralConvexShape polyshape = (btPolyhedralConvexShape)shape;

								int i;
								if( polyshape.getConvexPolyhedron() != null )
								{
									btConvexPolyhedron poly = polyshape.getConvexPolyhedron();
									for( i = 0; i < poly.m_faces.Count; i++ )
									{
										btVector3 centroid = btVector3.Zero;
										int numVerts = poly.m_faces[i].m_indices.Count;
										if( numVerts > 0 )
										{
											int lastV = poly.m_faces[i].m_indices[numVerts - 1];
											for( int v = 0; v < poly.m_faces[i].m_indices.Count; v++ )
											{
												int curVert = poly.m_faces[i].m_indices[v];
												centroid += poly.m_vertices[curVert];
												btVector3 tmp1, tmp2;
												worldTransform.Apply( ref poly.m_vertices.InternalArray[lastV], out tmp1 );
												worldTransform.Apply( ref poly.m_vertices.InternalArray[curVert], out tmp2 );

												m_debugDrawer.drawLine( ref tmp1, ref tmp2, ref color );
												lastV = curVert;
											}
										}
										centroid *= (double)( 1 ) / (double)( numVerts );
										if( ( m_debugDrawer.getDebugMode() & btIDebugDraw.DebugDrawModes.DBG_DrawNormals ) != 0 )
										{
											btVector3 normalColor = new btVector3( 1, 1, 0, 1 );
											btVector3 faceNormal = new btVector3( poly.m_faces[i].m_plane[0], poly.m_faces[i].m_plane[1], poly.m_faces[i].m_plane[2] );
											btVector3 tmp, tmp2;
											centroid.Add( ref faceNormal, out tmp );
											worldTransform.Apply( ref tmp, out tmp2 );
											worldTransform.Apply( ref centroid, out tmp );
											m_debugDrawer.drawLine( ref tmp, ref tmp2, ref normalColor );
										}
									}
								}
								else
								{
									for( i = 0; i < polyshape.getNumEdges(); i++ )
									{
										btVector3 a, b;
										polyshape.getEdge( i, out a, out b );
										btVector3 wa; worldTransform.Apply( ref a, out wa );
										btVector3 wb; worldTransform.Apply( ref b, out wb );
										m_debugDrawer.drawLine( ref wa, ref wb, ref color );
									}
								}

							}

							if( shape.isConcave() )
							{
								btConcaveShape concaveMesh = (btConcaveShape)shape;

								///@todo pass camera, for some culling? no . we are not a graphics lib
								btVector3 aabbMax = btVector3.Max;
								btVector3 aabbMin = btVector3.Min;

								DebugDrawcallback drawCallback = new DebugDrawcallback( m_debugDrawer, ref worldTransform, ref color );
								concaveMesh.processAllTriangles( drawCallback, ref aabbMin, ref aabbMax );

							}

							if( shape.getShapeType() == BroadphaseNativeTypes.CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE )
							{
								Debug.Assert( false, "This needs some work... don't know the mesher stride interface types; replace with interfaces..." );
#if asdfasdf
							btConvexTriangleMeshShape convexMesh = (btConvexTriangleMeshShape)shape;
							//todo: pass camera for some culling			
							btVector3 aabbMax = btVector3.Max;// ( (double)( BT_LARGE_FLOAT ), (double)( BT_LARGE_FLOAT ), (double)( BT_LARGE_FLOAT ));
							btVector3 aabbMin = btVector3.Min;// ( (double)( -BT_LARGE_FLOAT ), (double)( -BT_LARGE_FLOAT ), (double)( -BT_LARGE_FLOAT ));
							//DebugDrawcallback drawCallback;
							DebugDrawcallback drawCallback = new DebugDrawcallback( m_debugDrawer, ref worldTransform, color );
							convexMesh.getMeshInterface().InternalProcessAllTriangles( drawCallback, aabbMin, aabbMax );
#endif
							}

							break;
						}

				}
			}
		}

		public virtual void debugDrawWorld()
		{
			if( m_debugDrawer != null )
			{
				btIDebugDraw.DefaultColors defaultColors = m_debugDrawer.getDefaultColors();

				if( ( m_debugDrawer.getDebugMode() & btIDebugDraw.DebugDrawModes.DBG_DrawContactPoints ) != 0 )
				{

					if( m_dispatcher1 != null )
					{
						int numManifolds = m_dispatcher1.getNumManifolds();

						for( int i = 0; i < numManifolds; i++ )
						{
							btPersistentManifold contactManifold = m_dispatcher1.getManifoldByIndexInternal( i );
							//btCollisionObject obA = static_cast<btCollisionObject>(contactManifold.getBody0());
							//btCollisionObject obB = static_cast<btCollisionObject>(contactManifold.getBody1());

							int numContacts = contactManifold.m_cachedPoints;
							for( int j = 0; j < numContacts; j++ )
							{
								btManifoldPoint cp = contactManifold.getContactPoint( j );
								m_debugDrawer.drawContactPoint( ref cp.m_positionWorldOnB, ref cp.m_normalWorldOnB, cp.getDistance(), cp.getLifeTime(), ref defaultColors.m_contactPoint );
							}
						}
					}
				}

				if( ( m_debugDrawer.getDebugMode() & ( btIDebugDraw.DebugDrawModes.DBG_DrawWireframe | btIDebugDraw.DebugDrawModes.DBG_DrawAabb ) ) != 0 )
				{
					int i;

					for( i = 0; i < m_collisionObjects.Count; i++ )
					{
						btCollisionObject colObj = m_collisionObjects[i];
						if( ( colObj.getCollisionFlags() & btCollisionObject.CollisionFlags.CF_DISABLE_VISUALIZE_OBJECT ) == 0 )
						{
							if( m_debugDrawer != null && ( m_debugDrawer.getDebugMode() & btIDebugDraw.DebugDrawModes.DBG_DrawWireframe ) != 0 )
							{
								btVector3 color = new btVector3( (double)( 0.4 ), (double)( 0.4 ), (double)( 0.4 ), 1 );

								switch( colObj.getActivationState() )
								{
									case ActivationState.ACTIVE_TAG:
										color = defaultColors.m_activeObject; break;
									case ActivationState.ISLAND_SLEEPING:
										color = defaultColors.m_deactivatedObject; break;
									case ActivationState.WANTS_DEACTIVATION:
										color = defaultColors.m_wantsDeactivationObject; break;
									case ActivationState.DISABLE_DEACTIVATION:
										color = defaultColors.m_disabledDeactivationObject; break;
									case ActivationState.DISABLE_SIMULATION:
										color = defaultColors.m_disabledSimulationObject; break;
									default:
										{
											color = new btVector3( (double)( .3 ), (double)( 0.3 ), (double)( 0.3 ), 1 );
										}
										break;
								};

								debugDrawObject( ref colObj.m_worldTransform, colObj.getCollisionShape(), color );
							}
							if( m_debugDrawer != null && ( m_debugDrawer.getDebugMode() & btIDebugDraw.DebugDrawModes.DBG_DrawAabb ) != 0 )
							{
								btVector3 minAabb, maxAabb;
								btVector3 colorvec = defaultColors.m_aabb;
								colObj.getCollisionShape().getAabb( ref colObj.m_worldTransform, out minAabb, out maxAabb );
								btVector3 contactThreshold = new btVector3( btPersistentManifold.gContactBreakingThreshold );
								minAabb -= contactThreshold;
								maxAabb += contactThreshold;

								btVector3 minAabb2, maxAabb2;

								if( getDispatchInfo().m_useContinuous && colObj.getInternalType() == btCollisionObject.CollisionObjectTypes.CO_RIGID_BODY && !colObj.isStaticOrKinematicObject() )
								{
									colObj.getCollisionShape().getAabb( ref colObj.m_interpolationWorldTransform, out minAabb2, out maxAabb2 );
									minAabb2 -= contactThreshold;
									maxAabb2 += contactThreshold;
									minAabb.setMin( ref minAabb2 );
									maxAabb.setMax( ref maxAabb2 );
								}

								m_debugDrawer.drawAabb( ref minAabb, ref maxAabb, ref colorvec );
							}
						}
					}
				}
			}
		}


#if SERIALIZE_DONE
void serializeCollisionObjects( btSerializer serializer )
{
	int i;

	///keep track of shapes already serialized
	btHashMap<btHashPtr, btCollisionShape> serializedShapes;

	for( i = 0; i < m_collisionObjects.Count; i++ )
	{
		btCollisionObject colObj = m_collisionObjects[i];
		btCollisionShape shape = colObj.getCollisionShape();

		if( !serializedShapes.find( shape ) )
		{
			serializedShapes.insert( shape, shape );
			shape.serializeSingleShape( serializer );
		}
	}

	//serialize all collision objects
	for( i = 0; i < m_collisionObjects.Count; i++ )
	{
		btCollisionObject colObj = m_collisionObjects[i];
		if( ( colObj.getInternalType() == btCollisionObject::CO_COLLISION_OBJECT ) || ( colObj.getInternalType() == btCollisionObject::CO_FEATHERSTONE_LINK ) )
		{
			colObj.serializeSingleObject( serializer );
		}
	}
}


void serialize( btSerializer serializer )
{

	serializer.startSerialization();

	serializeCollisionObjects( serializer );

	serializer.finishSerialization();
}
#endif
	}
}
