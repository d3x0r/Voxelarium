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
using Bullet.BulletCollision;
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

		public btCollisionWorld( btDispatcher dispatcher
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
					getBroadphase().getOverlappingPairCache().cleanProxyFromPairs( bp, m_dispatcher1 );
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

			btITransform m_colObjWorldTransform;

			internal BridgeTriangleRaycastCallback( ref btVector3 from, ref btVector3 to
				, RayResultCallback resultCallback
				, btCollisionObject collisionObject, btConcaveShape triangleMesh
				, btITransform colObjWorldTransform )
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

				btVector3 hitNormalWorld; m_colObjWorldTransform.getBasis().Apply( ref hitNormalLocal, out hitNormalWorld );

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
			btITransform m_colObjWorldTransform;
			btITransform m_rayFromTrans;
			btITransform m_rayToTrans;
			RayResultCallback m_resultCallback;

			internal RayTester( btCollisionObject collisionObject,
					btCompoundShape compoundShape,
					btITransform colObjWorldTransform,
					btITransform rayFromTrans,
					btITransform rayToTrans,
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
				btITransform childTrans = m_compoundShape.getChildTransform( i );
				btTransform childWorldTrans; m_colObjWorldTransform.Apply( childTrans, out childWorldTrans );

				btCollisionObjectWrapper tmpOb = new btCollisionObjectWrapper( null, childCollisionShape, m_collisionObject, childWorldTrans, -1, i );
				// replace collision shape so that callback can determine the triangle



				LocalInfoAdder2 my_cb = new LocalInfoAdder2( i, m_resultCallback );

				rayTestSingleInternal(
					m_rayFromTrans,
					m_rayToTrans,
					tmpOb,
					my_cb );

			}

			public void Process( btDbvtNode leaf )
			{
				Process( leaf.dataAsInt );
			}
			public void Process( btDbvtNode n, btDbvtNode n2 ) { }

			public void Process( btDbvtNode n, double f )
			{
				Process( n );
			}
			public bool Descent( btDbvtNode n )
			{
				return true;
			}
			public bool AllLeaves( btDbvtNode n )
			{
				return true;
			}

		};

		protected virtual void addCollisionObject( btCollisionObject collisionObject
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

			btDispatcherInfo dispatchInfo = m_dispatchInfo;

			updateAabbs();

			computeOverlappingPairs();

			btDispatcher dispatcher = getDispatcher();
			{
				CProfileSample sample2 = new CProfileSample( "dispatchAllCollisionPairs" );
				if( dispatcher != null )
					dispatcher.dispatchAllCollisionPairs( m_broadphasePairCache.getOverlappingPairCache(), dispatchInfo, m_dispatcher1 );
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
		public static void rayTestSingle( btITransform rayFromTrans, btITransform rayToTrans,
										btCollisionObject collisionObject,
											btCollisionShape collisionShape,
											btITransform colObjWorldTransform,
											RayResultCallback resultCallback )
		{
			btCollisionObjectWrapper colObWrap = new btCollisionObjectWrapper( null, collisionShape, collisionObject, colObjWorldTransform, -1, -1 );
			rayTestSingleInternal( rayFromTrans, rayToTrans, colObWrap, resultCallback );
		}





		public static void rayTestSingleInternal( btITransform rayFromTrans, btITransform rayToTrans,
												btCollisionObjectWrapper collisionObjectWrap,
												RayResultCallback resultCallback )
		{
			btSphereShape pointShape = new btSphereShape( btScalar.BT_ZERO );
			pointShape.setMargin( 0f );
			btConvexShape castShape = pointShape;
			btCollisionShape collisionShape = collisionObjectWrap.getCollisionShape();
			btITransform colObjWorldTransform = collisionObjectWrap.getWorldTransform();

			if( collisionShape.isConvex() )
			{
				//		CProfileSample sample = new CProfileSample("rayTestConvex");
				btConvexCast.CastResult castResult = new btConvexCast.CastResult();

				castResult.m_fraction = resultCallback.m_closestHitFraction;

				btConvexShape convexShape = (btConvexShape)collisionShape;
				btVoronoiSimplexSolver simplexSolver = new btVoronoiSimplexSolver();
				btSubsimplexConvexCast subSimplexConvexCaster = new btSubsimplexConvexCast( castShape, convexShape, simplexSolver );

				btGjkConvexCast gjkConvexCaster = new btGjkConvexCast( castShape, convexShape, simplexSolver );

				//btContinuousConvexCollision convexCaster(castShape,convexShape,&simplexSolver,0);

				btConvexCast convexCasterPtr = null;
				//use kF_UseSubSimplexConvexCastRaytest by default
				if( ( resultCallback.m_flags & (uint)btTriangleRaycastCallback.EFlags.kF_UseGjkConvexCastRaytest ) != 0 )
					convexCasterPtr = gjkConvexCaster;
				else
					convexCasterPtr = subSimplexConvexCaster;

				btConvexCast convexCaster = convexCasterPtr;

				if( convexCaster.calcTimeOfImpact( rayFromTrans, rayToTrans, colObjWorldTransform, colObjWorldTransform, castResult ) )
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
								collisionObjectWrap.getCollisionObject(),
								null,
								castResult.m_normal,
								castResult.m_fraction
								);

							bool normalInWorldSpace = true;
							resultCallback.addSingleResult( localRayResult, normalInWorldSpace );

						}
					}
				}
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


						BridgeTriangleRaycastCallback rcb = new BridgeTriangleRaycastCallback( ref rayFromLocal, ref rayToLocal, resultCallback, collisionObjectWrap.getCollisionObject(), concaveShape, colObjWorldTransform );
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
							collisionObjectWrap.getCollisionObject(),
									compoundShape,
									colObjWorldTransform,
									rayFromTrans,
									rayToTrans,
									resultCallback );
#if !DISABLE_DBVT_COMPOUNDSHAPE_RAYCAST_ACCELERATION
						if( dbvt != null )
						{
							btTransform tmp;
							colObjWorldTransform.inverseTimes( rayFromTrans, out tmp );
							btVector3 localRayFrom; tmp.getOrigin( out localRayFrom );
							colObjWorldTransform.inverseTimes( rayToTrans, out tmp );
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
		}

		internal class BridgeTriangleConvexcastCallback : btTriangleConvexcastCallback
		{
			ConvexResultCallback m_resultCallback;
			btCollisionObject m_collisionObject;
			btTriangleMeshShape m_triangleMesh;

			internal BridgeTriangleConvexcastCallback( btConvexShape castShape, ref btTransform from, ref btTransform to,
				ConvexResultCallback resultCallback, btCollisionObject collisionObject
				, btTriangleMeshShape triangleMesh, ref btTransform triangleToWorld ) :
                    base( castShape, from, to, triangleToWorld, triangleMesh.getMargin())
			{
                        m_resultCallback = ( resultCallback);
                        m_collisionObject = ( collisionObject);
                        m_triangleMesh = ( triangleMesh);
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
						hitNormalLocal,
						hitPointLocal,
						hitFraction);

					bool normalInWorldSpace = true;


					return m_resultCallback.addSingleResult( convexResult, normalInWorldSpace );
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
			btCollisionObjectWrapper tmpOb = new btCollisionObjectWrapper( null, collisionShape, collisionObject, colObjWorldTransform, -1, -1 );
			objectQuerySingleInternal( castShape, ref convexFromTrans, ref convexToTrans, tmpOb, resultCallback, allowedPenetration );
		}

		public static void objectQuerySingleInternal( btConvexShape castShape, ref btTransform convexFromTrans, ref btTransform convexToTrans,
													btCollisionObjectWrapper colObjWrap,
													ConvexResultCallback resultCallback, double allowedPenetration )
		{
			btCollisionShape collisionShape = colObjWrap.getCollisionShape();
			btITransform colObjWorldTransform = colObjWrap.getWorldTransform();

			if( collisionShape.isConvex() )
			{
				//CProfileSample sample = new CProfileSample("convexSweepConvex");
				btConvexCast.CastResult castResult = new btConvexCast.CastResult();
				castResult.m_allowedPenetration = allowedPenetration;
				castResult.m_fraction = resultCallback.m_closestHitFraction;//btScalar.BT_ONE;//??

				btConvexShape convexShape = (btConvexShape)collisionShape;
				btVoronoiSimplexSolver simplexSolver = new btVoronoiSimplexSolver();
				btGjkEpaPenetrationDepthSolver gjkEpaPenetrationSolver = new btGjkEpaPenetrationDepthSolver();

				btContinuousConvexCollision convexCaster1 = new btContinuousConvexCollision( castShape, convexShape, simplexSolver, gjkEpaPenetrationSolver );
				//btGjkConvexCast convexCaster2(castShape,convexShape,&simplexSolver);
				//btSubsimplexConvexCast convexCaster3(castShape,convexShape,&simplexSolver);

				btConvexCast castPtr = convexCaster1;

				if( castPtr.calcTimeOfImpact( convexFromTrans, convexToTrans, colObjWorldTransform, colObjWorldTransform, castResult ) )
				{
					//add hit
					if( castResult.m_normal.length2() > (double)( 0.0001 ) )
					{
						if( castResult.m_fraction < resultCallback.m_closestHitFraction )
						{
							castResult.m_normal.normalize();
							LocalConvexResult localConvexResult = new LocalConvexResult
								(
								colObjWrap.getCollisionObject(),
								null,
								castResult.m_normal,
								castResult.m_hitPoint,
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
				if( collisionShape.isConcave() )
				{
					if( collisionShape.getShapeType() == BroadphaseNativeTypes.TRIANGLE_MESH_SHAPE_PROXYTYPE )
					{
						//CProfileSample sample = new CProfileSample("convexSweepbtBvhTriangleMesh");
						btBvhTriangleMeshShape triangleMesh = (btBvhTriangleMeshShape)collisionShape;
						btTransform worldTocollisionObject = colObjWorldTransform.inverse();
						btVector3 convexFromLocal = worldTocollisionObject * convexFromTrans.getOrigin();
						btVector3 convexToLocal = worldTocollisionObject * convexToTrans.getOrigin();
						// rotation of box in local mesh space = MeshRotation^-1  ConvexToRotation
						btTransform rotationXform = ( worldTocollisionObject.getBasis() * convexToTrans.getBasis() );

						//ConvexCast::CastResult

						BridgeTriangleConvexcastCallback tccb = new BridgeTriangleConvexcastCallback( castShape, convexFromTrans, convexToTrans
							, resultCallback, colObjWrap.getCollisionObject()
							, triangleMesh, colObjWorldTransform);
						tccb.m_hitFraction = resultCallback.m_closestHitFraction;
						tccb.m_allowedPenetration = allowedPenetration;
						btVector3 boxMinLocal, boxMaxLocal;
						castShape.getAabb( rotationXform, boxMinLocal, boxMaxLocal );
						triangleMesh.performConvexcast( &tccb, convexFromLocal, convexToLocal, boxMinLocal, boxMaxLocal );
					}
					else
					{
						if( collisionShape.getShapeType() == STATIC_PLANE_PROXYTYPE )
						{
							btConvexCast::CastResult castResult;
							castResult.m_allowedPenetration = allowedPenetration;
							castResult.m_fraction = resultCallback.m_closestHitFraction;
							btStaticPlaneShape planeShape = (btStaticPlaneShape)collisionShape;
							btContinuousConvexCollision convexCaster1( castShape, planeShape );
							btConvexCast castPtr = &convexCaster1;

							if( castPtr.calcTimeOfImpact( convexFromTrans, convexToTrans, colObjWorldTransform, colObjWorldTransform, castResult ) )
							{
								//add hit
								if( castResult.m_normal.length2() > (double)( 0.0001 ) )
								{
									if( castResult.m_fraction < resultCallback.m_closestHitFraction )
									{
										castResult.m_normal.normalize();
										LocalConvexResult localConvexResult
		                                    (
											colObjWrap.getCollisionObject(),
											0,
											castResult.m_normal,
											castResult.m_hitPoint,
											castResult.m_fraction
											);

										bool normalInWorldSpace = true;
										resultCallback.addSingleResult( localConvexResult, normalInWorldSpace );
									}
								}
							}

						}
						else
						{
							//CProfileSample sample = new CProfileSample("convexSweepConcave");
							btConcaveShape concaveShape = (btConcaveShape)collisionShape;
							btTransform worldTocollisionObject = colObjWorldTransform.inverse();
							btVector3 convexFromLocal = worldTocollisionObject  convexFromTrans.getOrigin();
							btVector3 convexToLocal = worldTocollisionObject  convexToTrans.getOrigin();
							// rotation of box in local mesh space = MeshRotation^-1  ConvexToRotation
							btTransform rotationXform = btTransform( worldTocollisionObject.getBasis() * convexToTrans.getBasis() );


							BridgeTriangleConvexcastCallback tccb( castShape, convexFromTrans, convexToTrans,&resultCallback, colObjWrap.getCollisionObject(), concaveShape, colObjWorldTransform);
							tccb.m_hitFraction = resultCallback.m_closestHitFraction;
							tccb.m_allowedPenetration = allowedPenetration;
							btVector3 boxMinLocal, boxMaxLocal;
							castShape.getAabb( rotationXform, boxMinLocal, boxMaxLocal );

							btVector3 rayAabbMinLocal = convexFromLocal;
							rayAabbMinLocal.setMin( convexToLocal );
							btVector3 rayAabbMaxLocal = convexFromLocal;
							rayAabbMaxLocal.setMax( convexToLocal );
							rayAabbMinLocal += boxMinLocal;
							rayAabbMaxLocal += boxMaxLocal;
							concaveShape.processAllTriangles( &tccb, rayAabbMinLocal, rayAabbMaxLocal );
						}
					}
				}
				else
				{
					///@todo : use AABB tree or other BVH acceleration structure!
					if( collisionShape.isCompound() )
					{
						CProfileSample sample = new CProfileSample( "convexSweepCompound" );
						btCompoundShape compoundShape = static_cast<btCompoundShape>( collisionShape );
						int i = 0;
						for( i = 0; i < compoundShape.getNumChildShapes(); i++ )
						{
							btTransform childTrans = compoundShape.getChildTransform( i );
							btCollisionShape childCollisionShape = compoundShape.getChildShape( i );
							btTransform childWorldTrans = colObjWorldTransform  childTrans;

					struct LocalInfoAdder : public ConvexResultCallback {
                            ConvexResultCallback m_userCallback;
		int m_i;

							LocalInfoAdder( int i, ConvexResultCallback user )
								: m_userCallback( user), m_i( i)
		{
			m_closestHitFraction = m_userCallback.m_closestHitFraction;
		}
		virtual bool needsCollision( btBroadphaseProxy p )
		{
			return m_userCallback.needsCollision( p );
		}
		virtual double addSingleResult( LocalConvexResult&	r, bool b )
		{
			LocalShapeInfo shapeInfo;
			shapeInfo.m_shapePart = -1;
			shapeInfo.m_triangleIndex = m_i;
			if( r.m_localShapeInfo == NULL )
				r.m_localShapeInfo = &shapeInfo;
			stringbtScalar result = m_userCallback.addSingleResult( r, b );
			m_closestHitFraction = m_userCallback.m_closestHitFraction;
			return result;

		}
	};

	LocalInfoAdder my_cb( i, &resultCallback);

	btCollisionObjectWrapper tmpObj( colObjWrap, childCollisionShape, colObjWrap.getCollisionObject(),childWorldTrans,-1,i);

					objectQuerySingleInternal( castShape, convexFromTrans, convexToTrans,
						&tmpObj, my_cb, allowedPenetration);

}
			}
		}
	}
}


public class btSingleRayCallback : btBroadphaseRayCallback
{

	btVector3 m_rayFromWorld;
	btVector3 m_rayToWorld;
	btTransform m_rayFromTrans;
	btTransform m_rayToTrans;
	btVector3 m_hitNormal;

	btCollisionWorld m_world;
	RayResultCallback&	m_resultCallback;

	btSingleRayCallback( btVector3 rayFromWorld, btVector3 rayToWorld, btCollisionWorld world, RayResultCallback& resultCallback)
		:m_rayFromWorld( rayFromWorld),
        m_rayToWorld( rayToWorld),
        m_world( world),
        m_resultCallback( resultCallback)
	{
		m_rayFromTrans.setIdentity();
		m_rayFromTrans.setOrigin( m_rayFromWorld );
		m_rayToTrans.setIdentity();
		m_rayToTrans.setOrigin( m_rayToWorld );

		btVector3 rayDir = ( rayToWorld - rayFromWorld );

		rayDir.normalize();
		///what about division by zero? -. just set rayDirection[i] to INF/BT_LARGE_FLOAT
		m_rayDirectionInverse[0] = rayDir[0] == btScalar.BT_ZERO ? (double)( BT_LARGE_FLOAT ) : (double)( 1.0 ) / rayDir[0];
		m_rayDirectionInverse[1] = rayDir[1] == btScalar.BT_ZERO ? (double)( BT_LARGE_FLOAT ) : (double)( 1.0 ) / rayDir[1];
		m_rayDirectionInverse[2] = rayDir[2] == btScalar.BT_ZERO ? (double)( BT_LARGE_FLOAT ) : (double)( 1.0 ) / rayDir[2];
		m_signs[0] = m_rayDirectionInverse[0] < 0.0;
		m_signs[1] = m_rayDirectionInverse[1] < 0.0;
		m_signs[2] = m_rayDirectionInverse[2] < 0.0;

		m_lambda_max = rayDir.dot( m_rayToWorld - m_rayFromWorld );

	}



	virtual bool process( stringbtBroadphaseProxy proxy )
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
#if 0
# ifdef RECALCULATE_AABB
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
				m_world.rayTestSingle( m_rayFromTrans, m_rayToTrans,
					collisionObject,
					collisionObject.getCollisionShape(),
					collisionObject.getWorldTransform(),
					m_resultCallback );
			}
		}
		return true;
	}
};

/// rayTest performs a raycast on all objects in the btCollisionWorld, and calls the resultCallback
/// This allows for several queries: first hit, all hits, any hit, dependent on the value returned by the callback.
void rayTest( btVector3 rayFromWorld, btVector3 rayToWorld, RayResultCallback& resultCallback)
{
	//CProfileSample sample = new CProfileSample("rayTest");
	/// use the broadphase to accelerate the search for objects, based on their aabb
	/// and for each object with ray-aabb overlap, perform an exact ray test
	btSingleRayCallback rayCB( rayFromWorld, rayToWorld,this, resultCallback);

#if !USE_BRUTEFORCE_RAYBROADPHASE
	m_broadphasePairCache.rayTest( rayFromWorld, rayToWorld, rayCB );
#else
	for( int i = 0; i < this.getNumCollisionObjects(); i++ )
	{
		rayCB.process( m_collisionObjects[i].getBroadphaseHandle() );
	}
#endif //USE_BRUTEFORCE_RAYBROADPHASE

}


struct btSingleSweepCallback : btBroadphaseRayCallback
{

	btTransform m_convexFromTrans;
	btTransform m_convexToTrans;
	btVector3 m_hitNormal;
	btCollisionWorld m_world;
	ConvexResultCallback&	m_resultCallback;
	double m_allowedCcdPenetration;
	btConvexShape m_castShape;


	btSingleSweepCallback( btConvexShape castShape, btTransform& convexFromTrans, btTransform& convexToTrans, btCollisionWorld world, ConvexResultCallback& resultCallback, double allowedPenetration )
		:m_convexFromTrans( convexFromTrans),
        m_convexToTrans( convexToTrans),
        m_world( world),
        m_resultCallback( resultCallback),
        m_allowedCcdPenetration( allowedPenetration),
        m_castShape( castShape)
	{
		btVector3 unnormalizedRayDir = ( m_convexToTrans.getOrigin() - m_convexFromTrans.getOrigin() );
		btVector3 rayDir = unnormalizedRayDir.normalized();
		///what about division by zero? -. just set rayDirection[i] to INF/BT_LARGE_FLOAT
		m_rayDirectionInverse[0] = rayDir[0] == btScalar.BT_ZERO ? (double)( BT_LARGE_FLOAT ) : (double)( 1.0 ) / rayDir[0];
		m_rayDirectionInverse[1] = rayDir[1] == btScalar.BT_ZERO ? (double)( BT_LARGE_FLOAT ) : (double)( 1.0 ) / rayDir[1];
		m_rayDirectionInverse[2] = rayDir[2] == btScalar.BT_ZERO ? (double)( BT_LARGE_FLOAT ) : (double)( 1.0 ) / rayDir[2];
		m_signs[0] = m_rayDirectionInverse[0] < 0.0;
		m_signs[1] = m_rayDirectionInverse[1] < 0.0;
		m_signs[2] = m_rayDirectionInverse[2] < 0.0;

		m_lambda_max = rayDir.dot( unnormalizedRayDir );

	}

	virtual bool process( stringbtBroadphaseProxy proxy )
	{
		///terminate further convex sweep tests, once the closestHitFraction reached zero
		if( m_resultCallback.m_closestHitFraction == (double)( 0f ) )
			return false;

		btCollisionObject collisionObject = (btCollisionObject)proxy.m_clientObject;

		//only perform raycast if filterMask matches
		if( m_resultCallback.needsCollision( collisionObject.getBroadphaseHandle() ) )
		{
			//RigidcollisionObject collisionObject = ctrl.GetRigidcollisionObject();
			m_world.objectQuerySingle( m_castShape, m_convexFromTrans, m_convexToTrans,
				collisionObject,
				collisionObject.getCollisionShape(),
				collisionObject.getWorldTransform(),
				m_resultCallback,
				m_allowedCcdPenetration );
		}

		return true;
	}
};



/// convexTest performs a swept convex cast on all objects in the btCollisionWorld, and calls the resultCallback
/// This allows for several queries: first hit, all hits, any hit, dependent on the value return by the callback.
void convexSweepTest( btConvexShape castShape, ref btTransform convexFromWorld, ref btTransform convexToWorld, ConvexResultCallback resultCallback, double allowedCcdPenetration )
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

	btSingleSweepCallback convexCB( castShape, convexFromWorld, convexToWorld,this, resultCallback, allowedCcdPenetration);

	m_broadphasePairCache.rayTest( convexFromTrans.getOrigin(), convexToTrans.getOrigin(), convexCB, castShapeAabbMin, castShapeAabbMax );

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



struct btBridgedManifoldResult : btManifoldResult
{

	ContactResultCallback m_resultCallback;

	btBridgedManifoldResult( btCollisionObjectWrapper obj0Wrap, btCollisionObjectWrapper obj1Wrap, ContactResultCallback& resultCallback )
		:btManifoldResult( obj0Wrap, obj1Wrap),
        m_resultCallback( resultCallback)
	{
	}

	virtual void addContactPoint( btVector3 normalOnBInWorld, btVector3 pointInWorld, double depth )
	{
		bool isSwapped = m_manifoldPtr.getBody0() != m_body0Wrap.getCollisionObject();
		btVector3 pointA = pointInWorld + normalOnBInWorld  depth;
		btVector3 localA;
		btVector3 localB;
		if( isSwapped )
		{
			localA = m_body1Wrap.getCollisionObject().getWorldTransform().invXform( pointA );
			localB = m_body0Wrap.getCollisionObject().getWorldTransform().invXform( pointInWorld );
		}
		else
		{
			localA = m_body0Wrap.getCollisionObject().getWorldTransform().invXform( pointA );
			localB = m_body1Wrap.getCollisionObject().getWorldTransform().invXform( pointInWorld );
		}

		btManifoldPoint newPt( localA, localB, normalOnBInWorld, depth );
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
	ContactResultCallback&	m_resultCallback;


	btSingleContactCallback( btCollisionObject collisionObject, btCollisionWorld world, ContactResultCallback& resultCallback)
		:m_collisionObject( collisionObject),
        m_world( world),
        m_resultCallback( resultCallback)
	{
	}

	virtual bool process( stringbtBroadphaseProxy proxy )
	{
		btCollisionObject collisionObject = (btCollisionObject)proxy.m_clientObject;
		if( collisionObject == m_collisionObject )
			return true;

		//only perform raycast if filterMask matches
		if( m_resultCallback.needsCollision( collisionObject.getBroadphaseHandle() ) )
		{
			btCollisionObjectWrapper ob0( 0, m_collisionObject.getCollisionShape(), m_collisionObject, m_collisionObject.getWorldTransform(), -1, -1);
			btCollisionObjectWrapper ob1( 0, collisionObject.getCollisionShape(), collisionObject, collisionObject.getWorldTransform(), -1, -1);

			btCollisionAlgorithm algorithm = m_world.getDispatcher().findAlgorithm( &ob0, &ob1 );
			if( algorithm )
			{
				btBridgedManifoldResult contactPointResult( &ob0, &ob1, m_resultCallback);
				//discrete collision detection query

				algorithm.processCollision( &ob0, &ob1, m_world.getDispatchInfo(), &contactPointResult );

				algorithm.~btCollisionAlgorithm();
				m_world.getDispatcher().freeCollisionAlgorithm( algorithm );
			}
		}
		return true;
	}
};


///contactTest performs a discrete collision test against all objects in the btCollisionWorld, and calls the resultCallback.
///it reports one or more contact points for every overlapping object (including the one with deepest penetration)
void contactTest( btCollisionObject colObj, ContactResultCallback& resultCallback)
{
	btVector3 aabbMin, aabbMax;
	colObj.getCollisionShape().getAabb( colObj.getWorldTransform(), aabbMin, aabbMax );
	btSingleContactCallback contactCB( colObj,this, resultCallback);

	m_broadphasePairCache.aabbTest( aabbMin, aabbMax, contactCB );
}


///contactTest performs a discrete collision test between two collision objects and calls the resultCallback if overlap if detected.
///it reports one or more contact points (including the one with deepest penetration)
void contactPairTest( btCollisionObject colObjA, btCollisionObject colObjB, ContactResultCallback& resultCallback)
{
	btCollisionObjectWrapper obA( 0, colObjA.getCollisionShape(), colObjA, colObjA.getWorldTransform(), -1, -1);
	btCollisionObjectWrapper obB( 0, colObjB.getCollisionShape(), colObjB, colObjB.getWorldTransform(), -1, -1);

	btCollisionAlgorithm algorithm = getDispatcher().findAlgorithm( &obA, &obB );
	if( algorithm )
	{
		btBridgedManifoldResult contactPointResult( &obA, &obB, resultCallback);
		//discrete collision detection query
		algorithm.processCollision( &obA, &obB, getDispatchInfo(), &contactPointResult );

		algorithm.~btCollisionAlgorithm();
		getDispatcher().freeCollisionAlgorithm( algorithm );
	}

}




class DebugDrawcallback : btTriangleCallback, public btInternalTriangleIndexCallback
{
	btIDebugDraw m_debugDrawer;
btVector3 m_color;
btTransform m_worldTrans;

public:

	DebugDrawcallback( btIDebugDraw debugDrawer, btTransform& worldTrans, btVector3 color ) :
      m_debugDrawer( debugDrawer),
          m_color( color),
          m_worldTrans( worldTrans)
{
}

virtual void internalProcessTriangleIndex( btVector3 triangle, int partId, int triangleIndex )
{
	processTriangle( triangle, partId, triangleIndex );
}

virtual void processTriangle( btVector3 triangle, int partId, int triangleIndex )
{
	(void)partId;
	(void)triangleIndex;

	btVector3 wv0, wv1, wv2;
	wv0 = m_worldTranstriangle;
	wv1 = m_worldTranstriangle[1];
	wv2 = m_worldTranstriangle[2];
	btVector3 center = ( wv0 + wv1 + wv2 )( double )( 1./ 3.);

	if( m_debugDrawer.getDebugMode() & btIDebugDraw::DBG_DrawNormals )
	{
		btVector3 normal = ( wv1 - wv0 ).cross( wv2 - wv0 );
		normal.normalize();
		btVector3 normalColor( 1, 1, 0);
		m_debugDrawer.drawLine( center, center + normal, normalColor );
	}
	m_debugDrawer.drawLine( wv0, wv1, m_color );
	m_debugDrawer.drawLine( wv1, wv2, m_color );
	m_debugDrawer.drawLine( wv2, wv0, m_color );
}
};


public virtual void debugDrawObject( btTransform& worldTransform, btCollisionShape shape, btVector3 color )
{
	// Draw a small simplex at the center of the object
	if( getDebugDrawer() && getDebugDrawer().getDebugMode() & btIDebugDraw::DBG_DrawFrames )
	{
		getDebugDrawer().drawTransform( worldTransform, 1 );
	}

	if( shape.getShapeType() == COMPOUND_SHAPE_PROXYTYPE )
	{
		btCompoundShape compoundShape = static_cast<btCompoundShape>( shape );
		for( int i = compoundShape.getNumChildShapes() - 1; i >= 0; i-- )
		{
			btTransform childTrans = compoundShape.getChildTransform( i );
			btCollisionShape colShape = compoundShape.getChildShape( i );
			debugDrawObject( worldTransformchildTrans, colShape, color );
		}

	}
	else
	{

		switch( shape.getShapeType() )
		{

			case BOX_SHAPE_PROXYTYPE:
				{
					stringbtBoxShape boxShape = static_cast<stringbtBoxShape>( shape );
					btVector3 halfExtents = boxShape.getHalfExtentsWithMargin();
					getDebugDrawer().drawBox( -halfExtents, halfExtents, worldTransform, color );
					break;
				}

			case SPHERE_SHAPE_PROXYTYPE:
				{
					stringbtSphereShape sphereShape = static_cast<stringbtSphereShape>( shape );
					double radius = sphereShape.getMargin();//radius doesn't include the margin, so draw with margin

					getDebugDrawer().drawSphere( radius, worldTransform, color );
					break;
				}
			case MULTI_SPHERE_SHAPE_PROXYTYPE:
				{
					stringbtMultiSphereShape multiSphereShape = static_cast<stringbtMultiSphereShape>( shape );

					btTransform childTransform;
					childTransform.setIdentity();

					for( int i = multiSphereShape.getSphereCount() - 1; i >= 0; i-- )
					{
						childTransform.setOrigin( multiSphereShape.getSpherePosition( i ) );
						getDebugDrawer().drawSphere( multiSphereShape.getSphereRadius( i ), worldTransformchildTransform, color );
					}

					break;
				}
			case CAPSULE_SHAPE_PROXYTYPE:
				{
					stringbtCapsuleShape capsuleShape = static_cast<stringbtCapsuleShape>( shape );

					double radius = capsuleShape.getRadius();
					double halfHeight = capsuleShape.getHalfHeight();

					int upAxis = capsuleShape.getUpAxis();
					getDebugDrawer().drawCapsule( radius, halfHeight, upAxis, worldTransform, color );
					break;
				}
			case CONE_SHAPE_PROXYTYPE:
				{
					stringbtConeShape coneShape = static_cast<stringbtConeShape>( shape );
					double radius = coneShape.getRadius();//+coneShape.getMargin();
					double height = coneShape.getHeight();//+coneShape.getMargin();

					int upAxis = coneShape.getConeUpIndex();
					getDebugDrawer().drawCone( radius, height, upAxis, worldTransform, color );
					break;

				}
			case CYLINDER_SHAPE_PROXYTYPE:
				{
					stringbtCylinderShape cylinder = static_cast<stringbtCylinderShape>( shape );
					int upAxis = cylinder.getUpAxis();
					double radius = cylinder.getRadius();
					double halfHeight = cylinder.getHalfExtentsWithMargin()[upAxis];
					getDebugDrawer().drawCylinder( radius, halfHeight, upAxis, worldTransform, color );
					break;
				}

			case STATIC_PLANE_PROXYTYPE:
				{
					stringbtStaticPlaneShape staticPlaneShape = static_cast<stringbtStaticPlaneShape>( shape );
					double planeConst = staticPlaneShape.getPlaneConstant();
					btVector3 planeNormal = staticPlaneShape.getPlaneNormal();
					getDebugDrawer().drawPlane( planeNormal, planeConst, worldTransform, color );
					break;

				}
			default:
				{

					/// for polyhedral shapes
					if( shape.isPolyhedral() )
					{
						btPolyhedralConvexShape polyshape = (btPolyhedralConvexShape)shape;

						int i;
						if( polyshape.getConvexPolyhedron() )
						{
							stringbtConvexPolyhedron poly = polyshape.getConvexPolyhedron();
							for( i = 0; i < poly.m_faces.Count; i++ )
							{
								btVector3 centroid( 0, 0, 0);
							int numVerts = poly.m_faces[i].m_indices.Count;
							if( numVerts )
							{
								int lastV = poly.m_faces[i].m_indices[numVerts - 1];
								for( int v = 0; v < poly.m_faces[i].m_indices.Count; v++ )
								{
									int curVert = poly.m_faces[i].m_indices[v];
									centroid += poly.m_vertices[curVert];
									getDebugDrawer().drawLine( worldTransformpoly.m_vertices[lastV], worldTransformpoly.m_vertices[curVert], color );
									lastV = curVert;
								}
							}
							centroid = (double)( 1 ) / (double)( numVerts );
							if( getDebugDrawer().getDebugMode() & btIDebugDraw::DBG_DrawNormals )
							{
								btVector3 normalColor( 1, 1, 0);
								btVector3 faceNormal( poly.m_faces[i].m_plane[0], poly.m_faces[i].m_plane[1], poly.m_faces[i].m_plane[2]);
								getDebugDrawer().drawLine( worldTransformcentroid, worldTransform( centroid + faceNormal ), normalColor );
							}

						}


					}
					else
					{
						for( i = 0; i < polyshape.getNumEdges(); i++ )
						{
							btVector3 a, b;
							polyshape.getEdge( i, a, b );
							btVector3 wa = worldTransform  a;
							btVector3 wb = worldTransform  b;
							getDebugDrawer().drawLine( wa, wb, color );
						}
					}


				}

				if( shape.isConcave() )
				{
					btConcaveShape concaveMesh = (btConcaveShape)shape;

					///@todo pass camera, for some culling? no . we are not a graphics lib
					btVector3 aabbMax( (double)( BT_LARGE_FLOAT ), (double)( BT_LARGE_FLOAT ), (double)( BT_LARGE_FLOAT ));
					btVector3 aabbMin( (double)( -BT_LARGE_FLOAT ), (double)( -BT_LARGE_FLOAT ), (double)( -BT_LARGE_FLOAT ));

					DebugDrawcallback drawCallback( getDebugDrawer(), worldTransform, color);
					concaveMesh.processAllTriangles( &drawCallback, aabbMin, aabbMax );

				}

				if( shape.getShapeType() == CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE )
				{
					btConvexTriangleMeshShape convexMesh = (btConvexTriangleMeshShape)shape;
					//todo: pass camera for some culling			
					btVector3 aabbMax( (double)( BT_LARGE_FLOAT ), (double)( BT_LARGE_FLOAT ), (double)( BT_LARGE_FLOAT ));
					btVector3 aabbMin( (double)( -BT_LARGE_FLOAT ), (double)( -BT_LARGE_FLOAT ), (double)( -BT_LARGE_FLOAT ));
					//DebugDrawcallback drawCallback;
					DebugDrawcallback drawCallback( getDebugDrawer(), worldTransform, color);
					convexMesh.getMeshInterface().InternalProcessAllTriangles( &drawCallback, aabbMin, aabbMax );
				}



		}

	}
}
}


public virtual void debugDrawWorld()
{
	if( getDebugDrawer() )
	{
		btIDebugDraw::DefaultColors defaultColors = getDebugDrawer().getDefaultColors();

		if( getDebugDrawer().getDebugMode() & btIDebugDraw::DBG_DrawContactPoints )
		{


			if( getDispatcher() )
			{
				int numManifolds = getDispatcher().getNumManifolds();

				for( int i = 0; i < numManifolds; i++ )
				{
					btPersistentManifold contactManifold = getDispatcher().getManifoldByIndexInternal( i );
					//btCollisionObject obA = static_cast<btCollisionObject>(contactManifold.getBody0());
					//btCollisionObject obB = static_cast<btCollisionObject>(contactManifold.getBody1());

					int numContacts = contactManifold.getNumContacts();
					for( int j = 0; j < numContacts; j++ )
					{
						btManifoldPoint & cp = contactManifold.getContactPoint( j );
						getDebugDrawer().drawContactPoint( cp.m_positionWorldOnB, cp.m_normalWorldOnB, cp.getDistance(), cp.getLifeTime(), defaultColors.m_contactPoint );
					}
				}
			}
		}

		if( ( getDebugDrawer().getDebugMode() & ( btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawAabb ) ) )
		{
			int i;

			for( i = 0; i < m_collisionObjects.Count; i++ )
			{
				btCollisionObject colObj = m_collisionObjects[i];
				if( ( colObj.getCollisionFlags() & btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT ) == 0 )
				{
					if( getDebugDrawer() && ( getDebugDrawer().getDebugMode() & btIDebugDraw::DBG_DrawWireframe ) )
					{
						btVector3 color( (double)( 0.4 ), (double)( 0.4 ), (double)( 0.4 ));

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
						color = btVector3( (double)( .3 ), (double)( 0.3 ), (double)( 0.3 ) );
					}
			};

			debugDrawObject( colObj.getWorldTransform(), colObj.getCollisionShape(), color );
		}
		if( m_debugDrawer && ( m_debugDrawer.getDebugMode() & btIDebugDraw::DBG_DrawAabb ) )
		{
			btVector3 minAabb, maxAabb;
			btVector3 colorvec = defaultColors.m_aabb;
			colObj.getCollisionShape().getAabb( colObj.getWorldTransform(), minAabb, maxAabb );
			btVector3 contactThreshold( btPersistentManifold.gContactBreakingThreshold, btPersistentManifold.gContactBreakingThreshold, btPersistentManifold.gContactBreakingThreshold );
			minAabb -= contactThreshold;
			maxAabb += contactThreshold;

			btVector3 minAabb2, maxAabb2;

			if( getDispatchInfo().m_useContinuous && colObj.getInternalType() == btCollisionObject::CO_RIGID_BODY && !colObj.isStaticOrKinematicObject() )
			{
				colObj.getCollisionShape().getAabb( colObj.getInterpolationWorldTransform(), minAabb2, maxAabb2 );
				minAabb2 -= contactThreshold;
				maxAabb2 += contactThreshold;
				minAabb.setMin( minAabb2 );
				maxAabb.setMax( maxAabb2 );
			}

			m_debugDrawer.drawAabb( minAabb, maxAabb, colorvec );
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