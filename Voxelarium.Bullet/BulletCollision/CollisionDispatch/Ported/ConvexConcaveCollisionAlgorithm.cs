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

	///For each triangle in the concave mesh that overlaps with the AABB of a convex (m_convexProxy), processTriangle is called.
	internal class btConvexTriangleCallback : btTriangleCallback
	{
		btCollisionObjectWrapper m_convexBodyWrap;
		btCollisionObjectWrapper m_triBodyWrap;

		internal btVector3 m_aabbMin;
		internal btVector3 m_aabbMax;


		btManifoldResult m_resultOut;
		btDispatcher m_dispatcher;
		btDispatcherInfo m_dispatchInfoPtr;
		double m_collisionMarginTriangle;

		//public int m_triangleCount;

		public btPersistentManifold m_manifoldPtr;

		public void clearWrapperData()
		{
			m_convexBodyWrap = null;
			m_triBodyWrap = null;
		}

#if InterfacesDidntSuck
		public btIVector3 getAabbMin()
		{
			return m_aabbMin;
		}
		public btIVector3 getAabbMax()
		{
			return m_aabbMax;
		}
#endif

		public btConvexTriangleCallback() { }

		public void Initialize( btDispatcher dispatcher, btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap, bool isSwapped )
		{
			m_dispatcher = ( dispatcher );
			m_dispatchInfoPtr = ( null );
			m_convexBodyWrap = isSwapped ? body1Wrap : body0Wrap;
			m_triBodyWrap = isSwapped ? body0Wrap : body1Wrap;

			//
			// create the manifold from the dispatcher 'manifold pool'
			//
			m_manifoldPtr = m_dispatcher.getNewManifold( m_convexBodyWrap.m_collisionObject, m_triBodyWrap.m_collisionObject );

			clearCache();
		}

		~btConvexTriangleCallback()
		{
			clearCache();
			m_dispatcher.releaseManifold( m_manifoldPtr );

		}


		public void clearCache()
		{
			m_dispatcher.clearManifold( m_manifoldPtr );
		}


		public void processTriangle( btVector3[] triangle, int partId, int triangleIndex )
		{

			if( !btAabbUtil.TestTriangleAgainstAabb2( triangle, ref m_aabbMin,ref  m_aabbMax ) )
			{
				return;
			}

			//just for debugging purposes
			//Console.WriteLine("triangle %d",m_triangleCount++);



			btCollisionAlgorithmConstructionInfo ci;
			ci.m_dispatcher1 = m_dispatcher;



#if false
	
	///debug drawing of the overlapping triangles
	if (m_dispatchInfoPtr && m_dispatchInfoPtr.m_debugDraw && (m_dispatchInfoPtr.m_debugDraw.getDebugMode() &btIDebugDraw::DBG_DrawWireframe ))
	{
		btCollisionObject ob = const_cast<btCollisionObject>(m_triBodyWrap.m_collisionObject);
		btVector3 color(1,1,0);
		ref btTransform tr = ob.getWorldTransform();
		m_dispatchInfoPtr.m_debugDraw.drawLine(tr(triangle),tr(triangle[1]),color);
		m_dispatchInfoPtr.m_debugDraw.drawLine(tr(triangle[1]),tr(triangle[2]),color);
		m_dispatchInfoPtr.m_debugDraw.drawLine(tr(triangle[2]),tr(triangle[0]),color);
	}
#endif

			if( m_convexBodyWrap.getCollisionShape().isConvex() )
			{
				btTriangleShape tm = BulletGlobals.TriangleShapePool.Get();
				tm.Initialize( ref triangle[0], ref triangle[1], ref triangle[2] );
				tm.setMargin( m_collisionMarginTriangle );


				btCollisionObjectWrapper triObWrap = BulletGlobals.CollisionObjectWrapperPool.Get();
				triObWrap.Initialize( m_triBodyWrap, tm, m_triBodyWrap.m_collisionObject,ref m_triBodyWrap.m_worldTransform, partId, triangleIndex );//correct transform?
				btCollisionAlgorithm colAlgo = ci.m_dispatcher1.findAlgorithm( m_convexBodyWrap, triObWrap, m_manifoldPtr );

				btCollisionObjectWrapper tmpWrap = null;

				if( m_resultOut.getBody0Internal() == m_triBodyWrap.m_collisionObject )
				{
					tmpWrap = m_resultOut.m_body0Wrap;
					m_resultOut.m_body0Wrap=( triObWrap );
					m_resultOut.setShapeIdentifiersA( partId, triangleIndex );
				}
				else
				{
					tmpWrap = m_resultOut.m_body1Wrap;
					m_resultOut.m_body1Wrap=( triObWrap );
					m_resultOut.setShapeIdentifiersB( partId, triangleIndex );
				}

				colAlgo.processCollision( m_convexBodyWrap, triObWrap, m_dispatchInfoPtr, m_resultOut );

				if( m_resultOut.getBody0Internal() == m_triBodyWrap.m_collisionObject )
				{
					m_resultOut.m_body0Wrap=( tmpWrap );
				}
				else
				{
					m_resultOut.m_body1Wrap=( tmpWrap );
				}



				//colAlgo.~btCollisionAlgorithm();
				ci.m_dispatcher1.freeCollisionAlgorithm( colAlgo );
				BulletGlobals.CollisionObjectWrapperPool.Free( triObWrap );
			}

		}



		public void setTimeStepAndCounters( double collisionMarginTriangle, btDispatcherInfo dispatchInfo, btCollisionObjectWrapper convexBodyWrap, btCollisionObjectWrapper triBodyWrap, btManifoldResult resultOut )
		{
			m_convexBodyWrap = convexBodyWrap;
			m_triBodyWrap = triBodyWrap;

			m_dispatchInfoPtr = dispatchInfo;
			m_collisionMarginTriangle = collisionMarginTriangle;
			m_resultOut = resultOut;

			//recalc aabbs
			btTransform convexInTriangleSpace;
			btTransform invTrans;
			m_triBodyWrap.m_worldTransform.inverse( out invTrans );
			invTrans.Apply( ref m_convexBodyWrap.m_worldTransform, out convexInTriangleSpace );
			btCollisionShape convexShape = m_convexBodyWrap.getCollisionShape();
			//CollisionShape* triangleShape = static_cast<btCollisionShape*>(triBody.m_collisionShape);
			convexShape.getAabb( ref convexInTriangleSpace, out m_aabbMin, out m_aabbMax );
			double extraMargin = collisionMarginTriangle;
			btVector3 extra = new btVector3( extraMargin );
			m_aabbMax += extra;
			m_aabbMin -= extra;
		}
	};




	/// btConvexConcaveCollisionAlgorithm  supports collision between convex shapes and (concave) trianges meshes.
	class btConvexConcaveCollisionAlgorithm : btActivatingCollisionAlgorithm
	{
		bool m_isSwapped;

		btConvexTriangleCallback m_btConvexTriangleCallback;

		internal class CreateFunc : btCollisionAlgorithmCreateFunc
		{
			internal override btCollisionAlgorithm CreateCollisionAlgorithm( btCollisionAlgorithmConstructionInfo ci, btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap )
			{
				btConvexConcaveCollisionAlgorithm ca = BulletGlobals.ConvexConcaveCollisionAlgorithmPool.Get();
				ca.Initialize( ci, body0Wrap, body1Wrap, false );
				return ca;
			}
		};

		internal class SwappedCreateFunc : btCollisionAlgorithmCreateFunc
		{
			internal override btCollisionAlgorithm CreateCollisionAlgorithm( btCollisionAlgorithmConstructionInfo ci, btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap )
			{
				btConvexConcaveCollisionAlgorithm ca = BulletGlobals.ConvexConcaveCollisionAlgorithmPool.Get();
				ca.Initialize( ci, body0Wrap, body1Wrap, true );
				return ca;
			}
		};

		public btConvexConcaveCollisionAlgorithm() { }

		internal void Initialize( btCollisionAlgorithmConstructionInfo ci, btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap, bool isSwapped )
		{
			base.Initialize( ci, body0Wrap, body1Wrap );
			m_isSwapped = ( isSwapped );
			m_btConvexTriangleCallback = BulletGlobals.ConvexTriangleCallbackPool.Get();
			m_btConvexTriangleCallback.Initialize( ci.m_dispatcher1, body0Wrap, body1Wrap, isSwapped );
		}

		internal override void Cleanup()
		{
			m_btConvexTriangleCallback.clearCache();
			BulletGlobals.ConvexTriangleCallbackPool.Free( m_btConvexTriangleCallback );
			m_btConvexTriangleCallback = null;
			BulletGlobals.ConvexConcaveCollisionAlgorithmPool.Free( this );
		}


		internal override void getAllContactManifolds( btManifoldArray manifoldArray )
		{
			if( m_btConvexTriangleCallback.m_manifoldPtr != null )
			{
				manifoldArray.Add( m_btConvexTriangleCallback.m_manifoldPtr );
			}
		}



		void clearCache()
		{
			m_btConvexTriangleCallback.clearCache();

		}

		internal override void processCollision( btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap, btDispatcherInfo dispatchInfo, btManifoldResult resultOut )
		{


			btCollisionObjectWrapper convexBodyWrap = m_isSwapped ? body1Wrap : body0Wrap;
			btCollisionObjectWrapper triBodyWrap = m_isSwapped ? body0Wrap : body1Wrap;

			if( triBodyWrap.getCollisionShape().isConcave() )
			{
				btConcaveShape concaveShape = (btConcaveShape)triBodyWrap.getCollisionShape();

				if( convexBodyWrap.getCollisionShape().isConvex() )
				{
					double collisionMarginTriangle = concaveShape.getMargin();

					resultOut.setPersistentManifold( m_btConvexTriangleCallback.m_manifoldPtr );
					m_btConvexTriangleCallback.setTimeStepAndCounters( collisionMarginTriangle, dispatchInfo, convexBodyWrap, triBodyWrap, resultOut );

					m_btConvexTriangleCallback.m_manifoldPtr.setBodies( convexBodyWrap.m_collisionObject, triBodyWrap.m_collisionObject );

					concaveShape.processAllTriangles( m_btConvexTriangleCallback, ref m_btConvexTriangleCallback.m_aabbMin, ref m_btConvexTriangleCallback.m_aabbMax );

					resultOut.refreshContactPoints();

					m_btConvexTriangleCallback.clearWrapperData();

				}

			}

		}

		internal struct LocalTriangleSphereCastCallback : btTriangleCallback
		{
			btTransform m_ccdSphereFromTrans;
			btTransform m_ccdSphereToTrans;
			//btTransform m_meshTransform;

			double m_ccdSphereRadius;
			internal double m_hitFraction;

			internal LocalTriangleSphereCastCallback( ref btTransform from, ref btTransform to, double ccdSphereRadius, double hitFraction )
			{
				m_ccdSphereFromTrans = ( from );
				m_ccdSphereToTrans = ( to );
				m_ccdSphereRadius = ( ccdSphereRadius );
				m_hitFraction = ( hitFraction );
			}


			public void processTriangle( btVector3[] triangle, int partId, int triangleIndex )
			{
				//(void)partId;
				//(void)triangleIndex;
				//do a swept sphere for now
				btTransform ident = btTransform.Identity;
				using( btConvexCast.CastResult castResult = BulletGlobals.CastResultPool.Get() )
				{
					castResult.m_fraction = m_hitFraction;
					using( btSphereShape pointShape = BulletGlobals.SphereShapePool.Get() )
					{
						pointShape.Initialize( m_ccdSphereRadius );
						using( btTriangleShape triShape = BulletGlobals.TriangleShapePool.Get() )
						{
							triShape.Initialize( triangle );
							using( btVoronoiSimplexSolver simplexSolver = BulletGlobals.VoronoiSimplexSolverPool.Get() )
							{
								using( btSubsimplexConvexCast convexCaster = BulletGlobals.SubSimplexConvexCastPool.Get() )
								{
									convexCaster.Initialize( pointShape, triShape, simplexSolver );
									//GjkConvexCast	convexCaster(&pointShape,convexShape,&simplexSolver);
									//ContinuousConvexCollision convexCaster(&pointShape,convexShape,&simplexSolver,0);
									//local space?

									if( convexCaster.calcTimeOfImpact( ref m_ccdSphereFromTrans, ref m_ccdSphereToTrans,
										ref ident,ref ident, castResult ) )
									{
										if( m_hitFraction > castResult.m_fraction )
											m_hitFraction = castResult.m_fraction;
									}
								}
							}
						}
					}
				}
			}

		};



		internal override double calculateTimeOfImpact( btCollisionObject body0, btCollisionObject body1, btDispatcherInfo dispatchInfo, btManifoldResult resultOut )
		{
			//(void)resultOut;
			//(void)dispatchInfo;
			btCollisionObject convexbody = m_isSwapped ? body1 : body0;
			btCollisionObject triBody = m_isSwapped ? body0 : body1;

			//quick approximation using raycast, todo: hook up to the continuous collision detection (one of the btConvexCast)

			//only perform CCD above a certain threshold, this prevents blocking on the long run
			//because object in a blocked ccd state (hitfraction<1) get their linear velocity halved each frame...
			double squareMot0 = btVector3.btDelLength2( ref convexbody.m_interpolationWorldTransform.m_origin, ref convexbody.m_worldTransform.m_origin );
			if( squareMot0 < convexbody.getCcdSquareMotionThreshold() )
			{
				return btScalar.BT_ONE;
			}

			//ref btVector3 from = convexbody.m_worldTransform.getOrigin();
			//btVector3 to = convexbody.m_interpolationWorldTransform.getOrigin();
			//todo: only do if the motion exceeds the 'radius'

			btTransform triInv; triBody.m_worldTransform.inverse( out triInv );
			btTransform convexFromLocal; triInv.Apply( ref convexbody.m_worldTransform, out convexFromLocal );
			btTransform convexToLocal; triInv.Apply( ref convexbody.m_interpolationWorldTransform, out convexToLocal );

			if( triBody.getCollisionShape().isConcave() )
			{
				btVector3 rayAabbMin = convexFromLocal.m_origin;
				rayAabbMin.setMin( ref convexToLocal.m_origin );
				btVector3 rayAabbMax = convexFromLocal.m_origin;
				rayAabbMax.setMax( ref convexToLocal.m_origin );
				double ccdRadius0 = convexbody.getCcdSweptSphereRadius();
				rayAabbMin.AddScale( ref btVector3.One, -ccdRadius0, out rayAabbMin );
				rayAabbMax.AddScale( ref btVector3.One, ccdRadius0, out rayAabbMax );

				double curHitFraction = btScalar.BT_ONE; //is this available?
				LocalTriangleSphereCastCallback raycastCallback = new LocalTriangleSphereCastCallback ( ref convexFromLocal, ref convexToLocal,
					convexbody.getCcdSweptSphereRadius(), curHitFraction);

				raycastCallback.m_hitFraction = convexbody.getHitFraction();

				btCollisionObject concavebody = triBody;

				btConcaveShape triangleMesh = (btConcaveShape)concavebody.getCollisionShape();

				if( triangleMesh != null )
				{
					triangleMesh.processAllTriangles( raycastCallback, ref rayAabbMin, ref rayAabbMax );
				}



				if( raycastCallback.m_hitFraction < convexbody.getHitFraction() )
				{
					convexbody.setHitFraction( raycastCallback.m_hitFraction );
					return raycastCallback.m_hitFraction;
				}
			}

			return btScalar.BT_ONE;

		}

	};

}
