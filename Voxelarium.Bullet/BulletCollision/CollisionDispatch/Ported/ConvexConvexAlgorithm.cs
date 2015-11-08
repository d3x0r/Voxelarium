//#define BT_DISABLE_CAPSULE_CAPSULE_COLLIDER
//#define USE_SEPDISTANCE_UTIL2
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
using System.Diagnostics;
using Bullet.Collision.BroadPhase;
using Bullet.Collision.NarrowPhase;
using Bullet.Collision.Shapes;
using Bullet.LinearMath;

namespace Bullet.Collision.Dispatch
{


	///Enabling USE_SEPDISTANCE_UTIL2 requires 100% reliable distance computation. However, when using large size ratios GJK can be imprecise
	///so the distance is not conservative. In that case, enabling this USE_SEPDISTANCE_UTIL2 would result in failing/missing collisions.
	///Either improve GJK for large size ratios (testing a 100 units versus a 0.1 unit object) or only enable the util
	///for certain pairs that have a small size ratio


	///The convexConvexAlgorithm collision algorithm implements time of impact, convex closest points and penetration depth calculations between two convex objects.
	///Multiple contact points are calculated by perturbing the orientation of the smallest object orthogonal to the separating normal.
	///This idea was described by Gino van den Bergen in this forum topic http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=4&t=288&p=888#p888
	internal class btConvexConvexAlgorithm : btActivatingCollisionAlgorithm
	{

		internal class btDummyResult : btDiscreteCollisionDetectorInterface.Result
		{
			public void setShapeIdentifiersA( int partId0, int index0 ) { }
			public void setShapeIdentifiersB( int partId1, int index1 ) { }
			public void addContactPoint( ref btVector3 normalOnBInWorld, ref btVector3 pointInWorld, double depth )
			{
			}
		};


		internal class btWithoutMarginResult : btDiscreteCollisionDetectorInterface.Result
		{
			btDiscreteCollisionDetectorInterface.Result m_originalResult;
			internal btVector3 m_reportedNormalOnWorld;
			internal double m_marginOnA;
			internal double m_marginOnB;
			internal double m_reportedDistance;

			internal bool m_foundResult;
			internal btWithoutMarginResult( btDiscreteCollisionDetectorInterface.Result result
				, double marginOnA, double marginOnB )
			{
				m_originalResult = ( result );
				m_marginOnA = ( marginOnA );
				m_marginOnB = ( marginOnB );
				m_foundResult = ( false );
			}

			public void setShapeIdentifiersA( int partId0, int index0 ) { }
			public void setShapeIdentifiersB( int partId1, int index1 ) { }
			public void addContactPoint( ref btVector3 normalOnBInWorld, ref btVector3 pointInWorldOrg, double depthOrg )
			{
				m_reportedDistance = depthOrg;
				m_reportedNormalOnWorld = normalOnBInWorld;

				btVector3 adjustedPointB; pointInWorldOrg.SubScale( ref normalOnBInWorld, m_marginOnB, out adjustedPointB );
				m_reportedDistance = depthOrg + ( m_marginOnA + m_marginOnB );
				if( m_reportedDistance < 0 )
				{
					m_foundResult = true;
				}
				m_originalResult.addContactPoint( ref normalOnBInWorld, ref adjustedPointB, m_reportedDistance );
			}
		};


#if USE_SEPDISTANCE_UTIL2
	btConvexSeparatingDistanceUtil	m_sepDistance;
#endif
		btSimplexSolverInterface m_simplexSolver;
		btConvexPenetrationDepthSolver m_pdSolver;


		bool m_ownManifold;
		btPersistentManifold m_manifoldPtr;
		bool m_lowLevelOfDetail;

		int m_numPerturbationIterations;
		int m_minimumPointsPerturbationThreshold;


		///cache separating vector to speedup collision detection
		internal override void Cleanup()
		{
			if( m_ownManifold )
				m_dispatcher.releaseManifold( m_manifoldPtr );

			m_simplexSolver = null;
			m_pdSolver = null;
			m_manifoldPtr = null;
		}


		internal override void getAllContactManifolds( btManifoldArray manifoldArray )
		{
			///should we use m_ownManifold to avoid adding duplicates?
			if( m_manifoldPtr != null && m_ownManifold )
				manifoldArray.Add( m_manifoldPtr );
		}


		public btPersistentManifold getManifold()
		{
			return m_manifoldPtr;
		}

		internal class CreateFunc : btCollisionAlgorithmCreateFunc
		{

			btConvexPenetrationDepthSolver m_pdSolver;
			btSimplexSolverInterface m_simplexSolver;
			internal int m_numPerturbationIterations;
			internal int m_minimumPointsPerturbationThreshold;

			internal CreateFunc( btSimplexSolverInterface simplexSolver, btConvexPenetrationDepthSolver pdSolver )
			{
				m_numPerturbationIterations = 0;
				m_minimumPointsPerturbationThreshold = 3;
				m_simplexSolver = simplexSolver;
				m_pdSolver = pdSolver;
			}

			internal override btCollisionAlgorithm CreateCollisionAlgorithm( btCollisionAlgorithmConstructionInfo ci, btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap )
			{
				btConvexConvexAlgorithm ca = BulletGlobals.ConvexConvexAlgorithmPool.Get();
				ca.Initialize( ci.m_manifold, ci, body0Wrap, body1Wrap, m_simplexSolver, m_pdSolver, m_numPerturbationIterations, m_minimumPointsPerturbationThreshold );
				return ca;
			}
		};


		static public void segmentsClosestPoints(
			out btVector3 ptsVector,
			out btVector3 offsetA,
			out btVector3 offsetB,
			out double tA, out double tB,
			ref btVector3 translation,
			ref btVector3 dirA, double hlenA,
			ref btVector3 dirB, double hlenB )
		{
			// compute the parameters of the closest points on each line segment

			double dirA_dot_dirB = btVector3.btDot( ref dirA, ref dirB );
			double dirA_dot_trans = btVector3.btDot( ref dirA, ref translation );
			double dirB_dot_trans = btVector3.btDot( ref dirB, ref translation );

			double denom = 1.0f - dirA_dot_dirB * dirA_dot_dirB;

			if( denom == 0.0f )
			{
				tA = 0.0f;
			}
			else
			{
				tA = ( dirA_dot_trans - dirB_dot_trans * dirA_dot_dirB ) / denom;
				if( tA < -hlenA )
					tA = -hlenA;
				else if( tA > hlenA )
					tA = hlenA;
			}

			tB = tA * dirA_dot_dirB - dirB_dot_trans;

			if( tB < -hlenB )
			{
				tB = -hlenB;
				tA = tB * dirA_dot_dirB + dirA_dot_trans;

				if( tA < -hlenA )
					tA = -hlenA;
				else if( tA > hlenA )
					tA = hlenA;
			}
			else if( tB > hlenB )
			{
				tB = hlenB;
				tA = tB * dirA_dot_dirB + dirA_dot_trans;

				if( tA < -hlenA )
					tA = -hlenA;
				else if( tA > hlenA )
					tA = hlenA;
			}

			// compute the closest points relative to segment centers.

			dirA.Mult( tA, out offsetA );
			dirB.Mult( tB, out offsetB );

			btVector3 tmp;
			translation.Sub( ref offsetA, out tmp );
			tmp.Add( ref offsetB, out ptsVector );
			//ptsVector = translation - offsetA + offsetB;
		}


		static public double capsuleCapsuleDistance(
			out btVector3 normalOnB,
			out btVector3 pointOnB,
			double capsuleLengthA,
			double capsuleRadiusA,
			double capsuleLengthB,
			double capsuleRadiusB,
			int capsuleAxisA,
			int capsuleAxisB,
			btITransform transformA,
			btITransform transformB,
			double distanceThreshold )
		{
			btVector3 directionA; transformA.Basis.getColumn( capsuleAxisA, out directionA );
			btVector3 translationA; transformA.getOrigin( out translationA );
			btVector3 directionB; transformB.getBasis().getColumn( capsuleAxisB, out directionB );
			btVector3 translationB; transformB.getOrigin( out translationB );

			// translation between centers

			btVector3 translation; translationB.Sub( ref translationA, out translation );

			// compute the closest points of the capsule line segments

			btVector3 ptsVector;           // the vector between the closest points

			btVector3 offsetA, offsetB;    // offsets from segment centers to their closest points
			double tA, tB;              // parameters on line segment

			segmentsClosestPoints( out ptsVector, out offsetA, out offsetB, out tA, out tB, ref translation,
								   ref directionA, capsuleLengthA, ref directionB, capsuleLengthB );

			double distance = ptsVector.length() - capsuleRadiusA - capsuleRadiusB;

			if( distance > distanceThreshold )
			{
				pointOnB = btVector3.Zero;
				normalOnB = btVector3.xAxis;
				return distance;
			}

			double lenSqr = ptsVector.length2();
			if( lenSqr <= ( btScalar.SIMD_EPSILON * btScalar.SIMD_EPSILON ) )
			{
				//degenerate case where 2 capsules are likely at the same location: take a vector tangential to 'directionA'
				btVector3 q;
				btVector3.btPlaneSpace1( ref directionA, out normalOnB, out q );
			}
			else
			{
				// compute the contact normal
				ptsVector.Mult( -btScalar.btRecipSqrt( lenSqr ), out normalOnB );
			}
			btVector3 tmp;
			btVector3 tmp2;
			translationB.Add( ref offsetB, out tmp );
			normalOnB.Mult( capsuleRadiusB, out tmp2 );
			tmp.Add( ref tmp2, out pointOnB );
			//pointOnB = transformB.getOrigin() + offsetB + normalOnB * capsuleRadiusB;

			return distance;
		}


		//////////
		public btConvexConvexAlgorithm() { }

		public void Initialize( btPersistentManifold mf, btCollisionAlgorithmConstructionInfo ci, btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap, btSimplexSolverInterface simplexSolver, btConvexPenetrationDepthSolver pdSolver, int numPerturbationIterations, int minimumPointsPerturbationThreshold )
		{
			base.Initialize( ci, body0Wrap, body1Wrap );
			m_simplexSolver = ( simplexSolver );
			m_pdSolver = ( pdSolver );
			m_ownManifold = ( false );
			m_manifoldPtr = ( mf );
			m_lowLevelOfDetail = ( false );
#if USE_SEPDISTANCE_UTIL2
m_sepDistance((static_cast<btConvexShape*>(body0.getCollisionShape())).getAngularMotionDisc(),
			  (static_cast<btConvexShape*>(body1.getCollisionShape())).getAngularMotionDisc()),
#endif
			m_numPerturbationIterations = ( numPerturbationIterations );
			m_minimumPointsPerturbationThreshold = ( minimumPointsPerturbationThreshold );
		}




		~btConvexConvexAlgorithm()
		{
			if( m_ownManifold )
			{
				if( m_manifoldPtr != null )
					m_dispatcher.releaseManifold( m_manifoldPtr );
			}
		}

		public void setLowLevelOfDetail( bool useLowLevel )
		{
			m_lowLevelOfDetail = useLowLevel;
		}


		internal class btPerturbedContactResult : btManifoldResult
		{
			btManifoldResult m_originalManifoldResult;
			btITransform m_transformA;
			btITransform m_transformB;
			btITransform m_unPerturbedTransform;
			bool m_perturbA;
			btIDebugDraw m_debugDrawer;


			internal btPerturbedContactResult( btManifoldResult originalResult
				, btITransform transformA, btITransform transformB
				, ref btTransform unPerturbedTransform, bool perturbA, btIDebugDraw debugDrawer )
			{
				m_originalManifoldResult = ( originalResult );
				m_transformA = ( transformA );
				m_transformB = ( transformB );
				m_unPerturbedTransform = ( unPerturbedTransform );
				m_perturbA = ( perturbA );
				m_debugDrawer = ( debugDrawer );
			}

			public override void addContactPoint( ref btVector3 normalOnBInWorld, ref btVector3 pointInWorld, double orgDepth )
			{
				btVector3 endPt, startPt;
				double newDepth;
				//btVector3 newNormal;

				if( m_perturbA )
				{
					btVector3 endPtOrg; pointInWorld.AddScale( ref normalOnBInWorld, orgDepth, out endPtOrg );
					btTransform inv; m_transformA.inverse( out inv );
					btTransform tmp;
					m_unPerturbedTransform.Apply( ref inv, out tmp );
					/*endPt =*/
					tmp.Apply( ref endPtOrg, out endPt );
					btVector3 tmp2;
					endPt.Sub( ref pointInWorld, out tmp2 );
					newDepth = tmp2.dot( ref normalOnBInWorld );
					endPt.AddScale( ref normalOnBInWorld, newDepth, out startPt );
				}
				else
				{
					pointInWorld.AddScale( ref normalOnBInWorld, orgDepth, out endPt );
					btTransform inv; m_transformB.inverse( out inv );
					btTransform tmp;
					m_unPerturbedTransform.Apply( ref inv, out tmp );
					tmp.Apply( ref pointInWorld, out startPt );
					//startPt = ( m_unPerturbedTransform * m_transformB.inverse() )( pointInWorld );
					btVector3 tmp2;
					endPt.Sub( ref startPt, out tmp2 );
					newDepth = tmp2.dot( ref normalOnBInWorld );

				}

				//#define DEBUG_CONTACTS 1
#if DEBUG_CONTACTS
		m_debugDrawer.drawLine(ref startPt,ref endPt,btVector3(1,0,0));
		m_debugDrawer.drawSphere(ref startPt,0.05,btVector3(0,1,0));
		m_debugDrawer.drawSphere(ref endPt,0.05,btVector3(0,0,1));
#endif //DEBUG_CONTACTS


				m_originalManifoldResult.addContactPoint( ref normalOnBInWorld, ref startPt, newDepth );
			}

		};


		//
		// Convex-Convex collision algorithm
		//
		internal override void processCollision( btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap
							, btDispatcherInfo dispatchInfo, btManifoldResult resultOut )
		{

			if( m_manifoldPtr == null )
			{
				//swapped?
				m_manifoldPtr = m_dispatcher.getNewManifold( body0Wrap.m_collisionObject, body1Wrap.m_collisionObject );
				m_ownManifold = true;
			}
			resultOut.setPersistentManifold( m_manifoldPtr );

			//comment-out next line to test multi-contact generation
			//resultOut.getPersistentManifold().clearManifold();


			btConvexShape min0 = (btConvexShape)body0Wrap.getCollisionShape();
			btConvexShape min1 = (btConvexShape)body1Wrap.getCollisionShape();

			btVector3 normalOnB;
			btVector3 pointOnBWorld;
#if !BT_DISABLE_CAPSULE_CAPSULE_COLLIDER
			if( ( min0.getShapeType() == BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE )
				&& ( min1.getShapeType() == BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE ) )
			{
				btCapsuleShape capsuleA = (btCapsuleShape)min0;
				btCapsuleShape capsuleB = (btCapsuleShape)min1;
				//	btVector3 localScalingA = capsuleA.getLocalScaling();
				//	btVector3 localScalingB = capsuleB.getLocalScaling();

				double threshold = m_manifoldPtr.getContactBreakingThreshold();

				double dist = capsuleCapsuleDistance( out normalOnB, out pointOnBWorld
						, capsuleA.getHalfHeight(), capsuleA.getRadius()
						, capsuleB.getHalfHeight(), capsuleB.getRadius()
						, capsuleA.getUpAxis(), capsuleB.getUpAxis()
						, body0Wrap.getWorldTransform(), body1Wrap.getWorldTransform(), threshold );

				if( dist < threshold )
				{
					Debug.Assert( normalOnB.length2() >= ( btScalar.SIMD_EPSILON * btScalar.SIMD_EPSILON ) );
					resultOut.addContactPoint( ref normalOnB, ref pointOnBWorld, dist );
				}
				resultOut.refreshContactPoints();
				return;
			}
#endif //BT_DISABLE_CAPSULE_CAPSULE_COLLIDER




#if USE_SEPDISTANCE_UTIL2
	if (dispatchInfo.m_useConvexConservativeDistanceUtil)
	{
		m_sepDistance.updateSeparatingDistance(body0.getWorldTransform(),body1.getWorldTransform());
	}

	if (!dispatchInfo.m_useConvexConservativeDistanceUtil || m_sepDistance.getConservativeSeparatingDistance()<=0)
#endif //USE_SEPDISTANCE_UTIL2

			{
				btGjkPairDetector.ClosestPointInput input = BulletGlobals.ClosestPointInputPool.Get();
				input.Initialize();

				btGjkPairDetector gjkPairDetector = BulletGlobals.GjkPairDetectorPool.Get();
				gjkPairDetector.Initialize( min0, min1, m_simplexSolver, m_pdSolver );
				//TODO: if (dispatchInfo.m_useContinuous)
				gjkPairDetector.setMinkowskiA( min0 );
				gjkPairDetector.setMinkowskiB( min1 );

#if USE_SEPDISTANCE_UTIL2
	if (dispatchInfo.m_useConvexConservativeDistanceUtil)
	{
		input.m_maximumDistanceSquared = BT_LARGE_FLOAT;
	} else
#endif //USE_SEPDISTANCE_UTIL2
				{
					//if (dispatchInfo.m_convexMaxDistanceUseCPT)
					//{
					//	input.m_maximumDistanceSquared = min0.getMargin() + min1.getMargin() + m_manifoldPtr.getContactProcessingThreshold();
					//} else
					//{
					input.m_maximumDistanceSquared = min0.getMargin() + min1.getMargin() + m_manifoldPtr.getContactBreakingThreshold();
					//		}

					input.m_maximumDistanceSquared *= input.m_maximumDistanceSquared;
				}

				input.m_transformA = body0Wrap.m_worldTransform.T;
				input.m_transformB = body1Wrap.m_worldTransform.T;





#if USE_SEPDISTANCE_UTIL2
	double sepDist = 0;
	if (dispatchInfo.m_useConvexConservativeDistanceUtil)
	{
		sepDist = gjkPairDetector.getCachedSeparatingDistance();
		if (sepDist>SIMD_EPSILON)
		{
			sepDist += dispatchInfo.m_convexConservativeDistanceThreshold;
			//now perturbe directions to get multiple contact points
			
		}
	}
#endif //USE_SEPDISTANCE_UTIL2

				if( min0.isPolyhedral() && min1.isPolyhedral() )
				{



					btDummyResult dummy = new btDummyResult();

					///btBoxShape is an exception: its vertices are created WITH margin so don't subtract it

					double min0Margin = min0.getShapeType() == BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE ? 0 : min0.getMargin();
					double min1Margin = min1.getShapeType() == BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE ? 0 : min1.getMargin();

					btWithoutMarginResult withoutMargin = new btWithoutMarginResult( resultOut, min0Margin, min1Margin );

					btPolyhedralConvexShape polyhedronA = (btPolyhedralConvexShape)min0;
					btPolyhedralConvexShape polyhedronB = (btPolyhedralConvexShape)min1;
					if( polyhedronA.getConvexPolyhedron() != null && polyhedronB.getConvexPolyhedron() != null )
					{
						double threshold = m_manifoldPtr.getContactBreakingThreshold();

						double minDist = -1e30f;
						btVector3 sepNormalWorldSpace;
						bool foundSepAxis = true;

						if( dispatchInfo.m_enableSatConvex )
						{
							foundSepAxis = btPolyhedralContactClipping.findSeparatingAxis(
								polyhedronA.getConvexPolyhedron(), polyhedronB.getConvexPolyhedron(),
								body0Wrap.m_worldTransform,
								body1Wrap.getWorldTransform(),
								out sepNormalWorldSpace, resultOut );
						}
						else
						{
#if ZERO_MARGIN
				gjkPairDetector.setIgnoreMargin(true);
				gjkPairDetector.getClosestPoints(input,*resultOut,dispatchInfo.m_debugDraw);
#else

							gjkPairDetector.getClosestPoints( input, withoutMargin, dispatchInfo.m_debugDraw );
							//gjkPairDetector.getClosestPoints(input,dummy,dispatchInfo.m_debugDraw);
#endif //ZERO_MARGIN
							//double l2 = gjkPairDetector.getCachedSeparatingAxis().length2();
							//if (l2>SIMD_EPSILON)
							{
								sepNormalWorldSpace = withoutMargin.m_reportedNormalOnWorld;//gjkPairDetector.getCachedSeparatingAxis()*(1/l2);
																							//minDist = -1e30f;//gjkPairDetector.getCachedSeparatingDistance();
								minDist = withoutMargin.m_reportedDistance;//gjkPairDetector.getCachedSeparatingDistance()+min0.getMargin()+min1.getMargin();

#if ZERO_MARGIN
					foundSepAxis = true;//gjkPairDetector.getCachedSeparatingDistance()<0;
#else
								foundSepAxis = withoutMargin.m_foundResult && minDist < 0;//-(min0.getMargin()+min1.getMargin());
#endif
							}
						}
						if( foundSepAxis )
						{

							//				Console.WriteLine("sepNormalWorldSpace=%f,%f,%f\n",sepNormalWorldSpace.x,sepNormalWorldSpace.y,sepNormalWorldSpace.z);

							btPolyhedralContactClipping.clipHullAgainstHull( ref sepNormalWorldSpace, polyhedronA.getConvexPolyhedron(), polyhedronB.getConvexPolyhedron(),
								body0Wrap.getWorldTransform(),
								body1Wrap.getWorldTransform()
								, minDist - threshold, threshold, resultOut );

						}
						if( m_ownManifold )
						{
							resultOut.refreshContactPoints();
						}
						BulletGlobals.ClosestPointInputPool.Free( input );
						BulletGlobals.GjkPairDetectorPool.Free( gjkPairDetector );
						return;

					}
					else
					{
						//we can also deal with convex versus triangle (without connectivity data)
						if( polyhedronA.getConvexPolyhedron() != null && polyhedronB.getShapeType() == BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE )
						{

							btVertexArray vertices = new btVertexArray();
							btTriangleShape tri = (btTriangleShape)polyhedronB;
							btVector3 tmp;
							body1Wrap.getWorldTransform().Apply( ref tri.m_vertices1, out tmp );
							vertices.Add( ref tmp );
							body1Wrap.getWorldTransform().Apply( ref tri.m_vertices2, out tmp );
							vertices.Add( ref tmp );
							body1Wrap.getWorldTransform().Apply( ref tri.m_vertices3, out tmp );
							vertices.Add( ref tmp );

							//tri.initializePolyhedralFeatures();

							double threshold = m_manifoldPtr.getContactBreakingThreshold();

							btVector3 sepNormalWorldSpace;
							double minDist = -btScalar.BT_LARGE_FLOAT;
							double maxDist = threshold;

							bool foundSepAxis = false;
							if( false )
							{
								polyhedronB.initializePolyhedralFeatures();
								foundSepAxis = btPolyhedralContactClipping.findSeparatingAxis(
										polyhedronA.getConvexPolyhedron(), polyhedronB.getConvexPolyhedron(),
										body0Wrap.getWorldTransform(),
										body1Wrap.getWorldTransform(),
										out sepNormalWorldSpace, resultOut );
								//	 Console.WriteLine("sepNormalWorldSpace=%f,%f,%f\n",sepNormalWorldSpace.x,sepNormalWorldSpace.y,sepNormalWorldSpace.z);
								btPolyhedralContactClipping.clipFaceAgainstHull( ref sepNormalWorldSpace
									, polyhedronA.getConvexPolyhedron(),
									body0Wrap.getWorldTransform(), vertices, minDist - threshold, maxDist, resultOut );

							}
							else
							{
#if ZERO_MARGIN
					gjkPairDetector.setIgnoreMargin(true);
					gjkPairDetector.getClosestPoints(input,*resultOut,dispatchInfo.m_debugDraw);
#else
								gjkPairDetector.getClosestPoints( input, dummy, dispatchInfo.m_debugDraw );
#endif//ZERO_MARGIN

								double l2 = gjkPairDetector.getCachedSeparatingAxis().length2();
								if( l2 > btScalar.SIMD_EPSILON )
								{
									gjkPairDetector.getCachedSeparatingAxis().Mult( ( 1 / l2 ), out sepNormalWorldSpace );
									//minDist = gjkPairDetector.getCachedSeparatingDistance();
									//maxDist = threshold;
									minDist = gjkPairDetector.getCachedSeparatingDistance() - min0.getMargin() - min1.getMargin();
									//foundSepAxis = true;
									btPolyhedralContactClipping.clipFaceAgainstHull( ref sepNormalWorldSpace
										, polyhedronA.getConvexPolyhedron(),
										body0Wrap.getWorldTransform(), vertices, minDist - threshold, maxDist, resultOut );
								}
								else
								{
									//sepNormalWorldSpace = btVector3.Zero;
								}
							}


							if( m_ownManifold )
							{
								resultOut.refreshContactPoints();
							}
							BulletGlobals.ClosestPointInputPool.Free( input );
							BulletGlobals.GjkPairDetectorPool.Free( gjkPairDetector );

							return;
						}

					}


				}

				gjkPairDetector.getClosestPoints( input, resultOut, dispatchInfo.m_debugDraw );

				//now perform 'm_numPerturbationIterations' collision queries with the perturbated collision objects

				//perform perturbation when more then 'm_minimumPointsPerturbationThreshold' points
				if( m_numPerturbationIterations != 0
					&& resultOut.m_manifoldPtr.m_cachedPoints < m_minimumPointsPerturbationThreshold )
				{

					int i;
					btVector3 v0, v1;
					btVector3 sepNormalWorldSpace;
					double l2 = gjkPairDetector.getCachedSeparatingAxis().length2();

					if( l2 > btScalar.SIMD_EPSILON )
					{
						gjkPairDetector.getCachedSeparatingAxis().Mult( ( 1 / l2 ), out sepNormalWorldSpace );

						btVector3.btPlaneSpace1( ref sepNormalWorldSpace, out v0, out v1 );


						bool perturbeA = true;
						double angleLimit = 0.125f * btScalar.SIMD_PI;
						double perturbeAngle;
						double radiusA = min0.getAngularMotionDisc();
						double radiusB = min1.getAngularMotionDisc();
						if( radiusA < radiusB )
						{
							perturbeAngle = btPersistentManifold.gContactBreakingThreshold / radiusA;
							perturbeA = true;
						}
						else
						{
							perturbeAngle = btPersistentManifold.gContactBreakingThreshold / radiusB;
							perturbeA = false;
						}
						if( perturbeAngle > angleLimit )
							perturbeAngle = angleLimit;

						btTransform unPerturbedTransform;
						if( perturbeA )
						{
							unPerturbedTransform = input.m_transformA.T;
						}
						else
						{
							unPerturbedTransform = input.m_transformB.T;
						}

						for( i = 0; i < m_numPerturbationIterations; i++ )
						{
							if( v0.length2() > btScalar.SIMD_EPSILON )
							{
								btQuaternion perturbeRot = new btQuaternion( ref v0, perturbeAngle );
								double iterationAngle = i * ( btScalar.SIMD_2_PI / (double)( m_numPerturbationIterations ) );
								btQuaternion rotq = new btQuaternion( ref sepNormalWorldSpace, iterationAngle );


								if( perturbeA )
								{
									btQuaternion tmpq;
									btQuaternion tmpq2;
									rotq.inverse( out tmpq );
									btQuaternion.Mult( ref tmpq, ref perturbeRot, out tmpq2 );
									btQuaternion.Mult( ref tmpq2, ref rotq, out tmpq );
									btMatrix3x3 m = new btMatrix3x3( ref tmpq );
									btMatrix3x3 m2; body0Wrap.m_worldTransform.getBasis( out m2 );
									btMatrix3x3 m3;
									btMatrix3x3.Mult( ref m, ref m2, out m3 );
									input.m_transformA.setBasis( ref m3 );
									input.m_transformB = body1Wrap.getWorldTransform().T;
#if DEBUG_CONTACTS
					dispatchInfo.m_debugDraw.drawTransform(input.m_transformA,10.0);
#endif //DEBUG_CONTACTS
								}
								else
								{
									btQuaternion tmpq;
									btQuaternion tmpq2;
									rotq.inverse( out tmpq );
									btQuaternion.Mult( ref tmpq, ref perturbeRot, out tmpq2 );
									btQuaternion.Mult( ref tmpq2, ref rotq, out tmpq );
									btMatrix3x3 m = new btMatrix3x3( ref tmpq );
									btMatrix3x3 m2; body1Wrap.m_worldTransform.getBasis( out m2 );
									btMatrix3x3 m3;
									btMatrix3x3.Mult( ref m, ref m2, out m3 );

									input.m_transformA = body0Wrap.getWorldTransform().T;
									input.m_transformB.setBasis( ref m3 );
#if DEBUG_CONTACTS
					dispatchInfo.m_debugDraw.drawTransform(input.m_transformB,10.0);
#endif
								}

								btPerturbedContactResult perturbedResultOut = new btPerturbedContactResult( resultOut
										, input.m_transformA, input.m_transformB
										, ref unPerturbedTransform, perturbeA
										, dispatchInfo.m_debugDraw );
								gjkPairDetector.getClosestPoints( input, perturbedResultOut, dispatchInfo.m_debugDraw );
							}
						}
					}
				}



#if USE_SEPDISTANCE_UTIL2
	if (dispatchInfo.m_useConvexConservativeDistanceUtil && (sepDist>SIMD_EPSILON))
	{
		m_sepDistance.initSeparatingDistance(gjkPairDetector.getCachedSeparatingAxis(),sepDist,body0.getWorldTransform(),body1.getWorldTransform());
	}
#endif //USE_SEPDISTANCE_UTIL2

				BulletGlobals.ClosestPointInputPool.Free( input );
				BulletGlobals.GjkPairDetectorPool.Free( gjkPairDetector );

			}

			if( m_ownManifold )
			{
				resultOut.refreshContactPoints();
			}

		}



		bool disableCcd = false;
		internal override double calculateTimeOfImpact( btCollisionObject col0, btCollisionObject col1, btDispatcherInfo dispatchInfo, btManifoldResult resultOut )
		{
			//(void)resultOut;
			//(void)dispatchInfo;
			///Rather then checking ALL pairs, only calculate TOI when motion exceeds threshold

			///Linear motion for one of objects needs to exceed m_ccdSquareMotionThreshold
			///col0.m_worldTransform,
			double resultFraction = btScalar.BT_ONE;

			btVector3 tmp;
			col0.m_interpolationWorldTransform.m_origin.Sub( ref col0.m_worldTransform.m_origin, out tmp );
			double squareMot0 = ( tmp ).length2();
			col1.m_interpolationWorldTransform.m_origin.Sub( ref col1.m_worldTransform.m_origin, out tmp );
			double squareMot1 = ( tmp ).length2();

			if( squareMot0 < col0.getCcdSquareMotionThreshold() &&
				squareMot1 < col1.getCcdSquareMotionThreshold() )
				return resultFraction;

			if( disableCcd )
				return btScalar.BT_ONE;


			//An adhoc way of testing the Continuous Collision Detection algorithms
			//One object is approximated as a sphere, to simplify things
			//Starting in penetration should report no time of impact
			//For proper CCD, better accuracy and handling of 'allowed' penetration should be added
			//also the mainloop of the physics should have a kind of toi queue (something like Brian Mirtich's application of Timewarp for Rigidbodies)


			/// Convex0 against sphere for Convex1
			{
				btConvexShape convex0 = (btConvexShape)col0.getCollisionShape();

				using( btSphereShape sphere1 = BulletGlobals.SphereShapePool.Get() )
				{
					sphere1.Initialize( col1.getCcdSweptSphereRadius() ); //todo: allow non-zero sphere sizes, for better approximation
					btConvexCast.CastResult result = BulletGlobals.CastResultPool.Get();
					btVoronoiSimplexSolver voronoiSimplex = BulletGlobals.VoronoiSimplexSolverPool.Get();
					//SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
					///Simplification, one object is simplified as a sphere
					btGjkConvexCast ccd1 = BulletGlobals.GjkConvexCastPool.Get();
					ccd1.Initialize( convex0, sphere1, voronoiSimplex );
					//ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
					if( ccd1.calcTimeOfImpact( col0.m_worldTransform, col0.m_interpolationWorldTransform,
							col1.m_worldTransform, col1.m_interpolationWorldTransform, result ) )
					{

						//store result.m_fraction in both bodies

						if( col0.getHitFraction() > result.m_fraction )
							col0.setHitFraction( result.m_fraction );

						if( col1.getHitFraction() > result.m_fraction )
							col1.setHitFraction( result.m_fraction );

						if( resultFraction > result.m_fraction )
							resultFraction = result.m_fraction;

					}
					BulletGlobals.GjkConvexCastPool.Free( ccd1 );
				}
			}

			/// Sphere (for convex0) against Convex1
			{
				btConvexShape convex1 = (btConvexShape)( col1.getCollisionShape() );

				using( btSphereShape sphere0 = BulletGlobals.SphereShapePool.Get() )
				{
					sphere0.Initialize( col0.getCcdSweptSphereRadius() ); //todo: allow non-zero sphere sizes, for better approximation
					btConvexCast.CastResult result = BulletGlobals.CastResultPool.Get();
					btVoronoiSimplexSolver voronoiSimplex = BulletGlobals.VoronoiSimplexSolverPool.Get();
					//SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
					///Simplification, one object is simplified as a sphere
					btGjkConvexCast ccd1 = BulletGlobals.GjkConvexCastPool.Get();
					ccd1.Initialize( sphere0, convex1, voronoiSimplex );
					//ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
					if( ccd1.calcTimeOfImpact( col0.m_worldTransform, col0.m_interpolationWorldTransform,
								col1.m_worldTransform, col1.m_interpolationWorldTransform, result ) )
					{
						//store result.m_fraction in both bodies

						if( col0.getHitFraction() > result.m_fraction )
							col0.setHitFraction( result.m_fraction );

						if( col1.getHitFraction() > result.m_fraction )
							col1.setHitFraction( result.m_fraction );

						if( resultFraction > result.m_fraction )
							resultFraction = result.m_fraction;
					}
					BulletGlobals.GjkConvexCastPool.Free( ccd1 );
				}
			}
			return resultFraction;

		}
	};

}
