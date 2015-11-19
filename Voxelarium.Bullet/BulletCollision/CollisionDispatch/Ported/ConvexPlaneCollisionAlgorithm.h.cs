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

	/// btSphereBoxCollisionAlgorithm  provides sphere-box collision detection.
	/// Other features are frame-coherency (persistent data) and collision response.
	internal class btConvexPlaneCollisionAlgorithm : btCollisionAlgorithm
	{
		bool m_ownManifold;
		btPersistentManifold m_manifoldPtr;
		internal bool m_swapped;
		int m_numPerturbationIterations;
		int m_minimumPointsPerturbationThreshold;

		internal override void getAllContactManifolds( btManifoldArray manifoldArray )
		{
			if( m_manifoldPtr != null && m_ownManifold )
			{
				manifoldArray.Add( m_manifoldPtr );
			}
		}

		internal class CreateFunc : btCollisionAlgorithmCreateFunc
		{
			internal int m_numPerturbationIterations;
			internal int m_minimumPointsPerturbationThreshold;

			internal CreateFunc()
			{
				m_numPerturbationIterations = ( 1 );
				m_minimumPointsPerturbationThreshold = ( 0 );
			}

			internal override btCollisionAlgorithm CreateCollisionAlgorithm( btCollisionAlgorithmConstructionInfo ci, btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap )
			{
				btConvexPlaneCollisionAlgorithm ca = BulletGlobals.ConvexPlaneAlgorithmPool.Get();
				if( !m_swapped )
					ca.Initialize( null, ci, body0Wrap, body1Wrap, false, m_numPerturbationIterations, m_minimumPointsPerturbationThreshold );
				else
					ca.Initialize( null, ci, body0Wrap, body1Wrap, true, m_numPerturbationIterations, m_minimumPointsPerturbationThreshold );
				return ca;
			}

		};

		public void btConvexPlaneCollisionALgorithm() { }

		internal void Initialize( btPersistentManifold mf, btCollisionAlgorithmConstructionInfo ci, btCollisionObjectWrapper col0Wrap, btCollisionObjectWrapper col1Wrap, bool isSwapped, int numPerturbationIterations, int minimumPointsPerturbationThreshold )
		{
			base.Initialize( ci );
			m_ownManifold = ( false );
			m_manifoldPtr = ( mf );
			m_swapped = ( isSwapped );
			m_numPerturbationIterations = ( numPerturbationIterations );
			m_minimumPointsPerturbationThreshold = ( minimumPointsPerturbationThreshold );
			btCollisionObjectWrapper convexObjWrap = m_swapped ? col1Wrap : col0Wrap;
			btCollisionObjectWrapper planeObjWrap = m_swapped ? col0Wrap : col1Wrap;

			if( m_manifoldPtr == null && m_dispatcher.needsCollision( convexObjWrap.m_collisionObject, planeObjWrap.m_collisionObject ) )
			{
				m_manifoldPtr = m_dispatcher.getNewManifold( convexObjWrap.m_collisionObject, planeObjWrap.m_collisionObject );
				m_ownManifold = true;
			}
		}

		void CleanManifold()
		{
			if( m_ownManifold )
			{
				if( m_manifoldPtr != null )
					m_dispatcher.releaseManifold( m_manifoldPtr );
			}
		}

		internal override void Cleanup()
		{
			CleanManifold();
			BulletGlobals.ConvexPlaneAlgorithmPool.Free( this );
		}

		void collideSingleContact( bool usePertube, ref btQuaternion perturbeRot
			, btCollisionObjectWrapper convexObjWrap
			, ref btTransform convexTransform
			, btCollisionObjectWrapper planeObjWrap
			, ref btTransform planeTransform
			, btDispatcherInfo dispatchInfo, btManifoldResult resultOut
			, ref btVector3 planeNormal, double planeConstant
			)
		{
			//btCollisionObjectWrapper convexObjWrap = m_swapped ? body1Wrap : body0Wrap;
			//btCollisionObjectWrapper planeObjWrap = m_swapped ? body0Wrap : body1Wrap;

			btConvexShape convexShape = (btConvexShape)convexObjWrap.getCollisionShape();
			btStaticPlaneShape planeShape = (btStaticPlaneShape)planeObjWrap.getCollisionShape();

			bool hasCollision = false;
			//planeNormal = planeShape.getPlaneNormal().Copy( out planeNormal );
			//double planeConstant = planeShape.getPlaneConstant();

			btTransform convexWorldTransform = convexTransform;
			//btTransform planeWorldTransform = planeObjWrap.m_worldTransform;
			btTransform convexInPlaneTrans;
			planeTransform.inverseTimes( ref convexWorldTransform, out convexInPlaneTrans );

			if( usePertube )
			{
				//now perturbe the convex-world transform
				btMatrix3x3 perturbeMat = new btMatrix3x3( ref perturbeRot );
				btMatrix3x3 tmpPerturbe; convexWorldTransform.m_basis.Apply( ref perturbeMat, out tmpPerturbe );
				convexWorldTransform.m_basis = tmpPerturbe;
				//convexWorldTransform.getBasis() *= btMatrix3x3( perturbeRot );
			}

			btTransform planeInConvex;
			convexTransform.inverseTimes( ref planeObjWrap.m_collisionObject.m_worldTransform, out planeInConvex );

			btVector3 tmp, tmp2;
			planeNormal.Invert( out tmp );
			planeInConvex.m_basis.Apply( ref tmp, out tmp2 );
			btVector3 vtx; convexShape.localGetSupportingVertex( ref tmp2, out vtx );

			btVector3 vtxInPlane; convexInPlaneTrans.Apply( ref vtx, out vtxInPlane );
			double distance = ( planeNormal.dot( ref vtxInPlane ) - planeConstant );

			btVector3 vtxInPlaneProjected; vtxInPlane.AddScale( ref planeNormal, -distance, out vtxInPlaneProjected );
			btVector3 vtxInPlaneWorld; planeTransform.Apply( ref vtxInPlaneProjected, out vtxInPlaneWorld );

			hasCollision = distance < m_manifoldPtr.getContactBreakingThreshold();
			resultOut.setPersistentManifold( m_manifoldPtr );
			if( hasCollision )
			{
				/// report a contact. internally this will be kept persistent, and contact reduction is done
				btVector3 normalOnSurfaceB; planeTransform.m_basis.Apply( ref planeNormal, out normalOnSurfaceB );
				btScalar.Dbg( "Convex plane adds point " + normalOnSurfaceB + " " + vtxInPlaneWorld + " " + distance.ToString( "g17" ) );
				resultOut.addContactPoint( ref normalOnSurfaceB, ref vtxInPlaneWorld, distance );
			}
		}


		internal override void processCollision( btCollisionObjectWrapper body0Wrap
			, ref btTransform body0Transform
			, btCollisionObjectWrapper body1Wrap
			, ref btTransform body1Transform
			, btDispatcherInfo dispatchInfo, btManifoldResult resultOut )
		{
			//(void)dispatchInfo;
			if( m_manifoldPtr == null )
				return;
			btCollisionObjectWrapper convexObjWrap = m_swapped ? body1Wrap : body0Wrap;
			btCollisionObjectWrapper planeObjWrap = m_swapped ? body0Wrap : body1Wrap;

			btConvexShape convexShape = (btConvexShape)convexObjWrap.m_shape;
			btStaticPlaneShape planeShape = (btStaticPlaneShape)planeObjWrap.getCollisionShape();
			//btVector3 planeNormal;
			//planeShape.getPlaneNormal().Copy( out planeNormal );
			double planeConstant = planeShape.getPlaneConstant();
			if( m_swapped )
				collideSingleContact( false, ref btQuaternion.Identity, convexObjWrap
					, ref body1Transform
					, planeObjWrap
					, ref body0Transform
					, dispatchInfo, resultOut
							, ref planeShape.m_planeNormal, planeConstant );
			else
				collideSingleContact( false, ref btQuaternion.Identity, convexObjWrap
					, ref body0Transform
					, planeObjWrap
					, ref body1Transform
					, dispatchInfo, resultOut
							, ref planeShape.m_planeNormal, planeConstant );
			/*
						btCollisionObjectWrapper convexObjWrap = m_swapped ? body1Wrap : body0Wrap;
						btCollisionObjectWrapper planeObjWrap = m_swapped ? body0Wrap : body1Wrap;

						btConvexShape convexShape = (btConvexShape)convexObjWrap.m_shape;
						btStaticPlaneShape planeShape = (btStaticPlaneShape)planeObjWrap.m_shape;

						bool hasCollision = false;
						btVector3 planeNormal; planeShape.m_planeNormal.Copy( out planeNormal );
						double planeConstant = planeShape.getPlaneConstant();
						btTransform planeInConvex;
						convexObjWrap.getWorldTransform().inverseTimes( planeObjWrap.getWorldTransform(), out planeInConvex );
						btTransform convexInPlaneTrans;
						planeObjWrap.getWorldTransform().inverseTimes( convexObjWrap.getWorldTransform(), out convexInPlaneTrans );

						btVector3 invPlaneNormal;
						planeNormal.Invert( out invPlaneNormal );
						btVector3 tmp;
						planeInConvex.getBasis().Apply( planeNormal, out tmp );
						btVector3 vtx; convexShape.localGetSupportingVertex( ref tmp, out vtx );
						btVector3 vtxInPlane; convexInPlaneTrans.Apply( ref vtx, out vtxInPlane );
						double distance = ( planeNormal.dot( ref vtxInPlane ) - planeConstant );

						btVector3 vtxInPlaneProjected; vtxInPlane.AddScale( planeNormal, -distance, out vtxInPlaneProjected );
						btVector3 vtxInPlaneWorld; planeObjWrap.getWorldTransform().Apply( ref vtxInPlaneProjected, out vtxInPlaneWorld );

						hasCollision = distance < m_manifoldPtr.getContactBreakingThreshold();
						resultOut.setPersistentManifold( m_manifoldPtr );
						if( hasCollision )
						{
							/// report a contact. internally this will be kept persistent, and contact reduction is done
							btVector3 normalOnSurfaceB = planeObjWrap.getWorldTransform().getBasis() * planeNormal;
							btVector3 pOnB = vtxInPlaneWorld;
							resultOut.addContactPoint( ref normalOnSurfaceB, ref pOnB, distance );
						}
			*/
			//the perturbation algorithm doesn't work well with implicit surfaces such as spheres, cylinder and cones:
			//they keep on rolling forever because of the additional off-center contact points
			//so only enable the feature for polyhedral shapes (btBoxShape, btConvexHullShape etc)
			if( convexShape.isPolyhedral() && resultOut.m_manifoldPtr.m_cachedPoints < m_minimumPointsPerturbationThreshold )
			{
				btVector3 v0, v1;
				btVector3.btPlaneSpace1( ref planeShape.m_planeNormal, out v0, out v1 );
				//now perform 'm_numPerturbationIterations' collision queries with the perturbated collision objects

				double angleLimit = 0.125f * btScalar.SIMD_PI;
				double perturbeAngle;
				double radius = convexShape.getAngularMotionDisc();
				perturbeAngle = btPersistentManifold.gContactBreakingThreshold / radius;
				if( perturbeAngle > angleLimit )
					perturbeAngle = angleLimit;

				btQuaternion perturbeRot = new btQuaternion( ref v0, perturbeAngle );
				double interval = btScalar.SIMD_2_PI / (double)( m_numPerturbationIterations );
				for( int i = 0; i < m_numPerturbationIterations; i++ )
				{
					double iterationAngle = i * interval;
					btQuaternion rotq = new btQuaternion( ref planeShape.m_planeNormal, iterationAngle );
					btQuaternion rotqInv;
					rotq.inverse( out rotqInv );
					btQuaternion tmpq, tmpq2;
					rotqInv.Mult( ref perturbeRot, out tmpq );
					tmpq.Mult( ref rotq, out tmpq2 );
					if( m_swapped )
						collideSingleContact( true, ref tmpq2
							, convexObjWrap, ref body1Transform
							, planeObjWrap, ref body0Transform
							, dispatchInfo
										, resultOut, ref planeShape.m_planeNormal, planeConstant );
					else
						collideSingleContact( true, ref tmpq2
							, convexObjWrap, ref body0Transform
							, planeObjWrap, ref body1Transform, dispatchInfo
									, resultOut, ref planeShape.m_planeNormal, planeConstant );
				}
			}

			if( m_ownManifold )
			{
				if( m_manifoldPtr.m_cachedPoints != 0 )
				{
					resultOut.refreshContactPoints();
				}
			}
		}

		internal override double calculateTimeOfImpact( btCollisionObject col0, btCollisionObject col1, btDispatcherInfo dispatchInfo, btManifoldResult resultOut )
		{
			//not yet
			return btScalar.BT_ONE;
		}


	};

}
