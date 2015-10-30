/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


using System.Collections.Generic;
using Bullet.Collision.BroadPhase;
using Bullet.Collision.Dispatch;
using Bullet.Collision.NarrowPhase;
using Bullet.Collision.Shapes;
using Bullet.Dynamics.ConstraintSolver;
using Bullet.LinearMath;
using Bullet.Types;

namespace Bullet.Dynamics
{

	///btDiscreteDynamicsWorld provides discrete rigid body simulation
	///those classes replace the obsolete CcdPhysicsEnvironment/CcdPhysicsController
	public partial class btDiscreteDynamicsWorld : btDynamicsWorld
	{

		protected btList<btTypedConstraint> m_sortedConstraints = new btList<btTypedConstraint>();
		internal InplaceSolverIslandCallback m_solverIslandCallback;

		internal btConstraintSolver m_constraintSolver;

		protected btSimulationIslandManager m_islandManager;

		protected btList<btTypedConstraint> m_constraints;

		protected btList<btRigidBody> m_nonStaticRigidBodies;

		protected btVector3 m_gravity;
		double m_g;

		//for variable timesteps
		double m_localTime;
		double m_fixedTimeStep;
		//for variable timesteps

		bool m_ownsIslandManager;
		bool m_ownsConstraintSolver;
		bool m_synchronizeAllMotionStates;
		bool m_applySpeculativeContactRestitution;

		List<btActionInterface> m_actions;

		int m_profileTimings;

		bool m_latencyMotionStateInterpolation;

		btList<btPersistentManifold> m_predictiveManifolds;

		public virtual void predictUnconstraintMotion( double timeStep )
		{
			CProfileSample sample = new CProfileSample( "predictUnconstraintMotion" );
			for( int i = 0; i < m_nonStaticRigidBodies.Count; i++ )
			{
				btRigidBody body = m_nonStaticRigidBodies[i];
				if( !body.isStaticOrKinematicObject() )
				{
					//don't integrate/update velocities here, it happens in the constraint solver

					body.applyDamping( timeStep );

					body.predictIntegratedTransform( timeStep, out body.m_interpolationWorldTransform );
				}
			}
		}


		public virtual void integrateTransforms( double timeStep )
		{
			CProfileSample sample = new CProfileSample( "integrateTransforms" );
			btTransform predictedTrans;
			for( int i = 0; i < m_nonStaticRigidBodies.Count; i++ )
			{
				btRigidBody body = m_nonStaticRigidBodies[i];
				body.setHitFraction( 1 );

				if( body.isActive() && ( !body.isStaticOrKinematicObject() ) )
				{

					body.predictIntegratedTransform( timeStep, out predictedTrans );
					btVector3 delta;
					predictedTrans.m_origin.Sub( ref body.m_worldTransform.m_origin, out delta );
					double squareMotion = ( delta ).length2();



					if( m_dispatchInfo.m_useContinuous && body.getCcdSquareMotionThreshold() != 0 && body.getCcdSquareMotionThreshold() < squareMotion )
					{
						CProfileSample sample2 = new CProfileSample( "CCD motion clamping" );
						if( body.getCollisionShape().isConvex() )
						{
							gNumClampedCcdMotions++;
#if USE_STATIC_ONLY
					class StaticOnlyCallback : btClosestNotMeConvexResultCallback
{
	public:

						StaticOnlyCallback( btCollisionObject me, ref btVector3 fromA, ref btVector3 toA, btOverlappingPairCache* pairCache, btDispatcher* dispatcher ) :
                          btClosestNotMeConvexResultCallback( me, fromA, toA, pairCache, dispatcher)
	{
	}

	virtual bool needsCollision( btBroadphaseProxy* proxy0 )
	{
		btCollisionObject otherObj = (btCollisionObject)proxy0.m_clientObject;
		if( !otherObj.isStaticOrKinematicObject() )
			return false;
		return btClosestNotMeConvexResultCallback::needsCollision( proxy0 );
	}
};

StaticOnlyCallback sweepResults( body, body.getWorldTransform().getOrigin(),predictedTrans.getOrigin(),getBroadphase().getOverlappingPairCache(),getDispatcher());
#else
							btClosestNotMeConvexResultCallback sweepResults
								= new btClosestNotMeConvexResultCallback( body, ref body.m_worldTransform.m_origin
											, ref predictedTrans.m_origin, getBroadphase().getOverlappingPairCache(), getDispatcher() );
#endif
							//btConvexShape* convexShape = static_cast<btConvexShape*>(body.getCollisionShape());
							btSphereShape tmpSphere = new btSphereShape( body.getCcdSweptSphereRadius() );//btConvexShape* convexShape = static_cast<btConvexShape*>(body.getCollisionShape());
							sweepResults.m_allowedPenetration = getDispatchInfo().m_allowedCcdPenetration;

							sweepResults.m_collisionFilterGroup = body.getBroadphaseProxy().m_collisionFilterGroup;
							sweepResults.m_collisionFilterMask = body.getBroadphaseProxy().m_collisionFilterMask;
							btTransform modifiedPredictedTrans = predictedTrans;
							modifiedPredictedTrans.setBasis( ref body.m_worldTransform.m_basis );

							convexSweepTest( tmpSphere, ref body.m_worldTransform, modifiedPredictedTrans, sweepResults );
							if( sweepResults.hasHit() && ( sweepResults.m_closestHitFraction < 1 ) )
							{

								//Console.WriteLine("clamped integration to hit fraction = %f\n",fraction);
								body.setHitFraction( sweepResults.m_closestHitFraction );
								body.predictIntegratedTransform( timeStep * body.getHitFraction(), predictedTrans );
								body.setHitFraction( 0 );
								body.proceedToTransform( predictedTrans );

#if 0
						btVector3 linVel = body.getLinearVelocity();

						double maxSpeed = body.getCcdMotionThreshold()/getSolverInfo().m_timeStep;
						double maxSpeedSqr = maxSpeed*maxSpeed;
						if (linVel.length2()>maxSpeedSqr)
						{
							linVel.normalize();
							linVel*= maxSpeed;
							body.setLinearVelocity(linVel);
							double ms2 = body.getLinearVelocity().length2();
							body.predictIntegratedTransform(timeStep, predictedTrans);

							double sm2 = (predictedTrans.getOrigin()-body.getWorldTransform().getOrigin()).length2();
							double smt = body.getCcdSquareMotionThreshold();
							Console.WriteLine("sm2=%f\n",sm2);
						}
#else

								//don't apply the collision response right now, it will happen next frame
								//if you really need to, you can uncomment next 3 lines. Note that is uses zero restitution.
								//double appliedImpulse = 0;
								//double depth = 0;
								//appliedImpulse = resolveSingleCollision(body,(btCollisionObject)sweepResults.m_hitCollisionObject,sweepResults.m_hitPointWorld,sweepResults.m_hitNormalWorld,getSolverInfo(), depth);


#endif

								continue;
							}
						}
					}


					body.proceedToTransform( predictedTrans );

				}

			}

			///this should probably be switched on by default, but it is not well tested yet
			if( m_applySpeculativeContactRestitution )
			{
				CProfileSample sample = new CProfileSample( "apply speculative contact restitution" );
				for( int i = 0; i < m_predictiveManifolds.Count; i++ )
				{
					btPersistentManifold* manifold = m_predictiveManifolds[i];
					btRigidBody body0 = btRigidBody::upcast( (btCollisionObject)manifold.getBody0() );
					btRigidBody body1 = btRigidBody::upcast( (btCollisionObject)manifold.getBody1() );

					for( int p = 0; p < manifold.getNumContacts(); p++ )
					{
						btManifoldPoint pt = manifold.getContactPoint( p );
						double combinedRestitution = btManifoldResult::calculateCombinedRestitution( body0, body1 );

						if( combinedRestitution > 0 && pt.m_appliedImpulse != 0 )
						//if (pt.getDistance()>0 && combinedRestitution>0 && pt.m_appliedImpulse != 0)
						{
							btVector3 imp = -pt.m_normalWorldOnB * pt.m_appliedImpulse * combinedRestitution;

					ref btVector3 pos1 = pt.getPositionWorldOnA();
					ref btVector3 pos2 = pt.getPositionWorldOnB();

							btVector3 rel_pos0 = pos1 - body0.getWorldTransform().getOrigin();
							btVector3 rel_pos1 = pos2 - body1.getWorldTransform().getOrigin();

							if( body0 )
								body0.applyImpulse( imp, rel_pos0 );
							if( body1 )
								body1.applyImpulse( -imp, rel_pos1 );
						}
					}
				}
			}

		}



		virtual void calculateSimulationIslands();

		virtual void solveConstraints( btContactSolverInfo& solverInfo);

		virtual void updateActivationState( double timeStep );

		void updateActions( double timeStep );

		void startProfiling( double timeStep );

		virtual void internalSingleStepSimulation( double timeStep );

		void createPredictiveContacts( double timeStep );

		virtual void saveKinematicState( double timeStep );

		void serializeRigidBodies( btSerializer serializer );

		void serializeDynamicsWorldInfo( btSerializer serializer );

		public:


	

	///this btDiscreteDynamicsWorld constructor gets created objects from the user, and will not delete those
	btDiscreteDynamicsWorld( btDispatcher dispatcher, btBroadphaseInterface pairCache, btConstraintSolver constraintSolver, btCollisionConfiguration collisionConfiguration );

		virtual ~btDiscreteDynamicsWorld();

		///if maxSubSteps > 0, it will interpolate motion between fixedTimeStep's
		virtual int stepSimulation( double timeStep, int maxSubSteps = 1, double fixedTimeStep = btScalar.BT_ONE_OVER_SIXTY );


		virtual void synchronizeMotionStates();

		///this can be useful to synchronize a single rigid body . graphics object
		void synchronizeSingleMotionState( btRigidBody body );

		virtual void addConstraint( btTypedConstraint constraint, bool disableCollisionsBetweenLinkedBodies = false );

		virtual void removeConstraint( btTypedConstraint constraint );

		virtual void addAction( btActionInterface);

		virtual void removeAction( btActionInterface);

		btSimulationIslandManager getSimulationIslandManager()
		{
			return m_islandManager;
		}

		btSimulationIslandManager getSimulationIslandManager()
		{
			return m_islandManager;
		}

		btCollisionWorld getCollisionWorld()
		{
			return this;
		}

		virtual void setGravity( btVector3 gravity );

		virtual btVector3 getGravity();

		virtual void addCollisionObject( btCollisionObject collisionObject, short int collisionFilterGroup = btBroadphaseProxy::StaticFilter, short int collisionFilterMask = btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter );

		virtual void addRigidBody( btRigidBody body );

		virtual void addRigidBody( btRigidBody body, short group, short mask );

		virtual void removeRigidBody( btRigidBody body );

		///removeCollisionObject will first check if it is a rigid body, if so call removeRigidBody otherwise call btCollisionWorld::removeCollisionObject
		virtual void removeCollisionObject( btCollisionObject collisionObject );


		virtual void debugDrawConstraint( btTypedConstraint constraint );

		virtual void debugDrawWorld();

		virtual void setConstraintSolver( btConstraintSolver solver );

		virtual btConstraintSolver getConstraintSolver();

		virtual int getNumConstraints();

		virtual btTypedConstraint getConstraint( int index );

		virtual btTypedConstraint getConstraint( int index );


		virtual btDynamicsWorldType getWorldType()
		{
			return BT_DISCRETE_DYNAMICS_WORLD;
		}

		///the forces on each rigidbody is accumulating together with gravity. clear this after each timestep.
		virtual void clearForces();

		///apply gravity, call this once per timestep
		virtual void applyGravity();

		virtual void setNumTasks( int numTasks )
		{
			(void)numTasks;
		}

		///obsolete, use updateActions instead
		virtual void updateVehicles( double timeStep )
		{
			updateActions( timeStep );
		}

		///obsolete, use addAction instead
		virtual void addVehicle( btActionInterface vehicle );
		///obsolete, use removeAction instead
		virtual void removeVehicle( btActionInterface vehicle );
		///obsolete, use addAction instead
		virtual void addCharacter( btActionInterface character );
		///obsolete, use removeAction instead
		virtual void removeCharacter( btActionInterface character );

		void setSynchronizeAllMotionStates( bool synchronizeAll )
		{
			m_synchronizeAllMotionStates = synchronizeAll;
		}
		bool getSynchronizeAllMotionStates()
		{
			return m_synchronizeAllMotionStates;
		}

		void setApplySpeculativeContactRestitution( bool enable )
		{
			m_applySpeculativeContactRestitution = enable;
		}

		bool getApplySpeculativeContactRestitution()
		{
			return m_applySpeculativeContactRestitution;
		}

		///Preliminary serialization test for Bullet 2.76. Loading those files requires a separate parser (see Bullet/Demos/SerializeDemo)
		virtual void serialize( btSerializer serializer );

		///Interpolate motion state between previous and current transform, instead of current and next transform.
		///This can relieve discontinuities in the rendering, due to penetrations
		void setLatencyMotionStateInterpolation( bool latencyInterpolation )
		{
			m_latencyMotionStateInterpolation = latencyInterpolation;
		}
		bool getLatencyMotionStateInterpolation()
		{
			return m_latencyMotionStateInterpolation;
		}


		class btClosestNotMeConvexResultCallback : btCollisionWorld.ClosestConvexResultCallback
		{

			public btCollisionObject m_me;
			public double m_allowedPenetration;
			public btOverlappingPairCache m_pairCache;
			public btDispatcher m_dispatcher;

			public
			btClosestNotMeConvexResultCallback( btCollisionObject me, ref btVector3 fromA, ref btVector3 toA, btOverlappingPairCache pairCache, btDispatcher dispatcher ) : base
					( ref fromA, ref toA )
			{
				m_me = ( me );
				m_allowedPenetration = ( 0.0f );
				m_pairCache = ( pairCache );
				m_dispatcher = ( dispatcher );
			}

			public virtual double addSingleResult( ref btCollisionWorld.LocalConvexResult convexResult, bool normalInWorldSpace )
			{
				if( convexResult.m_hitCollisionObject == m_me )
					return 1.0f;

				//ignore result if there is no contact response
				if( !convexResult.m_hitCollisionObject.hasContactResponse() )
					return 1.0f;

				btVector3 linVelA, linVelB;
				linVelA = m_convexToWorld - m_convexFromWorld;
				linVelB = btVector3.Zero;//toB.getOrigin()-fromB.getOrigin();

				btVector3 relativeVelocity = ( linVelA - linVelB );
				//don't report time of impact for motion away from the contact normal (or causes minor penetration)
				if( convexResult.m_hitNormalLocal.dot( relativeVelocity ) >= -m_allowedPenetration )
					return 1;

				return ClosestConvexResultCallback::addSingleResult( convexResult, normalInWorldSpace );
			}

			public virtual bool needsCollision( btBroadphaseProxy* proxy0 )
			{
				//don't collide with itself
				if( proxy0.m_clientObject == m_me )
					return false;

				///don't do CCD when the collision filters are not matching
				if( !ClosestConvexResultCallback::needsCollision( proxy0 ) )
					return false;

				btCollisionObject otherObj = (btCollisionObject)proxy0.m_clientObject;

				//call needsResponse, see http://code.google.com/p/bullet/issues/detail?id=179
				if( m_dispatcher.needsResponse( m_me, otherObj ) )
				{
#if 0
			///don't do CCD when there are already contact points (touching contact/penetration)
			List<btPersistentManifold*> manifoldArray;
			btBroadphasePair* collisionPair = m_pairCache.findPair(m_me.getBroadphaseHandle(),proxy0);
			if (collisionPair)
			{
				if (collisionPair.m_algorithm)
				{
					manifoldArray.resize(0);
					collisionPair.m_algorithm.getAllContactManifolds(manifoldArray);
					for (int j=0;j<manifoldArray.Count;j++)
					{
						btPersistentManifold* manifold = manifoldArray[j];
						if (manifold.getNumContacts()>0)
							return false;
					}
				}
			}
#endif
					return true;
				}

				return false;
			}


		};

	};

}
