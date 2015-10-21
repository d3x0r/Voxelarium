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

using System.Diagnostics;
using Bullet.BulletCollision;
using Bullet.Collision.BroadPhase;
using Bullet.Collision.Dispatch;
using Bullet.Collision.NarrowPhase;
using Bullet.Dynamics.ConstraintSolver;
using Bullet.LinearMath;
using Bullet.Types;

namespace Bullet.Dynamics
{


	class btSortConstraintOnIslandPredicate
	{
		public bool compare(btTypedConstraint lhs, btTypedConstraint rhs)
		{
            int rIslandId0, lIslandId0;
		rIslandId0 = btDiscreteDynamicsWorld.btGetConstraintIslandId( rhs);
		lIslandId0 = btDiscreteDynamicsWorld.btGetConstraintIslandId( lhs);
			return lIslandId0<rIslandId0;
		}
};

public class InplaceSolverIslandCallback : btSimulationIslandManager.IslandCallback
{
	btContactSolverInfo m_solverInfo;
	btConstraintSolver m_solver;
	btTypedConstraint[] m_sortedConstraints;
	int m_numConstraints;
	btIDebugDraw m_debugDrawer;
	btDispatcher m_dispatcher;

	btList<btCollisionObject> m_bodies;
	btList<btPersistentManifold> m_manifolds;
	btList<btTypedConstraint> m_constraints;


	InplaceSolverIslandCallback(
		btConstraintSolver solver,
		btStackAlloc stackAlloc,
		btDispatcher dispatcher )
		{
			m_solverInfo = null;
        m_solver( solver ),
        m_sortedConstraintsnull,
        m_numConstraints( 0 ),
        m_debugDrawernull,
        m_dispatcher( dispatcher )

	}

		InplaceSolverIslandCallback operator=(InplaceSolverIslandCallback& other)
	{
		Debug.Assert(false);
		(void)other;
		return *this;
	}

public void setup( btContactSolverInfo* solverInfo, btTypedConstraint[] sortedConstraints, int numConstraints, btIDebugDraw* debugDrawer )
{
	Debug.Assert( solverInfo );
	m_solverInfo = solverInfo;
	m_sortedConstraints = sortedConstraints;
	m_numConstraints = numConstraints;
	m_debugDrawer = debugDrawer;
	m_bodies.resize( 0 );
	m_manifolds.resize( 0 );
	m_constraints.resize( 0 );
}


virtual void processIsland( btCollisionObject bodies, int numBodies, btPersistentManifold[] manifolds, int numManifolds, int islandId )
{
	if( islandId < 0 )
	{
		///we don't split islands, so all constraints/contact manifolds/bodies are passed into the solver regardless the island id
		m_solver.solveGroup( bodies, numBodies, manifolds, numManifolds, &m_sortedConstraints, m_numConstraints, *m_solverInfo, m_debugDrawer, m_dispatcher );
	}
	else
	{
		//also add all non-contact constraints/joints for this island
		btTypedConstraint[] startConstraint = 0;
		int numCurConstraints = 0;
		int i;

		//find the first constraint for this island
		for( i = 0; i < m_numConstraints; i++ )
		{
			if( btGetConstraintIslandId( m_sortedConstraints[i] ) == islandId )
			{
				startConstraint = &m_sortedConstraints[i];
				break;
			}
		}
		//count the number of constraints in this island
		for( ; i < m_numConstraints; i++ )
		{
			if( btGetConstraintIslandId( m_sortedConstraints[i] ) == islandId )
			{
				numCurConstraints++;
			}
		}

		if( m_solverInfo.m_minimumSolverBatchSize <= 1 )
		{
			m_solver.solveGroup( bodies, numBodies, manifolds, numManifolds, startConstraint, numCurConstraints, *m_solverInfo, m_debugDrawer, m_dispatcher );
		}
		else
		{

			for( i = 0; i < numBodies; i++ )
				m_bodies.Add( bodies[i] );
			for( i = 0; i < numManifolds; i++ )
				m_manifolds.Add( manifolds[i] );
			for( i = 0; i < numCurConstraints; i++ )
				m_constraints.Add( startConstraint[i] );
			if( ( m_constraints.Count + m_manifolds.Count ) > m_solverInfo.m_minimumSolverBatchSize )
			{
				processConstraints();
			}
			else
			{
				//Console.WriteLine("deferred\n");
			}
		}
	}
}
void processConstraints()
{

	btCollisionObject bodies = m_bodies.Count ? m_bodies : 0;
	btPersistentManifold[] manifold = m_manifolds.Count ? m_manifolds : 0;
	btTypedConstraint[] constraints = m_constraints.Count ? m_constraints : 0;

	m_solver.solveGroup( bodies, m_bodies.Count, manifold, m_manifolds.Count, constraints, m_constraints.Count, *m_solverInfo, m_debugDrawer, m_dispatcher );
	m_bodies.resize( 0 );
	m_manifolds.resize( 0 );
	m_constraints.resize( 0 );

}

};


public partial class btDiscreteDynamicsWorld
{

	public static int btGetConstraintIslandId( btTypedConstraint lhs )
	{
		int islandId;

		btCollisionObject  rcolObj0 = lhs.getRigidBodyA();
		btCollisionObject  rcolObj1 = lhs.getRigidBodyB();
		islandId = rcolObj0.getIslandTag() >= 0 ? rcolObj0.getIslandTag() : rcolObj1.getIslandTag();
		return islandId;

	}


	public btDiscreteDynamicsWorld( btDispatcher dispatcher, btBroadphaseInterface pairCache, btConstraintSolver constraintSolver, btCollisionConfiguration collisionConfiguration )
				:base( dispatcher, pairCache, collisionConfiguration)
	{
		m_solverIslandCallback = null;
        m_constraintSolver = ( constraintSolver);
		m_gravity= new btVector3(0,-10,0);
        m_localTime = (0);
		m_fixedTimeStep = (0);
        m_synchronizeAllMotionStates = (false);
        m_applySpeculativeContactRestitution = (false);
        m_profileTimings = (0);
        m_latencyMotionStateInterpolation = (true);

		if( m_constraintSolver == null )
		{
			m_constraintSolver = new btSequentialImpulseConstraintSolver();
			m_ownsConstraintSolver = true;
		}
		else
		{
			m_ownsConstraintSolver = false;
		}

		m_islandManager = new btSimulationIslandManager();

		m_ownsIslandManager = true;

		m_solverIslandCallback = new InplaceSolverIslandCallback( m_constraintSolver, 0, dispatcher );
	}


	~btDiscreteDynamicsWorld()
	{
		//only delete it when we created it
		if( m_ownsIslandManager )
		{
			m_islandManager.~btSimulationIslandManager();
			btAlignedFree( m_islandManager );
		}
		if( m_solverIslandCallback )
		{
			m_solverIslandCallback.~InplaceSolverIslandCallback();
			btAlignedFree( m_solverIslandCallback );
		}
		if( m_ownsConstraintSolver )
		{

			m_constraintSolver.~btConstraintSolver();
			btAlignedFree( m_constraintSolver );
		}
	}

	void btDiscreteDynamicsWorld::saveKinematicState( double timeStep )
	{
		///would like to iterate over m_nonStaticRigidBodies, but unfortunately old API allows
		///to switch status _after_ adding kinematic objects to the world
		///fix it for Bullet 3.x release
		for( int i = 0; i < m_collisionObjects.Count; i++ )
		{
			btCollisionObject colObj = m_collisionObjects[i];
			btRigidBody body = btRigidBody::upcast( colObj );
			if( body && body.getActivationState() != ISLAND_SLEEPING )
			{
				if( body.isKinematicObject() )
				{
					//to calculate velocities next frame
					body.saveKinematicState( timeStep );
				}
			}
		}

	}

	void btDiscreteDynamicsWorld::debugDrawWorld()
	{
		CProfileSample sample = new CProfileSample( "debugDrawWorld" );

		btCollisionWorld::debugDrawWorld();

		bool drawConstraints = false;
		if( getDebugDrawer() )
		{
			int mode = getDebugDrawer().getDebugMode();
			if( mode & ( btIDebugDraw::DBG_DrawConstraints | btIDebugDraw::DBG_DrawConstraintLimits ) )
			{
				drawConstraints = true;
			}
		}
		if( drawConstraints )
		{
			for( int i = getNumConstraints() - 1; i >= 0; i-- )
			{
				btTypedConstraint* constraint = getConstraint( i );
				debugDrawConstraint( constraint );
			}
		}



		if( getDebugDrawer() && ( getDebugDrawer().getDebugMode() & ( btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawAabb | btIDebugDraw::DBG_DrawNormals ) ) )
		{
			int i;

			if( getDebugDrawer() && getDebugDrawer().getDebugMode() )
			{
				for( i = 0; i < m_actions.Count; i++ )
				{
					m_actions[i].debugDraw( m_debugDrawer );
				}
			}
		}
		if( getDebugDrawer() )
			getDebugDrawer().flushLines();

	}

	void btDiscreteDynamicsWorld::clearForces()
	{
		///@todo: iterate over awake simulation islands!
		for( int i = 0; i < m_nonStaticRigidBodies.Count; i++ )
		{
			btRigidBody body = m_nonStaticRigidBodies[i];
			//need to check if next line is ok
			//it might break backward compatibility (people applying forces on sleeping objects get never cleared and accumulate on wake-up
			body.clearForces();
		}
	}

	///apply gravity, call this once per timestep
	void btDiscreteDynamicsWorld::applyGravity()
	{
		///@todo: iterate over awake simulation islands!
		for( int i = 0; i < m_nonStaticRigidBodies.Count; i++ )
		{
			btRigidBody body = m_nonStaticRigidBodies[i];
			if( body.isActive() )
			{
				body.applyGravity();
			}
		}
	}


	void btDiscreteDynamicsWorld::synchronizeSingleMotionState( btRigidBody body )
	{
		Debug.Assert( body );

		if( body.getMotionState() && !body.isStaticOrKinematicObject() )
		{
			//we need to call the update at least once, even for sleeping objects
			//otherwise the 'graphics' transform never updates properly
			///@todo: add 'dirty' flag
			//if (body.getActivationState() != ISLAND_SLEEPING)
			{
				btTransform interpolatedTransform;
				btTransformUtil::integrateTransform( body.getInterpolationWorldTransform(),
					body.getInterpolationLinearVelocity(), body.getInterpolationAngularVelocity(),
					( m_latencyMotionStateInterpolation && m_fixedTimeStep ) ? m_localTime - m_fixedTimeStep : m_localTime * body.getHitFraction(),
					interpolatedTransform );
				body.getMotionState().setWorldTransform( interpolatedTransform );
			}
		}
	}


	void btDiscreteDynamicsWorld::synchronizeMotionStates()
	{
		CProfileSample sample = new CProfileSample( "synchronizeMotionStates" );
		if( m_synchronizeAllMotionStates )
		{
			//iterate  over all collision objects
			for( int i = 0; i < m_collisionObjects.Count; i++ )
			{
				btCollisionObject colObj = m_collisionObjects[i];
				btRigidBody body = btRigidBody::upcast( colObj );
				if( body )
					synchronizeSingleMotionState( body );
			}
		}
		else
		{
			//iterate over all active rigid bodies
			for( int i = 0; i < m_nonStaticRigidBodies.Count; i++ )
			{
				btRigidBody body = m_nonStaticRigidBodies[i];
				if( body.isActive() )
					synchronizeSingleMotionState( body );
			}
		}
	}


	int btDiscreteDynamicsWorld::stepSimulation( double timeStep, int maxSubSteps, double fixedTimeStep )
	{
		startProfiling( timeStep );

		CProfileSample sample = new CProfileSample( "stepSimulation" );

		int numSimulationSubSteps = 0;

		if( maxSubSteps )
		{
			//fixed timestep with interpolation
			m_fixedTimeStep = fixedTimeStep;
			m_localTime += timeStep;
			if( m_localTime >= fixedTimeStep )
			{
				numSimulationSubSteps = int( m_localTime / fixedTimeStep );
				m_localTime -= numSimulationSubSteps * fixedTimeStep;
			}
		}
		else
		{
			//variable timestep
			fixedTimeStep = timeStep;
			m_localTime = m_latencyMotionStateInterpolation ? 0 : timeStep;
			m_fixedTimeStep = 0;
			if( btFuzzyZero( timeStep ) )
			{
				numSimulationSubSteps = 0;
				maxSubSteps = 0;
			}
			else
			{
				numSimulationSubSteps = 1;
				maxSubSteps = 1;
			}
		}

		//process some debugging flags
		if( getDebugDrawer() )
		{
			btIDebugDraw* debugDrawer = getDebugDrawer();
			gDisableDeactivation = ( debugDrawer.getDebugMode() & btIDebugDraw::DBG_NoDeactivation ) != 0;
		}
		if( numSimulationSubSteps )
		{

			//clamp the number of substeps, to prevent simulation grinding spiralling down to a halt
			int clampedSimulationSteps = ( numSimulationSubSteps > maxSubSteps ) ? maxSubSteps : numSimulationSubSteps;

			saveKinematicState( fixedTimeStep * clampedSimulationSteps );

			applyGravity();



			for( int i = 0; i < clampedSimulationSteps; i++ )
			{
				internalSingleStepSimulation( fixedTimeStep );
				synchronizeMotionStates();
			}

		}
		else
		{
			synchronizeMotionStates();
		}

		clearForces();

# ifndef BT_NO_PROFILE
		CProfileManager::Increment_Frame_Counter();
#endif //BT_NO_PROFILE

		return numSimulationSubSteps;
	}

	void btDiscreteDynamicsWorld::internalSingleStepSimulation( double timeStep )
	{

		CProfileSample sample = new CProfileSample( "internalSingleStepSimulation" );

		if( 0 != m_internalPreTickCallback )
		{
			( *m_internalPreTickCallback )( this, timeStep );
		}

		///apply gravity, predict motion
		predictUnconstraintMotion( timeStep );

		btDispatcherInfo & dispatchInfo = getDispatchInfo();

		dispatchInfo.m_timeStep = timeStep;
		dispatchInfo.m_stepCount = 0;
		dispatchInfo.m_debugDraw = getDebugDrawer();


		createPredictiveContacts( timeStep );

		///perform collision detection
		performDiscreteCollisionDetection();

		calculateSimulationIslands();


		getSolverInfo().m_timeStep = timeStep;



		///solve contact and other joint constraints
		solveConstraints( getSolverInfo() );

		///CallbackTriggers();

		///integrate transforms

		integrateTransforms( timeStep );

		///update vehicle simulation
		updateActions( timeStep );

		updateActivationState( timeStep );

		if( 0 != m_internalTickCallback )
		{
			( *m_internalTickCallback )( this, timeStep );
		}
	}

	void btDiscreteDynamicsWorld::setGravity( ref btVector3 gravity )
	{
		m_gravity = gravity;
		for( int i = 0; i < m_nonStaticRigidBodies.Count; i++ )
		{
			btRigidBody body = m_nonStaticRigidBodies[i];
			if( body.isActive() && !( body.getFlags() & BT_DISABLE_WORLD_GRAVITY ) )
			{
				body.setGravity( gravity );
			}
		}
	}

	btVector3 btDiscreteDynamicsWorld::getGravity()
	{
		return m_gravity;
	}

	void btDiscreteDynamicsWorld::addCollisionObject( btCollisionObject collisionObject, short int collisionFilterGroup, short int collisionFilterMask )
	{
		btCollisionWorld::addCollisionObject( collisionObject, collisionFilterGroup, collisionFilterMask );
	}

	void btDiscreteDynamicsWorld::removeCollisionObject( btCollisionObject collisionObject )
	{
		btRigidBody body = btRigidBody::upcast( collisionObject );
		if( body )
			removeRigidBody( body );
		else
			btCollisionWorld::removeCollisionObject( collisionObject );
	}

	void btDiscreteDynamicsWorld::removeRigidBody( btRigidBody body )
	{
		m_nonStaticRigidBodies.remove( body );
		btCollisionWorld::removeCollisionObject( body );
	}


	void btDiscreteDynamicsWorld::addRigidBody( btRigidBody body )
	{
		if( !body.isStaticOrKinematicObject() && !( body.getFlags() & BT_DISABLE_WORLD_GRAVITY ) )
		{
			body.setGravity( m_gravity );
		}

		if( body.getCollisionShape() )
		{
			if( !body.isStaticObject() )
			{
				m_nonStaticRigidBodies.Add( body );
			}
			else
			{
				body.setActivationState( ISLAND_SLEEPING );
			}

			bool isDynamic = !( body.isStaticObject() || body.isKinematicObject() );
			short collisionFilterGroup = isDynamic ? short( btBroadphaseProxy::DefaultFilter ) : short( btBroadphaseProxy::StaticFilter );
			short collisionFilterMask = isDynamic ? short( btBroadphaseProxy::AllFilter ) : short( btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter );

			addCollisionObject( body, collisionFilterGroup, collisionFilterMask );
		}
	}

	void btDiscreteDynamicsWorld::addRigidBody( btRigidBody body, short group, short mask )
	{
		if( !body.isStaticOrKinematicObject() && !( body.getFlags() & BT_DISABLE_WORLD_GRAVITY ) )
		{
			body.setGravity( m_gravity );
		}

		if( body.getCollisionShape() )
		{
			if( !body.isStaticObject() )
			{
				m_nonStaticRigidBodies.Add( body );
			}
			else
			{
				body.setActivationState( ISLAND_SLEEPING );
			}
			addCollisionObject( body, group, mask );
		}
	}


	void btDiscreteDynamicsWorld::updateActions( double timeStep )
	{
		CProfileSample sample = new CProfileSample( "updateActions" );

		for( int i = 0; i < m_actions.Count; i++ )
		{
			m_actions[i].updateAction( this, timeStep );
		}
	}


	void btDiscreteDynamicsWorld::updateActivationState( double timeStep )
	{
		CProfileSample sample = new CProfileSample( "updateActivationState" );

		for( int i = 0; i < m_nonStaticRigidBodies.Count; i++ )
		{
			btRigidBody body = m_nonStaticRigidBodies[i];
			if( body )
			{
				body.updateDeactivation( timeStep );

				if( body.wantsSleeping() )
				{
					if( body.isStaticOrKinematicObject() )
					{
						body.setActivationState( ISLAND_SLEEPING );
					}
					else
					{
						if( body.getActivationState() == ACTIVE_TAG )
							body.setActivationState( WANTS_DEACTIVATION );
						if( body.getActivationState() == ISLAND_SLEEPING )
						{
							body.setAngularVelocity( btVector3.Zero );
							body.setLinearVelocity( btVector3.Zero );
						}

					}
				}
				else
				{
					if( body.getActivationState() != DISABLE_DEACTIVATION )
						body.setActivationState( ACTIVE_TAG );
				}
			}
		}
	}

	void btDiscreteDynamicsWorld::addConstraint( btTypedConstraint* constraint, bool disableCollisionsBetweenLinkedBodies )
	{
		m_constraints.Add( constraint );
		//Make sure the two bodies of a type constraint are different (possibly add this to the btTypedConstraint constructor?)
		Debug.Assert( &constraint.getRigidBodyA() != &constraint.getRigidBodyB() );

		if( disableCollisionsBetweenLinkedBodies )
		{
			constraint.getRigidBodyA().addConstraintRef( constraint );
			constraint.getRigidBodyB().addConstraintRef( constraint );
		}
	}

	void btDiscreteDynamicsWorld::removeConstraint( btTypedConstraint* constraint )
	{
		m_constraints.remove( constraint );
		constraint.getRigidBodyA().removeConstraintRef( constraint );
		constraint.getRigidBodyB().removeConstraintRef( constraint );
	}

	void btDiscreteDynamicsWorld::addAction( btActionInterface* action )
	{
		m_actions.Add( action );
	}

	void btDiscreteDynamicsWorld::removeAction( btActionInterface* action )
	{
		m_actions.remove( action );
	}


	void btDiscreteDynamicsWorld::addVehicle( btActionInterface* vehicle )
	{
		addAction( vehicle );
	}

	void btDiscreteDynamicsWorld::removeVehicle( btActionInterface* vehicle )
	{
		removeAction( vehicle );
	}

	void btDiscreteDynamicsWorld::addCharacter( btActionInterface* character )
	{
		addAction( character );
	}

	void btDiscreteDynamicsWorld::removeCharacter( btActionInterface* character )
	{
		removeAction( character );
	}




	void btDiscreteDynamicsWorld::solveConstraints( btContactSolverInfo& solverInfo)
	{
		CProfileSample sample = new CProfileSample( "solveConstraints" );

		m_sortedConstraints[m_constraints.Count - 1] = null;
        //m_sortedConstraints.resize( m_constraints.Count );
		int i;
		for( i = 0; i < getNumConstraints(); i++ )
		{
			m_sortedConstraints[i] = m_constraints[i];
		}

		//	Debug.Assert(false);



		m_sortedConstraints.quickSort( btSortConstraintOnIslandPredicate() );

		btTypedConstraint[] constraintsPtr = getNumConstraints() ? m_sortedConstraints : 0;

		m_solverIslandCallback.setup( &solverInfo, constraintsPtr, m_sortedConstraints.Count, getDebugDrawer() );
		m_constraintSolver.prepareSolve( getCollisionWorld().getNumCollisionObjects(), getCollisionWorld().getDispatcher().getNumManifolds() );

		/// solve all the constraints for this island
		m_islandManager.buildAndProcessIslands( getCollisionWorld().getDispatcher(), getCollisionWorld(), m_solverIslandCallback );

		m_solverIslandCallback.processConstraints();

		m_constraintSolver.allSolved( solverInfo, m_debugDrawer );
	}


	void btDiscreteDynamicsWorld::calculateSimulationIslands()
	{
		CProfileSample sample = new CProfileSample( "calculateSimulationIslands" );

		getSimulationIslandManager().updateActivationState( getCollisionWorld(), getCollisionWorld().getDispatcher() );

		{
			//merge islands based on speculative contact manifolds too
			for( int i = 0; i < this.m_predictiveManifolds.Count; i++ )
			{
				btPersistentManifold* manifold = m_predictiveManifolds[i];

				btCollisionObject colObj0 = manifold.getBody0();
				btCollisionObject colObj1 = manifold.getBody1();

				if( ( ( colObj0 ) && ( !( colObj0 ).isStaticOrKinematicObject() ) ) &&
					( ( colObj1 ) && ( !( colObj1 ).isStaticOrKinematicObject() ) ) )
				{
					getSimulationIslandManager().getUnionFind().unite( ( colObj0 ).getIslandTag(), ( colObj1 ).getIslandTag() );
				}
			}
		}

		{
			int i;
			int numConstraints = int( m_constraints.Count );
			for( i = 0; i < numConstraints; i++ )
			{
				btTypedConstraint* constraint = m_constraints[i];
				if( constraint.isEnabled() )
				{
					btRigidBody colObj0 = &constraint.getRigidBodyA();
					btRigidBody colObj1 = &constraint.getRigidBodyB();

					if( ( ( colObj0 ) && ( !( colObj0 ).isStaticOrKinematicObject() ) ) &&
						( ( colObj1 ) && ( !( colObj1 ).isStaticOrKinematicObject() ) ) )
					{
						getSimulationIslandManager().getUnionFind().unite( ( colObj0 ).getIslandTag(), ( colObj1 ).getIslandTag() );
					}
				}
			}
		}

		//Store the island id in each body
		getSimulationIslandManager().storeIslandActivationState( getCollisionWorld() );


	}





	///internal debugging variable. this value shouldn't be too high
	int gNumClampedCcdMotions = 0;


	void btDiscreteDynamicsWorld::createPredictiveContacts( double timeStep )
	{
		CProfileSample sample = new CProfileSample( "createPredictiveContacts" );

		{
			CProfileSample sample = new CProfileSample( "release predictive contact manifolds" );

			for( int i = 0; i < m_predictiveManifolds.Count; i++ )
			{
				btPersistentManifold* manifold = m_predictiveManifolds[i];
				this.m_dispatcher1.releaseManifold( manifold );
			}
			m_predictiveManifolds.clear();
		}

		btTransform predictedTrans;
		for( int i = 0; i < m_nonStaticRigidBodies.Count; i++ )
		{
			btRigidBody body = m_nonStaticRigidBodies[i];
			body.setHitFraction( 1 );

			if( body.isActive() & ( !body.isStaticOrKinematicObject() ) )
			{

				body.predictIntegratedTransform( timeStep, predictedTrans );

				double squareMotion = ( predictedTrans.getOrigin() - body.getWorldTransform().getOrigin() ).length2();

				if( getDispatchInfo().m_useContinuous && body.getCcdSquareMotionThreshold() && body.getCcdSquareMotionThreshold() < squareMotion )
				{
					CProfileSample sample = new CProfileSample( "predictive convexSweepTest" );
					if( body.getCollisionShape().isConvex() )
					{
						gNumClampedCcdMotions++;
#if PREDICTIVE_CONTACT_USE_STATIC_ONLY
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
					btClosestNotMeConvexResultCallback sweepResults( body, body.getWorldTransform().getOrigin(),predictedTrans.getOrigin(),getBroadphase().getOverlappingPairCache(),getDispatcher());
#endif
					//btConvexShape* convexShape = static_cast<btConvexShape*>(body.getCollisionShape());
					btSphereShape tmpSphere( body.getCcdSweptSphereRadius());//btConvexShape* convexShape = static_cast<btConvexShape*>(body.getCollisionShape());
					sweepResults.m_allowedPenetration=getDispatchInfo().m_allowedCcdPenetration;

					sweepResults.m_collisionFilterGroup = body.getBroadphaseProxy().m_collisionFilterGroup;
					sweepResults.m_collisionFilterMask  = body.getBroadphaseProxy().m_collisionFilterMask;
					btTransform modifiedPredictedTrans = predictedTrans;
	modifiedPredictedTrans.setBasis(body.getWorldTransform().getBasis());

					convexSweepTest(&tmpSphere, body.getWorldTransform(),modifiedPredictedTrans,sweepResults);
					if (sweepResults.hasHit() && (sweepResults.m_closestHitFraction< 1))
					{

						btVector3 distVec = ( predictedTrans.getOrigin() - body.getWorldTransform().getOrigin() ) * sweepResults.m_closestHitFraction;
	double distance = distVec.dot( -sweepResults.m_hitNormalWorld );


	btPersistentManifold* manifold = m_dispatcher1.getNewManifold( body, sweepResults.m_hitCollisionObject );
	m_predictiveManifolds.Add(manifold);

						btVector3 worldPointB = body.getWorldTransform().getOrigin() + distVec;
	btVector3 localPointB = sweepResults.m_hitCollisionObject.getWorldTransform().inverse() * worldPointB;

	btManifoldPoint newPoint( btVector3(0,0,0), localPointB,sweepResults.m_hitNormalWorld,distance);

						bool isPredictive = true;
	int index = manifold.addManifoldPoint( newPoint, isPredictive );
	btManifoldPoint pt = manifold.getContactPoint(index);
						pt.m_combinedRestitution = 0;
						pt.m_combinedFriction = btManifoldResult::calculateCombinedFriction(body,sweepResults.m_hitCollisionObject);
						pt.m_positionWorldOnA = body.getWorldTransform().getOrigin();
	pt.m_positionWorldOnB = worldPointB;

					}
				}
			}
		}
	}
}





void btDiscreteDynamicsWorld::startProfiling( double timeStep )
{
	(void)timeStep;

# ifndef BT_NO_PROFILE
	CProfileManager::Reset();
#endif //BT_NO_PROFILE

}






void btDiscreteDynamicsWorld::debugDrawConstraint( btTypedConstraint* constraint )
{
	bool drawFrames = ( getDebugDrawer().getDebugMode() & btIDebugDraw::DBG_DrawConstraints ) != 0;
	bool drawLimits = ( getDebugDrawer().getDebugMode() & btIDebugDraw::DBG_DrawConstraintLimits ) != 0;
	double dbgDrawSize = constraint.getDbgDrawSize();
	if( dbgDrawSize <= (double)( 0 ) )
	{
		return;
	}

	switch( constraint.getConstraintType() )
	{
		case POINT2POINT_CONSTRAINT_TYPE:
			{
				btPoint2PointConstraint* p2pC = (btPoint2PointConstraint*)constraint;
				btTransform tr;
				tr.setIdentity();
				btVector3 pivot = p2pC.getPivotInA();
				pivot = p2pC.getRigidBodyA().getCenterOfMassTransform() * pivot;
				tr.setOrigin( pivot );
				getDebugDrawer().drawTransform( tr, dbgDrawSize );
				// that ideally should draw the same frame
				pivot = p2pC.getPivotInB();
				pivot = p2pC.getRigidBodyB().getCenterOfMassTransform() * pivot;
				tr.setOrigin( pivot );
				if( drawFrames ) getDebugDrawer().drawTransform( tr, dbgDrawSize );
			}
			break;
		case HINGE_CONSTRAINT_TYPE:
			{
				btHingeConstraint* pHinge = (btHingeConstraint*)constraint;
				btTransform tr = pHinge.getRigidBodyA().getCenterOfMassTransform() * pHinge.getAFrame();
				if( drawFrames ) getDebugDrawer().drawTransform( tr, dbgDrawSize );
				tr = pHinge.getRigidBodyB().getCenterOfMassTransform() * pHinge.getBFrame();
				if( drawFrames ) getDebugDrawer().drawTransform( tr, dbgDrawSize );
				double minAng = pHinge.getLowerLimit();
				double maxAng = pHinge.getUpperLimit();
				if( minAng == maxAng )
				{
					break;
				}
				bool drawSect = true;
				if( !pHinge.hasLimit() )
				{
					minAng = (double)( 0 );
					maxAng = SIMD_2_PI;
					drawSect = false;
				}
				if( drawLimits )
				{
					ref btVector3 center = tr.getOrigin();
					btVector3 normal = tr.getBasis().getColumn( 2 );
					btVector3 axis = tr.getBasis().getColumn( 0 );
					getDebugDrawer().drawArc( center, normal, axis, dbgDrawSize, dbgDrawSize, minAng, maxAng, btVector3.Zero, drawSect );
				}
			}
			break;
		case CONETWIST_CONSTRAINT_TYPE:
			{
				btConeTwistConstraint* pCT = (btConeTwistConstraint*)constraint;
				btTransform tr = pCT.getRigidBodyA().getCenterOfMassTransform() * pCT.getAFrame();
				if( drawFrames ) getDebugDrawer().drawTransform( tr, dbgDrawSize );
				tr = pCT.getRigidBodyB().getCenterOfMassTransform() * pCT.getBFrame();
				if( drawFrames ) getDebugDrawer().drawTransform( tr, dbgDrawSize );
				if( drawLimits )
				{
					//double length = (double)(5);
					double length = dbgDrawSize;
					static int nSegments = 8 * 4;
					double fAngleInRadians = (double)( 2.* 3.1415926 ) * (double)( nSegments - 1 ) / (double)( nSegments );
					btVector3 pPrev = pCT.GetPointForAngle( fAngleInRadians, length );
					pPrev = tr * pPrev;
					for( int i = 0; i < nSegments; i++ )
					{
						fAngleInRadians = (double)( 2.* 3.1415926 ) * (double)i / (double)( nSegments );
						btVector3 pCur = pCT.GetPointForAngle( fAngleInRadians, length );
						pCur = tr * pCur;
						getDebugDrawer().drawLine( pPrev, pCur, btVector3.Zero );

						if( i % ( nSegments / 8 ) == 0 )
							getDebugDrawer().drawLine( tr.getOrigin(), pCur, btVector3.Zero );

						pPrev = pCur;
					}
					double tws = pCT.getTwistSpan();
					double twa = pCT.getTwistAngle();
					bool useFrameB = ( pCT.getRigidBodyB().getInvMass() > (double)( 0 ) );
					if( useFrameB )
					{
						tr = pCT.getRigidBodyB().getCenterOfMassTransform() * pCT.getBFrame();
					}
					else
					{
						tr = pCT.getRigidBodyA().getCenterOfMassTransform() * pCT.getAFrame();
					}
					btVector3 pivot = tr.getOrigin();
					btVector3 normal = tr.getBasis().getColumn( 0 );
					btVector3 axis1 = tr.getBasis().getColumn( 1 );
					getDebugDrawer().drawArc( pivot, normal, axis1, dbgDrawSize, dbgDrawSize, -twa - tws, -twa + tws, btVector3.Zero, true );

				}
			}
			break;
		case D6_SPRING_CONSTRAINT_TYPE:
		case D6_CONSTRAINT_TYPE:
			{
				btGeneric6DofConstraint* p6DOF = (btGeneric6DofConstraint*)constraint;
				btTransform tr = p6DOF.getCalculatedTransformA();
				if( drawFrames ) getDebugDrawer().drawTransform( tr, dbgDrawSize );
				tr = p6DOF.getCalculatedTransformB();
				if( drawFrames ) getDebugDrawer().drawTransform( tr, dbgDrawSize );
				if( drawLimits )
				{
					tr = p6DOF.getCalculatedTransformA();
					ref btVector3 center = p6DOF.getCalculatedTransformB().getOrigin();
					btVector3 up = tr.getBasis().getColumn( 2 );
					btVector3 axis = tr.getBasis().getColumn( 0 );
					double minTh = p6DOF.getRotationalLimitMotor( 1 ).m_loLimit;
					double maxTh = p6DOF.getRotationalLimitMotor( 1 ).m_hiLimit;
					double minPs = p6DOF.getRotationalLimitMotor( 2 ).m_loLimit;
					double maxPs = p6DOF.getRotationalLimitMotor( 2 ).m_hiLimit;
					getDebugDrawer().drawSpherePatch( center, up, axis, dbgDrawSize * (double)( .9f ), minTh, maxTh, minPs, maxPs, btVector3.Zero );
					axis = tr.getBasis().getColumn( 1 );
					double ay = p6DOF.getAngle( 1 );
					double az = p6DOF.getAngle( 2 );
					double cy = btCos( ay );
					double sy = btSin( ay );
					double cz = btCos( az );
					double sz = btSin( az );
					btVector3 ref;
					ref = cy* cz* axis[0] + cy* sz* axis[1] - sy* axis[2];
					ref[1] = -sz* axis[0] + cz* axis[1];
					ref[2] = cz* sy* axis[0] + sz* sy* axis[1] + cy* axis[2];
tr = p6DOF.getCalculatedTransformB();
					btVector3 normal = -tr.getBasis().getColumn( 0 );
double minFi = p6DOF.getRotationalLimitMotor( 0 ).m_loLimit;
double maxFi = p6DOF.getRotationalLimitMotor( 0 ).m_hiLimit;
					if(minFi > maxFi)
					{
                        getDebugDrawer().drawArc( center, normal, ref, dbgDrawSize, dbgDrawSize, -SIMD_PI, SIMD_PI, btVector3(0,0,0), false);
					}
					else if(minFi<maxFi)
					{
                        getDebugDrawer().drawArc( center, normal, ref, dbgDrawSize, dbgDrawSize, minFi, maxFi, btVector3(0,0,0), true);
					}
					tr = p6DOF.getCalculatedTransformA();
					btVector3 bbMin = p6DOF.getTranslationalLimitMotor().m_lowerLimit;
btVector3 bbMax = p6DOF.getTranslationalLimitMotor().m_upperLimit;
                    getDebugDrawer().drawBox( bbMin, bbMax, tr, btVector3(0,0,0));
				}
			}
			break;
		///note: the code for D6_SPRING_2_CONSTRAINT_TYPE is identical to D6_CONSTRAINT_TYPE, the D6_CONSTRAINT_TYPE+D6_SPRING_CONSTRAINT_TYPE will likely become obsolete/deprecated at some stage
		case D6_SPRING_2_CONSTRAINT_TYPE:
		{
			{
				btGeneric6DofSpring2Constraint* p6DOF = (btGeneric6DofSpring2Constraint*)constraint;
btTransform tr = p6DOF.getCalculatedTransformA();
				if (drawFrames) getDebugDrawer().drawTransform( tr, dbgDrawSize);
tr = p6DOF.getCalculatedTransformB();
				if (drawFrames) getDebugDrawer().drawTransform( tr, dbgDrawSize);
				if (drawLimits)
				{
					tr = p6DOF.getCalculatedTransformA();
					ref btVector3 center = p6DOF.getCalculatedTransformB().getOrigin();
btVector3 up = tr.getBasis().getColumn( 2 );
btVector3 axis = tr.getBasis().getColumn( 0 );
double minTh = p6DOF.getRotationalLimitMotor( 1 ).m_loLimit;
double maxTh = p6DOF.getRotationalLimitMotor( 1 ).m_hiLimit;
double minPs = p6DOF.getRotationalLimitMotor( 2 ).m_loLimit;
double maxPs = p6DOF.getRotationalLimitMotor( 2 ).m_hiLimit;
                    getDebugDrawer().drawSpherePatch( center, up, axis, dbgDrawSize* (double)(.9f), minTh, maxTh, minPs, maxPs, btVector3(0, 0, 0));
					axis = tr.getBasis().getColumn(1);
double ay = p6DOF.getAngle( 1 );
double az = p6DOF.getAngle( 2 );
double cy = btCos( ay );
double sy = btSin( ay );
double cz = btCos( az );
double sz = btSin( az );
btVector3 ref;
					ref[0] = cy* cz* axis[0] + cy* sz* axis[1] - sy* axis[2];
					ref[1] = -sz* axis[0] + cz* axis[1];
					ref[2] = cz* sy* axis[0] + sz* sy* axis[1] + cy* axis[2];
tr = p6DOF.getCalculatedTransformB();
					btVector3 normal = -tr.getBasis().getColumn( 0 );
double minFi = p6DOF.getRotationalLimitMotor( 0 ).m_loLimit;
double maxFi = p6DOF.getRotationalLimitMotor( 0 ).m_hiLimit;
					if (minFi > maxFi)
					{
                        getDebugDrawer().drawArc( center, normal, ref, dbgDrawSize, dbgDrawSize, -SIMD_PI, SIMD_PI, btVector3(0, 0, 0), false);
					}
					else if (minFi<maxFi)
					{
                        getDebugDrawer().drawArc( center, normal, ref, dbgDrawSize, dbgDrawSize, minFi, maxFi, btVector3(0, 0, 0), true);
					}
					tr = p6DOF.getCalculatedTransformA();
					btVector3 bbMin = p6DOF.getTranslationalLimitMotor().m_lowerLimit;
btVector3 bbMax = p6DOF.getTranslationalLimitMotor().m_upperLimit;
                    getDebugDrawer().drawBox( bbMin, bbMax, tr, btVector3(0, 0, 0));
				}
			}
			break;
		}
		case SLIDER_CONSTRAINT_TYPE:
			{
				btSliderConstraint* pSlider = (btSliderConstraint*)constraint;
btTransform tr = pSlider.getCalculatedTransformA();
				if(drawFrames) getDebugDrawer().drawTransform( tr, dbgDrawSize);
tr = pSlider.getCalculatedTransformB();
				if(drawFrames) getDebugDrawer().drawTransform( tr, dbgDrawSize);
				if(drawLimits)
				{
					btTransform tr = pSlider.getUseLinearReferenceFrameA() ? pSlider.getCalculatedTransformA() : pSlider.getCalculatedTransformB();
btVector3 li_min = tr * btVector3( pSlider.getLowerLinLimit(), 0, 0 );
btVector3 li_max = tr * btVector3( pSlider.getUpperLinLimit(), 0, 0 );
                    getDebugDrawer().drawLine( li_min, li_max, btVector3(0, 0, 0));
					btVector3 normal = tr.getBasis().getColumn( 0 );
btVector3 axis = tr.getBasis().getColumn( 1 );
double a_min = pSlider.getLowerAngLimit();
double a_max = pSlider.getUpperAngLimit();
					ref btVector3 center = pSlider.getCalculatedTransformB().getOrigin();
                    getDebugDrawer().drawArc( center, normal, axis, dbgDrawSize, dbgDrawSize, a_min, a_max, btVector3(0,0,0), true);
				}
			}
			break;
		default :
			break;
	}
	return;
}





void btDiscreteDynamicsWorld::setConstraintSolver( btConstraintSolver* solver )
{
	if( m_ownsConstraintSolver )
	{
		btAlignedFree( m_constraintSolver );
	}
	m_ownsConstraintSolver = false;
	m_constraintSolver = solver;
	m_solverIslandCallback.m_solver = solver;
}

btConstraintSolver* btDiscreteDynamicsWorld::getConstraintSolver()
{
	return m_constraintSolver;
}


int btDiscreteDynamicsWorld::getNumConstraints()
{
	return int( m_constraints.Count );
}
btTypedConstraint* btDiscreteDynamicsWorld::getConstraint( int index )
{
	return m_constraints[index];
}
btTypedConstraint* btDiscreteDynamicsWorld::getConstraint( int index )
{
	return m_constraints[index];
}



void btDiscreteDynamicsWorld::serializeRigidBodies( btSerializer* serializer )
{
	int i;
	//serialize all collision objects
	for( i = 0; i < m_collisionObjects.Count; i++ )
	{
		btCollisionObject colObj = m_collisionObjects[i];
		if( colObj.getInternalType() & btCollisionObject::CO_RIGID_BODY )
		{
			int len = colObj.calculateSerializeBufferSize();
			btChunk* chunk = serializer.allocate( len, 1 );
			string structType = colObj.serialize( chunk.m_oldPtr, serializer );
			serializer.finalizeChunk( chunk, structType, BT_RIGIDBODY_CODE, colObj );
		}
	}

	for( i = 0; i < m_constraints.Count; i++ )
	{
		btTypedConstraint* constraint = m_constraints[i];
		int size = constraint.calculateSerializeBufferSize();
		btChunk* chunk = serializer.allocate( size, 1 );
		string structType = constraint.serialize( chunk.m_oldPtr, serializer );
		serializer.finalizeChunk( chunk, structType, BT_CONSTRAINT_CODE, constraint );
	}
}




void btDiscreteDynamicsWorld::serializeDynamicsWorldInfo( btSerializer* serializer )
{
# ifdef BT_USE_DOUBLE_PRECISION
	int len = sizeof( btDynamicsWorldDoubleData );
	btChunk* chunk = serializer.allocate( len, 1 );
	btDynamicsWorldDoubleData* worldInfo = (btDynamicsWorldDoubleData*)chunk.m_oldPtr;
#else//BT_USE_DOUBLE_PRECISION
	int len = sizeof( btDynamicsWorldFloatData );
	btChunk* chunk = serializer.allocate( len, 1 );
	btDynamicsWorldFloatData* worldInfo = (btDynamicsWorldFloatData*)chunk.m_oldPtr;
#endif//BT_USE_DOUBLE_PRECISION

	memset( worldInfo, 0x00, len );

	m_gravity.serialize( worldInfo.m_gravity );
	worldInfo.m_solverInfo.m_tau = getSolverInfo().m_tau;
	worldInfo.m_solverInfo.m_damping = getSolverInfo().m_damping;
	worldInfo.m_solverInfo.m_friction = getSolverInfo().m_friction;
	worldInfo.m_solverInfo.m_timeStep = getSolverInfo().m_timeStep;

	worldInfo.m_solverInfo.m_restitution = getSolverInfo().m_restitution;
	worldInfo.m_solverInfo.m_maxErrorReduction = getSolverInfo().m_maxErrorReduction;
	worldInfo.m_solverInfo.m_sor = getSolverInfo().m_sor;
	worldInfo.m_solverInfo.m_erp = getSolverInfo().m_erp;

	worldInfo.m_solverInfo.m_erp2 = getSolverInfo().m_erp2;
	worldInfo.m_solverInfo.m_globalCfm = getSolverInfo().m_globalCfm;
	worldInfo.m_solverInfo.m_splitImpulsePenetrationThreshold = getSolverInfo().m_splitImpulsePenetrationThreshold;
	worldInfo.m_solverInfo.m_splitImpulseTurnErp = getSolverInfo().m_splitImpulseTurnErp;

	worldInfo.m_solverInfo.m_linearSlop = getSolverInfo().m_linearSlop;
	worldInfo.m_solverInfo.m_warmstartingFactor = getSolverInfo().m_warmstartingFactor;
	worldInfo.m_solverInfo.m_maxGyroscopicForce = getSolverInfo().m_maxGyroscopicForce;
	worldInfo.m_solverInfo.m_singleAxisRollingFrictionThreshold = getSolverInfo().m_singleAxisRollingFrictionThreshold;

	worldInfo.m_solverInfo.m_numIterations = getSolverInfo().m_numIterations;
	worldInfo.m_solverInfo.m_solverMode = getSolverInfo().m_solverMode;
	worldInfo.m_solverInfo.m_restingContactRestitutionThreshold = getSolverInfo().m_restingContactRestitutionThreshold;
	worldInfo.m_solverInfo.m_minimumSolverBatchSize = getSolverInfo().m_minimumSolverBatchSize;

	worldInfo.m_solverInfo.m_splitImpulse = getSolverInfo().m_splitImpulse;

# ifdef BT_USE_DOUBLE_PRECISION
	string structType = "btDynamicsWorldDoubleData";
#else//BT_USE_DOUBLE_PRECISION
	string structType = "btDynamicsWorldFloatData";
#endif//BT_USE_DOUBLE_PRECISION
	serializer.finalizeChunk( chunk, structType, BT_DYNAMICSWORLD_CODE, worldInfo );
}

void btDiscreteDynamicsWorld::serialize( btSerializer* serializer )
{

	serializer.startSerialization();

	serializeDynamicsWorldInfo( serializer );

	serializeCollisionObjects( serializer );

	serializeRigidBodies( serializer );

	serializer.finishSerialization();
}
}

}