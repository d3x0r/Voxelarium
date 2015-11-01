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
using Bullet.Collision.BroadPhase;
using Bullet.Collision.Dispatch;
using Bullet.Collision.NarrowPhase;
using Bullet.Dynamics.ConstraintSolver;
using Bullet.LinearMath;
using Bullet.Types;
using Bullet.Collision.Shapes;

namespace Bullet.Dynamics
{
	public partial class btDiscreteDynamicsWorld
	{

		internal class InplaceSolverIslandCallback : btSimulationIslandManager.IslandCallback
		{
			btContactSolverInfo m_solverInfo;
			internal btConstraintSolver m_solver;
			btTypedConstraint[] m_sortedConstraints;
			int m_numConstraints;
			btIDebugDraw m_debugDrawer;
			btDispatcher m_dispatcher;

			btList<btCollisionObject> m_bodies;
			btList<btPersistentManifold> m_manifolds;
			btList<btTypedConstraint> m_constraints;

			internal InplaceSolverIslandCallback(
				btConstraintSolver solver,
				btDispatcher dispatcher )
			{
				m_solverInfo = null;
				m_solver = ( solver );
				m_sortedConstraints = null;
				m_numConstraints = ( 0 );
				m_debugDrawer = null;
				m_dispatcher = ( dispatcher );

			}

			InplaceSolverIslandCallback( InplaceSolverIslandCallback other )
			{
				Debug.Assert( false );
				//(void)other;
				//return *this;

			}

			public void setup( btContactSolverInfo solverInfo
				, btTypedConstraint[] sortedConstraints, int numConstraints
				, btIDebugDraw debugDrawer )
			{
				Debug.Assert( solverInfo != null );
				m_solverInfo = solverInfo;
				m_sortedConstraints = sortedConstraints;
				m_numConstraints = numConstraints;
				m_debugDrawer = debugDrawer;
				m_bodies.Count = ( 0 );
				m_manifolds.Count = ( 0 );
				m_constraints.Count = ( 0 );
			}


			internal override void processIsland( btCollisionObject[] bodies, int numBodies, btPersistentManifold[] manifolds, int first_manifold, int numManifolds, int islandId )
			{
				if( islandId < 0 )
				{
					///we don't split islands, so all constraints/contact manifolds/bodies are passed into the solver regardless the island id
					m_solver.solveGroup( bodies, numBodies, manifolds, first_manifold, numManifolds, m_sortedConstraints, 0, m_numConstraints, m_solverInfo, m_debugDrawer, m_dispatcher );
				}
				else
				{
					//also add all non-contact constraints/joints for this island
					int startConstraint = 0;
					int numCurConstraints = 0;
					int i;

					//find the first constraint for this island
					for( i = 0; i < m_numConstraints; i++ )
					{
						if( btDiscreteDynamicsWorld.btGetConstraintIslandId( m_sortedConstraints[i] ) == islandId )
						{
							startConstraint = i;
							break;
						}
					}
					//count the number of constraints in this island
					for( ; i < m_numConstraints; i++ )
					{
						if( btDiscreteDynamicsWorld.btGetConstraintIslandId( m_sortedConstraints[i] ) == islandId )
						{
							numCurConstraints++;
						}
					}

					if( m_solverInfo.m_minimumSolverBatchSize <= 1 )
					{
						m_solver.solveGroup( bodies, numBodies, manifolds, first_manifold, numManifolds, m_sortedConstraints, startConstraint, numCurConstraints, m_solverInfo, m_debugDrawer, m_dispatcher );
					}
					else
					{

						for( i = 0; i < numBodies; i++ )
							m_bodies.Add( bodies[i] );
						for( i = 0; i < numManifolds; i++ )
							m_manifolds.Add( manifolds[first_manifold+i] );
						for( i = 0; i < numCurConstraints; i++ )
							m_constraints.Add( m_sortedConstraints[startConstraint + i] );
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
			internal void processConstraints()
			{

				btCollisionObject[] bodies = m_bodies.Count != 0 ? m_bodies.InternalArray : null;
				btPersistentManifold[] manifold = m_manifolds.Count != 0 ? m_manifolds.InternalArray : null;
				btTypedConstraint[] constraints = m_constraints.Count != 0 ? m_constraints.InternalArray : null;

				m_solver.solveGroup( bodies, m_bodies.Count, manifold, 0, m_manifolds.Count, constraints, 0, m_constraints.Count, m_solverInfo, m_debugDrawer, m_dispatcher );
				m_bodies.Count = ( 0 );
				m_manifolds.Count = ( 0 );
				m_constraints.Count = ( 0 );

			}

		};



		public static int btGetConstraintIslandId( btTypedConstraint lhs )
		{
			int islandId;

			btCollisionObject rcolObj0 = lhs.getRigidBodyA();
			btCollisionObject rcolObj1 = lhs.getRigidBodyB();
			islandId = rcolObj0.getIslandTag() >= 0 ? rcolObj0.getIslandTag() : rcolObj1.getIslandTag();
			return islandId;

		}


		internal btDiscreteDynamicsWorld( btDispatcher dispatcher, btBroadphaseInterface pairCache, btConstraintSolver constraintSolver, btCollisionConfiguration collisionConfiguration )
					: base( dispatcher, pairCache, collisionConfiguration )
		{
			m_solverIslandCallback = null;
			m_constraintSolver = ( constraintSolver );
			m_gravity = new btVector3( 0, -10, 0 );
			m_localTime = ( 0 );
			m_fixedTimeStep = ( 0 );
			m_synchronizeAllMotionStates = ( false );
			m_applySpeculativeContactRestitution = ( false );
			m_profileTimings = ( 0 );
			m_latencyMotionStateInterpolation = ( true );

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

			m_solverIslandCallback = new InplaceSolverIslandCallback( m_constraintSolver, dispatcher );
		}


		~btDiscreteDynamicsWorld()
		{
			//only delete it when we created it
			if( m_ownsIslandManager )
			{
				m_islandManager = null;
				//m_islandManager.~btSimulationIslandManager();
				//btAlignedFree( m_islandManager );
			}
			if( m_solverIslandCallback != null )
			{
				m_solverIslandCallback = null;
				//m_solverIslandCallback.~InplaceSolverIslandCallback();
				//btAlignedFree( m_solverIslandCallback );
			}
			if( m_ownsConstraintSolver )
			{
				//m_constraintSolver.~btConstraintSolver();
				//btAlignedFree( m_constraintSolver );
				m_constraintSolver = null;
			}
		}

		internal void saveKinematicState( double timeStep )
		{
			///would like to iterate over m_nonStaticRigidBodies, but unfortunately old API allows
			///to switch status _after_ adding kinematic objects to the world
			///fix it for Bullet 3.x release
			for( int i = 0; i < m_collisionObjects.Count; i++ )
			{
				btCollisionObject colObj = m_collisionObjects[i];
				btRigidBody body = btRigidBody.upcast( colObj );
				if( body && body.getActivationState() != ActivationState.ISLAND_SLEEPING )
				{
					if( body.isKinematicObject() )
					{
						//to calculate velocities next frame
						body.saveKinematicState( timeStep );
					}
				}
			}

		}

		public override void debugDrawWorld()
		{
			CProfileSample sample = new CProfileSample( "debugDrawWorld" );

			base.debugDrawWorld();

			bool drawConstraints = false;
			if( getDebugDrawer() != null )
			{
				btIDebugDraw.DebugDrawModes mode = getDebugDrawer().getDebugMode();
				if( ( mode & ( btIDebugDraw.DebugDrawModes.DBG_DrawConstraints | btIDebugDraw.DebugDrawModes.DBG_DrawConstraintLimits ) ) != 0 )
				{
					drawConstraints = true;
				}
			}
			if( drawConstraints )
			{
				for( int i = getNumConstraints() - 1; i >= 0; i-- )
				{
					btTypedConstraint constraint = getConstraint( i );
					debugDrawConstraint( constraint );
				}
			}



			if( getDebugDrawer() != null && 0 != ( getDebugDrawer().getDebugMode() & ( btIDebugDraw.DebugDrawModes.DBG_DrawWireframe | btIDebugDraw.DebugDrawModes.DBG_DrawAabb | btIDebugDraw.DebugDrawModes.DBG_DrawNormals ) ) )
			{
				int i;

				if( getDebugDrawer() != null && getDebugDrawer().getDebugMode() != 0 )
				{
					for( i = 0; i < m_actions.Count; i++ )
					{
						m_actions[i].debugDraw( m_debugDrawer );
					}
				}
			}
			if( getDebugDrawer() != null )
				getDebugDrawer().flushLines();

		}

		internal void clearForces()
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
		internal void applyGravity()
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


		internal void synchronizeSingleMotionState( btRigidBody body )
		{
			Debug.Assert( body );

			if( body.getMotionState() != null && !body.isStaticOrKinematicObject() )
			{
				//we need to call the update at least once, even for sleeping objects
				//otherwise the 'graphics' transform never updates properly
				///@todo: add 'dirty' flag
				//if (body.getActivationState() != ISLAND_SLEEPING)
				{
					btTransform interpolatedTransform;
					btTransformUtil.integrateTransform( body.getInterpolationWorldTransform(),
						body.getInterpolationLinearVelocity(), body.getInterpolationAngularVelocity(),
						( m_latencyMotionStateInterpolation && m_fixedTimeStep != 0 ) ? m_localTime - m_fixedTimeStep : m_localTime * body.getHitFraction(),
						out interpolatedTransform );
					body.getMotionState().setWorldTransform( ref interpolatedTransform );
				}
			}
		}


		internal void synchronizeMotionStates()
		{
			CProfileSample sample = new CProfileSample( "synchronizeMotionStates" );
			if( m_synchronizeAllMotionStates )
			{
				//iterate  over all collision objects
				for( int i = 0; i < m_collisionObjects.Count; i++ )
				{
					btCollisionObject colObj = m_collisionObjects[i];
					btRigidBody body = btRigidBody.upcast( colObj );
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


		///if maxSubSteps > 0, it will interpolate motion between fixedTimeStep's
		public override int stepSimulation( double timeStep, int maxSubSteps = 1, double fixedTimeStep = btScalar.BT_ONE_OVER_SIXTY )
		{
			startProfiling( timeStep );

			CProfileSample sample = new CProfileSample( "stepSimulation" );

			int numSimulationSubSteps = 0;

			if( maxSubSteps != 0 )
			{
				//fixed timestep with interpolation
				m_fixedTimeStep = fixedTimeStep;
				m_localTime += timeStep;
				if( m_localTime >= fixedTimeStep )
				{
					numSimulationSubSteps = (int)(m_localTime / fixedTimeStep);
					m_localTime -= numSimulationSubSteps * fixedTimeStep;
				}
			}
			else
			{
				//variable timestep
				fixedTimeStep = timeStep;
				m_localTime = m_latencyMotionStateInterpolation ? 0 : timeStep;
				m_fixedTimeStep = 0;
				if( btScalar.btFuzzyZero( timeStep ) )
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
			if( getDebugDrawer() != null )
			{
				btIDebugDraw debugDrawer = getDebugDrawer();
				btRigidBody.gDisableDeactivation = ( debugDrawer.getDebugMode() & btIDebugDraw.DebugDrawModes.DBG_NoDeactivation ) != 0;
			}
			if( numSimulationSubSteps != 0 )
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

# if !BT_NO_PROFILE
			CProfileManager.Increment_Frame_Counter();
#endif //BT_NO_PROFILE

			return numSimulationSubSteps;
		}

		internal void internalSingleStepSimulation( double timeStep )
		{

			CProfileSample sample = new CProfileSample( "internalSingleStepSimulation" );

			if( null != m_internalPreTickCallback )
			{
				m_internalPreTickCallback( this, timeStep );
			}

			///apply gravity, predict motion
			predictUnconstraintMotion( timeStep );

			btDispatcherInfo dispatchInfo = getDispatchInfo();

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

			if( null != m_internalTickCallback )
			{
				m_internalTickCallback( this, timeStep );
			}
		}

		internal void setGravity( ref btVector3 gravity )
		{
			m_gravity = gravity;
			for( int i = 0; i < m_nonStaticRigidBodies.Count; i++ )
			{
				btRigidBody body = m_nonStaticRigidBodies[i];
				if( body.isActive() && 0 == ( body.getFlags() &  btRigidBodyFlags.BT_DISABLE_WORLD_GRAVITY ) )
				{
					body.setGravity( ref gravity );
				}
			}
		}

		internal btVector3 getGravity()
		{
			return m_gravity;
		}

		public override void addCollisionObject( btCollisionObject collisionObject, btBroadphaseProxy.CollisionFilterGroups collisionFilterGroup = btBroadphaseProxy.CollisionFilterGroups.StaticFilter
			, btBroadphaseProxy.CollisionFilterGroups collisionFilterMask = btBroadphaseProxy.CollisionFilterGroups.StaticFilter )
		{
			base.addCollisionObject( collisionObject, collisionFilterGroup, collisionFilterMask );
		}

		public override void removeCollisionObject( btCollisionObject collisionObject )
		{
			btRigidBody body = btRigidBody.upcast( collisionObject );
			if( body )
				removeRigidBody( body );
			else
				base.removeCollisionObject( collisionObject );
		}

		internal void removeRigidBody( btRigidBody body )
		{
			m_nonStaticRigidBodies.Remove( body );
			base.removeCollisionObject( body );
		}


		internal void addRigidBody( btRigidBody body )
		{
			if( !body.isStaticOrKinematicObject() && ( body.getFlags() & btRigidBodyFlags.BT_DISABLE_WORLD_GRAVITY ) == 0 )
			{
				body.setGravity( ref m_gravity );
			}

			if( body.getCollisionShape() != null )
			{
				if( !body.isStaticObject() )
				{
					m_nonStaticRigidBodies.Add( body );
				}
				else
				{
					body.setActivationState( ActivationState.ISLAND_SLEEPING );
				}

				bool isDynamic = !( body.isStaticObject() || body.isKinematicObject() );
				btBroadphaseProxy.CollisionFilterGroups collisionFilterGroup = ( isDynamic ? btBroadphaseProxy.CollisionFilterGroups.DefaultFilter : btBroadphaseProxy.CollisionFilterGroups.StaticFilter );
				btBroadphaseProxy.CollisionFilterGroups collisionFilterMask = ( isDynamic ? btBroadphaseProxy.CollisionFilterGroups.AllFilter : ( btBroadphaseProxy.CollisionFilterGroups.AllFilter ^ btBroadphaseProxy.CollisionFilterGroups.StaticFilter ) );

				addCollisionObject( body, collisionFilterGroup, collisionFilterMask );
			}
		}

		internal void addRigidBody( btRigidBody body, btBroadphaseProxy.CollisionFilterGroups group, btBroadphaseProxy.CollisionFilterGroups mask )
		{
			if( !body.isStaticOrKinematicObject() && 0==( body.getFlags() &  btRigidBodyFlags.BT_DISABLE_WORLD_GRAVITY ) )
			{
				body.setGravity( ref m_gravity );
			}

			if( body.getCollisionShape() != null )
			{
				if( !body.isStaticObject() )
				{
					m_nonStaticRigidBodies.Add( body );
				}
				else
				{
					body.setActivationState(  ActivationState.ISLAND_SLEEPING );
				}
				addCollisionObject( body, group, mask );
			}
		}


		internal void updateActions( double timeStep )
		{
			CProfileSample sample = new CProfileSample( "updateActions" );

			for( int i = 0; i < m_actions.Count; i++ )
			{
				m_actions[i].updateAction( this, timeStep );
			}
		}


		internal void updateActivationState( double timeStep )
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
							body.setActivationState( ActivationState.ISLAND_SLEEPING );
						}
						else
						{
							if( body.getActivationState() == ActivationState.ACTIVE_TAG )
								body.setActivationState( ActivationState.WANTS_DEACTIVATION );
							if( body.getActivationState() == ActivationState.ISLAND_SLEEPING )
							{
								body.setAngularVelocity( ref btVector3.Zero );
								body.setLinearVelocity( ref btVector3.Zero );
							}

						}
					}
					else
					{
						if( body.getActivationState() != ActivationState.DISABLE_DEACTIVATION )
							body.setActivationState( ActivationState.ACTIVE_TAG );
					}
				}
			}
		}

		public override void addConstraint( btTypedConstraint constraint, bool disableCollisionsBetweenLinkedBodies = false )
		{
			m_constraints.Add( constraint );
			//Make sure the two bodies of a type constraint are different (possibly add this to the btTypedConstraint constructor?)
			Debug.Assert( constraint.getRigidBodyA() != constraint.getRigidBodyB() );

			if( disableCollisionsBetweenLinkedBodies )
			{
				constraint.getRigidBodyA().addConstraintRef( constraint );
				constraint.getRigidBodyB().addConstraintRef( constraint );
			}
		}

		public override void removeConstraint( btTypedConstraint constraint )
		{
			m_constraints.Remove( constraint );
			constraint.getRigidBodyA().removeConstraintRef( constraint );
			constraint.getRigidBodyB().removeConstraintRef( constraint );
		}

		internal void addAction( btActionInterface action )
		{
			m_actions.Add( action );
		}

		internal void removeAction( btActionInterface action )
		{
			m_actions.Remove( action );
		}



		static bool compare( btTypedConstraint lhs, btTypedConstraint rhs )
		{
			int rIslandId0, lIslandId0;
			rIslandId0 = btDiscreteDynamicsWorld.btGetConstraintIslandId( rhs );
			lIslandId0 = btDiscreteDynamicsWorld.btGetConstraintIslandId( lhs );
			return lIslandId0 < rIslandId0;
		}


		internal void solveConstraints( btContactSolverInfo solverInfo )
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

			m_sortedConstraints.quickSort( compare );

			btTypedConstraint[] constraintsPtr = ( getNumConstraints() != 0 ) ? m_sortedConstraints.InternalArray : null;

			m_solverIslandCallback.setup( solverInfo, constraintsPtr, m_sortedConstraints.Count, getDebugDrawer() );
			m_constraintSolver.prepareSolve( getNumCollisionObjects(), getDispatcher().getNumManifolds() );

			/// solve all the constraints for this island
			m_islandManager.buildAndProcessIslands( getDispatcher(), this, m_solverIslandCallback );

			m_solverIslandCallback.processConstraints();

			m_constraintSolver.allSolved( solverInfo, m_debugDrawer );
		}


		internal void calculateSimulationIslands()
		{
			CProfileSample sample = new CProfileSample( "calculateSimulationIslands" );

			getSimulationIslandManager().updateActivationState( this, getDispatcher() );

			{
				//merge islands based on speculative contact manifolds too
				for( int i = 0; i < this.m_predictiveManifolds.Count; i++ )
				{
					btPersistentManifold manifold = m_predictiveManifolds[i];

					btCollisionObject colObj0 = manifold.getBody0();
					btCollisionObject colObj1 = manifold.getBody1();

					if( ( ( colObj0 != null ) && ( !( colObj0 ).isStaticOrKinematicObject() ) ) &&
						( ( colObj1 != null ) && ( !( colObj1 ).isStaticOrKinematicObject() ) ) )
					{
						getSimulationIslandManager().getUnionFind().unite( ( colObj0 ).getIslandTag(), ( colObj1 ).getIslandTag() );
					}
				}
			}

			{
				int i;
				int numConstraints = m_constraints.Count;
				for( i = 0; i < numConstraints; i++ )
				{
					btTypedConstraint constraint = m_constraints[i];
					if( constraint.isEnabled() )
					{
						btRigidBody colObj0 = constraint.getRigidBodyA();
						btRigidBody colObj1 = constraint.getRigidBodyB();

						if( ( ( colObj0 != null ) && ( !( colObj0 ).isStaticOrKinematicObject() ) ) &&
							( ( colObj1 != null ) && ( !( colObj1 ).isStaticOrKinematicObject() ) ) )
						{
							getSimulationIslandManager().getUnionFind().unite( ( colObj0 ).getIslandTag(), ( colObj1 ).getIslandTag() );
						}
					}
				}
			}

			//Store the island id in each body
			getSimulationIslandManager().storeIslandActivationState( this );


		}



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

#endif

		///internal debugging variable. this value shouldn't be too high
		int gNumClampedCcdMotions = 0;


		internal void createPredictiveContacts( double timeStep )
		{
			CProfileSample sample = new CProfileSample( "createPredictiveContacts" );

			{
				CProfileSample sub_sample = new CProfileSample( "release predictive contact manifolds" );

				for( int i = 0; i < m_predictiveManifolds.Count; i++ )
				{
					btPersistentManifold manifold = m_predictiveManifolds[i];
					this.m_dispatcher1.releaseManifold( manifold );
				}
				m_predictiveManifolds.Clear();
			}

			btTransform predictedTrans;
			for( int i = 0; i < m_nonStaticRigidBodies.Count; i++ )
			{
				btRigidBody body = m_nonStaticRigidBodies[i];
				body.setHitFraction( 1 );

				if( body.isActive() & ( !body.isStaticOrKinematicObject() ) )
				{

					body.predictIntegratedTransform( timeStep, out predictedTrans );

					double squareMotion = ( predictedTrans.m_origin - body.getWorldTransform().getOrigin() ).length2();

					if( getDispatchInfo().m_useContinuous && 0 != body.getCcdSquareMotionThreshold() && body.getCcdSquareMotionThreshold() < squareMotion )
					{
						CProfileSample subsample = new CProfileSample( "predictive convexSweepTest" );
						if( body.getCollisionShape().isConvex() )
						{
							gNumClampedCcdMotions++;
#if PREDICTIVE_CONTACT_USE_STATIC_ONLY
							StaticOnlyCallback sweepResults( body, body.getWorldTransform().getOrigin(),predictedTrans.getOrigin(),getBroadphase().getOverlappingPairCache(),getDispatcher());
#else
							btClosestNotMeConvexResultCallback sweepResults = BulletGlobals.ClosestNotMeConvexResultCallbackPool.Get();
							sweepResults.Initialize( body, ref body.m_worldTransform.m_origin
										 , ref predictedTrans.m_origin, getBroadphase().getOverlappingPairCache(), getDispatcher() );
#endif
							//btConvexShape* convexShape = static_cast<btConvexShape*>(body.getCollisionShape());
							btSphereShape tmpSphere = BulletGlobals.SphereShapePool.Get();
							tmpSphere.Initialize( body.getCcdSweptSphereRadius() );//btConvexShape* convexShape = static_cast<btConvexShape*>(body.getCollisionShape());
							sweepResults.m_allowedPenetration = getDispatchInfo().m_allowedCcdPenetration;

							sweepResults.m_collisionFilterGroup = body.getBroadphaseProxy().m_collisionFilterGroup;
							sweepResults.m_collisionFilterMask = body.getBroadphaseProxy().m_collisionFilterMask;
							btTransform modifiedPredictedTrans = predictedTrans;
							modifiedPredictedTrans.m_basis = body.m_worldTransform.m_basis;

							convexSweepTest( tmpSphere, ref body.m_worldTransform, ref modifiedPredictedTrans, sweepResults );
							if( sweepResults.hasHit() && ( sweepResults.m_closestHitFraction < 1 ) )
							{

								btVector3 distVec = ( predictedTrans.m_origin - body.getWorldTransform().getOrigin() ) * sweepResults.m_closestHitFraction;
								btVector3 tmp;
								sweepResults.m_hitNormalWorld.Invert( out tmp );
                                double distance = distVec.dot( ref tmp );


								btPersistentManifold manifold = m_dispatcher1.getNewManifold( body, sweepResults.m_hitCollisionObject );
								m_predictiveManifolds.Add( manifold );

								btVector3 worldPointB; body.getWorldTransform().getOrigin().Add(ref distVec, out worldPointB );
								btTransform tmpT;
								sweepResults.m_hitCollisionObject.getWorldTransform().inverse( out tmpT );
                                btVector3 localPointB; tmpT.Apply( ref worldPointB, out localPointB );

								btManifoldPoint newPoint = BulletGlobals.ManifoldPointPool.Get();
								newPoint.Initialize( ref btVector3.Zero, ref localPointB, ref sweepResults.m_hitNormalWorld, distance );

								bool isPredictive = true;
								int index = manifold.addManifoldPoint( newPoint, isPredictive );
								btManifoldPoint pt = manifold.getContactPoint( index );
								pt.m_combinedRestitution = 0;
								pt.m_combinedFriction = btManifoldResult.calculateCombinedFriction( body, sweepResults.m_hitCollisionObject );
								body.getWorldTransform().getOrigin( out pt.m_positionWorldOnA );
								pt.m_positionWorldOnB = worldPointB;
								BulletGlobals.ManifoldPointPool.Free( newPoint );
							}
							BulletGlobals.SphereShapePool.Free( tmpSphere );
							BulletGlobals.ClosestNotMeConvexResultCallbackPool.Free( sweepResults );
						}
					}
				}
			}
		}





		internal void startProfiling( double timeStep )
		{
			//(void)timeStep;

#if !BT_NO_PROFILE
			CProfileManager.Reset();
#endif //BT_NO_PROFILE

		}






		internal void debugDrawConstraint( btTypedConstraint constraint )
		{
			bool drawFrames = ( getDebugDrawer().getDebugMode() & btIDebugDraw.DebugDrawModes.DBG_DrawConstraints ) != 0;
			bool drawLimits = ( getDebugDrawer().getDebugMode() & btIDebugDraw.DebugDrawModes.DBG_DrawConstraintLimits ) != 0;
			double dbgDrawSize = constraint.getDbgDrawSize();
			if( dbgDrawSize <= (double)( 0 ) )
			{
				return;
			}

			switch( constraint.getConstraintType() )
			{
				case  btObjectTypes.POINT2POINT_CONSTRAINT_TYPE:
					{
						btPoint2PointConstraint p2pC = (btPoint2PointConstraint)constraint;
						btTransform tr = btTransform.Identity;
						btVector3 pivot;
						p2pC.getPivotInA( out pivot );
						btVector3 tmp;
						p2pC.getRigidBodyA().getCenterOfMassTransform().Apply(ref pivot, out tmp );
						tr.setOrigin( ref tmp );
						getDebugDrawer().drawTransform( ref tr, dbgDrawSize );
						// that ideally should draw the same frame
						p2pC.getPivotInB(out pivot);
						p2pC.getRigidBodyB().getCenterOfMassTransform().Apply( ref pivot, out tmp );
						tr.setOrigin( tmp );
						if( drawFrames ) getDebugDrawer().drawTransform( ref tr, dbgDrawSize );
					}
					break;
				case btObjectTypes.HINGE_CONSTRAINT_TYPE:
					{
						btHingeConstraint pHinge = (btHingeConstraint)constraint;
						btTransform tr; pHinge.getRigidBodyA().getCenterOfMassTransform().Apply( pHinge.getAFrame(), out tr );
						if( drawFrames ) getDebugDrawer().drawTransform( ref tr, dbgDrawSize );
						pHinge.getRigidBodyB().getCenterOfMassTransform().Apply( pHinge.getBFrame(), out tr );
						if( drawFrames ) getDebugDrawer().drawTransform( ref tr, dbgDrawSize );
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
							maxAng = btScalar.SIMD_2_PI;
							drawSect = false;
						}
						if( drawLimits )
						{
							btIVector3 center = tr.getOrigin();
							btVector3 normal = tr.getBasis().getColumn( 2 );
							btVector3 axis = tr.getBasis().getColumn( 0 );
							getDebugDrawer().drawArc( center, ref normal, ref axis, dbgDrawSize, dbgDrawSize, minAng, maxAng,ref btVector3.Zero, drawSect );
						}
					}
					break;
				case btObjectTypes.CONETWIST_CONSTRAINT_TYPE:
					{
						btConeTwistConstraint pCT = (btConeTwistConstraint)constraint;
						btTransform tr; pCT.getRigidBodyA().getCenterOfMassTransform().Apply( ref pCT.m_rbAFrame, out tr );
						if( drawFrames ) getDebugDrawer().drawTransform( ref tr, dbgDrawSize );
						pCT.m_rbB.m_worldTransform.Apply( ref pCT.m_rbBFrame, out tr );
						if( drawFrames ) getDebugDrawer().drawTransform( ref tr, dbgDrawSize );
						if( drawLimits )
						{
							//double length = (double)(5);
							double length = dbgDrawSize;
							int nSegments = 8 * 4;
							double fAngleInRadians = btScalar.SIMD_2_PI * (double)( nSegments - 1 ) / (double)( nSegments );
							btVector3 pPrev; pCT.GetPointForAngle( fAngleInRadians, length, out pPrev );
							pPrev = tr * pPrev;
							for( int i = 0; i < nSegments; i++ )
							{
								fAngleInRadians = btScalar.SIMD_2_PI * (double)i / (double)( nSegments );
								btVector3 pCur; pCT.GetPointForAngle( fAngleInRadians, length, out pCur );
								btVector3 tmp;
								tr.Apply( ref pCur, out tmp );
								getDebugDrawer().drawLine( ref pPrev, ref tmp, ref btVector3.Zero );

								if( i % ( nSegments / 8 ) == 0 )
									getDebugDrawer().drawLine( ref tr.m_origin, ref pCur, ref btVector3.Zero );

								pPrev = pCur;
							}
							double tws = pCT.getTwistSpan();
							double twa = pCT.getTwistAngle();
							bool useFrameB = ( pCT.getRigidBodyB().getInvMass() > (double)( 0 ) );
							if( useFrameB )
							{
								pCT.getRigidBodyB().getCenterOfMassTransform().Apply( ref pCT.m_rbBFrame, out tr );
							}
							else
							{
								pCT.getRigidBodyA().getCenterOfMassTransform().Apply( ref pCT.m_rbBFrame, out tr );
							}
							btVector3 pivot = tr.m_origin;
							btVector3 normal; tr.getBasis().getColumn( 0, out normal );
							btVector3 axis1; tr.getBasis().getColumn( 1, out axis1 );
							getDebugDrawer().drawArc( ref pivot, ref normal, ref axis1, dbgDrawSize, dbgDrawSize, -twa - tws, -twa + tws, ref btVector3.Zero, true );
						}
					}
					break;
				case btObjectTypes.D6_SPRING_CONSTRAINT_TYPE:
				case btObjectTypes.D6_CONSTRAINT_TYPE:
					{
						btGeneric6DofConstraint p6DOF = (btGeneric6DofConstraint)constraint;
						btITransform tr;// = p6DOF.getCalculatedTransformA();
						if( drawFrames ) getDebugDrawer().drawTransform( ref p6DOF.m_calculatedTransformA, dbgDrawSize );
						//tr = p6DOF.getCalculatedTransformB();
						if( drawFrames ) getDebugDrawer().drawTransform( ref p6DOF.m_calculatedTransformB, dbgDrawSize );
						if( drawLimits )
						{
							tr = p6DOF.getCalculatedTransformA();
							btIVector3 center = p6DOF.getCalculatedTransformB().getOrigin();
							btVector3 up = tr.getBasis().getColumn( 2 );
							btVector3 axis = tr.getBasis().getColumn( 0 );
							double minTh = p6DOF.getRotationalLimitMotor( 1 ).m_loLimit;
							double maxTh = p6DOF.getRotationalLimitMotor( 1 ).m_hiLimit;
							double minPs = p6DOF.getRotationalLimitMotor( 2 ).m_loLimit;
							double maxPs = p6DOF.getRotationalLimitMotor( 2 ).m_hiLimit;
							getDebugDrawer().drawSpherePatch( center, ref up, ref axis, dbgDrawSize * (double)( .9f ), minTh, maxTh, minPs, maxPs, ref btVector3.Zero );
							axis = tr.getBasis().getColumn( 1 );
							double ay = p6DOF.getAngle( 1 );
							double az = p6DOF.getAngle( 2 );
							double cy = btScalar.btCos( ay );
							double sy = btScalar.btSin( ay );
							double cz = btScalar.btCos( az );
							double sz = btScalar.btSin( az );
							btVector3 ref_point;
							ref_point.x = cy * cz * axis[0] + cy * sz * axis[1] - sy * axis[2];
							ref_point.y = -sz * axis[0] + cz * axis[1];
							ref_point.z = cz * sy * axis[0] + sz * sy * axis[1] + cy * axis[2];
							ref_point.w = 0;
							tr = p6DOF.getCalculatedTransformB();
							btVector3 normal = -tr.getBasis().getColumn( 0 );
							double minFi = p6DOF.getRotationalLimitMotor( 0 ).m_loLimit;
							double maxFi = p6DOF.getRotationalLimitMotor( 0 ).m_hiLimit;
							if( minFi > maxFi )
							{
								getDebugDrawer().drawArc( center, ref normal, ref ref_point, dbgDrawSize, dbgDrawSize, -btScalar.SIMD_PI, btScalar.SIMD_PI, ref btVector3.Zero, false );
							}
							else if( minFi < maxFi )
							{
								getDebugDrawer().drawArc( center, ref normal, ref ref_point, dbgDrawSize, dbgDrawSize, minFi, maxFi, ref btVector3.Zero, true );
							}
							tr = p6DOF.getCalculatedTransformA();
							btVector3 bbMin = p6DOF.getTranslationalLimitMotor().m_lowerLimit;
							btVector3 bbMax = p6DOF.getTranslationalLimitMotor().m_upperLimit;
							getDebugDrawer().drawBox( ref bbMin, ref bbMax, tr, ref btVector3.Zero );
						}
					}
					break;
				///note: the code for D6_SPRING_2_CONSTRAINT_TYPE is identical to D6_CONSTRAINT_TYPE, the D6_CONSTRAINT_TYPE+D6_SPRING_CONSTRAINT_TYPE will likely become obsolete/deprecated at some stage
				case btObjectTypes.D6_SPRING_2_CONSTRAINT_TYPE:
					{
						{
							btGeneric6DofSpring2Constraint p6DOF = (btGeneric6DofSpring2Constraint)constraint;
							btITransform tr = p6DOF.getCalculatedTransformA();
							if( drawFrames ) getDebugDrawer().drawTransform( tr, dbgDrawSize );
							tr = p6DOF.getCalculatedTransformB();
							if( drawFrames ) getDebugDrawer().drawTransform( tr, dbgDrawSize );
							if( drawLimits )
							{
								tr = p6DOF.getCalculatedTransformA();
								btIVector3 center = p6DOF.getCalculatedTransformB().getOrigin();
								btVector3 up = tr.getBasis().getColumn( 2 );
								btVector3 axis = tr.getBasis().getColumn( 0 );
								double minTh = p6DOF.getRotationalLimitMotor( 1 ).m_loLimit;
								double maxTh = p6DOF.getRotationalLimitMotor( 1 ).m_hiLimit;
								double minPs = p6DOF.getRotationalLimitMotor( 2 ).m_loLimit;
								double maxPs = p6DOF.getRotationalLimitMotor( 2 ).m_hiLimit;
								getDebugDrawer().drawSpherePatch( center, ref up, ref axis, dbgDrawSize * (double)( .9f ), minTh, maxTh, minPs, maxPs, ref btVector3.Zero );
								axis = tr.getBasis().getColumn( 1 );
								double ay = p6DOF.getAngle( 1 );
								double az = p6DOF.getAngle( 2 );
								double cy = btScalar.btCos( ay );
								double sy = btScalar.btSin( ay );
								double cz = btScalar.btCos( az );
								double sz = btScalar.btSin( az );
								btVector3 ref_point ;
								ref_point.x = cy * cz * axis[0] + cy * sz * axis[1] - sy * axis[2];
								ref_point.y = -sz * axis[0] + cz * axis[1];
								ref_point.z = cz * sy * axis[0] + sz * sy * axis[1] + cy * axis[2];
								ref_point.w = 0;
								tr = p6DOF.getCalculatedTransformB();
								btVector3 normal = -tr.getBasis().getColumn( 0 );
								double minFi = p6DOF.getRotationalLimitMotor( 0 ).m_loLimit;
								double maxFi = p6DOF.getRotationalLimitMotor( 0 ).m_hiLimit;
								if( minFi > maxFi )
								{
									getDebugDrawer().drawArc( center, ref normal, ref ref_point, dbgDrawSize, dbgDrawSize, -btScalar.SIMD_PI, btScalar.SIMD_PI, ref btVector3.Zero, false );
								}
								else if( minFi < maxFi )
								{
									getDebugDrawer().drawArc( center, ref normal, ref ref_point, dbgDrawSize, dbgDrawSize, minFi, maxFi, ref btVector3.Zero, true );
								}
								tr = p6DOF.getCalculatedTransformA();
								btVector3 bbMin = p6DOF.getTranslationalLimitMotor().m_lowerLimit;
								btVector3 bbMax = p6DOF.getTranslationalLimitMotor().m_upperLimit;
								getDebugDrawer().drawBox( ref bbMin, ref bbMax, tr, ref btVector3.Zero );
							}
						}
						break;
					}
				case btObjectTypes.SLIDER_CONSTRAINT_TYPE:
					{
						btSliderConstraint pSlider = (btSliderConstraint)constraint;
						btITransform tr = pSlider.getCalculatedTransformA();
						if( drawFrames ) getDebugDrawer().drawTransform( tr, dbgDrawSize );
						tr = pSlider.getCalculatedTransformB();
						if( drawFrames ) getDebugDrawer().drawTransform( tr, dbgDrawSize );
						if( drawLimits )
						{
							tr = pSlider.getUseLinearReferenceFrameA() 
								? pSlider.getCalculatedTransformA() 
								: pSlider.getCalculatedTransformB();
							btVector3 tmp = new btVector3( pSlider.getLowerLinLimit(), 0, 0 );
                            btVector3 li_min; tr.Apply( ref tmp, out li_min );
							tmp = new btVector3( pSlider.getUpperLinLimit(), 0, 0 );
							btVector3 li_max; tr.Apply(  ref tmp, out li_max );
							getDebugDrawer().drawLine( ref li_min, ref li_max, ref btVector3.Zero );
							btVector3 normal = tr.getBasis().getColumn( 0 );
							btVector3 axis = tr.getBasis().getColumn( 1 );
							double a_min = pSlider.getLowerAngLimit();
							double a_max = pSlider.getUpperAngLimit();
							btIVector3 center = pSlider.getCalculatedTransformB().getOrigin();
							getDebugDrawer().drawArc( center, ref normal, ref axis, dbgDrawSize, dbgDrawSize, a_min, a_max, ref btVector3.Zero, true );
						}
					}
					break;
				default:
					break;
			}
			return;
		}





		internal void setConstraintSolver( btConstraintSolver solver )
		{
			if( m_ownsConstraintSolver )
			{
				m_constraintSolver = null;
			}
			m_ownsConstraintSolver = false;
			m_constraintSolver = solver;
			m_solverIslandCallback.m_solver = solver;
		}

		internal btConstraintSolver getConstraintSolver()
		{
			return m_constraintSolver;
		}


		public override int getNumConstraints()
		{
			return m_constraints.Count;
		}
		public override btTypedConstraint getConstraint( int index )
		{
			return m_constraints[index];
		}

#if SERILIZER_DONE

internal void serializeRigidBodies( btSerializer* serializer )
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
		btTypedConstraint constraint = m_constraints[i];
		int size = constraint.calculateSerializeBufferSize();
		btChunk* chunk = serializer.allocate( size, 1 );
		string structType = constraint.serialize( chunk.m_oldPtr, serializer );
		serializer.finalizeChunk( chunk, structType, BT_CONSTRAINT_CODE, constraint );
	}
}




internal void serializeDynamicsWorldInfo( btSerializer* serializer )
{
#if BT_USE_DOUBLE_PRECISION
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

#if BT_USE_DOUBLE_PRECISION
	string structType = "btDynamicsWorldDoubleData";
#else//BT_USE_DOUBLE_PRECISION
		string structType = "btDynamicsWorldFloatData";
#endif//BT_USE_DOUBLE_PRECISION
		serializer.finalizeChunk( chunk, structType, BT_DYNAMICSWORLD_CODE, worldInfo );
}

	internal void serialize( btSerializer* serializer )
	{

		serializer.startSerialization();

		serializeDynamicsWorldInfo( serializer );

		serializeCollisionObjects( serializer );

		serializeRigidBodies( serializer );

		serializer.finishSerialization();
	}

#endif

	}

}