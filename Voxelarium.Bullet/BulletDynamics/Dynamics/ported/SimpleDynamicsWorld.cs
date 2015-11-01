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

using Bullet.BulletCollision;
using Bullet.Collision.BroadPhase;
using Bullet.Collision.Dispatch;
using Bullet.Dynamics.ConstraintSolver;
using Bullet.LinearMath;

namespace Bullet.Dynamics
{



	public class btSimpleDynamicsWorld : btDynamicsWorld
	{


		protected btConstraintSolver m_constraintSolver;

		protected bool m_ownsConstraintSolver;

		protected btVector3 m_gravity;


		public virtual btDynamicsWorldType getWorldType()
		{
			return btDynamicsWorldType.BT_SIMPLE_DYNAMICS_WORLD;
		}

		public btSimpleDynamicsWorld()
		{
		}
		///this btSimpleDynamicsWorld constructor creates dispatcher, broadphase pairCache and constraintSolver
		public btSimpleDynamicsWorld(btDispatcher dispatcher
				, btBroadphaseInterface pairCache
				,btConstraintSolver constraintSolver
				, btCollisionConfiguration collisionConfiguration)
			: base( dispatcher, pairCache, collisionConfiguration )
		{
			m_constraintSolver = ( constraintSolver );
			m_ownsConstraintSolver = ( false );
			m_gravity = new btVector3( 0, 0, -9.8 );

		}


		///maxSubSteps/fixedTimeStep for interpolation is currently ignored for btSimpleDynamicsWorld, use btDiscreteDynamicsWorld instead
		int stepSimulation( double timeStep, int maxSubSteps = 1, double fixedTimeStep = btScalar.BT_ONE_OVER_SIXTY )
		{
			//(void)fixedTimeStep;
			//(void)maxSubSteps;


			///apply gravity, predict motion
			predictUnconstraintMotion( timeStep );

			btDispatcherInfo  dispatchInfo = getDispatchInfo();
			dispatchInfo.m_timeStep = timeStep;
			dispatchInfo.m_stepCount = 0;
			dispatchInfo.m_debugDraw = getDebugDrawer();

			///perform collision detection
			performDiscreteCollisionDetection();

			///solve contact constraints
			int numManifolds = m_dispatcher1.getNumManifolds();
			if( numManifolds )
			{
				btPersistentManifold[] manifoldPtr = ( (btCollisionDispatcher*)m_dispatcher1 ).getInternalManifoldPointer();

				btContactSolverInfo infoGlobal;
				infoGlobal.m_timeStep = timeStep;
				m_constraintSolver.prepareSolve( 0, numManifolds );
				m_constraintSolver.solveGroup( &getCollisionObjectArray(), getNumCollisionObjects(), manifoldPtr, numManifolds, 0, 0, infoGlobal, m_debugDrawer, m_dispatcher1 );
				m_constraintSolver.allSolved( infoGlobal, m_debugDrawer );
			}

			///integrate transforms
			integrateTransforms( timeStep );

			updateAabbs();

			synchronizeMotionStates();

			clearForces();

			return 1;

		}

		public virtual void clearForces()
		{
			///@todo: iterate over awake simulation islands!
			for( int i = 0; i < m_collisionObjects.Count; i++ )
			{
				btCollisionObject colObj = m_collisionObjects[i];

				btRigidBody body = colObj as btRigidBody;
				if( body != null )
				{
					body.clearForces();
				}
			}
		}


		public virtual void setGravity( ref btVector3 gravity )
		{
			m_gravity = gravity;
			for( int i = 0; i < m_collisionObjects.Count; i++ )
			{
				btCollisionObject colObj = m_collisionObjects[i];
				btRigidBody body =  colObj as btRigidBody;
				if( body != null )
				{
					body.setGravity( ref gravity );
				}
			}
		}

		public virtual void getGravity( out btVector3 result )
		{
			result = m_gravity;
		}

		public virtual void removeRigidBody( btRigidBody body )
		{
			removeCollisionObject( body );
		}

		///removeCollisionObject will first check if it is a rigid body, if so call removeRigidBody otherwise call btCollisionWorld::removeCollisionObject
		void removeCollisionObject( btCollisionObject collisionObject )
		{
			btRigidBody body = collisionObject as btRigidBody;
			if( body != null )
				removeRigidBody( body );
			else
				btCollisionWorld::removeCollisionObject( collisionObject );
		}


		public virtual void addRigidBody( btRigidBody body )
		{
			body.setGravity(ref  m_gravity );

			if( body.getCollisionShape() )
			{
				addCollisionObject( body );
			}
		}

		public virtual void addRigidBody( btRigidBody body, short group, short mask )
		{
			body.setGravity( ref m_gravity );

			if( body.getCollisionShape() )
			{
				addCollisionObject( body, group, mask );
			}
		}


		public virtual void debugDrawWorld()
		{

		}

		public virtual void addAction( btActionInterface* action )
		{

		}

		public virtual void removeAction( btActionInterface* action )
		{

		}


		public virtual void updateAabbs()
		{
			btTransform predictedTrans;
			for( int i = 0; i < m_collisionObjects.Count; i++ )
			{
				btCollisionObject colObj = m_collisionObjects[i];
				btRigidBody body = btRigidBody::upcast( colObj );
				if( body )
				{
					if( body.isActive() && ( !body.isStaticObject() ) )
					{
						btVector3 minAabb, maxAabb;
						colObj.getCollisionShape().getAabb( colObj.getWorldTransform(), minAabb, maxAabb );
						btBroadphaseInterface* bp = getBroadphase();
						bp.setAabb( body.getBroadphaseHandle(), minAabb, maxAabb, m_dispatcher1 );
					}
				}
			}
		}

		protected void integrateTransforms( double timeStep )
		{
			btTransform predictedTrans;
			for( int i = 0; i < m_collisionObjects.Count; i++ )
			{
				btCollisionObject colObj = m_collisionObjects[i];
				btRigidBody body = colObj as btRigidBody;
				if( body != null )
				{
					if( body.isActive() && ( !body.isStaticObject() ) )
					{
						body.predictIntegratedTransform( timeStep, predictedTrans );
						body.proceedToTransform( predictedTrans );
					}
				}
			}
		}



		protected void predictUnconstraintMotion( double timeStep )
		{
			for( int i = 0; i < m_collisionObjects.Count; i++ )
			{
				btCollisionObject colObj = m_collisionObjects[i];
				btRigidBody body = btRigidBody::upcast( colObj );
				if( body )
				{
					if( !body.isStaticObject() )
					{
						if( body.isActive() )
						{
							body.applyGravity();
							body.integrateVelocities( timeStep );
							body.applyDamping( timeStep );
							body.predictIntegratedTransform( timeStep, body.getInterpolationWorldTransform() );
						}
					}
				}
			}
		}


		public virtual void synchronizeMotionStates()
		{
			///@todo: iterate over awake simulation islands!
			for( int i = 0; i < m_collisionObjects.Count; i++ )
			{
				btCollisionObject colObj = m_collisionObjects[i];
				btRigidBody body = btRigidBody::upcast( colObj );
				if( body && body.getMotionState() )
				{
					if( body.getActivationState() != ISLAND_SLEEPING )
					{
						body.getMotionState().setWorldTransform( body.getWorldTransform() );
					}
				}
			}

		}


		public virtual void setConstraintSolver( btConstraintSolver* solver )
		{
			if( m_ownsConstraintSolver )
			{
				btAlignedFree( m_constraintSolver );
			}
			m_ownsConstraintSolver = false;
			m_constraintSolver = solver;
		}

		public virtual btConstraintSolver getConstraintSolver()
		{
			return m_constraintSolver;
		}

	}
}