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

using Bullet.Collision.BroadPhase;
using Bullet.Collision.Dispatch;
using Bullet.Dynamics.ConstraintSolver;
using Bullet.LinearMath;

namespace Bullet.Dynamics
{


	//class btTypedConstraint;
	//class btActionInterface;
	//class btConstraintSolver;
	//class btDynamicsWorld;


	/// Type for the callback for each tick
	public delegate void btInternalTickCallback( btDynamicsWorld world, double timeStep );

	public enum btDynamicsWorldType
	{
		BT_SIMPLE_DYNAMICS_WORLD = 1,
		BT_DISCRETE_DYNAMICS_WORLD = 2,
		BT_CONTINUOUS_DYNAMICS_WORLD = 3,
		BT_SOFT_RIGID_DYNAMICS_WORLD = 4,
		BT_GPU_DYNAMICS_WORLD = 5
	};

	///The btDynamicsWorld is the interface class for several dynamics implementation, basic, discrete, parallel, and continuous etc.
	public abstract class btDynamicsWorld : btCollisionWorld
	{
		protected btInternalTickCallback m_internalTickCallback;
		protected btInternalTickCallback m_internalPreTickCallback;
		protected object m_worldUserInfo;
		protected btContactSolverInfo m_solverInfo = new btContactSolverInfo();

		internal btDynamicsWorld()
		{
		}

		internal btDynamicsWorld( btDispatcher dispatcher, btBroadphaseInterface broadphase
						, btCollisionConfiguration collisionConfiguration )
		//: base( dispatcher, broadphase, collisionConfiguration )
		{
			Initialize( dispatcher, broadphase, collisionConfiguration );
		}

		new internal void Initialize( btDispatcher dispatcher, btBroadphaseInterface broadphase
								, btCollisionConfiguration collisionConfiguration )

		{
			base.Initialize( dispatcher, broadphase, collisionConfiguration );
			m_internalTickCallback = null;
			m_internalPreTickCallback = null;
			m_worldUserInfo = ( 0 );
		}


		///stepSimulation proceeds the simulation over 'timeStep', units in preferably in seconds.
		///By default, Bullet will subdivide the timestep in constant substeps of each 'fixedTimeStep'.
		///in order to keep the simulation real-time, the maximum number of substeps can be clamped to 'maxSubSteps'.
		///You can disable subdividing the timestep/substepping by passing maxSubSteps=0 as second argument to stepSimulation, but in that case you have to keep the timeStep constant.
		public abstract int stepSimulation( double timeStep, int maxSubSteps/* = 1*/, double fixedTimeStep/*=btScalar.BT_ONE/(double)(60)*/);


		public virtual void addConstraint( btTypedConstraint constraint, bool disableCollisionsBetweenLinkedBodies = false )
		{
			//(void)constraint; (void)disableCollisionsBetweenLinkedBodies;
		}

		public virtual void removeConstraint( btTypedConstraint constraint ) { }


		//once a rigidbody is added to the dynamics world, it will get this gravity assigned
		//existing rigidbodies in the world get gravity assigned too, during this method

		public virtual int getNumConstraints() { return 0; }

		public virtual btTypedConstraint getConstraint( int index ) { return null; }

		//public abstract btDynamicsWorldType getWorldType();

		//public abstract void clearForces();

		/// Set the callback for when an internal tick (simulation substep) happens, optional user info
		public void setInternalTickCallback( btInternalTickCallback cb, object worldUserInfo = null, bool isPreTick = false )
		{
			if( isPreTick )
			{
				m_internalPreTickCallback = cb;
			}
			else
			{
				m_internalTickCallback = cb;
			}
			m_worldUserInfo = worldUserInfo;
		}

		public void setWorldUserInfo( object worldUserInfo )
		{
			m_worldUserInfo = worldUserInfo;
		}

		public object getWorldUserInfo()
		{
			return m_worldUserInfo;
		}

		public btContactSolverInfo getSolverInfo()
		{
			return m_solverInfo;
		}

	};

#if SERIALIZE_DONE
	///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
	struct btDynamicsWorldDoubleData
{
	btContactSolverInfoDoubleData	m_solverInfo;
	btVector3DoubleData	m_gravity;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btDynamicsWorldFloatData
{
	btContactSolverInfoFloatData	m_solverInfo;
	btVector3FloatData	m_gravity;
};
#endif

}

