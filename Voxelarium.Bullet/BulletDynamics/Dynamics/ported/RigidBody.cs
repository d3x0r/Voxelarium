//#define DISABLE_OPERATORS
#define DISABLE_BOOL_IMPLICIT_OPERATOR
//#define USE_OLD_DAMPING_METHOD
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

using Bullet.Types;
using Bullet.LinearMath;
using Bullet.Collision.Dispatch;
using Bullet.Dynamics.ConstraintSolver;
using Bullet.Collision.Shapes;
using System.Diagnostics;
using Bullet.Collision.BroadPhase;

namespace Bullet.Dynamics
{

	/*
#if BT_USE_DOUBLE_PRECISION
#define btRigidBodyData	btRigidBodyDoubleData
#define btRigidBodyDataName	"btRigidBodyDoubleData"
#else
#define btRigidBodyData	btRigidBodyFloatData
#define btRigidBodyDataName	"btRigidBodyFloatData"
#endif //BT_USE_DOUBLE_PRECISION
*/

	public enum btRigidBodyFlags
	{
		BT_DISABLE_WORLD_GRAVITY = 1,
		///BT_ENABLE_GYROPSCOPIC_FORCE flags is enabled by default in Bullet 2.83 and onwards.
		///and it BT_ENABLE_GYROPSCOPIC_FORCE becomes equivalent to BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY
		///See Demos/GyroscopicDemo and computeGyroscopicImpulseImplicit
		BT_ENABLE_GYROSCOPIC_FORCE_EXPLICIT = 2,
		BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD = 4,
		BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY = 8,
		BT_ENABLE_GYROPSCOPIC_FORCE = BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY,
	};


	///The btRigidBody is the main class for rigid body objects. It is derived from btCollisionObject, so it keeps a pointer to a btCollisionShape.
	///It is recommended for performance and memory use to share btCollisionShape objects whenever possible.
	///There are 3 types of rigid bodies: 
	///- A) Dynamic rigid bodies, with positive mass. Motion is controlled by rigid body dynamics.
	///- B) Fixed objects with zero mass. They are not moving (basically collision objects)
	///- C) Kinematic objects, which are objects without mass, but the user can move them. There is on-way interaction, and Bullet calculates a velocity based on the timestep and previous and current world transform.
	///Bullet automatically deactivates dynamic rigid bodies, when the velocity is below a threshold for a given time.
	///Deactivated (sleeping) rigid bodies don't take any processing time, except a minor broadphase collision detection impact (to allow active objects to activate/wake up sleeping objects)
	public class btRigidBody : btCollisionObject
	{
		public static implicit operator bool ( btRigidBody b ) { return b != null; }
		internal btMatrix3x3 m_invInertiaTensorWorld;
		internal btVector3 m_linearVelocity;
		internal btVector3 m_angularVelocity;
		internal double m_inverseMass;
		internal btVector3 m_linearFactor;

		internal btVector3 m_gravity;
		internal btVector3 m_gravity_acceleration;
		internal btVector3 m_invInertiaLocal;
		internal btVector3 m_totalForce;
		internal btVector3 m_totalTorque;

		double m_linearDamping;
		double m_angularDamping;

		bool m_additionalDamping;
		double m_additionalDampingFactor;
		double m_additionalLinearDampingThresholdSqr;
		double m_additionalAngularDampingThresholdSqr;
		double m_additionalAngularDampingFactor;

		//for experimental overriding of friction/contact solver func
		int m_contactSolverType;
		int m_frictionSolverType;

		double m_linearSleepingThreshold;
		double m_angularSleepingThreshold;

		//m_optionalMotionState allows to automatic synchronize the world transform for active objects
		btMotionState m_optionalMotionState;

		//keep track of typed constraints referencing this rigid body, to disable collision between linked bodies
		btList<btTypedConstraint> m_constraintRefs;

		btRigidBodyFlags m_rigidbodyFlags;

		int m_debugBodyId;


		protected btVector3 m_deltaLinearVelocity;
		protected btVector3 m_deltaAngularVelocity;
		protected btVector3 m_angularFactor;
		protected btVector3 m_invMass;
		protected btVector3 m_pushVelocity;
		protected btVector3 m_turnVelocity;

		//'temporarily' global variables
		public static double gDeactivationTime = (double)( 2.0 );
		public static bool gDisableDeactivation = false;
		public static int uniqueId = 0;


		public btRigidBody( btRigidBodyConstructionInfo constructionInfo )
		{
			setupRigidBody( constructionInfo );
		}

		///btRigidBody constructor for backwards compatibility. 
		///To specify friction (etc) during rigid body construction, please use the other constructor (using btRigidBodyConstructionInfo)
		//    btRigidBody( double mass, btMotionState* motionState, btCollisionShape* collisionShape, ref btVector3 localInertia = btVector3.Zero );
		// cannot give default to ref parameter
		public btRigidBody( double mass, btMotionState motionState, btCollisionShape collisionShape, ref btVector3 localInertia )
		{
			btRigidBodyConstructionInfo cinfo = new btRigidBodyConstructionInfo( mass, motionState, collisionShape, ref localInertia );
			setupRigidBody( cinfo );
		}

		///setupRigidBody is only used internally by the constructor
		public void setupRigidBody( btRigidBodyConstructionInfo constructionInfo )
		{

			m_internalType = CollisionObjectTypes.CO_RIGID_BODY;

			m_linearVelocity.setValue( btScalar.BT_ZERO, btScalar.BT_ZERO, btScalar.BT_ZERO );
			m_angularVelocity.setValue( btScalar.BT_ZERO, btScalar.BT_ZERO, btScalar.BT_ZERO );
			m_angularFactor.setValue( 1, 1, 1 );
			m_linearFactor.setValue( 1, 1, 1 );
			m_gravity.setValue( btScalar.BT_ZERO, btScalar.BT_ZERO, btScalar.BT_ZERO );
			m_gravity_acceleration.setValue( btScalar.BT_ZERO, btScalar.BT_ZERO, btScalar.BT_ZERO );
			m_totalForce.setValue( btScalar.BT_ZERO, btScalar.BT_ZERO, btScalar.BT_ZERO );
			m_totalTorque.setValue( btScalar.BT_ZERO, btScalar.BT_ZERO, btScalar.BT_ZERO );
			setDamping( constructionInfo.m_linearDamping, constructionInfo.m_angularDamping );

			m_linearSleepingThreshold = constructionInfo.m_linearSleepingThreshold;
			m_angularSleepingThreshold = constructionInfo.m_angularSleepingThreshold;
			m_optionalMotionState = constructionInfo.m_motionState;
			m_contactSolverType = 0;
			m_frictionSolverType = 0;
			m_additionalDamping = constructionInfo.m_additionalDamping;
			m_additionalDampingFactor = constructionInfo.m_additionalDampingFactor;
			m_additionalLinearDampingThresholdSqr = constructionInfo.m_additionalLinearDampingThresholdSqr;
			m_additionalAngularDampingThresholdSqr = constructionInfo.m_additionalAngularDampingThresholdSqr;
			m_additionalAngularDampingFactor = constructionInfo.m_additionalAngularDampingFactor;

			if( m_optionalMotionState != null )
			{
				m_optionalMotionState.getWorldTransform( out m_worldTransform );
			}
			else
			{
				m_worldTransform = constructionInfo.m_startWorldTransform;
			}

			m_interpolationWorldTransform = m_worldTransform;
			m_interpolationLinearVelocity.setValue( 0, 0, 0 );
			m_interpolationAngularVelocity.setValue( 0, 0, 0 );

			//moved to btCollisionObject
			m_friction = constructionInfo.m_friction;
			m_rollingFriction = constructionInfo.m_rollingFriction;
			m_restitution = constructionInfo.m_restitution;

			setCollisionShape( constructionInfo.m_collisionShape );
			m_debugBodyId = uniqueId++;

			setMassProps( constructionInfo.m_mass, ref constructionInfo.m_localInertia );
			updateInertiaTensor();

			m_rigidbodyFlags = btRigidBodyFlags.BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY;

			m_deltaLinearVelocity.setZero();
			m_deltaAngularVelocity.setZero();
			m_linearFactor.Mult( m_inverseMass, out m_invMass );
			//m_invMass = m_inverseMass * m_linearFactor;
			m_pushVelocity.setZero();
			m_turnVelocity.setZero();
		}

		public void predictIntegratedTransform( double timeStep, out btTransform predictedTransform )
		{
			btTransformUtil.integrateTransform( m_worldTransform, m_linearVelocity,  m_angularVelocity, timeStep, out predictedTransform );
		}

		/// continuous collision detection needs prediction
		internal void saveKinematicState( double timeStep )
		{
			//todo: clamp to some (user definable) safe minimum timestep, to limit maximum angular/linear velocities
			if( timeStep != btScalar.BT_ZERO )
			{
				//if we use motionstate to synchronize world transforms, get the new kinematic/animated world transform
				if( m_optionalMotionState != null )
					m_optionalMotionState.getWorldTransform( out m_worldTransform );

				btTransformUtil.calculateVelocity( ref m_interpolationWorldTransform, ref m_worldTransform, timeStep, out m_linearVelocity, out m_angularVelocity );
				m_interpolationLinearVelocity = m_linearVelocity;
				m_interpolationAngularVelocity = m_angularVelocity;
				m_interpolationWorldTransform = m_worldTransform;
				//Console.WriteLine("angular = %f %f %f\n",m_angularVelocity.x,m_angularVelocity.y,m_angularVelocity.z);
			}
		}

		void getAabb( out btVector3 aabbMin, out btVector3 aabbMax )
		{
			m_collisionShape.getAabb( ref m_worldTransform, out aabbMin, out aabbMax );
		}


		public void setGravity( ref btVector3 acceleration )
		{
			if( m_inverseMass != btScalar.BT_ZERO )
			{
				acceleration.Div( m_inverseMass, out m_gravity );
				//m_gravity = acceleration * ( (double)( 1.0 ) / m_inverseMass );
			}
			m_gravity_acceleration = acceleration;
		}



		public void setDamping( double lin_damping, double ang_damping )
		{
			m_linearDamping = btScalar.btClamped( lin_damping, (double)btScalar.BT_ZERO, (double)(double)( 1.0 ) );
			m_angularDamping = btScalar.btClamped( ang_damping, (double)btScalar.BT_ZERO, (double)(double)( 1.0 ) );
		}


		///applyDamping damps the velocity, using the given m_linearDamping and m_angularDamping
		public void applyDamping( double timeStep )
		{
			//On new damping: see discussion/issue report here: http://code.google.com/p/bullet/issues/detail?id=74
			//todo: do some performance comparisons (but other parts of the engine are probably bottleneck anyway

			//#define USE_OLD_DAMPING_METHOD 1
#if USE_OLD_DAMPING_METHOD
			m_linearVelocity *= GEN_clamped( ( (double)( 1.0) - timeStep * m_linearDamping ), (double)btScalar.BT_ZERO, (double)(double)( 1.0 ) );
			m_angularVelocity *= GEN_clamped( ( (double)( 1.0) - timeStep * m_angularDamping ), (double)btScalar.BT_ZERO, (double)(double)( 1.0 ) );
#else
			m_linearVelocity.Mult( btScalar.btPow( (double)( 1 ) - m_linearDamping, timeStep ), out m_linearVelocity );
			m_angularVelocity.Mult( btScalar.btPow( (double)( 1 ) - m_angularDamping, timeStep ), out m_angularVelocity );
			//m_linearVelocity *= btScalar.btPow( (double)( 1 ) - m_linearDamping, timeStep );
			//m_angularVelocity *= btScalar.btPow( (double)( 1 ) - m_angularDamping, timeStep );
#endif

			if( m_additionalDamping )
			{
				//Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
				//Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics system has improved, this should become obsolete
				if( ( m_angularVelocity.length2() < m_additionalAngularDampingThresholdSqr ) &
					( m_linearVelocity.length2() < m_additionalLinearDampingThresholdSqr ) )
				{
					m_linearVelocity.Mult( m_additionalDampingFactor, out m_linearVelocity );
					m_angularVelocity.Mult( m_additionalDampingFactor, out m_angularVelocity );
					//m_angularVelocity *= m_additionalDampingFactor;
					//m_linearVelocity *= m_additionalDampingFactor;
				}


				double speed = m_linearVelocity.length();
				if( speed < m_linearDamping )
				{
					double dampVel = (double)( 0.005 );
					if( speed > dampVel )
					{
						btVector3 dir; m_linearVelocity.normalized( out dir );
						dir.Mult( dampVel, out dir );
						m_linearVelocity.Sub( ref dir, out m_linearVelocity );
						//m_linearVelocity -= dir * dampVel;
					}
					else
					{
						m_linearVelocity.setValue( btScalar.BT_ZERO, btScalar.BT_ZERO, btScalar.BT_ZERO );
					}
				}

				double angSpeed = m_angularVelocity.length();
				if( angSpeed < m_angularDamping )
				{
					double angDampVel = (double)( 0.005 );
					if( angSpeed > angDampVel )
					{
						btVector3 dir; m_angularVelocity.normalized( out dir );
						dir.Mult( angDampVel, out dir );
						m_angularVelocity.Sub( ref dir, out m_angularVelocity );
						//m_angularVelocity -= dir * angDampVel;
					}
					else
					{
						m_angularVelocity.setValue( btScalar.BT_ZERO, btScalar.BT_ZERO, btScalar.BT_ZERO );
					}
				}
			}
		}


		internal void applyGravity()
		{
			if( isStaticOrKinematicObject() )
				return;

			applyCentralForce( ref m_gravity );

		}

		internal void proceedToTransform( ref btTransform newTrans )
		{
			setCenterOfMassTransform( ref newTrans );
		}


		public void setMassProps( double mass, ref btVector3 inertia )
		{
			if( mass == btScalar.BT_ZERO )
			{
				m_collisionFlags |= CollisionFlags.CF_STATIC_OBJECT;
				m_inverseMass = btScalar.BT_ZERO;
			}
			else
			{
				m_collisionFlags &= ( ~CollisionFlags.CF_STATIC_OBJECT );
				m_inverseMass = (double)( 1.0 ) / mass;
			}

			//Fg = m * a
			m_gravity_acceleration.Mult( mass, out m_gravity );
			//m_gravity = mass * m_gravity_acceleration;

			m_invInertiaLocal.setValue( inertia.x != btScalar.BT_ZERO ? (double)( 1.0 ) / inertia.x : btScalar.BT_ZERO,
						   inertia.y != btScalar.BT_ZERO ? (double)( 1.0 ) / inertia.y : btScalar.BT_ZERO,
						   inertia.z != btScalar.BT_ZERO ? (double)( 1.0 ) / inertia.z : btScalar.BT_ZERO );

			m_linearFactor.Mult( m_inverseMass, out m_invMass );
			//m_invMass = m_linearFactor * m_inverseMass;
		}


		void updateInertiaTensor()
		{
			btMatrix3x3 tmp;
			btMatrix3x3 tmp2;
			m_worldTransform.getBasis().transpose( out tmp );
			m_worldTransform.m_basis.scaled( ref m_invInertiaLocal, out tmp2 );
			tmp2.Mult( ref tmp, out m_invInertiaTensorWorld );
            //m_invInertiaTensorWorld = m_worldTransform.m_basis.scaled( m_invInertiaLocal ) * m_worldTransform.getBasis().transpose();
		}



		void getLocalInertia( out btVector3 result )
		{
			//btVector3 inertia = m_invInertiaLocal;
			btVector3.setValue( out result ,
				m_invInertiaLocal.x != btScalar.BT_ZERO ? (double)( 1.0 ) / m_invInertiaLocal.x : btScalar.BT_ZERO,
				m_invInertiaLocal.y != btScalar.BT_ZERO ? (double)( 1.0 ) / m_invInertiaLocal.y : btScalar.BT_ZERO,
				m_invInertiaLocal.z != btScalar.BT_ZERO ? (double)( 1.0 ) / m_invInertiaLocal.z : btScalar.BT_ZERO );
		}

		public void evalEulerEqn( ref btVector3 w1, ref btVector3 w0, ref btVector3 T, double dt,
			ref btMatrix3x3 I, out btVector3 result )
		{
			btVector3 Iw0;
			btVector3 dtT;
			btVector3 Iw1;
			btVector3 cross;
			T.Mult( dt, out dtT );
			I.Apply( ref w0, out Iw0 );
			dtT.Add( ref Iw0, out Iw0 );
			// Iw0 = ( T * dt + I * w0 )

			I.Apply( ref w1, out Iw1 );
			w1.cross( ref Iw1, out cross );
			cross.Mult( dt, out cross );
			// cross = w1.cross( I * w1 ) * dt
			Iw1.Add( ref cross, out Iw1 );
			// Iw1 = Iw1 + cross

			Iw1.Sub( ref Iw0, out result );
			//btVector3 w2; = I * w1 + w1.cross( I * w1 ) * dt - ( T * dt + I * w0 );
			//return w2;
		}

		public void evalEulerEqnDeriv( ref btVector3 w1, ref btVector3 w0, double dt,
			ref btMatrix3x3 I, out btMatrix3x3 result )
		{

			btMatrix3x3 w1x, Iw1x;
			btMatrix3x3 tmpm;
			btMatrix3x3 tmp2;
			btVector3 Iwi; I.Apply( ref w1, out Iwi );
			w1.getSkewSymmetricMatrix( out w1x.m_el0, out w1x.m_el1, out w1x.m_el2 );
			w1x.m_el3 = btVector3.wAxis;
			Iwi.getSkewSymmetricMatrix( out Iw1x.m_el0, out Iw1x.m_el1, out Iw1x.m_el2 );
			Iw1x.m_el3 = btVector3.wAxis;
			I.Mult( ref w1x, out tmpm );
			tmpm.Sub( ref Iw1x, out tmp2 );
			tmp2.Mult( dt, out result );

			//btMatrix3x3 dfw1 = I + ( w1x * I - Iw1x ) * dt;
			//return dfw1;
		}

		///explicit version is best avoided, it gains energy
		public void computeGyroscopicForceExplicit( double maxGyroscopicForce, out btVector3 result )
		{
			btVector3 inertiaLocal; getLocalInertia( out inertiaLocal );
			btMatrix3x3 tmpm1; m_worldTransform.m_basis.scaled( ref inertiaLocal, out tmpm1 );
            btMatrix3x3 tmpm2; m_worldTransform.m_basis.transpose( out tmpm2 );
			btMatrix3x3 inertiaTensorWorld; tmpm1.Mult( ref tmpm2, out inertiaTensorWorld );;
			btVector3 tmp; inertiaTensorWorld.Apply( ref m_angularVelocity, out tmp );
			btVector3 gf; m_angularVelocity.cross( ref tmp, out gf );
			double l2 = gf.length2();
			if( l2 > maxGyroscopicForce * maxGyroscopicForce )
			{
				gf.Mult( (double)( 1.0) / btScalar.btSqrt( l2 ) * maxGyroscopicForce, out result );
			}
			result = gf;
		}
#if !DISABLE_OPERATORS
		public btVector3 computeGyroscopicForceExplicit( double maxGyroscopicForce )
		{
			btVector3 tmp;
			computeGyroscopicForceExplicit( maxGyroscopicForce, out tmp );
			return tmp;
		}
#endif

		public static void btSetCrossMatrixMinus( btMatrix3x3 res, ref btVector3 a )
		{
			double a_0 = a.x, a_1 = a.y, a_2 = a.z;
			res.setValue( 0, +a_2, -a_1,
				-a_2, 0, +a_0,
				+a_1, -a_0, 0 );
		}

		///perform implicit force computation in body space (inertial frame)
		public void computeGyroscopicImpulseImplicit_Body( double step, out btVector3 result )
		{
			btVector3 idl; getLocalInertia( out idl );
			btVector3 omega1; getAngularVelocity( out omega1 );
			btQuaternion q; m_worldTransform.getRotation( out q );
			q.inverse( out q );
			// Convert to body coordinates
			btVector3 omegab; q.Rotate( ref omega1, out omegab );
			btMatrix3x3 Ib;
			btMatrix3x3.setValue( out Ib, idl.x, 0, 0,
									0, idl.y, 0,
									0, 0, idl.z );

			btVector3 ibo; Ib.Apply( ref omegab, out ibo );

			// Residual vector
			btVector3 tmp;
			omegab.cross( ref ibo, out tmp );
			btVector3 f; tmp.Mult( step, out f );

			btMatrix3x3 skew0;
			omegab.getSkewSymmetricMatrix( out skew0.m_el0, out skew0.m_el1, out skew0.m_el2 );
			skew0.m_el3 = btVector3.wAxis;
			btVector3 om; Ib.Apply( ref omegab, out om );
			btMatrix3x3 skew1;
			om.getSkewSymmetricMatrix( out skew1.m_el0, out skew1.m_el1, out skew1.m_el2 );
			skew1.m_el3 = btVector3.wAxis;

			// Jacobian
			btMatrix3x3 tmpm;
			btMatrix3x3 tmpm2;
			skew0.Mult( ref Ib, out tmpm );
			tmpm.Sub( ref skew1, out tmpm2 );
			tmpm2.Mult( step, out tmpm2 );
			btMatrix3x3 J; Ib.Add( ref tmpm2, out J );// + ( skew0 * Ib - skew1 ) * step;

			//	btMatrix3x3 Jinv = J.inverse();
			//	btVector3 omega_div = Jinv*f;
			btVector3 omega_div; J.solve33( ref f, out omega_div );

			// Single Newton-Raphson update
			omegab.Sub( ref omega_div, out omegab );//Solve33(J, f);
															 // Back to world coordinates
			btVector3 omega2; q.Rotate( ref omegab, out omega2 );
			omega2.Sub( ref omega1, out result );
		}
#if !DISABLE_OPERATORS
		public btVector3 computeGyroscopicImpulseImplicit_Body( double maxGyroscopicForce )
		{
			btVector3 tmp;
			computeGyroscopicImpulseImplicit_Body( maxGyroscopicForce, out tmp );
			return tmp;
		}
#endif



		///perform implicit force computation in world space
		public void computeGyroscopicImpulseImplicit_World( double step, out btVector3 result )
		{
			// use full newton-euler equations.  common practice to drop the wxIw term. want it for better tumbling behavior.
			// calculate using implicit euler step so it's stable.

			btVector3 inertiaLocal; getLocalInertia( out inertiaLocal );
			btVector3 w0; getAngularVelocity( out w0 );

			btMatrix3x3 I;
			btMatrix3x3 tmp;
			btMatrix3x3 tmp2;
			m_worldTransform.m_basis.scaled( ref inertiaLocal, out tmp );
			m_worldTransform.m_basis.transpose( out tmp2 );
			tmp.Mult( ref tmp2, out I );

			// use newtons method to find implicit solution for new angular velocity (w')
			// f(w') = -(T*step + Iw) + Iw' + w' + w'xIw'*step = 0 
			// df/dw' = I + 1xIw'*step + w'xI*step

			btVector3 w1 = w0;

			// one step of newton's method
			{
				btVector3 fw; evalEulerEqn( ref w1, ref w0, ref btVector3.Zero, step, ref I, out fw );
				btMatrix3x3 dfw; evalEulerEqnDeriv( ref w1, ref w0, step, ref I, out dfw );

				btVector3 dw;
				dfw.solve33( ref fw, out dw );
				//btMatrix3x3 dfw_inv = dfw.inverse();
				//dw = dfw_inv*fw;
				w1.Sub( ref dw, out w1 );
				//w1 -= dw;
			}

			w1.Sub( ref w0, out result );
		}
#if !DISABLE_OPERATORS
		public btVector3 computeGyroscopicImpulseImplicit_World( double maxGyroscopicForce )
		{
			btVector3 tmp;
			computeGyroscopicImpulseImplicit_World( maxGyroscopicForce, out tmp );
			return tmp;
		}
#endif


		void integrateVelocities( double step )
		{
			if( isStaticOrKinematicObject() )
				return;

			btVector3 tmp;
			m_totalForce.Mult( ( m_inverseMass * step ), out tmp );
			m_linearVelocity.Add( ref tmp, out m_linearVelocity );
			m_invInertiaTensorWorld.Apply( ref m_totalTorque, out tmp );
			tmp.Mult( step, out tmp );
			m_angularVelocity.Add( ref tmp, out m_angularVelocity );

			//#define MAX_ANGVEL SIMD_HALF_PI
			/// clamp angular velocity. collision calculations will fail on higher angular velocities	
			double angvel = m_angularVelocity.length();
			if( angvel * step > btScalar.SIMD_HALF_PI )
			{
				m_angularVelocity.Mult( ( btScalar.SIMD_HALF_PI / step ) / angvel, out m_angularVelocity );
			}

		}

		void getOrientation( out btQuaternion result )
		{
			m_worldTransform.m_basis.getRotation( out result );
		}


		void setCenterOfMassTransform( ref btTransform xform )
		{

			if( isKinematicObject() )
			{
				m_interpolationWorldTransform = m_worldTransform;
			}
			else
			{
				m_interpolationWorldTransform = xform;
			}
			m_interpolationLinearVelocity = m_linearVelocity;
			m_interpolationAngularVelocity = m_angularVelocity;
			m_worldTransform = xform;
			updateInertiaTensor();
		}





		internal void addConstraintRef( btTypedConstraint c )
		{
			///disable collision with the 'other' body

			int index = m_constraintRefs.IndexOf( c );
			//don't add constraints that are already referenced
			//Debug.Assert(index == m_constraintRefs.Count);
			if( index == -1 )
			{
				m_constraintRefs.Add( c );
				btCollisionObject colObjA = c.getRigidBodyA();
				btCollisionObject colObjB = c.getRigidBodyB();
				if( colObjA == this )
				{
					colObjA.setIgnoreCollisionCheck( colObjB, true );
				}
				else
				{
					colObjB.setIgnoreCollisionCheck( colObjA, true );
				}
			}
		}

		internal void removeConstraintRef( btTypedConstraint c )
		{
			int index = m_constraintRefs.IndexOf( c );
			//don't remove constraints that are not referenced
			if( index != -1 )
			{
				m_constraintRefs.Remove( c );
				btCollisionObject colObjA = c.getRigidBodyA();
				btCollisionObject colObjB = c.getRigidBodyB();
				if( colObjA == this )
				{
					colObjA.setIgnoreCollisionCheck( colObjB, false );
				}
				else
				{
					colObjB.setIgnoreCollisionCheck( colObjA, false );
				}
			}
		}

#if SERIALIZE_DONE
		int calculateSerializeBufferSize()
		{
			int sz = sizeof( btRigidBodyData );
			return sz;
		}

		///fills the dataBuffer and returns the struct name (and 0 on failure)
		string serialize( object dataBuffer, class btSerializer* serializer)
{
	btRigidBodyData* rbd = (btRigidBodyData*)dataBuffer;

		btCollisionObject::serialize(&rbd.m_collisionObjectData, serializer);

	m_invInertiaTensorWorld.serialize(rbd.m_invInertiaTensorWorld);
	m_linearVelocity.serialize(rbd.m_linearVelocity);
	m_angularVelocity.serialize(rbd.m_angularVelocity);
	rbd.m_inverseMass = m_inverseMass;
	m_angularFactor.serialize(rbd.m_angularFactor);
	m_linearFactor.serialize(rbd.m_linearFactor);
	m_gravity.serialize(rbd.m_gravity);
	m_gravity_acceleration.serialize(rbd.m_gravity_acceleration);
	m_invInertiaLocal.serialize(rbd.m_invInertiaLocal);
	m_totalForce.serialize(rbd.m_totalForce);
	m_totalTorque.serialize(rbd.m_totalTorque);
	rbd.m_linearDamping = m_linearDamping;
	rbd.m_angularDamping = m_angularDamping;
	rbd.m_additionalDamping = m_additionalDamping;
	rbd.m_additionalDampingFactor = m_additionalDampingFactor;
	rbd.m_additionalLinearDampingThresholdSqr = m_additionalLinearDampingThresholdSqr;
	rbd.m_additionalAngularDampingThresholdSqr = m_additionalAngularDampingThresholdSqr;
	rbd.m_additionalAngularDampingFactor = m_additionalAngularDampingFactor;
	rbd.m_linearSleepingThreshold=m_linearSleepingThreshold;
	rbd.m_angularSleepingThreshold = m_angularSleepingThreshold;

	return btRigidBodyDataName;
}



	void serializeSingleObject(class btSerializer* serializer)
{
	btChunk* chunk = serializer.allocate( calculateSerializeBufferSize(), 1 );
	string structType = serialize( chunk.m_oldPtr, serializer );
	serializer.finalizeChunk( chunk, structType, BT_RIGIDBODY_CODE,(object)this);
	}
#endif

		///The btRigidBodyConstructionInfo structure provides information to create a rigid body. Setting mass to zero creates a fixed (non-dynamic) rigid body.
		///For dynamic objects, you can use the collision shape to approximate the local inertia tensor, otherwise use the zero vector (default argument)
		///You can use the motion state to synchronize the world transform between physics and graphics objects. 
		///And if the motion state is provided, the rigid body will initialize its initial world transform from the motion state,
		///m_startWorldTransform is only used when you don't provide a motion state.
		public struct btRigidBodyConstructionInfo
		{
			public double m_mass;

			///When a motionState is provided, the rigid body will initialize its world transform from the motion state
			///In this case, m_startWorldTransform is ignored.
			public btMotionState m_motionState;
			public btTransform m_startWorldTransform;

			public btCollisionShape m_collisionShape;
			public btVector3 m_localInertia;
			public double m_linearDamping;
			public double m_angularDamping;

			///best simulation results when friction is non-zero
			public double m_friction;
			///the m_rollingFriction prevents rounded shapes, such as spheres, cylinders and capsules from rolling forever.
			///See Bullet/Demos/RollingFrictionDemo for usage
			public double m_rollingFriction;
			///best simulation results using zero restitution.
			public double m_restitution;

			public double m_linearSleepingThreshold;
			public double m_angularSleepingThreshold;

			//Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
			//Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics system has improved, this should become obsolete
			public bool m_additionalDamping;
			public double m_additionalDampingFactor;
			public double m_additionalLinearDampingThresholdSqr;
			public double m_additionalAngularDampingThresholdSqr;
			public double m_additionalAngularDampingFactor;

			// interia default = btVector3.Zero
			public btRigidBodyConstructionInfo( double mass, btMotionState motionState
					, btCollisionShape collisionShape
					, ref btVector3 localInertia )
			{
				m_mass = ( mass );
				m_motionState = ( motionState );
				m_collisionShape = ( collisionShape );
				m_localInertia = ( localInertia );
				m_linearDamping = ( btScalar.BT_ZERO );
				m_angularDamping = ( btScalar.BT_ZERO );
				m_friction = ( (double)( 0.5 ) );
				m_rollingFriction = ( (double)( 0 ) );
				m_restitution = ( btScalar.BT_ZERO );
				m_linearSleepingThreshold = ( (double)( 0.8 ) );
				m_angularSleepingThreshold = ( (double)( 1 ) );
				m_additionalDamping = ( false );
				m_additionalDampingFactor = (double)( 0.005 );
				m_additionalLinearDampingThresholdSqr = ( (double)( 0.01 ) );
				m_additionalAngularDampingThresholdSqr = ( (double)( 0.01 ) );
				m_additionalAngularDampingFactor = ( (double)( 0.01 ) );
				m_startWorldTransform = btTransform.Identity;
			}
		}



		~btRigidBody()
		{
			//No constraints should point to this rigidbody
			//Remove constraints from the dynamics world before you delete the related rigidbodies. 
			Debug.Assert( m_constraintRefs.Count == 0 );
		}


		///to keep collision detection and dynamics separate we don't store a rigidbody pointer
		///but a rigidbody is derived from btCollisionObject, so we can safely perform an upcast
		public static btRigidBody upcast( btCollisionObject colObj )
		{
			if( ( colObj.getInternalType() & CollisionObjectTypes.CO_RIGID_BODY ) != 0 )
				return (btRigidBody)colObj;
			return null;
		}

		public void getGravity( out btVector3 result )
		{
			result = m_gravity_acceleration;
		}

		public double getLinearDamping()
		{
			return m_linearDamping;
		}

		public double getAngularDamping()
		{
			return m_angularDamping;
		}

		public double getLinearSleepingThreshold()
		{
			return m_linearSleepingThreshold;
		}

		public double getAngularSleepingThreshold()
		{
			return m_angularSleepingThreshold;
		}


		public void getLinearFactor( out btVector3 result )
		{
			result = m_linearFactor;
		}
		public void setLinearFactor( ref btVector3 linearFactor )
		{
			m_linearFactor = linearFactor;
			m_linearFactor.Mult( m_inverseMass, out m_invMass );
		}

		public double getInvMass() { return m_inverseMass; }

		public void getInvInertiaTensorWorld( out btMatrix3x3 result )
		{
			result = m_invInertiaTensorWorld;
		}
#if !DISABLE_OPERATORS
		public btVector3 getLinearFactor( )
		{
			return m_linearFactor;
		}
		public btIMatrix3x3 getInvInertiaTensorWorld()
		{
			return m_invInertiaTensorWorld;
		}
#endif

		public void applyCentralForce( ref btVector3 force )
		{
			btVector3 tmp;
			force.Mult( ref m_linearFactor, out tmp );
			m_totalForce.Add( ref tmp, out m_totalForce );
			//m_totalForce += force * m_linearFactor;
		}

		public void getTotalForce( out btVector3 result )
		{
			result = m_totalForce;
		}

		public void getTotalTorque( out btVector3 result )
		{
			result = m_totalTorque;
		}

		public void getInvInertiaDiagLocal( out btVector3 result )
		{
			result = m_invInertiaLocal;
		}

		public btIVector3 getInvInertiaDiagLocal( )
		{
			return m_invInertiaLocal;
		}
		public void setInvInertiaDiagLocal( ref btVector3 diagInvInertia )
		{
			m_invInertiaLocal = diagInvInertia;
		}

		public void setSleepingThresholds( double linear, double angular )
		{
			m_linearSleepingThreshold = linear;
			m_angularSleepingThreshold = angular;
		}

		public void applyTorque( ref btVector3 torque )
		{
			btVector3 tmp;
			torque.Mult( ref m_angularFactor, out tmp );
            m_totalTorque.Add( ref tmp, out m_totalTorque );
		}

		public void applyForce( ref btVector3 force, ref btVector3 rel_pos )
		{
			btVector3 tmp;
			btVector3 tmp2;
			applyCentralForce( ref force );
			force.Mult( ref m_linearFactor, out tmp );
			rel_pos.cross( ref tmp, out tmp2 );
            applyTorque( ref tmp2 );
		}

		public void applyCentralImpulse( ref btVector3 impulse )
		{
			btVector3 tmp;
			impulse.Mult( ref m_linearFactor, out tmp );
			tmp.Mult( m_inverseMass, out tmp );
			m_linearVelocity.Add( ref tmp, out m_linearVelocity );
		}

		public void applyTorqueImpulse( ref btVector3 torque )
		{
			btVector3 tmp;
			m_invInertiaTensorWorld.Apply( ref torque, out tmp );
			tmp.Mult( ref m_angularFactor, out tmp );
            m_angularVelocity.Add( ref tmp, out m_angularVelocity );
		}

		public void applyImpulse( ref btVector3 impulse, ref btVector3 rel_pos )
		{
			if( m_inverseMass != btScalar.BT_ZERO )
			{
				applyCentralImpulse( ref impulse );
				if( !m_angularFactor.isZero() )
				{
					btVector3 tmp;
					btVector3 tmp2;
					impulse.Mult( ref m_linearFactor, out tmp );
					rel_pos.cross( ref tmp, out tmp2 );
                    applyTorqueImpulse( ref tmp2 );
				}
			}
		}

		public void clearForces()
		{
			m_totalForce.setValue( btScalar.BT_ZERO, btScalar.BT_ZERO, btScalar.BT_ZERO );
			m_totalTorque.setValue( btScalar.BT_ZERO, btScalar.BT_ZERO, btScalar.BT_ZERO );
		}

		public void getCenterOfMassPosition( out btVector3 result )
		{
			m_worldTransform.getOrigin( out result );
		}

		public void getCenterOfMassTransform( out btTransform result )
		{
			result = m_worldTransform;
		}
		public btITransform getCenterOfMassTransform(  )
		{
			return m_worldTransform;
		}
		public void getLinearVelocity( out btVector3 result )
		{
			result = m_linearVelocity;
		}
		public void getAngularVelocity( out btVector3 result )
		{
			result = m_angularVelocity;
		}
#if !DISABLE_OPERATORS
		public btVector3 getLinearVelocity()
		{
			return m_linearVelocity;
		}
		public btVector3 getAngularVelocity( )
		{
			return m_angularVelocity;
		}
#endif

		public void setLinearVelocity( ref btVector3 lin_vel )
		{
			m_updateRevision++;
			m_linearVelocity = lin_vel;
		}

		public void setAngularVelocity( ref btVector3 ang_vel )
		{
			m_updateRevision++;
			m_angularVelocity = ang_vel;
		}

		public void getVelocityInLocalPoint( ref btVector3 rel_pos, out btVector3 result )
		{
			//we also calculate lin/ang velocity for kinematic objects
			m_angularVelocity.cross( ref rel_pos, out result );
            m_linearVelocity.Add( ref result, out result );

			//			return m_linearVelocity + m_angularVelocity.cross( ref rel_pos );

			//for kinematic objects, we could also use use:
			//		return 	(m_worldTransform(rel_pos) - m_interpolationWorldTransform(rel_pos)) / m_kinematicTimeStep;
		}

#if !DISABLE_OPERATORS
		public btVector3 getVelocityInLocalPoint( ref btVector3 rel_pos )
		{
			btVector3 result;
			//we also calculate lin/ang velocity for kinematic objects
			m_angularVelocity.cross( ref rel_pos, out result );
			m_linearVelocity.Add( ref result, out result );
			return result;
			//			return m_linearVelocity + m_angularVelocity.cross( ref rel_pos );

			//for kinematic objects, we could also use use:
			//		return 	(m_worldTransform(rel_pos) - m_interpolationWorldTransform(rel_pos)) / m_kinematicTimeStep;
		}
#endif
		public void translate( ref btVector3 v )
		{
			m_worldTransform.m_origin.Add( ref v, out m_worldTransform.m_origin );
		}



		public double computeImpulseDenominator( ref btVector3 pos, ref btVector3 normal )
		{
			btVector3 r0; pos.Sub( ref m_worldTransform.m_origin, out r0 );

			btVector3 c0; ( r0 ).cross( ref normal, out c0 );

			btVector3 tmp;
			m_invInertiaTensorWorld.Apply( ref c0, out tmp );
			btVector3 vec; tmp.cross( ref r0, out vec );

			return m_inverseMass + normal.dot( ref vec );

		}

		public double computeAngularImpulseDenominator( ref btVector3 axis )
		{
			btVector3 vec; m_invInertiaTensorWorld.Apply( ref axis, out vec );
			return axis.dot( ref vec );
		}

		public void updateDeactivation( double timeStep )
		{
			if( ( getActivationState() == ActivationState.ISLAND_SLEEPING ) || ( getActivationState() == ActivationState.DISABLE_DEACTIVATION ) )
				return;

			if( ( m_linearVelocity.length2() < m_linearSleepingThreshold * m_linearSleepingThreshold ) &&
				( m_angularVelocity.length2() < m_angularSleepingThreshold * m_angularSleepingThreshold ) )
			{
				m_deactivationTime += timeStep;
			}
			else
			{
				m_deactivationTime = btScalar.BT_ZERO;
				setActivationState( 0 );
			}

		}

		public bool wantsSleeping()
		{

			if( getActivationState() == ActivationState.DISABLE_DEACTIVATION )
				return false;

			//disable deactivation
			if( gDisableDeactivation || ( gDeactivationTime == btScalar.BT_ZERO ) )
				return false;

			if( ( getActivationState() == ActivationState.ISLAND_SLEEPING ) || ( getActivationState() == ActivationState.WANTS_DEACTIVATION ) )
				return true;

			if( m_deactivationTime > gDeactivationTime )
			{
				return true;
			}
			return false;
		}



		public btBroadphaseProxy getBroadphaseProxy()
		{
			return m_broadphaseHandle;
		}
		public void setNewBroadphaseProxy( btBroadphaseProxy broadphaseProxy )
		{
			m_broadphaseHandle = broadphaseProxy;
		}

		//btMotionState allows to automatic synchronize the world transform for active objects
		public btMotionState getMotionState()
		{
			return m_optionalMotionState;
		}
		public void setMotionState( btMotionState motionState )
		{
			m_optionalMotionState = motionState;
			if( m_optionalMotionState != null )
				motionState.getWorldTransform( out m_worldTransform );
		}


		public void setAngularFactor( ref btVector3 angFac )
		{
			m_updateRevision++;
			m_angularFactor = angFac;
		}

		public void setAngularFactor( double angFac )
		{
			m_updateRevision++;
			m_angularFactor.setValue( angFac, angFac, angFac );
		}

		public void getAngularFactor( out btVector3 result )
		{
			result = m_angularFactor;
		}
#if !DISABLE_OPERATORS
		public btIVector3 getAngularFactor()
		{
			return m_angularFactor;
		}
#endif

		//is this rigidbody added to a btCollisionWorld/btDynamicsWorld/btBroadphase?
		public bool isInWorld()
		{
			return ( m_broadphaseHandle != null );
		}


		public btTypedConstraint getConstraintRef( int index )
		{
			return m_constraintRefs[index];
		}

		public int getNumConstraintRefs()
		{
			return m_constraintRefs.Count;
		}

		void setFlags( btRigidBodyFlags flags )
		{
			m_rigidbodyFlags = flags;
		}

		public btRigidBodyFlags getFlags()
		{
			return m_rigidbodyFlags;
		}




		///////////////////////////////////////////////
#if SERIALIZE_DONE

virtual int calculateSerializeBufferSize()  const;

///fills the dataBuffer and returns the struct name (and 0 on failure)
virtual string serialize( object dataBuffer,  class btSerializer* serializer);

	virtual void serializeSingleObject(class btSerializer* serializer);
#endif
	};

};


#if SERIALIZE_DONE

//@todo add m_optionalMotionState and m_constraintRefs to btRigidBodyData
///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btRigidBodyFloatData
{
	btCollisionObjectFloatData m_collisionObjectData;
	btMatrix3x3FloatData m_invInertiaTensorWorld;
	btVector3FloatData m_linearVelocity;
	btVector3FloatData m_angularVelocity;
	btVector3FloatData m_angularFactor;
	btVector3FloatData m_linearFactor;
	btVector3FloatData m_gravity;
	btVector3FloatData m_gravity_acceleration;
	btVector3FloatData m_invInertiaLocal;
	btVector3FloatData m_totalForce;
	btVector3FloatData m_totalTorque;
	float m_inverseMass;
	float m_linearDamping;
	float m_angularDamping;
	float m_additionalDampingFactor;
	float m_additionalLinearDampingThresholdSqr;
	float m_additionalAngularDampingThresholdSqr;
	float m_additionalAngularDampingFactor;
	float m_linearSleepingThreshold;
	float m_angularSleepingThreshold;
	int m_additionalDamping;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btRigidBodyDoubleData
{
	btCollisionObjectDoubleData m_collisionObjectData;
	btMatrix3x3DoubleData m_invInertiaTensorWorld;
	btVector3DoubleData m_linearVelocity;
	btVector3DoubleData m_angularVelocity;
	btVector3DoubleData m_angularFactor;
	btVector3DoubleData m_linearFactor;
	btVector3DoubleData m_gravity;
	btVector3DoubleData m_gravity_acceleration;
	btVector3DoubleData m_invInertiaLocal;
	btVector3DoubleData m_totalForce;
	btVector3DoubleData m_totalTorque;
	double m_inverseMass;
	double m_linearDamping;
	double m_angularDamping;
	double m_additionalDampingFactor;
	double m_additionalLinearDampingThresholdSqr;
	double m_additionalAngularDampingThresholdSqr;
	double m_additionalAngularDampingFactor;
	double m_linearSleepingThreshold;
	double m_angularSleepingThreshold;
	int m_additionalDamping;
	char m_padding[4];
};

#endif


