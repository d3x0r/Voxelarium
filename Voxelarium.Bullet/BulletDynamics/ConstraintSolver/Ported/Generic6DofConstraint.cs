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


/*
2007-09-09
btGeneric6DofConstraint Refactored by Francisco Le?n
email: projectileman@yahoo.com
http://gimpact.sf.net
*/


using Bullet.LinearMath;
using Bullet.Types;
using System;
using System.Diagnostics;
/// 2009 March: btGeneric6DofConstraint refactored by Roman Ponomarev
/// Added support for generic constraint solver through getInfo1/getInfo2 methods
namespace Bullet.Dynamics.ConstraintSolver
{


	/*

	#if BT_USE_DOUBLE_PRECISION
	#define btGeneric6DofConstraintData2		btGeneric6DofConstraintDoubleData2
	#define btGeneric6DofConstraintDataName	"btGeneric6DofConstraintDoubleData2"
	#else
	#define btGeneric6DofConstraintData2		btGeneric6DofConstraintData
	#define btGeneric6DofConstraintDataName	"btGeneric6DofConstraintData"
	#endif //BT_USE_DOUBLE_PRECISION
	*/

	//! Rotation Limit structure for generic joints
	internal class btRotationalLimitMotor
	{

		//! limit_parameters
		//!@{
		internal double m_loLimit;//!< joint limit
		internal double m_hiLimit;//!< joint limit
		internal double m_targetVelocity;//!< target motor velocity
		internal double m_maxMotorForce;//!< max force on motor
		internal double m_maxLimitForce;//!< max force on limit
		internal double m_damping;//!< Damping.
		internal double m_limitSoftness;//! Relaxation factor
		internal double m_normalCFM;//!< Constraint force mixing factor
		internal double m_stopERP;//!< Error tolerance factor when joint is at limit
		internal double m_stopCFM;//!< Constraint force mixing factor when joint is at limit
		internal double m_bounce;//!< restitution factor
		internal bool m_enableMotor;

		//!@}

		//! temp_variables
		//!@{
		internal double m_currentLimitError;//!  How much is violated this limit
		internal double m_currentPosition;     //!  current value of angle 
		internal int m_currentLimit;//!< 0=free, 1=at lo limit, 2=at hi limit
		internal double m_accumulatedImpulse;
		//!@}

		internal btRotationalLimitMotor()
		{
			m_accumulatedImpulse = 0;
			m_targetVelocity = 0;
			m_maxMotorForce = 0.1f;
			m_maxLimitForce = 300.0f;
			m_loLimit = 1.0f;
			m_hiLimit = -1.0f;
			m_normalCFM = 0;
			m_stopERP = 0.2f;
			m_stopCFM = 0;
			m_bounce = 0.0f;
			m_damping = 1.0f;
			m_limitSoftness = 0.5f;
			m_currentLimit = 0;
			m_currentLimitError = 0;
			m_enableMotor = false;
		}

		internal btRotationalLimitMotor( ref btRotationalLimitMotor limot )
		{
			m_targetVelocity = limot.m_targetVelocity;
			m_maxMotorForce = limot.m_maxMotorForce;
			m_limitSoftness = limot.m_limitSoftness;
			m_loLimit = limot.m_loLimit;
			m_hiLimit = limot.m_hiLimit;
			m_normalCFM = limot.m_normalCFM;
			m_stopERP = limot.m_stopERP;
			m_stopCFM = limot.m_stopCFM;
			m_bounce = limot.m_bounce;
			m_currentLimit = limot.m_currentLimit;
			m_currentLimitError = limot.m_currentLimitError;
			m_enableMotor = limot.m_enableMotor;
		}



		//! Is limited
		internal bool isLimited()
		{
			if( m_loLimit > m_hiLimit ) return false;
			return true;
		}

		//! Need apply correction
		internal bool needApplyTorques()
		{
			if( m_currentLimit == 0 && m_enableMotor == false ) return false;
			return true;
		}

		//! calculates  error
		/*!
		calculates m_currentLimit and m_currentLimitError.
		*/
		internal int testLimitValue( double test_value )
		{
			if( m_loLimit > m_hiLimit )
			{
				m_currentLimit = 0;//Free from violation
				return 0;
			}
			if( test_value < m_loLimit )
			{
				m_currentLimit = 1;//low limit violation
				m_currentLimitError = test_value - m_loLimit;
				if( m_currentLimitError > btScalar.SIMD_PI )
					m_currentLimitError -= btScalar.SIMD_2_PI;
				else if( m_currentLimitError < -btScalar.SIMD_PI )
					m_currentLimitError += btScalar.SIMD_2_PI;
				return 1;
			}
			else if( test_value > m_hiLimit )
			{
				m_currentLimit = 2;//High limit violation
				m_currentLimitError = test_value - m_hiLimit;
				if( m_currentLimitError > btScalar.SIMD_PI )
					m_currentLimitError -= btScalar.SIMD_2_PI;
				else if( m_currentLimitError < -btScalar.SIMD_PI )
					m_currentLimitError += btScalar.SIMD_2_PI;
				return 2;
			};

			m_currentLimit = 0;//Free from violation
			return 0;

		}


		//! apply the correction impulses for two bodies
		//double solveAngularLimits(double timeStep,ref btVector3 axis, double jacDiagABInv,btRigidBody * body0, btRigidBody * body1);
		internal double solveAngularLimits(
			double timeStep, ref btVector3 axis, double jacDiagABInv,
			btRigidBody body0, btRigidBody body1 )
		{
			if( needApplyTorques() == false ) return 0.0f;

			double target_velocity = m_targetVelocity;
			double maxMotorForce = m_maxMotorForce;

			//current error correction
			if( m_currentLimit != 0 )
			{
				target_velocity = -m_stopERP * m_currentLimitError / ( timeStep );
				maxMotorForce = m_maxLimitForce;
			}

			maxMotorForce *= timeStep;

			// current velocity difference

			btVector3 angVelA = body0.getAngularVelocity();
			btVector3 angVelB = body1.getAngularVelocity();

			btVector3 vel_diff;
			vel_diff = angVelA - angVelB;



			double rel_vel = axis.dot( vel_diff );

			// correction velocity
			double motor_relvel = m_limitSoftness * ( target_velocity - m_damping * rel_vel );


			if( motor_relvel < btScalar.SIMD_EPSILON && motor_relvel > -btScalar.SIMD_EPSILON )
			{
				return 0.0f;//no need for applying force
			}


			// correction impulse
			double unclippedMotorImpulse = ( 1 + m_bounce ) * motor_relvel * jacDiagABInv;

			// clip correction impulse
			double clippedMotorImpulse;

			///@todo: should clip against accumulated impulse
			if( unclippedMotorImpulse > 0.0f )
			{
				clippedMotorImpulse = unclippedMotorImpulse > maxMotorForce ? maxMotorForce : unclippedMotorImpulse;
			}
			else
			{
				clippedMotorImpulse = unclippedMotorImpulse < -maxMotorForce ? -maxMotorForce : unclippedMotorImpulse;
			}


			// sort with accumulated impulses
			double lo = (double)( -btScalar.BT_LARGE_FLOAT );
			double hi = (double)( btScalar.BT_LARGE_FLOAT );

			double oldaccumImpulse = m_accumulatedImpulse;
			double sum = oldaccumImpulse + clippedMotorImpulse;
			m_accumulatedImpulse = sum > hi ? btScalar.BT_ZERO : sum < lo ? btScalar.BT_ZERO : sum;

			clippedMotorImpulse = m_accumulatedImpulse - oldaccumImpulse;

			btVector3 motorImp; axis.Mult( clippedMotorImpulse, out motorImp );

			body0.applyTorqueImpulse( ref motorImp );
			btVector3 tmp;
			motorImp.Invert( out tmp );
			body1.applyTorqueImpulse( ref tmp );

			return clippedMotorImpulse;


		}


	};



	internal class btTranslationalLimitMotor
	{
		internal btVector3 m_lowerLimit;//!< the constraint lower limits
		internal btVector3 m_upperLimit;//!< the constraint upper limits
		internal btVector3 m_accumulatedImpulse;
		//! Linear_Limit_parameters
		//!@{
		internal double m_limitSoftness;//!< Softness for linear limit
		internal double m_damping;//!< Damping for linear limit
		internal double m_restitution;//! Bounce parameter for linear limit
		internal btVector3 m_normalCFM;//!< Constraint force mixing factor
		internal btVector3 m_stopERP;//!< Error tolerance factor when joint is at limit
		internal btVector3 m_stopCFM;//!< Constraint force mixing factor when joint is at limit
									 //!@}
		internal bool[] m_enableMotor = new bool[3];
		internal btVector3 m_targetVelocity;//!< target motor velocity
		internal btVector3 m_maxMotorForce;//!< max force on motor
		internal btVector3 m_currentLimitError;//!  How much is violated this limit
		internal btVector3 m_currentLinearDiff;//!  Current relative offset of constraint frames
		internal int[] m_currentLimit = new int[3];//!< 0=free, 1=at lower limit, 2=at upper limit

		internal btTranslationalLimitMotor()
		{
			m_lowerLimit.setValue( 0, 0, 0 );
			m_upperLimit.setValue( 0, 0, 0 );
			m_accumulatedImpulse.setValue( 0, 0, 0 );
			m_normalCFM.setValue( 0, 0, 0 );
			m_stopERP.setValue( 0.2f, 0.2f, 0.2f );
			m_stopCFM.setValue( 0, 0, 0 );

			m_limitSoftness = 0.7f;
			m_damping = (double)( 1.0f );
			m_restitution = (double)( 0.5f );
			for( int i = 0; i < 3; i++ )
			{
				m_enableMotor[i] = false;
				m_targetVelocity[i] = (double)( 0 );
				m_maxMotorForce[i] = (double)( 0 );
			}
		}

		internal btTranslationalLimitMotor( btTranslationalLimitMotor other )
		{
			m_lowerLimit = other.m_lowerLimit;
			m_upperLimit = other.m_upperLimit;
			m_accumulatedImpulse = other.m_accumulatedImpulse;

			m_limitSoftness = other.m_limitSoftness;
			m_damping = other.m_damping;
			m_restitution = other.m_restitution;
			m_normalCFM = other.m_normalCFM;
			m_stopERP = other.m_stopERP;
			m_stopCFM = other.m_stopCFM;

			for( int i = 0; i < 3; i++ )
			{
				m_enableMotor[i] = other.m_enableMotor[i];
				m_targetVelocity[i] = other.m_targetVelocity[i];
				m_maxMotorForce[i] = other.m_maxMotorForce[i];
			}
		}

		//! Test limit
		/*!
		- free means upper < lower,
		- locked means upper == lower
		- limited means upper > lower
		- limitIndex: first 3 are linear, next 3 are angular
		*/
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal bool isLimited( int limitIndex )
		{
			return ( m_upperLimit[limitIndex] >= m_lowerLimit[limitIndex] );
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal bool needApplyForce( int limitIndex )
		{
			if( m_currentLimit[limitIndex] == 0 && m_enableMotor[limitIndex] == false ) return false;
			return true;
		}

		internal int testLimitValue( int limitIndex, double test_value )
		{
			double loLimit = m_lowerLimit[limitIndex];
			double hiLimit = m_upperLimit[limitIndex];
			if( loLimit > hiLimit )
			{
				m_currentLimit[limitIndex] = 0;//Free from violation
				m_currentLimitError[limitIndex] = (double)( 0 );
				return 0;
			}

			if( test_value < loLimit )
			{
				m_currentLimit[limitIndex] = 2;//low limit violation
				m_currentLimitError[limitIndex] = test_value - loLimit;
				return 2;
			}
			else if( test_value > hiLimit )
			{
				m_currentLimit[limitIndex] = 1;//High limit violation
				m_currentLimitError[limitIndex] = test_value - hiLimit;
				return 1;
			};

			m_currentLimit[limitIndex] = 0;//Free from violation
			m_currentLimitError[limitIndex] = (double)( 0 );
			return 0;
		}



		internal double solveLinearAxis(
			double timeStep,
			double jacDiagABInv,
			btRigidBody body1, btVector3 pointInA,
			btRigidBody body2, btVector3 pointInB,
			int limit_index,
			btVector3 axis_normal_on_a,
			btVector3 anchorPos )
		{

			///find relative velocity
			//    btVector3 rel_pos1 = pointInA - body1.getCenterOfMassPosition();
			//    btVector3 rel_pos2 = pointInB - body2.getCenterOfMassPosition();
			btVector3 rel_pos1 = anchorPos - body1.m_worldTransform.m_origin;
			btVector3 rel_pos2 = anchorPos - body2.m_worldTransform.m_origin;

			btVector3 vel1 = body1.getVelocityInLocalPoint( ref rel_pos1 );
			btVector3 vel2 = body2.getVelocityInLocalPoint( ref rel_pos2 );
			btVector3 vel = vel1 - vel2;

			double rel_vel = axis_normal_on_a.dot( vel );



			/// apply displacement correction

			//positional error (zeroth order error)
			double depth = -( pointInA - pointInB ).dot( axis_normal_on_a );
			double lo = (double)( -btScalar.BT_LARGE_FLOAT );
			double hi = (double)( btScalar.BT_LARGE_FLOAT );

			double minLimit = m_lowerLimit[limit_index];
			double maxLimit = m_upperLimit[limit_index];

			//handle the limits
			if( minLimit < maxLimit )
			{
				{
					if( depth > maxLimit )
					{
						depth -= maxLimit;
						lo = btScalar.BT_ZERO;

					}
					else
					{
						if( depth < minLimit )
						{
							depth -= minLimit;
							hi = btScalar.BT_ZERO;
						}
						else
						{
							return 0.0f;
						}
					}
				}
			}

			double normalImpulse = m_limitSoftness * ( m_restitution * depth / timeStep - m_damping * rel_vel ) * jacDiagABInv;




			double oldNormalImpulse = m_accumulatedImpulse[limit_index];
			double sum = oldNormalImpulse + normalImpulse;
			m_accumulatedImpulse[limit_index] = sum > hi ? btScalar.BT_ZERO : sum < lo ? btScalar.BT_ZERO : sum;
			normalImpulse = m_accumulatedImpulse[limit_index] - oldNormalImpulse;

			btVector3 impulse_vector = axis_normal_on_a * normalImpulse;
			body1.applyImpulse( ref impulse_vector, ref rel_pos1 );
			btVector3 tmp;
			impulse_vector.Invert( out tmp );
			body2.applyImpulse( ref tmp, ref rel_pos2 );



			return normalImpulse;
		}


	};


	/// btGeneric6DofConstraint between two rigidbodies each with a pivotpoint that descibes the axis location in local space
	/*!
	btGeneric6DofConstraint can leave any of the 6 degree of freedom 'free' or 'locked'.
	currently this limit supports rotational motors<br>
	<ul>
	<li> For Linear limits, use btGeneric6DofConstraint.setLinearUpperLimit, btGeneric6DofConstraint.setLinearLowerLimit. You can set the parameters with the btTranslationalLimitMotor structure accsesible through the btGeneric6DofConstraint.getTranslationalLimitMotor method.
	At this moment translational motors are not supported. May be in the future. </li>

	<li> For Angular limits, use the btRotationalLimitMotor structure for configuring the limit.
	This is accessible through btGeneric6DofConstraint.getLimitMotor method,
	This brings support for limit parameters and motors. </li>

	<li> Angulars limits have these possible ranges:
	<table border=1 >
	<tr>
		<td><b>AXIS</b></td>
		<td><b>MIN ANGLE</b></td>
		<td><b>MAX ANGLE</b></td>
	</tr><tr>
		<td>X</td>
		<td>-PI</td>
		<td>PI</td>
	</tr><tr>
		<td>Y</td>
		<td>-PI/2</td>
		<td>PI/2</td>
	</tr><tr>
		<td>Z</td>
		<td>-PI</td>
		<td>PI</td>
	</tr>
	</table>
	</li>
	</ul>

	*/
	internal class btGeneric6DofConstraint : btTypedConstraint
	{
		const bool D6_USE_OBSOLETE_METHOD = false;
		const bool D6_USE_FRAME_OFFSET = true;

		//const int GENERIC_D6_DISABLE_WARMSTARTING = 1;

		[Flags]
		internal enum bt6DofFlags
		{
			BT_6DOF_FLAGS_CFM_NORM = 1 << 0,
			BT_6DOF_FLAGS_CFM_STOP = 1 << 1,
			BT_6DOF_FLAGS_ERP_STOP = 1 << 2,
			BT_6DOF_FLAGS_CFM_NORM_Y = 1 << 3,
			BT_6DOF_FLAGS_CFM_STOP_Y = 1 << 4,
			BT_6DOF_FLAGS_ERP_STOP_Y = 1 << 5,
			BT_6DOF_FLAGS_CFM_NORM_Z = 1 << 6,
			BT_6DOF_FLAGS_CFM_STOP_Z = 1 << 7,
			BT_6DOF_FLAGS_ERP_STOP_Z = 1 << 8,
			BT_6DOF_FLAGS_CFM_NORM_ROT = 1 << 9,
			BT_6DOF_FLAGS_CFM_STOP_ROT = 1 << 10,
			BT_6DOF_FLAGS_ERP_STOP_ROT = 1 << 11,
			BT_6DOF_FLAGS_CFM_NORM_Y_ROT = 1 << 12,
			BT_6DOF_FLAGS_CFM_STOP_Y_ROT = 1 << 13,
			BT_6DOF_FLAGS_ERP_STOP_Y_ROT = 1 << 14,
			BT_6DOF_FLAGS_CFM_NORM_Z_ROT = 1 << 15,
			BT_6DOF_FLAGS_CFM_STOP_Z_ROT = 1 << 16,
			BT_6DOF_FLAGS_ERP_STOP_Z_ROT = 1 << 17
		};

		class bt6DofFlagsIndexed
		{
			static internal readonly bt6DofFlags[] BT_6DOF_FLAGS_CFM_NORM = { bt6DofFlags.BT_6DOF_FLAGS_CFM_NORM, bt6DofFlags.BT_6DOF_FLAGS_CFM_NORM_Y, bt6DofFlags.BT_6DOF_FLAGS_CFM_NORM_Z, bt6DofFlags.BT_6DOF_FLAGS_CFM_NORM_ROT, bt6DofFlags.BT_6DOF_FLAGS_CFM_NORM_Y_ROT, bt6DofFlags.BT_6DOF_FLAGS_CFM_NORM_Z_ROT };
			static internal readonly bt6DofFlags[] BT_6DOF_FLAGS_CFM_STOP = { bt6DofFlags.BT_6DOF_FLAGS_CFM_STOP, bt6DofFlags.BT_6DOF_FLAGS_CFM_STOP_Y, bt6DofFlags.BT_6DOF_FLAGS_CFM_STOP_Z, bt6DofFlags.BT_6DOF_FLAGS_CFM_STOP_ROT, bt6DofFlags.BT_6DOF_FLAGS_CFM_STOP_Y_ROT, bt6DofFlags.BT_6DOF_FLAGS_CFM_STOP_Z_ROT };
			static internal readonly bt6DofFlags[] BT_6DOF_FLAGS_ERP_STOP = { bt6DofFlags.BT_6DOF_FLAGS_ERP_STOP, bt6DofFlags.BT_6DOF_FLAGS_ERP_STOP_Y, bt6DofFlags.BT_6DOF_FLAGS_ERP_STOP_Z, bt6DofFlags.BT_6DOF_FLAGS_ERP_STOP_ROT, bt6DofFlags.BT_6DOF_FLAGS_ERP_STOP_Y_ROT, bt6DofFlags.BT_6DOF_FLAGS_ERP_STOP_Z_ROT };
		}

		const int BT_6DOF_FLAGS_AXIS_SHIFT = 3; // bits per axis


		//! relative_frames
		//!@{
		internal btTransform m_frameInA;//!< the constraint space w.r.t body A
		internal btTransform m_frameInB;//!< the constraint space w.r.t body B
										//!@}

		//! Jacobians
		//!@{
		btJacobianEntry[] m_jacLinear = new btJacobianEntry[3];//!< 3 orthogonal linear constraints
		btJacobianEntry[] m_jacAng = new btJacobianEntry[3];//!< 3 orthogonal angular constraints
															//!@}

		//! Linear_Limit_parameters
		//!@{
		btTranslationalLimitMotor m_linearLimits;
		//!@}


		//! hinge_parameters
		//!@{
		btRotationalLimitMotor[] m_angularLimits = new btRotationalLimitMotor[3];
		//!@}


		//! temporal variables
		//!@{
		double m_timeStep;
		internal btTransform m_calculatedTransformA;
		internal btTransform m_calculatedTransformB;
		btVector3 m_calculatedAxisAngleDiff;
		btVector3[] m_calculatedAxis = new btVector3[3];
		btVector3 m_calculatedLinearDiff;
		double m_factA;
		double m_factB;
		bool m_hasStaticBody;

		btVector3 m_AnchorPos; // point betwen pivots of bodies A and B to solve linear axes

		bool m_useLinearReferenceFrameA;
		bool m_useOffsetForConstraintFrame;

		bt6DofFlags m_flags;


		/*
		int setAngularLimits( btConstraintInfo2* info, int row_offset, ref btTransform transA, ref btTransform transB, ref btVector3 linVelA, ref btVector3 linVelB, ref btVector3 angVelA, ref btVector3 angVelB );

		int setLinearLimits( btConstraintInfo2* info, int row, ref btTransform transA, ref btTransform transB, ref btVector3 linVelA, ref btVector3 linVelB, ref btVector3 angVelA, ref btVector3 angVelB );

		void buildLinearJacobian(
			btJacobianEntry & jacLinear, btVector3 normalWorld,
			btVector3 pivotAInW, btVector3 pivotBInW );

		void buildAngularJacobian( btJacobianEntry & jacAngular, btVector3 jointAxisW );

		// tests linear limits
		void calculateLinearInfo();

		//! calcs the euler angles between the two bodies.
		void calculateAngleInfo();
		*/


		//public:



		///for backwards compatibility during the transition to 'getInfo/getInfo2'
		internal bool m_useSolveConstraintObsolete;


		internal btGeneric6DofConstraint( btRigidBody rbA, btRigidBody rbB, ref btTransform frameInA, ref btTransform frameInB, bool useLinearReferenceFrameA )
				: base( btObjectTypes.D6_CONSTRAINT_TYPE, rbA, rbB )
		{
			m_frameInA = ( frameInA );
			m_frameInB = ( frameInB );
			m_useLinearReferenceFrameA = ( useLinearReferenceFrameA );
			m_useOffsetForConstraintFrame = ( D6_USE_FRAME_OFFSET );
			m_flags = ( 0 );
			m_useSolveConstraintObsolete = ( D6_USE_OBSOLETE_METHOD );
			calculateTransforms();
		}



		internal btGeneric6DofConstraint( btRigidBody rbB, ref btTransform frameInB, bool useLinearReferenceFrameB )
			: base( btObjectTypes.D6_CONSTRAINT_TYPE, getFixedBody(), rbB )
		{
			m_frameInB = ( frameInB );
			m_useLinearReferenceFrameA = ( useLinearReferenceFrameB );
			m_useOffsetForConstraintFrame = ( D6_USE_FRAME_OFFSET );
			m_flags = ( 0 );
			m_useSolveConstraintObsolete = ( false );
			///not providing rigidbody A means implicitly using worldspace for body A
			rbB.m_worldTransform.Apply( ref m_frameInB, out m_frameInA );
			calculateTransforms();
		}



		//! Gets the global transform of the offset for body A
		/*!
		\sa btGeneric6DofConstraint.getFrameOffsetA, btGeneric6DofConstraint.getFrameOffsetB, btGeneric6DofConstraint.calculateAngleInfo.
		*/
		internal btITransform getCalculatedTransformA()
		{
			return m_calculatedTransformA;
		}

		//! Gets the global transform of the offset for body B
		/*!
		\sa btGeneric6DofConstraint.getFrameOffsetA, btGeneric6DofConstraint.getFrameOffsetB, btGeneric6DofConstraint.calculateAngleInfo.
		*/
		internal btITransform getCalculatedTransformB()
		{
			return m_calculatedTransformB;
		}

		internal btITransform getFrameOffsetA()
		{
			return m_frameInA;
		}

		internal btITransform getFrameOffsetB()
		{
			return m_frameInB;
		}

		void setLinearLowerLimit( ref btVector3 linearLower )
		{
			m_linearLimits.m_lowerLimit = linearLower;
		}

		void getLinearLowerLimit( ref btVector3 linearLower )
		{
			linearLower = m_linearLimits.m_lowerLimit;
		}

		void setLinearUpperLimit( ref btVector3 linearUpper )
		{
			m_linearLimits.m_upperLimit = linearUpper;
		}

		void getLinearUpperLimit( ref btVector3 linearUpper )
		{
			linearUpper = m_linearLimits.m_upperLimit;
		}

		void setAngularLowerLimit( ref btVector3 angularLower )
		{
			for( int i = 0; i < 3; i++ )
				m_angularLimits[i].m_loLimit = btScalar.btNormalizeAngle( angularLower[i] );
		}

		void getAngularLowerLimit( ref btVector3 angularLower )
		{
			for( int i = 0; i < 3; i++ )
				angularLower[i] = m_angularLimits[i].m_loLimit;
		}

		void setAngularUpperLimit( ref btVector3 angularUpper )
		{
			for( int i = 0; i < 3; i++ )
				m_angularLimits[i].m_hiLimit = btScalar.btNormalizeAngle( angularUpper[i] );
		}

		void getAngularUpperLimit( ref btVector3 angularUpper )
		{
			for( int i = 0; i < 3; i++ )
				angularUpper[i] = m_angularLimits[i].m_hiLimit;
		}

		//! Retrieves the angular limit informacion
		internal btRotationalLimitMotor getRotationalLimitMotor( int index )
		{
			return m_angularLimits[index];
		}

		//! Retrieves the  limit informacion
		internal btTranslationalLimitMotor getTranslationalLimitMotor()
		{
			return m_linearLimits;
		}

		//first 3 are linear, next 3 are angular
		void setLimit( int axis, double lo, double hi )
		{
			if( axis < 3 )
			{
				m_linearLimits.m_lowerLimit[axis] = lo;
				m_linearLimits.m_upperLimit[axis] = hi;
			}
			else
			{
				lo = btScalar.btNormalizeAngle( lo );
				hi = btScalar.btNormalizeAngle( hi );
				m_angularLimits[axis - 3].m_loLimit = lo;
				m_angularLimits[axis - 3].m_hiLimit = hi;
			}
		}

		//! Test limit
		/*!
		- free means upper < lower,
		- locked means upper == lower
		- limited means upper > lower
		- limitIndex: first 3 are linear, next 3 are angular
		*/
		internal bool isLimited( int limitIndex )
		{
			if( limitIndex < 3 )
			{
				return m_linearLimits.isLimited( limitIndex );

			}
			return m_angularLimits[limitIndex - 3].isLimited();
		}

		// access for UseFrameOffset
		internal bool getUseFrameOffset() { return m_useOffsetForConstraintFrame; }
		internal void setUseFrameOffset( bool frameOffsetOnOff ) { m_useOffsetForConstraintFrame = frameOffsetOnOff; }
		internal bool getUseLinearReferenceFrameA()  { return m_useLinearReferenceFrameA; }
		internal void setUseLinearReferenceFrameA( bool linearReferenceFrameA ) { m_useLinearReferenceFrameA = linearReferenceFrameA; }

		internal virtual bt6DofFlags getFlags() 
    	{
        	return m_flags;
		}

#if SERIALIZE_DONE
		virtual int calculateSerializeBufferSize();

		///fills the dataBuffer and returns the struct name (and 0 on failure)
		virtual string serialize( object dataBuffer, btSerializer* serializer );
#endif
	void calculateAngleInfo()
		{
			btMatrix3x3 tmp;
			m_calculatedTransformA.m_basis.inverse( out tmp );
			btMatrix3x3 relative_frame; tmp.Apply( ref m_calculatedTransformB.m_basis, out relative_frame );

			//matrixToEulerXYZ( 
			relative_frame.getEulerXYZ( out m_calculatedAxisAngleDiff );
			// in euler angle mode we do not actually constrain the angular velocity
			// along the axes axis[0] and axis[2] (although we do use axis[1]) :
			//
			//    to get			constrain w2-w1 along		...not
			//    ------			---------------------		------
			//    d(angle[0])/dt = 0	ax[1] x ax[2]			ax[0]
			//    d(angle[1])/dt = 0	ax[1]
			//    d(angle[2])/dt = 0	ax[0] x ax[1]			ax[2]
			//
			// constraining w2-w1 along an axis 'a' means that a'*(w2-w1)=0.
			// to prove the result for angle[0], write the expression for angle[0] from
			// GetInfo1 then take the derivative. to prove this for angle[2] it is
			// easier to take the euler rate expression for d(angle[2])/dt with respect
			// to the components of w and set that to 0.
			btVector3 axis0 = m_calculatedTransformB.m_basis.getColumn( 0 );
			btVector3 axis2 = m_calculatedTransformA.m_basis.getColumn( 2 );

			m_calculatedAxis[1] = axis2.cross( axis0 );
			m_calculatedAxis[0] = m_calculatedAxis[1].cross( axis2 );
			m_calculatedAxis[2] = axis0.cross( m_calculatedAxis[1] );

			m_calculatedAxis[0].normalize();
			m_calculatedAxis[1].normalize();
			m_calculatedAxis[2].normalize();

		}

		internal void calculateTransforms()
		{
			calculateTransforms( ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform );
		}

		//! Calcs global transform of the offsets
		/*!
		Calcs the global transform for the joint offset for body A an B, and also calcs the agle differences between the bodies.
		\sa btGeneric6DofConstraint.getCalculatedTransformA , btGeneric6DofConstraint.getCalculatedTransformB, btGeneric6DofConstraint.calculateAngleInfo
		*/
		internal void calculateTransforms( ref btTransform transA, ref btTransform transB )
		{
			transA.Apply( ref m_frameInA, out m_calculatedTransformA );
			transB.Apply( ref m_frameInB, out m_calculatedTransformB );
			calculateLinearInfo();
			calculateAngleInfo();
			if( m_useOffsetForConstraintFrame )
			{   //  get weight factors depending on masses
				double miA = getRigidBodyA().getInvMass();
				double miB = getRigidBodyB().getInvMass();
				m_hasStaticBody = ( miA < btScalar.SIMD_EPSILON ) || ( miB < btScalar.SIMD_EPSILON );
				double miS = miA + miB;
				if( miS > (double)( 0 ) )
				{
					m_factA = miB / miS;
				}
				else
				{
					m_factA = (double)( 0.5f );
				}
				m_factB = (double)( 1.0f ) - m_factA;
			}
		}


		/*
		void buildLinearJacobian(
			btJacobianEntry jacLinear, btVector3 normalWorld,
			btVector3 pivotAInW, btVector3 pivotBInW )
		{
			new ( &jacLinear ) btJacobianEntry(
				  m_rbA.m_worldTransform.m_basis.transpose(),
				  m_rbB.m_worldTransform.m_basis.transpose(),
				  pivotAInW - m_rbA.getCenterOfMassPosition(),
				  pivotBInW - m_rbB.getCenterOfMassPosition(),
				  normalWorld,
				  m_rbA.getInvInertiaDiagLocal(),
				  m_rbA.getInvMass(),
				  m_rbB.getInvInertiaDiagLocal(),
				  m_rbB.getInvMass() );
		}



		void buildAngularJacobian(
			btJacobianEntry & jacAngular, btVector3 jointAxisW )
		{
			new ( &jacAngular )  btJacobianEntry( jointAxisW,
											   m_rbA.m_worldTransform.m_basis.transpose(),
											   m_rbB.m_worldTransform.m_basis.transpose(),
											   m_rbA.getInvInertiaDiagLocal(),
											   m_rbB.getInvInertiaDiagLocal() );

		}
		*/


		//! Test angular limit.
		/*!
		Calculates angular correction and returns true if limit needs to be corrected.
		\pre calculateTransforms() must be called previously.
		*/
		bool testAngularLimitMotor( int axis_index )
		{
			double angle = m_calculatedAxisAngleDiff[axis_index];
			angle = btAdjustAngleToLimits( angle, m_angularLimits[axis_index].m_loLimit, m_angularLimits[axis_index].m_hiLimit );
			m_angularLimits[axis_index].m_currentPosition = angle;
			//test limits
			m_angularLimits[axis_index].testLimitValue( angle );
			return m_angularLimits[axis_index].needApplyTorques();
		}



		//! performs Jacobian calculation, and also calculates angle differences and axis
		internal override void buildJacobian()
		{
			/*
			if( m_useSolveConstraintObsolete )
			{

				// Clear accumulated impulses for the next simulation step
				m_linearLimits.m_accumulatedImpulse.setValue( btScalar.BT_ZERO, btScalar.BT_ZERO, btScalar.BT_ZERO );
				int i;
				for( i = 0; i < 3; i++ )
				{
					m_angularLimits[i].m_accumulatedImpulse = btScalar.BT_ZERO;
				}
				//calculates transform
				calculateTransforms( m_rbA.m_worldTransform, m_rbB.m_worldTransform );

				//  ref btVector3 pivotAInW = m_calculatedTransformA.m_origin;
				//  ref btVector3 pivotBInW = m_calculatedTransformB.m_origin;
				calcAnchorPos();
				btVector3 pivotAInW = m_AnchorPos;
				btVector3 pivotBInW = m_AnchorPos;

				// not used here
				//    btVector3 rel_pos1 = pivotAInW - m_rbA.getCenterOfMassPosition();
				//    btVector3 rel_pos2 = pivotBInW - m_rbB.getCenterOfMassPosition();

				btVector3 normalWorld;
				//linear part
				for( i = 0; i < 3; i++ )
				{
					if( m_linearLimits.isLimited( i ) )
					{
						if( m_useLinearReferenceFrameA )
							normalWorld = m_calculatedTransformA.m_basis.getColumn( i );
						else
							normalWorld = m_calculatedTransformB.m_basis.getColumn( i );

						buildLinearJacobian(
							m_jacLinear[i], normalWorld,
							pivotAInW, pivotBInW );

					}
				}

				// angular part
				for( i = 0; i < 3; i++ )
				{
					//calculates error angle
					if( testAngularLimitMotor( i ) )
					{
						normalWorld = this.getAxis( i );
						// Create angular atom
						buildAngularJacobian( m_jacAng[i], normalWorld );
					}
				}

			}
			*/
		}


		internal override void getInfo1( ref btConstraintInfo1 info )
		{
			if( m_useSolveConstraintObsolete )
			{
				info.m_numConstraintRows = 0;
				info.nub = 0;
			}
			else
			{
				//prepare constraint
				calculateTransforms( ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform );
				info.m_numConstraintRows = 0;
				info.nub = 6;
				int i;
				//test linear limits
				for( i = 0; i < 3; i++ )
				{
					if( m_linearLimits.needApplyForce( i ) )
					{
						info.m_numConstraintRows++;
						info.nub--;
					}
				}
				//test angular limits
				for( i = 0; i < 3; i++ )
				{
					if( testAngularLimitMotor( i ) )
					{
						info.m_numConstraintRows++;
						info.nub--;
					}
				}
			}
		}

		void getInfo1NonVirtual( ref btConstraintInfo1 info )
		{
			if( m_useSolveConstraintObsolete )
			{
				info.m_numConstraintRows = 0;
				info.nub = 0;
			}
			else
			{
				//pre-allocate all 6
				info.m_numConstraintRows = 6;
				info.nub = 0;
			}
		}


		internal override void getInfo2(  btConstraintInfo2 info )
		{
			Debug.Assert( !m_useSolveConstraintObsolete );

			if( m_useOffsetForConstraintFrame )
			{ // for stability better to solve angular limits first
				int row = setAngularLimits( ref info, 0, ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform
					, ref m_rbA.m_linearVelocity, ref m_rbB.m_linearVelocity
					, ref m_rbA.m_angularVelocity, ref m_rbB.m_angularVelocity );
				setLinearLimits( ref info, row, ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform
					, ref m_rbA.m_linearVelocity, ref m_rbB.m_linearVelocity
					, ref m_rbA.m_angularVelocity, ref m_rbB.m_angularVelocity );
			}
			else
			{ // leave old version for compatibility
				int row = setLinearLimits( ref info, 0, ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform
					, ref m_rbA.m_linearVelocity, ref m_rbB.m_linearVelocity
					, ref m_rbA.m_angularVelocity, ref m_rbB.m_angularVelocity );
				setAngularLimits( ref info, row, ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform
					, ref m_rbA.m_linearVelocity, ref m_rbB.m_linearVelocity
					, ref m_rbA.m_angularVelocity, ref m_rbB.m_angularVelocity );
			}

		}


		void getInfo2NonVirtual( ref btConstraintInfo2 info
			, ref btTransform transA, ref btTransform transB
			, ref btVector3 linVelA, ref btVector3 linVelB
			, ref btVector3 angVelA, ref btVector3 angVelB )
		{

			Debug.Assert( !m_useSolveConstraintObsolete );
			//prepare constraint
			calculateTransforms( ref transA, ref transB );

			int i;
			for( i = 0; i < 3; i++ )
			{
				testAngularLimitMotor( i );
			}

			if( m_useOffsetForConstraintFrame )
			{ // for stability better to solve angular limits first
				int row = setAngularLimits( ref info, 0
					, ref transA, ref transB
					, ref linVelA, ref linVelB
					, ref angVelA, ref angVelB );
				setLinearLimits( ref info, row
					, ref transA, ref transB
					, ref linVelA, ref linVelB
					, ref angVelA, ref angVelB );
			}
			else
			{ // leave old version for compatibility
				int row = setLinearLimits( ref info, 0, ref transA, ref transB, ref linVelA, ref linVelB, ref angVelA, ref angVelB );
				setAngularLimits( ref info, row, ref transA, ref transB, ref linVelA, ref linVelB, ref angVelA, ref angVelB );
			}
		}



		int setLinearLimits( ref btConstraintInfo2 info, int row
			, ref btTransform transA, ref btTransform transB
			, ref btVector3 linVelA, ref btVector3 linVelB
			, ref btVector3 angVelA, ref btVector3 angVelB )
		{
			//	int row = 0;
			//solve linear limits
			btRotationalLimitMotor limot = new btRotationalLimitMotor();
			for( int i = 0; i < 3; i++ )
			{
				if( m_linearLimits.needApplyForce( i ) )
				{ // re-use rotational motor code
					limot.m_bounce = btScalar.BT_ZERO;
					limot.m_currentLimit = m_linearLimits.m_currentLimit[i];
					limot.m_currentPosition = m_linearLimits.m_currentLinearDiff[i];
					limot.m_currentLimitError = m_linearLimits.m_currentLimitError[i];
					limot.m_damping = m_linearLimits.m_damping;
					limot.m_enableMotor = m_linearLimits.m_enableMotor[i];
					limot.m_hiLimit = m_linearLimits.m_upperLimit[i];
					limot.m_limitSoftness = m_linearLimits.m_limitSoftness;
					limot.m_loLimit = m_linearLimits.m_lowerLimit[i];
					limot.m_maxLimitForce = (double)( 0 );
					limot.m_maxMotorForce = m_linearLimits.m_maxMotorForce[i];
					limot.m_targetVelocity = m_linearLimits.m_targetVelocity[i];
					btVector3 axis = m_calculatedTransformA.m_basis.getColumn( i );
					//bt6DofFlags flags = m_flags >> ( i * BT_6DOF_FLAGS_AXIS_SHIFT );
					limot.m_normalCFM = ( m_flags & bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_NORM[i] ) != 0 ? m_linearLimits.m_normalCFM[i] : info.m_solverConstraints[0].m_cfm;
					limot.m_stopCFM = ( m_flags & bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_STOP[i] ) != 0 ? m_linearLimits.m_stopCFM[i] : info.m_solverConstraints[0].m_cfm;
					limot.m_stopERP = ( m_flags & bt6DofFlagsIndexed.BT_6DOF_FLAGS_ERP_STOP[i] ) != 0 ? m_linearLimits.m_stopERP[i] : info.erp;
					if( m_useOffsetForConstraintFrame )
					{
						int indx1 = ( i + 1 ) % 3;
						int indx2 = ( i + 2 ) % 3;
						bool rotAllowed = true; // rotations around orthos to current axis
						if( m_angularLimits[indx1].m_currentLimit != 0
							&& m_angularLimits[indx2].m_currentLimit != 0 )
						{
							rotAllowed = false;
						}
						row += get_limit_motor_info2( limot, ref transA, ref transB, ref linVelA, ref linVelB, ref angVelA, ref angVelB, ref info, row, ref axis, false, rotAllowed );
					}
					else
					{
						row += get_limit_motor_info2( limot, ref transA, ref transB, ref linVelA, ref linVelB, ref angVelA, ref angVelB, ref info, row, ref axis, false );
					}
				}
			}
			return row;
		}



		int setAngularLimits( ref btConstraintInfo2 info
			, int row_offset, ref btTransform transA, ref btTransform transB
			, ref btVector3 linVelA, ref btVector3 linVelB
			, ref btVector3 angVelA, ref btVector3 angVelB )
		{
			btGeneric6DofConstraint d6constraint = this;
			int row = row_offset;
			//solve angular limits
			for( int i = 0; i < 3; i++ )
			{
				if( d6constraint.getRotationalLimitMotor( i ).needApplyTorques() )
				{
					btVector3 axis = d6constraint.getAxis( i );
					//int flags = m_flags >> ( ( i + 3 ) * BT_6DOF_FLAGS_AXIS_SHIFT );
					if( ( m_flags & bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_NORM[i] ) == 0 )
					{
						m_angularLimits[i].m_normalCFM = info.m_solverConstraints[0].m_cfm;
					}
					if( ( m_flags & bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_STOP[i] ) == 0 )
					{
						m_angularLimits[i].m_stopCFM = info.m_solverConstraints[0].m_cfm;
					}
					if( ( m_flags & bt6DofFlagsIndexed.BT_6DOF_FLAGS_ERP_STOP[i] ) == 0 )
					{
						m_angularLimits[i].m_stopERP = info.erp;
					}
					row += get_limit_motor_info2( d6constraint.getRotationalLimitMotor( i ),
														ref transA, ref transB
														, ref linVelA, ref linVelB
														, ref angVelA, ref angVelB
														, ref info, row, ref axis, true );
				}
			}
			return row;
		}




		void updateRHS( double timeStep )
		{
			//(void)timeStep;

		}


		void setFrames( ref btTransform frameA, ref btTransform frameB )
		{
			m_frameInA = frameA;
			m_frameInB = frameB;
			if( m_useSolveConstraintObsolete ) buildJacobian();
			calculateTransforms();
		}



		//! Get the rotation axis in global coordinates
		/*!
		\pre btGeneric6DofConstraint.buildJacobian must be called previously.
		*/
		internal btVector3 getAxis( int axis_index )
		{
			return m_calculatedAxis[axis_index];
		}


		//! Get the relative position of the constraint pivot
		/*!
		\pre calculateTransforms() must be called previously.
		*/
		internal double getRelativePivotPosition( int axisIndex )
		{
			return m_calculatedLinearDiff[axisIndex];
		}


		//! Get the relative Euler angle
		/*!
		\pre calculateTransforms() must be called previously.
		*/
		internal double getAngle( int axisIndex )
		{
			return m_calculatedAxisAngleDiff[axisIndex];
		}



		internal virtual void calcAnchorPos()
		{
			double imA = m_rbA.getInvMass();
			double imB = m_rbB.getInvMass();
			double weight;
			if( imB == (double)( 0.0 ) )
			{
				weight = (double)( 1.0 );
			}
			else
			{
				weight = imA / ( imA + imB );
			}
			btIVector3 pA = m_calculatedTransformA.m_origin;
			btIVector3 pB = m_calculatedTransformB.m_origin;
			btVector3 tmp, tmp2;
			pA.Mult( weight, out tmp );
			pB.Mult( btScalar.BT_ONE - weight, out tmp2 );
			tmp.Add( ref tmp2, out m_AnchorPos );
			//m_AnchorPos[0] = pA * weight + pB * ( btScalar.BT_ONE - weight );
			return;
		}



		internal void calculateLinearInfo()
		{
			btVector3 tmpv;
			m_calculatedTransformB.m_origin.Sub( ref m_calculatedTransformA.m_origin, out tmpv );
			btMatrix3x3 tmp;
			m_calculatedTransformA.m_basis.inverse( out tmp );
			tmp.Apply( ref tmpv, out m_calculatedLinearDiff );
			for( int i = 0; i < 3; i++ )
			{
				m_linearLimits.m_currentLinearDiff[i] = m_calculatedLinearDiff[i];
				m_linearLimits.testLimitValue( i, m_calculatedLinearDiff[i] );
			}
		}


		internal int get_limit_motor_info2(
			btRotationalLimitMotor limot,
			ref btTransform transA, ref btTransform transB, ref btVector3 linVelA, ref btVector3 linVelB, ref btVector3 angVelA, ref btVector3 angVelB,
			ref btConstraintInfo2 info, int row, ref btVector3 ax1, bool rotational, bool rotAllowed = false )
		{
			//int srow = row * info.rowskip;
			bool powered = limot.m_enableMotor;
			int limit = limot.m_currentLimit;
			if( powered || limit != 0 )
			{   // if the joint is powered, or has joint limits, add in the extra row
				if( rotational )
				{
					info.m_solverConstraints[row].m_relpos1CrossNormal = ax1;
					ax1.Invert( out info.m_solverConstraints[row].m_relpos2CrossNormal );
				}
				else
				{
					info.m_solverConstraints[row].m_contactNormal1 = ax1;
					ax1.Invert( out info.m_solverConstraints[row].m_contactNormal2 );
				}
				/*
				double* J1 = rotational ? info.m_J1angularAxis : info.m_J1linearAxis;
				double* J2 = rotational ? info.m_J2angularAxis : info.m_J2linearAxis;
				J1[srow + 0] = ax1[0];
				J1[srow + 1] = ax1[1];
				J1[srow + 2] = ax1[2];

				J2[srow + 0] = -ax1[0];
				J2[srow + 1] = -ax1[1];
				J2[srow + 2] = -ax1[2];
				*/

				if( ( !rotational ) )
				{
					if( m_useOffsetForConstraintFrame )
					{
						btVector3 tmpA, tmpB, relA, relB;
						// get vector from bodyB to frameB in WCS
						m_calculatedTransformB.m_origin.Sub( ref transB.m_origin, out relB );
						// get its projection to constraint axis
						btVector3 projB = ax1 * relB.dot( ax1 );
						// get vector directed from bodyB to constraint axis (and orthogonal to it)
						btVector3 orthoB = relB - projB;
						// same for bodyA
						m_calculatedTransformA.m_origin.Sub( ref transA.m_origin, out relA );
						btVector3 projA = ax1 * relA.dot( ax1 );
						btVector3 orthoA = relA - projA;
						// get desired offset between frames A and B along constraint axis
						double desiredOffs = limot.m_currentPosition - limot.m_currentLimitError;
						// desired vector from projection of center of bodyA to projection of center of bodyB to constraint axis
						btVector3 totalDist = projA + ax1 * desiredOffs - projB;
						// get offset vectors relA and relB
						relA = orthoA + totalDist * m_factA;
						relB = orthoB - totalDist * m_factB;
						tmpA = relA.cross( ax1 );
						tmpB = relB.cross( ax1 );
						if( m_hasStaticBody && ( !rotAllowed ) )
						{
							tmpA *= m_factA;
							tmpB *= m_factB;
						}
						//int i;
						info.m_solverConstraints[row].m_relpos1CrossNormal = tmpA;
						tmpB.Invert( out info.m_solverConstraints[row].m_relpos2CrossNormal );
						//for( i = 0; i < 3; i++ ) info.m_J1angularAxis[srow + i] = tmpA[i];
						//for( i = 0; i < 3; i++ ) info.m_J2angularAxis[srow + i] = -tmpB[i];
					}
					else
					{
						btVector3 ltd;  // Linear Torque Decoupling vector
						btVector3 c = m_calculatedTransformB.m_origin - transA.m_origin;
						ltd = c.cross( ax1 );
						info.m_solverConstraints[row].m_relpos1CrossNormal = ltd;
						//info.m_J1angularAxis[srow + 0] = ltd[0];
						//info.m_J1angularAxis[srow + 1] = ltd[1];
						//info.m_J1angularAxis[srow + 2] = ltd[2];

						c = m_calculatedTransformB.m_origin - transB.m_origin;
						ltd = -c.cross( ax1 );
						info.m_solverConstraints[row].m_relpos2CrossNormal = ltd;
						//info.m_J2angularAxis[srow + 0] = ltd[0];
						//info.m_J2angularAxis[srow + 1] = ltd[1];
						//info.m_J2angularAxis[srow + 2] = ltd[2];
					}
				}
				// if we're limited low and high simultaneously, the joint motor is
				// ineffective
				if( limit != 0 && ( limot.m_loLimit == limot.m_hiLimit ) ) powered = false;
				info.m_solverConstraints[row].m_rhs = (double)( 0 );
				if( powered )
				{
					info.m_solverConstraints[row].m_cfm = limot.m_normalCFM;
					if( limit == 0 )
					{
						double tag_vel = rotational ? limot.m_targetVelocity : -limot.m_targetVelocity;

						double mot_fact = getMotorFactor( limot.m_currentPosition,
															limot.m_loLimit,
															limot.m_hiLimit,
															tag_vel,
															info.fps * limot.m_stopERP );
						info.m_solverConstraints[row].m_rhs += mot_fact * limot.m_targetVelocity;
						info.m_solverConstraints[row].m_lowerLimit = -limot.m_maxMotorForce;
						info.m_solverConstraints[row].m_upperLimit = limot.m_maxMotorForce;
					}
				}
				if( limit != 0 )
				{
					double k = info.fps * limot.m_stopERP;
					if( !rotational )
					{
						info.m_solverConstraints[row].m_rhs += k * limot.m_currentLimitError;
					}
					else
					{
						info.m_solverConstraints[row].m_rhs += -k * limot.m_currentLimitError;
					}
					info.m_solverConstraints[row].m_cfm = limot.m_stopCFM;
					if( limot.m_loLimit == limot.m_hiLimit )
					{   // limited low and high simultaneously
						info.m_solverConstraints[row].m_lowerLimit = btScalar.BT_MIN_FLOAT;
						info.m_solverConstraints[row].m_upperLimit = btScalar.BT_MAX_FLOAT;
					}
					else
					{
						if( limit == 1 )
						{
							info.m_solverConstraints[row].m_lowerLimit = 0;
							info.m_solverConstraints[row].m_upperLimit = btScalar.BT_MAX_FLOAT;
						}
						else
						{
							info.m_solverConstraints[row].m_lowerLimit = btScalar.BT_MIN_FLOAT;
							info.m_solverConstraints[row].m_upperLimit = 0;
						}
						// deal with bounce
						if( limot.m_bounce > 0 )
						{
							// calculate joint velocity
							double vel;
							if( rotational )
							{
								vel = angVelA.dot( ax1 );
								//make sure that if no body . angVelB == zero vec
								//                        if (body1)
								vel -= angVelB.dot( ax1 );
							}
							else
							{
								vel = linVelA.dot( ax1 );
								//make sure that if no body . angVelB == zero vec
								//                        if (body1)
								vel -= linVelB.dot( ax1 );
							}
							// only apply bounce if the velocity is incoming, and if the
							// resulting c[] exceeds what we already have.
							if( limit == 1 )
							{
								if( vel < 0 )
								{
									double newc = -limot.m_bounce * vel;
									if( newc > info.m_solverConstraints[row].m_rhs )
										info.m_solverConstraints[row].m_rhs = newc;
								}
							}
							else
							{
								if( vel > 0 )
								{
									double newc = -limot.m_bounce * vel;
									if( newc < info.m_solverConstraints[row].m_rhs )
										info.m_solverConstraints[row].m_rhs = newc;
								}
							}
						}
					}
				}
				return 1;
			}
			else return 0;
		}






		///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
		///If no axis is provided, it uses the default axis for this constraint.
		internal override void setParam( btConstraintParams num, double value, int axis = -1 )
		{
			if( ( axis >= 0 ) && ( axis < 3 ) )
			{
				switch( num )
				{
					case btConstraintParams.BT_CONSTRAINT_STOP_ERP:
						m_linearLimits.m_stopERP[axis] = value;
						m_flags |= bt6DofFlagsIndexed.BT_6DOF_FLAGS_ERP_STOP[axis];
						break;
					case btConstraintParams.BT_CONSTRAINT_STOP_CFM:
						m_linearLimits.m_stopCFM[axis] = value;
						m_flags |= bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_STOP[axis];
						break;
					case btConstraintParams.BT_CONSTRAINT_CFM:
						m_linearLimits.m_normalCFM[axis] = value;
						m_flags |= bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_NORM[axis];
						break;
					default:
						btAssertConstrParams( false );
						break;
				}
			}
			else if( ( axis >= 3 ) && ( axis < 6 ) )
			{
				switch( num )
				{
					case btConstraintParams.BT_CONSTRAINT_STOP_ERP:
						m_angularLimits[axis - 3].m_stopERP = value;
						m_flags |= bt6DofFlagsIndexed.BT_6DOF_FLAGS_ERP_STOP[axis];
						break;
					case btConstraintParams.BT_CONSTRAINT_STOP_CFM:
						m_angularLimits[axis - 3].m_stopCFM = value;
						m_flags |= bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_STOP[axis];
						break;
					case btConstraintParams.BT_CONSTRAINT_CFM:
						m_angularLimits[axis - 3].m_normalCFM = value;
						m_flags |= bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_NORM[axis];
						break;
					default:
						btAssertConstrParams( false );
						break;
				}
			}
			else
			{
				btAssertConstrParams( false );
			}
		}

		///return the local value of parameter
		internal override double getParam( btConstraintParams num, int axis = -1 )
		{
			double retVal = 0;
			if( ( axis >= 0 ) && ( axis < 3 ) )
			{
				switch( num )
				{
					case btConstraintParams.BT_CONSTRAINT_STOP_ERP:
						btAssertConstrParams( ( m_flags & ( bt6DofFlagsIndexed.BT_6DOF_FLAGS_ERP_STOP[axis] ) ) != 0 );
						retVal = m_linearLimits.m_stopERP[axis];
						break;
					case btConstraintParams.BT_CONSTRAINT_STOP_CFM:
						btAssertConstrParams( ( m_flags & ( bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_STOP[axis] ) ) != 0 );
						retVal = m_linearLimits.m_stopCFM[axis];
						break;
					case btConstraintParams.BT_CONSTRAINT_CFM:
						btAssertConstrParams( ( m_flags & ( bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_NORM[axis] ) ) != 0 );
						retVal = m_linearLimits.m_normalCFM[axis];
						break;
					default:
						btAssertConstrParams( false );
						break;
				}
			}
			else if( ( axis >= 3 ) && ( axis < 6 ) )
			{
				switch( num )
				{
					case btConstraintParams.BT_CONSTRAINT_STOP_ERP:
						btAssertConstrParams( ( m_flags & ( bt6DofFlagsIndexed.BT_6DOF_FLAGS_ERP_STOP[axis] ) ) != 0 );
						retVal = m_angularLimits[axis - 3].m_stopERP;
						break;
					case btConstraintParams.BT_CONSTRAINT_STOP_CFM:
						btAssertConstrParams( ( m_flags & ( bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_STOP[axis] ) ) != 0 );
						retVal = m_angularLimits[axis - 3].m_stopCFM;
						break;
					case btConstraintParams.BT_CONSTRAINT_CFM:
						btAssertConstrParams( ( m_flags & ( bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_NORM[axis] ) ) != 0 );
						retVal = m_angularLimits[axis - 3].m_normalCFM;
						break;
					default:
						btAssertConstrParams( false );
						break;
				}
			}
			else
			{
				btAssertConstrParams( false );
			}
			return retVal;
		}



		void setAxis( ref btVector3 axis1, ref btVector3 axis2 )
		{
			btVector3 zAxis; axis1.normalized( out zAxis );
			btVector3 yAxis; axis2.normalized( out yAxis );
			btVector3 xAxis; yAxis.cross( ref zAxis, out xAxis ); // we want right coordinate system

			btTransform frameInW = btTransform.Identity;
			//frameInW.setIdentity();
			frameInW.m_basis.setValue( xAxis[0], yAxis[0], zAxis[0],
											xAxis[1], yAxis[1], zAxis[1],
										   xAxis[2], yAxis[2], zAxis[2] );

			// now get constraint frame in local coordinate systems
			btTransform inv;
			m_rbA.m_worldTransform.inverse( out inv );
			inv.Apply( ref frameInW, out m_frameInA );
			m_rbB.m_worldTransform.inverse( out inv );
			inv.Apply( ref frameInW, out m_frameInB );
			//m_frameInB = m_rbB.m_worldTransform.inverse() * frameInW;

			calculateTransforms();
		}

	};

#if SERIALIZE_DONE

struct btGeneric6DofConstraintData
{
	btTypedConstraintData m_typeConstraintData;
	btTransformFloatData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btTransformFloatData m_rbBFrame;

	btVector3FloatData m_linearUpperLimit;
	btVector3FloatData m_linearLowerLimit;

	btVector3FloatData m_angularUpperLimit;
	btVector3FloatData m_angularLowerLimit;

	int m_useLinearReferenceFrameA;
	int m_useOffsetForConstraintFrame;
};

struct btGeneric6DofConstraintDoubleData2
{
	btTypedConstraintDoubleData m_typeConstraintData;
	btTransformDoubleData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btTransformDoubleData m_rbBFrame;

	btVector3DoubleData m_linearUpperLimit;
	btVector3DoubleData m_linearLowerLimit;

	btVector3DoubleData m_angularUpperLimit;
	btVector3DoubleData m_angularLowerLimit;

	int m_useLinearReferenceFrameA;
	int m_useOffsetForConstraintFrame;
};

public int calculateSerializeBufferSize()
{
	return sizeof( btGeneric6DofConstraintData2 );
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
public string serialize( object dataBuffer, btSerializer* serializer )
{

	btGeneric6DofConstraintData2* dof = (btGeneric6DofConstraintData2*)dataBuffer;
	btTypedConstraint::serialize( &dof.m_typeConstraintData, serializer );

	m_frameInA.serialize( dof.m_rbAFrame );
	m_frameInB.serialize( dof.m_rbBFrame );


	int i;
	for( i = 0; i < 3; i++ )
	{
		dof.m_angularLowerLimit.m_floats[i] = m_angularLimits[i].m_loLimit;
		dof.m_angularUpperLimit.m_floats[i] = m_angularLimits[i].m_hiLimit;
		dof.m_linearLowerLimit.m_floats[i] = m_linearLimits.m_lowerLimit[i];
		dof.m_linearUpperLimit.m_floats[i] = m_linearLimits.m_upperLimit[i];
	}

	dof.m_useLinearReferenceFrameA = m_useLinearReferenceFrameA ? 1 : 0;
	dof.m_useOffsetForConstraintFrame = m_useOffsetForConstraintFrame ? 1 : 0;

	return btGeneric6DofConstraintDataName;
}

#endif



}
