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
2014 May: btGeneric6DofSpring2Constraint is created from the original (2.82.2712) btGeneric6DofConstraint by Gabor Puhr and Tamas Umenhoffer
Pros:
- Much more accurate and stable in a lot of situation. (Especially when a sleeping chain of RBs connected with 6dof2 is pulled)
- Stable and accurate spring with minimal energy loss that works with all of the solvers. (latter is not true for the original 6dof spring)
- Servo motor functionality
- Much more accurate bouncing. 0 really means zero bouncing (not true for the original 6odf) and there is only a minimal energy loss when the value is 1 (because of the solvers' precision)
- Rotation order for the Euler system can be set. (One axis' freedom is still limited to pi/2)

Cons:
- It is slower than the original 6dof. There is no exact ratio, but half speed is a good estimation.
- At bouncing the correct velocity is calculated, but not the correct position. (it is because of the solver can correct position or velocity, but not both.)
*/


/*
2007-09-09
btGeneric6DofConstraint Refactored by Francisco Le?n
email: projectileman@yahoo.com
http://gimpact.sf.net
*/

using Bullet.LinearMath;
using Bullet.Types;
using System.Diagnostics;
/// 2009 March: btGeneric6DofConstraint refactored by Roman Ponomarev
/// Added support for generic constraint solver through getInfo1/getInfo2 methods
namespace Bullet.Dynamics.ConstraintSolver
{
	/*
	#if BT_USE_DOUBLE_PRECISION
	#define btGeneric6DofSpring2ConstraintData2		btGeneric6DofSpring2ConstraintDoubleData2
	#define btGeneric6DofSpring2ConstraintDataName	"btGeneric6DofSpring2ConstraintDoubleData2"
	#else
	#define btGeneric6DofSpring2ConstraintData2		btGeneric6DofSpring2ConstraintData
	#define btGeneric6DofSpring2ConstraintDataName	"btGeneric6DofSpring2ConstraintData"
	#endif //BT_USE_DOUBLE_PRECISION
	*/

	internal enum RotateOrder
	{
		RO_XYZ = 0,
		RO_XZY,
		RO_YXZ,
		RO_YZX,
		RO_ZXY,
		RO_ZYX
	};

	internal class btRotationalLimitMotor2
	{

		// upper < lower means free
		// upper == lower means locked
		// upper > lower means limited
		internal double m_loLimit;
		internal double m_hiLimit;
		internal double m_bounce;
		internal double m_stopERP;
		internal double m_stopCFM;
		internal double m_motorERP;
		internal double m_motorCFM;
		internal bool m_enableMotor;
		internal double m_targetVelocity;
		internal double m_maxMotorForce;
		internal bool m_servoMotor;
		internal double m_servoTarget;
		internal bool m_enableSpring;
		internal double m_springStiffness;
		internal bool m_springStiffnessLimited;
		internal double m_springDamping;
		internal bool m_springDampingLimited;
		internal double m_equilibriumPoint;

		internal double m_currentLimitError;
		internal double m_currentLimitErrorHi;
		internal double m_currentPosition;
		internal int m_currentLimit;

		internal btRotationalLimitMotor2()
		{
			m_loLimit = 1.0f;
			m_hiLimit = -1.0f;
			m_bounce = 0.0f;
			m_stopERP = 0.2f;
			m_stopCFM = 0;
			m_motorERP = 0.9f;
			m_motorCFM = 0;
			m_enableMotor = false;
			m_targetVelocity = 0;
			m_maxMotorForce = 0.1f;
			m_servoMotor = false;
			m_servoTarget = 0;
			m_enableSpring = false;
			m_springStiffness = 0;
			m_springStiffnessLimited = false;
			m_springDamping = 0;
			m_springDampingLimited = false;
			m_equilibriumPoint = 0;

			m_currentLimitError = 0;
			m_currentLimitErrorHi = 0;
			m_currentPosition = 0;
			m_currentLimit = 0;
		}

		internal btRotationalLimitMotor2( btRotationalLimitMotor2 limot )
		{
			m_loLimit = limot.m_loLimit;
			m_hiLimit = limot.m_hiLimit;
			m_bounce = limot.m_bounce;
			m_stopERP = limot.m_stopERP;
			m_stopCFM = limot.m_stopCFM;
			m_motorERP = limot.m_motorERP;
			m_motorCFM = limot.m_motorCFM;
			m_enableMotor = limot.m_enableMotor;
			m_targetVelocity = limot.m_targetVelocity;
			m_maxMotorForce = limot.m_maxMotorForce;
			m_servoMotor = limot.m_servoMotor;
			m_servoTarget = limot.m_servoTarget;
			m_enableSpring = limot.m_enableSpring;
			m_springStiffness = limot.m_springStiffness;
			m_springStiffnessLimited = limot.m_springStiffnessLimited;
			m_springDamping = limot.m_springDamping;
			m_springDampingLimited = limot.m_springDampingLimited;
			m_equilibriumPoint = limot.m_equilibriumPoint;

			m_currentLimitError = limot.m_currentLimitError;
			m_currentLimitErrorHi = limot.m_currentLimitErrorHi;
			m_currentPosition = limot.m_currentPosition;
			m_currentLimit = limot.m_currentLimit;
		}


		internal bool isLimited()
		{
			if( m_loLimit > m_hiLimit ) return false;
			return true;
		}

		internal void testLimitValue( double test_value )
		{
			//we can't normalize the angles here because we would lost the sign that we use later, but it doesn't seem to be a problem
			if( m_loLimit > m_hiLimit )
			{
				m_currentLimit = 0;
				m_currentLimitError = (double)( 0 );
			}
			else if( m_loLimit == m_hiLimit )
			{
				m_currentLimitError = test_value - m_loLimit;
				m_currentLimit = 3;
			}
			else
			{
				m_currentLimitError = test_value - m_loLimit;
				m_currentLimitErrorHi = test_value - m_hiLimit;
				m_currentLimit = 4;
			}
		}

	};



	class btTranslationalLimitMotor2
	{
		// upper < lower means free
		// upper == lower means locked
		// upper > lower means limited
		internal btVector3 m_lowerLimit;
		internal btVector3 m_upperLimit;
		internal btVector3 m_bounce;
		internal btVector3 m_stopERP;
		internal btVector3 m_stopCFM;
		internal btVector3 m_motorERP;
		internal btVector3 m_motorCFM;
		internal bool[] m_enableMotor = new bool[3];
		internal bool[] m_servoMotor = new bool[3];
		internal bool[] m_enableSpring = new bool[3];
		internal btVector3 m_servoTarget;
		internal btVector3 m_springStiffness;
		internal bool[] m_springStiffnessLimited = new bool[3];
		internal btVector3 m_springDamping;
		internal bool[] m_springDampingLimited = new bool[3];
		internal btVector3 m_equilibriumPoint;
		internal btVector3 m_targetVelocity;
		internal btVector3 m_maxMotorForce;

		internal btVector3 m_currentLimitError;
		internal btVector3 m_currentLimitErrorHi;
		internal btVector3 m_currentLinearDiff;
		internal int[] m_currentLimit = new int[3];

		internal btTranslationalLimitMotor2()
		{
			m_lowerLimit.setValue( 0, 0, 0 );
			m_upperLimit.setValue( 0, 0, 0 );
			m_bounce.setValue( 0, 0, 0 );
			m_stopERP.setValue( 0.2f, 0.2f, 0.2f );
			m_stopCFM.setValue( 0, 0, 0 );
			m_motorERP.setValue( 0.9f, 0.9f, 0.9f );
			m_motorCFM.setValue( 0, 0, 0 );

			m_currentLimitError.setValue( 0, 0, 0 );
			m_currentLimitErrorHi.setValue( 0, 0, 0 );
			m_currentLinearDiff.setValue( 0, 0, 0 );

			for( int i = 0; i < 3; i++ )
			{
				m_enableMotor[i] = false;
				m_servoMotor[i] = false;
				m_enableSpring[i] = false;
				m_servoTarget[i] = (double)( 0 );
				m_springStiffness[i] = (double)( 0 );
				m_springStiffnessLimited[i] = false;
				m_springDamping[i] = (double)( 0 );
				m_springDampingLimited[i] = false;
				m_equilibriumPoint[i] = (double)( 0 );
				m_targetVelocity[i] = (double)( 0 );
				m_maxMotorForce[i] = (double)( 0 );

				m_currentLimit[i] = 0;
			}
		}

		internal btTranslationalLimitMotor2( btTranslationalLimitMotor2 other )
		{
			m_lowerLimit = other.m_lowerLimit;
			m_upperLimit = other.m_upperLimit;
			m_bounce = other.m_bounce;
			m_stopERP = other.m_stopERP;
			m_stopCFM = other.m_stopCFM;
			m_motorERP = other.m_motorERP;
			m_motorCFM = other.m_motorCFM;

			m_currentLimitError = other.m_currentLimitError;
			m_currentLimitErrorHi = other.m_currentLimitErrorHi;
			m_currentLinearDiff = other.m_currentLinearDiff;

			for( int i = 0; i < 3; i++ )
			{
				m_enableMotor[i] = other.m_enableMotor[i];
				m_servoMotor[i] = other.m_servoMotor[i];
				m_enableSpring[i] = other.m_enableSpring[i];
				m_servoTarget[i] = other.m_servoTarget[i];
				m_springStiffness[i] = other.m_springStiffness[i];
				m_springStiffnessLimited[i] = other.m_springStiffnessLimited[i];
				m_springDamping[i] = other.m_springDamping[i];
				m_springDampingLimited[i] = other.m_springDampingLimited[i];
				m_equilibriumPoint[i] = other.m_equilibriumPoint[i];
				m_targetVelocity[i] = other.m_targetVelocity[i];
				m_maxMotorForce[i] = other.m_maxMotorForce[i];

				m_currentLimit[i] = other.m_currentLimit[i];
			}
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal bool isLimited( int limitIndex )
		{
			return ( m_upperLimit[limitIndex] >= m_lowerLimit[limitIndex] );
		}

		//////////////////////////// btTranslationalLimitMotor2 ////////////////////////////////////

		internal void testLimitValue( int limitIndex, double test_value )
		{
			double loLimit = m_lowerLimit[limitIndex];
			double hiLimit = m_upperLimit[limitIndex];
			if( loLimit > hiLimit )
			{
				m_currentLimitError[limitIndex] = 0;
				m_currentLimit[limitIndex] = 0;
			}
			else if( loLimit == hiLimit )
			{
				m_currentLimitError[limitIndex] = test_value - loLimit;
				m_currentLimit[limitIndex] = 3;
			}
			else
			{
				m_currentLimitError[limitIndex] = test_value - loLimit;
				m_currentLimitErrorHi[limitIndex] = test_value - hiLimit;
				m_currentLimit[limitIndex] = 4;
			}
		}
	};



	internal class btGeneric6DofSpring2Constraint : btTypedConstraint
	{
		//#define BT_6DOF_FLAGS_AXIS_SHIFT2 4 // bits per axis
		enum bt6DofFlags2
		{
			BT_6DOF_FLAGS_CFM_STOP2 = 1 << 0,
			BT_6DOF_FLAGS_ERP_STOP2 = 1 << 1,
			BT_6DOF_FLAGS_CFM_MOTO2 = 1 << 2,
			BT_6DOF_FLAGS_ERP_MOTO2 = 1 << 3,
			BT_6DOF_FLAGS_CFM_STOP2_Y = 1 << 4,
			BT_6DOF_FLAGS_ERP_STOP2_Y = 1 << 5,
			BT_6DOF_FLAGS_CFM_MOTO2_Y = 1 << 6,
			BT_6DOF_FLAGS_ERP_MOTO2_Y = 1 << 7,
			BT_6DOF_FLAGS_CFM_STOP2_Z = 1 << 8,
			BT_6DOF_FLAGS_ERP_STOP2_Z = 1 << 9,
			BT_6DOF_FLAGS_CFM_MOTO2_Z = 1 << 10,
			BT_6DOF_FLAGS_ERP_MOTO2_Z = 1 << 11,
			BT_6DOF_FLAGS_CFM_STOP2_ROT = 1 << 12,
			BT_6DOF_FLAGS_ERP_STOP2_ROT = 1 << 13,
			BT_6DOF_FLAGS_CFM_MOTO2_ROT = 1 << 14,
			BT_6DOF_FLAGS_ERP_MOTO2_ROT = 1 << 15,
			BT_6DOF_FLAGS_CFM_STOP2_Y_ROT = 1 << 16,
			BT_6DOF_FLAGS_ERP_STOP2_Y_ROT = 1 << 17,
			BT_6DOF_FLAGS_CFM_MOTO2_Y_ROT = 1 << 18,
			BT_6DOF_FLAGS_ERP_MOTO2_Y_ROT = 1 << 19,
			BT_6DOF_FLAGS_CFM_STOP2_Z_ROT = 1 << 20,
			BT_6DOF_FLAGS_ERP_STOP2_Z_ROT = 1 << 21,
			BT_6DOF_FLAGS_CFM_MOTO2_Z_ROT = 1 << 22,
			BT_6DOF_FLAGS_ERP_MOTO2_Z_ROT = 1 << 23,
		};
		const double D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION = 1.0e-3;

		class bt6DofFlagsIndexed
		{
			static internal readonly bt6DofFlags2[] BT_6DOF_FLAGS_CFM_STOP2 = { bt6DofFlags2.BT_6DOF_FLAGS_CFM_STOP2, bt6DofFlags2.BT_6DOF_FLAGS_CFM_STOP2_Y, bt6DofFlags2.BT_6DOF_FLAGS_CFM_STOP2_Z, bt6DofFlags2.BT_6DOF_FLAGS_CFM_STOP2_ROT, bt6DofFlags2.BT_6DOF_FLAGS_CFM_STOP2_Y_ROT, bt6DofFlags2.BT_6DOF_FLAGS_CFM_STOP2_Z_ROT };
			static internal readonly bt6DofFlags2[] BT_6DOF_FLAGS_ERP_STOP2 = { bt6DofFlags2.BT_6DOF_FLAGS_ERP_STOP2, bt6DofFlags2.BT_6DOF_FLAGS_ERP_STOP2_Y, bt6DofFlags2.BT_6DOF_FLAGS_ERP_STOP2_Z, bt6DofFlags2.BT_6DOF_FLAGS_ERP_STOP2_ROT, bt6DofFlags2.BT_6DOF_FLAGS_ERP_STOP2_Y_ROT, bt6DofFlags2.BT_6DOF_FLAGS_ERP_STOP2_Z_ROT };
			static internal readonly bt6DofFlags2[] BT_6DOF_FLAGS_CFM_MOTO2 = { bt6DofFlags2.BT_6DOF_FLAGS_CFM_MOTO2, bt6DofFlags2.BT_6DOF_FLAGS_CFM_MOTO2_Y, bt6DofFlags2.BT_6DOF_FLAGS_CFM_MOTO2_Z, bt6DofFlags2.BT_6DOF_FLAGS_CFM_MOTO2_ROT, bt6DofFlags2.BT_6DOF_FLAGS_CFM_MOTO2_Y_ROT, bt6DofFlags2.BT_6DOF_FLAGS_CFM_MOTO2_Z_ROT };
			static internal readonly bt6DofFlags2[] BT_6DOF_FLAGS_ERP_MOTO2 = { bt6DofFlags2.BT_6DOF_FLAGS_ERP_MOTO2, bt6DofFlags2.BT_6DOF_FLAGS_ERP_MOTO2_Y, bt6DofFlags2.BT_6DOF_FLAGS_ERP_MOTO2_Z, bt6DofFlags2.BT_6DOF_FLAGS_ERP_MOTO2_ROT, bt6DofFlags2.BT_6DOF_FLAGS_ERP_MOTO2_Y_ROT, bt6DofFlags2.BT_6DOF_FLAGS_ERP_MOTO2_Z_ROT };
		}


		btTransform m_frameInA;
		btTransform m_frameInB;

		btJacobianEntry[] m_jacLinear = new btJacobianEntry[3];
		btJacobianEntry[] m_jacAng = new btJacobianEntry[3];

		btTranslationalLimitMotor2 m_linearLimits = new btTranslationalLimitMotor2();
		btRotationalLimitMotor2[] m_angularLimits = new btRotationalLimitMotor2[3];

		RotateOrder m_rotateOrder;


		internal btTransform m_calculatedTransformA;
		internal btTransform m_calculatedTransformB;
		btVector3 m_calculatedAxisAngleDiff;
		btVector3[] m_calculatedAxis = new btVector3[3];
		btVector3 m_calculatedLinearDiff;
		double m_factA;
		double m_factB;
		bool m_hasStaticBody;
		bt6DofFlags2 m_flags;

		/*
		btGeneric6DofSpring2Constraint&	operator=(btGeneric6DofSpring2Constraint&)
	{
		Debug.Assert(false);
		return *this;
	}
	*/
		/*
			int setAngularLimits( btConstraintInfo2* info, int row_offset, ref btTransform transA, ref btTransform transB, ref btVector3 linVelA, ref btVector3 linVelB, ref btVector3 angVelA, ref btVector3 angVelB );
			int setLinearLimits( btConstraintInfo2* info, int row, ref btTransform transA, ref btTransform transB, ref btVector3 linVelA, ref btVector3 linVelB, ref btVector3 angVelA, ref btVector3 angVelB );

			void calculateLinearInfo();
			void calculateAngleInfo();
			void testAngularLimitMotor( int axis_index );

			void calculateJacobi( btRotationalLimitMotor2* limot, ref btTransform transA, ref btTransform transB, btConstraintInfo2* info, int srow, ref btVector3 ax1, int rotational, int rotAllowed );
			int get_limit_motor_info2( btRotationalLimitMotor2* limot,
				ref btTransform transA, ref btTransform transB, ref btVector3 linVelA, ref btVector3 linVelB, ref btVector3 angVelA, ref btVector3 angVelB,
				btConstraintInfo2* info, int row, ref btVector3 ax1, int rotational, int rotAllowed = false );

			static double btGetMatrixElem( btMatrix3x3& mat, int index );
			static bool matrixToEulerXYZ( btMatrix3x3& mat, ref btVector3 xyz );
			static bool matrixToEulerXZY( btMatrix3x3& mat, ref btVector3 xyz );
			static bool matrixToEulerYXZ( btMatrix3x3& mat, ref btVector3 xyz );
			static bool matrixToEulerYZX( btMatrix3x3& mat, ref btVector3 xyz );
			static bool matrixToEulerZXY( btMatrix3x3& mat, ref btVector3 xyz );
			static bool matrixToEulerZYX( btMatrix3x3& mat, ref btVector3 xyz );
			*/



		//btGeneric6DofSpring2Constraint( btRigidBody rbA, btRigidBody rbB, ref btTransform frameInA, ref btTransform frameInB, RotateOrder rotOrder = RO_XYZ );
		//	btGeneric6DofSpring2Constraint( btRigidBody rbB, ref btTransform frameInB, RotateOrder rotOrder = RO_XYZ );

		//virtual void buildJacobian() { }
		//virtual void getInfo1( btConstraintInfo1* info );
		//virtual void getInfo2( btConstraintInfo2* info );
		//virtual int calculateSerializeBufferSize();
		//virtual string serialize( object dataBuffer, btSerializer* serializer );

		internal btRotationalLimitMotor2 getRotationalLimitMotor( int index ) { return m_angularLimits[index]; }
		internal btTranslationalLimitMotor2 getTranslationalLimitMotor() { return m_linearLimits; }

		// Calculates the global transform for the joint offset for body A an B, and also calculates the angle differences between the bodies.
		//void calculateTransforms( ref btTransform transA, ref btTransform transB );
		//void calculateTransforms();

		// Gets the global transform of the offset for body A
		//internal btITransform getCalculatedTransformA() { return m_calculatedTransformA; }
		// Gets the global transform of the offset for body B
		//internal btITransform getCalculatedTransformB() { return m_calculatedTransformB; }

		//internal btITransform getFrameOffsetA() { return m_frameInA; }
		//internal btITransform getFrameOffsetB() { return m_frameInB; }


		// Get the rotation axis in global coordinates ( calculateTransforms() must be called previously )
		internal btVector3 getAxis( int axis_index ) { return m_calculatedAxis[axis_index]; }

		// Get the relative Euler angle ( calculateTransforms() must be called previously )
		internal double getAngle( int axis_index ) { return m_calculatedAxisAngleDiff[axis_index]; }

		// Get the relative position of the constraint pivot ( calculateTransforms() must be called previously )
		internal double getRelativePivotPosition( int axis_index ) { return m_calculatedLinearDiff[axis_index]; }

		//void setFrames( ref btTransform  frameA, ref btTransform  frameB);

		internal void setLinearLowerLimit( ref btVector3 linearLower ) { m_linearLimits.m_lowerLimit = linearLower; }
		internal void getLinearLowerLimit( ref btVector3 linearLower ) { linearLower = m_linearLimits.m_lowerLimit; }
		internal void setLinearUpperLimit( ref btVector3 linearUpper ) { m_linearLimits.m_upperLimit = linearUpper; }
		internal void getLinearUpperLimit( ref btVector3 linearUpper ) { linearUpper = m_linearLimits.m_upperLimit; }

		internal void setAngularLowerLimit( ref btVector3 angularLower )
		{
			for( int i = 0; i < 3; i++ )
				m_angularLimits[i].m_loLimit = btScalar.btNormalizeAngle( angularLower[i] );
		}

		internal void setAngularLowerLimitReversed( ref btVector3 angularLower )
		{
			for( int i = 0; i < 3; i++ )
				m_angularLimits[i].m_hiLimit = btScalar.btNormalizeAngle( -angularLower[i] );
		}

		void getAngularLowerLimit( ref btVector3 angularLower )
		{
			for( int i = 0; i < 3; i++ )
				angularLower[i] = m_angularLimits[i].m_loLimit;
		}

		void getAngularLowerLimitReversed( ref btVector3 angularLower )
		{
			for( int i = 0; i < 3; i++ )
				angularLower[i] = -m_angularLimits[i].m_hiLimit;
		}

		void setAngularUpperLimit( ref btVector3 angularUpper )
		{
			for( int i = 0; i < 3; i++ )
				m_angularLimits[i].m_hiLimit = btScalar.btNormalizeAngle( angularUpper[i] );
		}

		void setAngularUpperLimitReversed( ref btVector3 angularUpper )
		{
			for( int i = 0; i < 3; i++ )
				m_angularLimits[i].m_loLimit = btScalar.btNormalizeAngle( -angularUpper[i] );
		}

		void getAngularUpperLimit( ref btVector3 angularUpper )
		{
			for( int i = 0; i < 3; i++ )
				angularUpper[i] = m_angularLimits[i].m_hiLimit;
		}

		void getAngularUpperLimitReversed( ref btVector3 angularUpper )
		{
			for( int i = 0; i < 3; i++ )
				angularUpper[i] = -m_angularLimits[i].m_loLimit;
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

		void setLimitReversed( int axis, double lo, double hi )
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
				m_angularLimits[axis - 3].m_hiLimit = -lo;
				m_angularLimits[axis - 3].m_loLimit = -hi;
			}
		}

		bool isLimited( int limitIndex )
		{
			if( limitIndex < 3 )
			{
				return m_linearLimits.isLimited( limitIndex );
			}
			return m_angularLimits[limitIndex - 3].isLimited();
		}

		void setRotationOrder( RotateOrder order ) { m_rotateOrder = order; }
		RotateOrder getRotationOrder() { return m_rotateOrder; }
		/*
		void setAxis( ref btVector3 axis1, ref btVector3 axis2 );

		void setBounce( int index, double bounce );

		void enableMotor( int index, bool onOff );
		void setServo( int index, bool onOff ); // set the type of the motor (servo or not) (the motor has to be turned on for servo also)
		void setTargetVelocity( int index, double velocity );
		void setServoTarget( int index, double target );
		void setMaxMotorForce( int index, double force );

		void enableSpring( int index, bool onOff );
		void setStiffness( int index, double stiffness, bool limitIfNeeded = true ); // if limitIfNeeded is true the system will automatically limit the stiffness in necessary situations where otherwise the spring would move unrealistically too widely
		void setDamping( int index, double damping, bool limitIfNeeded = true ); // if limitIfNeeded is true the system will automatically limit the damping in necessary situations where otherwise the spring would blow up
		void setEquilibriumPoint(); // set the current constraint position/orientation as an equilibrium point for all DOF
		void setEquilibriumPoint( int index );  // set the current constraint position/orientation as an equilibrium point for given DOF
		void setEquilibriumPoint( int index, double val );
		*/

		//override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
		//If no axis is provided, it uses the default axis for this constraint.


		internal btGeneric6DofSpring2Constraint( btRigidBody rbA, btRigidBody rbB, ref btTransform frameInA, ref btTransform frameInB, RotateOrder rotOrder )
			: base( btObjectTypes.D6_SPRING_2_CONSTRAINT_TYPE, rbA, rbB )
		{
			m_frameInA = ( frameInA );
			m_frameInB = ( frameInB );
			m_rotateOrder = ( rotOrder );
			m_flags = ( 0 );
			calculateTransforms();
		}


		internal btGeneric6DofSpring2Constraint( btRigidBody rbB, ref btTransform frameInB, RotateOrder rotOrder )
			: base( btObjectTypes.D6_SPRING_2_CONSTRAINT_TYPE, getFixedBody(), rbB )
		{
			m_frameInB = ( frameInB );
			m_rotateOrder = ( rotOrder );
			m_flags = ( 0 );
			///not providing rigidbody A means implicitly using worldspace for body A
			rbB.m_worldTransform.Apply( ref m_frameInB, out m_frameInA );
			calculateTransforms();
		}

		
		static double btGetMatrixElem( ref btMatrix3x3 mat, int index )
		{
			int i = index % 3;
			int j = index / 3;
			return btMatrix3x3.getValue( ref mat, i,j );
		}

		// MatrixToEulerXYZ from http://www.geometrictools.com/LibFoundation/Mathematics/Wm4Matrix3.inl.html

		static bool matrixToEulerXYZ( ref btMatrix3x3 mat, ref btVector3 xyz )
		{
			// rot =  cy*cz          -cy*sz           sy
			//        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
			//       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy

			double fi = btGetMatrixElem( ref mat, 2 );
			if( fi < (double)( 1.0f ) )
			{
				if( fi > (double)( -1.0f ) )
				{
					xyz[0] = btScalar.btAtan2( -btGetMatrixElem( ref mat, 5 ), btGetMatrixElem( ref mat, 8 ) );
					xyz[1] = btScalar.btAsin( btGetMatrixElem( ref mat, 2 ) );
					xyz[2] = btScalar.btAtan2( -btGetMatrixElem( ref mat, 1 ), btGetMatrixElem( ref mat, 0 ) );
					return true;
				}
				else
				{
					// WARNING.  Not unique.  XA - ZA = -atan2(r10,r11)
					xyz[0] = -btScalar.btAtan2( btGetMatrixElem( ref mat, 3 ), btGetMatrixElem( ref mat, 4 ) );
					xyz[1] = -btScalar.SIMD_HALF_PI;
					xyz[2] = (double)( 0.0 );
					return false;
				}
			}
			else
			{
				// WARNING.  Not unique.  XAngle + ZAngle = atan2(r10,r11)
				xyz[0] = btScalar.btAtan2( btGetMatrixElem( ref mat, 3 ), btGetMatrixElem( ref mat, 4 ) );
				xyz[1] = btScalar.SIMD_HALF_PI;
				xyz[2] = 0.0;
			}
			return false;
		}

		bool matrixToEulerXZY( ref btMatrix3x3 mat, ref btVector3 xyz )
		{
			// rot =  cy*cz          -sz           sy*cz
			//        cy*cx*sz+sx*sy  cx*cz        sy*cx*sz-cy*sx
			//        cy*sx*sz-cx*sy  sx*cz        sy*sx*sz+cx*cy

			double fi = btGetMatrixElem( ref mat, 1 );
			if( fi < (double)( 1.0f ) )
			{
				if( fi > (double)( -1.0f ) )
				{
					xyz[0] = btScalar.btAtan2( btGetMatrixElem( ref mat, 7 ), btGetMatrixElem( ref mat, 4 ) );
					xyz[1] = btScalar.btAtan2( btGetMatrixElem( ref mat, 2 ), btGetMatrixElem( ref mat, 0 ) );
					xyz[2] = btScalar.btAsin( -btGetMatrixElem( ref mat, 1 ) );
					return true;
				}
				else
				{
					xyz[0] = -btScalar.btAtan2( -btGetMatrixElem( ref mat, 6 ), btGetMatrixElem( ref mat, 8 ) );
					xyz[1] = (double)( 0.0 );
					xyz[2] = btScalar.SIMD_HALF_PI;
					return false;
				}
			}
			else
			{
				xyz[0] = btScalar.btAtan2( -btGetMatrixElem( ref mat, 6 ), btGetMatrixElem( ref mat, 8 ) );
				xyz[1] = 0.0;
				xyz[2] = -btScalar.SIMD_HALF_PI;
			}
			return false;
		}

		bool matrixToEulerYXZ( ref btMatrix3x3 mat, ref btVector3 xyz )
		{
			// rot =  cy*cz+sy*sx*sz  cz*sy*sx-cy*sz  cx*sy
			//        cx*sz           cx*cz           -sx
			//        cy*sx*sz-cz*sy  sy*sz+cy*cz*sx  cy*cx

			double fi = btGetMatrixElem( ref mat, 5 );
			if( fi < (double)( 1.0f ) )
			{
				if( fi > (double)( -1.0f ) )
				{
					xyz[0] = btScalar.btAsin( -btGetMatrixElem( ref mat, 5 ) );
					xyz[1] = btScalar.btAtan2( btGetMatrixElem( ref mat, 2 ), btGetMatrixElem( ref mat, 8 ) );
					xyz[2] = btScalar.btAtan2( btGetMatrixElem( ref mat, 3 ), btGetMatrixElem( ref mat, 4 ) );
					return true;
				}
				else
				{
					xyz[0] = btScalar.SIMD_HALF_PI;
					xyz[1] = -btScalar.btAtan2( -btGetMatrixElem( ref mat, 1 ), btGetMatrixElem( ref mat, 0 ) );
					xyz[2] = (double)( 0.0 );
					return false;
				}
			}
			else
			{
				xyz[0] = -btScalar.SIMD_HALF_PI;
				xyz[1] = btScalar.btAtan2( -btGetMatrixElem( ref mat, 1 ), btGetMatrixElem( ref mat, 0 ) );
				xyz[2] = 0.0;
			}
			return false;
		}

		bool matrixToEulerYZX( ref btMatrix3x3 mat, ref btVector3 xyz )
		{
			// rot =  cy*cz   sy*sx-cy*cx*sz   cx*sy+cy*sz*sx
			//        sz           cz*cx           -cz*sx
			//        -cz*sy  cy*sx+cx*sy*sz   cy*cx-sy*sz*sx

			double fi = btGetMatrixElem( ref mat, 3 );
			if( fi < (double)( 1.0f ) )
			{
				if( fi > (double)( -1.0f ) )
				{
					xyz[0] = btScalar.btAtan2( -btGetMatrixElem( ref mat, 5 ), btGetMatrixElem( ref mat, 4 ) );
					xyz[1] = btScalar.btAtan2( -btGetMatrixElem( ref mat, 6 ), btGetMatrixElem( ref mat, 0 ) );
					xyz[2] = btScalar.btAsin( btGetMatrixElem( ref mat, 3 ) );
					return true;
				}
				else
				{
					xyz[0] = (double)( 0.0 );
					xyz[1] = -btScalar.btAtan2( btGetMatrixElem( ref mat, 7 ), btGetMatrixElem( ref mat, 8 ) );
					xyz[2] = -btScalar.SIMD_HALF_PI;
					return false;
				}
			}
			else
			{
				xyz[0] = (double)( 0.0 );
				xyz[1] = btScalar.btAtan2( btGetMatrixElem( ref mat, 7 ), btGetMatrixElem( ref mat, 8 ) );
				xyz[2] = btScalar.SIMD_HALF_PI;
			}
			return false;
		}

		bool matrixToEulerZXY( ref btMatrix3x3 mat, ref btVector3 xyz )
		{
			// rot =  cz*cy-sz*sx*sy    -cx*sz   cz*sy+cy*sz*sx
			//        cy*sz+cz*sx*sy     cz*cx   sz*sy-cz*xy*sx
			//        -cx*sy              sx     cx*cy

			double fi = btGetMatrixElem( ref mat, 7 );
			if( fi < (double)( 1.0f ) )
			{
				if( fi > (double)( -1.0f ) )
				{
					xyz[0] = btScalar.btAsin( btGetMatrixElem( ref mat, 7 ) );
					xyz[1] = btScalar.btAtan2( -btGetMatrixElem( ref mat, 6 ), btGetMatrixElem( ref mat, 8 ) );
					xyz[2] = btScalar.btAtan2( -btGetMatrixElem( ref mat, 1 ), btGetMatrixElem( ref mat, 4 ) );
					return true;
				}
				else
				{
					xyz[0] = -btScalar.SIMD_HALF_PI;
					xyz[1] = (double)( 0.0 );
					xyz[2] = -btScalar.btAtan2( btGetMatrixElem( ref mat, 2 ), btGetMatrixElem( ref mat, 0 ) );
					return false;
				}
			}
			else
			{
				xyz[0] = btScalar.SIMD_HALF_PI;
				xyz[1] = (double)( 0.0 );
				xyz[2] = btScalar.btAtan2( btGetMatrixElem( ref mat, 2 ), btGetMatrixElem( ref mat, 0 ) );
			}
			return false;
		}

		bool matrixToEulerZYX( ref btMatrix3x3 mat, ref btVector3 xyz )
		{
			// rot =  cz*cy   cz*sy*sx-cx*sz   sz*sx+cz*cx*sy
			//        cy*sz   cz*cx+sz*sy*sx   cx*sz*sy-cz*sx
			//        -sy          cy*sx         cy*cx

			double fi = btGetMatrixElem( ref mat, 6 );
			if( fi < (double)( 1.0f ) )
			{
				if( fi > (double)( -1.0f ) )
				{
					xyz[0] = btScalar.btAtan2( btGetMatrixElem( ref mat, 7 ), btGetMatrixElem( ref mat, 8 ) );
					xyz[1] = btScalar.btAsin( -btGetMatrixElem( ref mat, 6 ) );
					xyz[2] = btScalar.btAtan2( btGetMatrixElem( ref mat, 3 ), btGetMatrixElem( ref mat, 0 ) );
					return true;
				}
				else
				{
					xyz[0] = (double)( 0.0 );
					xyz[1] = btScalar.SIMD_HALF_PI;
					xyz[2] = -btScalar.btAtan2( btGetMatrixElem( ref mat, 1 ), btGetMatrixElem( ref mat, 2 ) );
					return false;
				}
			}
			else
			{
				xyz[0] = (double)( 0.0 );
				xyz[1] = -btScalar.SIMD_HALF_PI;
				xyz[2] = btScalar.btAtan2( -btGetMatrixElem( ref mat, 1 ), -btGetMatrixElem( ref mat, 2 ) );
			}
			return false;
		}

		void calculateAngleInfo()
		{
			btMatrix3x3 inv;
			m_calculatedTransformA.m_basis.inverse( out inv );
			btMatrix3x3 relative_frame; inv.Apply( ref m_calculatedTransformB.m_basis, out relative_frame );
			switch( m_rotateOrder )
			{
				case RotateOrder.RO_XYZ: matrixToEulerXYZ( ref relative_frame, ref m_calculatedAxisAngleDiff ); break;
				case RotateOrder.RO_XZY: matrixToEulerXZY( ref relative_frame, ref m_calculatedAxisAngleDiff ); break;
				case RotateOrder.RO_YXZ: matrixToEulerYXZ( ref relative_frame, ref m_calculatedAxisAngleDiff ); break;
				case RotateOrder.RO_YZX: matrixToEulerYZX( ref relative_frame, ref m_calculatedAxisAngleDiff ); break;
				case RotateOrder.RO_ZXY: matrixToEulerZXY( ref relative_frame, ref m_calculatedAxisAngleDiff ); break;
				case RotateOrder.RO_ZYX: matrixToEulerZYX( ref relative_frame, ref m_calculatedAxisAngleDiff ); break;
				default:
					Debug.Assert( false );
					break;
			}
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
			switch( m_rotateOrder )
			{
				case RotateOrder.RO_XYZ:
					{
						//Is this the "line of nodes" calculation choosing planes YZ (B coordinate system) and xy (A coordinate system)? (http://en.wikipedia.org/wiki/Euler_angles)
						//The two planes are non-homologous, so this is a Tait–Bryan angle formalism and not a proper Euler
						//Extrinsic rotations are equal to the reversed order intrinsic rotations so the above xyz extrinsic rotations (axes are fixed) are the same as the zy'x" intrinsic rotations (axes are refreshed after each rotation)
						//that is why xy and YZ planes are chosen (this will describe a zy'x" intrinsic rotation) (see the figure on the left at http://en.wikipedia.org/wiki/Euler_angles under Tait–Bryan angles)
						// x' = Nperp = N.cross(axis2)
						// y' = N = axis2.cross(axis0)	
						// z' = z
						//
						// x" = X
						// y" = y'
						// z" = ??
						//in other words:
						//first rotate around z
						//second rotate around y'= z.cross(X)
						//third rotate around x" = X
						//Original XYZ extrinsic rotation order. 
						//Planes: xy and YZ normals: z, X.  Plane intersection (N) is z.cross(X)
						btVector3 axis0 = m_calculatedTransformB.m_basis.getColumn( 0 );
						btVector3 axis2 = m_calculatedTransformA.m_basis.getColumn( 2 );
						m_calculatedAxis[1] = axis2.cross( axis0 );
						m_calculatedAxis[0] = m_calculatedAxis[1].cross( axis2 );
						m_calculatedAxis[2] = axis0.cross( m_calculatedAxis[1] );
						break;
					}
				case RotateOrder.RO_XZY:
					{
						//planes: xz,ZY normals: y, X
						//first rotate around y
						//second rotate around z'= y.cross(X)
						//third rotate around x" = X
						btVector3 axis0 = m_calculatedTransformB.m_basis.getColumn( 0 );
						btVector3 axis1 = m_calculatedTransformA.m_basis.getColumn( 1 );
						m_calculatedAxis[2] = axis0.cross( axis1 );
						m_calculatedAxis[0] = axis1.cross( m_calculatedAxis[2] );
						m_calculatedAxis[1] = m_calculatedAxis[2].cross( axis0 );
						break;
					}
				case RotateOrder.RO_YXZ:
					{
						//planes: yx,XZ normals: z, Y
						//first rotate around z
						//second rotate around x'= z.cross(Y)
						//third rotate around y" = Y
						btVector3 axis1 = m_calculatedTransformB.m_basis.getColumn( 1 );
						btVector3 axis2 = m_calculatedTransformA.m_basis.getColumn( 2 );
						m_calculatedAxis[0] = axis1.cross( axis2 );
						m_calculatedAxis[1] = axis2.cross( m_calculatedAxis[0] );
						m_calculatedAxis[2] = m_calculatedAxis[0].cross( axis1 );
						break;
					}
				case RotateOrder.RO_YZX:
					{
						//planes: yz,ZX normals: x, Y
						//first rotate around x
						//second rotate around z'= x.cross(Y)
						//third rotate around y" = Y
						btVector3 axis0 = m_calculatedTransformA.m_basis.getColumn( 0 );
						btVector3 axis1 = m_calculatedTransformB.m_basis.getColumn( 1 );
						m_calculatedAxis[2] = axis0.cross( axis1 );
						m_calculatedAxis[0] = axis1.cross( m_calculatedAxis[2] );
						m_calculatedAxis[1] = m_calculatedAxis[2].cross( axis0 );
						break;
					}
				case RotateOrder.RO_ZXY:
					{
						//planes: zx,XY normals: y, Z
						//first rotate around y
						//second rotate around x'= y.cross(Z)
						//third rotate around z" = Z
						btVector3 axis1 = m_calculatedTransformA.m_basis.getColumn( 1 );
						btVector3 axis2 = m_calculatedTransformB.m_basis.getColumn( 2 );
						m_calculatedAxis[0] = axis1.cross( axis2 );
						m_calculatedAxis[1] = axis2.cross( m_calculatedAxis[0] );
						m_calculatedAxis[2] = m_calculatedAxis[0].cross( axis1 );
						break;
					}
				case RotateOrder.RO_ZYX:
					{
						//planes: zy,YX normals: x, Z
						//first rotate around x
						//second rotate around y' = x.cross(Z)
						//third rotate around z" = Z
						btVector3 axis0 = m_calculatedTransformA.m_basis.getColumn( 0 );
						btVector3 axis2 = m_calculatedTransformB.m_basis.getColumn( 2 );
						m_calculatedAxis[1] = axis2.cross( axis0 );
						m_calculatedAxis[0] = m_calculatedAxis[1].cross( axis2 );
						m_calculatedAxis[2] = axis0.cross( m_calculatedAxis[1] );
						break;
					}
				default:
					Debug.Assert( false );
					break;
			}

			m_calculatedAxis[0].normalize();
			m_calculatedAxis[1].normalize();
			m_calculatedAxis[2].normalize();

		}

		void calculateTransforms()
		{
			calculateTransforms( ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform );
		}

		void calculateTransforms( ref btTransform transA, ref btTransform transB )
		{
			transA.Apply( ref m_frameInA, out m_calculatedTransformA );
			transB.Apply( ref m_frameInB, out m_calculatedTransformB );
			calculateLinearInfo();
			calculateAngleInfo();

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


		void testAngularLimitMotor( int axis_index )
		{
			double angle = m_calculatedAxisAngleDiff[axis_index];
			angle = btAdjustAngleToLimits( angle, m_angularLimits[axis_index].m_loLimit, m_angularLimits[axis_index].m_hiLimit );
			m_angularLimits[axis_index].m_currentPosition = angle;
			m_angularLimits[axis_index].testLimitValue( angle );
		}


		internal override void getInfo1( ref btConstraintInfo1 info )
		{
			//prepare constraint
			calculateTransforms( ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform );
			info.m_numConstraintRows = 0;
			info.nub = 0;
			int i;
			//test linear limits
			for( i = 0; i < 3; i++ )
			{
				if( m_linearLimits.m_currentLimit[i] == 4 ) info.m_numConstraintRows += 2;
				else if( m_linearLimits.m_currentLimit[i] != 0 ) info.m_numConstraintRows += 1;
				if( m_linearLimits.m_enableMotor[i] ) info.m_numConstraintRows += 1;
				if( m_linearLimits.m_enableSpring[i] ) info.m_numConstraintRows += 1;
			}
			//test angular limits
			for( i = 0; i < 3; i++ )
			{
				testAngularLimitMotor( i );
				if( m_angularLimits[i].m_currentLimit == 4 ) info.m_numConstraintRows += 2;
				else if( m_angularLimits[i].m_currentLimit != 0 ) info.m_numConstraintRows += 1;
				if( m_angularLimits[i].m_enableMotor ) info.m_numConstraintRows += 1;
				if( m_angularLimits[i].m_enableSpring ) info.m_numConstraintRows += 1;
			}
		}


		internal override void getInfo2( btConstraintInfo2 info )
		{
			btTransform transA = m_rbA.m_worldTransform;
			btTransform transB = m_rbB.m_worldTransform;
			btVector3 linVelA = m_rbA.m_linearVelocity;
			btVector3 linVelB = m_rbB.m_linearVelocity;
			btVector3 angVelA = m_rbA.m_angularVelocity;
			btVector3 angVelB = m_rbB.m_angularVelocity;

			// for stability better to solve angular limits first
			int row = setAngularLimits( info, 0, ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform
				, ref m_rbA.m_linearVelocity, ref m_rbB.m_linearVelocity
				, ref m_rbA.m_angularVelocity, ref m_rbB.m_angularVelocity );
			setLinearLimits( info, row, ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform
				, ref m_rbA.m_linearVelocity, ref m_rbB.m_linearVelocity
				, ref m_rbA.m_angularVelocity, ref m_rbB.m_angularVelocity );
		}


		int setLinearLimits( btConstraintInfo2 info, int row, ref btTransform transA, ref btTransform transB, ref btVector3 linVelA, ref btVector3 linVelB, ref btVector3 angVelA, ref btVector3 angVelB )
		{
			//solve linear limits
			btRotationalLimitMotor2 limot = new btRotationalLimitMotor2();
			for( int i = 0; i < 3; i++ )
			{
				if( m_linearLimits.m_currentLimit[i] != 0 || m_linearLimits.m_enableMotor[i] || m_linearLimits.m_enableSpring[i] )
				{ // re-use rotational motor code
					limot.m_bounce = m_linearLimits.m_bounce[i];
					limot.m_currentLimit = m_linearLimits.m_currentLimit[i];
					limot.m_currentPosition = m_linearLimits.m_currentLinearDiff[i];
					limot.m_currentLimitError = m_linearLimits.m_currentLimitError[i];
					limot.m_currentLimitErrorHi = m_linearLimits.m_currentLimitErrorHi[i];
					limot.m_enableMotor = m_linearLimits.m_enableMotor[i];
					limot.m_servoMotor = m_linearLimits.m_servoMotor[i];
					limot.m_servoTarget = m_linearLimits.m_servoTarget[i];
					limot.m_enableSpring = m_linearLimits.m_enableSpring[i];
					limot.m_springStiffness = m_linearLimits.m_springStiffness[i];
					limot.m_springStiffnessLimited = m_linearLimits.m_springStiffnessLimited[i];
					limot.m_springDamping = m_linearLimits.m_springDamping[i];
					limot.m_springDampingLimited = m_linearLimits.m_springDampingLimited[i];
					limot.m_equilibriumPoint = m_linearLimits.m_equilibriumPoint[i];
					limot.m_hiLimit = m_linearLimits.m_upperLimit[i];
					limot.m_loLimit = m_linearLimits.m_lowerLimit[i];
					limot.m_maxMotorForce = m_linearLimits.m_maxMotorForce[i];
					limot.m_targetVelocity = m_linearLimits.m_targetVelocity[i];
					btVector3 axis = m_calculatedTransformA.m_basis.getColumn( i );
					//int flags = m_flags >> ( i * BT_6DOF_FLAGS_AXIS_SHIFT2 );
					limot.m_stopCFM = ( m_flags & bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_STOP2[i] ) != 0 ? m_linearLimits.m_stopCFM[i] : info.m_solverConstraints[0].m_cfm;
					limot.m_stopERP = ( m_flags & bt6DofFlagsIndexed.BT_6DOF_FLAGS_ERP_STOP2[i] ) != 0 ? m_linearLimits.m_stopERP[i] : info.erp;
					limot.m_motorCFM = ( m_flags & bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_MOTO2[i] ) != 0 ? m_linearLimits.m_motorCFM[i] : info.m_solverConstraints[0].m_cfm;
					limot.m_motorERP = ( m_flags & bt6DofFlagsIndexed.BT_6DOF_FLAGS_ERP_MOTO2[i] ) != 0 ? m_linearLimits.m_motorERP[i] : info.erp;

					//rotAllowed is a bit of a magic from the original 6dof. The calculation of it here is something that imitates the original behavior as much as possible.
					int indx1 = ( i + 1 ) % 3;
					int indx2 = ( i + 2 ) % 3;
					bool rotAllowed = true; // rotations around orthos to current axis (it is used only when one of the body is static)
					bool indx1Violated = m_angularLimits[indx1].m_currentLimit == 1 ||
						m_angularLimits[indx1].m_currentLimit == 2 ||
						( m_angularLimits[indx1].m_currentLimit == 3 && ( m_angularLimits[indx1].m_currentLimitError < -D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION || m_angularLimits[indx1].m_currentLimitError > D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION ) ) ||
						( m_angularLimits[indx1].m_currentLimit == 4 && ( m_angularLimits[indx1].m_currentLimitError < -D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION || m_angularLimits[indx1].m_currentLimitErrorHi > D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION ) );
					bool indx2Violated = m_angularLimits[indx2].m_currentLimit == 1 ||
						m_angularLimits[indx2].m_currentLimit == 2 ||
						( m_angularLimits[indx2].m_currentLimit == 3 && ( m_angularLimits[indx2].m_currentLimitError < -D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION || m_angularLimits[indx2].m_currentLimitError > D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION ) ) ||
						( m_angularLimits[indx2].m_currentLimit == 4 && ( m_angularLimits[indx2].m_currentLimitError < -D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION || m_angularLimits[indx2].m_currentLimitErrorHi > D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION ) );
					if( indx1Violated && indx2Violated )
					{
						rotAllowed = false;
					}
					row += get_limit_motor_info2( limot, ref transA, ref transB
						, ref linVelA, ref linVelB
						, ref angVelA, ref angVelB
						, info, row, ref axis, false, rotAllowed );
				}
			}
			return row;
		}



		int setAngularLimits( btConstraintInfo2 info, int row_offset, ref btTransform transA, ref btTransform transB, ref btVector3 linVelA, ref btVector3 linVelB, ref btVector3 angVelA, ref btVector3 angVelB )
		{
			int row = row_offset;

			//order of rotational constraint rows
			int[] cIdx = { 0, 1, 2 };
			switch( m_rotateOrder )
			{
				case RotateOrder.RO_XYZ: cIdx[0] = 0; cIdx[1] = 1; cIdx[2] = 2; break;
				case RotateOrder.RO_XZY: cIdx[0] = 0; cIdx[1] = 2; cIdx[2] = 1; break;
				case RotateOrder.RO_YXZ: cIdx[0] = 1; cIdx[1] = 0; cIdx[2] = 2; break;
				case RotateOrder.RO_YZX: cIdx[0] = 1; cIdx[1] = 2; cIdx[2] = 0; break;
				case RotateOrder.RO_ZXY: cIdx[0] = 2; cIdx[1] = 0; cIdx[2] = 1; break;
				case RotateOrder.RO_ZYX: cIdx[0] = 2; cIdx[1] = 1; cIdx[2] = 0; break;
				default: Debug.Assert( false ); break;
			}

			for( int ii = 0; ii < 3; ii++ )
			{
				int i = cIdx[ii];
				if( m_angularLimits[i].m_currentLimit != 0 || m_angularLimits[i].m_enableMotor || m_angularLimits[i].m_enableSpring )
				{
					btVector3 axis = getAxis( i );
					//int flags = m_flags >> ( ( i + 3 ) * BT_6DOF_FLAGS_AXIS_SHIFT2 );
					if( 0 == ( m_flags & bt6DofFlagsIndexed.BT_6DOF_FLAGS_ERP_STOP2[i] ) )
					{
						m_angularLimits[i].m_stopCFM = info.m_solverConstraints[0].m_cfm;
					}
					if( 0 == ( m_flags & bt6DofFlagsIndexed.BT_6DOF_FLAGS_ERP_STOP2[i] ) )
					{
						m_angularLimits[i].m_stopERP = info.erp;
					}
					if( 0 == ( m_flags & bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_MOTO2[i] ) )
					{
						m_angularLimits[i].m_motorCFM = info.m_solverConstraints[0].m_cfm;
					}
					if( 0 == ( m_flags & bt6DofFlagsIndexed.BT_6DOF_FLAGS_ERP_MOTO2[i] ) )
					{
						m_angularLimits[i].m_motorERP = info.erp;
					}
					row += get_limit_motor_info2( m_angularLimits[i], ref transA, ref transB
						, ref linVelA, ref linVelB, ref angVelA, ref angVelB
						, info, row, ref axis, true );
				}
			}

			return row;
		}


		void setFrames( ref btTransform frameA, ref btTransform frameB )
		{
			m_frameInA = frameA;
			m_frameInB = frameB;
			buildJacobian();
			calculateTransforms();
		}


		void calculateLinearInfo()
		{
			btVector3 tmp;
			m_calculatedTransformB.m_origin.Sub( ref m_calculatedTransformA.m_origin, out tmp );
			btMatrix3x3 tmpm;
			m_calculatedTransformA.m_basis.inverse( out tmpm );
			tmpm.Apply( ref tmp, out m_calculatedLinearDiff );
			for( int i = 0; i < 3; i++ )
			{
				m_linearLimits.m_currentLinearDiff[i] = m_calculatedLinearDiff[i];
				m_linearLimits.testLimitValue( i, m_calculatedLinearDiff[i] );
			}
		}

		void calculateJacobi( btRotationalLimitMotor2 limot, ref btTransform transA, ref btTransform transB, btConstraintInfo2 info, int row, ref btVector3 ax1, bool rotational, bool rotAllowed )
		{

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

			if( !rotational )
			{
				btVector3 tmpA, tmpB, relA, relB;
				// get vector from bodyB to frameB in WCS
				m_calculatedTransformB.m_origin.Sub( ref transB.m_origin, out relB );
				// same for bodyA
				m_calculatedTransformA.m_origin.Sub( ref transA.m_origin, out relA ) ;
				relA.cross( ref ax1, out tmpA );
				relB.cross( ref ax1, out tmpB );
				if( m_hasStaticBody && ( !rotAllowed ) )
				{
					tmpA *= m_factA;
					tmpB *= m_factB;
				}
				//int i;
				info.m_solverConstraints[row].m_relpos1CrossNormal = tmpA;
				tmpB.Invert( out info.m_solverConstraints[row].m_relpos2CrossNormal );
			}
		}


		int get_limit_motor_info2(
			btRotationalLimitMotor2 limot,
			ref btTransform transA, ref btTransform transB, ref btVector3 linVelA, ref btVector3 linVelB, ref btVector3 angVelA, ref btVector3 angVelB,
			btConstraintInfo2 info, int row, ref btVector3 ax1, bool rotational, bool rotAllowed = false )
		{
			int count = 0;
			//int srow = row * info.rowskip;

			if( limot.m_currentLimit == 4 )
			{
				double vel = rotational ? angVelA.dot( ax1 ) - angVelB.dot( ax1 ) : linVelA.dot( ax1 ) - linVelB.dot( ax1 );

				calculateJacobi( limot, ref transA, ref transB, info, row, ref ax1, rotational, rotAllowed );
				info.m_solverConstraints[row].m_rhs = info.fps * limot.m_stopERP * limot.m_currentLimitError * ( rotational ? -1 : 1 );
				if( rotational )
				{
					if( info.m_solverConstraints[row].m_rhs - vel * limot.m_stopERP > 0 )
					{
						double bounceerror = -limot.m_bounce * vel;
						if( bounceerror > info.m_solverConstraints[row].m_rhs ) info.m_solverConstraints[row].m_rhs = bounceerror;
					}
				}
				else
				{
					if( info.m_solverConstraints[row].m_rhs - vel * limot.m_stopERP < 0 )
					{
						double bounceerror = -limot.m_bounce * vel;
						if( bounceerror < info.m_solverConstraints[row].m_rhs ) info.m_solverConstraints[row].m_rhs = bounceerror;
					}
				}
				info.m_solverConstraints[row].m_lowerLimit = rotational ? 0 : btScalar.BT_MIN_FLOAT;
				info.m_solverConstraints[row].m_upperLimit = rotational ? btScalar.BT_MAX_FLOAT : 0;
				info.m_solverConstraints[row].m_cfm = limot.m_stopCFM;
				row ++;
				++count;

				calculateJacobi( limot, ref transA, ref transB, info, row, ref ax1, rotational, rotAllowed );
				info.m_solverConstraints[row].m_rhs = info.fps * limot.m_stopERP * limot.m_currentLimitErrorHi * ( rotational ? -1 : 1 );
				if( rotational )
				{
					if( info.m_solverConstraints[row].m_rhs - vel * limot.m_stopERP < 0 )
					{
						double bounceerror = -limot.m_bounce * vel;
						if( bounceerror < info.m_solverConstraints[row].m_rhs ) info.m_solverConstraints[row].m_rhs = bounceerror;
					}
				}
				else
				{
					if( info.m_solverConstraints[row].m_rhs - vel * limot.m_stopERP > 0 )
					{
						double bounceerror = -limot.m_bounce * vel;
						if( bounceerror > info.m_solverConstraints[row].m_rhs ) info.m_solverConstraints[row].m_rhs = bounceerror;
					}
				}
				info.m_solverConstraints[row].m_lowerLimit = rotational ? btScalar.BT_MIN_FLOAT : 0;
				info.m_solverConstraints[row].m_upperLimit = rotational ? 0 : btScalar.BT_MAX_FLOAT;
				info.m_solverConstraints[row].m_cfm = limot.m_stopCFM;
				row ++;
				++count;
			}
			else
			if( limot.m_currentLimit == 3 )
			{
				calculateJacobi( limot, ref transA, ref transB, info, row, ref ax1, rotational, rotAllowed );
				info.m_solverConstraints[row].m_rhs = info.fps * limot.m_stopERP * limot.m_currentLimitError * ( rotational ? -1 : 1 );
				info.m_solverConstraints[row].m_lowerLimit = btScalar.BT_MIN_FLOAT;
				info.m_solverConstraints[row].m_upperLimit = btScalar.BT_MAX_FLOAT;
				info.m_solverConstraints[row].m_cfm = limot.m_stopCFM;
				row++;
				++count;
			}

			if( limot.m_enableMotor && !limot.m_servoMotor )
			{
				calculateJacobi( limot, ref transA, ref transB, info, row, ref ax1, rotational, rotAllowed );
				double tag_vel = rotational ? limot.m_targetVelocity : -limot.m_targetVelocity;
				double mot_fact = getMotorFactor( limot.m_currentPosition,
					limot.m_loLimit,
					limot.m_hiLimit,
					tag_vel,
					info.fps * limot.m_motorERP );
				info.m_solverConstraints[row].m_rhs = mot_fact * limot.m_targetVelocity;
				info.m_solverConstraints[row].m_lowerLimit= -limot.m_maxMotorForce;
				info.m_solverConstraints[row].m_upperLimit= limot.m_maxMotorForce;
				info.m_solverConstraints[row].m_cfm = limot.m_motorCFM;
				row++;
				++count;
			}

			if( limot.m_enableMotor && limot.m_servoMotor )
			{
				double error = limot.m_currentPosition - limot.m_servoTarget;
				calculateJacobi( limot, ref transA, ref transB, info, row, ref ax1, rotational, rotAllowed );
				double targetvelocity = error < 0 ? -limot.m_targetVelocity : limot.m_targetVelocity;
				double tag_vel = -targetvelocity;
				double mot_fact;
				if( error != 0 )
				{
					double lowLimit;
					double hiLimit;
					if( limot.m_loLimit > limot.m_hiLimit )
					{
						lowLimit = error > 0 ? limot.m_servoTarget : btScalar.BT_MIN_FLOAT;
						hiLimit = error < 0 ? limot.m_servoTarget : btScalar.BT_MAX_FLOAT;
					}
					else
					{
						lowLimit = error > 0 && limot.m_servoTarget > limot.m_loLimit ? limot.m_servoTarget : limot.m_loLimit;
						hiLimit = error < 0 && limot.m_servoTarget < limot.m_hiLimit ? limot.m_servoTarget : limot.m_hiLimit;
					}
					mot_fact = getMotorFactor( limot.m_currentPosition, lowLimit, hiLimit, tag_vel, info.fps * limot.m_motorERP );
				}
				else
				{
					mot_fact = 0;
				}
				info.m_solverConstraints[row].m_rhs = mot_fact * targetvelocity * ( rotational ? -1 : 1 );
				info.m_solverConstraints[row].m_lowerLimit = -limot.m_maxMotorForce;
				info.m_solverConstraints[row].m_upperLimit = limot.m_maxMotorForce;
				info.m_solverConstraints[row].m_cfm = limot.m_motorCFM;
				row++;
				//srow += info.rowskip;
				++count;
			}

			if( limot.m_enableSpring )
			{
				double error = limot.m_currentPosition - limot.m_equilibriumPoint;
				calculateJacobi( limot, ref transA, ref transB, info, row, ref ax1, rotational, rotAllowed );

				//double cfm = 1.0 / ((1.0/info.fps)*limot.m_springStiffness+ limot.m_springDamping);
				//if(cfm > 0.99999)
				//	cfm = 0.99999;
				//double erp = (1.0/info.fps)*limot.m_springStiffness / ((1.0/info.fps)*limot.m_springStiffness + limot.m_springDamping);
				//info.m_constraintError[srow] = info.fps * erp * error * (rotational ? -1.0 : 1.0);
				//info.m_lowerLimit[srow] = -SIMD_INFINITY;
				//info.m_upperLimit[srow] = SIMD_INFINITY;

				double dt = btScalar.BT_ONE / info.fps;
				double kd = limot.m_springDamping;
				double ks = limot.m_springStiffness;
				double vel = rotational ? angVelA.dot( ax1 ) - angVelB.dot( ax1 ) : linVelA.dot( ax1 ) - linVelB.dot( ax1 );
				//		double erp = 0.1;
				double cfm = btScalar.BT_ZERO;
				double mA = btScalar.BT_ONE / m_rbA.getInvMass();
				double mB = btScalar.BT_ONE / m_rbB.getInvMass();
				double m = mA > mB ? mB : mA;
				double angularfreq = btScalar.btSqrt( ks / m );


				//limit stiffness (the spring should not be sampled faster that the quarter of its angular frequency)
				if( limot.m_springStiffnessLimited && 0.25 < angularfreq * dt )
				{
					ks = btScalar.BT_ONE / dt / dt / (double)( 16.0 ) * m;
				}
				//avoid damping that would blow up the spring
				if( limot.m_springDampingLimited && kd * dt > m )
				{
					kd = m / dt;
				}
				double fs = ks * error * dt;
				double fd = -kd * ( vel ) * ( rotational ? -1 : 1 ) * dt;
				double f = ( fs + fd );

				info.m_solverConstraints[row].m_rhs = ( vel + f * ( rotational ? -1 : 1 ) );

				double minf = f < fd ? f : fd;
				double maxf = f < fd ? fd : f;
				if( !rotational )
				{
					info.m_solverConstraints[row].m_lowerLimit = minf > 0 ? 0 : minf;
					info.m_solverConstraints[row].m_upperLimit = maxf < 0 ? 0 : maxf;
				}
				else
				{
					info.m_solverConstraints[row].m_lowerLimit = -maxf > 0 ? 0 : -maxf;
					info.m_solverConstraints[row].m_upperLimit = -minf < 0 ? 0 : -minf;
				}

				info.m_solverConstraints[row].m_cfm = cfm;
				row++;
				++count;
			}

			return count;
		}


		//override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
		//If no axis is provided, it uses the default axis for this constraint.
		public override void setParam( btConstraintParams num, double value, int axis = -1 )
		{
			if( ( axis >= 0 ) && ( axis < 3 ) )
			{
				switch( num )
				{
					case btConstraintParams.BT_CONSTRAINT_STOP_ERP:
						m_linearLimits.m_stopERP[axis] = value;
						m_flags |= bt6DofFlagsIndexed.BT_6DOF_FLAGS_ERP_STOP2[axis];
						break;
					case btConstraintParams.BT_CONSTRAINT_STOP_CFM:
						m_linearLimits.m_stopCFM[axis] = value;
						m_flags |= bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_STOP2[axis];
						break;
					case btConstraintParams.BT_CONSTRAINT_ERP:
						m_linearLimits.m_motorERP[axis] = value;
						m_flags |= bt6DofFlagsIndexed.BT_6DOF_FLAGS_ERP_MOTO2[axis];
						break;
					case btConstraintParams.BT_CONSTRAINT_CFM:
						m_linearLimits.m_motorCFM[axis] = value;
						m_flags |= bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_MOTO2[axis];
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
						m_flags |= bt6DofFlagsIndexed.BT_6DOF_FLAGS_ERP_STOP2[axis];
						break;
					case btConstraintParams.BT_CONSTRAINT_STOP_CFM:
						m_angularLimits[axis - 3].m_stopCFM = value;
						m_flags |= bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_STOP2[axis];
						break;
					case btConstraintParams.BT_CONSTRAINT_ERP:
						m_angularLimits[axis - 3].m_motorERP = value;
						m_flags |= bt6DofFlagsIndexed.BT_6DOF_FLAGS_ERP_MOTO2[axis];
						break;
					case btConstraintParams.BT_CONSTRAINT_CFM:
						m_angularLimits[axis - 3].m_motorCFM = value;
						m_flags |= bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_MOTO2[axis];
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

		//return the local value of parameter
		public override double getParam( btConstraintParams num, int axis = -1 )
		{
			double retVal = 0;
			if( ( axis >= 0 ) && ( axis < 3 ) )
			{
				switch( num )
				{
					case btConstraintParams.BT_CONSTRAINT_STOP_ERP:
						btAssertConstrParams( ( m_flags & ( bt6DofFlagsIndexed.BT_6DOF_FLAGS_ERP_STOP2[axis] ) ) != 0 );
						retVal = m_linearLimits.m_stopERP[axis];
						break;
					case btConstraintParams.BT_CONSTRAINT_STOP_CFM:
						btAssertConstrParams( ( m_flags & ( bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_STOP2[axis] ) ) != 0 );
						retVal = m_linearLimits.m_stopCFM[axis];
						break;
					case btConstraintParams.BT_CONSTRAINT_ERP:
						btAssertConstrParams( ( m_flags & ( bt6DofFlagsIndexed.BT_6DOF_FLAGS_ERP_MOTO2[axis] ) ) != 0 );
						retVal = m_linearLimits.m_motorERP[axis];
						break;
					case btConstraintParams.BT_CONSTRAINT_CFM:
						btAssertConstrParams( ( m_flags & ( bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_MOTO2[axis] ) ) != 0 );
						retVal = m_linearLimits.m_motorCFM[axis];
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
						btAssertConstrParams( ( m_flags & ( bt6DofFlagsIndexed.BT_6DOF_FLAGS_ERP_STOP2[axis] ) ) != 0 );
						retVal = m_angularLimits[axis - 3].m_stopERP;
						break;
					case btConstraintParams.BT_CONSTRAINT_STOP_CFM:
						btAssertConstrParams( ( m_flags & ( bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_STOP2[axis] ) ) != 0 );
						retVal = m_angularLimits[axis - 3].m_stopCFM;
						break;
					case btConstraintParams.BT_CONSTRAINT_ERP:
						btAssertConstrParams( ( m_flags & ( bt6DofFlagsIndexed.BT_6DOF_FLAGS_ERP_MOTO2[axis] ) ) != 0 );
						retVal = m_angularLimits[axis - 3].m_motorERP;
						break;
					case btConstraintParams.BT_CONSTRAINT_CFM:
						btAssertConstrParams( ( m_flags & ( bt6DofFlagsIndexed.BT_6DOF_FLAGS_CFM_MOTO2[axis] ) ) != 0 );
						retVal = m_angularLimits[axis - 3].m_motorCFM;
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
			frameInW.m_basis.setValue( ref xAxis, ref yAxis, ref zAxis );

			// now get constraint frame in local coordinate systems
			btTransform tmp;
			m_rbA.m_worldTransform.inverse( out tmp );

			tmp.Apply( ref frameInW, out m_frameInA );
			m_rbB.m_worldTransform.inverse( out tmp );
			tmp.Apply( ref frameInW, out m_frameInB );

			calculateTransforms();
		}

		void setBounce( int index, double bounce )
		{
			Debug.Assert( ( index >= 0 ) && ( index < 6 ) );
			if( index < 3 )
				m_linearLimits.m_bounce[index] = bounce;
			else
				m_angularLimits[index - 3].m_bounce = bounce;
		}

		void enableMotor( int index, bool onOff )
		{
			Debug.Assert( ( index >= 0 ) && ( index < 6 ) );
			if( index < 3 )
				m_linearLimits.m_enableMotor[index] = onOff;
			else
				m_angularLimits[index - 3].m_enableMotor = onOff;
		}

		void setServo( int index, bool onOff )
		{
			Debug.Assert( ( index >= 0 ) && ( index < 6 ) );
			if( index < 3 )
				m_linearLimits.m_servoMotor[index] = onOff;
			else
				m_angularLimits[index - 3].m_servoMotor = onOff;
		}

		void setTargetVelocity( int index, double velocity )
		{
			Debug.Assert( ( index >= 0 ) && ( index < 6 ) );
			if( index < 3 )
				m_linearLimits.m_targetVelocity[index] = velocity;
			else
				m_angularLimits[index - 3].m_targetVelocity = velocity;
		}

		void setServoTarget( int index, double target )
		{
			Debug.Assert( ( index >= 0 ) && ( index < 6 ) );
			if( index < 3 )
				m_linearLimits.m_servoTarget[index] = target;
			else
				m_angularLimits[index - 3].m_servoTarget = target;
		}

		void setMaxMotorForce( int index, double force )
		{
			Debug.Assert( ( index >= 0 ) && ( index < 6 ) );
			if( index < 3 )
				m_linearLimits.m_maxMotorForce[index] = force;
			else
				m_angularLimits[index - 3].m_maxMotorForce = force;
		}

		void enableSpring( int index, bool onOff )
		{
			Debug.Assert( ( index >= 0 ) && ( index < 6 ) );
			if( index < 3 )
				m_linearLimits.m_enableSpring[index] = onOff;
			else
				m_angularLimits[index - 3].m_enableSpring = onOff;
		}

		void setStiffness( int index, double stiffness, bool limitIfNeeded )
		{
			Debug.Assert( ( index >= 0 ) && ( index < 6 ) );
			if( index < 3 )
			{
				m_linearLimits.m_springStiffness[index] = stiffness;
				m_linearLimits.m_springStiffnessLimited[index] = limitIfNeeded;
			}
			else
			{
				m_angularLimits[index - 3].m_springStiffness = stiffness;
				m_angularLimits[index - 3].m_springStiffnessLimited = limitIfNeeded;
			}
		}

		void setDamping( int index, double damping, bool limitIfNeeded )
		{
			Debug.Assert( ( index >= 0 ) && ( index < 6 ) );
			if( index < 3 )
			{
				m_linearLimits.m_springDamping[index] = damping;
				m_linearLimits.m_springDampingLimited[index] = limitIfNeeded;
			}
			else
			{
				m_angularLimits[index - 3].m_springDamping = damping;
				m_angularLimits[index - 3].m_springDampingLimited = limitIfNeeded;
			}
		}

		void setEquilibriumPoint()
		{
			calculateTransforms();
			int i;
			for( i = 0; i < 3; i++ )
				m_linearLimits.m_equilibriumPoint[i] = m_calculatedLinearDiff[i];
			for( i = 0; i < 3; i++ )
				m_angularLimits[i].m_equilibriumPoint = m_calculatedAxisAngleDiff[i];
		}

		void setEquilibriumPoint( int index )
		{
			Debug.Assert( ( index >= 0 ) && ( index < 6 ) );
			calculateTransforms();
			if( index < 3 )
				m_linearLimits.m_equilibriumPoint[index] = m_calculatedLinearDiff[index];
			else
				m_angularLimits[index - 3].m_equilibriumPoint = m_calculatedAxisAngleDiff[index - 3];
		}

		void setEquilibriumPoint( int index, double val )
		{
			Debug.Assert( ( index >= 0 ) && ( index < 6 ) );
			if( index < 3 )
				m_linearLimits.m_equilibriumPoint[index] = val;
			else
				m_angularLimits[index - 3].m_equilibriumPoint = val;
		}

	};

#if SERIALIZE_DONE

struct btGeneric6DofSpring2ConstraintData
{
	btTypedConstraintData m_typeConstraintData;
	btTransformFloatData m_rbAFrame;
	btTransformFloatData m_rbBFrame;

	btVector3FloatData m_linearUpperLimit;
	btVector3FloatData m_linearLowerLimit;
	btVector3FloatData m_linearBounce;
	btVector3FloatData m_linearStopERP;
	btVector3FloatData m_linearStopCFM;
	btVector3FloatData m_linearMotorERP;
	btVector3FloatData m_linearMotorCFM;
	btVector3FloatData m_linearTargetVelocity;
	btVector3FloatData m_linearMaxMotorForce;
	btVector3FloatData m_linearServoTarget;
	btVector3FloatData m_linearSpringStiffness;
	btVector3FloatData m_linearSpringDamping;
	btVector3FloatData m_linearEquilibriumPoint;
	char m_linearEnableMotor[4];
	char m_linearServoMotor[4];
	char m_linearEnableSpring[4];
	char m_linearSpringStiffnessLimited[4];
	char m_linearSpringDampingLimited[4];
	char m_padding1[4];

	btVector3FloatData m_angularUpperLimit;
	btVector3FloatData m_angularLowerLimit;
	btVector3FloatData m_angularBounce;
	btVector3FloatData m_angularStopERP;
	btVector3FloatData m_angularStopCFM;
	btVector3FloatData m_angularMotorERP;
	btVector3FloatData m_angularMotorCFM;
	btVector3FloatData m_angularTargetVelocity;
	btVector3FloatData m_angularMaxMotorForce;
	btVector3FloatData m_angularServoTarget;
	btVector3FloatData m_angularSpringStiffness;
	btVector3FloatData m_angularSpringDamping;
	btVector3FloatData m_angularEquilibriumPoint;
	char m_angularEnableMotor[4];
	char m_angularServoMotor[4];
	char m_angularEnableSpring[4];
	char m_angularSpringStiffnessLimited[4];
	char m_angularSpringDampingLimited[4];

	int m_rotateOrder;
};

struct btGeneric6DofSpring2ConstraintDoubleData2
{
	btTypedConstraintDoubleData m_typeConstraintData;
	btTransformDoubleData m_rbAFrame;
	btTransformDoubleData m_rbBFrame;

	btVector3DoubleData m_linearUpperLimit;
	btVector3DoubleData m_linearLowerLimit;
	btVector3DoubleData m_linearBounce;
	btVector3DoubleData m_linearStopERP;
	btVector3DoubleData m_linearStopCFM;
	btVector3DoubleData m_linearMotorERP;
	btVector3DoubleData m_linearMotorCFM;
	btVector3DoubleData m_linearTargetVelocity;
	btVector3DoubleData m_linearMaxMotorForce;
	btVector3DoubleData m_linearServoTarget;
	btVector3DoubleData m_linearSpringStiffness;
	btVector3DoubleData m_linearSpringDamping;
	btVector3DoubleData m_linearEquilibriumPoint;
	char m_linearEnableMotor[4];
	char m_linearServoMotor[4];
	char m_linearEnableSpring[4];
	char m_linearSpringStiffnessLimited[4];
	char m_linearSpringDampingLimited[4];
	char m_padding1[4];

	btVector3DoubleData m_angularUpperLimit;
	btVector3DoubleData m_angularLowerLimit;
	btVector3DoubleData m_angularBounce;
	btVector3DoubleData m_angularStopERP;
	btVector3DoubleData m_angularStopCFM;
	btVector3DoubleData m_angularMotorERP;
	btVector3DoubleData m_angularMotorCFM;
	btVector3DoubleData m_angularTargetVelocity;
	btVector3DoubleData m_angularMaxMotorForce;
	btVector3DoubleData m_angularServoTarget;
	btVector3DoubleData m_angularSpringStiffness;
	btVector3DoubleData m_angularSpringDamping;
	btVector3DoubleData m_angularEquilibriumPoint;
	char m_angularEnableMotor[4];
	char m_angularServoMotor[4];
	char m_angularEnableSpring[4];
	char m_angularSpringStiffnessLimited[4];
	char m_angularSpringDampingLimited[4];

	int m_rotateOrder;
};

public int calculateSerializeBufferSize()
{
	return sizeof( btGeneric6DofSpring2ConstraintData2 );
}

public string serialize( object dataBuffer, btSerializer* serializer )
{
	btGeneric6DofSpring2ConstraintData2* dof = (btGeneric6DofSpring2ConstraintData2*)dataBuffer;
	btTypedConstraint::serialize( &dof.m_typeConstraintData, serializer );

	m_frameInA.serialize( dof.m_rbAFrame );
	m_frameInB.serialize( dof.m_rbBFrame );

	int i;
	for( i = 0; i < 3; i++ )
	{
		dof.m_angularLowerLimit.m_floats[i] = m_angularLimits[i].m_loLimit;
		dof.m_angularUpperLimit.m_floats[i] = m_angularLimits[i].m_hiLimit;
		dof.m_angularBounce.m_floats[i] = m_angularLimits[i].m_bounce;
		dof.m_angularStopERP.m_floats[i] = m_angularLimits[i].m_stopERP;
		dof.m_angularStopCFM.m_floats[i] = m_angularLimits[i].m_stopCFM;
		dof.m_angularMotorERP.m_floats[i] = m_angularLimits[i].m_motorERP;
		dof.m_angularMotorCFM.m_floats[i] = m_angularLimits[i].m_motorCFM;
		dof.m_angularTargetVelocity.m_floats[i] = m_angularLimits[i].m_targetVelocity;
		dof.m_angularMaxMotorForce.m_floats[i] = m_angularLimits[i].m_maxMotorForce;
		dof.m_angularServoTarget.m_floats[i] = m_angularLimits[i].m_servoTarget;
		dof.m_angularSpringStiffness.m_floats[i] = m_angularLimits[i].m_springStiffness;
		dof.m_angularSpringDamping.m_floats[i] = m_angularLimits[i].m_springDamping;
		dof.m_angularEquilibriumPoint.m_floats[i] = m_angularLimits[i].m_equilibriumPoint;
	}
	dof.m_angularLowerLimit.m_floats[3] = 0;
	dof.m_angularUpperLimit.m_floats[3] = 0;
	dof.m_angularBounce.m_floats[3] = 0;
	dof.m_angularStopERP.m_floats[3] = 0;
	dof.m_angularStopCFM.m_floats[3] = 0;
	dof.m_angularMotorERP.m_floats[3] = 0;
	dof.m_angularMotorCFM.m_floats[3] = 0;
	dof.m_angularTargetVelocity.m_floats[3] = 0;
	dof.m_angularMaxMotorForce.m_floats[3] = 0;
	dof.m_angularServoTarget.m_floats[3] = 0;
	dof.m_angularSpringStiffness.m_floats[3] = 0;
	dof.m_angularSpringDamping.m_floats[3] = 0;
	dof.m_angularEquilibriumPoint.m_floats[3] = 0;
	for( i = 0; i < 4; i++ )
	{
		dof.m_angularEnableMotor[i] = i < 3 ? ( m_angularLimits[i].m_enableMotor ? 1 : 0 ) : 0;
		dof.m_angularServoMotor[i] = i < 3 ? ( m_angularLimits[i].m_servoMotor ? 1 : 0 ) : 0;
		dof.m_angularEnableSpring[i] = i < 3 ? ( m_angularLimits[i].m_enableSpring ? 1 : 0 ) : 0;
		dof.m_angularSpringStiffnessLimited[i] = i < 3 ? ( m_angularLimits[i].m_springStiffnessLimited ? 1 : 0 ) : 0;
		dof.m_angularSpringDampingLimited[i] = i < 3 ? ( m_angularLimits[i].m_springDampingLimited ? 1 : 0 ) : 0;
	}

	m_linearLimits.m_lowerLimit.serialize( dof.m_linearLowerLimit );
	m_linearLimits.m_upperLimit.serialize( dof.m_linearUpperLimit );
	m_linearLimits.m_bounce.serialize( dof.m_linearBounce );
	m_linearLimits.m_stopERP.serialize( dof.m_linearStopERP );
	m_linearLimits.m_stopCFM.serialize( dof.m_linearStopCFM );
	m_linearLimits.m_motorERP.serialize( dof.m_linearMotorERP );
	m_linearLimits.m_motorCFM.serialize( dof.m_linearMotorCFM );
	m_linearLimits.m_targetVelocity.serialize( dof.m_linearTargetVelocity );
	m_linearLimits.m_maxMotorForce.serialize( dof.m_linearMaxMotorForce );
	m_linearLimits.m_servoTarget.serialize( dof.m_linearServoTarget );
	m_linearLimits.m_springStiffness.serialize( dof.m_linearSpringStiffness );
	m_linearLimits.m_springDamping.serialize( dof.m_linearSpringDamping );
	m_linearLimits.m_equilibriumPoint.serialize( dof.m_linearEquilibriumPoint );
	for( i = 0; i < 4; i++ )
	{
		dof.m_linearEnableMotor[i] = i < 3 ? ( m_linearLimits.m_enableMotor[i] ? 1 : 0 ) : 0;
		dof.m_linearServoMotor[i] = i < 3 ? ( m_linearLimits.m_servoMotor[i] ? 1 : 0 ) : 0;
		dof.m_linearEnableSpring[i] = i < 3 ? ( m_linearLimits.m_enableSpring[i] ? 1 : 0 ) : 0;
		dof.m_linearSpringStiffnessLimited[i] = i < 3 ? ( m_linearLimits.m_springStiffnessLimited[i] ? 1 : 0 ) : 0;
		dof.m_linearSpringDampingLimited[i] = i < 3 ? ( m_linearLimits.m_springDampingLimited[i] ? 1 : 0 ) : 0;
	}

	dof.m_rotateOrder = m_rotateOrder;

	return btGeneric6DofSpring2ConstraintDataName;
}

#endif


}
