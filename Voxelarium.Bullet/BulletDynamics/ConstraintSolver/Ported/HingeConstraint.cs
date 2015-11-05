//#define HINGE_USE_OBSOLETE_SOLVER
#define HINGE_USE_FRAME_OFFSET
#define _BT_USE_CENTER_LIMIT_
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

/* Hinge Constraint by Dirk Gregorius. Limits added by Marcus Hennix at Starbreeze Studios */

using Bullet.LinearMath;
using Bullet.Types;
using System;
using System.Diagnostics;

namespace Bullet.Dynamics.ConstraintSolver
{
	/*
	#if BT_USE_DOUBLE_PRECISION
	#define btHingeConstraintData	btHingeConstraintDoubleData2 //rename to 2 for backwards compatibility, so we can still load the 'btHingeConstraintDoubleData' version
	#define btHingeConstraintDataName	"btHingeConstraintDoubleData2" 
	#else
	#define btHingeConstraintData	btHingeConstraintFloatData
	#define btHingeConstraintDataName	"btHingeConstraintFloatData"
	#endif //BT_USE_DOUBLE_PRECISION
	*/


	[Flags]
	internal enum btHingeFlags
	{
		BT_HINGE_FLAGS_CFM_STOP = 1,
		BT_HINGE_FLAGS_ERP_STOP = 2,
		BT_HINGE_FLAGS_CFM_NORM = 4,
		BT_HINGE_FLAGS_ERP_NORM = 8
	};


	/// hinge constraint between two rigidbodies each with a pivotpoint that descibes the axis location in local space
	/// axis defines the orientation of the hinge axis
	internal class btHingeConstraint : btTypedConstraint
	{

		internal btJacobianEntry[] m_jac = new btJacobianEntry[3]; //3 orthogonal linear constraints
		internal btJacobianEntry[] m_jacAng = new btJacobianEntry[3]; //2 orthogonal angular constraints+ 1 for limit/motor

		internal btTransform m_rbAFrame; // constraint axii. Assumes z is hinge axis.
		internal btTransform m_rbBFrame;

		internal double m_motorTargetVelocity;
		internal double m_maxMotorImpulse;


#if _BT_USE_CENTER_LIMIT_
		btAngularLimit m_limit;
#else
		double m_lowerLimit;
		double m_upperLimit;
		double m_limitSign;
		double m_correction;

		double m_limitSoftness;
		double m_biasFactor;
		double m_relaxationFactor;

		bool m_solveLimit;
#endif

		double m_kHinge;


		double m_accLimitImpulse;
		double m_hingeAngle;
		double m_referenceSign;

		bool m_angularOnly;
		bool m_enableAngularMotor;
		//bool m_useSolveConstraintObsolete;
		bool m_useOffsetForConstraintFrame;
		bool m_useReferenceFrameA;

		double m_accMotorImpulse;

		btHingeFlags m_flags;
		double m_normalCFM;
		double m_normalERP;
		double m_stopCFM;
		double m_stopERP;


		internal btITransform getFrameOffsetA()
		{
			return m_rbAFrame;
		}

		internal btITransform getFrameOffsetB()
		{
			return m_rbBFrame;
		}

		//void setFrames( ref btTransform frameA, ref btTransform frameB );

		internal void setAngularOnly( bool angularOnly )
		{
			m_angularOnly = angularOnly;
		}

		internal void enableAngularMotor( bool enableMotor, double targetVelocity, double maxMotorImpulse )
		{
			m_enableAngularMotor = enableMotor;
			m_motorTargetVelocity = targetVelocity;
			m_maxMotorImpulse = maxMotorImpulse;
		}

		// extra motor API, including ability to set a target rotation (as opposed to angular velocity)
		// note: setMotorTarget sets angular velocity under the hood, so you must call it every tick to
		//       maintain a given angular target.
		public void enableMotor( bool enableMotor ) { m_enableAngularMotor = enableMotor; }
		public void setMaxMotorImpulse( double maxMotorImpulse ) { m_maxMotorImpulse = maxMotorImpulse; }
		public void setMotorTargetVelocity( btScalar motorTargetVelocity ) { m_motorTargetVelocity = motorTargetVelocity; }
		//void setMotorTarget( btQuaternion& qAinB, double dt ); // qAinB is rotation of body A wrt body B.
		//void setMotorTarget( double targetAngle, double dt );


		internal void setLimit( double low, double high, double _softness = 0.9f, double _biasFactor = 0.3f, double _relaxationFactor = 1.0f )
		{
#if _BT_USE_CENTER_LIMIT_
			m_limit.set( low, high, _softness, _biasFactor, _relaxationFactor );
#else
			m_lowerLimit = btScalar.btNormalizeAngle( low );
			m_upperLimit = btScalar.btNormalizeAngle( high );
			m_limitSoftness = _softness;
			m_biasFactor = _biasFactor;
			m_relaxationFactor = _relaxationFactor;
#endif
		}


		public double getLimitSoftness()
		{
#if _BT_USE_CENTER_LIMIT_
			return m_limit.getSoftness();
#else
			return m_limitSoftness;
#endif
		}

		public double getLimitBiasFactor()
		{
#if _BT_USE_CENTER_LIMIT_
			return m_limit.getBiasFactor();
#else
			return m_biasFactor;
#endif
		}

		public double getLimitRelaxationFactor()
		{
#if _BT_USE_CENTER_LIMIT_
			return m_limit.getRelaxationFactor();
#else
			return m_relaxationFactor;
#endif
		}

		internal void setAxis( ref btVector3 axisInA )
		{
			btVector3 rbAxisA1, rbAxisA2;
			btVector3.btPlaneSpace1( ref axisInA, out rbAxisA1, out rbAxisA2 );
			btVector3 pivotInA; m_rbAFrame.getOrigin( out pivotInA );
			//		m_rbAFrame.getOrigin() = pivotInA;
			m_rbAFrame.getBasis().setValue( rbAxisA1.x, rbAxisA2.x, axisInA.x,
											rbAxisA1.y, rbAxisA2.y, axisInA.y,
											rbAxisA1.z, rbAxisA2.z, axisInA.z );

			btVector3 axisInB = m_rbA.getCenterOfMassTransform().getBasis() * axisInA;

			btQuaternion rotationArc; btQuaternion.shortestArcQuat( ref axisInA, ref axisInB, out rotationArc );
			btVector3 rbAxisB1; btQuaternion.quatRotate( ref rotationArc, ref rbAxisA1, out rbAxisB1 );
			btVector3 rbAxisB2; axisInB.cross( ref rbAxisB1, out rbAxisB2 );

			btTransform tmp;
			m_rbB.getCenterOfMassTransform().inverse( out tmp );
			btVector3 tmp2;
			m_rbA.getCenterOfMassTransform().Apply( ref pivotInA, out tmp2 );
			tmp.Apply( ref tmp2, out m_rbBFrame.m_origin );

			m_rbBFrame.getBasis().setValue( rbAxisB1.x, rbAxisB2.x, axisInB.x,
											rbAxisB1.y, rbAxisB2.y, axisInB.y,
											rbAxisB1.z, rbAxisB2.z, axisInB.z );

			m_rbB.getCenterOfMassTransform().inverse( out tmp );

			tmp.m_basis.Apply( ref m_rbBFrame.m_basis, out m_rbBFrame.m_basis );

		}

		internal bool hasLimit()
		{
#if _BT_USE_CENTER_LIMIT_
			return m_limit.getHalfRange() > 0;
#else
			return m_lowerLimit <= m_upperLimit;
#endif
		}

		public double getLowerLimit()
		{
#if _BT_USE_CENTER_LIMIT_
			return m_limit.getLow();
#else
			return m_lowerLimit;
#endif
		}

		public double getUpperLimit()
		{
#if _BT_USE_CENTER_LIMIT_
			return m_limit.getHigh();
#else
			return m_upperLimit;
#endif
		}


		///The getHingeAngle gives the hinge angle in range [-PI,PI]
		//double getHingeAngle();

		//double getHingeAngle( ref btTransform transA, ref btTransform transB );


		internal btITransform getAFrame() { return m_rbAFrame; }
		internal btITransform getBFrame() { return m_rbBFrame; }

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public bool getSolveLimit()
		{
#if _BT_USE_CENTER_LIMIT_
			return m_limit.isLimit();
#else
			return m_solveLimit;
#endif
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public double getLimitSign()
		{
#if _BT_USE_CENTER_LIMIT_
			return m_limit.getSign();
#else
			return m_limitSign;
#endif
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public bool getAngularOnly()
		{
			return m_angularOnly;
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public bool getEnableAngularMotor()
		{
			return m_enableAngularMotor;
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public double getMotorTargetVelosity()
		{
			return m_motorTargetVelocity;
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public double getMaxMotorImpulse()
		{
			return m_maxMotorImpulse;
		}
		// access for UseFrameOffset

		public bool getUseFrameOffset() { return m_useOffsetForConstraintFrame; }
		public void setUseFrameOffset( bool frameOffsetOnOff ) { m_useOffsetForConstraintFrame = frameOffsetOnOff; }

		// access for UseReferenceFrameA
		public bool getUseReferenceFrameA() { return m_useReferenceFrameA; }
		public void setUseReferenceFrameA( bool useReferenceFrameA ) { m_useReferenceFrameA = useReferenceFrameA; }

		public btHingeFlags getFlags()
		{
			return m_flags;
		}



		///return the local value of parameter

#if SERIALIZE_DONE
		virtual int calculateSerializeBufferSize();

		///fills the dataBuffer and returns the struct name (and 0 on failure)
		virtual string serialize( object dataBuffer, btSerializer* serializer );
#endif


		public btHingeConstraint( btRigidBody rbA, btRigidBody rbB, ref btVector3 pivotInA, ref btVector3 pivotInB,
												  ref btVector3 axisInA, ref btVector3 axisInB, bool useReferenceFrameA = false )
										 : base( btObjectTypes.HINGE_CONSTRAINT_TYPE, rbA, rbB )
		{
#if _BT_USE_CENTER_LIMIT_
			m_limit = new btAngularLimit();
#endif
			m_angularOnly = ( false );
			m_enableAngularMotor = ( false );
			//m_useSolveConstraintObsolete = ( false/*HINGE_USE_OBSOLETE_SOLVER*/ );
			//m_useOffsetForConstraintFrame = ( true/*HINGE_USE_FRAME_OFFSET*/ );
			m_useReferenceFrameA = ( useReferenceFrameA );
			m_flags = ( 0 );
			m_normalCFM = ( 0 );
			m_normalERP = ( 0 );
			m_stopCFM = ( 0 );
			m_stopERP = ( 0 );
			m_rbAFrame.m_origin = pivotInA;

			// since no frame is given, assume this to be zero angle and just pick rb transform axis
			btVector3 rbAxisA1 = rbA.getCenterOfMassTransform().getBasis().getColumn( 0 );

			btVector3 rbAxisA2;
			double projection = axisInA.dot( rbAxisA1 );
			if( projection >= 1.0f - btScalar.SIMD_EPSILON )
			{
				rbAxisA1 = -rbA.getCenterOfMassTransform().getBasis().getColumn( 2 );
				rbAxisA2 = rbA.getCenterOfMassTransform().getBasis().getColumn( 1 );
			}
			else if( projection <= -1.0f + btScalar.SIMD_EPSILON )
			{
				rbAxisA1 = rbA.getCenterOfMassTransform().getBasis().getColumn( 2 );
				rbAxisA2 = rbA.getCenterOfMassTransform().getBasis().getColumn( 1 );
			}
			else
			{
				rbAxisA2 = axisInA.cross( rbAxisA1 );
				rbAxisA1 = rbAxisA2.cross( axisInA );
			}

			m_rbAFrame.getBasis().setValue( rbAxisA1.x, rbAxisA2.x, axisInA.x,
											rbAxisA1.y, rbAxisA2.y, axisInA.y,
											rbAxisA1.z, rbAxisA2.z, axisInA.z );

			btQuaternion rotationArc; btQuaternion.shortestArcQuat( ref axisInA, ref axisInB, out rotationArc );
			btVector3 rbAxisB1; btQuaternion.quatRotate( ref rotationArc, ref rbAxisA1, out rbAxisB1 );
			btVector3 rbAxisB2; axisInB.cross( ref rbAxisB1, out rbAxisB2 );

			m_rbBFrame.m_origin = pivotInB;
			m_rbBFrame.getBasis().setValue( rbAxisB1.x, rbAxisB2.x, axisInB.x,
											rbAxisB1.y, rbAxisB2.y, axisInB.y,
											rbAxisB1.z, rbAxisB2.z, axisInB.z );

#if !_BT_USE_CENTER_LIMIT_
			//start with free
			m_lowerLimit = (double)( 1.0f );
			m_upperLimit = (double)( -1.0f );
			m_biasFactor = 0.3f;
			m_relaxationFactor = 1.0f;
			m_limitSoftness = 0.9f;
			m_solveLimit = false;
#endif
			m_referenceSign = m_useReferenceFrameA ? (double)( -1 ) : (double)( 1 );
		}



		public btHingeConstraint( btRigidBody rbA, ref btVector3 pivotInA, ref btVector3 axisInA, bool useReferenceFrameA = false )
				: base( btObjectTypes.HINGE_CONSTRAINT_TYPE, rbA )
		{
#if _BT_USE_CENTER_LIMIT_
			m_limit = new btAngularLimit();
#endif
			m_angularOnly = ( false ); m_enableAngularMotor = ( false );
			//m_useSolveConstraintObsolete = ( false/*HINGE_USE_OBSOLETE_SOLVER*/ );
			//m_useOffsetForConstraintFrame = ( true/*HINGE_USE_FRAME_OFFSET*/ );
			m_useReferenceFrameA = ( useReferenceFrameA );
			m_flags = ( 0 );
			m_normalCFM = ( 0 );
			m_normalERP = ( 0 );
			m_stopCFM = ( 0 );
			m_stopERP = ( 0 );

			// since no frame is given; assume this to be zero angle and just pick rb transform axis
			// fixed axis in worldspace
			btVector3 rbAxisA1, rbAxisA2;
			btVector3.btPlaneSpace1( ref axisInA, out rbAxisA1, out rbAxisA2 );

			m_rbAFrame.m_origin = pivotInA;
			m_rbAFrame.getBasis().setValue( rbAxisA1.x, rbAxisA2.x, axisInA.x,
											rbAxisA1.y, rbAxisA2.y, axisInA.y,
											rbAxisA1.z, rbAxisA2.z, axisInA.z );

			btVector3 axisInB = rbA.getCenterOfMassTransform().getBasis() * axisInA;

			btQuaternion rotationArc; btQuaternion.shortestArcQuat( ref axisInA, ref axisInB, out rotationArc );
			btVector3 rbAxisB1; btQuaternion.quatRotate( ref rotationArc, ref rbAxisA1, out rbAxisB1 );
			btVector3 rbAxisB2; axisInB.cross( ref rbAxisB1, out rbAxisB2 );

			//btVector3 tmp;
			rbA.m_worldTransform.Apply( ref pivotInA, out m_rbBFrame.m_origin );
			//m_rbBFrame.m_origin = rbA.getCenterOfMassTransform()( pivotInA );
			m_rbBFrame.getBasis().setValue( rbAxisB1.x, rbAxisB2.x, axisInB.x,
											rbAxisB1.y, rbAxisB2.y, axisInB.y,
											rbAxisB1.z, rbAxisB2.z, axisInB.z );

#if !_BT_USE_CENTER_LIMIT_
			//start with free
			m_lowerLimit = (double)( 1.0f );
			m_upperLimit = (double)( -1.0f );
			m_biasFactor = 0.3f;
			m_relaxationFactor = 1.0f;
			m_limitSoftness = 0.9f;
			m_solveLimit = false;
#endif
			m_referenceSign = m_useReferenceFrameA ? (double)( -1 ) : (double)( 1 );
		}



		public btHingeConstraint( btRigidBody rbA, btRigidBody rbB,
									 ref btTransform rbAFrame, ref btTransform rbBFrame, bool useReferenceFrameA = false )
						: base( btObjectTypes.HINGE_CONSTRAINT_TYPE, rbA, rbB )
		{
			m_rbAFrame = ( rbAFrame );
			m_rbBFrame = ( rbBFrame );
#if _BT_USE_CENTER_LIMIT_
			m_limit = new btAngularLimit();
#endif
			m_angularOnly = ( false );
			m_enableAngularMotor = ( false );
			//m_useSolveConstraintObsolete = ( false/*HINGE_USE_OBSOLETE_SOLVER*/ );
			//m_useOffsetForConstraintFrame = ( true /*HINGE_USE_FRAME_OFFSET*/ );
			m_useReferenceFrameA = ( useReferenceFrameA );
			m_flags = ( 0 );
			m_normalCFM = ( 0 );
			m_normalERP = ( 0 );
			m_stopCFM = ( 0 );
			m_stopERP = ( 0 );
#if !_BT_USE_CENTER_LIMIT_
			//start with free
			m_lowerLimit = (double)( 1.0f );
			m_upperLimit = (double)( -1.0f );
			m_biasFactor = 0.3f;
			m_relaxationFactor = 1.0f;
			m_limitSoftness = 0.9f;
			m_solveLimit = false;
#endif
			m_referenceSign = m_useReferenceFrameA ? (double)( -1 ) : (double)( 1 );
		}



		public btHingeConstraint( btRigidBody rbA, ref btTransform rbAFrame, bool useReferenceFrameA = false )
							: base( btObjectTypes.HINGE_CONSTRAINT_TYPE, rbA )
		{
			m_rbAFrame = ( rbAFrame );
			m_rbBFrame = ( rbAFrame );
#if _BT_USE_CENTER_LIMIT_
			m_limit = new btAngularLimit();
#endif
			m_angularOnly = ( false );
			m_enableAngularMotor = ( false );
			//m_useSolveConstraintObsolete = ( false/*HINGE_USE_OBSOLETE_SOLVER*/ );
			//m_useOffsetForConstraintFrame = ( true/*HINGE_USE_FRAME_OFFSET*/ );
			m_useReferenceFrameA = ( useReferenceFrameA );
			m_flags = ( 0 );
			m_normalCFM = ( 0 );
			m_normalERP = ( 0 );
			m_stopCFM = ( 0 );
			m_stopERP = ( 0 );
			///not providing rigidbody B means implicitly using worldspace for body B
			m_rbA.m_worldTransform.Apply( ref m_rbAFrame.m_origin, out m_rbBFrame.m_origin );
			//m_rbBFrame.getOrigin() = m_rbA.getCenterOfMassTransform()( m_rbAFrame.getOrigin() );
#if !_BT_USE_CENTER_LIMIT_
			//start with free
			m_lowerLimit = (double)( 1.0f );
			m_upperLimit = (double)( -1.0f );
			m_biasFactor = 0.3f;
			m_relaxationFactor = 1.0f;
			m_limitSoftness = 0.9f;
			m_solveLimit = false;
#endif
			m_referenceSign = m_useReferenceFrameA ? (double)( -1 ) : (double)( 1 );
		}

		internal override void buildJacobian()
		{
			/*
			if( m_useSolveConstraintObsolete )
			{
				m_appliedImpulse = btScalar.BT_ZERO;
				m_accMotorImpulse = btScalar.BT_ZERO;

				btMatrix3x3 tmpm1;
				btMatrix3x3 tmpm2;
				m_rbA.m_worldTransform.m_basis.transpose( out tmpm1 );
				m_rbB.m_worldTransform.m_basis.transpose( out tmpm2 );

				if( !m_angularOnly )
				{
					btVector3 pivotAInW; m_rbA.m_worldTransform.Apply( ref m_rbAFrame.m_origin, out pivotAInW );
					btVector3 pivotBInW; m_rbB.m_worldTransform.Apply( ref m_rbBFrame.m_origin, out pivotBInW );
					btVector3 relPos = pivotBInW - pivotAInW;

					btVector3[] normal = new btVector3[3];
					if( relPos.length2() > btScalar.SIMD_EPSILON )
					{
						relPos.normalized( out normal[0] );
					}
					else
					{
						normal[0] = btVector3.xAxis;
					}

					btVector3.btPlaneSpace1( ref normal[0], out normal[1], out normal[2] );


					for( int i = 0; i < 3; i++ )
					{
						btVector3 pivA, pivB;
						pivotAInW.Sub( ref m_rbA.m_worldTransform.m_origin, out pivA );
						pivotBInW.Sub( ref m_rbB.m_worldTransform.m_origin, out pivB );
                        m_jac[i] = new btJacobianEntry(
						  ref tmpm1,
						  ref tmpm2,
						  ref pivA,
						  ref pivB,
						  ref normal[i],
						  m_rbA.getInvInertiaDiagLocal(),
						  m_rbA.getInvMass(),
						  m_rbB.getInvInertiaDiagLocal(),
						  m_rbB.getInvMass() );
					}
				}

				//calculate two perpendicular jointAxis, orthogonal to hingeAxis
				//these two jointAxis require equal angular velocities for both bodies

				//this is unused for now, it's a todo
				btVector3 jointAxis0local;
				btVector3 jointAxis1local;

				btVector3 tmp; m_rbAFrame.getBasis().getColumn( 2, out tmp );
                btVector3.btPlaneSpace1( ref tmp, out jointAxis0local, out jointAxis1local );

				btVector3 jointAxis0; m_rbA.m_worldTransform.m_basis.Apply( ref jointAxis0local, out jointAxis0 );
				btVector3 jointAxis1; m_rbA.m_worldTransform.m_basis.Apply( ref jointAxis1local, out jointAxis1 );
				btVector3 hingeAxisWorld = m_rbA.m_worldTransform.getBasis() * m_rbAFrame.getBasis().getColumn( 2 );

				m_jacAng[0] = new btJacobianEntry( ref jointAxis0,
					 ref tmpm1,
					  ref tmpm2,
					  m_rbA.getInvInertiaDiagLocal(),
					  m_rbB.getInvInertiaDiagLocal() );

				m_jacAng[1] = new btJacobianEntry( ref jointAxis1,
					 ref tmpm1,
					  ref tmpm2,
					  m_rbA.getInvInertiaDiagLocal(),
					  m_rbB.getInvInertiaDiagLocal() );

				m_jacAng[2] = new btJacobianEntry( ref hingeAxisWorld,
					 ref tmpm1,
					  ref tmpm2,
					  m_rbA.getInvInertiaDiagLocal(),
					  m_rbB.getInvInertiaDiagLocal() );

				// clear accumulator
				m_accLimitImpulse = btScalar.BT_ZERO;

				// test angular limit
				testLimit( ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform );

				//Compute K = J*W*J' for hinge axis
				btVector3 axisA = getRigidBodyA().getCenterOfMassTransform().getBasis() * m_rbAFrame.getBasis().getColumn( 2 );
				m_kHinge = 1.0f / ( getRigidBodyA().computeAngularImpulseDenominator( ref axisA ) +
									 getRigidBodyB().computeAngularImpulseDenominator( ref axisA ) );

			}
			*/
		}


#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		static double btNormalizeAnglePositive( double angle )
		{
			return btScalar.btFmod( btScalar.btFmod( angle, (double)( 2.0 * btScalar.SIMD_PI ) ) + (double)( 2.0 * btScalar.SIMD_PI ), (double)( 2.0 * btScalar.SIMD_PI ) );
		}



		internal static double btShortestAngularDistance( double accAngle, double curAngle )
		{
			double result = btScalar.btNormalizeAngle( btNormalizeAnglePositive( btNormalizeAnglePositive( curAngle ) -
			btNormalizeAnglePositive( accAngle ) ) );
			return result;
		}

		internal static double btShortestAngleUpdate( double accAngle, double curAngle )
		{
			double tol = ( 0.3 );
			double result = btShortestAngularDistance( accAngle, curAngle );

			if( btScalar.btFabs( result ) > tol )
				return curAngle;
			else
				return accAngle + result;

			//return curAngle;
		}


		internal override void getInfo1( ref btConstraintInfo1 info )
		{
			/*
			if( m_useSolveConstraintObsolete )
			{
				info.m_numConstraintRows = 0;
				info.nub = 0;
			}
			else
			*/
			{
				info.m_numConstraintRows = 5; // Fixed 3 linear + 2 angular
				info.nub = 1;
				//always add the row, to avoid computation (data is not available yet)
				//prepare constraint
				testLimit( ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform );
				if( getSolveLimit() || getEnableAngularMotor() )
				{
					info.m_numConstraintRows++; // limit 3rd anguar as well
					info.nub--;
				}

			}
		}

		public void getInfo1NonVirtual( out btConstraintInfo1 info )
		{
			/*
			if( m_useSolveConstraintObsolete )
			{
				info.m_numConstraintRows = 0;
				info.nub = 0;
			}
			else
			*/
			{
				//always add the 'limit' row, to avoid computation (data is not available yet)
				info.m_numConstraintRows = 6; // Fixed 3 linear + 2 angular
				info.nub = 0;
			}
		}

		internal override void getInfo2( btConstraintInfo2 info )
		{
			if( m_useOffsetForConstraintFrame )
			{
				getInfo2InternalUsingFrameOffset( ref info, ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform, ref m_rbA.m_angularVelocity, ref m_rbB.m_angularVelocity );
			}
			/*
			else
			{
				getInfo2Internal( ref info, ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform, ref m_rbA.m_angularVelocity, ref m_rbB.m_angularVelocity );
			}
			*/
		}


		public void getInfo2NonVirtual( ref btConstraintInfo2 info, ref btTransform transA, ref btTransform transB, ref btVector3 angVelA, ref btVector3 angVelB )
		{
			///the regular (virtual) implementation getInfo2 already performs 'testLimit' during getInfo1, so we need to do it now
			testLimit( ref transA, ref transB );

			getInfo2InternalUsingFrameOffset( ref info, ref transA, ref transB, ref angVelA, ref angVelB );
			//getInfo2Internal( ref info, ref transA, ref transB, ref angVelA, ref angVelB );
		}
		/*

				public void getInfo2Internal( ref btConstraintInfo2 info, ref btTransform transA, ref btTransform transB, ref btVector3 angVelA, ref btVector3 angVelB )
				{

					//Debug.Assert( !m_useSolveConstraintObsolete );
					int i, skip = info.rowskip;
					// transforms in world space
					btTransform trA; transA.Apply( ref m_rbAFrame, out trA );
					btTransform trB; transB.Apply( ref m_rbBFrame, out trB );
					// pivot point
					btVector3 pivotAInW; trA.getOrigin( out pivotAInW );
					btVector3 pivotBInW; trB.getOrigin( out pivotBInW );
		#if false
			if (0)
			{
				for (i=0;i<6;i++)
				{
					info.m_J1linearAxis[i*skip]=0;
					info.m_J1linearAxis[i*skip+1]=0;
					info.m_J1linearAxis[i*skip+2]=0;

					info.m_J1angularAxis[i*skip]=0;
					info.m_J1angularAxis[i*skip+1]=0;
					info.m_J1angularAxis[i*skip+2]=0;

					info.m_J2linearAxis[i*skip]=0;
					info.m_J2linearAxis[i*skip+1]=0;
					info.m_J2linearAxis[i*skip+2]=0;

					info.m_J2angularAxis[i*skip]=0;
					info.m_J2angularAxis[i*skip+1]=0;
					info.m_J2angularAxis[i*skip+2]=0;

					info.m_constraintError[i*skip]=0;
				}
			}
		#endif //#if 0
					// linear (all fixed)

					if( !m_angularOnly )
					{
						info.m_J1linearAxis[0] = 1;
						info.m_J1linearAxis[skip + 1] = 1;
						info.m_J1linearAxis[2 * skip + 2] = 1;

						info.m_J2linearAxis[0] = -1;
						info.m_J2linearAxis[skip + 1] = -1;
						info.m_J2linearAxis[2 * skip + 2] = -1;
					}




					btVector3 a1 = pivotAInW - transA.getOrigin();
					{
						btIVector3[] angular0 = ( info.m_J1angularAxis );
						//btIVector3[] angular1 = (btVector3*)( info.m_J1angularAxis + skip );
						//btIVector3[] angular2 = (btVector3*)( info.m_J1angularAxis + 2 * skip );
						btVector3 a1neg = -a1;
						a1neg.getSkewSymmetricMatrix( angular0, angular1, angular2 );
					}
					btVector3 a2 = pivotBInW - transB.getOrigin();
					{
						btIVector3 angular0 = (btVector3*)( info.m_J2angularAxis );
						btIVector3 angular1 = (btVector3*)( info.m_J2angularAxis + skip );
						btIVector3 angular2 = (btVector3*)( info.m_J2angularAxis + 2 * skip );
						a2.getSkewSymmetricMatrix( angular0, angular1, angular2 );
					}
					// linear RHS
					double normalErp = (0!= (m_flags & btHingeFlags.BT_HINGE_FLAGS_ERP_NORM) ) ? m_normalERP : info.erp;

					double k = info.fps * normalErp;
					if( !m_angularOnly )
					{
						for( i = 0; i < 3; i++ )
						{
							info.m_constraintError[i * skip] = k * ( pivotBInW[i] - pivotAInW[i] );
						}
					}
					// make rotations around X and Y equal
					// the hinge axis should be the only unconstrained
					// rotational axis, the angular velocity of the two bodies perpendicular to
					// the hinge axis should be equal. thus the constraint equations are
					//    p*w1 - p*w2 = 0
					//    q*w1 - q*w2 = 0
					// where p and q are unit vectors normal to the hinge axis, and w1 and w2
					// are the angular velocity vectors of the two bodies.
					// get hinge axis (Z)
					btVector3 ax1 = trA.getBasis().getColumn( 2 );
					// get 2 orthos to hinge axis (X, Y)
					btVector3 p = trA.getBasis().getColumn( 0 );
					btVector3 q = trA.getBasis().getColumn( 1 );
					// set the two hinge angular rows 
					int s3 = 3 * info.rowskip;
					int s4 = 4 * info.rowskip;

					info.m_J1angularAxis[s3 + 0] = p[0];
					info.m_J1angularAxis[s3 + 1] = p[1];
					info.m_J1angularAxis[s3 + 2] = p[2];
					info.m_J1angularAxis[s4 + 0] = q[0];
					info.m_J1angularAxis[s4 + 1] = q[1];
					info.m_J1angularAxis[s4 + 2] = q[2];

					info.m_J2angularAxis[s3 + 0] = -p[0];
					info.m_J2angularAxis[s3 + 1] = -p[1];
					info.m_J2angularAxis[s3 + 2] = -p[2];
					info.m_J2angularAxis[s4 + 0] = -q[0];
					info.m_J2angularAxis[s4 + 1] = -q[1];
					info.m_J2angularAxis[s4 + 2] = -q[2];
					// compute the right hand side of the constraint equation. set relative
					// body velocities along p and q to bring the hinge back into alignment.
					// if ax1,ax2 are the unit length hinge axes as computed from body1 and
					// body2, we need to rotate both bodies along the axis u = (ax1 x ax2).
					// if `theta' is the angle between ax1 and ax2, we need an angular velocity
					// along u to cover angle erp*theta in one step :
					//   |angular_velocity| = angle/time = erp*theta / stepsize
					//                      = (erp*fps) * theta
					//    angular_velocity  = |angular_velocity| * (ax1 x ax2) / |ax1 x ax2|
					//                      = (erp*fps) * theta * (ax1 x ax2) / sin(theta)
					// ...as ax1 and ax2 are unit length. if theta is smallish,
					// theta ~= sin(theta), so
					//    angular_velocity  = (erp*fps) * (ax1 x ax2)
					// ax1 x ax2 is in the plane space of ax1, so we project the angular
					// velocity to p and q to find the right hand side.
					btVector3 ax2 = trB.getBasis().getColumn( 2 );
					btVector3 u = ax1.cross( ax2 );
					info.m_constraintError[s3] = k * u.dot( p );
					info.m_constraintError[s4] = k * u.dot( q );
					// check angular limits
					int nrow = 4; // last filled row
					int srow;
					double limit_err = (double)( 0.0 );
					int limit = 0;
					if( getSolveLimit() )
					{
		#if _BT_USE_CENTER_LIMIT_
			limit_err = m_limit.getCorrection() * m_referenceSign;
		#else
						limit_err = m_correction * m_referenceSign;
		#endif
						limit = ( limit_err > (double)( 0.0 ) ) ? 1 : 2;

					}
					// if the hinge has joint limits or motor, add in the extra row
					bool powered = false;
					if( getEnableAngularMotor() )
					{
						powered = true;
					}
					if( limit!= 0 || powered )
					{
						nrow++;
						srow = nrow * info.rowskip;
						info.m_J1angularAxis[srow + 0] = ax1[0];
						info.m_J1angularAxis[srow + 1] = ax1[1];
						info.m_J1angularAxis[srow + 2] = ax1[2];

						info.m_J2angularAxis[srow + 0] = -ax1[0];
						info.m_J2angularAxis[srow + 1] = -ax1[1];
						info.m_J2angularAxis[srow + 2] = -ax1[2];

						double lostop = getLowerLimit();
						double histop = getUpperLimit();
						if( limit != 0 && ( lostop == histop ) )
						{  // the joint motor is ineffective
							powered = false;
						}
						info.m_constraintError[srow] = (double)( 0.0f );
						double currERP = ( m_flags & BT_HINGE_FLAGS_ERP_STOP ) ? m_stopERP : normalErp;
						if( powered )
						{
							if( m_flags & BT_HINGE_FLAGS_CFM_NORM )
							{
								info.cfm[srow] = m_normalCFM;
							}
							double mot_fact = getMotorFactor( m_hingeAngle, lostop, histop, m_motorTargetVelocity, info.fps * currERP );
							info.m_constraintError[srow] += mot_fact * m_motorTargetVelocity * m_referenceSign;
							info.m_lowerLimit[srow] = -m_maxMotorImpulse;
							info.m_upperLimit[srow] = m_maxMotorImpulse;
						}
						if( limit )
						{
							k = info.fps * currERP;
							info.m_constraintError[srow] += k * limit_err;
							if( m_flags & BT_HINGE_FLAGS_CFM_STOP )
							{
								info.cfm[srow] = m_stopCFM;
							}
							if( lostop == histop )
							{
								// limited low and high simultaneously
								info.m_lowerLimit[srow] = -SIMD_INFINITY;
								info.m_upperLimit[srow] = SIMD_INFINITY;
							}
							else if( limit == 1 )
							{ // low limit
								info.m_lowerLimit[srow] = 0;
								info.m_upperLimit[srow] = SIMD_INFINITY;
							}
							else
							{ // high limit
								info.m_lowerLimit[srow] = -SIMD_INFINITY;
								info.m_upperLimit[srow] = 0;
							}
							// bounce (we'll use slider parameter abs(1.0 - m_dampingLimAng) for that)
		#if _BT_USE_CENTER_LIMIT_
					double bounce = m_limit.getRelaxationFactor();
		#else
							double bounce = m_relaxationFactor;
		#endif
							if( bounce > (double)( 0.0 ) )
							{
								double vel = angVelA.dot( ax1 );
								vel -= angVelB.dot( ax1 );
								// only apply bounce if the velocity is incoming, and if the
								// resulting c[] exceeds what we already have.
								if( limit == 1 )
								{   // low limit
									if( vel < 0 )
									{
										double newc = -bounce * vel;
										if( newc > info.m_constraintError[srow] )
										{
											info.m_constraintError[srow] = newc;
										}
									}
								}
								else
								{   // high limit - all those computations are reversed
									if( vel > 0 )
									{
										double newc = -bounce * vel;
										if( newc < info.m_constraintError[srow] )
										{
											info.m_constraintError[srow] = newc;
										}
									}
								}
							}
		#if _BT_USE_CENTER_LIMIT_
					info.m_constraintError[srow] *= m_limit.getBiasFactor();
		#else
							info.m_constraintError[srow] *= m_biasFactor;
		#endif
						} // if(limit)
					} // if angular limit or powered
				}
				*/

		internal void setFrames( ref btTransform frameA, ref btTransform frameB )
		{
			m_rbAFrame = frameA;
			m_rbBFrame = frameB;
			//buildJacobian();
		}


		internal void updateRHS( double timeStep )
		{
			//(void)timeStep;

		}




		internal double getHingeAngle()
		{
			return getHingeAngle( ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform );
		}

		internal double getHingeAngle( ref btTransform transA, ref btTransform transB )
		{
			btVector3 refAxis0 = transA.getBasis() * m_rbAFrame.getBasis().getColumn( 0 );
			btVector3 refAxis1 = transA.getBasis() * m_rbAFrame.getBasis().getColumn( 1 );
			btVector3 swingAxis = transB.getBasis() * m_rbBFrame.getBasis().getColumn( 1 );
			//	double angle = btAtan2Fast(swingAxis.dot(refAxis0), swingAxis.dot(refAxis1));
			double angle = btScalar.btAtan2( swingAxis.dot( refAxis0 ), swingAxis.dot( refAxis1 ) );
			return m_referenceSign * angle;
		}



		internal void testLimit( ref btTransform transA, ref btTransform transB )
		{
			// Compute limit information
			m_hingeAngle = getHingeAngle( ref transA, ref transB );
#if _BT_USE_CENTER_LIMIT_
			m_limit.test( m_hingeAngle );
#else
			m_correction = btScalar.BT_ZERO;
			m_limitSign = btScalar.BT_ZERO;
			m_solveLimit = false;
			if( m_lowerLimit <= m_upperLimit )
			{
				m_hingeAngle = btAdjustAngleToLimits( m_hingeAngle, m_lowerLimit, m_upperLimit );
				if( m_hingeAngle <= m_lowerLimit )
				{
					m_correction = ( m_lowerLimit - m_hingeAngle );
					m_limitSign = 1.0f;
					m_solveLimit = true;
				}
				else if( m_hingeAngle >= m_upperLimit )
				{
					m_correction = m_upperLimit - m_hingeAngle;
					m_limitSign = -1.0f;
					m_solveLimit = true;
				}
			}
#endif
			return;
		}


		static btVector3 vHinge = btVector3.zAxis;

		public void setMotorTarget( ref btQuaternion qAinB, double dt )
		{
			// convert target from body to constraint space
			btMatrix3x3 tmp;
			m_rbBFrame.m_basis.inverse( out tmp );
			btQuaternion tmpq;
			m_rbBFrame.getRotation( out tmpq );
			tmpq.inverse( out tmpq );
			btQuaternion tmpq2;
			tmpq.Mult( ref qAinB, out tmpq2 );
			m_rbAFrame.getRotation( out tmpq );
			btQuaternion qConstraint;// = m_rbBFrame.getRotation().inverse() * qAinB * m_rbAFrame.getRotation();
			tmpq2.Mult( ref tmpq, out qConstraint );
			qConstraint.normalize();

			// extract "pure" hinge component
			btVector3 vNoHinge; btQuaternion.quatRotate( ref qConstraint, ref vHinge, out vNoHinge ); vNoHinge.normalize();
			btQuaternion qNoHinge; btQuaternion.shortestArcQuat( ref vHinge, ref vNoHinge, out qNoHinge );
			qNoHinge.inverse( out tmpq );
			btQuaternion qHinge;// = qNoHinge.inverse() * qConstraint;
			tmpq.Mult( ref qConstraint, out qHinge );
			qHinge.normalize();

			// compute angular target, clamped to limits
			double targetAngle = qHinge.getAngle();
			if( targetAngle > btScalar.SIMD_PI ) // long way around. flip quat and recalculate.
			{
				qHinge.inverse( out qHinge );
				//qHinge = -( qHinge );
				targetAngle = qHinge.getAngle();
			}
			if( qHinge.z < 0 )
				targetAngle = -targetAngle;

			setMotorTarget( targetAngle, dt );
		}

		public void setMotorTarget( double targetAngle, double dt )
		{
#if _BT_USE_CENTER_LIMIT_
			m_limit.fit( targetAngle );
#else
			if( m_lowerLimit < m_upperLimit )
			{
				if( targetAngle < m_lowerLimit )
					targetAngle = m_lowerLimit;
				else if( targetAngle > m_upperLimit )
					targetAngle = m_upperLimit;
			}
#endif
			// compute angular velocity
			double curAngle = getHingeAngle( ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform );
			double dAngle = targetAngle - curAngle;
			m_motorTargetVelocity = dAngle / dt;
		}



		void getInfo2InternalUsingFrameOffset( ref btConstraintInfo2 info, ref btTransform transA, ref btTransform transB, ref btVector3 angVelA, ref btVector3 angVelB )
		{
			//Debug.Assert( !m_useSolveConstraintObsolete );
			//int i;
			// transforms in world space
			btTransform trA; transA.Apply( ref m_rbAFrame, out trA );
			btTransform trB; transB.Apply( ref m_rbBFrame, out trB );
			// pivot point
			//	btVector3 pivotAInW = trA.getOrigin();
			//	btVector3 pivotBInW = trB.getOrigin();

#if true
			// difference between frames in WCS
			btVector3 ofs; trB.m_origin.Sub( trA.m_origin, out ofs );// getOrigin() - trA.getOrigin();
																	 // now get weight factors depending on masses
			double miA = getRigidBodyA().getInvMass();
			double miB = getRigidBodyB().getInvMass();
			bool hasStaticBody = ( miA < btScalar.SIMD_EPSILON ) || ( miB < btScalar.SIMD_EPSILON );
			double miS = miA + miB;
			double factA, factB;
			if( miS > (double)( 0 ) )
			{
				factA = miB / miS;
			}
			else
			{
				factA = (double)( 0.5f );
			}
			factB = (double)( 1.0f ) - factA;
			// get the desired direction of hinge axis
			// as weighted sum of Z-orthos of frameA and frameB in WCS
			btVector3 ax1A = trA.getBasis().getColumn( 2 );
			btVector3 ax1B = trB.getBasis().getColumn( 2 );
			btVector3 ax1 = ax1A * factA + ax1B * factB;
			ax1.normalize();
			// fill first 3 rows 
			// we want: velA + wA x relA == velB + wB x relB
			btTransform bodyA_trans = transA;
			btTransform bodyB_trans = transB;
			//int nrow = 2; // last filled row
			btVector3 tmpA, tmpB, relA, relB, p, q;
			// get vector from bodyB to frameB in WCS
			trB.m_origin.Sub( ref bodyB_trans.m_origin, out relB );
			// get its projection to hinge axis
			btVector3 projB = ax1 * relB.dot( ax1 );
			// get vector directed from bodyB to hinge axis (and orthogonal to it)
			btVector3 orthoB = relB - projB;
			// same for bodyA
			trA.m_origin.Sub( ref bodyA_trans.m_origin, out relA );
			btVector3 projA = ax1 * relA.dot( ax1 );
			btVector3 orthoA = relA - projA;
			btVector3 totalDist = projA - projB;
			// get offset vectors relA and relB
			relA = orthoA + totalDist * factA;
			relB = orthoB - totalDist * factB;
			// now choose average ortho to hinge axis
			p = orthoB * factA + orthoA * factB;
			double len2 = p.length2();
			if( len2 > btScalar.SIMD_EPSILON )
			{
				p /= btScalar.btSqrt( len2 );
			}
			else
			{
				p = trA.getBasis().getColumn( 1 );
			}
			// make one more ortho
			q = ax1.cross( p );
			// fill three rows
			relA.cross( ref p, out tmpA );
			relB.cross( ref p, out tmpB );
			info.m_solverConstraints[0].m_relpos1CrossNormal = tmpA;
			tmpB.Invert( out info.m_solverConstraints[0].m_relpos1CrossNormal ); // = -tmpB;
			relA.cross( ref q, out tmpA );
			relB.cross( ref q, out tmpB );
			if( hasStaticBody && getSolveLimit() )
			{ // to make constraint between static and dynamic objects more rigid
			  // remove wA (or wB) from equation if angular limit is hit
				tmpB *= factB;
				tmpA *= factA;
			}
			info.m_solverConstraints[1].m_relpos1CrossNormal = tmpA;
			tmpB.Invert( out info.m_solverConstraints[1].m_relpos1CrossNormal );
			relA.cross( ref ax1, out tmpA );
			relB.cross( ref ax1, out tmpB );
			if( hasStaticBody )
			{ // to make constraint between static and dynamic objects more rigid
			  // remove wA (or wB) from equation
				tmpB *= factB;
				tmpA *= factA;
			}
			info.m_solverConstraints[2].m_relpos1CrossNormal = tmpA;
			tmpB.Invert( out info.m_solverConstraints[2].m_relpos1CrossNormal );

			double normalErp = ( ( m_flags & btHingeFlags.BT_HINGE_FLAGS_ERP_NORM ) != 0 ) ? m_normalERP : info.erp;
			double k = info.fps * normalErp;

			if( !m_angularOnly )
			{
				info.m_solverConstraints[0].m_contactNormal1 = p;
				info.m_solverConstraints[1].m_contactNormal1 = q;
				info.m_solverConstraints[2].m_contactNormal1 = ax1;

				p.Invert( out info.m_solverConstraints[0].m_contactNormal2 );
				q.Invert( out info.m_solverConstraints[1].m_contactNormal2 );
				ax1.Invert( out info.m_solverConstraints[2].m_contactNormal2 );

				// compute three elements of right hand side

				double rhs = k * p.dot( ofs );
				info.m_solverConstraints[0].m_rhs = rhs;
				rhs = k * q.dot( ofs );
				info.m_solverConstraints[1].m_rhs = rhs;
				rhs = k * ax1.dot( ofs );
				info.m_solverConstraints[2].m_rhs = rhs;
			}
			// the hinge axis should be the only unconstrained
			// rotational axis, the angular velocity of the two bodies perpendicular to
			// the hinge axis should be equal. thus the constraint equations are
			//    p*w1 - p*w2 = 0
			//    q*w1 - q*w2 = 0
			// where p and q are unit vectors normal to the hinge axis, and w1 and w2
			// are the angular velocity vectors of the two bodies.
			//int s3 = 3 * s;
			//int s4 = 4 * s;
			info.m_solverConstraints[3].m_relpos1CrossNormal = p;
			info.m_solverConstraints[3].m_relpos1CrossNormal = q;

			p.Invert( out info.m_solverConstraints[3].m_relpos2CrossNormal );
			q.Invert( out info.m_solverConstraints[3].m_relpos2CrossNormal );
			// compute the right hand side of the constraint equation. set relative
			// body velocities along p and q to bring the hinge back into alignment.
			// if ax1A,ax1B are the unit length hinge axes as computed from bodyA and
			// bodyB, we need to rotate both bodies along the axis u = (ax1 x ax2).
			// if "theta" is the angle between ax1 and ax2, we need an angular velocity
			// along u to cover angle erp*theta in one step :
			//   |angular_velocity| = angle/time = erp*theta / stepsize
			//                      = (erp*fps) * theta
			//    angular_velocity  = |angular_velocity| * (ax1 x ax2) / |ax1 x ax2|
			//                      = (erp*fps) * theta * (ax1 x ax2) / sin(theta)
			// ...as ax1 and ax2 are unit length. if theta is smallish,
			// theta ~= sin(theta), so
			//    angular_velocity  = (erp*fps) * (ax1 x ax2)
			// ax1 x ax2 is in the plane space of ax1, so we project the angular
			// velocity to p and q to find the right hand side.
			k = info.fps * normalErp;//??

			btVector3 u = ax1A.cross( ax1B );
			info.m_solverConstraints[3].m_rhs = k * u.dot( p );
			info.m_solverConstraints[4].m_rhs = k * u.dot( q );
#endif
			// check angular limits
			//int nrow = 4; // last filled row
			//int srow;
			double limit_err = (double)( 0.0 );
			int limit = 0;
			if( getSolveLimit() )
			{
#if _BT_USE_CENTER_LIMIT_
				limit_err = m_limit.getCorrection() * m_referenceSign;
#else
				limit_err = m_correction * m_referenceSign;
#endif
				limit = ( limit_err > (double)( 0.0 ) ) ? 1 : 2;

			}
			// if the hinge has joint limits or motor, add in the extra row
			bool powered = false;
			if( getEnableAngularMotor() )
			{
				powered = true;
			}
			if( limit != 0 || powered )
			{
				info.m_solverConstraints[5].m_relpos1CrossNormal = ax1;

				ax1.Invert( out info.m_solverConstraints[5].m_relpos2CrossNormal );

				double lostop = getLowerLimit();
				double histop = getUpperLimit();
				if( limit != 0 && ( lostop == histop ) )
				{  // the joint motor is ineffective
					powered = false;
				}
				info.m_solverConstraints[5].m_rhs = (double)( 0.0f );
				double currERP = ( ( m_flags & btHingeFlags.BT_HINGE_FLAGS_ERP_STOP ) != 0 ) ? m_stopERP : normalErp;
				if( powered )
				{
					if( ( m_flags & btHingeFlags.BT_HINGE_FLAGS_CFM_NORM ) != 0 )
					{
						info.m_solverConstraints[5].m_cfm = m_normalCFM;
					}
					double mot_fact = getMotorFactor( m_hingeAngle, lostop, histop, m_motorTargetVelocity, info.fps * currERP );
					info.m_solverConstraints[5].m_rhs += mot_fact * m_motorTargetVelocity * m_referenceSign;
					info.m_solverConstraints[5].m_lowerLimit = -m_maxMotorImpulse;
					info.m_solverConstraints[5].m_upperLimit = m_maxMotorImpulse;
				}
				if( limit != 0 )
				{
					k = info.fps * currERP;
					info.m_solverConstraints[5].m_rhs += k * limit_err;
					if( ( m_flags & btHingeFlags.BT_HINGE_FLAGS_CFM_STOP ) != 0 )
					{
						info.m_solverConstraints[5].m_cfm = m_stopCFM;
					}
					if( lostop == histop )
					{
						// limited low and high simultaneously
						info.m_solverConstraints[5].m_lowerLimit = btScalar.BT_MIN_FLOAT;
						info.m_solverConstraints[5].m_upperLimit = btScalar.BT_MAX_FLOAT;
					}
					else if( limit == 1 )
					{ // low limit
						info.m_solverConstraints[5].m_lowerLimit = 0;
						info.m_solverConstraints[5].m_upperLimit = btScalar.BT_MAX_FLOAT;
					}
					else
					{ // high limit
						info.m_solverConstraints[5].m_lowerLimit = btScalar.BT_MIN_FLOAT;
						info.m_solverConstraints[5].m_upperLimit = 0;
					}
					// bounce (we'll use slider parameter abs(1.0 - m_dampingLimAng) for that)
#if _BT_USE_CENTER_LIMIT_
					double bounce = m_limit.getRelaxationFactor();
#else
					double bounce = m_relaxationFactor;
#endif
					if( bounce > (double)( 0.0 ) )
					{
						double vel = angVelA.dot( ax1 );
						vel -= angVelB.dot( ax1 );
						// only apply bounce if the velocity is incoming, and if the
						// resulting c[] exceeds what we already have.
						if( limit == 1 )
						{   // low limit
							if( vel < 0 )
							{
								double newc = -bounce * vel;
								if( newc > info.m_solverConstraints[5].m_rhs )
								{
									info.m_solverConstraints[5].m_rhs = newc;
								}
							}
						}
						else
						{   // high limit - all those computations are reversed
							if( vel > 0 )
							{
								double newc = -bounce * vel;
								if( newc < info.m_solverConstraints[5].m_rhs )
								{
									info.m_solverConstraints[5].m_rhs = newc;
								}
							}
						}
					}
#if _BT_USE_CENTER_LIMIT_
					info.m_solverConstraints[5].m_rhs *= m_limit.getBiasFactor();
#else
					info.m_solverConstraints[5].m_rhs *= m_biasFactor;
#endif
				} // if(limit)
			} // if angular limit or powered
		}


		///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
		///If no axis is provided, it uses the default axis for this constraint.
		internal override void setParam( btTypedConstraint.btConstraintParams num, double value, int axis = -1 )
		{
			if( ( axis == -1 ) || ( axis == 5 ) )
			{
				switch( num )
				{
					case btConstraintParams.BT_CONSTRAINT_STOP_ERP:
						m_stopERP = value;
						m_flags |= btHingeFlags.BT_HINGE_FLAGS_ERP_STOP;
						break;
					case btConstraintParams.BT_CONSTRAINT_STOP_CFM:
						m_stopCFM = value;
						m_flags |= btHingeFlags.BT_HINGE_FLAGS_CFM_STOP;
						break;
					case btConstraintParams.BT_CONSTRAINT_CFM:
						m_normalCFM = value;
						m_flags |= btHingeFlags.BT_HINGE_FLAGS_CFM_NORM;
						break;
					case btConstraintParams.BT_CONSTRAINT_ERP:
						m_normalERP = value;
						m_flags |= btHingeFlags.BT_HINGE_FLAGS_ERP_NORM;
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
			if( ( axis == -1 ) || ( axis == 5 ) )
			{
				switch( num )
				{
					case btConstraintParams.BT_CONSTRAINT_STOP_ERP:
						btAssertConstrParams( ( m_flags & btHingeFlags.BT_HINGE_FLAGS_ERP_STOP ) != 0 );
						retVal = m_stopERP;
						break;
					case btConstraintParams.BT_CONSTRAINT_STOP_CFM:
						btAssertConstrParams( ( m_flags & btHingeFlags.BT_HINGE_FLAGS_CFM_STOP ) != 0 );
						retVal = m_stopCFM;
						break;
					case btConstraintParams.BT_CONSTRAINT_CFM:
						btAssertConstrParams( ( m_flags & btHingeFlags.BT_HINGE_FLAGS_CFM_NORM ) != 0 );
						retVal = m_normalCFM;
						break;
					case btConstraintParams.BT_CONSTRAINT_ERP:
						btAssertConstrParams( ( m_flags & btHingeFlags.BT_HINGE_FLAGS_ERP_NORM ) != 0 );
						retVal = m_normalERP;
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


	};


	//only for backward compatibility
#if BT_BACKWARDS_COMPATIBLE_SERIALIZATION
///this structure is not used, except for loading pre-2.82 .bullet files
struct	btHingeConstraintDoubleData
{
	btTypedConstraintData	m_typeConstraintData;
	btTransformDoubleData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btTransformDoubleData m_rbBFrame;
	int			m_useReferenceFrameA;
	int			m_angularOnly;
	int			m_enableAngularMotor;
	float	m_motorTargetVelocity;
	float	m_maxMotorImpulse;

	float	m_lowerLimit;
	float	m_upperLimit;
	float	m_limitSoftness;
	float	m_biasFactor;
	float	m_relaxationFactor;

};
#endif //BT_BACKWARDS_COMPATIBLE_SERIALIZATION




	///The getAccumulatedHingeAngle returns the accumulated hinge angle, taking rotation across the -PI/PI boundary into account
	internal class btHingeAccumulatedAngleConstraint : btHingeConstraint
	{
		protected double m_accumulatedAngle;



		public btHingeAccumulatedAngleConstraint( btRigidBody rbA, btRigidBody rbB, ref btVector3 pivotInA, ref btVector3 pivotInB, ref btVector3 axisInA, ref btVector3 axisInB, bool useReferenceFrameA = false )
				: base( rbA, rbB, ref pivotInA, ref pivotInB, ref axisInA, ref axisInB, useReferenceFrameA )
		{
			m_accumulatedAngle = getHingeAngle();
		}

		public btHingeAccumulatedAngleConstraint( btRigidBody rbA, ref btVector3 pivotInA, ref btVector3 axisInA, bool useReferenceFrameA = false )
		: base( rbA, ref pivotInA, ref axisInA, useReferenceFrameA )
		{
			m_accumulatedAngle = getHingeAngle();
		}

		public btHingeAccumulatedAngleConstraint( btRigidBody rbA, btRigidBody rbB, ref btTransform rbAFrame, ref btTransform rbBFrame, bool useReferenceFrameA = false )
		: base( rbA, rbB, ref rbAFrame, ref rbBFrame, useReferenceFrameA )
		{
			m_accumulatedAngle = getHingeAngle();
		}

		public btHingeAccumulatedAngleConstraint( btRigidBody rbA, ref btTransform rbAFrame, bool useReferenceFrameA = false )
		: base( rbA, ref rbAFrame, useReferenceFrameA )
		{
			m_accumulatedAngle = getHingeAngle();
		}

		//#define HINGE_USE_OBSOLETE_SOLVER false


		double getAccumulatedHingeAngle()
		{
			double hingeAngle = getHingeAngle();
			m_accumulatedAngle = btShortestAngleUpdate( m_accumulatedAngle, hingeAngle );
			return m_accumulatedAngle;
		}
		void setAccumulatedHingeAngle( double accAngle )
		{
			m_accumulatedAngle = accAngle;
		}

		void getInfo1( btConstraintInfo1 info )
		{
			//update m_accumulatedAngle
			double curHingeAngle = getHingeAngle();
			m_accumulatedAngle = btShortestAngleUpdate( m_accumulatedAngle, curHingeAngle );
			base.getInfo1( ref info );
		}

	};

#if SERIALIZE_DONE
struct btHingeConstraintFloatData
{
	btTypedConstraintData m_typeConstraintData;
	btTransformFloatData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btTransformFloatData m_rbBFrame;
	int m_useReferenceFrameA;
	int m_angularOnly;

	int m_enableAngularMotor;
	float m_motorTargetVelocity;
	float m_maxMotorImpulse;

	float m_lowerLimit;
	float m_upperLimit;
	float m_limitSoftness;
	float m_biasFactor;
	float m_relaxationFactor;

};



///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btHingeConstraintDoubleData2
{
	btTypedConstraintDoubleData m_typeConstraintData;
	btTransformDoubleData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btTransformDoubleData m_rbBFrame;
	int m_useReferenceFrameA;
	int m_angularOnly;
	int m_enableAngularMotor;
	double m_motorTargetVelocity;
	double m_maxMotorImpulse;

	double m_lowerLimit;
	double m_upperLimit;
	double m_limitSoftness;
	double m_biasFactor;
	double m_relaxationFactor;
	char m_padding1[4];

};




public int btHingeConstraint::calculateSerializeBufferSize()
{
	return sizeof( btHingeConstraintData );
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
public string btHingeConstraint::serialize( object dataBuffer, btSerializer* serializer )
{
	btHingeConstraintData* hingeData = (btHingeConstraintData*)dataBuffer;
	btTypedConstraint::serialize( &hingeData.m_typeConstraintData, serializer );

	m_rbAFrame.serialize( hingeData.m_rbAFrame );
	m_rbBFrame.serialize( hingeData.m_rbBFrame );

	hingeData.m_angularOnly = m_angularOnly;
	hingeData.m_enableAngularMotor = m_enableAngularMotor;
	hingeData.m_maxMotorImpulse = float( m_maxMotorImpulse );
	hingeData.m_motorTargetVelocity = float( m_motorTargetVelocity );
	hingeData.m_useReferenceFrameA = m_useReferenceFrameA;
#if _BT_USE_CENTER_LIMIT_
	hingeData.m_lowerLimit = float(m_limit.getLow());
	hingeData.m_upperLimit = float(m_limit.getHigh());
	hingeData.m_limitSoftness = float(m_limit.getSoftness());
	hingeData.m_biasFactor = float(m_limit.getBiasFactor());
	hingeData.m_relaxationFactor = float(m_limit.getRelaxationFactor());
#else
	hingeData.m_lowerLimit = float( m_lowerLimit );
	hingeData.m_upperLimit = float( m_upperLimit );
	hingeData.m_limitSoftness = float( m_limitSoftness );
	hingeData.m_biasFactor = float( m_biasFactor );
	hingeData.m_relaxationFactor = float( m_relaxationFactor );
#endif

	return btHingeConstraintDataName;
}
#endif

}
