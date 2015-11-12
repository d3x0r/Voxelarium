/*
Bullet Continuous Collision Detection and Physics Library
btConeTwistConstraint is Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marcus Hennix
*/



/*
Overview:

btConeTwistConstraint can be used to simulate ragdoll joints (upper arm, leg etc).
It is a fixed translation, 3 degree-of-freedom (DOF) rotational "joint".
It divides the 3 rotational DOFs into swing (movement within a cone) and twist.
Swing is divided into swing1 and swing2 which can have different limits, giving an elliptical shape.
(Note: the cone's base isn't flat, so this ellipse is "embedded" on the surface of a sphere.)

In the contraint's frame of reference:
twist is along the x-axis,
and swing 1 and 2 are along the z and y axes respectively.
*/


using Bullet.LinearMath;
using Bullet.Types;
using System;
using System.Diagnostics;

namespace Bullet.Dynamics.ConstraintSolver
{

	/*
	#if BT_USE_DOUBLE_PRECISION
	#define btConeTwistConstraintData2	btConeTwistConstraintDoubleData
	#define btConeTwistConstraintDataName	"btConeTwistConstraintDoubleData"
	#else
	#define btConeTwistConstraintData2	btConeTwistConstraintData 
	#define btConeTwistConstraintDataName	"btConeTwistConstraintData" 
	#endif //BT_USE_DOUBLE_PRECISION

	*/


	///btConeTwistConstraint can be used to simulate ragdoll joints (upper arm, leg etc)
	public class btConeTwistConstraint : btTypedConstraint
	{
		//#define CONETWIST_USE_OBSOLETE_SOLVER true
		const bool CONETWIST_USE_OBSOLETE_SOLVER = false;
		const double CONETWIST_DEF_FIX_THRESH = (double)( .05f );
		static btVector3 vTwist = btVector3.xAxis; // twist axis in constraint's space

		internal enum btConeTwistFlags
		{
			BT_CONETWIST_FLAGS_LIN_CFM = 1,
			BT_CONETWIST_FLAGS_LIN_ERP = 2,
			BT_CONETWIST_FLAGS_ANG_CFM = 4
		};

		//internal btJacobianEntry[] m_jac = new btJacobianEntry[3]; //3 orthogonal linear constraints

		internal btTransform m_rbAFrame;
		internal btTransform m_rbBFrame;

		double m_limitSoftness;
		double m_biasFactor;
		double m_relaxationFactor;

		double m_damping;

		double m_swingSpan1;
		double m_swingSpan2;
		double m_twistSpan;

		double m_fixThresh;

		btVector3 m_swingAxis;
		btVector3 m_twistAxis;

		double m_kSwing;
		double m_kTwist;

		double m_twistLimitSign;
		double m_swingCorrection;
		double m_twistCorrection;

		double m_twistAngle;

		/* old jacobian build 
		double m_accSwingLimitImpulse;
		double m_accTwistLimitImpulse;
		*/

		bool m_angularOnly;
		bool m_solveTwistLimit;
		bool m_solveSwingLimit;

		bool m_useSolveConstraintObsolete;

		// not yet used...
		double m_swingLimitRatio;
		double m_twistLimitRatio;
		btVector3 m_twistAxisA;

		// motor
		bool m_bMotorEnabled;
		bool m_bNormalizedMotorStrength;
		btQuaternion m_qTarget;
		double m_maxMotorImpulse;
		//btVector3 m_accMotorImpulse;  // obsolete jacobian variable

		// parameters
		btConeTwistFlags m_flags;
		double m_linCFM;
		double m_linERP;
		double m_angCFM;


		public void setAngularOnly( bool angularOnly )
		{
			m_angularOnly = angularOnly;
		}
		public bool getAngularOnly()
		{
			return m_angularOnly;
		}

		public void setLimit( int limitIndex, double limitValue )
		{
			switch( limitIndex )
			{
				case 3:
					{
						m_twistSpan = limitValue;
						break;
					}
				case 4:
					{
						m_swingSpan2 = limitValue;
						break;
					}
				case 5:
					{
						m_swingSpan1 = limitValue;
						break;
					}
				default:
					{
					}
					break;
			};
		}

		public double getLimit( int limitIndex )
		{
			switch( limitIndex )
			{
				case 3:
					{
						return m_twistSpan;
					}
				case 4:
					{
						return m_swingSpan2;
					}
				case 5:
					{
						return m_swingSpan1;
					}
				default:
					{
						Debug.Assert( false, "Invalid limitIndex specified for btConeTwistConstraint" );
						return 0.0;
					}
			};
		}

		// setLimit(), a few notes:
		// _softness:
		//		0.1, recommend ~0.8.1.
		//		describes % of limits where movement is free.
		//		beyond this softness %, the limit is gradually enforced until the "hard" (1.0) limit is reached.
		// _biasFactor:
		//		0.1?, recommend 0.3 +/-0.3 or so.
		//		strength with which constraint resists zeroth order (angular, not angular velocity) limit violation.
		// __relaxationFactor:
		//		0.1, recommend to stay near 1.
		//		the lower the value, the less the constraint will fight velocities which violate the angular limits.
		void setLimit( double _swingSpan1, double _swingSpan2, double _twistSpan, double _softness = 1, double _biasFactor = 0.3f, double _relaxationFactor = 1.0f )
		{
			m_swingSpan1 = _swingSpan1;
			m_swingSpan2 = _swingSpan2;
			m_twistSpan = _twistSpan;

			m_limitSoftness = _softness;
			m_biasFactor = _biasFactor;
			m_relaxationFactor = _relaxationFactor;
		}

		//internal btTransform getAFrame() { return m_rbAFrame; }
		//internal btTransform getBFrame() { return m_rbBFrame; }

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal bool getSolveTwistLimit()
		{
			return m_solveTwistLimit;
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal bool getSolveSwingLimit()
		{
			return m_solveTwistLimit;
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal double getTwistLimitSign()
		{
			return m_twistLimitSign;
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal double getSwingSpan1()
		{
			return m_swingSpan1;
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal double getSwingSpan2()
		{
			return m_swingSpan2;
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal double getTwistSpan()
		{
			return m_twistSpan;
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public double getLimitSoftness()
		{
			return m_limitSoftness;
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public double getBiasFactor()
		{
			return m_biasFactor;
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public double getRelaxationFactor()
		{
			return m_relaxationFactor;
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public double getTwistAngle()
		{
			return m_twistAngle;
		}
		public bool isPastSwingLimit() { return m_solveSwingLimit; }
		public double getDamping() { return m_damping; }

		public void setDamping( double damping ) { m_damping = damping; }

		public void enableMotor( bool b ) { m_bMotorEnabled = b; }
		public bool isMotorEnabled() { return m_bMotorEnabled; }
		public double getMaxMotorImpulse() { return m_maxMotorImpulse; }
		public bool isMaxMotorImpulseNormalized() { return m_bNormalizedMotorStrength; }
		public void setMaxMotorImpulse( double maxMotorImpulse ) { m_maxMotorImpulse = maxMotorImpulse; m_bNormalizedMotorStrength = false; }
		public void setMaxMotorImpulseNormalized( double maxMotorImpulse ) { m_maxMotorImpulse = maxMotorImpulse; m_bNormalizedMotorStrength = true; }
		public btQuaternion getMotorTarget() { return m_qTarget; }
		public void getMotorTarget( out btQuaternion result ) { result = m_qTarget; }

		public double getFixThresh() { return m_fixThresh; }
		public void setFixThresh( double fixThresh ) { m_fixThresh = fixThresh; }

		//internal void setMotorTarget( btQuaternion &q);

		// same as above, but q is the desired rotation of frameA wrt frameB in constraint space
		//internal void setMotorTargetInConstraintSpace( btQuaternion &q);

		//btVector3 GetPointForAngle( double fAngleInRadians, double fLength );


			/*
		btTransform getFrameOffsetA()
		{
			return m_rbAFrame;
		}

		btTransform getFrameOffsetB()
		{
			return m_rbBFrame;
		}
		*/
		internal btConeTwistFlags getFlags()
		{
			return m_flags;
		}


#if SERLIALIZE_DONE
		virtual int calculateSerializeBufferSize();

		///fills the dataBuffer and returns the struct name (and 0 on failure)
		virtual string serialize( object dataBuffer, btSerializer* serializer );
#endif


		public double computeAngularImpulseDenominator( ref btVector3 axis, btMatrix3x3 invInertiaWorld )
		{
			btVector3 vec = axis * invInertiaWorld;
			return axis.dot( vec );
		}


		public btConeTwistConstraint( btRigidBody rbA, btRigidBody rbB,
											 ref btTransform rbAFrame, ref btTransform rbBFrame )
											 : base( btObjectTypes.CONETWIST_CONSTRAINT_TYPE, rbA, rbB )
		{
			m_rbAFrame = ( rbAFrame ); m_rbBFrame = ( rbBFrame );
			m_angularOnly = ( false );
			m_useSolveConstraintObsolete = ( CONETWIST_USE_OBSOLETE_SOLVER );
			init();
		}

		public btConeTwistConstraint( btRigidBody rbA, ref btTransform rbAFrame )
												: base( btObjectTypes.CONETWIST_CONSTRAINT_TYPE, rbA )
		{
			m_rbAFrame = ( rbAFrame );
			m_angularOnly = ( false );
			m_useSolveConstraintObsolete = ( CONETWIST_USE_OBSOLETE_SOLVER );
			m_rbBFrame = m_rbAFrame;
			m_rbBFrame.setOrigin( ref btVector3.Zero );
			init();
		}


		void init()
		{
			m_angularOnly = false;
			m_solveTwistLimit = false;
			m_solveSwingLimit = false;
			m_bMotorEnabled = false;
			m_maxMotorImpulse = btScalar.BT_NEG_ONE;


			setLimit( ( btScalar.BT_LARGE_FLOAT ), ( btScalar.BT_LARGE_FLOAT ), btScalar.BT_LARGE_FLOAT );
			m_damping = (double)( 0.01 );
			m_fixThresh = CONETWIST_DEF_FIX_THRESH;
			m_flags = 0;
			m_linCFM = (double)( 0 );
			m_linERP = (double)( 0.7f );
			m_angCFM = (double)( 0 );
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
				info.m_numConstraintRows = 3;
				info.nub = 3;
				calcAngleInfo2( ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform, ref m_rbA.m_invInertiaTensorWorld, ref m_rbB.m_invInertiaTensorWorld );
				if( m_solveSwingLimit )
				{
					info.m_numConstraintRows++;
					info.nub--;
					if( ( m_swingSpan1 < m_fixThresh ) && ( m_swingSpan2 < m_fixThresh ) )
					{
						info.m_numConstraintRows++;
						info.nub--;
					}
				}
				if( m_solveTwistLimit )
				{
					info.m_numConstraintRows++;
					info.nub--;
				}
			}
		}

		void getInfo1NonVirtual( ref btConstraintInfo1 info )
		{
			//always reserve 6 rows: object transform is not available on SPU
			info.m_numConstraintRows = 6;
			info.nub = 0;
		}


		internal override void getInfo2( btConstraintInfo2 info )
		{
			getInfo2NonVirtual( ref info, ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform, ref m_rbA.m_invInertiaTensorWorld, ref m_rbB.m_invInertiaTensorWorld );
		}

		void getInfo2NonVirtual( ref btConstraintInfo2 info, ref btTransform transA, ref btTransform transB, ref btMatrix3x3 invInertiaWorldA, ref btMatrix3x3 invInertiaWorldB )
		{
			calcAngleInfo2( ref transA, ref transB, ref invInertiaWorldA, ref invInertiaWorldB );

			Debug.Assert( !m_useSolveConstraintObsolete );
			// set jacobian
			info.m_solverConstraints[0].m_contactNormal1 = btVector3.xAxis;
			info.m_solverConstraints[1].m_contactNormal1 = btVector3.yAxis;
			info.m_solverConstraints[2].m_contactNormal1 = btVector3.zAxis;

			//info.m_J1linearAxis = 1;
			//info.m_J1linearAxis[info.rowskip + 1] = 1;
			//info.m_J1linearAxis[2 * info.rowskip + 2] = 1;
			btVector3 a1; transA.m_basis.Apply( ref m_rbAFrame.m_origin, out a1 );
			{
				//btVector3* angular0 = (btVector3*)( info.m_J1angularAxis );
				//btVector3* angular1 = (btVector3*)( info.m_J1angularAxis + info.rowskip );
				//btVector3* angular2 = (btVector3*)( info.m_J1angularAxis + 2 * info.rowskip );
				btVector3 a1neg; a1.Invert( out a1neg );
				a1neg.getSkewSymmetricMatrix( out info.m_solverConstraints[0].m_contactNormal1
					, out info.m_solverConstraints[1].m_contactNormal1
					, out info.m_solverConstraints[2].m_contactNormal1 );
			}
			info.m_solverConstraints[0].m_contactNormal2 = -btVector3.xAxis;
			info.m_solverConstraints[1].m_contactNormal2 = -btVector3.yAxis;
			info.m_solverConstraints[2].m_contactNormal2 = -btVector3.zAxis;
			//info.m_J2linearAxis[0] = -1;
			//info.m_J2linearAxis[info.rowskip + 1] = -1;
			//info.m_J2linearAxis[2 * info.rowskip + 2] = -1;
			btVector3 a2; transB.m_basis.Apply( ref m_rbBFrame.m_origin, out a2 );
			{
				a2.getSkewSymmetricMatrix( out info.m_solverConstraints[0].m_contactNormal1
					, out info.m_solverConstraints[1].m_contactNormal1
					, out info.m_solverConstraints[2].m_contactNormal1 );
			}
			// set right hand side
			double linERP = ( m_flags & btConeTwistFlags.BT_CONETWIST_FLAGS_LIN_ERP ) != 0 ? m_linERP : info.erp;
			double k = info.fps * linERP;
			int j;
			for( j = 0; j < 3; j++ )
			{
				info.m_solverConstraints[j].m_rhs = k * ( a2[j] + transB.m_origin[j] - a1[j] - transA.m_origin[j] );
				info.m_solverConstraints[j].m_lowerLimit = btScalar.BT_MIN_FLOAT;
				info.m_solverConstraints[j].m_upperLimit = btScalar.BT_MAX_FLOAT;
				if( ( m_flags & btConeTwistFlags.BT_CONETWIST_FLAGS_LIN_CFM ) != 0 )
				{
					info.m_solverConstraints[j].m_cfm = m_linCFM;
				}
			}
			int row = 3;
			//int srow = row * info.rowskip;
			btVector3 ax1;
			// angular limits
			if( m_solveSwingLimit )
			{
				//double* J1 = info.m_J1angularAxis;
				//double* J2 = info.m_J2angularAxis;
				if( ( m_swingSpan1 < m_fixThresh ) && ( m_swingSpan2 < m_fixThresh ) )
				{
					btTransform trA; transA.Apply( ref m_rbAFrame, out trA );
					btVector3 p; trA.m_basis.getColumn( 1, out p );
					btVector3 q; trA.m_basis.getColumn( 2, out q );
					int row1 = row + 1;
					//int srow1 = srow + info.rowskip;
					info.m_solverConstraints[row].m_relpos1CrossNormal = p;
					info.m_solverConstraints[row1].m_relpos1CrossNormal = q;
					p.Invert( out info.m_solverConstraints[row].m_relpos2CrossNormal );
					q.Invert( out info.m_solverConstraints[row1].m_relpos2CrossNormal );
					double fact = info.fps * m_relaxationFactor;
					info.m_solverConstraints[row].m_rhs = fact * m_swingAxis.dot( p );
					info.m_solverConstraints[row1].m_rhs = fact * m_swingAxis.dot( q );
					info.m_solverConstraints[row].m_lowerLimit = btScalar.BT_MIN_FLOAT;
					info.m_solverConstraints[row1].m_upperLimit = btScalar.BT_MAX_FLOAT;
					info.m_solverConstraints[row].m_lowerLimit = btScalar.BT_MIN_FLOAT;
					info.m_solverConstraints[row1].m_upperLimit = btScalar.BT_MAX_FLOAT;
					row = row1 + 1;
					//srow = srow1 + info.rowskip;
				}
				else
				{
					ax1 = m_swingAxis * m_relaxationFactor * m_relaxationFactor;
					info.m_solverConstraints[row].m_relpos1CrossNormal = ax1;
					ax1.Invert( out info.m_solverConstraints[row].m_relpos2CrossNormal );
					k = info.fps * m_biasFactor;

					info.m_solverConstraints[row].m_rhs = k * m_swingCorrection;
					if( ( m_flags & btConeTwistFlags.BT_CONETWIST_FLAGS_ANG_CFM ) != 0 )
					{
						info.m_solverConstraints[row].m_cfm = m_angCFM;
					}
					// m_swingCorrection is always positive or 0
					info.m_solverConstraints[row].m_lowerLimit = 0;
					info.m_solverConstraints[row].m_upperLimit = ( m_bMotorEnabled && m_maxMotorImpulse >= 0.0f ) ? m_maxMotorImpulse : btScalar.BT_MAX_FLOAT;
					//srow += info.rowskip;
					row++;
				}
			}
			if( m_solveTwistLimit )
			{
				ax1 = m_twistAxis * m_relaxationFactor * m_relaxationFactor;
				info.m_solverConstraints[row].m_relpos1CrossNormal = ax1;
				ax1.Invert( out info.m_solverConstraints[row].m_relpos2CrossNormal );
				k = info.fps * m_biasFactor;
				info.m_solverConstraints[row].m_rhs = k * m_twistCorrection;
				if( ( m_flags & btConeTwistFlags.BT_CONETWIST_FLAGS_ANG_CFM ) != 0 )
				{
					info.m_solverConstraints[row].m_cfm = m_angCFM;
				}
				if( m_twistSpan > 0.0f )
				{

					if( m_twistCorrection > 0.0f )
					{
						info.m_solverConstraints[row].m_lowerLimit = 0;
						info.m_solverConstraints[row].m_upperLimit = btScalar.BT_MAX_FLOAT;
					}
					else
					{
						info.m_solverConstraints[row].m_lowerLimit = btScalar.BT_MIN_FLOAT;
						info.m_solverConstraints[row].m_upperLimit = 0;
					}
				}
				else
				{
					info.m_solverConstraints[row].m_lowerLimit = btScalar.BT_MIN_FLOAT;
					info.m_solverConstraints[row].m_upperLimit = btScalar.BT_MAX_FLOAT;
				}
				//srow += info.rowskip;
				row++;
			}
		}



		internal override void buildJacobian()
		{
			/*
			if( m_useSolveConstraintObsolete )
			{
				m_appliedImpulse = btScalar.BT_ZERO;
				m_accTwistLimitImpulse = btScalar.BT_ZERO;
				m_accSwingLimitImpulse = btScalar.BT_ZERO;
				m_accMotorImpulse = btVector3.Zero;

				if( !m_angularOnly )
				{
					btVector3 pivotAInW = m_rbA.m_worldTransform * m_rbAFrame.m_origin;
					btVector3 pivotBInW = m_rbB.m_worldTransform * m_rbBFrame.m_origin;
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
						m_jac[i] = new btJacobianEntry(
						  m_rbA.m_worldTransform.m_basis.transpose(),
						  m_rbB.m_worldTransform.m_basis.transpose(),
						  pivotAInW - m_rbA.m_worldTransform.m_origin,
						  pivotBInW - m_rbB.m_worldTransform.m_origin,
						  normal[i],
						  m_rbA.getInvInertiaDiagLocal(),
						  m_rbA.getInvMass(),
						  m_rbB.getInvInertiaDiagLocal(),
						  m_rbB.getInvMass() );
					}
				}

				calcAngleInfo2( m_rbA.m_worldTransform, m_rbB.m_worldTransform, m_rbA.m_invInertiaTensorWorld, m_rbB.m_invInertiaTensorWorld );
			}
			*/
		}


		/*
		void solveConstraintObsolete( btSolverBody bodyA, btSolverBody bodyB, double timeStep )
		{
			if( m_useSolveConstraintObsolete )
			{
				btVector3 pivotAInW = m_rbA.m_worldTransform * m_rbAFrame.m_origin;
				btVector3 pivotBInW = m_rbB.m_worldTransform * m_rbBFrame.m_origin;

				double tau = (double)( 0.3 );

				//linear part
				if( !m_angularOnly )
				{
					btVector3 rel_pos1 = pivotAInW - m_rbA.m_worldTransform.m_origin;
					btVector3 rel_pos2 = pivotBInW - m_rbB.m_worldTransform.m_origin;

					btVector3 vel1;
					bodyA.internalGetVelocityInLocalPointObsolete( rel_pos1, vel1 );
					btVector3 vel2;
					bodyB.internalGetVelocityInLocalPointObsolete( rel_pos2, vel2 );
					btVector3 vel = vel1 - vel2;

					for( int i = 0; i < 3; i++ )
					{		
						btIVector3 normal = m_jac[i].m_linearJointAxis;
						double jacDiagABInv = btScalar.BT_ONE / m_jac[i].getDiagonal();

						double rel_vel;
						rel_vel = normal.dot( vel );
						//positional error (zeroth order error)
						double depth = -( pivotAInW - pivotBInW ).dot( normal ); //this is the error projected on the normal
						double impulse = depth * tau / timeStep * jacDiagABInv - rel_vel * jacDiagABInv;
						m_appliedImpulse += impulse;

						btVector3 ftorqueAxis1 = rel_pos1.cross( normal );
						btVector3 ftorqueAxis2 = rel_pos2.cross( normal );
						bodyA.internalApplyImpulse( normal * m_rbA.getInvMass(), m_rbA.m_invInertiaTensorWorld * ftorqueAxis1, impulse );
						bodyB.internalApplyImpulse( normal * m_rbB.getInvMass(), m_rbB.m_invInertiaTensorWorld * ftorqueAxis2, -impulse );

					}
				}

				// apply motor
				if( m_bMotorEnabled )
				{
					// compute current and predicted transforms
					btTransform trACur = m_rbA.m_worldTransform;
					btTransform trBCur = m_rbB.m_worldTransform;
					btVector3 omegaA; bodyA.internalGetAngularVelocity( omegaA );
					btVector3 omegaB; bodyB.internalGetAngularVelocity( omegaB );
					btTransform trAPred; trAPred.setIdentity();
					btVector3 zerovec( 0, 0, 0);
					btTransformUtil::integrateTransform(
						trACur, zerovec, omegaA, timeStep, trAPred );
					btTransform trBPred; trBPred.setIdentity();
					btTransformUtil::integrateTransform(
						trBCur, zerovec, omegaB, timeStep, trBPred );

					// compute desired transforms in world
					btTransform trPose( m_qTarget );
					btTransform trABDes = m_rbBFrame * trPose * m_rbAFrame.inverse();
					btTransform trADes = trBPred * trABDes;
					btTransform trBDes = trAPred * trABDes.inverse();

					// compute desired omegas in world
					btVector3 omegaADes, omegaBDes;

					btTransformUtil::calculateVelocity( trACur, trADes, timeStep, zerovec, omegaADes );
					btTransformUtil::calculateVelocity( trBCur, trBDes, timeStep, zerovec, omegaBDes );

					// compute delta omegas
					btVector3 dOmegaA = omegaADes - omegaA;
					btVector3 dOmegaB = omegaBDes - omegaB;

					// compute weighted avg axis of dOmega (weighting based on inertias)
					btVector3 axisA, axisB;
					double kAxisAInv = 0, kAxisBInv = 0;

					if( dOmegaA.length2() > btScalar.SIMD_EPSILON )
					{
						axisA = dOmegaA.normalized();
						kAxisAInv = m_rbA.computeAngularImpulseDenominator( axisA );
					}

					if( dOmegaB.length2() > btScalar.SIMD_EPSILON )
					{
						axisB = dOmegaB.normalized();
						kAxisBInv = m_rbB.computeAngularImpulseDenominator( axisB );
					}

					btVector3 avgAxis = kAxisAInv * axisA + kAxisBInv * axisB;

					static bool bDoTorque = true;
					if( bDoTorque & avgAxis.length2() > btScalar.SIMD_EPSILON )
					{
						avgAxis.normalize();
						kAxisAInv = m_rbA.computeAngularImpulseDenominator( avgAxis );
						kAxisBInv = m_rbB.computeAngularImpulseDenominator( avgAxis );
						double kInvCombined = kAxisAInv + kAxisBInv;

						btVector3 impulse = ( kAxisAInv * dOmegaA - kAxisBInv * dOmegaB ) /
											( kInvCombined * kInvCombined );

						if( m_maxMotorImpulse >= 0 )
						{
							double fMaxImpulse = m_maxMotorImpulse;
							if( m_bNormalizedMotorStrength )
								fMaxImpulse = fMaxImpulse / kAxisAInv;

							btVector3 newUnclampedAccImpulse = m_accMotorImpulse + impulse;
							double newUnclampedMag = newUnclampedAccImpulse.length();
							if( newUnclampedMag > fMaxImpulse )
							{
								newUnclampedAccImpulse.normalize();
								newUnclampedAccImpulse *= fMaxImpulse;
								impulse = newUnclampedAccImpulse - m_accMotorImpulse;
							}
							m_accMotorImpulse += impulse;
						}

						double impulseMag = impulse.length();
						btVector3 impulseAxis = impulse / impulseMag;

						bodyA.internalApplyImpulse( btVector3( 0, 0, 0 ), m_rbA.m_invInertiaTensorWorld * impulseAxis, impulseMag );
						bodyB.internalApplyImpulse( btVector3( 0, 0, 0 ), m_rbB.m_invInertiaTensorWorld * impulseAxis, -impulseMag );

					}
				}
				else if( m_damping > btScalar.SIMD_EPSILON ) // no motor: do a little damping
				{
					btVector3 angVelA; bodyA.internalGetAngularVelocity( angVelA );
					btVector3 angVelB; bodyB.internalGetAngularVelocity( angVelB );
					btVector3 relVel = angVelB - angVelA;
					if( relVel.length2() > btScalar.SIMD_EPSILON )
					{
						btVector3 relVelAxis = relVel.normalized();
						double m_kDamping = btScalar.BT_ONE /
							( m_rbA.computeAngularImpulseDenominator( relVelAxis ) +
							 m_rbB.computeAngularImpulseDenominator( relVelAxis ) );
						btVector3 impulse = m_damping * m_kDamping * relVel;

						double impulseMag = impulse.length();
						btVector3 impulseAxis = impulse / impulseMag;
						bodyA.internalApplyImpulse( btVector3( 0, 0, 0 ), m_rbA.m_invInertiaTensorWorld * impulseAxis, impulseMag );
						bodyB.internalApplyImpulse( btVector3( 0, 0, 0 ), m_rbB.m_invInertiaTensorWorld * impulseAxis, -impulseMag );
					}
				}

				// joint limits
				{
					///solve angular part
					btVector3 angVelA;
					bodyA.internalGetAngularVelocity( angVelA );
					btVector3 angVelB;
					bodyB.internalGetAngularVelocity( angVelB );

					// solve swing limit
					if( m_solveSwingLimit )
					{
						double amplitude = m_swingLimitRatio * m_swingCorrection * m_biasFactor / timeStep;
						double relSwingVel = ( angVelB - angVelA ).dot( m_swingAxis );
						if( relSwingVel > 0 )
							amplitude += m_swingLimitRatio * relSwingVel * m_relaxationFactor;
						double impulseMag = amplitude * m_kSwing;

						// Clamp the accumulated impulse
						double temp = m_accSwingLimitImpulse;
						m_accSwingLimitImpulse = btMax( m_accSwingLimitImpulse + impulseMag, (double)( 0.0 ) );
						impulseMag = m_accSwingLimitImpulse - temp;

						btVector3 impulse = m_swingAxis * impulseMag;

						// don't let cone response affect twist
						// (this can happen since body A's twist doesn't match body B's AND we use an elliptical cone limit)
						{
							btVector3 impulseTwistCouple = impulse.dot( m_twistAxisA ) * m_twistAxisA;
							btVector3 impulseNoTwistCouple = impulse - impulseTwistCouple;
							impulse = impulseNoTwistCouple;
						}

						impulseMag = impulse.length();
						btVector3 noTwistSwingAxis = impulse / impulseMag;

						bodyA.internalApplyImpulse( btVector3( 0, 0, 0 ), m_rbA.m_invInertiaTensorWorld * noTwistSwingAxis, impulseMag );
						bodyB.internalApplyImpulse( btVector3( 0, 0, 0 ), m_rbB.m_invInertiaTensorWorld * noTwistSwingAxis, -impulseMag );
					}


					// solve twist limit
					if( m_solveTwistLimit )
					{
						double amplitude = m_twistLimitRatio * m_twistCorrection * m_biasFactor / timeStep;
						double relTwistVel = ( angVelB - angVelA ).dot( m_twistAxis );
						if( relTwistVel > 0 ) // only damp when moving towards limit (m_twistAxis flipping is important)
							amplitude += m_twistLimitRatio * relTwistVel * m_relaxationFactor;
						double impulseMag = amplitude * m_kTwist;

						// Clamp the accumulated impulse
						double temp = m_accTwistLimitImpulse;
						m_accTwistLimitImpulse = btMax( m_accTwistLimitImpulse + impulseMag, (double)( 0.0 ) );
						impulseMag = m_accTwistLimitImpulse - temp;

						//		btVector3 impulse = m_twistAxis * impulseMag;

						bodyA.internalApplyImpulse( btVector3( 0, 0, 0 ), m_rbA.m_invInertiaTensorWorld * m_twistAxis, impulseMag );
						bodyB.internalApplyImpulse( btVector3( 0, 0, 0 ), m_rbB.m_invInertiaTensorWorld * m_twistAxis, -impulseMag );
					}
				}
			}
		}
		*/



		/*
		void calcAngleInfo()
		{
			m_swingCorrection = btScalar.BT_ZERO;
			m_twistLimitSign = btScalar.BT_ZERO;
			m_solveTwistLimit = false;
			m_solveSwingLimit = false;

			btVector3 b1Axis1 = btVector3.Zero,b1Axis2 = btVector3.Zero, b1Axis3 = btVector3.Zero;
			btVector3 b2Axis1 = btVector3.Zero, b2Axis2 = btVector3.Zero;

			b1Axis1 = m_rbA.m_worldTransform.m_basis * this.m_rbAFrame.m_basis.getColumn( 0 );
			b2Axis1 = m_rbB.m_worldTransform.m_basis * this.m_rbBFrame.m_basis.getColumn( 0 );

			double swing1 = btScalar.BT_ZERO, swing2 = btScalar.BT_ZERO;

			double swx = btScalar.BT_ZERO, swy = btScalar.BT_ZERO;
			double thresh = 10;
			double fact;

			// Get Frame into world space
			if( m_swingSpan1 >= (double)( 0.05f ) )
			{
				b1Axis2 = m_rbA.m_worldTransform.m_basis * this.m_rbAFrame.m_basis.getColumn( 1 );
				swx = b2Axis1.dot( b1Axis1 );
				swy = b2Axis1.dot( b1Axis2 );
				swing1 = btScalar.btAtan2Fast( swy, swx );
				fact = ( swy * swy + swx * swx ) * thresh * thresh;
				fact = fact / ( fact + (double)( 1.0 ) );
				swing1 *= fact;
			}

			if( m_swingSpan2 >= (double)( 0.05f ) )
			{
				b1Axis3 = m_rbA.m_worldTransform.m_basis * this.m_rbAFrame.m_basis.getColumn( 2 );
				swx = b2Axis1.dot( b1Axis1 );
				swy = b2Axis1.dot( b1Axis3 );
				swing2 = btScalar.btAtan2Fast( swy, swx );
				fact = ( swy * swy + swx * swx ) * thresh * thresh;
				fact = fact / ( fact + (double)( 1.0 ) );
				swing2 *= fact;
			}

			double RMaxAngle1Sq = 1.0f / ( m_swingSpan1 * m_swingSpan1 );
			double RMaxAngle2Sq = 1.0f / ( m_swingSpan2 * m_swingSpan2 );
			double EllipseAngle = btScalar.btFabs( swing1 * swing1 ) * RMaxAngle1Sq + btScalar.btFabs( swing2 * swing2 ) * RMaxAngle2Sq;

			if( EllipseAngle > 1.0f )
			{
				m_swingCorrection = EllipseAngle - 1.0f;
				m_solveSwingLimit = true;
				// Calculate necessary axis & factors
				m_swingAxis = b2Axis1.cross( b1Axis2 * b2Axis1.dot( b1Axis2 ) + b1Axis3 * b2Axis1.dot( b1Axis3 ) );
				m_swingAxis.normalize();
				double swingAxisSign = ( b2Axis1.dot( b1Axis1 ) >= 0.0f ) ? 1.0f : -1.0f;
				m_swingAxis *= swingAxisSign;
			}

			// Twist limits
			if( m_twistSpan >= btScalar.BT_ZERO )
			{
				btVector3 b2Axis2 = m_rbB.m_worldTransform.m_basis * this.m_rbBFrame.m_basis.getColumn( 1 );
				btQuaternion rotationArc = btQuaternion.shortestArcQuat( b2Axis1, b1Axis1 );
				btVector3 TwistRef = btQuaternion.quatRotate( rotationArc, b2Axis2 );
				double twist = btScalar.btAtan2Fast( TwistRef.dot( b1Axis3 ), TwistRef.dot( b1Axis2 ) );
				m_twistAngle = twist;

				//		double lockedFreeFactor = (m_twistSpan > (double)(0.05f)) ? m_limitSoftness : btScalar.BT_ZERO;
				double lockedFreeFactor = ( m_twistSpan > (double)( 0.05f ) ) ? (double)( 1.0f ) : btScalar.BT_ZERO;
				if( twist <= -m_twistSpan * lockedFreeFactor )
				{
					m_twistCorrection = -( twist + m_twistSpan );
					m_solveTwistLimit = true;
					m_twistAxis = ( b2Axis1 + b1Axis1 ) * 0.5f;
					m_twistAxis.normalize();
					m_twistAxis *= -1.0f;
				}
				else if( twist > m_twistSpan * lockedFreeFactor )
				{
					m_twistCorrection = ( twist - m_twistSpan );
					m_solveTwistLimit = true;
					m_twistAxis = ( b2Axis1 + b1Axis1 ) * 0.5f;
					m_twistAxis.normalize();
				}
			}
		}

		*/


		void calcAngleInfo2( ref btTransform transA, ref btTransform transB, ref btMatrix3x3 invInertiaWorldA, ref btMatrix3x3 invInertiaWorldB )
		{
			m_swingCorrection = btScalar.BT_ZERO;
			m_twistLimitSign = btScalar.BT_ZERO;
			m_solveTwistLimit = false;
			m_solveSwingLimit = false;
			// compute rotation of A wrt B (in constraint space)
			if( m_bMotorEnabled && ( !m_useSolveConstraintObsolete ) )
			{   // it is assumed that setMotorTarget() was alredy called 
				// and motor target m_qTarget is within constraint limits
				// TODO : split rotation to pure swing and pure twist
				// compute desired transforms in world
				btTransform trPose = new btTransform( ref m_qTarget );
				btTransform trA; transA.Apply( ref m_rbAFrame, out trA );
				btTransform trB; transB.Apply( ref m_rbBFrame, out trB );
				btTransform tmp;
				btTransform trAInv;
				trA.inverse( out trAInv );
				trB.Apply( ref trPose, out tmp );
				btTransform trDeltaAB;// = trB * trPose * trA.inverse();
				tmp.Apply( ref trAInv, out trDeltaAB );
				btQuaternion qDeltaAB; trDeltaAB.getRotation( out qDeltaAB );
				btVector3 swingAxis = new btVector3( qDeltaAB.x, qDeltaAB.y, qDeltaAB.z );
				double swingAxisLen2 = swingAxis.length2();
				if( btScalar.btFuzzyZero( swingAxisLen2 ) )
				{
					return;
				}
				m_swingAxis = swingAxis;
				m_swingAxis.normalize();
				m_swingCorrection = qDeltaAB.getAngle();
				if( !btScalar.btFuzzyZero( m_swingCorrection ) )
				{
					m_solveSwingLimit = true;
				}
				return;
			}


			{
				// compute rotation of A wrt B (in constraint space)
				btQuaternion tmpA;
				transA.getRotation( out tmpA );
				btQuaternion tmpAFrame;
				m_rbAFrame.getRotation( out tmpAFrame );
				btQuaternion qA; tmpA.Mult( ref tmpAFrame, out qA );
				transB.getRotation( out tmpA );
				m_rbBFrame.getRotation( out tmpAFrame );
				btQuaternion qB; tmpA.Mult( ref tmpAFrame, out qB );// transB.getRotation() * m_rbBFrame.getRotation();
				btQuaternion qBInv;
				qB.inverse( out qBInv );
				btQuaternion qAB; qBInv.Mult( ref qA, out qAB );
				// split rotation into cone and twist
				// (all this is done from B's perspective. Maybe I should be averaging axes...)
				btVector3 vConeNoTwist; btQuaternion.quatRotate( ref qAB, ref vTwist, out vConeNoTwist ); vConeNoTwist.normalize();
				btQuaternion qABCone; btQuaternion.shortestArcQuat( ref vTwist, ref vConeNoTwist, out qABCone ); qABCone.normalize();
				btQuaternion qABInv;
				qABCone.inverse( out qABInv );
				btQuaternion qABTwist; qABInv.Mult( ref qAB, out qABTwist ); qABTwist.normalize();

				if( m_swingSpan1 >= m_fixThresh && m_swingSpan2 >= m_fixThresh )
				{
					double swingAngle, swingLimit = 0;
					btVector3 swingAxis;
					computeConeLimitInfo( ref qABCone, out swingAngle, out swingAxis, out swingLimit );

					if( swingAngle > swingLimit * m_limitSoftness )
					{
						m_solveSwingLimit = true;

						// compute limit ratio: 0.1, where
						// 0 == beginning of soft limit
						// 1 == hard/real limit
						m_swingLimitRatio = 1;
						if( swingAngle < swingLimit && m_limitSoftness < 1 - btScalar.SIMD_EPSILON )
						{
							m_swingLimitRatio = ( swingAngle - swingLimit * m_limitSoftness ) /
												( swingLimit - swingLimit * m_limitSoftness );
						}

						// swing correction tries to get back to soft limit
						m_swingCorrection = swingAngle - ( swingLimit * m_limitSoftness );

						// adjustment of swing axis (based on ellipse normal)
						adjustSwingAxisToUseEllipseNormal( ref swingAxis );

						// Calculate necessary axis & factors		
						btVector3 swingAxisInv;
						swingAxis.Invert( out swingAxisInv );
						btQuaternion.quatRotate( ref qB, ref swingAxisInv, out m_swingAxis );

						m_twistAxisA.setValue( 0, 0, 0 );

						m_kSwing = btScalar.BT_ONE /
							( computeAngularImpulseDenominator( ref m_swingAxis, invInertiaWorldA ) +
							 computeAngularImpulseDenominator( ref m_swingAxis, invInertiaWorldB ) );
					}
				}
				else
				{
					// you haven't set any limits;
					// or you're trying to set at least one of the swing limits too small. (if so, do you really want a conetwist constraint?)
					// anyway, we have either hinge or fixed joint
					btVector3 ivA = transA.m_basis * m_rbAFrame.m_basis.getColumn( 0 );
					btVector3 jvA = transA.m_basis * m_rbAFrame.m_basis.getColumn( 1 );
					btVector3 kvA = transA.m_basis * m_rbAFrame.m_basis.getColumn( 2 );
					btVector3 ivB = transB.m_basis * m_rbBFrame.m_basis.getColumn( 0 );
					btVector3 target;
					double x = ivB.dot( ivA );
					double y = ivB.dot( jvA );
					double z = ivB.dot( kvA );
					if( ( m_swingSpan1 < m_fixThresh ) && ( m_swingSpan2 < m_fixThresh ) )
					{ // fixed. We'll need to add one more row to constraint
						if( ( !btScalar.btFuzzyZero( y ) ) || ( !( btScalar.btFuzzyZero( z ) ) ) )
						{
							m_solveSwingLimit = true;
							m_swingAxis = -ivB.cross( ivA );
						}
					}
					else
					{
						if( m_swingSpan1 < m_fixThresh )
						{ // hinge around Y axis
						  //					if(!(btFuzzyZero(y)))
							if( ( !( btScalar.btFuzzyZero( x ) ) ) || ( !( btScalar.btFuzzyZero( z ) ) ) )
							{
								m_solveSwingLimit = true;
								if( m_swingSpan2 >= m_fixThresh )
								{
									y = (double)( 0 );
									double span2 = btScalar.btAtan2( z, x );
									if( span2 > m_swingSpan2 )
									{
										x = btScalar.btCos( m_swingSpan2 );
										z = btScalar.btSin( m_swingSpan2 );
									}
									else if( span2 < -m_swingSpan2 )
									{
										x = btScalar.btCos( m_swingSpan2 );
										z = -btScalar.btSin( m_swingSpan2 );
									}
								}
							}
						}
						else
						{ // hinge around Z axis
						  //					if(!btFuzzyZero(z))
							if( ( !( btScalar.btFuzzyZero( x ) ) ) || ( !( btScalar.btFuzzyZero( y ) ) ) )
							{
								m_solveSwingLimit = true;
								if( m_swingSpan1 >= m_fixThresh )
								{
									z = (double)( 0 );
									double span1 = btScalar.btAtan2( y, x );
									if( span1 > m_swingSpan1 )
									{
										x = btScalar.btCos( m_swingSpan1 );
										y = btScalar.btSin( m_swingSpan1 );
									}
									else if( span1 < -m_swingSpan1 )
									{
										x = btScalar.btCos( m_swingSpan1 );
										y = -btScalar.btSin( m_swingSpan1 );
									}
								}
							}
						}
						target.x = x * ivA[0] + y * jvA[0] + z * kvA[0];
						target.y = x * ivA[1] + y * jvA[1] + z * kvA[1];
						target.z = x * ivA[2] + y * jvA[2] + z * kvA[2];
						target.w = 0;
						target.normalize();
						m_swingAxis = -ivB.cross( target );
						m_swingCorrection = m_swingAxis.length();

						if( !btScalar.btFuzzyZero( m_swingCorrection ) )
							m_swingAxis.normalize();
					}
				}

				if( m_twistSpan >= (double)( 0 ) )
				{
					btVector3 twistAxis;
					computeTwistLimitInfo( ref qABTwist, out m_twistAngle, out twistAxis );
					twistAxis.Invert( out twistAxis );

					if( m_twistAngle > m_twistSpan * m_limitSoftness )
					{
						m_solveTwistLimit = true;

						m_twistLimitRatio = 1;
						if( m_twistAngle < m_twistSpan && m_limitSoftness < 1 - btScalar.SIMD_EPSILON )
						{
							m_twistLimitRatio = ( m_twistAngle - m_twistSpan * m_limitSoftness ) /
												( m_twistSpan - m_twistSpan * m_limitSoftness );
						}

						// twist correction tries to get back to soft limit
						m_twistCorrection = m_twistAngle - ( m_twistSpan * m_limitSoftness );

						btQuaternion.quatRotate( ref qB, ref twistAxis, out m_twistAxis );

						m_kTwist = btScalar.BT_ONE /
							( computeAngularImpulseDenominator( ref m_twistAxis, invInertiaWorldA ) +
							 computeAngularImpulseDenominator( ref m_twistAxis, invInertiaWorldB ) );
					}

					if( m_solveSwingLimit )
					{
						btQuaternion.quatRotate( ref qA, ref twistAxis, out m_twistAxisA );
					}
				}
				else
				{
					m_twistAngle = (double)( 0 );
				}
			}
		}



		// given a cone rotation in constraint space, (pre: twist must already be removed)
		// this method computes its corresponding swing angle and axis.
		// more interestingly, it computes the cone/swing limit (angle) for this cone "pose".
		void computeConeLimitInfo( ref btQuaternion qCone,
														 out double swingAngle, // out
														 out btVector3 vSwingAxis, // out
														 out double swingLimit ) // out
		{
			swingAngle = qCone.getAngle();
			if( swingAngle > btScalar.SIMD_EPSILON )
			{
				vSwingAxis = new btVector3( qCone.x, qCone.y, qCone.z );
				vSwingAxis.normalize();
#if false
        // non-zero twist?! this should never happen.
       Debug.Assert(Math.Abs(vSwingAxis.x) <= btScalar.SIMD_EPSILON));
#endif

				// Compute limit for given swing. tricky:
				// Given a swing axis, we're looking for the intersection with the bounding cone ellipse.
				// (Since we're dealing with angles, this ellipse is embedded on the surface of a sphere.)

				// For starters, compute the direction from center to surface of ellipse.
				// This is just the perpendicular (ie. rotate 2D vector by PI/2) of the swing axis.
				// (vSwingAxis is the cone rotation (in z,y); change vars and rotate to (x,y) coords.)
				double xEllipse = vSwingAxis.y;
				double yEllipse = -vSwingAxis.z;

				// Now, we use the slope of the vector (using x/yEllipse) and find the length
				// of the line that intersects the ellipse:
				//  x^2   y^2
				//  --- + --- = 1, where a and b are semi-major axes 2 and 1 respectively (ie. the limits)
				//  a^2   b^2
				// Do the math and it should be clear.

				swingLimit = m_swingSpan1; // if xEllipse == 0, we have a pure vSwingAxis.z rotation: just use swingspan1
				if( Math.Abs( xEllipse ) > btScalar.SIMD_EPSILON )
				{
					double surfaceSlope2 = ( yEllipse * yEllipse ) / ( xEllipse * xEllipse );
					double norm = 1 / ( m_swingSpan2 * m_swingSpan2 );
					norm += surfaceSlope2 / ( m_swingSpan1 * m_swingSpan1 );
					double swingLimit2 = ( 1 + surfaceSlope2 ) / norm;
					swingLimit = btScalar.btSqrt( swingLimit2 );
				}
				// test!
				/*swingLimit = m_swingSpan2;
				if (Math.Abs(vSwingAxis.z) > btScalar.SIMD_EPSILON)
				{
				double mag_2 = m_swingSpan1*m_swingSpan1 + m_swingSpan2*m_swingSpan2;
				double sinphi = m_swingSpan2 / sqrt(mag_2);
				double phi = asin(sinphi);
				double theta = atan2(Math.Abs(vSwingAxis.y),Math.Abs(vSwingAxis.z));
				double alpha = 3.14159f - theta - phi;
				double sinalpha = sin(alpha);
				swingLimit = m_swingSpan1 * sinphi/sinalpha;
				}*/
			}
			else //if( swingAngle < 0 )
			{
				vSwingAxis = btVector3.xAxis;
				swingLimit = 0;
				// this should never happen!
#if false
        Debug.Assert(false);
#endif
			}
		}

		internal void GetPointForAngle( double fAngleInRadians, double fLength, out btVector3 result )
		{
			// compute x/y in ellipse using cone angle (0 . 2*PI along surface of cone)
			double xEllipse = btScalar.btCos( fAngleInRadians );
			double yEllipse = btScalar.btSin( fAngleInRadians );

			// Use the slope of the vector (using x/yEllipse) and find the length
			// of the line that intersects the ellipse:
			//  x^2   y^2
			//  --- + --- = 1, where a and b are semi-major axes 2 and 1 respectively (ie. the limits)
			//  a^2   b^2
			// Do the math and it should be clear.

			double swingLimit = m_swingSpan1; // if xEllipse == 0, just use axis b (1)
			if( Math.Abs( xEllipse ) > btScalar.SIMD_EPSILON )
			{
				double surfaceSlope2 = ( yEllipse * yEllipse ) / ( xEllipse * xEllipse );
				double norm = 1 / ( m_swingSpan2 * m_swingSpan2 );
				norm += surfaceSlope2 / ( m_swingSpan1 * m_swingSpan1 );
				double swingLimit2 = ( 1 + surfaceSlope2 ) / norm;
				swingLimit = btScalar.btSqrt( swingLimit2 );
			}

			// convert into point in constraint space:
			// note: twist is x-axis, swing 1 and 2 are along the z and y axes respectively
			btVector3 vSwingAxis = new btVector3( 0, xEllipse, -yEllipse );
			btQuaternion qSwing = new btQuaternion( ref vSwingAxis, swingLimit );
			btVector3 vPointInConstraintSpace = new btVector3( fLength, 0, 0 );
			btQuaternion.quatRotate( ref qSwing, ref vPointInConstraintSpace, out result );
		}

		// given a twist rotation in constraint space, (pre: cone must already be removed)
		// this method computes its corresponding angle and axis.
		void computeTwistLimitInfo( ref btQuaternion qTwist,
														  out double twistAngle, // out
														  out btVector3 vTwistAxis ) // out
		{
			btQuaternion qMinTwist = qTwist;
			twistAngle = qTwist.getAngle();

			if( twistAngle > btScalar.SIMD_PI ) // long way around. flip quat and recalculate.
			{
				qTwist.inverse( out qMinTwist );
				twistAngle = qMinTwist.getAngle();
			}
			if( twistAngle < 0 )
			{
				// this should never happen
#if false
        Debug.Assert(false);
#endif
			}

			vTwistAxis = new btVector3( qMinTwist.x, qMinTwist.y, qMinTwist.z );
			if( twistAngle > btScalar.SIMD_EPSILON )
				vTwistAxis.normalize();
		}


		void adjustSwingAxisToUseEllipseNormal( ref btVector3 vSwingAxis )
		{
			// the swing axis is computed as the "twist-free" cone rotation,
			// but the cone limit is not circular, but elliptical (if swingspan1 != swingspan2).
			// so, if we're outside the limits, the closest way back inside the cone isn't 
			// along the vector back to the center. better (and more stable) to use the ellipse normal.

			// convert swing axis to direction from center to surface of ellipse
			// (ie. rotate 2D vector by PI/2)
			double y = -vSwingAxis.z;
			double z = vSwingAxis.y;

			// do the math...
			if( Math.Abs( z ) > btScalar.SIMD_EPSILON ) // avoid division by 0. and we don't need an update if z == 0.
			{
				// compute gradient/normal of ellipse surface at current "point"
				double grad = y / z;
				grad *= m_swingSpan2 / m_swingSpan1;

				// adjust y/z to represent normal at point (instead of vector to point)
				if( y > 0 )
					y = Math.Abs( grad * z );
				else
					y = -Math.Abs( grad * z );

				// convert ellipse direction back to swing axis
				vSwingAxis.z = ( -y );
				vSwingAxis.y = ( z );
				vSwingAxis.normalize();
			}
		}



		// setMotorTarget:
		// q: the desired rotation of bodyA wrt bodyB.
		// note: if q violates the joint limits, the internal target is clamped to avoid conflicting impulses (very bad for stability)
		// note: don't forget to enableMotor()
		public void setMotorTarget( ref btQuaternion q )
		{
			//btTransform trACur = m_rbA.m_worldTransform;
			//btTransform trBCur = m_rbB.m_worldTransform;
			//	btTransform trABCur = trBCur.inverse() * trACur;
			//	btQuaternion qABCur = trABCur.getRotation();
			//	btTransform trConstraintCur = (trBCur * m_rbBFrame).inverse() * (trACur * m_rbAFrame);
			//btQuaternion qConstraintCur = trConstraintCur.getRotation();

			btQuaternion tmp, tmp2, tmp3;

			m_rbBFrame.getRotation( out tmp );
			tmp.inverse( out tmp );
			tmp.Mult( ref q, out tmp3 );
			m_rbAFrame.getRotation( out tmp2 );
			btQuaternion qConstraint;// = m_rbBFrame.getRotation().inverse() * q * m_rbAFrame.getRotation();
			tmp3.Mult( ref tmp2, out qConstraint );
			setMotorTargetInConstraintSpace( ref qConstraint );
		}


		void setMotorTargetInConstraintSpace( ref btQuaternion q )
		{
			m_qTarget = q;

			// clamp motor target to within limits
			{
				double softness = 1;//m_limitSoftness;

				// split into twist and cone
				btVector3 vTwisted; btQuaternion.quatRotate( ref m_qTarget, ref vTwist, out vTwisted );
				btQuaternion qTargetCone; btQuaternion.shortestArcQuat( ref vTwist, ref vTwisted, out qTargetCone ); qTargetCone.normalize();
				qTargetCone.inverse( out qTargetCone );
				btQuaternion qTargetTwist; qTargetCone.Mult( ref m_qTarget, out qTargetTwist ); qTargetTwist.normalize();

				// clamp cone
				if( m_swingSpan1 >= (double)( 0.05f ) && m_swingSpan2 >= (double)( 0.05f ) )
				{
					double swingAngle, swingLimit; btVector3 swingAxis;
					computeConeLimitInfo( ref qTargetCone, out swingAngle, out swingAxis, out swingLimit );

					if( Math.Abs( swingAngle ) > btScalar.SIMD_EPSILON )
					{
						if( swingAngle > swingLimit * softness )
							swingAngle = swingLimit * softness;
						else if( swingAngle < -swingLimit * softness )
							swingAngle = -swingLimit * softness;
						qTargetCone = new btQuaternion( ref swingAxis, swingAngle );
					}
				}

				// clamp twist
				if( m_twistSpan >= (double)( 0.05f ) )
				{
					double twistAngle; btVector3 twistAxis;
					computeTwistLimitInfo( ref qTargetTwist, out twistAngle, out twistAxis );

					if( Math.Abs( twistAngle ) > btScalar.SIMD_EPSILON )
					{
						// eddy todo: limitSoftness used here???
						if( twistAngle > m_twistSpan * softness )
							twistAngle = m_twistSpan * softness;
						else if( twistAngle < -m_twistSpan * softness )
							twistAngle = -m_twistSpan * softness;
						qTargetTwist = new btQuaternion( ref twistAxis, twistAngle );
					}
				}

				qTargetCone.Mult( ref qTargetTwist, out m_qTarget );
			}
		}

		///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
		///If no axis is provided, it uses the default axis for this constraint.
		public override void setParam( btConstraintParams num, double value, int axis = -1 )
		{
			switch( num )
			{
				case btConstraintParams.BT_CONSTRAINT_ERP:
				case btConstraintParams.BT_CONSTRAINT_STOP_ERP:
					if( ( axis >= 0 ) && ( axis < 3 ) )
					{
						m_linERP = value;
						m_flags |= btConeTwistFlags.BT_CONETWIST_FLAGS_LIN_ERP;
					}
					else
					{
						m_biasFactor = value;
					}
					break;
				case btConstraintParams.BT_CONSTRAINT_CFM:
				case btConstraintParams.BT_CONSTRAINT_STOP_CFM:
					if( ( axis >= 0 ) && ( axis < 3 ) )
					{
						m_linCFM = value;
						m_flags |= btConeTwistFlags.BT_CONETWIST_FLAGS_LIN_CFM;
					}
					else
					{
						m_angCFM = value;
						m_flags |= btConeTwistFlags.BT_CONETWIST_FLAGS_ANG_CFM;
					}
					break;
				default:
					btAssertConstrParams( false );
					break;
			}
		}

		///return the local value of parameter
		public override double getParam( btConstraintParams num, int axis )
		{
			double retVal = 0;
			switch( num )
			{
				case btConstraintParams.BT_CONSTRAINT_ERP:
				case btConstraintParams.BT_CONSTRAINT_STOP_ERP:
					if( ( axis >= 0 ) && ( axis < 3 ) )
					{
						btAssertConstrParams( ( m_flags & btConeTwistFlags.BT_CONETWIST_FLAGS_LIN_ERP ) != 0 );
						retVal = m_linERP;
					}
					else if( ( axis >= 3 ) && ( axis < 6 ) )
					{
						retVal = m_biasFactor;
					}
					else
					{
						btAssertConstrParams( false );
					}
					break;
				case btConstraintParams.BT_CONSTRAINT_CFM:
				case btConstraintParams.BT_CONSTRAINT_STOP_CFM:
					if( ( axis >= 0 ) && ( axis < 3 ) )
					{
						btAssertConstrParams( ( m_flags & btConeTwistFlags.BT_CONETWIST_FLAGS_LIN_CFM ) != 0 );
						retVal = m_linCFM;
					}
					else if( ( axis >= 3 ) && ( axis < 6 ) )
					{
						btAssertConstrParams( ( m_flags & btConeTwistFlags.BT_CONETWIST_FLAGS_ANG_CFM ) != 0 );
						retVal = m_angCFM;
					}
					else
					{
						btAssertConstrParams( false );
					}
					break;
				default:
					btAssertConstrParams( false );
					break;
			}
			return retVal;
		}


		void setFrames( ref btTransform frameA, ref btTransform frameB )
		{
			m_rbAFrame = frameA;
			m_rbBFrame = frameB;
			//if( m_useSolveConstraintObsolete ) buildJacobian();
			//calculateTransforms();
		}




	};


#if SERIALIZE_DONE

	struct btConeTwistConstraintDoubleData
	{
		btTypedConstraintDoubleData m_typeConstraintData;
		btTransformDoubleData m_rbAFrame;
		btTransformDoubleData m_rbBFrame;

		//limits
		double m_swingSpan1;
		double m_swingSpan2;
		double m_twistSpan;
		double m_limitSoftness;
		double m_biasFactor;
		double m_relaxationFactor;

		double m_damping;



	};

#if BT_BACKWARDS_COMPATIBLE_SERIALIZATION
	///this structure is not used, except for loading pre-2.82 .bullet files
	struct btConeTwistConstraintData
	{
		btTypedConstraintData m_typeConstraintData;
		btTransformFloatData m_rbAFrame;
		btTransformFloatData m_rbBFrame;

		//limits
		float m_swingSpan1;
		float m_swingSpan2;
		float m_twistSpan;
		float m_limitSoftness;
		float m_biasFactor;
		float m_relaxationFactor;

		float m_damping;

		char m_pad[4];

	};
#endif //BT_BACKWARDS_COMPATIBLE_SERIALIZATION
#endif
	//

#if SERIALIZE_DONE
	public int calculateSerializeBufferSize()
	{
		return sizeof( btConeTwistConstraintData2 );

	}


	///fills the dataBuffer and returns the struct name (and 0 on failure)
	public string serialize( object dataBuffer, btSerializer* serializer )
	{
		btConeTwistConstraintData2* cone = (btConeTwistConstraintData2*)dataBuffer;
		btTypedConstraint::serialize( &cone.m_typeConstraintData, serializer );

		m_rbAFrame.serialize( cone.m_rbAFrame );
		m_rbBFrame.serialize( cone.m_rbBFrame );

		cone.m_swingSpan1 = m_swingSpan1;
		cone.m_swingSpan2 = m_swingSpan2;
		cone.m_twistSpan = m_twistSpan;
		cone.m_limitSoftness = m_limitSoftness;
		cone.m_biasFactor = m_biasFactor;
		cone.m_relaxationFactor = m_relaxationFactor;
		cone.m_damping = m_damping;

		return btConeTwistConstraintDataName;
	}

#endif
}
