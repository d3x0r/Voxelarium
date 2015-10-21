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

	enum btConeTwistFlags
	{
		BT_CONETWIST_FLAGS_LIN_CFM = 1,
		BT_CONETWIST_FLAGS_LIN_ERP = 2,
		BT_CONETWIST_FLAGS_ANG_CFM = 4
	};

	///btConeTwistConstraint can be used to simulate ragdoll joints (upper arm, leg etc)
	internal class btConeTwistConstraint : btTypedConstraint
	{
# ifdef IN_PARALLELL_SOLVER
		public:
#endif
	btJacobianEntry m_jac[3]; //3 orthogonal linear constraints

		btTransform m_rbAFrame;
		btTransform m_rbBFrame;

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

		double m_accSwingLimitImpulse;
		double m_accTwistLimitImpulse;

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
		btVector3 m_accMotorImpulse;

		// parameters
		int m_flags;
		double m_linCFM;
		double m_linERP;
		double m_angCFM;

		protected:

	void init();

		void computeConeLimitInfo( btQuaternion& qCone, // in
			double swingAngle, ref btVector3 vSwingAxis, double swingLimit ); // all outs

		void computeTwistLimitInfo( btQuaternion& qTwist, // in
			double twistAngle, ref btVector3 vTwistAxis ); // all outs

		void adjustSwingAxisToUseEllipseNormal( ref btVector3 vSwingAxis );


		public:

	

	btConeTwistConstraint( btRigidBody rbA, btRigidBody rbB, ref btTransform rbAFrame, ref btTransform rbBFrame );

		btConeTwistConstraint( btRigidBody rbA, ref btTransform rbAFrame );

		virtual void buildJacobian();

		virtual void getInfo1( btConstraintInfo1* info );

		void getInfo1NonVirtual( btConstraintInfo1* info );

		virtual void getInfo2( btConstraintInfo2* info );

		void getInfo2NonVirtual( btConstraintInfo2* info, ref btTransform transA, ref btTransform transB, btMatrix3x3& invInertiaWorldA, btMatrix3x3& invInertiaWorldB);

		virtual void solveConstraintObsolete( btSolverBody bodyA, btSolverBody bodyB, double timeStep );


		void updateRHS( double timeStep );


		btRigidBody getRigidBodyA()
		{
			return m_rbA;
		}
		btRigidBody getRigidBodyB()
		{
			return m_rbB;
		}

		void setAngularOnly( bool angularOnly )
		{
			m_angularOnly = angularOnly;
		}

		void setLimit( int limitIndex, double limitValue )
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

	ref btTransform getAFrame() { return m_rbAFrame; };	
	ref btTransform getBFrame() { return m_rbBFrame; };

		inline int getSolveTwistLimit()
		{
			return m_solveTwistLimit;
		}

		inline int getSolveSwingLimit()
		{
			return m_solveTwistLimit;
		}

		inline double getTwistLimitSign()
		{
			return m_twistLimitSign;
		}

		void calcAngleInfo();
		void calcAngleInfo2( ref btTransform transA, ref btTransform transB, btMatrix3x3& invInertiaWorldA, btMatrix3x3& invInertiaWorldB);

		inline double getSwingSpan1()
		{
			return m_swingSpan1;
		}
		inline double getSwingSpan2()
		{
			return m_swingSpan2;
		}
		inline double getTwistSpan()
		{
			return m_twistSpan;
		}
		inline double getTwistAngle()
		{
			return m_twistAngle;
		}
		bool isPastSwingLimit() { return m_solveSwingLimit; }

		void setDamping( double damping ) { m_damping = damping; }

		void enableMotor( bool b ) { m_bMotorEnabled = b; }
		void setMaxMotorImpulse( double maxMotorImpulse ) { m_maxMotorImpulse = maxMotorImpulse; m_bNormalizedMotorStrength = false; }
		void setMaxMotorImpulseNormalized( double maxMotorImpulse ) { m_maxMotorImpulse = maxMotorImpulse; m_bNormalizedMotorStrength = true; }

		double getFixThresh() { return m_fixThresh; }
		void setFixThresh( double fixThresh ) { m_fixThresh = fixThresh; }

		// setMotorTarget:
		// q: the desired rotation of bodyA wrt bodyB.
		// note: if q violates the joint limits, the internal target is clamped to avoid conflicting impulses (very bad for stability)
		// note: don't forget to enableMotor()
		void setMotorTarget( btQuaternion &q);

		// same as above, but q is the desired rotation of frameA wrt frameB in constraint space
		void setMotorTargetInConstraintSpace( btQuaternion &q);

		btVector3 GetPointForAngle( double fAngleInRadians, double fLength );

		///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
		///If no axis is provided, it uses the default axis for this constraint.
		virtual void setParam( int num, double value, int axis = -1 );

		virtual void setFrames( ref btTransform frameA, ref btTransform frameB );

	ref btTransform getFrameOffsetA()
		{
			return m_rbAFrame;
		}

	ref btTransform getFrameOffsetB()
		{
			return m_rbBFrame;
		}


		///return the local value of parameter
		virtual double getParam( int num, int axis = -1 );

		virtual int calculateSerializeBufferSize();

		///fills the dataBuffer and returns the struct name (and 0 on failure)
		virtual string serialize( object dataBuffer, btSerializer* serializer );

	};



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

# ifdef BT_BACKWARDS_COMPATIBLE_SERIALIZATION
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
	//

	public int btConeTwistConstraint::calculateSerializeBufferSize()
	{
		return sizeof( btConeTwistConstraintData2 );

	}


	///fills the dataBuffer and returns the struct name (and 0 on failure)
	public string btConeTwistConstraint::serialize( object dataBuffer, btSerializer* serializer )
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


}
