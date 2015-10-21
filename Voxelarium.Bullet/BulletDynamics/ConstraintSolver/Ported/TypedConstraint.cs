#define BT_BACKWARDS_COMPATIBLE_SERIALIZATION
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

using System.Diagnostics;
using Bullet.LinearMath;
using Bullet.Types;

namespace Bullet.Dynamics.ConstraintSolver
{
	///TypedConstraint is the baseclass for Bullet constraints and vehicles
	public abstract class btTypedConstraint : btTypedObject
	{
		const float DEFAULT_DEBUGDRAW_SIZE = 0.3f;
		/*
	# ifdef BT_USE_DOUBLE_PRECISION
	#define btTypedConstraintData2		btTypedConstraintDoubleData
	#define btTypedConstraintDataName	"btTypedConstraintDoubleData"
	#else
	#define btTypedConstraintData2 		btTypedConstraintFloatData
	#define btTypedConstraintDataName  "btTypedConstraintFloatData" 
	#endif //BT_USE_DOUBLE_PRECISION
	*/


		enum btConstraintParams
		{
			BT_CONSTRAINT_ERP = 1,
			BT_CONSTRAINT_STOP_ERP,
			BT_CONSTRAINT_CFM,
			BT_CONSTRAINT_STOP_CFM
		};


		public class btJointFeedback
		{
			public btVector3 m_appliedForceBodyA;
			public btVector3 m_appliedTorqueBodyA;
			public btVector3 m_appliedForceBodyB;
			public btVector3 m_appliedTorqueBodyB;
		};


		int m_userConstraintType;

		//union
		//{
		//int m_userConstraintId;
		object m_userConstraintPtr;
		//};

		double m_breakingImpulseThreshold;
		bool m_isEnabled;
		bool m_needsFeedback;
		int m_overrideNumSolverIterations;

		/*
		btTypedConstraint&	operator=(btTypedConstraint&	other)
	{
		Debug.Assert( false );
		(void)other;
		return *this;
	}
	*/

		protected btRigidBody m_rbA;
		protected btRigidBody m_rbB;
		protected double m_appliedImpulse;
		protected double m_dbgDrawSize;
		protected btJointFeedback m_jointFeedback;


		public struct btConstraintInfo1
		{
			public int m_numConstraintRows, nub;
		};


		unsafe public struct btConstraintInfo2
		{
			// integrator parameters: frames per second (1/stepsize), default error
			// reduction parameter (0..1).
			public double fps, erp;
			// for the first and second body, pointers to two (linear and angular)
			// n*3 jacobian sub matrices, stored by rows. these matrices will have
			// been initialized to 0 on entry. if the second body is zero then the
			// J2xx pointers may be 0.
			public btIVector3 m_J1linearAxis, m_J1angularAxis, m_J2linearAxis, m_J2angularAxis;

			// elements to jump from one row to the next in J's
			public int rowskip;

			// right hand sides of the equation J*v = c + cfm * lambda. cfm is the
			// "constraint force mixing" vector. c is set to zero on entry, cfm is
			// set to a constant value (typically very small or zero) value on entry.
			public btIScalar m_constraintError, cfm;

			// lo and hi limits for variables (set to -/+ infinity on entry).
			public btIScalar m_lowerLimit, m_upperLimit;

			// findex vector for variables. see the LCP solver interface for a
			// description of what this does. this is set to -1 on entry.
			// note that the returned indexes are relative to the first index of
			// the constraint.
			public int* findex;

			// number of solver iterations
			public int m_numIterations;

			//damping of the velocity
			public double m_damping;
		};

		public int getOverrideNumSolverIterations()
		{
			return m_overrideNumSolverIterations;
		}

		///override the number of constraint solver iterations used to solve this constraint
		///-1 will use the default number of iterations, as specified in SolverInfo.m_numIterations
		void setOverrideNumSolverIterations( int overideNumIterations )
		{
			m_overrideNumSolverIterations = overideNumIterations;
		}

		///internal method used by the constraint solver, don't use them directly
		public virtual void buildJacobian() { }

		///internal method used by the constraint solver, don't use them directly
		protected virtual void setupSolverConstraint( btConstraintArray ca
			, int solverBodyA, int solverBodyB, double timeStep )
		{
		}

		///internal method used by the constraint solver, don't use them directly
		public abstract void getInfo1( ref btConstraintInfo1 info );

		///internal method used by the constraint solver, don't use them directly
		public abstract void getInfo2( ref btConstraintInfo2 info );

		///internal method used by the constraint solver, don't use them directly
		public void internalSetAppliedImpulse( double appliedImpulse )
		{
			m_appliedImpulse = appliedImpulse;
		}
		///internal method used by the constraint solver, don't use them directly
		double internalGetAppliedImpulse()
		{
			return m_appliedImpulse;
		}


		public double getBreakingImpulseThreshold()
		{
			return m_breakingImpulseThreshold;
		}

		public void setBreakingImpulseThreshold( double threshold )
		{
			m_breakingImpulseThreshold = threshold;
		}

		public bool isEnabled()
		{
			return m_isEnabled;
		}

		public void setEnabled( bool enabled )
		{
			m_isEnabled = enabled;
		}

		///internal method used by the constraint solver, don't use them directly
		public virtual void solveConstraintObsolete( btSolverBody bodyA, btSolverBody bodyB, double timeStep )
		{ }

		public btRigidBody getRigidBodyA()
		{
			return m_rbA;
		}
		public btRigidBody getRigidBodyB()
		{
			return m_rbB;
		}

		int getUserConstraintType()
		{
			return m_userConstraintType;
		}

		void setUserConstraintType( int userConstraintType )
		{
			m_userConstraintType = userConstraintType;
		}

		void setUserConstraintId( int uid )
		{
			m_userConstraintPtr = uid;
		}

		int getUserConstraintId()
		{
			return (int)m_userConstraintPtr;
		}

		void setUserConstraintPtr( object ptr )
		{
			m_userConstraintPtr = ptr;
		}

		object getUserConstraintPtr()
		{
			return m_userConstraintPtr;
		}

		void setJointFeedback( btJointFeedback jointFeedback )
		{
			m_jointFeedback = jointFeedback;
		}

		public btJointFeedback getJointFeedback()
		{
			return m_jointFeedback;
		}


		int getUid()
		{
			return (int)m_userConstraintPtr;
		}

		bool needsFeedback()
		{
			return m_needsFeedback;
		}

		///enableFeedback will allow to read the applied linear and angular impulse
		///use getAppliedImpulse, getAppliedLinearImpulse and getAppliedAngularImpulse to read feedback information
		void enableFeedback( bool needsFeedback )
		{
			m_needsFeedback = needsFeedback;
		}

		///getAppliedImpulse is an estimated total applied impulse. 
		///This feedback could be used to determine breaking constraints or playing sounds.
		double getAppliedImpulse()
		{
			Debug.Assert( m_needsFeedback );
			return m_appliedImpulse;
		}

		public btObjectTypes getConstraintType()
		{
			return m_objectType;
		}

		void setDbgDrawSize( double dbgDrawSize )
		{
			m_dbgDrawSize = dbgDrawSize;
		}
		double getDbgDrawSize()
		{
			return m_dbgDrawSize;
		}

		///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
		///If no axis is provided, it uses the default axis for this constraint.
		public abstract void setParam( int num, double value, int axis = -1 );

		///return the local value of parameter
		public abstract double getParam( int num, int axis = -1 );

#if SERIALIZE_DONE
		public int calculateSerializeBufferSize();

		///fills the dataBuffer and returns the struct name (and 0 on failure)
		virtual string serialize( object dataBuffer, btSerializer* serializer );
#endif
		static btRigidBody s_fixed = new btRigidBody( 0, null, null, ref btVector3.zAxis );

		static btRigidBody getFixedBody()
		{
			s_fixed.setMassProps( (double)( 0.0 ), ref btVector3.Zero );
			return s_fixed;
		}



		btTypedConstraint( btObjectTypes type, btRigidBody rbA ) : base( type )
		{
			m_userConstraintType = ( -1 );
			m_userConstraintPtr = ( (object)-1 );
			m_breakingImpulseThreshold = ( btScalar.SIMD_INFINITY );
			m_isEnabled = ( true );
			m_needsFeedback = ( false );
			m_overrideNumSolverIterations = ( -1 );
			m_rbA = ( rbA );
			m_rbB = ( getFixedBody() );
			m_appliedImpulse = ( (double)( 0.0 ) );
			m_dbgDrawSize = ( DEFAULT_DEBUGDRAW_SIZE );
			m_jointFeedback = ( null );
		}


		btTypedConstraint( btObjectTypes type, btRigidBody rbA, btRigidBody rbB ) : base( type )
		{
			m_userConstraintType = ( -1 );
			m_userConstraintPtr = ( (object)-1 );
			m_breakingImpulseThreshold = ( btScalar.SIMD_INFINITY );
			m_isEnabled = ( true );
			m_needsFeedback = ( false );
			m_overrideNumSolverIterations = ( -1 );
			m_rbA = ( rbA );
			m_rbB = ( rbB );
			m_appliedImpulse = ( (double)( 0.0 ) );
			m_dbgDrawSize = ( DEFAULT_DEBUGDRAW_SIZE );
			m_jointFeedback = ( null );
		}


		///internal method used by the constraint solver, don't use them directly
		double getMotorFactor( double pos, double lowLim, double uppLim, double vel, double timeFact )
		{
			if( lowLim > uppLim )
			{
				return (double)( 1.0f );
			}
			else if( lowLim == uppLim )
			{
				return (double)( 0.0f );
			}
			double lim_fact = (double)( 1.0f );
			double delta_max = vel / timeFact;
			if( delta_max < (double)( 0.0f ) )
			{
				if( ( pos >= lowLim ) && ( pos < ( lowLim - delta_max ) ) )
				{
					lim_fact = ( lowLim - pos ) / delta_max;
				}
				else if( pos < lowLim )
				{
					lim_fact = (double)( 0.0f );
				}
				else
				{
					lim_fact = (double)( 1.0f );
				}
			}
			else if( delta_max > (double)( 0.0f ) )
			{
				if( ( pos <= uppLim ) && ( pos > ( uppLim - delta_max ) ) )
				{
					lim_fact = ( uppLim - pos ) / delta_max;
				}
				else if( pos > uppLim )
				{
					lim_fact = (double)( 0.0f );
				}
				else
				{
					lim_fact = (double)( 1.0f );
				}
			}
			else
			{
				lim_fact = (double)( 0.0f );
			}
			return lim_fact;
		}

#if SERIALIZE_DONE
		///fills the dataBuffer and returns the struct name (and 0 on failure)
		string serialize( object dataBuffer, btSerializer* serializer )
		{
			btTypedConstraintData2* tcd = (btTypedConstraintData2*)dataBuffer;

			tcd.m_rbA = (btRigidBodyData*)serializer.getUniquePointer( &m_rbA );
			tcd.m_rbB = (btRigidBodyData*)serializer.getUniquePointer( &m_rbB );
			char* name = (char*)serializer.findNameForPointer( this );
			tcd.m_name = (char*)serializer.getUniquePointer( name );
			if( tcd.m_name )
			{
				serializer.serializeName( name );
			}

			tcd.m_objectType = m_objectType;
			tcd.m_needsFeedback = m_needsFeedback;
			tcd.m_overrideNumSolverIterations = m_overrideNumSolverIterations;
			tcd.m_breakingImpulseThreshold = m_breakingImpulseThreshold;
			tcd.m_isEnabled = m_isEnabled ? 1 : 0;

			tcd.m_userConstraintId = m_userConstraintId;
			tcd.m_userConstraintType = m_userConstraintType;

			tcd.m_appliedImpulse = m_appliedImpulse;
			tcd.m_dbgDrawSize = m_dbgDrawSize;

			tcd.m_disableCollisionsBetweenLinkedBodies = false;

			int i;
			for( i = 0; i < m_rbA.getNumConstraintRefs(); i++ )
				if( m_rbA.getConstraintRef( i ) == this )
					tcd.m_disableCollisionsBetweenLinkedBodies = true;
			for( i = 0; i < m_rbB.getNumConstraintRefs(); i++ )
				if( m_rbB.getConstraintRef( i ) == this )
					tcd.m_disableCollisionsBetweenLinkedBodies = true;

			return btTypedConstraintDataName;
		}
#endif

		// returns angle in range [-SIMD_2_PI, SIMD_2_PI], closest to one of the limits 
		// all arguments should be normalized angles (i.e. in range [-SIMD_PI, SIMD_PI])
		public static double btAdjustAngleToLimits( double angleInRadians, double angleLowerLimitInRadians, double angleUpperLimitInRadians )
		{
			if( angleLowerLimitInRadians >= angleUpperLimitInRadians )
			{
				return angleInRadians;
			}
			else if( angleInRadians < angleLowerLimitInRadians )
			{
				double diffLo = btScalar.btFabs( btScalar.btNormalizeAngle( angleLowerLimitInRadians - angleInRadians ) );
				double diffHi = btScalar.btFabs( btScalar.btNormalizeAngle( angleUpperLimitInRadians - angleInRadians ) );
				return ( diffLo < diffHi ) ? angleInRadians : ( angleInRadians + btScalar.SIMD_2_PI );
			}
			else if( angleInRadians > angleUpperLimitInRadians )
			{
				double diffHi = btScalar.btFabs( btScalar.btNormalizeAngle( angleInRadians - angleUpperLimitInRadians ) );
				double diffLo = btScalar.btFabs( btScalar.btNormalizeAngle( angleInRadians - angleLowerLimitInRadians ) );
				return ( diffLo < diffHi ) ? ( angleInRadians - btScalar.SIMD_2_PI ) : angleInRadians;
			}
			else
			{
				return angleInRadians;
			}
		}
	};


#if SERIALIZE_DONE
///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btTypedConstraintFloatData
{
	btRigidBodyFloatData* m_rbA;
	btRigidBodyFloatData* m_rbB;
	char* m_name;

	int m_objectType;
	int m_userConstraintType;
	int m_userConstraintId;
	int m_needsFeedback;

	float m_appliedImpulse;
	float m_dbgDrawSize;

	int m_disableCollisionsBetweenLinkedBodies;
	int m_overrideNumSolverIterations;

	float m_breakingImpulseThreshold;
	int m_isEnabled;

};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64

#if BT_BACKWARDS_COMPATIBLE_SERIALIZATION
///this structure is not used, except for loading pre-2.82 .bullet files
struct btTypedConstraintData
{
	btRigidBodyData* m_rbA;
	btRigidBodyData* m_rbB;
	char* m_name;

	int m_objectType;
	int m_userConstraintType;
	int m_userConstraintId;
	int m_needsFeedback;

	float m_appliedImpulse;
	float m_dbgDrawSize;

	int m_disableCollisionsBetweenLinkedBodies;
	int m_overrideNumSolverIterations;

	float m_breakingImpulseThreshold;
	int m_isEnabled;

};
#endif //BACKWARDS_COMPATIBLE

struct btTypedConstraintDoubleData
{
	btRigidBodyDoubleData* m_rbA;
	btRigidBodyDoubleData* m_rbB;
	char* m_name;

	int m_objectType;
	int m_userConstraintType;
	int m_userConstraintId;
	int m_needsFeedback;

	double m_appliedImpulse;
	double m_dbgDrawSize;

	int m_disableCollisionsBetweenLinkedBodies;
	int m_overrideNumSolverIterations;

	double m_breakingImpulseThreshold;
	int m_isEnabled;
	char padding[4];

};


public int calculateSerializeBufferSize()
{
	return sizeof( btTypedConstraintData2 );
}
#endif


	public class btAngularLimit
	{
		double
			m_center,
			m_halfRange,
			m_softness,
			m_biasFactor,
			m_relaxationFactor,
			m_correction,
			m_sign;

		bool
			m_solveLimit;

		public
	/// Default constructor initializes limit as inactive, allowing free constraint movement
	btAngularLimit()
		{
			m_center = ( 0.0f );
			m_halfRange = ( -1.0f );
			m_softness = ( 0.9f );
			m_biasFactor = ( 0.3f );
			m_relaxationFactor = ( 1.0f );
			m_correction = ( 0.0f );
			m_sign = ( 0.0f );
			m_solveLimit = ( false );
		}



		/// Returns limit's softness
		public double getSoftness()
		{
			return m_softness;
		}

		/// Returns limit's bias factor
		public double getBiasFactor()
		{
			return m_biasFactor;
		}

		/// Returns limit's relaxation factor
		public double getRelaxationFactor()
		{
			return m_relaxationFactor;
		}

		/// Returns correction value evaluated when test() was invoked 
		public double getCorrection()
		{
			return m_correction;
		}

		/// Returns sign value evaluated when test() was invoked 
		public double getSign()
		{
			return m_sign;
		}

		/// Gives half of the distance between min and max limit angle
		public double getHalfRange()
		{
			return m_halfRange;
		}

		/// Returns true when the last test() invocation recognized limit violation
		public bool isLimit()
		{
			return m_solveLimit;
		}

		/// Sets all limit's parameters.
		/// When low > high limit becomes inactive.
		/// When high - low > 2PI limit is ineffective too becouse no angle can exceed the limit
		//public void set( double low, double high, double _softness = 0.9f, double _biasFactor = 0.3f, double _relaxationFactor = 1.0f );
		public void set( double low, double high, double _softness = btScalar.BT_ZERO_NINE, double _biasFactor = btScalar.BT_ZERO_THREE, double _relaxationFactor = btScalar.BT_ONE )
		{
			m_halfRange = ( high - low ) / 2.0f;
			m_center = btScalar.btNormalizeAngle( low + m_halfRange );
			m_softness = _softness;
			m_biasFactor = _biasFactor;
			m_relaxationFactor = _relaxationFactor;
		}

		/// Checks conastaint angle against limit. If limit is active and the angle violates the limit
		/// correction is calculated.
		public void test( double angle )
		{
			m_correction = 0.0f;
			m_sign = 0.0f;
			m_solveLimit = false;

			if( m_halfRange >= 0.0f )
			{
				double deviation = btScalar.btNormalizeAngle( angle - m_center );
				if( deviation < -m_halfRange )
				{
					m_solveLimit = true;
					m_correction = -( deviation + m_halfRange );
					m_sign = +1.0f;
				}
				else if( deviation > m_halfRange )
				{
					m_solveLimit = true;
					m_correction = m_halfRange - deviation;
					m_sign = -1.0f;
				}
			}
		}

		/// Returns correction value multiplied by sign value
		public double getError()
		{
			return m_correction * m_sign;
		}

		/// Checks given angle against limit. If limit is active and angle doesn't fit it, the angle
		/// returned is modified so it equals to the limit closest to given angle.
		public void fit( double angle )
		{
			if( m_halfRange > 0.0f )
			{
				double relativeAngle = btScalar.btNormalizeAngle( angle - m_center );
				if( !btScalar.btEqual( relativeAngle, m_halfRange ) )
				{
					if( relativeAngle > 0.0f )
					{
						angle = getHigh();
					}
					else
					{
						angle = getLow();
					}
				}
			}
		}

		public double getLow()
		{
			return btScalar.btNormalizeAngle( m_center - m_halfRange );
		}

		public double getHigh()
		{
			return btScalar.btNormalizeAngle( m_center + m_halfRange );
		}


	}

}
