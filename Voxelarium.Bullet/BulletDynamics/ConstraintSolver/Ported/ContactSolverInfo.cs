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

using Bullet.LinearMath;
using System;

namespace Bullet.Dynamics.ConstraintSolver
{

	[Flags]
	public enum btSolverMode
	{
		SOLVER_RANDMIZE_ORDER = 1,
		SOLVER_FRICTION_SEPARATE = 2,
		SOLVER_USE_WARMSTARTING = 4,
		SOLVER_USE_2_FRICTION_DIRECTIONS = 16,
		SOLVER_ENABLE_FRICTION_DIRECTION_CACHING = 32,
		SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION = 64,
		SOLVER_CACHE_FRIENDLY = 128,
		SOLVER_SIMD = 256,
		SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS = 512,
		SOLVER_ALLOW_ZERO_LENGTH_FRICTION_DIRECTIONS = 1024
	};

	public class btContactSolverInfoData
	{
		public double m_tau;
		public double m_damping;//global non-contact constraint damping, can be locally overridden by constraints during 'getInfo2'.
		public double m_friction;
		public double m_timeStep;
		public double m_restitution;
		public int m_numIterations;
		public double m_maxErrorReduction;
		public double m_sor;
		public double m_erp;//used as Baumgarte factor
		public double m_erp2;//used in Split Impulse
		public double m_globalCfm;//constraint force mixing
		public bool m_splitImpulse;
		public double m_splitImpulsePenetrationThreshold;
		public double m_splitImpulseTurnErp;
		public double m_linearSlop;
		public double m_warmstartingFactor;

		public btSolverMode m_solverMode;
		public int m_restingContactRestitutionThreshold;
		public int m_minimumSolverBatchSize;
		public double m_maxGyroscopicForce;
		public double m_singleAxisRollingFrictionThreshold;
	};

	public class btContactSolverInfo : btContactSolverInfoData
	{

		public btContactSolverInfo()
		{
			m_tau = (double)( 0.6 );
			m_damping = (double)( 1.0 );
			m_friction = (double)( 0.3 );
			m_timeStep = (double)( btScalar.BT_ONE_OVER_SIXTY );
			m_restitution = btScalar.BT_ZERO;
			m_maxErrorReduction = (double)( 20);
			m_numIterations = 10;
			m_erp = (double)( 0.2 );
			m_erp2 = (double)( 0.8 );
			m_globalCfm = btScalar.BT_ZERO;
			m_sor = (double)( 1.0);
			m_splitImpulse = true;
			m_splitImpulsePenetrationThreshold = -.04f;
			m_splitImpulseTurnErp = 0.1f;
			m_linearSlop = btScalar.BT_ZERO;
			m_warmstartingFactor = (double)( 0.85 );
			//m_solverMode =  SOLVER_USE_WARMSTARTING |  SOLVER_SIMD | SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION|SOLVER_USE_2_FRICTION_DIRECTIONS|SOLVER_ENABLE_FRICTION_DIRECTION_CACHING;// | SOLVER_RANDMIZE_ORDER;
			m_solverMode = btSolverMode.SOLVER_USE_WARMSTARTING | btSolverMode.SOLVER_SIMD;// | SOLVER_RANDMIZE_ORDER;
			m_restingContactRestitutionThreshold = 2;//unused as of 2.81
			m_minimumSolverBatchSize = 128; //try to combine islands until the amount of constraints reaches this limit
			m_maxGyroscopicForce = 100.0f; ///it is only used for 'explicit' version of gyroscopic force
			m_singleAxisRollingFrictionThreshold = 1e30f;///if the velocity is above this threshold, it will use a single constraint row (axis), otherwise 3 rows.
		}
	};

	///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
	public struct btContactSolverInfoDoubleData
	{
		public double m_tau;
		public double m_damping;//global non-contact constraint damping, can be locally overridden by constraints during 'getInfo2'.
		public double m_friction;
		public double m_timeStep;
		public double m_restitution;
		public double m_maxErrorReduction;
		public double m_sor;
		public double m_erp;//used as Baumgarte factor
		public double m_erp2;//used in Split Impulse
		public double m_globalCfm;//constraint force mixing
		public double m_splitImpulsePenetrationThreshold;
		public double m_splitImpulseTurnErp;
		public double m_linearSlop;
		public double m_warmstartingFactor;
		public double m_maxGyroscopicForce;///it is only used for 'explicit' version of gyroscopic force
		public double m_singleAxisRollingFrictionThreshold;

		public int m_numIterations;
		public int m_solverMode;
		public int m_restingContactRestitutionThreshold;
		public int m_minimumSolverBatchSize;
		public int m_splitImpulse;
		//char m_padding[4];

	};
	///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
	public struct btContactSolverInfoFloatData
	{
		public float m_tau;
		public float m_damping;//global non-contact constraint damping, can be locally overridden by constraints during 'getInfo2'.
		public float m_friction;
		public float m_timeStep;

		public float m_restitution;
		public float m_maxErrorReduction;
		public float m_sor;
		public float m_erp;//used as Baumgarte factor

		public float m_erp2;//used in Split Impulse
		public float m_globalCfm;//constraint force mixing
		public float m_splitImpulsePenetrationThreshold;
		public float m_splitImpulseTurnErp;

		public float m_linearSlop;
		public float m_warmstartingFactor;
		public float m_maxGyroscopicForce;
		public float m_singleAxisRollingFrictionThreshold;

		public int m_numIterations;
		public int m_solverMode;
		public int m_restingContactRestitutionThreshold;
		public int m_minimumSolverBatchSize;

		public int m_splitImpulse;
		//char m_padding[4];
	};


}
