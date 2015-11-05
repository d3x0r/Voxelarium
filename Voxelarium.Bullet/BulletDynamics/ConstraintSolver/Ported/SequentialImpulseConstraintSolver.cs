//#define COMPUTE_IMPULSE_DENOM 1
//#define BT_ADDITIONAL_DEBUG
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
using Bullet.Collision.BroadPhase;
using Bullet.Collision.Dispatch;
using Bullet.Collision.NarrowPhase;
using Bullet.LinearMath;
using Bullet.Types;

namespace Bullet.Dynamics.ConstraintSolver
{

	internal delegate double btSingleConstraintRowSolver( btSolverBody a, btSolverBody b, btSolverConstraint c );

	///The btSequentialImpulseConstraintSolver is a fast SIMD implementation of the Projected Gauss Seidel (iterative LCP) method.
	internal class btSequentialImpulseConstraintSolver : btConstraintSolver
	{
		protected btList<btSolverBody> m_tmpSolverBodyPool = new btList<btSolverBody>();
		protected btConstraintArray m_tmpSolverContactConstraintPool = new btConstraintArray();
		protected btConstraintArray m_tmpSolverNonContactConstraintPool = new btConstraintArray();
		protected btConstraintArray m_tmpSolverContactFrictionConstraintPool = new btConstraintArray();
		protected btConstraintArray m_tmpSolverContactRollingFrictionConstraintPool = new btConstraintArray();

		protected btList<int> m_orderTmpConstraintPool = new btList<int>();
		protected btList<int> m_orderNonContactConstraintPool = new btList<int>();
		protected btList<int> m_orderFrictionConstraintPool = new btList<int>();
		protected btList<btTypedConstraint.btConstraintInfo1> m_tmpConstraintSizesPool = new btList<btTypedConstraint.btConstraintInfo1>();
		protected PooledType<btTypedConstraint.btConstraintInfo2> m_tmpConstraintInfo2Pool = new PooledType<btTypedConstraint.btConstraintInfo2>();
		protected int m_maxOverrideNumSolverIterations;
		protected int m_fixedBodyId;

		protected btSingleConstraintRowSolver m_resolveSingleConstraintRowGeneric;
		protected btSingleConstraintRowSolver m_resolveSingleConstraintRowLowerLimit;

		///m_btSeed2 is used for re-arranging the constraint rows. improves convergence/quality of friction
		protected ulong m_btSeed2;

		public void setRandSeed( ulong seed )
		{
			m_btSeed2 = seed;
		}
		public ulong getRandSeed()
		{
			return m_btSeed2;
		}

		internal override void prepareSolve( int numBodies, int numManifolds ) { }
		internal override void allSolved( btContactSolverInfo info, btIDebugDraw debugDrawer ) {; }

		internal override btConstraintSolverType getSolverType()
		{
			return btConstraintSolverType.BT_SEQUENTIAL_IMPULSE_SOLVER;
		}

		public btSingleConstraintRowSolver getActiveConstraintRowSolverGeneric()
		{
			return m_resolveSingleConstraintRowGeneric;
		}
		public void setConstraintRowSolverGeneric( btSingleConstraintRowSolver rowSolver )
		{
			m_resolveSingleConstraintRowGeneric = rowSolver;
		}
		public btSingleConstraintRowSolver getActiveConstraintRowSolverLowerLimit()
		{
			return m_resolveSingleConstraintRowLowerLimit;
		}
		public void setConstraintRowSolverLowerLimit( btSingleConstraintRowSolver rowSolver )
		{
			m_resolveSingleConstraintRowLowerLimit = rowSolver;
		}

		public int gNumSplitImpulseRecoveries = 0;


		///This is the scalar reference implementation of solving a single constraint row, the innerloop of the Projected Gauss Seidel/Sequential Impulse constraint solver
		///Below are optional SSE2 and SSE4/FMA3 versions. We assume most hardware has SSE2. For SSE4/FMA3 we perform a CPU feature check.
		public static double gResolveSingleConstraintRowGeneric_scalar_reference( btSolverBody body1, btSolverBody body2, btSolverConstraint c )
		{
			double deltaImpulse = c.m_rhs - (double)( c.m_appliedImpulse ) * c.m_cfm;
			btVector3 tmplin, tmpang;
			body1.internalGetDeltaLinearVelocity( out tmplin );
			body1.internalGetDeltaAngularVelocity( out tmpang );
			double deltaVel1Dotn = c.m_contactNormal1.dot( ref tmplin ) + c.m_relpos1CrossNormal.dot( ref tmpang );
			body2.internalGetDeltaLinearVelocity( out tmplin );
			body2.internalGetDeltaAngularVelocity( out tmpang );
			double deltaVel2Dotn = c.m_contactNormal2.dot( ref tmplin ) + c.m_relpos2CrossNormal.dot( ref tmpang );

			//	double delta_rel_vel	=	deltaVel1Dotn-deltaVel2Dotn;
			deltaImpulse -= deltaVel1Dotn * c.m_jacDiagABInv;
			deltaImpulse -= deltaVel2Dotn * c.m_jacDiagABInv;

			double sum = (double)( c.m_appliedImpulse ) + deltaImpulse;
			if( sum < c.m_lowerLimit )
			{
				deltaImpulse = c.m_lowerLimit - c.m_appliedImpulse;
				c.m_appliedImpulse = c.m_lowerLimit;
			}
			else if( sum > c.m_upperLimit )
			{
				deltaImpulse = c.m_upperLimit - c.m_appliedImpulse;
				c.m_appliedImpulse = c.m_upperLimit;
			}
			else
			{
				c.m_appliedImpulse = sum;
			}
			btVector3 mass, val;
			body1.internalGetInvMass( out mass );
			mass.Mult( ref c.m_contactNormal1, out val );
			body1.applyImpulse( ref val, ref c.m_angularComponentA, deltaImpulse );
			body1.internalGetInvMass( out mass );
			mass.Mult( ref c.m_contactNormal2, out val );
			body2.applyImpulse( ref val, ref c.m_angularComponentB, deltaImpulse );

			return deltaImpulse;
		}


		public static double gResolveSingleConstraintRowLowerLimit_scalar_reference( btSolverBody body1, btSolverBody body2, btSolverConstraint c )
		{
			double deltaImpulse = c.m_rhs - (double)( c.m_appliedImpulse ) * c.m_cfm;
			btVector3 tmplin, tmpang;
			body1.internalGetDeltaLinearVelocity( out tmplin );
			body1.internalGetDeltaAngularVelocity( out tmpang );
			double deltaVel1Dotn = c.m_contactNormal1.dot( ref tmplin ) + c.m_relpos1CrossNormal.dot( ref tmpang );
			body2.internalGetDeltaLinearVelocity( out tmplin );
			body2.internalGetDeltaAngularVelocity( out tmpang );
			double deltaVel2Dotn = c.m_contactNormal2.dot( ref tmplin ) + c.m_relpos2CrossNormal.dot( ref tmpang );

			deltaImpulse -= deltaVel1Dotn * c.m_jacDiagABInv;
			deltaImpulse -= deltaVel2Dotn * c.m_jacDiagABInv;
			double sum = (double)( c.m_appliedImpulse ) + deltaImpulse;
			if( sum < c.m_lowerLimit )
			{
				deltaImpulse = c.m_lowerLimit - c.m_appliedImpulse;
				c.m_appliedImpulse = c.m_lowerLimit;
			}
			else
			{
				c.m_appliedImpulse = sum;
			}
			btVector3 mass, val;
			body1.internalGetInvMass( out mass );
			mass.Mult( ref c.m_contactNormal1, out val );
			body1.applyImpulse( ref val, ref c.m_angularComponentA, deltaImpulse );
			body1.internalGetInvMass( out mass );
			mass.Mult( ref c.m_contactNormal2, out val );
			body2.applyImpulse( ref val, ref c.m_angularComponentB, deltaImpulse );

			return deltaImpulse;
		}




		protected double resolveSingleConstraintRowGenericSIMD( btSolverBody body1, btSolverBody body2, btSolverConstraint c )
		{
#if USE_SIMD
	return m_resolveSingleConstraintRowGeneric(body1, body2, c);
#else
			return resolveSingleConstraintRowGeneric( body1, body2, c );
#endif
		}

		// Project Gauss Seidel or the equivalent Sequential Impulse
		protected double resolveSingleConstraintRowGeneric( btSolverBody body1, btSolverBody body2, btSolverConstraint c )
		{
			return gResolveSingleConstraintRowGeneric_scalar_reference( body1, body2, c );
		}

		protected double resolveSingleConstraintRowLowerLimitSIMD( btSolverBody body1, btSolverBody body2, btSolverConstraint c )
		{
#if USE_SIMD
	return m_resolveSingleConstraintRowLowerLimit(body1, body2, c);
#else
			return resolveSingleConstraintRowLowerLimit( body1, body2, c );
#endif
		}


		protected double resolveSingleConstraintRowLowerLimit( btSolverBody body1, btSolverBody body2, btSolverConstraint c )
		{
			return gResolveSingleConstraintRowLowerLimit_scalar_reference( body1, body2, c );
		}


		protected void resolveSplitPenetrationImpulseCacheFriendly(
				btSolverBody body1,
				btSolverBody body2,
				btSolverConstraint c )
		{
			if( c.m_rhsPenetration != 0 )
			{
				gNumSplitImpulseRecoveries++;
				double deltaImpulse = c.m_rhsPenetration - (double)( c.m_appliedPushImpulse ) * c.m_cfm;
				btVector3 tmplin, tmpang;
				body1.internalGetPushVelocity( out tmplin );
				body1.internalGetTurnVelocity( out tmpang );
				double deltaVel1Dotn = c.m_contactNormal1.dot( ref tmplin ) + c.m_relpos1CrossNormal.dot( ref tmpang );
				body2.internalGetPushVelocity( out tmplin );
				body2.internalGetTurnVelocity( out tmpang );
				double deltaVel2Dotn = c.m_contactNormal2.dot( ref tmplin ) + c.m_relpos2CrossNormal.dot( ref tmpang );

				deltaImpulse -= deltaVel1Dotn * c.m_jacDiagABInv;
				deltaImpulse -= deltaVel2Dotn * c.m_jacDiagABInv;
				double sum = (double)( c.m_appliedPushImpulse ) + deltaImpulse;
				if( sum < c.m_lowerLimit )
				{
					deltaImpulse = c.m_lowerLimit - c.m_appliedPushImpulse;
					c.m_appliedPushImpulse = c.m_lowerLimit;
				}
				else
				{
					c.m_appliedPushImpulse = sum;
				}
				btVector3 mass, val;
				body1.internalGetInvMass( out mass );
				mass.Mult( ref c.m_contactNormal1, out val );
				body1.applyPushImpulse( ref val, ref c.m_angularComponentA, deltaImpulse );
				body1.internalGetInvMass( out mass );
				mass.Mult( ref c.m_contactNormal2, out val );
				body2.applyPushImpulse( ref val, ref c.m_angularComponentB, deltaImpulse );
			}
		}

		protected void resolveSplitPenetrationSIMD( btSolverBody body1, btSolverBody body2, btSolverConstraint c )
		{
			resolveSplitPenetrationImpulseCacheFriendly( body1, body2, c );
		}


		public btSequentialImpulseConstraintSolver()
		{
			m_resolveSingleConstraintRowGeneric = ( gResolveSingleConstraintRowGeneric_scalar_reference );
			m_resolveSingleConstraintRowLowerLimit = ( gResolveSingleConstraintRowLowerLimit_scalar_reference );
			m_btSeed2 = ( 0 );

#if USE_SIMD
	 m_resolveSingleConstraintRowGeneric = gResolveSingleConstraintRowGeneric_sse2;
	 m_resolveSingleConstraintRowLowerLimit=gResolveSingleConstraintRowLowerLimit_sse2;
#endif //USE_SIMD

#if BT_ALLOW_SSE4
	 int cpuFeatures = btCpuFeatureUtility::getCpuFeatures();
	 if ((cpuFeatures & btCpuFeatureUtility::CPU_FEATURE_FMA3) && (cpuFeatures & btCpuFeatureUtility::CPU_FEATURE_SSE4_1))
	 {
		m_resolveSingleConstraintRowGeneric = gResolveSingleConstraintRowGeneric_sse4_1_fma3;
		m_resolveSingleConstraintRowLowerLimit = gResolveSingleConstraintRowLowerLimit_sse4_1_fma3;
	 }
#endif//BT_ALLOW_SSE4

		}


		public btSingleConstraintRowSolver getScalarConstraintRowSolverGeneric()
		{
			return gResolveSingleConstraintRowGeneric_scalar_reference;
		}

		public btSingleConstraintRowSolver getScalarConstraintRowSolverLowerLimit()
		{
			return gResolveSingleConstraintRowLowerLimit_scalar_reference;
		}


		public ulong btRand2()
		{
			m_btSeed2 = ( 1664525L * m_btSeed2 + 1013904223L ) & 0xffffffff;
			return m_btSeed2;
		}



		//See ODE: adam's all-int straightforward(?) dRandInt (0..n-1)
		public int btRandInt2( int n )
		{
			// seems good; xor-fold and modulus
			ulong un = (ulong)( n );
			ulong r = btRand2();

			// note: probably more aggressive than it needs to be -- might be
			//       able to get away without one or two of the innermost branches.
			if( un <= 0x00010000UL )
			{
				r ^= ( r >> 16 );
				if( un <= 0x00000100UL )
				{
					r ^= ( r >> 8 );
					if( un <= 0x00000010UL )
					{
						r ^= ( r >> 4 );
						if( un <= 0x00000004UL )
						{
							r ^= ( r >> 2 );
							if( un <= 0x00000002UL )
							{
								r ^= ( r >> 1 );
							}
						}
					}
				}
			}

			return (int)( r % un );
		}



		protected void initSolverBody( btSolverBody solverBody, btRigidBody rb, double timeStep )
		{
			//btRigidBody rb = collisionObject != null ? btRigidBody.upcast( collisionObject ) : null;

			solverBody.internalSetDeltaLinearVelocity( ref btVector3.Zero );
			solverBody.internalSetDeltaAngularVelocity( ref btVector3.Zero );
			solverBody.internalSetPushVelocity( ref btVector3.Zero );
			solverBody.internalSetTurnVelocity( ref btVector3.Zero );

			if( rb != null )
			{
				solverBody.m_worldTransform = rb.getWorldTransform();
				btVector3 tmp = new btVector3( rb.getInvMass() );
				btVector3 tmp2;
				btVector3 tmp3;
				rb.getLinearFactor( out tmp2 );
				tmp.Mult( ref tmp2, out tmp3 );
				solverBody.internalSetInvMass( ref tmp3 );
				solverBody.m_originalBody = rb;
				rb.getAngularFactor( out solverBody.m_angularFactor );
				rb.getLinearFactor( out solverBody.m_linearFactor );
				rb.getLinearVelocity( out solverBody.m_linearVelocity );
				rb.getAngularVelocity( out solverBody.m_angularVelocity );
				rb.m_totalForce.Mult( rb.getInvMass() * timeStep, out solverBody.m_externalForceImpulse );
				//= rb.getTotalForce() * rb.getInvMass() * timeStep;
				rb.m_invInertiaTensorWorld.Apply( ref rb.m_totalTorque, out tmp );
				tmp.Mult( timeStep, out solverBody.m_externalTorqueImpulse );
				///solverBody.m_externalTorqueImpulse = rb.getTotalTorque() * rb.getInvInertiaTensorWorld() * timeStep;

			}
			else
			{
				solverBody.m_worldTransform = null;
				solverBody.internalSetInvMass( ref btVector3.Zero );
				solverBody.m_originalBody = null;
				solverBody.m_angularFactor = btVector3.One;
				solverBody.m_linearFactor = btVector3.One;
				solverBody.m_linearVelocity = btVector3.Zero;
				solverBody.m_angularVelocity = btVector3.Zero;
				solverBody.m_externalForceImpulse = btVector3.Zero;
				solverBody.m_externalTorqueImpulse = btVector3.Zero;
			}


		}






		protected double restitutionCurve( double rel_vel, double restitution )
		{
			double rest = restitution * -rel_vel;
			return rest;
		}



		protected void applyAnisotropicFriction( btCollisionObject colObj, ref btVector3 frictionDirection
											, btCollisionObject.AnisotropicFrictionFlags frictionMode )
		{
			if( colObj != null && colObj.hasAnisotropicFriction( frictionMode ) )
			{
				// transform to local coordinates
				btVector3 loc_lateral;// = frictionDirection * colObj.getWorldTransform().getBasis();
				colObj.m_worldTransform.Basis.Apply( ref frictionDirection, out loc_lateral );
				//btVector3 friction_scaling; = colObj.getAnisotropicFriction();
				//apply anisotropic friction
				loc_lateral.Mult( ref colObj.m_anisotropicFriction, out loc_lateral );
				// ... and transform it back to global coordinates
				colObj.m_worldTransform.Basis.Apply( ref loc_lateral, out frictionDirection );
				//frictionDirection = colObj.getWorldTransform().getBasis() * loc_lateral;
			}

		}




		internal void setupFrictionConstraint( btSolverConstraint solverConstraint, ref btVector3 normalAxis, int solverBodyIdA, int solverBodyIdB, btManifoldPoint cp, ref btVector3 rel_pos1, ref btVector3 rel_pos2, btCollisionObject colObj0, btCollisionObject colObj1, double relaxation, double desiredVelocity = 0, double cfmSlip = 0.0 )
		{
			btSolverBody solverBodyA = m_tmpSolverBodyPool[solverBodyIdA];
			btSolverBody solverBodyB = m_tmpSolverBodyPool[solverBodyIdB];

			btRigidBody body0 = m_tmpSolverBodyPool[solverBodyIdA].m_originalBody;
			btRigidBody body1 = m_tmpSolverBodyPool[solverBodyIdB].m_originalBody;

			solverConstraint.m_solverBodyIdA = solverBodyIdA;
			solverConstraint.m_solverBodyIdB = solverBodyIdB;

			solverConstraint.m_friction = cp.m_combinedFriction;
			solverConstraint.m_originalContactPoint = 0;

			solverConstraint.m_appliedImpulse = 0;
			solverConstraint.m_appliedPushImpulse = 0;

			if( body0 != null )
			{
				solverConstraint.m_contactNormal1 = normalAxis;
				btVector3 ftorqueAxis1; rel_pos1.cross( ref solverConstraint.m_contactNormal1, out ftorqueAxis1 );
				solverConstraint.m_relpos1CrossNormal = ftorqueAxis1;
				solverConstraint.m_angularComponentA = body0.getInvInertiaTensorWorld() * ftorqueAxis1 * body0.getAngularFactor();
			}
			else
			{
				solverConstraint.m_contactNormal1.setZero();
				solverConstraint.m_relpos1CrossNormal.setZero();
				solverConstraint.m_angularComponentA.setZero();
			}

			if( body1 != null )
			{
				solverConstraint.m_contactNormal2 = -normalAxis;
				btVector3 ftorqueAxis1; rel_pos2.cross( ref solverConstraint.m_contactNormal2, out ftorqueAxis1 );
				solverConstraint.m_relpos2CrossNormal = ftorqueAxis1;
				solverConstraint.m_angularComponentB = body1.getInvInertiaTensorWorld() * ftorqueAxis1 * body1.getAngularFactor();
			}
			else
			{
				solverConstraint.m_contactNormal2.setZero();
				solverConstraint.m_relpos2CrossNormal.setZero();
				solverConstraint.m_angularComponentB.setZero();
			}

			{
				btVector3 vec;
				double denom0 = 0;
				double denom1 = 0;
				if( body0 != null )
				{
					vec = ( solverConstraint.m_angularComponentA ).cross( rel_pos1 );
					denom0 = body0.getInvMass() + normalAxis.dot( ref vec );
				}
				if( body1 != null )
				{
					vec = ( -solverConstraint.m_angularComponentB ).cross( rel_pos2 );
					denom1 = body1.getInvMass() + normalAxis.dot( ref vec );
				}
				double denom = relaxation / ( denom0 + denom1 );
				solverConstraint.m_jacDiagABInv = denom;
			}

			{


				double rel_vel;
				double vel1Dotn = solverConstraint.m_contactNormal1.dot( body0 != null ? solverBodyA.m_linearVelocity + solverBodyA.m_externalForceImpulse : btVector3.Zero )
					+ solverConstraint.m_relpos1CrossNormal.dot( body0 != null ? solverBodyA.m_angularVelocity : btVector3.Zero );
				double vel2Dotn = solverConstraint.m_contactNormal2.dot( body1 != null ? solverBodyB.m_linearVelocity + solverBodyB.m_externalForceImpulse : btVector3.Zero )
					+ solverConstraint.m_relpos2CrossNormal.dot( body1 != null ? solverBodyB.m_angularVelocity : btVector3.Zero );

				rel_vel = vel1Dotn + vel2Dotn;

				//		double positionalError = 0;

				double velocityError = desiredVelocity - rel_vel;
				double velocityImpulse = velocityError * solverConstraint.m_jacDiagABInv;
				solverConstraint.m_rhs = velocityImpulse;
				solverConstraint.m_rhsPenetration = 0;
				solverConstraint.m_cfm = cfmSlip;
				solverConstraint.m_lowerLimit = -solverConstraint.m_friction;
				solverConstraint.m_upperLimit = solverConstraint.m_friction;

			}
		}

		internal btSolverConstraint addFrictionConstraint( ref btVector3 normalAxis, int solverBodyIdA, int solverBodyIdB, int frictionIndex, btManifoldPoint cp, ref btVector3 rel_pos1, ref btVector3 rel_pos2, btCollisionObject colObj0, btCollisionObject colObj1, double relaxation, double desiredVelocity = 0, double cfmSlip = 0 )
		{
			btSolverConstraint solverConstraint;
			m_tmpSolverContactFrictionConstraintPool.Add( solverConstraint = new btSolverConstraint() );
			solverConstraint.m_frictionIndex = frictionIndex;
			setupFrictionConstraint( solverConstraint, ref normalAxis, solverBodyIdA, solverBodyIdB, cp, ref rel_pos1, ref rel_pos2,
									colObj0, colObj1, relaxation, desiredVelocity, cfmSlip );
			return solverConstraint;
		}


		internal void setupRollingFrictionConstraint( btSolverConstraint solverConstraint, ref btVector3 normalAxis1, int solverBodyIdA, int solverBodyIdB,
											btManifoldPoint cp, ref btVector3 rel_pos1, ref btVector3 rel_pos2,
											btCollisionObject colObj0, btCollisionObject colObj1, double relaxation,
											double desiredVelocity = 0, double cfmSlip = 0.0 )

		{
			btVector3 normalAxis = btVector3.Zero;


			solverConstraint.m_contactNormal1 = normalAxis;
			solverConstraint.m_contactNormal2 = -normalAxis;
			btSolverBody solverBodyA = m_tmpSolverBodyPool[solverBodyIdA];
			btSolverBody solverBodyB = m_tmpSolverBodyPool[solverBodyIdB];

			btRigidBody body0 = m_tmpSolverBodyPool[solverBodyIdA].m_originalBody;
			btRigidBody body1 = m_tmpSolverBodyPool[solverBodyIdB].m_originalBody;

			solverConstraint.m_solverBodyIdA = solverBodyIdA;
			solverConstraint.m_solverBodyIdB = solverBodyIdB;

			solverConstraint.m_friction = cp.m_combinedRollingFriction;
			solverConstraint.m_originalContactPoint = 0;

			solverConstraint.m_appliedImpulse = 0;
			solverConstraint.m_appliedPushImpulse = 0;

			{
				btVector3 ftorqueAxis1 = -normalAxis1;
				solverConstraint.m_relpos1CrossNormal = ftorqueAxis1;
				solverConstraint.m_angularComponentA = body0 != null ? body0.getInvInertiaTensorWorld() * ftorqueAxis1 * body0.getAngularFactor() : btVector3.Zero;
			}
			{
				btVector3 ftorqueAxis1 = normalAxis1;
				solverConstraint.m_relpos2CrossNormal = ftorqueAxis1;
				solverConstraint.m_angularComponentB = body1 != null ? body1.getInvInertiaTensorWorld() * ftorqueAxis1 * body1.getAngularFactor() : btVector3.Zero;
			}


			{
				btVector3 iMJaA = body0 != null ? body0.getInvInertiaTensorWorld() * solverConstraint.m_relpos1CrossNormal : btVector3.Zero;
				btVector3 iMJaB = body1 != null ? body1.getInvInertiaTensorWorld() * solverConstraint.m_relpos2CrossNormal : btVector3.Zero;
				double sum = 0;
				sum += iMJaA.dot( solverConstraint.m_relpos1CrossNormal );
				sum += iMJaB.dot( solverConstraint.m_relpos2CrossNormal );
				solverConstraint.m_jacDiagABInv = (double)( 1.0 ) / sum;
			}

			{


				double rel_vel;
				double vel1Dotn = solverConstraint.m_contactNormal1.dot( body0 != null ? solverBodyA.m_linearVelocity + solverBodyA.m_externalForceImpulse : btVector3.Zero )
					+ solverConstraint.m_relpos1CrossNormal.dot( body0 != null ? solverBodyA.m_angularVelocity : btVector3.Zero );
				double vel2Dotn = solverConstraint.m_contactNormal2.dot( body1 != null ? solverBodyB.m_linearVelocity + solverBodyB.m_externalForceImpulse : btVector3.Zero )
					+ solverConstraint.m_relpos2CrossNormal.dot( body1 != null ? solverBodyB.m_angularVelocity : btVector3.Zero );

				rel_vel = vel1Dotn + vel2Dotn;

				//		double positionalError = 0;

				double velocityError = desiredVelocity - rel_vel;
				double velocityImpulse = velocityError * ( solverConstraint.m_jacDiagABInv );
				solverConstraint.m_rhs = velocityImpulse;
				solverConstraint.m_cfm = cfmSlip;
				solverConstraint.m_lowerLimit = -solverConstraint.m_friction;
				solverConstraint.m_upperLimit = solverConstraint.m_friction;

			}
		}








		internal btSolverConstraint addRollingFrictionConstraint( ref btVector3 normalAxis, int solverBodyIdA, int solverBodyIdB, int frictionIndex, btManifoldPoint cp, ref btVector3 rel_pos1, ref btVector3 rel_pos2, btCollisionObject colObj0, btCollisionObject colObj1, double relaxation, double desiredVelocity = 0, double cfmSlip = 0 )
		{
			btSolverConstraint solverConstraint;
			m_tmpSolverContactRollingFrictionConstraintPool.Add( solverConstraint = new btSolverConstraint() );
			solverConstraint.m_frictionIndex = frictionIndex;
			setupRollingFrictionConstraint( solverConstraint, ref normalAxis, solverBodyIdA, solverBodyIdB, cp, ref rel_pos1, ref rel_pos2,
									colObj0, colObj1, relaxation, desiredVelocity, cfmSlip );
			return solverConstraint;
		}


		protected int getOrInitSolverBody( btCollisionObject body, double timeStep )
		{

			int solverBodyIdA = -1;

			if( body.getCompanionId() >= 0 )
			{
				//body has already been converted
				solverBodyIdA = body.getCompanionId();
				Debug.Assert( solverBodyIdA < m_tmpSolverBodyPool.Count );
			}
			else
			{
				btRigidBody rb = btRigidBody.upcast( body );
				//convert both active and kinematic objects (for their velocity)
				if( rb != null && ( rb.getInvMass() != 0 || rb.isKinematicObject() ) )
				{
					solverBodyIdA = m_tmpSolverBodyPool.Count;
					btSolverBody solverBody;
					m_tmpSolverBodyPool.Add( solverBody = new btSolverBody() );
					initSolverBody( solverBody, rb, timeStep );
					body.setCompanionId( solverBodyIdA );
				}
				else
				{

					if( m_fixedBodyId < 0 )
					{
						m_fixedBodyId = m_tmpSolverBodyPool.Count;
						btSolverBody fixedBody;
						m_tmpSolverBodyPool.Add( fixedBody = new btSolverBody() );
						initSolverBody( fixedBody, null, timeStep );
					}
					return m_fixedBodyId;
					//			return 0;//assume first one is a fixed solver body
				}
			}

			return solverBodyIdA;

		}


		internal void setupContactConstraint( btSolverConstraint solverConstraint,
											int solverBodyIdA, int solverBodyIdB,
											btManifoldPoint cp, btContactSolverInfo infoGlobal,
											out double relaxation,
											ref btVector3 rel_pos1, ref btVector3 rel_pos2 )
		{

			//	ref btVector3 pos1 = cp.getPositionWorldOnA();
			//	ref btVector3 pos2 = cp.getPositionWorldOnB();

			btSolverBody bodyA = m_tmpSolverBodyPool[solverBodyIdA];
			btSolverBody bodyB = m_tmpSolverBodyPool[solverBodyIdB];

			btRigidBody rb0 = bodyA.m_originalBody;
			btRigidBody rb1 = bodyB.m_originalBody;

			//			btVector3 rel_pos1 = pos1 - colObj0.getWorldTransform().getOrigin();
			//			btVector3 rel_pos2 = pos2 - colObj1.getWorldTransform().getOrigin();
			//rel_pos1 = pos1 - bodyA.getWorldTransform().getOrigin();
			//rel_pos2 = pos2 - bodyB.getWorldTransform().getOrigin();

			relaxation = 1;

			btVector3 torqueAxis0 = rel_pos1.cross( cp.m_normalWorldOnB );
			solverConstraint.m_angularComponentA = rb0 != null ? rb0.getInvInertiaTensorWorld() * torqueAxis0 * rb0.getAngularFactor() : btVector3.Zero;
			btVector3 torqueAxis1 = rel_pos2.cross( cp.m_normalWorldOnB );
			solverConstraint.m_angularComponentB = rb1 != null ? rb1.getInvInertiaTensorWorld() * -torqueAxis1 * rb1.getAngularFactor() : btVector3.Zero;

			{
#if COMPUTE_IMPULSE_DENOM
					double denom0 = rb0.computeImpulseDenominator(pos1,cp.m_normalWorldOnB);
					double denom1 = rb1.computeImpulseDenominator(pos2,cp.m_normalWorldOnB);
#else
				btVector3 vec;
				double denom0 = 0;
				double denom1 = 0;
				if( rb0 != null )
				{
					vec = ( solverConstraint.m_angularComponentA ).cross( rel_pos1 );
					denom0 = rb0.getInvMass() + cp.m_normalWorldOnB.dot( vec );
				}
				if( rb1 != null )
				{
					vec = ( -solverConstraint.m_angularComponentB ).cross( rel_pos2 );
					denom1 = rb1.getInvMass() + cp.m_normalWorldOnB.dot( vec );
				}
#endif //COMPUTE_IMPULSE_DENOM

				double denom = relaxation / ( denom0 + denom1 );
				solverConstraint.m_jacDiagABInv = denom;
			}

			if( rb0 != null )
			{
				solverConstraint.m_contactNormal1 = cp.m_normalWorldOnB;
				solverConstraint.m_relpos1CrossNormal = torqueAxis0;
			}
			else
			{
				solverConstraint.m_contactNormal1.setZero();
				solverConstraint.m_relpos1CrossNormal.setZero();
			}
			if( rb1 != null )
			{
				solverConstraint.m_contactNormal2 = -cp.m_normalWorldOnB;
				solverConstraint.m_relpos2CrossNormal = -torqueAxis1;
			}
			else
			{
				solverConstraint.m_contactNormal2.setZero();
				solverConstraint.m_relpos2CrossNormal.setZero();
			}

			double restitution = 0;
			double penetration = cp.getDistance() + infoGlobal.m_linearSlop;

			{
				btVector3 vel1, vel2;

				vel1 = rb0 != null ? rb0.getVelocityInLocalPoint( ref rel_pos1 ) : btVector3.Zero;
				vel2 = rb1 != null ? rb1.getVelocityInLocalPoint( ref rel_pos2 ) : btVector3.Zero;

				//			btVector3 vel2 = rb1 ? rb1.getVelocityInLocalPoint(rel_pos2) : btVector3(0,0,0);
				btVector3 vel = vel1 - vel2;
				double rel_vel = cp.m_normalWorldOnB.dot( vel );



				solverConstraint.m_friction = cp.m_combinedFriction;


				restitution = restitutionCurve( rel_vel, cp.m_combinedRestitution );
				if( restitution <= btScalar.BT_ZERO )
				{
					restitution = 0;
				};
			}


			///warm starting (or zero if disabled)
			if( ( infoGlobal.m_solverMode & btSolverMode.SOLVER_USE_WARMSTARTING ) != 0 )
			{
				solverConstraint.m_appliedImpulse = cp.m_appliedImpulse * infoGlobal.m_warmstartingFactor;
				if( rb0 != null )
				{
					btVector3 tmp = solverConstraint.m_contactNormal1 * bodyA.internalGetInvMass() * rb0.getLinearFactor();
					bodyA.applyImpulse( ref tmp, ref solverConstraint.m_angularComponentA, solverConstraint.m_appliedImpulse );
				}
				if( rb1 != null )
				{
					btVector3 tmp = -solverConstraint.m_contactNormal2 * bodyB.internalGetInvMass() * rb1.getLinearFactor();
					btVector3 tmp2 = -solverConstraint.m_angularComponentB;
					bodyB.applyImpulse( ref tmp, ref tmp2, -(double)solverConstraint.m_appliedImpulse );
				}
			}
			else
			{
				solverConstraint.m_appliedImpulse = 0;
			}

			solverConstraint.m_appliedPushImpulse = 0;

			{

				btVector3 externalForceImpulseA = bodyA.m_originalBody != null ? bodyA.m_externalForceImpulse : btVector3.Zero;
				btVector3 externalTorqueImpulseA = bodyA.m_originalBody != null ? bodyA.m_externalTorqueImpulse : btVector3.Zero;
				btVector3 externalForceImpulseB = bodyB.m_originalBody != null ? bodyB.m_externalForceImpulse : btVector3.Zero;
				btVector3 externalTorqueImpulseB = bodyB.m_originalBody != null ? bodyB.m_externalTorqueImpulse : btVector3.Zero;


				double vel1Dotn = solverConstraint.m_contactNormal1.dot( bodyA.m_linearVelocity + externalForceImpulseA )
					+ solverConstraint.m_relpos1CrossNormal.dot( bodyA.m_angularVelocity + externalTorqueImpulseA );
				double vel2Dotn = solverConstraint.m_contactNormal2.dot( bodyB.m_linearVelocity + externalForceImpulseB )
					+ solverConstraint.m_relpos2CrossNormal.dot( bodyB.m_angularVelocity + externalTorqueImpulseB );
				double rel_vel = vel1Dotn + vel2Dotn;

				double positionalError = 0;
				double velocityError = restitution - rel_vel;// * damping;


				double erp = infoGlobal.m_erp2;
				if( !infoGlobal.m_splitImpulse || ( penetration > infoGlobal.m_splitImpulsePenetrationThreshold ) )
				{
					erp = infoGlobal.m_erp;
				}

				if( penetration > 0 )
				{
					positionalError = 0;

					velocityError -= penetration / infoGlobal.m_timeStep;
				}
				else
				{
					positionalError = -penetration * erp / infoGlobal.m_timeStep;
				}

				double penetrationImpulse = positionalError * solverConstraint.m_jacDiagABInv;
				double velocityImpulse = velocityError * solverConstraint.m_jacDiagABInv;

				if( !infoGlobal.m_splitImpulse || ( penetration > infoGlobal.m_splitImpulsePenetrationThreshold ) )
				{
					//combine position and velocity into rhs
					solverConstraint.m_rhs = penetrationImpulse + velocityImpulse;//-solverConstraint.m_contactNormal1.dot(bodyA.m_externalForce*bodyA.m_invMass-bodyB.m_externalForce/bodyB.m_invMass)*solverConstraint.m_jacDiagABInv;
					solverConstraint.m_rhsPenetration = 0;

				}
				else
				{
					//split position and velocity into rhs and m_rhsPenetration
					solverConstraint.m_rhs = velocityImpulse;
					solverConstraint.m_rhsPenetration = penetrationImpulse;
				}
				solverConstraint.m_cfm = 0;
				solverConstraint.m_lowerLimit = 0;
				solverConstraint.m_upperLimit = 1e10f;
			}




		}



		internal void setFrictionConstraintImpulse( btSolverConstraint solverConstraint,
																				int solverBodyIdA, int solverBodyIdB,
																		 btManifoldPoint cp, btContactSolverInfo infoGlobal )
		{

			btSolverBody bodyA = m_tmpSolverBodyPool[solverBodyIdA];
			btSolverBody bodyB = m_tmpSolverBodyPool[solverBodyIdB];

			btRigidBody rb0 = bodyA.m_originalBody;
			btRigidBody rb1 = bodyB.m_originalBody;

			{
				btSolverConstraint frictionConstraint1 = m_tmpSolverContactFrictionConstraintPool[solverConstraint.m_frictionIndex];
				if( ( infoGlobal.m_solverMode & btSolverMode.SOLVER_USE_WARMSTARTING ) != 0 )
				{
					frictionConstraint1.m_appliedImpulse = cp.m_appliedImpulseLateral1 * infoGlobal.m_warmstartingFactor;
					if( rb0 != null )
					{
						btVector3 tmp = frictionConstraint1.m_contactNormal1 * rb0.getInvMass() * rb0.getLinearFactor();
						bodyA.applyImpulse( ref tmp, ref frictionConstraint1.m_angularComponentA, frictionConstraint1.m_appliedImpulse );
					}
					if( rb1 != null )
					{
						btVector3 tmp = -frictionConstraint1.m_contactNormal2 * rb1.getInvMass() * rb1.getLinearFactor();
						btVector3 tmp2 = -frictionConstraint1.m_angularComponentB;
						bodyB.applyImpulse( ref tmp, ref tmp2, -(double)frictionConstraint1.m_appliedImpulse );
					}
				}
				else
				{
					frictionConstraint1.m_appliedImpulse = 0;
				}
			}

			if( ( infoGlobal.m_solverMode & btSolverMode.SOLVER_USE_2_FRICTION_DIRECTIONS ) != 0 )
			{
				btSolverConstraint frictionConstraint2 = m_tmpSolverContactFrictionConstraintPool[solverConstraint.m_frictionIndex + 1];
				if( ( infoGlobal.m_solverMode & btSolverMode.SOLVER_USE_WARMSTARTING ) != 0 )
				{
					frictionConstraint2.m_appliedImpulse = cp.m_appliedImpulseLateral2 * infoGlobal.m_warmstartingFactor;
					if( rb0 != null )
					{
						btVector3 tmp = frictionConstraint2.m_contactNormal1 * rb0.getInvMass();

						bodyA.applyImpulse( ref tmp, ref frictionConstraint2.m_angularComponentA, frictionConstraint2.m_appliedImpulse );
					}
					if( rb1 != null )
					{
						btVector3 tmp = -frictionConstraint2.m_contactNormal2 * rb1.getInvMass();
						btVector3 tmp2 = -frictionConstraint2.m_angularComponentB;
						bodyB.applyImpulse( ref tmp, ref tmp2, -(double)frictionConstraint2.m_appliedImpulse );
					}
				}
				else
				{
					frictionConstraint2.m_appliedImpulse = 0;
				}
			}
		}




		public void convertContact( btPersistentManifold manifold, btContactSolverInfo infoGlobal )
		{
			btCollisionObject colObj0, colObj1;

			colObj0 = manifold.m_body0;
			colObj1 = manifold.m_body1;

			int solverBodyIdA = getOrInitSolverBody( colObj0, infoGlobal.m_timeStep );
			int solverBodyIdB = getOrInitSolverBody( colObj1, infoGlobal.m_timeStep );

			//	btRigidBody bodyA = btRigidBody::upcast(colObj0);
			//	btRigidBody bodyB = btRigidBody::upcast(colObj1);

			btSolverBody solverBodyA = m_tmpSolverBodyPool[solverBodyIdA];
			btSolverBody solverBodyB = m_tmpSolverBodyPool[solverBodyIdB];



			///avoid collision response between two static objects
			if( solverBodyA == null || ( solverBodyA.m_invMass.fuzzyZero() && ( solverBodyB == null || solverBodyB.m_invMass.fuzzyZero() ) ) )
				return;

			int rollingFriction = 1;
			for( int j = 0; j < manifold.m_cachedPoints; j++ )
			{

				btManifoldPoint cp = manifold.getContactPoint( j );

				if( cp.m_distance1 <= manifold.m_contactProcessingThreshold )
				{
					btVector3 rel_pos1;
					btVector3 rel_pos2;
					double relaxation;


					int frictionIndex = m_tmpSolverContactConstraintPool.Count;
					btSolverConstraint solverConstraint;
					m_tmpSolverContactConstraintPool.Add( solverConstraint = new btSolverConstraint() );
					btRigidBody rb0 = btRigidBody.upcast( colObj0 );
					btRigidBody rb1 = btRigidBody.upcast( colObj1 );
					solverConstraint.m_solverBodyIdA = solverBodyIdA;
					solverConstraint.m_solverBodyIdB = solverBodyIdB;

					solverConstraint.m_originalContactPoint = cp;

					btVector3 pos1 = cp.m_positionWorldOnA;
					btVector3 pos2 = cp.m_positionWorldOnB;

					rel_pos1 = pos1 - colObj0.getWorldTransform().getOrigin();
					rel_pos2 = pos2 - colObj1.getWorldTransform().getOrigin();

					btVector3 vel1;// = rb0 ? rb0.getVelocityInLocalPoint(rel_pos1) : btVector3(0,0,0);
					btVector3 vel2;// = rb1 ? rb1.getVelocityInLocalPoint(rel_pos2) : btVector3(0,0,0);

					solverBodyA.getVelocityInLocalPointNoDelta( ref rel_pos1, out vel1 );
					solverBodyB.getVelocityInLocalPointNoDelta( ref rel_pos2, out vel2 );

					btVector3 vel = vel1 - vel2;
					double rel_vel = cp.m_normalWorldOnB.dot( vel );

					setupContactConstraint( solverConstraint, solverBodyIdA, solverBodyIdB, cp, infoGlobal, out relaxation, ref rel_pos1, ref rel_pos2 );


					//			ref btVector3 pos1 = cp.getPositionWorldOnA();
					//			ref btVector3 pos2 = cp.getPositionWorldOnB();

					/////setup the friction constraints

					solverConstraint.m_frictionIndex = m_tmpSolverContactFrictionConstraintPool.Count;

					btVector3 relAngVel;
					if( rb0 != null && rb1 != null )
						rb1.m_angularVelocity.Sub( ref rb0.m_angularVelocity, out relAngVel );
					else if( rb0 != null )
						rb0.m_angularVelocity.Invert( out relAngVel );
					else if( rb1 != null )
						relAngVel = rb1.m_angularVelocity;
					else
						relAngVel = btVector3.Zero;

					if( ( cp.m_combinedRollingFriction > 0 ) && ( rollingFriction > 0 ) )
					{
						//only a single rollingFriction per manifold
						rollingFriction--;
						if( relAngVel.length() > infoGlobal.m_singleAxisRollingFrictionThreshold )
						{
							relAngVel.normalize();
							applyAnisotropicFriction( colObj0, ref relAngVel, btCollisionObject.AnisotropicFrictionFlags.CF_ANISOTROPIC_ROLLING_FRICTION );
							applyAnisotropicFriction( colObj1, ref relAngVel, btCollisionObject.AnisotropicFrictionFlags.CF_ANISOTROPIC_ROLLING_FRICTION );
							if( relAngVel.length() > 0.001 )
								addRollingFrictionConstraint( ref relAngVel, solverBodyIdA, solverBodyIdB, frictionIndex, cp, ref rel_pos1, ref rel_pos2, colObj0, colObj1, relaxation );

						}
						else
						{
							addRollingFrictionConstraint( ref cp.m_normalWorldOnB, solverBodyIdA, solverBodyIdB, frictionIndex, cp, ref rel_pos1, ref rel_pos2, colObj0, colObj1, relaxation );
							btVector3 axis0, axis1;
							btVector3.btPlaneSpace1( ref cp.m_normalWorldOnB, out axis0, out axis1 );
							applyAnisotropicFriction( colObj0, ref axis0, btCollisionObject.AnisotropicFrictionFlags.CF_ANISOTROPIC_ROLLING_FRICTION );
							applyAnisotropicFriction( colObj1, ref axis0, btCollisionObject.AnisotropicFrictionFlags.CF_ANISOTROPIC_ROLLING_FRICTION );
							applyAnisotropicFriction( colObj0, ref axis1, btCollisionObject.AnisotropicFrictionFlags.CF_ANISOTROPIC_ROLLING_FRICTION );
							applyAnisotropicFriction( colObj1, ref axis1, btCollisionObject.AnisotropicFrictionFlags.CF_ANISOTROPIC_ROLLING_FRICTION );
							if( axis0.length() > 0.001 )
								addRollingFrictionConstraint( ref axis0, solverBodyIdA, solverBodyIdB, frictionIndex, cp, ref rel_pos1, ref rel_pos2, colObj0, colObj1, relaxation );
							if( axis1.length() > 0.001 )
								addRollingFrictionConstraint( ref axis1, solverBodyIdA, solverBodyIdB, frictionIndex, cp, ref rel_pos1, ref rel_pos2, colObj0, colObj1, relaxation );

						}
					}

					///Bullet has several options to set the friction directions
					///By default, each contact has only a single friction direction that is recomputed automatically very frame
					///based on the relative linear velocity.
					///If the relative velocity it zero, it will automatically compute a friction direction.

					///You can also enable two friction directions, using the SOLVER_USE_2_FRICTION_DIRECTIONS.
					///In that case, the second friction direction will be orthogonal to both contact normal and first friction direction.
					///
					///If you choose SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION, then the friction will be independent from the relative projected velocity.
					///
					///The user can manually override the friction directions for certain contacts using a contact callback,
					///and set the cp.m_lateralFrictionInitialized to true
					///In that case, you can set the target relative motion in each friction direction (cp.m_contactMotion1 and cp.m_contactMotion2)
					///this will give a conveyor belt effect
					///
					if( ( ( infoGlobal.m_solverMode & btSolverMode.SOLVER_ENABLE_FRICTION_DIRECTION_CACHING ) == 0 )
								|| !cp.m_lateralFrictionInitialized )
					{
						cp.m_lateralFrictionDir1 = vel - cp.m_normalWorldOnB * rel_vel;
						double lat_rel_vel = cp.m_lateralFrictionDir1.length2();
						if( ( ( infoGlobal.m_solverMode & btSolverMode.SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION ) == 0 ) && lat_rel_vel > btScalar.SIMD_EPSILON )
						{
							cp.m_lateralFrictionDir1 *= 1 / btScalar.btSqrt( lat_rel_vel );
							applyAnisotropicFriction( colObj0, ref cp.m_lateralFrictionDir1, btCollisionObject.AnisotropicFrictionFlags.CF_ANISOTROPIC_FRICTION );
							applyAnisotropicFriction( colObj1, ref cp.m_lateralFrictionDir1, btCollisionObject.AnisotropicFrictionFlags.CF_ANISOTROPIC_FRICTION );
							addFrictionConstraint( ref cp.m_lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex, cp, ref rel_pos1, ref rel_pos2, colObj0, colObj1, relaxation );

							if( ( infoGlobal.m_solverMode & btSolverMode.SOLVER_USE_2_FRICTION_DIRECTIONS ) != 0 )
							{
								cp.m_lateralFrictionDir2 = cp.m_lateralFrictionDir1.cross( cp.m_normalWorldOnB );
								cp.m_lateralFrictionDir2.normalize();//??
								applyAnisotropicFriction( colObj0, ref cp.m_lateralFrictionDir2, btCollisionObject.AnisotropicFrictionFlags.CF_ANISOTROPIC_FRICTION );
								applyAnisotropicFriction( colObj1, ref cp.m_lateralFrictionDir2, btCollisionObject.AnisotropicFrictionFlags.CF_ANISOTROPIC_FRICTION );
								addFrictionConstraint( ref cp.m_lateralFrictionDir2, solverBodyIdA, solverBodyIdB, frictionIndex, cp, ref rel_pos1, ref rel_pos2, colObj0, colObj1, relaxation );
							}

						}
						else
						{
							btVector3.btPlaneSpace1( ref cp.m_normalWorldOnB, out cp.m_lateralFrictionDir1, out cp.m_lateralFrictionDir2 );

							applyAnisotropicFriction( colObj0, ref cp.m_lateralFrictionDir1, btCollisionObject.AnisotropicFrictionFlags.CF_ANISOTROPIC_FRICTION );
							applyAnisotropicFriction( colObj1, ref cp.m_lateralFrictionDir1, btCollisionObject.AnisotropicFrictionFlags.CF_ANISOTROPIC_FRICTION );
							addFrictionConstraint( ref cp.m_lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex, cp, ref rel_pos1, ref rel_pos2, colObj0, colObj1, relaxation );

							if( ( infoGlobal.m_solverMode & btSolverMode.SOLVER_USE_2_FRICTION_DIRECTIONS ) != 0 )
							{
								applyAnisotropicFriction( colObj0, ref cp.m_lateralFrictionDir2, btCollisionObject.AnisotropicFrictionFlags.CF_ANISOTROPIC_FRICTION );
								applyAnisotropicFriction( colObj1, ref cp.m_lateralFrictionDir2, btCollisionObject.AnisotropicFrictionFlags.CF_ANISOTROPIC_FRICTION );
								addFrictionConstraint( ref cp.m_lateralFrictionDir2, solverBodyIdA, solverBodyIdB, frictionIndex, cp, ref rel_pos1, ref rel_pos2, colObj0, colObj1, relaxation );
							}


							if( ( ( infoGlobal.m_solverMode & btSolverMode.SOLVER_USE_2_FRICTION_DIRECTIONS ) != 0 )
								&& ( ( infoGlobal.m_solverMode & btSolverMode.SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION ) != 0 ) )
							{
								cp.m_lateralFrictionInitialized = true;
							}
						}

					}
					else
					{
						addFrictionConstraint( ref cp.m_lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex, cp, ref rel_pos1, ref rel_pos2, colObj0, colObj1, relaxation, cp.m_contactMotion1, cp.m_contactCFM1 );

						if( ( infoGlobal.m_solverMode & btSolverMode.SOLVER_USE_2_FRICTION_DIRECTIONS ) != 0 )
							addFrictionConstraint( ref cp.m_lateralFrictionDir2, solverBodyIdA, solverBodyIdB, frictionIndex, cp, ref rel_pos1, ref rel_pos2, colObj0, colObj1, relaxation, cp.m_contactMotion2, cp.m_contactCFM2 );

					}
					setFrictionConstraintImpulse( solverConstraint, solverBodyIdA, solverBodyIdB, cp, infoGlobal );




				}
			}
		}

		protected void convertContacts( btPersistentManifold[] manifoldPtr, int start_manifold, int numManifolds, btContactSolverInfo infoGlobal )
		{
			int i;
			btPersistentManifold manifold;
			//			btCollisionObject colObj0=0,*colObj1=0;


			for( i = 0; i < numManifolds; i++ )
			{
				manifold = manifoldPtr[i + start_manifold];
				convertContact( manifold, infoGlobal );
			}
		}

		protected virtual double solveGroupCacheFriendlySetup( btCollisionObject[] bodies, int numBodies
			, btPersistentManifold[] manifoldPtr, int start_manifold, int numManifolds
			, btTypedConstraint[] constraints, int startConstraint, int numConstraints, btContactSolverInfo infoGlobal, btIDebugDraw debugDrawer )
		{
			m_fixedBodyId = -1;
			CProfileSample sample = new CProfileSample( "solveGroupCacheFriendlySetup" );
			//(void)debugDrawer;

			m_maxOverrideNumSolverIterations = 0;

#if BT_ADDITIONAL_DEBUG
	 //make sure that dynamic bodies exist for all (enabled)raints
	for (int i=0;i<numConstraints;i++)
	{
		btTypedConstraint* constraint = constraints[i];
		if (constraint.isEnabled())
		{
			if (!constraint.getRigidBodyA().isStaticOrKinematicObject())
			{
				bool found=false;
				for (int b=0;b<numBodies;b++)
				{

					if (&constraint.getRigidBodyA()==bodies[b])
					{
						found = true;
						break;
					}
				}
				Debug.Assert(found);
			}
			if (!constraint.getRigidBodyB().isStaticOrKinematicObject())
			{
				bool found=false;
				for (int b=0;b<numBodies;b++)
				{
					if (&constraint.getRigidBodyB()==bodies[b])
					{
						found = true;
						break;
					}
				}
				Debug.Assert(found);
			}
		}
	}
    //make sure that dynamic bodies exist for all contact manifolds
    for (int i=0;i<numManifolds;i++)
    {
        if (!manifoldPtr[i].getBody0().isStaticOrKinematicObject())
        {
            bool found=false;
            for (int b=0;b<numBodies;b++)
            {

                if (manifoldPtr[i].getBody0()==bodies[b])
                {
                    found = true;
                    break;
                }
            }
            Debug.Assert(found);
        }
        if (!manifoldPtr[i].getBody1().isStaticOrKinematicObject())
        {
            bool found=false;
            for (int b=0;b<numBodies;b++)
            {
                if (manifoldPtr[i].getBody1()==bodies[b])
                {
                    found = true;
                    break;
                }
            }
            Debug.Assert(found);
        }
    }
#endif //BT_ADDITIONAL_DEBUG


			for( int i = 0; i < numBodies; i++ )
			{
				bodies[i].setCompanionId( -1 );
			}


			m_tmpSolverBodyPool.Capacity = ( numBodies + 1 );
			m_tmpSolverBodyPool.Count = ( 0 );

			//btSolverBody fixedBody = m_tmpSolverBodyPool.expand();
			//initSolverBody(&fixedBody,0);

			//convert all bodies


			for( int i = 0; i < numBodies; i++ )
			{
				int bodyId = getOrInitSolverBody( bodies[i], infoGlobal.m_timeStep );

				btRigidBody body = btRigidBody.upcast( bodies[i] );
				if( body != null && body.getInvMass() != 0 )
				{
					btSolverBody solverBody = m_tmpSolverBodyPool[bodyId];
					btVector3 gyroForce = btVector3.Zero;
					if( ( body.getFlags() & btRigidBodyFlags.BT_ENABLE_GYROSCOPIC_FORCE_EXPLICIT ) != 0 )
					{
						body.computeGyroscopicForceExplicit( infoGlobal.m_maxGyroscopicForce, out gyroForce );
						solverBody.m_externalTorqueImpulse -= gyroForce * body.getInvInertiaTensorWorld() * infoGlobal.m_timeStep;
					}
					if( ( body.getFlags() & btRigidBodyFlags.BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD ) != 0 )
					{
						body.computeGyroscopicImpulseImplicit_World( infoGlobal.m_timeStep, out gyroForce );
						solverBody.m_externalTorqueImpulse.Add( ref gyroForce, out solverBody.m_externalTorqueImpulse );
					}
					if( ( body.getFlags() & btRigidBodyFlags.BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY ) != 0 )
					{
						body.computeGyroscopicImpulseImplicit_Body( infoGlobal.m_timeStep, out gyroForce );
						solverBody.m_externalTorqueImpulse.Add( ref gyroForce, out solverBody.m_externalTorqueImpulse );
					}
				}
			}

			if( true )
			{
				int j;
				for( j = 0; j < numConstraints; j++ )
				{
					btTypedConstraint constraint = constraints[j + startConstraint];
					constraint.buildJacobian();
					constraint.internalSetAppliedImpulse( 0.0f );
				}
			}

			//btRigidBody rb0=0,*rb1=0;

			//if (1)
			{
				{

					int totalNumRows = 0;
					int i;

					m_tmpConstraintSizesPool.Capacity = ( numConstraints );
					//calculate the total number of contraint rows
					for( i = 0; i < numConstraints; i++ )
					{
						btTypedConstraint.btConstraintInfo1 info1 = m_tmpConstraintSizesPool[i];
						btTypedConstraint.btJointFeedback fb = constraints[i + startConstraint].getJointFeedback();
						if( fb != null )
						{
							fb.m_appliedForceBodyA.setZero();
							fb.m_appliedTorqueBodyA.setZero();
							fb.m_appliedForceBodyB.setZero();
							fb.m_appliedTorqueBodyB.setZero();
						}

						if( constraints[i + startConstraint].isEnabled() )
						{
						}
						if( constraints[i + startConstraint].isEnabled() )
						{
							constraints[i + startConstraint].getInfo1( ref info1 );
						}
						else
						{
							info1.m_numConstraintRows = 0;
							info1.nub = 0;
						}
						totalNumRows += info1.m_numConstraintRows;
					}
					m_tmpSolverNonContactConstraintPool.Capacity = ( totalNumRows );


					///setup the btSolverConstraints
					int currentRow = 0;

					for( i = 0; i < numConstraints; i++ )
					{
						btTypedConstraint.btConstraintInfo1 info1 = m_tmpConstraintSizesPool[i];

						if( info1.m_numConstraintRows != 0 )
						{
							Debug.Assert( currentRow < totalNumRows );

							btSolverConstraint currentConstraintRow = m_tmpSolverNonContactConstraintPool[currentRow];
							btTypedConstraint constraint = constraints[i + startConstraint];
							btRigidBody rbA = constraint.getRigidBodyA();
							btRigidBody rbB = constraint.getRigidBodyB();

							int solverBodyIdA = getOrInitSolverBody( rbA, infoGlobal.m_timeStep );
							int solverBodyIdB = getOrInitSolverBody( rbB, infoGlobal.m_timeStep );

							btSolverBody bodyAPtr = m_tmpSolverBodyPool[solverBodyIdA];
							btSolverBody bodyBPtr = m_tmpSolverBodyPool[solverBodyIdB];

							int overrideNumSolverIterations = constraint.getOverrideNumSolverIterations() > 0 ? constraint.getOverrideNumSolverIterations() : infoGlobal.m_numIterations;
							if( overrideNumSolverIterations > m_maxOverrideNumSolverIterations )
								m_maxOverrideNumSolverIterations = overrideNumSolverIterations;

							int j;
							for( j = 0; j < info1.m_numConstraintRows; j++ )
							{
								btSolverConstraint current = m_tmpSolverNonContactConstraintPool[currentRow + j];
								current.Clear();
								//memset( &currentConstraintRow[j], 0, sizeof( btSolverConstraint ) );
								current.m_lowerLimit = btScalar.SIMD_NEG_INFINITY;
								current.m_upperLimit = btScalar.SIMD_INFINITY;
								current.m_appliedImpulse = 0;
								current.m_appliedPushImpulse = 0;
								current.m_solverBodyIdA = solverBodyIdA;
								current.m_solverBodyIdB = solverBodyIdB;
								current.m_overrideNumSolverIterations = overrideNumSolverIterations;
							}

							bodyAPtr.Clear();


							btTypedConstraint.btConstraintInfo2 info2 = m_tmpConstraintInfo2Pool.Get();// new btTypedConstraint.btConstraintInfo2();
							info2.m_numRows = info1.m_numConstraintRows;

							for( j = 0; j < info1.m_numConstraintRows; ++j )
							{
								info2.m_solverConstraints[j] = m_tmpSolverNonContactConstraintPool[currentRow + j];
							}

							info2.fps = 1 / infoGlobal.m_timeStep;
							info2.erp = infoGlobal.m_erp;
#if OLD_CONSTRAINT_INFO_INIT
							info2.m_J1linearAxis = currentConstraintRow.m_contactNormal1;
							info2.m_J1angularAxis = currentConstraintRow.m_relpos1CrossNormal;
							info2.m_J2linearAxis = currentConstraintRow.m_contactNormal2;
							info2.m_J2angularAxis = currentConstraintRow.m_relpos2CrossNormal;
							info2.rowskip = 0;// sizeof( btSolverConstraint ) / sizeof( double );//check this
																							///the size of btSolverConstraint needs be a multiple of double
							//Debug.Assert( info2.rowskip * sizeof( double ) == sizeof( btSolverConstraint ) );
							info2.m_constraintError = currentConstraintRow.m_rhs;
							info2.cfm = currentConstraintRow.m_cfm;
							info2.m_lowerLimit = currentConstraintRow.m_lowerLimit;
							info2.m_upperLimit = currentConstraintRow.m_upperLimit;
#endif

							currentConstraintRow.m_cfm = infoGlobal.m_globalCfm;
							info2.m_damping = infoGlobal.m_damping;
							info2.m_numIterations = infoGlobal.m_numIterations;
							constraint.getInfo2( info2 );

							///finalize the constraint setup
							for( j = 0; j < info1.m_numConstraintRows; j++ )
							{
								btSolverConstraint solverConstraint = m_tmpSolverNonContactConstraintPool[currentRow + j];

								if( solverConstraint.m_upperLimit >= constraint.getBreakingImpulseThreshold() )
								{
									solverConstraint.m_upperLimit = constraint.getBreakingImpulseThreshold();
								}

								if( solverConstraint.m_lowerLimit <= -constraint.getBreakingImpulseThreshold() )
								{
									solverConstraint.m_lowerLimit = -constraint.getBreakingImpulseThreshold();
								}

								solverConstraint.m_originalContactPoint = constraint;

								{
									//btVector3 ftorqueAxis1 = solverConstraint.m_relpos1CrossNormal;
									solverConstraint.m_angularComponentA = constraint.getRigidBodyA().getInvInertiaTensorWorld()
										* solverConstraint.m_relpos1CrossNormal * constraint.getRigidBodyA().getAngularFactor();
								}
								{
									//btVector3 ftorqueAxis2 = solverConstraint.m_relpos2CrossNormal;
									solverConstraint.m_angularComponentB = constraint.getRigidBodyB().getInvInertiaTensorWorld()
										* solverConstraint.m_relpos2CrossNormal * constraint.getRigidBodyB().getAngularFactor();
								}

								{
									btVector3 iMJlA = solverConstraint.m_contactNormal1 * rbA.getInvMass();
									btVector3 iMJaA = rbA.getInvInertiaTensorWorld() * solverConstraint.m_relpos1CrossNormal;
									btVector3 iMJlB = solverConstraint.m_contactNormal2 * rbB.getInvMass();//sign of normal?
									btVector3 iMJaB = rbB.getInvInertiaTensorWorld() * solverConstraint.m_relpos2CrossNormal;

									double sum = iMJlA.dot( solverConstraint.m_contactNormal1 );
									sum += iMJaA.dot( solverConstraint.m_relpos1CrossNormal );
									sum += iMJlB.dot( solverConstraint.m_contactNormal2 );
									sum += iMJaB.dot( solverConstraint.m_relpos2CrossNormal );
									double fsum = btScalar.btFabs( sum );
									Debug.Assert( fsum > btScalar.SIMD_EPSILON );
									solverConstraint.m_jacDiagABInv = fsum > btScalar.SIMD_EPSILON ? btScalar.BT_ONE / sum : 0;
								}



								{
									double rel_vel;
									btVector3 externalForceImpulseA = bodyAPtr.m_originalBody != null ? bodyAPtr.m_externalForceImpulse : btVector3.Zero;
									btVector3 externalTorqueImpulseA = bodyAPtr.m_originalBody != null ? bodyAPtr.m_externalTorqueImpulse : btVector3.Zero;

									btVector3 externalForceImpulseB = bodyBPtr.m_originalBody != null ? bodyBPtr.m_externalForceImpulse : btVector3.Zero;
									btVector3 externalTorqueImpulseB = bodyBPtr.m_originalBody != null ? bodyBPtr.m_externalTorqueImpulse : btVector3.Zero;

									double vel1Dotn = solverConstraint.m_contactNormal1.dot( rbA.getLinearVelocity() + externalForceImpulseA )
														+ solverConstraint.m_relpos1CrossNormal.dot( rbA.getAngularVelocity() + externalTorqueImpulseA );

									double vel2Dotn = solverConstraint.m_contactNormal2.dot( rbB.getLinearVelocity() + externalForceImpulseB )
																		+ solverConstraint.m_relpos2CrossNormal.dot( rbB.getAngularVelocity() + externalTorqueImpulseB );

									rel_vel = vel1Dotn + vel2Dotn;
									double restitution = 0;
									double positionalError = solverConstraint.m_rhs;//already filled in by getConstraintInfo2
									double velocityError = restitution - rel_vel * info2.m_damping;
									double penetrationImpulse = positionalError * solverConstraint.m_jacDiagABInv;
									double velocityImpulse = velocityError * solverConstraint.m_jacDiagABInv;
									solverConstraint.m_rhs = penetrationImpulse + velocityImpulse;
									solverConstraint.m_appliedImpulse = 0;
								}
							}
						}
						currentRow += m_tmpConstraintSizesPool[i].m_numConstraintRows;
					}
				}

				convertContacts( manifoldPtr, start_manifold, numManifolds, infoGlobal );

			}

			//	btContactSolverInfo info = infoGlobal;


			int numNonContactPool = m_tmpSolverNonContactConstraintPool.Count;
			int numConstraintPool = m_tmpSolverContactConstraintPool.Count;
			int numFrictionPool = m_tmpSolverContactFrictionConstraintPool.Count;

			///@todo: use stack allocator for such temporarily memory, same for solver bodies/constraints
			m_orderNonContactConstraintPool.Capacity = ( numNonContactPool );
			if( ( infoGlobal.m_solverMode & btSolverMode.SOLVER_USE_2_FRICTION_DIRECTIONS ) != 0 )
				m_orderTmpConstraintPool.Count = m_orderTmpConstraintPool.Capacity = ( numConstraintPool * 2 );
			else
				m_orderTmpConstraintPool.Count = m_orderTmpConstraintPool.Capacity = ( numConstraintPool );

			m_orderFrictionConstraintPool.Count = m_orderFrictionConstraintPool.Capacity = ( numFrictionPool );
			{
				int i;
				for( i = 0; i < numNonContactPool; i++ )
				{
					m_orderNonContactConstraintPool[i] = i;
				}
				for( i = 0; i < numConstraintPool; i++ )
				{
					m_orderTmpConstraintPool[i] = i;
				}
				for( i = 0; i < numFrictionPool; i++ )
				{
					m_orderFrictionConstraintPool[i] = i;
				}
			}

			return 0;

		}


		protected virtual double solveSingleIteration( int iteration, btCollisionObject[] bodies, int numBodies
			, btPersistentManifold[] manifoldPtr, int start_manifold, int numManifolds
			, btTypedConstraint[] constraints, int startConstraint, int numConstraints
			, btContactSolverInfo infoGlobal, btIDebugDraw debugDrawer )
		{

			int numNonContactPool = m_tmpSolverNonContactConstraintPool.Count;
			int numConstraintPool = m_tmpSolverContactConstraintPool.Count;
			int numFrictionPool = m_tmpSolverContactFrictionConstraintPool.Count;

			if( ( infoGlobal.m_solverMode & btSolverMode.SOLVER_RANDMIZE_ORDER ) != 0 )
			{
				if( true )         // uncomment this for a bit less random ((iteration & 7) == 0)
				{

					for( int j = 0; j < numNonContactPool; ++j )
					{
						int tmp = m_orderNonContactConstraintPool[j];
						int swapi = btRandInt2( j + 1 );
						m_orderNonContactConstraintPool[j] = m_orderNonContactConstraintPool[swapi];
						m_orderNonContactConstraintPool[swapi] = tmp;
					}

					//contact/friction constraints are not solved more than
					if( iteration < infoGlobal.m_numIterations )
					{
						for( int j = 0; j < numConstraintPool; ++j )
						{
							int tmp = m_orderTmpConstraintPool[j];
							int swapi = btRandInt2( j + 1 );
							m_orderTmpConstraintPool[j] = m_orderTmpConstraintPool[swapi];
							m_orderTmpConstraintPool[swapi] = tmp;
						}

						for( int j = 0; j < numFrictionPool; ++j )
						{
							int tmp = m_orderFrictionConstraintPool[j];
							int swapi = btRandInt2( j + 1 );
							m_orderFrictionConstraintPool[j] = m_orderFrictionConstraintPool[swapi];
							m_orderFrictionConstraintPool[swapi] = tmp;
						}
					}
				}
			}

			if( ( infoGlobal.m_solverMode & btSolverMode.SOLVER_SIMD ) != 0 )
			{
				///solve all joint constraints, using SIMD, if available
				for( int j = 0; j < m_tmpSolverNonContactConstraintPool.Count; j++ )
				{
					btSolverConstraint constraint = m_tmpSolverNonContactConstraintPool[m_orderNonContactConstraintPool[j]];
					if( iteration < constraint.m_overrideNumSolverIterations )
						resolveSingleConstraintRowGenericSIMD( m_tmpSolverBodyPool[constraint.m_solverBodyIdA]
							, m_tmpSolverBodyPool[constraint.m_solverBodyIdB], constraint );
				}

				if( iteration < infoGlobal.m_numIterations )
				{
					for( int j = 0; j < numConstraints; j++ )
					{
						btTypedConstraint constraint = constraints[j + startConstraint];
						if( constraints[j + startConstraint].isEnabled() )
						{
							int bodyAid = getOrInitSolverBody( constraints[j + startConstraint].getRigidBodyA(), infoGlobal.m_timeStep );
							int bodyBid = getOrInitSolverBody( constraints[j + startConstraint].getRigidBodyB(), infoGlobal.m_timeStep );
							btSolverBody bodyA = m_tmpSolverBodyPool[bodyAid];
							btSolverBody bodyB = m_tmpSolverBodyPool[bodyBid];
							constraints[j + startConstraint].solveConstraintObsolete( bodyA, bodyB, infoGlobal.m_timeStep );
						}
					}

					///solve all contact constraints using SIMD, if available
					if( ( infoGlobal.m_solverMode & btSolverMode.SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS ) != 0 )
					{
						int numPoolConstraints = m_tmpSolverContactConstraintPool.Count;
						int multiplier = ( ( infoGlobal.m_solverMode & btSolverMode.SOLVER_USE_2_FRICTION_DIRECTIONS ) != 0 ) ? 2 : 1;

						for( int c = 0; c < numPoolConstraints; c++ )
						{
							double totalImpulse = 0;

							{
								btSolverConstraint solveManifold = m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[c]];
								resolveSingleConstraintRowLowerLimitSIMD( m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA], m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB], solveManifold );
								totalImpulse = solveManifold.m_appliedImpulse;
							}
							bool applyFriction = true;
							if( applyFriction )
							{
								{

									btSolverConstraint solveManifold = m_tmpSolverContactFrictionConstraintPool[m_orderFrictionConstraintPool[c * multiplier]];

									if( totalImpulse > (double)( 0 ) )
									{
										solveManifold.m_lowerLimit = -( solveManifold.m_friction * totalImpulse );
										solveManifold.m_upperLimit = solveManifold.m_friction * totalImpulse;

										resolveSingleConstraintRowGenericSIMD( m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA], m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB], solveManifold );
									}
								}

								if( ( infoGlobal.m_solverMode & btSolverMode.SOLVER_USE_2_FRICTION_DIRECTIONS ) != 0 )
								{

									btSolverConstraint solveManifold = m_tmpSolverContactFrictionConstraintPool[m_orderFrictionConstraintPool[c * multiplier + 1]];

									if( totalImpulse > (double)( 0 ) )
									{
										solveManifold.m_lowerLimit = -( solveManifold.m_friction * totalImpulse );
										solveManifold.m_upperLimit = solveManifold.m_friction * totalImpulse;

										resolveSingleConstraintRowGenericSIMD( m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA], m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB], solveManifold );
									}
								}
							}
						}

					}
					else//SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS
					{
						//solve the friction constraints after all contact constraints, don't interleave them
						int numPoolConstraints = m_tmpSolverContactConstraintPool.Count;
						int j;

						for( j = 0; j < numPoolConstraints; j++ )
						{
							btSolverConstraint solveManifold = m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[j]];
							resolveSingleConstraintRowLowerLimitSIMD( m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA], m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB], solveManifold );

						}



						///solve all friction constraints, using SIMD, if available

						int numFrictionPoolConstraints = m_tmpSolverContactFrictionConstraintPool.Count;
						for( j = 0; j < numFrictionPoolConstraints; j++ )
						{
							btSolverConstraint solveManifold = m_tmpSolverContactFrictionConstraintPool[m_orderFrictionConstraintPool[j]];
							double totalImpulse = m_tmpSolverContactConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;

							if( totalImpulse > (double)( 0 ) )
							{
								solveManifold.m_lowerLimit = -( solveManifold.m_friction * totalImpulse );
								solveManifold.m_upperLimit = solveManifold.m_friction * totalImpulse;

								resolveSingleConstraintRowGenericSIMD( m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA], m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB], solveManifold );
							}
						}


						int numRollingFrictionPoolConstraints = m_tmpSolverContactRollingFrictionConstraintPool.Count;
						for( j = 0; j < numRollingFrictionPoolConstraints; j++ )
						{

							btSolverConstraint rollingFrictionConstraint = m_tmpSolverContactRollingFrictionConstraintPool[j];
							double totalImpulse = m_tmpSolverContactConstraintPool[rollingFrictionConstraint.m_frictionIndex].m_appliedImpulse;
							if( totalImpulse > (double)( 0 ) )
							{
								double rollingFrictionMagnitude = rollingFrictionConstraint.m_friction * totalImpulse;
								if( rollingFrictionMagnitude > rollingFrictionConstraint.m_friction )
									rollingFrictionMagnitude = rollingFrictionConstraint.m_friction;

								rollingFrictionConstraint.m_lowerLimit = -rollingFrictionMagnitude;
								rollingFrictionConstraint.m_upperLimit = rollingFrictionMagnitude;

								resolveSingleConstraintRowGenericSIMD( m_tmpSolverBodyPool[rollingFrictionConstraint.m_solverBodyIdA], m_tmpSolverBodyPool[rollingFrictionConstraint.m_solverBodyIdB], rollingFrictionConstraint );
							}
						}


					}
				}
			}
			else
			{
				//non-SIMD version
				///solve all joint constraints
				for( int j = 0; j < m_tmpSolverNonContactConstraintPool.Count; j++ )
				{
					btSolverConstraint constraint = m_tmpSolverNonContactConstraintPool[m_orderNonContactConstraintPool[j]];
					if( iteration < constraint.m_overrideNumSolverIterations )
						resolveSingleConstraintRowGeneric( m_tmpSolverBodyPool[constraint.m_solverBodyIdA], m_tmpSolverBodyPool[constraint.m_solverBodyIdB], constraint );
				}

				if( iteration < infoGlobal.m_numIterations )
				{
					for( int j = 0; j < numConstraints; j++ )
					{
						btTypedConstraint constraint = constraints[j + startConstraint];
						if( constraint.isEnabled() )
						{
							int bodyAid = getOrInitSolverBody( constraint.getRigidBodyA(), infoGlobal.m_timeStep );
							int bodyBid = getOrInitSolverBody( constraint.getRigidBodyB(), infoGlobal.m_timeStep );
							btSolverBody bodyA = m_tmpSolverBodyPool[bodyAid];
							btSolverBody bodyB = m_tmpSolverBodyPool[bodyBid];
							constraint.solveConstraintObsolete( bodyA, bodyB, infoGlobal.m_timeStep );
						}
					}
					///solve all contact constraints
					int numPoolConstraints = m_tmpSolverContactConstraintPool.Count;
					for( int j = 0; j < numPoolConstraints; j++ )
					{
						btSolverConstraint solveManifold = m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[j]];
						resolveSingleConstraintRowLowerLimit( m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA], m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB], solveManifold );
					}
					///solve all friction constraints
					int numFrictionPoolConstraints = m_tmpSolverContactFrictionConstraintPool.Count;
					for( int j = 0; j < numFrictionPoolConstraints; j++ )
					{
						btSolverConstraint solveManifold = m_tmpSolverContactFrictionConstraintPool[m_orderFrictionConstraintPool[j]];
						double totalImpulse = m_tmpSolverContactConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;

						if( totalImpulse > (double)( 0 ) )
						{
							solveManifold.m_lowerLimit = -( solveManifold.m_friction * totalImpulse );
							solveManifold.m_upperLimit = solveManifold.m_friction * totalImpulse;

							resolveSingleConstraintRowGeneric( m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA], m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB], solveManifold );
						}
					}

					int numRollingFrictionPoolConstraints = m_tmpSolverContactRollingFrictionConstraintPool.Count;
					for( int j = 0; j < numRollingFrictionPoolConstraints; j++ )
					{
						btSolverConstraint rollingFrictionConstraint = m_tmpSolverContactRollingFrictionConstraintPool[j];
						double totalImpulse = m_tmpSolverContactConstraintPool[rollingFrictionConstraint.m_frictionIndex].m_appliedImpulse;
						if( totalImpulse > (double)( 0 ) )
						{
							double rollingFrictionMagnitude = rollingFrictionConstraint.m_friction * totalImpulse;
							if( rollingFrictionMagnitude > rollingFrictionConstraint.m_friction )
								rollingFrictionMagnitude = rollingFrictionConstraint.m_friction;

							rollingFrictionConstraint.m_lowerLimit = -rollingFrictionMagnitude;
							rollingFrictionConstraint.m_upperLimit = rollingFrictionMagnitude;

							resolveSingleConstraintRowGeneric( m_tmpSolverBodyPool[rollingFrictionConstraint.m_solverBodyIdA], m_tmpSolverBodyPool[rollingFrictionConstraint.m_solverBodyIdB], rollingFrictionConstraint );
						}
					}
				}
			}
			return 0;
		}


		protected virtual void solveGroupCacheFriendlySplitImpulseIterations( btCollisionObject[] bodies, int numBodies
			, btPersistentManifold[] manifoldPtr, int start_manifold, int numManifolds
			, btTypedConstraint[] constraints, int startConstraint, int numConstraints, btContactSolverInfo infoGlobal, btIDebugDraw debugDrawer )
		{
			int iteration;
			if( infoGlobal.m_splitImpulse )
			{
				if( ( infoGlobal.m_solverMode & btSolverMode.SOLVER_SIMD ) != 0 )
				{
					for( iteration = 0; iteration < infoGlobal.m_numIterations; iteration++ )
					{
						{
							int numPoolConstraints = m_tmpSolverContactConstraintPool.Count;
							int j;
							for( j = 0; j < numPoolConstraints; j++ )
							{
								btSolverConstraint solveManifold = m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[j]];

								resolveSplitPenetrationSIMD( m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA], m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB], solveManifold );
							}
						}
					}
				}
				else
				{
					for( iteration = 0; iteration < infoGlobal.m_numIterations; iteration++ )
					{
						{
							int numPoolConstraints = m_tmpSolverContactConstraintPool.Count;
							int j;
							for( j = 0; j < numPoolConstraints; j++ )
							{
								btSolverConstraint solveManifold = m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[j]];

								resolveSplitPenetrationImpulseCacheFriendly( m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA], m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB], solveManifold );
							}
						}
					}
				}
			}
		}

		protected virtual double solveGroupCacheFriendlyIterations( btCollisionObject[] bodies, int numBodies
			, btPersistentManifold[] manifoldPtr, int start_manifold, int numManifolds
			, btTypedConstraint[] constraints, int startConstraint, int numConstraints
			, btContactSolverInfo infoGlobal, btIDebugDraw debugDrawer )
		{
			CProfileSample sample = new CProfileSample( "solveGroupCacheFriendlyIterations" );

			{
				///this is a special step to resolve penetrations (just for contacts)
				solveGroupCacheFriendlySplitImpulseIterations( bodies, numBodies, manifoldPtr, start_manifold, numManifolds, constraints, startConstraint, numConstraints, infoGlobal, debugDrawer );

				int maxIterations = m_maxOverrideNumSolverIterations > infoGlobal.m_numIterations ? m_maxOverrideNumSolverIterations : infoGlobal.m_numIterations;

				for( int iteration = 0; iteration < maxIterations; iteration++ )
				//for ( int iteration = maxIterations-1  ; iteration >= 0;iteration--)
				{
					solveSingleIteration( iteration, bodies, numBodies, manifoldPtr, start_manifold, numManifolds, constraints, startConstraint, numConstraints, infoGlobal, debugDrawer );
				}

			}
			return 0;
		}

		protected virtual double solveGroupCacheFriendlyFinish( btCollisionObject[] bodies, int numBodies, btContactSolverInfo infoGlobal )
		{
			int numPoolConstraints = m_tmpSolverContactConstraintPool.Count;
			int i, j;

			if( ( infoGlobal.m_solverMode & btSolverMode.SOLVER_USE_WARMSTARTING ) != 0 )
			{
				for( j = 0; j < numPoolConstraints; j++ )
				{
					btSolverConstraint solveManifold = m_tmpSolverContactConstraintPool[j];
					btManifoldPoint pt = solveManifold.m_originalContactPoint as btManifoldPoint;
					Debug.Assert( pt != null );
					pt.m_appliedImpulse = solveManifold.m_appliedImpulse;
					//	float f = m_tmpSolverContactFrictionConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;
					//	Console.WriteLine("pt.m_appliedImpulseLateral1 = %f\n", f);
					pt.m_appliedImpulseLateral1 = m_tmpSolverContactFrictionConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;
					//Console.WriteLine("pt.m_appliedImpulseLateral1 = %f\n", pt.m_appliedImpulseLateral1);
					if( ( infoGlobal.m_solverMode & btSolverMode.SOLVER_USE_2_FRICTION_DIRECTIONS ) != 0 )
					{
						pt.m_appliedImpulseLateral2 = m_tmpSolverContactFrictionConstraintPool[solveManifold.m_frictionIndex + 1].m_appliedImpulse;
					}
					//do a callback here?
				}
			}

			numPoolConstraints = m_tmpSolverNonContactConstraintPool.Count;
			for( j = 0; j < numPoolConstraints; j++ )
			{
				btSolverConstraint solverConstr = m_tmpSolverNonContactConstraintPool[j];
				btTypedConstraint constr = (btTypedConstraint)solverConstr.m_originalContactPoint;
				btTypedConstraint.btJointFeedback fb = constr.getJointFeedback();
				if( fb != null )
				{
					fb.m_appliedForceBodyA += solverConstr.m_contactNormal1 * solverConstr.m_appliedImpulse * constr.getRigidBodyA().getLinearFactor() / infoGlobal.m_timeStep;
					fb.m_appliedForceBodyB += solverConstr.m_contactNormal2 * solverConstr.m_appliedImpulse * constr.getRigidBodyB().getLinearFactor() / infoGlobal.m_timeStep;
					fb.m_appliedTorqueBodyA += solverConstr.m_relpos1CrossNormal * constr.getRigidBodyA().getAngularFactor() * solverConstr.m_appliedImpulse / infoGlobal.m_timeStep;
					fb.m_appliedTorqueBodyB += solverConstr.m_relpos2CrossNormal * constr.getRigidBodyB().getAngularFactor() * solverConstr.m_appliedImpulse / infoGlobal.m_timeStep; /*RGM ???? */

				}

				constr.internalSetAppliedImpulse( solverConstr.m_appliedImpulse );
				if( btScalar.btFabs( solverConstr.m_appliedImpulse ) >= constr.getBreakingImpulseThreshold() )
				{
					constr.setEnabled( false );
				}
			}



			for( i = 0; i < m_tmpSolverBodyPool.Count; i++ )
			{
				btSolverBody solverBody = m_tmpSolverBodyPool[i];
				btRigidBody body = solverBody.m_originalBody;
				if( body != null )
				{
					if( infoGlobal.m_splitImpulse )
						solverBody.writebackVelocityAndTransform( infoGlobal.m_timeStep, infoGlobal.m_splitImpulseTurnErp );
					else
						solverBody.writebackVelocity();

					btVector3 tmp;
					solverBody.m_linearVelocity.Add( ref solverBody.m_externalForceImpulse, out tmp );
					solverBody.m_originalBody.setLinearVelocity( ref tmp );

					solverBody.m_angularVelocity.Add( ref solverBody.m_externalTorqueImpulse, out tmp );
					solverBody.m_originalBody.setAngularVelocity( ref tmp );

					if( infoGlobal.m_splitImpulse )
						solverBody.m_originalBody.setWorldTransform( (btITransform)solverBody.m_worldTransform );

					solverBody.m_originalBody.setCompanionId( -1 );
				}
			}

			m_tmpSolverContactConstraintPool.Count = 0; // resizeNoInitialize( 0 );
			m_tmpSolverNonContactConstraintPool.Count = 0; //resizeNoInitialize( 0 );
			m_tmpSolverContactFrictionConstraintPool.Count = 0; //resizeNoInitialize( 0 );
			m_tmpSolverContactRollingFrictionConstraintPool.Count = 0; //resizeNoInitialize( 0 );

			m_tmpSolverBodyPool.Count = 0; //resizeNoInitialize( 0 );
			return 0;
		}



		/// btSequentialImpulseConstraintSolver Sequentially applies impulses
		internal override double solveGroup( btCollisionObject[] bodies, int numBodies
			, btPersistentManifold[] manifoldPtr, int start_manifold, int numManifolds
			, btTypedConstraint[] constraints, int startConstraint, int numConstraints
			, btContactSolverInfo infoGlobal
			, btIDebugDraw debugDrawer, btDispatcher dispatcher )
		{
			CProfileSample sample = new CProfileSample( "solveGroup" );
			//you need to provide at least some bodies

			solveGroupCacheFriendlySetup( bodies, numBodies, manifoldPtr, start_manifold, numManifolds, constraints, startConstraint, numConstraints, infoGlobal, debugDrawer );

			solveGroupCacheFriendlyIterations( bodies, numBodies, manifoldPtr, start_manifold, numManifolds, constraints, startConstraint, numConstraints, infoGlobal, debugDrawer );

			solveGroupCacheFriendlyFinish( bodies, numBodies, infoGlobal );

			return 0;
		}

		///clear internal cached data and reset random seed
		internal override void reset()
		{
			m_btSeed2 = 0;
		}

	};




}
