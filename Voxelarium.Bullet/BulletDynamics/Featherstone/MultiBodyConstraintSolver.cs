/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "btMultiBodyConstraintSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "btMultiBodyLinkCollider.h"

#include "BulletDynamics/ConstraintSolver/btSolverBody.h"
#include "btMultiBodyConstraint.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"

#include "LinearMath/btQuickprof.h"

double btMultiBodyConstraintSolver::solveSingleIteration(int iteration, btCollisionObject bodies ,int numBodies,btPersistentManifold[] manifoldPtr, int numManifolds,btTypedConstraint[] constraints,int numConstraints,btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer)
{
	double val = btSequentialImpulseConstraintSolver::solveSingleIteration(iteration, bodies ,numBodies,manifoldPtr, numManifolds,constraints,numConstraints,infoGlobal,debugDrawer);
	
	//solve featherstone non-contact constraints

	//Console.WriteLine("m_multiBodyNonContactConstraints = %d\n",m_multiBodyNonContactConstraints.Count);
	for (int j=0;j<m_multiBodyNonContactConstraints.Count;j++)
	{
		btMultiBodySolverConstraint& constraint = m_multiBodyNonContactConstraints[j];
		//if (iteration < constraint.m_overrideNumSolverIterations)
			//resolveSingleConstraintRowGenericMultiBody(constraint);
		resolveSingleConstraintRowGeneric(constraint);
		if(constraint.m_multiBodyA) 
			constraint.m_multiBodyA.setPosUpdated(false);
		if(constraint.m_multiBodyB) 
			constraint.m_multiBodyB.setPosUpdated(false);
	}

	//solve featherstone normal contact
	for (int j=0;j<m_multiBodyNormalContactConstraints.Count;j++)
	{
		btMultiBodySolverConstraint& constraint = m_multiBodyNormalContactConstraints[j];
		if (iteration < infoGlobal.m_numIterations)
			resolveSingleConstraintRowGeneric(constraint);

		if(constraint.m_multiBodyA) 
			constraint.m_multiBodyA.setPosUpdated(false);
		if(constraint.m_multiBodyB) 
			constraint.m_multiBodyB.setPosUpdated(false);
	}
	
	//solve featherstone frictional contact

	for (int j=0;j<this.m_multiBodyFrictionContactConstraints.Count;j++)
	{
		if (iteration < infoGlobal.m_numIterations)
		{
			btMultiBodySolverConstraint& frictionConstraint = m_multiBodyFrictionContactConstraints[j];
			double totalImpulse = m_multiBodyNormalContactConstraints[frictionConstraint.m_frictionIndex].m_appliedImpulse;
			//adjust friction limits here
			if (totalImpulse>(double)(0))
			{
				frictionConstraint.m_lowerLimit = -(frictionConstraint.m_friction*totalImpulse);
				frictionConstraint.m_upperLimit = frictionConstraint.m_friction*totalImpulse;
				resolveSingleConstraintRowGeneric(frictionConstraint);

				if(frictionConstraint.m_multiBodyA) 
					frictionConstraint.m_multiBodyA.setPosUpdated(false);
				if(frictionConstraint.m_multiBodyB) 
					frictionConstraint.m_multiBodyB.setPosUpdated(false);
			}
		}
	}
	return val;
}

double btMultiBodyConstraintSolver::solveGroupCacheFriendlySetup(btCollisionObject bodies,int numBodies,btPersistentManifold[] manifoldPtr, int numManifolds,btTypedConstraint[] constraints,int numConstraints,btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer)
{
	m_multiBodyNonContactConstraints.resize(0);
	m_multiBodyNormalContactConstraints.resize(0);
	m_multiBodyFrictionContactConstraints.resize(0);
	m_data.m_jacobians.resize(0);
	m_data.m_deltaVelocitiesUnitImpulse.resize(0);
	m_data.m_deltaVelocities.resize(0);

	for (int i=0;i<numBodies;i++)
	{
		btMultiBodyLinkCollider* fcA = btMultiBodyLinkCollider::upcast(bodies[i]);
		if (fcA)
		{
			fcA.m_multiBody.setCompanionId(-1);
		}
	}

	double val = btSequentialImpulseConstraintSolver::solveGroupCacheFriendlySetup( bodies,numBodies,manifoldPtr, numManifolds, constraints,numConstraints,infoGlobal,debugDrawer);

	return val;
}

void	btMultiBodyConstraintSolver::applyDeltaVee(double* delta_vee, double impulse, int velocityIndex, int ndof)
{
    for (int i = 0; i < ndof; ++i) 
		m_data.m_deltaVelocities[velocityIndex+i] += delta_vee[i] * impulse;
}

void btMultiBodyConstraintSolver::resolveSingleConstraintRowGeneric(btMultiBodySolverConstraint& c)
{

	double deltaImpulse = c.m_rhs-(double)(c.m_appliedImpulse)*c.m_cfm;
	double deltaVelADotn=0;
	double deltaVelBDotn=0;
	btSolverBody* bodyA = 0;
	btSolverBody* bodyB = 0;
	int ndofA=0;
	int ndofB=0;

	if (c.m_multiBodyA)
	{
		ndofA  = (c.m_multiBodyA.isMultiDof() ? c.m_multiBodyA.getNumDofs() : c.m_multiBodyA.getNumLinks()) + 6;
		for (int i = 0; i < ndofA; ++i) 
			deltaVelADotn += m_data.m_jacobians[c.m_jacAindex+i] * m_data.m_deltaVelocities[c.m_deltaVelAindex+i];
	} else if(c.m_solverBodyIdA >= 0)
	{
		bodyA = &m_tmpSolverBodyPool[c.m_solverBodyIdA];
		deltaVelADotn += c.m_contactNormal1.dot(bodyA.internalGetDeltaLinearVelocity()) 	+ c.m_relpos1CrossNormal.dot(bodyA.internalGetDeltaAngularVelocity());
	}

	if (c.m_multiBodyB)
	{
		ndofB  = (c.m_multiBodyB.isMultiDof() ? c.m_multiBodyB.getNumDofs() : c.m_multiBodyB.getNumLinks()) + 6;
		for (int i = 0; i < ndofB; ++i) 
			deltaVelBDotn += m_data.m_jacobians[c.m_jacBindex+i] * m_data.m_deltaVelocities[c.m_deltaVelBindex+i];
	} else if(c.m_solverBodyIdB >= 0)
	{
		bodyB = &m_tmpSolverBodyPool[c.m_solverBodyIdB];
		deltaVelBDotn += c.m_contactNormal2.dot(bodyB.internalGetDeltaLinearVelocity())  + c.m_relpos2CrossNormal.dot(bodyB.internalGetDeltaAngularVelocity());
	}

	
	deltaImpulse	-=	deltaVelADotn*c.m_jacDiagABInv;//m_jacDiagABInv = 1./denom
	deltaImpulse	-=	deltaVelBDotn*c.m_jacDiagABInv;
	double sum = (double)(c.m_appliedImpulse) + deltaImpulse;
	
	if (sum < c.m_lowerLimit)
	{
		deltaImpulse = c.m_lowerLimit-c.m_appliedImpulse;
		c.m_appliedImpulse = c.m_lowerLimit;
	}
	else if (sum > c.m_upperLimit) 
	{
		deltaImpulse = c.m_upperLimit-c.m_appliedImpulse;
		c.m_appliedImpulse = c.m_upperLimit;
	}
	else
	{
		c.m_appliedImpulse = sum;
	}

	if (c.m_multiBodyA)
	{
		applyDeltaVee(&m_data.m_deltaVelocitiesUnitImpulse[c.m_jacAindex],deltaImpulse,c.m_deltaVelAindex,ndofA);
#if DIRECTLY_UPDATE_VELOCITY_DURING_SOLVER_ITERATIONS
		//note: update of the actual velocities (below) in the multibody does not have to happen now since m_deltaVelocities can be applied after all iterations
		//it would make the multibody solver more like the regular one with m_deltaVelocities being equivalent to btSolverBody::m_deltaLinearVelocity/m_deltaAngularVelocity
		if(c.m_multiBodyA.isMultiDof())
			c.m_multiBodyA.applyDeltaVeeMultiDof2(&m_data.m_deltaVelocitiesUnitImpulse[c.m_jacAindex],deltaImpulse);
		else
			c.m_multiBodyA.applyDeltaVee(&m_data.m_deltaVelocitiesUnitImpulse[c.m_jacAindex],deltaImpulse);
#endif //DIRECTLY_UPDATE_VELOCITY_DURING_SOLVER_ITERATIONS
	} else if(c.m_solverBodyIdA >= 0)
	{
		bodyA.internalApplyImpulse(c.m_contactNormal1*bodyA.internalGetInvMass(),c.m_angularComponentA,deltaImpulse);

	}
	if (c.m_multiBodyB)
	{
		applyDeltaVee(&m_data.m_deltaVelocitiesUnitImpulse[c.m_jacBindex],deltaImpulse,c.m_deltaVelBindex,ndofB);
#if DIRECTLY_UPDATE_VELOCITY_DURING_SOLVER_ITERATIONS
		//note: update of the actual velocities (below) in the multibody does not have to happen now since m_deltaVelocities can be applied after all iterations
		//it would make the multibody solver more like the regular one with m_deltaVelocities being equivalent to btSolverBody::m_deltaLinearVelocity/m_deltaAngularVelocity
		if(c.m_multiBodyB.isMultiDof())
			c.m_multiBodyB.applyDeltaVeeMultiDof2(&m_data.m_deltaVelocitiesUnitImpulse[c.m_jacBindex],deltaImpulse);
		else
			c.m_multiBodyB.applyDeltaVee(&m_data.m_deltaVelocitiesUnitImpulse[c.m_jacBindex],deltaImpulse);
#endif //DIRECTLY_UPDATE_VELOCITY_DURING_SOLVER_ITERATIONS
	} else if(c.m_solverBodyIdB >= 0)
	{
		bodyB.internalApplyImpulse(c.m_contactNormal2*bodyB.internalGetInvMass(),c.m_angularComponentB,deltaImpulse);
	}

}


void btMultiBodyConstraintSolver::resolveSingleConstraintRowGenericMultiBody(btMultiBodySolverConstraint& c)
{

	double deltaImpulse = c.m_rhs-(double)(c.m_appliedImpulse)*c.m_cfm;
	double deltaVelADotn=0;
	double deltaVelBDotn=0;
	int ndofA=0;
	int ndofB=0;

	if (c.m_multiBodyA)
	{
		ndofA  = (c.m_multiBodyA.isMultiDof() ? c.m_multiBodyA.getNumDofs() : c.m_multiBodyA.getNumLinks()) + 6;
		for (int i = 0; i < ndofA; ++i) 
			deltaVelADotn += m_data.m_jacobians[c.m_jacAindex+i] * m_data.m_deltaVelocities[c.m_deltaVelAindex+i];
	}

	if (c.m_multiBodyB)
	{
		ndofB  = (c.m_multiBodyB.isMultiDof() ? c.m_multiBodyB.getNumDofs() : c.m_multiBodyB.getNumLinks()) + 6;
		for (int i = 0; i < ndofB; ++i) 
			deltaVelBDotn += m_data.m_jacobians[c.m_jacBindex+i] * m_data.m_deltaVelocities[c.m_deltaVelBindex+i];
	}

	
	deltaImpulse	-=	deltaVelADotn*c.m_jacDiagABInv;//m_jacDiagABInv = 1./denom
	deltaImpulse	-=	deltaVelBDotn*c.m_jacDiagABInv;
	double sum = (double)(c.m_appliedImpulse) + deltaImpulse;
	
	if (sum < c.m_lowerLimit)
	{
		deltaImpulse = c.m_lowerLimit-c.m_appliedImpulse;
		c.m_appliedImpulse = c.m_lowerLimit;
	}
	else if (sum > c.m_upperLimit) 
	{
		deltaImpulse = c.m_upperLimit-c.m_appliedImpulse;
		c.m_appliedImpulse = c.m_upperLimit;
	}
	else
	{
		c.m_appliedImpulse = sum;
	}

	if (c.m_multiBodyA)
	{
		applyDeltaVee(&m_data.m_deltaVelocitiesUnitImpulse[c.m_jacAindex],deltaImpulse,c.m_deltaVelAindex,ndofA);
		c.m_multiBodyA.applyDeltaVee(&m_data.m_deltaVelocitiesUnitImpulse[c.m_jacAindex],deltaImpulse);
	}
	if (c.m_multiBodyB)
	{
		applyDeltaVee(&m_data.m_deltaVelocitiesUnitImpulse[c.m_jacBindex],deltaImpulse,c.m_deltaVelBindex,ndofB);
		c.m_multiBodyB.applyDeltaVee(&m_data.m_deltaVelocitiesUnitImpulse[c.m_jacBindex],deltaImpulse);
	}
}


void btMultiBodyConstraintSolver::setupMultiBodyContactConstraint(btMultiBodySolverConstraint& solverConstraint, 
																 ref btVector3 contactNormal,
																 btManifoldPoint cp, btContactSolverInfo& infoGlobal,
																 double relaxation,
																 bool isFriction, double desiredVelocity, double cfmSlip)
{
			
	CProfileSample sample = new CProfileSample("setupMultiBodyContactConstraint");
	btVector3 rel_pos1;
	btVector3 rel_pos2;

	btMultiBody* multiBodyA = solverConstraint.m_multiBodyA;
	btMultiBody* multiBodyB = solverConstraint.m_multiBodyB;

	ref btVector3 pos1 = cp.getPositionWorldOnA();
	ref btVector3 pos2 = cp.getPositionWorldOnB();

	btSolverBody* bodyA = multiBodyA ? 0 : &m_tmpSolverBodyPool[solverConstraint.m_solverBodyIdA];
	btSolverBody* bodyB = multiBodyB ? 0 : &m_tmpSolverBodyPool[solverConstraint.m_solverBodyIdB];

	btRigidBody rb0 = multiBodyA ? 0 : bodyA.m_originalBody;
	btRigidBody rb1 = multiBodyB ? 0 : bodyB.m_originalBody;

	if (bodyA)
		rel_pos1 = pos1 - bodyA.getWorldTransform().getOrigin(); 
	if (bodyB)
		rel_pos2 = pos2 - bodyB.getWorldTransform().getOrigin();

	relaxation = 1;

	


	if (multiBodyA)
	{
		if (solverConstraint.m_linkA<0)
		{
			rel_pos1 = pos1 - multiBodyA.getBasePos();
		} else
		{
			rel_pos1 = pos1 - multiBodyA.getLink(solverConstraint.m_linkA).m_cachedWorldTransform.getOrigin();
		}
		int ndofA  = (multiBodyA.isMultiDof() ? multiBodyA.getNumDofs() : multiBodyA.getNumLinks()) + 6;

		solverConstraint.m_deltaVelAindex = multiBodyA.getCompanionId();

		if (solverConstraint.m_deltaVelAindex <0)
		{
			solverConstraint.m_deltaVelAindex = m_data.m_deltaVelocities.Count;
			multiBodyA.setCompanionId(solverConstraint.m_deltaVelAindex);
			m_data.m_deltaVelocities.resize(m_data.m_deltaVelocities.Count+ndofA);
		} else
		{
			Debug.Assert(m_data.m_deltaVelocities.Count >= solverConstraint.m_deltaVelAindex+ndofA);
		}

		solverConstraint.m_jacAindex = m_data.m_jacobians.Count;
		m_data.m_jacobians.resize(m_data.m_jacobians.Count+ndofA);
		m_data.m_deltaVelocitiesUnitImpulse.resize(m_data.m_deltaVelocitiesUnitImpulse.Count+ndofA);
		Debug.Assert(m_data.m_jacobians.Count == m_data.m_deltaVelocitiesUnitImpulse.Count);

		double* jac1=&m_data.m_jacobians[solverConstraint.m_jacAindex];
		if(multiBodyA.isMultiDof())
			multiBodyA.fillContactJacobianMultiDof(solverConstraint.m_linkA, cp.getPositionWorldOnA(), contactNormal, jac1, m_data.scratch_r, m_data.scratch_v, m_data.scratch_m);
		else
			multiBodyA.fillContactJacobian(solverConstraint.m_linkA, cp.getPositionWorldOnA(), contactNormal, jac1, m_data.scratch_r, m_data.scratch_v, m_data.scratch_m);
		double* delta = &m_data.m_deltaVelocitiesUnitImpulse[solverConstraint.m_jacAindex];
		if(multiBodyA.isMultiDof())
			multiBodyA.calcAccelerationDeltasMultiDof(&m_data.m_jacobians[solverConstraint.m_jacAindex],delta,m_data.scratch_r, m_data.scratch_v);
		else
			multiBodyA.calcAccelerationDeltas(&m_data.m_jacobians[solverConstraint.m_jacAindex],delta,m_data.scratch_r, m_data.scratch_v);

		btVector3 torqueAxis0 = rel_pos1.cross(contactNormal);
		solverConstraint.m_relpos1CrossNormal = torqueAxis0;
		solverConstraint.m_contactNormal1 = contactNormal;
	} else
	{
		btVector3 torqueAxis0 = rel_pos1.cross(contactNormal);
		solverConstraint.m_relpos1CrossNormal = torqueAxis0;
		solverConstraint.m_contactNormal1 = contactNormal;
		solverConstraint.m_angularComponentA = rb0 ? rb0.getInvInertiaTensorWorld()*torqueAxis0*rb0.getAngularFactor() : btVector3(0,0,0);
	}

	

	if (multiBodyB)
	{
		if (solverConstraint.m_linkB<0)
		{
			rel_pos2 = pos2 - multiBodyB.getBasePos();
		} else
		{
			rel_pos2 = pos2 - multiBodyB.getLink(solverConstraint.m_linkB).m_cachedWorldTransform.getOrigin();
		}

		int ndofB  = (multiBodyB.isMultiDof() ? multiBodyB.getNumDofs() : multiBodyB.getNumLinks()) + 6;

		solverConstraint.m_deltaVelBindex = multiBodyB.getCompanionId();
		if (solverConstraint.m_deltaVelBindex <0)
		{
			solverConstraint.m_deltaVelBindex = m_data.m_deltaVelocities.Count;
			multiBodyB.setCompanionId(solverConstraint.m_deltaVelBindex);
			m_data.m_deltaVelocities.resize(m_data.m_deltaVelocities.Count+ndofB);
		}

		solverConstraint.m_jacBindex = m_data.m_jacobians.Count;

		m_data.m_jacobians.resize(m_data.m_jacobians.Count+ndofB);
		m_data.m_deltaVelocitiesUnitImpulse.resize(m_data.m_deltaVelocitiesUnitImpulse.Count+ndofB);
		Debug.Assert(m_data.m_jacobians.Count == m_data.m_deltaVelocitiesUnitImpulse.Count);

		if(multiBodyB.isMultiDof())
			multiBodyB.fillContactJacobianMultiDof(solverConstraint.m_linkB, cp.getPositionWorldOnB(), -contactNormal, &m_data.m_jacobians[solverConstraint.m_jacBindex], m_data.scratch_r, m_data.scratch_v, m_data.scratch_m);
		else
			multiBodyB.fillContactJacobian(solverConstraint.m_linkB, cp.getPositionWorldOnB(), -contactNormal, &m_data.m_jacobians[solverConstraint.m_jacBindex], m_data.scratch_r, m_data.scratch_v, m_data.scratch_m);
		if(multiBodyB.isMultiDof())
			multiBodyB.calcAccelerationDeltasMultiDof(&m_data.m_jacobians[solverConstraint.m_jacBindex],&m_data.m_deltaVelocitiesUnitImpulse[solverConstraint.m_jacBindex],m_data.scratch_r, m_data.scratch_v);
		else
			multiBodyB.calcAccelerationDeltas(&m_data.m_jacobians[solverConstraint.m_jacBindex],&m_data.m_deltaVelocitiesUnitImpulse[solverConstraint.m_jacBindex],m_data.scratch_r, m_data.scratch_v);
		
		btVector3 torqueAxis1 = rel_pos2.cross(contactNormal);		
		solverConstraint.m_relpos2CrossNormal = -torqueAxis1;
		solverConstraint.m_contactNormal2 = -contactNormal;
	
	} else
	{
		btVector3 torqueAxis1 = rel_pos2.cross(contactNormal);		
		solverConstraint.m_relpos2CrossNormal = -torqueAxis1;
		solverConstraint.m_contactNormal2 = -contactNormal;
	
		solverConstraint.m_angularComponentB = rb1 ? rb1.getInvInertiaTensorWorld()*-torqueAxis1*rb1.getAngularFactor() : btVector3(0,0,0);
	}

	{
						
		btVector3 vec;
		double denom0 = 0;
		double denom1 = 0;
		double* jacB = 0;
		double* jacA = 0;
		double* lambdaA =0;
		double* lambdaB =0;
		int ndofA  = 0;
		if (multiBodyA)
		{
			ndofA  = (multiBodyA.isMultiDof() ? multiBodyA.getNumDofs() : multiBodyA.getNumLinks()) + 6;
			jacA = &m_data.m_jacobians[solverConstraint.m_jacAindex];
			lambdaA = &m_data.m_deltaVelocitiesUnitImpulse[solverConstraint.m_jacAindex];
			for (int i = 0; i < ndofA; ++i)
			{
				double j = jacA[i] ;
				double l =lambdaA[i];
				denom0 += j*l;
			}
		} else
		{
			if (rb0)
			{
				vec = ( solverConstraint.m_angularComponentA).cross(rel_pos1);
				denom0 = rb0.getInvMass() + contactNormal.dot(vec);
			}
		}
		if (multiBodyB)
		{
			int ndofB  = (multiBodyB.isMultiDof() ? multiBodyB.getNumDofs() : multiBodyB.getNumLinks()) + 6;
			jacB = &m_data.m_jacobians[solverConstraint.m_jacBindex];
			lambdaB = &m_data.m_deltaVelocitiesUnitImpulse[solverConstraint.m_jacBindex];
			for (int i = 0; i < ndofB; ++i)
			{
				double j = jacB[i] ;
				double l =lambdaB[i];
				denom1 += j*l;
			}

		} else
		{
			if (rb1)
			{
				vec = ( -solverConstraint.m_angularComponentB).cross(rel_pos2);
				denom1 = rb1.getInvMass() + contactNormal.dot(vec);
			}
		}

		 

		 double d = denom0+denom1;
		 if (d>SIMD_EPSILON)
		 {
			solverConstraint.m_jacDiagABInv = relaxation/(d);
		 } else
		 {
			//disable the constraint row to handle singularity/redundant constraint
			solverConstraint.m_jacDiagABInv  = 0;
		 }
		
	}

	
	//compute rhs and remaining solverConstraint fields

	

	double restitution = 0;
	double penetration = isFriction? 0 : cp.getDistance()+infoGlobal.m_linearSlop;

	double rel_vel = 0;
	int ndofA  = 0;
	int ndofB  = 0;
	{

		btVector3 vel1,vel2;
		if (multiBodyA)
		{
			ndofA  = (multiBodyA.isMultiDof() ? multiBodyA.getNumDofs() : multiBodyA.getNumLinks()) + 6;
			double* jacA = &m_data.m_jacobians[solverConstraint.m_jacAindex];
			for (int i = 0; i < ndofA ; ++i) 
				rel_vel += multiBodyA.getVelocityVector()[i] * jacA[i];
		} else
		{
			if (rb0)
			{
				rel_vel += rb0.getVelocityInLocalPoint(rel_pos1).dot(solverConstraint.m_contactNormal1);
			}
		}
		if (multiBodyB)
		{
			ndofB  = (multiBodyB.isMultiDof() ? multiBodyB.getNumDofs() : multiBodyB.getNumLinks()) + 6;
			double* jacB = &m_data.m_jacobians[solverConstraint.m_jacBindex];
			for (int i = 0; i < ndofB ; ++i) 
				rel_vel += multiBodyB.getVelocityVector()[i] * jacB[i];

		} else
		{
			if (rb1)
			{
				rel_vel += rb1.getVelocityInLocalPoint(rel_pos2).dot(solverConstraint.m_contactNormal2);
			}
		}

		solverConstraint.m_friction = cp.m_combinedFriction;

		if(!isFriction)
		{
			restitution =  restitutionCurve(rel_vel, cp.m_combinedRestitution);	
			if (restitution <= btScalar.BT_ZERO)
			{
				restitution = 0;
			}
		}
	}


	///warm starting (or zero if disabled)
	//disable warmstarting for btMultiBody, it has issues gaining energy (==explosion)
	if (0)//infoGlobal.m_solverMode & SOLVER_USE_WARMSTARTING)
	{
		solverConstraint.m_appliedImpulse = isFriction ? 0 : cp.m_appliedImpulse * infoGlobal.m_warmstartingFactor;

		if (solverConstraint.m_appliedImpulse)
		{
			if (multiBodyA)
			{
				double impulse = solverConstraint.m_appliedImpulse;
				double* deltaV = &m_data.m_deltaVelocitiesUnitImpulse[solverConstraint.m_jacAindex];
				if(multiBodyA.isMultiDof())
					multiBodyA.applyDeltaVeeMultiDof(deltaV,impulse);
				else
					multiBodyA.applyDeltaVee(deltaV,impulse);
				applyDeltaVee(deltaV,impulse,solverConstraint.m_deltaVelAindex,ndofA);
			} else
			{
				if (rb0)
					bodyA.internalApplyImpulse(solverConstraint.m_contactNormal1*bodyA.internalGetInvMass()*rb0.getLinearFactor(),solverConstraint.m_angularComponentA,solverConstraint.m_appliedImpulse);
			}
			if (multiBodyB)
			{
				double impulse = solverConstraint.m_appliedImpulse;
				double* deltaV = &m_data.m_deltaVelocitiesUnitImpulse[solverConstraint.m_jacBindex];
				if(multiBodyB.isMultiDof())
					multiBodyB.applyDeltaVeeMultiDof(deltaV,impulse);
				else
					multiBodyB.applyDeltaVee(deltaV,impulse);
				applyDeltaVee(deltaV,impulse,solverConstraint.m_deltaVelBindex,ndofB);
			} else
			{
				if (rb1)
					bodyB.internalApplyImpulse(-solverConstraint.m_contactNormal2*bodyB.internalGetInvMass()*rb1.getLinearFactor(),-solverConstraint.m_angularComponentB,-(double)solverConstraint.m_appliedImpulse);
			}
		}
	} else
	{
		solverConstraint.m_appliedImpulse = 0;
	}

	solverConstraint.m_appliedPushImpulse = 0;

	{

		double positionalError = 0;
		double velocityError = restitution - rel_vel;// * damping;	//note for friction restitution is always set to 0 (check above) so it is acutally velocityError = -rel_vel for friction

		double erp = infoGlobal.m_erp2;
		if (!infoGlobal.m_splitImpulse || (penetration > infoGlobal.m_splitImpulsePenetrationThreshold))
		{
			erp = infoGlobal.m_erp;
		}

		if (penetration>0)
		{
			positionalError = 0;
			velocityError -= penetration / infoGlobal.m_timeStep;

		} else
		{
			positionalError = -penetration * erp/infoGlobal.m_timeStep;
		}

		double  penetrationImpulse = positionalError*solverConstraint.m_jacDiagABInv;
		double velocityImpulse = velocityError *solverConstraint.m_jacDiagABInv;

		if(!isFriction)
		{
			if (!infoGlobal.m_splitImpulse || (penetration > infoGlobal.m_splitImpulsePenetrationThreshold))
			{
				//combine position and velocity into rhs
				solverConstraint.m_rhs = penetrationImpulse+velocityImpulse;
				solverConstraint.m_rhsPenetration = 0;

			} else
			{
				//split position and velocity into rhs and m_rhsPenetration
				solverConstraint.m_rhs = velocityImpulse;
				solverConstraint.m_rhsPenetration = penetrationImpulse;
			}

			solverConstraint.m_lowerLimit = 0;
			solverConstraint.m_upperLimit = 1e10f;
		}
		else
		{
			solverConstraint.m_rhs = velocityImpulse;
			solverConstraint.m_rhsPenetration = 0;
			solverConstraint.m_lowerLimit = -solverConstraint.m_friction;
			solverConstraint.m_upperLimit = solverConstraint.m_friction;
		}

		solverConstraint.m_cfm = 0;			//why not use cfmSlip?
	}

}




btMultiBodySolverConstraint&	btMultiBodyConstraintSolver::addMultiBodyFrictionConstraint(ref btVector3 normalAxis,btPersistentManifold* manifold,int frictionIndex,btManifoldPoint cp,btCollisionObject colObj0,btCollisionObject colObj1, double relaxation, btContactSolverInfo& infoGlobal, double desiredVelocity, double cfmSlip)
{
	CProfileSample sample = new CProfileSample("addMultiBodyFrictionConstraint");
	btMultiBodySolverConstraint& solverConstraint = m_multiBodyFrictionContactConstraints.expandNonInitializing();
    solverConstraint.m_orgConstraint = 0;
    solverConstraint.m_orgDofIndex = -1;
    
	solverConstraint.m_frictionIndex = frictionIndex;
	bool isFriction = true;

	btMultiBodyLinkCollider* fcA = btMultiBodyLinkCollider::upcast(manifold.getBody0());
	btMultiBodyLinkCollider* fcB = btMultiBodyLinkCollider::upcast(manifold.getBody1());
	
	btMultiBody* mbA = fcA? fcA.m_multiBody : 0;
	btMultiBody* mbB = fcB? fcB.m_multiBody : 0;

	int solverBodyIdA = mbA? -1 : getOrInitSolverBody(*colObj0,infoGlobal.m_timeStep);
	int solverBodyIdB = mbB ? -1 : getOrInitSolverBody(*colObj1,infoGlobal.m_timeStep);

	solverConstraint.m_solverBodyIdA = solverBodyIdA;
	solverConstraint.m_solverBodyIdB = solverBodyIdB;
	solverConstraint.m_multiBodyA = mbA;
	if (mbA)
		solverConstraint.m_linkA = fcA.m_link;

	solverConstraint.m_multiBodyB = mbB;
	if (mbB)
		solverConstraint.m_linkB = fcB.m_link;

	solverConstraint.m_originalContactPoint = &cp;

	setupMultiBodyContactConstraint(solverConstraint, normalAxis, cp, infoGlobal,relaxation,isFriction, desiredVelocity, cfmSlip);
	return solverConstraint;
}

void	btMultiBodyConstraintSolver::convertMultiBodyContact(btPersistentManifold* manifold,btContactSolverInfo& infoGlobal)
{
	btMultiBodyLinkCollider* fcA = btMultiBodyLinkCollider::upcast(manifold.getBody0());
	btMultiBodyLinkCollider* fcB = btMultiBodyLinkCollider::upcast(manifold.getBody1());
	
	btMultiBody* mbA = fcA? fcA.m_multiBody : 0;
	btMultiBody* mbB = fcB? fcB.m_multiBody : 0;

	btCollisionObject colObj0=0,*colObj1=0;

	colObj0 = (btCollisionObject)manifold.getBody0();
	colObj1 = (btCollisionObject)manifold.getBody1();

	int solverBodyIdA = mbA? -1 : getOrInitSolverBody(*colObj0,infoGlobal.m_timeStep);
	int solverBodyIdB = mbB ? -1 : getOrInitSolverBody(*colObj1,infoGlobal.m_timeStep);

//	btSolverBody* solverBodyA = mbA ? 0 : &m_tmpSolverBodyPool[solverBodyIdA];
//	btSolverBody* solverBodyB = mbB ? 0 : &m_tmpSolverBodyPool[solverBodyIdB];


	///avoid collision response between two static objects
//	if (!solverBodyA || (solverBodyA.m_invMass.isZero() && (!solverBodyB || solverBodyB.m_invMass.isZero())))
	//	return;



	for (int j=0;j<manifold.getNumContacts();j++)
	{

		btManifoldPoint cp = manifold.getContactPoint(j);

		if (cp.getDistance() <= manifold.getContactProcessingThreshold())
		{
		
			double relaxation;

			int frictionIndex = m_multiBodyNormalContactConstraints.Count;

			btMultiBodySolverConstraint& solverConstraint = m_multiBodyNormalContactConstraints.expandNonInitializing();

	//		btRigidBody rb0 = btRigidBody::upcast(colObj0);
	//		btRigidBody rb1 = btRigidBody::upcast(colObj1);
            solverConstraint.m_orgConstraint = 0;
            solverConstraint.m_orgDofIndex = -1;
			solverConstraint.m_solverBodyIdA = solverBodyIdA;
			solverConstraint.m_solverBodyIdB = solverBodyIdB;
			solverConstraint.m_multiBodyA = mbA;
			if (mbA)
				solverConstraint.m_linkA = fcA.m_link;

			solverConstraint.m_multiBodyB = mbB;
			if (mbB)
				solverConstraint.m_linkB = fcB.m_link;

			solverConstraint.m_originalContactPoint = &cp;

			bool isFriction = false;
			setupMultiBodyContactConstraint(solverConstraint, cp.m_normalWorldOnB,cp, infoGlobal, relaxation, isFriction);

//			ref btVector3 pos1 = cp.getPositionWorldOnA();
//			ref btVector3 pos2 = cp.getPositionWorldOnB();

			/////setup the friction constraints
#define ENABLE_FRICTION
#if ENABLE_FRICTION
			solverConstraint.m_frictionIndex = frictionIndex;
#if ROLLING_FRICTION
	int rollingFriction=1;
			btVector3 angVelA(0,0,0),angVelB(0,0,0);
			if (rb0)
				angVelA = rb0.getAngularVelocity();
			if (rb1)
				angVelB = rb1.getAngularVelocity();
			btVector3 relAngVel = angVelB-angVelA;

			if ((cp.m_combinedRollingFriction>0) && (rollingFriction>0))
			{
				//only a single rollingFriction per manifold
				rollingFriction--;
				if (relAngVel.length()>infoGlobal.m_singleAxisRollingFrictionThreshold)
				{
					relAngVel.normalize();
					applyAnisotropicFriction(colObj0,relAngVel,btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
					applyAnisotropicFriction(colObj1,relAngVel,btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
					if (relAngVel.length()>0.001)
						addRollingFrictionConstraint(relAngVel,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);

				} else
				{
					addRollingFrictionConstraint(cp.m_normalWorldOnB,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
					btVector3 axis0,axis1;
					btPlaneSpace1(cp.m_normalWorldOnB,axis0,axis1);
					applyAnisotropicFriction(colObj0,axis0,btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
					applyAnisotropicFriction(colObj1,axis0,btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
					applyAnisotropicFriction(colObj0,axis1,btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
					applyAnisotropicFriction(colObj1,axis1,btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
					if (axis0.length()>0.001)
						addRollingFrictionConstraint(axis0,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
					if (axis1.length()>0.001)
						addRollingFrictionConstraint(axis1,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
		
				}
			}
#endif //ROLLING_FRICTION

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
			if (!(infoGlobal.m_solverMode & SOLVER_ENABLE_FRICTION_DIRECTION_CACHING) || !cp.m_lateralFrictionInitialized)
			{/*
				cp.m_lateralFrictionDir1 = vel - cp.m_normalWorldOnB * rel_vel;
				double lat_rel_vel = cp.m_lateralFrictionDir1.length2();
				if (!(infoGlobal.m_solverMode & SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION) && lat_rel_vel > SIMD_EPSILON)
				{
					cp.m_lateralFrictionDir1 *= 1/btSqrt(lat_rel_vel);
					if((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS))
					{
						cp.m_lateralFrictionDir2 = cp.m_lateralFrictionDir1.cross(cp.m_normalWorldOnB);
						cp.m_lateralFrictionDir2.normalize();//??
						applyAnisotropicFriction(colObj0,cp.m_lateralFrictionDir2,btCollisionObject::CF_ANISOTROPIC_FRICTION);
						applyAnisotropicFriction(colObj1,cp.m_lateralFrictionDir2,btCollisionObject::CF_ANISOTROPIC_FRICTION);
						addMultiBodyFrictionConstraint(cp.m_lateralFrictionDir2,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);

					}

					applyAnisotropicFriction(colObj0,cp.m_lateralFrictionDir1,btCollisionObject::CF_ANISOTROPIC_FRICTION);
					applyAnisotropicFriction(colObj1,cp.m_lateralFrictionDir1,btCollisionObject::CF_ANISOTROPIC_FRICTION);
					addMultiBodyFrictionConstraint(cp.m_lateralFrictionDir1,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);

				} else
				*/
				{
					btPlaneSpace1(cp.m_normalWorldOnB,cp.m_lateralFrictionDir1,cp.m_lateralFrictionDir2);

					applyAnisotropicFriction(colObj0,cp.m_lateralFrictionDir1,btCollisionObject::CF_ANISOTROPIC_FRICTION);
					applyAnisotropicFriction(colObj1,cp.m_lateralFrictionDir1,btCollisionObject::CF_ANISOTROPIC_FRICTION);
					addMultiBodyFrictionConstraint(cp.m_lateralFrictionDir1,manifold,frictionIndex,cp,colObj0,colObj1, relaxation,infoGlobal);

					if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS))
					{
						applyAnisotropicFriction(colObj0,cp.m_lateralFrictionDir2,btCollisionObject::CF_ANISOTROPIC_FRICTION);
						applyAnisotropicFriction(colObj1,cp.m_lateralFrictionDir2,btCollisionObject::CF_ANISOTROPIC_FRICTION);
						addMultiBodyFrictionConstraint(cp.m_lateralFrictionDir2,manifold,frictionIndex,cp,colObj0,colObj1, relaxation,infoGlobal);
					}

					if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS) && (infoGlobal.m_solverMode & SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION))
					{
						cp.m_lateralFrictionInitialized = true;
					}
				}

			} else
			{
				addMultiBodyFrictionConstraint(cp.m_lateralFrictionDir1,manifold,frictionIndex,cp,colObj0,colObj1, relaxation,infoGlobal,cp.m_contactMotion1, cp.m_contactCFM1);

				if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS))
					addMultiBodyFrictionConstraint(cp.m_lateralFrictionDir2,manifold,frictionIndex,cp,colObj0,colObj1, relaxation, infoGlobal,cp.m_contactMotion2, cp.m_contactCFM2);

				//setMultiBodyFrictionConstraintImpulse( solverConstraint, solverBodyIdA, solverBodyIdB, cp, infoGlobal);
				//todo:
				solverConstraint.m_appliedImpulse = 0;
				solverConstraint.m_appliedPushImpulse = 0;
			}
		

#endif //ENABLE_FRICTION

		}
	}
}

void btMultiBodyConstraintSolver::convertContacts(btPersistentManifold[] manifoldPtr,int numManifolds, btContactSolverInfo& infoGlobal)
{
	//btPersistentManifold* manifold = 0;

	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* manifold= manifoldPtr[i];
		btMultiBodyLinkCollider* fcA = btMultiBodyLinkCollider::upcast(manifold.getBody0());
		btMultiBodyLinkCollider* fcB = btMultiBodyLinkCollider::upcast(manifold.getBody1());
		if (!fcA && !fcB)
		{
			//the contact doesn't involve any Featherstone btMultiBody, so deal with the regular btRigidBody/btCollisionObject case
			convertContact(manifold,infoGlobal);
		} else
		{
			convertMultiBodyContact(manifold,infoGlobal);
		}
	}

	//also convert the multibody constraints, if any

	
	for (int i=0;i<m_tmpNumMultiBodyConstraints;i++)
	{
		btMultiBodyConstraint* c = m_tmpMultiBodyConstraints[i];
		m_data.m_solverBodyPool = &m_tmpSolverBodyPool;
		m_data.m_fixedBodyId = m_fixedBodyId;
		
		c.createConstraintRows(m_multiBodyNonContactConstraints,m_data,	infoGlobal);
	}

}



double btMultiBodyConstraintSolver::solveGroup(btCollisionObject bodies,int numBodies,btPersistentManifold[] manifold,int numManifolds,btTypedConstraint[] constraints,int numConstraints,btContactSolverInfo& info, btIDebugDraw* debugDrawer,btDispatcher* dispatcher)
{
	return btSequentialImpulseConstraintSolver::solveGroup(bodies,numBodies,manifold,numManifolds,constraints,numConstraints,info,debugDrawer,dispatcher);
}

#if 0
static void applyJointFeedback(btMultiBodyJacobianData& data, btMultiBodySolverConstraint& solverConstraint, int jacIndex, btMultiBody* mb, double appliedImpulse)
{
	if (appliedImpulse!=0 && mb.internalNeedsJointFeedback())
	{
		//todo: get rid of those temporary memory allocations for the joint feedback
		List<double> forceVector;
		int numDofsPlusBase = 6+mb.getNumDofs();
		forceVector.resize(numDofsPlusBase);
		for (int i=0;i<numDofsPlusBase;i++)
		{
			forceVector[i] = data.m_jacobians[jacIndex+i]*appliedImpulse;
		}
		List<double> output;
		output.resize(numDofsPlusBase);
		bool applyJointFeedback = true;
		mb.calcAccelerationDeltasMultiDof(forceVector,output,data.scratch_r,data.scratch_v,applyJointFeedback);
	}
}
#endif

#include "Bullet3Common/b3Logging.h"
void btMultiBodyConstraintSolver::writeBackSolverBodyToMultiBody(btMultiBodySolverConstraint c, double deltaTime)
{
#if 1 
	
	//bod.addBaseForce(m_gravity * bod.getBaseMass());
	//bod.addLinkForce(j, m_gravity * bod.getLinkMass(j));

	if (c.m_orgConstraint)
	{
		c.m_orgConstraint.internalSetAppliedImpulse(c.m_orgDofIndex,c.m_appliedImpulse);
	}
	

	if (c.m_multiBodyA)
	{
		
		if(c.m_multiBodyA.isMultiDof())
		{
			c.m_multiBodyA.setCompanionId(-1);
			btVector3 force = c.m_contactNormal1*(c.m_appliedImpulse/deltaTime);
			btVector3 torque = c.m_relpos1CrossNormal*(c.m_appliedImpulse/deltaTime);
			if (c.m_linkA<0)
			{
				c.m_multiBodyA.addBaseConstraintForce(force);
				c.m_multiBodyA.addBaseConstraintTorque(torque);
			} else
			{
				c.m_multiBodyA.addLinkConstraintForce(c.m_linkA,force);
					//b3Printf("force = %f,%f,%f\n",force,force[1],force[2]);//[0],torque[1],torque[2]);
				c.m_multiBodyA.addLinkConstraintTorque(c.m_linkA,torque);
			}
		}
	}
	
	if (c.m_multiBodyB)
	{
		if(c.m_multiBodyB.isMultiDof())
		{
			{
				c.m_multiBodyB.setCompanionId(-1);
				btVector3 force = c.m_contactNormal2*(c.m_appliedImpulse/deltaTime);
				btVector3 torque = c.m_relpos2CrossNormal*(c.m_appliedImpulse/deltaTime);
				if (c.m_linkB<0)
				{
					c.m_multiBodyB.addBaseConstraintForce(force);
					c.m_multiBodyB.addBaseConstraintTorque(torque);
				} else
				{
					{
						c.m_multiBodyB.addLinkConstraintForce(c.m_linkB,force);
						//b3Printf("t = %f,%f,%f\n",force[0],force[1],force[2]);//[0],torque[1],torque[2]);
						c.m_multiBodyB.addLinkConstraintTorque(c.m_linkB,torque);
					}
					
				}
			}
		}
	}
#endif

#if ! DIRECTLY_UPDATE_VELOCITY_DURING_SOLVER_ITERATIONS

	if (c.m_multiBodyA)
	{
		
		if(c.m_multiBodyA.isMultiDof())
		{
			c.m_multiBodyA.applyDeltaVeeMultiDof(&m_data.m_deltaVelocitiesUnitImpulse[c.m_jacAindex],c.m_appliedImpulse);
		}
		else
		{
			c.m_multiBodyA.applyDeltaVee(&m_data.m_deltaVelocitiesUnitImpulse[c.m_jacAindex],c.m_appliedImpulse);
		}
	}
	
	if (c.m_multiBodyB)
	{
		if(c.m_multiBodyB.isMultiDof())
		{
			c.m_multiBodyB.applyDeltaVeeMultiDof(&m_data.m_deltaVelocitiesUnitImpulse[c.m_jacBindex],c.m_appliedImpulse);
		}
		else
		{
			c.m_multiBodyB.applyDeltaVee(&m_data.m_deltaVelocitiesUnitImpulse[c.m_jacBindex],c.m_appliedImpulse);
		}
	}
#endif



}

double btMultiBodyConstraintSolver::solveGroupCacheFriendlyFinish(btCollisionObject bodies,int numBodies,btContactSolverInfo& infoGlobal)
{
	CProfileSample sample = new CProfileSample("btMultiBodyConstraintSolver::solveGroupCacheFriendlyFinish");
	int numPoolConstraints = m_multiBodyNormalContactConstraints.Count;
	

	//write back the delta v to the multi bodies, either as applied impulse (direct velocity change) 
	//or as applied force, so we can measure the joint reaction forces easier
	for (int i=0;i<numPoolConstraints;i++)
	{
		btMultiBodySolverConstraint& solverConstraint = m_multiBodyNormalContactConstraints[i];
		writeBackSolverBodyToMultiBody(solverConstraint,infoGlobal.m_timeStep);

		writeBackSolverBodyToMultiBody(m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex],infoGlobal.m_timeStep);

		if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS))
		{
			writeBackSolverBodyToMultiBody(m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex+1],infoGlobal.m_timeStep);
		}
	}


	for (int i=0;i<m_multiBodyNonContactConstraints.Count;i++)
	{
		btMultiBodySolverConstraint& solverConstraint = m_multiBodyNonContactConstraints[i];
		writeBackSolverBodyToMultiBody(solverConstraint,infoGlobal.m_timeStep);
	}

	
	if (infoGlobal.m_solverMode & SOLVER_USE_WARMSTARTING)
	{
		CProfileSample sample = new CProfileSample("warm starting write back");
		for (int j=0;j<numPoolConstraints;j++)
		{
			btMultiBodySolverConstraint& solverConstraint = m_multiBodyNormalContactConstraints[j];
			btManifoldPoint* pt = (btManifoldPoint*) solverConstraint.m_originalContactPoint;
			Debug.Assert(pt);
			pt.m_appliedImpulse = solverConstraint.m_appliedImpulse;
			pt.m_appliedImpulseLateral1 = m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex].m_appliedImpulse;
			
			//Console.WriteLine("pt.m_appliedImpulseLateral1 = %f\n", pt.m_appliedImpulseLateral1);
			if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS))
			{
				pt.m_appliedImpulseLateral2 = m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex+1].m_appliedImpulse;
			}
			//do a callback here?
		}
	}
#if 0
	//multibody joint feedback
	{
		CProfileSample sample = new CProfileSample("multi body joint feedback");
		for (int j=0;j<numPoolConstraints;j++)
		{
			btMultiBodySolverConstraint& solverConstraint = m_multiBodyNormalContactConstraints[j];
		
			//apply the joint feedback into all links of the btMultiBody
			//todo: double-check the signs of the applied impulse

			if(solverConstraint.m_multiBodyA && solverConstraint.m_multiBodyA.isMultiDof())
			{
				applyJointFeedback(m_data,solverConstraint, solverConstraint.m_jacAindex,solverConstraint.m_multiBodyA, solverConstraint.m_appliedImpulse*double(1./infoGlobal.m_timeStep));
			}
			if(solverConstraint.m_multiBodyB && solverConstraint.m_multiBodyB.isMultiDof())
			{
				applyJointFeedback(m_data,solverConstraint, solverConstraint.m_jacBindex,solverConstraint.m_multiBodyB,solverConstraint.m_appliedImpulse*double(-1./infoGlobal.m_timeStep));
			}
#if 0
			if (m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex].m_multiBodyA && m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex].m_multiBodyA.isMultiDof())
			{
				applyJointFeedback(m_data,m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex],
					m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex].m_jacAindex,
					m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex].m_multiBodyA,
					m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex].m_appliedImpulse*double(1./infoGlobal.m_timeStep));

			}
			if (m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex].m_multiBodyB && m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex].m_multiBodyB.isMultiDof())
			{
				applyJointFeedback(m_data,m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex],
					m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex].m_jacBindex,
					m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex].m_multiBodyB,
					m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex].m_appliedImpulse*double(-1./infoGlobal.m_timeStep));
			}
		
			if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS))
			{
				if (m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex+1].m_multiBodyA && m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex+1].m_multiBodyA.isMultiDof())
				{
					applyJointFeedback(m_data,m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex+1],
						m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex+1].m_jacAindex,
						m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex+1].m_multiBodyA,
						m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex+1].m_appliedImpulse*double(1./infoGlobal.m_timeStep));
				}

				if (m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex+1].m_multiBodyB && m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex+1].m_multiBodyB.isMultiDof())
				{
					applyJointFeedback(m_data,m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex+1],
						m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex+1].m_jacBindex,
						m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex+1].m_multiBodyB,
						m_multiBodyFrictionContactConstraints[solverConstraint.m_frictionIndex+1].m_appliedImpulse*double(-1./infoGlobal.m_timeStep));
				}
			}
#endif
		}
	
		for (int i=0;i<m_multiBodyNonContactConstraints.Count;i++)
		{
			btMultiBodySolverConstraint& solverConstraint = m_multiBodyNonContactConstraints[i];
			if(solverConstraint.m_multiBodyA && solverConstraint.m_multiBodyA.isMultiDof())
			{
				applyJointFeedback(m_data,solverConstraint, solverConstraint.m_jacAindex,solverConstraint.m_multiBodyA, solverConstraint.m_appliedImpulse*double(1./infoGlobal.m_timeStep));
			}
			if(solverConstraint.m_multiBodyB && solverConstraint.m_multiBodyB.isMultiDof())
			{
				applyJointFeedback(m_data,solverConstraint, solverConstraint.m_jacBindex,solverConstraint.m_multiBodyB,solverConstraint.m_appliedImpulse*double(1./infoGlobal.m_timeStep));
			}
		}
	}

	numPoolConstraints = m_multiBodyNonContactConstraints.Count;

#if 0
	//@todo: m_originalContactPoint is not initialized for btMultiBodySolverConstraint
	for (int i=0;i<numPoolConstraints;i++)
	{
		btMultiBodySolverConstraint& c = m_multiBodyNonContactConstraints[i];

		btTypedConstraint* constr = (btTypedConstraint*)c.m_originalContactPoint;
		btJointFeedback* fb = constr.getJointFeedback();
		if (fb)
		{
			fb.m_appliedForceBodyA += c.m_contactNormal1*c.m_appliedImpulse*constr.getRigidBodyA().getLinearFactor()/infoGlobal.m_timeStep;
			fb.m_appliedForceBodyB += c.m_contactNormal2*c.m_appliedImpulse*constr.getRigidBodyB().getLinearFactor()/infoGlobal.m_timeStep;
			fb.m_appliedTorqueBodyA += c.m_relpos1CrossNormal* constr.getRigidBodyA().getAngularFactor()*c.m_appliedImpulse/infoGlobal.m_timeStep;
			fb.m_appliedTorqueBodyB += c.m_relpos2CrossNormal* constr.getRigidBodyB().getAngularFactor()*c.m_appliedImpulse/infoGlobal.m_timeStep; /*RGM ???? */
			
		}

		constr.internalSetAppliedImpulse(c.m_appliedImpulse);
		if (btFabs(c.m_appliedImpulse)>=constr.getBreakingImpulseThreshold())
		{
			constr.setEnabled(false);
		}

	}
#endif 
#endif

	return btSequentialImpulseConstraintSolver::solveGroupCacheFriendlyFinish(bodies,numBodies,infoGlobal);
}


void  btMultiBodyConstraintSolver::solveMultiBodyGroup(btCollisionObject bodies,int numBodies,btPersistentManifold[] manifold,int numManifolds,btTypedConstraint[] constraints,int numConstraints,btMultiBodyConstraint[] multiBodyConstraints, int numMultiBodyConstraints, btContactSolverInfo& info, btIDebugDraw* debugDrawer,btDispatcher* dispatcher)
{
	//Console.WriteLine("solveMultiBodyGroup start\n");
	m_tmpMultiBodyConstraints = multiBodyConstraints;
	m_tmpNumMultiBodyConstraints = numMultiBodyConstraints;
	
	btSequentialImpulseConstraintSolver::solveGroup(bodies,numBodies,manifold,numManifolds,constraints,numConstraints,info,debugDrawer,dispatcher);

	m_tmpMultiBodyConstraints = 0;
	m_tmpNumMultiBodyConstraints = 0;
	

}
