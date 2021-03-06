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

///This file was written by Erwin Coumans

#include "btMultiBodyJointMotor.h"
#include "btMultiBody.h"
#include "btMultiBodyLinkCollider.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"


btMultiBodyJointMotor::btMultiBodyJointMotor(btMultiBody* body, int link, double desiredVelocity, double maxMotorImpulse)
	:btMultiBodyConstraint(body,body,link,body.getLink(link).m_parent,1,true),
	m_desiredVelocity(desiredVelocity)
{

	m_maxAppliedImpulse = maxMotorImpulse;
	// the data.m_jacobians never change, so may as well
    // initialize them here


}

void btMultiBodyJointMotor::finalizeMultiDof()
{
	allocateJacobiansMultiDof();
	// note: we rely on the fact that data.m_jacobians are
	// always initialized to zero by the Constraint ctor
	int linkDoF = 0;
	uint offset = 6 + (m_bodyA.isMultiDof() ? m_bodyA.getLink(m_linkA).m_dofOffset + linkDoF : m_linkA);

	// row 0: the lower bound
	// row 0: the lower bound
	jacobianA(0)[offset] = 1;

	m_numDofsFinalized = m_jacSizeBoth;
}

btMultiBodyJointMotor::btMultiBodyJointMotor(btMultiBody* body, int link, int linkDoF, double desiredVelocity, double maxMotorImpulse)
	//:btMultiBodyConstraint(body,0,link,-1,1,true),
	:btMultiBodyConstraint(body,body,link,body.getLink(link).m_parent,1,true),
	m_desiredVelocity(desiredVelocity)
{
	Debug.Assert(linkDoF < body.getLink(link).m_dofCount);

	m_maxAppliedImpulse = maxMotorImpulse;

}
btMultiBodyJointMotor::~btMultiBodyJointMotor()
{
}

int btMultiBodyJointMotor::getIslandIdA()
{
	btMultiBodyLinkCollider* col = m_bodyA.getBaseCollider();
	if (col)
		return col.getIslandTag();
	for (int i=0;i<m_bodyA.getNumLinks();i++)
	{
		if (m_bodyA.getLink(i).m_collider)
			return m_bodyA.getLink(i).m_collider.getIslandTag();
	}
	return -1;
}

int btMultiBodyJointMotor::getIslandIdB()
{
	btMultiBodyLinkCollider* col = m_bodyB.getBaseCollider();
	if (col)
		return col.getIslandTag();

	for (int i=0;i<m_bodyB.getNumLinks();i++)
	{
		col = m_bodyB.getLink(i).m_collider;
		if (col)
			return col.getIslandTag();
	}
	return -1;
}


void btMultiBodyJointMotor::createConstraintRows(btMultiBodyConstraintArray constraintRows,
		btMultiBodyJacobianData& data,
		btContactSolverInfo& infoGlobal)
{
    // only positions need to be updated -- data.m_jacobians and force
    // directions were set in the ctor and never change.
	
	if (m_numDofsFinalized != m_jacSizeBoth)
	{
        finalizeMultiDof();
	}

	//don't crash
	if (m_numDofsFinalized != m_jacSizeBoth)
		return;

	double posError = 0;
	btVector3 dummy(0, 0, 0);

	for (int row=0;row<getNumRows();row++)
	{
		btMultiBodySolverConstraint& constraintRow = constraintRows.expandNonInitializing();


		fillMultiBodyConstraint(constraintRow,data,jacobianA(row),jacobianB(row),dummy,dummy,dummy,posError,infoGlobal,-m_maxAppliedImpulse,m_maxAppliedImpulse,1,false,m_desiredVelocity);
		constraintRow.m_orgConstraint = this;
		constraintRow.m_orgDofIndex = row;
		if (m_bodyA.isMultiDof())
		{
			//expect either prismatic or revolute joint type for now
			Debug.Assert((m_bodyA.getLink(m_linkA).m_jointType == btMultibodyLink::eRevolute)||(m_bodyA.getLink(m_linkA).m_jointType == btMultibodyLink::ePrismatic));
			switch (m_bodyA.getLink(m_linkA).m_jointType)
			{
				case btMultibodyLink::eRevolute:
				{
					constraintRow.m_contactNormal1.setZero();
					constraintRow.m_contactNormal2.setZero();
					btVector3 revoluteAxisInWorld = quatRotate(m_bodyA.getLink(m_linkA).m_cachedWorldTransform.getRotation(),m_bodyA.getLink(m_linkA).m_axes.m_topVec);
					constraintRow.m_relpos1CrossNormal=revoluteAxisInWorld;
					constraintRow.m_relpos2CrossNormal=-revoluteAxisInWorld;
					
					break;
				}
				case btMultibodyLink::ePrismatic:
				{
					btVector3 prismaticAxisInWorld = quatRotate(m_bodyA.getLink(m_linkA).m_cachedWorldTransform.getRotation(),m_bodyA.getLink(m_linkA).m_axes[0].m_bottomVec);
					constraintRow.m_contactNormal1=prismaticAxisInWorld;
					constraintRow.m_contactNormal2=-prismaticAxisInWorld;
					constraintRow.m_relpos1CrossNormal.setZero();
					constraintRow.m_relpos2CrossNormal.setZero();
					
					break;
				}
				default:
				{
					Debug.Assert(false);
				}
			};
			
		}

	}

}

