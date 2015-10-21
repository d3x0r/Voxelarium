/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006, 2007 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btGeneric6DofSpringConstraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btTransformUtil.h"


btGeneric6DofSpringConstraint::btGeneric6DofSpringConstraint(btRigidBody rbA, btRigidBody rbB, ref btTransform frameInA, ref btTransform frameInB ,bool useLinearReferenceFrameA)
	: btGeneric6DofConstraint(rbA, rbB, frameInA, frameInB, useLinearReferenceFrameA)
{
    init();
}


btGeneric6DofSpringConstraint::btGeneric6DofSpringConstraint(btRigidBody rbB, ref btTransform frameInB, bool useLinearReferenceFrameB)
        : btGeneric6DofConstraint(rbB, frameInB, useLinearReferenceFrameB)
{
    init();
}


void btGeneric6DofSpringConstraint::init()
{
	m_objectType = D6_SPRING_CONSTRAINT_TYPE;

	for(int i = 0; i < 6; i++)
	{
		m_springEnabled[i] = false;
		m_equilibriumPoint[i] = (double)(0);
		m_springStiffness[i] = (double)(0);
		m_springDamping[i] = (double)(1);
	}
}


void btGeneric6DofSpringConstraint::enableSpring(int index, bool onOff)
{
	Debug.Assert((index >= 0) && (index < 6));
	m_springEnabled[index] = onOff;
	if(index < 3)
	{
		m_linearLimits.m_enableMotor[index] = onOff;
	}
	else
	{
		m_angularLimits[index - 3].m_enableMotor = onOff;
	}
}



void btGeneric6DofSpringConstraint::setStiffness(int index, double stiffness)
{
	Debug.Assert((index >= 0) && (index < 6));
	m_springStiffness[index] = stiffness;
}


void btGeneric6DofSpringConstraint::setDamping(int index, double damping)
{
	Debug.Assert((index >= 0) && (index < 6));
	m_springDamping[index] = damping;
}


void btGeneric6DofSpringConstraint::setEquilibriumPoint()
{
	calculateTransforms();
	int i;

	for( i = 0; i < 3; i++)
	{
		m_equilibriumPoint[i] = m_calculatedLinearDiff[i];
	}
	for(i = 0; i < 3; i++)
	{
		m_equilibriumPoint[i + 3] = m_calculatedAxisAngleDiff[i];
	}
}



void btGeneric6DofSpringConstraint::setEquilibriumPoint(int index)
{
	Debug.Assert((index >= 0) && (index < 6));
	calculateTransforms();
	if(index < 3)
	{
		m_equilibriumPoint[index] = m_calculatedLinearDiff[index];
	}
	else
	{
		m_equilibriumPoint[index] = m_calculatedAxisAngleDiff[index - 3];
	}
}

void btGeneric6DofSpringConstraint::setEquilibriumPoint(int index, double val)
{
	Debug.Assert((index >= 0) && (index < 6));
	m_equilibriumPoint[index] = val;
}


void btGeneric6DofSpringConstraint::internalUpdateSprings(btConstraintInfo2* info)
{
	// it is assumed that calculateTransforms() have been called before this call
	int i;
	//btVector3 relVel = m_rbB.getLinearVelocity() - m_rbA.getLinearVelocity();
	for(i = 0; i < 3; i++)
	{
		if(m_springEnabled[i])
		{
			// get current position of constraint
			double currPos = m_calculatedLinearDiff[i];
			// calculate difference
			double delta = currPos - m_equilibriumPoint[i];
			// spring force is (delta * m_stiffness) according to Hooke's Law
			double force = delta * m_springStiffness[i];
			double velFactor = info.fps * m_springDamping[i] / (double)(info.m_numIterations);
			m_linearLimits.m_targetVelocity[i] =  velFactor * force;
			m_linearLimits.m_maxMotorForce[i] =  btFabs(force) / info.fps;
		}
	}
	for(i = 0; i < 3; i++)
	{
		if(m_springEnabled[i + 3])
		{
			// get current position of constraint
			double currPos = m_calculatedAxisAngleDiff[i];
			// calculate difference
			double delta = currPos - m_equilibriumPoint[i+3];
			// spring force is (-delta * m_stiffness) according to Hooke's Law
			double force = -delta * m_springStiffness[i+3];
			double velFactor = info.fps * m_springDamping[i+3] / (double)(info.m_numIterations);
			m_angularLimits[i].m_targetVelocity = velFactor * force;
			m_angularLimits[i].m_maxMotorForce = btFabs(force) / info.fps;
		}
	}
}


void btGeneric6DofSpringConstraint::getInfo2(btConstraintInfo2* info)
{
	// this will be called by constraint solver at the constraint setup stage
	// set current motor parameters
	internalUpdateSprings(info);
	// do the rest of job for constraint setup
	btGeneric6DofConstraint::getInfo2(info);
}


void btGeneric6DofSpringConstraint::setAxis(ref btVector3 axis1,ref btVector3 axis2)
{
	btVector3 zAxis = axis1.normalized();
	btVector3 yAxis = axis2.normalized();
	btVector3 xAxis = yAxis.cross(zAxis); // we want right coordinate system

	btTransform frameInW;
	frameInW.setIdentity();
	frameInW.getBasis().setValue(	xAxis[0], yAxis[0], zAxis[0],	
                                xAxis[1], yAxis[1], zAxis[1],
                                xAxis[2], yAxis[2], zAxis[2]);

	// now get constraint frame in local coordinate systems
	m_frameInA = m_rbA.getCenterOfMassTransform().inverse() * frameInW;
	m_frameInB = m_rbB.getCenterOfMassTransform().inverse() * frameInW;

  calculateTransforms();
}



