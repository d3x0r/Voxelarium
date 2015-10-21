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

#if ! BT_JACOBIAN_ENTRY_H
#define BT_JACOBIAN_ENTRY_H

#include "LinearMath/btMatrix3x3.h"


//notes:
// Another memory optimization would be to store m_1MinvJt in the remaining 3 w components
// which makes the btJacobianEntry memory layout 16 bytes
// if you only are interested in angular part, just feed massInvA and massInvB zero

/// Jacobian entry is an abstraction that allows to describe constraints
/// it can be used in combination with a constraint solver
/// Can be used to relate the effect of an impulse to the constraint error
internal class btJacobianEntry
{
public:
	btJacobianEntry() {};
	//constraint between two different rigidbodies
	btJacobianEntry(
		btMatrix3x3 world2A,
		btMatrix3x3& world2B,
		ref btVector3 rel_pos1,ref btVector3 rel_pos2,
		ref btVector3 jointAxis,
		ref btVector3 inertiaInvA, 
		double massInvA,
		ref btVector3 inertiaInvB,
		double massInvB)
		:m_linearJointAxis(jointAxis)
	{
		m_aJ = world2A*(rel_pos1.cross(m_linearJointAxis));
		m_bJ = world2B*(rel_pos2.cross(-m_linearJointAxis));
		m_0MinvJt	= inertiaInvA * m_aJ;
		m_1MinvJt = inertiaInvB * m_bJ;
		m_Adiag = massInvA + m_0MinvJt.dot(m_aJ) + massInvB + m_1MinvJt.dot(m_bJ);

		Debug.Assert(m_Adiag > (double)(0.0));
	}

	//angular constraint between two different rigidbodies
	btJacobianEntry(ref btVector3 jointAxis,
		btMatrix3x3& world2A,
		btMatrix3x3& world2B,
		ref btVector3 inertiaInvA,
		ref btVector3 inertiaInvB)
		:m_linearJointAxis(btVector3(btScalar.BT_ZERO,btScalar.BT_ZERO,btScalar.BT_ZERO))
	{
		m_aJ= world2A*jointAxis;
		m_bJ = world2B*-jointAxis;
		m_0MinvJt	= inertiaInvA * m_aJ;
		m_1MinvJt = inertiaInvB * m_bJ;
		m_Adiag =  m_0MinvJt.dot(m_aJ) + m_1MinvJt.dot(m_bJ);

		Debug.Assert(m_Adiag > (double)(0.0));
	}

	//angular constraint between two different rigidbodies
	btJacobianEntry(ref btVector3 axisInA,
		ref btVector3 axisInB,
		ref btVector3 inertiaInvA,
		ref btVector3 inertiaInvB)
		: m_linearJointAxis(btVector3(btScalar.BT_ZERO,btScalar.BT_ZERO,btScalar.BT_ZERO))
		, m_aJ(axisInA)
		, m_bJ(-axisInB)
	{
		m_0MinvJt	= inertiaInvA * m_aJ;
		m_1MinvJt = inertiaInvB * m_bJ;
		m_Adiag =  m_0MinvJt.dot(m_aJ) + m_1MinvJt.dot(m_bJ);

		Debug.Assert(m_Adiag > (double)(0.0));
	}

	//constraint on one rigidbody
	btJacobianEntry(
		btMatrix3x3& world2A,
		ref btVector3 rel_pos1,ref btVector3 rel_pos2,
		ref btVector3 jointAxis,
		ref btVector3 inertiaInvA, 
		double massInvA)
		:m_linearJointAxis(jointAxis)
	{
		m_aJ= world2A*(rel_pos1.cross(jointAxis));
		m_bJ = world2A*(rel_pos2.cross(-jointAxis));
		m_0MinvJt	= inertiaInvA * m_aJ;
		m_1MinvJt = btVector3(btScalar.BT_ZERO,btScalar.BT_ZERO,btScalar.BT_ZERO);
		m_Adiag = massInvA + m_0MinvJt.dot(m_aJ);

		Debug.Assert(m_Adiag > (double)(0.0));
	}

	double	getDiagonal() string  return m_Adiag; }

	// for two constraints on the same rigidbody (for example vehicle friction)
	double	getNonDiagonal(btJacobianEntry& jacB, double massInvA)
	{
		btJacobianEntry& jacA = *this;
		double lin = massInvA * jacA.m_linearJointAxis.dot(jacB.m_linearJointAxis);
		double ang = jacA.m_0MinvJt.dot(jacB.m_aJ);
		return lin + ang;
	}

	

	// for two constraints on sharing two same rigidbodies (for example two contact points between two rigidbodies)
	double	getNonDiagonal(btJacobianEntry& jacB,double massInvA,double massInvB)
	{
		btJacobianEntry& jacA = *this;
		btVector3 lin = jacA.m_linearJointAxis * jacB.m_linearJointAxis;
		btVector3 ang0 = jacA.m_0MinvJt * jacB.m_aJ;
		btVector3 ang1 = jacA.m_1MinvJt * jacB.m_bJ;
		btVector3 lin0 = massInvA * lin ;
		btVector3 lin1 = massInvB * lin;
		btVector3 sum = ang0+ang1+lin0+lin1;
		return sum+sum[1]+sum[2];
	}

	double getRelativeVelocity(ref btVector3 linvelA,ref btVector3 angvelA,ref btVector3 linvelB,ref btVector3 angvelB)
	{
		btVector3 linrel = linvelA - linvelB;
		btVector3 angvela  = angvelA * m_aJ;
		btVector3 angvelb  = angvelB * m_bJ;
		linrel *= m_linearJointAxis;
		angvela += angvelb;
		angvela += linrel;
		double rel_vel2 = angvela[0]+angvela[1]+angvela[2];
		return rel_vel2 + SIMD_EPSILON;
	}
//private:

	btVector3	m_linearJointAxis;
	btVector3	m_aJ;
	btVector3	m_bJ;
	btVector3	m_0MinvJt;
	btVector3	m_1MinvJt;
	//Optimization: can be stored in the w/last component of one of the vectors
	double	m_Adiag;

};

#endif //BT_JACOBIAN_ENTRY_H
